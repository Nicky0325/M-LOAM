#!/usr/bin/env python3
"""Raw IMU/GNSS-aided, fixed-translation rotation calibration for planar rigs.

The estimator deliberately solves a smaller problem than M-LOAM's original
online 6-DoF calibration. Vehicle-frame LiDAR translations are treated as
known constants. Raw dual-antenna GNSS and IMU measurements supply relative
rig motion, point-to-plane ICP measures the corresponding relative motion of
each LiDAR, and a robust hand-eye solve estimates only the three rotational
degrees of freedom::

    B_ij = X^-1 A_ij X

Here ``A_ij`` is vehicle motion, ``B_ij`` is LiDAR motion, and ``X`` is the
vehicle_T_lidar transform whose translation parameter is never changed.

The implementation intentionally does not read ``vehicle_pose``, ``ins.txt``,
or ``wheel.txt``. GNSS positions are interpolated with their measured ENU
velocities. Gyro-z is integrated and anchored to dual-antenna GNSS heading;
stationary accelerometer samples provide the small fixed roll/pitch tilt. The
unknown planar GNSS-antenna lever arm is estimated as a shared nuisance
parameter while all LiDAR translations remain fixed. The resulting ENU
trajectory and accepted extrinsics can build a colored global map.

In ``prior-free`` mode, identity-seeded ICP estimates each LiDAR's motion,
signed planar motion axes determine two rotation degrees of freedom, and known
translations provide a global Procrustes yaw. Manifest rotations are not
parsed unless post-run scoring is explicitly requested. An optional
deterministic perturbation mode separately validates the local recovery basin
without exposing trusted rotations to optimizer residuals.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
import open3d as o3d
import yaml
from scipy.interpolate import CubicHermiteSpline
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class LidarDefinition:
    name: str
    directory: Path
    translation: np.ndarray
    trusted_rotation: np.ndarray | None
    color: tuple[float, float, float]


@dataclass(frozen=True)
class PairCandidate:
    first: int
    second: int
    vehicle_motion: np.ndarray
    translation_m: float
    rotation_deg: float


@dataclass
class PairConstraint:
    candidate: PairCandidate
    lidar_motion: np.ndarray
    fitness: float
    inlier_rmse_m: float
    icp_translation_update_m: float
    icp_rotation_update_deg: float
    accepted: bool
    split: str = "rejected"
    registration_mode: str = "extrinsic_seeded"
    motion_angle_error_deg: float = math.inf
    axis_error_deg: float = math.inf
    rejection_reason: str = ""


@dataclass
class RotationCalibration:
    name: str
    initial_rotation: np.ndarray
    optimized_rotation: np.ndarray
    fixed_translation: np.ndarray
    constraints: list[PairConstraint] = field(default_factory=list)
    accepted: bool = False
    reason: str = ""
    refinement_selected: bool = True
    update_deg: float = 0.0
    hessian_eigenvalues: list[float] = field(default_factory=list)
    hessian_condition_number: float = math.inf
    one_sigma_deg: list[float] = field(default_factory=list)
    train_initial: dict[str, float] = field(default_factory=dict)
    train_final: dict[str, float] = field(default_factory=dict)
    heldout_initial: dict[str, float] = field(default_factory=dict)
    heldout_final: dict[str, float] = field(default_factory=dict)
    trusted_initial_error_deg: float | None = None
    trusted_final_error_deg: float | None = None


@dataclass
class NavigationLeverArmEstimate:
    value: np.ndarray
    accepted: bool = False
    reason: str = ""
    update_m: float = 0.0
    hessian_eigenvalues: list[float] = field(default_factory=list)
    hessian_condition_number: float = math.inf
    heldout_initial_translation_rmse_m: float = math.inf
    heldout_final_translation_rmse_m: float = math.inf


@dataclass
class NavigationTrajectory:
    timestamps: np.ndarray
    antenna_positions: np.ndarray
    vehicle_quaternions: np.ndarray
    metrics: dict[str, Any]

    def poses(
        self,
        timestamps: np.ndarray,
        gnss_lever_arm_vehicle: np.ndarray | None = None,
    ) -> np.ndarray:
        if timestamps.size == 0:
            return np.empty((0, 4, 4))
        if timestamps.min() < self.timestamps[0] or timestamps.max() > self.timestamps[-1]:
            raise ValueError("navigation interpolation requested outside pose coverage")
        antenna_positions = np.column_stack(
            [
                np.interp(timestamps, self.timestamps, self.antenna_positions[:, axis])
                for axis in range(3)
            ]
        )
        interpolated_rotations = Slerp(
            self.timestamps, Rotation.from_quat(self.vehicle_quaternions)
        )(timestamps).as_matrix()
        lever_arm = (
            np.zeros(3)
            if gnss_lever_arm_vehicle is None
            else np.asarray(gnss_lever_arm_vehicle, dtype=float)
        )
        vehicle_positions = antenna_positions - np.einsum(
            "nij,j->ni", interpolated_rotations, lever_arm
        )
        return np.asarray(
            [make_transform(rotation, translation)
             for rotation, translation in zip(interpolated_rotations, vehicle_positions)]
        )


@dataclass
class PriorFreeLidarInitialization:
    name: str
    rotation: np.ndarray
    vehicle_axis: np.ndarray
    lidar_axis: np.ndarray
    yaw_about_vehicle_axis_deg: float = 0.0
    axis_samples: int = 0
    axis_inliers: int = 0
    axis_p95_deg: float = math.inf
    train_metrics: dict[str, float] = field(default_factory=dict)
    heldout_metrics: dict[str, float] = field(default_factory=dict)
    accepted: bool = False
    reason: str = ""


@dataclass
class PriorFreeInitialization:
    lidars: dict[str, PriorFreeLidarInitialization]
    gnss_lever_arm: np.ndarray
    accepted: bool = False
    reason: str = ""
    starts_attempted: int = 0
    selected_cost: float = math.inf
    hessian_eigenvalues: list[float] = field(default_factory=list)
    hessian_condition_number: float = math.inf


def arguments(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--imu-file", type=Path)
    parser.add_argument("--gnss-dir", type=Path)
    parser.add_argument(
        "--gnss-heading-to-vehicle-yaw-deg",
        type=float,
        help=(
            "known yaw from the dual-antenna heading frame to vehicle x; "
            "otherwise estimate it from straight GNSS velocity segments"
        ),
    )
    parser.add_argument(
        "--inject-rotation-error-deg",
        type=float,
        default=0.0,
        help="deterministically perturb every input rotation for validation",
    )
    parser.add_argument(
        "--rotation-initialization",
        choices=("manifest", "prior-free"),
        default="manifest",
        help=(
            "manifest uses the supplied mounting rotation as the ICP seed; "
            "prior-free estimates independent LiDAR motion from identity and "
            "recovers rotation from planar axes plus known translations"
        ),
    )
    parser.add_argument(
        "--score-against-manifest",
        action="store_true",
        help=(
            "after optimization only, compare against manifest rotations; "
            "this never contributes an initializer or residual"
        ),
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--pair-count", type=int, default=40)
    parser.add_argument("--pair-frame-gap", type=int, default=15)
    parser.add_argument("--pair-candidate-stride", type=int, default=5)
    parser.add_argument("--calibration-voxel-size", type=float, default=0.45)
    parser.add_argument("--prior-free-voxel-size", type=float, default=0.70)
    parser.add_argument(
        "--prior-free-coarse-distance", type=float, default=5.0
    )
    parser.add_argument(
        "--prior-free-fallback-distance", type=float, default=8.0
    )
    parser.add_argument(
        "--prior-free-motion-angle-tolerance-deg", type=float, default=2.5
    )
    parser.add_argument(
        "--prior-free-axis-inlier-deg", type=float, default=4.0
    )
    parser.add_argument("--minimum-range", type=float, default=2.0)
    parser.add_argument("--maximum-range", type=float, default=80.0)
    parser.add_argument("--minimum-constraints", type=int, default=12)
    parser.add_argument("--rotation-update-bound-deg", type=float, default=15.0)
    parser.add_argument("--gnss-lever-arm-bound-m", type=float, default=10.0)
    parser.add_argument("--lever-arm-outer-iterations", type=int, default=15)
    parser.add_argument("--map-stride", type=int, default=10)
    parser.add_argument("--map-voxel-size", type=float, default=0.35)
    parser.add_argument("--skip-map", action="store_true")
    parser.add_argument(
        "--include",
        help="comma-separated LiDAR names; defaults to every enabled sensor",
    )
    return parser.parse_args(argv)


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def inverse(transform: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def motion_at_vehicle_origin(
    antenna_motion: np.ndarray, gnss_lever_arm_vehicle: np.ndarray | None
) -> np.ndarray:
    """Shift a relative motion from the GNSS antenna to the vehicle origin."""
    if gnss_lever_arm_vehicle is None:
        return antenna_motion
    vehicle_t_antenna = make_transform(
        np.eye(3), np.asarray(gnss_lever_arm_vehicle, dtype=float)
    )
    return vehicle_t_antenna @ antenna_motion @ inverse(vehicle_t_antenna)


def angular_distance_degrees(first: np.ndarray, second: np.ndarray) -> float:
    return float(
        np.rad2deg(Rotation.from_matrix(first.T @ second).magnitude())
    )


def block_vector(text: str, block_name: str) -> np.ndarray:
    match = re.search(rf"\b{re.escape(block_name)}\s*\{{([^}}]*)\}}", text, re.S)
    if not match:
        raise ValueError(f"missing {block_name} block")
    values = []
    for axis in "xyz":
        field = re.search(rf"\b{axis}\s*:\s*([-+0-9.eE]+)", match.group(1))
        if not field:
            raise ValueError(f"missing {block_name}.{axis}")
        values.append(float(field.group(1)))
    return np.asarray(values, dtype=float)


def prototxt_calibration(path: Path) -> tuple[np.ndarray, np.ndarray]:
    text = path.read_text()
    translation = block_vector(text, "translation")
    rpy_deg = block_vector(text, "rotation_rpy_deg")
    return Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix(), translation


def resolve_path(base: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else (base / path).resolve()


def load_lidars(
    manifest_path: Path,
    document: dict[str, Any],
    include: set[str] | None,
    load_manifest_rotations: bool = True,
) -> list[LidarDefinition]:
    dataset_root = resolve_path(manifest_path.parent, str(document["dataset_root"]))
    result: list[LidarDefinition] = []
    for node in document.get("lidars", []):
        if not node.get("enabled", True):
            continue
        name = str(node["name"])
        if include is not None and name not in include:
            continue
        if "vehicle_T_lidar" in node:
            transform = node["vehicle_T_lidar"]
            translation = np.asarray(transform["translation"], dtype=float)
            rotation = None
            if load_manifest_rotations:
                if "rpy_deg" not in transform:
                    raise ValueError(
                        f"{name} has no manifest rotation for initialization/scoring"
                    )
                rotation = Rotation.from_euler(
                    "xyz", np.asarray(transform["rpy_deg"], dtype=float), degrees=True
                ).as_matrix()
        elif "extrinsic_prototxt" in node:
            calibration_path = resolve_path(
                manifest_path.parent, str(node["extrinsic_prototxt"])
            )
            if load_manifest_rotations:
                rotation, translation = prototxt_calibration(calibration_path)
            else:
                rotation = None
                translation = block_vector(calibration_path.read_text(), "translation")
        else:
            raise ValueError(f"{name} has no vehicle-frame extrinsic")
        color = tuple(float(value) / 255.0 for value in node.get("color", [255, 255, 255]))
        result.append(
            LidarDefinition(
                name=name,
                directory=dataset_root / str(node["directory"]),
                translation=translation,
                trusted_rotation=rotation,
                color=color,
            )
        )
    if not result:
        raise ValueError("no enabled LiDARs selected")
    if include is not None:
        missing = include - {lidar.name for lidar in result}
        if missing:
            raise ValueError(f"unknown or disabled LiDARs: {sorted(missing)}")
    return result


def scalar_field(text: str, name: str) -> float:
    match = re.search(rf"\b{re.escape(name)}\s*:\s*([-+0-9.eE]+)", text)
    if not match:
        raise ValueError(f"missing {name}")
    return float(match.group(1))


def word_field(text: str, name: str, default: str = "UNKNOWN") -> str:
    match = re.search(rf"\b{re.escape(name)}\s*:\s*(\S+)", text)
    return match.group(1) if match else default


def lla_to_enu(lla: np.ndarray) -> np.ndarray:
    """Convert geodetic coordinates to a local WGS84 ENU frame."""
    coordinates = np.asarray(lla, dtype=float).copy()
    if np.max(np.abs(coordinates[:, 0])) > math.pi / 2.0 + 0.1:
        coordinates[:, :2] = np.deg2rad(coordinates[:, :2])
    latitude = coordinates[:, 0]
    longitude = coordinates[:, 1]
    altitude = coordinates[:, 2]
    semi_major = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    prime_vertical = semi_major / np.sqrt(
        1.0 - eccentricity_squared * np.square(np.sin(latitude))
    )
    ecef = np.column_stack(
        (
            (prime_vertical + altitude) * np.cos(latitude) * np.cos(longitude),
            (prime_vertical + altitude) * np.cos(latitude) * np.sin(longitude),
            (prime_vertical * (1.0 - eccentricity_squared) + altitude)
            * np.sin(latitude),
        )
    )
    latitude0 = float(latitude[0])
    longitude0 = float(longitude[0])
    ecef_to_enu = np.asarray(
        [
            [-math.sin(longitude0), math.cos(longitude0), 0.0],
            [
                -math.sin(latitude0) * math.cos(longitude0),
                -math.sin(latitude0) * math.sin(longitude0),
                math.cos(latitude0),
            ],
            [
                math.cos(latitude0) * math.cos(longitude0),
                math.cos(latitude0) * math.sin(longitude0),
                math.sin(latitude0),
            ],
        ]
    )
    return (ecef_to_enu @ (ecef - ecef[0]).T).T


def circular_mean(values: np.ndarray) -> float:
    return float(np.angle(np.mean(np.exp(1j * values))))


def wrapped_angle(values: np.ndarray) -> np.ndarray:
    return np.angle(np.exp(1j * values))


def load_raw_navigation(
    imu_file: Path,
    gnss_dir: Path,
    heading_to_vehicle_yaw_deg: float | None,
) -> NavigationTrajectory:
    """Build an ENU pose stream using only raw IMU and dual-antenna GNSS."""
    imu = np.genfromtxt(imu_file, delimiter=",", skip_header=1)
    imu = np.atleast_2d(imu)
    if imu.shape[1] < 7:
        raise ValueError(f"IMU file has fewer than seven columns: {imu_file}")
    imu = imu[np.all(np.isfinite(imu[:, :7]), axis=1), :7]
    imu = imu[np.argsort(imu[:, 0])]
    imu = imu[np.concatenate(([True], np.diff(imu[:, 0]) > 0.0))]
    if imu.shape[0] < 100:
        raise ValueError(f"need at least 100 finite IMU records; got {imu.shape[0]}")

    records: list[dict[str, Any]] = []
    for path in sorted(gnss_dir.glob("*.prototxt")):
        text = path.read_text()
        records.append(
            {
                "position_time": scalar_field(text, "pos_time_gps") * 1.0e-9,
                "heading_time": scalar_field(text, "heading_time_gps") * 1.0e-9,
                "lla": block_vector(text, "lla"),
                "velocity": block_vector(text, "velocity_enu"),
                "velocity_std": block_vector(text, "velocity_std"),
                "yaw": block_vector(text, "attitude_rpy")[2],
                "yaw_std": block_vector(text, "attitude_std")[2],
                "position_status": word_field(text, "pos_status"),
                "heading_status": word_field(text, "heading_status"),
                "heading_usable": word_field(text, "heading_usable", "false")
                == "true",
            }
        )
    if len(records) < 20:
        raise ValueError(f"need at least 20 GNSS records in {gnss_dir}; got {len(records)}")

    position_records = [
        record
        for record in records
        if record["position_status"] != "INVALID"
        and np.all(np.isfinite(record["lla"]))
        and np.all(np.isfinite(record["velocity"]))
    ]
    heading_records = [
        record
        for record in records
        if record["heading_usable"]
        and record["heading_status"] != "INVALID"
        and math.isfinite(record["yaw"])
    ]
    if len(position_records) < 20 or len(heading_records) < 20:
        raise ValueError(
            "raw GNSS lacks at least 20 usable position and dual-heading records"
        )

    position_records.sort(key=lambda item: item["position_time"])
    heading_records.sort(key=lambda item: item["heading_time"])
    position_times = np.asarray(
        [record["position_time"] for record in position_records]
    )
    position_unique = np.concatenate(([True], np.diff(position_times) > 0.0))
    position_times = position_times[position_unique]
    lla = np.asarray([record["lla"] for record in position_records])[position_unique]
    velocities = np.asarray(
        [record["velocity"] for record in position_records]
    )[position_unique]
    velocity_std = np.asarray(
        [record["velocity_std"] for record in position_records]
    )[position_unique]
    antenna_positions = lla_to_enu(lla)

    heading_times = np.asarray(
        [record["heading_time"] for record in heading_records]
    )
    heading_unique = np.concatenate(([True], np.diff(heading_times) > 0.0))
    heading_times = heading_times[heading_unique]
    heading_yaw = np.unwrap(
        np.asarray([record["yaw"] for record in heading_records])[heading_unique]
    )
    heading_std = np.asarray(
        [record["yaw_std"] for record in heading_records]
    )[heading_unique]
    if np.max(np.diff(position_times)) > 1.0:
        raise ValueError("GNSS position gap exceeds 1.0 second")
    if np.max(np.diff(heading_times)) > 1.0:
        raise ValueError("GNSS heading gap exceeds 1.0 second")

    imu_times = imu[:, 0]
    accelerometer = imu[:, 1:4]
    gyroscope = imu[:, 4:7]
    speed = np.linalg.norm(velocities[:, :2], axis=1)
    speed_at_imu = np.interp(
        imu_times, position_times, speed, left=math.inf, right=math.inf
    )
    acceleration_norm = np.linalg.norm(accelerometer, axis=1)
    stationary = (
        (speed_at_imu < 0.10)
        & (acceleration_norm > 8.0)
        & (acceleration_norm < 11.5)
    )
    if np.count_nonzero(stationary) < 50:
        raise ValueError(
            "need at least 50 stationary IMU samples to initialize gravity and gyro bias"
        )
    gyro_bias = np.median(gyroscope[stationary], axis=0)
    gravity_body = np.median(accelerometer[stationary], axis=0)
    fixed_roll = math.atan2(gravity_body[1], gravity_body[2])
    fixed_pitch = math.atan2(
        -gravity_body[0], math.hypot(gravity_body[1], gravity_body[2])
    )

    corrected_gyro_z = gyroscope[:, 2] - gyro_bias[2]
    integrated_yaw = np.zeros(imu_times.size)
    integrated_yaw[1:] = np.cumsum(
        0.5
        * (corrected_gyro_z[:-1] + corrected_gyro_z[1:])
        * np.diff(imu_times)
    )
    heading_overlap = (
        (heading_times >= imu_times[0]) & (heading_times <= imu_times[-1])
    )
    if np.count_nonzero(heading_overlap) < 20:
        raise ValueError("fewer than 20 GNSS headings overlap the IMU")
    fit_times = heading_times[heading_overlap]
    fit_yaw = heading_yaw[heading_overlap]
    integrated_at_heading = np.interp(fit_times, imu_times, integrated_yaw)
    fit_x = fit_times - imu_times[0]

    def heading_fit_residual(parameters: np.ndarray) -> np.ndarray:
        return integrated_at_heading + parameters[0] + parameters[1] * fit_x - fit_yaw

    heading_fit = least_squares(
        heading_fit_residual,
        np.asarray([float(np.median(fit_yaw - integrated_at_heading)), 0.0]),
        loss="cauchy",
        f_scale=math.radians(0.5),
        max_nfev=100,
    )
    if not heading_fit.success:
        raise ValueError(f"IMU/GNSS heading fusion failed: {heading_fit.message}")
    yaw_heading_frame = (
        integrated_yaw
        + heading_fit.x[0]
        + heading_fit.x[1] * (imu_times - imu_times[0])
    )
    heading_fit_error = wrapped_angle(heading_fit_residual(heading_fit.x))

    if heading_to_vehicle_yaw_deg is None:
        position_overlap = (
            (position_times >= imu_times[0])
            & (position_times <= imu_times[-1])
        )
        yaw_at_position = np.interp(
            position_times[position_overlap], imu_times, yaw_heading_frame
        )
        yaw_rate_at_position = np.interp(
            position_times[position_overlap], imu_times, corrected_gyro_z
        )
        moving_straight = (
            (speed[position_overlap] >= 2.0)
            & (np.abs(yaw_rate_at_position) <= 0.02)
        )
        course = np.arctan2(
            velocities[position_overlap, 1], velocities[position_overlap, 0]
        )
        offsets = wrapped_angle(course[moving_straight] - yaw_at_position[moving_straight])
        if offsets.size < 20:
            raise ValueError(
                "cannot infer GNSS-heading-to-vehicle yaw: need 20 samples with "
                "speed >= 2 m/s and |yaw rate| <= 0.02 rad/s; provide "
                "--gnss-heading-to-vehicle-yaw-deg"
            )
        heading_to_vehicle_yaw = circular_mean(offsets)
        for _ in range(3):
            errors = wrapped_angle(offsets - heading_to_vehicle_yaw)
            scale = 1.4826 * float(np.median(np.abs(errors)))
            inliers = np.abs(errors) <= max(math.radians(2.0), 3.0 * scale)
            if np.count_nonzero(inliers) < 20:
                break
            heading_to_vehicle_yaw = circular_mean(offsets[inliers])
        alignment_errors = wrapped_angle(offsets - heading_to_vehicle_yaw)
        alignment_source = "estimated_from_straight_gnss_velocity"
        alignment_samples = int(offsets.size)
        alignment_p95_deg = float(
            np.rad2deg(np.percentile(np.abs(alignment_errors), 95))
        )
        if alignment_p95_deg > 5.0:
            raise ValueError(
                "GNSS heading-to-vehicle yaw is inconsistent with straight-line "
                f"velocity (p95 {alignment_p95_deg:.2f} deg)"
            )
    else:
        heading_to_vehicle_yaw = math.radians(heading_to_vehicle_yaw_deg)
        alignment_source = "command_line"
        alignment_samples = 0
        alignment_p95_deg = 0.0

    navigation_start = max(
        imu_times[0], position_times[0], heading_times[0]
    )
    navigation_end = min(
        imu_times[-1], position_times[-1], heading_times[-1]
    )
    covered = (imu_times >= navigation_start) & (imu_times <= navigation_end)
    navigation_times = imu_times[covered]
    position_spline = CubicHermiteSpline(
        position_times, antenna_positions, velocities, axis=0
    )
    dense_positions = np.asarray(position_spline(navigation_times))
    dense_yaw = yaw_heading_frame[covered] + heading_to_vehicle_yaw
    dense_rpy = np.column_stack(
        (
            np.full(navigation_times.size, fixed_roll),
            np.full(navigation_times.size, fixed_pitch),
            dense_yaw,
        )
    )
    quaternions = Rotation.from_euler("xyz", dense_rpy).as_quat()
    step = np.linalg.norm(np.diff(dense_positions, axis=0), axis=1)
    position_statuses = [record["position_status"] for record in records]
    heading_statuses = [record["heading_status"] for record in records]
    metrics: dict[str, Any] = {
        "source": "raw_imu_and_dual_antenna_gnss",
        "uses_vehicle_pose": False,
        "uses_ins_txt": False,
        "uses_wheel": False,
        "imu_samples": int(imu.shape[0]),
        "gnss_position_samples": int(position_times.size),
        "gnss_heading_samples": int(heading_times.size),
        "pose_samples": int(navigation_times.size),
        "duration_s": float(navigation_times[-1] - navigation_times[0]),
        "path_length_m": float(step.sum()),
        "position_span_m": np.ptp(dense_positions, axis=0).tolist(),
        "rpy_span_deg": np.rad2deg(np.ptp(dense_rpy, axis=0)).tolist(),
        "yaw_net_deg": float(np.rad2deg(dense_yaw[-1] - dense_yaw[0])),
        "yaw_total_deg": float(np.rad2deg(np.abs(np.diff(dense_yaw)).sum())),
        "maximum_angular_rate_deg_s": float(
            np.rad2deg(np.max(np.abs(corrected_gyro_z[covered])))
        ),
        "gnss_position_status_counts": {
            status: position_statuses.count(status)
            for status in sorted(set(position_statuses))
        },
        "gnss_heading_status_counts": {
            status: heading_statuses.count(status)
            for status in sorted(set(heading_statuses))
        },
        "maximum_gnss_position_gap_s": float(np.max(np.diff(position_times))),
        "maximum_gnss_heading_gap_s": float(np.max(np.diff(heading_times))),
        "median_velocity_std_m_s": np.median(velocity_std, axis=0).tolist(),
        "median_heading_std_deg": float(np.rad2deg(np.median(heading_std))),
        "stationary_imu_samples": int(np.count_nonzero(stationary)),
        "gyro_bias_rad_s": gyro_bias.tolist(),
        "fixed_roll_pitch_from_gravity_deg": [
            math.degrees(fixed_roll),
            math.degrees(fixed_pitch),
        ],
        "heading_fit_bias_correction_rad_s": float(heading_fit.x[1]),
        "heading_fit_rmse_deg": float(
            np.rad2deg(np.sqrt(np.mean(np.square(heading_fit_error))))
        ),
        "heading_fit_p95_deg": float(
            np.rad2deg(np.percentile(np.abs(heading_fit_error), 95))
        ),
        "heading_to_vehicle_yaw_deg": math.degrees(heading_to_vehicle_yaw),
        "heading_to_vehicle_yaw_source": alignment_source,
        "heading_alignment_samples": alignment_samples,
        "heading_alignment_p95_deg": alignment_p95_deg,
    }
    return NavigationTrajectory(
        timestamps=navigation_times,
        antenna_positions=dense_positions,
        vehicle_quaternions=quaternions,
        metrics=metrics,
    )


def scan_index(directory: Path) -> tuple[list[Path], np.ndarray]:
    entries: list[tuple[float, Path]] = []
    for path in directory.glob("*.pcd"):
        try:
            timestamp = float(path.stem)
        except ValueError:
            continue
        if math.isfinite(timestamp):
            entries.append((timestamp, path))
    entries.sort(key=lambda item: item[0])
    if not entries:
        raise ValueError(f"no numeric PCD scans in {directory}")
    return [item[1] for item in entries], np.asarray([item[0] for item in entries])


def deterministic_axis(seed: int, name: str) -> np.ndarray:
    digest = hashlib.sha256(f"{seed}:{name}".encode()).digest()
    generator = np.random.default_rng(int.from_bytes(digest[:8], "little"))
    axis = generator.normal(size=3)
    return axis / np.linalg.norm(axis)


def perturbed_rotation(rotation: np.ndarray, degrees: float, seed: int, name: str) -> np.ndarray:
    if abs(degrees) <= 1.0e-15:
        return rotation.copy()
    perturbation = Rotation.from_rotvec(
        math.radians(degrees) * deterministic_axis(seed, name)
    ).as_matrix()
    return rotation @ perturbation


def select_pairs(
    scan_times: np.ndarray,
    navigation: NavigationTrajectory,
    frame_gap: int,
    candidate_stride: int,
    pair_count: int,
) -> list[PairCandidate]:
    if frame_gap <= 0 or candidate_stride <= 0 or pair_count <= 0:
        raise ValueError("pair selection arguments must be positive")
    valid = np.flatnonzero(
        (scan_times >= navigation.timestamps[0])
        & (scan_times <= navigation.timestamps[-1])
    )
    if valid.size < frame_gap + 1:
        return []
    poses = navigation.poses(scan_times[valid])
    pose_by_index = {int(index): pose for index, pose in zip(valid, poses)}
    candidates: list[PairCandidate] = []
    for first in valid[::candidate_stride]:
        second = int(first + frame_gap)
        if second not in pose_by_index:
            continue
        motion = inverse(pose_by_index[int(first)]) @ pose_by_index[second]
        translation_m = float(np.linalg.norm(motion[:3, 3]))
        rotation_deg = float(
            np.rad2deg(Rotation.from_matrix(motion[:3, :3]).magnitude())
        )
        if not 0.8 <= translation_m <= 6.0:
            continue
        candidates.append(
            PairCandidate(
                first=int(first),
                second=second,
                vehicle_motion=motion,
                translation_m=translation_m,
                rotation_deg=rotation_deg,
            )
        )
    candidates.sort(
        key=lambda item: (item.rotation_deg + 0.2 * item.translation_m, item.first),
        reverse=True,
    )
    selected = candidates[:pair_count]
    selected.sort(key=lambda item: item.first)
    return selected


def load_cloud(
    path: Path,
    voxel_size: float,
    minimum_range: float,
    maximum_range: float,
    *,
    estimate_normals: bool,
) -> o3d.geometry.PointCloud:
    cloud = o3d.io.read_point_cloud(
        str(path), remove_nan_points=True, remove_infinite_points=True
    )
    points = np.asarray(cloud.points)
    if points.size == 0:
        raise ValueError(f"empty point cloud: {path}")
    distance = np.linalg.norm(points, axis=1)
    keep = (distance >= minimum_range) & (distance <= maximum_range)
    cloud.points = o3d.utility.Vector3dVector(points[keep])
    cloud = cloud.voxel_down_sample(voxel_size)
    if estimate_normals:
        cloud.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=3.0 * voxel_size, max_nn=30
            )
        )
    return cloud


def multiscale_icp(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    initial: np.ndarray,
    voxel_size: float,
) -> tuple[np.ndarray, float, float]:
    transform = initial.copy()
    result = None
    for distance_scale, iterations in ((4.0, 30), (2.0, 20), (1.0, 15)):
        result = o3d.pipelines.registration.registration_icp(
            source,
            target,
            distance_scale * voxel_size,
            transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=iterations
            ),
        )
        transform = result.transformation
    assert result is not None
    return transform, float(result.fitness), float(result.inlier_rmse)


def independent_motion_icp(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    vehicle_rotation_deg: float,
    voxel_size: float,
    coarse_distance: float,
    fallback_distance: float,
    angle_tolerance_deg: float,
) -> tuple[np.ndarray, float, float, float, str]:
    """Register a LiDAR pair without consulting an extrinsic rotation.

    Conjugation preserves rotation angle, so raw IMU/GNSS supplies a valid
    hypothesis-selection test even though the LiDAR-frame rotation axis and
    translation direction are unknown.  Both hypotheses start at identity;
    only their coarse correspondence radii differ.
    """

    def schedule(coarse: float) -> list[float]:
        values = [coarse, min(coarse, 3.0), min(coarse, 2.0), 1.0, voxel_size]
        result: list[float] = []
        for value in values:
            value = max(voxel_size, float(value))
            if not result or value < result[-1] - 1.0e-9:
                result.append(value)
        return result

    def run(distances: Sequence[float]) -> tuple[np.ndarray, float, float, float]:
        transform = np.eye(4)
        result = None
        for distance in distances:
            result = o3d.pipelines.registration.registration_icp(
                source,
                target,
                distance,
                transform,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30),
            )
            transform = np.asarray(result.transformation).copy()
        assert result is not None
        measured_angle = math.degrees(
            Rotation.from_matrix(transform[:3, :3]).magnitude()
        )
        angle_error = abs(measured_angle - vehicle_rotation_deg)
        return transform, float(result.fitness), float(result.inlier_rmse), angle_error

    hypotheses: list[tuple[np.ndarray, float, float, float, str]] = []
    primary = run(schedule(coarse_distance))
    hypotheses.append((*primary, "identity_multiscale_primary"))
    primary_plausible = (
        primary[1] >= 0.15
        and primary[2] <= 0.90
        and primary[3] <= angle_tolerance_deg
        and np.linalg.norm(primary[0][:3, 3]) <= 10.0
    )
    if not primary_plausible and fallback_distance > coarse_distance + 1.0e-9:
        fallback = run(schedule(fallback_distance))
        hypotheses.append((*fallback, "identity_multiscale_fallback"))

    def score(item: tuple[np.ndarray, float, float, float, str]) -> float:
        transform, fitness, rmse, angle_error, _ = item
        translation_penalty = max(0.0, np.linalg.norm(transform[:3, 3]) - 8.0)
        return (
            angle_error
            + 0.25 * rmse
            + 2.0 * max(0.0, 0.20 - fitness)
            + translation_penalty
        )

    return min(hypotheses, key=score)


def collect_prior_free_constraints(
    lidar: LidarDefinition,
    files: list[Path],
    pairs: list[PairCandidate],
    voxel_size: float,
    minimum_range: float,
    maximum_range: float,
    coarse_distance: float,
    fallback_distance: float,
    angle_tolerance_deg: float,
) -> list[PairConstraint]:
    """Measure LiDAR-frame motion with no rotation or translation seed."""
    cache: dict[int, o3d.geometry.PointCloud] = {}
    constraints: list[PairConstraint] = []
    for number, candidate in enumerate(pairs, start=1):
        for index in (candidate.first, candidate.second):
            if index not in cache:
                cache[index] = load_cloud(
                    files[index],
                    voxel_size,
                    minimum_range,
                    maximum_range,
                    estimate_normals=True,
                )
        measured, fitness, inlier_rmse, angle_error, mode = independent_motion_icp(
            cache[candidate.second],
            cache[candidate.first],
            candidate.rotation_deg,
            voxel_size,
            coarse_distance,
            fallback_distance,
            angle_tolerance_deg,
        )
        measured_translation = float(np.linalg.norm(measured[:3, 3]))
        measured_rotation = math.degrees(
            Rotation.from_matrix(measured[:3, :3]).magnitude()
        )
        accepted = (
            fitness >= 0.15
            and inlier_rmse <= 0.90
            and angle_error <= angle_tolerance_deg
            and measured_translation <= 10.0
        )
        reasons: list[str] = []
        if fitness < 0.15:
            reasons.append("fitness below 0.15")
        if inlier_rmse > 0.90:
            reasons.append("RMSE above 0.90 m")
        if angle_error > angle_tolerance_deg:
            reasons.append("rotation-angle mismatch")
        if measured_translation > 10.0:
            reasons.append("translation above 10 m")
        constraints.append(
            PairConstraint(
                candidate=candidate,
                lidar_motion=measured,
                fitness=fitness,
                inlier_rmse_m=inlier_rmse,
                icp_translation_update_m=measured_translation,
                icp_rotation_update_deg=measured_rotation,
                accepted=accepted,
                registration_mode=mode,
                motion_angle_error_deg=angle_error,
                rejection_reason="; ".join(reasons),
            )
        )
        print(
            f"  {lidar.name}: prior-free ICP {number:02d}/{len(pairs)} "
            f"fitness={fitness:.3f} rmse={inlier_rmse:.3f}m "
            f"angle_error={angle_error:.2f}deg accepted={str(accepted).lower()}",
            flush=True,
        )
    return constraints


def assign_constraint_splits(constraints: Sequence[PairConstraint]) -> None:
    usable = [constraint for constraint in constraints if constraint.accepted]
    for index, constraint in enumerate(usable):
        constraint.split = "heldout" if index % 5 == 0 else "train"


def vector_angle_degrees(first: np.ndarray, second: np.ndarray) -> float:
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    denominator = np.linalg.norm(first) * np.linalg.norm(second)
    if denominator <= 1.0e-15:
        return math.inf
    cosine = float(np.clip(np.dot(first, second) / denominator, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def minimal_rotation_between(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Return the minimum-angle active rotation mapping ``first`` to ``second``."""
    source = np.asarray(first, dtype=float)
    target = np.asarray(second, dtype=float)
    source /= np.linalg.norm(source)
    target /= np.linalg.norm(target)
    cross = np.cross(source, target)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if sine > 1.0e-12:
        return Rotation.from_rotvec(
            cross / sine * math.atan2(sine, cosine)
        ).as_matrix()
    if cosine > 0.0:
        return np.eye(3)
    basis = np.eye(3)[int(np.argmin(np.abs(source)))]
    axis = np.cross(source, basis)
    axis /= np.linalg.norm(axis)
    return Rotation.from_rotvec(math.pi * axis).as_matrix()


def constraint_axis_samples(
    constraints: Sequence[PairConstraint],
) -> tuple[list[PairConstraint], np.ndarray, np.ndarray, np.ndarray]:
    selected: list[PairConstraint] = []
    vehicle_axes: list[np.ndarray] = []
    lidar_axes: list[np.ndarray] = []
    weights: list[float] = []
    for constraint in constraints:
        if not constraint.accepted:
            continue
        vehicle_vector = Rotation.from_matrix(
            constraint.candidate.vehicle_motion[:3, :3]
        ).as_rotvec()
        lidar_vector = Rotation.from_matrix(
            constraint.lidar_motion[:3, :3]
        ).as_rotvec()
        if np.linalg.norm(vehicle_vector) < math.radians(1.0):
            continue
        if np.linalg.norm(lidar_vector) < math.radians(0.5):
            continue
        vehicle_axis = vehicle_vector / np.linalg.norm(vehicle_vector)
        lidar_axis = lidar_vector / np.linalg.norm(lidar_vector)
        sign = 1.0 if vehicle_axis[2] >= 0.0 else -1.0
        selected.append(constraint)
        vehicle_axes.append(sign * vehicle_axis)
        lidar_axes.append(sign * lidar_axis)
        weights.append(
            math.sqrt(max(0.05, constraint.fitness))
            * np.linalg.norm(vehicle_vector)
            / max(0.10, constraint.inlier_rmse_m)
        )
    return (
        selected,
        np.asarray(vehicle_axes),
        np.asarray(lidar_axes),
        np.asarray(weights),
    )


def robust_planar_axes(
    constraints: Sequence[PairConstraint], axis_inlier_deg: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Estimate the conjugate planar rotation axes with a one-sample RANSAC."""
    selected, vehicle_axes, lidar_axes, weights = constraint_axis_samples(constraints)
    if len(selected) < 3:
        raise ValueError(f"need three rotational motion samples; got {len(selected)}")
    best_support = np.zeros(len(selected), dtype=bool)
    best_weight = -math.inf
    for hypothesis in lidar_axes:
        errors = np.asarray(
            [vector_angle_degrees(hypothesis, sample) for sample in lidar_axes]
        )
        support = errors <= axis_inlier_deg
        support_weight = float(np.sum(weights[support]))
        if np.count_nonzero(support) >= 3 and support_weight > best_weight:
            best_support = support
            best_weight = support_weight
    if np.count_nonzero(best_support) < 3:
        raise ValueError("no consistent LiDAR rotation-axis cluster")
    lidar_axis = np.average(
        lidar_axes[best_support], axis=0, weights=weights[best_support]
    )
    vehicle_axis = np.average(
        vehicle_axes[best_support], axis=0, weights=weights[best_support]
    )
    lidar_axis /= np.linalg.norm(lidar_axis)
    vehicle_axis /= np.linalg.norm(vehicle_axis)
    all_errors = np.asarray(
        [vector_angle_degrees(lidar_axis, sample) for sample in lidar_axes]
    )
    support = all_errors <= axis_inlier_deg
    p95 = float(np.percentile(all_errors[support], 95))
    for constraint, error in zip(selected, all_errors):
        constraint.axis_error_deg = float(error)
        if error > axis_inlier_deg:
            constraint.accepted = False
            constraint.rejection_reason = "rotation axis is inconsistent"
    return vehicle_axis, lidar_axis, support, p95


def basis_perpendicular_to(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    unit_axis = np.asarray(axis, dtype=float)
    unit_axis /= np.linalg.norm(unit_axis)
    seed = np.eye(3)[int(np.argmin(np.abs(unit_axis)))]
    first = seed - unit_axis * np.dot(unit_axis, seed)
    first /= np.linalg.norm(first)
    second = np.cross(unit_axis, first)
    return first, second


def planar_yaw_procrustes(
    base_rotation: np.ndarray,
    vehicle_axis: np.ndarray,
    translation: np.ndarray,
    constraints: Sequence[PairConstraint],
    gnss_lever_arm: np.ndarray,
) -> float:
    """Solve the remaining rotation about the planar vehicle axis globally."""
    first, second = basis_perpendicular_to(vehicle_axis)
    cosine_term = 0.0
    sine_term = 0.0
    for constraint in constraints:
        if not constraint.accepted or constraint.split != "train":
            continue
        vehicle_motion = motion_at_vehicle_origin(
            constraint.candidate.vehicle_motion, gnss_lever_arm
        )
        target = (
            vehicle_motion[:3, 3]
            + (vehicle_motion[:3, :3] - np.eye(3)) @ translation
        )
        source = base_rotation @ constraint.lidar_motion[:3, 3]
        source_xy = np.asarray([np.dot(first, source), np.dot(second, source)])
        target_xy = np.asarray([np.dot(first, target), np.dot(second, target)])
        weight = math.sqrt(max(0.05, constraint.fitness)) / max(
            0.10, constraint.inlier_rmse_m
        )
        cosine_term += weight * float(np.dot(target_xy, source_xy))
        sine_term += weight * float(
            target_xy[1] * source_xy[0] - target_xy[0] * source_xy[1]
        )
    if math.hypot(cosine_term, sine_term) <= 1.0e-12:
        raise ValueError("planar translation supplies no yaw information")
    return math.atan2(sine_term, cosine_term)


def estimate_prior_free_initialization(
    lidars: Sequence[LidarDefinition],
    constraints_by_lidar: dict[str, list[PairConstraint]],
    minimum_constraints: int,
    lever_arm_bound_m: float,
    axis_inlier_deg: float,
) -> PriorFreeInitialization:
    """Recover arbitrary LiDAR rotations from motion axes and fixed translations."""
    lidar_by_name = {lidar.name: lidar for lidar in lidars}
    names = [lidar.name for lidar in lidars]
    output = PriorFreeInitialization(lidars={}, gnss_lever_arm=np.zeros(3))
    base_rotations: dict[str, np.ndarray] = {}
    for name in names:
        constraints = constraints_by_lidar[name]
        assign_constraint_splits(constraints)
        training = [
            item
            for item in constraints
            if item.accepted and item.split == "train"
        ]
        axis_samples = len(training)
        try:
            vehicle_axis, lidar_axis, support, p95 = robust_planar_axes(
                training, axis_inlier_deg
            )
            del support
            selected_all, _, lidar_axes_all, _ = constraint_axis_samples(constraints)
            for constraint, sample_axis in zip(selected_all, lidar_axes_all):
                error = vector_angle_degrees(lidar_axis, sample_axis)
                constraint.axis_error_deg = error
                if error > axis_inlier_deg:
                    constraint.accepted = False
                    constraint.rejection_reason = "rotation axis is inconsistent"
            base_rotation = minimal_rotation_between(lidar_axis, vehicle_axis)
            base_rotations[name] = base_rotation
            axis_inliers = sum(
                item.accepted and item.axis_error_deg <= axis_inlier_deg
                for item in training
            )
            output.lidars[name] = PriorFreeLidarInitialization(
                name=name,
                rotation=base_rotation,
                vehicle_axis=vehicle_axis,
                lidar_axis=lidar_axis,
                axis_samples=axis_samples,
                axis_inliers=axis_inliers,
                axis_p95_deg=p95,
            )
        except ValueError as error:
            output.lidars[name] = PriorFreeLidarInitialization(
                name=name,
                rotation=np.eye(3),
                vehicle_axis=np.asarray([0.0, 0.0, 1.0]),
                lidar_axis=np.asarray([0.0, 0.0, 1.0]),
                reason=str(error),
            )
    if len(base_rotations) != len(names):
        output.reason = "at least one LiDAR lacks a consistent planar rotation axis"
        return output

    training_by_lidar = {
        name: [
            item
            for item in constraints_by_lidar[name]
            if item.accepted and item.split == "train"
        ]
        for name in names
    }

    def rotations(parameters: np.ndarray) -> dict[str, np.ndarray]:
        return {
            name: Rotation.from_rotvec(
                parameters[index] * output.lidars[name].vehicle_axis
            ).as_matrix()
            @ base_rotations[name]
            for index, name in enumerate(names)
        }

    def residual(parameters: np.ndarray) -> np.ndarray:
        lever_arm = np.asarray([parameters[-2], parameters[-1], 0.0])
        current_rotations = rotations(parameters)
        values: list[float] = []
        for name in names:
            lidar = lidar_by_name[name]
            current_rotation = current_rotations[name]
            for constraint in training_by_lidar[name]:
                vehicle_motion = motion_at_vehicle_origin(
                    constraint.candidate.vehicle_motion, lever_arm
                )
                target = (
                    vehicle_motion[:3, 3]
                    + (vehicle_motion[:3, :3] - np.eye(3)) @ lidar.translation
                )
                measured = current_rotation @ constraint.lidar_motion[:3, 3]
                weight = math.sqrt(max(0.05, constraint.fitness)) / max(
                    0.10, constraint.inlier_rmse_m
                )
                values.extend(weight * (measured - target))
        return np.asarray(values)

    lever_seeds = [np.zeros(2)]
    lever_seeds.extend(lidar.translation[:2] for lidar in lidars)
    lever_seeds.append(
        np.median(np.asarray([lidar.translation[:2] for lidar in lidars]), axis=0)
    )
    unique_seeds: list[np.ndarray] = []
    for seed in lever_seeds:
        clipped = np.clip(
            np.asarray(seed, dtype=float),
            -0.999 * lever_arm_bound_m,
            0.999 * lever_arm_bound_m,
        )
        if not any(np.linalg.norm(clipped - prior) < 1.0e-6 for prior in unique_seeds):
            unique_seeds.append(clipped)

    best_solution = None
    for lever_seed in unique_seeds:
        lever = np.asarray([lever_seed[0], lever_seed[1], 0.0])
        beta = []
        try:
            for name in names:
                beta.append(
                    planar_yaw_procrustes(
                        base_rotations[name],
                        output.lidars[name].vehicle_axis,
                        lidar_by_name[name].translation,
                        training_by_lidar[name],
                        lever,
                    )
                )
        except ValueError:
            continue
        initial = np.asarray([*beta, *lever_seed])
        lower = np.asarray([-4.0 * math.pi] * len(names) + [-lever_arm_bound_m] * 2)
        upper = np.asarray([4.0 * math.pi] * len(names) + [lever_arm_bound_m] * 2)
        solution = least_squares(
            residual,
            initial,
            bounds=(lower, upper),
            loss="cauchy",
            f_scale=1.0,
            max_nfev=400,
        )
        output.starts_attempted += 1
        if best_solution is None or solution.cost < best_solution.cost:
            best_solution = solution
    if best_solution is None:
        output.reason = "no prior-free yaw/lever-arm start was solvable"
        return output

    output.selected_cost = float(best_solution.cost)
    output.gnss_lever_arm = np.asarray(
        [best_solution.x[-2], best_solution.x[-1], 0.0]
    )
    final_rotations = rotations(best_solution.x)
    hessian = best_solution.jac.T @ best_solution.jac
    eigenvalues = np.linalg.eigvalsh(hessian)
    output.hessian_eigenvalues = eigenvalues.tolist()
    output.hessian_condition_number = (
        float(eigenvalues[-1] / eigenvalues[0])
        if eigenvalues.size and eigenvalues[0] > 0.0
        else math.inf
    )
    observable = (
        eigenvalues.size == len(names) + 2
        and eigenvalues[0] > 1.0e-9
        and math.isfinite(output.hessian_condition_number)
        and output.hessian_condition_number < 1.0e8
    )
    for index, name in enumerate(names):
        lidar_output = output.lidars[name]
        lidar_output.rotation = final_rotations[name]
        lidar_output.yaw_about_vehicle_axis_deg = math.degrees(best_solution.x[index])
        training = training_by_lidar[name]
        heldout = [
            item
            for item in constraints_by_lidar[name]
            if item.accepted and item.split == "heldout"
        ]
        lidar_output.train_metrics = constraint_metrics(
            final_rotations[name],
            lidar_by_name[name].translation,
            training,
            output.gnss_lever_arm,
        )
        lidar_output.heldout_metrics = constraint_metrics(
            final_rotations[name],
            lidar_by_name[name].translation,
            heldout,
            output.gnss_lever_arm,
        )
        enough = len(training) >= max(3, minimum_constraints * 3 // 5) and bool(heldout)
        axis_good = (
            lidar_output.axis_inliers >= max(3, minimum_constraints * 3 // 5)
            and lidar_output.axis_p95_deg <= axis_inlier_deg
        )
        heldout_good = (
            lidar_output.heldout_metrics["translation_rmse_m"] <= 0.75
            and lidar_output.heldout_metrics["rotation_rmse_deg"] <= 2.5
        )
        lidar_output.accepted = bool(enough and axis_good and heldout_good)
        if not enough:
            lidar_output.reason = "not enough train/held-out independent motions"
        elif not axis_good:
            lidar_output.reason = "LiDAR rotation axis is not stable"
        elif not heldout_good:
            lidar_output.reason = "held-out hand-eye residual exceeds capture gate"
        else:
            lidar_output.reason = (
                "accepted: axis plus fixed-translation yaw initialized"
            )
    output.accepted = bool(
        best_solution.success
        and observable
        and all(item.accepted for item in output.lidars.values())
    )
    if not best_solution.success:
        output.reason = f"joint prior-free solve failed: {best_solution.message}"
    elif not observable:
        output.reason = "joint yaw/lever-arm Hessian is rank-deficient"
    elif not output.accepted:
        output.reason = "at least one LiDAR failed prior-free validation"
    else:
        output.reason = "accepted: rotation priors were not used"
    return output


def collect_constraints(
    lidar: LidarDefinition,
    files: list[Path],
    pairs: list[PairCandidate],
    initial_rotation: np.ndarray,
    voxel_size: float,
    minimum_range: float,
    maximum_range: float,
    gnss_lever_arm_vehicle: np.ndarray | None = None,
) -> list[PairConstraint]:
    cache: dict[int, o3d.geometry.PointCloud] = {}
    initial_extrinsic = make_transform(initial_rotation, lidar.translation)
    initial_inverse = inverse(initial_extrinsic)
    constraints: list[PairConstraint] = []
    for number, candidate in enumerate(pairs, start=1):
        for index in (candidate.first, candidate.second):
            if index not in cache:
                cache[index] = load_cloud(
                    files[index],
                    voxel_size,
                    minimum_range,
                    maximum_range,
                    estimate_normals=True,
                )
        vehicle_motion = motion_at_vehicle_origin(
            candidate.vehicle_motion, gnss_lever_arm_vehicle
        )
        predicted = initial_inverse @ vehicle_motion @ initial_extrinsic
        measured, fitness, inlier_rmse = multiscale_icp(
            cache[candidate.second], cache[candidate.first], predicted, voxel_size
        )
        translation_update = float(
            np.linalg.norm(measured[:3, 3] - predicted[:3, 3])
        )
        rotation_update = angular_distance_degrees(
            predicted[:3, :3], measured[:3, :3]
        )
        accepted = (
            fitness >= 0.25
            and inlier_rmse <= 0.8
            and translation_update <= 2.5
            and rotation_update <= 10.0
        )
        constraints.append(
            PairConstraint(
                candidate=candidate,
                lidar_motion=measured,
                fitness=fitness,
                inlier_rmse_m=inlier_rmse,
                icp_translation_update_m=translation_update,
                icp_rotation_update_deg=rotation_update,
                accepted=accepted,
            )
        )
        print(
            f"  {lidar.name}: ICP {number:02d}/{len(pairs)} "
            f"fitness={fitness:.3f} rmse={inlier_rmse:.3f}m "
            f"accepted={str(accepted).lower()}",
            flush=True,
        )
    return constraints


def handeye_residual(
    delta: np.ndarray,
    initial_rotation: np.ndarray,
    fixed_translation: np.ndarray,
    constraints: Sequence[PairConstraint],
    gnss_lever_arm_vehicle: np.ndarray | None = None,
) -> np.ndarray:
    rotation = initial_rotation @ Rotation.from_rotvec(delta).as_matrix()
    extrinsic = make_transform(rotation, fixed_translation)
    extrinsic_inverse = inverse(extrinsic)
    residuals: list[float] = []
    for constraint in constraints:
        vehicle_motion = motion_at_vehicle_origin(
            constraint.candidate.vehicle_motion, gnss_lever_arm_vehicle
        )
        predicted = (
            extrinsic_inverse @ vehicle_motion @ extrinsic
        )
        measured = constraint.lidar_motion
        rotation_error = Rotation.from_matrix(
            measured[:3, :3].T @ predicted[:3, :3]
        ).as_rotvec()
        translation_error = predicted[:3, 3] - measured[:3, 3]
        weight = math.sqrt(max(0.05, constraint.fitness)) / max(
            0.10, constraint.inlier_rmse_m
        )
        residuals.extend(weight * 2.0 * rotation_error)
        residuals.extend(weight * translation_error)
    return np.asarray(residuals)


def constraint_metrics(
    rotation: np.ndarray,
    translation: np.ndarray,
    constraints: Sequence[PairConstraint],
    gnss_lever_arm_vehicle: np.ndarray | None = None,
) -> dict[str, float]:
    if not constraints:
        return {
            "count": 0,
            "translation_rmse_m": math.inf,
            "translation_p95_m": math.inf,
            "rotation_rmse_deg": math.inf,
            "rotation_p95_deg": math.inf,
        }
    extrinsic = make_transform(rotation, translation)
    extrinsic_inverse = inverse(extrinsic)
    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    for constraint in constraints:
        vehicle_motion = motion_at_vehicle_origin(
            constraint.candidate.vehicle_motion, gnss_lever_arm_vehicle
        )
        predicted = (
            extrinsic_inverse @ vehicle_motion @ extrinsic
        )
        measured = constraint.lidar_motion
        translation_errors.append(
            float(np.linalg.norm(predicted[:3, 3] - measured[:3, 3]))
        )
        rotation_errors.append(
            angular_distance_degrees(measured[:3, :3], predicted[:3, :3])
        )
    translation_array = np.asarray(translation_errors)
    rotation_array = np.asarray(rotation_errors)
    return {
        "count": len(constraints),
        "translation_rmse_m": float(np.sqrt(np.mean(np.square(translation_array)))),
        "translation_p95_m": float(np.percentile(translation_array, 95)),
        "rotation_rmse_deg": float(np.sqrt(np.mean(np.square(rotation_array)))),
        "rotation_p95_deg": float(np.percentile(rotation_array, 95)),
    }


def calibrate_rotation(
    lidar: LidarDefinition,
    initial_rotation: np.ndarray,
    constraints: list[PairConstraint],
    minimum_constraints: int,
    update_bound_deg: float,
    trusted_rotation: np.ndarray | None,
    gnss_lever_arm_vehicle: np.ndarray | None = None,
    require_heldout_improvement: bool = True,
) -> RotationCalibration:
    result = RotationCalibration(
        name=lidar.name,
        initial_rotation=initial_rotation,
        optimized_rotation=initial_rotation.copy(),
        fixed_translation=lidar.translation.copy(),
        constraints=constraints,
    )
    usable = [constraint for constraint in constraints if constraint.accepted]
    if not any(constraint.split in ("train", "heldout") for constraint in usable):
        assign_constraint_splits(constraints)
    training = [constraint for constraint in usable if constraint.split == "train"]
    heldout = [constraint for constraint in usable if constraint.split == "heldout"]
    if trusted_rotation is not None:
        result.trusted_initial_error_deg = angular_distance_degrees(
            trusted_rotation, initial_rotation
        )
    if len(usable) < minimum_constraints or len(training) < 3 or not heldout:
        result.reason = (
            f"need at least {minimum_constraints} accepted constraints with a "
            f"non-empty held-out split; got {len(usable)}"
        )
        return result

    bound = math.radians(update_bound_deg)
    solution = least_squares(
        handeye_residual,
        np.zeros(3),
        args=(
            initial_rotation,
            lidar.translation,
            training,
            gnss_lever_arm_vehicle,
        ),
        bounds=(-bound, bound),
        loss="cauchy",
        f_scale=1.0,
        max_nfev=200,
    )
    optimized_rotation = (
        initial_rotation @ Rotation.from_rotvec(solution.x).as_matrix()
    )
    result.optimized_rotation = optimized_rotation
    result.update_deg = float(np.rad2deg(np.linalg.norm(solution.x)))
    result.train_initial = constraint_metrics(
        initial_rotation,
        lidar.translation,
        training,
        gnss_lever_arm_vehicle,
    )
    result.train_final = constraint_metrics(
        optimized_rotation,
        lidar.translation,
        training,
        gnss_lever_arm_vehicle,
    )
    result.heldout_initial = constraint_metrics(
        initial_rotation,
        lidar.translation,
        heldout,
        gnss_lever_arm_vehicle,
    )
    result.heldout_final = constraint_metrics(
        optimized_rotation,
        lidar.translation,
        heldout,
        gnss_lever_arm_vehicle,
    )
    if trusted_rotation is not None:
        result.trusted_final_error_deg = angular_distance_degrees(
            trusted_rotation, optimized_rotation
        )

    if not require_heldout_improvement:
        initial_selection_score = (
            result.heldout_initial["translation_rmse_m"] / 0.20
            + result.heldout_initial["rotation_rmse_deg"] / 0.20
        )
        final_selection_score = (
            result.heldout_final["translation_rmse_m"] / 0.20
            + result.heldout_final["rotation_rmse_deg"] / 0.20
        )
        if final_selection_score > initial_selection_score:
            result.refinement_selected = False
            result.optimized_rotation = initial_rotation.copy()
            result.update_deg = 0.0
            result.train_final = dict(result.train_initial)
            result.heldout_final = dict(result.heldout_initial)
            if trusted_rotation is not None:
                result.trusted_final_error_deg = result.trusted_initial_error_deg

    hessian = solution.jac.T @ solution.jac
    eigenvalues = np.linalg.eigvalsh(hessian)
    result.hessian_eigenvalues = eigenvalues.tolist()
    result.hessian_condition_number = (
        float(eigenvalues[-1] / eigenvalues[0]) if eigenvalues[0] > 0.0 else math.inf
    )
    degrees_of_freedom = max(1, solution.fun.size - 3)
    residual_variance = float(np.dot(solution.fun, solution.fun) / degrees_of_freedom)
    covariance = np.linalg.pinv(hessian) * residual_variance
    result.one_sigma_deg = np.rad2deg(
        np.sqrt(np.maximum(0.0, np.diag(covariance)))
    ).tolist()

    heldout_translation_improved = (
        result.heldout_final["translation_rmse_m"]
        < result.heldout_initial["translation_rmse_m"]
    )
    heldout_translation_accepted = (
        heldout_translation_improved
        if require_heldout_improvement
        else result.heldout_final["translation_rmse_m"]
        <= max(
            result.heldout_initial["translation_rmse_m"] + 1.0e-6,
            1.05 * result.heldout_initial["translation_rmse_m"],
        )
    )
    heldout_rotation_not_regressed = (
        result.heldout_final["rotation_rmse_deg"]
        <= 1.05 * result.heldout_initial["rotation_rmse_deg"]
        if require_heldout_improvement
        else result.heldout_final["rotation_rmse_deg"] <= 2.5
    )
    observable = (
        eigenvalues[0] > 1.0e-9
        and math.isfinite(result.hessian_condition_number)
        and result.hessian_condition_number < 1.0e6
    )
    bounded = result.update_deg <= 1.05 * update_bound_deg
    result.accepted = bool(
        solution.success
        and observable
        and bounded
        and heldout_translation_accepted
        and heldout_rotation_not_regressed
    )
    if not solution.success:
        result.reason = f"nonlinear solve failed: {solution.message}"
    elif not observable:
        result.reason = "rotation Hessian is rank-deficient or ill-conditioned"
    elif not bounded:
        result.reason = "rotation update exceeded configured bound"
    elif not heldout_translation_accepted:
        result.reason = (
            "held-out translation consistency did not improve"
            if require_heldout_improvement
            else "held-out translation consistency regressed"
        )
    elif not heldout_rotation_not_regressed:
        result.reason = "held-out rotation consistency regressed"
    else:
        result.reason = (
            "accepted: prior-free initializer retained by held-out validation"
            if not result.refinement_selected
            else "accepted: fixed-translation hand-eye validation passed"
        )
    return result


def lever_arm_translation_errors(
    lidars: dict[str, LidarDefinition],
    calibrations: dict[str, RotationCalibration],
    lever_arm: np.ndarray,
    split: str,
) -> np.ndarray:
    errors: list[float] = []
    for name, calibration in calibrations.items():
        extrinsic = make_transform(
            calibration.optimized_rotation, lidars[name].translation
        )
        extrinsic_inverse = inverse(extrinsic)
        for constraint in calibration.constraints:
            if not constraint.accepted or constraint.split != split:
                continue
            vehicle_motion = motion_at_vehicle_origin(
                constraint.candidate.vehicle_motion, lever_arm
            )
            predicted = extrinsic_inverse @ vehicle_motion @ extrinsic
            errors.append(
                float(np.linalg.norm(predicted[:3, 3] - constraint.lidar_motion[:3, 3]))
            )
    return np.asarray(errors)


def estimate_gnss_lever_arm(
    lidar_definitions: Sequence[LidarDefinition],
    calibrations: dict[str, RotationCalibration],
    initial_lever_arm: np.ndarray,
    bound_m: float,
) -> NavigationLeverArmEstimate:
    """Estimate the shared planar vehicle-to-GNSS antenna translation."""
    definitions = {lidar.name: lidar for lidar in lidar_definitions}
    result = NavigationLeverArmEstimate(value=initial_lever_arm.copy())
    training_count = sum(
        constraint.accepted and constraint.split == "train"
        for calibration in calibrations.values()
        for constraint in calibration.constraints
    )
    heldout_count = sum(
        constraint.accepted and constraint.split == "heldout"
        for calibration in calibrations.values()
        for constraint in calibration.constraints
    )
    if training_count < 6 or heldout_count < 2:
        result.reason = "not enough train/held-out constraints for GNSS lever arm"
        return result

    def residual(planar_lever_arm: np.ndarray) -> np.ndarray:
        lever_arm = np.asarray(
            [planar_lever_arm[0], planar_lever_arm[1], 0.0], dtype=float
        )
        values: list[float] = []
        for name, calibration in calibrations.items():
            extrinsic = make_transform(
                calibration.optimized_rotation, definitions[name].translation
            )
            extrinsic_inverse = inverse(extrinsic)
            for constraint in calibration.constraints:
                if not constraint.accepted or constraint.split != "train":
                    continue
                vehicle_motion = motion_at_vehicle_origin(
                    constraint.candidate.vehicle_motion, lever_arm
                )
                predicted = extrinsic_inverse @ vehicle_motion @ extrinsic
                weight = math.sqrt(max(0.05, constraint.fitness)) / max(
                    0.10, constraint.inlier_rmse_m
                )
                values.extend(
                    weight
                    * (predicted[:3, 3] - constraint.lidar_motion[:3, 3])
                )
        return np.asarray(values)

    solution = least_squares(
        residual,
        initial_lever_arm[:2],
        bounds=(-bound_m, bound_m),
        loss="cauchy",
        f_scale=1.0,
        max_nfev=200,
    )
    result.value = np.asarray([solution.x[0], solution.x[1], 0.0])
    result.update_m = float(np.linalg.norm(result.value - initial_lever_arm))
    hessian = solution.jac.T @ solution.jac
    eigenvalues = np.linalg.eigvalsh(hessian)
    result.hessian_eigenvalues = eigenvalues.tolist()
    result.hessian_condition_number = (
        float(eigenvalues[-1] / eigenvalues[0])
        if eigenvalues[0] > 0.0
        else math.inf
    )
    initial_errors = lever_arm_translation_errors(
        definitions, calibrations, initial_lever_arm, "heldout"
    )
    final_errors = lever_arm_translation_errors(
        definitions, calibrations, result.value, "heldout"
    )
    result.heldout_initial_translation_rmse_m = float(
        np.sqrt(np.mean(np.square(initial_errors)))
    )
    result.heldout_final_translation_rmse_m = float(
        np.sqrt(np.mean(np.square(final_errors)))
    )
    observable = (
        eigenvalues[0] > 1.0e-9
        and math.isfinite(result.hessian_condition_number)
        and result.hessian_condition_number < 1.0e6
    )
    bounded = np.linalg.norm(result.value[:2]) <= 1.05 * bound_m
    heldout_not_regressed = (
        result.heldout_final_translation_rmse_m
        <= 1.05 * result.heldout_initial_translation_rmse_m
    )
    result.accepted = bool(
        solution.success and observable and bounded and heldout_not_regressed
    )
    if not solution.success:
        result.reason = f"GNSS lever-arm solve failed: {solution.message}"
    elif not observable:
        result.reason = "GNSS planar lever arm is rank-deficient or ill-conditioned"
    elif not bounded:
        result.reason = "GNSS planar lever arm exceeded configured bound"
    elif not heldout_not_regressed:
        result.reason = "GNSS lever arm regressed held-out translation consistency"
    else:
        result.reason = "accepted: shared planar GNSS lever arm is observable"
    return result


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def append_cloud(
    destination: o3d.geometry.PointCloud, points: np.ndarray
) -> o3d.geometry.PointCloud:
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(points)
    destination += source
    return destination


def cross_lidar_alignment_metrics(
    names: Sequence[str], point_groups: Sequence[np.ndarray]
) -> dict[str, Any]:
    per_lidar: dict[str, Any] = {}
    all_matched: list[np.ndarray] = []
    for index, (name, original_source) in enumerate(zip(names, point_groups)):
        target_groups = [
            points for other, points in enumerate(point_groups)
            if other != index and points.size != 0
        ]
        if original_source.size == 0 or not target_groups:
            per_lidar[name] = {"queries": 0, "overlap_fraction_lt_1m": 0.0}
            continue
        source = original_source
        if source.shape[0] > 30000:
            source = source[
                np.linspace(0, source.shape[0] - 1, 30000, dtype=int)
            ]
        distances, _ = cKDTree(np.concatenate(target_groups, axis=0)).query(
            source, k=1, workers=-1
        )
        matched = distances[distances < 1.0]
        all_matched.append(matched)
        per_lidar[name] = {
            "queries": int(distances.size),
            "overlap_fraction_lt_1m": float(matched.size / distances.size),
            "matched_median_m": (
                float(np.median(matched)) if matched.size else math.inf
            ),
            "matched_p95_m": (
                float(np.percentile(matched, 95)) if matched.size else math.inf
            ),
        }
    combined = np.concatenate(all_matched) if all_matched else np.asarray([])
    return {
        "matched_queries": int(combined.size),
        "mean_overlap_fraction_lt_1m": float(
            np.mean(
                [value["overlap_fraction_lt_1m"] for value in per_lidar.values()]
            )
        ),
        "matched_median_m": float(np.median(combined)) if combined.size else math.inf,
        "matched_p95_m": (
            float(np.percentile(combined, 95)) if combined.size else math.inf
        ),
        "per_lidar": per_lidar,
    }


def build_maps(
    lidars: Sequence[LidarDefinition],
    calibrations: dict[str, RotationCalibration],
    navigation: NavigationTrajectory,
    gnss_lever_arm_vehicle: np.ndarray,
    stride: int,
    voxel_size: float,
    minimum_range: float,
    maximum_range: float,
    output_dir: Path,
) -> dict[str, Any]:
    if stride <= 0:
        raise ValueError("map stride must be positive")
    combined_initial = o3d.geometry.PointCloud()
    combined_optimized = o3d.geometry.PointCloud()
    per_lidar: dict[str, dict[str, int]] = {}
    initial_groups: list[np.ndarray] = []
    optimized_groups: list[np.ndarray] = []
    for lidar in lidars:
        files, timestamps = scan_index(lidar.directory)
        valid = np.flatnonzero(
            (timestamps >= navigation.timestamps[0])
            & (timestamps <= navigation.timestamps[-1])
        )[::stride]
        poses = navigation.poses(timestamps[valid], gnss_lever_arm_vehicle)
        calibration = calibrations[lidar.name]
        initial_extrinsic = make_transform(
            calibration.initial_rotation, calibration.fixed_translation
        )
        optimized_extrinsic = make_transform(
            calibration.optimized_rotation, calibration.fixed_translation
        )
        initial_map = o3d.geometry.PointCloud()
        optimized_map = o3d.geometry.PointCloud()
        for number, (index, world_T_vehicle) in enumerate(zip(valid, poses), start=1):
            cloud = load_cloud(
                files[int(index)],
                voxel_size,
                minimum_range,
                maximum_range,
                estimate_normals=False,
            )
            points = np.asarray(cloud.points)
            append_cloud(
                initial_map,
                transform_points(points, world_T_vehicle @ initial_extrinsic),
            )
            append_cloud(
                optimized_map,
                transform_points(points, world_T_vehicle @ optimized_extrinsic),
            )
            if number % 8 == 0:
                initial_map = initial_map.voxel_down_sample(voxel_size)
                optimized_map = optimized_map.voxel_down_sample(voxel_size)
        initial_map = initial_map.voxel_down_sample(voxel_size)
        optimized_map = optimized_map.voxel_down_sample(voxel_size)
        initial_map.paint_uniform_color(lidar.color)
        optimized_map.paint_uniform_color(lidar.color)
        combined_initial += initial_map
        combined_optimized += optimized_map
        initial_groups.append(np.asarray(initial_map.points).copy())
        optimized_groups.append(np.asarray(optimized_map.points).copy())
        per_lidar[lidar.name] = {
            "frames": int(valid.size),
            "initial_points": len(initial_map.points),
            "optimized_points": len(optimized_map.points),
        }
        o3d.io.write_point_cloud(
            str(output_dir / f"map_{lidar.name}.pcd"),
            optimized_map,
            write_ascii=False,
            compressed=True,
        )
        print(
            f"  map {lidar.name}: {valid.size} frames, "
            f"{len(optimized_map.points)} points",
            flush=True,
        )

    initial_occupied = len(combined_initial.voxel_down_sample(voxel_size).points)
    optimized_occupied = len(combined_optimized.voxel_down_sample(voxel_size).points)
    o3d.io.write_point_cloud(
        str(output_dir / "map_initial_rgb.pcd"),
        combined_initial,
        write_ascii=False,
        compressed=True,
    )
    o3d.io.write_point_cloud(
        str(output_dir / "map_optimized_rgb.pcd"),
        combined_optimized,
        write_ascii=False,
        compressed=True,
    )
    return {
        "map_stride": stride,
        "voxel_size_m": voxel_size,
        "initial_colored_points": len(combined_initial.points),
        "optimized_colored_points": len(combined_optimized.points),
        "initial_occupied_voxels": initial_occupied,
        "optimized_occupied_voxels": optimized_occupied,
        "occupied_voxel_reduction_fraction": (
            float(initial_occupied - optimized_occupied) / initial_occupied
            if initial_occupied
            else 0.0
        ),
        "cross_lidar_alignment_initial": cross_lidar_alignment_metrics(
            [lidar.name for lidar in lidars], initial_groups
        ),
        "cross_lidar_alignment_optimized": cross_lidar_alignment_metrics(
            [lidar.name for lidar in lidars], optimized_groups
        ),
        "per_lidar": per_lidar,
    }


def rotation_values(rotation: np.ndarray) -> dict[str, list[float]]:
    quaternion = Rotation.from_matrix(rotation).as_quat()
    rpy_deg = Rotation.from_matrix(rotation).as_euler("xyz", degrees=True)
    return {
        "rpy_deg": [float(value) for value in rpy_deg],
        "quaternion_xyzw": [float(value) for value in quaternion],
    }


def calibration_document(calibration: RotationCalibration) -> dict[str, Any]:
    usable = [constraint for constraint in calibration.constraints if constraint.accepted]
    return {
        "accepted": calibration.accepted,
        "reason": calibration.reason,
        "translation_fixed": True,
        "vehicle_translation_m": calibration.fixed_translation.tolist(),
        "initial_rotation": rotation_values(calibration.initial_rotation),
        "optimized_rotation": rotation_values(calibration.optimized_rotation),
        "rotation_update_deg": calibration.update_deg,
        "refinement_selected": calibration.refinement_selected,
        "trusted_initial_error_deg": calibration.trusted_initial_error_deg,
        "trusted_final_error_deg": calibration.trusted_final_error_deg,
        "constraints_attempted": len(calibration.constraints),
        "constraints_accepted": len(usable),
        "training_constraints": sum(item.split == "train" for item in usable),
        "heldout_constraints": sum(item.split == "heldout" for item in usable),
        "train_initial": calibration.train_initial,
        "train_final": calibration.train_final,
        "heldout_initial": calibration.heldout_initial,
        "heldout_final": calibration.heldout_final,
        "hessian_eigenvalues": calibration.hessian_eigenvalues,
        "hessian_condition_number": calibration.hessian_condition_number,
        "rotation_one_sigma_deg": calibration.one_sigma_deg,
    }


def prior_free_initialization_document(
    initialization: PriorFreeInitialization | None,
) -> dict[str, Any]:
    if initialization is None:
        return {"enabled": False}
    return {
        "enabled": True,
        "accepted": initialization.accepted,
        "reason": initialization.reason,
        "manifest_rotations_used": False,
        "independent_motion_seed": "identity",
        "gnss_lever_arm_vehicle_m": initialization.gnss_lever_arm.tolist(),
        "starts_attempted": initialization.starts_attempted,
        "selected_cost": initialization.selected_cost,
        "hessian_eigenvalues": initialization.hessian_eigenvalues,
        "hessian_condition_number": initialization.hessian_condition_number,
        "lidars": {
            name: {
                "accepted": item.accepted,
                "reason": item.reason,
                "rotation": rotation_values(item.rotation),
                "vehicle_planar_axis": item.vehicle_axis.tolist(),
                "lidar_planar_axis": item.lidar_axis.tolist(),
                "yaw_about_vehicle_axis_deg": item.yaw_about_vehicle_axis_deg,
                "axis_samples": item.axis_samples,
                "axis_inliers": item.axis_inliers,
                "axis_p95_deg": item.axis_p95_deg,
                "train": item.train_metrics,
                "heldout": item.heldout_metrics,
            }
            for name, item in initialization.lidars.items()
        },
    }


def relative_extrinsics_document(
    lidars: Sequence[LidarDefinition],
    calibrations: dict[str, RotationCalibration],
    reference_name: str,
    score_against_manifest: bool = True,
) -> tuple[dict[str, Any], dict[str, Any]]:
    definitions = {lidar.name: lidar for lidar in lidars}
    reference = calibrations[reference_name]
    optimized_reference = make_transform(
        reference.optimized_rotation, reference.fixed_translation
    )
    trusted_reference = None
    if score_against_manifest:
        reference_rotation = definitions[reference_name].trusted_rotation
        if reference_rotation is None:
            raise ValueError("reference LiDAR has no manifest rotation for scoring")
        trusted_reference = make_transform(
            reference_rotation,
            definitions[reference_name].translation,
        )
    output: dict[str, Any] = {}
    pair_errors: list[dict[str, Any]] = []
    for lidar in lidars:
        calibration = calibrations[lidar.name]
        optimized = inverse(optimized_reference) @ make_transform(
            calibration.optimized_rotation, calibration.fixed_translation
        )
        output[lidar.name] = {
            "reference_T_lidar": [
                *[float(value) for value in optimized[:3, 3]],
                *[
                    float(value)
                    for value in Rotation.from_matrix(
                        optimized[:3, :3]
                    ).as_quat()
                ],
            ],
        }
        if score_against_manifest:
            if lidar.trusted_rotation is None or trusted_reference is None:
                raise ValueError(f"{lidar.name} has no manifest rotation for scoring")
            trusted = inverse(trusted_reference) @ make_transform(
                lidar.trusted_rotation, lidar.translation
            )
            output[lidar.name]["rotation_difference_from_manifest_deg"] = (
                angular_distance_degrees(trusted[:3, :3], optimized[:3, :3])
            )
    for first_index, first in enumerate(lidars):
        for second in lidars[first_index + 1 :]:
            first_calibration = calibrations[first.name]
            second_calibration = calibrations[second.name]
            optimized_relative_rotation = (
                first_calibration.optimized_rotation.T
                @ second_calibration.optimized_rotation
            )
            if score_against_manifest:
                if (
                    first.trusted_rotation is None
                    or second.trusted_rotation is None
                ):
                    raise ValueError(
                        "all LiDARs need manifest rotations for relative scoring"
                    )
                trusted_relative_rotation = (
                    first.trusted_rotation.T @ second.trusted_rotation
                )
                pair_errors.append(
                    {
                        "first": first.name,
                        "second": second.name,
                        "rotation_error_deg": angular_distance_degrees(
                            trusted_relative_rotation, optimized_relative_rotation
                        ),
                    }
                )
    errors = np.asarray([item["rotation_error_deg"] for item in pair_errors])
    metrics = {
        "scored": score_against_manifest,
        "pairs": pair_errors,
        "mean_rotation_error_deg": float(np.mean(errors)) if errors.size else 0.0,
        "p95_rotation_error_deg": (
            float(np.percentile(errors, 95)) if errors.size else 0.0
        ),
        "maximum_rotation_error_deg": float(np.max(errors)) if errors.size else 0.0,
    }
    return output, metrics


def write_constraints(path: Path, calibrations: Iterable[RotationCalibration]) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "lidar",
                "first_frame",
                "second_frame",
                "vehicle_translation_m",
                "vehicle_rotation_deg",
                "icp_fitness",
                "icp_inlier_rmse_m",
                "icp_translation_update_m",
                "icp_rotation_update_deg",
                "registration_mode",
                "motion_angle_error_deg",
                "axis_error_deg",
                "accepted",
                "split",
                "rejection_reason",
            ]
        )
        for calibration in calibrations:
            for constraint in calibration.constraints:
                candidate = constraint.candidate
                writer.writerow(
                    [
                        calibration.name,
                        candidate.first,
                        candidate.second,
                        candidate.translation_m,
                        candidate.rotation_deg,
                        constraint.fitness,
                        constraint.inlier_rmse_m,
                        constraint.icp_translation_update_m,
                        constraint.icp_rotation_update_deg,
                        constraint.registration_mode,
                        constraint.motion_angle_error_deg,
                        constraint.axis_error_deg,
                        str(constraint.accepted).lower(),
                        constraint.split,
                        constraint.rejection_reason,
                    ]
                )


def write_trajectory(
    path: Path,
    reference: LidarDefinition,
    calibration: RotationCalibration,
    navigation: NavigationTrajectory,
    gnss_lever_arm_vehicle: np.ndarray,
) -> None:
    files, timestamps = scan_index(reference.directory)
    del files
    valid = np.flatnonzero(
        (timestamps >= navigation.timestamps[0])
        & (timestamps <= navigation.timestamps[-1])
    )
    poses = navigation.poses(timestamps[valid], gnss_lever_arm_vehicle)
    extrinsic = make_transform(
        calibration.optimized_rotation, calibration.fixed_translation
    )
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "frame",
                "timestamp",
                "vehicle_tx",
                "vehicle_ty",
                "vehicle_tz",
                "vehicle_qx",
                "vehicle_qy",
                "vehicle_qz",
                "vehicle_qw",
                "reference_tx",
                "reference_ty",
                "reference_tz",
                "reference_qx",
                "reference_qy",
                "reference_qz",
                "reference_qw",
            ]
        )
        for index, timestamp, world_t_vehicle in zip(valid, timestamps[valid], poses):
            reference_pose = world_t_vehicle @ extrinsic
            vehicle_quaternion = Rotation.from_matrix(
                world_t_vehicle[:3, :3]
            ).as_quat()
            reference_quaternion = Rotation.from_matrix(reference_pose[:3, :3]).as_quat()
            writer.writerow(
                [
                    int(index),
                    float(timestamp),
                    *world_t_vehicle[:3, 3].tolist(),
                    *vehicle_quaternion.tolist(),
                    *reference_pose[:3, 3].tolist(),
                    *reference_quaternion.tolist(),
                ]
            )


def write_corrected_manifest(
    path: Path,
    source: dict[str, Any],
    calibrations: dict[str, RotationCalibration],
    output_dir: Path,
) -> None:
    document = copy.deepcopy(source)
    document["output_root"] = str((output_dir / "mloam_fixed").resolve())
    backend = document.get("joint_backend")
    if not isinstance(backend, dict):
        backend = {}
        document["joint_backend"] = backend
    backend["enabled"] = False
    backend["mode"] = "disabled"
    backend["optimize_extrinsic_translation"] = False
    for lidar in document.get("lidars", []):
        name = str(lidar["name"])
        if name not in calibrations:
            continue
        calibration = calibrations[name]
        lidar.pop("extrinsic_prototxt", None)
        lidar["vehicle_T_lidar"] = {
            "translation": [float(value) for value in calibration.fixed_translation],
            "rpy_deg": [
                float(value)
                for value in Rotation.from_matrix(
                    calibration.optimized_rotation
                ).as_euler("xyz", degrees=True)
            ],
        }
    path.write_text(yaml.safe_dump(document, sort_keys=False))


def safe_yaml_value(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): safe_yaml_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_yaml_value(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def main(argv: Sequence[str] | None = None) -> int:
    options = arguments(argv)
    if options.inject_rotation_error_deg < 0.0:
        raise ValueError("injected rotation error must be non-negative")
    if (
        options.rotation_initialization == "prior-free"
        and options.inject_rotation_error_deg > 0.0
    ):
        raise ValueError(
            "--inject-rotation-error-deg is a manifest-initialization test; "
            "it cannot be combined with --rotation-initialization prior-free"
        )
    if options.gnss_lever_arm_bound_m <= 0.0:
        raise ValueError("GNSS lever-arm bound must be positive")
    if options.lever_arm_outer_iterations <= 0:
        raise ValueError("lever-arm outer iterations must be positive")
    if options.prior_free_voxel_size <= 0.0:
        raise ValueError("prior-free voxel size must be positive")
    if options.prior_free_coarse_distance <= options.prior_free_voxel_size:
        raise ValueError("prior-free coarse distance must exceed its voxel size")
    manifest_path = options.manifest.resolve()
    manifest = yaml.safe_load(manifest_path.read_text())
    include = set(options.include.split(",")) if options.include else None
    load_manifest_rotations = bool(
        options.rotation_initialization == "manifest"
        or options.score_against_manifest
        or options.inject_rotation_error_deg > 0.0
    )
    lidars = load_lidars(
        manifest_path,
        manifest,
        include,
        load_manifest_rotations=load_manifest_rotations,
    )
    dataset_root = resolve_path(manifest_path.parent, str(manifest["dataset_root"]))
    imu_file = options.imu_file or dataset_root / "imu.txt"
    gnss_dir = options.gnss_dir or dataset_root / "gnss"
    output_dir = options.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print(
        f"Loading raw navigation from IMU {imu_file} and GNSS {gnss_dir}",
        flush=True,
    )
    navigation = load_raw_navigation(
        imu_file, gnss_dir, options.gnss_heading_to_vehicle_yaw_deg
    )
    print(
        f"Excitation: {navigation.metrics['path_length_m']:.1f} m, "
        f"{navigation.metrics['yaw_total_deg']:.1f} deg accumulated yaw, "
        f"heading-to-vehicle={navigation.metrics['heading_to_vehicle_yaw_deg']:.2f} deg",
        flush=True,
    )

    initial_rotations: dict[str, np.ndarray] = {}
    constraints_by_lidar: dict[str, list[PairConstraint]] = {}
    for lidar in lidars:
        print(f"Measuring motion constraints for {lidar.name}", flush=True)
        files, timestamps = scan_index(lidar.directory)
        pairs = select_pairs(
            timestamps,
            navigation,
            options.pair_frame_gap,
            options.pair_candidate_stride,
            options.pair_count,
        )
        if options.rotation_initialization == "prior-free":
            constraints = collect_prior_free_constraints(
                lidar,
                files,
                pairs,
                options.prior_free_voxel_size,
                options.minimum_range,
                options.maximum_range,
                options.prior_free_coarse_distance,
                options.prior_free_fallback_distance,
                options.prior_free_motion_angle_tolerance_deg,
            )
        else:
            if lidar.trusted_rotation is None:
                raise ValueError(
                    f"{lidar.name} has no manifest rotation for manifest initialization"
                )
            initial_rotation = perturbed_rotation(
                lidar.trusted_rotation,
                options.inject_rotation_error_deg,
                options.seed,
                lidar.name,
            )
            initial_rotations[lidar.name] = initial_rotation
            constraints = collect_constraints(
                lidar,
                files,
                pairs,
                initial_rotation,
                options.calibration_voxel_size,
                options.minimum_range,
                options.maximum_range,
                np.zeros(3),
            )
        constraints_by_lidar[lidar.name] = constraints

    prior_free_initialization: PriorFreeInitialization | None = None
    if options.rotation_initialization == "prior-free":
        print("Solving rotation-prior-free planar axis and yaw initializer", flush=True)
        prior_free_initialization = estimate_prior_free_initialization(
            lidars,
            constraints_by_lidar,
            options.minimum_constraints,
            options.gnss_lever_arm_bound_m,
            options.prior_free_axis_inlier_deg,
        )
        initial_rotations = {
            name: item.rotation.copy()
            for name, item in prior_free_initialization.lidars.items()
        }
        gnss_lever_arm = prior_free_initialization.gnss_lever_arm.copy()
        print(
            f"Prior-free initializer accepted={prior_free_initialization.accepted}; "
            f"lever_arm={gnss_lever_arm.tolist()}; "
            f"{prior_free_initialization.reason}",
            flush=True,
        )
        for name, item in prior_free_initialization.lidars.items():
            print(
                f"  {name}: accepted={item.accepted}, "
                f"axis_p95={item.axis_p95_deg:.2f}deg, "
                "heldout_translation="
                f"{item.heldout_metrics.get('translation_rmse_m', math.inf):.3f}m; "
                f"{item.reason}",
                flush=True,
            )
    else:
        gnss_lever_arm = np.zeros(3)
    lever_estimate = NavigationLeverArmEstimate(
        value=gnss_lever_arm.copy(), reason="not solved"
    )
    score_rotations = (
        options.score_against_manifest or options.inject_rotation_error_deg > 0.0
    )
    calibrations: dict[str, RotationCalibration] = {}
    for outer_iteration in range(options.lever_arm_outer_iterations):
        calibrations = {
            lidar.name: calibrate_rotation(
                lidar,
                initial_rotations[lidar.name],
                constraints_by_lidar[lidar.name],
                options.minimum_constraints,
                options.rotation_update_bound_deg,
                lidar.trusted_rotation if score_rotations else None,
                gnss_lever_arm,
                options.rotation_initialization != "prior-free",
            )
            for lidar in lidars
        }
        lever_estimate = estimate_gnss_lever_arm(
            lidars,
            calibrations,
            gnss_lever_arm,
            options.gnss_lever_arm_bound_m,
        )
        print(
            f"GNSS lever-arm iteration {outer_iteration + 1}: "
            f"value={lever_estimate.value.tolist()}, "
            f"accepted={lever_estimate.accepted}; {lever_estimate.reason}",
            flush=True,
        )
        if not lever_estimate.accepted:
            break
        change = float(np.linalg.norm(lever_estimate.value - gnss_lever_arm))
        gnss_lever_arm = lever_estimate.value.copy()
        if change < 1.0e-4:
            break

    calibrations = {
        lidar.name: calibrate_rotation(
            lidar,
            initial_rotations[lidar.name],
            constraints_by_lidar[lidar.name],
            options.minimum_constraints,
            options.rotation_update_bound_deg,
            lidar.trusted_rotation if score_rotations else None,
            gnss_lever_arm,
            options.rotation_initialization != "prior-free",
        )
        for lidar in lidars
    }
    for lidar in lidars:
        calibration = calibrations[lidar.name]
        error_text = (
            f", trusted error={calibration.trusted_final_error_deg:.3f} deg"
            if calibration.trusted_final_error_deg is not None
            else ""
        )
        print(
            f"  {lidar.name}: accepted={calibration.accepted}, "
            f"update={calibration.update_deg:.3f} deg{error_text}; "
            f"{calibration.reason}",
            flush=True,
        )

    reference_name = str(manifest["reference_lidar"])
    if reference_name not in calibrations:
        raise ValueError("selected LiDARs must include the manifest reference LiDAR")
    initializer_accepted = (
        prior_free_initialization is None or prior_free_initialization.accepted
    )
    all_accepted = initializer_accepted and lever_estimate.accepted and all(
        calibration.accepted for calibration in calibrations.values()
    )
    write_constraints(output_dir / "pair_constraints.csv", calibrations.values())
    write_trajectory(
        output_dir / "trajectory_navigation.csv",
        next(lidar for lidar in lidars if lidar.name == reference_name),
        calibrations[reference_name],
        navigation,
        gnss_lever_arm,
    )
    if all_accepted:
        write_corrected_manifest(
            output_dir / "corrected_manifest.yaml",
            manifest,
            calibrations,
            output_dir,
        )
    relative_extrinsics, relative_metrics = relative_extrinsics_document(
        lidars,
        calibrations,
        reference_name,
        score_against_manifest=(
            options.rotation_initialization == "manifest" or score_rotations
        ),
    )
    (output_dir / "relative_extrinsics.yaml").write_text(
        yaml.safe_dump(safe_yaml_value(relative_extrinsics), sort_keys=False)
    )

    map_metrics: dict[str, Any] = {"built": False}
    if not options.skip_map and all_accepted:
        print("Building GNSS-anchored initial and optimized maps", flush=True)
        map_metrics = {
            "built": True,
            **build_maps(
                lidars,
                calibrations,
                navigation,
                gnss_lever_arm,
                options.map_stride,
                options.map_voxel_size,
                options.minimum_range,
                options.maximum_range,
                output_dir,
            ),
        }
    elif not options.skip_map:
        print("Map build skipped because at least one calibration was rejected", flush=True)

    summary = {
        "status": "success" if all_accepted else "non_converged",
        "algorithm": (
            "rotation_prior_free_fixed_translation_raw_imu_gnss_handeye_icp"
            if options.rotation_initialization == "prior-free"
            else "fixed_translation_raw_imu_gnss_handeye_icp"
        ),
        "manifest": str(manifest_path),
        "dataset_root": str(dataset_root),
        "reference_lidar": reference_name,
        "seed": options.seed,
        "rotation_initialization": options.rotation_initialization,
        "manifest_rotations_used_for_initialization": (
            options.rotation_initialization == "manifest"
        ),
        "score_against_manifest_after_optimization": score_rotations,
        "injected_rotation_error_deg": options.inject_rotation_error_deg,
        "translation_optimized": False,
        "navigation": navigation.metrics,
        "prior_free_initialization": prior_free_initialization_document(
            prior_free_initialization
        ),
        "gnss_antenna_lever_arm": {
            "accepted": lever_estimate.accepted,
            "reason": lever_estimate.reason,
            "vehicle_translation_m": gnss_lever_arm.tolist(),
            "vertical_component_fixed": True,
            "update_m": lever_estimate.update_m,
            "hessian_eigenvalues": lever_estimate.hessian_eigenvalues,
            "hessian_condition_number": lever_estimate.hessian_condition_number,
            "heldout_initial_translation_rmse_m": (
                lever_estimate.heldout_initial_translation_rmse_m
            ),
            "heldout_final_translation_rmse_m": (
                lever_estimate.heldout_final_translation_rmse_m
            ),
        },
        "calibrations": {
            name: calibration_document(calibration)
            for name, calibration in calibrations.items()
        },
        "relative_lidar_rotation_vs_manifest": relative_metrics,
        "map": map_metrics,
    }
    (output_dir / "summary.yaml").write_text(
        yaml.safe_dump(safe_yaml_value(summary), sort_keys=False)
    )
    print(f"Wrote results to {output_dir}", flush=True)
    return 0 if all_accepted else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:  # keep CLI failures concise and actionable
        print(f"planar_nav_rotation_calibrator: {error}", file=sys.stderr)
        raise SystemExit(1)
