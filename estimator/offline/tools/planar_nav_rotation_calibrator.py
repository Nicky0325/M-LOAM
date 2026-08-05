#!/usr/bin/env python3
"""Navigation-aided, fixed-translation rotation calibration for planar rigs.

The estimator deliberately solves a smaller problem than M-LOAM's original
online 6-DoF calibration.  Vehicle-frame LiDAR translations are treated as
known constants.  Dense vehicle poses supply relative rig motion, point-to-
plane ICP measures the corresponding relative motion of each LiDAR, and a
robust hand-eye solve estimates only the three rotational degrees of freedom::

    B_ij = X^-1 A_ij X

Here ``A_ij`` is vehicle motion, ``B_ij`` is LiDAR motion, and ``X`` is the
vehicle_T_lidar transform whose translation parameter is never changed.

For the AIV5 extraction, the smooth DR pose stream is rigidly anchored to the
GNSS/INS ENU trajectory.  The same globally anchored trajectory and accepted
extrinsics can then be used to build a colored, globally consistent map.  An
optional deterministic perturbation mode is intended for recovery-basin
validation; it does not expose the trusted rotations to the optimizer.
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
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class LidarDefinition:
    name: str
    directory: Path
    translation: np.ndarray
    trusted_rotation: np.ndarray
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


@dataclass
class RotationCalibration:
    name: str
    initial_rotation: np.ndarray
    optimized_rotation: np.ndarray
    fixed_translation: np.ndarray
    constraints: list[PairConstraint] = field(default_factory=list)
    accepted: bool = False
    reason: str = ""
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
class NavigationTrajectory:
    timestamps: np.ndarray
    local_positions: np.ndarray
    local_quaternions: np.ndarray
    global_positions: np.ndarray
    global_quaternions: np.ndarray
    metrics: dict[str, Any]

    def poses(self, timestamps: np.ndarray, *, global_frame: bool) -> np.ndarray:
        if timestamps.size == 0:
            return np.empty((0, 4, 4))
        if timestamps.min() < self.timestamps[0] or timestamps.max() > self.timestamps[-1]:
            raise ValueError("navigation interpolation requested outside pose coverage")
        positions = self.global_positions if global_frame else self.local_positions
        quaternions = self.global_quaternions if global_frame else self.local_quaternions
        interpolated_positions = np.column_stack(
            [np.interp(timestamps, self.timestamps, positions[:, axis]) for axis in range(3)]
        )
        interpolated_rotations = Slerp(
            self.timestamps, Rotation.from_quat(quaternions)
        )(timestamps).as_matrix()
        return np.asarray(
            [make_transform(rotation, translation)
             for rotation, translation in zip(interpolated_rotations, interpolated_positions)]
        )


def arguments(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--pose-dir", type=Path)
    parser.add_argument("--ins-file", type=Path)
    parser.add_argument(
        "--inject-rotation-error-deg",
        type=float,
        default=0.0,
        help="deterministically perturb every input rotation for validation",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--pair-count", type=int, default=40)
    parser.add_argument("--pair-frame-gap", type=int, default=15)
    parser.add_argument("--pair-candidate-stride", type=int, default=5)
    parser.add_argument("--calibration-voxel-size", type=float, default=0.45)
    parser.add_argument("--minimum-range", type=float, default=2.0)
    parser.add_argument("--maximum-range", type=float, default=80.0)
    parser.add_argument("--minimum-constraints", type=int, default=12)
    parser.add_argument("--rotation-update-bound-deg", type=float, default=15.0)
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
    manifest_path: Path, document: dict[str, Any], include: set[str] | None
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
            rotation = Rotation.from_euler(
                "xyz", np.asarray(transform["rpy_deg"], dtype=float), degrees=True
            ).as_matrix()
        elif "extrinsic_prototxt" in node:
            calibration_path = resolve_path(
                manifest_path.parent, str(node["extrinsic_prototxt"])
            )
            rotation, translation = prototxt_calibration(calibration_path)
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


def vehicle_pose_records(pose_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    records: list[tuple[float, np.ndarray, np.ndarray, str]] = []
    for path in pose_dir.glob("*.prototxt"):
        text = path.read_text()
        time_match = re.search(r"\btime_meas\s*:\s*(\d+)", text)
        timestamp = int(time_match.group(1)) * 1.0e-9 if time_match else float(path.stem)
        position = block_vector(text, "pos")
        rpy = block_vector(text, "attitude_rpy")
        status_match = re.search(r"^status\s*:\s*(\S+)", text, re.M)
        records.append(
            (timestamp, position, rpy, status_match.group(1) if status_match else "UNKNOWN")
        )
    if not records:
        raise ValueError(f"no vehicle poses found in {pose_dir}")
    records.sort(key=lambda item: item[0])
    timestamps = np.asarray([item[0] for item in records])
    unique = np.concatenate(([True], np.diff(timestamps) > 0.0))
    positions = np.asarray([item[1] for item in records])[unique]
    quaternions = Rotation.from_euler(
        "xyz", np.asarray([item[2] for item in records])[unique]
    ).as_quat()
    statuses = [item[3] for item, keep in zip(records, unique) if keep]
    return timestamps[unique], positions, quaternions, statuses


def rigid_alignment_2d(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - source_center).T @ (target - target_center))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def load_navigation(pose_dir: Path, ins_file: Path | None) -> NavigationTrajectory:
    timestamps, positions, quaternions, statuses = vehicle_pose_records(pose_dir)
    local_rotations = Rotation.from_quat(quaternions)
    relative = local_rotations[:-1].inv() * local_rotations[1:]
    dt = np.diff(timestamps)
    step = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    rpy_deg = local_rotations.as_euler("xyz", degrees=True)
    unwrapped_yaw = np.unwrap(np.deg2rad(rpy_deg[:, 2]))
    metrics: dict[str, Any] = {
        "pose_samples": int(timestamps.size),
        "duration_s": float(timestamps[-1] - timestamps[0]),
        "path_length_m": float(step.sum()),
        "position_span_m": np.ptp(positions, axis=0).tolist(),
        "rpy_span_deg": np.ptp(rpy_deg, axis=0).tolist(),
        "yaw_net_deg": float(np.rad2deg(unwrapped_yaw[-1] - unwrapped_yaw[0])),
        "yaw_total_deg": float(np.rad2deg(np.abs(np.diff(unwrapped_yaw)).sum())),
        "maximum_angular_rate_deg_s": float(np.max(np.rad2deg(relative.magnitude()) / dt)),
        "status_counts": {status: statuses.count(status) for status in sorted(set(statuses))},
        "gnss_anchor_available": False,
    }

    global_positions = positions.copy()
    global_quaternions = quaternions.copy()
    if ins_file is not None and ins_file.is_file():
        ins = np.genfromtxt(ins_file, delimiter=",", skip_header=1)
        ins = np.atleast_2d(ins)
        finite = np.all(np.isfinite(ins[:, :7]), axis=1)
        ins = ins[finite]
        if ins.shape[0] >= 2:
            earth_radius_m = 6378137.0
            latitude0_rad = math.radians(float(ins[0, 1]))
            enu = np.column_stack(
                (
                    np.deg2rad(ins[:, 2] - ins[0, 2])
                    * earth_radius_m
                    * math.cos(latitude0_rad),
                    np.deg2rad(ins[:, 1] - ins[0, 1]) * earth_radius_m,
                    ins[:, 3] - ins[0, 3],
                )
            )
            common = (timestamps >= ins[0, 0]) & (timestamps <= ins[-1, 0])
            if np.count_nonzero(common) >= 10:
                interpolated_enu = np.column_stack(
                    [np.interp(timestamps[common], ins[:, 0], enu[:, axis]) for axis in range(3)]
                )
                rotation2, translation2 = rigid_alignment_2d(
                    positions[common, :2], interpolated_enu[:, :2]
                )
                global_positions[:, :2] = (
                    rotation2 @ positions[:, :2].T
                ).T + translation2
                z_offset = float(np.median(interpolated_enu[:, 2] - positions[common, 2]))
                global_positions[:, 2] = positions[:, 2] + z_offset
                yaw = math.atan2(rotation2[1, 0], rotation2[0, 0])
                world_rotation = Rotation.from_euler("z", yaw)
                global_quaternions = (world_rotation * local_rotations).as_quat()
                aligned_error = np.linalg.norm(
                    global_positions[common, :2] - interpolated_enu[:, :2], axis=1
                )
                metrics.update(
                    {
                        "gnss_anchor_available": True,
                        "ins_samples": int(ins.shape[0]),
                        "ins_status_counts": {
                            str(int(value)): int(count)
                            for value, count in zip(*np.unique(ins[:, -1], return_counts=True))
                        },
                        "dr_to_ins_yaw_deg": math.degrees(yaw),
                        "dr_to_ins_position_rmse_m": float(
                            np.sqrt(np.mean(np.square(aligned_error)))
                        ),
                        "dr_to_ins_position_p95_m": float(np.percentile(aligned_error, 95)),
                        "dr_to_ins_position_max_m": float(np.max(aligned_error)),
                    }
                )
    return NavigationTrajectory(
        timestamps=timestamps,
        local_positions=positions,
        local_quaternions=quaternions,
        global_positions=global_positions,
        global_quaternions=global_quaternions,
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
    poses = navigation.poses(scan_times[valid], global_frame=False)
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


def collect_constraints(
    lidar: LidarDefinition,
    files: list[Path],
    pairs: list[PairCandidate],
    initial_rotation: np.ndarray,
    voxel_size: float,
    minimum_range: float,
    maximum_range: float,
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
        predicted = initial_inverse @ candidate.vehicle_motion @ initial_extrinsic
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
) -> np.ndarray:
    rotation = initial_rotation @ Rotation.from_rotvec(delta).as_matrix()
    extrinsic = make_transform(rotation, fixed_translation)
    extrinsic_inverse = inverse(extrinsic)
    residuals: list[float] = []
    for constraint in constraints:
        predicted = (
            extrinsic_inverse @ constraint.candidate.vehicle_motion @ extrinsic
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
        predicted = (
            extrinsic_inverse @ constraint.candidate.vehicle_motion @ extrinsic
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
) -> RotationCalibration:
    result = RotationCalibration(
        name=lidar.name,
        initial_rotation=initial_rotation,
        optimized_rotation=initial_rotation.copy(),
        fixed_translation=lidar.translation.copy(),
        constraints=constraints,
    )
    usable = [constraint for constraint in constraints if constraint.accepted]
    for index, constraint in enumerate(usable):
        constraint.split = "heldout" if index % 5 == 0 else "train"
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
        args=(initial_rotation, lidar.translation, training),
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
        initial_rotation, lidar.translation, training
    )
    result.train_final = constraint_metrics(
        optimized_rotation, lidar.translation, training
    )
    result.heldout_initial = constraint_metrics(
        initial_rotation, lidar.translation, heldout
    )
    result.heldout_final = constraint_metrics(
        optimized_rotation, lidar.translation, heldout
    )
    if trusted_rotation is not None:
        result.trusted_final_error_deg = angular_distance_degrees(
            trusted_rotation, optimized_rotation
        )

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
    heldout_rotation_not_regressed = (
        result.heldout_final["rotation_rmse_deg"]
        <= 1.05 * result.heldout_initial["rotation_rmse_deg"]
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
        and heldout_translation_improved
        and heldout_rotation_not_regressed
    )
    if not solution.success:
        result.reason = f"nonlinear solve failed: {solution.message}"
    elif not observable:
        result.reason = "rotation Hessian is rank-deficient or ill-conditioned"
    elif not bounded:
        result.reason = "rotation update exceeded configured bound"
    elif not heldout_translation_improved:
        result.reason = "held-out translation consistency did not improve"
    elif not heldout_rotation_not_regressed:
        result.reason = "held-out rotation consistency regressed"
    else:
        result.reason = "accepted: fixed-translation hand-eye validation passed"
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
        poses = navigation.poses(timestamps[valid], global_frame=True)
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


def relative_extrinsics_document(
    lidars: Sequence[LidarDefinition],
    calibrations: dict[str, RotationCalibration],
    reference_name: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    definitions = {lidar.name: lidar for lidar in lidars}
    reference = calibrations[reference_name]
    optimized_reference = make_transform(
        reference.optimized_rotation, reference.fixed_translation
    )
    trusted_reference = make_transform(
        definitions[reference_name].trusted_rotation,
        definitions[reference_name].translation,
    )
    output: dict[str, Any] = {}
    pair_errors: list[dict[str, Any]] = []
    for lidar in lidars:
        calibration = calibrations[lidar.name]
        optimized = inverse(optimized_reference) @ make_transform(
            calibration.optimized_rotation, calibration.fixed_translation
        )
        trusted = inverse(trusted_reference) @ make_transform(
            lidar.trusted_rotation, lidar.translation
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
            "rotation_difference_from_manifest_deg": angular_distance_degrees(
                trusted[:3, :3], optimized[:3, :3]
            ),
        }
    for first_index, first in enumerate(lidars):
        for second in lidars[first_index + 1 :]:
            first_calibration = calibrations[first.name]
            second_calibration = calibrations[second.name]
            optimized_relative_rotation = (
                first_calibration.optimized_rotation.T
                @ second_calibration.optimized_rotation
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
        "pairs": pair_errors,
        "mean_rotation_error_deg": float(np.mean(errors)),
        "p95_rotation_error_deg": float(np.percentile(errors, 95)),
        "maximum_rotation_error_deg": float(np.max(errors)),
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
                "accepted",
                "split",
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
                        str(constraint.accepted).lower(),
                        constraint.split,
                    ]
                )


def write_trajectory(
    path: Path,
    reference: LidarDefinition,
    calibration: RotationCalibration,
    navigation: NavigationTrajectory,
) -> None:
    files, timestamps = scan_index(reference.directory)
    del files
    valid = np.flatnonzero(
        (timestamps >= navigation.timestamps[0])
        & (timestamps <= navigation.timestamps[-1])
    )
    poses = navigation.poses(timestamps[valid], global_frame=True)
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
        for index, timestamp, vehicle_pose in zip(valid, timestamps[valid], poses):
            reference_pose = vehicle_pose @ extrinsic
            vehicle_quaternion = Rotation.from_matrix(vehicle_pose[:3, :3]).as_quat()
            reference_quaternion = Rotation.from_matrix(reference_pose[:3, :3]).as_quat()
            writer.writerow(
                [
                    int(index),
                    float(timestamp),
                    *vehicle_pose[:3, 3].tolist(),
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
    if isinstance(backend, dict):
        backend["enabled"] = False
        backend["mode"] = "disabled"
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
    manifest_path = options.manifest.resolve()
    manifest = yaml.safe_load(manifest_path.read_text())
    include = set(options.include.split(",")) if options.include else None
    lidars = load_lidars(manifest_path, manifest, include)
    dataset_root = resolve_path(manifest_path.parent, str(manifest["dataset_root"]))
    pose_dir = options.pose_dir or dataset_root / "vehicle_pose"
    ins_file = options.ins_file or dataset_root / "ins.txt"
    output_dir = options.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading and GNSS-anchoring navigation from {pose_dir}", flush=True)
    navigation = load_navigation(pose_dir, ins_file)
    print(
        f"Excitation: {navigation.metrics['path_length_m']:.1f} m, "
        f"{navigation.metrics['yaw_total_deg']:.1f} deg accumulated yaw, "
        f"GNSS anchor={navigation.metrics['gnss_anchor_available']}",
        flush=True,
    )

    calibrations: dict[str, RotationCalibration] = {}
    for lidar in lidars:
        print(f"Calibrating {lidar.name}", flush=True)
        files, timestamps = scan_index(lidar.directory)
        pairs = select_pairs(
            timestamps,
            navigation,
            options.pair_frame_gap,
            options.pair_candidate_stride,
            options.pair_count,
        )
        initial_rotation = perturbed_rotation(
            lidar.trusted_rotation,
            options.inject_rotation_error_deg,
            options.seed,
            lidar.name,
        )
        constraints = collect_constraints(
            lidar,
            files,
            pairs,
            initial_rotation,
            options.calibration_voxel_size,
            options.minimum_range,
            options.maximum_range,
        )
        calibration = calibrate_rotation(
            lidar,
            initial_rotation,
            constraints,
            options.minimum_constraints,
            options.rotation_update_bound_deg,
            lidar.trusted_rotation
            if options.inject_rotation_error_deg > 0.0
            else None,
        )
        calibrations[lidar.name] = calibration
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
    all_accepted = all(calibration.accepted for calibration in calibrations.values())
    write_constraints(output_dir / "pair_constraints.csv", calibrations.values())
    write_trajectory(
        output_dir / "trajectory_navigation.csv",
        next(lidar for lidar in lidars if lidar.name == reference_name),
        calibrations[reference_name],
        navigation,
    )
    if all_accepted:
        write_corrected_manifest(
            output_dir / "corrected_manifest.yaml",
            manifest,
            calibrations,
            output_dir,
        )
    relative_extrinsics, relative_metrics = relative_extrinsics_document(
        lidars, calibrations, reference_name
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
        "algorithm": "fixed_translation_navigation_aided_handeye_icp",
        "manifest": str(manifest_path),
        "dataset_root": str(dataset_root),
        "reference_lidar": reference_name,
        "seed": options.seed,
        "injected_rotation_error_deg": options.inject_rotation_error_deg,
        "translation_optimized": False,
        "navigation": navigation.metrics,
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
