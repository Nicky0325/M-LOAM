#!/usr/bin/env python3
"""Evaluate an AIV5 M-LOAM offline session against vehicle_pose records."""

from __future__ import annotations

import argparse
import csv
import math
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--session-root", required=True, type=Path)
    parser.add_argument("--pose-dir", type=Path)
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=["precise", "calibrated_init", "prior_free"],
    )
    return parser.parse_args()


def block_vector(text: str, block_name: str) -> np.ndarray:
    match = re.search(rf"\b{re.escape(block_name)}\s*\{{([^}}]*)\}}", text, re.S)
    if not match:
        raise ValueError(f"missing {block_name} block")
    body = match.group(1)
    values = []
    for axis in "xyz":
        field = re.search(
            rf"\b{axis}\s*:\s*([-+0-9.eE]+)", body
        )
        if not field:
            raise ValueError(f"missing {block_name}.{axis}")
        values.append(float(field.group(1)))
    return np.asarray(values, dtype=float)


def calibration_transform(path: Path) -> tuple[np.ndarray, np.ndarray]:
    text = path.read_text()
    translation = block_vector(text, "translation")
    rpy_deg = block_vector(text, "rotation_rpy_deg")
    rotation = Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    return rotation, translation


def manifest_calibration(
    manifest_path: Path, lidar: dict[str, Any]
) -> tuple[np.ndarray, np.ndarray]:
    if "vehicle_T_lidar" in lidar:
        transform = lidar["vehicle_T_lidar"]
        translation = np.asarray(transform["translation"], dtype=float)
        rotation = Rotation.from_euler(
            "xyz", np.asarray(transform["rpy_deg"], dtype=float), degrees=True
        ).as_matrix()
        return rotation, translation
    if "extrinsic_prototxt" not in lidar:
        raise ValueError(f"LiDAR {lidar.get('name', '<unnamed>')} has no extrinsic")
    path = Path(lidar["extrinsic_prototxt"]).expanduser()
    if not path.is_absolute():
        path = manifest_path.parent / path
    return calibration_transform(path.resolve())


def pose_records(pose_dir: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    records: list[tuple[float, np.ndarray, np.ndarray, str]] = []
    for path in sorted(pose_dir.glob("*.prototxt")):
        text = path.read_text()
        time_match = re.search(r"\btime_meas\s*:\s*(\d+)", text)
        if time_match:
            timestamp = int(time_match.group(1)) * 1.0e-9
        else:
            timestamp = float(path.stem)
        position = block_vector(text, "pos")
        rpy = block_vector(text, "attitude_rpy")
        status_match = re.search(r"^status\s*:\s*(\S+)", text, re.M)
        status = status_match.group(1) if status_match else "UNKNOWN"
        records.append((timestamp, position, rpy, status))
    if not records:
        raise ValueError(f"no pose prototxt files in {pose_dir}")
    records.sort(key=lambda item: item[0])
    timestamps = np.asarray([item[0] for item in records])
    keep = np.concatenate(([True], np.diff(timestamps) > 0.0))
    positions = np.asarray([item[1] for item in records])[keep]
    rotations = Rotation.from_euler(
        "xyz", np.asarray([item[2] for item in records])[keep]
    ).as_quat()
    statuses = [item[3] for item, selected in zip(records, keep) if selected]
    return timestamps[keep], positions, rotations, statuses


def csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def estimate_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rows = csv_rows(path)
    if not rows:
        raise ValueError(f"empty trajectory: {path}")
    frames = np.asarray([int(row["frame"]) for row in rows], dtype=int)
    timestamps = np.asarray([float(row["timestamp"]) for row in rows])
    positions = np.asarray(
        [[float(row[name]) for name in ("tx", "ty", "tz")] for row in rows]
    )
    quaternions = np.asarray(
        [[float(row[name]) for name in ("qx", "qy", "qz", "qw")] for row in rows]
    )
    return frames, timestamps, positions, quaternions


def interpolate_ground_truth(
    sample_times: np.ndarray,
    gt_times: np.ndarray,
    gt_positions: np.ndarray,
    gt_quaternions: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    positions = np.column_stack(
        [np.interp(sample_times, gt_times, gt_positions[:, axis]) for axis in range(3)]
    )
    rotations = Slerp(gt_times, Rotation.from_quat(gt_quaternions))(sample_times)
    return positions, rotations.as_matrix()


def rigid_alignment(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def angle_degrees(rotation_matrices: np.ndarray) -> np.ndarray:
    return np.rad2deg(Rotation.from_matrix(rotation_matrices).magnitude())


def statistics(values: Iterable[float]) -> dict[str, Any]:
    array = np.asarray(list(values), dtype=float)
    if array.size == 0:
        return {"count": 0}
    return {
        "count": int(array.size),
        "rmse": float(np.sqrt(np.mean(np.square(array)))),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def relative_error(
    est_positions: np.ndarray,
    est_rotations: np.ndarray,
    gt_positions: np.ndarray,
    gt_rotations: np.ndarray,
    pairs: list[tuple[int, int]],
) -> dict[str, Any]:
    translation_errors: list[float] = []
    rotation_errors: list[float] = []
    for first, second in pairs:
        est_relative_r = est_rotations[first].T @ est_rotations[second]
        est_relative_t = est_rotations[first].T @ (
            est_positions[second] - est_positions[first]
        )
        gt_relative_r = gt_rotations[first].T @ gt_rotations[second]
        gt_relative_t = gt_rotations[first].T @ (
            gt_positions[second] - gt_positions[first]
        )
        error_r = gt_relative_r.T @ est_relative_r
        error_t = gt_relative_r.T @ (est_relative_t - gt_relative_t)
        translation_errors.append(float(np.linalg.norm(error_t)))
        rotation_errors.append(float(angle_degrees(error_r[None, :, :])[0]))
    return {
        "translation_m": statistics(translation_errors),
        "rotation_deg": statistics(rotation_errors),
    }


def time_pairs(timestamps: np.ndarray, delta_seconds: float) -> list[tuple[int, int]]:
    pairs: list[tuple[int, int]] = []
    for index, timestamp in enumerate(timestamps):
        second = int(np.searchsorted(timestamps, timestamp + delta_seconds))
        if second < timestamps.size and abs(
            timestamps[second] - timestamp - delta_seconds
        ) <= 0.2:
            pairs.append((index, second))
    return pairs


def distance_pairs(positions: np.ndarray, delta_metres: float) -> list[tuple[int, int]]:
    cumulative = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1)))
    )
    pairs: list[tuple[int, int]] = []
    for index, distance in enumerate(cumulative):
        second = int(np.searchsorted(cumulative, distance + delta_metres))
        if second < cumulative.size:
            pairs.append((index, second))
    return pairs


def reference_to_vehicle(
    positions: np.ndarray,
    quaternions: np.ndarray,
    vehicle_r_reference: np.ndarray,
    vehicle_t_reference: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    world_r_reference = Rotation.from_quat(quaternions).as_matrix()
    reference_r_vehicle = vehicle_r_reference.T
    reference_t_vehicle = -reference_r_vehicle @ vehicle_t_reference
    world_r_vehicle = world_r_reference @ reference_r_vehicle
    world_t_vehicle = positions + np.einsum(
        "nij,j->ni", world_r_reference, reference_t_vehicle
    )
    return world_t_vehicle, world_r_vehicle


def runtime_metrics(path: Path) -> dict[str, Any]:
    rows = csv_rows(path)
    result: dict[str, Any] = {}
    for name in ("preprocessing_ms", "odometry_ms", "mapping_ms"):
        result[name] = statistics(float(row[name]) for row in rows)
    result["total_ms"] = statistics(
        sum(float(row[name]) for name in ("preprocessing_ms", "odometry_ms", "mapping_ms"))
        for row in rows
    )
    return result


def synchronization_metrics(path: Path) -> dict[str, Any]:
    rows = csv_rows(path)
    status_counts = Counter(row["status"] for row in rows)
    drop_reasons = Counter(
        row["reason"] for row in rows if row["status"] != "processed"
    )
    processed = [row for row in rows if row["status"] == "processed"]
    return {
        "status_counts": dict(status_counts),
        "drop_reasons": dict(drop_reasons),
        "corrected_skew_s": statistics(
            float(row["max_corrected_skew_s"]) for row in processed
        ),
    }


def feature_metrics(path: Path) -> dict[str, Any]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in csv_rows(path):
        grouped[row["lidar"]].append(row)
    result: dict[str, Any] = {}
    for lidar, rows in sorted(grouped.items()):
        corners = [int(row["corner_count"]) for row in rows]
        surfaces = [int(row["surface_count"]) for row in rows]
        result[lidar] = {
            "frames": len(rows),
            "corner_count": statistics(corners),
            "surface_count": statistics(surfaces),
            "zero_feature_frames": sum(
                corner == 0 or surface == 0
                for corner, surface in zip(corners, surfaces)
            ),
        }
    return result


def observability_metrics(path: Path) -> dict[str, Any]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in csv_rows(path):
        grouped[row["lidar"]].append(row)
    result: dict[str, Any] = {}
    for lidar, rows in sorted(grouped.items()):
        observable = [row for row in rows if row["observable"].lower() == "true"]
        result[lidar] = {
            "observable_records": len(observable),
            "first_observable_frame": (
                int(observable[0]["frame"]) if observable else None
            ),
            "final_state": rows[-1]["state"],
        }
    return result


def optimized_extrinsic_metrics(
    path: Path,
    trusted: dict[str, tuple[np.ndarray, np.ndarray]],
) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text())
    optimized = document.get("vehicle_T_lidar", {})
    result: dict[str, Any] = {}
    for lidar, values in sorted(optimized.items()):
        if lidar not in trusted:
            continue
        array = np.asarray(values, dtype=float)
        optimized_t = array[:3]
        optimized_r = Rotation.from_quat(array[3:7]).as_matrix()
        trusted_r, trusted_t = trusted[lidar]
        result[lidar] = {
            "translation_error_m": float(np.linalg.norm(optimized_t - trusted_t)),
            "rotation_error_deg": float(
                angle_degrees((trusted_r.T @ optimized_r)[None, :, :])[0]
            ),
            "vehicle_T_lidar": [float(value) for value in array],
        }
    return result


def map_metrics(scenario_dir: Path) -> dict[str, Any]:
    maps = {}
    for path in sorted(scenario_dir.glob("map*.pcd")):
        maps[path.name] = {
            "bytes": path.stat().st_size,
            "mib": path.stat().st_size / (1024.0 * 1024.0),
        }
    keyframes: dict[str, int] = {}
    keyframe_root = scenario_dir / "keyframes"
    if keyframe_root.is_dir():
        for lidar_dir in sorted(path for path in keyframe_root.iterdir() if path.is_dir()):
            keyframes[lidar_dir.name] = sum(1 for _ in lidar_dir.glob("*.pcd"))
    return {
        "files": maps,
        "keyframes_per_lidar": keyframes,
        "keyframes_total": sum(keyframes.values()),
    }


def log_metrics(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"available": False}
    text = path.read_text(errors="replace")
    return {
        "available": True,
        "ceres_no_convergence": text.count("Termination: NO_CONVERGENCE"),
        "legacy_mapper_nan_trace": text.count("trace: -nan"),
        "pure_odometry_updates": text.count("optimization with pure odometry"),
        "online_calibration_updates": text.count(
            "optimization with online calibration"
        ),
        "initial_extrinsics_solved": text.count("Initial extrinsic of laser_"),
        "prior_free_transitioned_to_online": (
            "All initial extrinsic rotation calib success" in text
        ),
    }


def write_aligned_trajectory(
    path: Path,
    frames: np.ndarray,
    timestamps: np.ndarray,
    positions: np.ndarray,
    rotations: np.ndarray,
    gt_positions: np.ndarray,
) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "frame",
                "timestamp",
                "tx",
                "ty",
                "tz",
                "qx",
                "qy",
                "qz",
                "qw",
                "gt_tx",
                "gt_ty",
                "gt_tz",
            ]
        )
        quaternions = Rotation.from_matrix(rotations).as_quat()
        for frame, timestamp, position, quaternion, gt_position in zip(
            frames, timestamps, positions, quaternions, gt_positions
        ):
            writer.writerow(
                [
                    int(frame),
                    f"{timestamp:.9f}",
                    *[f"{value:.12g}" for value in position],
                    *[f"{value:.12g}" for value in quaternion],
                    *[f"{value:.12g}" for value in gt_position],
                ]
            )


def native(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): native(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [native(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def evaluate_scenario(
    scenario_dir: Path,
    gt_times: np.ndarray,
    gt_positions_all: np.ndarray,
    gt_quaternions_all: np.ndarray,
    vehicle_r_reference: np.ndarray,
    vehicle_t_reference: np.ndarray,
    trusted_extrinsics: dict[str, tuple[np.ndarray, np.ndarray]],
) -> dict[str, Any]:
    frames, timestamps, reference_positions, reference_quaternions = estimate_trajectory(
        scenario_dir / "trajectory.csv"
    )
    valid = (timestamps >= gt_times[0]) & (timestamps <= gt_times[-1])
    frames = frames[valid]
    timestamps = timestamps[valid]
    reference_positions = reference_positions[valid]
    reference_quaternions = reference_quaternions[valid]
    if timestamps.size < 3:
        raise ValueError(f"less than three timestamp-overlapping poses in {scenario_dir}")

    vehicle_positions, vehicle_rotations = reference_to_vehicle(
        reference_positions,
        reference_quaternions,
        vehicle_r_reference,
        vehicle_t_reference,
    )
    gt_positions, gt_rotations = interpolate_ground_truth(
        timestamps, gt_times, gt_positions_all, gt_quaternions_all
    )
    alignment_r, alignment_t = rigid_alignment(vehicle_positions, gt_positions)
    aligned_positions = (alignment_r @ vehicle_positions.T).T + alignment_t
    aligned_rotations = alignment_r @ vehicle_rotations

    absolute_translation = np.linalg.norm(aligned_positions - gt_positions, axis=1)
    absolute_rotation = angle_degrees(
        np.transpose(gt_rotations, (0, 2, 1)) @ aligned_rotations
    )
    rpe_1s = relative_error(
        aligned_positions,
        aligned_rotations,
        gt_positions,
        gt_rotations,
        time_pairs(timestamps, 1.0),
    )
    rpe_10m = relative_error(
        aligned_positions,
        aligned_rotations,
        gt_positions,
        gt_rotations,
        distance_pairs(gt_positions, 10.0),
    )
    summary = yaml.safe_load((scenario_dir / "summary.yaml").read_text())

    metrics: dict[str, Any] = {
        "scenario": scenario_dir.name,
        "run_status": summary.get("status", "unknown"),
        "processed_frames": int(summary.get("processed_frames", timestamps.size)),
        "evaluated_frames": int(timestamps.size),
        "frames_outside_vehicle_pose_range": int(
            summary.get("processed_frames", timestamps.size) - timestamps.size
        ),
        "time_range_s": [float(timestamps[0]), float(timestamps[-1])],
        "duration_s": float(timestamps[-1] - timestamps[0]),
        "ground_truth_path_length_m": float(
            np.linalg.norm(np.diff(gt_positions, axis=0), axis=1).sum()
        ),
        "estimated_path_length_m": float(
            np.linalg.norm(np.diff(aligned_positions, axis=0), axis=1).sum()
        ),
        "alignment": {
            "method": "rigid_se3_no_scale",
            "rotation_xyzw": [
                float(value) for value in Rotation.from_matrix(alignment_r).as_quat()
            ],
            "translation_m": [float(value) for value in alignment_t],
        },
        "ate_translation_m": statistics(absolute_translation),
        "absolute_orientation_error_deg": statistics(absolute_rotation),
        "rpe_1s": rpe_1s,
        "rpe_10m": rpe_10m,
        "runtime": runtime_metrics(scenario_dir / "runtime.csv"),
        "synchronization": synchronization_metrics(
            scenario_dir / "synchronization.csv"
        ),
        "features": feature_metrics(scenario_dir / "features.csv"),
        "observability": observability_metrics(
            scenario_dir / "observability_history.csv"
        ),
        "optimized_extrinsics": optimized_extrinsic_metrics(
            scenario_dir / "optimized_extrinsics.yaml", trusted_extrinsics
        ),
        "maps": map_metrics(scenario_dir),
        "diagnostics": log_metrics(
            scenario_dir.parent / f"{scenario_dir.name}.log"
        ),
    }
    write_aligned_trajectory(
        scenario_dir / "trajectory_vehicle_aligned.csv",
        frames,
        timestamps,
        aligned_positions,
        aligned_rotations,
        gt_positions,
    )
    (scenario_dir / "odometry_metrics.yaml").write_text(
        yaml.safe_dump(native(metrics), sort_keys=False)
    )
    return metrics


def metric(metrics: dict[str, Any], *keys: str) -> float | None:
    value: Any = metrics
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return float(value) if value is not None else None


def shown(value: float | None, digits: int = 3) -> str:
    return "n/a" if value is None or not math.isfinite(value) else f"{value:.{digits}f}"


def report(
    path: Path,
    manifest_path: Path,
    pose_dir: Path,
    reference_lidar: str,
    pose_status_counts: Counter[str],
    results: list[dict[str, Any]],
) -> None:
    lines = [
        "# M-LOAM AIV5 offline evaluation",
        "",
        f"- Manifest: `{manifest_path}`",
        f"- Vehicle-pose source: `{pose_dir}`",
        f"- Reference LiDAR: `{reference_lidar}`",
        f"- Vehicle-pose statuses: `{dict(pose_status_counts)}`",
        "- Position comparison uses one rigid SE(3) alignment per scenario; scale is not fitted.",
        "- The supplied vehicle_pose stream is DR and is treated as a reference, not surveyed ground truth.",
        "",
        "## Odometry summary",
        "",
        "| Scenario | Status | Frames | ATE RMSE (m) | ATE p95 (m) | RPE 1 s RMSE (m) | RPE 10 m RMSE (m) | Runtime median (ms) | Final map (MiB) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        lines.append(
            "| {scenario} | {status} | {frames} | {ate} | {ate95} | {rpe1} | {rpe10} | {runtime} | {map_size} |".format(
                scenario=result["scenario"],
                status=result["run_status"],
                frames=result["evaluated_frames"],
                ate=shown(metric(result, "ate_translation_m", "rmse")),
                ate95=shown(metric(result, "ate_translation_m", "p95")),
                rpe1=shown(metric(result, "rpe_1s", "translation_m", "rmse")),
                rpe10=shown(metric(result, "rpe_10m", "translation_m", "rmse")),
                runtime=shown(metric(result, "runtime", "total_ms", "median"), 1),
                map_size=shown(metric(result, "maps", "files", "map_final_rgb.pcd", "mib"), 1),
            )
        )

    for result in results:
        lines.extend(
            [
                "",
                f"## {result['scenario']}",
                "",
                f"Processed/evaluated frames: {result['processed_frames']}/{result['evaluated_frames']}; "
                f"reference path length: {result['ground_truth_path_length_m']:.1f} m.",
                "",
                "| LiDAR | Rotation error (deg) | Translation error (m) | Observable records | First observable frame | Final state |",
                "|---|---:|---:|---:|---:|---|",
            ]
        )
        for lidar, extrinsic in result["optimized_extrinsics"].items():
            observation = result["observability"].get(lidar, {})
            first = observation.get("first_observable_frame")
            lines.append(
                f"| {lidar} | {extrinsic['rotation_error_deg']:.4f} | "
                f"{extrinsic['translation_error_m']:.4f} | "
                f"{observation.get('observable_records', 0)} | "
                f"{first if first is not None else 'n/a'} | "
                f"{observation.get('final_state', 'n/a')} |"
            )
        zero_features = {
            lidar: values["zero_feature_frames"]
            for lidar, values in result["features"].items()
        }
        lines.extend(
            [
                "",
                f"- Synchronization status: `{result['synchronization']['status_counts']}`",
                f"- Synchronization drop reasons: `{result['synchronization']['drop_reasons']}`",
                f"- Zero-feature frames by LiDAR: `{zero_features}`",
                f"- Keyframes written: {result['maps']['keyframes_total']}",
                f"- Log diagnostics: `{result['diagnostics']}`",
                f"- Maximum recorded per-frame runtime: {shown(metric(result, 'runtime', 'total_ms', 'max'), 1)} ms",
                f"- Detailed metrics: `{result['scenario']}/odometry_metrics.yaml`",
                f"- Aligned odometry: `{result['scenario']}/trajectory_vehicle_aligned.csv`",
                f"- Optimized parameters: `{result['scenario']}/optimized_extrinsics.yaml`",
                f"- Accumulated map: `{result['scenario']}/map_final_rgb.pcd`",
            ]
        )

    lines.extend(
        [
            "",
            "## Metric definitions",
            "",
            "ATE is the vehicle-origin position error after rigid alignment. RPE is computed from relative SE(3) motion over approximately 1 second and over 10 metres of reference travel. Rotation errors use the geodesic SO(3) angle. Runtime is preprocessing plus odometry plus mapping as recorded by the offline runner.",
            "",
        ]
    )
    path.write_text("\n".join(lines))


def main() -> int:
    options = arguments()
    manifest_path = options.manifest.resolve()
    manifest = yaml.safe_load(manifest_path.read_text())
    dataset_root = Path(manifest["dataset_root"]).expanduser()
    if not dataset_root.is_absolute():
        dataset_root = manifest_path.parent / dataset_root
    pose_dir = options.pose_dir or dataset_root / "vehicle_pose"
    reference_lidar = manifest["reference_lidar"]

    trusted_extrinsics: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for lidar in manifest["lidars"]:
        trusted_extrinsics[lidar["name"]] = manifest_calibration(
            manifest_path, lidar
        )
    vehicle_r_reference, vehicle_t_reference = trusted_extrinsics[reference_lidar]
    gt_times, gt_positions, gt_quaternions, gt_statuses = pose_records(pose_dir)

    results: list[dict[str, Any]] = []
    for scenario in options.scenarios:
        scenario_dir = options.session_root / scenario
        if not (scenario_dir / "trajectory.csv").is_file():
            print(f"skip missing scenario: {scenario_dir}")
            continue
        print(f"evaluate {scenario_dir}")
        results.append(
            evaluate_scenario(
                scenario_dir,
                gt_times,
                gt_positions,
                gt_quaternions,
                vehicle_r_reference,
                vehicle_t_reference,
                trusted_extrinsics,
            )
        )
    if not results:
        raise ValueError("no completed scenarios found")

    session_metrics = {
        "manifest": str(options.manifest),
        "pose_directory": str(pose_dir),
        "reference_lidar": reference_lidar,
        "vehicle_pose_status_counts": dict(Counter(gt_statuses)),
        "scenarios": results,
    }
    options.session_root.mkdir(parents=True, exist_ok=True)
    (options.session_root / "session_metrics.yaml").write_text(
        yaml.safe_dump(native(session_metrics), sort_keys=False)
    )
    report(
        options.session_root / "session_report.md",
        options.manifest,
        pose_dir,
        reference_lidar,
        Counter(gt_statuses),
        results,
    )
    print(f"wrote {options.session_root / 'session_report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
