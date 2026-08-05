#!/usr/bin/env python3
"""Unit tests for the planar navigation-aided calibration prototype."""

from __future__ import annotations

import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "tools"
    / "planar_nav_rotation_calibrator.py"
)
SPEC = importlib.util.spec_from_file_location("planar_nav_calibrator", MODULE_PATH)
calibrator = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = calibrator
SPEC.loader.exec_module(calibrator)

EVALUATOR_PATH = Path(__file__).resolve().parents[1] / "tools" / "evaluate_aiv5_session.py"
EVALUATOR_SPEC = importlib.util.spec_from_file_location("evaluate_aiv5", EVALUATOR_PATH)
evaluator = importlib.util.module_from_spec(EVALUATOR_SPEC)
assert EVALUATOR_SPEC.loader is not None
sys.modules[EVALUATOR_SPEC.name] = evaluator
EVALUATOR_SPEC.loader.exec_module(evaluator)


class PlanarNavigationCalibrationTest(unittest.TestCase):
    def test_recovers_ten_degrees_without_changing_translation(self) -> None:
        true_rotation = Rotation.from_euler(
            "xyz", [1.0, -2.0, 125.0], degrees=True
        ).as_matrix()
        fixed_translation = np.asarray([7.17, 1.21, 0.59])
        initial_rotation = true_rotation @ Rotation.from_rotvec(
            math.radians(10.0) * np.asarray([0.4, -0.7, 0.591607978])
        ).as_matrix()
        lidar = calibrator.LidarDefinition(
            name="synthetic",
            directory=Path("."),
            translation=fixed_translation.copy(),
            trusted_rotation=true_rotation,
            color=(1.0, 0.0, 0.0),
        )
        true_extrinsic = calibrator.make_transform(
            true_rotation, fixed_translation
        )
        constraints = []
        for index in range(20):
            yaw = 2.0 + 0.35 * index
            vehicle_motion = calibrator.make_transform(
                Rotation.from_euler("z", yaw, degrees=True).as_matrix(),
                np.asarray(
                    [1.2 + 0.08 * index, 0.15 * math.sin(index), 0.0]
                ),
            )
            lidar_motion = (
                calibrator.inverse(true_extrinsic)
                @ vehicle_motion
                @ true_extrinsic
            )
            candidate = calibrator.PairCandidate(
                first=index,
                second=index + 10,
                vehicle_motion=vehicle_motion,
                translation_m=float(np.linalg.norm(vehicle_motion[:3, 3])),
                rotation_deg=yaw,
            )
            constraints.append(
                calibrator.PairConstraint(
                    candidate=candidate,
                    lidar_motion=lidar_motion,
                    fitness=0.9,
                    inlier_rmse_m=0.1,
                    icp_translation_update_m=0.2,
                    icp_rotation_update_deg=1.0,
                    accepted=True,
                )
            )

        result = calibrator.calibrate_rotation(
            lidar,
            initial_rotation,
            constraints,
            minimum_constraints=12,
            update_bound_deg=15.0,
            trusted_rotation=true_rotation,
        )

        self.assertTrue(result.accepted, result.reason)
        self.assertLess(result.trusted_final_error_deg, 1.0e-4)
        np.testing.assert_array_equal(result.fixed_translation, fixed_translation)
        self.assertLess(
            result.heldout_final["translation_rmse_m"],
            result.heldout_initial["translation_rmse_m"],
        )

    def test_deterministic_perturbation_is_exact_and_stable(self) -> None:
        identity = np.eye(3)
        first = calibrator.perturbed_rotation(identity, 10.0, 42, "lidar_a")
        second = calibrator.perturbed_rotation(identity, 10.0, 42, "lidar_a")
        other = calibrator.perturbed_rotation(identity, 10.0, 42, "lidar_b")

        np.testing.assert_allclose(first, second)
        self.assertAlmostEqual(
            calibrator.angular_distance_degrees(identity, first), 10.0, places=10
        )
        self.assertGreater(
            calibrator.angular_distance_degrees(first, other), 1.0
        )

    def test_evaluator_accepts_inline_fixed_translation_extrinsic(self) -> None:
        rotation, translation = evaluator.manifest_calibration(
            Path("/tmp/example/manifest.yaml"),
            {
                "name": "inline",
                "vehicle_T_lidar": {
                    "translation": [7.0, -1.0, 0.6],
                    "rpy_deg": [1.0, -2.0, 30.0],
                },
            },
        )

        np.testing.assert_array_equal(translation, [7.0, -1.0, 0.6])
        np.testing.assert_allclose(
            rotation,
            Rotation.from_euler("xyz", [1.0, -2.0, 30.0], degrees=True).as_matrix(),
        )

    def test_corrected_manifest_disables_backend_and_copies_translation(self) -> None:
        translation = np.asarray([7.17, 1.21, 0.59])
        calibration = calibrator.RotationCalibration(
            name="lidar",
            initial_rotation=np.eye(3),
            optimized_rotation=Rotation.from_euler(
                "xyz", [1.0, 2.0, 3.0], degrees=True
            ).as_matrix(),
            fixed_translation=translation,
            accepted=True,
        )
        source = {
            "dataset_root": "/data",
            "output_root": "/old",
            "joint_backend": {"enabled": True, "mode": "coarse_bootstrap"},
            "lidars": [
                {
                    "name": "lidar",
                    "extrinsic_prototxt": "/old/extrinsic.prototxt",
                }
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            manifest_path = output / "corrected.yaml"
            calibrator.write_corrected_manifest(
                manifest_path, source, {"lidar": calibration}, output
            )
            document = yaml.safe_load(manifest_path.read_text())

        self.assertFalse(document["joint_backend"]["enabled"])
        self.assertEqual(document["joint_backend"]["mode"], "disabled")
        np.testing.assert_array_equal(
            document["lidars"][0]["vehicle_T_lidar"]["translation"],
            translation,
        )
        self.assertNotIn("extrinsic_prototxt", document["lidars"][0])

    def test_cross_lidar_alignment_reports_overlap(self) -> None:
        first = np.asarray([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
        second = first + np.asarray([0.1, 0.0, 0.0])
        metrics = calibrator.cross_lidar_alignment_metrics(
            ["first", "second"], [first, second]
        )

        self.assertEqual(metrics["matched_queries"], 4)
        self.assertAlmostEqual(metrics["mean_overlap_fraction_lt_1m"], 1.0)
        self.assertAlmostEqual(metrics["matched_median_m"], 0.1)


if __name__ == "__main__":
    unittest.main()
