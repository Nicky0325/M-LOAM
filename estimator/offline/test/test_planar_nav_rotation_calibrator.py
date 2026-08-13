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
    def test_cli_uses_raw_imu_and_gnss_inputs(self) -> None:
        options = calibrator.arguments(
            ["--manifest", "manifest.yaml", "--output-dir", "output"]
        )

        self.assertTrue(hasattr(options, "imu_file"))
        self.assertTrue(hasattr(options, "gnss_dir"))
        self.assertFalse(hasattr(options, "pose_dir"))
        self.assertFalse(hasattr(options, "ins_file"))
        self.assertFalse(hasattr(options, "wheel_file"))

    def test_prior_free_loader_accepts_translation_only_manifest(self) -> None:
        document = {
            "dataset_root": "/data",
            "lidars": [
                {
                    "name": "translation_only",
                    "directory": "lidar",
                    "vehicle_T_lidar": {"translation": [7.0, 1.0, 0.5]},
                }
            ],
        }

        lidars = calibrator.load_lidars(
            Path("/tmp/manifest.yaml"),
            document,
            include=None,
            load_manifest_rotations=False,
        )

        np.testing.assert_array_equal(lidars[0].translation, [7.0, 1.0, 0.5])
        self.assertIsNone(lidars[0].trusted_rotation)
        with self.assertRaisesRegex(ValueError, "no manifest rotation"):
            calibrator.load_lidars(
                Path("/tmp/manifest.yaml"),
                document,
                include=None,
                load_manifest_rotations=True,
            )

    def test_shifts_antenna_motion_to_vehicle_origin(self) -> None:
        antenna_motion = calibrator.make_transform(
            Rotation.from_euler("z", 12.0, degrees=True).as_matrix(),
            np.asarray([2.0, -0.4, 0.0]),
        )
        lever_arm = np.asarray([3.0, 0.8, 0.0])
        shifted = calibrator.motion_at_vehicle_origin(antenna_motion, lever_arm)
        expected_translation = (
            antenna_motion[:3, 3]
            + (np.eye(3) - antenna_motion[:3, :3]) @ lever_arm
        )

        np.testing.assert_allclose(shifted[:3, :3], antenna_motion[:3, :3])
        np.testing.assert_allclose(shifted[:3, 3], expected_translation)

    def test_estimates_shared_planar_gnss_lever_arm(self) -> None:
        true_lever_arm = np.asarray([2.4, -0.7, 0.0])
        lidar_definitions = []
        calibrations = {}
        translations = (
            np.asarray([7.0, 1.2, 0.6]),
            np.asarray([-7.1, -1.1, 0.6]),
            np.asarray([0.2, 2.0, 1.5]),
        )
        rotations = (
            Rotation.from_euler("xyz", [1.0, -2.0, 120.0], degrees=True).as_matrix(),
            Rotation.from_euler("xyz", [-1.0, 1.0, -55.0], degrees=True).as_matrix(),
            Rotation.from_euler("xyz", [0.5, -3.0, 10.0], degrees=True).as_matrix(),
        )
        vehicle_t_antenna = calibrator.make_transform(np.eye(3), true_lever_arm)
        antenna_t_vehicle = calibrator.inverse(vehicle_t_antenna)
        for lidar_index, (translation, rotation) in enumerate(
            zip(translations, rotations)
        ):
            name = f"lidar_{lidar_index}"
            definition = calibrator.LidarDefinition(
                name=name,
                directory=Path("."),
                translation=translation,
                trusted_rotation=rotation,
                color=(1.0, 0.0, 0.0),
            )
            lidar_definitions.append(definition)
            extrinsic = calibrator.make_transform(rotation, translation)
            constraints = []
            for pair_index in range(15):
                vehicle_motion = calibrator.make_transform(
                    Rotation.from_euler(
                        "z", 2.0 + 0.7 * pair_index, degrees=True
                    ).as_matrix(),
                    np.asarray(
                        [1.0 + 0.1 * pair_index, 0.2 * math.sin(pair_index), 0.0]
                    ),
                )
                antenna_motion = (
                    antenna_t_vehicle @ vehicle_motion @ vehicle_t_antenna
                )
                lidar_motion = (
                    calibrator.inverse(extrinsic) @ vehicle_motion @ extrinsic
                )
                constraint = calibrator.PairConstraint(
                    candidate=calibrator.PairCandidate(
                        first=pair_index,
                        second=pair_index + 10,
                        vehicle_motion=antenna_motion,
                        translation_m=float(
                            np.linalg.norm(antenna_motion[:3, 3])
                        ),
                        rotation_deg=2.0 + 0.7 * pair_index,
                    ),
                    lidar_motion=lidar_motion,
                    fitness=0.9,
                    inlier_rmse_m=0.1,
                    icp_translation_update_m=0.0,
                    icp_rotation_update_deg=0.0,
                    accepted=True,
                    split="heldout" if pair_index % 5 == 0 else "train",
                )
                constraints.append(constraint)
            calibrations[name] = calibrator.RotationCalibration(
                name=name,
                initial_rotation=rotation,
                optimized_rotation=rotation,
                fixed_translation=translation,
                constraints=constraints,
                accepted=True,
            )

        estimate = calibrator.estimate_gnss_lever_arm(
            lidar_definitions,
            calibrations,
            np.zeros(3),
            bound_m=10.0,
        )

        self.assertTrue(estimate.accepted, estimate.reason)
        np.testing.assert_allclose(estimate.value, true_lever_arm, atol=1.0e-6)

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

    def test_recovers_arbitrary_rotations_without_using_manifest_rotations(self) -> None:
        true_lever_arm = np.asarray([3.2, -0.8, 0.0])
        translations = (
            np.asarray([7.1, 1.2, 0.6]),
            np.asarray([7.2, -1.1, 0.6]),
            np.asarray([-7.2, -1.2, 0.6]),
            np.asarray([-7.3, 1.1, 0.6]),
        )
        true_rotations = (
            Rotation.from_euler("xyz", [27.0, -41.0, 132.0], degrees=True).as_matrix(),
            Rotation.from_euler("xyz", [-65.0, 18.0, 37.0], degrees=True).as_matrix(),
            Rotation.from_euler("xyz", [91.0, -22.0, -54.0], degrees=True).as_matrix(),
            Rotation.from_euler("xyz", [-38.0, 73.0, -149.0], degrees=True).as_matrix(),
        )
        bogus_manifest_rotations = (
            Rotation.from_euler("xyz", [-5.0, 81.0, 11.0], degrees=True).as_matrix(),
            Rotation.from_euler("xyz", [55.0, -44.0, 170.0], degrees=True).as_matrix(),
            Rotation.from_euler("xyz", [-89.0, 3.0, 42.0], degrees=True).as_matrix(),
            Rotation.from_euler("xyz", [12.0, 33.0, -7.0], degrees=True).as_matrix(),
        )
        vehicle_t_antenna = calibrator.make_transform(np.eye(3), true_lever_arm)
        antenna_t_vehicle = calibrator.inverse(vehicle_t_antenna)
        lidars = []
        constraints_by_lidar = {}
        for lidar_index, (translation, true_rotation, bogus_rotation) in enumerate(
            zip(translations, true_rotations, bogus_manifest_rotations)
        ):
            name = f"lidar_{lidar_index}"
            lidar = calibrator.LidarDefinition(
                name=name,
                directory=Path("."),
                translation=translation,
                trusted_rotation=bogus_rotation,
                color=(1.0, 0.0, 0.0),
            )
            lidars.append(lidar)
            extrinsic = calibrator.make_transform(true_rotation, translation)
            constraints = []
            for motion_index in range(25):
                yaw_deg = 2.0 + 0.45 * motion_index
                vehicle_motion = calibrator.make_transform(
                    Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix(),
                    np.asarray(
                        [
                            1.0 + 0.08 * motion_index,
                            0.35 * math.sin(0.4 * motion_index),
                            0.0,
                        ]
                    ),
                )
                antenna_motion = (
                    antenna_t_vehicle @ vehicle_motion @ vehicle_t_antenna
                )
                lidar_motion = (
                    calibrator.inverse(extrinsic) @ vehicle_motion @ extrinsic
                )
                constraints.append(
                    calibrator.PairConstraint(
                        candidate=calibrator.PairCandidate(
                            first=motion_index,
                            second=motion_index + 10,
                            vehicle_motion=antenna_motion,
                            translation_m=float(
                                np.linalg.norm(antenna_motion[:3, 3])
                            ),
                            rotation_deg=yaw_deg,
                        ),
                        lidar_motion=lidar_motion,
                        fitness=0.9,
                        inlier_rmse_m=0.1,
                        icp_translation_update_m=float(
                            np.linalg.norm(lidar_motion[:3, 3])
                        ),
                        icp_rotation_update_deg=yaw_deg,
                        accepted=True,
                        registration_mode="synthetic_identity_seeded",
                        motion_angle_error_deg=0.0,
                    )
                )
            constraints_by_lidar[name] = constraints

        result = calibrator.estimate_prior_free_initialization(
            lidars,
            constraints_by_lidar,
            minimum_constraints=12,
            lever_arm_bound_m=10.0,
            axis_inlier_deg=4.0,
        )

        self.assertTrue(result.accepted, result.reason)
        np.testing.assert_allclose(result.gnss_lever_arm, true_lever_arm, atol=1.0e-5)
        for lidar, true_rotation, bogus_rotation in zip(
            lidars, true_rotations, bogus_manifest_rotations
        ):
            initialized = result.lidars[lidar.name].rotation
            self.assertLess(
                calibrator.angular_distance_degrees(true_rotation, initialized),
                1.0e-4,
            )
            self.assertGreater(
                calibrator.angular_distance_degrees(bogus_rotation, initialized),
                10.0,
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
        self.assertFalse(
            document["joint_backend"]["optimize_extrinsic_translation"]
        )
        np.testing.assert_array_equal(
            document["lidars"][0]["vehicle_T_lidar"]["translation"],
            translation,
        )
        self.assertNotIn("extrinsic_prototxt", document["lidars"][0])

        source_without_backend = dict(source)
        source_without_backend.pop("joint_backend")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            manifest_path = output / "corrected.yaml"
            calibrator.write_corrected_manifest(
                manifest_path,
                source_without_backend,
                {"lidar": calibration},
                output,
            )
            document = yaml.safe_load(manifest_path.read_text())
        self.assertFalse(document["joint_backend"]["enabled"])
        self.assertFalse(
            document["joint_backend"]["optimize_extrinsic_translation"]
        )

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
