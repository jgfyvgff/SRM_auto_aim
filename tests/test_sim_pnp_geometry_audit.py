#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "tools"))

import sim_pnp_geometry_audit as audit


class SimPnpGeometryAuditTest(unittest.TestCase):
    def setUp(self):
        self.calibration = audit.Calibration(
            camera_matrix=np.array(
                [[1000.0, 0.0, 640.0], [0.0, 1000.0, 360.0], [0.0, 0.0, 1.0]]
            ),
            distortion=np.zeros((1, 5)),
            rotation_camera2gimbal=np.eye(3),
            translation_camera2gimbal=np.zeros(3),
            rotation_gimbal2imubody=np.eye(3),
        )

    def test_exact_geometry_recovers_truth(self):
        object_points = audit.armor_object_points("small", 0.135, 0.056)
        rvec = np.array([[0.08], [0.12], [-0.03]])
        tvec = np.array([[0.10], [-0.06], [3.0]])
        image_points, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            self.calibration.camera_matrix,
            self.calibration.distortion,
        )
        image_points = image_points.reshape(4, 2)
        sample = {
            **{
                f"accepted_corner_{index}_{axis}": float(image_points[index, axis_index])
                for index in range(4)
                for axis, axis_index in (("x", 0), ("y", 1))
            },
            "association_primary_truth_x": float(tvec[0, 0]),
            "association_primary_truth_y": float(tvec[1, 0]),
            "association_primary_truth_z": float(tvec[2, 0]),
            "gimbal_tf_qx": 0.0,
            "gimbal_tf_qy": 0.0,
            "gimbal_tf_qz": 0.0,
            "gimbal_tf_qw": 1.0,
        }

        result = audit.evaluate_sample(
            sample, "small", 0.135, 0.056, self.calibration
        )
        self.assertIsNotNone(result)
        self.assertLess(result["position_error"], 1e-5)
        self.assertLess(result["reprojection_error"], 1e-5)

    def test_grid_contains_baseline_and_exact_configuration(self):
        self.assertEqual(
            audit._grid(0.050, 0.051, 0.0005), [0.05, 0.0505, 0.051]
        )

    def test_load_samples_skips_ros_echo_separators(self):
        path = Path("/tmp/sim_pnp_geometry_audit_test.jsonl")
        path.write_text('{"value": 1}\n---\n{"value": 2}\n', encoding="utf-8")
        try:
            self.assertEqual(audit.load_samples(path), [{"value": 1}, {"value": 2}])
        finally:
            path.unlink(missing_ok=True)

    def test_incomplete_runtime_sample_is_skipped(self):
        self.assertIsNone(
            audit.evaluate_sample(
                {}, "small", 0.135, 0.056, self.calibration
            )
        )

    def test_fit_origin_offset_recovers_synthetic_offset(self):
        object_points = audit.armor_object_points("small", 0.135, 0.056)
        expected_offset = np.array([0.010, -0.005, 0.003])
        rvec = np.array([[0.08], [0.12], [-0.03]])
        tvec = np.array([[0.10], [-0.06], [3.0]])
        shifted_points = object_points + expected_offset
        image_points, _ = cv2.projectPoints(
            shifted_points,
            rvec,
            tvec,
            self.calibration.camera_matrix,
            self.calibration.distortion,
        )
        image_points = image_points.reshape(4, 2)
        rotation, _ = cv2.Rodrigues(rvec)
        sample = {
            **{
                f"accepted_corner_{index}_{axis}": float(image_points[index, axis_index])
                for index in range(4)
                for axis, axis_index in (("x", 0), ("y", 1))
            },
            "association_primary_truth_x": float(tvec[0, 0]),
            "association_primary_truth_y": float(tvec[1, 0]),
            "association_primary_truth_z": float(tvec[2, 0]),
            "association_primary_optimized_yaw": float(
                np.arctan2(rotation[1, 0], rotation[0, 0])
            ),
            "gimbal_tf_qx": 0.0,
            "gimbal_tf_qy": 0.0,
            "gimbal_tf_qz": 0.0,
            "gimbal_tf_qw": 1.0,
        }

        fit = audit.fit_origin_offset(
            [sample], "small", 0.135, 0.056, self.calibration
        )

        self.assertIsNotNone(fit)
        np.testing.assert_allclose(
            fit["point_offset_to_add"], expected_offset, atol=1e-5
        )
        self.assertLess(fit["p95_position_error_after_offset"], 1e-5)


if __name__ == "__main__":
    unittest.main()
