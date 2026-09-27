#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "tools"))

import sim_tracker_analyzer as analyzer


def make_sample(index, angular_velocity, center_x, center_y, armor_id=0):
    return {
        "timestamp": index * 0.01,
        "angular_velocity": angular_velocity,
        "center_x": center_x,
        "center_y": center_y,
        "center_speed": 0.02,
        "vx": 0.01,
        "vy": -0.01,
        "radius": 0.20,
        "alternate_radius": 0.20,
        "current_armor_id": armor_id,
    }


def warning_codes(report):
    return {
        diagnosis["code"]
        for diagnosis in report["diagnoses"]
        if diagnosis["level"] == "WARN"
    }


def make_prediction_source(armor_id=2, generation=1):
    sample = make_sample(0, 4.0, 1.4, -0.3, armor_id)
    sample.update(
        {
            "tracker_generation": generation,
            "high_speed_mode": 1,
            "delay_time": 0.03,
            "base_prediction_dt": 0.03,
            "fly_time": 0.04,
            "aim_armor_id": armor_id,
            "prediction_dt": 0.1,
            "aim_current_x": 1.0,
            "aim_current_y": 0.0,
            "aim_current_z": 0.2,
            "aim_current_yaw": 0.0,
            "future_x": 1.1,
            "future_y": 0.0,
            "future_z": 0.2,
            "future_yaw": 0.1,
        }
    )
    return sample


def make_future_observation(prefix="association_primary", armor_id=2, generation=1):
    sample = make_sample(10, 4.0, 1.4, -0.3, armor_id)
    sample.update(
        {
            "tracker_generation": generation,
            f"{prefix}_id": armor_id,
            f"{prefix}_accepted": 1,
            f"{prefix}_observed_x": 1.1,
            f"{prefix}_observed_y": 0.0,
            f"{prefix}_observed_z": 0.2,
            f"{prefix}_optimized_yaw": 0.1,
        }
    )
    return sample


class SimTrackerAnalyzerTest(unittest.TestCase):
    def test_position_residual_keeps_fractional_precision(self):
        sample = analyzer.normalize_sample(
            {
                "association_primary_position_error": 0.2325,
                "association_primary_distance_error": 0.0095,
                "prediction_dt": 0.12,
                "aim_current_x": 1.0,
                "future_x": 1.1,
                "delay_time": 0.03,
                "base_prediction_dt": 0.03,
                "fly_time": 0.04,
            },
            1.0,
        )

        self.assertAlmostEqual(sample["association_primary_position_error"], 0.2325)
        self.assertAlmostEqual(sample["association_primary_distance_error"], 0.0095)
        self.assertAlmostEqual(sample["prediction_dt"], 0.12)
        self.assertAlmostEqual(sample["aim_current_x"], 1.0)
        self.assertAlmostEqual(sample["future_x"], 1.1)

    def test_prediction_reports_mode_and_signed_time_correction(self):
        source = make_prediction_source()
        source["future_yaw"] = 0.2
        report = analyzer.analyze_samples(
            [source, make_future_observation()]
        )
        prediction = report["aimer_prediction"]["spin"]
        stats = prediction["stats"]

        self.assertAlmostEqual(prediction["high_speed_mode_rate"], 1.0)
        self.assertAlmostEqual(
            stats["signed_prediction_yaw_error"]["mean"], 0.1
        )
        self.assertAlmostEqual(
            stats["equivalent_dt_correction"]["mean"], -0.025
        )

    def test_constant_velocity_prediction_reduces_error(self):
        report = analyzer.analyze_samples(
            [make_prediction_source(), make_future_observation()]
        )
        prediction = report["aimer_prediction"]["spin"]

        self.assertEqual(prediction["eligible_count"], 1)
        self.assertEqual(prediction["matched_count"], 1)
        self.assertAlmostEqual(prediction["match_rate"], 1.0)
        self.assertAlmostEqual(
            prediction["stats"]["baseline_position_error"]["mean"], 0.1
        )
        self.assertAlmostEqual(
            prediction["stats"]["prediction_position_error"]["mean"], 0.0
        )
        self.assertAlmostEqual(prediction["prediction_better_rate"], 1.0)

    def test_prediction_rejects_different_generation(self):
        report = analyzer.analyze_samples(
            [make_prediction_source(generation=1), make_future_observation(generation=2)]
        )
        prediction = report["aimer_prediction"]["overall"]

        self.assertEqual(prediction["eligible_count"], 1)
        self.assertEqual(prediction["matched_count"], 0)

    def test_prediction_requires_matching_armor_id(self):
        report = analyzer.analyze_samples(
            [make_prediction_source(armor_id=2), make_future_observation(armor_id=1)]
        )
        prediction = report["aimer_prediction"]["overall"]

        self.assertEqual(prediction["eligible_count"], 1)
        self.assertEqual(prediction["matched_count"], 0)

    def test_secondary_accepted_candidate_can_be_ground_truth(self):
        report = analyzer.analyze_samples(
            [
                make_prediction_source(),
                make_future_observation(prefix="association_secondary"),
            ]
        )
        prediction = report["aimer_prediction"]["overall"]

        self.assertEqual(prediction["matched_count"], 1)

    def test_stable_static_and_spin_do_not_trigger_center_warning(self):
        samples = []
        for index in range(80):
            samples.append(
                make_sample(index, 0.05, 1.4 + 0.002 * math.sin(index), -0.3)
            )
        for index in range(80, 160):
            samples.append(
                make_sample(index, 4.0, 1.4 + 0.003 * math.sin(index), -0.3, index // 20 % 4)
            )

        codes = warning_codes(analyzer.analyze_samples(samples))

        self.assertNotIn("static_center_span", codes)
        self.assertNotIn("spin_center_span", codes)
        self.assertNotIn("spin_id_jump", codes)

    def test_rotating_center_is_reported(self):
        samples = [
            make_sample(
                index,
                4.0,
                1.4 + 0.2 * math.sin(index * 0.15),
                -0.3 + 0.2 * math.cos(index * 0.15),
            )
            for index in range(100)
        ]

        codes = warning_codes(analyzer.analyze_samples(samples))

        self.assertIn("spin_center_span", codes)

    def test_id_switch_center_jump_is_reported(self):
        samples = []
        for index in range(100):
            armor_id = (index // 20) % 2
            center_x = 1.4 + 0.12 * armor_id
            samples.append(make_sample(index, 4.0, center_x, -0.3, armor_id))

        codes = warning_codes(analyzer.analyze_samples(samples))

        self.assertIn("spin_id_jump", codes)

    def test_association_collision_and_tracker_reset_are_counted(self):
        samples = []
        for index in range(40):
            sample = make_sample(index, 0.05, 1.4, -0.3)
            sample.update(
                {
                    "tracker_generation": 1 if index < 20 else 2,
                    "association_candidate_count": 2,
                    "association_accepted_count": 1,
                    "association_primary_id": 0,
                    "association_secondary_id": 0,
                }
            )
            if index >= 20:
                sample["center_x"] = 1.6
            samples.append(sample)

        phase = analyzer.analyze_samples(samples)["phases"]["static"]

        self.assertEqual(phase["tracker_reset_count"], 1)
        self.assertAlmostEqual(phase["tracker_reset_step_p95"], 0.2)
        self.assertEqual(phase["id_switch_count"], 0)
        self.assertAlmostEqual(phase["duplicate_model_id_rate"], 1.0)
        self.assertAlmostEqual(phase["rejected_candidate_frame_rate"], 1.0)

    def test_single_center_spike_is_not_reported_as_sustained_motion(self):
        samples = [make_sample(index, 0.05, 1.4, -0.3) for index in range(100)]
        samples[50]["center_x"] = 1.8

        report = analyzer.analyze_samples(samples)

        self.assertNotIn("static_center_span", warning_codes(report))
        self.assertAlmostEqual(
            report["phases"]["static"]["stats"]["center_x"]["span"],
            0.4,
        )
        self.assertAlmostEqual(
            report["phases"]["static"]["stats"]["center_x"]["robust_span"],
            0.0,
        )

    def test_outlier_snapshot_keeps_association_context(self):
        samples = [make_sample(index, 0.05, 1.4, -0.3) for index in range(20)]
        samples[10].update(
            {
                "center_x": 1.7,
                "current_ekf_error": 180.0,
                "tracker_generation": 3,
                "association_primary_id": 2,
                "association_primary_score": 0.8,
                "association_primary_position_error": 0.24,
                "association_primary_distance_error": 0.21,
                "association_primary_observed_x": 1.3,
                "association_primary_observed_y": -0.2,
                "association_primary_observed_z": 0.4,
                "association_primary_predicted_x": 1.1,
                "association_primary_predicted_y": -0.1,
                "association_primary_predicted_z": 0.4,
                "association_primary_observed_distance": 1.37,
                "association_primary_predicted_distance": 1.18,
                "association_primary_raw_yaw": 0.2,
                "association_primary_optimized_yaw": 1.0,
            }
        )

        outliers = analyzer.analyze_samples(samples)["outliers"]

        self.assertAlmostEqual(outliers["center_jump"][0]["center_step"], 0.3)
        self.assertEqual(outliers["center_jump"][0]["association_primary_id"], 2)
        self.assertAlmostEqual(
            outliers["center_jump"][0]["association_primary_position_error"],
            0.24,
        )
        self.assertAlmostEqual(
            outliers["center_jump"][0]["association_primary_observed_x"],
            1.3,
        )
        self.assertEqual(outliers["ekf_error"][0]["current_ekf_error"], 180.0)
        self.assertEqual(outliers["ekf_error"][0]["tracker_generation"], 3)

    def test_association_gate_rejection_is_reported_separately(self):
        samples = []
        for index in range(30):
            sample = make_sample(index, 4.0, 1.4, -0.3)
            sample.update(
                {
                    "association_candidate_count": 1,
                    "association_accepted_count": 0,
                    "association_primary_id": 2,
                    "association_primary_gate_passed": 0,
                }
            )
            samples.append(sample)

        phase = analyzer.analyze_samples(samples)["phases"]["spin"]

        self.assertAlmostEqual(phase["rejected_candidate_frame_rate"], 1.0)
        self.assertAlmostEqual(phase["gate_rejected_frame_rate"], 1.0)

    def test_observed_id_switch_is_separated_from_aim_id_switch(self):
        samples = []
        for index in range(30):
            sample = make_sample(index, 0.05, 1.4, -0.3, index % 2)
            sample.update(
                {
                    "aim_armor_id": 1,
                    "association_primary_id": 1,
                }
            )
            samples.append(sample)

        phase = analyzer.analyze_samples(samples)["phases"]["static"]

        self.assertEqual(phase["id_switch_count"], 29)
        self.assertEqual(phase["aim_id_switch_count"], 0)
        self.assertEqual(phase["association_primary_switch_count"], 0)


if __name__ == "__main__":
    unittest.main()
