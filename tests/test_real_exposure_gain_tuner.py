import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from tools.real_exposure_gain_tuner import (
    candidate_settings,
    evaluate_records,
    fit_response_surface,
    invalid_reason,
    main,
    predicted_optimum,
    propose_next_setting,
    run_setting,
    summarize_confirmation,
)


class RealExposureGainTunerTest(unittest.TestCase):
    def test_good_detection_beats_bad_detection(self):
        good = [
            {
                "captured_frame": index,
                "event": "frame",
                "detected": 1,
                "mean_confidence": 0.95,
                "image_dark_ratio": 0.01,
                "image_bright_ratio": 0.01,
                "image_laplacian_variance": 300.0,
            }
            for index in range(1, 11)
        ]
        bad = [
            {
                "captured_frame": index,
                "event": "frame",
                "detected": 0,
                "mean_confidence": 0.0,
                "image_dark_ratio": 0.20,
                "image_bright_ratio": 0.10,
                "image_laplacian_variance": 500.0,
            }
            for index in range(1, 11)
        ]
        self.assertGreater(
            evaluate_records(good, warmup_frames=0)["score"],
            evaluate_records(bad, warmup_frames=0)["score"],
        )

    def test_reports_missing_feedback(self):
        report = evaluate_records(
            [
                {
                    "captured_frame": 1,
                    "event": "skip",
                    "skip_reason": "no fresh feedback",
                    "image_dark_ratio": 0.01,
                    "image_bright_ratio": 0.01,
                    "image_laplacian_variance": 100.0,
                }
            ],
            warmup_frames=0,
        )
        self.assertEqual(report["processed_frames"], 0)
        self.assertTrue(report["warning"])

    def test_armor_specific_diagnostics_are_reported(self):
        records = [
            {
                "captured_frame": index,
                "event": "frame",
                "detected": 1 if index in (1, 5) else 0,
                "pnp_reprojection_errors_px": [
                    {"pnp_reprojection_error_px": 0.2 * index}
                ] if index in (1, 5) else [],
            }
            for index in range(1, 6)
        ]
        result = evaluate_records(records, warmup_frames=0)
        self.assertEqual(result["longest_miss_streak"], 3)
        self.assertAlmostEqual(result["pnp_reprojection_p95_px"], 0.96)

    def test_invalid_acquisition_is_not_a_camera_optimum(self):
        sample = {"return_code": 0, "timeout": False, "processed_frames": 30}
        self.assertEqual(invalid_reason(sample, 100, 0.7),
                         "有效检测帧不足；不能把串口/时间戳故障当作曝光效果")
        sample["processed_frames"] = 80
        self.assertEqual(invalid_reason(sample, 100, 0.7), "")
        sample["timeout"] = True
        self.assertEqual(invalid_reason(sample, 100, 0.7), "采集超时")

    def test_surface_suggests_unmeasured_interior_point(self):
        bounds = (6.0, 8.0, 8.0, 16.0)
        results = [
            {
                "exposure_ms": exposure,
                "gain": gain,
                "score": 0.9 - 0.04 * (exposure - 7.2) ** 2 -
                         0.002 * (gain - 11.0) ** 2,
                "eligible": True,
            }
            for exposure in (6.0, 7.0, 8.0)
            for gain in (8.0, 12.0, 16.0)
        ]
        model = fit_response_surface(results, bounds)
        self.assertIsNotNone(model)
        optimum = predicted_optimum(model, bounds, 0.1, 0.5)
        self.assertLess(abs(optimum["exposure_ms"] - 7.2), 0.4)
        self.assertLess(abs(optimum["gain"] - 11.0), 2.0)
        next_setting = propose_next_setting(results, bounds, 0.1, 0.5, model)
        self.assertNotIn(next_setting, {
            (item["exposure_ms"], item["gain"]) for item in results
        })
        self.assertTrue(bounds[0] <= next_setting[0] <= bounds[1])
        self.assertTrue(bounds[2] <= next_setting[1] <= bounds[3])

        # 异常高分的无效采样不能拉偏拟合曲面。
        invalid = {"exposure_ms": 8.0, "gain": 16.0, "score": 100.0,
                   "eligible": False}
        unchanged = fit_response_surface(results + [invalid], bounds)
        self.assertEqual(model["coefficients"], unchanged["coefficients"])

    def test_candidate_bounds_and_no_curve_fallback(self):
        bounds = (6.0, 8.0, 8.0, 16.0)
        settings = candidate_settings(bounds, 0.1, 0.5)
        self.assertTrue(settings)
        self.assertTrue(all(6.0 <= exposure <= 8.0 and 8.0 <= gain <= 16.0
                            for exposure, gain in settings))
        self.assertNotEqual(
            propose_next_setting([
                {"exposure_ms": 7.0, "gain": 12.0}
            ], bounds, 0.1, 0.5, None), (7.0, 12.0)
        )

    def test_confirmation_requires_three_valid_runs(self):
        setting = (7.1, 11.5)
        runs = [
            {"eligible": True, "score": value, "detection_rate": 0.9,
             "longest_miss_streak": 2}
            for value in (0.8, 0.82, 0.78)
        ]
        summary = summarize_confirmation(setting, runs, 3)
        self.assertTrue(summary["verified"])
        self.assertEqual(summary["score_min"], 0.78)
        self.assertFalse(summarize_confirmation(setting, runs[:2], 3)["verified"])
        runs[1]["eligible"] = False
        self.assertFalse(summarize_confirmation(setting, runs, 3)["verified"])
        runs[1]["eligible"] = True
        runs[1]["detection_rate"] = 0.0
        self.assertFalse(summarize_confirmation(setting, runs, 3)["verified"])

    def test_adaptive_cli_runs_without_real_hardware(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "report.json"
            arguments = [
                "real_exposure_gain_tuner.py", "--repo", temporary,
                "--binary", "/bin/true", "--config", "/dev/null",
                "--exposures", "6,7,8", "--gains", "8,12,16",
                "--frames", "10", "--warmup-frames", "0",
                "--adaptive", "--adaptive-steps", "2",
                "--confirm-top", "2", "--confirm-runs", "3",
                "--artifacts", temporary, "--output", str(output),
            ]

            def fake_run_setting(args, repo, exposure, gain, run_index):
                return {
                    "exposure_ms": exposure, "gain": gain,
                    "score": 0.9 - 0.04 * (exposure - 7.2) ** 2 -
                             0.002 * (gain - 11.0) ** 2,
                    "detection_rate": 0.95,
                    "mean_confidence": 0.9,
                    "longest_miss_streak": 1,
                    "eligible": True,
                    "invalid_reason": "",
                }

            with patch.object(sys, "argv", arguments), patch(
                "tools.real_exposure_gain_tuner.run_setting",
                side_effect=fake_run_setting,
            ), contextlib.redirect_stdout(io.StringIO()):
                main()
            report = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(len(report["results"]), 9 + 2 + 2 * 3)
            self.assertIsNotNone(report["response_surface"])
            self.assertIsNotNone(report["predicted_optimum"])
            self.assertTrue(report["recommendation"]["verified"])
            self.assertEqual(len(report["confirmation"]), 2)
            self.assertTrue(Path(report["artifacts_session"]).is_dir())

    def test_adaptive_refuses_to_overwrite_report_before_sampling(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "existing.json"
            output.write_text("old report", encoding="utf-8")
            arguments = [
                "real_exposure_gain_tuner.py", "--repo", temporary,
                "--exposures", "6,7", "--gains", "8,12",
                "--adaptive", "--artifacts", temporary, "--output", str(output),
            ]
            with patch.object(sys, "argv", arguments), patch(
                "tools.real_exposure_gain_tuner.run_setting"
            ) as fake_runner, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    main()
            fake_runner.assert_not_called()
            self.assertEqual(output.read_text(encoding="utf-8"), "old report")

    def test_subprocess_without_samples_is_not_eligible(self):
        with tempfile.TemporaryDirectory() as temporary:
            args = SimpleNamespace(
                artifacts=Path(temporary), binary=Path("/bin/true"),
                config=Path("/dev/null"), port="/dev/null", frames=10,
                warmup_frames=0, min_valid_fraction=0.7,
                timeout=2.0, keep_frames=False,
            )
            result = run_setting(args, Path(temporary), 7.0, 12.0, 1)
            self.assertEqual(result["return_code"], 0)
            self.assertEqual(result["processed_frames"], 0)
            self.assertFalse(result["eligible"])
            self.assertTrue(Path(result["process_log"]).is_file())


if __name__ == "__main__":
    unittest.main()
