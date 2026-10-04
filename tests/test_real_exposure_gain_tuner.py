import unittest

from tools.real_exposure_gain_tuner import evaluate_records


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


if __name__ == "__main__":
    unittest.main()
