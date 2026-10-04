import unittest

from tools.real_tracker_analyzer import analyze_records


class RealTrackerAnalyzerTest(unittest.TestCase):
    def test_reports_latency_and_skip_reason(self):
        records = [
            {
                "event": "frame",
                "frame_id": 1,
                "device_ticks": 100,
                "tick_hz": 1000,
                "timestamp_source": "device_clock",
                "mapped_age_ms": 20.0,
                "mapping_delay_ms": 5.0,
                "bracket_ms": 10.0,
                "detected": 2,
                "targets": 1,
                "tracker_state": "tracking",
                "center_speed": 0.1,
                "prediction_dt": 0.03,
                "diagnostic_yaw_deg": 2.0,
            },
            {
                "event": "skip",
                "frame_id": 2,
                "device_ticks": 110,
                "tick_hz": 1000,
                "timestamp_source": "device_clock",
                "skip_reason": "no fresh feedback on both sides of image time",
            },
        ]
        report = analyze_records(records)
        self.assertEqual(report["processed_frames"], 1)
        self.assertEqual(report["skipped_frames"], 1)
        self.assertEqual(report["timestamp"]["mapped_age_ms"]["p95"], 20.0)
        self.assertEqual(report["pipeline"]["detection_rate"], 1.0)
        self.assertIn("no fresh feedback on both sides of image time", report["skip_reasons"])
        self.assertEqual(report["timestamp"]["sources"]["device_clock"], 2)

    def test_warns_about_host_receive_timestamp(self):
        report = analyze_records([
            {
                "event": "frame",
                "device_ticks": 0,
                "timestamp_source": "host_receive",
                "mapped_age_ms": 10.0,
            }
        ])
        self.assertEqual(report["timestamp"]["sources"]["host_receive"], 1)
        self.assertTrue(any("主机收帧时间" in warning for warning in report["warnings"]))

    def test_warns_about_estimated_device_clock(self):
        report = analyze_records([
            {
                "event": "frame",
                "device_ticks": 100,
                "tick_hz": 100000000,
                "timestamp_source": "estimated_device_clock",
            }
        ])
        self.assertEqual(report["timestamp"]["sources"]["estimated_device_clock"], 1)
        self.assertTrue(any("估计的设备时钟" in warning for warning in report["warnings"]))

    def test_detects_timestamp_regression(self):
        records = [
            {"event": "frame", "frame_id": 3, "device_ticks": 200, "tick_hz": 1000},
            {"event": "frame", "frame_id": 2, "device_ticks": 199, "tick_hz": 1000},
        ]
        report = analyze_records(records)
        self.assertEqual(report["timestamp"]["tick_regressions"], 1)
        self.assertEqual(report["timestamp"]["frame_regressions"], 1)
        self.assertTrue(report["warnings"])


if __name__ == "__main__":
    unittest.main()
