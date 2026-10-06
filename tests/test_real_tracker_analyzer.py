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


    def test_reports_wire_command_rate_from_serial_thread_counters(self):
        records = [
            {
                "event": "frame",
                "device_ticks": 100,
                "tick_hz": 1000,
                "tx_wire_target_frames": 10,
                "tx_wire_zero_frames": 0,
                "tx_command_sent": True,
            },
            {
                "event": "frame",
                "device_ticks": 1100,
                "tick_hz": 1000,
                "tx_wire_target_frames": 40,
                "tx_wire_zero_frames": 10,
                "tx_command_sent": True,
            },
        ]
        report = analyze_records(records)
        wire = report["control"]["wire_command"]
        self.assertEqual(wire["frames"], 40)
        self.assertAlmostEqual(wire["hz"], 40.0)
        self.assertAlmostEqual(wire["period_ms"], 25.0)
        self.assertEqual(report["control"]["mailbox_frames"], 2)

    def test_ignores_mailbox_flag_without_wire_counters(self):
        report = analyze_records([
            {"event": "frame", "device_ticks": 100, "tick_hz": 1000, "tx_command_sent": True}
        ])
        self.assertIsNone(report["control"]["wire_command"])
        self.assertEqual(report["control"]["mailbox_frames"], 1)

    def test_warns_about_bursty_feedback_arrival(self):
        records = [
            {"event": "frame", "device_ticks": 100 + 10 * i, "tick_hz": 1000, "bracket_ms": value}
            for i, value in enumerate([24.0, 7.3, 24.0, 7.3])
        ]
        report = analyze_records(records)
        self.assertAlmostEqual(report["timestamp"]["bracket_short_ratio"], 0.5)
        self.assertTrue(any("串口到达节律" in warning for warning in report["warnings"]))

    def test_accepts_regular_feedback_arrival(self):
        records = [
            {"event": "frame", "device_ticks": 100 + 10 * i, "tick_hz": 1000, "bracket_ms": 24.0}
            for i in range(20)
        ] + [{"event": "frame", "device_ticks": 400, "tick_hz": 1000, "bracket_ms": 7.3}]
        report = analyze_records(records)
        self.assertLess(report["timestamp"]["bracket_short_ratio"], 0.10)
        self.assertFalse(any("串口到达节律" in warning for warning in report["warnings"]))


if __name__ == "__main__":
    unittest.main()
