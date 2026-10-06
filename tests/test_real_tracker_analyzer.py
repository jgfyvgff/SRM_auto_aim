import sys
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT))

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

    def test_warns_about_read_path_throttling(self):
        records = [
            {"event": "frame", "device_ticks": 100 + 10 * i, "tick_hz": 1000, "bracket_ms": value}
            for i, value in enumerate([24.0, 7.3, 24.0, 7.3])
        ]
        report = analyze_records(records)
        self.assertAlmostEqual(report["timestamp"]["bracket_short_ratio"], 0.5)
        self.assertTrue(any("读路径节流" in warning for warning in report["warnings"]))

    def test_accepts_fast_feedback_after_read_fix(self):
        # 读路径修好后样本间隔会落到单帧量级（这里取 1 个字节时间）：
        # 即使格点占比和 <20ms 占比都是 100%，也不应报读路径节流。
        records = [
            {"event": "frame", "device_ticks": 100 + 10 * i, "tick_hz": 1000, "bracket_ms": 1.0417}
            for i in range(21)
        ]
        report = analyze_records(records)
        self.assertEqual(report["timestamp"]["bracket_short_ratio"], 1.0)
        self.assertAlmostEqual(report["timestamp"]["bracket_lattice_share"], 1.0)
        self.assertFalse(any("读路径节流" in warning for warning in report["warnings"]))


    def test_detects_serial_byte_time_lattice(self):
        # 23.96ms = 23 × 1.0417ms（USB CDC 下库默认 9600 波特率的字节时间）
        records = [
            {"event": "frame", "device_ticks": 100 + 10 * i, "tick_hz": 1000, "bracket_ms": 23.96}
            for i in range(20)
        ]
        report = analyze_records(records)
        self.assertAlmostEqual(report["timestamp"]["bracket_lattice_share"], 1.0)
        self.assertAlmostEqual(report["timestamp"]["bracket_byte_time_ms"], 1.0417, places=3)
        self.assertTrue(any("字节时间格点" in warning for warning in report["warnings"]))

    def test_accepts_off_lattice_arrival_intervals(self):
        records = [
            {"event": "frame", "device_ticks": 100 + 10 * i, "tick_hz": 1000, "bracket_ms": value}
            for i, value in enumerate([8.0, 19.0, 3.0, 11.0])
        ]
        report = analyze_records(records)
        self.assertLess(report["timestamp"]["bracket_lattice_share"], 0.60)
        self.assertFalse(any("字节时间格点" in warning for warning in report["warnings"]))


    def test_reports_blind_gap_and_freeze_ratio(self):
        # 非 tracking 帧只发保持角：这些帧的占比就是云台停止跟随的占比。
        # 两个 temp_lost 帧都没有检测，说明盲区来自检测器而不是关联。
        records = []
        for index in range(3):
            records.append(
                {
                    "event": "frame",
                    "device_ticks": 100 + 10 * index,
                    "tick_hz": 1000,
                    "detected": 1,
                    "tracker_state": "tracking",
                    "tx_command_mode": "tracking",
                    "tx_command_yaw_deg": 1.0,
                }
            )
        for index in range(3, 5):
            records.append(
                {
                    "event": "frame",
                    "device_ticks": 100 + 10 * index,
                    "tick_hz": 1000,
                    "detected": 0,
                    "tracker_state": "temp_lost",
                    "tx_command_mode": "hold_last_aim",
                    "tx_command_yaw_deg": 1.0,
                }
            )
        records.append(
            {
                "event": "frame",
                "device_ticks": 150,
                "tick_hz": 1000,
                "detected": 1,
                "tracker_state": "tracking",
                "tx_command_mode": "tracking",
                "tx_command_yaw_deg": 1.0,
            }
        )
        report = analyze_records(records)
        continuity = report["continuity"]
        self.assertEqual(continuity["blind_frames"], 2)
        self.assertAlmostEqual(continuity["blind_ratio"], 2.0 / 6.0)
        self.assertEqual(continuity["blind_runs"], 1)
        self.assertEqual(continuity["longest_blind_frames"], 2)
        # 两帧盲区（10ms 间隔）+ 一个兜底周期 = 20ms
        self.assertAlmostEqual(continuity["longest_blind_ms"], 20.0)
        self.assertEqual(continuity["blind_frames_with_detection"], 0)
        self.assertEqual(continuity["run_causes"], {"no_detection": 1})
        self.assertTrue(any("云台保持帧占比" in warning for warning in report["warnings"]))
        self.assertTrue(any("没有任何敌方装甲板检测" in warning for warning in report["warnings"]))

    def test_classifies_blind_gap_causes(self):
        records = [
            {"event": "frame", "tracker_state": "temp_lost", "detected": 1,
             "association_matching_detection_count": 1, "association_candidate_count": 0},
            {"event": "frame", "tracker_state": "tracking", "detected": 1},
            {"event": "frame", "tracker_state": "temp_lost", "detected": 1,
             "association_matching_detection_count": 1, "association_candidate_count": 1,
             "association_accepted_count": 0, "association_primary_gate_passed": 0,
             "association_primary_position_gate_passed": 0},
            {"event": "frame", "tracker_state": "tracking", "detected": 1},
        ]
        report = analyze_records(records)
        self.assertEqual(
            report["continuity"]["run_causes"],
            {"detection_not_associated": 1, "candidate_rejected": 1},
        )
        self.assertEqual(report["continuity"]["blind_frames_with_detection"], 2)
        self.assertTrue(any("候选被全部拒绝" in warning for warning in report["warnings"]))
        self.assertFalse(any("没有任何敌方装甲板检测" in warning for warning in report["warnings"]))


    def test_reports_gate_and_posterior_rejection(self):
        records = [
            {"event": "frame", "tracker_state": "temp_lost", "detected": 1,
             "association_candidate_count": 1, "association_accepted_count": 0,
             "association_primary_gate_passed": 1,
             "association_primary_position_error": 0.5,
             "association_primary_mahalanobis_distance": 4.0},
            {"event": "frame", "tracker_state": "temp_lost", "detected": 1,
             "association_candidate_count": 1, "association_accepted_count": 0,
             "association_primary_gate_passed": 0,
             "association_primary_position_gate_passed": 0,
             "association_primary_distance_gate_passed": 1,
             "association_primary_mahalanobis_gate_passed": 1,
             "association_primary_score_gate_passed": 1,
             "association_primary_position_error": 0.9},
        ]
        report = analyze_records(records)
        association = report["association"]
        self.assertEqual(association["candidate_frames"], 2)
        self.assertEqual(association["accepted_frames"], 0)
        self.assertEqual(association["rejected_frames"], 2)
        self.assertEqual(association["gate_passed_but_rejected_frames"], 1)
        self.assertEqual(association["failing_gates"], {"position": 1})
        self.assertAlmostEqual(association["rejected_position_error"]["max"], 0.9)
        self.assertTrue(any("EKF 后验检查否决" in warning for warning in report["warnings"]))

    def test_reports_command_mode_and_resume_step(self):
        records = [
            {"event": "frame", "tracker_state": "tracking", "tx_command_mode": "tracking",
             "tx_command_yaw_deg": 10.0},
            {"event": "frame", "tracker_state": "temp_lost", "tx_command_mode": "hold_last_aim",
             "tx_command_yaw_deg": 10.0},
            {"event": "frame", "tracker_state": "tracking", "tx_command_mode": "tracking",
             "tx_command_yaw_deg": 12.0},
        ]
        report = analyze_records(records)
        jitter = report["jitter"]
        self.assertEqual(jitter["tx_command_mode_counts"], {"tracking": 2, "hold_last_aim": 1})
        self.assertEqual(jitter["hold_frames"], 1)
        self.assertAlmostEqual(jitter["hold_ratio"], 1.0 / 3.0)
        self.assertEqual(jitter["resume_step_deg"]["count"], 1)
        self.assertAlmostEqual(jitter["resume_step_deg"]["max"], 2.0)
        self.assertTrue(any("保持→跟随切换" in warning for warning in report["warnings"]))

    def test_reports_within_generation_radius_drift(self):
        # 跨世代重建会掩盖漂移，只有同一世代内的波动才是抖动来源。
        records = [
            {"event": "frame", "tracker_state": "tracking", "tracker_generation": 1,
             "radius": 0.1 if index % 2 == 0 else 0.3}
            for index in range(11)
        ]
        records += [
            {"event": "frame", "tracker_state": "tracking", "tracker_generation": 2, "radius": 0.5}
            for _ in range(3)
        ]
        report = analyze_records(records)
        stdev = report["jitter"]["radius_stdev_within_generation"]
        self.assertIsNotNone(stdev)
        self.assertGreater(stdev, 0.05)
        self.assertTrue(any("radius 波动" in warning for warning in report["warnings"]))

    def test_ignores_records_without_tracker_fields(self):
        report = analyze_records(
            [{"event": "frame", "device_ticks": 100, "tick_hz": 1000, "bracket_ms": 8.0}]
        )
        self.assertEqual(report["continuity"], {})
        self.assertEqual(report["jitter"]["tx_command_mode_counts"], {})
        self.assertEqual(report["association"]["candidate_frames"], 0)
        self.assertFalse(any("云台保持帧占比" in warning for warning in report["warnings"]))


    def test_reports_missing_diagnostic_fields(self):
        # 旧日志缺整组字段时必须显式提示，避免空段落被误读成"没有问题"。
        records = [
            {"event": "frame", "tracker_state": "tracking", "detected": 1}
            for _ in range(3)
        ]
        report = analyze_records(records)
        coverage = report["coverage"]
        self.assertEqual(coverage["盲区与保持帧"], 3)
        self.assertEqual(coverage["指令模式"], 0)
        self.assertEqual(coverage["关联拒绝归因"], 0)
        self.assertIn("指令模式", report["missing_fields"])
        self.assertIn("关联拒绝归因", report["missing_fields"])
        self.assertTrue(
            any("日志缺少" in warning and "指令模式" in warning for warning in report["warnings"])
        )

    def test_reports_full_coverage_for_new_logs(self):
        records = [
            {"event": "frame", "tracker_state": "tracking", "tx_command_mode": "tracking",
             "tx_command_yaw_deg": 1.0, "association_candidate_count": 1,
             "tracker_generation": 1, "radius": 0.2}
        ]
        report = analyze_records(records)
        self.assertEqual(report["missing_fields"], [])
        self.assertFalse(any("日志缺少" in warning for warning in report["warnings"]))


if __name__ == "__main__":
    unittest.main()
