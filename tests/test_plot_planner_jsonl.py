import math
import sys
import tempfile
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.plot_planner_jsonl import (
    PANELS,
    angle_series_from_records,
    plot_panels,
    required_yaw_step_deg,
    series_from_records,
    summarize_convergence,
    unwrap_deg_series,
    wrap_deg,
)


class SeriesTest(unittest.TestCase):
    def test_numbers_pass_through_and_missing_becomes_nan(self):
        records = [
            {"a": 1.5},
            {"a": None},
            {"a": "text"},
            {"a": [1, 2]},
            {"a": True},
            {"a": float("inf")},
        ]
        series = series_from_records(records, "a")
        self.assertAlmostEqual(series[0], 1.5)
        # 非数值与无穷都写 nan：JSONL 里 null 的语义是"这一帧没有值"，不是 0。
        for index in (1, 2, 3, 5):
            self.assertTrue(math.isnan(series[index]), msg=str(index))
        self.assertAlmostEqual(series[4], 1.0)


class WrapTest(unittest.TestCase):
    def test_shortest_arc(self):
        self.assertAlmostEqual(wrap_deg(0.0), 0.0)
        self.assertAlmostEqual(wrap_deg(179.0), 179.0)
        self.assertAlmostEqual(wrap_deg(-179.0), -179.0)
        self.assertAlmostEqual(wrap_deg(181.0), -179.0)
        self.assertAlmostEqual(wrap_deg(-181.0), 179.0)
        self.assertAlmostEqual(wrap_deg(360.0), 0.0)

    def test_nan_stays_nan(self):
        self.assertTrue(math.isnan(wrap_deg(math.nan)))


class UnwrapTest(unittest.TestCase):
    def test_crossing_180_is_continuous(self):
        # 实测里 target 稳定在 +174，而 measured 会从 +178 越界成 -176：
        # 直接画是两条相距 350° 的平线，解卷绕后应接成 178 → 184 → 186。
        series = unwrap_deg_series([178.0, -176.0, -174.0])
        self.assertAlmostEqual(series[0], 178.0)
        self.assertAlmostEqual(series[1], 184.0)
        self.assertAlmostEqual(series[2], 186.0)

    def test_nan_gap_keeps_last_reference(self):
        series = unwrap_deg_series([170.0, math.nan, -175.0])
        self.assertAlmostEqual(series[0], 170.0)
        self.assertTrue(math.isnan(series[1]))
        # 断点之后仍以最后一个有效值为基准接续，否则会重新从 -175 开始。
        self.assertAlmostEqual(series[2], 185.0)

    def test_already_continuous_is_unchanged(self):
        series = unwrap_deg_series([169.0, 170.0, 171.0])
        self.assertAlmostEqual(series[0], 169.0)
        self.assertAlmostEqual(series[2], 171.0)

    def test_angle_series_helper_reads_records(self):
        records = [{"planner_measured_yaw_deg": 178.0}, {"planner_measured_yaw_deg": -176.0}]
        series = angle_series_from_records(records, "planner_measured_yaw_deg")
        self.assertAlmostEqual(series[1], 184.0)
        # 解卷绕只用于绘图，不改变"需要转过的角度"这种按最短弧计算的统计口径：
        # 两个字段解卷绕后相差 6°，而原始最短弧也是 6°。
        self.assertAlmostEqual(required_yaw_step_deg(
            [{"planner_measured_yaw_deg": 178.0, "planner_target_yaw_deg": -176.0}]
        )[0], 6.0)


class RequiredStepTest(unittest.TestCase):
    def test_uses_shortest_arc_not_raw_difference(self):
        records = [{"planner_measured_yaw_deg": -179.0, "planner_target_yaw_deg": 179.0}]
        # 原始差是 358，跨过 ±180 的最短弧是 2；用错会让"需要转过的角度"整桶偏大。
        self.assertAlmostEqual(required_yaw_step_deg(records)[0], 2.0)

    def test_missing_side_is_nan(self):
        self.assertTrue(math.isnan(required_yaw_step_deg([{"planner_measured_yaw_deg": 1.0}])[0]))
        self.assertTrue(math.isnan(required_yaw_step_deg([{"planner_target_yaw_deg": 1.0}])[0]))


class SummarizeTest(unittest.TestCase):
    def setUp(self):
        self.records = [
            {
                "planner_yaw_deg": 1.0,
                "planner_solver_converged": True,
                "planner_yaw_solver_status": 0,
                "planner_pitch_solver_status": 0,
                "tracker_state": "tracking",
                "radius": 0.18,
                "planner_measured_yaw_deg": 0.0,
                "planner_target_yaw_deg": 0.2,
            },
            {
                "planner_yaw_deg": 2.0,
                "planner_solver_converged": False,
                "planner_yaw_solver_status": 1,
                "planner_pitch_solver_status": 1,
                "tracker_state": "temp_lost",
                "radius": 0.2765,
                "planner_measured_yaw_deg": 0.0,
                "planner_target_yaw_deg": 3.0,
            },
            {"planner_yaw_deg": None, "tracker_state": "lost"},
        ]

    def test_overview_counts(self):
        overview, _ = summarize_convergence(self.records)
        self.assertEqual(overview["frames"], 3)
        self.assertEqual(overview["planned"], 2)
        self.assertEqual(overview["converged"], 1)
        self.assertEqual(overview["yaw_status1"], 1)
        self.assertEqual(overview["pitch_status1"], 1)
        self.assertEqual(overview["tracking"], 1)
        # 只有一帧 radius 是 0.18，0.2765 不算标准四装甲。
        self.assertEqual(overview["standard_radius_frames"], 1)

    def test_buckets_split_by_required_step(self):
        _, rows = summarize_convergence(self.records)
        by_range = {(row["low"], row["high"]): row for row in rows}
        self.assertEqual(by_range[(0, 0.5)]["frames"], 1)
        self.assertEqual(by_range[(0, 0.5)]["unconverged"], 0)
        self.assertEqual(by_range[(2.0, 5.0)]["frames"], 1)
        self.assertEqual(by_range[(2.0, 5.0)]["unconverged"], 1)

    def test_no_plan_returns_empty(self):
        overview, rows = summarize_convergence([{"planner_yaw_deg": None}])
        self.assertEqual(overview, {})
        self.assertEqual(rows, [])


class PlotSmokeTest(unittest.TestCase):
    def test_writes_png_and_tolerates_missing_columns(self):
        # 只给一部分列：缺列的曲线必须跳过，而不是整张图失败。
        records = [
            {"device_ticks": 0, "tick_hz": 100.0, "planner_yaw_deg": 1.0},
            {"device_ticks": 10, "tick_hz": 100.0, "planner_yaw_deg": 2.0},
        ]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "plot.png"
            plot_panels(records, PANELS, output, "smoke", show=False, dpi=40)
            self.assertTrue(output.is_file())
            # 空图也会写出很小的文件，所以用尺寸兜底判断"确实画了东西"。
            self.assertGreater(output.stat().st_size, 2000)


if __name__ == "__main__":
    unittest.main()
