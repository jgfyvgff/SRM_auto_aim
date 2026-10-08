import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.jsonl_to_plotjuggler_csv import (
    ENUM_COLUMNS,
    build_legend,
    build_table,
    convert,
    format_number,
    scalar,
    select_columns,
)


def sample_records():
    """两帧最小样本：一帧有 planner 输出、一帧为 null，覆盖缺失与枚举两种路径。"""
    return [
        {
            "event": "frame",
            "device_ticks": 0,
            "tick_hz": 100.0,
            "planner_yaw_deg": 1.5,
            "planner_solver_converged": True,
            "tracker_state": "tracking",
            "rx": 1.0,
        },
        {
            "event": "frame",
            "device_ticks": 10,
            "tick_hz": 100.0,
            "planner_yaw_deg": None,
            "planner_solver_converged": False,
            "tracker_state": "lost",
            "rx": 2.0,
        },
    ]


class ScalarTest(unittest.TestCase):
    def test_bool_before_int(self):
        # True 也是 int，必须先判 bool，否则"是否收敛"会被画成 1 而不是 0/1 两种状态。
        self.assertEqual(scalar(True), 1.0)
        self.assertEqual(scalar(False), 0.0)

    def test_numbers_pass_through(self):
        self.assertEqual(scalar(3), 3.0)
        self.assertAlmostEqual(scalar(-2.5), -2.5)

    def test_missing_becomes_nan(self):
        for value in (None, "tracking", [1, 2], {"a": 1}):
            self.assertTrue(math.isnan(scalar(value)), msg=repr(value))

    def test_format_nan_is_literal(self):
        # 写 nan 而不是空串：空串在部分解析器里会被当成 0，会把"没有规划"画成"规划为零"。
        self.assertEqual(format_number(math.nan), "nan")
        self.assertEqual(format_number(float("inf")), "nan")
        self.assertEqual(format_number(0.0), "0")
        self.assertEqual(format_number(1.5), "1.5")


class LegendTest(unittest.TestCase):
    def test_alphabetical_and_deterministic(self):
        records = sample_records()
        legend = build_legend(records, ("tracker_state",))
        # 按字母序固定分配，与出现顺序无关：lost 在前也是 1。
        self.assertEqual(legend["tracker_state"], {"lost": 1, "tracking": 2})
        self.assertEqual(legend, build_legend(records, ("tracker_state",)))

    def test_missing_column_is_empty_mapping(self):
        legend = build_legend(sample_records(), ("planner_status",))
        self.assertEqual(legend["planner_status"], {})


class SelectColumnsTest(unittest.TestCase):
    def test_planner_preset_only_present_columns(self):
        columns = select_columns(sample_records(), "planner")
        self.assertIn("planner_yaw_deg", columns)
        # 日志里没有的列不导出，避免整列都是 nan。
        self.assertNotIn("planner_pitch_deg", columns)

    def test_all_preset_skips_lists(self):
        records = [{"a": 1.0, "b": [1.0, 2.0], "c": True, "d": "text"}]
        columns = select_columns(records, "all")
        self.assertEqual(set(columns), {"a", "c"})

    def test_unknown_preset_raises(self):
        with self.assertRaises(ValueError):
            select_columns(sample_records(), "nope")


class TableTest(unittest.TestCase):
    def test_header_and_row_width_match(self):
        header, rows = build_table(sample_records(), ("planner_yaw_deg",))
        self.assertEqual(header[:3], ["time_s", "frame", "planner_yaw_deg"])
        # 枚举码列固定追加在最后，顺序与 ENUM_COLUMNS 一致。
        self.assertEqual(header[3:], [f"{column}_code" for column in ENUM_COLUMNS])
        for row in rows:
            self.assertEqual(len(row), len(header))

    def test_time_axis_is_monotonic_seconds(self):
        _, rows = build_table(sample_records(), ())
        self.assertEqual(rows[0][0], "0")
        self.assertEqual(rows[1][0], "0.1")

    def test_missing_numeric_written_as_nan(self):
        _, rows = build_table(sample_records(), ("planner_yaw_deg",))
        self.assertEqual(rows[0][2], "1.5")
        self.assertEqual(rows[1][2], "nan")

    def test_enum_codes_follow_legend(self):
        header, rows = build_table(sample_records(), ("planner_yaw_deg",))
        index = header.index("tracker_state_code")
        self.assertEqual(rows[0][index], "2")  # tracking
        self.assertEqual(rows[1][index], "1")  # lost
        # 日志里没有的枚举列一律写 0，与"该帧没有这个字段"区分于任何真实取值。
        empty_index = header.index("planner_status_code")
        self.assertEqual(rows[0][empty_index], "0")

    def test_bool_column_written_as_one_zero(self):
        _, rows = build_table(sample_records(), ("planner_solver_converged",))
        self.assertEqual(rows[0][2], "1")
        self.assertEqual(rows[1][2], "0")


class ConvertTest(unittest.TestCase):
    def test_writes_csv_and_legend(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "run.jsonl"
            lines = [
                '{"event":"frame","device_ticks":0,"tick_hz":100.0,"planner_yaw_deg":1.0,'
                '"tracker_state":"tracking"}',
                '{"event":"frame","device_ticks":10,"tick_hz":100.0,"planner_yaw_deg":2.0,'
                '"tracker_state":"tracking"}',
                "这不是 JSON",
            ]
            source.write_text("\n".join(lines), encoding="utf-8")

            frames, columns, invalid = convert(source, Path(directory) / "run.csv", "planner")
            self.assertEqual((frames, invalid), (2, 1))
            self.assertGreater(columns, 3)

            # legend 文件名必须与实际写出的文件一致，否则用户按提示找不到码表。
            legend_path = Path(directory) / "run.csv.legend.txt"
            self.assertTrue(legend_path.is_file())
            self.assertIn("1 = tracking", legend_path.read_text(encoding="utf-8"))

            with (Path(directory) / "run.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.reader(handle))
            self.assertEqual(len(rows), 3)  # 表头 + 2 帧
            self.assertEqual(rows[0][:2], ["time_s", "frame"])
            self.assertEqual(rows[2][0], "0.1")


if __name__ == "__main__":
    unittest.main()
