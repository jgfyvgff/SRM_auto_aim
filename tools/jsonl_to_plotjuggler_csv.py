#!/usr/bin/env python3
"""把 standard_srm 的逐帧 JSONL 转成 PlotJuggler 可直接打开的 CSV。

为什么需要：PlotJuggler 4.0.1 在这台机器上是独立版（没有 ROS 插件），最省事的入口是
CSV。日志里 planner 的量本来就记全了（角度、角速度、求解器残差、实测姿态），只是
JSONL 不是时间序列格式，PlotJuggler 打不开。

三个约定，都是为了让 PlotJuggler 的 CSV 入口能正确解析：
- 时间轴复用 real_tracker_analyzer.frame_time_axis_ms，与盲区统计同一基准且保证单调；
  用帧序号当横轴会把"控制周期变了"误读成"时间没变"。
- 缺失值写 nan。planner 在没有目标时字段是 null，画成断点比填 0 更接近事实。
- 枚举字符串写成整数码（CSV 入口只画数值列），码表写到 <output>.legend.txt。
"""

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.real_tracker_analyzer import (  # noqa: E402
    DEFAULT_FRAME_PERIOD_MS,
    frame_time_axis_ms,
    load_records,
)

# 默认导出的一组列：调 planner 时关心的是"命令 / 目标 / 角速度 / 实测 / 求解器残差"，
# 把 80 多个字段全画上去反而看不出问题。需要全量时用 --preset all。
PLANNER_COLUMNS = (
    # planner 输出与 MPC 目标：两者的差就是规划跟踪误差。
    "planner_yaw_deg",
    "planner_pitch_deg",
    "planner_target_yaw_deg",
    "planner_target_pitch_deg",
    # 角速度：planner 期望的（MPC 速度项）与云台实测的，放在一起看滞后与超调。
    "planner_yaw_vel_rad_s",
    "planner_pitch_vel_rad_s",
    "planner_measured_yaw_vel_rad_s",
    "planner_measured_pitch_vel_rad_s",
    # planner 内部状态与云台实测角度。
    "planner_state_yaw_deg",
    "planner_state_pitch_deg",
    "planner_measured_yaw_deg",
    "planner_measured_pitch_deg",
    # 求解器：状态码/迭代次数/原始对偶残差，用来判断 MPC 是不是真的收敛。
    "planner_yaw_solver_status",
    "planner_pitch_solver_status",
    "planner_yaw_solver_iterations",
    "planner_pitch_solver_iterations",
    "planner_yaw_primal_residual_max",
    "planner_pitch_primal_residual_max",
    "planner_yaw_dual_residual_max",
    "planner_pitch_dual_residual_max",
    "planner_solver_converged",
    "planner_control",
    "planner_ms",
    # 实际下发的指令：与 planner 输出对比可看出限幅/保持/外推是否介入。
    "tx_command_yaw_deg",
    "tx_command_pitch_deg",
    "tx_command_sent",
    # 上下文：判断某段异常是检测、跟踪还是链路问题。
    "detected",
    "max_confidence",
    "temp_lost_count",
    "radius",
    "diagnostic_yaw_deg",
    "diagnostic_pitch_deg",
    "planner_bullet_speed_mps",
    "planner_feedback_age_ms",
    "planner_feedback_interval_ms",
    "detector_ms",
    "tracker_ms",
    "feedback_wait_ms",
)

# 枚举列：值本身是字符串，PlotJuggler 的 CSV 入口只画数值，所以转成整数码。
ENUM_COLUMNS = (
    "tracker_state",
    "tx_command_mode",
    "planner_status",
    "planner_speed_source",
    "planner_state_source",
)

PRESETS = ("planner", "all")


def scalar(value):
    """把日志值转成数值；字符串、列表、字典、None 一律视为缺失。

    布尔必须排在 int 前面判断，否则 True 会被当成 1 而不是"是否"。
    """
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        return float(value)
    return math.nan


def format_number(value):
    """格式化一个数值；nan 直接写 nan，PlotJuggler 会画成断点。"""
    if not math.isfinite(value):
        return "nan"
    return f"{value:.8g}"


def build_legend(records, enum_columns=ENUM_COLUMNS):
    """给枚举列分配整数码（0 保留给缺失）。

    按值的字母序固定分配，而不是按出现顺序：这样同一份日志多次转换结果一致，
    也便于和 <output>.legend.txt 对照。
    """
    legend = {}
    for column in enum_columns:
        values = sorted(
            {
                record[column]
                for record in records
                if isinstance(record.get(column), str) and record[column] != ""
            }
        )
        legend[column] = {value: index + 1 for index, value in enumerate(values)}
    return legend


def select_columns(records, preset="planner"):
    """选择要导出的数值列。

    preset=all 时只保留"至少在一帧里是数值标量"的字段：角点像素坐标之类的列表字段
    展开会炸成上百列，对调 planner 没有帮助。
    """
    if preset not in PRESETS:
        raise ValueError(f"unknown preset: {preset}")

    if preset == "planner":
        present = set()
        for record in records:
            present.update(record.keys())
        return tuple(column for column in PLANNER_COLUMNS if column in present)

    columns = set()
    for record in records:
        for key, value in record.items():
            if isinstance(value, bool) or isinstance(value, (int, float)):
                columns.add(key)
    return tuple(sorted(columns))


def build_table(records, columns, enum_columns=ENUM_COLUMNS, legend=None):
    """生成 (表头, 数据行)；列顺序固定，便于对比两次转换的结果。"""
    if legend is None:
        legend = build_legend(records, enum_columns)
    axis_ms = frame_time_axis_ms(records, DEFAULT_FRAME_PERIOD_MS)

    header = ["time_s", "frame"]
    header += list(columns)
    header += [f"{column}_code" for column in enum_columns]

    rows = []
    for index, record in enumerate(records):
        row = [format_number(axis_ms[index] / 1000.0), str(index)]
        row += [format_number(scalar(record.get(column))) for column in columns]
        row += [
            str(legend[column].get(record.get(column), 0)) for column in enum_columns
        ]
        rows.append(row)
    return header, rows


def write_csv(path, header, rows):
    """原子替换写入：先写临时文件再 rename，避免 --follow 时 PlotJuggler 读到半个文件。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)
    os.replace(temporary, path)


def write_legend(path, legend):
    path = Path(path)
    lines = ["# PlotJuggler 枚举码表（0 表示该帧没有这个字段）", ""]
    for column in sorted(legend):
        lines.append(f"[{column}]")
        mapping = legend[column]
        if not mapping:
            lines.append("  （日志里没有出现该字段）")
        for value, code in sorted(mapping.items(), key=lambda item: item[1]):
            lines.append(f"  {code} = {value}")
        lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def convert(input_path, output_path, preset="planner"):
    """转换一次，返回 (帧数, 列数)。供离线与 --follow 两种模式共用。"""
    records, invalid = load_records(input_path)
    if not records:
        return 0, 0, invalid
    columns = select_columns(records, preset)
    legend = build_legend(records, ENUM_COLUMNS)
    header, rows = build_table(records, columns, ENUM_COLUMNS, legend)
    write_csv(output_path, header, rows)
    write_legend(str(output_path) + ".legend.txt", legend)
    return len(rows), len(header), invalid


def follow(input_path, output_path, preset, interval_s):
    """边跑边看：周期性重新生成整份 CSV。

    每次都重新生成而不是追加：时间轴来自设备 tick，增量写要额外维护状态，
    而 PlotJuggler 每次刷新都要重读整个文件，增量写并不会更快。
    长时间采集（几万帧）时刷新会变慢，这时代价可控但需要知道这一点。
    """
    print(f"每 {interval_s:.1f}s 重新生成一次；在 PlotJuggler 里按 Ctrl+R 刷新")
    while True:
        try:
            frames, columns, invalid = convert(input_path, output_path, preset)
            stamp = time.strftime("%H:%M:%S")
            print(f"  [{stamp}] {frames} 帧 / {columns} 列" + (f"（跳过 {invalid} 行）" if invalid else ""))
            time.sleep(interval_s)
        except KeyboardInterrupt:
            print("\n停止")
            return 0


def main():
    parser = argparse.ArgumentParser(description="JSONL 转 PlotJuggler CSV（默认只导出 planner 相关列）")
    parser.add_argument("--input", required=True, help="standard_srm 的 --debug-jsonl 输出")
    parser.add_argument("--output", help="输出 CSV；默认与输入同名 .csv")
    parser.add_argument("--preset", choices=PRESETS, default="planner", help="planner=常用一组，all=全部数值标量")
    parser.add_argument("--follow", action="store_true", help="周期性重新生成，用于边跑边看")
    parser.add_argument("--interval", type=float, default=2.0, help="--follow 的刷新间隔秒数")
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.is_file():
        print(f"输入不存在: {input_path}", file=sys.stderr)
        return 1
    output_path = Path(args.output) if args.output else input_path.with_suffix(".csv")

    if args.follow:
        return follow(input_path, output_path, args.preset, args.interval)

    frames, columns, invalid = convert(input_path, output_path, args.preset)
    if frames == 0:
        print("没有可用记录", file=sys.stderr)
        return 2
    print(f"已写出 {output_path}：{frames} 帧 × {columns} 列（预设 {args.preset}）")
    if invalid:
        print(f"跳过 {invalid} 行非法 JSON")
    print(f"码表: {output_path}.legend.txt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
