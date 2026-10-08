#!/usr/bin/env python3
"""把 standard_srm 的逐帧 JSONL 直接画成图（不依赖 PlotJuggler）。

为什么另做一份：PlotJuggler 4.0.1 这台构建没有 UDP 插件、命令行文件参数也不确定生效，
而调 planner 其实只需要"误差/角速度/求解器状态"这几张固定图。直接出 PNG 还带来两个好处：
- 可以通过 SSH 在无显示器的机器上生成（默认 Agg 后端），再拷回来看；
- 图和数字一起给出：图上是曲线，终端里同时打印收敛率与"需要转过的角度"分桶表。

默认 7 个面板（共用横轴）：
    1 Yaw 跟踪误差 target-measured [deg]      5 Yaw 求解器：状态 0/1 + 原始残差（对轴、对数）
    2 Yaw 角速度 MPC/实测 [rad/s]             6 Pitch 求解器：状态 0/1 + 原始残差
    3 Pitch 跟踪误差 [deg]                    7 需要转过的角度 |误差| + radius
    4 Pitch 角速度 MPC/实测

**为什么画误差而不是角度本身**：实测里 planner_target_yaw_deg 稳定在 +174，而
planner_measured_yaw_deg 会从 +178 越过 ±180 变成 -176——两者实际只差 12°，但直接画
绝对值就是两条相距 350° 的平线，看图的人会误判成跟踪完全失效。误差是同一个量的
最短弧表示，不受 360° 分支影响，所以默认画误差；要看原始角度用 --curves。

两种 0/1 状态也分开画：叠在一起时"哪一路没解出来"根本看不出来。

时间轴复用 real_tracker_analyzer.frame_time_axis_ms，与盲区统计同一基准且保证单调。
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.real_tracker_analyzer import (  # noqa: E402
    DEFAULT_FRAME_PERIOD_MS,
    frame_time_axis_ms,
    load_records,
)

# 每个面板：标题、曲线、可选的对轴曲线、可选的对数轴/状态填充。
# curves 里允许出现本模块派生的列（yaw_error_deg / pitch_error_deg / required_yaw_step_deg）。
PANELS = (
    {
        "title": "Yaw tracking error: target - measured  [deg]",
        "curves": ("yaw_error_deg",),
        "zero_line": True,
    },
    {
        "title": "Yaw rate: MPC / measured  [rad/s]",
        "curves": ("planner_yaw_vel_rad_s", "planner_measured_yaw_vel_rad_s"),
    },
    {
        "title": "Pitch tracking error: target - measured  [deg]",
        "curves": ("pitch_error_deg",),
        "zero_line": True,
    },
    {
        "title": "Pitch rate: MPC / measured  [rad/s]",
        "curves": ("planner_pitch_vel_rad_s", "planner_measured_pitch_vel_rad_s"),
    },
    {
        "title": "Yaw solver: status (0 solved, 1 max_iter) + primal residual",
        "curves": ("planner_yaw_solver_status",),
        "fill_status": True,
        "twin": ("planner_yaw_primal_residual_max",),
        "twin_log": True,
    },
    {
        "title": "Pitch solver: status (0 solved, 1 max_iter) + primal residual",
        "curves": ("planner_pitch_solver_status",),
        "fill_status": True,
        "twin": ("planner_pitch_primal_residual_max",),
        "twin_log": True,
    },
    {
        "title": "Required yaw step |target - measured| [deg] + radius [m]",
        "curves": ("required_yaw_step_deg",),
        "twin": ("radius",),
        "radius_lines": True,
    },
)

DERIVED_CURVES = ("yaw_error_deg", "pitch_error_deg", "required_yaw_step_deg")


def series_from_records(records, key):
    """取一列并转成 float 数组；缺失或非数值一律写 nan。

    写 nan 而不是 0：planner 在没有目标时字段是 null，画成断点才能看出"那一刻没有规划"。
    """
    values = []
    for record in records:
        value = record.get(key)
        if isinstance(value, bool):
            values.append(1.0 if value else 0.0)
        elif isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
        else:
            values.append(math.nan)
    return np.asarray(values, dtype=float)


def plot_time_axis_s(records, max_gap_s=1.0, fallback_ms=DEFAULT_FRAME_PERIOD_MS):
    """绘图用的时间轴（秒）：把超过 max_gap_s 的间隔压到 max_gap_s。

    日志里可能夹着长时间暂停（实测有一处 57 分钟），按真实设备时间画会把有数据的部分
    压成一条竖线，什么都看不出来。压缩后横轴表示"有效数据时间"，只在图注里注明；
    统计（盲区时长等）仍然用真实时间轴，不受这里影响。
    """
    raw = np.asarray(frame_time_axis_ms(records, fallback_ms), dtype=float) / 1000.0
    if raw.size == 0:
        return raw
    out = np.empty_like(raw)
    out[0] = raw[0]
    for index in range(1, raw.size):
        gap = max(0.0, float(raw[index] - raw[index - 1]))
        out[index] = out[index - 1] + (gap if max_gap_s <= 0 else min(gap, max_gap_s))
    return out


def wrap_deg(delta):
    """把角度差折到 (-180, 180]，避免 179° 与 -179° 之间被判成 358°。"""
    if not math.isfinite(delta):
        return math.nan
    return (delta + 180.0) % 360.0 - 180.0


def tracking_error_deg(records, target_key, measured_key):
    """跟踪误差 target - measured（deg，最短弧，带符号）。

    这是判断"为什么这一帧不收敛"最直接的量：实测数据里 |误差| ≤1° 时约 97% 的帧收敛，
    2~5° 时约 97% 的帧不收敛——不是求解器写得不好，而是参考本身是一次大机动。
    """
    target = series_from_records(records, target_key)
    measured = series_from_records(records, measured_key)
    return np.asarray(
        [wrap_deg(float(b) - float(a)) for a, b in zip(measured, target)], dtype=float
    )


def yaw_error_deg(records):
    return tracking_error_deg(records, "planner_target_yaw_deg", "planner_measured_yaw_deg")


def pitch_error_deg(records):
    return tracking_error_deg(records, "planner_target_pitch_deg", "planner_measured_pitch_deg")


def required_yaw_step_deg(records):
    """需要转过的角度 = |yaw 跟踪误差|。取绝对值是因为分桶只看量级。"""
    return np.abs(yaw_error_deg(records))


def summarize_convergence(
    records, buckets=((0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 5.0), (5.0, 180.0))
):
    """统计收敛率，并按"需要转过的角度"分桶。

    返回 (总览 dict, 分桶列表)。分桶是这次排查的核心证据，所以和出图放在一起，
    免得"看图还要另外跑一个脚本算数"。
    """
    planned = [
        record for record in records if isinstance(record.get("planner_yaw_deg"), (int, float))
    ]
    if not planned:
        return {}, []

    frames = len(records)
    converged = sum(1 for r in planned if r.get("planner_solver_converged") is True)
    yaw_bad = sum(1 for r in planned if r.get("planner_yaw_solver_status") == 1)
    pitch_bad = sum(1 for r in planned if r.get("planner_pitch_solver_status") == 1)
    tracking = sum(1 for r in records if r.get("tracker_state") == "tracking")
    radii = series_from_records(records, "radius")
    standard = int(np.sum(np.isclose(radii, 0.18, atol=1e-3)))

    overview = {
        "frames": frames,
        "planned": len(planned),
        "converged": converged,
        "converged_ratio": converged / len(planned),
        "yaw_status1": yaw_bad,
        "pitch_status1": pitch_bad,
        "tracking": tracking,
        "standard_radius_frames": standard,
    }

    steps = required_yaw_step_deg(records)
    rows = []
    for low, high in buckets:
        selected = [
            record
            for record, step in zip(records, steps)
            if isinstance(record.get("planner_yaw_deg"), (int, float))
            and math.isfinite(float(step))
            and low <= step < high
        ]
        if not selected:
            continue
        bad = sum(1 for r in selected if r.get("planner_yaw_solver_status") == 1)
        rows.append({"low": low, "high": high, "frames": len(selected), "unconverged": bad})
    return overview, rows


def print_summary(overview, rows):
    if not overview:
        print("没有 planner 输出，无法统计", file=sys.stderr)
        return
    print(f"总帧 {overview['frames']}，有 planner 输出 {overview['planned']}")
    print(
        f"  收敛 {overview['converged']} / {overview['planned']} "
        f"({overview['converged_ratio'] * 100:.1f}%)"
    )
    print(
        f"  yaw status=1 {overview['yaw_status1']}  "
        f"pitch status=1 {overview['pitch_status1']}"
    )
    print(
        f"  tracker=tracking {overview['tracking']}  "
        f"radius=0.18（标准四装甲）{overview['standard_radius_frames']} 帧"
    )
    if rows:
        print("\n  需要转过的角度 → 未收敛占比")
        for row in rows:
            print(
                f"    {row['low']:>5}~{row['high']:<5}deg  n={row['frames']:<5} "
                f"未收敛 {row['unconverged']:>5} ({row['unconverged'] / row['frames'] * 100:5.1f}%)"
            )


def plot_panels(records, panels, output, title, show, dpi, max_gap_s=1.0):
    """画多面板图；缺列的曲线直接跳过，不因为某个字段没有就整张失败。"""
    import matplotlib

    matplotlib.use("TkAgg" if show else "Agg")
    import matplotlib.pyplot as plt

    times = plot_time_axis_s(records, max_gap_s)
    derived = {
        "yaw_error_deg": yaw_error_deg(records),
        "pitch_error_deg": pitch_error_deg(records),
        "required_yaw_step_deg": required_yaw_step_deg(records),
    }

    def values_for(key):
        if key in derived:
            return derived[key]
        return series_from_records(records, key)

    figure, axes = plt.subplots(
        len(panels), 1, figsize=(14, 2.2 * len(panels)), sharex=True, squeeze=False
    )
    axes = [row[0] for row in axes]

    for axis, panel in zip(axes, panels):
        plotted = 0
        collected = []
        for key in panel["curves"]:
            values = values_for(key)
            if np.all(np.isnan(values)):
                continue
            axis.plot(times, values, label=key, linewidth=1.1)
            collected.append(values[np.isfinite(values)])
            plotted += 1
        axis.set_title(panel["title"], fontsize=10)
        axis.grid(alpha=0.3)
        if panel.get("zero_line"):
            axis.axhline(0.0, color="gray", linewidth=0.8)

        twin_keys = panel.get("twin", ())
        twin_values = []
        if twin_keys:
            twin = axis.twinx()
            for key in twin_keys:
                values = values_for(key)
                if np.all(np.isnan(values)):
                    continue
                twin.plot(times, values, color="tab:red", alpha=0.6, linewidth=1.0)
                twin_values.append(values[np.isfinite(values)])
                if panel.get("twin_log"):
                    twin.set_yscale("log")
            twin.set_ylabel(" / ".join(twin_keys), fontsize=8)
            twin.tick_params(labelsize=8)
            # radius 的 0.18 / 0.2765 参考线必须画在 radius 自己的轴上：
            # 画在左侧角度轴上会变成"0.18 度"这种没有意义的刻度。
            if panel.get("radius_lines") and twin_values:
                twin.axhline(0.18, color="gray", linestyle="--", linewidth=0.8)
                twin.axhline(0.2765, color="gray", linestyle=":", linewidth=0.8)

        if panel.get("fill_status"):
            # 未收敛是 0/1 阶梯，填充比细线更容易看出"哪一段没解出来"。
            for key in panel["curves"]:
                values = values_for(key)
                if np.all(np.isnan(values)):
                    continue
                axis.fill_between(
                    times, 0.0, np.nan_to_num(values), step="post", alpha=0.3
                )
            axis.set_ylim(-0.1, 1.1)
            axis.set_ylabel("status", fontsize=8)
            if plotted:
                axis.legend(loc="upper left", fontsize=8)
        elif collected:
            # 按 1%~99% 分位数设 y 轴：末尾几帧的突发会把整体压平，细节会看不见。
            stacked = np.concatenate(collected)
            low, high = np.percentile(stacked, [1, 99])
            if math.isfinite(low) and math.isfinite(high) and high > low:
                margin = 0.1 * (high - low)
                axis.set_ylim(low - margin, high + margin)
            axis.set_ylabel("deg" if "deg" in panel["title"] else "rad/s", fontsize=8)
            if plotted:
                axis.legend(loc="upper right", fontsize=8)

    axes[-1].set_xlabel("time [s] (gaps > {:g}s collapsed)".format(max_gap_s))
    figure.suptitle(title, fontsize=11)
    figure.tight_layout(rect=(0, 0, 1, 0.985))

    if show:
        plt.show()
    else:
        figure.savefig(output, dpi=dpi)
        print(f"\n图已保存: {output}")
    plt.close(figure)


def plot_all_curves(records, output, title, show, dpi, max_gap_s=1.0):
    """--preset all：把日志里所有数值标量列铺成网格图。"""
    columns = []
    for record in records:
        for key, value in record.items():
            if isinstance(value, bool) or isinstance(value, (int, float)):
                if key not in columns:
                    columns.append(key)
    panels = [{"title": key, "curves": (key,), "_raw": True} for key in sorted(columns)]

    import matplotlib

    matplotlib.use("TkAgg" if show else "Agg")
    import matplotlib.pyplot as plt

    times = plot_time_axis_s(records, max_gap_s)
    per_figure = 12
    for start in range(0, len(panels), per_figure):
        chunk = panels[start : start + per_figure]
        figure, axes = plt.subplots(
            len(chunk), 1, figsize=(14, 1.8 * len(chunk)), sharex=True, squeeze=False
        )
        for axis, panel in zip([row[0] for row in axes], chunk):
            axis.plot(times, series_from_records(records, panel["curves"][0]), linewidth=1.0)
            axis.set_title(panel["title"], fontsize=9)
            axis.grid(alpha=0.3)
        axes[-1][0].set_xlabel("time [s]")
        figure.suptitle(f"{title} ({start // per_figure + 1})", fontsize=11)
        figure.tight_layout(rect=(0, 0, 1, 0.985))
        if show:
            plt.show()
        else:
            suffix = output.with_name(f"{output.stem}_{start // per_figure + 1}{output.suffix}")
            figure.savefig(suffix, dpi=dpi)
            print(f"图已保存: {suffix}")
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description="把 standard_srm 的 JSONL 直接画成图")
    parser.add_argument("--input", required=True, help="standard_srm 的 --debug-jsonl 输出")
    parser.add_argument("--output", help="输出 PNG；默认与输入同名 .png")
    parser.add_argument("--preset", choices=("planner", "all"), default="planner")
    parser.add_argument("--curves", help="只画这些列（逗号分隔，可用派生列），覆盖 --preset")
    parser.add_argument("--title", help="图标题；默认用输入文件名")
    parser.add_argument("--show", action="store_true", help="弹出窗口（需要显示器）")
    parser.add_argument("--dpi", type=int, default=110)
    parser.add_argument(
        "--max-gap", type=float, default=1.0,
        help="绘图时间轴把超过该秒数的间隔压缩掉；0 表示保留真实时间（默认 1.0）",
    )
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.is_file():
        print(f"输入不存在: {input_path}", file=sys.stderr)
        return 1

    records, invalid = load_records(input_path)
    if not records:
        print("没有可用记录", file=sys.stderr)
        return 2

    output = Path(args.output) if args.output else input_path.with_suffix(".png")
    title = args.title or input_path.name

    overview, rows = summarize_convergence(records)
    print_summary(overview, rows)
    if invalid:
        print(f"（跳过 {invalid} 行非法 JSON）")

    if args.curves:
        keys = tuple(key.strip() for key in args.curves.split(",") if key.strip())
        plot_panels(
            records, [{"title": ", ".join(keys), "curves": keys}], output, title, args.show,
            args.dpi, args.max_gap
        )
    elif args.preset == "all":
        plot_all_curves(records, output, title, args.show, args.dpi, args.max_gap)
    else:
        plot_panels(records, PANELS, output, title, args.show, args.dpi, args.max_gap)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
