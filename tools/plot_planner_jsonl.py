#!/usr/bin/env python3
"""把 standard_srm 的逐帧 JSONL 直接画成图（不依赖 PlotJuggler）。

为什么另做一份：PlotJuggler 4.0.1 这台构建没有 UDP 插件、命令行文件参数也不确定生效，
而调 planner 其实只需要"角度/角速度/求解器状态"这几张固定图。直接出 PNG 还带来两个好处：
- 可以通过 SSH 在无显示器的机器上生成（默认 Agg 后端），再拷回来看；
- 图和数字一起给出：图上是曲线，终端里同时打印收敛率与"需要转过的角度"分桶表。

默认画 6 个面板（共用横轴）：
    1 Yaw 角度（参考 / MPC / 实测）      4 Pitch 角速度
    2 Yaw 角速度                          5 求解器：yaw/pitch 状态 + yaw 原始残差（对轴、对数）
    3 Pitch 角度                          6 需要转过的角度 |参考-实测| + radius（对轴）

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

# 每个面板：标题、曲线、是否按阶梯画、可选的对轴曲线。
PANELS = (
    {
        "title": "Yaw angle: target / MPC / measured  [deg]",
        "curves": ("planner_target_yaw_deg", "planner_yaw_deg", "planner_measured_yaw_deg"),
    },
    {
        "title": "Yaw rate: MPC / measured  [rad/s]",
        "curves": ("planner_yaw_vel_rad_s", "planner_measured_yaw_vel_rad_s"),
    },
    {
        "title": "Pitch angle: target / MPC / measured  [deg]",
        "curves": ("planner_target_pitch_deg", "planner_pitch_deg", "planner_measured_pitch_deg"),
    },
    {
        "title": "Pitch rate: MPC / measured  [rad/s]",
        "curves": ("planner_pitch_vel_rad_s", "planner_measured_pitch_vel_rad_s"),
    },
    {
        "title": "Solver: yaw/pitch status (0 solved, 1 max_iter) + yaw primal residual",
        "curves": ("planner_yaw_solver_status", "planner_pitch_solver_status"),
        "step": True,
        "twin": ("planner_yaw_primal_residual_max",),
        "twin_log": True,
    },
    {
        "title": "Required yaw step |target - measured| [deg] + radius [m]",
        "curves": ("required_yaw_step_deg",),
        "twin": ("radius",),
    },
)


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


def wrap_deg(delta):
    """把角度差折到 (-180, 180]，避免 179° 与 -179° 之间被判成 358°。"""
    if not math.isfinite(delta):
        return math.nan
    return (delta + 180.0) % 360.0 - 180.0


def unwrap_deg_series(values):
    """把跨 ±180 的角度序列解卷绕，避免图上出现假的 350° 大误差。

    实测数据里 planner_target_yaw_deg 稳定在 +174，而 planner_measured_yaw_deg 会从
    +178 越过 ±180 变成 -176：两者只差 12°，直接画出来却是两条相距 350° 的平线，
    看图的人会以为是跟踪完全失效。逐点按最短弧接起来才是"云台实际转到了哪"。
    """
    out = np.asarray(values, dtype=float).copy()
    reference = math.nan
    for index, value in enumerate(out):
        if not math.isfinite(value):
            continue
        if math.isfinite(reference):
            out[index] = reference + wrap_deg(float(value) - reference)
        reference = out[index]
    return out


def angle_series_from_records(records, key):
    """角度列：先取原始值，再解卷绕（仅用于绘图，不改变统计口径）。"""
    return unwrap_deg_series(series_from_records(records, key))


def required_yaw_step_deg(records):
    """云台当前 yaw 到瞄准参考 yaw 的夹角（deg）。

    这是判断"为什么这一帧不收敛"最直接的量：实测数据里这个角度 ≤1° 时约 98% 的帧收敛，
    2~5° 时约 98% 的帧不收敛——不是求解器写得不好，而是参考本身是一次大机动。
    """
    target = series_from_records(records, "planner_target_yaw_deg")
    measured = series_from_records(records, "planner_measured_yaw_deg")
    steps = [abs(wrap_deg(float(b) - float(a))) for a, b in zip(measured, target)]
    return np.asarray(steps, dtype=float)


def summarize_convergence(records, buckets=((0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 5.0), (5.0, 180.0))):
    """统计收敛率，并按"需要转过的角度"分桶。

    返回 (总览 dict, 分桶列表)。分桶是这次排查的核心证据，所以和出图放在一起，
    免得"看图还要另外跑一个脚本算数"。
    """
    planned = [
        record
        for record in records
        if isinstance(record.get("planner_yaw_deg"), (int, float))
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


def plot_panels(records, panels, output, title, show, dpi):
    """画多面板图；缺列的曲线直接跳过，不因为某个字段没有就整张失败。"""
    import matplotlib

    matplotlib.use("TkAgg" if show else "Agg")
    import matplotlib.pyplot as plt

    times = np.asarray(frame_time_axis_ms(records, DEFAULT_FRAME_PERIOD_MS), dtype=float) / 1000.0
    steps = required_yaw_step_deg(records)

    def values_for(key):
        if key == "required_yaw_step_deg":
            return steps
        # 角度列解卷绕后再画：否则跨 ±180 时会被读成几百度的误差。
        if key.endswith("_yaw_deg") or key.endswith("_pitch_deg"):
            return angle_series_from_records(records, key)
        return series_from_records(records, key)

    figure, axes = plt.subplots(
        len(panels), 1, figsize=(14, 2.4 * len(panels)), sharex=True, squeeze=False
    )
    axes = [row[0] for row in axes]

    for axis, panel in zip(axes, panels):
        plotted = 0
        collected = []
        for key in panel["curves"]:
            values = values_for(key)
            if np.all(np.isnan(values)):
                continue
            axis.plot(
                times, values, label=key,
                drawstyle="steps-post" if panel.get("step") else "default",
            )
            collected.append(values[np.isfinite(values)])
            plotted += 1
        axis.set_title(panel["title"], fontsize=10)
        axis.grid(alpha=0.3)
        if plotted:
            axis.legend(loc="upper right", fontsize=8)
        else:
            axis.text(0.5, 0.5, "no data", ha="center", va="center", transform=axis.transAxes)

        twin_keys = panel.get("twin", ())
        twin = None
        if twin_keys:
            twin = axis.twinx()
            twin_values = []
            for key in twin_keys:
                values = values_for(key)
                if np.all(np.isnan(values)):
                    continue
                twin.plot(times, values, label=key, color="tab:red", alpha=0.7, linewidth=1.0)
                twin_values.append(values[np.isfinite(values)])
                if panel.get("twin_log"):
                    twin.set_yscale("log")
            twin.set_ylabel(" / ".join(twin_keys), fontsize=8)
            twin.tick_params(labelsize=8)
            # radius 的 0.18 / 0.2765 参考线必须画在 radius 自己的轴上：
            # 画在左侧角度轴上会变成"0.18 度"这种没有意义的刻度。
            if not panel.get("twin_log") and "radius" in twin_keys and twin_values:
                twin.axhline(0.18, color="gray", linestyle="--", linewidth=0.8)
                twin.axhline(0.2765, color="gray", linestyle=":", linewidth=0.8)

        if panel.get("step"):
            # 未收敛是 0/1 阶梯，用填充把"哪一段没解出来"直接画成色块，比线更容易看。
            for key in panel["curves"]:
                values = values_for(key)
                if np.all(np.isnan(values)):
                    continue
                axis.fill_between(times, 0, np.nan_to_num(values), step="post", alpha=0.25)
            axis.set_ylim(-0.1, 1.1)
            axis.set_ylabel("status", fontsize=8)
        elif collected:
            # 按 1%~99% 分位数设 y 轴：末尾几帧的突发会把整体压平，导致真正的细节看不见。
            stacked = np.concatenate(collected)
            low, high = np.percentile(stacked, [1, 99])
            if math.isfinite(low) and math.isfinite(high) and high > low:
                margin = 0.1 * (high - low)
                axis.set_ylim(low - margin, high + margin)
            axis.set_ylabel("deg" if "deg" in panel["title"] else "rad/s", fontsize=8)

    axes[-1].set_xlabel("time [s]")
    figure.suptitle(title, fontsize=11)
    figure.tight_layout(rect=(0, 0, 1, 0.985))

    if show:
        plt.show()
    else:
        figure.savefig(output, dpi=dpi)
        print(f"\n图已保存: {output}")
    plt.close(figure)


def plot_all_curves(records, output, title, show, dpi):
    """--preset all：把日志里所有数值标量列铺成网格图。"""
    columns = []
    for record in records:
        for key, value in record.items():
            if isinstance(value, bool) or isinstance(value, (int, float)):
                if key not in columns:
                    columns.append(key)
    panels = [{"title": key, "curves": (key,)} for key in sorted(columns)]

    import matplotlib

    matplotlib.use("TkAgg" if show else "Agg")
    import matplotlib.pyplot as plt

    times = np.asarray(frame_time_axis_ms(records, DEFAULT_FRAME_PERIOD_MS), dtype=float) / 1000.0
    per_figure = 12
    for start in range(0, len(panels), per_figure):
        chunk = panels[start : start + per_figure]
        figure, axes = plt.subplots(
            len(chunk), 1, figsize=(14, 1.8 * len(chunk)), sharex=True, squeeze=False
        )
        for axis, panel in zip([row[0] for row in axes], chunk):
            values = series_from_records(records, panel["curves"][0])
            axis.plot(times, values, linewidth=1.0)
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
    parser.add_argument("--curves", help="只画这些列（逗号分隔），覆盖 --preset")
    parser.add_argument("--title", help="图标题；默认用输入文件名")
    parser.add_argument("--show", action="store_true", help="弹出窗口（需要显示器）")
    parser.add_argument("--dpi", type=int, default=110)
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
        plot_panels(records, [{"title": ", ".join(keys), "curves": keys}], output, title, args.show, args.dpi)
    elif args.preset == "all":
        plot_all_curves(records, output, title, args.show, args.dpi)
    else:
        plot_panels(records, PANELS, output, title, args.show, args.dpi)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
