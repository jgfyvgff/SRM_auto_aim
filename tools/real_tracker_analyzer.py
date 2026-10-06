#!/usr/bin/env python3
"""离线分析 standard_srm 产生的真机逐帧 JSONL 诊断记录。"""

import argparse
import collections
import json
import math
import statistics
from pathlib import Path


# 反馈样本到达间隔低于该值即视为"采样变密"（只作指标，单独不能判定异常）。
SHORT_BRACKET_MS = 20.0

# 串口库按标称波特率换算"字节时间"；USB CDC 不设波特率，库保持默认 9600（8N1），
# 即 1.0417ms。采样间隔若大量落在这个格点上，说明 read() 在按字节时间等待凑满读缓冲，
# 采样节律被读路径节流，而不是由下位机反馈率决定。
BYTE_TIME_MS = 1000.0 / (9600.0 / 10.0)

# 判定读路径节流还要求平均插值区间明显长于单帧量级：修复后间隔会落到
# 2 个字节时间（读满 64 字节）附近，格点占比同样很高，但采样率已经恢复正常。
THROTTLED_BRACKET_MS = 5.0


def percentile(values, ratio):
    ordered = sorted(values)
    if not ordered:
        return None
    index = min(len(ordered) - 1, max(0, math.ceil(ratio * len(ordered)) - 1))
    return ordered[index]


def finite_values(samples, key):
    values = []
    for sample in samples:
        value = sample.get(key)
        if isinstance(value, (int, float)) and math.isfinite(value):
            values.append(float(value))
    return values


def summarize(values):
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "p95": percentile(values, 0.95),
        "min": min(values),
        "max": max(values),
    }


def frame_periods(records):
    """用设备 tick 计算相邻成功帧的间隔，得到链路真实控制周期。

    相机的帧号会持续前进，但容量 1 丢旧帧队列只保留最新图像，因此
    tick 间隔反映的是主循环周期，不是相机帧率。tick_hz 缺失（回退到
    主机收帧时间）或 tick 回退的样本无法换算，直接跳过。
    """
    periods = []
    previous = None
    for record in records:
        hz = record.get("tick_hz")
        ticks = record.get("device_ticks")
        if not hz or not isinstance(ticks, (int, float)):
            previous = None
            continue
        if previous is not None:
            delta = ticks - previous[1]
            if delta > 0:
                periods.append(delta / hz * 1000.0)
        previous = (hz, ticks)
    return periods


def wire_command_rate(records):
    """用串口线程的累计量估算真正上线发送的控制指令率。

    tx_command_sent 只表示指令进入邮箱，不代表写入串口；只有
    tx_wire_target_frames/tx_wire_zero_frames 由串口线程累加，
    所以控制指令周期必须看这两个量，不能拿主循环周期代替。
    """
    points = []
    for record in records:
        hz = record.get("tick_hz")
        ticks = record.get("device_ticks")
        target = record.get("tx_wire_target_frames")
        zero = record.get("tx_wire_zero_frames")
        if not hz or not isinstance(ticks, (int, float)):
            continue
        if not isinstance(target, (int, float)) or not isinstance(zero, (int, float)):
            continue
        points.append((ticks / hz, target + zero))
    if len(points) < 2:
        return None
    span = points[-1][0] - points[0][0]
    sent = points[-1][1] - points[0][1]
    if span <= 0.0 or sent <= 0:
        return None
    return {
        "frames": int(sent),
        "span_s": span,
        "hz": sent / span,
        "period_ms": 1000.0 * span / sent,
    }


def short_interval_ratio(values, threshold_ms):
    if not values:
        return None
    return sum(value < threshold_ms for value in values) / len(values)


def byte_time_lattice_share(values, tolerance=0.15):
    """落在串口库"字节时间"整数倍附近的采样间隔占比。

    与按字节时间等待凑满读缓冲的 read() 相比，随机分布的期望占比约为
    2 * tolerance / 1.0，因此显著高于该值即说明读路径在节流采样。
    """
    if not values:
        return None
    return sum(
        abs(value / BYTE_TIME_MS - round(value / BYTE_TIME_MS)) <= tolerance
        for value in values
    ) / len(values)


def load_records(path):
    records = []
    invalid = 0
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            value = json.loads(line)
            if isinstance(value, dict):
                records.append(value)
            else:
                invalid += 1
        except json.JSONDecodeError:
            invalid += 1
    return records, invalid


def analyze_records(
    records,
    max_mapped_age_ms=200.0,
    max_mapping_delay_ms=100.0,
    max_lattice_share=0.60,
    throttle_bracket_ms=THROTTLED_BRACKET_MS,
):
    """汇总逐帧诊断记录。

    bracket_ms 是姿态插值所用两侧反馈样本的接收间隔，它由串口读路径决定，
    不必然等于下位机反馈周期：间隔偏短说明样本变密；间隔既落在串口库字节时间
    格点上、平均值又远超单帧量级时，说明 read() 在等待凑满读缓冲，采样被节流。
    """
    processed = [record for record in records if record.get("event") == "frame"]
    skipped = [record for record in records if record.get("event") == "skip"]
    reasons = collections.Counter(record.get("skip_reason", "unknown") for record in skipped)
    warnings = []

    timestamp_records = [record for record in records if record.get("device_ticks", 0) > 0]
    timestamp_sources = collections.Counter(
        record.get("timestamp_source", "unknown") for record in records
    )
    if timestamp_sources.get("host_receive", 0):
        warnings.append(
            "部分帧使用主机收帧时间，不等于相机曝光时刻；当前结果仅用于只读链路验证"
        )
    if timestamp_sources.get("estimated_device_clock", 0):
        warnings.append(
            "部分帧使用设备 tick 与主机收帧间隔估计的设备时钟，不能视为真实曝光时刻"
        )
    tick_values = [record["device_ticks"] for record in timestamp_records]
    frame_values = [record["frame_id"] for record in timestamp_records if "frame_id" in record]
    tick_regressions = sum(
        current <= previous for previous, current in zip(tick_values, tick_values[1:])
    )
    frame_regressions = sum(
        current <= previous for previous, current in zip(frame_values, frame_values[1:])
    )
    if tick_regressions:
        warnings.append(f"设备时间戳非严格递增 {tick_regressions} 次")
    if frame_regressions:
        warnings.append(f"相机帧号非严格递增 {frame_regressions} 次")

    mapped_age = finite_values(processed, "mapped_age_ms")
    mapping_delay = finite_values(processed, "mapping_delay_ms")
    periods = frame_periods(processed)
    brackets = finite_values(processed, "bracket_ms")
    short_bracket_ratio = short_interval_ratio(brackets, SHORT_BRACKET_MS)
    lattice_share = byte_time_lattice_share(brackets)
    bracket_mean_ms = statistics.fmean(brackets) if brackets else None
    wire_command = wire_command_rate(processed)
    if mapped_age and percentile(mapped_age, 0.95) > max_mapped_age_ms:
        warnings.append(
            f"mapped_age P95={percentile(mapped_age, 0.95):.1f}ms 超过 {max_mapped_age_ms:.1f}ms"
        )
    if mapping_delay and percentile(mapping_delay, 0.95) > max_mapping_delay_ms:
        warnings.append(
            f"mapping_delay P95={percentile(mapping_delay, 0.95):.1f}ms 超过 "
            f"{max_mapping_delay_ms:.1f}ms"
        )
    if not processed:
        warnings.append("没有成功处理的 frame 记录")
    if reasons:
        warnings.append("存在被跳过的帧，需结合 skip_reason 判断原因")
    if (
        lattice_share is not None
        and lattice_share > max_lattice_share
        and bracket_mean_ms is not None
        and bracket_mean_ms > throttle_bracket_ms
    ):
        warnings.append(
            f"姿态插值区间平均 {bracket_mean_ms:.1f}ms（超过 {throttle_bracket_ms:.1f}ms），"
            f"且 {lattice_share * 100:.1f}% 的到达间隔落在串口库 {BYTE_TIME_MS:.4f}ms "
            "字节时间格点上（随机分布约 30%）：read() 正在按标称波特率等待凑满读缓冲，"
            "串口采样被读路径节流，采样率与插值区间由主机读路径决定，而不是下位机反馈率"
        )

    detections = finite_values(processed, "detected")
    targets = finite_values(processed, "targets")
    state_counts = collections.Counter(record.get("tracker_state", "unknown") for record in processed)
    report = {
        "records": len(records),
        "processed_frames": len(processed),
        "skipped_frames": len(skipped),
        "skip_reasons": dict(reasons),
        "tick_hz": sorted({record.get("tick_hz") for record in timestamp_records}),
        "timestamp": {
            "sources": dict(timestamp_sources),
            "tick_regressions": tick_regressions,
            "frame_regressions": frame_regressions,
            "mapped_age_ms": summarize(mapped_age),
            "mapping_delay_ms": summarize(mapping_delay),
            "bracket_ms": summarize(brackets),
            "bracket_short_ms": SHORT_BRACKET_MS,
            "bracket_short_ratio": short_bracket_ratio,
            "bracket_byte_time_ms": BYTE_TIME_MS,
            "bracket_lattice_share": lattice_share,
            "period_ms": summarize(periods),
            "effective_hz": 1000.0 / statistics.fmean(periods) if periods else None,
        },
        "control": {
            "wire_command": wire_command,
            "mailbox_frames": sum(
                1 for record in processed if record.get("tx_command_sent") is True
            ),
        },
        "pipeline": {
            "detection_rate": sum(value > 0 for value in detections) / len(detections)
            if detections else 0.0,
            "target_rate": sum(value > 0 for value in targets) / len(targets)
            if targets else 0.0,
            "detector_ms": summarize(finite_values(processed, "detector_ms")),
            "tracker_ms": summarize(finite_values(processed, "tracker_ms")),
            "aimer_ms": summarize(finite_values(processed, "aimer_ms")),
            "feedback_wait_ms": summarize(finite_values(processed, "feedback_wait_ms")),
            "capture_to_detector_ms": summarize(
                finite_values(processed, "capture_to_detector_ms")
            ),
            "capture_to_aimer_ms": summarize(finite_values(processed, "capture_to_aimer_ms")),
            "state_counts": dict(state_counts),
        },
        "tracker": {
            key: summarize(finite_values(processed, key))
            for key in ("center_speed", "vx", "vy", "vz", "angular_velocity", "radius")
        },
        "aim": {
            key: summarize(finite_values(processed, key))
            for key in (
                "prediction_dt",
                "delay_time",
                "fly_time",
                "diagnostic_yaw_deg",
                "diagnostic_pitch_deg",
            )
        },
        "warnings": warnings,
    }
    return report


def print_report(report):
    print("========== 真机自瞄自动分析 ==========")
    print(
        f"总记录: {report['records']}，成功帧: {report['processed_frames']}，"
        f"跳过帧: {report['skipped_frames']}"
    )
    print(f"设备频率: {report['tick_hz']}")
    print(f"时间戳来源: {report['timestamp']['sources']}")
    timestamp = report["timestamp"]
    for key in ("mapped_age_ms", "mapping_delay_ms", "bracket_ms", "period_ms"):
        summary = timestamp[key]
        if summary.get("count"):
            print(
                f"  {key}: mean={summary['mean']:.2f}ms "
                f"p95={summary['p95']:.2f}ms"
            )
    if timestamp.get("effective_hz"):
        print(f"  视觉链路帧率: {timestamp['effective_hz']:.1f}Hz （由相机 tick 间隔换算，不是控制指令周期）")
    if timestamp.get("bracket_short_ratio") is not None:
        print(
            f"  反馈到达间隔 <{timestamp['bracket_short_ms']:.0f}ms 占比: "
            f"{timestamp['bracket_short_ratio'] * 100:.1f}%"
        )
    if timestamp.get("bracket_lattice_share") is not None:
        print(
            f"  反馈到达间隔格点占比: {timestamp['bracket_lattice_share'] * 100:.1f}% "
            f"（串口库字节时间 {timestamp['bracket_byte_time_ms']:.4f}ms，随机分布约 30%）"
        )
    control = report.get("control", {})
    wire = control.get("wire_command")
    if wire:
        print(
            f"  控制指令上线率: {wire['hz']:.1f}Hz "
            f"（{wire['period_ms']:.2f}ms/次，{wire['frames']} 次 / {wire['span_s']:.1f}s，"
            "来自串口线程 tx_wire_* 累计量）"
        )
    pipeline = report["pipeline"]
    print(
        f"检测率={pipeline['detection_rate']:.3f} 目标率={pipeline['target_rate']:.3f} "
        f"Tracker状态={pipeline['state_counts']}"
    )
    print("跳过原因:", report["skip_reasons"] or "无")
    for key in (
        "detector_ms",
        "tracker_ms",
        "aimer_ms",
        "feedback_wait_ms",
        "capture_to_detector_ms",
        "capture_to_aimer_ms",
    ):
        summary = pipeline.get(key)
        if summary and summary.get("count"):
            print(
                f"  {key}: mean={summary['mean']:.2f}ms "
                f"p95={summary['p95']:.2f}ms max={summary['max']:.2f}ms"
            )
    print("[诊断]")
    if report["warnings"]:
        for warning in report["warnings"]:
            print("  [WARN]", warning)
    else:
        print("  [OK] 当前统计项未超过阈值")


def main():
    parser = argparse.ArgumentParser(description="分析 standard_srm 真机 JSONL 诊断记录")
    parser.add_argument("--input", required=True, help="standard_srm 的 JSONL 文件")
    parser.add_argument("--output", help="可选 JSON 报告路径")
    parser.add_argument("--max-mapped-age-ms", type=float, default=200.0)
    parser.add_argument("--max-mapping-delay-ms", type=float, default=100.0)
    parser.add_argument(
        "--max-lattice-share",
        type=float,
        default=0.60,
        help=f"反馈到达间隔落在 {BYTE_TIME_MS:.4f}ms 字节时间格点上的允许占比",
    )
    parser.add_argument(
        "--throttle-bracket-ms",
        type=float,
        default=THROTTLED_BRACKET_MS,
        help="判定串口读路径节流的平均插值区间门限",
    )
    args = parser.parse_args()
    records, invalid = load_records(args.input)
    report = analyze_records(
        records,
        args.max_mapped_age_ms,
        args.max_mapping_delay_ms,
        args.max_lattice_share,
        args.throttle_bracket_ms,
    )
    report["invalid_lines"] = invalid
    print_report(report)
    if args.output:
        Path(args.output).write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"报告已保存到: {args.output}")
    return 0 if records else 2


if __name__ == "__main__":
    raise SystemExit(main())
