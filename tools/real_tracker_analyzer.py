#!/usr/bin/env python3
"""离线分析 standard_srm 产生的真机逐帧 JSONL 诊断记录。"""

import argparse
import collections
import json
import math
import statistics
from pathlib import Path


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


def analyze_records(records, max_mapped_age_ms=200.0, max_mapping_delay_ms=100.0):
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
            "bracket_ms": summarize(finite_values(processed, "bracket_ms")),
            "period_ms": summarize(periods),
            "effective_hz": 1000.0 / statistics.fmean(periods) if periods else None,
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
        print(f"  链路实际帧率: {timestamp['effective_hz']:.1f}Hz （由相机 tick 间隔换算）")
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
    args = parser.parse_args()
    records, invalid = load_records(args.input)
    report = analyze_records(records, args.max_mapped_age_ms, args.max_mapping_delay_ms)
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
