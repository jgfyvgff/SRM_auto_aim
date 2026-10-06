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

# 只有 tracking 状态会发送跟踪指令；其余状态 standard_srm 发送保持角
# （即 last_valid_aim_command）。因此这些帧的占比等于"云台冻结"占比。
FOLLOWING_STATES = ("tracking",)

# 缺设备 tick 时把盲区帧数换算成时长的兜底控制周期（≈71Hz）。
DEFAULT_FRAME_PERIOD_MS = 14.0

# 保持帧占比超过该值即认为控制链路大部分时间没有在跟随目标。
FREEZE_RATIO_WARNING = 0.20

# 盲区片段成因占比判据：无检测占多数说明瓶颈在检测器，候选被拒占多数说明在门限。
NO_DETECTION_RUN_SHARE = 0.60
CANDIDATE_REJECT_RUN_SHARE = 0.30

# 同一 Tracker 世代内 radius 波动超过该值会明显平移瞄点（瞄点 = 中心 - r·方向）。
RADIUS_DRIFT_WARN_M = 0.05

# 保持→跟随切换那一帧的 yaw 跳变超过该值即存在可见追赶。
RESUME_STEP_WARN_DEG = 1.0

# 诊断字段分组：日志缺少整组字段时，对应统计必然为空。必须显式报告字段覆盖，
# 否则"字段缺失"会被误读成"没有这个问题"（旧日志尤其容易踩）。
FIELD_GROUPS = (
    ("tracker_state", "盲区与保持帧"),
    ("tx_command_mode", "指令模式"),
    ("tx_command_yaw_deg", "指令步进与恢复跳变"),
    ("association_candidate_count", "关联拒绝归因"),
    ("tracker_generation", "世代内波动"),
    ("radius", "半径漂移"),
)

# 缺这几组字段时主结论无法给出，必须告警；其余组只在覆盖行里按需展示。
CRITICAL_FIELD_GROUPS = ("盲区与保持帧", "指令模式", "关联拒绝归因")

# 长度不超过该值的盲区视为"单次漏检造成的短暂冻结"，与"目标真的不在视野"分开。
SHORT_BLIND_FRAMES = 2

# 打印最长的几段盲区及其内部成因构成，用于判断长段是"整段无检测（目标真的不在
# 视野）"还是"混合段（检测时有时无，被段级成因一言以蔽之）"。
LONGEST_RUN_REPORT = 3


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


def field_coverage(records):
    """统计各组诊断字段在成功帧里的出现次数。

    旧日志（早于字段落地）缺少整组字段时，对应统计会全部为空。若不显式报告
    覆盖，空段落会被误读成"该问题不存在"，因此字段覆盖必须和结论一起给出。
    """
    return {
        label: sum(1 for record in records if key in record)
        for key, label in FIELD_GROUPS
    }


def frame_time_axis_ms(records, fallback_ms):
    """为每帧给出单调时间轴（ms），缺设备 tick 时用兜底周期累加。

    盲区时长必须按时间而不是帧数衡量：控制周期随读路径修复变化，帧数相同
    不代表云台冻结时长相同。
    """
    axis = []
    current = None
    for record in records:
        hz = record.get("tick_hz")
        ticks = record.get("device_ticks")
        if isinstance(hz, (int, float)) and hz and isinstance(ticks, (int, float)) and ticks > 0:
            value = ticks / hz * 1000.0
            current = value if current is None or value >= current else current + fallback_ms
        elif current is None:
            current = 0.0
        else:
            current = current + fallback_ms
        axis.append(current)
    return axis


def blind_run_cause(run):
    """判定一段盲区（连续非跟踪帧）的主导成因。"""
    if any((frame.get("association_candidate_count") or 0) > 0 for frame in run) and not any(
        (frame.get("association_accepted_count") or 0) > 0 for frame in run
    ):
        return "candidate_rejected"
    if any(
        (frame.get("detected") or 0) > 0
        or (frame.get("association_matching_detection_count") or 0) > 0
        for frame in run
    ):
        return "detection_not_associated"
    return "no_detection"


def continuity_report(records, fallback_period_ms):
    """统计非跟踪帧占比、盲区段长度与成因。

    Tracker 在 temp_lost 仍返回外推 Target，但控制门限要求 state=="tracking"，
    所以这些帧云台只发保持角。只看总占比无法区分"检测器没输出"和"有输出但被
    关联/EKF 拒绝"，因此逐段归类成 no_detection / detection_not_associated /
    candidate_rejected 三种，分别对应检测器、关联前置条件和关联门限三条修复路径。
    """
    tracked = [record for record in records if "tracker_state" in record]
    if not tracked:
        return {}
    axis = frame_time_axis_ms(tracked, fallback_period_ms)
    runs = []
    current = []
    for index, record in enumerate(tracked):
        if record.get("tracker_state") in FOLLOWING_STATES:
            if current:
                runs.append(current)
                current = []
            continue
        current.append(index)
    if current:
        runs.append(current)

    causes = collections.Counter()
    frame_causes = collections.Counter()
    cause_longest_frames = collections.Counter()
    cause_longest_ms = {}
    longest_frames = 0
    longest_ms = 0.0
    runs_detail = []
    for run in runs:
        frames = [tracked[index] for index in run]
        cause = blind_run_cause(frames)
        duration_ms = axis[run[-1]] - axis[run[0]] + fallback_period_ms
        causes[cause] += 1
        # 段级成因只记录"这段里出现过什么证据"，会把整段的帧数记到那一个成因上。
        # 按帧加权必须逐帧用自身证据归类：一段 611 帧里只有 1 帧有检测时，段级
        # 成因是 detection_not_associated，但那 610 帧的真因是 no_detection。
        # 两种口径在真机日志上分别给出 2731/1944/1703（段级）与 1636/3912/824
        # （帧级），差值全部来自这种混合段，判读必须以帧级为准。
        composition = collections.Counter()
        for frame in frames:
            frame_cause = blind_run_cause([frame])
            frame_causes[frame_cause] += 1
            composition[frame_cause] += 1
        runs_detail.append(
            {
                "start_index": run[0],
                "end_index": run[-1],
                "frames": len(run),
                "ms": duration_ms,
                "cause": cause,
                "frame_causes": dict(composition),
            }
        )
        if len(run) > cause_longest_frames[cause]:
            cause_longest_frames[cause] = len(run)
            cause_longest_ms[cause] = duration_ms
        longest_frames = max(longest_frames, len(run))
        longest_ms = max(longest_ms, duration_ms)

    # 按段统计会被大量 1~2 帧的短盲区稀释：1853 段里可能只有几段是"目标真的不在
    # 视野"，其余全是单帧漏检造成的短暂冻结。必须同时给出按帧加权的成因占比与
    # 短盲区占比，否则无法判断该修检测器、门限，还是根本不用修。
    short_runs = [run for run in runs if len(run) <= SHORT_BLIND_FRAMES]
    blind = [index for run in runs for index in run]
    return {
        "following_states": list(FOLLOWING_STATES),
        "blind_frames": len(blind),
        "blind_ratio": len(blind) / len(tracked),
        "blind_runs": len(runs),
        "longest_blind_frames": longest_frames,
        "longest_blind_ms": longest_ms,
        "blind_frames_with_detection": sum(
            1 for index in blind if (tracked[index].get("detected") or 0) > 0
        ),
        "blind_frames_with_candidate": sum(
            1 for index in blind if (tracked[index].get("association_candidate_count") or 0) > 0
        ),
        "blind_frames_accepted": sum(
            1 for index in blind if (tracked[index].get("association_accepted_count") or 0) > 0
        ),
        "run_causes": dict(causes),
        "frame_causes": dict(frame_causes),
        "cause_longest_frames": dict(cause_longest_frames),
        "cause_longest_ms": cause_longest_ms,
        "longest_runs": sorted(
            runs_detail, key=lambda item: (item["frames"], item["ms"]), reverse=True
        )[:LONGEST_RUN_REPORT],
        "short_blind_runs": len(short_runs),
        "short_blind_frames": sum(len(run) for run in short_runs),
        "short_blind_ratio": (
            sum(len(run) for run in short_runs) / len(blind) if blind else 0.0
        ),
    }


def tri_state(value):
    return None if value is None else bool(value)


def association_report(records):
    """统计关联候选的接受/拒绝与失败门限。

    候选通过全部门限但 accepted=0，说明 Target::update 的 EKF 后验检查否决了
    更新（tracker.cpp 中 gate_passed 与后验共用距离门限）。这与"被关联门限拒绝"
    是两条不同修复路径，必须分开计数。
    """
    candidate_frames = [
        record for record in records if (record.get("association_candidate_count") or 0) > 0
    ]
    accepted = [
        record for record in candidate_frames if (record.get("association_accepted_count") or 0) > 0
    ]
    rejected = [
        record for record in candidate_frames if not (record.get("association_accepted_count") or 0)
    ]
    failing = collections.Counter()
    gate_passed_but_rejected = 0
    for record in rejected:
        if tri_state(record.get("association_primary_gate_passed")):
            gate_passed_but_rejected += 1
            continue
        for name, key in (
            ("position", "association_primary_position_gate_passed"),
            ("distance", "association_primary_distance_gate_passed"),
            ("mahalanobis", "association_primary_mahalanobis_gate_passed"),
            ("score", "association_primary_score_gate_passed"),
            ("angle", "association_primary_angle_gate_passed"),
        ):
            if tri_state(record.get(key)) is False:
                failing[name] += 1
    return {
        "candidate_frames": len(candidate_frames),
        "accepted_frames": len(accepted),
        "rejected_frames": len(rejected),
        "gate_passed_but_rejected_frames": gate_passed_but_rejected,
        "failing_gates": dict(failing),
        "rejected_position_error": summarize(
            finite_values(rejected, "association_primary_position_error")
        ),
        "rejected_distance_error": summarize(
            finite_values(rejected, "association_primary_distance_error")
        ),
        "rejected_mahalanobis_distance": summarize(
            finite_values(rejected, "association_primary_mahalanobis_distance")
        ),
        "accepted_position_error": summarize(
            finite_values(accepted, "association_primary_position_error")
        ),
        "accepted_mahalanobis_distance": summarize(
            finite_values(accepted, "association_primary_mahalanobis_distance")
        ),
    }


def change_count(records, key):
    """状态量逐帧变化次数；用于识别 Target 重建和装甲板 ID 跳变。"""
    values = [record.get(key) for record in records if isinstance(record.get(key), (int, float))]
    return sum(current != previous for previous, current in zip(values, values[1:]))


def within_group_stdev(records, key, group_key="tracker_generation", min_frames=10):
    """同一 Tracker 世代内的状态波动。

    直接对全程取标准差会把世代重建时的跳变算进"抖动"；只有同一世代内持续
    存在的波动才是真正的指令抖动来源。
    """
    groups = collections.defaultdict(list)
    for record in records:
        value = record.get(key)
        if isinstance(value, (int, float)) and math.isfinite(value):
            groups[record.get(group_key)].append(float(value))
    deviations = [
        statistics.pstdev(values) for values in groups.values() if len(values) >= min_frames
    ]
    if not deviations:
        return None
    return statistics.fmean(deviations)


def command_jitter_report(records):
    """控制指令模式、逐帧步进与目标重建次数。

    hold_* 表示保持角（未跟随）；resume_step_deg 专门统计"保持→跟随"切换那一帧
    的角度跳变，它直接对应"盲区结束后云台突然追赶"。
    """
    modes = collections.Counter(
        record["tx_command_mode"] for record in records if "tx_command_mode" in record
    )
    steps = []
    resume_steps = []
    previous = None
    for record in records:
        yaw = record.get("tx_command_yaw_deg")
        mode = record.get("tx_command_mode")
        if not isinstance(yaw, (int, float)) or not math.isfinite(yaw):
            previous = None
            continue
        if previous is not None:
            delta = abs(((float(yaw) - previous[0] + 180.0) % 360.0) - 180.0)
            steps.append(delta)
            if str(previous[1]).startswith("hold") and not str(mode).startswith("hold"):
                resume_steps.append(delta)
        previous = (float(yaw), mode)
    hold_frames = sum(count for mode, count in modes.items() if str(mode).startswith("hold"))
    return {
        "tx_command_mode_counts": dict(modes),
        "hold_frames": hold_frames,
        "hold_ratio": hold_frames / len(records) if records else 0.0,
        "command_yaw_step_deg": summarize(steps),
        "resume_step_deg": summarize(resume_steps),
        "generation_changes": change_count(records, "tracker_generation"),
        "armor_id_switches": change_count(records, "current_armor_id"),
        "radius_stdev_within_generation": within_group_stdev(records, "radius"),
        "center_yaw_stdev_within_generation": within_group_stdev(records, "center_yaw"),
    }


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

    control_period_ms = statistics.fmean(periods) if periods else DEFAULT_FRAME_PERIOD_MS
    continuity = continuity_report(processed, control_period_ms)
    association = association_report(processed)
    jitter = command_jitter_report(processed)
    coverage = field_coverage(processed)
    missing_fields = [label for label, count in coverage.items() if not count] if processed else []
    missing_critical = [label for label in missing_fields if label in CRITICAL_FIELD_GROUPS]
    if missing_critical:
        warnings.append(
            "日志缺少 " + "、".join(missing_critical) + " 字段：对应统计为空，"
            "不代表没有该问题；需要用带这些字段的版本（standard_srm 的 "
            "tx_command_mode / association_*）重新采集"
        )
    if continuity.get("blind_ratio", 0.0) > FREEZE_RATIO_WARNING:
        warnings.append(
            f"云台保持帧占比 {continuity['blind_ratio'] * 100:.1f}% "
            f"（超过 {FREEZE_RATIO_WARNING * 100:.0f}%），共 {continuity['blind_runs']} 段盲区，"
            f"最长 {continuity['longest_blind_frames']} 帧 / {continuity['longest_blind_ms']:.1f}ms："
            "非 tracking 帧只发送保持角，这段时间云台没有跟随目标"
        )
    frame_causes = continuity.get("frame_causes", {})
    total_blind = sum(frame_causes.values())
    if total_blind:
        no_detection_share = frame_causes.get("no_detection", 0) / total_blind
        rejected_share = frame_causes.get("candidate_rejected", 0) / total_blind
        if no_detection_share > NO_DETECTION_RUN_SHARE:
            warnings.append(
                f"{no_detection_share * 100:.0f}% 的冻结时长来自没有任何敌方装甲板检测"
                "（detected==0）的帧：瓶颈在检测器（min_confidence、曝光、焦距），"
                "而不是 Tracker 状态机或关联参数"
            )
        if rejected_share > CANDIDATE_REJECT_RUN_SHARE:
            warnings.append(
                f"{rejected_share * 100:.0f}% 的冻结时长来自有检测但候选被全部拒绝的帧："
                "瓶颈在关联门限或 EKF 后验，需要确认门限是否随距离与协方差自适应"
            )
        detection_share = frame_causes.get("detection_not_associated", 0) / total_blind
        if detection_share > NO_DETECTION_RUN_SHARE:
            warnings.append(
                f"{detection_share * 100:.0f}% 的冻结时长来自有检测但没有形成关联候选的帧："
                "候选列表在匹配/颜色/PnP 阶段就被清空，先查 matching_detection_count 与"
                "优化后的装甲板是否被判为无效，而不是看关联门限"
            )
        no_detection_runs = continuity.get("run_causes", {}).get("no_detection", 0)
        total_runs = sum(continuity.get("run_causes", {}).values())
        if total_runs and no_detection_runs / total_runs > NO_DETECTION_RUN_SHARE:
            warnings.append(
                f"{no_detection_runs / total_runs * 100:.0f}% 的盲区段（{total_runs} 段）"
                "完全没有检测：这些段里云台不动是目标真的不在视野，不是状态机缺陷"
            )
        short_ratio = continuity.get("short_blind_ratio", 0.0)
        if continuity.get("short_blind_runs") and short_ratio < FREEZE_RATIO_WARNING:
            warnings.append(
                f"{continuity['short_blind_runs']} 段短盲区（≤{SHORT_BLIND_FRAMES} 帧）"
                f"只占冻结帧的 {short_ratio * 100:.1f}%：主要冻结时长来自长盲区，"
                "优先按上面的成因修根因，而不是只做短时外推"
            )
    if association.get("gate_passed_but_rejected_frames", 0):
        warnings.append(
            f"{association['gate_passed_but_rejected_frames']} 帧的候选通过了关联门限，"
            "却被 Target::update 的 EKF 后验检查否决"
        )
    radius_stdev = jitter.get("radius_stdev_within_generation")
    if radius_stdev is not None and radius_stdev > RADIUS_DRIFT_WARN_M:
        warnings.append(
            f"同一 Tracker 世代内 radius 波动 {radius_stdev:.3f}m"
            f"（超过 {RADIUS_DRIFT_WARN_M:.2f}m）：瞄点按 中心-r·方向 计算，"
            "半径漂移会直接平移指令角，是抖动的独立来源"
        )
    resume_max = jitter.get("resume_step_deg", {}).get("max")
    if resume_max is not None and resume_max > RESUME_STEP_WARN_DEG:
        warnings.append(
            f"保持→跟随切换的最大 yaw 跳变 {resume_max:.2f}°"
            f"（超过 {RESUME_STEP_WARN_DEG:.1f}°）：对应盲区结束后云台突然追赶"
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
        "continuity": continuity,
        "association": association,
        "jitter": jitter,
        "coverage": coverage,
        "missing_fields": missing_fields,
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
    coverage = report.get("coverage", {})
    if coverage:
        print(
            "  诊断字段覆盖: "
            + " ".join(f"{label}={count}" for label, count in coverage.items())
        )
    missing_fields = report.get("missing_fields", [])
    if missing_fields:
        print("  缺失字段: " + "、".join(missing_fields) + "（对应统计为空，不代表没有问题）")
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
    continuity = report.get("continuity", {})
    if continuity.get("blind_frames"):
        print(
            f"  云台保持帧: {continuity['blind_frames']} 帧 "
            f"({continuity['blind_ratio'] * 100:.1f}%)，盲区 {continuity['blind_runs']} 段，"
            f"最长 {continuity['longest_blind_frames']} 帧 / {continuity['longest_blind_ms']:.1f}ms"
        )
        causes = " ".join(
            f"{name}={count}" for name, count in continuity["run_causes"].items()
        )
        print(
            f"  盲区成因(按段): {causes or '无'}"
            f"（其中有检测的盲区帧 {continuity['blind_frames_with_detection']}，"
            f"有候选 {continuity['blind_frames_with_candidate']}，"
            f"有确认观测 {continuity['blind_frames_accepted']}）"
        )
        frame_causes = " ".join(
            f"{name}={count}" for name, count in continuity.get("frame_causes", {}).items()
        )
        print(
            f"  盲区成因(按帧): {frame_causes or '无'}；"
            f"短盲区(≤{SHORT_BLIND_FRAMES}帧) {continuity.get('short_blind_runs', 0)} 段 "
            f"只占冻结帧 {continuity.get('short_blind_ratio', 0.0) * 100:.1f}%"
        )
        longest = " ".join(
            f"{name}={frames}帧/{continuity.get('cause_longest_ms', {}).get(name, 0.0):.0f}ms"
            for name, frames in continuity.get("cause_longest_frames", {}).items()
        )
        print(f"  各成因最长段: {longest or '无'}")
        for rank, run in enumerate(continuity.get("longest_runs", []), start=1):
            detail = " ".join(
                f"{name}={count}" for name, count in run.get("frame_causes", {}).items()
            )
            print(
                f"  最长盲区段 #{rank}: {run['frames']} 帧 / {run['ms']:.0f}ms"
                f"（帧 #{run['start_index']}~{run['end_index']}），"
                f"段级成因 {run['cause']}，段内帧成因 {detail or '无'}"
            )
    association = report.get("association", {})
    if association.get("candidate_frames"):
        print(
            f"  关联候选帧 {association['candidate_frames']}：接受 {association['accepted_frames']}，"
            f"拒绝 {association['rejected_frames']}，"
            f"门限通过但后验拒绝 {association['gate_passed_but_rejected_frames']}"
        )
        if association.get("failing_gates"):
            print(
                "  被拒门限: "
                + " ".join(f"{name}={count}" for name, count in association["failing_gates"].items())
            )
        for key in (
            "rejected_position_error",
            "rejected_distance_error",
            "rejected_mahalanobis_distance",
            "accepted_position_error",
        ):
            summary = association.get(key)
            if summary and summary.get("count"):
                print(
                    f"    {key}: mean={summary['mean']:.3f} "
                    f"p95={summary['p95']:.3f} max={summary['max']:.3f}"
                )
    jitter = report.get("jitter", {})
    if jitter.get("tx_command_mode_counts"):
        print(f"  指令模式: {jitter['tx_command_mode_counts']}")
    for key in ("command_yaw_step_deg", "resume_step_deg"):
        summary = jitter.get(key)
        if summary and summary.get("count"):
            print(
                f"  {key}: mean={summary['mean']:.3f}° "
                f"p95={summary['p95']:.3f}° max={summary['max']:.3f}° "
                f"({summary['count']} 次)"
            )
    if jitter.get("generation_changes") or jitter.get("armor_id_switches"):
        print(
            f"  Tracker世代切换 {jitter.get('generation_changes', 0)} 次，"
            f"装甲板ID切换 {jitter.get('armor_id_switches', 0)} 次"
        )
    for key in ("radius_stdev_within_generation", "center_yaw_stdev_within_generation"):
        value = jitter.get(key)
        if value is not None:
            print(f"  世代内波动 {key}: {value:.4f}")
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
