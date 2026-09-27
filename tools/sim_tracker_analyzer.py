#!/usr/bin/env python3
"""订阅仿真自瞄调试 Topic，并自动分析 Tracker 的静止与小陀螺表现。"""

import argparse
import bisect
import json
import math
import statistics
import time
from pathlib import Path


ANALYZED_FIELDS = (
    "prediction_dt",
    "target_distance",
    "armor_pixel_long_side",
    "armor_pixel_short_side",
    "command_yaw",
    "command_pitch",
    "command_yaw_deg",
    "command_pitch_deg",
    "delay_time",
    "base_prediction_dt",
    "fly_time",
    "capture_to_detector_ms",
    "detector_ms",
    "tracker_ms",
    "aimer_ms",
    "capture_to_aimer_ms",
    "aim_current_x",
    "aim_current_y",
    "aim_current_z",
    "aim_current_yaw",
    "future_x",
    "future_y",
    "future_z",
    "future_yaw",
    "center_x",
    "center_y",
    "center_z",
    "center_speed",
    "vx",
    "vy",
    "vz",
    "angular_velocity",
    "observed_yaw_rate",
    "radius",
    "radius_delta",
    "alternate_radius",
    "height_delta",
    "current_ekf_error",
    "post_update_position_error",
    "post_update_bearing_error",
    "post_update_distance_error",
    "post_update_orientation_error",
    "pnp_error",
    "nis",
    "nis_failure_rate",
    "association_primary_score",
    "association_primary_orientation_error",
    "association_primary_bearing_error",
    "association_primary_raw_yaw",
    "association_primary_optimized_yaw",
    "association_primary_yaw_correction",
    "association_primary_yaw_correction_abs",
    "association_primary_image_x",
    "association_primary_gate_passed",
    "association_primary_position_error",
    "association_primary_distance_error",
    "association_primary_mahalanobis_distance",
    "association_primary_position_angle_error",
    "association_primary_distance_angle_error",
    "association_primary_position_angle_error",
    "association_primary_distance_angle_error",
    "association_primary_observed_x",
    "association_primary_observed_y",
    "association_primary_observed_z",
    "association_primary_predicted_x",
    "association_primary_predicted_y",
    "association_primary_predicted_z",
    "association_primary_observed_distance",
    "association_primary_predicted_distance",
    "association_secondary_score",
    "association_secondary_orientation_error",
    "association_secondary_bearing_error",
    "association_secondary_raw_yaw",
    "association_secondary_optimized_yaw",
    "association_secondary_yaw_correction",
    "association_secondary_yaw_correction_abs",
    "association_secondary_image_x",
    "association_secondary_gate_passed",
    "association_secondary_position_error",
    "association_secondary_distance_error",
    "association_secondary_mahalanobis_distance",
    "association_secondary_position_angle_error",
    "association_secondary_distance_angle_error",
    "association_secondary_position_angle_error",
    "association_secondary_distance_angle_error",
    "association_secondary_observed_x",
    "association_secondary_observed_y",
    "association_secondary_observed_z",
    "association_secondary_predicted_x",
    "association_secondary_predicted_y",
    "association_secondary_predicted_z",
    "association_secondary_observed_distance",
    "association_secondary_predicted_distance",
)

OUTLIER_CONTEXT_FIELDS = (
    "timestamp",
    "tracker_generation",
    "current_armor_id",
    "aim_armor_id",
    "center_x",
    "center_y",
    "center_z",
    "angular_velocity",
    "current_ekf_error",
    "post_update_position_error",
    "post_update_bearing_error",
    "post_update_distance_error",
    "post_update_orientation_error",
    "pnp_error",
    "association_candidate_count",
    "association_accepted_count",
    "association_primary_id",
    "association_primary_accepted",
    "association_primary_gate_passed",
    "association_primary_position_error",
    "association_primary_distance_error",
    "association_primary_mahalanobis_distance",
    "association_primary_observed_x",
    "association_primary_observed_y",
    "association_primary_observed_z",
    "association_primary_predicted_x",
    "association_primary_predicted_y",
    "association_primary_predicted_z",
    "association_primary_observed_distance",
    "association_primary_predicted_distance",
    "association_primary_score",
    "association_primary_orientation_error",
    "association_primary_bearing_error",
    "association_primary_raw_yaw",
    "association_primary_optimized_yaw",
    "association_primary_yaw_correction",
    "association_secondary_id",
    "association_secondary_accepted",
    "association_secondary_gate_passed",
    "association_secondary_position_error",
    "association_secondary_distance_error",
    "association_secondary_mahalanobis_distance",
    "association_secondary_observed_x",
    "association_secondary_observed_y",
    "association_secondary_observed_z",
    "association_secondary_predicted_x",
    "association_secondary_predicted_y",
    "association_secondary_predicted_z",
    "association_secondary_observed_distance",
    "association_secondary_predicted_distance",
    "association_secondary_score",
    "association_secondary_orientation_error",
    "association_secondary_bearing_error",
    "association_secondary_raw_yaw",
    "association_secondary_optimized_yaw",
    "association_secondary_yaw_correction",
)

DEFAULT_THRESHOLDS = {
    "static_angular": 0.3,
    "spin_angular": 1.0,
    "center_span": 0.10,
    "center_speed": 0.20,
    "radius_span": 0.05,
    "id_jump": 0.05,
    "ekf_reprojection_armor_ratio": 1.0,
    "prediction_match_tolerance": 0.03,
    "range_bin_size": 1.0,
    "minimum_samples": 20,
}

RANGE_STATS_FIELDS = (
    "pnp_error",
    "current_ekf_error",
    "post_update_position_error",
    "post_update_bearing_error",
    "post_update_distance_error",
    "post_update_orientation_error",
    "association_primary_position_error",
    "association_primary_distance_error",
    "association_primary_mahalanobis_distance",
    "center_speed",
    "armor_pixel_long_side",
    "armor_pixel_short_side",
)

INVALID_NEGATIVE_FIELDS = frozenset(
    (
        "prediction_dt",
        "target_distance",
        "armor_pixel_long_side",
        "armor_pixel_short_side",
        "base_prediction_dt",
        "fly_time",
        "current_ekf_error",
        "post_update_position_error",
        "post_update_bearing_error",
        "post_update_distance_error",
        "post_update_orientation_error",
        "pnp_error",
    )
)


def _finite_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _percentile(values, ratio):
    if not values:
        return None
    ordered = sorted(values)
    position = ratio * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _summarize(values):
    if not values:
        return None
    minimum = min(values)
    maximum = max(values)
    p05 = _percentile(values, 0.05)
    p95 = _percentile(values, 0.95)
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "stddev": statistics.pstdev(values),
        "minimum": minimum,
        "maximum": maximum,
        "span": maximum - minimum,
        "p05": p05,
        "p95": p95,
        "robust_span": p95 - p05,
    }


def _analyze_range_buckets(samples, bin_size):
    """按目标距离分桶，避免近距离和远距离误差混在同一统计量中。"""
    buckets = {}
    for sample in samples:
        distance = sample.get("target_distance")
        if distance is None or distance < 0.0:
            continue
        bucket_index = math.floor(distance / bin_size)
        buckets.setdefault(bucket_index, []).append(sample)

    result = []
    for bucket_index in sorted(buckets):
        bucket_samples = buckets[bucket_index]
        stats = {}
        for field in RANGE_STATS_FIELDS:
            summary = _summarize(
                [
                    sample[field]
                    for sample in bucket_samples
                    if field in sample and sample[field] >= 0.0
                ]
            )
            if summary is not None:
                stats[field] = summary

        generations = {
            sample["tracker_generation"]
            for sample in bucket_samples
            if "tracker_generation" in sample
        }
        id_switch_count = sum(
            previous.get("current_armor_id") != current.get("current_armor_id")
            for previous, current in zip(bucket_samples, bucket_samples[1:])
            if "current_armor_id" in previous and "current_armor_id" in current
        )
        result.append(
            {
                "minimum_distance": bucket_index * bin_size,
                "maximum_distance": (bucket_index + 1) * bin_size,
                "sample_count": len(bucket_samples),
                "tracker_generation_count": len(generations),
                "id_switch_count": id_switch_count,
                "stats": stats,
            }
        )
    return result


def _outlier_snapshot(sample):
    """仅保留定位异常所需字段，避免报告复制全部高频采样数据。"""
    return {
        field: sample[field]
        for field in OUTLIER_CONTEXT_FIELDS
        if field in sample
    }


def _extract_outlier_events(samples, limit=10):
    center_jump_events = []
    for previous, current in zip(samples, samples[1:]):
        required = ("center_x", "center_y")
        if not all(field in previous and field in current for field in required):
            continue
        event = _outlier_snapshot(current)
        event["previous_timestamp"] = previous["timestamp"]
        event["center_step"] = math.hypot(
            current["center_x"] - previous["center_x"],
            current["center_y"] - previous["center_y"],
        )
        event["generation_changed"] = (
            "tracker_generation" in previous
            and "tracker_generation" in current
            and previous["tracker_generation"] != current["tracker_generation"]
        )
        center_jump_events.append(event)

    ekf_error_events = [
        _outlier_snapshot(sample)
        for sample in samples
        if "current_ekf_error" in sample
    ]
    center_jump_events.sort(key=lambda event: event["center_step"], reverse=True)
    ekf_error_events.sort(
        key=lambda event: event.get("current_ekf_error", -math.inf),
        reverse=True,
    )
    return {
        "center_jump": center_jump_events[:limit],
        "ekf_error": ekf_error_events[:limit],
    }


def normalize_sample(payload, timestamp):
    """保留分析需要的有限数值，防止异常 JSON 污染整段统计。"""
    sample = {"timestamp": float(timestamp)}
    for field in ANALYZED_FIELDS:
        value = payload.get(field)
        if _finite_number(value):
            sample[field] = float(value)

    for prefix in ("association_primary", "association_secondary"):
        correction = sample.get(f"{prefix}_yaw_correction")
        if correction is not None:
            sample[f"{prefix}_yaw_correction_abs"] = abs(correction)

    for field in (
        "current_armor_id",
        "aim_armor_id",
        "tracker_generation",
        "association_candidate_count",
        "association_accepted_count",
        "association_primary_id",
        "association_primary_accepted",
        "association_primary_gate_passed",
        "association_primary_angle_gate_passed",
        "association_primary_score_gate_passed",
        "association_primary_position_gate_passed",
        "association_primary_distance_gate_passed",
        "association_primary_mahalanobis_gate_passed",
        "association_secondary_id",
        "association_secondary_accepted",
        "association_secondary_gate_passed",
        "association_secondary_angle_gate_passed",
        "association_secondary_score_gate_passed",
        "association_secondary_position_gate_passed",
        "association_secondary_distance_gate_passed",
        "association_secondary_mahalanobis_gate_passed",
        "high_speed_mode",
        "command_control",
        "command_shoot",
    ):
        value = payload.get(field)
        if _finite_number(value):
            sample[field] = int(value)
    return sample


def _accepted_armor_observation(sample, armor_id):
    """返回指定模型 ID 的已接收观测；同一帧可能位于主候选或次候选。"""
    for prefix in ("association_primary", "association_secondary"):
        if sample.get(f"{prefix}_accepted") != 1:
            continue
        if sample.get(f"{prefix}_id") != armor_id:
            continue
        coordinate_fields = tuple(f"{prefix}_observed_{axis}" for axis in "xyz")
        if not all(field in sample for field in coordinate_fields):
            continue
        return {
            "xyz": tuple(sample[field] for field in coordinate_fields),
            "yaw": sample.get(f"{prefix}_optimized_yaw"),
        }
    return None


def _wrapped_angle_error(lhs, rhs):
    """计算两个弧度角之间的最小绝对差，结果范围为 [0, pi]。"""
    return abs((lhs - rhs + math.pi) % (2.0 * math.pi) - math.pi)


def _signed_angle_error(lhs, rhs):
    """返回带方向的最小角度差 lhs-rhs，范围为 [-pi, pi]。"""
    return (lhs - rhs + math.pi) % (2.0 * math.pi) - math.pi


def _add_observed_yaw_rate(samples):
    """从连续主关联观测估计 yaw 速率，仅用于区分观测和 EKF 的问题。"""
    augmented_samples = [dict(sample) for sample in samples]
    previous = None
    for sample in augmented_samples:
        sample.pop("observed_yaw_rate", None)
        generation = sample.get("tracker_generation")
        model_id = sample.get("association_primary_id")
        timestamp = sample.get("timestamp")
        observed_yaw = sample.get("association_primary_optimized_yaw")
        current = (generation, model_id, timestamp, observed_yaw)
        if (
            previous is not None
            and generation is not None
            and model_id is not None
            and timestamp is not None
            and observed_yaw is not None
            and previous[0] == generation
            and previous[1] == model_id
            and previous[2] is not None
            and previous[3] is not None
        ):
            dt = timestamp - previous[2]
            if 1e-4 < dt <= 0.1:
                sample["observed_yaw_rate"] = _signed_angle_error(
                    observed_yaw, previous[3]
                ) / dt
        if all(value is not None for value in current):
            previous = current
        else:
            previous = None
    return augmented_samples


def _prediction_bucket(records, eligible_count):
    stats = {}
    for field in (
        "time_alignment_error",
        "baseline_position_error",
        "prediction_position_error",
        "position_improvement",
        "baseline_yaw_error",
        "prediction_yaw_error",
        "signed_prediction_yaw_error",
        "equivalent_dt_correction",
        "delay_time",
        "base_prediction_dt",
        "fly_time",
        "command_yaw",
        "command_pitch",
        "command_yaw_deg",
        "command_pitch_deg",
        "capture_to_detector_ms",
        "detector_ms",
        "tracker_ms",
        "aimer_ms",
        "capture_to_aimer_ms",
    ):
        summary = _summarize([record[field] for record in records if field in record])
        if summary is not None:
            stats[field] = summary

    return {
        "eligible_count": eligible_count,
        "matched_count": len(records),
        "match_rate": len(records) / eligible_count if eligible_count else None,
        "high_speed_mode_count": sum(
            record.get("high_speed_mode") == 1 for record in records
        ),
        "high_speed_mode_rate": (
            sum(record.get("high_speed_mode") == 1 for record in records) / len(records)
            if records and any("high_speed_mode" in record for record in records)
            else None
        ),
        "prediction_better_rate": (
            sum(
                record["prediction_position_error"] < record["baseline_position_error"]
                for record in records
            ) / len(records)
            if records
            else None
        ),
        "stats": stats,
    }


def _analyze_aimer_predictions(samples, limits):
    """将当前预测与 t + prediction_dt 附近的同 ID 实测装甲板对齐。"""
    ordered = sorted(samples, key=lambda sample: sample["timestamp"])
    timestamps = [sample["timestamp"] for sample in ordered]
    tolerance = limits["prediction_match_tolerance"]
    records = {"overall": [], "static": [], "spin": []}
    eligible_counts = {"overall": 0, "static": 0, "spin": 0}

    for source in ordered:
        required = (
            "prediction_dt",
            "aim_armor_id",
            "tracker_generation",
            "aim_current_x",
            "aim_current_y",
            "aim_current_z",
            "future_x",
            "future_y",
            "future_z",
        )
        if not all(field in source for field in required):
            continue
        if source["prediction_dt"] <= 0.0:
            continue

        target_timestamp = source["timestamp"] + source["prediction_dt"]
        # 采集尾部没有对应的未来观测，不应计入匹配率分母。
        if not timestamps or target_timestamp > timestamps[-1]:
            continue

        phase_name = None
        angular_velocity = source.get("angular_velocity")
        if angular_velocity is not None:
            angular_speed = abs(angular_velocity)
            if angular_speed <= limits["static_angular"]:
                phase_name = "static"
            elif angular_speed >= limits["spin_angular"]:
                phase_name = "spin"

        eligible_counts["overall"] += 1
        if phase_name is not None:
            eligible_counts[phase_name] += 1

        begin = bisect.bisect_left(timestamps, target_timestamp - tolerance)
        best_match = None
        best_time_error = math.inf
        for candidate in ordered[begin:]:
            time_error = abs(candidate["timestamp"] - target_timestamp)
            if candidate["timestamp"] > target_timestamp + tolerance:
                break
            if candidate["timestamp"] <= source["timestamp"]:
                continue
            if candidate.get("tracker_generation") != source["tracker_generation"]:
                continue
            observation = _accepted_armor_observation(candidate, source["aim_armor_id"])
            if observation is None or time_error >= best_time_error:
                continue
            best_match = observation
            best_time_error = time_error

        if best_match is None:
            continue

        baseline_xyz = tuple(source[f"aim_current_{axis}"] for axis in "xyz")
        prediction_xyz = tuple(source[f"future_{axis}"] for axis in "xyz")
        observed_xyz = best_match["xyz"]
        baseline_error = math.dist(baseline_xyz, observed_xyz)
        prediction_error = math.dist(prediction_xyz, observed_xyz)
        record = {
            "time_alignment_error": best_time_error,
            "baseline_position_error": baseline_error,
            "prediction_position_error": prediction_error,
            "position_improvement": baseline_error - prediction_error,
        }
        if "high_speed_mode" in source:
            record["high_speed_mode"] = source["high_speed_mode"]
        for field in ("delay_time", "base_prediction_dt", "fly_time"):
            if field in source:
                record[field] = source[field]
        for field in (
            "command_yaw",
            "command_pitch",
            "command_yaw_deg",
            "command_pitch_deg",
        ):
            if field in source:
                record[field] = source[field]
        for field in (
            "capture_to_detector_ms",
            "detector_ms",
            "tracker_ms",
            "aimer_ms",
            "capture_to_aimer_ms",
        ):
            if field in source:
                record[field] = source[field]
        observed_yaw = best_match["yaw"]
        if (
            observed_yaw is not None
            and "aim_current_yaw" in source
            and "future_yaw" in source
        ):
            record["baseline_yaw_error"] = _wrapped_angle_error(
                source["aim_current_yaw"], observed_yaw
            )
            record["prediction_yaw_error"] = _wrapped_angle_error(
                source["future_yaw"], observed_yaw
            )
            record["signed_prediction_yaw_error"] = _signed_angle_error(
                source["future_yaw"], observed_yaw
            )
            angular_velocity = source.get("angular_velocity")
            if angular_velocity is not None and abs(angular_velocity) >= limits["spin_angular"]:
                # 预测角度 - 实测角度为正，表示模型相位偏超前；修正量应反向调整 dt。
                record["equivalent_dt_correction"] = (
                    -record["signed_prediction_yaw_error"] / angular_velocity
                )

        records["overall"].append(record)
        if phase_name is not None:
            records[phase_name].append(record)

    return {
        name: _prediction_bucket(records[name], eligible_counts[name])
        for name in ("overall", "static", "spin")
    }


def _analyze_phase(samples):
    stats = {}
    for field in ANALYZED_FIELDS:
        values = [
            sample[field]
            for sample in samples
            if field in sample
            and (field not in INVALID_NEGATIVE_FIELDS or sample[field] >= 0.0)
        ]
        summary = _summarize(values)
        if summary is not None:
            stats[field] = summary

    switch_steps = []
    same_id_steps = []
    reset_steps = []
    switch_count = 0
    aim_switch_count = 0
    aim_switch_steps = []
    primary_switch_count = 0
    primary_switch_steps = []
    reset_count = 0
    for previous, current in zip(samples, samples[1:]):
        required = ("center_x", "center_y", "current_armor_id")
        if not all(field in previous and field in current for field in required):
            continue
        step = math.hypot(
            current["center_x"] - previous["center_x"],
            current["center_y"] - previous["center_y"],
        )
        if (
            "tracker_generation" in previous
            and "tracker_generation" in current
            and current["tracker_generation"] != previous["tracker_generation"]
        ):
            reset_count += 1
            reset_steps.append(step)
            continue
        if current["current_armor_id"] != previous["current_armor_id"]:
            switch_count += 1
            switch_steps.append(step)
        else:
            same_id_steps.append(step)

        if (
            "aim_armor_id" in previous
            and "aim_armor_id" in current
            and current["aim_armor_id"] != previous["aim_armor_id"]
        ):
            aim_switch_count += 1
            aim_switch_steps.append(step)

        if (
            "association_primary_id" in previous
            and "association_primary_id" in current
            and current["association_primary_id"] != previous["association_primary_id"]
        ):
            primary_switch_count += 1
            primary_switch_steps.append(step)

    association_frame_count = 0
    two_candidate_count = 0
    duplicate_model_id_count = 0
    rejected_candidate_frame_count = 0
    gate_rejected_frame_count = 0
    gate_rejection_counts = {
        "angle": 0,
        "score": 0,
        "position": 0,
        "distance": 0,
        "mahalanobis": 0,
    }
    gate_evaluated_candidate_count = 0
    gate_rejected_candidate_count = 0
    for sample in samples:
        candidate_count = sample.get("association_candidate_count")
        accepted_count = sample.get("association_accepted_count")
        if candidate_count is None or accepted_count is None:
            continue
        association_frame_count += 1
        if accepted_count < candidate_count:
            rejected_candidate_frame_count += 1
        gate_passed = []
        if sample.get("association_primary_gate_passed") is not None:
            gate_passed.append(sample["association_primary_gate_passed"])
        if candidate_count >= 2 and sample.get("association_secondary_gate_passed") is not None:
            gate_passed.append(sample["association_secondary_gate_passed"])
        if gate_passed and not all(gate_passed):
            gate_rejected_frame_count += 1
        for prefix in ("association_primary", "association_secondary"):
            candidate_gate = sample.get(f"{prefix}_gate_passed")
            if candidate_gate is None:
                continue
            gate_evaluated_candidate_count += 1
            if candidate_gate:
                continue
            gate_rejected_candidate_count += 1
            for reason, suffix in (
                ("angle", "angle_gate_passed"),
                ("score", "score_gate_passed"),
                ("position", "position_gate_passed"),
                ("distance", "distance_gate_passed"),
                ("mahalanobis", "mahalanobis_gate_passed"),
            ):
                if sample.get(f"{prefix}_{suffix}") == 0:
                    gate_rejection_counts[reason] += 1
        if candidate_count < 2:
            continue
        primary_id = sample.get("association_primary_id")
        secondary_id = sample.get("association_secondary_id")
        if primary_id is None or secondary_id is None:
            continue
        two_candidate_count += 1
        if primary_id == secondary_id:
            duplicate_model_id_count += 1

    return {
        "sample_count": len(samples),
        "stats": stats,
        "id_switch_count": switch_count,
        "id_switch_step_p95": _percentile(switch_steps, 0.95),
        "aim_id_switch_count": aim_switch_count,
        "aim_id_switch_step_p95": _percentile(aim_switch_steps, 0.95),
        "association_primary_switch_count": primary_switch_count,
        "association_primary_switch_step_p95": _percentile(primary_switch_steps, 0.95),
        "same_id_step_p95": _percentile(same_id_steps, 0.95),
        "tracker_reset_count": reset_count,
        "tracker_reset_step_p95": _percentile(reset_steps, 0.95),
        "association_frame_count": association_frame_count,
        "two_candidate_count": two_candidate_count,
        "duplicate_model_id_rate": (
            duplicate_model_id_count / two_candidate_count
            if two_candidate_count
            else None
        ),
        "rejected_candidate_frame_rate": (
            rejected_candidate_frame_count / association_frame_count
            if association_frame_count
            else None
        ),
        "gate_rejected_frame_rate": (
            gate_rejected_frame_count / association_frame_count
            if association_frame_count
            else None
        ),
        # 这里只统计 ROS 调试消息保留的主、次两个候选，不冒充全部候选。
        "gate_evaluated_candidate_count": gate_evaluated_candidate_count,
        "gate_rejected_candidate_count": gate_rejected_candidate_count,
        "gate_rejection_counts": gate_rejection_counts,
    }


def _append_diagnosis(diagnoses, level, code, message):
    diagnoses.append({"level": level, "code": code, "message": message})


def analyze_samples(samples, thresholds=None):
    """对采集样本做纯计算分析，供 ROS 入口和单元测试共同使用。"""
    samples = _add_observed_yaw_rate(samples)
    limits = dict(DEFAULT_THRESHOLDS)
    if thresholds is not None:
        limits.update(thresholds)

    static_samples = []
    spin_samples = []
    transition_count = 0
    for sample in samples:
        angular_velocity = sample.get("angular_velocity")
        if angular_velocity is None:
            continue
        angular_speed = abs(angular_velocity)
        if angular_speed <= limits["static_angular"]:
            static_samples.append(sample)
        elif angular_speed >= limits["spin_angular"]:
            spin_samples.append(sample)
        else:
            transition_count += 1

    phases = {
        "static": _analyze_phase(static_samples),
        "spin": _analyze_phase(spin_samples),
    }
    diagnoses = []

    for phase_name, phase_samples in (
        ("static", static_samples),
        ("spin", spin_samples),
    ):
        phase = phases[phase_name]
        if phase["sample_count"] < limits["minimum_samples"]:
            _append_diagnosis(
                diagnoses,
                "WARN",
                f"{phase_name}_samples",
                f"{phase_name} 阶段只有 {phase['sample_count']} 个样本，暂不足以判断。",
            )
            continue

        stats = phase["stats"]
        # 以同帧装甲板尺寸归一化，只检查真正进入 EKF 的有效观测。
        ekf_error_ratios = [
            sample["current_ekf_error"] / sample["armor_pixel_long_side"]
            for sample in phase_samples
            if sample.get("association_accepted_count") == 1
            and sample.get("current_ekf_error", -1.0) >= 0.0
            and sample.get("armor_pixel_long_side", 0.0) > 0.0
        ]
        excessive_ratios = [
            ratio
            for ratio in ekf_error_ratios
            if ratio > limits["ekf_reprojection_armor_ratio"]
        ]
        if excessive_ratios:
            _append_diagnosis(
                diagnoses,
                "WARN",
                f"{phase_name}_ekf_reprojection",
                f"{phase_name} 阶段有 {len(excessive_ratios)} 帧已接收观测的 "
                f"EKF 回投影误差超过装甲板长边的 "
                f"{limits['ekf_reprojection_armor_ratio']:.2f} 倍；"
                f"最大为 {max(excessive_ratios):.2f} 倍。",
            )
        center_spans = [
            stats[field]["robust_span"]
            for field in ("center_x", "center_y")
            if field in stats
        ]
        if center_spans and max(center_spans) > limits["center_span"]:
            _append_diagnosis(
                diagnoses,
                "WARN",
                f"{phase_name}_center_span",
                f"{phase_name} 阶段旋转中心 P95-P05={max(center_spans):.3f}m，"
                f"超过阈值 {limits['center_span']:.3f}m。",
            )

        center_speed = stats.get("center_speed")
        if center_speed and center_speed["p95"] > limits["center_speed"]:
            _append_diagnosis(
                diagnoses,
                "WARN",
                f"{phase_name}_center_speed",
                f"{phase_name} 阶段中心速度 P95={center_speed['p95']:.3f}m/s，"
                f"超过阈值 {limits['center_speed']:.3f}m/s。",
            )

        for radius_field in ("radius", "alternate_radius"):
            radius = stats.get(radius_field)
            if radius and radius["span"] > limits["radius_span"]:
                _append_diagnosis(
                    diagnoses,
                    "WARN",
                    f"{phase_name}_{radius_field}_span",
                    f"{phase_name} 阶段 {radius_field} 峰峰值 {radius['span']:.3f}m，"
                    f"超过阈值 {limits['radius_span']:.3f}m。",
                )

        switch_step = phase["id_switch_step_p95"]
        if switch_step is not None and switch_step > limits["id_jump"]:
            _append_diagnosis(
                diagnoses,
                "WARN",
                f"{phase_name}_id_jump",
                f"{phase_name} 阶段 ID 切换中心跳变量 P95={switch_step:.3f}m，"
                f"超过阈值 {limits['id_jump']:.3f}m。",
            )

    warning_codes = {item["code"] for item in diagnoses if item["level"] == "WARN"}
    if "spin_center_span" in warning_codes:
        radius_is_unstable = any(
            code in warning_codes
            for code in ("spin_radius_span", "spin_alternate_radius_span")
        )
        id_jump_is_large = "spin_id_jump" in warning_codes
        if radius_is_unstable:
            conclusion = "小陀螺中心摆动同时伴随半径变化，优先检查半径模型和 EKF 参数。"
        elif id_jump_is_large:
            conclusion = "半径相对稳定，但 ID 切换伴随中心跳变，优先检查关联和 PnP 朝向。"
        else:
            conclusion = "小陀螺中心摆动未直接对应半径或 ID 跳变，继续检查坐标系和 PnP。"
    elif "spin_ekf_reprojection" in warning_codes:
        conclusion = (
            "小陀螺车辆中心虽未超过阈值，但部分已接收观测的 EKF "
            "回投影偏差超过装甲板尺度，应检查关联、PnP 和滤波更新。"
        )
    elif phases["spin"]["sample_count"] >= limits["minimum_samples"]:
        conclusion = "小陀螺阶段车辆中心未超过当前阈值。"
    else:
        conclusion = "小陀螺有效样本不足，无法形成结论。"

    return {
        "sample_count": len(samples),
        "transition_sample_count": transition_count,
        "thresholds": limits,
        "phases": phases,
        "range_buckets": _analyze_range_buckets(
            samples, limits["range_bin_size"]
        ),
        "aimer_prediction": _analyze_aimer_predictions(samples, limits),
        "outliers": _extract_outlier_events(samples),
        "diagnoses": diagnoses,
        "conclusion": conclusion,
    }


def _format_summary(summary, unit):
    return (
        f"mean={summary['mean']:.4f}{unit} "
        f"span={summary['span']:.4f}{unit} "
        f"P95-P05={summary['robust_span']:.4f}{unit} "
        f"p95={summary['p95']:.4f}{unit}"
    )


def _format_outlier_event(event, metric_name, unit):
    def value(name, default="-"):
        return event.get(name, default)

    metric = value(metric_name)
    metric_text = f"{metric:.4f}{unit}" if isinstance(metric, (int, float)) else str(metric)
    score = value("association_primary_score")
    score_text = f"{score:.4f}" if isinstance(score, (int, float)) else str(score)
    raw_yaw = value("association_primary_raw_yaw")
    raw_yaw_text = f"{raw_yaw:.4f}" if isinstance(raw_yaw, (int, float)) else str(raw_yaw)
    optimized_yaw = value("association_primary_optimized_yaw")
    optimized_yaw_text = (
        f"{optimized_yaw:.4f}"
        if isinstance(optimized_yaw, (int, float))
        else str(optimized_yaw)
    )
    position_error = value("association_primary_position_error")
    position_error_text = (
        f"{position_error:.4f}"
        if isinstance(position_error, (int, float))
        else str(position_error)
    )
    distance_error = value("association_primary_distance_error")
    distance_error_text = (
        f"{distance_error:.4f}"
        if isinstance(distance_error, (int, float))
        else str(distance_error)
    )
    return (
        f"t={value('timestamp'):.3f}s {metric_name}={metric_text} "
        f"gen={value('tracker_generation')} id={value('current_armor_id')} "
        f"assoc_id={value('association_primary_id')} score={score_text} "
        f"pos_err={position_error_text}m dist_err={distance_error_text}m "
        f"raw_yaw={raw_yaw_text} optimized_yaw={optimized_yaw_text}"
    )


def print_report(report):
    print("\n========== Tracker 自动分析 ==========")
    print(
        f"总样本: {report['sample_count']}，"
        f"过渡阶段样本: {report['transition_sample_count']}"
    )
    units = {
        "center_x": "m",
        "center_y": "m",
        "center_speed": "m/s",
        "vx": "m/s",
        "vy": "m/s",
        "angular_velocity": "rad/s",
        "observed_yaw_rate": "rad/s",
        "radius": "m",
        "alternate_radius": "m",
        "current_ekf_error": "px",
        "post_update_position_error": "m",
        "post_update_bearing_error": "rad",
        "post_update_distance_error": "m",
        "post_update_orientation_error": "rad",
        "pnp_error": "px",
        "nis": "",
        "nis_failure_rate": "",
        "association_primary_score": "rad",
        "association_primary_position_error": "m",
        "association_primary_distance_error": "m",
        "association_primary_mahalanobis_distance": "",
        "association_primary_position_angle_error": "rad",
        "association_primary_distance_angle_error": "rad",
        "association_primary_orientation_error": "rad",
        "association_primary_bearing_error": "rad",
        "association_primary_yaw_correction_abs": "rad",
        "association_secondary_score": "rad",
        "association_secondary_position_error": "m",
        "association_secondary_distance_error": "m",
        "association_secondary_mahalanobis_distance": "",
        "association_secondary_position_angle_error": "rad",
        "association_secondary_distance_angle_error": "rad",
        "association_secondary_yaw_correction_abs": "rad",
    }
    for phase_name, title in (("static", "静止阶段"), ("spin", "小陀螺阶段")):
        phase = report["phases"][phase_name]
        print(f"\n[{title}] 样本数={phase['sample_count']} ID切换={phase['id_switch_count']}")
        for field in units:
            summary = phase["stats"].get(field)
            if summary is not None:
                print(f"  {field:22s} {_format_summary(summary, units[field])}")
        if phase["id_switch_step_p95"] is not None:
            print(f"  ID切换中心跳变量P95   {phase['id_switch_step_p95']:.4f}m")
        if phase["aim_id_switch_count"]:
            print(
                f"  瞄准ID切换={phase['aim_id_switch_count']} "
                f"中心跳变量P95={phase['aim_id_switch_step_p95']:.4f}m"
            )
        if phase["association_primary_switch_count"]:
            print(
                f"  主关联ID切换={phase['association_primary_switch_count']} "
                f"中心跳变量P95={phase['association_primary_switch_step_p95']:.4f}m"
            )
        if phase["tracker_reset_count"]:
            reset_step = phase["tracker_reset_step_p95"]
            print(
                f"  Tracker重初始化={phase['tracker_reset_count']} "
                f"中心跳变量P95={reset_step:.4f}m"
            )
        if phase["two_candidate_count"]:
            print(
                f"  双候选帧={phase['two_candidate_count']} "
                f"同模型ID比例={phase['duplicate_model_id_rate']:.3f}"
            )
        if phase["association_frame_count"]:
            print(
                "  候选未完全接收帧比例="
                f"{phase['rejected_candidate_frame_rate']:.3f}"
            )
            if phase["gate_rejected_frame_rate"] is not None:
                print(f"  关联门限拒绝帧比例={phase['gate_rejected_frame_rate']:.3f}")

    print("\n[Aimer 时间对齐预测]")
    for phase_name, title in (
        ("overall", "全部"),
        ("static", "静止"),
        ("spin", "小陀螺"),
    ):
        prediction = report["aimer_prediction"][phase_name]
        eligible_count = prediction["eligible_count"]
        matched_count = prediction["matched_count"]
        if not eligible_count:
            print(f"  {title}: 无有效预测样本")
            continue
        print(
            f"  {title}: 有效预测={eligible_count} 成功匹配={matched_count} "
            f"匹配率={prediction['match_rate']:.3f}"
        )
        if not matched_count:
            continue
        stats = prediction["stats"]
        alignment = stats["time_alignment_error"]
        baseline = stats["baseline_position_error"]
        predicted = stats["prediction_position_error"]
        improvement = stats["position_improvement"]
        print(
            f"    时间对齐误差P95={alignment['p95'] * 1000.0:.2f}ms "
            f"不预测位置误差P95={baseline['p95']:.4f}m "
            f"预测后位置误差P95={predicted['p95']:.4f}m"
        )
        print(
            f"    平均位置改善={improvement['mean']:.4f}m "
            f"预测优于不预测比例={prediction['prediction_better_rate']:.3f}"
        )
        if prediction["high_speed_mode_rate"] is not None:
            print(
                f"    Aimer高速模式帧={prediction['high_speed_mode_count']} "
                f"比例={prediction['high_speed_mode_rate']:.3f}"
            )
        for field, label in (
            ("delay_time", "观测到Aimer实测处理延迟"),
            ("base_prediction_dt", "基础预测时间"),
            ("fly_time", "弹丸飞行时间"),
        ):
            summary = stats.get(field)
            if summary is not None:
                print(
                    f"    {label} mean={summary['mean'] * 1000.0:.2f}ms "
                    f"P95={summary['p95'] * 1000.0:.2f}ms"
                )
        for field, label in (
            ("capture_to_detector_ms", "图像回调到检测"),
            ("detector_ms", "检测耗时"),
            ("tracker_ms", "Tracker耗时"),
            ("aimer_ms", "Aimer耗时"),
            ("capture_to_aimer_ms", "图像回调到Aimer"),
        ):
            summary = stats.get(field)
            if summary is not None:
                print(
                    f"    {label} mean={summary['mean']:.3f}ms "
                    f"P95={summary['p95']:.3f}ms"
                )
        for field, label, unit in (
            ("command_yaw_deg", "命令yaw", "deg"),
            ("command_pitch_deg", "命令pitch", "deg"),
        ):
            summary = stats.get(field)
            if summary is not None:
                print(
                    f"    {label} mean={summary['mean']:.2f}{unit} "
                    f"P05-P95={summary['p05']:.2f}..{summary['p95']:.2f}{unit}"
                )
        baseline_yaw = stats.get("baseline_yaw_error")
        predicted_yaw = stats.get("prediction_yaw_error")
        if baseline_yaw is not None and predicted_yaw is not None:
            print(
                f"    不预测yaw误差P95={baseline_yaw['p95']:.4f}rad "
                f"预测后yaw误差P95={predicted_yaw['p95']:.4f}rad"
            )
        signed_yaw = stats.get("signed_prediction_yaw_error")
        dt_correction = stats.get("equivalent_dt_correction")
        if signed_yaw is not None and dt_correction is not None:
            print(
                f"    带符号预测yaw残差 mean={signed_yaw['mean']:.4f}rad "
                f"median={signed_yaw['median']:.4f}rad "
                f"等效dt修正 mean={dt_correction['mean'] * 1000.0:.2f}ms "
                f"median={dt_correction['median'] * 1000.0:.2f}ms"
            )

    print("\n[距离分段]")
    for bucket in report["range_buckets"]:
        stats = bucket["stats"]
        print(
            f"  {bucket['minimum_distance']:.1f}~{bucket['maximum_distance']:.1f}m "
            f"样本数={bucket['sample_count']} "
            f"Tracker世代数={bucket['tracker_generation_count']} "
            f"ID切换={bucket['id_switch_count']}"
        )
        for field, label, unit in (
            ("armor_pixel_long_side", "像素长边", "px"),
            ("armor_pixel_short_side", "像素短边", "px"),
            ("pnp_error", "PnP误差", "px"),
            ("current_ekf_error", "EKF回投影误差", "px"),
            ("post_update_position_error", "EKF后验位置残差", "m"),
            ("post_update_bearing_error", "EKF后验方位残差", "rad"),
            ("post_update_distance_error", "EKF后验距离残差", "m"),
            ("post_update_orientation_error", "EKF后验装甲板角残差", "rad"),
            ("association_primary_position_error", "关联位置误差", "m"),
            ("association_primary_distance_error", "关联距离误差", "m"),
            ("center_speed", "中心速度", "m/s"),
        ):
            summary = stats.get(field)
            if summary is not None:
                print(
                    f"    {label} P95={summary['p95']:.4f}{unit} "
                    f"P05-P95={summary['p05']:.4f}..{summary['p95']:.4f}{unit}"
                )

    print("\n[异常帧 Top 3]")
    for event in report["outliers"]["center_jump"][:3]:
        print(f"  中心跳变 {_format_outlier_event(event, 'center_step', 'm')}")
    for event in report["outliers"]["ekf_error"][:3]:
        print(f"  EKF误差  {_format_outlier_event(event, 'current_ekf_error', 'px')}")

    print("\n[诊断]")
    if report["diagnoses"]:
        for diagnosis in report["diagnoses"]:
            print(f"  [{diagnosis['level']}] {diagnosis['message']}")
    else:
        print("  [OK] 当前统计项均未超过阈值。")
    print(f"\n结论：{report['conclusion']}")


def collect_ros_samples(args):
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
    except ImportError as exception:
        raise RuntimeError("未找到 ROS2 Python 环境，请先 source /opt/ros/humble/setup.bash") from exception

    rclpy.init()
    node = rclpy.create_node("sim_tracker_analyzer")
    samples = []
    invalid_count = 0
    start_time = time.monotonic()

    # 与探针的 BEST_EFFORT + VOLATILE 保持一致，避免 QoS 不兼容导致收不到数据。
    qos = QoSProfile(depth=100)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE

    def on_message(message):
        nonlocal invalid_count
        try:
            payload = json.loads(message.data)
            samples.append(normalize_sample(payload, time.monotonic() - start_time))
        except (json.JSONDecodeError, TypeError, ValueError):
            invalid_count += 1

    # 保留订阅对象直到采集结束，避免生命周期短于事件循环。
    subscription = node.create_subscription(String, args.topic, on_message, qos)
    deadline = start_time + args.duration if args.duration > 0 else math.inf
    next_progress = start_time + 5.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_progress:
                elapsed = time.monotonic() - start_time
                print(f"已采集 {elapsed:.1f}s，样本数={len(samples)}")
                next_progress += 5.0
    except KeyboardInterrupt:
        print("收到 Ctrl-C，使用当前样本生成报告。")
    finally:
        del subscription
        node.destroy_node()
        rclpy.shutdown()

    if invalid_count:
        print(f"忽略了 {invalid_count} 条无效 JSON 消息。")
    return samples


def _positive_float(value):
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("必须是有限正数")
    return parsed


def parse_args():
    parser = argparse.ArgumentParser(
        description="自动分析 /sim_aim/debug 中的 Tracker 与 Aimer 数据"
    )
    parser.add_argument("--topic", default="/sim_aim/debug")
    parser.add_argument("--duration", type=float, default=30.0, help="采集秒数，0 表示直到 Ctrl-C")
    parser.add_argument("--static-angular-threshold", type=float, default=0.3)
    parser.add_argument("--spin-angular-threshold", type=float, default=1.0)
    parser.add_argument("--center-span-threshold", type=float, default=0.10)
    parser.add_argument("--center-speed-threshold", type=float, default=0.20)
    parser.add_argument("--radius-span-threshold", type=float, default=0.05)
    parser.add_argument("--id-jump-threshold", type=float, default=0.05)
    parser.add_argument(
        "--ekf-reprojection-armor-ratio",
        type=_positive_float,
        default=1.0,
        help="已接收观测的 EKF 回投影误差相对装甲板像素长边的报警倍数",
    )
    parser.add_argument(
        "--prediction-match-tolerance",
        type=_positive_float,
        default=0.03,
        help="未来预测与实测样本的最大时间差，单位为秒",
    )
    parser.add_argument(
        "--range-bin-size",
        type=_positive_float,
        default=1.0,
        help="目标距离分桶大小，单位为米",
    )
    parser.add_argument("--minimum-samples", type=int, default=20)
    parser.add_argument("--output", help="可选的 JSON 报告输出路径")
    return parser.parse_args()


def main():
    args = parse_args()
    thresholds = {
        "static_angular": args.static_angular_threshold,
        "spin_angular": args.spin_angular_threshold,
        "center_span": args.center_span_threshold,
        "center_speed": args.center_speed_threshold,
        "radius_span": args.radius_span_threshold,
        "id_jump": args.id_jump_threshold,
        "ekf_reprojection_armor_ratio": args.ekf_reprojection_armor_ratio,
        "prediction_match_tolerance": args.prediction_match_tolerance,
        "range_bin_size": args.range_bin_size,
        "minimum_samples": args.minimum_samples,
    }
    samples = collect_ros_samples(args)
    if not samples:
        print("未收到有效调试数据，请检查 sim_detector_probe 和 Topic QoS。")
        return 2

    report = analyze_samples(samples, thresholds)
    print_report(report)
    if args.output:
        output_path = Path(args.output)
        output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"报告已保存到: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
