#!/usr/bin/env python3
"""离线扫描 PnP 物点尺寸对仿真 TF 真值残差的影响。"""

import argparse
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class Calibration:
    camera_matrix: np.ndarray
    distortion: np.ndarray
    rotation_camera2gimbal: np.ndarray
    translation_camera2gimbal: np.ndarray
    rotation_gimbal2imubody: np.ndarray
    lightbar_length: float = 0.056


def _as_matrix(values, rows, columns, name):
    array = np.asarray(values, dtype=np.float64)
    if array.size != rows * columns:
        raise ValueError(f"{name} 必须包含 {rows * columns} 个数值")
    return array.reshape(rows, columns)


def load_calibration(path):
    """读取与 Solver 相同的相机、静态外参和 PnP 灯条长度配置。"""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return Calibration(
        camera_matrix=_as_matrix(data["camera_matrix"], 3, 3, "camera_matrix"),
        distortion=_as_matrix(data["distort_coeffs"], 1, 5, "distort_coeffs"),
        rotation_camera2gimbal=_as_matrix(
            data["R_camera2gimbal"], 3, 3, "R_camera2gimbal"
        ),
        translation_camera2gimbal=_as_matrix(
            data["t_camera2gimbal"], 3, 1, "t_camera2gimbal"
        ).reshape(3),
        rotation_gimbal2imubody=_as_matrix(
            data["R_gimbal2imubody"], 3, 3, "R_gimbal2imubody"
        ),
        lightbar_length=float(data.get("pnp_lightbar_length", 0.056)),
    )


def quaternion_to_matrix(x, y, z, w):
    """将 ROS xyzw 四元数转换为旋转矩阵，并拒绝无效姿态。"""
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError("gimbal_tf 四元数无效")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def gimbal_to_world_rotation(sample, calibration):
    """复现 Solver::set_R_gimbal2world 的坐标变换。"""
    rotation_imu2world = quaternion_to_matrix(
        sample["gimbal_tf_qx"],
        sample["gimbal_tf_qy"],
        sample["gimbal_tf_qz"],
        sample["gimbal_tf_qw"],
    )
    return (
        calibration.rotation_gimbal2imubody.T
        @ rotation_imu2world
        @ calibration.rotation_gimbal2imubody
    )


def armor_object_points(armor_type, width, lightbar_length, point_offset=None):
    """生成物点，并支持给所有物点施加装甲坐标系固定偏移。"""
    if armor_type not in ("small", "big"):
        raise ValueError(f"不支持的装甲板模型: {armor_type}")
    half_width = width / 2.0
    half_height = lightbar_length / 2.0
    points = np.asarray(
        [
            [0.0, half_width, half_height],
            [0.0, -half_width, half_height],
            [0.0, -half_width, -half_height],
            [0.0, half_width, -half_height],
        ],
        dtype=np.float64,
    )
    if point_offset is not None:
        offset = np.asarray(point_offset, dtype=np.float64)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("point_offset 必须是有限的三维向量")
        points += offset
    return points


def _sample_image_points(sample):
    points = []
    for index in range(4):
        x_key = f"accepted_corner_{index}_x"
        y_key = f"accepted_corner_{index}_y"
        if x_key not in sample or y_key not in sample:
            raise ValueError(f"样本缺少 {x_key}/{y_key}")
        points.append([sample[x_key], sample[y_key]])
    return np.asarray(points, dtype=np.float64)


def _sample_truth_position(sample):
    keys = ("association_primary_truth_x", "association_primary_truth_y", "association_primary_truth_z")
    if not all(key in sample for key in keys):
        raise ValueError("样本缺少 association_primary_truth_x/y/z")
    return np.asarray([sample[key] for key in keys], dtype=np.float64)


def evaluate_sample(sample, armor_type, width, lightbar_length, calibration):
    """评估全部 IPPE 分支，并分别保留在线、重投影和真值选择结果。"""
    try:
        object_points = armor_object_points(armor_type, width, lightbar_length)
        image_points = _sample_image_points(sample)
        truth_position = _sample_truth_position(sample)
        rotation_gimbal2world = gimbal_to_world_rotation(sample, calibration)
    except (KeyError, ValueError):
        # 探针启动、TF 暂缺或 Tracker 未完成首帧时，消息可能没有完整诊断字段；
        # 离线统计跳过该帧，不能把采样不完整误判成几何失败。
        return None

    result = cv2.solvePnPGeneric(
        object_points,
        image_points,
        calibration.camera_matrix,
        calibration.distortion,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if len(result) < 3 or not result[0]:
        return None

    candidates = []
    for rvec, tvec in zip(result[1], result[2]):
        rotation_armor2camera, _ = cv2.Rodrigues(rvec)
        rotation_armor2world = (
            rotation_gimbal2world
            @ calibration.rotation_camera2gimbal
            @ rotation_armor2camera
        )
        position_camera = np.asarray(tvec, dtype=np.float64).reshape(3)
        position_gimbal = (
            calibration.rotation_camera2gimbal @ position_camera
            + calibration.translation_camera2gimbal
        )
        position_world = rotation_gimbal2world @ position_gimbal
        projected, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            calibration.camera_matrix,
            calibration.distortion,
        )
        reprojection_error = float(
            np.mean(np.linalg.norm(image_points - projected.reshape(4, 2), axis=1))
        )
        candidates.append(
            {
                "position_world": position_world,
                "position_error": float(np.linalg.norm(position_world - truth_position)),
                "residual": position_world - truth_position,
                "rotation_armor2world": rotation_armor2world,
                "reprojection_error": reprojection_error,
                # 当前场景 pitch 不接近 90 度；该公式与 Solver 的 ZYX yaw 一致。
                "yaw": float(math.atan2(rotation_armor2world[1, 0], rotation_armor2world[0, 0])),
            }
        )

    if not candidates:
        return None

    truth_best = min(candidates, key=lambda candidate: candidate["position_error"])
    reprojection_best = min(candidates, key=lambda candidate: candidate["reprojection_error"])
    reference_yaw = sample.get("association_primary_optimized_yaw")
    if reference_yaw is None:
        runtime_best = reprojection_best
    else:
        runtime_best = min(
            candidates,
            key=lambda candidate: abs(
                math.atan2(
                    math.sin(candidate["yaw"] - reference_yaw),
                    math.cos(candidate["yaw"] - reference_yaw),
                )
            ),
        )
    # 顶层字段保留真值最近分支，兼容测试和调用方；汇总报告明确区分三种选择。
    return {
        "position_error": truth_best["position_error"],
        "residual": truth_best["residual"],
        "reprojection_error": truth_best["reprojection_error"],
        "truth_best": truth_best,
        "reprojection_best": reprojection_best,
        "runtime_best": runtime_best,
    }


def _percentile(values, ratio):
    ordered = sorted(values)
    if not ordered:
        return None
    position = ratio * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def fit_origin_offset(samples, armor_type, width, lightbar_length, calibration):
    """拟合应加到全部 PnP 物点上的固定装甲坐标系偏移。

    若当前 PnP 原点相对仿真中心存在固定偏移 o，则：
        current_position - truth_position ≈ R_armor2world @ o

    该结果只用于诊断，不会自动写入在线 Solver。
    """
    evaluations = [
        evaluate_sample(sample, armor_type, width, lightbar_length, calibration)
        for sample in samples
    ]
    evaluations = [evaluation for evaluation in evaluations if evaluation is not None]
    if not evaluations:
        return None

    runtime_results = [evaluation["runtime_best"] for evaluation in evaluations]
    design_matrix = np.vstack(
        [result["rotation_armor2world"] for result in runtime_results]
    )
    residual_vector = np.concatenate(
        [result["residual"] for result in runtime_results]
    )
    offset, _, _, _ = np.linalg.lstsq(
        design_matrix, residual_vector, rcond=None
    )
    corrected_residuals = (
        residual_vector - design_matrix @ offset
    ).reshape(-1, 3)
    corrected_errors = np.linalg.norm(corrected_residuals, axis=1)

    return {
        "sample_count": len(runtime_results),
        "point_offset_to_add": offset.tolist(),
        "mean_position_error_after_offset": float(statistics.fmean(corrected_errors)),
        "p95_position_error_after_offset": float(
            _percentile(corrected_errors.tolist(), 0.95)
        ),
        "mean_residual_after_offset": corrected_residuals.mean(axis=0).tolist(),
    }


def summarize_configuration(samples, armor_type, width, lightbar_length, calibration):
    evaluations = [
        evaluate_sample(sample, armor_type, width, lightbar_length, calibration)
        for sample in samples
    ]
    evaluations = [evaluation for evaluation in evaluations if evaluation is not None]
    if not evaluations:
        return None

    runtime_results = [evaluation["runtime_best"] for evaluation in evaluations]
    truth_results = [evaluation["truth_best"] for evaluation in evaluations]
    runtime_position_errors = [result["position_error"] for result in runtime_results]
    truth_position_errors = [result["position_error"] for result in truth_results]
    runtime_residuals = np.asarray([result["residual"] for result in runtime_results])
    truth_residuals = np.asarray([result["residual"] for result in truth_results])
    runtime_reprojection_errors = [
        result["reprojection_error"] for result in runtime_results
    ]
    return {
        "armor_type": armor_type,
        "width": float(width),
        "lightbar_length": float(lightbar_length),
        "sample_count": len(evaluations),
        # 主指标复现在线 Solver 根据预测 yaw 选择的 IPPE 分支。
        "mean_position_error": float(statistics.fmean(runtime_position_errors)),
        "p95_position_error": float(_percentile(runtime_position_errors, 0.95)),
        "stddev_position_error": float(statistics.pstdev(runtime_position_errors)),
        "mean_residual": runtime_residuals.mean(axis=0).tolist(),
        "mean_reprojection_error": float(statistics.fmean(runtime_reprojection_errors)),
        # truth_best 仅说明“某个 IPPE 分支是否存在可解释真值的可能”，不作为线上结论。
        "truth_best_mean_position_error": float(statistics.fmean(truth_position_errors)),
        "truth_best_p95_position_error": float(_percentile(truth_position_errors, 0.95)),
        "truth_best_mean_residual": truth_residuals.mean(axis=0).tolist(),
    }


def _grid(minimum, maximum, step):
    if step <= 0.0 or maximum < minimum:
        raise ValueError("扫描范围或步长无效")
    count = int(math.floor((maximum - minimum) / step + 0.5))
    return [round(minimum + index * step, 9) for index in range(count + 1)]


def scan_configurations(
    samples,
    armor_type,
    calibration,
    width_min,
    width_max,
    width_step,
    length_min,
    length_max,
    length_step,
):
    results = []
    for width in _grid(width_min, width_max, width_step):
        for lightbar_length in _grid(length_min, length_max, length_step):
            summary = summarize_configuration(
                samples, armor_type, width, lightbar_length, calibration
            )
            if summary is not None:
                results.append(summary)
    results.sort(key=lambda result: (result["mean_position_error"], result["p95_position_error"]))
    return results


def load_samples(path):
    """读取 JSON 数组、逐行 JSON，或 ros2 String 的 data JSON。"""
    text = Path(path).read_text(encoding="utf-8").strip()
    if not text:
        return []
    parsed = json.loads(text) if text.startswith("[") else None
    if parsed is not None:
        return parsed

    samples = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line == "---":
            continue
        payload = json.loads(line)
        if isinstance(payload, dict) and isinstance(payload.get("data"), str):
            payload = json.loads(payload["data"])
        elif isinstance(payload, str):
            payload = json.loads(payload)
        if isinstance(payload, dict):
            samples.append(payload)
    return samples


def _default_range(armor_type):
    if armor_type == "big":
        return 0.210, 0.250, 0.001
    return 0.125, 0.145, 0.001


def main():
    parser = argparse.ArgumentParser(description="扫描仿真 PnP 物点几何参数")
    parser.add_argument("--input", required=True, help="逐行 JSON 或 JSON 数组样本")
    parser.add_argument("--config", default="configs/demo.yaml")
    parser.add_argument("--armor-type", choices=("small", "big"))
    parser.add_argument("--width-min", type=float)
    parser.add_argument("--width-max", type=float)
    parser.add_argument("--width-step", type=float, default=0.001)
    parser.add_argument("--length-min", type=float, default=0.050)
    parser.add_argument("--length-max", type=float, default=0.062)
    parser.add_argument("--length-step", type=float, default=0.0005)
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument(
        "--fit-origin-offset",
        action="store_true",
        help="拟合固定装甲坐标系物点偏移，仅用于离线诊断",
    )
    parser.add_argument("--output")
    args = parser.parse_args()

    calibration = load_calibration(args.config)
    samples = load_samples(args.input)
    if args.armor_type:
        groups = {args.armor_type: samples}
    else:
        groups = {}
        for sample in samples:
            groups.setdefault(sample.get("target_armor_type", "small"), []).append(sample)

    report = {"input": str(args.input), "groups": {}}
    for armor_type, group_samples in sorted(groups.items()):
        width_default_min, width_default_max, _ = _default_range(armor_type)
        width_min = args.width_min if args.width_min is not None else width_default_min
        width_max = args.width_max if args.width_max is not None else width_default_max
        baseline_width = 0.230 if armor_type == "big" else 0.135
        results = scan_configurations(
            group_samples,
            armor_type,
            calibration,
            width_min,
            width_max,
            args.width_step,
            args.length_min,
            args.length_max,
            args.length_step,
        )
        report["groups"][armor_type] = {
            "input_sample_count": len(group_samples),
            "result_count": len(results),
            "baseline": next(
                (
                    result
                    for result in results
                    if abs(result["width"] - (0.230 if armor_type == "big" else 0.135)) < 1e-9
                    and abs(result["lightbar_length"] - calibration.lightbar_length) < 1e-9
                ),
                None,
            ),
            "best": results[0] if results else None,
            "top": results[: max(args.top_k, 0)],
            "origin_offset_fit": (
                fit_origin_offset(
                    group_samples,
                    armor_type,
                    baseline_width,
                    calibration.lightbar_length,
                    calibration,
                )
                if args.fit_origin_offset
                else None
            ),
        }

    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.output:
        Path(args.output).write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"报告已保存到: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
