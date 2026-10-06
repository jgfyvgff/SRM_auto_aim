#!/usr/bin/env python3
"""只读采集曝光/增益样本，并用有边界的响应曲面建议下一组实测参数。"""

import argparse
import json
import math
import random
import subprocess
import tempfile
import time
from pathlib import Path


def parse_float_list(value):
    values = []
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        number = float(item)
        if not math.isfinite(number) or number < 0:
            raise ValueError(f"invalid non-negative number: {item}")
        values.append(number)
    if not values:
        raise ValueError("parameter list must not be empty")
    return values


def load_jsonl(path):
    records = []
    if not path.exists():
        return records
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            records.append(value)
    return records


def finite_mean(records, key):
    values = [
        float(record[key])
        for record in records
        if isinstance(record.get(key), (int, float))
        and math.isfinite(float(record[key]))
    ]
    return sum(values) / len(values) if values else 0.0


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def evaluate_records(records, warmup_frames):
    usable = [
        record
        for record in records
        if isinstance(record.get("captured_frame"), int)
        and record["captured_frame"] > warmup_frames
    ]
    processed = [record for record in usable if record.get("event") == "frame"]
    detected_values = [
        float(record["detected"])
        for record in processed
        if isinstance(record.get("detected"), (int, float))
    ]
    detection_rate = (
        sum(value > 0 for value in detected_values) / len(detected_values)
        if detected_values
        else 0.0
    )
    mean_confidence = finite_mean(processed, "mean_confidence")
    dark_ratio = finite_mean(usable, "image_dark_ratio")
    bright_ratio = finite_mean(usable, "image_bright_ratio")
    sharpness = finite_mean(usable, "image_laplacian_variance")
    clipped_ratio = dark_ratio + bright_ratio
    longest_miss_streak = 0
    current_miss_streak = 0
    pnp_errors = []
    for record in processed:
        if record.get("detected", 0) > 0:
            current_miss_streak = 0
        else:
            current_miss_streak += 1
            longest_miss_streak = max(longest_miss_streak, current_miss_streak)
        for item in record.get("pnp_reprojection_errors_px", []):
            error = item.get("pnp_reprojection_error_px") if isinstance(item, dict) else None
            if isinstance(error, (int, float)) and math.isfinite(error) and error >= 0:
                pnp_errors.append(float(error))
    clipping_score = max(0.0, 1.0 - clipped_ratio / 0.05)
    sharpness_score = min(1.0, sharpness / 250.0)
    score = (
        0.65 * detection_rate
        + 0.20 * min(1.0, max(0.0, mean_confidence))
        + 0.10 * clipping_score
        + 0.05 * sharpness_score
    )
    return {
        "records": len(records),
        "usable_records": len(usable),
        "processed_frames": len(processed),
        "detection_rate": detection_rate,
        "mean_confidence": mean_confidence,
        "dark_ratio": dark_ratio,
        "bright_ratio": bright_ratio,
        "clipped_ratio": clipped_ratio,
        "laplacian_variance": sharpness,
        "longest_miss_streak": longest_miss_streak,
        "pnp_reprojection_p95_px": percentile(pnp_errors, 0.95),
        "score": score,
        "warning": (
            "没有 frame 记录，检测率无法评价；请先确认串口反馈和相机链路"
            if not processed
            else ""
        ),
    }


def invalid_reason(result, expected_frames, min_valid_fraction):
    if result.get("timeout"):
        return "采集超时"
    if result.get("return_code") != 0:
        return f"程序退出码异常: {result.get('return_code')}"
    if result.get("processed_frames", 0) < math.ceil(expected_frames * min_valid_fraction):
        return "有效检测帧不足；不能把串口/时间戳故障当作曝光效果"
    return ""


def normalized_point(exposure, gain, bounds):
    exposure_min, exposure_max, gain_min, gain_max = bounds
    exposure_axis = (
        2 * (math.log(exposure) - math.log(exposure_min)) /
        (math.log(exposure_max) - math.log(exposure_min)) - 1
        if exposure_min < exposure_max else 0.0
    )
    gain_axis = (
        2 * (gain - gain_min) / (gain_max - gain_min) - 1
        if gain_min < gain_max else 0.0
    )
    return exposure_axis, gain_axis


def surface_features(exposure, gain, bounds):
    u, v = normalized_point(exposure, gain, bounds)
    features = [1.0]
    if bounds[0] < bounds[1]:
        features.extend((u, u * u))
    if bounds[2] < bounds[3]:
        features.extend((v, v * v))
    if bounds[0] < bounds[1] and bounds[2] < bounds[3]:
        features.append(u * v)
    return features


def solve_linear_system(matrix, rhs):
    """小规模正规方程使用带主元的消元；奇异时不输出虚假的曲面。"""
    size = len(rhs)
    augmented = [matrix[index][:] + [rhs[index]] for index in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-10:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        for item in range(column, size + 1):
            augmented[column][item] /= divisor
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            for item in range(column, size + 1):
                augmented[row][item] -= factor * augmented[column][item]
    return [augmented[index][-1] for index in range(size)]


def fit_response_surface(results, bounds):
    """曲面仅用于提出下一组参数，不能把预测极值当成实测最优。"""
    valid = [item for item in results if item.get("eligible")]
    feature_count = len(surface_features(bounds[0], bounds[2], bounds))
    if len(valid) < feature_count:
        return None
    features = [
        surface_features(item["exposure_ms"], item["gain"], bounds)
        for item in valid
    ]
    normal = [
        [sum(row[i] * row[j] for row in features) for j in range(feature_count)]
        for i in range(feature_count)
    ]
    # 轻微正则化仅用于抑制稀疏采样的不稳定系数，不把模型外推到采样边界外。
    for index in range(1, feature_count):
        normal[index][index] += 1e-4
    rhs = [
        sum(row[index] * item["score"] for row, item in zip(features, valid))
        for index in range(feature_count)
    ]
    coefficients = solve_linear_system(normal, rhs)
    if coefficients is None:
        return None
    predictions = [
        sum(coefficient * value for coefficient, value in zip(coefficients, row))
        for row in features
    ]
    rmse = math.sqrt(sum(
        (prediction - item["score"]) ** 2
        for prediction, item in zip(predictions, valid)
    ) / len(valid))
    return {
        "bounds": list(bounds),
        "coefficients": coefficients,
        "training_samples": len(valid),
        "fit_rmse": rmse,
    }


def predict_response(model, exposure, gain):
    values = surface_features(exposure, gain, model["bounds"])
    return sum(value * coefficient for value, coefficient in zip(
        values, model["coefficients"]
    ))


def candidate_settings(bounds, exposure_step, gain_step, grid_size=31):
    exposure_min, exposure_max, gain_min, gain_max = bounds
    exposures = [exposure_min] if exposure_min == exposure_max else [
        exposure_min + (exposure_max - exposure_min) * index / (grid_size - 1)
        for index in range(grid_size)
    ]
    gains = [gain_min] if gain_min == gain_max else [
        gain_min + (gain_max - gain_min) * index / (grid_size - 1)
        for index in range(grid_size)
    ]
    settings = set()
    for exposure in exposures:
        for gain in gains:
            # 步长是请求值的分辨率，不代表相机 SDK 已确认实际接受该精度。
            requested_exposure = min(exposure_max, max(
                exposure_min, round(exposure / exposure_step) * exposure_step
            ))
            requested_gain = min(gain_max, max(
                gain_min, round(gain / gain_step) * gain_step
            ))
            settings.add((round(requested_exposure, 6), round(requested_gain, 6)))
    return sorted(settings)


def propose_next_setting(results, bounds, exposure_step, gain_step, model):
    measured = {
        (round(item["exposure_ms"], 6), round(item["gain"], 6))
        for item in results
    }
    candidates = [
        setting for setting in candidate_settings(bounds, exposure_step, gain_step)
        if setting not in measured
    ]
    if not candidates:
        return None
    measured_points = [
        normalized_point(item["exposure_ms"], item["gain"], bounds)
        for item in results
    ]

    def distance(setting):
        point = normalized_point(*setting, bounds)
        return min(
            math.dist(point, measured_point) for measured_point in measured_points
        ) if measured_points else 1.0

    if model is None:
        return max(candidates, key=lambda setting: (distance(setting), setting))
    # 曲面得分负责利用，距已测点的距离负责少量探索；模型拟合不佳时仍以实测为准。
    return max(candidates, key=lambda setting: (
        predict_response(model, *setting) + 0.02 * distance(setting),
        distance(setting),
    ))


def predicted_optimum(model, bounds, exposure_step, gain_step):
    if model is None:
        return None
    setting = max(
        candidate_settings(bounds, exposure_step, gain_step),
        key=lambda candidate: predict_response(model, *candidate),
    )
    return {
        "exposure_ms": setting[0],
        "gain": setting[1],
        "predicted_score": predict_response(model, *setting),
        "verified": False,
    }


def summarize_confirmation(setting, runs, required_runs):
    valid = [item for item in runs if item.get("eligible")]
    if (len(runs) != required_runs or len(valid) != required_runs or
            any(item["detection_rate"] <= 0 for item in valid)):
        return {
            "exposure_ms": setting[0], "gain": setting[1],
            "verified": False, "runs": runs,
        }
    scores = [item["score"] for item in valid]
    return {
        "exposure_ms": setting[0],
        "gain": setting[1],
        "verified": True,
        "score_mean": sum(scores) / len(scores),
        "score_min": min(scores),
        "detection_rate_mean": sum(item["detection_rate"] for item in valid) / len(valid),
        "longest_miss_streak_max": max(item["longest_miss_streak"] for item in valid),
        "runs": runs,
    }


def run_setting(args, repo, exposure, gain, run_index):
    run_dir = args.artifacts / f"run_{run_index:03d}_e{exposure:g}_g{gain:g}"
    run_dir.mkdir(parents=True, exist_ok=False)
    debug_path = run_dir / "debug.jsonl"
    log_path = run_dir / "process.log"
    command = [
        str(args.binary),
        str(args.config),
        f"--port={args.port}",
        "--show=0",
        f"--exposure-ms={exposure:g}",
        f"--gain={gain:g}",
        f"--max-frames={args.frames}",
        "--quality-metrics=1",
        f"--debug-jsonl={debug_path}",
    ]
    if args.keep_frames:
        command.append(f"--raw-frame-dir={run_dir / 'frames'}")
    started = time.monotonic()
    return_code = None
    timeout = False
    with log_path.open("w", encoding="utf-8") as log_file:
        try:
            completed = subprocess.run(
                command,
                cwd=repo,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                timeout=args.timeout,
                check=False,
            )
            return_code = completed.returncode
        except subprocess.TimeoutExpired:
            timeout = True
    records = load_jsonl(debug_path)
    result = evaluate_records(records, args.warmup_frames)
    result.update({
        "exposure_ms": exposure,
        "gain": gain,
        "run_index": run_index,
        "return_code": return_code,
        "timeout": timeout,
        "debug_jsonl": str(debug_path),
        "process_log": str(log_path),
        "elapsed_s": time.monotonic() - started,
    })
    reason = invalid_reason(result, args.frames - args.warmup_frames, args.min_valid_fraction)
    result["eligible"] = not reason
    result["invalid_reason"] = reason
    return result


def main():
    parser = argparse.ArgumentParser(
        description="只读扫描 HikRobot 曝光/增益；可自适应拟合曲面，不发送云台控制"
    )
    parser.add_argument("--repo", default=".", help="SRM_auto_aim 仓库路径")
    parser.add_argument("--binary", default="./build/standard_srm")
    parser.add_argument("--config", default="configs/real_auto_aim.yaml")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--exposures", default="1,2,3,4,5")
    parser.add_argument("--gains", default="0,6,12,18,24")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--warmup-frames", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--adaptive", action="store_true", help="先网格采样，再用曲面选点")
    parser.add_argument("--adaptive-steps", type=int, default=8, help="额外实测点数")
    parser.add_argument("--confirm-top", type=int, default=2, help="重复确认的候选组数")
    parser.add_argument("--confirm-runs", type=int, default=3, help="每个候选的独立确认次数")
    parser.add_argument("--exposure-step-ms", type=float, default=0.1)
    parser.add_argument("--gain-step", type=float, default=0.5)
    parser.add_argument("--min-valid-fraction", type=float, default=0.7)
    parser.add_argument("--seed", type=int, default=0, help="采样顺序的随机种子")
    parser.add_argument(
        "--artifacts", default="/tmp/real_exposure_gain_tuner",
        help="每组参数的 JSONL 和日志目录",
    )
    parser.add_argument("--output", default="/tmp/real_exposure_gain_report.json")
    parser.add_argument(
        "--keep-frames", action="store_true",
        help="额外保存每组未绘制的原始 JPG，文件量较大",
    )
    args = parser.parse_args()

    if args.frames <= args.warmup_frames or args.frames <= 0:
        parser.error("--frames must be greater than --warmup-frames and positive")
    if args.adaptive_steps < 0 or args.confirm_top < 1 or args.confirm_runs < 1:
        parser.error("adaptive-steps must be >= 0; confirm-top/runs must be positive")
    if (
        not math.isfinite(args.exposure_step_ms) or args.exposure_step_ms <= 0 or
        not math.isfinite(args.gain_step) or args.gain_step <= 0 or
        not math.isfinite(args.min_valid_fraction) or
        not 0 < args.min_valid_fraction <= 1
    ):
        parser.error("steps must be positive and min-valid-fraction must be in (0, 1]")

    repo = Path(args.repo).resolve()
    args.binary = Path(args.binary)
    if not args.binary.is_absolute():
        args.binary = (repo / args.binary).resolve()
    args.config = Path(args.config)
    if not args.config.is_absolute():
        args.config = (repo / args.config).resolve()
    exposures = parse_float_list(args.exposures)
    gains = parse_float_list(args.gains)
    if any(exposure <= 0 for exposure in exposures):
        parser.error("exposure values must be positive; 0 means no override")
    bounds = (min(exposures), max(exposures), min(gains), max(gains))
    if args.adaptive and bounds[0] == bounds[1] and bounds[2] == bounds[3]:
        parser.error("adaptive mode needs an exposure or gain range")
    output = Path(args.output).resolve()
    if args.adaptive and output.exists():
        parser.error(f"report already exists; choose a new --output path: {output}")

    artifacts_root = Path(args.artifacts).resolve()
    artifacts_root.mkdir(parents=True, exist_ok=True)
    # 每次执行独立建目录，失败重跑不能混入旧 JSONL，也不覆盖之前的原始样本。
    args.artifacts = Path(tempfile.mkdtemp(prefix="session_", dir=artifacts_root))

    initial_settings = list(dict.fromkeys(
        (exposure, gain) for exposure in exposures for gain in gains
    ))
    randomizer = random.Random(args.seed)
    if args.adaptive:
        randomizer.shuffle(initial_settings)
    results = []
    run_index = 0

    def measure(exposure, gain, stage):
        nonlocal run_index
        run_index += 1
        print(f"[{run_index}] {stage}: exposure={exposure:g}ms gain={gain:g}", flush=True)
        result = run_setting(args, repo, exposure, gain, run_index)
        result["stage"] = stage
        results.append(result)
        print(
            f"  score={result['score']:.3f} "
            f"detection_rate={result['detection_rate']:.3f} "
            f"confidence={result['mean_confidence']:.3f} "
            f"max_miss={result['longest_miss_streak']} "
            f"valid={result['eligible']} {result['invalid_reason']}",
            flush=True,
        )
        return result

    print(f"开始扫参：{len(initial_settings)} 组初始参数；记录目录 {args.artifacts}")
    for exposure, gain in initial_settings:
        measure(exposure, gain, "initial")

    model = fit_response_surface(results, bounds) if args.adaptive else None
    if args.adaptive:
        for _ in range(args.adaptive_steps):
            next_setting = propose_next_setting(
                results, bounds, args.exposure_step_ms, args.gain_step, model
            )
            if next_setting is None:
                break
            measure(*next_setting, "adaptive")
            model = fit_response_surface(results, bounds)

    prediction = predicted_optimum(
        model, bounds, args.exposure_step_ms, args.gain_step
    ) if args.adaptive else None
    confirmation = []
    if args.adaptive:
        finalists = []
        if prediction:
            finalists.append((prediction["exposure_ms"], prediction["gain"]))
        ranked_measured = sorted(
            (item for item in results if item["eligible"]),
            key=lambda item: (
                item["detection_rate"], -item["longest_miss_streak"],
                item["score"],
            ),
            reverse=True,
        )
        for item in ranked_measured:
            setting = (item["exposure_ms"], item["gain"])
            if setting not in finalists:
                finalists.append(setting)
            if len(finalists) >= args.confirm_top:
                break
        finalists = finalists[:args.confirm_top]
        confirmation_runs = {setting: [] for setting in finalists}
        for _ in range(args.confirm_runs):
            order = finalists[:]
            randomizer.shuffle(order)
            for setting in order:
                confirmation_runs[setting].append(measure(*setting, "confirmation"))
        confirmation = [
            summarize_confirmation(setting, confirmation_runs[setting], args.confirm_runs)
            for setting in finalists
        ]
        verified = [item for item in confirmation if item["verified"]]
        # 4～5 米识别先比较实测检测连续性；得分只用于同等识别率时的取舍。
        recommendation = max(verified, key=lambda item: (
            item["detection_rate_mean"], -item["longest_miss_streak_max"],
            item["score_min"], -item["exposure_ms"], -item["gain"],
        )) if verified else None
    else:
        ranked = sorted(
            (item for item in results if item["eligible"]),
            key=lambda item: item["score"], reverse=True,
        )
        recommendation = ranked[0] if ranked else None

    report = {
        "config": str(args.config),
        "binary": str(args.binary),
        "adaptive": args.adaptive,
        "bounds": bounds,
        "frames": args.frames,
        "warmup_frames": args.warmup_frames,
        "min_valid_fraction": args.min_valid_fraction,
        "seed": args.seed,
        "artifacts_session": str(args.artifacts),
        "results": results,
        "response_surface": model,
        "predicted_optimum": prediction,
        "confirmation": confirmation,
        "recommendation": recommendation,
        "notes": [
            "曲面只在给定范围内建议采样；predicted_optimum 不是实测最优。",
            "推荐值来自独立实测；自适应模式要求每个候选完成指定次数的有效确认。",
            "过曝/欠曝及清晰度取自全图，不是装甲板局部亮度；装甲板连续漏检和 PnP 误差另列诊断。",
            "相机曝光/增益只有请求值，未读取 SDK 实际生效值。",
            "结果只适用于采样时的距离、目标运动和光照；不自动修改 YAML。",
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    if prediction:
        print(
            f"曲面预测：exposure_ms={prediction['exposure_ms']:g}, "
            f"gain={prediction['gain']:g}, predicted_score={prediction['predicted_score']:.3f}"
        )
    if recommendation:
        print(
            f"推荐：exposure_ms={recommendation['exposure_ms']:g}, "
            f"gain={recommendation['gain']:g}, "
            f"score={recommendation.get('score_min', recommendation.get('score')):.3f}"
        )
    else:
        print("没有足够的有效实测确认，暂不输出推荐值；请检查相机、串口和逐组日志。")
    print(f"报告已保存到: {output}")


if __name__ == "__main__":
    main()
