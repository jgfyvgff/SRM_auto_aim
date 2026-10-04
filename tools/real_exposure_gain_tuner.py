#!/usr/bin/env python3
"""自动遍历真机相机曝光/增益，并依据检测和图像质量给出推荐值。"""

import argparse
import json
import math
import subprocess
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
        "score": score,
        "warning": (
            "没有 frame 记录，检测率无法评价；请先确认串口反馈和相机链路"
            if not processed
            else ""
        ),
    }


def run_setting(args, repo, exposure, gain, run_index):
    run_dir = args.artifacts / f"run_{run_index:03d}_e{exposure:g}_g{gain:g}"
    run_dir.mkdir(parents=True, exist_ok=True)
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
    return result


def main():
    parser = argparse.ArgumentParser(
        description="自动扫 HikRobot 曝光/增益；只读运行，不发送云台控制"
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

    repo = Path(args.repo).resolve()
    args.binary = Path(args.binary)
    if not args.binary.is_absolute():
        args.binary = (repo / args.binary).resolve()
    args.config = Path(args.config)
    if not args.config.is_absolute():
        args.config = (repo / args.config).resolve()
    args.artifacts = Path(args.artifacts).resolve()
    args.artifacts.mkdir(parents=True, exist_ok=True)

    exposures = parse_float_list(args.exposures)
    gains = parse_float_list(args.gains)
    total = len(exposures) * len(gains)
    results = []
    run_index = 0

    print(f"开始扫参：{len(exposures)} 个曝光 × {len(gains)} 个增益，共 {total} 组")
    for exposure in exposures:
        for gain in gains:
            run_index += 1
            print(f"[{run_index}/{total}] exposure={exposure:g}ms gain={gain:g}")
            result = run_setting(args, repo, exposure, gain, run_index)
            results.append(result)
            print(
                f"  score={result['score']:.3f} "
                f"detection_rate={result['detection_rate']:.3f} "
                f"confidence={result['mean_confidence']:.3f} "
                f"clip={result['clipped_ratio']:.4f}"
            )

    ranked = sorted(results, key=lambda item: item["score"], reverse=True)
    has_processed_frames = any(item["processed_frames"] > 0 for item in results)
    recommendation = ranked[0] if ranked and has_processed_frames else None
    report = {
        "config": str(args.config),
        "binary": str(args.binary),
        "frames": args.frames,
        "warmup_frames": args.warmup_frames,
        "results": results,
        "recommendation": recommendation,
        "notes": [
            "评分优先检测率，其次置信度，再考虑过曝/欠曝和清晰度。",
            "如果所有结果都没有 frame 记录，推荐值不具备检测意义。",
            "推荐值只写入报告，不自动修改 YAML，需人工确认后应用。",
        ],
    }
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    if recommendation:
        print(
            f"推荐：exposure_ms={recommendation['exposure_ms']:g}, "
            f"gain={recommendation['gain']:g}, score={recommendation['score']:.3f}"
        )
    else:
        print("没有有效 frame 检测记录，暂不输出推荐值；请先检查相机、串口反馈和检测链路。")
    print(f"报告已保存到: {output}")


if __name__ == "__main__":
    main()
