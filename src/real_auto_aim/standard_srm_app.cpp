#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/srm_auto_aim_transport.hpp"
#include "src/real_auto_aim/config.hpp"
#include "src/real_auto_aim/debug_recorder.hpp"
#include "src/real_auto_aim/frame_diagnostics.hpp"
#include "src/real_auto_aim/frame_path.hpp"
#include "src/real_auto_aim/frame_pipeline.hpp"
#include "src/real_auto_aim/frame_stages.hpp"
#include "src/real_auto_aim/serial_feedback_reader.hpp"
#include "src/real_auto_aim/tx_command_controller.hpp"
#include "src/real_auto_aim/standard_srm_app.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/plotter.hpp"

namespace
{
using Clock = real_auto_aim::Clock;
using Json = nlohmann::json;

const std::string kKeys =
    "{help h usage ?||输出命令行参数说明}"
    "{@config-path|configs/real_auto_aim.yaml|只读真机配置路径}"
    "{port|/dev/ttyACM0|USB CDC 串口设备}"
    "{check-config|false|只校验配置，不打开相机和串口}"
    "{show|false|显示实时调试窗口}"
    "{enable-tx|false|启用无开火 yaw/pitch 控制发送，默认关闭}"
    "{tx-use-nominal-speed|false|弹速反馈无效时允许使用配置名义弹速，仅无开火}"
    "{tx-period-ms|20|控制发送周期，单位 ms}"
    "{tx-command-ttl-ms|100|命令有效期，单位 ms}"
    "{tx-follow-temp-lost|true|temp_lost 期间继续发送预测跟随指令，默认开启}"
    "{tx-temp-lost-frames|5|允许外推的最大连续丢失帧数，超出后回到保持角}"
    "{save-frame|imgs/real_srm_frame.jpg|按 s 保存当前显示帧（相对运行目录，默认 imgs/；不带扩展名时按 .jpg 保存）}"
    "{debug-jsonl||保存逐帧真机诊断 JSONL}"
    "{debug-udp|false|通过 UDP 将逐帧诊断 JSON 发到 127.0.0.1:9871}"
    "{exposure-ms|0|临时覆盖 YAML 曝光时间，单位 ms，0 表示沿用 YAML}"
    "{gain|-1|临时覆盖 YAML 增益，负数表示沿用 YAML}"
    "{max-frames|0|采集指定数量的非空图像后退出，0 表示持续运行}"
    "{raw-frame-dir||保存未旋转、未绘制的原始 BGR 图像}"
    "{replay-frame-dir||保存离线算法回放用的无损 PNG 帧序列；需同时启用 debug-jsonl}"
    "{quality-metrics|false|将亮度、裁剪比例和清晰度写入 JSONL}";

}

using namespace real_auto_aim::diagnostics;

int run_standard_srm(int argc, char * argv[])
{
    try {
        cv::CommandLineParser cli(argc, argv, kKeys);
        if (cli.has("help")) {
            cli.printMessage();
            return 0;
        }
        if (!cli.check()) {
            cli.printErrors();
            return 2;
        }
        const auto config_path = cli.get<std::string>(0);
        const auto port = cli.get<std::string>("port");
        if (config_path.empty() || port.empty()) {
            throw std::invalid_argument("Config path and serial port must not be empty");
        }
        const auto real_yaml = YAML::LoadFile(config_path);
        const auto config = real_auto_aim::validate_real_config(real_yaml);
        const double planner_debug_speed =
            real_yaml["planner_debug_bullet_speed_mps"].as<double>();
        const bool enable_tx = cli.get<bool>("enable-tx");
        const bool tx_use_nominal_speed = cli.get<bool>("tx-use-nominal-speed");
        const int tx_period_ms = cli.get<int>("tx-period-ms");
        const int tx_command_ttl_ms = cli.get<int>("tx-command-ttl-ms");
        const bool tx_follow_temp_lost = cli.get<bool>("tx-follow-temp-lost");
        const int tx_temp_lost_max_frames = cli.get<int>("tx-temp-lost-frames");
        const double exposure_override = cli.get<double>("exposure-ms");
        const double gain_override = cli.get<double>("gain");
        const int max_frames = cli.get<int>("max-frames");
        if (!std::isfinite(exposure_override) || exposure_override < 0.0) {
            throw std::invalid_argument("--exposure-ms must be finite and non-negative");
        }
        if (!std::isfinite(gain_override) || gain_override < -1.0) {
            throw std::invalid_argument("--gain must be finite and >= -1");
        }
        if (max_frames < 0) {
            throw std::invalid_argument("--max-frames must be non-negative");
        }
        if (tx_period_ms <= 0 || tx_command_ttl_ms <= 0) {
            throw std::invalid_argument("Invalid TX timing configuration");
        }
        if (tx_temp_lost_max_frames < 0) {
            throw std::invalid_argument("--tx-temp-lost-frames must be non-negative");
        }
        if (cli.get<bool>("check-config")) {
            std::cout << "Real read-only config OK: " << config.image_width << 'x'
                      << config.image_height << ", max_image_age="
                      << config.max_image_age.count() << "ms\n";
            return 0;
        }

        tools::Exiter exiter;
        const bool show = cli.get<bool>("show");
        const bool quality_metrics = cli.get<bool>("quality-metrics");
        real_auto_aim::DebugRecorder recorder(cli.get<std::string>("debug-jsonl"));
        const auto replay_frame_dir = cli.get<std::string>("replay-frame-dir");
        if (!replay_frame_dir.empty() && !recorder.enabled()) {
            throw std::invalid_argument(
                "--replay-frame-dir requires --debug-jsonl so each image has synchronized inputs");
        }
        std::unique_ptr<tools::Plotter> udp_plotter;
        if (cli.get<bool>("debug-udp")) {
            // 复用仿真 PlotJuggler 的 UDP 接收端口，真机帧无需经过 ROS2 转发。
            udp_plotter = std::make_unique<tools::Plotter>();
        }
        io::CameraSettingsOverride camera_overrides;
        if (exposure_override > 0.0) camera_overrides.exposure_ms = exposure_override;
        if (gain_override >= 0.0) camera_overrides.gain = gain_override;
        io::srm_auto_aim::SerialConfig serial_config{port, 20};
        auto transport = std::make_unique<io::srm_auto_aim::SrmAutoAimTransport>(serial_config);
        real_auto_aim::SerialFeedbackReader serial_feedback(
            std::move(transport), std::chrono::milliseconds(100), std::chrono::milliseconds(30),
            enable_tx, std::chrono::milliseconds(tx_period_ms),
            std::chrono::milliseconds(tx_command_ttl_ms), config.max_image_age,
            recorder.enabled());
        if (enable_tx) {
            tools::logger()->warn(
                "[standard_srm/TX] enabled: yaw/pitch only, fire_flag is forced to 0; "
                "lower-controller timeout is not modified, nominal_speed={}",
                tx_use_nominal_speed);
        }

        const auto raw_frame_dir = cli.get<std::string>("raw-frame-dir");
        const auto save_frame_path = cli.get<std::string>("save-frame");
        const auto capture_stage = std::make_shared<real_auto_aim::FrameCaptureStage>(
            real_auto_aim::FrameCaptureOptions{
                config_path, camera_overrides, config.image_width, config.image_height,
                config.max_image_age, max_frames, enable_tx, quality_metrics,
                !replay_frame_dir.empty()});
        const auto algorithm_stage = std::make_shared<real_auto_aim::FrameAlgorithmStage>(
            real_auto_aim::FrameAlgorithmOptions{
                config_path,
                real_auto_aim::TxCommandOptions{
                    enable_tx, tx_use_nominal_speed, tx_period_ms, tx_command_ttl_ms,
                    tx_follow_temp_lost, tx_temp_lost_max_frames},
                config.max_image_age, planner_debug_speed, !replay_frame_dir.empty()},
            serial_feedback);
        // 只读采集可等待较慢的磁盘；启用 TX 时以命令 TTL 和图像时效较小者为上限。
        const auto output_timeout = enable_tx
            ? std::min(std::chrono::milliseconds(tx_command_ttl_ms), config.max_image_age)
            : std::chrono::milliseconds(5000);
        real_auto_aim::FramePipeline pipeline(
            [capture_stage] { return capture_stage->next(); },
            [algorithm_stage](real_auto_aim::FrameInput && input,
                              const std::atomic<bool> & stopping) {
                return algorithm_stage->process(std::move(input), stopping);
            },
            [&serial_feedback] { serial_feedback.inhibit_commands(); },
            output_timeout, 1, 2,
            enable_tx ? std::optional{config.max_image_age} : std::nullopt);

        std::uint64_t saved_frame_count = 0;
        auto next_report = Clock::now();
        bool window_exit = false;
        cv::Mat last_display;
        const auto save_raw_frame = [&raw_frame_dir](const real_auto_aim::FrameOutput & output) {
            if (raw_frame_dir.empty() || output.image.empty()) return;
            try {
                const std::filesystem::path output_dir(raw_frame_dir);
                std::filesystem::create_directories(output_dir);
                std::ostringstream filename;
                filename << "frame_" << std::setfill('0') << std::setw(6)
                         << output.captured_frame << ".jpg";
                const auto output_path = output_dir / filename.str();
                if (!cv::imwrite(
                      output_path.string(), output.image,
                      {cv::IMWRITE_JPEG_QUALITY, 95})) {
                    tools::logger()->warn("Failed to save raw frame to {}", output_path.string());
                }
            } catch (const std::exception & error) {
                tools::logger()->warn("Failed to save raw frame: {}", error.what());
            }
        };
        const auto save_replay_frame = [&replay_frame_dir](
                                           const real_auto_aim::FrameOutput & output) {
            if (replay_frame_dir.empty()) return;
            const std::filesystem::path output_dir(replay_frame_dir);
            std::filesystem::create_directories(output_dir);
            const auto output_path =
                output_dir / output.sample.at("replay_frame_file").get<std::string>();
            // 同一结果消费者先保存 PNG 再记录 JSONL，避免排队丢帧留下无对应日志的图片。
            if (!cv::imwrite(output_path.string(), output.image)) {
                throw std::runtime_error(
                    "Failed to save replay frame: " + output_path.string());
            }
        };
        const auto save_display_frame = [&save_frame_path, &saved_frame_count](
                                            const cv::Mat & image) {
            if (save_frame_path.empty() || image.empty()) return;
            try {
                const std::filesystem::path base_path(save_frame_path);
                const auto parent_path = base_path.parent_path();
                if (!parent_path.empty()) std::filesystem::create_directories(parent_path);
                const auto output_path =
                    real_auto_aim::numbered_frame_path(base_path, ++saved_frame_count);
                if (cv::imwrite(output_path.string(), image)) {
                    tools::logger()->info("Saved display frame to {}", output_path.string());
                } else {
                    tools::logger()->warn(
                        "Failed to save display frame to {}", output_path.string());
                }
            } catch (const std::exception & error) {
                tools::logger()->warn("Failed to save display frame: {}", error.what());
            }
        };
        const auto handle_window_key = [&window_exit, &last_display,
                                        &save_display_frame]() {
            const int key = cv::waitKey(1) & 0xff;
            if (key == 's' || key == 'S') {
                save_display_frame(last_display);
            } else if (key == 27 || key == 'q' || key == 'Q') {
                window_exit = true;
            }
        };
        const auto log_skip = [&next_report](const std::string & reason) {
            const auto now = Clock::now();
            if (now >= next_report) {
                tools::logger()->warn("[standard_srm] {}", reason);
                next_report = now + std::chrono::seconds(1);
            }
        };
        const auto record_sample = [&recorder, &serial_feedback,
                                    &udp_plotter](Json & sample) {
            // tx_command_sent 仅表示进入邮箱；累计量来自串口线程成功写入。
            const auto tx_stats = serial_feedback.tx_stats();
            sample["tx_wire_target_frames"] = tx_stats.target_frames;
            sample["tx_wire_zero_frames"] = tx_stats.zero_frames;
            if (recorder.enabled()) {
                // 串口线程只存有界事件；JSON 构造和落盘均在结果消费者线程完成。
                Json recorded = sample;
                const auto batch = serial_feedback.take_wire_events();
                recorded["wire_trace_dropped_total"] = batch.dropped_total;
                recorded["wire_trace_events"] = Json::array();
                for (const auto & event : batch.events) {
                    const char * kind =
                        event.kind == real_auto_aim::WireEventKind::TxTarget ? "tx_target" :
                        event.kind == real_auto_aim::WireEventKind::TxZero ? "tx_zero" :
                        "rx_feedback";
                    Json wire_record = {
                        {"kind", kind},
                        {"sequence", event.sequence},
                        {"started_steady_ns", event.started_ns},
                        {"completed_steady_ns", event.completed_ns},
                        {"yaw_deg", event.yaw_deg},
                        {"pitch_deg", event.pitch_deg}
                    };
                    if (event.kind == real_auto_aim::WireEventKind::RxFeedback) {
                        wire_record["roll_deg"] = event.roll_deg;
                        wire_record["mode_raw"] = event.mode;
                        wire_record["color_raw"] = event.color_raw;
                        wire_record["bullet_speed_mps"] = event.bullet_speed_mps;
                        wire_record["batch_index"] = event.batch_index;
                        wire_record["batch_frames"] = event.batch_frames;
                    }
                    recorded["wire_trace_events"].push_back(std::move(wire_record));
                }
                recorder.write(recorded);
            }
            if (udp_plotter) udp_plotter->plot(sample);
        };
        const auto consume_output = [&](real_auto_aim::FrameOutput output) {
            save_raw_frame(output);
            save_replay_frame(output);
            record_sample(output.sample);
            if (output.sample.value("event", std::string{}) == "skip") {
                log_skip(output.sample.value("skip_reason", std::string("frame skipped")));
                if (show && !output.image.empty()) {
                    last_display = output.image.clone();
                    cv::rotate(last_display, last_display, cv::ROTATE_180);
                    draw_text(last_display, output.display_reason, 0, cv::Scalar(0, 255, 255));
                    cv::imshow("standard_srm", last_display);
                    handle_window_key();
                }
                return;
            }
            if (!output.result || !output.feedback || !output.image_timestamp) return;
            const auto & result = *output.result;
            const auto & feedback = *output.feedback;
            if (show) {
                last_display = output.image.clone();
                for (const auto & armor : result.detections.armors) {
                    draw_polygon(
                        last_display, armor.points, cv::Scalar(0, 255, 255),
                        "det " + std::to_string(armor.confidence));
                }
                draw_polygon(last_display, output.current_polygon, cv::Scalar(255, 0, 0), "current");
                draw_polygon(last_display, output.future_polygon, cv::Scalar(255, 0, 255), "future");
                cv::rotate(last_display, last_display, cv::ROTATE_180);
                draw_text(
                    last_display,
                    "frame=" + std::to_string(output.timing.frame_id) +
                      " detected=" + std::to_string(result.detections.count) +
                      " targets=" + std::to_string(result.targets.size()),
                    0, cv::Scalar(0, 255, 255));
                draw_text(
                    last_display,
                    "state=" + result.tracker_state +
                      " age=" + std::to_string(output.sample["image_age_ms"].get<double>()) +
                      "ms bracket=" + std::to_string(feedback.bracket_ms) + "ms",
                    1, cv::Scalar(0, 255, 255));
                draw_text(
                    last_display,
                    "yaw=" + std::to_string(result.diagnostic.yaw * 180.0 / CV_PI) +
                      " pitch=" + std::to_string(result.diagnostic.pitch * 180.0 / CV_PI) +
                      " speed=" + std::to_string(result.bullet_speed),
                    2, cv::Scalar(0, 255, 255));
                cv::imshow("standard_srm", last_display);
                handle_window_key();
            }
            if (result.finished_at >= next_report) {
                const double image_age_ms = std::chrono::duration<double, std::milli>(
                    result.finished_at - *output.image_timestamp).count();
                const double mapping_delay_ms = output.timing.mapped_capture_at
                    ? std::chrono::duration<double, std::milli>(
                        output.timing.host_received_at - *output.timing.mapped_capture_at).count()
                    : 0.0;
                const bool tx_sent = output.sample.value("tx_command_sent", false);
                const double tx_yaw = output.sample["tx_command_yaw_deg"].is_number()
                    ? output.sample["tx_command_yaw_deg"].get<double>() : 0.0;
                const double tx_pitch = output.sample["tx_command_pitch_deg"].is_number()
                    ? output.sample["tx_command_pitch_deg"].get<double>() : 0.0;
                tools::logger()->info(
                    "[standard_srm] frame={} ticks={} tick_hz={} timestamp_source={} "
                    "image_age={:.1f}ms mapping_delay={:.1f}ms bracket={:.1f}ms "
                    "detected={} targets={} tracker={} speed={:.2f}m/s mode={} color={} "
                    "aim_control={} diagnostic_yaw={:.2f}deg diagnostic_pitch={:.2f}deg "
                    "tx_enabled={} tx_sent={} tx_mode={} tx_yaw={:.2f}deg tx_pitch={:.2f}deg",
                    output.timing.frame_id, output.timing.device_ticks,
                    output.timing.device_timestamp_hz,
                    io::frame_timestamp_source_name(output.timing.timestamp_source),
                    image_age_ms, mapping_delay_ms, feedback.bracket_ms,
                    result.detections.count, result.targets.size(), result.tracker_state,
                    result.bullet_speed, feedback.feedback.mode, feedback.feedback.color,
                    result.diagnostic.control,
                    result.diagnostic.yaw * 180.0 / CV_PI,
                    result.diagnostic.pitch * 180.0 / CV_PI,
                    enable_tx, tx_sent,
                    output.sample.value("tx_command_mode", std::string("none")),
                    tx_yaw, tx_pitch);
                next_report = result.finished_at + std::chrono::seconds(1);
            }
        };

        pipeline.start();
        try {
            while (!pipeline.finished()) {
                real_auto_aim::FrameOutput output;
                if (pipeline.try_pop_output(output)) {
                    consume_output(std::move(output));
                    continue;
                }
                pipeline.rethrow_if_failed();
                if (exiter.exit() || window_exit) pipeline.request_stop();
                if (show) handle_window_key();
                pipeline.wait_for_output(std::chrono::milliseconds(10));
            }
            pipeline.join();
            pipeline.rethrow_if_failed();
        } catch (...) {
            pipeline.request_stop();
            pipeline.join();
            throw;
        }
        if (show) cv::destroyWindow("standard_srm");
        serial_feedback.stop();
        if (max_frames > 0 && !window_exit && !exiter.exit()) {
            tools::logger()->info(
                "[standard_srm] max-frames reached: {} frames tx_enabled={}",
                capture_stage->captured_frames(), enable_tx);
        }
        return 0;
    } catch (const std::exception & error) {
        std::cerr << "standard_srm failed: " << error.what() << '\n';
        return 1;
    }
}
