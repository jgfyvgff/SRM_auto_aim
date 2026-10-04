#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/srm_auto_aim_transport.hpp"
#include "src/real_auto_aim/config.hpp"
#include "src/real_auto_aim/debug_recorder.hpp"
#include "src/real_auto_aim/serial_feedback_reader.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

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
    "{debug-jsonl||保存逐帧真机诊断 JSONL}";

double duration_ms(Clock::time_point begin, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

void draw_polygon(
    cv::Mat & image, const std::vector<cv::Point2f> & points, const cv::Scalar & color,
    const std::string & label)
{
    if (points.size() < 3) return;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto & current = points[index];
        const auto & next = points[(index + 1) % points.size()];
        cv::line(image, current, next, color, 3, cv::LINE_AA);
    }
    for (const auto & point : points) {
        cv::circle(image, point, 5, color, -1, cv::LINE_AA);
    }
    const auto rect = cv::boundingRect(points);
    cv::putText(
        image, label, cv::Point(rect.x, std::max(20, rect.y - 8)), cv::FONT_HERSHEY_SIMPLEX,
        0.65, color, 2, cv::LINE_AA);
}

void draw_text(cv::Mat & image, const std::string & text, int line, const cv::Scalar & color)
{
    cv::putText(
        image, text, cv::Point(15, 30 + line * 27), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2,
        cv::LINE_AA);
}

Json timing_json(const io::FrameTiming & timing, Clock::time_point now)
{
    Json sample{
        {"frame_id", timing.frame_id},
        {"device_ticks", timing.device_ticks},
        {"tick_hz", timing.device_timestamp_hz},
    };
    if (timing.mapped_capture_at) {
        sample["mapped_age_ms"] = duration_ms(*timing.mapped_capture_at, now);
        sample["mapping_delay_ms"] =
            duration_ms(*timing.mapped_capture_at, timing.host_received_at);
    }
    return sample;
}

void append_target_json(Json & sample, const std::list<auto_aim::Target> & targets)
{
    if (targets.empty()) {
        sample["targets"] = 0;
        return;
    }

    const auto & target = targets.front();
    const auto state = target.ekf_x();
    sample["targets"] = targets.size();
    sample["current_armor_id"] = target.last_id;
    if (state.size() >= 11) {
        sample["center_x"] = state[0];
        sample["vx"] = state[1];
        sample["center_y"] = state[2];
        sample["vy"] = state[3];
        sample["center_z"] = state[4];
        sample["vz"] = state[5];
        sample["center_yaw"] = state[6];
        sample["angular_velocity"] = state[7];
        sample["radius"] = state[8];
        sample["radius_delta"] = state[9];
        sample["height_delta"] = state[10];
        sample["center_speed"] = std::hypot(state[1], state[3]);
    }

    const auto armors = target.armor_xyza_list();
    if (target.last_id >= 0 && static_cast<std::size_t>(target.last_id) < armors.size()) {
        const auto & current = armors[static_cast<std::size_t>(target.last_id)];
        sample["current_x"] = current[0];
        sample["current_y"] = current[1];
        sample["current_z"] = current[2];
        sample["current_yaw"] = current[3];
    }
}

void append_aim_json(Json & sample, const auto_aim::Aimer & aimer, const io::Command & command)
{
    sample["aim_control"] = command.control ? 1 : 0;
    sample["aim_shoot"] = command.shoot ? 1 : 0;
    sample["diagnostic_yaw"] = command.yaw;
    sample["diagnostic_pitch"] = command.pitch;
    sample["diagnostic_yaw_deg"] = command.yaw * 180.0 / CV_PI;
    sample["diagnostic_pitch_deg"] = command.pitch * 180.0 / CV_PI;
    sample["prediction_dt"] = aimer.debug_prediction_dt;
    sample["delay_time"] = aimer.debug_delay_time;
    sample["base_prediction_dt"] = aimer.debug_base_prediction_dt;
    sample["fly_time"] = aimer.debug_fly_time;
    sample["aim_armor_id"] = aimer.debug_aim_point.armor_id;
    if (!aimer.debug_aim_point.valid) return;
    const auto & future = aimer.debug_aim_point.xyza;
    sample["future_x"] = future[0];
    sample["future_y"] = future[1];
    sample["future_z"] = future[2];
    sample["future_yaw"] = future[3];
}
}

int main(int argc, char * argv[])
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
        const auto config = real_auto_aim::validate_real_config(YAML::LoadFile(config_path));
        if (cli.get<bool>("check-config")) {
            std::cout << "Real read-only config OK: " << config.image_width << 'x'
                      << config.image_height << ", max_image_age="
                      << config.max_image_age.count() << "ms\n";
            return 0;
        }

        tools::Exiter exiter;
        const bool show = cli.get<bool>("show");
        real_auto_aim::DebugRecorder recorder(cli.get<std::string>("debug-jsonl"));
        io::Camera camera(config_path);
        io::srm_auto_aim::SerialConfig serial_config{port, 20};
        auto transport = std::make_unique<io::srm_auto_aim::SrmAutoAimTransport>(serial_config);
        real_auto_aim::SerialFeedbackReader serial_feedback(std::move(transport));
        auto_aim::YOLO detector(config_path, false);
        auto_aim::Solver solver(config_path);
        auto_aim::Tracker tracker(config_path, solver);
        auto_aim::Aimer aimer(config_path);

        // 本入口只有接收通道。下位机 mode/color 尚未确认映射，只按原始整数记录。
        // 缺姿态时不更新世界系 Tracker；弹速无效时不调用带旧默认值回退的 Aimer。
        auto next_report = Clock::now();
        bool window_exit = false;
        const auto log_skip = [&next_report](const char * reason) {
            const auto now = Clock::now();
            if (now >= next_report) {
                tools::logger()->warn("[standard_srm/READ_ONLY] {} (NO_TX)", reason);
                next_report = now + std::chrono::seconds(1);
            }
        };
        const auto show_skip_frame = [&window_exit, show](
                                         const cv::Mat & image, const std::string & reason) {
            if (!show || image.empty()) return;
            cv::Mat visualization = image.clone();
            draw_text(visualization, reason, 0, cv::Scalar(0, 255, 255));
            cv::imshow("standard_srm", visualization);
            const int key = cv::waitKey(1) & 0xff;
            if (key == 27 || key == 'q' || key == 'Q') window_exit = true;
        };
        while (!exiter.exit() && !window_exit) {
            cv::Mat image;
            io::FrameTiming timing;
            camera.read_timed(image, timing);
            if (image.empty()) {
                log_skip("empty camera frame");
                continue;
            }
            if (image.cols != config.image_width || image.rows != config.image_height) {
                throw std::runtime_error("Camera frame size differs from calibration image_size");
            }
            const auto frame_read_at = Clock::now();
            auto sample = timing_json(timing, frame_read_at);
            sample["image_width"] = image.cols;
            sample["image_height"] = image.rows;
            // 没有设备时标或无法确认计数频率时禁止把收帧时刻冒充曝光时刻。
            if (!timing.mapped_capture_at) {
                const char * reason = timing.device_timestamp_hz == 0
                    ? "camera timestamp frequency unavailable; no host fallback"
                    : timing.device_ticks == 0
                        ? "camera frame has no device timestamp; no host fallback"
                        : "camera timestamp mapping warming up or reset";
                sample["event"] = "skip";
                sample["skip_reason"] = reason;
                recorder.write(sample);
                show_skip_frame(image, reason);
                log_skip(reason);
                continue;
            }
            const auto image_time = *timing.mapped_capture_at;
            const auto received_at = Clock::now();
            if (image_time > received_at ||
                received_at - image_time > config.max_image_age) {
                sample["event"] = "skip";
                sample["skip_reason"] = "camera frame timestamp is invalid or stale";
                recorder.write(sample);
                show_skip_frame(image, "stale camera timestamp");
                log_skip("camera frame timestamp is invalid or stale");
                continue;
            }
            sample["mapped_age_ms"] = duration_ms(image_time, received_at);
            const auto feedback = serial_feedback.sample_at(image_time);
            if (!feedback) {
                sample["event"] = "skip";
                sample["skip_reason"] = "no fresh feedback on both sides of image time";
                recorder.write(sample);
                show_skip_frame(image, "waiting for serial feedback");
                log_skip("no fresh feedback on both sides of image time");
                continue;
            }

            solver.set_gimbal_to_world(feedback->gimbal_to_world);
            const auto detector_start = Clock::now();
            auto armors = detector.detect(image);
            const auto detector_end = Clock::now();
            const auto detected = armors.size();
            auto targets = tracker.track(armors, image_time);
            const auto tracker_end = Clock::now();
            const double bullet_speed = feedback->feedback.bullet_speed_mps;
            if (!std::isfinite(bullet_speed) || bullet_speed < 14.0) {
                sample["event"] = "skip";
                sample["skip_reason"] = "invalid bullet speed; Aimer default-speed fallback is disabled";
                sample["detected"] = detected;
                sample["tracker_state"] = tracker.state();
                sample["tracker_generation"] = tracker.target_generation();
                sample["detector_ms"] = duration_ms(detector_start, detector_end);
                sample["tracker_ms"] = duration_ms(detector_end, tracker_end);
                append_target_json(sample, targets);
                recorder.write(sample);
                show_skip_frame(image, "invalid bullet speed");
                log_skip("invalid bullet speed; Aimer default-speed fallback is disabled");
                continue;
            }
            const auto aimer_start = Clock::now();
            const auto diagnostic = aimer.aim(targets, image_time, bullet_speed);
            const auto finished_at = Clock::now();

            sample["event"] = "frame";
            sample["detected"] = detected;
            sample["tracker_state"] = tracker.state();
            sample["tracker_generation"] = tracker.target_generation();
            sample["temp_lost_count"] = tracker.temp_lost_count();
            sample["bracket_ms"] = feedback->bracket_ms;
            sample["bullet_speed_mps"] = bullet_speed;
            sample["mode"] = feedback->feedback.mode;
            sample["color"] = feedback->feedback.color;
            sample["detector_ms"] = duration_ms(detector_start, detector_end);
            sample["tracker_ms"] = duration_ms(detector_end, tracker_end);
            sample["aimer_ms"] = duration_ms(aimer_start, finished_at);
            sample["capture_to_detector_ms"] = duration_ms(image_time, detector_start);
            sample["capture_to_aimer_ms"] = duration_ms(image_time, finished_at);
            append_target_json(sample, targets);
            append_aim_json(sample, aimer, diagnostic);
            recorder.write(sample);

            if (show) {
                cv::Mat visualization = image.clone();
                for (const auto & armor : armors) {
                    draw_polygon(
                        visualization, armor.points, cv::Scalar(0, 255, 255),
                        "det " + std::to_string(armor.confidence));
                }
                if (!targets.empty()) {
                    const auto & target = targets.front();
                    const auto xyza = target.armor_xyza_list();
                    if (target.last_id >= 0 &&
                        static_cast<std::size_t>(target.last_id) < xyza.size()) {
                        draw_polygon(
                            visualization,
                            solver.reproject_armor(
                                xyza[static_cast<std::size_t>(target.last_id)].head(3),
                                xyza[static_cast<std::size_t>(target.last_id)][3],
                                target.armor_type, target.name),
                            cv::Scalar(255, 0, 0), "current");
                    }
                    if (aimer.debug_aim_point.valid) {
                        const auto & future = aimer.debug_aim_point.xyza;
                        draw_polygon(
                            visualization,
                            solver.reproject_armor(
                                future.head(3), future[3], target.armor_type, target.name),
                            cv::Scalar(255, 0, 255), "future");
                    }
                }
                draw_text(
                    visualization,
                    "frame=" + std::to_string(timing.frame_id) +
                      " detected=" + std::to_string(detected) +
                      " targets=" + std::to_string(targets.size()),
                    0, cv::Scalar(0, 255, 255));
                draw_text(
                    visualization,
                    "state=" + tracker.state() +
                      " age=" + std::to_string(sample["mapped_age_ms"].get<double>()) +
                      "ms bracket=" + std::to_string(feedback->bracket_ms) + "ms",
                    1, cv::Scalar(0, 255, 255));
                draw_text(
                    visualization,
                    "yaw=" + std::to_string(diagnostic.yaw * 180.0 / CV_PI) +
                      " pitch=" + std::to_string(diagnostic.pitch * 180.0 / CV_PI) +
                      " speed=" + std::to_string(bullet_speed),
                    2, cv::Scalar(0, 255, 255));
                cv::imshow("standard_srm", visualization);
                const int key = cv::waitKey(1) & 0xff;
                if (key == 27 || key == 'q' || key == 'Q') window_exit = true;
            }

            if (finished_at >= next_report) {
                const double image_age_ms =
                    std::chrono::duration<double, std::milli>(finished_at - image_time).count();
                const double mapping_delay_ms = std::chrono::duration<double, std::milli>(
                    timing.host_received_at - image_time).count();
                tools::logger()->info(
                    "[standard_srm/READ_ONLY] frame={} ticks={} tick_hz={} "
                    "mapped_age={:.1f}ms mapping_delay={:.1f}ms bracket={:.1f}ms "
                    "detected={} targets={} tracker={} speed={:.2f}m/s mode={} color={} "
                    "aim_control={} diagnostic_yaw={:.2f}deg diagnostic_pitch={:.2f}deg NO_TX",
                    timing.frame_id, timing.device_ticks, timing.device_timestamp_hz,
                    image_age_ms, mapping_delay_ms, feedback->bracket_ms, detected, targets.size(),
                    tracker.state(), bullet_speed, feedback->feedback.mode,
                    feedback->feedback.color, diagnostic.control,
                    diagnostic.yaw * 180.0 / CV_PI,
                    diagnostic.pitch * 180.0 / CV_PI);
                next_report = finished_at + std::chrono::seconds(1);
            }
            // 故意不创建 Shooter，也不调用 SrmAutoAimTransport::send()。
        }
        if (show) cv::destroyWindow("standard_srm");
        return 0;
    } catch (const std::exception & error) {
        std::cerr << "standard_srm failed: " << error.what() << '\n';
        return 1;
    }
}
