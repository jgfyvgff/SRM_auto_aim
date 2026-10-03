#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/srm_auto_aim_transport.hpp"
#include "src/real_auto_aim/config.hpp"
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

const std::string kKeys =
    "{help h usage ?||输出命令行参数说明}"
    "{@config-path|configs/real_auto_aim.yaml|只读真机配置路径}"
    "{port|/dev/ttyACM0|USB CDC 串口设备}"
    "{check-config|false|只校验配置，不打开相机和串口}";
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
        const auto log_skip = [&next_report](const char * reason) {
            const auto now = Clock::now();
            if (now >= next_report) {
                tools::logger()->warn("[standard_srm/READ_ONLY] {} (NO_TX)", reason);
                next_report = now + std::chrono::seconds(1);
            }
        };
        while (!exiter.exit()) {
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
            // 没有设备时标或无法确认计数频率时禁止把收帧时刻冒充曝光时刻。
            if (!timing.mapped_capture_at) {
                log_skip(timing.device_timestamp_hz == 0
                             ? "camera timestamp frequency unavailable; no host fallback"
                             : timing.device_ticks == 0
                                 ? "camera frame has no device timestamp; no host fallback"
                                 : "camera timestamp mapping warming up or reset");
                continue;
            }
            const auto image_time = *timing.mapped_capture_at;
            const auto received_at = Clock::now();
            if (image_time > received_at ||
                received_at - image_time > config.max_image_age) {
                log_skip("camera frame timestamp is invalid or stale");
                continue;
            }
            const auto feedback = serial_feedback.sample_at(image_time);
            if (!feedback) {
                log_skip("no fresh feedback on both sides of image time");
                continue;
            }

            solver.set_gimbal_to_world(feedback->gimbal_to_world);
            auto armors = detector.detect(image);
            const auto detected = armors.size();
            auto targets = tracker.track(armors, image_time);
            const double bullet_speed = feedback->feedback.bullet_speed_mps;
            if (!std::isfinite(bullet_speed) || bullet_speed < 14.0) {
                log_skip("invalid bullet speed; Aimer default-speed fallback is disabled");
                continue;
            }
            const auto diagnostic = aimer.aim(targets, image_time, bullet_speed);
            const auto now = Clock::now();
            if (now >= next_report) {
                const double image_age_ms =
                    std::chrono::duration<double, std::milli>(now - image_time).count();
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
                next_report = now + std::chrono::seconds(1);
            }
            // 故意不创建 Shooter，也不调用 SrmAutoAimTransport::send()。
        }
        return 0;
    } catch (const std::exception & error) {
        std::cerr << "standard_srm failed: " << error.what() << '\n';
        return 1;
    }
}
