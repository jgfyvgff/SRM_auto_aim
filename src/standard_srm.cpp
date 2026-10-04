#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <list>
#include <memory>
#include <limits>
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
    "{save-frame|/tmp/real_srm_frame.jpg|按 s 保存当前显示帧}"
    "{debug-jsonl||保存逐帧真机诊断 JSONL}"
    "{exposure-ms|0|临时覆盖 YAML 曝光时间，单位 ms，0 表示沿用 YAML}"
    "{gain|-1|临时覆盖 YAML 增益，负数表示沿用 YAML}"
    "{max-frames|0|采集指定数量的非空图像后退出，0 表示持续运行}"
    "{raw-frame-dir||保存未旋转、未绘制的原始 BGR 图像}"
    "{quality-metrics|false|将亮度、裁剪比例和清晰度写入 JSONL}";

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
        {"timestamp_source", io::frame_timestamp_source_name(timing.timestamp_source)},
    };
    if (timing.mapped_capture_at) {
        sample["mapped_age_ms"] = duration_ms(*timing.mapped_capture_at, now);
        sample["mapping_delay_ms"] =
            duration_ms(*timing.mapped_capture_at, timing.host_received_at);
    }
    return sample;
}

double mean_reprojection_error(
    const std::vector<cv::Point2f> & observed,
    const std::vector<cv::Point2f> & projected)
{
    if (observed.size() != 4 || projected.size() != observed.size()) return -1.0;

    double error_sum = 0.0;
    for (std::size_t index = 0; index < observed.size(); ++index) {
        if (
            !std::isfinite(observed[index].x) || !std::isfinite(observed[index].y) ||
            !std::isfinite(projected[index].x) || !std::isfinite(projected[index].y)) {
            return -1.0;
        }
        error_sum += cv::norm(observed[index] - projected[index]);
    }
    return error_sum / static_cast<double>(observed.size());
}

bool quad_center(const std::vector<cv::Point2f> & points, cv::Point2f & center)
{
    if (points.size() != 4) return false;
    center = cv::Point2f(0.0F, 0.0F);
    for (const auto & point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
        center += point;
    }
    center *= 0.25F;
    return true;
}

double best_order_reprojection_error(
    const std::vector<cv::Point2f> & observed,
    const std::vector<cv::Point2f> & projected)
{
    cv::Point2f observed_center;
    cv::Point2f projected_center;
    if (
        !quad_center(observed, observed_center) ||
        !quad_center(projected, projected_center)) {
        return -1.0;
    }

    double best_error = std::numeric_limits<double>::infinity();
    // 仅改变四角点的比较顺序，不改变 PnP、Tracker 或实际装甲板姿态。
    for (int direction = -1; direction <= 1; direction += 2) {
        for (int shift = 0; shift < 4; ++shift) {
            double error_sum = 0.0;
            for (int index = 0; index < 4; ++index) {
                const int projected_index = (shift + direction * index + 4) % 4;
                error_sum += cv::norm(observed[index] - projected[projected_index]);
            }
            best_error = std::min(best_error, error_sum / 4.0);
        }
    }
    return best_error;
}

void append_reprojection_json(
    Json & sample,
    const auto_aim::Solver & solver,
    const std::list<auto_aim::Armor> & detected_armors,
    const std::list<auto_aim::Target> & targets)
{
    Json pnp_errors = Json::array();
    std::size_t detection_index = 0;
    for (const auto & armor : detected_armors) {
        const auto projected = solver.reproject_pnp(armor);
        const double error = mean_reprojection_error(armor.points, projected);
        Json item{
            {"detection_index", detection_index},
            {"pnp_reprojection_error_px", error >= 0.0 ? Json(error) : Json(nullptr)},
        };
        pnp_errors.push_back(item);
        ++detection_index;
    }
    sample["pnp_reprojection_errors_px"] = pnp_errors;

    // 当前模型误差取与 EKF 当前装甲板投影最近的检测框，避免双装甲板时比较错对象。
    // 该值不是 Tracker 关联代价，而是用于隔离手眼、云台姿态和装甲板模型误差。
    sample["current_model_reprojection_error_px"] = nullptr;
    sample["current_model_reprojection_detection_index"] = nullptr;
    sample["current_model_center_error_px"] = nullptr;
    sample["current_model_center_dx_px"] = nullptr;
    sample["current_model_center_dy_px"] = nullptr;
    sample["current_model_center_detection_index"] = nullptr;
    sample["current_model_index_error_at_center_match_px"] = nullptr;
    sample["current_model_best_order_error_px"] = nullptr;
    sample["current_model_observed_points_px"] = nullptr;
    sample["current_model_projected_points_px"] = nullptr;
    if (targets.empty()) return;

    const auto & target = targets.front();
    const auto target_armors = target.armor_xyza_list();
    if (
        target.last_id < 0 ||
        static_cast<std::size_t>(target.last_id) >= target_armors.size()) {
        return;
    }

    const auto & current = target_armors[static_cast<std::size_t>(target.last_id)];
    const auto projected = solver.reproject_armor(
        current.head(3), current[3], target.armor_type, target.name);
    cv::Point2f projected_center;
    if (!quad_center(projected, projected_center)) return;

    double best_error = std::numeric_limits<double>::infinity();
    double best_center_error = std::numeric_limits<double>::infinity();
    cv::Point2f best_observed_center;
    const auto_aim::Armor * center_match = nullptr;
    std::size_t best_index = 0;
    std::size_t best_center_index = 0;
    std::size_t index = 0;
    for (const auto & armor : detected_armors) {
        const double error = mean_reprojection_error(armor.points, projected);
        if (error >= 0.0 && error < best_error) {
            best_error = error;
            best_index = index;
        }
        cv::Point2f observed_center;
        if (quad_center(armor.points, observed_center)) {
            const double center_error = cv::norm(projected_center - observed_center);
            if (center_error < best_center_error) {
                best_center_error = center_error;
                best_observed_center = observed_center;
                best_center_index = index;
                center_match = &armor;
            }
        }
        ++index;
    }
    if (std::isfinite(best_error)) {
        sample["current_model_reprojection_error_px"] = best_error;
        sample["current_model_reprojection_detection_index"] = best_index;
    }
    if (center_match) {
        // 中心匹配不依赖角点编号；dx/dy 定义为模型投影减检测角点中心。
        sample["current_model_center_error_px"] = best_center_error;
        sample["current_model_center_dx_px"] = projected_center.x - best_observed_center.x;
        sample["current_model_center_dy_px"] = projected_center.y - best_observed_center.y;
        sample["current_model_center_detection_index"] = best_center_index;
        sample["current_model_index_error_at_center_match_px"] =
            mean_reprojection_error(center_match->points, projected);
        sample["current_model_best_order_error_px"] =
            best_order_reprojection_error(center_match->points, projected);
        sample["current_model_observed_points_px"] = Json::array();
        sample["current_model_projected_points_px"] = Json::array();
        for (int point_index = 0; point_index < 4; ++point_index) {
            const auto & observed = center_match->points[point_index];
            const auto & model = projected[point_index];
            sample["current_model_observed_points_px"].push_back(
                Json::array({observed.x, observed.y}));
            sample["current_model_projected_points_px"].push_back(
                Json::array({model.x, model.y}));
        }
    }
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

void append_association_candidate_json(
    Json & sample, const std::string & prefix,
    const auto_aim::AssociationCandidateDebug & candidate)
{
    if (candidate.model_id < 0) return;
    sample[prefix + "_id"] = candidate.model_id;
    sample[prefix + "_gate_passed"] = candidate.gate_passed ? 1 : 0;
    sample[prefix + "_accepted"] = candidate.accepted ? 1 : 0;
    sample[prefix + "_angle_gate_passed"] = candidate.angle_gate_passed ? 1 : 0;
    sample[prefix + "_score_gate_passed"] = candidate.score_gate_passed ? 1 : 0;
    sample[prefix + "_position_gate_passed"] = candidate.position_gate_passed ? 1 : 0;
    sample[prefix + "_distance_gate_passed"] = candidate.distance_gate_passed ? 1 : 0;
    sample[prefix + "_mahalanobis_gate_passed"] =
        candidate.mahalanobis_gate_passed ? 1 : 0;
    sample[prefix + "_score"] = candidate.score;
    sample[prefix + "_position_error"] = candidate.position_error;
    sample[prefix + "_distance_error"] = candidate.distance_error;
    sample[prefix + "_mahalanobis_distance"] = candidate.mahalanobis_distance;
    sample[prefix + "_position_angle_error"] = candidate.position_angle_error;
    sample[prefix + "_distance_angle_error"] = candidate.distance_angle_error;
    sample[prefix + "_observed_x"] = candidate.observed_x;
    sample[prefix + "_observed_y"] = candidate.observed_y;
    sample[prefix + "_observed_z"] = candidate.observed_z;
    sample[prefix + "_predicted_x"] = candidate.predicted_x;
    sample[prefix + "_predicted_y"] = candidate.predicted_y;
    sample[prefix + "_predicted_z"] = candidate.predicted_z;
    sample[prefix + "_observed_distance"] = candidate.observed_distance;
    sample[prefix + "_predicted_distance"] = candidate.predicted_distance;
    sample[prefix + "_orientation_error"] = candidate.orientation_error;
    sample[prefix + "_bearing_error"] = candidate.bearing_error;
    sample[prefix + "_angle_error"] = candidate.angle_error;
    sample[prefix + "_raw_yaw_prediction_error"] = candidate.raw_yaw_prediction_error;
    sample[prefix + "_optimized_yaw_prediction_error"] =
        candidate.optimized_yaw_prediction_error;
    sample[prefix + "_raw_yaw"] = candidate.raw_yaw;
    sample[prefix + "_optimized_yaw"] = candidate.optimized_yaw;
    sample[prefix + "_yaw_correction"] = candidate.yaw_correction;
    sample[prefix + "_image_x"] = candidate.image_x;
    sample[prefix + "_image_y"] = candidate.image_y;
}

void append_association_json(Json & sample, const auto_aim::AssociationDebug & association)
{
    // 这些字段只记录关联决策的输入、门控和最终候选，不参与任何算法决策。
    // 真机日志需要据此区分 PnP 误差、模型 ID 切换和 EKF 中心漂移。
    sample["association_matching_detection_count"] =
        association.matching_detection_count;
    sample["association_gate_passed_count"] = association.gate_passed_count;
    sample["association_candidate_count"] = association.candidate_count;
    sample["association_accepted_count"] = association.accepted_count;
    append_association_candidate_json(sample, "association_primary", association.candidates[0]);
    append_association_candidate_json(sample, "association_secondary", association.candidates[1]);
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

void append_image_quality_json(Json & sample, const cv::Mat & image)
{
    if (image.empty()) return;

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = image;
    }
    if (gray.empty() || gray.depth() != CV_8U) return;

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(gray, mean, stddev);
    cv::Mat dark_mask;
    cv::Mat bright_mask;
    cv::inRange(gray, 0, 2, dark_mask);
    cv::inRange(gray, 253, 255, bright_mask);

    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar lap_mean;
    cv::Scalar lap_stddev;
    cv::meanStdDev(laplacian, lap_mean, lap_stddev);

    const double pixel_count = static_cast<double>(gray.total());
    sample["image_mean_luma"] = mean[0];
    sample["image_std_luma"] = stddev[0];
    sample["image_dark_ratio"] =
        pixel_count > 0.0 ? cv::countNonZero(dark_mask) / pixel_count : 0.0;
    sample["image_bright_ratio"] =
        pixel_count > 0.0 ? cv::countNonZero(bright_mask) / pixel_count : 0.0;
    sample["image_laplacian_variance"] = lap_stddev[0] * lap_stddev[0];
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
        io::CameraSettingsOverride camera_overrides;
        if (exposure_override > 0.0) camera_overrides.exposure_ms = exposure_override;
        if (gain_override >= 0.0) camera_overrides.gain = gain_override;
        io::Camera camera(config_path, camera_overrides);
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
        const auto save_frame_path = cli.get<std::string>("save-frame");
        std::uint64_t saved_frame_count = 0;
        std::uint64_t captured_frame_count = 0;
        const auto raw_frame_dir = cli.get<std::string>("raw-frame-dir");
        const auto save_raw_frame =
            [&raw_frame_dir, &captured_frame_count](const cv::Mat & image) {
                if (raw_frame_dir.empty() || image.empty()) return;
                try {
                    const std::filesystem::path output_dir(raw_frame_dir);
                    std::filesystem::create_directories(output_dir);
                    std::ostringstream filename;
                    filename << "frame_" << std::setfill('0') << std::setw(6)
                             << captured_frame_count << ".jpg";
                    const auto output_path = output_dir / filename.str();
                    if (!cv::imwrite(
                          output_path.string(), image,
                          {cv::IMWRITE_JPEG_QUALITY, 95})) {
                        tools::logger()->warn(
                          "Failed to save raw frame to {}", output_path.string());
                    }
                } catch (const std::exception & error) {
                    tools::logger()->warn("Failed to save raw frame: {}", error.what());
                }
            };
        const auto save_display_frame =
            [&save_frame_path, &saved_frame_count](const cv::Mat & image) {
                if (save_frame_path.empty() || image.empty()) return;
                try {
                    const std::filesystem::path base_path(save_frame_path);
                    const auto parent_path = base_path.parent_path();
                    if (!parent_path.empty()) {
                        std::filesystem::create_directories(parent_path);
                    }
                    std::ostringstream numbered_name;
                    numbered_name << base_path.stem().string() << "_"
                                  << std::setfill('0') << std::setw(6)
                                  << ++saved_frame_count
                                  << base_path.extension().string();
                    const auto output_path = parent_path / numbered_name.str();
                    if (cv::imwrite(output_path.string(), image)) {
                        tools::logger()->info(
                            "Saved display frame to {}", output_path.string());
                    } else {
                        tools::logger()->warn(
                            "Failed to save display frame to {}", output_path.string());
                    }
                } catch (const std::exception & error) {
                    tools::logger()->warn("Failed to save display frame: {}", error.what());
                }
            };
        // 保存的是已经旋转为正立并叠加诊断信息的显示帧，不改变算法使用的原始图像。
        const auto handle_window_key =
            [&window_exit, &save_display_frame](const cv::Mat & image) {
                const int key = cv::waitKey(1) & 0xff;
                if (key == 's' || key == 'S') {
                    save_display_frame(image);
                } else if (key == 27 || key == 'q' || key == 'Q') {
                    window_exit = true;
                }
            };
        const auto log_skip = [&next_report](const char * reason) {
            const auto now = Clock::now();
            if (now >= next_report) {
                tools::logger()->warn("[standard_srm/READ_ONLY] {} (NO_TX)", reason);
                next_report = now + std::chrono::seconds(1);
            }
        };
        const auto show_skip_frame = [show, &handle_window_key](
                                         const cv::Mat & image, const std::string & reason) {
            if (!show || image.empty()) return;
            cv::Mat visualization = image.clone();
            cv::rotate(visualization, visualization, cv::ROTATE_180);
            draw_text(visualization, reason, 0, cv::Scalar(0, 255, 255));
            cv::imshow("standard_srm", visualization);
            handle_window_key(visualization);
        };
        while (
            !exiter.exit() && !window_exit &&
            (max_frames == 0 || captured_frame_count < static_cast<std::uint64_t>(max_frames))) {
            cv::Mat image;
            io::FrameTiming timing;
            camera.read_timed(image, timing);
            if (image.empty()) {
                log_skip("empty camera frame");
                continue;
            }
            ++captured_frame_count;
            save_raw_frame(image);
            if (image.cols != config.image_width || image.rows != config.image_height) {
                throw std::runtime_error("Camera frame size differs from calibration image_size");
            }
            const auto frame_read_at = Clock::now();
            auto sample = timing_json(timing, frame_read_at);
            sample["captured_frame"] = captured_frame_count;
            sample["image_width"] = image.cols;
            sample["image_height"] = image.rows;
            if (quality_metrics) append_image_quality_json(sample, image);
            // 优先使用设备计数映射时间；USB 相机频率不可用时，退回主机收帧时间。
            // 两者在诊断数据中通过 timestamp_source 区分，避免误认为曝光时刻。
            std::optional<Clock::time_point> image_time;
            if (timing.mapped_capture_at) {
                image_time = timing.mapped_capture_at;
            } else if (timing.timestamp_source == io::FrameTimestampSource::HostReceive) {
                image_time = timing.host_received_at;
            }
            if (!image_time) {
                const char * reason = timing.device_timestamp_hz == 0
                    ? "camera timestamp unavailable"
                    : timing.device_ticks == 0
                        ? "camera frame has no device timestamp"
                        : "camera timestamp mapping warming up or reset";
                sample["event"] = "skip";
                sample["skip_reason"] = reason;
                recorder.write(sample);
                show_skip_frame(image, reason);
                log_skip(reason);
                continue;
            }
            const auto image_timestamp = *image_time;
            const auto received_at = Clock::now();
            if (image_timestamp > received_at ||
                received_at - image_timestamp > config.max_image_age) {
                sample["event"] = "skip";
                sample["skip_reason"] = "camera frame timestamp is invalid or stale";
                recorder.write(sample);
                show_skip_frame(image, "stale camera timestamp");
                log_skip("camera frame timestamp is invalid or stale");
                continue;
            }
            sample["image_age_ms"] = duration_ms(image_timestamp, received_at);
            if (timing.mapped_capture_at) {
                sample["mapped_age_ms"] = sample["image_age_ms"];
                sample["mapping_delay_ms"] =
                    duration_ms(*timing.mapped_capture_at, timing.host_received_at);
            }
            const auto feedback = serial_feedback.sample_at(image_timestamp);
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
            // Tracker 会原地过滤装甲板；诊断必须保留检测器原始结果，避免把过滤后的列表当作检测结果。
            const auto detected_armors_for_diagnostic = armors;
            if (!armors.empty()) {
                double confidence_sum = 0.0;
                double confidence_max = 0.0;
                for (const auto & armor : armors) {
                    confidence_sum += armor.confidence;
                    confidence_max = std::max(confidence_max, armor.confidence);
                }
                sample["mean_confidence"] =
                    confidence_sum / static_cast<double>(armors.size());
                sample["max_confidence"] = confidence_max;
            } else {
                sample["mean_confidence"] = 0.0;
                sample["max_confidence"] = 0.0;
            }
            auto targets = tracker.track(armors, image_timestamp);
            const auto tracker_end = Clock::now();
            const double bullet_speed = feedback->feedback.bullet_speed_mps;
            // if (!std::isfinite(bullet_speed) || bullet_speed < 14.0) {//弹速小于0,不能正常绘制检测框
            //简单验证无弹速的aimmer
            if (!std::isfinite(bullet_speed)) {
                sample["event"] = "skip";
                sample["skip_reason"] = "invalid bullet speed; Aimer default-speed fallback is disabled";
                sample["detected"] = detected;
                sample["tracker_state"] = tracker.state();
                sample["tracker_generation"] = tracker.target_generation();
                sample["detector_ms"] = duration_ms(detector_start, detector_end);
                sample["tracker_ms"] = duration_ms(detector_end, tracker_end);
                append_target_json(sample, targets);
                append_reprojection_json(
                    sample, solver, detected_armors_for_diagnostic, targets);
                append_association_json(sample, tracker.association_debug());
                recorder.write(sample);
                show_skip_frame(image, "invalid bullet speed");
                log_skip("invalid bullet speed; Aimer default-speed fallback is disabled");
                continue;
            }
            const auto aimer_start = Clock::now();
            const auto diagnostic = aimer.aim(targets, image_timestamp, bullet_speed);
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
            sample["capture_to_detector_ms"] = duration_ms(image_timestamp, detector_start);
            sample["capture_to_aimer_ms"] = duration_ms(image_timestamp, finished_at);
            append_target_json(sample, targets);
            append_reprojection_json(
                sample, solver, detected_armors_for_diagnostic, targets);
            append_association_json(sample, tracker.association_debug());
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
                cv::rotate(visualization, visualization, cv::ROTATE_180);
                draw_text(
                    visualization,
                    "frame=" + std::to_string(timing.frame_id) +
                      " detected=" + std::to_string(detected) +
                      " targets=" + std::to_string(targets.size()),
                    0, cv::Scalar(0, 255, 255));
                draw_text(
                    visualization,
                    "state=" + tracker.state() +
                      " age=" + std::to_string(sample["image_age_ms"].get<double>()) +
                      "ms bracket=" + std::to_string(feedback->bracket_ms) + "ms",
                    1, cv::Scalar(0, 255, 255));
                draw_text(
                    visualization,
                    "yaw=" + std::to_string(diagnostic.yaw * 180.0 / CV_PI) +
                      " pitch=" + std::to_string(diagnostic.pitch * 180.0 / CV_PI) +
                      " speed=" + std::to_string(bullet_speed),
                    2, cv::Scalar(0, 255, 255));
                cv::imshow("standard_srm", visualization);
                handle_window_key(visualization);
            }

            if (finished_at >= next_report) {
                const double image_age_ms = std::chrono::duration<double, std::milli>(
                    finished_at - image_timestamp).count();
                const double mapping_delay_ms = timing.mapped_capture_at
                    ? std::chrono::duration<double, std::milli>(
                        timing.host_received_at - *timing.mapped_capture_at).count()
                    : 0.0;
                tools::logger()->info(
                    "[standard_srm/READ_ONLY] frame={} ticks={} tick_hz={} "
                    "timestamp_source={} "
                    "image_age={:.1f}ms mapping_delay={:.1f}ms bracket={:.1f}ms "
                    "detected={} targets={} tracker={} speed={:.2f}m/s mode={} color={} "
                    "aim_control={} diagnostic_yaw={:.2f}deg diagnostic_pitch={:.2f}deg NO_TX",
                    timing.frame_id, timing.device_ticks, timing.device_timestamp_hz,
                    io::frame_timestamp_source_name(timing.timestamp_source),
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
        if (max_frames > 0) {
            tools::logger()->info(
              "[standard_srm/READ_ONLY] max-frames reached: {} frames (NO_TX)",
              captured_frame_count);
        }
        return 0;
    } catch (const std::exception & error) {
        std::cerr << "standard_srm failed: " << error.what() << '\n';
        return 1;
    }
}
