#include "src/real_auto_aim/frame_stages.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "src/real_auto_aim/frame_diagnostics.hpp"
#include "tools/logger.hpp"

namespace real_auto_aim
{
using namespace diagnostics;

FrameCaptureStage::FrameCaptureStage(FrameCaptureOptions options)
    : options_(std::move(options)),
      camera_(options_.config_path, options_.camera_overrides),
      detector_(options_.config_path)
{
    if (options_.image_width <= 0 || options_.image_height <= 0 ||
        options_.max_image_age.count() <= 0 || options_.max_frames < 0) {
        throw std::invalid_argument("Invalid frame capture options");
    }
}

CaptureResult FrameCaptureStage::next()
{
    if (options_.max_frames > 0 &&
        captured_frames_ >= static_cast<std::uint64_t>(options_.max_frames)) {
        return {CaptureKind::Done, {}};
    }
    cv::Mat image;
    io::FrameTiming timing;
    // 相机驱动内部已有取图线程；这里只取缓存帧，限时等待避免断流时无法 join。
    if (!camera_.read_timed_for(image, timing, std::chrono::milliseconds(50))) {
        return {CaptureKind::Retry, {}};
    }
    if (image.empty()) {
        const auto now = Clock::now();
        if (now >= next_empty_report_) {
            tools::logger()->warn("[standard_srm] empty camera frame");
            next_empty_report_ = now + std::chrono::seconds(1);
        }
        return {CaptureKind::Retry, {}};
    }
    ++captured_frames_;
    if (image.cols != options_.image_width || image.rows != options_.image_height) {
        throw std::runtime_error("Camera frame size differs from calibration image_size");
    }

    FrameInput input;
    input.image = std::move(image);
    input.timing = timing;
    input.captured_frame = captured_frames_;
    const auto frame_read_at = Clock::now();
    input.sample = timing_json(timing, frame_read_at);
    input.sample["captured_frame"] = captured_frames_;
    input.sample["image_width"] = input.image.cols;
    input.sample["image_height"] = input.image.rows;
    input.sample["tx_enabled"] = options_.tx_enabled;
    input.sample["tx_speed_source"] = "unavailable";
    input.sample["tx_command_sent"] = false;
    input.sample["tx_command_mode"] = "none";
    input.sample["tx_command_source"] = "none";
    input.sample["tx_command_yaw_deg"] = nullptr;
    input.sample["tx_command_pitch_deg"] = nullptr;
    if (options_.replay_enabled) {
        std::ostringstream filename;
        filename << "frame_" << std::setfill('0') << std::setw(6)
                 << captured_frames_ << ".png";
        input.sample["replay_schema_version"] = 1;
        input.sample["replay_frame_file"] = filename.str();
    }
    if (options_.quality_metrics) append_image_quality_json(input.sample, input.image);

    // 使用设备计数映射时间；旧相机只有主机收帧时间，不能当作真实曝光时刻。
    if (timing.mapped_capture_at) {
        input.image_timestamp = timing.mapped_capture_at;
    } else if (timing.timestamp_source == io::FrameTimestampSource::HostReceive) {
        input.image_timestamp = timing.host_received_at;
    }
    if (!input.image_timestamp) {
        input.skip_reason = timing.device_timestamp_hz == 0
            ? "camera timestamp unavailable"
            : timing.device_ticks == 0
                ? "camera frame has no device timestamp"
                : "camera timestamp mapping warming up or reset";
    } else {
        const auto image_timestamp = *input.image_timestamp;
        if (!replay_time_origin_) replay_time_origin_ = image_timestamp;
        if (options_.replay_enabled) {
            input.sample["replay_time_s"] =
                std::chrono::duration<double>(image_timestamp - *replay_time_origin_).count();
        }
        const auto received_at = Clock::now();
        if (image_timestamp > received_at ||
            received_at - image_timestamp > options_.max_image_age) {
            input.skip_reason = "camera frame timestamp is invalid or stale";
        } else {
            input.sample["image_age_ms"] = duration_ms(image_timestamp, received_at);
            if (timing.mapped_capture_at) {
                input.sample["mapped_age_ms"] = input.sample["image_age_ms"];
                input.sample["mapping_delay_ms"] =
                    duration_ms(*timing.mapped_capture_at, timing.host_received_at);
            }
            // 动态 ROI 前一帧先验只在本线程更新，不并行推理同一检测器。
            input.detections = detector_.detect(input.image, input.sample);
        }
    }
    input.queued_at = Clock::now();
    return {CaptureKind::Frame, std::move(input)};
}

FrameAlgorithmStage::FrameAlgorithmStage(
    FrameAlgorithmOptions options, SerialFeedbackReader & serial_feedback)
    : options_(std::move(options)), serial_feedback_(serial_feedback),
      processor_(options_.config_path),
      tx_controller_(serial_feedback_, options_.tx_options)
{}

FrameOutput FrameAlgorithmStage::process(
    FrameInput && input, const std::atomic<bool> & stop_requested)
{
    FrameOutput output;
    output.image = std::move(input.image);
    output.timing = input.timing;
    output.image_timestamp = input.image_timestamp;
    output.sample = std::move(input.sample);
    output.captured_frame = input.captured_frame;
    output.sample["pipeline_queue_wait_ms"] = duration_ms(input.queued_at, Clock::now());
    tx_controller_.begin_frame();

    const auto skip = [&](const std::string & reason, const std::string & display) {
        output.sample["event"] = "skip";
        output.sample["skip_reason"] = reason;
        output.display_reason = display;
        tx_controller_.clear();
    };
    if (!input.skip_reason.empty()) {
        skip(input.skip_reason, input.skip_reason);
        return output;
    }
    if (!input.image_timestamp) {
        skip("camera timestamp unavailable", "camera timestamp unavailable");
        return output;
    }
    const auto image_timestamp = *input.image_timestamp;
    const auto now = Clock::now();
    if (now < image_timestamp || now - image_timestamp > options_.max_image_age) {
        skip("frame became stale in processing queue", "stale queued frame");
        return output;
    }
    output.sample["pipeline_process_age_ms"] = duration_ms(image_timestamp, now);
    const auto feedback_wait_start = Clock::now();
    auto feedback = serial_feedback_.sample_at(image_timestamp);
    output.sample["feedback_wait_ms"] = duration_ms(feedback_wait_start, Clock::now());
    if (!feedback) {
        output.sample["detected"] = input.detections.count;
        output.sample["detector_ms"] = duration_ms(
            input.detections.detector_start, input.detections.detector_end);
        skip("no fresh feedback on both sides of image time", "waiting for serial feedback");
        return output;
    }
    output.feedback = *feedback;
    auto result = processor_.process(
        std::move(input.detections), image_timestamp, *feedback, output.sample,
        serial_feedback_, tx_controller_, options_.planner_debug_speed,
        options_.replay_enabled, options_.max_image_age, stop_requested);
    if (result.invalid_bullet_speed) {
        output.display_reason = "invalid bullet speed";
        output.result = std::move(result);
        return output;
    }

    if (!result.targets.empty()) {
        const auto & target = result.targets.front();
        const auto xyza = target.armor_xyza_list();
        if (target.last_id >= 0 &&
            static_cast<std::size_t>(target.last_id) < xyza.size()) {
            const auto & current = xyza[static_cast<std::size_t>(target.last_id)];
            output.current_polygon = processor_.solver().reproject_armor(
                current.head(3), current[3], target.armor_type, target.name);
        }
        if (processor_.aimer().debug_aim_point.valid) {
            const auto & future = processor_.aimer().debug_aim_point.xyza;
            output.future_polygon = processor_.solver().reproject_armor(
                future.head(3), future[3], target.armor_type, target.name);
        }
    }
    output.result = std::move(result);
    return output;
}
}  // namespace real_auto_aim
