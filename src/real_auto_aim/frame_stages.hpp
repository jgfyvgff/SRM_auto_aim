#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "io/camera.hpp"
#include "src/real_auto_aim/frame_pipeline.hpp"
#include "src/real_auto_aim/serial_feedback_reader.hpp"
#include "src/real_auto_aim/tx_command_controller.hpp"

namespace real_auto_aim
{
struct FrameCaptureOptions
{
    std::string config_path;
    io::CameraSettingsOverride camera_overrides;
    int image_width = 0;
    int image_height = 0;
    std::chrono::milliseconds max_image_age{200};
    int max_frames = 0;
    bool tx_enabled = false;
    bool quality_metrics = false;
    bool replay_enabled = false;
};

// 采集线程独占相机和 YOLO；timeout 使相机无帧时仍可响应停止请求。
class FrameCaptureStage
{
public:
    explicit FrameCaptureStage(FrameCaptureOptions options);
    CaptureResult next();
    std::uint64_t captured_frames() const { return captured_frames_; }

private:
    FrameCaptureOptions options_;
    io::Camera camera_;
    FrameDetector detector_;
    std::uint64_t captured_frames_ = 0;
    std::optional<Clock::time_point> replay_time_origin_;
    Clock::time_point next_empty_report_ = Clock::now();
};

struct FrameAlgorithmOptions
{
    std::string config_path;
    TxCommandOptions tx_options;
    std::chrono::milliseconds max_image_age{200};
    double planner_debug_speed = 0.0;
    bool replay_enabled = false;
};

// 算法线程独占 Tracker/Aimer/Planner 与命令限速状态；只通过串口读者的线程安全接口通信。
class FrameAlgorithmStage
{
public:
    FrameAlgorithmStage(FrameAlgorithmOptions options, SerialFeedbackReader & serial_feedback);
    FrameOutput process(FrameInput && input, const std::atomic<bool> & stop_requested);

private:
    FrameAlgorithmOptions options_;
    SerialFeedbackReader & serial_feedback_;
    FrameProcessor processor_;
    TxCommandController tx_controller_;
};
}  // namespace real_auto_aim
