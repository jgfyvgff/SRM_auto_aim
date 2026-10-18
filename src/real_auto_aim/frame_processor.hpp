#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <list>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "io/command.hpp"
#include "src/real_auto_aim/feedback_buffer.hpp"
#include "src/real_auto_aim/serial_feedback_reader.hpp"
#include "src/real_auto_aim/tx_command_controller.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace real_auto_aim
{
using Json = nlohmann::json;

// 检测器原始输出与 Tracker 可变更新的列表分开保存，便于日志复盘原始检测。
struct FrameDetections
{
    std::list<auto_aim::Armor> armors;
    std::list<auto_aim::Armor> diagnostic_armors;
    std::size_t count = 0;
    Clock::time_point detector_start;
    Clock::time_point detector_end;
};

// 处理器将算法结果返回给应用层；无效弹速时只填充跳帧诊断并设置标志。
struct FrameProcessResult
{
    FrameDetections detections;
    std::list<auto_aim::Target> targets;
    io::Command diagnostic{};
    double bullet_speed = 0.0;
    bool invalid_bullet_speed = false;
    std::string tracker_state;
    std::uint64_t tracker_generation = 0;
    int temp_lost_count = 0;
    Clock::time_point finished_at;
};

// 检测器只由采集线程调用；动态 ROI 使用连续检测帧维护上一帧先验。
class FrameDetector
{
public:
    explicit FrameDetector(const std::string & config_path);
    FrameDetections detect(const cv::Mat & image, Json & sample);

private:
    auto_aim::YOLO detector_;
};

// Tracker、Aimer 和 Planner 的可变状态只由同一个算法线程按帧顺序更新。
class FrameProcessor
{
public:
    explicit FrameProcessor(const std::string & config_path);

    // 在匹配姿态到达后更新 Tracker、Aimer、Planner，并填充本帧诊断字段。
    FrameProcessResult process(
        FrameDetections detections, Clock::time_point image_timestamp,
        const MatchedFeedback & feedback, Json & sample,
        SerialFeedbackReader & serial_feedback, TxCommandController & tx_controller,
        double planner_debug_speed, bool replay_enabled,
        std::chrono::milliseconds max_image_age,
        const std::atomic<bool> & stop_requested);

    auto_aim::Solver & solver() { return solver_; }
    const auto_aim::Aimer & aimer() const { return aimer_; }
    const auto_aim::Tracker & tracker() const { return tracker_; }

private:
    auto_aim::Solver solver_;
    auto_aim::Tracker tracker_;
    auto_aim::Aimer aimer_;
    auto_aim::Planner planner_;
};
}  // namespace real_auto_aim
