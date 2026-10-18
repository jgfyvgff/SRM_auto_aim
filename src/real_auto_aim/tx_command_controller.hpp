#pragma once

#include <optional>
#include <string>

#include "src/real_auto_aim/serial_feedback_reader.hpp"
#include "src/real_auto_aim/tx_command_limiter.hpp"
#include "tasks/auto_aim/planner/planner.hpp"

namespace real_auto_aim
{
// TX 参数沿用 CLI 单位：时间为 ms，角度为 deg，角速度为 deg/s。
struct TxCommandOptions
{
    bool enabled = false;
    bool nominal = false;
    int period_ms = 20;  // 正值，串口命令发送周期。
    int ttl_ms = 100;  // 正值，普通目标命令有效期。
    double yaw_rate = 60.0;  // 正值，yaw 角速度上限。
    double pitch_rate = 30.0;  // 正值，pitch 角速度上限。
    double yaw_step = 2.0;  // 正值，单条命令 yaw 变化上限。
    double pitch_step = 1.0;  // 正值，单条命令 pitch 变化上限。
    bool follow_lost = false;  // 是否允许短暂 temp_lost 外推。
    int lost_frames = 5;  // 外推允许的最大连续丢帧数，非负。
    double lost_scale = 0.35;  // 外推限速比例，必须位于 (0, 1]。
};

// 主线程拥有命令状态；串口读写仍由 SerialFeedbackReader 的唯一工作线程负责。
class TxCommandController
{
public:
    TxCommandController(SerialFeedbackReader & serial_feedback, TxCommandOptions options);
    // 每帧开头清除本帧状态快照，但保留跨帧的最后有效命令用于限速。
    void begin_frame();
    // 清除跨帧保持状态，并在 TX 启用时清空串口命令邮箱；可重复调用。
    void clear();
    // 在短暂无目标或速度信息不足时发送最后有效角度或当前姿态，禁止开火。
    void hold(const MatchedFeedback & feedback);
    // 根据 Planner 结果更新受限 TX 命令；feedback 必须是当前图像匹配到的有效姿态。
    void apply(
        const auto_aim::Plan & plan, const auto_aim::PlannerState & planner_state,
        const MatchedFeedback & feedback, bool measured_speed, bool has_measured_velocity,
        bool extrapolating);

    bool enabled() const { return options_.enabled; }
    bool nominal() const { return options_.nominal; }
    bool follow_lost() const { return options_.follow_lost; }
    int lost_frames() const { return options_.lost_frames; }
    bool sent() const { return sent_; }
    bool extrapolating() const { return sent_ && mode_ == "extrapolate"; }
    const std::string & mode() const { return mode_; }
    double yaw() const { return yaw_; }
    double pitch() const { return pitch_; }

private:
    SerialFeedbackReader & serial_feedback_;
    TxCommandOptions options_;
    TxRateLimits rate_limits_;
    bool sent_ = false;
    std::string mode_ = "none";
    double yaw_ = 0.0;
    double pitch_ = 0.0;
    std::optional<io::srm_auto_aim::CommandFrame> last_command_;
    std::optional<Clock::time_point> last_command_at_;
};
}  // namespace real_auto_aim
