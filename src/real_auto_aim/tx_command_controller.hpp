#pragma once

#include <optional>
#include <string>

#include "src/real_auto_aim/serial_feedback_reader.hpp"
#include "tasks/auto_aim/planner/planner.hpp"

namespace real_auto_aim
{
// TX 周期和有效期使用毫秒；Planner 角度转换为度后写入命令帧。
struct TxCommandOptions
{
    bool enabled = false;
    bool nominal = false;
    int period_ms = 20;  // 正值，串口命令发送周期。
    int ttl_ms = 100;  // 正值，普通目标命令有效期。
    bool follow_lost = true;  // 短暂 temp_lost 默认继续跟随 Tracker 预测目标。
    int lost_frames = 5;  // 外推允许的最大连续丢帧数，非负。
    bool allow_fire = false;  // 必须显式启用；实际开火还要满足 Planner 和弹速条件。
};

// 主线程拥有命令状态；串口读写仍由 SerialFeedbackReader 的唯一工作线程负责。
class TxCommandController
{
public:
    TxCommandController(SerialFeedbackReader & serial_feedback, TxCommandOptions options);
    // 每帧开头清除本帧状态快照，但保留最后有效命令供无目标时保持。
    void begin_frame();
    // 清除跨帧保持状态，并在 TX 启用时清空串口命令邮箱；可重复调用。
    void clear();
    // 无可用预测目标时保持上一条有效命令；首次跟踪前使用当前反馈角，禁止开火。
    void hold(const MatchedFeedback & feedback);
    // 写入 Planner 参考角；开火请求仍受 allow_fire、收敛、弹速和外推状态约束。
    void apply(
        const auto_aim::Plan & plan, const MatchedFeedback & feedback,
        bool measured_speed, bool extrapolating);

    bool enabled() const { return options_.enabled; }
    bool nominal() const { return options_.nominal; }
    bool follow_lost() const { return options_.follow_lost; }
    int lost_frames() const { return options_.lost_frames; }
    bool sent() const { return sent_; }
    bool extrapolating() const { return sent_ && mode_ == "extrapolate"; }
    const std::string & mode() const { return mode_; }
    const std::string & source() const { return source_; }
    double yaw() const { return yaw_; }
    double pitch() const { return pitch_; }
    int fire_flag() const {
        return sent_ && last_command_ ? last_command_->fire_flag : 0;
    }

private:
    SerialFeedbackReader & serial_feedback_;
    TxCommandOptions options_;
    bool sent_ = false;
    std::string mode_ = "none";
    std::string source_ = "none";
    double yaw_ = 0.0;
    double pitch_ = 0.0;
    std::optional<io::srm_auto_aim::CommandFrame> last_command_;
};
}  // namespace real_auto_aim
