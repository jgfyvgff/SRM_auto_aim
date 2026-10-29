#include "src/real_auto_aim/tx_command_controller.hpp"

#include <cmath>

#include "tools/math_tools.hpp"

namespace real_auto_aim
{
TxCommandController::TxCommandController(
    SerialFeedbackReader & serial_feedback, TxCommandOptions options)
    : serial_feedback_(serial_feedback), options_(options)
{}

void TxCommandController::begin_frame()
{
    mode_ = "none";
    source_ = "none";
    sent_ = false;
    yaw_ = 0.0;
    pitch_ = 0.0;
}

void TxCommandController::clear()
{
    last_command_.reset();
    if (options_.enabled) serial_feedback_.clear_command();
}

void TxCommandController::hold(const MatchedFeedback & feedback)
{
    if (!options_.enabled) return;
    // 预测窗口结束后保持最后一条命令，避免切到反馈角产生阶跃。
    const auto command = last_command_.value_or(
        io::srm_auto_aim::CommandFrame{
            feedback.feedback.yaw_deg, feedback.feedback.pitch_deg, 0});
    // 保持角不是新的瞄准判定，不能延续上一帧的开火请求。
    auto hold_command = command;
    hold_command.fire_flag = 0;
    if (std::isfinite(hold_command.yaw_deg) && std::isfinite(hold_command.pitch_deg)) {
        serial_feedback_.set_hold_command(hold_command);
        last_command_ = hold_command;
        sent_ = true;
        mode_ = "hold_last_aim";
        source_ = "hold";
        yaw_ = hold_command.yaw_deg;
        pitch_ = hold_command.pitch_deg;
    } else {
        clear();
    }
}

void TxCommandController::apply(
    const auto_aim::Plan & plan, const MatchedFeedback & feedback,
    bool measured_speed, bool extrapolating)
{
    const bool finite_target =
        std::isfinite(plan.target_yaw) && std::isfinite(plan.target_pitch) &&
        std::isfinite(plan.command_target_yaw) &&
        std::isfinite(plan.command_target_pitch) &&
        plan.tx_yaw_valid && std::isfinite(plan.tx_yaw);
    if (!plan.diagnostic_valid || !finite_target ||
        (!measured_speed && !options_.nominal)) {
        if (extrapolating) {
            hold(feedback);
        } else {
            clear();
        }
        return;
    }

    // Planner 已统一选择并处理最终 yaw；TX 只负责换算和发送。
    const bool ready = options_.enabled && (measured_speed || options_.nominal) &&
                       plan.diagnostic_valid && finite_target;
    if (ready) {
        const bool use_mpc = plan.control;
        const double target_yaw_deg = plan.tx_yaw * 180.0 / CV_PI;
        const double target_pitch_deg = use_mpc
            ? (plan.pitch + auto_aim::DT * plan.pitch_vel) * 180.0 / CV_PI
            : plan.command_target_pitch * 180.0 / CV_PI;
        if (!std::isfinite(target_yaw_deg) || !std::isfinite(target_pitch_deg)) {
            clear();
            return;
        }
        const io::srm_auto_aim::CommandFrame command{
            static_cast<float>(target_yaw_deg),
            static_cast<float>(target_pitch_deg),
            // 名义弹速模式下也可依据 Planner 判据申请开火；丢帧外推仍禁止。
            options_.allow_fire && !extrapolating &&
                    plan.control && plan.fire
                ? 1
                : 0};
        serial_feedback_.set_command(command);
        last_command_ = command;
        sent_ = true;
        mode_ = extrapolating ? "extrapolate" : "tracking";
        source_ = use_mpc ? "mpc_reference_20ms" : "target_reference";
        yaw_ = command.yaw_deg;
        pitch_ = command.pitch_deg;
    }
}
}  // namespace real_auto_aim
