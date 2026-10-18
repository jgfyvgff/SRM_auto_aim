#include "src/real_auto_aim/tx_command_controller.hpp"

#include <algorithm>
#include <cmath>

#include "tools/math_tools.hpp"

namespace real_auto_aim
{
TxCommandController::TxCommandController(
    SerialFeedbackReader & serial_feedback, TxCommandOptions options)
    : serial_feedback_(serial_feedback), options_(options),
      rate_limits_{
          options.yaw_rate, options.pitch_rate, options.yaw_step, options.pitch_step}
{}

void TxCommandController::begin_frame()
{
    mode_ = "none";
    sent_ = false;
    yaw_ = 0.0;
    pitch_ = 0.0;
}

void TxCommandController::clear()
{
    last_command_.reset();
    last_command_at_.reset();
    if (options_.enabled) serial_feedback_.clear_command();
}

void TxCommandController::hold(const MatchedFeedback & feedback)
{
    if (!options_.enabled) return;
    const auto command = last_command_.value_or(
        io::srm_auto_aim::CommandFrame{
            feedback.feedback.yaw_deg, feedback.feedback.pitch_deg, 0});
    if (std::isfinite(command.yaw_deg) && std::isfinite(command.pitch_deg)) {
        serial_feedback_.set_hold_command(command);
        last_command_at_ = Clock::now();
        sent_ = true;
        mode_ = last_command_ ? "hold_last_aim" : "hold_current";
        yaw_ = command.yaw_deg;
        pitch_ = command.pitch_deg;
    } else {
        clear();
    }
}

void TxCommandController::apply(
    const auto_aim::Plan & plan, const auto_aim::PlannerState & planner_state,
    const MatchedFeedback & feedback, bool measured_speed,
    bool has_measured_velocity, bool extrapolating)
{
    const bool finite_target =
        std::isfinite(plan.target_yaw) && std::isfinite(plan.target_pitch);
    if (!plan.diagnostic_valid || !finite_target ||
        (!measured_speed && !options_.nominal)) {
        if (extrapolating) {
            hold(feedback);
        } else {
            clear();
        }
    }

    // 有限参考角可用于命令跟随；最终变化仍受真实发送间隔和单步上限约束。
    const bool ready = options_.enabled && (measured_speed || options_.nominal) &&
                       has_measured_velocity && plan.diagnostic_valid && finite_target;
    if (ready) {
        const auto command_at = Clock::now();
        const double elapsed_s = last_command_at_ ? std::clamp(
            std::chrono::duration<double>(command_at - *last_command_at_).count(),
            0.0, options_.ttl_ms / 1000.0) : options_.period_ms / 1000.0;
        const TxAnglesDeg base = last_command_ ?
            TxAnglesDeg{last_command_->yaw_deg, last_command_->pitch_deg} :
            TxAnglesDeg{
                planner_state.yaw * 180.0 / CV_PI,
                planner_state.pitch * 180.0 / CV_PI};
        const TxAnglesDeg target{
            plan.target_yaw * 180.0 / CV_PI,
            plan.target_pitch * 180.0 / CV_PI};
        if (std::isfinite(base.yaw) && std::isfinite(base.pitch)) {
            const auto limited = limit_tx_angles(
                base, target, elapsed_s, rate_limits_,
                extrapolating ? options_.lost_scale : 1.0);
            if (std::isfinite(limited.yaw) && std::isfinite(limited.pitch)) {
                const io::srm_auto_aim::CommandFrame command{
                    static_cast<float>(limited.yaw),
                    static_cast<float>(limited.pitch),
                    0};
                serial_feedback_.set_command(command);
                last_command_ = command;
                last_command_at_ = command_at;
                sent_ = true;
                mode_ = extrapolating ? "extrapolate" : "tracking";
                yaw_ = command.yaw_deg;
                pitch_ = command.pitch_deg;
            } else {
                clear();
            }
        } else {
            clear();
        }
    } else if (options_.enabled && plan.diagnostic_valid && finite_target &&
               (measured_speed || options_.nominal) && !has_measured_velocity) {
        hold(feedback);
    }
}
}  // namespace real_auto_aim
