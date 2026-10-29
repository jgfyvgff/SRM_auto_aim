#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/aimer.hpp"

#include <cmath>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace
{
void require(bool condition, const char * message)
{
    if (!condition) throw std::runtime_error(message);
}
}

int main(int argc, char * argv[])
{
    if (argc != 2) throw std::invalid_argument("Expected Planner config path");
    auto_aim::Planner planner(argv[1]);
    auto_aim::Target target(3.0, 0.0, 0.2, 0.1);
    const auto_aim::PlannerState measured{0.35, 0.2, -0.1, -0.15};
    const auto plan = planner.plan(target, 23.0, measured);
    require(plan.diagnostic_valid, "Planner produced no finite diagnostic trajectory");
    require(!plan.control || plan.solver_converged,
            "Unconverged Planner trajectory was marked controllable");
    require(plan.solver_converged ==
                (plan.yaw_solver_status == 0 && plan.pitch_solver_status == 0),
            "Combined solver status disagrees with per-axis status");
    require(plan.yaw_solver_iterations > 0 && plan.pitch_solver_iterations > 0,
            "Planner did not report per-axis iteration counts");
    require(std::isfinite(plan.target_yaw_100ms) &&
                std::isfinite(plan.target_pitch_100ms) &&
                std::isfinite(plan.yaw_100ms) && std::isfinite(plan.pitch_100ms),
            "Planner 100 ms trajectory contains non-finite angles");
    require(std::isfinite(plan.yaw_primal_residual_max) &&
                std::isfinite(plan.yaw_dual_residual_max) &&
                std::isfinite(plan.pitch_primal_residual_max) &&
                std::isfinite(plan.pitch_dual_residual_max),
            "Planner per-axis residuals are non-finite");
    // 离散模型的角度在第一个 10 ms 步由实测角度和速度确定，不应从参考轨迹瞬移。
    require(std::abs(plan.yaw - (measured.yaw + auto_aim::DT * measured.yaw_vel)) < 0.001,
            "Yaw plan did not start from measured state");
    require(std::abs(plan.pitch - (measured.pitch + auto_aim::DT * measured.pitch_vel)) < 0.001,
            "Pitch plan did not start from measured state");
    require(!planner.plan(target, 0.0, measured).control,
            "Planner accepted zero bullet speed in current-state mode");
    auto_aim::Aimer aimer(argv[1]);
    // 正常选板入口按参考轨迹生成角度；反馈未对准时不能申请开火。
    auto tracking_plan = planner.plan(target, 23.0, measured, 0, &aimer);
    for (int attempt = 0; attempt < 20 && !tracking_plan.solver_converged; ++attempt) {
        tracking_plan = planner.plan(target, 23.0, measured, 0, &aimer);
    }
    require(tracking_plan.diagnostic_valid && tracking_plan.solver_converged,
            "Planner failed to solve reference-mode test case");
    require(std::hypot(tracking_plan.feedback_yaw_error_rad,
                       tracking_plan.feedback_pitch_error_rad) > 0.003 &&
                !tracking_plan.fire,
            "Planner requested fire while feedback was off target");
    const auto_aim::PlannerState aligned{
        tracking_plan.target_yaw, 0.0, tracking_plan.target_pitch, 0.0};
    auto aligned_plan = planner.plan(target, 23.0, aligned, 0, &aimer);
    for (int attempt = 0; attempt < 20 && !aligned_plan.solver_converged; ++attempt) {
        aligned_plan = planner.plan(target, 23.0, aligned, 0, &aimer);
    }
    require(aligned_plan.diagnostic_valid && aligned_plan.solver_converged,
            "Planner failed to solve aligned reference-mode case");
    require(std::hypot(aligned_plan.feedback_yaw_error_rad,
                       aligned_plan.feedback_pitch_error_rad) < 0.003,
            "Aligned feedback was not recognized");
    require(aligned_plan.fire, "Planner did not request fire when feedback was aligned");
    // 同板突刺沿已接受的斜边外推；实际换板保留角度跳变。
    constexpr double degree = 3.14159265358979323846 / 180.0;
    const auto start = std::chrono::steady_clock::time_point{};
    const auto filter = [&](double yaw_deg, int armor_id, int frame, bool mpc,
                            int preview_step = -1, int preview_id = -1) {
        auto_aim::Plan candidate{};
        candidate.control = mpc;
        candidate.diagnostic_valid = true;
        candidate.command_target_yaw = yaw_deg * degree;
        candidate.command_target_yaw_vel = -6.0 * degree;
        candidate.yaw = yaw_deg * degree;
        candidate.yaw_vel = 0.0F;
        candidate.fire = true;
        candidate.preview_switch_step = preview_step;
        candidate.preview_switch_id = preview_id;
        planner.prepare_tx_yaw(
            candidate, 1, armor_id,
            start + std::chrono::milliseconds(frame * 20));
        return candidate;
    };
    filter(103.43, 1, 0, false);
    filter(103.31, 1, 1, false);
    const auto spike = filter(106.34, 1, 2, false);
    require(spike.tx_yaw_spike_suppressed && !spike.fire &&
                std::abs(spike.tx_yaw / degree - 103.19) < 0.05,
            "Same-armor fallback spike reached TX yaw");
    const auto resumed = filter(103.14, 1, 3, false);
    require(!resumed.tx_yaw_spike_suppressed &&
                resumed.tx_yaw / degree < 103.25,
            "Fallback did not resume the descending line");
    const auto switched = filter(105.43, 0, 4, false);
    require(!switched.tx_yaw_spike_suppressed &&
                std::abs(switched.tx_yaw / degree - 105.43) < 0.01,
            "Actual armor switch was suppressed");

    // 单帧预览不动；同一未来板连续出现后限速提前，实际换板清除提前量。
    planner.reset_tx_yaw();
    filter(103.43, 1, 0, false);
    filter(103.31, 1, 1, false);
    const auto first_preview = filter(106.34, 1, 2, false, 2, 0);
    const auto second_preview = filter(106.22, 1, 3, false, 1, 0);
    const auto third_preview = filter(106.10, 1, 4, false, 1, 0);
    require(std::abs(first_preview.tx_preview_advance_yaw) < 1e-6 &&
                second_preview.tx_preview_advance_yaw > 0.0F &&
                third_preview.tx_preview_advance_yaw / degree <= 0.81 &&
                !second_preview.fire,
            "Preview transition was not bounded or gated");
    const auto preview_switched = filter(105.43, 0, 5, false);
    require(std::abs(preview_switched.tx_preview_advance_yaw) < 1e-6 &&
                std::abs(preview_switched.tx_yaw / degree - 105.43) < 0.01,
            "Actual switch retained preview advance");

    planner.reset_tx_yaw();
    filter(103.43, 1, 0, false);
    filter(103.31, 1, 1, false);
    filter(106.34, 1, 2, false, 2, 0);
    filter(106.22, 1, 3, false, 1, 0);
    const auto cancelled_preview = filter(102.95, 1, 4, false);
    require(std::abs(cancelled_preview.tx_preview_advance_yaw) < 1e-6,
            "Cancelled preview did not return to the old line");

    planner.reset_tx_yaw();
    filter(103.43, 1, 0, true);
    filter(103.31, 1, 1, true);
    const auto mpc_spike = filter(106.34, 1, 2, true);
    require(mpc_spike.tx_yaw_spike_suppressed &&
                std::abs(mpc_spike.tx_yaw / degree - 103.19) < 0.05,
            "Same-armor MPC spike reached TX yaw");

    planner.reset_tx_yaw();
    filter(179.9, 1, 0, false);
    const auto wrapped = filter(-179.9, 1, 1, false);
    require(!wrapped.tx_yaw_spike_suppressed,
            "Yaw wrap was mistaken for a spike");
    std::cout << "planner_state_test passed\n";
}
