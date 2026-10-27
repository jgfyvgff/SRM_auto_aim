#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/aimer.hpp"

#include <cmath>
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
    std::cout << "planner_state_test passed\n";
}
