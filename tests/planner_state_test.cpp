#include "tasks/auto_aim/planner/planner.hpp"

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
    std::cout << "planner_state_test passed\n";
}
