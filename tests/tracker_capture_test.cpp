#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <iostream>
#include <list>
#include <stdexcept>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/tracker_association_policy.hpp"

namespace
{
using Clock = std::chrono::steady_clock;

auto_aim::Armor make_armor(int class_id, float shift_x = 0.0F)
{
    // 使用已记录的真机装甲板角点；同一组角点只改变分类，可隔离类别投票的影响。
    std::vector<cv::Point2f> points{
        {565.9178F + shift_x, 299.3274F},
        {727.1277F + shift_x, 300.8722F},
        {726.9770F + shift_x, 366.4089F},
        {564.7903F + shift_x, 364.0601F}};
    auto_aim::Armor armor(
        class_id, 0.95F, cv::Rect(564 + static_cast<int>(shift_x), 295, 164, 72),
        points);
    armor.priority = auto_aim::ArmorPriority::first;
    return armor;
}

std::list<auto_aim::Target> track_one(
    auto_aim::Tracker & tracker, int class_id, Clock::time_point t,
    float shift_x = 0.0F)
{
    std::list<auto_aim::Armor> armors{make_armor(class_id, shift_x)};
    return tracker.track(armors, t);
}

void require(bool condition, const char * message)
{
    if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char ** argv)
{
    if (argc != 2) return 2;

    const auto_aim::AssociationRank lower_geometry_residual{true, 0.25, 0.98};
    const auto_aim::AssociationRank lower_mahalanobis{true, 0.45, 0.57};
    require(
        auto_aim::association_rank_precedes(lower_geometry_residual, lower_mahalanobis),
        "lower geometric residual did not win between gated candidates");
    const auto_aim::AssociationRank rejected_candidate{false, 0.01, 0.01};
    require(
        auto_aim::association_rank_precedes(lower_geometry_residual, rejected_candidate),
        "a rejected candidate outranked a gated candidate");

    auto_aim::Solver solver(argv[1]);
    solver.set_gimbal_to_world(Eigen::Quaterniond::Identity());
    const auto start = Clock::now();

    // sentry 没有已标定几何，首次捕获不能生成四装甲跟随目标。
    auto_aim::Tracker unknown(argv[1], solver);
    require(track_one(unknown, 0, start).empty(), "unknown geometry produced a target");
    require(unknown.state() == "lost", "unknown geometry left lost state");
    require(unknown.geometry_branch() == "unknown_rejected", "unknown branch not recorded");
    require(!unknown.geometry_trusted(), "unknown geometry was marked trusted");
    require(!unknown.observation_accepted(), "unknown observation was accepted");
    require(unknown.target_generation() == 0, "unknown geometry rebuilt EKF");
    require(unknown.diagnostic_target() == nullptr,
            "unknown capture exposed an old filter as diagnostic target");
    for (int index = 1; index <= 3; ++index) {
        require(track_one(
                    unknown, 0, start + std::chrono::milliseconds(20 * index)).empty(),
                "repeated unknown capture produced a target");
    }

    // 先以 two 锁定四装甲物理模型，再让相同角点连续投票为 sentry。
    auto_aim::Tracker relabel(argv[1], solver);
    auto targets = track_one(relabel, 6, start);
    require(!targets.empty(), "standard target not captured");
    require(relabel.observation_accepted(), "first capture not recorded as observation");
    require(relabel.geometry_branch() == "standard_four", "wrong initial geometry");
    require(relabel.geometry_constraint_active(), "standard geometry constraint missing");
    for (int index = 1; index <= 6; ++index) {
        targets = track_one(
            relabel, 0, start + std::chrono::milliseconds(20 * index));
    }
    require(!targets.empty(), "continuous relabel lost target");
    require(relabel.diagnostic_target() != nullptr, "active filter diagnostics missing");
    require(relabel.state() == "tracking", "continuous relabel did not recover tracking");
    require(relabel.tracked_label() == auto_aim::ArmorName::sentry,
            "observation label was not updated");
    require(targets.front().name == auto_aim::ArmorName::two,
            "physical model changed on relabel");
    require(relabel.target_generation() == 1, "relabel rebuilt EKF");
    require(relabel.geometry_branch() == "standard_four" &&
            relabel.geometry_constraint_active(), "relabel lost geometry constraint");
    // 真机配置只硬约束两组半径差；半径本身是允许更新的软先验。
    require(std::abs(targets.front().ekf_x()[9]) < 1e-9,
            "relabel changed constrained radius delta");
    require(relabel.association_debug().accepted_count == 1 &&
            relabel.observation_accepted(), "continuous observation was not accepted");

    // 无检测帧可保留旧 NIS，但不得声称本帧产生了新观测或新 NIS。
    std::list<auto_aim::Armor> empty;
    targets = relabel.track(empty, start + std::chrono::milliseconds(140));
    require(!targets.empty() && relabel.state() == "temp_lost",
            "missing detection did not enter temp_lost");
    require(!relabel.observation_accepted() &&
            relabel.association_debug().accepted_count == 0,
            "missing detection was recorded as fresh observation");

    // 新标签的角点远离当前预测时，投票不得改变标签或 generation。
    auto_aim::Tracker discontinuous(argv[1], solver);
    track_one(discontinuous, 6, start);
    for (int index = 1; index <= 6; ++index) {
        track_one(
            discontinuous, 0,
            start + std::chrono::milliseconds(20 * index), 800.0F);
    }
    require(discontinuous.state() == "temp_lost", "discontinuous relabel was accepted");
    require(discontinuous.tracked_label() == auto_aim::ArmorName::two,
            "discontinuous relabel changed label");
    require(discontinuous.target_generation() == 1,
            "discontinuous relabel rebuilt EKF");
    require(!discontinuous.observation_accepted(),
            "discontinuous observation was marked accepted");

    std::cout << "tracker_capture_test passed\n";
    return 0;
}
