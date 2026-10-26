#include <iostream>
#include <stdexcept>

#include "tasks/auto_aim/distance_observation_quality.hpp"

namespace
{
void require(bool value, const char * message)
{
    if (!value) throw std::runtime_error(message);
}
}  // namespace

int main()
{
    using namespace auto_aim;
    Armor armor(6, 0.95F, cv::Rect(0, 0, 100, 50),
        std::vector<cv::Point2f>{{10, 10}, {90, 10}, {90, 40}, {10, 40}});
    armor.xyz_in_world = Eigen::Vector3d(5, 0, 0);
    const auto original = armor.points;
    // 确定性替身故意重排 PnP 输出：预期仍沿同一完整姿态分支求导。
    const auto provider = [original](const Armor & sample) {
        std::vector<PnpCandidateDebug> branches(2);
        double distance = 5.0;
        for (std::size_t i = 0; i < 4; ++i) {
            distance += 0.1 * (sample.points[i].x - original[i].x);
            distance += 0.2 * (sample.points[i].y - original[i].y);
        }
        for (int i = 0; i < 2; ++i) {
            branches[i].xyz_in_camera = Eigen::Vector3d(0, 0, distance + i);
            branches[i].xyz_in_world = Eigen::Vector3d(distance + i, 0, 0);
            branches[i].yaw_in_world = i;
            branches[i].R_armor2world =
                Eigen::AngleAxisd(static_cast<double>(i), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        }
        if (distance > 5.0) std::swap(branches[0], branches[1]);
        return branches;
    };
    const auto quality = estimate_distance_quality(armor, 0.0, provider);
    require(quality.valid, "fixed branch sensitivity failed");
    require(std::abs(quality.sensitivity_m_per_px - std::sqrt(0.2)) < 1e-6,
        "world range Jacobian differs from analytic result");
    require(quality.update_scale(0.0) == 1.0, "record mode altered weight");
    require(quality.update_scale(1.0) == 1.0, "lower clamp failed");
    require(quality.update_scale(0.1) == 4.0, "upper clamp failed");
    require(std::abs(quality.update_scale(std::sqrt(0.1)) - 2.0) < 1e-6,
        "variance scale must square sensitivity ratio");
    auto malformed = armor;
    malformed.points.clear();
    require(!estimate_distance_quality(malformed, 0.0, provider).valid, "empty input accepted");
    require(!estimate_distance_quality(armor, 0.0,
        [](const Armor &) { return std::vector<PnpCandidateDebug>{}; }).valid,
        "PnP failure accepted");
    const auto ambiguous = [provider](const Armor & sample) {
        auto branches = provider(sample);
        branches[1].R_armor2world = branches[0].R_armor2world;
        return branches;
    };
    const auto failure = estimate_distance_quality(armor, 0.0, ambiguous);
    require(!failure.valid && failure.update_scale(0.1) == 1.0, "ambiguous branch must fall back");
    bool rejected = false;
    try { quality.update_scale(-1.0); } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "invalid reference accepted");
    std::cout << "distance_observation_quality_test passed\n";
}
