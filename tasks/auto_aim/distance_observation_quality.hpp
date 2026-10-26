#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include "solver.hpp"

namespace auto_aim
{
// 固定分支下的局部灵敏度，不是标定后的噪声标准差；失败时不得用于降权。
struct DistanceObservationQuality
{
    bool valid = false;
    std::string reason = "disabled";
    int selected_branch = -1;
    std::array<double, 8> range_gradient_m_per_px{};
    double sensitivity_m_per_px = 0.0;

    // 参考值由主序列记录组冻结；4 是本次隔离实验上限，不读取生产配置。
    double update_scale(double reference_m_per_px) const
    {
        if (!std::isfinite(reference_m_per_px) || reference_m_per_px < 0.0) {
            throw std::invalid_argument("Invalid distance quality reference");
        }
        if (!valid || reference_m_per_px == 0.0) return 1.0;
        const double ratio = sensitivity_m_per_px / reference_m_per_px;
        return std::clamp(ratio * ratio, 1.0, 4.0);
    }
};

// 按完整姿态匹配两组分支，避免 OpenCV 按重投影误差重排时误把索引当身份。
// 半个原始分支间隔是连续性可分辨条件；不满足时只回退，不拒绝原观测。
inline int match_distance_branch(
    const std::vector<PnpCandidateDebug> & base,
    const std::vector<PnpCandidateDebug> & moved, int selected)
{
    if (base.size() != 2 || moved.size() != 2 || selected < 0 || selected > 1) return -1;
    const auto angle = [](const Eigen::Matrix3d & a, const Eigen::Matrix3d & b) {
        return Eigen::AngleAxisd(a.transpose() * b).angle();
    };
    const double separation = angle(base[0].R_armor2world, base[1].R_armor2world);
    if (!std::isfinite(separation) || separation <= 1e-6) return -1;
    double cost[2][2];
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            cost[i][j] = angle(base[i].R_armor2world, moved[j].R_armor2world);
            if (!std::isfinite(cost[i][j])) return -1;
        }
    }
    const double straight = cost[0][0] + cost[1][1];
    const double swapped = cost[0][1] + cost[1][0];
    if (std::abs(straight - swapped) <= 1e-9) return -1;
    const int swap = swapped < straight ? 1 : 0;
    if (cost[0][swap] >= separation / 2.0 ||
        cost[1][1 - swap] >= separation / 2.0) return -1;
    return selected ^ swap;
}

// provider 是无状态 PnP 适配器：真实调用固定本帧 Solver 姿态，测试可传确定性替身。
// ±0.25 px 仅为中心差分步长，不是假设角点误差；距离采用 EKF 的世界系径向距离。
template <class CandidateProvider>
DistanceObservationQuality estimate_distance_quality(
    const Armor & solved, double predicted_yaw, CandidateProvider provider)
{
    DistanceObservationQuality result;
    result.reason = "invalid_input";
    if (solved.points.size() != 4 || !std::isfinite(predicted_yaw) ||
        !solved.xyz_in_world.allFinite()) return result;
    for (const auto & point : solved.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return result;
    }
    const auto usable = [](const std::vector<PnpCandidateDebug> & candidates) {
        return candidates.size() == 2 && std::all_of(
            candidates.begin(), candidates.end(), [](const auto & c) {
                return c.xyz_in_camera.allFinite() && c.xyz_in_camera.z() > 0.0 &&
                    c.xyz_in_world.allFinite() && c.xyz_in_world.norm() > 0.0 &&
                    c.R_armor2world.allFinite() && std::isfinite(c.yaw_in_world);
            });
    };
    try {
        const auto base = provider(solved);
        result.reason = "invalid_pnp_pair";
        if (!usable(base)) return result;
        const auto yaw_error = [predicted_yaw](double yaw) {
            return std::abs(std::remainder(yaw - predicted_yaw, 2.0 * CV_PI));
        };
        result.selected_branch = yaw_error(base[0].yaw_in_world) <=
            yaw_error(base[1].yaw_in_world) ? 0 : 1;
        result.reason = "anchor_mismatch";
        if ((base[result.selected_branch].xyz_in_world - solved.xyz_in_world).norm() > 1e-6) {
            return result;
        }
        constexpr float step_px = 0.25F;
        double squared_norm = 0.0;
        for (std::size_t coordinate = 0; coordinate < 8; ++coordinate) {
            Armor plus = solved;
            Armor minus = solved;
            float & hi = coordinate % 2 == 0
                ? plus.points[coordinate / 2].x : plus.points[coordinate / 2].y;
            float & lo = coordinate % 2 == 0
                ? minus.points[coordinate / 2].x : minus.points[coordinate / 2].y;
            hi += step_px;
            lo -= step_px;
            const auto positive = provider(plus);
            const auto negative = provider(minus);
            result.reason = "invalid_perturbed_pnp";
            if (!usable(positive) || !usable(negative) || hi <= lo) return result;
            const int pos = match_distance_branch(base, positive, result.selected_branch);
            const int neg = match_distance_branch(base, negative, result.selected_branch);
            result.reason = "branch_not_separable";
            if (pos < 0 || neg < 0) return result;
            const double derivative =
                (positive[pos].xyz_in_world.norm() - negative[neg].xyz_in_world.norm()) /
                static_cast<double>(hi - lo);
            result.reason = "invalid_derivative";
            if (!std::isfinite(derivative)) return result;
            result.range_gradient_m_per_px[coordinate] = derivative;
            squared_norm += derivative * derivative;
        }
        result.sensitivity_m_per_px = std::sqrt(squared_norm);
        if (!std::isfinite(result.sensitivity_m_per_px)) return result;
        result.valid = true;
        result.reason = "ok";
    } catch (const cv::Exception &) {
        result.reason = "pnp_exception";
    }
    return result;
}
}  // namespace auto_aim
