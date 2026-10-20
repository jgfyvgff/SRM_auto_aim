#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/math_tools.hpp"

namespace real_auto_aim
{
// 仅用于离线比较的选板过渡曲线；不读取实测云台状态，也不允许进入真机命令链。
class QuinticSwitchDiagnostic
{
public:
    struct Sample
    {
        double yaw_rad = 0.0;
        const char * mode = "follow";
        double transition_ms = 0.0;
    };

    // frame_s 是估计的命令周期；速度约束同时受每条指令步长和每秒速率限制。
    QuinticSwitchDiagnostic(double max_speed_rad_s, double max_acc_rad_s2)
        : max_speed_(max_speed_rad_s), max_acc_(max_acc_rad_s2)
    {
        if (!std::isfinite(max_speed_) || max_speed_ <= 0.0 ||
            !std::isfinite(max_acc_) || max_acc_ <= 0.0) {
            throw std::invalid_argument("Invalid quintic diagnostic limits");
        }
    }

    Sample update(
        double now_s, std::uint64_t generation, int armor_id,
        double follow_yaw_rad, const auto_aim::SwitchPreview & preview)
    {
        if (!std::isfinite(now_s) || !std::isfinite(follow_yaw_rad)) {
            reset();
            return {follow_yaw_rad, "invalid", 0.0};
        }
        if ((last_time_s_ && now_s <= *last_time_s_) ||
            (generation_ && generation != *generation_) ||
            (curve_ && armor_id != curve_->from_id &&
             armor_id != curve_->to_id)) {
            // 换目标或时间倒退时，旧目标上的曲线不能继续驱动新目标的诊断。
            reset();
        }
        generation_ = generation;
        last_time_s_ = now_s;
        if (curve_) {
            if (now_s <= curve_->end_s) {
                if (now_s < curve_->start_s) {
                    return {follow_yaw_rad, "scheduled", curve_->duration_s * 1000.0};
                }
                return {
                    tools::limit_rad(curve_->position(now_s)),
                    "transition",
                    curve_->duration_s * 1000.0};
            }
            curve_.reset();
        }

        if (!preview.valid || preview.switch_step < 0 ||
            preview.switch_id < 0 || preview.switch_id == armor_id) {
            candidate_.reset();
            return {follow_yaw_rad, "follow", 0.0};
        }
        const double switch_s = now_s + preview.switch_step * auto_aim::DT;
        if (!candidate_ || candidate_->to_id != preview.switch_id ||
            candidate_->from_id != armor_id ||
            std::abs(candidate_->switch_s - switch_s) > 0.05) {
            candidate_ = Candidate{armor_id, preview.switch_id, switch_s, 1};
            return {follow_yaw_rad, "forecast_unstable", 0.0};
        }
        candidate_->switch_s = switch_s;
        candidate_->seen += 1;
        if (candidate_->seen < 2) {
            return {follow_yaw_rad, "forecast_unstable", 0.0};
        }

        // 过渡终点固定在新板首点，避免接入峰值之后的下降段。
        // 起点必须在当前之后且不超过 80 ms，避免用已经错过的预测窗口补画曲线。
        for (int duration_steps : {12, 14, 16, 18, 20, 22, 24, 26, 28}) {
            const int end = preview.switch_step;
            const int start = end - duration_steps;
            if (start < 0 || start > 8 || end + 3 >= auto_aim::HORIZON) continue;
            const auto same_id = [&](int first, int id) {
                for (int step = first; step <= first + 3; ++step) {
                    if (preview.armor_id[step] != id) return false;
                }
                return true;
            };
            if (!same_id(start, armor_id) ||
                !same_id(end, preview.switch_id)) continue;
            const auto candidate_curve = fit_curve(
                preview, start, end, now_s, armor_id, preview.switch_id);
            if (candidate_curve) {
                curve_ = *candidate_curve;
                candidate_.reset();
                if (now_s < curve_->start_s) {
                    return {follow_yaw_rad, "scheduled", curve_->duration_s * 1000.0};
                }
                return {
                    tools::limit_rad(curve_->position(now_s)),
                    "transition", curve_->duration_s * 1000.0};
            }
        }
        return {follow_yaw_rad, "constraint_unavailable", 0.0};
    }

    void reset()
    {
        last_time_s_.reset();
        generation_.reset();
        candidate_.reset();
        curve_.reset();
    }

private:
    struct Candidate
    {
        int from_id;
        int to_id;
        double switch_s;
        int seen;
    };

    struct Curve
    {
        double start_s;
        double end_s;
        double duration_s;
        int from_id;
        int to_id;
        std::array<double, 6> coefficient;

        double position(double time_s) const
        {
            const double t = std::clamp((time_s - start_s) / duration_s, 0.0, 1.0);
            double value = coefficient[5];
            for (int i = 4; i >= 0; --i) value = value * t + coefficient[i];
            return value;
        }

        double velocity(double time_s) const
        {
            const double t = std::clamp((time_s - start_s) / duration_s, 0.0, 1.0);
            return (coefficient[1] + t * (2.0 * coefficient[2] +
                t * (3.0 * coefficient[3] + t * (4.0 * coefficient[4] +
                t * 5.0 * coefficient[5])))) / duration_s;
        }

        double acceleration(double time_s) const
        {
            const double t = std::clamp((time_s - start_s) / duration_s, 0.0, 1.0);
            return (2.0 * coefficient[2] + t * (6.0 * coefficient[3] +
                t * (12.0 * coefficient[4] + t * 20.0 * coefficient[5]))) /
                (duration_s * duration_s);
        }
    };

    std::optional<Curve> fit_curve(
        const auto_aim::SwitchPreview & preview, int start, int end,
        double now_s, int from_id, int to_id) const
    {
        const auto & yaw = preview.yaw_rad;
        const auto delta = [](double next, double previous) {
            return tools::limit_rad(next - previous);
        };
        const double p0 = yaw[start];
        const double p1 = p0 + delta(yaw[end], p0);
        // 两端都只取各自装甲板上的点；四点公式在端点估计二阶导数。
        const auto forward_velocity = [&](int step) {
            return (4.0 * delta(yaw[step + 1], yaw[step]) -
                    delta(yaw[step + 2], yaw[step])) /
                   (2.0 * auto_aim::DT);
        };
        const auto forward_acceleration = [&](int step) {
            return (-5.0 * delta(yaw[step + 1], yaw[step]) +
                     4.0 * delta(yaw[step + 2], yaw[step]) -
                           delta(yaw[step + 3], yaw[step])) /
                   (auto_aim::DT * auto_aim::DT);
        };
        const double v0 = forward_velocity(start);
        const double v1 = forward_velocity(end);
        const double start_acc = forward_acceleration(start);
        const double end_acc = forward_acceleration(end);
        const double duration = (end - start) * auto_aim::DT;
        const double c0 = p0;
        const double c1 = v0 * duration;
        const double c2 = 0.5 * start_acc * duration * duration;
        const double displacement = p1 - c0 - c1 - c2;
        const double velocity_delta = v1 * duration - c1 - 2.0 * c2;
        const double acceleration_delta =
            end_acc * duration * duration - 2.0 * c2;
        Curve curve{
            now_s + start * auto_aim::DT,
            now_s + end * auto_aim::DT,
            duration, from_id, to_id,
            {c0, c1, c2,
             10.0 * displacement - 4.0 * velocity_delta +
                 0.5 * acceleration_delta,
             -15.0 * displacement + 7.0 * velocity_delta -
                 acceleration_delta,
             6.0 * displacement - 3.0 * velocity_delta +
                 0.5 * acceleration_delta}};
        // 连续采样整段，不让数学上连续的曲线在 TX 周期内越过速度/加速度限值。
        for (int sample = 0; sample <= 240; ++sample) {
            const double at = curve.start_s + duration * sample / 240.0;
            if (!std::isfinite(curve.position(at)) ||
                !std::isfinite(curve.velocity(at)) ||
                !std::isfinite(curve.acceleration(at)) ||
                std::abs(curve.velocity(at)) > max_speed_ ||
                std::abs(curve.acceleration(at)) > max_acc_) {
                return std::nullopt;
            }
        }
        return curve;
    }

    double max_speed_;
    double max_acc_;
    std::optional<double> last_time_s_;
    std::optional<std::uint64_t> generation_;
    std::optional<Candidate> candidate_;
    std::optional<Curve> curve_;
};
}  // namespace real_auto_aim
