#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace calibration
{
using PoseClock = std::chrono::steady_clock;
constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;

// 反馈已是云台姿态：右手系 ZYX，角度制；返回把云台向量转换到 world 的旋转。
inline Eigen::Quaterniond gimbal_ypr_degrees(double yaw, double pitch, double roll)
{
    if (!std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(roll)) {
        throw std::invalid_argument("云台反馈角度必须为有限数值");
    }
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw * kRadiansPerDegree, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch * kRadiansPerDegree, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll * kRadiansPerDegree, Eigen::Vector3d::UnitX()));
}

// 以下时间均属于主机 steady_clock，不能当作下位机采样或相机曝光时间。
struct TimedGimbalPose
{
    PoseClock::time_point received_at;
    Eigen::Quaterniond q;
};

// 门限用于拒收不稳定/缺数据的静态标定样本，不作为固定延迟补偿。
struct PosePolicy
{
    std::size_t capacity = 256;  // 满时丢弃最旧姿态，避免检测变慢导致缓存无限增长。
    std::chrono::milliseconds stable_window{500};
    std::chrono::milliseconds max_gap{100};
    double max_spread_deg = 0.5;

    void validate() const
    {
        if (capacity < 3 || stable_window.count() <= 0 || max_gap.count() <= 0 ||
            max_gap > stable_window || !std::isfinite(max_spread_deg) ||
            max_spread_deg <= 0.0 || max_spread_deg >= 180.0) {
            throw std::invalid_argument("姿态缓存/稳定窗口/间隔/角度门限配置非法");
        }
    }
};

// 保存匹配质量，方便区分角度变化和主机接收延迟。
struct PoseMatch
{
    Eigen::Quaterniond q;
    double bracket_ms = 0.0;
    double spread_deg = 0.0;
};

// 纯逻辑缓存，不拥有串口和线程。调用方负责串行调用或在外部加锁。
class GimbalPoseBuffer
{
public:
    explicit GimbalPoseBuffer(PosePolicy policy = {}) : policy_(policy)
    {
        policy_.validate();
    }

    void push(TimedGimbalPose pose)
    {
        if (!pose.q.coeffs().allFinite() || std::abs(pose.q.norm() - 1.0) > 0.01) {
            throw std::invalid_argument("无效的云台四元数");
        }
        if (!samples_.empty() && pose.received_at <= samples_.back().received_at) {
            throw std::invalid_argument("姿态接收时间必须严格递增");
        }
        pose.q.normalize();
        samples_.push_back(std::move(pose));
        if (samples_.size() > policy_.capacity) samples_.pop_front();
    }

    std::size_t size() const { return samples_.size(); }
    bool reaches(PoseClock::time_point timestamp) const
    {
        return !samples_.empty() && samples_.back().received_at >= timestamp;
    }

    // 需要覆盖图像前 stable_window 及图像后的一个样本；不外推、不拿最新角度硬配旧图。
    PoseMatch match(PoseClock::time_point image_time, PoseClock::time_point now) const
    {
        if (samples_.empty()) throw std::runtime_error("尚未收到串口姿态");
        if (now < image_time || now < samples_.back().received_at ||
            now - samples_.back().received_at > policy_.max_gap) {
            throw std::runtime_error("串口姿态已过期或时间无效");
        }
        const auto begin_time = image_time - policy_.stable_window;
        auto begin = std::upper_bound(samples_.begin(), samples_.end(), begin_time,
            [](auto t, const TimedGimbalPose & p) { return t < p.received_at; });
        if (begin == samples_.begin()) throw std::runtime_error("稳定窗口不足，请停稳后再保存");
        --begin;
        auto after = std::lower_bound(samples_.begin(), samples_.end(), image_time,
            [](const TimedGimbalPose & p, auto t) { return p.received_at < t; });
        if (after == samples_.end() || after == samples_.begin()) {
            throw std::runtime_error("缺少图像两侧的串口姿态");
        }
        auto before = std::prev(after);
        const double interval = std::chrono::duration<double>(
            after->received_at - before->received_at).count();
        const double ratio = std::chrono::duration<double>(
            image_time - before->received_at).count() / interval;
        PoseMatch result{before->q.slerp(ratio, after->q).normalized(), interval * 1000.0, 0.0};
        for (auto it = begin; it != std::next(after); ++it) {
            if (it != begin && it->received_at - std::prev(it)->received_at > policy_.max_gap) {
                throw std::runtime_error("稳定窗口内串口反馈间隔过大");
            }
            result.spread_deg = std::max(
                result.spread_deg, result.q.angularDistance(it->q) / kRadiansPerDegree);
        }
        if (result.spread_deg > policy_.max_spread_deg) {
            throw std::runtime_error("云台仍在运动，请停稳后再保存");
        }
        return result;
    }

private:
    PosePolicy policy_;
    std::deque<TimedGimbalPose> samples_;
};
}  // namespace calibration
