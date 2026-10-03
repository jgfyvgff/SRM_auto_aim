#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iterator>
#include <optional>
#include <stdexcept>

#include "calibration/gimbal_pose.hpp"
#include "io/srm_auto_aim_protocol.hpp"

namespace real_auto_aim
{
using Clock = std::chrono::steady_clock;
using FeedbackFrame = io::srm_auto_aim::FeedbackFrame;

struct MatchedFeedback
{
    Eigen::Quaterniond gimbal_to_world;
    FeedbackFrame feedback;
    double bracket_ms = 0.0;
};

// 只保存主机接收时间；协议没有下位机采样时间，因此不声称匹配到曝光瞬间。
// 调用方负责加锁。本缓存满时丢弃最旧反馈，避免推理变慢时无限积压。
class FeedbackBuffer
{
public:
    explicit FeedbackBuffer(
        std::size_t capacity = 256,
        std::chrono::milliseconds max_gap = std::chrono::milliseconds(100))
        : capacity_(capacity), max_gap_(max_gap)
    {
        if (capacity_ < 2 || max_gap_.count() <= 0) {
            throw std::invalid_argument("Invalid feedback buffer policy");
        }
    }

    void push(Clock::time_point received_at, const FeedbackFrame & frame)
    {
        if (!samples_.empty() && received_at <= samples_.back().received_at) {
            throw std::invalid_argument("Feedback time must increase");
        }
        if (!std::isfinite(frame.bullet_speed_mps) || frame.bullet_speed_mps < 0.0F) {
            throw std::invalid_argument("Invalid bullet speed");
        }
        const auto q = calibration::gimbal_ypr_degrees(
            frame.yaw_deg, frame.pitch_deg, frame.roll_deg);
        samples_.push_back({received_at, frame, q});
        if (samples_.size() > capacity_) samples_.pop_front();
    }

    // 只有图像时间两侧都有足够新的反馈时才插值；否则返回空，禁止使用陈旧姿态。
    std::optional<MatchedFeedback> match(
        Clock::time_point image_time, Clock::time_point now) const
    {
        if (samples_.size() < 2 || now < image_time ||
            now < samples_.back().received_at ||
            now - samples_.back().received_at > max_gap_) {
            return std::nullopt;
        }
        const auto after = std::lower_bound(
            samples_.begin(), samples_.end(), image_time,
            [](const Sample & sample, Clock::time_point time) {
                return sample.received_at < time;
            });
        if (after == samples_.begin() || after == samples_.end()) {
            return std::nullopt;
        }
        const auto before = std::prev(after);
        const auto interval = after->received_at - before->received_at;
        if (interval > max_gap_ || image_time - before->received_at > max_gap_ ||
            after->received_at - image_time > max_gap_) {
            return std::nullopt;
        }
        const double ratio = std::chrono::duration<double>(
            image_time - before->received_at).count() /
            std::chrono::duration<double>(interval).count();
        const auto & nearest = ratio < 0.5 ? *before : *after;
        return MatchedFeedback{
            before->q.slerp(ratio, after->q).normalized(),
            nearest.frame,
            std::chrono::duration<double, std::milli>(interval).count()};
    }

    std::size_t size() const { return samples_.size(); }

    bool reaches(Clock::time_point time) const
    {
        return !samples_.empty() && samples_.back().received_at >= time;
    }

private:
    struct Sample
    {
        Clock::time_point received_at;
        FeedbackFrame frame;
        Eigen::Quaterniond q;
    };

    std::size_t capacity_;
    std::chrono::milliseconds max_gap_;
    std::deque<Sample> samples_;
};
}  // namespace real_auto_aim
