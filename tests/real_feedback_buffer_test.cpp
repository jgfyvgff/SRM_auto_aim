#include "src/real_auto_aim/feedback_buffer.hpp"

#include <iostream>
#include <cmath>
#include <stdexcept>

namespace
{
void require(bool condition, const char * message)
{
    if (!condition) throw std::runtime_error(message);
}
}

int main()
{
    using namespace std::chrono_literals;
    using real_auto_aim::Clock;
    using real_auto_aim::FeedbackBuffer;
    const auto t0 = Clock::now();
    FeedbackBuffer buffer(3, 100ms);

    buffer.push(t0, {170.0F, 0.0F, 0.0F, 0, 3, 20.0F});
    buffer.push(t0 + 20ms, {-170.0F, 0.0F, 0.0F, 0, 3, 22.0F});
    const auto matched = buffer.match(t0 + 10ms, t0 + 20ms);
    require(matched.has_value(), "Fresh bracket was rejected");
    // 跨越 ±180° 时必须走短弧；中点朝向约为 180°。
    const Eigen::Vector3d forward = matched->gimbal_to_world * Eigen::Vector3d::UnitX();
    require(forward.x() < -0.99, "Yaw interpolation took the long arc");
    const auto motion = buffer.latest_motion(t0 + 20ms);
    require(motion.has_value(), "Fresh motion was rejected");
    require(std::abs(motion->yaw_vel_rad_s - 17.4532925199433) < 0.001,
            "Yaw velocity did not unwrap across 180 degrees");
    require(std::abs(motion->yaw_rad + 170.0 * calibration::kRadiansPerDegree) < 0.001,
            "Latest yaw was not converted to radians");
    require(!buffer.latest_motion(t0 + 200ms).has_value(),
            "Stale motion was accepted");
    require(!buffer.match(t0 + 10ms, t0 + 200ms).has_value(),
            "Stale feedback was accepted");

    buffer.push(t0 + 40ms, {0.0F, 0.0F, 0.0F, 0, 3, 20.0F});
    buffer.push(t0 + 60ms, {0.0F, 0.0F, 0.0F, 0, 3, 20.0F});
    require(buffer.size() == 3, "Buffer exceeded capacity");
    require(!buffer.match(t0 + 10ms, t0 + 60ms).has_value(),
            "Evicted feedback was still matched");

    FeedbackBuffer bursty(8, 100ms);
    bursty.push(t0, {0.0F, 0.0F, 0.0F, 0, 3, 20.0F});
    bursty.push(t0 + 40ms, {4.0F, 0.0F, 0.0F, 0, 3, 20.0F});
    bursty.push(t0 + 40ms + 500us, {4.05F, 0.0F, 0.0F, 0, 3, 20.0F});
    // 串口突发包中最近两帧仅隔 0.5 ms，速度应使用窗口内的较长基线。
    const auto burst_motion = bursty.latest_motion(t0 + 40ms + 500us);
    require(burst_motion.has_value(), "Burst feedback lost usable motion baseline");
    require(std::abs(burst_motion->interval_ms - 40.5) < 0.01,
            "Burst motion did not use oldest fresh feedback");
    require(std::abs(burst_motion->yaw_vel_rad_s -
                     4.05 * calibration::kRadiansPerDegree / 0.0405) < 0.01,
            "Burst motion angular velocity used a short interval");
    require(!bursty.latest_motion(t0 + 200ms).has_value(),
            "Stale burst motion was accepted");
    std::cout << "real_feedback_buffer_test passed\n";
}
