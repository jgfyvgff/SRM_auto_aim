#include "src/real_auto_aim/feedback_buffer.hpp"

#include <iostream>
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
    require(!buffer.match(t0 + 10ms, t0 + 200ms).has_value(),
            "Stale feedback was accepted");

    buffer.push(t0 + 40ms, {0.0F, 0.0F, 0.0F, 0, 3, 20.0F});
    buffer.push(t0 + 60ms, {0.0F, 0.0F, 0.0F, 0, 3, 20.0F});
    require(buffer.size() == 3, "Buffer exceeded capacity");
    require(!buffer.match(t0 + 10ms, t0 + 60ms).has_value(),
            "Evicted feedback was still matched");
    std::cout << "real_feedback_buffer_test passed\n";
}
