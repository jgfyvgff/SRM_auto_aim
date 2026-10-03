#include "io/hikrobot/device_clock_mapper.hpp"
#include "tools/thread_safe_queue.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace
{
void require(bool condition, const char * message)
{
    if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main()
{
    using namespace std::chrono_literals;
    using Clock = io::DeviceClockMapper::Clock;
    const auto start = Clock::time_point(1s);
    io::DeviceClockMapper mapper(1'000'000);

    for (std::size_t index = 0; index < io::DeviceClockMapper::kWarmupFrames; ++index) {
        // 帧的设备时钟以 10 ms 增长；主机传输延迟含有 0 或 2 ms 抖动。
        const auto delay = (index % 3 == 0) ? 3ms : 5ms;
        const auto mapped = mapper.observe(
            1'000'000 + index * 10'000, start + index * 10ms + delay);
        require(mapped.has_value() ==
                    (index + 1 == io::DeviceClockMapper::kWarmupFrames),
                "Warmup published an unstable clock offset");
    }
    const auto next = mapper.observe(1'160'000, start + 160ms + 7ms);
    require(next.has_value(), "Valid device frame was rejected");
    require(*next == start + 163ms, "Host jitter changed device-based frame spacing");
    require(!mapper.observe(1'160'000, start + 170ms).has_value(),
            "Repeated device timestamp was accepted");
    require(!mapper.observe(1'170'000, start + 173ms).has_value(),
            "Reset clock bypassed warmup");
    io::DeviceClockMapper unknown_frequency(0);
    require(!unknown_frequency.observe(1'000'000, start).has_value(),
            "Missing tick frequency silently fell back to host time");
    io::DeviceClockMapper wrong_frequency(2'000'000);
    for (std::size_t index = 0; index < io::DeviceClockMapper::kWarmupFrames; ++index) {
        const auto mapped = wrong_frequency.observe(
            1'000'000 + index * 10'000, start + index * 10ms);
        require(!mapped.has_value(), "Mismatched frame tick frequency was accepted");
    }
    tools::ThreadSafeQueue<int, true> latest_frame(1);
    latest_frame.push(1);
    latest_frame.push(2);
    require(latest_frame.pop() == 2, "Camera queue did not drop the stale frame");
    std::cout << "device_clock_mapper_test passed\n";
}
