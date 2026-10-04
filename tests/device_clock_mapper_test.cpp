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
    io::DeviceClockMapper estimated_frequency(0, true);
    for (std::size_t index = 0; index <= 200; ++index) {
        const auto delay = (index % 3 == 0) ? 3ms : 5ms;
        const auto mapped = estimated_frequency.observe(
            100'000'000 + index * 1'000'000, start + index * 10ms + delay);
        require(mapped.has_value() == (index == 200),
                "Estimated device clock published before the two-second warmup");
    }
    require(estimated_frequency.using_estimated_frequency(),
            "Missing frequency was not marked as estimated");
    require(estimated_frequency.frequency_hz() > 0,
            "Estimated device clock frequency is invalid");
    require(estimated_frequency.observe(301'000'000, start + 2010ms + 7ms).has_value(),
            "Estimated device clock rejected a valid frame");
    // 早期 16 帧的主机抖动足以造成较大频偏；长时间跟踪不能冻结该频偏并逐渐过期。
    io::DeviceClockMapper long_running(0, true);
    std::size_t valid_frames = 0;
    for (std::size_t index = 0; index <= 6000; ++index) {
        const auto delay = (index % 17 == 0) ? 8ms : 3ms;
        const auto received = start + index * 10ms + delay;
        const auto mapped = long_running.observe(100'000'000 + index * 1'000'000, received);
        if (index < 201) {
            require(!mapped, "Long-running estimate bypassed warmup");
            continue;
        }
        require(mapped.has_value(), "Long-running device clock stopped mapping frames");
        require(*mapped <= received, "Estimated timestamp is later than host receive");
        require(received - *mapped < 30ms, "Estimated clock drifted beyond 30 ms");
        ++valid_frames;
    }
    require(valid_frames > 5700, "Too few long-running frames were mapped");
    require(long_running.frequency_hz() > 99'900'000 &&
                long_running.frequency_hz() < 100'100'000,
            "Long-running frequency did not converge to the device tick rate");
    io::DeviceClockMapper early_bias(0, true);
    for (std::size_t index = 0; index <= 6000; ++index) {
        // 初始两秒的收帧延迟缓慢增加，会让首次拟合产生频偏；之后恢复正常。
        const auto bias = index <= 200 ? std::chrono::microseconds(index * 50) :
                          index <= 400 ? std::chrono::microseconds((400 - index) * 50) : 0us;
        const auto received = start + index * 10ms + 3ms + bias;
        const auto mapped = early_bias.observe(100'000'000 + index * 1'000'000, received);
        if (index < 200) continue;
        require(mapped.has_value(), "Initial timing bias caused a permanent clock reset");
        if (index > 1000) {
            require(received - *mapped < 30ms,
                    "Initial timing bias accumulated into a stale timestamp");
        }
    }
    require(early_bias.frequency_hz() > 99'900'000 &&
                early_bias.frequency_hz() < 100'100'000,
            "Clock frequency did not recover from initial receive-time bias");
    require(!long_running.observe(1, start + 61s).has_value(),
            "Device counter regression crossed clock epochs");
    require(!long_running.using_estimated_frequency(),
            "Old frequency survived a device counter reset");
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
