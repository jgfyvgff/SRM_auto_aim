#ifndef IO__HIKROBOT__DEVICE_CLOCK_MAPPER_HPP
#define IO__HIKROBOT__DEVICE_CLOCK_MAPPER_HPP

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace io
{
// 设备计数器给出帧间真实时间；主机绝对偏移只能从最短收帧延迟估算，不能消除固定传输延迟。
class DeviceClockMapper
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr std::size_t kWarmupFrames = 16;

  explicit DeviceClockMapper(std::uint64_t frequency_hz) : frequency_hz_(frequency_hz) {}

  std::optional<Clock::time_point> observe(
    std::uint64_t ticks, Clock::time_point received_at)
  {
    if (frequency_hz_ == 0 || ticks == 0) {
      started_ = false;
      return std::nullopt;
    }
    if (!started_ || ticks <= last_ticks_) {
      // 相机重启、计数回绕或重复帧后重新估计偏移，不跨设备时钟纪元插值。
      started_ = true;
      first_ticks_ = last_ticks_ = ticks;
      anchor_ = received_at;
      first_received_at_ = received_at;
      samples_ = 1;
      return std::nullopt;
    }
    last_ticks_ = ticks;
    const long double elapsed_ns =
      static_cast<long double>(ticks - first_ticks_) * 1'000'000'000.0L /
      static_cast<long double>(frequency_hz_);
    if (!std::isfinite(elapsed_ns) ||
        elapsed_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      started_ = false;
      return std::nullopt;
    }
    const auto elapsed = std::chrono::nanoseconds(static_cast<std::int64_t>(elapsed_ns));
    auto mapped = anchor_ + elapsed;
    if (samples_ < kWarmupFrames) {
      // 预热期取收帧时间差的最小值作为偏移上界，丢弃预热帧避免偏移更新造成时间倒跳。
      if (mapped > received_at) {
        anchor_ -= mapped - received_at;
        mapped = received_at;
      }
      ++samples_;
      if (samples_ < kWarmupFrames) return std::nullopt;
      const long double host_elapsed_ns =
        std::chrono::duration<long double, std::nano>(received_at - first_received_at_).count();
      // 如果设备计数增长与主机帧间隔明显不符，节点频率可能不属于该帧时标。
      if (host_elapsed_ns <= 0 || elapsed_ns < host_elapsed_ns * 0.75L ||
          elapsed_ns > host_elapsed_ns * 1.25L) {
        started_ = false;
        return std::nullopt;
      }
    }
    if (mapped > received_at) {
      started_ = false;
      return std::nullopt;
    }
    return mapped;
  }

private:
  std::uint64_t frequency_hz_ = 0;
  std::uint64_t first_ticks_ = 0;
  std::uint64_t last_ticks_ = 0;
  Clock::time_point anchor_{};
  Clock::time_point first_received_at_{};
  std::size_t samples_ = 0;
  bool started_ = false;
};
}  // namespace io

#endif  // IO__HIKROBOT__DEVICE_CLOCK_MAPPER_HPP
