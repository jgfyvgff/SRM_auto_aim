#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace sim_capture_time
{
// 将模拟器的系统时钟采集时间映射到 Tracker/Aimer 使用的单调时钟。
struct CaptureTime
{
  std::chrono::steady_clock::time_point timestamp;
  double header_age_ms;
};

inline std::optional<CaptureTime> convert_system_stamp(
  std::int32_t sec, std::uint32_t nanosec,
  std::chrono::system_clock::time_point system_now,
  std::chrono::steady_clock::time_point steady_now)
{
  constexpr std::chrono::seconds max_valid_age{1};
  if (sec <= 0 || nanosec >= 1000000000U) {
    return std::nullopt;
  }

  const auto stamp_duration =
    std::chrono::seconds{sec} + std::chrono::nanoseconds{nanosec};
  const auto capture_wall = std::chrono::system_clock::time_point{
    std::chrono::duration_cast<std::chrono::system_clock::duration>(stamp_duration)};
  const auto age = system_now - capture_wall;
  // 一秒是过期帧拒绝上限，不是用于预测的固定延迟。
  if (age < std::chrono::system_clock::duration::zero() || age > max_valid_age) {
    return std::nullopt;
  }

  return CaptureTime{
    steady_now - std::chrono::duration_cast<std::chrono::steady_clock::duration>(age),
    std::chrono::duration<double, std::milli>(age).count()};
}
}  // namespace sim_capture_time
