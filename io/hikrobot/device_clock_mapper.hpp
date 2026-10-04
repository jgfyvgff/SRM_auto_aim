#ifndef IO__HIKROBOT__DEVICE_CLOCK_MAPPER_HPP
#define IO__HIKROBOT__DEVICE_CLOCK_MAPPER_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
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
  static constexpr auto kEstimatedWarmup = std::chrono::seconds(2);
  static constexpr auto kEstimateWindow = std::chrono::seconds(5);
  static constexpr auto kEstimateRefresh = std::chrono::seconds(1);
  static constexpr auto kMaxPhaseStep = std::chrono::milliseconds(5);
  static constexpr std::size_t kMaxEstimateSamples = 4096;

  // frequency_hz 为 0 时，可根据连续设备 tick 与主机收帧间隔估计时钟频率。
  // 估计结果只用于相对时序匹配，不能当作相机曝光时刻或硬件同步结果。
  explicit DeviceClockMapper(
    std::uint64_t frequency_hz, bool allow_frequency_estimation = false)
  : configured_frequency_hz_(frequency_hz),
    frequency_hz_(frequency_hz),
    allow_frequency_estimation_(allow_frequency_estimation)
  {
  }

  std::uint64_t frequency_hz() const { return frequency_hz_; }

  bool using_estimated_frequency() const
  {
    return allow_frequency_estimation_ && configured_frequency_hz_ == 0 && frequency_hz_ > 0;
  }

  std::optional<Clock::time_point> observe(
    std::uint64_t ticks, Clock::time_point received_at)
  {
    if (ticks == 0) {
      reset();
      return std::nullopt;
    }
    if (configured_frequency_hz_ == 0 && allow_frequency_estimation_) {
      return observe_estimated(ticks, received_at);
    }
    if (!started_ || ticks <= last_ticks_ || received_at <= last_received_at_) {
      // 相机重启、计数回绕或重复帧后重新估计偏移，不跨设备时钟纪元插值。
      start_epoch(ticks, received_at);
      return std::nullopt;
    }
    last_ticks_ = ticks;
    last_received_at_ = received_at;
    if (frequency_hz_ == 0) return std::nullopt;

    const long double elapsed_ns =
      static_cast<long double>(ticks - first_ticks_) * 1'000'000'000.0L /
      static_cast<long double>(frequency_hz_);
    if (!std::isfinite(elapsed_ns) ||
        elapsed_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      reset();
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
        reset();
        return std::nullopt;
      }
    }
    if (mapped > received_at) {
      reset();
      return std::nullopt;
    }
    return mapped;
  }

private:
  struct WarmupSample
  {
    std::uint64_t ticks = 0;
    Clock::time_point received_at{};
  };

  std::optional<Clock::time_point> observe_estimated(
    std::uint64_t ticks, Clock::time_point received_at)
  {
    if (!started_ || ticks <= last_ticks_ || received_at <= last_received_at_) {
      start_epoch(ticks, received_at);
      return std::nullopt;
    }
    last_ticks_ = ticks;
    last_received_at_ = received_at;
    estimate_samples_.push_back({ticks, received_at});
    // 仅保留最近五秒，且帧率异常时限制内存；估频按时间跨度而非帧数预热。
    while (estimate_samples_.size() > kMaxEstimateSamples ||
           received_at - estimate_samples_.front().received_at > kEstimateWindow) {
      estimate_samples_.pop_front();
    }
    if (estimate_samples_.size() < kWarmupFrames ||
        (frequency_hz_ == 0 &&
         received_at - estimate_samples_.front().received_at < kEstimatedWarmup)) {
      return std::nullopt;
    }

    if (frequency_hz_ == 0 || received_at >= next_estimate_at_) {
      const auto estimate = estimate_frequency();
      if (!estimate) {
        reset();
        return std::nullopt;
      }
      const auto [new_frequency, new_first_ticks, fitted_anchor] = *estimate;
      auto new_anchor = fitted_anchor;
      if (frequency_hz_ != 0) {
        const auto old_mapped = map_ticks(ticks, first_ticks_, frequency_hz_, anchor_);
        const auto new_mapped = map_ticks(ticks, new_first_ticks, new_frequency, new_anchor);
        if (!old_mapped || !new_mapped) {
          reset();
          return std::nullopt;
        }
        // 频率持续更新，但每次只允许小幅修正相位，避免目标时间突然倒跳。
        const auto phase_error = *new_mapped - *old_mapped;
        if (phase_error > kMaxPhaseStep) new_anchor -= phase_error - kMaxPhaseStep;
        if (phase_error < -kMaxPhaseStep) new_anchor -= phase_error + kMaxPhaseStep;
      }
      frequency_hz_ = new_frequency;
      first_ticks_ = new_first_ticks;
      anchor_ = new_anchor;
      next_estimate_at_ = received_at + kEstimateRefresh;
    }

    auto mapped = map_ticks(ticks, first_ticks_, frequency_hz_, anchor_);
    if (!mapped) {
      reset();
      return std::nullopt;
    }
    if (*mapped > received_at) {
      // 收帧时刻只是曝光时间的上界；轻微估频误差不能让映射跨到未来。
      anchor_ -= *mapped - received_at;
      mapped = received_at;
    }
    if (last_mapped_at_ && *mapped <= *last_mapped_at_) {
      mapped = *last_mapped_at_ + std::chrono::nanoseconds(1);
      if (*mapped > received_at) {
        reset();
        return std::nullopt;
      }
    }
    last_mapped_at_ = mapped;
    return mapped;
  }

  static std::optional<Clock::time_point> map_ticks(
    std::uint64_t ticks, std::uint64_t first_ticks, std::uint64_t frequency_hz,
    Clock::time_point anchor)
  {
    if (ticks < first_ticks || frequency_hz == 0) return std::nullopt;
    const long double elapsed_ns =
      static_cast<long double>(ticks - first_ticks) * 1'000'000'000.0L /
      static_cast<long double>(frequency_hz);
    if (!std::isfinite(elapsed_ns) ||
        elapsed_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      return std::nullopt;
    }
    return anchor + std::chrono::nanoseconds(static_cast<std::int64_t>(elapsed_ns));
  }

  struct FrequencyEstimate
  {
    std::uint64_t frequency_hz;
    std::uint64_t first_ticks;
    Clock::time_point anchor;
  };

  std::optional<FrequencyEstimate> estimate_frequency() const
  {
    if (estimate_samples_.size() < kWarmupFrames) return std::nullopt;
    const auto & first = estimate_samples_.front();
    const long double count = static_cast<long double>(estimate_samples_.size());
    long double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    for (const auto & sample : estimate_samples_) {
      const long double x = static_cast<long double>(sample.ticks - first.ticks);
      const long double y =
        std::chrono::duration<long double, std::nano>(sample.received_at - first.received_at).count();
      sum_x += x;
      sum_y += y;
      sum_xx += x * x;
      sum_xy += x * y;
    }
    // 用跨秒的最小二乘斜率平均 USB 收帧抖动；短窗口会把抖动误认为时钟频偏。
    const long double denominator = count * sum_xx - sum_x * sum_x;
    if (denominator <= 0) return std::nullopt;
    const long double slope = (count * sum_xy - sum_x * sum_y) / denominator;
    const long double frequency = 1'000'000'000.0L / slope;
    if (!std::isfinite(frequency) || frequency < 1.0L ||
        frequency > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      return std::nullopt;
    }
    const auto frequency_hz = static_cast<std::uint64_t>(std::llround(frequency));
    auto anchor = first.received_at;
    // 最早可能的曝光时间以最短收帧延迟为上界，仍无法识别固定 USB 传输延迟。
    for (const auto & sample : estimate_samples_) {
      const auto mapped = map_ticks(sample.ticks, first.ticks, frequency_hz, first.received_at);
      if (!mapped) return std::nullopt;
      anchor = std::min(anchor, first.received_at + (sample.received_at - *mapped));
    }
    return FrequencyEstimate{frequency_hz, first.ticks, anchor};
  }

  void start_epoch(std::uint64_t ticks, Clock::time_point received_at)
  {
    started_ = true;
    first_ticks_ = last_ticks_ = ticks;
    last_received_at_ = received_at;
    anchor_ = received_at;
    first_received_at_ = received_at;
    samples_ = 1;
    estimate_samples_.clear();
    estimate_samples_.push_back({ticks, received_at});
    last_mapped_at_.reset();
    next_estimate_at_ = {};
    if (configured_frequency_hz_ == 0) frequency_hz_ = 0;
  }

  void reset()
  {
    started_ = false;
    first_ticks_ = last_ticks_ = 0;
    last_received_at_ = {};
    anchor_ = {};
    first_received_at_ = {};
    samples_ = 0;
    estimate_samples_.clear();
    last_mapped_at_.reset();
    next_estimate_at_ = {};
    if (configured_frequency_hz_ == 0) frequency_hz_ = 0;
  }

  const std::uint64_t configured_frequency_hz_ = 0;
  std::uint64_t first_ticks_ = 0;
  std::uint64_t last_ticks_ = 0;
  std::uint64_t frequency_hz_ = 0;
  Clock::time_point anchor_{};
  Clock::time_point first_received_at_{};
  Clock::time_point last_received_at_{};
  Clock::time_point next_estimate_at_{};
  std::optional<Clock::time_point> last_mapped_at_;
  std::size_t samples_ = 0;
  bool started_ = false;
  bool allow_frequency_estimation_ = false;
  std::deque<WarmupSample> estimate_samples_;
};
}  // namespace io

#endif  // IO__HIKROBOT__DEVICE_CLOCK_MAPPER_HPP
