#include <chrono>
#include <cmath>
#include <iostream>

#include "src/sim_capture_time.hpp"

int main()
{
  using namespace std::chrono;
  const auto wall_now =
    system_clock::time_point{} + seconds{1800000000} + milliseconds{29};
  const auto steady_now = steady_clock::time_point{} + seconds{100};
  const auto valid = sim_capture_time::convert_system_stamp(
    1800000000, 0, wall_now, steady_now);
  if (
    !valid || valid->timestamp != steady_now - milliseconds{29} ||
    std::abs(valid->header_age_ms - 29.0) > 1e-6)
  {
    std::cerr << "有效采集时间映射失败\n";
    return 1;
  }

  if (
    sim_capture_time::convert_system_stamp(0, 0, wall_now, steady_now) ||
    sim_capture_time::convert_system_stamp(
      1800000000, 1000000000U, wall_now, steady_now) ||
    sim_capture_time::convert_system_stamp(
      1800000001, 0, wall_now, steady_now) ||
    sim_capture_time::convert_system_stamp(
      1799999998, 0, wall_now, steady_now))
  {
    std::cerr << "无效、未来或过期时间戳未被拒绝\n";
    return 1;
  }
  return 0;
}
