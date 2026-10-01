#pragma once

#include <cmath>

namespace auto_aim::tracker_time_policy
{
// 只用于记录异常时间间隔，不直接改变 Tracker 状态。
inline constexpr double large_dt_warning_seconds = 0.1;

// 最大预测时间是失联保护边界，不是人为添加的预测延迟。
// 在边界以内由 EKF 使用真实 dt 和协方差继续预测；超过边界才允许重建目标。
inline bool exceeds_max_prediction_gap(double dt, double max_prediction_gap)
{
  return std::isfinite(dt) && std::isfinite(max_prediction_gap) &&
         dt > max_prediction_gap;
}
}  // namespace auto_aim::tracker_time_policy
