#include <cassert>
#include <cmath>
#include <limits>

#include "tasks/auto_aim/tracker_time_policy.hpp"

int main()
{
  using auto_aim::tracker_time_policy::exceeds_max_prediction_gap;

  // 当前实测的 100~110 ms 单次图像间隔仍属于短时采集抖动，不应重建 Tracker。
  assert(!exceeds_max_prediction_gap(0.108, 0.3));
  assert(!exceeds_max_prediction_gap(0.3, 0.3));

  // 明显超过最大可信预测时间后，才进入失联保护路径。
  assert(exceeds_max_prediction_gap(0.301, 0.3));
  assert(!exceeds_max_prediction_gap(std::numeric_limits<double>::quiet_NaN(), 0.3));
  return 0;
}
