#ifndef AUTO_AIM__ENDPOINT_RANGE_SHADOW_HPP
#define AUTO_AIM__ENDPOINT_RANGE_SHADOW_HPP

#include <array>
#include <unordered_map>

#include "armor.hpp"

namespace auto_aim
{
// 回放专用的逐检测距离影子诊断，按稳定的 std::list 节点地址旁路传给 Tracker。
// 不把诊断字段塞进 Armor，避免改变真机检测对象布局和现有处理顺序。
struct EndpointRangeShadowDebug
{
  bool fit_found = false;
  bool reliable = false;
  bool selected = false;
  bool distance_replaced = false;
  bool update_accepted = false;
  std::array<cv::Point2f, 4> fitted_points{};
  double raw_left_length_px = -1.0;
  double raw_right_length_px = -1.0;
  double fitted_left_length_px = -1.0;
  double fitted_right_length_px = -1.0;
  double left_visible_fraction = -1.0;
  double right_visible_fraction = -1.0;
  double left_saturation_fraction = -1.0;
  double right_saturation_fraction = -1.0;
  double left_max_dark_gap_px = -1.0;
  double right_max_dark_gap_px = -1.0;
  double left_fitted_visible_fraction = -1.0;
  double right_fitted_visible_fraction = -1.0;
  double left_fitted_saturation_fraction = -1.0;
  double right_fitted_saturation_fraction = -1.0;
  double left_fitted_max_dark_gap_px = -1.0;
  double right_fitted_max_dark_gap_px = -1.0;
  double corner_shift_rms_px = -1.0;
  double raw_pnp_distance_m = -1.0;
  double fitted_pnp_distance_m = -1.0;
  double raw_reprojection_error_px = -1.0;
  double fitted_reprojection_error_px = -1.0;
  // 实际选中候选按同一 IPPE 分支重算的距离，便于核对送入 EKF 的值。
  double selected_raw_distance_m = -1.0;
  double selected_fitted_distance_m = -1.0;
};

using EndpointRangeShadowMap =
    std::unordered_map<const Armor *, EndpointRangeShadowDebug>;

}  // namespace auto_aim

#endif  // AUTO_AIM__ENDPOINT_RANGE_SHADOW_HPP
