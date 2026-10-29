#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tasks/auto_aim/tracker_association_policy.hpp"
#include "tracker_time_policy.hpp"

namespace
{
using P0Diagonal = std::array<double, 11>;

P0Diagonal read_p0_diagonal(
  const YAML::Node & yaml, const char * key, const P0Diagonal & fallback)
{
  const auto node = yaml[key];
  if (!node.IsDefined()) return fallback;
  if (!node.IsSequence() || node.size() != fallback.size()) {
    throw std::runtime_error(std::string("Invalid ") + key + ": expected 11 diagonal values");
  }

  P0Diagonal values{};
  for (std::size_t index = 0; index < values.size(); ++index) {
    values[index] = node[index].as<double>();
    if (!std::isfinite(values[index]) || values[index] < 0.0) {
      throw std::runtime_error(std::string("Invalid ") + key + ": values must be finite and non-negative");
    }
  }
  return values;
}

Eigen::VectorXd to_eigen_vector(const P0Diagonal & values)
{
  Eigen::VectorXd result(static_cast<Eigen::Index>(values.size()));
  for (Eigen::Index index = 0; index < result.size(); ++index) {
    result[index] = values[static_cast<std::size_t>(index)];
  }
  return result;
}
}  // namespace

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}//哨兵
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();//连续检测到目标的最小次数
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();//连续丢失目标的最大次数
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();//前哨站连续丢失目标的最大次数
  normal_temp_lost_count_ = max_temp_lost_count_;//普通模式连续丢失目标的最大次数
  balance_initial_covariance_diagonal_ = read_p0_diagonal(
    yaml, "balance_initial_covariance_diagonal", P0Diagonal{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1});
  outpost_initial_covariance_diagonal_ = read_p0_diagonal(
    yaml, "outpost_initial_covariance_diagonal", P0Diagonal{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0});
  base_initial_covariance_diagonal_ = read_p0_diagonal(
    yaml, "base_initial_covariance_diagonal", P0Diagonal{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0});

  standard_geometry_constraint_ = yaml["standard_geometry_constraint"].IsDefined()
                                    ? yaml["standard_geometry_constraint"].as<bool>()
                                    : false;
  standard_radius_delta_constraint_ =
    yaml["standard_radius_delta_constraint"].IsDefined()
      ? yaml["standard_radius_delta_constraint"].as<bool>()
      : false;
  standard_radius_ =
    yaml["standard_radius"].IsDefined() ? yaml["standard_radius"].as<double>() : 0.18;
  standard_radius_variance_ = yaml["standard_radius_variance"].IsDefined()
                                ? yaml["standard_radius_variance"].as<double>()
                                : 1.0;
  standard_radius_delta_variance_ = yaml["standard_radius_delta_variance"].IsDefined()
                                      ? yaml["standard_radius_delta_variance"].as<double>()
                                      : 1.0;
  standard_height_delta_variance_ = yaml["standard_height_delta_variance"].IsDefined()
                                      ? yaml["standard_height_delta_variance"].as<double>()
                                      : 1.0;
  standard_initial_position_variance_ = yaml["standard_initial_position_variance"].IsDefined()
                                         ? yaml["standard_initial_position_variance"].as<double>()
                                         : 1.0;
  standard_initial_center_yaw_variance_ = yaml["standard_initial_center_yaw_variance"].IsDefined()
                                            ? yaml["standard_initial_center_yaw_variance"].as<double>()
                                            : 0.4;
  standard_initial_velocity_variance_ =
    yaml["standard_initial_velocity_variance"].IsDefined()
      ? yaml["standard_initial_velocity_variance"].as<double>()
      : 64.0;
  standard_initial_angular_velocity_variance_ =
    yaml["standard_initial_angular_velocity_variance"].IsDefined()
      ? yaml["standard_initial_angular_velocity_variance"].as<double>()
      : 0.1;
  standard_capture_bootstrap_updates_ =
    yaml["standard_capture_bootstrap_updates"].IsDefined()
      ? yaml["standard_capture_bootstrap_updates"].as<int>()
      : 0;
  standard_translation_acceleration_variance_ =
    yaml["standard_translation_acceleration_variance"].IsDefined()
      ? yaml["standard_translation_acceleration_variance"].as<double>()
      : 100.0;
  standard_angular_acceleration_variance_ =
    yaml["standard_angular_acceleration_variance"].IsDefined()
      ? yaml["standard_angular_acceleration_variance"].as<double>()
      : 100.0;
  balance_translation_acceleration_variance_ = yaml["balance_translation_acceleration_variance"].IsDefined()
                                                  ? yaml["balance_translation_acceleration_variance"].as<double>()
                                                  : 100.0;
  balance_angular_acceleration_variance_ = yaml["balance_angular_acceleration_variance"].IsDefined()
                                              ? yaml["balance_angular_acceleration_variance"].as<double>()
                                              : 100.0;
  outpost_translation_acceleration_variance_ = yaml["outpost_translation_acceleration_variance"].IsDefined()
                                                  ? yaml["outpost_translation_acceleration_variance"].as<double>()
                                                  : 10.0;
  outpost_angular_acceleration_variance_ = yaml["outpost_angular_acceleration_variance"].IsDefined()
                                              ? yaml["outpost_angular_acceleration_variance"].as<double>()
                                              : 0.1;
  base_translation_acceleration_variance_ = yaml["base_translation_acceleration_variance"].IsDefined()
                                               ? yaml["base_translation_acceleration_variance"].as<double>()
                                               : 100.0;
  base_angular_acceleration_variance_ = yaml["base_angular_acceleration_variance"].IsDefined()
                                           ? yaml["base_angular_acceleration_variance"].as<double>()
                                           : 100.0;
  if (yaml["standard_max_angular_velocity"].IsDefined()) {
    standard_max_angular_velocity_ = yaml["standard_max_angular_velocity"].as<double>();
  }
  association_max_angle_error_ = yaml["association_max_angle_error"].IsDefined()
                                   ? yaml["association_max_angle_error"].as<double>()
                                   : std::numeric_limits<double>::infinity();
  association_max_score_ = yaml["association_max_score"].IsDefined()
                             ? yaml["association_max_score"].as<double>()
                             : 0.8;
  association_id_switch_score_margin_ =
    yaml["association_id_switch_score_margin"].IsDefined()
      ? yaml["association_id_switch_score_margin"].as<double>()
      : 0.0;
  association_max_position_error_ = yaml["association_max_position_error"].IsDefined()
                                      ? yaml["association_max_position_error"].as<double>()
                                      : 0.45;
  association_max_distance_error_ = yaml["association_max_distance_error"].IsDefined()
                                      ? yaml["association_max_distance_error"].as<double>()
                                      : 0.45;
  association_max_mahalanobis_distance_ =
    yaml["association_max_mahalanobis_distance"].IsDefined()
      ? yaml["association_max_mahalanobis_distance"].as<double>()
      : 3.2;
  // 默认 7 帧窗口内同类出现 5 次才改判类别：远距离分类是边缘的（实测同一块装甲板在
  // two / outpost 之间逐帧跳），单帧证据不足以定类别。参数做边界收敛，避免配出
  // "窗口 0 帧"或"要求票数多于窗口"这类永远不成立的条件。
  relock_name_window_ =
    yaml["relock_name_window"].IsDefined() ? yaml["relock_name_window"].as<int>() : 7;
  relock_name_votes_ =
    yaml["relock_name_votes"].IsDefined() ? yaml["relock_name_votes"].as<int>() : 5;
  if (relock_name_window_ < 1) relock_name_window_ = 1;
  if (relock_name_votes_ < 1) relock_name_votes_ = 1;
  if (relock_name_votes_ > relock_name_window_) relock_name_votes_ = relock_name_window_;
  measurement_bearing_variance_ =
    yaml["measurement_bearing_variance"].IsDefined()
      ? yaml["measurement_bearing_variance"].as<double>()
      : 4e-3;
  measurement_elevation_variance_ = yaml["measurement_elevation_variance"].IsDefined()
                                      ? yaml["measurement_elevation_variance"].as<double>()
                                      : 4e-3;
  measurement_distance_variance_log_scale_ = yaml["measurement_distance_variance_slope"].IsDefined()
                                               ? yaml["measurement_distance_variance_slope"].as<double>()
                                               : 1.0;
  measurement_distance_variance_offset_ = yaml["measurement_distance_variance_offset"].IsDefined()
                                           ? yaml["measurement_distance_variance_offset"].as<double>()
                                           : 1.0;
  measurement_armor_yaw_variance_log_scale_ = yaml["measurement_armor_yaw_variance_slope"].IsDefined()
                                                ? yaml["measurement_armor_yaw_variance_slope"].as<double>()
                                                : 0.005;
  measurement_armor_yaw_variance_offset_ = yaml["measurement_armor_yaw_variance_offset"].IsDefined()
                                             ? yaml["measurement_armor_yaw_variance_offset"].as<double>()
                                             : 9e-2;
  distance_update_r_scale_ = yaml["distance_update_r_scale"].IsDefined()
                               ? yaml["distance_update_r_scale"].as<double>()
                               : 1.0;
  yaw_update_sigma_deg_ = yaml["yaw_update_sigma_deg"].IsDefined()
                            ? yaml["yaw_update_sigma_deg"].as<double>()
                            : 0.0;
  max_prediction_gap_ = yaml["tracker_max_prediction_gap"].IsDefined()
                          ? yaml["tracker_max_prediction_gap"].as<double>()
                          : 0.3;

  if (
    standard_radius_ <= 0.05 || standard_radius_ >= 0.5 ||
    standard_radius_variance_ <= 0 || standard_radius_delta_variance_ <= 0 ||
    standard_height_delta_variance_ <= 0 ||
    !std::isfinite(standard_initial_position_variance_) || standard_initial_position_variance_ <= 0.0 ||
    !std::isfinite(standard_initial_center_yaw_variance_) ||
    standard_initial_center_yaw_variance_ <= 0.0 ||
    !std::isfinite(standard_initial_velocity_variance_) ||
    standard_initial_velocity_variance_ <= 0 ||
    !std::isfinite(standard_initial_angular_velocity_variance_) ||
    standard_initial_angular_velocity_variance_ <= 0)
  {
    throw std::runtime_error("Invalid standard armor geometry configuration");
  }
  if (standard_capture_bootstrap_updates_ < 0 || standard_capture_bootstrap_updates_ > 100) {
    throw std::runtime_error("Invalid standard_capture_bootstrap_updates");
  }
  if (
    !std::isfinite(standard_translation_acceleration_variance_) ||
    standard_translation_acceleration_variance_ <= 0.0)
  {
    throw std::runtime_error("Invalid standard_translation_acceleration_variance");
  }
  const std::array<double, 7> process_noise_variances{
    standard_angular_acceleration_variance_, balance_translation_acceleration_variance_,
    balance_angular_acceleration_variance_, outpost_translation_acceleration_variance_,
    outpost_angular_acceleration_variance_, base_translation_acceleration_variance_,
    base_angular_acceleration_variance_};
  if (std::any_of(process_noise_variances.begin(), process_noise_variances.end(),
      [](double value) { return !std::isfinite(value) || value <= 0.0; })) {
    throw std::runtime_error("Invalid target process noise variance configuration");
  }
  if (
    standard_max_angular_velocity_ &&
    (!std::isfinite(*standard_max_angular_velocity_) || *standard_max_angular_velocity_ <= 0.0))
  {
    throw std::runtime_error("Invalid standard_max_angular_velocity");
  }
  if (
    !std::isinf(association_max_angle_error_) &&
    (association_max_angle_error_ <= 0.0 || association_max_angle_error_ > 3.141592653589793))
  {
    throw std::runtime_error("Invalid association_max_angle_error configuration");
  }
  if (association_max_score_ <= 0.0 || association_max_score_ > 3.141592653589793 * 3.0) {
    throw std::runtime_error("Invalid association_max_score configuration");
  }
  if (
    !std::isfinite(association_id_switch_score_margin_) ||
    association_id_switch_score_margin_ < 0.0 ||
    association_id_switch_score_margin_ > association_max_score_)
  {
    throw std::runtime_error("Invalid association_id_switch_score_margin configuration");
  }
  if (
    !std::isfinite(association_max_position_error_) ||
    !std::isfinite(association_max_distance_error_) ||
    association_max_position_error_ <= 0.0 || association_max_distance_error_ <= 0.0)
  {
    throw std::runtime_error("Invalid association absolute error configuration");
  }
  if (
    !std::isfinite(association_max_mahalanobis_distance_) ||
    association_max_mahalanobis_distance_ <= 0.0)
  {
    throw std::runtime_error("Invalid association_max_mahalanobis_distance configuration");
  }
  if (!std::isfinite(measurement_bearing_variance_) ||
      measurement_bearing_variance_ <= 0.0) {
    throw std::runtime_error("Invalid measurement_bearing_variance configuration");
  }
  const std::array<double, 6> measurement_noise_parameters{
    measurement_elevation_variance_, measurement_distance_variance_log_scale_,
    measurement_distance_variance_offset_, measurement_armor_yaw_variance_log_scale_,
    measurement_armor_yaw_variance_offset_, distance_update_r_scale_};
  if (std::any_of(measurement_noise_parameters.begin(), measurement_noise_parameters.end(),
      [](double value) { return !std::isfinite(value); }) ||
      measurement_elevation_variance_ <= 0.0 ||
      measurement_distance_variance_log_scale_ < 0.0 ||
      measurement_distance_variance_offset_ <= 0.0 ||
      measurement_armor_yaw_variance_log_scale_ < 0.0 ||
      measurement_armor_yaw_variance_offset_ <= 0.0 ||
      distance_update_r_scale_ < 1.0 || distance_update_r_scale_ > 1e6 ||
      !std::isfinite(yaw_update_sigma_deg_) || yaw_update_sigma_deg_ < 0.0 ||
      yaw_update_sigma_deg_ > 180.0) {
    throw std::runtime_error("Invalid measurement noise configuration");
  }
  if (!std::isfinite(max_prediction_gap_) ||
      max_prediction_gap_ <= tracker_time_policy::large_dt_warning_seconds) {
    throw std::runtime_error("Invalid tracker_max_prediction_gap configuration");
  }
}

void Tracker::set_distance_update_r_scale(double scale)
{
  if (!std::isfinite(scale) || scale < 1.0 || scale > 1e6) {
    throw std::invalid_argument("Distance update R scale must be in [1, 1e6]");
  }
  distance_update_r_scale_ = scale;
}

void Tracker::set_distance_quality_experiment(bool enabled, double reference_m_per_px)
{
    if (!std::isfinite(reference_m_per_px) || reference_m_per_px < 0.0 ||
        (!enabled && reference_m_per_px != 0.0)) {
        throw std::invalid_argument("Invalid distance quality experiment configuration");
    }
    distance_quality_enabled_ = enabled;
    distance_quality_reference_m_per_px_ = reference_m_per_px;
}

void Tracker::set_distance_quality_sigma_px(double sigma_px, double floor_m)
{
  if (!std::isfinite(sigma_px) || sigma_px < 0.0 || sigma_px > 50.0 ||
      !std::isfinite(floor_m) || floor_m < 0.0) {
    throw std::invalid_argument("Invalid distance quality sigma/floor");
  }
  distance_quality_sigma_px_ = sigma_px;
  distance_quality_floor_m_ = floor_m;
}

void Tracker::set_yaw_update_sigma_deg(double sigma_deg)
{
  if (!std::isfinite(sigma_deg) || sigma_deg < 0.0 || sigma_deg > 180.0) {
    throw std::invalid_argument("Yaw update sigma must be in [0, 180] degrees");
  }
  yaw_update_sigma_deg_ = sigma_deg;
}

void Tracker::set_association_image_corner_rms_max_px(double max_error_px)
{
  if (!std::isfinite(max_error_px) || max_error_px < 0.0 || max_error_px > 10000.0) {
    throw std::invalid_argument(
      "Association image corner RMS limit must be in [0, 10000] px");
  }
  association_image_corner_rms_max_px_ = max_error_px;
}

void Tracker::set_association_physical_feasible_gate(bool enabled)
{
  association_physical_feasible_gate_ = enabled;
}

void Tracker::set_association_gate_passed_filter(bool enabled)
{
  association_gate_passed_filter_ = enabled;
}

void Tracker::set_association_corner_rms_ranking(bool enabled)
{
    association_corner_rms_ranking_ = enabled;
}

void Tracker::set_association_update_trace_enabled(bool enabled)
{
  association_update_trace_enabled_ = enabled;
}

std::string Tracker::state() const { return state_; }

const AssociationDebug & Tracker::association_debug() const { return association_debug_; }

std::uint64_t Tracker::target_generation() const { return target_generation_; }

int Tracker::temp_lost_count() const { return temp_lost_count_; }

int Tracker::max_temp_lost_count() const { return max_temp_lost_count_; }

std::string Tracker::geometry_branch() const
{
  if (state_ == "lost") {
    if (rejected_unknown_geometry_) return "unknown_rejected";
    return observation_accepted_ ? geometry_branch_ : "none";
  }
  return geometry_branch_;
}

bool Tracker::geometry_trusted() const
{
  return state_ != "lost" && geometry_branch_ != "none";
}

bool Tracker::geometry_constraint_active() const
{
  return geometry_trusted() && geometry_constraint_active_;
}

bool Tracker::observation_accepted() const { return observation_accepted_; }

ArmorName Tracker::tracked_label() const { return tracked_label_; }

const Target * Tracker::diagnostic_target() const
{
  // 未捕获目标或丢失后的旧 EKF 不参与逐帧统计；更新后当帧丢失仍保留诊断。
  if (state_ == "lost" && !observation_accepted_) return nullptr;
  return &target_;
}

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color,
  EndpointRangeShadowMap * endpoint_range_shadow)
{
  association_debug_ = AssociationDebug{};
  observation_accepted_ = false;
  rejected_unknown_geometry_ = false;
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 单次 100 ms 左右的图像采集抖动不能直接重建目标；EKF 会使用真实 dt
  // 增大预测协方差，再由关联门控、NIS 和几何发散检查判断观测是否可信。
  if (state_ != "lost" && dt > tracker_time_policy::large_dt_warning_seconds) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    if (tracker_time_policy::exceeds_max_prediction_gap(dt, max_prediction_gap_)) {
      tools::logger()->warn(
        "[Tracker] Prediction gap exceeded safety limit: {:.3f}s > {:.3f}s",
        dt, max_prediction_gap_);
      state_ = "lost";
    }
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    recent_names_.clear();
    found = set_target(armors, t, false, endpoint_range_shadow);
  }//如果是lost状态，则寻找优先级最高的装甲板

  else {
    found = update_target(armors, t, std::nullopt, endpoint_range_shadow);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->warn("[Tracker] Lost: target geometry diverged");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  const int nis_failures = std::accumulate(
    target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0);
  if (nis_failures >= (0.4 * target_.ekf().window_size)) {
    // 已在其他路径进入 lost 时不重复报因；此处只记录真正触发状态变化的 NIS 失败。
    if (state_ != "lost") {
      tools::logger()->warn(
        "[Tracker] Lost: NIS failures={} window_size={}",
        nis_failures, target_.ekf().window_size);
    }
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  association_debug_ = AssociationDebug{};
  observation_accepted_ = false;
  rejected_unknown_geometry_ = false;
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 与单路 Tracker 保持一致：短时抖动继续使用真实 dt，超过安全边界才重建。
  if (state_ != "lost" && dt > tracker_time_policy::large_dt_warning_seconds) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    if (tracker_time_policy::exceeds_max_prediction_gap(dt, max_prediction_gap_)) {
      tools::logger()->warn(
        "[Tracker] Prediction gap exceeded safety limit: {:.3f}s > {:.3f}s",
        dt, max_prediction_gap_);
      state_ = "lost";
    }
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) {
        tools::logger()->warn(
          "[Tracker] Lost: switching timeout after {} failed frames", temp_lost_count_);
        state_ = "lost";
      }
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) {
        // 只在真正丢失时输出一次；候选信息属于最后一帧，不代表此前每次失败。
        const auto & last_candidate = association_debug_.candidates[0];
        tools::logger()->warn(
          "[Tracker] Lost: association timeout after {} failed frames "
          "(limit={}, last_candidates={}, last_gate={}, last_accepted={})",
          temp_lost_count_, max_temp_lost_count_, association_debug_.candidate_count,
          last_candidate.gate_passed, association_debug_.accepted_count);
        state_ = "lost";
      }
    }
  }
}

void Tracker::push_recent_name(std::optional<ArmorName> name)
{
  if (!name.has_value()) return;
  recent_names_.push_back(*name);
  while (recent_names_.size() > static_cast<std::size_t>(relock_name_window_)) {
    recent_names_.pop_front();
  }
}

bool Tracker::set_target(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool pose_solved,
  EndpointRangeShadowMap * endpoint_range_shadow_map)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  // 根据兵种优化初始化参数
  const auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  // 保留同济原版的默认分支：非二装甲、前哨站和基地目标按四装甲模型初始化。
  const auto is_standard_four_armor =
    !is_balance && armor.name != ArmorName::outpost && armor.name != ArmorName::base;

  // 已通过关联的观测保留其 IPPE 姿态分支，重建模型时不重新求解。
  if (!pose_solved) solver_.solve(armor);
  EndpointRangeShadowDebug * endpoint_shadow = nullptr;
  if (endpoint_range_shadow_map != nullptr) {
    const auto it = endpoint_range_shadow_map->find(&armor);
    if (it != endpoint_range_shadow_map->end()) endpoint_shadow = &it->second;
  }
  if (endpoint_shadow != nullptr) {
    endpoint_shadow->selected = true;
    endpoint_shadow->selected_raw_distance_m = armor.ypd_in_world[2];
    if (endpoint_shadow->reliable) {
      auto fitted_armor = armor;
      fitted_armor.points.assign(
        endpoint_shadow->fitted_points.begin(), endpoint_shadow->fitted_points.end());
      try {
        solver_.solve(fitted_armor);
        const double fitted_distance = fitted_armor.ypd_in_world[2];
        endpoint_shadow->selected_fitted_distance_m = fitted_distance;
        if (std::isfinite(fitted_distance) && fitted_distance > 0.0) {
          // 初始化只替換距離，角度和板面 yaw 仍取原始 YOLO 角點的 PnP。
          armor.ypd_in_world[2] = fitted_distance;
          armor.xyz_in_world = tools::ypd2xyz(armor.ypd_in_world);
          endpoint_shadow->distance_replaced = true;
        }
      } catch (const cv::Exception &) {
        // 拟合 PnP 失败时继续使用原始距离，让影子试验只影响合格帧。
      }
    }
  }
  geometry_constraint_active_ = false;

  if (is_balance) {
    geometry_branch_ = "balance_two";
    Eigen::VectorXd P0_dig = to_eigen_vector(balance_initial_covariance_diagonal_);
    target_ = Target(armor, t, 0.18, 2, P0_dig);//参数分别为装甲板、时间戳、旋转半径、装甲板数量、初始协方差矩阵
    target_.set_process_noise_variances(
      balance_translation_acceleration_variance_, balance_angular_acceleration_variance_);
  }

  else if (armor.name == ArmorName::outpost) {
    geometry_branch_ = "outpost_three";
    Eigen::VectorXd P0_dig = to_eigen_vector(outpost_initial_covariance_diagonal_);
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
    target_.set_process_noise_variances(
      outpost_translation_acceleration_variance_, outpost_angular_acceleration_variance_);
  }

  else if (armor.name == ArmorName::base) {
    geometry_branch_ = "base_three";
    Eigen::VectorXd P0_dig = to_eigen_vector(base_initial_covariance_diagonal_);
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
    target_.set_process_noise_variances(
      base_translation_acceleration_variance_, base_angular_acceleration_variance_);
  }

  else if (is_standard_four_armor) {
    geometry_branch_ = "standard_four";
    // 单块装甲无法同时确定车辆中心和半径，因此 r 保持软先验；
    // 半径差 l 可按实测尺寸单独约束，不必连带固定 r。
    Eigen::VectorXd P0_dig{
      {standard_initial_position_variance_, standard_initial_velocity_variance_,
       standard_initial_position_variance_, standard_initial_velocity_variance_,
       standard_initial_position_variance_, standard_initial_velocity_variance_,
       standard_initial_center_yaw_variance_,
       standard_initial_angular_velocity_variance_, standard_radius_variance_,
       standard_radius_delta_variance_, standard_height_delta_variance_}};
    target_ = Target(armor, t, standard_radius_, 4, P0_dig);
    target_.set_process_noise_variances(
      standard_translation_acceleration_variance_, standard_angular_acceleration_variance_);
    if (standard_geometry_constraint_) {
      // 整体几何约束会同时固定 r 和 l。
      target_.set_geometry_constraint(
        standard_radius_, 0.0, standard_radius_variance_,
        standard_radius_delta_variance_);
      geometry_constraint_active_ = true;
    } else if (standard_radius_delta_constraint_) {
      // 实测两组装甲半径差为零时，只固定 l，继续让观测修正 r。
      target_.set_radius_delta_constraint(0.0, standard_radius_delta_variance_);
      geometry_constraint_active_ = true;
    }
  }

  target_.set_measurement_bearing_variance(measurement_bearing_variance_);
  target_.set_measurement_noise_model(
    measurement_elevation_variance_, measurement_distance_variance_log_scale_,
    measurement_distance_variance_offset_, measurement_armor_yaw_variance_log_scale_,
    measurement_armor_yaw_variance_offset_);
  if (endpoint_shadow != nullptr) endpoint_shadow->update_accepted = true;
  tracked_label_ = armor.name;
  observation_accepted_ = true;
  target_generation_++;
  successful_capture_updates_ = 0;
  return true;
}

bool Tracker::update_target(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
  std::optional<NameVote>, EndpointRangeShadowMap * endpoint_range_shadow_map)
{
  target_.predict(t);
  const auto predicted_armors = target_.armor_xyza_list();
  if (predicted_armors.empty()) return false;

  // 保留当前同济分支的最近三板候选范围，避免把不可见背面板扩成无差别匹配。
  std::vector<int> nearest_ids(predicted_armors.size());
  std::iota(nearest_ids.begin(), nearest_ids.end(), 0);
  std::stable_sort(nearest_ids.begin(), nearest_ids.end(), [&predicted_armors](int a, int b) {
    return tools::xyz2ypd(predicted_armors[a].head(3))[2] <
           tools::xyz2ypd(predicted_armors[b].head(3))[2];
  });
  nearest_ids.resize(std::min<std::size_t>(3, nearest_ids.size()));

  const auto quad_center = [](const std::vector<cv::Point2f> & points)
    -> std::optional<cv::Point2f> {
    if (points.size() != 4) return std::nullopt;
    cv::Point2f center{0.0F, 0.0F};
    for (const auto & point : points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) return std::nullopt;
      center += point;
    }
    return center * 0.25F;
  };
  const auto quad_geometry_error = [](
    const std::vector<cv::Point2f> & observed,
    const std::vector<cv::Point2f> & predicted)
    -> std::optional<AssociationQuadGeometryError> {
    if (observed.size() != 4 || predicted.size() != 4) return std::nullopt;
    std::array<AssociationPoint2d, 4> observed_quad{};
    std::array<AssociationPoint2d, 4> predicted_quad{};
    for (std::size_t i = 0; i < 4; ++i) {
      observed_quad[i] = {observed[i].x, observed[i].y};
      predicted_quad[i] = {predicted[i].x, predicted[i].y};
    }
    auto error = association_quad_geometry_error(observed_quad, predicted_quad);
    return error.valid ? std::optional<AssociationQuadGeometryError>{error} : std::nullopt;
  };

  // 所有预测板投影都在本帧 EKF 更新前生成，候选比较使用同一状态快照。
  std::vector<std::vector<cv::Point2f>> predicted_quads(predicted_armors.size());
  std::vector<std::optional<cv::Point2f>> predicted_centers(predicted_armors.size());
  for (std::size_t id = 0; id < predicted_armors.size(); ++id) {
    try {
      predicted_quads[id] = solver_.reproject_armor(
        predicted_armors[id].head(3), predicted_armors[id][3],
        target_.armor_type, target_.name);
      predicted_centers[id] = quad_center(predicted_quads[id]);
    } catch (const cv::Exception &) {
      // 单个模型投影失败只让该候选失去图像排序量，不影响其他候选。
    }
  }

  const bool check_image_model = geometry_branch_ == "standard_four";
  const bool image_model_gate_enabled = association_image_model_gate_enabled(
    check_image_model, successful_capture_updates_, standard_capture_bootstrap_updates_);
  association_debug_.model_name = target_.name;
  association_debug_.model_type = target_.armor_type;

  struct Candidate
  {
    std::size_t detection_index;
    Armor * source_armor;
    Armor solved_armor;
    int model_id;
    AssociationCandidateDebug debug;
  };
  std::vector<Candidate> candidates;

  std::size_t detection_index = 0;
  for (auto armor_it = armors.begin(); armor_it != armors.end();
       ++armor_it, ++detection_index) {
    auto & armor = *armor_it;
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    ++association_debug_.matching_detection_count;

    const auto observed_image_center = quad_center(armor.points);
    for (const int id : nearest_ids) {
      const auto & predicted = predicted_armors[id];
      // 每个检测框和板号组合独立选择 IPPE 分支，避免先选错板号再固定错误姿态。
      Armor solved_armor = armor;
      solver_.solve(solved_armor, predicted);

      const auto predicted_ypd = tools::xyz2ypd(predicted.head(3));
      const auto matches = target_.match_armors(solved_armor);
      const auto match_it = std::find_if(
        matches.begin(), matches.end(), [id](const auto & match) { return match.id == id; });
      const double mahalanobis_distance =
        match_it == matches.end() ? std::numeric_limits<double>::infinity()
                                  : match_it->mahalanobis_distance;
      const double angle_error =
        std::abs(tools::limit_rad(solved_armor.ypr_in_world[0] - predicted[3])) +
        std::abs(tools::limit_rad(solved_armor.ypd_in_world[0] - predicted_ypd[0]));
      const double position_error = (solved_armor.xyz_in_world - predicted.head(3)).norm();
      const double distance_error =
        std::abs(solved_armor.ypd_in_world[2] - predicted_ypd[2]);
      const double orientation_error =
        std::abs(tools::limit_rad(solved_armor.ypr_in_world[0] - predicted[3]));
      const double bearing_error =
        std::abs(tools::limit_rad(solved_armor.ypd_in_world[0] - predicted_ypd[0]));
      const double reference_distance = std::max(
        {std::abs(solved_armor.ypd_in_world[2]), std::abs(predicted_ypd[2]), 0.1});
      const double position_angle_error = std::atan2(position_error, reference_distance);
      const double distance_angle_error = std::atan2(distance_error, reference_distance);
      const double association_score =
        angle_error + position_angle_error + distance_angle_error;

      // 旧门限仅用于解释候选；实际排序以四边形代价为主，NIS 与复合分数用于打破平局。
      const bool angle_gate_passed = angle_error <= association_max_angle_error_;
      const bool score_gate_passed = association_score <= association_max_score_;
      const bool position_gate_passed = position_error <= association_max_position_error_;
      const bool distance_gate_passed = distance_error <= association_max_distance_error_;
      const bool mahalanobis_gate_passed =
        mahalanobis_distance <= association_max_mahalanobis_distance_;
      bool image_model_gate_passed = !image_model_gate_enabled;
      double image_reference_error_px = -1.0;
      double image_adjacent_error_px = -1.0;
      double image_observed_extent_px = -1.0;
      if (check_image_model && predicted_centers[id] && observed_image_center) {
        const auto previous_id = (id + 3) % 4;
        const auto next_id = (id + 1) % 4;
        image_reference_error_px =
          cv::norm(*observed_image_center - *predicted_centers[id]);
        const auto previous_error = predicted_centers[previous_id]
          ? cv::norm(*observed_image_center - *predicted_centers[previous_id])
          : std::numeric_limits<double>::infinity();
        const auto next_error = predicted_centers[next_id]
          ? cv::norm(*observed_image_center - *predicted_centers[next_id])
          : std::numeric_limits<double>::infinity();
        image_adjacent_error_px = std::min(previous_error, next_error);
        image_observed_extent_px =
          static_cast<double>(std::max(armor.box.width, armor.box.height));
        if (image_model_gate_enabled) {
          image_model_gate_passed = image_observed_extent_px > 0.0 &&
            image_reference_error_px <= image_adjacent_error_px &&
            image_reference_error_px <= image_observed_extent_px;
        }
      }
      const bool legacy_gates_passed = score_gate_passed && position_gate_passed &&
        distance_gate_passed && mahalanobis_gate_passed && image_model_gate_passed;
      const auto corner_errors = quad_geometry_error(armor.points, predicted_quads[id]);

      AssociationCandidateDebug debug;
      debug.detection_index = static_cast<int>(detection_index);
      debug.model_id = id;
      debug.gate_passed = legacy_gates_passed;
      debug.physical_feasible = score_gate_passed && position_gate_passed &&
        distance_gate_passed && mahalanobis_gate_passed;
      debug.angle_gate_passed = angle_gate_passed;
      debug.score_gate_passed = score_gate_passed;
      debug.position_gate_passed = position_gate_passed;
      debug.distance_gate_passed = distance_gate_passed;
      debug.mahalanobis_gate_passed = mahalanobis_gate_passed;
      debug.image_model_gate_enabled = image_model_gate_enabled;
      debug.image_model_gate_passed = image_model_gate_passed;
      debug.image_reference_error_px = image_reference_error_px;
      debug.image_adjacent_error_px = image_adjacent_error_px;
      debug.image_observed_extent_px = image_observed_extent_px;
      if (corner_errors) {
        debug.image_corner_error_px = corner_errors->corner_rms_px;
        debug.image_corner_error_norm = corner_errors->corner_rms_norm;
        debug.image_geometry_valid = true;
        debug.image_geometry_center_error_norm = corner_errors->center_error_norm;
        debug.image_geometry_edge_angle_error_norm = corner_errors->edge_angle_error_norm;
        debug.image_geometry_side_length_log_ratio = corner_errors->side_length_log_ratio;
        debug.image_geometry_cost = corner_errors->total_cost;
        debug.image_min_corner_rms_px = corner_errors->min_corner_rms_px;
        debug.image_min_corner_rms_norm = corner_errors->min_corner_rms_norm;
        debug.image_min_corner_point_permutation =
            corner_errors->min_corner_point_permutation;
        // 仅替换分配边的排序代价；诊断门、候选生成和匹配数量优先级保持不变。
        debug.image_ranking_cost = association_corner_rms_ranking_
            ? corner_errors->min_corner_rms_norm : corner_errors->total_cost;
        debug.image_point_permutation = corner_errors->point_permutation;
      }
      debug.score = association_score;
      debug.position_error = position_error;
      debug.distance_error = distance_error;
      debug.mahalanobis_distance = mahalanobis_distance;
      debug.position_angle_error = position_angle_error;
      debug.distance_angle_error = distance_angle_error;
      debug.observed_x = solved_armor.xyz_in_world[0];
      debug.observed_y = solved_armor.xyz_in_world[1];
      debug.observed_z = solved_armor.xyz_in_world[2];
      debug.predicted_x = predicted[0];
      debug.predicted_y = predicted[1];
      debug.predicted_z = predicted[2];
      debug.predicted_yaw = predicted[3];
      debug.observed_distance = solved_armor.ypd_in_world[2];
      debug.predicted_distance = predicted_ypd[2];
      debug.orientation_error = orientation_error;
      debug.bearing_error = bearing_error;
      debug.angle_error = angle_error;
      debug.raw_yaw_prediction_error =
        std::abs(tools::limit_rad(solved_armor.yaw_raw - predicted[3]));
      debug.optimized_yaw_prediction_error =
        std::abs(tools::limit_rad(solved_armor.ypr_in_world[0] - predicted[3]));
      debug.raw_yaw = solved_armor.yaw_raw;
      debug.optimized_yaw = solved_armor.ypr_in_world[0];
      debug.yaw_correction =
        tools::limit_rad(solved_armor.ypr_in_world[0] - solved_armor.yaw_raw);
      debug.image_x = solved_armor.center.x;
      debug.image_y = solved_armor.center.y;
      debug.observed_quad_center = observed_image_center;

      if (legacy_gates_passed) ++association_debug_.gate_passed_count;
      if (debug.physical_feasible) ++association_debug_.physical_candidate_count;
      if (association_debug_.model_candidate_count <
          static_cast<int>(association_debug_.model_candidates.size())) {
        association_debug_.model_candidates[association_debug_.model_candidate_count++] = debug;
      }
      candidates.push_back(
        {detection_index, &armor, std::move(solved_armor), id, std::move(debug)});
    }
  }

  std::vector<std::size_t> selected_indices;
  std::vector<AssociationAssignmentEdge> assignment_edges;
  const bool association_experiment_filter_enabled =
    association_image_corner_rms_max_px_ > 0.0 || association_physical_feasible_gate_ ||
    association_gate_passed_filter_;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    auto & debug = candidates[index].debug;
    debug.image_corner_rms_limit_passed = association_image_corner_rms_max_px_ <= 0.0 ||
      (debug.image_geometry_valid && std::isfinite(debug.image_corner_error_px) &&
       debug.image_corner_error_px >= 0.0 &&
       debug.image_corner_error_px <= association_image_corner_rms_max_px_);
    debug.physical_feasible_filter_passed =
      !association_physical_feasible_gate_ || debug.physical_feasible;
    debug.gate_passed_filter_passed =
      !association_gate_passed_filter_ || debug.gate_passed;
    debug.assignment_eligible = debug.image_corner_rms_limit_passed &&
      debug.physical_feasible_filter_passed && debug.gate_passed_filter_passed;
    const AssociationAssignmentEdge edge{
      index, debug.detection_index, debug.model_id, debug.image_geometry_valid,
      debug.image_ranking_cost, debug.mahalanobis_distance, debug.score,
      debug.image_corner_error_px, debug.physical_feasible, debug.gate_passed};
    if (!association_experiment_filter_enabled ||
        association_assignment_edge_eligible(
          edge, association_image_corner_rms_max_px_,
          association_physical_feasible_gate_, association_gate_passed_filter_)) {
      assignment_edges.push_back(edge);
    }
    for (int diagnostic_index = 0;
         diagnostic_index < association_debug_.model_candidate_count; ++diagnostic_index) {
      auto & diagnostic =
        association_debug_.model_candidates[static_cast<std::size_t>(diagnostic_index)];
      if (diagnostic.detection_index == debug.detection_index &&
          diagnostic.model_id == debug.model_id) {
        diagnostic.assignment_eligible = debug.assignment_eligible;
        diagnostic.image_corner_rms_limit_passed = debug.image_corner_rms_limit_passed;
        diagnostic.physical_feasible_filter_passed =
          debug.physical_feasible_filter_passed;
        diagnostic.gate_passed_filter_passed = debug.gate_passed_filter_passed;
        break;
      }
    }
  }
  const std::vector<int> visible_model_ids(nearest_ids.begin(), nearest_ids.end());
  const auto assigned_candidate_indices =
    association_min_cost_assignment(assignment_edges, visible_model_ids);
  if (!assigned_candidate_indices.empty()) {
    selected_indices = assigned_candidate_indices;
    const bool any_selected_image_geometry = std::any_of(
      selected_indices.begin(), selected_indices.end(), [&candidates](std::size_t index) {
        return candidates[index].debug.image_geometry_valid;
      });
    association_debug_.assignment_mode =
      any_selected_image_geometry ? "global_geometry" : "tongji_score_fallback";
  } else if (!candidates.empty() &&
             association_score_fallback_allowed(association_experiment_filter_enabled)) {
    // 候选缺少有限创新时仍沿用 Tongji 复合分数选一个，避免诊断量缺失直接造成整帧掉锁。
    const auto best = std::min_element(
      candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
        if (a.debug.score != b.debug.score) return a.debug.score < b.debug.score;
        if (a.debug.mahalanobis_distance != b.debug.mahalanobis_distance) {
          return a.debug.mahalanobis_distance < b.debug.mahalanobis_distance;
        }
        return a.debug.angle_error < b.debug.angle_error;
      });
    selected_indices.push_back(static_cast<std::size_t>(best - candidates.begin()));
    association_debug_.assignment_mode = "tongji_score_fallback";
  } else if (!candidates.empty()) {
    // 实验门限全拒绝时必须只保留预测，禁止分数兜底重新接纳被剔除的观测。
    association_debug_.assignment_mode = "association_experiment_no_match";
  }

  std::stable_sort(selected_indices.begin(), selected_indices.end(),
    [&candidates](std::size_t a, std::size_t b) {
      return candidates[a].detection_index < candidates[b].detection_index;
    });
  std::vector<bool> detection_assigned(armors.size(), false);
  for (const auto selected_index : selected_indices) {
    auto & candidate = candidates[selected_index];
    candidate.debug.selected = true;
    detection_assigned[candidate.detection_index] = true;
    for (int index = 0; index < association_debug_.model_candidate_count; ++index) {
      auto & diagnostic = association_debug_.model_candidates[static_cast<std::size_t>(index)];
      if (diagnostic.detection_index == candidate.debug.detection_index &&
          diagnostic.model_id == candidate.model_id) {
        diagnostic.selected = true;
        break;
      }
    }
  }
  const auto selected_detection_count = static_cast<int>(std::count(
    detection_assigned.begin(), detection_assigned.end(), true));
  association_debug_.unmatched_detection_count = std::max(
    0, association_debug_.matching_detection_count - selected_detection_count);

  association_debug_.candidate_count = static_cast<int>(std::min(
    selected_indices.size(), association_debug_.candidates.size()));
  for (int index = 0; index < association_debug_.candidate_count; ++index) {
    association_debug_.candidates[static_cast<std::size_t>(index)] =
      candidates[selected_indices[static_cast<std::size_t>(index)]].debug;
  }

  // 匹配确定后按输入检测顺序更新，让同帧多观测的滤波更新可复现。
  std::stable_sort(selected_indices.begin(), selected_indices.end(),
    [&candidates](std::size_t a, std::size_t b) {
      return candidates[a].detection_index < candidates[b].detection_index;
    });

  bool found = false;
  for (const auto selected_index : selected_indices) {
    auto & candidate = candidates[selected_index];
    EndpointRangeShadowDebug * endpoint_shadow = nullptr;
    if (endpoint_range_shadow_map != nullptr) {
      const auto it = endpoint_range_shadow_map->find(candidate.source_armor);
      if (it != endpoint_range_shadow_map->end()) endpoint_shadow = &it->second;
    }
    std::optional<Armor> shadow_measurement;
    const Armor * measurement = &candidate.solved_armor;
    if (endpoint_shadow != nullptr) {
      endpoint_shadow->selected = true;
      endpoint_shadow->selected_raw_distance_m = candidate.solved_armor.ypd_in_world[2];
      if (endpoint_shadow->reliable) {
        auto fitted_armor = candidate.solved_armor;
        fitted_armor.points.assign(
          endpoint_shadow->fitted_points.begin(), endpoint_shadow->fitted_points.end());
        try {
          solver_.solve(fitted_armor, predicted_armors[candidate.model_id]);
          const double fitted_distance = fitted_armor.ypd_in_world[2];
          endpoint_shadow->selected_fitted_distance_m = fitted_distance;
          if (std::isfinite(fitted_distance) && fitted_distance > 0.0) {
            // 关联始终用原始 PnP；这里只替换选中观测的径向分量，不动方位和板面 yaw。
            shadow_measurement.emplace(candidate.solved_armor);
            shadow_measurement->ypd_in_world[2] = fitted_distance;
            shadow_measurement->xyz_in_world = tools::ypd2xyz(shadow_measurement->ypd_in_world);
            measurement = &shadow_measurement.value();
            endpoint_shadow->distance_replaced = true;
          }
        } catch (const cv::Exception &) {
          // 同一预测姿态下重求拟合角点失败时，保留原始距离完成本次关联更新。
        }
      }
    }
    // 标准四装甲目标使用已配置的物理转速上限；超限时 Target 会回滚本次 EKF 更新。
    const auto max_abs_angular_velocity =
      geometry_branch_ == "standard_four"
        ? standard_max_angular_velocity_
        : std::optional<double>{};
    const Armor * measurement_noise_reference = endpoint_shadow != nullptr &&
        endpoint_shadow->distance_replaced
      ? &candidate.solved_armor : nullptr;
    // 先完成关联，再评估原角点；探针不改变初始化和本帧候选选择。
    DistanceObservationQuality distance_quality;
    double quality_scale = 1.0;
    double quality_ms = 0.0;
    std::optional<double> distance_variance_override;
    if (distance_quality_enabled_) {
        const auto start = std::chrono::steady_clock::now();
        distance_quality = estimate_distance_quality(
            candidate.solved_armor, predicted_armors[candidate.model_id][3],
            [this](const Armor & armor) { return solver_.pnp_candidates(armor); });
        if (distance_quality.valid && distance_quality_sigma_px_ > 0.0) {
            // 几何尺度模式：用本次观测的敏感度 s_d 直接给出距离方差，
            // 覆盖倍率规则；敏感度无效时回退到下面的倍率路径。
            const double s = distance_quality.sensitivity_m_per_px;
            distance_variance_override =
                distance_quality_sigma_px_ * distance_quality_sigma_px_ * s * s +
                distance_quality_floor_m_ * distance_quality_floor_m_;
        } else {
            quality_scale = distance_quality.update_scale(distance_quality_reference_m_per_px_);
        }
        quality_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    }
    TargetUpdateTrace target_update_trace;
    const bool accepted = target_.update(
      *measurement, candidate.model_id, std::nullopt,
      max_abs_angular_velocity, measurement_noise_reference,
      distance_update_r_scale_ * quality_scale,
      association_update_trace_enabled_ ? &target_update_trace : nullptr,
      yaw_update_sigma_deg_, distance_variance_override);
    candidate.debug.accepted = accepted;
    if (association_update_trace_enabled_) {
      AssociationUpdateDebug update_debug;
      update_debug.detection_index = static_cast<int>(candidate.detection_index);
      update_debug.model_id = candidate.model_id;
      update_debug.target_update = target_update_trace;
        update_debug.distance_quality = distance_quality;
        update_debug.distance_quality_scale = quality_scale;
        update_debug.distance_quality_ms = quality_ms;
      try {
        const auto branches = solver_.pnp_candidates(candidate.solved_armor);
        update_debug.ippe_branch_count =
          std::min(branches.size(), update_debug.ippe_branch_yaw_world_rad.size());
        for (std::size_t i = 0; i < update_debug.ippe_branch_count; ++i) {
          update_debug.ippe_branch_yaw_world_rad[i] = branches[i].yaw_in_world;
          update_debug.ippe_branch_reprojection_error_px[i] =
            branches[i].reprojection_error;
        }
        if (update_debug.ippe_branch_count == 2) {
          update_debug.ippe_branch_pair_valid = true;
          update_debug.ippe_branch_yaw_gap_rad = std::abs(tools::limit_rad(
            branches[0].yaw_in_world - branches[1].yaw_in_world));
          update_debug.ippe_branch_reprojection_gap_px = std::abs(
            branches[0].reprojection_error - branches[1].reprojection_error);
          const double predicted_yaw = predicted_armors[candidate.model_id][3];
          const double error0 = std::abs(tools::limit_rad(
            branches[0].yaw_in_world - predicted_yaw));
          const double error1 = std::abs(tools::limit_rad(
            branches[1].yaw_in_world - predicted_yaw));
          update_debug.selected_ippe_branch_index = error0 <= error1 ? 0 : 1;
        }
      } catch (const cv::Exception &) {
        // 诊断 PnP 失败只留下空分支记录，不影响已经完成的 Tracker 更新。
      }
      association_debug_.update_trace.push_back(std::move(update_debug));
    }
    if (endpoint_shadow != nullptr) endpoint_shadow->update_accepted = accepted;
    for (int index = 0; index < association_debug_.candidate_count; ++index) {
      auto & selected = association_debug_.candidates[static_cast<std::size_t>(index)];
      if (selected.detection_index == static_cast<int>(candidate.detection_index) &&
          selected.model_id == candidate.model_id) {
        selected.accepted = accepted;
      }
    }
    for (int index = 0; index < association_debug_.model_candidate_count; ++index) {
      auto & diagnostic = association_debug_.model_candidates[static_cast<std::size_t>(index)];
      if (diagnostic.detection_index == static_cast<int>(candidate.detection_index) &&
          diagnostic.model_id == candidate.model_id) {
        diagnostic.accepted = accepted;
        break;
      }
    }
    if (!accepted) continue;

    found = true;
    observation_accepted_ = true;
    ++association_debug_.accepted_count;
    *candidate.source_armor = candidate.solved_armor;
    if (geometry_branch_ == "standard_four") ++successful_capture_updates_;
  }

  if (association_update_trace_enabled_) {
    association_debug_.experiment_candidates.reserve(candidates.size());
    for (const auto & candidate : candidates) {
      association_debug_.experiment_candidates.push_back(candidate.debug);
    }
  }

  return found;
}
}  // namespace auto_aim
