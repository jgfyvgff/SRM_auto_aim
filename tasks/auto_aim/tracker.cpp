#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

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

  standard_geometry_constraint_ = yaml["standard_geometry_constraint"].IsDefined()
                                    ? yaml["standard_geometry_constraint"].as<bool>()
                                    : false;
  standard_radius_ =
    yaml["standard_radius"].IsDefined() ? yaml["standard_radius"].as<double>() : 0.2;
  standard_radius_variance_ = yaml["standard_radius_variance"].IsDefined()
                                ? yaml["standard_radius_variance"].as<double>()
                                : 1.0;
  standard_radius_delta_variance_ = yaml["standard_radius_delta_variance"].IsDefined()
                                      ? yaml["standard_radius_delta_variance"].as<double>()
                                      : 1.0;
  standard_height_delta_variance_ = yaml["standard_height_delta_variance"].IsDefined()
                                      ? yaml["standard_height_delta_variance"].as<double>()
                                      : 1.0;
  association_max_angle_error_ = yaml["association_max_angle_error"].IsDefined()
                                   ? yaml["association_max_angle_error"].as<double>()
                                   : std::numeric_limits<double>::infinity();
  association_max_score_ = yaml["association_max_score"].IsDefined()
                             ? yaml["association_max_score"].as<double>()
                             : 0.8;
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
  measurement_bearing_variance_ =
    yaml["measurement_bearing_variance"].IsDefined()
      ? yaml["measurement_bearing_variance"].as<double>()
      : 4e-3;

  if (
    standard_radius_ <= 0.05 || standard_radius_ >= 0.5 ||
    standard_radius_variance_ <= 0 || standard_radius_delta_variance_ <= 0 ||
    standard_height_delta_variance_ <= 0)
  {
    throw std::runtime_error("Invalid standard armor geometry configuration");
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
  if (association_max_position_error_ <= 0.0 || association_max_distance_error_ <= 0.0) {
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
}

std::string Tracker::state() const { return state_; }

const AssociationDebug & Tracker::association_debug() const { return association_debug_; }

std::uint64_t Tracker::target_generation() const { return target_generation_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  association_debug_ = AssociationDebug{};
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
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
    found = set_target(armors, t);
  }//如果是lost状态，则寻找优先级最高的装甲板

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
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
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
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
      // 初始 Target 刚建立时，单帧关联失败不应立即销毁；
      // 先进入临时丢失，保留预测状态，给后续帧重新关联的机会。
      temp_lost_count_ = 1;
      state_ = "temp_lost";
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
      if (temp_lost_count_ > 200) state_ = "lost";
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

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  solver_.solve(armor);

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  const auto is_standard_four_armor =
    armor.type == ArmorType::small &&
    (armor.name == ArmorName::two || armor.name == ArmorName::three ||
     armor.name == ArmorName::four || armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);//参数分别为装甲板、时间戳、旋转半径、装甲板数量、初始协方差矩阵
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else if (is_standard_four_armor) {
    // 单块静止装甲板无法同时观测车辆中心和半径，因此使用已知机械尺寸
    // 约束 r、两组半径差和高度差，避免半径向发散下限塌缩。
    Eigen::VectorXd P0_dig{
      {1, 64, 1, 64, 1, 64, 0.4, 100, standard_radius_variance_,
       standard_radius_delta_variance_, standard_height_delta_variance_}};
    target_ = Target(armor, t, standard_radius_, 4, P0_dig);
    if (standard_geometry_constraint_) {
      // 仿真车辆尺寸已知时固定 r 和 l，避免单块装甲观测把半径压到发散下限。
      target_.set_geometry_constraint(
        standard_radius_, 0.0, standard_radius_variance_,
        standard_radius_delta_variance_);
    }
  }

  else {
    // 其他车型保持上游原有行为，后续应按实际机械尺寸分别标定。
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  target_.set_measurement_bearing_variance(measurement_bearing_variance_);
  target_generation_++;
  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);// 预测，更新协方差矩阵和预测量

  struct Candidate
  {
    Armor * armor;
    // 不同模型 ID 可能选择不同的 IPPE 分支，候选必须保留各自的解算结果。
    Armor solved_armor;
    int id;
    double angle_error;
    double mahalanobis_distance;
    double position_error;
    double distance_error;
    double position_angle_error;
    double distance_angle_error;
    double association_score;
    double orientation_error;
    double bearing_error;
    bool angle_gate_passed;
    bool score_gate_passed;
    bool position_gate_passed;
    bool distance_gate_passed;
    bool mahalanobis_gate_passed;
    bool gate_passed;
  };

  const auto predicted_armors = target_.armor_xyza_list();
  // 所有模型假设和最终候选共用一份诊断字段转换，保证比较的是同一预测时刻。
  const auto make_candidate_debug = [&predicted_armors](const Candidate & candidate) {
    const auto & predicted_armor = predicted_armors[candidate.id];
    const auto predicted_ypd = tools::xyz2ypd(predicted_armor.head(3));
    AssociationCandidateDebug debug;
    debug.model_id = candidate.id;
    debug.gate_passed = candidate.gate_passed;
    debug.angle_gate_passed = candidate.angle_gate_passed;
    debug.score_gate_passed = candidate.score_gate_passed;
    debug.position_gate_passed = candidate.position_gate_passed;
    debug.distance_gate_passed = candidate.distance_gate_passed;
    debug.mahalanobis_gate_passed = candidate.mahalanobis_gate_passed;
    debug.score = candidate.association_score;
    debug.position_error = candidate.position_error;
    debug.distance_error = candidate.distance_error;
    debug.mahalanobis_distance = candidate.mahalanobis_distance;
    debug.position_angle_error = candidate.position_angle_error;
    debug.distance_angle_error = candidate.distance_angle_error;
    debug.observed_x = candidate.solved_armor.xyz_in_world[0];
    debug.observed_y = candidate.solved_armor.xyz_in_world[1];
    debug.observed_z = candidate.solved_armor.xyz_in_world[2];
    debug.predicted_x = predicted_armor[0];
    debug.predicted_y = predicted_armor[1];
    debug.predicted_z = predicted_armor[2];
    debug.observed_distance = candidate.solved_armor.ypd_in_world[2];
    debug.predicted_distance = predicted_ypd[2];
    debug.orientation_error = candidate.orientation_error;
    debug.bearing_error = candidate.bearing_error;
    debug.raw_yaw = candidate.solved_armor.yaw_raw;
    debug.optimized_yaw = candidate.solved_armor.ypr_in_world[0];
    debug.yaw_correction = tools::limit_rad(
      candidate.solved_armor.ypr_in_world[0] - candidate.solved_armor.yaw_raw);
    debug.image_x = candidate.solved_armor.center.x;
    debug.image_y = candidate.solved_armor.center.y;
    return debug;
  };
  std::vector<Candidate> candidates;
  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;

    std::optional<Candidate> best_gated_match;
    std::optional<Candidate> best_rejected_match;
    // 不能仅靠位置提前锁定模型 ID；旋转时相邻装甲的位置可能很近，
    // 每个 ID 都要先选择与自身预测 yaw 连续的 IPPE 分支，再独立计算关联残差。
    for (std::size_t reference_id = 0; reference_id < predicted_armors.size(); ++reference_id) {
      Armor solved_armor = armor;
      solver_.solve(solved_armor, predicted_armors[reference_id]);
      const auto matches = target_.match_armors(solved_armor);
      const auto match_it = std::find_if(
        matches.begin(), matches.end(), [reference_id](const auto & match) {
          return match.id == static_cast<int>(reference_id);
        });
      if (match_it == matches.end()) continue;
      const auto & match = *match_it;

      const auto & predicted_armor = predicted_armors[match.id];
      const auto predicted_ypd = tools::xyz2ypd(predicted_armor.head(3));
      const auto position_error =
        (solved_armor.xyz_in_world - predicted_armor.head(3)).norm();
      const auto distance_error =
        std::abs(solved_armor.ypd_in_world[2] - predicted_ypd[2]);
      const auto orientation_error =
        std::abs(tools::limit_rad(solved_armor.ypr_in_world[0] - predicted_armor[3]));
      const auto bearing_error =
        std::abs(tools::limit_rad(solved_armor.ypd_in_world[0] - predicted_ypd[0]));
      // 位置误差和距离误差会随目标距离改变量纲影响，先折算为视线角度，
      // 再与装甲板姿态/方位角误差相加，才能在近距离和远距离使用同一门限。
      const auto reference_distance = std::max(
        {std::abs(solved_armor.ypd_in_world[2]), std::abs(predicted_ypd[2]), 0.1});
      const auto position_angle_error = std::atan2(position_error, reference_distance);
      const auto distance_angle_error = std::atan2(distance_error, reference_distance);
      const auto association_score =
        match.angle_error + position_angle_error + distance_angle_error;
      const auto angle_gate_passed =
        match.angle_error <= association_max_angle_error_;
      const auto score_gate_passed = association_score <= association_max_score_;
      const auto position_gate_passed = position_error <= association_max_position_error_;
      const auto distance_gate_passed = distance_error <= association_max_distance_error_;
      const auto mahalanobis_gate_passed =
        match.mahalanobis_distance <= association_max_mahalanobis_distance_;
      const auto gate_passed =
        angle_gate_passed && score_gate_passed && position_gate_passed &&
        distance_gate_passed && mahalanobis_gate_passed;

      Candidate candidate{
        &armor, solved_armor, match.id, match.angle_error, match.mahalanobis_distance,
        position_error, distance_error, position_angle_error, distance_angle_error,
        association_score, orientation_error, bearing_error, angle_gate_passed,
        score_gate_passed, position_gate_passed, distance_gate_passed,
        mahalanobis_gate_passed, gate_passed};
      if (
        association_debug_.model_candidate_count <
        static_cast<int>(association_debug_.model_candidates.size()))
      {
        association_debug_.model_candidates[association_debug_.model_candidate_count++] =
          make_candidate_debug(candidate);
      }
      auto & best = gate_passed ? best_gated_match : best_rejected_match;
      if (!best || candidate.mahalanobis_distance < best->mahalanobis_distance) {
        best = candidate;
      }
    }

    // 每个检测只保留一个最优的“分支 × 模型 ID”组合，避免重复框多次更新 EKF。
    if (best_gated_match) {
      candidates.push_back(*best_gated_match);
    } else if (best_rejected_match) {
      candidates.push_back(*best_rejected_match);
    }
  }

  if (candidates.empty()) return false;

  // 所有关联都基于本帧同一个预测状态计算，再按误差从小到大更新。
  // 同一个模型 ID 每帧只接收一个观测，避免重复框或顺序变化把 EKF 拉向不同装甲板。
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {
      // 先保证候选通过全部安全门限，再在可接受候选中比较统计距离。
      // 否则最小马氏距离的坏观测会挡住后面本可吸收的有效观测。
      if (a.gate_passed != b.gate_passed) {
        return a.gate_passed > b.gate_passed;
      }
      if (a.mahalanobis_distance != b.mahalanobis_distance) {
        return a.mahalanobis_distance < b.mahalanobis_distance;
      }
      return a.association_score < b.association_score;
    });

  association_debug_.candidate_count = static_cast<int>(candidates.size());
  const auto debug_count = std::min(candidates.size(), association_debug_.candidates.size());
  for (std::size_t index = 0; index < debug_count; ++index) {
    association_debug_.candidates[index] = make_candidate_debug(candidates[index]);
  }

  // 同一帧的同名装甲板可能来自不同机器人，不能把它们分别写入同一个 EKF。
  // 只吸收通过门限且马氏距离最小的一个候选；其余候选仍保留在 debug 中供诊断。
  const auto & best_candidate = candidates.front();
  // 通过关联门限后仍需检查 EKF 后验；拒绝时保持预测状态和诊断候选。
  const bool accepted =
    best_candidate.gate_passed && target_.update(best_candidate.solved_armor, best_candidate.id);
  association_debug_.candidates[0].accepted = accepted;
  if (accepted) {
    association_debug_.accepted_count = 1;
    // 探针会从原始检测列表读取已接收的位姿，因此把选中的分支回填给原对象。
    *best_candidate.armor = best_candidate.solved_armor;
  }

  // 门限或后验拒绝时进入 temp_lost；持续拒绝由原状态机触发重捕获。
  return accepted;
}

}  // namespace auto_aim
