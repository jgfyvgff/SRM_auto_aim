#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <list>
#include <optional>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/auto_aim/tracker_name_vote.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
// 单个检测框在同一预测状态下的关联诊断；仅用于仿真分析，不参与控制决策。
struct AssociationCandidateDebug
{
  int model_id = -1;
  bool gate_passed = false;
  bool accepted = false;
  // 分别记录各安全门限，便于判断拒绝原因；固定 angle 门仅作诊断。
  bool angle_gate_passed = false;
  bool score_gate_passed = false;
  bool position_gate_passed = false;
  bool distance_gate_passed = false;
  bool mahalanobis_gate_passed = false;
  double score = 0.0;
  // 关联候选与预测装甲板的三维位置残差，单位为 m，仅用于诊断。
  double position_error = 0.0;
  // 关联候选与预测装甲板的距离残差，单位为 m，仅用于诊断。
  double distance_error = 0.0;
  // 创新协方差归一化残差，无量纲；值越小表示与该模型的统计一致性越高。
  double mahalanobis_distance = 0.0;
  // 将三维残差按目标距离折算成近似角度后的分量，单位为 rad。
  double position_angle_error = 0.0;
  double distance_angle_error = 0.0;
  double observed_x = 0.0;
  double observed_y = 0.0;
  double observed_z = 0.0;
  double predicted_x = 0.0;
  double predicted_y = 0.0;
  double predicted_z = 0.0;
  double observed_distance = 0.0;
  double predicted_distance = 0.0;
  double orientation_error = 0.0;
  double bearing_error = 0.0;
  // match.angle_error 的最终值，单位为 rad；用于定位角度门限失败。
  double angle_error = 0.0;
  // raw/optimized yaw 相对当前预测装甲板 yaw 的误差，单位为 rad。
  double raw_yaw_prediction_error = 0.0;
  double optimized_yaw_prediction_error = 0.0;
  double raw_yaw = 0.0;
  double optimized_yaw = 0.0;
  double yaw_correction = 0.0;
  double image_x = 0.0;
  double image_y = 0.0;
};

// 每帧最多保留两个最优候选用于诊断，但实际只允许一个候选更新 EKF。
struct AssociationDebug
{
  // matching_detection_count 表示输入中与当前目标同名同类型的检测数量。
  // gate_passed_count 表示这些检测中至少有一个模型 ID 通过全部关联门限的数量。
  int matching_detection_count = 0;
  int gate_passed_count = 0;
  int candidate_count = 0;
  int accepted_count = 0;
  std::array<AssociationCandidateDebug, 2> candidates{};
  // 最多记录前八个模型假设（四装甲车通常覆盖两个检测框）；满后只丢弃诊断。
  int model_candidate_count = 0;
  std::array<AssociationCandidateDebug, 8> model_candidates{};
};

class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;
  const AssociationDebug & association_debug() const;
  std::uint64_t target_generation() const;
  int temp_lost_count() const;
  int max_temp_lost_count() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  // 标准四装甲机器人的已知几何先验；方差单位为 m²。
  bool standard_geometry_constraint_;
  double standard_radius_;
  double standard_radius_variance_;
  double standard_radius_delta_variance_;
  double standard_height_delta_variance_;
  // 固定角度诊断阈值，单位为 rad；实际接受由综合分数和 EKF 协方差门控决定。
  double association_max_angle_error_;
  // 综合关联分数门限，单位为等效 rad；用于拒绝其他机器人同名装甲板。
  double association_max_score_;
  // 远距离下角度归一化可能掩盖较大的三维误差，单位为 m。
  double association_max_position_error_;
  double association_max_distance_error_;
  // 创新协方差归一化残差门限，无量纲；用于关联主排序和统计门控。
  double association_max_mahalanobis_distance_;
  // 配置值仅改变方位观测噪声；未配置时保持原实现。
  double measurement_bearing_variance_;
  // 最大可信预测间隔，单位为 s；短时抖动由 EKF 使用真实 dt 吸收。
  double max_prediction_gap_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  AssociationDebug association_debug_;
  // 每次 set_target 成功后递增，用于区分真实 ID 跳变和 Tracker 重初始化。
  std::uint64_t target_generation_ = 0;
  // 捕获阶段的类别投票窗口；只在还没进入 tracking 时用于改判类别。
  int relock_name_window_;
  int relock_name_votes_;
  std::deque<ArmorName> recent_names_;

  void state_machine(bool found);

  void push_recent_name(std::optional<ArmorName> name);

  // 捕获阶段按多数票改判类别；返回 true 表示已经用新类别重建了目标。
  bool relock_on_majority_name(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
