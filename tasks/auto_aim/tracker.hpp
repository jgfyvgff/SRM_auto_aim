#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cstdint>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
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
  double score = 0.0;
  // 关联候选与预测装甲板的三维位置残差，单位为 m，仅用于诊断。
  double position_error = 0.0;
  // 关联候选与预测装甲板的距离残差，单位为 m，仅用于诊断。
  double distance_error = 0.0;
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
  double raw_yaw = 0.0;
  double optimized_yaw = 0.0;
  double yaw_correction = 0.0;
  double image_x = 0.0;
};

// 每帧最多保留两个最优候选，足以覆盖仿真中同时可见的两块装甲板。
struct AssociationDebug
{
  int candidate_count = 0;
  int accepted_count = 0;
  std::array<AssociationCandidateDebug, 2> candidates{};
};

class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;
  const AssociationDebug & association_debug() const;
  std::uint64_t target_generation() const;

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
  // 关联误差门限，单位为 rad；超限观测只保留为诊断候选，不进入 EKF。
  double association_max_angle_error_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  AssociationDebug association_debug_;
  // 每次 set_target 成功后递增，用于区分真实 ID 跳变和 Tracker 重初始化。
  std::uint64_t target_generation_ = 0;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
