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
#include <vector>

#include "armor.hpp"
#include "endpoint_range_shadow.hpp"
#include "distance_observation_quality.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/auto_aim/tracker_name_vote.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
// 一个检测框与一个预测板号的关联诊断，几何排序使用更新前的同一状态快照。
struct AssociationCandidateDebug
{
  int detection_index = -1;
  int model_id = -1;
  // RMS 使用最优循环移位/反向绕序配对；仅作诊断，排序使用完整四边形几何代价。
  double image_corner_error_px = -1.0;
  double image_corner_error_norm = -1.0;
  bool image_geometry_valid = false;
  double image_geometry_center_error_norm = -1.0;
  double image_geometry_edge_angle_error_norm = -1.0;
  double image_geometry_side_length_log_ratio = -1.0;
  double image_geometry_cost = -1.0;
  int image_point_permutation = -1;
    // 旧几何字段保持原口径；新增字段记录独立最优 RMS 与实际用于分配的代价。
    double image_min_corner_rms_px = -1.0;
    double image_min_corner_rms_norm = -1.0;
    int image_min_corner_point_permutation = -1;
    double image_ranking_cost = -1.0;
  // 旧门限全部通过的反事实诊断；当前物理候选筛选单独记录在 physical_feasible。
  bool gate_passed = false;
  bool accepted = false;
  // 综合分数、三维位置、距离和 NIS 均满足配置阈值；固定角度和图像中心门不参与此标记。
  bool physical_feasible = false;
  bool selected = false;
  bool assignment_eligible = true;
  bool image_corner_rms_limit_passed = true;
  bool physical_feasible_filter_passed = true;
  bool gate_passed_filter_passed = true;
  // 记录旧 SRM 门限的反事实结果；原版角度关联不使用这些字段拒绝观测。
  bool angle_gate_passed = false;
  bool score_gate_passed = false;
  bool position_gate_passed = false;
  bool distance_gate_passed = false;
  bool mahalanobis_gate_passed = false;
  // 四装甲模型中仅与相邻板比较图像距离，防止沿用旧 ID 或被背面板误拒绝。
  bool image_model_gate_enabled = false;
  bool image_model_gate_passed = false;
  // 图像门限的三个比较量，单位 px；未完成投影或不使用该门限时为 -1。
  double image_reference_error_px = -1.0;
  double image_adjacent_error_px = -1.0;
  double image_observed_extent_px = -1.0;
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
  // 关联前预测的装甲板 yaw，单位 rad；用于日志回投影，不参与门控。
  double predicted_yaw = 0.0;
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
  // 检测四角点的真实均值；Armor.center 是外接框中心，不能替代该值。
  std::optional<cv::Point2f> observed_quad_center;
};

// 同一帧按检测顺序执行的 EKF 更新轨迹；每条记录对应一个候选更新尝试。
struct AssociationUpdateDebug
{
  int detection_index = -1;
  int model_id = -1;
  TargetUpdateTrace target_update;
    DistanceObservationQuality distance_quality;
    double distance_quality_scale = 1.0;
    double distance_quality_ms = 0.0;
  std::size_t ippe_branch_count = 0;
  std::array<double, 2> ippe_branch_yaw_world_rad{};
  std::array<double, 2> ippe_branch_reprojection_error_px{};
  bool ippe_branch_pair_valid = false;
  int selected_ippe_branch_index = -1;
  double ippe_branch_yaw_gap_rad = 0.0;
  double ippe_branch_reprojection_gap_px = 0.0;
};

// 每帧保留一对一分配后的候选；其余组合写入模型候选诊断列表。
struct AssociationDebug
{
  // 记录 Target 原有物理模型；检测标签改判时仍按继承的模型回投影。
  ArmorName model_name = ArmorName::not_armor;
  ArmorType model_type = ArmorType::small;
  // matching_detection_count 表示输入中与当前目标同名同类型的检测数量。
  // gate_passed_count 仅统计旧 SRM 门限的诊断结果，不代表当前选择条件。
  int matching_detection_count = 0;
  int gate_passed_count = 0;
  int physical_candidate_count = 0;
  int unmatched_detection_count = 0;
  std::string assignment_mode = "none";
  int candidate_count = 0;
  int accepted_count = 0;
  // 每个预测板号最多接收一个检测，标准模型最多四个。
  std::array<AssociationCandidateDebug, 4> candidates{};
  // 记录检测框 × 候选板号组合；容量满后只截断诊断，不影响匹配。
  int model_candidate_count = 0;
  std::array<AssociationCandidateDebug, 8> model_candidates{};
  // 离线实验逐候选完整记录，不受真机兼容诊断数组的八项上限影响。
  std::vector<AssociationCandidateDebug> experiment_candidates;
  std::vector<AssociationUpdateDebug> update_trace;
};

class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  // 仅供离线对照调整 EKF 更新距离方差；默认 1.0，不读取生产 YAML。
  void set_distance_update_r_scale(double scale);
    // 仅离线：reference=0 只记录；正值为冻结的主序列参考灵敏度，单位 m/px。
    void set_distance_quality_experiment(bool enabled, double reference_m_per_px);
  // 几何尺度模式：R_dd = (s_d·σ_px)² + floor²；σ_px=0 关闭该模式，保留倍率规则。
  void set_distance_quality_sigma_px(double sigma_px, double floor_m);
  // 仅供离线回放扫描板面 yaw 更新标准差；0 表示保持当前基准方差。
  void set_yaw_update_sigma_deg(double sigma_deg);
  // 以下关联筛选只由离线回放设置；默认关闭，不改变真机 Tracker 行为。
  void set_association_image_corner_rms_max_px(double max_error_px);
  void set_association_physical_feasible_gate(bool enabled);
  // 仅供离线对照：要求全部现有关联门通过；默认关闭。
  void set_association_gate_passed_filter(bool enabled);
    // 仅离线可切换到归一化角点 RMS 排序；默认 false 保持旧几何代价。
    void set_association_corner_rms_ranking(bool enabled);
  void set_association_update_trace_enabled(bool enabled);

  std::string state() const;
  const AssociationDebug & association_debug() const;
  std::uint64_t target_generation() const;
  int temp_lost_count() const;
  int max_temp_lost_count() const;
  // 物理模型与检测标签分开记录；跨几何模型的捕获期改判会显式重建目标。
  std::string geometry_branch() const;
  bool geometry_trusted() const;
  bool geometry_constraint_active() const;
  bool observation_accepted() const;
  ArmorName tracked_label() const;
  // 丢失发生在本帧 EKF 更新之后时，仍可只读记录触发丢失的滤波诊断。
  const Target * diagnostic_target() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true,
    EndpointRangeShadowMap * endpoint_range_shadow = nullptr);

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
  // 非标准模型完整 P0 对角线，顺序为 [x,vx,y,vy,z,vz,center_yaw,omega,r,l,h]。
  std::array<double, 11> balance_initial_covariance_diagonal_;
  std::array<double, 11> outpost_initial_covariance_diagonal_;
  std::array<double, 11> base_initial_covariance_diagonal_;
  // 标准四装甲机器人的已知几何先验；方差单位为 m²。
  bool standard_geometry_constraint_;
  bool standard_radius_delta_constraint_;
  double standard_radius_;
  double standard_radius_variance_;
  double standard_radius_delta_variance_;
  double standard_height_delta_variance_;
  double standard_initial_position_variance_;
  double standard_initial_center_yaw_variance_;
  // 标准四装甲捕获时的平移速度初始方差，单位 m²/s²。
  double standard_initial_velocity_variance_;
  // 标准四装甲捕获时的角速度初始方差，单位 (rad/s)²。
  double standard_initial_angular_velocity_variance_;
  // 捕获后累计多少次有效更新才启用图像选板门；0 表示保持原有立即启用行为。
  int standard_capture_bootstrap_updates_;
  // 标准四装甲目标的平移加速度方差，单位 m²/s⁴。
  double standard_translation_acceleration_variance_;
  double standard_angular_acceleration_variance_;
  double balance_translation_acceleration_variance_;
  double balance_angular_acceleration_variance_;
  double outpost_translation_acceleration_variance_;
  double outpost_angular_acceleration_variance_;
  double base_translation_acceleration_variance_;
  double base_angular_acceleration_variance_;
  // 可选标准四装甲角速度上限，单位 rad/s；只由仿真配置启用。
  std::optional<double> standard_max_angular_velocity_;
  // 固定角度诊断阈值，单位为 rad；实际接受由综合分数和 EKF 协方差门控决定。
  double association_max_angle_error_;
  // 综合关联分数门限，单位为等效 rad；用于拒绝其他机器人同名装甲板。
  double association_max_score_;
  // 旧 ID 仍通过门限时，新 ID 的排序附加分数，单位为等效 rad；只影响排序。
  double association_id_switch_score_margin_;
  // 远距离下角度归一化可能掩盖较大的三维误差，单位为 m。
  double association_max_position_error_;
  double association_max_distance_error_;
  // 创新协方差归一化残差门限，无量纲；用于关联主排序和统计门控。
  double association_max_mahalanobis_distance_;
  double distance_update_r_scale_ = 1.0;
    bool distance_quality_enabled_ = false;
    double distance_quality_reference_m_per_px_ = 0.0;
    double distance_quality_sigma_px_ = 0.0;
    double distance_quality_floor_m_ = 0.0;
  double yaw_update_sigma_deg_ = 0.0;
  double association_image_corner_rms_max_px_ = 0.0;
  bool association_physical_feasible_gate_ = false;
  bool association_gate_passed_filter_ = false;
    bool association_corner_rms_ranking_ = false;
  bool association_update_trace_enabled_ = false;
  // 配置值仅改变方位观测噪声；未配置时保持原实现。
  double measurement_bearing_variance_;
  double measurement_elevation_variance_;
  double measurement_distance_variance_log_scale_;
  double measurement_distance_variance_offset_;
  double measurement_armor_yaw_variance_log_scale_;
  double measurement_armor_yaw_variance_offset_;
  // 最大可信预测间隔，单位为 s；短时抖动由 EKF 使用真实 dt 吸收。
  double max_prediction_gap_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  AssociationDebug association_debug_;
  ArmorName tracked_label_ = ArmorName::not_armor;
  std::string geometry_branch_ = "none";
  bool geometry_constraint_active_ = false;
  bool rejected_unknown_geometry_ = false;
  bool observation_accepted_ = false;
  // 每次 set_target 成功后递增，用于区分真实 ID 跳变和 Tracker 重初始化。
  std::uint64_t target_generation_ = 0;
  // set_target 的首帧只初始化状态；仅累计其后的成功关联更新。
  int successful_capture_updates_ = 0;
  // 捕获阶段的类别投票窗口；只在还没进入 tracking 时用于改判类别。
  int relock_name_window_;
  int relock_name_votes_;
  std::deque<ArmorName> recent_names_;

  void state_machine(bool found);

  void push_recent_name(std::optional<ArmorName> name);

  bool set_target(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool pose_solved = false,
    EndpointRangeShadowMap * endpoint_range_shadow = nullptr);

  // 投票仅允许检查另一检测标签；位置、姿态与 EKF 后验仍需通过原有门限。
  bool update_target(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    std::optional<NameVote> relabel_vote = std::nullopt,
    EndpointRangeShadowMap * endpoint_range_shadow = nullptr);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
