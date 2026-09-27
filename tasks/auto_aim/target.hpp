#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

// 单个观测与当前 EKF 预测模型的关联结果；valid=false 表示协方差计算失败。
struct ArmorMatch
{
  int id = -1;
  double angle_error = 0.0;
  double mahalanobis_distance = 0.0;
  bool valid = false;
};

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);

  // 在当前 EKF 预测状态下寻找最匹配的车辆装甲板 ID。
  // 使用创新协方差归一化残差，只做关联，不修改滤波状态。
  std::vector<ArmorMatch> match_armors(const Armor & armor) const;
  // 兼容单匹配调用，返回马氏距离最小的模型。
  ArmorMatch match_armor(const Armor & armor) const;

  void update(const Armor & armor);
  // 使用已经完成关联的 ID 更新 EKF，避免同一帧内重复计算关联关系。
  void update(const Armor & armor, int id);

  // 对机械尺寸已知的车辆持续约束两组装甲半径。variance 的单位为 m²；
  // 该约束会清除几何量与运动状态的错误互协方差，避免不可观测半径退化。
  void set_geometry_constraint(
    double radius, double radius_delta, double radius_variance,
    double radius_delta_variance);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  struct GeometryConstraint
  {
    double radius;
    double radius_delta;
    double radius_variance;
    double radius_delta_variance;
  };

  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;
  std::optional<GeometryConstraint> geometry_constraint_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  void apply_geometry_constraint();
  Eigen::Matrix4d measurement_noise(const Armor & armor) const;

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
