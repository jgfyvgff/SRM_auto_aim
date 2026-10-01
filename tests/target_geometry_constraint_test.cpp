#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/math_tools.hpp"

namespace
{
bool nearly_equal(double lhs, double rhs, double tolerance = 1e-9)
{
  return std::abs(lhs - rhs) <= tolerance;
}
}  // namespace

int main()
{
  const std::vector<cv::Point2f> points{
    {100.0F, 100.0F}, {140.0F, 100.0F}, {140.0F, 120.0F}, {100.0F, 120.0F}};
  auto_aim::Armor armor(9, 0.9F, cv::Rect(100, 100, 40, 20), points);
  armor.priority = auto_aim::ArmorPriority::first;
  armor.xyz_in_world = Eigen::Vector3d(1.0, 0.0, 0.5);
  armor.ypr_in_world = Eigen::Vector3d::Zero();
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  Eigen::VectorXd initial_variance{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
  auto_aim::Target target(
    armor, std::chrono::steady_clock::now(), 0.1, 4, initial_variance);

  constexpr double expected_radius = 0.21147;
  constexpr double expected_variance = 1e-4;
  target.set_geometry_constraint(
    expected_radius, 0.0, expected_variance, expected_variance);
  target.predict(0.01);

  const auto state = target.ekf_x();
  const auto & covariance = target.ekf().P;
  if (!nearly_equal(state[8], expected_radius) || !nearly_equal(state[9], 0.0)) {
    std::cerr << "几何约束未保持标准四装甲半径\n";
    return 1;
  }
  if (
    !nearly_equal(covariance(8, 8), expected_variance) ||
    !nearly_equal(covariance(9, 9), expected_variance))
  {
    std::cerr << "几何约束方差错误\n";
    return 1;
  }
  for (Eigen::Index index = 0; index < covariance.rows(); ++index) {
    if (index == 8 || index == 9) continue;
    if (
      !nearly_equal(covariance(8, index), 0.0) ||
      !nearly_equal(covariance(index, 8), 0.0) ||
      !nearly_equal(covariance(9, index), 0.0) ||
      !nearly_equal(covariance(index, 9), 0.0))
    {
      std::cerr << "几何约束未清除错误互协方差\n";
      return 1;
    }
  }

  // 同一预测状态下，更可信的方位观测应产生更大的归一化创新。
  Eigen::VectorXd small_variance = Eigen::VectorXd::Constant(11, 1e-6);
  auto_aim::Target default_noise(
    armor, std::chrono::steady_clock::now(), 0.1, 4, small_variance);
  auto_aim::Target tuned_noise(
    armor, std::chrono::steady_clock::now(), 0.1, 4, small_variance);
  tuned_noise.set_measurement_bearing_variance(4e-5);
  auto shifted_armor = armor;
  shifted_armor.xyz_in_world[1] = 0.08;
  shifted_armor.ypd_in_world = tools::xyz2ypd(shifted_armor.xyz_in_world);
  const auto default_matches = default_noise.match_armors(shifted_armor);
  const auto tuned_matches = tuned_noise.match_armors(shifted_armor);
  if (default_matches.empty() || tuned_matches.empty() ||
      tuned_matches[0].mahalanobis_distance <=
        default_matches[0].mahalanobis_distance) {
    std::cerr << "方位观测噪声未影响关联统计距离\n";
    return 1;
  }

  auto_aim::Target rollback_target(
    armor, std::chrono::steady_clock::now(), 0.1, 4, initial_variance);
  const auto prior_ekf = rollback_target.ekf();
  auto inconsistent_armor = armor;
  // 模拟消息内三维位置与距离不一致：门控之前的候选可由 Tracker 校验，
  // Target 自身必须防止这类观测把已知装甲位置向远处拉走。
  inconsistent_armor.ypd_in_world = tools::xyz2ypd(Eigen::Vector3d(5.0, 0.0, 0.5));
  if (rollback_target.update(inconsistent_armor, 0)) {
    std::cerr << "后验位置残差劣化的观测未被拒绝\n";
    return 1;
  }
  if (
    !rollback_target.ekf_x().isApprox(prior_ekf.x) ||
    !rollback_target.ekf().P.isApprox(prior_ekf.P) ||
    rollback_target.ekf().data != prior_ekf.data ||
    rollback_target.ekf().recent_nis_failures != prior_ekf.recent_nis_failures ||
    !nearly_equal(rollback_target.ekf().last_nis, prior_ekf.last_nis) ||
    rollback_target.last_id != 0 || rollback_target.jumped)
  {
    std::cerr << "拒绝观测后 EKF 或装甲关联状态未完全恢复\n";
    return 1;
  }
  if (!rollback_target.update(armor, 0)) {
    std::cerr << "正常观测被后验检查拒绝\n";
    return 1;
  }
  auto_aim::Target small_error_target(
    armor, std::chrono::steady_clock::now(), 0.1, 4, initial_variance);
  auto small_error_armor = armor;
  small_error_armor.ypd_in_world = tools::xyz2ypd(Eigen::Vector3d(1.2, 0.0, 0.5));
  if (!small_error_target.update(small_error_armor, 0)) {
    std::cerr << "观测噪声范围内的小幅位置残差增加被误拒\n";
    return 1;
  }

  // 同一观测先不带门限更新，再把其后验残差作为确定性边界，验证
  // 越界时 EKF 的状态、协方差和诊断历史均回滚；真实跳变由仿真回归验证。
  auto far_armor = armor;
  far_armor.xyz_in_world = Eigen::Vector3d(3.2, 0.9, 0.3);
  far_armor.ypr_in_world[0] = 0.5;
  far_armor.ypd_in_world = tools::xyz2ypd(far_armor.xyz_in_world);
  auto_aim::Target predicted_target(
    far_armor, std::chrono::steady_clock::now(), expected_radius, 4, initial_variance);
  predicted_target.set_geometry_constraint(
    expected_radius, 0.0, expected_variance, expected_variance);
  predicted_target.set_measurement_bearing_variance(4e-5);
  predicted_target.predict(0.1);
  auto candidate = far_armor;
  candidate.xyz_in_world[1] += 0.2;
  candidate.ypr_in_world[0] += 0.2;
  candidate.ypd_in_world = tools::xyz2ypd(candidate.xyz_in_world);

  auto unguarded = predicted_target;
  if (!unguarded.update(candidate, 0)) {
    std::cerr << "测试观测未通过原有后验检查\n";
    return 1;
  }
  const double posterior_distance =
    tools::xyz2ypd(unguarded.armor_xyza_list()[0].head<3>())[2];
  const double posterior_distance_error =
    std::abs(candidate.ypd_in_world[2] - posterior_distance);
  if (posterior_distance_error <= 1e-8) {
    std::cerr << "测试观测没有可检查的后验距离残差\n";
    return 1;
  }
  auto guarded = predicted_target;
  const auto prior = guarded.ekf();
  if (
    guarded.update(candidate, 0, posterior_distance_error / 2.0) ||
    !guarded.ekf_x().isApprox(prior.x) || !guarded.ekf().P.isApprox(prior.P) ||
    guarded.ekf().data != prior.data ||
    guarded.ekf().recent_nis_failures != prior.recent_nis_failures ||
    guarded.last_id != predicted_target.last_id || guarded.jumped)
  {
    std::cerr << "后验距离越界未完整回滚\n";
    return 1;
  }
  if (!guarded.update(far_armor, 0, 0.45)) {
    std::cerr << "后验拒绝后正常观测不能恢复更新\n";
    return 1;
  }

  Eigen::VectorXd constrained_variance = initial_variance;
  constrained_variance[8] = expected_variance;
  constrained_variance[9] = expected_variance;
  auto_aim::Target unconstrained_target(
    armor, std::chrono::steady_clock::now(), expected_radius, 4, constrained_variance);
  auto_aim::Target diameter_guard_target(
    armor, std::chrono::steady_clock::now(), expected_radius, 4, constrained_variance);
  diameter_guard_target.set_geometry_constraint(
    expected_radius, 0.0, expected_variance, expected_variance);
  const auto guarded_prior = diameter_guard_target.ekf();
  auto radial_outlier = armor;
  // 只改变距离观测，构造位置残差增长大于真实旋转直径、但小于原距离噪声门限的情形。
  radial_outlier.ypd_in_world[2] += 1.5;
  if (!unconstrained_target.update(radial_outlier, 0) ||
      diameter_guard_target.update(radial_outlier, 0)) {
    std::cerr << "已知几何目标未按旋转直径拦截异常后验\n";
    return 1;
  }
  if (!diameter_guard_target.ekf_x().isApprox(guarded_prior.x) ||
      !diameter_guard_target.ekf().P.isApprox(guarded_prior.P)) {
    std::cerr << "旋转直径门限拒绝后未恢复 EKF 状态\n";
    return 1;
  }
  if (!diameter_guard_target.update(armor, 0)) {
    std::cerr << "旋转直径门限误拒正常观测\n";
    return 1;
  }

  std::cout << "target_geometry_constraint_test passed\n";
  return 0;
}
