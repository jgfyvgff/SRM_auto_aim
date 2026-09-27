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

  std::cout << "target_geometry_constraint_test passed\n";
  return 0;
}
