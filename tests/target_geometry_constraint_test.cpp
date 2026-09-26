#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"

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

  std::cout << "target_geometry_constraint_test passed\n";
  return 0;
}
