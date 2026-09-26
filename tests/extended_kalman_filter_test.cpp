#include <Eigen/Dense>

#include <cmath>
#include <iostream>

#include "tools/extended_kalman_filter.hpp"

namespace
{
tools::ExtendedKalmanFilter make_filter(Eigen::Index dimension)
{
  return tools::ExtendedKalmanFilter(
    Eigen::VectorXd::Zero(dimension), Eigen::MatrixXd::Identity(dimension, dimension));
}

bool nearly_equal(double lhs, double rhs, double tolerance = 1e-9)
{
  return std::abs(lhs - rhs) <= tolerance;
}
}  // namespace

int main()
{
  const Eigen::MatrixXd H4 = Eigen::MatrixXd::Identity(4, 4);
  const Eigen::MatrixXd R4 = Eigen::MatrixXd::Identity(4, 4);

  // 四维观测的 NIS=1.125，小于 95% 上界 9.488，不应被旧阈值 0.711 误判。
  auto normal_filter = make_filter(4);
  Eigen::VectorXd normal_measurement = Eigen::VectorXd::Zero(4);
  normal_measurement[0] = 1.5;
  normal_filter.update(normal_measurement, H4, R4);
  if (
    !nearly_equal(normal_filter.last_nis, 1.125) ||
    normal_filter.recent_nis_failures.back() != 0)
  {
    std::cerr << "正常创新被错误判定为 NIS 失败\n";
    return 1;
  }

  // 四维观测的 NIS=12.5，超过 9.488，应被判定为异常创新。
  auto outlier_filter = make_filter(4);
  Eigen::VectorXd outlier_measurement = Eigen::VectorXd::Zero(4);
  outlier_measurement[0] = 5.0;
  outlier_filter.update(outlier_measurement, H4, R4);
  if (
    !nearly_equal(outlier_filter.last_nis, 12.5) ||
    outlier_filter.recent_nis_failures.back() != 1)
  {
    std::cerr << "异常创新未被 NIS 检测\n";
    return 1;
  }

  // 一维 EKF 也会复用该工具，验证动态自由度和调试残差不会越界。
  auto scalar_filter = make_filter(1);
  Eigen::VectorXd scalar_measurement(1);
  scalar_measurement << 1.0;
  scalar_filter.update(
    scalar_measurement, Eigen::MatrixXd::Identity(1, 1), Eigen::MatrixXd::Identity(1, 1));
  if (
    !nearly_equal(scalar_filter.last_nis, 0.5) ||
    !nearly_equal(scalar_filter.data.at("residual_pitch"), 0.0))
  {
    std::cerr << "一维观测的 NIS 或调试残差错误\n";
    return 1;
  }

  std::cout << "extended_kalman_filter_test passed\n";
  return 0;
}
