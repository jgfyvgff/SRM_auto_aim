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
  const Eigen::MatrixXd expected_innovation_covariance = 2.0 * Eigen::MatrixXd::Identity(4, 4);
  const Eigen::MatrixXd expected_kalman_gain = 0.5 * Eigen::MatrixXd::Identity(4, 4);
  if (
    !nearly_equal(normal_filter.last_nis, 1.125) ||
    normal_filter.recent_nis_failures.back() != 0 ||
    (normal_filter.last_innovation - normal_measurement).norm() > 1e-9 ||
    (normal_filter.last_measurement_noise - R4).norm() > 1e-9 ||
    (normal_filter.last_innovation_covariance - expected_innovation_covariance).norm() > 1e-9 ||
    (normal_filter.last_kalman_gain - expected_kalman_gain).norm() > 1e-9)
  {
    std::cerr << "正常创新判定或 EKF 更新诊断量错误\n";
    return 1;
  }

  // 降低距离更新增益时仍用基准 R 做 NIS，避免倍率本身放宽 Tracker 的漏检灵敏度。
  auto distance_scaled_filter = make_filter(4);
  Eigen::VectorXd distance_measurement = Eigen::VectorXd::Zero(4);
  distance_measurement[2] = 2.0;
  Eigen::MatrixXd distance_update_R = R4;
  distance_update_R(2, 2) = 9.0;
  const Eigen::MatrixXd distance_nis_R = R4;
  const auto subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    return a - b;
  };
  distance_scaled_filter.update(
    distance_measurement, H4, distance_update_R,
    [](const Eigen::VectorXd & x) { return x; }, subtract, &distance_nis_R);
  if (
    !nearly_equal(distance_scaled_filter.last_kalman_gain(2, 2), 0.1) ||
    !nearly_equal(distance_scaled_filter.last_nis, 2.0) ||
    distance_scaled_filter.recent_nis_failures.back() != 0 ||
    (distance_scaled_filter.last_measurement_noise - distance_update_R).norm() > 1e-9 ||
    (distance_scaled_filter.last_nis_measurement_noise - R4).norm() > 1e-9)
  {
    std::cerr << "距离更新 R 与 NIS 基准 R 未正确解耦\n";
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
