#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace tools
{
class ExtendedKalmanFilter
{
public:
  Eigen::VectorXd x;//状态
  Eigen::MatrixXd P;//协方差

  ExtendedKalmanFilter() = default;//表示默认构造函数，即没有参数的构造函数

  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });//有参数的构造函数

  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);

  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

  // R 同时用于增益和 NIS；离线隔离实验可额外传 NIS 基准 R，省略时保持原行为。
  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; },
    const Eigen::MatrixXd * nis_measurement_noise = nullptr);

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; },
    const Eigen::MatrixXd * nis_measurement_noise = nullptr);

  std::map<std::string, double> data;  //卡方检验数据
  std::deque<int> recent_nis_failures{0};
  size_t window_size = 100;
  double last_nis = 0.0;

  // 最近一次更新的中间量，仅用于追踪各观测分量对 EKF 状态修正的贡献。
  Eigen::VectorXd last_innovation;
  Eigen::MatrixXd last_measurement_noise;
  // 诊断卡方检验所用 R；与更新 R 分离时用于证明 NIS 标尺未随降权倍率改变。
  Eigen::MatrixXd last_nis_measurement_noise;
  Eigen::MatrixXd last_innovation_covariance;
  // 固定基准 R 对应的创新协方差，供离线跨降权档位比较。
  Eigen::MatrixXd last_nis_innovation_covariance;
  Eigen::MatrixXd last_kalman_gain;

private:
  Eigen::MatrixXd I;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;

  int nees_count_ = 0;
  int nis_count_ = 0;
  int total_count_ = 0;
};

}  // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP
