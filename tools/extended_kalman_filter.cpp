#include "extended_kalman_filter.hpp"

#include <array>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace
{
double chi_square_95_threshold(Eigen::Index degrees_of_freedom)
{
  // 常用自由度采用精确表值；更高维度使用 Wilson-Hilferty 近似。
  // NIS 的自由度等于观测维数，不能把四维自瞄观测的阈值硬编码给所有 EKF。
  constexpr std::array<double, 11> thresholds{
    0.0, 3.841, 5.991, 7.815, 9.488, 11.070,
    12.592, 14.067, 15.507, 16.919, 18.307};
  if (degrees_of_freedom <= 0) {
    throw std::invalid_argument("NIS requires a positive measurement dimension");
  }
  if (degrees_of_freedom < static_cast<Eigen::Index>(thresholds.size())) {
    return thresholds[degrees_of_freedom];
  }

  constexpr double normal_95_quantile = 1.6448536269514722;
  const double dof = static_cast<double>(degrees_of_freedom);
  const double approximation =
    1.0 - 2.0 / (9.0 * dof) + normal_95_quantile * std::sqrt(2.0 / (9.0 * dof));
  return dof * std::pow(approximation, 3);
}
}  // namespace

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}// 预测步骤：更新协方差矩阵P和状态向量x

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  const Eigen::VectorXd x_prior = x;
  const Eigen::MatrixXd P_prior = P;

  // NIS 必须基于更新前的创新计算；使用后验残差会低估观测与预测的不一致程度。
  const Eigen::VectorXd residual = z_subtract(z, h(x_prior));
  const Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
  const Eigen::MatrixXd K = P_prior * H.transpose() * S.inverse();

  last_innovation = residual;
  last_measurement_noise = R;
  last_innovation_covariance = S;
  last_kalman_gain = K;

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P =
    (I - K * H) * P_prior * (I - K * H).transpose() + K * R * K.transpose();//鲁棒性更高
  //约瑟夫稳定化形式

  x = x_add(x_prior, K * residual);

  /// 卡方检验
  double nis = residual.transpose() * S.inverse() * residual;
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  // 95% 上分位点用于拒绝过大的创新；四维自瞄观测对应 9.488。
  const double nis_threshold = chi_square_95_threshold(residual.size());
  constexpr double nees_threshold = 0.711;

  const bool nis_failed = nis > nis_threshold;
  data["nis_fail"] = nis_failed ? 1.0 : 0.0;
  if (nis_failed) nis_count_++;
  if (nees > nees_threshold) {
    nees_count_++;
    data["nees_fail"] = 1.0;
  } else {
    data["nees_fail"] = 0.0;
  }
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis_failed ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  // 该 EKF 也被一维、三维观测复用，缺失的调试分量统一记为零。
  auto residual_at = [&](Eigen::Index index) {
    return index < residual.size() ? residual[index] : 0.0;
  };
  data["residual_yaw"] = residual_at(0);
  data["residual_pitch"] = residual_at(1);
  data["residual_distance"] = residual_at(2);
  data["residual_angle"] = residual_at(3);
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools
