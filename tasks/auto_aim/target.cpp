#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();//初始化协方差矩阵，这里使用对角矩阵

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);//yaw角度限制在[-pi, pi]范围内
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
//   在这 `dt` 内，假设有一个恒定的随机加速度 `a_noise`（方差为 σ²，每步独立）：
// 位置: x' = x + v·dt + ½·a_noise·dt²
// 速度: v' = v + a_noise·dt
//              ↑                     ↑
//         (加速度对位置的影响)   (加速度对速度的影响)
// __这个随机加速度给位置和速度带来的不确定量：
// Δ位置 = ½·a_noise·dt²
// Δ速度 = a_noise·dt
// 成向量形式：`噪声 = [½dt²; dt] · a_noise`
// __协方差__ = 噪声×噪声ᵀ 的期望：
// Q_2x2 = E[ [½dt²]·a_noise · a_noise·[½dt², dt] ]
//            [ dt ]
//       = [½dt²]·E[a_noise²]·[½dt², dt]
//         [ dt ]
//       = σ² · [½dt²]·[½dt², dt]
//              [ dt ]
//       = σ² · [ dt²·dt²/4   dt·dt²/2 ]
//              [ dt·dt²/2    dt·dt     ]
//       = σ² · [ dt⁴/4   dt³/2 ]
//              [ dt³/2   dt²   ]





  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 10;   // 前哨站加速度方差
    v2 = 0.1;  // 前哨站角加速度方差
  } else {
    v1 = 100;  // 加速度方差
    v2 = 400;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);//yaw角度限制在[-pi, pi]范围内
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);//计算预测量和预测协方差矩阵
}//

std::vector<ArmorMatch> Target::match_armors(const Armor & armor) const
{
  std::vector<ArmorMatch> matches;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();
  const Eigen::Matrix4d R = measurement_noise(armor);

  for (int i = 0; i < armor_num_; i++) {
    const auto & xyza = xyza_list[i];
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    Eigen::Vector4d innovation{
      tools::limit_rad(armor.ypd_in_world[0] - ypd[0]),
    armor.ypd_in_world[1] - ypd[1],
    armor.ypd_in_world[2] - ypd[2],
    tools::limit_rad(armor.ypr_in_world[0] - xyza[3])};
    const auto H = h_jacobian(ekf_.x, i);
    const Eigen::Matrix4d S = H * ekf_.P * H.transpose() + R;
    Eigen::LDLT<Eigen::Matrix4d> ldlt(S);
    if (ldlt.info() != Eigen::Success) continue;

    const auto squared_distance = innovation.dot(ldlt.solve(innovation));
    if (!std::isfinite(squared_distance) || squared_distance < 0.0) continue;
    const auto mahalanobis_distance = std::sqrt(squared_distance);

    matches.push_back({i, angle_error, mahalanobis_distance, true});
  }

  return matches;
}

ArmorMatch Target::match_armor(const Armor & armor) const
{
  const auto matches = match_armors(armor);
  if (matches.empty()) return {};
  return *std::min_element(
    matches.begin(), matches.end(),
    [](const ArmorMatch & a, const ArmorMatch & b) {
      if (a.mahalanobis_distance != b.mahalanobis_distance) {
        return a.mahalanobis_distance < b.mahalanobis_distance;
      }
      return std::abs(a.angle_error) < std::abs(b.angle_error);
    });
}

void Target::update(const Armor & armor)
{
  update(armor, match_armor(armor).id);
}

void Target::update(const Armor & armor, int id)
{
  if (id < 0 || id >= armor_num_) return;

  if (id != 0) jumped = true;//如果id不为0，说明跳过了

  if (id != last_id) {
    is_switch_ = true;//如果id不等于上一次的id，说明切换了装甲板
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;//如果切换了装甲板，切换计数器加1

  last_id = id;
  update_count_++;//更新计数器加1

  update_ypda(armor, id);//
}

void Target::set_geometry_constraint(
  double radius, double radius_delta, double radius_variance,
  double radius_delta_variance)
{
  if (radius <= 0.0 || radius_variance <= 0.0 || radius_delta_variance <= 0.0) {
    throw std::invalid_argument("Invalid target geometry constraint");
  }

  geometry_constraint_ =
    GeometryConstraint{radius, radius_delta, radius_variance, radius_delta_variance};
  apply_geometry_constraint();
}

void Target::set_measurement_bearing_variance(double variance)
{
  if (!std::isfinite(variance) || variance <= 0.0) {
    throw std::invalid_argument("Invalid measurement bearing variance");
  }
  measurement_bearing_variance_ = variance;
}

void Target::apply_geometry_constraint()
{
  if (!geometry_constraint_.has_value()) return;

  const auto & constraint = geometry_constraint_.value();
  auto constrain_state = [&](Eigen::Index index, double value, double variance) {
    ekf_.x[index] = value;
    // 已知机械尺寸不应继续与中心、速度和角速度共同漂移，因此同时清除互协方差。
    ekf_.P.row(index).setZero();
    ekf_.P.col(index).setZero();
    ekf_.P(index, index) = variance;
  };

  constrain_state(8, constraint.radius, constraint.radius_variance);
  constrain_state(9, constraint.radius_delta, constraint.radius_delta_variance);
}

void Target::update_ypda(const Armor & armor, int id)
{
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  // 关联和 EKF 更新必须使用同一套观测噪声，否则马氏距离会与实际更新不一致。
  const Eigen::Matrix4d R = measurement_noise(armor);

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  ekf_.update(z, H, R, h, z_subtract);
  // 观测更新可能再次把不可观测半径拉向发散下限，更新后恢复已知机械尺寸。
  apply_geometry_constraint();
}

Eigen::Matrix4d Target::measurement_noise(const Armor & armor) const
{
  const auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  const auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  R.diagonal() << measurement_bearing_variance_, 4e-3,
    std::log(std::abs(delta_angle) + 1) + 1,
    std::log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2;
  return R;
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);//计算出装甲板中心的坐标（考虑长短轴）
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;//返回装甲板中心坐标
}//一辆车的所有装甲板中心坐标

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;//判断r是否在合理范围内
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;//l是两组装甲板的半径差值，所以r+l也在合理范围内

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}//判断是否发散

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}//收敛条件：普通装甲板连续更新3次且不发散，前哨站连续更新10次且不发散

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);//当有4个装甲板，id为1或3时，使用长短轴

  auto r = (use_l_h) ? x[8] + x[9] : x[8];//r是装甲板的半径，x[8]是半径，x[9]是半径差值，当有4个装甲板时，使用长短轴，半径为r+l
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];//当有4个装甲板，id为1或3时，使用长短轴，z坐标为z+h

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}//由装甲板中心坐标到观测量的雅可比矩阵H = H_armor_ypda * H_armor_xyza

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
