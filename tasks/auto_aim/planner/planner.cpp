#include "planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
  // 老配置没有这个键时保持原来的 10 次，避免其他入口因缺键起不来；上限小于 1 时
  // TinyMPC 会直接返回"达上限"，等于永不解算，所以这里兜到 1。
  max_iter_ = yaml["planner_max_iter"].IsDefined() ? yaml["planner_max_iter"].as<int>() : 10;
  if (max_iter_ < 1) max_iter_ = 1;

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  target.predict(bullet_traj.fly_time);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  auto shoot_offset_ = 2;
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) return {false};

  // Target 内部保存观测时间；直接预测到当前实际时刻，避免使用固定高低速延迟。
  target->predict(std::chrono::steady_clock::now());

  return plan(*target, bullet_speed);
}

Plan Planner::plan(Target target, double bullet_speed, const PlannerState & state)
{
  // 真机链路只接受调用方明确给出的有效弹速，避免原入口的 22 m/s 隐式回退进入控制准备。
  if (bullet_speed < 10.0 || bullet_speed > 25.0 ||
      !std::isfinite(bullet_speed) || !std::isfinite(state.yaw) ||
      !std::isfinite(state.yaw_vel) || !std::isfinite(state.pitch) ||
      !std::isfinite(state.pitch_vel)) {
    return {false};
  }

  const auto armors = target.armor_xyza_list();
  if (armors.empty()) return {false};
  double min_dist = 1e10;
  Eigen::Vector3d nearest_xyz = Eigen::Vector3d::Zero();
  for (const auto & xyza : armors) {
    const double dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      nearest_xyz = xyza.head<3>();
    }
  }
  const auto flight = tools::Trajectory(bullet_speed, min_dist, nearest_xyz.z());
  if (flight.unsolvable || !std::isfinite(flight.fly_time)) return {false};
  target.predict(flight.fly_time);

  try {
    const double yaw0 = aim(target, bullet_speed)(0);
    // 旧入口的参考轨迹以当前时刻为中点；本入口从当前时刻起算，x0 才能对应同一时刻。
    const auto traj = get_trajectory(target, yaw0, bullet_speed, true);
    if (!std::isfinite(yaw0) || !traj.allFinite()) return {false};
    Eigen::Vector2d yaw_x0(tools::limit_rad(state.yaw - yaw0), state.yaw_vel);
    Eigen::Vector2d pitch_x0(state.pitch, state.pitch_vel);
    if (tiny_set_x0(yaw_solver_, yaw_x0) != 0) return {false};
    yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
    const int yaw_status = tiny_solve(yaw_solver_);
    if (tiny_set_x0(pitch_solver_, pitch_x0) != 0) return {false};
    pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
    const int pitch_status = tiny_solve(pitch_solver_);

    // 发送协议只有绝对角；第 1 个规划步是当前状态之后 10 ms 的角度诊断。
    constexpr int next_step = 1;
    constexpr int diagnostic_step = 10;
    Plan plan{};
    plan.yaw_solver_status = yaw_status;
    plan.pitch_solver_status = pitch_status;
    plan.yaw_solver_iterations = yaw_solver_->solution->iter;
    plan.pitch_solver_iterations = pitch_solver_->solution->iter;
    // TinyMPC 的状态/输入残差分别计算，记录较大者，便于定位未收敛的轴。
    plan.yaw_primal_residual_max = std::max(
        yaw_solver_->work->primal_residual_state,
        yaw_solver_->work->primal_residual_input);
    plan.yaw_dual_residual_max = std::max(
        yaw_solver_->work->dual_residual_state,
        yaw_solver_->work->dual_residual_input);
    plan.pitch_primal_residual_max = std::max(
        pitch_solver_->work->primal_residual_state,
        pitch_solver_->work->primal_residual_input);
    plan.pitch_dual_residual_max = std::max(
        pitch_solver_->work->dual_residual_state,
        pitch_solver_->work->dual_residual_input);
    plan.target_yaw = tools::limit_rad(traj(0, next_step) + yaw0);
    plan.target_pitch = traj(2, next_step);
    plan.target_yaw_100ms = tools::limit_rad(traj(0, diagnostic_step) + yaw0);
    plan.target_pitch_100ms = traj(2, diagnostic_step);
    plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, next_step) + yaw0);
    plan.yaw_100ms = tools::limit_rad(yaw_solver_->work->x(0, diagnostic_step) + yaw0);
    plan.yaw_vel = yaw_solver_->work->x(1, next_step);
    plan.yaw_acc = yaw_solver_->work->u(0, 0);
    plan.pitch = pitch_solver_->work->x(0, next_step);
    plan.pitch_100ms = pitch_solver_->work->x(0, diagnostic_step);
    plan.pitch_vel = pitch_solver_->work->x(1, next_step);
    plan.pitch_acc = pitch_solver_->work->u(0, 0);
    plan.fire = false;
    plan.diagnostic_valid = std::isfinite(plan.yaw) && std::isfinite(plan.pitch) &&
                            std::isfinite(plan.yaw_vel) && std::isfinite(plan.pitch_vel) &&
                            std::isfinite(plan.yaw_acc) && std::isfinite(plan.pitch_acc);
    plan.solver_converged = yaw_status == 0 && pitch_status == 0;
    // 未收敛的 TinyMPC 工作区仍可能有有限数值，只允许只读诊断使用。
    plan.control = plan.diagnostic_valid && plan.solver_converged;
    return plan;
  } catch (const std::exception & e) {
    tools::logger()->warn("Planner current-state solve failed: {}", e.what());
    return {false};
  }
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  // 上限可配：写死 10 时 yaw 长期顶在上限，是否收敛取决于运气。
  yaw_solver_->settings->max_iter = max_iter_;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  // 上限可配：写死 10 时 yaw 长期顶在上限，是否收敛取决于运气。
  pitch_solver_->settings->max_iter = max_iter_;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}//得到目标的yaw和pitch角度，yaw为世界坐标系下的偏航角，pitch为世界坐标系下的俯仰角

Trajectory Planner::get_trajectory(
  Target & target, double yaw0, double bullet_speed, bool start_at_now)
{
  Trajectory traj;

  target.predict(-DT * (start_at_now ? 1 : HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  // 旧入口第 50 列对应当前时刻；实测初态入口第 0 列对应当前时刻。
  target.predict(DT);
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

}  // namespace auto_aim
