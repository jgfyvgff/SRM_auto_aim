#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  // 新真机只读入口可记录未收敛但有限的轨迹；只有收敛结果允许 control=true。
  bool diagnostic_valid = false;
  bool solver_converged = false;
};

// 云台当前状态属于 world 绝对角，单位 rad、rad/s；与轨迹参考共用坐标和符号约定。
struct PlannerState
{
  double yaw = 0.0;
  double yaw_vel = 0.0;
  double pitch = 0.0;
  double pitch_vel = 0.0;
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  Planner(const std::string & config_path);

  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);
  // 真机诊断入口：target 应由调用方预测到当前计算时刻；首个规划点使用实测云台状态。
  Plan plan(Target target, double bullet_speed, const PlannerState & state);

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
  Trajectory get_trajectory(
    Target & target, double yaw0, double bullet_speed, bool start_at_now = false);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
