#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <array>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
class Aimer;

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
  // 100 ms 轨迹仅用于只读对比；角度单位 rad，求解残差使用 TinyMPC 原始单位。
  float target_yaw_100ms = 0.0F;
  float target_pitch_100ms = 0.0F;
  float yaw_100ms = 0.0F;
  float pitch_100ms = 0.0F;
  int yaw_solver_status = -1;
  int pitch_solver_status = -1;
  int yaw_solver_iterations = 0;
  int pitch_solver_iterations = 0;
  double yaw_primal_residual_max = 0.0;
  double yaw_dual_residual_max = 0.0;
  double pitch_primal_residual_max = 0.0;
  double pitch_dual_residual_max = 0.0;
  // 新真机只读入口可记录未收敛但有限的轨迹；只有收敛结果允许 control=true。
  bool diagnostic_valid = false;
  bool solver_converged = false;
  // 从当前时刻算起，预测参考轨迹首次换板的 10 ms 步数；-1 表示窗口内未换板。
  int preview_switch_step = -1;
};

// 云台当前状态属于 world 绝对角，单位 rad、rad/s；与轨迹参考共用坐标和符号约定。
struct PlannerState
{
  double yaw = 0.0;
  double yaw_vel = 0.0;
  double pitch = 0.0;
  double pitch_vel = 0.0;
};

// 离线回放专用：逐 10 ms 保存未来绝对 yaw 参考和首次换板目标；不写回 Aimer 锁。
struct SwitchPreview
{
  bool valid = false;
  int switch_step = -1;
  int switch_id = -1;
  std::array<double, HORIZON> yaw_rad{};
  std::array<double, HORIZON> yaw_vel_rad_s{};
  // 与 yaw_rad 同列的装甲 ID，供离线拟合确认单侧差分没有跨板。
  std::array<int, HORIZON> armor_id{};
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  Planner(const std::string & config_path);

  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);
  // 传入 Aimer 时，以参考角及参考速度为规划初态，并沿未来窗口预览选板。
  // 不传 Aimer 的兼容入口仍使用实测初态；预览不会写回真实 Aimer 的锁。
  Plan plan(
    Target target, double bullet_speed, const PlannerState & state,
    std::optional<int> armor_id = std::nullopt,
    const Aimer * preview_aimer = nullptr,
    SwitchPreview * switch_preview = nullptr);
  // 仅用于离线比较：重建可确定的选板历史，返回居中窗口当前时刻的规划值。
  // 不设置 control，不允许结果进入真机命令路径。
  Plan plan_centered_diagnostic(
    Target target, double bullet_speed, int armor_id, const Aimer & preview_aimer);

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  // TinyMPC 迭代上限。真机日志显示 yaw 的迭代次数中位正好等于 10（撞上限），
  // 是未收敛帧的主要来源，因此做成可配参数以便不重编就能比较收敛率。
  int max_iter_;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  Eigen::Matrix<double, 2, 1> aim(
    const Target & target, double bullet_speed,
    std::optional<int> armor_id = std::nullopt);
  Trajectory get_trajectory(
    Target & target, double yaw0, double bullet_speed, bool start_at_now = false,
    std::optional<int> armor_id = std::nullopt,
    const Aimer * preview_aimer = nullptr, int * preview_switch_step = nullptr,
    int * preview_switch_id = nullptr,
    std::array<int, HORIZON> * preview_armor_ids = nullptr);
  std::optional<Trajectory> get_centered_trajectory(
    const Target & target, double yaw0, double bullet_speed,
    int armor_id, const Aimer & preview_aimer);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
