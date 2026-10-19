#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
  // 对应 Target::armor_xyza_list() 的下标，用于调试装甲板切换。
  int armor_id = -1;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  // Aimer 最终瞄准点相对于当前检测时刻的预测时间，单位：秒。
  double debug_prediction_dt = -1.0;
  // 记录本帧是否进入小陀螺/高速选板分支，避免将分析器的 spin 分类与 Aimer 模式混淆。
  bool debug_high_speed_mode = false;
  // 从观测时间到 Aimer 开始计算的实测延迟，单位：秒。
  // 该值不再来自高低速固定配置。
  double debug_delay_time = 0.0;
  double debug_base_prediction_dt = -1.0;
  double debug_fly_time = -1.0;

  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  // 真机入口使用此接口，按传入弹速计算，不触发旧调用路径的默认弹速替换。
  // bullet_speed 必须是有限且大于 0 的速度，单位为 m/s。
  io::Command aim_with_explicit_speed(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

private:
  io::Command aim_impl(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now, bool allow_legacy_speed_fallback);

  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  // 跨帧选板锁只由本帧最终有效的预测结果更新。
  int lock_id_ = -1;
  double decision_speed_;

  AimPoint choose_aim_point(const Target & target, int & lock_id);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
