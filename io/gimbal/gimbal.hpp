#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
//下位机发送给上位机的数据
struct __attribute__((packed)) GimbalToVision
//使用__attribute__((packed))可以避免编译器优化，使得结构体按照声明的顺序对齐
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) <= 64);
/*
mode	-	0=空闲, 1=仅控制云台, 2=控制+开火
yaw/pitch	rad	目标角度
yaw_vel/pitch_vel	rad/s	速度前馈 → 让电控提前知道要跑多快
yaw_acc/pitch_acc	rad/s²	加速度前馈 → 让电控提前知道需要多大扭矩

__为什么需要前馈？__

回到 README 中第4.3节提到的：

“我们计算了轨迹对应的速度和加速度，作为前馈量一并发送给电控，提高云台的响应。”

传统 PID 的问题：


给定角度 → PID 发现误差 → 慢慢追上 → 滞后

带前馈的控制：

给定角度 + 速度 + 加速度 → 电控提前加力矩 → 几乎无滞后
      ↑                       ↑
  视觉规划器算出来的      下位机直接用


*/

//上位机发送给下位机的数据
struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) <= 64);//确保结构体大小不超过64字节

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP