#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <optional>
#include <string>

namespace sim_gimbal_tf
{
// 只承认与图像采集时间足够接近的云台姿态；单位为毫秒。
// TF 有前后样本时由 tf2 插值，只有最新样本时允许最多 5 ms 的外推替代。
constexpr double max_nearest_skew_ms = 5.0;
// 图像时间点的 TF 可能晚于图像消息到达；最多等待 4 ms，不使用无限等待。
constexpr double max_lookup_wait_ms = 4.0;

struct RotationSample
{
  Eigen::Quaterniond rotation;
  double skew_ms = 0.0;
};

inline std::optional<Eigen::Quaterniond> normalized_rotation(
  const geometry_msgs::msg::Quaternion & rotation)
{
  Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6) {
    return std::nullopt;
  }
  return q.normalized();
}

// 返回 gimbal_link -> odom 的旋转；找不到有效 TF 时调用方应跳过该图像，
// 避免把不同坐标系的观测写入同一个 Tracker 状态。
inline std::optional<RotationSample> lookup_rotation(
  const tf2_ros::Buffer & buffer, const rclcpp::Time & image_stamp,
  std::string * failure_reason = nullptr)
{
  const auto fail = [&failure_reason](const std::string & reason) {
    if (failure_reason != nullptr) {
      *failure_reason = reason;
    }
    return std::nullopt;
  };

  if (image_stamp.nanoseconds() <= 0) {
    return fail("invalid image timestamp");
  }

  geometry_msgs::msg::TransformStamped transform;
  std::string exact_error;
  try {
    // 等待极短时间让未来 TF 进入缓存，避免把 DDS 到达顺序误判为 TF 缺失。
    transform = buffer.lookupTransform(
      "odom", "gimbal_link", image_stamp,
      rclcpp::Duration::from_nanoseconds(
        static_cast<int64_t>(max_lookup_wait_ms * 1.0e6)));
  } catch (const tf2::TransformException & exception) {
    exact_error = exception.what();
    try {
      // 最新 TF 仅作短时间差回退；绝不使用不受限的“最新姿态”。
      transform = buffer.lookupTransform("odom", "gimbal_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & latest_exception) {
      return fail("exact=" + exact_error + "; latest=" + latest_exception.what());
    }
  }

  const double skew_ms =
    (image_stamp - rclcpp::Time(transform.header.stamp)).seconds() * 1000.0;
  if (!std::isfinite(skew_ms) || std::abs(skew_ms) > max_nearest_skew_ms) {
    return fail("nearest TF skew=" + std::to_string(skew_ms) + " ms");
  }
  const auto rotation = normalized_rotation(transform.transform.rotation);
  if (!rotation) {
    return fail("invalid TF quaternion");
  }
  return RotationSample{*rotation, skew_ms};
}
}  // namespace sim_gimbal_tf
