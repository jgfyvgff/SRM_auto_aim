#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <iostream>
#include <memory>

#include "src/sim_gimbal_tf.hpp"

int main()
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf2_ros::Buffer buffer(clock);
  // lookup_rotation 使用有限等待；测试中的 Buffer 也要声明其满足 tf2 线程契约。
  buffer.setUsingDedicatedThread(true);
  if (sim_gimbal_tf::lookup_rotation(buffer, rclcpp::Time(100, 0, RCL_ROS_TIME))) {
    std::cerr << "缺失 TF 时仍返回云台姿态\n";
    return 1;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "gimbal_link";
  transform.header.stamp.sec = 100;
  transform.transform.rotation.w = 1.0;
  if (!buffer.setTransform(transform, "sim_gimbal_tf_test")) {
    std::cerr << "无法注入第一帧 TF\n";
    return 1;
  }

  // 20 ms 转过 90 度；图像在中点时应由 tf2 插值得到 45 度。
  transform.header.stamp.nanosec = 20000000U;
  const Eigen::Quaterniond second(
    Eigen::AngleAxisd(3.14159265358979323846 / 2.0, Eigen::Vector3d::UnitZ()));
  transform.transform.rotation.x = second.x();
  transform.transform.rotation.y = second.y();
  transform.transform.rotation.z = second.z();
  transform.transform.rotation.w = second.w();
  if (!buffer.setTransform(transform, "sim_gimbal_tf_test")) {
    std::cerr << "无法注入第二帧 TF\n";
    return 1;
  }

  const auto midpoint = sim_gimbal_tf::lookup_rotation(
    buffer, rclcpp::Time(100, 10000000U, RCL_ROS_TIME));
  const Eigen::Vector3d expected(
    std::sqrt(0.5), std::sqrt(0.5), 0.0);
  if (
    !midpoint ||
    (midpoint->rotation * Eigen::Vector3d::UnitX() - expected).norm() > 1e-5 ||
    std::abs(midpoint->skew_ms) > 1e-6)
  {
    std::cerr << "TF 插值或 gimbal_link -> odom 旋转方向错误\n";
    return 1;
  }

  const auto nearest = sim_gimbal_tf::lookup_rotation(
    buffer, rclcpp::Time(100, 23000000U, RCL_ROS_TIME));
  if (!nearest || std::abs(nearest->skew_ms - 3.0) > 1e-6) {
    std::cerr << "短时间差 TF 回退失败\n";
    return 1;
  }
  if (sim_gimbal_tf::lookup_rotation(
      buffer, rclcpp::Time(100, 30000000U, RCL_ROS_TIME)))
  {
    std::cerr << "时间错位过大的 TF 未被拒绝\n";
    return 1;
  }

  geometry_msgs::msg::Quaternion invalid;
  invalid.w = 0.0;
  if (sim_gimbal_tf::normalized_rotation(invalid)) {
    std::cerr << "零四元数未被拒绝\n";
    return 1;
  }
  return 0;
}
