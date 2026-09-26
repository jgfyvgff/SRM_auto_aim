#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <cstdint>
#include <functional>

class SimDataProbe final : public rclcpp::Node
{
public:
  SimDataProbe()
  : Node("sim_data_probe"), image_count_(0)
  {
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&SimDataProbe::on_image, this, std::placeholders::_1));

    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&SimDataProbe::on_camera_info, this, std::placeholders::_1));

    rclcpp::QoS tf_qos(rclcpp::KeepLast(100));
    tf_qos.best_effort();
    tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", tf_qos,
      std::bind(&SimDataProbe::on_tf, this, std::placeholders::_1));
  }

private:
  // 验证 rgb8 消息能否安全转换为 OpenCV 使用的 BGR 图像。
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++image_count_;

    if (msg->encoding != "rgb8" || msg->step < msg->width * 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unexpected image: encoding=%s width=%u height=%u step=%u",
        msg->encoding.c_str(), msg->width, msg->height, msg->step);
      return;
    }

    cv::Mat rgb(
      static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()), msg->step);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    const cv::Scalar mean = cv::mean(bgr);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Image #%llu: %ux%u frame=%s mean=[%.1f, %.1f, %.1f]",
      static_cast<unsigned long long>(image_count_), msg->width, msg->height,
      msg->header.frame_id.c_str(), mean[0], mean[1], mean[2]);
  }

  // 检查相机内参是否与图像分辨率和仿真器配置一致。
  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (msg->k.size() != 9) {
      RCLCPP_WARN(get_logger(), "CameraInfo K has unexpected size: %zu", msg->k.size());
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "CameraInfo: %ux%u frame=%s fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
      msg->width, msg->height, msg->header.frame_id.c_str(),
      msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
  }

  // 检查自瞄所需的动态云台 TF 和静态相机外参是否存在。
  void on_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto & stamped : msg->transforms) {
      const auto & parent = stamped.header.frame_id;
      const auto & child = stamped.child_frame_id;

      if (
        (parent == "odom" && child == "gimbal_link") ||
        (parent == "gimbal_link" && child == "camera_link") ||
        (parent == "camera_link" && child == "camera_optical_frame")) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF %s -> %s: translation=[%.3f, %.3f, %.3f] rotation=[%.3f, %.3f, %.3f, %.3f]",
          parent.c_str(), child.c_str(),
          stamped.transform.translation.x,
          stamped.transform.translation.y,
          stamped.transform.translation.z,
          stamped.transform.rotation.x,
          stamped.transform.rotation.y,
          stamped.transform.rotation.z,
          stamped.transform.rotation.w);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  std::uint64_t image_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimDataProbe>());
  rclcpp::shutdown();
  return 0;
}
