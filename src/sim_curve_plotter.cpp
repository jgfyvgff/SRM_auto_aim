#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>

#include "tools/plotter.hpp"

class SimCurvePlotter final : public rclcpp::Node
{
public:
  SimCurvePlotter()
  : Node("sim_curve_plotter")
  {
    subscription_ = create_subscription<std_msgs::msg::String>(
      "/sim_aim/debug",
      rclcpp::QoS(100).best_effort(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        try {
          // 只负责把 ROS2 调试数据转发给 PlotJuggler。
          plotter_.plot(nlohmann::json::parse(message->data));
        } catch (const std::exception & exception) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "invalid debug JSON: %s",
            exception.what());
        }
      });
  }

private:
  tools::Plotter plotter_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimCurvePlotter>());
  rclcpp::shutdown();
  return 0;
}
