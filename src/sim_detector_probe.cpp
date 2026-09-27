#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <list>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tools/math_tools.hpp"

class SimDetectorProbe final : public rclcpp::Node
{
public:
  SimDetectorProbe(
    const std::string & detector_config_path, const std::string & tracker_config_path)
  : Node("sim_detector_probe"),
    detector_(detector_config_path, false),
    solver_(tracker_config_path),
    tracker_(tracker_config_path, solver_),
    aimer_(tracker_config_path),
    frame_count_(0)
  {
    debug_publisher_ = create_publisher<std_msgs::msg::String>(
      "/sim_aim/debug", rclcpp::QoS(100).best_effort());

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&SimDetectorProbe::on_image, this, std::placeholders::_1));

    // 推理放在 Timer 中执行，避免在 ROS2 图像回调里阻塞 DDS 接收线程。
    inference_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&SimDetectorProbe::run_inference, this));
  }

private:
  // 探针直接调用 Tracker 时没有经过 Decider，因此这里补齐模式1的优先级，
  // 避免 Tracker 按未初始化的 priority 排序，导致两台机器人之间选择不确定。
  static auto_aim::ArmorPriority probe_priority(auto_aim::ArmorName name)
  {
    switch (name) {
      case auto_aim::ArmorName::three:
      case auto_aim::ArmorName::four:
        return auto_aim::ArmorPriority::first;
      case auto_aim::ArmorName::one:
        return auto_aim::ArmorPriority::second;
      case auto_aim::ArmorName::five:
      case auto_aim::ArmorName::sentry:
        return auto_aim::ArmorPriority::third;
      case auto_aim::ArmorName::two:
        return auto_aim::ArmorPriority::forth;
      case auto_aim::ArmorName::outpost:
      case auto_aim::ArmorName::base:
      case auto_aim::ArmorName::not_armor:
      default:
        return auto_aim::ArmorPriority::fifth;
    }
  }

  static void set_probe_priority(std::list<auto_aim::Armor> & armors)
  {
    for (auto & armor : armors) {
      armor.priority = probe_priority(armor.name);
    }
  }

  // 根据投影中心寻找当前帧中同名的原始装甲板，避免未来预测点与错误目标比较。
  static const auto_aim::Armor * find_nearest_armor(
    const std::list<auto_aim::Armor> & armors,
    auto_aim::ArmorName name,
    const std::vector<cv::Point2f> & projected_points)
  {
    if (projected_points.size() != 4) {
      return nullptr;
    }

    cv::Point2f projected_center(0.0F, 0.0F);
    for (const auto & point : projected_points) {
      projected_center += point;
    }
    projected_center *= 0.25F;

    const auto * matched_armor = static_cast<const auto_aim::Armor *>(nullptr);
    auto nearest_center_distance = std::numeric_limits<double>::max();
    for (const auto & armor : armors) {
      if (armor.name != name || armor.points.size() != 4) {
        continue;
      }

      const auto center_distance = cv::norm(armor.center - projected_center);
      if (center_distance < nearest_center_distance) {
        nearest_center_distance = center_distance;
        matched_armor = &armor;
      }
    }
    return matched_armor;
  }

  // 按 Tracker 记录的检测中心回查同一个框，避免多机器人同屏时最近邻配错。
  static const auto_aim::Armor * find_associated_armor(
    const std::list<auto_aim::Armor> & armors,
    auto_aim::ArmorName name,
    auto_aim::ArmorType type,
    const cv::Point2f & center)
  {
    const auto * matched = static_cast<const auto_aim::Armor *>(nullptr);
    auto min_center_error = std::numeric_limits<double>::max();
    for (const auto & armor : armors) {
      if (armor.name != name || armor.type != type || armor.points.size() != 4) {
        continue;
      }
      const auto error = cv::norm(armor.center - center);
      if (error < min_center_error) {
        min_center_error = error;
        matched = &armor;
      }
    }
    return min_center_error <= 0.01 ? matched : nullptr;
  }

  // 计算四个角点的平均像素误差。
  static double mean_reprojection_error(
    const auto_aim::Armor & armor, const std::vector<cv::Point2f> & projected_points)
  {
    if (armor.points.size() != 4 || projected_points.size() != 4) {
      return -1.0;
    }

    double total_error = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
      total_error += cv::norm(armor.points[i] - projected_points[i]);
    }
    return total_error / 4.0;
  }

  static double duration_ms(
    const std::chrono::steady_clock::time_point & begin,
    const std::chrono::steady_clock::time_point & end)
  {
    return std::chrono::duration<double, std::milli>(end - begin).count();
  }

  // 计算未来瞄准点相对当前检测装甲板中心的像素位移。
  // 未来点可能对应相邻装甲板，因此不能当作当前帧重投影误差。
  static double center_shift_pixels(
    const auto_aim::Armor & armor, const std::vector<cv::Point2f> & projected_points)
  {
    if (armor.points.size() != 4 || projected_points.size() != 4) {
      return -1.0;
    }

    cv::Point2f detected_center(0.0F, 0.0F);
    cv::Point2f projected_center(0.0F, 0.0F);
    for (std::size_t i = 0; i < 4; ++i) {
      detected_center += armor.points[i];
      projected_center += projected_points[i];
    }

    detected_center *= 0.25F;
    projected_center *= 0.25F;
    return cv::norm(detected_center - projected_center);
  }

  // 只保留最新帧，旧帧被覆盖，优先保证检测延迟而不是处理完整历史帧。
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const auto frame_received_at = std::chrono::steady_clock::now();

    if (msg->encoding != "rgb8" || msg->step < msg->width * 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unsupported image: encoding=%s width=%u height=%u step=%u",
        msg->encoding.c_str(), msg->width, msg->height, msg->step);
      return;
    }

    cv::Mat rgb(
      static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()), msg->step);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = bgr.clone();
    latest_frame_timestamp_ = frame_received_at;
  }

  void run_inference()
  {
    cv::Mat frame;
    std::chrono::steady_clock::time_point frame_timestamp;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (latest_frame_.empty()) {
        return;
      }
      frame = std::move(latest_frame_);
      frame_timestamp = latest_frame_timestamp_;
    }

    const auto detector_start = std::chrono::steady_clock::now();
    const auto armors = detector_.detect(frame, frame_count_++);
    const auto detector_end = std::chrono::steady_clock::now();
    // 使用副本交给 Tracker，保留原始检测结果用于窗口显示和类别核对。
    auto tracker_armors = armors;
    set_probe_priority(tracker_armors);

    for (const auto & armor : tracker_armors) {
      RCLCPP_INFO(
        get_logger(),
        "candidate color=%s name=%s priority=%d confidence=%.3f center=[%.1f, %.1f]",
        auto_aim::COLORS.at(armor.color).c_str(),
        auto_aim::ARMOR_NAMES.at(armor.name).c_str(),
        static_cast<int>(armor.priority),
        armor.confidence,
        armor.center.x,
        armor.center.y);
    }

    // 当前假设底盘不动，Target 坐标直接作为云台局部坐标交给 Aimer。
    const auto tracker_start = std::chrono::steady_clock::now();
    const auto timestamp = frame_timestamp;
    const auto targets = tracker_.track(tracker_armors, timestamp);
    const auto tracker_end = std::chrono::steady_clock::now();
    constexpr double sim_bullet_speed_mps = 23.0;
    const auto aimer_start = std::chrono::steady_clock::now();
    const auto command = aimer_.aim(targets, timestamp, sim_bullet_speed_mps);
    const auto aimer_end = std::chrono::steady_clock::now();

    std::vector<cv::Point2f> current_reprojected_points;
    double current_reprojection_error = -1.0;
    double accepted_pnp_error = -1.0;
    double accepted_model_error = -1.0;
    std::vector<cv::Point2f> accepted_armor_points;
    double post_update_position_error = -1.0;
    double post_update_bearing_error = -1.0;
    double post_update_distance_error = -1.0;
    double post_update_orientation_error = -1.0;
    std::vector<cv::Point2f> reprojected_points;
    double future_center_shift = -1.0;
    std::vector<cv::Point2f> pnp_reprojected_points;
    double pnp_reprojection_error = -1.0;
    double target_distance = -1.0;
    double armor_pixel_long_side = -1.0;
    double armor_pixel_short_side = -1.0;

    if (!targets.empty()) {
      const auto & target = targets.front();
      const auto target_state = target.ekf_x();
      if (target_state.size() >= 3) {
        // state[0] 和 state[2] 是水平面坐标，距离单位为 m。
        target_distance = std::hypot(target_state[0], target_state[2]);
      }

      // 当前 EKF 状态不包含弹道提前量，用来检查 Tracker 是否贴合当前帧。
      const auto current_armor_xyza_list = target.armor_xyza_list();
      if (
        target.last_id >= 0 &&
        static_cast<std::size_t>(target.last_id) < current_armor_xyza_list.size())
      {
        const auto & current_xyza = current_armor_xyza_list[target.last_id];
        current_reprojected_points = solver_.reproject_armor(
          current_xyza.head<3>(), current_xyza[3], target.armor_type, target.name);

        const auto & accepted = tracker_.association_debug().candidates[0];
        const auto * current_armor = static_cast<const auto_aim::Armor *>(nullptr);
        if (accepted.accepted && accepted.model_id == target.last_id) {
          const cv::Point2f accepted_center(
            static_cast<float>(accepted.image_x),
            static_cast<float>(accepted.image_y));
          // Tracker 在副本上完成 PnP，后验残差必须读取该副本的解算位姿。
          current_armor = find_associated_armor(
            tracker_armors, target.name, target.armor_type, accepted_center);
        }
        if (current_armor != nullptr) {
          // 只记录本帧实际进入 EKF 的角点，避免拿其他检测框分析 PnP。
          accepted_armor_points = current_armor->points;
          // 两条基线都使用本帧真正进入 EKF 的同一个检测框。
          accepted_pnp_error = mean_reprojection_error(
            *current_armor, solver_.reproject_pnp(*current_armor));
          accepted_model_error = mean_reprojection_error(
            *current_armor,
            solver_.reproject_armor(
              current_armor->xyz_in_world, current_armor->ypr_in_world[0],
              current_armor->type, current_armor->name));
          current_reprojection_error =
            mean_reprojection_error(*current_armor, current_reprojected_points);
          // 与 current_ekf_error 使用同一个已接受观测，量出 EKF 更新后的状态残差。
          // 这些字段只用于诊断，不参与关联、滤波或控制决策。
          const auto post_update_ypd = tools::xyz2ypd(current_xyza.head<3>());
          post_update_position_error =
            (current_armor->xyz_in_world - current_xyza.head<3>()).norm();
          post_update_bearing_error = std::abs(tools::limit_rad(
            current_armor->ypd_in_world[0] - post_update_ypd[0]));
          post_update_distance_error =
            std::abs(current_armor->ypd_in_world[2] - post_update_ypd[2]);
          post_update_orientation_error = std::abs(tools::limit_rad(
            current_armor->ypr_in_world[0] - current_xyza[3]));
          const auto rect_size = cv::minAreaRect(current_armor->points).size;
          armor_pixel_long_side = std::max(rect_size.width, rect_size.height);
          armor_pixel_short_side = std::min(rect_size.width, rect_size.height);
        }
      }

      // Aimer 的瞄准点包含检测延时、发弹延时和子弹飞行时间。
      if (aimer_.debug_aim_point.valid) {
        const auto & aim_xyza = aimer_.debug_aim_point.xyza;
        reprojected_points = solver_.reproject_armor(
          aim_xyza.head<3>(), aim_xyza[3], target.armor_type, target.name);

        const auto * matched_armor = find_nearest_armor(
          armors, target.name, reprojected_points);
        if (matched_armor != nullptr) {
          future_center_shift = center_shift_pixels(*matched_armor, reprojected_points);

          // 直接对当前原始检测执行 PnP 回投影，作为检测/PnP 层基线。
          pnp_reprojected_points = solver_.reproject_pnp(*matched_armor);
          pnp_reprojection_error =
            mean_reprojection_error(*matched_armor, pnp_reprojected_points);
        }
      }
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "YOLO frame=%d detected=%zu after_enemy_color=%zu tracker_state=%s targets=%zu",
      frame_count_ - 1, armors.size(), tracker_armors.size(), tracker_.state().c_str(),
      targets.size());

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "aimer control=%s yaw=%.2fdeg pitch=%.2fdeg aim_valid=%s",
      command.control ? "true" : "false",
      command.yaw * 180.0 / CV_PI,
      command.pitch * 180.0 / CV_PI,
      aimer_.debug_aim_point.valid ? "true" : "false");

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "projection pnp_error=%.2fpx current_ekf_error=%.2fpx "
      "future_center_shift=%.2fpx",
      pnp_reprojection_error,
      current_reprojection_error,
      future_center_shift);

    for (const auto & target : targets) {
      const auto state = target.ekf_x();
      if (state.size() < 8) {
        continue;
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "selected target name=%s priority=%d world_center=[%.3f, %.3f, %.3f] yaw=%.3f",
        auto_aim::ARMOR_NAMES.at(target.name).c_str(),
        static_cast<int>(target.priority),
        state[0],
        state[2],
        state[4],
        state[6]);

      const auto current_armor_xyza_list = target.armor_xyza_list();
      if (
        aimer_.debug_aim_point.valid &&
        target.last_id >= 0 &&
        static_cast<std::size_t>(target.last_id) < current_armor_xyza_list.size() &&
        aimer_.debug_aim_point.armor_id >= 0 &&
        static_cast<std::size_t>(aimer_.debug_aim_point.armor_id) < current_armor_xyza_list.size())
      {
        const auto & current_xyza = current_armor_xyza_list[target.last_id];
        const auto & aim_current_xyza =
          current_armor_xyza_list[aimer_.debug_aim_point.armor_id];
        const auto & future_xyza = aimer_.debug_aim_point.xyza;

        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "prediction vx=%.3f vy=%.3f vz=%.3f angular_velocity=%.3f "
          "prediction_dt=%.4fs current_armor_id=%d aim_armor_id=%d "
          "current_xyza=[%.3f, %.3f, %.3f, %.3f] "
          "future_xyza=[%.3f, %.3f, %.3f, %.3f]",
          state[1],
          state[3],
          state[5],
          state[7],
          aimer_.debug_prediction_dt,
          target.last_id,
          aimer_.debug_aim_point.armor_id,
          current_xyza[0],
          current_xyza[1],
          current_xyza[2],
          current_xyza[3],
          future_xyza[0],
          future_xyza[1],
          future_xyza[2],
          future_xyza[3]);

        // 探针只发布 JSON 调试数据，PlotJuggler 的 UDP 转发由独立节点完成。
        nlohmann::json plot_data;
        plot_data["vx"] = state[1];
        plot_data["vy"] = state[3];
        plot_data["vz"] = state[5];
        plot_data["angular_velocity"] = state[7];
        plot_data["prediction_dt"] = aimer_.debug_prediction_dt;
        // Command 内部使用弧度；同时发布角度值，便于与模拟器云台消息核对单位。
        plot_data["command_control"] = command.control ? 1 : 0;
        plot_data["command_shoot"] = command.shoot ? 1 : 0;
        plot_data["command_yaw"] = command.yaw;
        plot_data["command_pitch"] = command.pitch;
        plot_data["command_yaw_deg"] = command.yaw * 180.0 / CV_PI;
        plot_data["command_pitch_deg"] = command.pitch * 180.0 / CV_PI;
        plot_data["target_distance"] = target_distance;
        plot_data["armor_pixel_long_side"] = armor_pixel_long_side;
        plot_data["armor_pixel_short_side"] = armor_pixel_short_side;
        plot_data["high_speed_mode"] = aimer_.debug_high_speed_mode ? 1 : 0;
        plot_data["delay_time"] = aimer_.debug_delay_time;
        plot_data["base_prediction_dt"] = aimer_.debug_base_prediction_dt;
        plot_data["fly_time"] = aimer_.debug_fly_time;
        plot_data["capture_to_detector_ms"] = duration_ms(frame_timestamp, detector_start);
        plot_data["detector_ms"] = duration_ms(detector_start, detector_end);
        plot_data["tracker_ms"] = duration_ms(tracker_start, tracker_end);
        plot_data["aimer_ms"] = duration_ms(aimer_start, aimer_end);
        plot_data["capture_to_aimer_ms"] = duration_ms(frame_timestamp, aimer_start);
        plot_data["pnp_error"] = pnp_reprojection_error;
        plot_data["current_ekf_error"] = current_reprojection_error;
        plot_data["accepted_pnp_error"] = accepted_pnp_error;
        plot_data["accepted_model_error"] = accepted_model_error;
        if (accepted_armor_points.size() == 4) {
          for (std::size_t i = 0; i < accepted_armor_points.size(); ++i) {
            const auto prefix = "accepted_corner_" + std::to_string(i);
            plot_data[prefix + "_x"] = accepted_armor_points[i].x;
            plot_data[prefix + "_y"] = accepted_armor_points[i].y;
          }
        }
        plot_data["post_update_position_error"] = post_update_position_error;
        plot_data["post_update_bearing_error"] = post_update_bearing_error;
        plot_data["post_update_distance_error"] = post_update_distance_error;
        plot_data["post_update_orientation_error"] = post_update_orientation_error;
        plot_data["future_center_shift"] = future_center_shift;
        // NIS 使用更新前创新计算；失败率是 Tracker 判定收敛质量的直接依据。
        plot_data["nis"] = target.ekf().last_nis;
        plot_data["nis_failure_rate"] = target.ekf().data.at("recent_nis_failures");
        const auto & association = tracker_.association_debug();
        plot_data["tracker_generation"] = tracker_.target_generation();
        plot_data["association_candidate_count"] = association.candidate_count;
        plot_data["association_accepted_count"] = association.accepted_count;
        auto add_association_candidate = [&plot_data](
                                           const std::string & prefix,
                                           const auto_aim::AssociationCandidateDebug & candidate) {
          if (candidate.model_id < 0) return;
          plot_data[prefix + "_id"] = candidate.model_id;
          plot_data[prefix + "_gate_passed"] = candidate.gate_passed ? 1 : 0;
          plot_data[prefix + "_accepted"] = candidate.accepted ? 1 : 0;
          plot_data[prefix + "_angle_gate_passed"] =
            candidate.angle_gate_passed ? 1 : 0;
          plot_data[prefix + "_score_gate_passed"] =
            candidate.score_gate_passed ? 1 : 0;
          plot_data[prefix + "_position_gate_passed"] =
            candidate.position_gate_passed ? 1 : 0;
          plot_data[prefix + "_distance_gate_passed"] =
            candidate.distance_gate_passed ? 1 : 0;
          plot_data[prefix + "_mahalanobis_gate_passed"] =
            candidate.mahalanobis_gate_passed ? 1 : 0;
          plot_data[prefix + "_score"] = candidate.score;
          plot_data[prefix + "_position_error"] = candidate.position_error;
          plot_data[prefix + "_distance_error"] = candidate.distance_error;
          plot_data[prefix + "_mahalanobis_distance"] = candidate.mahalanobis_distance;
          plot_data[prefix + "_position_angle_error"] = candidate.position_angle_error;
          plot_data[prefix + "_distance_angle_error"] = candidate.distance_angle_error;
          plot_data[prefix + "_observed_x"] = candidate.observed_x;
          plot_data[prefix + "_observed_y"] = candidate.observed_y;
          plot_data[prefix + "_observed_z"] = candidate.observed_z;
          plot_data[prefix + "_predicted_x"] = candidate.predicted_x;
          plot_data[prefix + "_predicted_y"] = candidate.predicted_y;
          plot_data[prefix + "_predicted_z"] = candidate.predicted_z;
          plot_data[prefix + "_observed_distance"] = candidate.observed_distance;
          plot_data[prefix + "_predicted_distance"] = candidate.predicted_distance;
          plot_data[prefix + "_orientation_error"] = candidate.orientation_error;
          plot_data[prefix + "_bearing_error"] = candidate.bearing_error;
          plot_data[prefix + "_raw_yaw"] = candidate.raw_yaw;
          plot_data[prefix + "_optimized_yaw"] = candidate.optimized_yaw;
          plot_data[prefix + "_yaw_correction"] = candidate.yaw_correction;
          plot_data[prefix + "_image_x"] = candidate.image_x;
          plot_data[prefix + "_image_y"] = candidate.image_y;
        };
        add_association_candidate("association_primary", association.candidates[0]);
        add_association_candidate("association_secondary", association.candidates[1]);
        plot_data["current_armor_id"] = target.last_id;
        plot_data["aim_armor_id"] = aimer_.debug_aim_point.armor_id;
        // center_* 表示车辆旋转中心；current_* 表示当前关联装甲板的位置。
        plot_data["center_x"] = state[0];
        plot_data["center_y"] = state[2];
        plot_data["center_z"] = state[4];
        plot_data["center_speed"] = std::hypot(state[1], state[3]);
        // 四装甲模型交替使用 r 和 r+l，用于判断中心摆动是否来自半径估计。
        plot_data["radius"] = state[8];
        plot_data["radius_delta"] = state[9];
        plot_data["alternate_radius"] = state[8] + state[9];
        plot_data["height_delta"] = state[10];
        plot_data["current_x"] = current_xyza[0];
        plot_data["current_y"] = current_xyza[1];
        plot_data["current_z"] = current_xyza[2];
        plot_data["current_yaw"] = current_xyza[3];
        // aim_current_* 与 future_* 使用同一个模型 ID，避免装甲板切换时错误比较不同板。
        plot_data["aim_current_x"] = aim_current_xyza[0];
        plot_data["aim_current_y"] = aim_current_xyza[1];
        plot_data["aim_current_z"] = aim_current_xyza[2];
        plot_data["aim_current_yaw"] = aim_current_xyza[3];
        plot_data["future_x"] = future_xyza[0];
        plot_data["future_y"] = future_xyza[1];
        plot_data["future_z"] = future_xyza[2];
        plot_data["future_yaw"] = future_xyza[3];

        std_msgs::msg::String debug_message;
        debug_message.data = plot_data.dump();
        debug_publisher_->publish(debug_message);
      }
    }

    show_visualization(
      frame, armors,
      pnp_reprojected_points, pnp_reprojection_error,
      current_reprojected_points, current_reprojection_error,
      reprojected_points, future_center_shift);
  }

  void show_visualization(
    const cv::Mat & frame,
    const std::list<auto_aim::Armor> & armors,
    const std::vector<cv::Point2f> & pnp_reprojected_points,
    double pnp_reprojection_error,
    const std::vector<cv::Point2f> & current_reprojected_points,
    double current_reprojection_error,
    const std::vector<cv::Point2f> & reprojected_points,
    double future_center_shift)
  {
    cv::Mat visualization = frame.clone();
    for (const auto & armor : armors) {
      if (armor.points.size() < 3) {
        continue;
      }

      // 使用网络四点计算旋转矩形，保留装甲板的倾角信息。
      const auto min_rect = cv::minAreaRect(armor.points);
      cv::Point2f corners[4];
      min_rect.points(corners);

      for (int i = 0; i < 4; ++i) {
        cv::line(
          visualization, corners[i], corners[(i + 1) % 4],
          cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
      }

      for (const auto & point : armor.points) {
        cv::circle(visualization, point, 5, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
      }

      const cv::Point text_origin(
        static_cast<int>(std::lround(armor.center.x)),
        static_cast<int>(std::lround(armor.center.y)) - 10);
      const auto label =
        auto_aim::COLORS.at(armor.color) + " " +
        auto_aim::ARMOR_NAMES.at(armor.name) + " " +
        std::to_string(armor.confidence);
      cv::putText(
        visualization, label, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.8,
        cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }

    if (pnp_reprojected_points.size() >= 3) {
      // 青色表示“原始检测点 -> PnP -> 回投影”的基线误差。
      const auto pnp_rect = cv::minAreaRect(pnp_reprojected_points);
      cv::Point2f corners[4];
      pnp_rect.points(corners);
      for (int i = 0; i < 4; ++i) {
        cv::line(
          visualization, corners[i], corners[(i + 1) % 4],
          cv::Scalar(255, 255, 0), 3, cv::LINE_AA);
      }

      for (const auto & point : pnp_reprojected_points) {
        cv::circle(visualization, point, 5, cv::Scalar(255, 255, 0), -1, cv::LINE_AA);
      }

      if (pnp_reprojection_error >= 0.0) {
        cv::Point2f center(0.0F, 0.0F);
        for (const auto & point : pnp_reprojected_points) {
          center += point;
        }
        center *= 0.25F;
        cv::putText(
          visualization,
          "pnp " + std::to_string(pnp_reprojection_error) + " px",
          center,
          cv::FONT_HERSHEY_SIMPLEX,
          0.8,
          cv::Scalar(255, 255, 0),
          2,
          cv::LINE_AA);
      }
    }

    if (current_reprojected_points.size() >= 3) {
      // 蓝色表示当前 EKF 状态回投影，不包含弹道提前量。
      const auto current_rect = cv::minAreaRect(current_reprojected_points);
      cv::Point2f corners[4];
      current_rect.points(corners);
      for (int i = 0; i < 4; ++i) {
        cv::line(
          visualization, corners[i], corners[(i + 1) % 4],
          cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
      }

      for (const auto & point : current_reprojected_points) {
        cv::circle(visualization, point, 5, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
      }

      if (current_reprojection_error >= 0.0) {
        cv::Point2f center(0.0F, 0.0F);
        for (const auto & point : current_reprojected_points) {
          center += point;
        }
        center *= 0.25F;
        cv::putText(
          visualization,
          "ekf now " + std::to_string(current_reprojection_error) + " px",
          center,
          cv::FONT_HERSHEY_SIMPLEX,
          0.8,
          cv::Scalar(255, 0, 0),
          2,
          cv::LINE_AA);
      }
    }

    if (reprojected_points.size() >= 3) {
      // 洋红色表示包含延时和弹道预测的未来瞄准点。
      const auto reprojected_rect = cv::minAreaRect(reprojected_points);
      cv::Point2f corners[4];
      reprojected_rect.points(corners);
      for (int i = 0; i < 4; ++i) {
        cv::line(
          visualization, corners[i], corners[(i + 1) % 4],
          cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
      }

      for (const auto & point : reprojected_points) {
        cv::circle(visualization, point, 5, cv::Scalar(255, 0, 255), -1, cv::LINE_AA);
      }

      if (future_center_shift >= 0.0) {
        cv::Point2f center(0.0F, 0.0F);
        for (const auto & point : reprojected_points) {
          center += point;
        }
        center *= 0.25F;
        cv::putText(
          visualization,
          "future shift " + std::to_string(future_center_shift) + " px",
          center,
          cv::FONT_HERSHEY_SIMPLEX,
          0.8,
          cv::Scalar(255, 0, 255),
          2,
          cv::LINE_AA);
      }
    }

    cv::imshow("sim_detector", visualization);
    // waitKey 不仅读取键盘，也负责刷新 HighGUI 窗口。
    if (cv::waitKey(1) == 27) {
      rclcpp::shutdown();
    }
  }

  auto_aim::YOLO detector_;
  auto_aim::Solver solver_;
  auto_aim::Tracker tracker_;
  auto_aim::Aimer aimer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::TimerBase::SharedPtr inference_timer_;
  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  std::chrono::steady_clock::time_point latest_frame_timestamp_;
  int frame_count_;
};

int main(int argc, char ** argv)
{
  const std::string detector_config_path =
    argc > 1 ? argv[1] : "configs/standard3.yaml";
  const std::string tracker_config_path =
    argc > 2 ? argv[2] : "configs/demo.yaml";

  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<SimDetectorProbe>(detector_config_path, tracker_config_path));
  rclcpp::shutdown();
  return 0;
}
