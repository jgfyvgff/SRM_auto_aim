#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <list>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tools/math_tools.hpp"
#include "src/sim_capture_time.hpp"
#include "src/sim_gimbal_tf.hpp"

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
    shooter_(tracker_config_path),
    tf_buffer_(get_clock()),
    // 使用 tf2 独立接收线程，避免 YOLO/PnP/窗口刷新阻塞 /tf 缓冲更新。
    // Listener 成员声明在 Buffer 后面，析构时先停止线程，再销毁 Buffer。
    tf_listener_(tf_buffer_, this, true),
    frame_count_(0)
  {
    // lookupTransform 的有限等待依赖独立 TF 线程；Listener 已启用 spin_thread。
    tf_buffer_.setUsingDedicatedThread(true);

    debug_publisher_ = create_publisher<std_msgs::msg::String>(
      "/sim_aim/debug", rclcpp::QoS(100).best_effort());
    tracker_status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/sim_aim/tracker_status", rclcpp::QoS(100).best_effort());
    gimbal_command_publisher_ =
      create_publisher<rm_interfaces::msg::GimbalCmd>(
        "/rm_gimbal/cmd", rclcpp::QoS(10).best_effort());

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&SimDetectorProbe::on_image, this, std::placeholders::_1));

    // 单线程执行器中，推理 Timer 与图像回调实际串行执行；
    // 这里保留最新帧是为了降低延迟，但不能保证图像回调不被推理耗时阻塞。
    inference_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&SimDetectorProbe::run_inference, this));
  }

private:
  // TF 尚未到达时暂存一帧；只保留这一帧，避免等待期间形成无界队列。
  struct PendingFrame
  {
    cv::Mat image;
    std::chrono::steady_clock::time_point timestamp;
    rclcpp::Time ros_stamp{0, 0, RCL_ROS_TIME};
    double header_age_ms = 0.0;
    std::uint64_t sequence = 0;
  };

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

  // 计算检测四角与 TF 真值四角的平均像素误差；只用于离线几何审计。
  static double mean_corner_error(
    const std::vector<cv::Point2f> & observed_points,
    const std::vector<cv::Point2f> & truth_points)
  {
    if (observed_points.size() != 4 || truth_points.size() != 4) {
      return -1.0;
    }

    double total_error = 0.0;
    for (std::size_t i = 0; i < observed_points.size(); ++i) {
      total_error += cv::norm(observed_points[i] - truth_points[i]);
    }
    return total_error / 4.0;
  }

  struct TruthArmorPose
  {
    int frame_id = -1;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
    double yaw = 0.0;
    double distance = 0.0;
    double position_error = 0.0;
    double second_position_error = std::numeric_limits<double>::infinity();
    double image_error = std::numeric_limits<double>::infinity();
    double second_image_error = std::numeric_limits<double>::infinity();
  };

  // 将候选中心投影到图像后与模拟器发布的 armor_N 真值逐一比较。
  // 这里只用于诊断，不把真值反馈给 Tracker，避免仿真真值污染算法。
  std::optional<TruthArmorPose> lookup_truth_armor(
    const rclcpp::Time & stamp, const Eigen::Vector3d & observed,
    const cv::Point2f & observed_center)
  {
    if (stamp.nanoseconds() <= 0 || !observed.allFinite()) {
      return std::nullopt;
    }

    std::optional<TruthArmorPose> nearest;
    constexpr int max_truth_frame_id = 64;
    for (int frame_id = 0; frame_id < max_truth_frame_id; ++frame_id) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          "odom", "armor_" + std::to_string(frame_id), tf2_ros::fromRclcpp(stamp));
        const Eigen::Vector3d position(
          transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z);
        const Eigen::Quaterniond orientation(
          transform.transform.rotation.w,
          transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z);
        if (!position.allFinite() || !orientation.coeffs().allFinite() ||
            orientation.norm() < 1e-6)
        {
          continue;
        }
        const auto normalized_orientation = orientation.normalized();

        const double position_error = (observed - position).norm();
        const auto projected_center = solver_.world2pixel({cv::Point3f(
          static_cast<float>(position.x()),
          static_cast<float>(position.y()),
          static_cast<float>(position.z()))});
        if (projected_center.size() != 1) {
          continue;
        }
        const double image_error = cv::norm(observed_center - projected_center.front());

        // 小陀螺时不同装甲板可能在世界坐标中很接近，世界坐标最近邻会误配。
        // 图像投影同时受当前云台姿态和相机模型约束，更适合判断检测对应的真值身份。
        if (!nearest || image_error < nearest->image_error ||
            (image_error == nearest->image_error &&
            position_error < nearest->position_error))
        {
          const auto previous_best_error =
            nearest ? nearest->position_error : std::numeric_limits<double>::infinity();
          const auto previous_best_image_error =
            nearest ? nearest->image_error : std::numeric_limits<double>::infinity();
          nearest = TruthArmorPose{
            frame_id,
            position,
            normalized_orientation.toRotationMatrix(),
            tools::eulers(normalized_orientation.toRotationMatrix(), 2, 1, 0)[0],
            position.norm(),
            position_error,
            previous_best_error,
            image_error,
            previous_best_image_error};
        } else {
          if (position_error < nearest->second_position_error) {
            nearest->second_position_error = position_error;
          }
          if (image_error < nearest->second_image_error) {
            nearest->second_image_error = image_error;
          }
        }
      } catch (const tf2::TransformException &) {
        // 仿真器不一定同时发布所有 armor_N，缺失帧不是算法错误。
      }
    }

    // 最近真值距离过大时不强行配对，避免把另一台机器人误当成真值。
    if (!nearest || nearest->position_error > 0.5) {
      return std::nullopt;
    }
    return nearest;
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
    const auto wall_received_at = std::chrono::system_clock::now();

    if (msg->encoding != "rgb8" || msg->step < msg->width * 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unsupported image: encoding=%s width=%u height=%u step=%u",
        msg->encoding.c_str(), msg->width, msg->height, msg->step);
      return;
    }

    const auto capture = sim_capture_time::convert_system_stamp(
      msg->header.stamp.sec, msg->header.stamp.nanosec,
      wall_received_at, frame_received_at);
    if (!capture) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring image with invalid, future or stale capture stamp");
      return;
    }

    cv::Mat rgb(
      static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()), msg->step);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

    std::lock_guard<std::mutex> lock(frame_mutex_);
    // GPU 异步回读可能乱序；不让旧图像把 Tracker 时间倒拨。
    if (
      latest_frame_timestamp_ != std::chrono::steady_clock::time_point{} &&
      capture->timestamp <= latest_frame_timestamp_)
    {
      return;
    }
    latest_frame_ = bgr.clone();
    ++latest_frame_sequence_;
    latest_frame_timestamp_ = capture->timestamp;
    latest_frame_ros_stamp_ = rclcpp::Time(msg->header.stamp);
    latest_header_age_ms_ = capture->header_age_ms;
  }

  void run_inference()
  {
    cv::Mat frame;
    std::chrono::steady_clock::time_point frame_timestamp;
    std::uint64_t frame_sequence = 0;
    rclcpp::Time frame_ros_stamp(0, 0, RCL_ROS_TIME);
    double frame_header_age_ms;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      // TF 暂不可用时，旧 pending 帧不能阻塞更新图像；
      // 实时探针优先处理最新帧，避免时间偏差随重试不断扩大。
      if (
        pending_frame_ && !latest_frame_.empty() &&
        latest_frame_sequence_ > pending_frame_->sequence)
      {
        pending_frame_.reset();
      }
      if (pending_frame_) {
        frame = std::move(pending_frame_->image);
        frame_timestamp = pending_frame_->timestamp;
        frame_sequence = pending_frame_->sequence;
        frame_ros_stamp = pending_frame_->ros_stamp;
        frame_header_age_ms = pending_frame_->header_age_ms;
        pending_frame_.reset();
      } else if (latest_frame_.empty()) {
        return;
      } else {
        frame = std::move(latest_frame_);
        frame_timestamp = latest_frame_timestamp_;
        frame_sequence = latest_frame_sequence_;
        frame_ros_stamp = latest_frame_ros_stamp_;
        frame_header_age_ms = latest_header_age_ms_;
      }
    }

    // TF 使用图像采集时间；缺失或错位时跳过整帧，不能混用旧云台姿态更新 EKF。
    std::string tf_failure_reason;
    const auto gimbal_tf = sim_gimbal_tf::lookup_rotation(
      tf_buffer_, frame_ros_stamp, &tf_failure_reason);
    if (!gimbal_tf) {
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        // 只有没有更新图像时才重试当前帧；有更新帧时直接丢弃旧帧，
        // 防止 pending_frame_ 无限占用推理循环。
        if (latest_frame_.empty() || latest_frame_sequence_ <= frame_sequence) {
          pending_frame_ = PendingFrame{
            std::move(frame), frame_timestamp, frame_ros_stamp,
            frame_header_age_ms, frame_sequence};
        }
      }
      ++tf_skipped_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping image without a time-aligned odom -> gimbal_link TF: %s",
        tf_failure_reason.c_str());
      return;
    }

    // 这些字段只用于区分“图像回调覆盖/跳过”与“Tracker 关联失败”，不参与算法决策。
    const double tracker_capture_dt_ms =
      last_tracked_frame_timestamp_ == std::chrono::steady_clock::time_point{}
        ? -1.0
        : duration_ms(last_tracked_frame_timestamp_, frame_timestamp);
    const auto received_frame_gap =
      last_tracked_frame_sequence_ == 0
        ? 0
        : frame_sequence - last_tracked_frame_sequence_;
    const auto tf_skipped_frames = tf_skipped_frames_;
    last_tracked_frame_timestamp_ = frame_timestamp;
    last_tracked_frame_sequence_ = frame_sequence;
    tf_skipped_frames_ = 0;

    solver_.set_R_gimbal2world(gimbal_tf->rotation);
    const Eigen::Vector3d gimbal_ypr = tools::eulers(solver_.R_gimbal2world(), 2, 1, 0);

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

    // 模拟器 odom 与云台原点重合；姿态已转换到 odom，Aimer 可使用同一原点求弹道。
    const auto tracker_start = std::chrono::steady_clock::now();
    const auto timestamp = frame_timestamp;
    const auto targets = tracker_.track(tracker_armors, timestamp);
    const auto tracker_end = std::chrono::steady_clock::now();

    // 状态 Topic 每个处理帧都发布，即使 Tracker 已经没有有效 target。
    // debug Topic 只在有 target 时发布完整几何数据，不能用于统计连续 temp_lost。
    // 这里使用 BEST_EFFORT，诊断消息绝不能反过来阻塞实时推理。
    // status_sequence 用于识别状态消息丢失；input_frame_sequence 只表示输入图像
    // 序号，可能因为“只保留最新帧”而跳变，不能拿它判断 DDS 是否丢状态消息。
    nlohmann::json tracker_status;
    tracker_status["status_sequence"] = tracker_status_sequence_++;
    tracker_status["input_frame_sequence"] = frame_sequence;
    tracker_status["stamp_ns"] = frame_ros_stamp.nanoseconds();
    tracker_status["tracker_state"] = tracker_.state();
    tracker_status["tracker_generation"] = tracker_.target_generation();
    tracker_status["temp_lost_count"] = tracker_.temp_lost_count();
    tracker_status["max_temp_lost_count"] = tracker_.max_temp_lost_count();
    tracker_status["target_present"] = targets.empty() ? 0 : 1;
    const auto & association_status = tracker_.association_debug();
    tracker_status["association_matching_detection_count"] =
      association_status.matching_detection_count;
    tracker_status["association_gate_passed_count"] =
      association_status.gate_passed_count;
    tracker_status["association_candidate_count"] = association_status.candidate_count;
    tracker_status["association_accepted_count"] = association_status.accepted_count;
    tracker_status["received_frame_gap"] = received_frame_gap;
    tracker_status["tf_skipped_frames"] = tf_skipped_frames;
    std_msgs::msg::String tracker_status_message;
    tracker_status_message.data = tracker_status.dump();
    tracker_status_publisher_->publish(tracker_status_message);

    // 必须与模拟器 projectile.speed=25.0 m/s 保持一致，
    // 否则自瞄预测弹道和实际弹丸轨迹会产生系统性偏差。
    constexpr double sim_bullet_speed_mps = 25.0;
    const auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer_.aim(targets, timestamp, sim_bullet_speed_mps);
    // Shooter 根据上一帧命令、当前云台 TF 和瞄准点稳定性决定是否开火。
    // 这里不强制 fire_advice，保持与真实自瞄链路相同的开火判定。
    command.shoot = shooter_.shoot(command, aimer_, targets, gimbal_ypr);
    const auto aimer_end = std::chrono::steady_clock::now();

    std::vector<cv::Point2f> current_reprojected_points;
    double current_reprojection_error = -1.0;
    double accepted_pnp_error = -1.0;
    double association_pnp_error = -1.0;
    double accepted_model_error = -1.0;
    std::vector<cv::Point2f> accepted_armor_points;
    double post_update_position_error = -1.0;
    double post_update_bearing_error = -1.0;
    double post_update_distance_error = -1.0;
    double post_update_orientation_error = -1.0;
    std::vector<cv::Point2f> reprojected_points;
    double future_center_shift = -1.0;
    std::vector<cv::Point2f> pnp_reprojected_points;
    std::vector<auto_aim::PnpCandidateDebug> pnp_candidates;
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
        const auto * observed_armor = static_cast<const auto_aim::Armor *>(nullptr);
        if (accepted.model_id >= 0) {
          const cv::Point2f observed_center(
            static_cast<float>(accepted.image_x),
            static_cast<float>(accepted.image_y));
          observed_armor = find_associated_armor(
            tracker_armors, target.name, target.armor_type, observed_center);
        }
        if (observed_armor != nullptr) {
          // 即使关联失败，也保留该观测的两个 IPPE 分支用于定位姿态歧义。
          pnp_candidates = solver_.pnp_candidates(*observed_armor);
          association_pnp_error = mean_reprojection_error(
            *observed_armor, solver_.reproject_pnp(*observed_armor));
        }
        if (accepted.accepted && accepted.model_id == target.last_id) {
          // Tracker 在副本上完成 PnP，后验残差必须读取该副本的解算位姿。
          current_armor = observed_armor;
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

    // 将 Aimer + Shooter 的最终结果送入模拟器真实订阅接口。
    // command 内部角度为弧度，rm_interfaces/GimbalCmd 使用角度。
    // distance=-1.0 表示当前没有有效目标，模拟器应清除旧瞄准状态。
    rm_interfaces::msg::GimbalCmd gimbal_command;
    gimbal_command.header.stamp = get_clock()->now();
    gimbal_command.header.frame_id = "gimbal_link";
    // Aimer 内部约定抬头为负，Daedalus GimbalCmd 约定 pitch>0 表示抬头。
    gimbal_command.pitch = -command.pitch * 180.0 / CV_PI;
    gimbal_command.yaw = command.yaw * 180.0 / CV_PI;
    gimbal_command.yaw_diff = 0.0;
    gimbal_command.pitch_diff = 0.0;
    // 模拟器不接收 command.control，只用 distance=-1.0 表示当前没有有效解。
    const bool has_valid_aim =
      command.control && !targets.empty() && target_distance >= 0.0;
    gimbal_command.distance = has_valid_aim ? target_distance : -1.0;
    gimbal_command.fire_advice = command.shoot;
    gimbal_command_publisher_->publish(gimbal_command);

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
        plot_data["target_armor_name"] = auto_aim::ARMOR_NAMES.at(target.name);
        plot_data["target_armor_type"] = auto_aim::ARMOR_TYPES.at(target.armor_type);
        plot_data["armor_pixel_long_side"] = armor_pixel_long_side;
        plot_data["armor_pixel_short_side"] = armor_pixel_short_side;
        plot_data["high_speed_mode"] = aimer_.debug_high_speed_mode ? 1 : 0;
        plot_data["delay_time"] = aimer_.debug_delay_time;
        plot_data["base_prediction_dt"] = aimer_.debug_base_prediction_dt;
        plot_data["fly_time"] = aimer_.debug_fly_time;
        plot_data["image_header_age_ms"] = frame_header_age_ms;
        plot_data["tracker_capture_dt_ms"] = tracker_capture_dt_ms;
        plot_data["received_frame_gap"] = received_frame_gap;
        plot_data["tf_skipped_frames"] = tf_skipped_frames;
        plot_data["gimbal_tf_skew_ms"] = gimbal_tf->skew_ms;
        plot_data["gimbal_yaw_rad"] = gimbal_ypr[0];
        plot_data["gimbal_pitch_rad"] = gimbal_ypr[1];
        // 保存完整云台姿态，供离线几何审计把 PnP 结果还原到 odom 坐标系；
        // 这些字段只用于复算，不改变在线 Solver 使用的姿态。
        plot_data["gimbal_tf_qx"] = gimbal_tf->rotation.x();
        plot_data["gimbal_tf_qy"] = gimbal_tf->rotation.y();
        plot_data["gimbal_tf_qz"] = gimbal_tf->rotation.z();
        plot_data["gimbal_tf_qw"] = gimbal_tf->rotation.w();
        plot_data["capture_to_detector_ms"] = duration_ms(frame_timestamp, detector_start);
        plot_data["detector_ms"] = duration_ms(detector_start, detector_end);
        plot_data["tracker_ms"] = duration_ms(tracker_start, tracker_end);
        plot_data["aimer_ms"] = duration_ms(aimer_start, aimer_end);
        plot_data["capture_to_aimer_ms"] = duration_ms(frame_timestamp, aimer_start);
        plot_data["pnp_error"] = pnp_reprojection_error;
        plot_data["association_pnp_error"] = association_pnp_error;
        plot_data["pnp_candidate_count"] =
          static_cast<int>(pnp_candidates.size());
        for (std::size_t i = 0; i < pnp_candidates.size() && i < 2; ++i) {
          const auto prefix = "pnp_candidate_" + std::to_string(i);
          plot_data[prefix + "_yaw"] = pnp_candidates[i].yaw_in_world;
          plot_data[prefix + "_x"] = pnp_candidates[i].xyz_in_world[0];
          plot_data[prefix + "_y"] = pnp_candidates[i].xyz_in_world[1];
          plot_data[prefix + "_z"] = pnp_candidates[i].xyz_in_world[2];
          plot_data[prefix + "_reprojection_error"] =
            pnp_candidates[i].reprojection_error;
        }
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
        const auto & update_debug = target.ekf();
        if (
          association.candidates[0].accepted && update_debug.last_innovation.size() == 4 &&
          update_debug.last_measurement_noise.rows() == 4 &&
          update_debug.last_innovation_covariance.rows() == 4 &&
          update_debug.last_kalman_gain.rows() >= 3 &&
          update_debug.last_kalman_gain.cols() == 4)
        {
          // 只记录本帧确实进入 EKF 的四维观测，定位各分量对车辆中心修正的贡献。
          for (Eigen::Index i = 0; i < 4; ++i) {
            const auto suffix = std::to_string(i);
            plot_data["ekf_innovation_" + suffix] = update_debug.last_innovation[i];
            plot_data["ekf_R_diagonal_" + suffix] = update_debug.last_measurement_noise(i, i);
            plot_data["ekf_S_diagonal_" + suffix] =
              update_debug.last_innovation_covariance(i, i);
            plot_data["ekf_center_x_correction_" + suffix] =
              update_debug.last_kalman_gain(0, i) * update_debug.last_innovation[i];
            plot_data["ekf_center_y_correction_" + suffix] =
              update_debug.last_kalman_gain(2, i) * update_debug.last_innovation[i];
          }
        }
        // temp_lost 只表示本帧关联未通过；记录各门限，避免盲目放宽全部阈值。
        if (tracker_.state() == "temp_lost") {
          const auto & primary = association.candidates[0];
          const double model_yaw =
            primary.model_id >= 0 &&
                static_cast<std::size_t>(primary.model_id) < current_armor_xyza_list.size()
            ? current_armor_xyza_list[primary.model_id][3]
            : std::numeric_limits<double>::quiet_NaN();
          RCLCPP_WARN(
            get_logger(),
            "[Probe] association rejected: candidates=%d primary_id=%d "
            "gate=%d angle=%d score=%d position=%d distance=%d mahalanobis=%d "
            "mahalanobis_distance=%.3f score=%.3f position_error=%.3f distance_error=%.3f "
            "raw_yaw=%.3f optimized_yaw=%.3f yaw_correction=%.3f model_yaw=%.3f "
            "orientation_error=%.3f bearing_error=%.3f",
            association.candidate_count,
            primary.model_id,
            primary.gate_passed,
            primary.angle_gate_passed,
            primary.score_gate_passed,
            primary.position_gate_passed,
            primary.distance_gate_passed,
            primary.mahalanobis_gate_passed,
            primary.mahalanobis_distance,
            primary.score,
            primary.position_error,
            primary.distance_error,
            primary.raw_yaw,
            primary.optimized_yaw,
            primary.yaw_correction,
            model_yaw,
            primary.orientation_error,
            primary.bearing_error);
        }
        plot_data["tracker_generation"] = tracker_.target_generation();
        plot_data["association_matching_detection_count"] =
          association.matching_detection_count;
        plot_data["association_gate_passed_count"] = association.gate_passed_count;
        plot_data["association_candidate_count"] = association.candidate_count;
        plot_data["association_accepted_count"] = association.accepted_count;
        auto add_association_candidate = [this, &plot_data, &frame_ros_stamp, &tracker_armors, &target](
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
          plot_data[prefix + "_angle_error"] = candidate.angle_error;
          plot_data[prefix + "_raw_yaw_prediction_error"] =
            candidate.raw_yaw_prediction_error;
          plot_data[prefix + "_optimized_yaw_prediction_error"] =
            candidate.optimized_yaw_prediction_error;
          plot_data[prefix + "_raw_yaw"] = candidate.raw_yaw;
          plot_data[prefix + "_optimized_yaw"] = candidate.optimized_yaw;
          plot_data[prefix + "_yaw_correction"] = candidate.yaw_correction;

          // 同时记录实际接受的主候选和被拒绝的候选：前者用于验证
          // PnP/外参/TF 真值是否一致，后者用于验证关联门限。
          // 真值只写入诊断消息，不反馈给 Tracker，也不改变任何算法决策。
          const auto truth = (candidate.accepted || !candidate.gate_passed)
            ? lookup_truth_armor(
              frame_ros_stamp,
              Eigen::Vector3d(candidate.observed_x, candidate.observed_y, candidate.observed_z),
              cv::Point2f(
                static_cast<float>(candidate.image_x),
                static_cast<float>(candidate.image_y)))
              : std::nullopt;
          plot_data[prefix + "_truth_corner_error"] = -1.0;
          for (int corner_id = 0; corner_id < 4; ++corner_id) {
            plot_data[prefix + "_truth_corner_error_" + std::to_string(corner_id)] = -1.0;
          }
          plot_data[prefix + "_truth_valid"] = truth ? 1 : 0;
          if (truth) {
            plot_data[prefix + "_truth_frame_id"] = truth->frame_id;
            plot_data[prefix + "_truth_x"] = truth->position[0];
            plot_data[prefix + "_truth_y"] = truth->position[1];
            plot_data[prefix + "_truth_z"] = truth->position[2];
            plot_data[prefix + "_truth_yaw"] = truth->yaw;
            plot_data[prefix + "_truth_distance"] = truth->distance;
            plot_data[prefix + "_truth_position_error"] = truth->position_error;
            plot_data[prefix + "_truth_second_position_error"] =
              truth->second_position_error;
            plot_data[prefix + "_truth_image_error"] = truth->image_error;
            plot_data[prefix + "_truth_second_image_error"] =
              truth->second_image_error;
            plot_data[prefix + "_truth_match_margin"] =
              truth->second_image_error - truth->image_error;
            const cv::Point2f observed_center(
              static_cast<float>(candidate.image_x),
              static_cast<float>(candidate.image_y));
            const auto * matched_armor = find_associated_armor(
              tracker_armors, target.name, target.armor_type, observed_center);
            if (matched_armor != nullptr) {
              const auto truth_points = solver_.reproject_armor_pose(
                truth->position, truth->orientation, target.armor_type);
              if (truth_points.size() == 4) {
                plot_data[prefix + "_truth_corner_error"] =
                  mean_corner_error(matched_armor->points, truth_points);
                for (std::size_t corner_id = 0; corner_id < truth_points.size(); ++corner_id) {
                  plot_data[prefix + "_truth_corner_error_" + std::to_string(corner_id)] =
                    cv::norm(matched_armor->points[corner_id] - truth_points[corner_id]);
                }
              }
            }
            // armor_N 的局部法向可能与 PnP 装甲板坐标相反，因此同时检查
            // 原始方向和绕法向翻转 pi 后的方向，不能把固定 pi 偏置误判为外参错误。
            const auto flipped_truth_yaw = tools::limit_rad(truth->yaw + CV_PI);
            plot_data[prefix + "_raw_yaw_truth_direct_error"] =
              std::abs(tools::limit_rad(candidate.raw_yaw - truth->yaw));
            plot_data[prefix + "_optimized_yaw_truth_direct_error"] =
              std::abs(tools::limit_rad(candidate.optimized_yaw - truth->yaw));
            plot_data[prefix + "_raw_yaw_truth_flipped_error"] =
              std::abs(tools::limit_rad(candidate.raw_yaw - flipped_truth_yaw));
            plot_data[prefix + "_optimized_yaw_truth_flipped_error"] =
              std::abs(tools::limit_rad(candidate.optimized_yaw - flipped_truth_yaw));
          }
          plot_data[prefix + "_image_x"] = candidate.image_x;
          plot_data[prefix + "_image_y"] = candidate.image_y;
        };
        add_association_candidate("association_primary", association.candidates[0]);
        add_association_candidate("association_secondary", association.candidates[1]);
        // 保留同一检测框的全部模型假设，便于判断初始 ID 是否被单一预测分支锁住。
        // 仅写入诊断话题，不参与 Tracker 的候选排序或控制输出。
        plot_data["association_model_candidates"] = nlohmann::json::array();
        for (int index = 0; index < association.model_candidate_count; ++index) {
          const auto & candidate = association.model_candidates[index];
          plot_data["association_model_candidates"].push_back({
            {"model_id", candidate.model_id},
            {"image_x", candidate.image_x},
            {"image_y", candidate.image_y},
            {"gate_passed", candidate.gate_passed},
            {"mahalanobis_distance", candidate.mahalanobis_distance},
            {"score", candidate.score},
            {"position_error", candidate.position_error},
            {"distance_error", candidate.distance_error},
            {"orientation_error", candidate.orientation_error},
            {"bearing_error", candidate.bearing_error},
            {"raw_yaw", candidate.raw_yaw},
            {"optimized_yaw", candidate.optimized_yaw}});
        }
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
  auto_aim::Shooter shooter_;
  // Listener 自带独立接收线程；先析构 Listener，再析构其引用的 Buffer。
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracker_status_publisher_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr
    gimbal_command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::TimerBase::SharedPtr inference_timer_;
  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  std::optional<PendingFrame> pending_frame_;
  std::uint64_t latest_frame_sequence_ = 0;
  std::chrono::steady_clock::time_point latest_frame_timestamp_;
  std::uint64_t last_tracked_frame_sequence_ = 0;
  std::chrono::steady_clock::time_point last_tracked_frame_timestamp_;
  std::uint64_t tf_skipped_frames_ = 0;
  std::uint64_t tracker_status_sequence_ = 0;
  rclcpp::Time latest_frame_ros_stamp_{0, 0, RCL_ROS_TIME};
  double latest_header_age_ms_ = 0.0;
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
