#include "src/real_auto_aim/frame_processor.hpp"

#include <utility>

#include "src/real_auto_aim/frame_diagnostics.hpp"
#include "tools/math_tools.hpp"

namespace real_auto_aim
{
using namespace diagnostics;

FrameDetector::FrameDetector(const std::string & config_path)
    : detector_(config_path, false)
{}

FrameProcessor::FrameProcessor(const std::string & config_path)
    : solver_(config_path), tracker_(config_path, solver_),
      aimer_(config_path), planner_(config_path)
{}

FrameDetections FrameDetector::detect(const cv::Mat & image, Json & sample)
{
    FrameDetections result;
    const auto detector_start = Clock::now();
    auto armors = detector_.detect(image);
    append_detector_debug_json(sample, detector_.debug_info(), armors);
    const auto detector_end = Clock::now();
    const auto detected = armors.size();
    // Tracker 会原地过滤装甲板；诊断必须保留检测器原始结果，避免把过滤后的列表当作检测结果。
    const auto detected_armors_for_diagnostic = armors;
    if (!armors.empty()) {
        double confidence_sum = 0.0;
        double confidence_max = 0.0;
        const auto_aim::Armor * best_armor = nullptr;
        for (const auto & armor : armors) {
            confidence_sum += armor.confidence;
            if (confidence_max < armor.confidence) {
                confidence_max = armor.confidence;
                best_armor = &armor;
            }
        }
        sample["mean_confidence"] =
            confidence_sum / static_cast<double>(armors.size());
        sample["max_confidence"] = confidence_max;
        // 记录最高置信度检测的类别名与框宽。动态 ROI 的窗口大小由上一帧框宽决定，
        // 两者放在一起才能区分"先验窗口抖动"和"类别判错"：排查远距离误分类时，
        // 旧日志只有 radius，无法判断是检测错了还是被跟踪器的名字过滤拒绝。
        if (best_armor != nullptr) {
            sample["best_detection_name"] = auto_aim::ARMOR_NAMES[best_armor->name];
            sample["best_detection_width_px"] = best_armor->box.width;
        }
    } else {
        sample["mean_confidence"] = 0.0;
        sample["max_confidence"] = 0.0;
    }
    result.count = armors.size();
    result.armors = std::move(armors);
    result.diagnostic_armors = result.armors;
    result.detector_start = detector_start;
    result.detector_end = detector_end;
    return result;
}

FrameProcessResult FrameProcessor::process(
    FrameDetections detections, Clock::time_point image_timestamp,
    const MatchedFeedback & feedback, Json & sample,
    SerialFeedbackReader & serial_feedback, TxCommandController & tx_controller,
    double planner_debug_speed, bool replay_enabled,
    std::chrono::milliseconds max_image_age,
    const std::atomic<bool> & stop_requested)
{
    FrameProcessResult result;
    result.detections = std::move(detections);
    auto & armors = result.detections.armors;
    const auto detected = result.detections.count;
    const auto detector_start = result.detections.detector_start;
    const auto detector_end = result.detections.detector_end;
    const auto detected_armors_for_diagnostic = result.detections.diagnostic_armors;
    if (replay_enabled) {
        sample["replay_gimbal_to_world_w"] = feedback.gimbal_to_world.w();
        sample["replay_gimbal_to_world_x"] = feedback.gimbal_to_world.x();
        sample["replay_gimbal_to_world_y"] = feedback.gimbal_to_world.y();
        sample["replay_gimbal_to_world_z"] = feedback.gimbal_to_world.z();
    }
    solver_.set_gimbal_to_world(feedback.gimbal_to_world);
    // tracker_ms 必须只覆盖 tracker_.track：检测与反馈等待已移到它之前，
    // 不能再沿用 detector_end 作为起点，否则等待时间会被误计入 Tracker。
    const auto tracker_start = Clock::now();
    auto targets = tracker_.track(armors, image_timestamp);
    const auto tracker_end = Clock::now();
    const double bullet_speed = feedback.feedback.bullet_speed_mps;
    const bool measured_speed = bullet_speed >= 10.0 && bullet_speed <= 25.0;
    // if (!std::isfinite(bullet_speed) || bullet_speed < 14.0) {//弹速小于0,不能正常绘制检测框
    //简单验证无弹速的aimmer
    if (!std::isfinite(bullet_speed)) {
        sample["event"] = "skip";
        sample["skip_reason"] = "invalid bullet speed; Aimer default-speed fallback is disabled";
        sample["detected"] = detected;
        sample["tracker_state"] = tracker_.state();
        sample["tracker_generation"] = tracker_.target_generation();
        sample["detector_ms"] = duration_ms(detector_start, detector_end);
        sample["tracker_ms"] = duration_ms(tracker_start, tracker_end);
        append_target_json(sample, targets);
        append_reprojection_json(
            sample, solver_, detected_armors_for_diagnostic, targets);
        append_association_json(sample, tracker_.association_debug(), solver_);
        append_tracker_filter_json(sample, tracker_, targets);
        tx_controller.clear();
        result.invalid_bullet_speed = true;
        result.targets = targets;
        result.bullet_speed = bullet_speed;
        result.tracker_state = tracker_.state();
        result.tracker_generation = tracker_.target_generation();
        return result;
    }
    const auto aimer_start = Clock::now();
    // Aimer 只生成诊断结果：有效反馈原样使用；反馈无效时用配置的名义弹速继续诊断。
    // 这不改变 Planner/TX 的控制门控，且将实际输入速度和来源写入 JSONL 供离线核对。
    const double aimer_speed = measured_speed ? bullet_speed : planner_debug_speed;
    sample["aimer_speed_source"] =
        measured_speed ? "feedback" : "diagnostic_nominal";
    sample["aimer_bullet_speed_mps"] = aimer_speed;
    if (replay_enabled) {
        sample["replay_aimer_offset_ms"] =
            duration_ms(image_timestamp, aimer_start);
        sample["replay_aimer_bullet_speed_mps"] = aimer_speed;
    }
    const auto diagnostic =
        aimer_.aim_with_explicit_speed(targets, image_timestamp, aimer_speed);
    const auto finished_at = Clock::now();

    sample["planner_status"] = "no_tracking_target";
    sample["planner_control"] = 0;
    sample["planner_speed_source"] = nullptr;
    sample["planner_yaw_deg"] = nullptr;
    sample["planner_pitch_deg"] = nullptr;
    sample["tx_skip_reason"] = nullptr;
    // 排队和求解都消耗时间；发命令前再次核对图像年龄，避免旧观测驱动云台。
    const auto command_allowed = [&]() {
        const auto now = Clock::now();
        if (stop_requested.load() || now < image_timestamp ||
            now - image_timestamp > max_image_age) {
            tx_controller.clear();
            sample["tx_skip_reason"] = stop_requested.load() ?
                "pipeline_stopping" : "image_stale_before_tx";
            return false;
        }
        return true;
    };
    const auto planner_start = Clock::now();
    if (replay_enabled) {
        sample["replay_planner_offset_ms"] =
            duration_ms(image_timestamp, planner_start);
        sample["replay_planner_enabled"] = false;
    }
    // temp_lost 短暂丢帧时继续使用 Tracker 预测目标规划；超过上限后保持上一条命令。
    // temp_lost 外推直接使用 Tracker 预测角；fire_flag 始终为 0。
    const int temp_lost_frames = tracker_.temp_lost_count();
    const bool tx_extrapolating =
        tx_controller.follow_lost() && tracker_.state() == "temp_lost" &&
        temp_lost_frames <= tx_controller.lost_frames();
    if (
        !targets.empty() &&
        (tracker_.state() == "tracking" || tx_extrapolating))
    {
        const auto motion = serial_feedback.latest_motion();
        // 无真实弹速时仅计算诊断轨迹；不能据此驱动云台或申请开火。
        const double planning_speed = measured_speed ? bullet_speed : planner_debug_speed;
        sample["planner_speed_source"] =
            measured_speed ? "feedback" : "diagnostic_nominal";
        sample["planner_bullet_speed_mps"] = planning_speed;
        sample["tx_speed_source"] =
            measured_speed ? "feedback" :
            (tx_controller.nominal() ? "diagnostic_nominal" : "unavailable");

        auto_aim::PlannerState planner_state{
            feedback.feedback.yaw_deg * CV_PI / 180.0,
            0.0,
            feedback.feedback.pitch_deg * CV_PI / 180.0,
            0.0};
        bool has_measured_velocity = false;
        sample["planner_state_source"] = "matched_pose_zero_velocity";
        sample["planner_measured_yaw_deg"] = feedback.feedback.yaw_deg;
        sample["planner_measured_pitch_deg"] = feedback.feedback.pitch_deg;
        sample["planner_measured_yaw_vel_rad_s"] = 0.0;
        sample["planner_measured_pitch_vel_rad_s"] = 0.0;
        if (motion) {
            // 差分速度仅保留作诊断；参考轨迹规划不依赖它作为控制初态。
            // 串口只有主机收帧时间，不能视为下位机真实速度。
            const double motion_age_s =
                std::chrono::duration<double>(planner_start - motion->received_at).count();
            planner_state = {
                tools::limit_rad(
                    motion->yaw_rad + motion_age_s * motion->yaw_vel_rad_s),
                motion->yaw_vel_rad_s,
                motion->pitch_rad + motion_age_s * motion->pitch_vel_rad_s,
                motion->pitch_vel_rad_s};
            has_measured_velocity = true;
            sample["planner_state_source"] = "latest_pose_extrapolated";
            sample["planner_feedback_age_ms"] =
                duration_ms(motion->received_at, planner_start);
            sample["planner_feedback_interval_ms"] = motion->interval_ms;
            sample["planner_measured_yaw_deg"] = motion->yaw_rad * 180.0 / CV_PI;
            sample["planner_measured_pitch_deg"] = motion->pitch_rad * 180.0 / CV_PI;
            sample["planner_measured_yaw_vel_rad_s"] = motion->yaw_vel_rad_s;
            sample["planner_measured_pitch_vel_rad_s"] = motion->pitch_vel_rad_s;
        }
        if (replay_enabled) {
            sample["replay_planner_enabled"] = true;
            sample["replay_planner_state_yaw_rad"] = planner_state.yaw;
            sample["replay_planner_state_yaw_vel_rad_s"] = planner_state.yaw_vel;
            sample["replay_planner_state_pitch_rad"] = planner_state.pitch;
            sample["replay_planner_state_pitch_vel_rad_s"] = planner_state.pitch_vel;
            sample["replay_planner_has_measured_velocity"] = has_measured_velocity;
            sample["replay_planner_bullet_speed_mps"] = planning_speed;
        }
        sample["planner_state_yaw_deg"] = planner_state.yaw * 180.0 / CV_PI;
        sample["planner_state_pitch_deg"] = planner_state.pitch * 180.0 / CV_PI;
        auto target = targets.front();
        target.predict(planner_start);
        // 当前板沿用 Aimer 的锁；未来板由同一选板规则在预测副本上决定。
        const std::optional<int> planner_armor_id =
            diagnostic.control && aimer_.debug_aim_point.valid &&
                    aimer_.debug_aim_point.armor_id >= 0
                ? std::optional<int>{aimer_.debug_aim_point.armor_id}
                : std::nullopt;
        const auto plan = planner_.plan(
            target, planning_speed, planner_state, planner_armor_id,
            planner_armor_id ? &aimer_ : nullptr);
        sample["planner_preview_switch_ms"] = plan.preview_switch_step >= 0
            ? Json(plan.preview_switch_step * auto_aim::DT * 1000.0)
            : Json(nullptr);
        sample["planner_status"] =
            !plan.diagnostic_valid ? "invalid" :
            plan.solver_converged ? "ok" : "unconverged_diagnostic";
        sample["planner_control"] =
            plan.control && planner_armor_id.has_value() ? 1 : 0;
        sample["planner_solver_converged"] = plan.solver_converged;
        sample["planner_yaw_solver_status"] = plan.yaw_solver_status;
        sample["planner_pitch_solver_status"] = plan.pitch_solver_status;
        sample["planner_yaw_solver_iterations"] = plan.yaw_solver_iterations;
        sample["planner_pitch_solver_iterations"] = plan.pitch_solver_iterations;
        if (plan.diagnostic_valid) {
            sample["planner_target_yaw_deg"] = plan.target_yaw * 180.0 / CV_PI;
            sample["planner_target_pitch_deg"] = plan.target_pitch * 180.0 / CV_PI;
            sample["planner_yaw_deg"] = plan.yaw * 180.0 / CV_PI;
            sample["planner_pitch_deg"] = plan.pitch * 180.0 / CV_PI;
            sample["planner_yaw_20ms_deg"] =
                tools::limit_rad(plan.yaw + auto_aim::DT * plan.yaw_vel) *
                    180.0 / CV_PI;
            sample["planner_pitch_20ms_deg"] =
                (plan.pitch + auto_aim::DT * plan.pitch_vel) * 180.0 / CV_PI;
            sample["planner_yaw_vel_rad_s"] = plan.yaw_vel;
            sample["planner_pitch_vel_rad_s"] = plan.pitch_vel;
            sample["planner_yaw_acc_rad_s2"] = plan.yaw_acc;
            sample["planner_pitch_acc_rad_s2"] = plan.pitch_acc;
            // 额外诊断值不参与 control 门控；非有限值单独写 null，避免污染 JSONL。
            if (std::isfinite(plan.target_yaw_100ms) &&
                std::isfinite(plan.target_pitch_100ms) &&
                std::isfinite(plan.yaw_100ms) && std::isfinite(plan.pitch_100ms)) {
                sample["planner_target_yaw_100ms_deg"] =
                    plan.target_yaw_100ms * 180.0 / CV_PI;
                sample["planner_target_pitch_100ms_deg"] =
                    plan.target_pitch_100ms * 180.0 / CV_PI;
                sample["planner_yaw_100ms_deg"] = plan.yaw_100ms * 180.0 / CV_PI;
                sample["planner_pitch_100ms_deg"] = plan.pitch_100ms * 180.0 / CV_PI;
            } else {
                sample["planner_target_yaw_100ms_deg"] = nullptr;
                sample["planner_target_pitch_100ms_deg"] = nullptr;
                sample["planner_yaw_100ms_deg"] = nullptr;
                sample["planner_pitch_100ms_deg"] = nullptr;
            }
            sample["planner_yaw_primal_residual_max"] =
                std::isfinite(plan.yaw_primal_residual_max) ?
                Json(plan.yaw_primal_residual_max) : Json(nullptr);
            sample["planner_yaw_dual_residual_max"] =
                std::isfinite(plan.yaw_dual_residual_max) ?
                Json(plan.yaw_dual_residual_max) : Json(nullptr);
            sample["planner_pitch_primal_residual_max"] =
                std::isfinite(plan.pitch_primal_residual_max) ?
                Json(plan.pitch_primal_residual_max) : Json(nullptr);
            sample["planner_pitch_dual_residual_max"] =
                std::isfinite(plan.pitch_dual_residual_max) ?
                Json(plan.pitch_dual_residual_max) : Json(nullptr);
        }

        if (command_allowed()) {
            if (planner_armor_id) {
                tx_controller.apply(
                    plan, feedback, measured_speed, tx_extrapolating);
            } else {
                tx_controller.hold(feedback);
            }
        }
    } else if (tx_controller.enabled() && command_allowed()) {
        tx_controller.hold(feedback);
    }
    sample["tx_command_sent"] = tx_controller.sent();
    sample["tx_command_mode"] = tx_controller.mode();
    sample["tx_command_source"] = tx_controller.source();
    sample["tx_extrapolating"] = tx_controller.extrapolating();
    if (tx_controller.sent()) {
        sample["tx_command_yaw_deg"] = tx_controller.yaw();
        sample["tx_command_pitch_deg"] = tx_controller.pitch();
    }
    sample["planner_ms"] = duration_ms(planner_start, Clock::now());

    sample["event"] = "frame";
    sample["detected"] = detected;
    sample["tracker_state"] = tracker_.state();
    sample["tracker_generation"] = tracker_.target_generation();
    sample["temp_lost_count"] = tracker_.temp_lost_count();
    sample["bracket_ms"] = feedback.bracket_ms;
    sample["bullet_speed_mps"] = bullet_speed;
    sample["mode"] = feedback.feedback.mode;
    sample["color"] = feedback.feedback.color;
    sample["detector_ms"] = duration_ms(detector_start, detector_end);
    sample["tracker_ms"] = duration_ms(tracker_start, tracker_end);
    sample["aimer_ms"] = duration_ms(aimer_start, finished_at);
    sample["capture_to_detector_ms"] = duration_ms(image_timestamp, detector_start);
    sample["capture_to_aimer_ms"] = duration_ms(image_timestamp, finished_at);
    append_target_json(sample, targets);
    append_reprojection_json(
        sample, solver_, detected_armors_for_diagnostic, targets);
    append_association_json(sample, tracker_.association_debug(), solver_);
    append_tracker_filter_json(sample, tracker_, targets);
    append_aim_json(sample, aimer_, diagnostic);
    result.targets = std::move(targets);
    result.bullet_speed = bullet_speed;
    result.diagnostic = diagnostic;
    result.tracker_state = tracker_.state();
    result.tracker_generation = tracker_.target_generation();
    result.temp_lost_count = tracker_.temp_lost_count();
    result.finished_at = finished_at;
    return result;
}

}  // namespace real_auto_aim
