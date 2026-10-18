#include "src/real_auto_aim/frame_diagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "tasks/auto_aim/target.hpp"
#include "tools/math_tools.hpp"

namespace real_auto_aim::diagnostics
{
using Json = nlohmann::json;
using Clock = real_auto_aim::Clock;
double duration_ms(Clock::time_point begin, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

void draw_polygon(
    cv::Mat & image, const std::vector<cv::Point2f> & points, const cv::Scalar & color,
    const std::string & label)
{
    if (points.size() < 3) return;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto & current = points[index];
        const auto & next = points[(index + 1) % points.size()];
        cv::line(image, current, next, color, 3, cv::LINE_AA);
    }
    for (const auto & point : points) {
        cv::circle(image, point, 5, color, -1, cv::LINE_AA);
    }
    const auto rect = cv::boundingRect(points);
    cv::putText(
        image, label, cv::Point(rect.x, std::max(20, rect.y - 8)), cv::FONT_HERSHEY_SIMPLEX,
        0.65, color, 2, cv::LINE_AA);
}

void draw_text(cv::Mat & image, const std::string & text, int line, const cv::Scalar & color)
{
    cv::putText(
        image, text, cv::Point(15, 30 + line * 27), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2,
        cv::LINE_AA);
}

Json timing_json(const io::FrameTiming & timing, Clock::time_point now)
{
    Json sample{
        {"frame_id", timing.frame_id},
        {"device_ticks", timing.device_ticks},
        {"tick_hz", timing.device_timestamp_hz},
        {"timestamp_source", io::frame_timestamp_source_name(timing.timestamp_source)},
    };
    if (timing.mapped_capture_at) {
        sample["mapped_age_ms"] = duration_ms(*timing.mapped_capture_at, now);
        sample["mapping_delay_ms"] =
            duration_ms(*timing.mapped_capture_at, timing.host_received_at);
    }
    return sample;
}

double mean_reprojection_error(
    const std::vector<cv::Point2f> & observed,
    const std::vector<cv::Point2f> & projected)
{
    if (observed.size() != 4 || projected.size() != observed.size()) return -1.0;

    double error_sum = 0.0;
    for (std::size_t index = 0; index < observed.size(); ++index) {
        if (
            !std::isfinite(observed[index].x) || !std::isfinite(observed[index].y) ||
            !std::isfinite(projected[index].x) || !std::isfinite(projected[index].y)) {
            return -1.0;
        }
        error_sum += cv::norm(observed[index] - projected[index]);
    }
    return error_sum / static_cast<double>(observed.size());
}

bool quad_center(const std::vector<cv::Point2f> & points, cv::Point2f & center)
{
    if (points.size() != 4) return false;
    center = cv::Point2f(0.0F, 0.0F);
    for (const auto & point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
        center += point;
    }
    center *= 0.25F;
    return true;
}

std::optional<double> candidate_predicted_center_error_px(
    const auto_aim::AssociationCandidateDebug & candidate,
    const auto_aim::AssociationDebug & association,
    const auto_aim::Solver & solver)
{
    if (
        candidate.model_id < 0 || !candidate.observed_quad_center ||
        !std::isfinite(candidate.predicted_x) ||
        !std::isfinite(candidate.predicted_y) ||
        !std::isfinite(candidate.predicted_z) ||
        !std::isfinite(candidate.predicted_yaw)) {
        return std::nullopt;
    }

    try {
        // 使用关联前的模型位置和 yaw；日志误差不参与 Tracker 决策。
        const auto projected = solver.reproject_armor(
            Eigen::Vector3d(
                candidate.predicted_x, candidate.predicted_y, candidate.predicted_z),
            candidate.predicted_yaw, association.model_type, association.model_name);
        cv::Point2f projected_center;
        if (!quad_center(projected, projected_center)) return std::nullopt;
        const double error = cv::norm(projected_center - *candidate.observed_quad_center);
        if (!std::isfinite(error)) return std::nullopt;
        return error;
    } catch (const cv::Exception &) {
        return std::nullopt;
    }
}

double best_order_reprojection_error(
    const std::vector<cv::Point2f> & observed,
    const std::vector<cv::Point2f> & projected)
{
    cv::Point2f observed_center;
    cv::Point2f projected_center;
    if (
        !quad_center(observed, observed_center) ||
        !quad_center(projected, projected_center)) {
        return -1.0;
    }

    double best_error = std::numeric_limits<double>::infinity();
    // 仅改变四角点的比较顺序，不改变 PnP、Tracker 或实际装甲板姿态。
    for (int direction = -1; direction <= 1; direction += 2) {
        for (int shift = 0; shift < 4; ++shift) {
            double error_sum = 0.0;
            for (int index = 0; index < 4; ++index) {
                const int projected_index = (shift + direction * index + 4) % 4;
                error_sum += cv::norm(observed[index] - projected[projected_index]);
            }
            best_error = std::min(best_error, error_sum / 4.0);
        }
    }
    return best_error;
}

void append_reprojection_json(
    Json & sample,
    const auto_aim::Solver & solver,
    const std::list<auto_aim::Armor> & detected_armors,
    const std::list<auto_aim::Target> & targets)
{
    Json pnp_errors = Json::array();
    std::size_t detection_index = 0;
    for (const auto & armor : detected_armors) {
        const auto projected = solver.reproject_pnp(armor);
        const double error = mean_reprojection_error(armor.points, projected);
        Json item{
            {"detection_index", detection_index},
            {"pnp_reprojection_error_px", error >= 0.0 ? Json(error) : Json(nullptr)},
        };
        pnp_errors.push_back(item);
        ++detection_index;
    }
    sample["pnp_reprojection_errors_px"] = pnp_errors;

    // 当前模型误差取与 EKF 当前装甲板投影最近的检测框，避免双装甲板时比较错对象。
    // 该值不是 Tracker 关联代价，而是用于隔离手眼、云台姿态和装甲板模型误差。
    sample["current_model_reprojection_error_px"] = nullptr;
    sample["current_model_reprojection_detection_index"] = nullptr;
    sample["current_model_center_error_px"] = nullptr;
    sample["current_model_center_dx_px"] = nullptr;
    sample["current_model_center_dy_px"] = nullptr;
    sample["current_model_center_detection_index"] = nullptr;
    sample["current_model_index_error_at_center_match_px"] = nullptr;
    sample["current_model_best_order_error_px"] = nullptr;
    sample["current_model_observed_points_px"] = nullptr;
    sample["current_model_projected_points_px"] = nullptr;
    if (targets.empty()) return;

    const auto & target = targets.front();
    const auto target_armors = target.armor_xyza_list();
    if (
        target.last_id < 0 ||
        static_cast<std::size_t>(target.last_id) >= target_armors.size()) {
        return;
    }

    const auto & current = target_armors[static_cast<std::size_t>(target.last_id)];
    const auto projected = solver.reproject_armor(
        current.head(3), current[3], target.armor_type, target.name);
    cv::Point2f projected_center;
    if (!quad_center(projected, projected_center)) return;

    double best_error = std::numeric_limits<double>::infinity();
    double best_center_error = std::numeric_limits<double>::infinity();
    cv::Point2f best_observed_center;
    const auto_aim::Armor * center_match = nullptr;
    std::size_t best_index = 0;
    std::size_t best_center_index = 0;
    std::size_t index = 0;
    for (const auto & armor : detected_armors) {
        const double error = mean_reprojection_error(armor.points, projected);
        if (error >= 0.0 && error < best_error) {
            best_error = error;
            best_index = index;
        }
        cv::Point2f observed_center;
        if (quad_center(armor.points, observed_center)) {
            const double center_error = cv::norm(projected_center - observed_center);
            if (center_error < best_center_error) {
                best_center_error = center_error;
                best_observed_center = observed_center;
                best_center_index = index;
                center_match = &armor;
            }
        }
        ++index;
    }
    if (std::isfinite(best_error)) {
        sample["current_model_reprojection_error_px"] = best_error;
        sample["current_model_reprojection_detection_index"] = best_index;
    }
    if (center_match) {
        // 中心匹配不依赖角点编号；dx/dy 定义为模型投影减检测角点中心。
        sample["current_model_center_error_px"] = best_center_error;
        sample["current_model_center_dx_px"] = projected_center.x - best_observed_center.x;
        sample["current_model_center_dy_px"] = projected_center.y - best_observed_center.y;
        sample["current_model_center_detection_index"] = best_center_index;
        sample["current_model_index_error_at_center_match_px"] =
            mean_reprojection_error(center_match->points, projected);
        sample["current_model_best_order_error_px"] =
            best_order_reprojection_error(center_match->points, projected);
        sample["current_model_observed_points_px"] = Json::array();
        sample["current_model_projected_points_px"] = Json::array();
        for (int point_index = 0; point_index < 4; ++point_index) {
            const auto & observed = center_match->points[point_index];
            const auto & model = projected[point_index];
            sample["current_model_observed_points_px"].push_back(
                Json::array({observed.x, observed.y}));
            sample["current_model_projected_points_px"].push_back(
                Json::array({model.x, model.y}));
        }
    }
}

void append_target_json(Json & sample, const std::list<auto_aim::Target> & targets)
{
    if (targets.empty()) {
        sample["targets"] = 0;
        return;
    }

    const auto & target = targets.front();
    const auto state = target.ekf_x();
    sample["targets"] = targets.size();
    sample["current_armor_id"] = target.last_id;
    if (state.size() >= 11) {
        sample["center_x"] = state[0];
        sample["vx"] = state[1];
        sample["center_y"] = state[2];
        sample["vy"] = state[3];
        sample["center_z"] = state[4];
        sample["vz"] = state[5];
        sample["center_yaw"] = state[6];
        sample["angular_velocity"] = state[7];
        sample["radius"] = state[8];
        sample["radius_delta"] = state[9];
        sample["height_delta"] = state[10];
        sample["center_speed"] = std::hypot(state[1], state[3]);
    }

    const auto armors = target.armor_xyza_list();
    if (target.last_id >= 0 && static_cast<std::size_t>(target.last_id) < armors.size()) {
        const auto & current = armors[static_cast<std::size_t>(target.last_id)];
        sample["current_x"] = current[0];
        sample["current_y"] = current[1];
        sample["current_z"] = current[2];
        sample["current_yaw"] = current[3];
    }
}

void append_association_candidate_json(
    Json & sample, const std::string & prefix,
    const auto_aim::AssociationCandidateDebug & candidate,
    const auto_aim::AssociationDebug & association,
    const auto_aim::Solver & solver)
{
    if (candidate.model_id < 0) return;
    sample[prefix + "_id"] = candidate.model_id;
    sample[prefix + "_gate_passed"] = candidate.gate_passed ? 1 : 0;
    sample[prefix + "_accepted"] = candidate.accepted ? 1 : 0;
    sample[prefix + "_angle_gate_passed"] = candidate.angle_gate_passed ? 1 : 0;
    sample[prefix + "_score_gate_passed"] = candidate.score_gate_passed ? 1 : 0;
    sample[prefix + "_position_gate_passed"] = candidate.position_gate_passed ? 1 : 0;
    sample[prefix + "_distance_gate_passed"] = candidate.distance_gate_passed ? 1 : 0;
    sample[prefix + "_mahalanobis_gate_passed"] =
        candidate.mahalanobis_gate_passed ? 1 : 0;
    sample[prefix + "_image_model_gate_passed"] =
        candidate.image_model_gate_passed ? 1 : 0;
    sample[prefix + "_image_reference_error_px"] = candidate.image_reference_error_px;
    sample[prefix + "_image_adjacent_error_px"] = candidate.image_adjacent_error_px;
    sample[prefix + "_image_observed_extent_px"] = candidate.image_observed_extent_px;
    sample[prefix + "_score"] = candidate.score;
    sample[prefix + "_position_error"] = candidate.position_error;
    sample[prefix + "_distance_error"] = candidate.distance_error;
    sample[prefix + "_mahalanobis_distance"] = candidate.mahalanobis_distance;
    sample[prefix + "_position_angle_error"] = candidate.position_angle_error;
    sample[prefix + "_distance_angle_error"] = candidate.distance_angle_error;
    sample[prefix + "_observed_x"] = candidate.observed_x;
    sample[prefix + "_observed_y"] = candidate.observed_y;
    sample[prefix + "_observed_z"] = candidate.observed_z;
    sample[prefix + "_predicted_x"] = candidate.predicted_x;
    sample[prefix + "_predicted_y"] = candidate.predicted_y;
    sample[prefix + "_predicted_z"] = candidate.predicted_z;
    sample[prefix + "_observed_distance"] = candidate.observed_distance;
    sample[prefix + "_predicted_distance"] = candidate.predicted_distance;
    sample[prefix + "_orientation_error"] = candidate.orientation_error;
    sample[prefix + "_bearing_error"] = candidate.bearing_error;
    sample[prefix + "_angle_error"] = candidate.angle_error;
    sample[prefix + "_raw_yaw_prediction_error"] = candidate.raw_yaw_prediction_error;
    sample[prefix + "_optimized_yaw_prediction_error"] =
        candidate.optimized_yaw_prediction_error;
    sample[prefix + "_raw_yaw"] = candidate.raw_yaw;
    sample[prefix + "_optimized_yaw"] = candidate.optimized_yaw;
    sample[prefix + "_yaw_correction"] = candidate.yaw_correction;
    sample[prefix + "_image_x"] = candidate.image_x;
    sample[prefix + "_image_y"] = candidate.image_y;
    const auto center_error = candidate_predicted_center_error_px(candidate, association, solver);
    sample[prefix + "_predicted_center_error_valid"] = center_error ? 1 : 0;
    if (center_error) {
        sample[prefix + "_predicted_center_error_px"] = *center_error;
    } else {
        sample[prefix + "_predicted_center_error_px"] = nullptr;
    }
}

void append_association_json(
    Json & sample, const auto_aim::AssociationDebug & association,
    const auto_aim::Solver & solver)
{
    // 这些字段只记录关联决策的输入、门控和最终候选，不参与任何算法决策。
    // 真机日志需要据此区分 PnP 误差、模型 ID 切换和 EKF 中心漂移。
    sample["association_matching_detection_count"] =
        association.matching_detection_count;
    sample["association_gate_passed_count"] = association.gate_passed_count;
    sample["association_candidate_count"] = association.candidate_count;
    sample["association_accepted_count"] = association.accepted_count;
    append_association_candidate_json(
        sample, "association_primary", association.candidates[0], association, solver);
    append_association_candidate_json(
        sample, "association_secondary", association.candidates[1], association, solver);
    // 同一检测还会按多个装甲板 ID 计算模型假设，保留完整列表便于复盘 ID 切换。
    sample["association_model_candidates"] = Json::array();
    for (int i = 0; i < association.model_candidate_count; ++i) {
        const auto & candidate = association.model_candidates[static_cast<std::size_t>(i)];
        if (candidate.model_id < 0) continue;
        Json item = Json::object();
        item["index"] = i;
        item["model_id"] = candidate.model_id;
        item["gate_passed"] = candidate.gate_passed ? 1 : 0;
        item["angle_gate_passed"] = candidate.angle_gate_passed ? 1 : 0;
        item["score_gate_passed"] = candidate.score_gate_passed ? 1 : 0;
        item["position_gate_passed"] = candidate.position_gate_passed ? 1 : 0;
        item["distance_gate_passed"] = candidate.distance_gate_passed ? 1 : 0;
        item["mahalanobis_gate_passed"] = candidate.mahalanobis_gate_passed ? 1 : 0;
        item["image_model_gate_passed"] = candidate.image_model_gate_passed ? 1 : 0;
        item["image_reference_error_px"] = candidate.image_reference_error_px;
        item["image_adjacent_error_px"] = candidate.image_adjacent_error_px;
        item["image_observed_extent_px"] = candidate.image_observed_extent_px;
        item["score"] = candidate.score;
        item["position_error"] = candidate.position_error;
        item["distance_error"] = candidate.distance_error;
        item["mahalanobis_distance"] = candidate.mahalanobis_distance;
        item["position_angle_error"] = candidate.position_angle_error;
        item["distance_angle_error"] = candidate.distance_angle_error;
        item["orientation_error"] = candidate.orientation_error;
        item["bearing_error"] = candidate.bearing_error;
        item["angle_error"] = candidate.angle_error;
        item["observed_x"] = candidate.observed_x;
        item["observed_y"] = candidate.observed_y;
        item["observed_z"] = candidate.observed_z;
        item["predicted_x"] = candidate.predicted_x;
        item["predicted_y"] = candidate.predicted_y;
        item["predicted_z"] = candidate.predicted_z;
        item["image_x"] = candidate.image_x;
        item["image_y"] = candidate.image_y;
        const auto center_error = candidate_predicted_center_error_px(candidate, association, solver);
        item["predicted_center_error_valid"] = center_error ? 1 : 0;
        if (center_error) {
            item["predicted_center_error_px"] = *center_error;
        } else {
            item["predicted_center_error_px"] = nullptr;
        }
        sample["association_model_candidates"].push_back(std::move(item));
    }
}

void append_tracker_filter_json(
    Json & sample, const auto_aim::Tracker & tracker,
    const std::list<auto_aim::Target> & targets)
{
    sample["tracker_geometry_branch"] = tracker.geometry_branch();
    sample["tracker_geometry_trusted"] = tracker.geometry_trusted();
    sample["tracker_geometry_constraint_active"] =
        tracker.geometry_constraint_active();
    // 首次捕获也接受了观测，但只有成功更新 EKF 才产生本帧的新 NIS。
    sample["tracker_observation_accepted"] = tracker.observation_accepted();
    sample["tracker_nis_fresh"] = false;
    sample["tracker_model_name"] = nullptr;
    sample["tracker_tracked_label"] = nullptr;
    sample["tracker_radius_variance_m2"] = nullptr;
    sample["tracker_last_nis"] = nullptr;
    sample["tracker_recent_nis_failures"] = nullptr;
    const auto * target = targets.empty() ? tracker.diagnostic_target() : &targets.front();
    if (target == nullptr) return;

    const auto & ekf = target->ekf();
    sample["tracker_model_name"] = auto_aim::ARMOR_NAMES[target->name];
    sample["tracker_tracked_label"] =
        auto_aim::ARMOR_NAMES[tracker.tracked_label()];
    if (ekf.P.rows() > 8 && ekf.P.cols() > 8) {
        sample["tracker_radius_variance_m2"] = ekf.P(8, 8);
    }
    sample["tracker_recent_nis_failures"] = std::accumulate(
        ekf.recent_nis_failures.begin(), ekf.recent_nis_failures.end(), 0);
    if (ekf.last_innovation.size() > 0) {
        sample["tracker_last_nis"] = ekf.last_nis;
        sample["tracker_nis_fresh"] =
            tracker.association_debug().accepted_count > 0;
    }
}

void append_aim_json(Json & sample, const auto_aim::Aimer & aimer, const io::Command & command)
{
    sample["aim_control"] = command.control ? 1 : 0;
    sample["aim_shoot"] = command.shoot ? 1 : 0;
    // 无有效瞄点时的零角度只是返回占位值，不能画成真实的瞄准尖峰。
    const bool aim_valid = command.control && aimer.debug_aim_point.valid;
    sample["diagnostic_yaw"] = aim_valid ? Json(command.yaw) : Json(nullptr);
    sample["diagnostic_pitch"] = aim_valid ? Json(command.pitch) : Json(nullptr);
    sample["diagnostic_yaw_deg"] =
        aim_valid ? Json(command.yaw * 180.0 / CV_PI) : Json(nullptr);
    sample["diagnostic_pitch_deg"] =
        aim_valid ? Json(command.pitch * 180.0 / CV_PI) : Json(nullptr);
    sample["prediction_dt"] = aimer.debug_prediction_dt;
    sample["delay_time"] = aimer.debug_delay_time;
    sample["base_prediction_dt"] = aimer.debug_base_prediction_dt;
    sample["fly_time"] = aimer.debug_fly_time;
    sample["aim_armor_id"] =
        aim_valid ? Json(aimer.debug_aim_point.armor_id) : Json(nullptr);
    if (!aim_valid) return;
    const auto & future = aimer.debug_aim_point.xyza;
    sample["future_x"] = future[0];
    sample["future_y"] = future[1];
    sample["future_z"] = future[2];
    sample["future_yaw"] = future[3];
}

void append_image_quality_json(Json & sample, const cv::Mat & image)
{
    if (image.empty()) return;

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = image;
    }
    if (gray.empty() || gray.depth() != CV_8U) return;

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(gray, mean, stddev);
    cv::Mat dark_mask;
    cv::Mat bright_mask;
    cv::inRange(gray, 0, 2, dark_mask);
    cv::inRange(gray, 253, 255, bright_mask);

    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar lap_mean;
    cv::Scalar lap_stddev;
    cv::meanStdDev(laplacian, lap_mean, lap_stddev);

    const double pixel_count = static_cast<double>(gray.total());
    sample["image_mean_luma"] = mean[0];
    sample["image_std_luma"] = stddev[0];
    sample["image_dark_ratio"] =
        pixel_count > 0.0 ? cv::countNonZero(dark_mask) / pixel_count : 0.0;
    sample["image_bright_ratio"] =
        pixel_count > 0.0 ? cv::countNonZero(bright_mask) / pixel_count : 0.0;
    sample["image_laplacian_variance"] = lap_stddev[0] * lap_stddev[0];
}
}  // namespace real_auto_aim::diagnostics
