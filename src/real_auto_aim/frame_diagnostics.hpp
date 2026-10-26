#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <list>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/command.hpp"
#include "src/real_auto_aim/feedback_buffer.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"

// 这些函数只产生可视化和日志字段，不参与 Tracker、Aimer 或 Planner 决策。
namespace real_auto_aim::diagnostics
{
using Json = nlohmann::json;
using Clock = real_auto_aim::Clock;

double duration_ms(Clock::time_point begin, Clock::time_point end);
void draw_polygon(cv::Mat & image, const std::vector<cv::Point2f> & points,
                  const cv::Scalar & color, const std::string & label);
void draw_text(cv::Mat & image, const std::string & text, int line, const cv::Scalar & color);
Json timing_json(const io::FrameTiming & timing, Clock::time_point now);
void append_reprojection_json(Json & sample, const auto_aim::Solver & solver,
    const std::list<auto_aim::Armor> & detected_armors,
    const std::list<auto_aim::Target> & targets);
void append_target_json(Json & sample, const std::list<auto_aim::Target> & targets);
// association_detections 必须是 Tracker 原地过滤、排序后的列表，索引才能对应候选。
void append_association_json(Json & sample, const auto_aim::AssociationDebug & association,
    const auto_aim::Solver & solver,
    const std::list<auto_aim::Armor> & association_detections);
void append_tracker_filter_json(Json & sample, const auto_aim::Tracker & tracker,
    const std::list<auto_aim::Target> & targets);
void append_aim_json(Json & sample, const auto_aim::Aimer & aimer, const io::Command & command);
void append_image_quality_json(Json & sample, const cv::Mat & image);

// 采集和离线回放共用相同字段，便于逐帧对照 ROI、先验框和模型输出。
inline void append_detector_debug_json(
    Json & sample, const auto_aim::DetectorDebugInfo & info,
    const std::list<auto_aim::Armor> & armors)
{
    sample["detector_model"] = info.model_name.empty() ? Json(nullptr) : Json(info.model_name);
    sample["detector_roi_active"] = info.roi_active;
    sample["detector_roi_x_px"] = info.roi_active ? Json(info.applied_roi.x) : Json(nullptr);
    sample["detector_roi_y_px"] = info.roi_active ? Json(info.applied_roi.y) : Json(nullptr);
    sample["detector_roi_width_px"] =
        info.roi_active ? Json(info.applied_roi.width) : Json(nullptr);
    sample["detector_roi_height_px"] =
        info.roi_active ? Json(info.applied_roi.height) : Json(nullptr);
    sample["detector_roi_prior_used"] = info.prior_used;
    sample["detector_roi_prior_x_px"] =
        info.prior_box_valid ? Json(info.prior_box.x) : Json(nullptr);
    sample["detector_roi_prior_y_px"] =
        info.prior_box_valid ? Json(info.prior_box.y) : Json(nullptr);
    sample["detector_roi_prior_width_px"] =
        info.prior_box_valid ? Json(info.prior_box.width) : Json(nullptr);
    sample["detector_roi_prior_height_px"] =
        info.prior_box_valid ? Json(info.prior_box.height) : Json(nullptr);
    sample["detector_roi_miss_streak_before"] = info.roi_miss_streak_before;
    sample["detector_roi_miss_streak_after"] = info.roi_miss_streak_after;
    sample["detector_score_candidates"] =
        info.score_candidates >= 0 ? Json(info.score_candidates) : Json(nullptr);
    sample["detector_nms_candidates"] =
        info.nms_candidates >= 0 ? Json(info.nms_candidates) : Json(nullptr);
    sample["detector_classifier_rejected"] =
        info.classifier_rejected >= 0 ? Json(info.classifier_rejected) : Json(nullptr);

    Json outputs = Json::array();
    for (const auto & armor : armors) {
        outputs.push_back({
            {"class_id", armor.class_id},
            {"name", auto_aim::ARMOR_NAMES[static_cast<std::size_t>(armor.name)]},
            {"confidence", armor.confidence},
            {"center_x_px", armor.center.x},
            {"center_y_px", armor.center.y},
            {"width_px", armor.box.width},
            {"height_px", armor.box.height}});
    }
    sample["detector_outputs"] = outputs;
}
}  // namespace real_auto_aim::diagnostics
