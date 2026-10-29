#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <list>
#include <limits>
#include <optional>
#include <tuple>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "src/real_auto_aim/config.hpp"
#include "src/real_auto_aim/frame_diagnostics.hpp"
#include "src/real_auto_aim/quintic_switch_diagnostic.hpp"
#include "src/real_auto_aim/tx_command_limiter.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/yolos/yolov8.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;

struct LightbarImageQuality
{
    double length_px = 0.0;
    double visible_fraction = 0.0;
    double saturation_fraction = 0.0;
    double max_dark_gap_px = 0.0;
};

LightbarImageQuality measure_lightbar_image_quality(
    const cv::Mat & gray, const cv::Point2f & top, const cv::Point2f & bottom,
    double threshold)
{
    LightbarImageQuality quality;
    const cv::Point2f axis = bottom - top;
    quality.length_px = cv::norm(axis);
    if (!std::isfinite(quality.length_px) || quality.length_px <= 0.0) return quality;

    const cv::Point2f tangent = axis * static_cast<float>(1.0 / quality.length_px);
    const cv::Point2f normal{-tangent.y, tangent.x};
    const int bins = std::max(1, static_cast<int>(std::ceil(quality.length_px)));
    int visible_bins = 0;
    int longest_dark_run = 0;
    int current_dark_run = 0;
    int sample_count = 0;
    int saturated_count = 0;
    for (int bin = 0; bin < bins; ++bin) {
        const float fraction = (static_cast<float>(bin) + 0.5F) / bins;
        const cv::Point2f center = top + axis * fraction;
        bool visible = false;
        // 5 px 横向采样覆盖细灯条，同时避免把外侧装甲板亮区算进可见长度。
        for (int offset = -2; offset <= 2; ++offset) {
            const auto point = center + normal * static_cast<float>(offset);
            const int x = cvRound(point.x);
            const int y = cvRound(point.y);
            if (x < 0 || y < 0 || x >= gray.cols || y >= gray.rows) continue;
            const auto intensity = gray.at<unsigned char>(y, x);
            visible = visible || static_cast<double>(intensity) > threshold;
            saturated_count += intensity >= 250 ? 1 : 0;
            ++sample_count;
        }
        if (visible) {
            ++visible_bins;
            current_dark_run = 0;
        } else {
            ++current_dark_run;
            longest_dark_run = std::max(longest_dark_run, current_dark_run);
        }
    }
    quality.visible_fraction = static_cast<double>(visible_bins) / bins;
    quality.saturation_fraction = sample_count > 0
        ? static_cast<double>(saturated_count) / sample_count : 0.0;
    quality.max_dark_gap_px =
        static_cast<double>(longest_dark_run) * quality.length_px / bins;
    return quality;
}

double minimum_pnp_reprojection_error(
    const auto_aim::Solver & solver, const auto_aim::Armor & armor)
{
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto & candidate : solver.pnp_candidates(armor)) {
        if (std::isfinite(candidate.reprojection_error)) {
            minimum = std::min(minimum, candidate.reprojection_error);
        }
    }
    return std::isfinite(minimum) ? minimum : -1.0;
}

// 仅供离线探针使用；双线性取样避免短灯条在整数像素上跳变。
std::optional<cv::Vec3f> sample_bgr(const cv::Mat & image, cv::Point2f point)
{
    if (point.x < 0 || point.y < 0 ||
        point.x >= image.cols - 1 || point.y >= image.rows - 1) return std::nullopt;
    const int x = static_cast<int>(point.x);
    const int y = static_cast<int>(point.y);
    const float dx = point.x - x;
    const float dy = point.y - y;
    const auto a = cv::Vec3f(image.at<cv::Vec3b>(y, x));
    const auto b = cv::Vec3f(image.at<cv::Vec3b>(y, x + 1));
    const auto c = cv::Vec3f(image.at<cv::Vec3b>(y + 1, x));
    const auto d = cv::Vec3f(image.at<cv::Vec3b>(y + 1, x + 1));
    return (1 - dy) * ((1 - dx) * a + dx * b) +
           dy * ((1 - dx) * c + dx * d);
}

Json lamp_endpoint_profile(
    const cv::Mat & image, cv::Point2f top, cv::Point2f bottom,
    auto_aim::Color color)
{
    Json profile = {{"valid", false}};
    const double length = cv::norm(bottom - top);
    if (image.type() != CV_8UC3 || !std::isfinite(length) || length < 5 ||
        (color != auto_aim::blue && color != auto_aim::red)) return profile;

    const cv::Point2f axis = (bottom - top) * static_cast<float>(1.0 / length);
    const cv::Point2f normal{-axis.y, axis.x};
    Json samples = Json::array();
    for (double s = -0.25 * length; s <= 1.25 * length; s += 0.5) {
        std::array<double, 5> gray{};
        std::array<double, 5> contrast{};
        int clipped = 0;
        bool complete = true;
        for (int offset = -2; offset <= 2; ++offset) {
            const auto pixel = sample_bgr(
                image, top + axis * static_cast<float>(s) +
                           normal * static_cast<float>(offset));
            if (!pixel) {
                complete = false;
                break;
            }
            const int index = offset + 2;
            gray[index] = 0.114 * (*pixel)[0] + 0.587 * (*pixel)[1] +
                          0.299 * (*pixel)[2];
            contrast[index] = color == auto_aim::blue
                ? (*pixel)[0] - (*pixel)[2] : (*pixel)[2] - (*pixel)[0];
            clipped += (*pixel)[color == auto_aim::blue ? 0 : 2] >= 250;
        }
        if (!complete) continue;
        std::sort(gray.begin(), gray.end());
        std::sort(contrast.begin(), contrast.end());
        samples.push_back({
            {"s_px", s}, {"gray", gray[2]},
            {"color_contrast", contrast[2]},
            {"target_channel_clip_fraction", clipped / 5.0}
        });
    }
    if (samples.size() < 5) return profile;

    // 最大梯度只是候选端点；不把它当作准确角点或可靠测距标志。
    const auto edge = [&](const char * field, bool top_edge) -> Json {
        double best = 0.0;
        Json candidate = nullptr;
        const double expected = top_edge ? 0.0 : length;
        for (std::size_t i = 1; i + 1 < samples.size(); ++i) {
            const double s = samples[i]["s_px"].get<double>();
            if (std::abs(s - expected) > 0.25 * length) continue;
            const double before = samples[i - 1][field].get<double>();
            const double after = samples[i + 1][field].get<double>();
            const double strength = top_edge ? after - before : before - after;
            if (strength > best) {
                best = strength;
                candidate = {{"s_px", s}, {"gradient", strength}};
            }
        }
        return candidate;
    };
    profile = {
        {"valid", true}, {"raw_top_s_px", 0.0},
        {"raw_bottom_s_px", length},
        {"gray_top_candidate", edge("gray", true)},
        {"gray_bottom_candidate", edge("gray", false)},
        {"color_top_candidate", edge("color_contrast", true)},
        {"color_bottom_candidate", edge("color_contrast", false)},
        {"samples", std::move(samples)}
    };
    return profile;
}

void prepare_endpoint_range_shadow(
    auto_aim::Armor & armor, const cv::Mat & image, const cv::Mat & gray,
    auto_aim::Detector & endpoint_detector, const auto_aim::Solver & solver,
    double threshold, auto_aim::EndpointRangeShadowDebug & shadow)
{
    if (armor.points.size() != 4) return;

    const auto raw_armor = armor;
    shadow.raw_left_length_px = cv::norm(raw_armor.points[3] - raw_armor.points[0]);
    shadow.raw_right_length_px = cv::norm(raw_armor.points[2] - raw_armor.points[1]);
    auto raw_pose = raw_armor;
    try {
        solver.solve(raw_pose);
        shadow.raw_pnp_distance_m = raw_pose.ypd_in_world[2];
        shadow.raw_reprojection_error_px = minimum_pnp_reprojection_error(solver, raw_armor);
    } catch (const cv::Exception &) {
        // 单个检测的 PnP 失败保留诊断缺省值，不中断整段离线回放。
    }

    auto fitted_armor = raw_armor;
    shadow.fit_found = endpoint_detector.detect(fitted_armor, image);
    if (!shadow.fit_found || fitted_armor.points.size() != 4) return;

    bool finite_points = true;
    double squared_shift = 0.0;
    for (std::size_t index = 0; index < 4; ++index) {
        const auto & point = fitted_armor.points[index];
        finite_points = finite_points && std::isfinite(point.x) && std::isfinite(point.y);
        shadow.fitted_points[index] = point;
        const double shift = cv::norm(point - raw_armor.points[index]);
        squared_shift += shift * shift;
    }
    if (!finite_points) return;
    shadow.corner_shift_rms_px = std::sqrt(squared_shift / 4.0);

    const auto raw_left = measure_lightbar_image_quality(
        gray, raw_armor.points[0], raw_armor.points[3], threshold);
    const auto raw_right = measure_lightbar_image_quality(
        gray, raw_armor.points[1], raw_armor.points[2], threshold);
    const auto fitted_left = measure_lightbar_image_quality(
        gray, fitted_armor.points[0], fitted_armor.points[3], threshold);
    const auto fitted_right = measure_lightbar_image_quality(
        gray, fitted_armor.points[1], fitted_armor.points[2], threshold);
    shadow.fitted_left_length_px = fitted_left.length_px;
    shadow.fitted_right_length_px = fitted_right.length_px;
    // 可见度沿 YOLO 原始灯条跨度测量，避免只在已经缩短的拟合线段上采样而虚高。
    shadow.left_visible_fraction = raw_left.visible_fraction;
    shadow.right_visible_fraction = raw_right.visible_fraction;
    shadow.left_saturation_fraction = raw_left.saturation_fraction;
    shadow.right_saturation_fraction = raw_right.saturation_fraction;
    shadow.left_max_dark_gap_px = raw_left.max_dark_gap_px;
    shadow.right_max_dark_gap_px = raw_right.max_dark_gap_px;
    shadow.left_fitted_visible_fraction = fitted_left.visible_fraction;
    shadow.right_fitted_visible_fraction = fitted_right.visible_fraction;
    shadow.left_fitted_saturation_fraction = fitted_left.saturation_fraction;
    shadow.right_fitted_saturation_fraction = fitted_right.saturation_fraction;
    shadow.left_fitted_max_dark_gap_px = fitted_left.max_dark_gap_px;
    shadow.right_fitted_max_dark_gap_px = fitted_right.max_dark_gap_px;

    try {
        solver.solve(fitted_armor);
        shadow.fitted_pnp_distance_m = fitted_armor.ypd_in_world[2];
        shadow.fitted_reprojection_error_px =
            minimum_pnp_reprojection_error(solver, fitted_armor);
    } catch (const cv::Exception &) {
        return;
    }

    // 可靠性要求检测器拟合成功、PnP 有限且原始检测跨度至少 90% 被亮区覆盖；
    // 饱和率、最大暗缺口和重投影误差仅作诊断，低重投影误差不能单独证明测距正确。
    shadow.reliable = std::isfinite(shadow.raw_pnp_distance_m) &&
        std::isfinite(shadow.fitted_pnp_distance_m) && shadow.fitted_pnp_distance_m > 0.0 &&
        shadow.left_visible_fraction >= 0.90 && shadow.right_visible_fraction >= 0.90 &&
        std::isfinite(shadow.fitted_reprojection_error_px) &&
        shadow.fitted_reprojection_error_px >= 0.0;
}

Json endpoint_range_shadow_json(const auto_aim::EndpointRangeShadowDebug & shadow)
{
    Json endpoint = Json::object();
    endpoint["fit_found"] = shadow.fit_found;
    endpoint["reliable"] = shadow.reliable;
    endpoint["selected"] = shadow.selected;
    endpoint["distance_replaced"] = shadow.distance_replaced;
    endpoint["update_accepted"] = shadow.update_accepted;
    endpoint["raw_left_length_px"] = shadow.raw_left_length_px;
    endpoint["raw_right_length_px"] = shadow.raw_right_length_px;
    endpoint["fitted_left_length_px"] = shadow.fitted_left_length_px;
    endpoint["fitted_right_length_px"] = shadow.fitted_right_length_px;
    endpoint["left_visible_fraction"] = shadow.left_visible_fraction;
    endpoint["right_visible_fraction"] = shadow.right_visible_fraction;
    endpoint["left_saturation_fraction"] = shadow.left_saturation_fraction;
    endpoint["right_saturation_fraction"] = shadow.right_saturation_fraction;
    endpoint["left_max_dark_gap_px"] = shadow.left_max_dark_gap_px;
    endpoint["right_max_dark_gap_px"] = shadow.right_max_dark_gap_px;
    endpoint["left_fitted_visible_fraction"] = shadow.left_fitted_visible_fraction;
    endpoint["right_fitted_visible_fraction"] = shadow.right_fitted_visible_fraction;
    endpoint["left_fitted_saturation_fraction"] = shadow.left_fitted_saturation_fraction;
    endpoint["right_fitted_saturation_fraction"] = shadow.right_fitted_saturation_fraction;
    endpoint["left_fitted_max_dark_gap_px"] = shadow.left_fitted_max_dark_gap_px;
    endpoint["right_fitted_max_dark_gap_px"] = shadow.right_fitted_max_dark_gap_px;
    endpoint["corner_shift_rms_px"] = shadow.corner_shift_rms_px;
    endpoint["raw_pnp_distance_m"] = shadow.raw_pnp_distance_m;
    endpoint["fitted_pnp_distance_m"] = shadow.fitted_pnp_distance_m;
    endpoint["raw_reprojection_error_px"] = shadow.raw_reprojection_error_px;
    endpoint["fitted_reprojection_error_px"] = shadow.fitted_reprojection_error_px;
    endpoint["selected_raw_distance_m"] = shadow.selected_raw_distance_m;
    endpoint["selected_fitted_distance_m"] = shadow.selected_fitted_distance_m;
    endpoint["fitted_corners_px"] = Json::array();
    for (const auto & point : shadow.fitted_points) {
        endpoint["fitted_corners_px"].push_back(Json::array({point.x, point.y}));
    }
    return endpoint;
}

cv::Rect roi_depth_probe_rect(
    const auto_aim::Armor & armor, const cv::Size & image_size,
    double scale, int min_side, int max_side)
{
    if (armor.points.size() != 4 || image_size.width <= 0 || image_size.height <= 0) {
        return {};
    }
    const double left_length = cv::norm(armor.points[3] - armor.points[0]);
    const double right_length = cv::norm(armor.points[2] - armor.points[1]);
    const double mean_length = 0.5 * (left_length + right_length);
    if (!std::isfinite(mean_length) || mean_length <= 0.0) return {};

    // 只扫描离线 ROI 尺寸；它衡量分辨率敏感性，不代表测距准确度。
    const double side = std::clamp(
        scale * mean_length, static_cast<double>(min_side),
        static_cast<double>(max_side));
    const int requested_side = cvRound(side);
    const int width = std::min(requested_side, image_size.width);
    const int height = std::min(requested_side, image_size.height);
    cv::Point2f center{};
    for (const auto & point : armor.points) center += point * 0.25F;
    const int x = std::clamp(cvRound(center.x - 0.5 * width), 0, image_size.width - width);
    const int y = std::clamp(cvRound(center.y - 0.5 * height), 0, image_size.height - height);
    return {x, y, width, height};
}

Json roi_depth_candidate_json(
    const auto_aim::Armor & candidate, const auto_aim::Armor & source,
    const auto_aim::Solver & solver, std::size_t candidate_index)
{
    Json item = {
        {"candidate_index", candidate_index},
        {"color", auto_aim::COLORS.at(static_cast<std::size_t>(candidate.color))},
        {"name", auto_aim::ARMOR_NAMES.at(static_cast<std::size_t>(candidate.name))},
        {"type", auto_aim::ARMOR_TYPES.at(static_cast<std::size_t>(candidate.type))},
        {"confidence", candidate.confidence},
        {"center_px", {candidate.center.x, candidate.center.y}},
        {"same_label_as_source", candidate.color == source.color &&
            candidate.name == source.name && candidate.type == source.type},
        {"left_lamp_length_px", cv::norm(candidate.points[3] - candidate.points[0])},
        {"right_lamp_length_px", cv::norm(candidate.points[2] - candidate.points[1])},
        {"corner_offset_from_source_px", Json::array()},
        {"corners_px", Json::array()}
    };
    const double center_offset = cv::norm(candidate.center - source.center);
    item["center_offset_from_source_px"] = center_offset;
    for (std::size_t index = 0; index < candidate.points.size(); ++index) {
        const auto & point = candidate.points[index];
        item["corners_px"].push_back({point.x, point.y});
        item["corner_offset_from_source_px"].push_back(
            cv::norm(point - source.points[index]));
    }

    try {
        const auto pnp_candidates = solver.pnp_candidates(candidate);
        if (pnp_candidates.empty()) {
            item["pnp_camera_range_m"] = nullptr;
            item["pnp_reprojection_error_px"] = nullptr;
        } else {
            const auto best = std::min_element(
                pnp_candidates.begin(), pnp_candidates.end(),
                [](const auto & left, const auto & right) {
                    return left.reprojection_error < right.reprojection_error;
                });
            const double camera_range = best->xyz_in_camera.norm();
            item["pnp_camera_range_m"] = std::isfinite(camera_range)
                ? Json(camera_range) : Json(nullptr);
            item["pnp_reprojection_error_px"] = std::isfinite(best->reprojection_error)
                ? Json(best->reprojection_error) : Json(nullptr);
        }
    } catch (const cv::Exception &) {
        item["pnp_camera_range_m"] = nullptr;
        item["pnp_reprojection_error_px"] = nullptr;
    }
    return item;
}

Json prepare_roi_depth_probe(
    const auto_aim::Armor & source, const cv::Mat & image,
    auto_aim::YOLOV8 & roi_detector, const auto_aim::Solver & solver,
    int frame_index, std::size_t source_index,
    double scale, int min_side, int max_side)
{
    Json record = {
        {"source_detector_index", source_index},
        {"source_center_px", {source.center.x, source.center.y}},
        {"source_corners_px", Json::array()},
        {"roi_px", nullptr},
        {"roi_inference_ms", nullptr},
        {"roi_candidate_count", 0},
        {"same_label_candidate_count", 0},
        {"nearest_same_label_center_offset_px", nullptr},
        {"nearest_same_label_camera_range_m", nullptr},
        {"roi_score_candidate_count", nullptr},
        {"roi_nms_candidate_count", nullptr},
        {"roi_classifier_rejected_count", nullptr},
        {"source_left_lamp_length_px", source.points.size() == 4
            ? Json(cv::norm(source.points[3] - source.points[0])) : Json(nullptr)},
        {"source_right_lamp_length_px", source.points.size() == 4
            ? Json(cv::norm(source.points[2] - source.points[1])) : Json(nullptr)},
        {"source_camera_range_m", nullptr},
        {"source_reprojection_error_px", nullptr},
        {"candidates", Json::array()}
    };
    for (const auto & point : source.points) {
        record["source_corners_px"].push_back({point.x, point.y});
    }
    if (source.points.size() != 4) {
        record["status"] = "invalid_source_corners";
        return record;
    }

    try {
        const auto pnp_candidates = solver.pnp_candidates(source);
        const auto best = std::min_element(
            pnp_candidates.begin(), pnp_candidates.end(),
            [](const auto & left, const auto & right) {
                return left.reprojection_error < right.reprojection_error;
            });
        const double source_range = pnp_candidates.empty()
            ? std::numeric_limits<double>::quiet_NaN()
            : best->xyz_in_camera.norm();
        if (!pnp_candidates.empty()) {
            if (std::isfinite(source_range)) record["source_camera_range_m"] = source_range;
            if (std::isfinite(best->reprojection_error)) {
                record["source_reprojection_error_px"] = best->reprojection_error;
            }
        }
    } catch (const cv::Exception &) {
        // 源检测的 PnP 失败只影响诊断字段，不中断回放。
    }

    const auto roi = roi_depth_probe_rect(
        source, image.size(), scale, min_side, max_side);
    if (roi.empty()) {
        record["status"] = "invalid_roi";
        return record;
    }
    record["roi_px"] = {roi.x, roi.y, roi.width, roi.height};
    roi_detector.set_fixed_roi(roi);
    const auto roi_detect_start = Clock::now();
    const auto roi_candidates = roi_detector.detect(image, frame_index);
    record["roi_inference_ms"] =
        std::chrono::duration<double, std::milli>(Clock::now() - roi_detect_start).count();
    record["roi_candidate_count"] = roi_candidates.size();
    const auto detector_debug = roi_detector.debug_info();
    record["roi_score_candidate_count"] = detector_debug.score_candidates;
    record["roi_nms_candidate_count"] = detector_debug.nms_candidates;
    record["roi_classifier_rejected_count"] = detector_debug.classifier_rejected;

    double nearest_offset = std::numeric_limits<double>::infinity();
    std::size_t same_label_candidate_count = 0;
    std::size_t candidate_index = 0;
    for (const auto & candidate : roi_candidates) {
        auto item = roi_depth_candidate_json(candidate, source, solver, candidate_index++);
        if (item["same_label_as_source"].get<bool>()) {
            ++same_label_candidate_count;
            const double offset = item["center_offset_from_source_px"].get<double>();
            if (offset < nearest_offset) {
                nearest_offset = offset;
                record["nearest_same_label_center_offset_px"] = offset;
                record["nearest_same_label_camera_range_m"] = item["pnp_camera_range_m"];
            }
        }
        record["candidates"].push_back(std::move(item));
    }
    record["same_label_candidate_count"] = same_label_candidate_count;
    record["status"] = roi_candidates.empty() ? "no_roi_detection" :
        (same_label_candidate_count == 0
            ? "no_same_label_candidate" :
             same_label_candidate_count == 1
                ? "one_same_label_candidate" : "multiple_same_label_candidates");
    return record;
}

// 仅在唯一同标签候选且 PnP 距离有效时返回 ROI 角点，避免把相邻板歧义带入 Tracker。
std::optional<std::vector<cv::Point2f>> unique_same_label_roi_corners(
    const Json & record)
{
    if (record.value("same_label_candidate_count", std::size_t{0}) != 1 ||
        !record.contains("candidates") || !record["candidates"].is_array()) {
        return std::nullopt;
    }

    for (const auto & candidate : record["candidates"]) {
        if (!candidate.value("same_label_as_source", false)) continue;
        const auto corners = candidate.find("corners_px");
        const auto range = candidate.find("pnp_camera_range_m");
        if (corners == candidate.end() || !corners->is_array() || corners->size() != 4 ||
            range == candidate.end() || !range->is_number() ||
            !std::isfinite(range->get<double>()) || range->get<double>() <= 0.0) {
            return std::nullopt;
        }

        std::vector<cv::Point2f> points;
        points.reserve(4);
        for (const auto & point : *corners) {
            if (!point.is_array() || point.size() != 2 ||
                !point[0].is_number() || !point[1].is_number()) {
                return std::nullopt;
            }
            const double x = point[0].get<double>();
            const double y = point[1].get<double>();
            if (!std::isfinite(x) || !std::isfinite(y)) return std::nullopt;
            points.emplace_back(static_cast<float>(x), static_cast<float>(y));
        }
        return points;
    }
    return std::nullopt;
}

const std::string kKeys =
    "{help h usage ?||输出命令行参数说明}"
    "{@config-path|configs/real_auto_aim.yaml|算法配置路径}"
    "{input-jsonl||standard_srm 回放输入 JSONL}"
    "{frames-dir||与输入 JSONL 对应的 PNG 帧目录}"
    "{output-jsonl||离线重跑结果 JSONL}"
    "{show|false|显示离线回放窗口，按 q 或 Esc 退出}"
    "{model-frame-id|-1|只显示此 frame_id 的单帧整车模型}"
    "{model-frame-output||可选的单帧模型 PNG 输出路径}"
    "{endpoint-range-shadow|false|离线影子试验：用原分辨率灯条端点拟合替换选中观测的距离分量}"
    "{endpoint-edge-probe|false|记录灯条灰度、颜色对比和端点候选，不改变观测}"
    "{roi-depth-probe|false|用独立 YOLOv8 实例重跑局部 ROI 并记录候选，不改变跟踪观测}"
    "{roi-corners-into-tracker|false|离线 A/B：唯一同标签 ROI 候选角点用于 Tracker，歧义或失败时回退原角点}"
    "{roi-depth-probe-scale|12.0|ROI 边长系数：系数×平均灯条像长，仅离线探针使用}"
    "{roi-depth-probe-min-side|160|ROI 边长下限，单位 px}"
    "{roi-depth-probe-max-side|320|ROI 边长上限，单位 px}"
    "{distance-update-r-scale|1.0|仅离线：EKF 更新距离方差倍率，范围 [1, 1e6]；不改变初始化、关联和 NIS}"
    "{distance-quality-mode|off|仅离线：off 关闭；record 记录固定分支距离敏感度；adaptive 条件降权；sigma 几何尺度合成}"
    "{distance-quality-reference|0|主序列记录组冻结的参考敏感度，单位 m/px；adaptive 必须为正}"
    "{distance-quality-sigma-px|0|sigma 模式的角点误差假设，单位 px；0 关闭}"
    "{distance-quality-floor-m|0|sigma 模式的距离方差下界，单位 m²}"
    "{yaw-update-sigma-deg|0|仅离线：板面 yaw 更新标准差，单位度；0 保持当前基准 R}"
    "{association-image-cost|legacy|仅离线：legacy 保持三项几何代价；corner-rms 使用独立最优归一化四角 RMS}"
    "{association-image-corner-rms-max-px|0|仅离线：候选角点 RMS 上限，单位 px；0 关闭}"
    "{association-physical-feasible-gate|false|仅离线：将 physical_feasible 作为关联资格条件}"
    "{association-require-gate-passed|false|仅离线：要求评分、位置、距离、马氏距离及启用时的图像门全部通过}"
    "{compare-centered-mpc|false|额外计算居中窗口 MPC 诊断，不参与模拟 TX}"
    "{planner-gimbal-response-delay-ms|-1|离线覆盖 Planner 滞后配置，单位 ms；-1 使用 YAML，范围 [0, 100]}"
    "{compare-quintic|false|额外计算因果五次多项式选板过渡，仅用于回放诊断}"
    "{quintic-frame-ms|65|五次曲线校验单条 TX 步长时假定的帧周期，单位 ms}"
    "{debug-udp|false|按回放时间实时发送逐帧诊断 JSON 到 127.0.0.1:9871}"
    "{simulate-tx|true|在回放结果中记录 Planner TX 参考角，不连接串口}"
    "{tx-use-nominal-speed|false|弹速反馈不可用时允许模拟名义弹速 TX}"
    "{tx-period-ms|20|TX 初始步长计算周期，单位 ms}"
    "{tx-command-ttl-ms|100|两次模拟命令间隔上限，单位 ms}"
    "{tx-max-yaw-step-deg|2.0|单条模拟命令 yaw 最大变化量，单位 deg}"
    "{tx-max-pitch-step-deg|1.0|单条模拟命令 pitch 最大变化量，单位 deg}"
    "{tx-max-yaw-rate-deg-s|60.0|yaw 最大角速度，单位 deg/s}"
    "{tx-max-pitch-rate-deg-s|30.0|pitch 最大角速度，单位 deg/s}"
    "{tx-follow-temp-lost|true|temp_lost 期间继续模拟受限预测跟随}"
    "{tx-temp-lost-frames|5|允许外推的最大连续丢失帧数}"
    "{tx-temp-lost-rate-scale|0.35|外推限速比例，范围 (0,1]}";

void copy_timeline_fields(const Json & source, Json & output)
{
    for (const char * key : {
             "frame_id", "device_ticks", "tick_hz", "timestamp_source",
             "captured_frame", "replay_time_s", "image_width", "image_height"}) {
        if (source.contains(key)) output[key] = source[key];
    }
}

void append_target(Json & output, const std::list<auto_aim::Target> & targets)
{
    output["targets"] = targets.size();
    if (targets.empty()) return;

    const auto & target = targets.front();
    const auto state = target.ekf_x();
    output["current_armor_id"] = target.last_id;
    if (state.size() >= 11) {
        output["center_x"] = state[0];
        output["center_y"] = state[2];
        output["center_z"] = state[4];
        output["vx"] = state[1];
        output["vy"] = state[3];
        output["vz"] = state[5];
        output["center_yaw"] = state[6];
        output["angular_velocity"] = state[7];
        output["radius"] = state[8];
        output["radius_delta"] = state[9];
        output["height_delta"] = state[10];
    }

    const auto armors = target.armor_xyza_list();
    if (target.last_id >= 0 &&
        static_cast<std::size_t>(target.last_id) < armors.size()) {
        const auto & current = armors[static_cast<std::size_t>(target.last_id)];
        output["current_x"] = current[0];
        output["current_y"] = current[1];
        output["current_z"] = current[2];
        output["current_yaw"] = current[3];
    }
}

void append_plan(
    Json & output, const auto_aim::Plan & plan,
    bool has_aimer_armor)
{
    output["planner_status"] =
        !plan.diagnostic_valid ? "invalid" :
        plan.solver_converged ? "ok" : "unconverged_diagnostic";
    output["planner_control"] =
        plan.control && has_aimer_armor ? 1 : 0;
    output["planner_fire"] = plan.fire ? 1 : 0;
    output["planner_feedback_yaw_error_deg"] = nullptr;
    output["planner_feedback_pitch_error_deg"] = nullptr;
    output["planner_fire_alignment_error_deg"] = nullptr;
    output["planner_fire_reference_error_deg"] = nullptr;
    output["planner_gimbal_response_delay_ms"] = plan.gimbal_response_delay_ms;
    output["planner_target_lead_compensation_ms"] =
        plan.target_lead_compensation_ms;
    output["planner_preview_switch_ms"] =
        plan.preview_switch_step >= 0
            ? Json(plan.preview_switch_step * auto_aim::DT * 1000.0)
            : Json(nullptr);
    output["planner_preview_switch_id"] =
        plan.preview_switch_id >= 0
            ? Json(plan.preview_switch_id) : Json(nullptr);
    output["planner_solver_converged"] = plan.solver_converged;
    output["planner_yaw_solver_status"] = plan.yaw_solver_status;
    output["planner_pitch_solver_status"] = plan.pitch_solver_status;
    output["planner_yaw_solver_iterations"] = plan.yaw_solver_iterations;
    output["planner_pitch_solver_iterations"] = plan.pitch_solver_iterations;

    for (const char * key : {
             "planner_target_yaw_deg", "planner_target_pitch_deg",
             "planner_command_target_yaw_deg", "planner_command_target_pitch_deg",
             "planner_yaw_deg", "planner_pitch_deg",
             "planner_yaw_20ms_deg", "planner_pitch_20ms_deg",
             "planner_target_yaw_100ms_deg", "planner_target_pitch_100ms_deg",
             "planner_yaw_100ms_deg", "planner_pitch_100ms_deg",
             "planner_yaw_vel_rad_s", "planner_pitch_vel_rad_s",
             "planner_yaw_acc_rad_s2", "planner_pitch_acc_rad_s2",
             "planner_yaw_primal_residual_max", "planner_yaw_dual_residual_max",
             "planner_pitch_primal_residual_max", "planner_pitch_dual_residual_max"}) {
        output[key] = nullptr;
    }
    if (!plan.diagnostic_valid) return;

    constexpr double kRadToDeg = 180.0 / CV_PI;
    output["planner_feedback_yaw_error_deg"] =
        plan.feedback_yaw_error_rad * kRadToDeg;
    output["planner_feedback_pitch_error_deg"] =
        plan.feedback_pitch_error_rad * kRadToDeg;
    output["planner_fire_alignment_error_deg"] =
        plan.fire_alignment_error_rad * kRadToDeg;
    output["planner_fire_reference_error_deg"] =
        plan.fire_reference_error_rad * kRadToDeg;
    output["planner_target_yaw_deg"] = plan.target_yaw * kRadToDeg;
    output["planner_target_pitch_deg"] = plan.target_pitch * kRadToDeg;
    output["planner_command_target_yaw_deg"] =
        plan.command_target_yaw * kRadToDeg;
    output["planner_command_target_pitch_deg"] =
        plan.command_target_pitch * kRadToDeg;
    output["planner_yaw_deg"] = plan.yaw * kRadToDeg;
    output["planner_pitch_deg"] = plan.pitch * kRadToDeg;
    output["planner_yaw_20ms_deg"] =
        tools::limit_rad(plan.yaw + auto_aim::DT * plan.yaw_vel) * kRadToDeg;
    output["planner_pitch_20ms_deg"] =
        (plan.pitch + auto_aim::DT * plan.pitch_vel) * kRadToDeg;
    output["planner_target_yaw_100ms_deg"] = plan.target_yaw_100ms * kRadToDeg;
    output["planner_target_pitch_100ms_deg"] = plan.target_pitch_100ms * kRadToDeg;
    output["planner_yaw_100ms_deg"] = plan.yaw_100ms * kRadToDeg;
    output["planner_pitch_100ms_deg"] = plan.pitch_100ms * kRadToDeg;
    output["planner_yaw_vel_rad_s"] = plan.yaw_vel;
    output["planner_pitch_vel_rad_s"] = plan.pitch_vel;
    output["planner_yaw_acc_rad_s2"] = plan.yaw_acc;
    output["planner_pitch_acc_rad_s2"] = plan.pitch_acc;
    output["planner_yaw_primal_residual_max"] = plan.yaw_primal_residual_max;
    output["planner_yaw_dual_residual_max"] = plan.yaw_dual_residual_max;
    output["planner_pitch_primal_residual_max"] = plan.pitch_primal_residual_max;
    output["planner_pitch_dual_residual_max"] = plan.pitch_dual_residual_max;
}

void draw_replay_polygon(
    cv::Mat & image, const std::vector<cv::Point2f> & points,
    const cv::Scalar & color, const std::string & label)
{
    if (points.size() < 3) return;
    for (std::size_t i = 0; i < points.size(); ++i) {
        cv::line(image, points[i], points[(i + 1) % points.size()],
                 color, 2, cv::LINE_AA);
        cv::circle(image, points[i], 4, color, -1, cv::LINE_AA);
    }
    const auto bounds = cv::boundingRect(points);
    cv::putText(image, label, {bounds.x, std::max(20, bounds.y - 6)},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv::LINE_AA);
}

// 单帧几何核验只画板框轮廓，避免检测框、标签和角点标记干扰判断。
void draw_replay_outline(
    cv::Mat & image, const std::vector<cv::Point2f> & points,
    const cv::Scalar & color)
{
    if (points.size() < 3) return;
    for (std::size_t i = 0; i < points.size(); ++i) {
        cv::line(image, points[i], points[(i + 1) % points.size()],
                 color, 2, cv::LINE_AA);
    }
}

void draw_replay_text(cv::Mat & image, const std::string & text, int line)
{
    cv::putText(image, text, {15, 28 + line * 26},
                cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
}

struct PnpObservation
{
    std::size_t detection_index = 0;
    const auto_aim::Armor * armor = nullptr;
    auto_aim::PnpCandidateDebug pose;
};

struct ObservationVehicleModel
{
    bool joint = false;
    std::size_t first_detection = 0;
    std::size_t second_detection = 0;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Matrix3d base_rotation = Eigen::Matrix3d::Identity();
    auto_aim::ArmorType armor_type = auto_aim::ArmorType::small;
    double height_delta = 0.0;
    double center_gap_m = 0.0;
    double rotation_gap_rad = 0.0;
};

Eigen::Vector3d inferred_center(
    const auto_aim::PnpCandidateDebug & pose, double radius)
{
    return pose.xyz_in_world + radius * Eigen::Vector3d(
        std::cos(pose.yaw_in_world), std::sin(pose.yaw_in_world), 0.0);
}

Eigen::Matrix3d undo_model_phase(const Eigen::Matrix3d & armor_rotation, int model_id)
{
    const double phase = model_id * CV_PI / 2.0;
    return Eigen::AngleAxisd(-phase, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
           armor_rotation;
}

std::optional<ObservationVehicleModel> fit_observation_vehicle_model(
    const std::list<auto_aim::Armor> & detections,
    const auto_aim::Solver & solver, double radius)
{
    std::vector<PnpObservation> observations;
    std::size_t detection_index = 0;
    for (const auto & armor : detections) {
        for (const auto & pose : solver.pnp_candidates(armor)) {
            if (!pose.xyz_in_world.allFinite() || !pose.R_armor2world.allFinite() ||
                !std::isfinite(pose.yaw_in_world) ||
                !std::isfinite(pose.reprojection_error)) {
                continue;
            }
            PnpObservation observation;
            observation.detection_index = detection_index;
            observation.armor = &armor;
            observation.pose = pose;
            observations.push_back(std::move(observation));
        }
        ++detection_index;
    }

    // 两个不同装甲板必须能反推出同一中心和同一车体朝向；按中心、姿态、PnP
    // 重投影误差依次排序，避免把米、弧度和像素混成一个没有物理意义的总分。
    std::optional<ObservationVehicleModel> best_pair;
    std::tuple<double, double, double, double> best_pair_rank{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
    for (std::size_t first = 0; first < observations.size(); ++first) {
        const auto & a = observations[first];
        for (std::size_t second = first + 1; second < observations.size(); ++second) {
            const auto & b = observations[second];
            if (a.detection_index == b.detection_index ||
                a.armor->color != b.armor->color ||
                a.armor->name != b.armor->name ||
                a.armor->type != b.armor->type) {
                continue;
            }

            for (int first_id = 0; first_id < 4; ++first_id) {
                for (int second_id = 0; second_id < 4; ++second_id) {
                    if (first_id == second_id) continue;

                    const auto center_a = inferred_center(a.pose, radius);
                    const auto center_b = inferred_center(b.pose, radius);
                    const double dx = center_a.x() - center_b.x();
                    const double dy = center_a.y() - center_b.y();
                    const bool first_high = first_id % 2 != 0;
                    const bool second_high = second_id % 2 != 0;
                    const double dz = first_high == second_high
                        ? center_a.z() - center_b.z() : 0.0;
                    const double center_gap = std::sqrt(dx * dx + dy * dy + dz * dz);

                    const auto base_rotation_a =
                        undo_model_phase(a.pose.R_armor2world, first_id);
                    const auto base_rotation_b =
                        undo_model_phase(b.pose.R_armor2world, second_id);
                    const Eigen::AngleAxisd relative_rotation(
                        base_rotation_a.transpose() * base_rotation_b);
                    const double rotation_gap = std::abs(relative_rotation.angle());
                    const double reprojection_error =
                        a.pose.reprojection_error + b.pose.reprojection_error;
                    const double confidence_rank =
                        -(a.armor->confidence + b.armor->confidence);
                    const auto rank = std::make_tuple(
                        center_gap, rotation_gap, reprojection_error, confidence_rank);
                    if (rank >= best_pair_rank) continue;

                    Eigen::Quaterniond rotation_a(base_rotation_a);
                    Eigen::Quaterniond rotation_b(base_rotation_b);
                    if (rotation_a.dot(rotation_b) < 0.0) {
                        rotation_b.coeffs() *= -1.0;
                    }
                    Eigen::Quaterniond mean_rotation;
                    mean_rotation.coeffs() = rotation_a.coeffs() + rotation_b.coeffs();
                    if (mean_rotation.norm() <= 1e-9) continue;
                    mean_rotation.normalize();

                    ObservationVehicleModel model;
                    model.joint = true;
                    model.first_detection = a.detection_index;
                    model.second_detection = b.detection_index;
                    model.armor_type = a.armor->type;
                    model.center.x() = (center_a.x() + center_b.x()) / 2.0;
                    model.center.y() = (center_a.y() + center_b.y()) / 2.0;
                    if (first_high != second_high) {
                        const auto & even_pose = first_high ? b.pose : a.pose;
                        const auto & odd_pose = first_high ? a.pose : b.pose;
                        model.center.z() = even_pose.xyz_in_world.z();
                        model.height_delta = odd_pose.xyz_in_world.z() - model.center.z();
                    } else {
                        // 同奇偶板位无法从两框单独估计高度差，沿用零高度差先验。
                        model.center.z() =
                            (a.pose.xyz_in_world.z() + b.pose.xyz_in_world.z()) / 2.0;
                    }
                    model.base_rotation = mean_rotation.toRotationMatrix();
                    model.center_gap_m = center_gap;
                    model.rotation_gap_rad = rotation_gap;
                    best_pair = model;
                    best_pair_rank = rank;
                }
            }
        }
    }
    if (best_pair) return best_pair;

    // 单检测帧只能给出一个锚定假设；没有第二块板时不能把它当作整车模型已验证。
    const PnpObservation * best_single = nullptr;
    for (const auto & observation : observations) {
        if (!best_single ||
            observation.armor->confidence > best_single->armor->confidence ||
            (observation.armor->confidence == best_single->armor->confidence &&
             observation.pose.reprojection_error < best_single->pose.reprojection_error)) {
            best_single = &observation;
        }
    }
    if (!best_single) return std::nullopt;

    ObservationVehicleModel model;
    model.first_detection = best_single->detection_index;
    model.armor_type = best_single->armor->type;
    model.center = inferred_center(best_single->pose, radius);
    model.base_rotation = best_single->pose.R_armor2world;
    return model;
}
}

int main(int argc, char * argv[])
{
    try {
        cv::CommandLineParser cli(argc, argv, kKeys);
        if (cli.has("help")) {
            cli.printMessage();
            return 0;
        }
        if (!cli.check()) {
            cli.printErrors();
            return 2;
        }

        const auto config_path = cli.get<std::string>(0);
        const std::filesystem::path input_path(cli.get<std::string>("input-jsonl"));
        const std::filesystem::path frames_dir(cli.get<std::string>("frames-dir"));
        const std::filesystem::path output_path(cli.get<std::string>("output-jsonl"));
        if (config_path.empty() || input_path.empty() || frames_dir.empty() ||
            output_path.empty()) {
            throw std::invalid_argument(
                "Config, input JSONL, frames directory and output JSONL are required");
        }
        if (std::filesystem::weakly_canonical(input_path) ==
            std::filesystem::weakly_canonical(output_path)) {
            throw std::invalid_argument("Input and output JSONL paths must differ");
        }
        const auto yaml = YAML::LoadFile(config_path);
        const auto config = real_auto_aim::validate_real_config(yaml);
        // 使用与 Tracker 相同的半径先验重建纯图像几何诊断模型。
        const double diagnostic_radius = yaml["standard_radius"].as<double>(0.22);
        const bool show = cli.get<bool>("show");
        const int model_frame_id = cli.get<int>("model-frame-id");
        const std::filesystem::path model_frame_output(
            cli.get<std::string>("model-frame-output"));
        if (model_frame_id == 0 || model_frame_id < -1 ||
            (model_frame_id > 0 && !show && model_frame_output.empty()) ||
            (model_frame_id < 0 && !model_frame_output.empty())) {
            throw std::invalid_argument("Invalid single-frame model display options");
        }
        if (!model_frame_output.empty() &&
            std::filesystem::exists(model_frame_output)) {
            throw std::invalid_argument("Single-frame PNG output already exists");
        }
        if (!model_frame_output.empty() &&
            (std::filesystem::weakly_canonical(model_frame_output) ==
                 std::filesystem::weakly_canonical(input_path) ||
             std::filesystem::weakly_canonical(model_frame_output) ==
                 std::filesystem::weakly_canonical(output_path))) {
            throw std::invalid_argument(
                "Single-frame PNG output must differ from JSONL paths");
        }
        if (show) cv::namedWindow("standard_srm_replay", cv::WINDOW_NORMAL);
        const bool endpoint_range_shadow_enabled = cli.get<bool>("endpoint-range-shadow");
        const bool endpoint_edge_probe_enabled = cli.get<bool>("endpoint-edge-probe");
        const bool roi_depth_probe_enabled = cli.get<bool>("roi-depth-probe");
        const bool roi_corners_into_tracker_enabled =
            cli.get<bool>("roi-corners-into-tracker");
        const double roi_depth_probe_scale = cli.get<double>("roi-depth-probe-scale");
        const int roi_depth_probe_min_side = cli.get<int>("roi-depth-probe-min-side");
        const int roi_depth_probe_max_side = cli.get<int>("roi-depth-probe-max-side");
        const double distance_update_r_scale = cli.get<double>("distance-update-r-scale");
        if (!std::isfinite(distance_update_r_scale) ||
            distance_update_r_scale < 1.0 || distance_update_r_scale > 1e6) {
            throw std::invalid_argument(
                "--distance-update-r-scale must be finite and in [1, 1e6]");
        }
        const double yaw_update_sigma_deg = cli.get<double>("yaw-update-sigma-deg");
        if (!std::isfinite(yaw_update_sigma_deg) ||
            yaw_update_sigma_deg < 0.0 || yaw_update_sigma_deg > 180.0) {
            throw std::invalid_argument(
                "--yaw-update-sigma-deg must be finite and in [0, 180]");
        }
        const auto association_image_cost = cli.get<std::string>("association-image-cost");
        if (association_image_cost != "legacy" && association_image_cost != "corner-rms") {
            throw std::invalid_argument(
                "--association-image-cost must be legacy or corner-rms");
        }
        const double association_corner_rms_limit_px =
            cli.get<double>("association-image-corner-rms-max-px");
        const bool association_physical_feasible_gate =
            cli.get<bool>("association-physical-feasible-gate");
        const bool association_require_gate_passed =
            cli.get<bool>("association-require-gate-passed");
        if (!std::isfinite(association_corner_rms_limit_px) ||
            association_corner_rms_limit_px < 0.0 ||
            association_corner_rms_limit_px > 10000.0) {
            throw std::invalid_argument(
                "--association-image-corner-rms-max-px must be finite and in [0, 10000]");
        }
        if ((association_corner_rms_limit_px > 0.0 &&
             association_physical_feasible_gate) ||
            (association_require_gate_passed &&
             (association_corner_rms_limit_px > 0.0 ||
              association_physical_feasible_gate))) {
            throw std::invalid_argument(
                "Select only one association experiment filter per replay");
        }
        // 第一阶段只隔离排序代价，避免与候选硬过滤同时改变而混淆归因。
        if (association_image_cost == "corner-rms" &&
            (association_corner_rms_limit_px > 0.0 ||
             association_physical_feasible_gate || association_require_gate_passed)) {
            throw std::invalid_argument(
                "corner-rms ranking must be evaluated without association experiment filters");
        }
        const auto distance_quality_mode = cli.get<std::string>("distance-quality-mode");
        const double distance_quality_reference = cli.get<double>("distance-quality-reference");
        const double distance_quality_sigma_px = cli.get<double>("distance-quality-sigma-px");
        const double distance_quality_floor_m = cli.get<double>("distance-quality-floor-m");
        if ((distance_quality_mode != "off" && distance_quality_mode != "record" &&
             distance_quality_mode != "adaptive" && distance_quality_mode != "sigma") ||
            !std::isfinite(distance_quality_reference) || distance_quality_reference < 0.0 ||
            (distance_quality_mode == "adaptive" && distance_quality_reference <= 0.0) ||
            (distance_quality_mode != "adaptive" && distance_quality_reference != 0.0)) {
            throw std::invalid_argument("Invalid distance quality mode/reference");
        }
        // sigma 模式必须有正 σ_px；其他模式必须为 0，避免两种规则同时生效无法归因。
        if (!std::isfinite(distance_quality_sigma_px) || distance_quality_sigma_px < 0.0 ||
            distance_quality_sigma_px > 50.0 ||
            !std::isfinite(distance_quality_floor_m) || distance_quality_floor_m < 0.0 ||
            (distance_quality_mode == "sigma" && distance_quality_sigma_px <= 0.0) ||
            (distance_quality_mode != "sigma" && distance_quality_sigma_px != 0.0)) {
            throw std::invalid_argument("Invalid distance quality sigma/floor");
        }
        // 一轮只改变一种因素，防止质量实验混入 ROI、排序或固定 R 扫描。
        if (distance_quality_mode != "off" &&
            (distance_update_r_scale != 1.0 || yaw_update_sigma_deg != 0.0 ||
             association_image_cost != "legacy" || association_corner_rms_limit_px != 0.0 ||
             association_physical_feasible_gate || association_require_gate_passed ||
             endpoint_range_shadow_enabled || roi_corners_into_tracker_enabled)) {
            throw std::invalid_argument("Distance quality experiment requires the baseline pipeline");
        }
        if (roi_depth_probe_enabled &&
            (!std::isfinite(roi_depth_probe_scale) || roi_depth_probe_scale <= 0.0 ||
             roi_depth_probe_min_side <= 0 ||
             roi_depth_probe_max_side < roi_depth_probe_min_side)) {
            throw std::invalid_argument("Invalid ROI depth probe scale or side limits");
        }
        if (roi_depth_probe_enabled && yaml["yolo_name"].as<std::string>() != "yolov8") {
            throw std::invalid_argument("--roi-depth-probe requires yolo_name: yolov8");
        }
        if (roi_corners_into_tracker_enabled && !roi_depth_probe_enabled) {
            throw std::invalid_argument(
                "--roi-corners-into-tracker requires --roi-depth-probe");
        }
        if (roi_depth_probe_enabled && endpoint_range_shadow_enabled) {
            throw std::invalid_argument(
                "--roi-depth-probe cannot be combined with --endpoint-range-shadow");
        }
        const bool simulate_tx = cli.get<bool>("simulate-tx");
        const bool compare_centered = cli.get<bool>("compare-centered-mpc");
        const double planner_delay_override_ms =
            cli.get<double>("planner-gimbal-response-delay-ms");
        if (!std::isfinite(planner_delay_override_ms) ||
            planner_delay_override_ms < -1.0 || planner_delay_override_ms > 100.0) {
            throw std::invalid_argument(
                "--planner-gimbal-response-delay-ms must be -1 or in [0, 100]");
        }
        const bool compare_quintic = cli.get<bool>("compare-quintic");
        const double quintic_frame_ms = cli.get<double>("quintic-frame-ms");
        const bool tx_use_nominal_speed = cli.get<bool>("tx-use-nominal-speed");
        const int tx_period_ms = cli.get<int>("tx-period-ms");
        const int tx_ttl_ms = cli.get<int>("tx-command-ttl-ms");
        const int tx_lost_frames = cli.get<int>("tx-temp-lost-frames");
        const double tx_lost_scale = cli.get<double>("tx-temp-lost-rate-scale");
        const bool tx_follow_lost = cli.get<bool>("tx-follow-temp-lost");
        const real_auto_aim::TxRateLimits tx_limits{
            cli.get<double>("tx-max-yaw-rate-deg-s"),
            cli.get<double>("tx-max-pitch-rate-deg-s"),
            cli.get<double>("tx-max-yaw-step-deg"),
            cli.get<double>("tx-max-pitch-step-deg")};
        const auto positive_finite = [](double value) {
            return std::isfinite(value) && value > 0.0;
        };
        if (tx_period_ms <= 0 || tx_ttl_ms <= 0 || tx_lost_frames < 0 ||
            !std::isfinite(tx_lost_scale) || tx_lost_scale <= 0.0 ||
            tx_lost_scale > 1.0 ||
            !positive_finite(tx_limits.yaw_rate_deg_s) ||
            !positive_finite(tx_limits.pitch_rate_deg_s) ||
            !positive_finite(tx_limits.yaw_step_cap_deg) ||
            !positive_finite(tx_limits.pitch_step_cap_deg) ||
            (compare_quintic && !positive_finite(quintic_frame_ms))) {
            throw std::invalid_argument("Invalid offline TX simulation limits");
        }

        std::ifstream input(input_path);
        if (!input) throw std::runtime_error("Unable to open input JSONL");
        const auto parent = output_path.parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent);
        std::ofstream output_file(output_path, std::ios::out | std::ios::trunc);
        if (!output_file) throw std::runtime_error("Unable to open output JSONL");

        std::optional<tools::Plotter> udp_plotter;
        if (cli.get<bool>("debug-udp")) udp_plotter.emplace();
        std::optional<double> udp_origin_s;
        std::optional<Clock::time_point> udp_origin_wall;
        const auto emit_sample = [&](const Json & sample) {
            output_file << sample.dump() << '\n';
            if (!output_file) throw std::runtime_error("Failed to write replay JSONL");
            if (!udp_plotter) return;

            // 算法仍使用采集时的虚拟时间；仅按回放时间控制 UDP 展示节奏。
            // 推理慢于采集时不丢帧，发送自然会落后于一倍速。
            if (sample.contains("replay_time_s") &&
                sample["replay_time_s"].is_number()) {
                const double replay_s = sample["replay_time_s"].get<double>();
                if (std::isfinite(replay_s)) {
                    if (!udp_origin_s) {
                        udp_origin_s = replay_s;
                        udp_origin_wall = Clock::now();
                    } else {
                        const auto due = *udp_origin_wall +
                            std::chrono::duration_cast<Clock::duration>(
                                std::chrono::duration<double>(
                                    replay_s - *udp_origin_s));
                        std::this_thread::sleep_until(due);
                    }
                }
            }
            udp_plotter->plot(sample);
        };

        auto_aim::YOLO detector(config_path, false);
        std::optional<auto_aim::YOLOV8> roi_depth_detector;
        if (roi_depth_probe_enabled) roi_depth_detector.emplace(config_path, false);
        std::optional<auto_aim::Detector> endpoint_detector;
        if (endpoint_range_shadow_enabled || endpoint_edge_probe_enabled) {
            endpoint_detector.emplace(config_path, false);
        }
        auto_aim::Solver solver(config_path);
        auto_aim::Tracker tracker(config_path, solver);
        tracker.set_distance_update_r_scale(distance_update_r_scale);
        tracker.set_distance_quality_experiment(
            distance_quality_mode != "off", distance_quality_reference);
        if (distance_quality_mode == "sigma") {
            tracker.set_distance_quality_sigma_px(distance_quality_sigma_px, distance_quality_floor_m);
        }
        tracker.set_yaw_update_sigma_deg(yaw_update_sigma_deg);
        tracker.set_association_image_corner_rms_max_px(association_corner_rms_limit_px);
        tracker.set_association_physical_feasible_gate(association_physical_feasible_gate);
        tracker.set_association_gate_passed_filter(association_require_gate_passed);
        tracker.set_association_corner_rms_ranking(association_image_cost == "corner-rms");
        tracker.set_association_update_trace_enabled(true);
        auto_aim::Aimer aimer(config_path);
        auto_aim::Planner planner(config_path);
        if (planner_delay_override_ms >= 0.0) {
            planner.set_gimbal_response_delay_ms(planner_delay_override_ms);
        }
        std::optional<auto_aim::Planner> centered_planner;
        if (compare_centered) centered_planner.emplace(config_path);
        std::optional<real_auto_aim::QuinticSwitchDiagnostic> quintic;
        if (compare_quintic) {
            const double max_speed_deg_s = std::min(
                tx_limits.yaw_rate_deg_s,
                tx_limits.yaw_step_cap_deg / (quintic_frame_ms / 1000.0));
            quintic.emplace(
                max_speed_deg_s * CV_PI / 180.0,
                yaml["max_yaw_acc"].as<double>());
        }

        std::string line;
        std::size_t line_number = 0;
        std::size_t replayed_frames = 0;
        bool model_frame_rendered = false;
        std::optional<double> previous_replay_time;
        std::optional<real_auto_aim::TxAnglesDeg> last_tx_command;
        std::optional<real_auto_aim::TxAnglesDeg> last_quintic_command;
        std::optional<double> last_quintic_command_time_s;
        const Clock::time_point virtual_epoch{};
        while (std::getline(input, line)) {
            ++line_number;
            if (line.empty()) continue;
            const Json source = Json::parse(line);
            Json result = Json::object();
            result["offline_replay"] = true;
            copy_timeline_fields(source, result);

            if (source.value("event", std::string{}) != "frame") {
                result["event"] = "skip";
                result["skip_reason"] = source.value(
                    "skip_reason", std::string("source row has no matched feedback pose"));
                emit_sample(result);
                continue;
            }
            if (source.value("replay_schema_version", 0) != 1) {
                throw std::runtime_error(
                    "Input lacks replay inputs; recapture with --replay-frame-dir (line " +
                    std::to_string(line_number) + ")");
            }

            const double elapsed_s = source.at("replay_time_s").get<double>();
            if (!std::isfinite(elapsed_s) ||
                (previous_replay_time && elapsed_s <= *previous_replay_time)) {
                throw std::runtime_error("Replay timestamps are not strictly increasing");
            }
            previous_replay_time = elapsed_s;
            const auto frame_time = virtual_epoch +
                std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(elapsed_s));

            const std::filesystem::path frame_name(
                source.at("replay_frame_file").get<std::string>());
            if (frame_name.empty() || frame_name.is_absolute() ||
                frame_name.filename() != frame_name) {
                throw std::runtime_error("Invalid replay frame filename");
            }
            const auto image_path = frames_dir / frame_name;
            const cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
            if (image.empty()) {
                throw std::runtime_error("Unable to read replay frame: " + image_path.string());
            }
            if (image.cols != config.image_width || image.rows != config.image_height) {
                throw std::runtime_error("Replay frame size differs from calibration image_size");
            }

            Eigen::Quaterniond pose(
                source.at("replay_gimbal_to_world_w").get<double>(),
                source.at("replay_gimbal_to_world_x").get<double>(),
                source.at("replay_gimbal_to_world_y").get<double>(),
                source.at("replay_gimbal_to_world_z").get<double>());
            if (!pose.coeffs().allFinite() || pose.norm() < 1e-9) {
                throw std::runtime_error("Invalid replay gimbal quaternion");
            }
            pose.normalize();
            solver.set_gimbal_to_world(pose);

            auto armors = detector.detect(image);
            auto_aim::EndpointRangeShadowMap endpoint_shadow_map;
            std::vector<auto_aim::EndpointRangeShadowDebug> endpoint_shadow_records;
            std::unordered_map<const auto_aim::Armor *, Json> endpoint_probe_records;
            std::unordered_map<const auto_aim::Armor *, Json> roi_depth_probe_records;
            std::unordered_map<const auto_aim::Armor *, std::vector<cv::Point2f>>
                roi_corner_overrides;
            if (roi_depth_probe_enabled) {
                std::size_t source_index = 0;
                for (const auto & armor : armors) {
                    auto probe = prepare_roi_depth_probe(
                        armor, image, *roi_depth_detector, solver,
                        static_cast<int>(replayed_frames), source_index++,
                        roi_depth_probe_scale, roi_depth_probe_min_side,
                        roi_depth_probe_max_side);
                    if (roi_corners_into_tracker_enabled) {
                        const auto same_label_count = probe.value(
                            "same_label_candidate_count", std::size_t{0});
                        auto roi_corners = unique_same_label_roi_corners(probe);
                        if (roi_corners) {
                            roi_corner_overrides.emplace(&armor, std::move(*roi_corners));
                        }
                        probe["tracker_corner_override_applied"] = roi_corners.has_value();
                        probe["tracker_corner_override_reason"] = roi_corners
                            ? "unique_same_label_candidate"
                            : same_label_count > 1
                                ? "ambiguous_same_label_candidates"
                                : same_label_count == 1
                                    ? "invalid_unique_same_label_candidate"
                                    : "no_same_label_candidate";
                    }
                    roi_depth_probe_records.emplace(&armor, std::move(probe));
                }
            }
            if (endpoint_range_shadow_enabled || endpoint_edge_probe_enabled) {
                cv::Mat gray;
                cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
                for (auto & armor : armors) {
                    if (endpoint_edge_probe_enabled && armor.points.size() == 4) {
                        endpoint_probe_records.emplace(
                            &armor, Json{
                                {"left", lamp_endpoint_profile(
                                    image, armor.points[0], armor.points[3],
                                    armor.color)},
                                {"right", lamp_endpoint_profile(
                                    image, armor.points[1], armor.points[2],
                                    armor.color)}
                            });
                    }
                    auto_aim::EndpointRangeShadowDebug shadow;
                    prepare_endpoint_range_shadow(
                        armor, image, gray, *endpoint_detector, solver,
                        yaml["threshold"].as<double>(), shadow);
                    endpoint_shadow_map.emplace(&armor, shadow);
                    endpoint_shadow_records.push_back(std::move(shadow));
                }
            }
            const auto detected_armors = armors;
            if (roi_corners_into_tracker_enabled) {
                for (auto & armor : armors) {
                    const auto found = roi_corner_overrides.find(&armor);
                    if (found == roi_corner_overrides.end()) continue;
                    armor.points = found->second;
                    armor.center = (armor.points[0] + armor.points[1] +
                        armor.points[2] + armor.points[3]) / 4.0F;
                }
            }
            real_auto_aim::diagnostics::append_detector_debug_json(
                result, detector.debug_info(), detected_armors);
            auto targets = tracker.track(
                armors, frame_time, true,
                endpoint_range_shadow_enabled ? &endpoint_shadow_map : nullptr);
            // 叠图固定使用本帧检测时间的 Tracker 状态，不使用 Aimer 的延迟预测状态。
            std::list<auto_aim::Target> frame_time_targets;
            if (show || model_frame_id > 0) frame_time_targets = targets;
            // 复用真机日志的门控字段，记录本次离线重跑实际做出的关联判断。
            real_auto_aim::diagnostics::append_association_json(
                result, tracker.association_debug(), solver, armors);
            const auto & association = tracker.association_debug();
            result["association_image_cost_mode"] = association_image_cost;
            result["association_experiment_corner_rms_limit_px"] =
                association_corner_rms_limit_px;
            result["association_experiment_physical_feasible_gate"] =
                association_physical_feasible_gate;
            result["association_experiment_require_gate_passed"] =
                association_require_gate_passed;
            result["association_experiment_unmatched_count"] =
                association.unmatched_detection_count;
            result["association_experiment_model_candidates"] = Json::array();
            result["association_experiment_candidate_count"] =
                association.experiment_candidates.size();
            for (const auto & candidate : association.experiment_candidates) {
                Json item = Json::object();
                item["detection_index"] = candidate.detection_index;
                item["model_id"] = candidate.model_id;
                item["image_corner_rms_px"] = candidate.image_corner_error_px;
                item["image_geometry_cost"] = candidate.image_geometry_cost;
                item["image_min_corner_rms_px"] = candidate.image_min_corner_rms_px;
                item["image_min_corner_rms_norm"] = candidate.image_min_corner_rms_norm;
                item["image_min_corner_point_permutation"] =
                    candidate.image_min_corner_point_permutation;
                item["image_ranking_cost"] = candidate.image_ranking_cost;
                item["predicted_yaw"] = candidate.predicted_yaw;
                item["predicted_corners_px"] = Json::array();
                if (candidate.image_geometry_valid) {
                    // 复用关联时的预测坐标和本帧 Solver 变换，避免用更新后状态画叠图。
                    const auto projected = solver.reproject_armor(
                        Eigen::Vector3d(candidate.predicted_x, candidate.predicted_y,
                                        candidate.predicted_z),
                        candidate.predicted_yaw, association.model_type,
                        association.model_name);
                    for (const auto & point : projected) {
                        item["predicted_corners_px"].push_back(
                            Json::array({point.x, point.y}));
                    }
                }
                item["image_geometry_valid"] = candidate.image_geometry_valid;
                item["physical_feasible"] = candidate.physical_feasible;
                item["gate_passed"] = candidate.gate_passed;
                item["assignment_eligible"] = candidate.assignment_eligible;
                item["corner_rms_limit_passed"] =
                    candidate.image_corner_rms_limit_passed;
                item["physical_feasible_filter_passed"] =
                    candidate.physical_feasible_filter_passed;
                item["gate_passed_filter_passed"] =
                    candidate.gate_passed_filter_passed;
                item["selected"] = candidate.selected;
                item["accepted"] = candidate.accepted;
                item["rejection_reason"] = candidate.accepted
                    ? "accepted"
                    : candidate.selected
                        ? "target_update_rejected"
                        : !candidate.assignment_eligible
                            ? (association_require_gate_passed &&
                               !candidate.gate_passed
                                ? "gate_passed"
                                : association_corner_rms_limit_px > 0.0 &&
                                  !candidate.image_corner_rms_limit_passed
                                    ? "corner_rms_limit"
                                    : "physical_feasible_gate")
                            : "unassigned_or_conflict";
                result["association_experiment_model_candidates"].push_back(
                    std::move(item));
            }
            const auto vector3_json = [](const std::array<double, 3> & values) {
                return Json::array({values[0], values[1], values[2]});
            };
            result["association_experiment_updates"] = Json::array();
            for (const auto & update : association.update_trace) {
                Json item = Json::object();
                item["detection_index"] = update.detection_index;
                item["model_id"] = update.model_id;
                item["attempted"] = update.target_update.attempted;
                item["accepted"] = update.target_update.accepted;
                item["innovation_yaw_pitch_distance_angle"] =
                    update.target_update.innovation;
                const auto & trace = update.target_update;
                const auto & quality = update.distance_quality;
                item["distance_r_base_m2"] = trace.distance_r_base_m2;
                item["distance_r_used_m2"] = trace.distance_r_used_m2;
                item["distance_quality"] = {
                    {"valid", quality.valid}, {"reason", quality.reason},
                    {"selected_branch", quality.selected_branch},
                    {"sensitivity_m_per_px", quality.valid
                        ? Json(quality.sensitivity_m_per_px) : Json(nullptr)},
                    {"gradient_m_per_px", quality.valid
                        ? Json(quality.range_gradient_m_per_px) : Json(nullptr)},
                    {"scale", update.distance_quality_scale},
                    {"elapsed_ms", update.distance_quality_ms}};
                item["yaw_update_innovation_variance_rad2"] =
                    trace.yaw_update_scale_valid
                      ? Json(trace.yaw_update_variance_rad2) : Json(nullptr);
                item["yaw_update_standardized_innovation"] =
                    trace.yaw_update_scale_valid
                      ? Json(trace.yaw_update_standardized_innovation) : Json(nullptr);
                item["yaw_base_innovation_variance_rad2"] =
                    trace.yaw_base_scale_valid
                      ? Json(trace.yaw_base_variance_rad2) : Json(nullptr);
                item["yaw_base_standardized_innovation"] =
                    trace.yaw_base_scale_valid
                      ? Json(trace.yaw_base_standardized_innovation) : Json(nullptr);
                item["center_gain_by_state_xyz_and_measurement"] =
                    update.target_update.center_gain;
                item["ippe_branch_yaw_world_rad"] = Json::array();
                item["ippe_branch_reprojection_error_px"] = Json::array();
                for (std::size_t i = 0; i < update.ippe_branch_count; ++i) {
                    item["ippe_branch_yaw_world_rad"].push_back(
                        update.ippe_branch_yaw_world_rad[i]);
                    item["ippe_branch_reprojection_error_px"].push_back(
                        update.ippe_branch_reprojection_error_px[i]);
                }
                item["ippe_branch_yaw_gap_rad"] = update.ippe_branch_pair_valid
                    ? Json(update.ippe_branch_yaw_gap_rad) : Json(nullptr);
                item["ippe_branch_reprojection_gap_px"] = update.ippe_branch_pair_valid
                    ? Json(update.ippe_branch_reprojection_gap_px) : Json(nullptr);
                item["selected_ippe_branch_index"] = update.selected_ippe_branch_index;
                item["center_before_xyz"] =
                    vector3_json(update.target_update.center_before);
                item["center_after_attempt_xyz"] =
                    vector3_json(update.target_update.center_after_attempt);
                item["center_after_commit_xyz"] =
                    vector3_json(update.target_update.center_after_commit);
                result["association_experiment_updates"].push_back(std::move(item));
            }
            result["endpoint_range_shadow_enabled"] = endpoint_range_shadow_enabled;
            result["endpoint_edge_probe_enabled"] = endpoint_edge_probe_enabled;
            result["roi_depth_probe_enabled"] = roi_depth_probe_enabled;
            result["roi_corners_into_tracker"] = roi_corners_into_tracker_enabled;
            result["distance_update_r_scale"] = distance_update_r_scale;
            result["distance_quality_mode"] = distance_quality_mode;
            result["distance_quality_reference_m_per_px"] = distance_quality_reference;
            result["distance_quality_sigma_px"] = distance_quality_sigma_px;
            result["distance_quality_floor_m2"] = distance_quality_floor_m;
            result["yaw_update_sigma_deg"] = yaw_update_sigma_deg;
            result["tracker_distance_update_gain_x_per_m"] = nullptr;
            result["tracker_distance_update_gain_y_per_m"] = nullptr;
            result["tracker_distance_update_gain_z_per_m"] = nullptr;
            result["tracker_distance_update_gain_center_norm_per_m"] = nullptr;
            result["tracker_distance_update_r_base_m2"] = nullptr;
            result["tracker_distance_update_r_used_m2"] = nullptr;
            if (tracker.observation_accepted() && tracker.association_debug().accepted_count > 0) {
                const auto * diagnostic_target = targets.empty()
                    ? tracker.diagnostic_target() : &targets.front();
                if (diagnostic_target != nullptr) {
                    const auto & ekf = diagnostic_target->ekf();
                    if (ekf.last_kalman_gain.rows() > 4 &&
                        ekf.last_kalman_gain.cols() > 2 &&
                        ekf.last_measurement_noise.rows() > 2 &&
                        ekf.last_nis_measurement_noise.rows() > 2) {
                        // Target 状态的中心位置位于 x、y、z 行 0、2、4；列 2 是距离观测。
                        const double gain_x = ekf.last_kalman_gain(0, 2);
                        const double gain_y = ekf.last_kalman_gain(2, 2);
                        const double gain_z = ekf.last_kalman_gain(4, 2);
                        result["tracker_distance_update_gain_x_per_m"] = gain_x;
                        result["tracker_distance_update_gain_y_per_m"] = gain_y;
                        result["tracker_distance_update_gain_z_per_m"] = gain_z;
                        result["tracker_distance_update_gain_center_norm_per_m"] =
                            std::sqrt(gain_x * gain_x + gain_y * gain_y + gain_z * gain_z);
                        result["tracker_distance_update_r_base_m2"] =
                            ekf.last_nis_measurement_noise(2, 2);
                        result["tracker_distance_update_r_used_m2"] =
                            ekf.last_measurement_noise(2, 2);
                    }
                }
            }
            if (roi_depth_probe_enabled) {
                result["roi_depth_probe_scale"] = roi_depth_probe_scale;
                result["roi_depth_probe_min_side_px"] = roi_depth_probe_min_side;
                result["roi_depth_probe_max_side_px"] = roi_depth_probe_max_side;
            }
            if (endpoint_range_shadow_enabled || endpoint_edge_probe_enabled) {
                result["endpoint_range_shadow_detections"] = Json::array();
                std::size_t endpoint_index = 0;
                for (const auto & shadow : endpoint_shadow_records) {
                    Json item = endpoint_range_shadow_json(shadow);
                    item["detection_index"] = endpoint_index++;
                    result["endpoint_range_shadow_detections"].push_back(std::move(item));
                }
                std::size_t processed_index = 0;
                for (const auto & armor : armors) {
                    const auto found = endpoint_shadow_map.find(&armor);
                    if (found != endpoint_shadow_map.end()) {
                        auto & detection = result["association_detections"][processed_index];
                        detection["endpoint_range_shadow"] =
                            endpoint_range_shadow_json(found->second);
                        for (auto & candidate : result["association_selected_candidates"]) {
                            if (candidate.value("detection_index", -1) ==
                                static_cast<int>(processed_index)) {
                                candidate["endpoint_range_reliable"] = found->second.reliable;
                                candidate["endpoint_range_distance_replaced"] =
                                    found->second.distance_replaced;
                                candidate["endpoint_range_update_accepted"] =
                                    found->second.update_accepted;
                                candidate["endpoint_range_raw_distance_m"] =
                                    found->second.selected_raw_distance_m;
                                candidate["endpoint_range_fitted_distance_m"] =
                                    found->second.selected_fitted_distance_m;
                            }
                        }
                    }
                    ++processed_index;
                }
            }
            if (endpoint_edge_probe_enabled) {
                std::size_t processed_index = 0;
                for (const auto & armor : armors) {
                    const auto found = endpoint_probe_records.find(&armor);
                    if (found != endpoint_probe_records.end()) {
                        result["association_detections"][processed_index]
                            ["endpoint_edge_probe"] = found->second;
                    }
                    ++processed_index;
                }
            }
            if (roi_depth_probe_enabled) {
                std::size_t processed_index = 0;
                for (const auto & armor : armors) {
                    const auto found = roi_depth_probe_records.find(&armor);
                    if (found != roi_depth_probe_records.end()) {
                        result["association_detections"][processed_index]
                            ["roi_depth_probe"] = found->second;
                    }
                    ++processed_index;
                }
            }
            const double aimer_speed =
                source.at("replay_aimer_bullet_speed_mps").get<double>();
            const double aimer_offset_ms =
                source.at("replay_aimer_offset_ms").get<double>();
            const auto aimer_time = frame_time +
                std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double, std::milli>(aimer_offset_ms));
            const auto aim = aimer.aim(targets, aimer_time, aimer_speed, false);

            result["event"] = "frame";
            result["image_width"] = image.cols;
            result["image_height"] = image.rows;
            result["detected"] = armors.size();
            result["tracker_state"] = tracker.state();
            result["tracker_generation"] = tracker.target_generation();
            result["temp_lost_count"] = tracker.temp_lost_count();
            result["tracker_observation_accepted"] = tracker.observation_accepted();
            result["tracker_geometry_branch"] = tracker.geometry_branch();
            result["tracker_geometry_trusted"] = tracker.geometry_trusted();
            result["tracker_geometry_constraint_active"] =
                tracker.geometry_constraint_active();
            append_target(result, targets);

            const bool aim_valid = aim.control && aimer.debug_aim_point.valid;
            result["aim_control"] = aim.control ? 1 : 0;
            result["aim_shoot"] = aim.shoot ? 1 : 0;
            result["diagnostic_yaw"] = aim_valid ? Json(aim.yaw) : Json(nullptr);
            result["diagnostic_pitch"] = aim_valid ? Json(aim.pitch) : Json(nullptr);
            result["diagnostic_yaw_deg"] =
                aim_valid ? Json(aim.yaw * 180.0 / CV_PI) : Json(nullptr);
            result["diagnostic_pitch_deg"] =
                aim_valid ? Json(aim.pitch * 180.0 / CV_PI) : Json(nullptr);
            result["prediction_dt"] = aimer.debug_prediction_dt;
            result["delay_time"] = aimer_offset_ms / 1000.0;
            result["base_prediction_dt"] = aimer.debug_base_prediction_dt;
            result["fly_time"] = aimer.debug_fly_time;
            result["aim_armor_id"] =
                aim_valid ? Json(aimer.debug_aim_point.armor_id) : Json(nullptr);
            if (aim_valid) {
                const auto & future = aimer.debug_aim_point.xyza;
                result["future_x"] = future[0];
                result["future_y"] = future[1];
                result["future_z"] = future[2];
                result["future_yaw"] = future[3];
            }

            for (const char * key : {
                     "planner_measured_yaw_deg", "planner_measured_pitch_deg",
                     "planner_measured_yaw_vel_rad_s",
                     "planner_measured_pitch_vel_rad_s", "planner_feedback_age_ms",
                     "planner_feedback_interval_ms", "planner_state_source",
                     "capture_to_aimer_ms"}) {
                if (source.contains(key)) result[key] = source[key];
            }
            result["planner_status"] = "no_tracking_target";
            result["planner_control"] = 0;
            if (compare_centered) {
                result["centered_valid"] = false;
                result["centered_solver_converged"] = false;
                result["centered_target_yaw_deg"] = nullptr;
                result["centered_yaw_deg"] = nullptr;
                result["centered_yaw_solver_status"] = nullptr;
                result["centered_yaw_solver_iterations"] = nullptr;
            }
            // 原始日志在 Planner 暂无有效速度时会写 null；回放输出统一为字符串以兼容绘图。
            const auto planner_speed_source = source.find("planner_speed_source");
            result["planner_speed_source"] =
                planner_speed_source != source.end() && planner_speed_source->is_string()
                    ? *planner_speed_source
                    : Json("unavailable");
            result["planner_yaw_deg"] = nullptr;
            result["planner_pitch_deg"] = nullptr;

            // 回放按重新运行后的 Tracker 状态判断暂失，避免沿用采集时的 TX 状态。
            const bool extrapolating =
                tx_follow_lost && tracker.state() == "temp_lost" &&
                tracker.temp_lost_count() <= tx_lost_frames;
            const bool planner_enabled = source.value("replay_planner_enabled", false);
            std::optional<auto_aim::Plan> replay_plan;
            std::optional<int> replay_armor_id;
            std::optional<auto_aim::PlannerState> replay_state;
            auto_aim::SwitchPreview switch_preview;
            if (planner_enabled && !targets.empty() &&
                (tracker.state() == "tracking" ||
                 (extrapolating && tracker.state() == "temp_lost"))) {
                auto target = targets.front();
                const double planner_offset_ms =
                    source.at("replay_planner_offset_ms").get<double>();
                const auto planner_time = frame_time +
                    std::chrono::duration_cast<Clock::duration>(
                        std::chrono::duration<double, std::milli>(planner_offset_ms));
                target.predict(planner_time);
                const auto armor_id =
                    aim.control && aimer.debug_aim_point.valid
                        ? std::optional<int>{aimer.debug_aim_point.armor_id}
                        : std::nullopt;
                const auto state = auto_aim::PlannerState{
                    source.at("replay_planner_state_yaw_rad").get<double>(),
                    source.at("replay_planner_state_yaw_vel_rad_s").get<double>(),
                    source.at("replay_planner_state_pitch_rad").get<double>(),
                    source.at("replay_planner_state_pitch_vel_rad_s").get<double>()};
                const auto plan = planner.plan(
                    target,
                    source.at("replay_planner_bullet_speed_mps").get<double>(),
                    state,
                    armor_id,
                    armor_id ? &aimer : nullptr,
                    quintic ? &switch_preview : nullptr);
                replay_plan = plan;
                replay_armor_id = armor_id;
                replay_state = state;
                result["planner_bullet_speed_mps"] =
                    source.at("replay_planner_bullet_speed_mps");
                result["planner_state_yaw_deg"] = state.yaw * 180.0 / CV_PI;
                result["planner_state_pitch_deg"] = state.pitch * 180.0 / CV_PI;
                append_plan(result, plan, armor_id.has_value());
                if (centered_planner && armor_id) {
                    const auto centered = centered_planner->plan_centered_diagnostic(
                        target,
                        source.at("replay_planner_bullet_speed_mps").get<double>(),
                        *armor_id, aimer);
                    result["centered_valid"] = centered.diagnostic_valid;
                    result["centered_solver_converged"] =
                        centered.solver_converged;
                    if (centered.diagnostic_valid) {
                        result["centered_target_yaw_deg"] =
                            centered.target_yaw * 180.0 / CV_PI;
                        result["centered_yaw_deg"] =
                            centered.yaw * 180.0 / CV_PI;
                        result["centered_yaw_solver_status"] =
                            centered.yaw_solver_status;
                        result["centered_yaw_solver_iterations"] =
                            centered.yaw_solver_iterations;
                    }
                }
            }

            // 用采集时的虚拟时间和 planner 耗时确定帧间限速；不读取墙钟或访问串口。
            const double captured_planner_offset_ms =
                source.contains("replay_planner_offset_ms") &&
                        source["replay_planner_offset_ms"].is_number()
                    ? source["replay_planner_offset_ms"].get<double>() : 0.0;
            const double captured_planner_ms =
                source.contains("planner_ms") && source["planner_ms"].is_number()
                    ? source["planner_ms"].get<double>() : 0.0;
            const double command_time_s =
                elapsed_s + (captured_planner_offset_ms + captured_planner_ms) / 1000.0;
            std::optional<real_auto_aim::QuinticSwitchDiagnostic::Sample> quintic_sample;
            if (quintic) {
                result["quintic_reference_yaw_deg"] = nullptr;
                result["quintic_transition_ms"] = nullptr;
                result["quintic_mode"] = "no_reference";
                if (replay_plan && replay_armor_id &&
                    replay_plan->diagnostic_valid) {
                    const auto & plan = *replay_plan;
                    const double reference_yaw = plan.control
                        ? tools::limit_rad(plan.yaw + auto_aim::DT * plan.yaw_vel)
                        : static_cast<double>(plan.target_yaw);
                    quintic_sample = quintic->update(
                        command_time_s, tracker.target_generation(),
                        *replay_armor_id, reference_yaw, switch_preview);
                    result["quintic_reference_yaw_deg"] =
                        quintic_sample->yaw_rad * 180.0 / CV_PI;
                    result["quintic_transition_ms"] =
                        quintic_sample->transition_ms;
                    result["quintic_mode"] = quintic_sample->mode;
                }
            }
            bool tx_sent = false;
            std::string tx_mode = "none";
            std::string tx_source = "none";
            std::optional<real_auto_aim::TxAnglesDeg> tx_command;
            const auto hold_tx = [&]() {
                if (!last_tx_command && replay_state) {
                    last_tx_command = real_auto_aim::TxAnglesDeg{
                        replay_state->yaw * 180.0 / CV_PI,
                        replay_state->pitch * 180.0 / CV_PI};
                }
                if (last_tx_command) {
                    tx_command = last_tx_command;
                    tx_sent = true;
                    tx_mode = "hold_last_aim";
                    tx_source = "hold";
                }
            };
            if (simulate_tx) {
                if (!replay_plan || !replay_state || !replay_armor_id) {
                    hold_tx();
                } else {
                    const auto & plan = *replay_plan;
                    const bool measured_speed =
                        source.value("tx_speed_source", std::string{}) == "feedback";
                    const bool nominal_speed =
                        tx_use_nominal_speed &&
                        source.contains("replay_planner_bullet_speed_mps") &&
                        source["replay_planner_bullet_speed_mps"].is_number();
                    const bool finite_target =
                        std::isfinite(plan.target_yaw) &&
                        std::isfinite(plan.target_pitch);
                    if (!plan.diagnostic_valid || !finite_target ||
                        (!measured_speed && !nominal_speed)) {
                        if (extrapolating) hold_tx();
                        else {
                            last_tx_command.reset();
                        }
                    } else {
                        const real_auto_aim::TxAnglesDeg target_command =
                            plan.control
                                ? real_auto_aim::TxAnglesDeg{
                                    tools::limit_rad(
                                        plan.yaw + auto_aim::DT * plan.yaw_vel) *
                                        180.0 / CV_PI,
                                    (plan.pitch + auto_aim::DT * plan.pitch_vel) *
                                        180.0 / CV_PI}
                                : real_auto_aim::TxAnglesDeg{
                                    plan.command_target_yaw * 180.0 / CV_PI,
                                    plan.command_target_pitch * 180.0 / CV_PI};
                        if (!std::isfinite(target_command.yaw) ||
                            !std::isfinite(target_command.pitch)) {
                            if (extrapolating) hold_tx();
                            else last_tx_command.reset();
                        } else {
                            last_tx_command = target_command;
                            tx_command = target_command;
                            tx_sent = true;
                            tx_mode = extrapolating ? "extrapolate" : "tracking";
                            tx_source = plan.control
                                ? "mpc_reference_20ms" : "target_reference";
                        }
                    }
                }
            }
            result["replay_tx_simulated"] = simulate_tx;
            result["replay_tx_command_sent"] = tx_sent;
            result["replay_tx_command_mode"] = tx_mode;
            result["replay_tx_command_source"] = tx_source;
            result["replay_tx_extrapolating"] =
                tx_sent && tx_mode == "extrapolate";
            result["replay_tx_command_yaw_deg"] =
                tx_command ? Json(tx_command->yaw) : Json(nullptr);
            result["replay_tx_command_pitch_deg"] =
                tx_command ? Json(tx_command->pitch) : Json(nullptr);

            if (quintic) {
                std::optional<real_auto_aim::TxAnglesDeg> quintic_command;
                if (simulate_tx && tx_source == "hold") {
                    if (!last_quintic_command && replay_state) {
                        last_quintic_command = real_auto_aim::TxAnglesDeg{
                            replay_state->yaw * 180.0 / CV_PI,
                            replay_state->pitch * 180.0 / CV_PI};
                    }
                    quintic_command = last_quintic_command;
                    if (quintic_command) {
                        last_quintic_command_time_s = std::max(
                            command_time_s,
                            last_quintic_command_time_s.value_or(command_time_s));
                    }
                } else if (simulate_tx && quintic_sample && replay_state &&
                           replay_plan && tx_sent && tx_source != "none") {
                    const real_auto_aim::TxAnglesDeg base = last_quintic_command
                        ? *last_quintic_command
                        : real_auto_aim::TxAnglesDeg{
                            replay_state->yaw * 180.0 / CV_PI,
                            replay_state->pitch * 180.0 / CV_PI};
                    const double elapsed_tx_s = last_quintic_command_time_s
                        ? std::clamp(
                            command_time_s - *last_quintic_command_time_s,
                            0.0, tx_ttl_ms / 1000.0)
                        : tx_period_ms / 1000.0;
                    const auto & plan = *replay_plan;
                    const double pitch_deg = plan.control
                        ? (plan.pitch + auto_aim::DT * plan.pitch_vel) * 180.0 / CV_PI
                        : plan.target_pitch * 180.0 / CV_PI;
                    quintic_command = real_auto_aim::limit_tx_angles(
                        base,
                        {quintic_sample->yaw_rad * 180.0 / CV_PI, pitch_deg},
                        elapsed_tx_s, tx_limits,
                        extrapolating ? tx_lost_scale : 1.0);
                    last_quintic_command = quintic_command;
                    last_quintic_command_time_s = std::max(
                        command_time_s,
                        last_quintic_command_time_s.value_or(command_time_s));
                } else if (tx_source == "none") {
                    last_quintic_command.reset();
                    last_quintic_command_time_s.reset();
                }
                result["quintic_tx_command_yaw_deg"] = quintic_command
                    ? Json(quintic_command->yaw) : Json(nullptr);
            }

            emit_sample(result);
            ++replayed_frames;
            if (show && model_frame_id < 0) {
                cv::Mat display = image.clone();
                const cv::Scalar model_color(0, 255, 0);
                std::string model_status = "model=none";
                if (!frame_time_targets.empty()) {
                    const auto & target = frame_time_targets.front();
                    const auto filtered_armors = target.armor_xyza_list();
                    for (std::size_t model_id = 0;
                         model_id < filtered_armors.size(); ++model_id) {
                        const auto & armor = filtered_armors[model_id];
                        draw_replay_polygon(
                            display,
                            solver.reproject_armor(
                                armor.head<3>(), armor[3],
                                target.armor_type, target.name),
                            model_color,
                            "EKF id=" + std::to_string(model_id));
                    }
                    model_status = "model=tracker_ekf_at_frame_time";
                }
                for (const auto & armor : detected_armors) {
                    draw_replay_polygon(
                        display, armor.points, cv::Scalar(0, 255, 255),
                        "det " + std::to_string(armor.confidence));
                }
                cv::rotate(display, display, cv::ROTATE_180);
                draw_replay_text(
                    display,
                    "frame=" + std::to_string(replayed_frames) +
                        " detected=" + std::to_string(detected_armors.size()) +
                        " targets=" + std::to_string(targets.size()),
                    0);
                draw_replay_text(
                    display,
                    "state=" + tracker.state() +
                        " obs=" + (tracker.observation_accepted()
                            ? std::string("accepted") : std::string("not_accepted")),
                    1);
                draw_replay_text(display, model_status, 2);
                cv::imshow("standard_srm_replay", display);
                const int key = cv::waitKey(1) & 0xff;
                if (key == 'q' || key == 27) break;
            }
            if (model_frame_id > 0 &&
                source.value("frame_id", -1) == model_frame_id) {
                const auto vehicle_model = fit_observation_vehicle_model(
                    detected_armors, solver, diagnostic_radius);
                if (vehicle_model && vehicle_model->joint) {
                    cv::Mat display = image.clone();
                    const auto & model = *vehicle_model;
                    const double base_yaw = std::atan2(
                        model.base_rotation(1, 0), model.base_rotation(0, 0));
                    for (int model_id = 0; model_id < 4; ++model_id) {
                        const double phase = model_id * CV_PI / 2.0;
                        const double angle = base_yaw + phase;
                        const Eigen::Vector3d board_position(
                            model.center.x() - diagnostic_radius * std::cos(angle),
                            model.center.y() - diagnostic_radius * std::sin(angle),
                            model.center.z() +
                                (model_id % 2 ? model.height_delta : 0.0));
                        const Eigen::Matrix3d board_rotation =
                            Eigen::AngleAxisd(phase, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            model.base_rotation;
                        draw_replay_outline(
                            display,
                            solver.reproject_armor_pose(
                                board_position, board_rotation, model.armor_type),
                            cv::Scalar(255, 255, 0));
                    }
                    cv::rotate(display, display, cv::ROTATE_180);
                    draw_replay_text(
                        display, "observation-fit model; not Tracker EKF", 0);
                    if (!model_frame_output.empty()) {
                        const auto parent = model_frame_output.parent_path();
                        if (!parent.empty()) std::filesystem::create_directories(parent);
                        if (!cv::imwrite(model_frame_output.string(), display)) {
                            throw std::runtime_error("Failed to save single-frame model PNG");
                        }
                    }
                    model_frame_rendered = true;
                    if (show) {
                        cv::imshow("standard_srm_replay", display);
                        const int key = cv::waitKey(0) & 0xff;
                        if (key == 'q' || key == 27) break;
                    }
                }
            }
        }
        if (replayed_frames == 0) {
            throw std::runtime_error("No replayable frames found in input JSONL");
        }
        if (model_frame_id > 0 && !model_frame_rendered) {
            throw std::runtime_error(
                "Selected frame was missing or had no two-detection joint model");
        }
        std::cout << "Replayed " << replayed_frames << " frames to "
                  << output_path << '\n';
        return 0;
    } catch (const std::exception & error) {
        std::cerr << "standard_srm_replay failed: " << error.what() << '\n';
        return 1;
    }
}
