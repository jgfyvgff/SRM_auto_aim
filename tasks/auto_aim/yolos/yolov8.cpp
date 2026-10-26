#include "yolov8.hpp"

#include <fmt/chrono.h>
#include <omp.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <optional>
#include <random>
#include <stdexcept>

#include "tasks/auto_aim/classifier.hpp"
#include "tasks/auto_aim/dynamic_roi.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
namespace
{
// 连续漏检约五帧后重回整帧，避免先验窗口长期留在目标原位置。
constexpr int kRoiMissTolerance = 5;
constexpr int kInputSize = 416;
}  // namespace

YOLOV8::YOLOV8(const std::string & config_path, bool debug)
: classifier_(config_path), detector_(config_path), debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolov8_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  dynamic_roi_ = yaml["yolov8_dynamic_roi"].IsDefined()
                   ? yaml["yolov8_dynamic_roi"].as<bool>()
                   : (yaml["dynamic_roi"].IsDefined() ? yaml["dynamic_roi"].as<bool>() : false);
  dynamic_roi_scale_ = yaml["yolov8_dynamic_roi_scale"].IsDefined()
                         ? yaml["yolov8_dynamic_roi_scale"].as<double>()
                         : 10.4;
  dynamic_roi_min_width_ = yaml["yolov8_dynamic_roi_min_width"].IsDefined()
                             ? yaml["yolov8_dynamic_roi_min_width"].as<int>()
                             : 208;
  score_threshold_ = yaml["yolov8_score_threshold"].IsDefined()
                       ? yaml["yolov8_score_threshold"].as<float>()
                       : 0.7F;
  if (!std::isfinite(dynamic_roi_scale_) || dynamic_roi_scale_ <= 0.0 ||
      dynamic_roi_min_width_ <= 0 || !std::isfinite(score_threshold_) ||
      score_threshold_ <= 0.0F || score_threshold_ > 1.0F ||
      !std::isfinite(min_confidence_) || min_confidence_ < 0.0 ||
      min_confidence_ > 1.0) {
    throw std::invalid_argument("Invalid YOLOv8 ROI or confidence parameter");
  }
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, kInputSize, kInputSize, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  // TODO: ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

void YOLOV8::set_fixed_roi(const cv::Rect & roi)
{
  if (roi.x < 0 || roi.y < 0 || roi.width <= 0 || roi.height <= 0) {
    throw std::invalid_argument("YOLOv8 fixed ROI must have positive in-image coordinates");
  }
  roi_ = roi;
  use_roi_ = true;
  dynamic_roi_ = false;
  last_target_box_.reset();
  roi_miss_streak_ = 0;
}

std::list<Armor> YOLOV8::detect(const cv::Mat & raw_img, int frame_count)
{
  debug_info_ = {};
  debug_info_.model_name = "yolov8";
  debug_info_.roi_active = dynamic_roi_ || use_roi_;
  debug_info_.roi_miss_streak_before = roi_miss_streak_;
  if (raw_img.empty()) {
    debug_info_.roi_miss_streak_after = roi_miss_streak_;
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat crop_img = raw_img;
  offset_ = {};
  if (dynamic_roi_) {
    const bool use_prior = last_target_box_.has_value() && roi_miss_streak_ <= kRoiMissTolerance;
    debug_info_.prior_used = use_prior;
    debug_info_.prior_box_valid = use_prior;
    if (use_prior) debug_info_.prior_box = *last_target_box_;
    applied_roi_ = dynamic_roi_rect(
      use_prior ? last_target_box_ : std::nullopt, dynamic_roi_scale_,
      dynamic_roi_min_width_, raw_img.cols, raw_img.rows);
    crop_img = raw_img(applied_roi_);
    offset_ = cv::Point2f(applied_roi_.x, applied_roi_.y);
  } else if (use_roi_) {
    auto rect = roi_;
    if (rect.width == -1) rect.width = raw_img.cols - rect.x;
    if (rect.height == -1) rect.height = raw_img.rows - rect.y;
    if (rect.width <= 0 || rect.height <= 0 || rect.x < 0 || rect.y < 0 ||
        rect.x + rect.width > raw_img.cols || rect.y + rect.height > raw_img.rows) {
      throw std::invalid_argument("YOLOv8 static ROI exceeds image bounds");
    }
    applied_roi_ = rect;
    crop_img = raw_img(rect);
    offset_ = cv::Point2f(rect.x, rect.y);
  }

  auto armors = infer_and_parse(crop_img, raw_img, frame_count);
  if (dynamic_roi_) {
    if (!armors.empty()) {
      const auto best = std::max_element(
        armors.begin(), armors.end(),
        [](const Armor & a, const Armor & b) { return a.confidence < b.confidence; });
      // box 的宽度不随裁剪原点变化；center 已由 Armor 构造函数回填原图坐标。
      const float width = static_cast<float>(best->box.width);
      if (width > 0.0F) {
        last_target_box_ = cv::Rect2f(
          best->center.x - width / 2.0F, best->center.y - width * 0.375F,
          width, width * 0.75F);
        roi_miss_streak_ = 0;
      } else {
        ++roi_miss_streak_;
      }
    } else {
      ++roi_miss_streak_;
    }
  }
  if (debug_info_.roi_active) debug_info_.applied_roi = applied_roi_;
  debug_info_.roi_miss_streak_after = roi_miss_streak_;
  return armors;
}

std::list<Armor> YOLOV8::infer_and_parse(
  const cv::Mat & crop_img, const cv::Mat & raw_img, int frame_count)
{
  auto x_scale = static_cast<double>(kInputSize) / crop_img.rows;
  auto y_scale = static_cast<double>(kInputSize) / crop_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(crop_img.rows * scale);
  auto w = static_cast<int>(crop_img.cols * scale);

  auto input = cv::Mat(kInputSize, kInputSize, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(crop_img, input(roi), {w, h});
  ov::Tensor input_tensor(ov::element::u8, {1, kInputSize, kInputSize, 3}, input.data);

  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  if (output_shape.size() != 3 || output_shape[0] != 1 || output_shape[1] != 14) {
    throw std::runtime_error("YOLOv8 output must have shape [1,14,N]");
  }
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLOV8::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // for each row: xywh + classess
  cv::transpose(output, output);

  std::vector<int> ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    auto xywh = output.row(r).colRange(0, 4);
    auto scores = output.row(r).colRange(4, 4 + class_num_);
    auto one_key_points = output.row(r).colRange(4 + class_num_, 14);

    std::vector<cv::Point2f> armor_key_points;

    double score;
    cv::Point max_point;
    cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

    if (!std::isfinite(score) || score < score_threshold_) continue;

    auto x = xywh.at<float>(0);
    auto y = xywh.at<float>(1);
    auto w = xywh.at<float>(2);
    auto h = xywh.at<float>(3);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w) ||
        !std::isfinite(h) || w <= 0.0F || h <= 0.0F) continue;
    auto left = static_cast<int>((x - 0.5 * w) / scale);
    auto top = static_cast<int>((y - 0.5 * h) / scale);
    auto width = static_cast<int>(w / scale);
    auto height = static_cast<int>(h / scale);

    bool valid_points = true;
    for (int i = 0; i < 4; i++) {
      float x = one_key_points.at<float>(0, i * 2 + 0) / scale;
      float y = one_key_points.at<float>(0, i * 2 + 1) / scale;
      if (!std::isfinite(x) || !std::isfinite(y)) {
        valid_points = false;
        break;
      }
      cv::Point2f kp = {x, y};
      armor_key_points.push_back(kp);
    }
    if (!valid_points || width <= 0 || height <= 0) continue;
    ids.emplace_back(max_point.x);
    confidences.emplace_back(score);
    boxes.emplace_back(left, top, width, height);
    armors_key_points.emplace_back(armor_key_points);
  }
  debug_info_.score_candidates = static_cast<int>(boxes.size());

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);
  debug_info_.nms_candidates = static_cast<int>(indices.size());
  debug_info_.classifier_rejected = 0;

  std::list<Armor> armors;
  for (const auto & i : indices) {
    sort_keypoints(armors_key_points[i]);
    if (use_roi_ || dynamic_roi_) {
      armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  for (auto it = armors.begin(); it != armors.end();) {
    it->pattern = get_pattern(bgr_img, *it);
    classifier_.classify(*it);

    if (!check_name(*it)) {
      ++debug_info_.classifier_rejected;
      it = armors.erase(it);
      continue;
    }

    it->type = get_type(*it);
    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

bool YOLOV8::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于分类器的迭代
  // if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool YOLOV8::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  // 保存异常的图案，用于分类器的迭代
  // if (!name_ok) save(armor);

  return name_ok;
}

ArmorType YOLOV8::get_type(const Armor & armor)
{
  // 英雄、基地只能是大装甲板
  if (armor.name == ArmorName::one || armor.name == ArmorName::base) {
    return ArmorType::big;
  }

  // 工程、哨兵、前哨站只能是小装甲板
  if (
    armor.name == ArmorName::two || armor.name == ArmorName::sentry ||
    armor.name == ArmorName::outpost) {
    return ArmorType::small;
  }

  // 步兵假设为小装甲板
  return ArmorType::small;
}

cv::Point2f YOLOV8::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

cv::Mat YOLOV8::get_pattern(const cv::Mat & bgr_img, const Armor & armor) const
{
  // 延长灯条获得装甲板角点
  // 1.125 = 0.5 * armor_height / lightbar_length = 0.5 * 126mm / 56mm
  auto tl = (armor.points[0] + armor.points[3]) / 2 - (armor.points[3] - armor.points[0]) * 1.125;
  auto bl = (armor.points[0] + armor.points[3]) / 2 + (armor.points[3] - armor.points[0]) * 1.125;
  auto tr = (armor.points[2] + armor.points[1]) / 2 - (armor.points[2] - armor.points[1]) * 1.125;
  auto br = (armor.points[2] + armor.points[1]) / 2 + (armor.points[2] - armor.points[1]) * 1.125;

  auto roi_left = std::max<int>(std::min(tl.x, bl.x), 0);
  auto roi_top = std::max<int>(std::min(tl.y, tr.y), 0);
  auto roi_right = std::min<int>(std::max(tr.x, br.x), bgr_img.cols);
  auto roi_bottom = std::min<int>(std::max(bl.y, br.y), bgr_img.rows);
  auto roi_tl = cv::Point(roi_left, roi_top);
  auto roi_br = cv::Point(roi_right, roi_bottom);
  auto roi = cv::Rect(roi_tl, roi_br);

  // 检查ROI是否有效
  if (roi_left < 0 || roi_top < 0 || roi_right <= roi_left || roi_bottom <= roi_top) {
    // std::cerr << "Invalid ROI: " << roi << std::endl;
    return cv::Mat();  // 返回一个空的Mat对象
  }

  // 检查ROI是否超出图像边界
  if (roi_right > bgr_img.cols || roi_bottom > bgr_img.rows) {
    // std::cerr << "ROI out of image bounds: " << roi << " Image size: " << bgr_img.size()
    //           << std::endl;
    return cv::Mat();  // 返回一个空的Mat对象
  }

  return bgr_img(roi);
}

void YOLOV8::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, armor.pattern);
}

void YOLOV8::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {}", armor.confidence, ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_ || dynamic_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, applied_roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
  cv::imshow("detection", detection);
}

void YOLOV8::sort_keypoints(std::vector<cv::Point2f> & keypoints)
{
  if (keypoints.size() != 4) {
    std::cout << "beyond 4!!" << std::endl;
    return;
  }

  std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
  std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

  std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });

  std::sort(
    bottom_points.begin(), bottom_points.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });

  keypoints[0] = top_points[0];     // top-left
  keypoints[1] = top_points[1];     // top-right
  keypoints[2] = bottom_points[1];  // bottom-right
  keypoints[3] = bottom_points[0];  // bottom-left
}

std::list<Armor> YOLOV8::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // 旧异步检测入口始终处理整帧；清除静态或动态 ROI 留下的裁剪原点。
  offset_ = {};
  applied_roi_ = cv::Rect(0, 0, bgr_img.cols, bgr_img.rows);
  debug_info_ = {};
  debug_info_.model_name = "yolov8";
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
