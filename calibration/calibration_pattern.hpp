#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace calibration
{

// 标定板检测类型。配置文件缺少 pattern_type 时默认使用原有圆点阵，保证旧配置仍可用。
enum class PatternType
{
  kCircles,
  kChessboard
};

// 标定板的几何参数。size 表示检测到的点阵尺寸，而不是棋盘格方格数量。
struct PatternSpec
{
  PatternType type;
  cv::Size size;
  double point_spacing_mm;
};

inline const char * pattern_type_name(PatternType type)
{
  switch (type) {
    case PatternType::kCircles:
      return "circles";
    case PatternType::kChessboard:
      return "chessboard";
  }
  return "unknown";
}

inline PatternSpec load_pattern_spec(const YAML::Node & yaml)
{
  const auto pattern_type = yaml["pattern_type"] ? yaml["pattern_type"].as<std::string>() : "circles";
  PatternType type;
  if (pattern_type == "circles") {
    type = PatternType::kCircles;
  } else if (pattern_type == "chessboard") {
    type = PatternType::kChessboard;
  } else {
    throw std::invalid_argument(
      "pattern_type must be either 'circles' or 'chessboard', got: " + pattern_type);
  }

  if (!yaml["pattern_cols"] || !yaml["pattern_rows"] || !yaml["center_distance_mm"]) {
    throw std::invalid_argument(
      "calibration pattern requires pattern_cols, pattern_rows and center_distance_mm");
  }

  const auto pattern_cols = yaml["pattern_cols"].as<int>();
  const auto pattern_rows = yaml["pattern_rows"].as<int>();
  const auto point_spacing_mm = yaml["center_distance_mm"].as<double>();
  if (pattern_cols <= 0 || pattern_rows <= 0) {
    throw std::invalid_argument("pattern_cols and pattern_rows must be positive");
  }
  if (!std::isfinite(point_spacing_mm) || point_spacing_mm <= 0.0) {
    throw std::invalid_argument("center_distance_mm must be a positive finite number");
  }

  return {type, cv::Size(pattern_cols, pattern_rows), point_spacing_mm};
}

// 统一处理两种标定板，避免采集、内参和手眼标定使用不同的检测分支。
inline bool detect_pattern(
  const cv::Mat & image, const PatternSpec & pattern, std::vector<cv::Point2f> & points)
{
  points.clear();
  if (image.empty()) {
    return false;
  }

  if (pattern.type == PatternType::kCircles) {
    return cv::findCirclesGrid(
      image, pattern.size, points, cv::CALIB_CB_SYMMETRIC_GRID);
  }

  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  }

  const auto flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  const auto found = cv::findChessboardCorners(gray, pattern.size, points, flags);
  if (found) {
    // 棋盘格角点需要亚像素优化；圆点阵分支保持 OpenCV 原有输出行为。
    cv::cornerSubPix(
      gray, points, cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01));
  }
  return found;
}

inline void draw_pattern(
  cv::Mat & image, const PatternSpec & pattern, const std::vector<cv::Point2f> & points,
  bool found)
{
  cv::drawChessboardCorners(image, pattern.size, points, found);
}

}  // namespace calibration
