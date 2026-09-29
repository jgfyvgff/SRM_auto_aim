#include <iostream>
#include <stdexcept>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "calibration/calibration_pattern.hpp"

namespace
{

bool test_config_parsing()
{
  const auto circles = calibration::load_pattern_spec(YAML::Load(
    "pattern_type: circles\npattern_cols: 10\npattern_rows: 7\ncenter_distance_mm: 40\n"));
  if (circles.type != calibration::PatternType::kCircles || circles.size != cv::Size(10, 7)) {
    return false;
  }

  const auto chessboard = calibration::load_pattern_spec(YAML::Load(
    "pattern_type: chessboard\npattern_cols: 6\npattern_rows: 5\ncenter_distance_mm: 30\n"));
  return chessboard.type == calibration::PatternType::kChessboard &&
         chessboard.size == cv::Size(6, 5) && chessboard.point_spacing_mm == 30.0;
}

bool test_chessboard_detection()
{
  constexpr int cell_size = 60;
  constexpr int squares_x = 7;
  constexpr int squares_y = 6;
  cv::Mat image(
    squares_y * cell_size + 120, squares_x * cell_size + 120, CV_8UC1, cv::Scalar(255));
  const cv::Point origin(60, 60);
  for (int row = 0; row < squares_y; ++row) {
    for (int col = 0; col < squares_x; ++col) {
      if ((row + col) % 2 == 0) {
        cv::rectangle(
          image, origin + cv::Point(col * cell_size, row * cell_size),
          origin + cv::Point((col + 1) * cell_size - 1, (row + 1) * cell_size - 1),
          cv::Scalar(0), cv::FILLED);
      }
    }
  }

  const calibration::PatternSpec pattern{
    calibration::PatternType::kChessboard, cv::Size(squares_x - 1, squares_y - 1), 30.0};
  std::vector<cv::Point2f> points;
  return calibration::detect_pattern(image, pattern, points) &&
         points.size() == static_cast<size_t>((squares_x - 1) * (squares_y - 1));
}

bool test_invalid_config()
{
  try {
    calibration::load_pattern_spec(YAML::Load(
      "pattern_type: unsupported\npattern_cols: 10\npattern_rows: 7\ncenter_distance_mm: 40\n"));
  } catch (const std::invalid_argument &) {
    return true;
  }
  return false;
}

}  // namespace

int main()
{
  if (!test_config_parsing() || !test_chessboard_detection() || !test_invalid_config()) {
    std::cerr << "calibration_pattern_test failed\n";
    return 1;
  }
  std::cout << "calibration_pattern_test passed\n";
  return 0;
}
