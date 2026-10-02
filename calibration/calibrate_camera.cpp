#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <opencv2/opencv.hpp>

#include "calibration/calibration_pattern.hpp"
#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }"
  "{min-samples m  | 12                       | 最少有效样本数   }"
  "{output o       |                          | 输出YAML路径，留空则仅打印 }"
  "{show           | 1                        | 是否逐帧显示检测结果 }";

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      centers_3d.push_back({j * center_distance, i * center_distance, 0});

  return centers_3d;
}

void load(
  const std::string & input_folder, const std::string & config_path, cv::Size & img_size,
  std::vector<std::vector<cv::Point3f>> & obj_points,
  std::vector<std::vector<cv::Point2f>> & img_points, bool show)
{
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  const auto pattern = calibration::load_pattern_spec(yaml);

  for (int i = 1; true; i++) {
    // 读取图片
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;

    // 所有样本必须来自同一分辨率，否则内参和主点没有统一物理意义。
    if (img_size.empty())
      img_size = img.size();
    else if (img.size() != img_size)
      throw std::runtime_error(
        fmt::format(
          "图像尺寸不一致：{} 为 {}x{}，期望 {}x{}", img_path, img.cols, img.rows, img_size.width,
          img_size.height));

    // 识别标定板
    std::vector<cv::Point2f> centers_2d;
    auto success = calibration::detect_pattern(img, pattern, centers_2d);

    // 显示识别结果
    auto drawing = img.clone();
    calibration::draw_pattern(drawing, pattern, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 缩小图片尺寸便于显示完全
    if (show) {
      cv::imshow("Press any to continue", drawing);
      cv::waitKey(0);
    }

    // 输出识别结果
    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    // 记录所需的数据
    img_points.emplace_back(centers_2d);
    obj_points.emplace_back(
      centers_3d(pattern.size, static_cast<float>(pattern.point_spacing_mm)));
  }
}

std::string make_yaml(
  const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs, cv::Size image_size,
  size_t sample_count, double calibration_rms, double mean_error, double p95_error,
  double max_error)
{
  YAML::Emitter result;
  std::vector<double> camera_matrix_data(
    camera_matrix.begin<double>(), camera_matrix.end<double>());
  std::vector<double> distort_coeffs_data(
    distort_coeffs.begin<double>(), distort_coeffs.end<double>());

  result << YAML::BeginMap;
  result << YAML::Comment(
    fmt::format(
      "标定样本: {}，RMS: {:.4f}px，逐帧平均误差: {:.4f}px，P95: {:.4f}px，最大: {:.4f}px",
      sample_count, calibration_rms, mean_error, p95_error, max_error));
  result << YAML::Key << "image_size";
  result << YAML::Value << YAML::Flow << std::vector<int>{image_size.width, image_size.height};
  result << YAML::Key << "calibration_rms";
  result << YAML::Value << calibration_rms;
  result << YAML::Key << "reprojection_error_mean";
  result << YAML::Value << mean_error;
  result << YAML::Key << "reprojection_error_p95";
  result << YAML::Value << p95_error;
  result << YAML::Key << "reprojection_error_max";
  result << YAML::Value << max_error;
  result << YAML::Key << "valid_samples";
  result << YAML::Value << sample_count;
  result << YAML::Key << "camera_matrix";
  result << YAML::Value << YAML::Flow << camera_matrix_data;
  result << YAML::Key << "distort_coeffs";
  result << YAML::Value << YAML::Flow << distort_coeffs_data;
  result << YAML::Newline;
  result << YAML::EndMap;

  return result.c_str();
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto min_samples = cli.get<int>("min-samples");
  auto output_path = cli.get<std::string>("output");
  auto show = cli.get<bool>("show");

  if (min_samples < 1)
    throw std::invalid_argument("min-samples 必须大于0");

  // 从输入文件夹中加载标定所需的数据
  cv::Size img_size;
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  load(input_folder, config_path, img_size, obj_points, img_points, show);

  if (obj_points.size() < static_cast<size_t>(min_samples))
    throw std::runtime_error(
      fmt::format(
        "有效标定样本只有 {} 张，少于要求的 {} 张；请继续采集或检查标定板检测",
        obj_points.size(), min_samples));

  // 相机标定
  cv::Mat camera_matrix, distort_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  auto criteria = cv::TermCriteria(
    cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100,
    DBL_EPSILON);  // 默认迭代次数(30)有时会导致结果发散，故设为100
  auto calibration_rms = cv::calibrateCamera(
    obj_points, img_points, img_size, camera_matrix, distort_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3,
    criteria);  // 由于视场角较小，不需要考虑k3

  // 逐帧计算误差，用于识别模糊、反光或姿态不充分的离群样本。
  double error_sum = 0;
  size_t total_points = 0;
  std::vector<double> frame_errors;
  for (size_t i = 0; i < obj_points.size(); i++) {
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      obj_points[i], rvecs[i], tvecs[i], camera_matrix, distort_coeffs, reprojected_points);

    double frame_squared_error = 0;
    total_points += reprojected_points.size();
    for (size_t j = 0; j < reprojected_points.size(); j++) {
      auto point_error = cv::norm(img_points[i][j] - reprojected_points[j]);
      error_sum += point_error;
      frame_squared_error += point_error * point_error;
    }
    frame_errors.push_back(
      std::sqrt(frame_squared_error / static_cast<double>(reprojected_points.size())));
  }
  std::sort(frame_errors.begin(), frame_errors.end());
  auto mean_error = error_sum / static_cast<double>(total_points);
  auto p95_error = frame_errors[static_cast<size_t>(0.95 * (frame_errors.size() - 1))];
  auto max_error = frame_errors.back();

  fmt::print(
    "标定完成：有效样本={}，RMS={:.4f}px，逐帧平均={:.4f}px，P95={:.4f}px，最大={:.4f}px\n",
    obj_points.size(), calibration_rms, mean_error, p95_error, max_error);

  auto yaml_text = make_yaml(
    camera_matrix, distort_coeffs, img_size, obj_points.size(), calibration_rms, mean_error,
    p95_error, max_error);

  if (output_path.empty()) {
    fmt::print("\n{}\n", yaml_text);
  } else {
    std::ofstream output(output_path);
    if (!output)
      throw std::runtime_error(fmt::format("无法写入标定结果: {}", output_path));
    output << yaml_text;
    fmt::print("标定结果已写入: {}\n", output_path);
  }
}
