#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <opencv2/opencv.hpp>

#include "calibration/calibration_pattern.hpp"
#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }"
  "{camera-only     | 0                        | 仅采集图像，不初始化CAN/IMU }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder,
  const calibration::PatternSpec & pattern, bool camera_only)
{
  std::unique_ptr<io::CBoard> cboard;
  if (!camera_only)
    cboard = std::make_unique<io::CBoard>(config_path);

  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);
    if (img.empty()) {
      tools::logger()->warn("相机未返回有效图像，跳过本帧");
      continue;
    }

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (!camera_only)
      q = cboard->imu_at(timestamp);

    auto img_with_ypr = img.clone();
    if (!camera_only) {
      // 手眼标定采集时显示 IMU 姿态，用于检查坐标轴方向和零漂。
      Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
      tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
      tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
      tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});
    } else {
      tools::draw_text(img_with_ypr, "Camera only", {40, 40}, {0, 255, 0});
    }

    std::vector<cv::Point2f> centers_2d;
    auto success = calibration::detect_pattern(img, pattern, centers_2d);
    calibration::draw_pattern(img_with_ypr, pattern, centers_2d, success);
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸

    // 按“s”保存图片和对应四元数，按“q”退出程序
    cv::imshow("Press s to save, q to quit", img_with_ypr);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    if (!success) {
      tools::logger()->warn("当前帧未检测到完整标定板，不保存");
      continue;
    }

    // 保存图片和四元数
    auto next_count = count + 1;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, next_count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, next_count);
    if (!cv::imwrite(img_path, img)) {
      tools::logger()->error("无法保存标定图像: {}", img_path);
      continue;
    }
    if (!camera_only)
      write_q(q_path, q);
    count = next_count;
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }

  // 离开该作用域时，camera和cboard会自动关闭
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");
  auto camera_only = cli.get<bool>("camera-only");
  auto calibration_yaml = YAML::LoadFile(config_path);
  const auto pattern = calibration::load_pattern_spec(calibration_yaml);

  // 采集程序可能首次运行在全新的工作区，递归创建目录避免保存时失败。
  std::error_code error;
  std::filesystem::create_directories(output_folder, error);
  if (error) {
    throw std::runtime_error(
      fmt::format("无法创建标定输出目录 {}: {}", output_folder, error.message()));
  }

  tools::logger()->info(
    "标定板模式: {}, 尺寸: {}列{}行", calibration::pattern_type_name(pattern.type),
    pattern.size.width, pattern.size.height);
  // 主循环，保存图片和对应四元数
  capture_loop(config_path, output_folder, pattern, camera_only);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
