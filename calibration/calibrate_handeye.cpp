#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>

#include "calibration/calibration_pattern.hpp"
#include "calibration/handeye_support.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
    "{help h usage ? |                          | 输出命令行参数说明}"
    "{config-path c  | configs/calibration.yaml | 标定板配置文件路径 }"
    "{@input-folder  | assets/img_with_q        | 输入图像和姿态文件夹 }"
    "{intrinsics     |                          | 独立内参YAML，留空则从配置读取 }"
    "{min-samples    | 12                       | 最少有效样本数，至少3 }"
    "{min-rotation-deg | 5                      | 最小相对转角，度 }"
    "{min-axis-ratio | 0.05                     | 第二/第一旋转奇异值下限 }"
    "{show           | 1                        | 是否逐张显示并等待按键 }"
    "{output o       |                          | 外参输出YAML，留空仅打印 }";

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
    std::vector<cv::Point3f> centers_3d;
    for (int i = 0; i < pattern_size.height; i++)
        for (int j = 0; j < pattern_size.width; j++)
            centers_3d.push_back({j * center_distance, i * center_distance, 0});
    return centers_3d;
}

// 先检查文件类型，使参数写错时得到路径提示，避免把目录交给 YAML 产生 ios_failure。
YAML::Node load_yaml_file(const std::string & path)
{
    if (!std::filesystem::is_regular_file(path)) {
        throw std::runtime_error("YAML 路径不是普通文件: " + path + "；请使用 --config-path=文件路径");
    }
    return YAML::LoadFile(path);
}

std::vector<calibration::HandeyeObservation> load(
    const std::string & input_folder, const YAML::Node & yaml, const YAML::Node & intrinsics,
    std::vector<double> & R_gimbal2imubody_data, calibration::PoseFrame & pose_frame, bool show)
{
    // 读取yaml参数
    const auto pattern = calibration::load_pattern_spec(yaml);
    const auto camera_matrix_data = intrinsics["camera_matrix"].as<std::vector<double>>();
    const auto distort_coeffs_data = intrinsics["distort_coeffs"].as<std::vector<double>>();
    const auto finite_values = [](const auto & values) {
        return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
    };
    const auto distortion_count = distort_coeffs_data.size();
    if (camera_matrix_data.size() != 9 || !finite_values(camera_matrix_data) ||
        camera_matrix_data[0] <= 0.0 || camera_matrix_data[4] <= 0.0 ||
        std::abs(camera_matrix_data[6]) > 1e-10 || std::abs(camera_matrix_data[7]) > 1e-10 ||
        std::abs(camera_matrix_data[8] - 1.0) > 1e-10 || !finite_values(distort_coeffs_data) ||
        (distortion_count != 4 && distortion_count != 5 && distortion_count != 8 &&
         distortion_count != 12 && distortion_count != 14)) {
        throw std::runtime_error("内参数量、焦距、齐次末行或畸变参数无效");
    }
    const cv::Matx33d camera_matrix(camera_matrix_data.data());
    const cv::Mat distort_coeffs(distort_coeffs_data);
    cv::Size image_size;
    if (intrinsics["image_size"]) {
        const auto dimensions = intrinsics["image_size"].as<std::vector<int>>();
        if (dimensions.size() != 2 || dimensions[0] <= 0 || dimensions[1] <= 0) {
            throw std::runtime_error("内参文件 image_size 无效");
        }
        image_size = cv::Size(dimensions[0], dimensions[1]);
    }
    if (!std::filesystem::is_directory(input_folder)) throw std::runtime_error("输入目录不存在: " + input_folder);
    std::map<unsigned long long, std::filesystem::path> images;
    for (const auto & entry : std::filesystem::directory_iterator(input_folder)) {
        const auto stem = entry.path().stem().string();
        if (!entry.is_regular_file() || entry.path().extension() != ".jpg" || stem.empty() ||
            stem.find_first_not_of("0123456789") != std::string::npos) continue;
        if (!images.emplace(std::stoull(stem), entry.path()).second) {
            throw std::runtime_error("输入目录包含重复图像编号");
        }
    }
    if (images.empty()) throw std::runtime_error("目录内没有编号 JPG 样本");

    std::optional<calibration::PoseFrame> dataset_frame;
    Eigen::Matrix3d R_gimbal2imubody = Eigen::Matrix3d::Identity();
    const auto object_points = centers_3d(pattern.size, static_cast<float>(pattern.point_spacing_mm));
    std::vector<calibration::HandeyeObservation> observations;
    // 按实际编号遍历，允许删除坏照片后存在编号空缺，不会在缺号处提前结束。
    for (const auto & [index, image_path] : images) {
        // 读取图片和对应四元数
        auto q_path = image_path;
        q_path.replace_extension(".txt");
        std::ifstream q_input(q_path);
        if (!q_input) throw std::runtime_error("缺少配对姿态，内参照片不能直接用于手眼标定: " + q_path.string());
        const auto pose = calibration::read_pose(q_input);
        if (dataset_frame && *dataset_frame != pose.frame) throw std::runtime_error("数据集混用了 CAN/串口坐标系");
        if (!dataset_frame) {
            dataset_frame = pose.frame;
            pose_frame = pose.frame;
            if (pose.frame == calibration::PoseFrame::ImuBody) {
                R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
                if (R_gimbal2imubody_data.size() != 9) throw std::runtime_error("R_gimbal2imubody 必须有9个元素");
                R_gimbal2imubody = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                    R_gimbal2imubody_data.data());
            } else {
                // 串口已经给出云台姿态，输出单位阵说明无需再次进行 IMU 安装轴转换。
                R_gimbal2imubody_data = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            }
        }
        const auto img = cv::imread(image_path.string());
        if (img.empty()) throw std::runtime_error("无法读取样本图片: " + image_path.string());
        if (image_size.empty()) image_size = img.size();
        if (img.size() != image_size) throw std::runtime_error("样本分辨率与内参或其他样本不一致: " + image_path.string());

        // 计算云台的欧拉角
        const Eigen::Matrix3d R_gimbal2world = calibration::gimbal_to_world(pose, R_gimbal2imubody);
        const Eigen::Vector3d ypr = tools::eulers(R_gimbal2world, 2, 1, 0) * 180.0 / CV_PI;
        // 识别标定板
        std::vector<cv::Point2f> centers_2d;
        const bool success = calibration::detect_pattern(img, pattern, centers_2d);
        // 显示识别结果
        if (show) {
            auto drawing = img.clone();
            // 在图片上显示云台的欧拉角，用来检验R_gimbal2imubody是否正确
            // 串口模式下此处显示直接收到的云台姿态，不应用 R_gimbal2imubody。
            tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
            tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
            tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});
            calibration::draw_pattern(drawing, pattern, centers_2d, success);
            cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
            cv::imshow("Press any to continue", drawing);
            cv::waitKey(0);
        }
        // 输出识别结果
        fmt::print("[{}] {}\n", success ? "success" : "failure", image_path.string());
        if (!success) continue;

        // 计算所需的数据
        cv::Mat rvec, tvec, rotation;
        if (!cv::solvePnP(object_points, centers_2d, camera_matrix, distort_coeffs,
                rvec, tvec, false, cv::SOLVEPNP_IPPE) || !cv::checkRange(rvec) ||
            !cv::checkRange(tvec) || tvec.at<double>(2) <= 0.0) {
            fmt::print("[跳过] PnP 无有效正深度解: {}\n", image_path.string());
            continue;
        }
        cv::Rodrigues(rvec, rotation);
        calibration::HandeyeObservation observation;
        observation.gimbal_to_world = R_gimbal2world;
        cv::cv2eigen(rotation, observation.board_to_camera);
        cv::cv2eigen(tvec, observation.board_in_camera_mm);
        // 记录所需的数据
        observations.push_back(observation);
    }
    return observations;
}

std::string make_yaml(const std::vector<double> & R_gimbal2imubody_data,
                      const calibration::HandeyeResult & result, calibration::PoseFrame frame,
                      std::size_t sample_count)
{
    YAML::Emitter output;
    std::vector<double> rotation;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) rotation.push_back(result.camera_to_gimbal(row, col));
    const std::vector<double> translation(result.camera_in_gimbal_m.data(), result.camera_in_gimbal_m.data() + 3);
    // 计算相机同理想情况的偏角
    const Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
    const Eigen::Vector3d ypr = tools::eulers(
        Eigen::Matrix3d(R_gimbal2ideal * result.camera_to_gimbal), 1, 0, 2) * 180.0 / CV_PI;
    output << YAML::BeginMap;
    output << YAML::Comment("固定云台原点模型；t_camera2gimbal 单位 m，矩阵按行排列");
    output << YAML::Key << "pose_frame" << YAML::Value << (frame == calibration::PoseFrame::Gimbal ? "gimbal" : "imubody");
    output << YAML::Key << "R_gimbal2imubody" << YAML::Value << YAML::Flow << R_gimbal2imubody_data;
    output << YAML::Comment(fmt::format("相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", ypr[0], ypr[1], ypr[2]));
    output << YAML::Key << "R_camera2gimbal" << YAML::Value << YAML::Flow << rotation;
    output << YAML::Key << "t_camera2gimbal" << YAML::Value << YAML::Flow << translation;
    output << YAML::Key << "valid_samples" << YAML::Value << sample_count;
    output << YAML::Key << "rotation_axis_ratio" << YAML::Value << result.axis_ratio;
    output << YAML::Key << "board_position_rms_m" << YAML::Value << result.board_position_rms_m;
    output << YAML::Key << "board_position_max_m" << YAML::Value << result.board_position_max_m;
    output << YAML::Key << "board_rotation_rms_deg" << YAML::Value << result.board_rotation_rms_deg;
    output << YAML::Key << "board_rotation_max_deg" << YAML::Value << result.board_rotation_max_deg;
    output << YAML::EndMap;
    return output.c_str();
}

int main(int argc, char * argv[])
{
    try {
        // 读取命令行参数
        cv::CommandLineParser cli(argc, argv, keys);
        if (cli.has("help")) { cli.printMessage(); return 0; }
        const auto input_folder = cli.get<std::string>(0);
        const auto config_path = cli.get<std::string>("config-path");
        auto intrinsics_path = cli.get<std::string>("intrinsics");
        const auto output_path = cli.get<std::string>("output");
        const auto show = cli.get<bool>("show");
        const auto min_samples = cli.get<int>("min-samples");
        calibration::HandeyePolicy policy;
        policy.min_rotation_deg = cli.get<double>("min-rotation-deg");
        policy.min_axis_ratio = cli.get<double>("min-axis-ratio");
        if (!cli.check()) { cli.printErrors(); return 1; }
        if (min_samples < 3) throw std::invalid_argument("min-samples 必须至少为3");
        policy.min_samples = static_cast<std::size_t>(min_samples);
        if (intrinsics_path.empty()) intrinsics_path = config_path;
        if (!output_path.empty() && std::filesystem::exists(output_path)) {
            throw std::runtime_error("输出文件已存在，请使用新文件名以保留已有标定: " + output_path);
        }

        // 从输入文件夹中加载标定所需的数据
        const auto yaml = load_yaml_file(config_path);
        const auto intrinsics = load_yaml_file(intrinsics_path);
        std::vector<double> R_gimbal2imubody_data;
        calibration::PoseFrame pose_frame = calibration::PoseFrame::ImuBody;
        const auto observations = load(input_folder, yaml, intrinsics, R_gimbal2imubody_data, pose_frame, show);
        fmt::print("有效样本={}，模型前提：标定板、底盘及云台参考原点固定，仅改变云台朝向。\n", observations.size());
        // 手眼标定
        const auto result = calibration::solve_handeye(observations, policy);
        fmt::print("闭环位置 RMS={:.3f}mm，最大={:.3f}mm；姿态 RMS={:.4f}deg，最大={:.4f}deg\n",
            result.board_position_rms_m * 1000.0, result.board_position_max_m * 1000.0,
            result.board_rotation_rms_deg, result.board_rotation_max_deg);
        fmt::print("以上是样本内部一致性，不代表已经完成实车精度验收。\n");
        // 输出yaml
        const auto text = make_yaml(R_gimbal2imubody_data, result, pose_frame, observations.size());
        fmt::print("\n{}\n", text);
        if (!output_path.empty()) {
            std::ofstream output(output_path);
            output << text << '\n';
            output.close();
            if (!output) throw std::runtime_error("无法写入外参结果: " + output_path);
            fmt::print("外参已保存到: {}\n", output_path);
        }
        return 0;
    } catch (const std::exception & error) {
        fmt::print(stderr, "手眼标定失败: {}\n", error.what());
        return 1;
    }
}
