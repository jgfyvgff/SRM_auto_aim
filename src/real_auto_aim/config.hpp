#pragma once

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace real_auto_aim
{
struct RealConfig
{
    int image_width;
    int image_height;
    std::chrono::milliseconds max_image_age;
};

// 先于硬件初始化校验真机标定，避免 Solver 对长度错误的数组直接建矩阵。
inline RealConfig validate_real_config(const YAML::Node & yaml)
{
    if (yaml["pose_frame"].as<std::string>() != "gimbal") {
        throw std::invalid_argument("Real pose_frame must be gimbal");
    }
    if (yaml["camera_name"].as<std::string>() != "hikrobot" ||
        yaml["yolo_name"].as<std::string>() != "yolov5") {
        throw std::invalid_argument("Real entry requires hikrobot camera and yolov5");
    }
    const auto color = yaml["enemy_color"].as<std::string>();
    if (color != "red" && color != "blue") {
        throw std::invalid_argument("Invalid enemy_color");
    }
    // 真机入口必须显式关闭不适用于倒装相机的固定俯仰 yaw 优化。
    if (!yaml["yaw_optimization_enabled"].IsDefined() ||
        yaml["yaw_optimization_enabled"].as<bool>()) {
        throw std::invalid_argument("Real auto-aim requires yaw optimization disabled");
    }

    const auto image_size = yaml["image_size"].as<std::vector<int>>();
    if (image_size.size() != 2 || image_size[0] <= 0 || image_size[1] <= 0) {
        throw std::invalid_argument("Invalid calibrated image_size");
    }
    const int max_image_age_ms = yaml["max_image_age_ms"].as<int>();
    if (max_image_age_ms <= 0 || max_image_age_ms > 1000) {
        throw std::invalid_argument("Invalid max_image_age_ms");
    }

    const auto check_vector = [&yaml](const char * key, std::size_t size) {
        const auto data = yaml[key].as<std::vector<double>>();
        if (data.size() != size) {
            throw std::invalid_argument(std::string(key) + " has invalid length");
        }
        for (double value : data) {
            if (!std::isfinite(value)) {
                throw std::invalid_argument(std::string(key) + " contains a non-finite value");
            }
        }
        return data;
    };
    const auto camera_matrix = check_vector("camera_matrix", 9);
    check_vector("distort_coeffs", 5);
    check_vector("R_gimbal2imubody", 9);
    const auto camera_to_gimbal = check_vector("R_camera2gimbal", 9);
    check_vector("t_camera2gimbal", 3);
    if (camera_matrix[0] <= 0.0 || camera_matrix[4] <= 0.0 ||
        std::abs(camera_matrix[8] - 1.0) > 1e-6) {
        throw std::invalid_argument("Invalid camera_matrix focal length or scale");
    }
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(
        camera_to_gimbal.data());
    if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 0.01 ||
        std::abs(rotation.determinant() - 1.0) > 0.01) {
        throw std::invalid_argument("R_camera2gimbal is not a proper rotation");
    }

    const double exposure_ms = yaml["exposure_ms"].as<double>();
    const double gain = yaml["gain"].as<double>();
    if (!std::isfinite(exposure_ms) || exposure_ms <= 0.0 ||
        !std::isfinite(gain) || gain < 0.0) {
        throw std::invalid_argument("Invalid camera exposure or gain");
    }
    // 真机规划器的约束在设备启动前检查，避免非有限权重进入求解器。
    const double planner_debug_speed = yaml["planner_debug_bullet_speed_mps"].as<double>();
    const double fire_thresh = yaml["fire_thresh"].as<double>();
    const double max_yaw_acc = yaml["max_yaw_acc"].as<double>();
    const double max_pitch_acc = yaml["max_pitch_acc"].as<double>();
    if (!std::isfinite(planner_debug_speed) || planner_debug_speed < 10.0 ||
        planner_debug_speed > 25.0 || !std::isfinite(fire_thresh) ||
        fire_thresh <= 0.0 || !std::isfinite(max_yaw_acc) || max_yaw_acc <= 0.0 ||
        !std::isfinite(max_pitch_acc) || max_pitch_acc <= 0.0) {
        throw std::invalid_argument("Invalid real Planner parameters");
    }
    for (const char * key : {"Q_yaw", "Q_pitch", "R_yaw", "R_pitch"}) {
        const std::size_t size = key[0] == 'Q' ? 2 : 1;
        const auto weights = check_vector(key, size);
        for (double weight : weights) {
            if (weight < 0.0) {
                throw std::invalid_argument(std::string(key) + " must be non-negative");
            }
        }
    }
    return {image_size[0], image_size[1],
            std::chrono::milliseconds(max_image_age_ms)};
}
}  // namespace real_auto_aim
