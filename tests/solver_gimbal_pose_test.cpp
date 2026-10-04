#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "tasks/auto_aim/solver.hpp"

int main(int argc, char ** argv)
{
    if (argc != 3) return 2;

    auto_aim::Solver solver(argv[1]);
    const Eigen::Quaterniond q(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));

    // 使用非单位阵 IMU 安装轴验证：串口入口只采用云台自身姿态。
    solver.set_gimbal_to_world(q);
    if ((solver.R_gimbal2world() - q.toRotationMatrix()).norm() > 1e-12) {
        throw std::runtime_error("serial gimbal pose was transformed twice");
    }

    solver.set_R_gimbal2world(q);
    if ((solver.R_gimbal2world() - q.toRotationMatrix()).norm() < 1e-3) {
        throw std::runtime_error("legacy IMU transform was bypassed");
    }

    auto_aim::Solver real_solver(argv[2]);
    const Eigen::Quaterniond real_pose(
        Eigen::AngleAxisd(-52.09 * CV_PI / 180.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-2.68 * CV_PI / 180.0, Eigen::Vector3d::UnitY()));
    real_solver.set_gimbal_to_world(real_pose);

    // 使用已有真机记录的倒装相机角点，测试不依赖相机、串口或实时检测。
    auto_aim::Armor armor(
        6, 0.95F, cv::Rect(564, 295, 164, 72),
        std::vector<cv::Point2f>{
            {565.9178F, 299.3274F}, {727.1277F, 300.8722F},
            {726.9770F, 366.4089F}, {564.7903F, 364.0601F}});
    real_solver.solve(armor);
    if (std::abs(armor.ypr_in_world[0] - armor.yaw_raw) > 1e-12 ||
        armor.yaw_raw < -1.3 || armor.yaw_raw > -0.5) {
        throw std::runtime_error("real inverted-camera yaw was overwritten");
    }

    Eigen::Vector4d prediction;
    prediction.head<3>() = armor.xyz_in_world;
    prediction[3] = armor.yaw_raw;
    real_solver.solve(armor, prediction);
    if (std::abs(armor.ypr_in_world[0] - armor.yaw_raw) > 1e-12) {
        throw std::runtime_error("predicted real yaw was overwritten");
    }

    std::cout << "solver_gimbal_pose_test passed\n";
}
