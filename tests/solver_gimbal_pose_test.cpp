#include <Eigen/Geometry>

#include <iostream>
#include <stdexcept>

#include "tasks/auto_aim/solver.hpp"

int main(int argc, char ** argv)
{
    if (argc != 2) return 2;

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

    std::cout << "solver_gimbal_pose_test passed\n";
}
