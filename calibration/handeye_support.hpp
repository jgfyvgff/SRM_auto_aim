#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <istream>
#include <ostream>
#include <vector>

namespace calibration
{
// 老 CAN 数据保存 IMU body→absolute；串口已保存 gimbal→world，不能再次换 IMU 安装轴。
enum class PoseFrame { ImuBody, Gimbal };
struct PoseRecord
{
    Eigen::Quaterniond q;
    PoseFrame frame = PoseFrame::ImuBody;
};

// 第一行兼容旧的 wxyz；第二行显式标记坐标系。缺少第二行只兼容旧 CAN 数据。
PoseRecord read_pose(std::istream & input);
void write_pose(std::ostream & output, const PoseRecord & pose);
Eigen::Matrix3d gimbal_to_world(const PoseRecord & pose, const Eigen::Matrix3d & gimbal_to_imu);

// 固定云台原点模型的一组观测。标定板必须静止；PnP 平移单位为 mm。
struct HandeyeObservation
{
    Eigen::Matrix3d gimbal_to_world;
    Eigen::Matrix3d board_to_camera;
    Eigen::Vector3d board_in_camera_mm;
};

struct HandeyePolicy
{
    std::size_t min_samples = 12;
    double min_rotation_deg = 5.0;
    double min_axis_ratio = 0.05;  // 相对旋转向量第二/第一奇异值，拒收近似单轴数据。
};

struct HandeyeResult
{
    Eigen::Matrix3d camera_to_gimbal;
    Eigen::Vector3d camera_in_gimbal_m;
    double axis_ratio = 0.0;
    double board_position_rms_m = 0.0;
    double board_position_max_m = 0.0;
    double board_rotation_rms_deg = 0.0;
    double board_rotation_max_deg = 0.0;
};

// 纯离线计算，不拥有相机/串口。返回相机→云台外参及固定标定板的闭环离散度。
// 固定原点是明确建模前提：若两旋转轴不共点，需另行提供云台平移运动学。
HandeyeResult solve_handeye(
    const std::vector<HandeyeObservation> & observations, HandeyePolicy policy = {});
}  // namespace calibration
