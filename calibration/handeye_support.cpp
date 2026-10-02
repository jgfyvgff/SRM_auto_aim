#include "calibration/handeye_support.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <string>

#include "calibration/gimbal_pose.hpp"

namespace calibration
{
namespace
{
void check_quaternion(const Eigen::Quaterniond & q)
{
    if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.0) > 0.01) {
        throw std::runtime_error("姿态四元数不是有效单位四元数");
    }
}

void check_rotation(const Eigen::Matrix3d & r)
{
    if (!r.allFinite() || std::abs(r.determinant() - 1.0) > 1e-5 ||
        (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() > 1e-5) {
        throw std::runtime_error("无效旋转矩阵");
    }
}
}

PoseRecord read_pose(std::istream & input)
{
    double w, x, y, z;
    if (!(input >> w >> x >> y >> z)) throw std::runtime_error("缺少有效 wxyz 姿态数据");
    PoseRecord pose{Eigen::Quaterniond(w, x, y, z), PoseFrame::ImuBody};
    check_quaternion(pose.q);
    pose.q.normalize();
    std::string frame;
    if (input >> frame) {
        if (frame == "gimbal") pose.frame = PoseFrame::Gimbal;
        else if (frame != "imubody") throw std::runtime_error("未知姿态坐标系: " + frame);
    }
    return pose;
}

void write_pose(std::ostream & output, const PoseRecord & pose)
{
    check_quaternion(pose.q);
    const auto q = pose.q.normalized();
    output << std::setprecision(17) << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z()
           << '\n' << (pose.frame == PoseFrame::Gimbal ? "gimbal" : "imubody") << '\n';
    if (!output) throw std::runtime_error("无法写入姿态文件");
}

Eigen::Matrix3d gimbal_to_world(const PoseRecord & pose, const Eigen::Matrix3d & gimbal_to_imu)
{
    check_quaternion(pose.q);
    const Eigen::Matrix3d rotation = pose.q.normalized().toRotationMatrix();
    if (pose.frame == PoseFrame::Gimbal) return rotation;
    check_rotation(gimbal_to_imu);
    return gimbal_to_imu.transpose() * rotation * gimbal_to_imu;
}

HandeyeResult solve_handeye(
    const std::vector<HandeyeObservation> & observations, HandeyePolicy policy)
{
    if (policy.min_samples < 3 || !std::isfinite(policy.min_rotation_deg) ||
        policy.min_rotation_deg <= 0.0 || policy.min_rotation_deg >= 180.0 ||
        !std::isfinite(policy.min_axis_ratio) || policy.min_axis_ratio <= 0.0 ||
        policy.min_axis_ratio >= 1.0) {
        throw std::invalid_argument("手眼标定样本数/旋转激励门限非法");
    }
    if (observations.size() < policy.min_samples) throw std::runtime_error("有效手眼样本不足");

    std::vector<cv::Mat> gimbal_rotations, gimbal_translations, board_rotations, board_translations;
    Eigen::MatrixXd axes(observations.size() - 1, 3);
    double max_angle = 0.0;
    for (std::size_t i = 0; i < observations.size(); ++i) {
        const auto & observation = observations[i];
        check_rotation(observation.gimbal_to_world);
        check_rotation(observation.board_to_camera);
        if (!observation.board_in_camera_mm.allFinite()) throw std::runtime_error("PnP 平移非法");
        cv::Mat rg, rb, tb;
        cv::eigen2cv(observation.gimbal_to_world, rg);
        cv::eigen2cv(observation.board_to_camera, rb);
        cv::eigen2cv(observation.board_in_camera_mm, tb);
        gimbal_rotations.push_back(rg);
        // 当前只有姿态反馈，假定云台原点固定；这不能补偿实际的平移和轴间偏置。
        gimbal_translations.push_back(cv::Mat::zeros(3, 1, CV_64F));
        board_rotations.push_back(rb);
        board_translations.push_back(tb);
        if (i > 0) {
            const Eigen::AngleAxisd motion(
                observations.front().gimbal_to_world.transpose() * observation.gimbal_to_world);
            axes.row(i - 1) = (motion.angle() * motion.axis()).transpose();
            max_angle = std::max(max_angle, motion.angle());
        }
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(axes);
    const auto values = svd.singularValues();
    const double axis_ratio = values[0] > 1e-12 ? values[1] / values[0] : 0.0;
    if (max_angle / kRadiansPerDegree < policy.min_rotation_deg || axis_ratio < policy.min_axis_ratio) {
        throw std::runtime_error("姿态变化不足或近似单轴：请固定标定板，分别改变云台 yaw 和 pitch");
    }

    cv::Mat rc, tc;
    // Park 方法直接求旋转矩阵，也覆盖相机倒装等大安装角情形。
    cv::calibrateHandEye(gimbal_rotations, gimbal_translations, board_rotations,
        board_translations, rc, tc, cv::CALIB_HAND_EYE_PARK);
    if (rc.empty() || tc.empty() || !cv::checkRange(rc) || !cv::checkRange(tc)) {
        throw std::runtime_error("手眼求解失败，外参含无效数值");
    }
    HandeyeResult result;
    cv::cv2eigen(rc, result.camera_to_gimbal);
    cv::cv2eigen(tc, result.camera_in_gimbal_m);
    result.camera_in_gimbal_m /= 1000.0;  // mm to m，仅在输出边界转换一次。
    result.axis_ratio = axis_ratio;
    check_rotation(result.camera_to_gimbal);

    // T_world_gimbal * T_gimbal_camera * T_camera_board 应为常量；报告训练样本闭环离散度。
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Quaterniond> rotations;
    Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();
    Eigen::Matrix4d quaternion_sum = Eigen::Matrix4d::Zero();
    for (const auto & observation : observations) {
        const Eigen::Vector3d position = observation.gimbal_to_world * (
            result.camera_to_gimbal * observation.board_in_camera_mm / 1000.0 + result.camera_in_gimbal_m);
        const Eigen::Quaterniond rotation(
            observation.gimbal_to_world * result.camera_to_gimbal * observation.board_to_camera);
        positions.push_back(position);
        rotations.push_back(rotation);
        mean_position += position;
        quaternion_sum += rotation.coeffs() * rotation.coeffs().transpose();
    }
    mean_position /= observations.size();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> mean_solver(quaternion_sum);
    if (mean_solver.info() != Eigen::Success) throw std::runtime_error("闭环姿态均值计算失败");
    Eigen::Quaterniond mean_rotation;
    mean_rotation.coeffs() = mean_solver.eigenvectors().col(3);
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const double position_error = (positions[i] - mean_position).norm();
        const double angle_error = rotations[i].angularDistance(mean_rotation) / kRadiansPerDegree;
        result.board_position_rms_m += position_error * position_error;
        result.board_rotation_rms_deg += angle_error * angle_error;
        result.board_position_max_m = std::max(result.board_position_max_m, position_error);
        result.board_rotation_max_deg = std::max(result.board_rotation_max_deg, angle_error);
    }
    result.board_position_rms_m = std::sqrt(result.board_position_rms_m / observations.size());
    result.board_rotation_rms_deg = std::sqrt(result.board_rotation_rms_deg / observations.size());
    return result;
}
}  // namespace calibration
