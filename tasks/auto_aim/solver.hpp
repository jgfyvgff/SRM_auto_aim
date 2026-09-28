#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <optional>

#include "armor.hpp"

namespace auto_aim
{
// 平面装甲板 IPPE 可能返回多个姿态候选；此结构只用于诊断候选分支。
struct PnpCandidateDebug
{
  double yaw_in_world = 0.0;
  double reprojection_error = 0.0;  // 四个角点平均像素误差
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();
};

class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  // predicted_armor 为空时保持初始化行为；非空时使用预测装甲板 yaw
  // 在 IPPE 多个平面姿态中选择连续分支。该参数只读，不转移所有权。
  void solve(
    Armor & armor,
    std::optional<Eigen::Vector4d> predicted_armor = std::nullopt) const;

  // 返回当前角点对应的全部 IPPE 候选，不改变 Armor 和 Tracker 状态。
  std::vector<PnpCandidateDebug> pnp_candidates(const Armor & armor) const;

  // 使用原始装甲板四点执行一次 IPPE PnP，并将同一姿态回投影到图像。
  // 该误差用于隔离检测点、装甲板尺寸和 PnP 点序问题。
  std::vector<cv::Point2f> reproject_pnp(const Armor & armor) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double oupost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;
  // yaw 优化相对原始 PnP yaw 的最大允许修正，单位为 rad。
  double max_yaw_optimization_correction_;

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP
