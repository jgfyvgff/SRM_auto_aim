#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
constexpr double DEFAULT_LIGHTBAR_LENGTH = 56e-3;  // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m

Solver::Solver(const std::string & config_path)
: R_gimbal2world_(Eigen::Matrix3d::Identity()), lightbar_length_(DEFAULT_LIGHTBAR_LENGTH)
{
  auto yaml = YAML::LoadFile(config_path);

  if (yaml["pnp_lightbar_length"].IsDefined()) {
    lightbar_length_ = yaml["pnp_lightbar_length"].as<double>();
  }
  if (
    !std::isfinite(lightbar_length_) || lightbar_length_ < 0.02 ||
    lightbar_length_ > 0.10)
  {
    throw std::runtime_error("Invalid pnp_lightbar_length configuration");
  }

  // 物点顺序必须与检测器输出的四个角点顺序一致；这里只参数化灯条长度，
  // 不改变装甲板宽度和点序，避免不同配置产生难以追踪的 PnP 分支差异。
  big_armor_points_ = {
    {0, BIG_ARMOR_WIDTH / 2, static_cast<float>(lightbar_length_ / 2)},
    {0, -BIG_ARMOR_WIDTH / 2, static_cast<float>(lightbar_length_ / 2)},
    {0, -BIG_ARMOR_WIDTH / 2, static_cast<float>(-lightbar_length_ / 2)},
    {0, BIG_ARMOR_WIDTH / 2, static_cast<float>(-lightbar_length_ / 2)}};

  small_armor_points_ = {
    {0, SMALL_ARMOR_WIDTH / 2, static_cast<float>(lightbar_length_ / 2)},
    {0, -SMALL_ARMOR_WIDTH / 2, static_cast<float>(lightbar_length_ / 2)},
    {0, -SMALL_ARMOR_WIDTH / 2, static_cast<float>(-lightbar_length_ / 2)},
    {0, SMALL_ARMOR_WIDTH / 2, static_cast<float>(-lightbar_length_ / 2)}};

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();//gimbal坐标系到imu坐标系的旋转矩阵
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();//camera坐标系到gimbal坐标系的旋转矩阵
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();//camera坐标系到gimbal坐标系的平移向量
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());//将旋转矩阵转换为Eigen矩阵
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());//将旋转矩阵转换为Eigen矩阵
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());//将平移向量转换为Eigen矩阵

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);//将Eigen矩阵转换为cv::Mat
  cv::eigen2cv(distort_coeffs, distort_coeffs_);//将Eigen矩阵转换为cv::Mat

  max_yaw_optimization_correction_ = yaml["max_yaw_optimization_correction"].IsDefined()
                                       ? yaml["max_yaw_optimization_correction"].as<double>()
                                       : std::numeric_limits<double>::infinity();
  if (
    !std::isinf(max_yaw_optimization_correction_) &&
    (!std::isfinite(max_yaw_optimization_correction_) ||
     max_yaw_optimization_correction_ <= 0.0 ||
     max_yaw_optimization_correction_ > CV_PI))
  {
    throw std::runtime_error("Invalid max_yaw_optimization_correction configuration");
  }
}

const std::vector<cv::Point3f> & Solver::armor_points(ArmorType type) const
{
  return type == ArmorType::big ? big_armor_points_ : small_armor_points_;
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();//将四元数转换为旋转矩阵，表示imu坐标系到imu绝对坐标系的旋转
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

std::vector<PnpCandidateDebug> Solver::pnp_candidates(const Armor & armor) const
{
  if (armor.points.size() != 4) {
    return {};
  }

  const auto & object_points = armor_points(armor.type);

  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  const bool solved = cv::solvePnPGeneric(
    object_points,
    armor.points,
    camera_matrix_,
    distort_coeffs_,
    rvecs,
    tvecs,
    false,
    cv::SOLVEPNP_IPPE);
  if (!solved || rvecs.empty() || rvecs.size() != tvecs.size()) {
    return {};
  }

  std::vector<PnpCandidateDebug> candidates;
  candidates.reserve(rvecs.size());
  for (std::size_t i = 0; i < rvecs.size(); ++i) {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvecs[i], rotation_matrix);

    Eigen::Matrix3d R_armor2camera;
    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(rotation_matrix, R_armor2camera);
    cv::cv2eigen(tvecs[i], xyz_in_camera);

    const Eigen::Vector3d xyz_in_gimbal =
      R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    const Eigen::Vector3d xyz_in_world =
      R_gimbal2world_ * xyz_in_gimbal;
    const Eigen::Matrix3d R_armor2world =
      R_gimbal2world_ * R_camera2gimbal_ * R_armor2camera;

    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(
      object_points,
      rvecs[i],
      tvecs[i],
      camera_matrix_,
      distort_coeffs_,
      projected_points);

    double reprojection_error = 0.0;
    for (std::size_t point_id = 0; point_id < armor.points.size(); ++point_id) {
      reprojection_error +=
        cv::norm(armor.points[point_id] - projected_points[point_id]);
    }
    reprojection_error /= static_cast<double>(armor.points.size());

    PnpCandidateDebug candidate;
    candidate.yaw_in_world = tools::eulers(R_armor2world, 2, 1, 0)[0];
    candidate.reprojection_error = reprojection_error;
    candidate.xyz_in_world = xyz_in_world;
    candidates.push_back(candidate);
  }
  return candidates;
}

//solvePnP（获得姿态）
void Solver::solve(
  Armor & armor, std::optional<Eigen::Vector4d> predicted_armor) const
{
  const auto & object_points = armor_points(armor.type);

  cv::Mat rvec, tvec;
  if (!predicted_armor.has_value()) {
    cv::solvePnP(
      object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
      cv::SOLVEPNP_IPPE);
  } else {
    std::vector<cv::Mat> candidate_rvecs;
    std::vector<cv::Mat> candidate_tvecs;
    const bool solved = cv::solvePnPGeneric(
      object_points,
      armor.points,
      camera_matrix_,
      distort_coeffs_,
      candidate_rvecs,
      candidate_tvecs,
      false,
      cv::SOLVEPNP_IPPE);

    if (!solved || candidate_rvecs.empty() ||
        candidate_rvecs.size() != candidate_tvecs.size()) {
      cv::solvePnP(
        object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
        cv::SOLVEPNP_IPPE);
    } else {
      std::size_t selected_index = 0;
      auto selected_error = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < candidate_rvecs.size(); ++i) {
        cv::Mat candidate_rotation;
        cv::Rodrigues(candidate_rvecs[i], candidate_rotation);
        Eigen::Matrix3d R_armor2camera;
        cv::cv2eigen(candidate_rotation, R_armor2camera);

        const auto R_armor2world =
          R_gimbal2world_ * R_camera2gimbal_ * R_armor2camera;
        const auto candidate_yaw = tools::eulers(R_armor2world, 2, 1, 0)[0];
        const auto yaw_error = std::abs(tools::limit_rad(
          candidate_yaw - (*predicted_armor)[3]));
        if (yaw_error < selected_error) {
          selected_error = yaw_error;
          selected_index = i;
        }
      }
      rvec = candidate_rvecs[selected_index];
      tvec = candidate_tvecs[selected_index];
    }
  }

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);//将旋转向量转换为旋转矩阵
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);//将旋转矩阵转换为Eigen矩阵
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);//将旋转矩阵转换为欧拉角
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);//将旋转矩阵转换为欧拉角

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);//转换为平面坐标系下的yaw、pitch、distance

  // 平衡不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);//判断是否为平衡步兵
  if (is_balance) return;

  const auto reference_yaw =
    predicted_armor ? std::optional<double>((*predicted_armor)[3]) : std::nullopt;
  optimize_yaw(armor, reference_yaw);
}

std::vector<cv::Point2f> Solver::reproject_pnp(const Armor & armor) const
{
  if (armor.points.size() != 4) {
    return {};
  }

  const auto & object_points = armor_points(armor.type);

  cv::Vec3d rvec, tvec;
  const bool solved = cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
  if (!solved) {
    return {};
  }

  std::vector<cv::Point2f> image_points;
  cv::projectPoints(
    object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;//
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = armor_points(type);
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}//重投影装甲板的四个角点到图像平面上，返回图像坐标系下的四个点

double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  // solve
  const auto & object_points = armor_points(armor.type);

  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

void Solver::optimize_yaw(
  Armor & armor, std::optional<double> reference_yaw) const
{
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  armor.yaw_raw = armor.ypr_in_world[0];//把原始yaw赋值给armor.yaw_raw
  const auto correction = std::abs(tools::limit_rad(best_yaw - armor.yaw_raw));
  const auto raw_reference_error = reference_yaw
    ? std::abs(tools::limit_rad(armor.yaw_raw - *reference_yaw))
    : std::numeric_limits<double>::infinity();
  const auto optimized_reference_error = reference_yaw
    ? std::abs(tools::limit_rad(best_yaw - *reference_yaw))
    : 0.0;
  const auto preserves_prediction_continuity =
    optimized_reference_error <= raw_reference_error;
  // 仿真中的矩形对称性可能让重投影优化落入约 ±90° 的错误分支。
  // 修正量异常时保留原始 PnP yaw，避免车辆中心和装甲板模型 ID 被整体旋错。
  armor.ypr_in_world[0] =
    correction <= max_yaw_optimization_correction_ && preserves_prediction_continuity
      ? best_yaw
      : armor.yaw_raw;
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
}  // namespace auto_aim
