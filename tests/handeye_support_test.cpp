#include "calibration/handeye_support.hpp"
#include "calibration/gimbal_pose.hpp"

#include <iostream>
#include <sstream>

namespace
{
using namespace calibration;
void require(bool value, const char * message)
{
    if (!value) throw std::runtime_error(message);
}
template<class Function> void rejects(Function action, const char * message)
{
    bool rejected = false;
    try { action(); } catch (const std::exception &) { rejected = true; }
    require(rejected, message);
}

void test_pose_format()
{
    const auto q = gimbal_ypr_degrees(23, -16, 180);
    std::stringstream saved;
    write_pose(saved, {q, PoseFrame::Gimbal});
    saved << "# image_host_s=10 bracket_ms=20 spread_deg=0.1\n";
    const auto loaded = read_pose(saved);
    require(loaded.frame == PoseFrame::Gimbal && loaded.q.angularDistance(q) < 1e-12, "serial pose roundtrip failed");
    const auto installation = gimbal_ypr_degrees(90, 0, 0).toRotationMatrix();
    require((gimbal_to_world(loaded, installation) - q.toRotationMatrix()).norm() < 1e-12,
        "serial pose applied IMU conversion twice");
    std::stringstream legacy;
    legacy.precision(17);
    legacy << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z();
    const auto old = read_pose(legacy);
    require(old.frame == PoseFrame::ImuBody, "legacy pose was relabeled");
    const Eigen::Matrix3d expected = installation.transpose() * q.toRotationMatrix() * installation;
    require((gimbal_to_world(old, installation) - expected).norm() < 1e-12, "legacy conversion changed");
    for (const std::string text : {"", "1 0", "0 0 0 0", "2 0 0 0", "1 0 0 0 unknown", "nan 0 0 0"}) {
        rejects([&] { std::istringstream input(text); read_pose(input); }, "invalid pose accepted");
    }
}

std::vector<HandeyeObservation> synthetic(const Eigen::Matrix3d & rc, const Eigen::Vector3d & tc, bool single_axis)
{
    const Eigen::Matrix3d board_rotation = gimbal_ypr_degrees(13, 4, -7).toRotationMatrix();
    const Eigen::Vector3d board_position(2000, 150, 250);
    std::vector<HandeyeObservation> observations;
    for (double yaw : {-25.0, -12.0, 0.0, 14.0, 27.0}) {
        for (double pitch : {-15.0, 0.0, 12.0}) {
            const Eigen::Matrix3d rg = gimbal_ypr_degrees(yaw, single_axis ? 0.0 : pitch, 0).toRotationMatrix();
            // 从已知外参生成真实刚体观测，独立验证求解方向和 mm→m 边界。
            observations.push_back({rg, rc.transpose() * rg.transpose() * board_rotation,
                rc.transpose() * (rg.transpose() * board_position - tc)});
        }
    }
    return observations;
}

void test_handeye()
{
    const Eigen::Matrix3d optical_to_gimbal{{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}};
    const Eigen::Vector3d expected_translation_mm(120, -25, 80);
    // 同时验证正常安装、相机图像倒置180度的安装，以及恰好180度的外参旋转。
    const std::vector<Eigen::Matrix3d> rotations{
        optical_to_gimbal, optical_to_gimbal * gimbal_ypr_degrees(180, 0, 0).toRotationMatrix(),
        gimbal_ypr_degrees(0, 0, 180).toRotationMatrix()};
    for (const auto & expected_rotation : rotations) {
        auto observations = synthetic(expected_rotation, expected_translation_mm, false);
        const auto result = solve_handeye(observations);
        require((result.camera_to_gimbal - expected_rotation).norm() < 1e-7, "handeye rotation direction incorrect");
        require((result.camera_in_gimbal_m - expected_translation_mm / 1000.0).norm() < 1e-8, "handeye translation/units incorrect");
        require(result.board_position_rms_m < 1e-8 && result.board_rotation_rms_deg < 1e-6, "synthetic closure failed");
        observations.front().board_in_camera_mm.x() += 5;
        require(solve_handeye(observations).board_position_max_m > 0.001, "closure ignored inconsistent observations");
    }
    rejects([&] { solve_handeye(synthetic(optical_to_gimbal, expected_translation_mm, true)); }, "single-axis dataset accepted");
    rejects([] { solve_handeye({}); }, "empty dataset accepted");
    auto observations = synthetic(optical_to_gimbal, expected_translation_mm, false);
    for (auto & observation : observations) observation.gimbal_to_world.setIdentity();
    rejects([&] { solve_handeye(observations); }, "identical poses accepted");
    rejects([&] { solve_handeye(observations, HandeyePolicy{2, 5, 0.05}); }, "invalid sample policy accepted");
}
}

int main()
{
    try {
        test_pose_format();
        test_handeye();
        std::cout << "handeye_support_test passed\n";
        return 0;
    } catch (const std::exception & error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
