#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <list>
#include <limits>
#include <optional>
#include <tuple>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "src/real_auto_aim/config.hpp"
#include "src/real_auto_aim/frame_diagnostics.hpp"
#include "src/real_auto_aim/quintic_switch_diagnostic.hpp"
#include "src/real_auto_aim/tx_command_limiter.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;

const std::string kKeys =
    "{help h usage ?||输出命令行参数说明}"
    "{@config-path|configs/real_auto_aim.yaml|算法配置路径}"
    "{input-jsonl||standard_srm 回放输入 JSONL}"
    "{frames-dir||与输入 JSONL 对应的 PNG 帧目录}"
    "{output-jsonl||离线重跑结果 JSONL}"
    "{show|false|显示离线回放窗口，按 q 或 Esc 退出}"
    "{model-frame-id|-1|只显示此 frame_id 的单帧整车模型}"
    "{model-frame-output||可选的单帧模型 PNG 输出路径}"
    "{compare-centered-mpc|false|额外计算居中窗口 MPC 诊断，不参与模拟 TX}"
    "{compare-quintic|false|额外计算因果五次多项式选板过渡，仅用于回放诊断}"
    "{quintic-frame-ms|65|五次曲线校验单条 TX 步长时假定的帧周期，单位 ms}"
    "{debug-udp|false|按回放时间实时发送逐帧诊断 JSON 到 127.0.0.1:9871}"
    "{simulate-tx|true|在回放结果中记录 Planner TX 参考角，不连接串口}"
    "{tx-use-nominal-speed|false|弹速反馈不可用时允许模拟名义弹速 TX}"
    "{tx-period-ms|20|TX 初始步长计算周期，单位 ms}"
    "{tx-command-ttl-ms|100|两次模拟命令间隔上限，单位 ms}"
    "{tx-max-yaw-step-deg|2.0|单条模拟命令 yaw 最大变化量，单位 deg}"
    "{tx-max-pitch-step-deg|1.0|单条模拟命令 pitch 最大变化量，单位 deg}"
    "{tx-max-yaw-rate-deg-s|60.0|yaw 最大角速度，单位 deg/s}"
    "{tx-max-pitch-rate-deg-s|30.0|pitch 最大角速度，单位 deg/s}"
    "{tx-follow-temp-lost|true|temp_lost 期间继续模拟受限预测跟随}"
    "{tx-temp-lost-frames|5|允许外推的最大连续丢失帧数}"
    "{tx-temp-lost-rate-scale|0.35|外推限速比例，范围 (0,1]}";

void copy_timeline_fields(const Json & source, Json & output)
{
    for (const char * key : {
             "frame_id", "device_ticks", "tick_hz", "timestamp_source",
             "captured_frame", "replay_time_s", "image_width", "image_height"}) {
        if (source.contains(key)) output[key] = source[key];
    }
}

void append_target(Json & output, const std::list<auto_aim::Target> & targets)
{
    output["targets"] = targets.size();
    if (targets.empty()) return;

    const auto & target = targets.front();
    const auto state = target.ekf_x();
    output["current_armor_id"] = target.last_id;
    if (state.size() >= 11) {
        output["center_x"] = state[0];
        output["center_y"] = state[2];
        output["center_z"] = state[4];
        output["vx"] = state[1];
        output["vy"] = state[3];
        output["vz"] = state[5];
        output["center_yaw"] = state[6];
        output["angular_velocity"] = state[7];
        output["radius"] = state[8];
        output["radius_delta"] = state[9];
        output["height_delta"] = state[10];
    }

    const auto armors = target.armor_xyza_list();
    if (target.last_id >= 0 &&
        static_cast<std::size_t>(target.last_id) < armors.size()) {
        const auto & current = armors[static_cast<std::size_t>(target.last_id)];
        output["current_x"] = current[0];
        output["current_y"] = current[1];
        output["current_z"] = current[2];
        output["current_yaw"] = current[3];
    }
}

void append_plan(
    Json & output, const auto_aim::Plan & plan,
    bool has_aimer_armor)
{
    output["planner_status"] =
        !plan.diagnostic_valid ? "invalid" :
        plan.solver_converged ? "ok" : "unconverged_diagnostic";
    output["planner_control"] =
        plan.control && has_aimer_armor ? 1 : 0;
    output["planner_preview_switch_ms"] =
        plan.preview_switch_step >= 0
            ? Json(plan.preview_switch_step * auto_aim::DT * 1000.0)
            : Json(nullptr);
    output["planner_solver_converged"] = plan.solver_converged;
    output["planner_yaw_solver_status"] = plan.yaw_solver_status;
    output["planner_pitch_solver_status"] = plan.pitch_solver_status;
    output["planner_yaw_solver_iterations"] = plan.yaw_solver_iterations;
    output["planner_pitch_solver_iterations"] = plan.pitch_solver_iterations;

    for (const char * key : {
             "planner_target_yaw_deg", "planner_target_pitch_deg",
             "planner_yaw_deg", "planner_pitch_deg",
             "planner_yaw_20ms_deg", "planner_pitch_20ms_deg",
             "planner_target_yaw_100ms_deg", "planner_target_pitch_100ms_deg",
             "planner_yaw_100ms_deg", "planner_pitch_100ms_deg",
             "planner_yaw_vel_rad_s", "planner_pitch_vel_rad_s",
             "planner_yaw_acc_rad_s2", "planner_pitch_acc_rad_s2",
             "planner_yaw_primal_residual_max", "planner_yaw_dual_residual_max",
             "planner_pitch_primal_residual_max", "planner_pitch_dual_residual_max"}) {
        output[key] = nullptr;
    }
    if (!plan.diagnostic_valid) return;

    constexpr double kRadToDeg = 180.0 / CV_PI;
    output["planner_target_yaw_deg"] = plan.target_yaw * kRadToDeg;
    output["planner_target_pitch_deg"] = plan.target_pitch * kRadToDeg;
    output["planner_yaw_deg"] = plan.yaw * kRadToDeg;
    output["planner_pitch_deg"] = plan.pitch * kRadToDeg;
    output["planner_yaw_20ms_deg"] =
        tools::limit_rad(plan.yaw + auto_aim::DT * plan.yaw_vel) * kRadToDeg;
    output["planner_pitch_20ms_deg"] =
        (plan.pitch + auto_aim::DT * plan.pitch_vel) * kRadToDeg;
    output["planner_target_yaw_100ms_deg"] = plan.target_yaw_100ms * kRadToDeg;
    output["planner_target_pitch_100ms_deg"] = plan.target_pitch_100ms * kRadToDeg;
    output["planner_yaw_100ms_deg"] = plan.yaw_100ms * kRadToDeg;
    output["planner_pitch_100ms_deg"] = plan.pitch_100ms * kRadToDeg;
    output["planner_yaw_vel_rad_s"] = plan.yaw_vel;
    output["planner_pitch_vel_rad_s"] = plan.pitch_vel;
    output["planner_yaw_acc_rad_s2"] = plan.yaw_acc;
    output["planner_pitch_acc_rad_s2"] = plan.pitch_acc;
    output["planner_yaw_primal_residual_max"] = plan.yaw_primal_residual_max;
    output["planner_yaw_dual_residual_max"] = plan.yaw_dual_residual_max;
    output["planner_pitch_primal_residual_max"] = plan.pitch_primal_residual_max;
    output["planner_pitch_dual_residual_max"] = plan.pitch_dual_residual_max;
}

void draw_replay_polygon(
    cv::Mat & image, const std::vector<cv::Point2f> & points,
    const cv::Scalar & color, const std::string & label)
{
    if (points.size() < 3) return;
    for (std::size_t i = 0; i < points.size(); ++i) {
        cv::line(image, points[i], points[(i + 1) % points.size()],
                 color, 2, cv::LINE_AA);
        cv::circle(image, points[i], 4, color, -1, cv::LINE_AA);
    }
    const auto bounds = cv::boundingRect(points);
    cv::putText(image, label, {bounds.x, std::max(20, bounds.y - 6)},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv::LINE_AA);
}

// 单帧几何核验只画板框轮廓，避免检测框、标签和角点标记干扰判断。
void draw_replay_outline(
    cv::Mat & image, const std::vector<cv::Point2f> & points,
    const cv::Scalar & color)
{
    if (points.size() < 3) return;
    for (std::size_t i = 0; i < points.size(); ++i) {
        cv::line(image, points[i], points[(i + 1) % points.size()],
                 color, 2, cv::LINE_AA);
    }
}

void draw_replay_text(cv::Mat & image, const std::string & text, int line)
{
    cv::putText(image, text, {15, 28 + line * 26},
                cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
}

struct PnpObservation
{
    std::size_t detection_index = 0;
    const auto_aim::Armor * armor = nullptr;
    auto_aim::PnpCandidateDebug pose;
};

struct ObservationVehicleModel
{
    bool joint = false;
    std::size_t first_detection = 0;
    std::size_t second_detection = 0;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Matrix3d base_rotation = Eigen::Matrix3d::Identity();
    auto_aim::ArmorType armor_type = auto_aim::ArmorType::small;
    double height_delta = 0.0;
    double center_gap_m = 0.0;
    double rotation_gap_rad = 0.0;
};

Eigen::Vector3d inferred_center(
    const auto_aim::PnpCandidateDebug & pose, double radius)
{
    return pose.xyz_in_world + radius * Eigen::Vector3d(
        std::cos(pose.yaw_in_world), std::sin(pose.yaw_in_world), 0.0);
}

Eigen::Matrix3d undo_model_phase(const Eigen::Matrix3d & armor_rotation, int model_id)
{
    const double phase = model_id * CV_PI / 2.0;
    return Eigen::AngleAxisd(-phase, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
           armor_rotation;
}

std::optional<ObservationVehicleModel> fit_observation_vehicle_model(
    const std::list<auto_aim::Armor> & detections,
    const auto_aim::Solver & solver, double radius)
{
    std::vector<PnpObservation> observations;
    std::size_t detection_index = 0;
    for (const auto & armor : detections) {
        for (const auto & pose : solver.pnp_candidates(armor)) {
            if (!pose.xyz_in_world.allFinite() || !pose.R_armor2world.allFinite() ||
                !std::isfinite(pose.yaw_in_world) ||
                !std::isfinite(pose.reprojection_error)) {
                continue;
            }
            PnpObservation observation;
            observation.detection_index = detection_index;
            observation.armor = &armor;
            observation.pose = pose;
            observations.push_back(std::move(observation));
        }
        ++detection_index;
    }

    // 两个不同装甲板必须能反推出同一中心和同一车体朝向；按中心、姿态、PnP
    // 重投影误差依次排序，避免把米、弧度和像素混成一个没有物理意义的总分。
    std::optional<ObservationVehicleModel> best_pair;
    std::tuple<double, double, double, double> best_pair_rank{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
    for (std::size_t first = 0; first < observations.size(); ++first) {
        const auto & a = observations[first];
        for (std::size_t second = first + 1; second < observations.size(); ++second) {
            const auto & b = observations[second];
            if (a.detection_index == b.detection_index ||
                a.armor->color != b.armor->color ||
                a.armor->name != b.armor->name ||
                a.armor->type != b.armor->type) {
                continue;
            }

            for (int first_id = 0; first_id < 4; ++first_id) {
                for (int second_id = 0; second_id < 4; ++second_id) {
                    if (first_id == second_id) continue;

                    const auto center_a = inferred_center(a.pose, radius);
                    const auto center_b = inferred_center(b.pose, radius);
                    const double dx = center_a.x() - center_b.x();
                    const double dy = center_a.y() - center_b.y();
                    const bool first_high = first_id % 2 != 0;
                    const bool second_high = second_id % 2 != 0;
                    const double dz = first_high == second_high
                        ? center_a.z() - center_b.z() : 0.0;
                    const double center_gap = std::sqrt(dx * dx + dy * dy + dz * dz);

                    const auto base_rotation_a =
                        undo_model_phase(a.pose.R_armor2world, first_id);
                    const auto base_rotation_b =
                        undo_model_phase(b.pose.R_armor2world, second_id);
                    const Eigen::AngleAxisd relative_rotation(
                        base_rotation_a.transpose() * base_rotation_b);
                    const double rotation_gap = std::abs(relative_rotation.angle());
                    const double reprojection_error =
                        a.pose.reprojection_error + b.pose.reprojection_error;
                    const double confidence_rank =
                        -(a.armor->confidence + b.armor->confidence);
                    const auto rank = std::make_tuple(
                        center_gap, rotation_gap, reprojection_error, confidence_rank);
                    if (rank >= best_pair_rank) continue;

                    Eigen::Quaterniond rotation_a(base_rotation_a);
                    Eigen::Quaterniond rotation_b(base_rotation_b);
                    if (rotation_a.dot(rotation_b) < 0.0) {
                        rotation_b.coeffs() *= -1.0;
                    }
                    Eigen::Quaterniond mean_rotation;
                    mean_rotation.coeffs() = rotation_a.coeffs() + rotation_b.coeffs();
                    if (mean_rotation.norm() <= 1e-9) continue;
                    mean_rotation.normalize();

                    ObservationVehicleModel model;
                    model.joint = true;
                    model.first_detection = a.detection_index;
                    model.second_detection = b.detection_index;
                    model.armor_type = a.armor->type;
                    model.center.x() = (center_a.x() + center_b.x()) / 2.0;
                    model.center.y() = (center_a.y() + center_b.y()) / 2.0;
                    if (first_high != second_high) {
                        const auto & even_pose = first_high ? b.pose : a.pose;
                        const auto & odd_pose = first_high ? a.pose : b.pose;
                        model.center.z() = even_pose.xyz_in_world.z();
                        model.height_delta = odd_pose.xyz_in_world.z() - model.center.z();
                    } else {
                        // 同奇偶板位无法从两框单独估计高度差，沿用零高度差先验。
                        model.center.z() =
                            (a.pose.xyz_in_world.z() + b.pose.xyz_in_world.z()) / 2.0;
                    }
                    model.base_rotation = mean_rotation.toRotationMatrix();
                    model.center_gap_m = center_gap;
                    model.rotation_gap_rad = rotation_gap;
                    best_pair = model;
                    best_pair_rank = rank;
                }
            }
        }
    }
    if (best_pair) return best_pair;

    // 单检测帧只能给出一个锚定假设；没有第二块板时不能把它当作整车模型已验证。
    const PnpObservation * best_single = nullptr;
    for (const auto & observation : observations) {
        if (!best_single ||
            observation.armor->confidence > best_single->armor->confidence ||
            (observation.armor->confidence == best_single->armor->confidence &&
             observation.pose.reprojection_error < best_single->pose.reprojection_error)) {
            best_single = &observation;
        }
    }
    if (!best_single) return std::nullopt;

    ObservationVehicleModel model;
    model.first_detection = best_single->detection_index;
    model.armor_type = best_single->armor->type;
    model.center = inferred_center(best_single->pose, radius);
    model.base_rotation = best_single->pose.R_armor2world;
    return model;
}
}

int main(int argc, char * argv[])
{
    try {
        cv::CommandLineParser cli(argc, argv, kKeys);
        if (cli.has("help")) {
            cli.printMessage();
            return 0;
        }
        if (!cli.check()) {
            cli.printErrors();
            return 2;
        }

        const auto config_path = cli.get<std::string>(0);
        const std::filesystem::path input_path(cli.get<std::string>("input-jsonl"));
        const std::filesystem::path frames_dir(cli.get<std::string>("frames-dir"));
        const std::filesystem::path output_path(cli.get<std::string>("output-jsonl"));
        if (config_path.empty() || input_path.empty() || frames_dir.empty() ||
            output_path.empty()) {
            throw std::invalid_argument(
                "Config, input JSONL, frames directory and output JSONL are required");
        }
        if (std::filesystem::weakly_canonical(input_path) ==
            std::filesystem::weakly_canonical(output_path)) {
            throw std::invalid_argument("Input and output JSONL paths must differ");
        }
        const auto yaml = YAML::LoadFile(config_path);
        const auto config = real_auto_aim::validate_real_config(yaml);
        // 使用与 Tracker 相同的半径先验重建纯图像几何诊断模型。
        const double diagnostic_radius = yaml["standard_radius"].as<double>(0.22);
        const bool show = cli.get<bool>("show");
        const int model_frame_id = cli.get<int>("model-frame-id");
        const std::filesystem::path model_frame_output(
            cli.get<std::string>("model-frame-output"));
        if (model_frame_id == 0 || model_frame_id < -1 ||
            (model_frame_id > 0 && !show && model_frame_output.empty()) ||
            (model_frame_id < 0 && !model_frame_output.empty())) {
            throw std::invalid_argument("Invalid single-frame model display options");
        }
        if (!model_frame_output.empty() &&
            std::filesystem::exists(model_frame_output)) {
            throw std::invalid_argument("Single-frame PNG output already exists");
        }
        if (!model_frame_output.empty() &&
            (std::filesystem::weakly_canonical(model_frame_output) ==
                 std::filesystem::weakly_canonical(input_path) ||
             std::filesystem::weakly_canonical(model_frame_output) ==
                 std::filesystem::weakly_canonical(output_path))) {
            throw std::invalid_argument(
                "Single-frame PNG output must differ from JSONL paths");
        }
        if (show) cv::namedWindow("standard_srm_replay", cv::WINDOW_NORMAL);
        const bool simulate_tx = cli.get<bool>("simulate-tx");
        const bool compare_centered = cli.get<bool>("compare-centered-mpc");
        const bool compare_quintic = cli.get<bool>("compare-quintic");
        const double quintic_frame_ms = cli.get<double>("quintic-frame-ms");
        const bool tx_use_nominal_speed = cli.get<bool>("tx-use-nominal-speed");
        const int tx_period_ms = cli.get<int>("tx-period-ms");
        const int tx_ttl_ms = cli.get<int>("tx-command-ttl-ms");
        const int tx_lost_frames = cli.get<int>("tx-temp-lost-frames");
        const double tx_lost_scale = cli.get<double>("tx-temp-lost-rate-scale");
        const bool tx_follow_lost = cli.get<bool>("tx-follow-temp-lost");
        const real_auto_aim::TxRateLimits tx_limits{
            cli.get<double>("tx-max-yaw-rate-deg-s"),
            cli.get<double>("tx-max-pitch-rate-deg-s"),
            cli.get<double>("tx-max-yaw-step-deg"),
            cli.get<double>("tx-max-pitch-step-deg")};
        const auto positive_finite = [](double value) {
            return std::isfinite(value) && value > 0.0;
        };
        if (tx_period_ms <= 0 || tx_ttl_ms <= 0 || tx_lost_frames < 0 ||
            !std::isfinite(tx_lost_scale) || tx_lost_scale <= 0.0 ||
            tx_lost_scale > 1.0 ||
            !positive_finite(tx_limits.yaw_rate_deg_s) ||
            !positive_finite(tx_limits.pitch_rate_deg_s) ||
            !positive_finite(tx_limits.yaw_step_cap_deg) ||
            !positive_finite(tx_limits.pitch_step_cap_deg) ||
            (compare_quintic && !positive_finite(quintic_frame_ms))) {
            throw std::invalid_argument("Invalid offline TX simulation limits");
        }

        std::ifstream input(input_path);
        if (!input) throw std::runtime_error("Unable to open input JSONL");
        const auto parent = output_path.parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent);
        std::ofstream output_file(output_path, std::ios::out | std::ios::trunc);
        if (!output_file) throw std::runtime_error("Unable to open output JSONL");

        std::optional<tools::Plotter> udp_plotter;
        if (cli.get<bool>("debug-udp")) udp_plotter.emplace();
        std::optional<double> udp_origin_s;
        std::optional<Clock::time_point> udp_origin_wall;
        const auto emit_sample = [&](const Json & sample) {
            output_file << sample.dump() << '\n';
            if (!output_file) throw std::runtime_error("Failed to write replay JSONL");
            if (!udp_plotter) return;

            // 算法仍使用采集时的虚拟时间；仅按回放时间控制 UDP 展示节奏。
            // 推理慢于采集时不丢帧，发送自然会落后于一倍速。
            if (sample.contains("replay_time_s") &&
                sample["replay_time_s"].is_number()) {
                const double replay_s = sample["replay_time_s"].get<double>();
                if (std::isfinite(replay_s)) {
                    if (!udp_origin_s) {
                        udp_origin_s = replay_s;
                        udp_origin_wall = Clock::now();
                    } else {
                        const auto due = *udp_origin_wall +
                            std::chrono::duration_cast<Clock::duration>(
                                std::chrono::duration<double>(
                                    replay_s - *udp_origin_s));
                        std::this_thread::sleep_until(due);
                    }
                }
            }
            udp_plotter->plot(sample);
        };

        auto_aim::YOLO detector(config_path, false);
        auto_aim::Solver solver(config_path);
        auto_aim::Tracker tracker(config_path, solver);
        auto_aim::Aimer aimer(config_path);
        auto_aim::Planner planner(config_path);
        std::optional<auto_aim::Planner> centered_planner;
        if (compare_centered) centered_planner.emplace(config_path);
        std::optional<real_auto_aim::QuinticSwitchDiagnostic> quintic;
        if (compare_quintic) {
            const double max_speed_deg_s = std::min(
                tx_limits.yaw_rate_deg_s,
                tx_limits.yaw_step_cap_deg / (quintic_frame_ms / 1000.0));
            quintic.emplace(
                max_speed_deg_s * CV_PI / 180.0,
                yaml["max_yaw_acc"].as<double>());
        }

        std::string line;
        std::size_t line_number = 0;
        std::size_t replayed_frames = 0;
        bool model_frame_rendered = false;
        std::optional<double> previous_replay_time;
        std::optional<real_auto_aim::TxAnglesDeg> last_tx_command;
        std::optional<real_auto_aim::TxAnglesDeg> last_quintic_command;
        std::optional<double> last_quintic_command_time_s;
        const Clock::time_point virtual_epoch{};
        while (std::getline(input, line)) {
            ++line_number;
            if (line.empty()) continue;
            const Json source = Json::parse(line);
            Json result = Json::object();
            result["offline_replay"] = true;
            copy_timeline_fields(source, result);

            if (source.value("event", std::string{}) != "frame") {
                result["event"] = "skip";
                result["skip_reason"] = source.value(
                    "skip_reason", std::string("source row has no matched feedback pose"));
                emit_sample(result);
                continue;
            }
            if (source.value("replay_schema_version", 0) != 1) {
                throw std::runtime_error(
                    "Input lacks replay inputs; recapture with --replay-frame-dir (line " +
                    std::to_string(line_number) + ")");
            }

            const double elapsed_s = source.at("replay_time_s").get<double>();
            if (!std::isfinite(elapsed_s) ||
                (previous_replay_time && elapsed_s <= *previous_replay_time)) {
                throw std::runtime_error("Replay timestamps are not strictly increasing");
            }
            previous_replay_time = elapsed_s;
            const auto frame_time = virtual_epoch +
                std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double>(elapsed_s));

            const std::filesystem::path frame_name(
                source.at("replay_frame_file").get<std::string>());
            if (frame_name.empty() || frame_name.is_absolute() ||
                frame_name.filename() != frame_name) {
                throw std::runtime_error("Invalid replay frame filename");
            }
            const auto image_path = frames_dir / frame_name;
            const cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
            if (image.empty()) {
                throw std::runtime_error("Unable to read replay frame: " + image_path.string());
            }
            if (image.cols != config.image_width || image.rows != config.image_height) {
                throw std::runtime_error("Replay frame size differs from calibration image_size");
            }

            Eigen::Quaterniond pose(
                source.at("replay_gimbal_to_world_w").get<double>(),
                source.at("replay_gimbal_to_world_x").get<double>(),
                source.at("replay_gimbal_to_world_y").get<double>(),
                source.at("replay_gimbal_to_world_z").get<double>());
            if (!pose.coeffs().allFinite() || pose.norm() < 1e-9) {
                throw std::runtime_error("Invalid replay gimbal quaternion");
            }
            pose.normalize();
            solver.set_gimbal_to_world(pose);

            auto armors = detector.detect(image);
            const auto detected_armors = armors;
            real_auto_aim::diagnostics::append_detector_debug_json(
                result, detector.debug_info(), armors);
            auto targets = tracker.track(armors, frame_time);
            // 复用真机日志的门控字段，记录本次离线重跑实际做出的关联判断。
            real_auto_aim::diagnostics::append_association_json(
                result, tracker.association_debug(), solver);
            const double aimer_speed =
                source.at("replay_aimer_bullet_speed_mps").get<double>();
            const double aimer_offset_ms =
                source.at("replay_aimer_offset_ms").get<double>();
            const auto aimer_time = frame_time +
                std::chrono::duration_cast<Clock::duration>(
                    std::chrono::duration<double, std::milli>(aimer_offset_ms));
            const auto aim = aimer.aim(targets, aimer_time, aimer_speed, false);

            result["event"] = "frame";
            result["image_width"] = image.cols;
            result["image_height"] = image.rows;
            result["detected"] = armors.size();
            result["tracker_state"] = tracker.state();
            result["tracker_generation"] = tracker.target_generation();
            result["temp_lost_count"] = tracker.temp_lost_count();
            result["tracker_observation_accepted"] = tracker.observation_accepted();
            result["tracker_geometry_branch"] = tracker.geometry_branch();
            result["tracker_geometry_trusted"] = tracker.geometry_trusted();
            result["tracker_geometry_constraint_active"] =
                tracker.geometry_constraint_active();
            append_target(result, targets);

            const bool aim_valid = aim.control && aimer.debug_aim_point.valid;
            result["aim_control"] = aim.control ? 1 : 0;
            result["aim_shoot"] = aim.shoot ? 1 : 0;
            result["diagnostic_yaw"] = aim_valid ? Json(aim.yaw) : Json(nullptr);
            result["diagnostic_pitch"] = aim_valid ? Json(aim.pitch) : Json(nullptr);
            result["diagnostic_yaw_deg"] =
                aim_valid ? Json(aim.yaw * 180.0 / CV_PI) : Json(nullptr);
            result["diagnostic_pitch_deg"] =
                aim_valid ? Json(aim.pitch * 180.0 / CV_PI) : Json(nullptr);
            result["prediction_dt"] = aimer.debug_prediction_dt;
            result["delay_time"] = aimer_offset_ms / 1000.0;
            result["base_prediction_dt"] = aimer.debug_base_prediction_dt;
            result["fly_time"] = aimer.debug_fly_time;
            result["aim_armor_id"] =
                aim_valid ? Json(aimer.debug_aim_point.armor_id) : Json(nullptr);
            if (aim_valid) {
                const auto & future = aimer.debug_aim_point.xyza;
                result["future_x"] = future[0];
                result["future_y"] = future[1];
                result["future_z"] = future[2];
                result["future_yaw"] = future[3];
            }

            for (const char * key : {
                     "planner_measured_yaw_deg", "planner_measured_pitch_deg",
                     "planner_measured_yaw_vel_rad_s",
                     "planner_measured_pitch_vel_rad_s", "planner_feedback_age_ms",
                     "planner_feedback_interval_ms", "planner_state_source",
                     "capture_to_aimer_ms"}) {
                if (source.contains(key)) result[key] = source[key];
            }
            result["planner_status"] = "no_tracking_target";
            result["planner_control"] = 0;
            if (compare_centered) {
                result["centered_valid"] = false;
                result["centered_solver_converged"] = false;
                result["centered_target_yaw_deg"] = nullptr;
                result["centered_yaw_deg"] = nullptr;
                result["centered_yaw_solver_status"] = nullptr;
                result["centered_yaw_solver_iterations"] = nullptr;
            }
            // 原始日志在 Planner 暂无有效速度时会写 null；回放输出统一为字符串以兼容绘图。
            const auto planner_speed_source = source.find("planner_speed_source");
            result["planner_speed_source"] =
                planner_speed_source != source.end() && planner_speed_source->is_string()
                    ? *planner_speed_source
                    : Json("unavailable");
            result["planner_yaw_deg"] = nullptr;
            result["planner_pitch_deg"] = nullptr;

            // 回放按重新运行后的 Tracker 状态判断暂失，避免沿用采集时的 TX 状态。
            const bool extrapolating =
                tx_follow_lost && tracker.state() == "temp_lost" &&
                tracker.temp_lost_count() <= tx_lost_frames;
            const bool planner_enabled = source.value("replay_planner_enabled", false);
            std::optional<auto_aim::Plan> replay_plan;
            std::optional<int> replay_armor_id;
            std::optional<auto_aim::PlannerState> replay_state;
            auto_aim::SwitchPreview switch_preview;
            if (planner_enabled && !targets.empty() &&
                (tracker.state() == "tracking" ||
                 (extrapolating && tracker.state() == "temp_lost"))) {
                auto target = targets.front();
                const double planner_offset_ms =
                    source.at("replay_planner_offset_ms").get<double>();
                const auto planner_time = frame_time +
                    std::chrono::duration_cast<Clock::duration>(
                        std::chrono::duration<double, std::milli>(planner_offset_ms));
                target.predict(planner_time);
                const auto armor_id =
                    aim.control && aimer.debug_aim_point.valid
                        ? std::optional<int>{aimer.debug_aim_point.armor_id}
                        : std::nullopt;
                const auto state = auto_aim::PlannerState{
                    source.at("replay_planner_state_yaw_rad").get<double>(),
                    source.at("replay_planner_state_yaw_vel_rad_s").get<double>(),
                    source.at("replay_planner_state_pitch_rad").get<double>(),
                    source.at("replay_planner_state_pitch_vel_rad_s").get<double>()};
                const auto plan = planner.plan(
                    target,
                    source.at("replay_planner_bullet_speed_mps").get<double>(),
                    state,
                    armor_id,
                    armor_id ? &aimer : nullptr,
                    quintic ? &switch_preview : nullptr);
                replay_plan = plan;
                replay_armor_id = armor_id;
                replay_state = state;
                result["planner_bullet_speed_mps"] =
                    source.at("replay_planner_bullet_speed_mps");
                result["planner_state_yaw_deg"] = state.yaw * 180.0 / CV_PI;
                result["planner_state_pitch_deg"] = state.pitch * 180.0 / CV_PI;
                append_plan(result, plan, armor_id.has_value());
                if (centered_planner && armor_id) {
                    const auto centered = centered_planner->plan_centered_diagnostic(
                        target,
                        source.at("replay_planner_bullet_speed_mps").get<double>(),
                        *armor_id, aimer);
                    result["centered_valid"] = centered.diagnostic_valid;
                    result["centered_solver_converged"] =
                        centered.solver_converged;
                    if (centered.diagnostic_valid) {
                        result["centered_target_yaw_deg"] =
                            centered.target_yaw * 180.0 / CV_PI;
                        result["centered_yaw_deg"] =
                            centered.yaw * 180.0 / CV_PI;
                        result["centered_yaw_solver_status"] =
                            centered.yaw_solver_status;
                        result["centered_yaw_solver_iterations"] =
                            centered.yaw_solver_iterations;
                    }
                }
            }

            // 用采集时的虚拟时间和 planner 耗时确定帧间限速；不读取墙钟或访问串口。
            const double captured_planner_offset_ms =
                source.contains("replay_planner_offset_ms") &&
                        source["replay_planner_offset_ms"].is_number()
                    ? source["replay_planner_offset_ms"].get<double>() : 0.0;
            const double captured_planner_ms =
                source.contains("planner_ms") && source["planner_ms"].is_number()
                    ? source["planner_ms"].get<double>() : 0.0;
            const double command_time_s =
                elapsed_s + (captured_planner_offset_ms + captured_planner_ms) / 1000.0;
            std::optional<real_auto_aim::QuinticSwitchDiagnostic::Sample> quintic_sample;
            if (quintic) {
                result["quintic_reference_yaw_deg"] = nullptr;
                result["quintic_transition_ms"] = nullptr;
                result["quintic_mode"] = "no_reference";
                if (replay_plan && replay_armor_id &&
                    replay_plan->diagnostic_valid) {
                    const auto & plan = *replay_plan;
                    const double reference_yaw = plan.control
                        ? tools::limit_rad(plan.yaw + auto_aim::DT * plan.yaw_vel)
                        : static_cast<double>(plan.target_yaw);
                    quintic_sample = quintic->update(
                        command_time_s, tracker.target_generation(),
                        *replay_armor_id, reference_yaw, switch_preview);
                    result["quintic_reference_yaw_deg"] =
                        quintic_sample->yaw_rad * 180.0 / CV_PI;
                    result["quintic_transition_ms"] =
                        quintic_sample->transition_ms;
                    result["quintic_mode"] = quintic_sample->mode;
                }
            }
            bool tx_sent = false;
            std::string tx_mode = "none";
            std::string tx_source = "none";
            std::optional<real_auto_aim::TxAnglesDeg> tx_command;
            const auto hold_tx = [&]() {
                if (!last_tx_command && replay_state) {
                    last_tx_command = real_auto_aim::TxAnglesDeg{
                        replay_state->yaw * 180.0 / CV_PI,
                        replay_state->pitch * 180.0 / CV_PI};
                }
                if (last_tx_command) {
                    tx_command = last_tx_command;
                    tx_sent = true;
                    tx_mode = "hold_last_aim";
                    tx_source = "hold";
                }
            };
            if (simulate_tx) {
                if (!replay_plan || !replay_state || !replay_armor_id) {
                    hold_tx();
                } else {
                    const auto & plan = *replay_plan;
                    const bool measured_speed =
                        source.value("tx_speed_source", std::string{}) == "feedback";
                    const bool nominal_speed =
                        tx_use_nominal_speed &&
                        source.contains("replay_planner_bullet_speed_mps") &&
                        source["replay_planner_bullet_speed_mps"].is_number();
                    const bool finite_target =
                        std::isfinite(plan.target_yaw) &&
                        std::isfinite(plan.target_pitch);
                    if (!plan.diagnostic_valid || !finite_target ||
                        (!measured_speed && !nominal_speed)) {
                        if (extrapolating) hold_tx();
                        else {
                            last_tx_command.reset();
                        }
                    } else {
                        const real_auto_aim::TxAnglesDeg target_command =
                            plan.control
                                ? real_auto_aim::TxAnglesDeg{
                                    tools::limit_rad(
                                        plan.yaw + auto_aim::DT * plan.yaw_vel) *
                                        180.0 / CV_PI,
                                    (plan.pitch + auto_aim::DT * plan.pitch_vel) *
                                        180.0 / CV_PI}
                                : real_auto_aim::TxAnglesDeg{
                                    plan.target_yaw * 180.0 / CV_PI,
                                    plan.target_pitch * 180.0 / CV_PI};
                        if (!std::isfinite(target_command.yaw) ||
                            !std::isfinite(target_command.pitch)) {
                            if (extrapolating) hold_tx();
                            else last_tx_command.reset();
                        } else {
                            last_tx_command = target_command;
                            tx_command = target_command;
                            tx_sent = true;
                            tx_mode = extrapolating ? "extrapolate" : "tracking";
                            tx_source = plan.control
                                ? "mpc_reference_20ms" : "target_reference";
                        }
                    }
                }
            }
            result["replay_tx_simulated"] = simulate_tx;
            result["replay_tx_command_sent"] = tx_sent;
            result["replay_tx_command_mode"] = tx_mode;
            result["replay_tx_command_source"] = tx_source;
            result["replay_tx_extrapolating"] =
                tx_sent && tx_mode == "extrapolate";
            result["replay_tx_command_yaw_deg"] =
                tx_command ? Json(tx_command->yaw) : Json(nullptr);
            result["replay_tx_command_pitch_deg"] =
                tx_command ? Json(tx_command->pitch) : Json(nullptr);

            if (quintic) {
                std::optional<real_auto_aim::TxAnglesDeg> quintic_command;
                if (simulate_tx && tx_source == "hold") {
                    if (!last_quintic_command && replay_state) {
                        last_quintic_command = real_auto_aim::TxAnglesDeg{
                            replay_state->yaw * 180.0 / CV_PI,
                            replay_state->pitch * 180.0 / CV_PI};
                    }
                    quintic_command = last_quintic_command;
                    if (quintic_command) {
                        last_quintic_command_time_s = std::max(
                            command_time_s,
                            last_quintic_command_time_s.value_or(command_time_s));
                    }
                } else if (simulate_tx && quintic_sample && replay_state &&
                           replay_plan && tx_sent && tx_source != "none") {
                    const real_auto_aim::TxAnglesDeg base = last_quintic_command
                        ? *last_quintic_command
                        : real_auto_aim::TxAnglesDeg{
                            replay_state->yaw * 180.0 / CV_PI,
                            replay_state->pitch * 180.0 / CV_PI};
                    const double elapsed_tx_s = last_quintic_command_time_s
                        ? std::clamp(
                            command_time_s - *last_quintic_command_time_s,
                            0.0, tx_ttl_ms / 1000.0)
                        : tx_period_ms / 1000.0;
                    const auto & plan = *replay_plan;
                    const double pitch_deg = plan.control
                        ? (plan.pitch + auto_aim::DT * plan.pitch_vel) * 180.0 / CV_PI
                        : plan.target_pitch * 180.0 / CV_PI;
                    quintic_command = real_auto_aim::limit_tx_angles(
                        base,
                        {quintic_sample->yaw_rad * 180.0 / CV_PI, pitch_deg},
                        elapsed_tx_s, tx_limits,
                        extrapolating ? tx_lost_scale : 1.0);
                    last_quintic_command = quintic_command;
                    last_quintic_command_time_s = std::max(
                        command_time_s,
                        last_quintic_command_time_s.value_or(command_time_s));
                } else if (tx_source == "none") {
                    last_quintic_command.reset();
                    last_quintic_command_time_s.reset();
                }
                result["quintic_tx_command_yaw_deg"] = quintic_command
                    ? Json(quintic_command->yaw) : Json(nullptr);
            }

            emit_sample(result);
            ++replayed_frames;
            if (show && model_frame_id < 0) {
                cv::Mat display = image.clone();
                const auto vehicle_model = fit_observation_vehicle_model(
                    detected_armors, solver, diagnostic_radius);
                const cv::Scalar model_color(0, 255, 0);
                std::string model_status = "model=none";
                if (vehicle_model) {
                    const auto & model = *vehicle_model;
                    const double base_yaw = std::atan2(
                        model.base_rotation(1, 0), model.base_rotation(0, 0));
                    for (int model_id = 0; model_id < 4; ++model_id) {
                        const double phase = model_id * CV_PI / 2.0;
                        const double angle = base_yaw + phase;
                        const Eigen::Vector3d board_position(
                            model.center.x() - diagnostic_radius * std::cos(angle),
                            model.center.y() - diagnostic_radius * std::sin(angle),
                            model.center.z() +
                                (model_id % 2 ? model.height_delta : 0.0));
                        const Eigen::Matrix3d board_rotation =
                            Eigen::AngleAxisd(phase, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            model.base_rotation;
                        draw_replay_polygon(
                            display,
                            solver.reproject_armor_pose(
                                board_position, board_rotation, model.armor_type),
                            model_color,
                            std::string(model.joint ? "joint" : "single") +
                                " id=" + std::to_string(model_id));
                    }
                    if (model.joint) {
                        model_status = "model=joint det=" +
                            std::to_string(model.first_detection) + "," +
                            std::to_string(model.second_detection) +
                            " dC=" + std::to_string(model.center_gap_m) + "m dR=" +
                            std::to_string(model.rotation_gap_rad * 180.0 / CV_PI) + "deg";
                    } else {
                        model_status = "model=single det=" +
                            std::to_string(model.first_detection) + " (unverified)";
                    }
                }
                for (const auto & armor : detected_armors) {
                    draw_replay_polygon(
                        display, armor.points, cv::Scalar(0, 255, 255),
                        "det " + std::to_string(armor.confidence));
                }
                cv::rotate(display, display, cv::ROTATE_180);
                draw_replay_text(
                    display,
                    "frame=" + std::to_string(replayed_frames) +
                        " detected=" + std::to_string(detected_armors.size()) +
                        " targets=" + std::to_string(targets.size()),
                    0);
                draw_replay_text(
                    display,
                    "state=" + tracker.state() +
                        " obs=" + (tracker.observation_accepted()
                            ? std::string("accepted") : std::string("not_accepted")),
                    1);
                draw_replay_text(display, model_status, 2);
                cv::imshow("standard_srm_replay", display);
                const int key = cv::waitKey(1) & 0xff;
                if (key == 'q' || key == 27) break;
            }
            if (model_frame_id > 0 &&
                source.value("frame_id", -1) == model_frame_id) {
                const auto vehicle_model = fit_observation_vehicle_model(
                    detected_armors, solver, diagnostic_radius);
                if (vehicle_model && vehicle_model->joint) {
                    cv::Mat display = image.clone();
                    const auto & model = *vehicle_model;
                    const double base_yaw = std::atan2(
                        model.base_rotation(1, 0), model.base_rotation(0, 0));
                    for (int model_id = 0; model_id < 4; ++model_id) {
                        const double phase = model_id * CV_PI / 2.0;
                        const double angle = base_yaw + phase;
                        const Eigen::Vector3d board_position(
                            model.center.x() - diagnostic_radius * std::cos(angle),
                            model.center.y() - diagnostic_radius * std::sin(angle),
                            model.center.z() +
                                (model_id % 2 ? model.height_delta : 0.0));
                        const Eigen::Matrix3d board_rotation =
                            Eigen::AngleAxisd(phase, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            model.base_rotation;
                        draw_replay_outline(
                            display,
                            solver.reproject_armor_pose(
                                board_position, board_rotation, model.armor_type),
                            cv::Scalar(0, 255, 0));
                    }
                    cv::rotate(display, display, cv::ROTATE_180);
                    if (!model_frame_output.empty()) {
                        const auto parent = model_frame_output.parent_path();
                        if (!parent.empty()) std::filesystem::create_directories(parent);
                        if (!cv::imwrite(model_frame_output.string(), display)) {
                            throw std::runtime_error("Failed to save single-frame model PNG");
                        }
                    }
                    model_frame_rendered = true;
                    if (show) {
                        cv::imshow("standard_srm_replay", display);
                        const int key = cv::waitKey(0) & 0xff;
                        if (key == 'q' || key == 27) break;
                    }
                }
            }
        }
        if (replayed_frames == 0) {
            throw std::runtime_error("No replayable frames found in input JSONL");
        }
        if (model_frame_id > 0 && !model_frame_rendered) {
            throw std::runtime_error(
                "Selected frame was missing or had no two-detection joint model");
        }
        std::cout << "Replayed " << replayed_frames << " frames to "
                  << output_path << '\n';
        return 0;
    } catch (const std::exception & error) {
        std::cerr << "standard_srm_replay failed: " << error.what() << '\n';
        return 1;
    }
}
