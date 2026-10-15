#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "src/real_auto_aim/config.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using Json = nlohmann::json;

const std::string kKeys =
    "{help h usage ?||输出命令行参数说明}"
    "{@config-path|configs/real_auto_aim.yaml|算法配置路径}"
    "{input-jsonl||standard_srm 回放输入 JSONL}"
    "{frames-dir||与输入 JSONL 对应的 PNG 帧目录}"
    "{output-jsonl||离线重跑结果 JSONL}";

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
    bool has_measured_velocity, bool extrapolating)
{
    output["planner_status"] =
        !plan.diagnostic_valid ? "invalid" :
        plan.solver_converged ? "ok" : "unconverged_diagnostic";
    output["planner_control"] =
        plan.control && has_measured_velocity && !extrapolating ? 1 : 0;
    output["planner_solver_converged"] = plan.solver_converged;
    output["planner_yaw_solver_status"] = plan.yaw_solver_status;
    output["planner_pitch_solver_status"] = plan.pitch_solver_status;
    output["planner_yaw_solver_iterations"] = plan.yaw_solver_iterations;
    output["planner_pitch_solver_iterations"] = plan.pitch_solver_iterations;

    for (const char * key : {
             "planner_target_yaw_deg", "planner_target_pitch_deg",
             "planner_yaw_deg", "planner_pitch_deg",
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

        std::ifstream input(input_path);
        if (!input) throw std::runtime_error("Unable to open input JSONL");
        const auto parent = output_path.parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent);
        std::ofstream output_file(output_path, std::ios::out | std::ios::trunc);
        if (!output_file) throw std::runtime_error("Unable to open output JSONL");

        auto_aim::YOLO detector(config_path, false);
        auto_aim::Solver solver(config_path);
        auto_aim::Tracker tracker(config_path, solver);
        auto_aim::Aimer aimer(config_path);
        auto_aim::Planner planner(config_path);

        std::string line;
        std::size_t line_number = 0;
        std::size_t replayed_frames = 0;
        std::optional<double> previous_replay_time;
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
                output_file << result.dump() << '\n';
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
            auto targets = tracker.track(armors, frame_time);
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
            result["planner_speed_source"] = source.value(
                "planner_speed_source", std::string("unavailable"));
            result["planner_yaw_deg"] = nullptr;
            result["planner_pitch_deg"] = nullptr;

            const bool extrapolating = source.value("tx_extrapolating", false);
            const bool planner_enabled = source.value("replay_planner_enabled", false);
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
                    armor_id);
                result["planner_bullet_speed_mps"] =
                    source.at("replay_planner_bullet_speed_mps");
                result["planner_state_yaw_deg"] = state.yaw * 180.0 / CV_PI;
                result["planner_state_pitch_deg"] = state.pitch * 180.0 / CV_PI;
                append_plan(
                    result, plan,
                    source.value("replay_planner_has_measured_velocity", false),
                    extrapolating);
            }

            output_file << result.dump() << '\n';
            ++replayed_frames;
        }
        if (replayed_frames == 0) {
            throw std::runtime_error("No replayable frames found in input JSONL");
        }
        std::cout << "Replayed " << replayed_frames << " frames to "
                  << output_path << '\n';
        return 0;
    } catch (const std::exception & error) {
        std::cerr << "standard_srm_replay failed: " << error.what() << '\n';
        return 1;
    }
}
