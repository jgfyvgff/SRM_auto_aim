#pragma once

#include <chrono>
#include <list>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/command.hpp"
#include "src/real_auto_aim/feedback_buffer.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"

// 这些函数只产生可视化和日志字段，不参与 Tracker、Aimer 或 Planner 决策。
namespace real_auto_aim::diagnostics
{
using Json = nlohmann::json;
using Clock = real_auto_aim::Clock;

double duration_ms(Clock::time_point begin, Clock::time_point end);
void draw_polygon(cv::Mat & image, const std::vector<cv::Point2f> & points,
                  const cv::Scalar & color, const std::string & label);
void draw_text(cv::Mat & image, const std::string & text, int line, const cv::Scalar & color);
Json timing_json(const io::FrameTiming & timing, Clock::time_point now);
void append_reprojection_json(Json & sample, const auto_aim::Solver & solver,
    const std::list<auto_aim::Armor> & detected_armors,
    const std::list<auto_aim::Target> & targets);
void append_target_json(Json & sample, const std::list<auto_aim::Target> & targets);
void append_association_json(Json & sample, const auto_aim::AssociationDebug & association,
    const auto_aim::Solver & solver);
void append_tracker_filter_json(Json & sample, const auto_aim::Tracker & tracker,
    const std::list<auto_aim::Target> & targets);
void append_aim_json(Json & sample, const auto_aim::Aimer & aimer, const io::Command & command);
void append_image_quality_json(Json & sample, const cv::Mat & image);
}  // namespace real_auto_aim::diagnostics
