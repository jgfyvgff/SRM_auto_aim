#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using Image = sensor_msgs::msg::Image;
using Pose = geometry_msgs::msg::QuaternionStamped;

struct Options
{
    std::filesystem::path source_jsonl;
    std::filesystem::path frames_dir;
    std::filesystem::path output_jsonl;
    double idle_timeout_s = 2.0;
};

struct PendingPair
{
    Image::ConstSharedPtr image;
    Pose::ConstSharedPtr pose;
};

std::optional<double> finite_number(const Json & row, const char * key)
{
    if (!row.contains(key) || !row[key].is_number()) return std::nullopt;
    const double value = row[key].get<double>();
    if (!std::isfinite(value)) return std::nullopt;
    return value;
}

std::vector<Json> load_frame_rows(const std::filesystem::path & path)
{
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open source JSONL: " + path.string());

    std::vector<Json> rows;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        auto row = Json::parse(line);
        if (row.value("event", std::string{}) == "frame") rows.push_back(std::move(row));
    }
    if (rows.empty()) throw std::runtime_error("Source JSONL contains no frame rows");
    return rows;
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

Options parse_options(int argc, char ** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        const auto set_value = [&](const std::string & name, std::string & target) {
            const std::string prefix = name + "=";
            if (arg.rfind(prefix, 0) == 0) {
                target = arg.substr(prefix.size());
                return true;
            }
            return false;
        };
        std::string value;
        if (set_value("--source-jsonl", value)) options.source_jsonl = value;
        else if (set_value("--frames-dir", value)) options.frames_dir = value;
        else if (set_value("--output-jsonl", value)) options.output_jsonl = value;
        else if (set_value("--idle-timeout-s", value)) {
            options.idle_timeout_s = std::stod(value);
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: standard_srm_bayer_prepare"
                << " --source-jsonl=FILE --frames-dir=DIR --output-jsonl=FILE"
                << " [--idle-timeout-s=2]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown option: " + arg);
        }
    }

    if (options.source_jsonl.empty() || options.frames_dir.empty() ||
        options.output_jsonl.empty() || !std::isfinite(options.idle_timeout_s) ||
        options.idle_timeout_s <= 0.0) {
        throw std::invalid_argument(
            "Source JSONL, frames directory, output JSONL, and positive idle timeout are required");
    }
    return options;
}

cv::Mat to_bgr(const Image & message)
{
    if (message.width == 0 || message.height == 0 ||
        message.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        message.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("Invalid image dimensions");
    }

    if (message.encoding == "bgr8") {
        if (message.step < message.width * 3U ||
            message.data.size() < static_cast<std::size_t>(message.step) * message.height) {
            throw std::runtime_error("Malformed bgr8 image data");
        }
        cv::Mat view(
            static_cast<int>(message.height), static_cast<int>(message.width),
            CV_8UC3, const_cast<std::uint8_t *>(message.data.data()), message.step);
        return view.clone();
    }

    if (message.step < message.width ||
        message.data.size() < static_cast<std::size_t>(message.step) * message.height) {
        throw std::runtime_error("Malformed Bayer8 image data");
    }
    int conversion = -1;
    if (message.encoding == "bayer_rggb8") conversion = cv::COLOR_BayerBG2BGR;
    else if (message.encoding == "bayer_bggr8") conversion = cv::COLOR_BayerRG2BGR;
    else if (message.encoding == "bayer_gbrg8") conversion = cv::COLOR_BayerGR2BGR;
    else if (message.encoding == "bayer_grbg8") conversion = cv::COLOR_BayerGB2BGR;
    else throw std::runtime_error("Unsupported image encoding: " + message.encoding);

    cv::Mat raw(
        static_cast<int>(message.height), static_cast<int>(message.width),
        CV_8UC1, const_cast<std::uint8_t *>(message.data.data()), message.step);
    cv::Mat bgr;
    // 该映射与 HikRobot 实时 Bayer 转换保持一致，避免离线回放出现红蓝通道互换。
    cv::cvtColor(raw, bgr, conversion);
    return bgr;
}

class BayerReplayPreparer final : public rclcpp::Node
{
public:
    BayerReplayPreparer(const Options & options, std::vector<Json> source_rows)
        : Node("standard_srm_bayer_prepare"),
          options_(options),
          source_rows_(std::move(source_rows)),
          output_(options_.output_jsonl)
    {
        if (!output_) throw std::runtime_error("Cannot create output JSONL");

        rclcpp::QoS qos(rclcpp::KeepLast(64));
        // Bayer 单帧约 1.5 MB；可靠 QoS 可重传 UDP 分片，避免 Best Effort 静默丢帧。
        qos.reliable();
        image_sub_ = create_subscription<Image>(
            "/camera/image_raw", qos,
            std::bind(&BayerReplayPreparer::on_image, this, std::placeholders::_1));
        pose_sub_ = create_subscription<Pose>(
            "/gimbal/orientation", qos,
            std::bind(&BayerReplayPreparer::on_pose, this, std::placeholders::_1));
        idle_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&BayerReplayPreparer::check_idle, this));
        RCLCPP_INFO(get_logger(), "Waiting for bag messages on the image and pose topics");
    }

    bool success() const { return success_; }
    const std::string & error() const { return error_; }

private:
    void on_image(const Image::ConstSharedPtr message)
    {
        guarded([&] {
            last_message_at_ = Clock::now();
            received_message_ = true;
            const auto stamp = stamp_ns(message->header.stamp);
            auto & pair = pending_[stamp];
            if (pair.image) throw std::runtime_error("Duplicate image timestamp");
            pair.image = message;
            finish_pair_if_ready(stamp);
        });
    }

    void on_pose(const Pose::ConstSharedPtr message)
    {
        guarded([&] {
            last_message_at_ = Clock::now();
            received_message_ = true;
            const auto stamp = stamp_ns(message->header.stamp);
            auto & pair = pending_[stamp];
            if (pair.pose) throw std::runtime_error("Duplicate pose timestamp");
            pair.pose = message;
            finish_pair_if_ready(stamp);
        });
    }

    template<typename Function>
    void guarded(Function && function)
    {
        if (done_) return;
        try {
            function();
            if (pending_.size() > 64) {
                throw std::runtime_error(
                    "More than 64 unmatched timestamps; replay the bag at a lower rate");
            }
        } catch (const std::exception & exception) {
            fail(exception.what());
        }
    }

    void finish_pair_if_ready(std::int64_t stamp)
    {
        auto it = pending_.find(stamp);
        if (it == pending_.end() || !it->second.image || !it->second.pose) return;
        if (next_row_ >= source_rows_.size()) {
            throw std::runtime_error("Bag contains more image pairs than source JSONL rows");
        }
        if (last_stamp_ns_ && stamp <= *last_stamp_ns_) {
            throw std::runtime_error("Paired image timestamps are not strictly increasing");
        }

        write_pair(stamp, *it->second.image, *it->second.pose, source_rows_[next_row_]);
        last_stamp_ns_ = stamp;
        pending_.erase(it);
        ++next_row_;
    }

    void write_pair(
        std::int64_t stamp, const Image & image, const Pose & pose, const Json & source)
    {
        if (const auto width = finite_number(source, "image_width");
            width && static_cast<std::uint32_t>(*width) != image.width) {
            throw std::runtime_error("Bag image width differs from source JSONL");
        }
        if (const auto height = finite_number(source, "image_height");
            height && static_cast<std::uint32_t>(*height) != image.height) {
            throw std::runtime_error("Bag image height differs from source JSONL");
        }

        const double qx = pose.quaternion.x;
        const double qy = pose.quaternion.y;
        const double qz = pose.quaternion.z;
        const double qw = pose.quaternion.w;
        const double qnorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        if (!std::isfinite(qnorm) || qnorm < 1e-9) {
            throw std::runtime_error("Invalid gimbal quaternion");
        }

        const cv::Mat bgr = to_bgr(image);
        std::ostringstream filename;
        filename << "frame_" << std::setfill('0') << std::setw(6)
                 << (next_row_ + 1) << ".png";
        const auto image_path = options_.frames_dir / filename.str();
        if (!cv::imwrite(image_path.string(), bgr)) {
            throw std::runtime_error("Failed to write " + image_path.string());
        }

        const auto speed = finite_number(source, "aimer_bullet_speed_mps");
        const auto capture_to_aimer = finite_number(source, "capture_to_aimer_ms");
        const auto aimer_ms = finite_number(source, "aimer_ms");
        if (!speed || *speed <= 0.0 || !capture_to_aimer || !aimer_ms ||
            *capture_to_aimer < *aimer_ms) {
            throw std::runtime_error("Source JSONL lacks valid offline Aimer inputs");
        }

        Json replay = source;
        if (!first_stamp_ns_) first_stamp_ns_ = stamp;
        replay["replay_schema_version"] = 1;
        replay["replay_time_s"] =
            static_cast<double>(stamp - *first_stamp_ns_) / 1e9;
        replay["replay_frame_file"] = filename.str();
        replay["replay_gimbal_to_world_w"] = qw / qnorm;
        replay["replay_gimbal_to_world_x"] = qx / qnorm;
        replay["replay_gimbal_to_world_y"] = qy / qnorm;
        replay["replay_gimbal_to_world_z"] = qz / qnorm;
        replay["replay_aimer_bullet_speed_mps"] = *speed;
        replay["replay_aimer_offset_ms"] = *capture_to_aimer - *aimer_ms;

        // 原日志保存了 Planner 状态和反馈速度；据此补齐回放输入，缺少状态时跳过该帧 Planner。
        const auto planner_yaw = finite_number(source, "planner_state_yaw_deg");
        const auto planner_pitch = finite_number(source, "planner_state_pitch_deg");
        const auto planner_speed = finite_number(source, "planner_bullet_speed_mps");
        if (planner_yaw && planner_pitch && planner_speed && *planner_speed > 0.0) {
            constexpr double kDegToRad = CV_PI / 180.0;
            replay["replay_planner_enabled"] = true;
            replay["replay_planner_state_yaw_rad"] = *planner_yaw * kDegToRad;
            replay["replay_planner_state_pitch_rad"] = *planner_pitch * kDegToRad;
            replay["replay_planner_state_yaw_vel_rad_s"] =
                finite_number(source, "planner_measured_yaw_vel_rad_s").value_or(0.0);
            replay["replay_planner_state_pitch_vel_rad_s"] =
                finite_number(source, "planner_measured_pitch_vel_rad_s").value_or(0.0);
            replay["replay_planner_bullet_speed_mps"] = *planner_speed;
            replay["replay_planner_offset_ms"] = *capture_to_aimer;
        } else {
            replay["replay_planner_enabled"] = false;
        }

        output_ << replay.dump() << '\n';
        if (!output_) throw std::runtime_error("Failed to write replay input JSONL");
    }

    void check_idle()
    {
        if (done_ || !received_message_ ||
            Clock::now() - last_message_at_ <
                std::chrono::duration<double>(options_.idle_timeout_s)) {
            return;
        }
        if (!pending_.empty()) {
            std::size_t images_without_pose = 0;
            std::size_t poses_without_image = 0;
            for (const auto & [stamp, pair] : pending_) {
                (void)stamp;
                images_without_pose += pair.image && !pair.pose;
                poses_without_image += pair.pose && !pair.image;
            }
            fail(
                "Bag ended with unmatched timestamps: " +
                std::to_string(images_without_pose) + " images lack poses, " +
                std::to_string(poses_without_image) + " poses lack images; converted " +
                std::to_string(next_row_) + " of " +
                std::to_string(source_rows_.size()));
            return;
        }
        if (next_row_ != source_rows_.size()) {
            fail(
                "Bag/source JSONL frame count mismatch: converted " +
                std::to_string(next_row_) + " of " +
                std::to_string(source_rows_.size()));
            return;
        }
        output_.flush();
        if (!output_) {
            fail("Failed to flush replay input JSONL");
            return;
        }
        success_ = true;
        done_ = true;
        RCLCPP_INFO(get_logger(), "Converted %zu synchronized frames", next_row_);
        rclcpp::shutdown();
    }

    void fail(const std::string & message)
    {
        if (done_) return;
        error_ = message;
        done_ = true;
        RCLCPP_ERROR(get_logger(), "%s", error_.c_str());
        rclcpp::shutdown();
    }

    Options options_;
    std::vector<Json> source_rows_;
    std::ofstream output_;
    std::map<std::int64_t, PendingPair> pending_;
    std::size_t next_row_ = 0;
    std::optional<std::int64_t> first_stamp_ns_;
    std::optional<std::int64_t> last_stamp_ns_;
    Clock::time_point last_message_at_{};
    bool received_message_ = false;
    bool done_ = false;
    bool success_ = false;
    std::string error_;
    rclcpp::Subscription<Image>::SharedPtr image_sub_;
    rclcpp::Subscription<Pose>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr idle_timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
    try {
        const auto options = parse_options(argc, argv);
        if (!std::filesystem::exists(options.source_jsonl)) {
            throw std::runtime_error("Source JSONL does not exist");
        }
        if (std::filesystem::exists(options.output_jsonl)) {
            throw std::runtime_error("Output JSONL already exists");
        }
        if (std::filesystem::exists(options.frames_dir) &&
            (!std::filesystem::is_directory(options.frames_dir) ||
             !std::filesystem::is_empty(options.frames_dir))) {
            throw std::runtime_error("Frames directory already exists and is not empty");
        }
        std::filesystem::create_directories(options.frames_dir);
        const auto parent = options.output_jsonl.parent_path();
        if (!parent.empty()) std::filesystem::create_directories(parent);

        auto rows = load_frame_rows(options.source_jsonl);
        rclcpp::init(0, nullptr);
        auto node = std::make_shared<BayerReplayPreparer>(options, std::move(rows));
        rclcpp::spin(node);
        if (rclcpp::ok()) rclcpp::shutdown();
        if (!node->success()) {
            std::cerr << "Bayer preparation failed: " << node->error() << '\n';
            return 1;
        }
        return 0;
    } catch (const std::exception & exception) {
        if (rclcpp::ok()) rclcpp::shutdown();
        std::cerr << "standard_srm_bayer_prepare failed: "
                  << exception.what() << '\n';
        return 1;
    }
}

