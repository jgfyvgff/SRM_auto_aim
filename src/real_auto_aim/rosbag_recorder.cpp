#include "src/real_auto_aim/rosbag_recorder.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace real_auto_aim
{
namespace
{
constexpr char kImageTopic[] = "/camera/image_raw";
constexpr char kOrientationTopic[] = "/gimbal/orientation";
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

void set_stamp(builtin_interfaces::msg::Time & stamp, std::int64_t timestamp_ns)
{
    if (timestamp_ns < 0) {
        throw std::runtime_error("Mapped ROS timestamp is negative");
    }
    stamp.sec = static_cast<std::int32_t>(timestamp_ns / kNanosecondsPerSecond);
    stamp.nanosec = static_cast<std::uint32_t>(timestamp_ns % kNanosecondsPerSecond);
}

bool is_bayer8_encoding(const std::string & encoding)
{
    return encoding == "bayer_rggb8" || encoding == "bayer_bggr8" ||
           encoding == "bayer_gbrg8" || encoding == "bayer_grbg8";
}
}  // namespace

RosbagRecorder::RosbagRecorder(const std::string & bag_uri, std::size_t queue_capacity)
    : queue_capacity_(queue_capacity),
      steady_origin_(Clock::now()),
      system_origin_ns_(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count())
{
    if (bag_uri.empty() || queue_capacity_ == 0) {
        throw std::invalid_argument("ROS bag URI and queue capacity must be valid");
    }
    const std::filesystem::path bag_path(bag_uri);
    if (std::filesystem::exists(bag_path)) {
        throw std::invalid_argument("ROS bag destination already exists: " + bag_uri);
    }
    const auto parent = bag_path.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_uri;
    storage_options.storage_id = "sqlite3";
    writer_.open(storage_options, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});

    writer_.create_topic(rosbag2_storage::TopicMetadata{
        kImageTopic, "sensor_msgs/msg/Image", "cdr", ""});
    writer_.create_topic(rosbag2_storage::TopicMetadata{
        kOrientationTopic, "geometry_msgs/msg/QuaternionStamped", "cdr", ""});

    worker_ = std::thread(&RosbagRecorder::worker_loop, this);
}

RosbagRecorder::~RosbagRecorder()
{
    close();
}

bool RosbagRecorder::enqueue(
    const cv::Mat & image, const std::string & image_encoding,
    Clock::time_point image_timestamp,
    const Eigen::Quaterniond & gimbal_to_world)
{
    const bool valid_bgr = image.type() == CV_8UC3 &&
        image_encoding == sensor_msgs::image_encodings::BGR8;
    const bool valid_bayer = image.type() == CV_8UC1 &&
        is_bayer8_encoding(image_encoding);
    if (image.empty() || (!valid_bgr && !valid_bayer) ||
        !gimbal_to_world.coeffs().allFinite() ||
        gimbal_to_world.squaredNorm() <= std::numeric_limits<double>::epsilon()) {
        std::lock_guard<std::mutex> lock(mutex_);
        accepting_ = false;
        stopping_ = true;
        set_error_locked("ROS bag received an unsupported image encoding or invalid quaternion");
        ++frames_dropped_;
        changed_.notify_one();
        return false;
    }

    const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        image_timestamp - steady_origin_).count();
    if ((elapsed_ns > 0 &&
         system_origin_ns_ > std::numeric_limits<std::int64_t>::max() - elapsed_ns) ||
        (elapsed_ns < 0 &&
         system_origin_ns_ < std::numeric_limits<std::int64_t>::min() - elapsed_ns)) {
        std::lock_guard<std::mutex> lock(mutex_);
        accepting_ = false;
        stopping_ = true;
        set_error_locked("ROS bag timestamp conversion overflow");
        ++frames_dropped_;
        changed_.notify_one();
        return false;
    }

    Packet packet;
    packet.image = image;
    packet.image_encoding = image_encoding;
    packet.timestamp_ns = system_origin_ns_ + elapsed_ns;
    packet.gimbal_to_world = gimbal_to_world.normalized();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!accepting_) {
            ++frames_dropped_;
            return false;
        }
        if (queue_.size() >= queue_capacity_) {
            accepting_ = false;
            stopping_ = true;
            ++frames_dropped_;
            set_error_locked("ROS bag queue reached its capacity; recording stopped");
            changed_.notify_one();
            return false;
        }
        queue_.push_back(std::move(packet));
    }
    changed_.notify_one();
    return true;
}

void RosbagRecorder::close() noexcept
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) return;
        accepting_ = false;
        stopping_ = true;
    }
    changed_.notify_one();
    if (worker_.joinable()) worker_.join();

    try {
        writer_.close();
    } catch (const std::exception & exception) {
        std::lock_guard<std::mutex> lock(mutex_);
        set_error_locked(std::string("Failed to close ROS bag: ") + exception.what());
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        set_error_locked("Failed to close ROS bag: unknown error");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
}

std::uint64_t RosbagRecorder::frames_written() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_written_;
}

std::uint64_t RosbagRecorder::frames_dropped() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_dropped_;
}

std::string RosbagRecorder::error() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
}

void RosbagRecorder::worker_loop() noexcept
{
    for (;;) {
        Packet packet;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            changed_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
            if (queue_.empty()) return;
            packet = std::move(queue_.front());
            queue_.pop_front();
        }

        try {
            write_packet(packet);
            std::lock_guard<std::mutex> lock(mutex_);
            ++frames_written_;
        } catch (const std::exception & exception) {
            std::lock_guard<std::mutex> lock(mutex_);
            accepting_ = false;
            stopping_ = true;
            ++frames_dropped_;
            frames_dropped_ += queue_.size();
            queue_.clear();
            set_error_locked(std::string("ROS bag writer failed: ") + exception.what());
            changed_.notify_one();
            return;
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex_);
            accepting_ = false;
            stopping_ = true;
            ++frames_dropped_;
            frames_dropped_ += queue_.size();
            queue_.clear();
            set_error_locked("ROS bag writer failed: unknown error");
            changed_.notify_one();
            return;
        }
    }
}

void RosbagRecorder::write_packet(const Packet & packet)
{
    sensor_msgs::msg::Image image_message;
    set_stamp(image_message.header.stamp, packet.timestamp_ns);
    image_message.header.frame_id = "camera";
    image_message.height = static_cast<std::uint32_t>(packet.image.rows);
    image_message.width = static_cast<std::uint32_t>(packet.image.cols);
    image_message.encoding = packet.image_encoding;
    image_message.is_bigendian = 0;
    image_message.step = static_cast<std::uint32_t>(
        packet.image.cols * packet.image.elemSize());
    image_message.data.resize(
        static_cast<std::size_t>(image_message.step) * image_message.height);
    for (int row = 0; row < packet.image.rows; ++row) {
        std::memcpy(
            image_message.data.data() + static_cast<std::size_t>(row) * image_message.step,
            packet.image.ptr(row), image_message.step);
    }

    geometry_msgs::msg::QuaternionStamped orientation_message;
    set_stamp(orientation_message.header.stamp, packet.timestamp_ns);
    orientation_message.header.frame_id = "world";
    orientation_message.quaternion.w = packet.gimbal_to_world.w();
    orientation_message.quaternion.x = packet.gimbal_to_world.x();
    orientation_message.quaternion.y = packet.gimbal_to_world.y();
    orientation_message.quaternion.z = packet.gimbal_to_world.z();

    const rclcpp::Time timestamp(packet.timestamp_ns, RCL_SYSTEM_TIME);
    writer_.write(image_message, kImageTopic, timestamp);
    writer_.write(orientation_message, kOrientationTopic, timestamp);
}

void RosbagRecorder::set_error_locked(const std::string & message)
{
    if (error_.empty()) error_ = message;
}
}  // namespace real_auto_aim
