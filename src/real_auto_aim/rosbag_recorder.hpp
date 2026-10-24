#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <rosbag2_cpp/writer.hpp>

#include "src/real_auto_aim/feedback_buffer.hpp"

namespace real_auto_aim
{
// 将图像与其匹配的云台姿态写入 ROS2 bag；写盘线程与实时视觉流水线隔离。
class RosbagRecorder
{
public:
    explicit RosbagRecorder(const std::string & bag_uri, std::size_t queue_capacity = 16);
    ~RosbagRecorder();

    RosbagRecorder(const RosbagRecorder &) = delete;
    RosbagRecorder & operator=(const RosbagRecorder &) = delete;

    // 只复制 cv::Mat 引用和姿态；图像序列化、复制与 rosbag 写盘由后台线程完成。
    // 返回 false 表示记录器已停止接收，实时跟踪仍可继续。
    bool enqueue(
        const cv::Mat & image, const std::string & image_encoding,
        Clock::time_point image_timestamp,
        const Eigen::Quaterniond & gimbal_to_world);

    // 停止接收新帧，排空队列，等待写盘线程退出并关闭 bag；可重复调用。
    void close() noexcept;

    std::uint64_t frames_written() const;
    std::uint64_t frames_dropped() const;
    std::string error() const;

private:
    struct Packet
    {
        cv::Mat image;
        std::string image_encoding;
        std::int64_t timestamp_ns = 0;
        Eigen::Quaterniond gimbal_to_world = Eigen::Quaterniond::Identity();
    };

    void worker_loop() noexcept;
    void write_packet(const Packet & packet);
    void set_error_locked(const std::string & message);

    const std::size_t queue_capacity_;
    const Clock::time_point steady_origin_;
    const std::int64_t system_origin_ns_;
    rosbag2_cpp::Writer writer_;

    // mutex 只保护有界队列、生命周期标志和计数；序列化、磁盘 I/O 与 join 都在锁外。
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<Packet> queue_;
    bool accepting_ = true;
    bool stopping_ = false;
    bool closed_ = false;
    std::uint64_t frames_written_ = 0;
    std::uint64_t frames_dropped_ = 0;
    std::string error_;
    std::thread worker_;
};
}  // namespace real_auto_aim
