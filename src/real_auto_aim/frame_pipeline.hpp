#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include "io/camera.hpp"
#include "src/real_auto_aim/frame_processor.hpp"

namespace real_auto_aim
{
// 一帧的图像、时间和检测结果一起过队列，避免姿态匹配与另一帧串线。
struct FrameInput
{
    cv::Mat image;
    cv::Mat raw_bayer;
    std::string raw_bayer_encoding;
    io::FrameTiming timing;
    Json sample;
    FrameDetections detections;
    std::optional<Clock::time_point> image_timestamp;
    std::string skip_reason;
    std::uint64_t captured_frame = 0;
    Clock::time_point queued_at{};
};

// 主线程独占写盘与界面；投影角点由算法线程预先算好，不跨线程读取 Solver。
struct FrameOutput
{
    cv::Mat image;
    cv::Mat raw_bayer;
    std::string raw_bayer_encoding;
    io::FrameTiming timing;
    Json sample;
    std::optional<Clock::time_point> image_timestamp;
    std::optional<FrameProcessResult> result;
    std::optional<MatchedFeedback> feedback;
    std::vector<cv::Point2f> current_polygon;
    std::vector<cv::Point2f> future_polygon;
    std::string display_reason;
    std::uint64_t captured_frame = 0;
};

enum class CaptureKind { Frame, Retry, Done };
struct CaptureResult
{
    CaptureKind kind = CaptureKind::Retry;
    FrameInput frame;
};

// 两个工作线程分别独占采集/检测和有状态算法。调用方必须在销毁串口前 join。
// 输入只缓存 1 帧、结果只缓存 2 帧；队列满时背压，结果等待超过 timeout 则
// 清除控制目标并将异常传给主线程，不能让无 TTL 的保持命令因写盘卡住而持续发送。
class FramePipeline
{
public:
    using Capture = std::function<CaptureResult()>;
    using Process = std::function<FrameOutput(FrameInput &&, const std::atomic<bool> &)>;
    using Abort = std::function<void()>;

    FramePipeline(
        Capture capture, Process process, Abort abort,
        std::chrono::milliseconds output_timeout,
        std::size_t input_capacity = 1, std::size_t output_capacity = 2,
        std::optional<std::chrono::milliseconds> source_timeout = std::nullopt);
    ~FramePipeline();

    FramePipeline(const FramePipeline &) = delete;
    FramePipeline & operator=(const FramePipeline &) = delete;

    void start();
    // 用户退出或故障时取消未处理帧；正常采满帧数时采集回调返回 Done，算法自然排空。
    void request_stop() noexcept;
    bool try_pop_output(FrameOutput & output);
    void wait_for_output(std::chrono::milliseconds timeout);
    bool finished() const;
    void join() noexcept;
    void rethrow_if_failed() const;

private:
    void capture_loop() noexcept;
    void algorithm_loop() noexcept;
    void fail(std::exception_ptr error) noexcept;

    Capture capture_;
    Process process_;
    Abort abort_;
    const std::chrono::milliseconds output_timeout_;
    const std::size_t input_capacity_;
    const std::size_t output_capacity_;
    // 只在首帧到达后启用断流检测；相机启动期间串口只发送零命令。
    const std::optional<std::chrono::milliseconds> source_timeout_;
    // mutex 只保护队列、完成标志及异常；采集、推理、串口、写盘和 join 都在锁外。
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<FrameInput> input_;
    std::deque<FrameOutput> output_;
    bool started_ = false;
    bool capture_done_ = false;
    bool algorithm_done_ = false;
    std::exception_ptr failure_;
    std::atomic<bool> stop_requested_{false};
    std::thread capture_thread_;
    std::thread algorithm_thread_;
};
}  // namespace real_auto_aim
