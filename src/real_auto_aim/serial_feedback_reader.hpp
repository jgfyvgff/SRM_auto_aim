#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "io/srm_auto_aim_transport.hpp"
#include "src/real_auto_aim/feedback_buffer.hpp"

namespace real_auto_aim
{
// 独占串口的只读接收器：后台线程收反馈，主线程按图像时间取匹配结果。
// transport 的 read 必须有界返回；停止时先退出线程，再关闭串口。
class SerialFeedbackReader
{
public:
    explicit SerialFeedbackReader(
        std::unique_ptr<io::srm_auto_aim::SrmAutoAimTransport> transport,
        std::chrono::milliseconds max_gap = std::chrono::milliseconds(100),
        std::chrono::milliseconds wait_after_image = std::chrono::milliseconds(30));
    ~SerialFeedbackReader();

    SerialFeedbackReader(const SerialFeedbackReader &) = delete;
    SerialFeedbackReader & operator=(const SerialFeedbackReader &) = delete;

    // 等待图像后的反馈；缺帧或过期返回空，后台串口错误重新抛给主线程。
    std::optional<MatchedFeedback> sample_at(Clock::time_point image_time);

    // 由拥有者线程调用；重复停止安全。停止后不能再调用 sample_at。
    void stop() noexcept;

private:
    enum class State { Running, Failed, Stopped };
    void receive_loop() noexcept;

    std::unique_ptr<io::srm_auto_aim::SrmAutoAimTransport> transport_;
    FeedbackBuffer buffer_;
    std::chrono::milliseconds wait_after_image_;
    // mutex 只保护缓存、状态和异常；串口读与线程 join 均在锁外。
    std::mutex mutex_;
    std::condition_variable changed_;
    State state_ = State::Running;
    std::exception_ptr failure_;
    std::atomic<bool> stop_requested_{false};
    std::thread worker_;
};
}  // namespace real_auto_aim
