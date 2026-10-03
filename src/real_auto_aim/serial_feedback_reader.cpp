#include "src/real_auto_aim/serial_feedback_reader.hpp"

#include <stdexcept>
#include <utility>

namespace real_auto_aim
{
SerialFeedbackReader::SerialFeedbackReader(
    std::unique_ptr<io::srm_auto_aim::SrmAutoAimTransport> transport,
    std::chrono::milliseconds max_gap,
    std::chrono::milliseconds wait_after_image)
    : transport_(std::move(transport)), buffer_(256, max_gap),
      wait_after_image_(wait_after_image)
{
    if (!transport_ || wait_after_image_.count() <= 0) {
        throw std::invalid_argument("Invalid serial feedback reader configuration");
    }
    // 全部共享成员构造完成后启动线程，避免线程访问半初始化的对象。
    worker_ = std::thread(&SerialFeedbackReader::receive_loop, this);
}

SerialFeedbackReader::~SerialFeedbackReader() { stop(); }

void SerialFeedbackReader::receive_loop() noexcept
{
    try {
        while (!stop_requested_.load()) {
            const auto frames = transport_->poll_feedback();
            const auto received_at = Clock::now();
            if (frames.empty()) {
                std::unique_lock<std::mutex> lock(mutex_);
                changed_.wait_for(lock, std::chrono::milliseconds(2), [this] {
                    return stop_requested_.load();
                });
                continue;
            }
            // 同批反馈没有独立采样时刻；只保留最后一帧，不能伪造批内时间戳。
            {
                std::lock_guard<std::mutex> lock(mutex_);
                buffer_.push(received_at, frames.back());
            }
            changed_.notify_all();
        }
    } catch (...) {
        // 后台错误必须传给采集线程，不能继续使用最后一次有效反馈。
        {
            std::lock_guard<std::mutex> lock(mutex_);
            failure_ = std::current_exception();
            state_ = State::Failed;
        }
        changed_.notify_all();
    }
}

std::optional<MatchedFeedback> SerialFeedbackReader::sample_at(Clock::time_point image_time)
{
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock, wait_after_image_, [this, image_time] {
        return state_ != State::Running || buffer_.reaches(image_time);
    });
    if (failure_) std::rethrow_exception(failure_);
    if (state_ != State::Running) {
        throw std::runtime_error("Serial feedback reader is stopped");
    }
    return buffer_.match(image_time, Clock::now());
}

void SerialFeedbackReader::stop() noexcept
{
    stop_requested_.store(true);
    changed_.notify_all();
    if (worker_.joinable()) worker_.join();
    transport_->close();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = State::Stopped;
    }
    changed_.notify_all();
}
}  // namespace real_auto_aim
