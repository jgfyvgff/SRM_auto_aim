#include "src/real_auto_aim/serial_feedback_reader.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace real_auto_aim
{
SerialFeedbackReader::SerialFeedbackReader(
    std::unique_ptr<io::srm_auto_aim::SrmAutoAimTransport> transport,
    std::chrono::milliseconds max_gap,
    std::chrono::milliseconds wait_after_image,
    bool enable_tx,
    std::chrono::milliseconds tx_period,
    std::chrono::milliseconds tx_ttl)
    : transport_(std::move(transport)), buffer_(256, max_gap),
      wait_after_image_(wait_after_image), enable_tx_(enable_tx),
      tx_period_(tx_period), tx_ttl_(tx_ttl)
{
    if (!transport_ || wait_after_image_.count() <= 0 || tx_period_.count() <= 0 ||
        tx_ttl_.count() <= 0) {
        throw std::invalid_argument("Invalid serial feedback reader configuration");
    }
    // 全部共享成员构造完成后启动线程，避免线程访问半初始化的对象。
    worker_ = std::thread(&SerialFeedbackReader::receive_loop, this);
}

SerialFeedbackReader::~SerialFeedbackReader() { stop(); }

void SerialFeedbackReader::receive_loop() noexcept
{
    try {
        auto next_tx = Clock::now();
        while (!stop_requested_.load()) {
            // 发送与接收必须由同一线程访问 transport，避免 USB CDC 读写交叉破坏协议状态。
            // 普通跟踪命令过期后回退零命令；显式保持角度用于暂时无目标，
            // 直到调用方清空或恢复普通跟踪，不能把过期的跟踪命令自动当保持命令。
            const auto before_io = Clock::now();
            if (enable_tx_ && before_io >= next_tx) {
                io::srm_auto_aim::CommandFrame command{};
                bool has_fresh_command = false;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (pending_command_ &&
                        (hold_command_ || before_io - command_updated_at_ <= tx_ttl_)) {
                        command = *pending_command_;
                        has_fresh_command = true;
                    }
                }
                transport_->send(command);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (has_fresh_command) {
                        ++tx_stats_.target_frames;
                    } else {
                        ++tx_stats_.zero_frames;
                    }
                }
                // 发送节拍对齐绝对时间表，而不是"本次发送时刻 + 周期"：后者会把每次串口读
                // 的相位误差逐周期累加，实测把 20ms 周期拖成 21.4ms。但线程被长期拖住后
                // 也不能连发补偿，落后超过一个周期就按当前时刻重新对齐。
                next_tx += tx_period_;
                if (before_io - next_tx > tx_period_) {
                    next_tx = before_io + tx_period_;
                }
            }

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

        // 正常停止前再发一次零命令。它不能覆盖 SIGKILL、USB 拔出等异常终止，
        // 但可以避免普通退出路径把最后一个非零视觉目标留在下位机缓存中。
        if (enable_tx_) {
            transport_->send(io::srm_auto_aim::CommandFrame{});
            std::lock_guard<std::mutex> lock(mutex_);
            ++tx_stats_.zero_frames;
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

std::optional<LatestGimbalMotion> SerialFeedbackReader::latest_motion()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
    if (state_ != State::Running) {
        throw std::runtime_error("Serial feedback reader is stopped");
    }
    return buffer_.latest_motion(Clock::now());
}

void SerialFeedbackReader::set_command(const io::srm_auto_aim::CommandFrame & command)
{
    if (!std::isfinite(command.yaw_deg) || !std::isfinite(command.pitch_deg) ||
        command.fire_flag != 0) {
        throw std::invalid_argument("Only finite no-fire commands may be sent");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
    if (state_ != State::Running) {
        throw std::runtime_error("Serial feedback reader is stopped");
    }
    pending_command_ = command;
    hold_command_ = false;
    command_updated_at_ = Clock::now();
}

void SerialFeedbackReader::set_hold_command(
    const io::srm_auto_aim::CommandFrame & command)
{
    if (!std::isfinite(command.yaw_deg) || !std::isfinite(command.pitch_deg) ||
        command.fire_flag != 0) {
        throw std::invalid_argument("Only finite no-fire hold commands may be sent");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
    if (state_ != State::Running) {
        throw std::runtime_error("Serial feedback reader is stopped");
    }
    pending_command_ = command;
    hold_command_ = true;
    command_updated_at_ = Clock::now();
}

void SerialFeedbackReader::clear_command()
{
    std::lock_guard<std::mutex> lock(mutex_);
    pending_command_.reset();
    hold_command_ = false;
    command_updated_at_ = Clock::now();
}

TxStats SerialFeedbackReader::tx_stats()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return tx_stats_;
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
