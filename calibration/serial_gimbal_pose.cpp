#include "calibration/serial_gimbal_pose.hpp"

namespace calibration
{
namespace
{
auto open_transport(const io::srm_auto_aim::SerialConfig & config, const PosePolicy & policy)
{
    policy.validate();
    if (config.timeout_ms == 0 || config.timeout_ms > 100) {
        throw std::invalid_argument("标定串口读取超时应为 1~100 ms，保证停止有界");
    }
    return std::make_unique<io::srm_auto_aim::SrmAutoAimTransport>(config);
}
}

SerialGimbalPose::SerialGimbalPose(
    const io::srm_auto_aim::SerialConfig & config, PosePolicy policy)
    : SerialGimbalPose(open_transport(config, policy), policy) {}

SerialGimbalPose::SerialGimbalPose(
    std::unique_ptr<io::srm_auto_aim::SrmAutoAimTransport> transport, PosePolicy policy)
    : transport_(std::move(transport)), policy_(policy), buffer_(policy)
{
    if (!transport_) throw std::invalid_argument("串口姿态接收器需要有效 transport");
    // 所有共享成员初始化后才启动线程，避免回调访问半初始化对象。
    worker_ = std::thread(&SerialGimbalPose::receive_loop, this);
}

SerialGimbalPose::~SerialGimbalPose() { stop(); }

void SerialGimbalPose::receive_loop() noexcept
{
    try {
        while (!stop_requested_.load()) {
            const auto frames = transport_->poll_feedback();
            const auto received_at = PoseClock::now();
            if (frames.empty()) {
                std::unique_lock<std::mutex> lock(mutex_);
                changed_.wait_for(lock, std::chrono::milliseconds(2),
                    [this] { return stop_requested_.load(); });
                continue;
            }
            // 协议无源时间戳。同批仅保留最后一帧，不能伪造批内各帧采样时刻。
            const auto & frame = frames.back();
            const auto q = gimbal_ypr_degrees(frame.yaw_deg, frame.pitch_deg, frame.roll_deg);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                buffer_.push({received_at, q});
            }
            changed_.notify_all();
        }
    } catch (...) {
        // 错误跨线程传到 UI；即使此刻用户没有按保存，主循环也会检查它。
        std::lock_guard<std::mutex> lock(mutex_);
        failure_ = std::current_exception();
        state_ = State::Failed;
        changed_.notify_all();
    }
}

PoseMatch SerialGimbalPose::sample_at(PoseClock::time_point image_time)
{
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock, policy_.max_gap, [this, image_time] {
        return state_ != State::Running || buffer_.reaches(image_time);
    });
    if (failure_) std::rethrow_exception(failure_);
    if (state_ != State::Running) throw std::runtime_error("串口姿态接收器已停止");
    return buffer_.match(image_time, PoseClock::now());
}

void SerialGimbalPose::rethrow_if_failed() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
}

void SerialGimbalPose::stop() noexcept
{
    stop_requested_.store(true);
    changed_.notify_all();
    if (worker_.joinable()) worker_.join();
    transport_->close();
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = State::Stopped;
    changed_.notify_all();
}
}  // namespace calibration
