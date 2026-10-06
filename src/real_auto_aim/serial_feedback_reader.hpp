#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "io/srm_auto_aim_transport.hpp"
#include "src/real_auto_aim/feedback_buffer.hpp"

namespace real_auto_aim
{
// 串口线程成功写入的帧数；目标帧表示命令邮箱有效，不代表下位机已经执行。
struct TxStats
{
    std::uint64_t target_frames = 0;
    std::uint64_t zero_frames = 0;
};

// 独占串口的反馈/命令协调器：后台线程收反馈并按固定周期发送最新命令。
// transport 的 read 必须有界返回；收到停止请求后线程补发零命令并退出，join 后关闭串口。
class SerialFeedbackReader
{
public:
    explicit SerialFeedbackReader(
        std::unique_ptr<io::srm_auto_aim::SrmAutoAimTransport> transport,
        std::chrono::milliseconds max_gap = std::chrono::milliseconds(100),
        std::chrono::milliseconds wait_after_image = std::chrono::milliseconds(30),
        bool enable_tx = false,
        std::chrono::milliseconds tx_period = std::chrono::milliseconds(20),
        std::chrono::milliseconds tx_ttl = std::chrono::milliseconds(100));
    ~SerialFeedbackReader();

    SerialFeedbackReader(const SerialFeedbackReader &) = delete;
    SerialFeedbackReader & operator=(const SerialFeedbackReader &) = delete;

    // 等待图像后的反馈；缺帧或过期返回空，后台串口错误重新抛给主线程。
    std::optional<MatchedFeedback> sample_at(Clock::time_point image_time);

    // 规划诊断从同一有界缓存获取最新云台状态；不足两帧或反馈过期返回空。
    std::optional<LatestGimbalMotion> latest_motion();

    // 设置最新的无开火云台目标。只保留最后一条命令，避免视觉处理变慢时堆积旧目标。
    // 当发送未启用时仅记录请求，不触碰串口；调用方可用同一套代码做只读诊断。
    void set_command(const io::srm_auto_aim::CommandFrame & command);

    // 清除当前目标。发送模式下后台线程会按周期发零命令，尽快使视觉目标失效。
    void clear_command();

    // 获取成功写入串口的累计计数；只统计 transport::send 正常返回的帧。
    TxStats tx_stats();

    // 由拥有者线程调用；重复停止安全。停止后不能再调用 sample_at。
    void stop() noexcept;

private:
    enum class State { Running, Failed, Stopped };
    void receive_loop() noexcept;

    std::unique_ptr<io::srm_auto_aim::SrmAutoAimTransport> transport_;
    FeedbackBuffer buffer_;
    std::chrono::milliseconds wait_after_image_;
    const bool enable_tx_;
    const std::chrono::milliseconds tx_period_;
    const std::chrono::milliseconds tx_ttl_;
    // mutex 只保护缓存、命令邮箱、发送计数、状态和异常；串口 I/O 与 join 均在锁外。
    std::mutex mutex_;
    std::condition_variable changed_;
    State state_ = State::Running;
    std::exception_ptr failure_;
    std::optional<io::srm_auto_aim::CommandFrame> pending_command_;
    Clock::time_point command_updated_at_ = Clock::now();
    TxStats tx_stats_;
    std::atomic<bool> stop_requested_{false};
    std::thread worker_;
};
}  // namespace real_auto_aim
