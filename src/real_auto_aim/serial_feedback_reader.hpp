#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

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

// 主机侧串口事件；时间使用同一 steady_clock，不能当作下位机采样时刻。
enum class WireEventKind { TxTarget, TxZero, RxFeedback };
struct WireEvent
{
    WireEventKind kind = WireEventKind::TxZero;
    std::int64_t started_ns = 0;
    std::int64_t completed_ns = 0;
    float yaw_deg = 0.0F;
    float pitch_deg = 0.0F;
    float roll_deg = 0.0F;
    std::int32_t mode = 0;
    std::int32_t color_raw = 0;
    float bullet_speed_mps = 0.0F;
    std::size_t batch_index = 0;
    std::size_t batch_frames = 1;
    std::uint64_t sequence = 0;
};
struct WireEventBatch
{
    std::vector<WireEvent> events;
    std::uint64_t dropped_total = 0;
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
        std::chrono::milliseconds tx_ttl = std::chrono::milliseconds(100),
        std::chrono::milliseconds hold_ttl = std::chrono::milliseconds(200),
        bool capture_wire_trace = false);
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

    // 暂时失去目标时保持最后的无开火角度；保持模式有独立的最长有效期。
    // 每帧须刷新，避免视觉线程卡住时无限发送旧角度；不能用于真实开火。
    void set_hold_command(const io::srm_auto_aim::CommandFrame & command);

    // 清除当前目标。发送模式下后台线程会按周期发零命令，尽快使视觉目标失效。
    void clear_command();

    // 退出或故障时禁止新的目标进入邮箱；重复调用安全，串口线程随后发送零命令。
    void inhibit_commands() noexcept;

    // 获取成功写入串口的累计计数；只统计 transport::send 正常返回的帧。
    TxStats tx_stats();
    // 取走有界事件缓存；仅复制内存，JSON 序列化由调用线程在锁外完成。
    WireEventBatch take_wire_events();

    // 由拥有者线程调用；重复停止安全。停止后不能再调用 sample_at。
    void stop() noexcept;

private:
    enum class State { Running, Failed, Stopped };
    void receive_loop() noexcept;
    void push_wire_event_locked(WireEvent event) noexcept;
    static constexpr std::size_t kWireEventCapacity = 1024;

    std::unique_ptr<io::srm_auto_aim::SrmAutoAimTransport> transport_;
    FeedbackBuffer buffer_;
    std::chrono::milliseconds wait_after_image_;
    const bool enable_tx_;
    const std::chrono::milliseconds tx_period_;
    const std::chrono::milliseconds tx_ttl_;
    const std::chrono::milliseconds hold_ttl_;
    const bool capture_wire_trace_;
    // mutex 只保护缓存、命令邮箱、发送计数、状态和异常；串口 I/O 与 join 均在锁外。
    std::mutex mutex_;
    std::condition_variable changed_;
    State state_ = State::Running;
    std::exception_ptr failure_;
    std::optional<io::srm_auto_aim::CommandFrame> pending_command_;
    bool hold_command_ = false;
    bool commands_inhibited_ = false;
    Clock::time_point command_updated_at_ = Clock::now();
    TxStats tx_stats_;
    // mutex 同时保护事件环；满时丢最旧事件并累加计数。
    std::array<WireEvent, kWireEventCapacity> wire_events_{};
    std::size_t wire_event_head_ = 0;
    std::size_t wire_event_size_ = 0;
    std::uint64_t wire_event_sequence_ = 0;
    std::uint64_t wire_event_dropped_total_ = 0;
    std::atomic<bool> stop_requested_{false};
    std::thread worker_;
};
}  // namespace real_auto_aim
