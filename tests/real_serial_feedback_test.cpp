#include "src/real_auto_aim/serial_feedback_reader.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
using io::srm_auto_aim::Bytes;
using io::srm_auto_aim::SrmAutoAimTransport;
using real_auto_aim::Clock;
using real_auto_aim::SerialFeedbackReader;

void require(bool condition, const char * message)
{
    if (!condition) throw std::runtime_error(message);
}

// Fake 与真实串口共用传输层协议解析，但不依赖设备或系统调度的固定睡眠时长。
class ScriptedStream final : public io::srm_auto_aim::ByteStream
{
public:
    ScriptedStream(Bytes first, Bytes second)
        : first_(std::move(first)), second_(std::move(second)) {}

    std::size_t read(std::uint8_t * data, std::size_t size) override
    {
        std::unique_lock<std::mutex> lock(mutex_);
        ++read_count_;
        changed_.notify_all();
        if (read_count_ == 1) {
            return copy_chunk(first_, 0, first_.size() / 2, data, size);
        }
        if (read_count_ == 2) {
            return copy_chunk(first_, first_.size() / 2, first_.size(), data, size);
        }
        if (read_count_ == 3) {
            // 第二帧由测试释放，使图像时间严格落在两次接收之间。
            changed_.wait_for(lock, std::chrono::seconds(1), [this] {
                return release_second_;
            });
            return release_second_
                       ? copy_chunk(second_, 0, second_.size(), data, size)
                       : 0;
        }
        return 0;
    }

    std::size_t write(const std::uint8_t *, std::size_t size) override
    {
        ++write_count;
        return size;
    }

    void close() noexcept override { ++close_count; }

    bool wait_for_second_frame_read()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return changed_.wait_for(lock, std::chrono::seconds(1), [this] {
            return read_count_ >= 3;
        });
    }

    void release_second_frame_read()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            release_second_ = true;
        }
        changed_.notify_all();
    }

    std::atomic<int> write_count{0};
    std::atomic<int> close_count{0};

private:
    static std::size_t copy_chunk(
        const Bytes & frame, std::size_t begin, std::size_t end,
        std::uint8_t * data, std::size_t size)
    {
        require(size >= end - begin, "Fake read buffer is too small");
        std::copy(frame.begin() + begin, frame.begin() + end, data);
        return end - begin;
    }

    Bytes first_;
    Bytes second_;
    std::mutex mutex_;
    std::condition_variable changed_;
    int read_count_ = 0;
    bool release_second_ = false;
};

class EmptyStream final : public io::srm_auto_aim::ByteStream
{
public:
    std::size_t read(std::uint8_t *, std::size_t) override { return 0; }
    std::size_t write(const std::uint8_t *, std::size_t) override
    {
        throw std::runtime_error("Read-only test unexpectedly sent a command");
    }
    void close() noexcept override {}
};

class CapturingStream final : public io::srm_auto_aim::ByteStream
{
public:
    std::size_t read(std::uint8_t *, std::size_t) override { return 0; }

    std::size_t write(const std::uint8_t * data, std::size_t size) override
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            writes_.emplace_back(data, data + size);
        }
        changed_.notify_all();
        return size;
    }

    void close() noexcept override
    {
        ++close_count;
        changed_.notify_all();
    }

    bool wait_for_write_count(std::size_t count)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return changed_.wait_for(lock, std::chrono::seconds(1), [this, count] {
            return writes_.size() >= count;
        });
    }

    std::size_t write_count()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return writes_.size();
    }

    Bytes latest_write()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return writes_.empty() ? Bytes{} : writes_.back();
    }

    std::atomic<int> close_count{0};

private:
    std::mutex mutex_;
    std::condition_variable changed_;
    std::vector<Bytes> writes_;
};

class FailingStream final : public io::srm_auto_aim::ByteStream
{
public:
    std::size_t read(std::uint8_t *, std::size_t) override
    {
        throw std::runtime_error("fake read failure");
    }
    std::size_t write(const std::uint8_t *, std::size_t) override
    {
        throw std::runtime_error("Read-only test unexpectedly sent a command");
    }
    void close() noexcept override {}
};

// 读阻塞成本固定的假流：无反馈，只用来观察发送节拍。
// 7ms 读 + 2ms 空等使循环粒度约 9ms，与 20ms 发送周期错位：若发送时刻写成
// "本次读之后再加一个周期"，每个周期都会带上一个相位（约 27ms/次），
// 40 个周期累计漂移约 280ms，足以被下面的判据抓住。
class SlowReadStream final : public io::srm_auto_aim::ByteStream
{
public:
    static constexpr std::chrono::milliseconds kReadCost{7};

    std::size_t read(std::uint8_t *, std::size_t) override
    {
        std::this_thread::sleep_for(kReadCost);
        return 0;
    }

    std::size_t write(const std::uint8_t *, std::size_t size) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        write_times_.push_back(Clock::now());
        return size;
    }

    void close() noexcept override {}

    std::size_t write_count()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return write_times_.size();
    }

    Clock::time_point write_time(std::size_t index)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return write_times_.at(index);
    }

private:
    std::mutex mutex_;
    std::vector<Clock::time_point> write_times_;
};

void test_matching_and_stop()
{
    using namespace std::chrono_literals;
    const auto first = io::srm_auto_aim::encode(
        io::srm_auto_aim::FeedbackFrame{170.0F, 0.0F, 0.0F, 0, 3, 20.0F});
    const auto second = io::srm_auto_aim::encode(
        io::srm_auto_aim::FeedbackFrame{-170.0F, 0.0F, 0.0F, 0, 3, 22.0F});
    auto stream = std::make_unique<ScriptedStream>(first, second);
    auto * fake = stream.get();
    auto transport = std::make_unique<SrmAutoAimTransport>(std::move(stream));
    SerialFeedbackReader reader(std::move(transport), 1000ms, 500ms);
    require(fake->wait_for_second_frame_read(), "Split feedback frame was not received");

    const auto image_time = Clock::now();
    fake->release_second_frame_read();
    const auto matched = reader.sample_at(image_time);
    require(matched.has_value(), "Image time was not bracketed by feedback");
    require(matched->gimbal_to_world.coeffs().allFinite(), "Matched pose is invalid");
    require(matched->feedback.bullet_speed_mps >= 20.0F &&
                matched->feedback.bullet_speed_mps <= 22.0F,
            "Bullet speed was not preserved");
    reader.stop();
    reader.stop();
    require(fake->write_count == 0, "Read-only receiver sent a command");
    require(fake->close_count == 1, "Serial transport was not closed exactly once");
}

void test_no_data_and_background_failure()
{
    using namespace std::chrono_literals;
    {
        auto transport = std::make_unique<SrmAutoAimTransport>(
            std::make_unique<EmptyStream>());
        SerialFeedbackReader reader(std::move(transport), 100ms, 10ms);
        require(!reader.sample_at(Clock::now()).has_value(),
                "Missing feedback was accepted");
        reader.stop();
    }
    {
        auto transport = std::make_unique<SrmAutoAimTransport>(
            std::make_unique<FailingStream>());
        SerialFeedbackReader reader(std::move(transport), 100ms, 100ms);
        bool reported = false;
        try {
            reader.sample_at(Clock::now());
        } catch (const std::runtime_error & error) {
            reported = std::string(error.what()) == "fake read failure";
        }
        require(reported, "Background read failure was not reported");
        reader.stop();
    }
}

void test_tx_is_no_fire_and_expires_to_zero()
{
    using namespace std::chrono_literals;
    auto stream = std::make_unique<CapturingStream>();
    auto * fake = stream.get();
    auto transport = std::make_unique<SrmAutoAimTransport>(std::move(stream));
    SerialFeedbackReader reader(
        std::move(transport), 100ms, 10ms, true, 5ms, 20ms);

    reader.set_command({12.5F, -3.0F, 0});
    require(fake->wait_for_write_count(1), "TX worker did not write a command");

    bool observed_command = false;
    const auto command_deadline = Clock::now() + 500ms;
    while (Clock::now() < command_deadline) {
        auto bytes = fake->latest_write();
        io::srm_auto_aim::CommandFrame command;
        if (io::srm_auto_aim::decode(bytes, command) &&
            std::abs(command.yaw_deg - 12.5F) < 1e-4F &&
            std::abs(command.pitch_deg + 3.0F) < 1e-4F) {
            require(command.fire_flag == 0, "TX worker sent a fire command");
            observed_command = true;
            break;
        }
        std::this_thread::sleep_for(1ms);
    }
    require(observed_command, "Requested TX command was not observed");

    // 不主动清除，验证命令超过 TTL 后后台线程会自动退回零命令。
    std::this_thread::sleep_for(30ms);
    const auto writes_before_expiry = fake->write_count();
    require(
        fake->wait_for_write_count(writes_before_expiry + 1),
        "TX worker did not continue after command expiry");

    bool observed_zero = false;
    const auto zero_deadline = Clock::now() + 500ms;
    while (Clock::now() < zero_deadline) {
        auto bytes = fake->latest_write();
        io::srm_auto_aim::CommandFrame command;
        if (io::srm_auto_aim::decode(bytes, command) && command.yaw_deg == 0.0F &&
            command.pitch_deg == 0.0F && command.fire_flag == 0) {
            observed_zero = true;
            break;
        }
        std::this_thread::sleep_for(1ms);
    }
    require(observed_zero, "Cleared TX command did not become a zero command");
    const auto tx_stats = reader.tx_stats();
    require(tx_stats.target_frames > 0, "Successful target writes were not counted");
    require(tx_stats.zero_frames > 0, "Successful zero writes were not counted");

    reader.set_command({12.5F, -3.0F, 0});
    const auto second_command_deadline = Clock::now() + 500ms;
    while (Clock::now() < second_command_deadline) {
        auto bytes = fake->latest_write();
        io::srm_auto_aim::CommandFrame command;
        if (io::srm_auto_aim::decode(bytes, command) &&
            std::abs(command.yaw_deg - 12.5F) < 1e-4F &&
            std::abs(command.pitch_deg + 3.0F) < 1e-4F) {
            break;
        }
        std::this_thread::sleep_for(1ms);
    }
    reader.clear_command();

    reader.stop();
    require(fake->close_count == 1, "TX transport was not closed exactly once");
}

void test_hold_survives_ttl_and_returns_to_normal_expiry()
{
    using namespace std::chrono_literals;
    auto stream = std::make_unique<CapturingStream>();
    auto * fake = stream.get();
    auto transport = std::make_unique<SrmAutoAimTransport>(std::move(stream));
    SerialFeedbackReader reader(
        std::move(transport), 100ms, 10ms, true, 5ms, 20ms);

    reader.set_hold_command({12.5F, -3.0F, 0});
    // 检查超过普通命令 TTL 后真正写出的帧，而非只检查命令邮箱。
    std::this_thread::sleep_for(30ms);
    auto before_write = fake->write_count();
    require(fake->wait_for_write_count(before_write + 1),
            "Hold command was not sent after normal TTL");
    io::srm_auto_aim::CommandFrame command;
    require(io::srm_auto_aim::decode(fake->latest_write(), command),
            "Hold command could not be decoded");
    require(std::abs(command.yaw_deg - 12.5F) < 1e-4F &&
                std::abs(command.pitch_deg + 3.0F) < 1e-4F && command.fire_flag == 0,
            "Hold mode sent a reset or fire command after normal TTL");

    reader.set_command({-4.0F, 1.0F, 0});
    bool observed_normal = false;
    const auto normal_deadline = Clock::now() + 500ms;
    while (Clock::now() < normal_deadline) {
        if (io::srm_auto_aim::decode(fake->latest_write(), command) &&
            std::abs(command.yaw_deg + 4.0F) < 1e-4F &&
            std::abs(command.pitch_deg - 1.0F) < 1e-4F && command.fire_flag == 0) {
            observed_normal = true;
            break;
        }
        std::this_thread::sleep_for(1ms);
    }
    require(observed_normal, "Normal command was not sent after hold mode");
    std::this_thread::sleep_for(30ms);
    before_write = fake->write_count();
    require(fake->wait_for_write_count(before_write + 1),
            "Normal command did not expire after hold mode");
    require(io::srm_auto_aim::decode(fake->latest_write(), command) &&
                command.yaw_deg == 0.0F && command.pitch_deg == 0.0F &&
                command.fire_flag == 0,
            "Normal command remained in hold mode after TTL");

    reader.set_hold_command({12.5F, -3.0F, 0});
    reader.clear_command();
    before_write = fake->write_count();
    require(fake->wait_for_write_count(before_write + 1),
            "Clearing a hold command did not send a zero command");
    require(io::srm_auto_aim::decode(fake->latest_write(), command) &&
                command.yaw_deg == 0.0F && command.pitch_deg == 0.0F &&
                command.fire_flag == 0,
            "Cleared hold command was still active");
    reader.stop();
}

// 发送周期必须跟绝对时间表对齐，不能把每次串口读的耗时相位逐周期累加到间隔里。
void test_tx_period_does_not_accumulate_read_phase()
{
    using namespace std::chrono_literals;
    auto stream = std::make_unique<SlowReadStream>();
    auto * fake = stream.get();
    auto transport = std::make_unique<SrmAutoAimTransport>(std::move(stream));
    // 空命令邮箱同样按周期发送零命令，节拍测量不依赖视觉命令是否新鲜。
    SerialFeedbackReader reader(std::move(transport), 100ms, 10ms, true, 20ms, 100ms);

    constexpr std::size_t kSends = 41;
    const auto deadline = Clock::now() + 3s;
    while (fake->write_count() < kSends && Clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    const auto sends = fake->write_count();
    require(sends >= kSends, "TX worker did not keep sending on the requested period");

    const auto span = fake->write_time(sends - 1) - fake->write_time(0);
    const auto intervals = static_cast<std::chrono::milliseconds::rep>(sends - 1);
    const auto nominal = std::chrono::milliseconds(intervals * 20);
    // 允许首末两次发送各自被循环粒度推迟一次，但不得逐周期累加读的相位。
    require(span <= nominal + 25ms,
            "TX period accumulated read phase instead of following an absolute schedule");
    reader.stop();
}
}  // namespace

int main()
{
    test_matching_and_stop();
    test_no_data_and_background_failure();
    test_tx_is_no_fire_and_expires_to_zero();
    test_hold_survives_ttl_and_returns_to_normal_expiry();
    test_tx_period_does_not_accumulate_read_phase();
    std::cout << "real_serial_feedback_test passed\n";
}
