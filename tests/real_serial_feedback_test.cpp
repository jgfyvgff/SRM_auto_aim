#include "src/real_auto_aim/serial_feedback_reader.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

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
}  // namespace

int main()
{
    test_matching_and_stop();
    test_no_data_and_background_failure();
    std::cout << "real_serial_feedback_test passed\n";
}
