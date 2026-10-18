#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "src/real_auto_aim/frame_pipeline.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;
using real_auto_aim::CaptureKind;
using real_auto_aim::CaptureResult;
using real_auto_aim::FrameInput;
using real_auto_aim::FrameOutput;
using real_auto_aim::FramePipeline;

namespace
{
void require(bool condition, const char * message)
{
    if (!condition) throw std::runtime_error(message);
}

FrameInput input_with_id(int id)
{
    FrameInput input;
    input.sample["id"] = id;
    return input;
}

FrameOutput output_with_id(FrameInput && input)
{
    FrameOutput output;
    output.sample = std::move(input.sample);
    return output;
}

void test_ordered_drain()
{
    int next_id = 0;
    std::atomic<int> aborts{0};
    FramePipeline pipeline(
        [&] {
            if (next_id == 3) return CaptureResult{CaptureKind::Done, {}};
            return CaptureResult{CaptureKind::Frame, input_with_id(next_id++)};
        },
        [](FrameInput && input, const std::atomic<bool> &) {
            return output_with_id(std::move(input));
        },
        [&] { ++aborts; }, 100ms);
    pipeline.start();
    std::vector<int> ids;
    while (!pipeline.finished()) {
        FrameOutput output;
        if (pipeline.try_pop_output(output)) {
            ids.push_back(output.sample.at("id").get<int>());
        } else {
            pipeline.wait_for_output(10ms);
        }
    }
    pipeline.join();
    pipeline.rethrow_if_failed();
    require(ids == std::vector<int>{0, 1, 2}, "results are not ordered");
    require(aborts == 0, "normal completion aborted");
}

void test_capacity_and_repeated_stop()
{
    std::mutex mutex;
    std::condition_variable entered;
    std::condition_variable released;
    bool processing = false;
    bool release = false;
    std::atomic<int> captured{0};
    std::atomic<int> aborts{0};
    FramePipeline pipeline(
        [&] {
            const int id = captured++;
            return CaptureResult{CaptureKind::Frame, input_with_id(id)};
        },
        [&](FrameInput && input, const std::atomic<bool> &) {
            {
                std::unique_lock<std::mutex> lock(mutex);
                processing = true;
                entered.notify_one();
                released.wait(lock, [&] { return release; });
            }
            return output_with_id(std::move(input));
        },
        [&] { ++aborts; }, 100ms, 1, 1);
    pipeline.start();
    {
        std::unique_lock<std::mutex> lock(mutex);
        require(entered.wait_for(lock, 1s, [&] { return processing; }), "worker did not start");
    }
    // 一个正在处理、一个排队、一个生产者待入队；容量固定后不会继续采集。
    std::this_thread::sleep_for(20ms);
    require(captured <= 3, "capture queue exceeded its capacity");
    pipeline.request_stop();
    pipeline.request_stop();
    {
        std::lock_guard<std::mutex> lock(mutex);
        release = true;
    }
    released.notify_one();
    pipeline.join();
    pipeline.rethrow_if_failed();
    require(pipeline.finished(), "pipeline did not finish after stop");
    require(aborts == 1, "abort callback count mismatch");
}

void test_sink_stall_propagates()
{
    int next_id = 0;
    std::atomic<int> aborts{0};
    FramePipeline pipeline(
        [&] {
            if (next_id == 4) return CaptureResult{CaptureKind::Done, {}};
            return CaptureResult{CaptureKind::Frame, input_with_id(next_id++)};
        },
        [](FrameInput && input, const std::atomic<bool> &) {
            return output_with_id(std::move(input));
        },
        [&] { ++aborts; }, 30ms, 1, 1);
    pipeline.start();
    // 不消费结果，第二次写满输出队列会限时失败，停止回调负责清空控制邮箱。
    std::this_thread::sleep_for(100ms);
    pipeline.join();
    bool raised = false;
    try {
        pipeline.rethrow_if_failed();
    } catch (const std::runtime_error & error) {
        raised = std::string(error.what()).find("sink stalled") != std::string::npos;
    }
    require(raised, "worker error did not propagate");
    require(aborts == 1, "abort callback count mismatch");
}

void test_worker_error_propagates()
{
    bool captured = false;
    std::atomic<int> aborts{0};
    FramePipeline pipeline(
        [&] {
            if (captured) return CaptureResult{CaptureKind::Done, {}};
            captured = true;
            return CaptureResult{CaptureKind::Frame, input_with_id(1)};
        },
        [](FrameInput &&, const std::atomic<bool> &) -> FrameOutput {
            throw std::runtime_error("fake algorithm failure");
        },
        [&] { ++aborts; }, 100ms);
    pipeline.start();
    pipeline.join();
    bool raised = false;
    try {
        pipeline.rethrow_if_failed();
    } catch (const std::runtime_error & error) {
        raised = std::string(error.what()) == "fake algorithm failure";
    }
    require(raised, "worker error did not propagate");
    require(aborts == 1, "abort callback count mismatch");
}

void test_source_stall_after_first_frame()
{
    bool captured = false;
    std::atomic<int> aborts{0};
    FramePipeline pipeline(
        [&] {
            if (!captured) {
                captured = true;
                return CaptureResult{CaptureKind::Frame, input_with_id(1)};
            }
            std::this_thread::sleep_for(2ms);
            return CaptureResult{CaptureKind::Retry, {}};
        },
        [](FrameInput && input, const std::atomic<bool> &) {
            return output_with_id(std::move(input));
        },
        [&] { ++aborts; }, 100ms, 1, 2, 30ms);
    pipeline.start();
    pipeline.join();
    bool raised = false;
    try {
        pipeline.rethrow_if_failed();
    } catch (const std::runtime_error & error) {
        raised = std::string(error.what()).find("source stalled") != std::string::npos;
    }
    require(raised, "camera source stall was not reported");
    require(aborts == 1, "camera source stall did not inhibit TX");
}

void test_camera_queue_timeout()
{
    tools::ThreadSafeQueue<int> queue(1);
    int value = 0;
    require(!queue.pop_for(value, 1ms), "Empty camera queue did not time out");
    queue.push(7);
    require(queue.pop_for(value, 1ms) && value == 7,
            "Timed camera queue lost its next frame");
}
}  // namespace

int main()
{
    test_ordered_drain();
    test_capacity_and_repeated_stop();
    test_sink_stall_propagates();
    test_worker_error_propagates();
    test_source_stall_after_first_frame();
    test_camera_queue_timeout();
}
