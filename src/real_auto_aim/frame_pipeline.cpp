#include "src/real_auto_aim/frame_pipeline.hpp"

#include <stdexcept>
#include <utility>

namespace real_auto_aim
{
FramePipeline::FramePipeline(
    Capture capture, Process process, Abort abort,
    std::chrono::milliseconds output_timeout,
    std::size_t input_capacity, std::size_t output_capacity,
    std::optional<std::chrono::milliseconds> source_timeout)
    : capture_(std::move(capture)), process_(std::move(process)), abort_(std::move(abort)),
      output_timeout_(output_timeout), input_capacity_(input_capacity),
      output_capacity_(output_capacity), source_timeout_(source_timeout)
{
    if (!capture_ || !process_ || !abort_ || output_timeout_.count() <= 0 ||
        input_capacity_ == 0 || output_capacity_ == 0 ||
        (source_timeout_ && source_timeout_->count() <= 0)) {
        throw std::invalid_argument("Invalid frame pipeline configuration");
    }
}

FramePipeline::~FramePipeline()
{
    request_stop();
    join();
}

void FramePipeline::start()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (started_) throw std::logic_error("Frame pipeline already started");
        started_ = true;
    }
    try {
        algorithm_thread_ = std::thread(&FramePipeline::algorithm_loop, this);
        capture_thread_ = std::thread(&FramePipeline::capture_loop, this);
    } catch (...) {
        request_stop();
        join();
        throw;
    }
}

void FramePipeline::request_stop() noexcept
{
    // 先禁用命令邮箱，再唤醒队列；并发算法即使刚进入发送路径也不能重新提交目标。
    if (!stop_requested_.exchange(true)) {
        try { abort_(); } catch (...) { /* 停止路径不得抛出异常。 */ }
    }
    changed_.notify_all();
}

bool FramePipeline::try_pop_output(FrameOutput & output)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (output_.empty()) return false;
    output = std::move(output_.front());
    output_.pop_front();
    changed_.notify_all();
    return true;
}

void FramePipeline::wait_for_output(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock, timeout, [this] {
        return !output_.empty() || (capture_done_ && algorithm_done_) || failure_;
    });
}

bool FramePipeline::finished() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return capture_done_ && algorithm_done_ && output_.empty();
}

void FramePipeline::join() noexcept
{
    // join 在锁外；算法线程可能正等待主线程排空结果，调用方应先消费输出。
    if (capture_thread_.joinable()) capture_thread_.join();
    if (algorithm_thread_.joinable()) algorithm_thread_.join();
}

void FramePipeline::rethrow_if_failed() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (failure_) std::rethrow_exception(failure_);
}

void FramePipeline::fail(std::exception_ptr error) noexcept
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!failure_) failure_ = error;
    }
    request_stop();
}

void FramePipeline::capture_loop() noexcept
{
    try {
        std::optional<std::chrono::steady_clock::time_point> last_frame_at;
        while (!stop_requested_.load()) {
            auto captured = capture_();
            if (captured.kind == CaptureKind::Retry) {
                // 有过有效帧后断流，必须撤销无期限保持的旧角度；启动前无帧只发送零命令。
                if (source_timeout_ && last_frame_at &&
                    std::chrono::steady_clock::now() - *last_frame_at > *source_timeout_) {
                    throw std::runtime_error("Camera source stalled; TX command inhibited");
                }
                continue;
            }
            if (captured.kind == CaptureKind::Done) break;
            last_frame_at = std::chrono::steady_clock::now();
            std::unique_lock<std::mutex> lock(mutex_);
            changed_.wait(lock, [this] {
                return stop_requested_.load() || input_.size() < input_capacity_;
            });
            if (stop_requested_.load()) break;
            input_.push_back(std::move(captured.frame));
            changed_.notify_all();
        }
    } catch (...) {
        fail(std::current_exception());
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        capture_done_ = true;
    }
    changed_.notify_all();
}

void FramePipeline::algorithm_loop() noexcept
{
    try {
        while (true) {
            FrameInput input;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                changed_.wait(lock, [this] {
                    return stop_requested_.load() || !input_.empty() || capture_done_;
                });
                if (stop_requested_.load() || (capture_done_ && input_.empty())) break;
                input = std::move(input_.front());
                input_.pop_front();
                changed_.notify_all();
            }
            auto output = process_(std::move(input), stop_requested_);
            std::unique_lock<std::mutex> lock(mutex_);
            const bool ready = changed_.wait_for(lock, output_timeout_, [this] {
                return stop_requested_.load() || output_.size() < output_capacity_;
            });
            if (!ready) {
                lock.unlock();
                fail(std::make_exception_ptr(
                    std::runtime_error("Frame result sink stalled; TX command inhibited")));
                break;
            }
            if (stop_requested_.load()) break;
            output_.push_back(std::move(output));
            changed_.notify_all();
        }
    } catch (...) {
        if (!stop_requested_.load()) fail(std::current_exception());
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        algorithm_done_ = true;
    }
    changed_.notify_all();
}
}  // namespace real_auto_aim
