#include "calibration/serial_gimbal_pose.hpp"

#include <iostream>
#include <limits>
#include <string>

namespace
{
using namespace std::chrono_literals;
using namespace calibration;

void require(bool value, const char * message)
{
    if (!value) throw std::runtime_error(message);
}
template<class Function> void rejects(Function action, const char * message)
{
    bool rejected = false;
    try { action(); } catch (const std::exception &) { rejected = true; }
    require(rejected, message);
}

void test_buffer()
{
    PosePolicy policy{32, 100ms, 30ms, 0.5};
    const auto origin = PoseClock::time_point{} + 1s;
    GimbalPoseBuffer buffer(policy);
    rejects([&] { buffer.match(origin, origin); }, "empty buffer accepted");
    for (int i = 0; i <= 20; ++i) {
        // 跨越 +/-180 度时应以旋转距离判断稳定，不能直接相减欧拉角。
        buffer.push({origin + i * 10ms, gimbal_ypr_degrees(i % 2 ? -179.9 : 179.9, 0, 0)});
    }
    const auto match = buffer.match(origin + 155ms, origin + 200ms);
    require(match.spread_deg < 0.21, "angle wrapping broke stability");
    require(std::abs(match.bracket_ms - 10.0) < 1e-8, "wrong image bracket");
    require(match.q.angularDistance(gimbal_ypr_degrees(180, 0, 0)) < 1e-8, "wrong quaternion interpolation");
    rejects([&] { buffer.match(origin + 155ms, origin + 1s); }, "stale data accepted");
    rejects([&] { buffer.match(origin + 215ms, origin + 220ms); }, "extrapolation accepted");
    rejects([&] { buffer.push({origin + 200ms, Eigen::Quaterniond::Identity()}); }, "duplicate timestamp accepted");

    GimbalPoseBuffer moving(policy);
    for (int i = 0; i <= 20; ++i) moving.push({origin + i * 10ms, gimbal_ypr_degrees(i, 0, 0)});
    rejects([&] { moving.match(origin + 155ms, origin + 200ms); }, "moving pose accepted");
    GimbalPoseBuffer gap(policy);
    for (int i : {0, 1, 2, 3, 4, 5, 6, 14, 15, 16, 17}) {
        gap.push({origin + i * 10ms, Eigen::Quaterniond::Identity()});
    }
    rejects([&] { gap.match(origin + 155ms, origin + 170ms); }, "receive gap accepted");
    policy.capacity = 4;
    GimbalPoseBuffer bounded(policy);
    for (int i = 0; i <= 20; ++i) bounded.push({origin + i * 10ms, Eigen::Quaterniond::Identity()});
    require(bounded.size() == 4, "unbounded pose queue");
    rejects([&] { bounded.match(origin + 195ms, origin + 200ms); }, "evicted history reused");
    rejects([] { GimbalPoseBuffer bad(PosePolicy{2, 100ms, 30ms, 0.5}); }, "invalid capacity accepted");
    rejects([] { gimbal_ypr_degrees(std::numeric_limits<double>::quiet_NaN(), 0, 0); }, "NaN accepted");
    rejects([&] { bounded.push({origin + 210ms, Eigen::Quaterniond(0, 0, 0, 0)}); }, "zero quaternion accepted");
    const auto yaw90 = gimbal_ypr_degrees(90, 0, 0);
    require((yaw90 * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitY()).norm() < 1e-10,
        "gimbal to world direction incorrect");
}

// 与真实 USB CDC 共用 ByteStream 契约，状态独立存活以检查销毁后的资源释放。
struct FakeState
{
    std::mutex mutex;
    std::condition_variable changed;
    io::srm_auto_aim::Bytes bytes;
    int reads = 0;
    int closes = 0;
    int writes = 0;
    bool fail = false;
};

class FakeStream final : public io::srm_auto_aim::ByteStream
{
public:
    explicit FakeStream(std::shared_ptr<FakeState> state) : state_(std::move(state)) {}
    std::size_t read(std::uint8_t * data, std::size_t size) override
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        ++state_->reads;
        state_->changed.notify_all();
        if (state_->fail) throw std::runtime_error("fake device disconnected");
        require(state_->closes == 0, "read after close");
        // 有意分包，验证接收组件仍走真实协议解码而非直接注入四元数。
        const auto count = std::min({size, state_->bytes.size(), std::size_t(7)});
        std::copy_n(state_->bytes.begin(), count, data);
        state_->bytes.erase(state_->bytes.begin(), state_->bytes.begin() + count);
        return count;
    }
    std::size_t write(const std::uint8_t *, std::size_t size) override
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        ++state_->writes;
        return size;
    }
    void close() noexcept override
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        ++state_->closes;
    }
private:
    std::shared_ptr<FakeState> state_;
};

void test_receiver()
{
    auto state = std::make_shared<FakeState>();
    state->bytes = io::srm_auto_aim::encode(io::srm_auto_aim::FeedbackFrame{10, 20, 30, 0, 0, 20});
    {
        auto transport = std::make_unique<io::srm_auto_aim::SrmAutoAimTransport>(std::make_unique<FakeStream>(state));
        SerialGimbalPose receiver(std::move(transport), PosePolicy{32, 100ms, 30ms, 0.5});
        {
            std::unique_lock<std::mutex> lock(state->mutex);
            require(state->changed.wait_for(lock, 1s, [&] { return state->reads >= 6; }), "receiver did not read");
        }
        receiver.rethrow_if_failed();
        // 一个姿态不构成稳定窗口，必须拒收，不能默默用单位阵或最新角度。
        rejects([&] { receiver.sample_at(PoseClock::now()); }, "incomplete history accepted");
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->fail = true;
        }
        bool propagated = false;
        const auto deadline = PoseClock::now() + 1s;
        while (!propagated && PoseClock::now() < deadline) {
            try { receiver.rethrow_if_failed(); }
            catch (const std::runtime_error & error) { propagated = std::string(error.what()) == "fake device disconnected"; }
            std::this_thread::sleep_for(1ms);
        }
        require(propagated, "worker failure not propagated");
        receiver.stop();
        receiver.stop();
        rejects([&] { receiver.sample_at(PoseClock::now()); }, "sample after stop accepted");
    }
    require(state->closes == 1, "stream not closed exactly once");
    require(state->writes == 0, "calibration sent a command");
    // 无数据时仍可停止；析构负责 join，不要求用户先拿到首帧。
    auto empty = std::make_shared<FakeState>();
    {
        auto transport = std::make_unique<io::srm_auto_aim::SrmAutoAimTransport>(std::make_unique<FakeStream>(empty));
        SerialGimbalPose receiver(std::move(transport));
        rejects([&] { receiver.sample_at(PoseClock::now()); }, "empty serial accepted");
        receiver.stop();
        rejects([&] { receiver.sample_at(PoseClock::now()); }, "stopped serial accepted");
    }
    require(empty->closes == 1, "empty receiver leaked stream");
}
}

int main()
{
    try {
        test_buffer();
        test_receiver();
        std::cout << "gimbal_pose_test passed\n";
        return 0;
    } catch (const std::exception & error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
