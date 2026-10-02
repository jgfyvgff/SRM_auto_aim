#pragma once

#include <atomic>
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <thread>

#include "calibration/gimbal_pose.hpp"
#include "io/srm_auto_aim_transport.hpp"

namespace calibration
{
// 只接收 USB CDC 姿态，不发送控制命令。构造成功即 Running；失败转 Failed，stop 转 Stopped。
// transport 仅由接收线程使用；销毁时先请求停止、join，再关闭串口。
class SerialGimbalPose
{
public:
    SerialGimbalPose(const io::srm_auto_aim::SerialConfig & config, PosePolicy policy = {});
    // Fake 和真实串口共用入口；流的 read 必须有界返回，否则析构无法及时 join。
    SerialGimbalPose(std::unique_ptr<io::srm_auto_aim::SrmAutoAimTransport> transport,
                    PosePolicy policy = {});
    ~SerialGimbalPose();
    SerialGimbalPose(const SerialGimbalPose &) = delete;
    SerialGimbalPose & operator=(const SerialGimbalPose &) = delete;

    // 最多等待 max_gap 以取得图像后的反馈。等待释放 mutex；数据问题返回异常由 UI 报告。
    PoseMatch sample_at(PoseClock::time_point image_time);
    void rethrow_if_failed() const;
    // 由拥有者线程调用；重复停止安全，停止后不能重新启动或采样。
    void stop() noexcept;

private:
    enum class State { Running, Failed, Stopped };
    void receive_loop() noexcept;
    std::unique_ptr<io::srm_auto_aim::SrmAutoAimTransport> transport_;
    PosePolicy policy_;
    // mutex 仅保护 buffer、state、failure；串口读、关闭和 join 都在锁外。
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    GimbalPoseBuffer buffer_;
    State state_ = State::Running;
    std::exception_ptr failure_;
    std::atomic<bool> stop_requested_{false};
    std::thread worker_;
};
}  // namespace calibration
