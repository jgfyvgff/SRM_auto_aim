#pragma once

#include <algorithm>
#include <cmath>

namespace real_auto_aim
{
// 仅处理无开火绝对角指令；单位均为度，调用方负责确认角度和限幅参数有限。
struct TxAnglesDeg
{
    double yaw = 0.0;
    double pitch = 0.0;
};

// 角速度决定不同图像帧率下的最大变化，单次上限保留旧 CLI 参数的安全约束。
struct TxRateLimits
{
    double yaw_rate_deg_s = 60.0;
    double pitch_rate_deg_s = 30.0;
    double yaw_step_cap_deg = 2.0;
    double pitch_step_cap_deg = 1.0;
};

inline double wrap_tx_yaw_deg(double angle)
{
    const double wrapped = std::remainder(angle, 360.0);
    if (wrapped >= 180.0) return wrapped - 360.0;
    if (wrapped < -180.0) return wrapped + 360.0;
    return wrapped;
}

// base 必须是上一条已下发角（首次发送可用当前云台角）；elapsed_s 为两次下发间隔。
// authority 在 [0,1]，用于暂失目标时降低运动权限；该函数不触碰串口或开火标志。
inline TxAnglesDeg limit_tx_angles(
    const TxAnglesDeg & base, const TxAnglesDeg & target, double elapsed_s,
    const TxRateLimits & limits, double authority = 1.0)
{
    const double yaw_step = std::min(
        limits.yaw_step_cap_deg, limits.yaw_rate_deg_s * elapsed_s * authority);
    const double pitch_step = std::min(
        limits.pitch_step_cap_deg, limits.pitch_rate_deg_s * elapsed_s * authority);
    const double yaw_delta = wrap_tx_yaw_deg(target.yaw - base.yaw);
    return {
        wrap_tx_yaw_deg(base.yaw + std::clamp(yaw_delta, -yaw_step, yaw_step)),
        base.pitch + std::clamp(target.pitch - base.pitch, -pitch_step, pitch_step),
    };
}
}  // namespace real_auto_aim
