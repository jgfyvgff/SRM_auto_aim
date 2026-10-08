#include <cmath>
#include <iostream>
#include <string>

#include "src/real_auto_aim/tx_command_limiter.hpp"

namespace
{
bool expect_near(double actual, double expected, const std::string & label)
{
    if (std::abs(actual - expected) < 1e-9) return true;
    std::cerr << label << ": expected " << expected << ", got " << actual << '\n';
    return false;
}
}

int main()
{
    using real_auto_aim::TxAnglesDeg;
    using real_auto_aim::TxRateLimits;
    using real_auto_aim::limit_tx_angles;
    using real_auto_aim::wrap_tx_yaw_deg;

    const TxRateLimits limits{};
    const auto first = limit_tx_angles({0.0, 0.0}, {20.0, 10.0}, 0.02, limits);
    if (!expect_near(first.yaw, 1.2, "first yaw") ||
        !expect_near(first.pitch, 0.6, "first pitch")) return 1;

    // 恢复跟踪时必须以上一条指令为基准；不能因当前反馈姿态不同而跳过限幅。
    const auto second = limit_tx_angles(first, {-20.0, -10.0}, 0.04, limits);
    if (!expect_near(second.yaw, -0.8, "second yaw") ||
        !expect_near(second.pitch, -0.4, "second pitch")) return 1;

    const auto extrapolated = limit_tx_angles(first, {20.0, 10.0}, 0.02, limits, 0.35);
    if (!expect_near(extrapolated.yaw, 1.62, "extrapolated yaw") ||
        !expect_near(extrapolated.pitch, 0.81, "extrapolated pitch")) return 1;

    // 控制周期变长时仍不能突破旧参数的单次安全上限；yaw 走最短跨界路径。
    const auto bounded = limit_tx_angles({179.0, 0.0}, {-170.0, 10.0}, 0.10, limits);
    if (!expect_near(bounded.yaw, -179.0, "wrapped yaw") ||
        !expect_near(bounded.pitch, 1.0, "bounded pitch") ||
        !expect_near(wrap_tx_yaw_deg(180.0), -180.0, "yaw boundary")) return 1;
    std::cout << "tx_command_limiter_test passed\n";
}
