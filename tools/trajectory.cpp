#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.7833;
//输入子弹初速度、目标水平距离和竖直高度，计算出飞行时间和抬头角度
Trajectory::Trajectory(const double v0, const double d, const double h)
{
  auto a = g * d * d / (2 * v0 * v0);//a = g*d^2/(2*v0^2)，a为二次项系数
  auto b = -d;//b = -d，b为一次项系数
  auto c = a + h;//c = a + h，c为常数项
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}

}  // namespace tools