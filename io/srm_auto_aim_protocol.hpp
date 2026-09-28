#ifndef IO__SRM_AUTO_AIM_PROTOCOL_HPP
#define IO__SRM_AUTO_AIM_PROTOCOL_HPP

#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>

namespace io::srm_auto_aim
{
using Bytes = std::vector<std::uint8_t>;

constexpr std::size_t kCommandFrameSize = 18;
constexpr std::size_t kFeedbackFrameSize = 30;

enum class Direction
{
  host_to_lower,
  lower_to_host
};

// 上位机发送给下位机的目标姿态和视觉开火请求。
// yaw/pitch 使用比赛协议规定的角度制，不在协议层转换为弧度。
struct CommandFrame
{
  float yaw_deg = 0.0F;
  float pitch_deg = 0.0F;
  std::int32_t fire_flag = 0;
};

// 下位机反馈的当前云台状态和裁判系统提供的弹速。
struct FeedbackFrame
{
  float yaw_deg = 0.0F;
  float pitch_deg = 0.0F;
  float roll_deg = 0.0F;
  std::int32_t mode = 0;
  std::int32_t color = 0;
  float bullet_speed_mps = 0.0F;
};

using Frame = std::variant<CommandFrame, FeedbackFrame>;

// 编码函数只负责协议字段，不打开串口，也不执行任何硬件操作。
Bytes encode(const CommandFrame & frame);
Bytes encode(const FeedbackFrame & frame);

// 解码函数要求输入恰好是一帧完整数据；失败时不会修改 output。
bool decode(const Bytes & bytes, CommandFrame & output);
bool decode(const Bytes & bytes, FeedbackFrame & output);

class StreamParser
{
public:
  explicit StreamParser(Direction direction);

  // 持久化接收缓存，允许一次追加半帧、整帧或多帧数据。
  void append(const std::uint8_t * data, std::size_t size);
  void append(const Bytes & data);

  // 取出当前缓存中已经完成且通过校验的帧；不完整帧会留在缓存中。
  std::vector<Frame> take_frames();

  // 清除尚未完成的接收数据，通常用于关闭或重新打开设备后的状态复位。
  void clear();

private:
  Direction direction_;
  Bytes buffer_;
};

}  // namespace io::srm_auto_aim

#endif  // IO__SRM_AUTO_AIM_PROTOCOL_HPP
