#include "srm_auto_aim_protocol.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace io::srm_auto_aim
{
namespace
{
constexpr std::int16_t kGimbalRecordId = 1;
constexpr std::int16_t kShootRecordId = 2;
constexpr std::uint16_t kCommandBodyLength = 16;
constexpr std::uint16_t kFeedbackBodyLength = 28;

static_assert(sizeof(float) == sizeof(std::uint32_t));
static_assert(std::numeric_limits<float>::is_iec559);

void append_u16_le(Bytes & bytes, std::uint16_t value)
{
  bytes.push_back(static_cast<std::uint8_t>(value & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
}

void append_i16_le(Bytes & bytes, std::int16_t value)
{
  append_u16_le(bytes, static_cast<std::uint16_t>(value));
}

void append_u32_le(Bytes & bytes, std::uint32_t value)
{
  bytes.push_back(static_cast<std::uint8_t>(value & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xffU));
}

void append_i32_le(Bytes & bytes, std::int32_t value)
{
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32_le(bytes, bits);
}

void append_f32_le(Bytes & bytes, float value)
{
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32_le(bytes, bits);
}

std::uint16_t read_u16_le(const Bytes & bytes, std::size_t offset)
{
  return static_cast<std::uint16_t>(bytes[offset]) |
         (static_cast<std::uint16_t>(bytes[offset + 1]) << 8U);
}

std::int16_t read_i16_le(const Bytes & bytes, std::size_t offset)
{
  return static_cast<std::int16_t>(read_u16_le(bytes, offset));
}

std::uint32_t read_u32_le(const Bytes & bytes, std::size_t offset)
{
  return static_cast<std::uint32_t>(bytes[offset]) |
         (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[offset + 2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[offset + 3]) << 24U);
}

std::int32_t read_i32_le(const Bytes & bytes, std::size_t offset)
{
  const auto bits = read_u32_le(bytes, offset);
  std::int32_t value = 0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

float read_f32_le(const Bytes & bytes, std::size_t offset)
{
  const auto bits = read_u32_le(bytes, offset);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool finite(float value)
{
  return std::isfinite(static_cast<double>(value));//isfinite判断浮点数是否有限
}

bool valid_command(const CommandFrame & frame)
{
  return finite(frame.yaw_deg) && finite(frame.pitch_deg);
}

bool valid_feedback(const FeedbackFrame & frame)
{
  return finite(frame.yaw_deg) && finite(frame.pitch_deg) && finite(frame.roll_deg) &&
         finite(frame.bullet_speed_mps);
}

}  // namespace

Bytes encode(const CommandFrame & frame)
{
  if (!valid_command(frame)) {
    throw std::invalid_argument("command frame contains a non-finite angle");
  }

  Bytes bytes;
  bytes.reserve(kCommandFrameSize);//总大小18
  append_u16_le(bytes, kCommandBodyLength);//命令体长度
  append_i16_le(bytes, kGimbalRecordId);//云台记录ID
  append_f32_le(bytes, frame.yaw_deg);
  append_f32_le(bytes, frame.pitch_deg);
  append_i16_le(bytes, kShootRecordId);//开火记录ID
  append_i32_le(bytes, frame.fire_flag);
  return bytes;
}

Bytes encode(const FeedbackFrame & frame)
{
  if (!valid_feedback(frame)) {
    throw std::invalid_argument("feedback frame contains a non-finite value");
  }

  Bytes bytes;
  bytes.reserve(kFeedbackFrameSize);//总大小30
  append_u16_le(bytes, kFeedbackBodyLength);//反馈体长度
  append_i16_le(bytes, kGimbalRecordId);//云台记录ID
  append_f32_le(bytes, frame.yaw_deg);
  append_f32_le(bytes, frame.pitch_deg);
  append_f32_le(bytes, frame.roll_deg);
  append_i32_le(bytes, frame.mode);
  append_i32_le(bytes, frame.color);
  append_i16_le(bytes, kShootRecordId);//开火记录ID
  append_f32_le(bytes, frame.bullet_speed_mps);
  return bytes;
}

bool decode(const Bytes & bytes, CommandFrame & output)
{
  if (bytes.size() != kCommandFrameSize || read_u16_le(bytes, 0) != kCommandBodyLength) {
    return false;
  }//确保数据长度正确，且前两个字节表示的命令体长度为16

  if (read_i16_le(bytes, 2) != kGimbalRecordId || read_i16_le(bytes, 12) != kShootRecordId) {
    return false;
  }//确保云台记录ID和开火记录ID正确

  CommandFrame candidate;
  candidate.yaw_deg = read_f32_le(bytes, 4);
  candidate.pitch_deg = read_f32_le(bytes, 8);
  candidate.fire_flag = read_i32_le(bytes, 14);
  if (!valid_command(candidate)) {
    return false;
  }

  output = candidate;
  return true;
}

bool decode(const Bytes & bytes, FeedbackFrame & output)
{
  if (bytes.size() != kFeedbackFrameSize || read_u16_le(bytes, 0) != kFeedbackBodyLength) {
    return false;
  }//确保数据长度正确，且前两个字节表示的反馈体长度为26

  if (read_i16_le(bytes, 2) != kGimbalRecordId || read_i16_le(bytes, 24) != kShootRecordId) {
    return false;
  }

  FeedbackFrame candidate;
  candidate.yaw_deg = read_f32_le(bytes, 4);
  candidate.pitch_deg = read_f32_le(bytes, 8);
  candidate.roll_deg = read_f32_le(bytes, 12);
  candidate.mode = read_i32_le(bytes, 16);
  candidate.color = read_i32_le(bytes, 20);
  candidate.bullet_speed_mps = read_f32_le(bytes, 26);
  if (!valid_feedback(candidate)) {
    return false;
  }

  output = candidate;
  return true;
}

StreamParser::StreamParser(Direction direction) : direction_(direction) {}

void StreamParser::append(const std::uint8_t * data, std::size_t size)
{
  if (size == 0) return;
  if (data == nullptr) throw std::invalid_argument("cannot append a null data pointer");

  buffer_.insert(buffer_.end(), data, data + size);
}

void StreamParser::append(const Bytes & data)
{
  append(data.data(), data.size());
}

std::vector<Frame> StreamParser::take_frames()
{
  std::vector<Frame> frames;
  const auto expected_body_length =
    direction_ == Direction::host_to_lower ? kCommandBodyLength : kFeedbackBodyLength;
  const auto expected_frame_size =
    direction_ == Direction::host_to_lower ? kCommandFrameSize : kFeedbackFrameSize;

  while (buffer_.size() >= sizeof(std::uint16_t)) {
    // 协议没有魔数和 CRC；长度非法时无法可靠地在任意字节处重同步，
    // 因此按规则丢弃当前缓存，等待下一次完整帧。
    if (read_u16_le(buffer_, 0) != expected_body_length) {
      buffer_.clear();
      break;
    }

    if (buffer_.size() < expected_frame_size) break;

    Bytes frame(buffer_.begin(), buffer_.begin() + expected_frame_size);
    buffer_.erase(buffer_.begin(), buffer_.begin() + expected_frame_size);

    if (direction_ == Direction::host_to_lower) {
      CommandFrame command;
      if (!decode(frame, command)) {
        buffer_.clear();
        break;
      }
      frames.emplace_back(command);
    } else {
      FeedbackFrame feedback;
      if (!decode(frame, feedback)) {
        buffer_.clear();
        break;
      }
      frames.emplace_back(feedback);
    }
  }

  return frames;
}

void StreamParser::clear()
{
  buffer_.clear();
}

}  // namespace io::srm_auto_aim
