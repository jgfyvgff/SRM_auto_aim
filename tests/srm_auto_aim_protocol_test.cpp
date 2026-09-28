#include "io/srm_auto_aim_protocol.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using io::srm_auto_aim::Bytes;
using io::srm_auto_aim::CommandFrame;
using io::srm_auto_aim::Direction;
using io::srm_auto_aim::FeedbackFrame;
using io::srm_auto_aim::StreamParser;

bool nearly_equal(float lhs, float rhs)
{
  return std::abs(lhs - rhs) < 1e-5F;
}

bool fail(const char * message)
{
  std::cerr << message << '\n';
  return false;
}

bool test_command_frame()
{
  const CommandFrame expected{-12.5F, 4.25F, 0};
  const auto bytes = io::srm_auto_aim::encode(expected);
  if (bytes.size() != io::srm_auto_aim::kCommandFrameSize) {
    return fail("command frame size mismatch");
  }
  if (bytes[0] != 0x10 || bytes[1] != 0x00) {
    return fail("command body length is not little-endian 16");
  }

  CommandFrame actual;
  if (!io::srm_auto_aim::decode(bytes, actual) || !nearly_equal(actual.yaw_deg, expected.yaw_deg) ||
      !nearly_equal(actual.pitch_deg, expected.pitch_deg) ||
      actual.fire_flag != expected.fire_flag) {
    return fail("command frame round trip failed");
  }
  return true;
}

bool test_feedback_frame()
{
  const FeedbackFrame expected{10.0F, -2.5F, 0.75F, 3, 7, 22.0F};
  const auto bytes = io::srm_auto_aim::encode(expected);
  if (bytes.size() != io::srm_auto_aim::kFeedbackFrameSize) {
    return fail("feedback frame size mismatch");
  }
  if (bytes[0] != 0x1c || bytes[1] != 0x00) {
    return fail("feedback body length is not little-endian 28");
  }

  FeedbackFrame actual;
  if (!io::srm_auto_aim::decode(bytes, actual) || !nearly_equal(actual.yaw_deg, expected.yaw_deg) ||
      !nearly_equal(actual.pitch_deg, expected.pitch_deg) ||
      !nearly_equal(actual.roll_deg, expected.roll_deg) || actual.mode != expected.mode ||
      actual.color != expected.color ||
      !nearly_equal(actual.bullet_speed_mps, expected.bullet_speed_mps)) {
    return fail("feedback frame round trip failed");
  }
  return true;
}

bool test_stream_parser()
{
  const CommandFrame command_a{1.0F, 2.0F, 0};
  const CommandFrame command_b{-3.0F, 4.0F, 1};
  const auto bytes_a = io::srm_auto_aim::encode(command_a);
  const auto bytes_b = io::srm_auto_aim::encode(command_b);

  StreamParser parser(Direction::host_to_lower);
  parser.append(bytes_a.data(), 3);
  if (!parser.take_frames().empty()) return fail("partial frame was emitted");
  parser.append(bytes_a.data() + 3, bytes_a.size() - 3);
  auto frames = parser.take_frames();
  if (frames.size() != 1 || !std::holds_alternative<CommandFrame>(frames.front())) {
    return fail("fragmented command frame was not parsed");
  }

  Bytes combined = bytes_a;
  combined.insert(combined.end(), bytes_b.begin(), bytes_b.end());
  parser.append(combined);
  frames = parser.take_frames();
  if (frames.size() != 2) return fail("multiple command frames were not parsed");

  const FeedbackFrame feedback{10.0F, -2.0F, 0.5F, 0, 7, 21.5F};
  const auto feedback_bytes = io::srm_auto_aim::encode(feedback);
  StreamParser feedback_parser(Direction::lower_to_host);
  feedback_parser.append(feedback_bytes.data(), 1);
  feedback_parser.append(feedback_bytes.data() + 1, 11);
  if (!feedback_parser.take_frames().empty()) {
    return fail("partial feedback frame was emitted");
  }
  feedback_parser.append(feedback_bytes.data() + 12, feedback_bytes.size() - 12);
  frames = feedback_parser.take_frames();
  if (frames.size() != 1 || !std::holds_alternative<FeedbackFrame>(frames.front())) {
    return fail("fragmented feedback frame was not parsed");
  }
  return true;
}

bool test_invalid_frames()
{
  const auto valid = io::srm_auto_aim::encode(CommandFrame{0.0F, 0.0F, 0});
  StreamParser parser(Direction::host_to_lower);

  const Bytes invalid_length{0x11, 0x00};
  parser.append(invalid_length);
  if (!parser.take_frames().empty()) return fail("invalid length produced a frame");
  parser.append(valid);
  if (parser.take_frames().size() != 1) return fail("parser did not recover after invalid length");

  auto invalid_id = valid;
  invalid_id[2] = 0x03;
  invalid_id[3] = 0x00;
  parser.append(invalid_id);
  if (!parser.take_frames().empty()) return fail("unknown ID produced a frame");
  parser.append(valid);
  if (parser.take_frames().size() != 1) return fail("parser did not recover after unknown ID");

  bool rejected_non_finite = false;
  try {
    io::srm_auto_aim::encode(
      CommandFrame{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0});
  } catch (const std::invalid_argument &) {
    rejected_non_finite = true;
  }
  if (!rejected_non_finite) return fail("non-finite command angle was accepted");
  return true;
}

}  // namespace

int main()
{
  if (!test_command_frame() || !test_feedback_frame() || !test_stream_parser() ||
      !test_invalid_frames()) {
    return 1;
  }

  std::cout << "srm_auto_aim_protocol_test passed\n";
  return 0;
}
