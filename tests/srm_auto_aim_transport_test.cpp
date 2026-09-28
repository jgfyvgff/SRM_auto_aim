#include "io/srm_auto_aim_transport.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace
{
using io::srm_auto_aim::ByteStream;
using io::srm_auto_aim::Bytes;
using io::srm_auto_aim::CommandFrame;
using io::srm_auto_aim::FeedbackFrame;
using io::srm_auto_aim::SrmAutoAimTransport;

class FakeByteStream final : public ByteStream
{
public:
  std::size_t read(std::uint8_t * data, std::size_t size) override
  {
    const auto count = std::min({size, max_read_size_, incoming_.size()});
    std::copy_n(incoming_.begin(), count, data);
    incoming_.erase(incoming_.begin(), incoming_.begin() + count);
    return count;
  }

  std::size_t write(const std::uint8_t * data, std::size_t size) override
  {
    written_.insert(written_.end(), data, data + size);
    return size;
  }

  void close() noexcept override { closed_ = true; }

  void feed(const Bytes & bytes) { incoming_.insert(incoming_.end(), bytes.begin(), bytes.end()); }

  Bytes written_;
  Bytes incoming_;
  std::size_t max_read_size_ = 64;
  bool closed_ = false;
};

bool nearly_equal(float lhs, float rhs)
{
  return std::abs(lhs - rhs) < 1e-5F;
}

bool fail(const char * message)
{
  std::cerr << message << '\n';
  return false;
}

bool test_send_and_receive()
{
  auto fake = std::make_unique<FakeByteStream>();
  auto * fake_ptr = fake.get();
  SrmAutoAimTransport transport(std::move(fake));

  transport.send(CommandFrame{1.5F, -2.0F, 0});
  const auto expected_command = io::srm_auto_aim::encode(CommandFrame{1.5F, -2.0F, 0});
  if (fake_ptr->written_ != expected_command) return fail("transport sent unexpected bytes");

  const FeedbackFrame expected_feedback{10.0F, -3.0F, 0.5F, 0, 7, 22.0F};
  const auto feedback_bytes = io::srm_auto_aim::encode(expected_feedback);
  fake_ptr->max_read_size_ = 7;
  fake_ptr->feed(feedback_bytes);

  std::vector<FeedbackFrame> feedback;
  for (int attempt = 0; attempt < 10 && feedback.empty(); ++attempt) {
    feedback = transport.poll_feedback();
  }
  if (feedback.size() != 1) return fail("transport did not assemble feedback frame");
  if (!nearly_equal(feedback.front().yaw_deg, expected_feedback.yaw_deg) ||
      !nearly_equal(feedback.front().pitch_deg, expected_feedback.pitch_deg) ||
      !nearly_equal(feedback.front().bullet_speed_mps, expected_feedback.bullet_speed_mps)) {
    return fail("transport decoded incorrect feedback values");
  }
  return true;
}

bool test_lifecycle()
{
  auto fake = std::make_unique<FakeByteStream>();
  auto * fake_ptr = fake.get();
  SrmAutoAimTransport transport(std::move(fake));
  transport.close();
  transport.close();
  if (!fake_ptr->closed_) return fail("transport did not close the byte stream");

  bool send_rejected = false;
  try {
    transport.send(CommandFrame{0.0F, 0.0F, 0});
  } catch (const std::runtime_error &) {
    send_rejected = true;
  }
  if (!send_rejected) return fail("send after close was not rejected");
  return true;
}

}  // namespace

int main()
{
  if (!test_send_and_receive() || !test_lifecycle()) return 1;

  std::cout << "srm_auto_aim_transport_test passed\n";
  return 0;
}
