#include "srm_auto_aim_transport.hpp"

#include <array>
#include <stdexcept>
#include <utility>

#include "serial/serial.h"

namespace io::srm_auto_aim
{
namespace
{
class SerialByteStream final : public ByteStream
{
public:
  explicit SerialByteStream(const SerialConfig & config)
  {
    if (config.device.empty()) {
      throw std::invalid_argument("self-aim serial device cannot be empty");
    }

    // USB CDC 不使用 UART 波特率；serial 库仍需要先创建对象，超时由这里配置。
    serial::Timeout timeout = serial::Timeout::simpleTimeout(config.timeout_ms);
    serial_.setTimeout(timeout);
    serial_.setPort(config.device);
    serial_.open();
  }

  std::size_t read(std::uint8_t * data, std::size_t size) override
  {
    return serial_.read(data, size);
  }

  std::size_t write(const std::uint8_t * data, std::size_t size) override
  {
    return serial_.write(data, size);
  }

  void close() noexcept override
  {
    try {
      if (serial_.isOpen()) serial_.close();
    } catch (...) {
      // 析构和显式 close 不能再次向外传播资源关闭异常。
    }
  }

private:
  serial::Serial serial_;
};
}  // namespace

SrmAutoAimTransport::SrmAutoAimTransport(std::unique_ptr<ByteStream> stream)
: stream_(std::move(stream))
{
  if (stream_ == nullptr) {
    throw std::invalid_argument("self-aim transport requires a byte stream");
  }
}

SrmAutoAimTransport::SrmAutoAimTransport(const SerialConfig & config)
: SrmAutoAimTransport(std::make_unique<SerialByteStream>(config))
{
}

SrmAutoAimTransport::~SrmAutoAimTransport()
{
  close();
}

void SrmAutoAimTransport::send(const CommandFrame & command)
{
  if (closed_) throw std::runtime_error("self-aim transport is closed");

  const auto bytes = encode(command);
  const auto written = stream_->write(bytes.data(), bytes.size());
  if (written != bytes.size()) {
    throw std::runtime_error("self-aim command frame was partially written");
  }
}

std::vector<FeedbackFrame> SrmAutoAimTransport::poll_feedback()
{
  if (closed_) throw std::runtime_error("self-aim transport is closed");

  std::array<std::uint8_t, 64> read_buffer{};
  const auto received = stream_->read(read_buffer.data(), read_buffer.size());
  if (received == 0) return {};

  parser_.append(read_buffer.data(), received);
  const auto frames = parser_.take_frames();

  std::vector<FeedbackFrame> feedback_frames;
  feedback_frames.reserve(frames.size());
  for (const auto & frame : frames) {
    if (const auto * feedback = std::get_if<FeedbackFrame>(&frame)) {
      feedback_frames.push_back(*feedback);
    }
  }
  return feedback_frames;
}

void SrmAutoAimTransport::close() noexcept
{
  if (closed_) return;
  closed_ = true;
  stream_->close();
  parser_.clear();
}

}  // namespace io::srm_auto_aim
