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
    // inter_byte_timeout 不能取 Timeout::max()：库在该值下会走"定长多字节读"分支，
    // 按标称波特率 sleep 等待凑满整个读缓冲。USB CDC 下波特率保持库默认 9600，
    // 一个"字节时间"= 1.0417ms，于是 read(64) 被拖成该值的整数倍（实测 bracket_ms
    // 92% 落在 1.0417ms 格点上，k=23/7），串口线程只能拿到 42~58Hz 姿态样本。
    // 给出有限的字节间隔超时后，read 拿到数据即返回，读写等待仍受 timeout_ms 限制。
    constexpr std::uint32_t kReadInterByteTimeoutMs = 2;
    serial::Timeout timeout(
      kReadInterByteTimeoutMs, config.timeout_ms, 0, config.timeout_ms, 0);
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
