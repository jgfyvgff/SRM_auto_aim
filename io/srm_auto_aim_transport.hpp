#ifndef IO__SRM_AUTO_AIM_TRANSPORT_HPP
#define IO__SRM_AUTO_AIM_TRANSPORT_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "io/srm_auto_aim_protocol.hpp"

namespace io::srm_auto_aim
{
// 字节流的最小外部资源边界，真实串口和 Fake 测试实现必须遵循同一契约。
// 读写失败直接抛出异常，由上层决定停止或重连策略。
class ByteStream
{
public:
  virtual ~ByteStream() = default;

  virtual std::size_t read(std::uint8_t * data, std::size_t size) = 0;
  virtual std::size_t write(const std::uint8_t * data, std::size_t size) = 0;
  virtual void close() noexcept = 0;
};

struct SerialConfig
{
  std::string device;
  std::uint32_t timeout_ms = 20;
};//串口配置结构体，包含设备名和超时时间

class SrmAutoAimTransport
{
public:
  // 当前接口是同步、单调用线程模型；send、poll_feedback 和 close 应由同一协调线程调用。
  // 它只负责一次读写和协议组帧，不自动创建心跳线程，也不替上层决定重连策略。
  // 该构造函数接管 stream 的所有权；stream 为空或构造失败时抛出异常。
  explicit SrmAutoAimTransport(std::unique_ptr<ByteStream> stream);

  // USB CDC 不依赖传统波特率；timeout_ms 仅用于限制一次读写等待时间。
  explicit SrmAutoAimTransport(const SerialConfig & config);

  ~SrmAutoAimTransport();

  SrmAutoAimTransport(const SrmAutoAimTransport &) = delete;
  SrmAutoAimTransport & operator=(const SrmAutoAimTransport &) = delete;

  // 同步发送一帧完整命令。调用者负责以固定频率持续发送安全状态。
  void send(const CommandFrame & command);

  // 读取当前可获得的字节并返回已经组好的反馈帧；没有新数据时返回空列表。
  std::vector<FeedbackFrame> poll_feedback();

  // 关闭底层设备；重复调用安全。
  void close() noexcept;

private:
  std::unique_ptr<ByteStream> stream_;
  StreamParser parser_{Direction::lower_to_host};
  bool closed_ = false;
};

}  // namespace io::srm_auto_aim

#endif  // IO__SRM_AUTO_AIM_TRANSPORT_HPP
