#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

namespace io
{
enum class FrameTimestampSource
{
  Unknown,
  DeviceClock,
  HostReceive,
};

inline const char * frame_timestamp_source_name(FrameTimestampSource source)
{
  switch (source) {
    case FrameTimestampSource::DeviceClock:
      return "device_clock";
    case FrameTimestampSource::HostReceive:
      return "host_receive";
    case FrameTimestampSource::Unknown:
    default:
      return "unknown";
  }
}

// 图像设备时标与主机时标必须分开保存；映射时间只是估计，不能当作硬件同步结果。
struct FrameTiming
{
  std::chrono::steady_clock::time_point host_received_at{};
  std::optional<std::chrono::steady_clock::time_point> mapped_capture_at;
  std::uint64_t device_ticks = 0;
  std::uint64_t device_timestamp_hz = 0;
  std::uint32_t frame_id = 0;
  // 记录算法实际使用的时间来源，避免把主机收帧时间误认为曝光时间。
  FrameTimestampSource timestamp_source = FrameTimestampSource::Unknown;
};

// 真机扫参时可临时覆盖配置文件中的曝光和增益；不提供覆盖时沿用 YAML。
struct CameraSettingsOverride
{
  std::optional<double> exposure_ms;
  std::optional<double> gain;
};

class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  // 旧相机没有设备时标时只返回主机收帧时间；调用方须检查 mapped_capture_at。
  virtual void read_timed(cv::Mat & img, FrameTiming & timing)
  {
    read(img, timing.host_received_at);
  }
};

class Camera
{
public:
  Camera(
    const std::string & config_path,
    const CameraSettingsOverride & overrides = {});
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  void read_timed(cv::Mat & img, FrameTiming & timing);

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP
