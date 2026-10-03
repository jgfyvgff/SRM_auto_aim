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
// 图像设备时标与主机时标必须分开保存；映射时间只是估计，不能当作硬件同步结果。
struct FrameTiming
{
  std::chrono::steady_clock::time_point host_received_at{};
  std::optional<std::chrono::steady_clock::time_point> mapped_capture_at;
  std::uint64_t device_ticks = 0;
  std::uint64_t device_timestamp_hz = 0;
  std::uint32_t frame_id = 0;
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
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  void read_timed(cv::Mat & img, FrameTiming & timing);

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP
