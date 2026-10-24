#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class HikRobot : public CameraBase
{
public:
  HikRobot(
    double exposure_ms, double gain, const std::string & vid_pid,
    bool capture_raw_bayer = false);
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  void read_timed(cv::Mat & img, FrameTiming & timing) override;
  bool read_timed_for(
    cv::Mat & img, FrameTiming & timing, std::chrono::milliseconds timeout) override;
  bool read_timed_for(
    cv::Mat & img, FrameTiming & timing, std::chrono::milliseconds timeout,
    cv::Mat & raw_bayer, std::string & raw_bayer_encoding) override;

private:
  struct CameraData
  {
    cv::Mat img;
    FrameTiming timing;
    cv::Mat raw_bayer;
    std::string raw_bayer_encoding;
  };

  double exposure_us_;//曝光时间
  double gain_;//增益
  // 仅 rosbag 录制时复制 SDK 帧，避免普通跟踪运行增加每帧内存拷贝。
  bool capture_raw_bayer_ = false;

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  void * handle_;
  std::thread capture_thread_;
  std::atomic<bool> capturing_;
  std::atomic<bool> capture_quit_;
  // 容量 1 且丢弃旧帧；检测速度低于相机帧率时优先保留最新图像。
  tools::ThreadSafeQueue<CameraData, true> queue_;

  int vid_, pid_;//usb设备id

  void capture_start();
  void capture_stop();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP
