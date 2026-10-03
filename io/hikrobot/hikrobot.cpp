#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include "device_clock_mapper.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(double exposure_ms, double gain, const std::string & vid_pid)
: exposure_us_(exposure_ms * 1e3), gain_(gain), queue_(1), daemon_quit_(false), vid_(-1), pid_(-1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      reset_usb();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  FrameTiming timing;
  read_timed(img, timing);
  timestamp = timing.host_received_at;
}

void HikRobot::read_timed(cv::Mat & img, FrameTiming & timing)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timing = data.timing;
}


/*
海康相机初始化
MV_CC_EnumDevices()     →  枚举USB设备，查找相机
MV_CC_CreateHandle()    →  创建设备句柄
MV_CC_OpenDevice()      →  打开设备
set_enum/set_float()    →  配置参数（曝光、增益、白平衡）
MV_CC_StartGrabbing()   →  开始拉流
创建 capture_thread     →  启动抓图线程


*/
void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;
  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);//自动白平衡
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);//关闭自动曝光
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);//关闭自动增益
  set_float_value("ExposureTime", exposure_us_);//设置曝光时间
  set_float_value("Gain", gain_);//设置增益
  MV_CC_SetFrameRate(handle_, 150);//设置帧率

  // 设备时间戳单位由相机节点给出，不猜测 SDK 字段的计数频率。
  MVCC_INTVALUE_EX frequency_value{};
  const int frequency_ret =
    MV_CC_GetIntValueEx(handle_, "DeviceTimestampFrequency", &frequency_value);
  const std::uint64_t timestamp_frequency_hz =
    frequency_ret == MV_OK && frequency_value.nCurValue > 0
      ? static_cast<std::uint64_t>(frequency_value.nCurValue)
      : 0;
  if (timestamp_frequency_hz == 0) {
    tools::logger()->warn(
      "HikRobot DeviceTimestampFrequency unavailable (SDK code {:#x}); "
      "real auto-aim will reject unmapped frames", frequency_ret);
  } else {
    tools::logger()->info("HikRobot device timestamp frequency: {} Hz", timestamp_frequency_hz);
  }

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  capture_thread_ = std::thread{[this, timestamp_frequency_hz] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;
    DeviceClockMapper clock_mapper(timestamp_frequency_hz);

    MV_FRAME_OUT raw;//原始图像
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;

    while (!capture_quit_) {
      std::this_thread::sleep_for(1ms);

      unsigned int ret;
      unsigned int nMsec = 100;

      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
        break;
      }

      // GetImageBuffer 返回后立即记录主机到达时刻，转换/排队耗时不混入时标配对。
      const auto received_at = std::chrono::steady_clock::now();
      const auto & frame_info = raw.stFrameInfo;
      const std::uint64_t device_ticks =
        (static_cast<std::uint64_t>(frame_info.nDevTimeStampHigh) << 32) |
        frame_info.nDevTimeStampLow;
      FrameTiming timing;
      timing.host_received_at = received_at;
      timing.device_ticks = device_ticks;
      timing.device_timestamp_hz = timestamp_frequency_hz;
      timing.frame_id = frame_info.nFrameNum;
      timing.mapped_capture_at = clock_mapper.observe(device_ticks, received_at);
      cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);
      //转换OPENCV格式

      cvt_param.nWidth = raw.stFrameInfo.nWidth;
      cvt_param.nHeight = raw.stFrameInfo.nHeight;

      cvt_param.pSrcData = raw.pBufAddr;
      cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
      cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

      cvt_param.pDstBuffer = img.data;
      cvt_param.nDstBufferSize = img.total() * img.elemSize();
      cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

      // ret = MV_CC_ConvertPixelType(handle_, &cvt_param);
      auto pixel_type = frame_info.enPixelType;
      cv::Mat dst_image;
      //Bayer格式转换映射表
      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR}};
      const auto conversion = type_map.find(pixel_type);
      if (conversion == type_map.end()) {
        tools::logger()->error(
          "HikRobot unsupported pixel type: {:#x}",
          static_cast<unsigned int>(pixel_type));
        MV_CC_FreeImageBuffer(handle_, &raw);
        break;
      }
      cv::cvtColor(img, dst_image, conversion->second);
      img = dst_image;

      queue_.push({img, timing});

      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
    return;
  }
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');//分隔符
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);//整数型转换
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  // https://github.com/ralight/usb-reset/blob/master/usb-reset.c
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

}  // namespace io
