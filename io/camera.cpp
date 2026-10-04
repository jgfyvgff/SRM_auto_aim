#include "camera.hpp"

#include <cmath>
#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(
  const std::string & config_path,
  const CameraSettingsOverride & overrides)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  if (overrides.exposure_ms) exposure_ms = *overrides.exposure_ms;
  if (!std::isfinite(exposure_ms) || exposure_ms <= 0.0) {
    throw std::invalid_argument("Camera exposure_ms must be finite and positive");
  }

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    if (overrides.gain) gain = *overrides.gain;
    if (!std::isfinite(gain) || gain < 0.0) {
      throw std::invalid_argument("Camera gain must be finite and non-negative");
    }
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

void Camera::read_timed(cv::Mat & img, FrameTiming & timing)
{
  camera_->read_timed(img, timing);
}

}  // namespace io
