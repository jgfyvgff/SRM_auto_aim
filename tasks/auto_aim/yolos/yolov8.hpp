#ifndef AUTO_AIM__YOLOV8_HPP
#define AUTO_AIM__YOLOV8_HPP

#include <list>
#include <optional>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/classifier.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{

class YOLOV8 : public YOLOBase
{
public:
  YOLOV8(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;
  DetectorDebugInfo debug_info() const override { return debug_info_; }

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  Classifier classifier_;
  Detector detector_;

  std::string device_, model_path_;
  std::string save_path_, debug_path_;
  bool debug_, use_roi_;

  // YOLOv8 的 416 输入与 YOLOv5 的 640 输入分别配置 ROI 尺度，便于独立回退。
  bool dynamic_roi_ = false;
  double dynamic_roi_scale_ = 10.4;
  int dynamic_roi_min_width_ = 208;
  std::optional<cv::Rect2f> last_target_box_;  // 上一帧已通过 Tiny 分类的原图目标框
  int roi_miss_streak_ = 0;

  const int class_num_ = 2;
  const float nms_threshold_ = 0.3;
  float score_threshold_ = 0.7F;
  double min_confidence_, binary_threshold_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Rect applied_roi_{};
  DetectorDebugInfo debug_info_{};

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Mat get_pattern(const cv::Mat & bgr_img, const Armor & armor) const;
  ArmorType get_type(const Armor & armor);
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);
  // crop_img 用于推理；raw_img 用于 Tiny 裁图和归一化，端点在 parse 前加回裁剪原点。
  std::list<Armor> infer_and_parse(
    const cv::Mat & crop_img, const cv::Mat & raw_img, int frame_count);

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
};

}  // namespace auto_aim

#endif  // TOOLS__YOLOV8_HPP
