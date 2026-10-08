#ifndef AUTO_AIM__YOLOV5_HPP
#define AUTO_AIM__YOLOV5_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/dynamic_roi.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{
class YOLOV5 : public YOLOBase
{
public:
  YOLOV5(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  std::string device_, model_path_;
  std::string save_path_, debug_path_;
  bool debug_, use_roi_, use_traditional_;

  // 动态 ROI（可选）：按上一帧目标框裁剪再缩放到 640，使装甲板在网络输入里保持约
  // 40px，远距离才不会因为数字像素不足被判错类别。原理与实测见 dynamic_roi.hpp。
  bool dynamic_roi_;
  double dynamic_roi_scale_;
  int dynamic_roi_min_width_;
  std::optional<cv::Rect2f> last_target_box_;  // 上一帧置信度最高的目标框（原图坐标）
  int roi_miss_streak_ = 0;                    // 连续未检出帧数，超限即回退整帧重捕获

  const int class_num_ = 13;
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  double min_confidence_, binary_threshold_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;
  cv::Rect applied_roi_;  // 本帧实际裁剪区域（原图坐标），仅供调试绘制

  Detector detector_;
  friend class MultiThreadDetector;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

  // 对给定图像做一次 预处理→推理→解析。crop_img 可能已裁剪，raw_img 恒为原图，用于
  // center_norm 归一化；坐标回填靠 offset_，由调用方在裁剪时设置。
  std::list<Armor> infer_and_parse(
    const cv::Mat & crop_img, const cv::Mat & raw_img, int frame_count);

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  double sigmoid(double x);
};

}  // namespace auto_aim

#endif  //AUTO_AIM__YOLOV5_HPP