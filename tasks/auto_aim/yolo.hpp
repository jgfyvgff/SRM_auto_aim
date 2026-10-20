#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim
{
// 记录本帧实际 ROI 和动态裁剪所用先验，只供诊断，不参与算法决策。
struct DetectorDebugInfo
{
  std::string model_name;
  bool roi_active = false;
  cv::Rect applied_roi{};
  bool prior_used = false;
  bool prior_box_valid = false;
  cv::Rect2f prior_box{};
  int roi_miss_streak_before = 0;
  int roi_miss_streak_after = 0;
  // -1 表示此模型未提供该阶段计数；区分 YOLO 无候选与 Tiny 分类拒绝。
  int score_candidates = -1;
  int nms_candidates = -1;
  int classifier_rejected = -1;
};

class YOLOBase
{
public:
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;
  virtual DetectorDebugInfo debug_info() const { return {}; }

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);
  DetectorDebugInfo debug_info() const;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
