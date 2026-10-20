#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>

#include "armor.hpp"

namespace auto_aim
{
class Classifier
{
public:
  explicit Classifier(const std::string & config_path);

  void classify(Armor & armor);

  void ovclassify(Armor & armor);

private:
  bool rotate_180_ = false;  // 倒装相机的数字需转正；旧配置默认保持原方向。
  cv::Mat gamma_lut_;       // 仅提亮 Tiny 输入，不改变 YOLO 检测或几何坐标。
  cv::dnn::Net net_;
  ov::Core core_;
  ov::CompiledModel compiled_model_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CLASSIFIER_HPP
