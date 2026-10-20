#include "classifier.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

namespace auto_aim
{
Classifier::Classifier(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  rotate_180_ = yaml["classifier_rotate_180"].IsDefined()
                  ? yaml["classifier_rotate_180"].as<bool>()
                  : false;
  const double gamma = yaml["classifier_gamma"].IsDefined()
                         ? yaml["classifier_gamma"].as<double>()
                         : 1.0;
  if (!std::isfinite(gamma) || gamma <= 0.0 || gamma > 4.0) {
    throw std::invalid_argument("classifier_gamma must be finite and in (0, 4]");
  }
  if (gamma != 1.0) {
    gamma_lut_ = cv::Mat(1, 256, CV_8UC1);
    for (int value = 0; value < 256; ++value) {
      gamma_lut_.at<unsigned char>(0, value) = static_cast<unsigned char>(
        255.0 * std::pow(value / 255.0, gamma));
    }
  }
  auto model = yaml["classify_model"].as<std::string>();
  net_ = cv::dnn::readNetFromONNX(model);
  auto ovmodel = core_.read_model(model);
  compiled_model_ = core_.compile_model(
    ovmodel, "AUTO", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}


//OPENCV DNN
void Classifier::classify(Armor & armor)
{
  if (armor.pattern.empty()) {
    armor.name = ArmorName::not_armor;
    return;
  }

  cv::Mat oriented;
  if (rotate_180_) {
    cv::rotate(armor.pattern, oriented, cv::ROTATE_180);
  } else {
    oriented = armor.pattern;
  }
  cv::Mat gray;
  cv::cvtColor(oriented, gray, cv::COLOR_BGR2GRAY);

  auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  auto x_scale = static_cast<double>(32) / gray.cols;
  auto y_scale = static_cast<double>(32) / gray.rows;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(gray.rows * scale);
  auto w = static_cast<int>(gray.cols * scale);

  if (h == 0 || w == 0) {
    armor.name = ArmorName::not_armor;
    return;
  }
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(gray, input(roi), {w, h});
  // gamma<1 提亮暗数字；在 32×32 输入上执行，保持与离线预处理对比一致。
  if (!gamma_lut_.empty()) cv::LUT(input, gamma_lut_, input);

  auto blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar());

  net_.setInput(blob);//前项传播
  cv::Mat outputs = net_.forward();

  // softmax，转换为概率
  float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::exp(outputs - max, outputs);
  float sum = cv::sum(outputs)[0];
  outputs /= sum;

  // 取概率最大的类别
  double confidence;
  cv::Point label_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;
  
  armor.confidence = confidence;
  armor.name = static_cast<ArmorName>(label_id);
}


//OPENVINO
void Classifier::ovclassify(Armor & armor)
{
  if (armor.pattern.empty()) {
    armor.name = ArmorName::not_armor;
    return;
  }

  cv::Mat oriented;
  if (rotate_180_) {
    cv::rotate(armor.pattern, oriented, cv::ROTATE_180);
  } else {
    oriented = armor.pattern;
  }
  cv::Mat gray;
  cv::cvtColor(oriented, gray, cv::COLOR_BGR2GRAY);

  // Resize image to 32x32
  auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  auto x_scale = static_cast<double>(32) / gray.cols;
  auto y_scale = static_cast<double>(32) / gray.rows;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(gray.rows * scale);
  auto w = static_cast<int>(gray.cols * scale);

  if (h == 0 || w == 0) {
    armor.name = ArmorName::not_armor;
    return;
  }

  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(gray, input(roi), {w, h});
  if (!gamma_lut_.empty()) cv::LUT(input, gamma_lut_, input);
  // Normalize the input image to [0, 1] range
  input.convertTo(input, CV_32F, 1.0 / 255.0);

  //
  ov::Tensor input_tensor(ov::element::f32, {1, 1, 32, 32}, input.data);//创建输入张量

  ov::InferRequest infer_request = compiled_model_.create_infer_request();//创建推理请求
  infer_request.set_input_tensor(input_tensor);//设置输入张量
  infer_request.infer();//执行推理

  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat outputs(1, 9, CV_32F, output_tensor.data());//9类输出

  // Softmax
  float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::exp(outputs - max, outputs);
  float sum = cv::sum(outputs)[0];
  outputs /= sum;

  double confidence;
  cv::Point label_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;

  armor.confidence = confidence;
  armor.name = static_cast<ArmorName>(label_id);
}

}  // namespace auto_aim
