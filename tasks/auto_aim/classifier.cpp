#include "classifier.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
Classifier::Classifier(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
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

  cv::Mat gray;
  cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

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

  cv::Mat gray;
  cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

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