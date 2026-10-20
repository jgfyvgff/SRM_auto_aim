#include "mt_detector.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  auto model_path = yaml[yolo_name + "_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();

  auto model = core_.read_model(model_path);
  const auto model_shape = model->input().get_shape();
  if (model_shape.size() != 4 || model_shape[0] != 1 || model_shape[1] != 3 ||
      model_shape[2] != model_shape[3] || model_shape[2] == 0) {
    throw std::invalid_argument("MultiThreadDetector requires square NCHW model input");
  }
  input_size_ = static_cast<int>(model_shape[2]);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, input_size_, input_size_, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    // .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)
    .scale(255.0);

  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));

  tools::logger()->info("[MultiThreadDetector] initialized !");
}

void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  auto x_scale = static_cast<double>(input_size_) / img.rows;
  auto y_scale = static_cast<double>(input_size_) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  // preproces
  auto input = cv::Mat(input_size_, input_size_, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(img, input(roi), {w, h});

  auto infer_request = compiled_model_.create_infer_request();
  ov::Tensor input_tensor(
    ov::element::u8, {1, static_cast<std::size_t>(input_size_),
                     static_cast<std::size_t>(input_size_), 3});
  // 异步推理可能晚于 push 返回，Tensor 必须拥有像素内存，不能引用局部 Mat。
  std::memcpy(input_tensor.data(), input.data, input.total() * input.elemSize());

  infer_request.set_input_tensor(input_tensor);
  infer_request.start_async();
  queue_.push({img.clone(), t, std::move(infer_request)});
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  auto [img, t, infer_request] = queue_.pop();
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  if (output_shape.size() != 3 || output_shape[0] != 1) {
    throw std::runtime_error("Unexpected detector output rank");
  }
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(input_size_) / img.rows;
  auto y_scale = static_cast<double>(input_size_) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, img, 0);  //暂不支持ROI

  return {std::move(armors), t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  auto [img, t, infer_request] = queue_.pop();
  infer_request.wait();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  if (output_shape.size() != 3 || output_shape[0] != 1) {
    throw std::runtime_error("Unexpected detector output rank");
  }
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(input_size_) / img.rows;
  auto y_scale = static_cast<double>(input_size_) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = yolo_.postprocess(scale, output, img, 0);  //暂不支持ROI

  return {img, std::move(armors), t};
}

}  // namespace multithread

}  // namespace auto_aim
