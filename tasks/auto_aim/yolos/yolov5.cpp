#include "yolov5.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <optional>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
namespace
{
// 连续漏检超过该帧数就放弃先验、回退整帧：66Hz 下约 75ms，够跨越单帧闪烁，
// 又不至于在目标已经移出窗口后一直空找。
constexpr int kRoiMissTolerance = 5;
}  // namespace

YOLOV5::YOLOV5(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolov5_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();
  // 动态 ROI 是可选增强：老配置文件没有这些键时保持原有整帧行为，便于 A/B 与回退。
  dynamic_roi_ = yaml["dynamic_roi"].IsDefined() ? yaml["dynamic_roi"].as<bool>() : false;
  dynamic_roi_scale_ =
    yaml["dynamic_roi_scale"].IsDefined() ? yaml["dynamic_roi_scale"].as<double>() : 16.0;
  dynamic_roi_min_width_ =
    yaml["dynamic_roi_min_width"].IsDefined() ? yaml["dynamic_roi_min_width"].as<int>() : 320;
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);
  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);//这个类用于预处理和后处理
  auto & input = ppp.input();//返回输入端口，后面的操作都是对输入端口进行的

  input.tensor()                    //描述用户要传入的 tensor
    .set_element_type(ov::element::u8)//设置数据类型
    .set_shape({1, 640, 640, 3})//形状：1个样本，640*640的图片，3个通道
    .set_layout("NHWC")   //布局，NHWC表示：N个样本，H行，W列，C通道
    .set_color_format(ov::preprocess::ColorFormat::BGR);//设置颜色格式，BGR表示输入图片是BGR格式

  input.model().set_layout("NCHW");//模型期望的输入布局，NCHW表示：N个样本，C通道，H行，W列

  input.preprocess()                  //预处理步骤，对输入进行预处理
    .convert_element_type(ov::element::f32)   //将数据类型转换为 float32
    .convert_color(ov::preprocess::ColorFormat::RGB)    //将颜色格式转换为 RGB
    .scale(255.0);  //将像素值缩放到0-1之间
//最终NHWC->RGB->float32->scale(1/255.0)->NCHW


  // TODO: ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
  model = ppp.build();//  构建预处理后的模型
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}//编译模型到指定设备(CPU/GPU)，并选择"低延迟"性能模式（适合实时自瞄）

std::list<Armor> YOLOV5::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  if (dynamic_roi_) {
    // 先验来自上一帧检测；连续漏检超限就用整帧重新捕获。
    const bool use_prior = last_target_box_.has_value() && roi_miss_streak_ <= kRoiMissTolerance;
    auto rect = dynamic_roi_rect(
      use_prior ? last_target_box_ : std::nullopt, dynamic_roi_scale_, dynamic_roi_min_width_,
      raw_img.cols, raw_img.rows);
    // offset_ 必须跟着裁剪原点走：parse 得到的坐标是裁剪图坐标，靠它加回原图坐标。
    offset_ = cv::Point2f(static_cast<float>(rect.x), static_cast<float>(rect.y));
    applied_roi_ = rect;
    bgr_img = raw_img(rect);
  } else if (use_roi_) {
    if (roi_.width == -1) {  // -1 表示该维度不裁切
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) {  // -1 表示该维度不裁切
      roi_.height = raw_img.rows;
    }
    applied_roi_ = roi_;
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  auto armors = infer_and_parse(bgr_img, raw_img, frame_count);

  if (dynamic_roi_) {
    if (!armors.empty() && !last_target_box_.has_value()) {
      // 首次捕获没有先验，整帧缩放到 640 会让远距离装甲板的数字只剩几个像素、类别读错；
      // 而追踪器正是在这一帧锁定类别，锁错之后会被名字过滤一直拦到超时（真机实测
      // 开 ROI 时 30 帧里 27 帧有检测却 0 候选）。这里按整帧检出结果就地再裁一次重新
      // 分类，只多花一次推理，且只发生在锁定目标之前。
      const auto best = std::max_element(
        armors.begin(), armors.end(),
        [](const Armor & a, const Armor & b) { return a.confidence < b.confidence; });
      const float width = static_cast<float>(best->box.width);
      const cv::Rect2f prior{
        best->center.x - width / 2.0F, best->center.y - width * 0.375F, width, width * 0.75F};
      const auto rect = dynamic_roi_rect(
        prior, dynamic_roi_scale_, dynamic_roi_min_width_, raw_img.cols, raw_img.rows);
      if (rect.width < raw_img.cols || rect.height < raw_img.rows) {
        offset_ = cv::Point2f(static_cast<float>(rect.x), static_cast<float>(rect.y));
        auto refined = infer_and_parse(raw_img(rect), raw_img, frame_count);
        // 细化失败（裁剪窗口内没检出）就保留整帧结果，宁可先用错类别也不要丢这一帧。
        if (!refined.empty()) {
          armors = std::move(refined);
          applied_roi_ = rect;
        }
      }
    }
    if (!armors.empty()) {
      // Armor 的 offset 构造已把 center 回填到原图坐标，可直接作为下一帧先验；
      // box 仍是裁剪图坐标，但宽度与原图一致，所以只取宽度。
      const auto best = std::max_element(
        armors.begin(), armors.end(),
        [](const Armor & a, const Armor & b) { return a.confidence < b.confidence; });
      const float prior_width = static_cast<float>(best->box.width);
      last_target_box_ = cv::Rect2f(
        best->center.x - prior_width / 2.0F, best->center.y - prior_width * 0.375F, prior_width,
        prior_width * 0.75F);
      roi_miss_streak_ = 0;
    } else {
      ++roi_miss_streak_;
    }
  }
  return armors;
}

std::list<Armor> YOLOV5::infer_and_parse(
  const cv::Mat & crop_img, const cv::Mat & raw_img, int frame_count)
{
  auto x_scale = static_cast<double>(640) / crop_img.rows;
  auto y_scale = static_cast<double>(640) / crop_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(crop_img.rows * scale);
  auto w = static_cast<int>(crop_img.cols * scale);

  // preproces
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(crop_img, input(roi), {w, h});
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);//创建输入张量，指定数据类型、形状和指针

  // infer
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());//创建输出矩阵，指定行数、列数、数据类型和数据指针

  // 解析输出矩阵，返回识别到的装甲板信息
  return parse(scale, output, raw_img, frame_count);
}


//output:  [行 = 检测框候选] × [列 = 每个框的特征值]

// 列索引	内容
// 0~1	角点0 (x, y)
// 2~3	角点1 (x, y)
// 4~5	角点2 (x, y)
// 6~7	角点3 (x, y)
// 8	置信度（objectness）
// 9~12	颜色 one-hot（4类）
// 13~21	数字类别 one-hot（9类）
std::list<Armor> YOLOV5::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // for each row: xywh + classess
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);//置信度
    score = sigmoid(score);//经过sigmoid函数映射到0-1之间，即概率

    if (score < score_threshold_) continue;

    std::vector<cv::Point2f> armor_key_points;

    //颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     //color，4类
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  //num，9类
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);//找到最大值
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);//找到最大值
    _class_id = class_id.x;
    _color_id = color_id.x;

    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (int i = 1; i < armor_key_points.size(); i++) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  //NMS，去重
  std::vector<int> indices;//存储NMS后保留的检测框索引
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    // 只要这一帧做过裁剪（静态 ROI 或动态 ROI），就必须把裁剪原点加回去：
    // 否则 center/points 会停在裁剪图坐标，跟踪器拿到的是错误位置，
    // 动态 ROI 也会因此按错误位置推算下一帧窗口。
    if (use_roi_ || dynamic_roi_) {
      armors.emplace_back(
        color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
    } else {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);
    }
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    // 使用传统方法二次矫正角点
    if (use_traditional_) detector_.detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

bool YOLOV5::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于神经网络的迭代
  // if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool YOLOV5::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  // 保存异常的图案，用于神经网络的迭代
  // if (!name_ok) save(armor);

  return name_ok;
}

cv::Point2f YOLOV5::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_ || dynamic_roi_) {
    // 画本帧实际裁剪区域；动态 ROI 下它每帧都在变，便于确认窗口有没有跟住目标。
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, applied_roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
  cv::imshow("detection", detection);
}

void YOLOV5::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, tmp_img_);
}

double YOLOV5::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::list<Armor> YOLOV5::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim