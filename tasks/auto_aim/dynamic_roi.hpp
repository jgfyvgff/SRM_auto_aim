#ifndef AUTO_AIM__DYNAMIC_ROI_HPP
#define AUTO_AIM__DYNAMIC_ROI_HPP

#include <algorithm>
#include <optional>
#include <opencv2/opencv.hpp>

namespace auto_aim
{
// 网络输入固定 640 宽，整帧缩放时 1440 宽的图像被压到 0.444 倍，远距离装甲板上的
// 数字只剩几个像素，模型会读错类别。实测 5.8m 处同一批帧像素：整帧路径 30/30 帧判成
// outpost（错），按目标框裁剪后 30/30 帧判成 two（对）；追踪器正是因为类别错才按
// 3 块装甲 / r=0.2765 建模，而目标实际是 4 块 / r=0.18。
//
// 裁剪边长按预测框宽度成比例，而不是取固定像素：边长 = 预测框宽 × 16 时，装甲板在
// 640 宽的输入里恒为 640/16 = 40px，任何距离都落在实测有效区间（40px 正确、30px 仍错）。
// 固定像素只在某一个距离上有效，更远处会重新掉出可读区间。
//
// 抽成纯函数是为了脱离相机与模型单测边界裁剪行为。
inline cv::Rect dynamic_roi_rect(
    const std::optional<cv::Rect2f> & predicted_box, double roi_scale, int min_width,
    int image_width, int image_height)
{
    if (
        !predicted_box.has_value() || predicted_box->width <= 0.0F || roi_scale <= 0.0 ||
        image_width <= 0 || image_height <= 0) {
        // 没有先验就退回整帧：交由完整视野重新捕获，避免在已失效的窗口里空找。
        return cv::Rect(0, 0, image_width, image_height);
    }

    const double wanted = std::max(
        static_cast<double>(predicted_box->width) * roi_scale,
        static_cast<double>(min_width));
    int width = std::min(static_cast<int>(wanted), image_width);
    // 保持 4:3 与相机同比，缩放系数才与整帧路径一致，不引入长宽比失真。
    int height = static_cast<int>(width * 0.75);
    if (height > image_height) {
        height = image_height;
        width = static_cast<int>(height / 0.75);
    }

    int x = static_cast<int>(predicted_box->x + predicted_box->width / 2.0F - width / 2.0F);
    int y = static_cast<int>(predicted_box->y + predicted_box->height / 2.0F - height / 2.0F);
    x = std::clamp(x, 0, image_width - width);
    y = std::clamp(y, 0, image_height - height);
    return cv::Rect(x, y, width, height);
}
}  // namespace auto_aim

#endif  // AUTO_AIM__DYNAMIC_ROI_HPP
