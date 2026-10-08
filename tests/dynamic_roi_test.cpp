#include <iostream>
#include <optional>
#include <string>

#include "tasks/auto_aim/dynamic_roi.hpp"

namespace
{
// 相机分辨率按实车 1440x1080；ROI 必须保持 4:3，否则缩放系数与整帧路径不一致。
constexpr int kImageWidth = 1440;
constexpr int kImageHeight = 1080;
constexpr double kScale = 16.0;
constexpr int kMinWidth = 320;

bool check(bool condition, const std::string & label)
{
    if (condition) return true;
    std::cerr << label << ": 断言失败\n";
    return false;
}

bool check_rect(const cv::Rect & actual, const cv::Rect & expected, const std::string & label)
{
    if (actual == expected) return true;
    std::cerr << label << ": expected " << expected << ", got " << actual << '\n';
    return false;
}

bool is_inside(const cv::Rect & rect, int width, int height)
{
    return rect.x >= 0 && rect.y >= 0 && rect.width > 0 && rect.height > 0 &&
           rect.x + rect.width <= width && rect.y + rect.height <= height;
}
}  // namespace

int main()
{
    // 没有先验（首帧或先验已失效）时必须回退整帧，交由完整视野重新捕获。
    if (!check_rect(
            auto_aim::dynamic_roi_rect(std::nullopt, kScale, kMinWidth, kImageWidth, kImageHeight),
            cv::Rect(0, 0, kImageWidth, kImageHeight), "无先验退回整帧")) {
        return 1;
    }

    // 5.8m 实车目标框宽约 45px：16 倍得 720x540，正是实测分类全对的那组参数。
    // 目标框以图像中心为中心，便于断言 ROI 的中心对齐。
    const cv::Rect2f center_box{697.5F, 523.125F, 45.0F, 33.75F};
    const auto centered =
        auto_aim::dynamic_roi_rect(center_box, kScale, kMinWidth, kImageWidth, kImageHeight);
    if (!check_rect(centered, cv::Rect(360, 270, 720, 540), "居中目标 720x540")) return 1;

    // 目标贴左上角时 ROI 不能越界。
    const cv::Rect2f top_left{0.0F, 0.0F, 45.0F, 33.75F};
    const auto clamped_top_left =
        auto_aim::dynamic_roi_rect(top_left, kScale, kMinWidth, kImageWidth, kImageHeight);
    if (!check_rect(clamped_top_left, cv::Rect(0, 0, 720, 540), "左上角裁剪")) return 1;

    // 目标贴右下角时同样不能越界：40px × 16 = 640，窗口 640x480 贴到右下。
    const cv::Rect2f bottom_right{1400.0F, 1050.0F, 40.0F, 30.0F};
    const auto clamped_bottom_right =
        auto_aim::dynamic_roi_rect(bottom_right, kScale, kMinWidth, kImageWidth, kImageHeight);
    if (!check_rect(clamped_bottom_right, cv::Rect(800, 600, 640, 480), "右下角裁剪")) return 1;

    // 目标框远小于下限时受 min_width 约束：4px × 16 = 64，应抬到 320x240。
    const cv::Rect2f tiny_box{600.0F, 500.0F, 4.0F, 3.0F};
    const auto min_bounded =
        auto_aim::dynamic_roi_rect(tiny_box, kScale, kMinWidth, kImageWidth, kImageHeight);
    if (!check_rect(min_bounded, cv::Rect(442, 381, 320, 240), "min_width 生效")) return 1;

    // 目标框很大时（近距离）截到整帧，不能产生超出图像的区域。
    const cv::Rect2f huge_box{0.0F, 0.0F, 400.0F, 300.0F};
    const auto full_frame =
        auto_aim::dynamic_roi_rect(huge_box, kScale, kMinWidth, kImageWidth, kImageHeight);
    if (!check_rect(full_frame, cv::Rect(0, 0, kImageWidth, kImageHeight), "超大目标截到整帧")) {
        return 1;
    }

    // 高度受限时宽度必须跟着回退，否则会出现非 4:3 的窗口（1280x480 的异常输入）。
    const auto height_limited = auto_aim::dynamic_roi_rect(center_box, kScale, kMinWidth, 1280, 480);
    if (!check_rect(height_limited, cv::Rect(400, 0, 640, 480), "高度受限时宽度回退")) return 1;

    // 倍率为 0 或目标框宽为 0 都属于先验失效，必须回退整帧而不是产生空窗口。
    if (!check_rect(
            auto_aim::dynamic_roi_rect(center_box, 0.0, kMinWidth, kImageWidth, kImageHeight),
            cv::Rect(0, 0, kImageWidth, kImageHeight), "倍率为 0 退回整帧")) {
        return 1;
    }
    if (!check_rect(
            auto_aim::dynamic_roi_rect(
                cv::Rect2f(100.0F, 100.0F, 0.0F, 0.0F), kScale, kMinWidth, kImageWidth,
                kImageHeight),
            cv::Rect(0, 0, kImageWidth, kImageHeight), "零宽目标框退回整帧")) {
        return 1;
    }

    // 所有给出先验的结果都必须落在图像内。
    const cv::Rect2f probes[] = {
        {5.0F, 5.0F, 20.0F, 15.0F}, {1435.0F, 1075.0F, 5.0F, 4.0F}, {720.0F, 540.0F, 90.0F, 67.5F}};
    for (const auto & probe : probes) {
        const auto rect =
            auto_aim::dynamic_roi_rect(probe, kScale, kMinWidth, kImageWidth, kImageHeight);
        if (!check(is_inside(rect, kImageWidth, kImageHeight), "边界不越界")) return 1;
    }

    std::cout << "dynamic_roi_test passed\n";
    return 0;
}
