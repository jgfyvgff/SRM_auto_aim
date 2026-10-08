#ifndef REAL_AUTO_AIM__FRAME_PATH_HPP
#define REAL_AUTO_AIM__FRAME_PATH_HPP

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

namespace real_auto_aim
{
// 保存显示帧/原始帧的编号路径。
//
// 为什么单独抽出来：扩展名决定 imwrite 的编码格式，路径不带扩展名时 OpenCV 无法判断格式，
// 会每保存一次就报一次 "could not find a writer for the specified extension"。这里统一补
// 默认扩展名；编号固定 6 位，保证按文件名排序就是保存顺序。
inline std::filesystem::path numbered_frame_path(
    const std::filesystem::path & base, int index, const std::string & fallback_extension = ".jpg")
{
    std::string extension = base.extension().string();
    if (extension.empty()) {
        extension = fallback_extension;
    }
    std::ostringstream name;
    name << base.stem().string() << "_" << std::setfill('0') << std::setw(6) << index << extension;
    return base.parent_path() / name.str();
}
}  // namespace real_auto_aim

#endif  // REAL_AUTO_AIM__FRAME_PATH_HPP
