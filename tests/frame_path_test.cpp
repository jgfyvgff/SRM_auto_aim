#include <filesystem>
#include <iostream>
#include <string>

#include "src/real_auto_aim/frame_path.hpp"

namespace
{
using real_auto_aim::numbered_frame_path;

bool check(bool condition, const std::string & label)
{
    if (condition) return true;
    std::cerr << label << ": 断言失败\n";
    return false;
}
}  // namespace

int main()
{
    // 带扩展名时保留原扩展名：用户明确指定 .png 就该写 png。
    if (!check(
            numbered_frame_path("imgs/real_srm_frame.png", 1).filename() ==
                "real_srm_frame_000001.png",
            "保留用户扩展名")) {
        return 1;
    }

    // 不带扩展名时补 .jpg：这正是线上"could not find a writer for the specified
    // extension"的原因——imwrite 无法从无扩展名的路径判断编码格式。
    if (!check(
            numbered_frame_path("imgs/real_srm_frame", 1).filename() == "real_srm_frame_000001.jpg",
            "缺扩展名时补 .jpg")) {
        return 1;
    }

    // 编号补零到 6 位，保证按文件名排序就是保存顺序。
    if (!check(
            numbered_frame_path("imgs/f.jpg", 12345).filename() == "f_012345.jpg", "编号补零") ||
        !check(numbered_frame_path("imgs/f.jpg", 1234567).filename() == "f_1234567.jpg",
               "超出 6 位不截断")) {
        return 1;
    }

    // 目录必须保留：保存到 imgs/ 下而不是运行目录，否则会把仓库根目录写满。
    if (!check(
            numbered_frame_path("imgs/f.jpg", 1).parent_path() == std::filesystem::path("imgs"),
            "保留父目录")) {
        return 1;
    }
    // 只有文件名（无目录）时 parent_path 为空，拼接后仍是单个文件名。
    if (!check(
            numbered_frame_path("f", 2) == std::filesystem::path("f_000002.jpg"), "无目录路径")) {
        return 1;
    }

    std::cout << "frame_path_test passed\n";
    return 0;
}
