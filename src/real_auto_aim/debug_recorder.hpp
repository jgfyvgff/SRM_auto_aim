#ifndef REAL_AUTO_AIM__DEBUG_RECORDER_HPP
#define REAL_AUTO_AIM__DEBUG_RECORDER_HPP

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

namespace real_auto_aim
{
// 真机调试记录器按行写 JSON，便于程序运行时持续落盘，结束后可离线复盘。
// 文件由调用方拥有；每条记录立即 flush，异常退出时仍尽量保留前面的样本。
class DebugRecorder
{
public:
    explicit DebugRecorder(const std::string & path)
    {
        if (path.empty()) return;
        stream_.open(path, std::ios::out | std::ios::trunc);
        if (!stream_) throw std::runtime_error("Unable to open debug JSONL: " + path);
    }

    bool enabled() const { return stream_.is_open(); }

    void write(const nlohmann::json & sample)
    {
        if (!stream_) return;
        stream_ << sample.dump() << '\n';
        stream_.flush();
        if (!stream_) throw std::runtime_error("Failed to write debug JSONL");
    }

private:
    std::ofstream stream_;
};
}  // namespace real_auto_aim

#endif  // REAL_AUTO_AIM__DEBUG_RECORDER_HPP
