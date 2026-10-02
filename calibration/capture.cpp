#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <opencv2/opencv.hpp>

#include "calibration/calibration_pattern.hpp"
#include "calibration/handeye_support.hpp"
#include "calibration/serial_gimbal_pose.hpp"
#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
    "{help h usage ?  |                          | 输出命令行参数说明}"
    "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
    "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }"
    "{camera-only     | 0                        | 兼容旧参数：仅采集图像 }"
    "{pose-source     | can                      | camera-only、can 或 serial }"
    "{serial-port     | /dev/ttyACM0             | USB CDC 串口设备 }"
    "{stable-ms       | 500                      | 保存前姿态稳定窗口，毫秒 }"
    "{pose-gap-ms     | 100                      | 允许的最大反馈间隔，毫秒 }"
    "{stable-deg      | 0.5                      | 稳定窗口最大角偏差，度 }";

void write_q(const std::string q_path, const calibration::PoseRecord & pose,
             const calibration::PoseMatch & match, calibration::PoseClock::time_point timestamp)
{
    std::ofstream q_file(q_path);
    if (!q_file) throw std::runtime_error("无法创建姿态文件: " + q_path);
    // 输出顺序为wxyz
    calibration::write_pose(q_file, pose);
    if (pose.frame == calibration::PoseFrame::Gimbal) {
        q_file << fmt::format("# image_host_s={} bracket_ms={} spread_deg={}\n",
                              std::chrono::duration<double>(timestamp.time_since_epoch()).count(),
                              match.bracket_ms, match.spread_deg);
    }
    q_file.close();
    if (!q_file) throw std::runtime_error("姿态文件写入失败: " + q_path);
}

void capture_loop(
    const std::string & config_path, const std::string & output_folder,
    const calibration::PatternSpec & pattern, const std::string & pose_source,
    const std::string & serial_port, const calibration::PosePolicy & pose_policy)
{
    const bool camera_only = pose_source == "camera-only";
    std::unique_ptr<io::CBoard> cboard;
    std::unique_ptr<calibration::SerialGimbalPose> serial_pose;
    if (pose_source == "can")
        cboard = std::make_unique<io::CBoard>(config_path);
    else if (pose_source == "serial")
        serial_pose = std::make_unique<calibration::SerialGimbalPose>(
            io::srm_auto_aim::SerialConfig{serial_port, 20}, pose_policy);

    io::Camera camera(config_path);
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;

    int count = 0;
    while (true) {
        if (serial_pose) serial_pose->rethrow_if_failed();
        camera.read(img, timestamp);
        if (img.empty()) {
            tools::logger()->warn("相机未返回有效图像，跳过本帧");
            continue;
        }

        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        if (cboard)
            q = cboard->imu_at(timestamp);

        auto img_with_ypr = img.clone();
        if (cboard) {
            // 手眼标定采集时显示 IMU 姿态，用于检查坐标轴方向和零漂。
            Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3; // degree
            tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
            tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
            tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});
        } else if (serial_pose) {
            tools::draw_text(img_with_ypr, "Serial gimbal: hold still, press s", {40, 40}, {0, 255, 0});
        } else {
            tools::draw_text(img_with_ypr, "Camera only", {40, 40}, {0, 255, 0});
        }
        // 先显示原始画面，避免棋盘格检测耗时导致用户误以为相机没有输出。
        cv::Mat preview;
        cv::resize(img_with_ypr, preview, {}, 0.5, 0.5);
        cv::imshow("Press s to save, q to quit", preview);
        const auto first_key = cv::waitKey(1);
        if (first_key == 'q')
            break;


        std::vector<cv::Point2f> centers_2d;
        auto success = calibration::detect_pattern(img, pattern, centers_2d);
        calibration::draw_pattern(img_with_ypr, pattern, centers_2d, success);
        cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5); // 显示时缩小图片尺寸

        // 按“s”保存图片和对应四元数，按“q”退出程序
        cv::imshow("Press s to save, q to quit", img_with_ypr);
        const auto second_key = cv::waitKey(1);
        const auto key = second_key == 'q' ? 'q' : (first_key == 's' ? 's' : second_key);
        if (key == 'q')
            break;
        else if (key != 's')
            continue;

        if (!success) {
            tools::logger()->warn("当前帧未检测到完整标定板，不保存");
            continue;
        }

        calibration::PoseMatch match;
        if (serial_pose) {
            try {
                match = serial_pose->sample_at(timestamp);
                q = match.q;
            } catch (const std::exception & error) {
                // 串口线程故障是终止错误；暂时缺数据/未停稳仅拒收这一张，继续预览。
                serial_pose->rethrow_if_failed();
                tools::logger()->warn("本次未保存: {}", error.what());
                continue;
            }
        }

        // 保存图片和四元数
        auto next_count = count + 1;
        // 重新启动采集时追加编号，避免覆盖已经采集的标定数据和上次失败留下的暂存文件。
        while (std::filesystem::exists(fmt::format("{}/{}.jpg", output_folder, next_count)) ||
               std::filesystem::exists(fmt::format("{}/{}.txt", output_folder, next_count)) ||
               std::filesystem::exists(fmt::format("{}/{}.pending.jpg", output_folder, next_count)) ||
               std::filesystem::exists(fmt::format("{}/{}.pending.txt", output_folder, next_count))) {
            ++next_count;
        }
        const auto stem = fmt::format("{}/{}", output_folder, next_count);
        if (!cv::imwrite(stem + ".pending.jpg", img)) {
            throw std::runtime_error("无法保存标定图像: " + stem);
        }
        if (!camera_only) {
            const auto frame = serial_pose ? calibration::PoseFrame::Gimbal : calibration::PoseFrame::ImuBody;
            write_q(stem + ".pending.txt", {q, frame}, match, timestamp);
            std::filesystem::rename(stem + ".pending.txt", stem + ".txt");
        }
        // 只有姿态成功落盘才公布 JPG；异常时保留 pending 文件供检查，不计作有效样本。
        std::filesystem::rename(stem + ".pending.jpg", stem + ".jpg");
        count = next_count;
        tools::logger()->info("[{}] Saved in {}", count, output_folder);
        if (serial_pose) {
            tools::logger()->info("gimbal→world，姿态间隔={:.2f}ms，稳定角偏差={:.4f}deg",
                                  match.bracket_ms, match.spread_deg);
        }
    }

    // 离开该作用域时，camera和cboard会自动关闭
    // serial_pose 析构先停止接收线程，再关闭串口；不依赖相机的退出顺序。
}

int main(int argc, char * argv[])
{
    try {
        // 读取命令行参数
        cv::CommandLineParser cli(argc, argv, keys);
        if (cli.has("help")) {
            cli.printMessage();
            return 0;
        }
        auto config_path = cli.get<std::string>(0);
        auto output_folder = cli.get<std::string>("output-folder");
        auto camera_only = cli.get<bool>("camera-only");
        auto pose_source = cli.get<std::string>("pose-source");
        const auto serial_port = cli.get<std::string>("serial-port");
        calibration::PosePolicy pose_policy;
        pose_policy.stable_window = std::chrono::milliseconds(cli.get<int>("stable-ms"));
        pose_policy.max_gap = std::chrono::milliseconds(cli.get<int>("pose-gap-ms"));
        pose_policy.max_spread_deg = cli.get<double>("stable-deg");
        if (!cli.check()) {
            cli.printErrors();
            return 1;
        }
        if (camera_only) {
            if (cli.has("pose-source") && pose_source != "camera-only") {
                throw std::invalid_argument("--camera-only=1 不能与 can/serial 姿态来源同时指定");
            }
            pose_source = "camera-only";
        }
        if (pose_source != "camera-only" && pose_source != "can" && pose_source != "serial") {
            throw std::invalid_argument("pose-source 必须为 camera-only、can 或 serial");
        }
        pose_policy.validate();
        auto calibration_yaml = YAML::LoadFile(config_path);
        const auto pattern = calibration::load_pattern_spec(calibration_yaml);

        // 采集程序可能首次运行在全新的工作区，递归创建目录避免保存时失败。
        std::error_code error;
        std::filesystem::create_directories(output_folder, error);
        if (error) {
            throw std::runtime_error(
                      fmt::format("无法创建标定输出目录 {}: {}", output_folder, error.message()));
        }

        // 内参照片不能补配本次姿态；各来源使用独立目录，避免不同坐标系混入同一数据集。
        for (const auto & entry : std::filesystem::directory_iterator(output_folder)) {
            if (entry.path().extension() != ".jpg") continue;
            const auto stem = entry.path().stem().string();
            if (stem.empty() || stem.find_first_not_of("0123456789") != std::string::npos) continue;
            auto pose_path = entry.path();
            pose_path.replace_extension(".txt");
            if (pose_source == "camera-only") {
                if (std::filesystem::exists(pose_path)) throw std::runtime_error("该目录已有手眼数据，请使用独立内参目录");
            } else {
                std::ifstream input(pose_path);
                if (!input) throw std::runtime_error("已有照片缺少配对姿态，请使用新的手眼采集目录");
                const auto pose = calibration::read_pose(input);
                if ((pose.frame == calibration::PoseFrame::Gimbal) != (pose_source == "serial")) {
                    throw std::runtime_error("已有样本姿态来源不同，请使用独立目录");
                }
            }
        }

        tools::logger()->info(
            "标定板模式: {}, 尺寸: {}列{}行", calibration::pattern_type_name(pattern.type),
            pattern.size.width, pattern.size.height);
        // 主循环，保存图片和对应四元数
        tools::logger()->info("姿态来源: {}", pose_source);
        capture_loop(config_path, output_folder, pattern, pose_source, serial_port, pose_policy);

        tools::logger()->warn("注意四元数输出顺序为wxyz");

        return 0;
    } catch (const std::exception & error) {
        fmt::print(stderr, "采集失败: {}\n", error.what());
        return 1;
    }
}
