#include "src/real_auto_aim/config.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace
{
void require(bool condition, const char * message)
{
    if (!condition) throw std::runtime_error(message);
}

void require_rejected(const YAML::Node & yaml, const char * message)
{
    bool rejected = false;
    try {
        real_auto_aim::validate_real_config(yaml);
    } catch (const std::exception &) {
        rejected = true;
    }
    require(rejected, message);
}
}

int main(int argc, char * argv[])
{
    if (argc != 2) throw std::invalid_argument("Expected real config path");
    const auto yaml = YAML::LoadFile(argv[1]);
    const auto config = real_auto_aim::validate_real_config(yaml);
    require(config.image_width == 1440 && config.image_height == 1080,
            "Unexpected calibrated image size");
    require(config.max_image_age == std::chrono::milliseconds(200),
            "Unexpected image age gate");

    auto bad = YAML::Load(YAML::Dump(yaml));
    bad["camera_matrix"] = YAML::Load("[1, 2]");
    require_rejected(bad, "Short camera_matrix was accepted");

    bad = YAML::Load(YAML::Dump(yaml));
    bad["pose_frame"] = "imubody";
    require_rejected(bad, "Non-gimbal pose frame was accepted");

    bad = YAML::Load(YAML::Dump(yaml));
    bad["image_size"] = YAML::Load("[0, 1080]");
    require_rejected(bad, "Invalid image dimensions were accepted");

    bad = YAML::Load(YAML::Dump(yaml));
    bad["R_camera2gimbal"] = YAML::Load("[1, 0, 0, 0, 1, 0, 0, 0, -1]");
    require_rejected(bad, "Reflected camera rotation was accepted");

    bad = YAML::Load(YAML::Dump(yaml));
    bad["enemy_color"] = "unknown";
    require_rejected(bad, "Invalid enemy color was accepted");

    bad = YAML::Load(YAML::Dump(yaml));
    bad["yaw_optimization_enabled"] = true;
    require_rejected(bad, "Real yaw optimization was enabled");

    bad = YAML::Load(YAML::Dump(yaml));
    bad["planner_debug_bullet_speed_mps"] = 0;
    require_rejected(bad, "Invalid Planner diagnostic speed was accepted");

    bad = YAML::Load(YAML::Dump(yaml));
    bad["Q_yaw"] = YAML::Load("[1]");
    require_rejected(bad, "Short Planner yaw weights were accepted");

    std::cout << "real_config_test passed\n";
}
