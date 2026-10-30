# SRM Auto Aim

面向 RoboMaster 自瞄的 C++ 视觉项目，基于 TongjiSuperPower/sp_vision_25 开发。
仓库包含上游自瞄框架及仿真接入、真机上位机适配和标定工具。

## 1. 功能

- 装甲板检测、PnP 位姿解算、Tracker/EKF、Aimer 与 Planner。
- 相机采集、云台串口反馈及 USB CDC 上位机通信。
- Daedalus/ROS 2 仿真接入和离线诊断工具。
- 相机内参、云台姿态和手眼标定程序。

## 2. 相对上游的主要扩展

以下根据上游原版说明和本仓库当前代码归纳，列出主要扩展，不是完整的逐文件差异清单。

- **真机上位机链路**：增加 `standard_srm` 入口，接入 USB CDC 云台反馈；默认只读，控制和开火分别通过显式选项启用。
- **时序匹配与离线复现**：使用相机设备时间戳映射帧时序，并按图像时间匹配串口反馈；支持 JSONL 与无损图像离线重放、PlotJuggler 数据输出，以及可选 ROS 2 rosbag 录制。时序映射不代表硬件曝光同步。
- **Tracker 与几何诊断**：增加关联候选、Mahalanobis 距离和几何约束诊断，并提供 PnP、EKF、Aimer 投影误差分析；仿真配置可启用标准装甲板几何约束。
- **Planner 真机适配**：基于原有规划器加入响应延迟补偿、未来装甲板预览和最终 TX yaw 连续性处理；同板突刺受限，实际换板允许重新建立角度基准。
- **仿真与标定扩展**：接入 Daedalus/ROS 2 图像与云台 TF；标定工具支持圆点阵、棋盘格及串口云台姿态采集。

## 3. 数据流与代码结构

```text
相机图像 → Detector → PnP/Solver → Tracker → Aimer/Planner → 诊断或可选下发
云台反馈 ────────────────────────────────────────→ Planner
```

| 目录 | 内容 |
|---|---|
| `tasks/auto_aim` | 自瞄算法模块 |
| `io` | 相机、串口及设备通信 |
| `src/real_auto_aim` | 真机上位机流程 |
| `calibration` | 图像采集和标定工具 |
| `configs` | 仿真、真机和标定配置 |
| `tests`、`tools` | 测试与诊断工具 |

## 4. 编译

需要 C++17、CMake、OpenCV、Eigen3、fmt、spdlog、yaml-cpp、nlohmann-json
和 OpenVINO。ROS 2 相关目标在检测到对应依赖时构建。

```bash
cmake -S . -B build
cmake --build build --target standard_srm standard_srm_replay -j2
```

## 5. 真机程序

`standard_srm` 使用 `configs/real_auto_aim.yaml`。先检查配置：

```bash
./build/standard_srm configs/real_auto_aim.yaml --check-config=1
```

连接相机和串口后可启动只读诊断：

```bash
./build/standard_srm configs/real_auto_aim.yaml --port=/dev/ttyACM0 --show=1
```

默认不发送云台控制指令；控制和开火需显式启用对应选项。
`standard_srm_replay` 用于基于采集帧和 JSONL 的离线重放。

## 6. 配置与验证

- `configs/real_auto_aim.yaml`：真机自瞄参数。
- `configs/demo.yaml`：仿真参数。
- `configs/calibration.yaml`：采集和标定参数。

仿真、离线重放和真机闭环属于不同验证场景，结果应结合对应数据记录解读。

## 7. 文档

- [开发手册与实验记录](docs/DEVELOPMENT_GUIDE.md)：标定、仿真、真机诊断和实验记录。
- [Planner 云台反馈延迟说明](docs/planner_gimbal_feedback_delay_20261027.md)
- [Tracker 重新标定记录](docs/tracker_recalibration.md)
- [上游原版说明](readme.md)

## 许可证

上游代码遵循 MIT License，详见 `LICENSE`；模型及第三方组件的许可见
`THIRD_PARTY_NOTICES.md`。
