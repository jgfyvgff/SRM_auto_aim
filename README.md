# SRM Auto Aim

本项目是面向上海大学 SRM 校内赛自瞄赛道的个人学习与仿真验证仓库。

## 上游来源

本项目基于 [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25) 开发。上游代码采用 MIT License，原始版权归 TongjiSuperPower 所有。本仓库保留上游 `LICENSE` 要求，并在此基础上记录个人修改。

## 本仓库新增内容

- Daedalus 模拟器 ROS2 数据接入；
- 仿真装甲板检测与跟踪探针；
- PnP、EKF 与未来瞄准点回投影验证；
- PlotJuggler 调试数据输出；
- 面向校内赛的通信与控制适配。

## 当前状态

当前代码仍处于仿真验证和调试阶段，Tracker 的多装甲板关联仍在验证中。请将实验性修改与上游实现区分阅读。

原始上游说明仍保留在仓库根目录的 `readme.md` 中。

## Tracker 自动分析

启动 `sim_detector_probe` 后，可使用独立脚本订阅 `/sim_aim/debug`，自动比较静止与小陀螺阶段的车辆中心、速度、半径和装甲板 ID 切换情况：

```bash
python3 tools/sim_tracker_analyzer.py --duration 30
```

建议在采集开始后先让目标静止 5～10 秒，再开启小陀螺。脚本会根据角速度自动分类，并输出中心峰峰值、半径波动、ID 切换跳变量、关联门限拒绝比例和诊断结论。

`configs/demo.yaml` 默认对 Daedalus 标准四装甲车辆启用已知半径约束。该约束只固定 `r` 和两组半径差 `l`，用于避免单块装甲观测下中心与半径不可同时观测造成的退化；其他配置默认不启用。

同一仿真配置将 yaw 重投影优化相对原始 PnP yaw 的最大修正限制为 `0.35 rad`。超过该范围时回退到原始 yaw，用于抑制矩形对称性造成的约 ±90° 错误分支；未配置此参数的其他配置保持原有优化行为。

需要保存完整 JSON 报告时使用：

```bash
python3 tools/sim_tracker_analyzer.py \
    --duration 30 \
    --output /tmp/sim_tracker_report.json
```

运行分析逻辑的硬件无关测试：

```bash
python3 -m unittest discover -s tests -p 'test_sim_tracker_analyzer.py'
```

几何约束的硬件无关测试：

```bash
./build/target_geometry_constraint_test
```

## 许可证

源代码中的上游部分遵循 MIT License，具体版权和许可条款见 `LICENSE`。模型文件和第三方组件可能具有独立许可证，见 `THIRD_PARTY_NOTICES.md`。
