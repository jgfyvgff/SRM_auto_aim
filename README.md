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

模拟器 `/image_raw` 的 Header 使用系统时钟记录采集时刻。探针将它映射为
单调时钟后交给 Tracker/Aimer；无效、未来、超过一秒的时间戳和乱序旧帧会被丢弃。
`image_header_age_ms` 是图像到达回调时的年龄；
`capture_to_detector_ms`、`capture_to_aimer_ms` 从采集时刻起算。
一秒仅是过期帧拒绝上限，不是预测延迟补偿。

启动 `sim_detector_probe` 后，可使用独立脚本订阅 `/sim_aim/debug`，自动比较静止与小陀螺阶段的车辆中心、速度、半径和装甲板 ID 切换情况：

```bash
python3 tools/sim_tracker_analyzer.py --duration 30
```

建议在采集开始后先让目标静止 5～10 秒，再开启小陀螺。脚本会根据角速度自动分类，并输出中心峰峰值、半径波动、ID 切换跳变量、关联门限拒绝比例和诊断结论。

报告中的 `Aimer 时间对齐预测` 会把时刻 `t` 生成的未来瞄准点，与
`t + prediction_dt` 附近相同 Tracker 世代、相同装甲板模型 ID 的已接收 PnP
观测进行比较。`不预测位置误差` 使用该瞄准 ID 在时刻 `t` 的模型位置作为基线，
`预测后位置误差` 使用 Aimer 的未来位置；只有后者更小时，才能说明当前运动预测
对该样本产生了正收益。默认允许的时间对齐误差为 `0.03 s`，可通过
`--prediction-match-tolerance` 调整。

Tracker 对每个同名检测框分别以各模型 ID 的预测 yaw 选择 IPPE 姿态分支，再计算
该分支与模型 ID 的关联误差。通过全部门限后，同一帧只使用马氏距离最小的一个组合
更新 EKF，避免两个机器人同时进入画面时把同名装甲板一起写入同一个目标；其余
检测候选仍会发布到 `/sim_aim/debug` 供诊断。
`association_model_candidates` 额外记录每个检测框对各模型 ID 的门限、马氏距离、
位置/姿态误差和该 ID 所选的 IPPE yaw，按检测框与模型 ID 的遍历顺序保存；
最多保留八项，超出时只丢弃诊断记录，不影响实际关联。它用于分析初始
模型 ID 锁定，不参与 Tracker 决策，也不能当作物理装甲板编号。
关联门限通过后，还会比较同一块装甲板更新前后的三维位置残差。已知几何目标
使用两组装甲中较大半径的旋转直径作为残差恶化上限；超过时恢复本帧预测状态与
EKF 统计。其他目标仍使用本帧距离观测噪声的一个标准差。拒绝帧不增加装甲切换计数。
`/sim_aim/debug` 中该候选可表现为 `gate_passed=true`、`accepted=false`。
`configs/demo.yaml` 中的
`association_max_score` 以等效弧度为单位，综合装甲板角度误差、按目标距离归一化的
位置误差和距离误差，超限时保留预测状态而不吸收坏观测。为避免远距离下角度归一化
掩盖较大的米制误差，`association_max_position_error` 和
`association_max_distance_error` 还分别限制三维位置、距离残差，单位为 m。
`association_max_angle_error` 单独限制装甲板姿态与视线方位的综合误差，单位为 rad，
用于拒绝位置误差尚未超限但朝向明显错误的观测。

Tracker 还会计算 4 维观测创新的马氏距离：`S = HPHᵀ + R`，关联优先选择马氏距离
最小的模型装甲板。`association_max_mahalanobis_distance` 是无量纲统计门限；原有
角度、位置和距离绝对误差门限仍保留，作为安全门和诊断项。它不是固定处理延迟，
也不替代基于真实时间戳的延迟测量与预测时间对齐。

该指标使用仿真视觉观测作为近似真值，适合比较预测前后的相对效果；它仍包含 PnP
噪声、时间戳误差和装甲板短时不可见造成的影响，不能替代真实弹丸落点测试。

报告还会显示 Aimer 实际进入高速模式的比例，以及观测到 Aimer 的实测处理延迟、基础预测时间和弹丸
飞行时间。小陀螺分类阈值与 Aimer 的 `decision_speed` 不是同一个概念；只有
`Aimer高速模式帧` 才表示进入了高速选板分支。带符号 yaw 残差为“预测角度减去
未来实测角度”，等效 `dt` 修正用于判断当前时间补偿是偏超前还是偏滞后。

`/sim_aim/debug` 中的 `command_yaw`、`command_pitch` 使用弧度，带 `_deg` 后缀的
字段使用角度；它们只是 Aimer 输出诊断，不会自动向模拟器发布云台控制命令。

`association_primary_*_error` 是 EKF 更新前的关联残差；`post_update_*_error`
使用同一帧实际接受的装甲板计算 EKF 更新后的残差。未接受或无法配对时值为 `-1`。
分析器统计均值和分位数时会排除这些无效的 `-1` 值。
仅在该帧确实接受装甲板并完成 EKF 更新时，`/sim_aim/debug` 才会发布
`ekf_innovation_0..3`、对应的 `ekf_R_diagonal_0..3`、`ekf_S_diagonal_0..3`，以及
`ekf_center_x_correction_0..3`、`ekf_center_y_correction_0..3`。观测下标依次为方位角、
俯仰角、距离和装甲板 yaw；修正量是 `K(row, i) * innovation[i]`，单位为 m，可用于检查
某个观测分量是否主导了车辆中心的跳变。这些字段只增加诊断，不改变滤波行为。
分析器会分别统计四个观测通道的创新、R/S 对角项，以及
`ekf_center_correction_norm_0..3`（对应 x/y 中心修正量的二维模长，单位 m）；
中心跳变和 EKF 误差异常帧也会保留这些通道数据，便于追查单帧异常来源。
异常帧还会保留主、次候选各项关联门限的通过状态，以区分候选被拒绝的具体原因。
现有 `pnp_error` 按 Aimer 未来投影选择装甲板，不保证与 `current_ekf_error` 使用
同一个检测框，因此两者不能直接作为同框的前后验对比。
`accepted_pnp_error` 是已接收框的原始 PnP 回投影误差；
`accepted_model_error` 是该框的 PnP 位置加当前固定俯仰角模型的回投影误差。
两者与 `current_ekf_error` 使用同一个框，仅用于区分 PnP、姿态模型和滤波误差。

`measurement_bearing_variance` 是方位观测噪声方差（rad²），同时用于关联创新
协方差和 EKF 更新。仅仿真 `demo.yaml` 暂用 `4e-5` 做对照实验；未配置时保持
原值 `4e-3`。是否保留该值取决于同场景下的误差、拒绝率和重初始化次数。

分析器对已接收观测检查 `current_ekf_error / armor_pixel_long_side`。
默认超过 `1.0` 时报警，可用 `--ekf-reprojection-armor-ratio` 调整；
缺少有效装甲板像素尺寸的帧不参与此项判断。报警只说明后验投影
偏离检测框，不能单凭它判定是关联、PnP 还是 EKF 更新造成的。

报告中的 `[距离分段]` 默认按 `1 m` 对 `target_distance` 分桶，分别统计装甲板
像素尺寸、PnP/EKF 回投影误差、关联位置/距离误差、中心速度和 Tracker 世代数。
这样可以区分近距离和远距离的误差放大，而不是把它们混在同一个 P95 中。需要调整
分桶宽度时可使用 `--range-bin-size 0.5`；距离分段只做诊断，不改变 Tracker 或
Aimer 的参数。

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
