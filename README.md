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

Tracker 对时间间隔采用两级策略：约 `0.1 s` 只用于记录采集抖动告警，短时
间隔仍保留当前目标，并由 EKF 使用真实 `dt` 继续预测；只有超过
`configs/demo.yaml` 中的 `tracker_max_prediction_gap`（默认 `0.3 s`）才进入
`lost`，作为长时间失联保护。因此 `0.1 s` 不是固定预测延迟，也不是直接
重置 Tracker 的阈值。

仿真探针还会按图像 Header 时间查询 `odom -> gimbal_link`：有前后 TF 时由
tf2 插值；仅有最近 TF 时，时间差必须不超过 5 ms。缺少有效姿态的图像会被
跳过，不更新 Tracker。`/sim_aim/debug` 的 `gimbal_tf_skew_ms`、
`gimbal_yaw_rad`、`gimbal_pitch_rad` 用于核对实际使用的云台姿态。
当前模拟器中 `odom -> gimbal_link` 的平移恒为零，探针因此只接入动态旋转；
本验证仍以底盘位置不变为前提，尚未覆盖底盘平移或实车 IMU 坐标系。

固定位置的小陀螺 ROI 实验使用
`./build/sim_detector_probe configs/sim_probe.yaml configs/demo.yaml`。
`sim_probe.yaml` 的 ROI 只覆盖图像 x=420～1020、y=50～650；
目标移出该范围时不能使用此配置判断检测性能。

## 仿真阶段记录（2026-09-28）

本阶段已经完成或改善：

- 接入 Daedalus 的图像、TF 和调试数据，并确认测试环境使用
  `ROS_DOMAIN_ID=30` 和 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`；
- 使用图像采集时间参与 Tracker/Aimer 计算，补充过期帧、未来帧和乱序帧检查；
- 修正模拟器相机到云台的静态外参；
- 按图像采集时间把 `odom -> gimbal_link` 的云台旋转送入仿真 Solver；
- 在装甲板关联中加入 EKF 预测协方差对应的马氏距离，同时保留位置、距离和姿态安全门；
- 增加 PnP、IPPE 分支、EKF 当前状态和 Aimer 未来状态的重投影诊断；
- 固定位置小陀螺的三轮 ROI 对照中，旋转中心 x 的典型摆动约由
  0.108 m 降至 0.068 m，中心速度 P95 约由 0.223 m/s 降至 0.120 m/s，
  ID 切换中心跳变量 P95 约由 0.0565 m 降至 0.0410 m，且 ROI 三轮没有
  Tracker 重置。

上述数据只能说明固定位置、约 3.7 m 距离的小陀螺场景有所改善；
ROI 测试样本数减少，且测试不是严格同步的 A/B 实验，不能据此证明算法
已经适用于所有距离和视角。

当前仍未解决：

- 尚未验证底盘平移和实车 IMU 姿态；模拟器的 `odom` 目前与云台原点重合，
  本阶段只验证其动态旋转；
- TF 真值对照下的 PnP 距离误差尾部仍约为 0.3～0.4 m，少数帧的
  `current_ekf_error` 仍可能达到数百像素；
- 双装甲板小陀螺中的少量错误关联、姿态分支异常和中心跳变还没有完全定位；
- `prediction_dt` 已接入真实时间基准，但预测前后误差尚未在多种运动场景中
  证明稳定改善；
- Aimer 输出尚未完整闭环到云台控制和弹丸命中验证，当前粉色框只能作为
  未来瞄准位置的投影；
- 当前 ROI 只适合目标位于指定图像范围内的诊断，不能作为通用检测配置。

因此当前项目仍处于仿真验证阶段：可以输出并分析瞄准结果，但还不能宣称
已经达到小陀螺和云台运动场景下的稳定击打要求。

## Tracker 后验距离门限回归（2026-09-30）

Tracker 对通过关联的观测复用 `association_max_distance_error`（`demo.yaml`
中为 0.45 m）检查 EKF 更新后的装甲距离；越界时恢复本帧更新前的预测状态。
这只约束后验距离残差，不限制切向中心移动，也不解决断帧后的目标重建。

旋转目标的三轮 30 秒回归中，Tracker 分别重建 2、14、9 次；同一目标世代
仍可观察到约 0.41～0.60 m 的单帧中心跳变。与旧三轮结果并非同步 A/B，
目前只能确认后验距离未越过该门限，不能宣称跟踪稳定性已经改善。

## 相机标定板模式

`configs/calibration.yaml` 中的 `pattern_type` 控制采集、相机内参标定和两种手眼标定程序使用的标定板检测分支：

- `circles`：保持当前的对称圆点阵方式；
- `chessboard`：使用棋盘格内角点，并进行亚像素角点优化。

`pattern_cols` 和 `pattern_rows` 对圆点阵表示点的列数和行数；对棋盘格表示内角点列数和行数，不是方格数量。`center_distance_mm` 表示相邻圆心或相邻棋盘格内角点之间的实际距离，单位为毫米。

例如使用 9×6 内角点棋盘格时，将配置改为：

```yaml
pattern_type: "chessboard"
pattern_cols: 9
pattern_rows: 6
center_distance_mm: 25
```

配置默认为 `circles`，因此旧的圆点阵采集数据不需要迁移。

启动 `sim_detector_probe` 后，可使用独立脚本订阅 `/sim_aim/debug`，自动比较静止与小陀螺阶段的车辆中心、速度、半径和装甲板 ID 切换情况：

```bash
python3 tools/sim_tracker_analyzer.py --duration 30
```

建议在采集开始后先让目标静止 5～10 秒，再开启小陀螺。脚本会根据角速度自动分类，并输出中心峰峰值、半径波动、ID 切换跳变量、关联门限拒绝比例和诊断结论。

探针还会在每个已处理帧发布 `/sim_aim/tracker_status`。该 Topic 即使 Tracker
暂时没有有效 target 也会发布，用于统计 `temp_lost` 的连续帧数、持续时间、
Tracker 世代变化和状态消息序列缺口。分析脚本会同时订阅该 Topic；状态数据不会
混入装甲板几何误差统计。若状态消息序列存在缺口，应先检查 DDS/QoS，再解释
连续丢失帧数。

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
`association_max_angle_error` 仅用于记录固定角度诊断结果，单位为 rad，不再单独拒绝
观测。实际关联接受由综合分数、位置/距离绝对误差和 `S = HPHᵀ + R` 的 Mahalanobis
门控共同决定；角度误差仍参与综合分数，因此明显错误的姿态仍会被拒绝。

Tracker 还会计算 4 维观测创新的马氏距离：`S = HPHᵀ + R`，关联优先选择马氏距离
最小的模型装甲板。`association_max_mahalanobis_distance` 是无量纲统计门限；位置
和距离绝对误差门限仍保留作为安全门；固定角度门降级为诊断项。它不是固定处理延迟，
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

### PnP-TF 几何审计

`/sim_aim/debug` 会额外发布 `accepted_corner_*`、同帧
`association_primary_truth_*` 和完整 `gimbal_tf_q*`。这些字段只用于离线审计，
不参与 Tracker 或 Aimer 决策。可以把保存的逐行 JSON 样本交给几何审计脚本，
扫描装甲板宽度和灯条长度对 PnP-TF 真值残差的影响：

```bash
python3 tools/sim_pnp_geometry_audit.py \
    --input /tmp/sim_debug_samples.jsonl \
    --config configs/demo.yaml \
    --armor-type small \
    --output /tmp/sim_pnp_geometry_audit.json
```

审计脚本会同时输出当前配置中的 `0.135 m × pnp_lightbar_length` 基线和扫描得到的最优组合。
`pnp_lightbar_length` 是 PnP 物点中的灯条有效长度，单位为 m。未声明时默认值为
`0.056`，保持实车配置兼容；Daedalus 的 `configs/demo.yaml` 使用 `0.060`，因为模型中
可见灯条高度约为 59.6 mm。该参数只影响 PnP 物点，不改变 YOLO 输出或 TF。
审计脚本的主 `best` 指标按样本中的 `association_primary_optimized_yaw` 选择
与在线 Tracker 接近的 IPPE 分支；报告同时保留 `truth_best_*`，仅用于判断
“是否存在某个姿态分支能够解释 TF 真值”，不能把它当作在线改进结果。只有在多个
视角和距离下重复确认后，才可以考虑修改 `solver.cpp` 中的 PnP 物点常量。

小陀螺时多个 `armor_N` 可能在世界坐标中距离接近，因此探针按真值中心投影到图像后
与检测中心的像素误差选择真值身份；`association_primary_truth_match_margin` 表示
第一、第二投影候选的像素误差差值，不再表示世界坐标最近邻的距离差。

`association_primary_truth_corner_error_0..3` 和对应的 `truth_corner_error` 使用 TF 发布的
完整装甲板姿态，将当前装甲板物点四角投影后与同一检测框逐点比较。它们只用于区分
角点点序、相机外参、装甲板尺寸和 PnP 深度误差，不参与 Tracker 或 Aimer 决策。

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
