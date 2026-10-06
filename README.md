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

配置缺少 `pattern_type` 时默认使用 `circles`，因此旧的圆点阵配置仍可读取。

## 实车相机内参标定

采集程序只保存成功检测到完整标定板的帧；按 `s` 保存，按 `q` 退出。
相机返回空帧或当前帧未检测到标定板时不会生成样本。

只做相机内参标定时，使用 `--camera-only=1` 跳过 CAN/IMU：

```bash
./build/capture \
  configs/calibration.yaml \
  --output-folder=assets/real_intrinsics \
  --camera-only=1
```

默认姿态来源仍为 `can`，用于兼容旧的 CAN 图像/IMU 数据采集。实车 USB CDC
请使用下一节的 `--pose-source=serial`，不要使用默认 CAN 模式。

```bash
./build/capture \
  configs/calibration.yaml \
  --pose-source=can \
  --output-folder=assets/handeye_can
```

离线计算内参时，程序会检查图像分辨率一致性、有效样本数量，并输出
RMS、逐帧平均误差、P95 和最大误差：

```bash
./build/calibrate_camera \
  assets/real_intrinsics \
  --config-path=configs/calibration.yaml \
  --min-samples=12 \
  --show=0 \
  --output=configs/intrinsics_real.yaml
```

`intrinsics_real.yaml` 只包含相机内参和畸变参数，不包含相机到云台的手眼外参，
因此不会直接覆盖 `configs/demo.yaml`。手眼标定仍需在完成内参和时间同步检查后单独执行。

## USB CDC 云台姿态手眼标定

编译及无硬件测试：

```bash
cmake -S . -B build
cmake --build build --target capture calibrate_handeye gimbal_pose_test handeye_support_test srm_auto_aim_protocol_test srm_auto_aim_transport_test -j2
ctest --test-dir build --output-on-failure -R '^(gimbal_pose_test|handeye_support_test|srm_auto_aim_protocol_test|srm_auto_aim_transport_test)$'
```

固定标定板和底盘，改变云台的 yaw、pitch，分别停稳后按 `s` 保存，按 `q` 退出。
与内参采集不同，手眼采集过程中**标定板不能移动**；也不能将原来的内参照片补配
当前姿态。建议保存 15～25 个不同朝向，避免全部只转 yaw 或反复保存同一姿态。
相机倒装时保持原始图像方向，由标定求解安装旋转；不要临时旋转部分样本。

```bash
./build/capture configs/calibration.yaml \
  --pose-source=serial \
  --serial-port=/dev/ttyACM0 \
  --output-folder=assets/real_handeye
```

串口由 `SerialGimbalPose` 独占，复用已验证的 `SrmAutoAimTransport` 接收通道，
采集工具不会发送云台角度或开火指令。构造后开始接收，后台异常传回主线程；
退出时先停止并等待接收线程，再关闭串口。Fake 字节流与真实串口走相同协议解析。

反馈角度按度、右手系 ZYX（`Rz(yaw) * Ry(pitch) * Rx(roll)`）转换为
`R_gimbal2world`：`p_world = R_gimbal2world * p_gimbal`。这里使用你确认的云台
姿态，不再乘旧 CAN 路径的 `R_gimbal2imubody`。下位机仍需与此轴方向和欧拉角顺序
一致；程序不会自动猜测符号或补偿某个轴。

接收缓存最多 256 个姿态，满时丢弃最旧项。同一批串口帧没有独立源时间戳，
仅保留最后一帧并记录主机 `steady_clock` 接收时间。按 `s` 时，要求图像时间
两侧存在姿态，在四元数上插值，并检查图像前的稳定窗口：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--stable-ms` | 500 ms | 图像前必须覆盖的停稳窗口 |
| `--pose-gap-ms` | 100 ms | 最大反馈间隔、最新反馈年龄和等待后侧姿态的上限 |
| `--stable-deg` | 0.5° | 窗口内姿态相对匹配姿态的最大旋转偏差 |

这些是拒收门限，不是固定延迟补偿。当前相机时间戳为 SDK 返回图像后的主机时间，
串口反馈没有下位机采样时间，因而此流程用于**停稳采集**，不能声称实现了曝光时刻
与 IMU 的精确硬件同步。反馈缺失、过期、间隔过大或仍在运动时，本次保存被拒绝。

照片以编号 JPG 保存，对应 TXT 第一行为 `w x y z`，第二行为 `gimbal`，随后记录
图像主机时间、姿态插值间隔和稳定角偏差。旧 CAN 文件的第二行是 `imubody`；
没有第二行的历史文件按旧 CAN 数据处理。各来源必须使用独立目录。
重新采集会寻找空闲编号，已有照片和姿态不会覆盖；写入失败的 `.pending.*`
文件保留供检查，不参与离线标定。

读取独立内参并计算外参：

```bash
./build/calibrate_handeye assets/real_handeye \
  --config-path=configs/calibration.yaml \
  --intrinsics=configs/intrinsics_real.yaml \
  --min-samples=12 \
  --show=0 \
  --output=configs/handeye_real.yaml
```

程序检查配对姿态、四元数、分辨率、内参和有效样本数，允许图像编号中间有空缺。
通过相对转角和旋转轴奇异值检查拒收近似单轴数据；默认最小转角为 5°，第二/第一
奇异值下限为 0.05，可用 `--min-rotation-deg` 和 `--min-axis-ratio` 配置。
满足采集条件后使用 OpenCV Park 手眼算法，输出 `R_camera2gimbal`（按行排列）
和 `t_camera2gimbal`（米）。该旋转的方向是相机 optical → 云台。
结果文件已存在时会拒绝覆盖，请使用新文件名保存新一次标定。

此版本沿用**云台参考原点固定**的运动模型，将每张样本的 `t_gimbal2world` 设为零。
只有姿态反馈不能提供平移：若 yaw/pitch 轴不共点使参考原点移动，或底盘移动，
需补充机构运动学/平移测量，不能把这种误差归入相机安装外参。
程序同时输出标定板在 world 中的位置及朝向离散度（RMS、最大值），用于检查
`T_world_gimbal * T_gimbal_camera * T_camera_board` 是否一致；这是采集样本的内部
一致性指标，不是实车精度真值验收。程序不会改写已完成的内参或自瞄运行配置。

实现边界：`gimbal_pose.hpp` 管理纯时间/稳定性逻辑，`serial_gimbal_pose.*` 管理串口
及线程，`handeye_support.*` 管理姿态格式、坐标变换和求解，两个入口负责采集/离线流程。
自动化测试覆盖缓存丢旧策略、无数据/过期/运动拒收、角度跨 ±180°、串口分包、后台异常、
重复停止、资源释放、零发送、旧 CAN 格式，以及已知外参（含相机倒装）的合成恢复。

真机自瞄的只读接收边界另由 `real_auto_aim/SerialFeedbackReader` 提供：它独占一个
`SrmAutoAimTransport`，后台接收反馈，按图像主机时间等待后侧样本并插值云台姿态；
同时返回邻近反馈的弹速、模式和颜色。默认等待上限 30 ms、反馈间隔上限 100 ms，
无数据或过期时不提供姿态。接收器不发送任何串口指令，错误传回采集主线程；
相机现在同时保存 SDK 帧号、设备原始计数和取帧后的主机时间。仅在相机提供
`DeviceTimestampFrequency` 节点、设备计数与主机帧间隔一致且完成 16 帧预热后，
使用设备计数间隔与最短收帧延迟估计映射到主机 `steady_clock`，供反馈匹配和
Tracker 使用。该映射仍带有未知的
固定传输延迟，**不是硬件同步或精确曝光中点**。若频率节点缺失，程序在设备 tick
与主机收帧时间连续覆盖至少 2 秒后估频，随后用最近
5 秒样本每秒更新一次，并限制单次时间映射相位修正；成功后标记
`timestamp_source=estimated_device_clock`。此估计用于避免早期短样本频偏随运行时间累积，
仍不能视为真实曝光时刻，也不能消除固定 USB 传输延迟。预热或计数异常期间使用
`GetImageBuffer()` 返回时刻，并在 JSONL 标记为 `host_receive`，供只读链路继续诊断。
串口协议不带下位机采样时刻，只能用完整反馈
到达主机的时间；同批反馈只保留最后一帧。预热和映射仅影响真机只读入口，旧相机
`read()` 保持主机收帧时间供标定程序使用。无硬件测试：
`ctest --test-dir build -R '^(device_clock_mapper|real_feedback_buffer|real_serial_feedback)_test$'`。

真机自瞄入口 `standard_srm` 复用海康图像、串口反馈、YOLOv5、PnP、Tracker
和 Aimer；默认只读，同时用已有 Planner 计算诊断轨迹，不发送串口控制帧。
只有显式指定 `--enable-tx=1` 才会通过现有 USB CDC 协议发送 yaw/pitch，且
`fire_flag` 永远固定为 0。本次只修改上位机仓库，不修改下位机固件。
Planner 从串口缓存有效窗口内相距较远的两帧主机接收时间估计当前角速度，按当前云台姿态建立
MPC 初始状态，并从当前时刻生成参考轨迹。协议没有真实采样时刻或速度，所以该估计
不能视作硬件同步测量。弹速反馈为 0 时，Planner 使用配置中的
`planner_debug_bullet_speed_mps`，日志明确标记 `diagnostic_nominal`；这种结果仅供
只读对比。先运行无硬件配置检查：
当前真机配置关闭固定俯仰角的 yaw 重投影优化，Tracker 使用 PnP 原始 yaw；
仿真配置仍保留原有优化。倒装相机的模型角点可观察几何位置，但逐点同序
重投影误差不能直接当作框体对齐误差。

```bash
./build/standard_srm configs/real_auto_aim.yaml --check-config=1
```

在相机与 `/dev/ttyACM0` 均已连接时运行只读链路：

```bash
./build/standard_srm configs/real_auto_aim.yaml --port=/dev/ttyACM0
```

需要在 NUC 桌面上观察实时调试画面并保存逐帧诊断数据时：

```bash
./build/standard_srm configs/real_auto_aim.yaml \
  --port=/dev/ttyACM0 \
  --show=1 \
  --debug-jsonl=/tmp/real_srm_debug.jsonl
```

窗口中黄色为 YOLO 检测框，蓝色为当前 Tracker/EKF 装甲板投影，洋红色为 Aimer
未来瞄准投影；按 `s` 保存当前正立显示帧，按 `q` 或 `Esc` 退出。默认保存到
`/tmp/real_srm_frame.jpg`，也可以使用 `--save-frame=/path/to/frame.jpg` 指定路径。
`--show=0`（默认）适用于无桌面环境。
程序会同时记录成功帧和被跳过的帧，避免只分析检测成功样本。运行结束后在任意
有 Python 的环境离线分析：

```bash
python3 tools/real_tracker_analyzer.py \
  --input /tmp/real_srm_debug.jsonl \
  --output /tmp/real_srm_report.json
```

评估脚本会检查设备计数和帧号单调性、`mapped_age`、`mapping_delay`、串口匹配间隔、
检测率、Tracker 状态、中心速度、预测时间、瞄准角以及 `skip_reason`。它只分析记录，
不修改 Tracker 或 Aimer 参数。脚本测试：

```bash
python3 -m unittest tests/test_real_tracker_analyzer.py
```

日志中的 `diagnostic_yaw/pitch` 只是 Aimer 计算结果；默认运行时 `tx_enabled=false`，
无控制输出。启用发送后，应结合 `tx_command_sent` 和 `tx_command_mode` 判断上位机
是否将无开火目标放入发送邮箱。
`planner_status=ok` 表示求解器收敛；`unconverged_diagnostic` 表示仅有有限的数值轨迹，
此时 `planner_control=0`，不可用于控制。可比较 `planner_measured_*`、
`planner_state_*`、`planner_target_*` 与 `planner_yaw_deg/pitch_deg`，并查看
`planner_feedback_age_ms`、`planner_feedback_interval_ms`、`planner_ms`。
`planner_feedback_interval_ms` 是用于速度差分的收帧时间跨度，不是串口或相机硬件时间。
另有 `planner_*_100ms_deg`（参考与解算后 100 ms 角度）、每轴
`planner_{yaw,pitch}_solver_status`、`planner_{yaw,pitch}_solver_iterations` 和
`planner_{yaw,pitch}_{primal,dual}_residual_max`；状态 0 表示该轴收敛，残差为
TinyMPC 原始数值，不是角度误差。100 ms 角度只用于查看轨迹趋势，不是下发命令；
`planner_control=0` 不代表接近阶段没有发送，接近阶段由 `tx_command_mode=acquire`
单独标记。
云台静止且无法形成可靠差分速度时，Planner 使用图像时刻已匹配的串口 yaw/pitch，
将角速度置零，并标记 `planner_state_source=matched_pose_zero_velocity`；该状态只能
用于静止场景诊断，`planner_control` 保持为 0。存在有效差分速度时，状态来源标记为
`latest_pose_extrapolated`。
Planner 输出的是下一 10 ms 规划步的
绝对角；当前串口协议没有速度、加速度字段，日志中的规划速度与加速度不会发送。
`frame/ticks/tick_hz` 为 SDK 帧号、原始计数和相机报告的频率；`mapped_age` 与
`mapping_delay` 均基于估计的主机映射时间，不能解释为已测得的曝光/串口硬件延迟。
配置暂用 `handeye_real2.yaml` 外参、`intrinsics_real.yaml` 内参；敌方颜色由
`enemy_color` 配置决定。
模式和颜色反馈只记录原始整数，不猜测协议映射。图像尺寸不符会直接报错；姿态缺失
或图像过期时不更新 Tracker。当前海康 `Camera::read()` 在无图像时仍可能阻塞，
该入口的退出有赖于相机持续返回图像，尚未完成硬件验收。

启用上位机无开火控制时，`standard_srm` 会在 Planner 收敛后发送 MPC 角度；
Planner 尚未收敛但轨迹有限时，会发送受限步进的目标角，先让云台接近目标。
有效目标跨帧保留，避免视觉处理期间交替发送目标与零命令。短暂丢目标时最多保持
100 ms；图像时间戳无效、串口反馈缺失或轨迹非法等硬故障会立即清空命令邮箱，
随后发送 yaw=0、pitch=0、fire=0。
反馈弹速在 10–25 m/s 时使用实测值；若反馈为 0，可显式设置
`--tx-use-nominal-speed=1`，让无开火控制计算使用配置中的名义弹速。有效反馈弹速
始终优先，`fire_flag` 始终为 0。JSONL 的 `tx_speed_source` 会标明本帧来源。
`tx_command_sent` 只表示命令进入邮箱；`tx_wire_target_frames` 和
`tx_wire_zero_frames` 是串口线程成功写入的累计帧数，不等于下位机执行回执。
默认参数为 20 ms 发送周期、100 ms 命令有效期、yaw 每帧最多 2 deg、pitch 每帧最多 1 deg，
可通过命令行覆盖。这里的 `tx_command_mode=acquire` 表示接近阶段，
`tx_command_mode=tracking` 表示 Planner 已收敛后的跟踪阶段。

首次只建议进行无开火短时测试：

```bash
./build/standard_srm configs/real_auto_aim.yaml \
  --port=/dev/ttyACM0 \
  --enable-tx=1 \
  --tx-use-nominal-speed=1 \
  --show=1 \
  --debug-jsonl=/tmp/real_srm_tx_test.jsonl
```

下位机固件没有被本项目修改，也没有新增下位机超时失效机制。上位机正常退出时
会尽力补发一次零命令；USB 断开、进程崩溃或 `kill -9` 时不能保证旧命令立即失效，
因此测试时必须保持急停和断电条件可用。Fake 串口测试可用：

```bash
ctest --test-dir build -R '^real_serial_feedback_test$' --output-on-failure
```

### MPC PlotJuggler 曲线

启动 `sim_curve_plotter` 后，探针的 `/sim_aim/debug` JSON 会通过 UDP
`127.0.0.1:9870` 转发给 PlotJuggler。加载根目录的 `mpc_layout.xml`，
即可查看 MPC 参考角度、MPC 状态、实际云台 TF 姿态、速度、加速度和开火建议：

```bash
./build/sim_curve_plotter
```

其中 `mpc_target_*` 是参考轨迹，`mpc_*` 是规划器输出，
`gimbal_*` 是 TF 测得的实际云台状态，`published_fire_advice` 是最终发送给
模拟器的开火建议。

### Tracker 自动分析

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
字段使用角度；它们保留为 Aimer 输出基线。探针现在使用 `auto_aim::Planner`
中的 TinyMPC 生成实际云台轨迹，`mpc_*` 是 MPC 规划结果，`published_*` 是实际
发布到 `/rm_gimbal/cmd` 的命令。没有有效 MPC 轨迹时，探针发布
`distance=-1` 并关闭开火建议，避免模拟器继续执行旧目标。

MPC 使用 `configs/demo.yaml` 中的 `max_yaw_acc`、`max_pitch_acc`、`Q_yaw`、
`R_yaw`、`Q_pitch` 和 `R_pitch`。`mpc_yaw_vel`、`mpc_pitch_vel` 的单位为
rad/s，`mpc_yaw_acc`、`mpc_pitch_acc` 的单位为 rad/s²，`mpc_ms` 为单帧
规划耗时。`mpc_fire` 是规划器自身的误差判定；最终 `published_fire_advice`
仍需同时通过现有 Shooter 判定。

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

### 真机曝光/增益自动扫参

自动调参程序逐组启动 standard_srm，统计检测率、置信度、过曝/欠曝比例和图像清晰度。程序保持只读模式，不发送云台控制。

命令：

    python3 tools/real_exposure_gain_tuner.py \
      --repo . \
      --config configs/real_auto_aim.yaml \
      --port /dev/ttyACM0 \
      --exposures 1,2,3,4,5 \
      --gains 0,6,12,18,24 \
      --frames 120 \
      --warmup-frames 20

推荐值只写入 /tmp/real_exposure_gain_report.json，不会自动修改 YAML。确认后再把 exposure_ms 和 gain 写入配置。每组运行的 JSONL、日志默认保存在 /tmp/real_exposure_gain_tuner；需要保留原始图像时增加 --keep-frames。

4～5 米识别调参可使用有边界的自适应模式。先在同一距离、光照和目标运动条件下采 3×3 初始点，再用曝光对数与增益的二次响应曲面建议额外实测点；预测极值不直接作为推荐值。每个候选会重新启动相机并独立确认 3 次，推荐先比较三次平均检测率和最长连续漏检，再比较三次中最低的综合分数。初始点的最小/最大值就是搜索边界，模型不会外推；若最优值落在边界，需人工扩大范围重新采样。

```bash
python3 tools/real_exposure_gain_tuner.py \
  --repo . --config configs/real_auto_aim.yaml --port /dev/ttyACM0 \
  --exposures 6,7,8 --gains 8,12,16 \
  --frames 240 --warmup-frames 40 \
  --adaptive --adaptive-steps 8 --confirm-top 2 --confirm-runs 3 \
  --artifacts /tmp/exposure_4to5m \
  --output /tmp/exposure_4to5m_report.json
```

`--exposure-step-ms` 与 `--gain-step` 分别限定额外请求值的间隔（默认 0.1 ms、0.5）；它们不是已验证的相机硬件步长。每次执行会在 `--artifacts` 下创建独立 `session_*` 目录，避免重跑混用旧 JSONL。`predicted_optimum` 是曲面预测，`recommendation` 只来自有效的重复实测；退出码异常、超时或有效帧不足的记录不参与拟合和推荐。报告同时记录最长连续漏检帧数及装甲 PnP 重投影误差 P95；过曝/欠曝、清晰度仍是全图指标，不能当作灯条局部亮度。扫参不发送云台控制，也不会自动更新 YAML。

曲面使用 `u = 2(ln(e)-ln(e_min))/(ln(e_max)-ln(e_min))-1` 和 `v = 2(g-g_min)/(g_max-g_min)-1`，拟合 `b0+b1*u+b2*u²+b3*v+b4*v²+b5*u*v`；某一维固定时省略对应项。报告给出系数和样本内拟合 RMSE，不能据此保证未测点的真实得分。确认样本若一次也未检出装甲板，不会产生推荐值。自适应模式还会拒绝覆盖已有的 `--output` 报告，重跑时请换新文件名。

只改曝光/增益不需要重标内外参；改了镜头焦距、分辨率或相机安装几何时，按上方内参/手眼流程分别重标。相机参数最优值只对当次距离、目标速度和光照有效，正式采用前仍需检查原图和漏检情况。
