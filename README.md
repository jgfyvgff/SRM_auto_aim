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
只有显式指定 `--enable-tx=1` 才会通过现有 USB CDC 协议发送 yaw/pitch。
开火请求还需额外指定 `--enable-fire=1`；默认关闭。启用后，仅当 Planner 收敛、
`fire_thresh` 判据通过且未处于 temp_lost 外推时发送 `fire_flag=1`。反馈弹速无效时，
仅启用 `--tx-use-nominal-speed=1` 才允许命令发送；此时开火判据使用配置中的名义弹速。
保持角和异常回退不会请求开火。本次只修改上位机仓库，不修改下位机固件。
Planner 从串口缓存有效窗口内相距较远的两帧主机接收时间估计当前角速度。
正常跟踪时 MPC 仍以参考轨迹建立初态并生成绝对角度指令；反馈角目前用于
跟随误差诊断和开火对准检查。协议没有真实采样时刻或速度，所以差分估计不能视作硬件同步测量。
真机配置 `planner_gimbal_response_delay_ms` 表示 TX 到反馈响应的有效滞后估计，当前设为
35 ms；Planner/TX 已有约 20 ms 目标前视，因此额外前推约 15 ms。这个时延来自主机日志的
有效滞后估计，不是下位机硬件时间戳或纯机械死区测量，需用后续运动日志复核。
未收敛时的目标参考也使用同一 20 ms 输出时刻，避免和收敛 MPC 命令差一个规划步。
弹速反馈为 0 时，Planner 使用配置中的
`planner_debug_bullet_speed_mps`，日志明确标记 `diagnostic_nominal`；默认仅供
只读对比，显式启用 `--tx-use-nominal-speed=1` 后可用于 TX 与开火判据。
先运行无硬件配置检查：
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

需要现场录制图像及对应的云台四元数时，可增加 `--rosbag-dir`。HikRobot 相机写入
传感器原始 Bayer8 图像，其他相机写入 BGR8；目录必须尚不存在。离线转换 Bayer 图像时，
应按消息的 `encoding` 去马赛克；HikRobot 使用的 Bayer 排列映射与实时 OpenCV 转换一致。
图像和 `/gimbal/orientation` 使用同一个 ROS 时间戳，姿态是按图像时间从串口反馈缓存
插值得到的 `gimbal_to_world`：

```bash
./build/standard_srm configs/real_auto_aim.yaml \
  --port=/dev/ttyACM0 \
  --rosbag-dir=/tmp/real_srm_capture
ros2 bag info /tmp/real_srm_capture
ros2 bag play /tmp/real_srm_capture
```

录制队列上限为 16 帧；磁盘写入在后台线程执行。队列满或写盘失败时停止 rosbag 录制并报告，
视觉跟踪继续运行。没有有效图像时间或匹配串口姿态的帧不会写入。时间戳保留 steady clock
的帧间隔，并映射到进程启动时的系统时钟基准；串口协议没有下位机采样时刻，因此该姿态
仍是主机反馈插值，不代表曝光瞬间的硬件同步姿态。rosbag 可供 ROS2 工具和订阅节点回放；
本仓库的 `standard_srm_replay` 仍读取 `--debug-jsonl` 与 `--replay-frame-dir` 配套数据，
不会直接从 rosbag 重跑算法。

#### Bayer rosbag 转换与离线重跑

`standard_srm_bayer_prepare` 订阅 `/camera/image_raw` 和 `/gimbal/orientation`，
按消息 Header 时间戳配对，生成无损 BGR PNG、回放输入 JSONL，并补齐 Aimer 与
Planner 的回放输入。bag 播放结束且超过空闲超时后，节点会检查配对数量和 JSONL
帧数；不一致会失败退出。订阅队列深度为 64 并使用 Reliable QoS；仍建议低速播放，
让大图像的转换和写盘有足够余量。

终端一启动转换订阅节点：

```bash
source /opt/ros/humble/setup.bash
./build/standard_srm_bayer_prepare \
  --source-jsonl=/home/ljq/Documents/ChatGPT/sim_aim/captures/bayer_20261023_051143.jsonl \
  --frames-dir=/home/ljq/Documents/ChatGPT/sim_aim/captures/bayer_20261023_051143_bgr \
  --output-jsonl=/home/ljq/Documents/ChatGPT/sim_aim/captures/bayer_20261023_051143_replay.jsonl
```

终端二播放原始 bag：

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /home/ljq/Documents/ChatGPT/sim_aim/captures/bayer_20261023_051143 --rate 0.05
```

转换成功后，在 PlotJuggler 启动 JSON UDP Server 并监听 `127.0.0.1:9871`，
然后运行离线算法回放：

如果当前机器没有可用的 OpenVINO GPU，将配置副本中的 `device` 改为 `CPU`，
再把下面命令的配置路径指向该副本；不要修改真机配置。

```bash
./build/standard_srm_replay configs/real_auto_aim.yaml \
  --input-jsonl=/home/ljq/Documents/ChatGPT/sim_aim/captures/bayer_20261023_051143_replay.jsonl \
  --frames-dir=/home/ljq/Documents/ChatGPT/sim_aim/captures/bayer_20261023_051143_bgr \
  --output-jsonl=/home/ljq/Documents/ChatGPT/sim_aim/captures/bayer_20261023_051143_result.jsonl \
  --show=1 --debug-udp
```

离线回放不打开相机或串口；它从头运行当前 Detector、Tracker、Aimer 和可重建
Planner 输入的代码。`--show=1` 打开叠图窗口，`--debug-udp` 将逐帧结果发送给
PlotJuggler。PNG 和结果 JSONL 会占用额外磁盘空间，输出路径须预先留空.

窗口中黄色为检测框，绿色为依据同帧 PnP 和半径先验拟合的四板整车模型；两块检测框
会共同约束车体中心和姿态，并显示两者的中心差、姿态差。只有一块检测时会标记为
`single (unverified)`，表示缺少第二块板的独立几何校验。按 `q` 或 `Esc` 退出。默认保存到
`/tmp/real_srm_frame.jpg`，也可以使用 `--save-frame=/path/to/frame.jpg` 指定路径。
`--show=0`（默认）适用于无桌面环境。

只核对一帧双检测的整车几何时，可设置 `--model-frame-id=88`。回放仍从头处理完整数据，
但只在指定帧显示四个绿色板框，不叠检测框或状态文字；窗口会停在该帧，按非 `q`/`Esc` 键后继续。
使用 `--model-frame-output=/path/to/model.png` 可另存这张单帧图；指定帧必须有两块可联合拟合的检测。
标准四装甲首次捕获时，Aimer 的低速选板锁从直接观测的 ID0 开始；目标丢失后清锁。
这避免捕获初期短暂关联相邻板后长期瞄向该板。相邻板自身的模型投影偏差仍需单独核查。
程序会同时记录成功帧和被跳过的帧，避免只分析检测成功样本。运行结束后在任意
有 Python 的环境离线分析：

```bash
python3 tools/real_tracker_analyzer.py \
  --input /tmp/real_srm_debug.jsonl \
  --output /tmp/real_srm_report.json
```

评估脚本会检查设备计数和帧号单调性、`mapped_age`、`mapping_delay`、串口匹配间隔、
检测率、Tracker 状态、中心速度、预测时间、瞄准角以及 `skip_reason`。它只分析记录，
不修改 Tracker 或 Aimer 参数。报告将 Tracker 非 tracking 状态、实际保持指令和
`temp_lost` 外推指令分开统计。关联统计保留实际门控失败项；固定 `angle_gate` 仅供
诊断，不会被报告为拦截原因。检测输出为空也不能单独证明目标离开视野，需结合原图。
脚本测试：

```bash
python3 -m unittest tests/test_real_tracker_analyzer.py
```

日志中的 `diagnostic_yaw/pitch` 只是 Aimer 计算结果；默认运行时 `tx_enabled=false`，
无控制输出。启用发送后，应结合 `tx_command_sent` 和 `tx_command_mode` 判断上位机
是否将无开火目标放入发送邮箱。
无有效瞄点时这些角度和 `aim_armor_id` 记为 null；转换旧日志到 CSV 时，
`aim_control=0` 的占位零角度也会转为 nan，避免出现虚假的 0° 尖峰。
`planner_status=ok` 表示求解器收敛；`unconverged_diagnostic` 表示求解器未收敛，
但仍记录有限的诊断轨迹。`planner_control` 只表示 MPC 控制判据，不是命令发送标志；
当前 Aimer 选板有效且求解收敛时，Planner 以参考轨迹为初态，预览未来换板，
发送 20 ms 规划角；未收敛时跟随 `planner_command_target_yaw/pitch_deg`，它使用同一输出时刻。
差分估计的云台速度只作诊断，不再是发送条件。选中的 MPC 规划角或目标参考角直接进入 TX，
发送链路不再叠加速率或单步限幅。
可比较 `planner_measured_*`、
`planner_state_*`、`planner_target_*` 与 `planner_yaw_deg/pitch_deg`，并查看
`planner_feedback_age_ms`、`planner_feedback_interval_ms`、`planner_ms`。
`planner_feedback_interval_ms` 是用于速度差分的收帧时间跨度，不是串口或相机硬件时间。
另有 `planner_*_100ms_deg`（参考与解算后 100 ms 角度）、每轴
`planner_{yaw,pitch}_solver_status`、`planner_{yaw,pitch}_solver_iterations` 和
`planner_{yaw,pitch}_{primal,dual}_residual_max`；状态 0 表示该轴收敛，残差为
TinyMPC 原始数值，不是角度误差。100 ms 角度只用于查看轨迹趋势，不是下发命令；新增
planner_yaw_20ms_deg/pitch_20ms_deg 用于核对 MPC 规划角与 TX 指令。
`planner_gimbal_response_delay_ms` 和 `planner_target_lead_compensation_ms` 分别记录配置的总滞后
与扣除现有 20 ms 前视后的额外目标前推；`planner_command_target_yaw/pitch_deg` 是未收敛时
实际使用的目标参考角。
`planner_preview_switch_ms` 记录预测窗口内首次换板距当前的时间；无换板时为 null。
`tx_command_source` 标记 `mpc_reference_20ms`、`target_reference` 或 `hold`；
`planner_control=0` 不代表没有发送角度命令，实际发送状态由 `tx_command_sent` 标记，
普通跟踪命令使用 `tx_command_mode=tracking`。
`planner_feedback_yaw_error_deg` 和 `planner_feedback_pitch_error_deg` 是规划时刻的
参考角减调用方估计的反馈角，正值表示参考角领先；yaw 误差已按 ±180° 折返。
`planner_fire_alignment_error_deg` 是两轴误差的合成值；它小于 `fire_thresh`
才允许 Planner 申请开火，且不改变下发角。
`planner_fire_reference_error_deg` 记录约 30 ms 处规划角与参考角的差，也使用
`fire_thresh`。需结合 `planner_state_source` 判读：
`matched_pose_zero_velocity` 没有同步到规划时刻的实测速率。
云台静止且无法形成可靠差分速度时，Planner 使用图像时刻已匹配的串口 yaw/pitch，
将角速度置零，并标记 `planner_state_source=matched_pose_zero_velocity`；规划是否
可控仍由求解结果决定。存在有效差分速度时，状态来源标记为
`latest_pose_extrapolated`。
`planner_yaw_deg/pitch_deg` 是下一 10 ms 规划步的状态角；正常跟踪模式的首步角度
由参考轨迹决定。检查 MPC 波形时应对比同为 100 ms 的
`planner_*_100ms_deg` 与 `planner_target_*_100ms_deg`，并同时查看求解状态。
实际下发角看 `tx_command_yaw_deg/pitch_deg`；串口协议没有速度、加速度字段。
启用 `--debug-jsonl` 后，每帧另附 `wire_trace_events`：`tx_target/tx_zero`
记录主机串口写入的起止时间与实际写入角度，`rx_feedback` 逐条记录一次
读取所解析出的全部反馈帧，包括 yaw/pitch/roll、原始 mode/color、弹速和
批内顺序。同批帧共用一次读取的主机时间区间；控制缓存仍只取最后一帧，
避免为没有协议时间戳的反馈编造采样间隔。`*_steady_ns` 只用于本次进程内
的时间差计算；`wire_trace_dropped_total` 非零表示有界事件缓存曾丢弃旧事件。
这些字段可估计“主机写入到主机收到反馈”的整体滞后，不能单独证明下位机
采样时刻或电机执行延迟；`tx_command_sent` 仍只表示命令进入邮箱。
规则定义 `color` 字段承载机器人 ID，并称参考固件的 `mode` 固定为 0；
这两个字段原样记录，不用 `mode` 推断云台是否上电或进入视觉控制。
真机入口将本帧 Aimer 已选的装甲板 ID 传给 Planner，整条预测轨迹固定该物理板；
当 Aimer 未给出有效瞄点时，Planner 保留原有最近距离选板行为。日志中的
`aim_armor_id`、`planner_target_yaw_deg` 和 `diagnostic_yaw_deg` 可用于检查两者是否一致。
`frame/ticks/tick_hz` 为 SDK 帧号、原始计数和相机报告的频率；`mapped_age` 与
`mapping_delay` 均基于估计的主机映射时间，不能解释为已测得的曝光/串口硬件延迟。
配置使用 `handeye_real6.yaml` 外参、`intrinsics_real6.yaml` 内参；敌方颜色由
`enemy_color` 配置决定。
Tracker 捕获阶段的类别投票须通过当前目标关联与 EKF 后验后才改变观测标签；
若误捕获为 `outpost_three` 后投票改为标准四装甲类别，则用已接受的姿态重建
`standard_four`，并重新累计捕获帧；其余改判保持原 EKF 和几何。首次捕获未标定
几何类别时不产生跟随目标。真机 JSONL
用 `tracker_geometry_branch`、`tracker_geometry_trusted`、
`tracker_geometry_constraint_active`、`tracker_radius_variance_m2`、
`tracker_last_nis`、`tracker_recent_nis_failures` 诊断模型；仅当
`tracker_nis_fresh=true` 时，`tracker_last_nis` 属于本帧。
`tracker_observation_accepted` 包含首次捕获，`tracker_model_name` 与
`tracker_tracked_label` 分别表示保留的物理模型和当前接受的检测标签。
`standard_radius` 与 `standard_radius_variance` 设置标准四装甲模型的半径初值和
EKF 初始方差；关闭 `standard_geometry_constraint` 时，它们提供软先验，不会在每帧
强制固定半径。`standard_radius_delta_constraint` 可单独固定两组装甲的半径差；
启用时半径差固定为 0，半径 `r` 仍可继续由观测修正。
`tracker_geometry_constraint_active` 表示当前至少有一项几何尺寸约束生效。
标准四装甲的候选还须满足图像中心误差不超过本帧检测框长边；
同号相邻板即使通过三维关联评分，也不能从远处更新同一模型 ID。
被拒绝的帧会进入现有暂失流程，可能增加短时保持帧，需用新采集复核。
模式和颜色反馈只记录原始整数，不猜测协议映射。图像尺寸不符会直接报错；姿态缺失
或图像过期时不更新 Tracker。真机入口使用 50 ms 限时取帧；无图像时可响应退出，
但相机 SDK 自身线程的停止行为仍需真机验收。

真机入口的线程职责如下：相机驱动线程负责取图；采集线程串行完成帧时标、图像质量
和 YOLO 检测，保留动态 ROI 的前帧状态；算法线程串行完成反馈匹配、PnP、Tracker、
Aimer、Planner 和命令限速；串口线程独占 USB 读写；主线程独占 JSONL、PNG/JPEG、
UDP 诊断与显示。采集到算法只缓存 1 帧，算法到主线程只缓存 2 帧，队列满时背压，
不让旧帧无限堆积。算法取出排队帧后再次检查图像年龄，发控制命令前再检查一次；
相应诊断字段为 `pipeline_queue_wait_ms`、`pipeline_process_age_ms` 和
`tx_skip_reason`。启用 TX 后，结果队列阻塞超过命令 TTL 与图像时效的较小值，
或首帧之后相机断流超过
`max_image_age_ms`，都会清空目标命令并停止流水线；后台异常传回主线程。
正常达到 `--max-frames` 时排空已采集的帧；用户主动退出则取消未处理帧，先禁止
新目标入串口邮箱，再等待工作线程结束和关闭串口。以上为源码和假数据验证，
相机断线及真实云台停止时序仍需真机复验。

启用上位机无开火控制时，`standard_srm` 在 TinyMPC 收敛帧使用参考初态的
20 ms 规划角；
未收敛帧使用目标参考角，来源由 `tx_command_source` 记录。
选中的 MPC 规划角或目标参考角直接进入串口命令邮箱，不再按上一条命令做速率或单步裁剪。
命令进入邮箱后跨帧保留，避免视觉处理期间交替发送目标与零命令。Tracker 未跟踪时，正常有图像
和新鲜反馈的帧会保持最后一条已提交命令；首次跟踪前使用当前反馈角，
默认 `fire_flag` 为 0；显式启用开火后，仅符合 Planner 和实测弹速条件的跟踪命令可置为 1。
保持角需要由视觉帧持续刷新；超过 `max_image_age_ms` 未刷新时，串口线程回退零命令，
即使检测或规划线程卡住也不会无限发送旧角度。
速度估计短时不可用时仍可跟随有效参考轨迹；图像时间戳无效、串口反馈缺失或轨迹非法等硬故障
会清空命令邮箱，随后发送 yaw=0、pitch=0、fire=0。
反馈弹速在 10–25 m/s 时使用实测值；若反馈为 0，可显式设置
`--tx-use-nominal-speed=1`，让云台控制和开火判据在反馈弹速无效时使用配置中的名义弹速。
有效反馈弹速始终优先。JSONL 的 `tx_speed_source` 会标明本帧来源。
默认情况下 `temp_lost` 帧会在连续丢失
帧数不超过 `--tx-temp-lost-frames`（默认 5 帧）时允许用 Tracker 预测目标继续跟随，
帧数对应的实际时长随视觉帧率变化；可用 `--tx-follow-temp-lost=0` 关闭预测跟随，
其抖动改善需要在实车上验证。成功提交的
外推命令标记为 `tx_command_mode=extrapolate` 和 `tx_extrapolating=true`；没有实际发送
的外推尝试不会计入。`planner_control` 仅描述 Planner 求解状态，不等同于命令是否已发送；
以 `tx_command_sent` 和 `tx_command_mode` 为准。超出帧数上限、解算无效或进入 `lost`
时保持上一条已提交命令，并清除 `fire_flag`。
`tx_command_sent` 只表示命令进入邮箱；`tx_wire_target_frames` 和
`tx_wire_zero_frames` 是串口线程成功写入的累计帧数，不等于下位机执行回执。
默认参数为 20 ms 发送周期和 100 ms 命令有效期。`tracking` 表示普通
跟踪命令，具体角度来源看 `tx_command_source`；TinyMPC 是否收敛看 `planner_status`。
`hold_last_aim` 表示保持上一条命令；旧日志里的 `acquire`、`follow_state` 和
`hold_current` 是旧版模式。

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
`127.0.0.1:9871` 转发给 PlotJuggler。加载根目录的 `mpc_layout.xml`，
即可查看 MPC 参考角度、MPC 状态、实际云台 TF 姿态、速度、加速度和开火建议：

```bash
./build/sim_curve_plotter
```

其中 `mpc_target_*` 是参考轨迹，`mpc_*` 是规划器输出，
`gimbal_*` 是 TF 测得的实际云台状态，`published_fire_advice` 是最终发送给
模拟器的开火建议。

### 真机 Tracker PlotJuggler 曲线

在 NUC 的 PlotJuggler 中启动 UDP Server 并监听 `9871`，然后运行：

```bash
timeout 60s ./build/standard_srm --max-frames=300 \
  --debug-jsonl=/tmp/real_srm_tracker.jsonl --debug-udp
```

UDP 会发送与 JSONL 相同的逐帧诊断记录。`mpc_layout.xml` 中是 MPC 曲线；
Tracker 字段可在 PlotJuggler 的数据列表中选择，例如
`tracker_last_nis`、`tracker_recent_nis_failures`、
`tracker_radius_variance_m2` 和 `tracker_observation_accepted`。
该选项只发送诊断数据，不启用云台控制。

### 真机视频帧离线重跑

要在后续修改 Detector、Tracker、Aimer 或 Planner 后，基于同一批图像重新生成波形，
采集时同时保存回放输入 JSONL 和无损 PNG 帧。帧目录与 JSONL 需要一起保留：

```bash
./build/standard_srm configs/real_auto_aim.yaml \
  --port=/dev/ttyACM0 \
  --max-frames=1500 \
  --debug-jsonl=/tmp/real6_replay/input.jsonl \
  --replay-frame-dir=/tmp/real6_replay/frames
```

离线回放不会打开相机或串口；它按采集时的图像时间、匹配云台姿态和规划状态，
重新运行当前编译的视觉与瞄准代码。Planner 会使用当帧 Aimer 选中的装甲板做未来换板预览，
结果写入 `planner_preview_switch_ms` 和 `planner_yaw_20ms_deg`。
逐帧 JSONL 还会记录 `detector_roi_*` 裁剪窗口与先验状态，以及 `detector_outputs` 检测框信息；
对比采集和回放时应使用相同的模型与推理设备配置。
切换 YOLOv8 时，检测模型输出装甲框、蓝红类别和四个灯条端点；
`assets/tiny_resnet.onnx` 继续从原图裁块识别数字。`detector_model`、
`detector_score_candidates`、`detector_nms_candidates` 和
`detector_classifier_rejected` 可区分 YOLO 无候选与 Tiny 分类拒绝。
倒装相机可用 `classifier_rotate_180` 转正数字，并用 `classifier_gamma`（小于 1
时提亮）处理暗图；旧配置缺少这两个键时仍按原图分类。
YOLOv8 的动态 ROI 参数独立于 YOLOv5：`yolov8_dynamic_roi_scale` 默认 10.4，
`yolov8_dynamic_roi_min_width` 默认 208 像素；这只是按输入尺寸换算的初值。
真机配置用 `yolov8_dynamic_roi: false` 独立关闭 YOLOv8 裁剪；
当前 701 帧回放里整帧比动态 ROI 少暂失。切回 `yolo_name: yolov5` 时，
原有 `dynamic_roi: true` 和 YOLOv5 参数继续生效。

回放可加 `--endpoint-range-shadow=1` 做端点测距影子对照。它在原分辨率图像上重新拟合灯条边缘，
记录原始/拟合长度、亮度覆盖率、饱和像素比例、最大暗缺口、PnP 距离和重投影误差。
只有端点检测成功、PnP 结果有限且沿原始 YOLO 灯条跨度采样的两侧亮度覆盖率都不低于 0.90 时，
才把拟合距离用于目标初始化或
已经选中的观测更新；关联选择仍使用原始 YOLO 角点和原始距离，方位、俯仰、板面 yaw 保留原始值，
yaw 噪声也沿用原始距离对应的值。最大暗缺口和饱和比例只作诊断，重投影误差不作为正确测距的证明。
该选项默认关闭，只用于离线回放，不改变 `standard_srm` 真机路径。
先用 `--endpoint-edge-probe=1 --endpoint-range-shadow=0` 做只读对照：
逐检测记录左右灯条的灰度、蓝红通道对比、目标颜色通道截顶比例，
以及灰度和颜色梯度给出的上下端点候选。端点候选不是可靠性结论；
该模式仍记录现有轮廓拟合及其 PnP 结果，但不替换 Tracker 的距离观测。
诊断按 Tracker 处理后的检测索引写入 `association_detections`。
`--roi-depth-probe=1` 要求 `yolo_name: yolov8`，会为每个原始检测框建立局部 ROI，
再用独立 YOLOv8 实例运行完整 C++ 检测、NMS 和分类流程。结果记录 ROI 范围、各阶段候选数、
全部候选的类别/置信度/中心偏移、灯条长度和角点偏移，以及按最小重投影误差 IPPE 分支计算的
相机系距离和重投影误差；多个同标签候选会明确标记，
不强行认定为同一块物理装甲板。该探针不替换初始化或更新距离，也不传入 Tracker，
因此只用于离线测量对照。可用 `--roi-depth-probe-scale` 分别取 `8`、`12`、`16`
扫描 ROI 边长倍率（默认 `12`）；`--roi-depth-probe-min-side` 和
`--roi-depth-probe-max-side` 分别设定边长上下限（默认 `160` 和 `320` px）；
结果会逐检测记录配置和 `roi_inference_ms`。该耗时覆盖一次完整 `YOLOV8::detect()` 调用，
包含预处理、推理和后处理，不是单独的模型内核耗时，也不代表真机运行时性能。
完整角点 A/B 时，两组都启用 `--roi-depth-probe=1`，仅 ROI 组再启用
`--roi-corners-into-tracker=1`。只有唯一同标签且 PnP 距离有效的候选角点会进入 Tracker；
候选歧义或无效时回退原角点。该开关仅供 `standard_srm_replay` 离线使用。
距离条件降权实验使用 `--distance-quality-mode=record|adaptive`，默认 `off`。
`record` 在固定相机姿态、固定 IPPE 分支下对 8 个角点坐标做 ±0.25 px 中心差分，
记录 EKF 所用世界系径向距离的灵敏度 `s=||J_d||`（m/px），不改观测和更新。
分支用完整姿态的一对一连续性匹配，不能以 OpenCV 输出索引认定身份；分支不明或 PnP 失败回退倍率 1。

先从主序列记录组所有有效更新尝试的 s 取中位数并冻结，再将同一个值传给两段完整回放：
`--distance-quality-mode=adaptive --distance-quality-reference=<冻结值>`。
此时仅更新用距离方差乘以 `clamp((s/reference)^2,1,4)`；其它 R 分量、初始化、
关联规则、NIS 基准 R 和后验容限不变。P 与后续关联会随轨迹改变，因此 NIS 数值不会保证相同。
实验拒绝与 ROI 替换、其它关联策略或固定 R 扫描混用，不读取或修改生产 YAML。

每次更新的 `distance_quality` 记录有效性、失败原因、分支、8 维距离导数、倍率和计算耗时；
`distance_r_base_m2/used_m2` 与已有创新、中心增益、更新前后中心组成逐次更新核查记录。
s 只描述局部几何敏感性，没有测量实际角点误差，倍率上限 4 也未完成噪声标定；
必须同时比较两段序列的跳变尾部、收敛、旋转速度、多板投影和暂失恢复，不能仅凭更新变小认定有效。

距离观测降权敏感性可用 `--distance-update-r-scale` 做离线扫描，默认 `1`，允许范围 `1` 到 `1e6`。
倍率只用于 EKF 更新的距离方差；目标初始化、关联噪声、NIS/lost 判据和后验残差容限保持基准值。
`1e6` 可近似表示初始化后不再依赖距离更新，但不能撤销初始化时使用的 PnP 距离。
回放 JSONL 逐帧记录倍率、基准/实际距离方差，以及本次更新中距离观测对中心 x/y/z 修正的 Kalman 增益。
该参数不进入 YAML，也不改变真机 `standard_srm`。
板面 yaw 更新标准差可用 `--yaw-update-sigma-deg=25|35|45` 做离线敏感性扫描；默认 `0`
保持当前距离相关的基准方差。该参数只替换 EKF 更新 R 的 yaw 对角项；关联和 NIS 使用的基准 R 不变，
参数不进入 YAML，也不改变真机 `standard_srm`。每帧 JSONL 记录该扫描值及更新 R、基准 NIS 下的 yaw 创新诊断。
旧异步 `MultiThreadDetector` 从模型读取输入尺寸，但仍只走整帧推理；
真机 `standard_srm` 和离线回放使用检测器自己的动态 ROI 路径。

回放的 replay_tx_command_* 直接记录与真机相同来源的 Planner TX 参考角，不叠加
发送端速率或单步限幅；回放不访问串口，也不代表云台实际运动。可用
--simulate-tx=false 关闭该输出。可选的五次曲线诊断仍有独立的离线速度、加速度约束，
不参与真机发送。

需要边回放边看波形时，先在 PlotJuggler 启动 JSON UDP Server，
监听 `127.0.0.1:9871`，再给 `standard_srm_replay` 加 `--debug-udp`。
它按 `replay_time_s` 以一倍速发送新计算的逐帧结果；推理赶不上采集帧率时
会自然变慢，但不会为画图丢帧。不加该选项时仍按原来的速度完成离线计算。
UDP 使用 PlotJuggler 的接收时间作为横轴；`replay_time_s` 也作为数值字段保留。
可加 `--compare-centered-mpc` 同时输出 `centered_target_yaw_deg` 和
`centered_yaw_deg`，与现有 `planner_*` 曲线比较。该诊断会枚举过去半秒
可能的 Aimer 选板锁；若无法唯一重建，`centered_valid=false`，不伪造
居中轨迹。居中结果只写回放 JSONL/UDP，不参与 `replay_tx_command_*`，
也不改变真机发送。

可加 `--compare-quintic` 计算因果五次多项式选板过渡：连续两帧预报同一换板
时间与目标板后，尝试在新板首个参考点结束的 120–280 ms 曲线，只保留符合 yaw 速度、加速度限制的
候选。`--quintic-frame-ms`（默认 65 ms）用于将单条 TX 步长上限换算成曲线
速度上限。`quintic_reference_yaw_deg` 是曲线参考，`quintic_mode` 表示跟随、
预定、过渡或约束不可用，`quintic_transition_ms` 是选中的过渡长度；
`quintic_tx_command_yaw_deg` 用独立的限速状态模拟这条参考。原有
`replay_tx_command_*` 不受影响。两组均为离线诊断，不代表云台反馈，
也不改变真机控制或开火决策。

```bash
./build/standard_srm_replay configs/real_auto_aim.yaml \
  --input-jsonl=/tmp/real6_replay/input.jsonl \
  --frames-dir=/tmp/real6_replay/frames \
  --output-jsonl=/tmp/real6_replay/replayed.jsonl

# 对照关闭额外补偿；默认 -1 使用 YAML 中的 35 ms。
./build/standard_srm_replay configs/real_auto_aim.yaml \
  --input-jsonl=/tmp/real6_replay/input.jsonl \
  --frames-dir=/tmp/real6_replay/frames \
  --output-jsonl=/tmp/real6_replay/replayed_delay0.jsonl \
  --planner-gimbal-response-delay-ms=0

python3 tools/jsonl_to_plotjuggler_csv.py \
  --input=/tmp/real6_replay/replayed.jsonl \
  --output=/tmp/real6_replay/replayed.csv \
  --preset=planner
```

也可以用现有 `tools/plot_planner_jsonl.py` 直接从回放 JSONL 生成曲线图。
旧 JSONL 和 `--raw-frame-dir` 保存的 JPEG 没有逐帧匹配姿态，不能用于这种算法重跑；
需要按上述选项重新采集。PNG 写盘是同步的，采集期间会增加磁盘 I/O。

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

Tracker 对每个同名同类型检测框枚举当前预测距离最近的三个板号，并针对每个检测框与板号
候选，以该板预测 yaw 选择 IPPE 姿态分支。预测四边形与检测四边形允许循环移位和反向绕序
后再比较，避免角点数组起点不同造成错误的大 RMS。图像几何代价由检测对角线归一化的中心
偏差、四边方向差和四边相对长度差相加；角点 RMS 与最优排列编号只作为诊断输出。

综合分数、三维位置、距离、Mahalanobis、固定角度和旧图像中心门都记录为诊断量，不用这些
阈值提前锁死候选板号。每帧精确枚举最多三个可见模型板的组合，先最大化一对一匹配数，再
最小化图像几何代价；无可用图像投影的候选只在同等匹配数下靠后，NIS 和 Tongji 复合分数
用于打破平局。每个检测框和模型板号最多使用一次，超出可分配板数的检测框保持未匹配。
只有图像几何量全部不可用时才按 NIS 与 Tongji 复合分数回退，避免固定像素门或现有物理
阈值把旋转时的正确 ID 排除。所有候选都基于同一帧 EKF 更新前状态，分配完成后按检测顺序
更新；Target::update 原有的后验有限性、距离和角速度约束保持不变。

真机 JSONL 的 association_detections 按 Tracker 实际过滤、排序后的顺序记录框和四角点；
association_assignment_mode 记录使用全局图像几何分配还是 Tongji 分数回退，
association_physical_candidate_count 记录通过旧物理阈值的诊断候选数，
association_unmatched_detection_count 记录未匹配框数。association_selected_candidates 用同一
detection_index 记录分配结果、物理可行性、几何代价、角点排列和 EKF 接收状态。
association_model_candidates 记录
检测框与候选板号的组合，最多保留八项；超出容量只截断诊断，不影响分配。association_primary
和 association_secondary 继续保留前两个分配候选的向后兼容字段。

第一阶段排序实验可用 `--association-image-cost=legacy|corner-rms`；默认 `legacy` 保持原行为。
`corner-rms` 对循环移位及反向绕序独立寻找最小四角 RMS，再除以检测框包围盒对角线长度，
只替换分配器的图像代价。匹配数量优先级、PnP 分支选择、观测更新顺序和滤波参数保持不变。
本阶段禁止同时开启下述关联过滤实验。`association_image_cost_mode` 记录模式；
完整候选中的 `image_min_corner_rms_px/norm`、`image_min_corner_point_permutation`
记录独立最优 RMS 及其对应，`image_ranking_cost` 记录实际分配代价。
`predicted_yaw` 和 `predicted_corners_px` 保存同帧更新前候选投影，供叠图复核。
旧 `image_geometry_cost`、`image_corner_rms_px` 仍属于旧三项代价最优对应，不能混读。
各组从首帧独立回放；RMS 参与排序后，其下降不能单独作为成功证据，还需检查旋转收敛、
多板一致性、暂失恢复和初始/重建阶段的中心修正。

`standard_srm_replay` 可用 `--association-image-corner-rms-max-px=20|40|80` 进行
允许未匹配的图像几何代价消融：阈值作用于旧三项几何代价所选角点对应的 RMS（px），
不是独立最小角点 RMS，也不是综合图像代价。
所有组合都超过上限时该帧只预测，不会走旧分数兜底。单独用
`--association-physical-feasible-gate=true` 可将 `physical_feasible` 作为对照资格条件；
图像 RMS、单独物理可行性、完整旧门三种实验过滤每轮只能选一种。可用
`--association-require-gate-passed=true` 测试全部现有关联门作为候选资格条件；它包含综合评分、
位置、距离、马氏距离，以及启用阶段的图像模型门。若整帧候选都未通过，该组只预测、不使用旧分数兜底。
每帧会在 `association_experiment_model_candidates` 记录完整候选、门状态和拒绝原因，并在
`association_experiment_updates` 记录每次 EKF 更新的四维创新、中心状态各观测通道的
增益行、更新前中心、尝试中心和最终提交中心。各组必须从首帧独立完整回放。

真机 JSONL 中的 association_primary_predicted_center_error_px、
association_secondary_predicted_center_error_px 和各
association_model_candidates[].predicted_center_error_px 仍记录更新前模型投影中心与检测四角
均值的距离。image_reference_error_px、image_adjacent_error_px 和
image_observed_extent_px 仅供回看旧图像中心门诊断；它们不参与当前板号排序。
角点误差、中心误差和位置残差都同时受模型、PnP、姿态时序和标定影响，不能单凭某一项
误差直接修改外参或新增固定像素门限。

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
由 Planner 的云台反馈对准误差和近未来参考误差共同决定；`command.shoot` 仍保留
为 Shooter 基线诊断，不参与仿真开火请求。Planner 使用图像时刻查询到的 TF 姿态和
相邻帧估计角速度，并外推到规划时刻；这能验证仿真闭环中的反馈门控，但不能单独证明
模拟器复现了真机的 33–38 ms 延迟。

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

`standard_initial_velocity_variance` 控制标准四装甲目标初次捕获时的中心速度先验
方差（m²/s²）。默认值 `64` 保持既有配置行为；固定位置旋转场景的仿真
`demo.yaml` 暂设为 `0.04`（0.2 m/s 标准差），用于降低首帧位置误差被放大成虚假速度的可能。

`standard_initial_angular_velocity_variance` 控制标准四装甲捕获时的角速度先验方差，单位
`(rad/s)²`；未配置时保持原值 `0.1`。仿真 `demo.yaml` 暂设为 `64`，使高速旋转目标的
初始角速度可由后续观测快速估计。`standard_capture_bootstrap_updates` 控制捕获后成功关联
更新的预热次数；预热期间只暂缓图像选板门，综合分数、位置、距离、Mahalanobis 与角速度
物理上限仍生效。仿真当前设为 4 次，达到次数后恢复图像门；默认值 0 保持真机配置原行为。

`standard_translation_acceleration_variance` 控制标准四装甲目标预测时的平移
加速度方差（m²/s⁴）。未配置时保持原值 `100`；固定位置旋转仿真暂设为 `1.0`，
用于检查过大的过程噪声是否导致中心速度漂移。

`standard_max_angular_velocity` 是标准四装甲 EKF 更新后的角速度物理上限（rad/s）。
仿真 `demo.yaml` 按 Daedalus 底盘 `7.42 rad/s` 的设定留余量设为 `8.0`；超限时回滚
本帧完整滤波状态并拒绝观测，避免异常选板结果继续放大角速度。未配置时不启用，
因此真机配置保持原行为；该保护用于限制发散影响，不代替关联策略解决 ID 误选。

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
