# 真机自瞄：云台抖动 / 远距离漏检 / EKF 滞后 的因果链分析

分析日期：2026-10-08
分析对象：`tasks/auto_aim/`（Tracker / Target / Aimer）、`src/standard_srm.cpp`（真机链路）、
`configs/real_auto_aim.yaml`、`tools/real_tracker_analyzer.py`

> **本文只做分析，没有修改任何源码或配置。**
> 仓库当前的 `git diff` 与本轮分析开始时逐项一致（7 个文件、227 插入 / 438 删除，
> 全部是你原有的未提交改动）。文中第 5、6 节是**建议方案与标定流程**，尚未实施。

---

## 1. 结论：三个症状是一个闭环，不是三个独立问题

```
每帧一条绝对角命令（帧率上的阶梯）
        ↓  ×3 ms 曝光
云台运动模糊（最坏约 6 px，原图）
        ↓  叠加远距离本身的分辨率损失
装甲板在 640 网络输入里只剩 14.1 × 5.8 px → 置信度下降 → 漏检
        ↓
Tracker 只预测 / temp_lost 保持角
        ↓  恢复瞬间一次性补跳（历史日志：p95 1.87° vs 正常帧 0.446°）
        ↓
观测噪声 R 比真实值大 100 倍 → EKF 增益塌缩 → 估计滞后于真实目标
        ↓
创新残差变大 → 撞上固定 0.45 m 绝对门限 → 正当观测被拒 → 漏检加剧
        ↑__________________________________________________|
```

闭环里有两处是**配置缺陷**（不是物理限制），第 4 节给出量化依据。

---

## 2. 链路一：控制指令是帧率上的离散阶梯

### 2.1 事实

- `standard_srm` 每帧只调用一次限幅器：
  [standard_srm.cpp:909-931](../src/standard_srm.cpp#L909-L931) 计算 `elapsed_s` 并调用
  `limit_tx_angles`，每帧产生**一条**命令。
- 串口线程只是按 `tx_period_ms`（默认 20 ms）把**同一条**命令重复写出：
  [serial_feedback_reader.cpp:44-50](../src/real_auto_aim/serial_feedback_reader.cpp#L44-L50)。
  命令内容只在视觉帧到达时变化。
- 默认 `--tx-max-yaw-rate-deg-s=60`、`--tx-max-yaw-step-deg=2.0`，
  单条命令的步长 = `min(tx_max_yaw_step_deg, tx_max_yaw_rate_deg_s × 帧间隔)`。

**结论：命令参考是"帧率采样的阶梯"，不是连续参考。** 两个上限谁起作用取决于实际帧率：

| 视觉帧率 | 速率上限给出的步长 | 单次步长上限 | 实际约束 |
| --- | --- | --- | --- |
| 30 Hz（33 ms） | 2.00° | 2.0° | 两者相等，速率限制形同失效 |
| 50 Hz（20 ms） | 1.20° | 2.0° | 速率限制 |
| 71 Hz（14 ms，分析器兜底周期） | 0.84° | 2.0° | 速率限制 |

实际帧率应从日志的 `period_ms` / `effective_hz` 读（分析器已输出），不要按 30 Hz 假设。

### 2.2 为什么这会变成抖动和模糊

限幅器**只限制幅度，不起滤波作用**。EKF 输出里逐帧的估计噪声（PnP 角点噪声 +
姿态插值误差 + 关联分支切换）只要小于步长上限，就会被原样搬进云台参考。
把逐帧指令变化 Δθ 折算成等效角速度 ω = Δθ / 帧间隔，再用
`blur_px = fx × ω × t_exp`（fx = 1879，t_exp = 3 ms，30 Hz）换算：

| 逐帧指令变化 Δθ | 等效角速度（30 Hz） | 3 ms 曝光模糊（原图） | 换算到 640 输入（×0.444） |
| --- | --- | --- | --- |
| 0.2° | 6 °/s | 0.6 px | 0.3 px |
| 0.5° | 15 °/s | 1.5 px | 0.7 px |
| 1.0° | 30 °/s | 3.0 px | 1.3 px |
| 2.0°（步长上限） | 60 °/s | 6.0 px | 2.6 px |

目标自身旋转带来的模糊可以忽略：小陀螺 ω = 3 rad/s、r = 0.18 m 时切向速度仅 0.54 m/s，
8 m 处 3 ms 只有 0.38 px。**模糊几乎全部来自云台自身运动。**

---

## 3. 链路二：远距离检测只剩十几个像素

真机 `fx ≈ 1879 px`（`configs/real_auto_aim.yaml` 的 `camera_matrix[0]`），
小装甲板 135 mm × 56 mm（[solver.cpp:17-18](../tasks/auto_aim/solver.cpp#L17-L18)，
`pnp_lightbar_length` 默认 56 mm），
检测器把整帧按 `scale = min(640/行, 640/列)` 缩放进 640×640
（[yolov5.cpp:80-88](../tasks/auto_aim/yolos/yolov5.cpp#L80-L88)，1440×1080 → 640×480，scale = 0.444）：

| 距离 | 装甲板（原图） | 装甲板（640 输入） | 与 2.6 px 模糊相比 |
| --- | --- | --- | --- |
| 4 m | 63.4 × 26.4 px | 28.2 × 11.7 px | 短边的 22% |
| 6 m | 42.3 × 17.6 px | 18.8 × 7.8 px | 33% |
| 8 m | 31.7 × 13.2 px | 14.1 × 5.8 px | **45%** |
| 10 m | 25.4 × 10.5 px | 11.3 × 4.7 px | 55% |

8 m 处目标短边只剩 5.8 px，而最坏情况的模糊是 2.6 px。这就是"远距离容易漏检"的
物理上限，而它被第 2 节的指令抖动持续触发。

两个相关事实（都会影响后续方案选择）：

- 真机 `min_confidence: 0.8` 是**唯一**的置信度防线：`yolov5` 的
  [check_name](../tasks/auto_aim/yolos/yolov5.cpp#L209-L216) 用它过滤，
  而配置里的 `classify_model: assets/tiny_resnet.onnx` 在 yolov5 分支下**根本没有被加载**
  （分类器只有 `yolov8` 在用，[yolov8.cpp:165](../tasks/auto_aim/yolos/yolov8.cpp#L165)）。
  所以"降阈值救远距离"会直接放进数字误分类，需要先接分类器。
- 分析器已经能区分"检测为空"和"有检测但被关联拒绝"两类盲区成因
  （`continuity.frame_causes`），这两种情况的修复方向完全不同，**不要混为一谈**。

---

## 4. 链路三：EKF 的两处配置缺陷（本次分析的核心）

### 4.1 观测噪声沿用了上游仿真默认值，与已验证值差 100 倍

`configs/real_auto_aim.yaml` **完全没有覆盖** `measurement_bearing_variance`，
于是继承了 [tracker.cpp:65-68](../tasks/auto_aim/tracker.cpp#L65-L68) 的默认值
`4e-3 rad²`（标准差 ≈ 3.6°）；而 `configs/demo.yaml` 中经仿真验证的值是
`4e-5`（≈ 0.36°，注释写明"约 8px / fx=1304px"）。

更隐蔽的是**俯仰项无法配置**：[target.cpp:356-365](../tasks/auto_aim/target.cpp#L356-L365)
把 R 的对角线写成

```
R = diag( 配置值(默认4e-3)、4e-3(硬编码)、log|Δangle|+1、log(distance)/200 + 9e-2 )
```

即使改了 YAML 的 `measurement_bearing_variance`，pitch 通道仍然是 4e-3。

用一维常速卡尔曼（过程噪声 `v1 = 100` 对应 [target.cpp:130](../tasks/auto_aim/target.cpp#L130)
的加速度方差，dt = 0.033 s）算稳态增益 α 与等效时间常数 τ = -dt/ln(1-α)：

| 距离 | R = 4e-3（真机实际） | R = 4e-5（仿真已验证） |
| --- | --- | --- |
| 4 m | α = 0.254，τ ≈ 113 ms | α = 0.601，τ ≈ 36 ms |
| 6 m | α = 0.213，τ ≈ 138 ms | α = 0.529，τ ≈ 44 ms |
| 8 m | α = 0.187，τ ≈ 159 ms | α = 0.480，τ ≈ 51 ms |
| 10 m | α = 0.169，τ ≈ 178 ms | α = 0.443，τ ≈ 56 ms |

**两个关键点：**

1. **时间常数随距离增大**（R 是角度量，等效到位置噪声随 d² 增长，而过程噪声不变）。
2. Aimer 要向前预测 `fly_time`：8 m、23 m/s 时约 0.35 s
   （[aimer.cpp:89-123](../tasks/auto_aim/aimer.cpp#L89-L123)，
   并用预测时刻的 ω 外推装甲板相位）。
   160 ms 的滞后在横移 2 m/s 的目标上就是 0.32 m，
   在 8 m 处约 2.3° 的系统性偏差，且被前推环节继续放大。

这与"卡尔曼跟不上实际、算不准 aimer 位置"的现象直接对应。
这里存在**权衡**：调小 R 会同时提高带宽和噪声，第 6 节的标定流程就是为了
用实测数据而非猜测来定这个值。

### 4.2 关联门限是固定米数，等价于随距离收紧的角度门限

`association_max_position_error` / `association_max_distance_error` 同样是
**未被真机配置覆盖的默认值 0.45 m**（[tracker.cpp:55-60](../tasks/auto_aim/tracker.cpp#L55-L60)），
它在 [tracker.cpp:519-525](../tasks/auto_aim/tracker.cpp#L519-L525) 作为硬门限使用。
0.45 m 等价于视线角 `atan(0.45 / d)`：

| 距离 | 0.45 m 等价视线角 | 与 EKF 自身假设的 1σ 之比（R = 4e-3） |
| --- | --- | --- |
| 3 m | 8.5° | 1.4 σ |
| 5 m | 5.1° | 0.9 σ |
| 8 m | 3.2° | **0.55 σ** |
| 10 m | 2.6° | 0.44 σ |

而 PnP 的位置残差随距离近似**线性增长**。于是门限比滤波自己的噪声模型还紧，
**并且越远越紧**——与需求完全相反。

这解释了你此前日志里的现象：被接受帧的 `position_error` 最大值恰好卡在 0.450，
而被拒帧 mean 0.718 / max 1.895。

同一处矛盾也在距离通道：`R(2,2) = log(|Δangle|+1) + 1 ∈ [1.0, 1.8] m²`
（标准差 ≥ 1 m），却用 0.45 m 的绝对门限去拒绝——门限只有假设噪声的 0.45σ。

代码里其实已经算出了距离无关的 `position_angle_error` / `distance_angle_error`
并放进综合分数（[tracker.cpp:512-518](../tasks/auto_aim/tracker.cpp#L512-L518)），
注释也写明"才能在近距离和远距离使用同一门限"，**但硬门限仍然用的是绝对米数**。
这是本次分析发现的最明确的一处内部不一致。

---

## 5. 建议改动（尚未实施，供你决定）

按"收益 / 风险 / 能否离线验证"排序。三处都在 `tasks/auto_aim/` 内，不动控制协议。

### 5.1 关联门限距离归一化（建议优先）

把绝对门限改成"角度门限随距离折算 + 原绝对值为下限"：

```
limit(d) = max(absolute_limit, tan(angle_limit) × d)
```

- 绝对下限保证近距离行为不变，也避免 d→0 时门限退化成 0；
- 角度项保证远距离按视线角而不是米来收紧；
- `angle_limit = 0` 表示关闭，**默认关闭可保证其他配置（demo / sentry 等）行为不变**。

涉及位置：`Tracker::update_target` 里 `position_gate_passed` / `distance_gate_passed`
两处（[tracker.cpp:519-525](../tasks/auto_aim/tracker.cpp#L519-L525)），以及传给
`Target::update` 的后验距离门限（[tracker.cpp:590-591](../tasks/auto_aim/tracker.cpp#L590-L591)）
——两者必须用**同一个**本帧限值，否则会出现"门控通过却被 EKF 后验拉出门限"。

建议新增键：`association_max_position_angle_error`、`association_max_distance_angle_error`（单位 rad）。

### 5.2 观测噪声模型参数化

把 [target.cpp:356-365](../tasks/auto_aim/target.cpp#L356-L365) 里硬编码的三项提成配置，
默认值**逐项复现旧行为**（可用旧公式做逐元素单测比对）：

| 建议键 | 当前值 | 说明 |
| --- | --- | --- |
| `measurement_bearing_variance` | 4e-3（默认） | 已可配置，只是真机没写 |
| `measurement_elevation_variance` | 4e-3（硬编码） | 新增，用于单独标定 pitch |
| `measurement_distance_variance_{slope,offset}` | 1.0 / 1.0 | 新增，`slope = 0` 即常数距离方差 |
| `measurement_armor_yaw_variance_{slope,offset}` | 0.005 / 9e-2 | 新增 |

真机取值建议：先把 `measurement_bearing_variance` 与 `measurement_elevation_variance`
都设为 `4e-5`，即**复用仿真已验证的角噪声带宽**（它本身就是角度量，与 fx 无关，
"8 px @ fx=1304"对应 6.1e-3 rad）。
距离项与朝向项**先不动**：它们的设计（斜视 / 远距离放大不确定度）本身合理，
但 `R(2,2) ≈ 1 m²` 明显偏大，应由第 6 节的深度创新统计决定新值，不靠猜。

### 5.3 防止"改了配置但没生效"

yaml-cpp 对拼错的键名只返回 `IsDefined() == false`，会**静默退回默认值**。
建议两处防护：

- Tracker 暴露一个只读的"构造后实际生效配置"（门限 + 噪声），单测断言
  "配置里写了什么就必须生效什么，没写就必须等于文档默认值"；
- 把本帧实际生效的门限写进 JSONL（如 `association_primary_position_gate_limit`），
  这样离线分析能直接确认门限形状，而不必反推配置。

---

## 6. 标定流程：用现在的日志字段就能做，不需要改代码

上面两处数值的正确性应该由实车数据决定。**当前版本的 JSONL 已经记录了所需的全部字段**，
不需要先改代码：

| 需要的量 | 现有字段 |
| --- | --- |
| 目标距离 | `association_primary_predicted_distance` / `_observed_distance` |
| 位置残差 | `association_primary_position_error`（m） |
| 距离无关残差 | `association_primary_position_angle_error`、`_distance_angle_error`（rad） |
| 方位创新 | `association_primary_bearing_error`（rad） |
| 门控结论 | `association_primary_{gate,position_gate,distance_gate,mahalanobis_gate,score_gate}_passed` |
| 是否被接受 | `association_accepted_count` |
| 帧率 | `device_ticks` / `tick_hz`（分析器已换算 `period_ms`） |

采一份日志：

```bash
./build/standard_srm configs/real_auto_aim.yaml --port=/dev/ttyACM0 \
  --debug-jsonl=/tmp/real_srm_debug.jsonl --quality-metrics=1
python3 tools/real_tracker_analyzer.py \
  --input /tmp/real_srm_debug.jsonl --output /tmp/real_srm_report.json
```

然后用下面这段**临时脚本**（用 `python3 - <<'PY'` 直接跑，不写入仓库）做距离分桶标定。
注意 `position_angle_error` 是 Tracker 已经算好的 `atan2(position_error, 参考距离)`：

```python
import json, math, statistics, collections

EDGES = [0.0, 3.0, 5.0, 8.0, 12.0, float("inf")]

def load(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]

def finite(records, key):
    return [float(r[key]) for r in records
            if isinstance(r.get(key), (int, float)) and math.isfinite(r[key])]

def pct(values, q):
    s = sorted(values)
    return None if not s else s[min(len(s) - 1, max(0, math.ceil(q * len(s)) - 1))]

def bucket_label(d):
    for lo, hi in zip(EDGES, EDGES[1:]):
        if d <= hi:
            return f">{lo:g}m" if hi == float("inf") else f"{lo:g}~{hi:g}m"

records = [r for r in load("/tmp/real_srm_debug.jsonl")
           if r.get("event") == "frame" and (r.get("association_candidate_count") or 0) > 0]

by_bucket = collections.defaultdict(list)
for r in records:
    d = r.get("association_primary_predicted_distance") or r.get("association_primary_observed_distance")
    if isinstance(d, (int, float)) and d > 0:
        by_bucket[bucket_label(d)].append(r)

accepted = [r for r in records if (r.get("association_accepted_count") or 0) > 0]
print(f"候选帧 {len(records)}，已接受 {len(accepted)}")

for name in sorted(by_bucket, key=lambda s: (s.startswith(">"), s)):
    frames = by_bucket[name]
    acc = [r for r in frames if (r.get("association_accepted_count") or 0) > 0]
    err, ang = finite(acc, "association_primary_position_error"), \
               finite(acc, "association_primary_position_angle_error")
    if not err:
        print(f"  {name}: 候选 {len(frames)}，无已接受帧（注意区分漏检与拒绝）")
        continue
    print(f"  {name}: 候选 {len(frames)} 接受 {len(acc)} | "
          f"位置残差 mean={statistics.fmean(err):.3f}m p95={pct(err, 0.95):.3f}m | "
          f"视线角 p95={math.degrees(pct(ang, 0.95)):.2f}°")

# 建议角度门限 = 已接受视线角残差 P95 × 1.5
ang_all = finite(accepted, "association_primary_position_angle_error")
if len(ang_all) >= 20:
    gate = pct(ang_all, 0.95) * 1.5
    sigma = statistics.median(abs(v) for v in ang_all) / 0.6744897501960817  # |X| 中位数估 σ
    print(f"建议 association_max_position_angle_error ≈ {gate:.4f} rad ({math.degrees(gate):.2f}°)")
    print(f"已接受帧视线角残差 σ≈{math.degrees(sigma):.2f}°")
else:
    print(f"已接受帧仅 {len(ang_all)}（<20）：样本不足以估计 σ 与门限")

# 方位创新 σ 上界：创新方差 = HPHᵀ + R ≥ R，实测只能作上界
sb = [abs(v) for v in finite(accepted, "association_primary_bearing_error")]
if len(sb) >= 20:
    sigma_b = statistics.median(sb) / 0.6744897501960817
    print(f"方位创新 σ≈{math.degrees(sigma_b):.3f}° → "
          f"measurement_bearing_variance 上界≈{sigma_b ** 2:.2e} rad²")

# 旧 0.45m 固定门限本会拒、现在已接受的帧数（量化收益）
print("位置残差 >0.45m 但仍被接受:",
      sum(1 for r in accepted if (r.get("association_primary_position_error") or 0) > 0.45), "帧")
```

判读规则：

1. **某个距离桶的"接受 / 候选"比例明显低于近距离桶** → 该距离的关联门限仍在削顶，
   按 `建议 association_max_position_angle_error` 放宽（它是已接受 P95 的 1.5 倍）。
2. **`位置残差 >0.45m 但仍被接受` 占比高** → 距离归一化确实救回了帧；
   同时要复核装甲板 ID 切换次数（分析器 `jitter.armor_id_switches`）是否上升，
   确认没有引入误关联。
3. **`方位创新 σ` 明显大于配置值**（例如实测 0.7° vs 配置 0.36°）→ 配置的 R 偏小；
   若实测稳定在 0.36° 附近，说明 4e-5 合适。注意实测创新方差 = HPHᵀ + R ≥ R，
   **只能当上界**，不能直接填进配置。
4. **样本不足 20 帧已接受观测时不要下结论**：半正态中位数估计在小样本下会出现
   σ > P95 这类自相矛盾的值（我验证脚本时实际遇到过：5 帧样本给出 σ = 5.18°
   而 P95 = 4.96°）。所以上面判断都加了 `>= 20` 门槛。

---

## 7. 尚未覆盖的两条链路（本次没有分析到可执行结论）

按预期收益排序：

1. **动态 ROI**（收益最大，工作量也最大）
   以 EKF 预测位置为中心裁一块（例如 720×540）再送网络，等效线性分辨率翻倍，
   直接突破第 3 节的上限。当前 `use_roi` 只支持配置里的**静态**矩形
   （[yolov5.cpp:67-74](../tasks/auto_aim/yolos/yolov5.cpp#L67-L74)），
   需要把 ROI 变成跟踪量，并同步修正 `offset_` 回填与 `center_norm` 的归一化分母
   （现在用 `bgr_img.rows/cols`，改成 ROI 后分母会变，会影响依赖 `center_norm` 的逻辑）。
2. **把 `tiny_resnet` 分类器接进 yolov5**
   真机配置写了 `classify_model` 却没被加载（见第 3 节）。接上之后才能安全降低
   `min_confidence`，用分类器而不是阈值来挡数字误分类。
3. **指令平滑**
   把第 2 节的帧率阶梯改成按 20 ms 发送节拍连续逼近 + 一阶低通（时间常数 40~80 ms）。
   直接削掉高频激励，代价是增加 20~30 ms 相位滞后——**小于**当前链路延迟
   （日志里的 `capture_to_aimer_ms`），因此净收益大概率是正的，但必须实车 A/B。
4. **`standard_geometry_constraint`**
   真机当前关闭。四装甲车的 r 是已知机械尺寸，单块装甲观测下 r 不可观测；
   打开约束能稳定中心与瞄点，但需先确认实车半径确实是 0.18 m
   （分析器已有 `jitter.radius_stdev_within_generation` 可以看当前 r 的漂移幅度）。
5. **掉落帧期间的 ω 质量**
   `fly_time` 前推把 ω 误差近似线性放大（8 m、0.35 s）。可考虑对 ω 单独低通，
   或让过程噪声随丢失帧数自适应。这一条我还没量化，需要先看 `angular_velocity`
   与 `center_speed` 在 temp_lost 前后的分布。

---

## 8. 风险与注意事项

- **调小 R 会让马氏门限同步变紧**。这在统计上是正确行为（门限本就该反映噪声模型），
  而且预测期间的 P 增大会自动放宽门限。但如果实车出现"门控通过但被后验拒绝"的帧数
  上升，说明真实姿态 / 手眼误差大于假设，**应该上调 R，而不是放宽门限**。
- **不要同时改 R 和门限并只看总检测率**。两者对"漏检"的作用相反（R 调小会收紧马氏门控），
  必须分开看：`association_*_gate_passed` 的分布（门限影响）与
  `accepted_*_bearing_error` 的分布（R 影响）。
- 第 2 节的模糊结论依赖 `exposure_ms = 3`。若实车曝光被改成别的值，
  表中数字按线性比例换算。
- 第 4.1 节的增益表用的是一维常速模型与 `v1 = 100`，用来**比较两种 R 的相对影响**，
  不是对真机绝对滞后量的精确预测（真实系统是 11 维状态、4 维观测、含几何约束）。
