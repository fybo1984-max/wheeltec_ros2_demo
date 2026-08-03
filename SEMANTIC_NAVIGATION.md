# 工业 AGV 语义感知导航

## 目标与边界

本工作在现有 WHEELTEC ROS 2 Humble + Nav2 系统上增加可解释的语义软成本，
并保留语音导航、点位记录、复合路线和稳定演示行为。

当前已完成静态 mask 基础和动态 RGB-D 风险 mask 的离线可测试数据管线。
语义层和动态生成节点均默认关闭，因此原有启动命令的导航行为不变。启用相机、
导航或执行真实路径实验前必须确认现场安全。

## 对论文与两个上游仓库的判断

### 论文

`Semantic Aware Costmap Guidance for Industrial AGV and Forklift Navigation
in Cluttered Warehouse Spaces` 给出的总体闭环合理：

1. YOLO 检测；
2. RGB-D 深度与相机内参投影；
3. TF 转换到 `map`；
4. 发布地图对齐的语义 mask；
5. Nav2 全局 costmap 注入可通行软成本；
6. 用 crossing ratio、路径长度、语义边界距离、重规划次数和时延做 A/B。

但当前 PDF 标注为 draft manuscript，仍含作者信息 TODO，并引用了 2026 年条目。
论文中的结果不能替代本工作空间里的原始日志、配置哈希和可重复脚本。后续实验
必须重新采集，不能把文稿数字直接当作本平台已验证结果。

### `concres/semantic_costmap_plugin`

- 上游提交：`1a9c13b76df244555ad967f92345ca96700ba8e2`
- 上游目标：ROS 2 Jazzy/Nav2 Jazzy
- 许可：Apache-2.0
- 采用内容：静态 mask layer、Sugeno 模糊成本、语义区域膨胀、动态参数
- 本地适配：Humble API、严格加载失败检查、参数范围检查、WHEELTEC 配置与测试

这是第一阶段的实现基础，源码位于
`src/semantic_costmap_plugin`。

### `e-cagan/semantic-slam-workspace`

- 上游提交：`b0154026d371f6a4e67272933e1eac1b87585b2a`
- 目标：ROS 2 Humble TurtleBot3/Gazebo 示例
- 可复用概念：`SemanticObservation`、语义对象去重、查询服务、RViz 标记
- 不直接移植：
  - `sim_bringup` 会引入第二套 TurtleBot3/Gazebo 导航栈；
  - `llm_navigator` 会直接发送 `NavigateToPose`，与现有语音/大模型导航重叠；
  - `perception` 用二维 LiDAR 方位近似目标深度，不符合论文的 RGB-D 投影方法；
  - 语义地图重启即丢失，不满足可重复实验要求。

因此只采用其接口和集成思路。动态阶段复用现有 `ultralytics_ros2` 检测消息，
并实现 WHEELTEC 自有的“Detection2D + Depth + CameraInfo + TF →
OccupancyGrid”管线，而不是复制整仓。

## 第一阶段：静态 mask PoC

当前车型 `senior_akm` 的全局 costmap 已增加：

```yaml
plugins: ["static_layer", "obstacle_layer", "mask_layer", "inflation_layer"]
```

`mask_layer.enabled` 默认为 `False`。示例 mask 与
`large_loop_final` 使用完全相同的尺寸、分辨率和原点：

- 尺寸：865 × 851
- 分辨率：0.05 m/cell
- 原点：(-10.2, -11.1)
- 示例黑色区域：图像像素 `x=[390, 539]`、`y=[600, 649]`
- 对应近似地图区域：`x=[9.3, 16.8] m`、`y=[-1.05, 1.45] m`

此区域只是复现实验用模板，尚未被现场确认成人行区、装卸区或禁行区。

## 不启动硬件的验证

在干净终端中只 source ROS 2 Humble 和创新工作空间：

```bash
cd /home/wheeltec/wheeltec_ros2_innovation
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install --packages-up-to \
  semantic_costmap_plugin semantic_mask_generator \
  semantic_planning_experiments wheeltec_nav2 largemodel
colcon test --packages-select \
  semantic_costmap_plugin semantic_mask_generator \
  semantic_planning_experiments
colcon test-result \
  --test-result-base build/semantic_costmap_plugin --all --verbose
colcon test-result \
  --test-result-base build/semantic_mask_generator --all --verbose
colcon test-result \
  --test-result-base build/semantic_planning_experiments --all --verbose
```

这些命令只编译和运行单元测试，不启动底盘、导航、雷达、相机或麦克风。
当前插件 10 个 CTest 全部通过；其中 14 个行为测试覆盖模糊成本、固定软代价、
硬障碍对照、风险强度、语义膨胀、静态/话题输入、插件加载，以及“不清除
未知区、不降低既有致命障碍”的合并边界。动态生成包 20 项测试通过，其中
17 项行为测试覆盖深度解码、像素投影、TF 数学、观测合并、时间衰减、风险
半径缩放、标记 ID 档案映射和地图栅格化。
规划器实验包 105 项测试通过，其中 102 项行为测试覆盖 Nav2 PGM 坐标转换、
OccupancyGrid 往返转换、动态 mask 空间摘要、穿越距离/比例、语义边界间距、
实际风险值摘要、指标差值、输入哈希、可比较性校验、统计汇总、路径重复性检测
和论文 manifest 安全展开及逐 trial 结果验收。

## 第二阶段：动态 RGB-D 风险 mask

`semantic_mask_generator` 的输入和输出为：

- 输入：`/detections`（`vision_msgs/Detection2DArray`）；
- 可选输入：`/aruco_marker_publisher/markers`（易碎品等标识位姿）；
- 可选输入：`/semantic/loading_zone_active`（装卸区作业状态真值）；
- 输入：配准到彩色图的深度图和 `/camera/color/camera_info`；
- 输入：`/map` 和检测相机到 `map` 的 TF；
- 输出：`/semantic_mask`（`nav_msgs/OccupancyGrid`，风险值 `0..100`）。

默认仓储风险档案接受 `person`、`other_vehicle`、`pallet_load`、
`temporary_cargo` 和 `fragile_goods`，初始风险值/半径分别为 `100/1.2 m`、
`95/1.8 m`、`65/0.8 m`、`60/0.8 m`、`85/1.0 m`。标准 COCO 模型通常只有
`person`；其他对象需要仓储检测模型或标签适配器。标记 `101` 默认映射为
`fragile_goods`，启用后直接使用 ArUco 位姿，不重复执行深度投影。

观测会按空间距离合并，保持 0.8 s 后在 1.2 s 内线性衰减。易碎品货区、固定
装卸区等持久化区域仍由静态 mask 表达。

生成节点默认 `enabled:=false`，并且默认拒绝未经明确确认的深度输入。
只有确认深度已经配准到 YOLO 使用的彩色图后，才允许：

```bash
ros2 launch semantic_mask_generator dynamic_semantic_mask.launch.py \
  enabled:=true \
  depth_is_registered:=true
```

仅验证标记输入时可使用 `marker_enabled:=true`；默认标记 `101` 映射到
`fragile_goods`。合成 `MarkerArray` 端到端验证在地图坐标 `(5.0, 5.0) m`
生成了 309 个风险值为 85 的栅格，质心约为 `(5.042, 5.042) m`。

装卸区使用地图坐标多边形和 `active/idle` 状态分离表示：Pilot 阶段先由人工发布
作业状态真值，验证 costmap 因果效果；人员和托盘联合识别留到感知阶段接入。
合成 `2 m × 2 m` 装卸区在 active 时生成 400 个风险值为 85 的栅格，切换为
idle 后下一发布周期清为 0。

这条命令只启动 mask 生成节点，不会自行启动相机、YOLO、Nav2 或底盘。
当前 Astra 相机启动参数 `depth_registration` 默认仍为 `false`；开启相机属于
硬件实验步骤，执行前必须通知现场人员并检查图像、深度尺寸和 `frame_id`。

真实数据录制前使用输入预检器检查这些条件：

```bash
ros2 run semantic_planning_experiments semantic_live_input_readiness \
  --ros-args \
  -p output_path:=/tmp/semantic_live_input_readiness.json
```

它只订阅已经运行的话题，不会启动相机或导航。报告必须满足
`experiment_input_ready=true`，其中包括人员检测、对齐尺寸与 frame、逐检测帧
最坏时间差不超过 0.15 s，以及相机检测 frame 到 `map` 的 TF 可用。

## 启用方式（会启动导航，可能使小车运动）

确认现场安全后：

```bash
ros2 launch largemodel largemodel_control.launch.py \
  use_nav:=true \
  fixed_command_mode:=true \
  semantic_costmap_enabled:=true \
  semantic_mask_source:=file \
  semantic_task_urgency:=0 \
  semantic_avoidance_level:=70.0
```

运行时可调整：

```bash
ros2 param set /global_costmap/global_costmap \
  mask_layer.task_urgency 5
ros2 param set /global_costmap/global_costmap \
  mask_layer.avoidance_level 80.0
```

有效模糊成本发布在
`/global_costmap/semantic_zone/fuzzy_cost_value`（`std_msgs/UInt8`）。

动态 mask 接入导航时使用 `semantic_mask_source:=topic`，该操作会启动导航并
可能使小车运动，必须先完成静止数据检查。

## 第三阶段：规划器级 A/B（已完成，不驱动底盘）

`semantic_planning_experiments` 提供独立 launch，仅启动 `map_server`、
`planner_server`、生命周期管理器和静态实验 TF。它不包含 controller、BT、
AMCL、传感器或底盘节点，不发布速度命令。实验默认在仅本机可见的独立
ROS domain 73 中运行。

```bash
ros2 launch semantic_planning_experiments planner_ab.launch.py \
  output_path:=/tmp/semantic_planning_ab.json
```

同一规划进程先关闭、再开启语义层，并在两次请求间清空和刷新全局 costmap。
实验专用配置关闭 `cache_obstacle_heuristic`，避免第二次规划复用基线启发式。
JSON 保存完整路径、规划耗时、路径长度、mask crossing length/ratio、语义
边界最小距离，以及代码版本和地图、mask、规划参数文件的 SHA-256。

默认模板路线 `(6.0, 0.0) → (20.0, 0.0)` 已连续运行两次，路径几何完全一致：

- 基线：路径约 14.01 m，语义区穿越约 7.50 m，穿越比例约 53.52%；
- 语义：路径约 17.27 m，语义区穿越为 0，最小语义边界间距约 0.64 m；
- 差值：路径增加约 3.26 m。

这些数字验证了“软成本在存在替代路径时改变规划”的工具链目标。示例 mask
尚未经过现场区域标定，结果不能直接作为论文正式对比数据。正式采集应将输出
写到仓库外部的实验数据目录，并在代码提交后重新运行，以获得干净提交哈希。

### 动态 topic 输入等价性

同一实验也可把 mask 作为在线 `OccupancyGrid` 发布，而不是由插件直接读取
YAML：

```bash
ros2 launch semantic_planning_experiments planner_ab.launch.py \
  mask_source:=topic \
  domain_id:=74 \
  output_path:=/tmp/semantic_planning_ab_topic.json
```

实验客户端等待 `/semantic_mask` 订阅建立后，以 reliable、
transient-local、depth 1 QoS 发布 0/100 风险栅格。当前模板的 file 与 topic
两种模式均已完成无控制器规划验证：基线路径逐点一致、语义路径逐点一致，除
规划耗时外的全部指标完全一致。这证明插件的动态话题输入与静态输入在相同 mask
下语义等价，但尚未证明真实 RGB-D 检测链路的精度或时延。

### 合成 RGB-D 到规划器的端到端验证

以下 launch 使用确定性的合成 CameraInfo、16UC1 配准深度图、`person`
Detection2D 和静态 TF，经过真实 `semantic_mask_generator` 生成
`/semantic_mask`，再送入真实 Nav2 planner：

```bash
ros2 launch semantic_planning_experiments generator_planner_ab.launch.py \
  output_path:=/tmp/semantic_generator_planning_ab.json
```

它使用独立 ROS domain 75，只启动合成输入、mask 生成器、map server、
planner server 和生命周期管理器；不启动 controller、BT navigator、AMCL、
相机、YOLO 或底盘，也不发布 `cmd_vel`。

当前确定性场景把 2 m 深的居中 `person` 检测投影到地图约 `(13.0, 0.0)`，
生成 1806 个风险栅格。验证结果：

- 基线：路径约 14.01 m，穿越动态风险区约 2.35 m；
- 语义：路径约 17.46 m，风险区穿越为 0，最小语义边界间距约 1.11 m；
- 差值：路径增加约 3.45 m，基线与语义路径几何不同。

JSON 同时记录动态 mask 的尺寸、占用数量、质心、边界、生成器参数文件哈希及
完整路径。这证明了“检测消息 → RGB-D 投影 → 地图风险栅格 → Nav2 软成本 →
路径改变”的软件链路。合成数据不证明真实相机标定、YOLO 检出率或现场时延，
不能直接作为论文正式实验数据。

### 软成本参数敏感性

同一合成输入可显式设置任务紧急度和避让等级，参数会通过 ROS 参数服务写入
真实 costmap layer，并记录在 JSON 中：

```bash
# 安全优先
ros2 launch semantic_planning_experiments generator_planner_ab.launch.py \
  domain_id:=76 task_urgency:=0 avoidance_level:=100.0 \
  output_path:=/tmp/semantic_sweep_safety.json

# 均衡
ros2 launch semantic_planning_experiments generator_planner_ab.launch.py \
  domain_id:=77 task_urgency:=5 avoidance_level:=70.0 \
  output_path:=/tmp/semantic_sweep_balanced.json

# 紧急通行
ros2 launch semantic_planning_experiments generator_planner_ab.launch.py \
  domain_id:=78 task_urgency:=10 avoidance_level:=0.0 \
  output_path:=/tmp/semantic_sweep_urgent.json
```

当前单次确定性对照结果：

| 模式 | `task_urgency` | `avoidance_level` | 路径长度 | 风险区穿越 | 最小间距 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 安全优先 | 0 | 100 | 17.97 m | 0 m | 1.11 m |
| 均衡 | 5 | 70 | 17.92 m | 0 m | 1.11 m |
| 紧急通行 | 10 | 0 | 14.01 m | 2.35 m | 0 m |

结果证明同一 `person` 风险 mask 可随任务策略从绕行切换为穿越，语义层仍是
软成本而非硬障碍。当前表格是软件链路单次验证；论文参数结论必须在固定提交上
重复多次并报告分布，不能只使用该表。

### 动态风险清除与路径恢复

合成输入可在指定时间后停止发布检测，但继续发布 CameraInfo 和深度图。以下
实验等待生成器完成保持与衰减，在语义层始终开启的情况下执行第三次规划：

```bash
ros2 launch semantic_planning_experiments generator_planner_ab.launch.py \
  domain_id:=82 \
  detection_active_duration_sec:=30.0 \
  observation_hold_sec:=0.5 \
  observation_decay_sec:=1.0 \
  recovery_timeout_seconds:=15.0 \
  output_path:=/tmp/semantic_decay_ab.json
```

当前确定性结果：

1. 无语义层基线约 14.01 m，穿越原风险区约 2.35 m；
2. 人员风险存在且语义层开启时约 17.46 m，风险区穿越为 0；
3. 停止检测并等到 mask 清空后，语义层保持开启，路径恢复为约 14.01 m；
4. 恢复路径与基线 120 个位姿逐点一致，路径长度、穿越长度、穿越比例和间距
   的差值均为 0。

报告中的 `recovered_after_mask_clear` 保存第三条路径，
`delta_recovered_minus_baseline` 保存恢复路径与基线的指标差异。这验证了动态
风险不会永久残留在 costmap。该实验验证软件时序，不替代真实检测丢失、遮挡和
跟踪抖动测试。

### 重复试验统计

`semantic_planning_summary` 读取两个或更多 A/B JSON，默认拒绝带 `-dirty`
提交或关键输入不一致的报告。关键输入包括代码提交、地图与规划器配置哈希、
生成器配置及运行参数、起终点、mask 输入模式和语义策略参数。

```bash
ros2 run semantic_planning_experiments semantic_planning_summary \
  --output /tmp/semantic_repeatability_summary.json \
  /tmp/semantic_repeatability_01.json \
  /tmp/semantic_repeatability_02.json \
  /tmp/semantic_repeatability_03.json
```

汇总报告为每个条件记录：

- 路径 SHA-256、唯一哈希数量和逐点重复性；
- 指标样本数、均值、总体标准差、最小值和最大值；
- 输入报告路径、报告文件 SHA-256 和统一比较指纹。

当前在干净提交 `da778c1` 上连续完成三次合成 RGB-D 端到端试验：

| 条件 | 路径长度 | 风险区穿越 | 最小间距 | 规划耗时均值 ± 总体标准差 |
| --- | ---: | ---: | ---: | ---: |
| 基线 | 14.0053 m | 2.3474 m | 0 m | 14.8 ± 1.4 ms |
| 语义 | 17.4593 m | 0 m | 1.1088 m | 339.0 ± 9.6 ms |

两种条件分别只有一个唯一路径哈希，所有非耗时指标总体标准差均为 0。该结果
验证确定性合成链路和统计工具；样本量仅为 3，不能替代论文中的多场景、多随机
种子和真实传感器重复试验。

### 论文 Pilot manifest

论文实验协议见 `PAPER_EXPERIMENT_PROTOCOL.md`。Pilot v1 manifest 将研究问题、
场景因素、重复数、launch 参数和数值验收阈值放在同一份 YAML 中：

```bash
ros2 run semantic_planning_experiments semantic_experiment_plan \
  src/semantic_planning_experiments/config/paper_pilot_manifest.yaml \
  --output /tmp/semantic_paper_pilot_plan.json
```

当前计划包含 4 个场景、11 个 trial，使用独立 ROS domain 90–100。输出保存
manifest 哈希、计划指纹、每个 trial 的 argv、报告路径、研究问题、因素和验收
条件。该命令只生成计划，不执行 launch，并明确记录 controller、硬件和速度
命令均不允许。

计划报告生成后，使用 `semantic_experiment_evaluate` 逐项检查阈值。只有全部
trial 通过才返回成功，缺失、失败、dirty revision 和不安全报告不会被静默
忽略。

论文方法消融另使用 `paper_ablation_manifest.yaml`，生成 `fixed`、`lethal`
和 `fuzzy` 各 3 次、共 9 个计划 trial（ROS domain 110–118）。每次报告同时
包含关闭语义层的 baseline，统计汇总会拒绝混合不同 `cost_mode`。

干净提交 `abe064d` 的 9 次合成方法消融全部通过验收，三种模式各自路径均
完全重复：`fixed`、`lethal`、`fuzzy` 平均路径长度分别约为 17.92 m、
18.26 m 和 17.46 m，风险区穿越均为 0。该单场景、小样本结果只证明实验与
统计链路可用，不能替代论文的多场景、异常注入和真实数据试验。

多场景 `paper_scenario_matrix_manifest.yaml` 和感知扰动
`paper_robustness_manifest.yaml` 各生成 6 个场景、18 个计划 trial。前者覆盖
人员距离、数量和风险半径；后者覆盖固定种子深度噪声、无效深度、检测丢帧、
同步偏移和置信度边界。

干净提交 `ce72d97` 的多场景矩阵 18/18 已通过验收，六组内部路径完全重复且
风险区穿越均为 0。人员深度 1.2、2.0、3.0 m 对应 mask 质心 x 约为
12.225、13.025、14.025 m，语义路径约为 16.2784、17.4593、17.6533 m；
小/大半径产生 810/3194 个风险格；三人聚集产生 2523 个风险格和 17.8690 m
绕行路径。

干净提交 `fc45dac` 的鲁棒性矩阵 18/18 已通过验收，六组内部路径完全重复且
风险区穿越均为 0。50% 无效深度、50% 检测丢帧、0.10 s 时间偏移和 0.55
置信度与无扰动路径逐点一致；0.10 m 固定种子深度噪声使 mask 质心约偏移
1.1 cm，并触发 16.2976 m 的不同路径。该结果支持“安全指标保持、路径拓扑
对深度噪声敏感”，不支持笼统的完全路径稳定结论。

当前仓储对象类别 `paper_object_class_manifest.yaml` 生成 `person`、
`other_vehicle`、`pallet_load`、`temporary_cargo`、`fragile_goods` 各 3 次，
共 15 个计划 trial（ROS domain 160–174）。它验证检测或标记适配后的风险档案
和路径响应，不测量真实识别精度：

```bash
ros2 run semantic_planning_experiments semantic_experiment_plan \
  src/semantic_planning_experiments/config/paper_object_class_manifest.yaml \
  --output /tmp/semantic_paper_object_class_plan.json
```

旧版干净提交 `b9e1611` 的 12 次对象类别 Pilot 已全部通过验收，四类各自的路径
几何完全重复。`person`、`forklift`、`fragile_box` 的语义路径均不穿越风险
区；低风险 `pallet` 保持软代价，语义路径约 14.0169 m，仍穿越约 1.3469 m。
该结果验证不同风险档案的下游响应，不代表真实 YOLO 已能识别这些仓储对象。

风险因素分离计划 `paper_risk_factor_ablation_manifest.yaml` 使用一个共享
人员基准，分别改变风险值和半径，共生成 5 个场景、15 个计划 trial（ROS
domain 180–194）。报告额外保存实际 OccupancyGrid 正值的数量、最小值、
最大值和均值，用于证明风险强度确实到达规划链路。

干净提交 `21f02ef` 的 15 次试验已全部通过验收，五组路径各自完全重复。
风险值 65 保持 1806 个风险格不变但仍穿越约 2.2536 m；风险值 85 和 100
均完全绕行。固定风险值 100 时，风险半径 0.804、1.2、1.8 m 对应语义路径
约 16.2867、17.4593、17.9692 m。风险值组存在非单调路径响应，不能把 Pilot
结果简化成“风险值越大路径必然越长”。

## 后续阶段与验收

动态输入后续验收顺序：

- 用录制的真实 RGB-D、检测和 TF 数据验证 `/semantic_mask`；
- 再对同一起终点分别使用 `file` 和 `topic` 输入；
- 模型权重继续由 `.gitignore` 排除。

### 阶段 4：受控实车实验

- 先做静止规划验证，再做低速闭环；
- 设置急停人员和物理安全边界；
- 记录成功率、接触/急停、路径长度、穿越率、重规划次数和端到端时延；
- 对漏检和误检做受控注入，不把普通在线结果当作因果验证。
