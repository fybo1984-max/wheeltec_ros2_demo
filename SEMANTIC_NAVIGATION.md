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
当前插件 10 个 CTest 全部通过；其中 11 个行为测试覆盖模糊成本、风险强度、
语义膨胀、静态/话题输入、插件加载，以及“只增加软成本、不清除未知区、
不降低致命障碍”的合并边界。动态生成包 12 项测试通过，其中 9 项行为测试
覆盖深度解码、像素投影、TF 数学、观测合并、时间衰减和地图栅格化。
规划器实验包 8 项测试通过，其中 5 项行为测试覆盖 Nav2 PGM 坐标转换、
穿越距离/比例、语义边界间距、指标差值和输入哈希。

## 第二阶段：动态 RGB-D 风险 mask

`semantic_mask_generator` 的输入和输出为：

- 输入：`/detections`（`vision_msgs/Detection2DArray`）；
- 输入：配准到彩色图的深度图和 `/camera/color/camera_info`；
- 输入：`/map` 和检测相机到 `map` 的 TF；
- 输出：`/semantic_mask`（`nav_msgs/OccupancyGrid`，风险值 `0..100`）。

默认仅处理 `person`，风险值为 100，半径为 1.2 m。观测会按空间距离合并，
保持 0.8 s 后在 1.2 s 内线性衰减。易碎品货区、固定装卸区等持久化区域仍由
静态 mask 表达。

生成节点默认 `enabled:=false`，并且默认拒绝未经明确确认的深度输入。
只有确认深度已经配准到 YOLO 使用的彩色图后，才允许：

```bash
ros2 launch semantic_mask_generator dynamic_semantic_mask.launch.py \
  enabled:=true \
  depth_is_registered:=true
```

这条命令只启动 mask 生成节点，不会自行启动相机、YOLO、Nav2 或底盘。
当前 Astra 相机启动参数 `depth_registration` 默认仍为 `false`；开启相机属于
硬件实验步骤，执行前必须通知现场人员并检查图像、深度尺寸和 `frame_id`。

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

## 后续阶段与验收

动态输入的验收顺序：

- 先用录制的 RGB-D、检测和 TF 数据验证 `/semantic_mask`；
- 再对同一起终点分别使用 `file` 和 `topic` 输入；
- 模型权重继续由 `.gitignore` 排除。

### 阶段 4：受控实车实验

- 先做静止规划验证，再做低速闭环；
- 设置急停人员和物理安全边界；
- 记录成功率、接触/急停、路径长度、穿越率、重规划次数和端到端时延；
- 对漏检和误检做受控注入，不把普通在线结果当作因果验证。
