# 创新工作空间使用说明

## 两套工作空间

- 稳定演示：`/home/wheeltec/wheeltec_ros2`，使用 `main` 分支。
- 创新开发：`/home/wheeltec/wheeltec_ros2_innovation`，使用
  `innovation` 分支。

创新工作空间具有自己的 `build/`、`install/` 和 `log/`，不会使用稳定
工作空间中的编译产物。地图、语音 SDK、模型权重和本机私密配置已保留在
创新工作空间本地，但受 `.gitignore` 保护，不会上传 GitHub。

## 启动稳定演示

请使用一个新的终端：

```bash
cd /home/wheeltec/wheeltec_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch largemodel largemodel_control.launch.py \
  use_nav:=true \
  vad_threshold:=0.7 \
  fixed_command_mode:=true
```

## 启动创新版本

请使用另一个新的终端：

```bash
cd /home/wheeltec/wheeltec_ros2_innovation
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch largemodel largemodel_control.launch.py \
  use_nav:=true \
  vad_threshold:=0.7 \
  fixed_command_mode:=true
```

## 注意事项

1. 不要在同一个终端先后 `source` 两套工作空间。需要切换时关闭该终端，
   再打开新终端。
   `source /opt/ros/humble/setup.bash` 只加载 ROS 2 基础环境，不会清除当前
   终端里已经加载的 overlay。若终端先前 source 过稳定工作空间，再 source
   `/opt/ros/humble/setup.bash` 仍然会残留稳定路径。编译前可检查：

   ```bash
   printenv AMENT_PREFIX_PATH | tr ':' '\n' | \
     grep -F '/home/wheeltec/wheeltec_ros2/install'
   ```

   有输出时必须换全新终端，不要在该终端编译创新工作空间。
2. 不要同时启动两套系统，它们会争用底盘串口、麦克风、雷达、相机和相同
   ROS 话题。
3. 老师需要演示时，停止创新版本，打开新终端并按“启动稳定演示”运行。
4. 创新修改只在 `wheeltec_ros2_innovation` 中进行，不要修改稳定目录。
5. 编译创新版本时使用：

   ```bash
   cd /home/wheeltec/wheeltec_ros2_innovation
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

6. 提交创新代码前先检查：

   ```bash
   git status
   git diff --check
   ```

## 已验证状态

- 独立构建成功：52 个依赖及目标包完成。
- 固定语音指令和点位持久化测试：81 项通过。
- 语义 costmap 插件：10 个 CTest 全部通过，其中 14 个行为测试通过。
- 动态语义 mask：13 项测试通过，其中 10 项投影、衰减、半径缩放和栅格化
  行为测试通过。
- 规划器级 A/B 实验包：52 项测试通过，其中 49 项地图坐标、动态栅格转换、
  路径指标、重复试验统计、论文 manifest、结果验收和成果导出行为测试
  通过；专用 launch 不含 controller、BT、AMCL、传感器或底盘节点。
- 默认模板路线的两次离线规划得到完全相同的路径几何：基线穿越语义区约
  7.50 m，启用语义层后穿越为 0，路径增加约 3.26 m，语义边界最小间距约
  0.64 m。该数字只是工具链验证，不是论文正式实验结果。
- 同一模板分别使用静态文件输入和
  `OccupancyGrid` topic 输入时，两组 A/B 路径逐点一致，除规划耗时外的
  指标完全一致；topic 使用 reliable、transient-local、depth 1 QoS。
- 合成 CameraInfo、16UC1 深度图和 `person` Detection2D 已通过真实
  `semantic_mask_generator` 生成 1806 个风险栅格，再由 topic 送入 Nav2。
  基线路径穿越动态风险区约 2.35 m，启用语义层后穿越为 0，最小间距约
  1.11 m。全程未启动 controller、底盘或硬件节点。
- 动态风险区参数敏感性验证通过：安全优先和均衡参数均完全绕行；紧急度最高、
  避让度最低时恢复约 14.01 m 的短路径并穿越约 2.35 m 风险区，验证该区域
  保持为可调软成本而非硬障碍。
- 动态 mask 清除恢复验证通过：合成人员停止发布并完成 0.5 s 保持、1.0 s
  衰减后，语义层保持开启，规划路径仍从约 17.46 m 绕行路径恢复为 14.01 m
  基线路径，120 个位姿逐点一致。
- 干净提交 `da778c1` 的三次端到端重复试验已自动汇总：比较指纹一致，基线
  和语义路径各只有一个唯一 SHA-256，路径长度、穿越距离和间距标准差均为 0；
  规划耗时保留均值、总体标准差及范围。
- 论文 Pilot v1 manifest 已展开为 4 个场景、11 个计划 trial，独立使用
  ROS domain 90–100；计划生成器固定为无控制器 launch，不执行命令。
- 论文结果验收器对每个 trial 输出通过、失败、缺失或无效状态；未采集报告的
  空白 Pilot 计划已正确识别为 11 项缺失并返回非零状态。
- 论文方法消融 manifest 已展开为固定软代价、硬障碍和完整模糊语义 3 个场景、
  9 个 trial，使用 ROS domain 110–118；每项报告内含关闭语义层 baseline。
- 干净提交 `abe064d` 的 9 次方法消融 Pilot 已全部通过验收；各模式 3 次路径
  几何完全重复。合成场景中 `fuzzy` 路径约 17.46 m，短于 `fixed` 的
  17.92 m 和 `lethal` 的 18.26 m，三者风险区穿越均为 0。该结果仅验证
  工具链，不作为论文最终结论。
- 多场景和鲁棒性 manifest 各包含 6 个场景、18 个计划 trial，分别使用
  ROS domain 120–137 和 140–157；随机种子、深度噪声、无效深度、检测丢帧、
  时间偏移、人员数量及风险半径缩放均写入计划。
- 干净提交 `ce72d97` 的 18 次多场景 Pilot 已全部通过验收，0 失败、0 缺失、
  0 无效，六组内部路径完全重复且风险区穿越均为 0。近/中/远人员投影质心按
  深度顺序移动；小/大半径和三人聚集均形成可行绕行路径。
- 干净提交 `fc45dac` 的 18 次鲁棒性 Pilot 已全部通过验收，0 失败、0 缺失、
  0 无效，六组内部路径完全重复且风险区穿越均为 0。除深度噪声外的四种扰动
  与无扰动路径逐点一致；0.10 m 深度噪声触发不同但仍不穿越风险区的路径。
- 动态风险档案现接受 `person`、`forklift`、`pallet`、`fragile_box` 精确
  标签；对象类别 manifest 包含 4 个场景、12 个计划 trial，使用 ROS domain
  160–171。除 `person` 外的真实识别仍需自定义仓储模型，当前合成试验不声称
  已完成真实 YOLO 识别。
- 干净提交 `b9e1611` 的 12 次对象类别 Pilot 已全部通过验收，无失败、缺失或
  无效报告，四类路径各自完全重复。`person`、`forklift`、`fragile_box`
  完全绕行；低风险 `pallet` 仍穿越约 1.35 m，验证类别档案保持可调软代价。
- 新增独立 `risk_value_scale` 与 `risk_radius_scale` 实验参数和实际 mask
  值摘要；风险因素分离消融计划包含共享基准、2 个风险值水平和 2 个半径水平，
  共 5 个场景、15 个计划 trial，使用 ROS domain 180–194。
- 干净提交 `21f02ef` 的 15 次风险因素分离 Pilot 已全部通过验收，0 失败、
  0 缺失、0 无效，五组路径各自完全重复。风险值 65 允许软穿越，85/100
  完全绕行；固定风险值 100 时，半径增大对应路径长度逐级增加。
- 新增论文成果导出器，将可比场景摘要生成 CSV、SVG 四指标图和 JSON 审计
  manifest。导出前重新核对干净 revision、固定实验输入、路径重复性以及每份
  原始 trial 的 SHA-256。已用 `ce72d97` 的 6 组多场景 Pilot 离线验证：CSV
  6 行、SVG XML 可解析，成果文件哈希均与 manifest 一致；临时成果位于
  `/tmp/semantic_paper_exports/`，不提交 Pilot 数据到仓库。
- 成果导出器支持显式声明方法模式、任务紧急度、避让度和路线实验因素；未声明
  差异或声明后实际未变化都会失败。CSV 同时保存 baseline、semantic 和从原始
  trial 重建的配对差值，避免用四舍五入后的论文表格数字计算效应。
- `largemodel`、`wheeltec_nav2`、两个麦克风包和 Nav2 均从创新工作空间
  的 `install/` 加载。
- 稳定工作空间仍保持在 `main`，没有因本次建立创新工作空间而修改。

## 语义导航创新

工业 AGV 语义 costmap 的设计判断、上游版本、静态 mask PoC、运行安全提示
和分阶段实验方法见 [`SEMANTIC_NAVIGATION.md`](SEMANTIC_NAVIGATION.md)。
面向论文发表的研究问题、方法组、指标、数据有效性规则和 Pilot 场景见
[`PAPER_EXPERIMENT_PROTOCOL.md`](PAPER_EXPERIMENT_PROTOCOL.md)。
