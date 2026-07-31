# 语义导航论文实验协议

## 协议状态

- 协议版本：Pilot v1
- 研究对象：WHEELTEC ROS 2 Humble 工业 AGV 语义感知全局规划
- 当前范围：无控制器合成 RGB-D 软件链路
- 当前 manifest：
  `src/semantic_planning_experiments/config/paper_pilot_manifest.yaml`
- 消融 manifest：
  `src/semantic_planning_experiments/config/paper_ablation_manifest.yaml`
- 多场景 manifest：
  `src/semantic_planning_experiments/config/paper_scenario_matrix_manifest.yaml`
- 鲁棒性 manifest：
  `src/semantic_planning_experiments/config/paper_robustness_manifest.yaml`
- 仓储对象类别 manifest：
  `src/semantic_planning_experiments/config/paper_object_class_manifest.yaml`
- 当前结果不能替代真实 RGB-D 和实车正式实验。

## 研究问题

### RQ1：语义风险是否改善路径安全性

在存在几何可行替代路径时，动态语义 costmap 是否能降低人员风险区穿越长度，
并提高路径与风险区域的最小间距？

主要指标：

- semantic crossing length；
- semantic crossing ratio；
- minimum semantic clearance。

### RQ2：安全与效率能否通过任务策略调节

`task_urgency` 和 `avoidance_level` 是否能在同一地图、起终点和风险 mask 下，
形成可解释的安全—效率权衡？

主要指标：

- path length；
- semantic crossing length；
- planning latency。

### RQ3：临时风险消失后是否能够恢复

检测停止、观测完成保持和衰减后，语义层保持开启时，规划是否能恢复到无风险
基线，而不保留陈旧代价？

主要指标：

- recovered-minus-baseline path length；
- recovered path SHA-256；
- stale-risk clearance time。

### RQ4：感知异常下是否保持鲁棒

深度噪声、漏检、时间戳偏差和 TF 延迟是否会导致错误风险区域、持续残留或规划
失败？

RQ4 尚未进入 Pilot v1，将在异常注入阶段加入。

### RQ5：不同仓储对象风险档案能否产生可解释的代价差异

在几何位置相同的条件下，人员、叉车、托盘和易碎箱的风险强度与保护半径是否
能形成可复现的 mask 尺寸、路径绕行和安全间距差异？

该问题当前只验证检测后的语义代价链路，不测量 YOLO 对这些类别的识别精度。
除 `person` 外的类别需要后续自定义仓储检测模型或上游标签适配器。

## 方法组

当前实验工具包含：

1. `baseline`：同一 Nav2 planner，关闭语义层；
2. `fixed`：风险强度映射为固定软成本，不使用任务策略模糊推理；
3. `lethal`：将相同风险区域作为致命障碍，仅用于论文对照；
4. `fuzzy`：动态风险强度与任务策略共同决定软成本；
5. `recovered_after_mask_clear`：风险清空后保持语义层开启再次规划。

每个启用语义层的报告均内含同进程、同输入的 `baseline`，并记录 `cost_mode`。
重复试验汇总禁止混合不同模式。

## Pilot v1 场景

| 场景 | RQ | 重复数 | 策略 | 主要验收条件 |
| --- | --- | ---: | --- | --- |
| `person_center_safety` | RQ1 | 3 | urgency 0 / avoidance 100 | 穿越 ≤ 0.05 m，间距 ≥ 1.0 m |
| `person_center_balanced` | RQ2 | 3 | urgency 5 / avoidance 70 | 穿越 ≤ 0.05 m，路径增量 ≤ 4.5 m |
| `person_center_urgent` | RQ2 | 3 | urgency 10 / avoidance 0 | 穿越 ≥ 2.0 m，路径 ≤ 14.1 m |
| `person_center_decay_recovery` | RQ3 | 2 | urgency 0 / avoidance 70 | 风险期不穿越，恢复长度差为 0 |

展开后共 11 个 trial，使用独立 ROS domain 90–100。manifest 只生成计划，
不会执行 launch。

## 方法消融 Pilot v1

| 场景 | 模式 | 重复数 | 对照目的 |
| --- | --- | ---: | --- |
| `person_center_fixed_soft` | `fixed` | 3 | 去除模糊策略推理 |
| `person_center_lethal` | `lethal` | 3 | 硬障碍上界对照 |
| `person_center_proposed_fuzzy` | `fuzzy` | 3 | 完整软语义方法 |

消融计划共 9 个 trial，使用 ROS domain 110–118。每个 trial 同时产生关闭语义
层的 baseline，因此可比较无语义、固定软代价、硬障碍和完整模糊语义四组。
计划生成命令：

```bash
ros2 run semantic_planning_experiments semantic_experiment_plan \
  src/semantic_planning_experiments/config/paper_ablation_manifest.yaml \
  --output /tmp/semantic_paper_ablation_plan.json
```

该命令仍只生成计划，不执行 Nav2 launch。

### 消融 Pilot 工具链验证结果

干净提交 `abe064d` 上完成了每种模式 3 次、共 9 次无控制器合成 RGB-D 试验。
验收器结果为 9 项通过、0 项失败、0 项缺失、0 项无效：

| 模式 | 路径长度均值 ± 总体标准差 (m) | 风险区穿越 (m) | 最小间距 (m) | 规划时间均值 ± 总体标准差 (s) |
| --- | ---: | ---: | ---: | ---: |
| `fixed` | 17.9170 ± 0.0000 | 0.0000 | 1.1087 | 0.7125 ± 0.0113 |
| `lethal` | 18.2578 ± 0.0000 | 0.0000 | 1.1087 | 0.8868 ± 0.0113 |
| `fuzzy` | 17.4593 ± 0.0000 | 0.0000 | 1.1088 | 0.3615 ± 0.0107 |

三种模式各自的语义路径均只有一个 SHA-256，9 次 baseline 也只有一个唯一路径
SHA-256。基线路径为 14.0053 m，并穿越风险区 2.3474 m。当前结果说明四组
对照链路和统计工具可用；由于仅含一个确定性合成场景且每组样本量为 3，不能
作为论文最终效果结论。

## 多场景与鲁棒性计划

多场景 Pilot v1 包含人员距离近/中/远、风险半径小/大和 3 人聚集共 6 个
场景，每个场景重复 3 次，共 18 个 trial，使用 ROS domain 120–137。

鲁棒性 Pilot v1 包含无扰动基线、0.10 m 固定种子深度噪声、50% 固定种子
无效深度、每 2 帧发布一次检测、接近同步阈值的 0.10 s 时间偏移，以及接近
置信度阈值的 0.55 检测分数，共 18 个 trial，使用 ROS domain 140–157。

两份计划均显式保存随机种子和全部注入参数。干净提交 `e47a105` 已分别完成
一个无控制器 smoke trial：3 人聚集场景生成 2523 个风险栅格，语义路径
17.8690 m、穿越 0；0.10 m 固定种子深度噪声场景生成 1809 个风险栅格，
语义路径 16.2976 m、穿越 0。两份完整计划仍各有 17 个 trial 未执行，当前
数字只用于运行时链路验证。

## 仓储对象类别计划

对象类别 Pilot v1 覆盖 `person`、`forklift`、`pallet` 和 `fragile_box`，
每类重复 3 次，共 12 个 trial，使用 ROS domain 160–171。初始风险值/半径
分别为 `100/1.2 m`、`95/1.8 m`、`65/0.8 m`、`85/1.0 m`。这些数值是待现场
标定的实验因素，不是已验证的工业安全阈值。

```bash
ros2 run semantic_planning_experiments semantic_experiment_plan \
  src/semantic_planning_experiments/config/paper_object_class_manifest.yaml \
  --output /tmp/semantic_paper_object_class_plan.json
```

该命令只生成计划，不启动检测器、Nav2 控制器或硬件。

生成报告后，逐 trial 应用 manifest 中的数值验收条件：

```bash
ros2 run semantic_planning_experiments semantic_experiment_evaluate \
  /tmp/semantic_paper_pilot_plan.json \
  --output /tmp/semantic_paper_pilot_evaluation.json
```

只有全部 trial 通过时命令才返回成功；失败、缺失、无效、dirty revision 和
不安全 scope 均保留在 JSON 审计中并返回非零状态。

## 数据有效性规则

正式结果必须同时满足：

1. Git 提交不带 `-dirty`；
2. 仅 source ROS 2 Humble 和创新工作空间；
3. 地图、规划器、生成器和 manifest 均记录 SHA-256；
4. 每个 trial 使用独立 ROS domain；
5. 原始 JSON、rosbag 和 ROS 日志保存在仓库外；
6. 报告明确记录失败和跳过，禁止静默丢弃；
7. 相同对照组必须通过比较指纹校验；
8. 论文只引用固定提交重新采集的结果。

## 统计规则

- 确定性输入报告唯一路径哈希和完全重复性；
- 含噪声输入报告随机种子、均值、标准差、置信区间和效应量；
- 规划耗时单独报告，不与路径几何确定性混为一谈；
- 实车正式重复数由 pilot 方差和功效分析决定；
- 样本量为 3 的当前结果只用于工具链验证。

## 安全边界

Pilot v1 不允许：

- controller；
- BT navigator；
- 物理相机或检测器；
- 底盘和硬件节点；
- `cmd_vel`。

进入真实相机、Rosbag 录制、完整 Nav2 或实车实验前必须通知现场人员。

## 后续协议修订

1. Scenario Matrix v1：执行已生成的人员距离、数量和风险半径矩阵；
2. Robustness v1：执行已生成的有界感知扰动，并补充超阈值失败注入；
3. Recorded RGB-D v1：真实 Rosbag 回放；
4. Physical AGV v1：静止规划和低速闭环；
5. Final paper：冻结方法组、样本量、统计检验和论文图表。
