# 语义导航论文实验协议

## 协议状态

- 协议版本：Pilot v1
- 研究对象：WHEELTEC ROS 2 Humble 工业 AGV 语义感知全局规划
- 当前范围：无控制器合成 RGB-D 软件链路
- 当前 manifest：
  `src/semantic_planning_experiments/config/paper_pilot_manifest.yaml`
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

## 方法组

Pilot v1 当前包含：

1. `baseline`：同一 Nav2 planner，关闭语义层；
2. `semantic`：同一 Nav2 planner，开启动态语义软成本；
3. `recovered_after_mask_clear`：风险清空后保持语义层开启再次规划。

正式论文还需要增加：

1. hard obstacle：将相同风险区域作为致命障碍；
2. fixed semantic cost：固定软成本，不使用任务策略模糊推理；
3. proposed method：动态风险强度、策略参数和时间衰减的完整方法。

## Pilot v1 场景

| 场景 | RQ | 重复数 | 策略 | 主要验收条件 |
| --- | --- | ---: | --- | --- |
| `person_center_safety` | RQ1 | 3 | urgency 0 / avoidance 100 | 穿越 ≤ 0.05 m，间距 ≥ 1.0 m |
| `person_center_balanced` | RQ2 | 3 | urgency 5 / avoidance 70 | 穿越 ≤ 0.05 m，路径增量 ≤ 4.5 m |
| `person_center_urgent` | RQ2 | 3 | urgency 10 / avoidance 0 | 穿越 ≥ 2.0 m，路径 ≤ 14.1 m |
| `person_center_decay_recovery` | RQ3 | 2 | urgency 0 / avoidance 70 | 风险期不穿越，恢复长度差为 0 |

展开后共 11 个 trial，使用独立 ROS domain 90–100。manifest 只生成计划，
不会执行 launch。

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

1. Pilot v2：人员位置、数量和风险半径矩阵；
2. Robustness v1：深度噪声、漏检、TF 延迟和过期检测；
3. Recorded RGB-D v1：真实 Rosbag 回放；
4. Physical AGV v1：静止规划和低速闭环；
5. Final paper：冻结方法组、样本量、统计检验和论文图表。
