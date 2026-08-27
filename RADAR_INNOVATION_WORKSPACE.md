# 雷达创新项目工作空间

## 项目定位

本目录用于在稳定演示版基础上更新和优化雷达感知、目标识别与跟踪算法。

- 雷达创新目录：`/home/wheeltec/wheeltec_ros2_radar_innovation`
- Git 分支：`radar-innovation`
- 稳定演示目录：`/home/wheeltec/wheeltec_ros2`
- 稳定演示分支：`main`

稳定目录只读。所有雷达算法源码、参数、启动文件和测试改动只能发生在雷达
创新目录中。

## 当前基线

- 基于稳定提交 `ec66ff56e1231127650a6bd4dd26f449b07610cf`。
- 同步了创建本项目时稳定目录中最新的语音导航点位。
- 未复制旧工作空间的 `build/`、`install/`、`log/`，避免绝对路径污染。
- 本机厂商 SDK、雷达资源、模型和私密配置保留在本地，并继续由
  `.gitignore` 排除。

## 第一阶段建议范围

1. 确认雷达型号、驱动包、扫描话题、点云/激光帧和发布频率。
2. 录制可重复的静态、行人、障碍物和走廊场景数据。
3. 建立现有算法基线：漏检率、误检率、距离误差、跟踪稳定性和延迟。
4. 分离驱动层、预处理层、检测层、跟踪层及导航融合层，逐层优化。
5. 用 Rosbag 离线回放验证算法后，再进行低速实车测试。

## 安全与开发规则

1. 修改前检查：

   ```bash
   cd /home/wheeltec/wheeltec_ros2_radar_innovation
   pwd
   git branch --show-current
   git status
   ```

2. 不要同时 source 稳定版和雷达创新版。切换项目时使用新终端。
3. 不要同时启动两套系统，以免争用底盘串口、雷达、相机和 ROS 话题。
4. 只进行雷达采集或识别验证时，不启动底盘控制器，不发布 `/cmd_vel`。
5. 涉及车辆移动、自动导航或实车跟踪前，必须先说明将启动的节点和车辆动作。
6. 优先使用 Rosbag 离线回放进行识别算法测试，避免每次修改都让小车运动。
7. 不提交 API Key、设备账号、厂商私密配置、模型权重、Rosbag，以及
   `build/`、`install/`、`log/`。
8. 每项优化都应保留可复现的输入数据、参数、指标和对照结果。

## 构建与加载

首次构建应在干净终端中只加载 ROS Humble：

```bash
cd /home/wheeltec/wheeltec_ros2_radar_innovation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

具体雷达包和最小构建范围应在确认当前雷达驱动后确定，不盲目全量修改。
