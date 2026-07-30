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
- 语义 costmap 插件：10 个 CTest 全部通过，其中 11 个行为测试通过。
- 动态语义 mask：12 项测试通过，其中 9 项投影、衰减和栅格化行为测试通过。
- 规划器级 A/B 实验包：11 项测试通过，其中 8 项地图坐标、动态栅格转换与路径指标行为测试
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
- `largemodel`、`wheeltec_nav2`、两个麦克风包和 Nav2 均从创新工作空间
  的 `install/` 加载。
- 稳定工作空间仍保持在 `main`，没有因本次建立创新工作空间而修改。

## 语义导航创新

工业 AGV 语义 costmap 的设计判断、上游版本、静态 mask PoC、运行安全提示
和分阶段实验方法见 [`SEMANTIC_NAVIGATION.md`](SEMANTIC_NAVIGATION.md)。
