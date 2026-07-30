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
- `largemodel`、`wheeltec_nav2`、两个麦克风包和 Nav2 均从创新工作空间
  的 `install/` 加载。
- 稳定工作空间仍保持在 `main`，没有因本次建立创新工作空间而修改。
