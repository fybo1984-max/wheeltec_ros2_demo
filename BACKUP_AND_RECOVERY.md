# 稳定演示版本与恢复说明

## 1. 当前机器的目录策略

保留以下目录作为随时可演示的稳定工作空间：

```text
/home/wheeltec/wheeltec_ros2
```

不要把后续创新实验直接写入稳定工作空间。创新开发应使用单独目录，例如：

```text
/home/wheeltec/wheeltec_ros2_innovation
```

复制或克隆源码后，不要复制旧工作空间的 `build/`、`install/`、`log/`。这些目录记录了旧工作空间的绝对路径。

## 2. 稳定版本启动

```bash
cd /home/wheeltec/wheeltec_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch largemodel largemodel_control.launch.py \
  use_nav:=true \
  vad_threshold:=0.7 \
  fixed_command_mode:=true
```

## 3. 查看与切换版本

```bash
cd /home/wheeltec/wheeltec_ros2
git status
git branch --show-current
git tag --list
```

不要在小车正在运行时切换代码版本。切换前应停止 ROS 2 启动进程，并提交或暂存当前修改。

稳定版本：

```bash
git switch demo-stable
```

创新版本：

```bash
git switch innovation
```

## 4. 私密配置

以下文件保留在当前机器人本地，但不会上传 GitHub：

```text
src/largemodel/config/model_config.yaml
src/ollama_ros_chat-ros2/ollama_ros_chat/config/ollama_params.yaml
src/tts_make_ros2/config/tts_params.yaml
src/wheeltec_mic_aiui/AIUI/cfg/aiui.cfg
src/wheeltec_mic_aiui/AIUI/assets/vtn/vtn.ini
```

在全新机器恢复时，需要从安全的离线备份复制这些文件，或重新填写并启用对应平台的凭据。不要把密钥粘贴到 Issue、README、提交信息或终端截图中。

## 5. 大型本机资源

语音模型、YOLO 权重、机器人 STL 网格、录音、rosbag、Cartographer 数据库和厂商动态库保留在机器人本机，不进入 Git 仓库。当前稳定工作空间不会删除这些资源。

全新机器恢复时，应先安装/复制对应的 WHEELTEC 厂商基础资料，再从本仓库覆盖受版本管理的源码和配置。

## 6. 干净编译原则

仅在新工作空间或切换源码版本后确实需要时编译：

```bash
cd /home/wheeltec/wheeltec_ros2_innovation
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

稳定演示工作空间当前已经编译完成，不应仅为了建立 Git 备份而重新编译。
