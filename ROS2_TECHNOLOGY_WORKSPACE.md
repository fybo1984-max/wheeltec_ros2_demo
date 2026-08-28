# Ros2_Technology 独立优化工作空间

## 项目定位

本目录用于在稳定演示版基础上进行 ROS 2 系统优化和功能实验，不影响随时可用的原版。

- 优化工作目录：`/home/wheeltec/Ros2_Technology`
- 优化开发分支：`ros2-technology`
- 稳定演示目录：`/home/wheeltec/wheeltec_ros2`
- 稳定演示分支：`main`

稳定演示目录只读。源码、参数、启动文件、地图和测试修改只能在本目录中进行。

## 当前基线

- 基于创建项目时稳定版的提交 `ec66ff56e1231127650a6bd4dd26f449b07610cf`。
- 同步了创建项目时稳定目录中的最新语音导航点位配置。
- 没有复制 `build/`、`install/`、`log/`，避免旧工作空间绝对路径污染。
- 私密配置、厂商 SDK 和本地模型继续由 `.gitignore` 排除，不上传 GitHub。

## 使用规则

1. 所有命令先确认当前目录是 `/home/wheeltec/Ros2_Technology`。
2. 第一次构建必须在本目录生成全新的 `build/`、`install/`、`log/`。
3. 每项优化单独提交；可以验证、比较和回退。
4. 未确认安全前，不启动底盘运动，不向 `/cmd_vel` 发布测试速度。
5. 需要演示稳定成果时，关闭本项目进程后再进入稳定目录启动。

## 首次构建

```bash
cd /home/wheeltec/Ros2_Technology
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

第一次完整构建前，先按实际任务确定需要编译的包；大型工作空间优先增量编译目标包。

## 快速检查

```bash
cd /home/wheeltec/Ros2_Technology
git status --short --branch
git log -1 --oneline
```

预期分支为 `ros2-technology`。如果当前路径是 `/home/wheeltec/wheeltec_ros2`，立即停止修改。
