# WHEELTEC ROS 2 演示与创新代码

这是 WHEELTEC ROS 2 Humble 小车的版本控制仓库，用于保存已经验证可演示的版本，并为后续创新开发提供可回退的基线。

## 分支约定

- `main`：当前已验证的稳定演示版本。
- `demo-stable`：与稳定演示提交完全一致，原则上不直接开发。
- `innovation`：后续创新开发分支。
- `demo-stable-2026-07-30`：当前演示版本的固定标签。

## 当前演示能力

- 2D/Cartographer 地图导航。
- 语音识别与固定指令控制。
- 大模型自然语言底盘控制。
- 语音点位记录和定点导航。
- 会议室等场景的复合路径导航。
- 导航参数、地图人工修正及 RViz 显示配置。

## 重要说明

当前机器人上的 `/home/wheeltec/wheeltec_ros2` 是稳定演示工作空间。建立 Git 仓库不会重新编译、移动或删除该目录中的文件。

`build/`、`install/` 和 `log/` 没有上传，因为其中包含工作空间绝对路径，复制到其他目录会引起 ROS 2 包索引和动态库路径错误。云服务密钥、设备账号配置以及大型厂商模型/SDK 二进制文件也不会上传。

恢复、编译和私密配置说明见 [BACKUP_AND_RECOVERY.md](BACKUP_AND_RECOVERY.md)。

雷达识别算法创新开发规则见
[RADAR_INNOVATION_WORKSPACE.md](RADAR_INNOVATION_WORKSPACE.md)。
