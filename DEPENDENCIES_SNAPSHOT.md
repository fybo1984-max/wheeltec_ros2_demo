# 第三方依赖快照

根仓库不直接嵌套保存下列独立 Git 仓库，以避免 Git 将其错误记录成不可恢复的嵌入仓库。

## ORB-SLAM2 ROS 2

```text
目录：src/wheeltec_robot_slam/orb_slam_2_ros/orb_slam_2_ros
远端：https://gitee.com/AzrealNoah/orb_slam_2_ros.git
提交：8f2369306e5d6b7f6ca558de177e1cd5b015bb81
分支：ros2
```

## serial_ros2

```text
目录：src/depend/serial_ros2
远端：https://github.com/jinmenglei/serial_ros2
提交：def526d378501e0d19d0a8724971688a3b174080
分支：master
```

该目录继续保留在当前机器人上，但不作为嵌套仓库加入根仓库。

## 系统环境

```text
ROS 2：Humble
工作空间：/home/wheeltec/wheeltec_ros2
平台：WHEELTEC / Jetson
```
