import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import launch_ros.actions

#def launch(launch_descriptor, argv):
def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    robot_nav = LaunchConfiguration('robot_nav')
    robot_nav_arg = DeclareLaunchArgument(
        'robot_nav',
        default_value='false',
        description='Use the navigation EKF configuration'
    )
    use_camera = LaunchConfiguration('use_camera')
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to start the configured camera'
    )
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
            launch_arguments={
                'carto_slam': 'false',
                'robot_nav': robot_nav,
            }.items(),
    )
    lidar_ros = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_camera.launch.py')),
            condition=IfCondition(use_camera),
    )

    return LaunchDescription([
        robot_nav_arg,
        use_camera_arg,
        wheeltec_robot,lidar_ros,wheeltec_camera,]
    )
