import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('ultralytics_ros2'),
        'model',
        'yolo11n.pt',
    )
    return LaunchDescription([
        Node(
            package='ultralytics_ros2',
            executable='detection_node',
            name='yolo_detector',
            parameters=[
                {'model': model_path},
                {'input_image_topic': '/image_raw'},
                {'enable_cuda': True},
                {'conf_threshold': 0.5}
            ]
        )
    ])
