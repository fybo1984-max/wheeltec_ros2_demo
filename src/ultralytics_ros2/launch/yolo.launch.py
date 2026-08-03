# Copyright 2026 WHEELTEC innovation workspace
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the warehouse person detector without starting a camera."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Expose sensor, model, output, and inference settings as arguments."""
    default_model = os.path.join(
        get_package_share_directory('ultralytics_ros2'),
        'model',
        'yolo11n.pt',
    )
    model = LaunchConfiguration('model')
    input_topic = LaunchConfiguration('input_image_topic')
    detections_topic = LaunchConfiguration('detections_topic')
    annotated_topic = LaunchConfiguration('annotated_image_topic')
    device = LaunchConfiguration('device')
    confidence = LaunchConfiguration('conf_threshold')
    publish_annotated = LaunchConfiguration('publish_annotated_image')
    detector = Node(
        package='ultralytics_ros2',
        executable='detection_node',
        name='warehouse_person_detector',
        output='screen',
        parameters=[{
            'model': model,
            'input_image_topic': input_topic,
            'detections_topic': detections_topic,
            'annotated_image_topic': annotated_topic,
            'device': device,
            'conf_threshold': ParameterValue(
                confidence,
                value_type=float,
            ),
            'class_names': ['person'],
            'publish_annotated_image': ParameterValue(
                publish_annotated,
                value_type=bool,
            ),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument(
            'input_image_topic',
            default_value='/camera/color/image_raw',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/detections',
        ),
        DeclareLaunchArgument(
            'annotated_image_topic',
            default_value='/semantic/detected_image',
        ),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('conf_threshold', default_value='0.5'),
        DeclareLaunchArgument(
            'publish_annotated_image',
            default_value='true',
        ),
        detector,
    ])
