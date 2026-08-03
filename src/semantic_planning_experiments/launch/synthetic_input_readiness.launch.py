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

"""Exercise the live-input readiness contract with synthetic ROS messages."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create a deterministic, hardware-free readiness test."""
    output_path = LaunchConfiguration('output_path')
    source = Node(
        package='semantic_planning_experiments',
        executable='synthetic_rgbd_source',
        name='synthetic_input_source',
        parameters=[{
            'color_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'detections_topic': '/detections',
            'detection_class_id': 'person',
            'publish_rate_hz': 10.0,
        }],
    )
    transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='synthetic_input_transform',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '1.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'synthetic_camera_optical_frame',
        ],
    )
    readiness = Node(
        package='semantic_planning_experiments',
        executable='semantic_live_input_readiness',
        name='semantic_live_input_readiness',
        parameters=[{
            'minimum_messages': 3,
            'timeout_sec': 5.0,
            'output_path': output_path,
        }],
    )
    stop_when_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=readiness,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_path',
            default_value='/tmp/semantic_input_readiness_synthetic.json',
        ),
        source,
        transform,
        readiness,
        stop_when_complete,
    ])
