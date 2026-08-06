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

"""Connect an existing Astra color stream to the fragile-goods marker input."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start marker recognition and mask generation without camera or Nav2."""
    config = os.path.join(
        get_package_share_directory('semantic_mask_generator'),
        'config',
        'semantic_mask.yaml',
    )
    enabled = LaunchConfiguration('enabled')
    camera_frame = LaunchConfiguration('camera_frame')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='false',
            description='Opt in to marker recognition and semantic mask output.',
        ),
        DeclareLaunchArgument(
            'marker_size_m',
            default_value='0.15',
            description='Measured outer black-square side length in metres.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/color/image_raw',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_color_optical_frame',
        ),
        DeclareLaunchArgument(
            'image_is_rectified',
            default_value='false',
            description='Use false for the Astra raw color topic.',
        ),
        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='aruco_marker_publisher',
            output='screen',
            parameters=[{
                'marker_size': LaunchConfiguration('marker_size_m'),
                'reference_frame': camera_frame,
                'camera_frame': camera_frame,
                'image_is_rectified': LaunchConfiguration(
                    'image_is_rectified'
                ),
                'use_camera_info': True,
            }],
            remappings=[
                ('/image', LaunchConfiguration('image_topic')),
                ('/camera_info', LaunchConfiguration('camera_info_topic')),
            ],
        ),
        Node(
            package='semantic_mask_generator',
            executable='semantic_mask_node',
            name='semantic_mask_generator',
            output='screen',
            parameters=[
                config,
                {
                    'enabled': enabled,
                    'depth_is_registered': False,
                    'marker_enabled': enabled,
                    'loading_zone_enabled': False,
                },
            ],
        ),
    ])
