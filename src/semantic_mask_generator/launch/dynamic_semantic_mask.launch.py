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

"""Launch only the dynamic mask generator; no camera or robot is started."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the opt-in semantic mask launch description."""
    config = os.path.join(
        get_package_share_directory('semantic_mask_generator'),
        'config',
        'semantic_mask.yaml',
    )
    enabled = LaunchConfiguration('enabled')
    depth_is_registered = LaunchConfiguration('depth_is_registered')
    marker_enabled = LaunchConfiguration('marker_enabled')
    loading_zone_enabled = LaunchConfiguration('loading_zone_enabled')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled',
            default_value='false',
            description='Enable RGB-D detection projection and mask publishing.',
        ),
        DeclareLaunchArgument(
            'depth_is_registered',
            default_value='true',
            description='Confirm depth pixels are aligned to the detection image.',
        ),
        DeclareLaunchArgument(
            'marker_enabled',
            default_value='false',
            description='Accept configured ArUco marker IDs as semantic risks.',
        ),
        DeclareLaunchArgument(
            'loading_zone_enabled',
            default_value='false',
            description='Accept the configured loading-zone active state.',
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
                    'depth_is_registered': depth_is_registered,
                    'marker_enabled': marker_enabled,
                    'loading_zone_enabled': loading_zone_enabled,
                },
            ],
        ),
    ])
