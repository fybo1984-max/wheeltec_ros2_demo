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

"""Launch synthetic RGB-D through the real generator and Nav2 planner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the controller-free generator-to-planner integration launch."""
    experiment_share = get_package_share_directory(
        'semantic_planning_experiments'
    )
    generator_share = get_package_share_directory('semantic_mask_generator')
    generator_config = os.path.join(
        generator_share, 'config', 'semantic_mask.yaml'
    )
    synthetic_config = os.path.join(
        experiment_share, 'config', 'synthetic_rgbd.yaml'
    )
    planner_launch = os.path.join(
        experiment_share, 'launch', 'planner_ab.launch.py'
    )
    domain_id = LaunchConfiguration('domain_id')
    output_path = LaunchConfiguration('output_path')
    task_urgency = LaunchConfiguration('task_urgency')
    avoidance_level = LaunchConfiguration('avoidance_level')
    detection_active_duration = LaunchConfiguration(
        'detection_active_duration_sec'
    )
    observation_hold = LaunchConfiguration('observation_hold_sec')
    observation_decay = LaunchConfiguration('observation_decay_sec')
    recovery_timeout = LaunchConfiguration('recovery_timeout_seconds')

    return LaunchDescription([
        DeclareLaunchArgument(
            'domain_id',
            default_value='75',
            description='Isolated ROS domain for synthetic RGB-D planning.',
        ),
        DeclareLaunchArgument(
            'output_path',
            default_value='/tmp/semantic_generator_planning_ab.json',
            description='Controller-free generator integration report.',
        ),
        DeclareLaunchArgument(
            'task_urgency',
            default_value='0',
            description='Semantic layer task urgency in [0, 10].',
        ),
        DeclareLaunchArgument(
            'avoidance_level',
            default_value='70.0',
            description='Semantic layer avoidance level in [0, 100].',
        ),
        DeclareLaunchArgument(
            'detection_active_duration_sec',
            default_value='0.0',
            description='Stop synthetic detections after this duration; zero is continuous.',
        ),
        DeclareLaunchArgument(
            'observation_hold_sec',
            default_value='60.0',
            description='Generator risk hold duration.',
        ),
        DeclareLaunchArgument(
            'observation_decay_sec',
            default_value='60.0',
            description='Generator risk decay duration.',
        ),
        DeclareLaunchArgument(
            'recovery_timeout_seconds',
            default_value='0.0',
            description='Wait for mask clearing and plan a recovery condition.',
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        Node(
            package='semantic_mask_generator',
            executable='semantic_mask_node',
            name='semantic_mask_generator',
            output='screen',
            parameters=[
                generator_config,
                {
                    'enabled': True,
                    'depth_is_registered': True,
                    'observation_hold_sec': observation_hold,
                    'observation_decay_sec': observation_decay,
                },
            ],
        ),
        Node(
            package='semantic_planning_experiments',
            executable='synthetic_rgbd_source',
            name='synthetic_rgbd_source',
            output='screen',
            parameters=[
                synthetic_config,
                {
                    'detections_active_duration_sec':
                        detection_active_duration,
                },
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='synthetic_camera_transform',
            output='screen',
            arguments=[
                '--x', '11.0',
                '--y', '0.0',
                '--z', '1.0',
                '--roll', '0.0',
                '--pitch', '1.5707963267948966',
                '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'synthetic_camera_optical_frame',
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(planner_launch),
            launch_arguments={
                'domain_id': domain_id,
                'mask_source': 'topic',
                'topic_input_mode': 'external',
                'mask_producer_config': generator_config,
                'producer_detection_active_duration_sec':
                    detection_active_duration,
                'producer_observation_hold_sec': observation_hold,
                'producer_observation_decay_sec': observation_decay,
                'output_path': output_path,
                'task_urgency': task_urgency,
                'avoidance_level': avoidance_level,
                'recovery_timeout_seconds': recovery_timeout,
            }.items(),
        ),
    ])
