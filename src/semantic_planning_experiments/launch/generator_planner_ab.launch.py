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
    cost_mode = LaunchConfiguration('cost_mode')
    task_urgency = LaunchConfiguration('task_urgency')
    avoidance_level = LaunchConfiguration('avoidance_level')
    detection_active_duration = LaunchConfiguration(
        'detection_active_duration_sec'
    )
    observation_hold = LaunchConfiguration('observation_hold_sec')
    observation_decay = LaunchConfiguration('observation_decay_sec')
    recovery_timeout = LaunchConfiguration('recovery_timeout_seconds')
    synthetic_depth_m = LaunchConfiguration('synthetic_depth_m')
    synthetic_depth_noise = LaunchConfiguration(
        'synthetic_depth_noise_std_m'
    )
    synthetic_depth_invalid = LaunchConfiguration(
        'synthetic_depth_invalid_fraction'
    )
    synthetic_random_seed = LaunchConfiguration('synthetic_random_seed')
    synthetic_detection_score = LaunchConfiguration(
        'synthetic_detection_score'
    )
    synthetic_detection_class_id = LaunchConfiguration(
        'synthetic_detection_class_id'
    )
    synthetic_person_count = LaunchConfiguration('synthetic_person_count')
    synthetic_center_x = LaunchConfiguration(
        'synthetic_detection_center_x_fraction'
    )
    synthetic_center_y = LaunchConfiguration(
        'synthetic_detection_center_y_fraction'
    )
    synthetic_person_spacing = LaunchConfiguration(
        'synthetic_person_spacing_y_fraction'
    )
    synthetic_detection_stride = LaunchConfiguration(
        'synthetic_detection_publish_every_n_frames'
    )
    synthetic_timestamp_offset = LaunchConfiguration(
        'synthetic_detection_timestamp_offset_sec'
    )
    risk_value_scale = LaunchConfiguration('risk_value_scale')
    risk_radius_scale = LaunchConfiguration('risk_radius_scale')

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
            'cost_mode',
            default_value='fuzzy',
            choices=['fuzzy', 'fixed', 'lethal'],
            description='Semantic cost transform for the enabled condition.',
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
        DeclareLaunchArgument('synthetic_depth_m', default_value='2.0'),
        DeclareLaunchArgument(
            'synthetic_depth_noise_std_m',
            default_value='0.0',
        ),
        DeclareLaunchArgument(
            'synthetic_depth_invalid_fraction',
            default_value='0.0',
        ),
        DeclareLaunchArgument('synthetic_random_seed', default_value='42'),
        DeclareLaunchArgument(
            'synthetic_detection_score',
            default_value='0.95',
        ),
        DeclareLaunchArgument(
            'synthetic_detection_class_id',
            default_value='person',
            description='Exact semantic class label emitted by the synthetic detector.',
        ),
        DeclareLaunchArgument('synthetic_person_count', default_value='1'),
        DeclareLaunchArgument(
            'synthetic_detection_center_x_fraction',
            default_value='0.5',
        ),
        DeclareLaunchArgument(
            'synthetic_detection_center_y_fraction',
            default_value='0.5',
        ),
        DeclareLaunchArgument(
            'synthetic_person_spacing_y_fraction',
            default_value='0.2',
        ),
        DeclareLaunchArgument(
            'synthetic_detection_publish_every_n_frames',
            default_value='1',
        ),
        DeclareLaunchArgument(
            'synthetic_detection_timestamp_offset_sec',
            default_value='0.0',
        ),
        DeclareLaunchArgument('risk_value_scale', default_value='1.0'),
        DeclareLaunchArgument('risk_radius_scale', default_value='1.0'),
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
                    'risk_value_scale': risk_value_scale,
                    'risk_radius_scale': risk_radius_scale,
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
                    'depth_m': synthetic_depth_m,
                    'depth_noise_std_m': synthetic_depth_noise,
                    'depth_invalid_fraction': synthetic_depth_invalid,
                    'random_seed': synthetic_random_seed,
                    'detection_score': synthetic_detection_score,
                    'detection_class_id': synthetic_detection_class_id,
                    'person_count': synthetic_person_count,
                    'detection_center_x_fraction': synthetic_center_x,
                    'detection_center_y_fraction': synthetic_center_y,
                    'person_spacing_y_fraction': synthetic_person_spacing,
                    'detection_publish_every_n_frames':
                        synthetic_detection_stride,
                    'detection_timestamp_offset_sec':
                        synthetic_timestamp_offset,
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
                'synthetic_source_config': synthetic_config,
                'producer_detection_active_duration_sec':
                    detection_active_duration,
                'producer_observation_hold_sec': observation_hold,
                'producer_observation_decay_sec': observation_decay,
                'producer_depth_m': synthetic_depth_m,
                'producer_depth_noise_std_m': synthetic_depth_noise,
                'producer_depth_invalid_fraction': synthetic_depth_invalid,
                'producer_random_seed': synthetic_random_seed,
                'producer_detection_score': synthetic_detection_score,
                'producer_detection_class_id':
                    synthetic_detection_class_id,
                'producer_person_count': synthetic_person_count,
                'producer_detection_center_x_fraction': synthetic_center_x,
                'producer_detection_center_y_fraction': synthetic_center_y,
                'producer_person_spacing_y_fraction':
                    synthetic_person_spacing,
                'producer_detection_publish_every_n_frames':
                    synthetic_detection_stride,
                'producer_detection_timestamp_offset_sec':
                    synthetic_timestamp_offset,
                'producer_risk_value_scale': risk_value_scale,
                'producer_risk_radius_scale': risk_radius_scale,
                'output_path': output_path,
                'cost_mode': cost_mode,
                'task_urgency': task_urgency,
                'avoidance_level': avoidance_level,
                'recovery_timeout_seconds': recovery_timeout,
            }.items(),
        ),
    ])
