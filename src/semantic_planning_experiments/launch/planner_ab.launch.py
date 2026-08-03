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

"""Launch a controller-free semantic planning A/B experiment."""

import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _workspace_revision(package_share: str) -> str:
    path = Path(package_share).resolve()
    candidates = [path, *path.parents]
    for candidate in candidates:
        if (candidate / '.git').exists():
            result = subprocess.run(
                ['git', '-C', str(candidate), 'rev-parse', 'HEAD'],
                check=False,
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                revision = result.stdout.strip()
                status = subprocess.run(
                    [
                        'git',
                        '-C',
                        str(candidate),
                        'status',
                        '--porcelain',
                        '--untracked-files=normal',
                    ],
                    check=False,
                    capture_output=True,
                    text=True,
                )
                if status.returncode == 0 and status.stdout.strip():
                    revision += '-dirty'
                return revision
    return 'unknown'


def generate_launch_description():
    experiment_share = get_package_share_directory(
        'semantic_planning_experiments'
    )
    navigation_share = get_package_share_directory('wheeltec_nav2')
    default_map = os.path.join(
        navigation_share, 'map', 'large_loop_final.yaml'
    )
    default_mask = os.path.join(
        navigation_share,
        'map',
        'large_loop_semantic_mask_example.yaml',
    )

    map_yaml = LaunchConfiguration('map_yaml')
    mask_yaml = LaunchConfiguration('mask_yaml')
    mask_source = LaunchConfiguration('mask_source')
    topic_input_mode = LaunchConfiguration('topic_input_mode')
    mask_producer_config = LaunchConfiguration('mask_producer_config')
    synthetic_source_config = LaunchConfiguration(
        'synthetic_source_config'
    )
    recorded_bag_path = LaunchConfiguration('recorded_bag_path')
    recorded_unit_id = LaunchConfiguration('recorded_unit_id')
    recorded_playback_offset = LaunchConfiguration(
        'recorded_playback_start_offset_seconds'
    )
    producer_detection_active_duration = LaunchConfiguration(
        'producer_detection_active_duration_sec'
    )
    producer_observation_hold = LaunchConfiguration(
        'producer_observation_hold_sec'
    )
    producer_observation_decay = LaunchConfiguration(
        'producer_observation_decay_sec'
    )
    producer_depth_m = LaunchConfiguration('producer_depth_m')
    producer_depth_noise = LaunchConfiguration(
        'producer_depth_noise_std_m'
    )
    producer_depth_invalid = LaunchConfiguration(
        'producer_depth_invalid_fraction'
    )
    producer_random_seed = LaunchConfiguration('producer_random_seed')
    producer_detection_score = LaunchConfiguration(
        'producer_detection_score'
    )
    producer_detection_class_id = LaunchConfiguration(
        'producer_detection_class_id'
    )
    producer_person_count = LaunchConfiguration('producer_person_count')
    producer_center_x = LaunchConfiguration(
        'producer_detection_center_x_fraction'
    )
    producer_center_y = LaunchConfiguration(
        'producer_detection_center_y_fraction'
    )
    producer_person_spacing = LaunchConfiguration(
        'producer_person_spacing_y_fraction'
    )
    producer_detection_stride = LaunchConfiguration(
        'producer_detection_publish_every_n_frames'
    )
    producer_timestamp_offset = LaunchConfiguration(
        'producer_detection_timestamp_offset_sec'
    )
    producer_risk_value_scale = LaunchConfiguration(
        'producer_risk_value_scale'
    )
    producer_risk_radius_scale = LaunchConfiguration(
        'producer_risk_radius_scale'
    )
    output_path = LaunchConfiguration('output_path')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_yaw = LaunchConfiguration('goal_yaw')
    settle_seconds = LaunchConfiguration('settle_seconds')
    cost_mode = LaunchConfiguration('cost_mode')
    task_urgency = LaunchConfiguration('task_urgency')
    avoidance_level = LaunchConfiguration('avoidance_level')
    recovery_timeout_seconds = LaunchConfiguration(
        'recovery_timeout_seconds'
    )
    mask_publish_timeout_seconds = LaunchConfiguration(
        'mask_publish_timeout_seconds'
    )
    runner_delay_seconds = LaunchConfiguration('runner_delay_seconds')
    domain_id = LaunchConfiguration('domain_id')

    configured_parameters = RewrittenYaml(
        source_file=os.path.join(
            experiment_share, 'config', 'planner_ab.yaml'
        ),
        param_rewrites={
            'yaml_filename': map_yaml,
            'map_yaml_path': mask_yaml,
            'mask_source': mask_source,
        },
        convert_types=True,
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_parameters],
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_parameters],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_semantic_planning',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'planner_server'],
        }],
    )
    experiment_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='semantic_experiment_transform',
        output='screen',
        arguments=[
            '--x', start_x,
            '--y', start_y,
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', start_yaw,
            '--frame-id', 'map',
            '--child-frame-id', 'semantic_experiment_base',
        ],
    )
    runner = Node(
        package='semantic_planning_experiments',
        executable='semantic_planning_ab',
        name='semantic_planning_ab',
        output='screen',
        parameters=[{
            'map_yaml_path': map_yaml,
            'mask_yaml_path': mask_yaml,
            'mask_source': mask_source,
            'topic_input_mode': topic_input_mode,
            'mask_producer_config_path': mask_producer_config,
            'synthetic_source_config_path': synthetic_source_config,
            'recorded_bag_path': recorded_bag_path,
            'recorded_unit_id': recorded_unit_id,
            'recorded_playback_start_offset_seconds':
                recorded_playback_offset,
            'producer_detection_active_duration_sec':
                producer_detection_active_duration,
            'producer_observation_hold_sec': producer_observation_hold,
            'producer_observation_decay_sec': producer_observation_decay,
            'producer_depth_m': producer_depth_m,
            'producer_depth_noise_std_m': producer_depth_noise,
            'producer_depth_invalid_fraction': producer_depth_invalid,
            'producer_random_seed': producer_random_seed,
            'producer_detection_score': producer_detection_score,
            'producer_detection_class_id': producer_detection_class_id,
            'producer_person_count': producer_person_count,
            'producer_detection_center_x_fraction': producer_center_x,
            'producer_detection_center_y_fraction': producer_center_y,
            'producer_person_spacing_y_fraction': producer_person_spacing,
            'producer_detection_publish_every_n_frames':
                producer_detection_stride,
            'producer_detection_timestamp_offset_sec':
                producer_timestamp_offset,
            'producer_risk_value_scale': producer_risk_value_scale,
            'producer_risk_radius_scale': producer_risk_radius_scale,
            'planner_config_path': os.path.join(
                experiment_share, 'config', 'planner_ab.yaml'
            ),
            'output_path': output_path,
            'start_x': start_x,
            'start_y': start_y,
            'start_yaw': start_yaw,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw': goal_yaw,
            'settle_seconds': settle_seconds,
            'cost_mode': cost_mode,
            'task_urgency': task_urgency,
            'avoidance_level': avoidance_level,
            'recovery_timeout_seconds': recovery_timeout_seconds,
            'mask_publish_timeout_seconds': mask_publish_timeout_seconds,
            'code_revision': _workspace_revision(experiment_share),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'domain_id',
            default_value='73',
            description='Isolated ROS domain for the controller-free test.',
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map,
            description='Absolute Nav2 occupancy map YAML path.',
        ),
        DeclareLaunchArgument(
            'mask_yaml',
            default_value=default_mask,
            description='Absolute Nav2 semantic mask YAML path.',
        ),
        DeclareLaunchArgument(
            'mask_source',
            default_value='file',
            choices=['file', 'topic'],
            description='Load the mask from YAML or publish it as OccupancyGrid.',
        ),
        DeclareLaunchArgument(
            'topic_input_mode',
            default_value='fixture',
            choices=['fixture', 'external'],
            description='Publish the YAML fixture or wait for an external mask.',
        ),
        DeclareLaunchArgument(
            'mask_producer_config',
            default_value='',
            description='Optional external mask producer config for hashing.',
        ),
        DeclareLaunchArgument(
            'synthetic_source_config',
            default_value='',
            description='Optional synthetic RGB-D config for hashing.',
        ),
        DeclareLaunchArgument(
            'recorded_bag_path',
            default_value='',
            description='Optional recorded RGB-D Rosbag directory.',
        ),
        DeclareLaunchArgument(
            'recorded_unit_id',
            default_value='',
            description='Optional independent archive unit id.',
        ),
        DeclareLaunchArgument(
            'recorded_playback_start_offset_seconds',
            default_value='0.0',
            description='Recorded bag start offset used for this report.',
        ),
        DeclareLaunchArgument(
            'producer_detection_active_duration_sec',
            default_value='-1.0',
            description='Recorded external producer detection duration.',
        ),
        DeclareLaunchArgument(
            'producer_observation_hold_sec',
            default_value='-1.0',
            description='Recorded external producer hold duration.',
        ),
        DeclareLaunchArgument(
            'producer_observation_decay_sec',
            default_value='-1.0',
            description='Recorded external producer decay duration.',
        ),
        DeclareLaunchArgument('producer_depth_m', default_value='-1.0'),
        DeclareLaunchArgument(
            'producer_depth_noise_std_m',
            default_value='-1.0',
        ),
        DeclareLaunchArgument(
            'producer_depth_invalid_fraction',
            default_value='-1.0',
        ),
        DeclareLaunchArgument('producer_random_seed', default_value='-1'),
        DeclareLaunchArgument(
            'producer_detection_score',
            default_value='-1.0',
        ),
        DeclareLaunchArgument(
            'producer_detection_class_id',
            default_value='',
        ),
        DeclareLaunchArgument('producer_person_count', default_value='-1'),
        DeclareLaunchArgument(
            'producer_detection_center_x_fraction',
            default_value='-1.0',
        ),
        DeclareLaunchArgument(
            'producer_detection_center_y_fraction',
            default_value='-1.0',
        ),
        DeclareLaunchArgument(
            'producer_person_spacing_y_fraction',
            default_value='-1.0',
        ),
        DeclareLaunchArgument(
            'producer_detection_publish_every_n_frames',
            default_value='-1',
        ),
        DeclareLaunchArgument(
            'producer_detection_timestamp_offset_sec',
            default_value='-1.0',
        ),
        DeclareLaunchArgument(
            'producer_risk_value_scale',
            default_value='-1.0',
        ),
        DeclareLaunchArgument(
            'producer_risk_radius_scale',
            default_value='-1.0',
        ),
        DeclareLaunchArgument(
            'output_path',
            default_value='/tmp/semantic_planning_ab.json',
            description='JSON report path; /tmp avoids repository artifacts.',
        ),
        DeclareLaunchArgument('start_x', default_value='6.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_x', default_value='20.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('settle_seconds', default_value='1.5'),
        DeclareLaunchArgument(
            'cost_mode',
            default_value='fuzzy',
            choices=['fuzzy', 'fixed', 'lethal'],
            description='Semantic cost transform used for the enabled condition.',
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
            'recovery_timeout_seconds',
            default_value='0.0',
            description='Wait for an empty external mask and replan if positive.',
        ),
        DeclareLaunchArgument(
            'mask_publish_timeout_seconds',
            default_value='10.0',
            description='Maximum wait for a nonempty external mask.',
        ),
        DeclareLaunchArgument(
            'runner_delay_seconds',
            default_value='5.0',
            description='Delay before starting controller-free A/B planning.',
        ),
        map_server,
        planner_server,
        lifecycle_manager,
        experiment_transform,
        TimerAction(period=runner_delay_seconds, actions=[runner]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=runner,
                on_exit=[Shutdown()],
            )
        ),
    ])
