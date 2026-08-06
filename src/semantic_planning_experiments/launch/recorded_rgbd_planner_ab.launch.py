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

"""Replay recorded RGB-D into the real mask generator and Nav2 planner."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from semantic_planning_experiments.recorded_replay import (
    build_recorded_replay,
)


def _workspace_root(package_share: str) -> str:
    resolved = Path(package_share).resolve()
    for candidate in (resolved, *resolved.parents):
        if (candidate / '.git').is_dir():
            return str(candidate)
    return str(Path.cwd().resolve())


def _start_recorded_pipeline(context, experiment_share, generator_config):
    bag_path = Path(LaunchConfiguration('bag_path').perform(context))
    unit_id = LaunchConfiguration('unit_id').perform(context)
    workspace_root = Path(
        LaunchConfiguration('workspace_root').perform(context)
    )
    playback_rate = float(
        LaunchConfiguration('playback_rate').perform(context)
    )
    playback_delay = float(
        LaunchConfiguration(
            'playback_start_delay_seconds'
        ).perform(context)
    )
    runner_delay = float(
        LaunchConfiguration('runner_delay_seconds').perform(context)
    )
    playback_offset = float(
        LaunchConfiguration('playback_start_offset_seconds').perform(context)
    )
    replay = build_recorded_replay(
        bag_path,
        unit_id,
        workspace_root,
        playback_rate,
        playback_delay,
        runner_delay,
        playback_offset,
    )
    planner_launch = os.path.join(
        experiment_share,
        'launch',
        'planner_ab.launch.py',
    )
    generator = Node(
        package='semantic_mask_generator',
        executable='semantic_mask_node',
        name='semantic_mask_generator',
        output='screen',
        parameters=[
            generator_config,
            {
                'enabled': True,
                'depth_is_registered': True,
                'depth_image_topic': (
                    '/camera/depth/image_raw'
                ),
                'observation_hold_sec': LaunchConfiguration(
                    'observation_hold_sec'
                ),
                'observation_decay_sec': LaunchConfiguration(
                    'observation_decay_sec'
                ),
                'risk_value_scale': LaunchConfiguration(
                    'risk_value_scale'
                ),
                'risk_radius_scale': LaunchConfiguration(
                    'risk_radius_scale'
                ),
            },
        ],
    )
    actions = [generator]
    playback_delay = replay['playback_start_delay_seconds']
    actions.append(TimerAction(
        period=playback_delay,
        actions=[ExecuteProcess(
            cmd=replay['tf_prime_command_argv'],
            output='screen',
        )],
    ))
    playback_delay += replay['tf_prime_lead_seconds']
    playback = TimerAction(
        period=playback_delay,
        actions=[ExecuteProcess(cmd=replay['command_argv'], output='screen')],
    )
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(planner_launch),
        launch_arguments={
            'domain_id': LaunchConfiguration('domain_id'),
            'map_yaml': LaunchConfiguration('map_yaml'),
            'mask_source': 'topic',
            'topic_input_mode': 'external',
            'mask_producer_config': generator_config,
            'synthetic_source_config': '',
            'recorded_bag_path': str(bag_path.expanduser().resolve()),
            'recorded_unit_id': unit_id,
            'recorded_playback_start_offset_seconds': str(playback_offset),
            'recorded_tf_preloaded': 'true',
            'producer_observation_hold_sec': LaunchConfiguration(
                'observation_hold_sec'
            ),
            'producer_observation_decay_sec': LaunchConfiguration(
                'observation_decay_sec'
            ),
            'producer_risk_value_scale': LaunchConfiguration(
                'risk_value_scale'
            ),
            'producer_risk_radius_scale': LaunchConfiguration(
                'risk_radius_scale'
            ),
            'output_path': LaunchConfiguration('output_path'),
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'start_yaw': LaunchConfiguration('start_yaw'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_yaw': LaunchConfiguration('goal_yaw'),
            'cost_mode': LaunchConfiguration('cost_mode'),
            'task_urgency': LaunchConfiguration('task_urgency'),
            'avoidance_level': LaunchConfiguration('avoidance_level'),
            'mask_publish_timeout_seconds': LaunchConfiguration(
                'mask_publish_timeout_seconds'
            ),
            'runner_delay_seconds': str(replay['runner_delay_seconds']),
        }.items(),
    )
    actions.extend([playback, planner])
    return actions


def generate_launch_description():
    """Create the controller-free recorded RGB-D replay launch."""
    experiment_share = get_package_share_directory(
        'semantic_planning_experiments'
    )
    generator_share = get_package_share_directory('semantic_mask_generator')
    generator_config = os.path.join(
        generator_share,
        'config',
        'semantic_mask.yaml',
    )
    navigation_share = get_package_share_directory('wheeltec_nav2')
    default_workspace = _workspace_root(experiment_share)
    default_map = os.path.join(
        navigation_share,
        'map',
        'large_loop_final.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('bag_path', description='Registered bag path.'),
        DeclareLaunchArgument('unit_id', description='Archive unit id.'),
        DeclareLaunchArgument(
            'workspace_root',
            default_value=default_workspace,
        ),
        DeclareLaunchArgument('domain_id', default_value='76'),
        DeclareLaunchArgument('map_yaml', default_value=default_map),
        DeclareLaunchArgument(
            'output_path',
            default_value='/tmp/semantic_recorded_rgbd_ab.json',
        ),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'playback_start_offset_seconds',
            default_value='0.0',
        ),
        DeclareLaunchArgument(
            'playback_start_delay_seconds',
            default_value='2.0',
        ),
        DeclareLaunchArgument('runner_delay_seconds', default_value='8.0'),
        DeclareLaunchArgument(
            'mask_publish_timeout_seconds',
            default_value='60.0',
        ),
        DeclareLaunchArgument('observation_hold_sec', default_value='120.0'),
        DeclareLaunchArgument('observation_decay_sec', default_value='30.0'),
        DeclareLaunchArgument('risk_value_scale', default_value='1.0'),
        DeclareLaunchArgument('risk_radius_scale', default_value='1.0'),
        DeclareLaunchArgument('start_x', default_value='6.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_x', default_value='20.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('cost_mode', default_value='fuzzy'),
        DeclareLaunchArgument('task_urgency', default_value='0'),
        DeclareLaunchArgument('avoidance_level', default_value='70.0'),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID',
            LaunchConfiguration('domain_id'),
        ),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        OpaqueFunction(
            function=_start_recorded_pipeline,
            args=[experiment_share, generator_config],
        ),
    ])
