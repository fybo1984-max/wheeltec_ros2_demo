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

"""Validate a recorded RGB-D bag before controller-free replay."""

import math
from pathlib import Path
import re

from semantic_planning_experiments.archive_audit import inspect_recorded_bag


RECORDED_RGBD_TOPICS = [
    '/camera/color/image_raw',
    '/camera/depth/image_raw',
    '/camera/color/camera_info',
    '/tf',
    '/tf_static',
    '/detections',
]
RECORDED_TF_TOPICS = ['/tf', '/tf_static']
TF_PRIME_RATE = 10.0
TF_PRIME_LEAD_SECONDS = 3.0
_UNIT_ID_PATTERN = re.compile(r'^[a-z0-9][a-z0-9_-]*$')


def build_recorded_replay(
    bag_path: Path,
    unit_id: str,
    workspace_root: Path,
    playback_rate: float,
    playback_start_delay_seconds: float,
    runner_delay_seconds: float,
    playback_start_offset_seconds: float = 0.0,
) -> dict:
    """Build a validated `ros2 bag play` command for the recorded launch."""
    bag_path = bag_path.expanduser().resolve()
    workspace_root = workspace_root.expanduser().resolve()
    try:
        bag_path.relative_to(workspace_root)
    except ValueError:
        pass
    else:
        raise ValueError('recorded bags must remain outside the repository')
    if not isinstance(unit_id, str) or not _UNIT_ID_PATTERN.fullmatch(unit_id):
        raise ValueError('recorded unit_id is invalid')
    numeric_values = {
        'playback_rate': playback_rate,
        'playback_start_delay_seconds': playback_start_delay_seconds,
        'runner_delay_seconds': runner_delay_seconds,
        'playback_start_offset_seconds': playback_start_offset_seconds,
    }
    for name, value in numeric_values.items():
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ValueError(f'{name} must be finite')
    if playback_rate <= 0.0:
        raise ValueError('playback_rate must be positive')
    if playback_start_delay_seconds < 0.0:
        raise ValueError('playback_start_delay_seconds must be nonnegative')
    if playback_start_offset_seconds < 0.0:
        raise ValueError('playback_start_offset_seconds must be nonnegative')
    if runner_delay_seconds <= (
        playback_start_delay_seconds + TF_PRIME_LEAD_SECONDS
    ):
        raise ValueError('runner_delay_seconds must follow playback start')
    bag = inspect_recorded_bag(bag_path, RECORDED_RGBD_TOPICS)
    command_argv = [
        'ros2',
        'bag',
        'play',
        str(bag_path),
        '--rate',
        str(float(playback_rate)),
    ]
    if playback_start_offset_seconds > 0.0:
        command_argv.extend([
            '--start-offset',
            str(float(playback_start_offset_seconds)),
        ])
    command_argv.extend(['--topics', *RECORDED_RGBD_TOPICS])
    tf_prime_command_argv = [
        'ros2',
        'bag',
        'play',
        str(bag_path),
        '--rate',
        str(TF_PRIME_RATE),
        '--topics',
        *RECORDED_TF_TOPICS,
        '--disable-keyboard-controls',
    ]
    return {
        'unit_id': unit_id,
        'bag': bag,
        'playback_rate': float(playback_rate),
        'playback_start_delay_seconds': float(
            playback_start_delay_seconds
        ),
        'runner_delay_seconds': float(runner_delay_seconds),
        'playback_start_offset_seconds': float(
            playback_start_offset_seconds
        ),
        'command_argv': command_argv,
        'tf_prime_command_argv': tf_prime_command_argv,
        'tf_prime_rate': TF_PRIME_RATE,
        'tf_prime_lead_seconds': TF_PRIME_LEAD_SECONDS,
        'safety_scope': {
            'controller_started': False,
            'velocity_commands_published': False,
            'hardware_nodes_started': False,
        },
    }
