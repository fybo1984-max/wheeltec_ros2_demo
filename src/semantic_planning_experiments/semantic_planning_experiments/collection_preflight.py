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

"""Prepare a safe, non-executing plan for one recorded RGB-D unit."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import sys

from semantic_planning_experiments.archive_audit import (
    load_validated_archive_index,
)
from semantic_planning_experiments.metrics import sha256_file


_GIB = 1024 ** 3


def _outside_workspace(path: Path, workspace_root: Path) -> None:
    try:
        path.expanduser().resolve().relative_to(
            workspace_root.expanduser().resolve()
        )
    except ValueError:
        return
    raise ValueError('collection plans must remain outside the repository')


def _is_velocity_topic(topic: str) -> bool:
    return any(part.startswith('cmd_vel') for part in topic.split('/'))


def _metadata_draft(
    required_metadata: list[str],
    requirements: dict | None = None,
    stratum: str | None = None,
) -> dict:
    pose_requirement = (requirements or {}).get('measured_person_pose', {})
    pose_template = {
        'x': None,
        'y': None,
        'frame_id': pose_requirement.get('frame_id', 'map'),
    }
    if requirements is not None:
        pose_template.update({
            'measurement_method': '',
            'uncertainty_m': None,
            'measured_before_recording': None,
        })
    templates = {
        'camera_model_and_serial': '',
        'detector_runtime_version': '',
        'detector_model_sha256': '',
        'map_yaml_sha256': '',
        'measured_person_pose_in_map': pose_template,
        'recording_start_and_end_utc': {
            'start_utc': '',
            'end_utc': '',
        },
        'observation_conditions': {
            'person_stable_before_recording': None,
            'setup_motion_recorded': None,
        },
        'assigned_distance_stratum': stratum,
        'measured_camera_to_person_distance_m': None,
    }
    return {
        name: templates.get(name)
        for name in required_metadata
    }


def build_collection_plan(
    index_path: Path,
    unit_id: str,
    workspace_root: Path,
    minimum_free_gib: float = 5.0,
) -> dict:
    """Validate one planned unit and return an unexecuted record plan."""
    if (
        isinstance(minimum_free_gib, bool)
        or not isinstance(minimum_free_gib, (int, float))
        or not math.isfinite(minimum_free_gib)
        or minimum_free_gib <= 0.0
    ):
        raise ValueError('minimum free GiB must be a positive finite number')
    index_path = index_path.expanduser().resolve()
    workspace_root = workspace_root.expanduser().resolve()
    validated, lock = load_validated_archive_index(
        index_path,
        workspace_root,
    )
    units = {
        unit['unit_id']: unit
        for unit in validated['units']
    }
    if unit_id not in units:
        raise ValueError(f'archive unit does not exist: {unit_id}')
    unit = units[unit_id]
    if unit['status'] != 'planned':
        raise ValueError(
            f'archive unit is not planned: {unit_id} ({unit["status"]})'
        )
    existing = [
        str(path)
        for path in (unit['bag_path'], unit['metadata_path'])
        if path.exists()
    ]
    if existing:
        raise ValueError(f'archive unit already has artifacts: {existing}')
    forbidden_topics = [
        topic for topic in validated['required_topics']
        if _is_velocity_topic(topic)
    ]
    if forbidden_topics:
        raise ValueError(
            f'velocity topics are forbidden during collection: '
            f'{forbidden_topics}'
        )
    collection_scope = lock['protocol']['collection_scope']
    required_scope = {
        'controller_allowed': False,
        'hardware_motion_allowed': False,
        'velocity_commands_allowed': False,
        'operator_notification_required': True,
    }
    if any(
        collection_scope.get(name) != expected
        for name, expected in required_scope.items()
    ):
        raise ValueError('protocol collection scope is unsafe')
    disk = shutil.disk_usage(validated['archive_root'])
    required_free_bytes = int(float(minimum_free_gib) * _GIB)
    if disk.free < required_free_bytes:
        raise ValueError(
            'archive filesystem has insufficient free space: '
            f'{disk.free} < {required_free_bytes} bytes'
        )
    required_topics = list(validated['required_topics'])
    requirements = lock['protocol'].get('data_collection', {}).get(
        'requirements'
    )
    actions = [
        'Notify the operator before starting the physical camera.',
        'Confirm the base and controller remain disabled.',
        'Fill every metadata draft field with measured values.',
        'Request separate authorization before executing any command.',
    ]
    if requirements is not None:
        duration = requirements['recording_duration_s']
        distance = requirements['person_distance_strata_m'][unit['stratum']]
        actions.insert(
            2,
            'Record only after the person is stable, with no setup motion, '
            f'for {duration["minimum"]:.1f} to '
            f'{duration["maximum"]:.1f} seconds.',
        )
        actions.insert(
            3,
            f'Measure camera-to-person distance for {unit["stratum"]}: '
            f'{distance["target"]:.2f} +/- '
            f'{distance["tolerance"]:.2f} m.',
        )
    return {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'preflight_ready': True,
        'commands_executed': False,
        'operator_authorization_required': True,
        'protocol': {
            'protocol_id': lock['protocol']['protocol_id'],
            'freeze_fingerprint_sha256': lock[
                'freeze_fingerprint_sha256'
            ],
            'code_revision': lock['code_revision'],
        },
        'archive': {
            'root': str(validated['archive_root']),
            'index_path': str(index_path),
            'index_sha256': sha256_file(index_path),
        },
        'unit': {
            'unit_id': unit['unit_id'],
            'stratum': unit['stratum'],
            'ordinal': unit['ordinal'],
            'bag_path': str(unit['bag_path']),
            'metadata_path': str(unit['metadata_path']),
        },
        'disk': {
            'available_bytes': disk.free,
            'minimum_required_bytes': required_free_bytes,
        },
        'safety': {
            'camera_started': False,
            'detector_started': False,
            'rosbag_started': False,
            'controller_allowed': False,
            'hardware_motion_allowed': False,
            'velocity_commands_allowed': False,
        },
        'required_topics': required_topics,
        'collection_requirements': requirements,
        'record_command_argv': [
            'ros2',
            'bag',
            'record',
            '--output',
            str(unit['bag_path']),
            *required_topics,
        ],
        'metadata_draft': _metadata_draft(
            validated['required_metadata'],
            requirements,
            unit['stratum'],
        ),
        'operator_actions_required': actions,
    }


def write_collection_plan(
    plan: dict,
    output_path: Path,
    workspace_root: Path,
) -> Path:
    """Write one immutable preflight plan outside the repository."""
    output_path = output_path.expanduser().resolve()
    _outside_workspace(output_path, workspace_root)
    if output_path.exists():
        raise ValueError(f'collection plan already exists: {output_path}')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(output_path.name + '.tmp')
    temporary.write_text(
        json.dumps(plan, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary, output_path)
    return output_path


def main(args=None) -> None:
    """Write a collection plan without launching ROS or hardware."""
    parser = argparse.ArgumentParser(
        description='Prepare one recorded RGB-D unit without execution.',
    )
    parser.add_argument('--workspace-root', type=Path, default=Path.cwd())
    parser.add_argument('--archive-index', required=True, type=Path)
    parser.add_argument('--unit-id', required=True)
    parser.add_argument('--minimum-free-gib', type=float, default=5.0)
    parser.add_argument('--output', required=True, type=Path)
    parsed = parser.parse_args(args)
    try:
        plan = build_collection_plan(
            parsed.archive_index,
            parsed.unit_id,
            parsed.workspace_root,
            parsed.minimum_free_gib,
        )
        output_path = write_collection_plan(
            plan,
            parsed.output,
            parsed.workspace_root,
        )
    except (OSError, KeyError, TypeError, ValueError) as error:
        print(f'collection preflight failed: {error}', file=sys.stderr)
        raise SystemExit(2) from error
    print(
        'collection preflight ready; commands_executed=false; '
        'operator_authorization_required=true; '
        f'output={output_path}'
    )
