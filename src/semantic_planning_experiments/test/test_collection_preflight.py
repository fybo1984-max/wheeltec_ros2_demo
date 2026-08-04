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

from pathlib import Path
import shutil

import pytest

from semantic_planning_experiments import collection_preflight
from semantic_planning_experiments.collection_preflight import (
    build_collection_plan,
)
from semantic_planning_experiments.collection_preflight import (
    write_collection_plan,
)


_TOPICS = ['/rgb', '/depth', '/detections']
_METADATA = [
    'camera_model_and_serial',
    'detector_model_sha256',
    'map_yaml_sha256',
    'measured_person_pose_in_map',
    'recording_start_and_end_utc',
]


def _inputs(tmp_path: Path, monkeypatch) -> tuple[Path, Path, dict, dict]:
    workspace = tmp_path / 'repo'
    archive = tmp_path / 'archive'
    workspace.mkdir()
    archive.mkdir()
    index_path = archive / 'index.yaml'
    index_path.write_text('index: test\n', encoding='utf-8')
    validated = {
        'archive_root': archive,
        'required_topics': list(_TOPICS),
        'required_metadata': list(_METADATA),
        'units': [{
            'unit_id': 'near_001',
            'stratum': 'near',
            'ordinal': 1,
            'status': 'planned',
            'bag_path': archive / 'bags/near_001',
            'metadata_path': archive / 'metadata/near_001.json',
        }],
    }
    lock = {
        'freeze_fingerprint_sha256': 'a' * 64,
        'code_revision': 'b' * 40,
        'protocol': {
            'protocol_id': 'recorded_rgbd_pilot_v1',
            'collection_scope': {
                'controller_allowed': False,
                'hardware_motion_allowed': False,
                'velocity_commands_allowed': False,
                'physical_camera_allowed': True,
                'operator_notification_required': True,
            },
        },
    }
    monkeypatch.setattr(
        collection_preflight,
        'load_validated_archive_index',
        lambda index, root: (validated, lock),
    )
    return workspace, index_path, validated, lock


def test_plan_is_explicitly_unexecuted_and_contains_metadata_draft(
    tmp_path: Path,
    monkeypatch,
):
    workspace, index_path, _, _ = _inputs(tmp_path, monkeypatch)

    plan = build_collection_plan(
        index_path,
        'near_001',
        workspace,
        minimum_free_gib=0.000001,
    )

    assert plan['preflight_ready']
    assert not plan['commands_executed']
    assert plan['operator_authorization_required']
    assert plan['safety']['rosbag_started'] is False
    assert plan['record_command_argv'][:5] == [
        'ros2',
        'bag',
        'record',
        '--output',
        str(index_path.parent / 'bags/near_001'),
    ]
    assert plan['record_command_argv'][5:] == _TOPICS
    assert set(plan['metadata_draft']) == set(_METADATA)
    assert plan['metadata_draft']['measured_person_pose_in_map'] == {
        'x': None,
        'y': None,
        'frame_id': 'map',
    }


def test_plan_includes_frozen_stable_observation_requirements(
    tmp_path: Path,
    monkeypatch,
):
    workspace, index_path, validated, lock = _inputs(tmp_path, monkeypatch)
    validated['required_metadata'].append('observation_conditions')
    requirements = {
        'recording_duration_s': {'minimum': 10.0, 'maximum': 15.0},
        'measured_person_pose': {
            'frame_id': 'map',
            'maximum_uncertainty_m': 0.05,
            'measured_before_recording': True,
        },
        'observation': {
            'person_stable_before_recording': True,
            'setup_motion_recorded': False,
        },
    }
    lock['protocol']['data_collection'] = {
        'requirements': requirements,
    }

    plan = build_collection_plan(
        index_path,
        'near_001',
        workspace,
        minimum_free_gib=0.000001,
    )

    assert plan['collection_requirements'] == requirements
    pose = plan['metadata_draft']['measured_person_pose_in_map']
    assert pose['measurement_method'] == ''
    assert pose['uncertainty_m'] is None
    assert plan['metadata_draft']['observation_conditions'] == {
        'person_stable_before_recording': None,
        'setup_motion_recorded': None,
    }
    assert any('10.0 to 15.0 seconds' in action for action in (
        plan['operator_actions_required']
    ))


def test_plan_rejects_unknown_unit(tmp_path: Path, monkeypatch):
    workspace, index_path, _, _ = _inputs(tmp_path, monkeypatch)

    with pytest.raises(ValueError, match='does not exist'):
        build_collection_plan(index_path, 'far_001', workspace, 0.000001)


def test_plan_rejects_nonplanned_unit(tmp_path: Path, monkeypatch):
    workspace, index_path, validated, _ = _inputs(tmp_path, monkeypatch)
    validated['units'][0]['status'] = 'collected'

    with pytest.raises(ValueError, match='not planned'):
        build_collection_plan(index_path, 'near_001', workspace, 0.000001)


def test_plan_rejects_existing_unit_artifact(tmp_path: Path, monkeypatch):
    workspace, index_path, validated, _ = _inputs(tmp_path, monkeypatch)
    artifact = validated['units'][0]['metadata_path']
    artifact.parent.mkdir()
    artifact.write_text('{}\n', encoding='utf-8')

    with pytest.raises(ValueError, match='already has artifacts'):
        build_collection_plan(index_path, 'near_001', workspace, 0.000001)


def test_plan_rejects_velocity_topic(tmp_path: Path, monkeypatch):
    workspace, index_path, validated, _ = _inputs(tmp_path, monkeypatch)
    validated['required_topics'].append('/robot/cmd_vel_nav')

    with pytest.raises(ValueError, match='velocity topics are forbidden'):
        build_collection_plan(index_path, 'near_001', workspace, 0.000001)


def test_plan_rejects_insufficient_disk_space(tmp_path: Path, monkeypatch):
    workspace, index_path, _, _ = _inputs(tmp_path, monkeypatch)
    usage = shutil._ntuple_diskusage(total=1000, used=900, free=100)
    monkeypatch.setattr(collection_preflight.shutil, 'disk_usage', lambda _: usage)

    with pytest.raises(ValueError, match='insufficient free space'):
        build_collection_plan(index_path, 'near_001', workspace, 1.0)


def test_writer_rejects_repository_output(tmp_path: Path, monkeypatch):
    workspace, index_path, _, _ = _inputs(tmp_path, monkeypatch)
    plan = build_collection_plan(index_path, 'near_001', workspace, 0.000001)

    with pytest.raises(ValueError, match='outside the repository'):
        write_collection_plan(plan, workspace / 'plan.json', workspace)


def test_writer_is_atomic_and_refuses_overwrite(tmp_path: Path, monkeypatch):
    workspace, index_path, _, _ = _inputs(tmp_path, monkeypatch)
    plan = build_collection_plan(index_path, 'near_001', workspace, 0.000001)
    output = tmp_path / 'plans/near_001.json'

    assert write_collection_plan(plan, output, workspace) == output
    assert '"commands_executed": false' in output.read_text(encoding='utf-8')
    with pytest.raises(ValueError, match='already exists'):
        write_collection_plan(plan, output, workspace)
