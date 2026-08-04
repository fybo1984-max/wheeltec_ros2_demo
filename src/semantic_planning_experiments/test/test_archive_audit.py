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

import json
from pathlib import Path
import sqlite3
import subprocess

import pytest
import yaml

from semantic_planning_experiments.archive_audit import audit_archive_index
from semantic_planning_experiments.archive_audit import _audit_collected_unit
from semantic_planning_experiments.archive_audit import _validate_unit_metadata
from semantic_planning_experiments.archive_audit import (
    initialize_archive_index,
)
from semantic_planning_experiments.protocol_freeze import freeze_protocol
from semantic_planning_experiments.protocol_freeze import write_protocol_lock


_TOPICS = ['/rgb', '/depth', '/tf']
_METADATA = [
    'camera_model_and_serial',
    'detector_model_sha256',
    'map_yaml_sha256',
    'measured_person_pose_in_map',
    'recording_start_and_end_utc',
]

_STABLE_REQUIREMENTS = {
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
    'person_distance_strata_m': {
        'near': {'target': 1.2, 'tolerance': 0.1},
        'far': {'target': 3.0, 'tolerance': 0.1},
    },
}


def _protocol() -> dict:
    return {
        'schema_version': 1,
        'protocol_id': 'archive_pilot_v1',
        'title': 'Archive pilot',
        'phase': 'recorded_rgbd_pilot',
        'expected_branch': 'innovation',
        'confirmatory': False,
        'research_questions': [{
            'id': 'RQ1',
            'hypothesis': 'Semantic planning reduces risk crossing.',
        }],
        'methods': ['baseline', 'fuzzy'],
        'experimental_unit': {
            'name': 'independent_bag',
            'independence_rule': 'Record each bag separately.',
            'pseudoreplication_rule': 'Replays are not new units.',
        },
        'endpoints': {
            'primary': [{
                'id': 'crossing',
                'metric': 'semantic.semantic_crossing_length_m',
                'direction': 'lower',
                'analysis': 'paired',
                'rationale': 'Primary safety outcome.',
            }],
            'secondary': [],
        },
        'sample_size': {
            'total_independent_units': 2,
            'allocation': {'near': 1, 'far': 1},
            'purpose': 'variance_estimation',
            'rationale': 'Exercise the archive tool.',
        },
        'statistics': {
            'alpha': 0.05,
            'confidence_level': 0.95,
            'paired_test': 'sign_flip',
            'exact_pair_limit': 16,
            'monte_carlo_samples': 100000,
            'monte_carlo_seed': 20260801,
            'effect_size': 'cohen_dz',
            'zero_variance_effect': 'null',
            'multiple_comparisons': 'none',
        },
        'exclusion_rules': ['Unreadable bag.'],
        'missing_data_policy': 'No imputation.',
        'data_collection': {
            'required_topics': _TOPICS,
            'required_metadata': _METADATA,
            'storage_policy': 'outside_repository',
        },
        'collection_scope': {
            'controller_allowed': False,
            'hardware_motion_allowed': False,
            'velocity_commands_allowed': False,
            'physical_camera_allowed': True,
            'operator_notification_required': True,
        },
        'assets': ['planner.yaml'],
    }


def _git(repo: Path, *arguments: str) -> None:
    subprocess.run(
        ['git', *arguments],
        cwd=repo,
        check=True,
        capture_output=True,
        text=True,
    )


def _archive(tmp_path: Path) -> tuple[Path, Path, Path]:
    repo = tmp_path / 'repo'
    repo.mkdir()
    _git(repo, 'init', '-b', 'innovation')
    _git(repo, 'config', 'user.email', 'archive-test@example.com')
    _git(repo, 'config', 'user.name', 'Archive Test')
    (repo / 'planner.yaml').write_text('planner: test\n', encoding='utf-8')
    protocol_path = repo / 'protocol.yaml'
    protocol_path.write_text(
        yaml.safe_dump(_protocol(), sort_keys=False),
        encoding='utf-8',
    )
    _git(repo, 'add', 'planner.yaml', 'protocol.yaml')
    _git(repo, 'commit', '-m', 'add archive protocol')
    lock = freeze_protocol(protocol_path, repo)
    lock_path = write_protocol_lock(lock, tmp_path / 'protocol-lock.json')
    index_path = tmp_path / 'archive' / 'index.yaml'
    initialize_archive_index(lock_path, repo, index_path)
    return repo, lock_path, index_path


def _read_index(index_path: Path) -> dict:
    return yaml.safe_load(index_path.read_text(encoding='utf-8'))


def _write_index(index_path: Path, index: dict) -> None:
    index_path.write_text(
        yaml.safe_dump(index, sort_keys=False),
        encoding='utf-8',
    )


def _valid_unit_artifacts(index_path: Path, unit_id: str) -> None:
    root = index_path.parent
    bag = root / 'bags' / unit_id
    bag.mkdir(parents=True)
    storage_path = bag / 'recording_0.db3'
    connection = sqlite3.connect(storage_path)
    connection.execute(
        'CREATE TABLE topics('
        'id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL)'
    )
    connection.execute(
        'CREATE TABLE messages('
        'id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
        'timestamp INTEGER NOT NULL, data BLOB NOT NULL)'
    )
    for topic_id, topic in enumerate(_TOPICS, start=1):
        connection.execute(
            'INSERT INTO topics(id, name, type) VALUES (?, ?, ?)',
            (topic_id, topic, 'test/msg/Type'),
        )
        for message_index in range(5):
            connection.execute(
                'INSERT INTO messages(topic_id, timestamp, data) '
                'VALUES (?, ?, ?)',
                (topic_id, message_index, b'test'),
            )
    connection.commit()
    connection.close()
    topics = [
        {
            'topic_metadata': {'name': topic, 'type': 'test/msg/Type'},
            'message_count': 5,
        }
        for topic in _TOPICS
    ]
    (bag / 'metadata.yaml').write_text(
        yaml.safe_dump({
            'rosbag2_bagfile_information': {
                'storage_identifier': 'sqlite3',
                'duration': {'nanoseconds': 12_000_000_000},
                'topics_with_message_count': topics,
                'relative_file_paths': ['recording_0.db3'],
            },
        }),
        encoding='utf-8',
    )
    metadata_dir = root / 'metadata'
    metadata_dir.mkdir(exist_ok=True)
    (metadata_dir / f'{unit_id}.json').write_text(
        json.dumps({
            'camera_model_and_serial': 'D435-TEST',
            'detector_model_sha256': 'a' * 64,
            'map_yaml_sha256': 'b' * 64,
            'measured_person_pose_in_map': {'x': 1.0, 'y': 2.0},
            'recording_start_and_end_utc': {
                'start_utc': '2026-01-01T00:00:00Z',
                'end_utc': '2026-01-01T00:00:10Z',
            },
        }),
        encoding='utf-8',
    )


def _mark_collected(index_path: Path, unit_ids: list[str]) -> None:
    index = _read_index(index_path)
    for unit in index['units']:
        if unit['unit_id'] in unit_ids:
            unit['status'] = 'collected'
    _write_index(index_path, index)


def _stable_metadata(path: Path) -> dict:
    metadata = {
        'measured_person_pose_in_map': {
            'x': 1.0,
            'y': 2.0,
            'frame_id': 'map',
            'measurement_method': 'floor_marker_center',
            'uncertainty_m': 0.03,
            'measured_before_recording': True,
        },
        'recording_start_and_end_utc': {
            'start_utc': '2026-01-01T00:00:00Z',
            'end_utc': '2026-01-01T00:00:12Z',
        },
        'observation_conditions': {
            'person_stable_before_recording': True,
            'setup_motion_recorded': False,
        },
        'assigned_distance_stratum': 'near',
        'measured_camera_to_person_distance_m': 1.2,
    }
    path.write_text(json.dumps(metadata), encoding='utf-8')
    return metadata


def test_stable_observation_metadata_meets_frozen_requirements(tmp_path: Path):
    metadata_path = tmp_path / 'unit.json'
    _stable_metadata(metadata_path)

    result = _validate_unit_metadata(
        metadata_path,
        list(_stable_metadata_fields()),
        _STABLE_REQUIREMENTS,
        'near',
    )

    assert result['values']['measured_person_pose_in_map'][
        'uncertainty_m'
    ] == 0.03


def test_rosbag_duration_must_meet_stable_requirements(tmp_path: Path):
    _, _, index_path = _archive(tmp_path)
    _valid_unit_artifacts(index_path, 'near_001')
    metadata_path = index_path.parent / 'metadata/near_001.json'
    _stable_metadata(metadata_path)
    unit = {
        'bag_path': index_path.parent / 'bags/near_001',
        'bag_relative': 'bags/near_001',
        'metadata_path': metadata_path,
        'metadata_relative': 'metadata/near_001.json',
        'stratum': 'near',
    }

    passed = _audit_collected_unit(
        unit,
        _TOPICS,
        list(_stable_metadata_fields()),
        _STABLE_REQUIREMENTS,
    )
    assert passed['bag']['duration_s'] == 12.0

    bag_metadata_path = unit['bag_path'] / 'metadata.yaml'
    bag_metadata = yaml.safe_load(bag_metadata_path.read_text())
    bag_metadata['rosbag2_bagfile_information']['duration'][
        'nanoseconds'
    ] = 16_000_000_000
    bag_metadata_path.write_text(
        yaml.safe_dump(bag_metadata),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='Rosbag duration violates'):
        _audit_collected_unit(
            unit,
            _TOPICS,
            list(_stable_metadata_fields()),
            _STABLE_REQUIREMENTS,
        )


def _stable_metadata_fields() -> tuple[str, ...]:
    return (
        'measured_person_pose_in_map',
        'recording_start_and_end_utc',
        'observation_conditions',
        'assigned_distance_stratum',
        'measured_camera_to_person_distance_m',
    )


@pytest.mark.parametrize(
    'section, field, value, message',
    [
        (
            'recording_start_and_end_utc',
            'end_utc',
            '2026-01-01T00:00:16Z',
            'recording duration violates',
        ),
        (
            'measured_person_pose_in_map',
            'uncertainty_m',
            0.06,
            'uncertainty_m exceeds',
        ),
        (
            'observation_conditions',
            'setup_motion_recorded',
            True,
            'observation_conditions violate',
        ),
    ],
)
def test_stable_observation_metadata_rejects_protocol_deviation(
    tmp_path: Path,
    section: str,
    field: str,
    value,
    message: str,
):
    metadata_path = tmp_path / 'unit.json'
    metadata = _stable_metadata(metadata_path)
    metadata[section][field] = value
    metadata_path.write_text(json.dumps(metadata), encoding='utf-8')

    with pytest.raises(ValueError, match=message):
        _validate_unit_metadata(
            metadata_path,
            list(_stable_metadata_fields()),
            _STABLE_REQUIREMENTS,
            'near',
        )


@pytest.mark.parametrize(
    'field, value, message',
    [
        (
            'assigned_distance_stratum',
            'far',
            'does not match archive unit',
        ),
        (
            'measured_camera_to_person_distance_m',
            1.31,
            'violates the assigned stratum',
        ),
    ],
)
def test_distance_metadata_rejects_wrong_stratum_or_range(
    tmp_path: Path,
    field: str,
    value,
    message: str,
):
    metadata_path = tmp_path / 'unit.json'
    metadata = _stable_metadata(metadata_path)
    metadata[field] = value
    metadata_path.write_text(json.dumps(metadata), encoding='utf-8')

    with pytest.raises(ValueError, match=message):
        _validate_unit_metadata(
            metadata_path,
            list(_stable_metadata_fields()),
            _STABLE_REQUIREMENTS,
            'near',
        )


def test_initialize_creates_all_units_and_refuses_overwrite(tmp_path: Path):
    repo, lock_path, index_path = _archive(tmp_path)
    index = _read_index(index_path)

    assert [unit['unit_id'] for unit in index['units']] == [
        'near_001',
        'far_001',
    ]
    assert all(unit['status'] == 'planned' for unit in index['units'])
    with pytest.raises(ValueError, match='already exists'):
        initialize_archive_index(lock_path, repo, index_path)


def test_uncollected_archive_reports_every_unit_missing(tmp_path: Path):
    repo, _, index_path = _archive(tmp_path)

    audit = audit_archive_index(index_path, repo)

    assert audit['unit_count'] == 2
    assert audit['status_counts'] == {
        'passed': 0,
        'missing': 2,
        'invalid': 0,
    }
    assert not audit['analysis_ready']


def test_unindexed_existing_artifacts_are_not_silently_missing(
    tmp_path: Path,
):
    repo, _, index_path = _archive(tmp_path)
    _valid_unit_artifacts(index_path, 'near_001')

    audit = audit_archive_index(index_path, repo)

    result = audit['units'][0]
    assert result['audit_status'] == 'invalid'
    assert 'status is still planned' in result['reason']
    assert result['artifacts']['bag']['file_count'] == 2


def test_complete_archive_passes_with_recursive_hashes(tmp_path: Path):
    repo, _, index_path = _archive(tmp_path)
    for unit_id in ('near_001', 'far_001'):
        _valid_unit_artifacts(index_path, unit_id)
    _mark_collected(index_path, ['near_001', 'far_001'])

    audit = audit_archive_index(index_path, repo)

    assert audit['analysis_ready']
    assert audit['status_counts']['passed'] == 2
    bag = audit['units'][0]['artifacts']['bag']
    assert bag['file_count'] == 2
    assert len(bag['inventory_sha256']) == 64
    assert bag['topics'] == {topic: 5 for topic in _TOPICS}


def test_missing_required_topic_marks_collected_unit_invalid(tmp_path: Path):
    repo, _, index_path = _archive(tmp_path)
    _valid_unit_artifacts(index_path, 'near_001')
    metadata_path = index_path.parent / 'bags/near_001/metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text())
    topics = metadata['rosbag2_bagfile_information'][
        'topics_with_message_count'
    ]
    topics.pop()
    metadata_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')
    _mark_collected(index_path, ['near_001'])

    audit = audit_archive_index(index_path, repo)

    result = audit['units'][0]
    assert result['audit_status'] == 'invalid'
    assert 'required topics are missing' in result['reason']


def test_non_sqlite_storage_marks_collected_unit_invalid(tmp_path: Path):
    repo, _, index_path = _archive(tmp_path)
    _valid_unit_artifacts(index_path, 'near_001')
    storage_path = index_path.parent / 'bags/near_001/recording_0.db3'
    storage_path.write_bytes(b'not a SQLite database')
    _mark_collected(index_path, ['near_001'])

    audit = audit_archive_index(index_path, repo)

    result = audit['units'][0]
    assert result['audit_status'] == 'invalid'
    assert 'SQLite storage is invalid' in result['reason']


def test_missing_required_metadata_marks_unit_invalid(tmp_path: Path):
    repo, _, index_path = _archive(tmp_path)
    _valid_unit_artifacts(index_path, 'near_001')
    metadata_path = index_path.parent / 'metadata/near_001.json'
    metadata = json.loads(metadata_path.read_text())
    del metadata['map_yaml_sha256']
    metadata_path.write_text(json.dumps(metadata), encoding='utf-8')
    _mark_collected(index_path, ['near_001'])

    audit = audit_archive_index(index_path, repo)

    result = audit['units'][0]
    assert result['audit_status'] == 'invalid'
    assert 'missing fields' in result['reason']


def test_index_rejects_artifact_path_escape(tmp_path: Path):
    repo, _, index_path = _archive(tmp_path)
    index = _read_index(index_path)
    index['units'][0]['bag_path'] = '../outside'
    _write_index(index_path, index)

    with pytest.raises(ValueError, match='escapes the archive root'):
        audit_archive_index(index_path, repo)


def test_invalid_unit_requires_preserved_reason(tmp_path: Path):
    repo, _, index_path = _archive(tmp_path)
    index = _read_index(index_path)
    index['units'][0]['status'] = 'invalid'
    _write_index(index_path, index)

    with pytest.raises(ValueError, match='needs a reason'):
        audit_archive_index(index_path, repo)


def test_index_rejects_protocol_binding_tamper(tmp_path: Path):
    repo, _, index_path = _archive(tmp_path)
    index = _read_index(index_path)
    index['protocol']['freeze_fingerprint_sha256'] = '0' * 64
    _write_index(index_path, index)

    with pytest.raises(ValueError, match='protocol binding mismatch'):
        audit_archive_index(index_path, repo)


def test_index_rejects_missing_planned_unit(tmp_path: Path):
    repo, _, index_path = _archive(tmp_path)
    index = _read_index(index_path)
    index['units'].pop()
    _write_index(index_path, index)

    with pytest.raises(ValueError, match='missing planned units'):
        audit_archive_index(index_path, repo)
