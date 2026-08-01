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
import subprocess

import pytest
import yaml

from semantic_planning_experiments.archive_audit import audit_archive_index
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
    (bag / 'recording_0.db3').write_bytes(b'rosbag data')
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
