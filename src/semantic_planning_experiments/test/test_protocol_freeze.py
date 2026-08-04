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

import copy
import json
from pathlib import Path
import subprocess

import pytest
import yaml

from semantic_planning_experiments.protocol_freeze import freeze_protocol
from semantic_planning_experiments.protocol_freeze import verify_protocol_lock
from semantic_planning_experiments.protocol_freeze import write_protocol_lock


def _protocol() -> dict:
    return {
        'schema_version': 1,
        'protocol_id': 'recorded_pilot_v1',
        'title': 'Recorded pilot',
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
            'independence_rule': 'Each bag is recorded separately.',
            'pseudoreplication_rule': 'Bag replay is not a new unit.',
        },
        'endpoints': {
            'primary': [{
                'id': 'risk_crossing',
                'metric': 'semantic.semantic_crossing_length_m',
                'direction': 'lower',
                'analysis': 'paired',
                'rationale': 'Primary safety outcome.',
            }],
            'secondary': [],
        },
        'sample_size': {
            'total_independent_units': 4,
            'allocation': {'near': 2, 'far': 2},
            'purpose': 'variance_estimation',
            'rationale': 'Estimate variability for a later power analysis.',
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
        'exclusion_rules': ['Unreadable bag before analysis.'],
        'missing_data_policy': 'No imputation; preserve missing units.',
        'data_collection': {
            'required_topics': ['/rgb', '/depth', '/tf'],
            'required_metadata': ['camera_serial', 'map_sha256'],
            'storage_policy': 'outside_repository',
        },
        'collection_scope': {
            'controller_allowed': False,
            'hardware_motion_allowed': False,
            'velocity_commands_allowed': False,
            'physical_camera_allowed': True,
            'operator_notification_required': True,
        },
        'assets': ['config/planner.yaml'],
    }


def _git(repo: Path, *arguments: str) -> str:
    result = subprocess.run(
        ['git', *arguments],
        cwd=repo,
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _repository(
    tmp_path: Path,
    protocol: dict | None = None,
) -> tuple[Path, Path]:
    repo = tmp_path / 'repo'
    repo.mkdir()
    _git(repo, 'init', '-b', 'innovation')
    _git(repo, 'config', 'user.email', 'test@example.com')
    _git(repo, 'config', 'user.name', 'Protocol Test')
    config = repo / 'config'
    config.mkdir()
    (config / 'planner.yaml').write_text('planner: test\n', encoding='utf-8')
    protocol_path = repo / 'protocol.yaml'
    protocol_path.write_text(
        yaml.safe_dump(protocol or _protocol(), sort_keys=False),
        encoding='utf-8',
    )
    _git(repo, 'add', 'config/planner.yaml', 'protocol.yaml')
    _git(repo, 'commit', '-m', 'add protocol')
    return repo, protocol_path


def test_freeze_records_revision_assets_and_verifies(tmp_path: Path):
    repo, protocol_path = _repository(tmp_path)

    lock = freeze_protocol(protocol_path, repo)
    lock_path = write_protocol_lock(lock, tmp_path / 'protocol-lock.json')
    verification = verify_protocol_lock(lock_path, repo)

    assert lock['git_branch'] == 'innovation'
    assert lock['code_revision'] == _git(repo, 'rev-parse', 'HEAD')
    assert lock['assets'][0]['path'] == 'config/planner.yaml'
    assert len(lock['assets'][0]['sha256']) == 64
    assert verification['valid']
    assert verification['freeze_fingerprint_sha256'] == (
        lock['freeze_fingerprint_sha256']
    )


def test_freeze_rejects_dirty_worktree(tmp_path: Path):
    repo, protocol_path = _repository(tmp_path)
    protocol_path.write_text('changed: true\n', encoding='utf-8')

    with pytest.raises(ValueError, match='clean Git worktree'):
        freeze_protocol(protocol_path, repo)


def test_freeze_rejects_wrong_branch(tmp_path: Path):
    protocol = _protocol()
    protocol['expected_branch'] = 'main'
    repo, protocol_path = _repository(tmp_path, protocol)

    with pytest.raises(ValueError, match='expected branch main'):
        freeze_protocol(protocol_path, repo)


def test_freeze_rejects_sample_allocation_mismatch(tmp_path: Path):
    protocol = _protocol()
    protocol['sample_size']['total_independent_units'] = 5
    repo, protocol_path = _repository(tmp_path, protocol)

    with pytest.raises(ValueError, match='sample allocation'):
        freeze_protocol(protocol_path, repo)


def test_freeze_rejects_unsafe_collection_scope(tmp_path: Path):
    protocol = _protocol()
    protocol['collection_scope']['hardware_motion_allowed'] = True
    repo, protocol_path = _repository(tmp_path, protocol)

    with pytest.raises(ValueError, match='collection_scope'):
        freeze_protocol(protocol_path, repo)


def test_confirmatory_protocol_requires_multiplicity_control(tmp_path: Path):
    protocol = _protocol()
    protocol['confirmatory'] = True
    protocol['sample_size']['purpose'] = 'confirmatory_powered'
    repo, protocol_path = _repository(tmp_path, protocol)

    with pytest.raises(ValueError, match='multiplicity control'):
        freeze_protocol(protocol_path, repo)


def test_freeze_records_stable_observation_requirements(tmp_path: Path):
    protocol = _protocol()
    protocol['data_collection']['required_metadata'].extend([
        'measured_person_pose_in_map',
        'assigned_distance_stratum',
        'measured_camera_to_person_distance_m',
        'recording_start_and_end_utc',
        'observation_conditions',
    ])
    protocol['data_collection']['requirements'] = {
        'recording_duration_s': {'minimum': 10, 'maximum': 15},
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
    repo, protocol_path = _repository(tmp_path, protocol)

    lock = freeze_protocol(protocol_path, repo)

    requirements = lock['protocol']['data_collection']['requirements']
    assert requirements['recording_duration_s'] == {
        'minimum': 10.0,
        'maximum': 15.0,
    }
    assert requirements['measured_person_pose'][
        'maximum_uncertainty_m'
    ] == 0.05
    assert requirements['person_distance_strata_m']['near'] == {
        'target': 1.2,
        'tolerance': 0.1,
    }


def test_freeze_rejects_reversed_recording_duration(tmp_path: Path):
    protocol = _protocol()
    protocol['data_collection']['required_metadata'].extend([
        'measured_person_pose_in_map',
        'assigned_distance_stratum',
        'measured_camera_to_person_distance_m',
        'recording_start_and_end_utc',
        'observation_conditions',
    ])
    protocol['data_collection']['requirements'] = {
        'recording_duration_s': {'minimum': 15, 'maximum': 10},
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
    repo, protocol_path = _repository(tmp_path, protocol)

    with pytest.raises(ValueError, match='duration maximum'):
        freeze_protocol(protocol_path, repo)


def test_verify_rejects_tampered_lock(tmp_path: Path):
    repo, protocol_path = _repository(tmp_path)
    lock = freeze_protocol(protocol_path, repo)
    lock['protocol']['title'] = 'Changed after freezing'
    lock_path = tmp_path / 'tampered-lock.json'
    lock_path.write_text(json.dumps(lock), encoding='utf-8')

    with pytest.raises(ValueError, match='fingerprint mismatch'):
        verify_protocol_lock(lock_path, repo)


def test_verify_rejects_new_revision(tmp_path: Path):
    repo, protocol_path = _repository(tmp_path)
    lock = freeze_protocol(protocol_path, repo)
    lock_path = write_protocol_lock(lock, tmp_path / 'protocol-lock.json')
    (repo / 'note.txt').write_text('new revision\n', encoding='utf-8')
    _git(repo, 'add', 'note.txt')
    _git(repo, 'commit', '-m', 'new revision')

    with pytest.raises(ValueError, match='revision mismatch'):
        verify_protocol_lock(lock_path, repo)


def test_freeze_rejects_untracked_ignored_asset(tmp_path: Path):
    protocol = copy.deepcopy(_protocol())
    protocol['assets'] = ['ignored.yaml']
    repo, protocol_path = _repository(tmp_path, protocol)
    (repo / '.gitignore').write_text('ignored.yaml\n', encoding='utf-8')
    (repo / 'ignored.yaml').write_text('ignored: true\n', encoding='utf-8')
    _git(repo, 'add', '.gitignore')
    _git(repo, 'commit', '-m', 'ignore external asset')

    with pytest.raises(ValueError, match='ls-files failed'):
        freeze_protocol(protocol_path, repo)
