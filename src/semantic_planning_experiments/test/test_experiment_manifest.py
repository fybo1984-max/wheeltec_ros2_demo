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
from pathlib import Path

import pytest
import yaml

from semantic_planning_experiments.experiment_manifest import (
    build_trial_plan,
    load_manifest,
)


def _manifest() -> dict:
    return {
        'schema_version': 1,
        'study_id': 'paper_pilot',
        'output_root': '/tmp/paper_pilot',
        'domain_id_start': 90,
        'defaults': {
            'package': 'semantic_planning_experiments',
            'launch_file': 'generator_planner_ab.launch.py',
            'repetitions': 2,
            'parameters': {
                'cost_mode': 'fuzzy',
                'start_x': 6.0,
                'goal_x': 20.0,
            },
        },
        'scenarios': [{
            'id': 'center_safety',
            'research_question': 'RQ1',
            'hypothesis': 'Avoid the center risk.',
            'factors': {'policy': 'safety'},
            'parameters': {
                'task_urgency': 0,
                'avoidance_level': 100.0,
            },
            'acceptance': [{
                'metric': 'semantic.semantic_crossing_length_m',
                'operator': 'le',
                'value': 0.05,
            }],
        }],
    }


def _write_manifest(path: Path, manifest: dict) -> Path:
    path.write_text(
        yaml.safe_dump(manifest, sort_keys=False),
        encoding='utf-8',
    )
    return path


def test_manifest_expands_deterministic_safe_trial_argv(tmp_path: Path):
    path = _write_manifest(tmp_path / 'manifest.yaml', _manifest())
    manifest = load_manifest(path)

    plan = build_trial_plan(manifest)

    assert plan['trial_count'] == 2
    assert plan['safety_scope'] == {
        'commands_executed': False,
        'controller_launch_allowed': False,
        'hardware_launch_allowed': False,
        'velocity_commands_allowed': False,
    }
    assert [trial['ros_domain_id'] for trial in plan['trials']] == [90, 91]
    first = plan['trials'][0]
    assert first['report_path'] == (
        '/tmp/paper_pilot/center_safety/trial_001.json'
    )
    assert first['command_argv'][:5] == [
        'ros2',
        'launch',
        'semantic_planning_experiments',
        'generator_planner_ab.launch.py',
        'avoidance_level:=100.0',
    ]
    assert 'domain_id:=90' in first['command_argv']
    assert 'cost_mode:=fuzzy' in first['command_argv']
    assert 'output_path:=/tmp/paper_pilot/center_safety/trial_001.json' in (
        first['command_argv']
    )


def test_manifest_rejects_duplicate_scenario_ids(tmp_path: Path):
    manifest = _manifest()
    manifest['scenarios'].append(copy.deepcopy(manifest['scenarios'][0]))
    path = _write_manifest(tmp_path / 'manifest.yaml', manifest)

    with pytest.raises(ValueError, match='duplicate scenario'):
        load_manifest(path)


def test_manifest_rejects_reserved_launch_parameters(tmp_path: Path):
    manifest = _manifest()
    manifest['scenarios'][0]['parameters']['output_path'] = '/tmp/overwrite'
    path = _write_manifest(tmp_path / 'manifest.yaml', manifest)

    with pytest.raises(ValueError, match='cannot override output_path'):
        load_manifest(path)


def test_manifest_rejects_ros_domain_overflow(tmp_path: Path):
    manifest = _manifest()
    manifest['domain_id_start'] = 232
    path = _write_manifest(tmp_path / 'manifest.yaml', manifest)

    with pytest.raises(ValueError, match='domains exceed'):
        load_manifest(path)


def test_manifest_rejects_invalid_cost_mode(tmp_path: Path):
    manifest = _manifest()
    manifest['defaults']['parameters']['cost_mode'] = 'unsupported'
    path = _write_manifest(tmp_path / 'manifest.yaml', manifest)

    with pytest.raises(ValueError, match='cost_mode'):
        load_manifest(path)


@pytest.mark.parametrize(
    'name, value',
    [
        ('synthetic_depth_invalid_fraction', 1.0),
        ('synthetic_person_count', 0),
        ('synthetic_detection_publish_every_n_frames', 0),
        ('risk_radius_scale', 0.0),
    ],
)
def test_manifest_rejects_invalid_synthetic_injection(
    tmp_path: Path,
    name: str,
    value,
):
    manifest = _manifest()
    manifest['defaults']['parameters'][name] = value
    path = _write_manifest(tmp_path / 'manifest.yaml', manifest)

    with pytest.raises(ValueError, match=name):
        load_manifest(path)
