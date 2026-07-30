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

import pytest

from semantic_planning_experiments.experiment_evaluate import (
    evaluate_plan,
)


def _report() -> dict:
    return {
        'schema_version': 1,
        'code_revision': '0123456789abcdef',
        'safety_scope': {
            'controller_started': False,
            'velocity_commands_published': False,
            'hardware_nodes_started': False,
        },
        'baseline': {
            'metrics': {
                'path_length_m': 14.0,
            },
        },
        'semantic': {
            'metrics': {
                'path_length_m': 17.4,
                'semantic_crossing_length_m': 0.0,
                'minimum_semantic_clearance_m': 1.1,
            },
        },
        'recovered_after_mask_clear': None,
        'delta_semantic_minus_baseline': {
            'path_length_m': 3.4,
        },
        'delta_recovered_minus_baseline': None,
    }


def _plan(report_paths: list[Path]) -> dict:
    trials = []
    for index, report_path in enumerate(report_paths, start=1):
        trials.append({
            'trial_id': f'safety__r{index:03d}',
            'scenario_id': 'safety',
            'repetition': index,
            'report_path': str(report_path),
            'acceptance': [
                {
                    'metric': 'semantic.semantic_crossing_length_m',
                    'operator': 'le',
                    'value': 0.05,
                },
                {
                    'metric': 'semantic.minimum_semantic_clearance_m',
                    'operator': 'ge',
                    'value': 1.0,
                },
                {
                    'metric': 'delta_semantic_minus_baseline.path_length_m',
                    'operator': 'le',
                    'value': 4.5,
                },
            ],
        })
    return {
        'schema_version': 1,
        'study_id': 'paper_pilot',
        'plan_fingerprint_sha256': 'plan-fingerprint',
        'safety_scope': {
            'commands_executed': False,
            'controller_launch_allowed': False,
            'hardware_launch_allowed': False,
            'velocity_commands_allowed': False,
        },
        'trial_count': len(trials),
        'trials': trials,
    }


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value), encoding='utf-8')


def test_evaluation_passes_all_numeric_criteria(tmp_path: Path):
    report_path = tmp_path / 'report.json'
    _write_json(report_path, _report())

    evaluation = evaluate_plan(_plan([report_path]))

    assert evaluation['all_passed'] is True
    assert evaluation['status_counts'] == {
        'passed': 1,
        'failed': 0,
        'missing': 0,
        'invalid': 0,
    }
    assert evaluation['trials'][0]['status'] == 'passed'
    assert all(
        criterion['passed']
        for criterion in evaluation['trials'][0]['criteria']
    )


def test_evaluation_surfaces_failed_and_missing_trials(tmp_path: Path):
    failed_path = tmp_path / 'failed.json'
    report = _report()
    report['semantic']['metrics']['semantic_crossing_length_m'] = 2.0
    _write_json(failed_path, report)

    evaluation = evaluate_plan(
        _plan([failed_path, tmp_path / 'missing.json'])
    )

    assert evaluation['all_passed'] is False
    assert evaluation['status_counts'] == {
        'passed': 0,
        'failed': 1,
        'missing': 1,
        'invalid': 0,
    }
    failed = evaluation['trials'][0]
    assert failed['criteria'][0] == {
        'metric': 'semantic.semantic_crossing_length_m',
        'operator': 'le',
        'target': 0.05,
        'actual': 2.0,
        'passed': False,
    }
    assert evaluation['trials'][1]['reason'] == (
        'report file does not exist'
    )


@pytest.mark.parametrize('revision', ['unknown', 'abc123-dirty'])
def test_evaluation_rejects_unpublishable_revision(
    tmp_path: Path,
    revision: str,
):
    report_path = tmp_path / 'report.json'
    report = _report()
    report['code_revision'] = revision
    _write_json(report_path, report)

    evaluation = evaluate_plan(_plan([report_path]))

    assert evaluation['status_counts']['invalid'] == 1
    assert revision in evaluation['trials'][0]['reason']


def test_evaluation_rejects_unsafe_plan(tmp_path: Path):
    plan = _plan([tmp_path / 'report.json'])
    plan['safety_scope']['controller_launch_allowed'] = True

    with pytest.raises(ValueError, match='safety_scope'):
        evaluate_plan(plan)
