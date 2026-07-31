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

import pytest

from semantic_planning_experiments.report_summary import summarize_reports


def _report(planning_time: float = 0.1) -> dict:
    metrics = {
        'path_length_m': 10.0,
        'semantic_crossing_length_m': 0.0,
        'semantic_crossing_ratio': 0.0,
        'minimum_semantic_clearance_m': 1.0,
        'pose_count': 3,
        'planning_time_s': planning_time,
    }
    return {
        'schema_version': 1,
        'created_at_utc': '2026-01-01T00:00:00+00:00',
        'code_revision': 'abc123',
        'inputs': {
            'map_yaml_sha256': 'map',
            'mask_yaml_sha256': None,
            'mask_image_sha256': None,
            'planner_config_sha256': 'planner',
            'mask_producer_config_sha256': 'producer',
            'mask_producer_runtime_parameters': {
                'detection_active_duration_sec': 0.0,
            },
            'planner_id': 'GridBased',
            'mask_source': 'topic',
            'topic_input_mode': 'external',
            'mask_topic_qos': {'reliability': 'reliable'},
            'semantic_layer_parameters': {
                'cost_mode': 'fuzzy',
                'task_urgency': 0,
                'avoidance_level': 70.0,
            },
            'recovery_timeout_seconds': 0.0,
            'start': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'goal': {'x': 2.0, 'y': 0.0, 'yaw': 0.0},
        },
        'baseline': {
            'metrics': copy.deepcopy(metrics),
            'path': [{'x': 0.0, 'y': 0.0}, {'x': 2.0, 'y': 0.0}],
        },
        'semantic': {
            'metrics': copy.deepcopy(metrics),
            'path': [
                {'x': 0.0, 'y': 0.0},
                {'x': 1.0, 'y': 1.0},
                {'x': 2.0, 'y': 0.0},
            ],
        },
        'recovered_after_mask_clear': None,
    }


def _write_report(path: Path, report: dict) -> Path:
    path.write_text(json.dumps(report), encoding='utf-8')
    return path


def test_summary_aggregates_metrics_and_exact_path_hashes(tmp_path: Path):
    first = _write_report(tmp_path / 'first.json', _report(0.1))
    second = _write_report(tmp_path / 'second.json', _report(0.3))

    summary = summarize_reports([first, second])

    planning = summary['conditions']['semantic']['metrics']['planning_time_s']
    assert summary['trial_count'] == 2
    assert summary['all_path_geometries_repeatable']
    assert planning['mean'] == pytest.approx(0.2)
    assert planning['population_stddev'] == pytest.approx(0.1)
    assert (
        summary['conditions']['semantic']['unique_path_hash_count'] == 1
    )


def test_summary_rejects_dirty_revisions(tmp_path: Path):
    report = _report()
    report['code_revision'] = 'abc123-dirty'
    first = _write_report(tmp_path / 'first.json', report)
    second = _write_report(tmp_path / 'second.json', report)

    with pytest.raises(ValueError, match='dirty code revision'):
        summarize_reports([first, second])


def test_summary_rejects_incomparable_inputs(tmp_path: Path):
    first_report = _report()
    second_report = _report()
    second_report['inputs']['goal']['x'] = 3.0
    first = _write_report(tmp_path / 'first.json', first_report)
    second = _write_report(tmp_path / 'second.json', second_report)

    with pytest.raises(ValueError, match='not comparable'):
        summarize_reports([first, second])


def test_summary_rejects_mixed_cost_modes(tmp_path: Path):
    first_report = _report()
    second_report = _report()
    second_report['inputs']['semantic_layer_parameters']['cost_mode'] = 'fixed'
    first = _write_report(tmp_path / 'first.json', first_report)
    second = _write_report(tmp_path / 'second.json', second_report)

    with pytest.raises(ValueError, match='not comparable'):
        summarize_reports([first, second])


def test_summary_marks_changed_path_geometry(tmp_path: Path):
    first_report = _report()
    second_report = _report()
    second_report['semantic']['path'][1]['y'] = 1.5
    first = _write_report(tmp_path / 'first.json', first_report)
    second = _write_report(tmp_path / 'second.json', second_report)

    summary = summarize_reports([first, second])

    assert not summary['all_path_geometries_repeatable']
    assert not summary['conditions']['semantic']['path_geometry_repeatable']
    assert summary['conditions']['semantic']['unique_path_hash_count'] == 2
