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
import csv
import hashlib
import io
import json
from pathlib import Path

import pytest

from semantic_planning_experiments.paper_export import prepare_export
from semantic_planning_experiments.paper_export import write_export


_METRIC_NAMES = (
    'path_length_m',
    'semantic_crossing_length_m',
    'minimum_semantic_clearance_m',
    'planning_time_s',
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _canonical_sha256(value) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(',', ':'),
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


def _summary(
    tmp_path: Path,
    name: str,
    path_length: float = 10.0,
    cost_mode: str = 'fuzzy',
) -> Path:
    comparison_inputs = {
        'schema_version': 1,
        'code_revision': 'abc123',
        'map_yaml_sha256': 'map',
        'planner_config_sha256': 'planner',
        'mask_value_summary': {'maximum': 100},
        'mask_producer_runtime_parameters': {'risk_radius_scale': 1.0},
        'planner_id': 'GridBased',
        'semantic_layer_parameters': {'cost_mode': cost_mode},
        'start': {'x': 0.0, 'y': 0.0},
        'goal': {'x': 2.0, 'y': 0.0},
    }
    baseline_values = {
        'path_length_m': 8.0,
        'semantic_crossing_length_m': 0.5,
        'minimum_semantic_clearance_m': 0.0,
        'planning_time_s': 0.05,
    }
    semantic_values = {
        'path_length_m': path_length,
        'semantic_crossing_length_m': 0.1,
        'minimum_semantic_clearance_m': 0.2,
        'planning_time_s': 0.1,
    }

    def summarized(values: dict) -> dict:
        return {
            metric: {
                'count': 1,
                'mean': values[metric],
                'population_stddev': 0.0,
                'minimum': values[metric],
                'maximum': values[metric],
            }
            for metric in _METRIC_NAMES
        }

    report_path = tmp_path / f'{name}_trial.json'
    report_path.write_text(json.dumps({
        'baseline': {'metrics': baseline_values},
        'semantic': {'metrics': semantic_values},
    }), encoding='utf-8')
    conditions = {
        name: {
            'path_geometry_repeatable': True,
            'metrics': summarized(values),
        }
        for name, values in (
            ('baseline', baseline_values),
            ('semantic', semantic_values),
        )
    }
    summary = {
        'schema_version': 1,
        'trial_count': 1,
        'code_revision': 'abc123',
        'comparison_fingerprint_sha256': _canonical_sha256(
            comparison_inputs
        ),
        'comparison_inputs': comparison_inputs,
        'all_path_geometries_repeatable': True,
        'conditions': conditions,
        'input_reports': [{
            'path': str(report_path),
            'sha256': _sha256(report_path),
        }],
    }
    path = tmp_path / f'{name}_summary.json'
    path.write_text(json.dumps(summary), encoding='utf-8')
    return path


def _rewrite(path: Path, summary: dict) -> None:
    summary['comparison_fingerprint_sha256'] = _canonical_sha256(
        summary['comparison_inputs']
    )
    path.write_text(json.dumps(summary), encoding='utf-8')


def test_export_writes_csv_svg_and_audit_manifest(tmp_path: Path):
    first = _summary(tmp_path, 'first', 10.0)
    second = _summary(tmp_path, 'second', 12.0)
    dataset = prepare_export(
        'Paper pilot',
        [('Near', first), ('Far & fragile', second)],
    )

    outputs = write_export(
        dataset,
        tmp_path / 'table.csv',
        tmp_path / 'figure.svg',
        tmp_path / 'audit.json',
    )
    manifest = json.loads(outputs['manifest'].read_text())
    rows = list(csv.DictReader(io.StringIO(outputs['csv'].read_text())))
    svg_text = outputs['svg'].read_text()

    assert rows[0]['baseline_path_length_m_mean'] == '8'
    assert rows[0]['semantic_path_length_m_mean'] == '10'
    assert rows[0]['semantic_minus_baseline_path_length_m_mean'] == '2'
    assert 'Far &amp; fragile' in svg_text
    assert manifest['code_revision'] == 'abc123'
    assert manifest['outputs']['csv']['sha256'] == _sha256(outputs['csv'])
    assert manifest['outputs']['svg']['sha256'] == _sha256(outputs['svg'])
    assert len(manifest['scenarios']) == 2


def test_export_allows_declared_scenario_varying_inputs(tmp_path: Path):
    first = _summary(tmp_path, 'first')
    second = _summary(tmp_path, 'second')
    summary = json.loads(second.read_text())
    summary['comparison_inputs']['mask_value_summary']['maximum'] = 65
    runtime = summary['comparison_inputs']['mask_producer_runtime_parameters']
    runtime['risk_radius_scale'] = 1.8
    _rewrite(second, summary)

    dataset = prepare_export(
        'Ablation',
        [('Nominal', first), ('Risk', second)],
    )

    assert dataset['scenarios'][1]['varying_inputs'][
        'mask_value_summary'
    ]['maximum'] == 65


def test_export_rejects_dirty_revision(tmp_path: Path):
    first = _summary(tmp_path, 'first')
    second = _summary(tmp_path, 'second')
    summary = json.loads(second.read_text())
    summary['code_revision'] = 'abc123-dirty'
    summary['comparison_inputs']['code_revision'] = 'abc123-dirty'
    _rewrite(second, summary)

    with pytest.raises(ValueError, match='clean code revision'):
        prepare_export('Study', [('First', first), ('Second', second)])


def test_export_rejects_non_repeatable_path(tmp_path: Path):
    first = _summary(tmp_path, 'first')
    second = _summary(tmp_path, 'second')
    summary = json.loads(second.read_text())
    summary['all_path_geometries_repeatable'] = False
    second.write_text(json.dumps(summary), encoding='utf-8')

    with pytest.raises(ValueError, match='non-repeatable'):
        prepare_export('Study', [('First', first), ('Second', second)])


def test_export_rejects_incomparable_fixed_inputs(tmp_path: Path):
    first = _summary(tmp_path, 'first')
    second = _summary(tmp_path, 'second')
    summary = json.loads(second.read_text())
    summary['comparison_inputs']['goal']['x'] = 3.0
    _rewrite(second, summary)

    with pytest.raises(ValueError, match='not comparable'):
        prepare_export('Study', [('First', first), ('Second', second)])


def test_export_allows_explicit_method_factor_and_records_values(
    tmp_path: Path,
):
    first = _summary(tmp_path, 'first', cost_mode='fixed')
    second = _summary(tmp_path, 'second', cost_mode='fuzzy')

    dataset = prepare_export(
        'Method ablation',
        [('Fixed', first), ('Fuzzy', second)],
        {'semantic_layer_parameters.cost_mode'},
    )

    factor = 'semantic_layer_parameters.cost_mode'
    assert factor in dataset['declared_varying_inputs']
    assert dataset['scenarios'][0]['varying_inputs'][factor] == 'fixed'
    assert dataset['scenarios'][1]['varying_inputs'][factor] == 'fuzzy'


def test_export_rejects_undeclared_method_factor(tmp_path: Path):
    first = _summary(tmp_path, 'first', cost_mode='fixed')
    second = _summary(tmp_path, 'second', cost_mode='fuzzy')

    with pytest.raises(ValueError, match='not comparable'):
        prepare_export('Study', [('Fixed', first), ('Fuzzy', second)])


def test_export_rejects_declared_factor_that_does_not_vary(tmp_path: Path):
    first = _summary(tmp_path, 'first')
    second = _summary(tmp_path, 'second')

    with pytest.raises(ValueError, match='does not vary'):
        prepare_export(
            'Study',
            [('First', first), ('Second', second)],
            {'semantic_layer_parameters.cost_mode'},
        )


def test_export_rejects_changed_source_report(tmp_path: Path):
    first = _summary(tmp_path, 'first')
    second = _summary(tmp_path, 'second')
    summary = json.loads(second.read_text())
    report_path = Path(summary['input_reports'][0]['path'])
    report_path.write_text('{"trial": 2}\n', encoding='utf-8')

    with pytest.raises(ValueError, match='checksum mismatch'):
        prepare_export('Study', [('First', first), ('Second', second)])


def test_export_rejects_invalid_metric_count(tmp_path: Path):
    first = _summary(tmp_path, 'first')
    second = _summary(tmp_path, 'second')
    summary = copy.deepcopy(json.loads(second.read_text()))
    semantic = summary['conditions']['semantic']['metrics']
    semantic['path_length_m']['count'] = 2
    second.write_text(json.dumps(summary), encoding='utf-8')

    with pytest.raises(ValueError, match='metric count'):
        prepare_export('Study', [('First', first), ('Second', second)])


def test_export_rejects_colliding_output_paths(tmp_path: Path):
    first = _summary(tmp_path, 'first')
    second = _summary(tmp_path, 'second')
    dataset = prepare_export(
        'Study',
        [('First', first), ('Second', second)],
    )

    with pytest.raises(ValueError, match='outputs must be distinct'):
        write_export(
            dataset,
            tmp_path / 'same.file',
            tmp_path / 'same.file',
            tmp_path / 'audit.json',
        )
