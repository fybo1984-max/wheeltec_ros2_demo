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

"""Validate and summarize repeated semantic planning JSON reports."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys

from semantic_planning_experiments.metrics import sha256_file


_CONDITION_NAMES = (
    'baseline',
    'semantic',
    'recovered_after_mask_clear',
)
_METRIC_NAMES = (
    'path_length_m',
    'semantic_crossing_length_m',
    'semantic_crossing_ratio',
    'minimum_semantic_clearance_m',
    'pose_count',
    'planning_time_s',
)


def _canonical_sha256(value) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _comparison_inputs(report: dict) -> dict:
    inputs = report['inputs']
    return {
        'schema_version': report['schema_version'],
        'code_revision': report['code_revision'],
        'map_yaml_sha256': inputs['map_yaml_sha256'],
        'mask_yaml_sha256': inputs.get('mask_yaml_sha256'),
        'mask_image_sha256': inputs.get('mask_image_sha256'),
        'planner_config_sha256': inputs['planner_config_sha256'],
        'mask_producer_config_sha256': inputs.get(
            'mask_producer_config_sha256'
        ),
        'mask_producer_runtime_parameters': inputs.get(
            'mask_producer_runtime_parameters'
        ),
        'planner_id': inputs['planner_id'],
        'mask_source': inputs['mask_source'],
        'topic_input_mode': inputs.get('topic_input_mode'),
        'mask_topic_qos': inputs.get('mask_topic_qos'),
        'semantic_layer_parameters': inputs['semantic_layer_parameters'],
        'recovery_timeout_seconds': inputs.get(
            'recovery_timeout_seconds',
            0.0,
        ),
        'start': inputs['start'],
        'goal': inputs['goal'],
    }


def _load_report(path: Path) -> dict:
    path = path.expanduser().resolve()
    report = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(report, dict):
        raise ValueError(f'report must contain a JSON object: {path}')
    required = ('schema_version', 'code_revision', 'inputs', 'baseline', 'semantic')
    missing = [name for name in required if name not in report]
    if missing:
        raise ValueError(f'report is missing {missing}: {path}')
    if str(report['code_revision']).endswith('-dirty'):
        raise ValueError(f'dirty code revision is not comparable: {path}')
    return report


def _metric_summary(values: list) -> dict | None:
    if all(value is None for value in values):
        return None
    if any(value is None for value in values):
        raise ValueError('metric mixes numeric and null values')
    numeric = [float(value) for value in values]
    if any(not math.isfinite(value) for value in numeric):
        raise ValueError('metric contains a non-finite value')
    return {
        'count': len(numeric),
        'mean': statistics.fmean(numeric),
        'population_stddev': statistics.pstdev(numeric),
        'minimum': min(numeric),
        'maximum': max(numeric),
    }


def summarize_reports(paths: list[Path]) -> dict:
    """Return a strict statistical summary for comparable report files."""
    if len(paths) < 2:
        raise ValueError('at least two reports are required')
    resolved_paths = [path.expanduser().resolve() for path in paths]
    reports = [_load_report(path) for path in resolved_paths]
    comparison = _comparison_inputs(reports[0])
    for path, report in zip(resolved_paths[1:], reports[1:]):
        if _comparison_inputs(report) != comparison:
            raise ValueError(f'report inputs are not comparable: {path}')

    condition_names = [
        name for name in _CONDITION_NAMES
        if reports[0].get(name) is not None
    ]
    for path, report in zip(resolved_paths[1:], reports[1:]):
        available = [
            name for name in _CONDITION_NAMES
            if report.get(name) is not None
        ]
        if available != condition_names:
            raise ValueError(f'report conditions are not comparable: {path}')

    conditions = {}
    all_repeatable = True
    for condition_name in condition_names:
        path_hashes = [
            _canonical_sha256(report[condition_name]['path'])
            for report in reports
        ]
        unique_hashes = sorted(set(path_hashes))
        repeatable = len(unique_hashes) == 1
        all_repeatable = all_repeatable and repeatable
        conditions[condition_name] = {
            'path_geometry_repeatable': repeatable,
            'unique_path_hash_count': len(unique_hashes),
            'path_sha256': unique_hashes,
            'metrics': {
                metric_name: _metric_summary([
                    report[condition_name]['metrics'].get(metric_name)
                    for report in reports
                ])
                for metric_name in _METRIC_NAMES
            },
        }

    return {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'trial_count': len(reports),
        'code_revision': reports[0]['code_revision'],
        'comparison_fingerprint_sha256': _canonical_sha256(comparison),
        'comparison_inputs': comparison,
        'all_path_geometries_repeatable': all_repeatable,
        'conditions': conditions,
        'input_reports': [
            {
                'path': str(path),
                'sha256': sha256_file(path),
                'created_at_utc': report.get('created_at_utc'),
            }
            for path, report in zip(resolved_paths, reports)
        ],
    }


def write_summary(summary: dict, output_path: Path) -> Path:
    """Write a summary atomically and return its resolved path."""
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(output_path.name + '.tmp')
    temporary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary_path, output_path)
    return output_path


def main(args=None) -> None:
    """Run the repeated-report summarizer."""
    parser = argparse.ArgumentParser(
        description='Summarize comparable semantic planning reports.',
    )
    parser.add_argument(
        '--output',
        required=True,
        type=Path,
        help='Output JSON path.',
    )
    parser.add_argument(
        'reports',
        nargs='+',
        type=Path,
        help='Two or more controller-free experiment reports.',
    )
    parsed = parser.parse_args(args)
    try:
        summary = summarize_reports(parsed.reports)
        output_path = write_summary(summary, parsed.output)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        print(f'cannot summarize reports: {error}', file=sys.stderr)
        raise SystemExit(2)
    print(
        'summarized %d trials; paths_repeatable=%s; output=%s'
        % (
            summary['trial_count'],
            summary['all_path_geometries_repeatable'],
            output_path,
        )
    )


if __name__ == '__main__':
    main()
