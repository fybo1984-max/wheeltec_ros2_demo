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

"""Export repeated planning summaries as paper-ready audited artifacts."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
from html import escape
import io
import json
import math
import os
from pathlib import Path
import statistics
import sys

from semantic_planning_experiments.metrics import sha256_file


_METRICS = (
    ('path_length_m', 'Path length', 'm'),
    ('semantic_crossing_length_m', 'Risk crossing', 'm'),
    ('minimum_semantic_clearance_m', 'Minimum clearance', 'm'),
    ('planning_time_s', 'Planning time', 's'),
)
_DEFAULT_VARYING_INPUTS = {
    'mask_value_summary',
    'mask_producer_runtime_parameters',
}
_EXPLICIT_VARYING_INPUTS = {
    'semantic_layer_parameters.cost_mode',
    'semantic_layer_parameters.task_urgency',
    'semantic_layer_parameters.avoidance_level',
    'start',
    'goal',
}
_COLORS = (
    '#2563eb',
    '#d97706',
    '#059669',
    '#7c3aed',
    '#dc2626',
    '#0891b2',
    '#4b5563',
    '#be185d',
)


def _canonical_sha256(value) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _read_summary(path: Path) -> dict:
    resolved = path.expanduser().resolve()
    summary = json.loads(resolved.read_text(encoding='utf-8'))
    if not isinstance(summary, dict):
        raise ValueError(f'summary must contain a JSON object: {resolved}')
    required = (
        'schema_version',
        'trial_count',
        'code_revision',
        'comparison_fingerprint_sha256',
        'comparison_inputs',
        'all_path_geometries_repeatable',
        'conditions',
        'input_reports',
    )
    missing = [name for name in required if name not in summary]
    if missing:
        raise ValueError(f'summary is missing {missing}: {resolved}')
    if summary['schema_version'] != 1:
        raise ValueError(f'unsupported summary schema: {resolved}')
    trial_count = summary['trial_count']
    if not isinstance(trial_count, int) or trial_count < 1:
        raise ValueError(f'invalid trial count: {resolved}')
    revision = str(summary['code_revision'])
    if not revision or revision == 'unknown' or revision.endswith('-dirty'):
        raise ValueError(f'clean code revision is required: {resolved}')
    if revision != summary['comparison_inputs'].get('code_revision'):
        raise ValueError(f'inconsistent code revision: {resolved}')
    if not summary['all_path_geometries_repeatable']:
        raise ValueError(f'non-repeatable path geometry: {resolved}')
    for condition_name in ('baseline', 'semantic'):
        condition = summary['conditions'].get(condition_name)
        if not condition or not condition.get('path_geometry_repeatable'):
            raise ValueError(
                f'non-repeatable {condition_name} path geometry: {resolved}'
            )
    fingerprint = _canonical_sha256(summary['comparison_inputs'])
    if fingerprint != summary['comparison_fingerprint_sha256']:
        raise ValueError(f'invalid comparison fingerprint: {resolved}')
    _verify_input_reports(summary, resolved)
    return summary


def _verify_input_reports(summary: dict, summary_path: Path) -> None:
    reports = summary['input_reports']
    if len(reports) != summary['trial_count']:
        raise ValueError(
            f'trial count does not match input reports: {summary_path}'
        )
    for report in reports:
        path = Path(report['path']).expanduser().resolve()
        if not path.is_file():
            raise ValueError(f'input report is missing: {path}')
        if sha256_file(path) != report['sha256']:
            raise ValueError(f'input report checksum mismatch: {path}')


def _metric(summary: dict, condition: str, name: str, path: Path) -> dict:
    try:
        metric = summary['conditions'][condition]['metrics'][name]
        mean = float(metric['mean'])
        stddev = float(metric['population_stddev'])
        count = int(metric['count'])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f'invalid {condition} metric {name}: {path}'
        ) from error
    if count != summary['trial_count']:
        raise ValueError(
            f'metric count does not match trials for {name}: {path}'
        )
    if not math.isfinite(mean) or not math.isfinite(stddev) or stddev < 0.0:
        raise ValueError(f'non-finite semantic metric {name}: {path}')
    return {'mean': mean, 'population_stddev': stddev}


def _path_value(value: dict, dotted_path: str):
    current = value
    for name in dotted_path.split('.'):
        if not isinstance(current, dict) or name not in current:
            raise ValueError(f'varying input does not exist: {dotted_path}')
        current = current[name]
    return current


def _fixed_inputs(summary: dict, varying_inputs: set[str]) -> dict:
    fixed = json.loads(json.dumps(summary['comparison_inputs']))
    for dotted_path in varying_inputs:
        names = dotted_path.split('.')
        parent = fixed
        for name in names[:-1]:
            parent = parent[name]
        del parent[names[-1]]
    return fixed


def _paired_delta(summary: dict, name: str, summary_path: Path) -> dict:
    values = []
    for reference in summary['input_reports']:
        report_path = Path(reference['path']).expanduser().resolve()
        report = json.loads(report_path.read_text(encoding='utf-8'))
        try:
            baseline = float(report['baseline']['metrics'][name])
            semantic = float(report['semantic']['metrics'][name])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f'invalid paired metric {name}: {report_path}'
            ) from error
        if not math.isfinite(baseline) or not math.isfinite(semantic):
            raise ValueError(f'non-finite paired metric {name}: {report_path}')
        values.append(semantic - baseline)
    if len(values) != summary['trial_count']:
        raise ValueError(f'paired metric count mismatch: {summary_path}')
    return {
        'count': len(values),
        'mean': statistics.fmean(values),
        'population_stddev': statistics.pstdev(values),
    }


def _scenario_metrics(summary: dict, path: Path) -> dict:
    metrics = {}
    for name, _, _ in _METRICS:
        baseline = _metric(summary, 'baseline', name, path)
        semantic = _metric(summary, 'semantic', name, path)
        delta = _paired_delta(summary, name, path)
        if not math.isclose(
            delta['mean'],
            semantic['mean'] - baseline['mean'],
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            raise ValueError(
                f'paired delta is inconsistent for {name}: {path}'
            )
        metrics[name] = {
            'baseline': baseline,
            'semantic': semantic,
            'semantic_minus_baseline': delta,
        }
    return metrics


def prepare_export(
    study_title: str,
    scenarios: list[tuple[str, Path]],
    varying_inputs: set[str] | None = None,
) -> dict:
    """Validate summaries and return the normalized export dataset."""
    if not study_title.strip():
        raise ValueError('study title must not be empty')
    if len(scenarios) < 2:
        raise ValueError('at least two scenarios are required')
    labels = [label.strip() for label, _ in scenarios]
    if any(not label for label in labels):
        raise ValueError('scenario labels must not be empty')
    if len(set(labels)) != len(labels):
        raise ValueError('scenario labels must be unique')
    explicit_varying = set(varying_inputs or ())
    unsupported = explicit_varying - _EXPLICIT_VARYING_INPUTS
    if unsupported:
        raise ValueError(f'unsupported varying inputs: {sorted(unsupported)}')
    declared_varying = _DEFAULT_VARYING_INPUTS | explicit_varying

    loaded = []
    common_inputs = None
    for label, path in zip(labels, (item[1] for item in scenarios)):
        resolved = path.expanduser().resolve()
        summary = _read_summary(resolved)
        scenario_varying = {
            name: _path_value(summary['comparison_inputs'], name)
            for name in sorted(declared_varying)
        }
        fixed_inputs = _fixed_inputs(summary, declared_varying)
        if common_inputs is None:
            common_inputs = fixed_inputs
        elif fixed_inputs != common_inputs:
            raise ValueError(f'scenario inputs are not comparable: {resolved}')
        loaded.append({
            'label': label,
            'trial_count': int(summary['trial_count']),
            'summary_path': str(resolved),
            'summary_sha256': sha256_file(resolved),
            'comparison_fingerprint_sha256': (
                summary['comparison_fingerprint_sha256']
            ),
            'varying_inputs': scenario_varying,
            'metrics': _scenario_metrics(summary, resolved),
        })

    assert common_inputs is not None
    for name in explicit_varying:
        values = {
            _canonical_sha256(item['varying_inputs'][name])
            for item in loaded
        }
        if len(values) < 2:
            raise ValueError(f'declared input does not vary: {name}')
    return {
        'study_title': study_title.strip(),
        'code_revision': common_inputs['code_revision'],
        'common_inputs': common_inputs,
        'declared_varying_inputs': sorted(declared_varying),
        'study_fingerprint_sha256': _canonical_sha256({
            'common_inputs': common_inputs,
            'scenarios': [
                {
                    'label': item['label'],
                    'summary_sha256': item['summary_sha256'],
                }
                for item in loaded
            ],
        }),
        'scenarios': loaded,
    }


def render_csv(dataset: dict) -> str:
    """Render the normalized dataset as deterministic CSV text."""
    stream = io.StringIO(newline='')
    fieldnames = ['scenario', 'trial_count', 'code_revision']
    for name, _, _ in _METRICS:
        for condition in (
            'baseline',
            'semantic',
            'semantic_minus_baseline',
        ):
            fieldnames.extend((
                f'{condition}_{name}_mean',
                f'{condition}_{name}_population_stddev',
            ))
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator='\n')
    writer.writeheader()
    for scenario in dataset['scenarios']:
        row = {
            'scenario': scenario['label'],
            'trial_count': scenario['trial_count'],
            'code_revision': dataset['code_revision'],
        }
        for name, _, _ in _METRICS:
            for condition in (
                'baseline',
                'semantic',
                'semantic_minus_baseline',
            ):
                metric = scenario['metrics'][name][condition]
                row[f'{condition}_{name}_mean'] = format(
                    metric['mean'], '.9g'
                )
                row[
                    f'{condition}_{name}_population_stddev'
                ] = format(metric['population_stddev'], '.9g')
        writer.writerow(row)
    return stream.getvalue()


def _svg_panel(dataset: dict, metric, x: int, y: int, width: int) -> list[str]:
    name, title, unit = metric
    scenarios = dataset['scenarios']
    label_width = 165
    value_width = 122
    plot_width = width - label_width - value_width - 28
    maximum = max(
        item['metrics'][name]['semantic']['mean']
        + item['metrics'][name]['semantic']['population_stddev']
        for item in scenarios
    )
    if maximum <= 0.0:
        maximum = 1.0
    lines = [
        f'<g transform="translate({x},{y})">',
        f'<rect class="panel" width="{width}" '
        f'height="{74 + 36 * len(scenarios)}"/>',
        f'<text class="panel-title" x="16" y="25">'
        f'{escape(title)} ({unit})</text>',
    ]
    for index, scenario in enumerate(scenarios):
        row_y = 49 + index * 36
        metric_values = scenario['metrics'][name]['semantic']
        mean = metric_values['mean']
        stddev = metric_values['population_stddev']
        bar_width = mean / maximum * plot_width
        whisker_start = max(0.0, mean - stddev) / maximum * plot_width
        whisker_end = (mean + stddev) / maximum * plot_width
        color = _COLORS[index % len(_COLORS)]
        lines.extend((
            f'<text class="label" x="16" y="{row_y + 12}">'
            f'{escape(scenario["label"])}</text>',
            f'<rect x="{label_width}" y="{row_y}" width="{bar_width:.2f}" '
            f'height="16" rx="2" fill="{color}"/>',
            f'<line class="whisker" x1="{label_width + whisker_start:.2f}" '
            f'y1="{row_y + 8}" x2="{label_width + whisker_end:.2f}" '
            f'y2="{row_y + 8}"/>',
            f'<line class="whisker" x1="{label_width + whisker_end:.2f}" '
            f'y1="{row_y + 3}" x2="{label_width + whisker_end:.2f}" '
            f'y2="{row_y + 13}"/>',
            f'<text class="value" x="{width - value_width}" '
            f'y="{row_y + 12}">{mean:.4f} ± {stddev:.4f}</text>',
        ))
    lines.append('</g>')
    return lines


def render_svg(dataset: dict) -> str:
    """Render a deterministic four-metric SVG figure."""
    panel_width = 570
    panel_height = 74 + 36 * len(dataset['scenarios'])
    width = 1180
    height = 88 + panel_height * 2 + 24
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}" role="img">',
        f'<title>{escape(dataset["study_title"])}</title>',
        '<desc>Mean and population standard deviation for four semantic '
        'planning metrics. Error whiskers show plus or minus one population '
        'standard deviation.</desc>',
        '<style>',
        'text { font-family: DejaVu Sans, Arial, sans-serif; fill: #111827; }',
        '.title { font-size: 22px; font-weight: 700; }',
        '.subtitle { font-size: 12px; fill: #4b5563; }',
        '.panel { fill: #ffffff; stroke: #d1d5db; }',
        '.panel-title { font-size: 15px; font-weight: 700; }',
        '.label { font-size: 12px; }',
        '.value { font-size: 11px; font-variant-numeric: tabular-nums; }',
        '.whisker { stroke: #111827; stroke-width: 1.25; }',
        '</style>',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text class="title" x="20" y="30">'
        f'{escape(dataset["study_title"])}</text>',
        f'<text class="subtitle" x="20" y="51">revision '
        f'{escape(str(dataset["code_revision"]))}; semantic values are '
        'mean ± population standard deviation</text>',
    ]
    positions = (
        (20, 68),
        (590, 68),
        (20, 80 + panel_height),
        (590, 80 + panel_height),
    )
    for metric, (x, y) in zip(_METRICS, positions):
        lines.extend(_svg_panel(dataset, metric, x, y, panel_width))
    lines.extend(('</svg>', ''))
    return '\n'.join(lines)


def _write_text(path: Path, content: str) -> Path:
    resolved = path.expanduser().resolve()
    resolved.parent.mkdir(parents=True, exist_ok=True)
    temporary = resolved.with_name(resolved.name + '.tmp')
    temporary.write_text(content, encoding='utf-8')
    os.replace(temporary, resolved)
    return resolved


def write_export(
    dataset: dict,
    csv_path: Path,
    svg_path: Path,
    manifest_path: Path,
) -> dict:
    """Write CSV, SVG, and a JSON audit manifest atomically per file."""
    resolved_csv = csv_path.expanduser().resolve()
    resolved_svg = svg_path.expanduser().resolve()
    resolved_manifest = manifest_path.expanduser().resolve()
    output_paths = {resolved_csv, resolved_svg, resolved_manifest}
    if len(output_paths) != 3:
        raise ValueError('CSV, SVG, and manifest outputs must be distinct')
    summary_paths = {
        Path(item['summary_path'])
        for item in dataset['scenarios']
    }
    if output_paths & summary_paths:
        raise ValueError('output paths must not overwrite input summaries')
    resolved_csv = _write_text(resolved_csv, render_csv(dataset))
    resolved_svg = _write_text(resolved_svg, render_svg(dataset))
    manifest = {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        **dataset,
        'outputs': {
            'csv': {
                'path': str(resolved_csv),
                'sha256': sha256_file(resolved_csv),
            },
            'svg': {
                'path': str(resolved_svg),
                'sha256': sha256_file(resolved_svg),
            },
        },
    }
    resolved_manifest = _write_text(
        resolved_manifest,
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
    )
    return {
        'csv': resolved_csv,
        'svg': resolved_svg,
        'manifest': resolved_manifest,
    }


def _parse_scenario(value: str) -> tuple[str, Path]:
    if '=' not in value:
        raise argparse.ArgumentTypeError('expected LABEL=SUMMARY.json')
    label, path = value.split('=', 1)
    if not label.strip() or not path.strip():
        raise argparse.ArgumentTypeError(
            'expected non-empty LABEL=SUMMARY.json'
        )
    return label.strip(), Path(path)


def main(args=None) -> None:
    """Run the audited paper artifact exporter."""
    parser = argparse.ArgumentParser(
        description='Export comparable semantic planning summaries.',
    )
    parser.add_argument('--study-title', required=True)
    parser.add_argument(
        '--scenario',
        action='append',
        required=True,
        type=_parse_scenario,
        help='Scenario label and summary in LABEL=SUMMARY.json form.',
    )
    parser.add_argument(
        '--vary-input',
        action='append',
        choices=sorted(_EXPLICIT_VARYING_INPUTS),
        help='Explicit experimental factor allowed to differ.',
    )
    parser.add_argument('--csv-output', required=True, type=Path)
    parser.add_argument('--svg-output', required=True, type=Path)
    parser.add_argument('--manifest-output', required=True, type=Path)
    parsed = parser.parse_args(args)
    try:
        dataset = prepare_export(
            parsed.study_title,
            parsed.scenario,
            set(parsed.vary_input or ()),
        )
        outputs = write_export(
            dataset,
            parsed.csv_output,
            parsed.svg_output,
            parsed.manifest_output,
        )
    except (
        OSError,
        KeyError,
        TypeError,
        ValueError,
        json.JSONDecodeError,
    ) as error:
        print(f'cannot export paper artifacts: {error}', file=sys.stderr)
        raise SystemExit(2)
    print(
        'exported %d scenarios; csv=%s; svg=%s; manifest=%s'
        % (
            len(dataset['scenarios']),
            outputs['csv'],
            outputs['svg'],
            outputs['manifest'],
        )
    )


if __name__ == '__main__':
    main()
