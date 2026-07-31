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

"""Validate a paper experiment manifest and expand safe ROS launch trials."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys

import yaml

from semantic_planning_experiments.metrics import sha256_file


_ALLOWED_LAUNCH_FILES = {
    'planner_ab.launch.py',
    'generator_planner_ab.launch.py',
}
_ALLOWED_PACKAGE = 'semantic_planning_experiments'
_RESERVED_PARAMETERS = {'domain_id', 'output_path'}
_ID_PATTERN = re.compile(r'^[a-z0-9][a-z0-9_-]*$')
_PARAMETER_PATTERN = re.compile(r'^[a-z][a-z0-9_]*$')
_METRIC_PATTERN = re.compile(
    r'^(baseline|semantic|recovered_after_mask_clear|'
    r'delta_semantic_minus_baseline|delta_recovered_minus_baseline)'
    r'\.[a-z][a-z0-9_]*$'
)
_ACCEPTANCE_OPERATORS = {'eq', 'ge', 'gt', 'le', 'lt'}
_COST_MODES = {'fuzzy', 'fixed', 'lethal'}


def _canonical_sha256(value) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _require_mapping(value, description: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f'{description} must be a mapping')
    return value


def _require_scalar(value, description: str):
    if isinstance(value, bool) or isinstance(value, (int, float, str)):
        return value
    raise ValueError(f'{description} must be a boolean, number, or string')


def _validate_parameters(value, description: str) -> dict:
    parameters = _require_mapping(value, description)
    validated = {}
    for name, parameter_value in parameters.items():
        if not isinstance(name, str) or not _PARAMETER_PATTERN.fullmatch(name):
            raise ValueError(f'{description} has invalid parameter name: {name}')
        if name in _RESERVED_PARAMETERS:
            raise ValueError(f'{description} cannot override {name}')
        if name == 'cost_mode' and parameter_value not in _COST_MODES:
            raise ValueError(
                f"{description}.cost_mode must be 'fuzzy', 'fixed', or 'lethal'"
            )
        if name in {
            'synthetic_depth_m',
            'risk_radius_scale',
        } and (
            isinstance(parameter_value, bool)
            or not isinstance(parameter_value, (int, float))
            or not math.isfinite(parameter_value)
            or parameter_value <= 0.0
        ):
            raise ValueError(f'{description}.{name} must be positive')
        if name == 'synthetic_depth_noise_std_m' and (
            isinstance(parameter_value, bool)
            or not isinstance(parameter_value, (int, float))
            or not math.isfinite(parameter_value)
            or parameter_value < 0.0
        ):
            raise ValueError(
                f'{description}.{name} must be nonnegative'
            )
        if name in {
            'synthetic_depth_invalid_fraction',
            'synthetic_detection_score',
            'synthetic_detection_center_x_fraction',
            'synthetic_detection_center_y_fraction',
        } and (
            isinstance(parameter_value, bool)
            or not isinstance(parameter_value, (int, float))
            or not math.isfinite(parameter_value)
            or parameter_value < 0.0
            or parameter_value > 1.0
            or (
                name == 'synthetic_depth_invalid_fraction'
                and parameter_value == 1.0
            )
        ):
            raise ValueError(f'{description}.{name} is outside its range')
        if name == 'synthetic_person_spacing_y_fraction' and (
            isinstance(parameter_value, bool)
            or not isinstance(parameter_value, (int, float))
            or not math.isfinite(parameter_value)
            or parameter_value < 0.0
        ):
            raise ValueError(
                f'{description}.{name} must be nonnegative'
            )
        if name == 'synthetic_person_count' and (
            isinstance(parameter_value, bool)
            or not isinstance(parameter_value, int)
            or parameter_value < 1
            or parameter_value > 20
        ):
            raise ValueError(f'{description}.{name} must be in [1, 20]')
        if name == 'synthetic_detection_class_id' and (
            not isinstance(parameter_value, str)
            or not parameter_value
            or parameter_value != parameter_value.strip()
        ):
            raise ValueError(
                f'{description}.{name} must be a nonempty trimmed label'
            )
        if name == 'synthetic_detection_publish_every_n_frames' and (
            isinstance(parameter_value, bool)
            or not isinstance(parameter_value, int)
            or parameter_value < 1
        ):
            raise ValueError(f'{description}.{name} must be positive')
        if name == 'synthetic_random_seed' and (
            isinstance(parameter_value, bool)
            or not isinstance(parameter_value, int)
            or parameter_value < 0
        ):
            raise ValueError(
                f'{description}.{name} must be a nonnegative integer'
            )
        if name == 'synthetic_detection_timestamp_offset_sec' and (
            isinstance(parameter_value, bool)
            or not isinstance(parameter_value, (int, float))
            or not math.isfinite(parameter_value)
        ):
            raise ValueError(f'{description}.{name} must be finite')
        validated[name] = _require_scalar(
            parameter_value,
            f'{description}.{name}',
        )
    return validated


def _validate_acceptance(value, scenario_id: str) -> list[dict]:
    if not isinstance(value, list) or not value:
        raise ValueError(f'scenario {scenario_id} needs acceptance criteria')
    validated = []
    for index, criterion in enumerate(value):
        criterion = _require_mapping(
            criterion,
            f'scenario {scenario_id} acceptance[{index}]',
        )
        metric = criterion.get('metric')
        operator = criterion.get('operator')
        target = criterion.get('value')
        if not isinstance(metric, str) or not _METRIC_PATTERN.fullmatch(metric):
            raise ValueError(
                f'scenario {scenario_id} has invalid acceptance metric'
            )
        if operator not in _ACCEPTANCE_OPERATORS:
            raise ValueError(
                f'scenario {scenario_id} has invalid acceptance operator'
            )
        if (
            isinstance(target, bool)
            or not isinstance(target, (int, float))
        ):
            raise ValueError(
                f'scenario {scenario_id} acceptance value must be numeric'
            )
        validated.append({
            'metric': metric,
            'operator': operator,
            'value': float(target),
        })
    return validated


def _format_launch_value(value) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def load_manifest(path: Path) -> dict:
    """Load and validate a semantic planning paper manifest."""
    path = path.expanduser().resolve()
    manifest = yaml.safe_load(path.read_text(encoding='utf-8'))
    manifest = _require_mapping(manifest, 'manifest')
    if manifest.get('schema_version') != 1:
        raise ValueError('manifest schema_version must be 1')

    study_id = manifest.get('study_id')
    if not isinstance(study_id, str) or not _ID_PATTERN.fullmatch(study_id):
        raise ValueError('study_id must use lowercase letters, digits, _ or -')

    output_root = Path(str(manifest.get('output_root', '')))
    if not output_root.is_absolute():
        raise ValueError('output_root must be an absolute path outside the repo')

    domain_start = manifest.get('domain_id_start')
    if (
        isinstance(domain_start, bool)
        or not isinstance(domain_start, int)
        or domain_start < 0
        or domain_start > 232
    ):
        raise ValueError('domain_id_start must be an integer in [0, 232]')

    defaults = _require_mapping(manifest.get('defaults'), 'defaults')
    package = defaults.get('package')
    launch_file = defaults.get('launch_file')
    repetitions = defaults.get('repetitions')
    if package != _ALLOWED_PACKAGE:
        raise ValueError(f'defaults.package must be {_ALLOWED_PACKAGE}')
    if launch_file not in _ALLOWED_LAUNCH_FILES:
        raise ValueError('defaults.launch_file is not controller-free')
    if (
        isinstance(repetitions, bool)
        or not isinstance(repetitions, int)
        or repetitions < 1
        or repetitions > 100
    ):
        raise ValueError('defaults.repetitions must be in [1, 100]')
    default_parameters = _validate_parameters(
        defaults.get('parameters', {}),
        'defaults.parameters',
    )

    scenarios = manifest.get('scenarios')
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError('manifest needs at least one scenario')
    seen_ids = set()
    validated_scenarios = []
    total_trials = 0
    for index, raw_scenario in enumerate(scenarios):
        scenario = _require_mapping(raw_scenario, f'scenarios[{index}]')
        scenario_id = scenario.get('id')
        if (
            not isinstance(scenario_id, str)
            or not _ID_PATTERN.fullmatch(scenario_id)
        ):
            raise ValueError(f'scenarios[{index}].id is invalid')
        if scenario_id in seen_ids:
            raise ValueError(f'duplicate scenario id: {scenario_id}')
        seen_ids.add(scenario_id)

        research_question = scenario.get('research_question')
        if (
            not isinstance(research_question, str)
            or not re.fullmatch(r'RQ[1-9][0-9]*', research_question)
        ):
            raise ValueError(
                f'scenario {scenario_id} needs research_question RQ<number>'
            )
        factors = _require_mapping(
            scenario.get('factors'),
            f'scenario {scenario_id} factors',
        )
        validated_factors = {
            str(name): _require_scalar(
                factor_value,
                f'scenario {scenario_id} factor {name}',
            )
            for name, factor_value in factors.items()
        }
        parameters = _validate_parameters(
            scenario.get('parameters', {}),
            f'scenario {scenario_id} parameters',
        )
        scenario_repetitions = scenario.get('repetitions', repetitions)
        if (
            isinstance(scenario_repetitions, bool)
            or not isinstance(scenario_repetitions, int)
            or scenario_repetitions < 1
            or scenario_repetitions > 100
        ):
            raise ValueError(
                f'scenario {scenario_id} repetitions must be in [1, 100]'
            )
        total_trials += scenario_repetitions
        validated_scenarios.append({
            'id': scenario_id,
            'research_question': research_question,
            'hypothesis': str(scenario.get('hypothesis', '')).strip(),
            'factors': validated_factors,
            'parameters': parameters,
            'repetitions': scenario_repetitions,
            'acceptance': _validate_acceptance(
                scenario.get('acceptance'),
                scenario_id,
            ),
        })

    if domain_start + total_trials - 1 > 232:
        raise ValueError('trial domains exceed ROS domain 232')

    return {
        'schema_version': 1,
        'study_id': study_id,
        'description': str(manifest.get('description', '')).strip(),
        'output_root': str(output_root),
        'domain_id_start': domain_start,
        'defaults': {
            'package': package,
            'launch_file': launch_file,
            'repetitions': repetitions,
            'parameters': default_parameters,
        },
        'scenarios': validated_scenarios,
        'source_path': str(path),
        'source_sha256': sha256_file(path),
    }


def build_trial_plan(
    manifest: dict,
    output_root: Path | None = None,
) -> dict:
    """Expand a validated manifest into deterministic, non-executed trials."""
    root = (
        output_root.expanduser().resolve()
        if output_root is not None
        else Path(manifest['output_root']).expanduser().resolve()
    )
    if not root.is_absolute():
        raise ValueError('trial output root must be absolute')
    defaults = manifest['defaults']
    trials = []
    next_domain = int(manifest['domain_id_start'])
    for scenario in manifest['scenarios']:
        merged_parameters = {
            **defaults['parameters'],
            **scenario['parameters'],
        }
        for repetition in range(1, scenario['repetitions'] + 1):
            trial_id = f"{scenario['id']}__r{repetition:03d}"
            report_path = root / scenario['id'] / f'trial_{repetition:03d}.json'
            launch_arguments = {
                **merged_parameters,
                'domain_id': next_domain,
                'output_path': str(report_path),
            }
            command = [
                'ros2',
                'launch',
                defaults['package'],
                defaults['launch_file'],
                *[
                    f'{name}:={_format_launch_value(value)}'
                    for name, value in sorted(launch_arguments.items())
                ],
            ]
            trials.append({
                'trial_id': trial_id,
                'scenario_id': scenario['id'],
                'repetition': repetition,
                'research_question': scenario['research_question'],
                'hypothesis': scenario['hypothesis'],
                'factors': scenario['factors'],
                'acceptance': scenario['acceptance'],
                'ros_domain_id': next_domain,
                'report_path': str(report_path),
                'launch_arguments': launch_arguments,
                'command_argv': command,
                'status': 'planned',
            })
            next_domain += 1

    stable_plan = {
        'study_id': manifest['study_id'],
        'manifest_sha256': manifest['source_sha256'],
        'output_root': str(root),
        'trials': trials,
    }
    return {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'study_id': manifest['study_id'],
        'description': manifest['description'],
        'source_manifest_path': manifest['source_path'],
        'source_manifest_sha256': manifest['source_sha256'],
        'plan_fingerprint_sha256': _canonical_sha256(stable_plan),
        'safety_scope': {
            'commands_executed': False,
            'controller_launch_allowed': False,
            'hardware_launch_allowed': False,
            'velocity_commands_allowed': False,
        },
        'output_root': str(root),
        'scenario_count': len(manifest['scenarios']),
        'trial_count': len(trials),
        'trials': trials,
    }


def write_trial_plan(plan: dict, output_path: Path) -> Path:
    """Write a trial plan atomically."""
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(output_path.name + '.tmp')
    temporary_path.write_text(
        json.dumps(plan, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary_path, output_path)
    return output_path


def main(args=None) -> None:
    """Validate a manifest and write its expanded trial plan."""
    parser = argparse.ArgumentParser(
        description='Expand a safe semantic planning paper manifest.',
    )
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--output-root', type=Path)
    parsed = parser.parse_args(args)
    try:
        manifest = load_manifest(parsed.manifest)
        plan = build_trial_plan(manifest, parsed.output_root)
        output_path = write_trial_plan(plan, parsed.output)
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        print(f'cannot build experiment plan: {error}', file=sys.stderr)
        raise SystemExit(2)
    print(
        'planned %d scenarios and %d trials; executed=false; output=%s'
        % (plan['scenario_count'], plan['trial_count'], output_path)
    )


if __name__ == '__main__':
    main()
