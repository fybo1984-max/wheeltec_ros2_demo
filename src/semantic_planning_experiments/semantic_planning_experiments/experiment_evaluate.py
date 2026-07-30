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

"""Evaluate planned paper acceptance criteria against experiment reports."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys


_OPERATORS = {
    'eq': lambda actual, target: math.isclose(
        actual, target, rel_tol=0.0, abs_tol=1e-9
    ),
    'ge': lambda actual, target: actual >= target,
    'gt': lambda actual, target: actual > target,
    'le': lambda actual, target: actual <= target,
    'lt': lambda actual, target: actual < target,
}
_CONDITION_GROUPS = {
    'baseline',
    'semantic',
    'recovered_after_mask_clear',
}
_DELTA_GROUPS = {
    'delta_semantic_minus_baseline',
    'delta_recovered_minus_baseline',
}


def _load_json(path: Path, description: str) -> dict:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except OSError as error:
        raise ValueError(f'cannot read {description}: {error}') from error
    except json.JSONDecodeError as error:
        raise ValueError(f'invalid {description} JSON: {error}') from error
    if not isinstance(value, dict):
        raise ValueError(f'{description} must be a JSON object')
    return value


def _validate_plan(plan: dict) -> list[dict]:
    if plan.get('schema_version') != 1:
        raise ValueError('plan schema_version must be 1')
    safety = plan.get('safety_scope')
    required_safety = {
        'commands_executed': False,
        'controller_launch_allowed': False,
        'hardware_launch_allowed': False,
        'velocity_commands_allowed': False,
    }
    if safety != required_safety:
        raise ValueError('plan safety_scope is missing or unsafe')
    trials = plan.get('trials')
    if not isinstance(trials, list) or not trials:
        raise ValueError('plan needs at least one trial')
    if plan.get('trial_count') != len(trials):
        raise ValueError('plan trial_count does not match trials')

    seen_ids = set()
    validated = []
    for index, trial in enumerate(trials):
        if not isinstance(trial, dict):
            raise ValueError(f'plan trial[{index}] must be an object')
        trial_id = trial.get('trial_id')
        if not isinstance(trial_id, str) or not trial_id:
            raise ValueError(f'plan trial[{index}] needs trial_id')
        if trial_id in seen_ids:
            raise ValueError(f'duplicate trial_id: {trial_id}')
        seen_ids.add(trial_id)
        report_path = trial.get('report_path')
        if not isinstance(report_path, str) or not Path(report_path).is_absolute():
            raise ValueError(f'trial {trial_id} report_path must be absolute')
        criteria = trial.get('acceptance')
        if not isinstance(criteria, list) or not criteria:
            raise ValueError(f'trial {trial_id} needs acceptance criteria')
        for criterion in criteria:
            if not isinstance(criterion, dict):
                raise ValueError(
                    f'trial {trial_id} acceptance criterion must be an object'
                )
            if criterion.get('operator') not in _OPERATORS:
                raise ValueError(
                    f'trial {trial_id} has invalid acceptance operator'
                )
            target = criterion.get('value')
            if isinstance(target, bool) or not isinstance(target, (int, float)):
                raise ValueError(
                    f'trial {trial_id} acceptance target must be numeric'
                )
            metric = criterion.get('metric')
            if not isinstance(metric, str) or metric.count('.') != 1:
                raise ValueError(
                    f'trial {trial_id} has invalid acceptance metric'
                )
            group, name = metric.split('.', 1)
            if (
                group not in _CONDITION_GROUPS | _DELTA_GROUPS
                or not name
            ):
                raise ValueError(
                    f'trial {trial_id} has invalid acceptance metric'
                )
        validated.append(trial)
    return validated


def _report_validity_error(report: dict) -> str | None:
    if report.get('schema_version') != 1:
        return 'report schema_version must be 1'
    revision = report.get('code_revision')
    if not isinstance(revision, str) or not revision:
        return 'report code_revision is missing'
    if revision == 'unknown' or revision.endswith('-dirty'):
        return f'report code_revision is not publishable: {revision}'
    safety = report.get('safety_scope')
    if not isinstance(safety, dict):
        return 'report safety_scope is missing'
    for name in (
        'controller_started',
        'velocity_commands_published',
        'hardware_nodes_started',
    ):
        if safety.get(name) is not False:
            return f'report safety_scope requires {name}=false'
    return None


def _metric_value(report: dict, metric_path: str) -> float:
    group, name = metric_path.split('.', 1)
    value = report.get(group)
    if group in _CONDITION_GROUPS:
        if not isinstance(value, dict):
            raise ValueError(f'report group {group} is missing')
        value = value.get('metrics')
    if not isinstance(value, dict) or name not in value:
        raise ValueError(f'report metric {metric_path} is missing')
    actual = value[name]
    if isinstance(actual, bool) or not isinstance(actual, (int, float)):
        raise ValueError(f'report metric {metric_path} is not numeric')
    actual = float(actual)
    if not math.isfinite(actual):
        raise ValueError(f'report metric {metric_path} is not finite')
    return actual


def evaluate_trial(trial: dict) -> dict:
    """Evaluate one planned trial without changing its report."""
    report_path = Path(trial['report_path'])
    result = {
        'trial_id': trial['trial_id'],
        'scenario_id': trial.get('scenario_id'),
        'repetition': trial.get('repetition'),
        'report_path': str(report_path),
        'status': 'missing',
        'criteria': [],
        'reason': None,
    }
    if not report_path.is_file():
        result['reason'] = 'report file does not exist'
        return result

    try:
        report = _load_json(report_path, f"trial {trial['trial_id']} report")
    except ValueError as error:
        result['status'] = 'invalid'
        result['reason'] = str(error)
        return result
    validity_error = _report_validity_error(report)
    if validity_error is not None:
        result['status'] = 'invalid'
        result['reason'] = validity_error
        return result

    try:
        for criterion in trial['acceptance']:
            actual = _metric_value(report, criterion['metric'])
            target = float(criterion['value'])
            operator = criterion['operator']
            passed = bool(_OPERATORS[operator](actual, target))
            result['criteria'].append({
                'metric': criterion['metric'],
                'operator': operator,
                'target': target,
                'actual': actual,
                'passed': passed,
            })
    except ValueError as error:
        result['status'] = 'invalid'
        result['reason'] = str(error)
        return result

    result['status'] = (
        'passed'
        if all(criterion['passed'] for criterion in result['criteria'])
        else 'failed'
    )
    return result


def evaluate_plan(plan: dict) -> dict:
    """Return a machine-readable acceptance audit for a trial plan."""
    trials = _validate_plan(plan)
    results = [evaluate_trial(trial) for trial in trials]
    counts = {
        status: sum(result['status'] == status for result in results)
        for status in ('passed', 'failed', 'missing', 'invalid')
    }
    return {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'study_id': plan.get('study_id'),
        'plan_fingerprint_sha256': plan.get('plan_fingerprint_sha256'),
        'trial_count': len(results),
        'status_counts': counts,
        'all_passed': counts == {
            'passed': len(results),
            'failed': 0,
            'missing': 0,
            'invalid': 0,
        },
        'trials': results,
    }


def write_evaluation(evaluation: dict, output_path: Path) -> Path:
    """Write an evaluation atomically."""
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(output_path.name + '.tmp')
    temporary_path.write_text(
        json.dumps(evaluation, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary_path, output_path)
    return output_path


def main(args=None) -> None:
    """Evaluate a generated paper trial plan."""
    parser = argparse.ArgumentParser(
        description='Evaluate semantic planning paper acceptance criteria.',
    )
    parser.add_argument('plan', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parsed = parser.parse_args(args)
    try:
        plan_path = parsed.plan.expanduser().resolve()
        plan = _load_json(plan_path, 'experiment plan')
        evaluation = evaluate_plan(plan)
        output_path = write_evaluation(evaluation, parsed.output)
    except (OSError, TypeError, ValueError) as error:
        print(f'cannot evaluate experiment plan: {error}', file=sys.stderr)
        raise SystemExit(2)

    counts = evaluation['status_counts']
    print(
        'evaluated %d trials: passed=%d failed=%d missing=%d invalid=%d; '
        'output=%s'
        % (
            evaluation['trial_count'],
            counts['passed'],
            counts['failed'],
            counts['missing'],
            counts['invalid'],
            output_path,
        )
    )
    if not evaluation['all_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
