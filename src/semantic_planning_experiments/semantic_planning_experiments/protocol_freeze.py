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

"""Freeze and verify publication experiment protocols against clean Git."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

import yaml

from semantic_planning_experiments.metrics import sha256_file


_ID_PATTERN = re.compile(r'^[a-z0-9][a-z0-9_-]*$')
_METRIC_PATTERN = re.compile(r'^[a-z][a-z0-9_.]*$')
_PHASES = {'recorded_rgbd_pilot', 'formal_offline', 'physical_agv'}
_METHODS = {'baseline', 'fixed', 'lethal', 'fuzzy'}
_DIRECTIONS = {'higher', 'lower'}
_ANALYSES = {'descriptive', 'paired'}
_SAMPLE_PURPOSES = {'variance_estimation', 'confirmatory_powered'}
_MULTIPLE_COMPARISONS = {'none', 'holm_bonferroni'}
_REQUIRED_COLLECTION_SCOPE = {
    'controller_allowed': False,
    'hardware_motion_allowed': False,
    'velocity_commands_allowed': False,
    'physical_camera_allowed': True,
    'operator_notification_required': True,
}


def _canonical_sha256(value) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _mapping(value, description: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f'{description} must be a mapping')
    return value


def _nonempty_string(value, description: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{description} must be a nonempty string')
    return value.strip()


def _string_list(value, description: str) -> list[str]:
    if not isinstance(value, list) or not value:
        raise ValueError(f'{description} must be a nonempty list')
    result = [
        _nonempty_string(item, f'{description}[{index}]')
        for index, item in enumerate(value)
    ]
    if len(set(result)) != len(result):
        raise ValueError(f'{description} must not contain duplicates')
    return result


def _git(workspace_root: Path, *arguments: str) -> str:
    result = subprocess.run(
        ['git', *arguments],
        cwd=workspace_root,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        reason = result.stderr.strip() or result.stdout.strip()
        raise ValueError(f'git {arguments[0]} failed: {reason}')
    return result.stdout.strip()


def _git_state(workspace_root: Path) -> dict:
    root = Path(
        _git(workspace_root, 'rev-parse', '--show-toplevel')
    ).resolve()
    if root != workspace_root:
        raise ValueError(f'workspace root must be the Git root: {root}')
    status = _git(workspace_root, 'status', '--porcelain=v1')
    if status:
        raise ValueError('protocol freeze requires a clean Git worktree')
    branch = _git(workspace_root, 'branch', '--show-current')
    revision = _git(workspace_root, 'rev-parse', 'HEAD')
    if not branch or not revision:
        raise ValueError('Git branch and revision must be available')
    return {'branch': branch, 'revision': revision}


def _is_ancestor(
    workspace_root: Path,
    ancestor: str,
    descendant: str,
) -> bool:
    result = subprocess.run(
        ['git', 'merge-base', '--is-ancestor', ancestor, descendant],
        cwd=workspace_root,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode not in (0, 1):
        reason = result.stderr.strip() or result.stdout.strip()
        raise ValueError(f'git merge-base failed: {reason}')
    return result.returncode == 0


def _tracked_relative_path(path: Path, workspace_root: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        relative = resolved.relative_to(workspace_root)
    except ValueError as error:
        raise ValueError(
            f'protocol asset is outside the workspace: {path}'
        ) from error
    if not resolved.is_file():
        raise ValueError(f'protocol asset does not exist: {relative}')
    _git(workspace_root, 'ls-files', '--error-unmatch', '--', str(relative))
    return relative.as_posix()


def _validate_endpoint(value, description: str) -> dict:
    endpoint = _mapping(value, description)
    endpoint_id = _nonempty_string(endpoint.get('id'), f'{description}.id')
    if not _ID_PATTERN.fullmatch(endpoint_id):
        raise ValueError(f'{description}.id is invalid')
    metric = _nonempty_string(endpoint.get('metric'), f'{description}.metric')
    if not _METRIC_PATTERN.fullmatch(metric):
        raise ValueError(f'{description}.metric is invalid')
    direction = endpoint.get('direction')
    if direction not in _DIRECTIONS:
        raise ValueError(f'{description}.direction is invalid')
    analysis = endpoint.get('analysis')
    if analysis not in _ANALYSES:
        raise ValueError(f'{description}.analysis is invalid')
    return {
        'id': endpoint_id,
        'metric': metric,
        'direction': direction,
        'analysis': analysis,
        'rationale': _nonempty_string(
            endpoint.get('rationale'),
            f'{description}.rationale',
        ),
    }


def _validate_endpoints(value) -> dict:
    endpoints = _mapping(value, 'endpoints')
    primary_raw = endpoints.get('primary')
    secondary_raw = endpoints.get('secondary')
    if not isinstance(primary_raw, list) or not primary_raw:
        raise ValueError('endpoints.primary must be a nonempty list')
    if not isinstance(secondary_raw, list):
        raise ValueError('endpoints.secondary must be a list')
    primary = [
        _validate_endpoint(item, f'endpoints.primary[{index}]')
        for index, item in enumerate(primary_raw)
    ]
    secondary = [
        _validate_endpoint(item, f'endpoints.secondary[{index}]')
        for index, item in enumerate(secondary_raw)
    ]
    identifiers = [item['id'] for item in primary + secondary]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError('endpoint ids must be unique')
    return {'primary': primary, 'secondary': secondary}


def _validate_sample_size(value) -> dict:
    sample_size = _mapping(value, 'sample_size')
    total = sample_size.get('total_independent_units')
    if isinstance(total, bool) or not isinstance(total, int) or total < 2:
        raise ValueError('sample_size.total_independent_units must be >= 2')
    allocation_raw = _mapping(
        sample_size.get('allocation'),
        'sample allocation',
    )
    allocation = {}
    for name, count in allocation_raw.items():
        if not isinstance(name, str) or not _ID_PATTERN.fullmatch(name):
            raise ValueError(f'invalid sample allocation id: {name}')
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError(f'sample allocation {name} must be positive')
        allocation[name] = count
    if sum(allocation.values()) != total:
        raise ValueError(
            'sample allocation does not equal total independent units'
        )
    purpose = sample_size.get('purpose')
    if purpose not in _SAMPLE_PURPOSES:
        raise ValueError('sample_size.purpose is invalid')
    return {
        'total_independent_units': total,
        'allocation': allocation,
        'purpose': purpose,
        'rationale': _nonempty_string(
            sample_size.get('rationale'),
            'sample_size.rationale',
        ),
    }


def _finite_probability(value, description: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0.0
        or value >= 1.0
    ):
        raise ValueError(f'{description} must be between zero and one')
    return float(value)


def _positive_finite(value, description: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0.0
    ):
        raise ValueError(f'{description} must be positive and finite')
    return float(value)


def _validate_collection_requirements(value) -> dict | None:
    if value is None:
        return None
    requirements = _mapping(value, 'data_collection.requirements')
    duration = _mapping(
        requirements.get('recording_duration_s'),
        'data_collection.requirements.recording_duration_s',
    )
    minimum = _positive_finite(
        duration.get('minimum'),
        'recording duration minimum',
    )
    maximum = _positive_finite(
        duration.get('maximum'),
        'recording duration maximum',
    )
    if maximum < minimum:
        raise ValueError('recording duration maximum must follow minimum')
    pose = _mapping(
        requirements.get('measured_person_pose'),
        'data_collection.requirements.measured_person_pose',
    )
    frame_id = _nonempty_string(
        pose.get('frame_id'),
        'measured person pose frame_id',
    )
    maximum_uncertainty = _positive_finite(
        pose.get('maximum_uncertainty_m'),
        'measured person pose maximum uncertainty',
    )
    if pose.get('measured_before_recording') is not True:
        raise ValueError('person pose must be measured before recording')
    observation = _mapping(
        requirements.get('observation'),
        'data_collection.requirements.observation',
    )
    if observation.get('person_stable_before_recording') is not True:
        raise ValueError('person must be stable before recording')
    if observation.get('setup_motion_recorded') is not False:
        raise ValueError('setup motion must not be recorded')
    strata_raw = _mapping(
        requirements.get('person_distance_strata_m'),
        'data_collection.requirements.person_distance_strata_m',
    )
    strata = {}
    for name, values_raw in strata_raw.items():
        if not isinstance(name, str) or not _ID_PATTERN.fullmatch(name):
            raise ValueError('person distance stratum name is invalid')
        values = _mapping(
            values_raw,
            f'person_distance_strata_m.{name}',
        )
        strata[name] = {
            'target': _positive_finite(
                values.get('target'),
                f'person distance stratum {name} target',
            ),
            'tolerance': _positive_finite(
                values.get('tolerance'),
                f'person distance stratum {name} tolerance',
            ),
        }
    if not strata:
        raise ValueError('person distance strata must not be empty')
    return {
        'recording_duration_s': {
            'minimum': minimum,
            'maximum': maximum,
        },
        'measured_person_pose': {
            'frame_id': frame_id,
            'maximum_uncertainty_m': maximum_uncertainty,
            'measured_before_recording': True,
        },
        'observation': {
            'person_stable_before_recording': True,
            'setup_motion_recorded': False,
        },
        'person_distance_strata_m': strata,
    }


def _validate_statistics(value, confirmatory: bool) -> dict:
    statistics_config = _mapping(value, 'statistics')
    alpha = _finite_probability(
        statistics_config.get('alpha'),
        'statistics.alpha',
    )
    confidence = _finite_probability(
        statistics_config.get('confidence_level'),
        'statistics.confidence_level',
    )
    if not math.isclose(confidence, 1.0 - alpha, abs_tol=1e-12):
        raise ValueError('confidence_level must equal 1 - alpha')
    if statistics_config.get('paired_test') != 'sign_flip':
        raise ValueError('statistics.paired_test must be sign_flip')
    if statistics_config.get('exact_pair_limit') != 16:
        raise ValueError('statistics.exact_pair_limit must be 16')
    if statistics_config.get('monte_carlo_samples') != 100000:
        raise ValueError('statistics.monte_carlo_samples must be 100000')
    if statistics_config.get('monte_carlo_seed') != 20260801:
        raise ValueError('statistics.monte_carlo_seed must be 20260801')
    if statistics_config.get('effect_size') != 'cohen_dz':
        raise ValueError('statistics.effect_size must be cohen_dz')
    if statistics_config.get('zero_variance_effect') != 'null':
        raise ValueError('statistics.zero_variance_effect must be null')
    multiple = statistics_config.get('multiple_comparisons')
    if multiple not in _MULTIPLE_COMPARISONS:
        raise ValueError('statistics.multiple_comparisons is invalid')
    if confirmatory and multiple == 'none':
        raise ValueError('confirmatory protocols require multiplicity control')
    return {
        'alpha': alpha,
        'confidence_level': confidence,
        'paired_test': 'sign_flip',
        'exact_pair_limit': 16,
        'monte_carlo_samples': 100000,
        'monte_carlo_seed': 20260801,
        'effect_size': 'cohen_dz',
        'zero_variance_effect': 'null',
        'multiple_comparisons': multiple,
    }


def load_protocol(path: Path, workspace_root: Path) -> dict:
    """Load and strictly validate a tracked experiment protocol."""
    workspace_root = workspace_root.expanduser().resolve()
    source_relative = _tracked_relative_path(path, workspace_root)
    source_path = workspace_root / source_relative
    protocol = yaml.safe_load(source_path.read_text(encoding='utf-8'))
    protocol = _mapping(protocol, 'protocol')
    if protocol.get('schema_version') != 1:
        raise ValueError('protocol schema_version must be 1')
    protocol_id = _nonempty_string(protocol.get('protocol_id'), 'protocol_id')
    if not _ID_PATTERN.fullmatch(protocol_id):
        raise ValueError('protocol_id is invalid')
    phase = protocol.get('phase')
    if phase not in _PHASES:
        raise ValueError('protocol phase is invalid')
    confirmatory = protocol.get('confirmatory')
    if not isinstance(confirmatory, bool):
        raise ValueError('protocol confirmatory must be boolean')

    questions_raw = protocol.get('research_questions')
    if not isinstance(questions_raw, list) or not questions_raw:
        raise ValueError('research_questions must be a nonempty list')
    questions = []
    seen_questions = set()
    for index, question_raw in enumerate(questions_raw):
        question = _mapping(question_raw, f'research_questions[{index}]')
        question_id = _nonempty_string(
            question.get('id'),
            f'research_questions[{index}].id',
        )
        if not re.fullmatch(r'RQ[1-9][0-9]*', question_id):
            raise ValueError(f'invalid research question id: {question_id}')
        if question_id in seen_questions:
            raise ValueError(f'duplicate research question: {question_id}')
        seen_questions.add(question_id)
        questions.append({
            'id': question_id,
            'hypothesis': _nonempty_string(
                question.get('hypothesis'),
                f'research_questions[{index}].hypothesis',
            ),
        })

    methods = _string_list(protocol.get('methods'), 'methods')
    if len(methods) < 2 or any(method not in _METHODS for method in methods):
        raise ValueError('methods need at least two supported method groups')
    unit = _mapping(protocol.get('experimental_unit'), 'experimental_unit')
    experimental_unit = {
        'name': _nonempty_string(unit.get('name'), 'experimental_unit.name'),
        'independence_rule': _nonempty_string(
            unit.get('independence_rule'),
            'experimental_unit.independence_rule',
        ),
        'pseudoreplication_rule': _nonempty_string(
            unit.get('pseudoreplication_rule'),
            'experimental_unit.pseudoreplication_rule',
        ),
    }
    sample_size = _validate_sample_size(protocol.get('sample_size'))
    collection = _mapping(protocol.get('data_collection'), 'data_collection')
    data_collection = {
        'required_topics': _string_list(
            collection.get('required_topics'),
            'data_collection.required_topics',
        ),
        'required_metadata': _string_list(
            collection.get('required_metadata'),
            'data_collection.required_metadata',
        ),
        'storage_policy': collection.get('storage_policy'),
    }
    requirements = _validate_collection_requirements(
        collection.get('requirements')
    )
    if requirements is not None:
        required_requirement_metadata = {
            'measured_person_pose_in_map',
            'assigned_distance_stratum',
            'measured_camera_to_person_distance_m',
            'recording_start_and_end_utc',
            'observation_conditions',
        }
        missing_requirement_metadata = sorted(
            required_requirement_metadata
            - set(data_collection['required_metadata'])
        )
        if missing_requirement_metadata:
            raise ValueError(
                'required_metadata is missing collection requirement '
                f'fields: {missing_requirement_metadata}'
            )
        if set(requirements['person_distance_strata_m']) != set(
            sample_size['allocation']
        ):
            raise ValueError(
                'person distance strata must match sample allocation'
            )
        data_collection['requirements'] = requirements
    if data_collection['storage_policy'] != 'outside_repository':
        raise ValueError(
            'data_collection.storage_policy must be outside_repository'
        )
    collection_scope = _mapping(
        protocol.get('collection_scope'),
        'collection_scope',
    )
    if collection_scope != _REQUIRED_COLLECTION_SCOPE:
        raise ValueError('collection_scope is missing or unsafe')

    asset_names = _string_list(protocol.get('assets'), 'assets')
    assets = [
        _tracked_relative_path(workspace_root / name, workspace_root)
        for name in asset_names
    ]
    return {
        'schema_version': 1,
        'protocol_id': protocol_id,
        'title': _nonempty_string(protocol.get('title'), 'title'),
        'phase': phase,
        'expected_branch': _nonempty_string(
            protocol.get('expected_branch'),
            'expected_branch',
        ),
        'confirmatory': confirmatory,
        'research_questions': questions,
        'methods': methods,
        'experimental_unit': experimental_unit,
        'endpoints': _validate_endpoints(protocol.get('endpoints')),
        'sample_size': sample_size,
        'statistics': _validate_statistics(
            protocol.get('statistics'),
            confirmatory,
        ),
        'exclusion_rules': _string_list(
            protocol.get('exclusion_rules'),
            'exclusion_rules',
        ),
        'missing_data_policy': _nonempty_string(
            protocol.get('missing_data_policy'),
            'missing_data_policy',
        ),
        'data_collection': data_collection,
        'collection_scope': dict(_REQUIRED_COLLECTION_SCOPE),
        'assets': assets,
        'source_path': source_relative,
        'source_sha256': sha256_file(source_path),
    }


def _stable_lock(lock: dict) -> dict:
    return {
        'schema_version': lock['schema_version'],
        'created_at_utc': lock['created_at_utc'],
        'protocol': lock['protocol'],
        'git_branch': lock['git_branch'],
        'code_revision': lock['code_revision'],
        'source_protocol': lock['source_protocol'],
        'assets': lock['assets'],
    }


def freeze_protocol(path: Path, workspace_root: Path) -> dict:
    """Return a protocol lock bound to a clean Git revision and assets."""
    workspace_root = workspace_root.expanduser().resolve()
    git_state = _git_state(workspace_root)
    protocol = load_protocol(path, workspace_root)
    if git_state['branch'] != protocol['expected_branch']:
        raise ValueError(
            'protocol expected branch %s, current branch is %s'
            % (protocol['expected_branch'], git_state['branch'])
        )
    lock = {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'protocol': {
            key: value
            for key, value in protocol.items()
            if key not in {'source_path', 'source_sha256', 'assets'}
        },
        'git_branch': git_state['branch'],
        'code_revision': git_state['revision'],
        'source_protocol': {
            'path': protocol['source_path'],
            'sha256': protocol['source_sha256'],
        },
        'assets': [
            {
                'path': relative,
                'sha256': sha256_file(workspace_root / relative),
            }
            for relative in protocol['assets']
        ],
    }
    lock['freeze_fingerprint_sha256'] = _canonical_sha256(_stable_lock(lock))
    return lock


def verify_protocol_lock(lock_path: Path, workspace_root: Path) -> dict:
    """Verify a frozen protocol and a compatible clean tooling checkout."""
    workspace_root = workspace_root.expanduser().resolve()
    git_state = _git_state(workspace_root)
    lock_path = lock_path.expanduser().resolve()
    lock = json.loads(lock_path.read_text(encoding='utf-8'))
    if not isinstance(lock, dict) or lock.get('schema_version') != 1:
        raise ValueError('protocol lock schema_version must be 1')
    expected_fingerprint = _canonical_sha256(_stable_lock(lock))
    if lock.get('freeze_fingerprint_sha256') != expected_fingerprint:
        raise ValueError('protocol lock fingerprint mismatch')
    if git_state['branch'] != lock.get('git_branch'):
        raise ValueError('protocol lock branch mismatch')
    locked_revision = lock.get('code_revision')
    if not isinstance(locked_revision, str) or not locked_revision:
        raise ValueError('protocol lock revision is invalid')
    revision_match = git_state['revision'] == locked_revision
    if not revision_match and not _is_ancestor(
        workspace_root,
        locked_revision,
        git_state['revision'],
    ):
        raise ValueError(
            'current tooling revision does not descend from the protocol lock'
        )
    references = [lock['source_protocol'], *lock['assets']]
    for reference in references:
        relative = _tracked_relative_path(
            workspace_root / reference['path'],
            workspace_root,
        )
        if sha256_file(workspace_root / relative) != reference['sha256']:
            raise ValueError(
                f'protocol lock asset checksum mismatch: {relative}'
            )
    return {
        'valid': True,
        'protocol_id': lock['protocol']['protocol_id'],
        'code_revision': locked_revision,
        'tooling_revision': git_state['revision'],
        'revision_relation': 'exact' if revision_match else 'descendant',
        'freeze_fingerprint_sha256': lock['freeze_fingerprint_sha256'],
    }


def write_protocol_lock(lock: dict, output_path: Path) -> Path:
    """Write a protocol lock atomically outside the source repository."""
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(output_path.name + '.tmp')
    temporary.write_text(
        json.dumps(lock, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary, output_path)
    return output_path


def main(args=None) -> None:
    """Freeze a protocol or verify an existing protocol lock."""
    parser = argparse.ArgumentParser(
        description='Freeze or verify a semantic experiment protocol.',
    )
    parser.add_argument('protocol', nargs='?', type=Path)
    parser.add_argument('--workspace-root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path)
    parser.add_argument('--verify-lock', type=Path)
    parsed = parser.parse_args(args)
    try:
        if parsed.verify_lock is not None:
            if parsed.protocol is not None or parsed.output is not None:
                raise ValueError(
                    '--verify-lock cannot be combined with protocol or '
                    '--output'
                )
            verification = verify_protocol_lock(
                parsed.verify_lock,
                parsed.workspace_root,
            )
            print(
                'protocol lock valid; protocol=%s; locked_revision=%s; '
                'tooling_revision=%s; relation=%s; fingerprint=%s'
                % (
                    verification['protocol_id'],
                    verification['code_revision'],
                    verification['tooling_revision'],
                    verification['revision_relation'],
                    verification['freeze_fingerprint_sha256'],
                )
            )
            return
        if parsed.protocol is None or parsed.output is None:
            raise ValueError('protocol and --output are required for freezing')
        lock = freeze_protocol(parsed.protocol, parsed.workspace_root)
        output_path = write_protocol_lock(lock, parsed.output)
    except (
        OSError,
        KeyError,
        TypeError,
        ValueError,
        json.JSONDecodeError,
        yaml.YAMLError,
    ) as error:
        print(f'cannot freeze protocol: {error}', file=sys.stderr)
        raise SystemExit(2)
    print(
        'protocol frozen; protocol=%s; revision=%s; fingerprint=%s; output=%s'
        % (
            lock['protocol']['protocol_id'],
            lock['code_revision'],
            lock['freeze_fingerprint_sha256'],
            output_path,
        )
    )


if __name__ == '__main__':
    main()
