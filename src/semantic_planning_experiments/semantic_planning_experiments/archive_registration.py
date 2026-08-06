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

"""Register a completed external Rosbag without launching collection."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

import yaml

from semantic_planning_experiments.archive_audit import (
    load_validated_archive_index,
)
from semantic_planning_experiments.archive_audit import (
    validate_collected_unit_artifacts,
)
from semantic_planning_experiments.metrics import sha256_file


def _outside_workspace(path: Path, workspace_root: Path) -> None:
    try:
        path.expanduser().resolve().relative_to(
            workspace_root.expanduser().resolve()
        )
    except ValueError:
        return
    raise ValueError('experiment artifacts must remain outside the repository')


def _load_json_mapping(path: Path, description: str) -> dict:
    if path.suffix.lower() != '.json':
        raise ValueError(f'{description} must be a JSON file')
    value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'{description} must contain a mapping')
    return value


def _encoded_json(value: dict) -> bytes:
    return (
        json.dumps(
            value,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        ) + '\n'
    ).encode('utf-8')


def _encoded_yaml(value: dict) -> bytes:
    return yaml.safe_dump(
        value,
        sort_keys=False,
        allow_unicode=True,
    ).encode('utf-8')


def _temporary_file(target: Path, content: bytes) -> Path:
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, raw_path = tempfile.mkstemp(
        prefix=f'.{target.name}.',
        suffix='.tmp',
        dir=target.parent,
    )
    temporary = Path(raw_path)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return temporary


def register_collected_unit(
    index_path: Path,
    unit_id: str,
    metadata_input: Path,
    receipt_path: Path,
    workspace_root: Path,
) -> dict:
    """Validate and atomically register one previously recorded unit."""
    workspace_root = workspace_root.expanduser().resolve()
    index_path = index_path.expanduser().resolve()
    metadata_input = metadata_input.expanduser().resolve()
    receipt_path = receipt_path.expanduser().resolve()
    _outside_workspace(metadata_input, workspace_root)
    _outside_workspace(receipt_path, workspace_root)
    if not metadata_input.is_file() or metadata_input.is_symlink():
        raise ValueError('metadata input must be a regular file')
    if receipt_path.exists():
        raise ValueError(
            f'registration receipt already exists: {receipt_path}'
        )

    index_before_sha256 = sha256_file(index_path)
    validated, lock = load_validated_archive_index(
        index_path,
        workspace_root,
    )
    if sha256_file(index_path) != index_before_sha256:
        raise ValueError('archive index changed during validation')
    units = {
        unit['unit_id']: unit
        for unit in validated['units']
    }
    if unit_id not in units:
        raise ValueError(f'archive unit does not exist: {unit_id}')
    unit = units[unit_id]
    if unit['status'] != 'planned':
        raise ValueError(
            f'archive unit is not planned: {unit_id} ({unit["status"]})'
        )
    if not unit['bag_path'].is_dir():
        raise ValueError(
            f'Rosbag directory does not exist: {unit["bag_path"]}'
        )
    if unit['metadata_path'].exists():
        raise ValueError(
            f'archive metadata already exists: {unit["metadata_path"]}'
        )
    if metadata_input == unit['metadata_path']:
        raise ValueError('metadata input must be separate from archive output')
    if receipt_path in {index_path, metadata_input, unit['metadata_path']}:
        raise ValueError('registration paths must be distinct')

    metadata_source_sha256 = sha256_file(metadata_input)
    metadata = _load_json_mapping(metadata_input, 'metadata input')
    artifacts = validate_collected_unit_artifacts(
        unit,
        validated['required_topics'],
        validated['required_metadata'],
        metadata_input,
        lock['protocol'].get('data_collection', {}).get('requirements'),
    )
    raw_index = yaml.safe_load(index_path.read_text(encoding='utf-8'))
    if not isinstance(raw_index, dict):
        raise ValueError('archive index must contain a mapping')
    matching_units = [
        raw_unit for raw_unit in raw_index.get('units', [])
        if isinstance(raw_unit, dict)
        and raw_unit.get('unit_id') == unit_id
    ]
    if len(matching_units) != 1:
        raise ValueError('archive unit mapping changed during registration')
    raw_unit = matching_units[0]
    if raw_unit.get('status') != 'planned':
        raise ValueError('archive unit status changed during registration')
    raw_unit['status'] = 'collected'
    raw_unit['invalid_reason'] = None

    metadata_content = _encoded_json(metadata)
    index_content = _encoded_yaml(raw_index)
    index_after_sha256 = hashlib.sha256(index_content).hexdigest()
    metadata_archive_sha256 = hashlib.sha256(metadata_content).hexdigest()
    artifacts['metadata']['sha256'] = metadata_archive_sha256
    receipt = {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'registration_status': 'collected',
        'commands_executed': False,
        'protocol': {
            'protocol_id': lock['protocol']['protocol_id'],
            'freeze_fingerprint_sha256': lock[
                'freeze_fingerprint_sha256'
            ],
            'code_revision': lock['code_revision'],
            'tooling_revision': lock.get(
                '_tooling_verification', {}
            ).get('tooling_revision', lock['code_revision']),
        },
        'unit': {
            'unit_id': unit_id,
            'stratum': unit['stratum'],
            'ordinal': unit['ordinal'],
            'replaces_unit_id': unit.get('replaces_unit_id'),
        },
        'index': {
            'path': str(index_path),
            'before_sha256': index_before_sha256,
            'after_sha256': index_after_sha256,
        },
        'metadata': {
            'source_path': str(metadata_input),
            'source_sha256': metadata_source_sha256,
            'archive_path': str(unit['metadata_path']),
            'archive_sha256': metadata_archive_sha256,
        },
        'artifacts': artifacts,
    }
    receipt_content = _encoded_json(receipt)

    temporary_paths = []
    try:
        metadata_temporary = _temporary_file(
            unit['metadata_path'],
            metadata_content,
        )
        temporary_paths.append(metadata_temporary)
        index_temporary = _temporary_file(index_path, index_content)
        temporary_paths.append(index_temporary)
        receipt_temporary = _temporary_file(receipt_path, receipt_content)
        temporary_paths.append(receipt_temporary)
        if sha256_file(index_path) != index_before_sha256:
            raise ValueError('archive index changed before registration')
        if sha256_file(metadata_input) != metadata_source_sha256:
            raise ValueError('metadata input changed before registration')
        if unit['metadata_path'].exists() or receipt_path.exists():
            raise ValueError('registration output appeared before commit')
        os.replace(metadata_temporary, unit['metadata_path'])
        temporary_paths.remove(metadata_temporary)
        os.replace(index_temporary, index_path)
        temporary_paths.remove(index_temporary)
        os.replace(receipt_temporary, receipt_path)
        temporary_paths.remove(receipt_temporary)
    finally:
        for temporary in temporary_paths:
            temporary.unlink(missing_ok=True)
    return receipt


def main(args=None) -> None:
    """Register one completed Rosbag without launching ROS or hardware."""
    parser = argparse.ArgumentParser(
        description='Validate and register one completed archive unit.',
    )
    parser.add_argument('--workspace-root', type=Path, default=Path.cwd())
    parser.add_argument('--archive-index', required=True, type=Path)
    parser.add_argument('--unit-id', required=True)
    parser.add_argument('--metadata-input', required=True, type=Path)
    parser.add_argument('--receipt', required=True, type=Path)
    parsed = parser.parse_args(args)
    try:
        receipt = register_collected_unit(
            parsed.archive_index,
            parsed.unit_id,
            parsed.metadata_input,
            parsed.receipt,
            parsed.workspace_root,
        )
    except (
        OSError,
        KeyError,
        TypeError,
        ValueError,
        json.JSONDecodeError,
        yaml.YAMLError,
    ) as error:
        print(f'archive registration failed: {error}', file=sys.stderr)
        raise SystemExit(2) from error
    print(
        'archive unit registered; status=collected; '
        'commands_executed=false; '
        f'unit={receipt["unit"]["unit_id"]}; output={parsed.receipt}'
    )
