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

"""Allocate an auditable replacement for one invalid archive unit."""

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
from semantic_planning_experiments.metrics import sha256_file


def _outside_workspace(path: Path, workspace_root: Path) -> None:
    try:
        path.expanduser().resolve().relative_to(
            workspace_root.expanduser().resolve()
        )
    except ValueError:
        return
    raise ValueError(
        'replacement artifacts must remain outside the repository'
    )


def _encoded_json(value: dict) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + '\n'
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


def allocate_replacement(
    index_path: Path,
    replaces_unit_id: str,
    receipt_path: Path,
    workspace_root: Path,
) -> dict:
    """Append one planned replacement and atomically record its receipt."""
    workspace_root = workspace_root.expanduser().resolve()
    index_path = index_path.expanduser().resolve()
    receipt_path = receipt_path.expanduser().resolve()
    _outside_workspace(index_path, workspace_root)
    _outside_workspace(receipt_path, workspace_root)
    if receipt_path.exists():
        raise ValueError(f'replacement receipt already exists: {receipt_path}')

    index_before_sha256 = sha256_file(index_path)
    validated, lock = load_validated_archive_index(index_path, workspace_root)
    if sha256_file(index_path) != index_before_sha256:
        raise ValueError('archive index changed during validation')
    units = {unit['unit_id']: unit for unit in validated['units']}
    source = units.get(replaces_unit_id)
    if source is None:
        raise ValueError(f'archive unit does not exist: {replaces_unit_id}')
    if source['status'] != 'invalid':
        raise ValueError(
            f'replacement source is not invalid: {replaces_unit_id}'
        )
    if any(
        unit.get('replaces_unit_id') == replaces_unit_id
        for unit in validated['units']
    ):
        raise ValueError(
            f'archive unit already has a replacement: {replaces_unit_id}'
        )

    stratum = source['stratum']
    ordinal = max(
        unit['ordinal'] for unit in validated['units']
        if unit['stratum'] == stratum
    ) + 1
    unit_id = f'{stratum}_{ordinal:03d}'
    archive_root = validated['archive_root']
    replacement = {
        'unit_id': unit_id,
        'stratum': stratum,
        'ordinal': ordinal,
        'status': 'planned',
        'bag_path': f'bags/{unit_id}',
        'metadata_path': f'metadata/{unit_id}.json',
        'invalid_reason': None,
        'replaces_unit_id': replaces_unit_id,
    }
    for relative in (replacement['bag_path'], replacement['metadata_path']):
        if (archive_root / relative).exists():
            raise ValueError(
                f'replacement artifact already exists: {relative}'
            )

    raw_index = yaml.safe_load(index_path.read_text(encoding='utf-8'))
    if not isinstance(raw_index, dict):
        raise ValueError('archive index must contain a mapping')
    raw_units = raw_index.get('units')
    if not isinstance(raw_units, list):
        raise ValueError('archive index units must be a list')
    raw_units.append(replacement)
    index_content = _encoded_yaml(raw_index)
    index_after_sha256 = hashlib.sha256(index_content).hexdigest()
    verification = lock.get('_tooling_verification', {})
    receipt = {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'allocation_status': 'planned',
        'commands_executed': False,
        'protocol': {
            'protocol_id': lock['protocol']['protocol_id'],
            'freeze_fingerprint_sha256': lock[
                'freeze_fingerprint_sha256'
            ],
            'code_revision': lock['code_revision'],
            'tooling_revision': verification.get(
                'tooling_revision', lock['code_revision']
            ),
        },
        'replacement': replacement,
        'index': {
            'path': str(index_path),
            'before_sha256': index_before_sha256,
            'after_sha256': index_after_sha256,
        },
    }
    receipt_content = _encoded_json(receipt)
    temporary_paths = []
    try:
        index_temporary = _temporary_file(index_path, index_content)
        temporary_paths.append(index_temporary)
        receipt_temporary = _temporary_file(receipt_path, receipt_content)
        temporary_paths.append(receipt_temporary)
        if sha256_file(index_path) != index_before_sha256:
            raise ValueError('archive index changed before replacement commit')
        if receipt_path.exists():
            raise ValueError('replacement receipt appeared before commit')
        os.replace(index_temporary, index_path)
        temporary_paths.remove(index_temporary)
        os.replace(receipt_temporary, receipt_path)
        temporary_paths.remove(receipt_temporary)
    finally:
        for temporary in temporary_paths:
            temporary.unlink(missing_ok=True)
    return receipt


def main(args=None) -> None:
    """Allocate one replacement without launching ROS or hardware."""
    parser = argparse.ArgumentParser(
        description='Allocate a new unit for one invalid archive unit.',
    )
    parser.add_argument('--workspace-root', type=Path, default=Path.cwd())
    parser.add_argument('--archive-index', required=True, type=Path)
    parser.add_argument('--replaces-unit-id', required=True)
    parser.add_argument('--receipt', required=True, type=Path)
    parsed = parser.parse_args(args)
    try:
        receipt = allocate_replacement(
            parsed.archive_index,
            parsed.replaces_unit_id,
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
        print(f'replacement allocation failed: {error}', file=sys.stderr)
        raise SystemExit(2) from error
    replacement = receipt['replacement']
    print(
        'replacement allocated; status=planned; commands_executed=false; '
        f'unit={replacement["unit_id"]}; '
        f'replaces={replacement["replaces_unit_id"]}; '
        f'output={parsed.receipt}'
    )


if __name__ == '__main__':
    main()
