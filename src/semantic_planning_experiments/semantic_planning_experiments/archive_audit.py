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

"""Initialize and audit external recorded RGB-D experiment archives."""

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import sys
from urllib.parse import quote

import yaml

from semantic_planning_experiments.metrics import sha256_file
from semantic_planning_experiments.protocol_freeze import verify_protocol_lock


_UNIT_ID_PATTERN = re.compile(r'^[a-z0-9][a-z0-9_-]*$')
_SHA256_PATTERN = re.compile(r'^[0-9a-fA-F]{64}$')
_UNIT_STATUSES = {'planned', 'collected', 'invalid'}


def _canonical_sha256(value) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    ).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _load_mapping(path: Path, description: str) -> dict:
    path = path.expanduser().resolve()
    if path.suffix.lower() == '.json':
        value = json.loads(path.read_text(encoding='utf-8'))
    else:
        value = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'{description} must contain a mapping')
    return value


def _outside_workspace(path: Path, workspace_root: Path) -> None:
    resolved = path.expanduser().resolve()
    try:
        resolved.relative_to(workspace_root.expanduser().resolve())
    except ValueError:
        return
    raise ValueError('experiment archives must remain outside the repository')


def _expected_units(lock: dict) -> list[dict]:
    allocation = lock['protocol']['sample_size']['allocation']
    units = []
    for stratum, count in allocation.items():
        for ordinal in range(1, count + 1):
            unit_id = f'{stratum}_{ordinal:03d}'
            units.append({
                'unit_id': unit_id,
                'stratum': stratum,
                'ordinal': ordinal,
                'status': 'planned',
                'bag_path': f'bags/{unit_id}',
                'metadata_path': f'metadata/{unit_id}.json',
                'invalid_reason': None,
            })
    return units


def initialize_archive_index(
    protocol_lock_path: Path,
    workspace_root: Path,
    index_path: Path,
) -> Path:
    """Create an external index from a verified protocol lock."""
    workspace_root = workspace_root.expanduser().resolve()
    protocol_lock_path = protocol_lock_path.expanduser().resolve()
    index_path = index_path.expanduser().resolve()
    _outside_workspace(protocol_lock_path, workspace_root)
    _outside_workspace(index_path, workspace_root)
    if index_path.exists():
        raise ValueError(f'archive index already exists: {index_path}')
    verify_protocol_lock(protocol_lock_path, workspace_root)
    lock = _load_mapping(protocol_lock_path, 'protocol lock')
    archive_root = index_path.parent
    index = {
        'schema_version': 1,
        'protocol': {
            'lock_path': str(protocol_lock_path),
            'lock_sha256': sha256_file(protocol_lock_path),
            'protocol_id': lock['protocol']['protocol_id'],
            'freeze_fingerprint_sha256': lock[
                'freeze_fingerprint_sha256'
            ],
            'code_revision': lock['code_revision'],
        },
        'archive_root': str(archive_root),
        'required_topics': lock['protocol']['data_collection'][
            'required_topics'
        ],
        'required_metadata': lock['protocol']['data_collection'][
            'required_metadata'
        ],
        'units': _expected_units(lock),
    }
    index_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = index_path.with_name(index_path.name + '.tmp')
    temporary.write_text(
        yaml.safe_dump(index, sort_keys=False, allow_unicode=True),
        encoding='utf-8',
    )
    os.replace(temporary, index_path)
    return index_path


def _relative_artifact_path(
    raw_path,
    archive_root: Path,
    description: str,
) -> tuple[str, Path]:
    if not isinstance(raw_path, str) or not raw_path.strip():
        raise ValueError(f'{description} must be a nonempty relative path')
    relative = Path(raw_path)
    if relative.is_absolute():
        raise ValueError(f'{description} must be relative')
    resolved = (archive_root / relative).resolve()
    try:
        normalized = resolved.relative_to(archive_root)
    except ValueError as error:
        raise ValueError(f'{description} escapes the archive root') from error
    return normalized.as_posix(), resolved


def _validate_index(index: dict, index_path: Path, lock: dict) -> dict:
    if index.get('schema_version') != 1:
        raise ValueError('archive index schema_version must be 1')
    archive_root = Path(
        str(index.get('archive_root', ''))
    ).expanduser().resolve()
    if archive_root != index_path.parent:
        raise ValueError('archive_root must equal the index parent directory')
    protocol = index.get('protocol')
    if not isinstance(protocol, dict):
        raise ValueError('archive index protocol reference is missing')
    expected_protocol = {
        'lock_path': str(Path(protocol['lock_path']).expanduser().resolve()),
        'lock_sha256': sha256_file(Path(protocol['lock_path'])),
        'protocol_id': lock['protocol']['protocol_id'],
        'freeze_fingerprint_sha256': lock['freeze_fingerprint_sha256'],
        'code_revision': lock['code_revision'],
    }
    normalized_protocol = {
        **protocol,
        'lock_path': expected_protocol['lock_path'],
    }
    if normalized_protocol != expected_protocol:
        raise ValueError('archive index protocol binding mismatch')
    if index.get('required_topics') != (
        lock['protocol']['data_collection']['required_topics']
    ):
        raise ValueError('archive index required topics mismatch')
    if index.get('required_metadata') != (
        lock['protocol']['data_collection']['required_metadata']
    ):
        raise ValueError('archive index required metadata mismatch')

    expected = {
        unit['unit_id']: (unit['stratum'], unit['ordinal'])
        for unit in _expected_units(lock)
    }
    units_raw = index.get('units')
    if not isinstance(units_raw, list):
        raise ValueError('archive index units must be a list')
    units = []
    seen_ids = set()
    seen_paths = set()
    for position, raw_unit in enumerate(units_raw):
        if not isinstance(raw_unit, dict):
            raise ValueError(f'archive unit[{position}] must be a mapping')
        unit_id = raw_unit.get('unit_id')
        if (
            not isinstance(unit_id, str)
            or not _UNIT_ID_PATTERN.fullmatch(unit_id)
        ):
            raise ValueError(f'archive unit[{position}] id is invalid')
        if unit_id in seen_ids:
            raise ValueError(f'duplicate archive unit id: {unit_id}')
        seen_ids.add(unit_id)
        if unit_id not in expected:
            raise ValueError(f'unplanned archive unit id: {unit_id}')
        stratum, ordinal = expected[unit_id]
        if (
            raw_unit.get('stratum') != stratum
            or raw_unit.get('ordinal') != ordinal
        ):
            raise ValueError(f'archive unit allocation mismatch: {unit_id}')
        status = raw_unit.get('status')
        if status not in _UNIT_STATUSES:
            raise ValueError(f'archive unit status is invalid: {unit_id}')
        reason = raw_unit.get('invalid_reason')
        if status == 'invalid':
            if not isinstance(reason, str) or not reason.strip():
                raise ValueError(f'invalid unit needs a reason: {unit_id}')
            reason = reason.strip()
        elif reason is not None:
            raise ValueError(
                f'non-invalid unit cannot have a reason: {unit_id}'
            )
        bag_relative, bag_path = _relative_artifact_path(
            raw_unit.get('bag_path'),
            archive_root,
            f'archive unit {unit_id} bag_path',
        )
        metadata_relative, metadata_path = _relative_artifact_path(
            raw_unit.get('metadata_path'),
            archive_root,
            f'archive unit {unit_id} metadata_path',
        )
        for artifact_path in (bag_relative, metadata_relative):
            if artifact_path in seen_paths:
                raise ValueError(
                    f'duplicate archive artifact path: {artifact_path}'
                )
            seen_paths.add(artifact_path)
        units.append({
            'unit_id': unit_id,
            'stratum': stratum,
            'ordinal': ordinal,
            'status': status,
            'invalid_reason': reason,
            'bag_relative': bag_relative,
            'bag_path': bag_path,
            'metadata_relative': metadata_relative,
            'metadata_path': metadata_path,
        })
    if seen_ids != set(expected):
        missing = sorted(set(expected) - seen_ids)
        raise ValueError(f'archive index is missing planned units: {missing}')
    return {
        'archive_root': archive_root,
        'protocol': expected_protocol,
        'required_topics': index['required_topics'],
        'required_metadata': index['required_metadata'],
        'units': units,
    }


def load_validated_archive_index(
    index_path: Path,
    workspace_root: Path,
) -> tuple[dict, dict]:
    """Load an archive index after verifying its frozen protocol binding."""
    workspace_root = workspace_root.expanduser().resolve()
    index_path = index_path.expanduser().resolve()
    _outside_workspace(index_path, workspace_root)
    index = _load_mapping(index_path, 'archive index')
    protocol_reference = index.get('protocol')
    if not isinstance(protocol_reference, dict):
        raise ValueError('archive index protocol reference is missing')
    lock_path = Path(str(protocol_reference.get('lock_path', '')))
    verify_protocol_lock(lock_path, workspace_root)
    lock = _load_mapping(lock_path, 'protocol lock')
    return _validate_index(index, index_path, lock), lock


def _directory_inventory(directory: Path) -> dict:
    if not directory.is_dir():
        raise ValueError('Rosbag path is not a directory')
    files = []
    total_size = 0
    for path in sorted(directory.rglob('*')):
        if path.is_symlink():
            raise ValueError(f'Rosbag archive contains a symlink: {path}')
        if not path.is_file():
            continue
        relative = path.relative_to(directory).as_posix()
        size = path.stat().st_size
        total_size += size
        files.append({
            'path': relative,
            'size_bytes': size,
            'sha256': sha256_file(path),
        })
    if not files:
        raise ValueError('Rosbag directory contains no files')
    return {
        'file_count': len(files),
        'total_size_bytes': total_size,
        'inventory_sha256': _canonical_sha256(files),
        'files': files,
    }


def _sqlite_topic_counts(storage_paths: list[Path]) -> dict[str, int]:
    counts = {}
    for storage_path in storage_paths:
        if storage_path.suffix.lower() != '.db3':
            raise ValueError('sqlite3 Rosbag storage file must end in .db3')
        uri = f'file:{quote(str(storage_path), safe="/")}?mode=ro'
        try:
            connection = sqlite3.connect(uri, uri=True)
            try:
                connection.execute('PRAGMA query_only = ON')
                integrity = connection.execute(
                    'PRAGMA quick_check'
                ).fetchall()
                if integrity != [('ok',)]:
                    raise ValueError('Rosbag SQLite quick_check failed')
                rows = connection.execute(
                    'SELECT topics.name, COUNT(messages.id) '
                    'FROM topics LEFT JOIN messages '
                    'ON messages.topic_id = topics.id '
                    'GROUP BY topics.id, topics.name'
                ).fetchall()
            finally:
                connection.close()
        except sqlite3.Error as error:
            raise ValueError(
                f'Rosbag SQLite storage is invalid: {storage_path.name}'
            ) from error
        for name, message_count in rows:
            if not isinstance(name, str) or not name:
                raise ValueError('Rosbag SQLite topic name is invalid')
            counts[name] = counts.get(name, 0) + int(message_count)
    return counts


def _rosbag_topics(bag_path: Path, required_topics: list[str]) -> dict:
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.is_file():
        raise ValueError('Rosbag metadata.yaml is missing')
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    try:
        information = metadata['rosbag2_bagfile_information']
        topics_raw = information['topics_with_message_count']
        relative_files = information['relative_file_paths']
        storage_identifier = information['storage_identifier']
    except (KeyError, TypeError) as error:
        raise ValueError(
            'Rosbag metadata.yaml structure is invalid'
        ) from error
    if (
        not isinstance(topics_raw, list)
        or not isinstance(relative_files, list)
    ):
        raise ValueError('Rosbag metadata topic or file list is invalid')
    topics = {}
    for item in topics_raw:
        try:
            name = item['topic_metadata']['name']
            message_count = int(item['message_count'])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError('Rosbag topic metadata is invalid') from error
        if not isinstance(name, str) or not name or message_count < 0:
            raise ValueError('Rosbag topic metadata is invalid')
        if name in topics:
            raise ValueError(f'duplicate Rosbag topic metadata: {name}')
        topics[name] = message_count
    missing_topics = [
        topic for topic in required_topics
        if topics.get(topic, 0) < 1
    ]
    if missing_topics:
        raise ValueError(
            f'Rosbag required topics are missing: {missing_topics}'
        )
    storage_paths = []
    for relative in relative_files:
        _, resolved = _relative_artifact_path(
            relative,
            bag_path,
            'Rosbag relative_file_paths entry',
        )
        if not resolved.is_file():
            raise ValueError(f'Rosbag storage file is missing: {relative}')
        storage_paths.append(resolved)
    if len(set(storage_paths)) != len(storage_paths):
        raise ValueError('Rosbag storage file list contains duplicates')
    if storage_identifier != 'sqlite3':
        raise ValueError(
            f'unsupported Rosbag storage identifier: {storage_identifier}'
        )
    declared_storage = set(storage_paths)
    discovered_storage = set(bag_path.glob('*.db3'))
    if declared_storage != discovered_storage:
        raise ValueError('Rosbag SQLite storage file list is incomplete')
    actual_topics = _sqlite_topic_counts(storage_paths)
    if actual_topics != topics:
        raise ValueError(
            'Rosbag metadata topic counts disagree with SQLite storage'
        )
    return topics


def _rosbag_duration_s(bag_path: Path) -> float | None:
    metadata = yaml.safe_load(
        (bag_path / 'metadata.yaml').read_text(encoding='utf-8')
    )
    try:
        nanoseconds = metadata['rosbag2_bagfile_information'][
            'duration'
        ]['nanoseconds']
    except (KeyError, TypeError):
        return None
    if (
        isinstance(nanoseconds, bool)
        or not isinstance(nanoseconds, int)
        or nanoseconds < 0
    ):
        raise ValueError('Rosbag duration metadata is invalid')
    return nanoseconds * 1.0e-9


def inspect_recorded_bag(
    bag_path: Path,
    required_topics: list[str],
) -> dict:
    """Return a verified SQLite Rosbag inventory and actual topic counts."""
    bag_path = bag_path.expanduser().resolve()
    inventory = _directory_inventory(bag_path)
    topics = _rosbag_topics(bag_path, required_topics)
    return {
        'path': str(bag_path),
        'topics': topics,
        'duration_s': _rosbag_duration_s(bag_path),
        **inventory,
    }


def _validate_unit_metadata(
    path: Path,
    required: list[str],
    requirements: dict | None = None,
) -> dict:
    if not path.is_file():
        raise ValueError('unit metadata JSON is missing')
    metadata = _load_mapping(path, 'unit metadata')
    missing = [name for name in required if name not in metadata]
    if missing:
        raise ValueError(f'unit metadata is missing fields: {missing}')
    timestamps = None
    for name in required:
        value = metadata[name]
        if name == 'camera_model_and_serial':
            if not isinstance(value, str) or not value.strip():
                raise ValueError(
                    'unit metadata camera_model_and_serial must be a string'
                )
        elif name.endswith('_sha256'):
            if (
                not isinstance(value, str)
                or not _SHA256_PATTERN.fullmatch(value)
            ):
                raise ValueError(f'unit metadata {name} is not SHA-256')
        elif name == 'measured_person_pose_in_map':
            if not isinstance(value, dict):
                raise ValueError(
                    'measured_person_pose_in_map must be a mapping'
                )
            for coordinate in ('x', 'y'):
                coordinate_value = value.get(coordinate)
                if (
                    isinstance(coordinate_value, bool)
                    or not isinstance(coordinate_value, (int, float))
                    or not math.isfinite(coordinate_value)
                ):
                    raise ValueError(
                        f'measured_person_pose_in_map.{coordinate} is invalid'
                    )
            pose_requirement = (requirements or {}).get(
                'measured_person_pose'
            )
            if pose_requirement is not None:
                if value.get('frame_id') != pose_requirement['frame_id']:
                    raise ValueError(
                        'measured_person_pose_in_map.frame_id is invalid'
                    )
                method = value.get('measurement_method')
                if not isinstance(method, str) or not method.strip():
                    raise ValueError(
                        'measured_person_pose_in_map.measurement_method '
                        'must be a string'
                    )
                uncertainty = value.get('uncertainty_m')
                if (
                    isinstance(uncertainty, bool)
                    or not isinstance(uncertainty, (int, float))
                    or not math.isfinite(uncertainty)
                    or uncertainty < 0.0
                    or uncertainty > pose_requirement[
                        'maximum_uncertainty_m'
                    ]
                ):
                    raise ValueError(
                        'measured_person_pose_in_map.uncertainty_m exceeds '
                        'the protocol maximum'
                    )
                if value.get('measured_before_recording') is not True:
                    raise ValueError(
                        'person pose was not measured before recording'
                    )
        elif name == 'recording_start_and_end_utc':
            if not isinstance(value, dict):
                raise ValueError(
                    'recording_start_and_end_utc must be a mapping'
                )
            timestamps = {}
            for boundary in ('start_utc', 'end_utc'):
                raw_timestamp = value.get(boundary)
                if not isinstance(raw_timestamp, str) or not raw_timestamp:
                    raise ValueError(
                        f'recording_start_and_end_utc.{boundary} is invalid'
                    )
                try:
                    timestamp = datetime.fromisoformat(
                        raw_timestamp.replace('Z', '+00:00')
                    )
                except ValueError as error:
                    raise ValueError(
                        f'recording_start_and_end_utc.{boundary} is invalid'
                    ) from error
                if timestamp.utcoffset() != timedelta(0):
                    raise ValueError(
                        f'recording_start_and_end_utc.{boundary} must be UTC'
                    )
                timestamps[boundary] = timestamp
            if timestamps['end_utc'] <= timestamps['start_utc']:
                raise ValueError(
                    'recording end_utc must be later than start_utc'
                )
        elif name == 'observation_conditions':
            if not isinstance(value, dict):
                raise ValueError(
                    'observation_conditions must be a mapping'
                )
            observation = (requirements or {}).get('observation')
            if observation is not None and value != observation:
                raise ValueError(
                    'observation_conditions violate the protocol'
                )
        elif value is None or value == '' or value == [] or value == {}:
            raise ValueError(f'unit metadata {name} must not be empty')
    duration_requirement = (requirements or {}).get('recording_duration_s')
    if duration_requirement is not None:
        if timestamps is None:
            raise ValueError('recording timestamps are required')
        duration = (
            timestamps['end_utc'] - timestamps['start_utc']
        ).total_seconds()
        if not (
            duration_requirement['minimum']
            <= duration
            <= duration_requirement['maximum']
        ):
            raise ValueError(
                'recording duration violates the protocol: '
                f'{duration:.3f} s'
            )
    return {
        'path': path.name,
        'sha256': sha256_file(path),
        'values': {name: metadata[name] for name in required},
    }


def _audit_collected_unit(
    unit: dict,
    required_topics,
    required_metadata,
    requirements: dict | None = None,
) -> dict:
    bag = inspect_recorded_bag(unit['bag_path'], required_topics)
    metadata = _validate_unit_metadata(
        unit['metadata_path'],
        required_metadata,
        requirements,
    )
    duration_requirement = (requirements or {}).get('recording_duration_s')
    if duration_requirement is not None:
        duration = bag['duration_s']
        if duration is None:
            raise ValueError('Rosbag duration metadata is missing')
        if not (
            duration_requirement['minimum']
            <= duration
            <= duration_requirement['maximum']
        ):
            raise ValueError(
                'Rosbag duration violates the protocol: '
                f'{duration:.3f} s'
            )
    return {
        'bag': {
            **bag,
            'path': unit['bag_relative'],
            'topics': {
                topic: bag['topics'][topic]
                for topic in required_topics
            },
        },
        'metadata': {
            **metadata,
            'path': unit['metadata_relative'],
        },
    }


def validate_collected_unit_artifacts(
    unit: dict,
    required_topics: list[str],
    required_metadata: list[str],
    metadata_path: Path | None = None,
    requirements: dict | None = None,
) -> dict:
    """Validate one bag and metadata file without changing the archive."""
    candidate = dict(unit)
    if metadata_path is not None:
        candidate['metadata_path'] = metadata_path.expanduser().resolve()
    return _audit_collected_unit(
        candidate,
        required_topics,
        required_metadata,
        requirements,
    )


def _preserved_invalid_artifacts(unit: dict) -> dict | None:
    artifacts = {}
    bag_path = unit['bag_path']
    if bag_path.exists():
        try:
            artifacts['bag'] = {
                'path': unit['bag_relative'],
                **_directory_inventory(bag_path),
            }
        except (OSError, ValueError) as error:
            artifacts['bag'] = {
                'path': unit['bag_relative'],
                'preservation_error': str(error),
            }
    metadata_path = unit['metadata_path']
    if metadata_path.is_file():
        artifacts['metadata'] = {
            'path': unit['metadata_relative'],
            'sha256': sha256_file(metadata_path),
            'size_bytes': metadata_path.stat().st_size,
        }
    return artifacts or None


def audit_archive_index(
    index_path: Path,
    workspace_root: Path,
) -> dict:
    """Audit all planned archive units without modifying archive contents."""
    index_path = index_path.expanduser().resolve()
    validated, lock = load_validated_archive_index(
        index_path,
        workspace_root,
    )

    results = []
    for unit in validated['units']:
        result = {
            'unit_id': unit['unit_id'],
            'stratum': unit['stratum'],
            'declared_status': unit['status'],
            'audit_status': 'missing',
            'reason': None,
            'artifacts': None,
        }
        if unit['status'] == 'planned':
            if unit['bag_path'].exists() or unit['metadata_path'].exists():
                result['audit_status'] = 'invalid'
                result['reason'] = (
                    'unit artifacts exist but status is still planned'
                )
                result['artifacts'] = _preserved_invalid_artifacts(unit)
            else:
                result['reason'] = 'unit has not been marked collected'
        elif unit['status'] == 'invalid':
            result['audit_status'] = 'invalid'
            result['reason'] = unit['invalid_reason']
            result['artifacts'] = _preserved_invalid_artifacts(unit)
        else:
            try:
                result['artifacts'] = _audit_collected_unit(
                    unit,
                    validated['required_topics'],
                    validated['required_metadata'],
                    lock['protocol']['data_collection'].get('requirements'),
                )
                result['audit_status'] = 'passed'
            except (
                OSError,
                KeyError,
                TypeError,
                ValueError,
                json.JSONDecodeError,
                yaml.YAMLError,
            ) as error:
                result['audit_status'] = 'invalid'
                result['reason'] = str(error)
        results.append(result)
    counts = {
        status: sum(item['audit_status'] == status for item in results)
        for status in ('passed', 'missing', 'invalid')
    }
    return {
        'schema_version': 1,
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'protocol_id': lock['protocol']['protocol_id'],
        'freeze_fingerprint_sha256': lock['freeze_fingerprint_sha256'],
        'code_revision': lock['code_revision'],
        'archive_root': str(validated['archive_root']),
        'index': {
            'path': str(index_path),
            'sha256': sha256_file(index_path),
        },
        'unit_count': len(results),
        'status_counts': counts,
        'analysis_ready': counts == {
            'passed': len(results),
            'missing': 0,
            'invalid': 0,
        },
        'units': results,
    }


def write_archive_audit(audit: dict, output_path: Path) -> Path:
    """Write an archive audit atomically without changing source artifacts."""
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(output_path.name + '.tmp')
    temporary.write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary, output_path)
    return output_path


def main(args=None) -> None:
    """Initialize an archive index or write its completeness audit."""
    parser = argparse.ArgumentParser(
        description='Initialize or audit an external experiment archive.',
    )
    parser.add_argument('--workspace-root', type=Path, default=Path.cwd())
    parser.add_argument('--protocol-lock', required=True, type=Path)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--initialize-index', type=Path)
    modes.add_argument('--audit-index', type=Path)
    parser.add_argument('--output', type=Path)
    parsed = parser.parse_args(args)
    try:
        if parsed.initialize_index is not None:
            if parsed.output is not None:
                raise ValueError('--output is only valid with --audit-index')
            index_path = initialize_archive_index(
                parsed.protocol_lock,
                parsed.workspace_root,
                parsed.initialize_index,
            )
            print(f'archive index initialized; output={index_path}')
            return
        if parsed.output is None:
            raise ValueError('--output is required with --audit-index')
        _outside_workspace(parsed.output, parsed.workspace_root)
        index = _load_mapping(parsed.audit_index, 'archive index')
        reference = index.get('protocol')
        if not isinstance(reference, dict):
            raise ValueError('archive index protocol reference is missing')
        supplied_lock = parsed.protocol_lock.expanduser().resolve()
        indexed_lock = Path(
            str(reference.get('lock_path', ''))
        ).expanduser().resolve()
        if supplied_lock != indexed_lock:
            raise ValueError(
                '--protocol-lock does not match the archive index'
            )
        audit = audit_archive_index(
            parsed.audit_index,
            parsed.workspace_root,
        )
        output_path = write_archive_audit(audit, parsed.output)
    except (
        OSError,
        KeyError,
        TypeError,
        ValueError,
        json.JSONDecodeError,
        yaml.YAMLError,
    ) as error:
        print(f'cannot audit archive: {error}', file=sys.stderr)
        raise SystemExit(2)
    print(
        'archive audited; passed=%d; missing=%d; invalid=%d; ready=%s; '
        'output=%s'
        % (
            audit['status_counts']['passed'],
            audit['status_counts']['missing'],
            audit['status_counts']['invalid'],
            audit['analysis_ready'],
            output_path,
        )
    )
    if not audit['analysis_ready']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
