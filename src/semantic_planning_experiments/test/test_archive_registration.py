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

import json
from pathlib import Path

import pytest
import yaml

from semantic_planning_experiments import archive_registration
from semantic_planning_experiments.archive_registration import (
    register_collected_unit,
)
from semantic_planning_experiments.metrics import sha256_file


_TOPICS = ['/rgb', '/depth', '/tf']
_METADATA = [
    'camera_model_and_serial',
    'detector_model_sha256',
    'map_yaml_sha256',
    'measured_person_pose_in_map',
    'recording_start_and_end_utc',
]


def _metadata() -> dict:
    return {
        'camera_model_and_serial': 'D435-TEST',
        'detector_model_sha256': 'a' * 64,
        'map_yaml_sha256': 'b' * 64,
        'measured_person_pose_in_map': {
            'x': 1.0,
            'y': 2.0,
            'frame_id': 'map',
        },
        'recording_start_and_end_utc': {
            'start_utc': '2026-08-03T00:00:00Z',
            'end_utc': '2026-08-03T00:00:10Z',
        },
    }


def _inputs(tmp_path: Path, monkeypatch) -> dict:
    workspace = tmp_path / 'repo'
    archive = tmp_path / 'archive'
    workspace.mkdir()
    archive.mkdir()
    bag = archive / 'bags/near_001'
    bag.mkdir(parents=True)
    (bag / 'recording_0.db3').write_bytes(b'unchanged rosbag payload')
    topic_metadata = [
        {
            'topic_metadata': {
                'name': topic,
                'type': 'test/msg/Type',
            },
            'message_count': 5,
        }
        for topic in _TOPICS
    ]
    (bag / 'metadata.yaml').write_text(
        yaml.safe_dump({
            'rosbag2_bagfile_information': {
                'topics_with_message_count': topic_metadata,
                'relative_file_paths': ['recording_0.db3'],
            },
        }),
        encoding='utf-8',
    )
    index_path = archive / 'index.yaml'
    index_path.write_text(
        yaml.safe_dump({
            'schema_version': 1,
            'units': [{
                'unit_id': 'near_001',
                'stratum': 'near',
                'ordinal': 1,
                'status': 'planned',
                'bag_path': 'bags/near_001',
                'metadata_path': 'metadata/near_001.json',
                'invalid_reason': None,
            }],
        }, sort_keys=False),
        encoding='utf-8',
    )
    metadata_input = tmp_path / 'staging/near_001.json'
    metadata_input.parent.mkdir()
    metadata_input.write_text(
        json.dumps(_metadata()),
        encoding='utf-8',
    )
    unit = {
        'unit_id': 'near_001',
        'stratum': 'near',
        'ordinal': 1,
        'status': 'planned',
        'invalid_reason': None,
        'bag_relative': 'bags/near_001',
        'bag_path': bag,
        'metadata_relative': 'metadata/near_001.json',
        'metadata_path': archive / 'metadata/near_001.json',
    }
    validated = {
        'archive_root': archive,
        'required_topics': list(_TOPICS),
        'required_metadata': list(_METADATA),
        'units': [unit],
    }
    lock = {
        'freeze_fingerprint_sha256': 'c' * 64,
        'code_revision': 'd' * 40,
        'protocol': {'protocol_id': 'recorded_rgbd_pilot_v1'},
    }
    monkeypatch.setattr(
        archive_registration,
        'load_validated_archive_index',
        lambda index, root: (validated, lock),
    )
    return {
        'workspace': workspace,
        'archive': archive,
        'index': index_path,
        'metadata_input': metadata_input,
        'receipt': archive / 'receipts/near_001.json',
        'validated': validated,
        'bag_data': bag / 'recording_0.db3',
    }


def _register(inputs: dict) -> dict:
    return register_collected_unit(
        inputs['index'],
        'near_001',
        inputs['metadata_input'],
        inputs['receipt'],
        inputs['workspace'],
    )


def test_registration_validates_and_closes_the_archive_unit(
    tmp_path: Path,
    monkeypatch,
):
    inputs = _inputs(tmp_path, monkeypatch)
    bag_sha256 = sha256_file(inputs['bag_data'])

    receipt = _register(inputs)

    index = yaml.safe_load(inputs['index'].read_text(encoding='utf-8'))
    archived_metadata = (
        inputs['archive'] / 'metadata/near_001.json'
    )
    assert index['units'][0]['status'] == 'collected'
    assert archived_metadata.is_file()
    assert inputs['metadata_input'].is_file()
    assert inputs['receipt'].is_file()
    assert receipt['registration_status'] == 'collected'
    assert not receipt['commands_executed']
    assert receipt['artifacts']['bag']['topics'] == {
        topic: 5 for topic in _TOPICS
    }
    assert sha256_file(inputs['bag_data']) == bag_sha256
    assert receipt['index']['after_sha256'] == sha256_file(inputs['index'])


def test_registration_rejects_unknown_unit(tmp_path: Path, monkeypatch):
    inputs = _inputs(tmp_path, monkeypatch)

    with pytest.raises(ValueError, match='does not exist'):
        register_collected_unit(
            inputs['index'],
            'far_001',
            inputs['metadata_input'],
            inputs['receipt'],
            inputs['workspace'],
        )


def test_registration_rejects_nonplanned_unit(tmp_path: Path, monkeypatch):
    inputs = _inputs(tmp_path, monkeypatch)
    inputs['validated']['units'][0]['status'] = 'collected'

    with pytest.raises(ValueError, match='not planned'):
        _register(inputs)


def test_registration_rejects_existing_archive_metadata(
    tmp_path: Path,
    monkeypatch,
):
    inputs = _inputs(tmp_path, monkeypatch)
    target = inputs['archive'] / 'metadata/near_001.json'
    target.parent.mkdir()
    target.write_text('{}\n', encoding='utf-8')

    with pytest.raises(ValueError, match='already exists'):
        _register(inputs)


def test_registration_rejects_missing_required_topic(
    tmp_path: Path,
    monkeypatch,
):
    inputs = _inputs(tmp_path, monkeypatch)
    bag_metadata_path = inputs['archive'] / 'bags/near_001/metadata.yaml'
    bag_metadata = yaml.safe_load(bag_metadata_path.read_text(encoding='utf-8'))
    bag_metadata['rosbag2_bagfile_information'][
        'topics_with_message_count'
    ].pop()
    bag_metadata_path.write_text(
        yaml.safe_dump(bag_metadata),
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='required topics are missing'):
        _register(inputs)
    index = yaml.safe_load(inputs['index'].read_text(encoding='utf-8'))
    assert index['units'][0]['status'] == 'planned'
    assert not (inputs['archive'] / 'metadata/near_001.json').exists()


def test_registration_rejects_invalid_metadata_before_writing(
    tmp_path: Path,
    monkeypatch,
):
    inputs = _inputs(tmp_path, monkeypatch)
    metadata = _metadata()
    del metadata['map_yaml_sha256']
    inputs['metadata_input'].write_text(
        json.dumps(metadata),
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='missing fields'):
        _register(inputs)
    assert not (inputs['archive'] / 'metadata/near_001.json').exists()
    assert not inputs['receipt'].exists()


def test_registration_rejects_reversed_recording_times(
    tmp_path: Path,
    monkeypatch,
):
    inputs = _inputs(tmp_path, monkeypatch)
    metadata = _metadata()
    metadata['recording_start_and_end_utc'] = {
        'start_utc': '2026-08-03T00:00:10Z',
        'end_utc': '2026-08-03T00:00:00Z',
    }
    inputs['metadata_input'].write_text(
        json.dumps(metadata),
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='later than start_utc'):
        _register(inputs)
    assert not (inputs['archive'] / 'metadata/near_001.json').exists()


def test_registration_rejects_metadata_inside_repository(
    tmp_path: Path,
    monkeypatch,
):
    inputs = _inputs(tmp_path, monkeypatch)
    metadata_input = inputs['workspace'] / 'metadata.json'
    metadata_input.write_text(json.dumps(_metadata()), encoding='utf-8')

    with pytest.raises(ValueError, match='outside the repository'):
        register_collected_unit(
            inputs['index'],
            'near_001',
            metadata_input,
            inputs['receipt'],
            inputs['workspace'],
        )


def test_registration_rejects_receipt_inside_repository(
    tmp_path: Path,
    monkeypatch,
):
    inputs = _inputs(tmp_path, monkeypatch)

    with pytest.raises(ValueError, match='outside the repository'):
        register_collected_unit(
            inputs['index'],
            'near_001',
            inputs['metadata_input'],
            inputs['workspace'] / 'receipt.json',
            inputs['workspace'],
        )


def test_registration_never_overwrites_receipt(tmp_path: Path, monkeypatch):
    inputs = _inputs(tmp_path, monkeypatch)
    inputs['receipt'].parent.mkdir()
    inputs['receipt'].write_text('keep me\n', encoding='utf-8')

    with pytest.raises(ValueError, match='receipt already exists'):
        _register(inputs)
    assert inputs['receipt'].read_text(encoding='utf-8') == 'keep me\n'
