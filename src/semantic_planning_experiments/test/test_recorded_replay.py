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

from pathlib import Path
import sqlite3

import pytest
import yaml

from semantic_planning_experiments.recorded_replay import (
    build_recorded_replay,
)
from semantic_planning_experiments.recorded_replay import (
    RECORDED_RGBD_TOPICS,
)


def _bag(tmp_path: Path, topics=None, messages_per_topic: int = 2) -> Path:
    bag = tmp_path / 'archive/bags/person_near_001'
    bag.mkdir(parents=True)
    topics = list(topics or RECORDED_RGBD_TOPICS)
    storage = bag / 'recording_0.db3'
    connection = sqlite3.connect(storage)
    connection.execute(
        'CREATE TABLE topics('
        'id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL)'
    )
    connection.execute(
        'CREATE TABLE messages('
        'id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
        'timestamp INTEGER NOT NULL, data BLOB NOT NULL)'
    )
    for topic_id, topic in enumerate(topics, start=1):
        connection.execute(
            'INSERT INTO topics(id, name, type) VALUES (?, ?, ?)',
            (topic_id, topic, 'test/msg/Type'),
        )
        for message_index in range(messages_per_topic):
            connection.execute(
                'INSERT INTO messages(topic_id, timestamp, data) '
                'VALUES (?, ?, ?)',
                (topic_id, message_index, b'test'),
            )
    connection.commit()
    connection.close()
    (bag / 'metadata.yaml').write_text(
        yaml.safe_dump({
            'rosbag2_bagfile_information': {
                'storage_identifier': 'sqlite3',
                'relative_file_paths': ['recording_0.db3'],
                'topics_with_message_count': [
                    {
                        'topic_metadata': {
                            'name': topic,
                            'type': 'test/msg/Type',
                        },
                        'message_count': messages_per_topic,
                    }
                    for topic in topics
                ],
            },
        }),
        encoding='utf-8',
    )
    return bag


def test_replay_builds_exact_safe_command_from_verified_bag(tmp_path: Path):
    workspace = tmp_path / 'repo'
    workspace.mkdir()
    bag = _bag(tmp_path)

    replay = build_recorded_replay(
        bag,
        'person_near_001',
        workspace,
        playback_rate=0.5,
        playback_start_delay_seconds=2.0,
        runner_delay_seconds=8.0,
    )

    assert replay['command_argv'] == [
        'ros2',
        'bag',
        'play',
        str(bag),
        '--rate',
        '0.5',
        '--topics',
        *RECORDED_RGBD_TOPICS,
    ]
    assert replay['bag']['topics'] == {
        topic: 2 for topic in RECORDED_RGBD_TOPICS
    }
    assert replay['bag']['file_count'] == 2
    assert replay['safety_scope']['controller_started'] is False
    assert replay['safety_scope']['hardware_nodes_started'] is False


def test_replay_rejects_bag_inside_repository(tmp_path: Path):
    workspace = tmp_path / 'repo'
    workspace.mkdir()
    bag = _bag(workspace)

    with pytest.raises(ValueError, match='outside the repository'):
        build_recorded_replay(bag, 'near_001', workspace, 1.0, 2.0, 8.0)


def test_replay_rejects_missing_protocol_topic(tmp_path: Path):
    workspace = tmp_path / 'repo'
    workspace.mkdir()
    bag = _bag(tmp_path, RECORDED_RGBD_TOPICS[:-1])

    with pytest.raises(ValueError, match='required topics are missing'):
        build_recorded_replay(bag, 'near_001', workspace, 1.0, 2.0, 8.0)


@pytest.mark.parametrize(
    'rate, playback_delay, runner_delay, message',
    [
        (0.0, 2.0, 8.0, 'playback_rate'),
        (1.0, -1.0, 8.0, 'playback_start_delay_seconds'),
        (1.0, 8.0, 8.0, 'runner_delay_seconds'),
    ],
)
def test_replay_rejects_unsafe_timing(
    tmp_path: Path,
    rate: float,
    playback_delay: float,
    runner_delay: float,
    message: str,
):
    workspace = tmp_path / 'repo'
    workspace.mkdir()
    bag = _bag(tmp_path)

    with pytest.raises(ValueError, match=message):
        build_recorded_replay(
            bag,
            'near_001',
            workspace,
            rate,
            playback_delay,
            runner_delay,
        )
