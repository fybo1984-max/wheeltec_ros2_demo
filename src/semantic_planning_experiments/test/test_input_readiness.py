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

from copy import deepcopy
from types import SimpleNamespace

import pytest

from semantic_planning_experiments.input_readiness import (
    LiveInputReadiness,
    evaluate_input_readiness,
)


def _ready_snapshot() -> dict:
    return {
        'message_counts': {
            'color': 5,
            'depth': 5,
            'camera_info': 5,
            'detections': 5,
        },
        'color': {
            'width': 640,
            'height': 480,
            'encoding': 'bgr8',
            'frame_id': 'camera_color_optical_frame',
        },
        'depth': {
            'width': 640,
            'height': 480,
            'encoding': '16UC1',
            'frame_id': 'camera_color_optical_frame',
        },
        'camera_info': {
            'width': 640,
            'height': 480,
            'fx': 525.0,
            'fy': 525.0,
            'frame_id': 'camera_color_optical_frame',
        },
        'detections': {
            'frame_id': 'camera_color_optical_frame',
            'target_class_id': 'person',
            'target_detection_count': 3,
            'maximum_target_confidence': 0.92,
        },
        'maximum_nearest_detection_depth_delta_sec': 0.03,
        'transform': {
            'target_frame': 'map',
            'source_frame': 'camera_color_optical_frame',
            'available': True,
        },
    }


def test_ready_snapshot_passes_every_behavior_check():
    result = evaluate_input_readiness(_ready_snapshot(), 3, 0.15)

    assert result['experiment_input_ready']
    assert all(result['checks'].values())
    assert result['failures'] == []


@pytest.mark.parametrize(
    ('mutation', 'failed_check'),
    [
        (lambda item: item['message_counts'].__setitem__('depth', 2),
         'depth_messages'),
        (lambda item: item['detections'].__setitem__(
            'target_detection_count', 0), 'target_class_detected'),
        (lambda item: item['depth'].__setitem__('encoding', 'mono8'),
         'depth_encoding_supported'),
        (lambda item: item['depth'].__setitem__('width', 320),
         'registered_dimensions_match'),
        (lambda item: item['depth'].__setitem__(
            'frame_id', 'camera_depth_optical_frame'),
         'registered_frames_match'),
        (lambda item: item.__setitem__(
            'maximum_nearest_detection_depth_delta_sec', 0.2),
         'detection_depth_synchronized'),
        (lambda item: item['transform'].__setitem__('available', False),
         'map_transform_available'),
    ],
)
def test_invalid_live_input_fails_visibly(mutation, failed_check):
    snapshot = deepcopy(_ready_snapshot())
    mutation(snapshot)

    result = evaluate_input_readiness(snapshot, 3, 0.15)

    assert not result['experiment_input_ready']
    assert failed_check in result['failures']


def test_invalid_requirements_are_rejected():
    with pytest.raises(ValueError, match='minimum_messages'):
        evaluate_input_readiness(_ready_snapshot(), 0, 0.15)
    with pytest.raises(ValueError, match='maximum_sync_delta_sec'):
        evaluate_input_readiness(_ready_snapshot(), 3, float('nan'))


def test_only_qualified_target_frames_enter_sync_window():
    state = SimpleNamespace(
        _counts={'detections': 0},
        _detections={
            'frame_id': '',
            'target_detection_count': 0,
            'maximum_target_confidence': None,
        },
        _detection_stamps=[],
        _target_class='person',
        _minimum_confidence=0.8,
    )

    def message(stamp, results):
        return SimpleNamespace(
            header=SimpleNamespace(
                frame_id='camera_color_optical_frame',
                stamp=SimpleNamespace(sec=stamp, nanosec=0),
            ),
            detections=[SimpleNamespace(results=results)],
        )

    below_threshold = SimpleNamespace(
        hypothesis=SimpleNamespace(class_id='person', score=0.7),
    )
    qualified = SimpleNamespace(
        hypothesis=SimpleNamespace(class_id='person', score=0.9),
    )
    LiveInputReadiness._on_detections(state, message(10, []))
    LiveInputReadiness._on_detections(
        state, message(11, [below_threshold]))
    LiveInputReadiness._on_detections(state, message(12, [qualified]))

    assert state._counts['detections'] == 3
    assert state._detections['target_detection_count'] == 1
    assert state._detections['maximum_target_confidence'] == 0.9
    assert state._detection_stamps == [12.0]
