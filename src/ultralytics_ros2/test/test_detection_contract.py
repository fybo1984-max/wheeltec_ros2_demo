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

import math

import pytest
from std_msgs.msg import Header

from ultralytics_ros2.detection_contract import build_detection_array
from ultralytics_ros2.detection_contract import DetectionCandidate


def _candidate(class_id='person', score=0.9):
    return DetectionCandidate(
        class_id=class_id,
        score=score,
        center_x=320.0,
        center_y=240.0,
        size_x=80.0,
        size_y=200.0,
    )


def test_person_detection_preserves_image_stamp_frame_and_geometry():
    header = Header()
    header.stamp.sec = 12
    header.stamp.nanosec = 345
    header.frame_id = 'camera_color_optical_frame'

    message = build_detection_array(
        header,
        [_candidate()],
        ['person'],
        0.5,
    )

    assert message.header == header
    assert len(message.detections) == 1
    detection = message.detections[0]
    assert detection.header == header
    assert detection.bbox.center.position.x == 320.0
    assert detection.bbox.center.position.y == 240.0
    assert detection.bbox.size_x == 80.0
    assert detection.bbox.size_y == 200.0
    assert detection.results[0].hypothesis.class_id == 'person'
    assert detection.results[0].hypothesis.score == pytest.approx(0.9)


def test_allowlist_and_confidence_remove_irrelevant_detections():
    message = build_detection_array(
        Header(),
        [
            _candidate('person', 0.95),
            _candidate('person', 0.4),
            _candidate('chair', 0.99),
        ],
        ['PERSON'],
        0.5,
    )

    assert len(message.detections) == 1
    assert message.detections[0].results[0].hypothesis.class_id == 'person'


@pytest.mark.parametrize(
    'candidate',
    [
        _candidate('', 0.9),
        _candidate('person', -0.1),
        DetectionCandidate('person', 0.9, 1.0, 1.0, 0.0, 2.0),
        DetectionCandidate('person', 0.9, math.nan, 1.0, 2.0, 2.0),
    ],
)
def test_invalid_candidates_fail_visibly(candidate):
    with pytest.raises(ValueError):
        build_detection_array(Header(), [candidate], ['person'], 0.5)


def test_invalid_confidence_threshold_is_rejected():
    with pytest.raises(ValueError, match='minimum_confidence'):
        build_detection_array(Header(), [], ['person'], math.nan)
