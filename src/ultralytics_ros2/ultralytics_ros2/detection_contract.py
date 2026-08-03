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

"""Convert detector-independent candidates to ROS vision messages."""

from dataclasses import dataclass
import math

from std_msgs.msg import Header
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose


@dataclass(frozen=True)
class DetectionCandidate:
    """One class-labelled 2-D bounding box in image pixels."""

    class_id: str
    score: float
    center_x: float
    center_y: float
    size_x: float
    size_y: float

    def validate(self) -> None:
        """Reject values that cannot form a meaningful detection."""
        if not self.class_id.strip():
            raise ValueError('detection class_id must not be empty')
        values = (
            self.score,
            self.center_x,
            self.center_y,
            self.size_x,
            self.size_y,
        )
        if any(not math.isfinite(value) for value in values):
            raise ValueError('detection values must be finite')
        if not 0.0 <= self.score <= 1.0:
            raise ValueError('detection score must be in [0, 1]')
        if self.size_x <= 0.0 or self.size_y <= 0.0:
            raise ValueError('detection bounding-box size must be positive')


def build_detection_array(
    header: Header,
    candidates: list[DetectionCandidate],
    allowed_class_names: list[str],
    minimum_confidence: float,
) -> Detection2DArray:
    """Build a filtered Detection2DArray while preserving image metadata."""
    if not math.isfinite(minimum_confidence) or not (
        0.0 <= minimum_confidence <= 1.0
    ):
        raise ValueError('minimum_confidence must be in [0, 1]')
    allowed = {
        str(name).strip().casefold()
        for name in allowed_class_names
        if str(name).strip()
    }
    output = Detection2DArray()
    output.header = header
    for candidate in candidates:
        candidate.validate()
        if candidate.score < minimum_confidence:
            continue
        if allowed and candidate.class_id.casefold() not in allowed:
            continue
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = candidate.class_id
        hypothesis.hypothesis.score = candidate.score
        detection = Detection2D()
        detection.header = header
        detection.bbox.center.position.x = candidate.center_x
        detection.bbox.center.position.y = candidate.center_y
        detection.bbox.size_x = candidate.size_x
        detection.bbox.size_y = candidate.size_y
        detection.results = [hypothesis]
        output.detections.append(detection)
    return output
