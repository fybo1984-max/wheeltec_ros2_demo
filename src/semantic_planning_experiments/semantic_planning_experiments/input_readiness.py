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

"""Validate live RGB-D and object-detection inputs before data collection."""

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2DArray


_ACCEPTED_DEPTH_ENCODINGS = {'16UC1', '32FC1'}


def _finite_positive(value) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(value)
        and value > 0.0
    )


def evaluate_input_readiness(
    snapshot: dict,
    minimum_messages: int,
    maximum_sync_delta_sec: float,
) -> dict:
    """Return explicit readiness checks for one live-input snapshot."""
    if isinstance(minimum_messages, bool) or minimum_messages < 1:
        raise ValueError('minimum_messages must be a positive integer')
    if not _finite_positive(maximum_sync_delta_sec):
        raise ValueError('maximum_sync_delta_sec must be positive and finite')

    counts = snapshot['message_counts']
    color = snapshot['color']
    depth = snapshot['depth']
    camera_info = snapshot['camera_info']
    detections = snapshot['detections']
    transform = snapshot['transform']
    sync_delta = snapshot[
        'maximum_nearest_detection_depth_delta_sec'
    ]

    checks = {
        'color_messages': counts['color'] >= minimum_messages,
        'depth_messages': counts['depth'] >= minimum_messages,
        'camera_info_messages': counts['camera_info'] >= minimum_messages,
        'detection_messages': counts['detections'] >= minimum_messages,
        'target_class_detected': detections['target_detection_count'] > 0,
        'depth_encoding_supported': (
            depth['encoding'] in _ACCEPTED_DEPTH_ENCODINGS
        ),
        'camera_intrinsics_valid': (
            _finite_positive(camera_info['fx'])
            and _finite_positive(camera_info['fy'])
        ),
        'registered_dimensions_match': (
            color['width'] > 0
            and color['height'] > 0
            and (color['width'], color['height'])
            == (depth['width'], depth['height'])
            == (camera_info['width'], camera_info['height'])
        ),
        'registered_frames_match': (
            bool(color['frame_id'])
            and color['frame_id'] == depth['frame_id']
            and color['frame_id'] == camera_info['frame_id']
            and color['frame_id'] == detections['frame_id']
        ),
        'detection_depth_synchronized': (
            sync_delta is not None
            and math.isfinite(sync_delta)
            and sync_delta <= maximum_sync_delta_sec
        ),
        'map_transform_available': bool(transform['available']),
    }
    failures = [name for name, passed in checks.items() if not passed]
    return {
        'experiment_input_ready': not failures,
        'checks': checks,
        'failures': failures,
    }


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class LiveInputReadiness(Node):
    """Observe, validate, report, and exit without starting other nodes."""

    def __init__(self) -> None:
        super().__init__('semantic_live_input_readiness')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter(
            'depth_topic',
            '/camera/depth/image_raw',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/color/camera_info',
        )
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('target_class_id', 'person')
        self.declare_parameter('minimum_confidence', 0.5)
        self.declare_parameter('minimum_messages', 3)
        self.declare_parameter('maximum_sync_delta_sec', 0.15)
        self.declare_parameter('timeout_sec', 10.0)
        self.declare_parameter(
            'output_path',
            '/tmp/semantic_live_input_readiness.json',
        )

        self._target_frame = str(self.get_parameter('target_frame').value)
        self._target_class = str(
            self.get_parameter('target_class_id').value
        ).casefold()
        self._minimum_confidence = float(
            self.get_parameter('minimum_confidence').value
        )
        self._minimum_messages = int(
            self.get_parameter('minimum_messages').value
        )
        self._maximum_sync_delta = float(
            self.get_parameter('maximum_sync_delta_sec').value
        )
        self._timeout_sec = float(self.get_parameter('timeout_sec').value)
        self._output_path = Path(
            str(self.get_parameter('output_path').value)
        ).expanduser().resolve()
        if not 0.0 <= self._minimum_confidence <= 1.0:
            raise ValueError('minimum_confidence must be in [0, 1]')
        if not _finite_positive(self._timeout_sec):
            raise ValueError('timeout_sec must be positive and finite')
        if self._output_path.exists():
            raise ValueError(
                f'input readiness report already exists: {self._output_path}'
            )

        self._started_at = time.monotonic()
        self._finished = False
        self.report = None
        self._counts = {
            'color': 0,
            'depth': 0,
            'camera_info': 0,
            'detections': 0,
        }
        self._color = {
            'width': 0,
            'height': 0,
            'encoding': '',
            'frame_id': '',
        }
        self._depth = dict(self._color)
        self._camera_info = {
            'width': 0,
            'height': 0,
            'fx': 0.0,
            'fy': 0.0,
            'frame_id': '',
        }
        self._detections = {
            'frame_id': '',
            'target_class_id': self._target_class,
            'target_detection_count': 0,
            'maximum_target_confidence': None,
        }
        self._depth_stamps = []
        self._detection_stamps = []

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(
            Image,
            str(self.get_parameter('color_topic').value),
            self._on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_topic').value),
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray,
            str(self.get_parameter('detections_topic').value),
            self._on_detections,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self._evaluate)

    def _on_color(self, message: Image) -> None:
        self._counts['color'] += 1
        self._color = {
            'width': int(message.width),
            'height': int(message.height),
            'encoding': str(message.encoding),
            'frame_id': str(message.header.frame_id),
        }

    def _on_depth(self, message: Image) -> None:
        self._counts['depth'] += 1
        self._depth = {
            'width': int(message.width),
            'height': int(message.height),
            'encoding': str(message.encoding),
            'frame_id': str(message.header.frame_id),
        }
        self._depth_stamps.append(_stamp_seconds(message.header.stamp))
        self._depth_stamps = self._depth_stamps[-200:]

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._counts['camera_info'] += 1
        self._camera_info = {
            'width': int(message.width),
            'height': int(message.height),
            'fx': float(message.k[0]),
            'fy': float(message.k[4]),
            'frame_id': str(message.header.frame_id),
        }

    def _on_detections(self, message: Detection2DArray) -> None:
        self._counts['detections'] += 1
        self._detections['frame_id'] = str(message.header.frame_id)
        self._detection_stamps.append(_stamp_seconds(message.header.stamp))
        self._detection_stamps = self._detection_stamps[-200:]
        for detection in message.detections:
            for result in detection.results:
                hypothesis = result.hypothesis
                if (
                    str(hypothesis.class_id).casefold() == self._target_class
                    and float(hypothesis.score) >= self._minimum_confidence
                ):
                    self._detections['target_detection_count'] += 1
                    previous = self._detections[
                        'maximum_target_confidence'
                    ]
                    score = float(hypothesis.score)
                    self._detections['maximum_target_confidence'] = (
                        score if previous is None else max(previous, score)
                    )

    def _maximum_nearest_sync_delta(self):
        if not self._depth_stamps or not self._detection_stamps:
            return None
        return max(
            min(
                abs(detection - depth)
                for depth in self._depth_stamps
            )
            for detection in self._detection_stamps
        )

    def _snapshot(self) -> dict:
        source_frame = self._detections['frame_id']
        transform_available = bool(
            source_frame
            and self._tf_buffer.can_transform(
                self._target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        )
        return {
            'message_counts': dict(self._counts),
            'color': dict(self._color),
            'depth': dict(self._depth),
            'camera_info': dict(self._camera_info),
            'detections': dict(self._detections),
            'maximum_nearest_detection_depth_delta_sec': (
                self._maximum_nearest_sync_delta()
            ),
            'transform': {
                'target_frame': self._target_frame,
                'source_frame': source_frame,
                'available': transform_available,
            },
        }

    def _evaluate(self) -> None:
        if self._finished:
            return
        snapshot = self._snapshot()
        readiness = evaluate_input_readiness(
            snapshot,
            self._minimum_messages,
            self._maximum_sync_delta,
        )
        timed_out = time.monotonic() - self._started_at >= self._timeout_sec
        if readiness['experiment_input_ready'] or timed_out:
            self._finish(snapshot, readiness, timed_out)

    def _finish(self, snapshot: dict, readiness: dict, timed_out: bool) -> None:
        self._finished = True
        self.report = {
            'schema_version': 1,
            'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'experiment_input_ready': readiness['experiment_input_ready'],
            'timed_out': timed_out,
            'requirements': {
                'minimum_messages': self._minimum_messages,
                'maximum_sync_delta_sec': self._maximum_sync_delta,
                'minimum_confidence': self._minimum_confidence,
                'target_class_id': self._target_class,
            },
            'checks': readiness['checks'],
            'failures': readiness['failures'],
            'observations': snapshot,
        }
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._output_path.with_name(
            self._output_path.name + '.tmp'
        )
        temporary.write_text(
            json.dumps(self.report, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        os.replace(temporary, self._output_path)
        if readiness['experiment_input_ready']:
            self.get_logger().info(
                f'experiment input ready; report={self._output_path}'
            )
        else:
            self.get_logger().error(
                f'experiment input not ready: {readiness["failures"]}; '
                f'report={self._output_path}'
            )


def main(args=None) -> None:
    """Observe the configured topics and exit with a readiness result."""
    rclpy.init(args=args)
    node = LiveInputReadiness()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        report = node.report
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not report or not report['experiment_input_ready']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
