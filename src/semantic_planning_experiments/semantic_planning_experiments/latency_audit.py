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

"""Record auditable ROS message timing without claiming false causality."""

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


_STAGES = (
    'image',
    'detection',
    'mask',
    'costmap',
    'plan',
    'cmd_vel',
    'odometry',
)
_PAIR_DEFINITIONS = (
    ('image', 'detection', 'matching_header_stamp'),
    ('detection', 'mask', 'nearest_preceding_receipt'),
    ('mask', 'costmap', 'nearest_preceding_receipt'),
    ('plan', 'cmd_vel', 'nearest_preceding_receipt'),
    ('cmd_vel', 'odometry', 'nearest_preceding_receipt'),
)


def _stamp_nanoseconds(stamp) -> int | None:
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else None


def _percentile(values: list[float], fraction: float) -> float:
    """Return a linearly interpolated percentile for sorted finite values."""
    if not values:
        raise ValueError('percentile needs at least one value')
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def associate_events(
    events: list[dict],
    upstream_stage: str,
    downstream_stage: str,
    method: str,
    maximum_gap_sec: float,
) -> list[float]:
    """Return receipt-time gaps under one explicitly named association rule."""
    if method not in {'matching_header_stamp', 'nearest_preceding_receipt'}:
        raise ValueError(f'unsupported association method: {method}')
    if not math.isfinite(maximum_gap_sec) or maximum_gap_sec <= 0.0:
        raise ValueError('maximum_gap_sec must be positive and finite')

    upstream = [event for event in events if event['stage'] == upstream_stage]
    downstream = [
        event for event in events if event['stage'] == downstream_stage
    ]
    gaps = []
    for target in downstream:
        candidates = upstream
        if method == 'matching_header_stamp':
            stamp = target['header_stamp_ns']
            if stamp is None:
                continue
            candidates = [
                event for event in upstream
                if event['header_stamp_ns'] == stamp
            ]
        candidates = [
            event for event in candidates
            if event['receipt_monotonic_ns'] <= target['receipt_monotonic_ns']
        ]
        if not candidates:
            continue
        source = max(candidates, key=lambda item: item['receipt_monotonic_ns'])
        gap = (
            target['receipt_monotonic_ns'] - source['receipt_monotonic_ns']
        ) / 1e9
        if gap <= maximum_gap_sec:
            gaps.append(gap)
    return gaps


def summarize_events(events: list[dict], maximum_gap_sec: float) -> dict:
    """Summarize coverage and pairwise receipt gaps with method labels."""
    pairs = []
    for upstream, downstream, method in _PAIR_DEFINITIONS:
        gaps = associate_events(
            events,
            upstream,
            downstream,
            method,
            maximum_gap_sec,
        )
        item = {
            'upstream_stage': upstream,
            'downstream_stage': downstream,
            'association_method': method,
            'matched_event_count': len(gaps),
            'maximum_association_gap_sec': maximum_gap_sec,
            'latency_claim_allowed': method == 'matching_header_stamp',
        }
        if gaps:
            item['receipt_gap_sec'] = {
                'median': statistics.median(gaps),
                'iqr': _percentile(gaps, 0.75) - _percentile(gaps, 0.25),
                'p95': _percentile(gaps, 0.95),
                'maximum': max(gaps),
            }
        else:
            item['receipt_gap_sec'] = None
        pairs.append(item)
    return {
        'event_count': len(events),
        'stage_counts': {
            stage: sum(event['stage'] == stage for event in events)
            for stage in _STAGES
        },
        'pairwise_receipt_associations': pairs,
        'interpretation': {
            'matching_header_stamp': (
                'The detector copies the source image header. The receipt gap '
                'uses the local monotonic clock and is an observable '
                'image-to-detection pipeline proxy, not GPU kernel-only '
                'inference time.'
            ),
            'nearest_preceding_receipt': (
                'Temporal association only. It is not proof that the upstream '
                'message caused the downstream message and must not be reported '
                'as causal end-to-end latency.'
            ),
            'cmd_vel': (
                'geometry_msgs/Twist has no header; only ROS receipt time is '
                'recorded.'
            ),
        },
    }


class LatencyAuditRecorder(Node):
    """Observe selected topics and write raw events plus a qualified summary."""

    def __init__(self) -> None:
        super().__init__('semantic_latency_audit')
        self.declare_parameter('duration_sec', 30.0)
        self.declare_parameter('maximum_association_gap_sec', 2.0)
        self.declare_parameter(
            'output_directory',
            '/tmp/semantic_latency_audit',
        )
        topic_defaults = {
            'image': '/camera/color/image_raw',
            'detection': '/detections',
            'mask': '/semantic_mask',
            'costmap': '/global_costmap/costmap',
            'plan': '/plan',
            'cmd_vel': '/cmd_vel',
            'odometry': '/odom_combined',
        }
        for stage, topic in topic_defaults.items():
            self.declare_parameter(f'{stage}_topic', topic)

        self._duration_sec = float(self.get_parameter('duration_sec').value)
        self._maximum_gap_sec = float(
            self.get_parameter('maximum_association_gap_sec').value
        )
        if not math.isfinite(self._duration_sec) or self._duration_sec <= 0.0:
            raise ValueError('duration_sec must be positive and finite')
        if (
            not math.isfinite(self._maximum_gap_sec)
            or self._maximum_gap_sec <= 0.0
        ):
            raise ValueError(
                'maximum_association_gap_sec must be positive and finite'
            )
        self._output_directory = Path(
            str(self.get_parameter('output_directory').value)
        ).expanduser().resolve()
        self._events_path = self._output_directory / 'latency_events.csv'
        self._summary_path = self._output_directory / 'latency_summary.json'
        if self._events_path.exists() or self._summary_path.exists():
            raise ValueError(
                f'latency audit output already exists: '
                f'{self._output_directory}'
            )

        self._events = []
        self._finished = False
        self._started_monotonic = time.monotonic()
        subscriptions = (
            ('image', Image, qos_profile_sensor_data),
            ('detection', Detection2DArray, qos_profile_sensor_data),
            ('mask', OccupancyGrid, 10),
            ('costmap', OccupancyGrid, 10),
            ('plan', PathMessage, 10),
            ('cmd_vel', Twist, 10),
            ('odometry', Odometry, qos_profile_sensor_data),
        )
        for stage, message_type, qos in subscriptions:
            topic = str(self.get_parameter(f'{stage}_topic').value)
            self.create_subscription(
                message_type,
                topic,
                lambda message, stage=stage, topic=topic: self._record(
                    stage, topic, message
                ),
                qos,
            )
        self.create_timer(0.1, self._check_duration)

    def _record(self, stage: str, topic: str, message) -> None:
        header = getattr(message, 'header', None)
        self._events.append({
            'sequence': len(self._events) + 1,
            'stage': stage,
            'topic': topic,
            'message_type': type(message).__name__,
            'receipt_ros_ns': int(self.get_clock().now().nanoseconds),
            'receipt_monotonic_ns': time.monotonic_ns(),
            'header_stamp_ns': (
                _stamp_nanoseconds(header.stamp)
                if header is not None else None
            ),
            'frame_id': str(header.frame_id) if header is not None else '',
        })

    def _check_duration(self) -> None:
        if time.monotonic() - self._started_monotonic >= self._duration_sec:
            self.finish('duration_elapsed')

    def finish(self, stop_reason: str) -> None:
        """Write outputs exactly once and mark the recorder complete."""
        if self._finished:
            return
        self._finished = True
        self._output_directory.mkdir(parents=True, exist_ok=True)
        fields = (
            'sequence',
            'stage',
            'topic',
            'message_type',
            'receipt_ros_ns',
            'receipt_monotonic_ns',
            'header_stamp_ns',
            'frame_id',
        )
        with self._events_path.open('x', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self._events)
        summary = summarize_events(self._events, self._maximum_gap_sec)
        summary.update({
            'schema_version': 1,
            'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'stop_reason': stop_reason,
            'duration_sec': self._duration_sec,
            'events_csv': str(self._events_path),
        })
        self._summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        self.get_logger().info(
            f'wrote {len(self._events)} latency events to '
            f'{self._output_directory}'
        )


def main(args=None) -> None:
    """Run the passive timing recorder until duration or interruption."""
    rclpy.init(args=args)
    node = LatencyAuditRecorder()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.finish('keyboard_interrupt')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
