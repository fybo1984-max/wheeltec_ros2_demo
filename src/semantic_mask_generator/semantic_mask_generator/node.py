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

"""ROS node that projects registered RGB-D detections into the map frame."""

import copy
import math
import time

from aruco_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from vision_msgs.msg import Detection2DArray

from semantic_mask_generator.core import build_marker_profile_map
from semantic_mask_generator.core import decode_depth_image
from semantic_mask_generator.core import GridSpec
from semantic_mask_generator.core import median_depth
from semantic_mask_generator.core import ObservationStore
from semantic_mask_generator.core import project_pixel
from semantic_mask_generator.core import rasterize_observations
from semantic_mask_generator.core import rasterize_polygon
from semantic_mask_generator.core import RiskProfile
from semantic_mask_generator.core import scale_risk_profile
from semantic_mask_generator.core import transform_point


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SemanticMaskNode(Node):
    """Generate a transient, map-aligned semantic risk OccupancyGrid."""

    def __init__(self) -> None:
        super().__init__('semantic_mask_generator')
        self._declare_parameters()
        self._read_parameters()

        self._depth_message = None
        self._camera_info = None
        self._map_info = None
        self._map_frame = ''
        self._last_warning: dict[str, float] = {}
        self._store = ObservationStore(
            self._merge_distance_m,
            self._max_observations,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        mask_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            self._map_topic,
            self._map_callback,
            map_qos,
        )
        self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self._depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection2DArray,
            self._detections_topic,
            self._detections_callback,
            qos_profile_sensor_data,
        )
        if self._marker_enabled:
            self.create_subscription(
                MarkerArray,
                self._marker_topic,
                self._marker_callback,
                qos_profile_sensor_data,
            )
        if self._loading_zone_enabled:
            self.create_subscription(
                Bool,
                self._loading_zone_active_topic,
                self._loading_zone_callback,
                10,
            )
        self._publisher = self.create_publisher(
            OccupancyGrid,
            self._mask_topic,
            mask_qos,
        )
        self.create_timer(1.0 / self._publish_rate_hz, self._publish_mask)

        if not self._enabled:
            self.get_logger().info(
                'Dynamic semantic mask is disabled; no masks will be published.'
            )
        if not self._depth_is_registered:
            self.get_logger().warning(
                'depth_is_registered is false; detections will be rejected '
                'until aligned RGB-D input is explicitly confirmed.'
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter('enabled', False)
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/color/camera_info',
        )
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('mask_topic', '/semantic_mask')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('depth_is_registered', False)
        self.declare_parameter('minimum_confidence', 0.5)
        self.declare_parameter('minimum_depth_m', 0.3)
        self.declare_parameter('maximum_depth_m', 8.0)
        self.declare_parameter('depth_window_radius_px', 3)
        self.declare_parameter('bbox_sample_y_fraction', 0.5)
        self.declare_parameter('maximum_sync_delta_sec', 0.15)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('observation_hold_sec', 0.8)
        self.declare_parameter('observation_decay_sec', 1.2)
        self.declare_parameter('merge_distance_m', 0.4)
        self.declare_parameter('maximum_observations', 200)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('risk_class_names', ['person'])
        self.declare_parameter('risk_class_values', [100])
        self.declare_parameter('risk_class_radii_m', [1.2])
        self.declare_parameter('risk_value_scale', 1.0)
        self.declare_parameter('risk_radius_scale', 1.0)
        self.declare_parameter('marker_enabled', False)
        self.declare_parameter(
            'marker_topic',
            '/aruco_marker_publisher/markers',
        )
        self.declare_parameter('marker_ids', [101])
        self.declare_parameter('marker_labels', ['fragile_goods'])
        self.declare_parameter('marker_minimum_confidence', 0.5)
        self.declare_parameter('loading_zone_enabled', False)
        self.declare_parameter(
            'loading_zone_active_topic',
            '/semantic/loading_zone_active',
        )
        self.declare_parameter('loading_zone_initial_active', False)
        self.declare_parameter('loading_zone_risk_value', 85)
        self.declare_parameter(
            'loading_zone_vertices_m',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    def _read_parameters(self) -> None:
        def value(name):
            return self.get_parameter(name).value

        self._enabled = bool(value('enabled'))
        self._detections_topic = str(value('detections_topic'))
        self._depth_topic = str(value('depth_image_topic'))
        self._camera_info_topic = str(value('camera_info_topic'))
        self._map_topic = str(value('map_topic'))
        self._mask_topic = str(value('mask_topic'))
        self._target_frame = str(value('target_frame'))
        self._depth_is_registered = bool(value('depth_is_registered'))
        self._minimum_confidence = float(value('minimum_confidence'))
        self._minimum_depth_m = float(value('minimum_depth_m'))
        self._maximum_depth_m = float(value('maximum_depth_m'))
        self._depth_window_radius = int(value('depth_window_radius_px'))
        self._sample_y_fraction = float(value('bbox_sample_y_fraction'))
        self._maximum_sync_delta = float(value('maximum_sync_delta_sec'))
        self._tf_timeout = float(value('tf_timeout_sec'))
        self._hold_sec = float(value('observation_hold_sec'))
        self._decay_sec = float(value('observation_decay_sec'))
        self._merge_distance_m = float(value('merge_distance_m'))
        self._max_observations = int(value('maximum_observations'))
        self._publish_rate_hz = float(value('publish_rate_hz'))

        names = list(value('risk_class_names'))
        risk_values = list(value('risk_class_values'))
        radii = list(value('risk_class_radii_m'))
        value_scale = float(value('risk_value_scale'))
        radius_scale = float(value('risk_radius_scale'))
        if not names or len(names) != len(risk_values) or len(names) != len(radii):
            raise ValueError('risk class names, values, and radii must align')
        self._profiles = {}
        for name, risk_value, radius in zip(names, risk_values, radii):
            profile = scale_risk_profile(
                RiskProfile(int(risk_value), float(radius)),
                radius_scale,
                value_scale,
            )
            self._profiles[str(name).casefold()] = profile

        self._marker_enabled = bool(value('marker_enabled'))
        self._marker_topic = str(value('marker_topic'))
        self._marker_minimum_confidence = float(
            value('marker_minimum_confidence')
        )
        if self._marker_enabled:
            self._marker_profiles = build_marker_profile_map(
                list(value('marker_ids')),
                list(value('marker_labels')),
                self._profiles,
            )
        else:
            self._marker_profiles = {}
        self._loading_zone_enabled = bool(value('loading_zone_enabled'))
        self._loading_zone_active_topic = str(
            value('loading_zone_active_topic')
        )
        self._loading_zone_active = bool(value('loading_zone_initial_active'))
        self._loading_zone_risk_value = int(value('loading_zone_risk_value'))
        flat_vertices = [
            float(item) for item in value('loading_zone_vertices_m')
        ]
        if self._loading_zone_enabled and (
            len(flat_vertices) < 6 or len(flat_vertices) % 2 != 0
        ):
            raise ValueError(
                'loading_zone_vertices_m must contain at least three x/y pairs'
            )
        self._loading_zone_vertices = list(
            zip(flat_vertices[0::2], flat_vertices[1::2])
        )
        if not 1 <= self._loading_zone_risk_value <= 100:
            raise ValueError('loading_zone_risk_value must be in [1, 100]')
        if self._loading_zone_enabled:
            rasterize_polygon(
                GridSpec(1, 1, 1.0, 0.0, 0.0),
                self._loading_zone_vertices,
                self._loading_zone_risk_value,
            )

        if not 0.0 <= self._minimum_confidence <= 1.0:
            raise ValueError('minimum_confidence must be in [0, 1]')
        if not 0.0 <= self._marker_minimum_confidence <= 1.0:
            raise ValueError('marker_minimum_confidence must be in [0, 1]')
        if (
            self._minimum_depth_m <= 0.0
            or self._maximum_depth_m <= self._minimum_depth_m
        ):
            raise ValueError('depth range is invalid')
        if not 0.0 <= self._sample_y_fraction <= 1.0:
            raise ValueError('bbox_sample_y_fraction must be in [0, 1]')
        if self._maximum_sync_delta < 0.0 or self._tf_timeout < 0.0:
            raise ValueError('sync delta and TF timeout must be nonnegative')
        if self._publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

    def _warn_throttled(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self._last_warning[key] = now

    def _map_callback(self, message: OccupancyGrid) -> None:
        orientation = message.info.origin.orientation
        if (
            message.info.width == 0
            or message.info.height == 0
            or message.info.resolution <= 0.0
        ):
            self._warn_throttled('invalid_map', 'Rejected invalid map metadata.')
            return
        if (
            abs(orientation.x) > 1e-6
            or abs(orientation.y) > 1e-6
            or abs(orientation.z) > 1e-6
            or abs(abs(orientation.w) - 1.0) > 1e-6
        ):
            self._warn_throttled(
                'rotated_map',
                'Rotated maps are not supported by semantic mask generation.',
            )
            return
        frame = message.header.frame_id or self._target_frame
        if frame != self._target_frame:
            self._warn_throttled(
                'map_frame',
                f'Map frame {frame!r} does not match target_frame '
                f'{self._target_frame!r}.',
            )
            return
        self._map_info = copy.deepcopy(message.info)
        self._map_frame = frame

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self._camera_info = message

    def _depth_callback(self, message: Image) -> None:
        self._depth_message = message

    def _detections_callback(self, message: Detection2DArray) -> None:
        if not self._enabled:
            return
        if not self._depth_is_registered:
            self._warn_throttled(
                'unaligned_depth',
                'Skipping detections because registered depth is not confirmed.',
            )
            return
        if self._depth_message is None or self._camera_info is None:
            self._warn_throttled(
                'missing_rgbd',
                'Waiting for depth image and color CameraInfo.',
            )
            return

        detection_stamp = _stamp_seconds(message.header.stamp)
        depth_stamp = _stamp_seconds(self._depth_message.header.stamp)
        if abs(detection_stamp - depth_stamp) > self._maximum_sync_delta:
            self._warn_throttled(
                'sync',
                'Detection and depth timestamps exceed maximum_sync_delta_sec.',
            )
            return
        source_frame = message.header.frame_id or self._camera_info.header.frame_id
        if not source_frame:
            self._warn_throttled('frame', 'Detection frame_id is empty.')
            return
        camera_frame = self._camera_info.header.frame_id
        if camera_frame and camera_frame != source_frame:
            self._warn_throttled(
                'camera_frame',
                f'CameraInfo frame {camera_frame!r} does not match '
                f'detection frame {source_frame!r}.',
            )
            return
        depth_frame = self._depth_message.header.frame_id
        if depth_frame and depth_frame != source_frame:
            self._warn_throttled(
                'depth_frame',
                f'Registered depth frame {depth_frame!r} does not match '
                f'detection frame {source_frame!r}.',
            )
            return

        try:
            depth_m = decode_depth_image(
                self._depth_message.data,
                self._depth_message.encoding,
                self._depth_message.width,
                self._depth_message.height,
                self._depth_message.step,
                self._depth_message.is_bigendian,
            )
        except ValueError as error:
            self._warn_throttled('depth_decode', str(error))
            return

        camera_info = self._camera_info
        if (
            camera_info.width != self._depth_message.width
            or camera_info.height != self._depth_message.height
        ):
            self._warn_throttled(
                'dimensions',
                'Registered depth dimensions do not match color CameraInfo.',
            )
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as error:
            self._warn_throttled('tf', f'Cannot transform detection: {error}')
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        for detection in message.detections:
            candidate = self._select_profile(detection.results)
            if candidate is None:
                continue
            label, score, profile = candidate
            bbox = detection.bbox
            if bbox.size_x <= 0.0 or bbox.size_y <= 0.0:
                continue
            sample_u = bbox.center.position.x
            top = bbox.center.position.y - bbox.size_y * 0.5
            sample_v = top + bbox.size_y * self._sample_y_fraction
            depth = median_depth(
                depth_m,
                sample_u,
                sample_v,
                self._depth_window_radius,
                self._minimum_depth_m,
                self._maximum_depth_m,
            )
            if depth is None:
                continue
            camera_point = project_pixel(
                sample_u,
                sample_v,
                depth,
                camera_info.k[0],
                camera_info.k[4],
                camera_info.k[2],
                camera_info.k[5],
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            map_point = transform_point(
                camera_point,
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
            self._store.add(
                label,
                map_point[0],
                map_point[1],
                profile,
                now,
                self._hold_sec,
                self._decay_sec,
            )
            self.get_logger().debug(
                f'Accepted {label} score={score:.3f} at '
                f'({map_point[0]:.2f}, {map_point[1]:.2f}).'
            )

    def _select_profile(self, results):
        candidates = []
        for result in results:
            label = result.hypothesis.class_id.casefold()
            score = float(result.hypothesis.score)
            profile = self._profiles.get(label)
            if profile is not None and score >= self._minimum_confidence:
                candidates.append((label, score, profile))
        return max(candidates, key=lambda item: item[1], default=None)

    def _marker_callback(self, message: MarkerArray) -> None:
        if not self._enabled or not self._marker_enabled:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for marker in message.markers:
            candidate = self._marker_profiles.get(int(marker.id))
            confidence = float(marker.confidence)
            if (
                candidate is None
                or not math.isfinite(confidence)
                or confidence < self._marker_minimum_confidence
            ):
                continue
            label, profile = candidate
            source_frame = marker.header.frame_id or message.header.frame_id
            if not source_frame:
                self._warn_throttled(
                    'marker_frame',
                    'Marker frame_id is empty.',
                )
                continue
            position = marker.pose.pose.position
            marker_point = (position.x, position.y, position.z)
            if not all(math.isfinite(value) for value in marker_point):
                continue

            if source_frame == self._target_frame:
                map_point = marker_point
            else:
                stamp = marker.header.stamp
                if stamp.sec == 0 and stamp.nanosec == 0:
                    stamp = message.header.stamp
                try:
                    transform = self._tf_buffer.lookup_transform(
                        self._target_frame,
                        source_frame,
                        Time.from_msg(stamp),
                        timeout=Duration(seconds=self._tf_timeout),
                    )
                except TransformException as error:
                    self._warn_throttled(
                        'marker_tf',
                        f'Cannot transform semantic marker: {error}',
                    )
                    continue
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                map_point = transform_point(
                    marker_point,
                    (translation.x, translation.y, translation.z),
                    (rotation.x, rotation.y, rotation.z, rotation.w),
                )

            self._store.add(
                label,
                map_point[0],
                map_point[1],
                profile,
                now,
                self._hold_sec,
                self._decay_sec,
            )
            self.get_logger().debug(
                f'Accepted marker {marker.id} as {label} '
                f'at ({map_point[0]:.2f}, {map_point[1]:.2f}).'
            )

    def _loading_zone_callback(self, message: Bool) -> None:
        self._loading_zone_active = bool(message.data)

    def _publish_mask(self) -> None:
        if not self._enabled or self._map_info is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        observations = self._store.active(now)
        spec = GridSpec(
            width=self._map_info.width,
            height=self._map_info.height,
            resolution=self._map_info.resolution,
            origin_x=self._map_info.origin.position.x,
            origin_y=self._map_info.origin.position.y,
        )
        grid = rasterize_observations(spec, observations, now)
        if self._loading_zone_enabled and self._loading_zone_active:
            loading_zone = rasterize_polygon(
                spec,
                self._loading_zone_vertices,
                self._loading_zone_risk_value,
            )
            grid = np.maximum(grid, loading_zone)
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        message.info = copy.deepcopy(self._map_info)
        message.data = grid.reshape(-1).tolist()
        self._publisher.publish(message)


def main(args=None) -> None:
    """Run the semantic mask generator."""
    rclpy.init(args=args)
    node = SemanticMaskNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
