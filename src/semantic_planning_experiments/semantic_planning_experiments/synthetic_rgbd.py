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

"""Publish deterministic registered RGB-D and object detections for tests."""

import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from semantic_planning_experiments.synthetic_scene import SyntheticSceneSpec


class SyntheticRGBDSource(Node):
    """Publish a centered detection with constant registered depth."""

    def __init__(self) -> None:
        super().__init__('synthetic_rgbd_source')
        self.declare_parameter('camera_frame', 'synthetic_camera_optical_frame')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('width', 64)
        self.declare_parameter('height', 48)
        self.declare_parameter('focal_length_px', 50.0)
        self.declare_parameter('depth_m', 2.0)
        self.declare_parameter('depth_noise_std_m', 0.0)
        self.declare_parameter('depth_invalid_fraction', 0.0)
        self.declare_parameter('random_seed', 42)
        self.declare_parameter('detection_score', 0.95)
        self.declare_parameter('detection_class_id', 'person')
        self.declare_parameter('person_count', 1)
        self.declare_parameter('detection_center_x_fraction', 0.5)
        self.declare_parameter('detection_center_y_fraction', 0.5)
        self.declare_parameter('person_spacing_y_fraction', 0.2)
        self.declare_parameter('bbox_width_fraction', 0.25)
        self.declare_parameter('bbox_height_fraction', 0.5)
        self.declare_parameter('detection_publish_every_n_frames', 1)
        self.declare_parameter('detection_timestamp_offset_sec', 0.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('detections_active_duration_sec', 0.0)

        self._camera_frame = self.get_parameter('camera_frame').value
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._focal = float(self.get_parameter('focal_length_px').value)
        self._score = float(self.get_parameter('detection_score').value)
        self._scene = SyntheticSceneSpec(
            width=self._width,
            height=self._height,
            depth_m=float(self.get_parameter('depth_m').value),
            depth_noise_std_m=float(
                self.get_parameter('depth_noise_std_m').value
            ),
            depth_invalid_fraction=float(
                self.get_parameter('depth_invalid_fraction').value
            ),
            random_seed=int(self.get_parameter('random_seed').value),
            detection_class_id=str(
                self.get_parameter('detection_class_id').value
            ),
            person_count=int(self.get_parameter('person_count').value),
            center_x_fraction=float(
                self.get_parameter('detection_center_x_fraction').value
            ),
            center_y_fraction=float(
                self.get_parameter('detection_center_y_fraction').value
            ),
            person_spacing_y_fraction=float(
                self.get_parameter('person_spacing_y_fraction').value
            ),
            bbox_width_fraction=float(
                self.get_parameter('bbox_width_fraction').value
            ),
            bbox_height_fraction=float(
                self.get_parameter('bbox_height_fraction').value
            ),
            detection_publish_every_n_frames=int(
                self.get_parameter(
                    'detection_publish_every_n_frames'
                ).value
            ),
            detection_timestamp_offset_sec=float(
                self.get_parameter('detection_timestamp_offset_sec').value
            ),
        )
        self._scene.validate()
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self._active_duration = float(
            self.get_parameter('detections_active_duration_sec').value
        )
        if self._focal <= 0.0 or publish_rate <= 0.0:
            raise ValueError('synthetic focal length and rate must be positive')
        if not 0.0 <= self._score <= 1.0:
            raise ValueError('synthetic detection score must be in [0, 1]')
        if self._active_duration < 0.0:
            raise ValueError('detections_active_duration_sec must be nonnegative')
        self._started_at = time.monotonic()
        self._reported_stop = False
        self._publish_count = 0

        self._camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            qos_profile_sensor_data,
        )
        self._color_publisher = self.create_publisher(
            Image,
            self.get_parameter('color_topic').value,
            qos_profile_sensor_data,
        )
        self._depth_publisher = self.create_publisher(
            Image,
            self.get_parameter('depth_topic').value,
            qos_profile_sensor_data,
        )
        self._detections_publisher = self.create_publisher(
            Detection2DArray,
            self.get_parameter('detections_topic').value,
            qos_profile_sensor_data,
        )
        self._depth_bytes = self._scene.depth_bytes()
        self._color_bytes = bytes(self._width * self._height * 3)
        self.create_timer(1.0 / publish_rate, self._publish)

    def _publish(self) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        detection_stamp = (
            now + Duration(
                seconds=self._scene.detection_timestamp_offset_sec
            )
        ).to_msg()

        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = self._camera_frame
        camera_info.width = self._width
        camera_info.height = self._height
        camera_info.k = [
            self._focal, 0.0, self._width * 0.5,
            0.0, self._focal, self._height * 0.5,
            0.0, 0.0, 1.0,
        ]

        color = Image()
        color.header.stamp = stamp
        color.header.frame_id = self._camera_frame
        color.width = self._width
        color.height = self._height
        color.encoding = 'bgr8'
        color.is_bigendian = 0
        color.step = self._width * 3
        color.data = self._color_bytes

        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = self._camera_frame
        depth.width = self._width
        depth.height = self._height
        depth.encoding = '16UC1'
        depth.is_bigendian = 0
        depth.step = self._width * 2
        depth.data = self._depth_bytes

        detection_messages = []
        for center_x, center_y in self._scene.detection_centers_px():
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = self._scene.detection_class_id
            result.hypothesis.score = self._score
            detection = Detection2D()
            detection.header.stamp = detection_stamp
            detection.header.frame_id = self._camera_frame
            detection.results = [result]
            detection.bbox.center.position.x = center_x
            detection.bbox.center.position.y = center_y
            detection.bbox.size_x = (
                self._width * self._scene.bbox_width_fraction
            )
            detection.bbox.size_y = (
                self._height * self._scene.bbox_height_fraction
            )
            detection_messages.append(detection)
        detections = Detection2DArray()
        detections.header.stamp = detection_stamp
        detections.header.frame_id = self._camera_frame
        detections_active = (
            self._active_duration == 0.0
            or time.monotonic() - self._started_at < self._active_duration
        )
        publish_detection_frame = (
            self._publish_count
            % self._scene.detection_publish_every_n_frames
            == 0
        )
        detections.detections = (
            detection_messages
            if detections_active and publish_detection_frame
            else []
        )
        if not detections_active and not self._reported_stop:
            self.get_logger().info(
                'synthetic detections stopped; RGB-D publishing continues'
            )
            self._reported_stop = True

        self._color_publisher.publish(color)
        self._camera_info_publisher.publish(camera_info)
        self._depth_publisher.publish(depth)
        self._detections_publisher.publish(detections)
        self._publish_count += 1


def main(args=None) -> None:
    """Run the deterministic synthetic RGB-D source."""
    rclpy.init(args=args)
    node = SyntheticRGBDSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
