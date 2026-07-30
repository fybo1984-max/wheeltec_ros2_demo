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

"""Publish deterministic registered RGB-D and person detections for tests."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class SyntheticRGBDSource(Node):
    """Publish a centered detection with constant registered depth."""

    def __init__(self) -> None:
        super().__init__('synthetic_rgbd_source')
        self.declare_parameter('camera_frame', 'synthetic_camera_optical_frame')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('width', 64)
        self.declare_parameter('height', 48)
        self.declare_parameter('focal_length_px', 50.0)
        self.declare_parameter('depth_m', 2.0)
        self.declare_parameter('detection_score', 0.95)
        self.declare_parameter('publish_rate_hz', 10.0)

        self._camera_frame = self.get_parameter('camera_frame').value
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._focal = float(self.get_parameter('focal_length_px').value)
        self._depth_m = float(self.get_parameter('depth_m').value)
        self._score = float(self.get_parameter('detection_score').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if self._width <= 0 or self._height <= 0:
            raise ValueError('synthetic image dimensions must be positive')
        if self._focal <= 0.0 or self._depth_m <= 0.0 or publish_rate <= 0.0:
            raise ValueError('synthetic focal length, depth, and rate must be positive')
        if not 0.0 <= self._score <= 1.0:
            raise ValueError('synthetic detection score must be in [0, 1]')

        self._camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
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
        depth_mm = int(round(self._depth_m * 1000.0))
        self._depth_bytes = np.full(
            (self._height, self._width),
            depth_mm,
            dtype='<u2',
        ).tobytes()
        self.create_timer(1.0 / publish_rate, self._publish)

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()

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

        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = self._camera_frame
        depth.width = self._width
        depth.height = self._height
        depth.encoding = '16UC1'
        depth.is_bigendian = 0
        depth.step = self._width * 2
        depth.data = self._depth_bytes

        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = 'person'
        result.hypothesis.score = self._score
        detection = Detection2D()
        detection.header.stamp = stamp
        detection.header.frame_id = self._camera_frame
        detection.results = [result]
        detection.bbox.center.position.x = self._width * 0.5
        detection.bbox.center.position.y = self._height * 0.5
        detection.bbox.size_x = self._width * 0.25
        detection.bbox.size_y = self._height * 0.5
        detections = Detection2DArray()
        detections.header.stamp = stamp
        detections.header.frame_id = self._camera_frame
        detections.detections = [detection]

        self._camera_info_publisher.publish(camera_info)
        self._depth_publisher.publish(depth)
        self._detections_publisher.publish(detections)


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
