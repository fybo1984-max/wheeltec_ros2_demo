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

"""Publish Ultralytics detections using the ROS 2 vision message contract."""

from pathlib import Path

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from ultralytics_ros2.detection_contract import build_detection_array
from ultralytics_ros2.detection_contract import DetectionCandidate


def _load_yolo(model_path: Path):
    try:
        from ultralytics import YOLO
    except ModuleNotFoundError as error:
        raise RuntimeError(
            'Ultralytics is not installed in this Python environment; '
            'install a Jetson-compatible ultralytics runtime before starting '
            'the detector.'
        ) from error
    return YOLO(str(model_path))


class YOLODetector(Node):
    """Run inference on Astra color images and publish Detection2DArray."""

    def __init__(self) -> None:
        super().__init__('yolo_detector')
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter(
            'input_image_topic',
            '/camera/color/image_raw',
        )
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter(
            'annotated_image_topic',
            '/semantic/detected_image',
        )
        self.declare_parameter('device', '0')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('half', False)
        self.declare_parameter('conf_threshold', 0.55)
        self.declare_parameter('class_names', ['person'])
        self.declare_parameter('publish_annotated_image', True)

        model_path = Path(
            str(self.get_parameter('model').value)
        ).expanduser().resolve()
        if not model_path.is_file():
            raise ValueError(f'YOLO model file does not exist: {model_path}')
        self._device = str(self.get_parameter('device').value).strip()
        self._image_size = int(self.get_parameter('imgsz').value)
        if self._image_size <= 0:
            raise ValueError('imgsz must be positive')
        self._half = bool(self.get_parameter('half').value)
        self._confidence = float(
            self.get_parameter('conf_threshold').value
        )
        if not 0.0 <= self._confidence <= 1.0:
            raise ValueError('conf_threshold must be in [0, 1]')
        self._class_names = [
            str(name)
            for name in self.get_parameter('class_names').value
        ]
        self._publish_annotated = bool(
            self.get_parameter('publish_annotated_image').value
        )
        self._model = _load_yolo(model_path)
        self._bridge = CvBridge()
        self._detections_publisher = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter('detections_topic').value),
            qos_profile_sensor_data,
        )
        self._annotated_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('annotated_image_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('input_image_topic').value),
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'person detector ready; model={model_path}; '
            f'device={self._device or "auto"}; '
            f'imgsz={self._image_size}; half={self._half}; '
            f'classes={self._class_names}'
        )

    def _image_callback(self, message: Image) -> None:
        image = self._bridge.imgmsg_to_cv2(
            message,
            desired_encoding='bgr8',
        )
        options = {
            'source': image,
            'conf': self._confidence,
            'imgsz': self._image_size,
            'half': self._half,
            'verbose': False,
        }
        if self._device:
            options['device'] = self._device
        results = self._model.predict(**options)
        result = results[0]
        candidates = []
        for box in result.boxes:
            center_x, center_y, size_x, size_y = (
                float(value) for value in box.xywh[0].tolist()
            )
            class_index = int(box.cls.item())
            candidates.append(DetectionCandidate(
                class_id=str(self._model.names[class_index]),
                score=float(box.conf.item()),
                center_x=center_x,
                center_y=center_y,
                size_x=size_x,
                size_y=size_y,
            ))
        detections = build_detection_array(
            message.header,
            candidates,
            self._class_names,
            self._confidence,
        )
        self._detections_publisher.publish(detections)
        if self._publish_annotated:
            annotated = self._bridge.cv2_to_imgmsg(
                result.plot(),
                encoding='bgr8',
            )
            annotated.header = message.header
            self._annotated_publisher.publish(annotated)


def main(args=None) -> None:
    """Run the configurable Ultralytics detection node."""
    rclpy.init(args=args)
    node = YOLODetector()
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
