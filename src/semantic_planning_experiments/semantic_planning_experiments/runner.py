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

"""Run baseline and semantic Nav2 planning requests without a controller."""

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from semantic_planning_experiments.metrics import (
    footprint_path_metrics,
    load_mask_grid,
    load_planner_footprint,
    mask_geometry_summary,
    mask_grid_from_occupancy_data,
    metric_delta,
    occupancy_data,
    occupancy_value_summary,
    path_metrics,
    sha256_file,
)
from semantic_planning_experiments.recorded_replay import (
    RECORDED_RGBD_TOPICS,
)
from semantic_planning_experiments.archive_audit import inspect_recorded_bag


class SemanticPlanningAB(Node):
    """Coordinate two planner requests that differ only by mask enablement."""

    def __init__(self) -> None:
        super().__init__('semantic_planning_ab')
        self.declare_parameter('action_name', '/compute_path_to_pose')
        self.declare_parameter(
            'costmap_parameter_service',
            '/global_costmap/global_costmap/set_parameters',
        )
        self.declare_parameter(
            'clear_costmap_service',
            '/global_costmap/clear_entirely_global_costmap',
        )
        self.declare_parameter(
            'lifecycle_manager_service',
            '/lifecycle_manager_semantic_planning/manage_nodes',
        )
        self.declare_parameter('mask_yaml_path', '')
        self.declare_parameter('mask_source', 'file')
        self.declare_parameter('mask_topic', '/semantic_mask')
        self.declare_parameter('topic_input_mode', 'fixture')
        self.declare_parameter('mask_producer_config_path', '')
        self.declare_parameter('synthetic_source_config_path', '')
        self.declare_parameter('recorded_bag_path', '')
        self.declare_parameter('recorded_unit_id', '')
        self.declare_parameter(
            'recorded_playback_start_offset_seconds',
            0.0,
        )
        self.declare_parameter(
            'producer_detection_active_duration_sec',
            -1.0,
        )
        self.declare_parameter('producer_observation_hold_sec', -1.0)
        self.declare_parameter('producer_observation_decay_sec', -1.0)
        self.declare_parameter('producer_depth_m', -1.0)
        self.declare_parameter('producer_depth_noise_std_m', -1.0)
        self.declare_parameter('producer_depth_invalid_fraction', -1.0)
        self.declare_parameter('producer_random_seed', -1)
        self.declare_parameter('producer_detection_score', -1.0)
        self.declare_parameter('producer_detection_class_id', '')
        self.declare_parameter('producer_person_count', -1)
        self.declare_parameter(
            'producer_detection_center_x_fraction',
            -1.0,
        )
        self.declare_parameter(
            'producer_detection_center_y_fraction',
            -1.0,
        )
        self.declare_parameter(
            'producer_person_spacing_y_fraction',
            -1.0,
        )
        self.declare_parameter(
            'producer_detection_publish_every_n_frames',
            -1,
        )
        self.declare_parameter(
            'producer_detection_timestamp_offset_sec',
            -1.0,
        )
        self.declare_parameter('producer_risk_value_scale', -1.0)
        self.declare_parameter('producer_risk_radius_scale', -1.0)
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('planner_config_path', '')
        self.declare_parameter('output_path', '/tmp/semantic_planning_ab.json')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('cost_mode', 'fuzzy')
        self.declare_parameter('task_urgency', 0)
        self.declare_parameter('avoidance_level', 70.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('start_x', 6.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_x', 20.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('settle_seconds', 1.5)
        self.declare_parameter('mask_publish_timeout_seconds', 10.0)
        self.declare_parameter('recovery_timeout_seconds', 0.0)
        self.declare_parameter('service_timeout_seconds', 60.0)
        self.declare_parameter('planning_timeout_seconds', 30.0)
        self.declare_parameter('code_revision', 'unknown')

        action_name = self.get_parameter('action_name').value
        parameter_service = self.get_parameter(
            'costmap_parameter_service'
        ).value
        clear_service = self.get_parameter('clear_costmap_service').value
        lifecycle_service = self.get_parameter(
            'lifecycle_manager_service'
        ).value
        self._action_client = ActionClient(
            self, ComputePathToPose, action_name
        )
        self._parameter_client = self.create_client(
            SetParameters, parameter_service
        )
        self._clear_client = self.create_client(
            ClearEntireCostmap, clear_service
        )
        self._lifecycle_client = self.create_client(
            ManageLifecycleNodes, lifecycle_service
        )
        self._mask_publisher = None
        self._received_topic_mask = None
        self._received_topic_mask_value_summary = None
        if self.get_parameter('mask_source').value == 'topic':
            mask_qos = QoSProfile(depth=1)
            mask_qos.reliability = ReliabilityPolicy.RELIABLE
            mask_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            mask_topic = self.get_parameter('mask_topic').value
            self._mask_subscription = self.create_subscription(
                OccupancyGrid,
                mask_topic,
                self._mask_callback,
                mask_qos,
            )
            if self.get_parameter('topic_input_mode').value == 'fixture':
                self._mask_publisher = self.create_publisher(
                    OccupancyGrid,
                    mask_topic,
                    mask_qos,
                )

    def _wait_for_interfaces(self) -> None:
        timeout = float(
            self.get_parameter('service_timeout_seconds').value
        )
        if not self._action_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError('ComputePathToPose action is unavailable')
        if not self._parameter_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('costmap parameter service is unavailable')
        if not self._clear_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('global costmap clear service is unavailable')
        if not self._lifecycle_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('lifecycle manager service is unavailable')

    def _spin_future(self, future, timeout: float, description: str):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f'timed out waiting for {description}')
        error = future.exception()
        if error is not None:
            raise RuntimeError(f'{description} failed: {error}')
        return future.result()

    def _configure_mask_layer(self, enabled: bool) -> None:
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                'mask_layer.enabled',
                value=enabled,
            ).to_parameter_msg(),
            Parameter(
                'mask_layer.cost_mode',
                value=str(self.get_parameter('cost_mode').value),
            ).to_parameter_msg(),
            Parameter(
                'mask_layer.task_urgency',
                value=int(self.get_parameter('task_urgency').value),
            ).to_parameter_msg(),
            Parameter(
                'mask_layer.avoidance_level',
                value=float(self.get_parameter('avoidance_level').value),
            ).to_parameter_msg(),
        ]
        timeout = float(
            self.get_parameter('service_timeout_seconds').value
        )
        response = self._spin_future(
            self._parameter_client.call_async(request),
            timeout,
            'mask parameter update',
        )
        if (
            len(response.results) != len(request.parameters)
            or any(not result.successful for result in response.results)
        ):
            reason = '; '.join(
                result.reason for result in response.results
                if not result.successful and result.reason
            )
            raise RuntimeError(f'mask parameter update was rejected: {reason}')

    def _clear_costmap(self) -> None:
        timeout = float(
            self.get_parameter('service_timeout_seconds').value
        )
        self._spin_future(
            self._clear_client.call_async(ClearEntireCostmap.Request()),
            timeout,
            'global costmap clear',
        )

    def _shutdown_managed_nodes(self) -> None:
        timeout = float(
            self.get_parameter('service_timeout_seconds').value
        )
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.SHUTDOWN
        response = self._spin_future(
            self._lifecycle_client.call_async(request),
            timeout,
            'managed node shutdown',
        )
        if not response.success:
            raise RuntimeError('lifecycle manager rejected managed node shutdown')

    def _mask_callback(self, message: OccupancyGrid) -> None:
        if message.header.frame_id != self.get_parameter('frame_id').value:
            return
        try:
            mask = mask_grid_from_occupancy_data(
                message.info.width,
                message.info.height,
                message.info.resolution,
                message.info.origin.position.x,
                message.info.origin.position.y,
                message.data,
            )
            value_summary = occupancy_value_summary(message.data)
        except ValueError:
            return
        self._received_topic_mask = mask
        self._received_topic_mask_value_summary = value_summary

    def _wait_for_topic_mask(
        self,
        expected_nonempty: bool | None,
        timeout_seconds: float | None = None,
    ):
        timeout = (
            float(self.get_parameter('mask_publish_timeout_seconds').value)
            if timeout_seconds is None else timeout_seconds
        )
        deadline = time.monotonic() + timeout
        while True:
            mask = self._received_topic_mask
            if mask is not None:
                is_nonempty = bool(mask.occupied.any())
                if expected_nonempty is None or is_nonempty == expected_nonempty:
                    return mask
            if time.monotonic() >= deadline:
                requirement = {
                    True: 'nonempty ',
                    False: 'empty ',
                    None: '',
                }[expected_nonempty]
                raise TimeoutError(
                    f'timed out waiting for {requirement}semantic mask'
                )
            rclpy.spin_once(self, timeout_sec=0.1)

    def _publish_topic_mask(self, mask):
        if self._mask_publisher is None:
            raise RuntimeError('topic mask publisher was not initialized')
        timeout = float(
            self.get_parameter('mask_publish_timeout_seconds').value
        )
        deadline = time.monotonic() + timeout
        while self._mask_publisher.get_subscription_count() < 1:
            if time.monotonic() >= deadline:
                raise TimeoutError('semantic mask topic has no subscriber')
            rclpy.spin_once(self, timeout_sec=0.1)

        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter('frame_id').value
        message.info.resolution = mask.resolution
        message.info.width = mask.width
        message.info.height = mask.height
        message.info.origin.position.x = mask.origin_x
        message.info.origin.position.y = mask.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = occupancy_data(mask)
        self._mask_publisher.publish(message)
        self.get_logger().info(
            'published %dx%d transient semantic mask on %s'
            % (
                mask.width,
                mask.height,
                self.get_parameter('mask_topic').value,
            )
        )
        transported_mask = self._wait_for_topic_mask(expected_nonempty=None)
        if (
            transported_mask.width != mask.width
            or transported_mask.height != mask.height
            or transported_mask.resolution != mask.resolution
            or transported_mask.origin_x != mask.origin_x
            or transported_mask.origin_y != mask.origin_y
            or not (
                transported_mask.occupied == mask.occupied
            ).all()
        ):
            raise RuntimeError('transported semantic mask differs from fixture')
        time.sleep(float(self.get_parameter('settle_seconds').value))
        return transported_mask

    def _pose(self, prefix: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('frame_id').value
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(
            self.get_parameter(prefix + '_x').value
        )
        pose.pose.position.y = float(
            self.get_parameter(prefix + '_y').value
        )
        yaw = float(self.get_parameter(prefix + '_yaw').value)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _compute_path(self) -> dict:
        goal = ComputePathToPose.Goal()
        goal.start = self._pose('start')
        goal.goal = self._pose('goal')
        goal.planner_id = self.get_parameter('planner_id').value
        goal.use_start = True
        timeout = float(
            self.get_parameter('planning_timeout_seconds').value
        )

        goal_handle = self._spin_future(
            self._action_client.send_goal_async(goal),
            timeout,
            'planner goal acceptance',
        )
        if not goal_handle.accepted:
            raise RuntimeError('planner rejected the goal')
        wrapped = self._spin_future(
            goal_handle.get_result_async(),
            timeout,
            'planner result',
        )
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f'planner returned action status {wrapped.status}'
            )

        poses = []
        for stamped in wrapped.result.path.poses:
            orientation = stamped.pose.orientation
            yaw = math.atan2(
                2.0 * (
                    orientation.w * orientation.z
                    + orientation.x * orientation.y
                ),
                1.0 - 2.0 * (
                    orientation.y ** 2 + orientation.z ** 2
                ),
            )
            poses.append((
                stamped.pose.position.x,
                stamped.pose.position.y,
                yaw,
            ))
        points = [(pose[0], pose[1]) for pose in poses]
        planning_time = wrapped.result.planning_time
        return {
            'points': points,
            'poses': poses,
            'planning_time_s': (
                float(planning_time.sec)
                + float(planning_time.nanosec) * 1.0e-9
            ),
        }

    def _plan_condition(self, name: str, enabled: bool, mask) -> dict:
        self.get_logger().info(
            'planning condition=%s, mask_layer.enabled=%s, '
            'cost_mode=%s, task_urgency=%d, avoidance_level=%.1f'
            % (
                name,
                enabled,
                self.get_parameter('cost_mode').value,
                int(self.get_parameter('task_urgency').value),
                float(self.get_parameter('avoidance_level').value),
            )
        )
        self._configure_mask_layer(enabled)
        self._clear_costmap()
        time.sleep(float(self.get_parameter('settle_seconds').value))
        planned = self._compute_path()
        metrics = path_metrics(planned['points'], mask)
        metrics.update(footprint_path_metrics(
            planned['poses'],
            self._footprint,
            mask,
        ))
        metrics['planning_time_s'] = planned['planning_time_s']
        return {
            'mask_layer_enabled': enabled,
            'cost_mode': (
                self.get_parameter('cost_mode').value
                if enabled else 'disabled'
            ),
            'metrics': metrics,
            'path': [
                {'x': point[0], 'y': point[1]}
                for point in planned['points']
            ],
        }

    def run(self) -> Path:
        """Run both conditions and write an atomic JSON report."""
        mask_yaml_value = str(
            self.get_parameter('mask_yaml_path').value
        ).strip()
        map_yaml_value = str(
            self.get_parameter('map_yaml_path').value
        ).strip()
        planner_config_value = str(
            self.get_parameter('planner_config_path').value
        ).strip()
        mask_source = str(
            self.get_parameter('mask_source').value
        ).strip()
        topic_input_mode = str(
            self.get_parameter('topic_input_mode').value
        ).strip()
        producer_config_value = str(
            self.get_parameter('mask_producer_config_path').value
        ).strip()
        synthetic_config_value = str(
            self.get_parameter('synthetic_source_config_path').value
        ).strip()
        recorded_bag_value = str(
            self.get_parameter('recorded_bag_path').value
        ).strip()
        recorded_unit_id = str(
            self.get_parameter('recorded_unit_id').value
        ).strip()
        cost_mode = str(self.get_parameter('cost_mode').value).strip()
        task_urgency = int(self.get_parameter('task_urgency').value)
        avoidance_level = float(
            self.get_parameter('avoidance_level').value
        )
        recovery_timeout = float(
            self.get_parameter('recovery_timeout_seconds').value
        )
        if not map_yaml_value:
            raise ValueError('map_yaml_path must not be empty')
        if not planner_config_value:
            raise ValueError('planner_config_path must not be empty')
        if mask_source not in ('file', 'topic'):
            raise ValueError("mask_source must be 'file' or 'topic'")
        if topic_input_mode not in ('fixture', 'external'):
            raise ValueError("topic_input_mode must be 'fixture' or 'external'")
        if cost_mode not in ('fuzzy', 'fixed', 'lethal'):
            raise ValueError(
                "cost_mode must be 'fuzzy', 'fixed', or 'lethal'"
            )
        if not 0 <= task_urgency <= 10:
            raise ValueError('task_urgency must be in [0, 10]')
        if not 0.0 <= avoidance_level <= 100.0:
            raise ValueError('avoidance_level must be in [0, 100]')
        if recovery_timeout < 0.0:
            raise ValueError('recovery_timeout_seconds must be nonnegative')
        if recovery_timeout > 0.0 and not (
            mask_source == 'topic' and topic_input_mode == 'external'
        ):
            raise ValueError(
                'mask recovery requires external topic input'
            )
        if bool(recorded_bag_value) != bool(recorded_unit_id):
            raise ValueError(
                'recorded_bag_path and recorded_unit_id must be set together'
            )
        if recorded_bag_value and not (
            mask_source == 'topic'
            and topic_input_mode == 'external'
            and not synthetic_config_value
        ):
            raise ValueError(
                'recorded bags require external topic input without synthetic data'
            )
        if mask_source == 'file' or topic_input_mode == 'fixture':
            if not mask_yaml_value:
                raise ValueError('mask_yaml_path must not be empty')
            mask = load_mask_grid(Path(mask_yaml_value))
        else:
            mask = None
        map_yaml_path = Path(map_yaml_value).expanduser().resolve()
        planner_config_path = Path(
            planner_config_value
        ).expanduser().resolve()
        self._footprint = load_planner_footprint(planner_config_path)
        producer_config_path = (
            Path(producer_config_value).expanduser().resolve()
            if producer_config_value else None
        )
        synthetic_config_path = (
            Path(synthetic_config_value).expanduser().resolve()
            if synthetic_config_value else None
        )
        recorded_bag = (
            inspect_recorded_bag(
                Path(recorded_bag_value),
                RECORDED_RGBD_TOPICS,
            )
            if recorded_bag_value else None
        )

        producer_runtime_parameters = None
        if producer_config_path:
            producer_runtime_parameters = {
                'observation_hold_sec': float(
                    self.get_parameter(
                        'producer_observation_hold_sec'
                    ).value
                ),
                'observation_decay_sec': float(
                    self.get_parameter(
                        'producer_observation_decay_sec'
                    ).value
                ),
                'risk_value_scale': float(
                    self.get_parameter('producer_risk_value_scale').value
                ),
                'risk_radius_scale': float(
                    self.get_parameter('producer_risk_radius_scale').value
                ),
            }
        if producer_runtime_parameters is not None and synthetic_config_path:
            producer_runtime_parameters.update({
                'detection_active_duration_sec': float(
                    self.get_parameter(
                        'producer_detection_active_duration_sec'
                    ).value
                ),
                'depth_m': float(
                    self.get_parameter('producer_depth_m').value
                ),
                'depth_noise_std_m': float(
                    self.get_parameter(
                        'producer_depth_noise_std_m'
                    ).value
                ),
                'depth_invalid_fraction': float(
                    self.get_parameter(
                        'producer_depth_invalid_fraction'
                    ).value
                ),
                'random_seed': int(
                    self.get_parameter('producer_random_seed').value
                ),
                'detection_score': float(
                    self.get_parameter('producer_detection_score').value
                ),
                'detection_class_id': str(
                    self.get_parameter('producer_detection_class_id').value
                ),
                'person_count': int(
                    self.get_parameter('producer_person_count').value
                ),
                'detection_center_x_fraction': float(
                    self.get_parameter(
                        'producer_detection_center_x_fraction'
                    ).value
                ),
                'detection_center_y_fraction': float(
                    self.get_parameter(
                        'producer_detection_center_y_fraction'
                    ).value
                ),
                'person_spacing_y_fraction': float(
                    self.get_parameter(
                        'producer_person_spacing_y_fraction'
                    ).value
                ),
                'detection_publish_every_n_frames': int(
                    self.get_parameter(
                        'producer_detection_publish_every_n_frames'
                    ).value
                ),
                'detection_timestamp_offset_sec': float(
                    self.get_parameter(
                        'producer_detection_timestamp_offset_sec'
                    ).value
                ),
            })

        self._wait_for_interfaces()
        if mask_source == 'topic':
            if topic_input_mode == 'fixture':
                mask = self._publish_topic_mask(mask)
            else:
                mask = self._wait_for_topic_mask(expected_nonempty=True)
                time.sleep(float(self.get_parameter('settle_seconds').value))
        if mask is None:
            raise RuntimeError('semantic mask is unavailable')
        baseline = self._plan_condition('baseline', False, mask)
        semantic = self._plan_condition('semantic', True, mask)
        recovered = None
        if recovery_timeout > 0.0:
            self.get_logger().info('waiting for semantic mask to become empty')
            self._wait_for_topic_mask(
                expected_nonempty=False,
                timeout_seconds=recovery_timeout,
            )
            time.sleep(float(self.get_parameter('settle_seconds').value))
            recovered = self._plan_condition(
                'recovered_after_mask_clear',
                True,
                mask,
            )
        start = self._pose('start').pose
        goal = self._pose('goal').pose
        report = {
            'schema_version': 1,
            'created_at_utc': datetime.now(timezone.utc).isoformat(),
            'code_revision': self.get_parameter('code_revision').value,
            'safety_scope': {
                'controller_started': False,
                'velocity_commands_published': False,
                'hardware_nodes_started': False,
                'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
                'ros_localhost_only': os.environ.get(
                    'ROS_LOCALHOST_ONLY', ''
                ),
            },
            'inputs': {
                'map_yaml_path': str(map_yaml_path),
                'map_yaml_sha256': sha256_file(map_yaml_path),
                'mask_yaml_path': (
                    str(mask.yaml_path) if mask.yaml_path else None
                ),
                'mask_yaml_sha256': (
                    sha256_file(mask.yaml_path) if mask.yaml_path else None
                ),
                'mask_image_path': (
                    str(mask.image_path) if mask.image_path else None
                ),
                'mask_image_sha256': (
                    sha256_file(mask.image_path) if mask.image_path else None
                ),
                'mask_grid': mask_geometry_summary(mask),
                'mask_value_summary': (
                    self._received_topic_mask_value_summary
                    if mask_source == 'topic' else None
                ),
                'planner_config_path': str(planner_config_path),
                'planner_config_sha256': sha256_file(planner_config_path),
                'robot_footprint': [list(point) for point in self._footprint],
                'planner_id': self.get_parameter('planner_id').value,
                'semantic_layer_parameters': {
                    'cost_mode': cost_mode,
                    'task_urgency': task_urgency,
                    'avoidance_level': avoidance_level,
                },
                'recovery_timeout_seconds': recovery_timeout,
                'mask_source': mask_source,
                'topic_input_mode': (
                    topic_input_mode if mask_source == 'topic' else None
                ),
                'mask_producer_config_path': (
                    str(producer_config_path)
                    if producer_config_path else None
                ),
                'mask_producer_config_sha256': (
                    sha256_file(producer_config_path)
                    if producer_config_path else None
                ),
                'synthetic_source_config_path': (
                    str(synthetic_config_path)
                    if synthetic_config_path else None
                ),
                'synthetic_source_config_sha256': (
                    sha256_file(synthetic_config_path)
                    if synthetic_config_path else None
                ),
                'recorded_unit_id': recorded_unit_id or None,
                'recorded_playback_start_offset_seconds': (
                    float(self.get_parameter(
                        'recorded_playback_start_offset_seconds'
                    ).value)
                    if recorded_unit_id else None
                ),
                'recorded_bag': recorded_bag,
                'mask_producer_runtime_parameters': (
                    producer_runtime_parameters
                ),
                'mask_topic': (
                    self.get_parameter('mask_topic').value
                    if mask_source == 'topic' else None
                ),
                'mask_topic_qos': (
                    {
                        'reliability': 'reliable',
                        'durability': 'transient_local',
                        'depth': 1,
                    }
                    if mask_source == 'topic' else None
                ),
                'start': {
                    'x': start.position.x,
                    'y': start.position.y,
                    'yaw': float(
                        self.get_parameter('start_yaw').value
                    ),
                },
                'goal': {
                    'x': goal.position.x,
                    'y': goal.position.y,
                    'yaw': float(
                        self.get_parameter('goal_yaw').value
                    ),
                },
            },
            'baseline': baseline,
            'semantic': semantic,
            'delta_semantic_minus_baseline': metric_delta(
                baseline['metrics'], semantic['metrics']
            ),
            'recovered_after_mask_clear': recovered,
            'delta_recovered_minus_baseline': (
                metric_delta(baseline['metrics'], recovered['metrics'])
                if recovered else None
            ),
        }

        output_path = Path(
            self.get_parameter('output_path').value
        ).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = output_path.with_name(output_path.name + '.tmp')
        temporary_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8',
        )
        os.replace(temporary_path, output_path)
        self.get_logger().info(f'wrote A/B report to {output_path}')
        self._shutdown_managed_nodes()
        return output_path


def main(args=None) -> None:
    """Run the A/B experiment process."""
    rclpy.init(args=args)
    node = SemanticPlanningAB()
    exit_code = 0
    try:
        node.run()
    except Exception as error:
        node.get_logger().error(f'A/B experiment failed: {error}')
        if node._lifecycle_client.service_is_ready():
            try:
                node._shutdown_managed_nodes()
            except Exception as shutdown_error:
                node.get_logger().error(
                    f'managed node shutdown also failed: {shutdown_error}'
                )
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)
