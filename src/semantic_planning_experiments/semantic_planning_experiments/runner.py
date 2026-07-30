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
import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from semantic_planning_experiments.metrics import (
    load_mask_grid,
    metric_delta,
    path_metrics,
    sha256_file,
)


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
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('planner_config_path', '')
        self.declare_parameter('output_path', '/tmp/semantic_planning_ab.json')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('start_x', 6.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_x', 20.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('settle_seconds', 1.5)
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

    def _set_mask_enabled(self, enabled: bool) -> None:
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                'mask_layer.enabled',
                value=enabled,
            ).to_parameter_msg()
        ]
        timeout = float(
            self.get_parameter('service_timeout_seconds').value
        )
        response = self._spin_future(
            self._parameter_client.call_async(request),
            timeout,
            'mask parameter update',
        )
        if len(response.results) != 1 or not response.results[0].successful:
            reason = response.results[0].reason if response.results else ''
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

        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in wrapped.result.path.poses
        ]
        planning_time = wrapped.result.planning_time
        return {
            'points': points,
            'planning_time_s': (
                float(planning_time.sec)
                + float(planning_time.nanosec) * 1.0e-9
            ),
        }

    def _plan_condition(self, name: str, enabled: bool, mask) -> dict:
        self.get_logger().info(
            f'planning condition={name}, mask_layer.enabled={enabled}'
        )
        self._set_mask_enabled(enabled)
        self._clear_costmap()
        time.sleep(float(self.get_parameter('settle_seconds').value))
        planned = self._compute_path()
        metrics = path_metrics(planned['points'], mask)
        metrics['planning_time_s'] = planned['planning_time_s']
        return {
            'mask_layer_enabled': enabled,
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
        if not mask_yaml_value:
            raise ValueError('mask_yaml_path must not be empty')
        if not map_yaml_value:
            raise ValueError('map_yaml_path must not be empty')
        if not planner_config_value:
            raise ValueError('planner_config_path must not be empty')
        mask_yaml_path = Path(mask_yaml_value)
        map_yaml_path = Path(map_yaml_value).expanduser().resolve()
        planner_config_path = Path(
            planner_config_value
        ).expanduser().resolve()
        mask = load_mask_grid(mask_yaml_path)

        self._wait_for_interfaces()
        baseline = self._plan_condition('baseline', False, mask)
        semantic = self._plan_condition('semantic', True, mask)
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
                'mask_yaml_path': str(mask.yaml_path),
                'mask_yaml_sha256': sha256_file(mask.yaml_path),
                'mask_image_path': str(mask.image_path),
                'mask_image_sha256': sha256_file(mask.image_path),
                'planner_config_path': str(planner_config_path),
                'planner_config_sha256': sha256_file(planner_config_path),
                'planner_id': self.get_parameter('planner_id').value,
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
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)
