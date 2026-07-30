#!/usr/bin/env python3

import copy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class OdomMappingCorrector(Node):
    def __init__(self):
        super().__init__('odom_mapping_corrector')

        self.declare_parameter('yaw_scale_positive', 1.0)
        self.declare_parameter('yaw_scale_negative', 1.0)
        self.declare_parameter('linear_velocity_variance', 0.01)
        self.declare_parameter('yaw_velocity_variance', 0.005)

        self.yaw_scale_positive = float(
            self.get_parameter('yaw_scale_positive').value
        )
        self.yaw_scale_negative = float(
            self.get_parameter('yaw_scale_negative').value
        )
        self.linear_velocity_variance = float(
            self.get_parameter('linear_velocity_variance').value
        )
        self.yaw_velocity_variance = float(
            self.get_parameter('yaw_velocity_variance').value
        )

        self.publisher = self.create_publisher(
            Odometry,
            '/odom_mapping',
            20,
        )
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            20,
        )

        self.get_logger().info(
            'Mapping odometry correction active: '
            f'left={self.yaw_scale_positive:.4f}, '
            f'right={self.yaw_scale_negative:.4f}'
        )

    def odom_callback(self, msg):
        corrected = copy.deepcopy(msg)
        yaw_rate = msg.twist.twist.angular.z
        scale = (
            self.yaw_scale_positive
            if yaw_rate >= 0.0
            else self.yaw_scale_negative
        )
        corrected.twist.twist.angular.z = yaw_rate * scale

        corrected.twist.covariance[0] = self.linear_velocity_variance
        corrected.twist.covariance[7] = self.linear_velocity_variance
        corrected.twist.covariance[35] = self.yaw_velocity_variance
        self.publisher.publish(corrected)


def main(args=None):
    rclpy.init(args=args)
    node = OdomMappingCorrector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
