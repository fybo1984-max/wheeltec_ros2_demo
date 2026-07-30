#!/usr/bin/env python3

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuBiasCorrector(Node):
    def __init__(self):
        super().__init__('imu_bias_corrector')

        self.declare_parameter('calibration_samples', 120)
        self.declare_parameter('stationary_linear_threshold', 0.01)
        self.declare_parameter('stationary_angular_threshold', 0.02)
        self.declare_parameter('bias_update_alpha', 0.001)
        self.declare_parameter('gyro_scale_factor', 1.0)

        self.calibration_samples = int(
            self.get_parameter('calibration_samples').value
        )
        self.stationary_linear_threshold = float(
            self.get_parameter('stationary_linear_threshold').value
        )
        self.stationary_angular_threshold = float(
            self.get_parameter('stationary_angular_threshold').value
        )
        self.bias_update_alpha = float(
            self.get_parameter('bias_update_alpha').value
        )
        self.gyro_scale_factor = float(
            self.get_parameter('gyro_scale_factor').value
        )

        self.stationary = False
        self.samples = []
        self.gyro_z_bias = 0.0
        self.calibrated = False

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            20,
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            qos_profile_sensor_data,
        )
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu/data_corrected',
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Keep the robot stationary while collecting '
            f'{self.calibration_samples} IMU samples. '
            f'gyro_scale_factor={self.gyro_scale_factor:.4f}'
        )

    def odom_callback(self, msg):
        linear_speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )
        angular_speed = abs(msg.twist.twist.angular.z)
        self.stationary = (
            linear_speed < self.stationary_linear_threshold
            and angular_speed < self.stationary_angular_threshold
        )

    def imu_callback(self, msg):
        raw_gyro_z = msg.angular_velocity.z

        if not self.calibrated:
            if not self.stationary:
                if self.samples:
                    self.get_logger().warning(
                        'Robot moved during IMU calibration; restarting samples.'
                    )
                    self.samples.clear()
                return

            self.samples.append(raw_gyro_z)
            if len(self.samples) < self.calibration_samples:
                return

            self.gyro_z_bias = sum(self.samples) / len(self.samples)
            self.calibrated = True
            self.get_logger().info(
                'IMU calibration complete: '
                f'gyro_z_bias={self.gyro_z_bias:.8f} rad/s'
            )

        elif self.stationary:
            alpha = self.bias_update_alpha
            self.gyro_z_bias = (
                (1.0 - alpha) * self.gyro_z_bias
                + alpha * raw_gyro_z
            )

        corrected = copy.deepcopy(msg)
        corrected.angular_velocity.z = (
            raw_gyro_z - self.gyro_z_bias
        ) * self.gyro_scale_factor

        if corrected.angular_velocity_covariance[8] <= 0.0:
            corrected.angular_velocity_covariance[8] = 0.0001
        else:
            corrected.angular_velocity_covariance[8] *= (
                self.gyro_scale_factor ** 2
            )

        self.imu_publisher.publish(corrected)


def main(args=None):
    rclpy.init(args=args)
    node = ImuBiasCorrector()
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
