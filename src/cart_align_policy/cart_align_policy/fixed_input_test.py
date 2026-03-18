#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from cart_align_msgs.msg import MotorState, MotorStateArray
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class FixedInputTestNode(Node):
    def __init__(self) -> None:
        super().__init__('fixed_input_test_node')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('target_x_local', 1.0)
        self.declare_parameter('target_y_local', 0.0)
        self.declare_parameter('heading_error', 0.0)
        self.declare_parameter('left_motor_vel', 0.0)
        self.declare_parameter('right_motor_vel', 0.0)
        self.declare_parameter('log_wheel_cmd', True)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.target_x_local = float(self.get_parameter('target_x_local').value)
        self.target_y_local = float(self.get_parameter('target_y_local').value)
        self.heading_error = float(self.get_parameter('heading_error').value)
        self.left_motor_vel = float(self.get_parameter('left_motor_vel').value)
        self.right_motor_vel = float(self.get_parameter('right_motor_vel').value)
        self.log_wheel_cmd = bool(self.get_parameter('log_wheel_cmd').value)

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn(
                'publish_rate_hz must be > 0. Falling back to 10.0 Hz.'
            )
            self.publish_rate_hz = 10.0

        self.target_pub = self.create_publisher(
            PoseStamped,
            '/align/target_local',
            10,
        )
        self.motor_pub = self.create_publisher(
            MotorStateArray,
            '/motor_states',
            10,
        )

        self.wheel_sub: Optional[object] = None
        self._wheel_count = 0
        if self.log_wheel_cmd:
            self.wheel_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self._wheel_cmd_callback,
                10,
            )

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            'Fixed input test node started: rate=%.2fHz, '
            'target=(x=%.3f, y=%.3f, heading_err=%.3f), '
            'motor=(left=%.3f, right=%.3f)'
            % (
                self.publish_rate_hz,
                self.target_x_local,
                self.target_y_local,
                self.heading_error,
                self.left_motor_vel,
                self.right_motor_vel,
            )
        )

    def _on_timer(self) -> None:
        if not self.context.ok():
            return

        target_msg = PoseStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'base_link'
        target_msg.pose.position.x = self.target_x_local
        target_msg.pose.position.y = self.target_y_local
        target_msg.pose.orientation.w = math.cos(self.heading_error * 0.5)
        target_msg.pose.orientation.z = math.sin(self.heading_error * 0.5)
        try:
            self.target_pub.publish(target_msg)
        except Exception:
            return

        motor_msg = MotorStateArray()
        motor_msg.stamp = self.get_clock().now().to_msg()
        left_state = MotorState()
        left_state.id = 1
        left_state.speed = self.left_motor_vel
        right_state = MotorState()
        right_state.id = 2
        right_state.speed = self.right_motor_vel
        motor_msg.states = [left_state, right_state]
        try:
            self.motor_pub.publish(motor_msg)
        except Exception:
            return

    def _wheel_cmd_callback(self, msg: Twist) -> None:
        self._wheel_count += 1
        if self._wheel_count % 10 != 0:
            return

        self.get_logger().info(
            (
                'wheel_cmd(Twist): angular.x(cmd_vel_r)=%.3f, '
                'angular.y(cmd_vel_l)=%.3f'
            )
            % (
                msg.angular.x,
                msg.angular.y,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedInputTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node.context.ok():
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
