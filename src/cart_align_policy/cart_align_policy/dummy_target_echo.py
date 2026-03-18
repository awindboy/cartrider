#!/usr/bin/env python3
import math

import rclpy
from cart_align_msgs.msg import MotorState, MotorStateArray
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class DummyTargetEchoNode(Node):
    def __init__(self) -> None:
        super().__init__('dummy_target_echo')
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
        self.wheel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._wheel_cmd_callback,
            10,
        )
        self.timer = self.create_timer(0.1, self._publish_dummy_target)
        self.start_time = self.get_clock().now()
        self._wheel_log_count = 0

        self.get_logger().info(
            'Dummy node started: publishing /align/target_local and /motor_states '
            'at 10 Hz, echoing /cmd_vel.'
        )

    def _publish_dummy_target(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        target_msg = PoseStamped()
        target_msg.header.stamp = now.to_msg()
        target_msg.header.frame_id = 'base_link'
        target_msg.pose.position.x = 1.0
        target_msg.pose.position.y = 0.3 * math.sin(0.7 * elapsed)
        heading_error = 0.4 * math.sin(0.5 * elapsed)
        target_msg.pose.orientation.w = math.cos(heading_error * 0.5)
        target_msg.pose.orientation.z = math.sin(heading_error * 0.5)
        self.target_pub.publish(target_msg)

        motor_msg = MotorStateArray()
        motor_msg.stamp = now.to_msg()
        left_state = MotorState()
        left_state.id = 1
        left_state.speed = 0.4 * math.sin(0.6 * elapsed)
        right_state = MotorState()
        right_state.id = 2
        right_state.speed = 0.4 * math.sin(0.6 * elapsed + 0.3)
        motor_msg.states = [left_state, right_state]
        self.motor_pub.publish(motor_msg)

    def _wheel_cmd_callback(self, msg: Twist) -> None:
        self._wheel_log_count += 1
        if self._wheel_log_count % 10 != 0:
            return

        self.get_logger().info(
            (
                'wheel_cmd(Twist): angular.x(cmd_vel_r)=%.3f rad/s, '
                'angular.y(cmd_vel_l)=%.3f rad/s'
            )
            % (
                msg.angular.x,
                msg.angular.y,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyTargetEchoNode()
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
