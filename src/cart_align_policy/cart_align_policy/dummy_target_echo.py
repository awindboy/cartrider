#!/usr/bin/env python3
import math

import rclpy
from cart_align_msgs.msg import MotorState, MotorStateArray
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


class DummyTargetEchoNode(Node):
    def __init__(self) -> None:
        super().__init__('dummy_target_echo')
        self.wheel_cmd_type = 'cartrider_rmd_sdk/msg/MotorCommandArray'
        self.wheel_cmd_msg_cls = get_message(self.wheel_cmd_type)

        self.target_pub = self.create_publisher(
            PoseStamped,
            '/align/target_local',
            10,
        )
        self.motor_pub = self.create_publisher(
            MotorStateArray,
            '/rmd_state',
            10,
        )
        self.wheel_sub = self.create_subscription(
            self.wheel_cmd_msg_cls,
            '/rmd_command',
            self._wheel_cmd_callback,
            10,
        )
        self.timer = self.create_timer(0.1, self._publish_dummy_target)
        self.start_time = self.get_clock().now()
        self._wheel_log_count = 0

        self.get_logger().info(
            'Dummy node started: publishing /align/target_local and /rmd_state '
            'at 10 Hz, echoing /rmd_command as MotorCommandArray.'
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

    def _wheel_cmd_callback(self, msg) -> None:
        self._wheel_log_count += 1
        if self._wheel_log_count % 10 != 0:
            return

        left_target = None
        right_target = None
        for cmd in msg.commands:
            if cmd.id == 1:
                left_target = cmd.target
            elif cmd.id == 2:
                right_target = cmd.target

        self.get_logger().info(
            (
                'wheel_cmd(MotorCommandArray): id1(left)=%.3f rad/s, '
                'id2(right)=%.3f rad/s'
            )
            % (
                left_target if left_target is not None else float('nan'),
                right_target if right_target is not None else float('nan'),
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
