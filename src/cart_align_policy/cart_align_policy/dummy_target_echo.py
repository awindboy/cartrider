#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


class DummyTargetEchoNode(Node):
    def __init__(self) -> None:
        super().__init__('dummy_target_echo')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('target_topic', '/align/target_local')
        self.declare_parameter('motor_state_topic', '/rmd_state')
        self.declare_parameter(
            'motor_state_type',
            'cartrider_rmd_sdk/msg/MotorStateArray',
        )
        self.declare_parameter(
            'motor_state_item_type',
            'cartrider_rmd_sdk/msg/MotorState',
        )
        self.declare_parameter('wheel_cmd_topic', '/rmd_command')
        self.declare_parameter(
            'wheel_cmd_type',
            'cartrider_rmd_sdk/msg/MotorCommandArray',
        )

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.motor_state_topic = str(self.get_parameter('motor_state_topic').value)
        self.motor_state_type = str(self.get_parameter('motor_state_type').value)
        self.motor_state_item_type = str(
            self.get_parameter('motor_state_item_type').value
        )
        self.wheel_cmd_topic = str(self.get_parameter('wheel_cmd_topic').value)
        self.wheel_cmd_type = str(self.get_parameter('wheel_cmd_type').value)

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn(
                'publish_rate_hz must be > 0. Falling back to 10.0 Hz.'
            )
            self.publish_rate_hz = 10.0

        try:
            self.motor_state_msg_cls = get_message(self.motor_state_type)
        except Exception as exc:
            raise RuntimeError(
                f'Failed to load motor_state_type={self.motor_state_type}: {exc}'
            ) from exc

        try:
            self.motor_state_item_msg_cls = get_message(self.motor_state_item_type)
        except Exception as exc:
            raise RuntimeError(
                f'Failed to load motor_state_item_type={self.motor_state_item_type}: {exc}'
            ) from exc

        try:
            self.wheel_cmd_msg_cls = get_message(self.wheel_cmd_type)
        except Exception as exc:
            raise RuntimeError(
                f'Failed to load wheel_cmd_type={self.wheel_cmd_type}: {exc}'
            ) from exc

        self.target_pub = self.create_publisher(
            PoseStamped,
            self.target_topic,
            10,
        )
        self.motor_pub = self.create_publisher(
            self.motor_state_msg_cls,
            self.motor_state_topic,
            10,
        )
        self.wheel_sub = self.create_subscription(
            self.wheel_cmd_msg_cls,
            self.wheel_cmd_topic,
            self._wheel_cmd_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_dummy_target,
        )
        self.start_time = self.get_clock().now()
        self._wheel_log_count = 0

        self.get_logger().info(
            'Dummy node started: publishing %s and %s at %.2f Hz, '
            'echoing %s as %s.'
            % (
                self.target_topic,
                self.motor_state_topic,
                self.publish_rate_hz,
                self.wheel_cmd_topic,
                self.wheel_cmd_type,
            )
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

        motor_msg = self.motor_state_msg_cls()
        if hasattr(motor_msg, 'stamp'):
            motor_msg.stamp = now.to_msg()

        left_state = self.motor_state_item_msg_cls()
        left_state.id = 1
        left_state.speed = 0.4 * math.sin(0.6 * elapsed)

        right_state = self.motor_state_item_msg_cls()
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
