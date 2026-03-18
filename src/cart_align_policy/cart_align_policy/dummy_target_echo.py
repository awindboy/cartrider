#!/usr/bin/env python3
import math

import rclpy
from cart_align_msgs.msg import AlignTargetLocal, WheelCmd
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class DummyTargetEchoNode(Node):
    def __init__(self) -> None:
        super().__init__('dummy_target_echo')
        self.target_pub = self.create_publisher(
            AlignTargetLocal,
            '/align/target_local',
            10,
        )
        self.wheel_sub = self.create_subscription(
            WheelCmd,
            '/align/wheel_cmd',
            self._wheel_cmd_callback,
            10,
        )
        self.timer = self.create_timer(0.1, self._publish_dummy_target)
        self.start_time = self.get_clock().now()
        self._wheel_log_count = 0

        self.get_logger().info(
            'Dummy node started: publishing /align/target_local at 10 Hz and '
            'echoing /align/wheel_cmd.'
        )

    def _publish_dummy_target(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        msg = AlignTargetLocal()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.target_x_local = 1.0
        msg.target_y_local = 0.3 * math.sin(0.7 * elapsed)
        msg.heading_error = 0.4 * math.sin(0.5 * elapsed)

        self.target_pub.publish(msg)

    def _wheel_cmd_callback(self, msg: WheelCmd) -> None:
        self._wheel_log_count += 1
        if self._wheel_log_count % 10 != 0:
            return

        self.get_logger().info(
            (
                'wheel_cmd: cmd_vel_r=%.3f rad/s, cmd_vel_l=%.3f rad/s, '
                'left_action_raw=%.3f, right_action_raw=%.3f'
            )
            % (
                msg.cmd_vel_r,
                msg.cmd_vel_l,
                msg.left_action_raw,
                msg.right_action_raw,
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
