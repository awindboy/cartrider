#!/usr/bin/env python3
from typing import Optional

import rclpy
from cart_align_msgs.msg import AlignTargetLocal, WheelCmd
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FixedInputTestNode(Node):
    def __init__(self) -> None:
        super().__init__('fixed_input_test_node')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('target_x_local', 1.0)
        self.declare_parameter('target_y_local', 0.0)
        self.declare_parameter('heading_error', 0.0)
        self.declare_parameter('left_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_joint_name', 'right_wheel_joint')
        self.declare_parameter('left_wheel_joint_vel', 0.0)
        self.declare_parameter('right_wheel_joint_vel', 0.0)
        self.declare_parameter('log_wheel_cmd', True)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.target_x_local = float(self.get_parameter('target_x_local').value)
        self.target_y_local = float(self.get_parameter('target_y_local').value)
        self.heading_error = float(self.get_parameter('heading_error').value)
        self.left_joint_name = str(self.get_parameter('left_joint_name').value)
        self.right_joint_name = str(self.get_parameter('right_joint_name').value)
        self.left_wheel_joint_vel = float(
            self.get_parameter('left_wheel_joint_vel').value
        )
        self.right_wheel_joint_vel = float(
            self.get_parameter('right_wheel_joint_vel').value
        )
        self.log_wheel_cmd = bool(self.get_parameter('log_wheel_cmd').value)

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn(
                'publish_rate_hz must be > 0. Falling back to 10.0 Hz.'
            )
            self.publish_rate_hz = 10.0

        self.target_pub = self.create_publisher(
            AlignTargetLocal,
            '/align/target_local',
            10,
        )
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10,
        )

        self.wheel_sub: Optional[object] = None
        self._wheel_count = 0
        if self.log_wheel_cmd:
            self.wheel_sub = self.create_subscription(
                WheelCmd,
                '/align/wheel_cmd',
                self._wheel_cmd_callback,
                10,
            )

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            'Fixed input test node started: rate=%.2fHz, target=(%.3f, %.3f, %.3f), '
            'joint=(%s: %.3f, %s: %.3f)'
            % (
                self.publish_rate_hz,
                self.target_x_local,
                self.target_y_local,
                self.heading_error,
                self.left_joint_name,
                self.left_wheel_joint_vel,
                self.right_joint_name,
                self.right_wheel_joint_vel,
            )
        )

    def _on_timer(self) -> None:
        now_msg = self.get_clock().now().to_msg()

        target_msg = AlignTargetLocal()
        target_msg.header.stamp = now_msg
        target_msg.header.frame_id = self.frame_id
        target_msg.target_x_local = self.target_x_local
        target_msg.target_y_local = self.target_y_local
        target_msg.heading_error = self.heading_error
        self.target_pub.publish(target_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = now_msg
        joint_msg.name = [self.left_joint_name, self.right_joint_name]
        joint_msg.velocity = [self.left_wheel_joint_vel, self.right_wheel_joint_vel]
        self.joint_pub.publish(joint_msg)

    def _wheel_cmd_callback(self, msg: WheelCmd) -> None:
        self._wheel_count += 1
        if self._wheel_count % 10 != 0:
            return

        self.get_logger().info(
            (
                'wheel_cmd: cmd_vel_r=%.3f, cmd_vel_l=%.3f, '
                'raw=(L %.3f, R %.3f)'
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
    node = FixedInputTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
