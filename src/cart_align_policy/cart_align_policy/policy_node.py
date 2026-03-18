#!/usr/bin/env python3
import os
from typing import Optional

import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from cart_align_msgs.msg import AlignTargetLocal, WheelCmd
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class CartAlignPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__('cart_align_policy_node')

        self.declare_parameter('model_path', self._default_model_path())
        self.declare_parameter('left_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_joint_name', 'right_wheel_joint')
        self.declare_parameter('action_scale', 4.0055306333)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('target_timeout_sec', 0.3)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        self.model_path = str(self.get_parameter('model_path').value)
        self.left_joint_name = str(self.get_parameter('left_joint_name').value)
        self.right_joint_name = str(self.get_parameter('right_joint_name').value)
        self.action_scale = float(self.get_parameter('action_scale').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)

        if self.control_rate_hz <= 0.0:
            self.get_logger().warn(
                'control_rate_hz must be > 0. Falling back to 10.0 Hz.'
            )
            self.control_rate_hz = 10.0

        if self.target_timeout_sec <= 0.0:
            self.get_logger().warn(
                'target_timeout_sec must be > 0. Falling back to 0.3 sec.'
            )
            self.target_timeout_sec = 0.3

        self.latest_target: Optional[AlignTargetLocal] = None
        self.last_target_rx_time = None
        self.left_wheel_joint_vel: Optional[float] = None
        self.right_wheel_joint_vel: Optional[float] = None
        self._warn_timestamps = {}

        self._load_model()

        self.target_sub = self.create_subscription(
            AlignTargetLocal,
            '/align/target_local',
            self._target_callback,
            10,
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.wheel_cmd_pub = self.create_publisher(
            WheelCmd,
            '/align/wheel_cmd',
            10,
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_callback,
        )

        self.get_logger().info(
            'Policy node started: model_path=%s, left_joint=%s, right_joint=%s, '
            'rate=%.2fHz, timeout=%.3fs, action_scale=%.6f'
            % (
                self.model_path,
                self.left_joint_name,
                self.right_joint_name,
                self.control_rate_hz,
                self.target_timeout_sec,
                self.action_scale,
            )
        )

    def _default_model_path(self) -> str:
        try:
            share_dir = get_package_share_directory('cart_align_policy')
            candidate = os.path.join(share_dir, 'models', 'policy.onnx')
            if os.path.isfile(candidate):
                return candidate
        except Exception:
            pass
        return '/home/kwon/Documents/policy.onnx'

    def _load_model(self) -> None:
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f'ONNX model not found: {self.model_path}'
            )

        providers = ort.get_available_providers()
        if 'CUDAExecutionProvider' in providers:
            selected_providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        elif 'CPUExecutionProvider' in providers:
            selected_providers = ['CPUExecutionProvider']
        else:
            selected_providers = providers

        self.session = ort.InferenceSession(
            self.model_path,
            providers=selected_providers,
        )
        active_providers = self.session.get_providers()

        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if not inputs:
            raise RuntimeError('ONNX model has no inputs.')
        if not outputs:
            raise RuntimeError('ONNX model has no outputs.')

        self.input_name = inputs[0].name
        self.output_name = outputs[0].name

        in_shape = tuple(inputs[0].shape)
        out_shape = tuple(outputs[0].shape)

        if (
            'CUDAExecutionProvider' in selected_providers
            and 'CUDAExecutionProvider' not in active_providers
        ):
            self.get_logger().warn(
                'CUDAExecutionProvider requested but not active. '
                f'Falling back providers={active_providers}'
            )

        self.get_logger().info(
            f'Loaded ONNX model with input={self.input_name} shape={in_shape}, '
            f'output={self.output_name} shape={out_shape}, '
            f'requested_providers={selected_providers}, '
            f'active_providers={active_providers}'
        )

    def _target_callback(self, msg: AlignTargetLocal) -> None:
        self.latest_target = msg
        self.last_target_rx_time = self.get_clock().now()

    def _joint_state_callback(self, msg: JointState) -> None:
        if not msg.name or not msg.velocity:
            return

        velocity_map = {
            name: msg.velocity[idx]
            for idx, name in enumerate(msg.name)
            if idx < len(msg.velocity)
        }

        if self.left_joint_name not in velocity_map or self.right_joint_name not in velocity_map:
            self._warn_throttle(
                'missing_joint_name',
                (
                    'JointState does not include configured wheel joints '
                    f'({self.left_joint_name}, {self.right_joint_name}).'
                ),
                2.0,
            )
            return

        self.left_wheel_joint_vel = float(velocity_map[self.left_joint_name])
        self.right_wheel_joint_vel = float(velocity_map[self.right_joint_name])

    def _control_callback(self) -> None:
        now = self.get_clock().now()

        if self.latest_target is None or self.last_target_rx_time is None:
            self._publish_zero('waiting_target')
            return

        dt_target = (now - self.last_target_rx_time).nanoseconds * 1e-9
        if dt_target > self.target_timeout_sec:
            self._publish_zero('stale_target')
            return

        if self.left_wheel_joint_vel is None or self.right_wheel_joint_vel is None:
            self._publish_zero('waiting_joint_state')
            return

        obs = np.array(
            [
                [
                    self.latest_target.target_x_local,
                    self.latest_target.target_y_local,
                    self.latest_target.heading_error,
                    self.left_wheel_joint_vel,
                    self.right_wheel_joint_vel,
                ]
            ],
            dtype=np.float32,
        )

        try:
            inference_out = self.session.run(
                [self.output_name],
                {self.input_name: obs},
            )[0]
        except Exception as exc:
            self._warn_throttle(
                'inference_failed',
                f'ONNX inference failed: {exc}',
                1.0,
            )
            self._publish_zero('inference_failed')
            return

        actions = np.asarray(inference_out, dtype=np.float32).reshape(-1)
        if actions.size < 2:
            self._warn_throttle(
                'invalid_output',
                f'Expected at least 2 actions, got shape={inference_out.shape}',
                1.0,
            )
            self._publish_zero('invalid_output')
            return

        left_action_raw = float(actions[0])
        right_action_raw = float(actions[1])

        left_action = float(np.clip(left_action_raw, -1.0, 1.0))
        right_action = float(np.clip(right_action_raw, -1.0, 1.0))

        cmd_vel_l = left_action * self.action_scale
        cmd_vel_r = right_action * self.action_scale

        if self.invert_left:
            cmd_vel_l *= -1.0
        if self.invert_right:
            cmd_vel_r *= -1.0

        self._publish_wheel_cmd(
            cmd_vel_r=cmd_vel_r,
            cmd_vel_l=cmd_vel_l,
            left_action_raw=left_action_raw,
            right_action_raw=right_action_raw,
        )

    def _publish_zero(self, reason_key: str) -> None:
        self._warn_throttle(
            reason_key,
            f'Publishing zero command due to: {reason_key}',
            2.0,
        )
        self._publish_wheel_cmd(
            cmd_vel_r=0.0,
            cmd_vel_l=0.0,
            left_action_raw=0.0,
            right_action_raw=0.0,
        )

    def _publish_wheel_cmd(
        self,
        cmd_vel_r: float,
        cmd_vel_l: float,
        left_action_raw: float,
        right_action_raw: float,
    ) -> None:
        msg = WheelCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.latest_target is not None:
            msg.header.frame_id = self.latest_target.header.frame_id

        msg.cmd_vel_r = float(cmd_vel_r)
        msg.cmd_vel_l = float(cmd_vel_l)
        msg.left_action_raw = float(left_action_raw)
        msg.right_action_raw = float(right_action_raw)
        self.wheel_cmd_pub.publish(msg)

    def _warn_throttle(self, key: str, text: str, period_sec: float) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        last = self._warn_timestamps.get(key, -1.0e12)
        if now_sec - last >= period_sec:
            self.get_logger().warn(text)
            self._warn_timestamps[key] = now_sec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartAlignPolicyNode()
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
