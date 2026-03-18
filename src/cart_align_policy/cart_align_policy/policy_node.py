#!/usr/bin/env python3
import math
import os
from typing import Optional

import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


class CartAlignPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__('cart_align_policy_node')

        self.declare_parameter('model_path', self._default_model_path())
        self.declare_parameter('target_topic', '/align/target_local')
        self.declare_parameter('motor_state_topic', '/rmd_state')
        self.declare_parameter(
            'motor_state_type',
            'cartrider_rmd_sdk/msg/MotorStateArray',
        )
        self.declare_parameter('wheel_cmd_topic', '/rmd_command')
        self.declare_parameter(
            'wheel_cmd_type',
            'cartrider_rmd_sdk/msg/MotorCommandArray',
        )
        self.declare_parameter(
            'wheel_cmd_item_type',
            'cartrider_rmd_sdk/msg/MotorCommand',
        )
        self.declare_parameter('action_scale', 4.0055306333)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('target_timeout_sec', 1000.0)
        self.declare_parameter('motor_timeout_sec', 1000.0)
        self.declare_parameter('target_xy_stop_tolerance_m', 0.01)
        self.declare_parameter('target_yaw_stop_tolerance_deg', 5.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('left_motor_id', 1)
        self.declare_parameter('right_motor_id', 2)

        self.model_path = str(self.get_parameter('model_path').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.motor_state_topic = str(self.get_parameter('motor_state_topic').value)
        self.motor_state_type = str(self.get_parameter('motor_state_type').value)
        self.wheel_cmd_topic = str(self.get_parameter('wheel_cmd_topic').value)
        self.wheel_cmd_type = str(self.get_parameter('wheel_cmd_type').value)
        self.wheel_cmd_item_type = str(
            self.get_parameter('wheel_cmd_item_type').value
        )
        self.action_scale = float(self.get_parameter('action_scale').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.motor_timeout_sec = float(self.get_parameter('motor_timeout_sec').value)
        self.target_xy_stop_tolerance_m = float(
            self.get_parameter('target_xy_stop_tolerance_m').value
        )
        self.target_yaw_stop_tolerance_deg = float(
            self.get_parameter('target_yaw_stop_tolerance_deg').value
        )
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)
        self.left_motor_id = int(self.get_parameter('left_motor_id').value)
        self.right_motor_id = int(self.get_parameter('right_motor_id').value)

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

        if self.motor_timeout_sec <= 0.0:
            self.get_logger().warn(
                'motor_timeout_sec must be > 0. Falling back to 0.3 sec.'
            )
            self.motor_timeout_sec = 0.3

        if self.target_xy_stop_tolerance_m < 0.0:
            self.get_logger().warn(
                'target_xy_stop_tolerance_m must be >= 0. Falling back to 0.01 m.'
            )
            self.target_xy_stop_tolerance_m = 0.01

        if self.target_yaw_stop_tolerance_deg < 0.0:
            self.get_logger().warn(
                'target_yaw_stop_tolerance_deg must be >= 0. Falling back to 5.0 deg.'
            )
            self.target_yaw_stop_tolerance_deg = 5.0
        self.target_yaw_stop_tolerance_rad = math.radians(
            self.target_yaw_stop_tolerance_deg
        )

        self.latest_target: Optional[PoseStamped] = None
        self.last_target_rx_time = None
        self.left_motor_speed: Optional[float] = None
        self.right_motor_speed: Optional[float] = None
        self.last_motor_rx_time = None
        self._warn_timestamps = {}

        self._load_model()
        self._load_motor_state_type()
        self._load_wheel_cmd_type()

        self.target_sub = self.create_subscription(
            PoseStamped,
            self.target_topic,
            self._target_callback,
            10,
        )
        self.motor_state_sub = self.create_subscription(
            self.motor_state_msg_cls,
            self.motor_state_topic,
            self._motor_state_callback,
            10,
        )
        self.wheel_cmd_pub = self.create_publisher(
            self.wheel_cmd_msg_cls,
            self.wheel_cmd_topic,
            10,
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_callback,
        )

        self.get_logger().info(
            'Policy node started: model_path=%s, target_topic=%s, '
            'motor_state_topic=%s, motor_state_type=%s, wheel_cmd_topic=%s, '
            'wheel_cmd_type=%s, wheel_cmd_item_type=%s, '
            'left_motor_id=%d, right_motor_id=%d, '
            'rate=%.2fHz, '
            'target_timeout=%.3fs, motor_timeout=%.3fs, '
            'target_xy_stop_tolerance=%.4fm, '
            'target_yaw_stop_tolerance=%.2fdeg, action_scale=%.6f'
            % (
                self.model_path,
                self.target_topic,
                self.motor_state_topic,
                self.motor_state_type,
                self.wheel_cmd_topic,
                self.wheel_cmd_type,
                self.wheel_cmd_item_type,
                self.left_motor_id,
                self.right_motor_id,
                self.control_rate_hz,
                self.target_timeout_sec,
                self.motor_timeout_sec,
                self.target_xy_stop_tolerance_m,
                self.target_yaw_stop_tolerance_deg,
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
        if 'CPUExecutionProvider' not in providers:
            raise RuntimeError(
                f'CPUExecutionProvider is not available. available={providers}'
            )

        selected_providers = ['CPUExecutionProvider']

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

        self.get_logger().info(
            f'Loaded ONNX model with input={self.input_name} shape={in_shape}, '
            f'output={self.output_name} shape={out_shape}, '
            f'providers={active_providers}'
        )

    def _load_motor_state_type(self) -> None:
        try:
            self.motor_state_msg_cls = get_message(self.motor_state_type)
        except Exception as exc:
            raise RuntimeError(
                f'Failed to load motor_state_type={self.motor_state_type}: {exc}'
            ) from exc

    def _load_wheel_cmd_type(self) -> None:
        try:
            self.wheel_cmd_msg_cls = get_message(self.wheel_cmd_type)
        except Exception as exc:
            raise RuntimeError(
                f'Failed to load wheel_cmd_type={self.wheel_cmd_type}: {exc}'
            ) from exc

        try:
            self.wheel_cmd_item_msg_cls = get_message(self.wheel_cmd_item_type)
        except Exception as exc:
            raise RuntimeError(
                f'Failed to load wheel_cmd_item_type={self.wheel_cmd_item_type}: {exc}'
            ) from exc

    def _target_callback(self, msg: PoseStamped) -> None:
        self.latest_target = msg
        self.last_target_rx_time = self.get_clock().now()

    def _motor_state_callback(self, msg) -> None:
        if not hasattr(msg, 'states'):
            self._warn_throttle(
                'invalid_motor_state_msg',
                f'{self.motor_state_type} has no "states" field.',
                2.0,
            )
            return

        left_speed = None
        right_speed = None
        for state in msg.states:
            if not hasattr(state, 'id') or not hasattr(state, 'speed'):
                continue
            if state.id == 1:
                left_speed = float(state.speed)
            elif state.id == 2:
                right_speed = float(state.speed)

        if left_speed is None or right_speed is None:
            self._warn_throttle(
                'missing_motor_ids',
                'MotorStateArray must include id=1(left), id=2(right).',
                2.0,
            )
            return

        self.left_motor_speed = left_speed
        self.right_motor_speed = right_speed
        self.last_motor_rx_time = self.get_clock().now()

    def _control_callback(self) -> None:
        now = self.get_clock().now()

        if self.latest_target is None or self.last_target_rx_time is None:
            self._publish_zero('waiting_target')
            return

        dt_target = (now - self.last_target_rx_time).nanoseconds * 1e-9
        if dt_target > self.target_timeout_sec:
            self._publish_zero('stale_target')
            return

        target_x_local = float(self.latest_target.pose.position.x)
        target_y_local = float(self.latest_target.pose.position.y)
        heading_error = self._yaw_from_quaternion(self.latest_target)
        if (
            abs(target_x_local) <= self.target_xy_stop_tolerance_m
            and abs(target_y_local) <= self.target_xy_stop_tolerance_m
            and abs(heading_error) <= self.target_yaw_stop_tolerance_rad
        ):
            self._publish_wheel_cmd(cmd_vel_r=0.0, cmd_vel_l=0.0)
            return

        if (
            self.left_motor_speed is None
            or self.right_motor_speed is None
            or self.last_motor_rx_time is None
        ):
            self._publish_zero('waiting_motor_vel')
            return

        dt_motor = (now - self.last_motor_rx_time).nanoseconds * 1e-9
        if dt_motor > self.motor_timeout_sec:
            self._publish_zero('stale_motor_vel')
            return

        obs = np.array(
            [
                [
                    self.latest_target.pose.position.x,
                    self.latest_target.pose.position.y,
                    heading_error,
                    self.left_motor_speed,
                    self.right_motor_speed,
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
        )

    def _publish_wheel_cmd(
        self,
        cmd_vel_r: float,
        cmd_vel_l: float,
    ) -> None:
        msg = self.wheel_cmd_msg_cls()

        left_cmd = self.wheel_cmd_item_msg_cls()
        left_cmd.id = int(self.left_motor_id)
        left_cmd.target = float(cmd_vel_l)

        right_cmd = self.wheel_cmd_item_msg_cls()
        right_cmd.id = int(self.right_motor_id)
        right_cmd.target = float(cmd_vel_r)

        msg.commands = [left_cmd, right_cmd]
        self.wheel_cmd_pub.publish(msg)

    def _warn_throttle(self, key: str, text: str, period_sec: float) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        last = self._warn_timestamps.get(key, -1.0e12)
        if now_sec - last >= period_sec:
            self.get_logger().warn(text)
            self._warn_timestamps[key] = now_sec

    @staticmethod
    def _yaw_from_quaternion(msg: PoseStamped) -> float:
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return float(math.atan2(siny_cosp, cosy_cosp))


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
