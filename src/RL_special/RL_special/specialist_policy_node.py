#!/usr/bin/env python3
import math
import os
import time
from typing import Optional

import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool


class CartAlignSpecialistPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__('RL_special_node')

        self.declare_parameter('model_path', self._default_model_path())
        self.declare_parameter('target_topic', '/rs/cart_pose')
        self.declare_parameter('motor_state_topic', '/rmd_state')
        self.declare_parameter(
            'motor_state_type',
            'cartrider_rmd_sdk/msg/MotorStateArray',
        )
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('gripper_toggle_topic', '/gripper_toggle')
        self.declare_parameter('linear_velocity_scale_m_s', 0.0)
        self.declare_parameter('angular_velocity_scale_rad_s', 0.0)
        self.declare_parameter('spin_in_place_angular_limit_rad_s', 0.0)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('target_timeout_sec', 1000.0)
        self.declare_parameter('motor_timeout_sec', 1000.0)
        self.declare_parameter('target_xy_stop_tolerance_m', 0.05)
        self.declare_parameter('target_yaw_stop_tolerance_deg', 5.0)
        self.declare_parameter('target_x_offset_m', 0.0)
        self.declare_parameter('final_forward_distance_m', 0.35)
        self.declare_parameter('near_target_distance_m', 0.5)
        self.declare_parameter('near_target_linear_speed_limit_m_s', 0.0)
        self.declare_parameter('near_target_angular_speed_limit_rad_s', 0.0)
        self.declare_parameter('wheel_radius_m', 0.0)
        self.declare_parameter('wheel_separation_m', 0.0)
        self.declare_parameter('external_reduction', 1.0)
        self.declare_parameter('state_invert_left', False)
        self.declare_parameter('state_invert_right', False)
        self.declare_parameter('left_motor_id', 1)
        self.declare_parameter('right_motor_id', 2)

        self.model_path = str(self.get_parameter('model_path').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.motor_state_topic = str(self.get_parameter('motor_state_topic').value)
        self.motor_state_type = str(self.get_parameter('motor_state_type').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.gripper_toggle_topic = str(
            self.get_parameter('gripper_toggle_topic').value
        )
        self.linear_velocity_scale_m_s = float(
            self.get_parameter('linear_velocity_scale_m_s').value
        )
        self.angular_velocity_scale_rad_s = float(
            self.get_parameter('angular_velocity_scale_rad_s').value
        )
        self.spin_in_place_angular_limit_rad_s = float(
            self.get_parameter('spin_in_place_angular_limit_rad_s').value
        )
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.motor_timeout_sec = float(self.get_parameter('motor_timeout_sec').value)
        self.target_xy_stop_tolerance_m = float(
            self.get_parameter('target_xy_stop_tolerance_m').value
        )
        self.target_yaw_stop_tolerance_deg = float(
            self.get_parameter('target_yaw_stop_tolerance_deg').value
        )
        self.target_x_offset_m = float(
            self.get_parameter('target_x_offset_m').value
        )
        self.final_forward_distance_m = float(
            self.get_parameter('final_forward_distance_m').value
        )
        self.near_target_distance_m = float(
            self.get_parameter('near_target_distance_m').value
        )
        self.near_target_linear_speed_limit_m_s = float(
            self.get_parameter('near_target_linear_speed_limit_m_s').value
        )
        self.near_target_angular_speed_limit_rad_s = float(
            self.get_parameter('near_target_angular_speed_limit_rad_s').value
        )
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_separation_m = float(
            self.get_parameter('wheel_separation_m').value
        )
        self.external_reduction = float(
            self.get_parameter('external_reduction').value
        )
        self.state_invert_left = bool(self.get_parameter('state_invert_left').value)
        self.state_invert_right = bool(self.get_parameter('state_invert_right').value)
        self.left_motor_id = int(self.get_parameter('left_motor_id').value)
        self.right_motor_id = int(self.get_parameter('right_motor_id').value)

        self._validate_parameters()

        self.target_yaw_stop_tolerance_rad = math.radians(
            self.target_yaw_stop_tolerance_deg
        )

        self.latest_target: Optional[Pose2D] = None
        self.last_target_rx_time = None
        self.current_linear_velocity_m_s: Optional[float] = None
        self.current_angular_velocity_rad_s: Optional[float] = None
        self.last_motor_rx_time = None
        self.control_phase = 'align'
        self.final_forward_distance_traveled_m = 0.0
        self.final_forward_last_update_time = None
        self.shutdown_requested = False
        self._warn_timestamps = {}

        self._load_model()
        self._load_motor_state_type()

        self.target_sub = self.create_subscription(
            Pose2D,
            self.target_topic,
            self._target_callback,
            qos_profile_sensor_data,
        )
        self.motor_state_sub = self.create_subscription(
            self.motor_state_msg_cls,
            self.motor_state_topic,
            self._motor_state_callback,
            10,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.gripper_toggle_pub = self.create_publisher(
            Bool,
            self.gripper_toggle_topic,
            10,
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_callback,
        )

        self.get_logger().info(
            'Specialist policy node started: model_path=%s, target_topic=%s, '
            'motor_state_topic=%s, motor_state_type=%s, cmd_vel_topic=%s, '
            'gripper_toggle_topic=%s, '
            'left_motor_id=%d, right_motor_id=%d, '
            'wheel_radius=%.6fm, wheel_separation=%.6fm, external_reduction=%.6f, '
            'state_invert_left=%s, state_invert_right=%s, '
            'rate=%.2fHz, target_timeout=%.3fs, motor_timeout=%.3fs, '
            'target_xy_stop_tolerance=%.4fm, target_yaw_stop_tolerance=%.2fdeg, '
            'target_x_offset=%.3fm, '
            'final_forward_distance=%.3fm, '
            'near_target_distance=%.3fm, near_target_linear_limit=%.3fm/s, '
            'near_target_angular_limit=%.3frad/s, spin_in_place_angular_limit=%.3frad/s, '
            'linear_velocity_scale=%.6fm/s, angular_velocity_scale=%.6frad/s'
            % (
                self.model_path,
                self.target_topic,
                self.motor_state_topic,
                self.motor_state_type,
                self.cmd_vel_topic,
                self.gripper_toggle_topic,
                self.left_motor_id,
                self.right_motor_id,
                self.wheel_radius_m,
                self.wheel_separation_m,
                self.external_reduction,
                self.state_invert_left,
                self.state_invert_right,
                self.control_rate_hz,
                self.target_timeout_sec,
                self.motor_timeout_sec,
                self.target_xy_stop_tolerance_m,
                self.target_yaw_stop_tolerance_deg,
                self.target_x_offset_m,
                self.final_forward_distance_m,
                self.near_target_distance_m,
                self.near_target_linear_speed_limit_m_s,
                self.near_target_angular_speed_limit_rad_s,
                self.spin_in_place_angular_limit_rad_s,
                self.linear_velocity_scale_m_s,
                self.angular_velocity_scale_rad_s,
            )
        )

    def _default_model_path(self) -> str:
        try:
            share_dir = get_package_share_directory('RL_special')
            candidate = os.path.join(
                share_dir,
                'models',
                'specialist_policy.onnx',
            )
            if os.path.isfile(candidate):
                return candidate
        except Exception:
            pass
        return '/home/kwon/ros2_ws/src/RL_special/models/specialist_policy.onnx'

    def _validate_parameters(self) -> None:
        if self.control_rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be > 0.')
        if self.target_timeout_sec <= 0.0:
            raise ValueError('target_timeout_sec must be > 0.')
        if self.motor_timeout_sec <= 0.0:
            raise ValueError('motor_timeout_sec must be > 0.')
        if self.target_xy_stop_tolerance_m < 0.0:
            raise ValueError('target_xy_stop_tolerance_m must be >= 0.')
        if self.target_yaw_stop_tolerance_deg < 0.0:
            raise ValueError('target_yaw_stop_tolerance_deg must be >= 0.')
        if self.target_x_offset_m < 0.0:
            raise ValueError('target_x_offset_m must be >= 0.')
        if self.final_forward_distance_m < 0.0:
            raise ValueError('final_forward_distance_m must be >= 0.')
        if self.near_target_distance_m < 0.0:
            raise ValueError('near_target_distance_m must be >= 0.')
        if self.near_target_linear_speed_limit_m_s <= 0.0:
            raise ValueError('near_target_linear_speed_limit_m_s must be > 0.')
        if self.near_target_angular_speed_limit_rad_s <= 0.0:
            raise ValueError('near_target_angular_speed_limit_rad_s must be > 0.')
        if self.linear_velocity_scale_m_s <= 0.0:
            raise ValueError('linear_velocity_scale_m_s must be > 0.')
        if self.angular_velocity_scale_rad_s <= 0.0:
            raise ValueError('angular_velocity_scale_rad_s must be > 0.')
        if self.spin_in_place_angular_limit_rad_s < 0.0:
            raise ValueError('spin_in_place_angular_limit_rad_s must be >= 0.')
        if self.wheel_radius_m <= 0.0:
            raise ValueError('wheel_radius_m must be > 0.')
        if self.wheel_separation_m <= 0.0:
            raise ValueError('wheel_separation_m must be > 0.')
        if self.external_reduction <= 0.0:
            raise ValueError('external_reduction must be > 0.')

    def _load_model(self) -> None:
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(f'ONNX model not found: {self.model_path}')

        providers = ort.get_available_providers()
        if 'CPUExecutionProvider' not in providers:
            raise RuntimeError(
                f'CPUExecutionProvider is not available. available={providers}'
            )

        self.session = ort.InferenceSession(
            self.model_path,
            providers=['CPUExecutionProvider'],
        )

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
            f'providers={self.session.get_providers()}'
        )

    def _load_motor_state_type(self) -> None:
        try:
            self.motor_state_msg_cls = get_message(self.motor_state_type)
        except Exception as exc:
            raise RuntimeError(
                f'Failed to load motor_state_type={self.motor_state_type}: {exc}'
            ) from exc

    def _target_callback(self, msg: Pose2D) -> None:
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
            if state.id == self.left_motor_id:
                left_speed = float(state.speed)
            elif state.id == self.right_motor_id:
                right_speed = float(state.speed)

        if left_speed is None or right_speed is None:
            self._warn_throttle(
                'missing_motor_ids',
                'MotorStateArray must include configured left/right motor ids.',
                2.0,
            )
            return

        if self.state_invert_left:
            left_speed *= -1.0
        if self.state_invert_right:
            right_speed *= -1.0

        left_wheel_angular_rad_s = left_speed / self.external_reduction
        right_wheel_angular_rad_s = right_speed / self.external_reduction

        left_linear_m_s = left_wheel_angular_rad_s * self.wheel_radius_m
        right_linear_m_s = right_wheel_angular_rad_s * self.wheel_radius_m

        self.current_linear_velocity_m_s = 0.5 * (left_linear_m_s + right_linear_m_s)
        self.current_angular_velocity_rad_s = (
            right_linear_m_s - left_linear_m_s
        ) / self.wheel_separation_m
        self.last_motor_rx_time = self.get_clock().now()

    def _control_callback(self) -> None:
        now = self.get_clock().now()

        if self.shutdown_requested:
            return

        if self.control_phase == 'final_forward':
            self._run_final_forward(now)
            return

        if self.control_phase == 'done':
            return

        if self.latest_target is None or self.last_target_rx_time is None:
            self._publish_zero('waiting_target')
            return

        dt_target = (now - self.last_target_rx_time).nanoseconds * 1e-9
        if dt_target > self.target_timeout_sec:
            self._publish_zero('stale_target')
            return

        raw_target_x_local = float(self.latest_target.x)
        raw_target_y_local = float(self.latest_target.y)
        heading_error = self._wrap_to_pi(float(self.latest_target.theta))
        target_x_local, target_y_local = self._apply_target_offset(
            raw_target_x_local,
            raw_target_y_local,
            heading_error,
        )
        if (
            abs(target_x_local) <= self.target_xy_stop_tolerance_m
            and abs(target_y_local) <= self.target_xy_stop_tolerance_m
            and abs(heading_error) <= self.target_yaw_stop_tolerance_rad
        ):
            self._start_final_forward(now)
            self._run_final_forward(now)
            return

        if (
            self.current_linear_velocity_m_s is None
            or self.current_angular_velocity_rad_s is None
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
                    target_x_local,
                    target_y_local,
                    heading_error,
                    self.current_linear_velocity_m_s,
                    self.current_angular_velocity_rad_s,
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

        linear_action = float(np.clip(float(actions[0]), -1.0, 1.0))
        angular_action = float(np.clip(float(actions[1]), -1.0, 1.0))

        cmd_linear = linear_action * self.linear_velocity_scale_m_s
        cmd_angular = angular_action * self.angular_velocity_scale_rad_s

        if (
            self.spin_in_place_angular_limit_rad_s > 0.0
            and self._is_spin_in_place(cmd_linear, cmd_angular)
        ):
            limit = self.spin_in_place_angular_limit_rad_s
            cmd_angular = float(np.clip(cmd_angular, -limit, limit))

        target_dist_m = math.hypot(target_x_local, target_y_local)
        if target_dist_m <= self.near_target_distance_m:
            cmd_linear = float(
                np.clip(
                    cmd_linear,
                    -self.near_target_linear_speed_limit_m_s,
                    self.near_target_linear_speed_limit_m_s,
                )
            )
            cmd_angular = float(
                np.clip(
                    cmd_angular,
                    -self.near_target_angular_speed_limit_rad_s,
                    self.near_target_angular_speed_limit_rad_s,
                )
            )

        self._publish_cmd_vel(
            linear_x_m_s=cmd_linear,
            angular_z_rad_s=cmd_angular,
        )

    def _start_final_forward(self, now) -> None:
        if self.control_phase != 'align':
            return
        self.control_phase = 'final_forward'
        self.final_forward_distance_traveled_m = 0.0
        self.final_forward_last_update_time = now
        self.get_logger().info(
            'Alignment complete. Starting final forward motion: distance=%.3fm speed=%.3fm/s'
            % (
                self.final_forward_distance_m,
                self.near_target_linear_speed_limit_m_s,
            )
        )

    def _run_final_forward(self, now) -> None:
        if (
            self.current_linear_velocity_m_s is None
            or self.current_angular_velocity_rad_s is None
            or self.last_motor_rx_time is None
        ):
            self._publish_zero('waiting_motor_vel')
            return

        dt_motor = (now - self.last_motor_rx_time).nanoseconds * 1e-9
        if dt_motor > self.motor_timeout_sec:
            self._publish_zero('stale_motor_vel')
            return

        if self.final_forward_last_update_time is None:
            self.final_forward_last_update_time = now

        dt_forward = (
            now - self.final_forward_last_update_time
        ).nanoseconds * 1e-9
        if dt_forward < 0.0:
            dt_forward = 0.0
        self.final_forward_last_update_time = now

        self.final_forward_distance_traveled_m += (
            max(0.0, self.current_linear_velocity_m_s) * dt_forward
        )

        if self.final_forward_distance_traveled_m >= self.final_forward_distance_m:
            self._publish_cmd_vel(
                linear_x_m_s=0.0,
                angular_z_rad_s=0.0,
            )
            self._publish_gripper_toggle(True)
            self.control_phase = 'done'
            self.shutdown_requested = True
            self.get_logger().info(
                'Final forward motion complete: traveled=%.3fm. Gripper toggled, shutting down node.'
                % self.final_forward_distance_traveled_m
            )
            # Give DDS a brief chance to flush the gripper toggle before shutdown.
            time.sleep(0.1)
            try:
                self.control_timer.cancel()
            except Exception:
                pass
            try:
                if self.context.ok():
                    self.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
            return

        self._publish_cmd_vel(
            linear_x_m_s=self.near_target_linear_speed_limit_m_s,
            angular_z_rad_s=0.0,
        )

    def _is_spin_in_place(self, linear_x_m_s: float, angular_z_rad_s: float) -> bool:
        left_linear_m_s = linear_x_m_s - 0.5 * angular_z_rad_s * self.wheel_separation_m
        right_linear_m_s = linear_x_m_s + 0.5 * angular_z_rad_s * self.wheel_separation_m
        if abs(angular_z_rad_s) <= 1.0e-9:
            return False
        return left_linear_m_s * right_linear_m_s <= 0.0

    def _publish_zero(self, reason_key: str) -> None:
        self._warn_throttle(
            reason_key,
            f'Publishing zero command due to: {reason_key}',
            2.0,
        )
        self._publish_cmd_vel(
            linear_x_m_s=0.0,
            angular_z_rad_s=0.0,
        )

    def _publish_cmd_vel(
        self,
        linear_x_m_s: float,
        angular_z_rad_s: float,
    ) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x_m_s)
        msg.angular.z = float(angular_z_rad_s)
        self.cmd_vel_pub.publish(msg)

    def _publish_gripper_toggle(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.gripper_toggle_pub.publish(msg)

    def _apply_target_offset(
        self,
        raw_target_x_local: float,
        raw_target_y_local: float,
        heading_error_rad: float,
    ) -> tuple[float, float]:
        offset_x_local = self.target_x_offset_m * math.cos(heading_error_rad)
        offset_y_local = self.target_x_offset_m * math.sin(heading_error_rad)
        return (
            raw_target_x_local - offset_x_local,
            raw_target_y_local - offset_y_local,
        )

    def _warn_throttle(self, key: str, text: str, period_sec: float) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        last = self._warn_timestamps.get(key, -1.0e12)
        if now_sec - last >= period_sec:
            self.get_logger().warn(text)
            self._warn_timestamps[key] = now_sec

    @staticmethod
    def _wrap_to_pi(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartAlignSpecialistPolicyNode()
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
