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
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('target_timeout_sec', 0.3)
        self.declare_parameter('motor_timeout_sec', 1000.0)
        self.declare_parameter('target_xy_stop_tolerance_m', 0.05)
        self.declare_parameter('target_yaw_stop_tolerance_deg', 5.0)
        self.declare_parameter('base_link_to_axle_center_x_m', 0.0)
        self.declare_parameter('base_link_to_axle_center_x_sign', -1.0)
        self.declare_parameter('target_x_offset_m', 0.0)
        self.declare_parameter('invert_target_xy_for_policy', False)
        self.declare_parameter('final_forward_distance_m', 0.35)
        self.declare_parameter('final_forward_motion_sign', 1.0)
        self.declare_parameter('calibration_escape_distance_m', 0.30)
        self.declare_parameter('calibration_escape_turn_deg', 30.0)
        self.declare_parameter('calibration_escape_motion_sign', -1.0)
        self.declare_parameter('calibration_target_y_sign', 1.0)
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
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.motor_timeout_sec = float(self.get_parameter('motor_timeout_sec').value)
        self.target_xy_stop_tolerance_m = float(
            self.get_parameter('target_xy_stop_tolerance_m').value
        )
        self.target_yaw_stop_tolerance_deg = float(
            self.get_parameter('target_yaw_stop_tolerance_deg').value
        )
        self.base_link_to_axle_center_x_m = float(
            self.get_parameter('base_link_to_axle_center_x_m').value
        )
        self.base_link_to_axle_center_x_sign = float(
            self.get_parameter('base_link_to_axle_center_x_sign').value
        )
        self.target_x_offset_m = float(
            self.get_parameter('target_x_offset_m').value
        )
        self.invert_target_xy_for_policy = bool(
            self.get_parameter('invert_target_xy_for_policy').value
        )
        self.final_forward_distance_m = float(
            self.get_parameter('final_forward_distance_m').value
        )
        self.final_forward_motion_sign = float(
            self.get_parameter('final_forward_motion_sign').value
        )
        self.calibration_escape_distance_m = float(
            self.get_parameter('calibration_escape_distance_m').value
        )
        self.calibration_escape_turn_deg = float(
            self.get_parameter('calibration_escape_turn_deg').value
        )
        self.calibration_escape_motion_sign = float(
            self.get_parameter('calibration_escape_motion_sign').value
        )
        self.calibration_target_y_sign = float(
            self.get_parameter('calibration_target_y_sign').value
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
        self.calibration_escape_turn_rad = math.radians(
            self.calibration_escape_turn_deg
        )

        self.last_target_rx_time = None
        self.target_x_local_m: Optional[float] = None
        self.target_y_local_m: Optional[float] = None
        self.target_theta_vision_rad: Optional[float] = None
        self.last_target_state_update_time = None
        self.current_linear_velocity_m_s: Optional[float] = None
        self.current_angular_velocity_rad_s: Optional[float] = None
        self.last_motor_rx_time = None
        self.control_phase = 'align'
        self.calibration_stage = ''
        self.calibration_distance_traveled_m = 0.0
        self.calibration_yaw_traveled_rad = 0.0
        self.calibration_last_update_time = None
        self.calibration_turn_direction_sign = 0.0
        self.final_forward_distance_traveled_m = 0.0
        self.final_forward_last_update_time = None
        self.shutdown_requested = False
        self.current_status = ''
        self.pending_target_msg: Optional[Pose2D] = None
        self.pending_target_rx_time = None

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

        self._set_status('starting')

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
        if self.base_link_to_axle_center_x_m < 0.0:
            raise ValueError('base_link_to_axle_center_x_m must be >= 0.')
        if self.base_link_to_axle_center_x_sign not in (-1.0, 1.0):
            raise ValueError('base_link_to_axle_center_x_sign must be -1.0 or 1.0.')
        if self.target_x_offset_m < 0.0:
            raise ValueError('target_x_offset_m must be >= 0.')
        if self.final_forward_distance_m < 0.0:
            raise ValueError('final_forward_distance_m must be >= 0.')
        if self.final_forward_motion_sign not in (-1.0, 1.0):
            raise ValueError('final_forward_motion_sign must be -1.0 or 1.0.')
        if self.calibration_escape_distance_m < 0.0:
            raise ValueError('calibration_escape_distance_m must be >= 0.')
        if self.calibration_escape_turn_deg < 0.0:
            raise ValueError('calibration_escape_turn_deg must be >= 0.')
        if self.calibration_escape_motion_sign not in (-1.0, 1.0):
            raise ValueError('calibration_escape_motion_sign must be -1.0 or 1.0.')
        if self.calibration_target_y_sign not in (-1.0, 1.0):
            raise ValueError('calibration_target_y_sign must be -1.0 or 1.0.')
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

        _ = tuple(inputs[0].shape)
        _ = tuple(outputs[0].shape)

    def _load_motor_state_type(self) -> None:
        try:
            self.motor_state_msg_cls = get_message(self.motor_state_type)
        except Exception as exc:
            raise RuntimeError(
                f'Failed to load motor_state_type={self.motor_state_type}: {exc}'
            ) from exc

    def _target_callback(self, msg: Pose2D) -> None:
        now = self.get_clock().now()
        if self.control_phase == 'calibration':
            self.pending_target_msg = msg
            self.pending_target_rx_time = now
            return
        
        self._apply_target_measurement(msg, now)

    def _apply_target_measurement(self, msg: Pose2D, now) -> None:
        target_theta_vision_rad = self._wrap_to_pi(float(msg.theta))
        target_x_axle_m, target_y_axle_m = self._shift_target_to_axle_center(
            float(msg.x),
            float(msg.y),
        )
        target_x_local_m, target_y_local_m = self._apply_target_offset(
            target_x_axle_m,
            target_y_axle_m,
            target_theta_vision_rad,
        )
        self.target_x_local_m = target_x_local_m
        self.target_y_local_m = target_y_local_m
        self.target_theta_vision_rad = target_theta_vision_rad
        self.last_target_rx_time = now
        self.last_target_state_update_time = now

    def _motor_state_callback(self, msg) -> None:
        if not hasattr(msg, 'states'):
            self._set_status('invalid_motor_state_msg')
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
            self._set_status('missing_motor_ids')
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

        if self.control_phase == 'calibration':
            self._run_calibration(now)
            return

        if self.control_phase == 'final_forward':
            self._run_final_forward(now)
            return

        if self.control_phase == 'done':
            return

        if (
            self.target_x_local_m is None
            or self.target_y_local_m is None
            or self.target_theta_vision_rad is None
            or self.last_target_rx_time is None
            or self.last_target_state_update_time is None
        ):
            self._publish_zero('waiting_target')
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

        dt_target = (now - self.last_target_rx_time).nanoseconds * 1e-9
        if dt_target > self.target_timeout_sec:
            self._start_calibration(now)
            self._run_calibration(now)
            return

        self._update_target_state_from_odometry(now)

        heading_error = self._wrap_to_pi(-self.target_theta_vision_rad)
        target_x_local = self.target_x_local_m
        target_y_local = self.target_y_local_m
        policy_target_x_local = target_x_local
        policy_target_y_local = target_y_local
        if self.invert_target_xy_for_policy:
            policy_target_x_local *= -1.0
            policy_target_y_local *= -1.0
        if (
            abs(target_x_local) <= self.target_xy_stop_tolerance_m
            and abs(target_y_local) <= self.target_xy_stop_tolerance_m
            and abs(heading_error) <= self.target_yaw_stop_tolerance_rad
        ):
            self._start_final_forward(now)
            self._run_final_forward(now)
            return

        self._set_status('align')

        obs = np.array(
            [
                [
                    policy_target_x_local,
                    policy_target_y_local,
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
            self._set_status('inference_failed')
            self._publish_zero('inference_failed')
            return

        actions = np.asarray(inference_out, dtype=np.float32).reshape(-1)
        if actions.size < 2:
            self._set_status('invalid_output')
            self._publish_zero('invalid_output')
            return

        linear_action = float(np.clip(float(actions[0]), -1.0, 1.0))
        angular_action = float(np.clip(float(actions[1]), -1.0, 1.0))

        cmd_linear = linear_action * self.linear_velocity_scale_m_s
        cmd_angular = angular_action * self.angular_velocity_scale_rad_s

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
        self._set_status('final_forward')

    def _start_calibration(self, now) -> None:
        if self.control_phase == 'calibration':
            return
        self.control_phase = 'calibration'
        target_y_local = (
            self.target_y_local_m if self.target_y_local_m is not None else 0.0
        )
        target_y_local *= self.calibration_target_y_sign
        if target_y_local < 0.0:
            self.calibration_turn_direction_sign = -self.calibration_escape_motion_sign
        elif target_y_local > 0.0:
            self.calibration_turn_direction_sign = self.calibration_escape_motion_sign
        else:
            self.calibration_turn_direction_sign = 0.0
        if abs(self.calibration_turn_direction_sign) > 0.0 and self.calibration_escape_turn_rad > 0.0:
            self.calibration_stage = 'rotate_out'
        else:
            self.calibration_stage = 'reverse_escape'
        self.calibration_distance_traveled_m = 0.0
        self.calibration_yaw_traveled_rad = 0.0
        self.calibration_last_update_time = now
        self.pending_target_msg = None
        self.pending_target_rx_time = None
        self._publish_cmd_vel(
            linear_x_m_s=0.0,
            angular_z_rad_s=0.0,
        )
        self._set_status(f'calibration_{self.calibration_stage}')

    def _run_calibration(self, now) -> None:
        if (
            self.current_linear_velocity_m_s is None
            or self.current_angular_velocity_rad_s is None
            or self.last_motor_rx_time is None
            or self.target_x_local_m is None
            or self.target_y_local_m is None
            or self.target_theta_vision_rad is None
        ):
            self._publish_zero('waiting_motor_vel')
            return

        dt_motor = (now - self.last_motor_rx_time).nanoseconds * 1e-9
        if dt_motor > self.motor_timeout_sec:
            self._publish_zero('stale_motor_vel')
            return

        if self.calibration_last_update_time is None:
            self.calibration_last_update_time = now

        dt = (now - self.calibration_last_update_time).nanoseconds * 1e-9
        if dt < 0.0:
            dt = 0.0
        self.calibration_last_update_time = now
        angular_limit = max(self.near_target_angular_speed_limit_rad_s, 0.3)
        linear_limit = self.near_target_linear_speed_limit_m_s

        if self.calibration_stage == 'rotate_out':
            self.calibration_yaw_traveled_rad += abs(self.current_angular_velocity_rad_s) * dt
            if self.calibration_yaw_traveled_rad >= self.calibration_escape_turn_rad:
                self.calibration_stage = 'reverse_escape'
                self.calibration_distance_traveled_m = 0.0
                self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
                self._set_status('calibration_reverse_escape')
                return
            self._publish_cmd_vel(
                linear_x_m_s=0.0,
                angular_z_rad_s=self.calibration_turn_direction_sign * angular_limit,
            )
            return

        if self.calibration_stage == 'reverse_escape':
            if self.calibration_escape_motion_sign > 0.0:
                self.calibration_distance_traveled_m += max(
                    0.0, self.current_linear_velocity_m_s
                ) * dt
            else:
                self.calibration_distance_traveled_m += max(
                    0.0, -self.current_linear_velocity_m_s
                ) * dt
            if self.calibration_distance_traveled_m >= self.calibration_escape_distance_m:
                if abs(self.calibration_turn_direction_sign) <= 0.0 or self.calibration_escape_turn_rad <= 0.0:
                    self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
                    self._finish_calibration()
                    return
                self.calibration_stage = 'rotate_back'
                self.calibration_yaw_traveled_rad = 0.0
                self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
                self._set_status('calibration_rotate_back')
                return
            self._publish_cmd_vel(
                linear_x_m_s=self.calibration_escape_motion_sign * linear_limit,
                angular_z_rad_s=0.0,
            )
            return

        if self.calibration_stage == 'rotate_back':
            self.calibration_yaw_traveled_rad += abs(self.current_angular_velocity_rad_s) * dt
            if self.calibration_yaw_traveled_rad >= self.calibration_escape_turn_rad:
                self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
                self._finish_calibration()
                return
            self._publish_cmd_vel(
                linear_x_m_s=0.0,
                angular_z_rad_s=-self.calibration_turn_direction_sign * angular_limit,
            )
            return

        self._publish_zero('invalid_calibration_state')

    def _finish_calibration(self) -> None:
        self.control_phase = 'align'
        self.calibration_stage = ''
        self.calibration_distance_traveled_m = 0.0
        self.calibration_yaw_traveled_rad = 0.0
        self.calibration_last_update_time = None
        self.calibration_turn_direction_sign = 0.0
        self._set_status('calibration_done')
        if self.pending_target_msg is not None and self.pending_target_rx_time is not None:
            self._apply_target_measurement(
                self.pending_target_msg,
                self.pending_target_rx_time,
            )
            self.pending_target_msg = None
            self.pending_target_rx_time = None
        else:
            self.last_target_rx_time = None
            self.last_target_state_update_time = None

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

        if self.final_forward_motion_sign > 0.0:
            self.final_forward_distance_traveled_m += (
                max(0.0, self.current_linear_velocity_m_s) * dt_forward
            )
        else:
            self.final_forward_distance_traveled_m += (
                max(0.0, -self.current_linear_velocity_m_s) * dt_forward
            )

        if self.final_forward_distance_traveled_m >= self.final_forward_distance_m:
            self._publish_cmd_vel(
                linear_x_m_s=0.0,
                angular_z_rad_s=0.0,
            )
            self._publish_gripper_toggle(True)
            self.control_phase = 'done'
            self.shutdown_requested = True
            self._set_status('done')
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
            linear_x_m_s=(
                self.final_forward_motion_sign
                * self.near_target_linear_speed_limit_m_s
            ),
            angular_z_rad_s=0.0,
        )

    def _publish_zero(self, reason_key: str) -> None:
        self._set_status(reason_key)
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
        axle_target_x_local: float,
        axle_target_y_local: float,
        target_theta_vision_rad: float,
    ) -> tuple[float, float]:
        offset_x_local = self.target_x_offset_m * math.cos(target_theta_vision_rad)
        offset_y_local = self.target_x_offset_m * math.sin(target_theta_vision_rad)
        return (
            axle_target_x_local - offset_x_local,
            axle_target_y_local - offset_y_local,
        )

    def _update_target_state_from_odometry(self, now) -> None:
        if (
            self.target_x_local_m is None
            or self.target_y_local_m is None
            or self.target_theta_vision_rad is None
            or self.last_target_state_update_time is None
            or self.current_linear_velocity_m_s is None
            or self.current_angular_velocity_rad_s is None
        ):
            return

        dt = (now - self.last_target_state_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            self.last_target_state_update_time = now
            return

        delta_yaw = self.current_angular_velocity_rad_s * dt
        if abs(self.current_angular_velocity_rad_s) <= 1.0e-9:
            delta_x = self.current_linear_velocity_m_s * dt
            delta_y = 0.0
        else:
            turn_radius_m = (
                self.current_linear_velocity_m_s / self.current_angular_velocity_rad_s
            )
            delta_x = turn_radius_m * math.sin(delta_yaw)
            delta_y = turn_radius_m * (1.0 - math.cos(delta_yaw))

        cos_yaw = math.cos(delta_yaw)
        sin_yaw = math.sin(delta_yaw)
        shifted_x = self.target_x_local_m - delta_x
        shifted_y = self.target_y_local_m - delta_y

        self.target_x_local_m = cos_yaw * shifted_x + sin_yaw * shifted_y
        self.target_y_local_m = -sin_yaw * shifted_x + cos_yaw * shifted_y
        self.target_theta_vision_rad = self._wrap_to_pi(
            self.target_theta_vision_rad + delta_yaw
        )
        self.last_target_state_update_time = now

    def _shift_target_to_axle_center(
        self,
        raw_target_x_local: float,
        raw_target_y_local: float,
    ) -> tuple[float, float]:
        return (
            raw_target_x_local
            + self.base_link_to_axle_center_x_sign
            * self.base_link_to_axle_center_x_m,
            raw_target_y_local,
        )

    def _set_status(self, status: str) -> None:
        if self.current_status == status:
            return
        self.current_status = status
        self.get_logger().info(f'mode={status}')

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
