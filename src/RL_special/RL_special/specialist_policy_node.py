#!/usr/bin/env python3
import math
import os
import re
from typing import Optional

import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool, Int32


class CartAlignSpecialistPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__('RL_special_node')

        self.declare_parameter('robot_type', 'front')
        self.declare_parameter('model_path', self._default_model_path())
        self.declare_parameter('docking_target_topic', '/docking_target')
        self.declare_parameter('target_topic', '/target_pose')
        self.declare_parameter('docking_state_topic', '/docking_state')
        self.declare_parameter('motor_state_topic', '/rmd_state')
        self.declare_parameter(
            'motor_state_type',
            'cartrider_rmd_sdk/msg/MotorStateArray',
        )
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter(
            'robot_docking_completion_topic',
            '/gripper_toggle',
        )
        self.declare_parameter(
            'cart_docking_completion_topic',
            '/gripper_toggle',
        )
        self.declare_parameter('rl_docking_done_topic', '/rl_docking_done')
        self.declare_parameter('linear_velocity_scale_m_s', 0.0)
        self.declare_parameter('angular_velocity_scale_rad_s', 0.0)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('target_timeout_sec', 0.3)
        self.declare_parameter('scan_no_target_timeout_sec', 3.0)
        self.declare_parameter('scan_settle_sec', 1.0)
        self.declare_parameter('scan_half_sweep_deg', 45.0)
        self.declare_parameter('motor_timeout_sec', 1000.0)
        self.declare_parameter('calibration_resume_delay_sec', 0.5)
        self.declare_parameter('target_xy_stop_tolerance_m', 0.05)
        self.declare_parameter('target_yaw_stop_tolerance_deg', 5.0)
        self.declare_parameter('robot_docking_target_xy_stop_tolerance_m', 0.05)
        self.declare_parameter('robot_docking_target_yaw_stop_tolerance_deg', 5.0)
        self.declare_parameter('base_link_to_axle_center_x_m', 0.0)
        self.declare_parameter('target_x_offset_m', 0.0)
        self.declare_parameter('rear_target_y_offset_m', 0.0)
        self.declare_parameter('front_calibration_safe_axis_x_m', 0.0)
        self.declare_parameter('rear_calibration_safe_axis_x_m', 0.0)
        self.declare_parameter('cart_docking_final_distance_m', 0.35)
        self.declare_parameter('robot_docking_final_distance_m', 0.35)
        self.declare_parameter('final_docking_motion_sign', 1.0)
        self.declare_parameter('near_target_distance_m', 0.5)
        self.declare_parameter('near_target_linear_speed_limit_m_s', 0.0)
        self.declare_parameter('robot_docking_final_linear_speed_m_s', 0.0)
        self.declare_parameter('robot_docking_calibration_target_x_threshold_m', -0.10)
        self.declare_parameter('rear_calibration_target_x_threshold_m', 0.10)
        self.declare_parameter('near_target_angular_speed_limit_rad_s', 0.0)
        self.declare_parameter('wheel_radius_m', 0.0)
        self.declare_parameter('wheel_separation_m', 0.0)
        self.declare_parameter('external_reduction', 1.0)
        self.declare_parameter('linear_odometry_scale', 1.0)
        self.declare_parameter('angular_odometry_scale', 1.0)
        self.declare_parameter('state_invert_left', False)
        self.declare_parameter('state_invert_right', False)
        self.declare_parameter('left_motor_id', 1)
        self.declare_parameter('right_motor_id', 2)

        self.robot_type = str(self.get_parameter('robot_type').value).strip().lower()
        self.model_path = str(self.get_parameter('model_path').value)
        self.docking_target_topic = str(
            self.get_parameter('docking_target_topic').value
        )
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.docking_state_topic = str(
            self.get_parameter('docking_state_topic').value
        )
        self.motor_state_topic = str(self.get_parameter('motor_state_topic').value)
        self.motor_state_type = str(self.get_parameter('motor_state_type').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.robot_docking_completion_topic = str(
            self.get_parameter('robot_docking_completion_topic').value
        )
        self.cart_docking_completion_topic = str(
            self.get_parameter('cart_docking_completion_topic').value
        )
        self.rl_docking_done_topic = str(
            self.get_parameter('rl_docking_done_topic').value
        )
        self.linear_velocity_scale_m_s = float(
            self.get_parameter('linear_velocity_scale_m_s').value
        )
        self.angular_velocity_scale_rad_s = float(
            self.get_parameter('angular_velocity_scale_rad_s').value
        )
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.target_timeout_sec = float(self.get_parameter('target_timeout_sec').value)
        self.scan_no_target_timeout_sec = float(
            self.get_parameter('scan_no_target_timeout_sec').value
        )
        self.scan_settle_sec = float(self.get_parameter('scan_settle_sec').value)
        self.scan_half_sweep_deg = float(
            self.get_parameter('scan_half_sweep_deg').value
        )
        self.motor_timeout_sec = float(self.get_parameter('motor_timeout_sec').value)
        self.calibration_resume_delay_sec = float(
            self.get_parameter('calibration_resume_delay_sec').value
        )
        self.target_xy_stop_tolerance_m = float(
            self.get_parameter('target_xy_stop_tolerance_m').value
        )
        self.target_yaw_stop_tolerance_deg = float(
            self.get_parameter('target_yaw_stop_tolerance_deg').value
        )
        self.robot_docking_target_xy_stop_tolerance_m = float(
            self.get_parameter('robot_docking_target_xy_stop_tolerance_m').value
        )
        self.robot_docking_target_yaw_stop_tolerance_deg = float(
            self.get_parameter(
                'robot_docking_target_yaw_stop_tolerance_deg'
            ).value
        )
        self.base_link_to_axle_center_x_m = float(
            self.get_parameter('base_link_to_axle_center_x_m').value
        )
        self.target_x_offset_m = float(
            self.get_parameter('target_x_offset_m').value
        )
        self.rear_target_y_offset_m = float(
            self.get_parameter('rear_target_y_offset_m').value
        )
        self.front_calibration_safe_axis_x_m = float(
            self.get_parameter('front_calibration_safe_axis_x_m').value
        )
        self.rear_calibration_safe_axis_x_m = float(
            self.get_parameter('rear_calibration_safe_axis_x_m').value
        )
        self.cart_docking_final_distance_m = float(
            self.get_parameter('cart_docking_final_distance_m').value
        )
        self.robot_docking_final_distance_m = float(
            self.get_parameter('robot_docking_final_distance_m').value
        )
        self.final_docking_motion_sign = float(
            self.get_parameter('final_docking_motion_sign').value
        )
        self.near_target_distance_m = float(
            self.get_parameter('near_target_distance_m').value
        )
        self.near_target_linear_speed_limit_m_s = float(
            self.get_parameter('near_target_linear_speed_limit_m_s').value
        )
        self.robot_docking_final_linear_speed_m_s = float(
            self.get_parameter('robot_docking_final_linear_speed_m_s').value
        )
        self.robot_docking_calibration_target_x_threshold_m = float(
            self.get_parameter(
                'robot_docking_calibration_target_x_threshold_m'
            ).value
        )
        self.rear_calibration_target_x_threshold_m = float(
            self.get_parameter('rear_calibration_target_x_threshold_m').value
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
        self.linear_odometry_scale = float(
            self.get_parameter('linear_odometry_scale').value
        )
        self.angular_odometry_scale = float(
            self.get_parameter('angular_odometry_scale').value
        )
        self.state_invert_left = bool(self.get_parameter('state_invert_left').value)
        self.state_invert_right = bool(self.get_parameter('state_invert_right').value)
        self.left_motor_id = int(self.get_parameter('left_motor_id').value)
        self.right_motor_id = int(self.get_parameter('right_motor_id').value)
        self.robot_docking_enabled = self.robot_type == 'front'

        if not self.robot_docking_enabled:
            self.robot_docking_completion_topic = ''

        self._validate_parameters()

        self.target_yaw_stop_tolerance_rad = math.radians(
            self.target_yaw_stop_tolerance_deg
        )
        self.robot_docking_target_yaw_stop_tolerance_rad = (
            math.radians(self.robot_docking_target_yaw_stop_tolerance_deg)
            if self.robot_docking_enabled
            else 0.0
        )

        self.last_target_rx_time = None
        self.target_x_local_m: Optional[float] = None
        self.target_y_local_m: Optional[float] = None
        self.target_yaw_error_rad: Optional[float] = None
        self.last_target_state_update_time = None
        self.active_docking_target = 0
        self.current_linear_velocity_m_s: Optional[float] = None
        self.current_angular_velocity_rad_s: Optional[float] = None
        self.last_motor_rx_time = None
        self.control_phase = 'waiting_docking_target'
        self.docking_activation_time = None
        self.target_seen_since_activation = False
        self.calibration_stage = ''
        self.calibration_distance_traveled_m = 0.0
        self.calibration_move_distance_target_m = 0.0
        self.calibration_move_motion_sign = 1.0
        self.calibration_yaw_traveled_rad = 0.0
        self.calibration_last_update_time = None
        self.calibration_rotate_out_target_rad = 0.0
        self.calibration_rotate_back_target_rad = 0.0
        self.final_docking_distance_traveled_m = 0.0
        self.final_docking_distance_target_m = 0.0
        self.final_docking_last_update_time = None
        self.calibration_resume_time = None
        self.target_loss_wait_start_time = None
        self.scan_settle_end_time = None
        self.scan_last_update_time = None
        self.scan_angle_from_center_rad = 0.0
        self.scan_direction_sign = 1.0
        self.current_status = ''
        self.pending_target_msg: Optional[PointStamped] = None
        self.pending_target_rx_time = None
        self.suppress_robot_docking_target_x_calibration = False
        self.rl_docking_done_active = False

        self._load_model()
        self._load_motor_state_type()

        self.docking_target_sub = self.create_subscription(
            Int32,
            self.docking_target_topic,
            self._docking_target_callback,
            10,
        )
        self.target_sub = self.create_subscription(
            PointStamped,
            self.target_topic,
            self._target_callback,
            qos_profile_sensor_data,
        )
        self.docking_state_sub = self.create_subscription(
            Bool,
            self.docking_state_topic,
            self._docking_state_callback,
            10,
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
        self.robot_docking_completion_pub = None
        if self.robot_docking_enabled and self.robot_docking_completion_topic:
            self.robot_docking_completion_pub = self.create_publisher(
                Bool,
                self.robot_docking_completion_topic,
                10,
            )
        self.cart_docking_completion_pub = self.create_publisher(
            Bool,
            self.cart_docking_completion_topic,
            10,
        )
        self.rl_docking_done_pub = self.create_publisher(
            Bool,
            self.rl_docking_done_topic,
            10,
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_callback,
        )

        self._set_status('waiting_docking_target')

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
        if self.robot_type not in ('front', 'rear'):
            raise ValueError('robot_type must be "front" or "rear".')
        if self.control_rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be > 0.')
        if self.target_timeout_sec <= 0.0:
            raise ValueError('target_timeout_sec must be > 0.')
        if self.scan_no_target_timeout_sec <= 0.0:
            raise ValueError('scan_no_target_timeout_sec must be > 0.')
        if self.scan_settle_sec < 0.0:
            raise ValueError('scan_settle_sec must be >= 0.')
        if self.scan_half_sweep_deg <= 0.0 or self.scan_half_sweep_deg > 180.0:
            raise ValueError('scan_half_sweep_deg must be in (0, 180].')
        if self.motor_timeout_sec <= 0.0:
            raise ValueError('motor_timeout_sec must be > 0.')
        if self.calibration_resume_delay_sec < 0.0:
            raise ValueError('calibration_resume_delay_sec must be >= 0.')
        if self.target_xy_stop_tolerance_m < 0.0:
            raise ValueError('target_xy_stop_tolerance_m must be >= 0.')
        if self.target_yaw_stop_tolerance_deg < 0.0:
            raise ValueError('target_yaw_stop_tolerance_deg must be >= 0.')
        if self.base_link_to_axle_center_x_m < 0.0:
            raise ValueError('base_link_to_axle_center_x_m must be >= 0.')
        if self.target_x_offset_m < 0.0:
            raise ValueError('target_x_offset_m must be >= 0.')
        if self.front_calibration_safe_axis_x_m < 0.0:
            raise ValueError('front_calibration_safe_axis_x_m must be >= 0.')
        if self.rear_calibration_safe_axis_x_m < 0.0:
            raise ValueError('rear_calibration_safe_axis_x_m must be >= 0.')
        if self.cart_docking_final_distance_m < 0.0:
            raise ValueError('cart_docking_final_distance_m must be >= 0.')
        if self.final_docking_motion_sign not in (-1.0, 1.0):
            raise ValueError('final_docking_motion_sign must be -1.0 or 1.0.')
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
        if self.linear_odometry_scale <= 0.0:
            raise ValueError('linear_odometry_scale must be > 0.')
        if self.angular_odometry_scale <= 0.0:
            raise ValueError('angular_odometry_scale must be > 0.')
        if self.robot_docking_enabled:
            if self.robot_docking_target_xy_stop_tolerance_m < 0.0:
                raise ValueError(
                    'robot_docking_target_xy_stop_tolerance_m must be >= 0.'
                )
            if self.robot_docking_target_yaw_stop_tolerance_deg < 0.0:
                raise ValueError(
                    'robot_docking_target_yaw_stop_tolerance_deg must be >= 0.'
                )
            if self.robot_docking_final_distance_m < 0.0:
                raise ValueError('robot_docking_final_distance_m must be >= 0.')
            if self.robot_docking_final_linear_speed_m_s <= 0.0:
                raise ValueError('robot_docking_final_linear_speed_m_s must be > 0.')
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

    def _docking_target_callback(self, msg: Int32) -> None:
        now = self.get_clock().now()
        docking_target = int(msg.data)

        if docking_target not in (0, 1, 2):
            self._enter_waiting_mode('invalid_docking_target')
            return

        if docking_target == 0:
            self._enter_waiting_mode('waiting_docking_target')
            return

        if docking_target == 1 and self.robot_type == 'rear':
            self._publish_completion_signal(
                docking_target=2,
                enabled=True,
            )
            self._publish_rl_docking_done(enabled=True)
            self._enter_waiting_mode('rear_robot_docking_idle')
            return

        if (
            self.active_docking_target == docking_target
            and self.control_phase in ('align', 'calibration', 'final_docking_motion')
        ):
            return

        clear_target_state = (
            self.active_docking_target in (1, 2)
            and self.active_docking_target != docking_target
        )
        self._activate_docking_target(
            docking_target,
            now,
            clear_target_state=clear_target_state,
        )

    def _activate_docking_target(
        self,
        docking_target: int,
        now,
        clear_target_state: bool,
    ) -> None:
        self.active_docking_target = docking_target
        self._reset_motion_state()
        self.docking_activation_time = now
        if (
            clear_target_state
            or self._is_target_cache_stale(now)
            or self._should_clear_target_state_for_new_marker_context(docking_target)
        ):
            self._clear_target_state()
        self.target_seen_since_activation = (
            self.last_target_rx_time is not None
            and self.target_x_local_m is not None
            and self.target_y_local_m is not None
            and self.target_yaw_error_rad is not None
            and not self._is_target_cache_stale(now)
        )
        self.control_phase = 'align'
        self._publish_cmd_vel(
            linear_x_m_s=0.0,
            angular_z_rad_s=0.0,
        )
        self._set_status('align')

    def _enter_waiting_mode(
        self,
        status: str = 'waiting_docking_target',
    ) -> None:
        self.active_docking_target = 0
        self._reset_motion_state()
        self.control_phase = 'waiting_docking_target'
        self._publish_cmd_vel(
            linear_x_m_s=0.0,
            angular_z_rad_s=0.0,
        )
        self._set_status(status)

    def _reset_motion_state(self) -> None:
        self.calibration_stage = ''
        self.docking_activation_time = None
        self.target_seen_since_activation = False
        self.calibration_distance_traveled_m = 0.0
        self.calibration_move_distance_target_m = 0.0
        self.calibration_move_motion_sign = 1.0
        self.calibration_yaw_traveled_rad = 0.0
        self.calibration_last_update_time = None
        self.calibration_rotate_out_target_rad = 0.0
        self.calibration_rotate_back_target_rad = 0.0
        self.final_docking_distance_traveled_m = 0.0
        self.final_docking_distance_target_m = 0.0
        self.final_docking_last_update_time = None
        self.calibration_resume_time = None
        self.target_loss_wait_start_time = None
        self.scan_settle_end_time = None
        self.scan_last_update_time = None
        self.scan_angle_from_center_rad = 0.0
        self.scan_direction_sign = 1.0
        self.pending_target_msg = None
        self.pending_target_rx_time = None
        self.suppress_robot_docking_target_x_calibration = False

    def _clear_target_state(self) -> None:
        self.last_target_rx_time = None
        self.target_x_local_m = None
        self.target_y_local_m = None
        self.target_yaw_error_rad = None
        self.last_target_state_update_time = None

    def _is_target_cache_stale(self, now) -> bool:
        if self.last_target_rx_time is None:
            return True
        dt_target = (now - self.last_target_rx_time).nanoseconds * 1e-9
        return dt_target > self.target_timeout_sec

    def _should_clear_target_state_for_new_marker_context(
        self,
        docking_target: int,
    ) -> bool:
        return self.robot_type == 'front' and docking_target == 1

    def _docking_state_callback(self, msg: Bool) -> None:
        if bool(msg.data):
            self.rl_docking_done_active = False

    def _target_callback(self, msg: PointStamped) -> None:
        now = self.get_clock().now()
        marker_id = self._extract_marker_id(msg)
        if marker_id != self._get_expected_target_marker_id():
            self._handle_invalid_target_marker_id(now)
            return
        self.target_seen_since_activation = True
        if self.control_phase == 'calibration':
            self.pending_target_msg = msg
            self.pending_target_rx_time = now
            return

        self._apply_canonical_target_measurement(msg, now)
        if self.control_phase == 'scan':
            self._start_scan_settle(now)
            return
        if self.control_phase == 'scan_settle':
            return
        if self.control_phase == 'target_loss_pause':
            self.target_loss_wait_start_time = None
            self.control_phase = 'align'
            self._set_status('align')

    def _apply_canonical_target_measurement(self, msg: PointStamped, now) -> None:
        target_x_local_m, target_y_local_m, target_yaw_error_rad = (
            self._canonical_target_from_pose(
                self.robot_type,
                float(msg.point.x),
                float(msg.point.y),
                float(msg.point.z),
                self.base_link_to_axle_center_x_m,
                self.target_x_offset_m,
                self.rear_target_y_offset_m,
            )
        )
        should_calibrate_for_target_x = (
            self._should_start_target_x_calibration(
                target_x_local_m,
                target_y_local_m,
                target_yaw_error_rad,
            )
        )
        self.target_x_local_m = target_x_local_m
        self.target_y_local_m = target_y_local_m
        self.target_yaw_error_rad = target_yaw_error_rad
        self.last_target_rx_time = now
        self.last_target_state_update_time = now
        if should_calibrate_for_target_x:
            self._start_calibration(now)

    def _handle_invalid_target_marker_id(self, now) -> None:
        if self.control_phase == 'calibration':
            return
        if self.control_phase == 'post_calibration_pause':
            self.calibration_resume_time = now
            self._set_status('post_calibration_pause')
            return
        if (
            self.active_docking_target in (1, 2)
            and self.control_phase == 'align'
            and self.target_x_local_m is not None
            and self.target_y_local_m is not None
            and self.target_yaw_error_rad is not None
        ):
            self._start_calibration(now)

    @staticmethod
    def _extract_marker_id(msg: PointStamped) -> Optional[int]:
        frame_id = msg.header.frame_id.strip()
        if not frame_id:
            return None
        match = re.search(r'-?\d+', frame_id)
        if match is None:
            return None
        try:
            return int(match.group(0))
        except ValueError:
            return None

    def _get_expected_target_marker_id(self) -> int:
        if self.robot_type == 'rear':
            return 1
        if self.active_docking_target == 1:
            return 4
        return 0

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

        self.current_linear_velocity_m_s = (
            0.5
            * (left_linear_m_s + right_linear_m_s)
            * self.linear_odometry_scale
        )
        self.current_angular_velocity_rad_s = (
            (right_linear_m_s - left_linear_m_s)
            / self.wheel_separation_m
            * self.angular_odometry_scale
        )
        self.last_motor_rx_time = self.get_clock().now()

    def _control_callback(self) -> None:
        now = self.get_clock().now()
        if self.rl_docking_done_active:
            self._publish_rl_docking_done(enabled=True)

        if self.control_phase == 'waiting_docking_target':
            return

        if self.control_phase == 'calibration':
            self._run_calibration(now)
            return

        if self.control_phase == 'post_calibration_pause':
            self._run_post_calibration_pause(now)
            return

        if self.control_phase == 'scan':
            self._run_scan(now)
            return

        if self.control_phase == 'scan_settle':
            self._run_scan_settle(now)
            return

        if self.control_phase == 'target_loss_pause':
            self._run_target_loss_pause(now)
            return

        if self.control_phase == 'final_docking_motion':
            self._run_final_docking_motion(now)
            return

        if self.active_docking_target not in (1, 2):
            self._enter_waiting_mode('waiting_docking_target')
            return

        if (
            self.target_x_local_m is None
            or self.target_y_local_m is None
            or self.target_yaw_error_rad is None
            or self.last_target_rx_time is None
            or self.last_target_state_update_time is None
        ):
            if (
                not self.target_seen_since_activation
                and self.docking_activation_time is not None
            ):
                dt_since_activation = (
                    now - self.docking_activation_time
                ).nanoseconds * 1e-9
                if dt_since_activation > self.scan_no_target_timeout_sec:
                    self._start_scan(now)
                    self._run_scan(now)
                    return
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

        self._update_target_state_from_odometry(now)

        dt_target = (now - self.last_target_rx_time).nanoseconds * 1e-9
        if dt_target > self.target_timeout_sec:
            self.suppress_robot_docking_target_x_calibration = False
            self._start_target_loss_pause(now)
            return

        if self._should_start_target_x_calibration(
            self.target_x_local_m,
            self.target_y_local_m,
            self.target_yaw_error_rad,
        ):
            self._start_calibration(now)
            self._run_calibration(now)
            return

        target_x_local = self.target_x_local_m
        target_y_local = self.target_y_local_m
        target_yaw_error = self.target_yaw_error_rad
        if self._target_is_aligned(
            target_x_local,
            target_y_local,
            target_yaw_error,
        ):
            self._start_final_docking_motion(now)
            self._run_final_docking_motion(now)
            return

        self._set_status('align')

        obs = np.array(
            [
                [
                    target_x_local,
                    target_y_local,
                    target_yaw_error,
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

    def _start_final_docking_motion(self, now) -> None:
        if self.control_phase != 'align':
            return
        self.control_phase = 'final_docking_motion'
        self.final_docking_distance_traveled_m = 0.0
        self.final_docking_last_update_time = now
        self.final_docking_distance_target_m = self._get_active_final_distance_m()
        self._set_status('final_docking_motion')

    def _start_calibration(self, now) -> None:
        if self.control_phase == 'calibration':
            return
        self.control_phase = 'calibration'
        target_x_local = (
            self.target_x_local_m if self.target_x_local_m is not None else 0.0
        )
        target_y_local = (
            self.target_y_local_m if self.target_y_local_m is not None else 0.0
        )
        (
            self.calibration_rotate_out_target_rad,
            self.calibration_move_distance_target_m,
            self.calibration_move_motion_sign,
        ) = self._compute_calibration_rotate_out_and_move_distance(
            target_x_local,
            target_y_local,
            self.target_yaw_error_rad if self.target_yaw_error_rad is not None else 0.0,
            self._get_calibration_axis_sign(),
            self._get_calibration_safe_axis_x_m(),
        )
        self.calibration_rotate_back_target_rad = 0.0
        self.calibration_distance_traveled_m = 0.0
        self.calibration_yaw_traveled_rad = 0.0
        self.calibration_last_update_time = now
        self.pending_target_msg = None
        self.pending_target_rx_time = None
        self._publish_cmd_vel(
            linear_x_m_s=0.0,
            angular_z_rad_s=0.0,
        )
        if abs(self.calibration_rotate_out_target_rad) > 1.0e-9:
            self.calibration_stage = 'rotate_out'
        elif self.calibration_move_distance_target_m > 1.0e-9:
            self.calibration_stage = 'move_to_axis'
        else:
            if self._start_calibration_rotate_back_or_finish():
                return
            self.calibration_stage = 'rotate_back'
        self._set_status(f'calibration_{self.calibration_stage}')

    def _run_calibration(self, now) -> None:
        if (
            self.current_linear_velocity_m_s is None
            or self.current_angular_velocity_rad_s is None
            or self.last_motor_rx_time is None
            or self.target_x_local_m is None
            or self.target_y_local_m is None
            or self.target_yaw_error_rad is None
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

        self._update_target_state_from_odometry(now)

        angular_limit = self._get_calibration_angular_speed_limit_rad_s()
        linear_limit = self.near_target_linear_speed_limit_m_s

        if self.calibration_stage == 'rotate_out':
            self.calibration_yaw_traveled_rad += (
                self._integrate_directed_yaw_progress(
                    self.calibration_rotate_out_target_rad,
                    self.current_angular_velocity_rad_s,
                    dt,
                )
            )
            if self.calibration_yaw_traveled_rad >= abs(
                self.calibration_rotate_out_target_rad
            ):
                self.calibration_stage = 'move_to_axis'
                self.calibration_distance_traveled_m = 0.0
                self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
                self._set_status('calibration_move_to_axis')
                return
            self._publish_cmd_vel(
                linear_x_m_s=0.0,
                angular_z_rad_s=(
                    math.copysign(
                        angular_limit,
                        self.calibration_rotate_out_target_rad,
                    )
                ),
            )
            return

        if self.calibration_stage == 'move_to_axis':
            if self.calibration_move_motion_sign > 0.0:
                self.calibration_distance_traveled_m += max(
                    0.0, self.current_linear_velocity_m_s
                ) * dt
            else:
                self.calibration_distance_traveled_m += max(
                    0.0, -self.current_linear_velocity_m_s
                ) * dt
            if (
                self.calibration_distance_traveled_m
                >= self.calibration_move_distance_target_m
            ):
                if self._start_calibration_rotate_back_or_finish():
                    return
                return
            self._publish_cmd_vel(
                linear_x_m_s=self.calibration_move_motion_sign * linear_limit,
                angular_z_rad_s=0.0,
            )
            return

        if self.calibration_stage == 'rotate_back':
            self.calibration_yaw_traveled_rad += (
                self._integrate_directed_yaw_progress(
                    self.calibration_rotate_back_target_rad,
                    self.current_angular_velocity_rad_s,
                    dt,
                )
            )
            if self.calibration_yaw_traveled_rad >= abs(
                self.calibration_rotate_back_target_rad
            ):
                self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
                self._finish_calibration()
                return
            self._publish_cmd_vel(
                linear_x_m_s=0.0,
                angular_z_rad_s=(
                    math.copysign(
                        angular_limit,
                        self.calibration_rotate_back_target_rad,
                    )
                ),
            )
            return

        self._publish_zero('invalid_calibration_state')

    def _finish_calibration(self) -> None:
        self.control_phase = 'post_calibration_pause'
        self.calibration_stage = ''
        self.calibration_distance_traveled_m = 0.0
        self.calibration_move_distance_target_m = 0.0
        self.calibration_move_motion_sign = 1.0
        self.calibration_yaw_traveled_rad = 0.0
        self.calibration_last_update_time = None
        self.calibration_rotate_out_target_rad = 0.0
        self.calibration_rotate_back_target_rad = 0.0
        self.calibration_resume_time = (
            self.get_clock().now()
            + Duration(seconds=self.calibration_resume_delay_sec)
        )
        self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
        self._set_status('post_calibration_pause')
        self.suppress_robot_docking_target_x_calibration = True
        if self.pending_target_msg is not None and self.pending_target_rx_time is not None:
            self._apply_canonical_target_measurement(
                self.pending_target_msg,
                self.pending_target_rx_time,
            )
            self.pending_target_msg = None
            self.pending_target_rx_time = None
        else:
            self.last_target_rx_time = None
            self.last_target_state_update_time = None

    def _start_target_loss_pause(self, now) -> None:
        if self.control_phase == 'target_loss_pause':
            return
        self.control_phase = 'target_loss_pause'
        self.target_loss_wait_start_time = now
        self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
        self._set_status('target_loss_pause')

    def _run_target_loss_pause(self, now) -> None:
        self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
        if self.target_loss_wait_start_time is None:
            self.target_loss_wait_start_time = now
            return
        wait_dt = (now - self.target_loss_wait_start_time).nanoseconds * 1e-9
        if wait_dt <= self.target_timeout_sec:
            return
        if self.last_target_rx_time is not None:
            dt_target = (now - self.last_target_rx_time).nanoseconds * 1e-9
            if dt_target <= self.target_timeout_sec:
                self.target_loss_wait_start_time = None
                self.control_phase = 'align'
                self._set_status('align')
                return
        self.target_loss_wait_start_time = None
        self._start_calibration(now)
        self._run_calibration(now)

    def _start_scan(self, now) -> None:
        if self.control_phase == 'scan':
            return
        self.control_phase = 'scan'
        self.scan_last_update_time = now
        self.scan_angle_from_center_rad = 0.0
        self.scan_direction_sign = 1.0
        self.scan_settle_end_time = None
        self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
        self._set_status('scan')

    def _run_scan(self, now) -> None:
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
        if self.scan_last_update_time is None:
            self.scan_last_update_time = now
        dt = (now - self.scan_last_update_time).nanoseconds * 1e-9
        if dt < 0.0:
            dt = 0.0
        self.scan_last_update_time = now
        self.scan_angle_from_center_rad += (
            self.current_angular_velocity_rad_s * dt
        )
        half_sweep_rad = math.radians(self.scan_half_sweep_deg)
        if self.scan_angle_from_center_rad >= half_sweep_rad:
            self.scan_angle_from_center_rad = half_sweep_rad
            self.scan_direction_sign = -1.0
        elif self.scan_angle_from_center_rad <= -half_sweep_rad:
            self.scan_angle_from_center_rad = -half_sweep_rad
            self.scan_direction_sign = 1.0
        self._publish_cmd_vel(
            linear_x_m_s=0.0,
            angular_z_rad_s=(
                self.scan_direction_sign
                * self._get_calibration_angular_speed_limit_rad_s()
            ),
        )

    def _start_scan_settle(self, now) -> None:
        self.control_phase = 'scan_settle'
        self.scan_settle_end_time = now + Duration(seconds=self.scan_settle_sec)
        self.scan_last_update_time = None
        self.scan_angle_from_center_rad = 0.0
        self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
        self._set_status('scan_settle')

    def _run_scan_settle(self, now) -> None:
        self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
        if (
            self.last_target_rx_time is None
            or (now - self.last_target_rx_time).nanoseconds * 1e-9
            > self.target_timeout_sec
        ):
            self._start_scan(now)
            return
        if self.scan_settle_end_time is None or now >= self.scan_settle_end_time:
            self.scan_settle_end_time = None
            self.control_phase = 'align'
            self._set_status('align')

    def _run_post_calibration_pause(self, now) -> None:
        self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
        if self.calibration_resume_time is None or now >= self.calibration_resume_time:
            self.calibration_resume_time = None
            self.control_phase = 'align'
            self._set_status('align')

    def _run_final_docking_motion(self, now) -> None:
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

        if self.final_docking_last_update_time is None:
            self.final_docking_last_update_time = now

        dt_forward = (
            now - self.final_docking_last_update_time
        ).nanoseconds * 1e-9
        if dt_forward < 0.0:
            dt_forward = 0.0
        self.final_docking_last_update_time = now

        if self.final_docking_motion_sign > 0.0:
            self.final_docking_distance_traveled_m += (
                max(0.0, self.current_linear_velocity_m_s) * dt_forward
            )
        else:
            self.final_docking_distance_traveled_m += (
                max(0.0, -self.current_linear_velocity_m_s) * dt_forward
            )

        if self.final_docking_distance_traveled_m >= self.final_docking_distance_target_m:
            self._publish_cmd_vel(
                linear_x_m_s=0.0,
                angular_z_rad_s=0.0,
            )
            self._publish_completion_signal(
                docking_target=self.active_docking_target,
                enabled=True,
            )
            self._publish_rl_docking_done(enabled=True)
            self._enter_waiting_mode('waiting_docking_target')
            return

        self._publish_cmd_vel(
            linear_x_m_s=(
                self.final_docking_motion_sign
                * self._get_active_final_linear_speed_m_s()
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

    def _publish_completion_signal(
        self,
        docking_target: int,
        enabled: bool,
    ) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        if docking_target == 1 and self.robot_docking_completion_pub is not None:
            self.robot_docking_completion_pub.publish(msg)
        elif docking_target == 2:
            self.cart_docking_completion_pub.publish(msg)

    def _publish_rl_docking_done(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.rl_docking_done_active = msg.data
        self.rl_docking_done_pub.publish(msg)

    def _get_calibration_angular_speed_limit_rad_s(self) -> float:
        return max(self.near_target_angular_speed_limit_rad_s, 0.3)

    def _get_active_final_distance_m(self) -> float:
        if self.active_docking_target == 1:
            return self.robot_docking_final_distance_m
        return self.cart_docking_final_distance_m

    def _get_active_final_linear_speed_m_s(self) -> float:
        if self.active_docking_target == 1:
            return self.robot_docking_final_linear_speed_m_s
        return self.near_target_linear_speed_limit_m_s

    def _target_is_aligned(
        self,
        target_x_local_m: float,
        target_y_local_m: float,
        target_yaw_error_rad: float,
    ) -> bool:
        xy_tolerance_m, yaw_tolerance_rad = self._get_active_target_tolerances()
        return (
            abs(target_x_local_m) <= xy_tolerance_m
            and abs(target_y_local_m) <= xy_tolerance_m
            and abs(target_yaw_error_rad) <= yaw_tolerance_rad
        )

    def _get_active_target_tolerances(self) -> tuple[float, float]:
        if self.active_docking_target == 1:
            return (
                self.robot_docking_target_xy_stop_tolerance_m,
                self.robot_docking_target_yaw_stop_tolerance_rad,
            )
        return (
            self.target_xy_stop_tolerance_m,
            self.target_yaw_stop_tolerance_rad,
        )

    def _should_start_target_x_calibration(
        self,
        target_x_local_m: float,
        target_y_local_m: float,
        target_yaw_error_rad: float,
    ) -> bool:
        if self.active_docking_target not in (1, 2):
            return False
        if self.suppress_robot_docking_target_x_calibration:
            return False
        if self.control_phase != 'align':
            return False
        if self._target_y_or_yaw_is_aligned(
            target_y_local_m,
            target_yaw_error_rad,
        ):
            return False
        if self.robot_type == 'front':
            return (
                target_x_local_m
                > self.robot_docking_calibration_target_x_threshold_m
            )
        return target_x_local_m < self.rear_calibration_target_x_threshold_m

    def _target_y_or_yaw_is_aligned(
        self,
        target_y_local_m: float,
        target_yaw_error_rad: float,
    ) -> bool:
        xy_tolerance_m, yaw_tolerance_rad = self._get_active_target_tolerances()
        return (
            abs(target_y_local_m) <= xy_tolerance_m
            and abs(target_yaw_error_rad) <= yaw_tolerance_rad
        )

    def _get_calibration_axis_sign(self) -> float:
        return 1.0 if self.robot_type == 'front' else -1.0

    def _get_calibration_safe_axis_x_m(self) -> float:
        if self.robot_type == 'front':
            return self.front_calibration_safe_axis_x_m
        return self.rear_calibration_safe_axis_x_m

    def _start_calibration_rotate_back_or_finish(self) -> bool:
        if self.target_yaw_error_rad is None:
            self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
            self._finish_calibration()
            return True
        self.calibration_rotate_back_target_rad = self.target_yaw_error_rad
        if abs(self.calibration_rotate_back_target_rad) <= 1.0e-9:
            self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
            self._finish_calibration()
            return True
        self.calibration_stage = 'rotate_back'
        self.calibration_yaw_traveled_rad = 0.0
        self._publish_cmd_vel(linear_x_m_s=0.0, angular_z_rad_s=0.0)
        self._set_status('calibration_rotate_back')
        return True

    @staticmethod
    def _canonical_target_from_pose(
        robot_type: str,
        pose_x_m: float,
        pose_y_m: float,
        pose_theta_rad: float,
        base_link_to_axle_center_x_m: float,
        target_x_offset_m: float,
        rear_target_y_offset_m: float,
    ) -> tuple[float, float, float]:
        target_pose_theta_rad = CartAlignSpecialistPolicyNode._wrap_to_pi(
            pose_theta_rad
        )
        target_yaw_error_rad = CartAlignSpecialistPolicyNode._wrap_to_pi(
            -target_pose_theta_rad
        )
        base_target_x_local = pose_x_m
        base_target_y_local = pose_y_m
        if robot_type == 'front':
            base_target_x_local *= -1.0
            base_target_y_local *= -1.0

        axle_target_x_local = (
            base_target_x_local - base_link_to_axle_center_x_m
        )
        axle_target_y_local = base_target_y_local
        cart_offset_sign = 1.0 if robot_type == 'front' else -1.0
        return (
            axle_target_x_local
            + cart_offset_sign
            * target_x_offset_m
            * math.cos(target_yaw_error_rad),
            axle_target_y_local
            + cart_offset_sign
            * target_x_offset_m
            * math.sin(target_yaw_error_rad)
            + (rear_target_y_offset_m if robot_type == 'rear' else 0.0),
            target_yaw_error_rad,
        )

    @staticmethod
    def _compute_calibration_rotate_out_and_move_distance(
        target_x_local: float,
        target_y_local: float,
        target_yaw_error_rad: float,
        calibration_axis_sign: float,
        calibration_safe_axis_x_m: float,
    ) -> tuple[float, float, float]:
        (
            robot_x_target_frame_m,
            robot_y_target_frame_m,
            robot_heading_target_frame_rad,
        ) = CartAlignSpecialistPolicyNode._robot_pose_in_target_frame(
            target_x_local,
            target_y_local,
            target_yaw_error_rad,
        )
        if calibration_axis_sign > 0.0:
            goal_x_target_frame_m = max(
                calibration_safe_axis_x_m,
                robot_x_target_frame_m,
            )
        else:
            goal_x_target_frame_m = min(
                -calibration_safe_axis_x_m,
                robot_x_target_frame_m,
            )
        delta_x_target_frame_m = (
            goal_x_target_frame_m - robot_x_target_frame_m
        )
        delta_y_target_frame_m = -robot_y_target_frame_m
        move_distance_m = math.hypot(
            delta_x_target_frame_m,
            delta_y_target_frame_m,
        )
        if move_distance_m <= 1.0e-9:
            return 0.0, 0.0, 1.0
        move_direction_target_frame_rad = math.atan2(
            delta_y_target_frame_m,
            delta_x_target_frame_m,
        )
        forward_rotate_out_target_rad = (
            CartAlignSpecialistPolicyNode._wrap_to_pi(
                move_direction_target_frame_rad
                - robot_heading_target_frame_rad
            )
        )
        reverse_rotate_out_target_rad = (
            CartAlignSpecialistPolicyNode._wrap_to_pi(
                move_direction_target_frame_rad
                + math.pi
                - robot_heading_target_frame_rad
            )
        )
        if abs(forward_rotate_out_target_rad) <= abs(reverse_rotate_out_target_rad):
            return forward_rotate_out_target_rad, move_distance_m, 1.0
        return reverse_rotate_out_target_rad, move_distance_m, -1.0

    @staticmethod
    def _robot_pose_in_target_frame(
        target_x_local: float,
        target_y_local: float,
        target_yaw_error_rad: float,
    ) -> tuple[float, float, float]:
        robot_heading_target_frame_rad = -target_yaw_error_rad
        cos_theta = math.cos(robot_heading_target_frame_rad)
        sin_theta = math.sin(robot_heading_target_frame_rad)
        robot_x_target_frame_m = -(
            cos_theta * target_x_local - sin_theta * target_y_local
        )
        robot_y_target_frame_m = -(
            sin_theta * target_x_local + cos_theta * target_y_local
        )
        return (
            robot_x_target_frame_m,
            robot_y_target_frame_m,
            robot_heading_target_frame_rad,
        )

    def _update_target_state_from_odometry(self, now) -> None:
        if (
            self.target_x_local_m is None
            or self.target_y_local_m is None
            or self.target_yaw_error_rad is None
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
        self.target_yaw_error_rad = self._wrap_to_pi(
            self.target_yaw_error_rad - delta_yaw
        )
        self.last_target_state_update_time = now

    @staticmethod
    def _integrate_directed_yaw_progress(
        target_angle_rad: float,
        angular_velocity_rad_s: float,
        dt_sec: float,
    ) -> float:
        if dt_sec <= 0.0 or abs(target_angle_rad) <= 1.0e-9:
            return 0.0
        target_direction = math.copysign(1.0, target_angle_rad)
        return max(0.0, target_direction * angular_velocity_rad_s * dt_sec)

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
