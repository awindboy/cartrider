#!/usr/bin/env python3
import math
import select
import sys
import termios
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


class OdometryKeyboardDebugNode(Node):
    def __init__(self) -> None:
        super().__init__('RL_special_odometry_debug_node')

        self.declare_parameter('robot_type', 'front')
        self.declare_parameter('motor_state_topic', '/front/rmd_state')
        self.declare_parameter(
            'motor_state_type',
            'cartrider_rmd_sdk/msg/MotorStateArray',
        )
        self.declare_parameter('cmd_vel_topic', '/front/cmd_vel')
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('motor_timeout_sec', 1.0)
        self.declare_parameter('move_distance_m', 0.30)
        self.declare_parameter('turn_angle_deg', 90.0)
        self.declare_parameter('linear_speed_m_s', 0.06)
        self.declare_parameter('angular_speed_rad_s', 0.30)
        self.declare_parameter('wheel_radius_m', 0.0635)
        self.declare_parameter('wheel_separation_m', 0.2460)
        self.declare_parameter('external_reduction', 1.0)
        self.declare_parameter('linear_odometry_scale', 1.0)
        self.declare_parameter('angular_odometry_scale', 1.0)
        self.declare_parameter('state_invert_left', False)
        self.declare_parameter('state_invert_right', True)
        self.declare_parameter('left_motor_id', 1)
        self.declare_parameter('right_motor_id', 2)
        self.declare_parameter('status_period_sec', 0.5)

        self.robot_type = str(self.get_parameter('robot_type').value).strip().lower()
        self.motor_state_topic = str(self.get_parameter('motor_state_topic').value)
        self.motor_state_type = str(self.get_parameter('motor_state_type').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.motor_timeout_sec = float(self.get_parameter('motor_timeout_sec').value)
        self.move_distance_m = float(self.get_parameter('move_distance_m').value)
        self.turn_angle_rad = math.radians(
            float(self.get_parameter('turn_angle_deg').value)
        )
        self.linear_speed_m_s = float(self.get_parameter('linear_speed_m_s').value)
        self.angular_speed_rad_s = float(
            self.get_parameter('angular_speed_rad_s').value
        )
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_separation_m = float(
            self.get_parameter('wheel_separation_m').value
        )
        self.external_reduction = float(self.get_parameter('external_reduction').value)
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
        self.status_period_sec = float(self.get_parameter('status_period_sec').value)

        self._validate_parameters()

        motor_state_msg_type = get_message(self.motor_state_type)
        self.motor_sub = self.create_subscription(
            motor_state_msg_type,
            self.motor_state_topic,
            self._motor_state_callback,
            qos_profile_sensor_data,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.current_linear_velocity_m_s: Optional[float] = None
        self.current_angular_velocity_rad_s: Optional[float] = None
        self.last_motor_rx_time = None
        self.last_update_time = self.get_clock().now()
        self.last_status_time = self.get_clock().now()

        self.active_kind = ''
        self.active_direction = 0.0
        self.active_target = 0.0
        self.active_progress = 0.0
        self.current_status = ''

        self.stdin_fd: Optional[int] = None
        self.original_terminal_settings = None
        self._configure_terminal()

        timer_period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self._print_help()
        self._set_status('idle')

    def _validate_parameters(self) -> None:
        if self.robot_type not in ('front', 'rear'):
            raise ValueError('robot_type must be "front" or "rear".')
        if self.control_rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be > 0.')
        if self.motor_timeout_sec < 0.0:
            raise ValueError('motor_timeout_sec must be >= 0.')
        if self.move_distance_m <= 0.0:
            raise ValueError('move_distance_m must be > 0.')
        if self.turn_angle_rad <= 0.0:
            raise ValueError('turn_angle_deg must be > 0.')
        if self.linear_speed_m_s <= 0.0:
            raise ValueError('linear_speed_m_s must be > 0.')
        if self.angular_speed_rad_s <= 0.0:
            raise ValueError('angular_speed_rad_s must be > 0.')
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
        if self.status_period_sec < 0.0:
            raise ValueError('status_period_sec must be >= 0.')

    def _configure_terminal(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warning(
                'stdin is not a TTY. Keyboard input may not work from this launch context.'
            )
            return
        self.stdin_fd = sys.stdin.fileno()
        self.original_terminal_settings = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)

    def destroy_node(self) -> bool:
        self._publish_zero()
        if self.stdin_fd is not None and self.original_terminal_settings is not None:
            termios.tcsetattr(
                self.stdin_fd,
                termios.TCSADRAIN,
                self.original_terminal_settings,
            )
            self.stdin_fd = None
            self.original_terminal_settings = None
        return super().destroy_node()

    def _print_help(self) -> None:
        self.get_logger().info(
            'Odometry debug keys: ↑/w=+30cm, ↓/s=-30cm, '
            '←/a=+90deg, →/d=-90deg, space=stop, q=quit'
        )
        self.get_logger().info(
            f'profile={self.robot_type}, cmd_vel={self.cmd_vel_topic}, '
            f'motor_state={self.motor_state_topic}'
        )

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

    def _timer_callback(self) -> None:
        now = self.get_clock().now()
        self._handle_keyboard()
        self._update_active_command(now)
        self.last_update_time = now

    def _handle_keyboard(self) -> None:
        key = self._read_key()
        if not key:
            return

        if key in ('UP', 'w', 'W'):
            self._start_command('move_forward', 1.0, self.move_distance_m)
        elif key in ('DOWN', 's', 'S'):
            self._start_command('move_backward', -1.0, self.move_distance_m)
        elif key in ('LEFT', 'a', 'A'):
            self._start_command('turn_left', 1.0, self.turn_angle_rad)
        elif key in ('RIGHT', 'd', 'D'):
            self._start_command('turn_right', -1.0, self.turn_angle_rad)
        elif key == ' ':
            self._cancel_active_command('manual_stop')
        elif key in ('q', 'Q'):
            self._cancel_active_command('quit')
            rclpy.shutdown()

    def _read_key(self) -> str:
        if self.stdin_fd is None:
            return ''
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return ''
        char = sys.stdin.read(1)
        if char != '\x1b':
            return char
        suffix = ''
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0.01)
            if not readable:
                break
            suffix += sys.stdin.read(1)
            if len(suffix) >= 2:
                break
        return {
            '[A': 'UP',
            '[B': 'DOWN',
            '[C': 'RIGHT',
            '[D': 'LEFT',
        }.get(suffix, '')

    def _start_command(self, kind: str, direction: float, target: float) -> None:
        self.active_kind = kind
        self.active_direction = direction
        self.active_target = target
        self.active_progress = 0.0
        self.last_update_time = self.get_clock().now()
        self.last_status_time = self.last_update_time
        self._set_status(kind)

    def _cancel_active_command(self, status: str) -> None:
        self.active_kind = ''
        self.active_direction = 0.0
        self.active_target = 0.0
        self.active_progress = 0.0
        self._publish_zero()
        self._set_status(status)

    def _update_active_command(self, now) -> None:
        if not self.active_kind:
            self._publish_zero()
            return

        if not self._motor_velocity_is_valid(now):
            self._publish_zero()
            self._set_status('waiting_motor_vel')
            return

        if self.current_status == 'waiting_motor_vel':
            self._set_status(self.active_kind)

        dt = (now - self.last_update_time).nanoseconds * 1.0e-9
        if dt < 0.0:
            dt = 0.0

        if self.active_kind.startswith('move'):
            measured_delta = (
                self.active_direction * self.current_linear_velocity_m_s * dt
            )
            command = Twist()
            command.linear.x = self.active_direction * self.linear_speed_m_s
        else:
            measured_delta = (
                self.active_direction * self.current_angular_velocity_rad_s * dt
            )
            command = Twist()
            command.angular.z = self.active_direction * self.angular_speed_rad_s

        self.active_progress += max(0.0, measured_delta)
        if self.active_progress >= self.active_target:
            self._cancel_active_command(f'{self.active_kind}_done')
            return

        self.cmd_pub.publish(command)
        self._log_progress(now)

    def _motor_velocity_is_valid(self, now) -> bool:
        if (
            self.current_linear_velocity_m_s is None
            or self.current_angular_velocity_rad_s is None
            or self.last_motor_rx_time is None
        ):
            return False
        dt_motor = (now - self.last_motor_rx_time).nanoseconds * 1.0e-9
        return dt_motor <= self.motor_timeout_sec

    def _log_progress(self, now) -> None:
        if self.status_period_sec <= 0.0:
            return
        dt_status = (now - self.last_status_time).nanoseconds * 1.0e-9
        if dt_status < self.status_period_sec:
            return
        self.last_status_time = now
        unit = 'm' if self.active_kind.startswith('move') else 'rad'
        self.get_logger().info(
            f'mode={self.active_kind} progress={self.active_progress:.3f}/'
            f'{self.active_target:.3f}{unit} v={self.current_linear_velocity_m_s:.3f} '
            f'omega={self.current_angular_velocity_rad_s:.3f}'
        )

    def _publish_zero(self) -> None:
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def _set_status(self, status: str) -> None:
        if self.current_status == status:
            return
        self.current_status = status
        self.get_logger().info(f'mode={status}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdometryKeyboardDebugNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
