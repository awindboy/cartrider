import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PROFILE_DEFAULTS = {
    'rear': {
        'model_file': 'specialist_policy.onnx',
        'linear_velocity_scale_m_s': 0.22,
        'angular_velocity_scale_rad_s': 1.47,
        'spin_in_place_angular_limit_rad_s': 0.29,
        'near_target_linear_speed_limit_m_s': 0.06,
        'near_target_angular_speed_limit_rad_s': 0.37,
        'near_target_distance_m': 0.5,
        'target_x_offset_m': 0.0,
        'motor_state_topic': '/rmd_state',
        'cmd_vel_topic': '/cmd_vel',
        'state_invert_left': True,
        'state_invert_right': False,
        'wheel_radius_m': 0.1100,
        'wheel_separation_m': 0.3000,
        'external_reduction': 1.0,
    },
    'front': {
        'model_file': 'front_specialist_policy.onnx',
        'linear_velocity_scale_m_s': 0.22,
        'angular_velocity_scale_rad_s': 1.81,
        'spin_in_place_angular_limit_rad_s': 0.0,
        'near_target_linear_speed_limit_m_s': 0.06,
        'near_target_angular_speed_limit_rad_s': 0.46,
        'near_target_distance_m': 0.5,
        'target_x_offset_m': 0.0,
        'motor_state_topic': '/front/rmd_state',
        'cmd_vel_topic': '/cmd_vel',
        'state_invert_left': False,
        'state_invert_right': True,
        'wheel_radius_m': 0.0635,
        'wheel_separation_m': 0.2460,
        'external_reduction': 3.0,
    },
}


def _resolve_arg(context, name: str, default):
    value = LaunchConfiguration(name).perform(context).strip()
    if value == '__auto__':
        if default is None:
            raise RuntimeError(
                f'No profile default for {name}. Please pass {name}:=<value>.'
            )
        return str(default)
    return value


def _parse_bool(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ('true', '1', 'yes', 'y', 'on'):
        return True
    if normalized in ('false', '0', 'no', 'n', 'off'):
        return False
    raise RuntimeError(
        f'Invalid boolean for {name}: "{value}". Use true/false or __auto__.'
    )


def _parse_float(value: str, name: str) -> float:
    try:
        return float(value)
    except ValueError as exc:
        raise RuntimeError(f'Invalid float for {name}: "{value}".') from exc


def _parse_int(value: str, name: str) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise RuntimeError(f'Invalid int for {name}: "{value}".') from exc


def _build_policy_node(context):
    robot_type = LaunchConfiguration('robot_type').perform(context).strip().lower()
    if robot_type not in PROFILE_DEFAULTS:
        raise RuntimeError(
            f'robot_type must be one of {list(PROFILE_DEFAULTS.keys())}, got "{robot_type}".'
        )

    profile = PROFILE_DEFAULTS[robot_type]
    share_dir = get_package_share_directory('RL_special')
    default_model_path = os.path.join(share_dir, 'models', profile['model_file'])

    wheel_radius_m = _parse_float(
        _resolve_arg(context, 'wheel_radius_m', profile['wheel_radius_m']),
        'wheel_radius_m',
    )
    wheel_separation_m = _parse_float(
        _resolve_arg(context, 'wheel_separation_m', profile['wheel_separation_m']),
        'wheel_separation_m',
    )
    external_reduction = _parse_float(
        _resolve_arg(
            context,
            'external_reduction',
            profile['external_reduction'],
        ),
        'external_reduction',
    )
    if wheel_radius_m <= 0.0:
        raise RuntimeError('wheel_radius_m must be > 0.')
    if wheel_separation_m <= 0.0:
        raise RuntimeError('wheel_separation_m must be > 0.')
    if external_reduction <= 0.0:
        raise RuntimeError('external_reduction must be > 0.')

    params = {
        'model_path': _resolve_arg(context, 'model_path', default_model_path),
        'target_topic': LaunchConfiguration('target_topic').perform(context),
        'motor_state_topic': _resolve_arg(
            context,
            'motor_state_topic',
            profile['motor_state_topic'],
        ),
        'motor_state_type': LaunchConfiguration('motor_state_type').perform(context),
        'cmd_vel_topic': _resolve_arg(
            context,
            'cmd_vel_topic',
            profile['cmd_vel_topic'],
        ),
        'linear_velocity_scale_m_s': _parse_float(
            _resolve_arg(
                context,
                'linear_velocity_scale_m_s',
                profile['linear_velocity_scale_m_s'],
            ),
            'linear_velocity_scale_m_s',
        ),
        'angular_velocity_scale_rad_s': _parse_float(
            _resolve_arg(
                context,
                'angular_velocity_scale_rad_s',
                profile['angular_velocity_scale_rad_s'],
            ),
            'angular_velocity_scale_rad_s',
        ),
        'spin_in_place_angular_limit_rad_s': _parse_float(
            _resolve_arg(
                context,
                'spin_in_place_angular_limit_rad_s',
                profile['spin_in_place_angular_limit_rad_s'],
            ),
            'spin_in_place_angular_limit_rad_s',
        ),
        'control_rate_hz': _parse_float(
            LaunchConfiguration('control_rate_hz').perform(context),
            'control_rate_hz',
        ),
        'target_timeout_sec': _parse_float(
            LaunchConfiguration('target_timeout_sec').perform(context),
            'target_timeout_sec',
        ),
        'motor_timeout_sec': _parse_float(
            LaunchConfiguration('motor_timeout_sec').perform(context),
            'motor_timeout_sec',
        ),
        'target_xy_stop_tolerance_m': _parse_float(
            LaunchConfiguration('target_xy_stop_tolerance_m').perform(context),
            'target_xy_stop_tolerance_m',
        ),
        'target_yaw_stop_tolerance_deg': _parse_float(
            LaunchConfiguration('target_yaw_stop_tolerance_deg').perform(context),
            'target_yaw_stop_tolerance_deg',
        ),
        'target_x_offset_m': _parse_float(
            _resolve_arg(
                context,
                'target_x_offset_m',
                profile['target_x_offset_m'],
            ),
            'target_x_offset_m',
        ),
        'final_forward_distance_m': _parse_float(
            LaunchConfiguration('final_forward_distance_m').perform(context),
            'final_forward_distance_m',
        ),
        'near_target_distance_m': _parse_float(
            _resolve_arg(
                context,
                'near_target_distance_m',
                profile['near_target_distance_m'],
            ),
            'near_target_distance_m',
        ),
        'near_target_linear_speed_limit_m_s': _parse_float(
            _resolve_arg(
                context,
                'near_target_linear_speed_limit_m_s',
                profile['near_target_linear_speed_limit_m_s'],
            ),
            'near_target_linear_speed_limit_m_s',
        ),
        'near_target_angular_speed_limit_rad_s': _parse_float(
            _resolve_arg(
                context,
                'near_target_angular_speed_limit_rad_s',
                profile['near_target_angular_speed_limit_rad_s'],
            ),
            'near_target_angular_speed_limit_rad_s',
        ),
        'wheel_radius_m': wheel_radius_m,
        'wheel_separation_m': wheel_separation_m,
        'external_reduction': external_reduction,
        'state_invert_left': _parse_bool(
            _resolve_arg(
                context,
                'state_invert_left',
                str(profile['state_invert_left']).lower(),
            ),
            'state_invert_left',
        ),
        'state_invert_right': _parse_bool(
            _resolve_arg(
                context,
                'state_invert_right',
                str(profile['state_invert_right']).lower(),
            ),
            'state_invert_right',
        ),
        'left_motor_id': _parse_int(
            LaunchConfiguration('left_motor_id').perform(context),
            'left_motor_id',
        ),
        'right_motor_id': _parse_int(
            LaunchConfiguration('right_motor_id').perform(context),
            'right_motor_id',
        ),
    }

    return [
        Node(
            package='RL_special',
            executable='specialist_policy_node',
            name='RL_special_node',
            output='screen',
            parameters=[params],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_type',
                default_value='front',
                choices=['rear', 'front'],
                description='Robot profile selector: rear or front',
            ),
            DeclareLaunchArgument(
                'model_path',
                default_value='__auto__',
                description='__auto__ uses profile default model path.',
            ),
            DeclareLaunchArgument(
                'linear_velocity_scale_m_s',
                default_value='__auto__',
                description='__auto__ derives the linear action scale from wheel geometry and legacy limits.',
            ),
            DeclareLaunchArgument(
                'angular_velocity_scale_rad_s',
                default_value='__auto__',
                description='__auto__ derives the angular action scale from wheel geometry and legacy limits.',
            ),
            DeclareLaunchArgument(
                'spin_in_place_angular_limit_rad_s',
                default_value='__auto__',
                description='__auto__ derives the in-place angular limit from wheel geometry and legacy limits.',
            ),
            DeclareLaunchArgument(
                'near_target_linear_speed_limit_m_s',
                default_value='__auto__',
                description='__auto__ derives the near-target linear limit from wheel geometry and legacy limits.',
            ),
            DeclareLaunchArgument(
                'near_target_angular_speed_limit_rad_s',
                default_value='__auto__',
                description='__auto__ derives the near-target angular limit from wheel geometry and legacy limits.',
            ),
            DeclareLaunchArgument(
                'near_target_distance_m',
                default_value='__auto__',
                description='__auto__ uses profile default near-target distance.',
            ),
            DeclareLaunchArgument(
                'target_topic',
                default_value='/rs/cart_pose',
            ),
            DeclareLaunchArgument(
                'motor_state_topic',
                default_value='__auto__',
                description='__auto__ uses profile default motor_state_topic.',
            ),
            DeclareLaunchArgument(
                'motor_state_type',
                default_value='cartrider_rmd_sdk/msg/MotorStateArray',
            ),
            DeclareLaunchArgument(
                'cmd_vel_topic',
                default_value='__auto__',
                description='__auto__ uses profile default cmd_vel_topic.',
            ),
            DeclareLaunchArgument(
                'wheel_radius_m',
                default_value='__auto__',
                description='Drive wheel radius in meters. __auto__ uses profile default.',
            ),
            DeclareLaunchArgument(
                'wheel_separation_m',
                default_value='__auto__',
                description='Distance between left/right wheel contact centers in meters. __auto__ uses profile default.',
            ),
            DeclareLaunchArgument(
                'external_reduction',
                default_value='__auto__',
                description='External motor-to-wheel reduction ratio. __auto__ uses profile default.',
            ),
            DeclareLaunchArgument(
                'state_invert_left',
                default_value='__auto__',
                description='__auto__ uses profile default left motor-state sign correction.',
            ),
            DeclareLaunchArgument(
                'state_invert_right',
                default_value='__auto__',
                description='__auto__ uses profile default right motor-state sign correction.',
            ),
            DeclareLaunchArgument('control_rate_hz', default_value='30.0'),
            DeclareLaunchArgument('target_timeout_sec', default_value='1000.0'),
            DeclareLaunchArgument('motor_timeout_sec', default_value='1000.0'),
            DeclareLaunchArgument('target_xy_stop_tolerance_m', default_value='0.02'),
            DeclareLaunchArgument('target_yaw_stop_tolerance_deg', default_value='2.0'),
            DeclareLaunchArgument(
                'target_x_offset_m',
                default_value='__auto__',
                description='Offset subtracted from target x to align the robot drive axle center to the cart.',
            ),
            DeclareLaunchArgument('final_forward_distance_m', default_value='0.35'),
            DeclareLaunchArgument('left_motor_id', default_value='1'),
            DeclareLaunchArgument('right_motor_id', default_value='2'),
            OpaqueFunction(function=_build_policy_node),
        ]
    )
