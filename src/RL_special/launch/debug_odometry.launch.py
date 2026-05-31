from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PROFILE_DEFAULTS = {
    'rear': {
        'motor_state_topic': '/rmd_state',
        'cmd_vel_topic': '/cmd_vel',
        'state_invert_left': True,
        'state_invert_right': False,
        'wheel_radius_m': 0.1100,
        'wheel_separation_m': 0.3000,
        'external_reduction': 1.0,
        'linear_odometry_scale': 1.0,
        'angular_odometry_scale': 1.0,
    },
    'front': {
        'motor_state_topic': '/front/rmd_state',
        'cmd_vel_topic': '/front/cmd_vel',
        'state_invert_left': False,
        'state_invert_right': True,
        'wheel_radius_m': 0.0635,
        'wheel_separation_m': 0.2460,
        'external_reduction': 1.0,
        'linear_odometry_scale': 1.0,
        'angular_odometry_scale': 1.0,
    },
}


def _resolve_arg(context, name: str, default):
    value = LaunchConfiguration(name).perform(context).strip()
    if value == '__auto__':
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


def _build_debug_node(context):
    robot_type = LaunchConfiguration('robot_type').perform(context).strip().lower()
    if robot_type not in PROFILE_DEFAULTS:
        raise RuntimeError(
            f'robot_type must be one of {list(PROFILE_DEFAULTS.keys())}, got "{robot_type}".'
        )

    profile = PROFILE_DEFAULTS[robot_type]
    wheel_radius_m = _parse_float(
        _resolve_arg(context, 'wheel_radius_m', profile['wheel_radius_m']),
        'wheel_radius_m',
    )
    wheel_separation_m = _parse_float(
        _resolve_arg(context, 'wheel_separation_m', profile['wheel_separation_m']),
        'wheel_separation_m',
    )
    external_reduction = _parse_float(
        _resolve_arg(context, 'external_reduction', profile['external_reduction']),
        'external_reduction',
    )
    if wheel_radius_m <= 0.0:
        raise RuntimeError('wheel_radius_m must be > 0.')
    if wheel_separation_m <= 0.0:
        raise RuntimeError('wheel_separation_m must be > 0.')
    if external_reduction <= 0.0:
        raise RuntimeError('external_reduction must be > 0.')

    params = {
        'robot_type': robot_type,
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
        'control_rate_hz': _parse_float(
            LaunchConfiguration('control_rate_hz').perform(context),
            'control_rate_hz',
        ),
        'motor_timeout_sec': _parse_float(
            LaunchConfiguration('motor_timeout_sec').perform(context),
            'motor_timeout_sec',
        ),
        'move_distance_m': _parse_float(
            LaunchConfiguration('move_distance_m').perform(context),
            'move_distance_m',
        ),
        'turn_angle_deg': _parse_float(
            LaunchConfiguration('turn_angle_deg').perform(context),
            'turn_angle_deg',
        ),
        'linear_speed_m_s': _parse_float(
            LaunchConfiguration('linear_speed_m_s').perform(context),
            'linear_speed_m_s',
        ),
        'angular_speed_rad_s': _parse_float(
            LaunchConfiguration('angular_speed_rad_s').perform(context),
            'angular_speed_rad_s',
        ),
        'wheel_radius_m': wheel_radius_m,
        'wheel_separation_m': wheel_separation_m,
        'external_reduction': external_reduction,
        'linear_odometry_scale': _parse_float(
            _resolve_arg(
                context,
                'linear_odometry_scale',
                profile['linear_odometry_scale'],
            ),
            'linear_odometry_scale',
        ),
        'angular_odometry_scale': _parse_float(
            _resolve_arg(
                context,
                'angular_odometry_scale',
                profile['angular_odometry_scale'],
            ),
            'angular_odometry_scale',
        ),
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
        'status_period_sec': _parse_float(
            LaunchConfiguration('status_period_sec').perform(context),
            'status_period_sec',
        ),
    }

    return [
        Node(
            package='RL_special',
            executable='debug_odometry_keyboard_node',
            name='RL_special_odometry_debug_node',
            output='screen',
            emulate_tty=True,
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
                description='Robot profile selector: rear or front.',
            ),
            DeclareLaunchArgument(
                'motor_state_topic',
                default_value='__auto__',
                description='__auto__ uses the selected robot profile default.',
            ),
            DeclareLaunchArgument(
                'motor_state_type',
                default_value='cartrider_rmd_sdk/msg/MotorStateArray',
            ),
            DeclareLaunchArgument(
                'cmd_vel_topic',
                default_value='__auto__',
                description='__auto__ uses the selected robot profile default.',
            ),
            DeclareLaunchArgument('control_rate_hz', default_value='30.0'),
            DeclareLaunchArgument('motor_timeout_sec', default_value='1.0'),
            DeclareLaunchArgument('move_distance_m', default_value='0.30'),
            DeclareLaunchArgument('turn_angle_deg', default_value='90.0'),
            DeclareLaunchArgument('linear_speed_m_s', default_value='0.06'),
            DeclareLaunchArgument('angular_speed_rad_s', default_value='0.30'),
            DeclareLaunchArgument(
                'wheel_radius_m',
                default_value='__auto__',
                description='Drive wheel radius in meters. __auto__ uses profile default.',
            ),
            DeclareLaunchArgument(
                'wheel_separation_m',
                default_value='__auto__',
                description='Drive track width in meters. __auto__ uses profile default.',
            ),
            DeclareLaunchArgument(
                'external_reduction',
                default_value='__auto__',
                description='Motor-to-wheel reduction ratio. __auto__ uses profile default.',
            ),
            DeclareLaunchArgument(
                'linear_odometry_scale',
                default_value='__auto__',
                description='Multiplier applied to odometry linear velocity.',
            ),
            DeclareLaunchArgument(
                'angular_odometry_scale',
                default_value='__auto__',
                description='Multiplier applied to odometry angular velocity.',
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
            DeclareLaunchArgument('left_motor_id', default_value='1'),
            DeclareLaunchArgument('right_motor_id', default_value='2'),
            DeclareLaunchArgument('status_period_sec', default_value='0.5'),
            OpaqueFunction(function=_build_debug_node),
        ]
    )
