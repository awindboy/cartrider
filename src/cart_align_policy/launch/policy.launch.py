import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PROFILE_DEFAULTS = {
    'rear': {
        'model_file': 'policy.onnx',
        'action_scale': 2.0,
        'spin_in_place_limit_rad_s': 0.4,
        'near_target_speed_limit_rad_s': 0.5,
        'near_target_distance_m': 0.5,
        'motor_state_topic': '/rmd_state',
        'wheel_cmd_topic': '/rmd_command',
        'invert_left': True,
        'invert_right': False,
    },
    'front': {
        'model_file': 'front_policy.onnx',
        'action_scale': 3.5,
        'spin_in_place_limit_rad_s': 0.0,
        'near_target_speed_limit_rad_s': 0.9,
        'near_target_distance_m': 0.5,
        'motor_state_topic': '/front/rmd_state',
        'wheel_cmd_topic': '/front/rmd_command',
        'invert_left': False,
        'invert_right': True,
    },
}


def _resolve_arg(context, name: str, default: str) -> str:
    value = LaunchConfiguration(name).perform(context).strip()
    if value == '__auto__':
        return default
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
        raise RuntimeError(
            f'Invalid float for {name}: "{value}".'
        ) from exc


def _parse_int(value: str, name: str) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise RuntimeError(
            f'Invalid int for {name}: "{value}".'
        ) from exc


def _build_policy_node(context):
    robot_type = LaunchConfiguration('robot_type').perform(context).strip().lower()
    if robot_type not in PROFILE_DEFAULTS:
        raise RuntimeError(
            f'robot_type must be one of {list(PROFILE_DEFAULTS.keys())}, got "{robot_type}".'
        )

    profile = PROFILE_DEFAULTS[robot_type]
    share_dir = get_package_share_directory('cart_align_policy')
    default_model_path = os.path.join(share_dir, 'models', profile['model_file'])

    params = {
        'model_path': _resolve_arg(context, 'model_path', default_model_path),
        'target_topic': LaunchConfiguration('target_topic').perform(context),
        'motor_state_topic': _resolve_arg(
            context,
            'motor_state_topic',
            profile['motor_state_topic'],
        ),
        'motor_state_type': LaunchConfiguration('motor_state_type').perform(context),
        'wheel_cmd_topic': _resolve_arg(
            context,
            'wheel_cmd_topic',
            profile['wheel_cmd_topic'],
        ),
        'wheel_cmd_type': LaunchConfiguration('wheel_cmd_type').perform(context),
        'wheel_cmd_item_type': LaunchConfiguration('wheel_cmd_item_type').perform(
            context
        ),
        'action_scale': _parse_float(
            _resolve_arg(
                context,
                'action_scale',
                str(profile['action_scale']),
            ),
            'action_scale',
        ),
        'spin_in_place_limit_rad_s': _parse_float(
            _resolve_arg(
                context,
                'spin_in_place_limit_rad_s',
                str(profile['spin_in_place_limit_rad_s']),
            ),
            'spin_in_place_limit_rad_s',
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
        'near_target_distance_m': _parse_float(
            _resolve_arg(
                context,
                'near_target_distance_m',
                str(profile['near_target_distance_m']),
            ),
            'near_target_distance_m',
        ),
        'near_target_speed_limit_rad_s': _parse_float(
            _resolve_arg(
                context,
                'near_target_speed_limit_rad_s',
                str(profile['near_target_speed_limit_rad_s']),
            ),
            'near_target_speed_limit_rad_s',
        ),
        'invert_left': _parse_bool(
            _resolve_arg(
                context,
                'invert_left',
                str(profile['invert_left']).lower(),
            ),
            'invert_left',
        ),
        'invert_right': _parse_bool(
            _resolve_arg(
                context,
                'invert_right',
                str(profile['invert_right']).lower(),
            ),
            'invert_right',
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
            package='cart_align_policy',
            executable='policy_node',
            name='cart_align_policy_node',
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
                'action_scale',
                default_value='__auto__',
                description='__auto__ uses profile default action scale.',
            ),
            DeclareLaunchArgument(
                'spin_in_place_limit_rad_s',
                default_value='__auto__',
                description='__auto__ uses profile default in-place turn speed limit.',
            ),
            DeclareLaunchArgument(
                'near_target_speed_limit_rad_s',
                default_value='__auto__',
                description='__auto__ uses profile default near-target speed limit.',
            ),
            DeclareLaunchArgument(
                'near_target_distance_m',
                default_value='__auto__',
                description='__auto__ uses profile default near-target distance.',
            ),
            DeclareLaunchArgument('target_topic', default_value='/align/target_local'),
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
                'wheel_cmd_topic',
                default_value='__auto__',
                description='__auto__ uses profile default wheel_cmd_topic.',
            ),
            DeclareLaunchArgument(
                'wheel_cmd_type',
                default_value='cartrider_rmd_sdk/msg/MotorCommandArray',
            ),
            DeclareLaunchArgument(
                'wheel_cmd_item_type',
                default_value='cartrider_rmd_sdk/msg/MotorCommand',
            ),
            DeclareLaunchArgument(
                'invert_left',
                default_value='__auto__',
                description='__auto__ uses profile default invert_left.',
            ),
            DeclareLaunchArgument(
                'invert_right',
                default_value='__auto__',
                description='__auto__ uses profile default invert_right.',
            ),
            DeclareLaunchArgument('control_rate_hz', default_value='40.0'),
            DeclareLaunchArgument('target_timeout_sec', default_value='1000.0'),
            DeclareLaunchArgument('motor_timeout_sec', default_value='1000.0'),
            DeclareLaunchArgument('target_xy_stop_tolerance_m', default_value='0.05'),
            DeclareLaunchArgument('target_yaw_stop_tolerance_deg', default_value='5.0'),
            DeclareLaunchArgument('left_motor_id', default_value='1'),
            DeclareLaunchArgument('right_motor_id', default_value='2'),
            OpaqueFunction(function=_build_policy_node),
        ]
    )
