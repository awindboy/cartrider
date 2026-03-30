from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    model_path = LaunchConfiguration('model_path')
    target_topic = LaunchConfiguration('target_topic')
    motor_state_topic = LaunchConfiguration('motor_state_topic')
    motor_state_type = LaunchConfiguration('motor_state_type')
    wheel_cmd_topic = LaunchConfiguration('wheel_cmd_topic')
    wheel_cmd_type = LaunchConfiguration('wheel_cmd_type')
    wheel_cmd_item_type = LaunchConfiguration('wheel_cmd_item_type')
    action_scale = LaunchConfiguration('action_scale')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    target_timeout_sec = LaunchConfiguration('target_timeout_sec')
    motor_timeout_sec = LaunchConfiguration('motor_timeout_sec')
    target_xy_stop_tolerance_m = LaunchConfiguration('target_xy_stop_tolerance_m')
    target_yaw_stop_tolerance_deg = LaunchConfiguration('target_yaw_stop_tolerance_deg')
    near_target_distance_m = LaunchConfiguration('near_target_distance_m')
    near_target_speed_limit_rad_s = LaunchConfiguration('near_target_speed_limit_rad_s')
    invert_left = LaunchConfiguration('invert_left')
    invert_right = LaunchConfiguration('invert_right')
    left_motor_id = LaunchConfiguration('left_motor_id')
    right_motor_id = LaunchConfiguration('right_motor_id')

    default_model_path = PathJoinSubstitution(
        [
            FindPackageShare('cart_align_policy'),
            'models',
            'policy_ensemble_p0p1.onnx',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('model_path', default_value=default_model_path),
            DeclareLaunchArgument('target_topic', default_value='/align/target_local'),
            DeclareLaunchArgument('motor_state_topic', default_value='/rmd_state'),
            DeclareLaunchArgument(
                'motor_state_type',
                default_value='cartrider_rmd_sdk/msg/MotorStateArray',
            ),
            DeclareLaunchArgument('wheel_cmd_topic', default_value='/rmd_command'),
            DeclareLaunchArgument(
                'wheel_cmd_type',
                default_value='cartrider_rmd_sdk/msg/MotorCommandArray',
            ),
            DeclareLaunchArgument(
                'wheel_cmd_item_type',
                default_value='cartrider_rmd_sdk/msg/MotorCommand',
            ),
            DeclareLaunchArgument('action_scale', default_value='3.0'),
            DeclareLaunchArgument('control_rate_hz', default_value='40.0'),
            DeclareLaunchArgument('target_timeout_sec', default_value='1000.0'),
            DeclareLaunchArgument('motor_timeout_sec', default_value='1000.0'),
            DeclareLaunchArgument('target_xy_stop_tolerance_m', default_value='0.05'),
            DeclareLaunchArgument('target_yaw_stop_tolerance_deg', default_value='5.0'),
            DeclareLaunchArgument('near_target_distance_m', default_value='0.5'),
            DeclareLaunchArgument('near_target_speed_limit_rad_s', default_value='3.0'),
            DeclareLaunchArgument('invert_left', default_value='false'),
            DeclareLaunchArgument('invert_right', default_value='false'),
            DeclareLaunchArgument('left_motor_id', default_value='1'),
            DeclareLaunchArgument('right_motor_id', default_value='2'),
            Node(
                package='cart_align_policy',
                executable='policy_node',
                name='cart_align_policy_node',
                output='screen',
                parameters=[
                    {
                        'model_path': model_path,
                        'target_topic': target_topic,
                        'motor_state_topic': motor_state_topic,
                        'motor_state_type': motor_state_type,
                        'wheel_cmd_topic': wheel_cmd_topic,
                        'wheel_cmd_type': wheel_cmd_type,
                        'wheel_cmd_item_type': wheel_cmd_item_type,
                        'action_scale': action_scale,
                        'control_rate_hz': control_rate_hz,
                        'target_timeout_sec': target_timeout_sec,
                        'motor_timeout_sec': motor_timeout_sec,
                        'target_xy_stop_tolerance_m': target_xy_stop_tolerance_m,
                        'target_yaw_stop_tolerance_deg': target_yaw_stop_tolerance_deg,
                        'near_target_distance_m': near_target_distance_m,
                        'near_target_speed_limit_rad_s': near_target_speed_limit_rad_s,
                        'invert_left': invert_left,
                        'invert_right': invert_right,
                        'left_motor_id': left_motor_id,
                        'right_motor_id': right_motor_id,
                    }
                ],
            ),
        ]
    )
