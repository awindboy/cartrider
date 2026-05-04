from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_type = LaunchConfiguration('robot_type')
    rear_model_path = LaunchConfiguration('rear_model_path')
    front_model_path = LaunchConfiguration('front_model_path')
    rear_action_scale = LaunchConfiguration('rear_action_scale')
    front_action_scale = LaunchConfiguration('front_action_scale')
    rear_near_target_speed_limit_rad_s = LaunchConfiguration(
        'rear_near_target_speed_limit_rad_s'
    )
    front_near_target_speed_limit_rad_s = LaunchConfiguration(
        'front_near_target_speed_limit_rad_s'
    )
    rear_motor_state_topic = LaunchConfiguration('rear_motor_state_topic')
    front_motor_state_topic = LaunchConfiguration('front_motor_state_topic')
    rear_wheel_cmd_topic = LaunchConfiguration('rear_wheel_cmd_topic')
    front_wheel_cmd_topic = LaunchConfiguration('front_wheel_cmd_topic')

    target_topic = LaunchConfiguration('target_topic')
    motor_state_type = LaunchConfiguration('motor_state_type')
    wheel_cmd_type = LaunchConfiguration('wheel_cmd_type')
    wheel_cmd_item_type = LaunchConfiguration('wheel_cmd_item_type')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    target_timeout_sec = LaunchConfiguration('target_timeout_sec')
    motor_timeout_sec = LaunchConfiguration('motor_timeout_sec')
    target_xy_stop_tolerance_m = LaunchConfiguration('target_xy_stop_tolerance_m')
    target_yaw_stop_tolerance_deg = LaunchConfiguration('target_yaw_stop_tolerance_deg')
    near_target_distance_m = LaunchConfiguration('near_target_distance_m')
    invert_left = LaunchConfiguration('invert_left')
    invert_right = LaunchConfiguration('invert_right')
    left_motor_id = LaunchConfiguration('left_motor_id')
    right_motor_id = LaunchConfiguration('right_motor_id')

    default_rear_model_path = PathJoinSubstitution(
        [
            FindPackageShare('cart_align_policy'),
            'models',
            'policy.onnx',
        ]
    )
    default_front_model_path = PathJoinSubstitution(
        [
            FindPackageShare('cart_align_policy'),
            'models',
            'front_policy.onnx',
        ]
    )

    common_params = {
        'target_topic': target_topic,
        'motor_state_type': motor_state_type,
        'wheel_cmd_type': wheel_cmd_type,
        'wheel_cmd_item_type': wheel_cmd_item_type,
        'control_rate_hz': control_rate_hz,
        'target_timeout_sec': target_timeout_sec,
        'motor_timeout_sec': motor_timeout_sec,
        'target_xy_stop_tolerance_m': target_xy_stop_tolerance_m,
        'target_yaw_stop_tolerance_deg': target_yaw_stop_tolerance_deg,
        'near_target_distance_m': near_target_distance_m,
        'invert_left': invert_left,
        'invert_right': invert_right,
        'left_motor_id': left_motor_id,
        'right_motor_id': right_motor_id,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_type',
                default_value='front',
                choices=['rear', 'front'],
                description='Robot profile selector: rear or front',
            ),
            DeclareLaunchArgument(
                'rear_model_path',
                default_value=default_rear_model_path,
            ),
            DeclareLaunchArgument(
                'front_model_path',
                default_value=default_front_model_path,
            ),
            DeclareLaunchArgument('rear_action_scale', default_value='2.0'),
            DeclareLaunchArgument('front_action_scale', default_value='3.5'),
            DeclareLaunchArgument(
                'rear_near_target_speed_limit_rad_s',
                default_value='0.5',
            ),
            DeclareLaunchArgument(
                'front_near_target_speed_limit_rad_s',
                default_value='0.9',
            ),
            DeclareLaunchArgument('target_topic', default_value='/align/target_local'),
            DeclareLaunchArgument('rear_motor_state_topic', default_value='/rmd_state'),
            DeclareLaunchArgument(
                'front_motor_state_topic',
                default_value='/front/rmd_state',
            ),
            DeclareLaunchArgument(
                'motor_state_type',
                default_value='cartrider_rmd_sdk/msg/MotorStateArray',
            ),
            DeclareLaunchArgument('rear_wheel_cmd_topic', default_value='/rmd_command'),
            DeclareLaunchArgument(
                'front_wheel_cmd_topic',
                default_value='/front/rmd_command',
            ),
            DeclareLaunchArgument(
                'wheel_cmd_type',
                default_value='cartrider_rmd_sdk/msg/MotorCommandArray',
            ),
            DeclareLaunchArgument(
                'wheel_cmd_item_type',
                default_value='cartrider_rmd_sdk/msg/MotorCommand',
            ),
            DeclareLaunchArgument('control_rate_hz', default_value='40.0'),
            DeclareLaunchArgument('target_timeout_sec', default_value='1000.0'),
            DeclareLaunchArgument('motor_timeout_sec', default_value='1000.0'),
            DeclareLaunchArgument('target_xy_stop_tolerance_m', default_value='0.05'),
            DeclareLaunchArgument('target_yaw_stop_tolerance_deg', default_value='5.0'),
            DeclareLaunchArgument('near_target_distance_m', default_value='0.5'),
            DeclareLaunchArgument('invert_left', default_value='true'),
            DeclareLaunchArgument('invert_right', default_value='false'),
            DeclareLaunchArgument('left_motor_id', default_value='1'),
            DeclareLaunchArgument('right_motor_id', default_value='2'),
            Node(
                package='cart_align_policy',
                executable='policy_node',
                name='cart_align_policy_node',
                output='screen',
                condition=IfCondition(
                    PythonExpression(["'", robot_type, "' == 'rear'"])
                ),
                parameters=[
                    {
                        **common_params,
                        'motor_state_topic': rear_motor_state_topic,
                        'wheel_cmd_topic': rear_wheel_cmd_topic,
                        'model_path': rear_model_path,
                        'action_scale': rear_action_scale,
                        'near_target_speed_limit_rad_s': rear_near_target_speed_limit_rad_s,
                    }
                ],
            ),
            Node(
                package='cart_align_policy',
                executable='policy_node',
                name='cart_align_policy_node',
                output='screen',
                condition=IfCondition(
                    PythonExpression(["'", robot_type, "' == 'front'"])
                ),
                parameters=[
                    {
                        **common_params,
                        'motor_state_topic': front_motor_state_topic,
                        'wheel_cmd_topic': front_wheel_cmd_topic,
                        'model_path': front_model_path,
                        'action_scale': front_action_scale,
                        'near_target_speed_limit_rad_s': front_near_target_speed_limit_rad_s,
                    }
                ],
            ),
        ]
    )
