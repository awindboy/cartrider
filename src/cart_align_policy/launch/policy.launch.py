from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    model_path = LaunchConfiguration('model_path')
    left_joint_name = LaunchConfiguration('left_joint_name')
    right_joint_name = LaunchConfiguration('right_joint_name')
    action_scale = LaunchConfiguration('action_scale')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    target_timeout_sec = LaunchConfiguration('target_timeout_sec')
    invert_left = LaunchConfiguration('invert_left')
    invert_right = LaunchConfiguration('invert_right')

    default_model_path = PathJoinSubstitution(
        [FindPackageShare('cart_align_policy'), 'models', 'policy.onnx']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('model_path', default_value=default_model_path),
            DeclareLaunchArgument('left_joint_name', default_value='left_wheel_joint'),
            DeclareLaunchArgument('right_joint_name', default_value='right_wheel_joint'),
            DeclareLaunchArgument('action_scale', default_value='4.0055306333'),
            DeclareLaunchArgument('control_rate_hz', default_value='10.0'),
            DeclareLaunchArgument('target_timeout_sec', default_value='0.3'),
            DeclareLaunchArgument('invert_left', default_value='false'),
            DeclareLaunchArgument('invert_right', default_value='false'),
            Node(
                package='cart_align_policy',
                executable='policy_node',
                name='cart_align_policy_node',
                output='screen',
                parameters=[
                    {
                        'model_path': model_path,
                        'left_joint_name': left_joint_name,
                        'right_joint_name': right_joint_name,
                        'action_scale': action_scale,
                        'control_rate_hz': control_rate_hz,
                        'target_timeout_sec': target_timeout_sec,
                        'invert_left': invert_left,
                        'invert_right': invert_right,
                    }
                ],
            ),
        ]
    )
