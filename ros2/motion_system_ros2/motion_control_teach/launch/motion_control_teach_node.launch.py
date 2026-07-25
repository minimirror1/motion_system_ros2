import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MOTION_SYSTEM_FILES_DIR = os.environ.get(
    'MOTION_SYSTEM_FILES_DIR',
    os.path.expanduser('~/colcon_ws/files'),
)
DEFAULT_MOTOR_CONFIG_FILE = os.path.join(
    MOTION_SYSTEM_FILES_DIR,
    'motor_manager',
    'active_motor_manager.yaml',
)
DEFAULT_ROBOT_CONFIG_FILE = os.path.join(
    MOTION_SYSTEM_FILES_DIR,
    'robot_manager',
    'active_robot_manager.yaml',
)


def generate_launch_description():
    motor_config_file = LaunchConfiguration('motor_config_file')
    robot_config_file = LaunchConfiguration('robot_config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'motor_config_file',
            default_value=DEFAULT_MOTOR_CONFIG_FILE,
            description='Absolute path to motor_manager YAML.',
        ),
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=DEFAULT_ROBOT_CONFIG_FILE,
            description=(
                'Robot YAML containing controller_indices and motion_data_file_path.'
            ),
        ),
        Node(
            package='motion_control_bridge',
            executable='motor_manager_node',
            name='motor_manager_node',
            output='screen',
            parameters=[{
                'config_file': motor_config_file,
            }],
        ),
        Node(
            package='motion_control_teach',
            executable='motion_control_teach_node',
            name='motion_control_teach_node',
            output='screen',
            parameters=[{
                'robot_config_file': robot_config_file,
            }],
        ),
    ])
