import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


MOTION_SYSTEM_FILES_DIR = os.environ.get(
    'MOTION_SYSTEM_FILES_DIR',
    os.path.expanduser('~/colcon_ws/files'),
)
DEFAULT_MOTOR_CONFIG_FILE = os.path.join(
    MOTION_SYSTEM_FILES_DIR,
    'motor_manager',
    'active_motor_manager.yaml',
)


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=DEFAULT_MOTOR_CONFIG_FILE,
        description='Absolute path to motor_manager YAML (masters / drivers).',
    )
    jog_mode_arg = DeclareLaunchArgument(
        'jog_mode',
        default_value='false',
        description='Enable jog commands that use raw encoder targets.',
    )

    return LaunchDescription([
        config_file_arg,
        jog_mode_arg,
        Node(
            package='motion_control_bridge',
            executable='motor_manager_node',
            name='motor_manager_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'jog_mode': ParameterValue(LaunchConfiguration('jog_mode'), value_type=bool),
            }],
        ),
    ])
