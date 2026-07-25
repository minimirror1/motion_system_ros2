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
DEFAULT_ROBOT_CONFIG_FILE = os.path.join(
    MOTION_SYSTEM_FILES_DIR,
    'robot_manager',
    'active_robot_manager.yaml',
)


def generate_launch_description():
    motor_config_file = LaunchConfiguration('motor_config_file')
    robot_config_file = LaunchConfiguration('robot_config_file')
    jog_mode = LaunchConfiguration('jog_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'motor_config_file',
            default_value=DEFAULT_MOTOR_CONFIG_FILE,
            description='Absolute path to motor_manager YAML.',
        ),
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=DEFAULT_ROBOT_CONFIG_FILE,
            description='Absolute path to robot_manager YAML.',
        ),
        DeclareLaunchArgument(
            'jog_mode',
            default_value='false',
            description='Enable jog commands that use raw encoder targets.',
        ),
        Node(
            package='motion_control_bridge',
            executable='motor_manager_node',
            name='motor_manager_node',
            output='screen',
            parameters=[{
                'config_file': motor_config_file,
                'jog_mode': ParameterValue(jog_mode, value_type=bool),
            }],
        ),
        Node(
            package='motion_control_robot',
            executable='robot_manager_node',
            name='robot_manager_node',
            output='screen',
            parameters=[{
                'config_file': robot_config_file,
            }],
        ),
        Node(
            package='motion_control_rqt',
            executable='run_rqt',
            name='motion_control',
            output='screen',
            arguments=['--force-discover'],
            parameters=[{
                'config_file': motor_config_file,
                'robot_config_file': robot_config_file,
                'jog_mode': ParameterValue(jog_mode, value_type=bool),
                'use_robot_manager_widget': True,
            }],
        ),
    ])
