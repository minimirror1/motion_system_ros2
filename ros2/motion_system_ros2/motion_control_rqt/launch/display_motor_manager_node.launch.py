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
    motor_config_file_arg = DeclareLaunchArgument(
        'motor_config_file',
        default_value=DEFAULT_MOTOR_CONFIG_FILE,
        description='Absolute path to motor_manager YAML (masters / drivers).',
    )
    jog_mode_arg = DeclareLaunchArgument(
        'jog_mode',
        default_value='false',
        description='Enable jog commands that use raw encoder targets.',
    )
    runtime_profile_arg = DeclareLaunchArgument(
        'runtime_profile',
        default_value='auto',
        description='Runtime profile: auto, desktop, or embedded.',
    )
    motor_status_publish_rate_hz_arg = DeclareLaunchArgument(
        'motor_status_publish_rate_hz',
        default_value='0',
        description='motor_status rate in Hz; 0 selects it from runtime_profile.',
    )

    motor_manager = Node(
        package='motion_control_bridge',
        executable='motor_manager_node',
        name='motor_manager_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('motor_config_file'),
            'jog_mode': ParameterValue(LaunchConfiguration('jog_mode'), value_type=bool),
            'runtime_profile': LaunchConfiguration('runtime_profile'),
            'motor_status_publish_rate_hz': ParameterValue(
                LaunchConfiguration('motor_status_publish_rate_hz'),
                value_type=int,
            ),
        }],
    )

    motion_control = Node(
        package='motion_control_rqt',
        executable='run_rqt',
        name='motion_control',
        output='screen',
        arguments=['--force-discover'],
        parameters=[{
            'config_file': LaunchConfiguration('motor_config_file'),
            'jog_mode': ParameterValue(LaunchConfiguration('jog_mode'), value_type=bool),
        }],
    )

    return LaunchDescription([
        motor_config_file_arg,
        jog_mode_arg,
        runtime_profile_arg,
        motor_status_publish_rate_hz_arg,
        motor_manager,
        motion_control,
    ])
