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
    record_mode = LaunchConfiguration('record_mode')
    home_position_tolerance = LaunchConfiguration('home_position_tolerance')
    jog_mode = LaunchConfiguration('jog_mode')
    runtime_profile = LaunchConfiguration('runtime_profile')
    motor_status_publish_rate_hz = LaunchConfiguration('motor_status_publish_rate_hz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'motor_config_file',
            default_value=DEFAULT_MOTOR_CONFIG_FILE,
            description='Absolute path to motor_manager YAML.',
        ),
        DeclareLaunchArgument(
            'record_mode',
            default_value='false',
            description=(
                'Arm one recording session for dial1-selected channels, start on selected '
                'btn3+touch, and stop when any selected btn1 is disabled.'
            ),
        ),
        DeclareLaunchArgument(
            'home_position_tolerance',
            default_value='0.5',
            description='Position tolerance used to auto-reset btn2 at home.',
        ),
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=DEFAULT_ROBOT_CONFIG_FILE,
            description='Robot YAML containing home positions and motion_data_file_path.',
        ),
        DeclareLaunchArgument(
            'jog_mode',
            default_value='false',
            description='Enable jog commands that use raw encoder targets.',
        ),
        DeclareLaunchArgument(
            'runtime_profile',
            default_value='auto',
            description='Runtime profile: auto, desktop, or embedded.',
        ),
        DeclareLaunchArgument(
            'motor_status_publish_rate_hz',
            default_value='0',
            description='motor_status rate in Hz; 0 selects it from runtime_profile.',
        ),
        Node(
            package='motion_control_bridge',
            executable='motor_manager_node',
            name='motor_manager_node',
            output='screen',
            parameters=[{
                'config_file': motor_config_file,
                'jog_mode': ParameterValue(jog_mode, value_type=bool),
                'runtime_profile': runtime_profile,
                'motor_status_publish_rate_hz': ParameterValue(
                    motor_status_publish_rate_hz,
                    value_type=int,
                ),
            }],
        ),
        Node(
            package='xtouch_midi',
            executable='xtouch_node',
            name='xtouch_node',
            output='screen',
            parameters=[{
                'config_file': motor_config_file,
                'jog_mode': ParameterValue(jog_mode, value_type=bool),
                'btn0_requires_fader_update': True,
                'btn3_requires_fader_update': True,
            }],
        ),
        Node(
            package='motion_control_midi',
            executable='motion_control_midi_node',
            name='motion_control_midi_node',
            output='screen',
            parameters=[{
                'config_file': motor_config_file,
                'robot_config_file': robot_config_file,
                'jog_mode': ParameterValue(jog_mode, value_type=bool),
                'record_mode': ParameterValue(record_mode, value_type=bool),
                'home_position_tolerance': ParameterValue(
                    home_position_tolerance,
                    value_type=float,
                ),
            }],
        ),
    ])
