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
DEFAULT_ROBOT_CONFIG_FILE = os.path.join(
    MOTION_SYSTEM_FILES_DIR,
    'robot_manager',
    'rocking_chair.yaml',
)
DEFAULT_MOTOR_CONFIG_FILE = os.path.join(
    MOTION_SYSTEM_FILES_DIR,
    'motor_manager',
    'active_motor_manager.yaml',
)


def generate_launch_description():
    robot_config_file_arg = DeclareLaunchArgument(
        'robot_config_file',
        default_value=DEFAULT_ROBOT_CONFIG_FILE,
        description='Absolute path to robot_manager YAML.',
    )
    motor_config_file_arg = DeclareLaunchArgument(
        'motor_config_file',
        default_value=DEFAULT_MOTOR_CONFIG_FILE,
        description='Absolute path to motor_manager YAML.',
    )
    hw_type_arg = DeclareLaunchArgument(
        'hw_type',
        default_value='DualSense',
        description='PlayStation controller hardware type used by p9n_node.',
    )
    jog_mode_arg = DeclareLaunchArgument(
        'jog_mode',
        default_value='false',
        description='Enable jog commands that use raw encoder targets.',
    )

    return LaunchDescription([
        robot_config_file_arg,
        motor_config_file_arg,
        hw_type_arg,
        jog_mode_arg,
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='p9n_node',
            executable='teleop_twist_joy_node_exec',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'hw_type': LaunchConfiguration('hw_type'),
            }],
        ),
        Node(
            package='motion_control_bridge',
            executable='motor_manager_node',
            name='motor_manager_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('motor_config_file'),
                'jog_mode': ParameterValue(LaunchConfiguration('jog_mode'), value_type=bool),
            }],
        ),
        Node(
            package='motion_control_robot',
            executable='robot_manager_node',
            name='robot_manager_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('robot_config_file'),
            }],
        ),
    ])
