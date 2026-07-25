import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Joy
from std_msgs.msg import Int8MultiArray
from std_srvs.srv import Trigger
from motion_control_msgs.msg import MotorStatus, RobotState

from common_robot_interface.joint_frame import joint_frame_t
from common_robot_interface.state_frame import State
from common_robot_interface.action_frame import Action, action_frame_t

from robot_manager.robot_manager import RobotManager

JOY_BUTTON_MAX = 10

JOY_BUTTON_CROSS = 0
JOY_BUTTON_CIRCLE = 1
JOY_BUTTON_TRIANGLE = 2
JOY_BUTTON_SQUARE = 3

JOY_BUTTON_PREVIOUS = 4
JOY_BUTTON_NEXT = 5
JOY_BUTTON_START = 9


class RobotManagerNode(Node):
    QOS_BEKL1V = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    def __init__(self):
        super().__init__('robot_manager_node')

        self.config_file = (
            self.declare_parameter(
                'config_file',
                '',
            )
            .get_parameter_value()
            .string_value
        )
        if not self.config_file:
            raise RuntimeError('config_file parameter must be set.')
        self.robot_manager = RobotManager(self.config_file)
        self.number_of_robots = self.robot_manager.number_of_robots
        self.dt = self.robot_manager.dt
        self.robot_indices = self.robot_manager.robot_indices()
        if not self.robot_indices:
            raise RuntimeError('Robot configuration requires at least one robot.')
        self.robot_action_indices = {
            robot_index: i
            for i, robot_index in enumerate(self.robot_indices)
        }

        self.request_publisher = self.create_publisher(
            Int8MultiArray,
            'motion_control/request',
            self.QOS_BEKL1V,
        )
        self.motor_status_subscriber = self.create_subscription(
            MotorStatus,
            'motion_control/motor_status',
            self.motor_status_callback,
            self.QOS_BEKL1V,
        )
        self.motor_command_publisher = self.create_publisher(
            MotorStatus,
            'motion_control/motor_command',
            self.QOS_BEKL1V,
        )
        self.robot_state_publisher = self.create_publisher(
            RobotState,
            'motion_control/robot_state',
            self.QOS_BEKL1V,
        )

        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10,
        )
        self.timer = self.create_timer(
            self.dt,
            self.timer_callback,
        )
        self.reload_service = self.create_service(
            Trigger,
            '~/reload_config',
            self.reload_config_callback,
        )

        self.joy_buttons: list[bool] = [False] * JOY_BUTTON_MAX
        self.joy_buttons_prev: list[bool] = [False] * JOY_BUTTON_MAX
        self.joy_button_action: dict[int, Action] = {
            JOY_BUTTON_TRIANGLE: Action.HOME,
            JOY_BUTTON_CIRCLE: Action.MOVE,
            JOY_BUTTON_SQUARE: Action.STOP,
        }

        self.is_valid_joint_status: bool = False
        self._last_invalid_status_warning_ns = 0

        self.selected_robot_index: int = self.robot_indices[0]
        self.robot_actions: list[action_frame_t] = [
            action_frame_t(robot_index=robot_index, action=Action.STOP)
            for robot_index in self.robot_indices
        ]

    def motor_status_callback(self, msg: MotorStatus):
        field_lengths = {
            'controller_index': len(msg.controller_index),
            'controlword': len(msg.controlword),
            'statusword': len(msg.statusword),
            'position': len(msg.position),
            'velocity': len(msg.velocity),
            'effort': len(msg.effort),
        }
        expected_size = field_lengths['controller_index']
        if expected_size == 0 or any(
            size != expected_size for size in field_lengths.values()
        ):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_invalid_status_warning_ns >= 5_000_000_000:
                lengths = ', '.join(
                    f'{name}={size}' for name, size in field_lengths.items()
                )
                self.get_logger().warning(
                    f'Ignoring malformed motor status with inconsistent array lengths: {lengths}'
                )
                self._last_invalid_status_warning_ns = now_ns
            return

        joint_status = joint_frame_t(
            controller_index=np.asarray(msg.controller_index, dtype=np.uint8),
            controlword=np.asarray(msg.controlword, dtype=np.uint16),
            position=np.asarray(msg.position, dtype=np.float64),
            velocity=np.asarray(msg.velocity, dtype=np.float64),
            effort=np.asarray(msg.effort, dtype=np.float64),
        )
        self.robot_manager.updateJointStatus(joint_status)

        statuswords = np.asarray(msg.statusword, dtype=np.uint16)
        if np.all(statuswords != 0):
            self.is_valid_joint_status = True

    def publish_motor_command(self, commands: joint_frame_t):
        msg = MotorStatus()
        msg.number_of_target_interfaces = [
            int(count)
            for count in self.robot_manager.number_of_target_interfaces()
        ]
        msg.target_interface_id = [
            Int8MultiArray(data=[int(interface_id) for interface_id in target_interface_ids])
            for target_interface_ids in self.robot_manager.target_interface_ids()
        ]
        msg.controller_index = [int(index) for index in commands.controller_index]
        msg.controlword = [int(controlword) for controlword in commands.controlword]
        msg.encoder = [0] * len(msg.controller_index)
        msg.position = [float(position) for position in commands.position]
        msg.velocity = [float(velocity) for velocity in commands.velocity]
        msg.effort = [float(effort) for effort in commands.effort]
        self.motor_command_publisher.publish(msg)

    def publish_robot_state(self):
        msg = RobotState()
        msg.selected_robot_index = int(self.selected_robot_index)
        for state_frame in self.robot_manager.get_state_frames():
            msg.robot_index.append(int(state_frame.robot_index))
            msg.state.append(int(state_frame.state.value))
            msg.progress.append(float(state_frame.progress))
        self.robot_state_publisher.publish(msg)

    def joy_callback(self, msg: Joy):
        for btn in range(JOY_BUTTON_MAX):
            self.joy_buttons[btn] = btn < len(msg.buttons) and bool(msg.buttons[btn])

    def selected_robot_action_index(self) -> int:
        return self.robot_action_indices[self.selected_robot_index]

    def publish_controller_request(self, request_value: int):
        controller_indices = self.robot_manager.controller_indices()
        if not controller_indices:
            return

        request = Int8MultiArray()
        request.data = [2] * (max(controller_indices) + 1)
        for controller_index in controller_indices:
            request.data[controller_index] = request_value
        self.request_publisher.publish(request)

    def reload_config_callback(self, request, response):
        # Relies on the single-threaded executor serializing this with
        # timer/subscription callbacks; add a lock before switching executors.
        if any(
            state_frame.state != State.STOPPED
            for state_frame in self.robot_manager.get_state_frames()
        ) or any(
            robot_action.action != Action.STOP
            for robot_action in self.robot_actions
        ):
            response.success = False
            response.message = 'Robots must be stopped before reloading configuration.'
            return response

        try:
            new_manager = RobotManager(self.config_file)
        except Exception as e:  # noqa: BLE001 - keep the old manager on any failure
            response.success = False
            response.message = f'Reload failed: {e}'
            return response

        self.robot_manager = new_manager
        self.number_of_robots = new_manager.number_of_robots
        self.robot_indices = new_manager.robot_indices()
        self.robot_action_indices = {
            robot_index: i
            for i, robot_index in enumerate(self.robot_indices)
        }
        if self.selected_robot_index not in self.robot_indices:
            self.selected_robot_index = self.robot_indices[0]
        self.robot_actions = [
            action_frame_t(robot_index=robot_index, action=Action.STOP)
            for robot_index in self.robot_indices
        ]
        if new_manager.dt != self.dt:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.dt = new_manager.dt
            self.timer = self.create_timer(self.dt, self.timer_callback)
        # The new manager has never seen a joint status; the motor_status
        # stream re-latches this within one message.
        self.is_valid_joint_status = False

        response.success = True
        response.message = (
            f'Config reloaded: {self.number_of_robots} robot(s), dt={self.dt}.'
        )
        self.get_logger().info(response.message)
        return response

    def timer_callback(self):
        if not self.is_valid_joint_status:
            self.publish_robot_state()
            return

        should_publish_motor_command = False

        if self.joy_buttons[JOY_BUTTON_CROSS] and not self.joy_buttons_prev[JOY_BUTTON_CROSS]:
            self.publish_controller_request(0)
        elif self.joy_buttons[JOY_BUTTON_START] and not self.joy_buttons_prev[JOY_BUTTON_START]:
            self.publish_controller_request(1)

        # Check if the robot is stopped because of the homing is completed
        state_frames = self.robot_manager.get_state_frames()

        for state_frame in state_frames:
            robot_action_index = self.robot_action_indices[state_frame.robot_index]
            if state_frame.state == State.STOPPED and self.robot_actions[robot_action_index].action != Action.STOP:
                self.robot_actions[robot_action_index].action = Action.STOP

        # Select the robot by the DPAD
        if self.joy_buttons[JOY_BUTTON_PREVIOUS] and not self.joy_buttons_prev[JOY_BUTTON_PREVIOUS]:
            selected_action_index = self.selected_robot_action_index()
            selected_action_index = selected_action_index - 1 if selected_action_index > 0 else self.number_of_robots - 1
            self.selected_robot_index = self.robot_indices[selected_action_index]
        elif self.joy_buttons[JOY_BUTTON_NEXT] and not self.joy_buttons_prev[JOY_BUTTON_NEXT]:
            selected_action_index = (self.selected_robot_action_index() + 1) % self.number_of_robots
            self.selected_robot_index = self.robot_indices[selected_action_index]

        # Check if action button is pressed
        for btn, action in self.joy_button_action.items():
            if self.joy_buttons[btn] and not self.joy_buttons_prev[btn]:
                self.robot_actions[self.selected_robot_action_index()].action = action
                should_publish_motor_command = True
                break

        should_publish_motor_command = (
            should_publish_motor_command or
            any(robot_action.action != Action.STOP for robot_action in self.robot_actions)
        )

        # Send the action to the robot
        if should_publish_motor_command:
            commands: joint_frame_t = self.robot_manager.set_action_frames(self.robot_actions)
            self.publish_motor_command(commands)

        self.publish_robot_state()
        self.joy_buttons_prev = self.joy_buttons.copy()


def main(args=None):
    rclpy.init(args=args)
    node = RobotManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
