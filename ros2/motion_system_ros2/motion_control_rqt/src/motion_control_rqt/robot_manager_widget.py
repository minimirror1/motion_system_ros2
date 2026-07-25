import os
import yaml

import numpy as np
import pyqtgraph as pg

from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from sensor_msgs.msg import Joy

from motion_control_msgs.msg import RobotState
from motion_control_rqt.motor_manager_widget import (
    MOTION_SYSTEM_FILES_DIR,
    MotorManagerWidget,
)


JOY_BUTTON_MAX = 10

JOY_BUTTON_CROSS = 0
JOY_BUTTON_CIRCLE = 1
JOY_BUTTON_TRIANGLE = 2
JOY_BUTTON_SQUARE = 3
JOY_BUTTON_PREVIOUS = 4
JOY_BUTTON_NEXT = 5
JOY_BUTTON_START = 9

ROBOT_STATE_LABELS = {
    RobotState.STATE_HOMING: 'HOMING',
    RobotState.STATE_STOPPED: 'STOPPED',
    RobotState.STATE_OPERATING: 'OPERATING',
    RobotState.STATE_INVALID: 'INVALID',
}


class RobotManagerWidget(MotorManagerWidget):
    def __init__(self, node):
        self._robot_config_file = ''
        self._robot_motion_infos = {}
        self._robot_states = {}
        self._selected_robot_index = None
        self._motion_plot_robot_index = None
        self._motion_progress_line = None
        self._robot_info_label = None
        self._robot_state_label = None
        self._motion_progress_bar = None
        self._motion_plot_widget = None

        super().__init__(node)

        self._joy_publisher = self._node.create_publisher(Joy, 'joy', 10)
        self._load_robot_config_parameter()
        self._robot_motion_infos = self._load_robot_motion_infos()
        if self._robot_motion_infos:
            self._selected_robot_index = next(iter(self._robot_motion_infos))

        self._robot_state_subscriber = self._node.create_subscription(
            RobotState,
            'motion_control/robot_state',
            self._robot_state_callback,
            self._QOS_REKL5V,
        )
        self._add_robot_manager_tab()

    def _load_robot_config_parameter(self):
        if not self._node.has_parameter('robot_config_file'):
            self._node.declare_parameter('robot_config_file', '')

        robot_config_file = str(self._node.get_parameter('robot_config_file').value or '')
        if robot_config_file:
            self._robot_config_file = os.path.abspath(os.path.expanduser(robot_config_file))
        else:
            self._robot_config_file = os.path.join(
                MOTION_SYSTEM_FILES_DIR,
                'robot_manager',
                'active_robot_manager.yaml',
            )

    def _resolve_motion_data_path(self, path, config_dir, robot_name):
        package_scheme = 'package://'
        if path.startswith(package_scheme):
            package_path = path[len(package_scheme):]
            package_name, _, relative_path = package_path.partition('/')
            if not package_name or not relative_path:
                raise ValueError(f'Invalid package resource path: {path}')
            resolved_path = os.path.join(
                get_package_share_directory(package_name),
                relative_path,
            )
        elif os.path.isabs(path):
            resolved_path = os.path.expanduser(path)
        else:
            resolved_path = os.path.join(config_dir, os.path.expanduser(path))

        if os.path.splitext(resolved_path)[1].lower() != '.csv':
            resolved_path = os.path.join(resolved_path, f'{robot_name}.csv')
        return os.path.normpath(resolved_path)

    def _fallback_time_axis(self, sample_count, move_duration):
        if sample_count <= 0:
            return np.asarray([], dtype=float)
        if move_duration > 0.0:
            return np.linspace(0.0, move_duration, sample_count)
        return np.arange(sample_count, dtype=float)

    def _load_motion_csv(self, motion_data_file_path, controller_indices, move_duration):
        raw_motion_data = np.atleast_2d(
            np.loadtxt(motion_data_file_path, delimiter=',')
        )
        has_time_axis = (
            controller_indices and
            raw_motion_data.shape[0] >= len(controller_indices) + 1
        )

        if has_time_axis:
            time_axis = np.asarray(raw_motion_data[0], dtype=float).reshape(-1)
            motion_data = np.atleast_2d(raw_motion_data[1:])
        else:
            motion_data = raw_motion_data
            time_axis = self._fallback_time_axis(motion_data.shape[1], move_duration)

        if controller_indices:
            max_controller_index = max(controller_indices)
            if motion_data.shape[0] > max_controller_index:
                motion_data = motion_data[controller_indices]
            elif motion_data.shape[0] != len(controller_indices):
                motion_data = motion_data.reshape(len(controller_indices), -1)

        if time_axis.size != motion_data.shape[1]:
            time_axis = self._fallback_time_axis(motion_data.shape[1], move_duration)

        return time_axis, motion_data

    def _load_robot_motion_infos(self):
        if not self._robot_config_file:
            self._node.get_logger().warning('robot_config_file parameter is empty.')
            return {}
        if not os.path.exists(self._robot_config_file):
            self._node.get_logger().warning(
                f'Robot config file does not exist: {self._robot_config_file}'
            )
            return {}

        try:
            with open(self._robot_config_file, 'r', encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file) or {}
        except Exception as exc:
            self._node.get_logger().error(f'Failed to load robot config: {exc}')
            return {}

        config_dir = os.path.dirname(os.path.abspath(self._robot_config_file))
        robot_motion_infos = {}

        for robot in config.get('robot', []):
            if not isinstance(robot, dict):
                continue

            try:
                robot_index = int(robot['index'])
                robot_name = str(robot['name'])
                controller_indices = [
                    int(controller_index)
                    for controller_index in robot.get('controller_indices', [])
                ]
                move_duration = float(robot.get('move_duration', 0.0) or 0.0)
                motion_data_file_path = self._resolve_motion_data_path(
                    str(robot.get('motion_data_file_path', '')),
                    config_dir,
                    robot_name,
                )
                time_axis, motion_data = self._load_motion_csv(
                    motion_data_file_path,
                    controller_indices,
                    move_duration,
                )
            except Exception as exc:
                self._node.get_logger().error(
                    f"Failed to load motion data for robot {robot.get('index')}: {exc}"
                )
                continue

            robot_motion_infos[robot_index] = {
                'name': robot_name,
                'controller_indices': controller_indices,
                'move_duration': move_duration,
                'motion_data_file_path': motion_data_file_path,
                'motion_file_name': os.path.basename(motion_data_file_path),
                'time_axis': time_axis,
                'motion_data': motion_data,
            }

        return robot_motion_infos

    def _add_robot_manager_tab(self):
        if self.q_tab_widget is None:
            return

        robot_tab = QWidget()
        robot_tab_layout = QVBoxLayout(robot_tab)

        command_box = QGroupBox('Robot Manager', robot_tab)
        command_layout = QHBoxLayout(command_box)

        disable_button = QPushButton('Disable', command_box)
        enable_button = QPushButton('Enable', command_box)
        home_button = QPushButton('Home', command_box)
        stop_button = QPushButton('Stop', command_box)
        move_button = QPushButton('Move', command_box)
        previous_button = QPushButton('Previous', command_box)
        next_button = QPushButton('Next', command_box)

        disable_button.clicked.connect(
            lambda: self._publish_joy_button(JOY_BUTTON_CROSS)
        )
        enable_button.clicked.connect(
            lambda: self._publish_joy_button(JOY_BUTTON_START)
        )
        home_button.clicked.connect(
            lambda: self._publish_joy_button(JOY_BUTTON_TRIANGLE)
        )
        stop_button.clicked.connect(
            lambda: self._publish_joy_button(JOY_BUTTON_SQUARE)
        )
        move_button.clicked.connect(
            lambda: self._publish_joy_button(JOY_BUTTON_CIRCLE)
        )
        previous_button.clicked.connect(
            lambda: self._publish_joy_button(JOY_BUTTON_PREVIOUS)
        )
        next_button.clicked.connect(
            lambda: self._publish_joy_button(JOY_BUTTON_NEXT)
        )

        command_layout.addWidget(disable_button)
        command_layout.addWidget(enable_button)
        command_layout.addWidget(home_button)
        command_layout.addWidget(stop_button)
        command_layout.addWidget(move_button)
        command_layout.addWidget(previous_button)
        command_layout.addWidget(next_button)

        motion_box = QGroupBox('Current Motion Data', robot_tab)
        motion_layout = QVBoxLayout(motion_box)
        motion_info_layout = QHBoxLayout()

        self._robot_info_label = QLabel('Robot: -', motion_box)
        self._robot_state_label = QLabel('State: WAITING', motion_box)
        motion_info_layout.addWidget(self._robot_info_label)
        motion_info_layout.addStretch(1)
        motion_info_layout.addWidget(self._robot_state_label)

        self._motion_progress_bar = QProgressBar(motion_box)
        self._motion_progress_bar.setRange(0, 1000)
        self._motion_progress_bar.setValue(0)
        self._motion_progress_bar.setFormat('0.0%')

        self._motion_plot_widget = pg.PlotWidget(title='Motion data')
        self._motion_plot_widget.setBackground('w')
        self._motion_plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self._motion_plot_widget.setLabel('bottom', 'time', units='s')
        self._motion_plot_widget.setLabel('left', 'position')
        self._motion_plot_widget.setMinimumHeight(420)

        motion_layout.addLayout(motion_info_layout)
        motion_layout.addWidget(self._motion_progress_bar)
        motion_layout.addWidget(self._motion_plot_widget, 1)

        robot_tab_layout.addWidget(command_box)
        robot_tab_layout.addWidget(motion_box, 1)

        self.q_tab_widget.addTab(robot_tab, 'Robot Manager')
        self._update_robot_motion_view()

    def _publish_joy_button(self, button_index: int):
        self._publish_joy(button_index, pressed=True)
        QTimer.singleShot(100, lambda: self._publish_joy(button_index, pressed=False))

    def _publish_joy(self, button_index: int, pressed: bool):
        msg = Joy()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.axes = []
        msg.buttons = [0] * JOY_BUTTON_MAX
        if 0 <= button_index < JOY_BUTTON_MAX:
            msg.buttons[button_index] = 1 if pressed else 0
        self._joy_publisher.publish(msg)

    def _robot_state_callback(self, msg):
        self._selected_robot_index = int(msg.selected_robot_index)
        self._robot_states = {
            int(robot_index): (int(state), float(progress))
            for robot_index, state, progress in zip(
                msg.robot_index,
                msg.state,
                msg.progress,
            )
        }

    def _current_robot_index(self):
        if self._selected_robot_index is not None:
            return self._selected_robot_index
        if self._robot_motion_infos:
            return next(iter(self._robot_motion_infos))
        return None

    def _plot_current_motion_data(self, robot_index):
        if self._motion_plot_widget is None:
            return
        if robot_index == self._motion_plot_robot_index:
            return

        self._motion_plot_widget.clear()
        self._motion_progress_line = None
        self._motion_plot_robot_index = robot_index

        motion_info = self._robot_motion_infos.get(robot_index)
        if motion_info is None:
            self._motion_plot_widget.setTitle('Motion data unavailable')
            return

        time_axis = motion_info['time_axis']
        motion_data = motion_info['motion_data']
        controller_indices = motion_info['controller_indices']
        if time_axis.size == 0 or motion_data.size == 0:
            self._motion_plot_widget.setTitle('Motion data unavailable')
            return

        self._motion_plot_widget.setTitle(
            f"Motion data: {motion_info['motion_file_name']}"
        )
        self._motion_plot_widget.addLegend(offset=(10, 10))

        for row_index, values in enumerate(motion_data):
            controller_index = (
                controller_indices[row_index]
                if row_index < len(controller_indices)
                else row_index
            )
            self._motion_plot_widget.plot(
                time_axis,
                values,
                pen=pg.mkPen(pg.intColor(row_index, hues=max(3, len(motion_data))), width=2),
                name=f'controller {controller_index}',
            )

        self._motion_progress_line = pg.InfiniteLine(
            pos=float(time_axis[0]),
            angle=90,
            movable=False,
            pen=pg.mkPen(color=(220, 70, 50), width=2),
        )
        self._motion_plot_widget.addItem(self._motion_progress_line)

    def _progress_time_text(self, motion_info, progress):
        if motion_info is None:
            return ''

        time_axis = motion_info['time_axis']
        if time_axis.size == 0:
            return ''

        start_time = float(time_axis[0])
        end_time = float(time_axis[-1])
        current_time = start_time + progress * (end_time - start_time)
        return f't={current_time:.2f}s / {end_time:.2f}s'

    def _update_motion_progress_line(self, motion_info, progress):
        if self._motion_progress_line is None or motion_info is None:
            return

        time_axis = motion_info['time_axis']
        if time_axis.size == 0:
            return

        start_time = float(time_axis[0])
        end_time = float(time_axis[-1])
        self._motion_progress_line.setPos(
            start_time + progress * (end_time - start_time)
        )

    def _update_robot_motion_view(self):
        if self._motion_progress_bar is None:
            return

        robot_index = self._current_robot_index()
        if robot_index is None:
            self._robot_info_label.setText('Robot: -')
            self._robot_state_label.setText('State: WAITING')
            self._motion_progress_bar.setValue(0)
            self._motion_progress_bar.setFormat('0.0%')
            return

        self._plot_current_motion_data(robot_index)

        motion_info = self._robot_motion_infos.get(robot_index)
        if motion_info is None:
            self._robot_info_label.setText(f'Robot {robot_index} | motion data unavailable')
        else:
            self._robot_info_label.setText(
                f"Robot {robot_index} | {motion_info['name']} | {motion_info['motion_file_name']}"
            )

        if robot_index in self._robot_states:
            state_value, progress = self._robot_states[robot_index]
            state_label = ROBOT_STATE_LABELS.get(state_value, f'UNKNOWN({state_value})')
        else:
            progress = 0.0
            state_label = 'WAITING'

        progress = float(np.clip(progress, 0.0, 1.0))
        progress_text = self._progress_time_text(motion_info, progress)
        state_text = f'State: {state_label}'
        if progress_text:
            state_text = f'{state_text} | {progress_text}'

        self._robot_state_label.setText(state_text)
        self._motion_progress_bar.setValue(int(round(progress * 1000.0)))
        self._motion_progress_bar.setFormat(f'{progress * 100.0:.1f}%')
        self._update_motion_progress_line(motion_info, progress)

    def _on_update_timer(self):
        super()._on_update_timer()
        self._update_robot_motion_view()
