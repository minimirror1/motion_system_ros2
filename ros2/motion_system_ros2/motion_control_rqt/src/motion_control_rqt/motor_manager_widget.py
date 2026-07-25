import os
import re
import yaml
import pyqtgraph as pg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QMainWindow,
    QWidget,
    QTabWidget,
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QMenu,
    QAction,
    QStyle,
    QRadioButton,
    QToolButton,
    QPlainTextEdit,
    QSlider,
    QPushButton,
)

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from motion_control_msgs.msg import MotorStatus
from std_msgs.msg import Int8MultiArray
import rclpy


ID_CONTROLWORD = 0
ID_TARGET_POSITION = 1
ID_TARGET_VELOCITY = 2
ID_TARGET_EFFORT = 3

REQUEST_DISABLE = 0
REQUEST_ENABLE = 1
REQUEST_NONE = 2

CW_NEW_SET_POINT_ZEROERR = 0x103F
CW_NEW_SET_POINT_MINAS = 0x003F
CW_SOCKETCAN_SET_POINT = 0x0001

MOTION_SYSTEM_FILES_DIR = os.environ.get(
    'MOTION_SYSTEM_FILES_DIR',
    os.path.expanduser('~/colcon_ws/files'),
)


class MotorManagerWidget(QMainWindow):
    _QOS_REKL5V = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    def __init__(self, node):
        super().__init__()

        ui_file_path = os.path.join(
            get_package_share_directory('motion_control_rqt'),
            'resource',
            'base.ui',
        )
        
        try:
            loadUi(ui_file_path, self)
        except Exception as e:
            rclpy.logging.get_logger('MotorManagerWidget').error(f'Failed to load UI file: {e}')
            raise

        self.resize(1500, 900)
        self._node = node

        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._on_update_timer)
        self._update_timer.start(10)

        self._current_controller_index = None
        
        self._node.declare_parameter('config_file', '')
        self._node.declare_parameter('jog_mode', False)
        self._config_file = self._resolve_config_file(
            str(self._node.get_parameter('config_file').value)
        )
        self._jog_mode = bool(self._node.get_parameter('jog_mode').value)
        self._master_infos, self._motor_infos = self._load_motor_infos()
        self._motor_info_by_controller_index = {
            int(motor_info['controller_index']): motor_info
            for motor_info in self._motor_infos
            if motor_info.get('controller_index') is not None
        }

        self._motor_status = None
        self._is_visible = False
        self._positions = []
        self._velocities = []
        self._efforts = []
        self._jog_current_encoder_label = None
        self._enable_motor_button = None
        self._disable_motor_button = None

        self._motor_command_publisher = self._node.create_publisher(
            MotorStatus,
            'motion_control/motor_command',
            self._QOS_REKL5V,
        )
        self._request_publisher = self._node.create_publisher(
            Int8MultiArray,
            'motion_control/request',
            self._QOS_REKL5V,
        )

        self._initialize_widget()

        self._motor_status_subscriber = self._node.create_subscription(
            MotorStatus,
            'motion_control/motor_status',
            self.motor_status_callback,
            self._QOS_REKL5V,
        )


    def _resolve_config_file(self, config_file):
        file_name = os.path.basename(config_file) if config_file else 'active_motor_manager.yaml'
        files_config = os.path.join(MOTION_SYSTEM_FILES_DIR, 'motor_manager', file_name)
        if config_file and os.path.abspath(config_file) != os.path.abspath(files_config):
            self._node.get_logger().info(
                f"Using writable motor config from files folder: {files_config}"
            )

        return files_config


    def _load_motor_infos(self):
        if not self._config_file:
            self._node.get_logger().warning('config_file parameter is empty.')
            return [], []

        try:
            with open(self._config_file, 'r', encoding='utf-8') as yaml_file:
                config = yaml.safe_load(yaml_file) or {}
        except Exception as exc:
            self._node.get_logger().error(f'Failed to load motor config: {exc}')
            return [], []

        drivers = {
            driver['id']: driver
            for driver in config.get('drivers', [])
            if isinstance(driver, dict) and 'id' in driver
        }

        motor_infos = []
        master_infos = []
        for master in config.get('masters', []):
            if not isinstance(master, dict):
                continue

            master_infos.append({
                'id': master.get('id'),
                'type': master.get('type'),
                'number_of_slaves': master.get('number_of_slaves'),
                'master_index': master.get('master_index'),
            })

            for slave in master.get('slaves', []):
                if not isinstance(slave, dict):
                    continue

                driver_id = slave.get('driver_id')
                driver_info = drivers.get(driver_id, {})
                motor_infos.append({
                    'controller_index': slave.get('controller_index'),
                    'profile_mode': slave.get('profile_mode'),
                    'driver_id': driver_id,
                    'alias': slave.get('alias'),
                    'lower': driver_info.get('lower'),
                    'upper': driver_info.get('upper'),
                    'speed': driver_info.get('speed'),
                    'rated_effort': driver_info.get('rated_effort'),
                    'type': driver_info.get('type'),
                })

        return master_infos, motor_infos

    def _initialize_widget(self):
        self.q_tab_widget = self.findChild(QTabWidget, 'TabWidget')
        if self.q_tab_widget is None:
            return

        style = self.style()
        refresh_icon = style.standardIcon(style.SP_BrowserReload)

        first_tab = QWidget()
        first_tab_layout = QVBoxLayout(first_tab)
        top_control_layout = QHBoxLayout()

        reset_button = QPushButton()
        reset_button.setMinimumSize(44, 34)
        reset_button.setIcon(refresh_icon)
        reset_button.clicked.connect(self._on_reset_button_clicked)

        self._select_motor_button = QToolButton()
        self._select_motor_button.setMinimumSize(320, 34)
        self._select_motor_button.setText("Select a Motor ...")
        self._select_motor_button.setPopupMode(QToolButton.MenuButtonPopup)
        self._add_select_motor_menu()

        self._enable_motor_button = QPushButton("Enable")
        self._enable_motor_button.setMinimumSize(104, 34)
        self._enable_motor_button.setEnabled(False)
        self._enable_motor_button.clicked.connect(self._on_enable_motor_clicked)

        self._disable_motor_button = QPushButton("Disable")
        self._disable_motor_button.setMinimumSize(104, 34)
        self._disable_motor_button.setEnabled(False)
        self._disable_motor_button.clicked.connect(self._on_disable_motor_clicked)

        top_control_layout.addWidget(self._select_motor_button, 0, Qt.AlignLeft)
        top_control_layout.addWidget(self._enable_motor_button, 0, Qt.AlignLeft)
        top_control_layout.addWidget(self._disable_motor_button, 0, Qt.AlignLeft)
        top_control_layout.addStretch()
        top_control_layout.addWidget(reset_button, 0, Qt.AlignRight)

        status_monitor = QGroupBox("Status Monitor", first_tab)
        status_monitor_layout = QVBoxLayout(status_monitor)

        visible_button = QRadioButton("Visible")
        visible_button.setChecked(False)
        visible_button.clicked.connect(self._on_visible_button_clicked)

        self._motor_infos_plot_widget = pg.PlotWidget(title="Motor infos")
        self._motor_infos_plot_widget.setBackground('w')
        self._motor_infos_plot_widget.setMinimumHeight(360)

        status_monitor_layout.addWidget(visible_button, 0, Qt.AlignRight)
        status_monitor_layout.addWidget(self._motor_infos_plot_widget)

        command_console = QGroupBox("Command Console", first_tab)
        command_console_layout = QVBoxLayout(command_console)

        command_slider = QGroupBox("Slider", command_console)
        command_slider_layout = QHBoxLayout(command_slider)

        self._cur_val_label = QLabel("Current Value: ")
        self._cur_val_label.setFixedWidth(330)

        self._command_slider = QSlider(Qt.Horizontal)
        self._command_slider.valueChanged.connect(self._on_slider_value_changed)
        self._command_slider.setEnabled(not self._jog_mode)

        self._max_value_label = QLabel()
        self._max_value_label.setFixedWidth(80)

        command_slider_layout.addWidget(self._cur_val_label)
        command_slider_layout.addWidget(self._command_slider)
        command_slider_layout.addWidget(self._max_value_label)

        command_console_layout.addWidget(command_slider)

        jog_console = QGroupBox("Jog Console", first_tab)
        jog_console.setEnabled(self._jog_mode)
        jog_console_layout = QVBoxLayout(jog_console)

        jog_layout = QHBoxLayout()
        forward_button = QPushButton("Forward")
        forward_button.clicked.connect(self._on_jog_forward_clicked)

        self._jog_encoder_text = QPlainTextEdit()
        self._jog_encoder_text.setFixedHeight(34)
        self._jog_encoder_text.setPlainText("0")

        backward_button = QPushButton("Backward")
        backward_button.clicked.connect(self._on_jog_backward_clicked)

        jog_layout.addWidget(forward_button)
        jog_layout.addWidget(self._jog_encoder_text)
        jog_layout.addWidget(backward_button)
        jog_console_layout.addLayout(jog_layout)

        zero_offset_layout = QHBoxLayout()
        self._jog_zero_offset_degree_text = QPlainTextEdit()
        self._jog_zero_offset_degree_text.setFixedHeight(34)
        self._jog_zero_offset_degree_text.setPlainText("0")

        zero_offset_reset_button = QPushButton("Reset")
        zero_offset_reset_button.clicked.connect(self._on_jog_zero_offset_reset_clicked)

        zero_offset_layout.addWidget(self._jog_zero_offset_degree_text)
        zero_offset_layout.addWidget(zero_offset_reset_button)
        jog_console_layout.addLayout(zero_offset_layout)

        self._jog_current_encoder_label = QLabel("current encoder: ")
        jog_console_layout.addWidget(self._jog_current_encoder_label)

        first_tab_layout.addLayout(top_control_layout)
        first_tab_layout.addWidget(status_monitor, 1)
        first_tab_layout.addWidget(command_console)
        first_tab_layout.addWidget(jog_console)

        self.q_tab_widget.addTab(first_tab, "Motor Manager")

    def _add_select_motor_menu(self):
        menu = QMenu(self._select_motor_button)
        for motor_info in self._motor_infos:
            action = QAction(str(motor_info["controller_index"]), self._select_motor_button)
            action.triggered.connect(lambda _, index=motor_info["controller_index"]: self._on_select_motor_clicked(index))
            menu.addAction(action)
        self._select_motor_button.setMenu(menu)

    def _initialize_motor_status_msg(self):
        n_slaves = 0
        for master in self._master_infos:
            n_slaves += master['number_of_slaves']

        max_controller_index = max(
            self._motor_info_by_controller_index.keys(),
            default=-1,
        )
        n_controllers = max(n_slaves, max_controller_index + 1)

        msg = MotorStatus()
        msg.number_of_target_interfaces = [0] * n_controllers
        msg.target_interface_id = [Int8MultiArray() for _ in range(n_controllers)]
        msg.controller_index = [i for i in range(n_controllers)]
        msg.controlword = [0] * n_controllers
        msg.statusword = [0] * n_controllers
        msg.errorcode = [0] * n_controllers
        msg.encoder = [0] * n_controllers
        msg.position = [0.0] * n_controllers
        msg.velocity = [0.0] * n_controllers
        msg.effort = [0.0] * n_controllers
        return msg, n_controllers

    def _set_current_value_label(self, profile_mode, value):
        if profile_mode == 0:
            if value < 0:
                self._cur_val_label.setText(f"Current Value:  -{int(-value // 100)}.{int(-value % 100)}")
            else:
                self._cur_val_label.setText(f"Current Value:  {int(value // 100)}.{int(value % 100)}")
        elif profile_mode == 1:
            self._cur_val_label.setText(f"Current Value:  {int(value)} deg/s")
        elif profile_mode == 2:
            self._cur_val_label.setText(f"Current Value:  {int(value)} Nm")


    def _driver_type(self, motor_info):
        return str(motor_info.get('type') or '').lower()

    def _controlword_for_driver(self, driver_type):
        if driver_type == 'zeroerr':
            return int(CW_NEW_SET_POINT_ZEROERR)
        if driver_type == 'minas':
            return int(CW_NEW_SET_POINT_MINAS)
        if driver_type == 'cubemars':
            return int(CW_SOCKETCAN_SET_POINT)

        self._node.get_logger().warning(
            f"Unknown driver type '{driver_type}'; using zeroerr set-point controlword."
        )
        return int(CW_NEW_SET_POINT_ZEROERR)

    def _set_position_target_interfaces(self, msg, motor_info, index):
        driver_type = self._driver_type(motor_info)
        if driver_type == 'dynamixel':
            msg.number_of_target_interfaces[index] = 1
            msg.target_interface_id[index] = Int8MultiArray(data=[ID_TARGET_POSITION])
            return

        msg.number_of_target_interfaces[index] = 2
        msg.target_interface_id[index] = Int8MultiArray(
            data=[ID_CONTROLWORD, ID_TARGET_POSITION]
        )
        msg.controlword[index] = self._controlword_for_driver(driver_type)

    def _set_position_command(self, msg, motor_info, index, value):
        self._set_position_target_interfaces(msg, motor_info, index)
        msg.position[index] = value / 100.0

    def _set_encoder_command(self, msg, motor_info, index, encoder):
        self._set_position_target_interfaces(msg, motor_info, index)
        msg.encoder[index] = int(encoder)

    def _selected_motor_info(self):
        if self._current_controller_index is None:
            return None
        return self._motor_info_by_controller_index.get(self._current_controller_index)

    def _update_motor_request_buttons(self):
        enabled = self._current_controller_index is not None
        if self._enable_motor_button is not None:
            self._enable_motor_button.setEnabled(enabled)
        if self._disable_motor_button is not None:
            self._disable_motor_button.setEnabled(enabled)

    def _publish_motor_request(self, request_value):
        if self._current_controller_index is None:
            return

        controller_index = int(self._current_controller_index)
        max_controller_index = max(
            self._motor_info_by_controller_index.keys(),
            default=controller_index,
        )
        if self._motor_status is not None and self._motor_status.controller_index:
            max_controller_index = max(
                max_controller_index,
                max(int(index) for index in self._motor_status.controller_index),
            )

        request = Int8MultiArray()
        request.data = [REQUEST_NONE] * (max_controller_index + 1)
        request.data[controller_index] = int(request_value)
        self._request_publisher.publish(request)

    def _on_enable_motor_clicked(self):
        self._publish_motor_request(REQUEST_ENABLE)

    def _on_disable_motor_clicked(self):
        self._publish_motor_request(REQUEST_DISABLE)

    def _driver_config(self, config, motor_info):
        driver_id = motor_info.get('driver_id')
        for driver in config.get('drivers', []):
            if isinstance(driver, dict) and driver.get('id') == driver_id:
                return driver
        return None

    def _write_driver_zero_offset(self, driver_id, zero_offset):
        with open(self._config_file, 'r', encoding='utf-8') as config_file:
            lines = config_file.readlines()

        drivers_indent = None
        in_drivers = False
        in_target_driver = False
        insert_after = None
        insert_indent = None

        for i, line in enumerate(lines):
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue

            indent = len(line) - len(line.lstrip(' '))

            if not in_drivers:
                if stripped == 'drivers:':
                    in_drivers = True
                    drivers_indent = indent
                continue

            if indent <= drivers_indent and not stripped.startswith('-'):
                break

            id_match = re.match(r'^(\s*)-\s+id:\s*(.*?)\s*(?:#.*)?$', line)
            if id_match:
                parsed_id = yaml.safe_load(id_match.group(2))
                in_target_driver = parsed_id == driver_id
                insert_after = None
                insert_indent = None
                continue

            if not in_target_driver:
                continue

            pulse_match = re.match(r'^(\s*)pulse_per_revolution:', line)
            if pulse_match:
                insert_after = i
                insert_indent = pulse_match.group(1)

            zero_match = re.match(r'^(\s*)zero_offset:\s*.*$', line)
            if zero_match:
                lines[i] = f"{zero_match.group(1)}zero_offset: {zero_offset}\n"
                with open(self._config_file, 'w', encoding='utf-8') as config_file:
                    config_file.writelines(lines)
                return True

        if insert_after is not None and insert_indent is not None:
            lines.insert(insert_after + 1, f"{insert_indent}zero_offset: {zero_offset}\n")
            with open(self._config_file, 'w', encoding='utf-8') as config_file:
                config_file.writelines(lines)
            return True

        return False

    def _update_jog_current_encoder_label(self):
        if self._jog_current_encoder_label is None:
            return

        if self._current_controller_index is None or self._motor_status is None:
            self._jog_current_encoder_label.setText("current encoder: ")
            return

        controller_index = self._current_controller_index
        if controller_index >= len(self._motor_status.encoder):
            self._jog_current_encoder_label.setText("current encoder: ")
            return

        self._jog_current_encoder_label.setText(
            f"current encoder: {int(self._motor_status.encoder[controller_index])}"
        )

    def _on_reset_button_clicked(self):
        self._current_controller_index = None
        self._is_visible = False
        self._positions = []
        self._velocities = []
        self._efforts = []
        self._add_select_motor_menu()
        self._update_motor_request_buttons()
        self._update_jog_current_encoder_label()

    def _on_visible_button_clicked(self):
        self._is_visible = False if self._is_visible else True

    def _on_slider_value_changed(self, value):
        if self._jog_mode:
            return

        if self._current_controller_index is None:
            return

        msg, _ = self._initialize_motor_status_msg()

        controller_index = self._current_controller_index
        motor_info = self._motor_info_by_controller_index.get(controller_index)
        if motor_info is None or controller_index >= len(msg.controller_index):
            return

        if motor_info['profile_mode'] == 0:
            self._set_position_command(
                msg, motor_info, controller_index, value
            )

        elif motor_info['profile_mode'] == 1:
            msg.number_of_target_interfaces[controller_index] = 1
            msg.target_interface_id[controller_index] = Int8MultiArray(
                data=[ID_TARGET_VELOCITY]
            )
            msg.velocity[controller_index] = value

        elif motor_info['profile_mode'] == 2:
            msg.number_of_target_interfaces[controller_index] = 1
            msg.target_interface_id[controller_index] = Int8MultiArray(
                data=[ID_TARGET_EFFORT]
            )
            msg.effort[controller_index] = value
            
        self._motor_command_publisher.publish(msg)

        self._set_current_value_label(motor_info['profile_mode'], value)

    def _set_slider_value_without_command(self, value):
        previous = self._command_slider.blockSignals(True)
        try:
            self._command_slider.setValue(value)
        finally:
            self._command_slider.blockSignals(previous)

    def _set_slider_state_without_command(self, lower, upper, value):
        previous = self._command_slider.blockSignals(True)
        try:
            self._command_slider.setRange(lower, upper)
            self._command_slider.setValue(value)
        finally:
            self._command_slider.blockSignals(previous)

    def _motor_status_array_index(self, controller_index):
        if self._motor_status is None:
            return None

        try:
            return list(self._motor_status.controller_index).index(controller_index)
        except ValueError:
            return None

    def _sync_position_slider_from_motor_status(self):
        if self._jog_mode or self._command_slider.isSliderDown():
            return
        if self._current_controller_index is None or self._motor_status is None:
            return

        motor_info = self._selected_motor_info()
        if motor_info is None or motor_info.get('profile_mode') != 0:
            return

        status_index = self._motor_status_array_index(self._current_controller_index)
        if status_index is None or status_index >= len(self._motor_status.position):
            return

        current_value = int(round(self._motor_status.position[status_index] * 100.0))
        current_value = max(
            self._command_slider.minimum(),
            min(self._command_slider.maximum(), current_value),
        )
        if self._command_slider.value() != current_value:
            self._set_slider_value_without_command(current_value)
        self._set_current_value_label(0, current_value)

    def _on_jog_forward_clicked(self):
        self._publish_jog_command(1)

    def _on_jog_backward_clicked(self):
        self._publish_jog_command(-1)

    def _publish_jog_command(self, direction):
        if not self._jog_mode:
            return
        if self._current_controller_index is None or self._motor_status is None:
            return

        controller_index = self._current_controller_index
        motor_info = self._motor_info_by_controller_index.get(controller_index)
        if motor_info is None or motor_info.get('profile_mode') != 0:
            return
        if controller_index >= len(self._motor_status.encoder):
            return

        text = self._jog_encoder_text.toPlainText().strip()
        try:
            encoder_delta = abs(int(text, 0))
        except ValueError:
            self._node.get_logger().warning(f"Invalid jog encoder value: '{text}'")
            return

        msg, _ = self._initialize_motor_status_msg()
        if controller_index >= len(msg.controller_index):
            return

        current_encoder = int(self._motor_status.encoder[controller_index])
        target_encoder = current_encoder + direction * encoder_delta
        self._set_encoder_command(msg, motor_info, controller_index, target_encoder)
        self._motor_command_publisher.publish(msg)

    def _on_jog_zero_offset_reset_clicked(self):
        if not self._jog_mode:
            return
        if self._current_controller_index is None or self._motor_status is None:
            return

        controller_index = self._current_controller_index
        motor_info = self._selected_motor_info()
        if motor_info is None or motor_info.get('profile_mode') != 0:
            return
        if controller_index >= len(self._motor_status.encoder):
            return

        text = self._jog_zero_offset_degree_text.toPlainText().strip()
        try:
            target_degree = float(text)
        except ValueError:
            self._node.get_logger().warning(f"Invalid zero-offset degree value: '{text}'")
            return

        try:
            with open(self._config_file, 'r', encoding='utf-8') as config_file:
                config = yaml.safe_load(config_file) or {}
        except Exception as exc:
            self._node.get_logger().error(f"Failed to load motor config: {exc}")
            return

        driver_config = self._driver_config(config, motor_info)
        if driver_config is None:
            self._node.get_logger().warning(
                f"Driver {motor_info.get('driver_id')} was not found in config."
            )
            return

        pulse_per_revolution = float(driver_config.get('pulse_per_revolution', 0))
        gear_ratio = float(driver_config.get('gear_ratio', 1.0))
        if pulse_per_revolution <= 0.0 or gear_ratio == 0.0:
            self._node.get_logger().warning("Invalid pulse_per_revolution or gear_ratio.")
            return

        current_encoder = int(self._motor_status.encoder[controller_index])
        zero_offset = int(round(
            current_encoder -
            target_degree * gear_ratio / 360.0 * pulse_per_revolution
        ))

        if self._write_driver_zero_offset(driver_config.get('id'), zero_offset):
            self._node.get_logger().info(
                f"Updated zero_offset for driver {driver_config.get('id')} to {zero_offset}."
            )
        else:
            self._node.get_logger().error("Failed to update zero_offset in motor config.")

    def _on_select_motor_clicked(self, index):
        if self._motor_status is None:
            return

        motor_info = self._motor_info_by_controller_index.get(index)
        if motor_info is None:
            return

        self._current_controller_index = index
        self._select_motor_button.setText(f"Motor {index}")
        self._update_motor_request_buttons()
        self._update_jog_current_encoder_label()

        status_index = self._motor_status_array_index(index)
        if status_index is None:
            return

        if motor_info["profile_mode"] == 0:
            if status_index >= len(self._motor_status.position):
                return
            lower = int(motor_info["lower"] * 100)
            upper = int(motor_info["upper"] * 100)
            current_value = int(round(self._motor_status.position[status_index] * 100.0))
            self._max_value_label.setText(f"{int(upper // 100)}.{int(upper % 100)}")

        elif motor_info["profile_mode"] == 1:
            if status_index >= len(self._motor_status.velocity):
                return
            velocity_limit = int(motor_info["speed"])
            lower = -int(velocity_limit)
            upper = int(velocity_limit)
            current_value = int(self._motor_status.velocity[status_index])
            self._max_value_label.setText(f"{int(upper)}")

        elif motor_info["profile_mode"] == 2:
            if status_index >= len(self._motor_status.effort):
                return
            effort_limit = int(motor_info["rated_effort"])
            lower = -int(effort_limit)
            upper = int(effort_limit)
            current_value = int(self._motor_status.effort[status_index])
            self._max_value_label.setText(f"{int(upper)}")

        self._set_slider_state_without_command(lower, upper, current_value)
        self._set_current_value_label(motor_info['profile_mode'], current_value)
    
    def _on_update_timer(self):
        self._sync_position_slider_from_motor_status()
        self._update_jog_current_encoder_label()
        if self._is_visible:
            self._plot_graph()
    

    def _plot_graph(self):
        if self._current_controller_index is None:
            return

        if (len(self._positions) == 50):
            self._motor_infos_plot_widget.clear()
            self._motor_infos_plot_widget.addLegend(offset=(10, 10))

            motor_info = self._motor_info_by_controller_index.get(self._current_controller_index)
            if motor_info is None:
                return

            idx = motor_info["controller_index"]
            if any(idx >= len(raw) for raw in self._positions):
                return

            pos = [raw[idx] for raw in self._positions]
            vel = [raw[idx] for raw in self._velocities]
            tau = [raw[idx] for raw in self._efforts]

            self._motor_infos_plot_widget.plot(pos, pen=pg.mkPen(color='b', width=3), name=f'{motor_info["alias"]}_position')
            self._motor_infos_plot_widget.plot(vel, pen=pg.mkPen(color='g', width=3), name=f'{motor_info["alias"]}_velocity')
            self._motor_infos_plot_widget.plot(tau, pen=pg.mkPen(color='r', width=3), name=f'{motor_info["alias"]}_effort')


    def motor_status_callback(self, msg):
        self._motor_status = msg
        self._positions.append(msg.position)
        self._velocities.append(msg.velocity)
        self._efforts.append(msg.effort)

        if len(self._positions) > 50:
            self._positions.pop(0)
            self._velocities.pop(0)
            self._efforts.pop(0)

    def shutdown_widget(self):
        self._update_timer.stop()
        self._node.destroy_node()
