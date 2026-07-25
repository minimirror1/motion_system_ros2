# Motion System

## English

### Installation

Clone this repository as the `src` directory of a colcon workspace.

```bash
mkdir -p ~/colcon_ws
cd ~/colcon_ws
git clone --recurse-submodules https://github.com/SeonilChoi/motion_system.git src
```

If the repository was cloned without submodules, initialize them separately.

```bash
cd ~/colcon_ws/src
git submodule update --init --recursive
```

Source ROS 2 Humble and install the required dependencies.

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### Install the `files` Directory

Open the
[Google Drive shared folder](https://drive.google.com/drive/folders/1aCavCTzrEjfnkBebYB36TDPYnzPHz5uE?usp=sharing),
download the `files` directory, and place it in the root of the colcon workspace.

```text
~/colcon_ws/
├── files/
└── src/
```

### Build

Build all packages.

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

To build only the packages required by the motor manager:

```bash
colcon build --packages-up-to motion_control_bridge
```

### Run

Source ROS 2 and the workspace in every new terminal.

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
```

Run the robot manager.

```bash
ros2 launch motion_control_robot robot_manager_node.launch.py
```

Run the RQt motor control UI.

```bash
ros2 launch motion_control_rqt display_motor_manager_node.launch.py
```

Run the RQt rocking-chair robot control UI.

```bash
ros2 launch motion_control_rqt active_robot_manager.launch.py
```

Run MIDI motor control.

```bash
ros2 launch motion_control_midi motion_control_midi_node.launch.py
```

## 한국어

### 설치

저장소를 colcon 워크스페이스의 `src` 폴더로 clone합니다.

```bash
mkdir -p ~/colcon_ws
cd ~/colcon_ws
git clone --recurse-submodules https://github.com/SeonilChoi/motion_system.git src
```

submodule 없이 clone한 경우 다음 명령을 추가로 실행합니다.

```bash
cd ~/colcon_ws/src
git submodule update --init --recursive
```

ROS 2 Humble 환경을 불러오고 필요한 의존성을 설치합니다.

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### `files` 폴더 설치

[Google Drive 공유 폴더](https://drive.google.com/drive/folders/1aCavCTzrEjfnkBebYB36TDPYnzPHz5uE?usp=sharing)에
접속하여 `files` 폴더를 다운로드한 뒤, colcon 워크스페이스 최상위 경로에
배치합니다.

```text
~/colcon_ws/
├── files/
└── src/
```

### 빌드

전체 패키지를 빌드합니다.

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

모터 매니저 관련 패키지만 빌드하려면 다음 명령을 사용합니다.

```bash
colcon build --packages-up-to motion_control_bridge
```

### 실행

새 터미널을 열 때마다 ROS 2와 워크스페이스 환경을 불러옵니다.

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
```

로봇 매니저를 실행합니다.

```bash
ros2 launch motion_control_robot robot_manager_node.launch.py
```

RQt 모터 제어 UI를 실행합니다.

```bash
ros2 launch motion_control_rqt display_motor_manager_node.launch.py
```

RQt rocking-chair 로봇 제어 UI를 실행합니다.

```bash
ros2 launch motion_control_rqt active_robot_manager.launch.py
```

MIDI 모터 제어를 실행합니다.

```bash
ros2 launch motion_control_midi motion_control_midi_node.launch.py
```
