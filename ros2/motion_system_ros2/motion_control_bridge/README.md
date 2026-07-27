# motion_control_bridge

`motor_manager_node` keeps the motor-manager control loop period from the motor
configuration file and publishes `motion_control/motor_status` at a separately
selected ROS telemetry rate.

## Runtime profile

The node declares these parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `runtime_profile` | `auto` | `auto`, `desktop`, or `embedded` |
| `motor_status_publish_rate_hz` | `0` | Explicit 1–10000 Hz telemetry rate; `0` selects it from the profile |

Selection priority is:

1. A positive `motor_status_publish_rate_hz`
2. An explicit `desktop` (1000 Hz) or `embedded` (100 Hz) profile
3. With `auto`, 100 Hz on Raspberry Pi and 1000 Hz on other systems

Raspberry Pi detection checks `/proc/device-tree/compatible`,
`/proc/device-tree/model`, and `/proc/cpuinfo`. The selected platform, telemetry
rate, and unchanged motor control period are logged during node startup.

Examples:

```bash
# Automatic Raspberry Pi/desktop selection
ros2 launch motion_control_bridge motor_manager_node.launch.py

# Force the embedded profile
ros2 launch motion_control_bridge motor_manager_node.launch.py \
  runtime_profile:=embedded

# Explicit rate overrides the profile
ros2 launch motion_control_bridge motor_manager_node.launch.py \
  motor_status_publish_rate_hz:=250
```
