# ROS 2 Workspace `src/` Overview

This workspace currently keeps the UAV control path: mapping/localization, waypoint publishing, aircraft PID, the STM32 serial bridge, and `drone_camera_pkg`.

## Quick Start

Run from the workspace root:

```bash
colcon build --symlink-install
# Windows PowerShell
.\install\setup.ps1
# Linux/macOS
source install/setup.bash
```

## Main Data Flow

- `bluesea2` publishes `/scan`
- `activity_control_pkg` publishes `/target_position` and `/active_controller`
- `drone_camera_pkg` publishes `/fine_data`, `/apriltag_code`, and handles `/photo_capture_request`
- An external ROS 2 node publishes `/route_choice` to select which waypoint group should run
- `activity_control_pkg` changes selected inspection waypoints into a photo flow: descend to the configured photo height, dwell, request one photo, then continue
- `pid_control_pkg` subscribes to `/target_position`, `/height`, `/visual_takeover_active`, and `/fine_data`, then publishes `/target_velocity`
- `drone_camera_pkg` saves the captured image into `src/photo` and replies on `/photo_capture_result`
- `uart_to_stm32` listens to `/route_choice` and forwards `/target_velocity` to the flight controller only while the selected route task is active
- `activity_control_pkg` publishes `/mission_complete` after all targets complete
- `uart_to_stm32` sends `/mission_complete` as serial frame `0x66` with payload `0x06`
- `uart_to_stm32` also publishes `/height`, `/is_st_ready`, and `/mission_step`

## Packages

### `activity_control_pkg`

Maintains the waypoint queue, checks whether the current target is reached, and treats `Target.is_takeover == true` as an inspection-photo waypoint.

Route selection behavior:

- The node starts in standby and does not publish any waypoint until `/route_choice` is received
- `/route_choice` uses `std_msgs/msg/UInt8`
- `1` starts the first built-in waypoint group and `2` starts the second built-in waypoint group
- After a valid route starts, later `/route_choice` messages are ignored for the rest of that run

Inspection photo behavior:

- The drone first reaches the waypoint's normal XY / Z target
- For photo waypoints, the node republishes the same XY / Yaw with `photo_target_height_cm`
- Once the aircraft is within `height_tolerance_cm`, it dwells for `photo_dwell_time_sec`
- The node publishes `/photo_capture_request` with a filename and waits for `/photo_capture_result`
- Success or timeout/failure both advance the mission so inspection does not get stuck

Key files:

- `activity_control_pkg/include/activity_control_pkg/route_target_publisher.hpp`
- `activity_control_pkg/src/route_target_publisher.cpp`
- `activity_control_pkg/src/route_target_publisher_main.cpp`
- `activity_control_pkg/src/route_test_node.cpp`

Inspection photo topics:

- `/visual_takeover_active`
- `/photo_capture_request`
- `/photo_capture_result`
- `/mission_complete`

### `drone_camera_pkg`

Runs camera preview, keeps AprilTag outputs available, and saves photos on demand. Topics:

- `/fine_data`: pixel error `[x_px, y_px]`
- `/apriltag_code`: current tag code
- `/photo_capture_request`: requested filename for a captured image
- `/photo_capture_result`: `true` on save success, `false` on failure

### `my_carto_pkg`

Launches lidar, URDF, Cartographer, and RViz together.

Key file:

- `my_carto_pkg/launch/fly_carto.launch.py`

### `my_launch`

Launches the complete demo flow.

Key file:

- `my_launch/launch/demo1.launch.py`

### `pid_control_pkg`

The only PID package left in the workspace. It converts target position and current pose into `/target_velocity`.

Control behavior:

- Normal mode: XY / Z / Yaw use waypoint PID
- Visual takeover mode: XY uses visual PID from `/fine_data`, while Z / Yaw keep using the original PID

Key files:

- `pid_control_pkg/include/pid_control_pkg/pid_controller.hpp`
- `pid_control_pkg/src/pid_controller.cpp`
- `pid_control_pkg/launch/position_pid_controller.launch.py`

### `serial_comm`

Reusable serial communication library used by `uart_to_stm32`.

### `uart_to_stm32`

Bridges ROS topics and the STM32/flight-controller serial protocol.

Remote-control gating:

- `/route_choice` is published by an external ROS 2 node using `std_msgs/msg/UInt8`
- valid route IDs currently include `1` and `2`
- after a valid `/route_choice`, target velocity forwarding stays enabled only for the active mission
- before route start and after mission completion, `/target_velocity` messages are ignored and not sent to STM32

Key files:

- `uart_to_stm32/src/uart_to_stm32_node.cpp`
- `uart_to_stm32/src/uart_to_stm32.cpp`
- `uart_to_stm32/launch/uart_to_stm32.launch.py`

Current serial frame usage:

- `0x31`: target velocity
- `0x32`: velocity/pose related data
- `0x66`: mission complete, payload `0x06`, sent `3` times

## Common Launch Commands

```bash
ros2 launch pid_control_pkg position_pid_controller.launch.py
ros2 launch uart_to_stm32 uart_to_stm32.launch.py
ros2 launch my_launch demo1.launch.py
```
