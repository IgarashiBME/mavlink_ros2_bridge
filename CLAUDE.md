# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Run

```bash
# Build (from workspace root ~/ros2_ws)
colcon build --packages-select mavlink_ros2_bridge
source install/setup.bash

# Run with default localhost GCS
ros2 run mavlink_ros2_bridge mavlink_ros2_bridge_node

# Run with remote GCS
ros2 run mavlink_ros2_bridge mavlink_ros2_bridge_node --ros-args -p gcs_ip:=192.168.11.100
```

Compiler flags: `-Wall -Wextra -Wpedantic`, C++17, C99. Linting uses `ament_lint_auto`/`ament_lint_common` (runs during `BUILD_TESTING`). No tests exist yet — `test/`, `launch/`, and `config/` directories are empty.

## Architecture

This is a single ROS2 C++ node (`MAVLinkBridgeNode`) that bridges QGroundControl (QGC) via MAVLink v2 over UDP with ROS2 topics. The MAVLink C library is vendored in `include/c_library_v2/` (included as SYSTEM to suppress warnings).

**Two source files:**
- `src/mavlink_ros2_bridge_node.cpp` — Main node: MAVLink message handlers, timers, ROS2 pub/sub, mission protocol, ARM/DISARM state machine
- `src/udp_socket.cpp` — Non-blocking UDP socket wrapper with GCS address auto-learning from first received packet

**Data flow:**
- **Inbound (sensors → GCS):** ROS2 subscribers (`/gnss`, `/heading`, `/auto_log`) store latest data → heartbeat timer (900ms) packs and sends 7 MAVLink telemetry messages to QGC
- **Outbound (GCS → ROS2):** Receive timer (10ms) polls UDP → parses MAVLink → handlers publish to `/mav/mission`, `/mav/modes`, `/mav/joystick`, `/mav/mission_set_current`

**Three timers drive the node:**
- `heartbeat_timer_` (900ms) — Sends telemetry suite (HEARTBEAT, SYS_STATUS, LOCAL_POSITION_NED, ATTITUDE, GPS_RAW_INT, ESTIMATOR_STATUS, MISSION_CURRENT) and publishes ARM/DISARM state to `/mav/modes`
- `receive_timer_` (10ms) — UDP receive polling, MAVLink parsing, PARAM_SET handling
- `mission_request_timer_` (30ms) — Activated on MISSION_COUNT to drive the mission download protocol sequence (MISSION_REQUEST_INT → MISSION_ITEM_INT → MISSION_ACK)

**UDP ports:** Binds locally on 14551 (RX), sends to GCS on 14550 (TX).

**QGC-accessible parameters** (synced via MAVLink PARAM_SET/PARAM_REQUEST_LIST): `Kp`, `Ki`, `Kd`, `look_ahead`, `i_control_dist`, `i_limit`, `linear_velocity`. These are standard ROS2 parameters that QGC can read/write through the MAVLink parameter protocol.

**Vehicle identity:** Emulates ArduPilot ground rover (MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA, sysid=1, compid=MAV_COMP_ID_AUTOPILOT1).

## Key Conventions

- Design doc is in `docs/design.md` (written in Japanese) — authoritative reference for protocol details, state diagrams, and topic specifications
- `reference/reference.cpp` is a legacy ROS1 implementation kept for reference only — do not modify
- All ROS2 topic data uses `Float64MultiArray`, `Int32MultiArray`, `TwistStamped`, or `UInt16` from `std_msgs`/`geometry_msgs` — no custom message types
- ARM state uses ArduPilot-specific base_mode constants: 217 (armed/guided), 89 (disarmed/guided)
