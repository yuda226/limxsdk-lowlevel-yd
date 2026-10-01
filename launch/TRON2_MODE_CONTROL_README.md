# TRON2 Mode Management & Velocity Control - Complete Guide

## Overview

The TRON2 control system requires state management through the **High-Level Application Development Interface** (WebSocket). The `tron2_request_mode` node handles all mode transitions and queries defined in SDK sections **3.8.1 through 3.8.9**:

| Section | Command | Purpose |
|---------|---------|---------|
| 3.8.1 | `request_stand_mode` | Robot stands up |
| 3.8.2 | `request_walk_mode` | Enter walking mode |
| 3.8.3 | `request_twist` | Send velocity commands |
| 3.8.4 | `request_base_height` | Adjust body height (±5cm) |
| 3.8.5 | `request_sitdown` | Robot sits down |
| 3.8.6 | `request_stair_mode` | Enable/disable stair climbing mode (wheel-leg only) |
| 3.8.7 | `request_emgy_stop` | Emergency stop |
| 3.8.8 | `request_enable_imu` | Subscribe to IMU data |
| 3.8.9 | `request_recover` | Fall recovery (auto get-up) |

## Nodes

### 1. `tron2_request_mode` Node
Comprehensive mode management and state control node supporting all bipedal/wheel-leg protocol commands.

**Services (all std_srvs/Trigger):**
- `/tron2_stand_mode` - Request robot to stand up
- `/tron2_walk_mode` - Request robot to enter walking mode
- `/tron2_sit_mode` - Request robot to sit down
- `/tron2_emgy_stop` - Request emergency stop
- `/tron2_enable_imu` - Enable IMU data push
- `/tron2_recover` - Request fall recovery (auto get-up)

**Services (with arguments):**
- `/tron2_stair_mode` (std_srvs/SetBool) - Enable/disable stair mode (wheel-leg only)
- `/tron2_base_height` (std_srvs/SetBool) - Adjust body height (true=raise, false=lower)

**Subscriptions:**
- Robot info notifications (internal - updates robot_info and mode_status)

**Parameters:**
- `robot_ip` (string): Robot IP address (default: "10.192.1.2")
- `robot_type` (string): "wheel_leg" or "sole_foot" (default: "wheel_leg")

### 2. `tron2_cmd_vel_translator` Node
Translates ROS2 Twist messages to TRON2 velocity commands (request_twist).
Automatically handles mode validation before sending commands.

**Subscriptions:**
- `/cmd_vel` (geometry_msgs/Twist) - Input velocity commands

**Publications:**
- `/tron2/cmd_vel` (geometry_msgs/Twist) - Optional relay output

**Parameters:**
- `enable_sdk_output` (bool): Send commands to robot (default: true)
- `enable_relay_output` (bool): Re-publish for debugging (default: false)
- `robot_ip` (string): Robot IP address (default: "10.192.1.2")
- `cmd_vel_topic_in` (string): Input subscription topic (default: "/cmd_vel")
- `cmd_vel_topic_out` (string): Output relay topic (default: "/tron2/cmd_vel")
- `max_linear_x` (double): Clamp forward velocity in m/s (default: 0.6)
- `max_linear_y` (double): Clamp lateral velocity in m/s (default: 0.4)
- `max_angular_z` (double): Clamp rotation velocity in rad/s (default: 1.2)
- `deadman_timeout_s` (double): Auto-stop timeout in seconds (default: 0.5)
- `sdk_json_template` (string): Custom JSON payload template

---

## Quick Start

### Launch Both Nodes Together
```bash
# For wheel-leg robot
ros2 launch limxsdk_lowlevel tron2_request_mode.launch.py robot_type:=wheel_leg

# In another terminal, start cmd_vel translator
ros2 launch limxsdk_lowlevel tron2_cmd_vel_translator.launch.py
```

---

## Usage Workflow

### Complete Control Sequence

```bash
# 1. Stand up
ros2 service call /tron2_stand_mode std_srvs/Trigger
# Wait for response: success or fail_imu/fail_motor

# 2. Enter walking mode
ros2 service call /tron2_walk_mode std_srvs/Trigger
# Wait for response: success or fail_imu/fail_motor

# 3. (Optional) Enable stair mode for wheel-leg robots
ros2 service call /tron2_stair_mode std_srvs/SetBool "{data: true}"

# 4. (Optional) Enable IMU data
ros2 service call /tron2_enable_imu std_srvs/Trigger

# 5. Send velocity commands (robot ready)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 6. Stop and sit down
ros2 service call /tron2_sit_mode std_srvs/Trigger
```

### Command Examples

**Forward motion:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Turn right:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
```

**Stop:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Adjust body height (raise by 5cm):**
```bash
ros2 service call /tron2_base_height std_srvs/SetBool "{data: true}"
```

**Adjust body height (lower by 5cm):**
```bash
ros2 service call /tron2_base_height std_srvs/SetBool "{data: false}"
```

**Emergency stop:**
```bash
ros2 service call /tron2_emgy_stop std_srvs/Trigger
```

**Fall recovery (auto get-up):**
```bash
ros2 service call /tron2_recover std_srvs/Trigger
```

## Velocity Command Ranges

### Wheel-Leg Mode (WF_TRON2A)
| Parameter | Range | Unit | Notes |
|-----------|-------|------|-------|
| `linear.x` | -3.0 to 3.0 | m/s | Forward/backward; must be >1.5 for stair climbing |
| `linear.y` | **0.0 only** | m/s | Lateral movement NOT supported in wheel-leg mode |
| `angular.z` | -1.5 to 1.5 | rad/s | Rotation velocity |

### Sole-Foot Mode (SF_TRON2A)
| Parameter | Range | Unit | Notes |
|-----------|-------|------|-------|
| `linear.x` | -1.0 to 1.0 | ratio | Forward/backward velocity ratio |
| `linear.y` | -1.0 to 1.0 | ratio | Lateral velocity ratio |
| `angular.z` | -1.0 to 1.0 | ratio | Rotation velocity ratio |

## Special Features

### Stair Climbing (Wheel-Leg Only)
1. Enable stair mode: `ros2 service call /tron2_stair_mode std_srvs/SetBool "{data: true}"`
2. Send velocity command with **linear.x ≥ 1.5 m/s**
3. Disable after climbing: `ros2 service call /tron2_stair_mode std_srvs/SetBool "{data: false}"`

### Body Height Adjustment
- Each call raises or lowers the body by **5 cm**
- Can only adjust when robot is standing or walking
- Response includes status (success or fail_status)

## Robot State Machine

```
        ┌─────────┐
        │  IDLE   │  (Robot powered on, not moving)
        └────┬────┘
             │ request_stand_mode
             ▼
        ┌──────────┐
        │ STANDING │  (Robot on feet, ready to walk)
        └────┬─────┘
             │ request_walk_mode
             ▼
        ┌──────────────────────────────────────────┐
        │            WALKING                       │  (Ready for cmd_vel)
        │  - Send velocity commands (request_twist)│
        │  - Enable stair mode (optional)          │
        │  - Enable IMU data (optional)            │
        │  - Adjust height (optional)              │
        └────┬────────────────────────┬────────────┘
             │                        │
             │ request_sitdown        │ Fall detected
             │                        │
             ▼                        ▼
        ┌─────────┐           ┌──────────────┐
        │  IDLE   │           │  FALLEN      │
        └─────────┘           └───────┬──────┘
                                     │
                                     │ request_recover
                                     ▼
                              ┌──────────────┐
                              │  RECOVERING  │
                              └──────────────┘
```

### State Transition Details

- **IDLE → STANDING**: `request_stand_mode` (requires IMU & motors OK)
- **STANDING → WALKING**: `request_walk_mode` (requires IMU & motors OK)
- **WALKING → IDLE**: `request_sitdown` 
- **Any State → Emergency**: `request_emgy_stop` (requires idle mode to execute)
- **FALLEN → WALKING**: `request_recover` (automatic get-up)

## Troubleshooting

### Network Connection
```bash
# Test robot reachability
ping 10.192.1.2

# Check if robot is responding to WebSocket
nc -zv 10.192.1.2 5000
```

### Service Failures
**Problem**: Service calls return fail_imu or fail_motor
- **Solution**: Verify robot hardware health; check battery level; restart robot

**Problem**: Robot doesn't respond to walk_mode request
- **Solution**: Must call stand_mode first; wait for completion (2-3 seconds)

**Problem**: cmd_vel commands ignored
- **Solution**: 
  1. Verify robot is in WALKING mode (`request_walk_mode` success)
  2. Check velocity values are within range for your robot type
  3. Verify deadman_timeout hasn't triggered (robot auto-stops after 0.5s)

### Debugging Output
```bash
# Monitor mode transitions
ros2 topic echo /tron2_mode_status

# Monitor cmd_vel relay (if enabled)
ros2 topic echo /tron2/cmd_vel

# Check node logs for errors
ros2 launch limxsdk_lowlevel tron2_request_mode.launch.py 2>&1 | grep -i "error\|fail\|warn"
```

### Debugging offline
```bash
source /opt/ros/jazzy/setup.bash && cd /home/yuda/robot/limx_ws && . install/setup.bash && ros2 run limxsdk_lowlevel tron2_request_mode --ros-args -p offline_mode:=true -p accid:=tron2a-405

# Use launch.py
source /opt/ros/jazzy/setup.bash
cd /home/yuda/robot/limx_ws
. install/setup.bash
ros2 launch limxsdk_lowlevel tron2_complete_setup.launch.py \
  robot_ip:=10.192.1.2 \
  robot_type:=wheel_leg \
  offline_mode:=true
```

### ACCID Issue
The `accid` (robot serial number) is automatically detected from the first robot notification received. If it shows "nav_bridge", this means:
- Robot info not yet received
- Network connectivity issue
- WebSocket connection not established

**Solution**: Verify network and robot is fully booted before launching nodes.

---

## Implementation Details

### Architecture

The `tron2_request_mode` node:
1. Connects to robot via WebSocket (port 5000)
2. Subscribes to robot notifications to get robot_info (ACCID)
3. Maintains state machine (IDLE ↔ STANDING ↔ WALKING)
4. Exposes ROS2 services for each SDK command (3.8.1 - 3.8.9)
5. Converts service calls to JSON WebSocket messages
6. Parses robot responses and updates internal state

### Protocol Reference

All commands follow the TRON2 SDK High-Level Interface format:

**Request Template:**
```json
{
  "accid": "WF_TRON2A_001",           // Robot serial number
  "title": "request_command_name",    // Command name (prefixed with request_)
  "timestamp": 1672373633989,         // Unix timestamp in milliseconds
  "guid": "unique-identifier",        // Unique message ID
  "data": {}                          // Command-specific parameters
}
```

**Response Template:**
```json
{
  "accid": "WF_TRON2A_001",
  "title": "response_command_name",
  "timestamp": 1672373633989,
  "guid": "unique-identifier",
  "data": {
    "result": "success"               // success | fail_xxx
  }
}
```

### Key Points

- All service calls are **blocking** (wait for response)
- Robot notifications are **asynchronous** (received without request)
- The `accid` must match the robot's actual serial number for commands to be accepted
- JSON messages are encoded as UTF-8 strings over WebSocket
- Timestamps are Unix milliseconds (not nanoseconds)
- Response `guid` matches request `guid` for correlation

### SDK References

For complete protocol details, see the official documentation:
- **High-Level Interface Overview**: Section 3.1 - 3.2
- **Bipedal/Wheel-Leg Commands**: Section 3.8.1 - 3.8.9
- **Communication Protocol**: Section 3.2
- **Robot Serial Number (ACCID)**: Section 3.3

---

## Performance Considerations

- **Service response time**: 100-500ms typical
- **Mode transition time**: 2-3 seconds (stand→walk or walk→sit)
- **cmd_vel publish rate**: 30+ Hz recommended (see translator implementation)
- **Watchdog check frequency**: Every 50ms (configurable)
- **WebSocket buffer**: 2MB send/receive (handles large sensor streams)
