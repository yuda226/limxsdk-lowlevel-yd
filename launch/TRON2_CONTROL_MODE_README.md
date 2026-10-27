# TRON2 SDK Mode Control

`tron2_control_mode` provides ROS 2 services for robot mode requests using
the SDK JSON API. Despite its name, it uses **SDK/MROS, not a direct WebSocket
client**. `ros2_bridger` is not required for this node.
MROS is LimX’s middleware for communicating with the robot. The SDK uses it to 
publish and subscribe to named topics, somewhat like ROS topics, but it is not 
ROS 2 DDS and the mode-control node is not opening a WebSocket.

```text
ROS 2 service -> control node -> SDK/MROS -> robot
Robot JSON response -> GUID/title/result matching -> ROS 2 service response
```

## Build and Start

Use a normal ROS workspace terminal, not one sourced with the bridge's customized
message packages.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/yuda/robot/limx_ws
colcon build --packages-select limxsdk_lowlevel --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

Keep the robot's operating area clear and its physical emergency stop accessible.
Stop other applications that might issue conflicting mode commands. Replace
`YOUR_ROBOT_SERIAL` with the robot's actual `accid`:

```bash
ros2 launch limxsdk_lowlevel tron2_control_mode.launch.py \
  robot_ip:=10.192.1.2 \
  accid:=YOUR_ROBOT_SERIAL
```

Omit `accid` to learn it from an incoming robot message. Commands are rejected
until a valid ID is known. A configured ID must match incoming messages.

For a hardware-free dry run:

```bash
ros2 launch limxsdk_lowlevel tron2_control_mode.launch.py offline_mode:=true
```

Offline service calls return `success=false` with a dry-run explanation. They do
not send commands or simulate robot acknowledgments.

## Verify Communication

In each additional terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/yuda/robot/limx_ws/install/setup.bash
```

Monitor these topics in separate terminals:

```bash
ros2 topic echo /tron2_control_mode/status
ros2 topic echo /tron2_control_mode/json_rx
ros2 topic echo /tron2_control_mode/json_tx
```

The status JSON includes connection freshness, requested mode, last command,
last result, and pending request count. `confirmed_mode` remains `UNKNOWN`:
the node does not yet decode physical mode feedback.

Test a request without commanding locomotion:

```bash
ros2 service call /tron2_control_mode/enable_imu \
  std_srvs/srv/SetBool "{data: true}"
```

A valid matching robot message establishes receive activity. A response with the
matching `guid`, expected response title, and recognized result establishes the
command outcome. `success=true` means **accepted, not physically completed**.

The default result schema is a string at `/data/result`, with `success` meaning
acceptance. Inspect `json_rx` before adapting this to your firmware, for example:

```bash
ros2 launch limxsdk_lowlevel tron2_control_mode.launch.py \
  accid:=YOUR_ROBOT_SERIAL \
  ack_result_pointer:=/data/status \
  ack_success_value:=ok
```

That example is a configurable alternative, not a verified robot response schema.

## Request Modes

Run one service call at a time. After the stand request is accepted, verify
that the robot has physically finished standing before requesting walk mode.
An accepted response means the robot acknowledged the request; it does not
confirm physical completion.

| Service | Type | Command |
| --- | --- | --- |
| `/tron2_control_mode/stand` | `std_srvs/srv/Trigger` | `ros2 service call /tron2_control_mode/stand std_srvs/srv/Trigger "{}"` |
| `/tron2_control_mode/walk` | `std_srvs/srv/Trigger` | `ros2 service call /tron2_control_mode/walk std_srvs/srv/Trigger "{}"` |
| `/tron2_control_mode/sitdown` | `std_srvs/srv/Trigger` | `ros2 service call /tron2_control_mode/sitdown std_srvs/srv/Trigger "{}"` |
| `/tron2_control_mode/emgy_stop` | `std_srvs/srv/Trigger` | `ros2 service call /tron2_control_mode/emgy_stop std_srvs/srv/Trigger "{}"` |
| `/tron2_control_mode/recover` | `std_srvs/srv/Trigger` | `ros2 service call /tron2_control_mode/recover std_srvs/srv/Trigger "{}"` |
| `/tron2_control_mode/enable_imu` | `std_srvs/srv/SetBool` | `ros2 service call /tron2_control_mode/enable_imu std_srvs/srv/SetBool "{data: true}"` |
| `/tron2_control_mode/stair_mode` | `std_srvs/srv/SetBool` | `ros2 service call /tron2_control_mode/stair_mode std_srvs/srv/SetBool "{data: true}"` (wheel-leg only) |
| `/tron2_control_mode/base_height` | `std_srvs/srv/SetBool` | `ros2 service call /tron2_control_mode/base_height std_srvs/srv/SetBool "{data: true}"` |

For the `SetBool` services, pass `data: false` to disable IMU streaming or
stair mode. For base height, `true` raises and `false` lowers the body. Stair
mode is supported only on wheel-leg robots.

Emergency-stop requests supersede pending mode requests, but remain dependent on
communication and robot firmware. They are not a substitute for the physical
emergency stop. Recovery and height/stair requests can cause motion; use the
robot's operating procedures.

After walk mode is acknowledged and the robot is ready, the node also accepts
velocity commands on `/cmd_vel`. For example, publish a low forward speed:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \\
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Stop the publisher with Ctrl+C. If no command arrives within `deadman_timeout_s`,
the node sends a zero-velocity request. Keep the operating area clear and the
physical emergency stop accessible.

## Launch Arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `robot_ip` | `10.192.1.2` | Robot address |
| `robot_type` | `wheel_leg` | `wheel_leg` or `sole_foot` |
| `accid` | empty | Robot ID; empty learns from messages |
| `offline_mode` | `false` | No robot connection; dry-run failures |
| `ack_timeout_s` | `3.0` | Positive acknowledgment timeout |
| `stale_timeout_s` | `5.0` | Positive receive freshness timeout |
| `ack_result_pointer` | `/data/result` | JSON pointer to a string result |
| `ack_success_value` | `success` | Result value meaning acceptance |

## Troubleshooting and Tests

- `SDK_INIT_FAILED`: check robot power, IP, network, and SDK logs.
- `UNKNOWN` connection: no valid matching robot message has been received.
- `STALE`: matching receive activity exceeded `stale_timeout_s`.
- `TIMED_OUT`: no timely matching acknowledgment; the robot outcome is unknown.
  Inspect the robot before retrying. Do not blindly repeat motion requests.
- `UNKNOWN_RESULT`: inspect raw JSON and verify the acknowledgment schema.
- `REJECTED`: inspect the robot's error result, such as `fail_imu` or `fail_motor`.

`ping` tests IP reachability only. An open TCP port does not prove WebSocket or
SDK command communication. The local test executable checks synthetic replies
and offline ROS services; it does not validate a live robot connection.

```bash
cd /home/yuda/robot/limx_ws
colcon test --packages-select limxsdk_lowlevel \
  --ctest-args -R '^tron2_control_mode$' --output-on-failure
colcon test-result --verbose
```