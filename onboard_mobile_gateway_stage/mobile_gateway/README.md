# mobile_gateway

ROS Noetic phone adapter for Diff-Planner. It is deliberately separated from flight-control code.

## Data flow

- subscribes to `/mavros/state`, `/mavros/battery`;
- prefers `/ekf/ekf_odom` and falls back to `/mavros/local_position/odom` for display;
- reads either a ROS `sensor_msgs/CompressedImage` topic or a Jetson CSI camera through GStreamer;
- serves status/command messages at `ws://<host>:8765/ws/control`;
- serves authenticated MJPEG at `http://<host>:8080/stream.mjpg`;
- forwards MOVE/ROTATE/ORBIT/HOLD only through the existing
  `/atomic_skill_executor/execute` action;
- forwards TAKEOFF mission and LAND only to `/mobile_gateway/mission_request`, and only
  when a real mission orchestrator is subscribed.

It never calls MAVROS arming/takeoff/land services, never publishes raw setpoints, and never runs
anything under `sh_files`.

## Safety defaults

Both `command_forwarding_enabled` and `mission_forwarding_enabled` default to `false`. A valid
authentication token, fresh timestamp, unique request ID, allow-listed action and parameter range
are required. Enabling the gateway does not change `atomic_skills.yaml`; the atomic executor's own
`execution_enabled` gate remains authoritative.

If the Jetson wall clock isn't synchronized, the authenticated Android `hello` message supplies a
session clock anchor. Command TTL checks then use monotonic elapsed time; the node does not modify
the Jetson system clock.

## Build and safe transport test

```bash
cd <DIFF_PLANNER_WORKSPACE>
catkin_make --pkg mobile_gateway
source devel/setup.bash
roslaunch mobile_gateway mobile_gateway.launch \
  command_forwarding_enabled:=false mission_forwarding_enabled:=false
```

In another terminal:

```bash
source <DIFF_PLANNER_WORKSPACE>/devel/setup.bash
rosrun mobile_gateway gateway_smoke_client.py \
  --token-file <PRIVATE_TOKEN_FILE>
```

The smoke test deliberately sends a HOLD envelope and passes only when the safety gate rejects it.

## Camera modes

- `camera_mode:=ros_compressed` (default): subscribes to
  `/camera/color/image_raw/compressed` and does not open camera hardware.
- `camera_mode:=gstreamer_csi`: opens the configured Jetson CSI sensor using
  `nvarguscamerasrc`. Use this only after confirming the physical sensor ID and pipeline.
- `camera_mode:=disabled`: returns a placeholder image.

For the D435 wiring used by VINS (`infra1` + `infra2`), the color topic used by YOLO,
and the depth image topic, start:

```bash
roslaunch mobile_gateway realsense_phone_gateway.launch \
  command_forwarding_enabled:=false mission_forwarding_enabled:=false
```

This starts color, infrared and depth acquisition, color compression and the phone gateway. It does not start
VINS, PX4Ctrl, Diff-Planner or any flight-control node. If no camera is physically enumerated,
`rs-enumerate-devices` and the launch log will report that explicitly.

For a hardware-independent camera/Yolo transport test, publish a repository image with:

```bash
rosrun mobile_gateway simulated_camera_node.py _image:=/absolute/path/to/test.jpg _fps:=10
```

The simulator defaults to `640x480`, matching the existing YOLO node's fixed input-copy size.
