# J4012 Jazzy handoff

## Baseline

- Branch: `kmu-finals-dev`
- Pre-release WIP baseline: `c180ab01602882ead7454c56aa5d819de8e4db7d`
- The release commit is the Git commit containing this document and the
  production changes listed below.
- No motor, serial, camera hardware, or vehicle driver was run on the laptop.

## Canonical deploy models

All paths below are package-share-relative at runtime; no Downloads path is
used by production launch files.

| Model | Package-share-relative path | SHA-256 |
|---|---|---|
| PIDNet-S | `segmentation_tools/model/pidnet_s_best.pt` | `8a83aa8993aa629d0931c3a5d44506186624ea79d1cfc54261c8408ad7f67b7b` |
| train-10 detector | `object_detection/model/train10_detector_best.onnx` | `0733b3d1f18058a0d03b918aefd288f3972cb5ea1240cf943ecad191a3deb0b6` |

PIDNet needs PyTorch. The single object/traffic detector uses ONNX Runtime. The
laptop verification provider was `CPUExecutionProvider`; PIDNet runtime was
deferred because torch is intentionally not installed in this environment.

## ROS contracts

- `/lane_offset`: `std_msgs/msg/Int16`
- `/lane_valid`: `std_msgs/msg/Bool`
- `/object_info`: `std_msgs/msg/Int32MultiArray`, exactly `[traffic, fixed_lane, moving_lane]`
- `/object_info_raw`: existing 12-field contract
- `/side_clearance`: `std_msgs/msg/Float32MultiArray`, exactly `[left_m, right_m]`
- `/rubbercone_info`: existing 4-field contract
- `/rubbercone_offset`: exactly `[offset, end_flag]`
- `/rubbercone_session_active`: reliable/transient-local/depth 1
- `/road_surface`: `std_msgs/msg/Int32`, `0=unknown`, `1=normal`, `2=shortcut`
- `/scan`: `sensor_msgs/msg/LaserScan`
- production motor output: `/xycar_motor`, `std_msgs/msg/Float32MultiArray`, `[steering_angle, speed]`
- laptop test motor output: `/kmu_main_offline/xycar_motor`

The perception flow is:

```text
/resized_image
  -> PIDNet -> /lane_segmentation_mask, /pidnet_class_map
  -> lane_node -> /lane_offset, /lane_valid
  -> train-10 detector -> direct vehicle/traffic class mapping
  -> object semantic producer -> /object_info and related contracts
  -> Main/FSM -> motor topic
```

## Build and dependencies

Build from the workspace root, not from `src`:

```bash
cd /home/bene15/xycar_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select image_resize segmentation_tools object_detection road_surface lane_detection main --symlink-install
```

ROS packages include `rclpy`, `rclcpp`, `ament_index_python`,
`ament_index_cpp`, `launch`, `launch_ros`, `sensor_msgs`, `std_msgs`,
`cv_bridge`, `rosbag2_py`, `xycar_msgs`, `image_resize`, `lane_detection`,
`object_detection`, `road_surface`, `rubbercone`, `joy`, `xycar_cam`,
`xycar_lidar`, and `xycar_ultrasonic`.

Python/runtime dependencies include NumPy, OpenCV, PyYAML, PyTorch,
ONNX Runtime, `rclpy`, `cv_bridge`, and `ament_index_python`.
C++ dependencies include OpenCV, cv_bridge, nlohmann JSON, rclcpp, and
ament_index_cpp.

## Hardware-free smoke checks

Verify the package-share model files and ONNX inference first:

```bash
cd /home/bene15/xycar_ws
source install/setup.bash
sha256sum install/segmentation_tools/share/segmentation_tools/model/pidnet_s_best.pt
sha256sum install/object_detection/share/object_detection/model/train10_detector_best.onnx
ros2 launch main module_drive_bag_test.py --show-args
```

Run the Main-only motor handoff test with `ROS_DOMAIN_ID=86`; it must use
`/kmu_main_offline/xycar_motor`. Never send a command to `/xycar_motor` on the
laptop.

Before any live motor test on J4012, confirm exactly one motor subscriber and
zero manual/joystick motor publishers, then verify the stale-input STOP state
with the vehicle secured. Only after that should low-speed steering sign and
speed scaling be checked.

## Deferred or blocked items

- `PIDNET_RUNTIME_RESULT: DEFERRED_TO_JAZZY_RUNTIME` — install/validate torch
  on J4012 and run one real checkpoint load/forward.
- `ROAD_SURFACE_THRESHOLD_UNVERIFIED` — no normal-road negative dataset was
  available; do not promote guessed thresholds.
- `SHORTCUT_ACTUATION_UNVERIFIED` — semantic FSM is tested, but real shortcut
  steering evidence is not established.
- `RUBBERCONE_PRODUCER_BLOCKER` — producer false-entry issue remains separate;
  Main cone lifecycle was not distorted to hide it.
- The repository does not contain a confirmed physical motor driver
  subscriber. Inspect the J4012 driver and its units/QoS before issuing any
  real `/xycar_motor` command.
- `HARDWARE_ACTUATION_RESULT: UNVERIFIED_HARDWARE`

## Jazzy revalidation checklist

1. Rebuild the listed packages on J4012 using the Jazzy environment.
2. Verify package-share model lookup and all three SHA-256 values.
3. Load and forward PIDNet once with the available CPU/CUDA provider.
4. Load and forward train-10 and light1 with ONNX Runtime.
5. Re-run semantic contract tests and remapped Main E2E.
6. Verify `/xycar_motor` driver type, array order, units, QoS, and single-owner
   publisher policy.
7. Validate road-surface thresholds with normal-road and shortcut data.
8. Validate shortcut steering and rubbercone producer behavior separately.
