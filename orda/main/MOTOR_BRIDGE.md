# J4012 ROS 2 to Noetic motor bridge

Run these scripts directly from the repository; they are operational scripts
and are not installed into `install_jazzy`.

```bash
bash ~/xycar_ws/src/orda/main/tools/start_noetic_motor.sh
```

Run a live mission only with required real producers connected. The profiles are
`1=wait_green`, `2=lane_center`, `3=lane_1`, `4=lane_2`, `5=cone`, `6=fixed`,
`7=overtake`, and `8=shortcut`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/xycar_ws/install_jazzy/setup.bash
ros2 launch main module_drive_mission_test.py test_profile:=2 live_drive:=true
```

Each live profile command is:

```bash
ros2 launch main module_drive_mission_test.py test_profile:=1 live_drive:=true
ros2 launch main module_drive_mission_test.py test_profile:=2 live_drive:=true
ros2 launch main module_drive_mission_test.py test_profile:=3 live_drive:=true
ros2 launch main module_drive_mission_test.py test_profile:=4 live_drive:=true
ros2 launch main module_drive_mission_test.py test_profile:=5 live_drive:=true
ros2 launch main module_drive_mission_test.py test_profile:=6 live_drive:=true
ros2 launch main module_drive_mission_test.py test_profile:=7 live_drive:=true
ros2 launch main module_drive_mission_test.py test_profile:=8 live_drive:=true
```

`live_drive:=false` keeps the motor output remapped and never starts the UDP
bridge. Lane-only live use:

```bash
ros2 launch main module_lane_only.py live_drive:=true udp_motor_bridge:=true
```

Manual drive uses the same bridge; start it explicitly once, then use the
existing live manual launch:

```bash
ros2 run main udp_motor_bridge
ros2 launch manual_drive manual_drive.py live_drive:=true
```

Do not run a second bridge instance while a mission launch owns one.

PIDNet warm-up happens during startup; Main remains in its requested mode with
HOLD zero output until fresh mission inputs arrive, then resumes automatically.

Inspect and stop:

```bash
bash ~/xycar_ws/src/orda/main/tools/status_noetic_motor.sh
bash ~/xycar_ws/src/orda/main/tools/stop_noetic_motor.sh
```

Charged-battery steering and propulsion validation remains required; do not use
these commands to issue a nonzero command during low-battery checks.
