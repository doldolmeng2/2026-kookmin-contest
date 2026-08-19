# Safe Xbox manual drive

This package converts `/joy` into `Float32MultiArray [steering, speed]` motor
commands. Non-zero commands require a held deadman button and fresh Joy input.

The default launch is isolated from the real motor topic:

```bash
ros2 launch manual_drive manual_drive.py \
  deadman_button:=BUTTON_INDEX \
  live_drive:=false
```

This publishes only on `/kmu_main_offline/xycar_motor`. The default
`deadman_button:=-1` is intentionally STOP-only because this repository does
not define a verified Xbox button index.

Only after checking the controller mapping and clearing the vehicle area, use:

```bash
ros2 launch manual_drive manual_drive.py \
  deadman_button:=BUTTON_INDEX \
  live_drive:=true
```

Never run live manual drive at the same time as `module_drive.py` or
`module_drive_mission_test.py live_drive:=true`. This launch does not include a
motor arbitration node.
