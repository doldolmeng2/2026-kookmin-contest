# TEST LOG

## 2026-08-05 — KMU finals rubber-cone FSM bag integration

### Baseline

- Git root: `/home/xytron/xycar_ws/src`
- Branch: `feature/2026-finals-fsm-rubbercone-integration`
- HEAD: `2a03e13c53a585c9ad5a9db036be11599c2e2526`
- Initial worktree: clean
- Bag: `/home/xytron/bags/kmu_real_lidar/rubbercone_20260725_221245`
- Bag `/scan`: `sensor_msgs/msg/LaserScan`, 68 messages, 14.782 s

### Initial failure and diagnosis

Initial evidence:

- `/tmp/kmu_rubbercone_bag_launch.log`
- `/tmp/kmu_current_rubbercone.log`
- `/tmp/kmu_bag_test_motor.log`

The launch used legacy `mode:=3`, which starts the production FSM directly in
`LANE_DRIVE`. `main_node` creates a 20 ms control timer, while the external
`/lane_offset` mock publishes at 10 Hz and first needs DDS discovery. The first
control cycle ran before a lane callback had recorded any receipt edge.

This is confirmed by the transition reason `missing required inputs:
perception:lane_offset`; a received but expired callback would have produced
`stale required inputs` instead. `LANE_DRIVE` is motion-enabled, so the safety
decision committed terminal `STOP`. Messages received after that commit cannot
recover the FSM, explaining the all-zero isolated motor trace.

Contract checks:

- Topic/type: main subscribes to `/lane_offset` as `std_msgs/msg/Int16`; the
  lane detector and mock use the same contract.
- Remap: `/lane_offset` is not remapped. Only main control output is remapped
  from `xycar_motor` to `/bag_test/xycar_motor`.
- QoS: main requests BestEffort/Volatile. The lane detector offers
  BestEffort/Volatile, and the mock profile used by the new harness matches it.
- Freshness: the callback records the main node's ROS receipt clock, not a bag
  timestamp or message header. `LANE_DRIVE` permits a maximum age of 0.5 s, so
  10 Hz is sufficient after the first receipt.

### Minimal test-only change

Production FSM, safety policy, controller, and detector behavior were not
changed. `main/tools/run_rubbercone_bag_test.sh` now orchestrates the test:

1. Abort unless `/xycar_motor` publisher count is zero.
2. Start BestEffort/Volatile lane and traffic mocks at 10 Hz.
3. Start a temporary empty scan mock and launch the existing bag-test launch in
   safe `mode:=0` (`INIT`).
4. Wait for `INIT -> WAIT_GREEN -> LANE_DRIVE`, then stop the empty scan mock.
5. Recheck `/xycar_motor` publisher count immediately before playback.
6. Run exactly `ros2 bag play <bag> --disable-keyboard-controls --topics /scan`.
7. Capture transition, detector, and isolated motor logs; clean up only the
   test processes started by the harness.

The empty scan is used only for the non-motion `INIT` readiness gate. It is
stopped before bag playback and contains no cone points, so it cannot arm cone
exit detection or overlap the recorded scan stream.

### Successful result

- Run artifacts: `/tmp/kmu_rubbercone_fsm_bag_test_20260805_022154`
- Playback topics: `/scan` only
- `/xycar_motor` publisher count: 0 before launch, 0 immediately before bag
  playback, and 0 after playback
- Observed transitions:
  - `INIT -> WAIT_GREEN: required inputs ready`
  - `WAIT_GREEN -> LANE_DRIVE: green signal debounced`
  - `LANE_DRIVE -> CONE_DRIVE: cone entry confirmed`
  - `CONE_DRIVE -> REJOIN: fresh cone end flag`
- Detector events:
  - `Rubber-cone session reset`
  - `Rubber-cone exit detection armed`
  - `Rubber-cone end latched after 3 missing frames`
- Non-zero command captured after `CONE_DRIVE` commit on the isolated topic:
  `[-39.0, 8.600000381469727]`
- `/bag_test/xycar_motor`: 835 samples, 724 non-zero and 111 zero
- No `-> STOP` transition in the successful launch log
- After cleanup, no test process remained and the ROS graph no longer exposed
  either motor topic after DDS discovery converged.

### Automated checks

- `python3 -m pytest -q main/test`: `280 passed`
- The integration test statically enforces the scan-only bag command, the three
  real-motor publisher interlocks, the isolated motor topic, safe startup, and
  absence of hardware driver commands.

