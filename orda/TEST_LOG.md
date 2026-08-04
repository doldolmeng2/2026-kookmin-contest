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

## 2026-08-05 — KMU REJOIN to lane integration

### Baseline and cause classification

- Branch: `feature/2026-finals-fsm-rubbercone-integration`
- HEAD: `58dd9169f0acb6e3353824e13bfc458af21635dd`
- Initial worktree: clean

The production REJOIN guard accepts only explicit `/lane_valid`
`std_msgs/msg/Bool=True` receipt edges. A qualifying sequence requires all of
the following:

- every edge was received strictly after the REJOIN entry timestamp;
- every edge is at most 0.25 s old when evaluated;
- at least three unique true edges are received;
- at least 0.2 s elapses between the first and final qualifying edge.

The runtime discards validity edges queued before the `CONE_DRIVE -> REJOIN`
commit. `/lane_offset` is intentionally not treated as lane validity.

The positive bag contains no `/lane_valid` topic, and scan-only playback would
not replay it even if present. The production graph also has no `/lane_valid`
publisher yet. The guard itself is implemented and covered by unit tests; the
missing item is the perception-side producer contract. The previous harness
also stopped after bag completion without requiring `REJOIN -> LANE_DRIVE`.

### Test-only extension

The production FSM and runtime were not changed. The harness now:

1. requires `CONE_DRIVE -> REJOIN`;
2. starts a BestEffort/Volatile 10 Hz `/lane_valid=True` test publisher only
   after that committed transition;
3. requires `REJOIN -> LANE_DRIVE: fresh lane validity confirmed`;
4. captures a non-zero `/bag_test/xycar_motor` lane command after the commit;
5. cleans up the new mock and waits through the Fast DDS graph cache window.

### Successful result

- Run artifacts: `/tmp/kmu_rubbercone_fsm_bag_test_20260805_023558`
- Playback command: `ros2 bag play <bag> --disable-keyboard-controls --topics /scan`
- `/xycar_motor` publisher count: 0 before launch, 0 immediately before
  playback, and 0 after playback
- Full transition chain:
  - `INIT -> WAIT_GREEN`
  - `WAIT_GREEN -> LANE_DRIVE`
  - `LANE_DRIVE -> CONE_DRIVE`
  - `CONE_DRIVE -> REJOIN`
  - `REJOIN -> LANE_DRIVE`
- CONE isolated sample: `[-34.0, 9.100000381469727]`
- Post-REJOIN lane isolated sample: `[0.0, 6.800000190734863]`
- No `-> STOP` transition
- No hardware driver was started and the bag replayed `/scan` only
- OS process inspection found no remaining test process. The first 5 s graph
  check observed stale Fast DDS discovery entries, which then expired without
  intervention; the harness convergence window is now 15 s.

### Next validation plan

Negative bag:

1. Confirm the scene label for
   `/home/xytron/bags/kmu_real_lidar/20260724/20260724_105545_kmu_real_lidar_C01_sensor_idle_static_10sec_raw`
   before treating it as ground-truth no-cone data. It contains 155 `/scan`
   messages over 15.975 s.
2. Add an explicit `negative` expectation to the same harness while retaining
   all motor interlocks and scan-only playback.
3. Require no `LANE_DRIVE -> CONE_DRIVE`, no detector session reset/end latch,
   no STOP, continued isolated lane control, and `/xycar_motor` publisher 0.
4. Treat any cone entry as a detector false positive and retain the full scan,
   detector, transition, and motor artifacts.

Repeated sessions in one process:

1. Keep one main/rubbercone launch alive and replay the positive bag twice,
   checking `/xycar_motor` publisher 0 before each replay.
2. Use transition occurrence counts or per-session log offsets so the second
   session cannot pass on first-session log lines.
3. Start and stop the lane-validity mock after each distinct REJOIN commit.
4. Require two complete `LANE -> CONE -> REJOIN -> LANE` chains, two detector
   resets, non-zero isolated cone and returned-lane control in both sessions,
   no STOP, and clean graph/process teardown.

## 2026-08-05 — KMU finals FSM orchestration skeleton

### Baseline and scope

- Branch: `feature/2026-finals-fsm-rubbercone-integration`
- Start/end HEAD: `9bef8f67a0d87559de052663850b1eadc6d5be04`
- Initial worktree: clean
- No ROS launch, hardware driver, bag playback, or motor publish was run.
- No production ROS topic was created for zone, route, overtake-complete, or
  shortcut-complete inputs. Those inputs exist only as typed runtime injection
  seams until real publishers are defined.

### Implemented contracts

- Completed the ten-state pure FSM transition skeleton, including explicit
  fresh receipt edges for fixed-zone entry/exit, overtake completion, shortcut
  completion, three traffic encounters, and `FINISH`.
- Replaced mutable Gate/shortcut flags with canonical `completed_laps` and
  `shortcut_lap`; compatibility names are derived aliases only.
- Added strict typed adapters for the existing ten-field `/object_info` and
  `/lane_change_state [changing, success]` topics.
- Added FIXED/OVERTAKE lane-action orchestration. A fresh object may select the
  opposite lane, fresh lane-change success changes only the action output from
  legacy mode 5 to mode 3, and only explicit mission completion exits the FSM
  state.
- Added an internal route-traffic contract. RED/AMBER latches a recoverable
  zero-control override; the existing Bool `/traffic_detection` remains start
  green only and cannot affect lap or route decisions.

### Verification classification

- UNIT PASS: `142 passed` for FSM, context, control selection, mode adapter, and
  safety tests.
- MOCK PASS: `104 passed` for typed runtime events, callbacks, QoS, action
  orchestration, and state-contract tests.
- Full main regression: `350 passed` (previous baseline: `280 passed`).
- BAG PASS: retained from the 2026-08-05 scan-only rubbercone run recorded
  above. The bag was deliberately not replayed for this change.
- UNVERIFIED: production fixed-zone entry/exit, route traffic/lap publisher,
  overtake-complete publisher, shortcut controller/completion, production IMU
  wiring, fixed/overtake mission bags, and real-vehicle behavior.

### Preserved backlog

- Rubbercone negative-bag validation and repeated sessions in one process
  remain the regression backlog described in the preceding section.

## 2026-08-05 — Object absence contract and fixed-source audit

### Checkpoint

- The preceding 19-file FSM skeleton was committed as
  `d5aedbde4d2bb242917f126c41068d47af548f91` with message
  `feat(kmu-real): complete finals FSM orchestration skeleton` after
  `350 passed` and `git diff --check`.
- The checkpoint was not pushed. This section's adapter/test changes remain
  uncommitted for review.

### Actual `/object_info` publisher contract

- Source: `perception/object_detection/src/object_detection.cpp`.
- `onScan()` calls `publishEmpty()` when no usable cluster exists;
  `resetLidarState()` stores `lidar_valid=false`, `min_dist=+inf`, angle/span
  zero, and cluster count zero.
- `onImage()` independently resets no-box image fields to box size/cx/cy/dx
  zero and lane label zero. Therefore the full no-cluster/no-box payload is
  `[0, +inf, 0, 0, 0, 0, 0, 0, 0, 0]`. If an image box exists while LiDAR is
  absent, the last five image-derived fields can be finite non-zero values but
  are not actionable object evidence.
- `onPublishTick()` creates exactly ten fields and publishes every 20 ms
  (50 Hz) with KeepLast(10), BestEffort, Volatile QoS. `exists` is exactly 0
  or 1. A later valid scan/image overwrites the reset state, so detection
  values recover without a sticky no-detection latch.
- The runtime adapter previously required all ten values to be finite and
  rejected this normal `exists=0, min_dist=+inf` heartbeat.

### Adapter correction

- `exists=0` now retains the receipt timestamp but normalizes typed distance
  to `None` and lane to `ObjectLane.UNKNOWN`; no lane-change action starts.
- `exists=1` still requires all fields to be finite, a non-negative distance,
  and a valid lane enum. NaN, invalid length/enum, negative values, and
  non-publisher infinities in absent metadata remain malformed rather than
  being silently converted to no-object.
- UNIT PASS: `36 passed` in `test_mission_adapters.py`, including the exact
  publisher sentinel, no-object/object recovery in both directions, and
  stale/pre-entry/action-completion isolation.
- MOCK PASS: `28 passed` in `test_main_runtime.py`, including callback receipt
  and no-cluster normalization.
- Full regression: `357 passed` (checkpoint baseline: `350 passed`).

### Fixed-zone source decision

| Decision | Available source | Meaning | Missing information |
|---|---|---|---|
| FIXED entry | `/object_info` plus internal race phase | Fresh LiDAR cluster and optional image lane label after the preceding phase | No fixed-zone identity, explicit boundary, active debounce, or validated distance threshold |
| Lane-change start | `/mode_info [5, target]` action output | Requests the lane detector's existing reference transition | Requires a trustworthy FIXED entry and valid fresh object lane |
| Lane-change complete | Declared `/lane_change_state [changing, success]` | Intended offset spike/settle success pulse | `updateLaneChangeState()` has no call site in current C++; inspected bags record zero messages |
| Obstacle passed | `/object_info exists=0` is the only possible clear observation | Current detector saw no qualifying LiDAR cluster | Cannot distinguish a passed obstacle from occlusion, scan dropout, or temporary clustering miss |
| FIXED exit | Internal typed test seam only | Explicit fresh edge used by the pure FSM tests | No production publisher, position/zone boundary, or independently validated pass evidence |

- The object detector has no temporal debounce. Its defaults accept points only
  through 2.0 m while `detect_threshold_m` defaults to 6.0 m, so every accepted
  cluster becomes `exists=1` and is not a course-zone classification.
- `perception/object_detection/config/object_detection.yaml` mentions three
  stable on/off frames for a different `object_detector_node` contract, but
  the current executable does not declare those parameters, CMake does not
  install the YAML, and neither production nor bag-test launch loads it.
- `pass_car`/`car` bag metadata contains raw scan/image/ultrasonic and, in some
  runs, a declared lane-change topic, but no fixed-entry/exit/pass topic. The
  inspected 2026-08-01 bag has 648 `/object_info` messages and zero
  `/lane_change_state` messages; it also has no zone/complete source.
- Decision: do not wire object evidence to FIXED entry and do not enable a
  partial production path. The existing internal timestamped entry/exit seams
  remain test-only; no ROS topic, threshold, debounce, or production transition
  was added.

### Unverified

- No bag was replayed and no launch, driver, hardware, or motor publisher was
  run. Fixed-zone identity, object-clear/pass semantics, operational
  lane-change completion, and real-vehicle behavior remain UNVERIFIED.
