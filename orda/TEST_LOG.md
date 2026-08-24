# TEST LOG

## 2026-08-25 — bag 로 스택을 돌려 채점 (`rosbag2_2026_08_13-13_33_23`)

`drive_eval` 을 새로 만들어, bag 의 **센서만** 재생해 스택을 돌리고 그 출력(조향·
모드·라바콘 판단)을 같은 bag 의 기록과 대조했다. bag 의 `/joy`·`/xycar_motor`·
`/commands/*` 는 사람이 조이콘으로 만든 값이라 입력으로 재생하지 않고 채점
기준으로만 썼다.

실행 설정: `test_profile:=0 mode:=0` (실전 `module_drive.py` 와 같은 초기화),
전체 200 초, PIDNet 75~90 Hz.

### 확인된 정상 동작

- 3바퀴 완주. 신호등을 0.3/69.7/135.9(좌회전)/177.4 초에 인식하고, 출발 신호는
  `WAIT_GREEN` 이 소비, 나머지 3회를 랩으로 세어 177.4 초에 FINISH.
- 좌회전 신호가 두 번 떴지만 지름길은 한 번만 (`shortcut_lap` 래치 정상).
- 미션 시퀀스가 계약대로. `race_fsm.py` 에 없는 전환 0건.
- 라바콘 진입 지연 +1.37/+1.46 초, 구간 커버리지 82.9/81.7%.

### 고친 것

**PD 미분항이 모드 전환마다 킥한다** (`control.py`)
`reset()` 이 `prev_offset` 을 `0.0` 으로 되돌려, 새 모드의 첫 사이클에서
`diff` 가 오프셋 전체가 됐다. `FIXED_AVOID` 는 kp 0.12·kd 0.35 라 그 한 사이클만
`0.47·e` 가 되고, 28.98 초(LANE_DRIVE→FIXED_AVOID 전환 순간) offset −289 px 가
**−135.8°** 로 나갔다. 다음 사이클부터는 −34.7° 였다. `prev_offset` 을 `None`
(=모른다)으로 두고 첫 사이클 `diff` 를 0 으로 바꿨다. 실측 최대가 135.8° → 32.2°.

**`FIXED_AVOID` 만 조향 상한이 없었다** (`control.py`)
`LANE_DRIVE` 100°, `CONE_DRIVE` 45° 와 달리 상한이 없어 큰 오프셋이 그대로 곱해져
나갈 수 있었다. 차선 주행과 같은 봉투(100°)로 제한했다. 킥을 고친 뒤로는 이
상한에 닿지 않는다 — 안전망이지 튜닝 지점이 아니다.

**라바콘 통로 폭을 파라미터로 노출** (`rubbercone.cpp`, `module_drive_bag_test.py`)
97.35 초에 라바콘이 없는 곳에서 `CONE_DRIVE` 로 들어갔다. 오른쪽 연석과 왼쪽
벽이 `entryGeometryValid`(양쪽 2점 이상)를 그대로 만족했고 confidence 도 88 로
정상 구간과 같아, 노드 인터페이스만으로는 구분되지 않는다. `/scan` 실측 폭만
갈린다.

| 구간 | 중앙값 | p90 | 최대 |
|---|---|---|---|
| 실제 라바콘 #1 | 0.83 m | 1.06 | 1.26 |
| 실제 라바콘 #2 | 0.83 m | 1.02 | 1.25 |
| 오진입 @97 s | 1.15 m | 1.32 | 1.32 |
| 오진입 @159 s | 1.28 m | 1.37 | 1.46 |

`max_corridor_width:=1.10` 으로 돌리면 오진입이 사라지고 실제 구간 2개는 그대로
진입한다(실측). 다만 근거가 이 bag 하나뿐이고 본선 배치가 다르므로 **기본값은
1.40 그대로** 두고 파라미터로만 뺐다. 본선 게이트 폭을 실측한 뒤 내리면 된다.

### 측정해서 아니라고 판명된 것

- **라바콘 구간 조향이 45° 제한에 걸린다** — 아니다. 실측 최대 44.0°, 44.9° 이상
  0.0%. 제한에 닿지도 않는다. `amp(cand/ref)=0.31` 이 낮게 나온 것은 사람 조향이
  이봉분포(중앙값 1.5°, 27~33% 가 풀락 100°)라 RMS 가 스파이크에 끌려간 탓이다.
  `drive_eval` 의 조향 크기 판정을 비율에서 **90 분위 절대값 5°** 로 바꿨다.
- **라바콘 이탈이 2.7 초 늦다 → FSM 문제** — 아니다. FSM 반응은 0.00~0.01 초이고
  2.7 초는 전부 `rubbercone_node` 가 세션 종료를 결정하는 시간이다. 한쪽 경계가
  계속 보이는 동안은 `path_valid` 가 유지되므로 노드가 틀린 것도 아니다.
- **상관계수 0.35 기준이 사람 조향 상대로 가혹하다** — 아니다. 같은 이봉분포
  기준에 대해 위상만 맞는 매끄러운 추종기는 0.594, 기준을 저역통과한 것만으로도
  0.456 이 나온다. 스택의 0.33 은 실제 미달이라 기준을 그대로 뒀다.
- **`/mode_info` 진동 → 추월 진입/이탈 조건 중첩** — 아니다(아래 미해결 참고).
  이 가설로 넣었던 `overtake_reentry_cooldown_s` 는 되돌렸다.

### 미해결

**`/mode_info` 가 FSM 상태와 어긋나게 발행된다.**
`OVERTAKE` 경계에서 15회(0.53 초), `SHORTCUT` 경계에서 25회(0.73 초) 모드 값이
번갈아 나갔다. 같은 구간에서 `main_node` 는 `FSM ... -> ...` 를 **한 번만** 찍었고
자체 `runtime mode=` 진단도 옛 모드를 유지했다. `/mode_info` 를 구독하는
`lane_node`·`pidnet_inference_node` 가 일어나지 않은 모드 전환에 반응한다.

발행 시점 진단(`mode_publish_diagnostic:=true`, 기본 꺼짐)을 `main.py` 에 넣고
10회 재실행했다. **한 번도 재현되지 않았고**, 매 실행 13~15건이 전부 정상 미션
전환이었다. 그 로그로 배제된 것:

| 가설 | 근거 |
|---|---|
| FSM/runtime/node 객체 중복 | 세 `id()` 가 전 구간 동일 |
| 멀티스레드 경합 | `thread` id 동일 (`rclpy.spin` 은 단일 스레드) |
| 같은 사이클 중복 발행 | `cycle` 번호가 수백~수천 이격 |
| `/clock` 밀린 주기 몰아서 실행 | `--clock` 200→20 Hz 로 낮추니 제어 루프도 50→20 Hz 로 같이 떨어졌다(사이클 7750→3047). rclpy 는 클럭 틱당 한 번만 돈다 |
| 전송·녹화 단계에서 생긴 값 변화 | 9개 실행에서 **발행 로그 건수 = 녹화된 값 변화 건수**로 정확히 일치. 녹화는 발행된 것을 그대로 담는다 |

마지막 항목 때문에, 진동이 났던 두 실행에서도 **노드가 실제로 그 값들을
발행했다**는 것은 확정이다. 남은 것은 그 순간 `self.runtime.fsm.state` 가
무엇이었는지 뿐이고, 그건 진단이 켜진 채로 재현될 때 한 줄로 나온다.

재현 조건 단서: 진동이 난 두 실행은 재생 배속이 1.0083 / 1.0161 이었고, 진단을
켠 10회는 대부분 1.0000 이었다. 다만 배속만으로는 충분조건이 아니다(1.0169 였던
`final_run` 은 정상). CPU 부하(12코어 중 6개 점유) 3회로도 유발되지 않았다 —
이 데스크톱은 여유가 너무 많다. **Jetson 연습 주행 때 진단을 켜 두는 것이 가장
현실적인 포착 경로다.**

**조향 위상.** 방향 일치 73~74%, 상관 0.33, 지연 +0.10 초. 사람의 주행선과 다른
것 자체는 문제가 아니지만, 위 합성 검증 기준(0.456~0.594)에는 못 미친다.

### 하네스 쪽에서 발견해 고친 것

- 이전 실행 노드가 남아 `main_node` 가 6개 겹친 채 40초 녹화에 모드 전환 2508회
  같은 값이 나왔다. 러너가 노드를 이름으로 확실히 종료하고 워밍업 후 개수를 세어
  1개가 아니면 녹화를 거부한다. 채점기도 `/mode_info` 주기가 50 Hz 제어 주기를
  넘으면 `run capture` 를 FAIL 로 놓고 나머지 수치를 믿지 말라고 먼저 말한다.
- 녹화를 SIGKILL 로 끊어 mcap 이 마무리되지 않던 회귀를 고치고, 채점 전 run bag
  판독 검사를 넣었다.
- 연습 주행 bag 이라 3바퀴 뒤에도 사람이 계속 돈다. 3랩을 채운 뒤의 0 출력은
  완주로 인정하고 채점 창을 거기서 끊는다 — 안 그러면 정상 완주가 "주행 포기" 로
  읽힌다.

### 재현

```bash
ros2 run drive_eval run_stack_eval.sh
EXTRA_LAUNCH_ARGS="rubbercone_max_corridor_width:=1.10" ros2 run drive_eval run_stack_eval.sh
EXTRA_LAUNCH_ARGS="mode_publish_diagnostic:=true"       ros2 run drive_eval run_stack_eval.sh
```

PIDNet 세그멘테이션 결과를 보려면 (셋 다 이 환경에서 바로 뜬다):

```bash
# 주행 스택 없이 세그멘테이션만 — 가장 가볍다
ros2 launch main module_pidnet_bag_preview.py bag_path:=~/my_rosbag/rosbag2_2026_08_13-13_33_23

# 전체 스택을 돌리면서 창도 같이 (창을 켜면 추론이 느려져 /lane_offset 주기가 떨어진다)
EXTRA_LAUNCH_ARGS="pidnet_show_visualization:=true" ros2 run drive_eval run_stack_eval.sh

# 창 없이, 노드 CPU 를 쓰지 않고 — 채점 실행 중에는 이쪽
ros2 run rqt_image_view rqt_image_view /pidnet_overlay
```

`module_drive_bag_test.py` 의 `mode` 기본값은 `1`(LANE_DRIVE)이라 그냥 띄우면
출발 신호가 1랩으로 세어져 FINISH 가 한 바퀴 일찍 뜬다. 실전 `module_drive.py`
기본값은 `0` 이고, `drive_eval` 러너도 `MODE=0` 을 기본으로 둔다.

---

## 2026-08-15 — PPT interface and FSM contract migration

### Team-approved decisions

- Removed `INIT`; startup readiness is part of `WAIT_GREEN`.
- Removed `REJOIN`; a fresh rubber-cone end edge returns directly to
  `LANE_DRIVE`.
- Official `/mode_info` is `std_msgs/msg/Int16` with `0=WAIT_GREEN`,
  `1=LANE_DRIVE`, `2=CONE_DRIVE`, `3=FIXED_AVOID`, `4=OVERTAKE`, and
  `5=SHORTCUT`.
- Internal `FINISH` and `STOP` have no assigned external value. Main suppresses
  `/mode_info` while either state is active instead of publishing `0`.
- Official `/lane_info` is `std_msgs/msg/Int16`: `1=lane 1`, `2=lane 2`,
  `3=center`.
- Object Detection publishes official `/object_info`
  `[traffic_signal, fixed_vehicle_lane, moving_vehicle_lane]`. Fixed and moving
  representatives occupy independent slots and can survive the same frame.
- Rubbercone publishes official `/rubbercone_offset [offset, end_flag]`.
- Object Detection and Rubbercone also publish their detailed internal payloads
  directly in the same calculation cycle.

### PPT-external implementation topics retained

| Topic | Payload | Reason |
|---|---|---|
| `/traffic_detection` | `Int32` traffic code | traffic YOLO → object aggregation |
| `/object_yolo` | `Float32MultiArray`, fixed and moving 10-field slots | preserve both object categories before lane fusion |
| `/object_info_raw` | existing detailed 12 fields | distance/box/type evidence used by validated Main logic |
| `/rubbercone_info` | `[offset, end_flag, confidence]` | confidence required by the existing cone-entry debounce |
| `/internal/lane_command` | `[legacy_lane_mode, internal_lane]` | isolate the unchanged lane detector's old array input from official `/mode_info` |
| `/lane_fit` | `[m, b]` | object-to-lane fusion input |
| `/lane_change_state` | `[changing, success]` | avoidance lane-change completion feedback |
| `/lane_position` | `Int16` | measured ego lane for side-clearance completion |
| `/lane_valid` | `Bool` | legacy lane-detector output; Main no longer subscribes after `REJOIN` removal |
| `/rubbercone_reset` | `Empty` | reset one detector session on cone entry |
| `/road_surface` | `Int32` road label | shortcut-exit evidence |

### Verification in the scratch clone

- Pure Main/FSM regression excluding ROS-runtime import tests: `418 passed`.
- Object YOLO preprocessing/dual-slot/parameter tests: `10 passed`.
- `compileall`, shell syntax, and `git diff --check`: pass.
- ROS entity tests, C++ package builds, full `colcon test`, bag, and real vehicle:
  pending in the ROS2 Humble WSL workspace.
- Known input blocker: the checked-in object model is fixed-only. The YAML fixes
  `fixed_class_ids: [0]` and deliberately leaves `moving_class_ids` unset. A
  team-provided moving-object model and exact class-ID mapping are required
  before `/object_info[2]` can be validated on real data.

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
| Lane-change complete | Declared `/lane_change_state [changing, success]` | Intended offset spike/settle success pulse | At audit time `updateLaneChangeState()` had no call site; inspected bags record zero messages |
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

## 2026-08-05 — Lane-change state feedback activation

### No-object checkpoint

- Committed the preceding five-file absent-object correction as
  `f9a1b1773cc7fb27421262d091385e70d9ca5a65` with message
  `fix(kmu-real): normalize absent object observations`.
- Before commit: adapter `36 passed`, main callback mocks `28 passed`, full
  Python regression `357 passed`, compileall and `git diff --check` passed.
- The checkpoint was not pushed and its worktree was clean before this
  lane-change work began.

### Actual lane callback and prior blocker

- Package: `lane_detection` (`ament_cmake`). The node subscribes to
  `/resized_image` with SensorData BestEffort QoS and `/mode_info [mode, lane]`
  with KeepLast(10), BestEffort, Volatile QoS.
- Each decoded non-empty image runs yellow-lane preprocessing, BEV/noise
  suppression, fitting, and offset calculation. A failed fit explicitly sets
  `valid=false` but reuses the previous fit/offset for legacy lane-control
  output.
- `/lane_change_state` was already declared as an `Int32MultiArray` publisher
  with KeepLast(10), BestEffort, Volatile QoS. Its
  `updateLaneChangeState()` publisher function had no call site, so the node
  could appear in the ROS graph without emitting feedback messages.

### Minimal wiring and state contract

- Added one call at the end of the successfully decoded image-processing path,
  after the current frame's fitting validity and offset are known. There is no
  timer and no stale geometry is accepted as completion evidence.
- Preserved the configured algorithm and values: spike 400 px, straight settle
  50 px, curve settle 300 px, eight settle frames, curve split `|m| >= 0.1`.
- A small ROS-independent tracker treats only mode 5 with target 0 or 1 as a
  valid action. Repeated mode-5 messages for the same target retain one command
  epoch; mode exit, invalid target, or target change resets/rearms safely.
- A valid action publishes `[1, 0]` until spike plus settle completes, then
  publishes one `[1, 1]` edge. The same action cannot generate another edge;
  the next processed frame is success-low. Invalid fitting resets progress but
  cannot produce success. Idle/completed/cancelled actions publish `[0, 0]`.
- No lane fitting, BEV, steering, topic fields, numeric mode meaning, or
  threshold was changed.

### Verification

- BUILD PASS: `colcon build --packages-select lane_detection --symlink-install`.
- UNIT/SYNTHETIC PASS: four GTests cover idle, incomplete action, single spike,
  settle completion, one-shot behavior, cancel, a second action, invalid
  target/fit, and target-change reset.
- STATIC PASS: `cppcheck`, CMake lint, XML lint, and uncrustify for the two new
  files. Full package `colcon test` still reports pre-existing whole-file
  uncrustify debt in four legacy sources, including the file-wide style of
  `lane_detection.cpp`; the new GTest itself passes.
- MOCK PASS: existing Python object/lane-change adapters and main callbacks,
  `64 passed`.
- Full Python regression: `357 passed`; compileall and `git diff --check`
  passed.
- BAG UNVERIFIED: available recordings are not labeled with a deterministic
  mode-5 spike/settle ground truth, so no bag was replayed and the old zero
  message count was not treated as a pass.
- REAL-VEHICLE UNVERIFIED: no launch, camera/hardware driver, motor node, or
  `/xycar_motor` publisher was run.
- Fixed-zone production entry/exit remains disabled. Lane-change feedback is
  action feedback only and is not used as fixed-zone evidence.
