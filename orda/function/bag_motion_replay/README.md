# bag_motion_replay — 녹화된 구동 명령을 바이트·타이밍 그대로 재발행

bag 에 남은 **구동 명령 스트림**을 내용과 발행 시각 그대로 다시 내보낸다.
`drive_eval` 이 "스택이 스스로 판단하게 하고 채점"하는 도구라면, 이쪽은
"녹화된 주행을 그대로 재현"하는 도구다.

## 조이콘 조향/속도는 기본적으로 재생하지 않는다

이 bag 의 명령 계층은 두 겹이다.

```
/joy ──▶ manual_drive ──▶ /xycar_motor [조향도, 속도] ──▶ vesc ──▶ /commands/servo/position
                                                                  /commands/motor/speed
```

`/joy` 와 `/xycar_motor` 는 **조이콘 조작에서 나온 조향·속도**라서 기본 토픽
집합에서 빠져 있고, `include_topics` 로 직접 지목해도 거부한다
(`allow_joycon:=true` 로만 열린다). 기본값 `topic_set:=actuation` 은 VESC 가
실제로 받아 차를 움직인 서보 위치·eRPM 을 재생한다.

## 쓰는 법

```bash
# bag 이 무엇을 담고 있고 무엇이 재생될지 먼저 본다
ros2 run bag_motion_replay bag_motion_cue info

# cue(재생용 추출본)를 미리 만들어 둔다 — 21 GB 를 한 번만 읽는다
ros2 run bag_motion_replay bag_motion_cue build

# 재생 (실차 토픽으로 나간다)
ros2 launch bag_motion_replay bag_motion_replay.launch.py

# 실차 토픽을 건드리지 않고 재생 + 검증
ros2 launch bag_motion_replay bag_motion_verify.launch.py
ros2 run bag_motion_replay bag_motion_selftest
```

## 무엇을 보장하는가

**내용** — bag 에 기록된 CDR 페이로드를 그대로 `publish()` 에 넘긴다
(`rclpy` 는 `bytes` 를 `publish_raw` 로 보낸다). 역직렬화를 하지 않으므로 float
하나 반올림되지 않고 헤더도 다시 쓰이지 않는다. `verify_node` 가 수신 바이트를
녹화 바이트와 하나하나 대조한다.

**타이밍** — 모든 마감 시각은 재생 시작점 하나를 기준으로 한 절대 오프셋이다.
간격을 하나씩 재는 방식은 늦은 발행이 뒤를 계속 밀지만, 이 방식은 밀리지 않는다.

- `timing_mode:=wall` — 마감 1.5 ms 전까지 자고 나머지는 busy-wait 으로 채운다
  (`pacing.py`). 잔여 오차를 측정해 리포트에 남긴다.
- `timing_mode:=sim` — `/clock` 을 직접 몰아준다. `use_sim_time:=true` 로 띄운
  구독자 입장에서 오차는 **정의상 0 ns** 다.

**완주** — 스케줄을 전부 열거한 뒤 시작하고, 모든 항목이 나갈 때까지 끝나지
않는다. 발행 실패는 재시도하고, 첫 Ctrl-C 는 이유를 찍고 무시한다. 정말 멈추려면
`abort_grace` 초 안에 한 번 더 눌러야 하고, 그때는 정지 명령을 먼저 보낸다.

## 실차에서 쓸 때

`/sensors/servo_position_command` 는 `vesc_driver` 의 **출력**이다. 드라이버가
살아 있는 차에서 같이 발행하면 퍼블리셔가 겹치므로 빼고 돌린다.

```bash
ros2 launch bag_motion_replay bag_motion_replay.launch.py \
  exclude_topics:="['/sensors/servo_position_command']"
```

## 이 bag 에 없는 것

`rosbag2_2026_08_13-13_33_23` 은 수동 주행 기록이라 `/mode_info` · `/lane_info`
가 **없다**. 주행 모드는 재생할 원본 자체가 없으므로 리포트가 "not in bag" 으로
표시하고 넘어간다.

## 테스트

```bash
cd src/orda/function/bag_motion_replay && python3 -m pytest test -q
```
