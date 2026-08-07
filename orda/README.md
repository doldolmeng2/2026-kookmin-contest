# 2026 제9회 국민대학교 자율주행 경진대회

본선: 2026년 8월 25일 (화) · 국민대 자율주행 스튜디오
연습 주행: 2026년 7월 1일 ~ 8월 21일

## 디렉터리 생성 및 클론
```bash
mkdir -p xycar_ws/src
cd xycar_ws/src
git clone https://github.com/doldolmeng2/2026-kookmin-contest.git .
```

---

## 주행 미션 시퀀스

트랙을 **총 3바퀴** 주행하며, 1바퀴와 2·3바퀴의 시작 방식이 다르다.

**1바퀴** — 정지 상태에서 신호등을 보고 출발
```
① 신호등 인식 출발 → ② 차선 주행 → ③ 라바콘 주행 → ④ 고정장애물 회피
→ ⑤ 방해차량 추월 → ⑥ S커브 구간 → ⑦ 결승선 통과
```

**2·3바퀴 — 좌회전 신호가 아닐 때** (직진)
```
① 신호등 경로 선택 → ② 차선 주행 → ③ 라바콘 주행 → ④ 고정장애물 회피
→ ⑤ 방해차량 추월 → ⑥ S커브 구간 → ⑦ 결승선 통과
```

**2·3바퀴 — 좌회전 신호일 때** (지름길, 경주 중 1회)
```
① 신호등 경로 선택 → ② 지름길 통과 → ③ S커브 구간 → ④ 결승선 통과
```

- **2·3바퀴에서도 신호등이 정지 신호(`1`)면 정지한다.** 신호 준수는 바퀴와 무관하다.
  다만 `WAIT_GREEN` 모드는 **출발 시 한 번만 사용하고 재진입하지 않는다.**
  2·3바퀴의 정지는 모드 전환 없이 **제어 계층의 정지 오버라이드**로 처리한다
  (`/traffic_detection == 1` → `speed = 0`, 신호가 `2`/`3` 으로 바뀌면 즉시 해제).
- **S커브 구간은 모든 바퀴가 통과한다.** 지름길을 타든 안 타든 결승선 직전에 반드시 지난다.
  별도 모드를 두지 않고 `LANE_DRIVE` 로 처리하며, 감속은 `control.py` 의 조향각 기반
  감속 로직에 맡긴다.
- **2바퀴째 또는 3바퀴째**에 4구 신호등에서 좌회전 신호가 나온다(어느 바퀴인지 랜덤).
  좌회전 신호는 **전체 경주에서 단 한 번만** 나오고, 지름길도 그때 한 번만 쓸 수 있다.
- 따라서 **좌회전 신호를 인식하면 반드시 좌회전하여 지름길로 주행한다.**
  직좌 동시신호라 직진도 규정상 허용되지만, 여기서 직진하면 지름길 기회가 사라져
  주행 거리가 그대로 길어진다. 직진/좌회전을 저울질하는 분기를 두지 않는다.
- **1차선/2차선 구분이 없다.** 양쪽 실선만 벗어나지 않으면 되고,
  가운데 점선을 두 바퀴 사이에 두고 주행하는 것도 허용된다.

### 지름길은 좌측 구간 전체를 건너뛴다

트랙은 좌측 루프와 우측 루프를 중앙 세로 도로가 잇는 형태다. 신호등은 중앙 도로가
상단에 닿는 교차로에 있고, **라바콘·고정장애물은 좌측 루프에 배치**되어 있다.

```
직진(정규 경로) : 신호등 ─▶ 라바콘 ─▶ 고정장애물 ─▶ 방해차량 ─▶ S커브 ─▶ 결승선
좌회전(지름길)  : 신호등 ─▶ 중앙 도로 ───────────────────────▶ S커브 ─▶ 결승선
```

따라서 **지름길을 타는 바퀴에는 라바콘·고정장애물·방해차량 구간을 모두 건너뛴다.**
라바콘은 3바퀴 중 2바퀴에서만 만난다. S커브와 결승선은 두 경로가 합류한 뒤라 항상 통과한다.

| | 1바퀴 | 2바퀴 | 3바퀴 |
|---|---|---|---|
| 좌회전 신호가 2바퀴째인 경우 | 라바콘·고정장애물·방해차량 | **지름길** | 라바콘·고정장애물·방해차량 |
| 좌회전 신호가 3바퀴째인 경우 | 라바콘·고정장애물·방해차량 | 라바콘·고정장애물·방해차량 | **지름길** |

> 방해차량은 고정 배치물이 아니라 **주행 중인 이동체**이므로 조우 위치가 완전히 고정되지는
> 않는다. `OVERTAKE` 전이 자체는 `LANE_DRIVE` 에서 항상 활성이므로 별도 조치는 필요 없다.

### 코드가 지켜야 하는 규정 제약

| 항목 | 제한 | 처리 |
|---|---|---|
| 주행시간 시작 | 파란불로 바뀐 순간 | 2026-07-29 「제9회 경주 진행 방법」 p.17 |
| 총 주행시간 | 3바퀴 4분 초과 | 실격, 같은 자료 p.37 |
| 라바콘 구간 | 1분 내 미통과 | 실격, 같은 자료 p.25 |
| 주행 정지 | 정지 후 1분 내 미재개 | 실격 |
| 라바콘 충돌 | 개당 | 벌초 3초 |
| 차량 터치 | 1회당 (한 바퀴 15회 초과 시 실격) | 벌초 5초 |
| 추돌 / 피추돌 | 1회당 | 벌초 10초 |
| 빨간불 출발 | 1회당 | 벌초 10초 |

> 정지 상태가 곧 실격으로 이어지므로, 안전 정지는 **복귀 가능한 감속**을 우선하고
> 완전 정지는 최후 수단으로만 사용한다.

---

## 소스코드 파일 구조

```
src/
├── orda/
│   ├── perception/                 # 원시 인지 (미션 중립)
│   │    ├── image_resize           # 카메라 영상 리사이즈 (640×360)
│   │    ├── traffic_light          # 4구 신호등 상태 판별
│   │    └── object_detection       # LiDAR + YOLO 차량 검출 (정지/이동 분류)
│   │
│   ├── driving/                    # 조향 오프셋 생성
│   │    ├── lane_detection         # BEV 기반 차선 검출
│   │    └── rubbercone             # LiDAR 기반 라바콘 오프셋 계산
│   │
│   ├── main/                       # 미션 상태 머신 + 모터 제어
│   │
│   └── function/                   # 보조 도구
│        ├── manual_drive           # Xbox 컨트롤러 수동 주행 / bag 수집
│        └── sensors_viewer         # 센서 데이터 시각화
│
├── track_drive/                    # (Xytron 제공) 예제 주행 패키지
├── xycar_application/              # (Xytron 제공) 예제 애플리케이션
├── xycar_device/                   # (Xytron 제공) 센서·모터 드라이버, xycar_msgs
└── xycar_simulator/                # (Xytron 제공) 시뮬레이터
```

`perception` / `driving` / `function` 은 **폴더 계층일 뿐 ROS 패키지가 아니다.**
colcon 은 `package.xml` 을 재귀로 탐색하므로 폴더 깊이는 빌드에 영향을 주지 않는다.

### main 패키지 내부

```
main/main/
├── main.py                 # [노드] ROS 배선 전담 (구독/발행/타이머)
├── control.py              # 모드별 조향각·속도 계산
├── control_selector.py     # 제어 소스 중재 + 입력 신선도 검사
├── race_fsm.py             # 미션 상태 머신
├── race_context.py         # 랩 수·지름길 사용 여부 등 경주 전역 상태
├── mission_observation.py  # 인지 입력 스냅샷
├── safety_monitor.py       # 안전 판정 (입력 유실, 시간 제한)
├── cone_entry.py           # 라바콘 진입 디바운스
├── shortcut_turn.py        # 지름길 좌회전 궤적 (IMU yaw 기반)
└── bag_replay.py           # 오프라인 bag 리플레이 검증
```

**노드 파일은 `main.py` 하나뿐이다.** 나머지는 `rclpy` 를 import 하지 않는 순수 로직
모듈이며, 토픽을 직접 다루지 않고 `main.py` 가 import 해서 사용한다.
이 규칙 덕분에 FSM·제어 로직을 실차 없이 단위 테스트와 bag 리플레이로 검증할 수 있다.

---

## 노드 및 토픽 구조

토픽 표기: **[유지]** 작년과 동일 · **[변경]** 이름 유지, 페이로드 재정의 · **[신규]** 신설

```
[Xycar HW]
  xycar_cam       → /image_raw          (sensor_msgs/Image)
  xycar_lidar     → /scan               (sensor_msgs/LaserScan)
  xycar_imu       → /imu                (sensor_msgs/Imu)
  xycar_ultrasonic→ /xycar_ultrasonic   (std_msgs/Int32MultiArray)
  joy_node        → /joy                (sensor_msgs/Joy)

[전처리]
  resize_node
    sub: /image_raw
    pub: /resized_image                 (sensor_msgs/Image, 640×360)      [유지]

[인지]
  traffic_node
    sub: /resized_image
    pub: /traffic_detection             (std_msgs/Int32)                  [변경]
         0 = 인식 못함
         1 = 정지   (빨강, 주황)
         2 = 직진   (녹색)
         3 = 좌회전 (녹색 + 좌회전 화살표 동시 점등 → 지름길 진입)

  rubbercone_node
    sub: /scan
    pub: /rubbercone_info               (std_msgs/Int32MultiArray)        [유지]
         [offset, end_flag, confidence]
         confidence: 경로 추정 신뢰도 (0~100)

  lane_node
    sub: /resized_image, /mode_info
    pub: /lane_offset                   (std_msgs/Int16, 픽셀 오프셋)     [유지]
         /lane_fit                      (std_msgs/Float32MultiArray, [m, b]) [유지]
         /lane_change_state             (std_msgs/Int32MultiArray)        [유지]
         [변경중, 성공여부]
         /lane_position                 (std_msgs/Int16)                  [신규]
         -1 = 미확정, 0 = 1차선, 1 = 2차선
         노란 중앙선의 화면상 좌우 위치로 판정한 **실측** 현재 차선

  object_node
    sub: /scan, /resized_image, /lane_fit
    pub: /object_info                   (std_msgs/Float32MultiArray, 11 필드) [변경]
         [exists, min_dist, angle, span, cluster_size,
          box_size, box_cx, box_cy, dx, car_lane, is_moving]
         car_lane : 0=중앙, 1=왼쪽, 2=오른쪽
         is_moving: 0=고정장애물, 1=방해차량      ← 11번째 필드 신설

[제어]
  main_node
    sub: /rubbercone_info, /lane_offset, /lane_change_state, /lane_position,
         /object_info, /traffic_detection, /imu, /joy, /xycar_ultrasonic
    pub: /xycar_motor                   (std_msgs/Float32MultiArray)      [유지]
         [angle, speed]
         /mode_info                     (std_msgs/Int32MultiArray)        [현재 호환]
         [legacy_mode_code, lane]
```

### 인터페이스 변경 사유

| 토픽 | 변경 내용 | 사유 |
|---|---|---|
| `/traffic_detection` | `Bool` → `Int32` (4상태) | 4구 신호등의 좌회전 화살표를 구분해야 지름길 판단이 가능 |
| `/object_info` | `is_moving` 필드 추가 | 고정장애물 회피와 방해차량 추월은 **규칙이 다른 별개 미션** (추월 중에는 차선 이탈이 허용됨) |
| `/mode_info` | 현재 `[legacy_mode_code, lane]` 유지 | 실제 소비자인 `lane_detection.cpp`가 아직 3=차선주행, 5=차선변경 계약을 사용한다. 4필드 신규 계약(`[mode, lap, shortcut_used, lane_target]`)은 소비자 변경 전까지 발행하지 않는다. |
| `/lane_position` | 신설 | `/mode_info` 의 차선 값은 **명령**(가고 싶은 차선)이라 차선 변경 도중에도 목표값을 가리킨다. 방해차량이 내 차선에 있는지 판단하려면 **지금 실제로 어디 있는지**가 필요하므로 실측 채널을 분리 |

`car_lane` 과 `lane_target` 은 **같은 정수 규약(0=중앙, 1=왼쪽, 2=오른쪽)** 을 쓴다.
방해차량이 있는 쪽의 반대편을 추월 방향으로 그대로 뒤집어 쓸 수 있게 하기 위함이다.

> `Int32MultiArray` 의 인덱스 의미는 이 문서뿐 아니라 **코드 내 상수로도 정의한다.**
> 문서에만 존재하는 인덱스 규약은 배선 실수의 주된 원인이 된다.

#### 정지 신호(`1`) 오검출 주의

`/traffic_detection == 1` 은 **주행 중에도 즉시 정지**를 유발하므로 오검출 비용이 가장 크다.
트랙 중간에서 잘못 정지하면 재개 지연으로 실격까지 이어질 수 있다.

- 정지 신호를 **빨강과 주황으로 함께 정의했는데, 라바콘이 주황색이다.** 라바콘 구간에서
  신호등이 아닌 물체를 정지 신호로 읽지 않도록 해야 한다.
- 색상 블롭만으로 판정하지 말고 **4구 신호등 하우징을 먼저 찾은 뒤 그 안에서 색을 판정**하거나,
  ROI 를 신호등이 나타나는 화면 상단으로 제한한다.
- 좌회전 신호(`3`)는 미검출을 줄이는 방향으로, 정지 신호(`1`)는 오검출을 줄이는 방향으로
  임계값을 잡는다. **두 신호의 튜닝 방향이 반대**라는 점에 유의한다.

---

## 차선 주행 정책

기본은 **중앙 주행**(가운데 점선을 두 바퀴 사이에 두고 주행)이다. 양쪽 실선에서
가장 멀어 차선 이탈 위험이 최소이고, 곡선 구간 코너 안쪽에 설치되는 방해 장애물과의
간격도 확보된다.

**단, 장애물이 나올 수 있는 구간에서는 중앙 주행이 오히려 불리하다.** 중앙 주행은 두 차선을
동시에 점유하므로 장애물이 어느 차선에 있든 충돌 대상이 된다. 따라서 장애물 구간에서는
**미리 한쪽 차선을 확정해서 진입한다.** 장애물이 반대 차선이면 아무 조작 없이 통과하고,
같은 차선이면 한 번의 완전한 차선 변경으로 회피한다.

### 구간별 목표 차선

| 구간 | 모드 | `lane_target` | 근거 |
|---|---|---|---|
| 신호등 → 라바콘 진입 | `LANE_DRIVE` | 0 (중앙) | 실선 이탈 위험 최소화 |
| 라바콘 | `CONE_DRIVE` | — | 라이다 경로를 따름 |
| 라바콘 종료 → 차선 재합류 | `REJOIN` → `LANE_DRIVE` | 0 (중앙) | fresh 차선 유효성 확인 뒤 정상 차선 주행 복귀 |
| 고정장애물 구간 | `FIXED_AVOID` | 미확정 | 별도의 고정장애물 진입 event 계약이 아직 없음 |
| 고정장애물 통과 → 추월 완료 | `OVERTAKE` | **직전 차선 유지** | 불필요한 차선 변경을 줄임. 방해차량 차선은 어차피 랜덤 |
| 추월 완료 → 결승선 (S커브 포함) | `LANE_DRIVE` | 0 (중앙) | S커브 코너링에 유리 |

- 고정장애물이 2차선에 있으면 1차선으로 회피하고, **그대로 1차선에서 방해차량 구간을 시작한다.**
  회피 후 2차선으로 되돌아오지 않는다.
- 추월 방향은 시작 차선과 무관하게 **방해차량이 있는 차선의 반대편**으로 결정한다
  (같은 쪽으로 추월하면 차선 이탈로 간주됨, 규정 p.33).

`main_node` 가 `/mode_info` 의 `lane_target` 으로 목표를 지시하고,
`lane_node` 가 `/lane_change_state` 로 이동 진행·완료를 보고한다.

> ⚠️ 방해차량은 **1차선과 2차선을 오가며 주행한다**(규정 p.32). `car_lane` 은 한 번 읽고
> 마는 값이 아니라 추월 직전까지 계속 갱신해야 하며, 접근 중에 방해차량이 차선을 바꾸면
> 추월 방향도 다시 결정해야 한다.

---

## 차선 인식 안정화 (lane_detection)

노란 중앙선에 피팅한 직선이 주행 중 좌우로 크게 튀는 문제를 잡기 위한 수정이다.

| 수정 | 내용 |
|---|---|
| `keepLongestComponent()` | BEV 이진 마스크에서 `connectedComponentsWithStats` 로 **가장 큰 연결 성분만 남긴다.** 반사광·노면 얼룩 같은 작은 덩어리가 피팅에 끌려들어가지 않는다 |
| `ref_hist_sigma_ratio` : 999.0 → **0.35** | 직전 프레임 차선 위치 주변에 가우시안 가중을 준다. 999.0 은 사실상 가중치 없음(균등)이라 히스토그램 피크가 매 프레임 엉뚱한 곳으로 점프했다 |
| `ref_hist_min_weight` : 1.0 → **0.5** | 위 가중의 하한. 0.5 라서 멀리 있는 후보도 완전히 죽지는 않아 **차선 변경 같은 실제 큰 이동은 따라간다** |
| `sliding_window_minpix` : 5 → **15** | 윈도 재중심화 최소 픽셀 수. 5 는 노이즈 몇 개만으로 윈도가 끌려갔다 |

> 시도했다가 **되돌린** 것: 오프셋 EMA 평활, 근거리 픽셀 가중, 직전 기울기와의
> 방향 일치 가중. 셋 다 피팅을 더 나쁘게 만들었다. 특히 근거리 가중은 오해였는데,
> 주행 중에는 모션 시차 때문에 **가까운 픽셀이 오히려 더 뭉개진다.**

### 실측 차선 판정

노란 중앙선의 화면상 x 위치 비율로 현재 차선을 정하고 `/lane_position` 으로 발행한다.

- `classifyLaneFromRatio()` — 화면 중앙 기준 **±0.05 데드존**. 그 안이면 `-1`(미확정)을
  반환해 애매한 프레임에서 억지 판정을 하지 않는다.
- `updateAndPublishDetectedLane()` — 같은 판정이 **5 프레임 연속**일 때만 값을 바꾼다
  (디바운스). 한 프레임 튐으로 차선 인식이 뒤집히는 것을 막는다.

`debug_lane_view` (기본 `false`) 로 BEV·윈도 시각화 창을 켜고 끌 수 있다.
실차에서는 반드시 `false` — imshow 가 인지 콜백을 지연시킨다.

---

## 방해차량 충돌 대응

인지(YOLO 검출·차선 피팅)는 정상인데 **회피 조향이 너무 늦게 나가 앞차를 들이받는**
문제가 있었다. bag 재생으로 계측한 결과 원인은 두 가지였다.

### 1. 박스 선택 기준: 신뢰도 → 면적 (근본 원인)

`object_detection` 은 NMS 통과 박스 중 **신뢰도가 가장 높은 것**을 골라 `/object_info` 로
내보내고 있었다. 차선마다 방해차량이 한 대씩 있으면 두 대가 동시에 검출되는데,
신뢰도는 프레임마다 엎치락뒤치락하므로 **바로 앞 차와 멀리 있는 차가 번갈아 선택**된다.
그 결과 `car_lane` 이 계속 뒤집히고, `box_size` 도 큰 값과 작은 값을 오가서
차선 변경 트리거가 계속 미뤄졌다.

→ **면적이 가장 큰 박스(= 가장 가까운 차)** 를 고르도록 변경했다.
충돌 위험을 만드는 것은 언제나 제일 가까운 차이므로 판단 대상이 흔들리지 않는다.

계측값 (bag `rosbag2_car_5`):

| | 변경 전 | 변경 후 |
|---|---|---|
| 차선변경 트리거 시점의 `box_size` 평균 | 13,540 px² | **2,096 px²** |
| `car_lane` 뒤집힘 (접근 구간) | 8 회 | 0 회 |

트리거 박스가 작을수록 **멀리서 미리 피한 것**이다. 관측된 최대 박스가 27,258 px²
(= 눈앞) 이므로, 변경 전에는 사실상 코앞에서야 조향이 시작되고 있었다.

### 2. 같은 차선일 때만 회피 (`is_obstacle_in_ego_lane`)

방해차량이 옆 차선에 있는데도 차선을 바꾸면 불필요한 이탈 위험만 커진다.
FSM 이 차선 변경을 걸기 전에 아래 게이트를 통과하도록 했다.

```
obstacle_lane = car_lane 을 0/1 로 환산      (/object_info)
ego_lane      = /lane_position (실측)         ← 없으면 lane_target 으로 폴백
차선 변경은 obstacle_lane == ego_lane 일 때만
```

`car_lane` 이 0(중앙)·미확정이면 **보수적으로 "내 차선에 있다"고 본다.**
회피를 놓치는 쪽이 불필요하게 피하는 쪽보다 비용이 크기 때문이다.

> 별도의 긴급 회피·후진 같은 안전장치는 넣지 않았다. 시도해 봤으나 방금 지나친 차가
> 여전히 크게 보여 회피가 즉시 재발동하며 진동했고(트리거 4회 → 74회),
> 위 두 가지 수정만으로 충돌이 해소되어 불필요했다.

### 조향 관련 보조 수정

- `control.py` 에 `MAX_ANGLE = 100.0` 클램프 추가 — PD 출력이 물리 조향 범위를
  넘어 튀는 것을 막는다.
- `control.reset(offset)` 이 `prev_offset` 을 **현재 오프셋으로 시드**한다.
  모드 전환 직후 미분항이 `0 - offset` 으로 계산되어 한 프레임 급조향이 나가던 문제.

---

## 주행 상태 머신 (main_node)

내부 상태는 문자열 값의 `race_fsm.Mode` 로 정의하며 외부 숫자와 직접 동일시하지
않는다. 현재 `/mode_info`의 유일한 실제 subscriber인 `lane_detection.cpp`가 legacy
숫자를 사용하므로 `main.mode_info`의 명시적 adapter를 거쳐 아래처럼 발행한다.
미확정 상태는 새 숫자를 만들지 않고 외부 STOP 코드 0을 발행한다.

| 내부 모드 | 현재 외부 값 | 설명 | 전이 조건 |
|---|---:|---|---|
| `INIT` | 0 (STOP) | 입력 대기 | 필수 입력 수신 |
| `WAIT_GREEN` | 0 (STOP) | 신호등 앞 정지, 출발 대기 | `/traffic_detection` Bool 디바운스 |
| `LANE_DRIVE` | 3 | 차선 주행 (기본 중앙 주행) | 아래 분기 참조 |
| `CONE_DRIVE` | 1 | 라바콘 구간 주행 | fresh `end_flag` 0→1 |
| `REJOIN` | 2 | 라바콘 종료 후 차선 복귀 대기 | 명시적 차선 유효성 입력 |
| `FIXED_AVOID` | 0 (STOP) | 고정장애물 구간, runtime 도달 불가 | 진입·종료 event 계약 모두 미확정 |
| `OVERTAKE` | 0 (STOP) | 방해차량 구간, 외부 제어 미확정 | 추월 완료 |
| `SHORTCUT` | 0 (STOP) | 지름길, 외부 제어 미확정 | 목표 yaw 도달 |
| `FINISH` | 0 (STOP) | 3바퀴 완료 | — |
| `STOP` | 0 (STOP) | 안전 정지 | — |

```
INIT ──(입력 준비)──▶ WAIT_GREEN ──(신호 2 = 직진)──▶ LANE_DRIVE   [시간 측정 시작]

[확정된 라바콘 흐름]
LANE_DRIVE ─(라바콘 진입 확정)─▶ CONE_DRIVE ─(end_flag)─▶ REJOIN
REJOIN      ─(fresh 차선 유효성)▶ LANE_DRIVE               [lane_target = 0]

[외부 event 계약 미확정]
LANE_DRIVE ─(별도 고정장애물 진입 event)─▶ FIXED_AVOID
FIXED_AVOID ─(고정장애물 통과)───────────▶ OVERTAKE
OVERTAKE    ─(추월 완료)────────────────▶ LANE_DRIVE

[지름길 / 종료]
LANE_DRIVE ─(신호 3 & lap∈{2,3} & !shortcut_used)─▶ SHORTCUT ─▶ LANE_DRIVE
LANE_DRIVE ─(3바퀴 완료)─▶ FINISH

(모든 상태) ─(안전 조건 위반)─▶ STOP
```

- `FIXED_AVOID` 와 `OVERTAKE` 는 **순간적인 회피 동작이 아니라 구간(zone)** 이다.
  다만 `REJOIN` 성공은 `LANE_DRIVE` 복귀만 의미하며 `FIXED_AVOID` 진입 증거가 아니다.
  고정장애물 진입 publisher/topic/type 계약이 확정될 때까지 `FIXED_AVOID` 는 runtime에서
  도달 불가능한 외부 blocker로 유지한다.
- 구간 안에서 실제 회피/추월 동작을 할지는 `/object_info` 로 판단하며,
  `is_moving` 은 **회피 규칙을 고르는 데 쓴다** (추월 중에는 차선 이탈이 허용되지만
  고정장애물 회피는 그렇지 않다). 구간 진입 조건이 아니다.
- **구간이 끝나지 않을 때의 탈출 경로가 필요하다.** 고정장애물이 미검출되거나 방해차량을
  그 바퀴에 만나지 못하면 이벤트가 오지 않아 다음 구간으로 넘어가지 못한다.
  각 구간에 시간 상한을 두고, 초과하면 다음 구간으로 진행한다.
  랩 경계(결승선 통과)에서는 조건 없이 `LANE_DRIVE` + `lane_target = 0` 으로 초기화한다.

- 좌회전 신호(`3`)를 인식하면 **조건 없이 즉시 `SHORTCUT` 으로 전이한다.**
  신호가 나오는 바퀴는 랜덤이고 기회는 한 번뿐이므로, 직진을 선택하는 분기는 두지 않는다.
  `shortcut_used` 는 전이가 확정되는 시점에만 세워, 한 번의 신호가 두 번 소비되지 않게 한다.
- 좌회전 신호 인식은 **놓치면 만회할 수 없는 유일한 이벤트**다. 미검출(false negative)을
  최소화하는 방향으로 임계값을 잡는다.
- 라바콘 구간은 지름길 통과할 때를 제외하고 전부 통과하므로, 진입 판정 latch 는 랩 경계마다 재무장되어야 한다.
- 지름길 바퀴에는 `FIXED_AVOID` · `OVERTAKE` 구간에도 진입하지 않는다.
  `CONE_DRIVE` 를 거치지 않으므로 구간 체인 자체가 시작되지 않고, 중앙 주행이 유지된다.

### 지름길 바퀴의 가드

> 인지 노드는 모드와 무관하게 **항상 동작한다**(작년 구조 유지). 따라서 특정 구간에서만
> 유효한 신호는 인지단을 끄는 대신 **FSM 쪽 가드로 걸러낸다.**

- **지름길을 사용한 바퀴에는 라바콘이 없다.** 그런데 rubbercone 노드는 그 바퀴에도 계속
  동작하므로, 중앙 도로의 벽이나 구조물을 라바콘으로 오인해 유효한 confidence 를 낼 수 있다.
  `shortcut_used` 가 선 뒤 같은 바퀴에서 올라오는 라바콘 진입 신호는 **오검출로 간주하고 무시한다.**
- `CONE_DRIVE` 를 거치지 않은 바퀴가 생기는 것은 정상이다. 이를 이상 상태로 판단하거나
  복구 동작을 넣지 않는다.
- 라바콘 1분 제한은 `CONE_DRIVE` 에 진입한 바퀴에만 적용한다. 지름길 바퀴에는 타이머를 걸지 않는다.

### 랩 카운트

결승선의 Gate(통과감지장치)는 주최측 장비라 차량이 신호를 받을 수 없다.
따라서 **신호등 조우를 랩 경계로 사용한다.** 지름길로 가든 직진하든 매 바퀴 반드시
신호등 앞을 지나므로 누락이 없고, 신호등 상태 스트림을 그대로 랩 카운터 입력으로 쓸 수 있다.

---

## 실행 방법

대회 규정상 실행 명령은 `ros2 launch ...` 형태여야 한다.

```bash
# 전체 시스템 (실차)
ros2 launch main module_drive.py

# bag 파일 테스트 — 아래 launch 는 --clock 재생을 전제로 한다 (설명은 다음 절)
ros2 launch main module_drive_bag_test.py mode:=3 show_debug:=true

# 수동 주행 (Xbox 컨트롤러)
ros2 launch manual_drive manual_drive.launch.py

# bag 수집
ros2 launch manual_drive ordabag.launch.py
```

### bag 재생 시 주의 — `--clock` 필수

`module_drive_bag_test.py` 는 `main_node` 를 `use_sim_time: True` 로 띄운다.
따라서 bag 은 **반드시 `--clock` 과 함께** 재생해야 한다.

```bash
ros2 bag play <bag_path> --clock --loop
```

`--clock` 없이 재생하면 `/clock` 이 발행되지 않아 ROS 시계가 0 에 멈추고,
`create_timer` 가 **한 번도 발동하지 않는다.** 상태 창이 뜨지 않거나 모드가
그대로 굳어 있으면 대부분 이 경우다.

`use_sim_time` 을 켜 두는 이유는 `--loop` 재생 때문이다. 루프가 돌면 bag 시각이
뒤로 튀는데, `main_node` 는 **2 초 이상의 시각 역행**(`bag_loop_backjump_sec`)을
"bag 이 처음부터 다시 재생됨"으로 판정하고 내부 상태를 초기화한다.
이것이 없으면 2 회차부터 경과 시간이 음수가 되어 `Lane Change` 표시가 계속
`ACTIVE` 로 붙어 있는 등의 오동작이 생긴다.

### 상태 디버그 창 (`show_debug:=true`)

`main_node` 가 OpenCV 창 하나에 현재 판단 근거를 그린다.

| 항목 | 의미 |
|---|---|
| `Current Lane` | `/lane_position` 기반 **실측** 현재 차선 (Lane 1 / Lane 2 / Unknown) |
| `Lane Change` | 차선 변경 명령이 나간 직후 `ACTIVE` — 표시 전용이며 제어에 영향 없음 |
| `Target lane` | `lane_target`, 즉 **가려는** 차선 (`Current Lane` 과 다를 수 있음) |
| `Box` | 검출 박스 면적(px²) 과 `car_lane`. 차선 변경 트리거 판단의 직접 입력 |
| `Box shrink rate` | 박스 면적 변화율(px/s, EMA). **양수 = 멀어지는 중**(추월 성공), 음수 = 접근 중 |

`imshow` / `waitKey` 는 rclpy 콜백에서 직접 부르지 않고 **전용 표시 스레드**
(`_display_loop`)에 넘긴다. WSLg 환경에서 콜백 안 imshow 가 창을 띄우지 못하거나
인지 주기를 늘어뜨리는 문제가 있었다.

### 단위 테스트 / 오프라인 검증

```bash
colcon test --packages-select main
python3 -m main.tools.replay_fsm_bag <bag_path>
```

---

## 작업 현황

| 패키지 / 모듈 | 상태 | 비고 |
|---|---|---|
| `image_resize` | 유지 | 카메라 동일, 변경 없음 |
| `lane_detection` | 수정 | 피팅 안정화 + `/lane_position` 신설. 파라미터 재튜닝 ([차선 인식 안정화](#차선-인식-안정화-lane_detection)) |
| `rubbercone` | 수정중 | 라바콘 시작 지점 다름 |
| `object_detection` | 수정 | YOLO 모델(`best.onnx`) 유지, `is_moving` 분류 추가. **박스 선택을 신뢰도 → 면적 기준으로 변경** ([방해차량 충돌 대응](#방해차량-충돌-대응)) |
| `traffic_light` | 재작성 | 초록 Bool → 4구 신호등 4상태 |
| `main/main.py` | 재작성 | 옛 정수 FSM 폐기, 순수 모듈 계층 배선으로 교체 |
| `main/control.py` | 수정 | `MAX_ANGLE` 클램프, `reset(offset)` 시드 |
| `main` 로직 모듈 | 진행 중 | FSM 전이 일부 미구현 (아래 참조) |
| `manual_drive`, `sensors_viewer` | 유지 | 하드웨어 동일 |

### 검증 현황

`rosbag2_car_5_2026_07_24-14_50_54` 로 6개 노드 전체 재생 확인 — 에러 0건,
차선변경 트리거 `box_size` 1,960 / 2,470 (평균 2,215) 으로 여유 있게 회피.

### 미해결 항목

- `REJOIN → LANE_DRIVE` 는 구현됨. `FIXED_AVOID` 진입과 `OVERTAKE` · `SHORTCUT` · `FINISH` 전이는 외부 event 계약 미확정
- `shortcut_turn.py` 미작성 (지름길 좌회전 궤적)
- 랩 카운터 미구현 — `race_context.finish_gate_passes` 가 아직 증가하지 않음
- 시간 제한 감시는 현재 `SafetyMonitor` 기본값(라바콘 60초 / 전체 240초)으로 동작한다.
  이는 2026-07-29 「제9회 경주 진행 방법」 p.25와 p.37의 공식 제한이다. 전체 주행시간은
  p.17에 따라 파란불의 첫 fresh 관측 시각부터 계산하며, debounce 완료 시각으로 늦추지 않는다.
- `STOP` 이 종료 상태로만 정의되어 있어, 복귀 가능한 감속 정책 필요
- YOLO 모델이 고정장애물(고장난 차량)까지 검출하는지 미확인 —
  미검출 시 LiDAR 클러스터 보완 또는 재학습 필요

---
