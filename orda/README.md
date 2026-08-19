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
  (`/object_info[0] == 1` → `speed = 0`, 신호가 `2`/`3` 으로 바뀌면 즉시 해제).
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

PPT에 있는 토픽은 **공식 외부 인터페이스**, PPT에 없는 토픽은 패키지 내부 구현용으로
구분한다. 내부 토픽은 Main의 검증된 세부 판단과 기존 lane detector를 유지하기 위한
호환 계층이며, 다른 패키지가 새 외부 계약으로 의존해서는 안 된다.

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
  rubbercone_node
    sub: /scan
    pub: /rubbercone_offset             (std_msgs/Int32MultiArray)        [PPT 공식]
         [offset, end_flag]
         /rubbercone_info               (std_msgs/Int32MultiArray)        [내부 상세]
         [offset, end_flag, confidence, entry_ready]
         confidence: 경로 추정 신뢰도 (0~100)

  lane_node
    sub: /resized_image, /internal/lane_command
    pub: /lane_offset                   (std_msgs/Int16, 픽셀 오프셋)     [PPT 공식]
         /lane_fit                      (std_msgs/Float32MultiArray, [m, b]) [내부]
         /lane_change_state             (std_msgs/Int32MultiArray)        [내부]
         [변경중, 성공여부]
         /lane_position                 (std_msgs/Int16)                  [내부]
         /lane_valid                    (std_msgs/Bool)                   [레거시·미사용]

  object_yolo_node (Python ONNX Runtime)
    sub: /resized_image
    pub: /object_yolo                   (std_msgs/Float32MultiArray, 20 필드) [내부]
         /traffic_detection             (std_msgs/Int32)                  [내부]
         [fixed 10필드 슬롯, moving 10필드 슬롯]
         슬롯: [detected, object_type, confidence, box_size, box_cx, box_cy,
                box_x, box_y, box_w, box_h]
    train-10 detector가 같은 프레임의 신호등 candidate crop을 만들고,
    light1 classifier가 0=green, 1=left, 2=orange, 3=red를 분류한다.
    최종 mapping은 green→2, left→3, orange/red→1, 없음/불확실/오류→0이다.

  road_surface_node
    sub: /pidnet_class_map              (sensor_msgs/Image)
    pub: /road_surface                  (std_msgs/Int32, 0/1/2)
    PIDNet class 4/5 ratio와 최대 connected component를 parameterized ROI에서
    판정한다. normal bag separation 전에는 threshold를 명시해야만 시작한다.

  object_node (C++ LiDAR/차선 융합)
    sub: /scan, /resized_image, /lane_fit, /object_yolo, /traffic_detection
    pub: /object_info                   (std_msgs/Int32MultiArray)         [PPT 공식]
         [traffic_signal, fixed_vehicle_lane, moving_vehicle_lane]
         traffic_signal: 0=미검출, 1=정지, 2=직진, 3=좌회전
         vehicle_lane: 0=미검출/미확정, 1=1차선, 2=2차선
         /object_info_raw               (std_msgs/Float32MultiArray, 12 필드) [내부 상세]
         [exists, min_dist, angle, span, cluster_size,
          box_size, box_cx, box_cy, dx, car_lane, object_type, confidence]
         car_lane : 0=중앙, 1=왼쪽, 2=오른쪽
         object_type: -1=미확정, 0=고정장애물, 1=방해차량

[제어]
  main_node
    sub: /object_info, /rubbercone_offset, /lane_offset,
         /object_info_raw, /rubbercone_info, /lane_change_state,
         /road_surface, /scan, /imu, /joy,
         /xycar_ultrasonic
    pub: /xycar_motor                   (std_msgs/Float32MultiArray)       [PPT 공식]
         [angle, speed]
         /mode_info                     (std_msgs/Int16)                   [PPT 공식]
         0=WAIT_GREEN, 1=LANE_DRIVE, 2=CONE_DRIVE,
         3=FIXED_AVOID, 4=OVERTAKE, 5=SHORTCUT
         /lane_info                     (std_msgs/Int16)                   [PPT 공식]
         1=1차선, 2=2차선, 3=중앙
         /internal/lane_command         (std_msgs/Int32MultiArray)         [내부 호환]
         [legacy_lane_mode, internal_lane]
```

PPT 외 보조 인지는 `object_yolo_node`와 `road_surface_node`다. 별도
`interface_adapter_node`는 만들지 않았다. `/object_info` 집계는 `object_node`가 직접,
공식 모드와 lane detector 호환 명령의 분리는 `main_node`와 launch remap이 직접 맡는다.

### 인터페이스 변경 사유

| 토픽 | 확정 계약 | 사유 |
|---|---|---|
| `/object_info` | `Int32MultiArray [신호등, 고정 객체 차선, 이동 객체 차선]` | Object Detection이 신호등·고정·이동 객체를 한 공식 메시지에서 동시에 표현 |
| `/object_info_raw` | 기존 12필드 상세값 | Main의 거리·박스·분류 기반 판단을 보존하는 내부 이중 발행 |
| `/rubbercone_offset` | `[offset, end_flag]` | PPT 공식 2필드 계약 |
| `/rubbercone_info` | `[offset, end_flag, confidence, entry_ready]` | producer가 승인한 진입 edge를 보존하는 내부 이중 발행 |
| `/mode_info` | `Int16`, 코드 `0..5` | PPT의 모드 인터페이스. `FINISH`/`STOP`은 팀 코드가 정해질 때까지 발행하지 않음 |
| `/lane_info` | `Int16`, `1=1차선, 2=2차선, 3=중앙` | PPT의 차선 인터페이스 |
| `/internal/lane_command` | `[legacy_lane_mode, internal_lane]` | 기존 lane detector의 배열 계약을 launch remap으로 격리하여 차선 알고리즘은 수정하지 않음 |
| `/traffic_detection` | `Int32` 4상태 | same-frame detector crop classifier와 object_node 사이의 내부 결과 |
| `/object_yolo` | 고정·이동 객체별 10필드 슬롯 | Python ONNX 결과를 C++ 차선 융합 노드로 전달 |
| `/lane_fit` | `[m, b]` | 객체 박스를 1·2차선으로 분류하는 내부 회귀선 |
| `/lane_change_state` | `[changing, success]` | FIXED/OVERTAKE 차선 변경 완료 피드백 |
| `/lane_position` | `Int16` | 측면 LiDAR 완료 판단에 쓰는 자차 실측 차선 |
| `/lane_valid` | `Bool` | lane detector가 계속 발행하지만 `REJOIN` 제거 후 Main은 구독하지 않음. 담당자 소스를 임의 수정하지 않아 남긴 레거시 출력 |
| `/rubbercone_reset` | `Empty` | 한 라바콘 세션의 종료 래치를 새 진입 때 초기화 |
| `/road_surface` | `Int32` 신설 (`0` 미확정, `1` 기본 검은 도로, `2` 흰 지름길) | 지름길을 실제로 본 뒤 기본 도로가 연속 인식될 때만 종료 |

`car_lane` · `lane_target` · `/lane_position` 은 **같은 정수 규약(0=중앙, 1=왼쪽, 2=오른쪽)** 을
쓴다. `/lane_position` 만 미확정을 뜻하는 `-1` 을 추가로 쓴다.
방해차량이 있는 쪽의 반대편을 추월 방향으로 그대로 뒤집어 쓸 수 있게 하기 위함이다.

공식 `/object_info`는 신호등과 두 객체 종류를 같은 발행 주기에서 내보낸다. 고정 객체와
이동 객체가 동시에 보이면 각 종류에서 기존 정책대로 가장 큰 박스(가장 가까운 객체)를
하나씩 선택하므로 어느 한 종류도 다른 종류 때문에 사라지지 않는다.

#### Competition detector/classifier assets

Production은 package share의 `model/train-10/best.onnx`와
`model/classify/light1/weights/best.onnx`를 기본으로 사용하며 launch argument로만
override한다. 두 파일의 class metadata가 고정 계약과 다르면 fail-fast한다. 저장소의
기존 단일 클래스 `best.onnx`는 아래 legacy 자산이며 train-10 대체물로 사용하지 않는다.

#### Legacy 고정장애물 YOLO 모델 (`best.onnx`)

고정장애물(빨간 차량)을 검출하도록 재학습했다. 학습 데이터는 `rosbag2_fixed_obstacles_*`
6개에서 뽑은 658장(박스 463개 + 차량이 없는 프레임 200장)이다. 차량이 없는 프레임의
빈 라벨은 배경 오검출을 줄이는 negative 샘플이므로 반드시 포함한다.

- 현재 포함 모델은 단일 클래스 `obstacle_car`, 입력 640 고정 → 출력 `(1, 5, 8400)`이다.
  후처리는 Python ONNX Runtime으로 옮겼고 `(1,C,N)`/`(1,N,C)` 및 동적 클래스 수를
  처리한다. 클래스 매핑은 `config/object_detection.yaml`의 정수 배열
  `fixed_class_ids`와 `moving_class_ids`로 관리한다(예: `[0]`, `[1, 2]`).
- Production YAML은 train-10 계약대로 `fixed_class_ids: [0]`,
  `moving_class_ids: [1]`을 사용한다. 단일 클래스 legacy weight에는 적용하지 않는다.
- 검증 성능 mAP50 0.995 / precision 0.999 / recall 1.000
- 전처리는 **레터박스**(비율 유지 + 회색 114 패딩)를 쓴다. 640×360 원본을 640×640으로
  늘리면 세로가 1.78배 왜곡되는데 YOLOv8 은 레터박스로 학습되므로 어긋난다.
  검증용 bag 기준 conf ≥ 0.5 검출률이 82.3% → 98.1% 로 올랐다.
- `conf_threshold_ = 0.50`. 검출 가능한 프레임의 99%를 잡고, 차량이 없는 프레임의
  오검출은 conf 0.05 에서도 0이었다.

재학습 절차와 도구는 `object_detection/tools/TRAINING.md` 에 있다.

#### 정지 신호(`1`) 오검출 주의

공식 `/object_info[0] == 1` 은 **주행 중에도 즉시 정지**를 유발하므로 오검출 비용이
가장 크다. 이 값의 내부 원천은 `/traffic_detection`이다.
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
| 라바콘 종료 | `CONE_DRIVE` → `LANE_DRIVE` | 진입 전 차선 유지 | 별도 `REJOIN` 없이 즉시 복귀 |
| 고정장애물 구간 | `FIXED_AVOID` | 장애물 **반대 차선** | 통과 후에도 그 차선을 유지한다 (되돌아오지 않음) |
| 고정장애물 통과 → 결승선 | `LANE_DRIVE` | 회피한 차선 유지 | 측면 LiDAR clear 뒤 복귀 |
| 방해차량 구간 | `OVERTAKE` | 방해차량 **반대 차선** | `object_type=1`로 독립 진입 |
| 추월 완료 → 결승선 | `LANE_DRIVE` | 추월한 차선 유지 | 임의 중앙 복귀 없음 |
| 지름길 | `SHORTCUT` | 진입 당시 차선 유지 | 차선 조향을 계속 사용하고 CNN 도로 라벨로 종료 |

- 고정장애물이 2차선에 있으면 1차선으로 회피하고, **그대로 1차선을 유지한다.**
  회피 후 2차선으로 되돌아오지 않는다. (방해차량 구간이 구현되면 그 차선에서 이어간다.)
- 추월 방향은 시작 차선과 무관하게 **방해차량이 있는 차선의 반대편**으로 결정한다
  (같은 쪽으로 추월하면 차선 이탈로 간주됨, 규정 p.33).

`main_node` 가 내부 `/internal/lane_command` 의 차선 필드로 목표를 지시하고,
`lane_node` 가 `/lane_change_state` 로 이동 진행·완료를 보고한다.

#### 센서 역할 분담 (고정장애물 구간)

| 판단 | 사용 센서 | 근거 |
|---|---|---|
| 구간 진입 | 카메라 (`box_size` > 1900px²) | — |
| 회피 방향 | 카메라 (`car_lane`) | 아래 참조 |
| 추월 완료 | 측면 LiDAR (`side_left`/`side_right`) | 옆을 지나간 사실은 카메라로 볼 수 없다 |

회피 방향은 예전에 LiDAR 기반 `exists` 를 먼저 요구했다. 구간 진입은 카메라로 하면서
방향 결정만 LiDAR에 걸어둔 비대칭이라, 카메라가 방해차량을 또렷이 보는데도 회피가
시작되지 않았다. `rosbag2_fixed_obstacles_overtake_1` 실측에서 방해차량이 대부분 2 m
밖이라 `range_max_m`(2.0)에 걸려 전방 클러스터가 103 스캔 중 1번만 형성됐고,
`exists` 가 1810 샘플 내내 0이었다. 같은 구간에서 카메라는 박스 최대 10,549 px²,
`car_lane=1` 을 안정적으로 냈다. 지금은 카메라 산출물(`box_size`, `car_lane`)만으로
방향을 정한다.

박스는 있는데 좌/우가 미확정(`car_lane=0`)이면 현재 차선을 유지하며 다음 fresh
카메라 증거를 기다린다. 한 프레임의 반전으로 방향을 바꾸지 않고 연속 합의를 통과한
`car_lane`만 목표 차선에 반영한다.

**추월 완료 판정** (`main/overtake.py`)

- 회피로 옮겨간 차선의 **반대편**만 감시한다 (1차선 주행 → 오른쪽, 2차선 주행 → 왼쪽).
  기준은 실측 차선(`/lane_position`) 우선, 미확정이면 `lane_target` 으로 폴백.
- 측면 60~100° 섹터의 최소 거리가 `side_detect_m`(0.40 m) 미만인 것을 먼저 확인한다.
  그 장애물이 **사라진 뒤** `pass_delay_s`(2.0초) 동안 연속으로 비어 있어야 완료한다.
  중간에 다시 잡히면 clear 타이머를 초기화한다.
- `zone_timeout_s`(12초)는 이상 경고용이다. 측면 통과 증거가 없는데 시간만 지났다는
  이유로 `LANE_DRIVE`로 복귀하지 않는 fail-closed 정책이다.

`side_detect_m` 은 고정장애물 bag 6개 실측으로 정했다. 통과 순간 감시 측면의 최소거리가
**0.26~0.34 m**, 벽·빈 차선은 **0.60 m 이상**이다. 0.60 이었을 때는 구간 진입 1초 만에
0.59 m로 걸려 회피 차선 변경이 끝나기 전에 완료 카운트다운이 시작됐다.

> ⚠️ 방해차량은 **1차선과 2차선을 오가며 주행한다**(규정 p.32). `car_lane` 은 한 번 읽고
> 마는 값이 아니라 추월 직전까지 계속 갱신해야 하며, 접근 중에 방해차량이 차선을 바꾸면
> 추월 방향도 다시 결정해야 한다.

---

## 주행 상태 머신 (main_node)

내부 상태는 `race_fsm.Mode`로 유지한다. 숫자 런치 입력과 공식 `/mode_info`는 같은
코드 `0..5`를 사용한다. `FINISH`와 `STOP`은 내부 상태로만 남기고, 팀에서 외부 코드를
정하기 전까지 `/mode_info`를 발행하지 않는다.

| 내부 모드 | `/mode_info` | 설명 | 전이 조건 |
|---|---:|---|---|
| `WAIT_GREEN` | 0 | 준비 입력 확인 + 신호등 앞 정지 | 준비 완료 후 `/object_info[0]` 녹색 디바운스 |
| `LANE_DRIVE` | 1 | 차선 주행 (기본 중앙 주행) | 아래 분기 참조 |
| `CONE_DRIVE` | 2 | 라바콘 구간 주행 | fresh `end_flag` 0→1 |
| `FIXED_AVOID` | 3 | 고정장애물 구간 | 측면 LiDAR seen→clear→hold |
| `OVERTAKE` | 4 | 방해차량 구간 | 측면 LiDAR seen→clear→hold |
| `SHORTCUT` | 5 | 지름길 차선 주행 | 흰 도로 확인 후 검은 도로 연속 인식 |
| `FINISH` | 미발행 | 3바퀴 완료 | — |
| `STOP` | 미발행 | 복귀 가능한 안전 정지 | 필수 입력 회복 후 직전 상태 복귀 |

```
WAIT_GREEN ──(입력 준비 + 신호 2/3)──▶ LANE_DRIVE   [시간 측정 시작]

[확정된 라바콘 흐름]
LANE_DRIVE ─(라바콘 진입 확정)─▶ CONE_DRIVE ─(end_flag)─▶ LANE_DRIVE

[장애물 — object_type으로 분리]
LANE_DRIVE ─(YOLO fixed, 박스 ≥ 1900px²)──▶ FIXED_AVOID ─(LiDAR clear)─▶ LANE_DRIVE
LANE_DRIVE ─(YOLO moving, 박스 ≥ 1900px²)─▶ OVERTAKE    ─(LiDAR clear)─▶ LANE_DRIVE

[지름길 / 종료]
LANE_DRIVE ─(신호 3 & lap∈{2,3} & !shortcut_used)─▶ SHORTCUT
SHORTCUT ─(흰 지름길 확인 후 검은 도로 연속 인식)─▶ LANE_DRIVE
LANE_DRIVE ─(3바퀴 완료)─▶ FINISH

(모든 상태) ─(안전 조건 위반)─▶ STOP
```

- `FIXED_AVOID`와 `OVERTAKE`는 순간 동작이 아니라 독립 구간이다. 둘 다 완료 후
  피한 차선을 그대로 유지하며, 서로를 자동으로 연쇄하지 않는다.
- 장애물 타입과 세부 박스 정보는 내부 `/object_info_raw[10]`으로 결정한다.
  실제 다중 클래스 모델을 교체할 때
  `fixed_class_ids`/`moving_class_ids` 매핑을 학습 라벨 순서와 맞춰야 한다.
- 시간 초과는 경고만 남긴다. 측면에서 장애물을 본 뒤 사라졌다는 증거가 없으면
  상태를 유지해 장애물 앞에서 일반 차선 주행으로 잘못 복귀하지 않는다.

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

# 번호 기반 구간 테스트 (기본값은 motor 격리)
# 1=wait, 2=lane_center, 3=lane_1, 4=lane_2,
# 5=cone, 6=fixed, 7=overtake, 8=shortcut
ros2 launch main module_drive_mission_test.py test_profile:=2 live_drive:=false

# bag 파일 테스트
ros2 launch main module_drive_bag_test.py

# 국민대 라바콘 FSM bag 통합 테스트 (scan-only, motor 출력 격리)
./main/tools/run_rubbercone_bag_test.sh \
  ~/bags/kmu_real_lidar/rubbercone_20260725_221245

# 수동 주행 (Xbox 컨트롤러)
ros2 launch manual_drive manual_drive.launch.py

# bag 수집
ros2 launch manual_drive ordabag.launch.py
```

### 단위 테스트 / 오프라인 검증

```bash
colcon test --packages-select main
python3 -m main.tools.replay_fsm_bag <bag_path>
```

`object_detection/tools/` 에 YOLO 재학습 파이프라인이 있다. 절차는
`object_detection/tools/TRAINING.md` 참조.

```bash
# 노드와 동일한 전처리로 모델의 프레임별 신뢰도 분포를 측정
g++ -O2 -std=c++17 measure_conf.cpp -o /tmp/measure_conf \
    -I/usr/local/include/opencv4 -L/usr/local/lib -Wl,-rpath,/usr/local/lib \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_dnn
```

---

## 작업 현황

| 패키지 / 모듈 | 상태 | 비고 |
|---|---|---|
| `image_resize` | 유지 | 카메라 동일, 변경 없음 |
| `lane_detection` | 수정 | 차선 위치/변경 상태 계약 및 파라미터 병합 |
| `rubbercone` | 수정중 | 라바콘 시작 지점 다름 |
| `object_detection` | 수정 | ONNX Runtime, 동적 클래스 디코더, `object_type`/confidence 계약 |
| `traffic_light` | 재작성 | 초록 Bool → 4구 신호등 4상태 |
| `main/main.py` | 재작성 | 옛 정수 FSM 폐기, 순수 모듈 계층 배선으로 교체 |
| `main` 로직 모듈 | 구현 | WAIT/차선 3종/fixed/overtake/shortcut/lap/FINISH 및 단위 테스트 |
| `manual_drive`, `sensors_viewer` | 유지 | 하드웨어 동일 |

### 미해결 항목

- 현재 포함된 장애물 `best.onnx`는 단일 클래스다. 방해차량 자동 진입까지 쓰려면 팀의
  다중 클래스 모델과 정확한 `fixed_class_ids`/`moving_class_ids` 매핑을 넣어야 한다.
- 지름길 종료 가드와 `/road_surface` 구독 계약은 구현됐지만, 학습 중인 CNN publisher와
  모델은 이 저장소에 아직 없다. publisher가 `2(흰 지름길) → 1(검은 기본 도로)`을
  내야 자동 복귀가 완성된다.
- 시간 제한 감시는 현재 `SafetyMonitor` 기본값(라바콘 60초 / 전체 240초)으로 동작한다.
  이는 2026-07-29 「제9회 경주 진행 방법」 p.25와 p.37의 공식 제한이다. 전체 주행시간은
  p.17에 따라 파란불의 첫 fresh 관측 시각부터 계산하며, debounce 완료 시각으로 늦추지 않는다.
- `STOP`은 입력이 0.5초 연속 회복되면 정지 직전 모드로 복귀한다. `FINISH`만 종료 상태다.
- **`rubbercone_node` 가 라바콘이 없는 구간에서 벽·장비를 콘으로 오인한다.**
  고정장애물 bag 실측에서 `confidence ≥ 75` 가 7회, 그중 5회 연속으로 나와
  진입 조건(75 이상 3회 연속, 0.2초)을 통과했고 FSM 이 `LANE_DRIVE → CONE_DRIVE`
  로 새버렸다. `cone_entry.py` 주석대로 네거티브 bag 으로 임계값 재검증이 필요하다.
  고정장애물 bag 6개가 그 네거티브 데이터가 된다. 그때까지 고정장애물 시나리오를
  검증할 때는 `rubbercone_node` 를 띄우지 않는다 (아래 실행 방법 참조).

---
