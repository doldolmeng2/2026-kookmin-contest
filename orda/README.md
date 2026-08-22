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
  다만 `WAIT_TRAFFIC` 모드는 **출발 시 한 번만 사용하고 재진입하지 않는다.**
  2·3바퀴의 정지는 모드 전환 없이 **제어 계층의 정지 오버라이드**로 처리한다
  (신호등 상태 `== 1` → `speed = 0`, `2`/`3` 으로 바뀌면 즉시 해제). 이 신호등 상태는
  더 이상 `/traffic_detection` 토픽이 아니라 **`/object_info`의 첫 필드**로 들어온다
  — 자세한 배경은 [인지 파이프라인 변경 이력](#인지perception-파이프라인-변경-이력) 참고.
  ⚠️ `main_node` 는 아직 이 새 계약에 맞춰 갱신되지 않아 실제로는 이 오버라이드가
  동작하지 않는다 (같은 절 "미반영 통합" 항목).
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
│   │    ├── traffic_light          # (미사용, launch 제외) 구 4구 신호등 판별 노드 — 기능은 object_detection 으로 통합됨
│   │    └── object_detection       # YOLO 통합 검출 (정지/이동 차량 + 신호등) → /object_info 발행
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
  rubbercone_node
    sub: /scan
    pub: /rubbercone_offset             (std_msgs/Int32MultiArray)        [PPT 공식]
         [offset, end_flag]
         /rubbercone_info               (std_msgs/Int32MultiArray)        [내부 상세]
         [offset, end_flag, confidence, entry_ready]
         confidence: 경로 추정 신뢰도 (0~100)

  lane_node
    sub: /resized_image, /mode_info
    pub: /lane_offset                   (std_msgs/Int16, 픽셀 오프셋)     [유지]
         /lane_fit                      (std_msgs/Float32MultiArray, [m, b]) [유지]
         /lane_change_state             (std_msgs/Int32MultiArray)        [유지]
         [변경중, 성공여부]

  object_yolo_node (Python, ONNX Runtime — object_detection 패키지)   [신규 구조, 2026-08-18]
    sub: /resized_image
    pub: /object_yolo                   (std_msgs/Float32MultiArray, 20 필드) [내부]
         [fixed 10필드 슬롯, moving 10필드 슬롯]
         슬롯: [detected, object_type, confidence, box_size, box_cx, box_cy,
                box_x, box_y, box_w, box_h]
         /traffic_boxes                 (std_msgs/Float32MultiArray, 6개씩 반복) [내부]
         [class_id, confidence, x, y, w, h] — 신호등 클래스 검출 전부
    단일 7클래스 통합 모델(train-2, `best.onnx`)이 차량과 신호등을 같이 검출한다.
    신호등 색은 검출기 클래스 id 그대로다: 1=green_light 2=left_green_light
    3=orange_light 5=red_light. 몸체 클래스(6=traffic)는 색을 못 정하므로 뺀다.
    ROI 크롭 + 별도 분류기 2단 경로는 폐지했다 (2026-08-22).

  road_surface_node
    sub: /pidnet_class_map              (sensor_msgs/Image)
    pub: /road_surface                  (std_msgs/Int32, 0/1/2)
    PIDNet class 4/5 ratio와 최대 connected component를 parameterized ROI에서
    판정한다. normal bag separation 전에는 threshold를 명시해야만 시작한다.

  object_node (C++ — object_detection 패키지)
    sub: /scan, /resized_image, /lane_fit, /object_yolo, /traffic_boxes
    pub: /object_info                   (std_msgs/Int32MultiArray, 3 필드)  [변경, 2026-08-18]
         [신호등 정보, 고정차량 위치, 방해차량 위치]
         신호등 정보:   0=인식x  1=정지(빨강/주황)  2=직진(초록)  3=좌회전
         고정차량 위치: 0=인식x  1=1차선  2=2차선
         방해차량 위치: 0=인식x  1=1차선  2=2차선

  ※ traffic_light 패키지(traffic_node)는 더 이상 launch 되지 않는다. 신호등 인식은
    object_yolo_node(검출) + object_node(우선순위 판정·디바운스)가 전담한다.
    자세한 배경은 [인지 파이프라인 변경 이력](#인지perception-파이프라인-변경-이력) 참고.

[제어]
  main_node
    sub: /rubbercone_info, /rubbercone_offset, /lane_offset, /lane_change_state,
         /object_info(Int32MultiArray 3필드), /object_info_raw(Float32MultiArray 12필드),
         /side_clearance, /lane_position, /road_surface, /scan, /joy, /xycar_ultrasonic
    pub: /xycar_motor                   (std_msgs/Float32MultiArray)      [유지]
         [angle, speed]
         /mode_info                     (std_msgs/Int32MultiArray)        [현재 호환]
         [legacy_mode_code, lane]
```

PPT 외 보조 인지는 `object_yolo_node`와 `road_surface_node`다. 별도
`interface_adapter_node`는 만들지 않았다. `/object_info` 집계는 `object_node`가 직접,
공식 모드와 lane detector 호환 명령의 분리는 `main_node`와 launch remap이 직접 맡는다.

### 인터페이스 변경 사유

| 토픽 | 변경 내용 | 사유 |
|---|---|---|
| `/object_info` | `Int32MultiArray [신호등, 고정 객체 차선, 이동 객체 차선]` | Object Detection이 신호등·고정·이동 객체를 한 공식 메시지에서 동시에 표현 |
| `/object_info_raw` | 기존 12필드 상세값 | Main의 거리·박스·분류 기반 판단을 보존하는 내부 이중 발행 |
| `/rubbercone_offset` | `[offset, end_flag]` | PPT 공식 2필드 계약 |
| `/rubbercone_info` | `[offset, end_flag, confidence, entry_ready]` | producer가 승인한 진입 edge를 보존하는 내부 이중 발행 |
| `/mode_info` | `Int16`, 코드 `0..5` | PPT의 모드 인터페이스. `FINISH`/`STOP`은 팀 코드가 정해질 때까지 발행하지 않음 |
| `/lane_info` | `Int16`, `1=1차선, 2=2차선, 3=중앙` | PPT의 차선 인터페이스 |
| `/internal/lane_command` | `[legacy_lane_mode, internal_lane]` | 기존 lane detector의 배열 계약을 launch remap으로 격리하여 차선 알고리즘은 수정하지 않음 |
| `/traffic_detection` | **폐지 (2026-08-22)** | 크롭 분류기 경로를 없애면서 발행자가 사라졌다. 신호등 상태는 `/object_info[0]` 하나로만 나간다. 옛 bag 에는 남아 있어 `bag_replay.py` 는 계속 읽는다 |
| `/traffic_boxes` | `Float32MultiArray` 6필드 반복 | 검출기가 본 신호등 박스를 그대로 `object_node` 에 넘겨 우선순위·디바운스를 한 곳에서 하게 한다 |
| `/side_clearance` | `Float32MultiArray [left_m, right_m]` | 좌우 60~100도 sector 의 1.5 m 이내 최소 거리. 반사가 없으면 1.5 m sentinel |
| `/object_yolo` | 고정·이동 객체별 10필드 슬롯 | Python ONNX 결과를 C++ 차선 융합 노드로 전달 |
| `/lane_fit` | `[m, b]` | 객체 박스를 1·2차선으로 분류하는 내부 회귀선 |
| `/lane_change_state` | `[changing, success]` | FIXED/OVERTAKE 차선 변경 완료 피드백 |
| `/lane_position` | `Int16` | 측면 LiDAR 완료 판단에 쓰는 자차 실측 차선 |
| `/lane_valid` | `Bool` | lane detector가 계속 발행하지만 `REJOIN` 제거 후 Main은 구독하지 않음. 담당자 소스를 임의 수정하지 않아 남긴 레거시 출력 |
| `/rubbercone_reset` | `Empty` | 한 라바콘 세션의 종료 래치를 새 진입 때 초기화 |
| `/road_surface` | `Int32` 신설 (`0` 미확정, `1` 기본 검은 도로, `2` 흰 지름길) | 지름길을 실제로 본 뒤 기본 도로가 연속 인식될 때만 종료 |
| `/mode_info` | 현재 `[legacy_mode_code, lane]` 유지 | 실제 소비자인 `lane_detection.cpp`가 아직 3=차선주행, 5=차선변경 계약을 사용한다. 4필드 신규 계약은 소비자 변경 전까지 발행하지 않는다. |

`car_lane` · `lane_target` · `/lane_position` 은 **같은 정수 규약(0=중앙, 1=왼쪽, 2=오른쪽)** 을
쓴다. `/lane_position` 만 미확정을 뜻하는 `-1` 을 추가로 쓴다.
방해차량이 있는 쪽의 반대편을 추월 방향으로 그대로 뒤집어 쓸 수 있게 하기 위함이다.

> `Int32MultiArray` 의 인덱스 의미는 이 문서뿐 아니라 **코드 내 상수로도 정의한다.**
> 문서에만 존재하는 인덱스 규약은 배선 실수의 주된 원인이 된다.

#### 검출기 자산 (2026-08-22 기준)

`object_yolo_node.py` 는 package share 의 `model/best.onnx` 하나만 로드한다.
`model_path` launch argument 로만 override 한다. 이 파일은 학습 산출물
`runs/detect/best_model/weights/best.onnx`(train-2, yolov8n, 100 epoch)와
md5 까지 동일하며, `model/train10_detector_best.onnx` 와도 같은 파일이다.

클래스는 7개다: `0=green_car 1=green_light 2=left_green_light 3=orange_light
4=red_car 5=red_light 6=traffic`. 차량 매핑은 `fixed_class_ids: [4]`(red_car),
`moving_class_ids: [0]`(green_car) 로 노드 파라미터 기본값이 들고 있다.

신호등 크롭 분류기 자산(`light_cls.onnx`, `model/light1_classifier_best.onnx`,
`object_detection/traffic_classifier.py`)은 삭제했다 — 검출기가 색까지 구분하므로
2단 경로를 유지할 이유가 없다.

#### Legacy 고정장애물 YOLO 모델 (`best.onnx`)

차량(고정장애물/방해차량)과 신호등을 **하나의 YOLO 모델**로 같이 검출한다.
`object_detection/best.onnx`와 `traffic_light/model/best_traffic.onnx`는 md5까지
동일한 파일이며, 두 인지 기능이 이 모델 하나를 공유한다.

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

**학습 이력**:

| 모델 | 클래스 | precision | recall | mAP50 | mAP50-95 | 비고 |
|---|---|---:|---:|---:|---:|---|
| `train-3` | 7 (`4-traffic` 몸체 포함) | 98.84% | 98.47% | 99.49% | 80.46% | 실주행 bag 검출률 낮음(아래 참고) |
| `train-4` | 7 | 99.15% | 99.32% | 99.43% | 81.33% | `train-3` 재학습 |
| `train-5` | 6 (`4-traffic` 제거) | **99.41%** | **99.44%** | 99.50% | 80.45% | val loss 최저, **현재 배포** |

지표는 각 모델 학습 시 val split 기준(`runs/detect/train-N/results.csv`)이라, 실제
주행 bag과는 분포가 다를 수 있다. `manual_20260818_134220`(오프라인, stride 5,
1176프레임)로 conf=0.50/NMS=0.40 기준 재검증한 결과:

| 모델 | 차량 검출 프레임율 | 신호등 검출 프레임율 | 평균 추론시간 |
|---|---:|---:|---:|
| `train-3` | 15.4% | 6.0% | 74.3ms |
| `train-4` | 27.7% | 25.9% | 74.5ms |
| `train-5` | 23.3% | 25.5% | 74.2ms |

`train-3`는 학습 지표는 높지만 실주행 조건(각도/거리/조명)에서 검출률이 크게 떨어진다 —
val split과 실제 주행 영상의 분포 차이로 보인다. `train-4`/`train-5`는 비슷한 수준이고,
`train-5`가 신뢰도는 약간 더 높다. 이 bag엔 `red_light`/`orange_light` 장면이 아예
없어서 두 클래스는 이걸로 검증이 안 됐다.

**알려진 약점 — `green_light` ↔ `left_green_light` 혼동**: `manual_20260818_134220`
뒷부분에서 `train-5`가 직진 초록불을 좌회전 화살표로 **170프레임 이상 연속** 확신을 갖고
오검출하는 구간을 발견했다(`left_conf` 0.5~0.87). 좌회전 오검출은 `SHORTCUT`을
잘못 트리거할 수 있어 위험도가 높다. 해당 구간(frame 5140~5330, 191장)을 추출해
`green_light`로 재라벨링해서 `/home/dxer1/yolo_hardneg_leftgreen/`에 준비해뒀다 —
기존 `yolo_merged` 데이터셋에 hard negative로 합쳐서 `train-6` 재학습이 필요하다
(아직 미실행, 아래 "미해결 항목" 참고).

전처리는 **레터박스**(비율 유지 + 회색 114 패딩)를 쓴다. 640×360 원본을 640×640으로
늘리면 세로가 1.78배 왜곡되는데 YOLOv8은 레터박스로 학습되므로 어긋난다.
`conf_threshold_ = 0.50`, `nms_threshold_ = 0.40`.

재학습 절차와 도구는 `object_detection/tools/TRAINING.md`에 있다(현재 문서는 옛 단일
클래스 기준으로 작성돼 있어 다중 클래스 재학습 시 감안해서 읽을 것).

#### 정지 신호(`1`) 오검출 주의

신호등 상태(`/object_info[0]`) `== 1` 은 **주행 중에도 즉시 정지**를 유발하므로 오검출 비용이 가장 크다.
트랙 중간에서 잘못 정지하면 재개 지연으로 실격까지 이어질 수 있다.

- 정지 신호를 **빨강과 주황으로 함께 정의했는데, 라바콘이 주황색이다.** 라바콘 구간에서
  신호등이 아닌 물체를 정지 신호로 읽지 않도록 해야 한다.
- 색상 블롭만으로 판정하지 말고 **4구 신호등 하우징을 먼저 찾은 뒤 그 안에서 색을 판정**하거나,
  ROI 를 신호등이 나타나는 화면 상단으로 제한한다.
- 좌회전 신호(`3`)는 미검출을 줄이는 방향으로, 정지 신호(`1`)는 오검출을 줄이는 방향으로
  임계값을 잡는다. **두 신호의 튜닝 방향이 반대**라는 점에 유의한다.

---

## 인지(perception) 파이프라인 변경 이력

2026-08-18 세션에서 `object_detection`/`traffic_light`/`lane_detection` 세 패키지를
손봤다. 아래는 무엇을 왜 바꿨고 실제로 측정한 결과가 어땠는지 기록이다.

### `object_detection` ↔ `traffic_light` 통합

**배경**: 신호등(`traffic_node`, Python+onnxruntime)과 차량(`object_node`, C++ +`cv::dnn`)이
따로 있었는데, 팀의 YOLO 모델이 차량+신호등 클래스를 한 모델로 합쳐서(`yolo_merged`
데이터셋) 나오기 시작했다. 또 타겟 보드(J4012)의 **OpenCV 4.5.4 `cv::dnn`이 이
YOLOv8 ONNX를 `forward()`할 때 shape assertion으로 죽는** 고질적인 문제가 있어서
(`object_detection.cpp:109-110`), 원래도 C++ 내부 추론 대신 Python `onnxruntime`
프론트엔드(`object_yolo_node.py`)를 거치고 있었다.

**결정**: 신호등 인식을 `traffic_node`에서 떼어내 `object_yolo_node.py` +
`object_node`로 흡수했다.
- `object_yolo_node.py`가 같은 프레임 추론 결과에서 차량(`/object_yolo`, 가장 가까운
  1개)과 신호등(`/traffic_boxes`)을 **같이** 발행한다 — 추론을 두 번 안 해도 된다.
- `object_node`가 두 토픽을 받아 신호등 우선순위(좌회전>직진>정지)·디바운스와
  차량 차선 판정을 전부 계산해서 `/object_info` **하나**로 낸다.
- `traffic_node`는 launch에서 뺐다(같이 띄우면 `/traffic_boxes`에 퍼블리셔가
  겹친다). 코드는 참고용으로 남아있다.

#### 신호등 판정을 ROI 크롭 + 분류기 2단계로 (2026-08-19)

> 이 절의 "크롭·분류를 C++ 로 옮겼다"는 부분은 아래 「크롭·분류를 다시
> `object_yolo_node.py` 로」에서 되돌렸다. 2단계 구조와 전처리 규약 자체는 유효하다.

**배경**: 검출기 하나로 신호등 위치와 색을 같이 맞히게 했더니, **위치는 맞고 색이
틀리는** 패턴이 남았다 (홀드아웃: 박스 찾기 100%, 클래스 86.9%). 원인은 검출기가
`green_light`와 `left_green_light`를 **박스 크기**로 가르고 있었다는 것 — 둘은 색이
같아 화살표를 봐야 하는데, 폭 40px에서는 화살표가 몇 픽셀뿐이라 크기가 더 쉬운
단서였다. 실제로 같은 신호등을 축소만 해도 `left_green_light` → `green_light`로
판정이 뒤집혔고, 좌회전 recall이 0.66에 머물렀다.

**결정**: 검출기는 '신호등이 여기 있다'는 **위치만** 쓰고, 무슨 신호인지는 박스를
잘라(마진 15%) 64×64 정사각으로 늘려 넣는 별도 분류기(`light_cls.onnx`)가 정한다.
크기를 고정하면 '멀면 작다'는 지름길 단서가 사라진다. 같은 val에서 좌회전 정확도
0.94, 전체 0.983.

- `object_detection/light_classifier.py` — 크롭·전처리·분류·디바운스. 전처리는 학습
  크롭 생성(`tools/make_light_crops.py`)과 반드시 같아야 한다(정사각 stretch → RGB
  → /255 → CHW).
- `object_yolo_node.py`가 신호등 박스 중 가장 가까운 1개를 잘라 분류하고, 그 결과를
  `/traffic_boxes`의 `class_id` 자리에 넣는다. **메시지 형식과 클래스 ID 규약은
  종전 그대로**라서 `object_node`(C++)와 `/object_info` 3필드 계약은 손대지 않았다 —
  판정 근거만 바뀌었다. 여러 박스를 그대로 내보내면 C++ 우선순위 판정이 검출기
  클래스를 다시 보게 되어 크롭 분류가 무의미해지므로 1개만 낸다.
- 분류기 모델이 없거나 `traffic_class_ids` 길이가 분류기 클래스 수와 다르면 경고만
  남기고 **검출기 클래스를 그대로 내보내는 종전 동작**으로 돌아간다.
- 관련 파라미터: `light_classifier_path`(빈 값이면 share의 `model/light_cls.onnx`),
  `light_crop_margin`(0.15), `light_input_size`(64), `light_min_confidence`(0.90).
  `traffic_class_ids`는 이제 **순서가 의미를 갖는다** — 분류기 출력 순서(green,
  left_green, orange, red)와 1:1.

**bag 검증 (traffic2~5, `/resized_image`만 재생, rate 0.25)**: 검출된 429프레임의
크롭 분류 클래스 정확도 94%. 디바운스 후 `/object_info[0]` 오답은 traffic5(초록)에
17프레임 남았는데, **연속 14프레임이라 디바운스로는 못 걸렀다**.

**확신도 게이트 (`light_min_confidence`, 기본 0.90)**: 위 오답 구간을 뜯어보니
원인이 분류기가 아니라 **검출기 박스**였다. 오답 프레임의 박스는 폭 200px에 높이
18px(종횡비 10:1 이상)이고 `y≈0~2`로 화면 상단에 잘려 붙어 있었다 — 학습 크롭
분포(약 4:1 하우징)를 완전히 벗어나서, 분류기가 늘려진 띠를 보고 찍은 셈이다.
그래서 확신도가 정답과 깨끗하게 갈렸다:

| | n | 최소 | 25%분위 | 중앙 | 최대 |
|---|---|---|---|---|---|
| 정답 | 403 | 0.338 | **0.998** | 1.000 | 1.000 |
| 오답 | 26 | 0.462 | 0.565 | 0.663 | **0.889** |

0.90 하나로 4개 bag 전체 오답 26 → 0 (남은 고확신 오답 2프레임은 연속되지 않아
디바운스가 흡수). **첫 정답 판정 시점은 네 bag 모두 전혀 늦어지지 않았다.**
게이트 탈락은 틀린 상태가 아니라 보류(`0`)로 나가는데, `WAIT_TRAFFIC`에서 `0`은
"계속 대기"라 안전한 방향이다. 종횡비 게이트(2~8:1)도 시도했으나 오답 감소는
같으면서 정답을 더 많이 버려서(403→370 vs 381) 채택하지 않았다.

#### 크롭·분류를 다시 `object_yolo_node.py` 로 (2026-08-19, 위 C++ 이전을 되돌림)

**증상**: `rosbag2_2026_08_13-13_33_23` 에서 **초록 신호등에 근접해 하우징이 화면 위로
빠져나가는 구간에, 분류기가 `orange` 를 0.9 이상으로 확신**했다. `orange` 는
`traffic_raw=1`(정지)이라 초록불 바로 아래에서 정지 판정이 나간다.

**원인은 분류기가 아니라 크롭 원본 프레임이었다.** `classifyLight()` 는 `/traffic_boxes`
로 받은 박스를 자기가 들고 있는 `last_raw_img_`(= 가장 최근에 도착한 프레임)에서
잘랐는데, 그건 그 박스를 만든 프레임이 아니다. 검출기 forward 가 CPU 에서 **73.6 ms**
(30fps 기준 2~3프레임)이고, 접근 말미의 박스 이동량은 **평균 4~6 px/frame, 최대
20 px/frame** 인데 그때 박스 높이는 이미 10~25 px 다. 즉 몇 프레임만 밀려도 크롭이
램프 줄을 통째로 비껴간다. 같은 bag 의 검출 268개에 인위적으로 lag 를 넣어 재현:

| lag | orange 판정 | orange ≥0.90 | orange 최대 |
|---|---|---|---|
| 0 (같은 프레임) | 4 | **0** | 0.614 |
| 3 | 32 | 4 | 0.989 |
| 6 | 67 | 9 | 0.996 |
| 8 | 103 | 26 | 0.999 |

`red` 로는 한 번도 안 간다. **분류기에 background 클래스가 없어서** 램프가 안 잡힌
크롭도 4개 중 하나를 뱉어야 하는데, 그 fallback 이 `orange` 다 — 평평한 패치
(black/gray114/어두운 하우징)를 넣어도 전부 `orange` 가 argmax 로 나온다.

**결정**: 크롭·분류를 `object_yolo_node.py` 의 `on_image()` 안으로 되돌렸다. 검출에 쓴
`image` 배열을 그대로 자르므로 프레임 어긋남이 **구조적으로** 생길 수 없다(타임스탬프
매칭이나 링버퍼가 필요 없다). `light_cls.onnx` 는 크롭당 **1.03 ms** 라 73.6 ms 짜리
검출기 옆에서 비용이 무시된다.

- `/traffic_boxes` 형식(6개씩 `[class_id, confidence, x, y, w, h]`)은 **그대로**다.
  `class_id`/`confidence` 자리에 이제 **분류기** 인덱스(0=green 1=left_green 2=orange
  3=red)와 그 확신도가 들어간다.
- 박스는 **전부** 내보낸다. 예전 Python 구현이 1개만 낸 이유(C++ 우선순위 판정이
  검출기 클래스를 다시 보게 됨)가 사라졌고, 좌회전 화살표는 초록 원과 같이 켜지는
  경우가 많아 여러 박스를 한꺼번에 봐야 한다.
- **`object_node` 의 `light_net_`/`classifyLight()` 는 남겨 뒀다.** `class_id` 가
  0~3 밖이면(= Python 쪽이 분류기를 못 띄웠으면) 예전처럼 여기서 직접 자르는 폴백으로
  간다. 폴백은 위 프레임 어긋남 한계를 그대로 갖는다.
- 우선순위(좌회전>직진>정지)·디바운스·확신도 게이트는 계속 `object_node` 가 한다.
  `/object_info` 3필드 계약은 안 바뀐다.

**기하 게이트 (`light_max_aspect` 8.0, `light_edge_min_h` 20)**: 확신도 게이트로는 못
거르는 두 번째 경로가 있다. 신호등이 화면 위로 빠지면 검출 박스가 보이는 띠만 잡아서
`203x48 → 226x25 → 240x14`, 종횡비 **4:1 → 17:1** 로 무너진다. 이걸 64×64 정사각으로
늘리면 학습 크롭 분포(약 4:1 하우징) 밖이다. 같은 프레임에서도 확신도가
`0.99 → 0.48` 로 붕괴하고 `orange` 가 이기기 시작한다.

같은 bag 의 신호등 검출 391개 기준으로 임계값을 골랐다:

| 종횡비 | n | 오답 | 확신도<0.90 |
|---|---|---|---|
| ~5 | 268 | 0 | 12 |
| 5~8 | 91 | 0 | 4 |
| 8~10 | 11 | 1 | **11 (전부)** |
| 10~ | 21 | 4 | **21 (전부)** |

종횡비 8 초과 박스는 **전부 이미 확신도 게이트 아래**라, 상한 8:1 을 걸어도 증거로
쓰이던 박스 343개를 **하나도 잃지 않는다**. 높이는 단독으로 자르지 않는다 — 멀리 있는
신호등은 `h` 가 10px 대여도 정상이고(같은 bag 에서 `h<12` 인 24개가 전부 정답), 문제는
"화면 최상단(`y<=1`)에 붙어 있으면서 납작한" 조합뿐이라 그 교집합에만 `h>=20` 을
요구한다. 예전에 기각했던 종횡비 게이트는 하한(2:1)까지 있어서 정답을 버렸는데, 여기는
**상한만** 둔다.

**검증 (같은 bag, 접근 구간 384프레임 전수, 디바운스 3 포함)**:

| | 증거 박스 | 그중 오답 | `/object_info[0]==1`(정지) 프레임 |
|---|---|---|---|
| 수정 전 (lag=6) | 119 | 9 | **8** |
| 수정 전 (lag=8) | 99 | 26 | **18** |
| 수정 후 (같은 프레임 + 기하 게이트) | **232** | **0** | **0** |

증거 박스 232개는 lag=0 무게이트 기준선과 **정확히 같다** — 게이트가 정답을 하나도
버리지 않았다. 디바운스는 lag=3 까지만 흡수하고 lag=6 부터 뚫린다.

**남은 한계**: 기하 게이트 하나만으로는 lag 실패 모드를 못 잡는다(lag=3 에서
오답 4 → 4 그대로). 그건 크롭을 같은 프레임으로 되돌린 쪽이 잡는다 — 두 수정은
서로 다른 경로를 막으므로 하나만 넣으면 안 된다.

**background 클래스 재학습은 아직 안 했다.** 두 수정을 다 넣은 뒤 이 bag 으로 다시
재보면, 게이트를 통과한 신호등 박스 **240개가 전부 정답**(중앙 확신도 0.999)이고
`orange`/`red` 오답은 0이다 — 실차에 "신호등이 꺼져 있는 경우"가 없다는 점과 별개로,
지금 상태에서 4개 클래스 중 하나를 억지로 골라야 하는 상황 자체가 이 bag 에는 거의
없다는 뜻이다. 그래서 지금 당장 급한 작업은 아니다.

다만 이유는 "꺼진 신호등"이 아니라 **크롭에 램프 줄이 안 들어오는 모든 경우**다 — 불이
켜져 있어도 크롭이 하우징 테두리·차양·전선만 담으면 같은 상황이 된다(수정 전 8129~8134
프레임이 그 예). 그리고 이걸 진짜로 유발할 수 있는 경로가 하나 남아 있다: **검출기가
신호등이 아닌 걸 신호등으로 오검출하는 경우**다. 이 bag 은 신호등 검출 391개가 전부
진짜 신호등이라 오검출이 0인데, 프레임 하단(도로·차량·배경)에서 게이트를 통과하는
모양(종횡비 3~6:1)으로 임의 크롭 576개를 만들어 넣어보면:

| | 결과 |
|---|---|
| argmax 분포 | green 258 · orange 250 · left_green 37 · red 31 |
| 확신도 ≥0.90 로 뚫린 것 | **71개 (12.3%)**, 최대 0.9998 |

즉 **램프 없는 크롭의 12%가 확신도 게이트를 그냥 통과한다** — 모양은 정상, 확신도는
0.99라 기하 게이트도 확신도 게이트도 못 막는다. 막을 수 있는 건 background 클래스뿐.
이 경로가 실제로 터지는지는 검출기가 다른 코스·조명에서 신호등 오검출을 내는지에
달려 있다. **우선순위: 다른 코스에서 신호등 오검출이 관측되면 background 클래스
재학습이 1순위로 올라간다.** 재학습 데이터는 이미 있다 — 이 bag 의 lag 크롭(램프 없는
하우징)과 위 임의 크롭이 그대로 hard negative 후보라 새로 촬영할 필요는 없다.

**폴백 상태 감지 (2026-08-19)**: `class_id` 가 -1(Python 쪽 `light_cls.onnx` 미로드)로
와서 `object_node` 가 `classifyLight()` 폴백을 타면, `/object_yolo`·`/traffic_boxes` 는
계속 정상 발행되기 때문에 겉보기엔 시스템이 멀쩡해 보인다 — 위 프레임 어긋남 버그가
소리 없이 되살아나 있는 상태를 아무도 못 알아채는 게 진짜 위험이다. 그래서 이 상태를
계속 알리는 장치를 넣었다:
- `object_yolo_node.py`: 분류기를 못 띄웠으면 5초 타이머로 `WARN`을 반복한다(시작 시점
  로그 한 줄은 스크롤로 사라지므로, 나중에 붙어서 로그를 보는 사람도 알 수 있어야 한다).
  크롭·추론 자체가 매 프레임 실패하는 경우의 에러 로그도 5초로 묶었다(`throttle_duration_sec`).
- `object_node`(C++): `onTrafficBoxes()`가 폴백을 탈 때마다 5초 묶음
  `RCLCPP_WARN_THROTTLE`을 낸다. 폴백조차 안 되면(`light_ok_=false`, 신호등 인식이
  완전히 죽은 상태) `RCLCPP_ERROR_THROTTLE`로 더 크게 알린다.

**모델 클래스 변천**: `train-3`(7클래스, `4-traffic` 몸체 포함) → `train-4`(같은
7클래스, 재학습) → `train-5`(6클래스, `4-traffic` 제거 — **현재 배포**). 클래스가
바뀔 때마다 `object_yolo_node.py`/`traffic_node.py`/`object_detection.cpp` 세
곳의 클래스 ID 상수를 같이 맞춰야 했다. 상세 지표는 위 "통합 YOLO 모델" 절 참고.

#### 크롭 분류기 경로 제거, 단일 검출기로 복귀 (2026-08-22)

`object_detection` 브랜치의 추론 경로를 main 에 올리면서 신호등 판정을 다시
**검출기 한 번**으로 되돌렸다. 위 두 절(2026-08-19)의 ROI 크롭 + `light_cls.onnx`
분류 경로는 삭제했다.

- 삭제: `light_cls.onnx`, `model/light1_classifier_best.onnx`,
  `object_detection/traffic_classifier.py`, `test_traffic_classifier.py`
- `/traffic_detection` 발행자가 사라졌다. 신호등 상태는 `/object_info[0]` 하나뿐이고,
  `object_yolo_node` 는 `/traffic_boxes` 만 낸다 (색은 검출기 class_id 그대로).
- `config/object_detection.yaml` 의 `object_yolo_node` 블록을 없앴다. 그 블록의
  `traffic_output_topic: "/traffic_detection"` 이 새 노드 기본값(`/traffic_boxes`)을
  덮어써서, 신호등 박스가 C++ 구독자에 **에러 없이** 안 닿는 상태가 됐다.
  클래스 매핑은 이제 노드 파라미터 기본값이 들고 있고
  `test_object_yolo_contract.py` 가 그 조합을 고정한다.
- `/object_yolo` 는 main 의 20필드(fixed/moving 2슬롯) 계약을 그대로 유지했다.
  단일 슬롯으로 줄이면 두 종류가 동시에 보이는 프레임에서 `/object_info` 의
  고정차량/방해차량 위치 중 하나가 항상 0 이 된다. 슬롯별 차선 확정은
  `LaneStabilizer` 두 개가 따로 한다.
- `/side_clearance`, `/object_info_raw` 는 그대로 발행한다 — `main_node` 가 구독한다.
- launch argument `detector_model_path` / `traffic_classifier_model_path` 는
  노드가 선언하지 않는 이름이라 조용히 무시되고 있었다. `model_path` 하나로 합쳤다.

### 차량 차선 판정 (`object_node`, 고정장애물/방해차량이 1차선인지 2차선인지)

`/lane_fit`(중앙선 `x=m·y+b`)과 검출 박스 중심(`box_cx`)의 거리(`dx`)를
`lane_split_margin_px`와 비교해서 판정한다 (`onYoloDetection()`).

- **마진 값 변천**: 45 → 30 → 25 → 10 → **5(현재)**. 처음 45였던 이유는 `/lane_fit`
  흔들림만으로 `dx` 부호가 뒤집히는 사례(-41→+31) 때문이었는데, 아래 차선 피팅
  안정화 작업으로 `/lane_fit` 자체가 훨씬 안정되면서 마진을 줄여도 괜찮아졌다.
- **디바운스 추가**: `lane_debounce_frames`(기본 3) — raw 판정이 N프레임 연속 같아야
  `lane_stable_`에 반영. 순간적으로 1프레임만 튀는 값을 걸러낸다.
- **박스 모서리 기반 판정은 시도했다가 롤백했다**: 중심점 1개 대신 박스 4개 모서리 중
  차선에서 가장 먼 모서리를 기준으로 판정하는 방식(차 크기에 비례해서 판정 엄격도가
  자동 조절됨)을 구현했으나, 사용자 판단으로 롤백해서 **현재는 중심점 기반**으로
  돌아가 있다. 필요하면 git 히스토리 또는 이 문서 개정 이력에서 코드를 다시 찾을 수
  있다(커밋되지 않았다면 이 PR의 이전 커밋을 참고).

### 차선 피팅 안정화 (`lane_detection`, `/lane_fit` 자체의 지터링)

**증상**: 급조향처럼 화면이 크게 도는 게 아니어도, 정지된 카메라에서조차 `/lane_fit`이
프레임마다 크게 흔들리는 문제가 있었다.

**적용한 변경 4가지** (`lane_detection_parameter.json`, `src/lane_detection.cpp`):

1. **Canny 엣지 대신 꽉 찬 색상 마스크를 그대로 사용** (`preprocessYellow()`). 예전엔
   HLS∩HSV∩YCrCb로 만든 꽉 찬 마스크를 다시 그레이스케일+Canny로 윤곽선만 뽑아서
   슬라이딩 윈도우에 넘겼는데, Canny의 그래디언트 임계값이 조명/압축 노이즈에
   반응해 프레임마다 살아남는 점이 달라졌다. 모폴로지로 정리한 마스크를 그대로
   쓰면 점이 훨씬 조밀해지고 안정적이다. 부수 효과로 처리 속도도 50Hz→62Hz로 올랐다.
2. **강건 피팅(반복적 이상치 제거)**: 1차 SVD 피팅 → `fit_outlier_reject_px`(20px)
   보다 먼 점 제거 → 재피팅, `fit_outlier_iterations`(2)회 반복. 윈도우 1~2개가
   노이즈를 주워도 전체 피팅이 안 흔들리게 한다.
3. **탐색 앵커를 고정 기준점(`ref_x_`)에서 "직전 프레임 위치" 추적으로 변경**
   (`fitLaneFromBEV()`의 `anchor_x`). 예전엔 매 프레임 `ref_x_` 주변에서 새로
   찾아서, fallback으로 버티다 신호가 다시 잡히면 아무 노이즈로나 확 튀었다.
   **실측(같은 bag, 같은 구간): 평균 이동량 70px→3.4px, 최대 590px→(재획득 로직 추가
   전 기준) 안정 구간 다수 확인.**
4. **연속 실패 시 재획득**(`fit_reacquire_after_frames`, 기본 10): 3번 변경만으로는
   앵커가 한 번 잘못된 자리에 멈추면 좁은 코리도어 안에 영원히 갇히는 문제가 있었다
   (실측: `ok=false` 최장 818프레임 연속). N프레임 연속 실패하면 `ref_x_`로 강제
   복귀하도록 해서 최장 정체 구간을 818→330프레임으로 줄였다.

**코리도어 폭(`corridor_width`) 변천**: 900(원래값, 반대 차선까지 탐색범위에 들어옴)
→ 300(반대 차선 배제) → **900(현재, 급조향 시 300은 못 따라가서 원복)**. 3번(앵커
추적) 도입 이후에는 코리도어가 넓어도 매 프레임 "직전 위치" 중심으로 움직이므로
예전만큼 반대 차선을 잘못 물 위험이 적다 — 단, 이론적 가능성은 남아있다.

**남은 문제**: 위 개선에도 실주행 bag(`manual_20260818_134220`) 기준 피팅 성공률이
26.5%에 그친다. 재획득 로직으로 정체 시간은 줄었지만, `ref_x_` 근처에서도 애초에
픽셀이 부족한 구간이 많다는 뜻이라 — 코리도어/앵커와는 별개의, 색상 임계값이나
조명 조건 쪽 원인일 가능성이 크다. 미해결로 남겨둔다.

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
| 라바콘 종료 | `CONE_DRIVE` → `LANE_DRIVE` | 진입 전 차선 유지 | `REJOIN`은 production 경로에서 사용하지 않음 |
| 고정장애물 구간 | `FIXED_AVOID` | 장애물 **반대 차선** | 통과 후에도 그 차선을 유지한다 (되돌아오지 않음) |
| 고정장애물 통과 → 결승선 | `LANE_DRIVE` | 회피한 차선 유지 | 측면 LiDAR clear 뒤 복귀 |
| 방해차량 구간 | `OVERTAKE` | 방해차량 **반대 차선** | `object_type=1`로 독립 진입 |
| 추월 완료 → 결승선 | `LANE_DRIVE` | 추월한 차선 유지 | 임의 중앙 복귀 없음 |
| 지름길 | `SHORTCUT` | 진입 당시 차선 유지 | 차선 조향을 계속 사용하고 CNN 도로 라벨로 종료 |

- 고정장애물이 2차선에 있으면 1차선으로 회피하고, **그대로 1차선을 유지한다.**
  회피 후 2차선으로 되돌아오지 않는다. (방해차량 구간이 구현되면 그 차선에서 이어간다.)
- 추월 방향은 시작 차선과 무관하게 **방해차량이 있는 차선의 반대편**으로 결정한다
  (같은 쪽으로 추월하면 차선 이탈로 간주됨, 규정 p.33).

`main_node` 가 `/mode_info` 의 `lane_target` 으로 목표를 지시하고,
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

내부 상태는 `race_fsm.Mode`로 유지하지만 런치 입력은 번호를 우선 사용한다.
`mode`: `0=INIT, 1=WAIT_TRAFFIC, 2=LANE_DRIVE, 3=CONE_DRIVE,
4=FIXED_AVOID, 5=OVERTAKE, 6=SHORTCUT, 7=FINISH, 8=STOP`.
`/mode_info`는 `lane_detection.cpp`의 기존 숫자 계약으로 별도 변환한다.

| 내부 모드 | 현재 외부 값 | 설명 | 전이 조건 |
|---|---:|---|---|
| `INIT` | 0 (STOP) | 입력 대기 | 필수 입력 수신 |
| `WAIT_TRAFFIC` | 0 (STOP) | 신호등 앞 정지, 출발 대기 | `/object_info[0]` 디바운스 (옛 `/traffic_detection`) |
| `LANE_DRIVE` | 3 | 차선 주행 (기본 중앙 주행) | 아래 분기 참조 |
| `CONE_DRIVE` | 1 | 라바콘 구간 주행 | fresh `end_flag` 0→1 |
| `REJOIN` | 2 | 옛 bag 호환용이며 production 미사용 | fresh lane-validity → `LANE_DRIVE` |
| `FIXED_AVOID` | action 중 5, 유지 시 3 | 고정장애물 구간 | 측면 LiDAR seen→clear→hold |
| `OVERTAKE` | action 중 5, 유지 시 3 | 방해차량 구간 | 측면 LiDAR seen→clear→hold |
| `SHORTCUT` | 3 | 지름길 차선 주행 | 흰 도로 확인 후 검은 도로 연속 인식 |
| `FINISH` | 0 (STOP) | 3바퀴 완료 | — |
| `STOP` | 0 (STOP) | 안전 정지 | — |

```
INIT ──(입력 준비)──▶ WAIT_TRAFFIC ──(신호 2/3)──▶ LANE_DRIVE   [시간 측정 시작]

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
- 장애물 타입은 `/object_info[10]`(옛 12필드 `Float32MultiArray` 계약 기준)으로
  결정하도록 `main.py`/`runtime_adapter.py`가 짜여 있다. **`object_node`는 이제 이
  포맷을 발행하지 않으므로(3필드 `Int32MultiArray`) 이 판정은 실제로 동작하지 않는다**
  — `main_node`를 새 계약에 맞게 갱신하기 전까지의 알려진 갭이다.
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
| `lane_detection` | 수정 | 차선 위치/변경 상태 계약 및 파라미터 병합. 차선 피팅 지터링 감소(Canny→마스크 직접 사용, 코리도어/이상치 제거/재획득) — [상세](#인지perception-파이프라인-변경-이력) |
| `rubbercone` | 수정중 | 라바콘 시작 지점 다름 |
| `object_detection` | 수정 | `traffic_light` 통합, 6클래스 모델(`train-5`), `/object_info` 3필드 `Int32MultiArray`로 재정의 |
| `traffic_light` | **미사용(launch 제외)** | 로직은 파일에 남아있으나 `object_detection`이 전담 — 코드는 참고용으로만 유지 |
| `main/main.py` | 재작성 | 옛 정수 FSM 폐기, 순수 모듈 계층 배선으로 교체 |
| `main` 로직 모듈 | 구현 | WAIT/차선 3종/fixed/overtake/shortcut/lap/FINISH 및 단위 테스트 |
| `manual_drive`, `sensors_viewer` | 유지 | 하드웨어 동일 |

### 미해결 항목

- **(중요) `main_node`가 새 `/object_info` 계약(3필드 `Int32MultiArray`)에 안 맞춰져 있다.**
  `object_node`는 이제 `[신호등, 고정차량 위치, 방해차량 위치]` 3필드 `Int32MultiArray`를
  내는데, `main_node`(`main.py`/`runtime_adapter.py`)는 여전히 옛 12필드
  `Float32MultiArray`(`exists, min_dist, ..., object_type, confidence`)를 구독하려
  한다. ROS2는 같은 토픽 이름이라도 타입이 다르면 아예 연결되지 않으므로,
  `main_node`와 `object_node`를 같이 띄우면 **`/object_info`가 조용히 안 이어진다**
  (`ros2 topic echo /object_info`가 "타입이 2개"라고 에러 내는 것으로 확인 가능).
  장애물 회피(`FIXED_AVOID`/`OVERTAKE`)와 신호 정지 오버라이드 둘 다 이 영향을 받는다.
  `main_node`를 새 계약에 맞춰 다시 배선하기 전까지는 실차 통합 테스트가 불가능하다.
- `object_detection`의 통합 YOLO 모델이 `green_light`를 `left_green_light`로 오검출하는
  구간이 있다 (위 "통합 YOLO 모델" 절 참고). 라벨링한 hard negative 데이터가
  `/home/dxer1/yolo_hardneg_leftgreen/`에 준비돼 있으나, 기존 데이터셋과 병합 후
  `train-6` 재학습·검증이 아직 안 됐다.
- `lane_detection`의 차선 피팅이 이번 세션에서 크게 안정화됐지만(아래 "인지 파이프라인
  변경 이력" 참고), 여전히 **실주행 bag 기준 프레임의 약 74%에서 피팅이 실패(fallback)
  한다** — 코리도어/앵커 문제가 아니라 노란 마스크 자체가 안 잡히는 더 근본적인
  원인(조명/색상 임계값/구간별 실제 가시성)으로 보이며, 아직 원인 파악이 안 됐다.
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
