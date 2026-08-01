# 2025 제8회 국민대학교 자율주행 경진대회

## 디렉터리 생성 및 클론
```bash
mkdir -p xycar_ws/src/orda
cd xycar_ws/src/orda
git clone https://github.com/doldolmeng2/2025-kookmin-contest.git .
```

## 소스코드 파일 구조
```
src
├── modular
│    ├── image_resize      # 카메라 영상 리사이즈 (640×360)
│    ├── lane_detection    # BEV 기반 차선 검출
│    ├── object_detection  # LiDAR + YOLO 장애물 검출
│    ├── rubbercone        # LiDAR 기반 라바콘 오프셋 계산
│    ├── traffic_light     # 초록 신호등 검출
│    └── main              # 상태 머신 + 모터 제어 (오케스트레이터)
│
└── function
     ├── manual_drive      # Xbox 컨트롤러 수동 주행
     └── sensors_viewer    # 센서 데이터 시각화 툴
```

## 노드 및 토픽 구조

```
[Xycar HW]
  xycar_cam       → /image_raw          (sensor_msgs/Image)
  xycar_lidar     → /scan               (sensor_msgs/LaserScan)
  xycar_ultrasonic→ /xycar_ultrasonic   (std_msgs/Int32MultiArray)
  joy_node        → /joy                (sensor_msgs/Joy)

[전처리]
  resize_node
    sub: /image_raw
    pub: /resized_image                 (sensor_msgs/Image, 640×360)

[인지]
  traffic_node
    sub: /resized_image
    pub: /traffic_detection             (std_msgs/Bool)

  rubbercone_node
    sub: /scan
    pub: /rubbercone_info               (std_msgs/Int32MultiArray,
                                         [offset, end_flag, confidence])
                                         confidence: 경로 추정 신뢰도(0~100)

  lane_node
    sub: /resized_image, /mode_info
    pub: /lane_offset                   (std_msgs/Int16, 픽셀 오프셋)
         /lane_fit                      (std_msgs/Float32MultiArray, [m, b])
         /lane_change_state             (std_msgs/Int32MultiArray)

  object_node
    sub: /scan, /resized_image, /lane_fit
    pub: /object_info                   (std_msgs/Float32MultiArray, 10 필드)
         [exists, min_dist, angle, span, cluster_size,
          box_size, box_cx, box_cy, dx, car_lane]

[제어]
  main_node
    sub: /rubbercone_info, /lane_offset, /object_info,
         /traffic_detection, /joy, /xycar_ultrasonic
    pub: /xycar_motor                   (std_msgs/Float32MultiArray, [angle, speed])
         /mode_info                     (std_msgs/Int32MultiArray, [mode, lane])
```

## 주행 상태 머신 (main_node)

| 모드 | 값 | 설명 | 전환 조건 |
|---|---|---|---|
| TRAFFIC_WAIT | 0 | 신호 대기 (정지) | 초록불 감지 |
| RUBBERCONE_DRIVE | 1 | 라바콘 구간 주행 | end_flag = 1 |
| RUBBERCONE_END | 2 | 라바콘 종료 후 차선 진입 | 1.4초 경과 |
| BEFORE | 4 | 장애물 접근 대기 | 추월 조건 충족 |
| LANE_DRIVE | 3 | 차선 주행 | 박스 크기 ≥ 1900 px² |
| CHANGE_LANE | 5 | 차선 변경 중 | 변경 완료 |

## 실행 방법
```bash
# 전체 시스템 (실차)
ros2 launch main module_drive.py mode:=1

# bag 파일 테스트
ros2 launch main module_drive_bag_test.py mode:=1

# 수동 주행
ros2 launch manual_drive manual_drive.launch.py
```

## 브랜치 구조
```
main
├── modular-main
│    ├── modular/main
│    ├── modular/object_detection
│    ├── modular/lane_detection
│    ├── modular/traffic_light
│    ├── modular/rubbercone
│    └── modular/control
└── monolithic-main
```
