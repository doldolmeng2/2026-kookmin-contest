# 방해차량 YOLO 모델 재학습 절차

빨간 방해차량을 검출하는 단일 클래스 YOLOv8 모델을 로스백 데이터로 학습시키는 전체 과정.

## 왜 재학습이 필요한가 (2026-08-10 측정)

기존 `best.onnx` 를 `~/my_rosbag` 의 고정 장애물 백에 그대로 돌린 결과:

| 백 | 프레임 | 최고 conf | conf≥0.83 통과 |
|---|---|---|---|
| stop_1 | 248 | 0.820 | **0 (0.0%)** |
| pass_1 | 534 | 0.859 | 5 (0.9%) |
| overtake_1 | 545 | 0.835 | 2 (0.4%) |

코드의 임계값이 `conf_threshold_ = 0.83` 이라 사실상 아무것도 검출되지 않는다.
더 심각한 건, 빨간 차가 화면 중앙에 크게 있는 프레임에서도 상위 박스가 차가 아니라
배경(유모차 바퀴, 검은 카트)에 붙는다는 점이다. 임계값만의 문제가 아니다.

## 재학습 결과 (2026-08-11)

658장(라벨 463박스 + 빈 라벨 200장)으로 yolov8n 을 150에폭 설정으로 돌려 53에폭에
조기 종료. **mAP50 0.995 / precision 0.999 / recall 1.000**, mAP50-95 0.760.

검증용 백(`pass_2`, `stop_2` — 학습에 미사용)에서 전처리별 conf>=0.50 검출률:

| 백 | stretch(옛 전처리) | letterbox |
|---|---|---|
| stop_2 (260) | 82.3% | **98.1%** |
| pass_2 (343) | 49.6% | **55.4%** |

`pass_2` 는 차가 시야 밖인 구간이 많아 상한이 56.0%다. 즉 letterbox 는 검출 가능한
프레임의 99%를 잡는다. 차가 없는 프레임의 오검출은 conf 0.05 에서도 0이었다.

이 결과를 반영해 `object_detection.cpp` 를 레터박스 전처리로 고치고
`conf_threshold_` 를 0.83 → 0.50 으로 낮췄다.

## 지켜야 할 제약

**클래스 1개, 입력 640 고정.** C++ 후처리가 출력 형태를 하드코딩한다
(`src/object_detection.cpp`):

```cpp
out = out.reshape(1, {5, 8400});   // 5 = cx,cy,w,h + 클래스 1개
```

`5 = 4(박스) + 클래스 수`, `8400 = 640 입력의 앵커 수`. 클래스를 늘리면 이 줄이 깨진다.

**전처리 불일치 주의.** 카메라 원본은 640×360인데 코드가 비율을 무시하고 640×640으로
늘린다(`cv::resize(img, resized, cv::Size(640, 640))`). YOLOv8 은 레터박스로 학습되므로
학습/추론이 어긋나 있다. 아래 둘 중 하나로 맞출 것:

- (권장) C++ 를 레터박스로 수정하고, 학습은 원본 640×360 프레임 그대로
- 또는 학습 이미지도 미리 640×640으로 늘려 저장해 왜곡을 학습에 포함

---

## 0. 환경 준비

RTX 50 시리즈(Blackwell, sm_120)는 **CUDA 12.8 이상으로 빌드된 PyTorch** 가 필요하다.
구버전 휠은 GPU를 인식하지 못한다.

Ubuntu 는 `venv` 를 별도 패키지로 쪼개 놨다. 없으면 `ensurepip is not available`
로 실패한다(`python3 -m venv --help` 는 이 경우에도 통과하므로 확인용으로 쓸 수 없다).

```bash
sudo apt install python3.10-venv
```

```bash
python3 -m venv ~/yolo_env
source ~/yolo_env/bin/activate
pip install -U pip          # venv 기본 pip 22 는 최신 CUDA 휠 메타데이터를 못 읽는 경우가 있다
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
python -c "import torch; print(torch.cuda.is_available())"   # 반드시 True
pip install ultralytics labelImg
```

`False` 가 나오면 휠 버전 문제다. 다른 단계로 넘어가기 전에 해결할 것.

torch 설치 중 `generate-parameter-library-py ... requires pyyaml` 류의 의존성 경고가
뜨는 건 무시해도 된다. `.bashrc` 의 ROS `PYTHONPATH` 때문에 ROS 패키지가 pip 눈에
보여서 나는 경고이고, torch 와는 무관하다.

### 어느 단계를 어느 파이썬으로 돌리는가

venv 에는 ROS 패키지가 없고, 시스템 파이썬에는 torch 가 없다. 섞으면 바로 막힌다.

| 단계 | 실행 환경 | 이유 |
|---|---|---|
| 1. 프레임 추출 | **venv 밖** (시스템 python3) | `rosbag2_py`, `rclpy` 필요 |
| 2~4. 라벨링·분할 | venv 밖 (시스템 python3) | 시스템 `cv2` 4.5.4 로 충분 |
| 5~7. 학습·내보내기 | **venv 안** | torch / ultralytics |

프롬프트 앞의 `(yolo_env)` 표시로 현재 위치를 확인할 것. venv 에서 나오려면 `deactivate`.

## 1. 프레임 추출

로스백의 `/resized_image`(640×360)를 학습 이미지로 뽑는다. 30fps 연속 프레임은
거의 중복이라 `--stride` 로 솎아낸다. 단일 클래스면 600~900장으로 충분하다.

```bash
python3 extract_frames.py --out ~/yolo_raw --stride 4 ~/my_rosbag/rosbag2_fixed_obstacles_*
```

**여러 백을 반드시 섞을 것.** 한 백만 쓰면 특정 거리·각도에 과적합된다.
조명이 다른 날 찍은 백이 있다면 같이 넣는 게 좋다.

## 2. 라벨 초안 자동 생성

빨간 차는 색이 특징적이라 HSV 색 분할로 박스 초안을 뽑을 수 있다.
600장을 맨손으로 그리는 대신 초안을 만들고 **틀린 것만 고친다.**

```bash
python3 autolabel_red.py --images ~/yolo_raw --labels ~/yolo_raw_labels
python3 autolabel_red.py --images ~/yolo_raw --preview /tmp/check.jpg   # 품질 눈으로 확인
```

측정된 초안 품질:
- 근거리·중거리: 대체로 정확 (`stop_1` 은 원거리 포함 248/248 검출)
- 차가 시야 밖: 정상적으로 빈 라벨 → 그대로 negative 샘플이 된다
- 오검출: 다른 카트의 빨간 부품에 박스가 붙는 경우가 있다 → 검수에서 지운다

## 3. 검수 (가장 중요한 단계)

```bash
labelImg ~/yolo_raw ~/yolo_raw_labels/classes.txt ~/yolo_raw_labels
```

좌하단 버튼으로 저장 형식을 **YOLO** 로 바꾼 뒤 진행한다. 단축키:

| 키 | 동작 |
|---|---|
| `d` / `a` | 다음 / 이전 이미지 |
| `w` | 새 박스 그리기 |
| `del` | 선택 박스 삭제 |
| `Ctrl+S` | 저장 |

체크할 것:

1. **오검출 박스 삭제** — 차가 아닌 것에 붙은 박스
2. **누락 추가** — 원거리에서 놓친 차
3. **박스 범위** — 검은 범퍼와 스포일러까지 차 전체를 감싸는지
4. **빈 라벨 유지** — 차가 없는 프레임의 0바이트 txt 를 지우지 말 것.
   배경 오검출(지금 유모차 바퀴에 박스가 붙는 문제)을 줄이는 핵심이다.

## 4. 데이터셋 분할

**백 단위로 나눈다.** 무작위로 섞으면 같은 백의 연속 프레임이 train/val 양쪽에 들어가
val mAP 가 부풀려지고 실제 성능을 오판하게 된다.

```bash
python3 split_dataset.py \
    --images ~/yolo_raw --labels ~/yolo_raw_labels \
    --out ~/yolo_dataset --val-bags rosbag2_fixed_obstacles_pass_2
```

`~/yolo_dataset/data.yaml` 이 자동 생성된다.

## 5. 학습

```bash
yolo detect train model=yolov8n.pt data=~/yolo_dataset/data.yaml \
    epochs=150 imgsz=640 batch=16 patience=30 hsv_h=0.005 name=obstacle_car
```

| 옵션 | 이유 |
|---|---|
| `yolov8n` | 차량 CPU 실시간 추론. 정확도가 부족하면 `yolov8s` 까지가 한계 |
| `epochs=150`, `patience=30` | 600~900장 규모에 적당. 개선이 없으면 조기 종료 |
| `hsv_h=0.005` | 색상이 이 물체의 핵심 단서다. 기본 색조 증강은 오히려 방해된다 |
| `fliplr` (기본 0.5) | 좌/우 차선 상황이 모두 나오므로 켜 둔다 |

## 6. 평가

`runs/detect/obstacle_car/` 의 `results.png` 와 혼동행렬을 본다.
단일 클래스에 이 정도 데이터면 **mAP50 0.9 이상**이 정상이다.
크게 낮으면 데이터를 늘리기 전에 라벨 품질부터 의심할 것.

## 7. ONNX 내보내기

```bash
yolo export model=runs/detect/obstacle_car/weights/best.pt \
    format=onnx imgsz=640 opset=12 simplify=True
```

`opset=12` 는 OpenCV DNN 호환성 때문이다. 너무 높으면 `readNet` 이 실패한다.

출력 형태가 `(1, 5, 8400)` 인지 반드시 확인한다:

```bash
python -c "import onnx; m=onnx.load('runs/detect/obstacle_car/weights/best.onnx'); print(m.graph.output[0])"
```

다르면 C++ 후처리와 맞지 않는다.

## 8. 배포와 임계값 재설정

```bash
cp runs/detect/obstacle_car/weights/best.onnx ~/2026-kookmin-contest/orda/perception/object_detection/best.onnx
```

모델 경로는 런타임 참조라 **재빌드가 필요 없다.**

`conf_threshold_ = 0.83` 을 그대로 두지 말 것. 새 모델의 신뢰도 분포를 측정해서
정해야 한다. 보통 0.4~0.6 사이가 된다.

`measure_conf.cpp` 가 노드와 **동일한 전처리·후처리·OpenCV 4.11** 로 프레임별
신뢰도 분포를 뽑아 준다. 위의 기준선 표도 이 도구로 측정한 값이다.

```bash
g++ -O2 -std=c++17 measure_conf.cpp -o /tmp/measure_conf \
    -I/usr/local/include/opencv4 -L/usr/local/lib -Wl,-rpath,/usr/local/lib \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_dnn
```

```bash
python3 extract_frames.py --out /tmp/eval_frames --stride 1 ~/my_rosbag/rosbag2_fixed_obstacles_stop_1
/tmp/measure_conf ../best.onnx /tmp/eval_frames /tmp/eval_boxes.jpg
```

출력의 임계값별 통과 프레임 비율을 보고 `conf_threshold_` 를 정한다.
`/tmp/eval_boxes.jpg` 에는 임계값 없이 상위 박스가 그려지므로, 박스가 실제로
차에 붙는지 눈으로 확인할 수 있다(초록 ≥0.83, 주황 ≥0.30, 빨강 그 미만).

`-I/usr/local/include/opencv4` 를 빼먹으면 시스템 OpenCV 4.5.4 로 링크되는데,
4.5.4 는 이 YOLOv8 ONNX 를 실행하지 못한다(`readNet` 은 되고 `forward` 에서 죽는다).
