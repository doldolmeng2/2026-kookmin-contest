# PIDNet-S 데이터셋 도구

ROS 2 bag에서 HLS/HSV/YCrCb 색 필터를 조정하고, 자동 생성된 semantic label을 직접 검수하기 위한 도구입니다.
라벨 PNG에는 화면 표시용 색이 아니라 PIDNet 학습에 쓰는 클래스 ID `0~5`가 단일 채널로 저장됩니다.

| ID | 클래스 | 키 |
|---:|---|---|
| 0 | background | `0` 또는 `e` |
| 1 | 중앙차선 (`center_lane`) | `1` |
| 2 | 왼쪽실선 (`left_solid`) | `2` |
| 3 | 오른쪽실선 (`right_solid`) | `3` |
| 4 | 도로 (`road`) | `4` |
| 5 | 지름길 (`shortcut`) | `5` |

## 빌드

```bash
cd ~/xycar_ws
colcon build --symlink-install --packages-select segmentation_tools
source install/setup.bash
```

OpenCV 창을 사용하므로 데스크톱 세션에서 실행해야 합니다. 기본 이미지 topic은 용량이 작은
`/image_raw/compressed`이며 `--topic /resized_image`처럼 변경할 수 있습니다.

## 1. 색 필터 튜닝

```bash
ros2 run segmentation_tools color_filter_tuner \
  ~/xycar_ws/bags/rosbag2_2026_08_06-13_27_27 \
  --config ~/xycar_ws/src/2026-kookmin-contest/segmentation_tools/config/color_filters.yaml
```

- `time` 바: bag 프레임 탐색
- `Space`: 재생/정지, `a`/`d`: 이전/다음 프레임
- `1`~`5`: 튜닝할 클래스
- HLS, HSV 및 YCrCb별 제어 창의 min/max 바: 현재 클래스 범위
- `m`: HLS, HSV와 YCrCb 마스크를 AND/OR로 결합
- `s`: YAML 저장, `q` 또는 `Esc`: 종료

왼쪽 화면은 현재 클래스 필터, 오른쪽은 모든 클래스의 최종 합성 결과입니다. 클래스가 겹치면
YAML의 `priority` 숫자가 작은 클래스가 우선합니다. 기본값은 모든 색을 통과시키므로 실제 추출
전에 각 클래스를 반드시 튜닝해야 합니다. 중앙차선은 기존 차선주행 코드의 노란색 필터값으로
초기화되어 있고 나머지 클래스는 전체 범위로 시작합니다.

## 2. 자동 라벨 및 데이터셋 추출

```bash
ros2 run segmentation_tools extract_dataset BAG_PATH ~/datasets/xycar_pidnet \
  --config CONFIG_PATH --interval 0.5 --val-every 5
```

튜너 time 바의 특정 프레임부터 일부만 확인하려면 `--start-index 852 --count 5`처럼 지정합니다.
기본적으로 라벨 영상의 상단 1/3은 천장/벽 영역으로 보고 항상 background(ID 0)로 저장됩니다.
비율은 `--top-background`로 바꿀 수 있습니다.
`--start-index 0 --end-index 2605 --count 250 --uniform`을 사용하면 지정 프레임 범위에서
정확히 250장을 균등하게 선택합니다.

여러 bag에서 뽑은 결과를 하나의 검수 목록으로 합칠 때는 다음 명령을 사용합니다.

```bash
ros2 run segmentation_tools merge_datasets OUTPUT INPUT_1 INPUT_2
```

결과 구조:

```text
xycar_pidnet/
├── images/{train,val}/*.png
├── labels/{train,val}/*.png
├── previews/{train,val}/*.jpg
├── lists/{train,val}.lst
├── dataset.yaml
└── classes.json
```

`*.lst`의 각 줄은 `이미지상대경로 라벨상대경로`이며 PIDNet의 일반적인 list 기반 dataset
loader에 바로 전달할 수 있습니다. 모델 설정의 `NUM_CLASSES`는 **6**, `IGNORE_LABEL`은 **255**로
맞추세요. 프레임 간격은 초 단위이고, 검증 샘플은 재현 가능하게 매 N번째 프레임으로 나뉩니다.

## 3. 라벨 검수

```bash
ros2 run segmentation_tools label_editor ~/datasets/xycar_pidnet --split train --config CONFIG_PATH
```

- `a`/`d`: 자동 저장 후 이전/다음 이미지
- `c`: 저장 후 다음 이미지, `q`: 저장 후 종료
- `b`: 브러시, `g`: 연결 컴포넌트 모드
- `[`/`]`: 브러시 축소/확대
- `0`~`5`: 칠할 클래스 (`e`도 background)
- `z`: 브러시/컴포넌트 작업 실행 취소(최대 20회), `x`: 오버레이/원본 전환
- `-`/`+`: 오버레이 투명도 변경

브러시와 컴포넌트 모두 누른 채 이동할 수 있습니다. 컴포넌트 모드는 클릭 위치의 현재 라벨과
8방향으로 이어진 픽셀 전체를 선택 클래스 ID로 바꿉니다. `train` 검수가 끝나면 `--split val`도
별도로 실행하세요. `a`, `d`, `c`, `q`로 저장될 때 `labels`의 클래스 ID PNG와 `previews`의
컬러 오버레이 JPG가 함께 즉시 갱신됩니다.
