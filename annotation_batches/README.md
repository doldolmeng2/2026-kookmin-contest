# 라벨링 배치 두는 곳

배치 데이터는 용량이 커서 저장소에 넣지 않습니다 (`.gitignore` 로 제외). 별도로 받은
zip 을 **이 폴더에 풀어서** 작업하세요. 푼 내용은 git 에 잡히지 않습니다.

```
annotation_batches/
  04_shortcut_250_서형찬_50/     ← zip 을 여기에 푼다
    images/  labels/  lists/  previews/
    dataset.yaml  classes.json  ANNOTATION_GUIDE.md
```

## 작업 절차

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select segmentation_tools
source install/setup.bash
ros2 run segmentation_tools label_editor \
    src/2026-kookmin-contest/annotation_batches/<배치_폴더> --split train
```

자세한 규칙(라벨링 범위, 클래스, 단축키)은 배치 폴더 안의 `ANNOTATION_GUIDE.md` 에
있습니다. 요약하면 **화면의 흰 가로선 아래(아래 40%, y≥216)만 칠하면 됩니다.**
그 위는 어둡게 덮여 있고 칠해도 저장되지 않습니다.

## 이미 라벨링을 시작한 경우

zip 을 덮어쓰면 작업물이 날아갑니다. 새 zip 을 풀지 말고 이 저장소만 다시 받아
`colcon build` 하세요. 이미지는 그대로이고 편집기 동작만 바뀝니다.

## 제출

`labels/train/` 폴더만 압축해서 보내면 됩니다. `images/` 는 바뀌지 않습니다.
받는 쪽에서 범위 밖 라벨을 정리합니다.

```bash
ros2 run segmentation_tools apply_roi_mask <배치_폴더> --split train
```
