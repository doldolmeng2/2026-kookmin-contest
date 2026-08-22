#!/usr/bin/env bash
# xycar 장치 심링크(udev)를 설치하고, usb_cam 설정이 그 심링크를 보게 맞춘다.
#
#   sudo <워크스페이스>/src/orda/main/tools/udev/apply.sh
#
# 두 번 이상 실행해도 안전하다(멱등).
set -euo pipefail

RULES_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/99-xycar.rules"
RULES_DST=/etc/udev/rules.d/99-xycar.rules
CAM_YAML=/opt/ros/jazzy/share/usb_cam/config/params_1.yaml
CAM_LINK=/dev/xycar_cam

[[ $EUID -eq 0 ]] || { echo "ERROR: sudo 로 실행할 것" >&2; exit 1; }
[[ -f $RULES_SRC ]] || { echo "ERROR: $RULES_SRC 없음" >&2; exit 1; }

echo "==> udev 룰 설치: $RULES_DST"
install -m 0644 "$RULES_SRC" "$RULES_DST"

echo "==> udev 재적용"
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --action=add
udevadm trigger --subsystem-match=video4linux --action=add
udevadm settle

echo "==> 심링크 확인"
fail=0
for link in /dev/ttyLIDAR /dev/ttyMOTOR "$CAM_LINK"; do
  if [[ -e $link ]]; then
    printf '    OK   %-16s -> %s\n' "$link" "$(readlink -f "$link")"
  else
    printf '    MISS %-16s (장치가 연결되어 있는지 확인)\n' "$link"
    fail=1
  fi
done

# usb_cam 은 /opt/ros 안의 params_1.yaml 을 그대로 읽는다. video_device 가
# 커널이 그때그때 매기는 번호(/dev/video0)를 가리키면 재연결 한 번에 깨지므로
# 위에서 만든 고정 심링크로 바꾼다. 원본은 .bak 로 남긴다.
if [[ -f $CAM_YAML ]]; then
  current=$(sed -n 's/.*video_device: *"\([^"]*\)".*/\1/p' "$CAM_YAML" | head -1)
  if [[ $current == "$CAM_LINK" ]]; then
    echo "==> usb_cam video_device 이미 $CAM_LINK (변경 없음)"
  else
    backup="${CAM_YAML}.bak.$(date +%Y%m%d%H%M%S)"
    cp -a "$CAM_YAML" "$backup"
    sed -i "s|video_device: *\"[^\"]*\"|video_device: \"${CAM_LINK}\"|" "$CAM_YAML"
    echo "==> usb_cam video_device: ${current:-?} -> $CAM_LINK   (백업: $backup)"
  fi
else
  echo "ERROR: $CAM_YAML 없음 — usb_cam 설치 경로 확인 필요" >&2
  fail=1
fi

echo
if [[ $fail -eq 0 ]]; then
  echo "완료. 이제 launch 를 다시 실행하면 된다."
else
  echo "일부 항목 실패 — 위 MISS/ERROR 줄 확인." >&2
  exit 1
fi
