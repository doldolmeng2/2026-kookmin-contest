#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// class_map.hpp
//
// PIDNet 클래스맵(/pidnet_class_map, MONO8 라벨 0~5)에서 원하는 클래스만 골라
// 이진 마스크로 만든다.
//
// 예전에는 pidnet 이 중앙선만 골라 만든 /lane_segmentation_mask 를 구독했다.
// 가드레일 항은 같은 프레임의 left_solid/right_solid 도 필요한데, 마스크와
// 클래스맵을 따로 구독하면 두 토픽 사이에 프레임이 어긋날 수 있다(pidnet 이
// 마스크를 먼저 발행하므로 마스크 콜백 시점에는 같은 stamp 의 클래스맵이 아직
// 안 왔을 수 있다). 그래서 클래스맵 하나만 구독하고 중앙선 마스크는 여기서
// 만든다 — pidnet 의 np.isin(labels, lane_classes) * 255 와 같은 연산이다.
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <array>
#include <vector>

#include <opencv2/core.hpp>

namespace lane_detection
{

// ─────────────────────────────────────────────────────────────────────────────
// 라벨 영상에서 classes 에 포함된 픽셀만 255 로 남긴 마스크를 만든다.
//
// pidnet 의 `np.isin(labels, lane_classes).astype(np.uint8) * 255` 와
// 비트 단위로 같은 결과여야 한다 (test_class_map_extraction 이 이를 지킨다).
// ─────────────────────────────────────────────────────────────────────────────
inline cv::Mat extractClassMask(
  const cv::Mat & labels, const std::vector<int> & classes)
{
  cv::Mat mask = cv::Mat::zeros(labels.size(), CV_8UC1);
  if (labels.empty() || labels.type() != CV_8UC1 || classes.empty()) {
    return mask;
  }

  // 라벨은 0~255 이므로 조회표 한 번이면 클래스 개수와 무관하게 1패스로 끝난다.
  std::array<uchar, 256> lookup{};
  for (const int value : classes) {
    if (value >= 0 && value < 256) {
      lookup[static_cast<std::size_t>(value)] = 255;
    }
  }

  for (int y = 0; y < labels.rows; ++y) {
    const uchar * source = labels.ptr<uchar>(y);
    uchar * target = mask.ptr<uchar>(y);
    for (int x = 0; x < labels.cols; ++x) {
      target[x] = lookup[source[x]];
    }
  }
  return mask;
}

inline cv::Mat extractClassMask(const cv::Mat & labels, int single_class)
{
  return extractClassMask(labels, std::vector<int>{single_class});
}

}  // namespace lane_detection
