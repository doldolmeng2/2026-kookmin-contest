#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// lane_guardrail.hpp
//
// BEV 상에서 차량 중심(ego_x)으로부터 좌/우 바깥 실선(left_solid/right_solid)까지의
// 여유를 잰다. 중앙선 하나만 보는 Pure Pursuit 은 구조적으로 조향 상한(실측 38.9)에
// 묶여 있어서, 코너에서 차선을 벗어나기 직전에 더 꺾을 방법이 없다. 이 여유가
// 그 상한을 넘길 근거를 제공한다.
//
// 중앙선용 수평/수직 노이즈 억제기는 여기에 쓰지 않는다. 그쪽은 "얇은 선 하나"를
// 전제로 튜닝된 것이라 굵은 실선에는 맞지 않는다. 대신 근거리 행마다 최근접
// 픽셀을 찾고 행 중앙값을 취해 스페클을 걸러낸다.
//
// bag 실측(carlane1/carlane2, 3601프레임): 관측률 83.8~89.5%, 프레임 간 변동
// 중앙값 1px (중앙선 피팅은 2px). 굵은 면적 신호라 얇은 선 피팅보다 안정적이다.
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <vector>

#include <opencv2/core.hpp>

namespace lane_detection
{

// 여유를 재지 못했음을 나타내는 값. 음수 하나로 통일해 토픽까지 그대로 나간다.
inline constexpr float kRailMarginUnknown = -1.0F;

struct RailMarginConfig
{
  // 한 행에서 이 개수 미만이면 스페클로 보고 그 행은 기권한다.
  int min_row_pixels{3};
  // 유효 행이 이 비율 미만이면 그 레일은 미관측으로 둔다.
  float min_valid_row_ratio{0.5F};
  // 샘플링할 근거리 행 구간 (BEV 높이 대비 비율).
  float band_top_ratio{0.60F};
  float band_bottom_ratio{0.95F};
};

struct RailMargins
{
  float left{kRailMarginUnknown};
  float right{kRailMarginUnknown};

  bool leftKnown() const {return left >= 0.0F;}
  bool rightKnown() const {return right >= 0.0F;}
};

namespace detail
{

inline float medianOf(std::vector<int> & values)
{
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const float upper = static_cast<float>(values[middle]);
  if (values.size() % 2 != 0) {
    return upper;
  }
  // 짝수 개면 아래쪽 값도 필요하다. nth_element 가 앞부분을 이미 갈라 놓았으므로
  // 그 안에서 최대값을 고르면 된다.
  const auto lower = std::max_element(values.begin(), values.begin() + middle);
  return 0.5F * (upper + static_cast<float>(*lower));
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// BEV 이진 마스크에서 ego_x 기준 좌/우 최근접 레일까지의 여유(px)를 잰다.
//
// bev_binary : 레일 클래스만 남긴 BEV 이진 영상 (0 또는 비0)
// ego_x      : BEV 상의 차량 중심 x. 사다리꼴이 cx 대칭이므로 bev_width/2 다.
//
// 반환값의 left/right 는 "거리"이므로 항상 0 이상이고, 못 재면
// kRailMarginUnknown 이다. 부호는 호출자가 좌/우로 구분한다.
// ─────────────────────────────────────────────────────────────────────────────
inline RailMargins railMargins(
  const cv::Mat & bev_binary,
  int ego_x,
  const RailMarginConfig & config)
{
  RailMargins margins;
  if (bev_binary.empty() || bev_binary.type() != CV_8UC1) {
    return margins;
  }

  const int height = bev_binary.rows;
  const int width = bev_binary.cols;
  ego_x = std::clamp(ego_x, 0, width - 1);

  int y_from = static_cast<int>(static_cast<float>(height) * config.band_top_ratio);
  int y_to = static_cast<int>(static_cast<float>(height) * config.band_bottom_ratio);
  y_from = std::clamp(y_from, 0, height - 1);
  y_to = std::clamp(y_to, y_from, height - 1);

  std::vector<int> left_hits;
  std::vector<int> right_hits;
  const int band_rows = y_to - y_from + 1;
  left_hits.reserve(band_rows);
  right_hits.reserve(band_rows);

  for (int y = y_from; y <= y_to; ++y) {
    const uchar * row = bev_binary.ptr<uchar>(y);

    int row_pixels = 0;
    for (int x = 0; x < width; ++x) {
      if (row[x] != 0) {++row_pixels;}
    }
    if (row_pixels < config.min_row_pixels) {continue;}

    // ego_x 에서 바깥으로 걸어 나가며 처음 만나는 픽셀이 최근접이다.
    for (int x = ego_x - 1; x >= 0; --x) {
      if (row[x] != 0) {left_hits.push_back(x); break;}
    }
    for (int x = ego_x + 1; x < width; ++x) {
      if (row[x] != 0) {right_hits.push_back(x); break;}
    }
  }

  const std::size_t need = std::max<std::size_t>(
    1U,
    static_cast<std::size_t>(
      static_cast<float>(band_rows) * config.min_valid_row_ratio));

  if (left_hits.size() >= need) {
    margins.left = static_cast<float>(ego_x) - detail::medianOf(left_hits);
  }
  if (right_hits.size() >= need) {
    margins.right = detail::medianOf(right_hits) - static_cast<float>(ego_x);
  }
  return margins;
}

// ─────────────────────────────────────────────────────────────────────────────
// 좌/우 레일 마스크 두 장을 합쳐 "각 방향에서 가장 가까운 레일"을 고른다.
//
// 어느 차선에 있든 동작하게 하려는 것이다. 2차선 주행이면 왼쪽 실선은 반대 차선
// 너머라 자연히 멀어서 선택되지 않고, 1차선이면 그 반대가 된다. 그래서 이 항은
// /lane_position 에 의존하지 않는다 (실측: 클래스 선택이 차선과 98% 일치).
// ─────────────────────────────────────────────────────────────────────────────
inline RailMargins mergeRailMargins(const RailMargins & a, const RailMargins & b)
{
  RailMargins merged;
  if (a.leftKnown() && b.leftKnown()) {
    merged.left = std::min(a.left, b.left);
  } else if (a.leftKnown()) {
    merged.left = a.left;
  } else if (b.leftKnown()) {
    merged.left = b.left;
  }

  if (a.rightKnown() && b.rightKnown()) {
    merged.right = std::min(a.right, b.right);
  } else if (a.rightKnown()) {
    merged.right = a.right;
  } else if (b.rightKnown()) {
    merged.right = b.right;
  }
  return merged;
}

}  // namespace lane_detection
