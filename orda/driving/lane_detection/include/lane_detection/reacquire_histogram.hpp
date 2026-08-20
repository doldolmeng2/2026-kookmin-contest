#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

namespace lane_detection
{

struct WeightedHistogramBase
{
  bool usable{false};
  int base_x{0};
  float weighted_peak{0.0F};
};

struct ReacquireHistogramSelection
{
  int base_x{0};
  int tracking_x_min{0};
  int tracking_x_max{0};
  bool fallback_attempted{false};
  bool fallback_candidate{false};
};

inline WeightedHistogramBase weightedHistogramBase(
  const cv::Mat & bev_binary,
  int x_min,
  int x_max,
  int y_start,
  int anchor_x,
  float sigma_ratio,
  float min_weight)
{
  const int height = bev_binary.rows;
  const int width = bev_binary.cols;
  x_min = std::clamp(x_min, 0, width - 1);
  x_max = std::clamp(x_max, x_min, width - 1);
  y_start = std::clamp(y_start, 0, height - 1);
  const int histogram_width = x_max - x_min + 1;

  const cv::Mat histogram_roi = bev_binary(
    cv::Rect(x_min, y_start, histogram_width, height - y_start));
  const cv::Mat nonzero = histogram_roi > 0;
  cv::Mat column_sum;
  cv::reduce(nonzero, column_sum, 0, cv::REDUCE_SUM, CV_32S);

  const float effective_sigma_ratio = sigma_ratio > 0.0F ? sigma_ratio : 0.35F;
  const float effective_min_weight =
    min_weight > 0.0F && min_weight < 1.0F ? min_weight : 0.50F;
  const float sigma_pixels = std::max(
    1.0F, effective_sigma_ratio * static_cast<float>(histogram_width));
  const float two_sigma_squared = 2.0F * sigma_pixels * sigma_pixels;

  std::vector<float> weighted_histogram(histogram_width);
  for (int index = 0; index < histogram_width; ++index) {
    const int histogram_value = column_sum.at<int>(0, index) / 255;
    const int x = x_min + index;
    const float distance = static_cast<float>(std::abs(x - anchor_x));
    const float weight = effective_min_weight +
      (1.0F - effective_min_weight) *
      std::exp(-(distance * distance) / two_sigma_squared);
    weighted_histogram[index] = static_cast<float>(histogram_value) * weight;
  }

  const auto best = std::max_element(weighted_histogram.begin(), weighted_histogram.end());
  const float peak = best != weighted_histogram.end() ? *best : 0.0F;
  if (peak <= 0.0F) {
    return {false, std::clamp(anchor_x, x_min, x_max), peak};
  }
  return {
    true,
    x_min + static_cast<int>(std::distance(weighted_histogram.begin(), best)),
    peak,
  };
}

inline ReacquireHistogramSelection selectReacquireHistogram(
  const cv::Mat & bev_binary,
  int corridor_x_min,
  int corridor_x_max,
  int y_start,
  int anchor_x,
  float sigma_ratio,
  float min_weight,
  bool reacquire_active)
{
  ReacquireHistogramSelection selection;
  selection.tracking_x_min = corridor_x_min;
  selection.tracking_x_max = corridor_x_max;

  const auto corridor = weightedHistogramBase(
    bev_binary, corridor_x_min, corridor_x_max, y_start, anchor_x,
    sigma_ratio, min_weight);
  selection.base_x = corridor.base_x;
  if (corridor.usable || !reacquire_active || cv::countNonZero(bev_binary) == 0) {
    return selection;
  }

  selection.fallback_attempted = true;
  const auto full_bev = weightedHistogramBase(
    bev_binary, 0, bev_binary.cols - 1, y_start, anchor_x,
    sigma_ratio, min_weight);
  if (full_bev.usable) {
    selection.base_x = full_bev.base_x;
    selection.tracking_x_min = 0;
    selection.tracking_x_max = bev_binary.cols - 1;
    selection.fallback_candidate = true;
  }
  return selection;
}

}  // namespace lane_detection
