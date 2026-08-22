#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

namespace lane_detection
{

// A quadratic preview of the detected lane centre in BEV coordinates.
//
// coefficients_norm stores [a, b, c] for
//   x / bev_width = a * (y / bev_height)^2 + b * (y / bev_height) + c
// Keeping both axes normalized makes signed_curvature (= 2*a) independent of
// the BEV resolution. It is a normalized second-derivative curve score, not
// curvature in metres. target_offset_px remains in pixels so it can be consumed
// by the existing steering pipeline without another conversion.
struct PathPreview
{
  bool valid{false};
  float target_offset_px{0.0F};
  float signed_curvature{0.0F};
  float confidence{0.0F};
  float rmse_px{std::numeric_limits<float>::infinity()};
  cv::Vec3d coefficients_norm{0.0, 0.0, 0.0};
  std::size_t input_points{0U};
  std::size_t inlier_points{0U};
  float y_span_ratio{0.0F};
};

namespace path_preview_detail
{

inline double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }

  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if ((values.size() % 2U) != 0U) {
    return upper;
  }

  const auto lower = std::max_element(values.begin(), values.begin() + middle);
  return 0.5 * (upper + *lower);
}

inline bool fitQuadratic(
  const std::vector<cv::Point2f> & points,
  const std::vector<std::size_t> & indices,
  double width,
  double height,
  cv::Vec3d & coefficients)
{
  if (indices.size() < 3U) {
    return false;
  }

  cv::Mat design(static_cast<int>(indices.size()), 3, CV_64F);
  cv::Mat observations(static_cast<int>(indices.size()), 1, CV_64F);
  for (std::size_t row = 0U; row < indices.size(); ++row) {
    const cv::Point2f & point = points[indices[row]];
    const double y = static_cast<double>(point.y) / height;
    design.at<double>(static_cast<int>(row), 0) = y * y;
    design.at<double>(static_cast<int>(row), 1) = y;
    design.at<double>(static_cast<int>(row), 2) = 1.0;
    observations.at<double>(static_cast<int>(row), 0) =
      static_cast<double>(point.x) / width;
  }

  // Reject data with fewer than three independent y locations. SVD can return
  // a pseudo-solution for such data, but its quadratic term has no meaning.
  cv::Mat singular_values;
  cv::SVD::compute(design, singular_values);
  if (singular_values.total() < 3U) {
    return false;
  }
  const double largest = singular_values.at<double>(0);
  const double smallest = singular_values.at<double>(2);
  if (!std::isfinite(largest) || !std::isfinite(smallest) ||
    largest <= 0.0 || smallest <= largest * 1.0e-8)
  {
    return false;
  }

  cv::Mat solution;
  if (!cv::solve(design, observations, solution, cv::DECOMP_SVD)) {
    return false;
  }

  coefficients = cv::Vec3d(
    solution.at<double>(0), solution.at<double>(1), solution.at<double>(2));
  return std::isfinite(coefficients[0]) && std::isfinite(coefficients[1]) &&
         std::isfinite(coefficients[2]);
}

inline double residualPx(
  const cv::Point2f & point,
  const cv::Vec3d & coefficients,
  double width,
  double height)
{
  const double y = static_cast<double>(point.y) / height;
  const double predicted_x =
    width * (coefficients[0] * y * y + coefficients[1] * y + coefficients[2]);
  return static_cast<double>(point.x) - predicted_x;
}

inline double ySpanRatio(
  const std::vector<cv::Point2f> & points,
  const std::vector<std::size_t> & indices,
  double height)
{
  double y_min = std::numeric_limits<double>::infinity();
  double y_max = -std::numeric_limits<double>::infinity();
  for (const std::size_t index : indices) {
    const double y = static_cast<double>(points[index].y);
    y_min = std::min(y_min, y);
    y_max = std::max(y_max, y);
  }
  return (y_max - y_min) / height;
}

inline bool targetYIsObserved(
  const std::vector<cv::Point2f> & points,
  const std::vector<std::size_t> & indices,
  double height,
  double target_y_ratio)
{
  double y_min = std::numeric_limits<double>::infinity();
  double y_max = -std::numeric_limits<double>::infinity();
  for (const std::size_t index : indices) {
    const double y_ratio = static_cast<double>(points[index].y) / height;
    y_min = std::min(y_min, y_ratio);
    y_max = std::max(y_max, y_ratio);
  }
  // A quadratic can look perfect over the near half of the image yet explode
  // when extrapolated toward an unseen far target. Only evaluate a target that
  // is bracketed by actual sliding-window centres.
  return target_y_ratio >= y_min && target_y_ratio <= y_max;
}

}  // namespace path_preview_detail

// Fits x(y) to the sliding-window centres and evaluates it at target_y_ratio.
// Finite, isolated outliers are removed with three rounds of MAD residual
// clipping. At least 60% of the supplied points must survive, which prevents a
// noisy cloud from looking like a good curve after excessive trimming.
inline PathPreview estimatePathPreview(
  const std::vector<cv::Point2f> & center_points,
  int bev_width,
  int bev_height,
  float reference_x,
  float target_y_ratio,
  std::size_t min_points,
  float min_span_ratio,
  float max_rmse_px)
{
  PathPreview result;
  result.input_points = center_points.size();

  if (bev_width < 2 || bev_height < 2 || min_points < 3U ||
    center_points.size() < min_points || !std::isfinite(reference_x) ||
    !std::isfinite(target_y_ratio) || target_y_ratio < 0.0F || target_y_ratio > 1.0F ||
    !std::isfinite(min_span_ratio) || min_span_ratio <= 0.0F || min_span_ratio > 1.0F ||
    !std::isfinite(max_rmse_px) || max_rmse_px <= 0.0F)
  {
    return result;
  }

  for (const cv::Point2f & point : center_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return result;
    }
  }

  std::vector<std::size_t> inliers(center_points.size());
  for (std::size_t index = 0U; index < inliers.size(); ++index) {
    inliers[index] = index;
  }

  const double width = static_cast<double>(bev_width);
  const double height = static_cast<double>(bev_height);
  cv::Vec3d coefficients;

  // A one-pixel floor prevents exact or quantized data from clipping harmless
  // floating-point residuals. MAD keeps the threshold independent of outliers.
  constexpr int kClipIterations = 3;
  constexpr double kClipSigma = 3.0;
  constexpr double kMinClipPx = 1.0;
  for (int iteration = 0; iteration < kClipIterations; ++iteration) {
    if (!path_preview_detail::fitQuadratic(
        center_points, inliers, width, height, coefficients))
    {
      return result;
    }

    std::vector<double> residuals;
    residuals.reserve(inliers.size());
    for (const std::size_t index : inliers) {
      residuals.push_back(path_preview_detail::residualPx(
        center_points[index], coefficients, width, height));
    }

    const double residual_median = path_preview_detail::median(residuals);
    std::vector<double> deviations;
    deviations.reserve(residuals.size());
    for (const double residual : residuals) {
      deviations.push_back(std::abs(residual - residual_median));
    }
    const double robust_sigma = 1.4826 * path_preview_detail::median(deviations);
    const double clip_limit = std::max(kMinClipPx, kClipSigma * robust_sigma);

    std::vector<std::size_t> clipped;
    clipped.reserve(inliers.size());
    for (std::size_t row = 0U; row < inliers.size(); ++row) {
      if (std::abs(residuals[row] - residual_median) <= clip_limit) {
        clipped.push_back(inliers[row]);
      }
    }

    if (clipped.size() == inliers.size()) {
      break;
    }
    if (clipped.size() < min_points) {
      return result;
    }
    inliers.swap(clipped);
  }

  const std::size_t minimum_retained = std::max(
    min_points,
    static_cast<std::size_t>(std::ceil(0.60 * static_cast<double>(center_points.size()))));
  if (inliers.size() < minimum_retained ||
    !path_preview_detail::fitQuadratic(
      center_points, inliers, width, height, coefficients))
  {
    return result;
  }

  const double span_ratio = path_preview_detail::ySpanRatio(center_points, inliers, height);
  result.inlier_points = inliers.size();
  result.y_span_ratio = static_cast<float>(span_ratio);
  result.coefficients_norm = coefficients;
  if (!std::isfinite(span_ratio) || span_ratio < static_cast<double>(min_span_ratio) ||
    !path_preview_detail::targetYIsObserved(
      center_points, inliers, height, static_cast<double>(target_y_ratio)))
  {
    return result;
  }

  double squared_error = 0.0;
  for (const std::size_t index : inliers) {
    const double residual = path_preview_detail::residualPx(
      center_points[index], coefficients, width, height);
    squared_error += residual * residual;
  }
  const double rmse = std::sqrt(squared_error / static_cast<double>(inliers.size()));
  result.rmse_px = static_cast<float>(rmse);
  if (!std::isfinite(rmse) || rmse > static_cast<double>(max_rmse_px)) {
    return result;
  }

  const double target_y = static_cast<double>(target_y_ratio);
  const double target_x_normalized =
    coefficients[0] * target_y * target_y + coefficients[1] * target_y + coefficients[2];
  const double target_offset = width * target_x_normalized - static_cast<double>(reference_x);
  const double signed_curvature = 2.0 * coefficients[0];
  if (!std::isfinite(target_offset) || !std::isfinite(signed_curvature)) {
    return result;
  }

  const double inlier_score =
    static_cast<double>(inliers.size()) / static_cast<double>(center_points.size());
  const double coverage_score = std::clamp(
    span_ratio / static_cast<double>(min_span_ratio), 0.0, 1.0);
  const double residual_score = std::clamp(
    1.0 - rmse / static_cast<double>(max_rmse_px), 0.0, 1.0);

  result.valid = true;
  result.target_offset_px = static_cast<float>(target_offset);
  result.signed_curvature = static_cast<float>(signed_curvature);
  result.confidence = static_cast<float>(std::clamp(
    coverage_score * std::sqrt(inlier_score * residual_score), 0.0, 1.0));
  return result;
}

}  // namespace lane_detection
