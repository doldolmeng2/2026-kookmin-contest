#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

#include "lane_detection/path_preview.hpp"

namespace
{

constexpr int kWidth = 640;
constexpr int kHeight = 240;
constexpr float kReferenceX = 320.0F;

lane_detection::PathPreview preview(
  const std::vector<cv::Point2f> & points,
  float max_rmse_px = 3.0F)
{
  return lane_detection::estimatePathPreview(
    points, kWidth, kHeight, kReferenceX, 0.25F, 7U, 0.45F, max_rmse_px);
}

std::vector<cv::Point2f> sampleCurve(
  int width,
  int height,
  double a,
  double b,
  double c)
{
  std::vector<cv::Point2f> points;
  for (int index = 0; index < 12; ++index) {
    const double y = 0.15 + 0.07 * static_cast<double>(index);
    const double x = a * y * y + b * y + c;
    points.emplace_back(
      static_cast<float>(x * static_cast<double>(width)),
      static_cast<float>(y * static_cast<double>(height)));
  }
  return points;
}

}  // namespace

TEST(PathPreview, CenteredStraightHasZeroPreviewAndCurvature)
{
  const auto points = sampleCurve(kWidth, kHeight, 0.0, 0.0, 0.5);
  const auto result = preview(points);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(0.0F, result.target_offset_px, 1.0e-3F);
  EXPECT_NEAR(0.0F, result.signed_curvature, 1.0e-5F);
  EXPECT_NEAR(0.0F, result.rmse_px, 1.0e-3F);
  EXPECT_GT(result.confidence, 0.99F);
}

TEST(PathPreview, SignedBendsPreviewBeforeTheNearOffsetMoves)
{
  // x = 0.5 +/- 0.18*(y-0.92)^2 is centred at the nearest sample but has
  // already moved sideways at the look-ahead target y=0.25.
  constexpr double a = 0.18;
  constexpr double near_y = 0.92;
  const double b = -2.0 * a * near_y;
  const double right_c = 0.5 + a * near_y * near_y;
  const auto right = preview(sampleCurve(kWidth, kHeight, a, b, right_c));
  ASSERT_TRUE(right.valid);
  EXPECT_GT(right.target_offset_px, 45.0F);
  EXPECT_NEAR(2.0 * a, right.signed_curvature, 1.0e-4);

  const auto left = preview(sampleCurve(kWidth, kHeight, -a, -b, 1.0 - right_c));
  ASSERT_TRUE(left.valid);
  EXPECT_LT(left.target_offset_px, -45.0F);
  EXPECT_NEAR(-2.0 * a, left.signed_curvature, 1.0e-4);
}

TEST(PathPreview, InsufficientPointsOrCoverageIsInvalid)
{
  std::vector<cv::Point2f> too_few(6U, cv::Point2f(kReferenceX, 100.0F));
  EXPECT_FALSE(preview(too_few).valid);

  std::vector<cv::Point2f> narrow;
  for (int index = 0; index < 12; ++index) {
    narrow.emplace_back(kReferenceX, 100.0F + static_cast<float>(index));
  }
  EXPECT_FALSE(preview(narrow).valid);
}

TEST(PathPreview, FarTargetOutsideObservedWindowsIsInvalid)
{
  // Enough near windows and y span alone must not authorize a quadratic
  // extrapolation into the unseen far part of the BEV.
  std::vector<cv::Point2f> near_only;
  for (int index = 0; index < 8; ++index) {
    const float y_ratio = 0.45F + 0.07F * static_cast<float>(index);
    const float x_ratio = 0.5F + 0.08F * y_ratio * y_ratio;
    near_only.emplace_back(
      x_ratio * static_cast<float>(kWidth),
      y_ratio * static_cast<float>(kHeight));
  }

  EXPECT_FALSE(preview(near_only).valid);
}

TEST(PathPreview, SingleLargeOutlierIsClipped)
{
  auto points = sampleCurve(kWidth, kHeight, 0.12, -0.18, 0.55);
  points[5].x += 260.0F;

  const auto result = preview(points);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(points.size() - 1U, result.inlier_points);
  EXPECT_NEAR(0.24F, result.signed_curvature, 1.0e-3F);
  EXPECT_LT(result.rmse_px, 0.05F);
}

TEST(PathPreview, IncoherentNoisyPointsFailResidualGate)
{
  auto points = sampleCurve(kWidth, kHeight, 0.0, 0.0, 0.5);
  constexpr float noise[] = {
    -20.0F, 17.0F, -15.0F, 22.0F, -19.0F, 14.0F,
    18.0F, -23.0F, 16.0F, -17.0F, 21.0F, -14.0F};
  for (std::size_t index = 0U; index < points.size(); ++index) {
    points[index].x += noise[index];
  }

  const auto result = preview(points, 4.0F);
  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.rmse_px, 4.0F);
}

TEST(PathPreview, NonFiniteInputIsInvalid)
{
  auto points = sampleCurve(kWidth, kHeight, 0.0, 0.0, 0.5);
  points[3].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(preview(points).valid);
}

TEST(PathPreview, CoefficientsAndCurvatureAreScaleNormalized)
{
  constexpr double a = -0.14;
  constexpr double b = 0.10;
  constexpr double c = 0.52;
  const auto small_points = sampleCurve(640, 240, a, b, c);
  const auto large_points = sampleCurve(1280, 480, a, b, c);

  const auto small = lane_detection::estimatePathPreview(
    small_points, 640, 240, 320.0F, 0.25F, 7U, 0.45F, 3.0F);
  const auto large = lane_detection::estimatePathPreview(
    large_points, 1280, 480, 640.0F, 0.25F, 7U, 0.45F, 6.0F);

  ASSERT_TRUE(small.valid);
  ASSERT_TRUE(large.valid);
  EXPECT_NEAR(small.signed_curvature, large.signed_curvature, 1.0e-5F);
  EXPECT_NEAR(
    small.target_offset_px / 640.0F,
    large.target_offset_px / 1280.0F,
    1.0e-5F);
  for (int index = 0; index < 3; ++index) {
    EXPECT_NEAR(small.coefficients_norm[index], large.coefficients_norm[index], 1.0e-5);
  }
}
