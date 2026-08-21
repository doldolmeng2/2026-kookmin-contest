#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "lane_detection/lane_guardrail.hpp"

namespace
{

// 실제 BEV(1280x109)를 줄인 비율로 흉내낸다. 밴드는 y=60..95.
constexpr int kWidth = 200;
constexpr int kHeight = 100;
constexpr int kEgoX = 100;

lane_detection::RailMarginConfig config()
{
  lane_detection::RailMarginConfig cfg;
  cfg.min_row_pixels = 3;
  cfg.min_valid_row_ratio = 0.5F;
  cfg.band_top_ratio = 0.60F;
  cfg.band_bottom_ratio = 0.95F;
  return cfg;
}

cv::Mat blank()
{
  return cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
}

// 밴드 전체를 지나는 세로선. 두께를 주어 행당 픽셀 조건을 만족시킨다.
void drawRail(cv::Mat & image, int x)
{
  cv::line(image, {x, 0}, {x, kHeight - 1}, cv::Scalar(255), 3);
}

}  // namespace

TEST(LaneGuardrail, RightRailOnlyMeasuresRightMargin)
{
  cv::Mat bev = blank();
  drawRail(bev, kEgoX + 40);

  const auto margins = lane_detection::railMargins(bev, kEgoX, config());
  EXPECT_FALSE(margins.leftKnown());
  ASSERT_TRUE(margins.rightKnown());
  // 두께 3 이므로 가장 가까운 가장자리는 중심에서 1px 앞이다.
  EXPECT_NEAR(39.0F, margins.right, 1.0F);
}

TEST(LaneGuardrail, LeftRailOnlyMeasuresLeftMargin)
{
  cv::Mat bev = blank();
  drawRail(bev, kEgoX - 55);

  const auto margins = lane_detection::railMargins(bev, kEgoX, config());
  ASSERT_TRUE(margins.leftKnown());
  EXPECT_FALSE(margins.rightKnown());
  EXPECT_NEAR(54.0F, margins.left, 1.0F);
}

TEST(LaneGuardrail, BothRailsMeasuredIndependently)
{
  cv::Mat bev = blank();
  drawRail(bev, kEgoX - 70);
  drawRail(bev, kEgoX + 30);

  const auto margins = lane_detection::railMargins(bev, kEgoX, config());
  ASSERT_TRUE(margins.leftKnown());
  ASSERT_TRUE(margins.rightKnown());
  EXPECT_NEAR(69.0F, margins.left, 1.0F);
  EXPECT_NEAR(29.0F, margins.right, 1.0F);
}

TEST(LaneGuardrail, EmptyMaskReportsUnknown)
{
  const auto margins = lane_detection::railMargins(blank(), kEgoX, config());
  EXPECT_FALSE(margins.leftKnown());
  EXPECT_FALSE(margins.rightKnown());
  EXPECT_LT(margins.left, 0.0F);
  EXPECT_LT(margins.right, 0.0F);
}

// 스페클 몇 점이 레일보다 가까이 찍혀도 결과를 끌고 가면 안 된다.
// 행 중앙값이 이를 걸러낸다.
TEST(LaneGuardrail, SpeckleDoesNotPullMarginIn)
{
  cv::Mat bev = blank();
  drawRail(bev, kEgoX + 60);
  // 밴드 안 서너 행에만 찍힌 가까운 잡점.
  cv::rectangle(bev, {kEgoX + 10, 62}, {kEgoX + 13, 64}, cv::Scalar(255), -1);

  const auto margins = lane_detection::railMargins(bev, kEgoX, config());
  ASSERT_TRUE(margins.rightKnown());
  EXPECT_GT(margins.right, 50.0F);
}

// 밴드 아래쪽 몇 행에만 레일이 있으면 유효 행 비율을 못 채워 미관측이다.
TEST(LaneGuardrail, TooFewValidRowsReportsUnknown)
{
  cv::Mat bev = blank();
  cv::rectangle(bev, {kEgoX + 40, 92}, {kEgoX + 43, 95}, cv::Scalar(255), -1);

  const auto margins = lane_detection::railMargins(bev, kEgoX, config());
  EXPECT_FALSE(margins.rightKnown());
}

// 행 픽셀이 min_row_pixels 미만이면 그 행은 통째로 기권한다.
TEST(LaneGuardrail, ThinnerThanMinRowPixelsIsIgnored)
{
  cv::Mat bev = blank();
  for (int y = 0; y < kHeight; ++y) {
    bev.at<uchar>(y, kEgoX + 25) = 255;   // 행당 1px 뿐
  }

  const auto margins = lane_detection::railMargins(bev, kEgoX, config());
  EXPECT_FALSE(margins.rightKnown());
}

TEST(LaneGuardrail, MergeKeepsNearerRailOnEachSide)
{
  lane_detection::RailMargins left_solid;
  left_solid.left = 120.0F;
  left_solid.right = 300.0F;

  lane_detection::RailMargins right_solid;
  right_solid.left = lane_detection::kRailMarginUnknown;
  right_solid.right = 210.0F;

  const auto merged = lane_detection::mergeRailMargins(left_solid, right_solid);
  EXPECT_FLOAT_EQ(120.0F, merged.left);
  EXPECT_FLOAT_EQ(210.0F, merged.right);
}

TEST(LaneGuardrail, MergeOfTwoUnknownsStaysUnknown)
{
  const lane_detection::RailMargins none;
  const auto merged = lane_detection::mergeRailMargins(none, none);
  EXPECT_FALSE(merged.leftKnown());
  EXPECT_FALSE(merged.rightKnown());
}

// 2차선 주행 형상: 중앙선은 왼쪽에 있고 바깥 실선은 오른쪽에 있다.
// 레일 마스크에 중앙선이 섞이지 않는 한 왼쪽은 미관측이어야 한다.
TEST(LaneGuardrail, CenterLaneIsNotTreatedAsRail)
{
  cv::Mat rails = blank();
  drawRail(rails, kEgoX + 48);        // right_solid 만 레일 마스크에 있다

  const auto margins = lane_detection::railMargins(rails, kEgoX, config());
  EXPECT_FALSE(margins.leftKnown());
  ASSERT_TRUE(margins.rightKnown());
  EXPECT_NEAR(47.0F, margins.right, 1.0F);
}
