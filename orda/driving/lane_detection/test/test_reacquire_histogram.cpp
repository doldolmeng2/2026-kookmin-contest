#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "lane_detection/reacquire_histogram.hpp"

namespace
{

constexpr int kWidth = 100;
constexpr int kHeight = 60;
constexpr int kYStart = 18;

lane_detection::ReacquireHistogramSelection select(
  const cv::Mat & image, bool reacquire_active)
{
  return lane_detection::selectReacquireHistogram(
    image, 40, 60, kYStart, 50, 0.35F, 0.50F, reacquire_active);
}

}  // namespace

TEST(ReacquireHistogram, NormalCorridorSignalDoesNotCallFallback)
{
  cv::Mat bev = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
  cv::line(bev, {45, kYStart}, {45, kHeight - 1}, cv::Scalar(255), 2);
  const auto result = select(bev, true);
  EXPECT_FALSE(result.fallback_attempted);
  EXPECT_FALSE(result.fallback_candidate);
  EXPECT_GE(result.base_x, 44);
  EXPECT_LE(result.base_x, 46);
  EXPECT_EQ(40, result.tracking_x_min);
  EXPECT_EQ(60, result.tracking_x_max);
}

TEST(ReacquireHistogram, EmptyBevDoesNotCallFallback)
{
  const cv::Mat bev = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
  const auto result = select(bev, true);
  EXPECT_FALSE(result.fallback_attempted);
  EXPECT_FALSE(result.fallback_candidate);
  EXPECT_EQ(50, result.base_x);
}

TEST(ReacquireHistogram, DisabledFallbackKeepsCorridorBaselinePath)
{
  cv::Mat bev = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
  cv::line(bev, {88, kYStart}, {88, kHeight - 1}, cv::Scalar(255), 2);
  const auto result = select(bev, false);
  EXPECT_FALSE(result.fallback_attempted);
  EXPECT_FALSE(result.fallback_candidate);
  EXPECT_EQ(50, result.base_x);
  EXPECT_EQ(40, result.tracking_x_min);
  EXPECT_EQ(60, result.tracking_x_max);
}

TEST(ReacquireHistogram, ReacquiresSignalOutsideCorridor)
{
  cv::Mat bev = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
  cv::line(bev, {88, kYStart}, {88, kHeight - 1}, cv::Scalar(255), 2);
  const auto result = select(bev, true);
  EXPECT_TRUE(result.fallback_attempted);
  EXPECT_TRUE(result.fallback_candidate);
  EXPECT_GE(result.base_x, 87);
  EXPECT_LE(result.base_x, 89);
  EXPECT_EQ(0, result.tracking_x_min);
  EXPECT_EQ(kWidth - 1, result.tracking_x_max);
}

TEST(ReacquireHistogram, FullBevWithoutUsableHistogramKeepsInvalidCandidate)
{
  cv::Mat bev = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
  bev.at<unsigned char>(0, 90) = 255;
  const auto result = select(bev, true);
  EXPECT_TRUE(result.fallback_attempted);
  EXPECT_FALSE(result.fallback_candidate);
  EXPECT_EQ(50, result.base_x);
  EXPECT_EQ(40, result.tracking_x_min);
  EXPECT_EQ(60, result.tracking_x_max);
}

TEST(ReacquireHistogram, SparseCandidateDoesNotMutateCallerHistory)
{
  cv::Mat bev = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
  bev.at<unsigned char>(kHeight - 1, 90) = 255;
  const int previous_anchor = 50;
  const int previous_history_token = 1234;
  const auto result = select(bev, true);
  EXPECT_TRUE(result.fallback_candidate);
  EXPECT_EQ(50, previous_anchor);
  EXPECT_EQ(1234, previous_history_token);
}

TEST(ReacquireHistogram, EligibleNextFrameAttemptsAgain)
{
  cv::Mat bev = cv::Mat::zeros(kHeight, kWidth, CV_8UC1);
  cv::line(bev, {88, kYStart}, {88, kHeight - 1}, cv::Scalar(255), 1);
  EXPECT_TRUE(select(bev, true).fallback_attempted);
  EXPECT_TRUE(select(bev, true).fallback_attempted);
}
