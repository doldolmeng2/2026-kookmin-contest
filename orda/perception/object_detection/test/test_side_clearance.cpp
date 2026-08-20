#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "object_detection/side_clearance.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::vector<float> scanWithPoints(
  const std::vector<std::pair<double, float>> & points,
  std::size_t count = 361)
{
  std::vector<float> ranges(count, std::numeric_limits<float>::infinity());
  const double increment = 2.0 * kPi / static_cast<double>(count - 1);
  for (const auto & point : points) {
    const double radians = point.first * kPi / 180.0;
    const auto index = static_cast<std::size_t>(
      std::llround((radians + kPi) / increment));
    ranges.at(index) = point.second;
  }
  return ranges;
}

object_detection::SideClearanceResult calculate(
  const std::vector<float> & ranges,
  float range_min = 0.1F,
  float range_max = 10.0F)
{
  return object_detection::calculateSideClearance(
    ranges, -kPi, kPi, 2.0 * kPi / static_cast<double>(ranges.size() - 1),
    range_min, range_max);
}

}  // namespace

TEST(SideClearance, SeparatesPositiveAndNegativeNinetyDegrees)
{
  const auto result = calculate(scanWithPoints({{90.0, 0.42F}, {-90.0, 0.85F}}));

  ASSERT_TRUE(result.publishable);
  EXPECT_FLOAT_EQ(result.left_m, 0.42F);
  EXPECT_FLOAT_EQ(result.right_m, 0.85F);
}

TEST(SideClearance, IgnoresBodyReflectionsBeyondOneHundredDegrees)
{
  const auto result = calculate(scanWithPoints({{130.0, 0.12F}, {-130.0, 0.11F}}));

  ASSERT_TRUE(result.publishable);
  EXPECT_FLOAT_EQ(result.left_m, object_detection::kSideClearanceNoReturnM);
  EXPECT_FLOAT_EQ(result.right_m, object_detection::kSideClearanceNoReturnM);
}

TEST(SideClearance, IgnoresReflectionsBeyondOnePointFiveMeters)
{
  const auto result = calculate(scanWithPoints({{90.0, 1.50F}, {-90.0, 1.51F}}));

  ASSERT_TRUE(result.publishable);
  EXPECT_FLOAT_EQ(result.left_m, 1.50F);
  EXPECT_FLOAT_EQ(result.right_m, object_detection::kSideClearanceNoReturnM);
}

TEST(SideClearance, IgnoresNonFiniteNegativeAndScanRangeViolations)
{
  const auto result = calculate(scanWithPoints({
      {90.0, std::numeric_limits<float>::quiet_NaN()},
      {80.0, -0.2F},
      {70.0, 0.05F},
      {-90.0, 11.0F},
    }));

  ASSERT_TRUE(result.publishable);
  EXPECT_FLOAT_EQ(result.left_m, object_detection::kSideClearanceNoReturnM);
  EXPECT_FLOAT_EQ(result.right_m, object_detection::kSideClearanceNoReturnM);
}

TEST(SideClearance, EmptySideSectorsProduceFiniteClearSentinel)
{
  const auto result = calculate(scanWithPoints({{0.0, 0.5F}}));

  ASSERT_TRUE(result.publishable);
  EXPECT_FLOAT_EQ(result.left_m, object_detection::kSideClearanceNoReturnM);
  EXPECT_FLOAT_EQ(result.right_m, object_detection::kSideClearanceNoReturnM);
}

TEST(SideClearance, ValidScanAlwaysProducesFiniteNonnegativeDistances)
{
  const auto result = calculate(scanWithPoints({{90.0, 0.32F}}));

  ASSERT_TRUE(result.publishable);
  EXPECT_TRUE(std::isfinite(result.left_m));
  EXPECT_TRUE(std::isfinite(result.right_m));
  EXPECT_GE(result.left_m, 0.0F);
  EXPECT_GE(result.right_m, 0.0F);
}

TEST(SideClearance, MalformedScanIsNotPublishable)
{
  const std::vector<float> empty;
  EXPECT_FALSE(object_detection::calculateSideClearance(
      empty, -kPi, kPi, kPi / 180.0, 0.1F, 10.0F).publishable);

  const auto ranges = scanWithPoints({{90.0, 0.5F}});
  EXPECT_FALSE(object_detection::calculateSideClearance(
      ranges, -kPi, kPi, 0.0, 0.1F, 10.0F).publishable);
  EXPECT_FALSE(object_detection::calculateSideClearance(
      ranges, -kPi, kPi, kPi / 180.0, 10.0F, 0.1F).publishable);
}
