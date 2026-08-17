#include <gtest/gtest.h>

#include "lane_detection/lane_measurement_publication_policy.hpp"

TEST(LaneMeasurementPublicationPolicy, InvalidFitPublishesOnlyValidity)
{
  const auto policy = lane_detection::measurementPublicationPolicy(false, true);

  EXPECT_FALSE(policy.publish_offset);
  EXPECT_FALSE(policy.publish_fit);
  EXPECT_FALSE(policy.publish_lane_position);
}

TEST(LaneMeasurementPublicationPolicy, ValidMappedFitPublishesAllMeasurements)
{
  const auto policy = lane_detection::measurementPublicationPolicy(true, true);

  EXPECT_TRUE(policy.publish_offset);
  EXPECT_TRUE(policy.publish_fit);
  EXPECT_TRUE(policy.publish_lane_position);
}

TEST(LaneMeasurementPublicationPolicy, MappingFailureSuppressesOnlyFrameFit)
{
  const auto policy = lane_detection::measurementPublicationPolicy(true, false);

  EXPECT_TRUE(policy.publish_offset);
  EXPECT_FALSE(policy.publish_fit);
  EXPECT_TRUE(policy.publish_lane_position);
}
