#include <gtest/gtest.h>

#include "object_detection/lane_stabilizer.hpp"

TEST(LaneStabilizer, ConfirmsThreeMatchingSamplesAcrossPointTwoFiveSeconds)
{
  object_detection::LaneStabilizer stabilizer;

  EXPECT_EQ(stabilizer.update(true, 1, 1.0), 0);
  EXPECT_EQ(stabilizer.update(true, 1, 1.1), 0);
  EXPECT_EQ(stabilizer.update(true, 1, 1.25), 1);
}

TEST(LaneStabilizer, SingleOppositeSampleCannotRetargetStableLane)
{
  object_detection::LaneStabilizer stabilizer;
  stabilizer.update(true, 1, 1.0);
  stabilizer.update(true, 1, 1.1);
  ASSERT_EQ(stabilizer.update(true, 1, 1.25), 1);

  EXPECT_EQ(stabilizer.update(true, 2, 1.4), 1);
  EXPECT_EQ(stabilizer.confirmedLane(), 1);
}

TEST(LaneStabilizer, OppositeLaneRequiresRestabilizationAndRetargetHold)
{
  object_detection::LaneStabilizer stabilizer;
  stabilizer.update(true, 1, 1.0);
  stabilizer.update(true, 1, 1.1);
  ASSERT_EQ(stabilizer.update(true, 1, 1.25), 1);

  EXPECT_EQ(stabilizer.update(true, 2, 1.3), 1);
  EXPECT_EQ(stabilizer.update(true, 2, 1.45), 1);
  EXPECT_EQ(stabilizer.update(true, 2, 1.55), 1);
  EXPECT_EQ(stabilizer.update(true, 2, 1.75), 2);
}

TEST(LaneStabilizer, MissingDetectionPublishesUnknownAndBreaksCandidate)
{
  object_detection::LaneStabilizer stabilizer;
  stabilizer.update(true, 1, 1.0);
  stabilizer.update(true, 1, 1.1);

  EXPECT_EQ(stabilizer.update(false, 0, 1.2), 0);
  EXPECT_EQ(stabilizer.streak(), 0U);
  EXPECT_EQ(stabilizer.update(true, 1, 1.3), 0);
}
