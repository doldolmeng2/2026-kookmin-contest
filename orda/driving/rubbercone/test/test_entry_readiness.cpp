#include <gtest/gtest.h>

#include "rubbercone/entry_geometry.hpp"
#include "rubbercone/entry_readiness.hpp"

TEST(EntryReadiness, RequiresThreeQualifyingScansAcrossPointTwoSeconds)
{
  rubbercone::EntryReadiness readiness;

  EXPECT_FALSE(readiness.update(true, 75, 1.0));
  EXPECT_FALSE(readiness.update(true, 90, 1.1));
  EXPECT_TRUE(readiness.update(true, 80, 1.2));
}

TEST(EntryReadiness, LowConfidenceOrInvalidPathResetsCandidateSequence)
{
  rubbercone::EntryReadiness readiness;

  EXPECT_FALSE(readiness.update(true, 90, 1.0));
  EXPECT_FALSE(readiness.update(true, 90, 1.1));
  EXPECT_FALSE(readiness.update(true, 74, 1.2));
  EXPECT_FALSE(readiness.update(true, 90, 1.3));
  EXPECT_FALSE(readiness.update(false, 100, 1.4));
  EXPECT_EQ(readiness.qualifyingCount(), 0U);
}

TEST(EntryReadiness, ResetClearsAnApprovedSession)
{
  rubbercone::EntryReadiness readiness;
  readiness.update(true, 90, 1.0);
  readiness.update(true, 90, 1.1);
  ASSERT_TRUE(readiness.update(true, 90, 1.2));

  readiness.reset();

  EXPECT_FALSE(readiness.ready());
  EXPECT_EQ(readiness.qualifyingCount(), 0U);
}

TEST(EntryGeometry, SearchRejectsOneByThreeBilateralGeometry)
{
  rubbercone::EntryReadiness readiness;
  const bool geometry_valid = rubbercone::entryGeometryValid(1, 3);

  EXPECT_FALSE(geometry_valid);
  EXPECT_FALSE(readiness.update(geometry_valid, 94, 1.0));
  EXPECT_FALSE(readiness.update(geometry_valid, 94, 1.1));
  EXPECT_FALSE(readiness.update(geometry_valid, 94, 1.2));
  EXPECT_EQ(readiness.qualifyingCount(), 0U);
}

TEST(EntryGeometry, SearchAcceptsTwoByTwoGeometry)
{
  rubbercone::EntryReadiness readiness;
  const bool geometry_valid = rubbercone::entryGeometryValid(2, 2);

  ASSERT_TRUE(geometry_valid);
  EXPECT_FALSE(readiness.update(geometry_valid, 94, 1.0));
  EXPECT_FALSE(readiness.update(geometry_valid, 94, 1.1));
  EXPECT_TRUE(readiness.update(geometry_valid, 94, 1.2));
}

TEST(EntryGeometry, SearchRejectsValidOneSidePath)
{
  EXPECT_FALSE(rubbercone::lifecyclePathValid(false, true, false));
}

TEST(EntryGeometry, ActiveSessionPreservesValidOneSidePath)
{
  EXPECT_TRUE(rubbercone::lifecyclePathValid(true, true, false));
}
