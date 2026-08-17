#include <gtest/gtest.h>

#include "rubbercone/session_lifecycle.hpp"

namespace
{

void armExit(rubbercone::SessionLifecycle & lifecycle, double start)
{
  for (int index = 0; index < 5; ++index) {
    const auto update = lifecycle.update(true, 0.80F, start + index * 0.1);
    EXPECT_FALSE(update.entry_ready);
    EXPECT_FALSE(update.end_latched);
  }
  ASSERT_TRUE(lifecycle.exitArmed());
}

void latchEnd(rubbercone::SessionLifecycle & lifecycle, double start)
{
  for (int index = 0; index < 3; ++index) {
    lifecycle.update(false, 0.0F, start + index * 0.1);
  }
  ASSERT_TRUE(lifecycle.endLatched());
}

}  // namespace

TEST(SessionLifecycle, StartupSearchNeverArmsOrLatchesExit)
{
  rubbercone::SessionLifecycle lifecycle;

  for (int index = 0; index < 5; ++index) {
    const auto update = lifecycle.update(true, 0.80F, 1.0 + index * 0.1);
    EXPECT_FALSE(update.exit_armed);
    EXPECT_FALSE(update.end_latched);
  }
  for (int index = 0; index < 3; ++index) {
    const auto update = lifecycle.update(false, 0.0F, 2.0 + index * 0.1);
    EXPECT_FALSE(update.exit_armed);
    EXPECT_FALSE(update.end_latched);
  }
}

TEST(SessionLifecycle, SearchTrailingGeometryCannotLatchAndCanStartNextEntry)
{
  rubbercone::SessionLifecycle lifecycle;

  lifecycle.update(true, 0.80F, 1.0);
  lifecycle.update(true, 0.80F, 1.1);
  lifecycle.update(true, 0.80F, 1.2);
  for (int index = 0; index < 3; ++index) {
    EXPECT_FALSE(lifecycle.update(false, 0.0F, 1.3 + index * 0.1).end_latched);
  }

  EXPECT_FALSE(lifecycle.update(true, 0.80F, 2.0).entry_ready);
  EXPECT_FALSE(lifecycle.update(true, 0.80F, 2.1).entry_ready);
  EXPECT_TRUE(lifecycle.update(true, 0.80F, 2.2).entry_ready);
  EXPECT_FALSE(lifecycle.endLatched());
}

TEST(SessionLifecycle, SearchRequiresExistingEntryReadinessContract)
{
  rubbercone::SessionLifecycle lifecycle;

  EXPECT_FALSE(lifecycle.update(true, 0.75F, 1.0).entry_ready);
  EXPECT_FALSE(lifecycle.update(true, 0.90F, 1.1).entry_ready);
  EXPECT_TRUE(lifecycle.update(true, 0.80F, 1.2).entry_ready);
}

TEST(SessionLifecycle, ActiveCommandClearsEntryAndStartsFreshExitState)
{
  rubbercone::SessionLifecycle lifecycle;
  lifecycle.update(true, 0.80F, 1.0);
  lifecycle.update(true, 0.80F, 1.1);
  ASSERT_TRUE(lifecycle.update(true, 0.80F, 1.2).entry_ready);

  ASSERT_TRUE(lifecycle.setActive(true));

  EXPECT_TRUE(lifecycle.active());
  EXPECT_FALSE(lifecycle.entryReady());
  EXPECT_FALSE(lifecycle.exitArmed());
  EXPECT_FALSE(lifecycle.endLatched());
  EXPECT_FALSE(lifecycle.update(true, 0.90F, 2.0).entry_ready);
}

TEST(SessionLifecycle, ActiveExitArmsThenLatchesAfterThreeMissingFrames)
{
  rubbercone::SessionLifecycle lifecycle;
  ASSERT_TRUE(lifecycle.setActive(true));
  armExit(lifecycle, 1.0);

  EXPECT_FALSE(lifecycle.update(false, 0.0F, 2.0).end_latched);
  EXPECT_FALSE(lifecycle.update(false, 0.0F, 2.1).end_latched);
  const auto latched = lifecycle.update(false, 0.0F, 2.2);

  EXPECT_TRUE(latched.latched_this_sample);
  EXPECT_TRUE(latched.end_latched);
}

TEST(SessionLifecycle, ActiveEndToSearchClearsLatchAndAllowsNewEntry)
{
  rubbercone::SessionLifecycle lifecycle;
  lifecycle.setActive(true);
  armExit(lifecycle, 1.0);
  latchEnd(lifecycle, 2.0);

  ASSERT_TRUE(lifecycle.setActive(false));

  EXPECT_FALSE(lifecycle.active());
  EXPECT_FALSE(lifecycle.exitArmed());
  EXPECT_FALSE(lifecycle.endLatched());
  EXPECT_FALSE(lifecycle.update(true, 0.80F, 3.0).entry_ready);
  EXPECT_FALSE(lifecycle.update(true, 0.80F, 3.1).entry_ready);
  EXPECT_TRUE(lifecycle.update(true, 0.80F, 3.2).entry_ready);
}

TEST(SessionLifecycle, ThreeSessionsNeverReuseLatchOrCounters)
{
  rubbercone::SessionLifecycle lifecycle;

  for (int session = 0; session < 3; ++session) {
    const double base = 10.0 * session;
    EXPECT_FALSE(lifecycle.update(true, 0.80F, base + 1.0).entry_ready);
    EXPECT_FALSE(lifecycle.update(true, 0.80F, base + 1.1).entry_ready);
    EXPECT_TRUE(lifecycle.update(true, 0.80F, base + 1.2).entry_ready);

    ASSERT_TRUE(lifecycle.setActive(true));
    armExit(lifecycle, base + 2.0);
    latchEnd(lifecycle, base + 3.0);
    ASSERT_TRUE(lifecycle.setActive(false));

    EXPECT_FALSE(lifecycle.entryReady());
    EXPECT_FALSE(lifecycle.exitArmed());
    EXPECT_FALSE(lifecycle.endLatched());
  }
}
