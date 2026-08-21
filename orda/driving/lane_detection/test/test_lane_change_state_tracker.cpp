#include <gtest/gtest.h>

#include "lane_change_state_tracker.hpp"

namespace
{

using lane_detection::LaneChangeFeedback;
using lane_detection::LaneChangeStateTracker;

void expectFeedback(const LaneChangeFeedback & feedback, int changing, int success)
{
  EXPECT_EQ(feedback.changing, changing);
  EXPECT_EQ(feedback.success, success);
}

LaneChangeStateTracker tracker()
{
  return LaneChangeStateTracker(
    50.0F,       // straight settle tolerance
    300.0F,      // curve settle tolerance
    400.0F,      // change spike threshold
    3,           // consecutive settle frames
    0.1F);       // curve slope split
}

TEST(LaneChangeStateTrackerTest, PublishesOneSuccessEdgePerCommand) {
  auto state = tracker();

  expectFeedback(state.update(true, 0.0F, 0.0F), 0, 0);

  state.handleCommand(5, 1);
  expectFeedback(state.update(true, 0.0F, 100.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 450.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 1);

  // Repeated mode-5 messages for the same target are the same action.
  state.handleCommand(5, 1);
  expectFeedback(state.update(true, 0.0F, 450.0F), 0, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 0, 0);
}

TEST(LaneChangeStateTrackerTest, CancelResetsAndASecondActionCanComplete) {
  auto state = tracker();

  state.handleCommand(5, 0);
  expectFeedback(state.update(true, 0.0F, 450.0F), 1, 0);
  state.handleCommand(3, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 0, 0);

  state.handleCommand(5, 0);
  expectFeedback(state.update(true, 0.0F, 450.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 1);
}

TEST(LaneChangeStateTrackerTest, InvalidTargetAndInvalidFitCannotSucceed) {
  auto state = tracker();

  state.handleCommand(5, 2);
  expectFeedback(state.update(true, 0.0F, 450.0F), 0, 0);

  state.handleCommand(5, 0);
  expectFeedback(state.update(false, 0.0F, 450.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);

  expectFeedback(state.update(true, 0.0F, 450.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 1);
}

TEST(LaneChangeStateTrackerTest, TargetChangeStartsANewCommandEpoch) {
  auto state = tracker();

  state.handleCommand(5, 0);
  expectFeedback(state.update(true, 0.0F, 450.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);

  state.handleCommand(5, 1);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 450.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 0);
  expectFeedback(state.update(true, 0.0F, 0.0F), 1, 1);
}

}  // namespace
