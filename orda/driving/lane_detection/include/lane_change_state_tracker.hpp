#pragma once

#include <cmath>

namespace lane_detection
{

struct LaneChangeFeedback
{
  int changing;
  int success;
};


class LaneChangeStateTracker
{
public:
  LaneChangeStateTracker(
    float settle_straight,
    float settle_curve,
    float change_spike,
    int settle_frames,
    float curve_slope_split)
  : settle_straight_(settle_straight),
    settle_curve_(settle_curve),
    change_spike_(change_spike),
    settle_frames_(settle_frames),
    curve_slope_split_(curve_slope_split) {}

  void handleCommand(int mode, int lane_target)
  {
    // 0=중앙, 1=왼쪽, 2=오른쪽 (README 규약)
    const bool valid_target = lane_target >= 0 && lane_target <= 2;
    const bool valid_change_command = mode == 5 && valid_target;

    if (!valid_change_command) {
      command_latched_ = false;
      action_active_ = false;
      lane_target_ = -1;
      resetProgress();
      return;
    }

    if (!command_latched_ || lane_target != lane_target_) {
      command_latched_ = true;
      action_active_ = true;
      lane_target_ = lane_target;
      resetProgress();
    }
  }

  LaneChangeFeedback update(bool lane_valid, float center_slope, float offset)
  {
    if (!action_active_) {
      return {0, 0};
    }

    if (!lane_valid || !std::isfinite(center_slope) || !std::isfinite(offset)) {
      resetProgress();
      return {1, 0};
    }

    const float absolute_offset = std::abs(offset);
    const bool is_curve = std::abs(center_slope) >= curve_slope_split_;
    const float settle_tolerance = is_curve ? settle_curve_ : settle_straight_;

    if (phase_ == Phase::WAIT_SPIKE) {
      if (absolute_offset >= change_spike_) {
        phase_ = Phase::WAIT_SETTLE;
        stable_streak_ = 0;
      }
      return {1, 0};
    }

    if (absolute_offset <= settle_tolerance) {
      ++stable_streak_;
    } else {
      stable_streak_ = 0;
    }

    if (stable_streak_ >= settle_frames_) {
      action_active_ = false;
      resetProgress();
      return {1, 1};
    }
    return {1, 0};
  }

private:
  enum class Phase { WAIT_SPIKE, WAIT_SETTLE };

  void resetProgress()
  {
    phase_ = Phase::WAIT_SPIKE;
    stable_streak_ = 0;
  }

  float settle_straight_;
  float settle_curve_;
  float change_spike_;
  int settle_frames_;
  float curve_slope_split_;
  bool command_latched_ = false;
  bool action_active_ = false;
  int lane_target_ = -1;
  Phase phase_ = Phase::WAIT_SPIKE;
  int stable_streak_ = 0;
};

}  // namespace lane_detection
