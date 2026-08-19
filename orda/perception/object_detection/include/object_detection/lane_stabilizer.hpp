#pragma once

#include <cmath>
#include <cstddef>

namespace object_detection
{

class LaneStabilizer
{
public:
  static constexpr std::size_t kMinConsecutiveSamples = 3;
  static constexpr double kMinSampleDurationS = 0.25;
  static constexpr double kRetargetHoldS = 0.5;

  int update(bool detected, int raw_lane, double received_at)
  {
    if (!std::isfinite(received_at) ||
      (has_last_sample_ && received_at <= last_sample_at_))
    {
      breakCandidate();
      return detected ? confirmed_lane_ : 0;
    }

    last_sample_at_ = received_at;
    has_last_sample_ = true;
    if (!detected) {
      breakCandidate();
      return 0;
    }
    if (raw_lane != 1 && raw_lane != 2) {
      breakCandidate();
      return confirmed_lane_;
    }

    if (candidate_lane_ == raw_lane) {
      ++streak_;
    } else {
      candidate_lane_ = raw_lane;
      streak_ = 1;
      streak_started_at_ = received_at;
    }

    const double elapsed = received_at - streak_started_at_;
    const bool candidate_stable =
      streak_ >= kMinConsecutiveSamples &&
      elapsed + 1e-9 >= kMinSampleDurationS;
    const bool retarget_hold_elapsed =
      confirmed_lane_ == 0 ||
      received_at - confirmed_at_ + 1e-9 >= kRetargetHoldS;
    if (candidate_stable &&
      candidate_lane_ != confirmed_lane_ &&
      retarget_hold_elapsed)
    {
      confirmed_lane_ = candidate_lane_;
      confirmed_at_ = received_at;
    }
    return confirmed_lane_;
  }

  void reset()
  {
    confirmed_lane_ = 0;
    confirmed_at_ = 0.0;
    candidate_lane_ = 0;
    streak_ = 0;
    streak_started_at_ = 0.0;
    last_sample_at_ = 0.0;
    has_last_sample_ = false;
  }

  int confirmedLane() const {return confirmed_lane_;}
  std::size_t streak() const {return streak_;}

private:
  void breakCandidate()
  {
    candidate_lane_ = 0;
    streak_ = 0;
    streak_started_at_ = 0.0;
  }

  int confirmed_lane_{0};
  double confirmed_at_{0.0};
  int candidate_lane_{0};
  std::size_t streak_{0};
  double streak_started_at_{0.0};
  double last_sample_at_{0.0};
  bool has_last_sample_{false};
};

}  // namespace object_detection
