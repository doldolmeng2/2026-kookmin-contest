#pragma once

#include <cmath>
#include <cstddef>

namespace rubbercone
{

class EntryReadiness
{
public:
  static constexpr int kMinConfidence = 75;
  static constexpr std::size_t kMinQualifyingScans = 3;
  static constexpr double kMinQualifyingDurationS = 0.2;

  void reset()
  {
    qualifying_count_ = 0;
    first_qualifying_at_ = 0.0;
    last_sample_at_ = 0.0;
    has_last_sample_ = false;
    ready_ = false;
  }

  bool update(bool path_valid, int confidence, double received_at)
  {
    if (!std::isfinite(received_at) ||
      (has_last_sample_ && received_at <= last_sample_at_))
    {
      resetCandidate();
      ready_ = false;
      return ready_;
    }

    last_sample_at_ = received_at;
    has_last_sample_ = true;
    if (!path_valid || confidence < kMinConfidence) {
      resetCandidate();
      ready_ = false;
      return ready_;
    }

    if (qualifying_count_ == 0) {
      first_qualifying_at_ = received_at;
    }
    ++qualifying_count_;
    ready_ =
      qualifying_count_ >= kMinQualifyingScans &&
      received_at - first_qualifying_at_ + 1e-9 >= kMinQualifyingDurationS;
    return ready_;
  }

  bool ready() const {return ready_;}
  std::size_t qualifyingCount() const {return qualifying_count_;}

private:
  void resetCandidate()
  {
    qualifying_count_ = 0;
    first_qualifying_at_ = 0.0;
  }

  std::size_t qualifying_count_{0};
  double first_qualifying_at_{0.0};
  double last_sample_at_{0.0};
  bool has_last_sample_{false};
  bool ready_{false};
};

}  // namespace rubbercone
