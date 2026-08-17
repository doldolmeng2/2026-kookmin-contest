#pragma once

#include <algorithm>
#include <cstddef>

#include "rubbercone/entry_readiness.hpp"

namespace rubbercone
{

enum class SessionPhase
{
  SEARCH_ENTRY,
  CONE_ACTIVE,
};

struct SessionUpdate
{
  bool entry_ready{false};
  bool end_latched{false};
  bool exit_armed{false};
  bool armed_this_sample{false};
  bool latched_this_sample{false};
};

class SessionLifecycle
{
public:
  SessionLifecycle(
    std::size_t arm_valid_frames = 5,
    std::size_t end_missing_frames = 3)
  : arm_valid_frames_(std::max<std::size_t>(1, arm_valid_frames)),
    end_missing_frames_(std::max<std::size_t>(1, end_missing_frames))
  {
  }

  void setEndMissingFrames(std::size_t value)
  {
    end_missing_frames_ = std::max<std::size_t>(1, value);
  }

  bool setActive(bool active)
  {
    const auto requested = active ? SessionPhase::CONE_ACTIVE : SessionPhase::SEARCH_ENTRY;
    if (requested == phase_) {
      return false;
    }
    phase_ = requested;
    resetPhaseState();
    return true;
  }

  void manualResetToSearch()
  {
    phase_ = SessionPhase::SEARCH_ENTRY;
    resetPhaseState();
  }

  SessionUpdate update(bool path_valid, float confidence, double received_at)
  {
    if (phase_ == SessionPhase::SEARCH_ENTRY) {
      exit_armed_ = false;
      end_latched_ = false;
      valid_frame_count_ = 0;
      missing_frame_count_ = 0;
      const int confidence_percent = static_cast<int>(confidence * 100.0F + 0.5F);
      return SessionUpdate{
        entry_readiness_.update(path_valid, confidence_percent, received_at),
        false,
        false,
        false,
        false,
      };
    }

    entry_readiness_.reset();
    if (end_latched_) {
      return SessionUpdate{false, true, true, false, false};
    }

    bool armed_this_sample = false;
    bool latched_this_sample = false;
    if (path_valid) {
      missing_frame_count_ = 0;
      if (!exit_armed_) {
        if (confidence >= 0.55F) {
          ++valid_frame_count_;
        } else {
          valid_frame_count_ = 0;
        }
        if (valid_frame_count_ >= arm_valid_frames_) {
          exit_armed_ = true;
          armed_this_sample = true;
        }
      }
    } else if (!exit_armed_) {
      valid_frame_count_ = 0;
    } else {
      ++missing_frame_count_;
      if (missing_frame_count_ >= end_missing_frames_) {
        end_latched_ = true;
        latched_this_sample = true;
      }
    }

    return SessionUpdate{
      false,
      end_latched_,
      exit_armed_,
      armed_this_sample,
      latched_this_sample,
    };
  }

  SessionPhase phase() const {return phase_;}
  bool active() const {return phase_ == SessionPhase::CONE_ACTIVE;}
  bool entryReady() const {return entry_readiness_.ready();}
  bool exitArmed() const {return exit_armed_;}
  bool endLatched() const {return end_latched_;}
  std::size_t validFrameCount() const {return valid_frame_count_;}
  std::size_t missingFrameCount() const {return missing_frame_count_;}

private:
  void resetPhaseState()
  {
    valid_frame_count_ = 0;
    missing_frame_count_ = 0;
    exit_armed_ = false;
    end_latched_ = false;
    entry_readiness_.reset();
  }

  SessionPhase phase_{SessionPhase::SEARCH_ENTRY};
  std::size_t arm_valid_frames_;
  std::size_t end_missing_frames_;
  std::size_t valid_frame_count_{0};
  std::size_t missing_frame_count_{0};
  bool exit_armed_{false};
  bool end_latched_{false};
  EntryReadiness entry_readiness_;
};

}  // namespace rubbercone
