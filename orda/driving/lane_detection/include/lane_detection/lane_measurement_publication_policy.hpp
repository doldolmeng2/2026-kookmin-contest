#pragma once

namespace lane_detection
{

struct LaneMeasurementPublicationPolicy
{
  bool publish_offset{false};
  bool publish_fit{false};
  bool publish_lane_position{false};
};

inline LaneMeasurementPublicationPolicy measurementPublicationPolicy(
  bool fit_valid, bool frame_fit_mapped)
{
  if (!fit_valid) {
    return {};
  }
  return {true, frame_fit_mapped, true};
}

}  // namespace lane_detection
