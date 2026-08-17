#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace object_detection
{

struct SideClearanceResult
{
  bool publishable{false};
  float left_m{std::numeric_limits<float>::infinity()};
  float right_m{std::numeric_limits<float>::infinity()};
};

inline SideClearanceResult calculateSideClearance(
  const std::vector<float> & ranges,
  double angle_min,
  double angle_max,
  double angle_increment,
  float range_min,
  float range_max)
{
  SideClearanceResult result;
  if (
    ranges.empty() || !std::isfinite(angle_min) || !std::isfinite(angle_max) ||
    !std::isfinite(angle_increment) || angle_increment <= 0.0 ||
    !std::isfinite(range_min) || !std::isfinite(range_max) ||
    range_min < 0.0F || range_max < range_min || angle_max < angle_min)
  {
    return result;
  }

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kRadiansToDegrees = 180.0 / kPi;
  constexpr double kSideFovMinDegrees = 60.0;
  constexpr double kSideFovMaxDegrees = 100.0;
  constexpr float kSideMaxRangeM = 1.5F;

  result.publishable = true;
  for (std::size_t index = 0; index < ranges.size(); ++index) {
    const float distance = ranges[index];
    if (!std::isfinite(distance) || distance < range_min || distance > range_max ||
      distance > kSideMaxRangeM)
    {
      continue;
    }

    double angle = angle_min + static_cast<double>(index) * angle_increment;
    angle = std::remainder(angle, 2.0 * kPi);
    const double degrees = angle * kRadiansToDegrees;
    const double magnitude = std::abs(degrees);
    if (magnitude < kSideFovMinDegrees || magnitude > kSideFovMaxDegrees) {
      continue;
    }

    if (degrees > 0.0) {
      result.left_m = std::min(result.left_m, distance);
    } else if (degrees < 0.0) {
      result.right_m = std::min(result.right_m, distance);
    }
  }
  return result;
}

}  // namespace object_detection
