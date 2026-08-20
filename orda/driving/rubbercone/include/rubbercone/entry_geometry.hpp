#pragma once

namespace rubbercone
{

inline constexpr bool entryGeometryValid(int left_count, int right_count)
{
  return left_count >= 2 && right_count >= 2;
}

inline constexpr bool lifecyclePathValid(
  bool session_active,
  bool path_valid,
  bool entry_geometry_valid)
{
  return session_active ? path_valid : entry_geometry_valid;
}

}  // namespace rubbercone
