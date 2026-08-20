#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_msgs/msg/header.hpp>

namespace lane_detection
{

struct LanePipelineDiagnostics
{
  std::size_t input_mask_pixels{0};
  std::size_t roi_mask_pixels{0};
  std::size_t roi_after_morphology_pixels{0};
  std::size_t bev_pixels{0};
  std::size_t after_horizontal_pixels{0};
  std::size_t after_column_pixels{0};
  std::size_t corridor_pixels{0};
  std::size_t sliding_points{0};
  std::size_t robust_input_points{0};
  std::size_t robust_retained_points{0};
  int ref_x{0};
  int anchor_x{0};
  int corridor_x_min{0};
  int corridor_x_max{0};
  int base_x{0};
  int consecutive_fail_count_before{0};
  int consecutive_fail_count_after{0};
  int fit_reacquire_after_frames{0};
  std::uint64_t processing_time_us{0};
  bool horizontal_suppressed{false};
  bool column_suppressed{false};
  bool anchor_from_previous{false};
  bool reacquire_active{false};
  bool reacquire_fallback_attempted{false};
  bool reacquire_fallback_candidate{false};
  bool reacquire_fallback_success{false};
  bool fit_valid{false};
  bool frame_mapping_ok{false};
};

inline LanePipelineDiagnostics * diagnosticsDataIfEnabled(
  bool enabled, LanePipelineDiagnostics & data)
{
  return enabled ? &data : nullptr;
}

inline std::string laneInvalidReason(const LanePipelineDiagnostics & data)
{
  if (!data.fit_valid) {
    if (data.input_mask_pixels == 0U) {
      return "EMPTY_INPUT_MASK";
    }
    if (data.roi_mask_pixels == 0U) {
      return "EMPTY_ROI";
    }
    if (data.roi_after_morphology_pixels == 0U) {
      return "REMOVED_BY_MORPHOLOGY";
    }
    if (data.bev_pixels == 0U) {
      return "EMPTY_BEV";
    }
    if (data.after_column_pixels == 0U) {
      return "REMOVED_BY_NOISE_FILTER";
    }
    if (data.corridor_pixels < 10U) {
      return "INSUFFICIENT_CORRIDOR_PIXELS";
    }
    if (data.sliding_points < 10U) {
      return "INSUFFICIENT_SLIDING_POINTS";
    }
    // The current production code has no other fit-invalid branch.  Keep an
    // explicit fallback so a future branch cannot be silently misclassified.
    return "UNCLASSIFIED_FIT_FAILURE";
  }
  if (!data.frame_mapping_ok) {
    return "FRAME_MAPPING_FAILED";
  }
  return "NONE";
}

inline diagnostic_msgs::msg::KeyValue diagnosticValue(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

inline diagnostic_msgs::msg::DiagnosticArray makeLanePipelineDiagnostic(
  const std_msgs::msg::Header & input_header,
  const LanePipelineDiagnostics & data)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header = input_header;

  diagnostic_msgs::msg::DiagnosticStatus status;
  const std::string reason = laneInvalidReason(data);
  status.level = reason == "NONE" ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.name = "lane_detection/pipeline";
  status.message = reason;
  status.hardware_id = "";

  auto add_size = [&status](const std::string & key, std::size_t value) {
      status.values.push_back(diagnosticValue(key, std::to_string(value)));
    };
  auto add_int = [&status](const std::string & key, int value) {
      status.values.push_back(diagnosticValue(key, std::to_string(value)));
    };
  auto add_bool = [&status](const std::string & key, bool value) {
      status.values.push_back(diagnosticValue(key, value ? "true" : "false"));
    };

  add_size("input_mask_pixels", data.input_mask_pixels);
  add_size("roi_mask_pixels", data.roi_mask_pixels);
  add_size("roi_after_morphology_pixels", data.roi_after_morphology_pixels);
  add_size("bev_pixels", data.bev_pixels);
  add_size("after_horizontal_pixels", data.after_horizontal_pixels);
  add_size("after_column_pixels", data.after_column_pixels);
  add_size("corridor_pixels", data.corridor_pixels);
  add_size("sliding_points", data.sliding_points);
  add_size("robust_input_points", data.robust_input_points);
  add_size("robust_retained_points", data.robust_retained_points);
  add_int("ref_x", data.ref_x);
  add_int("anchor_x", data.anchor_x);
  add_int("corridor_x_min", data.corridor_x_min);
  add_int("corridor_x_max", data.corridor_x_max);
  add_int("base_x", data.base_x);
  add_int("consecutive_fail_count_before", data.consecutive_fail_count_before);
  add_int("consecutive_fail_count_after", data.consecutive_fail_count_after);
  add_int("fit_reacquire_after_frames", data.fit_reacquire_after_frames);
  status.values.push_back(
    diagnosticValue("processing_time_us", std::to_string(data.processing_time_us)));
  add_bool("horizontal_suppressed", data.horizontal_suppressed);
  add_bool("column_suppressed", data.column_suppressed);
  add_bool("anchor_from_previous", data.anchor_from_previous);
  add_bool("reacquire_active", data.reacquire_active);
  add_bool("reacquire_fallback_attempted", data.reacquire_fallback_attempted);
  add_bool("reacquire_fallback_candidate", data.reacquire_fallback_candidate);
  add_bool("reacquire_fallback_success", data.reacquire_fallback_success);
  add_bool("fit_valid", data.fit_valid);
  add_bool("frame_mapping_ok", data.frame_mapping_ok);
  status.values.push_back(diagnosticValue("invalid_reason", reason));

  array.status.push_back(std::move(status));
  return array;
}

}  // namespace lane_detection
