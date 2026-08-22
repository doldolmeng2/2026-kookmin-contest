#include <gtest/gtest.h>

#include "lane_detection/lane_measurement_publication_policy.hpp"
#include "lane_detection/lane_pipeline_diagnostics.hpp"

namespace
{

std::string valueFor(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key)
{
  for (const auto & value : status.values) {
    if (value.key == key) {
      return value.value;
    }
  }
  return "";
}

}  // namespace

TEST(LanePipelineDiagnostics, DisabledCollectionUsesNullPointer)
{
  lane_detection::LanePipelineDiagnostics data;
  EXPECT_EQ(nullptr, lane_detection::diagnosticsDataIfEnabled(false, data));
  EXPECT_EQ(&data, lane_detection::diagnosticsDataIfEnabled(true, data));
}

TEST(LanePipelineDiagnostics, CopiesInputHeaderStampExactly)
{
  std_msgs::msg::Header header;
  header.stamp.sec = 123;
  header.stamp.nanosec = 456789U;
  header.frame_id = "pidnet_mask";
  lane_detection::LanePipelineDiagnostics data;
  data.input_mask_pixels = 100U;
  data.roi_mask_pixels = 80U;
  data.roi_after_morphology_pixels = 80U;
  data.bev_pixels = 120U;
  data.after_column_pixels = 100U;
  data.corridor_pixels = 90U;
  data.sliding_points = 70U;
  data.robust_input_points = 70U;
  data.robust_retained_points = 60U;
  data.fit_valid = true;
  data.frame_mapping_ok = true;

  const auto message = lane_detection::makeLanePipelineDiagnostic(header, data);

  ASSERT_EQ(1U, message.status.size());
  EXPECT_EQ(header.stamp.sec, message.header.stamp.sec);
  EXPECT_EQ(header.stamp.nanosec, message.header.stamp.nanosec);
  EXPECT_EQ(header.frame_id, message.header.frame_id);
  EXPECT_EQ("NONE", message.status.front().message);
  EXPECT_EQ("true", valueFor(message.status.front(), "fit_valid"));
}

TEST(LanePipelineDiagnostics, UsesOnlyExistingFitFailureCondition)
{
  lane_detection::LanePipelineDiagnostics data;
  data.input_mask_pixels = 1000U;
  data.roi_mask_pixels = 900U;
  data.roi_after_morphology_pixels = 850U;
  data.bev_pixels = 700U;
  data.after_column_pixels = 600U;
  data.corridor_pixels = 500U;
  data.sliding_points = 9U;
  data.fit_valid = false;

  EXPECT_EQ("INSUFFICIENT_SLIDING_POINTS", lane_detection::laneInvalidReason(data));
}

TEST(LanePipelineDiagnostics, DiagnosticsDoNotChangePublicationPolicy)
{
  lane_detection::LanePipelineDiagnostics data;
  data.fit_valid = false;
  data.frame_mapping_ok = false;
  const auto invalid_policy =
    lane_detection::measurementPublicationPolicy(data.fit_valid, data.frame_mapping_ok);
  EXPECT_FALSE(invalid_policy.publish_offset);

  data.fit_valid = true;
  data.frame_mapping_ok = true;
  const auto valid_policy =
    lane_detection::measurementPublicationPolicy(data.fit_valid, data.frame_mapping_ok);
  EXPECT_TRUE(valid_policy.publish_offset);
}
