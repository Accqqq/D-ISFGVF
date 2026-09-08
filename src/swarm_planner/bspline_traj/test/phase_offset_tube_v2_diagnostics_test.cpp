#include <gtest/gtest.h>

#include "bspline_race/integration/phase_offset_tube_v2_diagnostics.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace FLAG_Race {
namespace {

phase_offset_navigation::TubePathKey PathKey() {
  phase_offset_navigation::TubePathKey key;
  key.execution_generation = 0x100000000ULL;
  key.path_instance_id = 0x200000000ULL;
  key.path_revision = 7U;
  key.frame_revision = 8U;
  key.frame_convention_id = 9U;
  key.frame_convention = "world_horizontal_cross_product_v1";
  key.phase_orientation = 1;
  key.domain_start = 0.0;
  key.domain_end = 2.0;
  return key;
}

phase_offset_navigation::TubeConfigurationKey ConfigurationKey() {
  phase_offset_navigation::TubeConfigurationKey key;
  key.configuration_id = 0x300000000ULL;
  key.epsilon = 0.4;
  key.nominal_half_width = 1.0;
  key.ray_step = 0.05;
  key.snapshot_resolution = 0.05;
  key.minimum_reference_speed = 1e-8;
  return key;
}

phase_offset_navigation::TubeMapCaptureKey MapKey() {
  phase_offset_navigation::TubeMapCaptureKey key;
  key.map_instance_id = 0x400000000ULL;
  key.state_id = 0x500000000ULL;
  key.accepted_sequence = 0x600000000ULL;
  key.configuration_generation = 0x700000000ULL;
  key.configuration_id = 0x300000000ULL;
  key.frame_provenance_id = 0x800000000ULL;
  key.frame_provenance = "sdfmap-effective-backing";
  key.support_provenance_id = std::numeric_limits<std::uint64_t>::max();
  key.accepted_time_ticks = 1000U;
  key.support_expiry_ticks = 2000U;
  key.support_halo = 0.25;
  key.halo_reconciled = true;
  key.grid_min_index_x = -2;
  key.grid_min_index_y = -3;
  key.grid_min_index_z = -4;
  key.grid_max_index_x = 4;
  key.grid_max_index_y = 5;
  key.grid_max_index_z = 6;
  key.grid_voxel_resolution = Eigen::Vector3d::Constant(0.05);
  key.complete_support = true;
  return key;
}

phase_offset_navigation::TubeProfileV2 Profile() {
  phase_offset_navigation::TubeProfileV2 profile;
  profile.path_key = PathKey();
  profile.configuration_key = ConfigurationKey();
  profile.map_capture_key = MapKey();
  profile.profile_id = 0xa00000000ULL;
  profile.request_id = 0xb00000000ULL;
  profile.requested_start = 0.0;
  profile.requested_end = 2.0;
  profile.anchor_w = 0.5;
  profile.certified_start = 0.0;
  profile.certified_end = 2.0;
  profile.valid = true;
  profile.complete = true;
  profile.contains_anchor = true;
  profile.contains_zero_everywhere = true;
  profile.nonzero_capacity = true;
  profile.capability =
      phase_offset_navigation::TubeProfileV2Capability::OFFSET_CERTIFIED;
  phase_offset_navigation::TubePwlKnotV2 first;
  first.w = 0.0;
  first.lower = -0.2;
  first.upper = 0.2;
  first.right_cell_id = 11U;
  first.valid = true;
  phase_offset_navigation::TubePwlKnotV2 second = first;
  second.w = 2.0;
  second.left_cell_id = 11U;
  second.right_cell_id = 0U;
  profile.knots = {first, second};
  return profile;
}

TubeWorkerCompletionV2 Completion(const TubeWorkerCompletionStatusV2 status) {
  TubeWorkerCompletionV2 completion;
  completion.status = status;
  completion.purpose = TubeWorkerPurposeV2::CURRENT;
  completion.request_id = 0xb00000000ULL;
  completion.execution_generation = 0x100000000ULL;
  completion.accepted_state_demand = 0x600000000ULL;
  completion.useful_start = 0.0;
  completion.useful_end = 2.0;
  completion.path_key = PathKey();
  completion.configuration_key = ConfigurationKey();
  completion.map_capture_key = MapKey();
  completion.build_start_ticks = 100U;
  completion.build_end_ticks = 150U;
  completion.build_duration_ns = 50U;
  completion.path_callback_count = 3U;
  completion.path_callback_duration_ns = 31U;
  completion.producer_breakpoint_callback_count = 2U;
  completion.producer_breakpoint_callback_duration_ns = 17U;
  completion.free_ball_callback_count = 4U;
  completion.free_ball_callback_duration_ns = 29U;
  completion.heavy_build_entered = status == TubeWorkerCompletionStatusV2::BUILT;
  completion.cancellation_requested =
      status == TubeWorkerCompletionStatusV2::CANCELLED;
  completion.error_message = status == TubeWorkerCompletionStatusV2::BUILT
      ? std::string() : "source-recorded unavailable";
  completion.build.profile = Profile();
  completion.build.success = status == TubeWorkerCompletionStatusV2::BUILT;
  completion.build.stats.path_cell_query_count = 13U;
  completion.build.stats.failed_path_cell_query_count = 1U;
  completion.build.stats.query_count = 17U;
  completion.build.stats.failed_query_count = 2U;
  completion.build.stats.child_query_count = 5U;
  completion.build.stats.cell_count = 19U;
  completion.build.stats.scheduled_cell_count = 21U;
  completion.build.stats.accepted_cell_count = 18U;
  completion.build.stats.witness_count = 23U;
  completion.build.stats.max_depth_observed = 4;
  completion.build.stats.query_budget_reached = false;
  completion.build.stats.cell_budget_reached = false;
  completion.build.stats.witness_budget_reached = false;
  completion.build.stats.sample_budget_reached = false;
  if (status != TubeWorkerCompletionStatusV2::BUILT) {
    completion.build.profile.valid = false;
    completion.build.profile.complete = false;
    completion.build.profile.capability =
        phase_offset_navigation::TubeProfileV2Capability::UNAVAILABLE;
  }
  return completion;
}

TubeWorkerStatsV2 Stats() {
  TubeWorkerStatsV2 stats;
  stats.submitted = 7U;
  stats.accepted = 6U;
  stats.rejected = 1U;
  stats.coalesced = 2U;
  stats.less_useful_rejected = 1U;
  stats.build_started = 3U;
  stats.build_finished = 3U;
  stats.build_succeeded = 1U;
  stats.build_failed = 2U;
  stats.path_callback_count = 9U;
  stats.path_callback_duration_ns = 101U;
  stats.producer_breakpoint_callback_count = 5U;
  stats.producer_breakpoint_callback_duration_ns = 41U;
  stats.free_ball_callback_count = 11U;
  stats.free_ball_callback_duration_ns = 61U;
  stats.cancellation_requested = 2U;
  stats.cancellation_discarded = 1U;
  stats.stale_discarded = 1U;
  stats.delivery_count = 1U;
  stats.delivery_discarded = 1U;
  stats.event_log_dropped = 0U;
  stats.peak_in_flight = 1U;
  return stats;
}

std::vector<TubeWorkerEventV2> Events() {
  TubeWorkerEventV2 current;
  current.type = TubeWorkerEventTypeV2::SUBMITTED;
  current.timestamp_ticks = 10U;
  current.purpose = TubeWorkerPurposeV2::CURRENT;
  current.request_id = 0xb00000000ULL;
  current.execution_generation = 0x100000000ULL;
  current.accepted_state_demand = 0x600000000ULL;
  TubeWorkerEventV2 successor = current;
  successor.type = TubeWorkerEventTypeV2::BUILD_FINISHED;
  successor.timestamp_ticks = 20U;
  successor.purpose = TubeWorkerPurposeV2::SUCCESSOR;
  successor.request_id = 0xc00000000ULL;
  successor.heavy_build_entered = true;
  successor.build_duration_ns = 45U;
  TubeWorkerEventV2 delivered = current;
  delivered.type = TubeWorkerEventTypeV2::COMPLETION_DELIVERED;
  delivered.timestamp_ticks = 30U;
  return {current, successor, delivered};
}

TubeV2MetricAttribution Metrics() {
  TubeV2MetricAttribution metrics;
  metrics.closed_inflated_voxel_volume_metric = true;
  metrics.epsilon_present = true;
  metrics.epsilon = 0.4;
  metrics.epsilon_unchanged = true;
  metrics.effective_map_inflation_axes_present = true;
  metrics.effective_map_inflation_axes = Eigen::Vector3d(0.2, 0.2, 0.35);
  metrics.observed_obstacle_containment_proven = false;
  metrics.vehicle_tracking_budget_proven = false;
  metrics.map_inflation_provenance = "sdfmap-layered-effective-backing";
  metrics.support_complete = true;
  metrics.support_halo_present = true;
  metrics.support_halo = 0.25;
  metrics.support_expiry_ticks = 2000U;
  metrics.support_provenance_id = 0x900000000ULL;
  metrics.frame_provenance = "sdfmap-effective-backing";
  return metrics;
}

TEST(PhaseOffsetTubeV2Diagnostics, SchemaAndCanonicalFormattingAreStable) {
  const TubeWorkerCompletionV2 completion = Completion(
      TubeWorkerCompletionStatusV2::BUILT);
  TubeV2DiagnosticsSnapshot snapshot = makeTubeV2DiagnosticsSnapshot(
      &completion, Stats(), Events(), true, Metrics());
  ASSERT_EQ(snapshot.capability, TubeV2CapabilityState::CERTIFIED_PROFILE);
  ASSERT_TRUE(validateTubeV2DiagnosticsSnapshot(snapshot));
  ASSERT_EQ(snapshot.events.size(), 3U);
  EXPECT_EQ(snapshot.events[0].purpose, TubeWorkerPurposeV2::CURRENT);
  EXPECT_EQ(snapshot.events[1].purpose, TubeWorkerPurposeV2::SUCCESSOR);
  EXPECT_EQ(snapshot.stats.value.build_started, 3U);
  EXPECT_EQ(snapshot.completion.path_key.path_instance_id, 0x200000000ULL);
  EXPECT_EQ(snapshot.completion.map_capture_key.state_id, 0x500000000ULL);
  EXPECT_EQ(snapshot.completion.build_path_cell_query_count, 13U);
  EXPECT_EQ(snapshot.completion.build_failed_path_cell_query_count, 1U);
  EXPECT_EQ(snapshot.completion.build_query_count, 17U);
  EXPECT_EQ(snapshot.completion.build_failed_query_count, 2U);
  EXPECT_EQ(snapshot.completion.build_child_query_count, 5U);
  EXPECT_EQ(snapshot.completion.build_cell_count, 19U);
  EXPECT_EQ(snapshot.completion.build_scheduled_cell_count, 21U);
  EXPECT_EQ(snapshot.completion.build_accepted_cell_count, 18U);
  EXPECT_EQ(snapshot.completion.build_witness_count, 23U);
  EXPECT_EQ(snapshot.completion.build_max_depth_observed, 4);
  EXPECT_TRUE(snapshot.metrics.effective_map_inflation_axes_present);
  EXPECT_FALSE(snapshot.metrics.physical_containment_proven);
  const std::string first = formatTubeV2DiagnosticsCsv(snapshot);
  const std::string second = formatTubeV2DiagnosticsCsv(snapshot);
  EXPECT_EQ(first, second);
  EXPECT_NE(first.find("tube_v2_diagnostics_v1"), std::string::npos);
  EXPECT_NE(first.find("18446744073709551615"), std::string::npos)
      << "64-bit identity fields must remain decimal value records";
  const std::vector<std::string>& names = tubeV2DiagnosticsCsvFieldNames();
  EXPECT_EQ(std::count(names.begin(), names.end(), "schema_version"), 1);
  EXPECT_EQ(first.find("boundary_live_k_present"),
            tubeV2DiagnosticsCsvHeader().find("boundary_live_k_present"));
  EXPECT_NE(first.find("not_applicable"), std::string::npos);
}

TEST(PhaseOffsetTubeV2Diagnostics, FailedCancelledAndStaleAreUnavailable) {
  for (const TubeWorkerCompletionStatusV2 status : {
           TubeWorkerCompletionStatusV2::FAILED,
           TubeWorkerCompletionStatusV2::CANCELLED,
           TubeWorkerCompletionStatusV2::STALE}) {
    const TubeWorkerCompletionV2 completion = Completion(status);
    const TubeV2DiagnosticsSnapshot snapshot = makeTubeV2DiagnosticsSnapshot(
        &completion, Stats(), Events(), true, TubeV2MetricAttribution());
    EXPECT_EQ(snapshot.capability, TubeV2CapabilityState::UNAVAILABLE);
    EXPECT_TRUE(validateTubeV2DiagnosticsSnapshot(snapshot));
    EXPECT_NE(formatTubeV2DiagnosticsCsv(snapshot).find(
                   tubeWorkerCompletionStatusName(status)),
               std::string::npos);
  }
  const TubeV2DiagnosticsSnapshot planner_only =
      makeTubeV2DiagnosticsSnapshot(nullptr, TubeWorkerStatsV2(), {}, true);
  EXPECT_EQ(planner_only.capability, TubeV2CapabilityState::PLANNER_ONLY);
  EXPECT_TRUE(validateTubeV2DiagnosticsSnapshot(planner_only));
}

TEST(PhaseOffsetTubeV2Diagnostics, SnapshotCopiesFactsAndRejectsLiveKOrBadOrder) {
  TubeWorkerCompletionV2 completion = Completion(
      TubeWorkerCompletionStatusV2::BUILT);
  const TubeV2DiagnosticsSnapshot snapshot = makeTubeV2DiagnosticsSnapshot(
      &completion, Stats(), Events(), false, Metrics());
  completion.request_id = 1U;
  completion.build.profile.knots.front().lower = -9.0;
  EXPECT_EQ(snapshot.completion.request_id, 0xb00000000ULL);
  EXPECT_DOUBLE_EQ(snapshot.completion.profile.boundaries.front().lower, -0.2);

  TubeV2DiagnosticsSnapshot bad = snapshot;
  bad.completion.profile.boundaries.front().live_k_present = true;
  std::string reason;
  EXPECT_FALSE(validateTubeV2DiagnosticsSnapshot(bad, &reason));
  EXPECT_EQ(reason, "invalid certified boundary or fabricated live K");

  bad = snapshot;
  bad.events[1].ordinal = 0U;
  EXPECT_FALSE(validateTubeV2DiagnosticsSnapshot(bad, &reason));
  EXPECT_EQ(reason, "event order/ordinal mismatch");
}

TEST(PhaseOffsetTubeV2Diagnostics, PhysicalContainmentNeedsAxesMetricAndSupport) {
  TubeWorkerCompletionV2 completion = Completion(
      TubeWorkerCompletionStatusV2::BUILT);
  TubeV2MetricAttribution metrics = Metrics();
  metrics.physical_containment_proven = true;
  metrics.effective_map_inflation_axes_present = false;
  TubeV2DiagnosticsSnapshot snapshot = makeTubeV2DiagnosticsSnapshot(
      &completion, Stats(), Events(), false, metrics);
  std::string reason;
  EXPECT_FALSE(validateTubeV2DiagnosticsSnapshot(snapshot, &reason));
  EXPECT_EQ(reason,
            "physical containment claimed without closed metric/axes/support");
  metrics.effective_map_inflation_axes_present = true;
  metrics.observed_obstacle_containment_proven = true;
  metrics.vehicle_tracking_budget_proven = true;
  snapshot = makeTubeV2DiagnosticsSnapshot(&completion, Stats(), Events(), false,
                                           metrics);
  EXPECT_TRUE(validateTubeV2DiagnosticsSnapshot(snapshot));
}

TEST(PhaseOffsetTubeV2Diagnostics, RejectsIdentityAndWorkerCountContradictions) {
  const TubeWorkerCompletionV2 completion = Completion(
      TubeWorkerCompletionStatusV2::BUILT);
  TubeV2DiagnosticsSnapshot snapshot = makeTubeV2DiagnosticsSnapshot(
      &completion, Stats(), Events(), false, Metrics());
  ASSERT_TRUE(validateTubeV2DiagnosticsSnapshot(snapshot));

  std::string reason;
  snapshot.completion.profile.request_id += 1U;
  EXPECT_FALSE(validateTubeV2DiagnosticsSnapshot(snapshot, &reason));
  EXPECT_EQ(reason, "built profile identity mismatch");

  snapshot = makeTubeV2DiagnosticsSnapshot(&completion, Stats(), Events(),
                                           false, Metrics());
  snapshot.stats.value.peak_in_flight = 2U;
  EXPECT_FALSE(validateTubeV2DiagnosticsSnapshot(snapshot, &reason));
  EXPECT_EQ(reason, "one-worker in-flight count exceeded");

  snapshot = makeTubeV2DiagnosticsSnapshot(&completion, Stats(), Events(),
                                           false, Metrics());
  snapshot.stats.value.build_finished = 4U;
  EXPECT_FALSE(validateTubeV2DiagnosticsSnapshot(snapshot, &reason));
  EXPECT_EQ(reason, "build-finished count exceeds build-started count");

  snapshot = makeTubeV2DiagnosticsSnapshot(&completion, Stats(), Events(),
                                           false, Metrics());
  snapshot.stats.value.build_succeeded = 2U;
  snapshot.stats.value.build_failed = 2U;
  EXPECT_FALSE(validateTubeV2DiagnosticsSnapshot(snapshot, &reason));
  EXPECT_EQ(reason, "build outcome counts exceed finished builds");

  snapshot = makeTubeV2DiagnosticsSnapshot(&completion, Stats(), Events(),
                                           false, Metrics());
  snapshot.completion.build_accepted_cell_count =
      snapshot.completion.build_cell_count + 1U;
  EXPECT_FALSE(validateTubeV2DiagnosticsSnapshot(snapshot, &reason));
  EXPECT_EQ(reason, "accepted cells exceed visited cells");
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
