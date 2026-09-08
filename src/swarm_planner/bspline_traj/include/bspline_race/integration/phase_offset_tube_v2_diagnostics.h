#pragma once

#include <bspline_race/integration/phase_offset_tube_runtime_v2.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace FLAG_Race {

// Diagnostics are a value-only observation boundary.  They intentionally do
// not contain Runtime, allocator, publisher, READY, or authority handles.
enum class TubeV2CapabilityState {
  PLANNER_ONLY = 0,
  CERTIFIED_PROFILE,
  UNAVAILABLE,
};

const char* tubeV2CapabilityStateName(TubeV2CapabilityState state);

// Explicit Stage-2 metric attribution.  Values are absent unless the caller
// supplied the corresponding immutable map/config fact; diagnostics never
// infer inflation or a closed-volume proof from a boolean/allocated vector.
struct TubeV2MetricAttribution {
  bool closed_inflated_voxel_volume_metric = false;
  bool epsilon_present = false;
  double epsilon = std::numeric_limits<double>::quiet_NaN();
  bool epsilon_unchanged = false;
  // Keep anisotropic map inflation explicit.  A scalar summary is not enough
  // to establish physical containment when XY and Z backing differ.
  bool effective_map_inflation_axes_present = false;
  Eigen::Vector3d effective_map_inflation_axes =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  bool observed_obstacle_containment_proven = false;
  bool vehicle_tracking_budget_proven = false;
  bool physical_containment_proven = false;
  std::string map_inflation_provenance;
  bool support_complete = false;
  bool support_halo_present = false;
  double support_halo = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t support_expiry_ticks = 0U;
  bool support_expiry_timeless = false;
  std::uint64_t support_provenance_id = 0U;
  std::string frame_provenance;
  std::string unavailability_reason;
};

struct TubeV2BoundaryRecord {
  double w = std::numeric_limits<double>::quiet_NaN();
  double lower = std::numeric_limits<double>::quiet_NaN();
  double upper = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t left_cell_id = 0U;
  std::uint64_t right_cell_id = 0U;
  // Stage 2 has no live admission K.  Keep this explicit instead of copying
  // a later live value into the certified I/profile record.
  bool live_k_present = false;
  double live_k = std::numeric_limits<double>::quiet_NaN();
};

struct TubeV2ProfileRecord {
  bool present = false;
  std::uint64_t profile_id = 0U;
  std::uint64_t request_id = 0U;
  double requested_start = std::numeric_limits<double>::quiet_NaN();
  double requested_end = std::numeric_limits<double>::quiet_NaN();
  double anchor_w = std::numeric_limits<double>::quiet_NaN();
  double certified_start = std::numeric_limits<double>::quiet_NaN();
  double certified_end = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool complete = false;
  bool contains_anchor = false;
  bool contains_zero_everywhere = false;
  bool nonzero_capacity = false;
  phase_offset_navigation::TubeProfileV2Capability capability =
      phase_offset_navigation::TubeProfileV2Capability::UNAVAILABLE;
  phase_offset_navigation::TubeCertificateFailureReason failure_reason =
      phase_offset_navigation::TubeCertificateFailureReason::NONE;
  phase_offset_navigation::TubeCertificateTruncation truncation =
      phase_offset_navigation::TubeCertificateTruncation::NONE;
  std::string failure_detail;
  phase_offset_navigation::TubePathKey path_key;
  phase_offset_navigation::TubeConfigurationKey configuration_key;
  phase_offset_navigation::TubeMapCaptureKey map_capture_key;
  std::vector<TubeV2BoundaryRecord> boundaries;
};

struct TubeV2CompletionRecord {
  bool present = false;
  TubeWorkerCompletionStatusV2 status =
      TubeWorkerCompletionStatusV2::FAILED;
  TubeWorkerPurposeV2 purpose = TubeWorkerPurposeV2::CURRENT;
  std::uint64_t request_id = 0U;
  std::uint64_t execution_generation = 0U;
  std::uint64_t accepted_state_demand = 0U;
  double useful_start = std::numeric_limits<double>::quiet_NaN();
  double useful_end = std::numeric_limits<double>::quiet_NaN();
  phase_offset_navigation::TubePathKey path_key;
  phase_offset_navigation::TubeConfigurationKey configuration_key;
  phase_offset_navigation::TubeMapCaptureKey map_capture_key;
  std::uint64_t build_start_ticks = 0U;
  std::uint64_t build_end_ticks = 0U;
  std::uint64_t build_duration_ns = 0U;
  std::uint64_t path_callback_count = 0U;
  std::uint64_t path_callback_duration_ns = 0U;
  std::uint64_t producer_breakpoint_callback_count = 0U;
  std::uint64_t producer_breakpoint_callback_duration_ns = 0U;
  std::uint64_t free_ball_callback_count = 0U;
  std::uint64_t free_ball_callback_duration_ns = 0U;
  bool heavy_build_entered = false;
  bool cancellation_requested = false;
  bool callback_exception = false;
  bool builder_exception = false;
  bool build_success = false;
  std::uint64_t build_path_cell_query_count = 0U;
  std::uint64_t build_failed_path_cell_query_count = 0U;
  std::uint64_t build_query_count = 0U;
  std::uint64_t build_failed_query_count = 0U;
  std::uint64_t build_child_query_count = 0U;
  std::uint64_t build_cell_count = 0U;
  std::uint64_t build_scheduled_cell_count = 0U;
  std::uint64_t build_accepted_cell_count = 0U;
  std::uint64_t build_witness_count = 0U;
  int build_max_depth_observed = 0;
  bool build_query_budget_reached = false;
  bool build_cell_budget_reached = false;
  bool build_witness_budget_reached = false;
  bool build_sample_budget_reached = false;
  std::string error_message;
  TubeV2ProfileRecord profile;
};

struct TubeV2EventRecord {
  std::size_t ordinal = 0U;
  TubeWorkerEventTypeV2 type = TubeWorkerEventTypeV2::REJECTED;
  std::uint64_t timestamp_ticks = 0U;
  TubeWorkerPurposeV2 purpose = TubeWorkerPurposeV2::CURRENT;
  std::uint64_t request_id = 0U;
  std::uint64_t execution_generation = 0U;
  std::uint64_t accepted_state_demand = 0U;
  bool heavy_build_entered = false;
  std::uint64_t build_duration_ns = 0U;
};

// Stats remain a direct copy of the worker value.  No count is inferred from
// event rows or from a completion/profile, so one heavy proof cannot be
// accidentally reported twice.
struct TubeV2StatsRecord {
  TubeWorkerStatsV2 value;
};

struct TubeV2DiagnosticsSnapshot {
  bool planner_only = false;
  bool has_completion = false;
  TubeV2CapabilityState capability = TubeV2CapabilityState::UNAVAILABLE;
  TubeV2CompletionRecord completion;
  TubeV2StatsRecord stats;
  TubeV2MetricAttribution metrics;
  std::vector<TubeV2EventRecord> events;
};

// Build one immutable diagnostic snapshot from already-produced worker
// values.  The completion pointer may be null for a planner-only/unavailable
// observation.  This function never calls worker, map, path, or query code.
TubeV2DiagnosticsSnapshot makeTubeV2DiagnosticsSnapshot(
    const TubeWorkerCompletionV2* completion,
    const TubeWorkerStatsV2& stats,
    const std::vector<TubeWorkerEventV2>& events,
    bool planner_only,
    const TubeV2MetricAttribution& metrics = TubeV2MetricAttribution());

// Stable, versioned CSV schema.  The returned field names are unique and in
// the exact order used by tubeV2DiagnosticsCsvHeader/format.
const std::vector<std::string>& tubeV2DiagnosticsCsvFieldNames();
std::string tubeV2DiagnosticsCsvHeader();
std::string formatTubeV2DiagnosticsCsv(
    const TubeV2DiagnosticsSnapshot& snapshot);

// Structural checks are intentionally diagnostic-only.  A false result does
// not create a safety/authority decision; it reports malformed evidence for
// an offline recorder/replay caller.
bool validateTubeV2DiagnosticsSnapshot(
    const TubeV2DiagnosticsSnapshot& snapshot,
    std::string* reason = nullptr);

}  // namespace FLAG_Race
