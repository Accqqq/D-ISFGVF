#include "bspline_race/integration/phase_offset_tube_v2_diagnostics.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>

namespace FLAG_Race {
namespace {

constexpr const char* kSchemaVersion = "tube_v2_diagnostics_v1";

bool finite(const double value) { return std::isfinite(value); }

const char* profileCapabilityName(
    const phase_offset_navigation::TubeProfileV2Capability capability) {
  switch (capability) {
    case phase_offset_navigation::TubeProfileV2Capability::UNAVAILABLE:
      return "UNAVAILABLE";
    case phase_offset_navigation::TubeProfileV2Capability::ZERO_ONLY:
      return "ZERO_ONLY";
    case phase_offset_navigation::TubeProfileV2Capability::OFFSET_CERTIFIED:
      return "OFFSET_CERTIFIED";
  }
  return "UNKNOWN";
}

std::string u64(const std::uint64_t value) {
  return std::to_string(value);
}

std::string i64(const std::int64_t value) {
  return std::to_string(value);
}

std::string sizeValue(const std::size_t value) {
  return std::to_string(static_cast<unsigned long long>(value));
}

std::string boolean(const bool value) { return value ? "true" : "false"; }

std::string number(const double value) {
  if (!finite(value)) return "absent";
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::max_digits10)
         << value;
  return stream.str();
}

std::string csvEscape(const std::string& value) {
  if (value.find_first_of(",\"\n\r") == std::string::npos) return value;
  std::string escaped = "\"";
  for (const char character : value) {
    if (character == '\"') escaped += "\"\"";
    else escaped += character;
  }
  escaped += "\"";
  return escaped;
}

using Row = std::map<std::string, std::string>;

void setPath(Row& row, const phase_offset_navigation::TubePathKey& key) {
  row["path_execution_generation"] = u64(key.execution_generation);
  row["path_instance_id"] = u64(key.path_instance_id);
  row["path_revision"] = u64(key.path_revision);
  row["path_frame_revision"] = u64(key.frame_revision);
  row["path_frame_convention_id"] = u64(key.frame_convention_id);
  row["path_frame_convention"] = key.frame_convention;
  row["path_phase_orientation"] = std::to_string(key.phase_orientation);
  row["path_domain_start"] = number(key.domain_start);
  row["path_domain_end"] = number(key.domain_end);
}

void setConfiguration(
    Row& row, const phase_offset_navigation::TubeConfigurationKey& key) {
  row["configuration_id"] = u64(key.configuration_id);
  row["configuration_epsilon"] = number(key.epsilon);
  row["configuration_nominal_half_width"] = number(key.nominal_half_width);
  row["configuration_ray_step"] = number(key.ray_step);
  row["configuration_snapshot_resolution"] = number(key.snapshot_resolution);
  row["configuration_minimum_reference_speed"] =
      number(key.minimum_reference_speed);
}

void setMap(Row& row, const phase_offset_navigation::TubeMapCaptureKey& key) {
  row["map_instance_id"] = u64(key.map_instance_id);
  row["map_state_id"] = u64(key.state_id);
  row["map_accepted_sequence"] = u64(key.accepted_sequence);
  row["map_configuration_generation"] = u64(key.configuration_generation);
  row["map_configuration_id"] = u64(key.configuration_id);
  row["map_frame_provenance_id"] = u64(key.frame_provenance_id);
  row["map_frame_provenance"] = key.frame_provenance;
  row["map_support_provenance_id"] = u64(key.support_provenance_id);
  row["map_accepted_time_ticks"] = u64(key.accepted_time_ticks);
  row["map_support_expiry_ticks"] = u64(key.support_expiry_ticks);
  row["map_support_expiry_timeless"] = boolean(key.support_expiry_timeless);
  row["map_support_halo"] = number(key.support_halo);
  row["map_halo_reconciled"] = boolean(key.halo_reconciled);
  row["map_grid_min_x"] = i64(key.grid_min_index_x);
  row["map_grid_min_y"] = i64(key.grid_min_index_y);
  row["map_grid_min_z"] = i64(key.grid_min_index_z);
  row["map_grid_max_x"] = i64(key.grid_max_index_x);
  row["map_grid_max_y"] = i64(key.grid_max_index_y);
  row["map_grid_max_z"] = i64(key.grid_max_index_z);
  row["map_grid_origin_x"] = number(key.grid_native_origin.x());
  row["map_grid_origin_y"] = number(key.grid_native_origin.y());
  row["map_grid_origin_z"] = number(key.grid_native_origin.z());
  row["map_grid_resolution_x"] = number(key.grid_voxel_resolution.x());
  row["map_grid_resolution_y"] = number(key.grid_voxel_resolution.y());
  row["map_grid_resolution_z"] = number(key.grid_voxel_resolution.z());
  row["map_source_offset_x"] = i64(key.grid_source_offset_x);
  row["map_source_offset_y"] = i64(key.grid_source_offset_y);
  row["map_source_offset_z"] = i64(key.grid_source_offset_z);
  row["map_native_index"] = boolean(key.grid_native_index);
  row["map_complete_support"] = boolean(key.complete_support);
}

void setCompletion(Row& row, const TubeV2CompletionRecord& completion) {
  row["completion_present"] = boolean(completion.present);
  row["completion_status"] = tubeWorkerCompletionStatusName(completion.status);
  row["completion_purpose"] = tubeWorkerPurposeName(completion.purpose);
  row["completion_request_id"] = u64(completion.request_id);
  row["completion_execution_generation"] = u64(completion.execution_generation);
  row["completion_accepted_state_demand"] =
      u64(completion.accepted_state_demand);
  row["completion_useful_start"] = number(completion.useful_start);
  row["completion_useful_end"] = number(completion.useful_end);
  setPath(row, completion.path_key);
  setConfiguration(row, completion.configuration_key);
  setMap(row, completion.map_capture_key);
  row["build_start_ticks"] = u64(completion.build_start_ticks);
  row["build_end_ticks"] = u64(completion.build_end_ticks);
  row["build_duration_ns"] = u64(completion.build_duration_ns);
  row["path_callback_count"] = u64(completion.path_callback_count);
  row["path_callback_duration_ns"] = u64(completion.path_callback_duration_ns);
  row["producer_breakpoint_callback_count"] =
      u64(completion.producer_breakpoint_callback_count);
  row["producer_breakpoint_callback_duration_ns"] =
      u64(completion.producer_breakpoint_callback_duration_ns);
  row["free_ball_callback_count"] = u64(completion.free_ball_callback_count);
  row["free_ball_callback_duration_ns"] =
      u64(completion.free_ball_callback_duration_ns);
  row["heavy_build_entered"] = boolean(completion.heavy_build_entered);
  row["cancellation_requested"] = boolean(completion.cancellation_requested);
  row["callback_exception"] = boolean(completion.callback_exception);
  row["builder_exception"] = boolean(completion.builder_exception);
  row["build_success"] = boolean(completion.build_success);
  row["build_path_cell_query_count"] =
      u64(completion.build_path_cell_query_count);
  row["build_failed_path_cell_query_count"] =
      u64(completion.build_failed_path_cell_query_count);
  row["build_query_count"] = u64(completion.build_query_count);
  row["build_failed_query_count"] = u64(completion.build_failed_query_count);
  row["build_child_query_count"] = u64(completion.build_child_query_count);
  row["build_cell_count"] = u64(completion.build_cell_count);
  row["build_scheduled_cell_count"] =
      u64(completion.build_scheduled_cell_count);
  row["build_accepted_cell_count"] =
      u64(completion.build_accepted_cell_count);
  row["build_witness_count"] = u64(completion.build_witness_count);
  row["build_max_depth_observed"] =
      std::to_string(completion.build_max_depth_observed);
  row["build_query_budget_reached"] =
      boolean(completion.build_query_budget_reached);
  row["build_cell_budget_reached"] =
      boolean(completion.build_cell_budget_reached);
  row["build_witness_budget_reached"] =
      boolean(completion.build_witness_budget_reached);
  row["build_sample_budget_reached"] =
      boolean(completion.build_sample_budget_reached);
  row["error_message"] = completion.error_message;
}

void setProfile(Row& row, const TubeV2ProfileRecord& profile) {
  row["profile_present"] = boolean(profile.present);
  row["profile_id"] = u64(profile.profile_id);
  row["profile_request_id"] = u64(profile.request_id);
  row["profile_requested_start"] = number(profile.requested_start);
  row["profile_requested_end"] = number(profile.requested_end);
  row["profile_anchor_w"] = number(profile.anchor_w);
  row["profile_certified_start"] = number(profile.certified_start);
  row["profile_certified_end"] = number(profile.certified_end);
  row["profile_valid"] = boolean(profile.valid);
  row["profile_complete"] = boolean(profile.complete);
  row["profile_contains_anchor"] = boolean(profile.contains_anchor);
  row["profile_contains_zero_everywhere"] =
      boolean(profile.contains_zero_everywhere);
  row["profile_nonzero_capacity"] = boolean(profile.nonzero_capacity);
  row["profile_capability"] = profileCapabilityName(profile.capability);
  row["profile_failure_reason"] =
      phase_offset_navigation::tubeCertificateFailureReasonName(
          profile.failure_reason);
  row["profile_truncation"] =
      phase_offset_navigation::tubeCertificateTruncationName(profile.truncation);
  row["profile_failure_detail"] = profile.failure_detail;
}

void setMetrics(Row& row, const TubeV2MetricAttribution& metrics) {
  row["metric_closed_inflated_voxel_volume"] =
      boolean(metrics.closed_inflated_voxel_volume_metric);
  row["metric_epsilon_present"] = boolean(metrics.epsilon_present);
  row["metric_epsilon"] = number(metrics.epsilon);
  row["metric_epsilon_unchanged"] = boolean(metrics.epsilon_unchanged);
  row["metric_inflation_axes_present"] =
      boolean(metrics.effective_map_inflation_axes_present);
  row["metric_inflation_x"] = number(metrics.effective_map_inflation_axes.x());
  row["metric_inflation_y"] = number(metrics.effective_map_inflation_axes.y());
  row["metric_inflation_z"] = number(metrics.effective_map_inflation_axes.z());
  row["metric_observed_obstacle_containment_proven"] =
      boolean(metrics.observed_obstacle_containment_proven);
  row["metric_vehicle_tracking_budget_proven"] =
      boolean(metrics.vehicle_tracking_budget_proven);
  row["metric_physical_containment_proven"] =
      boolean(metrics.physical_containment_proven);
  row["metric_inflation_provenance"] = metrics.map_inflation_provenance;
  row["metric_support_complete"] = boolean(metrics.support_complete);
  row["metric_support_halo_present"] =
      boolean(metrics.support_halo_present);
  row["metric_support_halo"] = number(metrics.support_halo);
  row["metric_support_expiry_ticks"] = u64(metrics.support_expiry_ticks);
  row["metric_support_expiry_timeless"] =
      boolean(metrics.support_expiry_timeless);
  row["metric_support_provenance_id"] = u64(metrics.support_provenance_id);
  row["metric_frame_provenance"] = metrics.frame_provenance;
  row["metric_unavailability_reason"] = metrics.unavailability_reason;
}

void setStats(Row& row, const TubeWorkerStatsV2& value) {
  row["stats_submitted"] = u64(value.submitted);
  row["stats_accepted"] = u64(value.accepted);
  row["stats_rejected"] = u64(value.rejected);
  row["stats_coalesced"] = u64(value.coalesced);
  row["stats_less_useful_rejected"] = u64(value.less_useful_rejected);
  row["stats_build_started"] = u64(value.build_started);
  row["stats_build_finished"] = u64(value.build_finished);
  row["stats_build_succeeded"] = u64(value.build_succeeded);
  row["stats_build_failed"] = u64(value.build_failed);
  row["stats_path_callback_count"] = u64(value.path_callback_count);
  row["stats_path_callback_duration_ns"] = u64(value.path_callback_duration_ns);
  row["stats_breakpoint_callback_count"] =
      u64(value.producer_breakpoint_callback_count);
  row["stats_breakpoint_callback_duration_ns"] =
      u64(value.producer_breakpoint_callback_duration_ns);
  row["stats_free_ball_callback_count"] = u64(value.free_ball_callback_count);
  row["stats_free_ball_callback_duration_ns"] =
      u64(value.free_ball_callback_duration_ns);
  row["stats_cancellation_requested"] = u64(value.cancellation_requested);
  row["stats_cancellation_discarded"] = u64(value.cancellation_discarded);
  row["stats_stale_discarded"] = u64(value.stale_discarded);
  row["stats_delivery_count"] = u64(value.delivery_count);
  row["stats_delivery_discarded"] = u64(value.delivery_discarded);
  row["stats_event_log_dropped"] = u64(value.event_log_dropped);
  row["stats_in_flight"] = u64(value.in_flight);
  row["stats_peak_in_flight"] = u64(value.peak_in_flight);
  row["stats_pending_current"] = u64(value.pending_current);
  row["stats_pending_successor"] = u64(value.pending_successor);
  row["stats_retained_current"] = u64(value.retained_current);
  row["stats_retained_successor"] = u64(value.retained_successor);
}

const std::vector<std::string>& fields() {
  static const std::vector<std::string> value = {
      "schema_version", "record_kind", "ordinal", "capability_state",
      "planner_only", "has_completion", "completion_present",
      "completion_status", "completion_purpose", "completion_request_id",
      "completion_execution_generation", "completion_accepted_state_demand",
      "completion_useful_start", "completion_useful_end",
      "path_execution_generation", "path_instance_id", "path_revision",
      "path_frame_revision", "path_frame_convention_id",
      "path_frame_convention", "path_phase_orientation", "path_domain_start",
      "path_domain_end", "configuration_id", "configuration_epsilon",
      "configuration_nominal_half_width", "configuration_ray_step",
      "configuration_snapshot_resolution",
      "configuration_minimum_reference_speed", "map_instance_id",
      "map_state_id", "map_accepted_sequence", "map_configuration_generation",
      "map_configuration_id", "map_frame_provenance_id",
      "map_frame_provenance", "map_support_provenance_id",
      "map_accepted_time_ticks", "map_support_expiry_ticks",
      "map_support_expiry_timeless", "map_support_halo",
      "map_halo_reconciled", "map_grid_min_x", "map_grid_min_y",
      "map_grid_min_z", "map_grid_max_x", "map_grid_max_y", "map_grid_max_z",
      "map_grid_origin_x", "map_grid_origin_y", "map_grid_origin_z",
      "map_grid_resolution_x", "map_grid_resolution_y",
      "map_grid_resolution_z", "map_source_offset_x", "map_source_offset_y",
      "map_source_offset_z", "map_native_index", "map_complete_support",
      "build_start_ticks", "build_end_ticks", "build_duration_ns",
      "path_callback_count", "path_callback_duration_ns",
      "producer_breakpoint_callback_count",
      "producer_breakpoint_callback_duration_ns", "free_ball_callback_count",
      "free_ball_callback_duration_ns", "heavy_build_entered",
      "cancellation_requested", "callback_exception", "builder_exception",
      "build_success", "build_path_cell_query_count",
      "build_failed_path_cell_query_count", "build_query_count",
      "build_failed_query_count", "build_child_query_count",
      "build_cell_count", "build_scheduled_cell_count",
      "build_accepted_cell_count", "build_witness_count",
      "build_max_depth_observed", "build_query_budget_reached",
      "build_cell_budget_reached", "build_witness_budget_reached",
      "build_sample_budget_reached", "error_message", "profile_present",
      "profile_id",
      "profile_request_id",
      "profile_requested_start", "profile_requested_end", "profile_anchor_w",
      "profile_certified_start", "profile_certified_end", "profile_valid",
      "profile_complete", "profile_contains_anchor",
      "profile_contains_zero_everywhere", "profile_nonzero_capacity",
      "profile_capability", "profile_failure_reason", "profile_truncation",
      "profile_failure_detail", "metric_closed_inflated_voxel_volume",
      "metric_epsilon_present", "metric_epsilon", "metric_epsilon_unchanged",
      "metric_inflation_axes_present", "metric_inflation_x",
      "metric_inflation_y", "metric_inflation_z",
      "metric_observed_obstacle_containment_proven",
      "metric_vehicle_tracking_budget_proven",
      "metric_physical_containment_proven", "metric_inflation_provenance",
      "metric_support_complete", "metric_support_halo_present",
      "metric_support_halo", "metric_support_expiry_ticks",
      "metric_support_expiry_timeless", "metric_support_provenance_id",
      "metric_frame_provenance", "metric_unavailability_reason",
      "stats_submitted", "stats_accepted", "stats_rejected", "stats_coalesced",
      "stats_less_useful_rejected", "stats_build_started",
      "stats_build_finished", "stats_build_succeeded", "stats_build_failed",
      "stats_path_callback_count", "stats_path_callback_duration_ns",
      "stats_breakpoint_callback_count",
      "stats_breakpoint_callback_duration_ns", "stats_free_ball_callback_count",
      "stats_free_ball_callback_duration_ns", "stats_cancellation_requested",
      "stats_cancellation_discarded", "stats_stale_discarded",
      "stats_delivery_count", "stats_delivery_discarded",
      "stats_event_log_dropped", "stats_in_flight", "stats_peak_in_flight",
      "stats_pending_current", "stats_pending_successor",
      "stats_retained_current", "stats_retained_successor", "event_type",
      "event_timestamp_ticks", "event_purpose", "event_request_id",
      "event_execution_generation", "event_accepted_state_demand",
      "event_heavy_build_entered", "event_build_duration_ns", "boundary_index",
      "boundary_w", "boundary_lower", "boundary_upper",
      "boundary_left_cell_id", "boundary_right_cell_id",
      "boundary_live_k_present", "boundary_live_k"};
  return value;
}

std::string renderRow(const Row& row) {
  std::ostringstream stream;
  const std::vector<std::string>& names = fields();
  for (std::size_t index = 0U; index < names.size(); ++index) {
    if (index != 0U) stream << ',';
    const auto found = row.find(names[index]);
    stream << csvEscape(found == row.end() ? "not_applicable" : found->second);
  }
  stream << '\n';
  return stream.str();
}

TubeV2ProfileRecord copyProfile(
    const phase_offset_navigation::TubeProfileV2& profile) {
  TubeV2ProfileRecord result;
  result.present = true;
  result.profile_id = profile.profile_id;
  result.request_id = profile.request_id;
  result.requested_start = profile.requested_start;
  result.requested_end = profile.requested_end;
  result.anchor_w = profile.anchor_w;
  result.certified_start = profile.certified_start;
  result.certified_end = profile.certified_end;
  result.valid = profile.valid;
  result.complete = profile.complete;
  result.contains_anchor = profile.contains_anchor;
  result.contains_zero_everywhere = profile.contains_zero_everywhere;
  result.nonzero_capacity = profile.nonzero_capacity;
  result.capability = profile.capability;
  result.failure_reason = profile.failure_reason;
  result.truncation = profile.truncation;
  result.failure_detail = profile.failure.detail;
  result.path_key = profile.path_key;
  result.configuration_key = profile.configuration_key;
  result.map_capture_key = profile.map_capture_key;
  result.boundaries.reserve(profile.knots.size());
  for (const phase_offset_navigation::TubePwlKnotV2& knot : profile.knots) {
    TubeV2BoundaryRecord boundary;
    boundary.w = knot.w;
    boundary.lower = knot.lower;
    boundary.upper = knot.upper;
    boundary.left_cell_id = knot.left_cell_id;
    boundary.right_cell_id = knot.right_cell_id;
    result.boundaries.push_back(boundary);
  }
  return result;
}

TubeV2CompletionRecord copyCompletion(
    const TubeWorkerCompletionV2& completion) {
  TubeV2CompletionRecord result;
  result.present = true;
  result.status = completion.status;
  result.purpose = completion.purpose;
  result.request_id = completion.request_id;
  result.execution_generation = completion.execution_generation;
  result.accepted_state_demand = completion.accepted_state_demand;
  result.useful_start = completion.useful_start;
  result.useful_end = completion.useful_end;
  result.path_key = completion.path_key;
  result.configuration_key = completion.configuration_key;
  result.map_capture_key = completion.map_capture_key;
  result.build_start_ticks = completion.build_start_ticks;
  result.build_end_ticks = completion.build_end_ticks;
  result.build_duration_ns = completion.build_duration_ns;
  result.path_callback_count = completion.path_callback_count;
  result.path_callback_duration_ns = completion.path_callback_duration_ns;
  result.producer_breakpoint_callback_count =
      completion.producer_breakpoint_callback_count;
  result.producer_breakpoint_callback_duration_ns =
      completion.producer_breakpoint_callback_duration_ns;
  result.free_ball_callback_count = completion.free_ball_callback_count;
  result.free_ball_callback_duration_ns = completion.free_ball_callback_duration_ns;
  result.heavy_build_entered = completion.heavy_build_entered;
  result.cancellation_requested = completion.cancellation_requested;
  result.callback_exception = completion.callback_exception;
  result.builder_exception = completion.builder_exception;
  result.build_success = completion.build.success;
  result.build_path_cell_query_count =
      completion.build.stats.path_cell_query_count;
  result.build_failed_path_cell_query_count =
      completion.build.stats.failed_path_cell_query_count;
  result.build_query_count = completion.build.stats.query_count;
  result.build_failed_query_count = completion.build.stats.failed_query_count;
  result.build_child_query_count = completion.build.stats.child_query_count;
  result.build_cell_count = completion.build.stats.cell_count;
  result.build_scheduled_cell_count = completion.build.stats.scheduled_cell_count;
  result.build_accepted_cell_count = completion.build.stats.accepted_cell_count;
  result.build_witness_count = completion.build.stats.witness_count;
  result.build_max_depth_observed = completion.build.stats.max_depth_observed;
  result.build_query_budget_reached = completion.build.stats.query_budget_reached;
  result.build_cell_budget_reached = completion.build.stats.cell_budget_reached;
  result.build_witness_budget_reached =
      completion.build.stats.witness_budget_reached;
  result.build_sample_budget_reached = completion.build.stats.sample_budget_reached;
  result.error_message = completion.error_message;
  result.profile = copyProfile(completion.build.profile);
  return result;
}

bool profileIdentityMatchesCompletion(
    const TubeV2CompletionRecord& completion) {
  return completion.profile.present &&
      completion.profile.request_id == completion.request_id &&
      completion.profile.path_key == completion.path_key &&
      completion.profile.configuration_key == completion.configuration_key &&
      completion.profile.map_capture_key == completion.map_capture_key;
}

void addEvent(Row& row, const TubeV2EventRecord& event) {
  row["event_type"] = tubeWorkerEventTypeName(event.type);
  row["event_timestamp_ticks"] = u64(event.timestamp_ticks);
  row["event_purpose"] = tubeWorkerPurposeName(event.purpose);
  row["event_request_id"] = u64(event.request_id);
  row["event_execution_generation"] = u64(event.execution_generation);
  row["event_accepted_state_demand"] = u64(event.accepted_state_demand);
  row["event_heavy_build_entered"] = boolean(event.heavy_build_entered);
  row["event_build_duration_ns"] = u64(event.build_duration_ns);
}

}  // namespace

const char* tubeV2CapabilityStateName(const TubeV2CapabilityState state) {
  switch (state) {
    case TubeV2CapabilityState::PLANNER_ONLY: return "PLANNER_ONLY";
    case TubeV2CapabilityState::CERTIFIED_PROFILE: return "CERTIFIED_PROFILE";
    case TubeV2CapabilityState::UNAVAILABLE: return "UNAVAILABLE";
  }
  return "UNAVAILABLE";
}

TubeV2DiagnosticsSnapshot makeTubeV2DiagnosticsSnapshot(
    const TubeWorkerCompletionV2* completion,
    const TubeWorkerStatsV2& stats,
    const std::vector<TubeWorkerEventV2>& events,
    const bool planner_only,
    const TubeV2MetricAttribution& metrics) {
  TubeV2DiagnosticsSnapshot result;
  result.planner_only = planner_only;
  result.has_completion = completion != nullptr;
  result.stats.value = stats;
  result.metrics = metrics;
  if (completion != nullptr) {
    result.completion = copyCompletion(*completion);
    result.has_completion = result.completion.present;
    const TubeV2ProfileRecord& profile = result.completion.profile;
    if (result.completion.status == TubeWorkerCompletionStatusV2::BUILT &&
        result.completion.build_success && profile.valid &&
        profile.complete && profileIdentityMatchesCompletion(result.completion)) {
      result.capability = TubeV2CapabilityState::CERTIFIED_PROFILE;
    } else {
      result.capability = TubeV2CapabilityState::UNAVAILABLE;
    }
    // The configuration epsilon is copied from the exact completion key; an
    // unchanged-budget claim still requires explicit caller attribution.
    if (!result.metrics.epsilon_present &&
        finite(result.completion.configuration_key.epsilon)) {
      result.metrics.epsilon = result.completion.configuration_key.epsilon;
      result.metrics.epsilon_present = true;
    }
    if (!result.metrics.support_halo_present &&
        finite(result.completion.map_capture_key.support_halo)) {
      result.metrics.support_halo =
          result.completion.map_capture_key.support_halo;
      result.metrics.support_halo_present = true;
    }
    result.metrics.support_complete =
        result.completion.map_capture_key.complete_support;
    result.metrics.support_expiry_ticks =
        result.completion.map_capture_key.support_expiry_ticks;
    result.metrics.support_expiry_timeless =
        result.completion.map_capture_key.support_expiry_timeless;
    result.metrics.support_provenance_id =
        result.completion.map_capture_key.support_provenance_id;
    result.metrics.frame_provenance =
        result.completion.map_capture_key.frame_provenance;
    if (result.capability == TubeV2CapabilityState::UNAVAILABLE &&
        result.metrics.unavailability_reason.empty()) {
      if (result.completion.status == TubeWorkerCompletionStatusV2::BUILT &&
          result.completion.build_success && profile.valid &&
          profile.complete && !profileIdentityMatchesCompletion(result.completion)) {
        result.metrics.unavailability_reason = "profile_identity_mismatch";
      } else {
        result.metrics.unavailability_reason =
            completion->error_message.empty()
            ? tubeWorkerCompletionStatusName(completion->status)
            : completion->error_message;
      }
    }
  } else {
    result.capability = planner_only ? TubeV2CapabilityState::PLANNER_ONLY
                                     : TubeV2CapabilityState::UNAVAILABLE;
    if (result.metrics.unavailability_reason.empty()) {
      result.metrics.unavailability_reason = planner_only
          ? "no_completion_planner_only" : "no_completion";
    }
  }
  result.events.reserve(events.size());
  for (std::size_t index = 0U; index < events.size(); ++index) {
    TubeV2EventRecord event;
    event.ordinal = index;
    event.type = events[index].type;
    event.timestamp_ticks = events[index].timestamp_ticks;
    event.purpose = events[index].purpose;
    event.request_id = events[index].request_id;
    event.execution_generation = events[index].execution_generation;
    event.accepted_state_demand = events[index].accepted_state_demand;
    event.heavy_build_entered = events[index].heavy_build_entered;
    event.build_duration_ns = events[index].build_duration_ns;
    result.events.push_back(event);
  }
  return result;
}

const std::vector<std::string>& tubeV2DiagnosticsCsvFieldNames() {
  return fields();
}

std::string tubeV2DiagnosticsCsvHeader() {
  std::ostringstream stream;
  const std::vector<std::string>& names = fields();
  for (std::size_t index = 0U; index < names.size(); ++index) {
    if (index != 0U) stream << ',';
    stream << names[index];
  }
  stream << '\n';
  return stream.str();
}

std::string formatTubeV2DiagnosticsCsv(
    const TubeV2DiagnosticsSnapshot& snapshot) {
  std::ostringstream stream;
  stream << tubeV2DiagnosticsCsvHeader();
  Row base;
  base["schema_version"] = kSchemaVersion;
  base["capability_state"] = tubeV2CapabilityStateName(snapshot.capability);
  base["planner_only"] = boolean(snapshot.planner_only);
  base["has_completion"] = boolean(snapshot.has_completion);
  setCompletion(base, snapshot.completion);
  setProfile(base, snapshot.completion.profile);
  setMetrics(base, snapshot.metrics);
  setStats(base, snapshot.stats.value);
  base["record_kind"] = "SNAPSHOT";
  base["ordinal"] = "0";
  stream << renderRow(base);

  for (const TubeV2EventRecord& event : snapshot.events) {
    Row row;
    row["schema_version"] = kSchemaVersion;
    row["record_kind"] = "EVENT";
    row["ordinal"] = sizeValue(event.ordinal);
    row["capability_state"] = tubeV2CapabilityStateName(snapshot.capability);
    row["planner_only"] = boolean(snapshot.planner_only);
    addEvent(row, event);
    stream << renderRow(row);
  }
  for (std::size_t index = 0U;
       index < snapshot.completion.profile.boundaries.size(); ++index) {
    const TubeV2BoundaryRecord& boundary =
        snapshot.completion.profile.boundaries[index];
    Row row;
    row["schema_version"] = kSchemaVersion;
    row["record_kind"] = "BOUNDARY";
    row["ordinal"] = sizeValue(index);
    row["capability_state"] = tubeV2CapabilityStateName(snapshot.capability);
    row["planner_only"] = boolean(snapshot.planner_only);
    row["boundary_index"] = sizeValue(index);
    row["boundary_w"] = number(boundary.w);
    row["boundary_lower"] = number(boundary.lower);
    row["boundary_upper"] = number(boundary.upper);
    row["boundary_left_cell_id"] = u64(boundary.left_cell_id);
    row["boundary_right_cell_id"] = u64(boundary.right_cell_id);
    row["boundary_live_k_present"] = boolean(boundary.live_k_present);
    row["boundary_live_k"] = number(boundary.live_k);
    stream << renderRow(row);
  }
  return stream.str();
}

bool validateTubeV2DiagnosticsSnapshot(
    const TubeV2DiagnosticsSnapshot& snapshot, std::string* reason) {
  if (reason != nullptr) reason->clear();
  const auto fail = [reason](const char* message) {
    if (reason != nullptr) *reason = message;
    return false;
  };
  if (snapshot.capability == TubeV2CapabilityState::CERTIFIED_PROFILE &&
      (!snapshot.has_completion || !snapshot.completion.present ||
       snapshot.completion.status != TubeWorkerCompletionStatusV2::BUILT ||
       !snapshot.completion.build_success ||
       !snapshot.completion.profile.present ||
       !snapshot.completion.profile.valid ||
       !snapshot.completion.profile.complete)) {
    return fail("certified capability lacks a built complete profile");
  }
  if (snapshot.has_completion && snapshot.completion.present &&
      snapshot.completion.status != TubeWorkerCompletionStatusV2::BUILT &&
      snapshot.capability == TubeV2CapabilityState::CERTIFIED_PROFILE) {
    return fail("failed/cancelled/stale completion is certified");
  }
  if (snapshot.completion.present &&
      snapshot.completion.status == TubeWorkerCompletionStatusV2::BUILT &&
      snapshot.completion.build_success && snapshot.completion.profile.present &&
      snapshot.completion.profile.valid && snapshot.completion.profile.complete &&
      !profileIdentityMatchesCompletion(snapshot.completion)) {
    return fail("built profile identity mismatch");
  }
  if (snapshot.completion.present &&
      snapshot.completion.execution_generation !=
          snapshot.completion.path_key.execution_generation) {
    return fail("completion/path generation mismatch");
  }
  if (snapshot.completion.present &&
      snapshot.completion.accepted_state_demand !=
          snapshot.completion.map_capture_key.accepted_sequence) {
    return fail("completion/map accepted-state mismatch");
  }
  const TubeWorkerStatsV2& stats = snapshot.stats.value;
  if (stats.in_flight > 1U || stats.peak_in_flight > 1U) {
    return fail("one-worker in-flight count exceeded");
  }
  if (stats.build_finished > stats.build_started) {
    return fail("build-finished count exceeds build-started count");
  }
  if (stats.build_succeeded > stats.build_finished ||
      stats.build_failed > stats.build_finished - stats.build_succeeded) {
    return fail("build outcome counts exceed finished builds");
  }
  if (snapshot.completion.present) {
    const TubeV2CompletionRecord& completion = snapshot.completion;
    if (completion.build_failed_path_cell_query_count >
        completion.build_path_cell_query_count) {
      return fail("failed path queries exceed path queries");
    }
    if (completion.build_failed_query_count > completion.build_query_count) {
      return fail("failed queries exceed queries");
    }
    if (completion.build_cell_count > completion.build_scheduled_cell_count) {
      return fail("visited cells exceed scheduled cells");
    }
    if (completion.build_accepted_cell_count > completion.build_cell_count) {
      return fail("accepted cells exceed visited cells");
    }
    if (completion.build_max_depth_observed < 0) {
      return fail("negative maximum build depth");
    }
  }
  if (snapshot.metrics.epsilon_present &&
      !finite(snapshot.metrics.epsilon)) {
    return fail("epsilon marked present without a finite value");
  }
  if (snapshot.metrics.epsilon_unchanged &&
      !snapshot.metrics.epsilon_present) {
    return fail("unchanged epsilon lacks an explicit value");
  }
  if (snapshot.metrics.effective_map_inflation_axes_present &&
      (!finite(snapshot.metrics.effective_map_inflation_axes.x()) ||
       !finite(snapshot.metrics.effective_map_inflation_axes.y()) ||
       !finite(snapshot.metrics.effective_map_inflation_axes.z()))) {
    return fail("inflation axes marked present without finite values");
  }
  if (snapshot.metrics.effective_map_inflation_axes_present &&
      snapshot.metrics.map_inflation_provenance.empty()) {
    return fail("inflation axes lack provenance");
  }
  if (snapshot.metrics.support_halo_present &&
      !finite(snapshot.metrics.support_halo)) {
    return fail("support halo marked present without a finite value");
  }
  std::uint64_t previous_event_ticks = 0U;
  bool have_event_ticks = false;
  std::uint64_t delivered_event_count = 0U;
  for (std::size_t index = 0U; index < snapshot.events.size(); ++index) {
    const TubeV2EventRecord& event = snapshot.events[index];
    if (event.ordinal != index) return fail("event order/ordinal mismatch");
    if (have_event_ticks && event.timestamp_ticks < previous_event_ticks) {
      return fail("event timing is not monotonic");
    }
    previous_event_ticks = event.timestamp_ticks;
    have_event_ticks = true;
    if (event.type == TubeWorkerEventTypeV2::COMPLETION_DELIVERED) {
      ++delivered_event_count;
    }
    if (snapshot.completion.present &&
        (event.type == TubeWorkerEventTypeV2::BUILD_STARTED ||
         event.type == TubeWorkerEventTypeV2::BUILD_FINISHED ||
         event.type == TubeWorkerEventTypeV2::COMPLETION_DELIVERED) &&
        event.purpose == snapshot.completion.purpose &&
        (event.request_id != snapshot.completion.request_id ||
         event.execution_generation != snapshot.completion.execution_generation ||
         event.accepted_state_demand !=
             snapshot.completion.accepted_state_demand)) {
      return fail("completion/event identity mismatch");
    }
  }
  if (delivered_event_count > stats.delivery_count) {
    return fail("event delivery count exceeds worker stats");
  }
  if (stats.event_log_dropped == 0U &&
      delivered_event_count != stats.delivery_count) {
    return fail("delivery count not conserved");
  }
  for (const TubeV2BoundaryRecord& boundary :
       snapshot.completion.profile.boundaries) {
    if (boundary.live_k_present || !finite(boundary.w) ||
        !finite(boundary.lower) || !finite(boundary.upper) ||
        boundary.lower > boundary.upper) {
      return fail("invalid certified boundary or fabricated live K");
    }
  }
  if (snapshot.completion.present && snapshot.completion.build_start_ticks != 0U &&
      snapshot.completion.build_end_ticks != 0U &&
      snapshot.completion.build_end_ticks < snapshot.completion.build_start_ticks) {
    return fail("nonmonotonic build timing");
  }
  if (snapshot.metrics.physical_containment_proven &&
      (!snapshot.metrics.closed_inflated_voxel_volume_metric ||
       !snapshot.metrics.effective_map_inflation_axes_present ||
       !snapshot.metrics.support_complete ||
       !snapshot.metrics.observed_obstacle_containment_proven ||
       !snapshot.metrics.vehicle_tracking_budget_proven)) {
    return fail("physical containment claimed without closed metric/axes/support");
  }
  return true;
}

}  // namespace FLAG_Race
