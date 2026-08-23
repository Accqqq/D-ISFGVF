#pragma once

#include "phase_offset_navigation/tube_cross_section.h"

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace phase_offset_navigation {

enum class TubeSource {
  NONE,
  FIXED,
  ESDF,
};

// The planner owns the delta=0 centreline.  A TubeProfile therefore either
// carries an independently proved nonzero offset capacity, or is the valid
// zero-width baseline that only represents the planner path.  Do not express
// this with a collection of overlapping boolean gates: the classification is
// the single authority boundary for offset activation.
enum class TubeProfileClassification {
  NONE,
  ZERO_ONLY_PLANNER_BASELINE,
  OFFSET_CERTIFIED,
};

enum class TubeProofLevel {
  NONE,
  SAMPLED_EVIDENCE,
  FRAME_CELL_PROOF,
  CONTINUOUS_COVER_PROOF,
};

enum class TubeComponentSelection {
  NONE,
  ZERO_CONNECTED,
  CURRENT_DELTA_CONNECTED,
};

enum class TubeStopReason {
  NONE,
  INVALID_PATH,
  UNAVAILABLE,
  OUT_OF_MAP,
  UNKNOWN,
  OCCUPIED,
  INSUFFICIENT_CLEARANCE,
  REGULARITY,
};

struct ErosionMargins {
  double uav_radius = 0.25;
  double localization_margin = 0.05;
  double tracking_error_bound = 0.15;
  double map_margin = 0.10;
  double extra_margin = 0.05;
  double discretization_margin = 0.10;

  double requiredReferenceClearance() const {
    return uav_radius + localization_margin + tracking_error_bound +
        map_margin + extra_margin + discretization_margin;
  }

  double requiredActualClearance() const {
    return uav_radius + localization_margin + map_margin + extra_margin +
        discretization_margin;
  }
};

struct TubeBuilderConfig {
  double fixed_delta_max = 0.06;
  double max_offset = 0.20;
  double sample_step_w = 0.10;
  double lookahead_w = 2.0;
  double back_w = 0.2;
  double min_certified_forward_w = 0.4;
  double ray_step = 0.05;
  double regularity_margin = 0.1;
  double invariant_gain = 1.0;
  double interior_margin = 0.0;
  ErosionMargins erosion;

  // This is the raw-occupancy contract.  The older max_offset/ray_step/erosion
  // fields remain solely for the temporary legacy DistanceQuery path.
  TubeCrossSectionConfig cross_section;
};

struct TubeFilterConfig {
  double boundary_slope_max = 0.8;
  int dense_samples_per_segment = 16;
};

struct TubeRawSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w = 0.0;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  double raw_lower = 0.0;
  double raw_upper = 0.0;
  double filtered_lower = 0.0;
  double filtered_upper = 0.0;
  double lower_w = 0.0;
  double upper_w = 0.0;
  double fixed_lower = 0.0;
  double fixed_upper = 0.0;
  double regularity_lower = 0.0;
  double regularity_upper = 0.0;
  double esdf_lower = 0.0;
  double esdf_upper = 0.0;
  double base_signed_distance = 0.0;
  bool complete = false;
  bool positive_certified = false;
  bool negative_certified = false;
  bool regularity_intersection = false;
  TubeStopReason positive_stop = TubeStopReason::NONE;
  TubeStopReason negative_stop = TubeStopReason::NONE;

  // Raw-occupancy cross-section facts.  These are intentionally distinct from
  // the legacy fixed/ESDF fields above: raw_lower/raw_upper are the final
  // environmental bounds on this path, not controller offset limits.
  double c_plus_raw = 0.0;
  double c_minus_raw = 0.0;
  double full_effective_radius = 0.0;
  double preincluded_map_uncertainty = 0.0;
  double residual_effective_radius = 0.0;
  // Retained for pre-G2d readers.  It is the actual residual erosion applied
  // to this selected (possibly pre-inflated) environment backing.
  double effective_radius = 0.0;
  double obstacle_lower = 0.0;
  double obstacle_upper = 0.0;
  // Bounds selected by the direct-clearance cross-section before the cloud
  // resolution inset.  They intentionally remain separate from raw_lower /
  // raw_upper, which are the post-inset Filter input on the cloud path.
  double pre_inset_lower = 0.0;
  double pre_inset_upper = 0.0;
  double continuous_inset = 0.0;
  double curvature_lower = 0.0;
  double curvature_upper = 0.0;
  double environment_lower = 0.0;
  double environment_upper = 0.0;
  double environment_width = 0.0;
  bool environment_interval_nonempty = false;
  bool environment_contains_zero = false;
  bool pre_inset_contains_zero = false;
  bool post_inset_contains_zero = false;
  // A cell-local geometric residual may replace the fixed cloud-resolution
  // inset only when the complete immutable-path certificate covers every
  // active Builder cell.  This remains generic in delta (including zero).
  bool cell_geometry_certificate_used = false;
  double cell_geometry_inset = 0.0;
  bool filter_input_contains_zero = false;
  bool filtered_contains_zero = false;
  double regularity_speed_min = 0.0;
  double regularity_speed_at_delta = 0.0;
  TubeProofLevel proof_level = TubeProofLevel::NONE;
  TubeCrossSectionReason cross_section_reason = TubeCrossSectionReason::NONE;
  TubeRayTermination positive_ray_termination = TubeRayTermination::UNAVAILABLE;
  TubeRayTermination negative_ray_termination = TubeRayTermination::UNAVAILABLE;
};

// In-memory evidence emitted by the continuous ribbon validator.  It is
// attached to the same TubeProfile produced by the Builder; it is not a ROS
// diagnostics schema and does not change profile geometry or Runtime policy.
struct TubeValidatorKnotEvidence {
  double w = 0.0;
  double max_cover_radius = 0.0;
  double max_requested_clearance = 0.0;
  bool observed = false;
  bool filtered_contains_zero = false;
  // True only when a successful validator cell covers this raw/filter knot
  // and its filtered interval includes the nominal centreline.
  bool zero_surface_covered = false;
};

struct TubeBuildDiagnostics {
  std::size_t sample_count = 0U;
  std::size_t invalid_count = 0U;
  std::size_t unavailable_count = 0U;
  std::size_t out_of_map_count = 0U;
  std::size_t unknown_count = 0U;
  std::size_t occupied_count = 0U;
  std::size_t insufficient_clearance_count = 0U;
  double min_width = 0.0;
  double min_safety_margin = 0.0;
  double first_invalid_w = 0.0;
  int first_invalid_side = 0;
  int first_stop_reason = static_cast<int>(TubeStopReason::NONE);
  std::string invalid_reason;
};

struct TubeProfile {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<TubeRawSample, Eigen::aligned_allocator<TubeRawSample>> samples;
  // Complete, unfiltered output of the one actual Builder invocation.  The
  // Filter deliberately trims `samples`; retaining these facts lets audits
  // identify whether zero disappeared before Filter, in Filter, or later in
  // continuous validation without rebuilding or re-querying a different map.
  std::vector<TubeRawSample, Eigen::aligned_allocator<TubeRawSample>>
      raw_build_samples;
  std::vector<TubeValidatorKnotEvidence> validator_knot_evidence;
  TubeSource source = TubeSource::NONE;
  // preview_start_w/preview_end_w are always the retained, current-containing
  // certified segment.  Keep the original request separately so a raw map
  // observation that ends ahead of the vehicle does not erase a valid local
  // tube or hide the fact that its preview was truncated.
  double preview_start_w = 0.0;
  double preview_end_w = 0.0;
  double requested_preview_start_w = 0.0;
  double requested_preview_end_w = 0.0;
  double certified_segment_start_w = 0.0;
  double certified_segment_end_w = 0.0;
  bool certified_segment_truncated_before = false;
  bool certified_segment_truncated_after = false;
  double first_truncated_w = 0.0;
  TubeStopReason first_truncated_reason = TubeStopReason::NONE;
  std::uint64_t source_revision = 0U;
  std::uint64_t tube_revision = 0U;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t map_revision = 0U;
  std::uint64_t profile_revision = 0U;
  std::string obstacle_contract_id;
  TubeProofLevel proof_level = TubeProofLevel::NONE;
  TubeComponentSelection selected_component = TubeComponentSelection::NONE;
  double current_delta = 0.0;
  bool current_delta_valid = false;
  bool current_component_contains_delta = false;
  bool zero_component_contains_zero = false;
  bool zero_only = false;
  std::uint64_t snapshot_sequence = 0U;
  double snapshot_resolution = 0.0;
  bool snapshot_provenance_is_immutable = false;
  bool cell_geometry_certified = false;
  // True only after Builder has combined each cell's frame bounds with the
  // actual admissible delta interval and verified the configured m_r.
  bool combined_regularity_proof_complete = false;
  // Conservative minimum of inf||p_w|| - sup||N_w||*max|delta| over the
  // certified cells used by this profile.
  double combined_regularity_speed_min = 0.0;
  std::size_t certified_cell_count = 0U;
  bool raw_complete = false;
  bool filtered_complete = false;
  bool complete = false;
  bool obstacle_certified = false;
  TubeProfileClassification classification =
      TubeProfileClassification::NONE;
  // This is evidence for a centreline already contained in the filtered
  // ribbon.  It never manufactures a disconnected or degenerate {delta=0}
  // tube when the post-inset interval excluded zero.
  bool zero_centerline_continuously_certified = false;
  TubeBuildDiagnostics diagnostics;
};

struct TubeBounds {
  double query_w = 0.0;
  double lower = 0.0;
  double upper = 0.0;
  double lower_w = 0.0;
  double upper_w = 0.0;
  bool valid = false;
};

struct TubeRuntimeStatus {
  TubeSource source = TubeSource::NONE;
  bool readiness_evaluated = false;
  bool source_ready = false;
  bool raw_complete = false;
  bool filtered_complete = false;
  bool profile_complete = false;
  bool obstacle_certified = false;
  bool current_inside = false;
  bool next_inside = false;
  bool tracking_within_bound = false;
  bool tube_violation = false;
  bool failure_latched = false;
  int stable_rebuild_count = 0;
  int rebuild_count = 0;
  int reject_count = 0;
  int violation_count = 0;
  double certified_forward_w = 0.0;
  double reference_signed_distance = 0.0;
  double actual_signed_distance = 0.0;
  double tracking_error_norm = 0.0;
};

const char* tubeSourceName(TubeSource source);
const char* tubeStopReasonName(TubeStopReason reason);
const char* tubeProofLevelName(TubeProofLevel level);
const char* tubeComponentSelectionName(TubeComponentSelection selection);

}  // namespace phase_offset_navigation
