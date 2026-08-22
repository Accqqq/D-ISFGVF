#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/StdVector>

#include <phase_offset_core/geometry.h>
#include <phase_offset_navigation/distance_query.h>
#include <phase_offset_navigation/tube_surface_validator.h>
#include <phase_offset_navigation/tube_types.h>

#include <bspline_race/continuous_phase_path.h>

namespace FLAG_Race {

// The disposition is evidence-only.  It is deliberately kept outside the
// legacy 60-field payload so existing ROS diagnostics schemas remain stable.
// A planner/tube mismatch means the actual Builder selected a robust
// pre-inset component which already excludes nominal delta=0; it is never an
// authorization to install or execute a nonzero retained offset.
enum class PhaseOffsetClearanceDisposition {
  UNKNOWN = 0,
  ZERO_CENTERLINE_CERTIFIED = 1,
  INSET_ZERO_EXCLUSION_REQUIRES_PROOF = 2,
  PLANNER_TUBE_ROBUST_CONTRACT_MISMATCH = 3,
  FILTER_ZERO_EXCLUSION = 4,
  VALIDATOR_ZERO_EXCLUSION = 5,
};

enum PhaseOffsetClearanceAuditIndex : std::size_t {
  kAuditSchemaVersion = 0U,
  kAuditValid,
  kAuditTubeSource,
  kAuditSourceRevision,
  kAuditTubeRevision,
  kAuditZeroGateOpen,
  kAuditFailureLatched,
  kAuditCurrentW,
  kAuditPreviewStartW,
  kAuditPreviewEndW,
  kAuditSampleStepW,
  kAuditLateralProbeHalfWidth,
  kAuditRequiredReferenceClearance,
  kAuditPlannerSafeDistance,
  kAuditConfiguredSearchMargin,
  kAuditSampleCount,
  kAuditGeometryInvalidCount,
  kAuditBaseKnownFreeCount,
  kAuditBaseUnavailableCount,
  kAuditBaseOutOfMapCount,
  kAuditBaseUnknownCount,
  kAuditBaseOccupiedCount,
  kAuditCountDltPlannerSafe,
  kAuditCountPlannerSafeLeDltSearchMargin,
  kAuditCountSearchMarginLeDltRequired,
  kAuditCountDgeRequired,
  kAuditMinBaseDistanceValid,
  kAuditMinBaseDistance,
  kAuditMinBaseW,
  kAuditMinBaseX,
  kAuditMinBaseY,
  kAuditMinBaseZ,
  kAuditMinBaseSegmentCode,
  kAuditFirstContractFailurePresent,
  kAuditFirstContractFailureReason,
  kAuditFirstContractFailureDistanceValid,
  kAuditFirstContractFailureDistance,
  kAuditFirstContractFailureDeficitValid,
  kAuditFirstContractFailureDeficit,
  kAuditFirstContractFailureW,
  kAuditFirstContractFailureX,
  kAuditFirstContractFailureY,
  kAuditFirstContractFailureZ,
  kAuditFirstContractFailureSegmentCode,
  kAuditPositiveProbeKnownFreeCount,
  kAuditPositiveProbeInvalidCount,
  kAuditPositiveProbeMinDistanceValid,
  kAuditPositiveProbeMinDistance,
  kAuditPositiveProbeMinW,
  kAuditNegativeProbeKnownFreeCount,
  kAuditNegativeProbeInvalidCount,
  kAuditNegativeProbeMinDistanceValid,
  kAuditNegativeProbeMinDistance,
  kAuditNegativeProbeMinW,
  kAuditCurrentSegmentCode,
  kAuditProfileComplete,
  kAuditObstacleCertified,
  kAuditTubeFirstInvalidW,
  kAuditTubeFirstStopReason,
  kAuditTubeInsufficientClearanceCount,
  kAuditCount,
};

static_assert(kAuditCount == 60U,
              "clearance contract audit schema must be exactly 60 fields");

struct PhaseOffsetClearanceAuditInput {
  const ContinuousPhasePath* path = nullptr;
  double current_w = 0.0;
  double back_w = 0.20;
  double lookahead_w = 2.0;
  double sample_step_w = 0.02;
  double lateral_probe_half_width = 0.10;
  double required_reference_clearance = 0.0;
  double planner_safe_distance = 0.4;
  double configured_search_margin = 0.5;
  std::uint64_t source_revision = 0U;
  std::uint64_t tube_revision = 0U;
  int tube_source = 0;
  bool zero_gate_open = false;
  bool failure_latched = false;
  bool profile_complete = false;
  bool obstacle_certified = false;
  double tube_first_invalid_w = 0.0;
  int tube_first_stop_reason = 0;
  double tube_insufficient_clearance_count = 0.0;
  // Immutable snapshot provenance.  These values are evidence labels only;
  // the audit never changes the clearance query or any navigation parameter.
  std::uint64_t snapshot_sequence = 0U;
  double snapshot_stamp = 0.0;
  double snapshot_resolution = 0.0;
  double snapshot_included_map_inflation = 0.0;
  double continuous_inset = -1.0;
  double validator_cover_radius = -1.0;
  bool snapshot_inflation_is_map_uncertainty = false;
  phase_offset_navigation::RobustTubeMargins margins;
  // Optional planner/query/profile evidence from the same path and snapshot.
  // `distance_query` remains the legacy centerline query and is used as the
  // planner query when planner_distance_query is absent.
  phase_offset_navigation::DistanceQuery planner_distance_query;
  phase_offset_navigation::ClearanceQuery raw_clearance_query;
  const phase_offset_navigation::TubeProfile* tube_profile = nullptr;
  const phase_offset_navigation::TubeSurfaceValidationResult*
      surface_validation = nullptr;
  phase_offset_navigation::DistanceQuery distance_query;
};

// One immutable path/snapshot accounting record.  The vector is deliberately
// separate from the legacy 60-field payload so existing diagnostics schemas do
// not move or change shape.
struct PhaseOffsetClearanceAuditPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w = 0.0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  std::uint64_t source_revision = 0U;
  std::uint64_t tube_revision = 0U;
  std::uint64_t snapshot_sequence = 0U;
  double snapshot_stamp = 0.0;
  int segment_code = 0;
  bool path_valid = false;
  bool profile_sample_found = false;
  bool path_profile_match = false;

  double planner_clearance = 0.0;
  bool planner_clearance_valid = false;
  double raw_clearance_lower_bound = 0.0;
  double raw_clearance_request_radius = 0.0;
  bool raw_clearance_valid = false;
  int raw_clearance_status = 0;

  double uav_radius = 0.0;
  double map_uncertainty = 0.0;
  double preincluded_map_uncertainty = 0.0;
  double localization_uncertainty = 0.0;
  double tracking_error_bound = 0.0;
  double full_effective_radius = 0.0;
  double residual_effective_radius = 0.0;
  double snapshot_included_map_inflation = 0.0;
  double snapshot_resolution = 0.0;
  double continuous_inset = 0.0;
  double validator_cover_radius = 0.0;
  double validator_requested_radius = 0.0;

  double pre_inset_lower = 0.0;
  double pre_inset_upper = 0.0;
  double raw_lower = 0.0;
  double raw_upper = 0.0;
  double filtered_lower = 0.0;
  double filtered_upper = 0.0;
  bool pre_inset_contains_zero = false;
  bool pre_inset_interval_valid = false;
  bool pre_inset_cross_section_valid = false;
  bool raw_contains_zero = false;
  bool filtered_contains_zero = false;
  bool validator_contains_zero = false;
  bool filter_sample_found = false;
  bool validator_sample_found = false;
  bool validator_complete = false;

  // Branch-B evidence: the same actual TubeProfile selected a pre-inset
  // robust component which excludes zero.  Keep this distinct from a raw
  // clearance deficit and from post-inset zero exclusion (Branch A).
  bool planner_tube_robust_contract_mismatch = false;

  bool residual_matches_full_minus_preincluded = false;
  bool snapshot_inflation_covers_preincluded = false;
  bool zero_excluded_by_inset = false;
  bool centerline_clearance_sufficient = false;
  bool unknown_or_out_of_map = false;
};

struct PhaseOffsetClearanceAuditResult {
  std::array<double, kAuditCount> values = {{0.0}};
  std::vector<PhaseOffsetClearanceAuditPoint,
              Eigen::aligned_allocator<PhaseOffsetClearanceAuditPoint>> points;
  bool margin_accounting_complete = false;
  bool same_path_snapshot = false;
  bool zero_contained_raw_all_points = false;
  bool zero_contained_filter_all_points = false;
  bool zero_contained_validator_all_points = false;
  bool map_inflation_alignment_evidence = false;
  bool duplicate_margin_proven = false;
  PhaseOffsetClearanceDisposition disposition =
      PhaseOffsetClearanceDisposition::UNKNOWN;
  bool planner_tube_robust_contract_mismatch_observed = false;
};

PhaseOffsetClearanceAuditResult RunPhaseOffsetClearanceAudit(
    const PhaseOffsetClearanceAuditInput& input);

}  // namespace FLAG_Race
