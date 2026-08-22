#pragma once

#include "phase_offset_navigation/path_state_query.h"
#include "phase_offset_navigation/tube_surface_validator.h"
#include "phase_offset_navigation/tube_types.h"

#include <phase_offset_core/path_state.h>
#include <phase_offset_core/port_types.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace phase_offset_navigation {

enum class TubeEpochState {
  NO_ACTIVE_TUBE,
  WAITING_FOR_CANDIDATE,
  ROLLING,
  CERTIFICATE_DENIED,
  CONFIGURATION_ERROR,
};

enum class TubeInstallDisposition {
  NONE,
  INITIAL_INSTALL,
  REPLACED_ACTIVE,
  EQUIVALENT_REFRESH,
  SAFETY_REPLACEMENT,
  REJECTED_CANDIDATE,
};

enum class CurrentSafetyStatus {
  NOT_EVALUATED,
  SAFE,
  INDETERMINATE,
  UNSAFE,
};

enum class TubeEpochReason {
  NONE,
  INVALID_CONFIGURATION,
  SOURCE_NONE,
  CANDIDATE_INCOMPLETE,
  CURRENT_GEOMETRY_INVALID,
  CURRENT_BOUNDS_INVALID,
  CURRENT_OFFSET_OUTSIDE,
  REFERENCE_INDETERMINATE,
  ACTUAL_INDETERMINATE,
  REFERENCE_CLEARANCE_INSUFFICIENT,
  ACTUAL_CLEARANCE_INSUFFICIENT,
  BASE_CENTERLINE_CLEARANCE_INSUFFICIENT,
  BASE_CENTERLINE_INDETERMINATE,
  BASE_CENTERLINE_CONTINUITY_UNCERTIFIED,
  FORWARD_HORIZON_SHORT,
};

// A permanent adapter failure is reserved for an internal contract
// contradiction, never for a transient tube/map/tracking classification.
enum class ControlFailureReason {
  NONE,
  ZERO_PORT_EQUIVALENCE,
  GEOMETRY_INVARIANT,
  BASE_GUIDANCE_INVARIANT,
  PORT_PROJECTOR_CONTRADICTION,
  MATCHED_PORT_INVARIANT,
};

const char* controlFailureReasonName(ControlFailureReason reason);

const char* tubeEpochReasonName(TubeEpochReason reason);

using TubeEpochPathSamples =
    std::vector<phase_offset_core::PathDifferentialState,
                Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>;

struct TubeEpochManagerConfig {
  TubeBuilderConfig builder;
  TubeFilterConfig filter;
  TubeSurfaceValidatorConfig surface_validator;
  double profile_equivalence_tolerance = 1e-10;
  double inside_tolerance = 1e-10;
};

struct TubeEpochUpdateInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TubeSource source = TubeSource::NONE;
  phase_offset_core::PathDifferentialState current_path;
  TubeEpochPathSamples preview_path;
  TubeBounds authority_request;
  Eigen::Vector3d actual_position = Eigen::Vector3d::Zero();
  // G2g immutable PointCloud2 / voxel-volume certificate.  ESDF production
  // uses this cloud-clearance query exclusively and fails closed without it.
  ClearanceQuery cloud_clearance_query;
  PathStateQuery path_state_query;
  PathCellBoundQuery path_cell_bound_query;
  double cloud_snapshot_resolution = 0.0;
  double retained_delta = 0.0;
  std::uint64_t path_source_revision = 0U;
  // Identity of the immutable observation used for this one build.  It is
  // deliberately not used as profile identity by itself.
  std::uint64_t map_observation_sequence = 0U;
  bool map_observation_is_snapshot = false;
};

struct TubeEpochStatus {
  TubeEpochState state = TubeEpochState::NO_ACTIVE_TUBE;
  TubeInstallDisposition disposition = TubeInstallDisposition::NONE;
  CurrentSafetyStatus current_safety_status = CurrentSafetyStatus::NOT_EVALUATED;
  TubeEpochReason reason = TubeEpochReason::NONE;
  std::string reason_text;

  std::uint64_t candidate_sequence = 0U;
  std::uint64_t active_tube_epoch = 0U;
  std::uint64_t candidate_path_source_revision = 0U;
  std::uint64_t active_path_source_revision = 0U;
  std::uint64_t candidate_map_observation_sequence = 0U;
  std::uint64_t active_map_observation_sequence = 0U;

  bool candidate_raw_complete = false;
  bool candidate_filtered_complete = false;
  bool candidate_complete = false;
  TubeProfileClassification candidate_classification =
      TubeProfileClassification::NONE;
  TubeProfileClassification active_classification =
      TubeProfileClassification::NONE;
  // Compatibility name: true means the asymmetric categorical environment
  // cross-section path was used.  It no longer implies SDFMap raw log odds.
  bool raw_cross_section_path_used = false;
  bool map_observation_is_snapshot = false;
  bool active_available = false;
  bool active_current_validation_valid = false;
  bool current_geometry_valid = false;
  bool current_bounds_valid = false;
  bool current_interval_nonempty = false;
  bool current_interval_contains_zero = false;
  bool current_interval_contains_retained_delta = false;
  bool base_centerline_clearance_sufficient = false;
  bool retained_delta_current_inside = false;
  bool current_state_admissible = false;
  bool reference_clearance_sufficient = false;
  bool actual_clearance_sufficient = false;
  bool tracking_within_bound = false;
  bool forward_horizon_sufficient = false;
  // This denies the offset certificate only.  It does not issue an emergency
  // command, replan request, or safety-control mode.
  bool certificate_denied = false;
  bool transient_blocked = false;
  bool genuine_fatal_invariant = false;
  ControlFailureReason control_failure_reason = ControlFailureReason::NONE;

  double retained_delta = 0.0;
  double tracking_error_norm = 0.0;
  double tracking_error_bound = 0.0;
  double reference_signed_distance = 0.0;
  double actual_signed_distance = 0.0;
  double required_reference_clearance = 0.0;
  double required_actual_clearance = 0.0;
  double full_effective_radius = 0.0;
  double preincluded_map_uncertainty = 0.0;
  double residual_effective_radius = 0.0;
  double certified_forward_w = 0.0;

  std::uint64_t equivalent_refresh_count = 0U;
  std::uint64_t install_count = 0U;
  std::uint64_t reject_count = 0U;
  std::uint64_t wait_count = 0U;
};

struct TubeEpochUpdateResult {
  TubeProfile candidate_profile;
  TubeProfile active_profile;
  TubeEpochStatus status;
};

}  // namespace phase_offset_navigation
