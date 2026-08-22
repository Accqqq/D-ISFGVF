#pragma once

#include "phase_offset_navigation/tube_builder.h"
#include "phase_offset_navigation/tube_filter.h"
#include "phase_offset_navigation/tube_surface_validator.h"

#include <phase_offset_core/path_state.h>

#include <Eigen/StdVector>

#include <cstdint>
#include <vector>

namespace phase_offset_navigation {

// Deliberately independent from tube_epoch_types.h: this builder owns only
// immutable candidate construction, not epoch or installation policy.
using CertifiedTubePathSamples =
    std::vector<phase_offset_core::PathDifferentialState,
                Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>;

struct CertifiedTubeBuildInput {
  TubeSource source = TubeSource::NONE;
  CertifiedTubePathSamples preview_path;
  // Immutable transaction input.  ESDF certification proves only the
  // post-filter intersection with this requested controller-authority set;
  // raw/environment geometry remains untouched.
  TubeBounds authority_request;
  DistanceQuery distance_query;
  ClearanceQuery cloud_clearance_query;
  PathStateQuery path_state_query;
  PathCellBoundQuery path_cell_bound_query;
  double cloud_snapshot_resolution = 0.0;
  double current_w = 0.0;
  std::uint64_t path_source_revision = 0U;
  std::uint64_t tube_revision = 0U;
  std::uint64_t map_observation_sequence = 0U;
  bool map_observation_is_snapshot = false;
};

struct CertifiedTubeBuildResult {
  TubeProfile profile;
  bool raw_complete = false;
  bool filtered_complete = false;
  bool complete = false;
  bool surface_validation_attempted = false;
  TubeSurfaceValidationResult surface_validation;
};

// The only production candidate construction entry point.  It combines the
// retained Builder, Filter, and SurfaceValidator without taking ownership of
// current-state, retained-delta, installation, or epoch policy.
class CertifiedTubeBuilder {
 public:
  CertifiedTubeBuilder(const TubeBuilderConfig& builder_config = TubeBuilderConfig(),
                       const TubeFilterConfig& filter_config = TubeFilterConfig(),
                       const TubeSurfaceValidatorConfig& validator_config =
                           TubeSurfaceValidatorConfig());

  bool configurationValid() const;
  bool build(const CertifiedTubeBuildInput& input,
             CertifiedTubeBuildResult& result) const;

 private:
  TubeBuilder builder_;
  TubeFilter filter_;
  TubeSurfaceValidator surface_validator_;
  TubeBuilderConfig builder_config_;
  TubeSurfaceValidatorConfig validator_config_;
};

}  // namespace phase_offset_navigation
