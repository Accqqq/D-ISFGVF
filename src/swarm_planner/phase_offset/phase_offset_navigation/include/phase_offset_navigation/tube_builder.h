#pragma once

#include "phase_offset_navigation/distance_query.h"
#include "phase_offset_navigation/path_state_query.h"
#include "phase_offset_navigation/tube_cross_section.h"
#include "phase_offset_navigation/tube_types.h"

#include <phase_offset_core/path_state.h>

#include <Eigen/StdVector>

#include <cstdint>
#include <vector>

namespace phase_offset_navigation {

class TubeBuilder {
 public:
  explicit TubeBuilder(const TubeBuilderConfig& config = TubeBuilderConfig());

  bool configurationValid() const;
  // Source-aware validation is the production gate.  ESDF validates only the
  // nominal bounded-cloud contract; FIXED additionally validates its legacy
  // fixed width.  Deprecated DistanceQuery max_offset is checked only by the
  // legacy entry point that actually consumes it.
  bool configurationValidForSource(TubeSource source) const;
  bool build(
      TubeSource source,
      const std::vector<phase_offset_core::PathDifferentialState,
                        Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
          preview,
      const DistanceQuery& distance_query,
      std::uint64_t source_revision,
      std::uint64_t tube_revision,
      TubeProfile& profile,
      double current_delta = 0.0) const;

  // Deprecated categorical compatibility entry point.  G2g does not allow
  // its result to establish a Candidate, Active, or clearance certificate.
  bool buildRawOccupancy(
      TubeSource source,
      const std::vector<phase_offset_core::PathDifferentialState,
                        Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
          preview,
      const RawOccupancyQuery& occupancy_query,
      double current_w,
      std::uint64_t source_revision,
      std::uint64_t tube_revision,
      TubeProfile& profile,
      double current_delta = 0.0) const;

  // G2g production path: direct occupied-voxel-volume clearance at every
  // cross-section centre, with adaptive exact path samples.  The categorical
  // RawOccupancyQuery overloads remain only compatibility/test diagnostics and
  // must not be used to certify a Candidate ribbon.
  bool buildCloudClearance(
      TubeSource source,
      const std::vector<phase_offset_core::PathDifferentialState,
                        Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
          preview,
      const ClearanceQuery& clearance_query,
      const PathStateQuery& path_state_query,
      double snapshot_resolution,
      double current_w,
      std::uint64_t source_revision,
      std::uint64_t tube_revision,
      TubeProfile& profile,
      double current_delta = 0.0) const;

  // Certified variant.  The cell query must share the immutable owner of the
  // point query.  An absent or failed certificate retains the legacy fixed
  // snapshot-resolution inset for this build; it never implies zero inset.
  bool buildCloudClearance(
      TubeSource source,
      const std::vector<phase_offset_core::PathDifferentialState,
                        Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
          preview,
      const ClearanceQuery& clearance_query,
      const PathStateQuery& path_state_query,
      const PathCellBoundQuery& path_cell_bound_query,
      double snapshot_resolution,
      double current_w,
      std::uint64_t source_revision,
      std::uint64_t tube_revision,
      TubeProfile& profile,
      double current_delta = 0.0) const;

  // Kept only for source compatibility with G2a pure-navigation callers.  New
  // runtime callers must pass the explicit current phase above.
  bool buildRawOccupancy(
      TubeSource source,
      const std::vector<phase_offset_core::PathDifferentialState,
                        Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>&
          preview,
      const RawOccupancyQuery& occupancy_query,
      std::uint64_t source_revision,
      std::uint64_t tube_revision,
      TubeProfile& profile,
      double current_delta = 0.0) const;

  const TubeBuilderConfig& config() const { return config_; }

 private:
  TubeBuilderConfig config_;
};

}  // namespace phase_offset_navigation
