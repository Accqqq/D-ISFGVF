#pragma once

#include "phase_offset_navigation/distance_query.h"

#include <Eigen/Core>

#include <functional>
#include <cstdint>
#include <cstdint>

namespace phase_offset_navigation {

using RawOccupancyQuery = std::function<DistanceStatus(const Eigen::Vector3d&)>;

struct RobustTubeMargins {
  double uav_radius = 0.0;
  double map_uncertainty = 0.0;
  double localization_uncertainty = 0.0;
  double tracking_error_bound = 0.0;
  // Part of map_uncertainty already represented by the categorical
  // occupancy snapshot's voxel inflation.  It is subtracted once, and only
  // once, from the cross-section erosion.
  double preincluded_map_uncertainty = 0.0;

  double fullEffectiveRadius() const;
  double residualEffectiveRadius() const;
  // Deprecated diagnostic compatibility only.  Production geometry uses
  // TubeCrossSectionConfig::planner_safe_distance instead.
  double effectiveRadius() const;
};

struct TubeCrossSectionConfig {
  double search_extent = 3.0;
  double ray_step = 0.05;
  double boundary_tolerance = 1e-3;
  double regularity_margin = 0.1;
  double curvature_epsilon = 1e-9;
  // The planner-authoritative clearance contract.  This is the sole
  // production clearance requested from the immutable Tube snapshot.
  double planner_safe_distance = 0.4;
  double minimum_reference_speed = 1e-8;
  // Deprecated diagnostic accounting retained only while downstream schemas
  // are migrated.  These values never determine cross-section geometry.
  RobustTubeMargins margins;
};

enum class TubeCrossSectionReason {
  NONE,
  INVALID_CONFIGURATION,
  INVALID_GEOMETRY,
  CENTER_UNAVAILABLE,
  CENTER_OUT_OF_MAP,
  CENTER_UNKNOWN,
  CENTER_OCCUPIED,
  EMPTY_AFTER_OBSTACLE_BOUNDS,
  CURVATURE_NUMERICAL_FAILURE,
  EMPTY_AFTER_CURVATURE_INTERSECTION,
  REGULARITY_ZERO_UNSAFE,
};

enum class TubeRayTermination {
  SEARCH_EXTENT,
  OCCUPIED,
  UNKNOWN,
  OUT_OF_MAP,
  UNAVAILABLE,
};

struct TubeCrossSectionInput {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  double curvature = 0.0;
  Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d N_w = Eigen::Vector3d::Zero();
  double minimum_reference_speed = 0.0;
  double current_delta = 0.0;
  bool current_delta_valid = false;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  // The safety-producing input.  It directly certifies clearance of the
  // candidate centre p + N * delta against occupied voxel volumes.
  ClearanceQuery clearance_query;
  // Compatibility-only categorical facts for old diagnostics.  It must never
  // be used to establish a cross-section safety interval.
  RawOccupancyQuery occupancy_query;
};

struct TubeCrossSectionResult {
  double c_plus_raw = 0.0;
  double c_minus_raw = 0.0;
  double full_effective_radius = 0.0;
  double preincluded_map_uncertainty = 0.0;
  double residual_effective_radius = 0.0;
  // Legacy field retained for schema users; it equals residual_effective_radius.
  double effective_radius = 0.0;
  double lower_obstacle = 0.0;
  double upper_obstacle = 0.0;
  double lower_curvature = 0.0;
  double upper_curvature = 0.0;
  double lower_final = 0.0;
  double upper_final = 0.0;
  double width = 0.0;
  bool contains_zero = false;
  bool regularity_proven = false;
  double regularity_speed_at_zero = 0.0;
  double regularity_speed_min = 0.0;
  // A valid result always describes the planner-authoritative zero-connected
  // interval.  When the immutable snapshot cannot establish non-zero space,
  // that interval is the valid zero-only fallback [0, 0].
  bool valid = false;
  TubeRayTermination positive_termination = TubeRayTermination::UNAVAILABLE;
  TubeRayTermination negative_termination = TubeRayTermination::UNAVAILABLE;
  TubeCrossSectionReason reason = TubeCrossSectionReason::NONE;
};

class TubeCrossSectionSolver {
 public:
  explicit TubeCrossSectionSolver(
      const TubeCrossSectionConfig& config = TubeCrossSectionConfig());

  bool configurationValid() const;
  TubeCrossSectionResult solve(const TubeCrossSectionInput& input) const;

  const TubeCrossSectionConfig& config() const { return config_; }

 private:
  TubeCrossSectionConfig config_;
};

}  // namespace phase_offset_navigation
