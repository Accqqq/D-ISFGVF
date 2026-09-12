#ifndef PHASE_OFFSET_NAVIGATION_SECTION_TUBE_H
#define PHASE_OFFSET_NAVIGATION_SECTION_TUBE_H

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace phase_offset_navigation {

struct SectionBox {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
};

struct SectionEnvironment {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // available means that the supplied obstacle set and query domains are a
  // complete immutable local view.  An empty obstacles vector is therefore a
  // valid known-empty set only when available is true.
  bool available = false;
  SectionBox reference_domain;
  SectionBox obstacle_region;
  std::vector<SectionBox, Eigen::aligned_allocator<SectionBox>> obstacles;
};

struct SectionPathSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_ww = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  Eigen::Vector3d N_w = Eigen::Vector3d::Zero();
};

struct SectionCellBounds {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Bounds are caller-provided whole-cell derivative/speed contracts.  The
  // builder spot-checks a few samples for consistency, but those samples do
  // not establish or repair an underestimated bound.
  double horizontal_speed_lower = 0.0;
  double path_speed_lower = 0.0;
  Eigen::Vector3d abs_p_ww = Eigen::Vector3d::Zero();
  Eigen::Vector3d abs_N_ww = Eigen::Vector3d::Zero();
  bool valid = false;
};

struct SectionBuildConfig {
  double clearance = 0.4;
  double half_width = 1.0;
  double minimum_reference_speed = 1e-8;
  // M4F: guaranteed lower bound on the horizontal parallel-curve regularity
  // ratio |1 - kappa*delta|.  The cross-section never claims an offset that
  // would collapse the active reference r = p + N*delta.  Values outside
  // [0, 1) disable the cap.
  double min_regularity_ratio = 0.35;
  double curvature_epsilon = 1e-9;
  double max_step_w = 0.1;
  double min_step_w = 1e-4;
  int max_depth = 8;
  std::size_t max_cells = 2048U;
  std::size_t max_obstacle_checks = 200000U;
  std::size_t max_obstacles = 20000U;
};

struct SectionBuildInput {
  double w_start = 0.0;
  double w_end = 0.0;
  std::vector<double> structural_breakpoints;
  // Callbacks and environment are read-only for the duration of one build,
  // use the same coordinate frame, and describe one immutable path/domain
  // snapshot.  The builder does not synchronize or refresh them.
  std::function<bool(double, SectionPathSample&)> point_query;
  // Optional cell-aware point callback.  When present, the closed cell
  // endpoints are part of the query identity so adjacent cells may retain
  // distinct values at a structural seam.
  std::function<bool(double, double, double, SectionPathSample&)>
      cell_point_query;
  std::function<bool(double, double, SectionCellBounds&)> bounds_query;
  SectionEnvironment environment;
};

struct SectionTubeKnot {
  double w = 0.0;
  double lower = 0.0;
  double upper = 0.0;
};

enum class SectionTubeStatus {
  COMPLETE = 0,
  PARTIAL,
  ZERO_ONLY,
  INVALID_INPUT,
  UNKNOWN_DOMAIN,
  GEOMETRY_UNAVAILABLE,
  FRAME_DEGENERATE,
  SEED_BLOCKED,
  REGULARITY_LIMIT,
  BUDGET_EXCEEDED
};

struct SectionTubeProfile {
  double valid_start = 0.0;
  double valid_end = 0.0;
  SectionTubeStatus status = SectionTubeStatus::INVALID_INPUT;
  bool usable = false;
  bool complete = false;
  // usable means that this profile is a finite, queryable geometric prefix;
  // it does not imply control look-ahead, runtime installation, or any other
  // downstream feasibility condition.
  std::vector<SectionTubeKnot> knots;

  std::size_t visited_cells = 0U;
  std::size_t point_queries = 0U;
  std::size_t bounds_queries = 0U;
  std::size_t obstacle_checks = 0U;
  int max_depth_reached = 0;
  double first_failure_w = std::numeric_limits<double>::quiet_NaN();
  std::string failure_reason;

  bool evaluate(double w, double& lower, double& upper) const;
};

SectionTubeProfile buildSectionTube(
    const SectionBuildInput& input,
    const SectionBuildConfig& config = SectionBuildConfig());

// Numerical comparisons and inward repairs in the implementation assume the
// usual IEEE-754 finite, round-to-nearest behavior.  No formal guarantee is
// made for FTZ/DAZ, fast-math, or other non-IEEE execution modes.

}  // namespace phase_offset_navigation

#endif  // PHASE_OFFSET_NAVIGATION_SECTION_TUBE_H
