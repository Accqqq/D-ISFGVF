#ifndef BSPLINE_RACE_PHASE_OFFSET_GEOMETRY_H
#define BSPLINE_RACE_PHASE_OFFSET_GEOMETRY_H

#include <Eigen/Core>

#include <functional>
#include <string>
#include <vector>

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/phase_offset_types.h>

namespace FLAG_Race
{

struct PhaseOffsetGeometryParams
{
  // ISF terms (same convention as the legacy lifted guidance: K2 < 0).
  double K1 = 2.0;
  double K2 = -2.2;
  double alpha_min = 0.2;
  double rho0 = 1.0;
  double delta_band = 1.0;
  double convergence_bandwidth = 0.10;

  // Regularity / level-flight guards.
  double mu_regular = 0.20;   // 1 - kappa*delta >= mu
  double v_xy_min = 0.05;     // minimum horizontal |p_w|
  double z_tolerance = 0.10;  // maximum |T_z| allowed
  double max_delta = 1.20;    // |delta| clamp for degenerate queries
};

// Pure geometry / ISF evaluation on the ACTIVE reference r = p + N*delta.
// No ROS topics, no control history: easy to unit test.
class PhaseOffsetGeometryEvaluator
{
public:
  bool evaluate(const ContinuousPhasePathState& path_state,
                const Eigen::Vector3d& position,
                double delta,
                const PhaseOffsetGeometryParams& params,
                PhaseOffsetGeometry& out) const;
};

// Batch 9: C2 connector sampling check (plan section 7.9).
struct C2ConnectorCheckResult
{
  double min_regularity = 1e9;
  double min_obstacle_distance = 1e9;
  double min_tube_margin = 1e9;
  bool ok = true;
  std::string reason;
};

// Evaluate a set of path samples inside the C2 connector region with the
// CURRENT delta kept:
//   1. regularity 1 - kappa*delta >= mu_regular;
//   2. active reference r = p + N*delta keeps obstacle distance
//      >= required_clearance (when obstacle_distance callback is provided).
// Fills the minimum regularity / obstacle distance / tube margin seen.
bool checkC2ConnectorSamples(
  const std::vector<ContinuousPhasePathState>& samples,
  double delta,
  double mu_regular,
  const std::function<double(const Eigen::Vector3d&)>& obstacle_distance,
  double required_clearance,
  C2ConnectorCheckResult& out);

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_PHASE_OFFSET_GEOMETRY_H
