#ifndef BSPLINE_RACE_PHASE_OFFSET_ALLOCATOR_H
#define BSPLINE_RACE_PHASE_OFFSET_ALLOCATOR_H

#include <Eigen/Core>

#include <string>
#include <vector>

#include <bspline_race/phase_offset_types.h>

namespace FLAG_Race
{

struct LinearConstraint2D
{
  Eigen::Vector2d a = Eigen::Vector2d::Zero();
  double b = 0.0;  // a.dot(u) >= b
  std::string label;
};

struct AllocatorParams
{
  double K1 = 2.0;
  double phase_speed_min = 0.10;
  double phase_speed_max = 2.00;
  double tangent_speed_min = 0.10;
  double tangent_speed_max = 2.00;
  double u_w_slew_rate = 1.50;
  double u_delta_slew_rate = 1.50;
  double qp_dt_max = 0.03;
  double weight_world_error = 1.00;
  double weight_u_w = 0.10;
  double weight_u_delta = 0.10;
  double weight_delta_u_w = 0.05;
  double weight_delta_u_delta = 0.05;
};

// Two-variable convex QP with an active-set-style enumerator:
// candidates = unconstrained optimum, projections onto every constraint
// boundary, and intersections of every constraint pair; pick the feasible
// candidate with the lowest cost.
class PhaseOffsetAllocator
{
public:
  PortCommand solve(const PhaseOffsetGeometry& geometry,
                    const TubeBounds& tube,
                    const SwarmIntent& intent,
                    const std::vector<PredictedNeighborState,
                        Eigen::aligned_allocator<PredictedNeighborState>>&
                        safety_neighbors,
                    double dt,
                    const PortCommand& previous,
                    SwarmControlMode requested_mode =
                        SwarmControlMode::ROLLING,
                    const std::vector<LinearConstraint2D>& extra_constraints =
                        std::vector<LinearConstraint2D>());

  static Eigen::Vector2d unconstrainedSolution(
      const PhaseOffsetGeometry& geometry, const Eigen::Vector3d& g_des,
      const AllocatorParams& params, const PortCommand& previous);

  static Eigen::Vector2d analyticRawPort(
      const PhaseOffsetGeometry& geometry, const Eigen::Vector3d& g_des);

  AllocatorParams params;
};

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_PHASE_OFFSET_ALLOCATOR_H
