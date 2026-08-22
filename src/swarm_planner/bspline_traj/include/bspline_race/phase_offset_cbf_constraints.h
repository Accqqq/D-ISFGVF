#ifndef BSPLINE_RACE_PHASE_OFFSET_CBF_CONSTRAINTS_H
#define BSPLINE_RACE_PHASE_OFFSET_CBF_CONSTRAINTS_H

#include <Eigen/Core>

#include <string>
#include <vector>

#include <bspline_race/phase_offset_allocator.h>
#include <bspline_race/phase_offset_types.h>

namespace FLAG_Race
{

// Analytic CBF constraint generators.  They only write linear constraints
// (a.dot(u) >= b); they never store UAV state, publish commands or modify the
// matched port.  Pairwise constraints read the already-predicted
// PredictedNeighborState (no ROS time / raw messages here); tube constraints
// read the TubeBounds fields (query_w, lower/upper, lower_dw/upper_dw).
class PhaseOffsetCbfConstraints
{
public:
  void appendTubeConstraints(const PhaseOffsetGeometry& geometry,
                             const TubeBounds& tube,
                             std::vector<LinearConstraint2D>& constraints)
    const;

  void appendPairwiseConstraints(
    const PhaseOffsetGeometry& geometry,
    const Eigen::Vector3d& self_position,
    const std::vector<PredictedNeighborState,
                      Eigen::aligned_allocator<PredictedNeighborState>>&
      safety_neighbors,
    std::vector<LinearConstraint2D>& constraints) const;

  double cbf_gamma_pair = 2.0;
  double cbf_gamma_tube = 3.0;
  double tube_margin = 0.05;
  double max_message_age = 0.30;

  double physical_safe_distance = 0.60;
  double self_position_error_bound = 0.08;
  double neighbor_position_error_bound = 0.08;
  double communication_position_margin = 0.05;
  double self_velocity_error_bound = 0.10;
  double neighbor_velocity_error_bound = 0.10;
  double execution_velocity_error_bound = 0.10;
  double self_acceleration_bound = 2.50;
  double neighbor_acceleration_bound = 2.50;
  double dt_qp_max = 0.03;
};

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_PHASE_OFFSET_CBF_CONSTRAINTS_H
