#include "bspline_race/phase_offset_cbf_constraints.h"

#include <cmath>

namespace FLAG_Race
{

void
PhaseOffsetCbfConstraints::appendTubeConstraints(
  const PhaseOffsetGeometry& geometry, const TubeBounds& tube,
  std::vector<LinearConstraint2D>& constraints) const
{
  if (!tube.valid)
    return;
  // When the tube is narrower than 2*margin the two CBF half-spaces would
  // contradict each other (both h become negative and demand a delta rate the
  // slew limit cannot provide).  Shrink the margin to fit the tube so the
  // center band stays feasible (the boundary itself is then kept at zero
  // margin, which is the best the geometry allows).
  const double tube_half = 0.5 * (tube.upper - tube.lower);
  const double margin_eff =
    std::max(0.0, std::min(tube_margin, tube_half - 1e-6));
  const double h_plus = tube.upper - geometry.delta - margin_eff;
  const double h_minus = geometry.delta - tube.lower - margin_eff;
  const double w_dot = geometry.base_w_dot;
  constraints.push_back(
    {Eigen::Vector2d(tube.upper_dw, -1.0),
     -cbf_gamma_tube * h_plus - tube.upper_dw * w_dot, "tube_upper"});
  constraints.push_back(
    {Eigen::Vector2d(-tube.lower_dw, 1.0),
     -cbf_gamma_tube * h_minus + tube.lower_dw * w_dot, "tube_lower"});
}

void
PhaseOffsetCbfConstraints::appendPairwiseConstraints(
  const PhaseOffsetGeometry& geometry,
  const Eigen::Vector3d& self_position,
  const std::vector<PredictedNeighborState,
                    Eigen::aligned_allocator<PredictedNeighborState>>&
    safety_neighbors,
  std::vector<LinearConstraint2D>& constraints) const
{
  for (const PredictedNeighborState& n : safety_neighbors)
  {
    const Eigen::Vector3d r = self_position - n.predicted_position;
    const double d2 = r.squaredNorm();
    const double tau_max = std::max(0.0, max_message_age);
    const double d_rob =
      physical_safe_distance + self_position_error_bound +
      neighbor_position_error_bound +
      0.5 * neighbor_acceleration_bound * tau_max * tau_max +
      0.5 * (self_acceleration_bound + neighbor_acceleration_bound) *
        dt_qp_max * dt_qp_max +
      communication_position_margin;
    const double h = d2 - d_rob * d_rob;
    const double rho = 2.0 * r.norm() * n.velocity_uncertainty_bound;
    const Eigen::Vector3d dv = geometry.base_v - n.advertised_velocity;

    LinearConstraint2D c;
    c.a(0) = 2.0 * r.dot(geometry.r_w);
    c.a(1) = 2.0 * r.dot(geometry.N);
    c.b = -cbf_gamma_pair * h + rho - 2.0 * r.dot(dv);
    c.label = "pair_cbf_" + std::to_string(n.robot_id);
    constraints.push_back(c);
  }
}

}  // namespace FLAG_Race
