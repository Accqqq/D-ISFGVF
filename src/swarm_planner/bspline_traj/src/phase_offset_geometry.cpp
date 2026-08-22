#include "bspline_race/phase_offset_geometry.h"

#include <algorithm>
#include <cmath>

namespace FLAG_Race
{

bool
checkC2ConnectorSamples(
  const std::vector<ContinuousPhasePathState>& samples,
  double delta,
  double mu_regular,
  const std::function<double(const Eigen::Vector3d&)>& obstacle_distance,
  double required_clearance,
  C2ConnectorCheckResult& out)
{
  out = C2ConnectorCheckResult();
  if (samples.empty() || !std::isfinite(delta))
  {
    out.ok = false;
    out.reason = "empty_samples_or_delta";
    return false;
  }
  for (const ContinuousPhasePathState& st : samples)
  {
    if (!st.valid || !st.p.allFinite() || !st.dp_dw.allFinite() ||
        !st.d2p_dw2.allFinite())
    {
      out.ok = false;
      out.reason = "nonfinite_sample";
      return false;
    }
    const double v_p = st.dp_dw.norm();
    if (v_p < 1e-9)
    {
      out.ok = false;
      out.reason = "degenerate_tangent";
      return false;
    }
    const Eigen::Vector3d T = st.dp_dw / v_p;
    const Eigen::Vector3d N(-T.y(), T.x(), 0.0);
    const double kappa =
      (st.dp_dw.x() * st.d2p_dw2.y() - st.dp_dw.y() * st.d2p_dw2.x()) /
      (v_p * v_p * v_p);
    const double regularity = 1.0 - kappa * delta;
    out.min_regularity = std::min(out.min_regularity, regularity);
    if (regularity < mu_regular)
    {
      out.ok = false;
      out.reason = "regularity_below_mu";
      return false;
    }

    if (obstacle_distance)
    {
      const Eigen::Vector3d r = st.p + N * delta;
      const double d_obs = obstacle_distance(r);
      if (std::isfinite(d_obs))
      {
        out.min_obstacle_distance = std::min(out.min_obstacle_distance,
                                             d_obs);
        if (d_obs < required_clearance)
        {
          out.ok = false;
          out.reason = "active_reference_too_close";
          return false;
        }
      }
    }
  }
  return true;
}

bool
PhaseOffsetGeometryEvaluator::evaluate(
  const ContinuousPhasePathState& path_state,
  const Eigen::Vector3d& position,
  double delta,
  const PhaseOffsetGeometryParams& params,
  PhaseOffsetGeometry& out) const
{
  out = PhaseOffsetGeometry();
  out.p = path_state.p;
  out.dp_dw = path_state.dp_dw;
  out.d2p_dw2 = path_state.d2p_dw2;
  out.delta = delta;

  const Eigen::Vector3d p = path_state.p;
  const Eigen::Vector3d p_w = path_state.dp_dw;
  const Eigen::Vector3d p_ww = path_state.d2p_dw2;
  const double v_p = p_w.norm();
  if (v_p < 1e-9 || !p_w.allFinite() || !p_ww.allFinite() ||
      !std::isfinite(delta))
  {
    out.invalid_reason = "nonfinite_or_degenerate";
    return false;
  }

  // Level-flight guard: only approximately horizontal paths enter the
  // phase-offset mode.
  if (p_w.head<2>().norm() < params.v_xy_min)
  {
    out.invalid_reason = "v_xy_too_small";
    return false;
  }

  const Eigen::Vector3d T = p_w / v_p;
  if (std::abs(T.z()) > params.z_tolerance)
  {
    out.invalid_reason = "T_z_out_of_tolerance";
    return false;
  }

  const Eigen::Vector3d N(-T.y(), T.x(), 0.0);
  const double kappa =
    (p_w.x() * p_ww.y() - p_w.y() * p_ww.x()) / (v_p * v_p * v_p);

  // Regularity: 1 - kappa*delta >= mu > 0.
  const double regularity = 1.0 - kappa * delta;
  if (regularity < params.mu_regular)
  {
    out.invalid_reason = "regularity_below_mu";
    return false;
  }

  const Eigen::Vector3d r = p + N * delta;
  const Eigen::Vector3d r_w = regularity * p_w;
  if (r_w.norm() < 1e-9)
  {
    out.invalid_reason = "r_w_degenerate";
    return false;
  }

  // Error decomposition on the active reference.
  const Eigen::Vector3d tau = r_w / r_w.norm();
  const Eigen::Vector3d e = position - r;
  const double e_parallel = tau.dot(e);
  const Eigen::Vector3d e_perp = e - e_parallel * tau;
  const double rho = e_perp.norm();

  // ISF shaping terms (same convention as the legacy lifted guidance).
  const double bandwidth = std::max(1e-6, params.convergence_bandwidth);
  const double q =
    (rho > 1e-6) ? std::tanh(rho / bandwidth) / rho : 1.0 / bandwidth;
  const double rho0 = std::max(1e-6, params.rho0);
  const double alpha =
    params.alpha_min +
    (1.0 - params.alpha_min) / (1.0 + (rho / rho0) * (rho / rho0));
  const double sigma = std::tanh(e_parallel / std::max(1e-6, params.delta_band));

  const Eigen::Vector3d base_v =
    params.K1 * alpha * T + params.K2 * q * e_perp;
  const double base_w_dot = params.K1 * (alpha + sigma) / r_w.norm();

  out.T = T;
  out.N = N;
  out.r = r;
  out.r_w = r_w;
  out.e_perp = e_perp;
  out.base_v = base_v;
  out.curvature = kappa;
  out.e_parallel = e_parallel;
  out.rho = rho;
  out.alpha = alpha;
  out.q = q;
  out.sigma = sigma;
  out.base_w_dot = base_w_dot;
  out.valid = true;
  return true;
}

}  // namespace FLAG_Race
