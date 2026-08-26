#include "bspline_race/phase_offset_geometry.h"
#include <phase_offset_core/normal_frame.h>

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
    if (!std::isfinite(v_p) || v_p < 1e-9)
    {
      out.ok = false;
      out.reason = "degenerate_tangent";
      return false;
    }
    const Eigen::Vector3d T = st.dp_dw / v_p;
    const Eigen::Vector3d horizontal_cross =
        Eigen::Vector3d::UnitZ().cross(st.dp_dw);
    const double horizontal_speed = horizontal_cross.norm();
    if (!std::isfinite(horizontal_speed)) {
      out.ok = false;
      out.reason = "horizontal_normal_speed_not_finite";
      return false;
    }
    if (st.frame_valid &&
        !phase_offset_core::isWorldHorizontalCrossProductProvenance(
            st.frame_provenance)) {
      out.ok = false;
      out.reason = "normal_frame_provenance_incompatible";
      return false;
    }
    if (st.frame_valid &&
        (!st.T.allFinite() || !st.N.allFinite() || !st.N_w.allFinite() ||
         std::abs(st.T.norm() - 1.0) > 1e-6 ||
         std::abs(st.T.dot(T) - 1.0) > 1e-6 ||
         std::abs(st.N.norm() - 1.0) > 1e-6 ||
         std::abs(st.N.z()) > 1e-12 ||
         std::abs(st.N_w.z()) > 1e-12)) {
      out.ok = false;
      out.reason = "normal_frame_geometry_incompatible";
      return false;
    }
    const bool horizontal_capability =
        horizontal_speed > phase_offset_core::kHorizontalNormalSpeedEpsilon;
    Eigen::Vector3d N = Eigen::Vector3d::Zero();
    Eigen::Vector3d N_w = Eigen::Vector3d::Zero();
    if (horizontal_capability) {
      N = horizontal_cross / horizontal_speed;
      const Eigen::Vector3d h_w = Eigen::Vector3d::UnitZ().cross(st.d2p_dw2);
      N_w = (h_w - N * N.dot(h_w)) / horizontal_speed;
      if (st.frame_valid) {
        if (std::abs(st.N.dot(N) - 1.0) > 1e-8 ||
            (st.N - N).norm() > 1e-8 ||
            (st.N_w - N_w).norm() > 1e-8 ||
            std::abs(st.N_w.z()) > 1e-12) {
          out.ok = false;
          out.reason = "normal_frame_geometry_incompatible";
          return false;
        }
      }
    }
    if (st.frame_valid && !horizontal_capability && delta != 0.0) {
      out.ok = false;
      out.reason = "horizontal_normal_speed_too_small";
      return false;
    }
    if (!horizontal_capability && delta != 0.0) {
      out.ok = false;
      out.reason = "horizontal_normal_speed_too_small";
      return false;
    }
    const double regularity = (st.dp_dw + N_w * delta).norm();
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
  if (!std::isfinite(v_p) || v_p < 1e-9 || !p_w.allFinite() ||
      !p_ww.allFinite() ||
      !std::isfinite(delta))
  {
    out.invalid_reason = "nonfinite_or_degenerate";
    return false;
  }

  const Eigen::Vector3d T = p_w / v_p;
  const Eigen::Vector3d horizontal_cross =
      Eigen::Vector3d::UnitZ().cross(p_w);
  const double horizontal_speed = horizontal_cross.norm();
  if (!std::isfinite(horizontal_speed)) {
    out.invalid_reason = "horizontal_normal_speed_not_finite";
    return false;
  }
  if (path_state.frame_valid &&
      !phase_offset_core::isWorldHorizontalCrossProductProvenance(
          path_state.frame_provenance)) {
    out.invalid_reason = "normal_frame_provenance_incompatible";
    return false;
  }
  if (path_state.frame_valid &&
      (!path_state.T.allFinite() || !path_state.N.allFinite() ||
       !path_state.N_w.allFinite() || std::abs(path_state.T.norm() - 1.0) > 1e-6 ||
       std::abs(path_state.T.dot(T) - 1.0) > 1e-6 ||
       std::abs(path_state.N.norm() - 1.0) > 1e-6 ||
       std::abs(path_state.N.z()) > 1e-12 ||
       std::abs(path_state.N_w.z()) > 1e-12)) {
    out.invalid_reason = "normal_frame_geometry_incompatible";
    return false;
  }
  const bool horizontal_capability =
      horizontal_speed > phase_offset_core::kHorizontalNormalSpeedEpsilon;
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  Eigen::Vector3d N_w = Eigen::Vector3d::Zero();
  if (horizontal_capability) {
    N = horizontal_cross / horizontal_speed;
    const Eigen::Vector3d h_w = Eigen::Vector3d::UnitZ().cross(p_ww);
    N_w = (h_w - N * N.dot(h_w)) / horizontal_speed;
    if (path_state.frame_valid) {
      if (std::abs(path_state.N.dot(N) - 1.0) > 1e-8 ||
          (path_state.N - N).norm() > 1e-8 ||
          (path_state.N_w - N_w).norm() > 1e-8 ||
          std::abs(path_state.N_w.z()) > 1e-12) {
        out.invalid_reason = "normal_frame_geometry_incompatible";
        return false;
      }
    }
  } else if (path_state.frame_valid && delta != 0.0) {
    out.invalid_reason = "horizontal_normal_speed_too_small";
    return false;
  }
  if (!horizontal_capability && delta != 0.0) {
    out.invalid_reason = "horizontal_normal_speed_too_small";
    return false;
  }
  const double regularity = (p_w + N_w * delta).norm();
  if (regularity < params.minimum_reference_speed)
  {
    out.invalid_reason = "r_w_degenerate";
    return false;
  }

  const Eigen::Vector3d r = p + N * delta;
  const Eigen::Vector3d r_w = p_w + N_w * delta;
  if (r_w.norm() < params.minimum_reference_speed)
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
  out.curvature = horizontal_speed >
      phase_offset_core::kHorizontalNormalSpeedEpsilon
      ? (p_w.x() * p_ww.y() - p_w.y() * p_ww.x()) /
          (horizontal_speed * horizontal_speed * horizontal_speed)
      : 0.0;
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
