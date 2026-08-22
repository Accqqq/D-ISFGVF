#include "bspline_race/phase_offset_allocator.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace FLAG_Race
{

namespace
{

Eigen::Matrix2d buildH(const PhaseOffsetGeometry& g,
                       const AllocatorParams& p)
{
  Eigen::Matrix<double, 3, 2> J(3, 2);
  J.col(0) = g.r_w;
  J.col(1) = g.N;
  Eigen::Matrix2d H = p.weight_world_error * J.transpose() * J;
  H(0, 0) += p.weight_u_w;
  H(1, 1) += p.weight_u_delta;
  H(0, 0) += p.weight_delta_u_w;
  H(1, 1) += p.weight_delta_u_delta;
  return H;
}

Eigen::Vector2d buildF(const PhaseOffsetGeometry& g,
                       const Eigen::Vector3d& g_des,
                       const AllocatorParams& p, const PortCommand& prev)
{
  Eigen::Matrix<double, 3, 2> J(3, 2);
  J.col(0) = g.r_w;
  J.col(1) = g.N;
  Eigen::Vector2d f = -p.weight_world_error * J.transpose() * g_des;
  f(0) -= p.weight_delta_u_w * prev.u_w;
  f(1) -= p.weight_delta_u_delta * prev.u_delta;
  return f;
}

double objective(const Eigen::Vector2d& u, const Eigen::Matrix2d& H,
                 const Eigen::Vector2d& f)
{
  return 0.5 * u.dot(H * u) + f.dot(u);
}

bool feasible(const Eigen::Vector2d& u,
              const std::vector<LinearConstraint2D>& cons, double tol = 1e-6)
{
  for (const LinearConstraint2D& c : cons)
    if (c.a.dot(u) < c.b - tol)
      return false;
  return true;
}

// Project the unconstrained optimum onto the boundary of one constraint.
bool projectOnto(const Eigen::Vector2d& u0, const Eigen::Matrix2d& H,
                 const Eigen::Vector2d& f, const LinearConstraint2D& c,
                 Eigen::Vector2d& out)
{
  const double an = c.a.norm();
  if (an < 1e-12)
    return false;
  const Eigen::Vector2d a = c.a / an;
  const double b = c.b / an;
  // min 1/2 u'Hu + f'u s.t. a'u = b:
  // u = u0 - H^-1 a (a' H^-1 a)^-1 (a' u0 - b)
  const Eigen::Matrix2d Hinv = H.inverse();
  const double denom = a.dot(Hinv * a);
  if (denom < 1e-12)
    return false;
  out = u0 - Hinv * a * ((a.dot(u0) - b) / denom);
  return true;
}

// Intersection of two constraint boundaries.
bool intersect(const LinearConstraint2D& c1, const LinearConstraint2D& c2,
               Eigen::Vector2d& out)
{
  Eigen::Matrix2d M;
  M.row(0) = c1.a.transpose();
  M.row(1) = c2.a.transpose();
  const Eigen::Vector2d rhs(c1.b, c2.b);
  const double det = M.determinant();
  if (std::abs(det) < 1e-12)
    return false;
  out = M.inverse() * rhs;
  return true;
}

}  // namespace

Eigen::Vector2d
PhaseOffsetAllocator::analyticRawPort(const PhaseOffsetGeometry& geometry,
                                      const Eigen::Vector3d& g_des)
{
  Eigen::Vector2d u;
  const double rw2 = geometry.r_w.squaredNorm();
  u(0) = rw2 > 1e-12 ? geometry.r_w.dot(g_des) / rw2 : 0.0;
  u(1) = geometry.N.dot(g_des);
  return u;
}

Eigen::Vector2d
PhaseOffsetAllocator::unconstrainedSolution(
  const PhaseOffsetGeometry& geometry, const Eigen::Vector3d& g_des,
  const AllocatorParams& p, const PortCommand& previous)
{
  const Eigen::Matrix2d H = buildH(geometry, p);
  const Eigen::Vector2d f = buildF(geometry, g_des, p, previous);
  return -H.inverse() * f;
}

PortCommand
PhaseOffsetAllocator::solve(
  const PhaseOffsetGeometry& geometry, const TubeBounds& tube,
  const SwarmIntent& intent,
  const std::vector<PredictedNeighborState,
                    Eigen::aligned_allocator<PredictedNeighborState>>&
    safety_neighbors,
  double dt, const PortCommand& previous, SwarmControlMode requested_mode,
  const std::vector<LinearConstraint2D>& extra_constraints)
{
  PortCommand result;
  (void)safety_neighbors;
  const double dt_qp = std::max(1e-3, std::min(params.qp_dt_max, dt));

  const double w_dot_base = geometry.base_w_dot;
  const double rw_norm = geometry.r_w.norm();

  // Rolling constraint set.
  std::vector<LinearConstraint2D> rolling;
  // Speed-band constraints keep u=0 feasible: when the base speed already
  // leaves the band, the bound on the non-empty side is relaxed to 0 so the
  // allocator can hold the current rate instead of becoming infeasible.
  rolling.push_back({Eigen::Vector2d(1.0, 0.0),
                     std::min(params.phase_speed_min - w_dot_base, 0.0),
                     "phase_min"});
  rolling.push_back({Eigen::Vector2d(-1.0, 0.0),
                     std::min(w_dot_base - params.phase_speed_max, 0.0),
                     "phase_max"});
  if (rw_norm > 1e-9)
  {
    rolling.push_back(
      {Eigen::Vector2d(rw_norm, 0.0),
       std::min(params.tangent_speed_min - params.K1 * geometry.alpha, 0.0),
       "tangent_min"});
    rolling.push_back(
      {Eigen::Vector2d(-rw_norm, 0.0),
       std::min(params.K1 * geometry.alpha - params.tangent_speed_max, 0.0),
       "tangent_max"});
  }
  rolling.push_back({Eigen::Vector2d(0.0, 1.0),
                     (tube.valid ? tube.lower : -1e9) - geometry.delta,
                     "delta_lower"});
  rolling.push_back({Eigen::Vector2d(0.0, -1.0),
                     geometry.delta - (tube.valid ? tube.upper : 1e9),
                     "delta_upper"});
  rolling.push_back({Eigen::Vector2d(1.0, 0.0),
                     previous.u_w - params.u_w_slew_rate * dt_qp,
                     "uw_slew_low"});
  rolling.push_back({Eigen::Vector2d(-1.0, 0.0),
                     -previous.u_w - params.u_w_slew_rate * dt_qp,
                     "uw_slew_high"});
  rolling.push_back({Eigen::Vector2d(0.0, 1.0),
                     previous.u_delta - params.u_delta_slew_rate * dt_qp,
                     "ud_slew_low"});
  rolling.push_back({Eigen::Vector2d(0.0, -1.0),
                     -previous.u_delta - params.u_delta_slew_rate * dt_qp,
                     "ud_slew_high"});
  for (const LinearConstraint2D& c : extra_constraints)
    rolling.push_back(c);

  // Nonnegative-progress set: only w_dot >= 0 + slew + delta bounds.
  std::vector<LinearConstraint2D> nonneg;
  nonneg.push_back({Eigen::Vector2d(1.0, 0.0), -w_dot_base,
                    "phase_nonneg"});
  nonneg.push_back({Eigen::Vector2d(0.0, 1.0),
                    (tube.valid ? tube.lower : -1e9) - geometry.delta,
                    "delta_lower"});
  nonneg.push_back({Eigen::Vector2d(0.0, -1.0),
                    geometry.delta - (tube.valid ? tube.upper : 1e9),
                    "delta_upper"});
  nonneg.push_back({Eigen::Vector2d(1.0, 0.0),
                    previous.u_w - params.u_w_slew_rate * dt_qp,
                    "uw_slew_low"});
  nonneg.push_back({Eigen::Vector2d(-1.0, 0.0),
                    -previous.u_w - params.u_w_slew_rate * dt_qp,
                    "uw_slew_high"});
  nonneg.push_back({Eigen::Vector2d(0.0, 1.0),
                    previous.u_delta - params.u_delta_slew_rate * dt_qp,
                    "ud_slew_low"});
  nonneg.push_back({Eigen::Vector2d(0.0, -1.0),
                    -previous.u_delta - params.u_delta_slew_rate * dt_qp,
                    "ud_slew_high"});
  // CBF constraints (pairwise / tube) must be enforced in EVERY mode,
  // including the nonnegative-progress fallback.
  for (const LinearConstraint2D& c : extra_constraints)
    nonneg.push_back(c);

  const Eigen::Vector3d g_des = intent.g_des;
  const Eigen::Matrix2d H = buildH(geometry, params);
  const Eigen::Vector2d f = buildF(geometry, g_des, params, previous);
  const Eigen::Vector2d u0 = -H.inverse() * f;

  auto solveSet = [&](const std::vector<LinearConstraint2D>& cons,
                      SwarmControlMode mode, PortCommand& out) {
    std::vector<Eigen::Vector2d> candidates;
    candidates.push_back(u0);
    for (const LinearConstraint2D& c : cons)
    {
      Eigen::Vector2d p;
      if (projectOnto(u0, H, f, c, p))
        candidates.push_back(p);
    }
    for (size_t i = 0; i < cons.size(); ++i)
      for (size_t j = i + 1; j < cons.size(); ++j)
      {
        Eigen::Vector2d p;
        if (intersect(cons[i], cons[j], p))
          candidates.push_back(p);
      }

    double best_cost = std::numeric_limits<double>::infinity();
    Eigen::Vector2d best = u0;
    bool found = false;
    for (const Eigen::Vector2d& u : candidates)
    {
      if (!u.allFinite() || !feasible(u, cons))
        continue;
      const double cost = objective(u, H, f);
      if (std::getenv("ALLOC_DEBUG"))
        fprintf(stderr, "[alloc] cand u=(%.4f, %.4f) cost=%.6f\n",
                u(0), u(1), cost);
      if (cost < best_cost)
      {
        best_cost = cost;
        best = u;
        found = true;
      }
    }
    if (!found)
      return false;
    if (std::getenv("ALLOC_DEBUG"))
      fprintf(stderr, "[alloc] %s chosen u=(%.4f, %.4f)\n",
              mode == SwarmControlMode::ROLLING ? "ROLLING"
              : mode == SwarmControlMode::SAFETY_PRIORITY ? "SAFETY"
              : mode == SwarmControlMode::TERMINAL ? "TERMINAL" : "?",
              best(0), best(1));
    out.u_w = best(0);
    out.u_delta = best(1);
    out.objective = best_cost;
    out.mode = mode;
    out.feasible = true;
    const Eigen::Vector3d J_u = geometry.r_w * best(0) + geometry.N * best(1);
    out.allocation_residual = (J_u - g_des).norm();
    for (const LinearConstraint2D& c : cons)
      if (c.a.dot(best) <= c.b + 1e-6)
        out.active_constraints.push_back(c.label);
    return true;
  };

  if (requested_mode == SwarmControlMode::TERMINAL)
  {
    // Allow the total phase speed to decay to zero; keep the safety bounds.
    if (solveSet(nonneg, SwarmControlMode::TERMINAL, result))
      return result;
    auto nonneg_safety = nonneg;
    for (LinearConstraint2D& c : nonneg_safety)
    {
      if (c.label.find("slew") != std::string::npos)
        c.b -= 30.0 * params.u_w_slew_rate * dt_qp;
    }
    if (solveSet(nonneg_safety, SwarmControlMode::SAFETY_PRIORITY, result))
      return result;
    if (std::getenv("ALLOC_DEBUG"))
    {
      fprintf(stderr, "[alloc] TERMINAL infeasible: delta=%.4f "
                      "tube_l=%.4f tube_u=%.4f wdot=%.4f prev=(%.4f,%.4f)\n",
              geometry.delta, tube.lower, tube.upper, geometry.base_w_dot,
              previous.u_w, previous.u_delta);
      for (const LinearConstraint2D& c : nonneg)
        fprintf(stderr, "[alloc]   nonneg %-16s a=(%+.4f,%+.4f) b=%+.4f\n",
                c.label.c_str(), c.a(0), c.a(1), c.b);
    }
    result.mode = SwarmControlMode::EMERGENCY;
    result.feasible = false;
    return result;
  }

  if (solveSet(rolling, SwarmControlMode::ROLLING, result))
    return result;
  // Batch 10: when the nominal slew-limited QP is infeasible, the CBF safety
  // constraints must still be able to act: relax ONLY the slew bounds (the
  // nominal smoothness limit) and re-solve with the CBF constraints kept.
  auto safety_rolling = rolling;
  for (LinearConstraint2D& c : safety_rolling)
  {
    if (c.label.find("slew") != std::string::npos)
      c.b -= 30.0 * params.u_w_slew_rate * dt_qp;
  }
  if (solveSet(safety_rolling, SwarmControlMode::SAFETY_PRIORITY, result))
    return result;
  if (solveSet(nonneg, SwarmControlMode::SAFETY_PRIORITY, result))
    return result;
  auto nonneg_safety = nonneg;
  for (LinearConstraint2D& c : nonneg_safety)
  {
    if (c.label.find("slew") != std::string::npos)
      c.b -= 30.0 * params.u_w_slew_rate * dt_qp;
  }
  if (solveSet(nonneg_safety, SwarmControlMode::SAFETY_PRIORITY, result))
    return result;
  if (std::getenv("ALLOC_DEBUG"))
  {
    fprintf(stderr, "[alloc] ROLLING+SAFETY infeasible: delta=%.4f "
                    "tube_l=%.4f tube_u=%.4f wdot=%.4f prev=(%.4f,%.4f)\n",
            geometry.delta, tube.lower, tube.upper, geometry.base_w_dot,
            previous.u_w, previous.u_delta);
    for (const LinearConstraint2D& c : rolling)
      fprintf(stderr, "[alloc]   rolling %-16s a=(%+.4f,%+.4f) b=%+.4f\n",
              c.label.c_str(), c.a(0), c.a(1), c.b);
  }
  result.mode = SwarmControlMode::EMERGENCY;
  result.feasible = false;
  return result;
}

}  // namespace FLAG_Race
