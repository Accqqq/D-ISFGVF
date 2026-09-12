#pragma once

#include <Eigen/Core>

#include <string>

#include "phase_offset_swarm/agent_state.h"
#include "phase_offset_swarm/neighbor_state.h"

namespace phase_offset_swarm {

// Horizontal SPH workspace coordination, exactly as written in the paper's
// "SPH Workspace Coordination with Path-Tube Preview" section:
//
//   q_ij      = d_ij / h
//   Wbar(q)   = cubic compact-support shape on [0, 2]
//   phi(q)    = -dWbar/dq >= 0
//   rho_i     = 1 + sum_j Wbar(q_ij)              (local density index)
//   rho_0     = 1 + sum_j Wbar(d_star / h)        (nominal/open density)
//   eta_i     = (rho_i / rho_0)^gamma - 1
//   s_i       = sum_j phi(q_ij) n_ij              (density direction)
//   g_den,rep = -k_rho * eta^+ / rho_i * s_i
//   g_den,att = +k_rho * eta^- / rho_i * s_i      (modulated by beta_i)
//   g_rep     = -k_rep * sum_j [1 - d_ij/d_rep]_+^2 n_ij
//   g_damp    = +k_d  * sum_j (Wbar(q_ij)/rho_i) ddot_ij n_ij
//
// with n_ij = (x_j - x_i)/d_ij pointing from i to j and
// ddot_ij = (v_j - v_i)^T n_ij the radial relative speed.  A positive
// ddot_ij means the pair is separating, so the damping term opposes both
// relative contraction and relative expansion and vanishes under common
// translation.
//
// Gains are expressed directly in [m/s]: the paper absorbs the constant
// kernel normalization into the coordination gains, so no separate
// normalization/mass/rest-density triple is carried here.
//
// rho_0 is the density of the *nominal* configuration
// (paper: rho_0 = 1 + sum_{j in N_i^*} Wbar(d*_ij/h)).  The scenario generator
// knows the formation geometry and passes the per-agent value through
// `reference_density`; with `reference_density <= 0` the provider falls back to
// its local estimate 1 + N_vis * Wbar(d_star/h), which is only exact for an
// agent whose visible neighbours all sit at d_star (the centre of a disk
// formation) and is biased low for everyone else.
struct SphParameters {
  // Smoothing length [m].  The kernel support is 2*h; the paper requires
  // d_rep < d_star < 2*h.
  double h = 2.0;
  // d_star: nominal inter-UAV spacing of the formation [m].
  double reference_spacing = 1.5;
  // rho_0: reference (open-space nominal) density for THIS agent.  Zero means
  // "estimate it locally from reference_spacing"; a positive value is used
  // verbatim and is the paper-faithful setting.
  double reference_density = 0.0;
  // d_rep: activation distance of the bounded short-range repulsion [m].
  // Kept just below the nominal spacing so there is no dead band between
  // d_rep and d_star: zero at the nominal configuration, pulling as soon as a
  // pair closes in.
  double d_rep = 1.4;
  // gamma: density sensitivity in eta = (rho/rho_0)^gamma - 1.
  double gamma = 7.0;
  // k_rho: density branch gain (paper's k_rho) [m/s].
  // The paper absorbs the constant kernel normalization into the gains, so
  // this value is implementation-defined.  0.15 is the measured compromise
  // between a visible coordination term and a formation that keeps its
  // spacing (see docs/PhaseOffsetSwarm_SPH_Paper_Conformance_2026-09-11.md).
  double k_rho = 0.15;
  // k_rep: bounded short-range repulsion gain [m/s].
  double k_rep = 1.2;
  // k_d: relative radial damping gain [m/s].
  double k_damp = 1.0;
  double g_max = 1.5;
  double distance_epsilon = 1.0e-6;

  bool validate(std::string* error = nullptr) const;
  void validateOrThrow() const;
};

enum class SphIntentStatus {
  VALID = 0,
  INVALID_SELF = 1,
  INVALID_BETA = 2,
  INVALID_FRESH_NEIGHBOR = 3,
  DUPLICATE_FRESH_ID = 4,
  NUMERIC_FAILURE = 5
};

struct SphIntentOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector2d g_density_attractive = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_density_repulsive = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_explicit_repulsive = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_damping = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_coord = Eigen::Vector2d::Zero();
  double rho_i = 0.0;
  double rho_reference = 0.0;
  double eta_i = 0.0;
  double beta_used = 0.0;
  int support_neighbor_count = 0;
  int ignored_nonfresh_count = 0;
  int invalid_input_count = 0;
  int near_zero_count = 0;
  bool output_saturated = false;
  bool valid = false;
  SphIntentStatus status = SphIntentStatus::INVALID_SELF;
};

class SphIntentProvider {
 public:
  explicit SphIntentProvider(const SphParameters& parameters);

  SphIntentOutput compute(const SwarmAgentState& self,
                          const NeighborSnapshot& snapshot,
                          double beta_i) const;

 private:
  SphParameters parameters_;
};

}  // namespace phase_offset_swarm
