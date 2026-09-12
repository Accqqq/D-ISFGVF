#include "phase_offset_swarm/sph_intent.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

namespace phase_offset_swarm {
namespace {

// Cubic compact-support shape Wbar(q) of the paper, written without its
// constant normalization (that factor is absorbed into the gains).
double cubicKernel(double k) {
  if (k >= 0.0 && k <= 1.0) {
    return 1.0 - 1.5 * k * k + 0.75 * k * k * k;
  }
  if (k > 1.0 && k < 2.0) {
    const double tail = 2.0 - k;
    return 0.25 * tail * tail * tail;
  }
  return 0.0;
}

// dWbar/dq; non-positive on the support.
double cubicKernelDerivative(double k) {
  if (k >= 0.0 && k <= 1.0) {
    return -3.0 * k + 2.25 * k * k;
  }
  if (k > 1.0 && k < 2.0) {
    const double tail = 2.0 - k;
    return -0.75 * tail * tail;
  }
  return 0.0;
}

// phi(q) = -dWbar/dq >= 0.
double cubicKernelNegDerivative(double k) {
  return -cubicKernelDerivative(k);
}

SphIntentOutput invalidOutput(SphIntentStatus status) {
  SphIntentOutput output;
  output.status = status;
  return output;
}

bool finiteVector(const Eigen::Vector2d& value) { return value.allFinite(); }

bool finiteScalar(double value) { return std::isfinite(value); }

}  // namespace

bool SphParameters::validate(std::string* error) const {
  if (error != nullptr) {
    error->clear();
  }
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };

  if (!std::isfinite(h) || !std::isfinite(reference_spacing) ||
      !std::isfinite(reference_density) || !std::isfinite(d_rep) ||
      !std::isfinite(gamma) ||
      !std::isfinite(k_rho) || !std::isfinite(k_rep) ||
      !std::isfinite(k_damp) || !std::isfinite(g_max) ||
      !std::isfinite(distance_epsilon)) {
    return fail("all floating parameters must be finite");
  }
  if (!(h > 0.0)) {
    return fail("h must be strictly positive");
  }
  if (!(reference_spacing > 0.0)) {
    return fail("reference_spacing must be strictly positive");
  }
  // 0 selects the local estimate; anything positive is the nominal density.
  if (reference_density < 0.0) {
    return fail("reference_density must be non-negative");
  }
  if (!(d_rep > 0.0)) {
    return fail("d_rep must be strictly positive");
  }
  // Paper ordering: 0 < d_rep < d_star < 2h.
  if (!(d_rep < reference_spacing)) {
    return fail("d_rep must be strictly smaller than reference_spacing");
  }
  if (!(reference_spacing < 2.0 * h)) {
    return fail("reference_spacing must be strictly smaller than 2*h");
  }
  if (k_rho < 0.0 || k_rep < 0.0 || k_damp < 0.0) {
    return fail("density, repulsion, and viscosity gains must be nonnegative");
  }
  if (!(gamma > 0.0)) {
    return fail("gamma must be strictly positive");
  }
  if (!(g_max > 0.0)) {
    return fail("g_max must be strictly positive");
  }
  const double support_limit = 2.0 * h;
  if (!std::isfinite(support_limit)) {
    return fail("2*h must be finite");
  }
  if (!(distance_epsilon > 0.0 && distance_epsilon < support_limit)) {
    return fail("distance_epsilon must satisfy 0 < distance_epsilon < 2*h");
  }
  return true;
}

void SphParameters::validateOrThrow() const {
  std::string error;
  if (!validate(&error)) {
    throw std::invalid_argument("invalid SphParameters: " + error);
  }
}

SphIntentProvider::SphIntentProvider(const SphParameters& parameters)
    : parameters_(parameters) {
  parameters_.validateOrThrow();
}

SphIntentOutput SphIntentProvider::compute(const SwarmAgentState& self,
                                           const NeighborSnapshot& snapshot,
                                           double beta_i) const {
  if (!self.isValid()) {
    return invalidOutput(SphIntentStatus::INVALID_SELF);
  }
  if (!finiteScalar(beta_i) || beta_i < 0.0 || beta_i > 1.0) {
    return invalidOutput(SphIntentStatus::INVALID_BETA);
  }

  std::vector<const NeighborState*> fresh_neighbors;
  fresh_neighbors.reserve(snapshot.all.size());
  int ignored_nonfresh_count = 0;
  for (const NeighborState& neighbor : snapshot.all) {
    if (neighbor.freshness != NeighborFreshness::FRESH) {
      if (ignored_nonfresh_count == std::numeric_limits<int>::max()) {
        return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
      }
      ++ignored_nonfresh_count;
      continue;
    }
    if (neighbor.id < 0 || neighbor.id == self.id ||
        !finiteVector(neighbor.position) ||
        !finiteVector(neighbor.velocity)) {
      return invalidOutput(SphIntentStatus::INVALID_FRESH_NEIGHBOR);
    }
    fresh_neighbors.push_back(&neighbor);
  }

  std::set<int> fresh_ids;
  for (const NeighborState* neighbor : fresh_neighbors) {
    if (!fresh_ids.insert(neighbor->id).second) {
      return invalidOutput(SphIntentStatus::DUPLICATE_FRESH_ID);
    }
  }

  const double support_limit = 2.0 * parameters_.h;
  const double q_reference = parameters_.reference_spacing / parameters_.h;
  const double w_reference = cubicKernel(q_reference);
  if (!finiteScalar(support_limit) || !finiteScalar(q_reference) ||
      !finiteScalar(w_reference)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }

  // The self term represents the unit contribution of UAV i itself.
  double rho_i = 1.0;
  Eigen::Vector2d density_direction = Eigen::Vector2d::Zero();
  Eigen::Vector2d repulsion_sum = Eigen::Vector2d::Zero();
  Eigen::Vector2d damping_sum = Eigen::Vector2d::Zero();
  int support_neighbor_count = 0;
  int near_zero_count = 0;

  for (const NeighborState* neighbor : fresh_neighbors) {
    // n_ij points from UAV i to UAV j, as in the paper.
    const Eigen::Vector2d displacement = neighbor->position - self.position;
    if (!finiteVector(displacement)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    const double distance = std::hypot(displacement.x(), displacement.y());
    if (!finiteScalar(distance) || distance < 0.0) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    if (!(distance < support_limit)) {
      continue;
    }
    if (support_neighbor_count == std::numeric_limits<int>::max()) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    ++support_neighbor_count;

    const bool near_zero = distance <= parameters_.distance_epsilon;
    if (near_zero) {
      if (near_zero_count == std::numeric_limits<int>::max()) {
        return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
      }
      ++near_zero_count;
      // A coincident neighbor is counted at full kernel weight but carries no
      // direction; every directional term below is therefore skipped.
      rho_i += cubicKernel(0.0);
      if (!finiteScalar(rho_i)) {
        return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
      }
      continue;
    }
    const double kernel_argument = distance / parameters_.h;
    if (!finiteScalar(kernel_argument)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    const double kernel_value = cubicKernel(kernel_argument);
    if (!finiteScalar(kernel_value)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    rho_i += kernel_value;
    if (!finiteScalar(rho_i)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }

    const Eigen::Vector2d direction_ij = displacement / distance;
    if (!finiteVector(direction_ij)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    // s_i = sum_j phi(q_ij) n_ij, phi = -dWbar/dq >= 0.
    const double phi = cubicKernelNegDerivative(kernel_argument);
    if (!finiteScalar(phi)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    density_direction += phi * direction_ij;
    if (!finiteVector(density_direction)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }

    // Bounded short-range repulsion: [1 - d/d_rep]_+^2, zero from d_rep
    // outward so the nominal configuration is unaffected.
    const double shortfall = 1.0 - distance / parameters_.d_rep;
    if (shortfall > 0.0) {
      repulsion_sum += (shortfall * shortfall) * direction_ij;
      if (!finiteVector(repulsion_sum)) {
        return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
      }
    }

    // Radial relative damping, weighted by the normalized kernel value.
    const double radial_relative_speed =
        (neighbor->velocity - self.velocity).dot(direction_ij);
    if (!finiteScalar(radial_relative_speed)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    damping_sum += (kernel_value * radial_relative_speed) * direction_ij;
    if (!finiteVector(damping_sum)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
  }

  if (!(rho_i > 0.0) || !finiteScalar(rho_i)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }

  // Paper definition: rho_0 = 1 + sum_{j in N_i^*} Wbar(d*_ij/h), i.e. the
  // density UAV i would have in the nominal formation.  The scenario generator
  // passes that per-agent value in `reference_density`.  Without it we fall
  // back to placing every visible neighbour at the nominal spacing, which is
  // exact only when the visible neighbours really do sit at d_star.
  const double rho_reference =
      parameters_.reference_density > 0.0
          ? parameters_.reference_density
          : (1.0 + static_cast<double>(support_neighbor_count) * w_reference);
  if (!finiteScalar(rho_reference) || !(rho_reference > 0.0)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  const double density_ratio = rho_i / rho_reference;
  if (!finiteScalar(density_ratio) || density_ratio < 0.0) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  const double density_power = std::pow(density_ratio, parameters_.gamma);
  const double eta_i = density_power - 1.0;
  if (!finiteScalar(density_power) || !finiteScalar(eta_i)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }

  const double inverse_rho = 1.0 / rho_i;
  if (!finiteScalar(inverse_rho)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  const double eta_plus = std::max(eta_i, 0.0);
  const double eta_minus = std::max(-eta_i, 0.0);
  // g_den,rep = -k_rho * eta^+ / rho_i * s_i
  // g_den,att = +k_rho * eta^- / rho_i * s_i
  const double attractive_scale = parameters_.k_rho * eta_minus * inverse_rho;
  const double repulsive_scale = -parameters_.k_rho * eta_plus * inverse_rho;
  if (!finiteScalar(attractive_scale) || !finiteScalar(repulsive_scale)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  const Eigen::Vector2d density_attractive =
      attractive_scale * density_direction;
  const Eigen::Vector2d density_repulsive =
      repulsive_scale * density_direction;
  const Eigen::Vector2d explicit_repulsive =
      -parameters_.k_rep * repulsion_sum;
  const Eigen::Vector2d damping =
      (parameters_.k_damp * inverse_rho) * damping_sum;
  if (!finiteVector(density_attractive) || !finiteVector(density_repulsive) ||
      !finiteVector(explicit_repulsive) || !finiteVector(damping)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }

  const Eigen::Vector2d beta_attractive = beta_i * density_attractive;
  if (!finiteVector(beta_attractive)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  Eigen::Vector2d raw = density_repulsive + explicit_repulsive;
  if (!finiteVector(raw)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  raw += beta_attractive;
  if (!finiteVector(raw)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  raw += damping;
  if (!finiteVector(raw)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  const double raw_norm = raw.norm();
  if (!finiteScalar(raw_norm)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }

  Eigen::Vector2d g_coord = raw;
  bool output_saturated = false;
  if (raw_norm > parameters_.g_max) {
    const double saturation_scale = parameters_.g_max / raw_norm;
    if (!finiteScalar(saturation_scale)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    g_coord = saturation_scale * raw;
    output_saturated = true;
  }
  if (!finiteVector(g_coord)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }

  SphIntentOutput output;
  output.g_density_attractive = density_attractive;
  output.g_density_repulsive = density_repulsive;
  output.g_explicit_repulsive = explicit_repulsive;
  output.g_damping = damping;
  output.g_coord = g_coord;
  output.rho_i = rho_i;
  output.rho_reference = rho_reference;
  output.eta_i = eta_i;
  output.beta_used = beta_i;
  output.support_neighbor_count = support_neighbor_count;
  output.ignored_nonfresh_count = ignored_nonfresh_count;
  output.invalid_input_count = 0;
  output.near_zero_count = near_zero_count;
  output.output_saturated = output_saturated;
  output.valid = true;
  output.status = SphIntentStatus::VALID;

  if (!finiteVector(output.g_density_attractive) ||
      !finiteVector(output.g_density_repulsive) ||
      !finiteVector(output.g_explicit_repulsive) ||
      !finiteVector(output.g_damping) || !finiteVector(output.g_coord) ||
      !finiteScalar(output.rho_i) || !finiteScalar(output.rho_reference) ||
      !finiteScalar(output.eta_i) || !finiteScalar(output.beta_used)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  return output;
}

}  // namespace phase_offset_swarm
