#include "phase_offset_swarm/sph_intent.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

namespace phase_offset_swarm {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

// Clean-room horizontal convention: the paper's cubic shape is normalized in
// two dimensions with C2 = 10*c/(7*pi*h^2).  The derivative below is the
// floating-point mathematical derivative of that shape.
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

  if (!std::isfinite(normalization_c) || !std::isfinite(h) ||
      !std::isfinite(mass) || !std::isfinite(rest_density) ||
      !std::isfinite(k_den) || !std::isfinite(k_rep) ||
      !std::isfinite(k_vis) || !std::isfinite(g_max) ||
      !std::isfinite(distance_epsilon)) {
    return fail("all floating parameters must be finite");
  }
  if (normalization_c != 54.97) {
    return fail("normalization_c must equal 54.97");
  }
  if (!(h > 0.0)) {
    return fail("h must be strictly positive");
  }
  if (!(mass > 0.0)) {
    return fail("mass must be strictly positive");
  }
  if (!(rest_density > 0.0)) {
    return fail("rest_density must be strictly positive");
  }
  if (k_den < 0.0 || k_rep < 0.0 || k_vis < 0.0) {
    return fail("density, repulsion, and viscosity gains must be nonnegative");
  }
  if (gamma != 7) {
    return fail("gamma must equal 7");
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
  const double h2 = parameters_.h * parameters_.h;
  const double normalization_numerator = 10.0 * parameters_.normalization_c;
  const double normalization_denominator = 7.0 * kPi * h2;
  if (!finiteScalar(support_limit) || !finiteScalar(h2) ||
      !finiteScalar(normalization_numerator) ||
      !finiteScalar(normalization_denominator) ||
      !(normalization_denominator > 0.0)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  const double c2 = normalization_numerator / normalization_denominator;
  if (!finiteScalar(c2)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }

  double rho_i = parameters_.mass * c2;
  if (!finiteScalar(rho_i)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  Eigen::Vector2d explicit_repulsive = Eigen::Vector2d::Zero();
  int support_neighbor_count = 0;
  int near_zero_count = 0;

  for (const NeighborState* neighbor : fresh_neighbors) {
    const Eigen::Vector2d displacement = self.position - neighbor->position;
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
    }
    const double kernel_argument = near_zero ? 0.0 : distance / parameters_.h;
    if (!finiteScalar(kernel_argument)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    const double kernel_value = cubicKernel(kernel_argument);
    const double density_term = parameters_.mass * c2 * kernel_value;
    if (!finiteScalar(kernel_value) || !finiteScalar(density_term)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    rho_i += density_term;
    if (!finiteScalar(rho_i)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    if (near_zero) {
      continue;
    }

    const Eigen::Vector2d direction = displacement / distance;
    if (!finiteVector(direction)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    const double derivative = cubicKernelDerivative(kernel_argument);
    const double gradient_scale = (c2 / parameters_.h) * derivative;
    const double repulsion_scale = parameters_.k_rep / distance;
    if (!finiteScalar(derivative) || !finiteScalar(gradient_scale) ||
        !finiteScalar(repulsion_scale)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    const Eigen::Vector2d gradient_term = gradient_scale * direction;
    const Eigen::Vector2d repulsion_term = repulsion_scale * direction;
    if (!finiteVector(gradient_term) || !finiteVector(repulsion_term)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
    gradient += gradient_term;
    explicit_repulsive += repulsion_term;
    if (!finiteVector(gradient) || !finiteVector(explicit_repulsive)) {
      return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
    }
  }

  if (!(rho_i > 0.0) || !finiteScalar(rho_i)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  const double density_ratio = rho_i / parameters_.rest_density;
  if (!finiteScalar(density_ratio) || density_ratio < 0.0) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  const double density_power = std::pow(density_ratio, parameters_.gamma);
  const double a_i = density_power - 1.0;
  if (!finiteScalar(density_power) || !finiteScalar(a_i)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  const double pressure_scale = parameters_.k_den / rho_i;
  const double pressure_i = -pressure_scale * a_i;
  if (!finiteScalar(pressure_scale) || !finiteScalar(pressure_i)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }

  // With self-minus-neighbor displacement, the signed split keeps the
  // low-density branch inward and the high-density branch outward.
  const double a_minus = std::min(a_i, 0.0);
  const double a_plus = std::max(a_i, 0.0);
  const double attractive_scale = -pressure_scale * a_minus;
  const double repulsive_scale = -pressure_scale * a_plus;
  if (!finiteScalar(attractive_scale) || !finiteScalar(repulsive_scale)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  const Eigen::Vector2d density_attractive = attractive_scale * gradient;
  const Eigen::Vector2d density_repulsive = repulsive_scale * gradient;
  const Eigen::Vector2d viscosity = -parameters_.k_vis * self.velocity;
  if (!finiteVector(density_attractive) || !finiteVector(density_repulsive) ||
      !finiteVector(viscosity)) {
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
  raw += viscosity;
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
  output.g_viscosity = viscosity;
  output.g_coord = g_coord;
  output.rho_i = rho_i;
  output.a_i = a_i;
  output.pressure_i = pressure_i;
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
      !finiteVector(output.g_viscosity) || !finiteVector(output.g_coord) ||
      !finiteScalar(output.rho_i) || !finiteScalar(output.a_i) ||
      !finiteScalar(output.pressure_i) || !finiteScalar(output.beta_used)) {
    return invalidOutput(SphIntentStatus::NUMERIC_FAILURE);
  }
  return output;
}

}  // namespace phase_offset_swarm
