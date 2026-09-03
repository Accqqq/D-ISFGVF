#pragma once

#include <Eigen/Core>

#include <string>

#include "phase_offset_swarm/agent_state.h"
#include "phase_offset_swarm/neighbor_state.h"

namespace phase_offset_swarm {

struct SphParameters {
  double normalization_c = 54.97;
  double h = 10.0;
  double mass = 1.0;
  double rest_density = 1.0e6;
  int gamma = 7;
  double k_den = 1.0;
  double k_rep = 1.0;
  double k_vis = 1.5;
  double g_max = 1.5;
  double distance_epsilon = 1.0e-9;

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
  Eigen::Vector2d g_viscosity = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_coord = Eigen::Vector2d::Zero();
  double rho_i = 0.0;
  double a_i = 0.0;
  double pressure_i = 0.0;
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
