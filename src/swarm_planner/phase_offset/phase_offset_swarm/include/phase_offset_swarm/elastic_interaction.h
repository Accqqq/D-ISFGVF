#pragma once

#include <Eigen/Core>

#include "phase_offset_swarm/agent_state.h"
#include "phase_offset_swarm/neighbor_state.h"
#include "phase_offset_swarm/swarm_parameters.h"

namespace phase_offset_swarm {

class ElasticInteraction {
 public:
  explicit ElasticInteraction(const SwarmParameters& parameters);

  Eigen::Vector2d computeSeparation(const SwarmAgentState& self,
                                    const NeighborSnapshot& neighbors) const;

  Eigen::Vector2d computeCohesionUnscaled(
      const SwarmAgentState& self, const NeighborSnapshot& neighbors) const;

  static double smootherstep(double s);
  static double cohesionBump(double s);

 private:
  SwarmParameters parameters_;
};

}  // namespace phase_offset_swarm
