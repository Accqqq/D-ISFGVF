#pragma once

#include <Eigen/Core>

#include "phase_offset_swarm/agent_state.h"
#include "phase_offset_swarm/neighbor_state.h"
#include "phase_offset_swarm/swarm_parameters.h"

namespace phase_offset_swarm {

class ConflictFriction {
 public:
  explicit ConflictFriction(const SwarmParameters& parameters);

  Eigen::Vector2d compute(const SwarmAgentState& self,
                          const NeighborSnapshot& neighbors) const;

 private:
  SwarmParameters parameters_;
};

}  // namespace phase_offset_swarm
