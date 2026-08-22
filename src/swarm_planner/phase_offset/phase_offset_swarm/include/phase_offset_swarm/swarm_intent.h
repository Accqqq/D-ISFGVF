#pragma once

#include "phase_offset_swarm/agent_state.h"
#include "phase_offset_swarm/neighbor_state.h"
#include "phase_offset_swarm/swarm_output.h"
#include "phase_offset_swarm/swarm_parameters.h"

#include "phase_offset_swarm/conflict_friction.h"
#include "phase_offset_swarm/elastic_interaction.h"

namespace phase_offset_swarm {

class SwarmIntentCalculator {
 public:
  explicit SwarmIntentCalculator(const SwarmParameters& parameters);

  SwarmOutput compute(const SwarmAgentState& self,
                      const NeighborSnapshot& neighbors,
                      double beta_preview) const;

 private:
  SwarmParameters parameters_;
  ElasticInteraction elastic_interaction_;
  ConflictFriction conflict_friction_;
};

}  // namespace phase_offset_swarm
