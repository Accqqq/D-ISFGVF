#pragma once

#include <Eigen/StdVector>

#include <functional>
#include <map>
#include <mutex>
#include <set>

#include "phase_offset_swarm/agent_state.h"
#include "phase_offset_swarm/neighbor_state.h"
#include "phase_offset_swarm/swarm_parameters.h"

namespace phase_offset_swarm {

class NeighborManager {
 public:
  NeighborManager(int self_id, const SwarmParameters& parameters);

  bool update(const SwarmAgentState& received);

  NeighborSnapshot snapshot(const SwarmAgentState& self, double now);

 private:
  int self_id_;
  SwarmParameters parameters_;
  mutable std::mutex mutex_;
  using StateMap = std::map<
      int, SwarmAgentState, std::less<int>,
      Eigen::aligned_allocator<std::pair<const int, SwarmAgentState>>>;
  StateMap cache_;
  std::set<int> organization_membership_;
  std::set<int> conflict_distance_membership_;
};

}  // namespace phase_offset_swarm
