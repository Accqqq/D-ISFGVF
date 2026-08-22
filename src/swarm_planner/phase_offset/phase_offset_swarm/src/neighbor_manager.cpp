#include "phase_offset_swarm/neighbor_manager.h"

#include <algorithm>
#include <cmath>

#include "phase_offset_swarm/pair_geometry.h"

namespace phase_offset_swarm {

NeighborManager::NeighborManager(int self_id,
                                 const SwarmParameters& parameters)
    : self_id_(self_id), parameters_(parameters) {
  parameters_.validateOrThrow();
  if (self_id_ < 0) {
    throw std::invalid_argument("self_id must be non-negative");
  }
}

bool NeighborManager::update(const SwarmAgentState& received) {
  if (!received.isValid() || received.id == self_id_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto existing = cache_.find(received.id);
  if (existing != cache_.end() && received.stamp <= existing->second.stamp) {
    return false;
  }
  cache_[received.id] = received;
  return true;
}

NeighborSnapshot NeighborManager::snapshot(const SwarmAgentState& self,
                                           double now) {
  NeighborSnapshot result;
  if (!self.isValid() || !std::isfinite(now)) {
    return result;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = cache_.begin(); it != cache_.end();) {
    const SwarmAgentState& source = it->second;
    const double tau = now - source.stamp;
    if (tau > parameters_.lost_retention_timeout) {
      it = cache_.erase(it);
      organization_membership_.erase(source.id);
      conflict_distance_membership_.erase(source.id);
      continue;
    }

    NeighborState neighbor;
    neighbor.id = source.id;
    neighbor.position = source.position;
    neighbor.velocity = source.velocity;
    neighbor.source_stamp = source.stamp;
    neighbor.message_age = std::max(0.0, tau);
    neighbor.predicted_position =
        source.position + source.velocity * neighbor.message_age;

    // A packet from materially in the future cannot be used for a selected
    // set; retain it so that the same cache entry can become usable when time
    // catches up.
    const bool future_outside_tolerance =
        tau < -parameters_.future_stamp_tolerance;
    if (future_outside_tolerance) {
      neighbor.freshness = NeighborFreshness::LOST;
    } else if (neighbor.message_age <= parameters_.fresh_timeout) {
      neighbor.freshness = NeighborFreshness::FRESH;
    } else if (neighbor.message_age <= parameters_.stale_timeout) {
      neighbor.freshness = NeighborFreshness::STALE;
    } else {
      neighbor.freshness = NeighborFreshness::LOST;
    }
    result.all.push_back(neighbor);
    if (neighbor.freshness == NeighborFreshness::FRESH) {
      ++result.fresh_count;
    } else if (neighbor.freshness == NeighborFreshness::STALE) {
      ++result.stale_count;
    } else {
      ++result.lost_count;
    }

    if (neighbor.freshness == NeighborFreshness::FRESH) {
      const PairGeometry geometry = computePairGeometry(
          self, neighbor, parameters_.distance_epsilon, parameters_.d_safe,
          parameters_.ttc_epsilon);
      const double d = geometry.distance;
      const double enter_comm =
          std::max(0.0, parameters_.r_comm - parameters_.neighbor_hysteresis);
      const bool was_org = organization_membership_.count(neighbor.id) != 0;
      const bool org_selected =
          was_org ? d <= parameters_.r_comm : d <= enter_comm;
      if (org_selected) {
        organization_membership_.insert(neighbor.id);
        result.organization.push_back(neighbor);
      } else if (was_org && d > parameters_.r_comm) {
        organization_membership_.erase(neighbor.id);
      }

      const double enter_conf =
          std::max(0.0, parameters_.r_conf - parameters_.neighbor_hysteresis);
      const bool was_conf =
          conflict_distance_membership_.count(neighbor.id) != 0;
      const bool in_conf_distance =
          was_conf ? d <= parameters_.r_conf : d <= enter_conf;
      if (in_conf_distance) {
        conflict_distance_membership_.insert(neighbor.id);
      } else if (was_conf && d > parameters_.r_conf) {
        conflict_distance_membership_.erase(neighbor.id);
      }

      const bool closing = geometry.closing_speed > parameters_.ttc_epsilon;
      const bool ttc_active = std::isfinite(geometry.ttc) &&
                              geometry.ttc <= parameters_.ttc_activation;
      if (closing && (in_conf_distance || ttc_active)) {
        result.conflict.push_back(neighbor);
      }

      const bool safety_selected =
          d <= parameters_.r_safe ||
          (std::isfinite(geometry.ttc) &&
           geometry.ttc <= parameters_.ttc_activation);
      if (safety_selected) {
        result.safety.push_back(neighbor);
      }
    }
    ++it;
  }
  return result;
}

}  // namespace phase_offset_swarm
