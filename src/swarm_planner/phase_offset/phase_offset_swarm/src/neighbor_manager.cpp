#include "phase_offset_swarm/neighbor_manager.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "phase_offset_swarm/pair_geometry.h"

namespace phase_offset_swarm {
namespace {

NeighborManagerConfig strictConfig(int self_id, int agent_count,
                                    const std::string& world_frame) {
  NeighborManagerConfig config;
  config.self_id = self_id;
  config.agent_count = agent_count;
  config.world_frame = world_frame;
  return config;
}

NeighborManagerConfig legacyConfig(int self_id,
                                   const SwarmParameters& parameters) {
  NeighborManagerConfig config;
  config.self_id = self_id;
  config.agent_count = std::numeric_limits<int>::max();
  config.world_frame.clear();
  config.fresh_timeout = parameters.fresh_timeout;
  config.lost_timeout = parameters.stale_timeout;
  config.retention_timeout = parameters.lost_retention_timeout;
  config.prediction_horizon_max = parameters.stale_timeout;
  config.future_timestamp_tolerance = parameters.future_stamp_tolerance;
  config.source_too_old_timeout = parameters.stale_timeout;
  config.enter_radius = std::max(
      0.0, parameters.r_comm - parameters.neighbor_hysteresis);
  config.exit_radius = parameters.r_comm;
  return config;
}

double nonnegativeAge(double query, double event) {
  if (!std::isfinite(query) || !std::isfinite(event)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, query - event);
}

}  // namespace

const char* neighborUpdateReasonName(NeighborUpdateReason reason) {
  switch (reason) {
    case NeighborUpdateReason::INVALID_ID:
      return "INVALID_ID";
    case NeighborUpdateReason::SELF_FILTERED:
      return "SELF_FILTERED";
    case NeighborUpdateReason::NONFINITE_STATE:
      return "NONFINITE_STATE";
    case NeighborUpdateReason::FRAME_MISMATCH:
      return "FRAME_MISMATCH";
    case NeighborUpdateReason::INVALID_TIMESTAMP:
      return "INVALID_TIMESTAMP";
    case NeighborUpdateReason::FUTURE_TIMESTAMP:
      return "FUTURE_TIMESTAMP";
    case NeighborUpdateReason::SOURCE_TOO_OLD:
      return "SOURCE_TOO_OLD";
    case NeighborUpdateReason::DUPLICATE:
      return "DUPLICATE";
    case NeighborUpdateReason::OUT_OF_ORDER:
      return "OUT_OF_ORDER";
    case NeighborUpdateReason::ACCEPTED:
      return "ACCEPTED";
  }
  return "UNKNOWN";
}

NeighborManager::NeighborManager(const NeighborManagerConfig& config)
    : self_id_(config.self_id), config_(config), strict_mode_(true) {
  config_.validateOrThrow();
  parameters_.fresh_timeout = config_.fresh_timeout;
  parameters_.stale_timeout = config_.lost_timeout;
  parameters_.lost_retention_timeout = config_.retention_timeout;
  parameters_.r_comm = config_.exit_radius;
  parameters_.neighbor_hysteresis =
      std::max(0.0, config_.exit_radius - config_.enter_radius);
}

NeighborManager::NeighborManager(int self_id, int agent_count,
                                 const std::string& world_frame)
    : NeighborManager(strictConfig(self_id, agent_count, world_frame)) {}

NeighborManager::NeighborManager(int self_id,
                                 const SwarmParameters& parameters)
    : NeighborManager(self_id, parameters, legacyConfig(self_id, parameters),
                      false) {}

NeighborManager::NeighborManager(int self_id,
                                 const SwarmParameters& parameters,
                                 const NeighborManagerConfig& config,
                                 bool strict_mode)
    : self_id_(self_id),
      parameters_(parameters),
      config_(config),
      strict_mode_(strict_mode) {
  parameters_.validateOrThrow();
  if (strict_mode_) {
    config_.validateOrThrow();
    if (config_.self_id != self_id_) {
      throw std::invalid_argument("config.self_id does not match self_id");
    }
  } else if (self_id_ < 0) {
    throw std::invalid_argument("self_id must be non-negative");
  }
}

NeighborUpdateReason NeighborManager::reject(NeighborUpdateReason reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  switch (reason) {
    case NeighborUpdateReason::INVALID_ID:
      ++counters_.invalid_id;
      break;
    case NeighborUpdateReason::SELF_FILTERED:
      ++counters_.self_filtered;
      break;
    case NeighborUpdateReason::NONFINITE_STATE:
      ++counters_.nonfinite_state;
      break;
    case NeighborUpdateReason::FRAME_MISMATCH:
      ++counters_.frame_mismatch;
      break;
    case NeighborUpdateReason::INVALID_TIMESTAMP:
      ++counters_.invalid_timestamp;
      break;
    case NeighborUpdateReason::FUTURE_TIMESTAMP:
      ++counters_.future_timestamp;
      break;
    case NeighborUpdateReason::SOURCE_TOO_OLD:
      ++counters_.source_too_old;
      break;
    case NeighborUpdateReason::DUPLICATE:
      ++counters_.duplicate;
      break;
    case NeighborUpdateReason::OUT_OF_ORDER:
      ++counters_.out_of_order;
      break;
    case NeighborUpdateReason::ACCEPTED:
      ++counters_.accepted;
      break;
  }
  return reason;
}

bool NeighborManager::stateMatchesWorld(const SwarmAgentState& state) const {
  return strict_mode_ && !config_.world_frame.empty() &&
         state.frame_id == config_.world_frame;
}

NeighborUpdateReason NeighborManager::updateWithReason(
    const SwarmAgentState& received, double receive_time, double source_now) {
  if (strict_mode_) {
    if (received.id < 0 || received.id >= config_.agent_count) {
      return reject(NeighborUpdateReason::INVALID_ID);
    }
    if (received.id == self_id_) {
      return reject(NeighborUpdateReason::SELF_FILTERED);
    }
    if (!received.isFinite() || !std::isfinite(receive_time) ||
        !std::isfinite(source_now)) {
      return reject(NeighborUpdateReason::NONFINITE_STATE);
    }
    if (!stateMatchesWorld(received)) {
      return reject(NeighborUpdateReason::FRAME_MISMATCH);
    }
    if (received.stamp <= 0.0) {
      return reject(NeighborUpdateReason::INVALID_TIMESTAMP);
    }
    if (received.stamp - source_now > config_.future_timestamp_tolerance) {
      return reject(NeighborUpdateReason::FUTURE_TIMESTAMP);
    }
    if (source_now - received.stamp > config_.source_too_old_timeout) {
      return reject(NeighborUpdateReason::SOURCE_TOO_OLD);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto existing = cache_.find(received.id);
    if (existing != cache_.end()) {
      if (received.stamp == existing->second.state.stamp) {
        ++counters_.duplicate;
        return NeighborUpdateReason::DUPLICATE;
      }
      if (received.stamp < existing->second.state.stamp) {
        ++counters_.out_of_order;
        return NeighborUpdateReason::OUT_OF_ORDER;
      }
    }
    CacheRecord record;
    record.state = received;
    record.receive_time = receive_time;
    cache_[received.id] = record;
    ++counters_.accepted;
    return NeighborUpdateReason::ACCEPTED;
  }

  // The old prototype has no independent receive clock or world-frame
  // requirement.  Keep its established bool semantics for protected tests.
  if (received.id < 0) {
    return reject(NeighborUpdateReason::INVALID_ID);
  }
  if (received.id == self_id_) {
    return reject(NeighborUpdateReason::SELF_FILTERED);
  }
  if (!received.isValid()) {
    return reject(NeighborUpdateReason::NONFINITE_STATE);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto existing = cache_.find(received.id);
  if (existing != cache_.end() &&
      received.stamp <= existing->second.state.stamp) {
    const NeighborUpdateReason reason =
        received.stamp == existing->second.state.stamp
            ? NeighborUpdateReason::DUPLICATE
            : NeighborUpdateReason::OUT_OF_ORDER;
    if (reason == NeighborUpdateReason::DUPLICATE) {
      ++counters_.duplicate;
    } else {
      ++counters_.out_of_order;
    }
    return reason;
  }
  CacheRecord record;
  record.state = received;
  record.receive_time = received.stamp;
  cache_[received.id] = record;
  ++counters_.accepted;
  return NeighborUpdateReason::ACCEPTED;
}

bool NeighborManager::update(const SwarmAgentState& received) {
  return updateWithReason(received, received.stamp, received.stamp) ==
         NeighborUpdateReason::ACCEPTED;
}

bool NeighborManager::update(const SwarmAgentState& received,
                             double receive_time, double source_now) {
  return updateWithReason(received, receive_time, source_now) ==
         NeighborUpdateReason::ACCEPTED;
}

NeighborSnapshot NeighborManager::snapshot(const SwarmAgentState& self,
                                           double now) {
  return snapshotImpl(self, now, now);
}

NeighborSnapshot NeighborManager::snapshot(const SwarmAgentState& self,
                                           double source_query_time,
                                           double receive_query_time) {
  return snapshotImpl(self, source_query_time, receive_query_time);
}

NeighborSnapshot NeighborManager::snapshotImpl(
    const SwarmAgentState& self, double source_query_time,
    double receive_query_time) {
  NeighborSnapshot result;
  result.self_id = self_id_;
  result.source_query_time = source_query_time;
  result.receive_query_time = receive_query_time;
  result.query_source_time = source_query_time;
  result.query_receive_time = receive_query_time;
  if (!self.isValid() || !std::isfinite(source_query_time) ||
      !std::isfinite(receive_query_time) ||
      (strict_mode_ && !stateMatchesWorld(self))) {
    result.counters = counters();
    result.update_counters = result.counters;
    return result;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = cache_.begin(); it != cache_.end();) {
    const SwarmAgentState& source = it->second.state;
    const double source_age = nonnegativeAge(source_query_time, source.stamp);
    const double receive_age =
        nonnegativeAge(receive_query_time, it->second.receive_time);
    const double retention_age = strict_mode_ ? receive_age : source_age;
    if (retention_age > config_.retention_timeout) {
      active_membership_.erase(source.id);
      organization_membership_.erase(source.id);
      conflict_distance_membership_.erase(source.id);
      it = cache_.erase(it);
      continue;
    }

    NeighborState neighbor;
    neighbor.id = source.id;
    neighbor.position = source.position;
    neighbor.velocity = source.velocity;
    neighbor.predicted_position = source.position;
    neighbor.position_z = source.position_z;
    neighbor.velocity_z = source.velocity_z;
    neighbor.predicted_position_z = source.position_z;
    neighbor.frame_id = source.frame_id;
    neighbor.source_stamp = source.stamp;
    neighbor.source_age = source_age;
    neighbor.receive_age = receive_age;
    neighbor.message_age = strict_mode_ ? receive_age : source_age;
    if (strict_mode_) {
      neighbor.prediction_dt = std::min(
          source_age, std::max(0.0, config_.prediction_horizon_max));
    } else {
      neighbor.prediction_dt = neighbor.message_age;
    }
    neighbor.predicted_position =
        source.position + source.velocity * neighbor.prediction_dt;
    neighbor.predicted_position_z =
        source.position_z + source.velocity_z * neighbor.prediction_dt;

    if (strict_mode_) {
      if (receive_age <= config_.fresh_timeout) {
        neighbor.freshness = NeighborFreshness::FRESH;
      } else if (receive_age <= config_.lost_timeout) {
        neighbor.freshness = NeighborFreshness::STALE;
      } else {
        neighbor.freshness = NeighborFreshness::LOST;
      }
    } else {
      const double tau = source_query_time - source.stamp;
      if (tau < -config_.future_timestamp_tolerance) {
        neighbor.freshness = NeighborFreshness::LOST;
      } else if (neighbor.message_age <= config_.fresh_timeout) {
        neighbor.freshness = NeighborFreshness::FRESH;
      } else if (neighbor.message_age <= config_.lost_timeout) {
        neighbor.freshness = NeighborFreshness::STALE;
      } else {
        neighbor.freshness = NeighborFreshness::LOST;
      }
    }

    result.all.push_back(neighbor);
    if (neighbor.freshness == NeighborFreshness::FRESH) {
      ++result.fresh_count;
    } else if (neighbor.freshness == NeighborFreshness::STALE) {
      ++result.stale_count;
    } else {
      ++result.lost_count;
    }

    if (strict_mode_) {
      const bool usable = neighbor.freshness == NeighborFreshness::FRESH ||
                          neighbor.freshness == NeighborFreshness::STALE;
      if (!usable) {
        active_membership_.erase(neighbor.id);
      } else {
        const Eigen::Vector3d displacement =
            neighbor.predictedPositionWorld() - self.positionWorld();
        const double distance = displacement.norm();
        const bool was_active = active_membership_.count(neighbor.id) != 0;
        const bool selected =
            std::isfinite(distance) &&
            (was_active ? distance <= config_.exit_radius
                        : distance <= config_.enter_radius);
        if (selected) {
          active_membership_.insert(neighbor.id);
          result.active.push_back(neighbor);
          result.active_neighbors.push_back(neighbor);
        } else {
          active_membership_.erase(neighbor.id);
        }
      }
      ++it;
      continue;
    }

    // Existing SPH organization/conflict/safety vectors remain planar and
    // retain their original FRESH-only behavior.
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
  result.cache_record_count = cache_.size();
  result.counters = counters_;
  result.update_counters = result.counters;
  return result;
}

NeighborUpdateCounters NeighborManager::counters() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return counters_;
}

std::size_t NeighborManager::cacheSize() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cache_.size();
}

}  // namespace phase_offset_swarm
