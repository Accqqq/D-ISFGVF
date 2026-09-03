#pragma once

#include <Eigen/StdVector>

#include <cmath>
#include <cstddef>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>

#include "phase_offset_swarm/agent_state.h"
#include "phase_offset_swarm/neighbor_state.h"
#include "phase_offset_swarm/swarm_parameters.h"

namespace phase_offset_swarm {

// Strict SIM-C data-plane configuration.  The legacy SwarmParameters-backed
// constructor remains available for the pre-existing planar SPH/shadow
// prototype.
struct NeighborManagerConfig {
  int self_id = 0;
  int agent_count = 1;
  std::string world_frame = "world";
  double fresh_timeout = 0.30;
  double lost_timeout = 0.60;
  double retention_timeout = 1.50;
  double prediction_horizon_max = 0.30;
  double future_timestamp_tolerance = 0.02;
  double source_too_old_timeout = 0.60;
  double enter_radius = 2.45;
  double exit_radius = 2.50;
  std::size_t max_neighbors = 0;

  bool validate(std::string* error = nullptr) const {
    const auto fail = [error](const std::string& message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };
    const auto finite = [](double value) { return std::isfinite(value); };
    if (agent_count <= 0) {
      return fail("agent_count must be positive");
    }
    if (self_id < 0 || self_id >= agent_count) {
      return fail("self_id must be in the configured agent domain");
    }
    if (world_frame.empty()) {
      return fail("world_frame must be non-empty");
    }
    if (!finite(fresh_timeout) || !finite(lost_timeout) ||
        !finite(retention_timeout) ||
        !(0.0 < fresh_timeout && fresh_timeout < lost_timeout &&
          lost_timeout < retention_timeout)) {
      return fail("require 0 < fresh_timeout < lost_timeout < retention");
    }
    if (!finite(prediction_horizon_max) || prediction_horizon_max < 0.0 ||
        prediction_horizon_max > lost_timeout) {
      return fail("prediction_horizon_max must be in [0, lost_timeout]");
    }
    if (!finite(future_timestamp_tolerance) ||
        future_timestamp_tolerance < 0.0) {
      return fail("future_timestamp_tolerance must be non-negative");
    }
    if (!finite(source_too_old_timeout) || source_too_old_timeout < 0.0) {
      return fail("source_too_old_timeout must be non-negative");
    }
    if (!finite(enter_radius) || !finite(exit_radius) ||
        !(0.0 < enter_radius && enter_radius < exit_radius)) {
      return fail("require 0 < enter_radius < exit_radius");
    }
    return true;
  }

  void validateOrThrow() const {
    std::string error;
    if (!validate(&error)) {
      throw std::invalid_argument("invalid NeighborManagerConfig: " + error);
    }
  }
};

using NeighborManagerConfiguration = NeighborManagerConfig;

enum class NeighborUpdateReason {
  INVALID_ID,
  SELF_FILTERED,
  NONFINITE_STATE,
  FRAME_MISMATCH,
  INVALID_TIMESTAMP,
  FUTURE_TIMESTAMP,
  SOURCE_TOO_OLD,
  DUPLICATE,
  OUT_OF_ORDER,
  ACCEPTED,
  // Readable aliases for clients that use the C++ naming convention.
  kInvalidId = INVALID_ID,
  kSelfFiltered = SELF_FILTERED,
  kNonfiniteState = NONFINITE_STATE,
  kFrameMismatch = FRAME_MISMATCH,
  kInvalidTimestamp = INVALID_TIMESTAMP,
  kFutureTimestamp = FUTURE_TIMESTAMP,
  kSourceTooOld = SOURCE_TOO_OLD,
  kDuplicate = DUPLICATE,
  kOutOfOrder = OUT_OF_ORDER,
  kAccepted = ACCEPTED
};

using UpdateReason = NeighborUpdateReason;

const char* neighborUpdateReasonName(NeighborUpdateReason reason);

class NeighborManager {
 public:
  using Config = NeighborManagerConfig;
  using UpdateResult = NeighborUpdateReason;
  using UpdateOutcome = NeighborUpdateReason;
  // Strict SIM-C constructor.
  explicit NeighborManager(const NeighborManagerConfig& config);

  // Compatibility constructor for the existing planar SPH/shadow code.  It
  // retains the previous acceptance semantics and does not impose an agent
  // count because that prototype predates the configured generic-N domain.
  NeighborManager(int self_id, const SwarmParameters& parameters);

  // Convenience strict constructor with frozen defaults.
  NeighborManager(int self_id, int agent_count,
                  const std::string& world_frame = "world");

  // One-argument update is retained for the old ROS-free API.  Strict callers
  // should use the two-clock overload so source and receive time remain
  // independent.
  bool update(const SwarmAgentState& received);
  bool update(const SwarmAgentState& received, double receive_time,
              double source_now);

  NeighborUpdateReason updateWithReason(const SwarmAgentState& received,
                                        double receive_time,
                                        double source_now);
  NeighborUpdateReason updateDetailed(const SwarmAgentState& received,
                                      double receive_time,
                                      double source_now) {
    return updateWithReason(received, receive_time, source_now);
  }

  // The one-clock form is source-compatible with the original core.  The
  // strict form receives independently injected source and steady times.
  NeighborSnapshot snapshot(const SwarmAgentState& self, double now);
  NeighborSnapshot snapshot(const SwarmAgentState& self,
                            double source_query_time,
                            double receive_query_time);

  NeighborUpdateCounters counters() const;
  std::size_t cacheSize() const;
  const NeighborManagerConfig& config() const { return config_; }
  int selfId() const { return self_id_; }

 private:
  struct CacheRecord {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SwarmAgentState state;
    double receive_time = 0.0;
  };

  using StateMap = std::map<
      int, CacheRecord, std::less<int>,
      Eigen::aligned_allocator<std::pair<const int, CacheRecord>>>;

  NeighborManager(int self_id, const SwarmParameters& parameters,
                  const NeighborManagerConfig& config, bool strict_mode);

  NeighborUpdateReason reject(NeighborUpdateReason reason);
  bool stateMatchesWorld(const SwarmAgentState& state) const;
  NeighborSnapshot snapshotImpl(const SwarmAgentState& self,
                                double source_query_time,
                                double receive_query_time);

  int self_id_ = -1;
  SwarmParameters parameters_;
  NeighborManagerConfig config_;
  bool strict_mode_ = true;
  mutable std::mutex mutex_;
  StateMap cache_;
  std::set<int> active_membership_;
  std::set<int> organization_membership_;
  std::set<int> conflict_distance_membership_;
  NeighborUpdateCounters counters_;
};

}  // namespace phase_offset_swarm
