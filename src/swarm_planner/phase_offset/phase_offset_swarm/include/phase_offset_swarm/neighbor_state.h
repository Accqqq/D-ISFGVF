#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <limits>
#include <cstdint>
#include <string>
#include <vector>

namespace phase_offset_swarm {

enum class NeighborFreshness {
  FRESH,
  STALE,
  LOST
};

struct NeighborState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int id = -1;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  Eigen::Vector2d predicted_position = Eigen::Vector2d::Zero();
  double position_z = 0.0;
  double velocity_z = 0.0;
  double predicted_position_z = 0.0;
  std::string frame_id;
  double source_stamp = 0.0;
  double source_age = 0.0;
  double receive_age = 0.0;
  double prediction_dt = 0.0;
  // Existing diagnostics use message_age as source age.  Keep it populated
  // for source compatibility while SIM-C exposes both independent ages.
  double message_age = 0.0;
  NeighborFreshness freshness = NeighborFreshness::LOST;

  Eigen::Vector3d positionWorld() const {
    return Eigen::Vector3d(position.x(), position.y(), position_z);
  }

  Eigen::Vector3d velocityWorld() const {
    return Eigen::Vector3d(velocity.x(), velocity.y(), velocity_z);
  }

  Eigen::Vector3d predictedPositionWorld() const {
    return Eigen::Vector3d(predicted_position.x(), predicted_position.y(),
                           predicted_position_z);
  }

  Eigen::Vector3d worldPosition() const { return positionWorld(); }
  Eigen::Vector3d worldVelocity() const { return velocityWorld(); }
  Eigen::Vector3d worldPredictedPosition() const {
    return predictedPositionWorld();
  }
  Eigen::Vector3d position3d() const { return positionWorld(); }
  Eigen::Vector3d velocity3d() const { return velocityWorld(); }
  Eigen::Vector3d predictedPosition3d() const {
    return predictedPositionWorld();
  }
};

using NeighborStateVector =
    std::vector<NeighborState, Eigen::aligned_allocator<NeighborState>>;

struct NeighborUpdateCounters {
  std::uint64_t invalid_id = 0;
  std::uint64_t self_filtered = 0;
  std::uint64_t nonfinite_state = 0;
  std::uint64_t frame_mismatch = 0;
  std::uint64_t invalid_timestamp = 0;
  std::uint64_t future_timestamp = 0;
  std::uint64_t source_too_old = 0;
  std::uint64_t duplicate = 0;
  std::uint64_t out_of_order = 0;
  std::uint64_t accepted = 0;

  std::uint64_t rejected() const {
    return invalid_id + self_filtered + nonfinite_state + frame_mismatch +
           invalid_timestamp + future_timestamp + source_too_old + duplicate +
           out_of_order;
  }
};

using UpdateCounters = NeighborUpdateCounters;

struct NeighborSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int self_id = -1;
  double source_query_time = 0.0;
  double receive_query_time = 0.0;
  // Explicit names for callers that distinguish the two time domains.
  double query_source_time = 0.0;
  double query_receive_time = 0.0;
  NeighborStateVector all;
  NeighborStateVector active;
  NeighborStateVector active_neighbors;
  NeighborStateVector organization;
  NeighborStateVector conflict;
  NeighborStateVector safety;
  int fresh_count = 0;
  int stale_count = 0;
  int lost_count = 0;
  std::size_t cache_record_count = 0;
  NeighborUpdateCounters counters;
  NeighborUpdateCounters update_counters;
};

}  // namespace phase_offset_swarm
