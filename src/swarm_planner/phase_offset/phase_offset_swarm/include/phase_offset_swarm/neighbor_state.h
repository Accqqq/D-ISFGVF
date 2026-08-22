#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <limits>
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
  double source_stamp = 0.0;
  double message_age = 0.0;
  NeighborFreshness freshness = NeighborFreshness::LOST;
};

using NeighborStateVector =
    std::vector<NeighborState, Eigen::aligned_allocator<NeighborState>>;

struct NeighborSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NeighborStateVector all;
  NeighborStateVector organization;
  NeighborStateVector conflict;
  NeighborStateVector safety;
  int fresh_count = 0;
  int stale_count = 0;
  int lost_count = 0;
};

}  // namespace phase_offset_swarm
