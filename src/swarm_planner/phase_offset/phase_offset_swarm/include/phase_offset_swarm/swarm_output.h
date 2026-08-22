#pragma once

#include <Eigen/Core>

#include <limits>

namespace phase_offset_swarm {

struct SwarmOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector2d g_coord = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_sep = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_coh = Eigen::Vector2d::Zero();
  Eigen::Vector2d g_conf = Eigen::Vector2d::Zero();
  double min_distance = std::numeric_limits<double>::infinity();
  double min_ttc = std::numeric_limits<double>::infinity();
  int num_org_neighbors = 0;
  int num_conflict_neighbors = 0;
  int num_safety_neighbors = 0;
  int num_fresh_neighbors = 0;
  int num_stale_neighbors = 0;
  int num_lost_neighbors = 0;
  bool output_saturated = false;
};

}  // namespace phase_offset_swarm
