#ifndef BSPLINE_RACE_PHASE_OFFSET_TYPES_H
#define BSPLINE_RACE_PHASE_OFFSET_TYPES_H

#include <Eigen/Core>

#include <ros/time.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace FLAG_Race
{

enum class SwarmControlMode
{
  DISABLED = 0,
  ROLLING = 1,
  SAFETY_PRIORITY = 2,
  TERMINAL = 3,
  EMERGENCY = 4
};

struct PhaseOffsetGeometry
{
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d dp_dw = Eigen::Vector3d::Zero();
  Eigen::Vector3d d2p_dw2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d T = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d e_perp = Eigen::Vector3d::Zero();
  Eigen::Vector3d base_v = Eigen::Vector3d::Zero();
  double curvature = 0.0;
  double delta = 0.0;
  double e_parallel = 0.0;
  double rho = 0.0;
  double alpha = 0.0;
  double q = 0.0;
  double sigma = 0.0;
  double base_w_dot = 0.0;
  std::string invalid_reason;
  bool valid = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct TubeBounds
{
  double query_w = 0.0;
  double lower = 0.0;
  double upper = 0.0;
  double lower_dw = 0.0;
  double upper_dw = 0.0;
  double beta = 1.0;
  double min_regularity = 1.0;
  ros::Time stamp;
  uint32_t epoch = 0;
  bool valid = false;
};

struct NeighborState
{
  int robot_id = -1;
  ros::Time stamp;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  bool fresh = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PredictedNeighborState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int robot_id = -1;
  ros::Time source_stamp;
  double age = 0.0;
  Eigen::Vector3d predicted_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d advertised_velocity = Eigen::Vector3d::Zero();
  double velocity_uncertainty_bound = 0.0;
  bool safety_active = false;
};

struct NeighborSelection
{
  std::vector<NeighborState> organization;
  std::vector<PredictedNeighborState,
              Eigen::aligned_allocator<PredictedNeighborState>> safety;
  bool communication_fault = false;
};

struct NeighborEventState
{
  int robot_id = -1;
  int branch_event_id = -1;
  uint32_t branch_sequence = 0;
  ros::Time branch_valid_until;
  double channel_beta = 1.0;
  uint32_t conflict_sequence = 0;
  ros::Time conflict_valid_until;
  bool branch_active = false;
  bool conflict_active = false;
};

struct PairIntentContribution
{
  int robot_id = -1;
  double distance = 0.0;
  double distance_rate = 0.0;
  Eigen::Vector3d n_ij = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_damp = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_safe = Eigen::Vector3d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SwarmIntent
{
  Eigen::Vector3d g_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_damp = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_safe = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_swarm = Eigen::Vector3d::Zero();
  Eigen::Vector3d g_des = Eigen::Vector3d::Zero();
  double min_neighbor_distance = 1e9;
  int organization_neighbor_count = 0;
  int safety_neighbor_count = 0;
  std::vector<int> organization_neighbor_ids;
  std::vector<int> safety_neighbor_ids;
  std::vector<PairIntentContribution> pair_contributions;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PortCommand
{
  double u_w = 0.0;
  double u_delta = 0.0;
  double objective = 0.0;
  double allocation_residual = 0.0;
  bool feasible = false;
  SwarmControlMode mode = SwarmControlMode::DISABLED;
  std::vector<std::string> active_constraints;
};

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_PHASE_OFFSET_TYPES_H
