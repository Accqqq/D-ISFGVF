#ifndef BSPLINE_RACE_SWARM_NEIGHBOR_MODEL_H
#define BSPLINE_RACE_SWARM_NEIGHBOR_MODEL_H

#include <Eigen/Core>

#include <ros/time.h>

#include <cstdint>
#include <map>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <common_msgs/SwarmConflictState.h>
#include <common_msgs/SwarmPathEvent.h>
#include <common_msgs/SwarmState.h>
#include <plan_env/sdf_map.h>

#include <bspline_race/phase_offset_types.h>

namespace FLAG_Race
{

struct NeighborSelectionParams
{
  double organization_enter_radius = 1.55;
  double organization_exit_radius = 1.65;
  double safety_radius = 1.40;
  double ttc_safe = 2.00;
  double max_message_age = 0.30;
  double neighbor_timeout = 0.30;
  double neighbor_retention_timeout = 1.00;
  double self_velocity_error_bound = 0.10;
  double neighbor_velocity_error_bound = 0.10;
  double self_acceleration_bound = 2.50;
  double neighbor_acceleration_bound = 2.50;
  double execution_velocity_error_bound = 0.10;
  double dt_qp_max = 0.03;
  bool use_los = true;
  bool use_semantic_filter = false;
};

struct SwarmIntentParams
{
  double d_minus = 0.80;
  double d_plus = 1.20;
  double k_repulsion = 1.20;
  double k_cohesion = 0.25;
  double k_damping = 0.80;
  double soft_safety_gain = 0.50;
  double soft_safety_max = 0.80;
  double d_act = 0.60;  // soft safety activation distance
  double d_safe = 0.60; // physical safe distance (single authority)
  double k_recenter = 0.40;
};

class NeighborStateBuffer
{
public:
  void update(const common_msgs::SwarmState& msg);
  void prune(const ros::Time& now, double neighbor_timeout,
             double retention_timeout);
  std::vector<NeighborState> snapshot() const;

private:
  mutable std::mutex mutex_;
  std::unordered_map<int, NeighborState> states_;
};

class SwarmEventBuffer
{
public:
  void updatePathEvent(const common_msgs::SwarmPathEvent& msg);
  void updateConflictState(const common_msgs::SwarmConflictState& msg);
  void prune(const ros::Time& now);
  NeighborEventState query(int robot_id) const;
  std::unordered_map<int, NeighborEventState> snapshot() const;

private:
  mutable std::mutex mutex_;
  std::unordered_map<int, NeighborEventState> events_;
};

class NeighborSelector
{
public:
  NeighborSelection select(
      const ros::Time& now,
      const Eigen::Vector3d& self_pos,
      const Eigen::Vector3d& self_vel,
      const std::vector<NeighborState>& raw_neighbors,
      const std::unordered_map<int, NeighborEventState>& events,
      SDFMap* map,
      const NeighborSelectionParams& params) const;

  static bool lineOfSight(SDFMap* map, const Eigen::Vector3d& a,
                          const Eigen::Vector3d& b);

private:
  mutable std::unordered_map<int, bool> membership_;  // enter/exit hysteresis
};

class ElasticSwarmIntent
{
public:
  SwarmIntent compute(const Eigen::Vector3d& self_pos,
                      const Eigen::Vector3d& self_vel,
                      double self_beta,
                      const NeighborSelection& selection,
                      const std::unordered_map<int, NeighborEventState>& events,
                      SDFMap* map,
                      const SwarmIntentParams& params) const;
};

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_SWARM_NEIGHBOR_MODEL_H
