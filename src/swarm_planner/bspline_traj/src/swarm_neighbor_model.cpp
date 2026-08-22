#include "bspline_race/swarm_neighbor_model.h"

#include <algorithm>
#include <cmath>

namespace FLAG_Race
{

// ---------------------------------------------------------------------------
// NeighborStateBuffer
// ---------------------------------------------------------------------------

void
NeighborStateBuffer::update(const common_msgs::SwarmState& msg)
{
  NeighborState s;
  s.robot_id = static_cast<int>(msg.robot_id);
  s.stamp = msg.header.stamp;
  s.position = Eigen::Vector3d(msg.position_world.x, msg.position_world.y,
                               msg.position_world.z);
  s.velocity = Eigen::Vector3d(msg.velocity_world.x, msg.velocity_world.y,
                               msg.velocity_world.z);
  s.fresh = true;
  std::lock_guard<std::mutex> lock(mutex_);
  states_[s.robot_id] = s;
}

void
NeighborStateBuffer::prune(const ros::Time& now, double neighbor_timeout,
                           double retention_timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = states_.begin(); it != states_.end();)
  {
    const double age = (now - it->second.stamp).toSec();
    if (age > retention_timeout)
    {
      it = states_.erase(it);
    }
    else
    {
      it->second.fresh = age <= neighbor_timeout;
      ++it;
    }
  }
}

std::vector<NeighborState>
NeighborStateBuffer::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<NeighborState> out;
  out.reserve(states_.size());
  for (const auto& kv : states_)
    out.push_back(kv.second);
  return out;
}

// ---------------------------------------------------------------------------
// SwarmEventBuffer
// ---------------------------------------------------------------------------

void
SwarmEventBuffer::updatePathEvent(const common_msgs::SwarmPathEvent& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  NeighborEventState& e = events_[static_cast<int>(msg.robot_id)];
  if (msg.event_sequence >= e.branch_sequence)
  {
    e.branch_sequence = msg.event_sequence;
    e.branch_event_id = msg.branch_event_id;
    e.branch_valid_until = msg.valid_until;
    e.branch_active = msg.active;
  }
}

void
SwarmEventBuffer::updateConflictState(
  const common_msgs::SwarmConflictState& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  NeighborEventState& e = events_[static_cast<int>(msg.robot_id)];
  if (msg.event_sequence >= e.conflict_sequence)
  {
    e.conflict_sequence = msg.event_sequence;
    e.channel_beta = msg.channel_beta;
    e.conflict_valid_until = msg.valid_until;
    e.conflict_active = msg.active;
  }
}

void
SwarmEventBuffer::prune(const ros::Time& now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = events_.begin(); it != events_.end();)
  {
    if (it->second.branch_active &&
        it->second.branch_valid_until < now)
      it->second.branch_active = false;
    if (it->second.conflict_active &&
        it->second.conflict_valid_until < now)
      it->second.conflict_active = false;
    if (!it->second.branch_active && !it->second.conflict_active)
      it = events_.erase(it);
    else
      ++it;
  }
}

NeighborEventState
SwarmEventBuffer::query(int robot_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = events_.find(robot_id);
  return it == events_.end() ? NeighborEventState() : it->second;
}

std::unordered_map<int, NeighborEventState>
SwarmEventBuffer::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return events_;
}

// ---------------------------------------------------------------------------
// NeighborSelector
// ---------------------------------------------------------------------------

bool
NeighborSelector::lineOfSight(SDFMap* map, const Eigen::Vector3d& a,
                              const Eigen::Vector3d& b)
{
  if (!map)
    return true;
  const Eigen::Vector3d dir = b - a;
  const double dist = dir.norm();
  if (dist < 1e-6)
    return true;
  const Eigen::Vector3d step = dir / dist;
  const double ds = 0.1;
  for (double s = 0.0; s <= dist; s += ds)
  {
    const Eigen::Vector3d p = a + step * s;
    if (!map->isInMap(p))
      return false;  // out-of-map / unknown treated as blocked
    if (map->getInflateOccupancy(p) > 0)
      return false;
  }
  return true;
}

NeighborSelection
NeighborSelector::select(
  const ros::Time& now, const Eigen::Vector3d& self_pos,
  const Eigen::Vector3d& self_vel,
  const std::vector<NeighborState>& raw_neighbors,
  const std::unordered_map<int, NeighborEventState>& events, SDFMap* map,
  const NeighborSelectionParams& params) const
{
  NeighborSelection out;

  for (const NeighborState& n : raw_neighbors)
  {
    const double age = (now - n.stamp).toSec();
    const Eigen::Vector3d rel = n.position - self_pos;
    const double d = rel.norm();

    // Predicted state for safety logic (nominal constant-velocity hold).
    PredictedNeighborState pred;
    pred.robot_id = n.robot_id;
    pred.source_stamp = n.stamp;
    pred.age = std::max(0.0, age);
    pred.predicted_position = n.position + n.velocity * pred.age;
    pred.advertised_velocity = n.velocity;
    // Plan 4.5: velocity uncertainty from the actual message age + execution
    // and sampling-hold margins.
    pred.velocity_uncertainty_bound =
      params.self_velocity_error_bound +
      params.neighbor_velocity_error_bound +
      params.neighbor_acceleration_bound * pred.age +
      (params.self_acceleration_bound +
       params.neighbor_acceleration_bound) * params.dt_qp_max +
      2.0 * params.execution_velocity_error_bound;
    pred.safety_active = true;

    if (!n.fresh)
    {
      // A stale state that is still close must not silently vanish from the
      // safety logic.
      if (d < params.safety_radius + 0.5)
        out.communication_fault = true;
      if (d < params.safety_radius)
        out.safety.push_back(pred);
      continue;
    }

    // Safety set: distance or TTC.
    Eigen::Vector3d rel_v = n.velocity - self_vel;
    const double ddot = rel.dot(rel_v) / std::max(d, 1e-6);
    const double ttc = (ddot < 0.0 && d > 1e-6) ? -d / ddot : 1e9;
    if (d < params.safety_radius || ttc < params.ttc_safe)
      out.safety.push_back(pred);

    // Organization set with enter/exit hysteresis.
    const double radius = membership_[n.robot_id]
                            ? params.organization_exit_radius
                            : params.organization_enter_radius;
    if (d < radius)
    {
      if (params.use_los && map &&
          !lineOfSight(map, self_pos, n.position))
      {
        continue;  // LOS only gates organization edges
      }
      NeighborState org = n;
      org.fresh = true;
      out.organization.push_back(org);
      membership_[n.robot_id] = true;
    }
    else
    {
      membership_[n.robot_id] = false;
    }
  }

  // Drop hysteresis membership for neighbors that vanished entirely.
  std::unordered_map<int, bool> next_membership;
  for (const NeighborState& n : raw_neighbors)
  {
    auto it = membership_.find(n.robot_id);
    if (it != membership_.end() && it->second)
      next_membership[n.robot_id] = true;
  }
  membership_ = next_membership;
  return out;
}

// ---------------------------------------------------------------------------
// ElasticSwarmIntent
// ---------------------------------------------------------------------------

SwarmIntent
ElasticSwarmIntent::compute(
  const Eigen::Vector3d& self_pos, const Eigen::Vector3d& self_vel,
  double self_beta, const NeighborSelection& selection,
  const std::unordered_map<int, NeighborEventState>& events, SDFMap* map,
  const SwarmIntentParams& params) const
{
  SwarmIntent intent;
  const double Rc = 1.55;  // organization radius for the weight kernel

  auto weight = [Rc](double d) {
    if (d >= Rc)
      return 0.0;
    return 0.5 * (1.0 + std::cos(M_PI * d / Rc));
  };

  // Organization edges: position band + radial damping.
  for (const NeighborState& n : selection.organization)
  {
    PairIntentContribution c;
    c.robot_id = n.robot_id;
    const Eigen::Vector3d rel = n.position - self_pos;
    c.distance = rel.norm();
    if (c.distance < 1e-6)
      continue;
    c.n_ij = rel / c.distance;
    c.distance_rate = (n.velocity - self_vel).dot(c.n_ij);

    const double w = weight(c.distance);
    double phi = 0.0;
    if (c.distance < params.d_minus)
      phi = -params.k_repulsion * (params.d_minus - c.distance);
    else if (c.distance > params.d_plus && c.distance < Rc)
      phi = params.k_cohesion * (c.distance - params.d_plus);

    // beta_ij only attenuates weak cohesion.
    double beta_ij = self_beta;
    auto eit = events.find(n.robot_id);
    if (eit != events.end())
      beta_ij = std::min(beta_ij, eit->second.channel_beta);
    if (phi > 0.0)
      phi *= beta_ij;

    c.g_pos = w * phi * c.n_ij;
    c.g_damp = params.k_damping * w * c.distance_rate * c.n_ij;
    intent.g_pos += c.g_pos;
    intent.g_damp += c.g_damp;
    intent.pair_contributions.push_back(c);
    intent.min_neighbor_distance = std::min(intent.min_neighbor_distance,
                                            c.distance);
  }
  intent.organization_neighbor_count =
    static_cast<int>(selection.organization.size());
  for (const NeighborState& n : selection.organization)
    intent.organization_neighbor_ids.push_back(n.robot_id);

  // Safety set: bounded soft safety repulsion.
  for (const PredictedNeighborState& n : selection.safety)
  {
    const Eigen::Vector3d rel = self_pos - n.predicted_position;
    const double d = rel.norm();
    const double d_eps = std::sqrt(d * d + 0.01);
    const Eigen::Vector3d n_eps = rel / d_eps;
    const double push =
      (1.0 / d_eps - 1.0 / params.d_act);
    if (push > 0.0)
    {
      const Eigen::Vector3d g =
        params.soft_safety_gain * push * n_eps / (d_eps * d_eps);
      const double norm = g.norm();
      intent.g_safe += norm > params.soft_safety_max
                         ? g * (params.soft_safety_max / norm)
                         : g;
    }
    intent.safety_neighbor_ids.push_back(n.robot_id);
    intent.min_neighbor_distance = std::min(intent.min_neighbor_distance, d);
  }
  intent.safety_neighbor_count = static_cast<int>(selection.safety.size());

  intent.g_swarm = intent.g_pos + intent.g_damp + intent.g_safe;
  intent.g_des = intent.g_swarm;
  return intent;
}

}  // namespace FLAG_Race
