// Read-only generic-N SIM-B visualizer.
//
// The node observes only each scenario-selected /uav_i/sim/odom stream and
// publishes package-owned MarkerArrays.  It has no goal, planner, command,
// or control feedback interfaces.
//
// The trajectory marker is the FLOWN path, not a fading tail: it accumulates
// the whole run so a finished flight stays on screen in RViz.
//
// This node publishes per-UAV labels and the flown-path trail only.  The
// quadrotor body itself is the SAME marker the single-UAV chain shows: every
// agent runs odom_visualization, which publishes the hummingbird mesh on
// /uav_i/odom_visualization/robot (ns "mesh").  A coloured sphere used to be
// drawn here as well; it enclosed that 0.4 m mesh, so the swarm looked like
// balls instead of quadrotors.

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <set>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace
{

const double kColors[7][3] = {
  {1.0, 0.2, 0.2},
  {0.2, 1.0, 0.2},
  {0.2, 0.4, 1.0},
  {1.0, 0.8, 0.2},
  {1.0, 0.3, 0.8},
  {0.2, 0.9, 0.9},
  {0.8, 0.6, 1.0},
};

class SwarmVisualizer
{
public:
  explicit SwarmVisualizer(ros::NodeHandle& nh)
  {
    nh.param("scenario_param", scenario_param_,
             std::string("/sim_b_world/scenario/agents"));
    // The swarm is normally started as a separate launch after this world, and
    // its orchestrator publishes the agent list (count-driven or file-driven).
    // Wait for it: 0 (default) waits indefinitely, a positive value bounds the
    // wait.  A scenario file loaded by this world is already present and skips
    // the wait entirely.
    nh.param("scenario_wait_timeout", scenario_wait_timeout_s_, 0.0);
    nh.param("frame_id", frame_id_, std::string("world"));
    // Persistent flown-path trail.  Points are decimated to one per
    // trajectory_min_step so a full run costs a few hundred points per UAV
    // instead of one per odometry tick; trajectory_buffer stays as a hard cap
    // (20000 x 0.05 m = 1 km of flight).
    nh.param("trajectory_buffer", trajectory_buffer_, 20000);
    trajectory_buffer_ = std::max(10, trajectory_buffer_);
    nh.param("trajectory_min_step", trajectory_min_step_, 0.05);
    trajectory_min_step_ = std::max(1e-3, trajectory_min_step_);
    // An odometry jump larger than this is a new run or a teleport, not a
    // flight; restarting the trail there keeps a line from being drawn across
    // the map between two runs.
    nh.param("trajectory_reset_step", trajectory_reset_step_, 5.0);
    trajectory_reset_step_ = std::max(1.0, trajectory_reset_step_);

    if (frame_id_ != "world")
    {
      ROS_ERROR("[SIM_B_VIS] frame_id is frozen to world (got '%s')",
                frame_id_.c_str());
      valid_ = false;
      return;
    }
    if (!loadScenarioAgentIds())
    {
      valid_ = false;
      return;
    }

    uav_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/phase_offset_swarm/vis/uavs", 1);
    trajectory_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/phase_offset_swarm/vis/trajectories", 1);

    nh_ = nh;
    buildSubscriptions();
  }

  bool valid() const { return valid_; }

  void spin()
  {
    ros::Rate rate(10.0);
    ros::Time next_scenario_check = ros::Time::now() + ros::Duration(1.0);
    while (ros::ok())
    {
      ros::spinOnce();
      publishMarkers();
      // The swarm launch may be restarted with a different agent count while
      // this world keeps running.  Follow the published list instead of
      // caching the first one seen.
      if (ros::Time::now() >= next_scenario_check)
      {
        next_scenario_check = ros::Time::now() + ros::Duration(1.0);
        refreshScenarioAgents();
      }
      rate.sleep();
    }
  }

private:
  bool loadScenarioAgentIds()
  {
    XmlRpc::XmlRpcValue agents;
    bool loaded = ros::param::has(scenario_param_) &&
                  ros::param::get(scenario_param_, agents);
    if (!loaded)
    {
      const bool bounded = scenario_wait_timeout_s_ > 0.0;
      const ros::WallTime deadline = bounded
        ? ros::WallTime::now() + ros::WallDuration(scenario_wait_timeout_s_)
        : ros::WallTime::now();
      if (bounded)
      {
        ROS_INFO("[SIM_B_VIS] waiting up to %.1f s for '%s'",
                 scenario_wait_timeout_s_, scenario_param_.c_str());
      }
      else
      {
        ROS_INFO("[SIM_B_VIS] waiting for '%s' (start the swarm launch when "
                 "ready)", scenario_param_.c_str());
      }
      ros::WallTime next_report = ros::WallTime::now() + ros::WallDuration(5.0);
      while (!loaded && ros::ok() && (!bounded || ros::WallTime::now() < deadline))
      {
        ros::WallDuration(0.2).sleep();
        loaded = ros::param::has(scenario_param_) &&
                 ros::param::get(scenario_param_, agents);
        if (!loaded && !bounded && ros::WallTime::now() >= next_report)
        {
          next_report = ros::WallTime::now() + ros::WallDuration(5.0);
          ROS_INFO("[SIM_B_VIS] still waiting for '%s'",
                   scenario_param_.c_str());
        }
      }
    }
    if (!loaded)
    {
      ROS_ERROR("[SIM_B_VIS] required scenario parameter '%s' is unavailable",
                scenario_param_.c_str());
      return false;
    }
    if (agents.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        agents.size() <= 0)
    {
      ROS_ERROR("[SIM_B_VIS] %s must be a non-empty array",
                scenario_param_.c_str());
      return false;
    }

    std::set<int> ids;
    for (int index = 0; index < agents.size(); ++index)
    {
      const XmlRpc::XmlRpcValue& item = agents[index];
      if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
          !item.hasMember("robot_id") ||
          item["robot_id"].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR("[SIM_B_VIS] scenario agent %d has an invalid robot_id",
                  index);
        return false;
      }
      const int robot_id = static_cast<int>(item["robot_id"]);
      if (robot_id < 0 || !ids.insert(robot_id).second)
      {
        ROS_ERROR("[SIM_B_VIS] robot_id values must be unique and non-negative");
        return false;
      }
      robot_ids_.push_back(robot_id);
    }
    std::sort(robot_ids_.begin(), robot_ids_.end());
    for (std::size_t index = 0; index < robot_ids_.size(); ++index)
    {
      if (robot_ids_[index] != static_cast<int>(index))
      {
        ROS_ERROR("[SIM_B_VIS] robot_id values must be contiguous from zero");
        robot_ids_.clear();
        return false;
      }
    }
    ROS_INFO("[SIM_B_VIS] observing %zu scenario agents", robot_ids_.size());
    return true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message,
                    std::size_t index)
  {
    const std::array<double, 3> position{{
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z}};
    if (!std::isfinite(position[0]) || !std::isfinite(position[1]) ||
        !std::isfinite(position[2]))
      return;
    std::deque<std::array<double, 3>>& trail = trajectories_[index];
    if (trail.empty())
    {
      trail.push_back(position);
    }
    else
    {
      const std::array<double, 3>& tip = trail.back();
      const double dx = position[0] - tip[0];
      const double dy = position[1] - tip[1];
      const double dz = position[2] - tip[2];
      const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (step > trajectory_reset_step_)
      {
        trail.clear();
        trail.push_back(position);
      }
      else if (step >= trajectory_min_step_)
      {
        trail.push_back(position);
      }
      // Anything closer than trajectory_min_step_ is not stored: the live pose
      // is appended to the drawn strip instead (see publishMarkers), so the
      // line always reaches the UAV without the trail gaining a point per tick.
    }
    while (static_cast<int>(trail.size()) > trajectory_buffer_)
      trail.pop_front();
    last_positions_[index] = position;
    has_odom_[index] = true;
  }

  // Rebuild the odometry subscriptions for the current agent list.
  void buildSubscriptions()
  {
    odom_subscribers_.clear();
    trajectories_.clear();
    last_positions_.clear();
    has_odom_.clear();
    for (std::size_t index = 0; index < robot_ids_.size(); ++index)
    {
      const int robot_id = robot_ids_[index];
      const std::string topic =
        "/uav_" + std::to_string(robot_id) + "/sim/odom";
      odom_subscribers_.push_back(nh_.subscribe<nav_msgs::Odometry>(
        topic, 10,
        boost::bind(&SwarmVisualizer::odomCallback, this, _1, index)));
      trajectories_.emplace_back();
      last_positions_.push_back(std::array<double, 3>{{0.0, 0.0, 0.0}});
      has_odom_.push_back(false);
    }
  }

  // Re-read the scenario agent list; rebuild only when it actually changed.
  void refreshScenarioAgents()
  {
    // Only re-read when the parameter is present; loadScenarioAgentIds() would
    // otherwise enter its (possibly unbounded) wait from the spin loop.
    if (!ros::param::has(scenario_param_)) return;
    const std::vector<int> previous = robot_ids_;
    robot_ids_.clear();
    if (!loadScenarioAgentIds())
    {
      robot_ids_ = previous;
      return;
    }
    if (robot_ids_ == previous)
    {
      robot_ids_ = previous;
      return;
    }
    buildSubscriptions();
    ROS_INFO("[SIM_B_VIS] scenario agents changed to %zu",
             robot_ids_.size());
  }

  void publishMarkers()
  {
    visualization_msgs::MarkerArray uavs;
    visualization_msgs::MarkerArray paths;
    const ros::Time now = ros::Time::now();

    for (std::size_t index = 0; index < robot_ids_.size(); ++index)
    {
      if (!has_odom_[index])
        continue;
      const int robot_id = robot_ids_[index];
      const double* color = kColors[static_cast<std::size_t>(robot_id) % 7];

      visualization_msgs::Marker label;
      label.header.frame_id = frame_id_;
      label.header.stamp = now;
      label.ns = "uav_" + std::to_string(robot_id) + "/id";
      label.id = robot_id * 1000 + 2;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.pose.position.x = last_positions_[index][0];
      label.pose.position.y = last_positions_[index][1];
      label.pose.position.z = last_positions_[index][2] + 0.5;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.5;
      label.color.r = 1.0;
      label.color.g = 1.0;
      label.color.b = 1.0;
      label.color.a = 1.0;
      label.text = "uav_" + std::to_string(robot_id);
      uavs.markers.push_back(label);

      visualization_msgs::Marker path;
      path.header = label.header;
      path.ns = "uav_" + std::to_string(robot_id) + "/trajectory";
      path.id = robot_id * 1000 + 3;
      path.type = visualization_msgs::Marker::LINE_STRIP;
      path.action = visualization_msgs::Marker::ADD;
      path.pose.orientation.w = 1.0;
      path.scale.x = 0.06;
      path.color.r = color[0];
      path.color.g = color[1];
      path.color.b = color[2];
      path.color.a = 0.7;
      for (const std::array<double, 3>& point : trajectories_[index])
      {
        geometry_msgs::Point marker_point;
        marker_point.x = point[0];
        marker_point.y = point[1];
        marker_point.z = point[2];
        path.points.push_back(marker_point);
      }
      // Close the drawn strip on the live pose (not stored, so it costs no
      // memory and does not defeat the distance decimation above).
      {
        geometry_msgs::Point live_point;
        live_point.x = last_positions_[index][0];
        live_point.y = last_positions_[index][1];
        live_point.z = last_positions_[index][2];
        path.points.push_back(live_point);
      }
      paths.markers.push_back(path);
    }
    uav_pub_.publish(uavs);
    trajectory_pub_.publish(paths);
  }

  bool valid_ = true;
  std::string scenario_param_ = "/sim_b_world/scenario/agents";
  std::string frame_id_ = "world";
  int trajectory_buffer_ = 20000;
  double trajectory_min_step_ = 0.05;
  double trajectory_reset_step_ = 5.0;
  double scenario_wait_timeout_s_ = 0.0;
  ros::NodeHandle nh_;
  std::vector<int> robot_ids_;
  ros::Publisher uav_pub_;
  ros::Publisher trajectory_pub_;
  std::vector<ros::Subscriber> odom_subscribers_;
  std::vector<std::deque<std::array<double, 3>>> trajectories_;
  std::vector<std::array<double, 3>> last_positions_;
  std::vector<bool> has_odom_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "phase_offset_swarm_visualizer");
  ros::NodeHandle nh("~");
  SwarmVisualizer visualizer(nh);
  if (!visualizer.valid())
  {
    ROS_ERROR("[SIM_B_VIS] refusing to run with an invalid scenario");
    return 1;
  }
  visualizer.spin();
  return 0;
}
