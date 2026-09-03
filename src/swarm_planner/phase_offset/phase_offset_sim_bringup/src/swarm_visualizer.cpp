// Read-only generic-N SIM-B visualizer.
//
// The node observes only each scenario-selected /uav_i/sim/odom stream and
// publishes package-owned MarkerArrays.  It has no goal, planner, command,
// or control feedback interfaces.

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
    nh.param("frame_id", frame_id_, std::string("world"));
    nh.param("trajectory_buffer", trajectory_buffer_, 600);
    trajectory_buffer_ = std::max(10, trajectory_buffer_);

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

    for (std::size_t index = 0; index < robot_ids_.size(); ++index)
    {
      const int robot_id = robot_ids_[index];
      const std::string topic =
        "/uav_" + std::to_string(robot_id) + "/sim/odom";
      odom_subscribers_.push_back(nh.subscribe<nav_msgs::Odometry>(
        topic, 10,
        boost::bind(&SwarmVisualizer::odomCallback, this, _1, index)));
      trajectories_.emplace_back();
      last_positions_.push_back(std::array<double, 3>{{0.0, 0.0, 0.0}});
      has_odom_.push_back(false);
    }
  }

  bool valid() const { return valid_; }

  void spin()
  {
    ros::Rate rate(10.0);
    while (ros::ok())
    {
      ros::spinOnce();
      publishMarkers();
      rate.sleep();
    }
  }

private:
  bool loadScenarioAgentIds()
  {
    XmlRpc::XmlRpcValue agents;
    if (!ros::param::has(scenario_param_) ||
        !ros::param::get(scenario_param_, agents))
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
    trajectories_[index].push_back(position);
    while (static_cast<int>(trajectories_[index].size()) > trajectory_buffer_)
      trajectories_[index].pop_front();
    last_positions_[index] = position;
    has_odom_[index] = true;
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

      visualization_msgs::Marker body;
      body.header.frame_id = frame_id_;
      body.header.stamp = now;
      body.ns = "uav_" + std::to_string(robot_id) + "/body";
      body.id = robot_id * 1000 + 1;
      body.type = visualization_msgs::Marker::SPHERE;
      body.action = visualization_msgs::Marker::ADD;
      body.pose.position.x = last_positions_[index][0];
      body.pose.position.y = last_positions_[index][1];
      body.pose.position.z = last_positions_[index][2];
      body.pose.orientation.w = 1.0;
      body.scale.x = 0.55;
      body.scale.y = 0.55;
      body.scale.z = 0.35;
      body.color.r = color[0];
      body.color.g = color[1];
      body.color.b = color[2];
      body.color.a = 0.9;
      uavs.markers.push_back(body);

      visualization_msgs::Marker label;
      label.header = body.header;
      label.ns = "uav_" + std::to_string(robot_id) + "/id";
      label.id = robot_id * 1000 + 2;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.pose.position.x = body.pose.position.x;
      label.pose.position.y = body.pose.position.y;
      label.pose.position.z = body.pose.position.z + 0.5;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.5;
      label.color.r = 1.0;
      label.color.g = 1.0;
      label.color.b = 1.0;
      label.color.a = 1.0;
      label.text = "uav_" + std::to_string(robot_id);
      uavs.markers.push_back(label);

      visualization_msgs::Marker path;
      path.header = body.header;
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
      paths.markers.push_back(path);
    }
    uav_pub_.publish(uavs);
    trajectory_pub_.publish(paths);
  }

  bool valid_ = true;
  std::string scenario_param_ = "/sim_b_world/scenario/agents";
  std::string frame_id_ = "world";
  int trajectory_buffer_ = 600;
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
