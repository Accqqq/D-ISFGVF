// Read-only swarm visualizer (batch 2).
//
// Pure observer: subscribes to /uav_i/sim/odom and publishes aggregated
// MarkerArrays on /phase_offset_swarm/vis/uavs and
// /phase_offset_swarm/vis/trajectories.  It never publishes commands or
// control feedback; killing it must not change any UAV trajectory.

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace
{

const double kColors[7][3] = {
  { 1.0, 0.2, 0.2 },  // red
  { 0.2, 1.0, 0.2 },  // green
  { 0.2, 0.4, 1.0 },  // blue
  { 1.0, 0.8, 0.2 },  // yellow
  { 1.0, 0.3, 0.8 },  // magenta
  { 0.2, 0.9, 0.9 },  // cyan
  { 0.8, 0.6, 1.0 },  // violet
};

class SwarmVisualizer
{
public:
  SwarmVisualizer(ros::NodeHandle& nh)
  {
    nh.param("num_agents", num_agents_, 1);
    nh.param("frame_id", frame_id_, std::string("world"));
    nh.param("trajectory_buffer", trajectory_buffer_, 600);
    num_agents_ = std::max(1, std::min(20, num_agents_));
    trajectory_buffer_ = std::max(10, trajectory_buffer_);

    uav_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/phase_offset_swarm/vis/uavs", 1);
    traj_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/phase_offset_swarm/vis/trajectories", 1);

    for (int i = 0; i < num_agents_; ++i)
    {
      const std::string topic =
        "/uav_" + std::to_string(i) + "/sim/odom";
      odom_subs_.push_back(nh.subscribe<nav_msgs::Odometry>(
        topic, 10,
        boost::bind(&SwarmVisualizer::odomCallback, this, _1, i)));
      traj_.emplace_back();
      last_pos_.emplace_back();
      has_odom_.push_back(false);
    }
  }

  void spin()
  {
    ros::Rate r(10.0);
    while (ros::ok())
    {
      ros::spinOnce();
      publishMarkers();
      r.sleep();
    }
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int robot_id)
  {
    // Use std::array<double,3> (not Eigen fixed-size types) inside the
    // deque: with C++14 the standard allocator does not guarantee the 16-byte
    // alignment Eigen::Vector3d requires.
    std::array<double, 3> pos = {
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z };
    traj_[robot_id].push_back(pos);
    while (static_cast<int>(traj_[robot_id].size()) > trajectory_buffer_)
      traj_[robot_id].pop_front();
    last_pos_[robot_id] = pos;
    has_odom_[robot_id] = true;
  }

  void publishMarkers()
  {
    visualization_msgs::MarkerArray uavs;
    visualization_msgs::MarkerArray trajs;
    const ros::Time now = ros::Time::now();

    for (int i = 0; i < num_agents_; ++i)
    {
      if (!has_odom_[i])
        continue;
      const double* c = kColors[i % 7];

      visualization_msgs::Marker body;
      body.header.frame_id = frame_id_;
      body.header.stamp = now;
      body.ns = "uav_" + std::to_string(i) + "/body";
      body.id = i * 1000 + 1;
      body.type = visualization_msgs::Marker::SPHERE;
      body.action = visualization_msgs::Marker::ADD;
      body.pose.position.x = last_pos_[i][0];
      body.pose.position.y = last_pos_[i][1];
      body.pose.position.z = last_pos_[i][2];
      body.pose.orientation.w = 1.0;
      body.scale.x = 0.55;
      body.scale.y = 0.55;
      body.scale.z = 0.35;
      body.color.r = c[0];
      body.color.g = c[1];
      body.color.b = c[2];
      body.color.a = 0.9;
      uavs.markers.push_back(body);

      visualization_msgs::Marker label;
      label.header = body.header;
      label.ns = "uav_" + std::to_string(i) + "/id";
      label.id = i * 1000 + 2;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.pose.position.x = last_pos_[i][0];
      label.pose.position.y = last_pos_[i][1];
      label.pose.position.z = last_pos_[i][2] + 0.5;
      label.scale.z = 0.5;
      label.color.r = 1.0;
      label.color.g = 1.0;
      label.color.b = 1.0;
      label.color.a = 1.0;
      label.text = "uav_" + std::to_string(i);
      uavs.markers.push_back(label);

      visualization_msgs::Marker traj;
      traj.header = body.header;
      traj.ns = "uav_" + std::to_string(i) + "/trajectory";
      traj.id = i * 1000 + 3;
      traj.type = visualization_msgs::Marker::LINE_STRIP;
      traj.action = visualization_msgs::Marker::ADD;
      traj.scale.x = 0.06;
      traj.color.r = c[0];
      traj.color.g = c[1];
      traj.color.b = c[2];
      traj.color.a = 0.7;
      traj.pose.orientation.w = 1.0;
      for (const std::array<double, 3>& p : traj_[i])
      {
        geometry_msgs::Point pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];
        traj.points.push_back(pt);
      }
      trajs.markers.push_back(traj);
    }
    uav_pub_.publish(uavs);
    traj_pub_.publish(trajs);
  }

  int num_agents_ = 1;
  std::string frame_id_ = "world";
  int trajectory_buffer_ = 600;
  ros::Publisher uav_pub_;
  ros::Publisher traj_pub_;
  std::vector<ros::Subscriber> odom_subs_;
  std::vector<std::deque<std::array<double, 3>>> traj_;
  std::vector<std::array<double, 3>> last_pos_;
  std::vector<bool> has_odom_;
};

}  // namespace

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "phase_offset_swarm_visualizer");
  ros::NodeHandle nh("~");
  SwarmVisualizer viz(nh);
  viz.spin();
  return 0;
}
