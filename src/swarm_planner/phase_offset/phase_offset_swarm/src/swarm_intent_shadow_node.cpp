#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <phase_offset_msgs/AgentState.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "phase_offset_swarm/neighbor_manager.h"
#include "phase_offset_swarm/swarm_intent.h"

namespace phase_offset_swarm {

class SwarmIntentShadowNode {
 public:
  SwarmIntentShadowNode()
      : nh_(), private_nh_("~"), have_odom_(false) {
    loadParameters();
    manager_.reset(new NeighborManager(robot_id_, parameters_));
    calculator_.reset(new SwarmIntentCalculator(parameters_));

    odom_subscriber_ = nh_.subscribe(local_odom_topic_, 10,
                                     &SwarmIntentShadowNode::odomCallback,
                                     this);
    state_subscriber_ = nh_.subscribe(state_topic_, 100,
                                      &SwarmIntentShadowNode::stateCallback,
                                      this);
    state_publisher_ = nh_.advertise<phase_offset_msgs::AgentState>(
        state_topic_, 10);
    g_coord_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        g_coord_topic_, 10);
    g_sep_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        g_sep_topic_, 10);
    g_coh_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        g_coh_topic_, 10);
    g_conf_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        g_conf_topic_, 10);

    timer_ = nh_.createTimer(
        ros::Duration(1.0 / update_rate_hz_),
        &SwarmIntentShadowNode::timerCallback, this);
  }

 private:
  void loadParameters() {
    int configured_robot_id = 1;
    private_nh_.param("robot_id", configured_robot_id, configured_robot_id);
    if (configured_robot_id < 0 || configured_robot_id > 65535) {
      throw std::invalid_argument("robot_id must fit uint16");
    }
    robot_id_ = configured_robot_id;

    private_nh_.param("expected_world_frame", expected_world_frame_,
                      std::string("world"));
    private_nh_.param("local_odom_topic", local_odom_topic_,
                      std::string("odom"));
    private_nh_.param("state_topic", state_topic_,
                      std::string("agent_states"));
    private_nh_.param("g_coord_topic", g_coord_topic_,
                      std::string("g_coord"));
    private_nh_.param("g_sep_topic", g_sep_topic_, std::string("g_sep"));
    private_nh_.param("g_coh_topic", g_coh_topic_, std::string("g_coh"));
    private_nh_.param("g_conf_topic", g_conf_topic_,
                      std::string("g_conf"));
    private_nh_.param("update_rate_hz", update_rate_hz_, 20.0);
    private_nh_.param("beta_preview", beta_preview_, 1.0);
    if (!std::isfinite(update_rate_hz_) || update_rate_hz_ <= 0.0) {
      throw std::invalid_argument("update_rate_hz must be positive and finite");
    }

    private_nh_.param("d_safe", parameters_.d_safe, parameters_.d_safe);
    private_nh_.param("d_minus", parameters_.d_minus, parameters_.d_minus);
    private_nh_.param("d_plus", parameters_.d_plus, parameters_.d_plus);
    private_nh_.param("r_comm", parameters_.r_comm, parameters_.r_comm);
    private_nh_.param("r_conf", parameters_.r_conf, parameters_.r_conf);
    private_nh_.param("r_safe", parameters_.r_safe, parameters_.r_safe);
    private_nh_.param("k_sep", parameters_.k_sep, parameters_.k_sep);
    private_nh_.param("k_coh", parameters_.k_coh, parameters_.k_coh);
    private_nh_.param("k_conf", parameters_.k_conf, parameters_.k_conf);
    private_nh_.param("ttc_activation", parameters_.ttc_activation,
                      parameters_.ttc_activation);
    private_nh_.param("closing_speed_activation",
                      parameters_.closing_speed_activation,
                      parameters_.closing_speed_activation);
    private_nh_.param("fresh_timeout", parameters_.fresh_timeout,
                      parameters_.fresh_timeout);
    private_nh_.param("stale_timeout", parameters_.stale_timeout,
                      parameters_.stale_timeout);
    private_nh_.param("lost_retention_timeout",
                      parameters_.lost_retention_timeout,
                      parameters_.lost_retention_timeout);
    private_nh_.param("g_max", parameters_.g_max, parameters_.g_max);
    private_nh_.param("distance_epsilon", parameters_.distance_epsilon,
                      parameters_.distance_epsilon);
    private_nh_.param("ttc_epsilon", parameters_.ttc_epsilon,
                      parameters_.ttc_epsilon);
    private_nh_.param("neighbor_hysteresis", parameters_.neighbor_hysteresis,
                      parameters_.neighbor_hysteresis);
    private_nh_.param("future_stamp_tolerance",
                      parameters_.future_stamp_tolerance,
                      parameters_.future_stamp_tolerance);
    parameters_.validateOrThrow();
  }

  bool acceptsFrame(const std::string& frame) const {
    return expected_world_frame_.empty() || frame.empty() ||
           frame == expected_world_frame_;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    if (!acceptsFrame(message->header.frame_id)) {
      ROS_WARN_THROTTLE(2.0,
                        "Ignoring odometry with unexpected world frame '%s'",
                        message->header.frame_id.c_str());
      return;
    }
    self_state_.id = robot_id_;
    self_state_.position =
        Eigen::Vector2d(message->pose.pose.position.x,
                        message->pose.pose.position.y);
    self_state_.velocity =
        Eigen::Vector2d(message->twist.twist.linear.x,
                        message->twist.twist.linear.y);
    self_state_.stamp = message->header.stamp.toSec();
    if (!std::isfinite(self_state_.stamp) || !self_state_.isFinite()) {
      ROS_WARN_THROTTLE(2.0, "Ignoring non-finite local odometry");
      have_odom_ = false;
      return;
    }
    output_frame_ = message->header.frame_id.empty()
                        ? expected_world_frame_
                        : message->header.frame_id;
    have_odom_ = true;

    phase_offset_msgs::AgentState state_message;
    state_message.header = message->header;
    state_message.robot_id = static_cast<uint16_t>(robot_id_);
    state_message.position_world.x = self_state_.position.x();
    state_message.position_world.y = self_state_.position.y();
    state_message.position_world.z = 0.0;
    state_message.velocity_world.x = self_state_.velocity.x();
    state_message.velocity_world.y = self_state_.velocity.y();
    state_message.velocity_world.z = 0.0;
    state_publisher_.publish(state_message);
  }

  void stateCallback(const phase_offset_msgs::AgentStateConstPtr& message) {
    if (!acceptsFrame(message->header.frame_id)) {
      ROS_WARN_THROTTLE(2.0,
                        "Ignoring agent state with unexpected world frame '%s'",
                        message->header.frame_id.c_str());
      return;
    }
    SwarmAgentState received;
    received.id = static_cast<int>(message->robot_id);
    received.position = Eigen::Vector2d(message->position_world.x,
                                        message->position_world.y);
    received.velocity = Eigen::Vector2d(message->velocity_world.x,
                                        message->velocity_world.y);
    received.stamp = message->header.stamp.toSec();
    manager_->update(received);
  }

  geometry_msgs::Vector3Stamped makeVectorMessage(
      const Eigen::Vector2d& vector, const ros::Time& stamp) const {
    geometry_msgs::Vector3Stamped message;
    message.header.stamp = stamp;
    message.header.frame_id = output_frame_;
    message.vector.x = vector.x();
    message.vector.y = vector.y();
    message.vector.z = 0.0;
    return message;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!have_odom_) {
      return;
    }
    const ros::Time now = ros::Time::now();
    const NeighborSnapshot neighbors =
        manager_->snapshot(self_state_, now.toSec());
    const SwarmOutput output =
        calculator_->compute(self_state_, neighbors, beta_preview_);
    g_coord_publisher_.publish(makeVectorMessage(output.g_coord, now));
    g_sep_publisher_.publish(makeVectorMessage(output.g_sep, now));
    g_coh_publisher_.publish(makeVectorMessage(output.g_coh, now));
    g_conf_publisher_.publish(makeVectorMessage(output.g_conf, now));
    ROS_INFO_STREAM_THROTTLE(
        2.0, "swarm intent org=" << output.num_org_neighbors
                                  << " conflict=" << output.num_conflict_neighbors
                                  << " safety=" << output.num_safety_neighbors
                                  << " fresh=" << output.num_fresh_neighbors
                                  << " stale=" << output.num_stale_neighbors
                                  << " lost=" << output.num_lost_neighbors
                                  << " min_distance=" << output.min_distance
                                  << " min_ttc=" << output.min_ttc
                                  << " saturated=" << output.output_saturated);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Publisher state_publisher_;
  ros::Publisher g_coord_publisher_;
  ros::Publisher g_sep_publisher_;
  ros::Publisher g_coh_publisher_;
  ros::Publisher g_conf_publisher_;
  ros::Timer timer_;

  int robot_id_ = 1;
  std::string expected_world_frame_;
  std::string output_frame_;
  std::string local_odom_topic_;
  std::string state_topic_;
  std::string g_coord_topic_;
  std::string g_sep_topic_;
  std::string g_coh_topic_;
  std::string g_conf_topic_;
  double update_rate_hz_ = 20.0;
  double beta_preview_ = 1.0;
  SwarmParameters parameters_;
  SwarmAgentState self_state_;
  bool have_odom_;
  std::unique_ptr<NeighborManager> manager_;
  std::unique_ptr<SwarmIntentCalculator> calculator_;
};

}  // namespace phase_offset_swarm

int main(int argc, char** argv) {
  ros::init(argc, argv, "swarm_intent_shadow_node");
  try {
    phase_offset_swarm::SwarmIntentShadowNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("swarm intent shadow node failed to start: %s", error.what());
    return 1;
  }
  return 0;
}
