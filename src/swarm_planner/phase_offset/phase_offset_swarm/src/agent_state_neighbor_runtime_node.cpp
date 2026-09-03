#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <phase_offset_msgs/AgentState.h>
#include <phase_offset_msgs/BetaSample.h>
#include <phase_offset_msgs/GCoordSample.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include "phase_offset_swarm/neighbor_manager.h"
#include "phase_offset_swarm/sph_provider_runtime.h"

namespace phase_offset_swarm {
namespace {

bool finite(double value) { return std::isfinite(value); }

bool finitePoint(const geometry_msgs::Point& point) {
  return finite(point.x) && finite(point.y) && finite(point.z);
}

bool finiteVector(const geometry_msgs::Vector3& vector) {
  return finite(vector.x) && finite(vector.y) && finite(vector.z);
}

}  // namespace

class AgentStateNeighborRuntime {
 public:
  AgentStateNeighborRuntime()
      : nh_(), private_nh_("~"), have_odom_(false),
        last_published_stamp_(0.0) {
    loadParameters();
    manager_.reset(new NeighborManager(manager_config_));

    // The odometry input is private/remappable; the AgentState wire is one
    // configurable shared topic used by every independent runtime.
    odom_subscriber_ = private_nh_.subscribe(
        "odom", 10, &AgentStateNeighborRuntime::odomCallback, this);
    state_subscriber_ = nh_.subscribe(
        agent_state_topic_, 200,
        &AgentStateNeighborRuntime::stateCallback, this);
    state_publisher_ = nh_.advertise<phase_offset_msgs::AgentState>(
        agent_state_topic_, 10, false);
    timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_rate_hz_),
        &AgentStateNeighborRuntime::publishTimer, this);
    snapshot_service_ = private_nh_.advertiseService(
        "snapshot", &AgentStateNeighborRuntime::snapshotService, this);

    latest_snapshot_.self_id = robot_id_;

    if (enable_sph_provider_) {
      SphProviderConfig provider_config;
      provider_config.robot_id = robot_id_;
      provider_config.world_frame = world_frame_;
      provider_config.beta_fresh_timeout = beta_fresh_timeout_;
      provider_config.snapshot_fresh_timeout = snapshot_fresh_timeout_;
      provider_config.own_odom_timeout = own_odom_timeout_;
      provider_config.future_timestamp_tolerance =
          manager_config_.future_timestamp_tolerance;
      provider_config.provider_epoch = providerEpoch();
      sph_provider_.reset(new SphProviderRuntime(provider_config));
      beta_subscriber_ = private_nh_.subscribe(
          "beta", 10, &AgentStateNeighborRuntime::betaCallback, this);
      g_coord_publisher_ = private_nh_.advertise<phase_offset_msgs::GCoordSample>(
          "g_coord", 10, false);
      sph_provider_timer_ = nh_.createTimer(
          ros::Duration(kSphProviderPeriodSec),
          &AgentStateNeighborRuntime::sphProviderTimer, this);
    }
  }

 private:
  void loadParameters() {
    int robot_id = 0;
    int agent_count = 1;
    private_nh_.param("robot_id", robot_id, robot_id);
    private_nh_.param("agent_count", agent_count, agent_count);
    private_nh_.param("world_frame", world_frame_, std::string("world"));
    private_nh_.param("agent_state_topic", agent_state_topic_,
                      std::string("/phase_offset/agent_state"));
    private_nh_.param("publish_rate_hz", publish_rate_hz_, 20.0);
    private_nh_.param("own_odom_timeout", own_odom_timeout_, 0.20);
    private_nh_.param("enable_sph_provider", enable_sph_provider_, false);
    bool enable_neighbor_transport = true;
    private_nh_.param("enable_neighbor_transport", enable_neighbor_transport,
                      true);
    private_nh_.param("beta_fresh_timeout", beta_fresh_timeout_, 0.10);
    private_nh_.param("snapshot_fresh_timeout", snapshot_fresh_timeout_,
                      0.10);

    manager_config_.self_id = robot_id;
    manager_config_.agent_count = agent_count;
    manager_config_.world_frame = world_frame_;
    private_nh_.param("fresh_timeout", manager_config_.fresh_timeout,
                      manager_config_.fresh_timeout);
    private_nh_.param("lost_timeout", manager_config_.lost_timeout,
                      manager_config_.lost_timeout);
    private_nh_.param("retention_timeout", manager_config_.retention_timeout,
                      manager_config_.retention_timeout);
    private_nh_.param("prediction_horizon_max",
                      manager_config_.prediction_horizon_max,
                      manager_config_.prediction_horizon_max);
    private_nh_.param("future_timestamp_tolerance",
                      manager_config_.future_timestamp_tolerance,
                      manager_config_.future_timestamp_tolerance);
    private_nh_.param("source_too_old_timeout",
                      manager_config_.source_too_old_timeout,
                      manager_config_.source_too_old_timeout);
    private_nh_.param("enter_radius", manager_config_.enter_radius,
                      manager_config_.enter_radius);
    private_nh_.param("exit_radius", manager_config_.exit_radius,
                      manager_config_.exit_radius);
    int max_neighbors = 0;
    private_nh_.param("max_neighbors", max_neighbors, max_neighbors);
    if (max_neighbors < 0) {
      throw std::invalid_argument("max_neighbors must be non-negative");
    }
    manager_config_.max_neighbors = static_cast<std::size_t>(max_neighbors);

    if (robot_id < 0 || !finite(publish_rate_hz_) ||
        publish_rate_hz_ <= 0.0 || !finite(own_odom_timeout_) ||
        own_odom_timeout_ < 0.0 || !finite(beta_fresh_timeout_) ||
        beta_fresh_timeout_ < 0.0 || !finite(snapshot_fresh_timeout_) ||
        snapshot_fresh_timeout_ < 0.0 || agent_state_topic_.empty()) {
      throw std::invalid_argument("invalid SIM-C runtime parameters");
    }
    if (enable_sph_provider_ && !enable_neighbor_transport) {
      throw std::invalid_argument(
          "enable_sph_provider requires enable_neighbor_transport");
    }
    manager_config_.validateOrThrow();
    robot_id_ = robot_id;
  }

  std::uint64_t providerEpoch() const {
    const std::uint64_t wall_epoch = ros::WallTime::now().toNSec();
    if (wall_epoch != 0u) {
      return wall_epoch;
    }
    // The fallback is initialized once per process.  Steady time and PID
    // make it nonzero and distinct across respawn while remaining fixed for
    // the lifetime of this process.
    static const std::uint64_t process_fallback = []() {
      const std::uint64_t steady_epoch = ros::SteadyTime::now().toNSec();
      const std::uint64_t pid_epoch = static_cast<std::uint64_t>(::getpid());
      const std::uint64_t mixed =
          steady_epoch ^ (pid_epoch * 0x9e3779b97f4a7c15ULL) ^
          0xd1b54a32d192ed03ULL;
      return mixed == 0u ? 1u : mixed;
    }();
    return process_fallback;
  }

  bool validOdom(const nav_msgs::Odometry& message) const {
    const double stamp = message.header.stamp.toSec();
    return message.header.frame_id == world_frame_ && finite(stamp) &&
           stamp > 0.0 && finitePoint(message.pose.pose.position) &&
           finiteVector(message.twist.twist.linear);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    if (!validOdom(*message)) {
      ROS_WARN_THROTTLE(2.0,
                        "SIM-C ignoring odometry that is not finite world data");
      return;
    }
    const double stamp = message->header.stamp.toSec();
    if (have_odom_ && stamp <= latest_self_.stamp) {
      // Callback arrival order must not rewind the latest own sample.
      return;
    }

    latest_self_.id = robot_id_;
    latest_self_.frame_id = message->header.frame_id;
    latest_self_.position = Eigen::Vector2d(
        message->pose.pose.position.x, message->pose.pose.position.y);
    latest_self_.velocity = Eigen::Vector2d(
        message->twist.twist.linear.x, message->twist.twist.linear.y);
    latest_self_.position_z = message->pose.pose.position.z;
    latest_self_.velocity_z = message->twist.twist.linear.z;
    latest_self_.stamp = stamp;
    latest_odom_header_ = message->header;
    latest_odom_receive_time_ = ros::SteadyTime::now();
    have_odom_ = true;
  }

  SwarmAgentState convert(const phase_offset_msgs::AgentState& message) const {
    SwarmAgentState result;
    result.id = static_cast<int>(message.robot_id);
    result.frame_id = message.header.frame_id;
    result.position = Eigen::Vector2d(message.position_world.x,
                                      message.position_world.y);
    result.velocity = Eigen::Vector2d(message.velocity_world.x,
                                      message.velocity_world.y);
    result.position_z = message.position_world.z;
    result.velocity_z = message.velocity_world.z;
    result.stamp = message.header.stamp.toSec();
    return result;
  }

  void stateCallback(const phase_offset_msgs::AgentStateConstPtr& message) {
    const ros::SteadyTime receive_time = ros::SteadyTime::now();
    const ros::Time source_now = ros::Time::now();
    const NeighborUpdateReason reason = manager_->updateWithReason(
        convert(*message), receive_time.toSec(), source_now.toSec());
    if (reason != NeighborUpdateReason::ACCEPTED &&
        reason != NeighborUpdateReason::SELF_FILTERED) {
      ROS_WARN_THROTTLE(
          2.0, "SIM-C AgentState rejected: %s",
          neighborUpdateReasonName(reason));
    }
  }

  void betaCallback(const phase_offset_msgs::BetaSampleConstPtr& message) {
    if (!sph_provider_) {
      return;
    }
    const ros::Time source_now = ros::Time::now();
    const ros::SteadyTime receive_steady_time = ros::SteadyTime::now();
    BetaSampleValue sample;
    sample.robot_id = static_cast<int>(message->robot_id);
    sample.frame_id = message->header.frame_id;
    sample.beta_i = message->beta_i;
    sample.source_stamp = message->header.stamp.toSec();
    sample.sequence = message->sequence;
    sample.producer_epoch = message->producer_epoch;
    sample.valid = message->valid;
    const BetaInputUpdateResult result = sph_provider_->betaBuffer().update(
        sample, receive_steady_time.toSec(), source_now.toSec());
    if (!result.accepted()) {
      ROS_WARN_THROTTLE(2.0, "SPH beta sample rejected: status=%d",
                        static_cast<int>(result.status));
    }
  }

  void sphProviderTimer(const ros::TimerEvent&) {
    if (!sph_provider_) {
      return;
    }
    const ros::Time source_now = ros::Time::now();
    const ros::SteadyTime provider_steady_time = ros::SteadyTime::now();
    const SphProviderEvaluation evaluation = sph_provider_->evaluate(
        latest_self_, latest_odom_receive_time_.toSec(), latest_snapshot_,
        source_now.toSec(), provider_steady_time.toSec());

    phase_offset_msgs::GCoordSample message;
    message.header.stamp = source_now;
    message.header.frame_id = evaluation.sample.frame_id;
    message.robot_id = static_cast<std::uint16_t>(evaluation.sample.robot_id);
    message.g_coord.x = evaluation.sample.g_coord.x();
    message.g_coord.y = evaluation.sample.g_coord.y();
    message.g_coord.z = evaluation.sample.g_coord.z();
    message.sequence = evaluation.sample.sequence;
    message.producer_epoch = evaluation.sample.producer_epoch;
    message.valid = evaluation.sample.valid;
    g_coord_publisher_.publish(message);
  }

  void publishTimer(const ros::TimerEvent&) {
    const ros::Time source_now = ros::Time::now();
    const ros::SteadyTime receive_now = ros::SteadyTime::now();
    if (have_odom_) {
      const double receive_age =
          (receive_now - latest_odom_receive_time_).toSec();
      if (finite(receive_age) && receive_age <= own_odom_timeout_ &&
          latest_self_.stamp > last_published_stamp_) {
        phase_offset_msgs::AgentState message;
        message.header = latest_odom_header_;
        message.robot_id = static_cast<uint16_t>(robot_id_);
        const Eigen::Vector3d position = latest_self_.positionWorld();
        const Eigen::Vector3d velocity = latest_self_.velocityWorld();
        message.position_world.x = position.x();
        message.position_world.y = position.y();
        message.position_world.z = position.z();
        message.velocity_world.x = velocity.x();
        message.velocity_world.y = velocity.y();
        message.velocity_world.z = velocity.z();
        state_publisher_.publish(message);
        last_published_stamp_ = latest_self_.stamp;
      }

      latest_snapshot_ = manager_->snapshot(
          latest_self_, source_now.toSec(), receive_now.toSec());
    }
  }

  static void appendIds(std::ostringstream& stream,
                        const NeighborStateVector& states) {
    stream << '[';
    for (std::size_t index = 0; index < states.size(); ++index) {
      if (index != 0u) {
        stream << ',';
      }
      stream << states[index].id;
    }
    stream << ']';
  }

  static void appendCounters(std::ostringstream& stream,
                             const NeighborUpdateCounters& counters) {
    stream << "{\"INVALID_ID\":" << counters.invalid_id
           << ",\"SELF_FILTERED\":" << counters.self_filtered
           << ",\"NONFINITE_STATE\":" << counters.nonfinite_state
           << ",\"FRAME_MISMATCH\":" << counters.frame_mismatch
           << ",\"INVALID_TIMESTAMP\":" << counters.invalid_timestamp
           << ",\"FUTURE_TIMESTAMP\":" << counters.future_timestamp
           << ",\"SOURCE_TOO_OLD\":" << counters.source_too_old
           << ",\"DUPLICATE\":" << counters.duplicate
           << ",\"OUT_OF_ORDER\":" << counters.out_of_order
           << ",\"ACCEPTED\":" << counters.accepted << '}';
  }

  bool snapshotService(std_srvs::Trigger::Request&,
                       std_srvs::Trigger::Response& response) {
    const NeighborSnapshot snapshot = latest_snapshot_;
    std::ostringstream stream;
    stream << std::setprecision(17);
    stream << "{\"self_id\":" << snapshot.self_id
           << ",\"source_query_time\":" << snapshot.source_query_time
           << ",\"receive_query_time\":" << snapshot.receive_query_time
           << ",\"active_ids\":";
    appendIds(stream, snapshot.active);
    stream << ",\"fresh_count\":" << snapshot.fresh_count
           << ",\"stale_count\":" << snapshot.stale_count
           << ",\"lost_count\":" << snapshot.lost_count
           << ",\"cache_record_count\":" << snapshot.cache_record_count
           << ",\"counters\":";
    appendCounters(stream, snapshot.counters);
    stream << '}';
    response.success = true;
    response.message = stream.str();
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Publisher state_publisher_;
  ros::Publisher g_coord_publisher_;
  ros::Timer timer_;
  ros::Timer sph_provider_timer_;
  ros::ServiceServer snapshot_service_;
  ros::Subscriber beta_subscriber_;

  int robot_id_ = 0;
  std::string world_frame_;
  std::string agent_state_topic_;
  double publish_rate_hz_ = 20.0;
  double own_odom_timeout_ = 0.20;
  double beta_fresh_timeout_ = 0.10;
  double snapshot_fresh_timeout_ = 0.10;
  bool enable_sph_provider_ = false;
  NeighborManagerConfig manager_config_;
  std::unique_ptr<NeighborManager> manager_;
  std::unique_ptr<SphProviderRuntime> sph_provider_;

  SwarmAgentState latest_self_;
  bool have_odom_;
  ros::SteadyTime latest_odom_receive_time_;
  std_msgs::Header latest_odom_header_;
  double last_published_stamp_;
  NeighborSnapshot latest_snapshot_;
};

}  // namespace phase_offset_swarm

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_c_neighbor_runtime");
  try {
    phase_offset_swarm::AgentStateNeighborRuntime runtime;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("SIM-C AgentState/NeighborManager runtime failed: %s",
              error.what());
    return 1;
  }
  return 0;
}
