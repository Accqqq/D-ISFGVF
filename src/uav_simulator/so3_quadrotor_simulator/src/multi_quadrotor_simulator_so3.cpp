#include <quadrotor_simulator/multi_quadrotor_simulator.h>

#include <algorithm>
#include <limits>

#include <xmlrpcpp/XmlRpcValue.h>

namespace {

struct InitialState {
  int robot_id = -1;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double yaw = 0.0;
};

bool finiteNumber(const XmlRpc::XmlRpcValue& value, double& output) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    output = static_cast<int>(value);
    return std::isfinite(output);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    output = static_cast<double>(value);
    return std::isfinite(output);
  }
  return false;
}

bool readInitialStates(const ros::NodeHandle& node, int num_agents,
                       std::vector<InitialState>& states) {
  XmlRpc::XmlRpcValue value;
  if (!node.getParam("initial_states", value) ||
      value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      value.size() != num_agents) {
    ROS_ERROR("[B0_MULTI] initial_states must contain exactly %d entries",
              num_agents);
    return false;
  }

  states.clear();
  states.resize(static_cast<std::size_t>(num_agents));
  std::vector<bool> seen(static_cast<std::size_t>(num_agents), false);
  for (int index = 0; index < num_agents; ++index) {
    const XmlRpc::XmlRpcValue& item = value[index];
    if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        item.size() != 5 || !item.hasMember("robot_id") ||
        !item.hasMember("x") || !item.hasMember("y") ||
        !item.hasMember("z") || !item.hasMember("yaw")) {
      ROS_ERROR("[B0_MULTI] initial_states[%d] has an invalid field set",
                index);
      return false;
    }

    const XmlRpc::XmlRpcValue& id_value = item["robot_id"];
    if (id_value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
      ROS_ERROR("[B0_MULTI] initial_states[%d].robot_id must be an integer",
                index);
      return false;
    }
    const int robot_id = static_cast<int>(id_value);
    if (robot_id < 0 || robot_id >= num_agents || seen[robot_id]) {
      ROS_ERROR("[B0_MULTI] robot_id values must be unique and contiguous");
      return false;
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    if (!finiteNumber(item["x"], x) || !finiteNumber(item["y"], y) ||
        !finiteNumber(item["z"], z) || !finiteNumber(item["yaw"], yaw) ||
        z <= 0.0) {
      ROS_ERROR("[B0_MULTI] initial state numeric values must be finite and z>0");
      return false;
    }

    InitialState state;
    state.robot_id = robot_id;
    state.position = Eigen::Vector3d(x, y, z);
    state.yaw = yaw;
    states[robot_id] = state;
    seen[robot_id] = true;
  }
  return std::all_of(seen.begin(), seen.end(), [](bool value) { return value; });
}

QuadrotorSimulator::SimulatorCommand defaultCommand() {
  QuadrotorSimulator::SimulatorCommand command{};
  command.force[0] = 0.0f;
  command.force[1] = 0.0f;
  command.force[2] = 0.0f;
  command.qx = 0.0f;
  command.qy = 0.0f;
  command.qz = 0.0f;
  command.qw = 1.0f;
  command.kR[0] = 0.0f;
  command.kR[1] = 0.0f;
  command.kR[2] = 0.0f;
  command.kOm[0] = 0.0f;
  command.kOm[1] = 0.0f;
  command.kOm[2] = 0.0f;
  command.corrections[0] = 0.0f;
  command.corrections[1] = 0.0f;
  command.corrections[2] = 0.0f;
  command.current_yaw = 0.0f;
  command.use_external_yaw = false;
  return command;
}

void setHoverCommand(QuadrotorSimulator::SimulatedAgent& agent, double yaw) {
  const double hover_force = agent.quad.getMass() * agent.quad.getGravity();
  agent.command = defaultCommand();
  agent.command.force[2] = static_cast<float>(hover_force);
  agent.command.qz = static_cast<float>(std::sin(yaw / 2.0));
  agent.command.qw = static_cast<float>(std::cos(yaw / 2.0));
}

}  // namespace

namespace QuadrotorSimulator {

void SimulatedAgent::commandCallback(
    const quadrotor_msgs::SO3Command::ConstPtr& message) {
  command.force[0] = message->force.x;
  command.force[1] = message->force.y;
  command.force[2] = message->force.z;
  command.qx = message->orientation.x;
  command.qy = message->orientation.y;
  command.qz = message->orientation.z;
  command.qw = message->orientation.w;
  command.kR[0] = message->kR[0];
  command.kR[1] = message->kR[1];
  command.kR[2] = message->kR[2];
  command.kOm[0] = message->kOm[0];
  command.kOm[1] = message->kOm[1];
  command.kOm[2] = message->kOm[2];
  command.corrections[0] = message->aux.kf_correction;
  command.corrections[1] = message->aux.angle_corrections[0];
  command.corrections[2] = message->aux.angle_corrections[1];
  command.current_yaw = message->aux.current_yaw;
  command.use_external_yaw = message->aux.use_external_yaw;
}

void SimulatedAgent::forceCallback(
    const geometry_msgs::Vector3::ConstPtr& message) {
  disturbance.force.x() = message->x;
  disturbance.force.y() = message->y;
  disturbance.force.z() = message->z;
}

void SimulatedAgent::momentCallback(
    const geometry_msgs::Vector3::ConstPtr& message) {
  disturbance.moment.x() = message->x;
  disturbance.moment.y() = message->y;
  disturbance.moment.z() = message->z;
}

bool MultiQuadrotorSimulator::initialize(
    ros::NodeHandle& node, int num_agents, const std::string& frame_id,
    bool start_at_hover, const std::vector<Eigen::Vector3d>& positions,
    const std::vector<double>& yaws) {
  if ((num_agents != 1 && num_agents != 3) ||
      positions.size() != static_cast<std::size_t>(num_agents) ||
      yaws.size() != static_cast<std::size_t>(num_agents)) {
    return false;
  }

  agents_.reserve(static_cast<std::size_t>(num_agents));
  for (int i = 0; i < num_agents; ++i) {
    std::unique_ptr<SimulatedAgent> agent(new SimulatedAgent());
    agent->robot_id = i;
    agent->command = defaultCommand();
    agent->previous_control = SimulatorControl{{0.0, 0.0, 0.0, 0.0}};
    agent->next_odom_pub_time = ros::Time::now();

    Quadrotor::State state = agent->quad.getState();
    state.x = positions[static_cast<std::size_t>(i)];
    state.v = Eigen::Vector3d::Zero();
    state.R = Eigen::AngleAxisd(yaws[static_cast<std::size_t>(i)],
                                Eigen::Vector3d::UnitZ())
                  .toRotationMatrix();
    state.omega = Eigen::Vector3d::Zero();
    state.motor_rpm = Eigen::Array4d::Zero();
    if (start_at_hover) {
      const double hover_force =
          agent->quad.getMass() * agent->quad.getGravity();
      const double hover_rpm = std::sqrt(
          hover_force / (4.0 * agent->quad.getPropellerThrustCoefficient()));
      state.motor_rpm = Eigen::Array4d::Constant(hover_rpm);
    }
    agent->quad.setState(state);
    if (start_at_hover) {
      setHoverCommand(*agent, yaws[static_cast<std::size_t>(i)]);
    }

    agent->odom_message.header.frame_id = frame_id;
    agent->odom_message.child_frame_id = "uav_" + std::to_string(i) +
                                         "/base_link";
    agent->imu_message.header.frame_id = frame_id;
    agents_.push_back(std::move(agent));
  }

  for (const std::unique_ptr<SimulatedAgent>& owned_agent : agents_) {
    SimulatedAgent* agent = owned_agent.get();
    const std::string prefix = "/uav_" + std::to_string(agent->robot_id);
    agent->odom_pub =
        node.advertise<nav_msgs::Odometry>(prefix + "/sim/odom", 100);
    agent->imu_pub = node.advertise<sensor_msgs::Imu>(prefix + "/sim/imu", 10);
    agent->command_subscriber = node.subscribe<quadrotor_msgs::SO3Command>(
        prefix + "/so3_cmd", 100, &SimulatedAgent::commandCallback, agent,
        ros::TransportHints().tcpNoDelay());
    agent->force_subscriber = node.subscribe<geometry_msgs::Vector3>(
        prefix + "/force_disturbance", 100, &SimulatedAgent::forceCallback,
        agent, ros::TransportHints().tcpNoDelay());
    agent->moment_subscriber = node.subscribe<geometry_msgs::Vector3>(
        prefix + "/moment_disturbance", 100, &SimulatedAgent::momentCallback,
        agent, ros::TransportHints().tcpNoDelay());
  }
  return true;
}

void MultiQuadrotorSimulator::step(double dt) {
  for (const std::unique_ptr<SimulatedAgent>& owned_agent : agents_) {
    SimulatedAgent& agent = *owned_agent;
    SimulatorControl control = getControl(agent.quad, agent.command);
    for (int motor = 0; motor < 4; ++motor) {
      if (std::isnan(control.rpm[motor])) {
        control.rpm[motor] = agent.previous_control.rpm[motor];
      }
    }
    agent.previous_control = control;
    agent.quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2],
                        control.rpm[3]);
    agent.quad.setExternalForce(agent.disturbance.force);
    agent.quad.setExternalMoment(agent.disturbance.moment);
    agent.quad.step(dt);
  }
}

void MultiQuadrotorSimulator::publish(const ros::Time& stamp,
                                      const ros::Duration& period) {
  for (const std::unique_ptr<SimulatedAgent>& owned_agent : agents_) {
    SimulatedAgent& agent = *owned_agent;
    if (stamp >= agent.next_odom_pub_time) {
      agent.next_odom_pub_time += period;
      agent.odom_message.header.stamp = stamp;
      agent.imu_message.header.stamp = stamp;
      const Quadrotor::State state = agent.quad.getState();
      stateToOdomMsg(state, agent.odom_message);
      quadToImuMsg(agent.quad, agent.imu_message);
      agent.odom_pub.publish(agent.odom_message);
      agent.imu_pub.publish(agent.imu_message);
    }
  }
}

}  // namespace QuadrotorSimulator

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_quadrotor_simulator_so3");
  ros::NodeHandle node("~");

  int num_agents = 3;
  node.param("num_agents", num_agents, 3);
  if (num_agents != 1 && num_agents != 3) {
    ROS_FATAL("[B0_MULTI] num_agents must be exactly 1 or 3");
    return 1;
  }

  std::string frame_id = "world";
  node.param("frame_id", frame_id, frame_id);
  double simulation_rate = 1000.0;
  double odom_rate = 100.0;
  node.param("rate/simulation", simulation_rate, simulation_rate);
  node.param("rate/odom", odom_rate, odom_rate);
  bool start_at_hover = true;
  node.param("start_at_hover", start_at_hover, start_at_hover);
  if (!std::isfinite(simulation_rate) || !std::isfinite(odom_rate) ||
      simulation_rate <= 0.0 || odom_rate <= 0.0 ||
      odom_rate > simulation_rate || frame_id.empty()) {
    ROS_FATAL("[B0_MULTI] invalid frame or rate configuration");
    return 1;
  }

  std::vector<InitialState> initial_states;
  if (!readInitialStates(node, num_agents, initial_states)) {
    return 1;
  }
  std::vector<Eigen::Vector3d> positions;
  std::vector<double> yaws;
  positions.reserve(initial_states.size());
  yaws.reserve(initial_states.size());
  for (const InitialState& state : initial_states) {
    positions.push_back(state.position);
    yaws.push_back(state.yaw);
  }

  QuadrotorSimulator::MultiQuadrotorSimulator simulator;
  if (!simulator.initialize(node, num_agents, frame_id, start_at_hover,
                            positions, yaws)) {
    ROS_FATAL("[B0_MULTI] simulator initialization failed");
    return 1;
  }

  ros::Rate rate(simulation_rate);
  const double dt = 1.0 / simulation_rate;
  const ros::Duration odom_period(1.0 / odom_rate);
  while (node.ok()) {
    ros::spinOnce();
    simulator.step(dt);
    simulator.publish(ros::Time::now(), odom_period);
    rate.sleep();
  }
  return 0;
}
