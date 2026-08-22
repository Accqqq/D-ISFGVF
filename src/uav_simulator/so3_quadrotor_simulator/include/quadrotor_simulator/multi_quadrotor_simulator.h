#ifndef QUADROTOR_SIMULATOR_MULTI_QUADROTOR_SIMULATOR_H_
#define QUADROTOR_SIMULATOR_MULTI_QUADROTOR_SIMULATOR_H_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_simulator/Quadrotor.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <uav_utils/geometry_utils.h>

namespace QuadrotorSimulator {

struct SimulatorControl {
  double rpm[4];
};

struct SimulatorCommand {
  float force[3];
  float qx;
  float qy;
  float qz;
  float qw;
  float kR[3];
  float kOm[3];
  float corrections[3];
  float current_yaw;
  bool use_external_yaw;
};

struct SimulatorDisturbance {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  Eigen::Vector3d moment = Eigen::Vector3d::Zero();
};

inline SimulatorControl getControl(const Quadrotor& quad,
                                   const SimulatorCommand& command) {
  const double _kf = quad.getPropellerThrustCoefficient();
  const double _km = quad.getPropellerMomentCoefficient();
  const double kf = _kf - command.corrections[0];
  const double km = _km / _kf * kf;
  const double d = quad.getArmLength();
  const Eigen::Matrix3f inertia = quad.getInertia().cast<float>();
  const float I[3][3] = {{inertia(0, 0), inertia(0, 1), inertia(0, 2)},
                         {inertia(1, 0), inertia(1, 1), inertia(1, 2)},
                         {inertia(2, 0), inertia(2, 1), inertia(2, 2)}};
  const Quadrotor::State state = quad.getState();

  Eigen::Vector3d ypr = uav_utils::R_to_ypr(state.R);
  if (command.use_external_yaw) {
    ypr[0] = command.current_yaw;
  }
  const Eigen::Matrix3d R =
      (Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  const float R11 = static_cast<float>(R(0, 0));
  const float R12 = static_cast<float>(R(0, 1));
  const float R13 = static_cast<float>(R(0, 2));
  const float R21 = static_cast<float>(R(1, 0));
  const float R22 = static_cast<float>(R(1, 1));
  const float R23 = static_cast<float>(R(1, 2));
  const float R31 = static_cast<float>(R(2, 0));
  const float R32 = static_cast<float>(R(2, 1));
  const float R33 = static_cast<float>(R(2, 2));
  const float Om1 = static_cast<float>(state.omega(0));
  const float Om2 = static_cast<float>(state.omega(1));
  const float Om3 = static_cast<float>(state.omega(2));

  const float Rd11 = command.qw * command.qw + command.qx * command.qx -
                     command.qy * command.qy - command.qz * command.qz;
  const float Rd12 = 2.0f * (command.qx * command.qy -
                             command.qw * command.qz);
  const float Rd13 = 2.0f * (command.qx * command.qz +
                             command.qw * command.qy);
  const float Rd21 = 2.0f * (command.qx * command.qy +
                             command.qw * command.qz);
  const float Rd22 = command.qw * command.qw - command.qx * command.qx +
                     command.qy * command.qy - command.qz * command.qz;
  const float Rd23 = 2.0f * (command.qy * command.qz -
                             command.qw * command.qx);
  const float Rd31 = 2.0f * (command.qx * command.qz -
                             command.qw * command.qy);
  const float Rd32 = 2.0f * (command.qy * command.qz +
                             command.qw * command.qx);
  const float Rd33 = command.qw * command.qw - command.qx * command.qx -
                     command.qy * command.qy + command.qz * command.qz;

  const float Psi =
      0.5f * (3.0f - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 +
                      Rd12 * R12 + Rd22 * R22 + Rd32 * R32 +
                      Rd13 * R13 + Rd23 * R23 + Rd33 * R33));

  float force = 0.0f;
  if (Psi < 1.0f) {
    force = command.force[0] * R13 + command.force[1] * R23 +
            command.force[2] * R33;
  }

  const float eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 -
                             R23 * Rd22 + R32 * Rd33 - R33 * Rd32);
  const float eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 +
                             R23 * Rd21 - R31 * Rd33 + R33 * Rd31);
  const float eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 -
                             R22 * Rd21 + R31 * Rd32 - R32 * Rd31);

  const float in1 = Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) -
                    Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
  const float in2 = Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) -
                    Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
  const float in3 = Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) -
                    Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);

  const float M1 = -command.kR[0] * eR1 - command.kOm[0] * Om1 + in1;
  const float M2 = -command.kR[1] * eR2 - command.kOm[1] * Om2 + in2;
  const float M3 = -command.kR[2] * eR3 - command.kOm[2] * Om3 + in3;

  float w_sq[4];
  w_sq[0] = force / (4.0f * kf) - M2 / (2.0f * d * kf) + M3 / (4.0f * km);
  w_sq[1] = force / (4.0f * kf) + M2 / (2.0f * d * kf) + M3 / (4.0f * km);
  w_sq[2] = force / (4.0f * kf) + M1 / (2.0f * d * kf) - M3 / (4.0f * km);
  w_sq[3] = force / (4.0f * kf) - M1 / (2.0f * d * kf) - M3 / (4.0f * km);

  SimulatorControl control;
  for (int i = 0; i < 4; ++i) {
    if (w_sq[i] < 0.0f) {
      w_sq[i] = 0.0f;
    }
    control.rpm[i] = std::sqrt(w_sq[i]);
  }
  return control;
}

inline void stateToOdomMsg(const Quadrotor::State& state,
                           nav_msgs::Odometry& odom) {
  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);
  const Eigen::Quaterniond q(state.R);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = state.v(0);
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = state.v(2);
  odom.twist.twist.angular.x = state.omega(0);
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);
}

inline void quadToImuMsg(const Quadrotor& quad, sensor_msgs::Imu& imu) {
  const Quadrotor::State state = quad.getState();
  const Eigen::Quaterniond q(state.R);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();
  imu.angular_velocity.x = state.omega(0);
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);
  imu.linear_acceleration.x = quad.getAcc()[0];
  imu.linear_acceleration.y = quad.getAcc()[1];
  imu.linear_acceleration.z = quad.getAcc()[2];
}

struct SimulatedAgent {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int robot_id = 0;
  Quadrotor quad;
  SimulatorCommand command{};
  SimulatorDisturbance disturbance;
  SimulatorControl previous_control{{0.0, 0.0, 0.0, 0.0}};
  ros::Time next_odom_pub_time;
  ros::Publisher odom_pub;
  ros::Publisher imu_pub;
  ros::Subscriber command_subscriber;
  ros::Subscriber force_subscriber;
  ros::Subscriber moment_subscriber;
  nav_msgs::Odometry odom_message;
  sensor_msgs::Imu imu_message;

  void commandCallback(const quadrotor_msgs::SO3Command::ConstPtr& message);
  void forceCallback(const geometry_msgs::Vector3::ConstPtr& message);
  void momentCallback(const geometry_msgs::Vector3::ConstPtr& message);
};

class MultiQuadrotorSimulator {
 public:
  bool initialize(ros::NodeHandle& node, int num_agents,
                  const std::string& frame_id, bool start_at_hover,
                  const std::vector<Eigen::Vector3d>& positions,
                  const std::vector<double>& yaws);

  void step(double dt);
  void publish(const ros::Time& stamp, const ros::Duration& period);
  std::size_t numAgents() const { return agents_.size(); }

 private:
  std::vector<std::unique_ptr<SimulatedAgent>> agents_;
};

}  // namespace QuadrotorSimulator

#endif  // QUADROTOR_SIMULATOR_MULTI_QUADROTOR_SIMULATOR_H_
