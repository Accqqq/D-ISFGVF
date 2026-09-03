#include "bspline_race/integration/phase_offset_sph_ros_bridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace bspline_race {
namespace integration {
namespace {

bool finite(double value) { return std::isfinite(value); }

bool finiteVector(const geometry_msgs::Vector3& value) {
  return finite(value.x) && finite(value.y) && finite(value.z);
}

geometry_msgs::Vector3 zeroVector() {
  geometry_msgs::Vector3 value;
  value.x = 0.0;
  value.y = 0.0;
  value.z = 0.0;
  return value;
}

}  // namespace

bool PhaseOffsetSphRosBridgeConfig::validate(std::string* error) const {
  if (error != nullptr) {
    error->clear();
  }
  const auto fail = [error](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  if (robot_id < 0 || robot_id > 65535) {
    return fail("robot_id must be in [0, 65535]");
  }
  if (frame_id.empty()) {
    return fail("frame_id must be non-empty");
  }
  if (beta_topic.empty() || g_coord_topic.empty()) {
    return fail("beta_topic and g_coord_topic must be non-empty");
  }
  if (!finite(g_coord_fresh_timeout) || g_coord_fresh_timeout < 0.0) {
    return fail("g_coord_fresh_timeout must be finite and non-negative");
  }
  if (!finite(future_timestamp_tolerance) ||
      future_timestamp_tolerance < 0.0) {
    return fail(
        "future_timestamp_tolerance must be finite and non-negative");
  }
  return true;
}

void PhaseOffsetSphRosBridgeConfig::validateOrThrow() const {
  std::string error;
  if (!validate(&error)) {
    throw std::invalid_argument(
        "invalid PhaseOffsetSphRosBridgeConfig: " + error);
  }
}

const char* gCoordUpdateStatusName(GCoordUpdateStatus status) {
  switch (status) {
    case GCoordUpdateStatus::ACCEPTED_VALID:
      return "ACCEPTED_VALID";
    case GCoordUpdateStatus::ACCEPTED_INVALID:
      return "ACCEPTED_INVALID";
    case GCoordUpdateStatus::REJECTED_RECEIPT_TIME:
      return "REJECTED_RECEIPT_TIME";
    case GCoordUpdateStatus::REJECTED_ID:
      return "REJECTED_ID";
    case GCoordUpdateStatus::REJECTED_FRAME:
      return "REJECTED_FRAME";
    case GCoordUpdateStatus::REJECTED_SOURCE_TIME:
      return "REJECTED_SOURCE_TIME";
    case GCoordUpdateStatus::REJECTED_FUTURE:
      return "REJECTED_FUTURE";
    case GCoordUpdateStatus::REJECTED_ZERO_EPOCH:
      return "REJECTED_ZERO_EPOCH";
    case GCoordUpdateStatus::REJECTED_LOWER_EPOCH:
      return "REJECTED_LOWER_EPOCH";
    case GCoordUpdateStatus::REJECTED_DUPLICATE_SEQUENCE:
      return "REJECTED_DUPLICATE_SEQUENCE";
    case GCoordUpdateStatus::REJECTED_OUT_OF_ORDER_SEQUENCE:
      return "REJECTED_OUT_OF_ORDER_SEQUENCE";
    case GCoordUpdateStatus::REJECTED_SOURCE_REGRESSION:
      return "REJECTED_SOURCE_REGRESSION";
  }
  return "UNKNOWN";
}

const char* gCoordResolveStatusName(GCoordResolveStatus status) {
  switch (status) {
    case GCoordResolveStatus::VALID:
      return "VALID";
    case GCoordResolveStatus::MISSING:
      return "MISSING";
    case GCoordResolveStatus::INVALID:
      return "INVALID";
    case GCoordResolveStatus::STALE_SOURCE:
      return "STALE_SOURCE";
    case GCoordResolveStatus::STALE_RECEIPT:
      return "STALE_RECEIPT";
    case GCoordResolveStatus::INVALID_QUERY:
      return "INVALID_QUERY";
  }
  return "UNKNOWN";
}

PhaseOffsetSphRosBridge::PhaseOffsetSphRosBridge(
    ros::NodeHandle& node, const PhaseOffsetSphRosBridgeConfig& config)
    : config_(config), producer_epoch_(processEpoch()) {
  if (config_.beta_topic.empty() && config_.robot_id >= 0) {
    config_.beta_topic = "/uav_" + std::to_string(config_.robot_id) +
                         "/phase_offset/sph_beta";
  }
  if (config_.g_coord_topic.empty() && config_.robot_id >= 0) {
    config_.g_coord_topic = "/uav_" + std::to_string(config_.robot_id) +
                            "/phase_offset/g_coord";
  }
  config_.validateOrThrow();

  beta_publisher_ = node.advertise<phase_offset_msgs::BetaSample>(
      config_.beta_topic, 10, false);
  g_coord_subscriber_ = node.subscribe(
      config_.g_coord_topic, 20,
      &PhaseOffsetSphRosBridge::gCoordCallback, this);
}

bool PhaseOffsetSphRosBridge::publishBeta(const ros::Time& source_stamp,
                                          double beta_i, bool valid) {
  std::lock_guard<std::mutex> lock(beta_mutex_);
  if (beta_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    ROS_ERROR("PhaseOffsetSphRosBridge beta sequence exhausted");
    return false;
  }

  ++beta_sequence_;
  phase_offset_msgs::BetaSample message;
  message.header.stamp = source_stamp;
  message.header.frame_id = config_.frame_id;
  message.robot_id = static_cast<std::uint16_t>(config_.robot_id);
  const bool valid_payload = valid && finite(beta_i) && beta_i >= 0.0 &&
                             beta_i <= 1.0;
  message.beta_i = valid_payload ? beta_i : 0.0;
  message.sequence = beta_sequence_;
  message.producer_epoch = producer_epoch_;
  message.valid = valid_payload;
  beta_publisher_.publish(message);
  return true;
}

GCoordUpdateResult PhaseOffsetSphRosBridge::updateGCoord(
    const phase_offset_msgs::GCoordSample& message,
    double receive_steady_time, double source_now) {
  GCoordUpdateResult result;
  if (!finite(receive_steady_time) || receive_steady_time < 0.0 ||
      !finite(source_now) || source_now <= 0.0) {
    result.status = GCoordUpdateStatus::REJECTED_RECEIPT_TIME;
    return result;
  }
  if (static_cast<int>(message.robot_id) != config_.robot_id) {
    result.status = GCoordUpdateStatus::REJECTED_ID;
    return result;
  }
  if (message.header.frame_id != config_.frame_id) {
    result.status = GCoordUpdateStatus::REJECTED_FRAME;
    return result;
  }

  const double source_stamp = message.header.stamp.toSec();
  if (!finite(source_stamp) || source_stamp <= 0.0) {
    result.status = GCoordUpdateStatus::REJECTED_SOURCE_TIME;
    return result;
  }
  if (source_stamp - source_now > config_.future_timestamp_tolerance) {
    result.status = GCoordUpdateStatus::REJECTED_FUTURE;
    return result;
  }
  if (message.producer_epoch == 0u) {
    result.status = GCoordUpdateStatus::REJECTED_ZERO_EPOCH;
    return result;
  }

  const bool valid_payload = message.valid && finiteVector(message.g_coord);
  GCoordSampleValue candidate;
  candidate.robot_id = static_cast<int>(message.robot_id);
  candidate.frame_id = message.header.frame_id;
  candidate.g_coord = valid_payload ? message.g_coord : zeroVector();
  candidate.source_stamp = source_stamp;
  candidate.receive_steady_time = receive_steady_time;
  candidate.sequence = message.sequence;
  candidate.producer_epoch = message.producer_epoch;
  candidate.valid = valid_payload;

  std::lock_guard<std::mutex> lock(g_coord_mutex_);
  if (has_g_coord_) {
    if (candidate.producer_epoch < latest_g_coord_.producer_epoch) {
      result.status = GCoordUpdateStatus::REJECTED_LOWER_EPOCH;
      return result;
    }
    if (candidate.producer_epoch == latest_g_coord_.producer_epoch) {
      if (candidate.sequence == latest_g_coord_.sequence) {
        result.status = GCoordUpdateStatus::REJECTED_DUPLICATE_SEQUENCE;
        return result;
      }
      if (candidate.sequence < latest_g_coord_.sequence) {
        result.status = GCoordUpdateStatus::REJECTED_OUT_OF_ORDER_SEQUENCE;
        return result;
      }
      if (candidate.source_stamp < latest_g_coord_.source_stamp) {
        result.status = GCoordUpdateStatus::REJECTED_SOURCE_REGRESSION;
        return result;
      }
    }
  }

  latest_g_coord_ = candidate;
  has_g_coord_ = true;
  result.status = valid_payload ? GCoordUpdateStatus::ACCEPTED_VALID
                                : GCoordUpdateStatus::ACCEPTED_INVALID;
  result.retained = true;
  result.valid = valid_payload;
  return result;
}

GCoordCapture PhaseOffsetSphRosBridge::captureGCoord() const {
  std::lock_guard<std::mutex> lock(g_coord_mutex_);
  GCoordCapture capture;
  capture.has_sample = has_g_coord_;
  if (has_g_coord_) {
    capture.sample = latest_g_coord_;
  }
  return capture;
}

GCoordResolveResult PhaseOffsetSphRosBridge::resolveGCoord(
    const GCoordCapture& capture, double source_query_time,
    double steady_query_time) const {
  GCoordResolveResult result;
  if (!finite(source_query_time) || source_query_time <= 0.0 ||
      !finite(steady_query_time) || steady_query_time < 0.0) {
    result.status = GCoordResolveStatus::INVALID_QUERY;
    return result;
  }
  if (!capture.has_sample) {
    result.status = GCoordResolveStatus::MISSING;
    return result;
  }

  result.sample = capture.sample;
  if (!result.sample.valid || result.sample.robot_id != config_.robot_id ||
      result.sample.frame_id != config_.frame_id ||
      result.sample.producer_epoch == 0u ||
      !finite(result.sample.source_stamp) ||
      result.sample.source_stamp <= 0.0 ||
      !finite(result.sample.receive_steady_time) ||
      result.sample.receive_steady_time < 0.0 ||
      !finiteVector(result.sample.g_coord)) {
    result.status = GCoordResolveStatus::INVALID;
    return result;
  }

  const double source_age = source_query_time - result.sample.source_stamp;
  if (!finite(source_age) || source_age < 0.0 ||
      source_age > config_.g_coord_fresh_timeout) {
    result.status = GCoordResolveStatus::STALE_SOURCE;
    return result;
  }
  const double receipt_age =
      steady_query_time - result.sample.receive_steady_time;
  if (!finite(receipt_age) || receipt_age < 0.0 ||
      receipt_age > config_.g_coord_fresh_timeout) {
    result.status = GCoordResolveStatus::STALE_RECEIPT;
    return result;
  }
  result.status = GCoordResolveStatus::VALID;
  return result;
}

std::uint64_t PhaseOffsetSphRosBridge::betaSequence() const {
  std::lock_guard<std::mutex> lock(beta_mutex_);
  return beta_sequence_;
}

bool PhaseOffsetSphRosBridge::hasGCoord() const {
  std::lock_guard<std::mutex> lock(g_coord_mutex_);
  return has_g_coord_;
}

GCoordSampleValue PhaseOffsetSphRosBridge::retainedGCoord() const {
  std::lock_guard<std::mutex> lock(g_coord_mutex_);
  return latest_g_coord_;
}

void PhaseOffsetSphRosBridge::gCoordCallback(
    const phase_offset_msgs::GCoordSampleConstPtr& message) {
  if (!message) {
    return;
  }
  const GCoordUpdateResult result = updateGCoord(
      *message, ros::SteadyTime::now().toSec(), ros::Time::now().toSec());
  if (!result.accepted()) {
    ROS_WARN_THROTTLE(
        2.0, "SPH GCoord sample rejected: status=%s",
        gCoordUpdateStatusName(result.status));
  }
}

std::uint64_t PhaseOffsetSphRosBridge::processEpoch() {
  static const std::uint64_t process_epoch = []() {
    const std::uint64_t wall_epoch = ros::WallTime::now().toNSec();
    if (wall_epoch != 0u) {
      return wall_epoch;
    }
    const std::uint64_t steady_epoch = static_cast<std::uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    return steady_epoch == 0u ? 1u : steady_epoch;
  }();
  return process_epoch;
}

}  // namespace integration
}  // namespace bspline_race
