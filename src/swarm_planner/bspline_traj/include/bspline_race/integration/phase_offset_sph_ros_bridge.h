#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <phase_offset_msgs/BetaSample.h>
#include <phase_offset_msgs/GCoordSample.h>

#include <cstdint>
#include <limits>
#include <mutex>
#include <string>

namespace bspline_race {
namespace integration {

// ROS/value-only boundary for the SPH coordination handoff.  This class does
// not evaluate navigation policy or hold any control state.
struct PhaseOffsetSphRosBridgeConfig {
  int robot_id = -1;
  std::string frame_id = "world";
  std::string beta_topic;
  std::string g_coord_topic;
  double g_coord_fresh_timeout = 0.10;
  double future_timestamp_tolerance = 0.02;

  bool validate(std::string* error = nullptr) const;
  void validateOrThrow() const;
};

enum class GCoordUpdateStatus {
  ACCEPTED_VALID = 0,
  ACCEPTED_INVALID = 1,
  REJECTED_RECEIPT_TIME = 2,
  REJECTED_ID = 3,
  REJECTED_FRAME = 4,
  REJECTED_SOURCE_TIME = 5,
  REJECTED_FUTURE = 6,
  REJECTED_ZERO_EPOCH = 7,
  REJECTED_LOWER_EPOCH = 8,
  REJECTED_DUPLICATE_SEQUENCE = 9,
  REJECTED_OUT_OF_ORDER_SEQUENCE = 10,
  REJECTED_SOURCE_REGRESSION = 11,
  kAcceptedValid = ACCEPTED_VALID,
  kAcceptedInvalid = ACCEPTED_INVALID,
  kRejectedReceiptTime = REJECTED_RECEIPT_TIME,
  kRejectedId = REJECTED_ID,
  kRejectedFrame = REJECTED_FRAME,
  kRejectedSourceTime = REJECTED_SOURCE_TIME,
  kRejectedFuture = REJECTED_FUTURE,
  kRejectedZeroEpoch = REJECTED_ZERO_EPOCH,
  kRejectedLowerEpoch = REJECTED_LOWER_EPOCH,
  kRejectedDuplicateSequence = REJECTED_DUPLICATE_SEQUENCE,
  kRejectedOutOfOrderSequence = REJECTED_OUT_OF_ORDER_SEQUENCE,
  kRejectedSourceRegression = REJECTED_SOURCE_REGRESSION
};

const char* gCoordUpdateStatusName(GCoordUpdateStatus status);

struct GCoordUpdateResult {
  GCoordUpdateStatus status = GCoordUpdateStatus::REJECTED_RECEIPT_TIME;
  bool retained = false;
  bool valid = false;

  bool accepted() const {
    return status == GCoordUpdateStatus::ACCEPTED_VALID ||
           status == GCoordUpdateStatus::ACCEPTED_INVALID;
  }
  operator GCoordUpdateStatus() const { return status; }
};

// Immutable value representation kept by the callback and copied by the
// caller-side capture.  Receipt time is local steady time and is not
// serialized on the wire.
struct GCoordSampleValue {
  int robot_id = -1;
  std::string frame_id;
  geometry_msgs::Vector3 g_coord;
  double source_stamp = 0.0;
  double receive_steady_time = 0.0;
  std::uint64_t sequence = 0;
  std::uint64_t producer_epoch = 0;
  bool valid = false;
};

struct GCoordCapture {
  bool has_sample = false;
  GCoordSampleValue sample;
};

enum class GCoordResolveStatus {
  VALID = 0,
  MISSING = 1,
  INVALID = 2,
  STALE_SOURCE = 3,
  STALE_RECEIPT = 4,
  INVALID_QUERY = 5,
  kValid = VALID,
  kMissing = MISSING,
  kInvalid = INVALID,
  kStaleSource = STALE_SOURCE,
  kStaleReceipt = STALE_RECEIPT,
  kInvalidQuery = INVALID_QUERY
};

const char* gCoordResolveStatusName(GCoordResolveStatus status);

struct GCoordResolveResult {
  GCoordResolveStatus status = GCoordResolveStatus::MISSING;
  GCoordSampleValue sample;

  bool usable() const { return status == GCoordResolveStatus::VALID; }
  operator GCoordResolveStatus() const { return status; }
};

class PhaseOffsetSphRosBridge {
 public:
  using Config = PhaseOffsetSphRosBridgeConfig;
  using UpdateStatus = GCoordUpdateStatus;
  using UpdateResult = GCoordUpdateResult;
  using Capture = GCoordCapture;
  using ResolveStatus = GCoordResolveStatus;
  using ResolveResult = GCoordResolveResult;

  explicit PhaseOffsetSphRosBridge(
      ros::NodeHandle& node,
      const PhaseOffsetSphRosBridgeConfig& config);

  PhaseOffsetSphRosBridge(
      ros::NodeHandle& node, ros::NodeHandle& /*private_node*/,
      const PhaseOffsetSphRosBridgeConfig& config)
      : PhaseOffsetSphRosBridge(node, config) {}

  PhaseOffsetSphRosBridge(const PhaseOffsetSphRosBridge&) = delete;
  PhaseOffsetSphRosBridge& operator=(const PhaseOffsetSphRosBridge&) = delete;

  // Publish exactly one value for a caller-provided source stamp.  Invalid or
  // non-finite values are emitted as valid=false with beta_i=0, and every
  // publication advances the sequence.
  bool publishBeta(const ros::Time& source_stamp, double beta_i, bool valid);
  bool publishValidBeta(const ros::Time& source_stamp, double beta_i) {
    return publishBeta(source_stamp, beta_i, true);
  }
  bool publishInvalidBeta(const ros::Time& source_stamp) {
    return publishBeta(source_stamp, 0.0, false);
  }
  bool publishBetaSample(const ros::Time& source_stamp, double beta_i,
                         bool valid) {
    return publishBeta(source_stamp, beta_i, valid);
  }

  // Public for deterministic tests and for callback adapters that already
  // own injected source/steady clocks.
  GCoordUpdateResult updateGCoord(
      const phase_offset_msgs::GCoordSample& message,
      double receive_steady_time, double source_now);
  GCoordUpdateResult ingestGCoord(
      const phase_offset_msgs::GCoordSample& message,
      double receive_steady_time, double source_now) {
    return updateGCoord(message, receive_steady_time, source_now);
  }

  GCoordCapture captureGCoord() const;
  GCoordCapture captureGCoordSample() const { return captureGCoord(); }
  GCoordCapture latestGCoord() const { return captureGCoord(); }
  GCoordResolveResult resolveGCoord(const GCoordCapture& capture,
                                    double source_query_time,
                                    double steady_query_time) const;
  GCoordResolveResult resolveGCoordSample(
      const GCoordCapture& capture, double source_query_time,
      double steady_query_time) const {
    return resolveGCoord(capture, source_query_time, steady_query_time);
  }

  bool hasGCoord() const;
  GCoordSampleValue retainedGCoord() const;

  const PhaseOffsetSphRosBridgeConfig& config() const { return config_; }
  std::uint64_t producerEpoch() const { return producer_epoch_; }
  std::uint64_t betaSequence() const;

 private:
  void gCoordCallback(
      const phase_offset_msgs::GCoordSampleConstPtr& message);
  static std::uint64_t processEpoch();

  PhaseOffsetSphRosBridgeConfig config_;
  ros::Publisher beta_publisher_;
  ros::Subscriber g_coord_subscriber_;

  const std::uint64_t producer_epoch_;
  mutable std::mutex beta_mutex_;
  std::uint64_t beta_sequence_ = 0;

  mutable std::mutex g_coord_mutex_;
  bool has_g_coord_ = false;
  GCoordSampleValue latest_g_coord_;
};

}  // namespace integration
}  // namespace bspline_race
