#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <mutex>
#include <string>

#include "phase_offset_swarm/agent_state.h"
#include "phase_offset_swarm/neighbor_state.h"
#include "phase_offset_swarm/sph_intent.h"

namespace phase_offset_swarm {

struct SphProviderConfig {
  int robot_id = -1;
  std::string world_frame = "world";
  double beta_fresh_timeout = 0.10;
  double snapshot_fresh_timeout = 0.10;
  double own_odom_timeout = 0.20;
  double future_timestamp_tolerance = 0.02;
  std::uint64_t provider_epoch = 0;
  SphParameters sph_parameters;

  bool validate(std::string* error = nullptr) const;
  void validateOrThrow() const;
};

constexpr double kSphProviderPeriodSec = 0.05;

struct BetaSampleValue {
  int robot_id = -1;
  std::string frame_id;
  double beta_i = 0.0;
  double source_stamp = 0.0;
  std::uint64_t sequence = 0;
  std::uint64_t producer_epoch = 0;
  bool valid = false;
};

struct GCoordSampleValue {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int robot_id = -1;
  Eigen::Vector3d g_coord = Eigen::Vector3d::Zero();
  double provider_stamp = 0.0;
  std::uint64_t sequence = 0;
  std::uint64_t producer_epoch = 0;
  std::string frame_id;
  bool valid = false;
};

enum class BetaInputUpdateStatus {
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

struct BetaInputUpdateResult {
  BetaInputUpdateStatus status = BetaInputUpdateStatus::REJECTED_RECEIPT_TIME;
  bool retained = false;
  bool valid = false;

  bool accepted() const {
    return status == BetaInputUpdateStatus::ACCEPTED_VALID ||
           status == BetaInputUpdateStatus::ACCEPTED_INVALID;
  }
  operator BetaInputUpdateStatus() const { return status; }
};

enum class BetaResolveStatus {
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

struct BetaResolveResult {
  BetaResolveStatus status = BetaResolveStatus::MISSING;
  BetaSampleValue sample;

  bool usable() const { return status == BetaResolveStatus::VALID; }
  operator BetaResolveStatus() const { return status; }
};

class BetaInputBuffer {
 public:
  explicit BetaInputBuffer(const SphProviderConfig& config);

  BetaInputUpdateResult update(const BetaSampleValue& sample,
                               double receive_steady_time,
                               double source_now);
  BetaInputUpdateResult receive(const BetaSampleValue& sample,
                                double receive_steady_time,
                                double source_now) {
    return update(sample, receive_steady_time, source_now);
  }
  BetaInputUpdateResult updateBeta(const BetaSampleValue& sample,
                                   double receive_steady_time,
                                   double source_now) {
    return update(sample, receive_steady_time, source_now);
  }

  BetaResolveResult resolve(double provider_source_time,
                            double provider_steady_time) const;
  BetaResolveResult read(double provider_source_time,
                         double provider_steady_time) const {
    return resolve(provider_source_time, provider_steady_time);
  }

  bool hasRetained() const;
  BetaSampleValue retainedSample() const;
  double retainedReceiveSteadyTime() const;

 private:
  SphProviderConfig config_;
  mutable std::mutex mutex_;
  bool has_retained_ = false;
  BetaSampleValue retained_;
  double retained_receive_steady_time_ = 0.0;
};

enum class SphProviderEvaluationStatus {
  VALID = 0,
  INVALID_PROVIDER_ENVELOPE = 1,
  INVALID_S0_RESULT = 2,
  kValid = VALID,
  kInvalidProviderEnvelope = INVALID_PROVIDER_ENVELOPE,
  kInvalidS0Result = INVALID_S0_RESULT
};

struct SphProviderEvaluation {
  GCoordSampleValue sample;
  SphProviderEvaluationStatus status =
      SphProviderEvaluationStatus::INVALID_PROVIDER_ENVELOPE;
  bool s0_invoked = false;
  SphIntentStatus s0_status = SphIntentStatus::INVALID_SELF;
};

class SphProviderRuntime {
 public:
  explicit SphProviderRuntime(const SphProviderConfig& config);

  SphProviderEvaluation evaluate(const SwarmAgentState& self,
                                 double self_receive_steady_time,
                                 const NeighborSnapshot& snapshot,
                                 double provider_source_time,
                                 double provider_steady_time);

  BetaInputBuffer& betaBuffer() { return beta_buffer_; }
  const BetaInputBuffer& betaBuffer() const { return beta_buffer_; }
  BetaInputBuffer& beta_buffer() { return beta_buffer_; }
  const BetaInputBuffer& beta_buffer() const { return beta_buffer_; }

  const SphProviderConfig& config() const { return config_; }
  std::uint64_t outputSequence() const { return output_sequence_; }

 private:
  bool validEnvelope(const SwarmAgentState& self,
                    double self_receive_steady_time,
                    const NeighborSnapshot& snapshot,
                    double provider_source_time,
                    double provider_steady_time,
                    double* beta_value) const;
  GCoordSampleValue invalidSample(double provider_source_time,
                                  std::uint64_t sequence) const;

  SphProviderConfig config_;
  SphIntentProvider s0_provider_;
  BetaInputBuffer beta_buffer_;
  std::uint64_t output_sequence_ = 0;
};

}  // namespace phase_offset_swarm
