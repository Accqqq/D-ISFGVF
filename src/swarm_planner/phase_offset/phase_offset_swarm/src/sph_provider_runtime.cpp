#include "phase_offset_swarm/sph_provider_runtime.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace phase_offset_swarm {
namespace {

bool finite(double value) { return std::isfinite(value); }

SphProviderConfig validatedConfig(const SphProviderConfig& config) {
  config.validateOrThrow();
  return config;
}

double age(double now, double stamp) {
  if (!finite(now) || !finite(stamp)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return now - stamp;
}

}  // namespace

bool SphProviderConfig::validate(std::string* error) const {
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
  if (world_frame.empty()) {
    return fail("world_frame must be non-empty");
  }
  if (!finite(beta_fresh_timeout) || beta_fresh_timeout < 0.0) {
    return fail("beta_fresh_timeout must be finite and non-negative");
  }
  if (!finite(snapshot_fresh_timeout) || snapshot_fresh_timeout < 0.0) {
    return fail("snapshot_fresh_timeout must be finite and non-negative");
  }
  if (!finite(own_odom_timeout) || own_odom_timeout < 0.0) {
    return fail("own_odom_timeout must be finite and non-negative");
  }
  if (!finite(future_timestamp_tolerance) ||
      future_timestamp_tolerance < 0.0) {
    return fail("future_timestamp_tolerance must be finite and non-negative");
  }
  if (provider_epoch == 0u) {
    return fail("provider_epoch must be nonzero");
  }
  std::string sph_error;
  if (!sph_parameters.validate(&sph_error)) {
    return fail("invalid SphParameters: " + sph_error);
  }
  return true;
}

void SphProviderConfig::validateOrThrow() const {
  std::string error;
  if (!validate(&error)) {
    throw std::invalid_argument("invalid SphProviderConfig: " + error);
  }
}

BetaInputBuffer::BetaInputBuffer(const SphProviderConfig& config)
    : config_(validatedConfig(config)) {}

BetaInputUpdateResult BetaInputBuffer::update(const BetaSampleValue& sample,
                                              double receive_steady_time,
                                              double source_now) {
  BetaInputUpdateResult result;
  if (!finite(receive_steady_time) || receive_steady_time < 0.0 ||
      !finite(source_now) || source_now <= 0.0) {
    result.status = BetaInputUpdateStatus::REJECTED_RECEIPT_TIME;
    return result;
  }
  if (sample.robot_id != config_.robot_id) {
    result.status = BetaInputUpdateStatus::REJECTED_ID;
    return result;
  }
  if (sample.frame_id != config_.world_frame) {
    result.status = BetaInputUpdateStatus::REJECTED_FRAME;
    return result;
  }
  if (!finite(sample.source_stamp) || sample.source_stamp <= 0.0) {
    result.status = BetaInputUpdateStatus::REJECTED_SOURCE_TIME;
    return result;
  }
  if (sample.producer_epoch == 0u) {
    result.status = BetaInputUpdateStatus::REJECTED_ZERO_EPOCH;
    return result;
  }
  if (sample.source_stamp - source_now >
      config_.future_timestamp_tolerance) {
    result.status = BetaInputUpdateStatus::REJECTED_FUTURE;
    return result;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (has_retained_) {
    if (sample.producer_epoch < retained_.producer_epoch) {
      result.status = BetaInputUpdateStatus::REJECTED_LOWER_EPOCH;
      return result;
    }
    if (sample.producer_epoch == retained_.producer_epoch) {
      if (sample.sequence == retained_.sequence) {
        result.status = BetaInputUpdateStatus::REJECTED_DUPLICATE_SEQUENCE;
        return result;
      }
      if (sample.sequence < retained_.sequence) {
        result.status = BetaInputUpdateStatus::REJECTED_OUT_OF_ORDER_SEQUENCE;
        return result;
      }
      if (sample.source_stamp < retained_.source_stamp) {
        result.status = BetaInputUpdateStatus::REJECTED_SOURCE_REGRESSION;
        return result;
      }
    }
  }

  retained_ = sample;
  const bool valid_payload = sample.valid && finite(sample.beta_i) &&
                             sample.beta_i >= 0.0 && sample.beta_i <= 1.0;
  if (!valid_payload) {
    retained_.beta_i = 0.0;
    retained_.valid = false;
  } else {
    retained_.valid = true;
  }
  retained_receive_steady_time_ = receive_steady_time;
  has_retained_ = true;
  result.status = valid_payload ? BetaInputUpdateStatus::ACCEPTED_VALID
                                : BetaInputUpdateStatus::ACCEPTED_INVALID;
  result.retained = true;
  result.valid = valid_payload;
  return result;
}

BetaResolveResult BetaInputBuffer::resolve(double provider_source_time,
                                           double provider_steady_time) const {
  BetaResolveResult result;
  if (!finite(provider_source_time) || provider_source_time <= 0.0 ||
      !finite(provider_steady_time) || provider_steady_time < 0.0) {
    result.status = BetaResolveStatus::INVALID_QUERY;
    return result;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_retained_) {
    result.status = BetaResolveStatus::MISSING;
    return result;
  }
  result.sample = retained_;
  if (!retained_.valid) {
    result.status = BetaResolveStatus::INVALID;
    return result;
  }
  const double source_age = age(provider_source_time, retained_.source_stamp);
  if (!finite(source_age) || source_age < 0.0 ||
      source_age > config_.beta_fresh_timeout) {
    result.status = BetaResolveStatus::STALE_SOURCE;
    return result;
  }
  const double receive_age =
      age(provider_steady_time, retained_receive_steady_time_);
  if (!finite(receive_age) || receive_age < 0.0 ||
      receive_age > config_.beta_fresh_timeout) {
    result.status = BetaResolveStatus::STALE_RECEIPT;
    return result;
  }
  result.status = BetaResolveStatus::VALID;
  return result;
}

bool BetaInputBuffer::hasRetained() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_retained_;
}

BetaSampleValue BetaInputBuffer::retainedSample() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return retained_;
}

double BetaInputBuffer::retainedReceiveSteadyTime() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return retained_receive_steady_time_;
}

SphProviderRuntime::SphProviderRuntime(const SphProviderConfig& config)
    : config_(validatedConfig(config)),
      s0_provider_(config_.sph_parameters),
      beta_buffer_(config_) {}

bool SphProviderRuntime::validEnvelope(
    const SwarmAgentState& self, double self_receive_steady_time,
    const NeighborSnapshot& snapshot, double provider_source_time,
    double provider_steady_time, double* beta_value) const {
  if (!finite(provider_source_time) || provider_source_time <= 0.0 ||
      !finite(provider_steady_time) || provider_steady_time < 0.0) {
    return false;
  }
  if (self.id != config_.robot_id || !self.isValid() ||
      self.frame_id != config_.world_frame || !finite(self.stamp) ||
      self.stamp <= 0.0) {
    return false;
  }
  const double self_source_age = age(provider_source_time, self.stamp);
  const double self_receive_age =
      age(provider_steady_time, self_receive_steady_time);
  if (!finite(self_source_age) || self_source_age < 0.0 ||
      self_source_age > config_.own_odom_timeout ||
      !finite(self_receive_age) || self_receive_age < 0.0 ||
      self_receive_age > config_.own_odom_timeout) {
    return false;
  }
  if (snapshot.self_id != config_.robot_id) {
    return false;
  }
  if (!finite(snapshot.source_query_time) ||
      !finite(snapshot.query_source_time) ||
      snapshot.source_query_time <= 0.0 ||
      snapshot.query_source_time <= 0.0 ||
      snapshot.source_query_time != snapshot.query_source_time) {
    return false;
  }
  if (!finite(snapshot.receive_query_time) ||
      !finite(snapshot.query_receive_time) ||
      snapshot.receive_query_time < 0.0 ||
      snapshot.query_receive_time < 0.0 ||
      snapshot.receive_query_time != snapshot.query_receive_time) {
    return false;
  }
  const double snapshot_source_age =
      age(provider_source_time, snapshot.source_query_time);
  const double snapshot_receive_age =
      age(provider_steady_time, snapshot.receive_query_time);
  if (!finite(snapshot_source_age) || snapshot_source_age < 0.0 ||
      snapshot_source_age > config_.snapshot_fresh_timeout ||
      !finite(snapshot_receive_age) || snapshot_receive_age < 0.0 ||
      snapshot_receive_age > config_.snapshot_fresh_timeout) {
    return false;
  }

  const BetaResolveResult beta =
      beta_buffer_.resolve(provider_source_time, provider_steady_time);
  if (!beta.usable() || !finite(beta.sample.beta_i) ||
      beta.sample.beta_i < 0.0 || beta.sample.beta_i > 1.0) {
    return false;
  }
  if (beta_value != nullptr) {
    *beta_value = beta.sample.beta_i;
  }
  return true;
}

GCoordSampleValue SphProviderRuntime::invalidSample(
    double provider_source_time, std::uint64_t sequence) const {
  GCoordSampleValue sample;
  sample.robot_id = config_.robot_id;
  sample.g_coord = Eigen::Vector3d::Zero();
  sample.provider_stamp = provider_source_time;
  sample.sequence = sequence;
  sample.producer_epoch = config_.provider_epoch;
  sample.frame_id = config_.world_frame;
  sample.valid = false;
  return sample;
}

SphProviderEvaluation SphProviderRuntime::evaluate(
    const SwarmAgentState& self, double self_receive_steady_time,
    const NeighborSnapshot& snapshot, double provider_source_time,
    double provider_steady_time) {
  const std::uint64_t sequence = ++output_sequence_;
  SphProviderEvaluation evaluation;
  evaluation.sample = invalidSample(provider_source_time, sequence);
  double beta_value = 0.0;
  if (!validEnvelope(self, self_receive_steady_time, snapshot,
                     provider_source_time, provider_steady_time,
                     &beta_value)) {
    evaluation.status = SphProviderEvaluationStatus::INVALID_PROVIDER_ENVELOPE;
    return evaluation;
  }

  const SphIntentOutput s0 =
      s0_provider_.compute(self, snapshot, beta_value);
  evaluation.s0_invoked = true;
  evaluation.s0_status = s0.status;
  if (s0.status == SphIntentStatus::VALID && s0.valid &&
      s0.g_coord.allFinite()) {
    evaluation.status = SphProviderEvaluationStatus::VALID;
    evaluation.sample.g_coord =
        Eigen::Vector3d(s0.g_coord.x(), s0.g_coord.y(), 0.0);
    evaluation.sample.valid = true;
    return evaluation;
  }

  evaluation.status = SphProviderEvaluationStatus::INVALID_S0_RESULT;
  return evaluation;
}

}  // namespace phase_offset_swarm
