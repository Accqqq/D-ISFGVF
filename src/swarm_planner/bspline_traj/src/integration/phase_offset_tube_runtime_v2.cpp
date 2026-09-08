#include "bspline_race/integration/phase_offset_tube_runtime_v2.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <cmath>
#include <exception>
#include <limits>
#include <mutex>
#include <thread>
#include <utility>

namespace FLAG_Race {
namespace {

using Clock = std::chrono::steady_clock;

std::uint64_t steadyTicks() {
  const Clock::time_point now = Clock::now();
  const std::chrono::nanoseconds ticks =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          now.time_since_epoch());
  return ticks.count() < 0 ? 0U : static_cast<std::uint64_t>(ticks.count());
}

bool finite(const double value) { return std::isfinite(value); }

bool samePurpose(const TubeWorkerPurposeV2 lhs,
                 const TubeWorkerPurposeV2 rhs) {
  return lhs == rhs;
}

bool validPurpose(const TubeWorkerPurposeV2 purpose) {
  return purpose == TubeWorkerPurposeV2::CURRENT ||
      purpose == TubeWorkerPurposeV2::SUCCESSOR;
}

bool sameAuthority(const TubeWorkerRequestV2& lhs,
                   const TubeWorkerRequestV2& rhs) {
  return lhs.execution_generation == rhs.execution_generation &&
      lhs.input.path_key == rhs.input.path_key &&
      lhs.input.configuration_key == rhs.input.configuration_key &&
      lhs.input.map_capture_key == rhs.input.map_capture_key;
}

bool sameAuthority(const TubeWorkerCompletionV2& lhs,
                   const TubeWorkerCompletionV2& rhs) {
  return lhs.execution_generation == rhs.execution_generation &&
      lhs.path_key == rhs.path_key &&
      lhs.configuration_key == rhs.configuration_key &&
      lhs.map_capture_key == rhs.map_capture_key;
}

bool forwardDomainMatches(const phase_offset_navigation::TubeBuildInputV2& a,
                          const phase_offset_navigation::TubeBuildInputV2& b) {
  return a.path_key == b.path_key && a.configuration_key == b.configuration_key &&
      a.map_capture_key.map_instance_id == b.map_capture_key.map_instance_id &&
      a.map_capture_key.configuration_generation == b.map_capture_key.configuration_generation &&
      a.map_capture_key.configuration_id == b.map_capture_key.configuration_id &&
      a.map_capture_key.frame_provenance == b.map_capture_key.frame_provenance;
}
bool forwardDomainMatches(const TubeWorkerRequestV2& a, const TubeWorkerRequestV2& b) {
  return forwardDomainMatches(a.input, b.input);
}
bool forwardDomainMatches(const TubeWorkerCompletionV2& a, const TubeWorkerCompletionV2& b) {
  return a.path_key == b.path_key && a.configuration_key == b.configuration_key &&
      a.map_capture_key.map_instance_id == b.map_capture_key.map_instance_id &&
      a.map_capture_key.configuration_generation == b.map_capture_key.configuration_generation &&
      a.map_capture_key.configuration_id == b.map_capture_key.configuration_id &&
      a.map_capture_key.frame_provenance == b.map_capture_key.frame_provenance;
}

template <typename Value>
bool coverageDominates(const Value& incoming, const Value& retained) {
  if (incoming.execution_generation != retained.execution_generation ||
      !samePurpose(incoming.purpose, retained.purpose) ||
      !finite(incoming.useful_start) || !finite(incoming.useful_end) ||
      !finite(retained.useful_start) || !finite(retained.useful_end)) {
    return false;
  }
  // A request with a newer path/config/map key is a replacement only when it
  // also carries a newer accepted-state demand.  Otherwise a mere request ID
  // cannot erase useful retained proof.
  if (!sameAuthority(incoming, retained)) {
    const bool forward_extension = forwardDomainMatches(incoming, retained) &&
        incoming.useful_start > retained.useful_start &&
        incoming.useful_start <= retained.useful_end &&
        incoming.useful_end > retained.useful_end;
    return incoming.request_id > retained.request_id &&
        incoming.accepted_state_demand >= retained.accepted_state_demand &&
        ((incoming.useful_start <= retained.useful_start &&
          incoming.useful_end >= retained.useful_end) || forward_extension);
  }
  if (incoming.accepted_state_demand < retained.accepted_state_demand) {
    return false;
  }
  // Preserve a wider old prefix when a newer live-window request is merely a
  // narrower shifted suffix.  A contiguous extension is useful even when its
  // start is later than the old start.
  return (incoming.useful_start <= retained.useful_start &&
          incoming.useful_end >= retained.useful_end) ||
      (incoming.useful_start > retained.useful_start &&
       incoming.useful_start <= retained.useful_end &&
       incoming.useful_end > retained.useful_end);
}

template <typename Value>
void copyRequestMetadata(const TubeWorkerRequestV2& request, Value& value) {
  value.purpose = request.purpose;
  value.request_id = request.request_id;
  value.execution_generation = request.execution_generation;
  value.accepted_state_demand = request.accepted_state_demand;
  value.useful_start = request.useful_start;
  value.useful_end = request.useful_end;
  value.path_key = request.input.path_key;
  value.configuration_key = request.input.configuration_key;
  value.map_capture_key = request.input.map_capture_key;
}

}  // namespace

const char* tubeWorkerPurposeName(const TubeWorkerPurposeV2 purpose) {
  switch (purpose) {
    case TubeWorkerPurposeV2::CURRENT:
      return "CURRENT";
    case TubeWorkerPurposeV2::SUCCESSOR:
      return "SUCCESSOR";
  }
  return "UNKNOWN";
}

bool TubeWorkerRequestV2::complete() const {
  const int purpose_value = static_cast<int>(purpose);
  return (purpose_value == static_cast<int>(TubeWorkerPurposeV2::CURRENT) ||
          purpose_value == static_cast<int>(TubeWorkerPurposeV2::SUCCESSOR)) &&
      request_id != 0U && execution_generation != 0U &&
      accepted_state_demand != 0U && finite(useful_start) &&
      finite(useful_end) && useful_end > useful_start && input.request_id ==
      request_id && accepted_state_demand == input.map_capture_key.accepted_sequence &&
      input.path_key.complete() &&
      input.path_key.execution_generation == execution_generation &&
      input.configuration_key.complete() && input.map_capture_key.complete() &&
      useful_start >= input.requested_start && useful_end <= input.requested_end &&
      useful_start >= input.path_key.domain_start &&
      useful_end <= input.path_key.domain_end && input.complete();
}

const char* tubeWorkerCompletionStatusName(
    const TubeWorkerCompletionStatusV2 status) {
  switch (status) {
    case TubeWorkerCompletionStatusV2::BUILT:
      return "BUILT";
    case TubeWorkerCompletionStatusV2::FAILED:
      return "FAILED";
    case TubeWorkerCompletionStatusV2::CANCELLED:
      return "CANCELLED";
    case TubeWorkerCompletionStatusV2::STALE:
      return "STALE";
  }
  return "UNKNOWN";
}

const char* tubeWorkerEventTypeName(const TubeWorkerEventTypeV2 type) {
  switch (type) {
    case TubeWorkerEventTypeV2::SUBMITTED:
      return "SUBMITTED";
    case TubeWorkerEventTypeV2::COALESCED:
      return "COALESCED";
    case TubeWorkerEventTypeV2::REJECTED:
      return "REJECTED";
    case TubeWorkerEventTypeV2::BUILD_STARTED:
      return "BUILD_STARTED";
    case TubeWorkerEventTypeV2::BUILD_FINISHED:
      return "BUILD_FINISHED";
    case TubeWorkerEventTypeV2::CANCEL_REQUESTED:
      return "CANCEL_REQUESTED";
    case TubeWorkerEventTypeV2::COMPLETION_DISCARDED:
      return "COMPLETION_DISCARDED";
    case TubeWorkerEventTypeV2::COMPLETION_DELIVERED:
      return "COMPLETION_DELIVERED";
    case TubeWorkerEventTypeV2::RESET:
      return "RESET";
    case TubeWorkerEventTypeV2::PATH_INVALIDATED:
      return "PATH_INVALIDATED";
    case TubeWorkerEventTypeV2::SHUTDOWN:
      return "SHUTDOWN";
  }
  return "UNKNOWN";
}

struct PhaseOffsetTubeWorkerV2::Impl {
  using RequestPtr = std::unique_ptr<TubeWorkerRequestV2>;
  using CompletionPtr = std::unique_ptr<TubeWorkerCompletionV2>;

  explicit Impl(const phase_offset_navigation::TubeCertificateConfigV2& config,
                const TubeWorkerHooksV2& hooks)
      : config(config), hooks(hooks) {}

  phase_offset_navigation::TubeCertificateConfigV2 config;
  TubeWorkerHooksV2 hooks;
  mutable std::mutex mutex;
  std::condition_variable condition;
  std::thread worker;
  bool stop_requested = false;
  bool joined = false;
  bool running = false;
  bool running_cancel_requested = false;
  std::uint64_t active_generation = 0U;
  RequestPtr pending_current;
  RequestPtr pending_successor;
  TubeWorkerRequestV2 running_request;
  bool invalidated_path = false;
  // Keep the complete immutable path identity.  Instance/revision alone are
  // insufficient because frame, orientation, domain, and generation are all
  // authority-bearing fields of TubePathKey.
  phase_offset_navigation::TubePathKey invalidated_path_key;
  bool successor_turn = false;
  mutable std::mutex shutdown_mutex;
  struct WorkRecord {
    TubeWorkerPurposeV2 purpose = TubeWorkerPurposeV2::CURRENT;
    std::uint64_t request_id = 0U;
    std::uint64_t execution_generation = 0U;
    std::uint64_t accepted_state_demand = 0U;
    double useful_start = 0.0;
    double useful_end = 0.0;
    phase_offset_navigation::TubePathKey path_key;
    phase_offset_navigation::TubeConfigurationKey configuration_key;
    phase_offset_navigation::TubeMapCaptureKey map_capture_key;
  };
  // Caller-issued request IDs are monotone and non-reused per purpose within
  // one execution generation.  Keeping two high-water marks gives bounded
  // deduplication without a request-history registry.
  std::uint64_t last_submitted_current_id = 0U;
  std::uint64_t last_submitted_successor_id = 0U;
  bool have_last_completed_current = false;
  bool have_last_completed_successor = false;
  WorkRecord last_completed_current;
  WorkRecord last_completed_successor;
  CompletionPtr retained_current;
  CompletionPtr retained_successor;
  TubeWorkerStatsV2 aggregate;
  std::deque<TubeWorkerEventV2> event_log;

  static constexpr std::size_t kMaxEvents = 512U;

  bool pathInvalidatedLocked(const TubeWorkerRequestV2& request) const {
    return invalidated_path && request.input.path_key == invalidated_path_key;
  }

  bool pathInvalidatedLocked(const TubeWorkerCompletionV2& completion) const {
    return invalidated_path && completion.path_key == invalidated_path_key;
  }

  void eventLocked(const TubeWorkerEventTypeV2 type,
                   const TubeWorkerPurposeV2 purpose,
                   const std::uint64_t request_id,
                   const std::uint64_t generation,
                   const std::uint64_t demand = 0U,
                   const bool heavy = false,
                   const std::uint64_t duration = 0U) {
    TubeWorkerEventV2 event;
    event.type = type;
    event.timestamp_ticks = steadyTicks();
    event.purpose = purpose;
    event.request_id = request_id;
    event.execution_generation = generation;
    event.accepted_state_demand = demand;
    event.heavy_build_entered = heavy;
    event.build_duration_ns = duration;
    if (event_log.size() >= kMaxEvents) {
      event_log.pop_front();
      ++aggregate.event_log_dropped;
    }
    event_log.push_back(event);
  }

  RequestPtr& pendingSlot(const TubeWorkerPurposeV2 purpose) {
    return purpose == TubeWorkerPurposeV2::CURRENT ? pending_current
                                                    : pending_successor;
  }

  CompletionPtr& retainedSlot(const TubeWorkerPurposeV2 purpose) {
    return purpose == TubeWorkerPurposeV2::CURRENT ? retained_current
                                                    : retained_successor;
  }

  void discardPendingLocked(RequestPtr& pending, const bool stale,
                            const bool cancellation) {
    if (!pending) return;
    if (cancellation) ++aggregate.cancellation_discarded;
    if (stale) ++aggregate.stale_discarded;
    eventLocked(TubeWorkerEventTypeV2::COMPLETION_DISCARDED,
                pending->purpose, pending->request_id,
                pending->execution_generation, pending->accepted_state_demand);
    pending.reset();
  }

  void discardRetainedLocked(CompletionPtr& retained, const bool stale) {
    if (!retained) return;
    ++aggregate.delivery_discarded;
    if (stale) ++aggregate.stale_discarded;
    eventLocked(TubeWorkerEventTypeV2::COMPLETION_DISCARDED,
                retained->purpose, retained->request_id,
                retained->execution_generation,
                retained->accepted_state_demand);
    retained.reset();
  }

  void updateSlotCountsLocked() {
    aggregate.pending_current = pending_current ? 1U : 0U;
    aggregate.pending_successor = pending_successor ? 1U : 0U;
    aggregate.retained_current = retained_current ? 1U : 0U;
    aggregate.retained_successor = retained_successor ? 1U : 0U;
  }

  static WorkRecord workRecord(const TubeWorkerRequestV2& request) {
    WorkRecord record;
    record.purpose = request.purpose;
    record.request_id = request.request_id;
    record.execution_generation = request.execution_generation;
    record.accepted_state_demand = request.accepted_state_demand;
    record.useful_start = request.useful_start;
    record.useful_end = request.useful_end;
    record.path_key = request.input.path_key;
    record.configuration_key = request.input.configuration_key;
    record.map_capture_key = request.input.map_capture_key;
    return record;
  }

  static WorkRecord workRecord(const TubeWorkerCompletionV2& completion) {
    WorkRecord record;
    record.purpose = completion.purpose;
    record.request_id = completion.request_id;
    record.execution_generation = completion.execution_generation;
    record.accepted_state_demand = completion.accepted_state_demand;
    record.useful_start = completion.useful_start;
    record.useful_end = completion.useful_end;
    record.path_key = completion.path_key;
    record.configuration_key = completion.configuration_key;
    record.map_capture_key = completion.map_capture_key;
    return record;
  }

  static bool sameRecordAuthority(const TubeWorkerRequestV2& request,
                                  const WorkRecord& record) {
    return request.execution_generation == record.execution_generation &&
        request.purpose == record.purpose &&
        request.input.path_key == record.path_key &&
        request.input.configuration_key == record.configuration_key &&
        request.input.map_capture_key == record.map_capture_key;
  }

  static bool coveredBy(const TubeWorkerRequestV2& request,
                        const WorkRecord& record) {
    return sameRecordAuthority(request, record) &&
        record.accepted_state_demand >= request.accepted_state_demand &&
        record.useful_start <= request.useful_start &&
        record.useful_end >= request.useful_end;
  }

  static bool coveredBy(const TubeWorkerRequestV2& request,
                        const TubeWorkerRequestV2& existing) {
    return request.execution_generation == existing.execution_generation &&
        request.purpose == existing.purpose &&
        sameAuthority(request, existing) &&
        existing.accepted_state_demand >= request.accepted_state_demand &&
        existing.useful_start <= request.useful_start &&
        existing.useful_end >= request.useful_end;
  }

  static bool coveredBy(const TubeWorkerRequestV2& request,
                        const TubeWorkerCompletionV2& existing) {
    // A failed completion may be retained as diagnostic evidence, but it did
    // not establish geometric coverage and must never suppress a retry.
    return existing.built() && coveredBy(request, workRecord(existing));
  }

  // A newer accepted map capture supersedes an older pending capture for the
  // same path/configuration.  This is deliberately limited to pending
  // requests: a narrower or failed completion must not erase a useful
  // retained incumbent, which is handled by completionDominates() below.
  static bool acceptedMapRefreshDominates(
      const TubeWorkerRequestV2& incoming,
      const TubeWorkerRequestV2& pending) {
    return incoming.execution_generation == pending.execution_generation &&
        incoming.purpose == pending.purpose &&
        incoming.input.path_key == pending.input.path_key &&
        incoming.input.configuration_key == pending.input.configuration_key &&
        incoming.input.map_capture_key != pending.input.map_capture_key &&
        incoming.input.map_capture_key.state_id >=
            pending.input.map_capture_key.state_id &&
        incoming.input.map_capture_key.accepted_sequence >=
            pending.input.map_capture_key.accepted_sequence &&
        incoming.input.map_capture_key.accepted_time_ticks >=
            pending.input.map_capture_key.accepted_time_ticks &&
        (incoming.input.map_capture_key.state_id !=
             pending.input.map_capture_key.state_id ||
         incoming.input.map_capture_key.accepted_sequence !=
             pending.input.map_capture_key.accepted_sequence ||
         incoming.input.map_capture_key.accepted_time_ticks !=
             pending.input.map_capture_key.accepted_time_ticks) &&
        incoming.request_id > pending.request_id &&
        incoming.accepted_state_demand >= pending.accepted_state_demand;
  }

  void rememberCompletionLocked(const TubeWorkerCompletionV2& completion) {
    // Only a live successful proof may become bounded deduplication history.
    // Failed/cancelled/stale work can be delivered or accounted for, but it
    // cannot claim coverage for a later request.
    if (!completion.built()) return;
    if (completion.purpose == TubeWorkerPurposeV2::CURRENT) {
      last_completed_current = workRecord(completion);
      have_last_completed_current = true;
    } else {
      last_completed_successor = workRecord(completion);
      have_last_completed_successor = true;
    }
  }

  bool requestGenerationValidLocked(const TubeWorkerRequestV2& request) const {
    return !stop_requested && request.execution_generation == active_generation &&
        !pathInvalidatedLocked(request);
  }

  struct JobCancellationToken {
    struct Stats {
      std::uint64_t path_count = 0U;
      std::uint64_t path_duration_ns = 0U;
      std::uint64_t breakpoint_count = 0U;
      std::uint64_t breakpoint_duration_ns = 0U;
      std::uint64_t free_ball_count = 0U;
      std::uint64_t free_ball_duration_ns = 0U;
    };

    Impl* owner = nullptr;
    const TubeWorkerRequestV2* request = nullptr;
    std::shared_ptr<Stats> stats;

    void check() const {
      if (owner == nullptr || request == nullptr) {
        throw TubeWorkerCancelledV2();
      }
      std::lock_guard<std::mutex> lock(owner->mutex);
      if (owner->stop_requested || owner->running_cancel_requested ||
          request->execution_generation != owner->active_generation ||
          owner->pathInvalidatedLocked(*request)) {
        throw TubeWorkerCancelledV2();
      }
    }
  };

  phase_offset_navigation::TubeBuildInputV2 instrumentInput(
      const phase_offset_navigation::TubeBuildInputV2& input,
      const std::shared_ptr<JobCancellationToken>& token) {
    phase_offset_navigation::TubeBuildInputV2 instrumented = input;
    if (instrumented.path_cell_query) {
      const phase_offset_navigation::TubePathCellQueryV2 callback =
          instrumented.path_cell_query;
      instrumented.path_cell_query = [token, callback](
          const double w0, const double w1,
          phase_offset_core::CertifiedPathCellV2& cell) {
        token->check();
        const std::uint64_t start = steadyTicks();
        bool result = false;
        try {
          result = callback(w0, w1, cell);
        } catch (...) {
          const std::uint64_t end = steadyTicks();
          ++token->stats->path_count;
          if (end >= start) token->stats->path_duration_ns += end - start;
          throw;
        }
        const std::uint64_t end = steadyTicks();
        ++token->stats->path_count;
        if (end >= start) token->stats->path_duration_ns += end - start;
        token->check();
        return result;
      };
    }
    if (instrumented.producer_breakpoint_query) {
      const std::function<bool(std::vector<double>&)> callback =
          instrumented.producer_breakpoint_query;
      instrumented.producer_breakpoint_query = [token, callback](
          std::vector<double>& points) {
        token->check();
        const std::uint64_t start = steadyTicks();
        bool result = false;
        try {
          result = callback(points);
        } catch (...) {
          const std::uint64_t end = steadyTicks();
          ++token->stats->breakpoint_count;
          if (end >= start) token->stats->breakpoint_duration_ns += end - start;
          throw;
        }
        const std::uint64_t end = steadyTicks();
        ++token->stats->breakpoint_count;
        if (end >= start) token->stats->breakpoint_duration_ns += end - start;
        token->check();
        return result;
      };
    }
    if (instrumented.free_ball_query) {
      const phase_offset_navigation::TubeFreeBallQuery callback =
          instrumented.free_ball_query;
      instrumented.free_ball_query = [token, callback](
          const Eigen::Vector3d& witness, const double required) {
        token->check();
        const std::uint64_t start = steadyTicks();
        phase_offset_navigation::TubeFreeBallQueryResult result;
        try {
          result = callback(witness, required);
        } catch (...) {
          const std::uint64_t end = steadyTicks();
          ++token->stats->free_ball_count;
          if (end >= start) token->stats->free_ball_duration_ns += end - start;
          throw;
        }
        const std::uint64_t end = steadyTicks();
        ++token->stats->free_ball_count;
        if (end >= start) token->stats->free_ball_duration_ns += end - start;
        token->check();
        return result;
      };
    }
    return instrumented;
  }


  bool completionDominates(const TubeWorkerCompletionV2& incoming,
                           const TubeWorkerCompletionV2& retained) const {
    // A failed delivery is diagnostic evidence only.  It must never block a
    // later successful retry merely because that retry has a narrower useful
    // range (for example, after an accepted-map refresh).
    if (incoming.built() && !retained.built()) return true;
    if (!incoming.built() && retained.built()) return false;
    return coverageDominates(incoming, retained);
  }

  void requestRunningCancellationLocked() {
    if (!running || running_cancel_requested) return;
    running_cancel_requested = true;
    ++aggregate.cancellation_requested;
    eventLocked(TubeWorkerEventTypeV2::CANCEL_REQUESTED,
                running_request.purpose, running_request.request_id,
                running_request.execution_generation,
                running_request.accepted_state_demand);
  }

  void deliverLocked(CompletionPtr completion) {
    CompletionPtr& retained = retainedSlot(completion->purpose);
    if (!retained) {
      eventLocked(TubeWorkerEventTypeV2::COMPLETION_DELIVERED,
                  completion->purpose, completion->request_id,
                  completion->execution_generation,
                  completion->accepted_state_demand, completion->heavy_build_entered,
                  completion->build_duration_ns);
      retained = std::move(completion);
      ++aggregate.delivery_count;
      updateSlotCountsLocked();
      return;
    }
    if (completionDominates(*completion, *retained)) {
      ++aggregate.delivery_discarded;
      eventLocked(TubeWorkerEventTypeV2::COMPLETION_DISCARDED,
                  retained->purpose, retained->request_id,
                  retained->execution_generation,
                  retained->accepted_state_demand, retained->heavy_build_entered,
                  retained->build_duration_ns);
      eventLocked(TubeWorkerEventTypeV2::COMPLETION_DELIVERED,
                  completion->purpose, completion->request_id,
                  completion->execution_generation,
                  completion->accepted_state_demand, completion->heavy_build_entered,
                  completion->build_duration_ns);
      retained = std::move(completion);
      ++aggregate.delivery_count;
    } else {
      ++aggregate.delivery_discarded;
      eventLocked(TubeWorkerEventTypeV2::COMPLETION_DISCARDED,
                  completion->purpose, completion->request_id,
                  completion->execution_generation,
                  completion->accepted_state_demand, completion->heavy_build_entered,
                  completion->build_duration_ns);
    }
    updateSlotCountsLocked();
  }

  void run() {
    for (;;) {
      TubeWorkerRequestV2 request;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [this]() {
          return stop_requested || pending_current || pending_successor;
        });
        if (stop_requested && !pending_current && !pending_successor) break;
        RequestPtr selected;
        if (pending_current && pending_successor) {
          if (successor_turn) {
            selected = std::move(pending_successor);
          } else {
            selected = std::move(pending_current);
          }
          successor_turn = !successor_turn;
        } else if (pending_current) {
          selected = std::move(pending_current);
          successor_turn = true;
        } else {
          selected = std::move(pending_successor);
          successor_turn = false;
        }
        updateSlotCountsLocked();
        if (!selected) continue;
        request = std::move(*selected);
        running_request = request;
        running = true;
        running_cancel_requested = false;
        aggregate.in_flight = 0U;
      }

      TubeWorkerCompletionV2 completion;
      copyRequestMetadata(request, completion);
      bool run_builder = false;
      {
        std::lock_guard<std::mutex> lock(mutex);
        run_builder = requestGenerationValidLocked(request) &&
            !running_cancel_requested;
      }
      const std::shared_ptr<JobCancellationToken> token(
          new JobCancellationToken());
      token->owner = this;
      token->request = &request;
      token->stats.reset(new JobCancellationToken::Stats());
      phase_offset_navigation::TubeBuildInputV2 instrumented_input;
      if (run_builder) {
        try {
          token->check();
          if (hooks.before_build) hooks.before_build(request);
          token->check();
          instrumented_input = instrumentInput(request.input, token);
        } catch (const TubeWorkerCancelledV2&) {
          completion.callback_exception = true;
          completion.cancellation_requested = true;
          completion.error_message = "cancelled before proof";
          std::lock_guard<std::mutex> lock(mutex);
          requestRunningCancellationLocked();
          run_builder = false;
        } catch (const std::exception& exception) {
          completion.callback_exception = true;
          completion.error_message = exception.what();
          run_builder = false;
        } catch (...) {
          completion.callback_exception = true;
          completion.error_message = "unknown pre-build callback exception";
          run_builder = false;
        }
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        run_builder = run_builder && requestGenerationValidLocked(request) &&
            !running_cancel_requested;
      }

      if (run_builder) {
        const phase_offset_navigation::TubeCertificateBuilderV2 builder(config);
        completion.build = phase_offset_navigation::TubeCertificateBuildResultV2();
        completion.build_start_ticks = steadyTicks();
        {
          std::lock_guard<std::mutex> lock(mutex);
          ++aggregate.in_flight;
          aggregate.peak_in_flight = std::max<std::uint64_t>(
              aggregate.peak_in_flight, aggregate.in_flight);
          ++aggregate.build_started;
          completion.heavy_build_entered = true;
          eventLocked(TubeWorkerEventTypeV2::BUILD_STARTED,
                      request.purpose, request.request_id,
                      request.execution_generation, request.accepted_state_demand,
                      true);
        }
        try {
          completion.build.success = builder.build(instrumented_input,
                                                   completion.build);
          token->check();
        } catch (const TubeWorkerCancelledV2&) {
          completion.callback_exception = true;
          completion.error_message = "cancelled at proof-unit boundary";
          std::lock_guard<std::mutex> lock(mutex);
          requestRunningCancellationLocked();
        } catch (const std::exception& exception) {
          completion.builder_exception = true;
          completion.error_message = exception.what();
        } catch (...) {
          completion.builder_exception = true;
          completion.error_message = "unknown builder exception";
        }
        completion.build_end_ticks = steadyTicks();
        completion.build_duration_ns = completion.build_end_ticks >=
                completion.build_start_ticks
            ? completion.build_end_ticks - completion.build_start_ticks : 0U;
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (aggregate.in_flight > 0U) --aggregate.in_flight;
          ++aggregate.build_finished;
          if (completion.build.success) {
            ++aggregate.build_succeeded;
          } else {
            ++aggregate.build_failed;
          }
          eventLocked(TubeWorkerEventTypeV2::BUILD_FINISHED,
                      request.purpose, request.request_id,
                      request.execution_generation, request.accepted_state_demand,
                      true, completion.build_duration_ns);
        }
        completion.status = completion.build.success
            ? TubeWorkerCompletionStatusV2::BUILT
            : TubeWorkerCompletionStatusV2::FAILED;
      } else {
        completion.status = completion.callback_exception
            ? TubeWorkerCompletionStatusV2::FAILED
            : TubeWorkerCompletionStatusV2::CANCELLED;
      }

      completion.path_callback_count = token->stats->path_count;
      completion.path_callback_duration_ns = token->stats->path_duration_ns;
      completion.producer_breakpoint_callback_count =
          token->stats->breakpoint_count;
      completion.producer_breakpoint_callback_duration_ns =
          token->stats->breakpoint_duration_ns;
      completion.free_ball_callback_count = token->stats->free_ball_count;
      completion.free_ball_callback_duration_ns =
          token->stats->free_ball_duration_ns;
      if (completion.build.success && completion.build.profile.complete &&
          completion.build.profile.valid &&
          finite(completion.build.profile.certified_start) &&
          finite(completion.build.profile.certified_end) &&
          completion.build.profile.certified_end >
              completion.build.profile.certified_start) {
        completion.useful_start = completion.build.profile.certified_start;
        completion.useful_end = completion.build.profile.certified_end;
      }

      if (hooks.after_build) {
        try {
          hooks.after_build(request, completion);
        } catch (const TubeWorkerCancelledV2&) {
          completion.callback_exception = true;
          completion.error_message = "cancelled at proof-unit boundary";
          std::lock_guard<std::mutex> lock(mutex);
          requestRunningCancellationLocked();
          completion.build.success = false;
          completion.status = TubeWorkerCompletionStatusV2::FAILED;
        } catch (const std::exception& exception) {
          completion.callback_exception = true;
          completion.error_message = exception.what();
          completion.build.success = false;
          completion.status = TubeWorkerCompletionStatusV2::FAILED;
        } catch (...) {
          completion.callback_exception = true;
          completion.error_message = "unknown callback exception";
          completion.build.success = false;
          completion.status = TubeWorkerCompletionStatusV2::FAILED;
        }
      }

      {
        std::lock_guard<std::mutex> lock(mutex);
        aggregate.path_callback_count += completion.path_callback_count;
        aggregate.path_callback_duration_ns +=
            completion.path_callback_duration_ns;
        aggregate.producer_breakpoint_callback_count +=
            completion.producer_breakpoint_callback_count;
        aggregate.producer_breakpoint_callback_duration_ns +=
            completion.producer_breakpoint_callback_duration_ns;
        aggregate.free_ball_callback_count += completion.free_ball_callback_count;
        aggregate.free_ball_callback_duration_ns +=
            completion.free_ball_callback_duration_ns;
        const bool stale = request.execution_generation != active_generation ||
            pathInvalidatedLocked(request);
        const bool cancelled = running_cancel_requested || stop_requested;
        completion.cancellation_requested = cancelled;
        if (stale) {
          completion.status = TubeWorkerCompletionStatusV2::STALE;
          ++aggregate.stale_discarded;
          eventLocked(TubeWorkerEventTypeV2::COMPLETION_DISCARDED,
                      request.purpose, request.request_id,
                      request.execution_generation, request.accepted_state_demand,
                      completion.heavy_build_entered, completion.build_duration_ns);
        } else if (cancelled) {
          completion.status = TubeWorkerCompletionStatusV2::CANCELLED;
          ++aggregate.cancellation_discarded;
          eventLocked(TubeWorkerEventTypeV2::COMPLETION_DISCARDED,
                      request.purpose, request.request_id,
                      request.execution_generation, request.accepted_state_demand,
                      completion.heavy_build_entered, completion.build_duration_ns);
        } else {
          rememberCompletionLocked(completion);
          deliverLocked(std::unique_ptr<TubeWorkerCompletionV2>(
              new TubeWorkerCompletionV2(std::move(completion))));
        }
        running_request = TubeWorkerRequestV2();
        running = false;
        running_cancel_requested = false;
        aggregate.in_flight = 0U;
        updateSlotCountsLocked();
      }
      condition.notify_all();
    }
    std::lock_guard<std::mutex> lock(mutex);
    running = false;
    aggregate.in_flight = 0U;
    updateSlotCountsLocked();
  }
};

PhaseOffsetTubeWorkerV2::PhaseOffsetTubeWorkerV2(
    const phase_offset_navigation::TubeCertificateConfigV2& config,
    const TubeWorkerHooksV2& hooks)
    : config_(config), hooks_(hooks), impl_(new Impl(config, hooks)) {
  impl_->worker = std::thread([impl = impl_]() { impl->run(); });
}

PhaseOffsetTubeWorkerV2::~PhaseOffsetTubeWorkerV2() {
  shutdown();
}

bool PhaseOffsetTubeWorkerV2::submit(const TubeWorkerRequestV2& request) {
  if (!impl_) return false;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  ++impl_->aggregate.submitted;
  if (!request.complete() ||
      !(request.input.configuration_key == config_.key())) {
    ++impl_->aggregate.rejected;
    impl_->eventLocked(TubeWorkerEventTypeV2::REJECTED, request.purpose,
                       request.request_id, request.execution_generation,
                       request.accepted_state_demand);
    return false;
  }
  if (impl_->active_generation == 0U) {
    impl_->active_generation = request.execution_generation;
  }
  if (!impl_->requestGenerationValidLocked(request)) {
    ++impl_->aggregate.rejected;
    impl_->eventLocked(TubeWorkerEventTypeV2::REJECTED, request.purpose,
                       request.request_id, request.execution_generation,
                       request.accepted_state_demand);
    return false;
  }
  std::uint64_t& last_submitted_id = request.purpose ==
          TubeWorkerPurposeV2::CURRENT ? impl_->last_submitted_current_id
                                       : impl_->last_submitted_successor_id;
  if (request.request_id <= last_submitted_id) {
    ++impl_->aggregate.rejected;
    impl_->eventLocked(TubeWorkerEventTypeV2::REJECTED, request.purpose,
                       request.request_id, request.execution_generation,
                       request.accepted_state_demand);
    return false;
  }
  const auto covered_by = [&request, this]() {
    if (impl_->running && impl_->running_request.purpose == request.purpose &&
        impl_->running_request.execution_generation ==
            request.execution_generation &&
        impl_->running_request.input.path_key == request.input.path_key &&
        impl_->running_request.input.configuration_key ==
            request.input.configuration_key &&
        impl_->running_request.input.map_capture_key ==
            request.input.map_capture_key &&
        impl_->running_request.accepted_state_demand >=
            request.accepted_state_demand &&
        impl_->running_request.useful_start <= request.useful_start &&
        impl_->running_request.useful_end >= request.useful_end) {
      return true;
    }
    const Impl::RequestPtr& pending = impl_->pendingSlot(request.purpose);
    if (pending && Impl::coveredBy(request, *pending)) return true;
    const Impl::CompletionPtr& retained = impl_->retainedSlot(request.purpose);
    if (retained && Impl::coveredBy(request, *retained)) return true;
    if (request.purpose == TubeWorkerPurposeV2::CURRENT) {
      return impl_->have_last_completed_current &&
          Impl::coveredBy(request, impl_->last_completed_current);
    }
    return impl_->have_last_completed_successor &&
        Impl::coveredBy(request, impl_->last_completed_successor);
  };
  if (covered_by()) {
    ++impl_->aggregate.rejected;
    ++impl_->aggregate.less_useful_rejected;
    impl_->eventLocked(TubeWorkerEventTypeV2::REJECTED, request.purpose,
                       request.request_id, request.execution_generation,
                       request.accepted_state_demand);
    return false;
  }
  Impl::RequestPtr& slot = impl_->pendingSlot(request.purpose);
  if (slot) {
    if (coverageDominates(request, *slot) ||
        Impl::acceptedMapRefreshDominates(request, *slot)) {
      impl_->eventLocked(TubeWorkerEventTypeV2::COMPLETION_DISCARDED,
                         slot->purpose, slot->request_id,
                         slot->execution_generation,
                         slot->accepted_state_demand);
      impl_->eventLocked(TubeWorkerEventTypeV2::COALESCED, request.purpose,
                         request.request_id, request.execution_generation,
                         request.accepted_state_demand);
      slot.reset(new TubeWorkerRequestV2(request));
      ++impl_->aggregate.accepted;
      ++impl_->aggregate.coalesced;
      last_submitted_id = request.request_id;
      impl_->updateSlotCountsLocked();
      impl_->condition.notify_one();
      return true;
    }
    ++impl_->aggregate.rejected;
    ++impl_->aggregate.less_useful_rejected;
    impl_->eventLocked(TubeWorkerEventTypeV2::REJECTED, request.purpose,
                       request.request_id, request.execution_generation,
                       request.accepted_state_demand);
    return false;
  }
  slot.reset(new TubeWorkerRequestV2(request));
  ++impl_->aggregate.accepted;
  last_submitted_id = request.request_id;
  impl_->eventLocked(TubeWorkerEventTypeV2::SUBMITTED, request.purpose,
                     request.request_id, request.execution_generation,
                     request.accepted_state_demand);
  impl_->updateSlotCountsLocked();
  impl_->condition.notify_one();
  return true;
}

bool PhaseOffsetTubeWorkerV2::cancel(const TubeWorkerPurposeV2 purpose) {
  if (!impl_ || !validPurpose(purpose)) return false;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  Impl::RequestPtr& pending = impl_->pendingSlot(purpose);
  if (pending) {
    const std::uint64_t request_id = pending->request_id;
    const std::uint64_t generation = pending->execution_generation;
    const std::uint64_t demand = pending->accepted_state_demand;
    impl_->discardPendingLocked(pending, false, true);
    ++impl_->aggregate.cancellation_requested;
    impl_->eventLocked(TubeWorkerEventTypeV2::CANCEL_REQUESTED, purpose,
                       request_id, generation, demand);
    impl_->updateSlotCountsLocked();
    impl_->condition.notify_one();
    return true;
  }
  if (impl_->running && impl_->running_request.purpose == purpose) {
    if (!impl_->running_cancel_requested) {
      impl_->running_cancel_requested = true;
      ++impl_->aggregate.cancellation_requested;
      impl_->eventLocked(TubeWorkerEventTypeV2::CANCEL_REQUESTED, purpose,
                         impl_->running_request.request_id,
                         impl_->running_request.execution_generation,
                         impl_->running_request.accepted_state_demand);
    }
    return true;
  }
  return false;
}

bool PhaseOffsetTubeWorkerV2::reset(const std::uint64_t execution_generation) {
  if (!impl_ || execution_generation == 0U) return false;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->stop_requested || execution_generation <= impl_->active_generation) {
    return false;
  }
  impl_->active_generation = execution_generation;
  impl_->discardPendingLocked(impl_->pending_current, true, true);
  impl_->discardPendingLocked(impl_->pending_successor, true, true);
  impl_->discardRetainedLocked(impl_->retained_current, true);
  impl_->discardRetainedLocked(impl_->retained_successor, true);
  if (impl_->running) {
    impl_->running_cancel_requested = true;
    ++impl_->aggregate.cancellation_requested;
    impl_->eventLocked(TubeWorkerEventTypeV2::CANCEL_REQUESTED,
                       impl_->running_request.purpose,
                       impl_->running_request.request_id,
                       impl_->running_request.execution_generation,
                       impl_->running_request.accepted_state_demand);
  }
  impl_->invalidated_path = false;
  impl_->invalidated_path_key = phase_offset_navigation::TubePathKey();
  // Request IDs are monotone/non-reused within one execution generation;
  // resetting the high-water marks starts the new generation's bounded ID
  // domain while stale work from the old generation remains retired.
  impl_->last_submitted_current_id = 0U;
  impl_->last_submitted_successor_id = 0U;
  impl_->have_last_completed_current = false;
  impl_->have_last_completed_successor = false;
  impl_->eventLocked(TubeWorkerEventTypeV2::RESET,
                     TubeWorkerPurposeV2::CURRENT, 0U,
                     execution_generation);
  impl_->updateSlotCountsLocked();
  impl_->condition.notify_one();
  return true;
}

bool PhaseOffsetTubeWorkerV2::invalidatePath(
    const phase_offset_navigation::TubePathKey& path_key) {
  if (!impl_ || !path_key.complete()) return false;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  // Invalidation is scoped to the caller's exact execution generation.  An
  // old-generation key must never poison a subsequently reset worker.
  if (impl_->stop_requested || impl_->active_generation == 0U ||
      path_key.execution_generation != impl_->active_generation) {
    return false;
  }
  impl_->invalidated_path = true;
  impl_->invalidated_path_key = path_key;
  for (Impl::RequestPtr* pending : {&impl_->pending_current,
                                    &impl_->pending_successor}) {
    if (*pending && (*pending)->input.path_key == path_key) {
      impl_->discardPendingLocked(*pending, true, true);
    }
  }
  for (Impl::CompletionPtr* retained : {&impl_->retained_current,
                                        &impl_->retained_successor}) {
    if (*retained && (*retained)->path_key == path_key) {
      impl_->discardRetainedLocked(*retained, true);
    }
  }
  if (impl_->running && impl_->running_request.input.path_key == path_key) {
    impl_->requestRunningCancellationLocked();
  }
  impl_->eventLocked(TubeWorkerEventTypeV2::PATH_INVALIDATED,
                     TubeWorkerPurposeV2::CURRENT, 0U,
                     path_key.execution_generation);
  impl_->updateSlotCountsLocked();
  impl_->condition.notify_one();
  return true;
}

bool PhaseOffsetTubeWorkerV2::tryTake(
    const TubeWorkerPurposeV2 purpose,
    TubeWorkerCompletionV2& completion) {
  if (!impl_ || !validPurpose(purpose)) return false;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  Impl::CompletionPtr& retained = impl_->retainedSlot(purpose);
  if (!retained) return false;
  completion = std::move(*retained);
  retained.reset();
  impl_->updateSlotCountsLocked();
  return true;
}

bool PhaseOffsetTubeWorkerV2::tryTakeAny(
    TubeWorkerCompletionV2& completion) {
  if (tryTake(TubeWorkerPurposeV2::CURRENT, completion)) return true;
  return tryTake(TubeWorkerPurposeV2::SUCCESSOR, completion);
}

void PhaseOffsetTubeWorkerV2::shutdown() {
  if (!impl_) return;
  // Serialize join attempts.  std::thread::join is not safe when concurrent
  // callers race on the same joinable worker.
  std::lock_guard<std::mutex> shutdown_lock(impl_->shutdown_mutex);
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->stop_requested) {
      impl_->stop_requested = true;
      ++impl_->aggregate.cancellation_requested;
      if (impl_->running) impl_->running_cancel_requested = true;
      impl_->discardPendingLocked(impl_->pending_current, false, true);
      impl_->discardPendingLocked(impl_->pending_successor, false, true);
      // Once shutdown starts no retained proof may be consumed through the
      // public channel; release both slots before allowing callers to return.
      impl_->discardRetainedLocked(impl_->retained_current, false);
      impl_->discardRetainedLocked(impl_->retained_successor, false);
      impl_->eventLocked(TubeWorkerEventTypeV2::SHUTDOWN,
                         TubeWorkerPurposeV2::CURRENT, 0U,
                         impl_->active_generation);
      impl_->updateSlotCountsLocked();
    }
  }
  impl_->condition.notify_all();
  if (impl_->worker.joinable() &&
      impl_->worker.get_id() != std::this_thread::get_id()) {
    impl_->worker.join();
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->joined = !impl_->worker.joinable();
}

bool PhaseOffsetTubeWorkerV2::joined() const {
  if (!impl_) return true;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->joined;
}

bool PhaseOffsetTubeWorkerV2::running() const {
  if (!impl_) return false;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->running;
}

TubeWorkerStatsV2 PhaseOffsetTubeWorkerV2::stats() const {
  if (!impl_) return TubeWorkerStatsV2();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  TubeWorkerStatsV2 result = impl_->aggregate;
  result.pending_current = impl_->pending_current ? 1U : 0U;
  result.pending_successor = impl_->pending_successor ? 1U : 0U;
  result.retained_current = impl_->retained_current ? 1U : 0U;
  result.retained_successor = impl_->retained_successor ? 1U : 0U;
  return result;
}

std::vector<TubeWorkerEventV2> PhaseOffsetTubeWorkerV2::events() const {
  if (!impl_) return std::vector<TubeWorkerEventV2>();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return std::vector<TubeWorkerEventV2>(impl_->event_log.begin(),
                                        impl_->event_log.end());
}

}  // namespace FLAG_Race
