#pragma once

#include <phase_offset_navigation/tube_certificate_v2.h>

#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace FLAG_Race {

// A worker request is an immutable value once submitted.  The navigation
// builder input already carries the exact path/configuration/map keys and all
// immutable owners needed by a proof; these additional fields identify the
// bounded worker purpose and the accepted-state demand used for coalescing.
enum class TubeWorkerPurposeV2 {
  CURRENT = 0,
  SUCCESSOR = 1,
};

const char* tubeWorkerPurposeName(TubeWorkerPurposeV2 purpose);

struct TubeWorkerRequestV2 {
  TubeWorkerPurposeV2 purpose = TubeWorkerPurposeV2::CURRENT;
  std::uint64_t request_id = 0U;
  // This is the execution generation, not a live phase or an observation
  // sequence.  Reset advances it and retires all requests from older values.
  std::uint64_t execution_generation = 0U;
  // Accepted-map demand is monotone only within an otherwise matching path /
  // configuration key.  It is deliberately not a raw-cloud arrival counter.
  std::uint64_t accepted_state_demand = 0U;
  // Useful proof coverage used to decide whether a pending/retained result is
  // dominated by a replacement.  The exact live anchor is intentionally not
  // part of this coalescing key.
  double useful_start = 0.0;
  double useful_end = 0.0;
  phase_offset_navigation::TubeBuildInputV2 input;

  bool complete() const;
};

// A test barrier may use this typed exception to stop at a proof-unit
// boundary without escaping the worker thread.  It never crosses the public
// completion channel and is not used by production geometry code.
class TubeWorkerCancelledV2 : public std::exception {
 public:
  const char* what() const noexcept override { return "tube worker cancelled"; }
};

enum class TubeWorkerCompletionStatusV2 {
  BUILT = 0,
  FAILED,
  CANCELLED,
  STALE,
};

const char* tubeWorkerCompletionStatusName(TubeWorkerCompletionStatusV2 status);

// A completion retains the exact builder result and request identity.  A
// failed result remains useful evidence (and retains its immutable owners),
// while CANCELLED/STALE results are normally accounted for in events without
// occupying a delivery slot.
struct TubeWorkerCompletionV2 {
  TubeWorkerCompletionStatusV2 status =
      TubeWorkerCompletionStatusV2::FAILED;
  TubeWorkerPurposeV2 purpose = TubeWorkerPurposeV2::CURRENT;
  std::uint64_t request_id = 0U;
  std::uint64_t execution_generation = 0U;
  std::uint64_t accepted_state_demand = 0U;
  double useful_start = 0.0;
  double useful_end = 0.0;
  phase_offset_navigation::TubePathKey path_key;
  phase_offset_navigation::TubeConfigurationKey configuration_key;
  phase_offset_navigation::TubeMapCaptureKey map_capture_key;

  // Monotonic steady-clock timestamps.  Zero means no builder invocation
  // occurred (for example a request cancelled while pending).
  std::uint64_t build_start_ticks = 0U;
  std::uint64_t build_end_ticks = 0U;
  std::uint64_t build_duration_ns = 0U;
  std::uint64_t path_callback_count = 0U;
  std::uint64_t path_callback_duration_ns = 0U;
  std::uint64_t producer_breakpoint_callback_count = 0U;
  std::uint64_t producer_breakpoint_callback_duration_ns = 0U;
  std::uint64_t free_ball_callback_count = 0U;
  std::uint64_t free_ball_callback_duration_ns = 0U;
  bool heavy_build_entered = false;
  bool cancellation_requested = false;
  bool callback_exception = false;
  bool builder_exception = false;
  std::string error_message;
  phase_offset_navigation::TubeCertificateBuildResultV2 build;

  bool built() const {
    return status == TubeWorkerCompletionStatusV2::BUILT && build.success;
  }
};

struct TubeWorkerStatsV2 {
  std::uint64_t submitted = 0U;
  std::uint64_t accepted = 0U;
  std::uint64_t rejected = 0U;
  std::uint64_t coalesced = 0U;
  std::uint64_t less_useful_rejected = 0U;
  std::uint64_t build_started = 0U;
  std::uint64_t build_finished = 0U;
  std::uint64_t build_succeeded = 0U;
  std::uint64_t build_failed = 0U;
  std::uint64_t path_callback_count = 0U;
  std::uint64_t path_callback_duration_ns = 0U;
  std::uint64_t producer_breakpoint_callback_count = 0U;
  std::uint64_t producer_breakpoint_callback_duration_ns = 0U;
  std::uint64_t free_ball_callback_count = 0U;
  std::uint64_t free_ball_callback_duration_ns = 0U;
  std::uint64_t cancellation_requested = 0U;
  std::uint64_t cancellation_discarded = 0U;
  std::uint64_t stale_discarded = 0U;
  std::uint64_t delivery_count = 0U;
  std::uint64_t delivery_discarded = 0U;
  std::uint64_t event_log_dropped = 0U;
  std::uint64_t in_flight = 0U;
  std::uint64_t peak_in_flight = 0U;
  std::uint64_t pending_current = 0U;
  std::uint64_t pending_successor = 0U;
  std::uint64_t retained_current = 0U;
  std::uint64_t retained_successor = 0U;
};

enum class TubeWorkerEventTypeV2 {
  SUBMITTED = 0,
  COALESCED,
  REJECTED,
  BUILD_STARTED,
  BUILD_FINISHED,
  CANCEL_REQUESTED,
  COMPLETION_DISCARDED,
  COMPLETION_DELIVERED,
  RESET,
  PATH_INVALIDATED,
  SHUTDOWN,
};

const char* tubeWorkerEventTypeName(TubeWorkerEventTypeV2 type);

struct TubeWorkerEventV2 {
  TubeWorkerEventTypeV2 type = TubeWorkerEventTypeV2::REJECTED;
  std::uint64_t timestamp_ticks = 0U;
  TubeWorkerPurposeV2 purpose = TubeWorkerPurposeV2::CURRENT;
  std::uint64_t request_id = 0U;
  std::uint64_t execution_generation = 0U;
  std::uint64_t accepted_state_demand = 0U;
  bool heavy_build_entered = false;
  std::uint64_t build_duration_ns = 0U;
};

// Supplemental test barriers are called outside the worker mutex and are
// never consulted by geometry or map predicates.  They make overlap and
// cancellation ordering deterministic without introducing a second proof
// implementation.  Production callers leave both hooks empty.
struct TubeWorkerHooksV2 {
  std::function<void(const TubeWorkerRequestV2&)> before_build;
  std::function<void(const TubeWorkerRequestV2&,
                     const TubeWorkerCompletionV2&)> after_build;
};

// One joined worker and one bounded completion channel.  There is exactly
// one running TubeCertificateBuilderV2 call at a time, with one pending slot
// for each purpose and one retained useful completion slot for each purpose.
// The class is intentionally disconnected from adapters, ROS, mutable SDFMap
// objects, and authoritative phase/delta/port state.
class PhaseOffsetTubeWorkerV2 {
 public:
  explicit PhaseOffsetTubeWorkerV2(
      const phase_offset_navigation::TubeCertificateConfigV2& config,
      const TubeWorkerHooksV2& hooks = TubeWorkerHooksV2());
  ~PhaseOffsetTubeWorkerV2();

  PhaseOffsetTubeWorkerV2(const PhaseOffsetTubeWorkerV2&) = delete;
  PhaseOffsetTubeWorkerV2& operator=(const PhaseOffsetTubeWorkerV2&) = delete;

  // The constructor starts one joinable worker thread.  Submit copies the
  // request value and its immutable owners; the caller may release/mutate
  // only its own wrapper immediately after this call.
  bool submit(const TubeWorkerRequestV2& request);

  // Remove a pending request or mark the matching running request for
  // cancellation.  Cancellation is observed at proof-unit boundaries and a
  // completed cancelled build is discarded, never delivered as authority.
  bool cancel(TubeWorkerPurposeV2 purpose);

  // Retire all pending/running/retained work and establish a new execution
  // generation.  A stale running completion is discarded after the builder
  // returns; no forced thread termination is used.
  bool reset(std::uint64_t execution_generation);

  // Retire an exact path identity in all matching pending/running/retained
  // slots.  Different path identities in the same execution generation stay
  // eligible; callers must issue a fresh path key when replacing a retired
  // path, and a running retired request can never be delivered.
  bool invalidatePath(const phase_offset_navigation::TubePathKey& path_key);

  // Try-consume one retained completion.  Consumption is explicit and never
  // starts another builder.  A completion can be consumed only once.
  bool tryTake(TubeWorkerPurposeV2 purpose, TubeWorkerCompletionV2& completion);
  bool tryTakeAny(TubeWorkerCompletionV2& completion);

  // Idempotent shutdown.  It requests cancellation, wakes the worker and
  // joins it before returning.  No mutex is held while the builder or hooks
  // execute.
  void shutdown();

  bool joined() const;
  bool running() const;
  TubeWorkerStatsV2 stats() const;
  std::vector<TubeWorkerEventV2> events() const;
  const phase_offset_navigation::TubeCertificateConfigV2& config() const {
    return config_;
  }

 private:
  struct Impl;
  phase_offset_navigation::TubeCertificateConfigV2 config_;
  TubeWorkerHooksV2 hooks_;
  std::shared_ptr<Impl> impl_;
};

}  // namespace FLAG_Race
