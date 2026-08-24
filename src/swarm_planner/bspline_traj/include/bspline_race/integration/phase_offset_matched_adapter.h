#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/continuous_phase_normal_frame.h>
#include <bspline_race/integration/phase_offset_active_adapter.h>
#include <bspline_race/integration/phase_offset_cloud_occupancy_query.h>
#include <bspline_race/integration/phase_offset_raw_candidate_diagnostics.h>
#include <plan_env/cloud_occupancy_snapshot.h>
#include <phase_offset_navigation/phase_offset_runtime.h>
#include <phase_offset_navigation/immutable_executed_reference_query.h>
#include <phase_offset_navigation/active_reference_snapshot.h>
#include <phase_offset_navigation/active_reference_authority.h>
#include <phase_offset_navigation/phase_offset_recovery_owner.h>
#include <phase_offset_navigation/preview_feasibility.h>
#include <phase_offset_navigation/handoff_state_machine.h>
#include <phase_offset_navigation/tube_epoch_manager.h>

namespace FLAG_Race {

class gvf_manager;
struct TubeEpochSnapshot;
struct TubeBuildRequest;

enum class PhaseOffsetMatchedMode { ACTIVE = 0, MANUAL = 1 };

// This is deliberately an owning value type.  A command cycle may hand it to
// the tube timer, but neither side may retain a mutable path/vector owned by
// the other callback.
using MatchedAdapterPathSamples =
    phase_offset_navigation::RuntimePathSamples;

struct PhaseOffsetMatchedAdapterConfig {
  PhaseOffsetMatchedMode mode = PhaseOffsetMatchedMode::ACTIVE;
  double equivalence_tolerance = 1e-10;
  int warmup_cycles = 100;
  double amplitude = 0.10;
  // Candidate construction and exact-port evaluation remain enabled, while
  // Runtime gate-closed semantics prevent control selection and state commit.
  bool observe_only = true;
  double profile_period = 10.0;
  double delta_tracking_gain = 3.0;
  double u_w_amplitude = 0.05;
  double u_w_abs_max = 0.12;
  double u_delta_abs_max = 0.25;
  double u_w_rate_max = 0.60;
  double u_delta_rate_max = 1.20;
  double phase_dot_min = 0.02;
  double tangent_speed_min = 0.02;
  double preflight_sample_step_w = 0.10;
  phase_offset_navigation::TubeSource tube_source =
      phase_offset_navigation::TubeSource::NONE;
  phase_offset_navigation::TubeBuilderConfig tube;
  phase_offset_navigation::TubeFilterConfig filter;
  double tube_update_period = 0.10;
  // ESDF-selected tube construction accepts the local occupycloud snapshot
  // only when its producer promises a complete local obstacle set.  The
  // default remains fail-closed until an explicit launch enables it.
  bool cloud_obstacle_set_complete = false;
  std::string frame_id = "world";
  // S3-T0-only sidecar timing is opt-in and never changes tube construction.
  bool measurement_enabled = false;
  std::string measurement_tube_due_csv_path;

  PhaseOffsetMatchedAdapterConfig() {
    tube.cross_section.search_extent = 3.0;
    tube.cross_section.ray_step = 0.05;
    tube.cross_section.boundary_tolerance = 0.01;
    tube.cross_section.regularity_margin = 0.10;
    tube.cross_section.curvature_epsilon = 1e-9;
    tube.cross_section.margins.uav_radius = 0.25;
    tube.cross_section.margins.map_uncertainty = 0.10;
    tube.cross_section.margins.localization_uncertainty = 0.05;
    tube.cross_section.margins.tracking_error_bound = 0.15;
    tube.cross_section.margins.preincluded_map_uncertainty = 0.0;
  }
};

// Immutable control authority for an H2 path/tube handoff.  This is an
// ownership value only: it adds no ROS-facing state, gate, diagnostic field,
// or execution mode.  A command captures one shared_ptr and must take its
// path, Runtime profile/status, and map provenance from that one object.
struct PathTubePair {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t source_revision = 0U;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t generation = 0U;
  // Private H2 retirement domain.  It is neither a Runtime state nor a
  // diagnostic: a goal/reset increments it so a command that captured an old
  // shared_ptr cannot continue to execute or install that retired authority.
  std::uint64_t authority_session = 0U;
  std::uint64_t map_observation_sequence = 0U;
  bool map_observation_is_snapshot = false;
  std::shared_ptr<const ContinuousPhasePath> path_owner;
  // One immutable Bishop frame is shared by all path-state, cell-proof, and
  // executed-reference queries for this authority handoff.
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame_owner;
  std::shared_ptr<const plan_env::CloudOccupancySnapshot>
      frozen_cloud_occupancy_snapshot;
  std::shared_ptr<const MatchedAdapterPathSamples> full_path_samples;
  std::shared_ptr<const phase_offset_navigation::TubeProfile> active_profile;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
      executed_reference_query;
  // Exact predecessor authority used to seed a staged successor frame.  It
  // is immutable provenance only; governor/reference consumers never read the
  // stored executed_N as a control input.
  std::shared_ptr<const phase_offset_navigation::ActiveReferenceSnapshot>
      successor_seed_authority;
  phase_offset_navigation::TubeEpochStatus epoch_status;
  // The precommit facts are retained verbatim so a later command can reject
  // a stale stage before it exchanges authority.
  double captured_w0 = 0.0;
  double future_seam_w = 0.0;
  double existing_future_horizon_end_w = 0.0;
  double captured_retained_delta = 0.0;
  phase_offset_core::PortCommand captured_previous_final_port;
  std::shared_ptr<const TubeEpochSnapshot> epoch_snapshot;
};

// Immutable evidence captured atomically with a transaction-scoped pin.  It
// deliberately copies identities/provenance and Runtime history rather than
// exposing an adapter lock to seam/C2/tube work.
struct PathTubePairPinCapture {
  std::shared_ptr<const PathTubePair> pair;
  std::uint64_t source_revision = 0U;
  std::uint64_t generation = 0U;
  std::uint64_t authority_session = 0U;
  std::uint64_t map_observation_sequence = 0U;
  bool map_observation_is_snapshot = false;
  std::shared_ptr<const ContinuousPhasePath> path_owner;
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame_owner;
  std::shared_ptr<const plan_env::CloudOccupancySnapshot>
      frozen_cloud_occupancy_snapshot;
  std::shared_ptr<const MatchedAdapterPathSamples> full_path_samples;
  std::shared_ptr<const phase_offset_navigation::TubeProfile> active_profile;
  std::shared_ptr<const TubeEpochSnapshot> epoch_snapshot;
  double retained_delta = 0.0;
  phase_offset_core::PortCommand previous_final_port;

  bool valid() const {
    return pair && source_revision == pair->source_revision &&
        generation == pair->generation &&
        authority_session == pair->authority_session &&
        map_observation_sequence == pair->map_observation_sequence &&
        map_observation_is_snapshot == pair->map_observation_is_snapshot &&
        path_owner == pair->path_owner &&
        frame_owner == pair->frame_owner &&
        frozen_cloud_occupancy_snapshot ==
            pair->frozen_cloud_occupancy_snapshot &&
        full_path_samples == pair->full_path_samples &&
        active_profile == pair->active_profile &&
        epoch_snapshot == pair->epoch_snapshot;
  }
};

struct PathTubePairPinRegistry;

// One exclusive, transaction-owned H2 lease.  It has no control/runtime
// semantics: holding it only prevents timer pair-authority replacement while
// a manager builds and validates a successor from the exact captured pair.
// The registry outlives the adapter so a late guard destruction after
// shutdown is harmless.  Release is idempotent and ABA-safe by lease id plus
// exact pair/generation/session identity.
class PathTubePairPin {
 public:
  PathTubePairPin() = default;
  ~PathTubePairPin();
  PathTubePairPin(const PathTubePairPin&) = delete;
  PathTubePairPin& operator=(const PathTubePairPin&) = delete;
  PathTubePairPin(PathTubePairPin&& other) noexcept;
  PathTubePairPin& operator=(PathTubePairPin&& other) noexcept;

  bool valid() const { return lease_id_ != 0U && capture_.valid(); }
  std::uint64_t leaseId() const { return lease_id_; }
  const PathTubePairPinCapture& capture() const { return capture_; }
  void release();

 private:
  friend class PhaseOffsetMatchedAdapter;
  PathTubePairPin(const std::shared_ptr<PathTubePairPinRegistry>& registry,
                  const PathTubePairPinCapture& capture,
                  std::uint64_t lease_id);

  std::shared_ptr<PathTubePairPinRegistry> registry_;
  PathTubePairPinCapture capture_;
  std::uint64_t lease_id_ = 0U;
};

// A staged pair is never an authority.  It keeps the old authority alive for
// identity revalidation and is consumed only by the command boundary.
struct PathTubePairTransaction {
  std::shared_ptr<const PathTubePair> expected_pair;
  std::shared_ptr<const PathTubePair> candidate_pair;
  // Stage 2 copies capture evidence and its lease identity only.  Ownership
  // remains with the manager's pending frontend guard, never with a staging
  // value that could be copied or outlive a cancelled transaction.
  PathTubePairPinCapture expected_capture;
  std::uint64_t pin_lease_id = 0U;
  std::uint64_t authority_session = 0U;
  double captured_retained_delta = 0.0;
  phase_offset_core::PortCommand captured_previous_final_port;
};

// Result of the lock-free half of an H2 handoff commit.  It contains no live
// Runtime reference: all expensive owner evaluation, map checking and exact
// port dry-run have already completed against a local Runtime copy.  The
// manager serializes the short final Runtime/pair CAS with its phase tuple.
struct PathTubePairCommitPreparation {
  PathTubePairTransaction transaction;
  double current_w = 0.0;
  // Private-to-the-adapter commit predicate.  These are captured at the
  // command-boundary prepare step, never copied from the older transaction
  // pin capture, and are not Runtime state to be installed by pair CAS.
  double expected_retained_delta = 0.0;
  phase_offset_core::PortCommand expected_previous_final_port;
  bool valid = false;
};

// Temporary A6/H2 first-false attribution only.  This records no authority,
// runtime, or ROS-facing state; it lets the manager distinguish a transaction
// precondition from raw Tube, filter, validator, coverage, or dry-run denial.
// Remove after the live bootstrap first-false is attributed.
enum class PathTubePairStageFailure {
  NONE,
  INPUT_PRECONDITION,
  TRANSACTION_PRECONDITION,
  PAIR_SESSION_RUNTIME_SNAPSHOT,
  OWNER_EVALUATE,
  TUBE_BUILD_PRECONDITION,
  TUBE_RAW_BUILD,
  TUBE_FILTER,
  TUBE_SURFACE_VALIDATOR,
  TUBE_PROFILE_COVERAGE,
  TUBE_PROFILE_OWNER_MATCH,
  STAGING_DRY_RUN,
};

// Private timer-refresh prepare/finalize value.  It is not a control state or
// an authority: it merely preserves the exact candidate and latest command /
// Runtime predicate that the short final pair CAS must revalidate.
struct TimerPairRefreshPreparation {
  std::shared_ptr<const TubeBuildRequest> build_request;
  std::shared_ptr<const TubeBuildRequest> latest_request;
  std::shared_ptr<const TubeEpochSnapshot> epoch;
  std::shared_ptr<const PathTubePair> expected_pair;
  std::uint64_t authority_session = 0U;
  double expected_retained_delta = 0.0;
  phase_offset_core::PortCommand expected_previous_final_port;
  bool valid = false;
};

struct MatchedAdapterInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  phase_offset_core::PathDifferentialState path;
  std::vector<phase_offset_core::PathDifferentialState,
              Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>
      sampled_path;
  std::function<bool(double, std::vector<double>&,
                     std::vector<ContinuousPhasePathState>&)> sample_path;
  // Optional exact pure-C++ evaluator for a synthetic/test or adapter-owned
  // semantic path.  Production normally binds semantic_path below.
  phase_offset_navigation::PathStateQuery path_state_query;
  // Command-thread-only compatibility inputs.  They are never retained in a
  // TubeBuildRequest: the timer receives semantic_path_owner and/or immutable
  // sampled_path values instead.
  const ContinuousPhasePath* semantic_path = nullptr;
  const void* semantic_path_identity = nullptr;
  double semantic_path_start_w = 0.0;
  double semantic_path_end_w = 0.0;
  // Production ownership handoff.  The manager captures these immutable
  // owners in the command callback before storing a build request.
  std::shared_ptr<const ContinuousPhasePath> semantic_path_owner;
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame_owner;
  // When H2 authority is installed this is the sole path/tube source for the
  // control step.  It is intentionally immutable and never synthesized from
  // independent path/epoch slots inside update().
  std::shared_ptr<const PathTubePair> path_tube_pair;
  // A manager-staged successor is transaction evidence only.  It is supplied
  // from the exact pending PathTubePair handoff and is never installed or
  // treated as authority by the adapter.  Recovery/Preview may validate it
  // while the current pair remains the safe executed owner.
  std::shared_ptr<const PathTubePair> successor_path_tube_pair;
  std::shared_ptr<const plan_env::CloudOccupancySnapshot>
      cloud_occupancy_snapshot;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  guidance::IsfGains gains;
  LegacyGuidanceSnapshot legacy;
  double dt = 0.0;
  ros::Time stamp;
};

using ManualPreflightResult = phase_offset_navigation::ManualPreflightResult;

enum ManualDiagnosticIndex : std::size_t {
  kModeManual = 0U,
  kZeroGateOpen,
  kZeroGateConsecutiveCount,
  kFailureLatched,
  kProfileActive,
  kPreflightComplete,
  kConfiguredAmplitude,
  kAcceptedAmplitude,
  kDeltaRef,
  kDelta,
  kDeltaTrackingError,
  kUWRaw,
  kUDeltaRaw,
  kUWFinal,
  kUDeltaFinal,
  kUWLimited,
  kUDeltaLimited,
  kBaseWDot,
  kFinalWDot,
  kBaseTangentSpeed,
  kFinalTangentSpeed,
  kCurrentRegularity,
  kNextRegularity,
  kMatchedResidualNorm,
  kPhysicalPortNorm,
  kRMinusPNorm,
  kRZMinusPZ,
  kSelectedManual,
  kManualValid,
  kPreflightSampleCount,
  kPreflightInvalidSampleCount,
  kPreflightMinRegularity,
  kManualInvalidCount,
  kManualFallbackCount,
  kManualLegacyDiagnosticCount,
  kTubeSource = kManualLegacyDiagnosticCount,
  kTubeSourceReady,
  kTubeRawComplete,
  kTubeFilteredComplete,
  kTubeProfileComplete,
  kTubeObstacleCertified,
  kTubeSourceRevision,
  kTubeRevision,
  kTubeSampleCount,
  kTubeInvalidCount,
  kTubeUnavailableCount,
  kTubeOutOfMapCount,
  kTubeUnknownCount,
  kTubeOccupiedCount,
  kTubePreviewStart,
  kTubePreviewEnd,
  kTubeCertifiedForward,
  kTubeLower,
  kTubeUpper,
  kTubeLowerW,
  kTubeUpperW,
  kTubeNextLower,
  kTubeNextUpper,
  kTubeCurrentInside,
  kTubeNextInside,
  kTubeUpperH,
  kTubeLowerH,
  kTubeUpperInvariant,
  kTubeLowerInvariant,
  kTubeRequiredReferenceClearance,
  kTubeRequiredActualClearance,
  kTubeReferenceDistance,
  kTubeActualDistance,
  kTubeTrackingNorm,
  kTubeTrackingBound,
  kTubeViolation,
  kTubeRebuildCount,
  kTubeRejectCount,
  kTubeViolationCount,
  kTubeReadinessEvaluated,
  kTubeDisplayCertified,
  kPreflightFirstInvalidW,
  kPreflightFirstInvalidSide,
  kTubeFirstInvalidW,
  kTubeFirstInvalidSide,
  kTubeFirstStopReason,
  kTubeInsufficientClearanceCount,
  kTubeMinWidth,
  kTubeMinSafetyMargin,
  kManualDiagnosticCount,
};

static_assert(kManualDiagnosticCount == 83U,
              "manual diagnostics must remain exactly 83 fields");

struct MatchedAdapterOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  guidance::IsfGuidance guidance;
  guidance::IsfGuidance base_guidance;
  phase_offset_core::PhaseOffsetGeometryState geometry;
  phase_offset_core::PortCommand raw_port;
  phase_offset_core::PortProjectionResult projection;
  phase_offset_core::MatchedPortOutput matched;
  ActiveAdapterOutput zero_port;
  ActiveEquivalenceResult zero_comparison;
  ManualPreflightResult preflight;
  // Candidate and installed active ownership are intentionally distinct.
  std::shared_ptr<const phase_offset_navigation::TubeProfile> candidate_profile;
  std::shared_ptr<const phase_offset_navigation::TubeProfile> active_profile;
  // Compatibility view: legacy 83-field tube entries always read active.
  phase_offset_navigation::TubeProfile tube_profile;
  phase_offset_navigation::TubeBounds tube_current_bounds;
  phase_offset_navigation::TubeBounds tube_next_bounds;
  phase_offset_navigation::TubeRuntimeStatus tube_status;
  phase_offset_navigation::TubeEpochStatus tube_epoch_status;
  phase_offset_navigation::RuntimeExecutionStatus runtime_execution;
  double delta_ref = 0.0;
  double delta = 0.0;
  bool zero_gate_open = false;
  int zero_gate_consecutive_count = 0;
  bool failure_latched = false;
  phase_offset_navigation::ControlFailureReason control_failure_reason =
      phase_offset_navigation::ControlFailureReason::NONE;
  bool profile_active = false;
  bool tube_update_due_this_cycle = false;
  // Independent raw-candidate diagnostics are available only for the due
  // MANUAL+ESDF attempt that generated them; they are not an 83/49 extension.
  bool raw_candidate_diagnostics_generated = false;
  std::array<double, kRawCandidateDiagnosticCount> raw_candidate_diagnostics = {{0.0}};
  bool cloud_snapshot_diagnostics_generated = false;
  std::array<double, kCloudSnapshotDiagnosticCount>
      cloud_snapshot_diagnostics = {{0.0}};
  bool selected = false;
  bool valid = false;
  phase_offset_navigation::RecoveryStepStatus recovery_status =
      phase_offset_navigation::RecoveryStepStatus::NONE;
  bool recovery_replan_required = false;
  std::string invalid_reason;
  std::array<double, kManualDiagnosticCount> diagnostics = {{0.0}};
};

// Immutable command -> timer handoff.  In particular it intentionally has no
// SDFMap pointer, raw ContinuousPhasePath pointer, callback, or query object.
// The timer may create local query adapters from the owned semantic path and
// immutable cloud snapshot while it is building one epoch.
struct TubeBuildRequest {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool active = false;
  // New navigation tasks are distinct timer-owned evidence domains even when
  // a planner happens to restart its local source revision at the same value.
  // This token is private to the adapter and is never a ROS-facing field.
  std::uint64_t task_generation = 0U;
  std::uint64_t control_sequence = 0U;
  std::uint64_t source_revision = 0U;
  std::uint64_t authority_session = 0U;
  std::uint64_t map_observation_sequence = 0U;
  bool map_observation_is_snapshot = false;
  ros::Time stamp;
  std::shared_ptr<const ContinuousPhasePath> semantic_path_owner;
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame_owner;
  // H2 same-path refresh is anchored to this immutable authority.  The timer
  // may only replace its tube/profile if this exact pair is still live.
  std::shared_ptr<const PathTubePair> base_path_tube_pair;
  std::uint64_t base_path_tube_pair_generation = 0U;
  double base_retained_delta = 0.0;
  phase_offset_core::PortCommand base_previous_final_port;
  double semantic_path_start_w = 0.0;
  double semantic_path_end_w = 0.0;
  std::shared_ptr<const MatchedAdapterPathSamples> supplied_path_samples;
  phase_offset_core::PathDifferentialState current_path;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  guidance::IsfGains gains;
  double dt = 0.0;
  double retained_delta = 0.0;
  phase_offset_navigation::TubeBounds authority_request;
  std::shared_ptr<const plan_env::CloudOccupancySnapshot> cloud_snapshot;
};

// Immutable timer -> command handoff.  It records the exact source/map facts
// used for the build, as well as the diagnostic payloads that are pending a
// timer-only publication after a command cycle has consumed this snapshot.
struct TubeEpochSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool active = false;
  std::uint64_t task_generation = 0U;
  std::uint64_t request_control_sequence = 0U;
  std::uint64_t build_sequence = 0U;
  std::uint64_t source_revision = 0U;
  std::uint64_t map_observation_sequence = 0U;
  bool map_observation_is_snapshot = false;
  ros::Time request_stamp;
  ros::Time completion_stamp;
  std::shared_ptr<const MatchedAdapterPathSamples> full_path_samples;
  std::shared_ptr<const phase_offset_navigation::TubeProfile> candidate_profile;
  std::shared_ptr<const phase_offset_navigation::TubeProfile> active_profile;
  phase_offset_navigation::TubeEpochStatus epoch_status;
  CloudOccupancyQueryStatus cloud_status;
  bool raw_candidate_diagnostics_generated = false;
  std::array<double, kRawCandidateDiagnosticCount>
      raw_candidate_diagnostics = {{0.0}};
  bool cloud_snapshot_diagnostics_generated = false;
  std::array<double, kCloudSnapshotDiagnosticCount>
      cloud_snapshot_diagnostics = {{0.0}};
};

// Private staging value for a future-seam handoff.  It is deliberately not a
// published epoch: all profiles are newly built from its supplied new-owner
// samples and it carries no Candidate/Active authority.
struct PreparedTubeBuildResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool complete = false;
  std::uint64_t source_revision = 0U;
  std::uint64_t map_observation_sequence = 0U;
  bool map_observation_is_snapshot = false;
  // Keep the semantic owner and immutable map evidence that were used for
  // this uncommitted build.  A PreparedTubeBuildResult must never rely on a
  // stack-local query closure surviving the staging manager.
  std::shared_ptr<const ContinuousPhasePath> semantic_path_owner;
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame_owner;
  std::shared_ptr<const plan_env::CloudOccupancySnapshot>
      frozen_cloud_occupancy_snapshot;
  // These are facts verified from the actual prepared samples/profile, not
  // caller assertions.  H2-3 uses them to revalidate a future-seam commit.
  double prepared_start_w = 0.0;
  double prepared_end_w = 0.0;
  double captured_w0 = 0.0;
  double future_seam_w = 0.0;
  double existing_future_horizon_end_w = 0.0;
  std::shared_ptr<const MatchedAdapterPathSamples> full_path_samples;
  std::shared_ptr<const phase_offset_navigation::TubeProfile> active_profile;
  phase_offset_navigation::TubeEpochStatus epoch_status;
};

// Transaction-local owner proof used only to avoid re-evaluating an exact
// captured knot while H2 stages one immutable replacement.  It is never stored
// in Runtime, Pair, or any cross-callback cache.
struct CanonicalOwnerStateReuse {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::shared_ptr<const ContinuousPhasePath> owner;
  phase_offset_core::PathDifferentialState state;
  const MatchedAdapterPathSamples* verified_samples = nullptr;
  std::uint64_t task_generation = 0U;
  std::uint64_t source_revision = 0U;
  std::uint64_t authority_session = 0U;
  std::uint64_t map_observation_sequence = 0U;
  bool map_observation_is_snapshot = false;
  bool valid = false;
};

// Immutable command -> timer publication handoff.  `epoch_snapshot` is the
// exact epoch that Runtime consumed for this control step; the remaining
// fields are value copies so marker/83/50 publication never reads Runtime.
struct ControlPublishSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // These are the command-cycle facts which created this publication
  // snapshot.  The candidate epoch below retains its own immutable map
  // provenance: a newer observation does not invalidate a completed,
  // same-source profile by itself.
  bool active = false;
  std::uint64_t task_generation = 0U;
  std::uint64_t control_sequence = 0U;
  std::uint64_t source_revision = 0U;
  std::uint64_t map_observation_sequence = 0U;
  bool map_observation_is_snapshot = false;
  std::uint64_t candidate_epoch_source_revision = 0U;
  std::uint64_t candidate_epoch_map_observation_sequence = 0U;
  bool candidate_epoch_map_observation_is_snapshot = false;
  // True means the current request cannot use the epoch as Runtime input
  // under the existing cloud-observation contract.  It is publishable
  // Candidate evidence, never a Runtime or Certified input.
  bool candidate_only = false;
  std::uint64_t epoch_build_sequence = 0U;
  ros::Time stamp;
  // Exact command-cycle phase used to select the same epoch's raw candidate
  // display range.  It is immutable publication provenance, not a gate.
  double current_w = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  std::shared_ptr<const TubeEpochSnapshot> epoch_snapshot;
  std::shared_ptr<const MatchedAdapterPathSamples> full_path_samples;
  // Command/diagnostic publication is tied to one immutable execution
  // authority snapshot.  The timer consumes this value-only handoff and
  // cannot observe a different selected-u transaction.
  std::shared_ptr<const phase_offset_navigation::ActiveReferenceSnapshot>
      authority_snapshot;
  MatchedAdapterOutput output;
};

struct DeactivationCommitToken {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t task_generation = 0U;
  std::uint64_t next_control_sequence = 0U;
  bool command_active_before = false;
  bool valid = false;
  std::shared_ptr<const TubeBuildRequest> inactive_request;
  std::shared_ptr<const TubeEpochSnapshot> inactive_candidate_snapshot;
  std::shared_ptr<const TubeEpochSnapshot> inactive_epoch_snapshot;
  std::shared_ptr<const ControlPublishSnapshot> inactive_control;
};

// Immutable command-side capture used to bind the governor's reference query
// to the exact staged PositionCommand transaction that may publish it.  A
// capture with pending=false/valid=true means no transaction was present at
// capture time; pending=true requires the identity to remain unchanged until
// publish.
struct PendingPositionCommandCapture {
  bool pending = false;
  bool valid = false;
  std::uint64_t identity = 0U;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr reference_query;
};

struct MatchedAdapterMarkerBundle {
  visualization_msgs::Marker base_path;
  visualization_msgs::Marker active_path;
  visualization_msgs::MarkerArray frame;
  visualization_msgs::MarkerArray tube;
  visualization_msgs::MarkerArray tube_candidate;
};

class PhaseOffsetMatchedAdapter {
 public:
  explicit PhaseOffsetMatchedAdapter(const PhaseOffsetMatchedAdapterConfig& config);
  ~PhaseOffsetMatchedAdapter();

  static PhaseOffsetMatchedAdapterConfig loadConfig(
      ros::NodeHandle& private_nh, PhaseOffsetMatchedMode mode);
  bool configurationValid() const;
  bool requiresPathSamples() const;
  double sampleStepW() const;
  bool requiresTubeTimer() const;
  // Tube observation remains enabled for MANUAL mode.  This predicate is only
  // true after an offset profile has executed (or a nonzero retained delta is
  // live); pending configuration alone must not force terminal H2 denial.
  bool requiresAuthoritativeOffsetHandoff() const;
  // The activation edge is the only time a neutral Runtime may request its
  // first PathTubePair.  Before this edge, configured manual amplitude is
  // observation/intent only and the planner remains the command owner.
  // A true result means the zero-port gate is open, Runtime is still neutral,
  // no pair is installed, and the pending profile must be proven atomically
  // before it can issue its first nonzero port.
  bool requiresPathTubePairBootstrap() const;
  // A committed pair may be passed to Runtime for the one activation command
  // even though Runtime has not yet committed its first nonzero port.  The
  // command guidance remains on the planner owner until that commit occurs.
  bool hasPendingOffsetActivationPair(
      const std::shared_ptr<const PathTubePair>& pair) const;
  // Begin bounded in-owner recentering after a successor denial.  This does
  // not clear the pair, delta, or planner authority; Runtime retires the
  // profile only after an exact accepted step reaches neutral.
  bool requestRecenter();
  bool recenterRequested() const;
  // A bootstrap has no old pair whose certified end can be retained.  Reuse
  // the existing manager installation horizon and cap it by the immutable
  // path; this is not a new handoff threshold.
  double bootstrapPreparedHorizonEnd(double current_w,
                                     double future_seam_w,
                                     double path_end_w) const;
  // Captures the single H2 control authority.  A non-null return value is
  // immutable for the caller's entire command; consumers must not combine it
  // with the legacy GVF path mirror or the timer's evidence slots.
  std::shared_ptr<const PathTubePair> capturePathTubePair() const;
  // Atomically captures the exact authoritative pair and Runtime history,
  // then acquires the sole H2 transaction lease before returning.  It is the
  // required entry point before any future seam enumeration or connector
  // work; null means no exact pair or another transaction already owns it.
  std::unique_ptr<PathTubePairPin> captureAndAcquirePathTubePairPin();
  // Retires only H2 ownership/timer evidence.  Runtime's retained delta and
  // previous final port deliberately survive.  If executed offset authority
  // survives, it remains unpaired/fail-closed until an authorized recovery;
  // only pending neutral intent may use a following bootstrap.
  // `authority_session` is manager-owned and monotonically changes on each
  // goal/reset.
  std::uint64_t retirePathTubeAuthority(std::uint64_t authority_session);
  // A user-issued navigation goal is a new path-coordinate task, not an H2
  // replacement.  Under the command boundary it retires all old authority
  // and neutralizes only Runtime task history.  The exact expected session
  // prevents a stale goal callback from clearing a newer task.
  bool resetForNewNavigationTask(std::uint64_t expected_authority_session,
                                 std::uint64_t& retired_authority_session);
  void advertise(ros::NodeHandle& private_nh);
  bool update(const MatchedAdapterInput& input, MatchedAdapterOutput& output);
  // Discard only a staged production transaction.  Authority/Runtime commit
  // is intentionally unreachable through a public boolean bypass; production
  // commit occurs only inside publishPendingPositionCommand() after the local
  // PositionCommand publication succeeds.
  void discardPendingPositionCommand();
  bool hasPendingPositionCommand() const;
  bool validatePendingPositionCommand() const;
  PendingPositionCommandCapture capturePendingPositionCommand() const;
  // Returns the immutable executed-reference query staged for the pending
  // PositionCommand transaction.  The governor may consume this value only
  // after validatePendingPositionCommand(); publication revalidates it again
  // under the serialized task/runtime boundary.
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
  pendingExecutedReferenceQuery() const;
  // Serializes final validation, local PositionCommand publication and the
  // no-fail authority/token commit against task reset and pair retirement.
  bool publishPendingPositionCommand(
      const std::function<bool()>& local_publish,
      std::uint64_t expected_identity = 0U,
      bool cancel_pending_for_goal_override = false);
  // Called only by the manager's 10 Hz MANUAL timer (or deterministically by
  // an owning test).  It is non-reentrant and owns all TubeEpochManager work.
  bool timerTick();
  // Command-thread transition used when no goal/path is active.  It clears
  // only async request/tube exposure and queues one timer-side DELETE; Runtime
  // execution state (delta, previous port, profile lifecycle) is preserved.
  void deactivate(const ros::Time& stamp = ros::Time(0));
  // Manager shutdown order is: set this flag, stop the ROS timer, wait for an
  // in-flight tick, clear the atomic snapshots, then destroy this adapter.
  void requestShutdown();
  void shutdown();
  bool buildMarkers(const MatchedAdapterInput& input,
                    const MatchedAdapterOutput& output,
                    MatchedAdapterMarkerBundle& markers) const;

 private:
  using PathSamples = MatchedAdapterPathSamples;
  struct TubeDueTimingSample {
    std::uint64_t sequence = 0U;
    std::uint64_t steady_duration_ns = 0U;
    std::uint64_t ros_stamp_ns = 0U;
    bool source_current_finalized = false;
    bool raw_cloud_publish_attempted = false;
  };

  bool collectSamples(const TubeBuildRequest& request,
                      PathSamples& full_path) const;
  bool makePreview(const PathSamples& full_path,
                   const phase_offset_core::PathDifferentialState& current,
                   PathSamples& preview) const;
  std::uint64_t sourceRevision(const MatchedAdapterInput& input);
  bool requiresAuthoritativeOffsetHandoffLocked() const;
  bool requiresPathTubePairBootstrapLocked() const;
  bool hasPendingOffsetActivationPairLocked(
      const std::shared_ptr<const PathTubePair>& pair) const;
  std::uint64_t retirePathTubeAuthorityLocked(
      std::uint64_t authority_session);
  bool retirePathTubeAuthorityIfNeutral(
      std::uint64_t authority_session,
      std::uint64_t& retired_session);
  void deactivateLocked(const ros::Time& stamp,
                        std::uint64_t expected_task_generation);
  bool prepareDeactivationLocked(
      const ros::Time& stamp, std::uint64_t expected_task_generation,
      DeactivationCommitToken& token) const;
  void commitDeactivationNoFailLocked(
      const DeactivationCommitToken& token) noexcept;
  bool buildTubeEpoch(const std::shared_ptr<const TubeBuildRequest>& request,
                      TubeEpochSnapshot& snapshot);
  bool buildPreparedTubeEpoch(const TubeBuildRequest& request,
                              const MatchedAdapterPathSamples& prepared_path,
                              double prepared_start_w,
                              double prepared_end_w,
                              double future_seam_w,
                              double existing_future_horizon_end_w,
                              PreparedTubeBuildResult& result,
                              phase_offset_navigation::TubeEpochUpdateResult*
                                  temporary_epoch_result = nullptr,
                              std::string* temporary_failure_layer = nullptr,
                              const CanonicalOwnerStateReuse*
                                  canonical_owner_state = nullptr) const;
  // Attribution only: classify a failed staged update without changing Tube
  // acceptance, Runtime, or the builder/validator pipeline.
  static PathTubePairStageFailure classifyTubeUpdateStatusStageFailure(
      const phase_offset_navigation::TubeEpochUpdateResult& staged);
  bool dryRunPreparedRuntime(
      const PreparedTubeBuildResult& prepared_tube,
      const phase_offset_core::PathDifferentialState& current_path,
      const Eigen::Vector3d& position,
      const guidance::IsfGains& gains,
      double dt,
      phase_offset_navigation::RuntimeDryRunResult& result) const;
  bool dryRunPreparedRuntime(
      phase_offset_navigation::PhaseOffsetRuntime& runtime_snapshot,
      const PreparedTubeBuildResult& prepared_tube,
      const phase_offset_core::PathDifferentialState& current_path,
      const Eigen::Vector3d& position,
      const guidance::IsfGains& gains,
      double dt,
      phase_offset_navigation::RuntimeDryRunResult& result) const;
  bool stagePathTubePair(
      const std::shared_ptr<const PathTubePair>& expected_pair,
      const std::shared_ptr<const ContinuousPhasePath>& new_path_owner,
      const MatchedAdapterPathSamples& new_path_samples,
      double captured_w0,
      double future_seam_w,
      double existing_future_horizon_end_w,
      const Eigen::Vector3d& position,
      const guidance::IsfGains& gains,
      double dt,
      const std::shared_ptr<const plan_env::CloudOccupancySnapshot>&
          frozen_cloud_occupancy_snapshot,
      PathTubePairTransaction& transaction,
      std::uint64_t authority_session = 0U,
      const PathTubePairPinCapture* expected_capture = nullptr,
      std::uint64_t pin_lease_id = 0U,
      PathTubePairStageFailure* temporary_failure = nullptr);
  bool preparePathTubePairCommit(
      const PathTubePairTransaction& transaction,
      double current_w,
      const Eigen::Vector3d& position,
      const guidance::IsfGains& gains,
      double dt,
      const std::shared_ptr<const plan_env::CloudOccupancySnapshot>&
          latest_cloud_occupancy_snapshot,
      PathTubePairCommitPreparation& preparation);
  bool finalizePreparedPathTubePairCommit(
      const PathTubePairCommitPreparation& preparation,
      std::shared_ptr<const PathTubePair>& committed_pair);
  // Timer-only completion protocol.  It accepts a completed epoch when its
  // immutable path source is still current; its frozen map snapshot remains
  // the epoch's provenance rather than an equality key for later commands.
  bool finalizeTubeEpoch(const std::shared_ptr<const TubeBuildRequest>& request,
                         const TubeEpochSnapshot& snapshot);
  bool requestSourceStillCurrent(const TubeBuildRequest& request) const;
  bool requestCloudSnapshotUsable(const TubeBuildRequest& request) const;
  bool epochMatchesRequest(const TubeEpochSnapshot& epoch,
                           const TubeBuildRequest& request) const;
  // Timer-only task-boundary consumption.  The command callback publishes a
  // new generation but never mutates these timer-owned objects directly.
  void consumeTimerTaskGeneration(std::uint64_t task_generation);
  bool refreshPairFromTimerEpoch(
      const std::shared_ptr<const TubeBuildRequest>& request,
      const std::shared_ptr<const TubeEpochSnapshot>& epoch);
  bool prepareTimerPairRefresh(
      const std::shared_ptr<const TubeBuildRequest>& request,
      const std::shared_ptr<const TubeEpochSnapshot>& epoch,
      TimerPairRefreshPreparation& preparation);
  bool finalizePreparedTimerPairRefresh(
      const TimerPairRefreshPreparation& preparation,
      std::shared_ptr<const PathTubePair>& refreshed_pair);
  std::shared_ptr<const TubeEpochSnapshot> makeCandidateOnlyEpoch(
      const TubeEpochSnapshot& epoch) const;
  bool makeAuthorityRequest(
      double retained_delta,
      phase_offset_navigation::TubeBounds& authority_request) const;
  std::shared_ptr<const TubeBuildRequest> makeBuildRequest(
      const MatchedAdapterInput& input,
      std::uint64_t source_revision,
      double retained_delta = 0.0);
  std::shared_ptr<const TubeEpochSnapshot> matchingEpochForRequest(
      const std::shared_ptr<const TubeBuildRequest>& request) const;
  void makeControlPublishSnapshot(
      const MatchedAdapterInput& input,
      const std::shared_ptr<const TubeBuildRequest>& request,
      const std::shared_ptr<const TubeEpochSnapshot>& epoch,
      bool candidate_only,
      const MatchedAdapterOutput& output);
  bool updateGate(const MatchedAdapterInput& input, MatchedAdapterOutput& output);
  void fillLegacyTubeStatus(MatchedAdapterOutput& output) const;
  bool tubeDisplayCertified(const MatchedAdapterOutput& output) const;
  void fillManualDiagnostics(MatchedAdapterOutput& output) const;
  // Runtime::complete is evaluated only on a local value copy.  The exact
  // selected port and resulting state cross the serialized execution
  // authority before the live Runtime is replaced; a rejected authority
  // transaction therefore cannot mutate command state.
  bool completeThroughExecutionAuthority(
      const MatchedAdapterInput& input,
      const std::shared_ptr<const PathTubePair>& pair,
      const std::shared_ptr<const TubeBuildRequest>& request,
      const std::shared_ptr<const TubeEpochSnapshot>& epoch,
      const phase_offset_navigation::RuntimePreparedStep& prepared,
      const Eigen::Vector3d& base_v_cmd,
      double base_w_dot,
      bool base_guidance_valid,
      phase_offset_navigation::RuntimeStepOutput& output);
  bool completeThroughRecoveryOwner(
      const MatchedAdapterInput& input,
      const std::shared_ptr<const PathTubePair>& pair,
      const std::shared_ptr<const PathTubePair>& successor_pair,
      const phase_offset_navigation::RuntimePreparedStep& prepared,
      const Eigen::Vector3d& base_v_cmd,
      double base_w_dot,
      bool base_guidance_valid,
      phase_offset_navigation::RuntimeStepOutput& output);
  bool completeAtomicNeutralHandoff(
      const MatchedAdapterInput& input,
      const std::shared_ptr<const PathTubePair>& pair,
      const phase_offset_navigation::RuntimePreparedStep& prepared,
      const Eigen::Vector3d& base_v_cmd,
      double base_w_dot,
      bool base_guidance_valid,
      phase_offset_navigation::RuntimeStepOutput& output);
  void clearPendingPositionCommandLocked();
  bool validatePendingPositionCommandLocked(std::string* reason) const;
  static bool exactLivePairPublicationControl(
      const ControlPublishSnapshot& control,
      const TubeBuildRequest& request,
      const std::shared_ptr<const PathTubePair>& live_pair,
      std::uint64_t current_task_generation,
      std::uint64_t current_authority_session);
  bool buildMarkers(const ControlPublishSnapshot& control,
                    MatchedAdapterMarkerBundle& markers) const;
  bool buildMarkers(
      const ControlPublishSnapshot& control,
      const std::shared_ptr<const phase_offset_navigation::TubeProfile>&
          candidate_marker_profile,
      MatchedAdapterMarkerBundle& markers) const;
  bool markEpochBuildPublished(std::uint64_t build_sequence);
  void publishBuildDiagnostics(const TubeEpochSnapshot& epoch);
  void publishManual(const ControlPublishSnapshot& control);
  void publishManualDelete(const ControlPublishSnapshot& control);
  void latchFailure(phase_offset_navigation::ControlFailureReason reason);
  void recordTubeDueTiming(std::uint64_t steady_duration_ns,
                           std::uint64_t ros_stamp_ns,
                           bool source_current_finalized,
                           bool raw_cloud_publish_attempted);
  void flushTubeDueTiming();

  friend class gvf_manager;
  friend class GvfManagerS4AnchorTestAccess;

  PhaseOffsetMatchedAdapterConfig config_;
  bool configuration_valid_ = false;
  // Command callback ownership: Runtime, gate/latch, source revision, and
  // control-publication snapshot construction never run on the tube timer.
  PhaseOffsetActiveAdapter zero_port_adapter_;
  std::unique_ptr<phase_offset_navigation::PhaseOffsetRuntime> runtime_;
  // Sole mutable execution-state authority for selected-u/runtime commits.
  // H2 pair ownership remains a separate immutable handoff contract.
  phase_offset_navigation::PhaseOffsetExecutionAuthority execution_authority_;
  phase_offset_navigation::PhaseOffsetRecoveryOwner recovery_owner_;
  phase_offset_navigation::HandoffStateMachine handoff_state_machine_;
  // Deferred production transaction.  The token is a bounded value DTO,
  // never a copied Runtime/path/profile, and remains staging only until a
  // successful local PositionCommand publication is reported.
  phase_offset_navigation::RuntimeCommitToken pending_runtime_commit_;
  phase_offset_navigation::AuthorityPreparedStep pending_authority_prepared_;
  phase_offset_navigation::RecoveryPreparedStep pending_recovery_step_;
  bool pending_recovery_step_valid_ = false;
  phase_offset_navigation::HandoffStateInput pending_handoff_input_;
  phase_offset_navigation::HandoffDecision pending_handoff_decision_;
  bool pending_handoff_valid_ = false;
  // A RECOVERY snapshot may intentionally execute against a staged successor
  // before the manager can install that pair.  Keep both immutable pair
  // identities bound to the pending PositionCommand: the source is the exact
  // live predecessor captured for this tick, while execution is the exact
  // source/target pair whose geometry/query produced the candidate.  Final
  // publication accepts only these shared_ptr identities, never a same-
  // revision clone assembled from independent slots.
  std::shared_ptr<const PathTubePair> pending_recovery_source_pair_;
  std::shared_ptr<const PathTubePair> pending_recovery_execution_pair_;
  std::shared_ptr<const PathTubePair> pending_recovery_target_pair_;
  double recovery_deadline_ = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t recovery_deadline_session_ = 0U;
  std::uint64_t recovery_deadline_target_revision_ = 0U;
  std::uint64_t pending_authority_session_ = 0U;
  bool pending_authority_valid_ = false;
  // Runtime is command-owned.  Staging takes only a short snapshot/dry-run
  // lock; it never holds this lock while constructing a path or tube.
  mutable std::mutex runtime_command_mutex_;
  int zero_gate_consecutive_count_ = 0;
  bool zero_gate_open_ = false;
  bool failure_latched_ = false;
  phase_offset_navigation::ControlFailureReason control_failure_reason_ =
      phase_offset_navigation::ControlFailureReason::NONE;
  int manual_invalid_count_ = 0;
  int manual_fallback_count_ = 0;
  const void* source_identity_ = nullptr;
  double source_start_w_ = 0.0;
  double source_end_w_ = 0.0;
  bool have_source_identity_ = false;
  std::uint64_t source_revision_ = 0U;
  std::uint64_t last_preflight_revision_ = 0U;
  bool have_preflight_revision_ = false;
  std::uint64_t control_sequence_ = 0U;
  std::uint64_t last_consumed_epoch_build_sequence_ = 0U;
  bool command_active_ = false;

  // Cross-thread slots.  Every access uses std::atomic_load/store on the
  // shared_ptr; no raw pointer or mutable object is shared between callbacks.
  std::shared_ptr<const TubeBuildRequest> latest_build_request_;
  // Latest completed Candidate evidence, including its immutable map
  // provenance.  A later observation alone does not erase same-source
  // Candidate evidence.
  std::shared_ptr<const TubeEpochSnapshot> latest_candidate_epoch_snapshot_;
  // Runtime-eligible epoch only: it is stored only when the path source still
  // matches and the current request satisfies the existing cloud contract.
  std::shared_ptr<const TubeEpochSnapshot> latest_epoch_snapshot_;
  std::shared_ptr<const ControlPublishSnapshot> latest_control_snapshot_;
  // The sole H2 control authority.  Candidate/epoch evidence above remains
  // timer/display state only and is never consulted when an input carries a
  // PathTubePair.
  std::shared_ptr<const PathTubePair> authoritative_path_tube_pair_;
  std::uint64_t next_path_tube_pair_generation_ = 0U;
  // This registry represents transaction ownership rather than a Runtime
  // mode/gate.  Guards retain it independently so late teardown cannot touch
  // a destroyed adapter instance.
  std::shared_ptr<PathTubePairPinRegistry> path_tube_pin_registry_;
  // Lock-free reads from timer/build requests use this only as a retirement
  // generation.  Runtime/pair mutation remains serialized by
  // runtime_command_mutex_.
  std::atomic<std::uint64_t> authority_session_ {0U};
  // Linearizes new-task retirement against timer completion publication.
  // Both paths take this before runtime_command_mutex_; expensive tube build
  // remains outside both locks.
  mutable std::mutex task_publication_mutex_;
  std::function<void()> finalize_publication_test_hook_;
  std::function<void()> inactive_publication_test_hook_;
  std::function<void()> deactivate_test_hook_;
  // Published by the command-owned new-task reset.  A timer tick consumes it
  // before observing a request, so no TubeEpochManager/profile/cache state is
  // carried across navigation tasks.
  std::atomic<std::uint64_t> task_generation_ {1U};

  // Timer callback ownership: candidate construction, sampling/cache,
  // TubeEpochManager, cloud status, and pending diagnostics are timer-only.
  std::unique_ptr<phase_offset_navigation::TubeEpochManager> tube_epoch_manager_;
  PathSamples cached_full_path_samples_;
  std::uint64_t cached_path_source_revision_ = 0U;
  bool have_cached_path_ = false;
  std::shared_ptr<const phase_offset_navigation::TubeProfile>
      timer_active_profile_;
  std::uint64_t timer_installed_active_epoch_ = 0U;
  std::uint64_t timer_build_sequence_ = 0U;
  std::uint64_t timer_last_deactivate_sequence_ = 0U;
  std::uint64_t timer_last_publish_delete_source_revision_ = 0U;
  std::uint64_t timer_last_publish_delete_map_observation_sequence_ = 0U;
  std::uint64_t timer_last_published_epoch_build_sequence_ = 0U;
  std::uint64_t timer_last_raw_diagnostic_build_sequence_ = 0U;
  std::uint64_t timer_last_cloud_diagnostic_build_sequence_ = 0U;
  std::uint64_t timer_task_generation_ = 1U;
  std::atomic<bool> timer_inflight_ {false};
  std::atomic<bool> shutdown_requested_ {false};
  ros::Publisher active_diagnostics_pub_;
  ros::Publisher manual_base_path_pub_;
  ros::Publisher manual_active_path_pub_;
  ros::Publisher manual_frame_pub_;
  ros::Publisher manual_tube_pub_;
  ros::Publisher manual_tube_candidate_pub_;
  ros::Publisher manual_diagnostics_pub_;
  ros::Publisher manual_tube_epoch_diagnostics_pub_;
  ros::Publisher manual_raw_candidate_diagnostics_pub_;
  ros::Publisher manual_cloud_snapshot_diagnostics_pub_;
  CloudOccupancyQueryConfig cloud_occupancy_query_config_;
  CloudOccupancyQueryStatus latest_cloud_occupancy_query_status_;
  bool measurement_tube_due_enabled_ = false;
  std::mutex measurement_tube_due_mutex_;
  std::vector<TubeDueTimingSample> measurement_tube_due_samples_;
  std::uint64_t measurement_tube_due_sequence_ = 0U;
  bool advertised_ = false;
};

}  // namespace FLAG_Race
