#pragma once

#include "phase_offset_navigation/tube_epoch_types.h"
#include "phase_offset_navigation/tube_execution_v2.h"
#include "phase_offset_navigation/tube_filter.h"
#include "phase_offset_navigation/recovery_prepared_step.h"
#include "phase_offset_navigation/phase_offset_allocator.h"

#include <phase_offset_core/geometry.h>
#include <phase_offset_core/matched_port.h>
#include <phase_offset_core/port_projector.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace phase_offset_navigation {

using RuntimePathSamples =
    std::vector<phase_offset_core::PathDifferentialState,
                Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>;

struct ManualProfileConfig {
  double amplitude = 0.10;
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
};

// Execution-only tube data.  Candidate construction and filtering belong to
// TubeEpochManager, not this high-rate Runtime.
struct RuntimeTubeExecutionConfig {
  TubeSource source = TubeSource::NONE;
  double invariant_gain = 1.0;
  double interior_margin = 0.0;
  double tracking_error_bound = 0.15;
  double regularity_margin = 0.1;
  // Physical active-reference speed threshold.  Frame-bound geometry uses
  // this value; regularity_margin remains only for revision-0 planar data.
  double minimum_reference_speed = 1e-8;
};

struct PhaseOffsetRuntimeConfig {
  ManualProfileConfig manual;
  RuntimeTubeExecutionConfig tube;
};

struct ManualPreflightResult {
  bool complete = false;
  bool amplitude_reduced = false;
  std::size_t sample_count = 0U;
  std::size_t invalid_sample_count = 0U;
  double configured_amplitude = 0.0;
  double accepted_amplitude = 0.0;
  double min_regularity = 0.0;
  double max_abs_r_z_minus_p_z = 0.0;
  double first_invalid_w = 0.0;
  int first_invalid_side = 0;
  std::string reduction_reason;
  std::string invalid_reason;
};

struct RuntimePreflightInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RuntimePathSamples path;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  std::uint64_t path_source_revision = 0U;
};

struct RuntimeInstalledTubeView {
  std::shared_ptr<const TubeProfile> active_profile;
  TubeEpochStatus epoch_status;
};

// A navigation-only contract for one predicted Runtime step.  The adapter
// owns the immutable path provenance and the ISF kernel; Runtime owns the
// exact tube/port checks.  Keeping that split avoids exposing ROS or a
// B-spline implementation to phase_offset_navigation.
struct RuntimeFutureStepInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t step_index = 0U;
  double phase = 0.0;
  double delta = 0.0;
  phase_offset_core::PortCommand previous_final_port;
  // Predicted physical state after the preceding matched port.
  Eigen::Vector3d matched_position = Eigen::Vector3d::Zero();
  // The preceding active reference is carried explicitly so an adapter can
  // audit the reference transition without relying on a frozen current fact.
  Eigen::Vector3d previous_matched_reference = Eigen::Vector3d::Zero();
  double dt = 0.0;
};

struct RuntimeFutureStepResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  phase_offset_core::PathDifferentialState path;
  // The adapter recomputes these from its immutable owner at `path.w` using
  // the same ISF gains as the live command.  Runtime independently checks
  // that this reference agrees with its generic GeometryEvaluator result.
  Eigen::Vector3d matched_reference = Eigen::Vector3d::Zero();
  Eigen::Vector3d matched_tangent = Eigen::Vector3d::Zero();
  double matched_derivative_norm = 0.0;
  Eigen::Vector3d base_v_cmd = Eigen::Vector3d::Zero();
  double base_w_dot = 0.0;
  bool base_guidance_valid = false;
  bool valid = false;
  std::string invalid_reason;
};

using RuntimeFutureStepEvaluator = std::function<bool(
    const RuntimeFutureStepInput&, RuntimeFutureStepResult&)>;

// Values are bound by the adapter from already-approved tube settings.  They
// are not Runtime parameters and introduce neither a new mode nor a new
// safety threshold.
struct RuntimeFutureStepContract {
  RuntimeFutureStepEvaluator evaluate;
  double tube_update_period = 0.0;
  double min_certified_forward_w = 0.0;
  // The adapter derives this from the immutable active profile's certified
  // domain.  Runtime cross-checks it against that profile before rollout.
  double profile_domain_end_w = 0.0;
};

enum class RuntimeExecutionMode {
  NO_TUBE_REQUIRED,
  NORMAL,
  BLOCKED,
  WAITING_FOR_CANDIDATE,
  SAFETY_PRIORITY,
  CERTIFICATE_DENIED,
  FATAL_CONTROL_FAILURE,
};

struct RuntimeExecutionStatus {
  RuntimeExecutionMode mode = RuntimeExecutionMode::BLOCKED;
  bool active_profile_available = false;
  bool current_geometry_valid = false;
  bool current_bounds_valid = false;
  bool retained_delta_current_inside = false;
  bool tracking_within_bound = false;
  bool executable = false;
  bool fatal_control_failure = false;
  // This is a certificate denial, not an emergency control action.
  bool certificate_denied = false;
  bool transient_blocked = false;
  bool genuine_fatal_invariant = false;
  ControlFailureReason failure_reason = ControlFailureReason::NONE;
  double tracking_error_norm = 0.0;
  double tracking_error_bound = 0.0;
};

struct RuntimePrepareInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  phase_offset_core::PathDifferentialState current_path;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  RuntimeInstalledTubeView tube_view;
  double dt = 0.0;
  RuntimeFutureStepContract future_step;
  bool zero_gate_open = false;
  bool fatal_adapter_failure_latched = false;
};

struct RuntimePreparedStep {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  phase_offset_core::PhaseOffsetGeometryState geometry;
  phase_offset_core::PortCommand raw_port;
  std::shared_ptr<const TubeProfile> active_profile;
  TubeBounds current_bounds;
  RuntimeFutureStepContract future_step;
  ManualPreflightResult preflight;
  TubeEpochStatus epoch_status;
  RuntimeExecutionStatus execution;
  double delta = 0.0;
  double delta_ref = 0.0;
  double dt = 0.0;
  Eigen::Vector3d current_position = Eigen::Vector3d::Zero();
  bool profile_active = false;
  bool should_start_profile = false;
  // The arming gate is deliberately carried only to the final selection and
  // state-commit step.  It never controls tube geometry or exact-port
  // evaluation.
  bool zero_gate_open = false;
  bool requires_base_guidance = false;
  bool frame_bound = false;
  bool exact_terminal_predicate = false;
  bool valid = false;
  std::string invalid_reason;
};

struct RuntimeStepOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  phase_offset_core::PhaseOffsetGeometryState geometry;
  phase_offset_core::PortCommand raw_port;
  phase_offset_core::PortProjectionResult projection;
  phase_offset_core::MatchedPortOutput matched;
  std::shared_ptr<const TubeProfile> active_profile;
  TubeBounds current_bounds;
  TubeBounds next_bounds;
  ManualPreflightResult preflight;
  TubeEpochStatus epoch_status;
  RuntimeExecutionStatus execution;
  double delta = 0.0;
  double delta_ref = 0.0;
  bool profile_active = false;
  bool selected = false;
  bool exact_terminal_predicate = false;
  RecoveryStepStatus recovery_status = RecoveryStepStatus::NONE;
  bool recovery_replan_required = false;
  bool valid = false;
  std::string invalid_reason;
};

// Minimal no-allocation commit token for the publish-then-commit boundary.
// It carries only the exact next ZOH state and lifecycle facts; it never owns
// a path/profile copy or a second Runtime authority.
struct RuntimeCommitToken {
  phase_offset_core::PortCommand expected_previous_final_port;
  double expected_delta = 0.0;
  phase_offset_core::PortCommand next_previous_final_port;
  double next_delta = 0.0;
  double dt = 0.0;
  bool selected = false;
  bool valid = false;
  bool safety_priority = false;
  bool should_start_profile = false;
  bool profile_active = false;
  // Explicit post-publication terminalization for an already validated
  // atomic neutral handoff.  This is a value-only commit fact; completion is
  // never inferred from a pre-publication tolerance check.
  bool complete_profile = false;
  // Explicit proof that the selected recurrence arrived exactly at neutral;
  // this is never inferred from a tolerance in the commit seam.
  bool exact_terminal_predicate = false;
};

// One-shot exact-port evaluation over a caller-owned staging copy.  It carries
// no Runtime authority and therefore cannot change control state by itself.
struct RuntimeDryRunInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RuntimePreflightInput preflight;
  RuntimePrepareInput prepare;
  Eigen::Vector3d base_v_cmd = Eigen::Vector3d::Zero();
  double base_w_dot = 0.0;
  bool base_guidance_valid = false;
};

struct RuntimeDryRunResult {
  RuntimePreparedStep prepared;
  RuntimeStepOutput step;
  bool valid = false;
};

// Narrow value-only V2 handoff.  This path deliberately lives alongside the
// legacy Runtime API: adapters can stage an immutable V2 admission without
// exposing TubeExecutionGuardV2 (or any mutable authority) to the caller.
// Every value below is captured at preparation time and copied into the
// commit token.  The only Runtime state changed by a successful V2 commit is
// the existing exact ZOH successor (delta_/previous_final_port_) plus the
// immutable reserve evidence pointer.
struct RuntimeV2PrepareInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::shared_ptr<const TubeProfileV2> profile;
  // `identity` is the proposed immutable execution binding carried by the
  // profile.  A copied-prefix handoff may prepare that proposal while the
  // currently committed Runtime binding is still the source binding; in
  // that narrow case `binding_transition` is true and `expected_identity`
  // names the exact binding against which the publication CAS is made.
  // Ordinary CURRENT preparation leaves the flag false and retains the
  // original identity==expected-identity contract.
  TubeExecutionIdentityV2 identity;
  TubeExecutionIdentityV2 expected_identity;
  bool binding_transition = false;
  TubeExecutionStateV2 current;
  NormalPreviewProductionPolicy preview_policy;
  TubeExecutionLimitsV2 limits;
  phase_offset_core::PortCommand selected_u;
  std::string selected_u_owner = "PhaseOffsetAllocator";
  double base_phase_rate = std::numeric_limits<double>::quiet_NaN();
  // Immutable source phase-rate bounds captured with the selected command;
  // the profile's certified w-domain remains the sole phase-range authority.
  double phase_rate_lower = std::numeric_limits<double>::quiet_NaN();
  double phase_rate_upper = std::numeric_limits<double>::quiet_NaN();
  double horizon_w = std::numeric_limits<double>::quiet_NaN();
  double sample_spacing_w = std::numeric_limits<double>::quiet_NaN();
  double upper_u_delta = std::numeric_limits<double>::quiet_NaN();
  double dt = std::numeric_limits<double>::quiet_NaN();
  double now = std::numeric_limits<double>::quiet_NaN();
  double applicability_deadline = std::numeric_limits<double>::quiet_NaN();
  bool applicability_deadline_valid = false;
  TubeExecutionTrackingEvidenceV2 tracking;
  std::size_t max_work = 0U;
  std::string provenance;
};

struct RuntimeV2PreparedStep {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TubeStepAdmissionV2 admission;
  std::shared_ptr<const TubeProfileV2> profile;
  TubeExecutionIdentityV2 identity;
  TubeExecutionIdentityV2 expected_identity;
  bool binding_transition = false;
  TubeExecutionStateV2 expected_current;
  TubeExecutionStateV2 successor;
  phase_offset_core::PortCommand selected_u;
  NormalPreviewProductionPolicy preview_policy;
  TubeExecutionLimitsV2 limits;
  TubeExecutionTrackingEvidenceV2 tracking;
  double base_phase_rate = 0.0;
  double phase_rate_lower = 0.0;
  double phase_rate_upper = 0.0;
  // Authoritative endpoint copied from the P07/P08 viability result.  The
  // caller's nominal endpoint is never retained as a second window authority.
  double horizon_w = 0.0;
  double sample_spacing_w = 0.0;
  double upper_u_delta = 0.0;
  double dt = 0.0;
  double now = 0.0;
  double applicability_deadline = std::numeric_limits<double>::quiet_NaN();
  bool applicability_deadline_valid = false;
  std::size_t max_work = 0U;
  std::string provenance;
  // Allocated while preparation is still fallible.  The no-fail commit seam
  // only assigns this already-materialized owner.
  std::shared_ptr<const TubeFiniteReserveV2> reserve_owner;
  bool valid = false;
  std::string invalid_reason;
};

struct RuntimeV2CommitToken {
  // Public value snapshot is useful to the publication owner for auditing;
  // `sealed_prepared` is the immutable copy used to reject substituted token
  // fields at commit time.
  RuntimeV2PreparedStep prepared;
  std::shared_ptr<const RuntimeV2PreparedStep> sealed_prepared;
  bool valid = false;
};

// Side-effect-free Section handoff input.  The profile/preview are caller
// authorities borrowed for prepare; the publication owner must retain the
// immutable profile and Preview bundle until its finalValidate/publish/commit
// transaction has completed.
class PhaseOffsetRuntime;

struct RuntimeSectionPrepareInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::shared_ptr<const SectionTubeProfile> profile;
  const NormalPreviewResult* preview = nullptr;
  phase_offset_core::MatchedPortInput matched;
  phase_offset_core::PortCommand previous_u;
  PhaseOffsetAllocatorBounds limits;
  double dt = 0.0;
  std::size_t max_work = 0U;
  // Set by the caller when the allocator could not bring the base phase rate
  // back inside [lower_nu, upper_nu] with the available u_w authority and
  // therefore saturated on that authority boundary (its own
  // `phase_window_clipped` evidence).  The Section seam accepts that saturated
  // rate instead of re-applying the same window and turning an approved
  // "take the boundary" tick into a persistent HOLD.  The flag is cross-checked
  // against the base rate and the selected port, so it cannot waive an ordinary
  // out-of-window command.
  bool phase_window_saturated = false;
};

// Immutable value-only result of one Section preparation.  The private fields
// are populated solely by PhaseOffsetRuntime; callers can inspect the exact
// selected port and successor but cannot substitute either value before the
// publish-gated commit seam.
class RuntimeSectionPreparedStep {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RuntimeSectionPreparedStep() = default;

  bool valid() const { return valid_; }
  double nextW() const { return next_w_; }
  double nextDelta() const { return next_delta_; }
  const phase_offset_core::PortCommand& selectedPort() const {
    return selected_u_;
  }
  const phase_offset_core::MatchedPortOutput& matchedOutput() const {
    return matched_output_;
  }
  std::size_t workCount() const { return work_count_; }
  const std::string& reason() const { return reason_; }

 private:
  friend class PhaseOffsetRuntime;

  phase_offset_core::MatchedPortOutput matched_output_;
  phase_offset_core::PortCommand selected_u_;
  double next_w_ = 0.0;
  double next_delta_ = 0.0;
  std::size_t work_count_ = 0U;
  std::string reason_;
  const PhaseOffsetRuntime* producer_ = nullptr;
  std::uint64_t expected_revision_ = 0U;
  double expected_delta_ = 0.0;
  phase_offset_core::PortCommand expected_previous_u;
  bool valid_ = false;
};

using RuntimePrepareInputV2 = RuntimeV2PrepareInput;
using RuntimePrepareResultV2 = RuntimeV2PreparedStep;
using RuntimeCommitTokenV2 = RuntimeV2CommitToken;

class PhaseOffsetRuntime {
 public:
  explicit PhaseOffsetRuntime(const PhaseOffsetRuntimeConfig& config);

  bool configurationValid() const;
  // Refresh is explicit and is intended for an accepted-path revision event.
  // Repeated calls with the same revision do not resample/recompute preflight.
  // These legacy preflight/complete/dry-run entry points are defined only by
  // the explicit regression library.  The production target supplies the V2
  // Runtime implementation below and has no old proof-path definitions.
  bool refreshPreflight(const RuntimePreflightInput& input);
  bool prepare(const RuntimePrepareInput& input, RuntimePreparedStep& prepared);
  bool complete(const RuntimePreparedStep& prepared,
                const Eigen::Vector3d& base_v_cmd,
                double base_w_dot,
                bool base_guidance_valid,
                RuntimeStepOutput& output);
  bool makeCommitToken(const RuntimePreparedStep& prepared,
                       const RuntimeStepOutput& output,
                       RuntimeCommitToken& token) const;
  bool commitToken(const RuntimeCommitToken& token);
  // Called only after authority finalValidate() and successful local
  // PositionCommand publication.  It performs bounded value writes and
  // cannot reject, allocate, or run another proof.
  void commitTokenNoFail(const RuntimeCommitToken& token) noexcept;
  // Runs refresh/prepare/complete on a local copy.  Live preflight, delta,
  // previous port and profile lifecycle remain unchanged on every outcome.
  bool dryRun(const RuntimeDryRunInput& input, RuntimeDryRunResult& result) const;

  // V2 preparation is side-effect free.  It accepts a complete immutable
  // profile/identity/policy snapshot and calls TubeExecutionGuardV2 exactly
  // once.  A successful result contains the exact successor and finite
  // reserve needed by the post-publication no-fail commit.
  bool prepareV2(const RuntimeV2PrepareInput& input,
                RuntimeV2PreparedStep& prepared);
  bool dryRunV2(const RuntimeV2PrepareInput& input,
                RuntimeV2PreparedStep& prepared) const;
  bool makeCommitTokenV2(const RuntimeV2PreparedStep& prepared,
                         RuntimeV2CommitToken& token) const;
  bool commitV2(const RuntimeV2CommitToken& token);
  void commitV2NoFail(const RuntimeV2CommitToken& token) noexcept;

  // Section preparation is a narrow value interface over one immutable
  // Section PWL preview.  It does not project/reselect the caller's final
  // port and does not change Runtime state.
  bool sectionConfigurationValid() const;
  bool prepareSection(const RuntimeSectionPrepareInput& input,
                      RuntimeSectionPreparedStep& prepared) const;
  bool validateSectionBeforePublish(
      const RuntimeSectionPreparedStep& prepared) const;
  // The caller must hold the same serial lock across final validation,
  // successful command publication and this no-fail commit.
  void commitSectionNoFail(
      const RuntimeSectionPreparedStep& prepared) noexcept;
  // Offline convenience seam: this validates and commits values but does not
  // itself publish a ROS command.
  bool commitSection(const RuntimeSectionPreparedStep& prepared);
  // Alias names keep the seam readable to publication owners that use the
  // token terminology directly.
  bool makeV2CommitToken(const RuntimeV2PreparedStep& prepared,
                         RuntimeV2CommitToken& token) const {
    return makeCommitTokenV2(prepared, token);
  }
  bool commitV2Token(const RuntimeV2CommitToken& token) {
    return commitV2(token);
  }
  void commitV2TokenNoFail(const RuntimeV2CommitToken& token) noexcept {
    commitV2NoFail(token);
  }
  const TubeFiniteReserveV2* committedV2Reserve() const {
    return v2_successor_reserve_.get();
  }
  // A user-issued navigation goal starts a new path-coordinate task.  This
  // deliberately differs from an H2 owner retirement: it drops only
  // task-scoped execution history, while retaining validated configuration.
  void resetForNewNavigationTask();
  double retainedDelta() const { return delta_; }
  // Planner ownership requires a matching PathTubePair while a configured
  // manual profile is pending bootstrap or has actually started and remains
  // incomplete, or while a nonzero retained offset still exists.  Once the
  // profile completes and Runtime has returned to delta==0, authority is
  // neutral again.
  bool hasPendingOrActiveOffsetIntent() const;
  // True only after the manual profile has actually started and remains
  // incomplete, or while a nonzero retained offset is live.  This is the
  // predicate for requiring an atomic nonzero path--tube handoff.
  bool hasExecutedOffsetAuthority() const;
  // Recovery/Handoff requests a continuous return through the currently
  // authoritative owner.  It does not reset delta or swap to planner-only;
  // completion is observed and committed only after an accepted exact step
  // reaches neutral.
  void requestRecenter();
  // Deterministic, non-rejecting bounded decay of the retained transverse
  // offset: delta <- delta - sign(delta)*min(|delta|, max_step).  Same class as
  // commitV2NoFail -- it cannot reject, allocate or run a proof.  Used only by
  // the fallback that keeps a nonzero offset alive while no certified Section
  // exists (instead of dropping the offset, which loses lateral containment).
  void applyBoundedDeltaDecayNoFail(double max_step);
  bool recenterRequested() const { return returning_to_center_; }
  phase_offset_core::PortCommand previousFinalPort() const {
    return previous_final_port_;
  }
  const PhaseOffsetRuntimeConfig& config() const { return config_; }

 private:
  bool runPreflightCandidate(const RuntimePathSamples& samples,
                             const Eigen::Vector3d& position,
                             double amplitude,
                             ManualPreflightResult& result) const;
  bool makePrepared(const RuntimePrepareInput& input,
                    RuntimePreparedStep& prepared);
  // Constructs a finite, deterministic continuous port witness over the
  // already scheduled refresh interval.  Every held port is checked against
  // the full exact-PWL segment it traverses (including crossed knots), so the
  // returned root port is the first segment of the same continuous witness.
  // Failure means only that this construction did not supply a certificate;
  // callers retain the existing CERTIFICATE_DENIED fail-closed semantics and
  // must not infer that the mathematical viable set is empty.
  bool buildContinuousExactPwlWitness(
      const RuntimePreparedStep& prepared,
      const phase_offset_core::PortProjectionInput& root_input,
      const Eigen::Vector3d& base_v_cmd,
      double base_w_dot,
      const RuntimeFutureStepContract& contract,
      bool nonnegative_progress,
      phase_offset_core::PortProjectionResult& selected_projection,
      phase_offset_core::MatchedPortOutput& selected_matched,
      TubeBounds& selected_next_bounds,
      std::string& first_failure) const;
  void fillOutput(const RuntimePreparedStep& prepared,
                  RuntimeStepOutput& output) const;
  void makeManualRawPort(RuntimePreparedStep& prepared,
                         bool allow_new_excursion,
                         bool force_recenter) const;
  bool v2CurrentStateMatches(const TubeExecutionStateV2& state) const;
  void invalidateV2State() noexcept;
  void bumpSectionStateRevision() noexcept;

  PhaseOffsetRuntimeConfig config_;
  bool configuration_valid_ = false;
  phase_offset_core::GeometryEvaluator geometry_evaluator_;
  double delta_ = 0.0;
  phase_offset_core::PortCommand previous_final_port_;
  double profile_elapsed_ = 0.0;
  bool profile_started_ = false;
  bool profile_completed_ = false;
  bool returning_to_center_ = false;
  // V2 reserve evidence is immutable and never a second command authority.
  // It is installed only by commitV2NoFail after publication.
  std::shared_ptr<const TubeFiniteReserveV2> v2_successor_reserve_;
  TubeExecutionIdentityV2 v2_identity_;
  bool v2_state_valid_ = false;
  // Local monotonic value version for Section prepare/validate/commit ABA
  // protection.  It is not a map/profile identity and never leaves Runtime.
  std::uint64_t section_state_revision_ = 0U;
  std::uint64_t last_preflight_source_revision_ = 0U;
  bool have_preflight_source_revision_ = false;
  ManualPreflightResult preflight_;
};

}  // namespace phase_offset_navigation
