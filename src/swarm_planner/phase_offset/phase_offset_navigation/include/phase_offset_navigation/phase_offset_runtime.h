#pragma once

#include "phase_offset_navigation/tube_epoch_types.h"
#include "phase_offset_navigation/tube_filter.h"

#include <phase_offset_core/geometry.h>
#include <phase_offset_core/matched_port.h>
#include <phase_offset_core/port_projector.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstdint>
#include <functional>
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
  bool valid = false;
  std::string invalid_reason;
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

class PhaseOffsetRuntime {
 public:
  explicit PhaseOffsetRuntime(const PhaseOffsetRuntimeConfig& config);

  bool configurationValid() const;
  // Refresh is explicit and is intended for an accepted-path revision event.
  // Repeated calls with the same revision do not resample/recompute preflight.
  bool refreshPreflight(const RuntimePreflightInput& input);
  bool prepare(const RuntimePrepareInput& input, RuntimePreparedStep& prepared);
  bool complete(const RuntimePreparedStep& prepared,
                const Eigen::Vector3d& base_v_cmd,
                double base_w_dot,
                bool base_guidance_valid,
                RuntimeStepOutput& output);
  // Runs refresh/prepare/complete on a local copy.  Live preflight, delta,
  // previous port and profile lifecycle remain unchanged on every outcome.
  bool dryRun(const RuntimeDryRunInput& input, RuntimeDryRunResult& result) const;
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

  PhaseOffsetRuntimeConfig config_;
  bool configuration_valid_ = false;
  phase_offset_core::GeometryEvaluator geometry_evaluator_;
  double delta_ = 0.0;
  phase_offset_core::PortCommand previous_final_port_;
  double profile_elapsed_ = 0.0;
  bool profile_started_ = false;
  bool profile_completed_ = false;
  bool returning_to_center_ = false;
  std::uint64_t last_preflight_source_revision_ = 0U;
  bool have_preflight_source_revision_ = false;
  ManualPreflightResult preflight_;
};

}  // namespace phase_offset_navigation
