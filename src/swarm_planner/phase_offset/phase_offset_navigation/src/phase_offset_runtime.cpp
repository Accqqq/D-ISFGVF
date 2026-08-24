#include "phase_offset_navigation/phase_offset_runtime.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace phase_offset_navigation {
namespace {

constexpr double kMinimumAcceptedAmplitude = 0.05;
constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-10;

bool IsFinite(double value) { return std::isfinite(value); }
bool IsFinite(const Eigen::Vector3d& value) { return value.allFinite(); }

bool IsFinite(const phase_offset_core::PortCommand& value) {
  return IsFinite(value.u_w) && IsFinite(value.u_delta);
}

bool HasExecutedOffsetAuthorityState(const double delta,
                                     const bool profile_started,
                                     const bool profile_completed) {
  return delta != 0.0 ||
      (profile_started && !profile_completed);
}

bool ValidManual(const ManualProfileConfig& manual) {
  return IsFinite(manual.amplitude) && manual.amplitude >= kMinimumAcceptedAmplitude &&
      IsFinite(manual.profile_period) && manual.profile_period > 0.0 &&
      IsFinite(manual.delta_tracking_gain) && manual.delta_tracking_gain >= 0.0 &&
      IsFinite(manual.u_w_amplitude) && manual.u_w_amplitude >= 0.0 &&
      IsFinite(manual.u_w_abs_max) && manual.u_w_abs_max >= 0.0 &&
      IsFinite(manual.u_delta_abs_max) && manual.u_delta_abs_max >= 0.0 &&
      IsFinite(manual.u_w_rate_max) && manual.u_w_rate_max >= 0.0 &&
      IsFinite(manual.u_delta_rate_max) && manual.u_delta_rate_max >= 0.0 &&
      IsFinite(manual.phase_dot_min) && manual.phase_dot_min > 0.0 &&
      IsFinite(manual.tangent_speed_min) && manual.tangent_speed_min > 0.0 &&
      IsFinite(manual.preflight_sample_step_w) && manual.preflight_sample_step_w > 0.0;
}

bool ValidTubeExecution(const RuntimeTubeExecutionConfig& tube) {
  return IsFinite(tube.invariant_gain) && tube.invariant_gain >= 0.0 &&
      IsFinite(tube.interior_margin) && tube.interior_margin >= 0.0 &&
      IsFinite(tube.tracking_error_bound) && tube.tracking_error_bound >= 0.0 &&
      IsFinite(tube.regularity_margin) && tube.regularity_margin > 0.0 &&
      tube.regularity_margin < 1.0 &&
      IsFinite(tube.minimum_reference_speed) &&
      tube.minimum_reference_speed > 0.0;
}

phase_offset_core::PortProjectionLimits MakeProjectionLimits(
    const ManualProfileConfig& manual, double regularity_margin,
    const bool nonnegative_progress = false) {
  phase_offset_core::PortProjectionLimits limits;
  limits.u_w_abs_max = manual.u_w_abs_max;
  limits.u_delta_abs_max = manual.u_delta_abs_max;
  limits.u_w_rate_max = manual.u_w_rate_max;
  limits.u_delta_rate_max = manual.u_delta_rate_max;
  limits.phase_dot_min = nonnegative_progress ? 0.0 : manual.phase_dot_min;
  limits.tangent_speed_min = nonnegative_progress ? 0.0 : manual.tangent_speed_min;
  limits.regularity_margin = regularity_margin;
  return limits;
}

bool Inside(const TubeBounds& bounds, double delta, double margin) {
  return bounds.valid && IsFinite(delta) && IsFinite(margin) &&
      delta >= bounds.lower + margin - kTolerance &&
      delta <= bounds.upper - margin + kTolerance;
}

bool BuildExactPwlCells(
    const TubeProfile& profile,
    std::vector<phase_offset_core::PortProjectionPwlCell>& cells) {
  cells.clear();
  if (!profile.complete || profile.samples.size() < 2U) return false;
  cells.reserve(profile.samples.size() - 1U);
  for (std::size_t index = 0U; index + 1U < profile.samples.size(); ++index) {
    const TubeRawSample& first = profile.samples[index];
    const TubeRawSample& second = profile.samples[index + 1U];
    const double dw = second.w - first.w;
    if (!IsFinite(first.w) || !IsFinite(second.w) || dw <= kTolerance ||
        !IsFinite(first.filtered_lower) || !IsFinite(first.filtered_upper) ||
        !IsFinite(second.filtered_lower) || !IsFinite(second.filtered_upper) ||
        first.filtered_lower > first.filtered_upper + kTolerance ||
        second.filtered_lower > second.filtered_upper + kTolerance) {
      cells.clear();
      return false;
    }
    phase_offset_core::PortProjectionPwlCell cell;
    cell.w0 = first.w;
    cell.w1 = second.w;
    cell.lower_at_w0 = first.filtered_lower;
    cell.upper_at_w0 = first.filtered_upper;
    cell.lower_w = (second.filtered_lower - first.filtered_lower) / dw;
    cell.upper_w = (second.filtered_upper - first.filtered_upper) / dw;
    if (!IsFinite(cell.lower_w) || !IsFinite(cell.upper_w)) {
      cells.clear();
      return false;
    }
    cells.push_back(cell);
  }
  return !cells.empty();
}

bool VerifyExactPwlStep(const TubeProfile& profile,
                        const double current_w,
                        const double dt,
                        const double current_delta,
                        const double interior_margin,
                        const phase_offset_core::PortProjectionResult& projection,
                        TubeBounds& next_bounds) {
  next_bounds = TubeBounds();
  if (!IsFinite(current_w) || !IsFinite(dt) || dt <= 0.0 ||
      !IsFinite(current_delta) || !IsFinite(interior_margin) ||
      interior_margin < 0.0 || !projection.valid ||
      projection.final_w_dot < -kTolerance) {
    return false;
  }
  const double next_w = current_w + dt * projection.final_w_dot;
  if (!IsFinite(next_w) || !TubeFilter::query(profile, next_w, next_bounds) ||
      !Inside(next_bounds, projection.next_delta, interior_margin)) {
    return false;
  }
  if (projection.final_w_dot <= kTolerance) return true;
  for (const TubeRawSample& sample : profile.samples) {
    if (sample.w <= current_w + kTolerance || sample.w >= next_w - kTolerance) {
      continue;
    }
    const double elapsed = (sample.w - current_w) / projection.final_w_dot;
    const double delta = current_delta + elapsed * projection.final_port.u_delta;
    TubeBounds knot_bounds;
    if (!IsFinite(elapsed) || elapsed < -kTolerance || elapsed > dt + kTolerance ||
        !TubeFilter::query(profile, sample.w, knot_bounds) ||
        !Inside(knot_bounds, delta, interior_margin)) {
      return false;
    }
  }
  return true;
}

bool FutureContractValid(const RuntimeFutureStepContract& contract) {
  return static_cast<bool>(contract.evaluate) &&
      IsFinite(contract.tube_update_period) &&
      contract.tube_update_period > 0.0 &&
      IsFinite(contract.min_certified_forward_w) &&
      contract.min_certified_forward_w >= 0.0 &&
      IsFinite(contract.profile_domain_end_w);
}

bool MatchesAdapterReference(
    const phase_offset_core::PhaseOffsetGeometryState& geometry,
    const RuntimeFutureStepResult& future) {
  return future.valid && future.path.valid &&
      IsFinite(future.matched_reference) && IsFinite(future.matched_tangent) &&
      IsFinite(future.matched_derivative_norm) &&
      future.matched_derivative_norm > 0.0 && IsFinite(future.base_v_cmd) &&
      IsFinite(future.base_w_dot) && future.base_guidance_valid &&
      (future.matched_reference - geometry.r).norm() <= kTolerance &&
      (future.matched_tangent - geometry.T).norm() <= kTolerance &&
      std::abs(future.matched_derivative_norm - geometry.r_w.norm()) <=
          kTolerance;
}

bool SameProfileDomain(const TubeProfile& profile, const double phase) {
  if (!profile.complete || profile.samples.size() < 2U || !IsFinite(phase)) {
    return false;
  }
  return phase >= profile.samples.front().w - kTolerance &&
      phase <= profile.samples.back().w + kTolerance;
}

bool AddProjectionCandidate(
    const phase_offset_core::PortProjectionInput& input,
    const phase_offset_core::PortProjectionLimits& limits,
    const phase_offset_core::PortCommand& raw,
    std::vector<phase_offset_core::PortProjectionResult>& candidates) {
  phase_offset_core::PortProjectionInput candidate_input = input;
  candidate_input.raw = raw;
  phase_offset_core::PortProjectionResult candidate;
  if (!phase_offset_core::PortProjector::project(candidate_input, limits,
                                                  candidate) ||
      !candidate.valid) {
    return false;
  }
  for (const auto& existing : candidates) {
    if (std::abs(existing.final_port.u_w - candidate.final_port.u_w) <=
            kTolerance &&
        std::abs(existing.final_port.u_delta -
                 candidate.final_port.u_delta) <= kTolerance) {
      return true;
    }
  }
  candidates.push_back(candidate);
  return true;
}

bool BuildDeterministicProjectionCandidates(
    const phase_offset_core::PortProjectionInput& input,
    const phase_offset_core::PortProjectionLimits& limits,
    std::vector<phase_offset_core::PortProjectionResult>& candidates,
    phase_offset_core::PortProjectionResult* failure_projection = nullptr) {
  candidates.clear();
  if (failure_projection != nullptr) {
    *failure_projection = phase_offset_core::PortProjectionResult();
  }
  phase_offset_core::PortProjectionResult nearest;
  if (!phase_offset_core::PortProjector::project(input, limits, nearest) ||
      !nearest.valid) {
    if (failure_projection != nullptr) *failure_projection = nearest;
    return false;
  }
  candidates.push_back(nearest);
  for (const auto& vertex : nearest.diagnostics.feasible_polygon_vertices) {
    AddProjectionCandidate(input, limits, vertex, candidates);
  }
  for (const auto& edge : nearest.diagnostics.feasible_polygon_edges) {
    phase_offset_core::PortCommand midpoint;
    midpoint.u_w = 0.5 * (edge.first.u_w + edge.second.u_w);
    midpoint.u_delta = 0.5 * (edge.first.u_delta + edge.second.u_delta);
    AddProjectionCandidate(input, limits, midpoint, candidates);
  }
  return !candidates.empty();
}

// This remains deliberately local to Runtime: it is rendered through the
// existing RuntimeStepOutput::invalid_reason and creates no product status,
// enum, or telemetry field.  A finite DFS rejects many shallow siblings on
// the way to a deeper frontier.  Reporting the literal minimum rejected step
// would therefore always hide the useful boundary reached by another valid
// prefix.  Keep the first failure at the greatest reached step; equal-depth
// failures retain the existing deterministic candidate/DFS order.
class BoundedViabilityFailure {
 public:
  void record(const std::size_t step, const char* category,
              const std::string& detail = std::string()) {
    if (recorded_ && step <= step_) return;
    recorded_ = true;
    step_ = step;
    category_ = category;
    detail_ = detail;
  }

  std::string describe() const {
    if (!recorded_) {
      return "first_failure_step=0 category=continuation";
    }
    std::string description = "first_failure_step=" + std::to_string(step_) +
        " category=" + category_;
    if (!detail_.empty()) description += " detail=" + detail_;
    return description;
  }

 private:
  bool recorded_ = false;
  std::size_t step_ = 0U;
  std::string category_;
  std::string detail_;
};

bool HasProjectorConstraintPrefix(
    const phase_offset_core::PortProjectionResult& projection,
    const char* prefix) {
  const std::size_t prefix_size = std::strlen(prefix);
  for (const std::string& label : projection.diagnostics.conflicting_constraints) {
    if (label.compare(0U, prefix_size, prefix) == 0) return true;
  }
  return false;
}

struct ProjectorFailureDescription {
  const char* category = "projector";
  std::string detail;
};

ProjectorFailureDescription DescribeProjectorFailure(
    const phase_offset_core::PortProjectionResult& projection) {
  ProjectorFailureDescription description;
  const std::string& reason = projection.invalid_reason;
  const phase_offset_core::PortProjectionFailureStage stage =
      projection.diagnostics.failure_stage;
  const char* constraint = "unspecified";

  // Prefer the existing diagnostic facts when they are present.  The fallback
  // strings are the existing PortProjector failure reasons, not a new Runtime
  // classification scheme.  A core-polygon empty result is intentionally not
  // called exact-PWL unless the existing conflicting-constraint diagnostics
  // actually identify a PWL plane: local slew/amplitude/non-reverse/tangent
  // constraints can produce the same empty polygon.
  if (HasProjectorConstraintPrefix(projection, "pwl_terminal_") ||
      HasProjectorConstraintPrefix(projection, "pwl_crossed_")) {
    description.category = "exact terminal-or-crossed-knot";
    constraint = "exact PWL";
  } else if (stage == phase_offset_core::PortProjectionFailureStage::
                          CURRENT_REGULARITY ||
             reason.find("regularity") != std::string::npos) {
    constraint = "regularity";
  } else if (stage == phase_offset_core::PortProjectionFailureStage::
                          CURRENT_OFFSET_MARGIN ||
             reason.find("current offset constraint") != std::string::npos) {
    description.category = "current containment";
    constraint = "current offset margin";
  } else if (stage == phase_offset_core::PortProjectionFailureStage::
                          INVALID_OFFSET_CONSTRAINT ||
             reason.find("piecewise-linear offset cells") != std::string::npos ||
             reason.find("current phase is outside piecewise-linear") !=
                 std::string::npos) {
    description.category = "profile domain";
    constraint = "PWL profile domain";
  } else if (stage == phase_offset_core::PortProjectionFailureStage::
                          LOCAL_INTERVAL_EMPTY ||
             reason.find("local port feasible interval") != std::string::npos ||
             reason.find("u_w feasible interval") != std::string::npos ||
             reason.find("u_delta feasible interval") != std::string::npos) {
    constraint = "local bounds";
  } else if (stage == phase_offset_core::PortProjectionFailureStage::
                          CORE_POLYGON_EMPTY ||
             reason.find("joint port feasible polygon") != std::string::npos) {
    constraint = "joint port polygon";
  } else if (stage == phase_offset_core::PortProjectionFailureStage::
                          FINAL_CONSTRAINT_CHECK ||
             reason.find("joint projected port") != std::string::npos) {
    constraint = "final constraint check";
  } else if (stage == phase_offset_core::PortProjectionFailureStage::
                          INVALID_LIMITS ||
             reason.find("projection limits") != std::string::npos) {
    constraint = "projection limits";
  } else if (stage == phase_offset_core::PortProjectionFailureStage::
                          INVALID_INPUT ||
             reason.find("projection input") != std::string::npos) {
    constraint = "projection input";
  }

  description.detail = "constraint=" + std::string(constraint);
  if (!reason.empty()) description.detail += "; " + reason;
  return description;
}

}  // namespace

PhaseOffsetRuntime::PhaseOffsetRuntime(const PhaseOffsetRuntimeConfig& config)
    : config_(config) {
  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = config_.tube.regularity_margin;
  geometry_params.minimum_reference_speed =
      config_.tube.minimum_reference_speed;
  geometry_evaluator_ = phase_offset_core::GeometryEvaluator(geometry_params);
  configuration_valid_ = ValidManual(config_.manual) && ValidTubeExecution(config_.tube);
}

bool PhaseOffsetRuntime::hasPendingOrActiveOffsetIntent() const {
  if (delta_ != 0.0) return true;
  // A not-yet-started profile still needs a matching PathTubePair bootstrap;
  // otherwise the first selected nonzero port could execute through the
  // sidecar epoch without an authoritative owner.  Once the profile has
  // completed and returned to delta==0, however, the configured amplitude is
  // no longer an authority requirement.  A started-but-incomplete profile
  // retains authority through instantaneous zero crossings.
  if (profile_started_) return !profile_completed_;
  return config_.manual.amplitude != 0.0;
}

bool PhaseOffsetRuntime::hasExecutedOffsetAuthority() const {
  return HasExecutedOffsetAuthorityState(
      delta_, profile_started_, profile_completed_);
}

void PhaseOffsetRuntime::requestRecenter() {
  // This is a lifecycle intent, not a state reset.  The next prepare emits a
  // bounded inward port through the existing active owner; complete() retires
  // the profile only after the exact command reaches neutral.
  returning_to_center_ = true;
}

bool PhaseOffsetRuntime::configurationValid() const { return configuration_valid_; }

void PhaseOffsetRuntime::resetForNewNavigationTask() {
  // A new planner goal has a new path coordinate.  Do not retain any value
  // whose meaning is tied to the retired path or to the preceding exact-port
  // selection.  `configuration_valid_`, `config_`, and GeometryEvaluator stay
  // intact: this is a task rebase, not configuration recovery.
  delta_ = 0.0;
  previous_final_port_ = phase_offset_core::PortCommand();
  profile_elapsed_ = 0.0;
  profile_started_ = false;
  profile_completed_ = false;
  returning_to_center_ = false;
  last_preflight_source_revision_ = 0U;
  have_preflight_source_revision_ = false;
  preflight_ = ManualPreflightResult();
}

bool PhaseOffsetRuntime::runPreflightCandidate(const RuntimePathSamples& samples,
                                                const Eigen::Vector3d& position,
                                                double amplitude,
                                                ManualPreflightResult& result) const {
  result = ManualPreflightResult();
  result.configured_amplitude = std::abs(config_.manual.amplitude);
  if (samples.empty() || !IsFinite(position) || !IsFinite(amplitude) || amplitude < 0.0) {
    result.invalid_reason = "preflight input is invalid";
    return false;
  }
  result.min_regularity = std::numeric_limits<double>::infinity();
  std::string first_reason;
  auto record_invalid = [&](double w, double sign, const std::string& reason) {
    if (result.invalid_sample_count++ == 0U) {
      result.first_invalid_w = w;
      result.first_invalid_side = sign < 0.0 ? -1 : 1;
    }
    if (first_reason.empty()) first_reason = reason;
  };
  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = config_.tube.regularity_margin;
  geometry_params.minimum_reference_speed =
      config_.tube.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator evaluator(geometry_params);
  for (double sign : {-1.0, 1.0}) {
    for (const auto& path : samples) {
      ++result.sample_count;
      phase_offset_core::PhaseOffsetGeometryState geometry;
      if (!evaluator.evaluate(path, position, sign * amplitude, geometry)) {
        record_invalid(path.w, sign, geometry.invalid_reason);
        continue;
      }
      const double z_residual = std::abs(geometry.r.z() - geometry.p.z());
      if (!IsFinite(z_residual) || z_residual > 1e-12) {
        record_invalid(path.w, sign, "active reference height is inconsistent");
        continue;
      }
      result.max_abs_r_z_minus_p_z = std::max(result.max_abs_r_z_minus_p_z, z_residual);
      result.min_regularity = std::min(result.min_regularity, geometry.regularity);
    }
  }
  if (result.invalid_sample_count != 0U || !IsFinite(result.min_regularity)) {
    result.invalid_reason = first_reason.empty() ? "preflight sample set is incomplete" : first_reason;
    return false;
  }
  result.complete = true;
  result.accepted_amplitude = amplitude;
  return true;
}

bool PhaseOffsetRuntime::refreshPreflight(const RuntimePreflightInput& input) {
  if (have_preflight_source_revision_ &&
      input.path_source_revision == last_preflight_source_revision_) {
    return preflight_.complete;
  }
  have_preflight_source_revision_ = true;
  last_preflight_source_revision_ = input.path_source_revision;
  preflight_ = ManualPreflightResult();
  preflight_.configured_amplitude = std::abs(config_.manual.amplitude);
  ManualPreflightResult candidate;
  const double configured = preflight_.configured_amplitude;
  if (runPreflightCandidate(input.path, input.position, configured, candidate)) {
    preflight_ = candidate;
    return true;
  }
  const std::string original_reason = candidate.invalid_reason;
  for (int index = 1; index <= 70; ++index) {
    const double ratio = static_cast<double>(index) / 70.0;
    const double amplitude = configured + (kMinimumAcceptedAmplitude - configured) * ratio;
    if (runPreflightCandidate(input.path, input.position, amplitude, candidate)) {
      preflight_ = candidate;
      preflight_.amplitude_reduced = true;
      preflight_.reduction_reason = original_reason;
      return true;
    }
  }
  preflight_ = candidate;
  preflight_.configured_amplitude = configured;
  if (preflight_.invalid_reason.empty()) {
    preflight_.invalid_reason = "no bilateral amplitude is feasible";
  }
  if (profile_started_ || std::abs(delta_) > kTolerance) {
    returning_to_center_ = true;
    profile_completed_ = true;
  }
  return false;
}

void PhaseOffsetRuntime::makeManualRawPort(RuntimePreparedStep& prepared,
                                           bool allow_new_excursion,
                                           bool force_recenter) const {
  const bool can_start = allow_new_excursion && !profile_started_ &&
      !profile_completed_ && !returning_to_center_ && preflight_.complete;
  prepared.should_start_profile = can_start;
  const bool use_profile = !force_recenter && (profile_started_ || can_start) &&
      !profile_completed_ && !returning_to_center_ &&
      profile_elapsed_ <= config_.manual.profile_period;
  prepared.profile_active = use_profile;
  if (!use_profile) {
    prepared.delta_ref = 0.0;
    prepared.raw_port.u_w = 0.0;
    prepared.raw_port.u_delta = -config_.manual.delta_tracking_gain * delta_;
    return;
  }
  const double s = std::max(0.0, std::min(1.0, profile_elapsed_ / config_.manual.profile_period));
  const double smooth_s = s * s * (3.0 - 2.0 * s);
  const double smooth_s_dot = 6.0 * s * (1.0 - s) / config_.manual.profile_period;
  const double phase = 2.0 * kPi * smooth_s;
  prepared.delta_ref = -preflight_.accepted_amplitude * std::sin(phase);
  const double delta_ref_dot = -preflight_.accepted_amplitude * 2.0 * kPi *
      std::cos(phase) * smooth_s_dot;
  prepared.raw_port.u_w = config_.manual.u_w_amplitude * std::sin(phase);
  prepared.raw_port.u_delta = delta_ref_dot + config_.manual.delta_tracking_gain *
      (prepared.delta_ref - delta_);
}

bool PhaseOffsetRuntime::makePrepared(const RuntimePrepareInput& input,
                                      RuntimePreparedStep& prepared) {
  prepared = RuntimePreparedStep();
  prepared.active_profile = input.tube_view.active_profile;
  prepared.epoch_status = input.tube_view.epoch_status;
  prepared.future_step = input.future_step;
  prepared.frame_bound = input.current_path.frame_valid;
  prepared.preflight = preflight_;
  prepared.delta = delta_;
  prepared.dt = input.dt;
  prepared.current_position = input.position;
  prepared.zero_gate_open = input.zero_gate_open;
  prepared.execution.active_profile_available = static_cast<bool>(prepared.active_profile);
  prepared.execution.tracking_error_bound = config_.tube.tracking_error_bound;
  if (!configuration_valid_ || !IsFinite(input.dt) || input.dt <= 0.0) {
    prepared.invalid_reason = "runtime configuration or step time is invalid";
    return false;
  }
  if (input.fatal_adapter_failure_latched) {
    prepared.execution.mode = RuntimeExecutionMode::FATAL_CONTROL_FAILURE;
    prepared.execution.fatal_control_failure = true;
    prepared.invalid_reason = "adapter control failure is latched";
    return false;
  }
  if (!geometry_evaluator_.evaluate(input.current_path, input.position, delta_, prepared.geometry)) {
    prepared.execution.mode = RuntimeExecutionMode::FATAL_CONTROL_FAILURE;
    prepared.execution.fatal_control_failure = true;
    prepared.execution.genuine_fatal_invariant = true;
    prepared.execution.failure_reason = ControlFailureReason::GEOMETRY_INVARIANT;
    prepared.invalid_reason = prepared.geometry.invalid_reason.empty()
        ? "current active geometry is invalid" : prepared.geometry.invalid_reason;
    return false;
  }
  prepared.execution.current_geometry_valid = true;
  prepared.execution.tracking_error_norm = (input.position - prepared.geometry.r).norm();
  prepared.execution.tracking_within_bound =
      IsFinite(prepared.execution.tracking_error_norm) &&
      prepared.execution.tracking_error_norm <= config_.tube.tracking_error_bound + kTolerance;
  if (prepared.active_profile && prepared.active_profile->complete) {
    prepared.execution.current_bounds_valid = TubeFilter::query(
        *prepared.active_profile, prepared.geometry.w, prepared.current_bounds);
    prepared.execution.retained_delta_current_inside = Inside(
        prepared.current_bounds, delta_, config_.tube.interior_margin);
  }
  // Completion is a state transition owned by `complete`, not by prepare.
  // Observe it locally so repeated prepares are side-effect free.
  const bool completes_on_this_step = profile_started_ && !profile_completed_ &&
      profile_elapsed_ >= config_.manual.profile_period &&
      delta_ == 0.0;
  if (config_.tube.source == TubeSource::NONE) {
    if (!preflight_.complete && !returning_to_center_) {
      prepared.execution.mode = RuntimeExecutionMode::BLOCKED;
      prepared.invalid_reason = preflight_.invalid_reason.empty()
          ? "manual preflight is incomplete" : preflight_.invalid_reason;
      return false;
    }
    prepared.execution.mode = RuntimeExecutionMode::NO_TUBE_REQUIRED;
    prepared.execution.executable = true;
    makeManualRawPort(prepared, true, completes_on_this_step);
  } else {
    const bool active_current_ok = prepared.active_profile &&
        prepared.active_profile->complete &&
        prepared.epoch_status.active_current_validation_valid &&
        prepared.execution.current_bounds_valid &&
        prepared.execution.retained_delta_current_inside;
    const bool latest_explicit_unsafe =
        prepared.epoch_status.certificate_denied ||
        prepared.epoch_status.current_safety_status == CurrentSafetyStatus::UNSAFE;
    if (!active_current_ok || latest_explicit_unsafe) {
      if (!latest_explicit_unsafe) {
        prepared.execution.mode = RuntimeExecutionMode::WAITING_FOR_CANDIDATE;
        prepared.execution.transient_blocked = true;
        prepared.invalid_reason = "no executable active tube is installed";
        return false;
      }
      prepared.execution.mode = RuntimeExecutionMode::CERTIFICATE_DENIED;
      prepared.execution.certificate_denied = true;
      prepared.invalid_reason = "current tube state denies the offset certificate";
      return false;
    } else {
      prepared.execution.mode = RuntimeExecutionMode::NORMAL;
      prepared.execution.executable = true;
      makeManualRawPort(prepared, true, completes_on_this_step);
    }
  }
  // Near neutral, choose the bounded inward command that reaches the neutral
  // handoff at the end of this ZOH interval.  This is only enabled after an
  // explicit recenter request and remains subject to PortProjector limits;
  // it avoids an asymptotic residual keeping nonzero authority alive forever.
  if (returning_to_center_ && delta_ != 0.0 && input.dt > 0.0) {
    prepared.raw_port.u_delta = -delta_ / input.dt;
    prepared.delta_ref = 0.0;
  }
  prepared.requires_base_guidance = prepared.execution.executable;
  prepared.valid = prepared.execution.executable;
  return prepared.valid;
}

bool PhaseOffsetRuntime::prepare(const RuntimePrepareInput& input,
                                 RuntimePreparedStep& prepared) {
  return makePrepared(input, prepared);
}

void PhaseOffsetRuntime::fillOutput(const RuntimePreparedStep& prepared,
                                    RuntimeStepOutput& output) const {
  output = RuntimeStepOutput();
  output.geometry = prepared.geometry;
  output.raw_port = prepared.raw_port;
  output.active_profile = prepared.active_profile;
  output.current_bounds = prepared.current_bounds;
  output.preflight = prepared.preflight;
  output.epoch_status = prepared.epoch_status;
  output.execution = prepared.execution;
  output.exact_terminal_predicate = false;
  output.delta = delta_;
  output.delta_ref = prepared.delta_ref;
  output.profile_active = prepared.profile_active;
  output.invalid_reason = prepared.invalid_reason;
}

bool PhaseOffsetRuntime::buildContinuousExactPwlWitness(
    const RuntimePreparedStep& prepared,
    const phase_offset_core::PortProjectionInput& root_input,
    const Eigen::Vector3d& base_v_cmd,
    const double base_w_dot,
    const RuntimeFutureStepContract& contract,
    const bool nonnegative_progress,
    phase_offset_core::PortProjectionResult& selected_projection,
    phase_offset_core::MatchedPortOutput& selected_matched,
    TubeBounds& selected_next_bounds,
    std::string& first_failure) const {
  selected_projection = phase_offset_core::PortProjectionResult();
  selected_matched = phase_offset_core::MatchedPortOutput();
  selected_next_bounds = TubeBounds();
  first_failure.clear();
  BoundedViabilityFailure failure;
  const auto fail = [&failure, &first_failure](const std::size_t step,
                                               const char* category,
                                               const std::string& detail =
                                                   std::string()) {
    failure.record(step, category, detail);
    first_failure = failure.describe();
    return false;
  };
  if (!prepared.active_profile) {
    return fail(0U, "profile domain", "active profile is unavailable");
  }
  if (!FutureContractValid(contract)) {
    return fail(0U, "contract", "future-step contract is invalid");
  }
  if (!IsFinite(base_v_cmd) || !IsFinite(base_w_dot) ||
      !IsFinite(root_input.previous_final) ||
      !IsFinite(prepared.geometry.r) || !IsFinite(prepared.geometry.w) ||
      !IsFinite(prepared.delta) || !IsFinite(prepared.dt) ||
      prepared.dt <= 0.0) {
    return fail(0U, "contract", "root rollout inputs are invalid");
  }

  const TubeProfile& profile = *prepared.active_profile;
  if (!profile.complete || profile.samples.size() < 2U) {
    return fail(0U, "profile domain", "active profile is incomplete");
  }
  const double sample_start = profile.samples.front().w;
  const double sample_end = profile.samples.back().w;
  const double certified_start = profile.certified_segment_start_w;
  const double certified_end = profile.certified_segment_end_w;
  const double current_phase = prepared.geometry.w;
  if (!SameProfileDomain(profile, current_phase) ||
      !IsFinite(certified_start) || !IsFinite(certified_end) ||
      certified_start < sample_start - kTolerance ||
      certified_end > sample_end + kTolerance ||
      current_phase < certified_start - kTolerance ||
      current_phase > certified_end + kTolerance) {
    return fail(0U, "profile domain", "current phase is outside certified profile");
  }
  if (std::abs(contract.profile_domain_end_w - certified_end) > kTolerance) {
    return fail(0U, "contract", "future-step profile domain does not match active profile");
  }

  // `min_certified_forward_w` is the TubeEpochManager installation-domain
  // contract (and remains the source of reason 11).  Runtime must instead
  // prove the finite, time-based rollout that it will actually execute.  In
  // particular, U_safe may legitimately remain in the current PWL cell while
  // waiting for the next tube refresh, so it cannot be required to advance by
  // the manager's installation length here.
  const double time_horizon = contract.tube_update_period;
  const std::size_t refresh_steps = static_cast<std::size_t>(
      std::ceil(time_horizon / prepared.dt - kTolerance));
  if (!IsFinite(time_horizon) || refresh_steps == 0U) {
    return fail(0U, "contract", "future-step rollout horizon is invalid");
  }
  // A witness need only cover the already-scheduled next tube refresh.  The
  // manager has separately checked its installation-domain coverage; Runtime
  // must not create a longer recovery requirement for a valid U_safe hold.
  const std::size_t bounded_horizon_steps =
      std::max<std::size_t>(1U, refresh_steps);
  if (bounded_horizon_steps == 0U || bounded_horizon_steps >
      static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return fail(0U, "contract", "future-step rollout horizon is out of range");
  }

  std::vector<phase_offset_core::PortProjectionPwlCell> cells;
  if (!BuildExactPwlCells(profile, cells)) {
    return fail(0U, "profile domain", "active profile PWL cells are invalid");
  }

  const double regularity_threshold = prepared.frame_bound
      ? config_.tube.minimum_reference_speed
      : config_.tube.regularity_margin;
  const phase_offset_core::PortProjectionLimits limits = MakeProjectionLimits(
      config_.manual, regularity_threshold, nonnegative_progress);
  std::function<bool(std::size_t, double, double, const Eigen::Vector3d&,
                     const phase_offset_core::PortCommand&,
                     const phase_offset_core::PhaseOffsetGeometryState&,
                     const Eigen::Vector3d&, double,
                     phase_offset_core::PortProjectionResult*,
                     phase_offset_core::MatchedPortOutput*, TubeBounds*)>
      search;
  search = [&](const std::size_t step, const double phase, const double delta,
               const Eigen::Vector3d& position,
               const phase_offset_core::PortCommand& previous_port,
               const phase_offset_core::PhaseOffsetGeometryState& geometry,
               const Eigen::Vector3d& step_base_v_cmd,
               const double step_base_w_dot,
               phase_offset_core::PortProjectionResult* root_projection,
               phase_offset_core::MatchedPortOutput* root_matched,
               TubeBounds* root_next_bounds) -> bool {
    if (step >= bounded_horizon_steps) return true;
    if (!SameProfileDomain(profile, phase) ||
        phase < certified_start - kTolerance ||
        phase > certified_end + kTolerance) {
      failure.record(step, "profile domain", "rollout phase is outside certified profile");
      return false;
    }
    TubeBounds bounds;
    if (!TubeFilter::query(profile, phase, bounds) ||
        !Inside(bounds, delta, config_.tube.interior_margin)) {
      failure.record(step, "current containment", "rollout offset is outside active bounds");
      return false;
    }
    phase_offset_core::PortProjectionInput input = root_input;
    input.raw = step == 0U ? root_input.raw
                           : phase_offset_core::PortCommand();
    input.previous_final = previous_port;
    input.delta = delta;
    input.curvature = geometry.curvature;
    input.r_w_norm = geometry.r_w.norm();
    input.phase = phase;
    input.base_w_dot = step_base_w_dot;
    input.base_tangent_speed = geometry.T.dot(step_base_v_cmd);
    input.offset_constraint.lower = bounds.lower;
    input.offset_constraint.upper = bounds.upper;
    input.offset_constraint.lower_w = bounds.lower_w;
    input.offset_constraint.upper_w = bounds.upper_w;
    input.offset_constraint.valid = bounds.valid;
    input.offset_pwl_cells = cells;
    std::vector<phase_offset_core::PortProjectionResult> candidates;
    phase_offset_core::PortProjectionResult projector_failure;
    if (!BuildDeterministicProjectionCandidates(input, limits, candidates,
                                                &projector_failure)) {
      const ProjectorFailureDescription projector_description =
          DescribeProjectorFailure(projector_failure);
      failure.record(step, projector_description.category,
                     projector_description.detail);
      return false;
    }
    for (const auto& projection : candidates) {
      TubeBounds next_bounds;
      if (!VerifyExactPwlStep(profile, phase, prepared.dt, delta,
                              config_.tube.interior_margin, projection,
                              next_bounds)) {
        failure.record(step, "exact terminal-or-crossed-knot",
                       "projected port does not preserve exact PWL containment");
        continue;
      }
      phase_offset_core::MatchedPortInput matched_input;
      matched_input.geometry = geometry;
      matched_input.base_v_cmd = step_base_v_cmd;
      matched_input.base_w_dot = step_base_w_dot;
      matched_input.final_port = projection.final_port;
      phase_offset_core::MatchedPortOutput matched;
      if (!phase_offset_core::MatchedPort::evaluate(matched_input, matched) ||
          !matched.valid) {
        failure.record(step, "matched port", matched.invalid_reason);
        continue;
      }
      const double next_phase = phase + prepared.dt * projection.final_w_dot;
      if (!IsFinite(next_phase) || next_phase < phase - kTolerance ||
          next_phase > certified_end + kTolerance ||
          !SameProfileDomain(profile, next_phase)) {
        failure.record(step, "next domain", "projected phase leaves active profile");
        continue;
      }
      bool continuation = step + 1U >= bounded_horizon_steps;
      if (!continuation) {
        const Eigen::Vector3d next_position = position +
            prepared.dt * matched.v_cmd;
        RuntimeFutureStepInput future_input;
        future_input.step_index = step + 1U;
        future_input.phase = next_phase;
        future_input.delta = projection.next_delta;
        future_input.previous_final_port = projection.final_port;
        future_input.matched_position = next_position;
        future_input.previous_matched_reference = geometry.r;
        future_input.dt = prepared.dt;
        RuntimeFutureStepResult future;
        if (!contract.evaluate(future_input, future) || !future.valid) {
          failure.record(step + 1U, "future evaluator", future.invalid_reason);
          continue;
        }
        if (!future.path.valid || !IsFinite(future.path.w) ||
            std::abs(future.path.w - next_phase) > kTolerance) {
          failure.record(step + 1U, "future geometry-reference",
                         future.invalid_reason.empty()
                             ? "future path phase is invalid or mismatched"
                             : future.invalid_reason);
          continue;
        }
        phase_offset_core::PhaseOffsetGeometryState future_geometry;
        if (!geometry_evaluator_.evaluate(future.path, next_position,
                                         projection.next_delta,
                                         future_geometry)) {
          failure.record(step + 1U, "future geometry-reference",
                         future_geometry.invalid_reason);
          continue;
        }
        if (!MatchesAdapterReference(future_geometry, future)) {
          failure.record(step + 1U, "future geometry-reference",
                         future.invalid_reason.empty()
                             ? "future adapter reference does not match geometry"
                             : future.invalid_reason);
          continue;
        }
        continuation = search(step + 1U, next_phase,
                              projection.next_delta, next_position,
                              projection.final_port,
                              future_geometry, future.base_v_cmd,
                              future.base_w_dot, root_projection,
                              root_matched, root_next_bounds);
        if (!continuation) {
          failure.record(step + 1U, "continuation");
        }
      }
      if (!continuation) continue;
      if (step == 0U) {
        *root_projection = projection;
        *root_matched = matched;
        *root_next_bounds = next_bounds;
      }
      return true;
    }
    failure.record(step, "continuation", "no deterministic candidate continues");
    return false;
  };

  if (search(0U, current_phase, prepared.delta, prepared.current_position,
             root_input.previous_final,
             prepared.geometry, base_v_cmd, base_w_dot,
             &selected_projection, &selected_matched,
             &selected_next_bounds)) {
    return true;
  }
  // The loop includes the current held command as its first elapsed period.
  // Both U+ and U_safe therefore prove time-continuous exact containment at
  // least through the next existing tube refresh.  U_safe intentionally may
  // have zero phase progress while remaining safe inside its current PWL cell.
  first_failure = failure.describe();
  return false;
}

bool PhaseOffsetRuntime::complete(const RuntimePreparedStep& prepared,
                                  const Eigen::Vector3d& base_v_cmd,
                                  double base_w_dot,
                                  bool base_guidance_valid,
                                  RuntimeStepOutput& output) {
  fillOutput(prepared, output);
  if (!prepared.valid || !prepared.requires_base_guidance) return false;
  if (!base_guidance_valid || !IsFinite(base_v_cmd) || !IsFinite(base_w_dot)) {
    output.execution.mode = RuntimeExecutionMode::FATAL_CONTROL_FAILURE;
    output.execution.fatal_control_failure = true;
    output.execution.genuine_fatal_invariant = true;
    output.execution.failure_reason =
        ControlFailureReason::BASE_GUIDANCE_INVARIANT;
    output.invalid_reason = "base guidance is invalid";
    return false;
  }
  phase_offset_core::PortProjectionInput projection_input;
  projection_input.raw = prepared.raw_port;
  projection_input.previous_final = previous_final_port_;
  projection_input.dt = prepared.dt;
  projection_input.delta = delta_;
  projection_input.curvature = prepared.geometry.curvature;
  projection_input.r_w_norm = prepared.geometry.r_w.norm();
  projection_input.phase = prepared.geometry.w;
  projection_input.base_w_dot = base_w_dot;
  projection_input.base_tangent_speed = prepared.geometry.T.dot(base_v_cmd);
  if (config_.tube.source != TubeSource::NONE) {
    projection_input.offset_constraint.lower = prepared.current_bounds.lower;
    projection_input.offset_constraint.upper = prepared.current_bounds.upper;
    projection_input.offset_constraint.lower_w = prepared.current_bounds.lower_w;
    projection_input.offset_constraint.upper_w = prepared.current_bounds.upper_w;
    projection_input.offset_constraint.invariant_gain = config_.tube.invariant_gain;
    projection_input.offset_constraint.interior_margin = config_.tube.interior_margin;
    projection_input.offset_constraint.enabled = true;
    projection_input.offset_constraint.valid = prepared.current_bounds.valid;
    if (!prepared.active_profile || !BuildExactPwlCells(
            *prepared.active_profile, projection_input.offset_pwl_cells)) {
      projection_input.offset_constraint.valid = false;
    }
  }
  const bool tube_enabled = config_.tube.source != TubeSource::NONE;
  const auto project_port = [&](const bool nonnegative_progress,
                                const phase_offset_core::PortCommand& raw,
                                phase_offset_core::PortProjectionResult& projection) {
    projection_input.raw = raw;
    return phase_offset_core::PortProjector::project(
        projection_input, MakeProjectionLimits(
            config_.manual,
            prepared.frame_bound
                ? config_.tube.minimum_reference_speed
                : config_.tube.regularity_margin,
            nonnegative_progress), projection);
  };

  if (tube_enabled) {
    bool selected_nonnegative = false;
    std::string positive_failure;
    const bool have_witness = buildContinuousExactPwlWitness(
        prepared, projection_input, base_v_cmd, base_w_dot,
        prepared.future_step, false, output.projection, output.matched,
        output.next_bounds, positive_failure);
    std::string safe_failure;
    bool witness_available = have_witness;
    if (!witness_available) {
      selected_nonnegative = true;
      witness_available = buildContinuousExactPwlWitness(
          prepared, projection_input, base_v_cmd, base_w_dot,
          prepared.future_step, true, output.projection, output.matched,
          output.next_bounds, safe_failure);
    }
    if (!witness_available) {
      output.execution.mode = RuntimeExecutionMode::CERTIFICATE_DENIED;
      output.execution.certificate_denied = true;
      output.execution.transient_blocked = false;
      output.execution.genuine_fatal_invariant = false;
      output.execution.failure_reason = ControlFailureReason::NONE;
      output.invalid_reason = "short-horizon exact-port witness unavailable; U+ " +
          positive_failure + "; U_safe " + safe_failure;
      return false;
    }
    output.execution.mode = selected_nonnegative
        ? RuntimeExecutionMode::SAFETY_PRIORITY : RuntimeExecutionMode::NORMAL;
    if (selected_nonnegative) {
      output.delta_ref = 0.0;
      output.profile_active = false;
    }
  } else {
    if (!project_port(false, prepared.raw_port, output.projection)) {
    output.execution.mode = RuntimeExecutionMode::FATAL_CONTROL_FAILURE;
    output.execution.fatal_control_failure = true;
    output.execution.genuine_fatal_invariant = true;
    output.execution.failure_reason = ControlFailureReason::PORT_PROJECTOR_CONTRADICTION;
      output.invalid_reason = output.projection.invalid_reason.empty()
          ? "port projection is invalid" : output.projection.invalid_reason;
    return false;
    }
    phase_offset_core::MatchedPortInput matched_input;
    matched_input.geometry = prepared.geometry;
    matched_input.base_v_cmd = base_v_cmd;
    matched_input.base_w_dot = base_w_dot;
    matched_input.final_port = output.projection.final_port;
    if (!phase_offset_core::MatchedPort::evaluate(matched_input,
                                                   output.matched)) {
    output.execution.mode = RuntimeExecutionMode::FATAL_CONTROL_FAILURE;
    output.execution.fatal_control_failure = true;
    output.execution.genuine_fatal_invariant = true;
    output.execution.failure_reason =
        ControlFailureReason::MATCHED_PORT_INVARIANT;
    output.invalid_reason = output.matched.invalid_reason.empty()
        ? "matched port is invalid" : output.matched.invalid_reason;
    return false;
    }
  }
  output.selected = prepared.zero_gate_open;
  output.valid = true;
  output.invalid_reason.clear();
  output.exact_terminal_predicate = output.projection.valid &&
      output.projection.next_delta == 0.0;
  if (output.selected) {
    previous_final_port_ = output.projection.final_port;
    delta_ = output.projection.next_delta;
    const bool executed_manual_profile = output.execution.mode !=
        RuntimeExecutionMode::SAFETY_PRIORITY;
    if (prepared.should_start_profile && executed_manual_profile) {
      profile_started_ = true;
    }
    if (prepared.profile_active && executed_manual_profile) {
      profile_elapsed_ += prepared.dt;
    }
    if (returning_to_center_ && delta_ == 0.0) {
      profile_completed_ = true;
      returning_to_center_ = false;
    } else if (profile_started_ && !profile_completed_ &&
               profile_elapsed_ >= config_.manual.profile_period &&
               delta_ == 0.0) {
      profile_completed_ = true;
      returning_to_center_ = false;
    }
  }
  return true;
}

bool PhaseOffsetRuntime::makeCommitToken(
    const RuntimePreparedStep& prepared, const RuntimeStepOutput& output,
    RuntimeCommitToken& token) const {
  token = RuntimeCommitToken();
  if (!prepared.valid || !prepared.requires_base_guidance ||
      !output.selected || !output.valid || !output.projection.valid ||
      !output.matched.valid || !IsFinite(output.projection.final_port) ||
      !IsFinite(output.projection.next_delta) || !IsFinite(prepared.dt) ||
      prepared.dt <= 0.0) {
    return false;
  }
  token.expected_previous_final_port = previous_final_port_;
  token.expected_delta = delta_;
  token.next_previous_final_port = output.projection.final_port;
  token.next_delta = output.projection.next_delta;
  token.dt = prepared.dt;
  token.selected = true;
  token.valid = true;
  token.safety_priority = output.execution.mode ==
      RuntimeExecutionMode::SAFETY_PRIORITY;
  token.should_start_profile = prepared.should_start_profile;
  token.profile_active = prepared.profile_active;
  token.exact_terminal_predicate = output.exact_terminal_predicate;
  return true;
}

bool PhaseOffsetRuntime::commitToken(const RuntimeCommitToken& token) {
  if (!configuration_valid_ || !token.valid || !token.selected ||
      !IsFinite(token.expected_previous_final_port) ||
      !IsFinite(token.expected_delta) ||
      !IsFinite(token.next_previous_final_port) ||
      !IsFinite(token.next_delta) || !IsFinite(token.dt) || token.dt <= 0.0) {
    return false;
  }
  constexpr double kCommitTolerance = 1e-12;
  if (std::abs(previous_final_port_.u_w - token.expected_previous_final_port.u_w) >
          kCommitTolerance ||
      std::abs(previous_final_port_.u_delta -
               token.expected_previous_final_port.u_delta) > kCommitTolerance ||
      std::abs(delta_ - token.expected_delta) > kCommitTolerance) {
    return false;
  }
  previous_final_port_ = token.next_previous_final_port;
  delta_ = token.next_delta;
  if (token.should_start_profile && !token.safety_priority) {
    profile_started_ = true;
  }
  if (token.profile_active && !token.safety_priority) {
    profile_elapsed_ += token.dt;
  }
  if (returning_to_center_ && delta_ == 0.0) {
    profile_completed_ = true;
    returning_to_center_ = false;
  } else if (token.complete_profile && token.exact_terminal_predicate &&
             delta_ == 0.0) {
    profile_completed_ = true;
    returning_to_center_ = false;
  } else if (profile_started_ && !profile_completed_ &&
             profile_elapsed_ >= config_.manual.profile_period &&
             delta_ == 0.0) {
    profile_completed_ = true;
    returning_to_center_ = false;
  }
  return true;
}

void PhaseOffsetRuntime::commitTokenNoFail(
    const RuntimeCommitToken& token) noexcept {
  previous_final_port_ = token.next_previous_final_port;
  delta_ = token.next_delta;
  if (token.should_start_profile && !token.safety_priority) {
    profile_started_ = true;
  }
  if (token.profile_active && !token.safety_priority) {
    profile_elapsed_ += token.dt;
  }
  if (returning_to_center_ && delta_ == 0.0) {
    profile_completed_ = true;
    returning_to_center_ = false;
  } else if (token.complete_profile && token.exact_terminal_predicate &&
             delta_ == 0.0) {
    profile_completed_ = true;
    returning_to_center_ = false;
  } else if (profile_started_ && !profile_completed_ &&
             profile_elapsed_ >= config_.manual.profile_period &&
             delta_ == 0.0) {
    profile_completed_ = true;
    returning_to_center_ = false;
  }
}

bool PhaseOffsetRuntime::dryRun(const RuntimeDryRunInput& input,
                                RuntimeDryRunResult& result) const {
  result = RuntimeDryRunResult();
  // PhaseOffsetRuntime holds only value state and immutable profile owners;
  // this copy is the staging boundary for an uncommitted handoff.
  PhaseOffsetRuntime staged(*this);
  if (!staged.refreshPreflight(input.preflight)) return false;
  if (!staged.prepare(input.prepare, result.prepared)) return false;
  result.valid = staged.complete(result.prepared, input.base_v_cmd,
                                 input.base_w_dot,
                                 input.base_guidance_valid, result.step);
  return result.valid;
}

}  // namespace phase_offset_navigation
