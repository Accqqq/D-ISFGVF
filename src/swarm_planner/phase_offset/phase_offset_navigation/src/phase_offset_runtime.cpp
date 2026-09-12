#include "phase_offset_navigation/phase_offset_runtime.h"

#include <algorithm>
#include <cmath>
#include <exception>
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

bool SamePortExact(const phase_offset_core::PortCommand& first,
                   const phase_offset_core::PortCommand& second) {
  return IsFinite(first) && IsFinite(second) && first.u_w == second.u_w &&
      first.u_delta == second.u_delta;
}

bool SameStateExact(const TubeExecutionStateV2& first,
                    const TubeExecutionStateV2& second) {
  return first.finite() && second.finite() && first.w == second.w &&
      first.delta == second.delta &&
      SamePortExact(first.previous_u, second.previous_u);
}

bool SameIdentityExact(const TubeExecutionIdentityV2& first,
                       const TubeExecutionIdentityV2& second) {
  return first.execution_generation == second.execution_generation &&
      first.path_instance_id == second.path_instance_id &&
      first.path_revision == second.path_revision &&
      first.frame_revision == second.frame_revision &&
      first.frame_convention_id == second.frame_convention_id &&
      first.configuration_id == second.configuration_id &&
      first.map_instance_id == second.map_instance_id &&
      first.map_state_id == second.map_state_id &&
      first.accepted_sequence == second.accepted_sequence &&
      first.profile_id == second.profile_id &&
      first.binding_sequence == second.binding_sequence;
}

bool EmptyIdentity(const TubeExecutionIdentityV2& identity) {
  return identity.execution_generation == 0U &&
      identity.path_instance_id == 0U && identity.path_revision == 0U &&
      identity.frame_revision == 0U && identity.frame_convention_id == 0U &&
      identity.configuration_id == 0U && identity.map_instance_id == 0U &&
      identity.map_state_id == 0U && identity.accepted_sequence == 0U &&
      identity.profile_id == 0U && identity.binding_sequence == 0U;
}

// The Runtime does not prove copied-prefix geometry; the adapter owns that
// proof.  It does seal the smallest possible identity transition so a token
// prepared against one source binding cannot be replayed against another.
bool ValidBindingTransition(const TubeExecutionIdentityV2& expected,
                            const TubeExecutionIdentityV2& proposed) {
  return expected.complete() && proposed.complete() &&
      !SameIdentityExact(expected, proposed) &&
      expected.execution_generation == proposed.execution_generation &&
      expected.configuration_id == proposed.configuration_id &&
      expected.frame_convention_id == proposed.frame_convention_id &&
      expected.path_instance_id != proposed.path_instance_id &&
      expected.profile_id != proposed.profile_id &&
      expected.binding_sequence != std::numeric_limits<std::uint64_t>::max() &&
      proposed.binding_sequence == expected.binding_sequence + 1U;
}

bool ValidPreparedIdentityContract(const RuntimeV2PreparedStep& prepared) {
  if (!prepared.identity.complete() ||
      !prepared.expected_identity.complete()) {
    return false;
  }
  return prepared.binding_transition
      ? ValidBindingTransition(prepared.expected_identity, prepared.identity)
      : SameIdentityExact(prepared.expected_identity, prepared.identity);
}

bool SamePolicyExact(const NormalPreviewProductionPolicy& first,
                     const NormalPreviewProductionPolicy& second) {
  return first.immutable == second.immutable &&
      first.preview_horizon_w == second.preview_horizon_w &&
      first.sample_spacing_w == second.sample_spacing_w &&
      first.lower_nu == second.lower_nu && first.upper_nu == second.upper_nu &&
      first.b_tight == second.b_tight && first.b_open == second.b_open &&
      first.policy_revision == second.policy_revision &&
      first.configuration_identity == second.configuration_identity &&
      first.configuration_id == second.configuration_id;
}

bool SameIntervalExact(const TubeExecutionIntervalV2& first,
                       const TubeExecutionIntervalV2& second) {
  return first.valid == second.valid && first.w == second.w &&
      first.lower == second.lower && first.upper == second.upper;
}

bool SameTrackingExact(const TubeExecutionTrackingEvidenceV2& first,
                       const TubeExecutionTrackingEvidenceV2& second) {
  return first.valid == second.valid && first.physical_tangent_valid ==
      second.physical_tangent_valid && first.error_norm == second.error_norm &&
      first.error_bound == second.error_bound;
}

bool SameLimitsExact(const TubeExecutionLimitsV2& first,
                     const TubeExecutionLimitsV2& second) {
  return first.lower_phase_rate == second.lower_phase_rate &&
      first.upper_phase_rate == second.upper_phase_rate &&
      first.upper_nu == second.upper_nu && first.max_u_w == second.max_u_w &&
      first.max_u_delta == second.max_u_delta &&
      first.u_w_slew_rate == second.u_w_slew_rate &&
      first.u_delta_slew_rate == second.u_delta_slew_rate &&
      first.return_u_delta_max == second.return_u_delta_max &&
      first.return_u_delta_slew_rate == second.return_u_delta_slew_rate &&
      first.max_schedule_steps == second.max_schedule_steps &&
      first.max_work == second.max_work && first.valid == second.valid;
}

bool SameReserveExact(const TubeFiniteReserveV2& first,
                      const TubeFiniteReserveV2& second) {
  if (first.valid != second.valid || first.reserve_id != second.reserve_id ||
      !SameIdentityExact(first.identity, second.identity) ||
      !SameStateExact(first.initial, second.initial) ||
      !SameStateExact(first.terminal, second.terminal) ||
      first.steps.size() != second.steps.size() || first.dt != second.dt ||
      first.w_max != second.w_max || first.common_lower != second.common_lower ||
      first.common_upper != second.common_upper || first.cursor != second.cursor ||
      first.work_count != second.work_count || first.provenance != second.provenance ||
      first.reason != second.reason) {
    return false;
  }
  for (std::size_t index = 0U; index < first.steps.size(); ++index) {
    const TubeReserveStepV2& lhs = first.steps[index];
    const TubeReserveStepV2& rhs = second.steps[index];
    if (lhs.ordinal != rhs.ordinal || lhs.segment != rhs.segment ||
        !SameStateExact(lhs.before, rhs.before) ||
        !SameStateExact(lhs.after, rhs.after) ||
        !SamePortExact(lhs.command, rhs.command) ||
        lhs.base_phase_rate != rhs.base_phase_rate ||
        lhs.phase_rate_lower != rhs.phase_rate_lower ||
        lhs.phase_rate_upper != rhs.phase_rate_upper || lhs.valid != rhs.valid) {
      return false;
    }
  }
  return true;
}

bool SameLivePreviewBinding(const TubeLiveKPreviewV2& first,
                            const TubeLiveKPreviewV2& second) {
  if (first.valid != second.valid || first.min_width != second.min_width ||
      first.work_count != second.work_count || first.reason != second.reason ||
      first.nodes.size() != second.nodes.size() ||
      first.viability.proof_kind != second.viability.proof_kind ||
      first.viability.status != second.viability.status ||
      first.viability.valid != second.viability.valid ||
      first.viability.feasible != second.viability.feasible ||
      first.viability.current_w != second.viability.current_w ||
      first.viability.preview_start_w != second.viability.preview_start_w ||
      first.viability.preview_end_w != second.viability.preview_end_w ||
      first.viability.upper_u_delta != second.viability.upper_u_delta ||
      first.viability.work_count != second.viability.work_count ||
      first.viability.max_work != second.viability.max_work ||
      first.viability.reason != second.viability.reason) {
    return false;
  }
  for (std::size_t index = 0U; index < first.nodes.size(); ++index) {
    if (!SameIntervalExact(first.nodes[index], second.nodes[index])) return false;
  }
  return true;
}

bool SameAdmissionBinding(const TubeStepAdmissionV2& first,
                          const TubeStepAdmissionV2& second) {
  return first.valid == second.valid && first.status == second.status &&
      SameIdentityExact(first.identity, second.identity) &&
      SameStateExact(first.current, second.current) &&
      SameStateExact(first.successor, second.successor) &&
      SameLivePreviewBinding(first.live_k, second.live_k) &&
      SameReserveExact(first.successor_reserve, second.successor_reserve) &&
      first.crossed_breakpoint_count == second.crossed_breakpoint_count &&
      first.work_count == second.work_count && first.reason == second.reason &&
      first.provenance == second.provenance;
}

bool SamePreparedBinding(const RuntimeV2PreparedStep& first,
                         const RuntimeV2PreparedStep& second) {
  return first.valid == second.valid && first.profile == second.profile &&
      SameAdmissionBinding(first.admission, second.admission) &&
      SameIdentityExact(first.identity, second.identity) &&
      SameIdentityExact(first.expected_identity, second.expected_identity) &&
      first.binding_transition == second.binding_transition &&
      SameStateExact(first.expected_current, second.expected_current) &&
      SameStateExact(first.successor, second.successor) &&
      SamePortExact(first.selected_u, second.selected_u) &&
      SamePolicyExact(first.preview_policy, second.preview_policy) &&
      SameLimitsExact(first.limits, second.limits) &&
      SameTrackingExact(first.tracking, second.tracking) &&
      first.base_phase_rate == second.base_phase_rate &&
      first.phase_rate_lower == second.phase_rate_lower &&
      first.phase_rate_upper == second.phase_rate_upper &&
      first.horizon_w == second.horizon_w &&
      first.sample_spacing_w == second.sample_spacing_w &&
      first.upper_u_delta == second.upper_u_delta && first.dt == second.dt &&
      first.now == second.now &&
      first.applicability_deadline == second.applicability_deadline &&
      first.applicability_deadline_valid == second.applicability_deadline_valid &&
      first.max_work == second.max_work && first.provenance == second.provenance &&
      first.invalid_reason == second.invalid_reason &&
      ((!first.reserve_owner && !second.reserve_owner) ||
       (first.reserve_owner && second.reserve_owner &&
        SameReserveExact(*first.reserve_owner, *second.reserve_owner)));
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

bool ValidSectionLimits(const PhaseOffsetAllocatorBounds& limits,
                        const double dt, std::string& reason) {
  if (!IsFinite(limits.lower_nu) || !IsFinite(limits.upper_nu) ||
      limits.lower_nu < 0.0 || limits.upper_nu < limits.lower_nu ||
      !IsFinite(limits.u_w_abs_max) || limits.u_w_abs_max < 0.0 ||
      !IsFinite(limits.upper_u_delta) || limits.upper_u_delta < 0.0 ||
      !IsFinite(limits.u_w_slew_rate) || limits.u_w_slew_rate < 0.0 ||
      !IsFinite(limits.u_delta_slew_rate) ||
          limits.u_delta_slew_rate < 0.0 ||
      !IsFinite(limits.zoh_min_dt) || limits.zoh_min_dt < 0.0 ||
      !IsFinite(limits.zoh_max_dt) || limits.zoh_max_dt < 0.0 ||
      !IsFinite(limits.zoh_dt) || limits.zoh_dt < 0.0 ||
      !IsFinite(dt) || dt <= 0.0) {
    reason = "Section scalar limits or ZOH duration are invalid";
    return false;
  }
  if (limits.zoh_max_dt > 0.0 && limits.zoh_max_dt < limits.zoh_min_dt) {
    reason = "Section ZOH duration limits are inverted";
    return false;
  }
  if (limits.zoh_min_dt > 0.0 && dt < limits.zoh_min_dt) {
    reason = "Section tick is shorter than the minimum ZOH duration";
    return false;
  }
  if (limits.zoh_max_dt > 0.0 && dt > limits.zoh_max_dt) {
    reason = "Section tick exceeds the maximum ZOH duration";
    return false;
  }
  if (limits.zoh_dt > 0.0 && limits.zoh_dt != dt) {
    reason = "Section tick does not match the frozen ZOH duration";
    return false;
  }
  return true;
}

bool SectionSlewContains(const double previous, const double selected,
                         const double rate, const double dt) {
  if (!IsFinite(previous) || !IsFinite(selected) || !IsFinite(rate) ||
      rate < 0.0 || !IsFinite(dt) || dt <= 0.0) {
    return false;
  }
  const double step = rate * dt;
  if (!IsFinite(step)) return false;
  const double lower = previous - step;
  const double upper = previous + step;
  if (!IsFinite(lower) || !IsFinite(upper) || lower > upper) return false;
  return selected >= lower && selected <= upper;
}


#ifndef PHASE_OFFSET_RUNTIME_V2_PRODUCTION
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
#endif

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
  if (!returning_to_center_) {
    returning_to_center_ = true;
    bumpSectionStateRevision();
  }
}

bool PhaseOffsetRuntime::configurationValid() const { return configuration_valid_; }

bool PhaseOffsetRuntime::sectionConfigurationValid() const {
  // Section preparation uses only these three existing physical/configuration
  // values.  Legacy manual profile/preflight fields intentionally remain
  // outside this gate (the legacy configurationValid() contract is unchanged).
  return IsFinite(config_.tube.tracking_error_bound) &&
      config_.tube.tracking_error_bound >= 0.0 &&
      IsFinite(config_.tube.minimum_reference_speed) &&
      config_.tube.minimum_reference_speed > 0.0 &&
      IsFinite(config_.manual.tangent_speed_min) &&
      config_.manual.tangent_speed_min > 0.0;
}

void PhaseOffsetRuntime::bumpSectionStateRevision() noexcept {
  if (section_state_revision_ != std::numeric_limits<std::uint64_t>::max()) {
    ++section_state_revision_;
  }
}

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
  invalidateV2State();
  last_preflight_source_revision_ = 0U;
  have_preflight_source_revision_ = false;
  preflight_ = ManualPreflightResult();
  bumpSectionStateRevision();
}

#ifndef PHASE_OFFSET_RUNTIME_V2_PRODUCTION
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
  const bool returning_before = returning_to_center_;
  const bool completed_before = profile_completed_;
  if (profile_started_ || std::abs(delta_) > kTolerance) {
    returning_to_center_ = true;
    profile_completed_ = true;
  }
  if (returning_to_center_ != returning_before ||
      profile_completed_ != completed_before) {
    bumpSectionStateRevision();
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
    invalidateV2State();
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
    bumpSectionStateRevision();
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
  invalidateV2State();
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
  bumpSectionStateRevision();
  return true;
}

void PhaseOffsetRuntime::commitTokenNoFail(
    const RuntimeCommitToken& token) noexcept {
  previous_final_port_ = token.next_previous_final_port;
  delta_ = token.next_delta;
  invalidateV2State();
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
  bumpSectionStateRevision();
}
#endif

bool PhaseOffsetRuntime::prepareSection(
    const RuntimeSectionPrepareInput& input,
    RuntimeSectionPreparedStep& prepared) const {
  RuntimeSectionPreparedStep staged;
  const auto fail = [&prepared, &staged](const std::string& reason) {
    staged.valid_ = false;
    staged.reason_ = reason;
    prepared = std::move(staged);
    return false;
  };
  if (!sectionConfigurationValid()) {
    return fail("Section runtime configuration is invalid");
  }
  if (section_state_revision_ == std::numeric_limits<std::uint64_t>::max()) {
    return fail("Section runtime state revision is saturated");
  }
  if (!input.profile || input.preview == nullptr) {
    return fail("Section profile or immutable Preview is missing");
  }
  const NormalPreviewResult& preview = *input.preview;
  // M4C: the same rule as the allocator.  A rate-degraded but still
  // state-feasible immutable Preview is prepared; the allocator clips the
  // requested motion onto the corridor window instead of dropping the tick.
  if (preview.proof_kind != TubeViabilityProofKind::SECTION_PWL ||
      (!preview.valid) || !preview.feasible ||
      (preview.status != TubeViabilityStatus::FEASIBLE &&
       preview.status != TubeViabilityStatus::RATE_INFEASIBLE &&
       preview.status != TubeViabilityStatus::CURRENT_DELTA_OUTSIDE) ||
      preview.section_profile != input.profile.get() ||
      !preview.policy.numericValid()) {
    return fail("Section Preview is not a feasible immutable PWL result");
  }
  if (!preview.provenance.immutable ||
      preview.provenance.path_revision == 0U ||
      preview.provenance.frame_revision == 0U) {
    return fail("Section Preview path/frame provenance is unbound");
  }
  const phase_offset_core::PhaseOffsetGeometryState& geometry =
      input.matched.geometry;
  if (!geometry.valid || geometry.path_revision == 0U ||
      geometry.frame_revision == 0U ||
      geometry.path_revision != preview.provenance.path_revision ||
      geometry.frame_revision != preview.provenance.frame_revision ||
      geometry.w != preview.current_w ||
      geometry.delta != preview.evaluated_delta ||
      !IsFinite(geometry.r_w) || !IsFinite(geometry.T) ||
      !IsFinite(geometry.N) || !IsFinite(geometry.r) ||
      !IsFinite(geometry.error) || !IsFinite(geometry.w) ||
      !IsFinite(geometry.delta)) {
    return fail("Section geometry does not match Preview W/path/frame state");
  }
  const double error_norm = geometry.error.norm();
  const double reference_speed = geometry.r_w.norm();
  // M4C-2: the tracking-error bound is the assumption the tube was sized with,
  // not an enforceable command condition.  The offset is shared by the
  // reference and the physical command through B*u, so it cannot be the source
  // of this error; the error comes from base path tracking and the governor's
  // normal loop recovers it.  Holding the whole tick on it would freeze both
  // the vehicle and the reference and leave the error permanently outside.
  if (!IsFinite(error_norm) || !IsFinite(reference_speed) ||
      reference_speed < config_.tube.minimum_reference_speed) {
    return fail("Section geometry violates reference-speed bounds");
  }
  std::string limits_reason;
  if (!ValidSectionLimits(input.limits, input.dt, limits_reason)) {
    return fail(limits_reason);
  }
  if (!IsFinite(input.matched.base_v_cmd) ||
      !IsFinite(input.matched.base_w_dot) ||
      !IsFinite(input.matched.final_port)) {
    return fail("Section MatchedPort input is not finite");
  }
  if (!IsFinite(input.previous_u) ||
      !SamePortExact(input.previous_u, previous_final_port_) ||
      geometry.delta != delta_) {
    return fail("Section previous port or retained delta is stale");
  }
  if (std::abs(input.matched.final_port.u_w) > input.limits.u_w_abs_max ||
      std::abs(input.matched.final_port.u_delta) > input.limits.upper_u_delta ||
      !SectionSlewContains(previous_final_port_.u_w,
                           input.matched.final_port.u_w,
                           input.limits.u_w_slew_rate, input.dt) ||
      !SectionSlewContains(previous_final_port_.u_delta,
                           input.matched.final_port.u_delta,
                           input.limits.u_delta_slew_rate, input.dt)) {
    return fail("Section final port violates amplitude or slew limits");
  }

  phase_offset_core::MatchedPortOutput matched_output;
  if (!phase_offset_core::MatchedPort::evaluate(input.matched, matched_output) ||
      !matched_output.valid || !IsFinite(matched_output.v_cmd) ||
      !IsFinite(matched_output.physical_port) ||
      !IsFinite(matched_output.w_dot) || !IsFinite(matched_output.delta_dot)) {
    return fail(matched_output.invalid_reason.empty()
                    ? "Section MatchedPort evaluation failed"
                    : matched_output.invalid_reason);
  }
  const double tangent_speed = geometry.T.dot(matched_output.v_cmd);
  if (!IsFinite(tangent_speed) ||
      tangent_speed < config_.manual.tangent_speed_min) {
    return fail("Section MatchedPort tangential speed is below the configured minimum");
  }
  // M4E: the phase window is an allocator-owned bound (M4C).  When the base
  // guidance rate leaves [lower_nu, upper_nu] and the +/-u_w_abs_max authority
  // cannot bring it back, the allocator saturates on that authority boundary
  // and marks the tick.  Re-applying the same window to that saturated value
  // would convert an approved "take the boundary" tick into a persistent HOLD
  // of a planner-valid task.  The waiver is accepted only when this really is
  // that saturation: the base rate is outside the window and the selected
  // correction sits exactly on the boundary that reduces the violation.
  const bool base_rate_above_window =
      input.matched.base_w_dot > input.limits.upper_nu;
  const bool base_rate_below_window =
      input.matched.base_w_dot < input.limits.lower_nu;
  // The allocator also owns the slew budget: when the admissible phase-rate set
  // is more than one tick away from the last committed rate it saturates on the
  // slew boundary and ramps towards the set over the following ticks.  That
  // tick carries `phase_window_clipped` too, but its correction is not on the
  // amplitude boundary, so recognise the ramp as the second accepted form.
  const bool at_authority_boundary =
      (base_rate_above_window &&
       input.matched.final_port.u_w == -input.limits.u_w_abs_max) ||
      (base_rate_below_window &&
       input.matched.final_port.u_w == input.limits.u_w_abs_max);
  // Distance of one phase rate to the frozen window (0 when inside it).  A
  // saturated tick is legal while it does not move *away* from the window:
  // that covers both the amplitude boundary and every intermediate slew-ramp
  // point (including one that is momentarily stationary on the ramp).
  const auto window_distance = [](const double rate, const double lower,
                                  const double upper) {
    if (rate < lower) return lower - rate;
    if (rate > upper) return rate - upper;
    return 0.0;
  };
  const bool phase_window_approach =
      (base_rate_above_window || base_rate_below_window) &&
      window_distance(matched_output.w_dot, input.limits.lower_nu,
                      input.limits.upper_nu) <=
          window_distance(input.matched.base_w_dot + input.previous_u.u_w,
                          input.limits.lower_nu, input.limits.upper_nu) +
              1e-9;
  const bool phase_window_saturated =
      input.phase_window_saturated &&
      (at_authority_boundary || phase_window_approach);
  const bool phase_rate_within_window =
      matched_output.w_dot >= input.limits.lower_nu &&
      matched_output.w_dot <= input.limits.upper_nu &&
      matched_output.w_dot >= preview.policy.lower_nu &&
      matched_output.w_dot <= preview.policy.upper_nu;
  if (!IsFinite(matched_output.delta_dot) ||
      matched_output.delta_dot != input.matched.final_port.u_delta ||
      !IsFinite(matched_output.w_dot) ||
      (!phase_window_saturated && !phase_rate_within_window)) {
    return fail("Section MatchedPort phase rate is outside the frozen limits");
  }

  // Option B: the horizontal-section tube is the safe lateral set, so the hard
  // transverse constraint is the corridor cross-section itself.  The allocator
  // has already clipped the transverse command into the corridor one tick
  // ahead, and this seam only materialises the exact successor the command
  // produces.  The former per-step "reachable envelope conformance" arithmetic
  // is deliberately not applied here: it rejected ordinary boundary-tracking
  // lag (sub-millimetre in practice), which froze the phase and locked the
  // vehicle.
  // A saturated-low tick carries a negative rate because the guidance wants the
  // reference to retreat while the vehicle is behind it.  Non-reversing phase
  // progression wins: hold the phase coordinate for this tick.  Only the phase
  // rate is affected; the transverse port, the physical command and the
  // committed delta remain exactly the allocator's saturated pair, and the
  // recorded matched evidence is recomputed so it stays self-consistent.
  if (phase_window_saturated && matched_output.w_dot < 0.0) {
    matched_output.w_dot = 0.0;
    matched_output.matched_residual =
        (matched_output.v_cmd - geometry.r_w * matched_output.w_dot -
         geometry.N * matched_output.delta_dot) -
        (input.matched.base_v_cmd - geometry.r_w * input.matched.base_w_dot);
    matched_output.matched_residual_norm = matched_output.matched_residual.norm();
  }
  const double held_next_w = geometry.w + input.dt * matched_output.w_dot;
  const double held_next_delta =
      geometry.delta + input.dt * matched_output.delta_dot;
  if (!IsFinite(held_next_w) || !IsFinite(held_next_delta) ||
      held_next_w < geometry.w) {
    return fail("Section successor state is not representable");
  }
  staged.matched_output_ = matched_output;
  staged.selected_u_ = input.matched.final_port;
  staged.next_w_ = held_next_w;
  staged.next_delta_ = held_next_delta;
  staged.work_count_ = 0U;
  staged.producer_ = this;
  staged.expected_revision_ = section_state_revision_;
  staged.expected_delta_ = delta_;
  staged.expected_previous_u = previous_final_port_;
  staged.valid_ = true;
  staged.reason_.clear();
  prepared = std::move(staged);
  return true;
}

bool PhaseOffsetRuntime::validateSectionBeforePublish(
    const RuntimeSectionPreparedStep& prepared) const {
  if (!prepared.valid_ || prepared.producer_ != this ||
      prepared.expected_revision_ != section_state_revision_ ||
      section_state_revision_ == std::numeric_limits<std::uint64_t>::max() ||
      !sectionConfigurationValid() ||
      !IsFinite(prepared.next_w_) || !IsFinite(prepared.next_delta_)) {
    return false;
  }
  if (!IsFinite(prepared.expected_delta_) ||
      prepared.expected_delta_ != delta_ ||
      !SamePortExact(prepared.expected_previous_u, previous_final_port_) ||
      !prepared.matched_output_.valid ||
      !IsFinite(prepared.matched_output_.v_cmd) ||
      !IsFinite(prepared.matched_output_.physical_port) ||
      !IsFinite(prepared.matched_output_.w_dot) ||
      !IsFinite(prepared.matched_output_.delta_dot) ||
      prepared.matched_output_.delta_dot != prepared.selected_u_.u_delta) {
    return false;
  }
  return true;
}

void PhaseOffsetRuntime::commitSectionNoFail(
    const RuntimeSectionPreparedStep& prepared) noexcept {
  // The publication owner has already run validateSectionBeforePublish() and
  // published the exact selected_u_ while holding its serial transaction lock.
  // Keep this seam bounded and non-fallible: a late rejection would split the
  // physical command from Runtime's ZOH state.
  previous_final_port_ = prepared.selected_u_;
  delta_ = prepared.next_delta_;
  // Section execution retires any V2 identity marker, but deliberately leaves
  // the immutable reserve owner for the outer lifecycle to reclaim outside
  // this no-fail lock seam.
  v2_state_valid_ = false;
  bumpSectionStateRevision();
}

bool PhaseOffsetRuntime::commitSection(
    const RuntimeSectionPreparedStep& prepared) {
  if (!validateSectionBeforePublish(prepared)) return false;
  commitSectionNoFail(prepared);
  return true;
}

bool PhaseOffsetRuntime::v2CurrentStateMatches(
    const TubeExecutionStateV2& state) const {
  if (!state.finite() || !IsFinite(delta_) ||
      !IsFinite(previous_final_port_)) {
    return false;
  }
  // Runtime owns only the transverse ZOH state.  The phase coordinate is
  // supplied by the caller's immutable execution transaction and is carried
  // through the V2 result/token as evidence; it is never installed here.
  return state.delta == delta_ &&
      SamePortExact(state.previous_u, previous_final_port_);
}

void PhaseOffsetRuntime::invalidateV2State() noexcept {
  v2_successor_reserve_.reset();
  v2_identity_ = TubeExecutionIdentityV2();
  v2_state_valid_ = false;
}

bool PhaseOffsetRuntime::prepareV2(const RuntimeV2PrepareInput& input,
                                   RuntimeV2PreparedStep& prepared) {
  RuntimeV2PreparedStep staged;
  const auto fail = [&prepared, &staged](const std::string& reason) {
    staged.valid = false;
    staged.invalid_reason = reason;
    prepared = std::move(staged);
    return false;
  };
  if (!configuration_valid_) {
    return fail("runtime configuration is invalid");
  }
  if (input.profile == nullptr || !input.identity.complete() ||
      !input.current.finite() || !v2CurrentStateMatches(input.current)) {
    return fail("V2 current execution state is stale or unavailable");
  }
  TubeExecutionIdentityV2 expected_identity = input.identity;
  if (input.binding_transition) {
    if (!ValidBindingTransition(input.expected_identity, input.identity)) {
      return fail("V2 binding transition identity is invalid");
    }
    expected_identity = input.expected_identity;
    if (!v2_state_valid_) {
      return fail("V2 binding transition has no committed source identity");
    }
  } else if (!EmptyIdentity(input.expected_identity)) {
    if (!input.expected_identity.complete() ||
        !SameIdentityExact(input.expected_identity, input.identity)) {
      return fail("V2 current identity expectation is invalid");
    }
    expected_identity = input.expected_identity;
  }
  if (v2_state_valid_ &&
      !SameIdentityExact(v2_identity_, expected_identity)) {
    return fail("V2 execution identity is stale");
  }
  if (!IsFinite(input.current.w) || !IsFinite(input.horizon_w)) {
    return fail("V2 phase/window input is invalid");
  }

  TubeExecutionAdmissionInputV2 admission_input;
  admission_input.profile = input.profile.get();
  admission_input.identity = input.identity;
  admission_input.current = input.current;
  admission_input.selected_u = input.selected_u;
  admission_input.selected_u_owner = input.selected_u_owner;
  admission_input.base_phase_rate = input.base_phase_rate;
  admission_input.phase_rate_lower = input.phase_rate_lower;
  admission_input.phase_rate_upper = input.phase_rate_upper;
  admission_input.horizon_w = input.horizon_w;
  admission_input.sample_spacing_w = input.sample_spacing_w;
  admission_input.preview_policy = input.preview_policy;
  admission_input.upper_u_delta = input.upper_u_delta;
  admission_input.dt = input.dt;
  admission_input.now = input.now;
  admission_input.applicability_deadline = input.applicability_deadline;
  admission_input.applicability_deadline_valid =
      input.applicability_deadline_valid;
  admission_input.limits = input.limits;
  admission_input.tracking = input.tracking;
  admission_input.max_work = input.max_work;
  admission_input.provenance = input.provenance;

  TubeStepAdmissionV2 admission;
  if (!TubeExecutionGuardV2::prepareAdmission(admission_input, admission)) {
    staged.admission = std::move(admission);
    return fail(staged.admission.reason.empty()
                    ? "V2 execution admission was rejected"
                    : staged.admission.reason);
  }
  if (!admission.valid || admission.status != TubeExecutionStatusV2::ADMISSIBLE ||
      !SameIdentityExact(admission.identity, input.identity) ||
      !SameStateExact(admission.current, input.current) ||
      !SamePortExact(admission.successor.previous_u, input.selected_u) ||
      !SameStateExact(admission.successor, admission.successor_reserve.initial) ||
      !admission.successor_reserve.valid ||
      !admission.successor_reserve.terminalExact() ||
      !admission.successor_reserve.identity.complete()) {
    staged.admission = std::move(admission);
    return fail("V2 admission result is not an exact successor");
  }
  std::shared_ptr<const TubeFiniteReserveV2> reserve_owner;
  try {
    reserve_owner = std::make_shared<const TubeFiniteReserveV2>(
        admission.successor_reserve);
  } catch (const std::exception&) {
    staged.admission = std::move(admission);
    return fail("V2 successor reserve could not be materialized");
  }
  staged.admission = std::move(admission);
  staged.profile = input.profile;
  staged.identity = input.identity;
  staged.expected_identity = expected_identity;
  staged.binding_transition = input.binding_transition;
  staged.expected_current = input.current;
  staged.successor = staged.admission.successor;
  staged.selected_u = input.selected_u;
  staged.preview_policy = input.preview_policy;
  staged.limits = input.limits;
  staged.tracking = input.tracking;
  staged.base_phase_rate = input.base_phase_rate;
  staged.phase_rate_lower = input.phase_rate_lower;
  staged.phase_rate_upper = input.phase_rate_upper;
  // P07/P08 own the exact outward endpoint.  Do not retain the caller's
  // ordinary `current_w + horizon` arithmetic as a second authority.
  staged.horizon_w = staged.admission.live_k.viability.preview_end_w;
  staged.sample_spacing_w = input.sample_spacing_w;
  staged.upper_u_delta = input.upper_u_delta;
  staged.dt = input.dt;
  staged.now = input.now;
  staged.applicability_deadline = input.applicability_deadline;
  staged.applicability_deadline_valid = input.applicability_deadline_valid;
  staged.max_work = input.max_work;
  staged.provenance = input.provenance;
  staged.reserve_owner = std::move(reserve_owner);
  staged.valid = true;
  staged.invalid_reason.clear();
  prepared = std::move(staged);
  return true;
}

bool PhaseOffsetRuntime::dryRunV2(const RuntimeV2PrepareInput& input,
                                  RuntimeV2PreparedStep& prepared) const {
  // Keep dry-run physically separate from the live object even though
  // prepareV2 is currently pure; this preserves the no-authority guarantee if
  // the staging implementation gains additional value-only checks later.
  PhaseOffsetRuntime staged(*this);
  return staged.prepareV2(input, prepared);
}

bool PhaseOffsetRuntime::makeCommitTokenV2(
    const RuntimeV2PreparedStep& prepared,
    RuntimeV2CommitToken& token) const {
  token = RuntimeV2CommitToken();
  if (!prepared.valid || !prepared.admission.valid ||
      prepared.admission.status != TubeExecutionStatusV2::ADMISSIBLE ||
      prepared.profile == nullptr || !ValidPreparedIdentityContract(prepared) ||
      !prepared.expected_current.finite() || !prepared.successor.finite() ||
      !SamePortExact(prepared.selected_u, prepared.successor.previous_u) ||
      !prepared.reserve_owner ||
      !SameReserveExact(*prepared.reserve_owner,
                        prepared.admission.successor_reserve) ||
      !v2CurrentStateMatches(prepared.expected_current)) {
    return false;
  }
  if ((prepared.binding_transition && !v2_state_valid_) ||
      (v2_state_valid_ &&
       !SameIdentityExact(v2_identity_, prepared.expected_identity))) {
    return false;
  }
  try {
    token.prepared = prepared;
    token.sealed_prepared =
        std::make_shared<const RuntimeV2PreparedStep>(prepared);
  } catch (const std::exception&) {
    token = RuntimeV2CommitToken();
    return false;
  }
  token.valid = true;
  return true;
}

bool PhaseOffsetRuntime::commitV2(const RuntimeV2CommitToken& token) {
  if (!configuration_valid_ || !token.valid || !token.sealed_prepared) {
    return false;
  }
  const RuntimeV2PreparedStep& sealed = *token.sealed_prepared;
  if (!SamePreparedBinding(token.prepared, sealed) || !sealed.valid ||
      !sealed.admission.valid ||
      sealed.admission.status != TubeExecutionStatusV2::ADMISSIBLE ||
      sealed.profile == nullptr || !ValidPreparedIdentityContract(sealed) ||
      !sealed.expected_current.finite() || !sealed.successor.finite() ||
      !SamePortExact(sealed.selected_u, sealed.successor.previous_u) ||
      !sealed.reserve_owner ||
      !SameReserveExact(*sealed.reserve_owner,
                        sealed.admission.successor_reserve) ||
      !v2CurrentStateMatches(sealed.expected_current)) {
    return false;
  }
  // A V2 token cannot be replayed against a replacement immutable binding.
  // Legacy commits invalidate this marker, so a subsequent V2 preparation must
  // capture the new current state afresh.
  if ((sealed.binding_transition && !v2_state_valid_) ||
      (v2_state_valid_ &&
       !SameIdentityExact(v2_identity_, sealed.expected_identity))) {
    return false;
  }
  commitV2NoFail(token);
  return true;
}

void PhaseOffsetRuntime::commitV2NoFail(
    const RuntimeV2CommitToken& token) noexcept {
  if (!token.valid || !token.sealed_prepared) return;
  const RuntimeV2PreparedStep& prepared = *token.sealed_prepared;
  // All fallible validation (including public/sealed token equality and the
  // expected-vs-proposed identity CAS) is completed by commitV2 before this
  // publication-gated no-fail seam.  Consume only the immutable sealed value
  // here; adding a late predicate would permit a published command and
  // Runtime state to diverge.
  previous_final_port_ = prepared.successor.previous_u;
  delta_ = prepared.successor.delta;
  v2_successor_reserve_ = prepared.reserve_owner;
  v2_identity_ = prepared.identity;
  v2_state_valid_ = true;
  bumpSectionStateRevision();
}

#ifndef PHASE_OFFSET_RUNTIME_V2_PRODUCTION
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
#endif

}  // namespace phase_offset_navigation
