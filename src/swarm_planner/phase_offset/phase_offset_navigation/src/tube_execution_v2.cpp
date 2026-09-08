#include "phase_offset_navigation/tube_execution_v2.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace phase_offset_navigation {
namespace {

bool Finite(const double value) { return std::isfinite(value); }

bool Finite(const phase_offset_core::PortCommand& command) {
  return Finite(command.u_w) && Finite(command.u_delta);
}

double Up(const double value) {
  return Finite(value)
      ? std::nextafter(value, std::numeric_limits<double>::infinity())
      : std::numeric_limits<double>::quiet_NaN();
}

bool ExactIdentity(const TubeExecutionIdentityV2& identity,
                   const TubeProfileV2& profile) {
  return identity.complete() && profile.profile_id == identity.profile_id &&
      profile.path_key.execution_generation == identity.execution_generation &&
      profile.path_key.path_instance_id == identity.path_instance_id &&
      profile.path_key.path_revision == identity.path_revision &&
      profile.path_key.frame_revision == identity.frame_revision &&
      profile.path_key.frame_convention_id == identity.frame_convention_id &&
      profile.configuration_key.configuration_id == identity.configuration_id &&
      profile.map_capture_key.map_instance_id == identity.map_instance_id &&
      profile.map_capture_key.state_id == identity.map_state_id &&
      profile.map_capture_key.accepted_sequence == identity.accepted_sequence;
}

bool SamePreviewPolicy(const NormalPreviewProductionPolicy& first,
                       const NormalPreviewProductionPolicy& second) {
  return first.immutable == second.immutable &&
      first.preview_horizon_w == second.preview_horizon_w &&
      first.sample_spacing_w == second.sample_spacing_w &&
      first.lower_nu == second.lower_nu &&
      first.upper_nu == second.upper_nu && first.b_tight == second.b_tight &&
      first.b_open == second.b_open &&
      first.policy_revision == second.policy_revision &&
      first.configuration_identity == second.configuration_identity &&
      first.configuration_id == second.configuration_id;
}

bool OrderedKnots(const TubeProfileV2& profile) {
  if (profile.knots.size() < 2U) return false;
  for (std::size_t index = 0U; index < profile.knots.size(); ++index) {
    const TubePwlKnotV2& knot = profile.knots[index];
    if (!knot.valid || !Finite(knot.w) || !Finite(knot.lower) ||
        !Finite(knot.upper) || knot.lower > knot.upper ||
        (index > 0U && !(knot.w > profile.knots[index - 1U].w))) {
      return false;
    }
  }
  return true;
}

bool MakeNodes(const TubeProfileV2& profile, const double first,
               const double last, std::vector<double>& nodes,
               std::string& reason) {
  nodes.clear();
  if (!Finite(first) || !Finite(last) || last < first ||
      first < profile.certified_start || last > profile.certified_end ||
      !OrderedKnots(profile)) {
    reason = "profile range or PWL knots are invalid";
    return false;
  }
  nodes.push_back(first);
  for (const TubePwlKnotV2& knot : profile.knots) {
    if (knot.w > first && knot.w < last) nodes.push_back(knot.w);
  }
  if (last != first) nodes.push_back(last);
  std::sort(nodes.begin(), nodes.end());
  nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
  if (nodes.empty() || nodes.front() != first || nodes.back() != last) {
    reason = "profile PWL node merge is malformed";
    return false;
  }
  return true;
}

bool IntervalAt(const TubeProfileV2& profile, const double w,
                TubeExecutionIntervalV2& output) {
  double lower = 0.0;
  double upper = 0.0;
  if (!profile.evaluate(w, lower, upper) || !Finite(lower) ||
      !Finite(upper) || lower > upper) {
    return false;
  }
  output.w = w;
  output.lower = lower;
  output.upper = upper;
  output.valid = true;
  return true;
}

bool KAt(const TubeLiveKPreviewV2& preview, const double w,
         TubeExecutionIntervalV2& output) {
  if (!preview.valid || !preview.viability.valid || !Finite(w)) return false;
  TubeViabilityInterval interval;
  if (!TubeViability::queryEnvelope(preview.viability, w, interval) ||
      !interval.valid || !Finite(interval.lower) ||
      !Finite(interval.upper) || interval.lower > interval.upper) {
    return false;
  }
  output.w = w;
  output.lower = interval.lower;
  output.upper = interval.upper;
  output.valid = true;
  return true;
}

bool BuildLiveKFromViability(const TubeProfileV2& profile,
                            const TubeExecutionAdmissionInputV2& input,
                            const std::size_t work_limit,
                            TubeLiveKPreviewV2& output) {
  output = TubeLiveKPreviewV2();
  TubeViabilityInput viability_input;
  viability_input.current_w = input.current.w;
  viability_input.current_delta = input.current.delta;
  viability_input.policy = input.preview_policy;
  viability_input.upper_u_delta = input.upper_u_delta;
  viability_input.max_work = work_limit;
  viability_input.v2_provenance_bound = true;
  viability_input.expected_v2_path_key = profile.path_key;
  viability_input.expected_v2_configuration_key = profile.configuration_key;
  viability_input.expected_v2_map_capture_key = profile.map_capture_key;
  TubeViabilityResult viability;
  bool evaluated = false;
  if (input.preview_result != nullptr) {
    viability = *input.preview_result;
  } else {
    if (!input.preview_policy.valid()) {
      output.reason = "captured V2 preview policy is unavailable";
      return false;
    }
    evaluated = TubeViability::evaluate(profile, viability_input, viability);
  }
  if ((!evaluated && input.preview_result == nullptr) ||
      viability.proof_kind != TubeViabilityProofKind::V2_LIVE_PWL ||
      viability.path_key != profile.path_key ||
      viability.configuration_key != profile.configuration_key ||
      viability.map_capture_key != profile.map_capture_key ||
      viability.profile_id != profile.profile_id ||
      viability.upper_u_delta != input.upper_u_delta ||
      !SamePreviewPolicy(viability.policy, input.preview_policy) ||
      viability.policy.sample_spacing_w != input.sample_spacing_w ||
      viability.max_work == 0U || viability.work_count > viability.max_work ||
      viability.work_count > work_limit ||
      !viability.valid || !viability.feasible ||
      viability.preview_start_w != input.current.w ||
      !Finite(viability.preview_end_w) ||
      viability.preview_end_w <= viability.preview_start_w ||
      viability.preview_end_w > profile.certified_end ||
      viability.work_count > work_limit) {
    output.viability = viability;
    output.reason = viability.reason.empty()
        ? "live V2 viability result is unavailable" : viability.reason;
    return false;
  }
  output.viability = viability;
  output.nodes.reserve(viability.knots.size());
  for (const TubeViabilityKnot& knot : viability.knots) {
    if (!knot.reachable.valid || !Finite(knot.w) ||
        !Finite(knot.reachable.lower) || !Finite(knot.reachable.upper) ||
        knot.reachable.lower > knot.reachable.upper) {
      output = TubeLiveKPreviewV2();
      output.reason = "live V2 viability knot is malformed";
      return false;
    }
    TubeExecutionIntervalV2 node;
    node.w = knot.w;
    node.lower = knot.reachable.lower;
    node.upper = knot.reachable.upper;
    node.valid = true;
    output.nodes.push_back(node);
  }
  output.min_width = viability.b_pre;
  output.work_count = viability.work_count;
  output.valid = true;
  return true;
}

bool CheckHeldSegment(const TubeProfileV2& profile,
                      const TubeExecutionStateV2& before,
                      const phase_offset_core::PortCommand& command,
                      const double base_phase_rate, const double dt,
                      const TubeExecutionIntervalV2* corridor,
                      const TubeLiveKPreviewV2* live_k,
                      const std::size_t count_limit,
                      std::size_t& crossed) {
  const double nu = base_phase_rate + command.u_w;
  const double after_w = before.w + dt * nu;
  const double after_delta = before.delta + dt * command.u_delta;
  if (!Finite(nu) || !Finite(after_w) || !Finite(after_delta) ||
      after_w < before.w) {
    return false;
  }
  std::vector<double> times;
  times.push_back(0.0);
  times.push_back(dt);
  if (nu > 0.0) {
    for (const TubePwlKnotV2& knot : profile.knots) {
      if (knot.w > before.w && knot.w < after_w) {
        const double time = (knot.w - before.w) / nu;
        if (!Finite(time) || time <= 0.0 || time >= dt) return false;
        times.push_back(time);
      }
    }
    if (live_k != nullptr) {
      for (const TubeExecutionIntervalV2& node : live_k->nodes) {
        if (node.w > before.w && node.w < after_w) {
          const double time = (node.w - before.w) / nu;
          if (!Finite(time) || time <= 0.0 || time >= dt) return false;
          times.push_back(time);
        }
      }
    }
  }
  std::sort(times.begin(), times.end());
  times.erase(std::unique(times.begin(), times.end()), times.end());
  for (const double time : times) {
    if (time > 0.0 && time < dt) {
      if (crossed >= count_limit) return false;
      ++crossed;
    }
  }
  for (const double time : times) {
    const double w = before.w + time * nu;
    const double delta = before.delta + time * command.u_delta;
    double lower = 0.0;
    double upper = 0.0;
    if (!profile.evaluate(w, lower, upper) || !Finite(delta) ||
        delta < lower || delta > upper) {
      return false;
    }
    if (corridor != nullptr &&
        (delta < corridor->lower || delta > corridor->upper)) {
      return false;
    }
    // The moving-window K is a separate inward contraction of the same
    // immutable I.  A held command must remain inside it at every endpoint,
    // every geometric knot, and every uniform preview breakpoint; checking
    // only the current and final K would permit an intermediate excursion.
    if (live_k != nullptr) {
      TubeExecutionIntervalV2 k_interval;
      if (!KAt(*live_k, w, k_interval) ||
          !k_interval.contains(delta)) {
        return false;
      }
    }
  }
  return true;
}

bool SafeCount(const double value, const std::size_t limit,
               std::size_t& result) {
  if (!Finite(value) || value < 0.0 || value >=
      static_cast<double>(std::numeric_limits<std::size_t>::max())) {
    return false;
  }
  const double rounded = std::ceil(value);
  if (!Finite(rounded) || rounded >=
      static_cast<double>(std::numeric_limits<std::size_t>::max()) ||
      rounded > static_cast<double>(limit)) {
    return false;
  }
  result = static_cast<std::size_t>(rounded);
  return true;
}

bool ValidateLimits(const TubeExecutionLimitsV2& limits) {
  return limits.valid && Finite(limits.lower_phase_rate) &&
      Finite(limits.upper_phase_rate) &&
      limits.lower_phase_rate <= limits.upper_phase_rate &&
      Finite(limits.upper_nu) && limits.upper_nu > 0.0 &&
      Finite(limits.max_u_w) && limits.max_u_w >= 0.0 &&
      Finite(limits.max_u_delta) && limits.max_u_delta >= 0.0 &&
      Finite(limits.u_w_slew_rate) && limits.u_w_slew_rate >= 0.0 &&
      Finite(limits.u_delta_slew_rate) && limits.u_delta_slew_rate >= 0.0 &&
      Finite(limits.return_u_delta_max) && limits.return_u_delta_max >= 0.0 &&
      Finite(limits.return_u_delta_slew_rate) &&
      limits.return_u_delta_slew_rate >= 0.0 && limits.max_schedule_steps > 0U &&
      limits.max_work >= limits.max_schedule_steps;
}

bool CheckProfile(const TubeProfileV2* profile,
                  const TubeExecutionIdentityV2& identity,
                  std::string& reason) {
  if (profile == nullptr) {
    reason = "immutable V2 profile is unavailable";
    return false;
  }
  if (!profile->structurallyValid() || !ExactIdentity(identity, *profile)) {
    reason = "profile identity or structural certificate is unavailable";
    return false;
  }
  return true;
}

bool AppendReserveStep(const TubeExecutionReserveInputV2& input,
                       TubeFiniteReserveV2& reserve,
                       TubeExecutionStateV2& state,
                       const TubeReserveSegmentKindV2 segment,
                       const phase_offset_core::PortCommand& command,
                       std::size_t& crossed) {
  const double return_amplitude = input.limits.return_u_delta_max > 0.0
      ? input.limits.return_u_delta_max : input.limits.max_u_delta;
  if (reserve.steps.size() >= input.max_schedule_steps ||
      reserve.work_count >= input.max_work || !Finite(command) ||
      std::abs(command.u_w) > input.limits.max_u_w ||
      std::abs(command.u_delta) > input.limits.max_u_delta ||
      (segment == TubeReserveSegmentKindV2::RETURN &&
       std::abs(command.u_delta) > return_amplitude)) {
    return false;
  }
  const double prev_w = state.previous_u.u_w;
  const double prev_delta = state.previous_u.u_delta;
  const double delta_slew_rate =
      segment == TubeReserveSegmentKindV2::RETURN &&
              input.limits.return_u_delta_slew_rate > 0.0
          ? input.limits.return_u_delta_slew_rate
          : input.limits.u_delta_slew_rate;
  const bool w_slew_valid = input.limits.u_w_slew_rate == 0.0
      ? command.u_w == prev_w
      : std::abs(command.u_w - prev_w) <=
            input.limits.u_w_slew_rate * input.dt;
  const bool delta_slew_valid = delta_slew_rate == 0.0
      ? command.u_delta == prev_delta
      : std::abs(command.u_delta - prev_delta) <=
            delta_slew_rate * input.dt;
  if (!w_slew_valid || !delta_slew_valid) {
    return false;
  }
  if (input.phase_rate_lower + command.u_w < 0.0 ||
      input.phase_rate_upper + command.u_w > input.approved_upper_nu ||
      !Finite(input.phase_rate_lower + command.u_w) ||
      !Finite(input.phase_rate_upper + command.u_w)) {
    return false;
  }
  TubeExecutionStateV2 next = state;
  next.w = state.w + input.dt * (input.base_phase_rate + command.u_w);
  next.delta = state.delta + input.dt * command.u_delta;
  next.previous_u = command;
  if (!next.finite() || next.w < state.w || next.w > input.profile->certified_end) {
    return false;
  }
  ++reserve.work_count;
  if (!CheckHeldSegment(*input.profile, state, command,
                        input.base_phase_rate, input.dt, nullptr, nullptr,
                        input.max_work,
                        crossed)) {
    return false;
  }
  TubeReserveStepV2 step;
  step.ordinal = reserve.steps.size();
  step.segment = segment;
  step.before = state;
  step.after = next;
  step.command = command;
  step.base_phase_rate = input.base_phase_rate;
  step.phase_rate_lower = input.phase_rate_lower + command.u_w;
  step.phase_rate_upper = input.phase_rate_upper + command.u_w;
  step.valid = true;
  reserve.steps.push_back(step);
  state = next;
  return true;
}

bool CommonCorridor(const TubeProfileV2& profile, const double first,
                    const double last, double& lower, double& upper,
                    const std::size_t work_limit, std::size_t& work) {
  std::vector<double> nodes;
  std::string reason;
  if (!MakeNodes(profile, first, last, nodes, reason)) return false;
  lower = -std::numeric_limits<double>::infinity();
  upper = std::numeric_limits<double>::infinity();
  for (const double w : nodes) {
    TubeExecutionIntervalV2 interval;
    if (!IntervalAt(profile, w, interval)) return false;
    lower = std::max(lower, interval.lower);
    upper = std::min(upper, interval.upper);
    if (!Finite(lower) || !Finite(upper)) return false;
    if (work >= work_limit) return false;
    ++work;
    if (lower > upper) return false;
  }
  return lower <= upper;
}

bool ValidateReserveSteps(const TubeFiniteReserveV2& reserve,
                          std::string* reason) {
  const auto fail = [reason](const char* value) {
    if (reason != nullptr) *reason = value;
    return false;
  };
  if (!reserve.valid || reserve.reserve_id == 0U ||
      !reserve.identity.complete() || !reserve.initial.finite() ||
      !reserve.terminal.finite() || reserve.steps.empty() ||
      reserve.cursor > reserve.steps.size() ||
      !Finite(reserve.dt) || reserve.dt <= 0.0 ||
      !Finite(reserve.w_max) || reserve.w_max < reserve.initial.w ||
      !Finite(reserve.common_lower) ||
      !Finite(reserve.common_upper) || reserve.common_lower > reserve.common_upper ||
      reserve.initial.delta < reserve.common_lower ||
      reserve.initial.delta > reserve.common_upper ||
      reserve.provenance.empty()) {
    return fail("reserve identity, range, or terminal value is invalid");
  }
  TubeExecutionStateV2 state = reserve.initial;
  for (std::size_t index = 0U; index < reserve.steps.size(); ++index) {
    const TubeReserveStepV2& step = reserve.steps[index];
    if (!step.valid || step.ordinal != index || step.before.w != state.w ||
        step.before.delta != state.delta ||
        step.before.previous_u.u_w != state.previous_u.u_w ||
        step.before.previous_u.u_delta != state.previous_u.u_delta ||
        !Finite(step.command) || !step.after.finite() ||
        !Finite(step.base_phase_rate) ||
        !Finite(step.phase_rate_lower) ||
        !Finite(step.phase_rate_upper) ||
        step.phase_rate_lower > step.phase_rate_upper ||
        step.after.previous_u.u_w != step.command.u_w ||
        step.after.previous_u.u_delta != step.command.u_delta ||
        step.after.w != step.before.w + reserve.dt *
            (step.base_phase_rate + step.command.u_w) ||
        step.after.delta != step.before.delta + reserve.dt *
            step.command.u_delta || step.after.w < step.before.w ||
        step.after.w > reserve.w_max ||
        step.after.delta < reserve.common_lower ||
        step.after.delta > reserve.common_upper) {
      return fail("reserve step identity or recurrence is invalid");
    }
    state = step.after;
  }
  if (state.w != reserve.terminal.w || state.delta != reserve.terminal.delta ||
      state.previous_u.u_w != reserve.terminal.previous_u.u_w ||
      state.previous_u.u_delta != reserve.terminal.previous_u.u_delta ||
      state.delta != 0.0 || state.previous_u.u_w != 0.0 ||
      state.previous_u.u_delta != 0.0 || state.w < reserve.initial.w ||
      state.w > reserve.w_max || state.delta < reserve.common_lower ||
      state.delta > reserve.common_upper) {
    return fail("reserve terminal state is not exact zero");
  }
  return true;
}

}  // namespace

const char* tubeExecutionStatusName(const TubeExecutionStatusV2 status) {
  switch (status) {
    case TubeExecutionStatusV2::UNAVAILABLE: return "UNAVAILABLE";
    case TubeExecutionStatusV2::ADMISSIBLE: return "ADMISSIBLE";
    case TubeExecutionStatusV2::INVALID_INPUT: return "INVALID_INPUT";
    case TubeExecutionStatusV2::PROFILE_UNAVAILABLE:
      return "PROFILE_UNAVAILABLE";
    case TubeExecutionStatusV2::RANGE_UNAVAILABLE: return "RANGE_UNAVAILABLE";
    case TubeExecutionStatusV2::K_INFEASIBLE: return "K_INFEASIBLE";
    case TubeExecutionStatusV2::COMMAND_INFEASIBLE:
      return "COMMAND_INFEASIBLE";
    case TubeExecutionStatusV2::RESERVE_UNAVAILABLE:
      return "RESERVE_UNAVAILABLE";
    case TubeExecutionStatusV2::TRACKING_UNAVAILABLE:
      return "TRACKING_UNAVAILABLE";
    case TubeExecutionStatusV2::DEADLINE_EXPIRED:
      return "DEADLINE_EXPIRED";
    case TubeExecutionStatusV2::BUDGET_EXHAUSTED:
      return "BUDGET_EXHAUSTED";
  }
  return "UNAVAILABLE";
}

const char* tubeReserveSegmentKindName(const TubeReserveSegmentKindV2 kind) {
  switch (kind) {
    case TubeReserveSegmentKindV2::BRAKE: return "BRAKE";
    case TubeReserveSegmentKindV2::RETURN: return "RETURN";
    case TubeReserveSegmentKindV2::SETTLE: return "SETTLE";
  }
  return "SETTLE";
}

bool TubeExecutionStateV2::finite() const {
  return Finite(w) && Finite(delta) && Finite(previous_u);
}

bool TubeExecutionIdentityV2::complete() const {
  return execution_generation != 0U && path_instance_id != 0U &&
      path_revision != 0U && frame_revision != 0U &&
      frame_convention_id != 0U && configuration_id != 0U &&
      map_instance_id != 0U && map_state_id != 0U && accepted_sequence != 0U &&
      profile_id != 0U && binding_sequence != 0U;
}

bool TubeExecutionLimitsV2::complete() const { return ValidateLimits(*this); }

bool TubeExecutionIntervalV2::contains(const double value) const {
  return valid && Finite(value) && value >= lower && value <= upper;
}

bool TubeFiniteReserveV2::terminalExact() const {
  return valid && terminal.delta == 0.0 &&
      terminal.previous_u.u_delta == 0.0 && terminal.previous_u.u_w == 0.0;
}

bool TubeExecutionGuardV2::prepareReserve(
    const TubeExecutionReserveInputV2& input, TubeFiniteReserveV2& output) {
  output = TubeFiniteReserveV2();
  output.identity = input.identity;
  output.reserve_id = input.identity.profile_id;
  if (input.identity.binding_sequence != 0U) {
    output.reserve_id = input.identity.binding_sequence;
  }
  std::string reason;
  const std::size_t step_limit = input.max_schedule_steps == 0U
      ? input.limits.max_schedule_steps : input.max_schedule_steps;
  const std::size_t work_limit = input.max_work == 0U
      ? input.limits.max_work : input.max_work;
  if (!CheckProfile(input.profile, input.identity, reason) ||
      !ValidateLimits(input.limits) || !input.initial.finite() ||
      !Finite(input.base_phase_rate) || !Finite(input.phase_rate_lower) ||
      !Finite(input.phase_rate_upper) ||
      input.phase_rate_lower > input.phase_rate_upper ||
      !Finite(input.approved_upper_nu) || input.approved_upper_nu <= 0.0 ||
      input.phase_rate_lower < 0.0 || input.phase_rate_upper <= 0.0 ||
      input.phase_rate_lower > input.approved_upper_nu ||
      !Finite(input.dt) || input.dt <= 0.0 ||
      !Finite(input.now) || step_limit == 0U || work_limit == 0U ||
      step_limit > input.limits.max_schedule_steps ||
      work_limit > input.limits.max_work || input.provenance.empty()) {
    output.reason = reason.empty() ? "reserve input is unavailable" : reason;
    return false;
  }
  TubeExecutionReserveInputV2 work_input = input;
  work_input.max_schedule_steps = step_limit;
  work_input.max_work = work_limit;
  if (input.deadline_valid &&
      (!Finite(input.deadline) || input.now >= input.deadline)) {
    output.reason = "reserve applicability deadline expired";
    return false;
  }
  output.initial = input.initial;
  output.dt = input.dt;
  output.provenance = input.provenance;
  TubeExecutionStateV2 state = input.initial;
  const double delta_step = input.limits.u_delta_slew_rate * input.dt;
  const double w_step = input.limits.u_w_slew_rate * input.dt;
  if ((std::abs(state.previous_u.u_delta) > 0.0 &&
       (!Finite(delta_step) || delta_step <= 0.0)) ||
      (std::abs(state.previous_u.u_w) > 0.0 &&
       (!Finite(w_step) || w_step <= 0.0)) ||
      std::abs(state.previous_u.u_delta) > input.limits.max_u_delta ||
      std::abs(state.previous_u.u_w) > input.limits.max_u_w) {
    output.reason = "initial port cannot be slewed to zero";
    return false;
  }
  std::size_t brake_delta = 0U;
  std::size_t brake_w = 0U;
  const double max_brake_steps = static_cast<double>(step_limit);
  if (!SafeCount(delta_step > 0.0 ?
                     std::abs(state.previous_u.u_delta) / delta_step :
                     (std::abs(state.previous_u.u_delta) == 0.0 ? 0.0 :
                                                                  max_brake_steps + 1.0),
                 step_limit, brake_delta) ||
      !SafeCount(w_step > 0.0 ?
                     std::abs(state.previous_u.u_w) / w_step :
                     (std::abs(state.previous_u.u_w) == 0.0 ? 0.0 :
                                                              max_brake_steps + 1.0),
                 step_limit, brake_w)) {
    output.reason = "braking schedule exceeds finite budget";
    return false;
  }
  const std::size_t brake_steps = std::max(brake_delta, brake_w);
  std::size_t crossed = 0U;
  for (std::size_t index = 1U; index <= brake_steps; ++index) {
    const double delta_value = std::copysign(
        std::max(std::abs(input.initial.previous_u.u_delta) -
                     static_cast<double>(index) * delta_step,
                 0.0),
        input.initial.previous_u.u_delta);
    const double w_value = std::copysign(
        std::max(std::abs(input.initial.previous_u.u_w) -
                     static_cast<double>(index) * w_step,
                 0.0),
        input.initial.previous_u.u_w);
    phase_offset_core::PortCommand command;
    command.u_delta = delta_value;
    command.u_w = w_value;
    if (!AppendReserveStep(work_input, output, state,
                           TubeReserveSegmentKindV2::BRAKE, command,
                           crossed)) {
      output.reason = "braking command violates exact recurrence or bounds";
      return false;
    }
  }
  // The phase and transverse port are exactly zero after the last braking
  // command.  The resulting delta, including any outward braking overshoot,
  // is the only starting point for the fixed rest-to-rest return family.
  const double delta_b = state.delta;
  const double return_v = input.limits.return_u_delta_max > 0.0
      ? input.limits.return_u_delta_max : input.limits.max_u_delta;
  const double return_a = input.limits.return_u_delta_slew_rate > 0.0
      ? input.limits.return_u_delta_slew_rate :
        input.limits.u_delta_slew_rate;
  if (!Finite(delta_b) ||
      (delta_b != 0.0 && (return_v <= 0.0 || return_a <= 0.0))) {
    output.reason = "nonzero return has no finite amplitude/slew bound";
    return false;
  }
  if (delta_b != 0.0) {
    std::size_t ramp = 0U;
    std::size_t cruise = 0U;
    const double ramp_step = return_a * input.dt;
    const double cruise_step = return_v * input.dt;
    if (!SafeCount(return_v / ramp_step, step_limit, ramp) ||
        !SafeCount(std::abs(delta_b) / cruise_step, step_limit,
                   cruise) ||
        ramp == 0U || cruise == 0U) {
      output.reason = "return schedule count is unavailable";
      return false;
    }
    std::vector<double> shape;
    if (ramp > step_limit - cruise ||
        ramp - 1U > step_limit - ramp - cruise) {
      output.reason = "return schedule count overflows";
      return false;
    }
    if (ramp > (step_limit - cruise + 1U) / 2U) {
      output.reason = "return schedule shape count overflows";
      return false;
    }
    shape.reserve(2U * ramp + cruise - 1U);
    for (std::size_t index = 1U; index <= ramp; ++index) {
      shape.push_back(std::min(static_cast<double>(index) * ramp_step,
                               return_v));
    }
    for (std::size_t index = 0U; index < cruise; ++index) {
      shape.push_back(return_v);
    }
    if (ramp > 1U) {
      for (std::size_t index = ramp - 1U; index > 0U; --index) {
        shape.push_back(std::min(static_cast<double>(index) * ramp_step,
                                 return_v));
      }
    }
    double shape_sum = 0.0;
    for (const double value : shape) {
      shape_sum += value;
      if (!Finite(shape_sum)) {
        output.reason = "return shape sum overflow";
        return false;
      }
    }
    const double denominator = input.dt * shape_sum;
    const double lambda = std::abs(delta_b) / denominator;
    if (!Finite(lambda) || lambda <= 0.0 || lambda > 1.0) {
      output.reason = "return shape cannot cover exact delta";
      return false;
    }
    const double sign = delta_b > 0.0 ? -1.0 : 1.0;
    for (const double value : shape) {
      phase_offset_core::PortCommand command;
      command.u_w = 0.0;
      command.u_delta = sign * lambda * value;
      if (!AppendReserveStep(work_input, output, state,
                             TubeReserveSegmentKindV2::RETURN, command,
                             crossed)) {
        output.reason = "return command violates exact recurrence or bounds";
        return false;
      }
    }
    if (state.delta != 0.0) {
      phase_offset_core::PortCommand correction;
      correction.u_w = 0.0;
      correction.u_delta = -state.delta / input.dt;
      if (!Finite(correction.u_delta) ||
          (state.delta > 0.0 && correction.u_delta >= 0.0) ||
          (state.delta < 0.0 && correction.u_delta <= 0.0) ||
          !AppendReserveStep(work_input, output, state,
                             TubeReserveSegmentKindV2::RETURN, correction,
                             crossed)) {
        output.reason = "return residual cannot reach exact zero";
        return false;
      }
    }
  }
  // Explicit SETTLE records the zero transverse port and keeps the schedule
  // terminal even when braking/return had no nonzero work.
  phase_offset_core::PortCommand settle;
  settle.u_w = 0.0;
  settle.u_delta = 0.0;
  if (!AppendReserveStep(work_input, output, state,
                         TubeReserveSegmentKindV2::SETTLE, settle, crossed) ||
      state.delta != 0.0 || state.previous_u.u_delta != 0.0 ||
      state.previous_u.u_w != 0.0) {
    output.reason = "settle does not establish exact terminal state";
    return false;
  }
  output.terminal = state;
  output.w_max = output.initial.w;
  double bound_w = output.initial.w;
  for (const TubeReserveStepV2& step : output.steps) {
    const double upper = input.phase_rate_upper + step.command.u_w;
    double contribution_upper = 0.0;
    if (!Finite(upper) || upper < 0.0 ||
        !phase_offset_core::outwardUpperProduct(input.dt, upper,
                                                contribution_upper)) {
      output.reason = "source-backed phase bound overflow";
      return false;
    }
    const double summed_bound = bound_w + contribution_upper;
    if (!Finite(summed_bound)) {
      output.reason = "source-backed phase bound overflow";
      return false;
    }
    bound_w = Up(summed_bound);
    if (!Finite(bound_w)) {
      output.reason = "source-backed phase bound overflow";
      return false;
    }
  }
  output.w_max = bound_w;
  if (output.w_max > input.profile->certified_end) {
    output.reason = "reserve phase coverage exceeds certified profile range";
    return false;
  }
  if (!CommonCorridor(*input.profile, output.initial.w, output.w_max,
                      output.common_lower, output.common_upper,
                      work_limit,
                      output.work_count) || output.work_count > work_limit) {
    output.reason = "reserve common corridor is unavailable";
    return false;
  }
  TubeExecutionIntervalV2 corridor;
  corridor.lower = output.common_lower;
  corridor.upper = output.common_upper;
  corridor.valid = true;
  TubeExecutionStateV2 check = output.initial;
  std::size_t check_crossed = 0U;
  for (const TubeReserveStepV2& step : output.steps) {
    if (step.before.w != check.w || step.before.delta != check.delta ||
        !CheckHeldSegment(*input.profile, check, step.command,
                          input.base_phase_rate, input.dt, &corridor, nullptr,
                          work_limit,
                          check_crossed)) {
      output.reason = "reserve schedule leaves common corridor";
      return false;
    }
    check = step.after;
  }
  if (check.w != output.terminal.w || check.delta != output.terminal.delta ||
      check.previous_u.u_w != 0.0 || check.previous_u.u_delta != 0.0) {
    output.reason = "reserve terminal recurrence mismatch";
    return false;
  }
  output.valid = true;
  return true;
}

bool TubeExecutionGuardV2::prepareAdmission(
    const TubeExecutionAdmissionInputV2& input, TubeStepAdmissionV2& output) {
  output = TubeStepAdmissionV2();
  output.identity = input.identity;
  output.current = input.current;
  output.provenance = input.provenance;
  std::string reason;
  const std::size_t work_limit = input.max_work == 0U
      ? input.limits.max_work : input.max_work;
  if (!CheckProfile(input.profile, input.identity, reason)) {
    output.status = TubeExecutionStatusV2::PROFILE_UNAVAILABLE;
    output.reason = reason;
    return false;
  }
  if (!ValidateLimits(input.limits) || !input.current.finite() ||
      !Finite(input.selected_u) || input.selected_u_owner != "PhaseOffsetAllocator" ||
      !Finite(input.base_phase_rate) || !Finite(input.phase_rate_lower) ||
      !Finite(input.phase_rate_upper) ||
      input.phase_rate_lower > input.phase_rate_upper ||
      input.phase_rate_lower < 0.0 || input.phase_rate_upper <= 0.0 ||
      !Finite(input.horizon_w) ||
      !Finite(input.sample_spacing_w) || input.sample_spacing_w <= 0.0 ||
      !input.preview_policy.valid() ||
      input.preview_policy.lower_nu != input.phase_rate_lower ||
      input.preview_policy.upper_nu != input.limits.upper_nu ||
      input.preview_policy.sample_spacing_w != input.sample_spacing_w ||
      !Finite(input.upper_u_delta) || input.upper_u_delta < 0.0 ||
      !Finite(input.dt) || input.dt <= 0.0 ||
      work_limit == 0U || !Finite(input.now) ||
      input.provenance.empty()) {
    output.status = TubeExecutionStatusV2::INVALID_INPUT;
    output.reason = "execution admission input is invalid";
    return false;
  }
  if (input.applicability_deadline_valid &&
      (!Finite(input.applicability_deadline) ||
       input.now >= input.applicability_deadline)) {
    output.status = TubeExecutionStatusV2::DEADLINE_EXPIRED;
    output.reason = "execution applicability deadline expired";
    return false;
  }
  if (!input.tracking.valid || !input.tracking.physical_tangent_valid ||
      !Finite(input.tracking.error_norm) ||
      !Finite(input.tracking.error_bound) || input.tracking.error_bound < 0.0 ||
      input.tracking.error_norm < 0.0 ||
      input.tracking.error_norm > input.tracking.error_bound) {
    output.status = TubeExecutionStatusV2::TRACKING_UNAVAILABLE;
    output.reason = "tracking or physical-tangent evidence is unavailable";
    return false;
  }
  if (input.current.w < input.profile->certified_start ||
      input.current.w > input.profile->certified_end) {
    output.status = TubeExecutionStatusV2::RANGE_UNAVAILABLE;
    output.reason = "live normal horizon is outside certified profile range";
    return false;
  }
  if (input.phase_rate_lower + input.selected_u.u_w < 0.0 ||
      input.phase_rate_upper + input.selected_u.u_w > input.limits.upper_nu ||
      input.base_phase_rate + input.selected_u.u_w < 0.0 ||
      input.base_phase_rate + input.selected_u.u_w > input.limits.upper_nu ||
      std::abs(input.selected_u.u_w) > input.limits.max_u_w ||
      std::abs(input.selected_u.u_delta) > input.limits.max_u_delta) {
    output.status = TubeExecutionStatusV2::COMMAND_INFEASIBLE;
    output.reason = "selected port violates phase or amplitude bounds";
    return false;
  }
  const double delta_w = input.limits.u_delta_slew_rate * input.dt;
  const double delta_w_prev =
      std::abs(input.selected_u.u_delta - input.current.previous_u.u_delta);
  const double w_prev = std::abs(input.selected_u.u_w - input.current.previous_u.u_w);
  if ((input.limits.u_delta_slew_rate == 0.0 ? delta_w_prev != 0.0
                                             : delta_w_prev > delta_w) ||
      (input.limits.u_w_slew_rate == 0.0
           ? w_prev != 0.0
           : w_prev > input.limits.u_w_slew_rate * input.dt)) {
    output.status = TubeExecutionStatusV2::COMMAND_INFEASIBLE;
    output.reason = "selected port violates slew bounds";
    return false;
  }
  if (!BuildLiveKFromViability(*input.profile, input, work_limit,
                               output.live_k)) {
    output.status = TubeExecutionStatusV2::K_INFEASIBLE;
    output.reason = output.live_k.reason;
    return false;
  }
  if (output.live_k.viability.preview_end_w > input.profile->certified_end) {
    output.status = TubeExecutionStatusV2::RANGE_UNAVAILABLE;
    output.reason = "authoritative live normal horizon is outside certified profile range";
    return false;
  }
  TubeExecutionIntervalV2 current_k;
  if (!KAt(output.live_k, input.current.w, current_k) ||
      !current_k.contains(input.current.delta)) {
    output.status = TubeExecutionStatusV2::K_INFEASIBLE;
    output.reason = "live delta is outside inward K";
    return false;
  }
  const double next_w = input.current.w +
      input.dt * (input.base_phase_rate + input.selected_u.u_w);
  const double next_delta = input.current.delta +
      input.dt * input.selected_u.u_delta;
  const double authoritative_horizon_w = output.live_k.viability.preview_end_w;
  if (!Finite(next_w) || !Finite(next_delta) ||
      next_w > authoritative_horizon_w ||
      next_w > input.profile->certified_end ||
      !CheckHeldSegment(*input.profile, input.current, input.selected_u,
                        input.base_phase_rate, input.dt, nullptr,
                        &output.live_k, work_limit,
                        output.crossed_breakpoint_count)) {
    output.status = TubeExecutionStatusV2::COMMAND_INFEASIBLE;
    output.reason = "selected ZOH command leaves certified I";
    return false;
  }
  TubeExecutionIntervalV2 next_k;
  if (!KAt(output.live_k, next_w, next_k) || !next_k.contains(next_delta)) {
    output.status = TubeExecutionStatusV2::COMMAND_INFEASIBLE;
    output.reason = "selected ZOH successor leaves live K";
    return false;
  }
  output.successor.w = next_w;
  output.successor.delta = next_delta;
  output.successor.previous_u = input.selected_u;
  TubeExecutionReserveInputV2 reserve_input;
  reserve_input.profile = input.profile;
  reserve_input.identity = input.identity;
  reserve_input.initial = output.successor;
  reserve_input.base_phase_rate = input.base_phase_rate;
  reserve_input.phase_rate_lower = input.phase_rate_lower;
  reserve_input.phase_rate_upper = input.phase_rate_upper;
  reserve_input.approved_upper_nu = input.limits.upper_nu;
  reserve_input.dt = input.dt;
  reserve_input.now = input.now;
  reserve_input.deadline = input.applicability_deadline;
  reserve_input.deadline_valid = input.applicability_deadline_valid;
  reserve_input.limits = input.limits;
  reserve_input.max_schedule_steps =
      std::min(work_limit, input.limits.max_schedule_steps);
  reserve_input.max_work = work_limit;
  reserve_input.provenance = input.provenance + "/successor-reserve";
  if (!prepareReserve(reserve_input, output.successor_reserve)) {
    output.status = TubeExecutionStatusV2::RESERVE_UNAVAILABLE;
    output.reason = output.successor_reserve.reason;
    return false;
  }
  const std::size_t live_k_work = output.live_k.work_count;
  if (live_k_work > work_limit ||
      output.successor_reserve.work_count > work_limit - live_k_work) {
    output.status = TubeExecutionStatusV2::BUDGET_EXHAUSTED;
    output.reason = "admission work budget exhausted";
    return false;
  }
  // Reserve work already includes every scheduled command and common-corridor
  // node.  Charge those operations together with the live-K node work so the
  // reported total is truthful and cannot wrap a size_t.
  output.work_count = live_k_work + output.successor_reserve.work_count;
  output.valid = output.work_count <= work_limit;
  output.status = output.valid ? TubeExecutionStatusV2::ADMISSIBLE
                               : TubeExecutionStatusV2::BUDGET_EXHAUSTED;
  if (!output.valid) output.reason = "admission work budget exhausted";
  return output.valid;
}

bool TubeExecutionGuardV2::validateReserve(const TubeFiniteReserveV2& reserve,
                                            std::string* reason) {
  if (reason != nullptr) reason->clear();
  return ValidateReserveSteps(reserve, reason);
}

}  // namespace phase_offset_navigation
