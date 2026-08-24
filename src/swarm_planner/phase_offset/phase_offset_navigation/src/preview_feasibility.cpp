#include "phase_offset_navigation/preview_feasibility.h"

#include "phase_offset_navigation/tube_filter.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace phase_offset_navigation {
namespace {

constexpr double kTolerance = 1e-10;

bool Finite(const double value) { return std::isfinite(value); }

bool RevisionMatches(const std::uint64_t actual,
                     const std::uint64_t expected) {
  return expected == 0U || actual == expected;
}

}  // namespace

const char* previewStatusName(const PreviewStatus status) {
  switch (status) {
    case PreviewStatus::NOT_EVALUATED: return "NOT_EVALUATED";
    case PreviewStatus::FEASIBLE: return "FEASIBLE";
    case PreviewStatus::TARGET_OVERLAP: return "TARGET_OVERLAP";
    case PreviewStatus::RATE_LIMITED: return "RATE_LIMITED";
    case PreviewStatus::INFEASIBLE: return "INFEASIBLE";
    case PreviewStatus::STALE: return "STALE";
    case PreviewStatus::DENIED: return "DENIED";
    case PreviewStatus::CURRENT_STATE_UNSAFE: return "CURRENT_STATE_UNSAFE";
  }
  return "UNKNOWN";
}

bool PreviewFeasibility::evaluate(const PreviewFeasibilityInput& input,
                                  PreviewFeasibilityResult& output) {
  output = PreviewFeasibilityResult();
  output.path_revision = input.path_revision;
  output.frame_revision = input.frame_revision;
  output.profile_revision = input.profile_revision;
  output.zero_only = input.successor_zero_only;
  output.provenance = "phase_offset_navigation/preview/value-only";

  if (!RevisionMatches(input.path_revision, input.expected_path_revision) ||
      !RevisionMatches(input.frame_revision, input.expected_frame_revision) ||
      !RevisionMatches(input.profile_revision, input.expected_profile_revision)) {
    output.status = PreviewStatus::STALE;
    output.reason = "preview revision mismatch";
    return false;
  }
  if (!input.current_state_safe) {
    output.status = PreviewStatus::CURRENT_STATE_UNSAFE;
    output.reason = "current state is explicitly unsafe";
    return false;
  }
  if (input.temporary_deny) {
    output.status = PreviewStatus::DENIED;
    output.reason = "preview is temporarily denied";
    return false;
  }
  if (!input.profile || !input.profile->complete ||
      input.profile->samples.size() < 2U || !Finite(input.current_w) ||
      !Finite(input.current_delta) || !Finite(input.target_w) ||
      !Finite(input.target_delta) || !Finite(input.dt) || input.dt <= 0.0 ||
      !Finite(input.horizon) || input.horizon < 0.0 ||
      !Finite(input.max_phase_rate) || input.max_phase_rate < 0.0 ||
      !Finite(input.max_delta_rate) || input.max_delta_rate < 0.0 ||
      !Finite(input.max_delta_slew) || input.max_delta_slew < 0.0) {
    output.status = PreviewStatus::INFEASIBLE;
    output.reason = "preview input or profile is invalid";
    return false;
  }

  TubeBounds current;
  if (!TubeFilter::query(*input.profile, input.current_w, current) ||
      !current.valid || input.current_delta < current.lower - kTolerance ||
      input.current_delta > current.upper + kTolerance) {
    output.status = PreviewStatus::INFEASIBLE;
    output.reason = "current state is outside the geometric component";
    return false;
  }

  TubeBounds target;
  if (!TubeFilter::query(*input.profile, input.target_w, target) ||
      !target.valid) {
    output.status = PreviewStatus::INFEASIBLE;
    output.reason = "target phase is outside the preview domain";
    return false;
  }
  output.reachable_delta_lower = current.lower;
  output.reachable_delta_upper = current.upper;
  output.reachable_w_lower = input.current_w;
  output.reachable_w_upper = input.current_w;

  const double phase_distance = std::max(0.0, input.target_w - input.current_w);
  const double delta_distance = std::abs(input.target_delta - input.current_delta);
  if (input.max_phase_rate <= kTolerance && phase_distance > kTolerance) {
    output.status = PreviewStatus::RATE_LIMITED;
    output.rate_limited = true;
    output.reason = "phase rate cannot reach target";
    return false;
  }
  const double phase_time = input.max_phase_rate > kTolerance
      ? phase_distance / input.max_phase_rate : 0.0;
  const double delta_time = input.max_delta_rate > kTolerance
      ? delta_distance / input.max_delta_rate
      : (delta_distance <= kTolerance ? 0.0
                                      : std::numeric_limits<double>::infinity());
  const double slew_time = input.max_delta_slew > kTolerance
      ? std::max(0.0, std::abs(input.previous_u.u_delta) -
          input.max_delta_slew) / input.max_delta_slew : 0.0;
  output.required_time = std::max(phase_time, std::max(delta_time, slew_time));
  output.horizon_remaining = input.horizon - output.required_time;
  output.reachable_w_upper = input.current_w +
      input.max_phase_rate * input.horizon;
  output.reachable_delta_lower = std::max(
      current.lower, input.current_delta - input.max_delta_rate * input.horizon);
  output.reachable_delta_upper = std::min(
      current.upper, input.current_delta + input.max_delta_rate * input.horizon);
  if (!Finite(output.required_time) || output.required_time > input.horizon +
      kTolerance) {
    output.status = PreviewStatus::RATE_LIMITED;
    output.rate_limited = true;
    output.slowing_required = true;
    output.reason = "target is outside the rate-limited preview horizon";
    return false;
  }

  output.target_overlap = input.target_delta >= current.lower - kTolerance &&
      input.target_delta <= current.upper + kTolerance &&
      input.target_w >= input.current_w - kTolerance;
  output.reachable = true;
  output.valid = true;
  output.status = output.target_overlap ? PreviewStatus::TARGET_OVERLAP
                                        : PreviewStatus::FEASIBLE;
  output.reason = output.target_overlap ? "target overlaps current component"
                                        : "rate-limited preview is feasible";
  return true;
}

}  // namespace phase_offset_navigation
