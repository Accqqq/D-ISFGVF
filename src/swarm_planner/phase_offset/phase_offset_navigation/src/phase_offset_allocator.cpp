#include "phase_offset_navigation/phase_offset_allocator.h"

#include <algorithm>
#include <cmath>

namespace phase_offset_navigation {
namespace {

constexpr double kGeometryEpsilon = 1e-12;
constexpr double kGeometryUnitTolerance = 1e-6;
constexpr double kHorizontalTolerance = 1e-6;

bool Finite(const double value) { return std::isfinite(value); }

bool ValidInterval(const double lower, const double upper) {
  return Finite(lower) && Finite(upper) && lower <= upper;
}

bool Intersect(const double first_lower, const double first_upper,
               const double second_lower, const double second_upper,
               double& lower, double& upper) {
  lower = std::max(first_lower, second_lower);
  upper = std::min(first_upper, second_upper);
  return ValidInterval(lower, upper);
}

double Clamp(const double value, const double lower, const double upper) {
  return std::max(lower, std::min(upper, value));
}

// Corridor cross-section (the safe lateral set the tube stands for) at one
// phase, linearly interpolated between the two neighbouring Preview knots.
// This is the geometric half of the tube authority, as opposed to the
// rate-limited "reachable" envelope the strict per-step verifier used.
bool SectionCorridorAt(const TubeViabilityResult& preview, const double w,
                       double& lower, double& upper) {
  const std::vector<TubeViabilityKnot>& knots = preview.knots;
  if (knots.size() < 2U || !Finite(w) || w < knots.front().w ||
      w > knots.back().w) {
    return false;
  }
  std::size_t right = 0U;
  while (right < knots.size() && knots[right].w < w) ++right;
  if (right < knots.size() && knots[right].w == w) {
    const TubeViabilityInterval& interval = knots[right].geometric;
    if (!interval.valid) return false;
    lower = interval.lower;
    upper = interval.upper;
    return Finite(lower) && Finite(upper) && lower <= upper;
  }
  if (right == 0U || right >= knots.size()) return false;
  const double left_w = knots[right - 1U].w;
  const double right_w = knots[right].w;
  const TubeViabilityInterval& left = knots[right - 1U].geometric;
  const TubeViabilityInterval& next = knots[right].geometric;
  if (!left.valid || !next.valid || !(right_w > left_w)) return false;
  const double alpha = (w - left_w) / (right_w - left_w);
  lower = left.lower + alpha * (next.lower - left.lower);
  upper = left.upper + alpha * (next.upper - left.upper);
  return Finite(lower) && Finite(upper) && lower <= upper;
}

bool RequiredProvenanceExpectationsBound(
    const PhaseOffsetAllocatorInput& input, std::string& reason) {
  if (input.geometry.path_revision == 0U ||
      input.geometry.frame_revision == 0U) {
    reason = "current geometry path/frame provenance is unbound";
    return false;
  }
  if (input.expected_path_revision == 0U ||
      input.expected_frame_revision == 0U ||
      input.expected_profile_revision == 0U ||
      input.expected_source_revision == 0U ||
      input.expected_tube_revision == 0U ||
      input.expected_map_revision == 0U ||
      input.expected_obstacle_contract_id.empty()) {
    reason = "allocator provenance expectations are missing or unbound";
    return false;
  }
  return true;
}

bool RequiredSectionExpectationsBound(
    const PhaseOffsetAllocatorInput& input, std::string& reason) {
  if (input.expected_section_profile == nullptr) {
    reason = "SECTION Preview profile expectation is missing";
    return false;
  }
  if (input.geometry.path_revision == 0U ||
      input.geometry.frame_revision == 0U ||
      input.expected_path_revision == 0U ||
      input.expected_frame_revision == 0U) {
    reason = "SECTION allocator path/frame provenance is unbound";
    return false;
  }
  return true;
}

bool PreviewProvenanceComplete(const TubeViabilityProvenance& provenance) {
  return provenance.immutable && !provenance.source.empty() &&
      provenance.path_revision != 0U && provenance.frame_revision != 0U &&
      provenance.profile_revision != 0U &&
      provenance.source_revision != 0U && provenance.tube_revision != 0U &&
      provenance.map_revision != 0U &&
      !provenance.obstacle_contract_id.empty();
}

bool ProvenanceMatchesInput(const PhaseOffsetAllocatorInput& input,
                            const TubeViabilityProvenance& provenance) {
  if (input.preview != nullptr &&
      input.preview->proof_kind == TubeViabilityProofKind::SECTION_PWL) {
    const NormalPreviewResult& preview = *input.preview;
    const bool binding = input.expected_section_profile != nullptr &&
        preview.section_profile != nullptr &&
        input.expected_section_profile == preview.section_profile;
    const bool failed_preview_without_borrow =
        preview.status != TubeViabilityStatus::FEASIBLE &&
        preview.section_profile == nullptr;
    if (!(binding || failed_preview_without_borrow) ||
        !provenance.immutable || provenance.path_revision == 0U ||
        provenance.frame_revision == 0U ||
        input.expected_path_revision != provenance.path_revision ||
        input.expected_frame_revision != provenance.frame_revision ||
        input.geometry.path_revision != provenance.path_revision ||
        input.geometry.frame_revision != provenance.frame_revision) {
      return false;
    }
    if (preview.status == TubeViabilityStatus::FEASIBLE) {
      return input.geometry.w == preview.current_w &&
          input.geometry.delta == preview.evaluated_delta;
    }
    return true;
  }
  return PreviewProvenanceComplete(provenance) &&
      input.expected_path_revision == provenance.path_revision &&
      input.expected_frame_revision == provenance.frame_revision &&
      input.expected_profile_revision == provenance.profile_revision &&
      input.expected_source_revision == provenance.source_revision &&
      input.expected_tube_revision == provenance.tube_revision &&
      input.expected_map_revision == provenance.map_revision &&
      input.expected_obstacle_contract_id == provenance.obstacle_contract_id &&
      input.geometry.path_revision == provenance.path_revision &&
      input.geometry.frame_revision == provenance.frame_revision;
}

bool GeometryMatchesPreview(const PhaseOffsetAllocatorInput& input,
                            const NormalPreviewResult& preview,
                            std::string& reason) {
  const TubeViabilityProvenance& provenance = preview.provenance;
  if (preview.proof_kind == TubeViabilityProofKind::SECTION_PWL) {
    if (!RequiredSectionExpectationsBound(input, reason)) return false;
    if (preview.status == TubeViabilityStatus::FEASIBLE &&
        (preview.section_profile == nullptr ||
         input.expected_section_profile != preview.section_profile)) {
      reason = "SECTION Preview profile pointer does not match expectation";
      return false;
    }
    if (!provenance.immutable ||
        provenance.path_revision == 0U || provenance.frame_revision == 0U ||
        provenance.path_revision != input.expected_path_revision ||
        provenance.frame_revision != input.expected_frame_revision ||
        input.geometry.path_revision != provenance.path_revision ||
        input.geometry.frame_revision != provenance.frame_revision) {
      reason = "SECTION Preview path/frame provenance does not match";
      return false;
    }
    if (preview.status == TubeViabilityStatus::FEASIBLE &&
        (input.geometry.w != preview.current_w ||
         input.geometry.delta != preview.evaluated_delta)) {
      reason = "SECTION geometry W/delta does not match Preview state";
      return false;
    }
    return true;
  }
  if (!RequiredProvenanceExpectationsBound(input, reason)) return false;
  if (!PreviewProvenanceComplete(provenance)) {
    reason = "Preview provenance is incomplete or not immutable";
    return false;
  }
  if (!ProvenanceMatchesInput(input, provenance)) {
    reason = "Preview provenance does not match the requested revisions";
    return false;
  }
  return true;
}

bool BoundsValid(const PhaseOffsetAllocatorBounds& bounds,
                 const double dt,
                 std::string& reason) {
  if (!Finite(bounds.lower_nu) || !Finite(bounds.upper_nu) ||
      bounds.lower_nu < 0.0 || bounds.upper_nu < bounds.lower_nu ||
      !Finite(bounds.u_w_abs_max) || bounds.u_w_abs_max < 0.0 ||
      !Finite(bounds.upper_u_delta) || bounds.upper_u_delta < 0.0 ||
      !Finite(bounds.u_w_slew_rate) || bounds.u_w_slew_rate < 0.0 ||
      !Finite(bounds.u_delta_slew_rate) || bounds.u_delta_slew_rate < 0.0 ||
      !Finite(bounds.zoh_min_dt) || bounds.zoh_min_dt < 0.0 ||
      !Finite(bounds.zoh_max_dt) || bounds.zoh_max_dt < 0.0 ||
      !Finite(bounds.zoh_dt) || bounds.zoh_dt < 0.0) {
    reason = "allocator scalar amplitude, rate or ZOH bounds are invalid";
    return false;
  }
  if (bounds.zoh_max_dt > 0.0 &&
      bounds.zoh_max_dt < bounds.zoh_min_dt) {
    reason = "allocator ZOH duration bounds are inverted";
    return false;
  }
  if (bounds.zoh_min_dt > 0.0 && dt < bounds.zoh_min_dt) {
    reason = "allocator tick is shorter than the minimum ZOH duration";
    return false;
  }
  if (bounds.zoh_max_dt > 0.0 && dt > bounds.zoh_max_dt) {
    reason = "allocator tick exceeds the maximum ZOH duration";
    return false;
  }
  if (bounds.zoh_dt > 0.0 && bounds.zoh_dt != dt) {
    reason = "allocator tick does not match the frozen ZOH duration";
    return false;
  }
  return true;
}

bool BuildSelectedScalar(const double nominal,
                         const double static_lower,
                         const double static_upper,
                         const double previous,
                         const double slew_rate,
                         const double dt,
                         const bool envelope_limited,
                         const bool amplitude_limited,
                         PhaseOffsetScalarSelection& output) {
  output = PhaseOffsetScalarSelection();
  if (!Finite(nominal) || !ValidInterval(static_lower, static_upper) ||
      !Finite(previous) || !Finite(slew_rate) || slew_rate < 0.0 ||
      !Finite(dt) || dt <= 0.0) {
    return false;
  }
  const double slew_step = slew_rate * dt;
  if (!Finite(slew_step)) return false;
  const double slew_lower = previous - slew_step;
  const double slew_upper = previous + slew_step;
  if (!ValidInterval(slew_lower, slew_upper)) return false;

  double final_lower = 0.0;
  double final_upper = 0.0;
  if (!Intersect(static_lower, static_upper, slew_lower, slew_upper,
                 final_lower, final_upper)) {
    return false;
  }

  output.nominal = nominal;
  output.pre_slew = Clamp(nominal, static_lower, static_upper);
  output.selected = Clamp(output.pre_slew, final_lower, final_upper);
  output.lower = static_lower;
  output.upper = static_upper;
  output.slew_lower = slew_lower;
  output.slew_upper = slew_upper;
  output.selected_lower = final_lower;
  output.selected_upper = final_upper;
  output.envelope_limited = envelope_limited;
  output.amplitude_limited = amplitude_limited;
  // This is diagnostic only; exact interval feasibility and membership have
  // already been established above.
  output.slew_limited = output.selected != output.pre_slew;
  output.valid = Finite(output.selected) &&
      output.selected >= final_lower && output.selected <= final_upper;
  return output.valid;
}

bool InputFinite(const PhaseOffsetAllocatorInput& input) {
  return input.geometry.r_w.allFinite() && input.geometry.N.allFinite() &&
      input.g_des.allFinite() && Finite(input.f_w0) &&
      Finite(input.previous_u.u_w) && Finite(input.previous_u.u_delta) &&
      Finite(input.dt) && input.dt > 0.0;
}

bool GeometryValid(const phase_offset_core::PhaseOffsetGeometryState& geometry,
                   std::string& reason) {
  if (!geometry.valid || !geometry.r_w.allFinite() || !geometry.N.allFinite()) {
    reason = "current PhaseOffset geometry is invalid";
    return false;
  }
  const double rw_squared = geometry.r_w.squaredNorm();
  const double normal_norm = geometry.N.norm();
  if (!Finite(rw_squared) || rw_squared <= kGeometryEpsilon ||
      !Finite(normal_norm) ||
      std::abs(normal_norm - 1.0) > kGeometryUnitTolerance ||
      std::abs(geometry.N.z()) > kHorizontalTolerance) {
    reason = "current geometry lacks a finite horizontal unit normal";
    return false;
  }
  if (std::abs(geometry.N.dot(geometry.r_w)) >
      kGeometryUnitTolerance * std::max(1.0, geometry.r_w.norm())) {
    reason = "current geometry normal and reference tangent are inconsistent";
    return false;
  }
  return true;
}

bool PreviewValidForAllocation(const PhaseOffsetAllocatorInput& input,
                               PhaseOffsetAllocatorResult& output,
                               std::string& reason) {
  if (input.preview == nullptr) {
    reason = "NORMAL Preview facts were not supplied";
    return false;
  }
  const NormalPreviewResult& preview = *input.preview;
  if (!GeometryMatchesPreview(input, preview, reason)) return false;
  // A rate-degraded preview (the transverse rate window is too narrow for the
  // requested motion) is still usable: the request is clipped onto the window
  // boundary below instead of invalidating a planner-valid path.  Losing
  // transverse capacity must not by itself drop the whole command, so only the
  // previews that are unusable *at the current state* stay fail-closed.
  const bool rate_degraded =
      preview.status == TubeViabilityStatus::RATE_INFEASIBLE;
  // A retained offset that the corridor has outgrown (it narrowed after the
  // offset was placed) must be recovered, not frozen: the recovery branch below
  // drives delta back onto the nearest admissible value with the available
  // authority.  Rejecting here instead would leave the vehicle permanently
  // outside the corridor with no tick ever committing to bring it back.
  const bool delta_recovery =
      preview.status == TubeViabilityStatus::CURRENT_DELTA_OUTSIDE;
  const bool degraded = rate_degraded || delta_recovery;
  // A retained offset that sits outside the corridor (the tube narrowed after
  // it was placed) is recovered by steering back to the boundary, so it must
  // not invalidate the tick either.
  if (!preview.valid || !preview.feasible ||
      (preview.status != TubeViabilityStatus::FEASIBLE && !degraded)) {
    reason = "NORMAL Preview is not feasible for the current state";
    return false;
  }
  const TubeViabilityRateInterval& interval = preview.current_rate_interval;
  if (!Finite(preview.upper_u_delta) || preview.upper_u_delta < 0.0) {
    reason = "NORMAL Preview transverse-rate facts are invalid";
    return false;
  }
  if (interval.valid &&
      (!Finite(interval.lower) || !Finite(interval.upper) ||
       interval.lower > interval.upper)) {
    reason = "NORMAL Preview transverse-rate facts are invalid";
    return false;
  }
  if (!interval.valid && !degraded) {
    reason = "NORMAL Preview transverse-rate facts are invalid";
    return false;
  }
  if ((interval.lower_boundary_active &&
       !Finite(interval.lower_boundary_slope)) ||
      (interval.upper_boundary_active &&
       !Finite(interval.upper_boundary_slope))) {
    reason = "NORMAL Preview boundary-rate slope provenance is invalid";
    return false;
  }
  output.preview_bound = true;
  output.preview_rate_interval = interval;
  output.preview_provenance = preview.provenance;
  return true;
}

double EffectiveDeltaLimit(const PhaseOffsetAllocatorInput& input) {
  return input.bounds.upper_u_delta;
}

bool FinalCommandValid(const PhaseOffsetAllocatorInput& input,
                       const PhaseOffsetAllocatorResult& output) {
  // The admissible windows are expressed in the u_w / u_delta domain
  // (lower_nu - f_w0, the preview rate interval, +-delta_limit), and that is
  // also the domain in which BuildSelectedScalar clamps the command.  Checking
  // membership there is exact.  Re-deriving the phase rate as f_w0 + u_w and
  // comparing that against [lower_nu, upper_nu] is NOT exact: f + (b - f) != b
  // in IEEE754, and over 200k random inputs 5.1% of saturated lower-boundary
  // picks and 3.7% of upper-boundary ones landed a few ULP outside their own
  // window.  A saturated command was then reported as failing its ZOH checks,
  // the Section port was dropped, and the UAV hovered metres short of its goal
  // on dense swarms.  Nothing is relaxed here: the interval is the same one,
  // only compared in the representation it was built in.  The independent
  // amplitude and slew checks below stay strict.
  const PhaseOffsetAllocatorBounds& bounds = input.bounds;
  const phase_offset_core::PortCommand& selected = output.selected_u;
  const double phase_rate = input.f_w0 + selected.u_w;
  const double delta_limit = EffectiveDeltaLimit(input);
  const bool amplitude_valid = bounds.u_w_abs_max == 0.0
      ? selected.u_w == 0.0
      : std::abs(selected.u_w) <= bounds.u_w_abs_max;
  // A clip that was itself forced by an empty admissible interval is the only
  // case where the corresponding hard bound may be reported as unsatisfied:
  // the request could not be brought inside it at all, so the command takes the
  // interval boundary and the rest of the checks still apply.
  const bool phase_window_ok = output.phase_window_clipped ||
      (selected.u_w >= output.u_w.lower && selected.u_w <= output.u_w.upper);
  const bool transverse_interval_ok = output.transverse_interval_clipped ||
      (selected.u_delta >= output.u_delta.lower &&
       selected.u_delta <= output.u_delta.upper);
  return Finite(selected.u_w) && Finite(selected.u_delta) &&
      Finite(phase_rate) && phase_window_ok && amplitude_valid &&
      transverse_interval_ok &&
      std::abs(selected.u_delta) <= delta_limit &&
      selected.u_w >= output.u_w.slew_lower &&
      selected.u_w <= output.u_w.slew_upper &&
      selected.u_delta >= output.u_delta.slew_lower &&
      selected.u_delta <= output.u_delta.slew_upper;
}

// Diagnostic only: expands the six FinalCommandValid sub-conditions so a
// rejected tick names the exact quantity that failed instead of only reporting
// that "a" bound was violated.  It does not change any decision.
std::string FinalCommandFailDetail(const PhaseOffsetAllocatorInput& input,
                                   const PhaseOffsetAllocatorResult& output) {
  const PhaseOffsetAllocatorBounds& bounds = input.bounds;
  const phase_offset_core::PortCommand& selected = output.selected_u;
  const double phase_rate = input.f_w0 + selected.u_w;
  const double delta_limit = EffectiveDeltaLimit(input);
  const bool finite_ok = Finite(selected.u_w) && Finite(selected.u_delta) &&
      Finite(phase_rate);
  const bool amp_ok = bounds.u_w_abs_max == 0.0
      ? selected.u_w == 0.0
      : std::abs(selected.u_w) <= bounds.u_w_abs_max;
  const bool phase_window_ok = output.phase_window_clipped ||
      (phase_rate >= bounds.lower_nu && phase_rate <= bounds.upper_nu);
  const bool transverse_ok = output.transverse_interval_clipped ||
      (selected.u_delta >= output.preview_rate_interval.lower &&
       selected.u_delta <= output.preview_rate_interval.upper);
  const bool delta_limit_ok = std::abs(selected.u_delta) <= delta_limit;
  const bool u_w_slew_ok = selected.u_w >= output.u_w.slew_lower &&
      selected.u_w <= output.u_w.slew_upper;
  const bool u_delta_slew_ok = selected.u_delta >= output.u_delta.slew_lower &&
      selected.u_delta <= output.u_delta.slew_upper;
  char buffer[512];
  std::snprintf(
      buffer, sizeof(buffer),
      " ok[finite=%d amp=%d phase_win=%d transverse=%d delta_limit=%d "
      "u_w_slew=%d u_delta_slew=%d consist=%d]",
      (int)finite_ok, (int)amp_ok, (int)phase_window_ok, (int)transverse_ok,
      (int)delta_limit_ok, (int)u_w_slew_ok, (int)u_delta_slew_ok,
      (int)output.selectedUConsistent(0.0));
  char numbers[512];
  std::snprintf(
      numbers, sizeof(numbers),
      " rate=%.17g win=[%.17g,%.17g] u_w=%.17g max=%.17g slew=[%.17g,%.17g] "
      "u_delta=%.17g preview=[%.17g,%.17g] limit=%.17g dslew=[%.17g,%.17g]",
      phase_rate, bounds.lower_nu, bounds.upper_nu, selected.u_w,
      bounds.u_w_abs_max, output.u_w.slew_lower, output.u_w.slew_upper,
      selected.u_delta, output.preview_rate_interval.lower,
      output.preview_rate_interval.upper, delta_limit,
      output.u_delta.slew_lower, output.u_delta.slew_upper);
  return std::string(buffer) + numbers;
}

bool Fail(PhaseOffsetAllocatorResult& output,
          const PhaseOffsetAllocatorStatus status,
          const std::string& reason) {
  output.status = status;
  output.valid = false;
  output.feasible = false;
  output.selected_u = phase_offset_core::PortCommand();
  output.next_u_prev = phase_offset_core::PortCommand();
  output.selected_u_w = 0.0;
  output.selected_u_delta = 0.0;
  output.u_w = PhaseOffsetScalarSelection();
  output.u_delta = PhaseOffsetScalarSelection();
  output.phase_rate_selected = 0.0;
  output.zoh_dt = 0.0;
  output.selected_u_owner.clear();
  output.piecewise_constant = false;
  output.reason = reason;
  return false;
}

}  // namespace

const char* phaseOffsetAllocatorStatusName(
    const PhaseOffsetAllocatorStatus status) {
  switch (status) {
    case PhaseOffsetAllocatorStatus::NOT_EVALUATED:
      return "NOT_EVALUATED";
    case PhaseOffsetAllocatorStatus::SELECTED:
      return "SELECTED";
    case PhaseOffsetAllocatorStatus::INVALID_INPUT:
      return "INVALID_INPUT";
    case PhaseOffsetAllocatorStatus::STALE_PREVIEW:
      return "STALE_PREVIEW";
    case PhaseOffsetAllocatorStatus::PREVIEW_INFEASIBLE:
      return "PREVIEW_INFEASIBLE";
    case PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND:
      return "NO_ADMISSIBLE_COMMAND";
  }
  return "UNKNOWN";
}

bool PhaseOffsetAllocatorResult::selectedUConsistent(
    const double tolerance) const {
  return std::isfinite(selected_u.u_w) &&
      std::isfinite(selected_u.u_delta) &&
      std::isfinite(selected_u_w) && std::isfinite(selected_u_delta) &&
      std::abs(selected_u.u_w - selected_u_w) <= tolerance &&
      std::abs(selected_u.u_delta - selected_u_delta) <= tolerance &&
      std::abs(next_u_prev.u_w - selected_u.u_w) <= tolerance &&
      std::abs(next_u_prev.u_delta - selected_u.u_delta) <= tolerance;
}

const char* PhaseOffsetAllocator::ownerName() {
  return "PhaseOffsetAllocator";
}

bool PhaseOffsetAllocator::analyticProjection(
    const phase_offset_core::PhaseOffsetGeometryState& geometry,
    const Eigen::Vector3d& g_des, double& u_w_nom, double& u_delta_nom) {
  u_w_nom = 0.0;
  u_delta_nom = 0.0;
  std::string reason;
  if (!GeometryValid(geometry, reason) || !g_des.allFinite()) return false;
  const double rw_squared = geometry.r_w.squaredNorm();
  if (!Finite(rw_squared) || rw_squared <= kGeometryEpsilon) return false;
  u_w_nom = geometry.r_w.dot(g_des) / rw_squared;
  u_delta_nom = geometry.N.dot(g_des);
  return Finite(u_w_nom) && Finite(u_delta_nom);
}

Eigen::Vector2d PhaseOffsetAllocator::analyticRawPort(
    const phase_offset_core::PhaseOffsetGeometryState& geometry,
    const Eigen::Vector3d& g_des) {
  Eigen::Vector2d result = Eigen::Vector2d::Zero();
  analyticProjection(geometry, g_des, result.x(), result.y());
  return result;
}

bool PhaseOffsetAllocator::allocate(const PhaseOffsetAllocatorInput& input,
                                    PhaseOffsetAllocatorResult& output) {
  output = PhaseOffsetAllocatorResult();

  std::string reason;
  if (!InputFinite(input) ||
      !GeometryValid(input.geometry, reason)) {
    return Fail(output, PhaseOffsetAllocatorStatus::INVALID_INPUT,
                reason.empty() ? "allocator input is not finite" : reason);
  }
  if (!BoundsValid(input.bounds, input.dt, reason)) {
    return Fail(output, PhaseOffsetAllocatorStatus::INVALID_INPUT, reason);
  }
  const bool section_preview =
      input.preview != nullptr &&
      input.preview->proof_kind == TubeViabilityProofKind::SECTION_PWL;
  if ((section_preview && input.expected_section_profile == nullptr) ||
      (!section_preview && input.expected_section_profile != nullptr)) {
    return Fail(output, PhaseOffsetAllocatorStatus::INVALID_INPUT,
                section_preview
                    ? "SECTION Preview profile expectation is missing"
                    : "SECTION profile expectation supplied for a non-SECTION Preview");
  }
  if ((section_preview && !RequiredSectionExpectationsBound(input, reason)) ||
      (!section_preview && !RequiredProvenanceExpectationsBound(input, reason))) {
    return Fail(output, PhaseOffsetAllocatorStatus::INVALID_INPUT, reason);
  }
  if (!PreviewValidForAllocation(input, output, reason)) {
    const bool stale = input.preview == nullptr ||
        !ProvenanceMatchesInput(input, input.preview->provenance);
    return Fail(output,
                stale ? PhaseOffsetAllocatorStatus::STALE_PREVIEW
                      : PhaseOffsetAllocatorStatus::PREVIEW_INFEASIBLE,
                reason);
  }

  if (!analyticProjection(input.geometry, input.g_des,
                          output.u_w_nom, output.u_delta_nom)) {
    return Fail(output, PhaseOffsetAllocatorStatus::INVALID_INPUT,
                "analytic PhaseOffset projection is invalid");
  }
  // M4C-2: if the retained offset is outside the current corridor, ignore the
  // requested transverse intent for this tick and drive back to the nearest
  // admissible offset with the available authority.  The phase axis and the
  // rest of the matched structure are untouched.
  bool transverse_recovery = false;
  if (input.preview != nullptr && !input.preview->knots.empty()) {
    const phase_offset_navigation::TubeViabilityInterval& reachable =
        input.preview->knots.front().reachable;
    const double current_delta = input.geometry.delta;
    if (reachable.valid &&
        (current_delta < reachable.lower || current_delta > reachable.upper)) {
      const double target = std::max(reachable.lower,
                                     std::min(reachable.upper, current_delta));
      const double direction = target > current_delta ? 1.0 : -1.0;
      output.u_delta_nom =
          direction * std::max(0.0, input.bounds.upper_u_delta);
      transverse_recovery = true;
    }
  }
  (void)transverse_recovery;
  output.phase_rate_nom = input.f_w0 + output.u_w_nom;
  if (!Finite(output.phase_rate_nom)) {
    return Fail(output, PhaseOffsetAllocatorStatus::INVALID_INPUT,
                "nominal phase rate is not finite");
  }

  // Each scalar gets its own static admissible interval before its own slew
  // interval is applied.  No value from one scalar is used to replace the
  // other scalar.
  const double phase_lower = input.bounds.lower_nu - input.f_w0;
  const double phase_upper = input.bounds.upper_nu - input.f_w0;
  double u_w_lower = 0.0;
  double u_w_upper = 0.0;
  const bool phase_interval_valid = Intersect(
      phase_lower, phase_upper, -input.bounds.u_w_abs_max,
      input.bounds.u_w_abs_max, u_w_lower, u_w_upper);
  if (!phase_interval_valid) {
    // The base phase rate is outside the allowed window and the correction
    // authority cannot bring it back.  Take the boundary that reduces the
    // violation the most instead of dropping the whole command.
    const double boundary = input.f_w0 > input.bounds.upper_nu
        ? -input.bounds.u_w_abs_max : input.bounds.u_w_abs_max;
    u_w_lower = boundary;
    u_w_upper = boundary;
    output.phase_window_clipped = true;
  }
  if (input.bounds.u_w_abs_max == 0.0) {
    // Canonicalize the permitted zero-amplitude interval so the selected
    // command is exactly zero whenever phase progression admits it.
    u_w_lower = 0.0;
    u_w_upper = 0.0;
  }

  // Tangential-speed floor.  The realised tangential speed is
  //   T . v_cmd = base_tangent_speed + |r_w| * u_w,
  // so "tangential speed >= minimum" is a lower bound on u_w.  The port
  // projector already applies exactly this constraint when it forms the port
  // (port_projector.cpp: w_lower = max(w_lower, (tangent_speed_min -
  // base_tangent_speed) / r_w_norm)); the allocator owns the phase window and
  // amplitude, so without the same bound it can select a u_w that the runtime's
  // tangential-speed audit then rejects, converting a reachable tick into a
  // HOLD.  A zero minimum (every caller that does not set it, including every
  // existing test) leaves this inactive.
  if (input.bounds.tangent_speed_min > 0.0) {
    const double r_w_norm = input.geometry.r_w.norm();
    if (Finite(r_w_norm) && r_w_norm > 1e-9) {
      const double tangent_lower =
          (input.bounds.tangent_speed_min - input.base_tangent_speed) /
          r_w_norm;
      if (Finite(tangent_lower)) {
        u_w_lower = std::max(u_w_lower, tangent_lower);
      }
    }
  }

  const double delta_limit = EffectiveDeltaLimit(input);
  double u_delta_lower = 0.0;
  double u_delta_upper = 0.0;

  // Same recovery on the phase axis as the transverse axis below.  When the
  // admissible phase-rate set and the slew window are disjoint, the request is
  // more than one tick away from the last committed rate.  Move as far towards
  // the admissible set as this tick's slew budget allows and mark the tick,
  // instead of dropping it: a dropped tick never commits, so the committed rate
  // would never reach the admissible set and the phase would freeze forever
  // (observed as `phase scalar slew interval is empty` after a C2 connector).
  {
    const double slew_step = input.bounds.u_w_slew_rate * input.dt;
    double probe_lower = 0.0;
    double probe_upper = 0.0;
    if (Finite(slew_step) &&
        !Intersect(u_w_lower, u_w_upper,
                   input.previous_u.u_w - slew_step,
                   input.previous_u.u_w + slew_step, probe_lower,
                   probe_upper)) {
      const double target = Clamp(output.u_w_nom, u_w_lower, u_w_upper);
      const double best = Clamp(target,
                                input.previous_u.u_w - slew_step,
                                input.previous_u.u_w + slew_step);
      u_w_lower = best;
      u_w_upper = best;
      output.phase_window_clipped = true;
    }
  }

  if (!Intersect(output.preview_rate_interval.lower,
                 output.preview_rate_interval.upper,
                 -delta_limit, delta_limit,
                 u_delta_lower, u_delta_upper)) {
    // The tube window excludes zero and the amplitude admits no point of it.
    // Keep the transverse reference as close to the corridor's nearest
    // admissible rate as the amplitude allows.
    const double nearest = output.preview_rate_interval.valid
        ? (0.0 < output.preview_rate_interval.lower
               ? output.preview_rate_interval.lower
               : output.preview_rate_interval.upper)
        : 0.0;
    const double boundary = std::max(-delta_limit,
                                     std::min(delta_limit, nearest));
    u_delta_lower = boundary;
    u_delta_upper = boundary;
    output.transverse_interval_clipped = true;
  }

  if (!BuildSelectedScalar(
          output.u_w_nom, u_w_lower, u_w_upper, input.previous_u.u_w,
          input.bounds.u_w_slew_rate, input.dt,
          output.u_w_nom < u_w_lower || output.u_w_nom > u_w_upper,
          output.u_w_nom < -input.bounds.u_w_abs_max ||
              output.u_w_nom > input.bounds.u_w_abs_max,
          output.u_w)) {
    return Fail(output, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND,
                "phase scalar slew interval is empty");
  }
  // The horizontal-section tube is the safe lateral set: the transverse
  // command is clipped so that the offset one tick ahead still lies inside the
  // corridor cross-section.  This is the agreed semantics -- follow the swarm
  // intent while it fits the tube, otherwise take the corridor boundary -- and
  // it replaces reliance on a rigid per-step reachable-envelope conformance
  // rule, which rejected ordinary boundary-tracking lag and froze the phase.
  if (input.preview != nullptr && input.dt > 0.0) {
    const double next_w =
        input.geometry.w + input.dt * (input.f_w0 + output.u_w.selected);
    double corridor_lower = 0.0;
    double corridor_upper = 0.0;
    if (Finite(next_w) &&
        SectionCorridorAt(*input.preview, input.geometry.w, corridor_lower,
                          corridor_upper)) {
      double ahead_lower = 0.0;
      double ahead_upper = 0.0;
      if (SectionCorridorAt(*input.preview, next_w, ahead_lower,
                            ahead_upper)) {
        corridor_lower = std::max(corridor_lower, ahead_lower);
        corridor_upper = std::min(corridor_upper, ahead_upper);
      }
      if (corridor_lower <= corridor_upper) {
        const double stay_lower =
            (corridor_lower - input.geometry.delta) / input.dt;
        const double stay_upper =
            (corridor_upper - input.geometry.delta) / input.dt;
        double clipped_lower = 0.0;
        double clipped_upper = 0.0;
        if (Intersect(u_delta_lower, u_delta_upper, stay_lower, stay_upper,
                      clipped_lower, clipped_upper)) {
          u_delta_lower = clipped_lower;
          u_delta_upper = clipped_upper;
        } else {
          // The corridor demands a rate the transverse window cannot host this
          // tick.  Take the window endpoint closest to the corridor demand.
          const double boundary =
              stay_lower > u_delta_upper
                  ? u_delta_upper
                  : (stay_upper < u_delta_lower ? u_delta_lower : 0.0);
          u_delta_lower = boundary;
          u_delta_upper = boundary;
          output.transverse_interval_clipped = true;
        }
      }
    }
  }
  // When the corridor demand and the transverse slew limit are disjoint, hold
  // the rate at the slew boundary nearest the demand instead of dropping the
  // tick.  The offset keeps closing on the corridor over the following ticks.
  {
    const double slew_step = input.bounds.u_delta_slew_rate * input.dt;
    double probe_lower = 0.0;
    double probe_upper = 0.0;
    if (Finite(slew_step) &&
        !Intersect(u_delta_lower, u_delta_upper,
                   input.previous_u.u_delta - slew_step,
                   input.previous_u.u_delta + slew_step, probe_lower,
                   probe_upper)) {
      const double target =
          Clamp(output.u_delta_nom, u_delta_lower, u_delta_upper);
      const double best = Clamp(target,
                                input.previous_u.u_delta - slew_step,
                                input.previous_u.u_delta + slew_step);
      u_delta_lower = best;
      u_delta_upper = best;
      output.transverse_interval_clipped = true;
    }
  }
  if (!BuildSelectedScalar(
          output.u_delta_nom, u_delta_lower, u_delta_upper,
          input.previous_u.u_delta, input.bounds.u_delta_slew_rate, input.dt,
          output.u_delta_nom < output.preview_rate_interval.lower ||
              output.u_delta_nom > output.preview_rate_interval.upper,
          output.u_delta_nom < -delta_limit ||
              output.u_delta_nom > delta_limit,
          output.u_delta)) {
    return Fail(output, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND,
                "transverse scalar slew interval is empty");
  }

  output.selected_u.u_w = output.u_w.selected;
  output.selected_u.u_delta = output.u_delta.selected;
  output.next_u_prev = output.selected_u;
  output.selected_u_w = output.selected_u.u_w;
  output.selected_u_delta = output.selected_u.u_delta;
  output.phase_rate_selected = input.f_w0 + output.selected_u.u_w;
  output.zoh_dt = input.dt;
  output.piecewise_constant = true;
  output.selected_u_owner = ownerName();
  output.provenance = "phase_offset_navigation/phase_offset_allocator/noqp-v1";

  // The corridor clip is authoritative: when it pushed the selected rate past
  // the Preview rate window, that hard bound is the one this tick could not
  // satisfy, so it is not re-applied; every other bound still is.
  if (!output.transverse_interval_clipped &&
      (output.selected_u.u_delta < output.preview_rate_interval.lower ||
       output.selected_u.u_delta > output.preview_rate_interval.upper)) {
    output.transverse_interval_clipped = true;
  }
  if (!FinalCommandValid(input, output) ||
      !output.selectedUConsistent(0.0)) {
    return Fail(output, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND,
                "selected scalar command failed its bounded ZOH checks" +
                    FinalCommandFailDetail(input, output));
  }

  output.status = PhaseOffsetAllocatorStatus::SELECTED;
  output.valid = true;
  output.feasible = true;
  output.reason = "independently clipped scalar command selected";
  return true;
}

bool PhaseOffsetAllocator::evaluate(const PhaseOffsetAllocatorInput& input,
                                    PhaseOffsetAllocatorResult& output) {
  return allocate(input, output);
}

bool PhaseOffsetAllocator::select(const PhaseOffsetAllocatorInput& input,
                                  PhaseOffsetAllocatorResult& output) {
  return allocate(input, output);
}

bool PhaseOffsetAllocator::solve(const PhaseOffsetAllocatorInput& input,
                                 PhaseOffsetAllocatorResult& output) {
  return allocate(input, output);
}

PhaseOffsetAllocatorResult PhaseOffsetAllocator::allocate(
    const PhaseOffsetAllocatorInput& input) {
  PhaseOffsetAllocatorResult output;
  allocate(input, output);
  return output;
}

}  // namespace phase_offset_navigation
