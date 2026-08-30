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
  if (!preview.valid || !preview.feasible ||
      preview.status != TubeViabilityStatus::FEASIBLE ||
      !preview.current_delta_inside ||
      !preview.current_rate_interval.valid) {
    reason = "NORMAL Preview is not feasible for the current state";
    return false;
  }
  const TubeViabilityRateInterval& interval = preview.current_rate_interval;
  if (!Finite(interval.lower) || !Finite(interval.upper) ||
      interval.lower > interval.upper ||
      !Finite(preview.upper_u_delta) || preview.upper_u_delta < 0.0) {
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
  const PhaseOffsetAllocatorBounds& bounds = input.bounds;
  const phase_offset_core::PortCommand& selected = output.selected_u;
  const double phase_rate = input.f_w0 + selected.u_w;
  const double delta_limit = EffectiveDeltaLimit(input);
  const bool amplitude_valid = bounds.u_w_abs_max == 0.0
      ? selected.u_w == 0.0
      : std::abs(selected.u_w) <= bounds.u_w_abs_max;
  return Finite(selected.u_w) && Finite(selected.u_delta) &&
      Finite(phase_rate) && phase_rate >= bounds.lower_nu &&
      phase_rate <= bounds.upper_nu &&
      amplitude_valid &&
      selected.u_delta >= output.preview_rate_interval.lower &&
      selected.u_delta <= output.preview_rate_interval.upper &&
      std::abs(selected.u_delta) <= delta_limit &&
      selected.u_w >= output.u_w.slew_lower &&
      selected.u_w <= output.u_w.slew_upper &&
      selected.u_delta >= output.u_delta.slew_lower &&
      selected.u_delta <= output.u_delta.slew_upper;
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
  if (!RequiredProvenanceExpectationsBound(input, reason)) {
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
    return Fail(output, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND,
                "phase scalar admissible interval is empty");
  }
  if (input.bounds.u_w_abs_max == 0.0) {
    // Canonicalize the permitted zero-amplitude interval so the selected
    // command is exactly zero whenever phase progression admits it.
    u_w_lower = 0.0;
    u_w_upper = 0.0;
  }

  const double delta_limit = EffectiveDeltaLimit(input);
  double u_delta_lower = 0.0;
  double u_delta_upper = 0.0;
  if (!Intersect(output.preview_rate_interval.lower,
                 output.preview_rate_interval.upper,
                 -delta_limit, delta_limit,
                 u_delta_lower, u_delta_upper)) {
    return Fail(output, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND,
                "transverse scalar admissible interval is empty");
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

  if (!FinalCommandValid(input, output) ||
      !output.selectedUConsistent(0.0)) {
    return Fail(output, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND,
                "selected scalar command failed its bounded ZOH checks");
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
