#include "phase_offset_core/port_projector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace phase_offset_core {
namespace {

constexpr double kBoundTolerance = 1e-12;

struct HalfPlane {
  double a_w = 0.0;
  double a_delta = 0.0;
  double b = 0.0;  // a_w * u_w + a_delta * u_delta >= b.
  const char* label = "";
};

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool IsFinite(const PortCommand& value) {
  return IsFinite(value.u_w) && IsFinite(value.u_delta);
}

bool HasValidLimits(const PortProjectionLimits& limits) {
  return IsFinite(limits.u_w_abs_max) && limits.u_w_abs_max >= 0.0 &&
         IsFinite(limits.u_delta_abs_max) && limits.u_delta_abs_max >= 0.0 &&
         IsFinite(limits.u_w_rate_max) && limits.u_w_rate_max >= 0.0 &&
         IsFinite(limits.u_delta_rate_max) && limits.u_delta_rate_max >= 0.0 &&
         // A normal rolling certificate keeps both margins strictly positive.
         // The same projector is also the single constraint kernel used by
         // safety-priority mode, where proposal §14.1 permits both lower
         // margins to be exactly zero but never negative.
         IsFinite(limits.phase_dot_min) && limits.phase_dot_min >= 0.0 &&
         IsFinite(limits.tangent_speed_min) && limits.tangent_speed_min >= 0.0 &&
         IsFinite(limits.regularity_margin) && limits.regularity_margin > 0.0;
}

bool HasValidConstraint(const OffsetConstraint& constraint) {
  if (!constraint.enabled) return true;
  return constraint.valid && IsFinite(constraint.lower) &&
         IsFinite(constraint.upper) && constraint.lower <= constraint.upper &&
         IsFinite(constraint.lower_w) && IsFinite(constraint.upper_w) &&
         IsFinite(constraint.invariant_gain) &&
         constraint.invariant_gain >= 0.0 &&
         IsFinite(constraint.interior_margin) &&
         constraint.interior_margin >= 0.0;
}

bool IsFinite(const PortProjectionPwlCell& cell) {
  return IsFinite(cell.w0) && IsFinite(cell.w1) &&
      IsFinite(cell.lower_w) && IsFinite(cell.lower_at_w0) &&
      IsFinite(cell.upper_w) && IsFinite(cell.upper_at_w0);
}

double LowerAt(const PortProjectionPwlCell& cell, const double w) {
  return cell.lower_at_w0 + cell.lower_w * (w - cell.w0);
}

double UpperAt(const PortProjectionPwlCell& cell, const double w) {
  return cell.upper_at_w0 + cell.upper_w * (w - cell.w0);
}

bool HasValidPwlCells(const std::vector<PortProjectionPwlCell>& cells,
                      const double current_w) {
  if (cells.empty() || !IsFinite(current_w)) return false;
  for (std::size_t index = 0U; index < cells.size(); ++index) {
    const PortProjectionPwlCell& cell = cells[index];
    if (!IsFinite(cell) || cell.w1 <= cell.w0 + kBoundTolerance ||
        LowerAt(cell, cell.w0) > UpperAt(cell, cell.w0) + kBoundTolerance ||
        LowerAt(cell, cell.w1) > UpperAt(cell, cell.w1) + kBoundTolerance) {
      return false;
    }
    if (index > 0U) {
      const PortProjectionPwlCell& previous = cells[index - 1U];
      if (std::abs(previous.w1 - cell.w0) > kBoundTolerance ||
          std::abs(LowerAt(previous, previous.w1) - LowerAt(cell, cell.w0)) >
              kBoundTolerance ||
          std::abs(UpperAt(previous, previous.w1) - UpperAt(cell, cell.w0)) >
              kBoundTolerance) {
        return false;
      }
    }
  }
  return current_w >= cells.front().w0 - kBoundTolerance &&
      current_w <= cells.back().w1 + kBoundTolerance;
}

bool PwlContains(const std::vector<PortProjectionPwlCell>& cells,
                 const double current_w,
                 const double dt,
                 const double delta,
                 const double interior_margin,
                 const PortCommand& port,
                 const double base_w_dot) {
  const double v = base_w_dot + port.u_w;
  const double next_w = current_w + dt * v;
  const double next_delta = delta + dt * port.u_delta;
  if (!IsFinite(v) || !IsFinite(next_w) || !IsFinite(next_delta) ||
      v < -kBoundTolerance) {
    return false;
  }
  std::size_t terminal = cells.size();
  for (std::size_t index = 0U; index < cells.size(); ++index) {
    // A shared terminal knot is assigned to its right/forward cell, except at
    // the last domain endpoint, matching FindCurrentPwlCell and query().
    if (next_w >= cells[index].w0 - kBoundTolerance &&
        (next_w < cells[index].w1 - kBoundTolerance ||
         (index + 1U == cells.size() &&
          next_w <= cells[index].w1 + kBoundTolerance))) {
      terminal = index;
      break;
    }
  }
  if (terminal == cells.size()) return false;
  const PortProjectionPwlCell& terminal_cell = cells[terminal];
  const double terminal_lower = LowerAt(terminal_cell, next_w) + interior_margin;
  const double terminal_upper = UpperAt(terminal_cell, next_w) - interior_margin;
  if (!IsFinite(terminal_lower) || !IsFinite(terminal_upper) ||
      next_delta < terminal_lower - kBoundTolerance ||
      next_delta > terminal_upper + kBoundTolerance) {
    return false;
  }
  // With nonnegative phase progress, both w(t) and delta(t) are affine on
  // every traversed PWL cell.  Checking each crossed knot and the terminal
  // endpoint therefore proves containment on the complete held-command path.
  for (std::size_t index = 0U; index < terminal; ++index) {
    const double knot_w = cells[index].w1;
    if (knot_w <= current_w + kBoundTolerance) continue;
    const double elapsed = (knot_w - current_w) / v;
    if (!IsFinite(elapsed) || elapsed < -kBoundTolerance ||
        elapsed > dt + kBoundTolerance) {
      return false;
    }
    const double knot_delta = delta + elapsed * port.u_delta;
    const double lower = LowerAt(cells[index], knot_w) + interior_margin;
    const double upper = UpperAt(cells[index], knot_w) - interior_margin;
    if (!IsFinite(knot_delta) || knot_delta < lower - kBoundTolerance ||
        knot_delta > upper + kBoundTolerance) {
      return false;
    }
  }
  return true;
}

void SetFailureStage(PortProjectionResult& output,
                     const PortProjectionFailureStage stage) {
  output.diagnostics.failure_stage = stage;
}

void EnsureFinite(PortProjectionResult& output) {
  if (!IsFinite(output.final_port)) output.final_port = PortCommand();
  if (!IsFinite(output.final_w_dot)) output.final_w_dot = 0.0;
  if (!IsFinite(output.final_tangent_speed)) output.final_tangent_speed = 0.0;
  if (!IsFinite(output.next_delta)) output.next_delta = 0.0;
  if (!IsFinite(output.next_regularity)) output.next_regularity = 0.0;
  if (!IsFinite(output.upper_invariant_residual)) {
    output.upper_invariant_residual = 0.0;
  }
  if (!IsFinite(output.lower_invariant_residual)) {
    output.lower_invariant_residual = 0.0;
  }
}

bool Invalidate(PortProjectionResult& output, std::string reason) {
  EnsureFinite(output);
  output.valid = false;
  output.invalid_reason = std::move(reason);
  return false;
}

bool Invalidate(PortProjectionResult& output,
                const PortProjectionFailureStage stage,
                std::string reason) {
  SetFailureStage(output, stage);
  return Invalidate(output, std::move(reason));
}

bool IsEmpty(const double lower, const double upper) {
  return lower > upper + kBoundTolerance;
}

double Clamp(const double value, const double lower, const double upper) {
  return std::max(lower, std::min(value, upper));
}

std::string LimitReason(const double raw,
                        const double amplitude,
                        const double rate_lower,
                        const double rate_upper,
                        const double feasible_lower,
                        const double feasible_upper,
                        const char* directional_reason) {
  std::string reason;
  if (raw < -amplitude - kBoundTolerance || raw > amplitude + kBoundTolerance) {
    reason = "amplitude";
  }
  const double amplitude_limited = Clamp(raw, -amplitude, amplitude);
  if (amplitude_limited < rate_lower - kBoundTolerance ||
      amplitude_limited > rate_upper + kBoundTolerance) {
    reason += reason.empty() ? "rate" : "+rate";
  }
  if (amplitude_limited < feasible_lower - kBoundTolerance ||
      amplitude_limited > feasible_upper + kBoundTolerance) {
    reason += reason.empty() ? directional_reason
                             : std::string("+") + directional_reason;
  }
  return reason.empty() ? "projection" : reason;
}

bool ProjectA4Compatible(const PortProjectionInput& input,
                         const PortProjectionLimits& limits,
                         PortProjectionResult& output) {
  const double current_regularity = 1.0 - input.curvature * input.delta;
  if (!IsFinite(current_regularity)) {
    return Invalidate(output, "current regularity is not finite");
  }
  if (current_regularity < limits.regularity_margin) {
    return Invalidate(output, "current regularity margin is violated");
  }

  const double w_rate_delta = limits.u_w_rate_max * input.dt;
  const double w_rate_lower = input.previous_final.u_w - w_rate_delta;
  const double w_rate_upper = input.previous_final.u_w + w_rate_delta;
  double w_lower = std::max(-limits.u_w_abs_max, w_rate_lower);
  double w_upper = std::min(limits.u_w_abs_max, w_rate_upper);
  w_lower = std::max(w_lower, limits.phase_dot_min - input.base_w_dot);
  w_lower = std::max(
      w_lower,
      (limits.tangent_speed_min - input.base_tangent_speed) / input.r_w_norm);
  if (IsEmpty(w_lower, w_upper)) {
    return Invalidate(output, "u_w feasible interval is empty");
  }
  output.diagnostics.u_w_interval = {w_lower, w_upper, true};
  const double desired_w = Clamp(
      input.raw.u_w, -limits.u_w_abs_max, limits.u_w_abs_max);
  output.final_port.u_w = Clamp(desired_w, w_lower, w_upper);

  const double delta_rate_delta = limits.u_delta_rate_max * input.dt;
  const double delta_rate_lower = input.previous_final.u_delta - delta_rate_delta;
  const double delta_rate_upper = input.previous_final.u_delta + delta_rate_delta;
  double lateral_lower = std::max(-limits.u_delta_abs_max, delta_rate_lower);
  double lateral_upper = std::min(limits.u_delta_abs_max, delta_rate_upper);
  if (input.curvature > 0.0) {
    const double upper_delta =
        ((1.0 - limits.regularity_margin) / input.curvature - input.delta) /
        input.dt;
    lateral_upper = std::min(lateral_upper, upper_delta);
  } else if (input.curvature < 0.0) {
    const double lower_delta =
        ((1.0 - limits.regularity_margin) / input.curvature - input.delta) /
        input.dt;
    lateral_lower = std::max(lateral_lower, lower_delta);
  }
  if (IsEmpty(lateral_lower, lateral_upper)) {
    return Invalidate(output, "u_delta feasible interval is empty");
  }
  output.diagnostics.u_delta_interval = {lateral_lower, lateral_upper, true};
  const double desired_delta = Clamp(
      input.raw.u_delta, -limits.u_delta_abs_max, limits.u_delta_abs_max);
  output.final_port.u_delta = Clamp(desired_delta, lateral_lower, lateral_upper);

  output.final_w_dot = input.base_w_dot + output.final_port.u_w;
  output.final_tangent_speed = input.base_tangent_speed +
      input.r_w_norm * output.final_port.u_w;
  output.next_delta = input.delta + input.dt * output.final_port.u_delta;
  output.next_regularity = 1.0 - input.curvature * output.next_delta;
  if (!IsFinite(output.final_w_dot) || !IsFinite(output.final_tangent_speed) ||
      !IsFinite(output.next_delta) || !IsFinite(output.next_regularity) ||
      output.final_w_dot < limits.phase_dot_min - kBoundTolerance ||
      output.final_tangent_speed < limits.tangent_speed_min - kBoundTolerance ||
      output.next_regularity < limits.regularity_margin - kBoundTolerance) {
    return Invalidate(output, "projected port violates a local constraint");
  }

  output.u_w_limited =
      std::abs(output.final_port.u_w - input.raw.u_w) > kBoundTolerance;
  output.u_delta_limited =
      std::abs(output.final_port.u_delta - input.raw.u_delta) > kBoundTolerance;
  if (output.u_w_limited) {
    output.u_w_limit_reason = LimitReason(
        input.raw.u_w, limits.u_w_abs_max, w_rate_lower, w_rate_upper,
        w_lower, w_upper, "positive_margin");
  }
  if (output.u_delta_limited) {
    output.u_delta_limit_reason = LimitReason(
        input.raw.u_delta, limits.u_delta_abs_max,
        delta_rate_lower, delta_rate_upper, lateral_lower, lateral_upper,
        "regularity");
  }
  output.valid = true;
  output.invalid_reason.clear();
  return true;
}

bool Satisfies(const PortCommand& port, const std::vector<HalfPlane>& planes) {
  for (const HalfPlane& plane : planes) {
    const double residual = plane.a_w * port.u_w +
        plane.a_delta * port.u_delta - plane.b;
    if (!IsFinite(residual) || residual < -kBoundTolerance) return false;
  }
  return true;
}

void AddCandidate(const PortCommand& candidate,
                  const std::vector<HalfPlane>& planes,
                  const PortCommand& raw,
                  bool& found,
                  double& best_cost,
                  PortCommand& best) {
  if (!IsFinite(candidate) || !Satisfies(candidate, planes)) return;
  const double dw = candidate.u_w - raw.u_w;
  const double dd = candidate.u_delta - raw.u_delta;
  const double cost = dw * dw + dd * dd;
  if (!IsFinite(cost)) return;
  const bool strictly_better = cost < best_cost - kBoundTolerance;
  const bool tied_but_lexicographically_first =
      std::abs(cost - best_cost) <= kBoundTolerance &&
      (candidate.u_w < best.u_w - kBoundTolerance ||
       (std::abs(candidate.u_w - best.u_w) <= kBoundTolerance &&
        candidate.u_delta < best.u_delta - kBoundTolerance));
  if (!found || strictly_better || tied_but_lexicographically_first) {
    found = true;
    best_cost = cost;
    best = candidate;
  }
}

bool BoundaryProjection(const PortCommand& raw,
                        const HalfPlane& plane,
                        PortCommand& candidate) {
  const double norm_squared = plane.a_w * plane.a_w +
      plane.a_delta * plane.a_delta;
  if (!IsFinite(norm_squared) || norm_squared <= kBoundTolerance) return false;
  const double residual = plane.b -
      plane.a_w * raw.u_w - plane.a_delta * raw.u_delta;
  candidate.u_w = raw.u_w + plane.a_w * residual / norm_squared;
  candidate.u_delta = raw.u_delta + plane.a_delta * residual / norm_squared;
  return IsFinite(candidate);
}

bool BoundaryIntersection(const HalfPlane& first,
                          const HalfPlane& second,
                          PortCommand& candidate) {
  const double determinant = first.a_w * second.a_delta -
      first.a_delta * second.a_w;
  if (!IsFinite(determinant) || std::abs(determinant) <= kBoundTolerance) {
    return false;
  }
  candidate.u_w = (first.b * second.a_delta - first.a_delta * second.b) /
      determinant;
  candidate.u_delta = (first.a_w * second.b - first.b * second.a_w) /
      determinant;
  return IsFinite(candidate);
}

void AddBoxPlanes(const double lower, const double upper,
                  const char* lower_label, const char* upper_label,
                  const bool is_w, std::vector<HalfPlane>& planes) {
  HalfPlane lower_plane;
  lower_plane.a_w = is_w ? 1.0 : 0.0;
  lower_plane.a_delta = is_w ? 0.0 : 1.0;
  lower_plane.b = lower;
  lower_plane.label = lower_label;
  planes.push_back(lower_plane);
  HalfPlane upper_plane;
  upper_plane.a_w = is_w ? -1.0 : 0.0;
  upper_plane.a_delta = is_w ? 0.0 : -1.0;
  upper_plane.b = -upper;
  upper_plane.label = upper_label;
  planes.push_back(upper_plane);
}

bool LabelIsActive(const PortCommand& port,
                   const std::vector<HalfPlane>& planes,
                   const char* label) {
  for (const HalfPlane& plane : planes) {
    if (std::string(plane.label) != label) continue;
    const double residual = plane.a_w * port.u_w +
        plane.a_delta * port.u_delta - plane.b;
    if (std::abs(residual) <= 1e-9) return true;
  }
  return false;
}

void AddUniqueVertex(const PortCommand& candidate,
                     std::vector<PortCommand>& vertices) {
  if (!IsFinite(candidate)) return;
  for (const PortCommand& existing : vertices) {
    if (std::abs(existing.u_w - candidate.u_w) <= kBoundTolerance &&
        std::abs(existing.u_delta - candidate.u_delta) <= kBoundTolerance) {
      return;
    }
  }
  vertices.push_back(candidate);
}

bool LexicographicallyLess(const PortCommand& first, const PortCommand& second) {
  return first.u_w < second.u_w - kBoundTolerance ||
      (std::abs(first.u_w - second.u_w) <= kBoundTolerance &&
       first.u_delta < second.u_delta - kBoundTolerance);
}

void RotateLexicographicallyFirst(std::vector<PortCommand>& vertices) {
  if (vertices.empty()) return;
  std::size_t first = 0U;
  for (std::size_t index = 1U; index < vertices.size(); ++index) {
    if (LexicographicallyLess(vertices[index], vertices[first])) first = index;
  }
  std::rotate(vertices.begin(), vertices.begin() + first, vertices.end());
}

std::string EdgeLabel(const PortCommand& first, const PortCommand& second,
                      const std::vector<HalfPlane>& planes) {
  std::string label;
  for (const HalfPlane& plane : planes) {
    const double first_residual = plane.a_w * first.u_w +
        plane.a_delta * first.u_delta - plane.b;
    const double second_residual = plane.a_w * second.u_w +
        plane.a_delta * second.u_delta - plane.b;
    if (std::abs(first_residual) > 1e-8 || std::abs(second_residual) > 1e-8) {
      continue;
    }
    if (label.empty() || std::string(plane.label) < label) label = plane.label;
  }
  return label;
}

bool BuildFeasiblePolygon(const std::vector<HalfPlane>& planes,
                          PortProjectionDiagnostics& diagnostics) {
  diagnostics.feasible_polygon_vertices.clear();
  diagnostics.feasible_polygon_edges.clear();
  diagnostics.core_polygon_nonempty = false;
  std::vector<PortCommand>& vertices = diagnostics.feasible_polygon_vertices;
  for (std::size_t first = 0U; first < planes.size(); ++first) {
    for (std::size_t second = first + 1U; second < planes.size(); ++second) {
      PortCommand candidate;
      if (BoundaryIntersection(planes[first], planes[second], candidate) &&
          Satisfies(candidate, planes)) {
        AddUniqueVertex(candidate, vertices);
      }
    }
  }
  if (vertices.empty()) return false;
  if (vertices.size() > 2U) {
    PortCommand center;
    for (const PortCommand& vertex : vertices) {
      center.u_w += vertex.u_w;
      center.u_delta += vertex.u_delta;
    }
    center.u_w /= static_cast<double>(vertices.size());
    center.u_delta /= static_cast<double>(vertices.size());
    std::sort(vertices.begin(), vertices.end(), [&center](const PortCommand& first,
                                                            const PortCommand& second) {
      const double first_angle = std::atan2(first.u_delta - center.u_delta,
                                            first.u_w - center.u_w);
      const double second_angle = std::atan2(second.u_delta - center.u_delta,
                                             second.u_w - center.u_w);
      if (std::abs(first_angle - second_angle) <= kBoundTolerance) {
        return LexicographicallyLess(first, second);
      }
      return first_angle < second_angle;
    });
  } else {
    std::sort(vertices.begin(), vertices.end(), LexicographicallyLess);
  }
  RotateLexicographicallyFirst(vertices);
  if (vertices.size() == 2U) {
    PortProjectionPolygonEdge edge;
    edge.first = vertices[0U];
    edge.second = vertices[1U];
    edge.active_constraint = EdgeLabel(edge.first, edge.second, planes);
    diagnostics.feasible_polygon_edges.push_back(edge);
  } else if (vertices.size() > 2U) {
    for (std::size_t index = 0U; index < vertices.size(); ++index) {
      PortProjectionPolygonEdge edge;
      edge.first = vertices[index];
      edge.second = vertices[(index + 1U) % vertices.size()];
      edge.active_constraint = EdgeLabel(edge.first, edge.second, planes);
      diagnostics.feasible_polygon_edges.push_back(edge);
    }
  }
  diagnostics.core_polygon_nonempty = true;
  return true;
}

void PopulateResiduals(const PortCommand& port,
                       const std::vector<HalfPlane>& planes,
                       PortProjectionDiagnostics& diagnostics) {
  diagnostics.constraint_residuals.clear();
  for (const HalfPlane& plane : planes) {
    PortProjectionConstraintResidual residual;
    residual.label = plane.label;
    residual.residual = plane.a_w * port.u_w + plane.a_delta * port.u_delta -
        plane.b;
    if (!IsFinite(residual.residual)) residual.residual = 0.0;
    diagnostics.constraint_residuals.push_back(residual);
  }
}

void PopulateConflicts(const PortCommand& seed,
                       const std::vector<HalfPlane>& planes,
                       PortProjectionDiagnostics& diagnostics) {
  diagnostics.conflicting_constraints.clear();
  for (const HalfPlane& plane : planes) {
    const double residual = plane.a_w * seed.u_w + plane.a_delta * seed.u_delta -
        plane.b;
    if (!IsFinite(residual) || residual < -kBoundTolerance) {
      const std::string label = plane.label;
      if (std::find(diagnostics.conflicting_constraints.begin(),
                    diagnostics.conflicting_constraints.end(), label) ==
          diagnostics.conflicting_constraints.end()) {
        diagnostics.conflicting_constraints.push_back(label);
      }
    }
  }
}

bool FindCurrentPwlCell(const std::vector<PortProjectionPwlCell>& cells,
                        const double current_w,
                        std::size_t& current_cell) {
  current_cell = cells.size();
  for (std::size_t index = 0U; index < cells.size(); ++index) {
    const PortProjectionPwlCell& cell = cells[index];
    // A shared knot belongs to its forward/right cell, matching the
    // non-reversing phase convention and TubeFilter::query().
    if (current_w >= cell.w0 - kBoundTolerance &&
        (current_w < cell.w1 - kBoundTolerance ||
         (index + 1U == cells.size() &&
          current_w <= cell.w1 + kBoundTolerance))) {
      current_cell = index;
      return true;
    }
  }
  return false;
}

void AddNearestCandidates(const std::vector<HalfPlane>& planes,
                          const PortCommand& raw,
                          bool& found,
                          double& best_cost,
                          PortCommand& best) {
  AddCandidate(raw, planes, raw, found, best_cost, best);
  for (const HalfPlane& plane : planes) {
    PortCommand candidate;
    if (BoundaryProjection(raw, plane, candidate)) {
      AddCandidate(candidate, planes, raw, found, best_cost, best);
    }
  }
  for (std::size_t first = 0U; first < planes.size(); ++first) {
    for (std::size_t second = first + 1U; second < planes.size(); ++second) {
      PortCommand candidate;
      if (BoundaryIntersection(planes[first], planes[second], candidate)) {
        AddCandidate(candidate, planes, raw, found, best_cost, best);
      }
    }
  }
}

}  // namespace

bool PortProjector::project(const PortProjectionInput& input,
                            const PortProjectionLimits& limits,
                            PortProjectionResult& output) {
  output = PortProjectionResult();
  if (!HasValidLimits(limits)) {
    return Invalidate(output, "projection limits are invalid");
  }
  if (!IsFinite(input.raw) || !IsFinite(input.previous_final) ||
      !IsFinite(input.dt) || input.dt <= 0.0 || !IsFinite(input.delta) ||
      !IsFinite(input.curvature) || !IsFinite(input.r_w_norm) ||
      input.r_w_norm <= 0.0 || !IsFinite(input.base_w_dot) ||
      !IsFinite(input.base_tangent_speed) ||
      (!input.offset_pwl_cells.empty() && !IsFinite(input.phase))) {
    return Invalidate(output, "projection input is invalid");
  }
  if (!HasValidConstraint(input.offset_constraint)) {
    return Invalidate(output, "offset constraint is invalid");
  }
  if (!input.offset_pwl_cells.empty() &&
      !HasValidPwlCells(input.offset_pwl_cells, input.phase)) {
    return Invalidate(output, "piecewise-linear offset cells are invalid");
  }

  // The disabled path is intentionally the exact A4 scalar-clamp algorithm.
  if (!input.offset_constraint.enabled) {
    return ProjectA4Compatible(input, limits, output);
  }

  const double current_regularity = 1.0 - input.curvature * input.delta;
  if (!IsFinite(current_regularity)) {
    return Invalidate(output, "current regularity is not finite");
  }
  if (current_regularity < limits.regularity_margin) {
    return Invalidate(output, "current regularity margin is violated");
  }

  const OffsetConstraint& constraint = input.offset_constraint;
  const double upper_h = constraint.upper - constraint.interior_margin -
      input.delta;
  const double lower_h = input.delta - constraint.lower -
      constraint.interior_margin;
  if (!IsFinite(upper_h) || !IsFinite(lower_h) || upper_h < -kBoundTolerance ||
      lower_h < -kBoundTolerance) {
    return Invalidate(output, "current offset constraint margin is violated");
  }

  const double w_rate_delta = limits.u_w_rate_max * input.dt;
  double w_lower = std::max(-limits.u_w_abs_max,
                            input.previous_final.u_w - w_rate_delta);
  const double w_upper = std::min(limits.u_w_abs_max,
                                  input.previous_final.u_w + w_rate_delta);
  w_lower = std::max(w_lower, limits.phase_dot_min - input.base_w_dot);
  w_lower = std::max(w_lower, (limits.tangent_speed_min -
      input.base_tangent_speed) / input.r_w_norm);

  const double delta_rate_delta = limits.u_delta_rate_max * input.dt;
  double delta_lower = std::max(-limits.u_delta_abs_max,
      input.previous_final.u_delta - delta_rate_delta);
  double delta_upper = std::min(limits.u_delta_abs_max,
      input.previous_final.u_delta + delta_rate_delta);
  if (input.curvature > 0.0) {
    delta_upper = std::min(delta_upper,
        ((1.0 - limits.regularity_margin) / input.curvature - input.delta) /
        input.dt);
  } else if (input.curvature < 0.0) {
    delta_lower = std::max(delta_lower,
        ((1.0 - limits.regularity_margin) / input.curvature - input.delta) /
        input.dt);
  }
  if (IsEmpty(w_lower, w_upper) || IsEmpty(delta_lower, delta_upper)) {
    return Invalidate(output, "local port feasible interval is empty");
  }
  output.diagnostics.u_w_interval = {w_lower, w_upper, true};
  output.diagnostics.u_delta_interval = {delta_lower, delta_upper, true};

  std::vector<HalfPlane> planes;
  const bool exact_pwl = !input.offset_pwl_cells.empty();
  AddBoxPlanes(w_lower, w_upper, "local_w_lower", "local_w_upper", true,
               planes);
  AddBoxPlanes(delta_lower, delta_upper, "local_delta_lower",
               "local_delta_upper", false, planes);
  // Without a PWL profile retain the legacy affine current-tangent exact-next
  // behavior.  With PWL cells, terminal and crossed-knot constraints below
  // replace this extrapolation rather than intersecting it as an extra gate.
  if (!exact_pwl) {
    HalfPlane exact_next_upper;
    exact_next_upper.a_w = constraint.upper_w;
    exact_next_upper.a_delta = -1.0;
    exact_next_upper.b = -constraint.upper_w * input.base_w_dot -
        upper_h / input.dt;
    exact_next_upper.label = "exact_next_upper";
    planes.push_back(exact_next_upper);
    HalfPlane exact_next_lower;
    exact_next_lower.a_w = -constraint.lower_w;
    exact_next_lower.a_delta = 1.0;
    exact_next_lower.b = constraint.lower_w * input.base_w_dot -
        lower_h / input.dt;
    exact_next_lower.label = "exact_next_lower";
    planes.push_back(exact_next_lower);
  }
  HalfPlane upper_invariant;
  upper_invariant.a_w = constraint.upper_w;
  upper_invariant.a_delta = -1.0;
  upper_invariant.b = -constraint.upper_w * input.base_w_dot -
      constraint.invariant_gain * upper_h;
  upper_invariant.label = "upper_invariant";
  HalfPlane lower_invariant;
  lower_invariant.a_w = -constraint.lower_w;
  lower_invariant.a_delta = 1.0;
  lower_invariant.b = constraint.lower_w * input.base_w_dot -
      constraint.invariant_gain * lower_h;
  lower_invariant.label = "lower_invariant";

  // Exact-PWL terminal/crossed-knot containment proves the complete held
  // step directly.  The continuous invariant planes are therefore retained
  // as residual diagnostics only for this path; adding them here would make
  // an otherwise exact feasible polygon spuriously empty.  Non-PWL callers
  // retain their historical invariant contract unchanged.
  if (!exact_pwl) {
    planes.push_back(upper_invariant);
    planes.push_back(lower_invariant);
  }

  std::size_t current_pwl_cell = input.offset_pwl_cells.size();
  if (exact_pwl && !FindCurrentPwlCell(input.offset_pwl_cells, input.phase,
                                       current_pwl_cell)) {
    return Invalidate(output, "current phase is outside piecewise-linear cells");
  }

  // Exact PWL terminal/crossed-knot constraints are added per possible
  // terminal cell below, rather than extrapolating the current tangent across
  // a knot or intersecting it with a diagnostic invariant plane.
  const std::vector<HalfPlane> base_planes = planes;

  PortCommand best;
  bool found = false;
  double best_cost = std::numeric_limits<double>::infinity();
  std::vector<HalfPlane> winning_planes;
  PortProjectionDiagnostics aggregate_diagnostics;
  if (!exact_pwl) {
    AddNearestCandidates(planes, input.raw, found, best_cost, best);
    if (found) {
      winning_planes = planes;
      BuildFeasiblePolygon(planes, aggregate_diagnostics);
    }
  } else {
    // Every terminal cell receives its own convex port polygon.  Enumerating
    // all forward cells and selecting the closest feasible candidate gives the
    // global optimum without making a product lifecycle decision in the core.
    for (std::size_t terminal = current_pwl_cell;
         terminal < input.offset_pwl_cells.size(); ++terminal) {
      const PortProjectionPwlCell& cell = input.offset_pwl_cells[terminal];
      const double reachable_w_lower = input.phase + input.dt *
          (input.base_w_dot + w_lower);
      const double reachable_w_upper = input.phase + input.dt *
          (input.base_w_dot + w_upper);
      if (cell.w1 < reachable_w_lower - kBoundTolerance ||
          cell.w0 > reachable_w_upper + kBoundTolerance) {
        continue;
      }
      std::vector<HalfPlane> cell_planes = base_planes;
      const double w_next_base = input.phase + input.dt * input.base_w_dot;
      const double lower_base = LowerAt(cell, w_next_base) +
          constraint.interior_margin;
      const double upper_base = UpperAt(cell, w_next_base) -
          constraint.interior_margin;
      // Terminal cell membership: cell.w0 <= w_next <= cell.w1.
      HalfPlane terminal_w_lower;
      terminal_w_lower.a_w = input.dt;
      terminal_w_lower.b = cell.w0 - w_next_base;
      terminal_w_lower.label = "pwl_terminal_w_lower";
      cell_planes.push_back(terminal_w_lower);
      HalfPlane terminal_w_upper;
      terminal_w_upper.a_w = -input.dt;
      terminal_w_upper.b = w_next_base - cell.w1;
      terminal_w_upper.label = "pwl_terminal_w_upper";
      cell_planes.push_back(terminal_w_upper);
      // terminal delta >= lower(w_next) + margin.
      HalfPlane terminal_lower;
      terminal_lower.a_w = -cell.lower_w * input.dt;
      terminal_lower.a_delta = input.dt;
      terminal_lower.b = lower_base - input.delta;
      terminal_lower.label = "pwl_terminal_lower";
      cell_planes.push_back(terminal_lower);
      // terminal delta <= upper(w_next) - margin.
      HalfPlane terminal_upper;
      terminal_upper.a_w = cell.upper_w * input.dt;
      terminal_upper.a_delta = -input.dt;
      terminal_upper.b = input.delta - upper_base;
      terminal_upper.label = "pwl_terminal_upper";
      cell_planes.push_back(terminal_upper);

      // The terminal membership implies all earlier knots are crossed.  At a
      // crossed knot xi, elapsed=(xi-phase)/v; multiplying by v (>0 under the
      // existing non-reversing limits) yields linear half-planes in ports.
      bool terminal_candidate_valid = true;
      for (std::size_t crossed = current_pwl_cell; crossed < terminal; ++crossed) {
        const double xi = input.offset_pwl_cells[crossed].w1;
        const double distance = xi - input.phase;
        const double lower = LowerAt(input.offset_pwl_cells[crossed], xi) +
            constraint.interior_margin;
        const double upper = UpperAt(input.offset_pwl_cells[crossed], xi) -
            constraint.interior_margin;
        if (!IsFinite(distance) || distance < -kBoundTolerance ||
            !IsFinite(lower) || !IsFinite(upper) || lower > upper + kBoundTolerance) {
          terminal_candidate_valid = false;
          break;
        }
        HalfPlane knot_lower;
        knot_lower.a_w = -(lower - input.delta);
        knot_lower.a_delta = distance;
        knot_lower.b = (lower - input.delta) * input.base_w_dot;
        knot_lower.label = "pwl_crossed_lower";
        cell_planes.push_back(knot_lower);
        HalfPlane knot_upper;
        knot_upper.a_w = upper - input.delta;
        knot_upper.a_delta = -distance;
        knot_upper.b = -(upper - input.delta) * input.base_w_dot;
        knot_upper.label = "pwl_crossed_upper";
        cell_planes.push_back(knot_upper);
      }
      if (!terminal_candidate_valid) continue;
      PortProjectionDiagnostics cell_diagnostics;
      if (BuildFeasiblePolygon(cell_planes, cell_diagnostics)) {
        aggregate_diagnostics.core_polygon_nonempty = true;
        for (const PortCommand& vertex :
             cell_diagnostics.feasible_polygon_vertices) {
          AddUniqueVertex(vertex,
                          aggregate_diagnostics.feasible_polygon_vertices);
        }
        aggregate_diagnostics.feasible_polygon_edges.insert(
            aggregate_diagnostics.feasible_polygon_edges.end(),
            cell_diagnostics.feasible_polygon_edges.begin(),
            cell_diagnostics.feasible_polygon_edges.end());
      }
      const bool previously_found = found;
      const double prior_cost = best_cost;
      PortCommand prior_best = best;
      AddNearestCandidates(cell_planes, input.raw, found, best_cost, best);
      if (found && (!previously_found || best_cost < prior_cost - kBoundTolerance ||
                    (std::abs(best_cost - prior_cost) <= kBoundTolerance &&
                     (best.u_w != prior_best.u_w || best.u_delta != prior_best.u_delta)))) {
        winning_planes = cell_planes;
      }
    }
  }
  if (!found) {
    return Invalidate(output, "joint port feasible polygon is empty");
  }
  if (winning_planes.empty()) winning_planes = planes;

  // Keep the exact polygon witness set in the existing core diagnostics.  It
  // is not a Runtime state or a product gate: Runtime uses the same finite
  // raw-projection/vertex candidates only to prove a short viable sequence.
  // BuildFeasiblePolygon owns the polygon witness, while the scalar intervals
  // were derived before enumeration.  Preserve those authoritative bounds
  // when replacing the diagnostics object; dropping them here made a valid
  // bounded polygon appear unbounded to RecoveryOwner::buildAdmissibleSet().
  aggregate_diagnostics.u_w_interval = {w_lower, w_upper, true};
  aggregate_diagnostics.u_delta_interval = {delta_lower, delta_upper, true};
  output.diagnostics = aggregate_diagnostics;
  output.diagnostics.core_polygon_nonempty = true;
  AddUniqueVertex(best, output.diagnostics.feasible_polygon_vertices);
  output.diagnostics.projection_cost = best_cost;

  output.final_port = best;
  output.final_w_dot = input.base_w_dot + best.u_w;
  output.final_tangent_speed = input.base_tangent_speed + input.r_w_norm * best.u_w;
  output.next_delta = input.delta + input.dt * best.u_delta;
  output.next_regularity = 1.0 - input.curvature * output.next_delta;
  output.upper_invariant_residual = constraint.upper_w * output.final_w_dot -
      best.u_delta + constraint.invariant_gain * upper_h;
  output.lower_invariant_residual = best.u_delta -
      constraint.lower_w * output.final_w_dot +
      constraint.invariant_gain * lower_h;
  const double exact_next_upper_residual = constraint.upper_w *
      output.final_w_dot - best.u_delta + upper_h / input.dt;
  const double exact_next_lower_residual = best.u_delta -
      constraint.lower_w * output.final_w_dot + lower_h / input.dt;
  const double next_lower = constraint.lower + constraint.lower_w * input.dt *
      output.final_w_dot + constraint.interior_margin;
  const double next_upper = constraint.upper + constraint.upper_w * input.dt *
      output.final_w_dot - constraint.interior_margin;
  const bool pwl_contains = !exact_pwl || PwlContains(
      input.offset_pwl_cells, input.phase, input.dt, input.delta,
      constraint.interior_margin, best, input.base_w_dot);
  if (!IsFinite(output.final_w_dot) || !IsFinite(output.final_tangent_speed) ||
      !IsFinite(output.next_delta) || !IsFinite(output.next_regularity) ||
      !IsFinite(output.upper_invariant_residual) ||
      !IsFinite(output.lower_invariant_residual) ||
      !IsFinite(exact_next_upper_residual) ||
      !IsFinite(exact_next_lower_residual) ||
      !IsFinite(next_lower) || !IsFinite(next_upper) || !pwl_contains ||
      output.final_w_dot < limits.phase_dot_min - kBoundTolerance ||
      output.final_tangent_speed < limits.tangent_speed_min - kBoundTolerance ||
      output.next_regularity < limits.regularity_margin - kBoundTolerance ||
      (!exact_pwl && (output.next_delta < next_lower - kBoundTolerance ||
                      output.next_delta > next_upper + kBoundTolerance ||
                      exact_next_upper_residual < -kBoundTolerance ||
                      exact_next_lower_residual < -kBoundTolerance)) ||
      (!exact_pwl &&
       (output.upper_invariant_residual < -kBoundTolerance ||
        output.lower_invariant_residual < -kBoundTolerance))) {
    return Invalidate(output, "joint projected port violates a constraint");
  }

  std::vector<HalfPlane> diagnostic_planes = winning_planes;
  if (exact_pwl) {
    // The planes deliberately remain observable even though exact PWL
    // containment, not their continuous tangent approximation, is hard.
    diagnostic_planes.push_back(upper_invariant);
    diagnostic_planes.push_back(lower_invariant);
  }
  PopulateResiduals(best, diagnostic_planes, output.diagnostics);
  PopulateConflicts(input.raw, diagnostic_planes, output.diagnostics);

  output.u_w_limited = std::abs(best.u_w - input.raw.u_w) > kBoundTolerance;
  output.u_delta_limited =
      std::abs(best.u_delta - input.raw.u_delta) > kBoundTolerance;
  output.offset_constraint_limited =
      (output.u_w_limited || output.u_delta_limited) &&
      (exact_pwl || LabelIsActive(best, winning_planes, "exact_next_upper") ||
       LabelIsActive(best, winning_planes, "exact_next_lower") ||
       (!exact_pwl &&
        (LabelIsActive(best, winning_planes, "upper_invariant") ||
         LabelIsActive(best, winning_planes, "lower_invariant"))));
  if (output.u_w_limited) {
    output.u_w_limit_reason = output.offset_constraint_limited
        ? "joint_offset_constraint" : "joint_local_constraint";
  }
  if (output.u_delta_limited) {
    output.u_delta_limit_reason = output.offset_constraint_limited
        ? "joint_offset_constraint" : "joint_local_constraint";
  }
  if (output.offset_constraint_limited) {
    output.offset_constraint_limit_reason = exact_pwl
        ? "joint_exact_pwl" : "joint_invariant";
  }
  output.valid = true;
  output.invalid_reason.clear();
  return true;
}

bool PortProjector::buildAdmissibleSet(
    const PortProjectionInput& input, const PortProjectionLimits& limits,
    AdmissibleSet& output) {
  output = AdmissibleSet();
  PortProjectionInput probe = input;
  // The probe is only used to expose the bounded polygon generated by the
  // existing half-plane kernel.  It must never become an execution command.
  probe.raw = PortCommand();
  PortProjectionResult projected;
  if (!project(probe, limits, projected) || !projected.valid) {
    output.invalid_reason = projected.invalid_reason.empty()
        ? "admissible port set is empty" : projected.invalid_reason;
    return false;
  }
  output.u_w = projected.diagnostics.u_w_interval;
  output.u_delta = projected.diagnostics.u_delta_interval;
  output.vertices = projected.diagnostics.feasible_polygon_vertices;
  output.edges = projected.diagnostics.feasible_polygon_edges;
  output.bounded = output.u_w.valid && output.u_delta.valid &&
      std::isfinite(output.u_w.lower) && std::isfinite(output.u_w.upper) &&
      std::isfinite(output.u_delta.lower) &&
      std::isfinite(output.u_delta.upper);
  output.closed = output.bounded && !output.vertices.empty();
  for (const PortCommand& vertex : output.vertices) {
    if (!IsFinite(vertex) || vertex.u_w < output.u_w.lower - kBoundTolerance ||
        vertex.u_w > output.u_w.upper + kBoundTolerance ||
        vertex.u_delta < output.u_delta.lower - kBoundTolerance ||
        vertex.u_delta > output.u_delta.upper + kBoundTolerance) {
      output.closed = false;
      break;
    }
  }
  for (const PortProjectionPolygonEdge& edge : output.edges) {
    if (!IsFinite(edge.first) || !IsFinite(edge.second)) {
      output.closed = false;
      break;
    }
  }
  output.valid = output.bounded && output.closed &&
      projected.diagnostics.core_polygon_nonempty;
  if (!output.valid) {
    output.invalid_reason = "admissible port set is unbounded or empty";
  }
  return output.valid;
}

bool PortProjector::verify(const PortProjectionInput& input,
                           const PortProjectionLimits& limits,
                           const PortCommand& command,
                           PortProjectionResult& output) {
  PortProjectionInput exact = input;
  exact.raw = command;
  if (!project(exact, limits, output) || !output.valid) return false;
  constexpr double kCommandTolerance = 1e-12;
  if (std::abs(output.final_port.u_w - command.u_w) > kCommandTolerance ||
      std::abs(output.final_port.u_delta - command.u_delta) >
          kCommandTolerance) {
    output.valid = false;
    output.invalid_reason = "caller-selected command is not admissible";
    return false;
  }
  return true;
}

}  // namespace phase_offset_core
