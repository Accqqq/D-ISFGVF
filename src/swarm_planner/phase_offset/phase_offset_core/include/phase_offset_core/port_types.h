#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "phase_offset_core/offset_constraint.h"

namespace phase_offset_core {

// A pair of scalar ports applied to the phase and lateral-reference dynamics.
struct PortCommand {
  double u_w = 0.0;
  double u_delta = 0.0;
};

// A focused status for the single PortProjector kernel.  The value is part of
// diagnostics only; callers must use valid rather than infer feasibility from
// a text string.
enum class PortProjectionFailureStage {
  NONE,
  INVALID_LIMITS,
  INVALID_INPUT,
  INVALID_OFFSET_CONSTRAINT,
  CURRENT_REGULARITY,
  CURRENT_OFFSET_MARGIN,
  LOCAL_INTERVAL_EMPTY,
  CORE_POLYGON_EMPTY,
  FINAL_CONSTRAINT_CHECK,
};

struct PortProjectionInterval {
  double lower = 0.0;
  double upper = 0.0;
  bool valid = false;
};

// One caller-owned affine cell of an enabled piecewise-linear offset tube.
// It is generic core mathematics: both boundaries are affine in phase and the
// interval is valid only on [w0, w1].  Runtime supplies cells from its active
// immutable profile; the projector stores no tube or path state.
struct PortProjectionPwlCell {
  double w0 = 0.0;
  double w1 = 0.0;
  double lower_w = 0.0;
  double lower_at_w0 = 0.0;
  double upper_w = 0.0;
  double upper_at_w0 = 0.0;
};

// One directed boundary of the core convex port set.  The coordinates are
// always ports, never path or tube geometry, so the core remains pure C++.
struct PortProjectionPolygonEdge {
  PortCommand first;
  PortCommand second;
  std::string active_constraint;
};

struct PortProjectionConstraintResidual {
  std::string label;
  double residual = 0.0;
};

struct PortProjectionDiagnostics {
  PortProjectionFailureStage failure_stage = PortProjectionFailureStage::NONE;
  PortProjectionInterval u_w_interval;
  PortProjectionInterval u_delta_interval;
  PortProjectionInterval offset_step_interval;
  bool core_polygon_nonempty = false;
  std::vector<PortCommand> feasible_polygon_vertices;
  std::vector<PortProjectionPolygonEdge> feasible_polygon_edges;
  std::vector<PortProjectionConstraintResidual> constraint_residuals;
  std::vector<std::string> conflicting_constraints;
  double projection_cost = 0.0;
};

// Local, map-free kinematic constraints for a candidate port.
struct PortProjectionLimits {
  double u_w_abs_max = 0.0;
  double u_delta_abs_max = 0.0;
  double u_w_rate_max = 0.0;
  double u_delta_rate_max = 0.0;
  double phase_dot_min = 0.0;
  double tangent_speed_min = 0.0;
  double regularity_margin = 0.1;
};

// All history is explicit: the projector itself owns no time or port state.
struct PortProjectionInput {
  PortCommand raw;
  PortCommand previous_final;
  double dt = 0.0;
  double delta = 0.0;
  double curvature = 0.0;
  double r_w_norm = 0.0;
  double phase = 0.0;
  double base_w_dot = 0.0;
  double base_tangent_speed = 0.0;
  OffsetConstraint offset_constraint;
  // Empty preserves the existing affine-offset-constraint behavior.  When
  // present, these are exact sample-and-hold terminal/crossed-knot cells.
  std::vector<PortProjectionPwlCell> offset_pwl_cells;
};

struct PortProjectionResult {
  PortCommand final_port;
  double final_w_dot = 0.0;
  double final_tangent_speed = 0.0;
  double next_delta = 0.0;
  double next_regularity = 0.0;
  double upper_invariant_residual = 0.0;
  double lower_invariant_residual = 0.0;
  bool u_w_limited = false;
  bool u_delta_limited = false;
  bool offset_constraint_limited = false;
  std::string u_w_limit_reason;
  std::string u_delta_limit_reason;
  std::string offset_constraint_limit_reason;
  bool valid = false;
  std::string invalid_reason;
  PortProjectionDiagnostics diagnostics;
};

}  // namespace phase_offset_core
