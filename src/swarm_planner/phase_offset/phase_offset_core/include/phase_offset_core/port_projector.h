#pragma once

#include "phase_offset_core/port_types.h"

#include <string>
#include <vector>
#include <cmath>

namespace phase_offset_core {

// Stateless local projector for the two matched ports.  It has no path or
// hidden-history state.
class PortProjector {
 public:
  static bool project(const PortProjectionInput& input,
                      const PortProjectionLimits& limits,
                      PortProjectionResult& output);

  // Feasibility-only companion to project().  This API deliberately exposes
  // the bounded local admissible set without selecting an execution command.
  // Recovery/allocator owners may use the returned vertices and intervals to
  // perform their own deterministic selection.  The projector remains a
  // stateless kinematic kernel and owns no mode, lifecycle, or authority.
  struct AdmissibleSet {
    PortProjectionInterval u_w;
    PortProjectionInterval u_delta;
    std::vector<PortCommand> vertices;
    std::vector<PortProjectionPolygonEdge> edges;
    bool bounded = false;
    bool closed = true;
    bool valid = false;
    std::string invalid_reason;

    bool contains(const PortCommand& command,
                  double tolerance = 1e-12) const {
      return valid && bounded && closed && std::isfinite(command.u_w) &&
          std::isfinite(command.u_delta) && u_w.valid && u_delta.valid &&
          command.u_w >= u_w.lower - tolerance &&
          command.u_w <= u_w.upper + tolerance &&
          command.u_delta >= u_delta.lower - tolerance &&
          command.u_delta <= u_delta.upper + tolerance;
    }
  };

  static bool buildAdmissibleSet(const PortProjectionInput& input,
                                 const PortProjectionLimits& limits,
                                 AdmissibleSet& output);

  // Verifies one caller-selected command against exactly the same local
  // constraints.  No replacement or clamping is performed: a command is
  // either accepted unchanged or rejected.
  static bool verify(const PortProjectionInput& input,
                     const PortProjectionLimits& limits,
                     const PortCommand& command,
                     PortProjectionResult& output);
};

}  // namespace phase_offset_core
