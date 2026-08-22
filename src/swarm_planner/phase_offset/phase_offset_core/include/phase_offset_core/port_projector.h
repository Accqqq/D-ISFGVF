#pragma once

#include "phase_offset_core/port_types.h"

namespace phase_offset_core {

// Stateless local projector for the two matched ports.  It has no path or
// hidden-history state.
class PortProjector {
 public:
  static bool project(const PortProjectionInput& input,
                      const PortProjectionLimits& limits,
                      PortProjectionResult& output);
};

}  // namespace phase_offset_core
