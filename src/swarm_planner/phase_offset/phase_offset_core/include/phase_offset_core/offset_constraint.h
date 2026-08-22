#pragma once

namespace phase_offset_core {

// A generic scalar envelope in phase coordinates.  It deliberately contains
// no map, ESDF, tube, or ROS concepts so that the core remains reusable.
struct OffsetConstraint {
  double lower = 0.0;
  double upper = 0.0;
  double lower_w = 0.0;
  double upper_w = 0.0;
  double invariant_gain = 0.0;
  double interior_margin = 0.0;
  bool enabled = false;
  bool valid = false;
};

}  // namespace phase_offset_core
