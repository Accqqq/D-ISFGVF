#pragma once

#include <phase_offset_core/path_state.h>

#include <functional>

namespace phase_offset_navigation {

// Pure-C++ bridge to the authoritative continuous base path.  Navigation
// owns neither the B-spline nor ROS objects; adapters bind this callback for
// one immutable candidate build.
using PathStateQuery = std::function<bool(
    double w, phase_offset_core::PathDifferentialState& state)>;

// Query-only cell certificate tied to the same immutable path owner as the
// point query.  Callers must not synthesize a certificate from dense samples;
// a false result is the explicit unsupported/segment-crossing/low-speed
// fail-closed outcome.
using PathCellBoundQuery = std::function<bool(
    double w0, double w1,
    phase_offset_core::PathCellGeometryCertificate& certificate)>;

}  // namespace phase_offset_navigation
