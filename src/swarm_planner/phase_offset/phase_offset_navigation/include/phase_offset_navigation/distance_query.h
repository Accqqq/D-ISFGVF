#pragma once

#include <Eigen/Core>

#include <functional>

namespace phase_offset_navigation {

enum class DistanceStatus {
  UNAVAILABLE,
  OUT_OF_MAP,
  UNKNOWN,
  KNOWN_FREE,
  OCCUPIED,
};

struct DistanceQueryResult {
  DistanceStatus status = DistanceStatus::UNAVAILABLE;
  double signed_distance = 0.0;
};

using DistanceQuery = std::function<DistanceQueryResult(const Eigen::Vector3d&)>;

// A bounded, read-only Euclidean-clearance query.  `required_radius` is a
// lower bound the caller needs certified; an implementation may cap a larger
// true distance at that value.  This lets an immutable voxel snapshot search
// only the required local neighborhood without leaking map-specific types
// into navigation.
struct ClearanceQueryResult {
  DistanceStatus status = DistanceStatus::UNAVAILABLE;
  double clearance = 0.0;
  bool clearance_certified = false;
  // True only when `clearance` is the exact nearest-centre distance.  A
  // certified value equal to (or capped at) the requested radius is a lower
  // bound and must remain conservatively distinguishable by proof consumers.
  bool clearance_is_exact = false;
};

using ClearanceQuery = std::function<ClearanceQueryResult(
    const Eigen::Vector3d&, double required_radius)>;

}  // namespace phase_offset_navigation
