#include "phase_offset_navigation/environment_evidence.h"

#include <cmath>

namespace phase_offset_navigation {
namespace {

constexpr double kLegacyDistanceSentinel = 10000.0;

bool knownLayerMask(const std::uint32_t mask) {
  return (mask & ~static_cast<std::uint32_t>(kEnvironmentLayerAll)) == 0U;
}

bool masksDoNotOverlap(const EnvironmentEvidenceResult& result) {
  return (result.included_layer_mask & result.unsupported_active_layer_mask) ==
      0U;
}

bool finiteNonnegativeDistance(const double distance) {
  return std::isfinite(distance) && distance >= 0.0;
}

}  // namespace

bool environmentEvidenceConsistent(const EnvironmentEvidenceResult& result) {
  if (!finiteNonnegativeDistance(result.uninflated_distance) ||
      !knownLayerMask(result.included_layer_mask) ||
      !knownLayerMask(result.unsupported_active_layer_mask) ||
      !masksDoNotOverlap(result)) {
    return false;
  }

  switch (result.status) {
    case DistanceStatus::UNAVAILABLE:
    case DistanceStatus::OUT_OF_MAP:
    case DistanceStatus::UNKNOWN:
      return !result.distance_valid && result.uninflated_distance == 0.0;
    case DistanceStatus::OCCUPIED:
      return result.distance_valid && result.uninflated_distance == 0.0 &&
          result.included_layer_mask != kEnvironmentLayerNone &&
          result.unsupported_active_layer_mask == kEnvironmentLayerNone;
    case DistanceStatus::KNOWN_FREE:
      return result.distance_valid && result.uninflated_distance > 0.0 &&
          result.uninflated_distance != kLegacyDistanceSentinel &&
          result.included_layer_mask != kEnvironmentLayerNone &&
          result.unsupported_active_layer_mask == kEnvironmentLayerNone;
  }
  return false;
}

bool environmentEvidenceHasUsableDistance(
    const EnvironmentEvidenceResult& result) {
  if (!environmentEvidenceConsistent(result) || !result.distance_valid ||
      result.unsupported_active_layer_mask != kEnvironmentLayerNone) {
    return false;
  }
  return result.status == DistanceStatus::OCCUPIED ||
      result.status == DistanceStatus::KNOWN_FREE;
}

}  // namespace phase_offset_navigation
