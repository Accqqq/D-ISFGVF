#include <plan_env/sdf_map.h>

#include <cmath>

namespace plan_env {
namespace {

constexpr double kLegacyDistanceSentinel = 10000.0;

bool knownLayerMask(const std::uint32_t mask) {
  return (mask & ~static_cast<std::uint32_t>(
                     kSDFMapEnvironmentLayerAll)) == 0U;
}

bool finiteNonnegativeDistance(const double distance) {
  return std::isfinite(distance) && distance >= 0.0;
}

SDFMapEnvironmentEvidenceResult unavailableResult(
    const SDFMapEnvironmentEvidenceStatus status) {
  SDFMapEnvironmentEvidenceResult result;
  result.status = status;
  result.unsupported_active_layer_mask =
      kSDFMapCurrentUnsupportedEnvironmentLayerMask;
  return result;
}

}  // namespace

bool sdfMapEnvironmentEvidenceConsistent(
    const SDFMapEnvironmentEvidenceResult& result) {
  if (!finiteNonnegativeDistance(result.uninflated_distance) ||
      !knownLayerMask(result.included_layer_mask) ||
      !knownLayerMask(result.unsupported_active_layer_mask) ||
      (result.included_layer_mask & result.unsupported_active_layer_mask) !=
          0U) {
    return false;
  }

  switch (result.status) {
    case SDFMapEnvironmentEvidenceStatus::UNAVAILABLE:
    case SDFMapEnvironmentEvidenceStatus::OUT_OF_MAP:
    case SDFMapEnvironmentEvidenceStatus::UNKNOWN:
      return !result.distance_valid && result.uninflated_distance == 0.0;
    case SDFMapEnvironmentEvidenceStatus::OCCUPIED:
      return result.distance_valid && result.uninflated_distance == 0.0 &&
          result.included_layer_mask != kSDFMapEnvironmentLayerNone &&
          result.unsupported_active_layer_mask ==
              kSDFMapEnvironmentLayerNone;
    case SDFMapEnvironmentEvidenceStatus::KNOWN_FREE:
      return result.distance_valid && result.uninflated_distance > 0.0 &&
          result.uninflated_distance != kLegacyDistanceSentinel &&
          result.included_layer_mask != kSDFMapEnvironmentLayerNone &&
          result.unsupported_active_layer_mask ==
              kSDFMapEnvironmentLayerNone;
  }
  return false;
}

}  // namespace plan_env

plan_env::SDFMapEnvironmentEvidenceCapabilities
SDFMap::observedUninflatedEnvironmentCapabilities() const {
  return plan_env::SDFMapEnvironmentEvidenceCapabilities();
}

plan_env::SDFMapEnvironmentEvidenceResult
SDFMap::queryObservedUninflatedEnvironment(const Eigen::Vector3d& point) {
  if (!point.allFinite() || !isInMap(point)) {
    return plan_env::unavailableResult(
        plan_env::SDFMapEnvironmentEvidenceStatus::OUT_OF_MAP);
  }
  return plan_env::unavailableResult(
      plan_env::SDFMapEnvironmentEvidenceStatus::UNAVAILABLE);
}
