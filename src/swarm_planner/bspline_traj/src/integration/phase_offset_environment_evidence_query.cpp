#include "bspline_race/integration/phase_offset_environment_evidence_query.h"

#include <plan_env/sdf_map.h>

namespace FLAG_Race {
namespace {

phase_offset_navigation::EnvironmentEvidenceResult unavailableResult() {
  return phase_offset_navigation::EnvironmentEvidenceResult();
}

bool translateStatus(const plan_env::SDFMapEnvironmentEvidenceStatus source,
                     phase_offset_navigation::DistanceStatus& destination) {
  switch (source) {
    case plan_env::SDFMapEnvironmentEvidenceStatus::UNAVAILABLE:
      destination = phase_offset_navigation::DistanceStatus::UNAVAILABLE;
      return true;
    case plan_env::SDFMapEnvironmentEvidenceStatus::OUT_OF_MAP:
      destination = phase_offset_navigation::DistanceStatus::OUT_OF_MAP;
      return true;
    case plan_env::SDFMapEnvironmentEvidenceStatus::UNKNOWN:
      destination = phase_offset_navigation::DistanceStatus::UNKNOWN;
      return true;
    case plan_env::SDFMapEnvironmentEvidenceStatus::KNOWN_FREE:
      destination = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
      return true;
    case plan_env::SDFMapEnvironmentEvidenceStatus::OCCUPIED:
      destination = phase_offset_navigation::DistanceStatus::OCCUPIED;
      return true;
  }
  return false;
}

}  // namespace

phase_offset_navigation::EnvironmentEvidenceResult
translateSDFMapEnvironmentEvidence(
    const plan_env::SDFMapEnvironmentEvidenceResult& source) {
  if (!plan_env::sdfMapEnvironmentEvidenceConsistent(source)) {
    return unavailableResult();
  }

  phase_offset_navigation::EnvironmentEvidenceResult result;
  if (!translateStatus(source.status, result.status)) {
    return unavailableResult();
  }
  result.uninflated_distance = source.uninflated_distance;
  result.distance_valid = source.distance_valid;
  result.included_layer_mask = source.included_layer_mask;
  result.unsupported_active_layer_mask = source.unsupported_active_layer_mask;
  if (!phase_offset_navigation::environmentEvidenceConsistent(result)) {
    return unavailableResult();
  }
  return result;
}

phase_offset_navigation::EnvironmentEvidenceQuery makeEnvironmentEvidenceQuery(
    SDFMap* map) {
  return [map](const Eigen::Vector3d& point) {
    if (map == nullptr) return unavailableResult();
    return translateSDFMapEnvironmentEvidence(
        map->queryObservedUninflatedEnvironment(point));
  };
}

}  // namespace FLAG_Race
