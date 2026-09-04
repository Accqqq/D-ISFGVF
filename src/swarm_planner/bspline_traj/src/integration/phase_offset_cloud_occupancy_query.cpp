#include "bspline_race/integration/phase_offset_cloud_occupancy_query.h"

#include <cmath>
#include <limits>

namespace FLAG_Race {
namespace {

constexpr double kTolerance = 1e-12;
constexpr double kCurrentPhaseMatchTolerance = 1e-9;
constexpr double kMissingStatus = -1.0;

bool finite(const double value) {
  return std::isfinite(value);
}

bool finite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

double finiteOrZero(const double value) {
  return finite(value) ? value : 0.0;
}

double asDouble(const bool value) {
  return value ? 1.0 : 0.0;
}

phase_offset_navigation::DistanceStatus translate(
    const plan_env::CloudOccupancyStatus status) {
  using phase_offset_navigation::DistanceStatus;
  switch (status) {
    case plan_env::CloudOccupancyStatus::UNAVAILABLE:
      return DistanceStatus::UNAVAILABLE;
    case plan_env::CloudOccupancyStatus::OUT_OF_MAP:
      return DistanceStatus::OUT_OF_MAP;
    case plan_env::CloudOccupancyStatus::UNKNOWN:
      return DistanceStatus::UNKNOWN;
    case plan_env::CloudOccupancyStatus::KNOWN_FREE:
      return DistanceStatus::KNOWN_FREE;
    case plan_env::CloudOccupancyStatus::OCCUPIED:
      return DistanceStatus::OCCUPIED;
  }
  return DistanceStatus::UNAVAILABLE;
}

double queryStatus(const phase_offset_navigation::RawOccupancyQuery& query,
                   const Eigen::Vector3d& point) {
  if (!query || !finite(point)) return kMissingStatus;
  return static_cast<double>(query(point));
}

const phase_offset_navigation::TubeRawSample* currentSample(
    const phase_offset_navigation::TubeProfile* profile,
    const double current_w) {
  if (profile == nullptr || !finite(current_w)) return nullptr;
  const phase_offset_navigation::TubeRawSample* selected = nullptr;
  double best = std::numeric_limits<double>::infinity();
  for (const auto& sample : profile->samples) {
    if (!finite(sample.w)) continue;
    const double error = std::abs(sample.w - current_w);
    if (error <= kCurrentPhaseMatchTolerance && error < best) {
      selected = &sample;
      best = error;
    }
  }
  return selected;
}

}  // namespace

bool cloudOccupancyQueryConfigurationValid(
    const CloudOccupancyQueryConfig& config) {
  return finite(config.required_preincluded_map_uncertainty) &&
      config.required_preincluded_map_uncertainty >= 0.0;
}

CloudOccupancyQueryStatus inspectCloudOccupancyQuery(
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    const CloudOccupancyQueryConfig& config) {
  CloudOccupancyQueryStatus status;
  status.configuration_valid = cloudOccupancyQueryConfigurationValid(config);
  status.obstacle_set_complete = config.obstacle_set_complete;
  status.snapshot_available = static_cast<bool>(snapshot);
  if (snapshot) {
    status.snapshot_valid = plan_env::cloudOccupancySnapshotConsistent(*snapshot);
    status.observation_sequence = snapshot->observation_sequence;
    status.observation_stamp = snapshot->observation_stamp;
    status.included_map_inflation = finite(snapshot->included_map_inflation)
        ? snapshot->included_map_inflation : 0.0;
  }
  status.included_map_inflation_sufficient = status.snapshot_valid &&
      status.included_map_inflation + kTolerance >=
          config.required_preincluded_map_uncertainty;
  status.usable = status.configuration_valid && status.obstacle_set_complete &&
      status.snapshot_valid && status.included_map_inflation_sufficient;
  return status;
}

phase_offset_navigation::RawOccupancyQuery makeCloudOccupancyQuery(
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    const CloudOccupancyQueryConfig& config) {
  const CloudOccupancyQueryStatus status =
      inspectCloudOccupancyQuery(snapshot, config);
  return [snapshot, status](const Eigen::Vector3d& point) {
    if (!status.usable || !snapshot) {
      return phase_offset_navigation::DistanceStatus::UNAVAILABLE;
    }
    return translate(plan_env::queryCloudOccupancySnapshot(*snapshot, point).status);
  };
}

phase_offset_navigation::ClearanceQuery makeCloudOccupancyClearanceQuery(
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    const CloudOccupancyQueryConfig& config) {
  const CloudOccupancyQueryStatus status =
      inspectCloudOccupancyQuery(snapshot, config);
  return [snapshot, status](const Eigen::Vector3d& point,
                            const double required_radius) {
    phase_offset_navigation::ClearanceQueryResult result;
    if (!status.usable || !snapshot || !finite(point) ||
        !finite(required_radius) || required_radius < 0.0) {
      return result;
    }
    const plan_env::CloudOccupancySnapshotPlannerEsdfBaseClearanceResult cloud =
        plan_env::queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
            *snapshot, point, required_radius);
    result.status = translate(cloud.status);
    result.clearance =
        finite(cloud.nearest_inflated_occupied_voxel_center_distance)
            ? cloud.nearest_inflated_occupied_voxel_center_distance : 0.0;
    result.clearance_certified = cloud.clearance_certified;
    // The immutable planner-ESDF query caps a larger distance at the
    // requested radius.  Equality is therefore conservatively treated as a
    // certified lower bound; only a strict interior value is exact evidence.
    result.clearance_is_exact = result.status ==
            phase_offset_navigation::DistanceStatus::KNOWN_FREE &&
        result.clearance_certified && result.clearance < required_radius;
    return result;
  };
}

const std::array<const char*, kCloudSnapshotDiagnosticCount>&
cloudSnapshotDiagnosticFieldNames() {
  static const std::array<const char*, kCloudSnapshotDiagnosticCount> names = {{
      "schema_version", "cloud_obstacle_set_complete",
      "query_configuration_valid", "snapshot_available", "snapshot_valid",
      "snapshot_usable", "observation_sequence", "observation_stamp",
      "included_map_inflation", "configured_preincluded_map_uncertainty",
      "full_effective_radius", "residual_effective_radius",
      "current_base_status", "actual_position_status",
      "current_plus_step_status", "current_minus_step_status",
      "raw_storage_accessed", "self_free_seed_used"}};
  return names;
}

std::array<double, kCloudSnapshotDiagnosticCount>
makeCloudSnapshotDiagnostics(const CloudSnapshotDiagnosticsInput& input) {
  std::array<double, kCloudSnapshotDiagnosticCount> values = {{0.0}};
  const auto* current = currentSample(input.candidate_profile, input.current_w);
  const Eigen::Vector3d base = current != nullptr ? current->p : input.current_path.p;
  values[kCloudSnapshotSchemaVersion] = 1.0;
  values[kCloudSnapshotObstacleSetComplete] =
      asDouble(input.query_status.obstacle_set_complete);
  values[kCloudSnapshotQueryConfigurationValid] =
      asDouble(input.query_status.configuration_valid);
  values[kCloudSnapshotAvailable] = asDouble(input.query_status.snapshot_available);
  values[kCloudSnapshotValid] = asDouble(input.query_status.snapshot_valid);
  values[kCloudSnapshotUsable] = asDouble(input.query_status.usable);
  values[kCloudSnapshotObservationSequence] =
      static_cast<double>(input.query_status.observation_sequence);
  values[kCloudSnapshotObservationStamp] =
      input.query_status.observation_stamp.toSec();
  values[kCloudSnapshotIncludedMapInflation] =
      input.query_status.included_map_inflation;
  values[kCloudSnapshotConfiguredPreincludedMapUncertainty] =
      input.query_config.required_preincluded_map_uncertainty;
  values[kCloudSnapshotFullEffectiveRadius] =
      input.margins.fullEffectiveRadius();
  values[kCloudSnapshotResidualEffectiveRadius] =
      input.margins.residualEffectiveRadius();
  values[kCloudSnapshotCurrentBaseStatus] =
      queryStatus(input.categorical_query, base);
  values[kCloudSnapshotActualStatus] =
      queryStatus(input.categorical_query, input.actual_position);
  values[kCloudSnapshotCurrentPlusStepStatus] = kMissingStatus;
  values[kCloudSnapshotCurrentMinusStepStatus] = kMissingStatus;
  if (current != nullptr && finite(base) && finite(current->N) &&
      finite(input.ray_step) && input.ray_step > 0.0) {
    const double normal_norm = current->N.norm();
    if (finite(normal_norm) && normal_norm > kTolerance) {
      const Eigen::Vector3d normal = current->N / normal_norm;
      values[kCloudSnapshotCurrentPlusStepStatus] = queryStatus(
          input.categorical_query, base + input.ray_step * normal);
      values[kCloudSnapshotCurrentMinusStepStatus] = queryStatus(
          input.categorical_query, base - input.ray_step * normal);
    }
  }
  // These two facts intentionally remain zero: production construction never
  // touches SDFMap raw occupancy or a RawOccupancySelfFreeSeed.
  values[kCloudSnapshotRawStorageAccessed] = 0.0;
  values[kCloudSnapshotSelfFreeSeedUsed] = 0.0;
  for (double& value : values) value = finiteOrZero(value);
  return values;
}

}  // namespace FLAG_Race
