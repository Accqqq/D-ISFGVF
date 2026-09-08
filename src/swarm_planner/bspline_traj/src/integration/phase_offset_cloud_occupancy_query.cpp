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

constexpr std::uint32_t kKnownCaptureLayerMask =
    plan_env::kSDFMapCaptureLayerSensor |
    plan_env::kSDFMapCaptureLayerManual |
    plan_env::kSDFMapCaptureLayerStatic |
    plan_env::kSDFMapCaptureLayerCeiling;
constexpr std::uint32_t kKnownSupportEvidenceMask =
    plan_env::kSDFMapCaptureSupportEvidenceDepthRaycast |
    plan_env::kSDFMapCaptureSupportEvidenceCompletePreknownDomain;

bool validPositiveCount(const Eigen::Vector3i& count) {
  return (count.array() > 0).all();
}

bool captureGeometryValid(const plan_env::SDFMapCaptureV2& capture) {
  return finite(capture.map_min) && finite(capture.map_max) &&
      finite(capture.capture_min) && finite(capture.capture_max) &&
      finite(capture.grid_origin) && finite(capture.resolution) &&
      capture.resolution > 0.0 && validPositiveCount(capture.voxel_count) &&
      (capture.map_max.array() > capture.map_min.array()).all() &&
      (capture.capture_max.array() >= capture.capture_min.array()).all() &&
      (capture.effective_layer_mask & ~kKnownCaptureLayerMask) == 0U;
}

bool supportExpiryMetadataValid(
    const plan_env::SDFMapCaptureV2& capture) {
  const plan_env::SDFMapCaptureSupportV2& support = capture.support;
  if (!std::isfinite(support.valid_until.toSec())) return false;
  if (support.valid_until_accepted_ticks == 0U) {
    return support.valid_until.isZero();
  }
  return !support.valid_until.isZero() &&
      support.valid_until_accepted_ticks >=
          support.evidence_accepted_ticks &&
      support.valid_until_accepted_ticks >= capture.accepted_time_ticks;
}

bool supportMetadataValid(const plan_env::SDFMapCaptureV2& capture) {
  const plan_env::SDFMapCaptureSupportV2& support = capture.support;
  if (!support.valid || !support.complete ||
      support.evidence_sequence == 0U ||
      support.evidence_accepted_ticks == 0U ||
      support.evidence_accepted_ticks > capture.accepted_time_ticks ||
      support.evidence_sequence > capture.accepted_state_sequence ||
      support.map_instance_id != capture.map_instance_id ||
      support.configuration_generation != capture.configuration_generation ||
      support.configuration_key != capture.configuration_key ||
      support.frame_id != capture.frame_id ||
      (support.evidence_basis & ~kKnownSupportEvidenceMask) != 0U ||
      (support.evidence_basis &
           plan_env::kSDFMapCaptureSupportEvidenceCompletePreknownDomain) ==
          0U ||
      !finite(support.support_min) || !finite(support.support_max) ||
      (support.support_max.array() < support.support_min.array()).any() ||
      !finite(support.required_halo) || support.required_halo < 0.0 ||
      !support.halo_reconciled ||
      support.mask.size() != capture.occupied.size()) {
    return false;
  }
  return supportExpiryMetadataValid(capture);
}

void fillMapCaptureKey(
    const plan_env::SDFMapCaptureV2& capture,
    const SDFMapCaptureQueryStatusV2& status,
    phase_offset_navigation::TubeMapCaptureKey& key) {
  key = phase_offset_navigation::TubeMapCaptureKey();
  key.map_instance_id = capture.map_instance_id;
  key.state_id = capture.accepted_state_sequence;
  key.accepted_sequence = capture.accepted_state_sequence;
  key.configuration_generation = capture.configuration_generation;
  key.configuration_id = capture.configuration_key;
  // TubeMapCaptureKey carries the exact map frame string.  Its numeric frame
  // attribution reuses the captured configuration identity; it is not a
  // registry handle and cannot outlive/rebind to another frame.
  key.frame_provenance_id = capture.configuration_key;
  key.frame_provenance = capture.frame_id;
  // The accepted support evidence sequence is the producer-owned support
  // cohort identity.  Keep it direct so cropped/full views agree.
  key.support_provenance_id = capture.support.evidence_sequence;
  key.accepted_time_ticks = capture.accepted_time_ticks;
  key.support_expiry_timeless =
      capture.support.valid_until_accepted_ticks == 0U &&
      capture.support.valid_until.isZero();
  key.support_expiry_ticks =
      key.support_expiry_timeless ? 0U
                                  : capture.support.valid_until_accepted_ticks;
  key.support_halo = finite(capture.support.required_halo)
      ? capture.support.required_halo : 0.0;
  key.halo_reconciled = capture.support.halo_reconciled;
  key.grid_min_index_x = 0;
  key.grid_min_index_y = 0;
  key.grid_min_index_z = 0;
  key.grid_max_index_x =
      capture.voxel_count.x() > 0
      ? static_cast<std::int64_t>(capture.voxel_count.x()) - 1 : -1;
  key.grid_max_index_y =
      capture.voxel_count.y() > 0
      ? static_cast<std::int64_t>(capture.voxel_count.y()) - 1 : -1;
  key.grid_max_index_z =
      capture.voxel_count.z() > 0
      ? static_cast<std::int64_t>(capture.voxel_count.z()) - 1 : -1;
  key.grid_native_origin = capture.grid_origin;
  key.grid_voxel_resolution =
      Eigen::Vector3d::Constant(capture.resolution);
  key.grid_source_offset_x = capture.source_min_index.x();
  key.grid_source_offset_y = capture.source_min_index.y();
  key.grid_source_offset_z = capture.source_min_index.z();
  // Predicate ranges are capture-local.  The native grid origin and integer
  // source offsets are retained separately, so no rebasing is needed.
  key.grid_native_index = false;
  key.complete_support = status.usable;
}

void fillCommonResultMetadata(
    const SDFMapCaptureQueryDescriptorV2& descriptor,
    phase_offset_navigation::TubeFreeBallQueryResult& result) {
  const phase_offset_navigation::TubeMapCaptureKey& key =
      descriptor.map_capture_key;
  result.map_instance_id = key.map_instance_id;
  result.map_state_id = key.state_id;
  result.configuration_id = key.configuration_id;
  result.frame_provenance_id = key.frame_provenance_id;
  result.frame_provenance = key.frame_provenance;
  result.accepted_sequence = key.accepted_sequence;
  result.configuration_generation = key.configuration_generation;
  result.support_provenance_id = key.support_provenance_id;
  result.accepted_time_ticks = key.accepted_time_ticks;
  result.support_expiry_ticks = key.support_expiry_ticks;
  result.support_expiry_timeless = key.support_expiry_timeless;
  result.halo_reconciled = key.halo_reconciled;
  result.provenance = std::string("SDFMapCaptureV2/closed-inflated-voxel-volume/") +
      descriptor.frame_id;
}

phase_offset_navigation::DistanceStatus translateFreeBallStatus(
    const plan_env::SDFMapCaptureFreeBallStatusV2 status) {
  using phase_offset_navigation::DistanceStatus;
  switch (status) {
    case plan_env::SDFMapCaptureFreeBallStatusV2::OUT_OF_MAP:
      return DistanceStatus::OUT_OF_MAP;
    case plan_env::SDFMapCaptureFreeBallStatusV2::OCCUPIED:
      return DistanceStatus::OCCUPIED;
    case plan_env::SDFMapCaptureFreeBallStatusV2::UNKNOWN:
      // Navigation has no INCONCLUSIVE category.  Preserve the fail-closed
      // meaning instead of claiming free, occupied, or an exact distance.
      return DistanceStatus::UNKNOWN;
    case plan_env::SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE:
      // INCONCLUSIVE denotes an unresolved numerical/coverage predicate, not
      // an observed unknown voxel.  Keep that attribution as UNAVAILABLE.
      return DistanceStatus::UNAVAILABLE;
    case plan_env::SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE:
      return DistanceStatus::KNOWN_FREE;
    case plan_env::SDFMapCaptureFreeBallStatusV2::UNAVAILABLE:
      return DistanceStatus::UNAVAILABLE;
  }
  return DistanceStatus::UNAVAILABLE;
}

bool validCertifiedFootprint(
    const plan_env::SDFMapCaptureV2& capture,
    const plan_env::SDFMapCaptureFreeBallResultV2& native) {
  const Eigen::Vector3i lower = native.enumerated_min_index;
  const Eigen::Vector3i upper = native.enumerated_max_index;
  return (lower.array() >= 0).all() &&
      (upper.array() >= lower.array()).all() &&
      (upper.array() < capture.voxel_count.array()).all();
}

void fillCertifiedSupport(
    const plan_env::SDFMapCaptureV2& capture,
    const SDFMapCaptureQueryDescriptorV2& descriptor,
    const Eigen::Vector3d& witness,
    const double radius,
    const plan_env::SDFMapCaptureFreeBallResultV2& native,
    phase_offset_navigation::TubeFreeBallQueryResult& result) {
  result.certified_radius = native.certified_radius;
  result.certified = true;
  // The predicate certifies the requested lower-bound radius.  It does not
  // compute an exact nearest occupied-voxel distance.
  result.exact = false;
  result.complete_support = true;
  result.support.lower = native.support_min;
  result.support.upper = native.support_max;
  result.support.witness = witness;
  result.support.radius = radius;
  result.support.map_instance_id = descriptor.map_capture_key.map_instance_id;
  result.support.map_state_id = descriptor.map_capture_key.state_id;
  result.support.support_provenance_id =
      descriptor.map_capture_key.support_provenance_id;
  result.support.accepted_time_ticks =
      descriptor.map_capture_key.accepted_time_ticks;
  result.support.support_expiry_ticks =
      descriptor.map_capture_key.support_expiry_ticks;
  result.support.support_expiry_timeless =
      descriptor.map_capture_key.support_expiry_timeless;
  result.support.valid = true;

  phase_offset_navigation::TubeVoxelFootprintV2 footprint;
  footprint.min_index_x = native.enumerated_min_index.x();
  footprint.min_index_y = native.enumerated_min_index.y();
  footprint.min_index_z = native.enumerated_min_index.z();
  footprint.max_index_x = native.enumerated_max_index.x();
  footprint.max_index_y = native.enumerated_max_index.y();
  footprint.max_index_z = native.enumerated_max_index.z();
  footprint.native_origin = capture.grid_origin;
  footprint.voxel_resolution =
      Eigen::Vector3d::Constant(capture.resolution);
  footprint.source_offset_x = capture.source_min_index.x();
  footprint.source_offset_y = capture.source_min_index.y();
  footprint.source_offset_z = capture.source_min_index.z();
  // The predicate's enumerated range is indexed from the crop's local
  // source_min_index.  Preserve that offset instead of pretending it is a
  // native index range.
  footprint.native_index = false;
  footprint.valid = true;
  result.support.voxel_footprint.push_back(footprint);
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

SDFMapCaptureQueryStatusV2 inspectSDFMapCaptureQueryV2(
    const std::shared_ptr<const plan_env::SDFMapCaptureV2>& capture) {
  SDFMapCaptureQueryStatusV2 status;
  status.closed_inflated_voxel_volume_metric = true;
  status.floating_point_environment_supported =
      plan_env::sdfMapCaptureV2FloatingPointEnvironmentSupported();
  status.capture_available = static_cast<bool>(capture);
  if (!capture) return status;

  status.map_instance_id = capture->map_instance_id;
  status.map_state_id = capture->accepted_state_sequence;
  status.accepted_sequence = capture->accepted_state_sequence;
  status.accepted_state_notification_sequence =
      capture->accepted_state_notification_sequence;
  status.configuration_generation = capture->configuration_generation;
  status.configuration_key = capture->configuration_key;
  status.support_sequence = capture->support.evidence_sequence;
  status.support_evidence_accepted_ticks =
      capture->support.evidence_accepted_ticks;
  status.accepted_time_ticks = capture->accepted_time_ticks;
  status.support_expiry_ticks =
      capture->support.valid_until_accepted_ticks;
  status.support_expiry_timeless =
      capture->support.valid_until_accepted_ticks == 0U &&
      capture->support.valid_until.isZero();
  status.frame_id = capture->frame_id;

  status.identity_valid = capture->map_instance_id != 0U &&
      capture->accepted_state_sequence != 0U &&
      capture->accepted_time_ticks != 0U;
  status.configuration_valid = capture->configuration_generation != 0U &&
      capture->configuration_key != 0U;
  status.frame_valid = !capture->frame_id.empty() &&
      capture->support.frame_id == capture->frame_id;
  status.geometry_valid = captureGeometryValid(*capture);
  status.support_valid = capture->support.valid;
  status.support_complete = capture->support.complete;
  status.support_metadata_valid = supportMetadataValid(*capture);
  status.expiry_metadata_valid = supportExpiryMetadataValid(*capture);
  status.capture_consistent = plan_env::sdfMapCaptureV2Consistent(*capture);
  status.usable = status.floating_point_environment_supported &&
      status.capture_available && status.capture_consistent &&
      status.identity_valid && status.configuration_valid &&
      status.frame_valid && status.geometry_valid && status.support_valid &&
      status.support_complete && status.support_metadata_valid &&
      status.expiry_metadata_valid;
  return status;
}

SDFMapCaptureQueryBridgeV2 makeSDFMapCaptureQueryV2(
    const std::shared_ptr<const plan_env::SDFMapCaptureV2>& capture) {
  SDFMapCaptureQueryBridgeV2 bridge;
  bridge.status = inspectSDFMapCaptureQueryV2(capture);
  if (capture) {
    bridge.descriptor.capture = capture;
    bridge.descriptor.frame_id = capture->frame_id;
    bridge.descriptor.observation_stamp = capture->observation_stamp;
    bridge.descriptor.accepted_state_stamp = capture->accepted_state_stamp;
    bridge.descriptor.accepted_state_notification_sequence =
        capture->accepted_state_notification_sequence;
    bridge.descriptor.support_sequence = capture->support.evidence_sequence;
    bridge.descriptor.support_evidence_accepted_ticks =
        capture->support.evidence_accepted_ticks;
    bridge.descriptor.support_evidence_stamp =
        capture->support.evidence_stamp;
    bridge.descriptor.support_valid_until = capture->support.valid_until;
    bridge.descriptor.support_valid_until_ticks =
        capture->support.valid_until_accepted_ticks;
    bridge.descriptor.support_expiry_timeless =
        bridge.status.support_expiry_timeless;
    bridge.descriptor.support_evidence_basis =
        capture->support.evidence_basis;
    bridge.descriptor.support_complete = capture->support.complete;
    bridge.descriptor.map_min = capture->map_min;
    bridge.descriptor.map_max = capture->map_max;
    bridge.descriptor.support_min = capture->support.support_min;
    bridge.descriptor.support_max = capture->support.support_max;
    bridge.descriptor.required_halo = capture->support.required_halo;
    bridge.descriptor.halo_reconciled = capture->support.halo_reconciled;
    bridge.descriptor.grid_origin = capture->grid_origin;
    bridge.descriptor.resolution = capture->resolution;
    bridge.descriptor.source_min_index = capture->source_min_index;
    bridge.descriptor.source_max_index = capture->source_max_index;
    bridge.descriptor.capture_min = capture->capture_min;
    bridge.descriptor.capture_max = capture->capture_max;
    bridge.descriptor.effective_layer_mask = capture->effective_layer_mask;
    bridge.descriptor.included_map_inflation = capture->included_map_inflation;
    bridge.descriptor.closed_inflated_voxel_volume_metric = true;
    bridge.descriptor.clearance_metric = "closed_inflated_voxel_volume";
    fillMapCaptureKey(*capture, bridge.status,
                      bridge.descriptor.map_capture_key);
    bridge.capture_owner = capture;
    // There is no second query object or mutable map.  Both owner slots keep
    // the exact immutable capture alive for the worker/build DTO contract.
    bridge.query_owner = capture;
    bridge.descriptor.valid = bridge.status.usable;
  }

  const SDFMapCaptureQueryStatusV2 status = bridge.status;
  const SDFMapCaptureQueryDescriptorV2 descriptor = bridge.descriptor;
  bridge.free_ball_query = [capture, status, descriptor](
      const Eigen::Vector3d& witness,
      const double requested_radius) {
    phase_offset_navigation::TubeFreeBallQueryResult result;
    if (!status.usable || !capture || !finite(witness) ||
        !finite(requested_radius) || requested_radius < 0.0) {
      return result;
    }
    fillCommonResultMetadata(descriptor, result);

    const plan_env::SDFMapCaptureFreeBallResultV2 native =
        plan_env::certifySDFMapCaptureFreeBallV2(
            *capture, witness, requested_radius);
    result.status = translateFreeBallStatus(native.status);
    if (native.status ==
        plan_env::SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE) {
      result.provenance += "/inconclusive";
    }
    if (native.status != plan_env::SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE ||
        !native.clearance_certified ||
        native.certified_radius != requested_radius ||
        !finite(native.certified_radius) ||
        !finite(native.support_min) || !finite(native.support_max) ||
        !validCertifiedFootprint(*capture, native)) {
      // INCONCLUSIVE and malformed producer answers remain uncertified.  In
      // particular, never turn a failed exact comparison into KNOWN_FREE or
      // expose a fabricated nearest-distance value.
      if (native.status ==
          plan_env::SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE) {
        result.status = phase_offset_navigation::DistanceStatus::UNAVAILABLE;
        result.provenance += "/malformed";
      }
      return result;
    }
    fillCertifiedSupport(*capture, descriptor, witness, requested_radius,
                         native, result);
    return result;
  };
  return bridge;
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
