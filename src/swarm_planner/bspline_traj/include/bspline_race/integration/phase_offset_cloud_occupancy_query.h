#pragma once

#include <plan_env/cloud_occupancy_snapshot.h>
#include <phase_offset_navigation/tube_epoch_types.h>
#include <phase_offset_navigation/tube_certificate_v2.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace FLAG_Race {

// The local occupycloud contract is deliberately explicit.  A sparse hit-only
// cloud must leave this disabled and therefore fail closed; the original
// local_sensing simulator is enabled because it publishes a complete local
// obstacle set for its AABB.
struct CloudOccupancyQueryConfig {
  bool obstacle_set_complete = false;
  double required_preincluded_map_uncertainty = 0.0;
};

struct CloudOccupancyQueryStatus {
  bool configuration_valid = false;
  bool obstacle_set_complete = false;
  bool snapshot_available = false;
  bool snapshot_valid = false;
  bool included_map_inflation_sufficient = false;
  bool usable = false;
  std::uint64_t observation_sequence = 0U;
  ros::Time observation_stamp;
  double included_map_inflation = 0.0;
};

bool cloudOccupancyQueryConfigurationValid(
    const CloudOccupancyQueryConfig& config);
CloudOccupancyQueryStatus inspectCloudOccupancyQuery(
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    const CloudOccupancyQueryConfig& config);

// Captures exactly one immutable snapshot.  It never reads SDFMap, raw log
// odds, ESDF, a self-free seed, or any mutable occupancy buffer.
phase_offset_navigation::RawOccupancyQuery makeCloudOccupancyQuery(
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    const CloudOccupancyQueryConfig& config);

// G2g production bridge.  Captures the same immutable snapshot as the
// diagnostics query and maps its planner-ESDF-base occupied-centre clearance
// contract to navigation without exposing ROS, PointCloud2, or plan_env
// there.
phase_offset_navigation::ClearanceQuery makeCloudOccupancyClearanceQuery(
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    const CloudOccupancyQueryConfig& config);

// V2 is deliberately a separate bridge from the legacy cloud snapshot
// adapters above.  It accepts exactly one immutable authoritative SDFMap
// capture.  No legacy completeness flag or single-frame cloud observation is
// consulted by this contract.
struct SDFMapCaptureQueryStatusV2 {
  bool floating_point_environment_supported = false;
  bool capture_available = false;
  bool capture_consistent = false;
  bool identity_valid = false;
  bool configuration_valid = false;
  bool frame_valid = false;
  bool geometry_valid = false;
  bool support_valid = false;
  bool support_complete = false;
  bool support_metadata_valid = false;
  bool expiry_metadata_valid = false;
  // V2's certified predicate is a closed inflated-voxel-volume predicate;
  // this attribution is intentionally separate from legacy clearance fields.
  bool closed_inflated_voxel_volume_metric = false;
  bool usable = false;

  std::uint64_t map_instance_id = 0U;
  std::uint64_t map_state_id = 0U;
  std::uint64_t accepted_sequence = 0U;
  std::uint64_t accepted_state_notification_sequence = 0U;
  std::uint64_t configuration_generation = 0U;
  std::uint64_t configuration_key = 0U;
  std::uint64_t support_sequence = 0U;
  std::uint64_t support_evidence_accepted_ticks = 0U;
  std::uint64_t accepted_time_ticks = 0U;
  std::uint64_t support_expiry_ticks = 0U;
  bool support_expiry_timeless = false;
  std::string frame_id;
};

// Immutable value metadata retained alongside a V2 query.  `capture` is the
// same const object held by the callback and by `capture_owner`; it is exposed
// only at this ROS-facing integration seam and is never read through a
// mutable SDFMap pointer.
struct SDFMapCaptureQueryDescriptorV2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  phase_offset_navigation::TubeMapCaptureKey map_capture_key;
  std::string frame_id;
  ros::Time observation_stamp;
  ros::Time accepted_state_stamp;
  std::uint64_t accepted_state_notification_sequence = 0U;
  std::uint64_t support_sequence = 0U;
  std::uint64_t support_evidence_accepted_ticks = 0U;
  ros::Time support_evidence_stamp;
  ros::Time support_valid_until;
  std::uint64_t support_valid_until_ticks = 0U;
  bool support_expiry_timeless = false;
  std::uint32_t support_evidence_basis = 0U;
  bool support_complete = false;
  Eigen::Vector3d map_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d map_max = Eigen::Vector3d::Zero();
  Eigen::Vector3d support_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d support_max = Eigen::Vector3d::Zero();
  double required_halo = 0.0;
  bool halo_reconciled = false;
  Eigen::Vector3d grid_origin = Eigen::Vector3d::Zero();
  double resolution = 0.0;
  Eigen::Vector3i source_min_index = Eigen::Vector3i::Zero();
  Eigen::Vector3i source_max_index = Eigen::Vector3i::Constant(-1);
  Eigen::Vector3d capture_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d capture_max = Eigen::Vector3d::Zero();
  std::uint32_t effective_layer_mask = 0U;
  double included_map_inflation = 0.0;
  bool closed_inflated_voxel_volume_metric = false;
  std::string clearance_metric;
  std::shared_ptr<const plan_env::SDFMapCaptureV2> capture;
  bool valid = false;
};

// One immutable capture/query cohort.  `capture_owner` and `query_owner`
// intentionally retain the same immutable capture so a caller may release
// its input shared_ptr immediately after construction without invalidating a
// worker-side query or descriptor.
struct SDFMapCaptureQueryBridgeV2 {
  SDFMapCaptureQueryStatusV2 status;
  SDFMapCaptureQueryDescriptorV2 descriptor;
  phase_offset_navigation::TubeFreeBallQuery free_ball_query;
  std::shared_ptr<const void> capture_owner;
  std::shared_ptr<const void> query_owner;

  bool usable() const { return status.usable && static_cast<bool>(free_ball_query); }
};

SDFMapCaptureQueryStatusV2 inspectSDFMapCaptureQueryV2(
    const std::shared_ptr<const plan_env::SDFMapCaptureV2>& capture);

SDFMapCaptureQueryBridgeV2 makeSDFMapCaptureQueryV2(
    const std::shared_ptr<const plan_env::SDFMapCaptureV2>& capture);

// Independent schema: existing 83 / 49 / 49 payloads retain their positions
// and meanings.  It records the cloud-contract and margin-accounting facts
// used for the candidate build captured by this adapter cycle.
enum CloudSnapshotDiagnosticIndex : std::size_t {
  kCloudSnapshotSchemaVersion = 0U,
  kCloudSnapshotObstacleSetComplete,
  kCloudSnapshotQueryConfigurationValid,
  kCloudSnapshotAvailable,
  kCloudSnapshotValid,
  kCloudSnapshotUsable,
  kCloudSnapshotObservationSequence,
  kCloudSnapshotObservationStamp,
  kCloudSnapshotIncludedMapInflation,
  kCloudSnapshotConfiguredPreincludedMapUncertainty,
  kCloudSnapshotFullEffectiveRadius,
  kCloudSnapshotResidualEffectiveRadius,
  kCloudSnapshotCurrentBaseStatus,
  kCloudSnapshotActualStatus,
  kCloudSnapshotCurrentPlusStepStatus,
  kCloudSnapshotCurrentMinusStepStatus,
  kCloudSnapshotRawStorageAccessed,
  kCloudSnapshotSelfFreeSeedUsed,
  kCloudSnapshotDiagnosticCount,
};

static_assert(kCloudSnapshotDiagnosticCount == 18U,
              "cloud snapshot diagnostics schema must remain exactly 18 fields");

struct CloudSnapshotDiagnosticsInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CloudOccupancyQueryStatus query_status;
  CloudOccupancyQueryConfig query_config;
  phase_offset_navigation::RobustTubeMargins margins;
  const phase_offset_navigation::TubeProfile* candidate_profile = nullptr;
  double current_w = 0.0;
  phase_offset_core::PathDifferentialState current_path;
  Eigen::Vector3d actual_position = Eigen::Vector3d::Zero();
  phase_offset_navigation::RawOccupancyQuery categorical_query;
  double ray_step = 0.0;
};

const std::array<const char*, kCloudSnapshotDiagnosticCount>&
cloudSnapshotDiagnosticFieldNames();
std::array<double, kCloudSnapshotDiagnosticCount>
makeCloudSnapshotDiagnostics(const CloudSnapshotDiagnosticsInput& input);

}  // namespace FLAG_Race
