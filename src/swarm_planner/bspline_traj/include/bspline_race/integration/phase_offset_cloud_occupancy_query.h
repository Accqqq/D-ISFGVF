#pragma once

#include <plan_env/cloud_occupancy_snapshot.h>
#include <phase_offset_navigation/tube_epoch_types.h>

#include <array>
#include <cstddef>
#include <memory>

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
// diagnostics query and maps its occupied-voxel-volume clearance contract to
// navigation without exposing ROS, PointCloud2, or plan_env there.
phase_offset_navigation::ClearanceQuery makeCloudOccupancyClearanceQuery(
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    const CloudOccupancyQueryConfig& config);

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
