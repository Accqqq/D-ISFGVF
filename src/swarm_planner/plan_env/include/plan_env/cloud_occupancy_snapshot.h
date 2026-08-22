#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <ros/time.h>
#include <vector>

namespace plan_env {

// Categorical evidence from one complete local occupycloud observation.  This
// is deliberately separate from SDFMap's mutable planner buffers: a tube
// build holds one immutable value while cloud callbacks continue updating the
// planner's inflated occupancy / ESDF backing.
enum class CloudOccupancyStatus {
  UNAVAILABLE,
  OUT_OF_MAP,
  UNKNOWN,
  KNOWN_FREE,
  OCCUPIED,
};

struct CloudOccupancyColumnIndex;
struct CloudOccupancySnapshot;
struct CloudOccupancySnapshotBuildInput;
struct CloudOccupancySnapshotClearanceResult;

CloudOccupancySnapshot buildCloudOccupancySnapshot(
    const CloudOccupancySnapshotBuildInput& input);
CloudOccupancySnapshotClearanceResult
queryCloudOccupancySnapshotClearance(
    const CloudOccupancySnapshot& snapshot,
    const Eigen::Vector3d& point,
    double required_radius);

struct CloudOccupancySnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  std::uint64_t observation_sequence = 0U;
  ros::Time observation_stamp;
  Eigen::Vector3d map_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d map_max = Eigen::Vector3d::Zero();
  Eigen::Vector3d observed_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d observed_max = Eigen::Vector3d::Zero();
  Eigen::Vector3d grid_origin = Eigen::Vector3d::Zero();
  Eigen::Vector3i voxel_count = Eigen::Vector3i::Zero();
  double resolution = 0.0;
  // The distance represented by the cloud voxel inflation already included
  // in occupied.  It is environmental-map evidence only, never a planner or
  // controller safety setting.
  double included_map_inflation = 0.0;
  std::vector<std::uint8_t> occupied;

 private:
  // Optional exact acceleration for clearance queries.  Its implementation is
  // opaque, builder-only, and private, so the public dense vector remains
  // authoritative.  Manually constructed snapshots leave this null and use
  // the dense path.
  std::shared_ptr<const CloudOccupancyColumnIndex> occupied_column_index;

  friend CloudOccupancySnapshot buildCloudOccupancySnapshot(
      const CloudOccupancySnapshotBuildInput& input);
  friend CloudOccupancySnapshotClearanceResult
  queryCloudOccupancySnapshotClearance(
      const CloudOccupancySnapshot& snapshot,
      const Eigen::Vector3d& point,
      double required_radius);
};

struct CloudOccupancySnapshotBuildInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool odom_valid = false;
  std::uint64_t observation_sequence = 0U;
  ros::Time observation_stamp;
  Eigen::Vector3d camera_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d map_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d map_max = Eigen::Vector3d::Zero();
  Eigen::Vector3d grid_origin = Eigen::Vector3d::Zero();
  Eigen::Vector3i voxel_count = Eigen::Vector3i::Zero();
  Eigen::Vector3d local_update_range = Eigen::Vector3d::Zero();
  double resolution = 0.0;
  double obstacles_inflation = 0.0;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      cloud_points;
};

struct CloudOccupancySnapshotQueryResult {
  CloudOccupancyStatus status = CloudOccupancyStatus::UNAVAILABLE;
};

// A conservative, immutable-snapshot clearance result.  The distance is from
// `point` to the closed AABB of the nearest occupied voxel, not to a voxel
// centre.  When no occupied voxel is found inside the requested radius, the
// result is capped at that radius and `clearance_certified` proves the lower
// bound.  Callers that need a larger lower bound must ask for that larger
// radius; this keeps the search local and deterministic.
struct CloudOccupancySnapshotClearanceResult {
  CloudOccupancyStatus status = CloudOccupancyStatus::UNAVAILABLE;
  double nearest_occupied_voxel_volume_distance = 0.0;
  bool clearance_certified = false;
};

// Validates all metadata and the exact occupied-vector size.  Invalid values
// are always queried fail-closed as UNAVAILABLE.
bool cloudOccupancySnapshotConsistent(const CloudOccupancySnapshot& snapshot);

// Builds an immutable occupycloud snapshot using the same voxel rounding and
// XY inflation convention as SDFMap::cloudCallback.  An empty cloud with a
// valid odom is therefore a valid all-free local observation.
CloudOccupancySnapshot buildCloudOccupancySnapshot(
    const CloudOccupancySnapshotBuildInput& input);

// Implements the cloud-map contract in one place:
// invalid -> UNAVAILABLE; global-map outside -> OUT_OF_MAP; outside this
// observation's local AABB -> UNKNOWN; otherwise inflated occupied/free.
CloudOccupancySnapshotQueryResult queryCloudOccupancySnapshot(
    const CloudOccupancySnapshot& snapshot,
    const Eigen::Vector3d& point);

// Queries Euclidean clearance against the occupied *voxel volumes* in one
// immutable snapshot.  The complete closed ball required for the query must
// lie inside the observed AABB; otherwise the result is UNKNOWN rather than
// treating unobserved space as free.  Invalid metadata, invalid radius, NaN,
// or arithmetic inconsistencies fail closed as UNAVAILABLE.
CloudOccupancySnapshotClearanceResult
queryCloudOccupancySnapshotClearance(
    const CloudOccupancySnapshot& snapshot,
    const Eigen::Vector3d& point,
    double required_radius);

}  // namespace plan_env
