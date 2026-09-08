#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <ros/time.h>
#include <string>
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
struct CloudOccupancySnapshotPlannerEsdfBaseClearanceResult;
struct SDFMapCaptureRegionV2;
struct SDFMapCaptureSupportV2;
struct SDFMapCaptureV2;
struct SDFMapCaptureFreeBallResultV2;

CloudOccupancySnapshot buildCloudOccupancySnapshot(
    const CloudOccupancySnapshotBuildInput& input);
CloudOccupancySnapshotClearanceResult
queryCloudOccupancySnapshotClearance(
    const CloudOccupancySnapshot& snapshot,
    const Eigen::Vector3d& point,
    double required_radius);
CloudOccupancySnapshotPlannerEsdfBaseClearanceResult
queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
    const CloudOccupancySnapshot& snapshot,
    const Eigen::Vector3d& point,
    double required_radius);

struct CloudOccupancySnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  std::uint64_t observation_sequence = 0U;
  ros::Time observation_stamp;
  // Effective planner-admissible domain.  This is the native SDFMap base
  // domain intersected with an enabled manual boundary; grid_origin and the
  // integer source offsets below remain the native backing geometry.
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
  friend CloudOccupancySnapshotPlannerEsdfBaseClearanceResult
  queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
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

// A planner-ESDF-base clearance result.  The distance is from `point` to the
// nearest centre of an inflated occupied voxel in the immutable snapshot,
// matching the occupied-centre geometry that seeds SDFMap's positive EDT.
// The result is capped at the requested radius and `clearance_certified`
// proves the returned lower bound.  This sibling contract intentionally keeps
// the closed-volume primitive above unchanged.
struct CloudOccupancySnapshotPlannerEsdfBaseClearanceResult {
  CloudOccupancyStatus status = CloudOccupancyStatus::UNAVAILABLE;
  double nearest_inflated_occupied_voxel_center_distance = 0.0;
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

// Queries Euclidean clearance against the centres of inflated occupied voxels
// in one immutable snapshot.  This is the planner-ESDF-base contract: at a
// free grid centre it is the same centre-set distance used by SDFMap's
// positive EDT.  Domain, completeness, bounded-work, and fail-closed
// semantics match queryCloudOccupancySnapshotClearance(); only the geometric
// primitive differs.
CloudOccupancySnapshotPlannerEsdfBaseClearanceResult
queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
    const CloudOccupancySnapshot& snapshot,
    const Eigen::Vector3d& point,
    double required_radius);

// Evidence basis for the authoritative SDFMap support carried by a V2
// capture.  A caller cannot make support complete by setting a legacy
// cloud-completeness flag; the producer must attribute it to an actual
// observation or a pre-known complete domain.
enum SDFMapCaptureSupportEvidenceBasisV2 : std::uint32_t {
  kSDFMapCaptureSupportEvidenceNone = 0U,
  kSDFMapCaptureSupportEvidenceDepthRaycast = 1U << 0,
  kSDFMapCaptureSupportEvidenceCompletePreknownDomain = 1U << 1,
};

enum SDFMapCaptureEffectiveLayerV2 : std::uint32_t {
  kSDFMapCaptureLayerSensor = 1U << 0,
  kSDFMapCaptureLayerManual = 1U << 1,
  kSDFMapCaptureLayerStatic = 1U << 2,
  kSDFMapCaptureLayerCeiling = 1U << 3,
};

// Optional requested map-space region.  The capture includes every grid voxel
// whose closed volume intersects the region expanded by halo.  An invalid or
// omitted region asks for the complete map, which is useful for small unit
// fixtures and remains one value copied from SDFMap.
struct SDFMapCaptureRegionV2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
  double halo = 0.0;
};

// Minimum valid support/completeness evidence accompanying one authoritative
// map value.  The dense mask is indexed exactly like SDFMapCaptureV2::occupied
// and is copied at the same serialization boundary.  `valid` is intentionally
// false by default; zero occupancy/allocation and legacy cloud flags never
// promote it.
struct SDFMapCaptureSupportV2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  bool complete = false;
  std::uint32_t evidence_basis = kSDFMapCaptureSupportEvidenceNone;
  std::uint64_t evidence_sequence = 0U;
  std::uint64_t evidence_accepted_ticks = 0U;
  std::uint64_t map_instance_id = 0U;
  std::uint64_t configuration_generation = 0U;
  std::uint64_t configuration_key = 0U;
  std::string frame_id;
  ros::Time evidence_stamp;
  ros::Time valid_until;
  // A nonzero finite expiry must be grounded in the same monotonic acceptance
  // tick domain as SDFMapCaptureV2::accepted_time_ticks.  Zero means an
  // explicitly timeless preknown domain, not an unknown expiry.
  std::uint64_t valid_until_accepted_ticks = 0U;
  Eigen::Vector3d support_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d support_max = Eigen::Vector3d::Zero();
  double required_halo = 0.0;
  bool halo_reconciled = false;
  std::vector<std::uint8_t> mask;
};

// Immutable read-only value of the existing authoritative SDFMap.  The
// occupied vector is the effective accumulated/inflated planner backing after
// enabled manual/static/ceiling layers have been applied; no cloud
// reconstruction or second map lifecycle is represented here.
struct SDFMapCaptureV2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  std::uint64_t map_instance_id = 0U;
  std::uint64_t configuration_generation = 0U;
  std::uint64_t configuration_key = 0U;
  std::string frame_id;
  std::uint64_t accepted_state_sequence = 0U;
  // Monotonic notification fact for accepted authoritative mutations.  It is
  // copied with the state under SDFMap's serialization boundary and is never
  // advanced by raw cloud arrival or by a consumer reading this value.
  std::uint64_t accepted_state_notification_sequence = 0U;
  // Monotonic steady-clock acceptance metadata.  It is separate from the
  // source observation stamp below and never synthesized from ros::Time::now.
  std::uint64_t accepted_time_ticks = 0U;
  ros::Time observation_stamp;
  ros::Time accepted_state_stamp;
  std::uint32_t effective_layer_mask = 0U;
  Eigen::Vector3d map_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d map_max = Eigen::Vector3d::Zero();
  // Native SDFMap grid origin.  Cropped captures retain this value and use
  // source_min_index/source_max_index as integer offsets into the native
  // grid; this is never a rounded/rebased crop origin.
  Eigen::Vector3d grid_origin = Eigen::Vector3d::Zero();
  // Directed clipped request bounds in map coordinates.  The occupied vector
  // contains every native voxel whose closed volume intersects these bounds.
  Eigen::Vector3d capture_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d capture_max = Eigen::Vector3d::Zero();
  Eigen::Vector3i source_min_index = Eigen::Vector3i::Zero();
  Eigen::Vector3i source_max_index = Eigen::Vector3i::Constant(-1);
  Eigen::Vector3i voxel_count = Eigen::Vector3i::Zero();
  double resolution = 0.0;
  double included_map_inflation = 0.0;
  SDFMapCaptureSupportV2 support;
  std::vector<std::uint8_t> occupied;
};

// Bounded value-only visibility of the latest accepted authoritative map
// mutation.  This DTO intentionally carries identity/support metadata but no
// mutable SDFMap pointer or callback.  A zero/default value means that no
// accepted state has been published yet.
struct SDFMapAcceptedStateVisibilityV2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid = false;
  std::uint64_t map_instance_id = 0U;
  std::uint64_t accepted_state_sequence = 0U;
  std::uint64_t accepted_state_notification_sequence = 0U;
  std::uint64_t accepted_time_ticks = 0U;
  std::uint64_t configuration_generation = 0U;
  std::uint64_t configuration_key = 0U;
  std::string frame_id;
  ros::Time accepted_state_stamp;
  SDFMapCaptureSupportV2 support;
};

enum class SDFMapCaptureFreeBallStatusV2 {
  UNAVAILABLE,
  OUT_OF_MAP,
  UNKNOWN,
  OCCUPIED,
  INCONCLUSIVE,
  CERTIFIED_FREE,
};

// Result of the pure closed-volume free-ball predicate.  It is intentionally
// convertible to bool for small callers while retaining typed fail-closed
// status and support/work evidence for V2 diagnostics.
struct SDFMapCaptureFreeBallResultV2 {
  SDFMapCaptureFreeBallStatusV2 status =
      SDFMapCaptureFreeBallStatusV2::UNAVAILABLE;
  bool clearance_certified = false;
  double certified_radius = 0.0;
  std::size_t voxel_checks = 0U;
  Eigen::Vector3d support_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d support_max = Eigen::Vector3d::Zero();
  // Capture-local inclusive range of every voxel enumerated as potentially
  // touched by the closed query ball.  Constant(-1) denotes that no bounded
  // voxel range was established (for example, out-of-domain input).
  Eigen::Vector3i enumerated_min_index = Eigen::Vector3i::Zero();
  Eigen::Vector3i enumerated_max_index = Eigen::Vector3i::Constant(-1);

  explicit operator bool() const { return clearance_certified; }
};

// Short aliases keep the DTO ergonomic without creating a second contract.
using SDFMapCaptureFreeBallResult = SDFMapCaptureFreeBallResultV2;
using SDFMapCaptureFreeBallStatus = SDFMapCaptureFreeBallStatusV2;

// Pure predicate over one immutable authoritative capture.  It certifies a
// complete closed Euclidean ball against closed occupied voxel volumes and a
// complete attributed support mask.  Any unsupported, nonfinite, overflowed,
// or unresolved comparison is fail-closed.
SDFMapCaptureFreeBallResultV2 certifySDFMapCaptureFreeBallV2(
    const SDFMapCaptureV2& capture,
    const Eigen::Vector3d& qhat,
    double radius);

// Structural validation for manually assembled/captured V2 values.  Support
// validity is checked separately by the predicate and is never inferred from
// an allocated or all-zero occupied vector.
bool sdfMapCaptureV2FloatingPointEnvironmentSupported();
bool sdfMapCaptureV2Consistent(const SDFMapCaptureV2& capture);

}  // namespace plan_env
