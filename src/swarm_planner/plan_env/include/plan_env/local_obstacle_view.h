#ifndef PLAN_ENV_LOCAL_OBSTACLE_VIEW_H
#define PLAN_ENV_LOCAL_OBSTACLE_VIEW_H

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace plan_env {

// Closed axis-aligned world-coordinate box.  A box may be degenerate in a
// request/reference domain, while map and voxel boxes are validated as
// non-empty by the implementation.
struct LocalObstacleBox {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
};

// A stable, coherent borrow supplied by the caller.  The pointed-to buffers
// must not be modified for the duration of buildLocalObstacleView(); this
// interface deliberately has no lock or "coherent" flag of its own.
struct LocalObstacleGridView {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3i voxel_count = Eigen::Vector3i::Zero();
  double resolution = 0.0;
  LocalObstacleBox map_bounds;
  std::string frame_id;

  const std::vector<char>* inflated = nullptr;
  const std::vector<char>* manual = nullptr;
  const std::vector<char>* static_layer = nullptr;
};

struct LocalObstacleRequest {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalObstacleBox reference_region;
  double clearance = 0.0;
  LocalObstacleBox known_region;
  bool known_region_valid = false;
  std::size_t max_voxel_checks = 200000U;
  std::size_t max_occupied_voxels = 100000U;
};

enum class LocalObstacleViewStatus {
  VALID = 0,
  INVALID_INPUT,
  UNKNOWN_DOMAIN,
  OUT_OF_DOMAIN,
  BUDGET_EXCEEDED
};

struct LocalObstacleVoxel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3i index = Eigen::Vector3i::Zero();
  LocalObstacleBox bounds;
  // bit0=inflated, bit1=manual, bit2=static_layer.
  std::uint8_t layer_mask = 0U;
};

struct LocalObstacleView {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // VALID means that this local obstacle information and its query domain
  // were completely enumerated.  It does not mean reference_domain is free
  // of collision: obstacles may be non-empty, and a downstream builder must
  // still avoid them and enforce its requested clearance.
  LocalObstacleViewStatus status = LocalObstacleViewStatus::INVALID_INPUT;
  std::string frame_id;
  double resolution = 0.0;
  // Native grid geometry of the map that produced this view.  Downstream
  // consumers that want a direct voxel lookup (rather than iterating the
  // occupied boxes) need the same index arithmetic the map uses.
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3i voxel_count = Eigen::Vector3i::Zero();
  LocalObstacleBox obstacle_region;
  // The domain on which a downstream query may rely on this view; it is not a
  // declaration of free space or a collision-free reference set.
  LocalObstacleBox reference_domain;
  bool request_clipped = false;
  std::size_t visited_voxels = 0U;
  std::size_t occupied_voxels = 0U;
  std::vector<LocalObstacleVoxel,
              Eigen::aligned_allocator<LocalObstacleVoxel>> obstacles;
};

// Map-owned validity for a static environment assumption.  The flag is
// intentionally not an identity, revision, or timestamp.  Writers and
// readers of valid must hold the shared environment_change_mutex carried by
// LocalObstacleCapture/SectionPathBundle.  A capture keeps the state alive so
// a later real map change can invalidate the exact observation that was used
// to build it without relying on an ABA-prone integer token.
struct EnvironmentValidityState {
  bool valid = true;
};

// Value returned by a map-local read.  LocalObstacleView owns the copied
// voxel boxes; the shared state/mutex are only the small publication boundary
// used to reject a bundle after a true static-environment change.  This type
// deliberately stays in plan_env and must not cross into phase_offset_core or
// the navigation mathematics layer.
struct LocalObstacleCapture {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalObstacleView view;
  std::shared_ptr<EnvironmentValidityState> environment_validity;
  std::shared_ptr<std::recursive_mutex> environment_change_mutex;
};

LocalObstacleView buildLocalObstacleView(
    const LocalObstacleGridView& grid,
    const LocalObstacleRequest& request);

}  // namespace plan_env

#endif  // PLAN_ENV_LOCAL_OBSTACLE_VIEW_H
