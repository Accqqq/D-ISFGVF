#include "plan_env/cloud_occupancy_snapshot.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace plan_env {

struct CloudOccupancyColumnIndex {
  std::uint64_t observation_sequence = 0U;
  Eigen::Vector3i voxel_count = Eigen::Vector3i::Zero();
  std::size_t occupied_size = 0U;
  std::vector<std::size_t> offsets;
  std::vector<std::size_t> addresses;
};

namespace {

constexpr double kMapBoundaryTolerance = 1e-4;
constexpr std::size_t kMaxClearanceVoxelChecks = 4U * 1024U * 1024U;

bool finite(const double value) {
  return std::isfinite(value);
}

bool finite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

bool validColumnCount(const Eigen::Vector3i& voxel_count,
                      std::size_t& count) {
  if (voxel_count.x() <= 0 || voxel_count.y() <= 0) return false;
  const std::size_t x = static_cast<std::size_t>(voxel_count.x());
  const std::size_t y = static_cast<std::size_t>(voxel_count.y());
  if (x > std::numeric_limits<std::size_t>::max() / y) return false;
  count = x * y;
  return true;
}

bool validVoxelCount(const Eigen::Vector3i& voxel_count,
                     std::size_t& count) {
  if (voxel_count.z() <= 0) return false;
  std::size_t xy = 0U;
  if (!validColumnCount(voxel_count, xy)) return false;
  const std::size_t z = static_cast<std::size_t>(voxel_count.z());
  if (xy > std::numeric_limits<std::size_t>::max() / z) return false;
  count = xy * z;
  return true;
}

bool pointInMap(const Eigen::Vector3d& point,
                const Eigen::Vector3d& map_min,
                const Eigen::Vector3d& map_max) {
  return finite(point) && point.x() >= map_min.x() + kMapBoundaryTolerance &&
      point.y() >= map_min.y() + kMapBoundaryTolerance &&
      point.z() >= map_min.z() + kMapBoundaryTolerance &&
      point.x() <= map_max.x() - kMapBoundaryTolerance &&
      point.y() <= map_max.y() - kMapBoundaryTolerance &&
      point.z() <= map_max.z() - kMapBoundaryTolerance;
}

bool pointInObservedBox(const Eigen::Vector3d& point,
                        const Eigen::Vector3d& observed_min,
                        const Eigen::Vector3d& observed_max) {
  return point.x() >= observed_min.x() && point.y() >= observed_min.y() &&
      point.z() >= observed_min.z() && point.x() <= observed_max.x() &&
      point.y() <= observed_max.y() && point.z() <= observed_max.z();
}

bool pointToIndex(const Eigen::Vector3d& point,
                  const Eigen::Vector3d& grid_origin,
                  const Eigen::Vector3i& voxel_count,
                  const double resolution,
                  Eigen::Vector3i& index) {
  if (!finite(point) || !finite(grid_origin) || !finite(resolution) ||
      resolution <= 0.0) {
    return false;
  }
  const Eigen::Vector3d continuous = (point - grid_origin) / resolution;
  if (!finite(continuous)) return false;
  const Eigen::Vector3d floored = continuous.array().floor();
  const double int_min = static_cast<double>(std::numeric_limits<int>::min());
  const double int_max = static_cast<double>(std::numeric_limits<int>::max());
  if ((floored.array() < int_min).any() ||
      (floored.array() > int_max).any()) {
    return false;
  }
  index = floored.cast<int>();
  return index.x() >= 0 && index.y() >= 0 && index.z() >= 0 &&
      index.x() < voxel_count.x() && index.y() < voxel_count.y() &&
      index.z() < voxel_count.z();
}

std::size_t address(const Eigen::Vector3i& index,
                    const Eigen::Vector3i& voxel_count) {
  return (static_cast<std::size_t>(index.x()) *
          static_cast<std::size_t>(voxel_count.y()) +
          static_cast<std::size_t>(index.y())) *
          static_cast<std::size_t>(voxel_count.z()) +
      static_cast<std::size_t>(index.z());
}

std::size_t columnAddress(const int x, const int y,
                          const Eigen::Vector3i& voxel_count) {
  return (static_cast<std::size_t>(x) *
          static_cast<std::size_t>(voxel_count.y()) +
          static_cast<std::size_t>(y)) *
      static_cast<std::size_t>(voxel_count.z());
}

bool closedBallInBox(const Eigen::Vector3d& point,
                     const double radius,
                     const Eigen::Vector3d& lower,
                     const Eigen::Vector3d& upper) {
  if (!finite(point) || !finite(radius) || radius < 0.0 ||
      !finite(lower) || !finite(upper)) {
    return false;
  }
  const Eigen::Vector3d ball_lower = point.array() - radius;
  const Eigen::Vector3d ball_upper = point.array() + radius;
  return finite(ball_lower) && finite(ball_upper) &&
      (ball_lower.array() >= lower.array()).all() &&
      (ball_upper.array() <= upper.array()).all();
}

bool gridBounds(const CloudOccupancySnapshot& snapshot,
                Eigen::Vector3d& lower,
                Eigen::Vector3d& upper) {
  lower = snapshot.grid_origin;
  const Eigen::Vector3d count = snapshot.voxel_count.cast<double>();
  upper = snapshot.grid_origin + snapshot.resolution * count;
  return finite(lower) && finite(upper) &&
      (upper.array() > lower.array()).all();
}

bool clearanceIndexRange(const double lower_coordinate,
                         const double upper_coordinate,
                         const int count,
                         int& first,
                         int& last) {
  if (!finite(lower_coordinate) || !finite(upper_coordinate) || count <= 0 ||
      lower_coordinate > upper_coordinate) {
    return false;
  }
  const double lower_floor = std::floor(lower_coordinate);
  const double upper_floor = std::floor(upper_coordinate);
  const double int_min = static_cast<double>(std::numeric_limits<int>::min());
  const double int_max = static_cast<double>(std::numeric_limits<int>::max());
  if (!finite(lower_floor) || !finite(upper_floor) ||
      lower_floor < int_min + 1.0 || upper_floor > int_max - 1.0) {
    return false;
  }
  const int padded_first = static_cast<int>(lower_floor) - 1;
  const int padded_last = static_cast<int>(upper_floor) + 1;
  first = std::max(0, padded_first);
  last = std::min(count - 1, padded_last);
  return first <= last;
}

bool boundedVoxelCount(const int first_x, const int last_x,
                       const int first_y, const int last_y,
                       const int first_z, const int last_z) {
  if (first_x > last_x || first_y > last_y || first_z > last_z) return false;
  const std::size_t x = static_cast<std::size_t>(last_x - first_x) + 1U;
  const std::size_t y = static_cast<std::size_t>(last_y - first_y) + 1U;
  const std::size_t z = static_cast<std::size_t>(last_z - first_z) + 1U;
  if (x > kMaxClearanceVoxelChecks || y > kMaxClearanceVoxelChecks ||
      z > kMaxClearanceVoxelChecks ||
      x > kMaxClearanceVoxelChecks / y) {
    return false;
  }
  const std::size_t xy = x * y;
  return xy <= kMaxClearanceVoxelChecks / z;
}

double pointToClosedVoxelDistance(const Eigen::Vector3d& point,
                                  const Eigen::Vector3d& voxel_lower,
                                  const Eigen::Vector3d& voxel_upper) {
  const Eigen::Vector3d lower_delta = (voxel_lower - point).cwiseMax(0.0);
  const Eigen::Vector3d upper_delta = (point - voxel_upper).cwiseMax(0.0);
  const Eigen::Vector3d delta = lower_delta.cwiseMax(upper_delta);
  return delta.norm();
}

std::shared_ptr<const CloudOccupancyColumnIndex> buildOccupiedColumnIndex(
    const CloudOccupancySnapshot& snapshot) {
  std::size_t column_count = 0U;
  if (!validColumnCount(snapshot.voxel_count, column_count) ||
      column_count == std::numeric_limits<std::size_t>::max()) {
    return std::shared_ptr<const CloudOccupancyColumnIndex>();
  }
  const std::shared_ptr<CloudOccupancyColumnIndex> index(
      new CloudOccupancyColumnIndex());
  index->observation_sequence = snapshot.observation_sequence;
  index->voxel_count = snapshot.voxel_count;
  index->occupied_size = snapshot.occupied.size();
  index->offsets.assign(column_count + 1U, 0U);
  for (int x = 0; x < snapshot.voxel_count.x(); ++x) {
    for (int y = 0; y < snapshot.voxel_count.y(); ++y) {
      const std::size_t column = static_cast<std::size_t>(x) *
              static_cast<std::size_t>(snapshot.voxel_count.y()) +
          static_cast<std::size_t>(y);
      index->offsets[column] = index->addresses.size();
      const std::size_t column_address =
          columnAddress(x, y, snapshot.voxel_count);
      for (int z = 0; z < snapshot.voxel_count.z(); ++z) {
        const std::size_t voxel_address = column_address +
            static_cast<std::size_t>(z);
        if (snapshot.occupied[voxel_address] != 0U) {
          index->addresses.push_back(voxel_address);
        }
      }
    }
  }
  index->offsets[column_count] = index->addresses.size();
  return index;
}

const CloudOccupancyColumnIndex* usableOccupiedColumnIndex(
    const CloudOccupancySnapshot& snapshot,
    const CloudOccupancyColumnIndex* const index) {
  if (index == nullptr ||
      index->observation_sequence != snapshot.observation_sequence ||
      index->voxel_count != snapshot.voxel_count ||
      index->occupied_size != snapshot.occupied.size()) {
    return nullptr;
  }
  std::size_t column_count = 0U;
  if (!validColumnCount(snapshot.voxel_count, column_count) ||
      column_count == std::numeric_limits<std::size_t>::max()) {
    return nullptr;
  }
  if (index->offsets.size() != column_count + 1U ||
      index->offsets.empty() || index->offsets.front() != 0U ||
      index->offsets.back() != index->addresses.size()) {
    return nullptr;
  }
  return index;
}

enum class IndexedScanResult {
  COMPLETE,
  FALLBACK_DENSE,
  UNAVAILABLE,
};

IndexedScanResult scanIndexedOccupiedVoxels(
    const CloudOccupancySnapshot& snapshot,
    const CloudOccupancyColumnIndex& column_index,
    const Eigen::Vector3d& point,
    const int first_x, const int last_x, const int first_y, const int last_y,
    const int first_z, const int last_z, double& nearest) {
  const std::size_t z_count =
      static_cast<std::size_t>(snapshot.voxel_count.z());
  const std::size_t y_count =
      static_cast<std::size_t>(snapshot.voxel_count.y());
  for (int x = first_x; x <= last_x; ++x) {
    for (int y = first_y; y <= last_y; ++y) {
      const std::size_t column = static_cast<std::size_t>(x) * y_count +
          static_cast<std::size_t>(y);
      if (column + 1U >= column_index.offsets.size()) {
        return IndexedScanResult::FALLBACK_DENSE;
      }
      const std::size_t begin = column_index.offsets[column];
      const std::size_t end = column_index.offsets[column + 1U];
      if (begin > end || end > column_index.addresses.size()) {
        return IndexedScanResult::FALLBACK_DENSE;
      }
      const std::size_t column_address =
          columnAddress(x, y, snapshot.voxel_count);
      if (column_address > snapshot.occupied.size() ||
          z_count > snapshot.occupied.size() - column_address) {
        return IndexedScanResult::FALLBACK_DENSE;
      }
      std::size_t previous_address = 0U;
      for (std::size_t entry = begin; entry < end; ++entry) {
        const std::size_t voxel_address =
            column_index.addresses[entry];
        if (voxel_address < column_address ||
            voxel_address >= column_address + z_count ||
            voxel_address >= snapshot.occupied.size() ||
            snapshot.occupied[voxel_address] == 0U ||
            (entry != begin && voxel_address <= previous_address)) {
          return IndexedScanResult::FALLBACK_DENSE;
        }
        previous_address = voxel_address;
      }
      const std::size_t first_address = column_address +
          static_cast<std::size_t>(first_z);
      const std::size_t last_address = column_address +
          static_cast<std::size_t>(last_z);
      const std::vector<std::size_t>::const_iterator z_begin =
          std::lower_bound(column_index.addresses.begin() + begin,
                           column_index.addresses.begin() + end,
                           first_address);
      for (std::vector<std::size_t>::const_iterator entry = z_begin;
           entry != column_index.addresses.begin() + end &&
           *entry <= last_address; ++entry) {
        const std::size_t voxel_address = *entry;
        const Eigen::Vector3i index(
            x, y, static_cast<int>(voxel_address - column_address));
        const Eigen::Vector3d voxel_lower = snapshot.grid_origin +
            snapshot.resolution * index.cast<double>();
        const Eigen::Vector3d voxel_upper = voxel_lower +
            Eigen::Vector3d::Constant(snapshot.resolution);
        const double distance = pointToClosedVoxelDistance(
            point, voxel_lower, voxel_upper);
        if (!finite(distance)) return IndexedScanResult::UNAVAILABLE;
        nearest = std::min(nearest, distance);
      }
    }
  }
  return IndexedScanResult::COMPLETE;
}

}  // namespace

bool cloudOccupancySnapshotConsistent(const CloudOccupancySnapshot& snapshot) {
  if (!snapshot.valid || snapshot.observation_sequence == 0U ||
      !finite(snapshot.map_min) || !finite(snapshot.map_max) ||
      !finite(snapshot.observed_min) || !finite(snapshot.observed_max) ||
      !finite(snapshot.grid_origin) || !finite(snapshot.resolution) ||
      snapshot.resolution <= 0.0 ||
      !finite(snapshot.included_map_inflation) ||
      snapshot.included_map_inflation < 0.0 ||
      (snapshot.map_max.array() <= snapshot.map_min.array()).any() ||
      (snapshot.observed_max.array() < snapshot.observed_min.array()).any() ||
      (snapshot.observed_min.array() < snapshot.map_min.array() -
       kMapBoundaryTolerance).any() ||
      (snapshot.observed_max.array() > snapshot.map_max.array() +
       kMapBoundaryTolerance).any()) {
    return false;
  }
  std::size_t voxel_count = 0U;
  return validVoxelCount(snapshot.voxel_count, voxel_count) &&
      snapshot.occupied.size() == voxel_count;
}

CloudOccupancySnapshot buildCloudOccupancySnapshot(
    const CloudOccupancySnapshotBuildInput& input) {
  CloudOccupancySnapshot snapshot;
  snapshot.observation_sequence = input.observation_sequence;
  snapshot.observation_stamp = input.observation_stamp;
  snapshot.map_min = input.map_min;
  snapshot.map_max = input.map_max;
  snapshot.grid_origin = input.grid_origin;
  snapshot.voxel_count = input.voxel_count;
  snapshot.resolution = input.resolution;

  std::size_t voxel_total = 0U;
  if (!input.odom_valid || input.observation_sequence == 0U ||
      !finite(input.camera_position) || !finite(input.map_min) ||
      !finite(input.map_max) || !finite(input.grid_origin) ||
      !finite(input.local_update_range) ||
      (input.local_update_range.array() <= 0.0).any() ||
      !finite(input.resolution) || input.resolution <= 0.0 ||
      !finite(input.obstacles_inflation) || input.obstacles_inflation < 0.0 ||
      (input.map_max.array() <= input.map_min.array()).any() ||
      !pointInMap(input.camera_position, input.map_min, input.map_max) ||
      !validVoxelCount(input.voxel_count, voxel_total)) {
    return snapshot;
  }

  snapshot.observed_min = (input.camera_position - input.local_update_range)
                              .cwiseMax(input.map_min);
  snapshot.observed_max = (input.camera_position + input.local_update_range)
                              .cwiseMin(input.map_max);
  if ((snapshot.observed_max.array() < snapshot.observed_min.array()).any()) {
    return snapshot;
  }

  const int inflation_steps = static_cast<int>(
      std::ceil(input.obstacles_inflation / input.resolution));
  if (inflation_steps < 0) return snapshot;
  snapshot.included_map_inflation =
      static_cast<double>(inflation_steps) * input.resolution;
  if (!finite(snapshot.included_map_inflation)) return snapshot;
  snapshot.occupied.assign(voxel_total, 0U);

  for (const Eigen::Vector3d& point : input.cloud_points) {
    if (!finite(point)) {
      snapshot.occupied.clear();
      return snapshot;
    }
    const Eigen::Vector3d deviation = point - input.camera_position;
    // Match SDFMap::cloudCallback: only cloud points inside the current local
    // update range generate the inflated environment occupancy.
    if (std::abs(deviation.x()) >= input.local_update_range.x() ||
        std::abs(deviation.y()) >= input.local_update_range.y() ||
        std::abs(deviation.z()) >= input.local_update_range.z()) {
      continue;
    }
    for (int x = -inflation_steps; x <= inflation_steps; ++x) {
      for (int y = -inflation_steps; y <= inflation_steps; ++y) {
        // cloudCallback intentionally uses one z voxel for its planar map
        // inflation.  Preserve that exact environment backing here.
        for (int z = -1; z <= 1; ++z) {
          const Eigen::Vector3d inflated = point + input.resolution *
              Eigen::Vector3d(static_cast<double>(x),
                              static_cast<double>(y),
                              static_cast<double>(z));
          Eigen::Vector3i index;
          if (pointToIndex(inflated, input.grid_origin, input.voxel_count,
                           input.resolution, index)) {
            snapshot.occupied[address(index, input.voxel_count)] = 1U;
          }
        }
      }
    }
  }
  snapshot.occupied_column_index = buildOccupiedColumnIndex(snapshot);
  snapshot.valid = true;
  return snapshot;
}

CloudOccupancySnapshotQueryResult queryCloudOccupancySnapshot(
    const CloudOccupancySnapshot& snapshot,
    const Eigen::Vector3d& point) {
  CloudOccupancySnapshotQueryResult result;
  if (!cloudOccupancySnapshotConsistent(snapshot)) return result;
  if (!finite(point) || !pointInMap(point, snapshot.map_min, snapshot.map_max)) {
    result.status = CloudOccupancyStatus::OUT_OF_MAP;
    return result;
  }
  if (!pointInObservedBox(point, snapshot.observed_min, snapshot.observed_max)) {
    result.status = CloudOccupancyStatus::UNKNOWN;
    return result;
  }
  Eigen::Vector3i index;
  if (!pointToIndex(point, snapshot.grid_origin, snapshot.voxel_count,
                    snapshot.resolution, index)) {
    result.status = CloudOccupancyStatus::OUT_OF_MAP;
    return result;
  }
  result.status = snapshot.occupied[address(index, snapshot.voxel_count)] != 0U
      ? CloudOccupancyStatus::OCCUPIED : CloudOccupancyStatus::KNOWN_FREE;
  return result;
}

CloudOccupancySnapshotClearanceResult
queryCloudOccupancySnapshotClearance(
    const CloudOccupancySnapshot& snapshot,
    const Eigen::Vector3d& point,
    const double required_radius) {
  CloudOccupancySnapshotClearanceResult result;
  if (!cloudOccupancySnapshotConsistent(snapshot) || !finite(point) ||
      !finite(required_radius) || required_radius < 0.0) {
    return result;
  }
  if (!pointInMap(point, snapshot.map_min, snapshot.map_max)) {
    result.status = CloudOccupancyStatus::OUT_OF_MAP;
    return result;
  }
  if (!pointInObservedBox(point, snapshot.observed_min, snapshot.observed_max) ||
      !closedBallInBox(point, required_radius, snapshot.observed_min,
                       snapshot.observed_max)) {
    result.status = CloudOccupancyStatus::UNKNOWN;
    return result;
  }

  Eigen::Vector3d grid_min;
  Eigen::Vector3d grid_max;
  if (!gridBounds(snapshot, grid_min, grid_max)) return result;
  if (!closedBallInBox(point, required_radius, grid_min, grid_max)) {
    result.status = CloudOccupancyStatus::OUT_OF_MAP;
    return result;
  }

  Eigen::Vector3i point_index;
  if (!pointToIndex(point, snapshot.grid_origin, snapshot.voxel_count,
                    snapshot.resolution, point_index)) {
    result.status = CloudOccupancyStatus::OUT_OF_MAP;
    return result;
  }

  const Eigen::Vector3d lower =
      (point.array() - required_radius - snapshot.grid_origin.array()) /
      snapshot.resolution;
  const Eigen::Vector3d upper =
      (point.array() + required_radius - snapshot.grid_origin.array()) /
      snapshot.resolution;
  int first_x = 0;
  int last_x = -1;
  int first_y = 0;
  int last_y = -1;
  int first_z = 0;
  int last_z = -1;
  if (!finite(lower) || !finite(upper) ||
      !clearanceIndexRange(lower.x(), upper.x(), snapshot.voxel_count.x(),
                           first_x, last_x) ||
      !clearanceIndexRange(lower.y(), upper.y(), snapshot.voxel_count.y(),
                           first_y, last_y) ||
      !clearanceIndexRange(lower.z(), upper.z(), snapshot.voxel_count.z(),
                           first_z, last_z) ||
      !boundedVoxelCount(first_x, last_x, first_y, last_y, first_z, last_z)) {
    return result;
  }

  double nearest = std::numeric_limits<double>::infinity();
  const CloudOccupancyColumnIndex* const column_index =
      usableOccupiedColumnIndex(snapshot,
                                snapshot.occupied_column_index.get());
  bool indexed_complete = false;
  if (column_index != nullptr) {
    const IndexedScanResult indexed_result = scanIndexedOccupiedVoxels(
        snapshot, *column_index, point, first_x, last_x, first_y, last_y,
        first_z, last_z, nearest);
    if (indexed_result == IndexedScanResult::UNAVAILABLE) {
      return CloudOccupancySnapshotClearanceResult();
    }
    indexed_complete = indexed_result == IndexedScanResult::COMPLETE;
  }
  if (!indexed_complete) {
    nearest = std::numeric_limits<double>::infinity();
    for (int x = first_x; x <= last_x; ++x) {
      for (int y = first_y; y <= last_y; ++y) {
        for (int z = first_z; z <= last_z; ++z) {
          const Eigen::Vector3i index(x, y, z);
          const std::size_t voxel_address = address(index, snapshot.voxel_count);
          if (voxel_address >= snapshot.occupied.size() ||
              snapshot.occupied[voxel_address] == 0U) {
            continue;
          }
          const Eigen::Vector3d voxel_lower = snapshot.grid_origin +
              snapshot.resolution * index.cast<double>();
          const Eigen::Vector3d voxel_upper = voxel_lower +
              Eigen::Vector3d::Constant(snapshot.resolution);
          const double distance = pointToClosedVoxelDistance(
              point, voxel_lower, voxel_upper);
          if (!finite(distance)) return CloudOccupancySnapshotClearanceResult();
          nearest = std::min(nearest, distance);
        }
      }
    }
  }

  if (nearest <= 0.0) {
    result.status = CloudOccupancyStatus::OCCUPIED;
    result.nearest_occupied_voxel_volume_distance = 0.0;
    return result;
  }
  result.status = CloudOccupancyStatus::KNOWN_FREE;
  result.nearest_occupied_voxel_volume_distance =
      std::isfinite(nearest) ? std::min(nearest, required_radius)
                             : required_radius;
  result.clearance_certified = true;
  return result;
}

}  // namespace plan_env
