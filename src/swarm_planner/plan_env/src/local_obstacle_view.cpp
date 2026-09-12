#include "plan_env/local_obstacle_view.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace plan_env {
namespace {

bool finite(const double value) {
  return std::isfinite(value);
}

bool finite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

bool orderedBox(const LocalObstacleBox& box, const bool allow_degenerate) {
  if (!finite(box.min) || !finite(box.max)) return false;
  if (allow_degenerate) {
    return (box.min.array() <= box.max.array()).all();
  }
  return (box.min.array() < box.max.array()).all();
}

// Outward intervals for the two elementary operations used to construct a
// native voxel boundary.  These are deliberately small binary64 repairs, not
// an assertion that a wider floating-point type is exact for all inputs.
bool outwardProduct(const std::int64_t integer, const double value,
                    double& lower, double& upper) {
  if (integer < 0 || !finite(value) || value < 0.0) return false;
  if (integer == 0 || value == 0.0) {
    lower = 0.0;
    upper = 0.0;
    return true;
  }
  const double product = static_cast<double>(integer) * value;
  if (!finite(product)) return false;
  lower = product == 0.0
      ? 0.0
      : std::nextafter(product, -std::numeric_limits<double>::infinity());
  upper = std::nextafter(product, std::numeric_limits<double>::infinity());
  if (!finite(lower) || !finite(upper)) return false;
  return lower <= upper;
}

bool outwardSum(const double offset, const double lower_offset,
                const double upper_offset, double& lower, double& upper) {
  if (!finite(offset) || !finite(lower_offset) || !finite(upper_offset) ||
      lower_offset > upper_offset) {
    return false;
  }
  if (lower_offset == 0.0 && upper_offset == 0.0) {
    lower = offset;
    upper = offset;
    return true;
  }
  const double lower_sum = offset + lower_offset;
  const double upper_sum = offset + upper_offset;
  if (!finite(lower_sum) || !finite(upper_sum)) return false;
  lower = std::nextafter(lower_sum,
                         -std::numeric_limits<double>::infinity());
  upper = std::nextafter(upper_sum,
                         std::numeric_limits<double>::infinity());
  return finite(lower) && finite(upper) && lower <= upper;
}

bool safeGridBounds(const LocalObstacleGridView& grid,
                    Eigen::Vector3d& lower, Eigen::Vector3d& upper) {
  lower = grid.origin;
  upper = Eigen::Vector3d::Zero();
  if (!finite(grid.origin) || !finite(grid.resolution) ||
      grid.resolution <= 0.0) {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    const double raw_first_upper =
        grid.origin(axis) + grid.resolution;
    const double raw_extent = grid.resolution *
        static_cast<double>(grid.voxel_count(axis));
    const double raw_upper = grid.origin(axis) + raw_extent;
    if (!finite(raw_first_upper) || !(raw_first_upper > grid.origin(axis)) ||
        !finite(raw_extent) || raw_extent <= 0.0 || !finite(raw_upper) ||
        !(raw_upper > grid.origin(axis))) {
      // Do not turn an unrepresentable native grid span into a valid one by
      // applying only an outward ULP repair.
      return false;
    }
    double extent_lower = 0.0;
    double extent_upper = 0.0;
    if (!outwardProduct(static_cast<std::int64_t>(grid.voxel_count(axis)),
                        grid.resolution, extent_lower, extent_upper)) {
      return false;
    }
    double sum_lower = 0.0;
    double sum_upper = 0.0;
    if (!outwardSum(grid.origin(axis), extent_lower, extent_upper,
                    sum_lower, sum_upper)) {
      return false;
    }
    upper(axis) = sum_upper;
    if (!(upper(axis) > lower(axis))) return false;
  }
  return true;
}

bool validBuffers(const LocalObstacleGridView& grid, const std::size_t total) {
  if (grid.inflated == nullptr || grid.inflated->size() != total) {
    return false;
  }
  if (grid.manual != nullptr && grid.manual->size() != total) return false;
  if (grid.static_layer != nullptr && grid.static_layer->size() != total) {
    return false;
  }
  return true;
}

bool checkedProduct(const Eigen::Vector3i& count, std::size_t& total) {
  if ((count.array() <= 0).any()) return false;
  const std::size_t x = static_cast<std::size_t>(count.x());
  const std::size_t y = static_cast<std::size_t>(count.y());
  const std::size_t z = static_cast<std::size_t>(count.z());
  if (x > std::numeric_limits<std::size_t>::max() / y) return false;
  const std::size_t xy = x * y;
  if (xy > std::numeric_limits<std::size_t>::max() / z) return false;
  total = xy * z;
  return total > 0U;
}

bool intersectBoxes(const LocalObstacleBox& lhs, const LocalObstacleBox& rhs,
                    LocalObstacleBox& intersection) {
  intersection.min = lhs.min.cwiseMax(rhs.min);
  intersection.max = lhs.max.cwiseMin(rhs.max);
  return (intersection.min.array() <= intersection.max.array()).all() &&
      finite(intersection.min) && finite(intersection.max);
}

bool sameBox(const LocalObstacleBox& lhs, const LocalObstacleBox& rhs) {
  return (lhs.min.array() == rhs.min.array()).all() &&
      (lhs.max.array() == rhs.max.array()).all();
}

bool safeExpandLower(const double value, const double clearance,
                     double& expanded) {
  if (!finite(value) || !finite(clearance) || clearance < 0.0) return false;
  if (clearance == 0.0) {
    expanded = value;
    return true;
  }
  expanded = value - clearance;
  if (!finite(expanded)) return false;
  expanded = std::nextafter(expanded,
                            -std::numeric_limits<double>::infinity());
  return finite(expanded);
}

bool safeExpandUpper(const double value, const double clearance,
                     double& expanded) {
  if (!finite(value) || !finite(clearance) || clearance < 0.0) return false;
  if (clearance == 0.0) {
    expanded = value;
    return true;
  }
  expanded = value + clearance;
  if (!finite(expanded)) return false;
  expanded = std::nextafter(expanded,
                            std::numeric_limits<double>::infinity());
  return finite(expanded);
}

bool safeInsetLower(const double value, const double clearance,
                    double& inset) {
  if (!finite(value) || !finite(clearance) || clearance < 0.0) return false;
  if (clearance == 0.0) {
    inset = value;
    return true;
  }
  inset = value + clearance;
  if (!finite(inset)) return false;
  inset = std::nextafter(inset,
                         std::numeric_limits<double>::infinity());
  return finite(inset);
}

bool safeInsetUpper(const double value, const double clearance,
                    double& inset) {
  if (!finite(value) || !finite(clearance) || clearance < 0.0) return false;
  if (clearance == 0.0) {
    inset = value;
    return true;
  }
  inset = value - clearance;
  if (!finite(inset)) return false;
  inset = std::nextafter(inset,
                         -std::numeric_limits<double>::infinity());
  return finite(inset);
}

bool nativeVoxelBox(const LocalObstacleGridView& grid,
                    const Eigen::Vector3i& index,
                    LocalObstacleBox& box) {
  if ((index.array() < 0).any() ||
      (index.array() >= grid.voxel_count.array()).any()) {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    const double raw_offset = static_cast<double>(index(axis)) *
        grid.resolution;
    const double raw_lower = grid.origin(axis) + raw_offset;
    const double raw_upper = raw_lower + grid.resolution;
    if (!finite(raw_offset) || !finite(raw_lower) || !finite(raw_upper) ||
        !(raw_upper > raw_lower)) {
      return false;
    }
    double lower_offset = 0.0;
    double upper_offset = 0.0;
    double upper_index_offset_lower = 0.0;
    double upper_index_offset_upper = 0.0;
    if (!outwardProduct(static_cast<std::int64_t>(index(axis)),
                        grid.resolution, lower_offset, upper_offset) ||
        !outwardProduct(static_cast<std::int64_t>(index(axis)) + 1,
                        grid.resolution, upper_index_offset_lower,
                        upper_index_offset_upper)) {
      return false;
    }
    double lower = 0.0;
    double unused_upper = 0.0;
    double upper = 0.0;
    if (!outwardSum(grid.origin(axis), lower_offset, upper_offset,
                    lower, unused_upper) ||
        !outwardSum(grid.origin(axis), upper_index_offset_lower,
                    upper_index_offset_upper, unused_upper, upper)) {
      return false;
    }
    if (!(upper >= lower) || !finite(lower) || !finite(upper)) return false;
    box.min(axis) = lower;
    box.max(axis) = upper;
  }
  return true;
}

bool closedBoxesIntersect(const LocalObstacleBox& lhs,
                          const LocalObstacleBox& rhs) {
  return (lhs.min.array() <= rhs.max.array()).all() &&
      (rhs.min.array() <= lhs.max.array()).all();
}

bool indexRangeForAxis(const double roi_min, const double roi_max,
                       const double origin, const double resolution,
                       const int count, int& first, int& last) {
  if (!finite(roi_min) || !finite(roi_max) || !finite(origin) ||
      !finite(resolution) || resolution <= 0.0 || count <= 0 ||
      roi_min > roi_max) {
    return false;
  }
  const double lower_ratio = (roi_min - origin) / resolution;
  const double upper_ratio = (roi_max - origin) / resolution;
  if (!finite(lower_ratio) || !finite(upper_ratio)) return false;
  const double lower_floor = std::floor(lower_ratio);
  const double upper_floor = std::floor(upper_ratio);
  if (!finite(lower_floor) || !finite(upper_floor)) return false;
  const double min_int =
      static_cast<double>(std::numeric_limits<std::int64_t>::min());
  const double max_int =
      static_cast<double>(std::numeric_limits<std::int64_t>::max());
  if (lower_floor < min_int + 2.0 || upper_floor > max_int - 2.0) {
    return false;
  }
  const std::int64_t lower_index = static_cast<std::int64_t>(lower_floor) - 1;
  const std::int64_t upper_index = static_cast<std::int64_t>(upper_floor) + 1;
  const std::int64_t clamped_first = std::max<std::int64_t>(0, lower_index);
  const std::int64_t clamped_last = std::min<std::int64_t>(
      static_cast<std::int64_t>(count) - 1, upper_index);
  if (clamped_first > clamped_last) return false;
  first = static_cast<int>(clamped_first);
  last = static_cast<int>(clamped_last);
  return true;
}

bool checkedRangeProduct(const int first_x, const int last_x,
                         const int first_y, const int last_y,
                         const int first_z, const int last_z,
                         std::size_t& total) {
  if (first_x < 0 || first_y < 0 || first_z < 0 ||
      first_x > last_x || first_y > last_y || first_z > last_z) {
    return false;
  }
  const std::size_t x = static_cast<std::size_t>(last_x - first_x) + 1U;
  const std::size_t y = static_cast<std::size_t>(last_y - first_y) + 1U;
  const std::size_t z = static_cast<std::size_t>(last_z - first_z) + 1U;
  if (x > std::numeric_limits<std::size_t>::max() / y) return false;
  const std::size_t xy = x * y;
  if (xy > std::numeric_limits<std::size_t>::max() / z) return false;
  total = xy * z;
  return true;
}

std::size_t address(const Eigen::Vector3i& index,
                    const Eigen::Vector3i& voxel_count) {
  return (static_cast<std::size_t>(index.x()) *
              static_cast<std::size_t>(voxel_count.y()) +
          static_cast<std::size_t>(index.y())) *
          static_cast<std::size_t>(voxel_count.z()) +
      static_cast<std::size_t>(index.z());
}

void clearFailure(LocalObstacleView& view, const LocalObstacleViewStatus status) {
  view = LocalObstacleView();
  view.status = status;
}

}  // namespace

LocalObstacleView buildLocalObstacleView(
    const LocalObstacleGridView& grid,
    const LocalObstacleRequest& request) {
  LocalObstacleView view;

  std::size_t total_voxels = 0U;
  if (!finite(grid.origin) || !finite(grid.resolution) ||
      grid.resolution <= 0.0 || grid.frame_id.empty() ||
      !orderedBox(grid.map_bounds, false) ||
      !orderedBox(request.reference_region, true) ||
      !checkedProduct(grid.voxel_count, total_voxels) ||
      !validBuffers(grid, total_voxels) || request.max_voxel_checks == 0U ||
      request.max_occupied_voxels == 0U || !finite(request.clearance) ||
      request.clearance < 0.0) {
    return view;
  }
  if (request.known_region_valid && !orderedBox(request.known_region, true)) {
    return view;
  }

  Eigen::Vector3d grid_min;
  Eigen::Vector3d grid_max;
  if (!safeGridBounds(grid, grid_min, grid_max) ||
      (grid.map_bounds.min.array() < grid_min.array()).any() ||
      (grid.map_bounds.max.array() > grid_max.array()).any()) {
    return view;
  }

  // No explicit complete known domain means that an empty/partial layer can
  // never be interpreted as known free space.
  if (!request.known_region_valid) {
    clearFailure(view, LocalObstacleViewStatus::UNKNOWN_DOMAIN);
    return view;
  }

  LocalObstacleBox valid_domain;
  if (!intersectBoxes(grid.map_bounds, request.known_region, valid_domain)) {
    clearFailure(view, LocalObstacleViewStatus::OUT_OF_DOMAIN);
    return view;
  }

  LocalObstacleBox requested_roi;
  for (int axis = 0; axis < 3; ++axis) {
    if (!safeExpandLower(request.reference_region.min(axis),
                         request.clearance, requested_roi.min(axis)) ||
        !safeExpandUpper(request.reference_region.max(axis),
                         request.clearance, requested_roi.max(axis))) {
      return view;
    }
  }

  LocalObstacleBox obstacle_region;
  if (!intersectBoxes(requested_roi, valid_domain, obstacle_region)) {
    clearFailure(view, LocalObstacleViewStatus::OUT_OF_DOMAIN);
    return view;
  }
  const bool clipped = !sameBox(requested_roi, obstacle_region);

  LocalObstacleBox reference_domain;
  if (!clipped) {
    // Preserve the caller's exact zero-thickness/line/plane box when no
    // clipping was needed; this avoids an inessential round-trip eroding a
    // degenerate reference set.
    reference_domain = request.reference_region;
  } else {
    for (int axis = 0; axis < 3; ++axis) {
      const bool lower_face_clipped =
          obstacle_region.min(axis) != requested_roi.min(axis);
      const bool upper_face_clipped =
          obstacle_region.max(axis) != requested_roi.max(axis);
      if (lower_face_clipped) {
        double lower_support = 0.0;
        if (!safeInsetLower(obstacle_region.min(axis), request.clearance,
                            lower_support)) {
          return view;
        }
        reference_domain.min(axis) = std::max(
            request.reference_region.min(axis), lower_support);
      } else {
        reference_domain.min(axis) = request.reference_region.min(axis);
      }
      if (upper_face_clipped) {
        double upper_support = 0.0;
        if (!safeInsetUpper(obstacle_region.max(axis), request.clearance,
                            upper_support)) {
          return view;
        }
        reference_domain.max(axis) = std::min(
            request.reference_region.max(axis), upper_support);
      } else {
        reference_domain.max(axis) = request.reference_region.max(axis);
      }
    }
  }
  if (!orderedBox(reference_domain, true)) {
    clearFailure(view, LocalObstacleViewStatus::OUT_OF_DOMAIN);
    return view;
  }

  // Verify the defining halo relation using the binary64 operations of the
  // interface.  If an inward-rounded endpoint is still too close to an
  // obstacle boundary, take at most one further representable step inward;
  // an unrepresentable relation fails closed instead of looping over ulps.
  for (int axis = 0; axis < 3; ++axis) {
    double lower_halo = reference_domain.min(axis) - request.clearance;
    double upper_halo = reference_domain.max(axis) + request.clearance;
    if (!finite(lower_halo) || !finite(upper_halo)) return view;
    if (lower_halo < obstacle_region.min(axis)) {
      const double next = std::nextafter(
          reference_domain.min(axis), std::numeric_limits<double>::infinity());
      if (!finite(next) || next <= reference_domain.min(axis)) {
        clearFailure(view, LocalObstacleViewStatus::OUT_OF_DOMAIN);
        return view;
      }
      reference_domain.min(axis) = next;
      lower_halo = reference_domain.min(axis) - request.clearance;
    }
    if (upper_halo > obstacle_region.max(axis)) {
      const double next = std::nextafter(
          reference_domain.max(axis), -std::numeric_limits<double>::infinity());
      if (!finite(next) || next >= reference_domain.max(axis)) {
        clearFailure(view, LocalObstacleViewStatus::OUT_OF_DOMAIN);
        return view;
      }
      reference_domain.max(axis) = next;
      upper_halo = reference_domain.max(axis) + request.clearance;
    }
    if (reference_domain.min(axis) > reference_domain.max(axis)) {
      clearFailure(view, LocalObstacleViewStatus::OUT_OF_DOMAIN);
      return view;
    }
    if (!finite(lower_halo) || !finite(upper_halo) ||
        lower_halo < obstacle_region.min(axis) ||
        upper_halo > obstacle_region.max(axis)) {
      clearFailure(view, LocalObstacleViewStatus::OUT_OF_DOMAIN);
      return view;
    }
  }

  int first_x = 0;
  int last_x = -1;
  int first_y = 0;
  int last_y = -1;
  int first_z = 0;
  int last_z = -1;
  if (!indexRangeForAxis(obstacle_region.min.x(), obstacle_region.max.x(),
                         grid.origin.x(), grid.resolution,
                         grid.voxel_count.x(), first_x, last_x) ||
      !indexRangeForAxis(obstacle_region.min.y(), obstacle_region.max.y(),
                         grid.origin.y(), grid.resolution,
                         grid.voxel_count.y(), first_y, last_y) ||
      !indexRangeForAxis(obstacle_region.min.z(), obstacle_region.max.z(),
                         grid.origin.z(), grid.resolution,
                         grid.voxel_count.z(), first_z, last_z)) {
    return view;
  }
  std::size_t candidate_count = 0U;
  if (!checkedRangeProduct(first_x, last_x, first_y, last_y,
                           first_z, last_z, candidate_count)) {
    return view;
  }
  if (candidate_count > request.max_voxel_checks) {
    clearFailure(view, LocalObstacleViewStatus::BUDGET_EXCEEDED);
    return view;
  }

  view.status = LocalObstacleViewStatus::VALID;
  view.frame_id = grid.frame_id;
  view.resolution = grid.resolution;
  view.origin = grid.origin;
  view.voxel_count = grid.voxel_count;
  view.obstacle_region = obstacle_region;
  view.reference_domain = reference_domain;
  view.request_clipped = clipped ||
      !sameBox(reference_domain, request.reference_region);
  view.obstacles.reserve(std::min<std::size_t>(
      256U, request.max_occupied_voxels));

  std::size_t visited = 0U;
  for (int x = first_x; x <= last_x; ++x) {
    for (int y = first_y; y <= last_y; ++y) {
      for (int z = first_z; z <= last_z; ++z) {
        ++visited;
        const Eigen::Vector3i index(x, y, z);
        const std::size_t voxel_address = address(index, grid.voxel_count);
        std::uint8_t layer_mask = 0U;
        if ((*grid.inflated)[voxel_address] != 0) layer_mask |= 0x01U;
        if (grid.manual != nullptr && (*grid.manual)[voxel_address] != 0) {
          layer_mask |= 0x02U;
        }
        if (grid.static_layer != nullptr &&
            (*grid.static_layer)[voxel_address] != 0) {
          layer_mask |= 0x04U;
        }
        if (layer_mask == 0U) continue;
        LocalObstacleBox voxel_box;
        if (!nativeVoxelBox(grid, index, voxel_box)) {
          clearFailure(view, LocalObstacleViewStatus::INVALID_INPUT);
          return view;
        }
        if (!closedBoxesIntersect(voxel_box, obstacle_region)) continue;
        if (view.obstacles.size() >= request.max_occupied_voxels) {
          clearFailure(view, LocalObstacleViewStatus::BUDGET_EXCEEDED);
          return view;
        }
        LocalObstacleVoxel voxel;
        voxel.index = index;
        voxel.bounds = voxel_box;
        voxel.layer_mask = layer_mask;
        view.obstacles.push_back(voxel);
      }
    }
  }
  if (visited != candidate_count) {
    clearFailure(view, LocalObstacleViewStatus::INVALID_INPUT);
    return view;
  }
  view.visited_voxels = visited;
  view.occupied_voxels = view.obstacles.size();
  return view;
}

}  // namespace plan_env
