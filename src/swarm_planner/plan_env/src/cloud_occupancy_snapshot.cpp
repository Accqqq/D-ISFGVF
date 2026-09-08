#include "plan_env/cloud_occupancy_snapshot.h"

#include <algorithm>
#include <boost/multiprecision/cpp_int.hpp>
#include <climits>
#include <cmath>
#include <cfenv>
#include <cstring>
#include <limits>

#if defined(__i386__) || defined(__x86_64__)
#include <immintrin.h>
#endif

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
constexpr std::uint32_t kKnownCaptureSupportEvidenceMask =
    kSDFMapCaptureSupportEvidenceDepthRaycast |
    kSDFMapCaptureSupportEvidenceCompletePreknownDomain;

bool finite(const double value) {
  return std::isfinite(value);
}

bool finite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

// All directed interval and exact-boundary arithmetic below assumes IEEE-754
// binary64, round-to-nearest, separate multiply/add operations, and gradual
// underflow.  These are environmental inputs to the proof; widening an
// interval after FTZ/DAZ or a non-nearest rounding mode cannot recover the
// missed voxel or boundary.  Never mutate the caller's FP environment here.
bool floatingPointEnvironmentSupportedV2() {
#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && \
                               __FINITE_MATH_ONLY__)
  return false;
#endif
#if defined(__FMA__) || defined(__FP_FAST_FMA) || defined(__FP_FAST_FMAF)
  return false;
#endif
#if defined(__FLT_EVAL_METHOD__) && __FLT_EVAL_METHOD__ != 0
  return false;
#endif
  if (!std::numeric_limits<double>::is_iec559 ||
      std::numeric_limits<double>::radix != 2 ||
      std::numeric_limits<double>::digits != 53 ||
      std::numeric_limits<double>::has_denorm != std::denorm_present ||
      std::fegetround() != FE_TONEAREST) {
    return false;
  }
#if defined(__i386__) || defined(__x86_64__)
  const unsigned int csr = _mm_getcsr();
#ifdef _MM_FLUSH_ZERO_ON
  if ((csr & _MM_FLUSH_ZERO_ON) != 0U) return false;
#endif
#ifdef _MM_DENORMALS_ZERO_ON
  if ((csr & _MM_DENORMALS_ZERO_ON) != 0U) return false;
#endif
  return true;
#else
  // No portable C++ query proves gradual underflow on non-x86 targets.
  return false;
#endif
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

bool pointToOccupiedVoxelCenterDistance(
    const CloudOccupancySnapshot& snapshot,
    const Eigen::Vector3d& point,
    const Eigen::Vector3i& index,
    double& distance) {
  const Eigen::Vector3d center = snapshot.grid_origin +
      snapshot.resolution *
          (index.cast<double>() + Eigen::Vector3d::Constant(0.5));
  if (!finite(center)) return false;
  const double candidate = (point - center).norm();
  if (!finite(candidate)) return false;
  distance = candidate;
  return true;
}

IndexedScanResult scanIndexedOccupiedVoxelCenters(
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
        const std::size_t voxel_address = column_index.addresses[entry];
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
        double distance = 0.0;
        if (!pointToOccupiedVoxelCenterDistance(snapshot, point, index,
                                                distance)) {
          return IndexedScanResult::UNAVAILABLE;
        }
        nearest = std::min(nearest, distance);
      }
    }
  }
  return IndexedScanResult::COMPLETE;
}

}  // namespace

namespace {

using boost::multiprecision::cpp_int;

struct ExactDyadic {
  cpp_int mantissa;
  int exponent = 0;
};

ExactDyadic exactDouble(const double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  const bool negative = (bits >> 63U) != 0U;
  const std::uint64_t exponent_bits = (bits >> 52U) & 0x7ffU;
  const std::uint64_t fraction = bits & ((std::uint64_t(1) << 52U) - 1U);
  ExactDyadic result;
  if (exponent_bits == 0U) {
    result.mantissa = cpp_int(fraction);
    result.exponent = -1022 - 52;
  } else {
    result.mantissa = cpp_int(fraction | (std::uint64_t(1) << 52U));
    result.exponent = static_cast<int>(exponent_bits) - 1023 - 52;
  }
  if (negative) result.mantissa = -result.mantissa;
  return result;
}

ExactDyadic exactAdd(const ExactDyadic& left, const ExactDyadic& right) {
  ExactDyadic result;
  result.exponent = std::min(left.exponent, right.exponent);
  result.mantissa = (left.mantissa << (left.exponent - result.exponent)) +
      (right.mantissa << (right.exponent - result.exponent));
  return result;
}

ExactDyadic exactSubtract(const ExactDyadic& left, const ExactDyadic& right) {
  ExactDyadic negative_right = right;
  negative_right.mantissa = -negative_right.mantissa;
  return exactAdd(left, negative_right);
}

ExactDyadic exactMultiply(const ExactDyadic& left,
                          const ExactDyadic& right) {
  ExactDyadic result;
  result.mantissa = left.mantissa * right.mantissa;
  result.exponent = left.exponent + right.exponent;
  return result;
}

ExactDyadic exactMultiplyInteger(const ExactDyadic& value,
                                 const long long integer) {
  ExactDyadic result = value;
  result.mantissa *= integer;
  return result;
}

int compareExact(const ExactDyadic& left, const ExactDyadic& right) {
  const int exponent = std::min(left.exponent, right.exponent);
  const cpp_int left_value =
      left.mantissa << (left.exponent - exponent);
  const cpp_int right_value =
      right.mantissa << (right.exponent - exponent);
  if (left_value < right_value) return -1;
  if (left_value > right_value) return 1;
  return 0;
}

// `exactGridBounds()` intentionally returns outward-rounded display bounds so
// that serialization and crop metadata never shrink a native grid.  Those
// enclosures are not themselves a proof that a closed query ball lies inside
// the grid.  Use the exact dyadic native-origin + integer-offset geometry for
// this containment decision, including non-dyadic binary64 resolutions.
bool closedBallWithinExactGrid(const Eigen::Vector3d& origin,
                               const Eigen::Vector3i& source_min,
                               const Eigen::Vector3i& count,
                               const double resolution,
                               const Eigen::Vector3d& point,
                               const double radius) {
  if (!finite(origin) || !finite(point) || !finite(resolution) ||
      resolution <= 0.0 || !finite(radius) || radius < 0.0 ||
      (source_min.array() < 0).any() ||
      (count.array() <= 0).any()) {
    return false;
  }
  const ExactDyadic exact_resolution = exactDouble(resolution);
  const ExactDyadic exact_radius = exactDouble(radius);
  for (int axis = 0; axis < 3; ++axis) {
    const long long first = static_cast<long long>(source_min(axis));
    const long long span = static_cast<long long>(count(axis));
    const long long last = first + span;
    if (last <= first || last >
        static_cast<long long>(std::numeric_limits<int>::max()) + 1LL) {
      return false;
    }
    const ExactDyadic lower = exactAdd(
        exactDouble(origin(axis)),
        exactMultiplyInteger(exact_resolution, first));
    const ExactDyadic upper = exactAdd(
        exactDouble(origin(axis)),
        exactMultiplyInteger(exact_resolution, last));
    const ExactDyadic ball_lower = exactSubtract(
        exactDouble(point(axis)), exact_radius);
    const ExactDyadic ball_upper = exactAdd(
        exactDouble(point(axis)), exact_radius);
    if (compareExact(ball_lower, lower) < 0 ||
        compareExact(ball_upper, upper) > 0) {
      return false;
    }
  }
  return true;
}

bool exactSubLessThan(const double left, const double right,
                      const double bound) {
  ExactDyadic negative_right = exactDouble(right);
  negative_right.mantissa = -negative_right.mantissa;
  const ExactDyadic value = exactAdd(exactDouble(left), negative_right);
  return compareExact(value, exactDouble(bound)) < 0;
}

bool exactAddGreaterThan(const double left, const double right,
                         const double bound) {
  return compareExact(exactAdd(exactDouble(left), exactDouble(right)),
                      exactDouble(bound)) > 0;
}

// New captures retain the native SDFMap grid origin and carry integer source
// offsets for crops.  The all--1 source-max sentinel keeps older hand-built
// full-map fixtures value-compatible; map-produced captures always populate
// both source bounds explicitly.
bool captureSourceIndices(const SDFMapCaptureV2& capture,
                          Eigen::Vector3i& source_min,
                          Eigen::Vector3i& source_max) {
  std::size_t voxel_count = 0U;
  if (!validVoxelCount(capture.voxel_count, voxel_count)) return false;
  source_min = capture.source_min_index;
  source_max = capture.source_max_index;
  if ((source_min.array() < 0).any()) return false;

  if ((source_max.array() == -1).all() && source_min.isZero()) {
    source_max = capture.voxel_count - Eigen::Vector3i::Ones();
  }
  if ((source_max.array() < source_min.array()).any()) return false;
  for (int axis = 0; axis < 3; ++axis) {
    const long long span = static_cast<long long>(source_max(axis)) -
        static_cast<long long>(source_min(axis)) + 1LL;
    if (span <= 0 || span != static_cast<long long>(capture.voxel_count(axis))) {
      return false;
    }
  }
  return true;
}

bool exactVoxelContains(const SDFMapCaptureV2& capture,
                        const Eigen::Vector3i& index,
                        const Eigen::Vector3d& point) {
  Eigen::Vector3i source_min;
  Eigen::Vector3i source_max;
  if (!captureSourceIndices(capture, source_min, source_max)) return false;
  for (int axis = 0; axis < 3; ++axis) {
    const ExactDyadic origin = exactDouble(capture.grid_origin(axis));
    const ExactDyadic resolution = exactDouble(capture.resolution);
    const long long absolute_index = static_cast<long long>(source_min(axis)) +
        static_cast<long long>(index(axis));
    const ExactDyadic lower = exactAdd(
        origin, exactMultiplyInteger(resolution, absolute_index));
    const ExactDyadic upper = exactAdd(
        lower, resolution);
    const ExactDyadic query = exactDouble(point(axis));
    if (compareExact(query, lower) < 0 || compareExact(query, upper) > 0) {
      return false;
    }
  }
  return true;
}

struct DirectedInterval {
  double lower = 0.0;
  double upper = 0.0;
};

bool finiteInterval(const DirectedInterval& value) {
  return std::isfinite(value.lower) && std::isfinite(value.upper) &&
      value.lower <= value.upper;
}

DirectedInterval intervalFromDouble(const double value) {
  DirectedInterval result;
  result.lower = value;
  result.upper = value;
  return result;
}

bool directedAdd(const DirectedInterval& left,
                 const DirectedInterval& right,
                 DirectedInterval& result) {
  const double lower = left.lower + right.lower;
  const double upper = left.upper + right.upper;
  if (!std::isfinite(lower) || !std::isfinite(upper)) return false;
  result.lower = std::nextafter(lower,
                                -std::numeric_limits<double>::infinity());
  result.upper = std::nextafter(upper,
                                std::numeric_limits<double>::infinity());
  return finiteInterval(result);
}

bool directedSub(const DirectedInterval& left,
                 const DirectedInterval& right,
                 DirectedInterval& result) {
  const double lower = left.lower - right.upper;
  const double upper = left.upper - right.lower;
  if (!std::isfinite(lower) || !std::isfinite(upper)) return false;
  result.lower = std::nextafter(lower,
                                -std::numeric_limits<double>::infinity());
  result.upper = std::nextafter(upper,
                                std::numeric_limits<double>::infinity());
  return finiteInterval(result);
}

bool directedMul(const DirectedInterval& left,
                 const DirectedInterval& right,
                 DirectedInterval& result) {
  const double products[4] = {
      left.lower * right.lower, left.lower * right.upper,
      left.upper * right.lower, left.upper * right.upper};
  double lower = products[0];
  double upper = products[0];
  for (double value : products) {
    if (!std::isfinite(value)) return false;
    lower = std::min(lower, value);
    upper = std::max(upper, value);
  }
  result.lower = std::nextafter(lower,
                                -std::numeric_limits<double>::infinity());
  result.upper = std::nextafter(upper,
                                std::numeric_limits<double>::infinity());
  return finiteInterval(result);
}

bool directedDiv(const DirectedInterval& left,
                 const DirectedInterval& right,
                 DirectedInterval& result) {
  if (right.lower <= 0.0 && right.upper >= 0.0) return false;
  const double quotients[4] = {
      left.lower / right.lower, left.lower / right.upper,
      left.upper / right.lower, left.upper / right.upper};
  double lower = quotients[0];
  double upper = quotients[0];
  for (double value : quotients) {
    if (!std::isfinite(value)) return false;
    lower = std::min(lower, value);
    upper = std::max(upper, value);
  }
  result.lower = std::nextafter(lower,
                                -std::numeric_limits<double>::infinity());
  result.upper = std::nextafter(upper,
                                std::numeric_limits<double>::infinity());
  return finiteInterval(result);
}

bool closedBallBoundsExact(const Eigen::Vector3d& point,
                           const double radius,
                           const Eigen::Vector3d& lower,
                           const Eigen::Vector3d& upper,
                           Eigen::Vector3d* ball_lower,
                           Eigen::Vector3d* ball_upper) {
  if (!finite(point) || !finite(radius) || radius < 0.0 ||
      !finite(lower) || !finite(upper) ||
      (upper.array() < lower.array()).any()) {
    return false;
  }
  Eigen::Vector3d out_lower = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_upper = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    if (exactSubLessThan(point(axis), radius, lower(axis)) ||
        exactAddGreaterThan(point(axis), radius, upper(axis))) {
      return false;
    }
    DirectedInterval lo;
    DirectedInterval hi;
    if (!directedSub(intervalFromDouble(point(axis)),
                     intervalFromDouble(radius), lo) ||
        !directedAdd(intervalFromDouble(point(axis)),
                     intervalFromDouble(radius), hi)) {
      return false;
    }
    out_lower(axis) = lo.lower;
    out_upper(axis) = hi.upper;
  }
  if (ball_lower != nullptr) *ball_lower = out_lower;
  if (ball_upper != nullptr) *ball_upper = out_upper;
  return true;
}

bool exactGridBounds(const Eigen::Vector3d& origin,
                     const Eigen::Vector3i& source_min,
                     const Eigen::Vector3i& count,
                     const double resolution,
                     Eigen::Vector3d& lower,
                     Eigen::Vector3d& upper) {
  if (!finite(origin) || !finite(resolution) || resolution <= 0.0 ||
      count.x() <= 0 || count.y() <= 0 || count.z() <= 0) {
    return false;
  }
  lower = origin;
  upper = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const long long absolute_first = static_cast<long long>(source_min(axis));
    const long long absolute_last = absolute_first +
        static_cast<long long>(count(axis));
    if (absolute_first < 0 || absolute_last <= absolute_first ||
        absolute_last > static_cast<long long>(std::numeric_limits<int>::max()) +
            1LL) {
      return false;
    }
    DirectedInterval product;
    DirectedInterval sum;
    if (!directedMul(intervalFromDouble(resolution),
                     intervalFromDouble(static_cast<double>(absolute_last)),
                     product) ||
        !directedAdd(intervalFromDouble(origin(axis)), product, sum)) {
      return false;
    }
    // The native grid lower boundary is offset by the crop's integer source
    // index.  Compute that offset with the same directed enclosure rather
    // than rebasing/storing a rounded crop origin.
    DirectedInterval lower_product;
    DirectedInterval lower_sum;
    if (!directedMul(intervalFromDouble(resolution),
                     intervalFromDouble(static_cast<double>(absolute_first)),
                     lower_product) ||
        !directedAdd(intervalFromDouble(origin(axis)), lower_product,
                     lower_sum)) {
      return false;
    }
    lower(axis) = lower_sum.lower;
    upper(axis) = sum.upper;
    if (!finite(upper(axis))) return false;
  }
  return (upper.array() > lower.array()).all();
}

bool exactGridBounds(const Eigen::Vector3d& origin,
                     const Eigen::Vector3i& count,
                     const double resolution,
                     Eigen::Vector3d& lower,
                     Eigen::Vector3d& upper) {
  return exactGridBounds(origin, Eigen::Vector3i::Zero(), count, resolution,
                         lower, upper);
}

bool captureIndexRange(const double lower_coordinate,
                       const double upper_coordinate,
                       const int count, int& first, int& last) {
  if (!finite(lower_coordinate) || !finite(upper_coordinate) || count <= 0 ||
      lower_coordinate > upper_coordinate) {
    return false;
  }
  // Outward one-ulp expansion makes closed contacts survive conversion to
  // integer voxel indices.  The -1 term accounts for a voxel's upper face:
  // i+1 >= lower and i <= upper.
  const double lower_out = std::nextafter(
      lower_coordinate, -std::numeric_limits<double>::infinity());
  const double upper_out = std::nextafter(
      upper_coordinate, std::numeric_limits<double>::infinity());
  const double first_value = std::ceil(std::nextafter(
      lower_out - 1.0, -std::numeric_limits<double>::infinity()));
  const double last_value = std::floor(upper_out);
  if (!finite(first_value) || !finite(last_value) ||
      first_value < static_cast<double>(INT_MIN) ||
      first_value > static_cast<double>(INT_MAX) ||
      last_value < static_cast<double>(INT_MIN) ||
      last_value > static_cast<double>(INT_MAX)) {
    return false;
  }
  const int raw_first = static_cast<int>(first_value);
  const int raw_last = static_cast<int>(last_value);
  first = std::max(0, raw_first);
  last = std::min(count - 1, raw_last);
  return first <= last;
}

bool captureAddressChecked(const Eigen::Vector3i& index,
                           const Eigen::Vector3i& count,
                           std::size_t& result) {
  if (index.x() < 0 || index.y() < 0 || index.z() < 0 ||
      index.x() >= count.x() || index.y() >= count.y() ||
      index.z() >= count.z()) {
    return false;
  }
  std::size_t xy = 0U;
  const std::size_t x = static_cast<std::size_t>(index.x());
  const std::size_t y = static_cast<std::size_t>(index.y());
  const std::size_t z = static_cast<std::size_t>(index.z());
  const std::size_t y_count = static_cast<std::size_t>(count.y());
  const std::size_t z_count = static_cast<std::size_t>(count.z());
  if (x > std::numeric_limits<std::size_t>::max() / y_count) return false;
  xy = x * y_count;
  if (xy > std::numeric_limits<std::size_t>::max() - y) return false;
  xy += y;
  if (xy > std::numeric_limits<std::size_t>::max() / z_count) return false;
  const std::size_t base = xy * z_count;
  if (base > std::numeric_limits<std::size_t>::max() - z) return false;
  result = base + z;
  return true;
}

bool captureVoxelBounds(const SDFMapCaptureV2& capture,
                        const Eigen::Vector3i& index,
                        Eigen::Vector3d& lower,
                        Eigen::Vector3d& upper) {
  if (!finite(capture.grid_origin) || !finite(capture.resolution) ||
      capture.resolution <= 0.0) {
    return false;
  }
  Eigen::Vector3i source_min;
  Eigen::Vector3i source_max;
  if (!captureSourceIndices(capture, source_min, source_max)) return false;
  lower = Eigen::Vector3d::Zero();
  upper = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    const long long absolute_index = static_cast<long long>(source_min(axis)) +
        static_cast<long long>(index(axis));
    if (absolute_index < 0 ||
        absolute_index > std::numeric_limits<int>::max()) {
      return false;
    }
    DirectedInterval product;
    DirectedInterval lo;
    DirectedInterval hi;
    if (!directedMul(intervalFromDouble(capture.resolution),
                     intervalFromDouble(static_cast<double>(absolute_index)),
                     product) ||
        !directedAdd(intervalFromDouble(capture.grid_origin(axis)),
                     product, lo) ||
        !directedAdd(lo, intervalFromDouble(capture.resolution), hi)) {
      return false;
    }
    lower(axis) = lo.lower;
    upper(axis) = hi.upper;
    if (!finite(lower(axis)) || !finite(upper(axis))) return false;
  }
  return (upper.array() >= lower.array()).all();
}

bool closedVoxelDistanceSquaredLower(const SDFMapCaptureV2& capture,
                                     const Eigen::Vector3i& index,
                                     const Eigen::Vector3d& point,
                                     double& lower_bound) {
  Eigen::Vector3d voxel_lower;
  Eigen::Vector3d voxel_upper;
  if (!captureVoxelBounds(capture, index, voxel_lower, voxel_upper)) {
    return false;
  }
  DirectedInterval sum = intervalFromDouble(0.0);
  for (int axis = 0; axis < 3; ++axis) {
    DirectedInterval gap = intervalFromDouble(0.0);
    if (point(axis) < voxel_lower(axis)) {
      DirectedInterval difference;
      if (!directedSub(intervalFromDouble(voxel_lower(axis)),
                       intervalFromDouble(point(axis)), difference)) {
        return false;
      }
      gap.lower = std::max(0.0, difference.lower);
      gap.upper = std::max(0.0, difference.upper);
    } else if (point(axis) > voxel_upper(axis)) {
      DirectedInterval difference;
      if (!directedSub(intervalFromDouble(point(axis)),
                       intervalFromDouble(voxel_upper(axis)), difference)) {
        return false;
      }
      gap.lower = std::max(0.0, difference.lower);
      gap.upper = std::max(0.0, difference.upper);
    }
    DirectedInterval term;
    if (!directedMul(gap, gap, term) ||
        !directedAdd(sum, term, sum)) {
      return false;
    }
  }
  lower_bound = std::max(0.0, sum.lower);
  return finite(lower_bound);
}

// When directed floating-point bounds overlap at a closed-ball tangency,
// prove d^2 >= R^2 with exact dyadic arithmetic before returning
// INCONCLUSIVE.  This admits mathematically proven contact while retaining a
// fail-closed result whenever the exact comparison itself is unavailable.
bool exactVoxelDistanceSquaredCompare(const SDFMapCaptureV2& capture,
                                      const Eigen::Vector3i& index,
                                      const Eigen::Vector3d& point,
                                      const double radius,
                                      int& comparison) {
  Eigen::Vector3i source_min;
  Eigen::Vector3i source_max;
  if (!captureSourceIndices(capture, source_min, source_max) ||
      !finite(point) || !finite(radius) || radius < 0.0) {
    return false;
  }

  ExactDyadic distance_squared;
  distance_squared.mantissa = 0;
  distance_squared.exponent = 0;
  for (int axis = 0; axis < 3; ++axis) {
    const long long absolute_index = static_cast<long long>(source_min(axis)) +
        static_cast<long long>(index(axis));
    if (absolute_index < 0 ||
        absolute_index > std::numeric_limits<int>::max()) {
      return false;
    }
    const ExactDyadic origin = exactDouble(capture.grid_origin(axis));
    const ExactDyadic resolution = exactDouble(capture.resolution);
    const ExactDyadic lower = exactAdd(
        origin, exactMultiplyInteger(resolution, absolute_index));
    const ExactDyadic upper = exactAdd(lower, resolution);
    const ExactDyadic query = exactDouble(point(axis));
    ExactDyadic gap;
    if (compareExact(query, lower) < 0) {
      gap = exactSubtract(lower, query);
    } else if (compareExact(query, upper) > 0) {
      gap = exactSubtract(query, upper);
    } else {
      gap.mantissa = 0;
      gap.exponent = 0;
    }
    distance_squared = exactAdd(distance_squared, exactMultiply(gap, gap));
  }
  const ExactDyadic radius_squared = exactMultiply(exactDouble(radius),
                                                   exactDouble(radius));
  comparison = compareExact(distance_squared, radius_squared);
  return true;
}

bool squaredRadiusUpper(const double radius, double& upper) {
  DirectedInterval squared;
  if (!directedMul(intervalFromDouble(radius), intervalFromDouble(radius),
                   squared)) {
    return false;
  }
  upper = squared.upper;
  return finite(upper) && upper >= 0.0;
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

CloudOccupancySnapshotPlannerEsdfBaseClearanceResult
queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
    const CloudOccupancySnapshot& snapshot,
    const Eigen::Vector3d& point,
    const double required_radius) {
  CloudOccupancySnapshotPlannerEsdfBaseClearanceResult result;
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

  // Preserve categorical occupancy semantics before evaluating the
  // centre-set clearance: an occupied containing voxel has no certificate.
  const std::size_t point_address = address(point_index, snapshot.voxel_count);
  if (point_address >= snapshot.occupied.size()) return result;
  if (snapshot.occupied[point_address] != 0U) {
    result.status = CloudOccupancyStatus::OCCUPIED;
    return result;
  }

  double nearest = std::numeric_limits<double>::infinity();
  const CloudOccupancyColumnIndex* const column_index =
      usableOccupiedColumnIndex(snapshot,
                                snapshot.occupied_column_index.get());
  bool indexed_complete = false;
  if (column_index != nullptr) {
    const IndexedScanResult indexed_result = scanIndexedOccupiedVoxelCenters(
        snapshot, *column_index, point, first_x, last_x, first_y, last_y,
        first_z, last_z, nearest);
    if (indexed_result == IndexedScanResult::UNAVAILABLE) {
      return CloudOccupancySnapshotPlannerEsdfBaseClearanceResult();
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
          double distance = 0.0;
          if (!pointToOccupiedVoxelCenterDistance(snapshot, point, index,
                                                  distance)) {
            return CloudOccupancySnapshotPlannerEsdfBaseClearanceResult();
          }
          nearest = std::min(nearest, distance);
        }
      }
    }
  }

  if (nearest <= 0.0) {
    result.status = CloudOccupancyStatus::OCCUPIED;
    return result;
  }
  result.status = CloudOccupancyStatus::KNOWN_FREE;
  result.nearest_inflated_occupied_voxel_center_distance =
      std::isfinite(nearest) ? std::min(nearest, required_radius)
                             : required_radius;
  result.clearance_certified = true;
  return result;
}

bool sdfMapCaptureV2FloatingPointEnvironmentSupported() {
  return floatingPointEnvironmentSupportedV2();
}

bool sdfMapCaptureV2Consistent(const SDFMapCaptureV2& capture) {
  if (!floatingPointEnvironmentSupportedV2() ||
      !capture.valid || capture.map_instance_id == 0U ||
      capture.configuration_generation == 0U ||
      capture.configuration_key == 0U || capture.frame_id.empty() ||
      capture.accepted_state_sequence == 0U ||
      (capture.accepted_state_notification_sequence != 0U &&
       capture.accepted_state_notification_sequence <
           capture.accepted_state_sequence) ||
      capture.accepted_time_ticks == 0U ||
      !finite(capture.map_min) || !finite(capture.map_max) ||
      !finite(capture.grid_origin) || !finite(capture.capture_min) ||
      !finite(capture.capture_max) || !finite(capture.resolution) ||
      capture.resolution <= 0.0 ||
      !finite(capture.included_map_inflation) ||
      capture.included_map_inflation < 0.0 ||
      (capture.map_max.array() <= capture.map_min.array()).any() ||
      (capture.capture_max.array() < capture.capture_min.array()).any()) {
    return false;
  }
  std::size_t voxel_count = 0U;
  if (!validVoxelCount(capture.voxel_count, voxel_count) ||
      capture.occupied.size() != voxel_count) {
    return false;
  }
  Eigen::Vector3i source_min;
  Eigen::Vector3i source_max;
  if (!captureSourceIndices(capture, source_min, source_max)) return false;
  Eigen::Vector3d grid_min;
  Eigen::Vector3d grid_max;
  if (!exactGridBounds(capture.grid_origin, source_min, capture.voxel_count,
                       capture.resolution, grid_min, grid_max)) {
    return false;
  }
  // Capture geometry is required to be within the planner map.  Comparisons
  // use the already-directed grid bounds and never add a safety tolerance.
  for (int axis = 0; axis < 3; ++axis) {
    if (capture.capture_min(axis) < grid_min(axis) ||
        capture.capture_max(axis) > grid_max(axis) ||
        capture.capture_min(axis) < capture.map_min(axis) ||
        capture.capture_max(axis) > capture.map_max(axis) ||
        capture.map_min(axis) > capture.map_max(axis)) {
      return false;
    }
  }
  if (capture.support.valid || capture.support.complete) {
    if (!capture.support.valid || !capture.support.complete ||
        capture.support.evidence_sequence == 0U ||
        capture.support.evidence_accepted_ticks == 0U ||
        capture.support.map_instance_id != capture.map_instance_id ||
        capture.support.configuration_generation !=
            capture.configuration_generation ||
        capture.support.configuration_key != capture.configuration_key ||
        capture.support.frame_id != capture.frame_id ||
        (capture.support.evidence_basis &
             ~kKnownCaptureSupportEvidenceMask) != 0U ||
        (capture.support.evidence_basis &
             kSDFMapCaptureSupportEvidenceCompletePreknownDomain) == 0U ||
        !finite(capture.support.support_min) ||
        !finite(capture.support.support_max) ||
        (capture.support.support_max.array() <
         capture.support.support_min.array()).any() ||
        (capture.support.support_min.array() < capture.map_min.array()).any() ||
        (capture.support.support_max.array() > capture.map_max.array()).any() ||
        !finite(capture.support.required_halo) ||
        capture.support.required_halo < 0.0 ||
        !capture.support.halo_reconciled ||
        capture.support.mask.size() != voxel_count ||
        (capture.support.valid_until_accepted_ticks != 0U &&
         capture.support.valid_until.isZero()) ||
        (capture.support.valid_until_accepted_ticks == 0U &&
         !capture.support.valid_until.isZero()) ||
        (!capture.support.valid_until.isZero() &&
         capture.support.valid_until_accepted_ticks <
             capture.support.evidence_accepted_ticks) ||
        (!capture.support.valid_until.isZero() &&
         capture.accepted_time_ticks >
             capture.support.valid_until_accepted_ticks) ||
        (!capture.support.evidence_stamp.isZero() &&
         !capture.accepted_state_stamp.isZero() &&
         capture.accepted_state_stamp < capture.support.evidence_stamp) ||
        (!capture.support.valid_until.isZero() &&
         !capture.support.evidence_stamp.isZero() &&
         capture.support.valid_until < capture.support.evidence_stamp)) {
      return false;
    }
  }
  return true;
}

SDFMapCaptureFreeBallResultV2 certifySDFMapCaptureFreeBallV2(
    const SDFMapCaptureV2& capture,
    const Eigen::Vector3d& qhat,
    const double radius) {
  SDFMapCaptureFreeBallResultV2 result;
  if (!sdfMapCaptureV2Consistent(capture) || !finite(qhat) ||
      !finite(radius) || radius < 0.0) {
    return result;
  }
  if (!capture.support.valid || !capture.support.complete ||
      capture.support.evidence_sequence == 0U ||
      capture.support.evidence_accepted_ticks == 0U ||
      (capture.support.evidence_basis &
           ~kKnownCaptureSupportEvidenceMask) != 0U ||
      (capture.support.evidence_basis &
           kSDFMapCaptureSupportEvidenceCompletePreknownDomain) == 0U ||
      capture.support.mask.size() != capture.occupied.size()) {
    return result;
  }

  Eigen::Vector3d map_ball_min;
  Eigen::Vector3d map_ball_max;
  if (!closedBallBoundsExact(qhat, radius, capture.map_min, capture.map_max,
                             &map_ball_min, &map_ball_max)) {
    result.status = SDFMapCaptureFreeBallStatusV2::OUT_OF_MAP;
    return result;
  }
  Eigen::Vector3d grid_min;
  Eigen::Vector3d grid_max;
  Eigen::Vector3i source_min;
  Eigen::Vector3i source_max;
  if (!captureSourceIndices(capture, source_min, source_max) ||
      !exactGridBounds(capture.grid_origin, source_min, capture.voxel_count,
                       capture.resolution, grid_min, grid_max)) {
    return result;
  }
  // A crop boundary or an unsupported support box is UNKNOWN, while a point
  // outside the planner map is OUT_OF_MAP.
  if (!closedBallBoundsExact(qhat, radius, capture.capture_min,
                             capture.capture_max, nullptr, nullptr) ||
      !closedBallWithinExactGrid(capture.grid_origin, source_min,
                                 capture.voxel_count, capture.resolution,
                                 qhat, radius) ||
      !closedBallBoundsExact(qhat, radius, capture.support.support_min,
                             capture.support.support_max, nullptr, nullptr)) {
    result.status = SDFMapCaptureFreeBallStatusV2::UNKNOWN;
    return result;
  }

  Eigen::Vector3d ball_lower;
  Eigen::Vector3d ball_upper;
  if (!closedBallBoundsExact(qhat, radius, grid_min, grid_max,
                             &ball_lower, &ball_upper)) {
    result.status = SDFMapCaptureFreeBallStatusV2::UNKNOWN;
    return result;
  }
  int first[3] = {0, 0, 0};
  int last[3] = {-1, -1, -1};
  for (int axis = 0; axis < 3; ++axis) {
    DirectedInterval ball_lower_interval;
    DirectedInterval ball_upper_interval;
    DirectedInterval lower_offset;
    DirectedInterval upper_offset;
    DirectedInterval lower_coordinate;
    DirectedInterval upper_coordinate;
    if (!directedSub(intervalFromDouble(qhat(axis)),
                     intervalFromDouble(radius), ball_lower_interval) ||
        !directedAdd(intervalFromDouble(qhat(axis)),
                     intervalFromDouble(radius), ball_upper_interval) ||
        !directedSub(ball_lower_interval,
                     intervalFromDouble(capture.grid_origin(axis)),
                     lower_offset) ||
        !directedSub(ball_upper_interval,
                     intervalFromDouble(capture.grid_origin(axis)),
                     upper_offset) ||
        !directedDiv(lower_offset,
                     intervalFromDouble(capture.resolution),
                     lower_coordinate) ||
        !directedDiv(upper_offset,
                     intervalFromDouble(capture.resolution),
                     upper_coordinate) ||
        // Convert to native-grid coordinates before subtracting the integer
        // crop offset.  The offset is dimensionless; subtracting it from a
        // world-space numerator would be wrong whenever resolution != 1.
        !directedSub(lower_coordinate,
                     intervalFromDouble(static_cast<double>(source_min(axis))),
                     lower_coordinate) ||
        !directedSub(upper_coordinate,
                     intervalFromDouble(static_cast<double>(source_min(axis))),
                     upper_coordinate) ||
        !captureIndexRange(lower_coordinate.lower, upper_coordinate.upper,
                           capture.voxel_count(axis), first[axis],
                           last[axis])) {
      result.status = SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE;
      return result;
    }
  }
  const std::size_t x_count =
      static_cast<std::size_t>(last[0] - first[0]) + 1U;
  const std::size_t y_count =
      static_cast<std::size_t>(last[1] - first[1]) + 1U;
  const std::size_t z_count =
      static_cast<std::size_t>(last[2] - first[2]) + 1U;
  if (x_count > kMaxClearanceVoxelChecks ||
      y_count > kMaxClearanceVoxelChecks ||
      z_count > kMaxClearanceVoxelChecks ||
      x_count > kMaxClearanceVoxelChecks / y_count) {
    result.status = SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE;
    return result;
  }
  const std::size_t xy_count = x_count * y_count;
  if (xy_count > kMaxClearanceVoxelChecks / z_count) {
    result.status = SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE;
    return result;
  }

  result.enumerated_min_index = Eigen::Vector3i(first[0], first[1], first[2]);
  result.enumerated_max_index = Eigen::Vector3i(last[0], last[1], last[2]);
  double radius_squared_upper = 0.0;
  if (!squaredRadiusUpper(radius, radius_squared_upper)) {
    result.status = SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE;
    return result;
  }
  result.support_min = ball_lower;
  result.support_max = ball_upper;

  for (int x = first[0]; x <= last[0]; ++x) {
    for (int y = first[1]; y <= last[1]; ++y) {
      for (int z = first[2]; z <= last[2]; ++z) {
        ++result.voxel_checks;
        const Eigen::Vector3i index(x, y, z);
        std::size_t voxel_address = 0U;
        if (!captureAddressChecked(index, capture.voxel_count,
                                   voxel_address) ||
            voxel_address >= capture.occupied.size() ||
            voxel_address >= capture.support.mask.size()) {
          result.status = SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE;
          result.clearance_certified = false;
          return result;
        }
        if (capture.support.mask[voxel_address] == 0U) {
          result.status = SDFMapCaptureFreeBallStatusV2::UNKNOWN;
          result.clearance_certified = false;
          return result;
        }
        if (capture.occupied[voxel_address] == 0U) continue;

        Eigen::Vector3d voxel_lower;
        Eigen::Vector3d voxel_upper;
        if (!captureVoxelBounds(capture, index, voxel_lower, voxel_upper)) {
          result.status = SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE;
          return result;
        }
        const bool point_inside = exactVoxelContains(capture, index, qhat);
        double distance_squared_lower = 0.0;
        if (!closedVoxelDistanceSquaredLower(
                capture, index, qhat, distance_squared_lower)) {
          result.status = SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE;
          return result;
        }
        if (point_inside) {
          result.status = SDFMapCaptureFreeBallStatusV2::OCCUPIED;
          result.clearance_certified = false;
          result.certified_radius = 0.0;
          return result;
        }
        if (distance_squared_lower < radius_squared_upper) {
          int exact_comparison = -1;
          if (!exactVoxelDistanceSquaredCompare(
                  capture, index, qhat, radius, exact_comparison) ||
              exact_comparison < 0) {
            // The conservative lower bound cannot establish separation from
            // the requested ball.  Exact dyadic comparison admits only a
            // mathematically proved d >= R tangency; all other uncertainty
            // remains fail-closed.
            result.status = SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE;
            result.clearance_certified = false;
            return result;
          }
        }
      }
    }
  }
  result.status = SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE;
  result.clearance_certified = true;
  result.certified_radius = radius;
  return result;
}

}  // namespace plan_env
