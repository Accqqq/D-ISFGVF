#include "bspline_race/integration/phase_offset_raw_occupancy_query.h"

// DEPRECATED TEST-ONLY REFERENCE (A5-G2d).  This translation unit is linked
// only by phase_offset_raw_occupancy_query_test.

#include <plan_env/sdf_map.h>

#include <cmath>
#include <limits>

namespace FLAG_Race {
namespace {

bool validSelfFreeSeed(const RawOccupancySelfFreeSeed& seed) {
  return seed.center.allFinite() && std::isfinite(seed.radius) &&
      seed.radius > 0.0;
}

bool selfFreeSeedContains(const RawOccupancySelfFreeSeed& seed,
                          const Eigen::Vector3d& point) {
  return validSelfFreeSeed(seed) &&
      (point - seed.center).squaredNorm() <= seed.radius * seed.radius;
}

RawOccupancyProbeResult ProbeRawOccupancy(
    SDFMap* map, const RawOccupancySelfFreeSeed& seed,
    const Eigen::Vector3d& point) {
  using phase_offset_navigation::DistanceStatus;

  if (!rawOccupancyStorageReady(map)) {
    return {DistanceStatus::UNAVAILABLE, RawOccupancyProbeEvidence::UNAVAILABLE};
  }
  if (!point.allFinite() || !map->isInMap(point)) {
    return {DistanceStatus::OUT_OF_MAP, RawOccupancyProbeEvidence::OUT_OF_MAP};
  }
  if (map->getOccupancy(point) != 0) {
    return {DistanceStatus::OCCUPIED, RawOccupancyProbeEvidence::RAW_OCCUPIED};
  }
  if (!map->isUnknown(point)) {
    return {DistanceStatus::KNOWN_FREE,
            RawOccupancyProbeEvidence::RAW_KNOWN_FREE};
  }
  if (selfFreeSeedContains(seed, point)) {
    return {DistanceStatus::KNOWN_FREE,
            RawOccupancyProbeEvidence::KNOWN_FREE_BY_SELF};
  }
  return {DistanceStatus::UNKNOWN, RawOccupancyProbeEvidence::RAW_UNKNOWN};
}

}  // namespace

bool rawOccupancyStorageReady(const SDFMap* map) {
  if (map == nullptr) return false;
  const Eigen::Vector3i voxels = map->mp_.map_voxel_num_;
  if (voxels.x() <= 0 || voxels.y() <= 0 || voxels.z() <= 0) return false;
  const std::size_t x = static_cast<std::size_t>(voxels.x());
  const std::size_t y = static_cast<std::size_t>(voxels.y());
  const std::size_t z = static_cast<std::size_t>(voxels.z());
  if (x > std::numeric_limits<std::size_t>::max() / y) return false;
  const std::size_t xy = x * y;
  if (xy > std::numeric_limits<std::size_t>::max() / z) return false;
  return map->md_.occupancy_buffer_.size() >= xy * z;
}

RawOccupancyProbe makeRawOccupancyProbe(
    SDFMap* map, const RawOccupancySelfFreeSeed& seed) {
  return [map, seed](const Eigen::Vector3d& point) {
    return ProbeRawOccupancy(map, seed, point);
  };
}

RawOccupancyProbe makeRawOccupancyProbe(SDFMap* map) {
  return makeRawOccupancyProbe(map, RawOccupancySelfFreeSeed());
}

phase_offset_navigation::RawOccupancyQuery makeRawOccupancyQuery(
    SDFMap* map, const RawOccupancySelfFreeSeed& seed) {
  const RawOccupancyProbe probe = makeRawOccupancyProbe(map, seed);
  return [probe](const Eigen::Vector3d& point) { return probe(point).status; };
}

phase_offset_navigation::RawOccupancyQuery makeRawOccupancyQuery(SDFMap* map) {
  return makeRawOccupancyQuery(map, RawOccupancySelfFreeSeed());
}

}  // namespace FLAG_Race
