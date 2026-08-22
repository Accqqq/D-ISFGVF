#include <gtest/gtest.h>

#include <limits>
#include <vector>

#define private public
#include <plan_env/sdf_map.h>
#undef private

#include "bspline_race/integration/phase_offset_raw_occupancy_query.h"

namespace FLAG_Race {
namespace {

using phase_offset_navigation::DistanceStatus;
using RawProbeEvidence = RawOccupancyProbeEvidence;

void InitializeRawMap(SDFMap& map) {
  map.mp_.resolution_ = 1.0;
  map.mp_.resolution_inv_ = 1.0;
  map.mp_.map_origin_ = Eigen::Vector3d::Zero();
  map.mp_.map_size_ = Eigen::Vector3d(10.0, 10.0, 10.0);
  map.mp_.map_voxel_num_ = Eigen::Vector3i(10, 10, 10);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
  map.mp_.clamp_min_log_ = -2.0;
  map.mp_.min_occupancy_log_ = 0.0;
  const std::size_t size = 1000U;
  map.md_.occupancy_buffer_.assign(size, -1.0);
  // Deliberately poison scalar ESDF values.  KNOWN_FREE below proves this
  // bridge is driven only by raw occupancy status, not scalar distance.
  map.md_.distance_buffer_all_.assign(
      size, std::numeric_limits<double>::quiet_NaN());
  map.md_.occupancy_buffer_inflate_.assign(size, 1);
  map.md_.manual_boundary_enabled_ = false;
}

int AddressFor(const SDFMap& map, const Eigen::Vector3d& point) {
  Eigen::Vector3i index;
  const_cast<SDFMap&>(map).posToIndex(point, index);
  return const_cast<SDFMap&>(map).toAddress(index);
}

void SetRawLogOdds(SDFMap& map, const Eigen::Vector3d& point,
                   const double value) {
  map.md_.occupancy_buffer_[AddressFor(map, point)] = value;
}

TEST(PhaseOffsetRawOccupancyQueryTest, MapsRawStatesInRequiredOrder) {
  const Eigen::Vector3d inside(1.0, 1.0, 1.0);
  EXPECT_EQ(makeRawOccupancyQuery(nullptr)(inside), DistanceStatus::UNAVAILABLE);

  SDFMap map;
  InitializeRawMap(map);
  const auto query = makeRawOccupancyQuery(&map);
  EXPECT_EQ(query(Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN())),
            DistanceStatus::OUT_OF_MAP);
  EXPECT_EQ(query(Eigen::Vector3d(-1.0, 1.0, 1.0)),
            DistanceStatus::OUT_OF_MAP);

  const int address = AddressFor(map, inside);
  map.md_.occupancy_buffer_[address] = -3.0;
  EXPECT_EQ(query(inside), DistanceStatus::UNKNOWN);

  map.md_.occupancy_buffer_[address] = 1.0;
  EXPECT_EQ(query(inside), DistanceStatus::OCCUPIED);

  map.md_.occupancy_buffer_[address] = -1.0;
  EXPECT_EQ(query(inside), DistanceStatus::KNOWN_FREE);
}

TEST(PhaseOffsetRawOccupancyQueryTest, MissingRawStorageFailsClosed) {
  SDFMap map;
  map.mp_.map_voxel_num_ = Eigen::Vector3i(4, 4, 4);
  map.mp_.map_min_boundary_ = Eigen::Vector3d::Zero();
  map.mp_.map_max_boundary_ = Eigen::Vector3d(4.0, 4.0, 4.0);
  RawOccupancySelfFreeSeed seed;
  seed.center = Eigen::Vector3d(1.0, 1.0, 1.0);
  seed.radius = 0.25;
  EXPECT_FALSE(rawOccupancyStorageReady(&map));
  EXPECT_EQ(makeRawOccupancyQuery(&map, seed)(Eigen::Vector3d(1.0, 1.0, 1.0)),
            DistanceStatus::UNAVAILABLE);
  EXPECT_EQ(makeRawOccupancyProbe(&map, seed)(Eigen::Vector3d(1.0, 1.0, 1.0)).evidence,
            RawProbeEvidence::UNAVAILABLE);
}

TEST(PhaseOffsetRawOccupancyQueryTest,
     InflatedOnlyVoxelIsRawFreeWithoutAnEsdfRead) {
  SDFMap map;
  InitializeRawMap(map);
  const Eigen::Vector3d point(2.0, 2.0, 2.0);
  const int address = AddressFor(map, point);
  map.md_.occupancy_buffer_[address] = -1.0;
  map.md_.occupancy_buffer_inflate_[address] = 1;
  EXPECT_EQ(map.getInflateOccupancy(point), 1);
  EXPECT_EQ(makeRawOccupancyQuery(&map)(point), DistanceStatus::KNOWN_FREE);
}

TEST(PhaseOffsetRawOccupancyQueryTest,
     SelfFreeSeedOverlaysOnlyUnknownInsideItsExactThreeDimensionalSphere) {
  SDFMap map;
  InitializeRawMap(map);
  RawOccupancySelfFreeSeed seed;
  seed.center = Eigen::Vector3d(2.0, 2.0, 2.0);
  seed.radius = 0.25;
  const Eigen::Vector3d inside(2.10, 2.0, 2.0);
  const Eigen::Vector3d on_boundary(2.25, 2.0, 2.0);
  const Eigen::Vector3d just_beyond(2.250001, 2.0, 2.0);
  const Eigen::Vector3d outside_seed(3.0, 2.0, 2.0);
  SetRawLogOdds(map, inside, -3.0);
  SetRawLogOdds(map, outside_seed, -3.0);

  const auto probe = makeRawOccupancyProbe(&map, seed);
  EXPECT_EQ(probe(inside).status, DistanceStatus::KNOWN_FREE);
  EXPECT_EQ(probe(inside).evidence, RawProbeEvidence::KNOWN_FREE_BY_SELF);
  EXPECT_EQ(probe(on_boundary).status, DistanceStatus::KNOWN_FREE);
  EXPECT_EQ(probe(on_boundary).evidence,
            RawProbeEvidence::KNOWN_FREE_BY_SELF);
  EXPECT_EQ(probe(just_beyond).status, DistanceStatus::UNKNOWN);
  EXPECT_EQ(probe(just_beyond).evidence, RawProbeEvidence::RAW_UNKNOWN);
  EXPECT_EQ(probe(outside_seed).status, DistanceStatus::UNKNOWN);
  EXPECT_EQ(probe(outside_seed).evidence, RawProbeEvidence::RAW_UNKNOWN);

  // A raw occupied cell always wins, even if it lies at the seed center.
  SetRawLogOdds(map, inside, 1.0);
  EXPECT_EQ(probe(inside).status, DistanceStatus::OCCUPIED);
  EXPECT_EQ(probe(inside).evidence, RawProbeEvidence::RAW_OCCUPIED);

  // Observed raw free remains raw-map evidence both inside and outside.
  SetRawLogOdds(map, inside, -1.0);
  SetRawLogOdds(map, outside_seed, -1.0);
  EXPECT_EQ(probe(inside).status, DistanceStatus::KNOWN_FREE);
  EXPECT_EQ(probe(inside).evidence, RawProbeEvidence::RAW_KNOWN_FREE);
  EXPECT_EQ(probe(outside_seed).status, DistanceStatus::KNOWN_FREE);
  EXPECT_EQ(probe(outside_seed).evidence, RawProbeEvidence::RAW_KNOWN_FREE);
}

TEST(PhaseOffsetRawOccupancyQueryTest,
     SeedCannotOverlayOutOfMapOrNonfinitePointsAndInvalidSeedsAreDisabled) {
  SDFMap map;
  InitializeRawMap(map);
  RawOccupancySelfFreeSeed boundary_seed;
  boundary_seed.center = Eigen::Vector3d(0.05, 1.0, 1.0);
  boundary_seed.radius = 1.0;
  const auto boundary_probe = makeRawOccupancyProbe(&map, boundary_seed);
  EXPECT_EQ(boundary_probe(Eigen::Vector3d(-0.01, 1.0, 1.0)).status,
            DistanceStatus::OUT_OF_MAP);
  EXPECT_EQ(boundary_probe(Eigen::Vector3d(-0.01, 1.0, 1.0)).evidence,
            RawProbeEvidence::OUT_OF_MAP);
  EXPECT_EQ(boundary_probe(Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN())).status,
            DistanceStatus::OUT_OF_MAP);

  const Eigen::Vector3d unknown_point(4.0, 4.0, 4.0);
  SetRawLogOdds(map, unknown_point, -3.0);
  RawOccupancySelfFreeSeed invalid_center = boundary_seed;
  invalid_center.center.x() = std::numeric_limits<double>::quiet_NaN();
  RawOccupancySelfFreeSeed nonfinite_radius = boundary_seed;
  nonfinite_radius.radius = std::numeric_limits<double>::quiet_NaN();
  RawOccupancySelfFreeSeed zero_radius = boundary_seed;
  zero_radius.radius = 0.0;
  RawOccupancySelfFreeSeed negative_radius = boundary_seed;
  negative_radius.radius = -0.25;
  for (const RawOccupancySelfFreeSeed& invalid_seed :
       {invalid_center, nonfinite_radius, zero_radius, negative_radius}) {
    const auto probe = makeRawOccupancyProbe(&map, invalid_seed);
    EXPECT_EQ(probe(unknown_point).status, DistanceStatus::UNKNOWN);
    EXPECT_EQ(probe(unknown_point).evidence, RawProbeEvidence::RAW_UNKNOWN);
  }
}

TEST(PhaseOffsetRawOccupancyQueryTest,
     NoSeedStatusSemanticsRemainExactlyEquivalent) {
  SDFMap map;
  InitializeRawMap(map);
  const Eigen::Vector3d known_free(1.0, 1.0, 1.0);
  const Eigen::Vector3d unknown(2.0, 2.0, 2.0);
  const Eigen::Vector3d occupied(3.0, 3.0, 3.0);
  SetRawLogOdds(map, unknown, -3.0);
  SetRawLogOdds(map, occupied, 1.0);
  RawOccupancySelfFreeSeed no_seed;
  no_seed.center = Eigen::Vector3d::Zero();
  no_seed.radius = 0.0;
  const auto original = makeRawOccupancyQuery(&map);
  const auto explicit_no_seed = makeRawOccupancyQuery(&map, no_seed);
  const std::vector<Eigen::Vector3d> points = {
      known_free, unknown, occupied, Eigen::Vector3d(-1.0, 1.0, 1.0),
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};
  for (const Eigen::Vector3d& point : points) {
    EXPECT_EQ(original(point), explicit_no_seed(point));
  }
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
