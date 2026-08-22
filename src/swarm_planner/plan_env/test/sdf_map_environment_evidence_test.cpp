#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#define private public
#include <plan_env/sdf_map.h>
#undef private

namespace {

using plan_env::SDFMapEnvironmentEvidenceResult;
using plan_env::SDFMapEnvironmentEvidenceStatus;

void InitializeMap(SDFMap& map) {
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
  map.md_.occupancy_buffer_inflate_.assign(size, 0);
  map.md_.distance_buffer_all_.assign(size, 2.0);
  map.md_.manual_occupancy_buffer_.assign(size, 0);
  map.md_.static_preinflated_buffer_.assign(size, 0);
  map.md_.manual_boundary_enabled_ = false;
}

int AddressFor(SDFMap& map, const Eigen::Vector3d& point) {
  Eigen::Vector3i index;
  map.posToIndex(point, index);
  return map.toAddress(index);
}

void ExpectFiniteFailClosed(const SDFMapEnvironmentEvidenceResult& result,
                            const SDFMapEnvironmentEvidenceStatus status) {
  EXPECT_EQ(result.status, status);
  EXPECT_FALSE(result.distance_valid);
  EXPECT_DOUBLE_EQ(result.uninflated_distance, 0.0);
  EXPECT_TRUE(std::isfinite(result.uninflated_distance));
  EXPECT_EQ(result.included_layer_mask,
            plan_env::kSDFMapEnvironmentLayerNone);
  EXPECT_EQ(result.unsupported_active_layer_mask,
            plan_env::kSDFMapCurrentUnsupportedEnvironmentLayerMask);
  EXPECT_TRUE(plan_env::sdfMapEnvironmentEvidenceConsistent(result));
}

TEST(SDFMapEnvironmentEvidenceTest, CapabilitiesAdvertiseNoCurrentBacking) {
  SDFMap map;
  const auto capabilities = map.observedUninflatedEnvironmentCapabilities();
  EXPECT_FALSE(capabilities.observation_status_available);
  EXPECT_FALSE(capabilities.uninflated_distance_available);
  EXPECT_EQ(capabilities.supported_layer_mask,
            plan_env::kSDFMapEnvironmentLayerNone);
}

TEST(SDFMapEnvironmentEvidenceTest, NonfiniteAndOutsidePointsAreOutOfMap) {
  SDFMap map;
  InitializeMap(map);
  ExpectFiniteFailClosed(
      map.queryObservedUninflatedEnvironment(
          Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())),
      SDFMapEnvironmentEvidenceStatus::OUT_OF_MAP);
  ExpectFiniteFailClosed(
      map.queryObservedUninflatedEnvironment(Eigen::Vector3d(-1.0, 1.0, 1.0)),
      SDFMapEnvironmentEvidenceStatus::OUT_OF_MAP);
}

TEST(SDFMapEnvironmentEvidenceTest, EveryFiniteInsidePointIsUnavailable) {
  SDFMap map;
  InitializeMap(map);
  ExpectFiniteFailClosed(
      map.queryObservedUninflatedEnvironment(Eigen::Vector3d(1.0, 1.0, 1.0)),
      SDFMapEnvironmentEvidenceStatus::UNAVAILABLE);
}

TEST(SDFMapEnvironmentEvidenceTest, RawAndEsdfValuesCannotClaimEvidence) {
  SDFMap map;
  InitializeMap(map);
  const Eigen::Vector3d point(2.0, 2.0, 2.0);
  const int address = AddressFor(map, point);

  map.md_.occupancy_buffer_[address] = -1.0;
  map.md_.distance_buffer_all_[address] = 3.0;
  ExpectFiniteFailClosed(map.queryObservedUninflatedEnvironment(point),
                         SDFMapEnvironmentEvidenceStatus::UNAVAILABLE);

  map.md_.occupancy_buffer_[address] = 1.0;
  map.md_.distance_buffer_all_[address] = 0.0;
  ExpectFiniteFailClosed(map.queryObservedUninflatedEnvironment(point),
                         SDFMapEnvironmentEvidenceStatus::UNAVAILABLE);

  map.md_.distance_buffer_all_[address] = 10000.0;
  ExpectFiniteFailClosed(map.queryObservedUninflatedEnvironment(point),
                         SDFMapEnvironmentEvidenceStatus::UNAVAILABLE);
}

TEST(SDFMapEnvironmentEvidenceTest, InflatedManualAndStaticStatesCannotClaimEvidence) {
  SDFMap map;
  InitializeMap(map);
  const Eigen::Vector3d point(3.0, 3.0, 3.0);
  const int address = AddressFor(map, point);

  map.md_.occupancy_buffer_inflate_[address] = 1;
  ExpectFiniteFailClosed(map.queryObservedUninflatedEnvironment(point),
                         SDFMapEnvironmentEvidenceStatus::UNAVAILABLE);
  map.md_.occupancy_buffer_inflate_[address] = 0;
  ExpectFiniteFailClosed(map.queryObservedUninflatedEnvironment(point),
                         SDFMapEnvironmentEvidenceStatus::UNAVAILABLE);

  map.md_.manual_occupancy_buffer_[address] = 1;
  map.md_.occupancy_buffer_inflate_[address] = 1;
  ExpectFiniteFailClosed(map.queryObservedUninflatedEnvironment(point),
                         SDFMapEnvironmentEvidenceStatus::UNAVAILABLE);

  map.md_.static_preinflated_buffer_[address] = 1;
  ExpectFiniteFailClosed(map.queryObservedUninflatedEnvironment(point),
                         SDFMapEnvironmentEvidenceStatus::UNAVAILABLE);
}

TEST(SDFMapEnvironmentEvidenceTest, ResultFieldsRemainFiniteForBothOutcomes) {
  SDFMap map;
  InitializeMap(map);
  const std::vector<SDFMapEnvironmentEvidenceResult> results = {
      map.queryObservedUninflatedEnvironment(Eigen::Vector3d(1.0, 1.0, 1.0)),
      map.queryObservedUninflatedEnvironment(Eigen::Vector3d(-1.0, 1.0, 1.0)),
  };
  for (const SDFMapEnvironmentEvidenceResult& result : results) {
    EXPECT_TRUE(std::isfinite(result.uninflated_distance));
    EXPECT_TRUE(plan_env::sdfMapEnvironmentEvidenceConsistent(result));
  }
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
