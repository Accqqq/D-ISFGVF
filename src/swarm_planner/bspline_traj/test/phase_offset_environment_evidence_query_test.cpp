#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#define private public
#include <plan_env/sdf_map.h>
#undef private

#include "bspline_race/integration/phase_offset_environment_evidence_query.h"

namespace FLAG_Race {
namespace {

using phase_offset_navigation::DistanceStatus;
using phase_offset_navigation::EnvironmentEvidenceResult;
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
  map.md_.occupancy_buffer_.assign(1000U, -1.0);
  map.md_.occupancy_buffer_inflate_.assign(1000U, 1);
  map.md_.distance_buffer_all_.assign(
      1000U, std::numeric_limits<double>::quiet_NaN());
  map.md_.manual_boundary_enabled_ = false;
}

SDFMapEnvironmentEvidenceResult InvalidResult(
    const SDFMapEnvironmentEvidenceStatus status) {
  SDFMapEnvironmentEvidenceResult result;
  result.status = status;
  return result;
}

SDFMapEnvironmentEvidenceResult OccupiedResult() {
  SDFMapEnvironmentEvidenceResult result;
  result.status = SDFMapEnvironmentEvidenceStatus::OCCUPIED;
  result.distance_valid = true;
  result.included_layer_mask =
      plan_env::kSDFMapEnvironmentLayerSensorUninflated;
  return result;
}

SDFMapEnvironmentEvidenceResult KnownFreeResult() {
  SDFMapEnvironmentEvidenceResult result;
  result.status = SDFMapEnvironmentEvidenceStatus::KNOWN_FREE;
  result.uninflated_distance = 1.75;
  result.distance_valid = true;
  result.included_layer_mask =
      plan_env::kSDFMapEnvironmentLayerManualUninflated;
  return result;
}

void ExpectFailClosed(const EnvironmentEvidenceResult& result) {
  EXPECT_EQ(result.status, DistanceStatus::UNAVAILABLE);
  EXPECT_FALSE(result.distance_valid);
  EXPECT_DOUBLE_EQ(result.uninflated_distance, 0.0);
  EXPECT_EQ(result.included_layer_mask,
            phase_offset_navigation::kEnvironmentLayerNone);
  EXPECT_EQ(result.unsupported_active_layer_mask,
            phase_offset_navigation::kEnvironmentLayerNone);
  EXPECT_TRUE(phase_offset_navigation::environmentEvidenceConsistent(result));
  EXPECT_FALSE(phase_offset_navigation::environmentEvidenceHasUsableDistance(
      result));
}

TEST(PhaseOffsetEnvironmentEvidenceQueryTest, TranslatesEveryConsistentStatusExactly) {
  const std::vector<SDFMapEnvironmentEvidenceResult> inputs = {
      InvalidResult(SDFMapEnvironmentEvidenceStatus::UNAVAILABLE),
      InvalidResult(SDFMapEnvironmentEvidenceStatus::OUT_OF_MAP),
      InvalidResult(SDFMapEnvironmentEvidenceStatus::UNKNOWN),
      OccupiedResult(),
      KnownFreeResult(),
  };
  for (const SDFMapEnvironmentEvidenceResult& input : inputs) {
    ASSERT_TRUE(plan_env::sdfMapEnvironmentEvidenceConsistent(input));
    const EnvironmentEvidenceResult output =
        translateSDFMapEnvironmentEvidence(input);
    EXPECT_EQ(static_cast<int>(output.status), static_cast<int>(input.status));
    EXPECT_EQ(output.distance_valid, input.distance_valid);
    EXPECT_DOUBLE_EQ(output.uninflated_distance, input.uninflated_distance);
    EXPECT_EQ(output.included_layer_mask, input.included_layer_mask);
    EXPECT_EQ(output.unsupported_active_layer_mask,
              input.unsupported_active_layer_mask);
    EXPECT_TRUE(phase_offset_navigation::environmentEvidenceConsistent(output));
  }
}

TEST(PhaseOffsetEnvironmentEvidenceQueryTest, NullMapReturnsFiniteUnavailable) {
  const auto query = makeEnvironmentEvidenceQuery(nullptr);
  ExpectFailClosed(query(Eigen::Vector3d::Zero()));
}

TEST(PhaseOffsetEnvironmentEvidenceQueryTest, CurrentInsideMapResultRemainsUnavailable) {
  SDFMap map;
  InitializeMap(map);
  const auto query = makeEnvironmentEvidenceQuery(&map);
  const EnvironmentEvidenceResult result = query(Eigen::Vector3d(1.0, 1.0, 1.0));
  EXPECT_EQ(result.status, DistanceStatus::UNAVAILABLE);
  EXPECT_FALSE(result.distance_valid);
  EXPECT_DOUBLE_EQ(result.uninflated_distance, 0.0);
  EXPECT_EQ(result.unsupported_active_layer_mask,
            plan_env::kSDFMapCurrentUnsupportedEnvironmentLayerMask);
  EXPECT_TRUE(phase_offset_navigation::environmentEvidenceConsistent(result));
}

TEST(PhaseOffsetEnvironmentEvidenceQueryTest, OutOfMapTranslationIsExact) {
  SDFMap map;
  InitializeMap(map);
  const auto query = makeEnvironmentEvidenceQuery(&map);
  const EnvironmentEvidenceResult result =
      query(Eigen::Vector3d(-1.0, 1.0, 1.0));
  EXPECT_EQ(result.status, DistanceStatus::OUT_OF_MAP);
  EXPECT_FALSE(result.distance_valid);
  EXPECT_DOUBLE_EQ(result.uninflated_distance, 0.0);
  EXPECT_EQ(result.unsupported_active_layer_mask,
            plan_env::kSDFMapCurrentUnsupportedEnvironmentLayerMask);
}

TEST(PhaseOffsetEnvironmentEvidenceQueryTest, InconsistentSourceFailsClosed) {
  SDFMapEnvironmentEvidenceResult invalid = KnownFreeResult();
  invalid.uninflated_distance = 0.0;
  ExpectFailClosed(translateSDFMapEnvironmentEvidence(invalid));

  invalid = KnownFreeResult();
  invalid.unsupported_active_layer_mask = invalid.included_layer_mask;
  ExpectFailClosed(translateSDFMapEnvironmentEvidence(invalid));
}

TEST(PhaseOffsetEnvironmentEvidenceQueryTest, ValidLayerMasksRemainExact) {
  SDFMapEnvironmentEvidenceResult input = KnownFreeResult();
  input.included_layer_mask =
      plan_env::kSDFMapEnvironmentLayerSensorUninflated |
      plan_env::kSDFMapEnvironmentLayerGlobalStaticTest;
  ASSERT_TRUE(plan_env::sdfMapEnvironmentEvidenceConsistent(input));
  const EnvironmentEvidenceResult output =
      translateSDFMapEnvironmentEvidence(input);
  EXPECT_EQ(output.included_layer_mask, input.included_layer_mask);
  EXPECT_EQ(output.unsupported_active_layer_mask,
            input.unsupported_active_layer_mask);
  EXPECT_TRUE(phase_offset_navigation::environmentEvidenceHasUsableDistance(
      output));
}

TEST(PhaseOffsetEnvironmentEvidenceQueryTest, TranslatedNumericFieldsAreAlwaysFinite) {
  SDFMap map;
  InitializeMap(map);
  const auto query = makeEnvironmentEvidenceQuery(&map);
  const std::vector<EnvironmentEvidenceResult> results = {
      query(Eigen::Vector3d(1.0, 1.0, 1.0)),
      query(Eigen::Vector3d(-1.0, 1.0, 1.0)),
      query(Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())),
      translateSDFMapEnvironmentEvidence(OccupiedResult()),
      translateSDFMapEnvironmentEvidence(KnownFreeResult()),
  };
  for (const EnvironmentEvidenceResult& result : results) {
    EXPECT_TRUE(std::isfinite(result.uninflated_distance));
  }
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
