#include <gtest/gtest.h>

#include "phase_offset_navigation/environment_evidence.h"

#include <cmath>
#include <limits>
#include <vector>

namespace phase_offset_navigation {
namespace {

EnvironmentEvidenceResult InvalidStatus(const DistanceStatus status) {
  EnvironmentEvidenceResult result;
  result.status = status;
  return result;
}

EnvironmentEvidenceResult OccupiedResult() {
  EnvironmentEvidenceResult result;
  result.status = DistanceStatus::OCCUPIED;
  result.distance_valid = true;
  result.included_layer_mask = kEnvironmentLayerSensorUninflated;
  return result;
}

EnvironmentEvidenceResult KnownFreeResult(const double distance = 1.25) {
  EnvironmentEvidenceResult result;
  result.status = DistanceStatus::KNOWN_FREE;
  result.uninflated_distance = distance;
  result.distance_valid = true;
  result.included_layer_mask = kEnvironmentLayerSensorUninflated;
  return result;
}

TEST(EnvironmentEvidenceTest, InvalidStatusesRequireInvalidZeroDistance) {
  const DistanceStatus statuses[] = {
      DistanceStatus::UNAVAILABLE,
      DistanceStatus::OUT_OF_MAP,
      DistanceStatus::UNKNOWN,
  };
  for (const DistanceStatus status : statuses) {
    const EnvironmentEvidenceResult valid = InvalidStatus(status);
    EXPECT_TRUE(environmentEvidenceConsistent(valid));
    EXPECT_FALSE(environmentEvidenceHasUsableDistance(valid));

    EnvironmentEvidenceResult nonzero = valid;
    nonzero.uninflated_distance = 0.01;
    EXPECT_FALSE(environmentEvidenceConsistent(nonzero));

    EnvironmentEvidenceResult distance_valid = valid;
    distance_valid.distance_valid = true;
    EXPECT_FALSE(environmentEvidenceConsistent(distance_valid));
  }
}

TEST(EnvironmentEvidenceTest, OccupiedAndKnownFreeRequireExplicitProvenance) {
  const EnvironmentEvidenceResult occupied = OccupiedResult();
  EXPECT_TRUE(environmentEvidenceConsistent(occupied));
  EXPECT_TRUE(environmentEvidenceHasUsableDistance(occupied));

  const EnvironmentEvidenceResult known_free = KnownFreeResult();
  EXPECT_TRUE(environmentEvidenceConsistent(known_free));
  EXPECT_TRUE(environmentEvidenceHasUsableDistance(known_free));

  EnvironmentEvidenceResult missing_layer = known_free;
  missing_layer.included_layer_mask = kEnvironmentLayerNone;
  EXPECT_FALSE(environmentEvidenceConsistent(missing_layer));

  EnvironmentEvidenceResult zero_free = known_free;
  zero_free.uninflated_distance = 0.0;
  EXPECT_FALSE(environmentEvidenceConsistent(zero_free));
}

TEST(EnvironmentEvidenceTest, NonfiniteNegativeAndSentinelDistancesAreRejected) {
  const double invalid_distances[] = {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -0.01,
      10000.0,
  };
  for (const double distance : invalid_distances) {
    EXPECT_FALSE(environmentEvidenceConsistent(KnownFreeResult(distance)));
    EXPECT_FALSE(environmentEvidenceHasUsableDistance(KnownFreeResult(distance)));
  }
}

TEST(EnvironmentEvidenceTest, LayerMasksMustBeKnownAndNonOverlapping) {
  EnvironmentEvidenceResult overlap = KnownFreeResult();
  overlap.unsupported_active_layer_mask = kEnvironmentLayerSensorUninflated;
  EXPECT_FALSE(environmentEvidenceConsistent(overlap));

  EnvironmentEvidenceResult unknown_bit = KnownFreeResult();
  unknown_bit.included_layer_mask = 1U << 31;
  EXPECT_FALSE(environmentEvidenceConsistent(unknown_bit));

  EnvironmentEvidenceResult unsupported =
      InvalidStatus(DistanceStatus::UNAVAILABLE);
  unsupported.unsupported_active_layer_mask =
      kEnvironmentLayerManualUninflated;
  EXPECT_TRUE(environmentEvidenceConsistent(unsupported));
  EXPECT_FALSE(environmentEvidenceHasUsableDistance(unsupported));
}

TEST(EnvironmentEvidenceTest, EverySupportedLayerCanBeRepresented) {
  EnvironmentEvidenceResult result = KnownFreeResult(2.0);
  result.included_layer_mask = kEnvironmentLayerAll;
  EXPECT_TRUE(environmentEvidenceConsistent(result));
  EXPECT_TRUE(environmentEvidenceHasUsableDistance(result));
}

TEST(EnvironmentEvidenceTest, ConsumerAppliesRobustRadiusOnceToUninflatedDistance) {
  const EnvironmentEvidenceResult result = KnownFreeResult(1.20);
  ASSERT_TRUE(environmentEvidenceHasUsableDistance(result));
  const double effective_radius = 0.55;
  const double reference_clearance =
      result.uninflated_distance - effective_radius;
  EXPECT_DOUBLE_EQ(reference_clearance, 0.65);
}

TEST(EnvironmentEvidenceTest, QueryAcceptsDeterministicSyntheticResults) {
  const std::vector<EnvironmentEvidenceResult> synthetic = {
      InvalidStatus(DistanceStatus::UNAVAILABLE),
      InvalidStatus(DistanceStatus::OUT_OF_MAP),
      InvalidStatus(DistanceStatus::UNKNOWN),
      OccupiedResult(),
      KnownFreeResult(),
  };
  for (const EnvironmentEvidenceResult& expected : synthetic) {
    const EnvironmentEvidenceQuery query = [expected](const Eigen::Vector3d&) {
      return expected;
    };
    const EnvironmentEvidenceResult observed = query(Eigen::Vector3d::Zero());
    EXPECT_EQ(observed.status, expected.status);
    EXPECT_EQ(observed.distance_valid, expected.distance_valid);
    EXPECT_DOUBLE_EQ(observed.uninflated_distance,
                     expected.uninflated_distance);
  }
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
