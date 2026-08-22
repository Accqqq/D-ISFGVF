#include <gtest/gtest.h>

#include "phase_offset_navigation/tube_cross_section.h"

#include <cmath>
#include <limits>

namespace phase_offset_navigation {
namespace {

TubeCrossSectionConfig MakeConfig() {
  TubeCrossSectionConfig config;
  config.search_extent = 1.0;
  config.ray_step = 0.05;
  config.boundary_tolerance = 1e-4;
  config.regularity_margin = 0.1;
  config.curvature_epsilon = 1e-8;
  config.planner_safe_distance = 0.30;
  config.margins.uav_radius = 0.20;
  config.margins.map_uncertainty = 0.05;
  config.margins.localization_uncertainty = 0.03;
  config.margins.tracking_error_bound = 0.02;
  return config;
}

ClearanceQuery OpenSpace() {
  return [](const Eigen::Vector3d&, const double required) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

TubeCrossSectionInput MakeInput(const ClearanceQuery& query) {
  TubeCrossSectionInput input;
  input.p = Eigen::Vector3d::Zero();
  input.N = Eigen::Vector3d::UnitY();
  input.clearance_query = query;
  return input;
}

void ExpectFinite(const TubeCrossSectionResult& result) {
  EXPECT_TRUE(std::isfinite(result.c_plus_raw));
  EXPECT_TRUE(std::isfinite(result.c_minus_raw));
  EXPECT_TRUE(std::isfinite(result.full_effective_radius));
  EXPECT_TRUE(std::isfinite(result.preincluded_map_uncertainty));
  EXPECT_TRUE(std::isfinite(result.residual_effective_radius));
  EXPECT_TRUE(std::isfinite(result.lower_final));
  EXPECT_TRUE(std::isfinite(result.upper_final));
}

TEST(TubeCrossSectionTest, DirectClearanceOpenSpaceHasZeroConnectedInterval) {
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(MakeInput(OpenSpace()));
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.lower_final, -1.0, 1e-12);
  EXPECT_NEAR(result.upper_final, 1.0, 1e-12);
  EXPECT_TRUE(result.contains_zero);
  EXPECT_NEAR(result.c_plus_raw, 1.0, 1e-12);
  EXPECT_NEAR(result.c_minus_raw, 1.0, 1e-12);
  ExpectFinite(result);
}

TEST(TubeCrossSectionTest, OneSideBlockedImmediatelyRetainsOtherSide) {
  const ClearanceQuery query = [](const Eigen::Vector3d& point,
                                  const double) {
    ClearanceQueryResult result;
    if (point.y() < 0.0) {
      result.status = DistanceStatus::OCCUPIED;
      result.clearance = 0.0;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 10.0;
    result.clearance_certified = true;
    return result;
  };
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(MakeInput(query));
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.lower_final, 0.0);
  EXPECT_NEAR(result.upper_final, 1.0, 1e-12);
  EXPECT_TRUE(result.contains_zero);
  EXPECT_EQ(result.negative_termination, TubeRayTermination::OCCUPIED);
}

TEST(TubeCrossSectionTest, BothSidesBlockedImmediatelyYieldZeroOnly) {
  const ClearanceQuery query = [](const Eigen::Vector3d& point,
                                  const double) {
    ClearanceQueryResult result;
    if (std::abs(point.y()) > 1e-12) {
      result.status = DistanceStatus::OCCUPIED;
      result.clearance = 0.0;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 10.0;
    result.clearance_certified = true;
    return result;
  };
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(MakeInput(query));
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.contains_zero);
  EXPECT_DOUBLE_EQ(result.lower_final, 0.0);
  EXPECT_DOUBLE_EQ(result.upper_final, 0.0);
  EXPECT_EQ(result.positive_termination, TubeRayTermination::OCCUPIED);
  EXPECT_EQ(result.negative_termination, TubeRayTermination::OCCUPIED);
}

TEST(TubeCrossSectionTest, UnprovenZeroSnapshotYieldsValidZeroOnly) {
  const ClearanceQuery query = [](const Eigen::Vector3d&,
                                  const double) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::UNKNOWN;
    return result;
  };
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(MakeInput(query));
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.contains_zero);
  EXPECT_DOUBLE_EQ(result.lower_final, 0.0);
  EXPECT_DOUBLE_EQ(result.upper_final, 0.0);
  EXPECT_EQ(result.reason, TubeCrossSectionReason::CENTER_UNKNOWN);
}

TEST(TubeCrossSectionTest, DisconnectedNonzeroRegionIsIgnored) {
  const ClearanceQuery query = [](const Eigen::Vector3d& point,
                                  const double) {
    ClearanceQueryResult result;
    const double magnitude = std::abs(point.y());
    if (magnitude > 0.20 && magnitude < 0.50) {
      result.status = DistanceStatus::OCCUPIED;
      result.clearance = 0.0;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 10.0;
    result.clearance_certified = true;
    return result;
  };
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(MakeInput(query));
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.contains_zero);
  EXPECT_NEAR(result.lower_final, -0.20, 0.01);
  EXPECT_NEAR(result.upper_final, 0.20, 0.01);
  EXPECT_EQ(result.positive_termination, TubeRayTermination::OCCUPIED);
  EXPECT_EQ(result.negative_termination, TubeRayTermination::OCCUPIED);
}

TEST(TubeCrossSectionTest, PlannerSafeDistanceIsTheOnlyGeometryClearance) {
  TubeCrossSectionConfig config = MakeConfig();
  config.planner_safe_distance = 0.45;
  config.margins.uav_radius = 0.25;
  config.margins.map_uncertainty = 0.10;
  config.margins.localization_uncertainty = 0.05;
  config.margins.tracking_error_bound = 0.35;
  config.margins.preincluded_map_uncertainty = 0.10;
  double requested_clearance = -1.0;
  const ClearanceQuery corridor = [&requested_clearance](
      const Eigen::Vector3d& point, const double required) {
    requested_clearance = required;
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 0.90 - std::abs(point.y());
    result.clearance_certified = result.clearance > 0.0;
    return result;
  };
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(config).solve(MakeInput(corridor));
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.full_effective_radius, 0.75, 1e-12);
  EXPECT_NEAR(result.residual_effective_radius, 0.45, 1e-12);
  EXPECT_NEAR(result.effective_radius, 0.45, 1e-12);
  EXPECT_NEAR(requested_clearance, 0.45, 1e-12);
  // Direct clearance gives 0.90 - |delta| >= 0.45, with no second erosion.
  EXPECT_NEAR(result.lower_final, -0.45, 0.01);
  EXPECT_NEAR(result.upper_final, 0.45, 0.01);
}

TEST(TubeCrossSectionTest, CurvatureAndInvalidInputsFailClosed) {
  TubeCrossSectionInput curved = MakeInput(OpenSpace());
  curved.curvature = 2.0;
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(curved);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.upper_final, 0.45, 1e-12);

  TubeCrossSectionInput no_query = MakeInput(ClearanceQuery());
  EXPECT_EQ(TubeCrossSectionSolver(MakeConfig()).solve(no_query).reason,
            TubeCrossSectionReason::INVALID_CONFIGURATION);
  curved.curvature = std::numeric_limits<double>::infinity();
  EXPECT_EQ(TubeCrossSectionSolver(MakeConfig()).solve(curved).reason,
            TubeCrossSectionReason::CURVATURE_NUMERICAL_FAILURE);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
