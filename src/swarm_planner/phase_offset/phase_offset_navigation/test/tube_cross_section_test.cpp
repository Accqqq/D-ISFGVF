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

TEST(TubeCrossSectionTest, HorizontalNormalRaysPreserveCenterlineAltitude) {
  std::size_t samples = 0U;
  TubeCrossSectionInput input = MakeInput(
      [&samples](const Eigen::Vector3d& point, const double required) {
        ++samples;
        EXPECT_DOUBLE_EQ(point.z(), 2.5);
        ClearanceQueryResult result;
        result.status = DistanceStatus::KNOWN_FREE;
        result.clearance = std::max(10.0, required);
        result.clearance_certified = true;
        return result;
      });
  input.p = Eigen::Vector3d(0.0, 0.0, 2.5);
  input.N = Eigen::Vector3d(0.0, 1.0, 0.0);
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(input);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(samples, 0U);
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

TEST(TubeCrossSectionTest, SelectedCurrentComponentIsNotConvexified) {
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
  TubeCrossSectionInput input = MakeInput(query);
  input.current_delta = 0.60;
  input.current_delta_valid = true;
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(input);
  ASSERT_TRUE(result.valid);
  EXPECT_GE(result.lower_final, 0.49);
  EXPECT_LE(result.upper_final, 1.0);
  EXPECT_GE(input.current_delta, result.lower_final);
  EXPECT_LE(input.current_delta, result.upper_final);
}

TEST(TubeCrossSectionTest, UnsafeCurrentComponentFailsClosed) {
  const ClearanceQuery query = [](const Eigen::Vector3d& point,
                                  const double) {
    ClearanceQueryResult result;
    if (point.y() > 0.20 && point.y() < 0.50) {
      result.status = DistanceStatus::OCCUPIED;
      result.clearance = 0.0;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 10.0;
    result.clearance_certified = true;
    return result;
  };
  TubeCrossSectionInput input = MakeInput(query);
  input.current_delta = 0.30;
  input.current_delta_valid = true;
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, TubeCrossSectionReason::CENTER_OCCUPIED);
}

TubeCrossSectionInput QuadraticInput(const double selected_delta,
                                     const double tangent_x = 0.20,
                                     const double normal_w_x = 1.0,
                                     const double minimum = 0.50) {
  TubeCrossSectionInput input = MakeInput(OpenSpace());
  input.p_w = Eigen::Vector3d(tangent_x, 0.0, 0.0);
  input.N_w = Eigen::Vector3d(normal_w_x, 0.0, 0.0);
  input.minimum_reference_speed = minimum;
  input.current_delta = selected_delta;
  input.current_delta_valid = true;
  return input;
}

TEST(TubeCrossSectionTest, QuadraticNegativeCSelectsLeftSafeComponent) {
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(QuadraticInput(-0.90));
  ASSERT_TRUE(result.valid);
  // (0.2 + delta)^2 >= 0.5^2 has roots -0.7 and 0.3; retain the left
  // connected component containing the selected current delta.
  EXPECT_NEAR(result.upper_curvature, -0.70, 1e-12);
  EXPECT_LE(result.upper_final, -0.70 + 1e-6);
}

TEST(TubeCrossSectionTest, QuadraticNegativeCSelectsRightSafeComponent) {
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(QuadraticInput(0.60));
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.lower_curvature, 0.30, 1e-12);
  EXPECT_GE(result.lower_final, 0.30 - 1e-6);
  EXPECT_FALSE(result.contains_zero);
}

TEST(TubeCrossSectionTest, QuadraticNegativeCRejectsUnsafeRootComponent) {
  const TubeCrossSectionResult result =
      TubeCrossSectionSolver(MakeConfig()).solve(QuadraticInput(0.0));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, TubeCrossSectionReason::REGULARITY_ZERO_UNSAFE);
}

TEST(TubeCrossSectionTest, LinearRegularityPositiveBUsesLowerLimit) {
  const TubeCrossSectionResult result = TubeCrossSectionSolver(MakeConfig()).solve(
      QuadraticInput(0.60, 0.20, 1e-8, 0.200000005));
  ASSERT_TRUE(result.valid);
  EXPECT_GE(result.lower_curvature, 0.49);
}

TEST(TubeCrossSectionTest, LinearRegularityNegativeBUsesUpperLimit) {
  const TubeCrossSectionResult result = TubeCrossSectionSolver(MakeConfig()).solve(
      QuadraticInput(-0.60, -0.20, 1e-8, 0.200000005));
  ASSERT_TRUE(result.valid);
  EXPECT_LE(result.upper_curvature, -0.49);
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
