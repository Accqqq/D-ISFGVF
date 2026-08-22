#include <gtest/gtest.h>

#include "phase_offset_navigation/tube_builder.h"

#include <cmath>
#include <limits>
#include <vector>

namespace phase_offset_navigation {
namespace {

using PathSamples = std::vector<phase_offset_core::PathDifferentialState,
                                Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>;

phase_offset_core::PathDifferentialState State(const double w) {
  phase_offset_core::PathDifferentialState state;
  state.p = Eigen::Vector3d(w, 0.0, 1.0);
  state.p_w = Eigen::Vector3d::UnitX();
  state.p_ww = Eigen::Vector3d::Zero();
  state.w = w;
  state.valid = true;
  return state;
}

PathSamples Line(const double end = 0.40, const double step = 0.10) {
  PathSamples path;
  for (double w = 0.0; w <= end + 1e-12; w += step) path.push_back(State(w));
  return path;
}

PathStateQuery ExactLine() {
  return [](const double w, phase_offset_core::PathDifferentialState& state) {
    state = State(w);
    return true;
  };
}

TubeBuilderConfig Config() {
  TubeBuilderConfig config;
  config.fixed_delta_max = 0.10;
  config.max_offset = 0.80;
  config.sample_step_w = 0.10;
  config.lookahead_w = 0.40;
  config.back_w = 0.0;
  config.min_certified_forward_w = 0.20;
  config.ray_step = 0.05;
  config.erosion.uav_radius = 0.10;
  config.erosion.localization_margin = 0.02;
  config.erosion.tracking_error_bound = 0.03;
  config.erosion.map_margin = 0.02;
  config.erosion.extra_margin = 0.01;
  config.erosion.discretization_margin = 0.02;
  config.cross_section.search_extent = 1.0;
  config.cross_section.ray_step = 0.05;
  config.cross_section.boundary_tolerance = 1e-4;
  config.cross_section.regularity_margin = 0.10;
  config.cross_section.curvature_epsilon = 1e-8;
  config.cross_section.planner_safe_distance = 0.40;
  config.cross_section.margins.uav_radius = 0.25;
  config.cross_section.margins.map_uncertainty = 0.10;
  config.cross_section.margins.localization_uncertainty = 0.05;
  config.cross_section.margins.tracking_error_bound = 0.15;
  config.cross_section.margins.preincluded_map_uncertainty = 0.10;
  return config;
}

DistanceQuery LegacyOpen() {
  return [](const Eigen::Vector3d&) {
    DistanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.signed_distance = 10.0;
    return result;
  };
}

ClearanceQuery Open() {
  return [](const Eigen::Vector3d&, const double required) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

ClearanceQuery Obstacle(const Eigen::Vector3d& obstacle) {
  return [obstacle](const Eigen::Vector3d& point, const double) {
    ClearanceQueryResult result;
    const double distance = (point - obstacle).norm();
    if (distance <= 1e-12) {
      result.status = DistanceStatus::OCCUPIED;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = distance;
    result.clearance_certified = true;
    return result;
  };
}

PathCellBoundQuery CertifiedLineCells() {
  return [](const double w0, const double w1,
            phase_offset_core::PathCellGeometryCertificate& certificate) {
    certificate = phase_offset_core::PathCellGeometryCertificate();
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.segment_w0 = 0.0;
    certificate.segment_w1 = 1.0;
    certificate.segment_identity = 1U;
    certificate.inf_p_w_norm = 1.0;
    certificate.inf_horizontal_p_w_norm = 1.0;
    certificate.sup_p_w_norm = 1.0;
    certificate.sup_p_ww_norm = 0.0;
    certificate.sup_p_www_norm = 0.0;
    certificate.sup_N_w_norm = 0.0;
    certificate.sup_abs_curvature = 0.0;
    certificate.normal_variation_bound = 0.0;
    certificate.curvature_variation_bound = 0.0;
    certificate.midpoint_position_variation_bound = 0.0;
    certificate.chord_deviation_bound = 0.0;
    certificate.valid = std::isfinite(w0) && std::isfinite(w1) && w1 > w0;
    certificate.complete = certificate.valid;
    return certificate.valid;
  };
}

TEST(TubeBuilderTest, FixedAndLegacyDistanceRegressionRemainSeparate) {
  TubeProfile fixed;
  ASSERT_TRUE(TubeBuilder(Config()).build(TubeSource::FIXED, Line(),
                                          DistanceQuery(), 1U, 1U, fixed));
  ASSERT_TRUE(fixed.raw_complete);
  EXPECT_FALSE(fixed.obstacle_certified);
  EXPECT_NEAR(fixed.samples.front().raw_lower, -0.10, 1e-12);
  EXPECT_NEAR(fixed.samples.front().raw_upper, 0.10, 1e-12);

  TubeProfile legacy;
  ASSERT_TRUE(TubeBuilder(Config()).build(TubeSource::ESDF, Line(), LegacyOpen(),
                                          1U, 2U, legacy));
  EXPECT_TRUE(legacy.raw_complete);
  EXPECT_TRUE(legacy.obstacle_certified);
}

TEST(TubeBuilderTest, CloudClearanceUsesPlannerSafeDistanceWithoutDoubleErosion) {
  TubeProfile profile;
  ASSERT_TRUE(TubeBuilder(Config()).buildCloudClearance(
      TubeSource::ESDF, Line(), Open(), ExactLine(), 0.05, 0.0, 3U, 4U,
      profile));
  ASSERT_TRUE(profile.raw_complete);
  ASSERT_FALSE(profile.samples.empty());
  ASSERT_EQ(profile.raw_build_samples.size(), profile.samples.size());
  EXPECT_NEAR(profile.snapshot_resolution, 0.05, 1e-12);
  const TubeRawSample& sample = profile.samples.front();
  EXPECT_NEAR(sample.full_effective_radius, 0.55, 1e-12);
  EXPECT_NEAR(sample.residual_effective_radius, 0.40, 1e-12);
  EXPECT_NEAR(sample.raw_lower, -0.95, 1e-12);
  EXPECT_NEAR(sample.raw_upper, 0.95, 1e-12);
  EXPECT_NEAR(sample.pre_inset_lower, -1.0, 1e-12);
  EXPECT_NEAR(sample.pre_inset_upper, 1.0, 1e-12);
  EXPECT_NEAR(sample.continuous_inset, 0.05, 1e-12);
  EXPECT_TRUE(sample.pre_inset_contains_zero);
  EXPECT_TRUE(sample.post_inset_contains_zero);
  EXPECT_TRUE(sample.filter_input_contains_zero);
  EXPECT_TRUE(sample.environment_contains_zero);
}

TEST(TubeBuilderTest, CertifiedStraightCellUsesZeroLocalInsetWithoutLosingZero) {
  TubeProfile profile;
  ASSERT_TRUE(TubeBuilder(Config()).buildCloudClearance(
      TubeSource::ESDF, Line(), Open(), ExactLine(), CertifiedLineCells(), 0.05,
      0.0, 3U, 4U, profile));
  ASSERT_TRUE(profile.raw_complete);
  EXPECT_TRUE(profile.cell_geometry_certified);
  EXPECT_EQ(profile.certified_cell_count, profile.raw_build_samples.size() - 1U);
  for (const TubeRawSample& sample : profile.raw_build_samples) {
    EXPECT_TRUE(sample.cell_geometry_certificate_used);
    EXPECT_DOUBLE_EQ(sample.continuous_inset, 0.0);
    EXPECT_DOUBLE_EQ(sample.cell_geometry_inset, 0.0);
    EXPECT_TRUE(sample.post_inset_contains_zero);
    EXPECT_TRUE(sample.filtered_contains_zero);
  }
}

TEST(TubeBuilderTest,
     CertifiedCellWithOffsetInvariantFailureFallsBackToFixedInset) {
  const PathCellBoundQuery invalid_offset_certificate =
      [](const double w0, const double w1,
         phase_offset_core::PathCellGeometryCertificate& certificate) {
        certificate = phase_offset_core::PathCellGeometryCertificate();
        certificate.w0 = w0;
        certificate.w1 = w1;
        certificate.segment_w0 = 0.0;
        certificate.segment_w1 = 1.0;
        certificate.segment_identity = 9U;
        certificate.inf_p_w_norm = 0.1;
        certificate.inf_horizontal_p_w_norm = 0.1;
        certificate.sup_p_w_norm = 1.0;
        certificate.sup_p_ww_norm = 0.0;
        certificate.sup_p_www_norm = 0.0;
        certificate.sup_N_w_norm = 2.0;
        certificate.sup_abs_curvature = 10.0;
        certificate.normal_variation_bound = 0.2 * (w1 - w0);
        certificate.curvature_variation_bound = 0.0;
        certificate.midpoint_position_variation_bound = 0.5 * (w1 - w0);
        certificate.chord_deviation_bound = 0.0;
        certificate.valid = true;
        certificate.complete = true;
        return true;
      };
  TubeProfile profile;
  ASSERT_TRUE(TubeBuilder(Config()).buildCloudClearance(
      TubeSource::ESDF, Line(), Open(), ExactLine(),
      invalid_offset_certificate, 0.05, 0.0, 3U, 4U, profile));
  EXPECT_FALSE(profile.cell_geometry_certified);
  EXPECT_EQ(profile.certified_cell_count, 0U);
  ASSERT_FALSE(profile.raw_build_samples.empty());
  EXPECT_DOUBLE_EQ(profile.raw_build_samples.front().continuous_inset, 0.05);
  EXPECT_FALSE(profile.raw_build_samples.front().cell_geometry_certificate_used);
}

TEST(TubeBuilderTest, PreInsetNarrowingCollapsesToZeroWithoutExcludingIt) {
  const ClearanceQuery narrow_positive_side = [](const Eigen::Vector3d& point,
                                                  const double required) {
    ClearanceQueryResult result;
    const double clearance = std::min(0.44 - point.y(), point.y() + 2.0);
    if (clearance <= 0.0) {
      result.status = DistanceStatus::OCCUPIED;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = clearance;
    result.clearance_certified = clearance >= required;
    return result;
  };
  TubeProfile profile;
  ASSERT_TRUE(TubeBuilder(Config()).buildCloudClearance(
      TubeSource::ESDF, Line(), narrow_positive_side, ExactLine(), 0.05,
      0.0, 31U, 41U, profile));
  ASSERT_FALSE(profile.raw_build_samples.empty());
  const TubeRawSample& sample = profile.raw_build_samples.front();
  EXPECT_TRUE(sample.pre_inset_contains_zero);
  EXPECT_TRUE(sample.post_inset_contains_zero);
  EXPECT_TRUE(sample.filter_input_contains_zero);
  EXPECT_LT(sample.pre_inset_upper, sample.continuous_inset);
  EXPECT_DOUBLE_EQ(sample.raw_upper, 0.0);
  EXPECT_TRUE(sample.filtered_contains_zero);
}

TEST(TubeBuilderTest, DiagonalAndBetweenKnotObstacleCauseAdaptiveCrossSections) {
  const PathSamples sparse = Line(0.20, 0.20);
  TubeProfile profile;
  ASSERT_TRUE(TubeBuilder(Config()).buildCloudClearance(
      TubeSource::ESDF, sparse,
      Obstacle(Eigen::Vector3d(0.10, 0.15, 1.0)), ExactLine(), 0.05, 0.0,
      1U, 1U, profile));
  EXPECT_GT(profile.diagnostics.sample_count, sparse.size());
  bool midpoint_found = false;
  for (const TubeRawSample& sample : profile.samples) {
    if (std::abs(sample.w - 0.10) < 1e-9) {
      midpoint_found = true;
      EXPECT_TRUE(sample.environment_contains_zero);
    }
  }
  EXPECT_TRUE(midpoint_found);
}

TEST(TubeBuilderTest, UnsafeCentreProducesPlannerZeroOnlyGeometry) {
  TubeProfile profile;
  ASSERT_TRUE(TubeBuilder(Config()).buildCloudClearance(
      TubeSource::ESDF, Line(), Obstacle(Eigen::Vector3d(0.0, 0.0, 1.0)),
      ExactLine(), 0.05, 0.0, 1U, 1U, profile));
  ASSERT_FALSE(profile.samples.empty());
  EXPECT_TRUE(profile.samples.front().complete);
  EXPECT_TRUE(profile.samples.front().environment_contains_zero);
  EXPECT_DOUBLE_EQ(profile.samples.front().raw_lower, 0.0);
  EXPECT_DOUBLE_EQ(profile.samples.front().raw_upper, 0.0);
}

TEST(TubeBuilderTest, UnknownAtCurrentOrFutureFallsBackToPlannerZeroOnly) {
  const ClearanceQuery unknown_after = [](const Eigen::Vector3d& point,
                                          const double) {
    ClearanceQueryResult result;
    if (point.x() >= 0.25) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 10.0;
    result.clearance_certified = true;
    return result;
  };
  TubeProfile future;
  ASSERT_TRUE(TubeBuilder(Config()).buildCloudClearance(
      TubeSource::ESDF, Line(0.40), unknown_after, ExactLine(), 0.05,
      0.0, 1U, 1U, future));
  EXPECT_FALSE(future.certified_segment_truncated_after);
  EXPECT_DOUBLE_EQ(future.preview_end_w, 0.40);

  TubeProfile current;
  EXPECT_TRUE(TubeBuilder(Config()).buildCloudClearance(
      TubeSource::ESDF, Line(0.40), unknown_after, ExactLine(), 0.05,
      0.30, 1U, 2U, current));
  ASSERT_TRUE(current.raw_complete);
  for (const TubeRawSample& sample : current.samples) {
    EXPECT_LE(sample.raw_lower, 0.0);
    EXPECT_GE(sample.raw_upper, 0.0);
  }
}

TEST(TubeBuilderTest, MissingExactPathOrClearanceQueryFailsClosed) {
  TubeProfile profile;
  EXPECT_FALSE(TubeBuilder(Config()).buildCloudClearance(
      TubeSource::ESDF, Line(), ClearanceQuery(), ExactLine(), 0.05,
      0.0, 1U, 1U, profile));
  EXPECT_FALSE(TubeBuilder(Config()).buildCloudClearance(
      TubeSource::ESDF, Line(), Open(), PathStateQuery(), 0.05,
      0.0, 1U, 1U, profile));
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
