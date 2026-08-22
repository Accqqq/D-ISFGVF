#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "bspline_race/integration/phase_offset_shadow_adapter.h"

namespace FLAG_Race {
namespace {

void ExpectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected,
                      const double tolerance) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

void ExpectPointNear(const geometry_msgs::Point& actual,
                     const Eigen::Vector3d& expected,
                     const double tolerance) {
  EXPECT_NEAR(actual.x, expected.x(), tolerance);
  EXPECT_NEAR(actual.y, expected.y(), tolerance);
  EXPECT_NEAR(actual.z, expected.z(), tolerance);
}

ContinuousPhasePathState MakeLineState(const double w) {
  ContinuousPhasePathState state;
  state.p = Eigen::Vector3d(w, -1.0, 1.0);
  state.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  state.d2p_dw2 = Eigen::Vector3d::Zero();
  state.vel = Eigen::Vector3d(1.0, 0.0, 0.0);
  state.valid = true;
  return state;
}

ContinuousPhasePathState MakeSlopedLineState(const double w, const double slope) {
  ContinuousPhasePathState state;
  state.p = Eigen::Vector3d(w, -1.0, 1.0 + slope * w);
  state.dp_dw = Eigen::Vector3d(1.0, 0.0, slope);
  state.d2p_dw2 = Eigen::Vector3d::Zero();
  state.vel = state.dp_dw;
  state.valid = true;
  return state;
}

ContinuousPhasePathState MakeHeightVaryingParabolaState(const double w,
                                                         const double coefficient) {
  ContinuousPhasePathState state;
  state.p = Eigen::Vector3d(w, -1.0, 1.0 + coefficient * w * w);
  state.dp_dw = Eigen::Vector3d(1.0, 0.0, 2.0 * coefficient * w);
  state.d2p_dw2 = Eigen::Vector3d(0.0, 0.0, 2.0 * coefficient);
  state.vel = state.dp_dw;
  state.valid = true;
  return state;
}

ShadowUpdateInput MakeLineInput() {
  ShadowUpdateInput input;
  input.current_path = ConvertContinuousPhasePathState(MakeLineState(1.0), 1.0);
  input.sampled_path.push_back(ConvertContinuousPhasePathState(MakeLineState(0.0), 0.0));
  input.sampled_path.push_back(ConvertContinuousPhasePathState(MakeLineState(1.0), 1.0));
  input.sampled_path.push_back(ConvertContinuousPhasePathState(MakeLineState(2.0), 2.0));
  input.position = Eigen::Vector3d(1.2, -0.8, 1.0);
  input.stamp = ros::Time(3, 0);
  return input;
}

ShadowUpdateInput MakeHeightVaryingInput() {
  ShadowUpdateInput input;
  input.current_path = ConvertContinuousPhasePathState(
      MakeHeightVaryingParabolaState(1.0, 0.7), 1.0);
  for (const double w : {0.0, 0.5, 1.0, 1.5, 2.0}) {
    input.sampled_path.push_back(ConvertContinuousPhasePathState(
        MakeHeightVaryingParabolaState(w, 0.7), w));
  }
  input.position = Eigen::Vector3d(1.2, -0.8, 2.0);
  input.stamp = ros::Time(3, 0);
  return input;
}

ShadowConfig MakeShadowConfig() {
  ShadowConfig config;
  config.mode = PhaseOffsetShadowMode::SHADOW;
  config.delta = 0.4;
  config.sample_step_w = 0.1;
  config.publish_rate = 5.0;
  config.frame_id = "world";
  return config;
}

TEST(PhaseOffsetShadowAdapterTest, ConvertsContinuousStateFieldByField) {
  ContinuousPhasePathState source;
  source.p = Eigen::Vector3d(1.0, 2.0, 3.0);
  source.dp_dw = Eigen::Vector3d(4.0, 5.0, 6.0);
  source.d2p_dw2 = Eigen::Vector3d(7.0, 8.0, 9.0);
  source.vel = Eigen::Vector3d(10.0, 11.0, 12.0);
  source.valid = true;

  const phase_offset_core::PathDifferentialState converted =
      ConvertContinuousPhasePathState(source, 12.5);
  ExpectVectorNear(converted.p, source.p, 1e-12);
  ExpectVectorNear(converted.p_w, source.dp_dw, 1e-12);
  ExpectVectorNear(converted.p_ww, source.d2p_dw2, 1e-12);
  EXPECT_NEAR(converted.w, 12.5, 1e-12);
  EXPECT_EQ(converted.valid, source.valid);
}

TEST(PhaseOffsetShadowAdapterTest, PlanarStraightSamplesBuildExpectedCandidatePath) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  const ShadowUpdateInput input = MakeLineInput();
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  ASSERT_TRUE(adapter.buildMarkers(input, markers, diagnostics));
  ASSERT_TRUE(diagnostics.valid);
  EXPECT_TRUE(diagnostics.candidate_path_complete);
  EXPECT_EQ(diagnostics.sample_count, 3U);
  EXPECT_EQ(diagnostics.invalid_sample_count, 0U);
  EXPECT_NEAR(diagnostics.path_z_min, 1.0, 1e-12);
  EXPECT_NEAR(diagnostics.path_z_max, 1.0, 1e-12);
  EXPECT_NEAR(diagnostics.path_z_span, 0.0, 1e-12);
  EXPECT_NEAR(diagnostics.max_abs_p_w_z, 0.0, 1e-12);
  EXPECT_NEAR(diagnostics.max_abs_p_ww_z, 0.0, 1e-12);
  EXPECT_NEAR(diagnostics.min_horizontal_path_speed, 1.0, 1e-12);
  EXPECT_NEAR(diagnostics.min_horizontal_to_total_speed_ratio, 1.0, 1e-12);
  EXPECT_NEAR(diagnostics.max_abs_candidate_z_offset, 0.0, 1e-12);
  ASSERT_EQ(markers.base_path.points.size(), 3U);
  ASSERT_EQ(markers.candidate_path.points.size(), 3U);
  for (size_t index = 0; index < markers.base_path.points.size(); ++index) {
    ExpectPointNear(markers.base_path.points[index],
                    Eigen::Vector3d(static_cast<double>(index), -1.0, 1.0),
                    1e-12);
    ExpectPointNear(markers.candidate_path.points[index],
                    Eigen::Vector3d(static_cast<double>(index), -0.6, 1.0),
                    1e-12);
  }
}

TEST(PhaseOffsetShadowAdapterTest, TracksRawVerticalDerivativeExtremaAndPhase) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  ShadowUpdateInput input = MakeLineInput();
  input.sampled_path[0].p_w.z() = -0.02;
  input.sampled_path[1].p_w.z() = 0.07;
  input.sampled_path[2].p_w.z() = -0.04;
  input.sampled_path[0].p_ww.z() = 0.03;
  input.sampled_path[1].p_ww.z() = -0.08;
  input.sampled_path[2].p_ww.z() = 0.05;
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  ASSERT_TRUE(adapter.buildMarkers(input, markers, diagnostics));
  EXPECT_TRUE(diagnostics.candidate_path_complete);
  EXPECT_EQ(diagnostics.invalid_sample_count, 0U);
  EXPECT_NEAR(diagnostics.max_abs_p_w_z, 0.07, 1e-12);
  EXPECT_NEAR(diagnostics.max_abs_p_w_z_at_w, 1.0, 1e-12);
  EXPECT_NEAR(diagnostics.max_abs_p_ww_z, 0.08, 1e-12);
  EXPECT_NEAR(diagnostics.max_abs_p_ww_z_at_w, 1.0, 1e-12);
}

TEST(PhaseOffsetShadowAdapterTest, TracksRawPathZSpan) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  ShadowUpdateInput input = MakeLineInput();
  input.sampled_path[0].p.z() = 0.8;
  input.sampled_path[1].p.z() = 1.3;
  input.sampled_path[2].p.z() = 1.0;
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  ASSERT_TRUE(adapter.buildMarkers(input, markers, diagnostics));
  EXPECT_NEAR(diagnostics.path_z_min, 0.8, 1e-12);
  EXPECT_NEAR(diagnostics.path_z_max, 1.3, 1e-12);
  EXPECT_NEAR(diagnostics.path_z_span, 0.5, 1e-12);
}

TEST(PhaseOffsetShadowAdapterTest, TracksMinimumHorizontalSpeedAndRatio) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  ShadowUpdateInput input = MakeLineInput();
  input.sampled_path[0].p_w = Eigen::Vector3d(3.0, 4.0, 0.0);
  input.sampled_path[1].p_w = Eigen::Vector3d(2.0, 0.0, 2.0);
  input.sampled_path[2].p_w = Eigen::Vector3d(1.0, 0.0, 4.0);
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  ASSERT_TRUE(adapter.buildMarkers(input, markers, diagnostics));
  EXPECT_TRUE(diagnostics.candidate_path_complete);
  EXPECT_NEAR(diagnostics.min_horizontal_path_speed, 1.0, 1e-12);
  EXPECT_NEAR(diagnostics.min_horizontal_path_speed_at_w, 2.0, 1e-12);
  EXPECT_NEAR(diagnostics.min_horizontal_to_total_speed_ratio,
              1.0 / std::sqrt(17.0), 1e-12);
  EXPECT_NEAR(diagnostics.min_horizontal_to_total_speed_ratio_at_w, 2.0, 1e-12);
}

TEST(PhaseOffsetShadowAdapterTest, HeightVaryingSamplesKeepCandidateCompleteAndHeight) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  const ShadowUpdateInput input = MakeHeightVaryingInput();
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  ASSERT_TRUE(adapter.buildMarkers(input, markers, diagnostics));
  EXPECT_TRUE(diagnostics.valid);
  EXPECT_TRUE(diagnostics.candidate_path_complete);
  EXPECT_EQ(diagnostics.sample_count, input.sampled_path.size());
  EXPECT_EQ(diagnostics.invalid_sample_count, 0U);
  EXPECT_GT(diagnostics.path_z_span, 0.0);
  EXPECT_GT(diagnostics.max_abs_p_w_z, 0.0);
  EXPECT_GT(diagnostics.max_abs_p_ww_z, 0.0);
  EXPECT_NEAR(diagnostics.max_abs_candidate_z_offset, 0.0, 1e-12);
  ASSERT_EQ(markers.base_path.points.size(), markers.candidate_path.points.size());
  for (size_t index = 0; index < markers.base_path.points.size(); ++index) {
    EXPECT_NEAR(markers.candidate_path.points[index].z,
                markers.base_path.points[index].z, 1e-12);
  }
}

TEST(PhaseOffsetShadowAdapterTest, HeightVaryingCurrentFrameUsesThreeDimensionalTangent) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  ShadowUpdateInput input = MakeLineInput();
  input.current_path = ConvertContinuousPhasePathState(MakeSlopedLineState(1.0, 0.75), 1.0);
  input.sampled_path.clear();
  for (const double w : {0.0, 1.0, 2.0}) {
    input.sampled_path.push_back(ConvertContinuousPhasePathState(
        MakeSlopedLineState(w, 0.75), w));
  }
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  ASSERT_TRUE(adapter.buildMarkers(input, markers, diagnostics));
  ASSERT_EQ(markers.frame.markers.size(), 4U);
  const visualization_msgs::Marker& tangent = markers.frame.markers[2];
  const visualization_msgs::Marker& normal = markers.frame.markers[3];
  ASSERT_EQ(tangent.points.size(), 2U);
  ASSERT_EQ(normal.points.size(), 2U);
  const Eigen::Vector3d tangent_vector(tangent.points[1].x - tangent.points[0].x,
                                       tangent.points[1].y - tangent.points[0].y,
                                       tangent.points[1].z - tangent.points[0].z);
  const Eigen::Vector3d normal_vector(normal.points[1].x - normal.points[0].x,
                                      normal.points[1].y - normal.points[0].y,
                                      normal.points[1].z - normal.points[0].z);
  const Eigen::Vector3d expected_tangent =
      Eigen::Vector3d(1.0, 0.0, 0.75).normalized();
  ExpectVectorNear(tangent_vector, 0.8 * expected_tangent, 1e-12);
  EXPECT_NEAR(tangent_vector.norm(), 0.8, 1e-12);
  EXPECT_GT(std::abs(tangent_vector.z()), 1e-3);
  EXPECT_NEAR(normal_vector.z(), 0.0, 1e-12);
  EXPECT_NEAR(tangent_vector.dot(normal_vector), 0.0, 1e-12);
}

TEST(PhaseOffsetShadowAdapterTest, InvalidVerticalSampleDeletesIncompleteCandidatePath) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  ShadowUpdateInput input = MakeLineInput();
  input.sampled_path[1].p_w = Eigen::Vector3d(0.0, 0.0, 1.0);
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  ASSERT_TRUE(adapter.buildMarkers(input, markers, diagnostics));
  EXPECT_TRUE(diagnostics.valid);
  EXPECT_EQ(diagnostics.sample_count, 3U);
  EXPECT_EQ(diagnostics.invalid_sample_count, 1U);
  EXPECT_FALSE(diagnostics.candidate_path_complete);
  EXPECT_FALSE(diagnostics.invalid_reason.empty());
  EXPECT_EQ(markers.base_path.points.size(), 3U);
  EXPECT_EQ(markers.candidate_path.action, visualization_msgs::Marker::DELETE);
  EXPECT_TRUE(markers.candidate_path.points.empty());
}

TEST(PhaseOffsetShadowAdapterTest, NearVerticalCurrentPathDeletesCandidateAndFrame) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  ShadowUpdateInput input = MakeLineInput();
  input.current_path.p_w = Eigen::Vector3d(1e-10, 0.0, 1.0);
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  EXPECT_FALSE(adapter.buildMarkers(input, markers, diagnostics));
  EXPECT_FALSE(diagnostics.valid);
  EXPECT_FALSE(diagnostics.candidate_path_complete);
  EXPECT_FALSE(diagnostics.invalid_reason.empty());
  EXPECT_EQ(markers.candidate_path.action, visualization_msgs::Marker::DELETE);
  ASSERT_EQ(markers.frame.markers.size(), 4U);
  for (const visualization_msgs::Marker& marker : markers.frame.markers) {
    EXPECT_EQ(marker.action, visualization_msgs::Marker::DELETE);
  }
}

TEST(PhaseOffsetShadowAdapterTest, CurrentFrameMarkersAreFiniteAndOrthogonal) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  ASSERT_TRUE(adapter.buildMarkers(MakeLineInput(), markers, diagnostics));
  ASSERT_EQ(markers.frame.markers.size(), 4U);
  for (const visualization_msgs::Marker& marker : markers.frame.markers) {
    EXPECT_TRUE(std::isfinite(marker.pose.position.x));
    EXPECT_TRUE(std::isfinite(marker.pose.position.y));
    EXPECT_TRUE(std::isfinite(marker.pose.position.z));
    for (const geometry_msgs::Point& point : marker.points) {
      EXPECT_TRUE(std::isfinite(point.x));
      EXPECT_TRUE(std::isfinite(point.y));
      EXPECT_TRUE(std::isfinite(point.z));
    }
  }
}

TEST(PhaseOffsetShadowAdapterTest, InvalidCurrentDeletesAllFrameMarkers) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  ShadowUpdateInput input = MakeLineInput();
  input.current_path.valid = false;
  input.current_path.p.x() = std::numeric_limits<double>::quiet_NaN();
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  EXPECT_FALSE(adapter.buildMarkers(input, markers, diagnostics));
  EXPECT_FALSE(diagnostics.valid);
  EXPECT_FALSE(diagnostics.candidate_path_complete);
  EXPECT_FALSE(diagnostics.invalid_reason.empty());
  EXPECT_EQ(diagnostics.sample_count, 3U);
  EXPECT_EQ(diagnostics.invalid_sample_count, 0U);
  EXPECT_EQ(markers.base_path.points.size(), 3U);
  EXPECT_EQ(markers.candidate_path.action, visualization_msgs::Marker::DELETE);
  EXPECT_TRUE(markers.candidate_path.points.empty());
  ASSERT_EQ(markers.frame.markers.size(), 4U);
  for (size_t id = 0; id < markers.frame.markers.size(); ++id) {
    const visualization_msgs::Marker& marker = markers.frame.markers[id];
    EXPECT_EQ(marker.action, visualization_msgs::Marker::DELETE);
    EXPECT_EQ(marker.id, static_cast<int>(id));
    EXPECT_TRUE(std::isfinite(marker.pose.position.x));
    EXPECT_TRUE(std::isfinite(marker.pose.position.y));
    EXPECT_TRUE(std::isfinite(marker.pose.position.z));
  }
}

TEST(PhaseOffsetShadowAdapterTest, DisabledModeDoesNotEvaluateOrPublish) {
  ShadowConfig config;
  PhaseOffsetShadowAdapter adapter(config);
  ShadowUpdateInput input = MakeLineInput();
  ShadowDiagnostics diagnostics;
  diagnostics.invalid_reason = "unchanged";

  EXPECT_FALSE(adapter.enabled());
  EXPECT_FALSE(adapter.isPublishDue(input.stamp));
  EXPECT_FALSE(adapter.update(input, &diagnostics));
  EXPECT_EQ(diagnostics.invalid_reason, "unchanged");
}

TEST(PhaseOffsetShadowAdapterTest, MarkerConstructionDoesNotModifyInput) {
  const PhaseOffsetShadowAdapter adapter(MakeShadowConfig());
  const ShadowUpdateInput input = MakeHeightVaryingInput();
  const ShadowUpdateInput original = input;
  ShadowMarkerBundle markers;
  ShadowDiagnostics diagnostics;

  ASSERT_TRUE(adapter.buildMarkers(input, markers, diagnostics));
  ExpectVectorNear(input.current_path.p, original.current_path.p, 1e-12);
  ExpectVectorNear(input.current_path.p_w, original.current_path.p_w, 1e-12);
  ExpectVectorNear(input.current_path.p_ww, original.current_path.p_ww, 1e-12);
  EXPECT_NEAR(input.current_path.w, original.current_path.w, 1e-12);
  ASSERT_EQ(input.sampled_path.size(), original.sampled_path.size());
  for (size_t index = 0; index < input.sampled_path.size(); ++index) {
    ExpectVectorNear(input.sampled_path[index].p, original.sampled_path[index].p, 1e-12);
    ExpectVectorNear(input.sampled_path[index].p_w,
                     original.sampled_path[index].p_w, 1e-12);
    ExpectVectorNear(input.sampled_path[index].p_ww,
                     original.sampled_path[index].p_ww, 1e-12);
    EXPECT_NEAR(input.sampled_path[index].w, original.sampled_path[index].w, 1e-12);
  }
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
