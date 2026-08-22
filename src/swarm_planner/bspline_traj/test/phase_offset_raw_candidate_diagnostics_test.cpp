#include <gtest/gtest.h>

#include "bspline_race/integration/phase_offset_raw_candidate_diagnostics.h"

#include <cmath>
#include <set>

namespace FLAG_Race {
namespace {

using phase_offset_navigation::DistanceStatus;
using phase_offset_navigation::TubeCrossSectionReason;
using phase_offset_navigation::TubeEpochReason;
using phase_offset_navigation::TubeEpochState;
using phase_offset_navigation::TubeProfile;
using phase_offset_navigation::TubeRawSample;
using phase_offset_navigation::TubeRayTermination;
using phase_offset_navigation::TubeStopReason;

constexpr double kTolerance = 1e-12;

TubeRawSample MakeCurrentSample() {
  TubeRawSample sample;
  sample.w = 1.0;
  sample.p = Eigen::Vector3d(1.0, 2.0, 1.0);
  sample.N = Eigen::Vector3d(0.0, 2.0, 0.0);
  sample.complete = true;
  sample.c_plus_raw = 0.75;
  sample.c_minus_raw = 1.25;
  sample.effective_radius = 0.55;
  sample.obstacle_lower = -0.70;
  sample.obstacle_upper = 0.20;
  sample.curvature_lower = -3.0;
  sample.curvature_upper = 2.0;
  sample.environment_lower = -0.70;
  sample.environment_upper = 0.20;
  sample.environment_width = 0.90;
  sample.environment_interval_nonempty = true;
  sample.environment_contains_zero = true;
  sample.cross_section_reason = TubeCrossSectionReason::NONE;
  sample.positive_ray_termination = TubeRayTermination::OCCUPIED;
  sample.negative_ray_termination = TubeRayTermination::SEARCH_EXTENT;
  return sample;
}

TubeProfile MakeProfile(const TubeRawSample& sample = MakeCurrentSample()) {
  TubeProfile profile;
  profile.source = phase_offset_navigation::TubeSource::ESDF;
  profile.raw_complete = sample.complete;
  profile.filtered_complete = sample.complete;
  profile.complete = sample.complete;
  profile.requested_preview_start_w = 0.8;
  profile.requested_preview_end_w = 2.2;
  profile.certified_segment_start_w = 0.8;
  profile.certified_segment_end_w = 2.0;
  profile.preview_start_w = profile.certified_segment_start_w;
  profile.preview_end_w = profile.certified_segment_end_w;
  profile.diagnostics.invalid_count = sample.complete ? 0U : 1U;
  profile.diagnostics.unavailable_count = 2U;
  profile.diagnostics.out_of_map_count = 3U;
  profile.diagnostics.unknown_count = 4U;
  profile.diagnostics.occupied_count = 5U;
  profile.diagnostics.insufficient_clearance_count = 6U;
  profile.samples.push_back(sample);
  return profile;
}

phase_offset_navigation::RawOccupancyQuery ProbeQuery() {
  return [](const Eigen::Vector3d& point) {
    if (std::abs(point.x() - 9.0) <= kTolerance) return DistanceStatus::OCCUPIED;
    if (std::abs(point.y() - 2.05) <= kTolerance) return DistanceStatus::UNKNOWN;
    if (std::abs(point.y() - 1.95) <= kTolerance) return DistanceStatus::OUT_OF_MAP;
    return DistanceStatus::KNOWN_FREE;
  };
}

RawCandidateDiagnosticsInput MakeInput(const TubeProfile* profile) {
  RawCandidateDiagnosticsInput input;
  input.raw_source_configured = true;
  input.raw_storage_ready = true;
  input.raw_query_injected = true;
  input.tube_update_due = true;
  input.epoch_status.state = TubeEpochState::WAITING_FOR_CANDIDATE;
  input.epoch_status.reason = TubeEpochReason::CANDIDATE_INCOMPLETE;
  input.epoch_status.candidate_sequence = 11U;
  input.epoch_status.candidate_raw_complete = false;
  input.epoch_status.candidate_filtered_complete = false;
  input.epoch_status.candidate_complete = false;
  input.candidate_profile = profile;
  input.current_w = 1.0;
  input.current_path.w = 1.0;
  input.current_path.p = Eigen::Vector3d(1.0, 2.0, 1.0);
  input.actual_position = Eigen::Vector3d(9.0, 2.0, 1.0);
  input.raw_occupancy_query = ProbeQuery();
  input.raw_ray_step = 0.05;
  return input;
}

void ExpectFinite(const std::array<double, kRawCandidateDiagnosticCount>& values) {
  ASSERT_EQ(values.size(), kRawCandidateDiagnosticCount);
  for (const double value : values) EXPECT_TRUE(std::isfinite(value));
}

TEST(RawCandidateDiagnosticsTest, SchemaIsExactlyFortyNineAndNamesAreUnique) {
  static_assert(kRawCandidateDiagnosticCount == 49U, "schema count changed");
  const auto& names = rawCandidateDiagnosticFieldNames();
  ASSERT_EQ(names.size(), kRawCandidateDiagnosticCount);
  std::set<std::string> unique;
  for (const char* name : names) {
    ASSERT_NE(name, nullptr);
    EXPECT_TRUE(unique.insert(name).second);
  }
  EXPECT_EQ(std::string(names[kRawCandidateCurrentCrossSectionReason]),
            "current_cross_section_reason");
}

TEST(RawCandidateDiagnosticsTest, CompleteCurrentSampleExportsStoredFactsAndRawProbes) {
  const TubeProfile profile = MakeProfile();
  RawCandidateDiagnosticsInput input = MakeInput(&profile);
  input.epoch_status.candidate_raw_complete = true;
  input.epoch_status.candidate_filtered_complete = true;
  input.epoch_status.candidate_complete = true;
  const auto values = makeRawCandidateDiagnostics(input);
  ExpectFinite(values);
  EXPECT_DOUBLE_EQ(values[kRawCandidateSchemaVersion], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateRawSourceConfigured], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateRawStorageReady], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateRawQueryInjected], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateTubeUpdateDue], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateSequence], 11.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateSampleCount], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateInvalidCount], 0.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateUnavailableCount], 2.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateOutOfMapCount], 3.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateUnknownCount], 4.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateOccupiedCount], 5.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateInsufficientClearanceCount], 6.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentSampleFound], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentSampleComplete], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentCrossSectionReason],
                   static_cast<double>(TubeCrossSectionReason::NONE));
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentPositiveRayTermination],
                   static_cast<double>(TubeRayTermination::OCCUPIED));
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentNegativeRayTermination],
                   static_cast<double>(TubeRayTermination::SEARCH_EXTENT));
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentBaseRawStatus],
                   static_cast<double>(DistanceStatus::KNOWN_FREE));
  EXPECT_DOUBLE_EQ(values[kRawCandidateActualPositionRawStatus],
                   static_cast<double>(DistanceStatus::OCCUPIED));
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentPlusStepRawStatus],
                   static_cast<double>(DistanceStatus::UNKNOWN));
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentMinusStepRawStatus],
                   static_cast<double>(DistanceStatus::OUT_OF_MAP));
  EXPECT_NEAR(values[kRawCandidateCurrentCPlusRaw], 0.75, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentCMinusRaw], 1.25, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentEffectiveRadius], 0.55, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentObstacleLower], -0.70, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentObstacleUpper], 0.20, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentCurvatureLower], -3.0, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentCurvatureUpper], 2.0, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentEnvironmentLower], -0.70, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentEnvironmentUpper], 0.20, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentEnvironmentWidth], 0.90, kTolerance);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentContainsZero], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentContainsPreferredDelta], 0.0);
  EXPECT_NEAR(values[kRawCandidateCurrentBaseToActualNorm], 8.0, kTolerance);
}

TEST(RawCandidateDiagnosticsTest, CenterUnknownExportsExactFailureAndBaseStatus) {
  TubeRawSample sample = MakeCurrentSample();
  sample.complete = false;
  sample.cross_section_reason = TubeCrossSectionReason::CENTER_UNKNOWN;
  TubeProfile profile = MakeProfile(sample);
  RawCandidateDiagnosticsInput input = MakeInput(&profile);
  input.raw_occupancy_query = [](const Eigen::Vector3d&) {
    return DistanceStatus::UNKNOWN;
  };
  const auto values = makeRawCandidateDiagnostics(input);
  ExpectFinite(values);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentSampleFound], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentSampleComplete], 0.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentCrossSectionReason],
                   static_cast<double>(TubeCrossSectionReason::CENTER_UNKNOWN));
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentBaseRawStatus],
                   static_cast<double>(DistanceStatus::UNKNOWN));
}

TEST(RawCandidateDiagnosticsTest, CenterOccupiedExportsExactFailureAndStatus) {
  TubeRawSample sample = MakeCurrentSample();
  sample.complete = false;
  sample.cross_section_reason = TubeCrossSectionReason::CENTER_OCCUPIED;
  TubeProfile profile = MakeProfile(sample);
  RawCandidateDiagnosticsInput input = MakeInput(&profile);
  input.raw_occupancy_query = [](const Eigen::Vector3d&) {
    return DistanceStatus::OCCUPIED;
  };
  const auto values = makeRawCandidateDiagnostics(input);
  ExpectFinite(values);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentSampleComplete], 0.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentCrossSectionReason],
                   static_cast<double>(TubeCrossSectionReason::CENTER_OCCUPIED));
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentBaseRawStatus],
                   static_cast<double>(DistanceStatus::OCCUPIED));
}

TEST(RawCandidateDiagnosticsTest, ObstacleEmptyExportsReasonAndFinalBounds) {
  TubeRawSample sample = MakeCurrentSample();
  sample.complete = false;
  sample.cross_section_reason =
      TubeCrossSectionReason::EMPTY_AFTER_OBSTACLE_BOUNDS;
  sample.c_plus_raw = 0.25;
  sample.c_minus_raw = 0.25;
  sample.effective_radius = 0.30;
  sample.obstacle_lower = 0.05;
  sample.obstacle_upper = -0.05;
  sample.environment_lower = 0.05;
  sample.environment_upper = -0.05;
  sample.environment_width = -0.10;
  TubeProfile profile = MakeProfile(sample);
  const auto values = makeRawCandidateDiagnostics(MakeInput(&profile));
  ExpectFinite(values);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentCrossSectionReason],
                   static_cast<double>(
                       TubeCrossSectionReason::EMPTY_AFTER_OBSTACLE_BOUNDS));
  EXPECT_NEAR(values[kRawCandidateCurrentObstacleLower], 0.05, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentObstacleUpper], -0.05, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentEnvironmentLower], 0.05, kTolerance);
  EXPECT_NEAR(values[kRawCandidateCurrentEnvironmentUpper], -0.05, kTolerance);
}

TEST(RawCandidateDiagnosticsTest, FutureTruncationKeepsCurrentFactsAndExportsSegment) {
  TubeProfile profile = MakeProfile();
  profile.certified_segment_truncated_after = true;
  profile.certified_segment_end_w = 1.3;
  profile.preview_end_w = 1.3;
  profile.first_truncated_w = 1.4;
  profile.first_truncated_reason = TubeStopReason::UNKNOWN;
  RawCandidateDiagnosticsInput input = MakeInput(&profile);
  input.epoch_status.candidate_raw_complete = true;
  input.epoch_status.candidate_filtered_complete = true;
  input.epoch_status.candidate_complete = true;
  const auto values = makeRawCandidateDiagnostics(input);
  ExpectFinite(values);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentSampleFound], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentSampleComplete], 1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCertifiedSegmentTruncatedAfter], 1.0);
  EXPECT_NEAR(values[kRawCandidateCertifiedSegmentEndW], 1.3, kTolerance);
  EXPECT_NEAR(values[kRawCandidateFirstTruncatedW], 1.4, kTolerance);
  EXPECT_DOUBLE_EQ(values[kRawCandidateFirstTruncatedReason],
                   static_cast<double>(TubeStopReason::UNKNOWN));
}

TEST(RawCandidateDiagnosticsTest, MissingCurrentSampleUsesFiniteSentinels) {
  const TubeProfile profile = MakeProfile();
  RawCandidateDiagnosticsInput input = MakeInput(&profile);
  input.current_w = 9.0;
  input.current_path.w = 9.0;
  const auto values = makeRawCandidateDiagnostics(input);
  ExpectFinite(values);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentSampleFound], 0.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentSampleComplete], 0.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentCrossSectionReason], -1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentPositiveRayTermination], -1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentNegativeRayTermination], -1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentPlusStepRawStatus], -1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentMinusStepRawStatus], -1.0);
}

TEST(RawCandidateDiagnosticsTest, InvalidNormalUsesProbeSentinelsWithoutChangingBaseOrActual) {
  TubeRawSample sample = MakeCurrentSample();
  sample.N.setZero();
  TubeProfile profile = MakeProfile(sample);
  const auto values = makeRawCandidateDiagnostics(MakeInput(&profile));
  ExpectFinite(values);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentBaseRawStatus],
                   static_cast<double>(DistanceStatus::KNOWN_FREE));
  EXPECT_DOUBLE_EQ(values[kRawCandidateActualPositionRawStatus],
                   static_cast<double>(DistanceStatus::OCCUPIED));
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentPlusStepRawStatus], -1.0);
  EXPECT_DOUBLE_EQ(values[kRawCandidateCurrentMinusStepRawStatus], -1.0);
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
