#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "bspline_race/integration/phase_offset_tube_markers.h"

namespace FLAG_Race {
namespace {

phase_offset_navigation::TubeProfile MakeProfile(
    const std::size_t sample_count = 2U,
    const bool complete = true,
    const bool finite = true,
    const phase_offset_navigation::TubeSource source =
        phase_offset_navigation::TubeSource::FIXED) {
  phase_offset_navigation::TubeProfile profile;
  profile.source = source;
  profile.complete = complete;
  for (std::size_t index = 0U; index < sample_count; ++index) {
    phase_offset_navigation::TubeRawSample sample;
    sample.p = Eigen::Vector3d(static_cast<double>(index), 0.0, 1.0);
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.filtered_lower = finite ? -0.04
        : (index == 0U ? std::numeric_limits<double>::quiet_NaN() : -0.04);
    sample.filtered_upper = 0.04;
    profile.samples.push_back(sample);
  }
  return profile;
}

void ExpectActions(const visualization_msgs::MarkerArray& markers,
                   const int action) {
  ASSERT_EQ(markers.markers.size(), 3U);
  for (std::size_t index = 0U; index < markers.markers.size(); ++index) {
    EXPECT_EQ(markers.markers[index].id, static_cast<int>(index));
    EXPECT_EQ(markers.markers[index].action, action);
  }
}

phase_offset_navigation::TubeProfile MakeRawCandidateProfile() {
  phase_offset_navigation::TubeProfile profile = MakeProfile(2U);
  profile.complete = true;
  for (auto& sample : profile.samples) {
    sample.complete = true;
    sample.filtered_lower = 0.0;
    sample.filtered_upper = 0.0;
  }
  profile.raw_build_samples.clear();
  for (const double w : {0.0, 0.5, 1.0}) {
    phase_offset_navigation::TubeRawSample sample;
    sample.w = w;
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.filtered_lower = -0.04;
    sample.filtered_upper = 0.04;
    sample.raw_lower = sample.filtered_lower;
    sample.raw_upper = sample.filtered_upper;
    sample.complete = true;
    profile.raw_build_samples.push_back(sample);
  }
  return profile;
}

TEST(TubeMarkersTest, CompleteFixedProfileAddsBothWhenCertified) {
  const auto profile = MakeProfile();
  const ros::Time stamp(1.0);
  const auto candidate = MakeCandidateTubeMarkers(stamp, "world", profile, true);
  const auto certified = MakeCertifiedTubeMarkers(stamp, "world", profile, true);
  ExpectActions(candidate, visualization_msgs::Marker::ADD);
  ExpectActions(certified, visualization_msgs::Marker::ADD);
  EXPECT_FLOAT_EQ(candidate.markers[2].scale.x, 1.0F);
  EXPECT_FLOAT_EQ(candidate.markers[2].scale.y, 1.0F);
  EXPECT_FLOAT_EQ(candidate.markers[2].scale.z, 1.0F);
  EXPECT_FLOAT_EQ(certified.markers[2].scale.x, 1.0F);
  EXPECT_FLOAT_EQ(certified.markers[2].scale.y, 1.0F);
  EXPECT_FLOAT_EQ(certified.markers[2].scale.z, 1.0F);
}

TEST(TubeMarkersTest, CandidateIsIndependentOfCertificationState) {
  const auto profile = MakeProfile();
  const ros::Time stamp(1.0);
  const auto candidate = MakeCandidateTubeMarkers(stamp, "world", profile, true);
  const auto certified = MakeCertifiedTubeMarkers(stamp, "world", profile, false);
  ExpectActions(candidate, visualization_msgs::Marker::ADD);
  ExpectActions(certified, visualization_msgs::Marker::DELETE);
}

TEST(TubeMarkersTest, CandidateRequiresManualMode) {
  const auto profile = MakeProfile();
  const auto candidate = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, false);
  ExpectActions(candidate, visualization_msgs::Marker::DELETE);
}

TEST(TubeMarkersTest, NoneSourceDeletesCandidate) {
  auto profile = MakeProfile();
  profile.source = phase_offset_navigation::TubeSource::NONE;
  const auto candidate = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  ExpectActions(candidate, visualization_msgs::Marker::DELETE);
}

TEST(TubeMarkersTest, IncompleteProfileDeletesBoth) {
  const auto profile = MakeProfile(2U, false);
  const auto candidate = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  const auto certified = MakeCertifiedTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  ExpectActions(candidate, visualization_msgs::Marker::DELETE);
  ExpectActions(certified, visualization_msgs::Marker::DELETE);
}

TEST(TubeMarkersTest, NonFiniteBoundaryDeletesBoth) {
  const auto profile = MakeProfile(2U, true, false);
  const auto candidate = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  const auto certified = MakeCertifiedTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  ExpectActions(candidate, visualization_msgs::Marker::DELETE);
  ExpectActions(certified, visualization_msgs::Marker::DELETE);
}

TEST(TubeMarkersTest, FewerThanTwoSamplesDeletesBoth) {
  const auto profile = MakeProfile(1U);
  const auto candidate = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  const auto certified = MakeCertifiedTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  ExpectActions(candidate, visualization_msgs::Marker::DELETE);
  ExpectActions(certified, visualization_msgs::Marker::DELETE);
}

TEST(TubeMarkersTest, RibbonContainsExactlySixVerticesPerSamplePair) {
  const auto profile = MakeProfile(5U);
  const auto candidate = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  const auto certified = MakeCertifiedTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  EXPECT_EQ(candidate.markers[2].points.size(),
            6U * (profile.samples.size() - 1U));
  EXPECT_EQ(certified.markers[2].points.size(),
            6U * (profile.samples.size() - 1U));
}

TEST(TubeMarkersTest, CandidateAndCertifiedShareExactGeometry) {
  const auto profile = MakeProfile(5U);
  const auto candidate = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  const auto certified = MakeCertifiedTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  for (std::size_t marker = 0U; marker < 3U; ++marker) {
    ASSERT_EQ(candidate.markers[marker].points.size(),
              certified.markers[marker].points.size());
    for (std::size_t point = 0U; point < candidate.markers[marker].points.size();
         ++point) {
      EXPECT_NEAR(candidate.markers[marker].points[point].x,
                  certified.markers[marker].points[point].x, 0.0);
      EXPECT_NEAR(candidate.markers[marker].points[point].y,
                  certified.markers[marker].points[point].y, 0.0);
      EXPECT_NEAR(candidate.markers[marker].points[point].z,
                  certified.markers[marker].points[point].z, 0.0);
    }
  }
}

TEST(TubeMarkersTest, CandidateNamespaceStyleAndAlphaAreUncertified) {
  const auto profile = MakeProfile();
  const auto candidate = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  for (const auto& marker : candidate.markers) {
    EXPECT_EQ(marker.ns, "phase_offset_manual_tube_candidate");
  }
  EXPECT_NE(candidate.markers[0].ns, "phase_offset_manual_tube");
  EXPECT_NEAR(candidate.markers[2].color.a, 0.10F, 1e-6F);
  EXPECT_GT(candidate.markers[2].color.a, 0.0F);
  EXPECT_LT(candidate.markers[2].color.a, 0.15F);
  EXPECT_FALSE(candidate.markers[0].color.g > 0.7F);
  EXPECT_FALSE(candidate.markers[1].color.g > 0.7F);
}

TEST(TubeMarkersTest, CandidateUsesCurrentEpochRawGeometryWhenFinalProfileIsZero) {
  const auto profile = MakeRawCandidateProfile();
  const auto candidate = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true, 0.5);
  const auto certified = MakeCertifiedTubeMarkers(
      ros::Time(1.0), "world", profile, false);
  ExpectActions(candidate, visualization_msgs::Marker::ADD);
  ExpectActions(certified, visualization_msgs::Marker::DELETE);
  EXPECT_EQ(candidate.markers[0].points.size(), 3U);
  EXPECT_NEAR(candidate.markers[0].points[1].x, 0.5, 0.0);
  EXPECT_NEAR(candidate.markers[0].points[1].y, -0.04, 0.0);
}

TEST(TubeMarkersTest, CandidateRequiresExactCurrentRawAnchor) {
  const auto profile = MakeRawCandidateProfile();
  const auto near_anchor = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true, 0.5000000001);
  ExpectActions(near_anchor, visualization_msgs::Marker::DELETE);
}

TEST(TubeMarkersTest, CandidateRejectsSingleCurrentRawSampleOrRemoteGap) {
  auto profile = MakeRawCandidateProfile();
  profile.raw_build_samples[0].complete = false;
  profile.raw_build_samples[2].complete = false;
  const auto markers = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true, 0.5);
  ExpectActions(markers, visualization_msgs::Marker::DELETE);
}

TEST(TubeMarkersTest,
     CandidateKeepsBroadRawGeometryWhileCertifiedMarkerUsesNarrowProfile) {
  phase_offset_navigation::TubeProfile profile = MakeProfile(
      3U, true, true, phase_offset_navigation::TubeSource::ESDF);
  profile.raw_complete = true;
  profile.filtered_complete = true;
  profile.obstacle_certified = true;
  profile.classification =
      phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
  profile.raw_build_samples.clear();
  for (std::size_t index = 0U; index < profile.samples.size(); ++index) {
    auto& certified = profile.samples[index];
    certified.w = 0.5 * static_cast<double>(index);
    certified.p.x() = certified.w;
    certified.complete = true;
    certified.filtered_lower = -0.10;
    certified.filtered_upper = 0.10;
    auto raw = certified;
    raw.raw_lower = -2.0;
    raw.raw_upper = 2.0;
    raw.filtered_lower = raw.raw_lower;
    raw.filtered_upper = raw.raw_upper;
    profile.raw_build_samples.push_back(raw);
  }

  const auto candidate = MakeCandidateTubeMarkers(
      ros::Time(1.0), "world", profile, true, 0.5);
  const auto certified = MakeCertifiedTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  ExpectActions(candidate, visualization_msgs::Marker::ADD);
  ExpectActions(certified, visualization_msgs::Marker::ADD);
  ASSERT_EQ(candidate.markers[0].points.size(), 3U);
  ASSERT_EQ(certified.markers[0].points.size(), 3U);
  EXPECT_DOUBLE_EQ(candidate.markers[0].points[1].y, -2.0);
  EXPECT_DOUBLE_EQ(candidate.markers[1].points[1].y, 2.0);
  EXPECT_DOUBLE_EQ(certified.markers[0].points[1].y, -0.10);
  EXPECT_DOUBLE_EQ(certified.markers[1].points[1].y, 0.10);
  EXPECT_EQ(candidate.markers[0].ns, "phase_offset_manual_tube_candidate");
  EXPECT_EQ(certified.markers[0].ns, "phase_offset_manual_tube");
}

TEST(TubeMarkersTest, CertifiedGeometryUsesFilteredEsdfSamplesOnly) {
  auto profile = MakeProfile(4U, true, true,
                             phase_offset_navigation::TubeSource::ESDF);
  profile.raw_build_samples = profile.samples;
  for (std::size_t index = 0U; index < profile.samples.size(); ++index) {
    profile.samples[index].w = 0.5 * static_cast<double>(index);
    profile.samples[index].complete = true;
    profile.samples[index].filtered_lower = -0.10;
    profile.samples[index].filtered_upper = 0.10;
    profile.raw_build_samples[index].raw_lower = -2.0;
    profile.raw_build_samples[index].raw_upper = 2.0;
    profile.raw_build_samples[index].filtered_lower = -2.0;
    profile.raw_build_samples[index].filtered_upper = 2.0;
  }
  const auto markers = MakeCertifiedGeometryTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  ExpectActions(markers, visualization_msgs::Marker::ADD);
  ASSERT_EQ(markers.markers[0].points.size(), profile.samples.size());
  ASSERT_EQ(markers.markers[1].points.size(), profile.samples.size());
  EXPECT_DOUBLE_EQ(markers.markers[0].points[1].y, -0.10);
  EXPECT_DOUBLE_EQ(markers.markers[1].points[1].y, 0.10);
  EXPECT_EQ(markers.markers[0].ns,
            "phase_offset_manual_tube_certified_geometry");
  EXPECT_EQ(markers.markers[1].type, visualization_msgs::Marker::LINE_STRIP);
  EXPECT_EQ(markers.markers[2].type,
            visualization_msgs::Marker::TRIANGLE_LIST);
  EXPECT_EQ(markers.markers[2].points.size(),
            6U * (profile.samples.size() - 1U));
  EXPECT_FLOAT_EQ(markers.markers[2].color.r, 1.0F);
  EXPECT_FLOAT_EQ(markers.markers[2].color.g, 0.2F);
}

TEST(TubeMarkersTest, CertifiedGeometryRejectsInvalidFilteredSamplesAsAllDelete) {
  auto profile = MakeProfile(3U, true, true,
                             phase_offset_navigation::TubeSource::ESDF);
  profile.samples[1].w = profile.samples[0].w;
  const auto duplicate = MakeCertifiedGeometryTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  ExpectActions(duplicate, visualization_msgs::Marker::DELETE);
  profile = MakeProfile(3U, true, true,
                        phase_offset_navigation::TubeSource::ESDF);
  profile.samples[0].filtered_lower = 0.2;
  profile.samples[0].filtered_upper = -0.2;
  const auto inverted = MakeCertifiedGeometryTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  ExpectActions(inverted, visualization_msgs::Marker::DELETE);
  profile = MakeProfile(1U, true, true,
                        phase_offset_navigation::TubeSource::ESDF);
  const auto short_profile = MakeCertifiedGeometryTubeMarkers(
      ros::Time(1.0), "world", profile, true);
  ExpectActions(short_profile, visualization_msgs::Marker::DELETE);
}

TEST(TubeMarkersTest, CertifiedGeometryRequiresEsdfAndDisplayability) {
  const auto fixed = MakeProfile();
  const auto fixed_markers = MakeCertifiedGeometryTubeMarkers(
      ros::Time(1.0), "world", fixed, true);
  ExpectActions(fixed_markers, visualization_msgs::Marker::DELETE);
  auto profile = MakeProfile(2U, true, true,
                             phase_offset_navigation::TubeSource::ESDF);
  const auto denied = MakeCertifiedGeometryTubeMarkers(
      ros::Time(1.0), "world", profile, false);
  ExpectActions(denied, visualization_msgs::Marker::DELETE);
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
