#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

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

void SetV2Interval(phase_offset_core::Binary64Interval& interval,
                   const double lower, const double upper) {
  interval.lower = lower;
  interval.upper = upper;
  interval.valid = true;
}

void SetV2Vector(phase_offset_core::Binary64VectorInterval& interval,
                 const Eigen::Vector3d& value) {
  interval.valid = true;
  for (std::size_t index = 0U; index < 3U; ++index) {
    SetV2Interval(interval.component[index], value(index), value(index));
  }
}

std::shared_ptr<const ContinuousPhasePath> MakeV2MarkerPath(
    const std::uint64_t revision = 41U) {
  std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  if (!path->appendSegment(
          0.0, 1.0, "v2-marker-straight",
          [](const double w, ContinuousPhasePathState& state) {
            state.p = Eigen::Vector3d(w, 0.0, 1.0);
            state.dp_dw = Eigen::Vector3d::UnitX();
            state.d2p_dw2.setZero();
            state.vel = state.dp_dw;
            state.valid = std::isfinite(w) && w >= 0.0 && w <= 1.0;
            return state.valid;
          })) {
    return std::shared_ptr<const ContinuousPhasePath>();
  }
  path->setPathRevision(revision);
  return path;
}

phase_offset_core::CertifiedPathCellV2 MakeV2MarkerPathCell(
    const std::uint64_t revision) {
  phase_offset_core::CertifiedPathCellV2 cell;
  cell.w0 = 0.0;
  cell.w1 = 1.0;
  cell.anchor_w = 0.5;
  cell.path_revision = revision;
  cell.frame_revision = revision;
  cell.segment_identity = 1U;
  cell.proof_identity = 2U;
  SetV2Vector(cell.anchor_position, Eigen::Vector3d(0.5, 0.0, 1.0));
  SetV2Vector(cell.anchor_p_w, Eigen::Vector3d::UnitX());
  SetV2Vector(cell.anchor_p_ww, Eigen::Vector3d::Zero());
  SetV2Interval(cell.inf_p_w_norm, 1.0, 1.0);
  SetV2Interval(cell.sup_p_w_norm, 1.0, 1.0);
  SetV2Interval(cell.inf_horizontal_p_w_norm, 1.0, 1.0);
  SetV2Interval(cell.sup_p_ww_norm, 0.0, 0.0);
  SetV2Interval(cell.sup_horizontal_p_ww_norm, 0.0, 0.0);
  SetV2Interval(cell.sup_p_www_norm, 0.0, 0.0);
  SetV2Interval(cell.sup_normal_derivative, 0.0, 0.0);
  SetV2Interval(cell.normal_variation, 0.0, 0.0);
  SetV2Interval(cell.tangent_variation, 0.0, 0.0);
  SetV2Interval(cell.curvature_variation, 0.0, 0.0);
  SetV2Interval(cell.midpoint_position_variation, 0.0, 0.5);
  SetV2Interval(cell.chord_deviation, 0.0, 0.0);
  cell.horizontal_acceleration_bound_complete = true;
  cell.normal_frame_proof_complete = true;
  cell.phase_map_proof_complete = true;
  cell.provenance =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  cell.valid = true;
  cell.complete = true;
  return cell;
}

std::shared_ptr<phase_offset_navigation::TubeProfileV2> MakeV2MarkerProfile(
    const std::shared_ptr<const ContinuousPhasePath>& path) {
  const std::uint64_t revision = path ? path->pathRevision() : 41U;
  std::shared_ptr<phase_offset_navigation::TubeProfileV2> profile(
      new phase_offset_navigation::TubeProfileV2());
  profile->path_key.execution_generation = 3U;
  profile->path_key.path_instance_id = 17U;
  profile->path_key.path_revision = revision;
  profile->path_key.frame_revision = revision;
  profile->path_key.frame_convention_id = 1U;
  profile->path_key.frame_convention =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  profile->path_key.phase_orientation = 1;
  profile->path_key.domain_start = 0.0;
  profile->path_key.domain_end = 1.0;
  profile->configuration_key.configuration_id = 5U;
  profile->configuration_key.epsilon = 0.4;
  profile->configuration_key.nominal_half_width = 0.5;
  profile->configuration_key.ray_step = 0.05;
  profile->configuration_key.snapshot_resolution = 0.1;
  profile->configuration_key.minimum_reference_speed = 1e-8;
  profile->map_capture_key.map_instance_id = 7U;
  profile->map_capture_key.state_id = 8U;
  profile->map_capture_key.accepted_sequence = 8U;
  profile->map_capture_key.configuration_generation = 9U;
  profile->map_capture_key.configuration_id = 5U;
  profile->map_capture_key.frame_provenance_id = 10U;
  profile->map_capture_key.frame_provenance = "world";
  profile->map_capture_key.support_provenance_id = 11U;
  profile->map_capture_key.accepted_time_ticks = 12U;
  profile->map_capture_key.support_expiry_timeless = true;
  profile->map_capture_key.halo_reconciled = true;
  profile->map_capture_key.grid_min_index_x = -1;
  profile->map_capture_key.grid_min_index_y = -1;
  profile->map_capture_key.grid_min_index_z = -1;
  profile->map_capture_key.grid_max_index_x = 1;
  profile->map_capture_key.grid_max_index_y = 1;
  profile->map_capture_key.grid_max_index_z = 1;
  profile->map_capture_key.grid_voxel_resolution =
      Eigen::Vector3d::Constant(0.1);
  profile->map_capture_key.complete_support = true;
  profile->profile_id = 13U;
  profile->request_id = 14U;
  profile->requested_start = 0.0;
  profile->requested_end = 1.0;
  profile->anchor_w = 0.5;
  profile->certified_start = 0.0;
  profile->certified_end = 1.0;
  profile->valid = true;
  profile->complete = true;
  profile->contains_anchor = true;
  profile->contains_zero_everywhere = true;
  profile->nonzero_capacity = true;
  profile->capability =
      phase_offset_navigation::TubeProfileV2Capability::OFFSET_CERTIFIED;
  profile->path_owner = path;
  profile->capture_owner = std::make_shared<const int>(1);
  profile->query_owner = std::make_shared<const int>(2);
  profile->applicability_assumptions = "v2-marker-test";
  profile->applicability_deadline_timeless = true;

  phase_offset_navigation::TubePwlKnotV2 first;
  first.w = 0.0;
  first.lower = -0.2;
  first.upper = 0.3;
  first.right_lower_slope_interval = {0.0, 0.0, true};
  first.right_upper_slope_interval = {0.0, 0.0, true};
  first.right_cell_id = 1U;
  first.valid = true;
  phase_offset_navigation::TubePwlKnotV2 last = first;
  last.w = 1.0;
  last.left_lower_slope_interval = {0.0, 0.0, true};
  last.left_upper_slope_interval = {0.0, 0.0, true};
  last.right_lower_slope_interval = {};
  last.right_upper_slope_interval = {};
  last.left_cell_id = 1U;
  last.right_cell_id = 0U;
  profile->knots = {first, last};

  phase_offset_navigation::TubeProofCellV2 proof;
  proof.w0 = 0.0;
  proof.w1 = 1.0;
  proof.lower = -0.2;
  proof.upper = 0.3;
  proof.cell_id = 1U;
  proof.segment_identity = 1U;
  proof.proof_identity = 2U;
  proof.complete = true;
  proof.valid = true;
  proof.path_cell = MakeV2MarkerPathCell(revision);
  profile->cells.push_back(proof);
  EXPECT_TRUE(profile->structurallyValid());
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

TEST(TubeMarkersTest, V2ActiveAndCandidateUseCertifiedKnotsAndFrameOwner) {
  const auto path = MakeV2MarkerPath();
  ASSERT_TRUE(path);
  const auto profile = MakeV2MarkerProfile(path);
  const auto frame = std::make_shared<const ContinuousPhaseNormalFrame>(
      path, profile->path_key.path_revision, profile->path_key.frame_revision);
  const auto active = MakeCertifiedTubeMarkersV2(
      ros::Time(2.0), "world", profile, frame);
  const auto candidate = MakeCandidateTubeMarkersV2(
      ros::Time(2.0), "world", profile, frame);
  ExpectActions(active, visualization_msgs::Marker::ADD);
  ExpectActions(candidate, visualization_msgs::Marker::ADD);
  ASSERT_EQ(active.markers[0].points.size(), 2U);
  ASSERT_EQ(active.markers[1].points.size(), 2U);
  EXPECT_DOUBLE_EQ(active.markers[0].points[0].x, 0.0);
  EXPECT_DOUBLE_EQ(active.markers[0].points[0].y, -0.2);
  EXPECT_DOUBLE_EQ(active.markers[0].points[0].z, 1.0);
  EXPECT_DOUBLE_EQ(active.markers[1].points[1].x, 1.0);
  EXPECT_DOUBLE_EQ(active.markers[1].points[1].y, 0.3);
  EXPECT_EQ(active.markers[0].ns, "phase_offset_manual_tube");
  EXPECT_EQ(candidate.markers[0].ns,
            "phase_offset_manual_tube_candidate");
  EXPECT_FLOAT_EQ(active.markers[2].color.b, 1.0F);
  EXPECT_GT(candidate.markers[2].color.r,
            candidate.markers[2].color.b);
  EXPECT_EQ(active.markers[2].points.size(), 6U);
}

TEST(TubeMarkersTest, V2ZeroOnlyProfileRemainsVisibleOnCenterline) {
  const auto path = MakeV2MarkerPath();
  auto profile = MakeV2MarkerProfile(path);
  for (auto& knot : profile->knots) {
    knot.lower = 0.0;
    knot.upper = 0.0;
  }
  profile->cells[0].lower = 0.0;
  profile->cells[0].upper = 0.0;
  profile->nonzero_capacity = false;
  profile->capability =
      phase_offset_navigation::TubeProfileV2Capability::ZERO_ONLY;
  ASSERT_TRUE(profile->structurallyValid());
  const auto frame = std::make_shared<const ContinuousPhaseNormalFrame>(
      path, profile->path_key.path_revision, profile->path_key.frame_revision);
  const auto markers = MakeCertifiedTubeMarkersV2(
      ros::Time(2.0), "world", profile, frame);
  ExpectActions(markers, visualization_msgs::Marker::ADD);
  ASSERT_EQ(markers.markers[0].points.size(), 2U);
  EXPECT_DOUBLE_EQ(markers.markers[0].points[0].y, 0.0);
  EXPECT_DOUBLE_EQ(markers.markers[1].points[0].y, 0.0);
}

TEST(TubeMarkersTest, V2InvalidOrPathFrameMismatchDeletesWholeBundle) {
  const auto path = MakeV2MarkerPath();
  auto profile = MakeV2MarkerProfile(path);
  const auto correct_frame =
      std::make_shared<const ContinuousPhaseNormalFrame>(path, 41U, 41U);
  profile->valid = false;
  ExpectActions(MakeCertifiedTubeMarkersV2(
                    ros::Time(2.0), "world", profile, correct_frame),
                visualization_msgs::Marker::DELETE);
  profile->valid = true;
  const auto wrong_frame =
      std::make_shared<const ContinuousPhaseNormalFrame>(path, 41U, 42U);
  ExpectActions(MakeCandidateTubeMarkersV2(
                    ros::Time(2.0), "world", profile, wrong_frame),
                visualization_msgs::Marker::DELETE);
  profile->knots[1].w = profile->knots[0].w;
  ExpectActions(MakeCertifiedTubeMarkersV2(
                    ros::Time(2.0), "world", profile, correct_frame),
                visualization_msgs::Marker::DELETE);
}

TEST(TubeMarkersTest, V2MissingSnapshotPublishesDeletesForBothTopics) {
  const std::shared_ptr<const phase_offset_navigation::TubeProfileV2> profile;
  const std::shared_ptr<const ContinuousPhaseNormalFrame> frame;
  ExpectActions(MakeCertifiedTubeMarkersV2(
                    ros::Time(2.0), "world", profile, frame),
                visualization_msgs::Marker::DELETE);
  ExpectActions(MakeCandidateTubeMarkersV2(
                    ros::Time(2.0), "world", profile, frame),
                visualization_msgs::Marker::DELETE);
}

// ---------------------------------------------------------------------------
// M3C Section display.  These cases exercise only the immutable path plus the
// committed SectionTubeProfile; no map identity, certificate, or worker state
// is involved.
// ---------------------------------------------------------------------------

std::shared_ptr<const ContinuousPhasePath> MakeSectionMarkerPath(
    const double end_w = 2.0, const std::uint64_t revision = 5U,
    const bool curved = false) {
  std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  if (!path->appendSegment(
          0.0, end_w, "section-marker",
          [end_w, curved](const double w, ContinuousPhasePathState& state) {
            state.p = curved
                ? Eigen::Vector3d(w, 0.4 * std::sin(w), 1.0)
                : Eigen::Vector3d(w, 0.0, 1.0);
            state.dp_dw = curved
                ? Eigen::Vector3d(1.0, 0.4 * std::cos(w), 0.0)
                : Eigen::Vector3d::UnitX();
            state.d2p_dw2 = curved
                ? Eigen::Vector3d(0.0, -0.4 * std::sin(w), 0.0)
                : Eigen::Vector3d::Zero();
            state.vel = state.dp_dw;
            state.valid = std::isfinite(w) && w >= 0.0 && w <= end_w;
            return state.valid;
          })) {
    return std::shared_ptr<const ContinuousPhasePath>();
  }
  path->setPathRevision(revision);
  return std::shared_ptr<const ContinuousPhasePath>(path);
}

std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>
MakeSectionMarkerProfile(const bool complete = true,
                         const bool finite = true) {
  std::shared_ptr<phase_offset_navigation::SectionTubeProfile> profile(
      new phase_offset_navigation::SectionTubeProfile());
  profile->valid_start = 0.0;
  profile->valid_end = 2.0;
  profile->status = complete
      ? phase_offset_navigation::SectionTubeStatus::COMPLETE
      : phase_offset_navigation::SectionTubeStatus::PARTIAL;
  profile->usable = true;
  profile->complete = complete;
  for (const double w : {0.0, 0.5, 1.0, 1.5, 2.0}) {
    phase_offset_navigation::SectionTubeKnot knot;
    knot.w = w;
    knot.lower = -0.3;
    knot.upper = 0.6;
    profile->knots.push_back(knot);
  }
  if (!finite) {
    profile->knots.front().upper =
        std::numeric_limits<double>::quiet_NaN();
  }
  return std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>(
      profile);
}

void ExpectSectionDelete(const visualization_msgs::MarkerArray& markers) {
  ASSERT_EQ(markers.markers.size(), 3U);
  EXPECT_EQ(markers.markers[0].ns, "phase_offset_section_tube_left");
  EXPECT_EQ(markers.markers[1].ns, "phase_offset_section_tube_right");
  EXPECT_EQ(markers.markers[2].ns, "phase_offset_section_tube_fill");
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::Marker::DELETE);
  EXPECT_EQ(markers.markers[1].action, visualization_msgs::Marker::DELETE);
  EXPECT_EQ(markers.markers[2].action, visualization_msgs::Marker::DELETE);
}

TEST(SectionTubeMarkersTest, AsymmetricStraightProfileDrawsBothBoundaries) {
  const auto path = MakeSectionMarkerPath();
  const auto profile = MakeSectionMarkerProfile();
  ASSERT_TRUE(path);
  ASSERT_TRUE(profile);
  const auto markers = MakeSectionTubeMarkers(
      ros::Time(3.0), "world", path, profile);
  ASSERT_EQ(markers.markers.size(), 3U);
  const auto& left = markers.markers[0];
  const auto& right = markers.markers[1];
  const auto& fill = markers.markers[2];
  EXPECT_EQ(left.ns, "phase_offset_section_tube_left");
  EXPECT_EQ(right.ns, "phase_offset_section_tube_right");
  EXPECT_EQ(left.action, visualization_msgs::Marker::ADD);
  EXPECT_EQ(right.action, visualization_msgs::Marker::ADD);
  EXPECT_EQ(left.type, visualization_msgs::Marker::LINE_STRIP);
  EXPECT_EQ(right.type, visualization_msgs::Marker::LINE_STRIP);
  EXPECT_EQ(left.points.size(), profile->knots.size());
  EXPECT_EQ(right.points.size(), profile->knots.size());
  // The straight path is along +x, so the horizontal normal is +y and the
  // signed offsets appear directly on the y coordinate.
  for (std::size_t index = 0U; index < left.points.size(); ++index) {
    EXPECT_DOUBLE_EQ(left.points[index].y, -0.3);
    EXPECT_DOUBLE_EQ(right.points[index].y, 0.6);
    EXPECT_DOUBLE_EQ(left.points[index].x,
                     profile->knots[index].w);
  }
  EXPECT_FLOAT_EQ(left.color.a, 1.0F);
  EXPECT_FLOAT_EQ(right.color.a, 1.0F);
  // The band between the boundaries is a filled triangle list whose scale must
  // stay at one on every axis.
  EXPECT_EQ(fill.ns, "phase_offset_section_tube_fill");
  EXPECT_EQ(fill.type, visualization_msgs::Marker::TRIANGLE_LIST);
  EXPECT_EQ(fill.action, visualization_msgs::Marker::ADD);
  EXPECT_EQ(fill.points.size(), 6U * (profile->knots.size() - 1U));
  EXPECT_FLOAT_EQ(fill.scale.x, 1.0F);
  EXPECT_FLOAT_EQ(fill.scale.y, 1.0F);
  EXPECT_FLOAT_EQ(fill.scale.z, 1.0F);
  EXPECT_FLOAT_EQ(fill.color.a, 0.35F);
}

TEST(SectionTubeMarkersTest, CurvedPathOffsetsAlongItsOwnNormal) {
  const auto path = MakeSectionMarkerPath(2.0, 5U, true);
  const auto profile = MakeSectionMarkerProfile();
  ASSERT_TRUE(path);
  const auto markers = MakeSectionTubeMarkers(
      ros::Time(3.0), "world", path, profile);
  ASSERT_EQ(markers.markers.size(), 3U);
  ASSERT_EQ(markers.markers[0].points.size(), profile->knots.size());
  // Boundary points must not collapse onto the centerline, and the two
  // boundaries must stay on opposite sides of it.
  for (std::size_t index = 0U; index < markers.markers[0].points.size();
       ++index) {
    const auto& low = markers.markers[0].points[index];
    const auto& high = markers.markers[1].points[index];
    const double separation =
        std::hypot(high.x - low.x, high.y - low.y);
    EXPECT_GT(separation, 0.5);
  }
  const auto& second = markers.markers[0].points[1];
  const auto& first = markers.markers[0].points[0];
  const double dx = second.x - first.x;
  const double dy = second.y - first.y;
  EXPECT_GT(std::abs(dy), 1e-6);
  EXPECT_GT(dx, 0.0);
}

TEST(SectionTubeMarkersTest, PartialProfileDimsBothBoundaries) {
  const auto path = MakeSectionMarkerPath();
  const auto profile = MakeSectionMarkerProfile(false);
  const auto markers = MakeSectionTubeMarkers(
      ros::Time(3.0), "world", path, profile);
  ASSERT_EQ(markers.markers.size(), 3U);
  EXPECT_EQ(markers.markers[0].action, visualization_msgs::Marker::ADD);
  EXPECT_EQ(markers.markers[1].action, visualization_msgs::Marker::ADD);
  EXPECT_EQ(markers.markers[2].action, visualization_msgs::Marker::ADD);
  EXPECT_FLOAT_EQ(markers.markers[0].color.a, 0.45F);
  EXPECT_FLOAT_EQ(markers.markers[1].color.a, 0.45F);
  EXPECT_FLOAT_EQ(markers.markers[2].color.a, 0.20F);
}

TEST(SectionTubeMarkersTest, MissingOrMalformedInputDeletesBothMarkers) {
  const auto path = MakeSectionMarkerPath();
  const auto profile = MakeSectionMarkerProfile();
  const auto empty_profile =
      std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>();
  const auto empty_path = std::shared_ptr<const ContinuousPhasePath>();

  ExpectSectionDelete(
      MakeSectionTubeMarkers(ros::Time(3.0), "world", empty_path, profile));
  ExpectSectionDelete(
      MakeSectionTubeMarkers(ros::Time(3.0), "world", path, empty_profile));
  ExpectSectionDelete(
      MakeSectionTubeMarkers(ros::Time(3.0), "world", path,
                             MakeSectionMarkerProfile(true, false)));

  auto single_knot = MakeSectionMarkerProfile();
  auto mutable_single =
      std::const_pointer_cast<phase_offset_navigation::SectionTubeProfile>(
          single_knot);
  mutable_single->knots.resize(1U);
  ExpectSectionDelete(MakeSectionTubeMarkers(
      ros::Time(3.0), "world", path, single_knot));

  auto out_of_domain = MakeSectionMarkerProfile();
  auto mutable_domain =
      std::const_pointer_cast<phase_offset_navigation::SectionTubeProfile>(
          out_of_domain);
  mutable_domain->knots.back().w = 5.0;
  ExpectSectionDelete(MakeSectionTubeMarkers(
      ros::Time(3.0), "world", path, out_of_domain));
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
