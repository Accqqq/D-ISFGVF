#include <gtest/gtest.h>

#include "phase_offset_navigation/tube_epoch_manager.h"

#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <utility>

namespace phase_offset_navigation {
namespace {

TubeEpochPathSamples MakePath(double end_w = 0.6) {
  TubeEpochPathSamples path;
  for (int index = 0; index <= static_cast<int>(std::round(end_w / 0.1)); ++index) {
    const double w = 0.1 * static_cast<double>(index);
    phase_offset_core::PathDifferentialState state;
    state.p = Eigen::Vector3d(w, 0.0, 1.0 + 0.1 * w);
    state.p_w = Eigen::Vector3d(1.0, 0.0, 0.1);
    state.p_ww = Eigen::Vector3d::Zero();
    state.w = w;
    state.valid = true;
    path.push_back(state);
  }
  return path;
}

TubeEpochManagerConfig MakeConfig() {
  TubeEpochManagerConfig config;
  config.builder.fixed_delta_max = 0.10;
  config.builder.max_offset = 0.10;
  config.builder.sample_step_w = 0.10;
  config.builder.lookahead_w = 0.60;
  config.builder.back_w = 0.0;
  config.builder.min_certified_forward_w = 0.30;
  config.builder.ray_step = 0.05;
  config.builder.interior_margin = 0.0;
  config.builder.erosion.uav_radius = 0.01;
  config.builder.erosion.localization_margin = 0.01;
  config.builder.erosion.tracking_error_bound = 0.03;
  config.builder.erosion.map_margin = 0.01;
  config.builder.erosion.extra_margin = 0.01;
  config.builder.erosion.discretization_margin = 0.01;
  config.builder.cross_section.search_extent = 3.0;
  config.builder.cross_section.ray_step = 0.05;
  config.builder.cross_section.boundary_tolerance = 1e-4;
  config.builder.cross_section.regularity_margin = 0.1;
  config.builder.cross_section.curvature_epsilon = 1e-8;
  config.builder.cross_section.planner_safe_distance = 0.40;
  config.builder.cross_section.margins.uav_radius = 0.20;
  config.builder.cross_section.margins.map_uncertainty = 0.05;
  config.builder.cross_section.margins.localization_uncertainty = 0.03;
  config.builder.cross_section.margins.tracking_error_bound = 0.02;
  config.filter.boundary_slope_max = 0.8;
  return config;
}

TubeEpochUpdateInput MakeInput(TubeSource source = TubeSource::FIXED,
                               double end_w = 0.6) {
  TubeEpochUpdateInput input;
  input.source = source;
  input.preview_path = MakePath(end_w);
  input.current_path = input.preview_path.front();
  input.actual_position = input.current_path.p;
  input.path_source_revision = 1U;
  input.map_observation_sequence = 1U;
  if (source == TubeSource::ESDF) {
    input.authority_request.lower = -3.0;
    input.authority_request.upper = 3.0;
    input.authority_request.valid = true;
  }
  return input;
}

PathStateQuery ExactPathQuery() {
  return [](const double w, phase_offset_core::PathDifferentialState& state) {
    state.p = Eigen::Vector3d(w, 0.0, 1.0 + 0.1 * w);
    state.p_w = Eigen::Vector3d(1.0, 0.0, 0.1);
    state.p_ww = Eigen::Vector3d::Zero();
    state.T = state.p_w.normalized();
    state.N = Eigen::Vector3d::UnitY();
    state.N_w = Eigen::Vector3d::Zero();
    state.path_revision = 1U;
    state.frame_revision = 1U;
    state.frame_valid = true;
    state.frame_provenance =
        "ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct";
    state.w = w;
    state.valid = std::isfinite(w);
    return state.valid;
  };
}

ClearanceQuery CloudOpenQuery() {
  return [](const Eigen::Vector3d&, const double required) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

ClearanceQuery CloudCorridorQuery(const double positive_wall,
                                  const double negative_wall) {
  return [positive_wall, negative_wall](const Eigen::Vector3d& point,
                                        const double) {
    ClearanceQueryResult result;
    const double clearance = std::min(positive_wall - point.y(),
                                      point.y() + negative_wall);
    if (clearance <= 0.0) {
      result.status = DistanceStatus::OCCUPIED;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = clearance;
    result.clearance_certified = true;
    return result;
  };
}

PathCellBoundQuery MatchingCloudCellCertificate() {
  return [](const double w0, const double w1,
            phase_offset_core::PathCellGeometryCertificate& certificate) {
    certificate = phase_offset_core::PathCellGeometryCertificate();
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.path_revision = 1U;
    certificate.frame_revision = 1U;
    certificate.segment_identity = 1U;
    certificate.segment_w0 = 0.0;
    certificate.segment_w1 = 1.0;
    certificate.inf_p_w_norm = 1.0;
    certificate.inf_horizontal_p_w_norm = 1.0;
    certificate.sup_p_w_norm = 1.01;
    certificate.sup_p_ww_norm = 0.0;
    certificate.sup_p_www_norm = 0.0;
    certificate.sup_horizontal_p_ww_norm = 0.0;
    certificate.horizontal_acceleration_bound_complete = true;
    certificate.sup_N_w_norm = 0.0;
    certificate.sup_abs_curvature = 0.0;
    certificate.normal_variation_bound = 0.0;
    certificate.tangent_variation_bound = 0.0;
    certificate.curvature_variation_bound = 0.0;
    certificate.midpoint_position_variation_bound = 0.0;
    certificate.chord_deviation_bound = 0.0;
    certificate.normal_frame_proof_complete = true;
    certificate.combined_regularity_proof_complete = true;
    certificate.regularity_speed_min = 1.0;
    certificate.regularity_speed_max = 1.01;
    certificate.provenance =
        "ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct";
    certificate.valid = std::isfinite(w0) && std::isfinite(w1) && w1 > w0;
    certificate.complete = certificate.valid;
    return certificate.valid;
  };
}

PathCellBoundQuery InvalidOffsetCertificate() {
  // Builder consumes one complete certificate per active interval.  Return
  // those valid premises on the first pass, then make each Validator witness
  // explicitly malformed on its second hit so the owner cannot rescue it with
  // a fixed inset.  Keying by the actual interval keeps this deterministic
  // even when Validator scheduling visits a different cell first.
  return [valid = MatchingCloudCellCertificate(),
          hits = std::map<std::pair<double, double>, std::size_t>()](
      const double w0, const double w1,
      phase_offset_core::PathCellGeometryCertificate& certificate) mutable {
    const std::pair<double, double> key(w0, w1);
    const std::size_t hit = hits[key]++;
    if (hit >= 1U) {
      certificate = phase_offset_core::PathCellGeometryCertificate();
      certificate.w0 = w0;
      certificate.w1 = w1;
      certificate.valid = false;
      certificate.complete = false;
      return true;
    }
    return valid(w0, w1, certificate);
  };
}

TubeEpochUpdateInput MakeCloudInput(const ClearanceQuery& clearance = CloudOpenQuery(),
                                    double end_w = 0.6) {
  TubeEpochUpdateInput input = MakeInput(TubeSource::ESDF, end_w);
  input.cloud_clearance_query = clearance;
  input.path_state_query = ExactPathQuery();
  input.path_cell_bound_query = MatchingCloudCellCertificate();
  input.cloud_snapshot_resolution = 0.05;
  input.map_observation_is_snapshot = true;
  return input;
}

TubeProfile MakeProfile(double terminal_upper = 0.10) {
  TubeProfile profile;
  profile.source = TubeSource::FIXED;
  profile.source_revision = 1U;
  profile.preview_start_w = 0.0;
  profile.preview_end_w = 0.3;
  profile.raw_complete = true;
  profile.filtered_complete = true;
  profile.complete = true;
  for (int index = 0; index < 4; ++index) {
    TubeRawSample sample;
    sample.w = 0.1 * static_cast<double>(index);
    sample.p = Eigen::Vector3d(sample.w, 0.0, 1.0);
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.raw_lower = -0.10;
    sample.raw_upper = index == 3 ? terminal_upper : 0.10;
    sample.filtered_lower = sample.raw_lower;
    sample.filtered_upper = sample.raw_upper;
    sample.fixed_lower = sample.raw_lower;
    sample.fixed_upper = sample.raw_upper;
    sample.regularity_lower = sample.raw_lower;
    sample.regularity_upper = sample.raw_upper;
    sample.esdf_lower = sample.raw_lower;
    sample.esdf_upper = sample.raw_upper;
    sample.complete = true;
    sample.positive_certified = true;
    sample.negative_certified = true;
    sample.regularity_intersection = true;
    profile.samples.push_back(sample);
  }
  return profile;
}

bool StatusNumbersAreFinite(const TubeEpochStatus& status) {
  return std::isfinite(status.retained_delta) &&
      std::isfinite(status.tracking_error_norm) &&
      std::isfinite(status.tracking_error_bound) &&
      std::isfinite(status.reference_signed_distance) &&
      std::isfinite(status.actual_signed_distance) &&
      std::isfinite(status.required_reference_clearance) &&
      std::isfinite(status.required_actual_clearance) &&
      std::isfinite(status.certified_forward_w);
}

TEST(TubeEpochManagerTest, InvalidConfigurationReportsConfigurationError) {
  TubeEpochManagerConfig config = MakeConfig();
  config.builder.ray_step = 0.0;
  TubeEpochManager manager(config);
  TubeEpochUpdateResult result;
  EXPECT_FALSE(manager.configurationValid());
  EXPECT_FALSE(manager.update(MakeInput(), result));
  EXPECT_EQ(result.status.state, TubeEpochState::CONFIGURATION_ERROR);
  EXPECT_EQ(result.status.reason, TubeEpochReason::INVALID_CONFIGURATION);
}

TEST(TubeEpochManagerTest, FirstFixedCandidateInstallsEpochOne) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(MakeInput(), result));
  EXPECT_EQ(result.status.active_tube_epoch, 1U);
  EXPECT_EQ(result.status.disposition, TubeInstallDisposition::INITIAL_INSTALL);
  EXPECT_EQ(result.status.state, TubeEpochState::ROLLING);
  EXPECT_FALSE(result.active_profile.obstacle_certified);
}

TEST(TubeEpochManagerTest, FirstEsdfCandidateInstallsEpochOne) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(MakeCloudInput(), result));
  EXPECT_EQ(result.status.active_tube_epoch, 1U);
  EXPECT_TRUE(result.active_profile.obstacle_certified);
  EXPECT_EQ(result.status.current_safety_status, CurrentSafetyStatus::SAFE);
}

TEST(TubeEpochManagerTest,
     EsdfAuthorityRequestIsCompatibilityMetadataOnly) {
  TubeEpochUpdateInput input = MakeCloudInput();
  input.authority_request.lower = -0.10;
  input.authority_request.upper = 0.10;
  input.authority_request.valid = true;
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(input, result));
  ASSERT_TRUE(result.status.active_available);
  ASSERT_EQ(result.active_profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  ASSERT_EQ(result.active_profile.samples.size(),
            result.active_profile.raw_build_samples.size());
  bool broad_raw = false;
  for (std::size_t index = 0U; index < result.active_profile.samples.size();
       ++index) {
    const TubeRawSample& certified = result.active_profile.samples[index];
    const TubeRawSample& raw = result.active_profile.raw_build_samples[index];
    EXPECT_DOUBLE_EQ(certified.filtered_lower, raw.filtered_lower);
    EXPECT_DOUBLE_EQ(certified.filtered_upper, raw.filtered_upper);
    broad_raw = broad_raw || raw.filtered_lower < -1.0 - 1e-12 ||
        raw.filtered_upper > 1.0 + 1e-12;
  }
  EXPECT_FALSE(broad_raw);
}

TEST(TubeEpochManagerTest, EsdfMissingAuthorityRequestDoesNotVetoGeometry) {
  TubeEpochUpdateInput input = MakeCloudInput();
  input.authority_request = TubeBounds();
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  EXPECT_TRUE(manager.update(input, result));
  EXPECT_TRUE(result.status.candidate_raw_complete);
  EXPECT_TRUE(result.status.candidate_filtered_complete);
  EXPECT_TRUE(result.status.candidate_complete);
}

TEST(TubeEpochManagerTest,
     CandidateStateAndZeroOnlyInstallationComeFromCertifiedBuilderResult) {
  const TubeEpochManagerConfig config = MakeConfig();
  const TubeEpochUpdateInput input = MakeCloudInput(CloudCorridorQuery(0.10, 2.30));
  CertifiedTubeBuildInput build_input;
  build_input.source = input.source;
  build_input.preview_path = input.preview_path;
  build_input.authority_request = input.authority_request;
  build_input.cloud_clearance_query = input.cloud_clearance_query;
  build_input.path_state_query = input.path_state_query;
  build_input.path_cell_bound_query = input.path_cell_bound_query;
  build_input.cloud_snapshot_resolution = input.cloud_snapshot_resolution;
  build_input.current_w = input.current_path.w;
  build_input.path_source_revision = input.path_source_revision;
  build_input.tube_revision = 1U;
  build_input.map_observation_sequence = input.map_observation_sequence;
  build_input.map_observation_is_snapshot = input.map_observation_is_snapshot;
  CertifiedTubeBuildResult expected;
  ASSERT_TRUE(CertifiedTubeBuilder(config.builder, config.filter,
                                   config.surface_validator)
                  .build(build_input, expected));
  ASSERT_EQ(expected.profile.classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);

  TubeEpochManager manager(config);
  TubeEpochUpdateResult actual;
  ASSERT_TRUE(manager.update(input, actual));
  EXPECT_TRUE(TubeEpochManager::profilesEquivalent(expected.profile,
                                                    actual.candidate_profile,
                                                    1e-12));
  EXPECT_EQ(actual.status.candidate_raw_complete, expected.raw_complete);
  EXPECT_EQ(actual.status.candidate_filtered_complete, expected.filtered_complete);
  EXPECT_EQ(actual.status.candidate_complete, expected.complete);
  EXPECT_EQ(actual.status.candidate_classification,
            expected.profile.classification);
  EXPECT_EQ(actual.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
}

TEST(TubeEpochManagerTest,
     InvalidCertifiedOffsetKeepsZeroBuilderInsetAndCandidatePath) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput();
  input.path_cell_bound_query = InvalidOffsetCertificate();
  TubeEpochUpdateResult result;
  EXPECT_FALSE(manager.update(input, result));
  EXPECT_FALSE(result.status.candidate_complete);
  EXPECT_FALSE(result.status.candidate_filtered_complete);
  EXPECT_EQ(result.status.candidate_classification,
            TubeProfileClassification::NONE);
  EXPECT_FALSE(result.status.active_available);
  EXPECT_EQ(result.status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  EXPECT_EQ(result.status.reason, TubeEpochReason::CANDIDATE_INCOMPLETE);
  EXPECT_FALSE(result.candidate_profile.complete);
  EXPECT_FALSE(result.candidate_profile.obstacle_certified);
  EXPECT_EQ(result.candidate_profile.classification,
            TubeProfileClassification::NONE);
  EXPECT_TRUE(result.candidate_profile.cell_geometry_certified);
  EXPECT_EQ(result.candidate_profile.diagnostics.surface_outcome,
            TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.candidate_profile.diagnostics.surface_inconclusive_reason,
            TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MALFORMED);
  EXPECT_EQ(result.candidate_profile.diagnostics.inward_search_attempt_count,
            0U);
  ASSERT_FALSE(result.candidate_profile.samples.empty());
  EXPECT_DOUBLE_EQ(result.candidate_profile.samples.front().continuous_inset,
                   0.0);
}

TEST(TubeEpochManagerTest, NonzeroSurfaceProofLimitFailsClosedWithoutRetry) {
  TubeEpochManagerConfig config = MakeConfig();
  // The anchor consumes the only allowed centre query; all nondegenerate
  // pending cells therefore terminate as typed QUERY_BUDGET evidence.
  config.surface_validator.max_query_samples = 1U;
  TubeEpochManager manager(config);
  TubeEpochUpdateResult result;
  EXPECT_FALSE(manager.update(MakeCloudInput(), result));
  EXPECT_FALSE(result.status.candidate_complete);
  EXPECT_EQ(result.status.candidate_classification,
            TubeProfileClassification::NONE);
  EXPECT_FALSE(result.status.active_available);
  EXPECT_EQ(result.status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  EXPECT_EQ(result.candidate_profile.classification,
            TubeProfileClassification::NONE);
  EXPECT_FALSE(result.candidate_profile.complete);
  EXPECT_FALSE(result.candidate_profile.obstacle_certified);
  EXPECT_EQ(result.candidate_profile.diagnostics.surface_outcome,
            TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.candidate_profile.diagnostics.surface_inconclusive_reason,
            TubeSurfaceInconclusiveReason::QUERY_BUDGET);
  EXPECT_TRUE(result.candidate_profile.diagnostics.surface_query_budget_reached);
  EXPECT_EQ(result.candidate_profile.diagnostics.inward_search_attempt_count,
            0U);
  ASSERT_EQ(result.candidate_profile.samples.size(),
            result.candidate_profile.raw_build_samples.size());
  for (std::size_t index = 0U;
       index < result.candidate_profile.samples.size(); ++index) {
    const TubeRawSample& sample = result.candidate_profile.samples[index];
    const TubeRawSample& raw = result.candidate_profile.raw_build_samples[index];
    EXPECT_DOUBLE_EQ(sample.filtered_lower, raw.filtered_lower);
    EXPECT_DOUBLE_EQ(sample.filtered_upper, raw.filtered_upper);
    EXPECT_GE(sample.filtered_lower, -1.0 - 1e-12);
    EXPECT_LE(sample.filtered_upper, 1.0 + 1e-12);
  }
}

TEST(TubeEpochManagerTest, CandidateAndActiveProfilesAreIndependentObjects) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(MakeInput(), result));
  const double active_upper = result.active_profile.samples.front().filtered_upper;
  result.candidate_profile.samples.front().filtered_upper = -7.0;
  EXPECT_DOUBLE_EQ(result.active_profile.samples.front().filtered_upper, active_upper);
}

TEST(TubeEpochManagerTest,
     StackLocalPreparedBuildDoesNotMutateLiveCandidateActiveOrCounters) {
  const TubeEpochManagerConfig config = MakeConfig();
  TubeEpochManager live(config);
  TubeEpochUpdateResult installed;
  ASSERT_TRUE(live.update(MakeInput(), installed));
  const std::uint64_t live_epoch = installed.status.active_tube_epoch;
  const std::uint64_t live_candidate_sequence = installed.status.candidate_sequence;
  const std::size_t live_sample_count = installed.active_profile.samples.size();

  TubeEpochUpdateInput staged_input = MakeInput();
  staged_input.path_source_revision = 7U;
  staged_input.retained_delta = 0.20;
  TubeEpochManager staged(config);
  TubeEpochUpdateResult staged_result;
  EXPECT_FALSE(staged.update(staged_input, staged_result));
  EXPECT_EQ(staged_result.status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  EXPECT_EQ(staged_result.status.reason, TubeEpochReason::CURRENT_OFFSET_OUTSIDE);

  // Rebuild through a separate stack object: live ownership/counters remain
  // exactly where they were before the failed staged candidate.
  TubeEpochUpdateResult live_after;
  ASSERT_TRUE(live.update(MakeInput(), live_after));
  EXPECT_EQ(live_after.status.active_tube_epoch, live_epoch);
  EXPECT_EQ(live_after.status.candidate_sequence, live_candidate_sequence + 1U);
  EXPECT_EQ(live_after.active_profile.samples.size(), live_sample_count);
  EXPECT_EQ(live_after.status.active_path_source_revision, 1U);
}

TEST(TubeEpochManagerTest, CandidateSequenceMonotonicallyIncreases) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult first;
  TubeEpochUpdateResult second;
  ASSERT_TRUE(manager.update(MakeInput(), first));
  ASSERT_TRUE(manager.update(MakeInput(), second));
  EXPECT_EQ(first.status.candidate_sequence, 1U);
  EXPECT_EQ(second.status.candidate_sequence, 2U);
}

TEST(TubeEpochManagerTest, ActiveEpochNeverMovesBackward) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult first;
  ASSERT_TRUE(manager.update(MakeInput(), first));
  TubeEpochUpdateResult none;
  ASSERT_TRUE(manager.update(MakeInput(TubeSource::NONE), none));
  TubeEpochUpdateResult second;
  ASSERT_TRUE(manager.update(MakeInput(), second));
  EXPECT_EQ(none.status.active_tube_epoch, 1U);
  EXPECT_EQ(second.status.active_tube_epoch, 2U);
}

TEST(TubeEpochManagerTest, EquivalentRebuildDoesNotAdvanceActiveEpoch) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult first;
  TubeEpochUpdateResult second;
  ASSERT_TRUE(manager.update(MakeInput(), first));
  ASSERT_TRUE(manager.update(MakeInput(), second));
  EXPECT_EQ(second.status.active_tube_epoch, first.status.active_tube_epoch);
  EXPECT_EQ(second.status.disposition, TubeInstallDisposition::EQUIVALENT_REFRESH);
}

TEST(TubeEpochManagerTest, MapObservationOnlyRefreshDoesNotAdvanceActiveEpoch) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult first;
  ASSERT_TRUE(manager.update(MakeInput(), first));
  TubeEpochUpdateInput input = MakeInput();
  input.map_observation_sequence = 99U;
  TubeEpochUpdateResult second;
  ASSERT_TRUE(manager.update(input, second));
  EXPECT_EQ(second.status.active_tube_epoch, first.status.active_tube_epoch);
  EXPECT_EQ(second.status.active_map_observation_sequence, 99U);
}

TEST(TubeEpochManagerTest, MaterialBoundChangeIsRejectedWithoutInwardRetry) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult first;
  TubeEpochUpdateInput input = MakeCloudInput();
  ASSERT_TRUE(manager.update(input, first));
  // Leave enough residual clearance for the Validator's Contract-A cover;
  // the resulting asymmetric raw interval is still materially different from
  // the open-space nominal ribbon.
  input.cloud_clearance_query = CloudCorridorQuery(1.00, 2.50);
  TubeEpochUpdateResult second;
  EXPECT_FALSE(manager.update(input, second));
  EXPECT_EQ(second.status.active_tube_epoch, first.status.active_tube_epoch);
  EXPECT_EQ(second.status.disposition,
            TubeInstallDisposition::REJECTED_CANDIDATE);
  EXPECT_TRUE(second.status.active_available);
}

TEST(TubeEpochManagerTest, ChangedPathProvenanceAdvancesEpochEvenWithSameBounds) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult first;
  ASSERT_TRUE(manager.update(MakeInput(), first));
  TubeEpochUpdateInput input = MakeInput();
  input.path_source_revision = 2U;
  TubeEpochUpdateResult second;
  ASSERT_TRUE(manager.update(input, second));
  EXPECT_EQ(second.status.active_tube_epoch, first.status.active_tube_epoch + 1U);
  EXPECT_EQ(second.status.active_path_source_revision, 2U);
}

TEST(TubeEpochManagerTest, BuildCountersDoNotDefineProfileIdentity) {
  TubeProfile first = MakeProfile();
  TubeProfile second = first;
  second.tube_revision = 42U;
  second.diagnostics.invalid_count = 9U;
  EXPECT_TRUE(TubeEpochManager::profilesEquivalent(first, second, 1e-10));
}

TEST(TubeEpochManagerTest, EquivalenceToleranceIsDeterministicAndRejectsNonFinite) {
  TubeProfile first = MakeProfile();
  TubeProfile second = first;
  second.samples.front().filtered_upper += 5e-11;
  EXPECT_TRUE(TubeEpochManager::profilesEquivalent(first, second, 1e-10));
  EXPECT_FALSE(TubeEpochManager::profilesEquivalent(first, second, 1e-12));
  second.samples.front().filtered_upper = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(TubeEpochManager::profilesEquivalent(first, second, 1e-10));
}

TEST(TubeEpochManagerTest, UnavailableSnapshotProducesNeutralZeroOnlyProfile) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult first;
  ASSERT_TRUE(manager.update(MakeCloudInput(), first));
  TubeEpochUpdateInput input = MakeCloudInput(
      [](const Eigen::Vector3d&, double) {
        ClearanceQueryResult result;
        result.status = DistanceStatus::UNAVAILABLE;
        return result;
      });
  TubeEpochUpdateResult rejected;
  EXPECT_TRUE(manager.update(input, rejected));
  EXPECT_EQ(rejected.status.candidate_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_EQ(rejected.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_GT(rejected.status.active_tube_epoch, first.status.active_tube_epoch);
  ASSERT_FALSE(rejected.active_profile.samples.empty());
  EXPECT_DOUBLE_EQ(rejected.active_profile.samples.front().filtered_lower, 0.0);
  EXPECT_DOUBLE_EQ(rejected.active_profile.samples.front().filtered_upper, 0.0);
}

TEST(TubeEpochManagerTest, LatestUnavailableCandidateIsObservableZeroOnly) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput(
      [](const Eigen::Vector3d&, double) {
        ClearanceQueryResult result;
        result.status = DistanceStatus::UNAVAILABLE;
        return result;
      });
  TubeEpochUpdateResult result;
  EXPECT_TRUE(manager.update(input, result));
  EXPECT_TRUE(result.status.candidate_complete);
  EXPECT_EQ(result.status.candidate_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_EQ(result.candidate_profile.samples.front().cross_section_reason,
            TubeCrossSectionReason::CENTER_UNAVAILABLE);
}

TEST(TubeEpochManagerTest, RejectedCandidateDoesNotPartiallyOverwriteActiveMetadata) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult installed;
  ASSERT_TRUE(manager.update(MakeInput(), installed));
  TubeEpochUpdateInput input = MakeInput();
  input.path_source_revision = 2U;
  input.map_observation_sequence = 2U;
  input.current_path.p_w.setZero();
  TubeEpochUpdateResult rejected;
  EXPECT_FALSE(manager.update(input, rejected));
  EXPECT_EQ(rejected.status.active_path_source_revision, 1U);
  EXPECT_EQ(rejected.status.active_map_observation_sequence, 1U);
  EXPECT_EQ(rejected.status.active_tube_epoch, installed.status.active_tube_epoch);
}

TEST(TubeEpochManagerTest, CandidateCopyCannotMutateActiveProfile) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(MakeInput(), result));
  result.candidate_profile.samples.back().filtered_lower = 8.0;
  EXPECT_NE(result.candidate_profile.samples.back().filtered_lower,
            result.active_profile.samples.back().filtered_lower);
}

TEST(TubeEpochManagerTest, SourceNoneMakesActiveUnavailableWithoutReusingEpoch) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult installed;
  ASSERT_TRUE(manager.update(MakeInput(), installed));
  TubeEpochUpdateResult none;
  ASSERT_TRUE(manager.update(MakeInput(TubeSource::NONE), none));
  EXPECT_FALSE(none.status.active_available);
  EXPECT_EQ(none.status.active_tube_epoch, installed.status.active_tube_epoch);
  EXPECT_TRUE(none.active_profile.samples.empty());
}

TEST(TubeEpochManagerTest, CurrentGeometryAndRetainedDeltaInsideAllowInstallation) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeInput();
  input.retained_delta = 0.05;
  input.actual_position += Eigen::Vector3d(0.0, 0.05, 0.0);
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(input, result));
  EXPECT_TRUE(result.status.current_geometry_valid);
  EXPECT_TRUE(result.status.retained_delta_current_inside);
  EXPECT_TRUE(result.status.current_state_admissible);
}

TEST(TubeEpochManagerTest, CurrentRetainedDeltaOutsideDoesNotInstallOrLatch) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeInput();
  input.retained_delta = 0.20;
  TubeEpochUpdateResult result;
  EXPECT_FALSE(manager.update(input, result));
  EXPECT_TRUE(result.status.candidate_complete);
  EXPECT_TRUE(result.candidate_profile.complete);
  EXPECT_EQ(result.status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  EXPECT_EQ(result.status.reason, TubeEpochReason::CURRENT_OFFSET_OUTSIDE);
  EXPECT_FALSE(result.status.certificate_denied);
  EXPECT_DOUBLE_EQ(result.status.retained_delta, 0.20);
  EXPECT_FALSE(result.status.retained_delta_current_inside);
}

TEST(TubeEpochManagerTest,
     EsdfRetainedDeltaOutsideNominalUsesBoundedConstructionAndKeepsMetadata) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput();
  input.retained_delta = 1.2;
  TubeEpochUpdateResult result;
  EXPECT_FALSE(manager.update(input, result));
  EXPECT_TRUE(result.status.candidate_complete);
  EXPECT_EQ(result.status.reason, TubeEpochReason::CURRENT_OFFSET_OUTSIDE);
  EXPECT_DOUBLE_EQ(result.status.retained_delta, 1.2);
  EXPECT_FALSE(result.status.retained_delta_current_inside);
  ASSERT_FALSE(result.candidate_profile.samples.empty());
  EXPECT_DOUBLE_EQ(result.candidate_profile.current_delta, 1.2);
  EXPECT_LE(result.candidate_profile.diagnostics.max_bounded_construction_abs_delta,
            MakeConfig().builder.cross_section.nominal_half_width + 1e-12);
  EXPECT_EQ(result.candidate_profile.selected_component,
            TubeComponentSelection::ZERO_CONNECTED);
  EXPECT_FALSE(result.candidate_profile.current_component_contains_delta);
}

TEST(TubeEpochManagerTest,
     ZeroOnlyCandidateDoesNotTreatTinyRetainedDeltaAsNeutral) {
  TubeEpochManagerConfig config = MakeConfig();
  config.builder.fixed_delta_max = 0.0;
  config.builder.max_offset = 0.0;
  TubeEpochManager manager(config);
  const double tiny_values[] = {
      1e-13, -1e-13, std::numeric_limits<double>::denorm_min(),
      -std::numeric_limits<double>::denorm_min()};
  for (const double delta : tiny_values) {
    TubeEpochUpdateInput input = MakeInput();
    input.retained_delta = delta;
    TubeEpochUpdateResult result;
    EXPECT_FALSE(manager.update(input, result));
    EXPECT_TRUE(result.status.candidate_complete);
    EXPECT_EQ(result.status.candidate_classification,
              TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
    EXPECT_FALSE(result.status.current_state_admissible);
    EXPECT_EQ(result.status.reason, TubeEpochReason::CURRENT_OFFSET_OUTSIDE);
    EXPECT_DOUBLE_EQ(result.status.retained_delta, delta);
  }
}

TEST(TubeEpochManagerTest, Full3DCurrentRegularityUsesAnalyticHorizontalNormal) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeInput();
  input.retained_delta = -0.10;
  input.current_path.p_ww = Eigen::Vector3d(0.0, -100.0, 0.0);
  TubeEpochUpdateResult result;
  EXPECT_TRUE(manager.update(input, result));
  EXPECT_TRUE(result.status.current_geometry_valid);
}

TEST(TubeEpochManagerTest, NeutralExplicitCloudUnsafeProducesZeroOnlyObservation) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput(
      [](const Eigen::Vector3d&, double) {
        ClearanceQueryResult result;
        result.status = DistanceStatus::OCCUPIED;
        return result;
      });
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(input, result));
  EXPECT_EQ(result.status.state, TubeEpochState::ROLLING);
  EXPECT_FALSE(result.status.certificate_denied);
  EXPECT_EQ(result.status.current_safety_status, CurrentSafetyStatus::UNSAFE);
  EXPECT_EQ(result.status.reason, TubeEpochReason::NONE);
  EXPECT_EQ(result.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
}

TEST(TubeEpochManagerTest, NeutralUnknownCloudEvidenceProducesZeroOnlyObservation) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput(
      [](const Eigen::Vector3d&, double) {
        ClearanceQueryResult result;
        result.status = DistanceStatus::UNKNOWN;
        return result;
      });
  TubeEpochUpdateResult result;
  EXPECT_TRUE(manager.update(input, result));
  EXPECT_EQ(result.status.state, TubeEpochState::ROLLING);
  EXPECT_FALSE(result.status.certificate_denied);
  EXPECT_EQ(result.status.current_safety_status, CurrentSafetyStatus::INDETERMINATE);
  EXPECT_EQ(result.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
}

TEST(TubeEpochManagerTest, TrackingErrorOverBoundInstallsWithoutChangingRuntimeMode) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeInput();
  input.actual_position.y() = 0.20;
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(input, result));
  EXPECT_FALSE(result.status.tracking_within_bound);
  EXPECT_TRUE(result.status.active_available);
  EXPECT_EQ(result.status.state, TubeEpochState::ROLLING);
  EXPECT_FALSE(result.status.certificate_denied);
  EXPECT_EQ(result.status.reason, TubeEpochReason::NONE);
}

TEST(TubeEpochManagerTest, FixedSourceDoesNotRequireCloudClearance) {
  TubeEpochUpdateInput input = MakeInput();
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(input, result));
  EXPECT_FALSE(result.active_profile.obstacle_certified);
  EXPECT_EQ(result.status.current_safety_status, CurrentSafetyStatus::NOT_EVALUATED);
}

TEST(TubeEpochManagerTest, SufficientHorizonAndContainedOffsetRoll) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(MakeInput(), result));
  EXPECT_TRUE(result.status.forward_horizon_sufficient);
  EXPECT_EQ(result.status.state, TubeEpochState::ROLLING);
}

TEST(TubeEpochManagerTest, ShortForwardHorizonWaitsWithoutInstalling) {
  TubeEpochManagerConfig config = MakeConfig();
  config.builder.min_certified_forward_w = 0.80;
  TubeEpochManager manager(config);
  TubeEpochUpdateResult result;
  EXPECT_FALSE(manager.update(MakeInput(), result));
  EXPECT_EQ(result.status.active_tube_epoch, 0U);
  EXPECT_EQ(result.status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  EXPECT_FALSE(result.status.forward_horizon_sufficient);
}

TEST(TubeEpochManagerTest, TransientUnavailableRecoversWithoutPermanentFailure) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput();
  TubeEpochUpdateResult first;
  ASSERT_TRUE(manager.update(input, first));
  input.cloud_clearance_query = [](const Eigen::Vector3d&, double) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::UNAVAILABLE;
    return result;
  };
  TubeEpochUpdateResult waiting;
  EXPECT_TRUE(manager.update(input, waiting));
  EXPECT_EQ(waiting.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  input.cloud_clearance_query = CloudOpenQuery();
  TubeEpochUpdateResult recovered;
  ASSERT_TRUE(manager.update(input, recovered));
  EXPECT_EQ(recovered.status.state, TubeEpochState::ROLLING);
  EXPECT_GT(recovered.status.active_tube_epoch, first.status.active_tube_epoch);
}

TEST(TubeEpochManagerTest, IncompleteThenEquivalentCandidateRecoversWithoutEpochNoise) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput();
  TubeEpochUpdateResult first;
  ASSERT_TRUE(manager.update(input, first));
  input.cloud_clearance_query = [](const Eigen::Vector3d&, double) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::UNKNOWN;
    return result;
  };
  TubeEpochUpdateResult waiting;
  EXPECT_TRUE(manager.update(input, waiting));
  EXPECT_EQ(waiting.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  input.cloud_clearance_query = CloudOpenQuery();
  TubeEpochUpdateResult recovered;
  ASSERT_TRUE(manager.update(input, recovered));
  EXPECT_EQ(recovered.status.candidate_sequence, 3U);
  EXPECT_GT(recovered.status.active_tube_epoch, first.status.active_tube_epoch);
}

TEST(TubeEpochManagerTest, RetainedOffsetRejectionCanRecoverToRolling) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeInput();
  TubeEpochUpdateResult rolling;
  ASSERT_TRUE(manager.update(input, rolling));
  input.retained_delta = 0.20;
  TubeEpochUpdateResult unsafe;
  EXPECT_FALSE(manager.update(input, unsafe));
  input.retained_delta = 0.0;
  TubeEpochUpdateResult recovered;
  ASSERT_TRUE(manager.update(input, recovered));
  EXPECT_EQ(unsafe.status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  EXPECT_TRUE(unsafe.status.candidate_complete);
  EXPECT_EQ(unsafe.status.reason, TubeEpochReason::CURRENT_OFFSET_OUTSIDE);
  EXPECT_EQ(recovered.status.state, TubeEpochState::ROLLING);
  EXPECT_EQ(unsafe.status.active_tube_epoch, rolling.status.active_tube_epoch);
  EXPECT_EQ(recovered.status.active_tube_epoch, rolling.status.active_tube_epoch);
}

TEST(TubeEpochManagerTest, IncompleteEvidenceRetainsActiveCurrentValidation) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult active;
  ASSERT_TRUE(manager.update(MakeInput(), active));
  TubeEpochUpdateInput input = MakeCloudInput(
      [](const Eigen::Vector3d&, double) {
        ClearanceQueryResult result;
        result.status = DistanceStatus::UNAVAILABLE;
        return result;
      });
  TubeEpochUpdateResult waiting;
  EXPECT_TRUE(manager.update(input, waiting));
  EXPECT_TRUE(waiting.status.active_available);
  EXPECT_TRUE(waiting.status.active_current_validation_valid);
  EXPECT_EQ(waiting.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
}

TEST(TubeEpochManagerTest, RetainedOffsetRejectionDoesNotResetRetainedDelta) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeInput();
  input.retained_delta = -0.20;
  TubeEpochUpdateResult result;
  EXPECT_FALSE(manager.update(input, result));
  EXPECT_TRUE(result.status.candidate_complete);
  EXPECT_EQ(result.status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  EXPECT_EQ(result.status.reason, TubeEpochReason::CURRENT_OFFSET_OUTSIDE);
  EXPECT_DOUBLE_EQ(result.status.retained_delta, -0.20);
}

TEST(TubeEpochManagerTest, NeutralUnavailableBuildInstallsNewZeroOnlyProfile) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult active;
  ASSERT_TRUE(manager.update(MakeInput(), active));
  TubeEpochUpdateInput input = MakeCloudInput(
      [](const Eigen::Vector3d&, double) {
        ClearanceQueryResult result;
        result.status = DistanceStatus::UNAVAILABLE;
        return result;
      });
  input.path_source_revision = 8U;
  TubeEpochUpdateResult failed;
  EXPECT_TRUE(manager.update(input, failed));
  EXPECT_EQ(failed.status.active_path_source_revision, 8U);
  EXPECT_GT(failed.status.active_tube_epoch, active.status.active_tube_epoch);
  EXPECT_EQ(failed.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
}

TEST(TubeEpochManagerTest, FailedCurrentCheckDoesNotHalfUpdateProvenance) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult active;
  ASSERT_TRUE(manager.update(MakeInput(), active));
  TubeEpochUpdateInput input = MakeInput();
  input.current_path.p_w.setZero();
  input.path_source_revision = 7U;
  input.map_observation_sequence = 7U;
  TubeEpochUpdateResult failed;
  EXPECT_FALSE(manager.update(input, failed));
  EXPECT_EQ(failed.status.active_path_source_revision, 1U);
  EXPECT_EQ(failed.status.active_map_observation_sequence, 1U);
}

TEST(TubeEpochManagerTest, MaterialInstallDoesNotCommitAfterValidatorFailure) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput();
  TubeEpochUpdateResult first;
  ASSERT_TRUE(manager.update(input, first));
  input.cloud_clearance_query = CloudCorridorQuery(1.00, 2.50);
  TubeEpochUpdateResult material;
  EXPECT_FALSE(manager.update(input, material));
  TubeEpochUpdateResult refresh;
  EXPECT_FALSE(manager.update(input, refresh));
  EXPECT_EQ(material.status.active_tube_epoch, first.status.active_tube_epoch);
  EXPECT_EQ(refresh.status.active_tube_epoch, first.status.active_tube_epoch);
  EXPECT_EQ(refresh.status.install_count, first.status.install_count);
}

TEST(TubeEpochManagerTest, RepeatedSequenceIsDeterministic) {
  TubeEpochManager first(MakeConfig());
  TubeEpochManager second(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput();
  TubeEpochUpdateResult first_result;
  TubeEpochUpdateResult second_result;
  ASSERT_TRUE(first.update(input, first_result));
  ASSERT_TRUE(second.update(input, second_result));
  EXPECT_EQ(first_result.status.state, second_result.status.state);
  EXPECT_EQ(first_result.status.active_tube_epoch, second_result.status.active_tube_epoch);
  EXPECT_EQ(first_result.active_profile.samples.size(),
            second_result.active_profile.samples.size());
  EXPECT_DOUBLE_EQ(first_result.active_profile.samples.front().filtered_upper,
                   second_result.active_profile.samples.front().filtered_upper);
}

TEST(TubeEpochManagerTest, PublicStatusNumbersRemainFinite) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult active;
  ASSERT_TRUE(manager.update(MakeCloudInput(), active));
  EXPECT_TRUE(StatusNumbersAreFinite(active.status));
  TubeEpochUpdateInput input = MakeCloudInput(
      [](const Eigen::Vector3d&, double) {
        ClearanceQueryResult result;
        result.status = DistanceStatus::UNKNOWN;
        return result;
      });
  TubeEpochUpdateResult waiting;
  EXPECT_TRUE(manager.update(input, waiting));
  EXPECT_TRUE(StatusNumbersAreFinite(waiting.status));
}

TEST(TubeEpochManagerTest, CloudClearancePathHasNoLegacyQueryInput) {
  TubeEpochUpdateInput input = MakeCloudInput();
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(input, result));
  EXPECT_TRUE(result.status.raw_cross_section_path_used);
  EXPECT_EQ(result.status.state, TubeEpochState::ROLLING);
  EXPECT_TRUE(result.status.reference_clearance_sufficient);
  EXPECT_TRUE(result.status.actual_clearance_sufficient);
  EXPECT_TRUE(result.active_profile.snapshot_provenance_is_immutable);
  EXPECT_EQ(result.active_profile.snapshot_sequence,
            result.status.active_map_observation_sequence);
  EXPECT_NEAR(result.active_profile.snapshot_resolution, 0.05, 1e-12);
  EXPECT_NEAR(result.status.residual_effective_radius, 0.40, 1e-12);
  ASSERT_FALSE(result.active_profile.samples.empty());
  EXPECT_NEAR(result.active_profile.samples.front().raw_lower, -1.0, 1e-12);
  EXPECT_NEAR(result.active_profile.samples.front().raw_upper, 1.0, 1e-12);
}

TEST(TubeEpochManagerTest,
     ActiveRetainedOffsetUsesSelectedCurrentComponent) {
  TubeEpochUpdateInput input = MakeCloudInput(CloudCorridorQuery(0.10, 2.30));
  input.retained_delta = -1.0;
  input.actual_position += Eigen::Vector3d(0.0, -1.0, 0.0);
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  manager.update(input, result);
  ASSERT_FALSE(result.candidate_profile.raw_build_samples.empty());
  EXPECT_FALSE(result.candidate_profile.raw_build_samples.front().pre_inset_contains_zero);
  EXPECT_FALSE(result.candidate_profile.zero_centerline_continuously_certified);
}

TEST(TubeEpochManagerTest,
     CurrentDeltaSelectsSafeComponentWithoutZeroBridging) {
  TubeEpochUpdateInput input = MakeCloudInput(CloudCorridorQuery(0.34, 2.30));
  input.retained_delta = -1.0;
  input.actual_position += Eigen::Vector3d(0.0, -1.0, 0.0);
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  manager.update(input, result);
  ASSERT_FALSE(result.candidate_profile.raw_build_samples.empty());
  const TubeRawSample& raw = result.candidate_profile.raw_build_samples.front();
  EXPECT_FALSE(raw.pre_inset_contains_zero);
  EXPECT_FALSE(result.candidate_profile.zero_centerline_continuously_certified);
}

TEST(TubeEpochManagerTest, CloudGeometricContainmentStaysSeparateFromInteriorMargin) {
  TubeEpochManagerConfig config = MakeConfig();
  config.builder.interior_margin = 0.05;
  TubeEpochUpdateInput input = MakeCloudInput(CloudCorridorQuery(1.00, 1.00));
  // The corridor's geometric interval is approximately [-0.60,+0.60].  A
  // retained offset near its upper boundary is outside the legacy interior
  // margin but remains geometrically contained for ESDF installation.
  input.retained_delta = 0.58;
  input.actual_position += Eigen::Vector3d(0.0, 0.58, 0.0);
  TubeEpochManager manager(config);
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(input, result));
  EXPECT_FALSE(result.status.retained_delta_current_inside);
  EXPECT_TRUE(result.status.current_interval_contains_retained_delta);
  EXPECT_TRUE(result.status.current_state_admissible);
  EXPECT_EQ(result.status.state, TubeEpochState::ROLLING);
}

TEST(TubeEpochManagerTest, CloudExcludingZeroCandidateInstallsNeutralZeroOnly) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  EXPECT_TRUE(manager.update(MakeCloudInput(CloudCorridorQuery(0.10, 2.30)), result));
  EXPECT_TRUE(result.status.raw_cross_section_path_used);
  EXPECT_TRUE(result.status.candidate_complete);
  EXPECT_TRUE(result.candidate_profile.complete);
  EXPECT_TRUE(result.status.current_interval_contains_zero);
  EXPECT_TRUE(result.status.current_interval_contains_retained_delta);
  EXPECT_TRUE(result.status.retained_delta_current_inside);
  EXPECT_TRUE(result.status.active_available);
  EXPECT_FALSE(result.active_profile.samples.empty());
  EXPECT_EQ(result.status.active_tube_epoch, 1U);
  EXPECT_EQ(result.status.state, TubeEpochState::ROLLING);
  EXPECT_EQ(result.status.disposition, TubeInstallDisposition::INITIAL_INSTALL);
  EXPECT_EQ(result.status.reason, TubeEpochReason::NONE);
  EXPECT_FALSE(result.status.certificate_denied);
  EXPECT_FALSE(result.status.base_centerline_clearance_sufficient);
  ASSERT_FALSE(result.candidate_profile.samples.empty());
  EXPECT_EQ(result.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_DOUBLE_EQ(result.candidate_profile.samples.front().raw_upper, 0.0);
}

TEST(TubeEpochManagerTest, NeutralZeroOnlyCandidateReplacesPriorOffsetProfile) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult active;
  ASSERT_TRUE(manager.update(MakeCloudInput(), active));
  TubeEpochUpdateResult zero_only;
  EXPECT_TRUE(manager.update(MakeCloudInput(CloudCorridorQuery(0.10, 2.30)),
                             zero_only));
  EXPECT_GT(zero_only.status.active_tube_epoch, active.status.active_tube_epoch);
  EXPECT_EQ(zero_only.status.active_path_source_revision,
            active.status.active_path_source_revision);
  EXPECT_TRUE(zero_only.status.active_current_validation_valid);
  EXPECT_TRUE(zero_only.status.candidate_complete);
  EXPECT_EQ(zero_only.status.reason, TubeEpochReason::NONE);
  EXPECT_EQ(zero_only.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  ASSERT_FALSE(zero_only.active_profile.samples.empty());
  EXPECT_DOUBLE_EQ(zero_only.active_profile.samples.front().raw_upper, 0.0);
}

TEST(TubeEpochManagerTest, CloudUnknownGapAndCurrentUnknownBecomeZeroOnly) {
  const ClearanceQuery side_unknown = [](const Eigen::Vector3d& point,
                                         const double) {
    ClearanceQueryResult result;
    if (point.y() >= 0.40 - 1e-10) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 10.0;
    result.clearance_certified = true;
    return result;
  };
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult truncated;
  ASSERT_TRUE(manager.update(MakeCloudInput(side_unknown), truncated));
  EXPECT_TRUE(truncated.status.candidate_complete);
  EXPECT_EQ(truncated.status.state, TubeEpochState::ROLLING);
  EXPECT_LT(truncated.candidate_profile.samples.front().raw_upper, 0.40);

  TubeEpochUpdateInput center_unknown = MakeCloudInput(
      [](const Eigen::Vector3d&, const double) {
        ClearanceQueryResult result;
        result.status = DistanceStatus::UNKNOWN;
        return result;
      });
  TubeEpochUpdateResult zero_only;
  EXPECT_TRUE(manager.update(center_unknown, zero_only));
  EXPECT_TRUE(zero_only.status.candidate_complete);
  EXPECT_EQ(zero_only.status.state, TubeEpochState::ROLLING);
  EXPECT_GT(zero_only.status.active_tube_epoch,
            truncated.status.active_tube_epoch);
  EXPECT_EQ(zero_only.status.active_classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
}

TEST(TubeEpochManagerTest, CloudTrackingViolationInstallsUncertifiedCandidateWithoutGate) {
  TubeEpochUpdateInput input = MakeCloudInput();
  input.actual_position.y() = 0.10;
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(input, result));
  EXPECT_TRUE(result.status.candidate_complete);
  EXPECT_FALSE(result.status.tracking_within_bound);
  EXPECT_TRUE(result.status.active_available);
  EXPECT_EQ(result.status.active_tube_epoch, 1U);
  EXPECT_EQ(result.status.disposition, TubeInstallDisposition::INITIAL_INSTALL);
  EXPECT_EQ(result.status.state, TubeEpochState::ROLLING);
  EXPECT_EQ(result.status.reason, TubeEpochReason::NONE);
}

TEST(TubeEpochManagerTest, CloudMaterialEnvironmentFailurePreservesEquivalentActiveEpoch) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput();
  TubeEpochUpdateResult first;
  ASSERT_TRUE(manager.update(input, first));
  input.map_observation_sequence = 2U;
  TubeEpochUpdateResult equivalent;
  ASSERT_TRUE(manager.update(input, equivalent));
  EXPECT_EQ(equivalent.status.active_tube_epoch, first.status.active_tube_epoch);
  EXPECT_EQ(equivalent.status.disposition, TubeInstallDisposition::EQUIVALENT_REFRESH);

  input.cloud_clearance_query = CloudCorridorQuery(1.00, 2.50);
  input.map_observation_sequence = 3U;
  TubeEpochUpdateResult material;
  EXPECT_FALSE(manager.update(input, material));
  EXPECT_EQ(material.status.disposition,
            TubeInstallDisposition::REJECTED_CANDIDATE);
  EXPECT_EQ(material.status.active_tube_epoch, first.status.active_tube_epoch);
  EXPECT_DOUBLE_EQ(material.active_profile.samples.front().raw_lower,
                   first.active_profile.samples.front().raw_lower);
}

TEST(TubeEpochManagerTest,
     CloudFutureUnknownRetainsCandidateAndWaitsForShortHorizon) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput([](const Eigen::Vector3d& point,
                                                  const double) {
    ClearanceQueryResult result;
    if (point.x() >= 0.20 - 1e-12) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 10.0;
    result.clearance_certified = true;
    return result;
  });
  TubeEpochUpdateResult result;
  EXPECT_FALSE(manager.update(input, result));
  EXPECT_TRUE(result.status.candidate_complete);
  EXPECT_TRUE(result.candidate_profile.raw_complete);
  EXPECT_TRUE(result.candidate_profile.certified_segment_truncated_after);
  EXPECT_FALSE(result.candidate_profile.certified_segment_truncated_before);
  EXPECT_GE(result.candidate_profile.samples.size(), 2U);
  EXPECT_NEAR(result.candidate_profile.requested_preview_end_w, 0.6, 1e-12);
  EXPECT_NEAR(result.candidate_profile.certified_segment_end_w, 0.20, 1e-12);
  EXPECT_NEAR(result.status.certified_forward_w, 0.20, 1e-12);
  EXPECT_FALSE(result.status.forward_horizon_sufficient);
  EXPECT_EQ(result.status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  EXPECT_EQ(result.status.reason, TubeEpochReason::FORWARD_HORIZON_SHORT);
  EXPECT_NEAR(result.candidate_profile.first_truncated_w, 0.20, 1e-12);
  EXPECT_EQ(result.candidate_profile.first_truncated_reason,
            TubeStopReason::UNKNOWN);
}

TEST(TubeEpochManagerTest,
     CloudFutureUnknownRollsWhenTheRetainedForwardHorizonIsSufficient) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput([](const Eigen::Vector3d& point,
                                                  const double) {
    ClearanceQueryResult result;
    if (point.x() >= 0.50 - 1e-12) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 10.0;
    result.clearance_certified = true;
    return result;
  });
  TubeEpochUpdateResult result;
  ASSERT_TRUE(manager.update(input, result));
  EXPECT_TRUE(result.status.candidate_complete);
  EXPECT_TRUE(result.candidate_profile.certified_segment_truncated_after);
  EXPECT_GE(result.status.certified_forward_w, 0.30);
  EXPECT_TRUE(result.status.forward_horizon_sufficient);
  EXPECT_EQ(result.status.state, TubeEpochState::ROLLING);
  EXPECT_TRUE(result.status.active_available);
}

TEST(TubeEpochManagerTest,
     CloudInvalidSampleBeforeCurrentDoesNotInvalidateTheCurrentContainingSegment) {
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateInput input = MakeCloudInput([](const Eigen::Vector3d& point,
                                                  const double) {
    ClearanceQueryResult result;
    if (point.x() < 0.20 - 1e-12) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 10.0;
    result.clearance_certified = true;
    return result;
  });
  input.current_path = input.preview_path[4U];
  input.actual_position = input.current_path.p;
  TubeEpochUpdateResult result;
  EXPECT_FALSE(manager.update(input, result));
  EXPECT_TRUE(result.status.candidate_complete);
  EXPECT_TRUE(result.candidate_profile.certified_segment_truncated_before);
  EXPECT_FALSE(result.candidate_profile.certified_segment_truncated_after);
  EXPECT_NEAR(result.candidate_profile.certified_segment_start_w, 0.2, 1e-12);
  EXPECT_NEAR(result.candidate_profile.certified_segment_end_w, 0.6, 1e-12);
  EXPECT_EQ(result.status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  EXPECT_EQ(result.status.reason, TubeEpochReason::FORWARD_HORIZON_SHORT);
}

TEST(TubeEpochManagerTest, EsdfWithoutCloudClearanceFailsClosed) {
  TubeEpochUpdateInput input = MakeInput(TubeSource::ESDF);
  // This fixture intentionally omits both cloud and cell-certificate
  // callbacks; keep the missing-certificate condition explicit now that the
  // normal cloud fixture is certificate-backed.
  input.path_cell_bound_query = PathCellBoundQuery();
  TubeEpochManager manager(MakeConfig());
  TubeEpochUpdateResult result;
  EXPECT_FALSE(manager.update(input, result));
  EXPECT_FALSE(result.status.raw_cross_section_path_used);
  EXPECT_FALSE(result.status.candidate_complete);
  EXPECT_EQ(result.status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
