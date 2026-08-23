#include <gtest/gtest.h>

#include "phase_offset_navigation/tube_surface_validator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <iostream>
#include <vector>

namespace phase_offset_navigation {
namespace {

std::uint64_t Bits(const double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

std::array<std::uint64_t, 11U> PathStateBits(
    const phase_offset_core::PathDifferentialState& state) {
  return {{Bits(state.p.x()), Bits(state.p.y()), Bits(state.p.z()),
           Bits(state.p_w.x()), Bits(state.p_w.y()), Bits(state.p_w.z()),
           Bits(state.p_ww.x()), Bits(state.p_ww.y()), Bits(state.p_ww.z()),
           Bits(state.w), state.valid ? 1U : 0U}};
}

TubeProfile MakeProfile(const std::vector<double>& w,
                        double lower = -0.20,
                        double upper = 0.20) {
  TubeProfile profile;
  profile.source = TubeSource::ESDF;
  profile.raw_complete = true;
  profile.filtered_complete = true;
  profile.complete = true;
  profile.obstacle_certified = true;
  profile.preview_start_w = w.front();
  profile.preview_end_w = w.back();
  profile.requested_preview_start_w = w.front();
  profile.requested_preview_end_w = w.back();
  profile.certified_segment_start_w = w.front();
  profile.certified_segment_end_w = w.back();
  for (const double phase : w) {
    TubeRawSample sample;
    sample.w = phase;
    sample.p = Eigen::Vector3d(phase, 0.0, 0.0);
    sample.N = Eigen::Vector3d::UnitY();
    sample.raw_lower = lower;
    sample.raw_upper = upper;
    sample.filtered_lower = lower;
    sample.filtered_upper = upper;
    sample.fixed_lower = lower;
    sample.fixed_upper = upper;
    sample.regularity_lower = lower;
    sample.regularity_upper = upper;
    sample.esdf_lower = lower;
    sample.esdf_upper = upper;
    sample.complete = true;
    sample.positive_certified = true;
    sample.negative_certified = true;
    sample.regularity_intersection = true;
    profile.samples.push_back(sample);
  }
  return profile;
}

TubeProfile MakeExactCurrentAnchorProfile() {
  // Exact zero-width current knot: the current-anchor 3x3 points are all
  // identical, but the validator must still issue all nine queries.
  return MakeProfile({0.0, 0.20}, 0.0, 0.0);
}

PathStateQuery LinePath() {
  return [](const double w, phase_offset_core::PathDifferentialState& state) {
    state.p = Eigen::Vector3d(w, 0.0, 0.0);
    state.p_w = Eigen::Vector3d::UnitX();
    state.p_ww = Eigen::Vector3d::Zero();
    state.w = w;
    state.valid = std::isfinite(w);
    return state.valid;
  };
}

PathStateQuery UnitCirclePath() {
  return [](const double w, phase_offset_core::PathDifferentialState& state) {
    state.p = Eigen::Vector3d(std::cos(w), std::sin(w), 0.0);
    state.p_w = Eigen::Vector3d(-std::sin(w), std::cos(w), 0.0);
    state.p_ww = Eigen::Vector3d(-std::cos(w), -std::sin(w), 0.0);
    state.w = w;
    state.valid = std::isfinite(w);
    return state.valid;
  };
}

PathStateQuery SlowFrameBoundPath() {
  return [](const double w, phase_offset_core::PathDifferentialState& state) {
    state = phase_offset_core::PathDifferentialState();
    state.p = Eigen::Vector3d(w, 0.0, 0.0);
    state.p_w = Eigen::Vector3d(0.20, 0.0, 0.0);
    state.p_ww = Eigen::Vector3d::Zero();
    state.T = Eigen::Vector3d::UnitX();
    state.N = Eigen::Vector3d::UnitY();
    state.N_w = Eigen::Vector3d::UnitX();
    state.frame_valid = true;
    state.w = w;
    state.valid = std::isfinite(w);
    return state.valid;
  };
}

ClearanceQuery OpenQuery() {
  return [](const Eigen::Vector3d&, const double required) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(5.0, required);
    result.clearance_certified = true;
    return result;
  };
}

ClearanceQuery PointObstacle(const Eigen::Vector3d& obstacle) {
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

PathCellBoundQuery CertifiedLineCells(const double inf_speed = 1.0,
                                      const double sup_normal_w = 0.0,
                                      const double sup_curvature = 0.0,
                                      const double midpoint_scale = 0.5) {
  return [inf_speed, sup_normal_w, sup_curvature, midpoint_scale](
      const double w0, const double w1,
      phase_offset_core::PathCellGeometryCertificate& certificate) {
    certificate = phase_offset_core::PathCellGeometryCertificate();
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.segment_w0 = 0.0;
    certificate.segment_w1 = 1.0;
    certificate.segment_identity = 1U;
    certificate.inf_p_w_norm = inf_speed;
    certificate.inf_horizontal_p_w_norm = inf_speed;
    certificate.sup_p_w_norm = 1.0;
    certificate.sup_p_ww_norm = 0.0;
    certificate.sup_p_www_norm = 0.0;
    certificate.sup_N_w_norm = sup_normal_w;
    certificate.sup_abs_curvature = sup_curvature;
    certificate.normal_variation_bound = sup_normal_w * (w1 - w0);
    certificate.curvature_variation_bound = 0.0;
    certificate.midpoint_position_variation_bound = midpoint_scale * (w1 - w0);
    certificate.chord_deviation_bound = 0.0;
    certificate.valid = std::isfinite(w0) && std::isfinite(w1) && w1 > w0;
    certificate.complete = certificate.valid;
    return certificate.valid;
  };
}

TubeProfile MakeDeterministicWideRibbonProfile() {
  // Constant wide bounds make the decomposition exactly v-only:
  // fixed=0.025, w_reducible=0, v_reducible=1.2*v_span.  The current anchor
  // and the one full path cell each need three v halvings before cover<=0.20.
  // Under the pre-change isotropic splitter the full cell also creates an
  // unnecessary 2^3 w partition, so a 288-query budget is exhausted even
  // though the 16 accepted v leaves would need only 144 queries.
  TubeProfile profile = MakeProfile({0.0, 1.0}, -1.2, 1.2);
  profile.cell_geometry_certified = true;
  return profile;
}

PathCellBoundQuery CertifiedConstantLineCells() {
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

ClearanceQuery CountingKnownFreeQuery(const std::shared_ptr<std::size_t>& calls,
                                      const std::shared_ptr<std::size_t>& known_free,
                                      const std::shared_ptr<bool>& all_known_free) {
  return [calls, known_free, all_known_free](const Eigen::Vector3d&,
                                             const double required) {
    ++(*calls);
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    if (result.status == DistanceStatus::KNOWN_FREE) {
      ++(*known_free);
    } else {
      *all_known_free = false;
    }
    return result;
  };
}

TEST(TubeSurfaceValidatorTest, OpenRibbonIsCertifiedAndQueriesResidualPlusCover) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20});
  const std::shared_ptr<double> minimum(new double(std::numeric_limits<double>::infinity()));
  const ClearanceQuery query = [minimum](const Eigen::Vector3d& point,
                                          const double required) {
    *minimum = std::min(*minimum, required);
    return OpenQuery()(point, required);
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(profile, 0.0, LinePath(), query,
                                               0.05, 0.45, 0.10, result));
  EXPECT_TRUE(result.complete);
  EXPECT_TRUE(result.current_anchor_valid);
  EXPECT_TRUE(result.cover_accounting_observed);
  EXPECT_TRUE(profile.obstacle_certified);
  EXPECT_GE(*minimum, 0.45);
  EXPECT_GT(result.max_cover_radius, 0.0);
  EXPECT_GE(result.max_requested_clearance, 0.45);
  EXPECT_GT(result.min_cover_radius, 0.0);
  EXPECT_EQ(profile.samples.size(), 3U);
  EXPECT_TRUE(profile.zero_centerline_continuously_certified);
  EXPECT_TRUE(result.zero_centerline_continuously_certified);
  ASSERT_EQ(profile.validator_knot_evidence.size(), profile.samples.size());
  for (const TubeValidatorKnotEvidence& evidence :
       profile.validator_knot_evidence) {
    EXPECT_TRUE(evidence.observed);
    EXPECT_TRUE(evidence.filtered_contains_zero);
    EXPECT_TRUE(evidence.zero_surface_covered);
    EXPECT_GT(evidence.max_cover_radius, 0.0);
    EXPECT_GE(evidence.max_requested_clearance, 0.45);
  }
}

TEST(TubeSurfaceValidatorTest, ConfiguredMinimumReferenceSpeedIsEnforced) {
  TubeSurfaceValidatorConfig config;
  config.minimum_reference_speed = 0.50;
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.10, 0.10);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, SlowFrameBoundPath(), OpenQuery(), 0.05, 0.0, 0.1,
      result));
  EXPECT_EQ(result.first_failure_reason, TubeStopReason::REGULARITY);
}

TEST(TubeSurfaceValidatorTest,
     ExactCurrentAnchorSampledCoverOmitsOnlyFixedHalfVoxel) {
  TubeProfile profile = MakeExactCurrentAnchorProfile();
  std::vector<double> requested;
  std::vector<double> returned;
  std::vector<Eigen::Vector3d> points;
  const ClearanceQuery query = [&requested, &returned, &points](
      const Eigen::Vector3d& point, const double required) {
    points.push_back(point);
    requested.push_back(required);
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    // The query is immutable by world point: every sample on the exact
    // current anchor receives the same clearance, independent of traversal
    // order or callback count.  The non-anchor cell retains the larger
    // clearance needed for its sampled cover.
    result.clearance = std::abs(point.x()) <= 1e-12
        ? 0.425 : std::max(5.0, required);
    result.clearance_certified = true;
    returned.push_back(result.clearance);
    return result;
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), query, 0.05, 0.40, 0.10, result));
  ASSERT_GE(requested.size(), 18U);
  ASSERT_EQ(requested.size(), result.query_sample_count);
  EXPECT_TRUE(result.current_anchor_valid);
  EXPECT_TRUE(profile.obstacle_certified);
  // The exact anchor has sampled_radius=0, so only residual + epsilon remains.
  for (std::size_t index = 0U; index < 9U; ++index) {
    EXPECT_NEAR(requested[index], 0.400001, 1e-12);
    EXPECT_DOUBLE_EQ(points[index].x(), 0.0);
    EXPECT_LT(returned[index], 0.450001);
  }
  // The first non-anchor cell is nondegenerate and retains its fixed term.
  EXPECT_GT(requested[9U], 0.400001);
}

TEST(TubeSurfaceValidatorTest,
     ExactCurrentAnchorTrueInsufficientClearanceRemainsFailClosed) {
  TubeProfile profile = MakeExactCurrentAnchorProfile();
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 0;
  std::size_t calls = 0U;
  const ClearanceQuery query = [&calls](const Eigen::Vector3d&, double) {
    ++calls;
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 0.4000005;
    result.clearance_certified = true;
    return result;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), query, 0.05, 0.40, 0.10, result));
  EXPECT_FALSE(result.current_anchor_valid);
  EXPECT_EQ(calls, 9U);
  EXPECT_EQ(result.query_sample_count, 9U);
  EXPECT_EQ(result.first_failure_reason, TubeStopReason::INSUFFICIENT_CLEARANCE);
  EXPECT_FALSE(profile.obstacle_certified);
}

TEST(TubeSurfaceValidatorTest,
     ExactCurrentAnchorCategoricalAndCertificationFailuresFailClosed) {
  struct FailureCase {
    DistanceStatus status;
    bool certified;
    double clearance;
  };
  const FailureCase cases[] = {
      {DistanceStatus::UNKNOWN, false, 0.0},
      {DistanceStatus::OUT_OF_MAP, false, 0.0},
      {DistanceStatus::OCCUPIED, false, 0.0},
      {DistanceStatus::UNAVAILABLE, false, 0.0},
      {DistanceStatus::KNOWN_FREE, false, 0.425},
      {DistanceStatus::KNOWN_FREE, true,
       std::numeric_limits<double>::quiet_NaN()},
  };
  for (const FailureCase& failure : cases) {
    TubeProfile profile = MakeExactCurrentAnchorProfile();
    TubeSurfaceValidatorConfig config;
    config.max_subdivision_depth = 0;
    std::size_t calls = 0U;
    const ClearanceQuery query = [&calls, failure](
        const Eigen::Vector3d&, double) {
      ++calls;
      ClearanceQueryResult result;
      result.status = failure.status;
      result.clearance = failure.clearance;
      result.clearance_certified = failure.certified;
      return result;
    };
    TubeSurfaceValidationResult result;
    EXPECT_FALSE(TubeSurfaceValidator(config).validate(
        profile, 0.0, LinePath(), query, 0.05, 0.40, 0.10, result));
    EXPECT_FALSE(result.current_anchor_valid);
    EXPECT_EQ(calls, 9U);
    EXPECT_EQ(result.query_sample_count, 9U);
    EXPECT_NE(result.first_failure_reason, TubeStopReason::NONE);
    EXPECT_FALSE(profile.obstacle_certified);
  }
}

TEST(TubeSurfaceValidatorTest,
     NondegenerateSampledFallbackRetainsFixedHalfVoxel) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.10, 0.10);
  // A caller cannot carry a stale continuous label through sampled fallback.
  profile.proof_level = TubeProofLevel::CONTINUOUS_COVER_PROOF;
  std::vector<Eigen::Vector3d> points;
  std::vector<double> requested;
  const ClearanceQuery query = [&points, &requested](
      const Eigen::Vector3d& point, const double required) {
    points.push_back(point);
    requested.push_back(required);
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(5.0, required);
    result.clearance_certified = true;
    return result;
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), query, 0.05, 0.40, 0.10, result));
  ASSERT_EQ(points.size(), 18U);
  ASSERT_EQ(requested.size(), 18U);
  const Eigen::Vector3d& center = points[13U];
  double sampled_radius = 0.0;
  for (std::size_t index = 9U; index < 18U; ++index) {
    sampled_radius = std::max(sampled_radius,
                               (points[index] - center).norm());
  }
  const double expected_cover = 1.1 * sampled_radius + 0.025;
  EXPECT_NEAR(requested[9U], 0.40 + expected_cover + 1e-6, 1e-12);
  EXPECT_NEAR(result.max_cover_radius, expected_cover, 1e-12);
  EXPECT_NE(profile.proof_level, TubeProofLevel::CONTINUOUS_COVER_PROOF);
}

TEST(TubeSurfaceValidatorTest, CertifiedCellCoverUsesAnalyticRadiusAndKeepsZero) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20});
  profile.cell_geometry_certified = true;
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), OpenQuery(), 0.05, 0.45,
      0.10, result));
  EXPECT_TRUE(result.complete);
  EXPECT_TRUE(result.zero_centerline_continuously_certified);
  EXPECT_LT(result.max_cover_radius, 0.50);
  EXPECT_NE(profile.proof_level, TubeProofLevel::CONTINUOUS_COVER_PROOF);
}

TEST(TubeSurfaceValidatorTest,
     MissingOrMismatchedCellEvidenceCannotUpgradeContinuousProof) {
  TubeProfile missing = MakeProfile({0.0, 0.20}, -0.10, 0.10);
  missing.cell_geometry_certified = true;
  missing.combined_regularity_proof_complete = true;
  TubeSurfaceValidationResult missing_result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      missing, 0.0, LinePath(), OpenQuery(), 0.05, 0.40, 0.10,
      missing_result));
  EXPECT_NE(missing.proof_level, TubeProofLevel::CONTINUOUS_COVER_PROOF);

  TubeProfile mismatched = MakeProfile({0.0, 0.20}, -0.10, 0.10);
  mismatched.cell_geometry_certified = true;
  mismatched.combined_regularity_proof_complete = true;
  mismatched.path_revision = 11U;
  mismatched.frame_revision = 12U;
  const PathCellBoundQuery wrong_revision =
      [](const double w0, const double w1,
         phase_offset_core::PathCellGeometryCertificate& certificate) {
    certificate = phase_offset_core::PathCellGeometryCertificate();
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.segment_w0 = 0.0;
    certificate.segment_w1 = 1.0;
    certificate.segment_identity = 1U;
    certificate.path_revision = 99U;
    certificate.frame_revision = 100U;
    certificate.inf_p_w_norm = 1.0;
    certificate.inf_horizontal_p_w_norm = 1.0;
    certificate.sup_p_w_norm = 1.0;
    certificate.valid = true;
    certificate.complete = true;
    certificate.normal_frame_proof_complete = true;
    return true;
  };
  TubeSurfaceValidationResult mismatched_result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      mismatched, 0.0, LinePath(), wrong_revision, OpenQuery(), 0.05,
      0.40, 0.10, mismatched_result));
  EXPECT_NE(mismatched.proof_level, TubeProofLevel::CONTINUOUS_COVER_PROOF);

  TubeProfile unbound = MakeProfile({0.0, 0.20}, -0.10, 0.10);
  unbound.cell_geometry_certified = true;
  unbound.combined_regularity_proof_complete = true;
  unbound.path_revision = 11U;
  unbound.frame_revision = 12U;
  const PathCellBoundQuery unbound_revision =
      [](const double w0, const double w1,
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
    certificate.valid = true;
    certificate.complete = true;
    certificate.normal_frame_proof_complete = true;
    return true;
  };
  TubeSurfaceValidationResult unbound_result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      unbound, 0.0, LinePath(), unbound_revision, OpenQuery(), 0.05,
      0.40, 0.10, unbound_result));
  EXPECT_NE(unbound.proof_level, TubeProofLevel::CONTINUOUS_COVER_PROOF);
}

TEST(TubeSurfaceValidatorTest,
     CertifiedCellRegularityOrOffsetSpeedFailureDoesNotUseSampledFallback) {
  {
    TubeProfile profile = MakeProfile({0.0, 0.10});
    profile.cell_geometry_certified = true;
    TubeSurfaceValidationResult result;
    EXPECT_FALSE(TubeSurfaceValidator().validate(
        profile, 0.0, LinePath(), CertifiedLineCells(0.1, 2.0, 0.0),
        OpenQuery(), 0.05, 0.20, 0.10, result));
    EXPECT_LE(result.query_sample_count, 27U);
    EXPECT_EQ(result.first_failure_reason, TubeStopReason::REGULARITY);
  }
  {
    TubeProfile profile = MakeProfile({0.0, 0.10});
    profile.cell_geometry_certified = true;
    TubeSurfaceValidationResult result;
    EXPECT_FALSE(TubeSurfaceValidator().validate(
        profile, 0.0, LinePath(), CertifiedLineCells(0.1, 2.0, 0.0),
        OpenQuery(), 0.05, 0.20, 0.10, result));
    EXPECT_LE(result.query_sample_count, 27U);
    EXPECT_EQ(result.first_failure_reason, TubeStopReason::REGULARITY);
  }
}

TEST(TubeSurfaceValidatorTest,
     NonzeroRibbonDoesNotCreateZeroCentrelineCertificate) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20}, -0.20, -0.05);
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(profile, 0.0, LinePath(),
                                               OpenQuery(), 0.05, 0.45, 0.10,
                                               result));
  EXPECT_TRUE(result.complete);
  EXPECT_FALSE(profile.zero_centerline_continuously_certified);
  EXPECT_FALSE(result.zero_centerline_continuously_certified);
  ASSERT_EQ(profile.validator_knot_evidence.size(), profile.samples.size());
  for (const TubeValidatorKnotEvidence& evidence :
       profile.validator_knot_evidence) {
    EXPECT_TRUE(evidence.observed);
    EXPECT_FALSE(evidence.filtered_contains_zero);
    EXPECT_FALSE(evidence.zero_surface_covered);
  }
}

TEST(TubeSurfaceValidatorTest,
     SubdivisionPreservesRowMajorNinePointGeometryAndClearanceTraversal) {
  TubeProfile profile = MakeProfile({0.0, 0.40});
  const std::shared_ptr<std::vector<double>> path_w(new std::vector<double>());
  const std::shared_ptr<std::vector<std::array<std::uint64_t, 11U>>> path_states(
      new std::vector<std::array<std::uint64_t, 11U>>());
  const std::shared_ptr<bool> repeated_path_state_is_identical(new bool(true));
  const std::shared_ptr<std::vector<Eigen::Vector3d>> clearance_points(
      new std::vector<Eigen::Vector3d>());
  const std::shared_ptr<std::vector<double>> clearance_radii(new std::vector<double>());
  const PathStateQuery path = [path_w, path_states, repeated_path_state_is_identical](
      const double w, phase_offset_core::PathDifferentialState& state) {
    state.p = Eigen::Vector3d(w, 0.0, 0.0);
    state.p_w = Eigen::Vector3d::UnitX();
    state.p_ww = Eigen::Vector3d::Zero();
    state.w = w;
    state.valid = std::isfinite(w);
    const std::array<std::uint64_t, 11U> state_bits = PathStateBits(state);
    for (std::size_t index = 0U; index < path_w->size(); ++index) {
      if (Bits((*path_w)[index]) == Bits(w) &&
          (*path_states)[index] != state_bits) {
        *repeated_path_state_is_identical = false;
      }
    }
    path_w->push_back(w);
    path_states->push_back(state_bits);
    return state.valid;
  };
  const ClearanceQuery query = [clearance_points, clearance_radii](
      const Eigen::Vector3d& point, const double required) {
    clearance_points->push_back(point);
    clearance_radii->push_back(required);
    return OpenQuery()(point, required);
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(profile, 0.0, path, query,
                                               0.05, 0.20, 0.10, result));
  EXPECT_GT(path_w->size(), 0U);
  ASSERT_EQ(result.query_sample_count, clearance_points->size());
  ASSERT_EQ(result.query_sample_count, clearance_radii->size());
  EXPECT_EQ(result.clearance_leaf_cell_count * 9U,
            result.query_sample_count);
  EXPECT_TRUE(*repeated_path_state_is_identical);
  for (std::size_t index = 0U; index < result.query_sample_count; ++index) {
    EXPECT_GE((*clearance_radii)[index], 0.20);
    bool row_w_observed = false;
    for (const double observed_w : *path_w) {
      row_w_observed = row_w_observed ||
          Bits(observed_w) == Bits((*clearance_points)[index].x());
    }
    EXPECT_TRUE(row_w_observed);
  }
  EXPECT_TRUE(result.complete);
  EXPECT_TRUE(result.current_anchor_valid);
  EXPECT_TRUE(profile.obstacle_certified);
  EXPECT_EQ(profile.samples.size(), 2U);
}

TEST(TubeSurfaceValidatorTest, BetweenKnotObstacleRejectsContinuousRibbon) {
  TubeProfile profile = MakeProfile({0.0, 1.0});
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), PointObstacle(Eigen::Vector3d(0.5, 0.0, 0.0)),
      0.05, 0.20, 0.10, result));
  EXPECT_FALSE(result.complete);
  EXPECT_FALSE(profile.obstacle_certified);
}

TEST(TubeSurfaceValidatorTest, InteriorUnsafeIsFoundWhenBothBoundariesAreClear) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.30, 0.30);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), PointObstacle(Eigen::Vector3d(0.10, 0.0, 0.0)),
      0.05, 0.15, 0.10, result));
  EXPECT_NE(result.first_failure_reason, TubeStopReason::NONE);
}

TEST(TubeSurfaceValidatorTest, FarUnsafeSuffixTruncatesButRetainsCurrentSegment) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20, 0.30});
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), PointObstacle(Eigen::Vector3d(0.28, 0.0, 0.0)),
      0.05, 0.12, 0.10, result));
  EXPECT_TRUE(result.complete);
  EXPECT_TRUE(result.truncated_after);
  EXPECT_LE(profile.preview_end_w, 0.20);
  EXPECT_GE(profile.samples.size(), 2U);
}

TEST(TubeSurfaceValidatorTest, UnknownObservedEdgeFailsClosed) {
  TubeProfile profile = MakeProfile({0.0, 0.20});
  const ClearanceQuery query = [](const Eigen::Vector3d& point,
                                  const double) {
    ClearanceQueryResult result;
    if (point.x() > 0.12) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 5.0;
    result.clearance_certified = true;
    return result;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(profile, 0.0, LinePath(), query,
                                                0.05, 0.10, 0.10, result));
  EXPECT_FALSE(profile.obstacle_certified);
}

TEST(TubeSurfaceValidatorTest, SampleLimitFailsClosed) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, -1.0, 1.0);
  TubeSurfaceValidatorConfig config;
  config.max_query_samples = 9U;
  config.max_subdivision_depth = 4;
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), OpenQuery(), 0.01, 0.10, 0.10, result));
  EXPECT_TRUE(result.limit_exceeded);
}

TEST(TubeSurfaceValidatorTest,
     RegularityFailureAtEachRowDeltaSkipsCellClearance) {
  const double lower[] = {0.95, 0.70, 0.60};
  const double upper[] = {1.00, 1.20, 1.00};
  for (std::size_t index = 0U; index < 3U; ++index) {
    TubeProfile profile = MakeProfile({0.0, 0.10}, lower[index], upper[index]);
    TubeSurfaceValidatorConfig config;
    // Isolate the failing cell: without subdivision, any observed clearance
    // call would have to belong to that same 3x3 row-major collection.
    config.max_subdivision_depth = 0;
    std::size_t clearance_calls = 0U;
    const ClearanceQuery query = [&clearance_calls](const Eigen::Vector3d&,
                                                     const double) {
      ++clearance_calls;
      return OpenQuery()(Eigen::Vector3d::Zero(), 0.0);
    };
    TubeSurfaceValidationResult result;
    EXPECT_FALSE(TubeSurfaceValidator(config).validate(
        profile, 0.0, UnitCirclePath(), query, 0.05, 0.20, 0.10, result));
    EXPECT_EQ(TubeStopReason::REGULARITY, result.first_failure_reason);
    EXPECT_DOUBLE_EQ(0.0, result.first_failure_w);
    EXPECT_EQ(0U, result.query_sample_count);
    EXPECT_EQ(0U, clearance_calls);
    EXPECT_FALSE(profile.obstacle_certified);
  }
}

TEST(TubeSurfaceValidatorTest,
     DeterministicVOnlyWideRibbonReachesQueryLimitBeforeCapacityCorrection) {
  TubeProfile profile = MakeDeterministicWideRibbonProfile();
  TubeSurfaceValidatorConfig config;
  config.max_query_samples = 288U;
  config.max_subdivision_depth = 8;
  const std::shared_ptr<std::size_t> callback_calls(new std::size_t(0U));
  const std::shared_ptr<std::size_t> known_free_calls(new std::size_t(0U));
  const std::shared_ptr<bool> all_known_free(new bool(true));
  const ClearanceQuery query = CountingKnownFreeQuery(
      callback_calls, known_free_calls, all_known_free);
  std::size_t path_calls = 0U;
  const PathStateQuery path = [&path_calls](
      const double w, phase_offset_core::PathDifferentialState& state) {
    ++path_calls;
    return LinePath()(w, state);
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator(config).validate(
      profile, 0.0, path, CertifiedConstantLineCells(), query, 0.05, 0.20,
      0.10,
      result));
  EXPECT_GE(path_calls, 3U);
  EXPECT_TRUE(profile.complete);
  EXPECT_TRUE(*all_known_free);
  EXPECT_EQ(*callback_calls, *known_free_calls);
  EXPECT_EQ(*callback_calls, result.query_sample_count);
  EXPECT_EQ(result.query_sample_count, 144U);
  EXPECT_LT(result.query_sample_count, config.max_query_samples);
  EXPECT_FALSE(result.limit_exceeded);
  EXPECT_EQ(result.first_failure_reason, TubeStopReason::NONE);
  EXPECT_TRUE(result.complete);
  EXPECT_TRUE(result.current_anchor_valid);
  EXPECT_TRUE(result.cover_accounting_observed);
  EXPECT_GT(result.max_cover_radius, 0.0);
  EXPECT_EQ(result.split_w_count, 0U);
  EXPECT_EQ(result.split_both_count, 0U);
  EXPECT_GT(result.split_v_count, 0U);
  EXPECT_GT(result.anisotropic_split_count, 0U);
  EXPECT_NEAR(result.max_cover_radius, 1.1 * 1.2, 1e-12);
  // All observed queries are KNOWN_FREE; the immutable query never reports
  // OCCUPIED/UNKNOWN/OUT_OF_MAP/UNAVAILABLE.
  EXPECT_EQ(*known_free_calls, result.query_sample_count);
  std::cout << "WIDE_RIBBON_T3 geometry_valid=1 path_calls="
            << path_calls << " callback_calls=" << *callback_calls
            << " known_free_calls=" << *known_free_calls
            << " query_sample_count=" << result.query_sample_count
            << " budget=" << config.max_query_samples
            << " geometry_cell_count=" << result.geometry_cell_count
            << " clearance_leaf_cell_count="
            << result.clearance_leaf_cell_count
            << " prequery_cover_split_count="
            << result.prequery_cover_split_count
            << " max_depth_observed=" << result.max_depth_observed
            << " split_w_count=" << result.split_w_count
            << " split_v_count=" << result.split_v_count
            << " split_both_count=" << result.split_both_count
            << " anisotropic_split_count="
            << result.anisotropic_split_count
            << " limit_exceeded=" << (result.limit_exceeded ? 1 : 0)
            << " first_failure_reason="
            << tubeStopReasonName(result.first_failure_reason)
            << " max_cover_radius=" << result.max_cover_radius << '\n';
}

TEST(TubeSurfaceValidatorTest,
     CertifiedCoverBreakdownKeepsLegacyNondegenerateTotal) {
  TubeProfile profile = MakeDeterministicWideRibbonProfile();
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedConstantLineCells(), OpenQuery(),
      0.05, 2.0, 0.10, result));
  const double fixed = 0.5 * 0.05;
  const double w_reducible = 0.0;
  const double v_reducible = 0.5 * 2.4 * 1.0;
  const double expected_cover = fixed + w_reducible + v_reducible;
  EXPECT_NEAR(result.min_cover_radius, expected_cover, 1e-12);
  // The path cell remains certified/nondegenerate and keeps its fixed term;
  // only the separate exact current-anchor sampled fallback drops it.
  EXPECT_NEAR(result.max_cover_radius, 1.1 * 1.2, 1e-12);
  EXPECT_TRUE(result.complete);
}

TEST(TubeSurfaceValidatorTest, ProofDerivedWOnlyPressureSplitsW) {
  TubeProfile profile = MakeProfile({0.0, 1.0}, -0.05, 0.05);
  profile.cell_geometry_certified = true;
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(1.0, 0.0, 0.0, 0.5),
      OpenQuery(), 0.05, 0.20, 0.10, result));
  EXPECT_GT(result.split_w_count, 0U);
  EXPECT_EQ(result.split_v_count, 0U);
  EXPECT_EQ(result.split_both_count, 0U);
  EXPECT_GT(result.anisotropic_split_count, 0U);
}

TEST(TubeSurfaceValidatorTest, ProofDerivedBothPressureSplitsBoth) {
  TubeProfile profile = MakeProfile({0.0, 1.0}, -1.2, 1.2);
  profile.cell_geometry_certified = true;
  TubeSurfaceValidatorConfig config;
  config.max_query_samples = 5000U;
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(1.0, 0.0, 0.0, 0.5),
      OpenQuery(), 0.05, 0.20, 0.10, result));
  EXPECT_GT(result.split_both_count, 0U);
  EXPECT_TRUE(result.complete);
}

TEST(TubeSurfaceValidatorTest,
     CombinedOnlyPressureChoosesLargerVTerm) {
  TubeProfile profile = MakeProfile({0.0, 1.0}, -0.15, 0.15);
  profile.cell_geometry_certified = true;
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(1.0, 0.0, 0.0, 0.10),
      OpenQuery(), 0.05, 0.20, 0.10, result));
  EXPECT_EQ(result.split_w_count, 0U);
  EXPECT_GT(result.split_v_count, 0U);
  EXPECT_EQ(result.split_both_count, 0U);
}

TEST(TubeSurfaceValidatorTest, ExactTieChoosesWDeterministically) {
  TubeProfile profile = MakeProfile({0.0, 1.0}, -0.10, 0.10);
  profile.cell_geometry_certified = true;
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(1.0, 0.0, 0.0, 0.10),
      OpenQuery(), 0.05, 0.20, 0.10, result));
  EXPECT_GT(result.split_w_count, 0U);
  EXPECT_EQ(result.split_v_count, 0U);
  EXPECT_EQ(result.split_both_count, 0U);
}

TEST(TubeSurfaceValidatorTest,
     SampledFallbackUsesConservativeAvailableSplitAtDepthLimit) {
  TubeProfile profile = MakeProfile({0.0, 1.0}, -1.2, 1.2);
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 1;
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), OpenQuery(), 0.05, 0.20, 0.10, result));
  EXPECT_TRUE(result.limit_exceeded);
  EXPECT_EQ(result.split_w_count, 0U);
  EXPECT_EQ(result.split_both_count, 0U);
  EXPECT_EQ(result.split_v_count, 1U);
  EXPECT_EQ(result.anisotropic_split_count, 0U);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
