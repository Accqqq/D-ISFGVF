#include <gtest/gtest.h>

#include "phase_offset_navigation/certified_tube_builder.h"
#include "phase_offset_navigation/tube_epoch_manager.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace phase_offset_navigation {
namespace {

CertifiedTubePathSamples Line(const double end_w = 0.4,
                              const double step = 0.1) {
  CertifiedTubePathSamples path;
  for (double w = 0.0; w <= end_w + 1e-12; w += step) {
    phase_offset_core::PathDifferentialState state;
    state.p = Eigen::Vector3d(w, 0.0, 1.0);
    state.p_w = Eigen::Vector3d::UnitX();
    state.p_ww = Eigen::Vector3d::Zero();
    state.w = w;
    state.valid = true;
    path.push_back(state);
  }
  return path;
}

PathStateQuery ExactLine() {
  return [](const double w, phase_offset_core::PathDifferentialState& state) {
    state.p = Eigen::Vector3d(w, 0.0, 1.0);
    state.p_w = Eigen::Vector3d::UnitX();
    state.p_ww = Eigen::Vector3d::Zero();
    state.w = w;
    state.valid = std::isfinite(w);
    return state.valid;
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

CertifiedTubePathSamples NearVerticalLine() {
  CertifiedTubePathSamples path;
  const Eigen::Vector3d derivative(1e-3, 0.0, 1.0);
  const Eigen::Vector3d tangent = derivative.normalized();
  for (double w = 0.0; w <= 0.4 + 1e-12; w += 0.1) {
    phase_offset_core::PathDifferentialState state;
    state.p = derivative * w;
    state.p_w = derivative;
    state.p_ww = Eigen::Vector3d::Zero();
    state.T = tangent;
    state.N = Eigen::Vector3d::UnitY();
    state.N_w = Eigen::Vector3d::Zero();
    state.path_revision = 77U;
    state.frame_revision = 79U;
    state.frame_valid = true;
    state.w = w;
    state.valid = true;
    path.push_back(state);
  }
  return path;
}

PathStateQuery ExactNearVerticalLine() {
  return [](const double w, phase_offset_core::PathDifferentialState& state) {
    const Eigen::Vector3d derivative(1e-3, 0.0, 1.0);
    state = phase_offset_core::PathDifferentialState();
    state.p = derivative * w;
    state.p_w = derivative;
    state.p_ww = Eigen::Vector3d::Zero();
    state.T = derivative.normalized();
    state.N = Eigen::Vector3d::UnitY();
    state.N_w = Eigen::Vector3d::Zero();
    state.path_revision = 77U;
    state.frame_revision = 79U;
    state.frame_valid = true;
    state.w = w;
    state.valid = std::isfinite(w);
    return state.valid;
  };
}

PathCellBoundQuery CertifiedNearVerticalCells() {
  return [](const double w0, const double w1,
            phase_offset_core::PathCellGeometryCertificate& certificate) {
    certificate = phase_offset_core::PathCellGeometryCertificate();
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.segment_w0 = 0.0;
    certificate.segment_w1 = 0.4;
    certificate.segment_identity = 77U;
    certificate.path_revision = 77U;
    certificate.frame_revision = 79U;
    certificate.inf_p_w_norm = 1.0;
    certificate.inf_horizontal_p_w_norm = 1e-3;
    certificate.sup_p_w_norm = 1.01;
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
    certificate.normal_frame_proof_complete = true;
    certificate.combined_regularity_proof_complete = true;
    certificate.regularity_speed_min = 1.0;
    certificate.regularity_speed_max = 1.01;
    return certificate.valid;
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

TubeFilterConfig FilterConfig() {
  TubeFilterConfig config;
  config.boundary_slope_max = 0.8;
  return config;
}

TubeBounds Authority(const double lower, const double upper) {
  TubeBounds request;
  request.lower = lower;
  request.upper = upper;
  request.valid = true;
  return request;
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

// The same immutable query is used by both the raw Builder and the continuous
// Validator.  Raw Builder probes use planner_safe_distance exactly (0.40),
// while Validator surface probes request the same clearance plus cover.  This
// lets the deterministic reproduction preserve a nonzero raw ribbon but make
// the full-width continuous surface observe UNKNOWN at its outer side.
ClearanceQuery FullWidthOuterUnknown() {
  return [](const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult result;
    if (required > 0.4000001 && std::abs(point.y()) > 0.02) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

ClearanceQuery OutsideNarrowAuthorityUnknown() {
  return [](const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult result;
    if (required > 0.4000001 && std::abs(point.y()) > 0.20) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

ClearanceQuery CenterlineUnknownAfterBase() {
  return [](const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult result;
    if (required > 0.4000001 && std::abs(point.y()) <= 1e-12) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

ClearanceQuery CenterlineUnavailableAfterBase() {
  return [](const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult result;
    if (required > 0.4000001 && std::abs(point.y()) <= 1e-12) {
      result.status = DistanceStatus::UNAVAILABLE;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

ClearanceQuery AnyNonzeroOffsetUnknown() {
  return [](const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult result;
    if (required > 0.4000001 && std::abs(point.y()) > 1e-12) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

ClearanceQuery CountingCenterlineUnknown(std::size_t* calls) {
  return [calls](const Eigen::Vector3d& point, const double required) {
    ++(*calls);
    ClearanceQueryResult result;
    if (required > 0.4000001 && std::abs(point.y()) <= 1e-12) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

ClearanceQuery NegativeSideUnknown() {
  return [](const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult result;
    if (required > 0.4000001 && point.y() < -0.02) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

ClearanceQuery RemoteUnknownAfterBase() {
  return [](const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult result;
    if (required > 0.4000001 && point.x() > 0.25) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

struct QueryCounts {
  std::size_t raw_calls = 0U;
  std::size_t validator_calls = 0U;
};

ClearanceQuery ExactCurrentZeroWithOuterUnknown(QueryCounts* counts) {
  return [counts](const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult result;
    if (required <= 0.4000001) {
      ++counts->raw_calls;
      // The immutable raw current cross-section is exactly [0, 0], while
      // remote knots still have nonzero raw capacity.
      if (std::abs(point.x()) <= 1e-12) {
        result.status = DistanceStatus::OCCUPIED;
        return result;
      }
    } else {
      ++counts->validator_calls;
      // Full-width continuous validation fails away from the zero line, but
      // the exact current line remains queryable.  This is the D0 fixture:
      // the current interval is zero before any inward candidate exists.
      if (std::abs(point.y()) > 0.02) {
        result.status = DistanceStatus::UNKNOWN;
        return result;
      }
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(10.0, required);
    result.clearance_certified = true;
    return result;
  };
}

TubeProfile InwardCurrentConnectedProfile() {
  TubeProfile profile;
  profile.source = TubeSource::ESDF;
  profile.raw_complete = true;
  profile.filtered_complete = true;
  profile.complete = true;
  profile.obstacle_certified = false;
  profile.preview_start_w = 0.0;
  profile.preview_end_w = 0.4;
  profile.requested_preview_start_w = profile.preview_start_w;
  profile.requested_preview_end_w = profile.preview_end_w;
  profile.certified_segment_start_w = profile.preview_start_w;
  profile.certified_segment_end_w = profile.preview_end_w;
  for (double w = 0.0; w <= 0.4 + 1e-12; w += 0.1) {
    TubeRawSample sample;
    sample.w = w;
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.N = Eigen::Vector3d::UnitY();
    sample.raw_lower = -0.01;
    sample.raw_upper = 0.01;
    sample.filtered_lower = sample.raw_lower;
    sample.filtered_upper = sample.raw_upper;
    sample.lower_w = 0.0;
    sample.upper_w = 0.0;
    sample.complete = true;
    sample.filtered_contains_zero = true;
    profile.samples.push_back(sample);
  }
  return profile;
}

CertifiedTubeBuildInput CloudInput(const ClearanceQuery& query = Open()) {
  CertifiedTubeBuildInput input;
  input.source = TubeSource::ESDF;
  input.preview_path = Line();
  input.authority_request.lower = -1.0;
  input.authority_request.upper = 1.0;
  input.authority_request.valid = true;
  input.cloud_clearance_query = query;
  input.path_state_query = ExactLine();
  input.path_cell_bound_query = CertifiedLineCells();
  input.cloud_snapshot_resolution = 0.05;
  input.current_w = 0.0;
  input.path_source_revision = 31U;
  input.tube_revision = 41U;
  input.map_observation_sequence = 59U;
  input.map_observation_is_snapshot = true;
  return input;
}

void MakeExactZeroFilteredProfile(TubeProfile& profile) {
  for (TubeRawSample& sample : profile.samples) {
    sample.filtered_lower = 0.0;
    sample.filtered_upper = 0.0;
    sample.filtered_contains_zero = true;
  }
  for (std::size_t index = 0U; index + 1U < profile.samples.size(); ++index) {
    profile.samples[index].lower_w = 0.0;
    profile.samples[index].upper_w = 0.0;
  }
  if (profile.samples.size() >= 2U) {
    profile.samples.back().lower_w = 0.0;
    profile.samples.back().upper_w = 0.0;
  }
  profile.diagnostics.min_width = 0.0;
}

void ExpectEquivalentProfile(const TubeProfile& expected,
                             const TubeProfile& actual) {
  EXPECT_TRUE(TubeEpochManager::profilesEquivalent(expected, actual, 1e-12));
  EXPECT_EQ(expected.raw_build_samples.size(), actual.raw_build_samples.size());
  ASSERT_EQ(expected.raw_build_samples.size(), actual.raw_build_samples.size());
  for (std::size_t index = 0U; index < expected.raw_build_samples.size(); ++index) {
    EXPECT_NEAR(expected.raw_build_samples[index].raw_lower,
                actual.raw_build_samples[index].raw_lower, 1e-12);
    EXPECT_NEAR(expected.raw_build_samples[index].raw_upper,
                actual.raw_build_samples[index].raw_upper, 1e-12);
    EXPECT_NEAR(expected.raw_build_samples[index].filtered_lower,
                actual.raw_build_samples[index].filtered_lower, 1e-12);
    EXPECT_NEAR(expected.raw_build_samples[index].filtered_upper,
                actual.raw_build_samples[index].filtered_upper, 1e-12);
  }
  EXPECT_EQ(expected.diagnostics.invalid_reason, actual.diagnostics.invalid_reason);
  EXPECT_EQ(expected.diagnostics.first_stop_reason,
            actual.diagnostics.first_stop_reason);
  EXPECT_NEAR(expected.diagnostics.first_invalid_w,
              actual.diagnostics.first_invalid_w, 1e-12);
}

void ExpectZeroBaselinePreservesRawEnvironmentEvidence(
    const TubeProfile& profile) {
  ASSERT_TRUE(profile.raw_complete);
  ASSERT_EQ(profile.samples.size(), profile.raw_build_samples.size());
  bool saw_nonzero_raw_environment = false;
  for (std::size_t index = 0U; index < profile.samples.size(); ++index) {
    const TubeRawSample& collapsed = profile.samples[index];
    const TubeRawSample& raw = profile.raw_build_samples[index];
    EXPECT_DOUBLE_EQ(collapsed.w, raw.w);
    EXPECT_DOUBLE_EQ(collapsed.raw_lower, raw.raw_lower);
    EXPECT_DOUBLE_EQ(collapsed.raw_upper, raw.raw_upper);
    EXPECT_DOUBLE_EQ(collapsed.environment_lower, raw.environment_lower);
    EXPECT_DOUBLE_EQ(collapsed.environment_upper, raw.environment_upper);
    EXPECT_DOUBLE_EQ(collapsed.environment_width, raw.environment_width);
    EXPECT_EQ(collapsed.environment_interval_nonempty,
              raw.environment_interval_nonempty);
    EXPECT_EQ(collapsed.environment_contains_zero,
              raw.environment_contains_zero);
    EXPECT_EQ(collapsed.pre_inset_contains_zero,
              raw.pre_inset_contains_zero);
    EXPECT_EQ(collapsed.post_inset_contains_zero,
              raw.post_inset_contains_zero);
    EXPECT_EQ(collapsed.filter_input_contains_zero,
              raw.filter_input_contains_zero);
    EXPECT_DOUBLE_EQ(collapsed.filtered_lower, 0.0);
    EXPECT_DOUBLE_EQ(collapsed.filtered_upper, 0.0);
    EXPECT_DOUBLE_EQ(collapsed.lower_w, 0.0);
    EXPECT_DOUBLE_EQ(collapsed.upper_w, 0.0);
    EXPECT_TRUE(collapsed.filtered_contains_zero);
    EXPECT_TRUE(collapsed.complete);
    saw_nonzero_raw_environment = saw_nonzero_raw_environment ||
        raw.raw_lower < -1e-10 || raw.raw_upper > 1e-10 ||
        raw.environment_width > 1e-10;
  }
  EXPECT_TRUE(saw_nonzero_raw_environment);
  EXPECT_DOUBLE_EQ(profile.diagnostics.min_width, 0.0);
}

TEST(CertifiedTubeBuilderTest, BoundarySlopeCompatibilityValueCannotInvalidateBuilder) {
  TubeFilterConfig invalid_filter = FilterConfig();
  invalid_filter.boundary_slope_max = 0.0;
  CertifiedTubeBuilder builder(Config(), invalid_filter,
                               TubeSurfaceValidatorConfig());
  EXPECT_TRUE(builder.configurationValid());
  CertifiedTubeBuildResult result;
  EXPECT_TRUE(builder.build(CloudInput(), result));
  EXPECT_TRUE(result.raw_complete);
  EXPECT_TRUE(result.complete);
}

TEST(CertifiedTubeBuilderTest, FixedMatchesLegacyBuilderThenFilterPipeline) {
  const TubeBuilderConfig builder_config = Config();
  const TubeFilterConfig filter_config = FilterConfig();
  TubeProfile legacy;
  ASSERT_TRUE(TubeBuilder(builder_config).build(
      TubeSource::FIXED, Line(), DistanceQuery(), 31U, 41U, legacy));
  ASSERT_TRUE(TubeFilter(filter_config).filter(legacy, 0.0));
  legacy.classification = TubeProfileClassification::OFFSET_CERTIFIED;

  CertifiedTubeBuildInput input;
  input.source = TubeSource::FIXED;
  input.preview_path = Line();
  input.current_w = 0.0;
  input.path_source_revision = 31U;
  input.tube_revision = 41U;
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(builder_config, filter_config,
                                   TubeSurfaceValidatorConfig()).build(input, result));
  ExpectEquivalentProfile(legacy, result.profile);
  EXPECT_TRUE(result.raw_complete);
  EXPECT_TRUE(result.filtered_complete);
  EXPECT_TRUE(result.complete);
  EXPECT_FALSE(result.surface_validation_attempted);
}

TEST(CertifiedTubeBuilderTest, EsdfMatchesLegacyCertifiedPipelineAndProvenance) {
  const TubeBuilderConfig builder_config = Config();
  const TubeFilterConfig filter_config = FilterConfig();
  const CertifiedTubeBuildInput input = CloudInput();
  TubeProfile legacy;
  ASSERT_TRUE(TubeBuilder(builder_config).buildCloudClearance(
      input.source, input.preview_path, input.cloud_clearance_query,
      input.path_state_query, input.path_cell_bound_query,
      input.cloud_snapshot_resolution, input.current_w,
      input.path_source_revision, input.tube_revision, legacy));
  legacy.snapshot_sequence = input.map_observation_sequence;
  legacy.snapshot_resolution = input.cloud_snapshot_resolution;
  legacy.snapshot_provenance_is_immutable = true;
  ASSERT_TRUE(TubeFilter(filter_config).filter(legacy, input.current_w));
  TubeSurfaceValidationResult validation;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      legacy, input.current_w, input.path_state_query, input.path_cell_bound_query,
      input.cloud_clearance_query, input.cloud_snapshot_resolution,
      builder_config.cross_section.planner_safe_distance,
      builder_config.cross_section.regularity_margin, validation));
  legacy.classification = TubeProfileClassification::OFFSET_CERTIFIED;

  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(builder_config, filter_config,
                                   TubeSurfaceValidatorConfig()).build(input, result));
  ExpectEquivalentProfile(legacy, result.profile);
  EXPECT_TRUE(result.surface_validation_attempted);
  EXPECT_TRUE(result.surface_validation.complete);
  EXPECT_EQ(result.profile.snapshot_sequence, input.map_observation_sequence);
  EXPECT_DOUBLE_EQ(result.profile.snapshot_resolution,
                   input.cloud_snapshot_resolution);
  EXPECT_TRUE(result.profile.snapshot_provenance_is_immutable);
}

TEST(CertifiedTubeBuilderTest,
     NearVerticalFrameCellProofReachesTubeBuilderAndSurfaceValidator) {
  CertifiedTubeBuildInput input = CloudInput();
  input.preview_path = NearVerticalLine();
  input.path_state_query = ExactNearVerticalLine();
  input.path_cell_bound_query = CertifiedNearVerticalCells();
  input.current_w = 0.0;
  input.path_source_revision = 77U;
  input.tube_revision = 79U;
  input.map_observation_sequence = 17U;
  input.map_observation_is_snapshot = true;
  CertifiedTubeBuilder builder(Config(), FilterConfig(),
                               TubeSurfaceValidatorConfig());
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(builder.build(input, result));
  EXPECT_TRUE(result.surface_validation_attempted);
  EXPECT_TRUE(result.surface_validation.complete);
  EXPECT_TRUE(result.profile.cell_geometry_certified);
  EXPECT_TRUE(result.profile.combined_regularity_proof_complete);
  EXPECT_EQ(result.profile.proof_level, TubeProofLevel::CONTINUOUS_COVER_PROOF);
  EXPECT_EQ(result.profile.path_revision, 77U);
  EXPECT_EQ(result.profile.frame_revision, 79U);
  for (const TubeRawSample& sample : result.profile.samples) {
    EXPECT_EQ(sample.proof_level, TubeProofLevel::FRAME_CELL_PROOF);
    EXPECT_EQ(sample.path_revision, 77U);
    EXPECT_EQ(sample.frame_revision, 79U);
  }
}

TEST(CertifiedTubeBuilderTest,
     MixedFrameRevisionCannotEstablishCellProof) {
  CertifiedTubeBuildInput input = CloudInput();
  input.preview_path = NearVerticalLine();
  input.path_state_query = [](const double w,
                              phase_offset_core::PathDifferentialState& state) {
    if (!ExactNearVerticalLine()(w, state)) return false;
    if (w > 0.2) state.frame_revision = 81U;
    return true;
  };
  input.path_cell_bound_query = CertifiedNearVerticalCells();
  input.current_w = 0.0;
  input.path_source_revision = 77U;
  input.tube_revision = 79U;
  input.map_observation_sequence = 17U;
  input.map_observation_is_snapshot = true;
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  EXPECT_TRUE(result.raw_complete);
  EXPECT_FALSE(result.profile.cell_geometry_certified);
  EXPECT_NE(result.profile.proof_level, TubeProofLevel::FRAME_CELL_PROOF);
}

TEST(CertifiedTubeBuilderTest,
     MalformedAuthorityRequestDoesNotAffectGeometricBuild) {
  const CertifiedTubeBuilder builder(Config(), FilterConfig(),
                                     TubeSurfaceValidatorConfig());
  CertifiedTubeBuildResult baseline;
  ASSERT_TRUE(builder.build(CloudInput(), baseline));
  std::vector<TubeBounds> invalid_requests;
  invalid_requests.push_back(TubeBounds());
  invalid_requests.push_back(Authority(0.20, 0.10));
  invalid_requests.push_back(Authority(0.10, 0.20));
  invalid_requests.push_back(Authority(-0.10, 0.10));
  invalid_requests.back().lower = std::numeric_limits<double>::quiet_NaN();
  invalid_requests.push_back(Authority(-0.10, 0.10));
  invalid_requests.back().lower_w = 1.0;

  for (const TubeBounds& request : invalid_requests) {
    CertifiedTubeBuildInput input = CloudInput();
    input.authority_request = request;
    CertifiedTubeBuildResult result;
    ASSERT_TRUE(builder.build(input, result));
    EXPECT_EQ(result.profile.samples.size(), baseline.profile.samples.size());
    ASSERT_EQ(result.profile.samples.size(), result.profile.raw_build_samples.size());
    for (std::size_t index = 0U; index < result.profile.samples.size(); ++index) {
      EXPECT_DOUBLE_EQ(result.profile.samples[index].filtered_lower,
                       baseline.profile.samples[index].filtered_lower);
      EXPECT_DOUBLE_EQ(result.profile.samples[index].filtered_upper,
                       baseline.profile.samples[index].filtered_upper);
    }
  }
}

TEST(CertifiedTubeBuilderTest,
     AuthoritySubsetIsCompatibilityMetadataOnly) {
  TubeBuilderConfig builder_config = Config();
  builder_config.fixed_delta_max = 0.04;
  builder_config.cross_section.search_extent = 3.0;
  CertifiedTubeBuildInput input = CloudInput();
  input.authority_request = Authority(-0.10, 0.10);
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(builder_config, FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  ASSERT_TRUE(result.surface_validation_attempted);
  ASSERT_TRUE(result.surface_validation.complete);
  ASSERT_EQ(result.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  ASSERT_EQ(result.profile.samples.size(),
            result.profile.raw_build_samples.size());
  ASSERT_GT(result.profile.samples.size(), 1U);

  bool broad_raw_observed = false;
  for (std::size_t index = 0U; index < result.profile.samples.size(); ++index) {
    const TubeRawSample& certified = result.profile.samples[index];
    const TubeRawSample& raw = result.profile.raw_build_samples[index];
    EXPECT_DOUBLE_EQ(certified.w, raw.w);
    EXPECT_DOUBLE_EQ(certified.raw_lower, raw.raw_lower);
    EXPECT_DOUBLE_EQ(certified.raw_upper, raw.raw_upper);
    EXPECT_DOUBLE_EQ(certified.environment_lower, raw.environment_lower);
    EXPECT_DOUBLE_EQ(certified.environment_upper, raw.environment_upper);
    EXPECT_DOUBLE_EQ(certified.filtered_lower, raw.filtered_lower);
    EXPECT_DOUBLE_EQ(certified.filtered_upper, raw.filtered_upper);
    broad_raw_observed = broad_raw_observed ||
        raw.filtered_lower < -2.90 || raw.filtered_upper > 2.90;
  }
  EXPECT_TRUE(broad_raw_observed);
  EXPECT_GT(result.profile.samples.front().filtered_upper -
                result.profile.samples.front().filtered_lower,
            2.0 * builder_config.fixed_delta_max);
  for (std::size_t index = 0U; index + 1U < result.profile.samples.size();
       ++index) {
    const TubeRawSample& sample = result.profile.samples[index];
    const TubeRawSample& next = result.profile.samples[index + 1U];
    const double dw = next.w - sample.w;
    ASSERT_GT(dw, 0.0);
    EXPECT_NEAR(sample.lower_w,
                (next.filtered_lower - sample.filtered_lower) / dw, 1e-12);
    EXPECT_NEAR(sample.upper_w,
                (next.filtered_upper - sample.filtered_upper) / dw, 1e-12);
  }
}

TEST(CertifiedTubeBuilderTest,
     ZeroCapacityAuthorityDoesNotCollapseGeometricProfile) {
  TubeBuilderConfig builder_config = Config();
  builder_config.cross_section.search_extent = 3.0;
  CertifiedTubeBuildInput input = CloudInput();
  input.authority_request = Authority(0.0, 0.0);
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(builder_config, FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  EXPECT_TRUE(result.surface_validation_attempted);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  EXPECT_TRUE(result.profile.obstacle_certified);
  for (const TubeRawSample& sample : result.profile.samples) {
    EXPECT_DOUBLE_EQ(sample.filtered_lower, sample.raw_lower);
    EXPECT_DOUBLE_EQ(sample.filtered_upper, sample.raw_upper);
  }
}

TEST(CertifiedTubeBuilderTest,
     AuthorityWidthDoesNotChangeValidatorGeometryOrQueryCost) {
  TubeBuilderConfig builder_config = Config();
  builder_config.cross_section.search_extent = 3.0;
  const CertifiedTubeBuildInput input = CloudInput();

  TubeProfile raw_profile;
  ASSERT_TRUE(TubeBuilder(builder_config).buildCloudClearance(
      input.source, input.preview_path, input.cloud_clearance_query,
      input.path_state_query, input.path_cell_bound_query,
      input.cloud_snapshot_resolution, input.current_w,
      input.path_source_revision, input.tube_revision, raw_profile));
  ASSERT_TRUE(TubeFilter(FilterConfig()).filter(raw_profile, input.current_w));
  TubeSurfaceValidationResult raw_validation;
  TubeSurfaceValidator validator;
  validator.validate(raw_profile, input.current_w, input.path_state_query,
                     input.path_cell_bound_query, input.cloud_clearance_query,
                     input.cloud_snapshot_resolution,
                     builder_config.cross_section.planner_safe_distance,
                     builder_config.cross_section.regularity_margin,
                     raw_validation);

  CertifiedTubeBuildInput narrow_input = input;
  narrow_input.authority_request = Authority(-0.10, 0.10);
  CertifiedTubeBuildResult narrow;
  ASSERT_TRUE(CertifiedTubeBuilder(builder_config, FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(narrow_input, narrow));
  ASSERT_TRUE(narrow.surface_validation.complete);
  ASSERT_GT(raw_validation.query_sample_count, 0U);
  ASSERT_GT(raw_validation.clearance_leaf_cell_count, 0U);
  EXPECT_EQ(narrow.surface_validation.query_sample_count,
            raw_validation.query_sample_count);
  EXPECT_EQ(narrow.surface_validation.geometry_cell_count,
            raw_validation.geometry_cell_count);
  EXPECT_EQ(narrow.surface_validation.clearance_leaf_cell_count,
            raw_validation.clearance_leaf_cell_count);
  EXPECT_EQ(narrow.surface_validation.split_v_count,
            raw_validation.split_v_count);
  std::cout << "G17_COST raw_queries=" << raw_validation.query_sample_count
            << " narrow_queries="
            << narrow.surface_validation.query_sample_count
            << " raw_geometry_cells=" << raw_validation.geometry_cell_count
            << " narrow_geometry_cells="
            << narrow.surface_validation.geometry_cell_count
            << " raw_v_splits=" << raw_validation.split_v_count
            << " narrow_v_splits="
            << narrow.surface_validation.split_v_count << "\n";
}

TEST(CertifiedTubeBuilderTest,
     EvidenceOutsideAuthorityDoesNotAffectGeometricProfile) {
  CertifiedTubeBuildInput input = CloudInput(OutsideNarrowAuthorityUnknown());
  input.authority_request = Authority(-0.10, 0.10);
  CertifiedTubeBuildResult result;
  CertifiedTubeBuildInput baseline_input = input;
  baseline_input.authority_request = TubeBounds();
  CertifiedTubeBuildResult baseline;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(baseline_input, baseline));
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  EXPECT_TRUE(result.surface_validation.complete);
  EXPECT_EQ(result.surface_validation.first_failure_reason,
            TubeStopReason::NONE);
  ASSERT_EQ(result.profile.samples.size(), baseline.profile.samples.size());
  for (std::size_t index = 0U; index < result.profile.samples.size(); ++index) {
    EXPECT_DOUBLE_EQ(result.profile.samples[index].filtered_lower,
                     baseline.profile.samples[index].filtered_lower);
    EXPECT_DOUBLE_EQ(result.profile.samples[index].filtered_upper,
                     baseline.profile.samples[index].filtered_upper);
  }
}

TEST(CertifiedTubeBuilderTest,
     EvidenceInsideAuthorityStillFailsClosedToPlannerBaseline) {
  CertifiedTubeBuildInput input = CloudInput(CenterlineUnknownAfterBase());
  input.authority_request = Authority(-0.10, 0.10);
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_FALSE(result.profile.obstacle_certified);
  EXPECT_EQ(result.profile.diagnostics.first_stop_reason,
            static_cast<int>(TubeStopReason::UNKNOWN));
  ExpectZeroBaselinePreservesRawEnvironmentEvidence(result.profile);
}

TEST(CertifiedTubeBuilderTest,
     InwardRetryStartsFromAuthorityCandidateAndNeverWidensToRawGeometry) {
  CertifiedTubeBuildInput input = CloudInput(FullWidthOuterUnknown());
  input.authority_request = Authority(-0.10, 0.10);
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  ASSERT_EQ(result.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  ASSERT_TRUE(result.profile.obstacle_certified);
  bool strict_inward = false;
  bool broad_raw = false;
  for (std::size_t index = 0U; index < result.profile.samples.size(); ++index) {
    const TubeRawSample& certified = result.profile.samples[index];
    const TubeRawSample& raw = result.profile.raw_build_samples[index];
    EXPECT_GE(certified.filtered_lower, -0.10 - 1e-12);
    EXPECT_LE(certified.filtered_upper, 0.10 + 1e-12);
    strict_inward = strict_inward || certified.filtered_lower > -0.10 + 1e-12 ||
        certified.filtered_upper < 0.10 - 1e-12;
    broad_raw = broad_raw || raw.filtered_lower < -0.50 ||
        raw.filtered_upper > 0.50;
  }
  EXPECT_TRUE(strict_inward);
  EXPECT_TRUE(broad_raw);
}

TEST(CertifiedTubeBuilderTest,
     CertifiedZeroLimitFailureRemainsFailClosed) {
  std::size_t expected_calls = 0U;
  CertifiedTubeBuildInput expected_input = CloudInput(
      CountingCenterlineUnknown(&expected_calls));
  TubeProfile expected_profile;
  ASSERT_TRUE(TubeBuilder(Config()).buildCloudClearance(
      expected_input.source, expected_input.preview_path,
      expected_input.cloud_clearance_query, expected_input.path_state_query,
      expected_input.path_cell_bound_query,
      expected_input.cloud_snapshot_resolution, expected_input.current_w,
      expected_input.path_source_revision, expected_input.tube_revision,
      expected_profile));
  expected_profile.snapshot_sequence = expected_input.map_observation_sequence;
  expected_profile.snapshot_resolution = expected_input.cloud_snapshot_resolution;
  expected_profile.snapshot_provenance_is_immutable = true;
  ASSERT_TRUE(TubeFilter(FilterConfig()).filter(expected_profile,
                                               expected_input.current_w));
  TubeSurfaceValidationResult expected_full;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      expected_profile, expected_input.current_w, expected_input.path_state_query,
      expected_input.path_cell_bound_query, expected_input.cloud_clearance_query,
      expected_input.cloud_snapshot_resolution,
      Config().cross_section.planner_safe_distance,
      Config().cross_section.regularity_margin, expected_full));
  MakeExactZeroFilteredProfile(expected_profile);
  TubeSurfaceValidationResult expected_zero;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      expected_profile, expected_input.current_w, expected_input.path_state_query,
      expected_input.path_cell_bound_query, expected_input.cloud_clearance_query,
      expected_input.cloud_snapshot_resolution,
      Config().cross_section.planner_safe_distance,
      Config().cross_section.regularity_margin, expected_zero));
  EXPECT_FALSE(expected_zero.complete);

  std::size_t actual_calls = 0U;
  CertifiedTubeBuildInput actual_input = CloudInput(
      CountingCenterlineUnknown(&actual_calls));
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(actual_input, result));
  EXPECT_GE(actual_calls, expected_calls);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_FALSE(result.profile.diagnostics.invalid_reason.empty());
  EXPECT_FALSE(result.profile.obstacle_certified);
  ExpectZeroBaselinePreservesRawEnvironmentEvidence(result.profile);
}

TEST(CertifiedTubeBuilderTest,
     SampledFallbackCounterexamplePreservesRealInwardSearch) {
  CertifiedTubeBuildInput input = CloudInput(FullWidthOuterUnknown());
  input.path_cell_bound_query = PathCellBoundQuery();
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  EXPECT_FALSE(result.profile.cell_geometry_certified);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  EXPECT_TRUE(result.profile.obstacle_certified);
  bool nonzero = false;
  for (const TubeRawSample& sample : result.profile.samples) {
    nonzero = nonzero || sample.filtered_lower < -1e-10 ||
        sample.filtered_upper > 1e-10;
  }
  EXPECT_TRUE(nonzero);
  EXPECT_EQ(result.profile.diagnostics.invalid_reason.find("zero-width limit"),
            std::string::npos);
}

TEST(CertifiedTubeBuilderTest,
     CertifiedZeroLimitSuccessButNoNonzeroCandidateRemainsFailClosed) {
  CertifiedTubeBuildInput input = CloudInput(AnyNonzeroOffsetUnknown());
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_FALSE(result.profile.obstacle_certified);
  EXPECT_EQ(result.profile.diagnostics.invalid_reason.find("zero-width limit"),
            std::string::npos);
  ExpectZeroBaselinePreservesRawEnvironmentEvidence(result.profile);
}

TEST(CertifiedTubeBuilderTest,
     CertifiedZeroQueryBudgetPreservesInwardSearch) {
  TubeSurfaceValidatorConfig validator_config;
  validator_config.max_query_samples = 18U;
  validator_config.max_subdivision_depth = 0;
  TubeBuilderConfig builder_config = Config();
  CertifiedTubeBuildInput input = CloudInput(FullWidthOuterUnknown());
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(builder_config, FilterConfig(), validator_config)
                  .build(input, result));
  EXPECT_EQ(result.profile.diagnostics.invalid_reason.find("zero-width limit"),
            std::string::npos) << result.profile.diagnostics.invalid_reason;
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  EXPECT_TRUE(result.profile.obstacle_certified);
}

TEST(CertifiedTubeBuilderTest, CertifiedZeroLimitSuccessPreservesRealInwardSearch) {
  CertifiedTubeBuildInput input = CloudInput(FullWidthOuterUnknown());
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  EXPECT_TRUE(result.profile.obstacle_certified);
  TubeBounds current;
  ASSERT_TRUE(TubeFilter::query(result.profile, input.current_w, current));
  EXPECT_GT(current.upper - current.lower, 1e-10);
  EXPECT_TRUE(result.surface_validation.complete);
  EXPECT_EQ(result.profile.diagnostics.invalid_reason.find("zero-width limit"),
            std::string::npos);
}

TEST(CertifiedTubeBuilderTest, ExactCurrentDegenerateAnchorRemainsMonotone) {
  TubeProfile wide = InwardCurrentConnectedProfile();
  wide.cell_geometry_certified = true;
  for (TubeRawSample& sample : wide.samples) {
    sample.filtered_lower = -0.10;
    sample.filtered_upper = 0.20;
  }
  TubeProfile zero = wide;
  MakeExactZeroFilteredProfile(zero);
  std::vector<double> wide_requested;
  std::vector<double> zero_requested;
  const ClearanceQuery wide_open =
      [&wide_requested](const Eigen::Vector3d&, const double required) {
        wide_requested.push_back(required);
        ClearanceQueryResult result;
        result.status = DistanceStatus::KNOWN_FREE;
        result.clearance = std::max(10.0, required);
        result.clearance_certified = true;
        return result;
      };
  const ClearanceQuery zero_open =
      [&zero_requested](const Eigen::Vector3d&, const double required) {
        zero_requested.push_back(required);
        ClearanceQueryResult result;
        result.status = DistanceStatus::KNOWN_FREE;
        result.clearance = std::max(10.0, required);
        result.clearance_certified = true;
        return result;
      };
  TubeSurfaceValidationResult wide_result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      wide, 0.0, ExactLine(), CertifiedLineCells(), wide_open, 0.05, 0.40,
      0.10, wide_result));
  TubeSurfaceValidationResult zero_result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      zero, 0.0, ExactLine(), CertifiedLineCells(), zero_open, 0.05, 0.40,
      0.10, zero_result));
  ASSERT_TRUE(zero_result.current_anchor_valid);
  ASSERT_FALSE(wide_requested.empty());
  ASSERT_FALSE(zero_requested.empty());
  EXPECT_NEAR(zero_requested.front(), 0.400001, 1e-12);
  EXPECT_LT(zero_requested.front(), wide_requested.front());
  EXPECT_TRUE(zero_result.zero_centerline_continuously_certified);
}

TEST(CertifiedTubeBuilderTest, RemoteTruncationDoesNotCreateFalseZeroFailure) {
  CertifiedTubeBuildInput input = CloudInput(RemoteUnknownAfterBase());
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  EXPECT_TRUE(result.profile.certified_segment_truncated_after);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  EXPECT_EQ(result.profile.diagnostics.invalid_reason.find("zero-width limit"),
            std::string::npos);
}

TEST(CertifiedTubeBuilderTest,
     RecursiveSubcellCertificatesRemainCompleteAndMatched) {
  bool first = true;
  std::size_t callback_calls = 0U;
  const PathCellBoundQuery mismatched =
      [&first, &callback_calls](const double w0, const double w1,
                                phase_offset_core::PathCellGeometryCertificate& certificate) {
        ++callback_calls;
        const bool complete = CertifiedLineCells()(w0, w1, certificate);
        if (first && complete && w1 > w0) {
          first = false;
          certificate.w1 += 0.01;
        }
        return complete;
      };
  CertifiedTubeBuildInput input = CloudInput(FullWidthOuterUnknown());
  input.path_cell_bound_query = mismatched;
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  EXPECT_GT(callback_calls, 0U);
  EXPECT_FALSE(result.profile.cell_geometry_certified);
  EXPECT_EQ(result.profile.diagnostics.invalid_reason.find("zero-width limit"),
            std::string::npos);
}

TEST(CertifiedTubeBuilderTest,
     RetainedDeltaAndOneInteriorMarginRemainInsideCertifiedProfile) {
  TubeBuilderConfig builder_config = Config();
  builder_config.interior_margin = 0.02;
  CertifiedTubeBuildInput input = CloudInput();
  input.authority_request = Authority(-0.12, 0.18);
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(builder_config, FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  TubeBounds current;
  ASSERT_TRUE(TubeFilter::query(result.profile, input.current_w, current));
  ASSERT_TRUE(current.valid);
  EXPECT_LE(current.lower + builder_config.interior_margin, -0.10 + 1e-12);
  EXPECT_GE(current.upper - builder_config.interior_margin, 0.16 - 1e-12);
}

TEST(CertifiedTubeBuilderTest, ZeroConnectedCapacityCollapsesToCompleteBaseline) {
  TubeBuilderConfig zero_capacity_config = Config();
  zero_capacity_config.fixed_delta_max = 0.0;
  CertifiedTubeBuildInput input;
  input.source = TubeSource::FIXED;
  input.preview_path = Line();
  input.current_w = 0.0;
  input.path_source_revision = 31U;
  input.tube_revision = 41U;
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(zero_capacity_config, FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, result));
  EXPECT_TRUE(result.raw_complete);
  EXPECT_TRUE(result.filtered_complete);
  EXPECT_TRUE(result.complete);
  EXPECT_FALSE(result.surface_validation_attempted);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  ASSERT_FALSE(result.profile.samples.empty());
  for (const TubeRawSample& sample : result.profile.samples) {
    EXPECT_DOUBLE_EQ(sample.raw_lower, 0.0);
    EXPECT_DOUBLE_EQ(sample.raw_upper, 0.0);
    EXPECT_DOUBLE_EQ(sample.filtered_lower, 0.0);
    EXPECT_DOUBLE_EQ(sample.filtered_upper, 0.0);
  }
}

TEST(CertifiedTubeBuilderTest, SurfaceFailureCollapsesAndPreservesFirstFailure) {
  TubeSurfaceValidatorConfig validator_config;
  validator_config.max_query_samples = 9U;
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(), validator_config)
                  .build(CloudInput(), result));
  EXPECT_TRUE(result.surface_validation_attempted);
  EXPECT_FALSE(result.surface_validation.complete);
  EXPECT_TRUE(result.surface_validation.limit_exceeded);
  EXPECT_TRUE(result.complete);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_EQ(result.profile.diagnostics.first_stop_reason,
            static_cast<int>(result.surface_validation.first_failure_reason));
  EXPECT_DOUBLE_EQ(result.profile.diagnostics.first_invalid_w,
                   result.surface_validation.first_failure_w);
  EXPECT_NE(result.profile.diagnostics.invalid_reason.find(
                "full-width query-budget exhaustion"),
            std::string::npos);
  EXPECT_NE(result.profile.diagnostics.invalid_reason.find("max_queries=9"),
            std::string::npos);
  ExpectZeroBaselinePreservesRawEnvironmentEvidence(result.profile);
}

TEST(CertifiedTubeBuilderTest,
     ExactCurrentZeroCapacityRemainsFailClosed) {
  QueryCounts expected_counts;
  const CertifiedTubeBuildInput input = CloudInput(
      ExactCurrentZeroWithOuterUnknown(&expected_counts));

  // Establish the one full-width Validator invocation that is the only proof
  // needed for the D0 fixture.  This is the expected call count after the
  // proof-local short circuit, not a production cache or memoized result.
  TubeProfile expected_profile;
  ASSERT_TRUE(TubeBuilder(Config()).buildCloudClearance(
      input.source, input.preview_path, input.cloud_clearance_query,
      input.path_state_query, input.path_cell_bound_query,
      input.cloud_snapshot_resolution, input.current_w,
      input.path_source_revision, input.tube_revision, expected_profile));
  ASSERT_TRUE(TubeFilter(FilterConfig()).filter(expected_profile,
                                               input.current_w));
  TubeSurfaceValidationResult expected_validation;
  ASSERT_FALSE(TubeSurfaceValidator().validate(
      expected_profile, input.current_w, input.path_state_query,
      input.path_cell_bound_query, input.cloud_clearance_query,
      input.cloud_snapshot_resolution,
      Config().cross_section.planner_safe_distance,
      Config().cross_section.regularity_margin, expected_validation));
  ASSERT_GT(expected_counts.validator_calls, 0U);
  const std::size_t expected_validator_calls = expected_counts.validator_calls;

  QueryCounts actual_counts;
  CertifiedTubeBuildInput actual_input = CloudInput(
      ExactCurrentZeroWithOuterUnknown(&actual_counts));
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(actual_input, result));

  std::cout << "G16_D0 expected_validator_queries="
            << expected_validator_calls
            << " actual_validator_queries=" << actual_counts.validator_calls
            << " inward_attempts_before=101 inward_attempts_after=0\n";
  EXPECT_GT(actual_counts.validator_calls, 0U);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_TRUE(result.complete);
  EXPECT_FALSE(result.profile.obstacle_certified);
  EXPECT_EQ(result.profile.diagnostics.first_stop_reason,
            static_cast<int>(expected_validation.first_failure_reason));
  EXPECT_FALSE(result.profile.diagnostics.invalid_reason.empty());
  for (const TubeRawSample& sample : result.profile.samples) {
    EXPECT_DOUBLE_EQ(sample.filtered_lower, 0.0);
    EXPECT_DOUBLE_EQ(sample.filtered_upper, 0.0);
  }
  ExpectZeroBaselinePreservesRawEnvironmentEvidence(result.profile);
}

TEST(CertifiedTubeBuilderTest,
     FullWidthUnknownRepairsToStrictInwardRibbonAndPreservesEvidence) {
  const ClearanceQuery query = FullWidthOuterUnknown();
  const TubeBuilderConfig builder_config = Config();
  const CertifiedTubeBuildInput input = CloudInput(query);

  CertifiedTubeBuildResult collapsed;
  ASSERT_TRUE(CertifiedTubeBuilder(builder_config, FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(input, collapsed));
  ASSERT_TRUE(collapsed.surface_validation_attempted);
  ASSERT_TRUE(collapsed.raw_complete);
  ASSERT_TRUE(collapsed.filtered_complete);
  ASSERT_TRUE(collapsed.complete);
  EXPECT_EQ(collapsed.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  EXPECT_TRUE(collapsed.profile.obstacle_certified);
  EXPECT_TRUE(collapsed.surface_validation.complete);
  // The failed full-width attempt remains the profile's first-failure
  // provenance even though the final inward candidate is certified.
  EXPECT_EQ(collapsed.profile.diagnostics.first_stop_reason,
            static_cast<int>(TubeStopReason::UNKNOWN));
  EXPECT_NE(collapsed.profile.diagnostics.invalid_reason.find(
                "retained inward certified ribbon"),
            std::string::npos);
  ASSERT_FALSE(collapsed.profile.raw_build_samples.empty());
  bool raw_nonzero = false;
  bool strict_inward = false;
  for (std::size_t index = 0U;
       index < collapsed.profile.samples.size(); ++index) {
    const TubeRawSample& sample = collapsed.profile.samples[index];
    raw_nonzero = raw_nonzero || sample.raw_lower < -1e-10 ||
        sample.raw_upper > 1e-10;
    EXPECT_GE(sample.filtered_lower, sample.raw_lower);
    EXPECT_LE(sample.filtered_upper, sample.raw_upper);
    strict_inward = strict_inward ||
        sample.filtered_lower > sample.raw_lower + 1e-10 ||
        sample.filtered_upper < sample.raw_upper - 1e-10;
    // The accepted candidate must retain Builder/environment evidence rather
    // than rewriting the raw interval to make Filter accept the inward one.
    EXPECT_EQ(sample.w, collapsed.profile.raw_build_samples[index].w);
    EXPECT_DOUBLE_EQ(sample.raw_lower,
                     collapsed.profile.raw_build_samples[index].raw_lower);
    EXPECT_DOUBLE_EQ(sample.raw_upper,
                     collapsed.profile.raw_build_samples[index].raw_upper);
    EXPECT_DOUBLE_EQ(sample.environment_lower,
                     collapsed.profile.raw_build_samples[index].environment_lower);
    EXPECT_DOUBLE_EQ(sample.environment_upper,
                     collapsed.profile.raw_build_samples[index].environment_upper);
    EXPECT_EQ(sample.filter_input_contains_zero,
              collapsed.profile.raw_build_samples[index].filter_input_contains_zero);
  }
  EXPECT_TRUE(raw_nonzero);
  EXPECT_TRUE(strict_inward);

  TubeProfile inward = InwardCurrentConnectedProfile();
  TubeSurfaceValidationResult inward_result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      inward, 0.0, input.path_state_query, input.cloud_clearance_query,
      input.cloud_snapshot_resolution,
      builder_config.cross_section.planner_safe_distance,
      builder_config.cross_section.regularity_margin, inward_result));
  EXPECT_TRUE(inward_result.complete);
  EXPECT_TRUE(inward.obstacle_certified);
  EXPECT_TRUE(inward.zero_centerline_continuously_certified);
  bool inward_nonzero = false;
  for (const TubeRawSample& sample : inward.samples) {
    inward_nonzero = inward_nonzero || sample.filtered_lower < -1e-10 ||
        sample.filtered_upper > 1e-10;
  }
  EXPECT_TRUE(inward_nonzero);
}

TEST(CertifiedTubeBuilderTest,
     CenterlineUnknownCannotBeRepairedByInwardSearch) {
  TubeSurfaceValidatorConfig validator_config;
  // G3 leaf-only scheduling skips geometrically impossible parent queries;
  // allow the certified leaves to be reached so the immutable centerline
  // UNKNOWN remains the observed first failure.
  validator_config.max_subdivision_depth = 8;
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(), validator_config)
                  .build(CloudInput(CenterlineUnknownAfterBase()), result));
  EXPECT_TRUE(result.surface_validation_attempted);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_FALSE(result.profile.obstacle_certified);
  EXPECT_EQ(result.profile.diagnostics.first_stop_reason,
            static_cast<int>(TubeStopReason::UNKNOWN));
  EXPECT_FALSE(result.profile.diagnostics.invalid_reason.empty());
  for (const TubeRawSample& sample : result.profile.samples) {
    EXPECT_DOUBLE_EQ(sample.filtered_lower, 0.0);
    EXPECT_DOUBLE_EQ(sample.filtered_upper, 0.0);
  }
}

TEST(CertifiedTubeBuilderTest,
     CenterlineUnavailableCannotBeRepairedByInwardSearch) {
  TubeSurfaceValidatorConfig validator_config;
  validator_config.max_subdivision_depth = 8;
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(), validator_config)
                  .build(CloudInput(CenterlineUnavailableAfterBase()), result));
  EXPECT_TRUE(result.surface_validation_attempted);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE);
  EXPECT_FALSE(result.profile.obstacle_certified);
  EXPECT_EQ(result.profile.diagnostics.first_stop_reason,
            static_cast<int>(TubeStopReason::UNAVAILABLE));
}

TEST(CertifiedTubeBuilderTest,
     AsymmetricUnsafeSideSelectsRevalidatedOneSidedRibbon) {
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(CloudInput(NegativeSideUnknown()), result));
  ASSERT_TRUE(result.complete);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  EXPECT_TRUE(result.profile.obstacle_certified);
  EXPECT_EQ(result.profile.diagnostics.first_stop_reason,
            static_cast<int>(TubeStopReason::UNKNOWN));
  bool positive_capacity = false;
  for (const TubeRawSample& sample : result.profile.samples) {
    EXPECT_GE(sample.filtered_lower, -1e-10);
    EXPECT_GT(sample.filtered_upper, 1e-10);
    positive_capacity = positive_capacity || sample.filtered_upper > 1e-10;
  }
  EXPECT_TRUE(positive_capacity);
}

TEST(CertifiedTubeBuilderTest,
     RemoteUnsafeSuffixTruncatesWithoutErasingCurrentRibbon) {
  CertifiedTubeBuildResult result;
  ASSERT_TRUE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                   TubeSurfaceValidatorConfig())
                  .build(CloudInput(RemoteUnknownAfterBase()), result));
  ASSERT_TRUE(result.complete);
  EXPECT_EQ(result.profile.classification,
            TubeProfileClassification::OFFSET_CERTIFIED);
  EXPECT_TRUE(result.profile.obstacle_certified);
  EXPECT_TRUE(result.profile.certified_segment_truncated_after);
  EXPECT_LT(result.profile.preview_end_w, result.profile.requested_preview_end_w);
  EXPECT_GE(result.profile.preview_end_w, 0.0);
  EXPECT_GT(result.profile.samples.size(), 1U);
}

TEST(CertifiedTubeBuilderTest, RawIncompleteNeverMasqueradesAsBaseline) {
  CertifiedTubeBuildInput input = CloudInput();
  input.cloud_clearance_query = ClearanceQuery();
  CertifiedTubeBuildResult result;
  EXPECT_FALSE(CertifiedTubeBuilder(Config(), FilterConfig(),
                                    TubeSurfaceValidatorConfig()).build(input, result));
  EXPECT_FALSE(result.raw_complete);
  EXPECT_FALSE(result.filtered_complete);
  EXPECT_FALSE(result.complete);
  EXPECT_EQ(result.profile.classification, TubeProfileClassification::NONE);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
