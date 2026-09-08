#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "bspline_race/integration/phase_offset_clearance_audit.h"
#include "bspline_race/integration/phase_offset_cloud_occupancy_query.h"
#include "bspline_race/integration/phase_offset_matched_adapter.h"
#include "phase_offset_navigation/tube_builder.h"
#include "phase_offset_navigation/tube_filter.h"

namespace FLAG_Race {
namespace {

ContinuousPhasePath::Evaluator MakeStraightEvaluator() {
  return [](const double w, ContinuousPhasePathState& state) {
    state.p = Eigen::Vector3d(w, 0.0, 1.0);
    state.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
    state.d2p_dw2 = Eigen::Vector3d::Zero();
    state.vel = state.dp_dw;
    state.valid = true;
    return true;
  };
}

ContinuousPhasePath::Evaluator MakeHeightVaryingEvaluator() {
  return [](const double w, ContinuousPhasePathState& state) {
    state.p = Eigen::Vector3d(w, 0.4 * std::sin(w), 1.0 + 0.2 * w);
    state.dp_dw = Eigen::Vector3d(1.0, 0.4 * std::cos(w), 0.2);
    state.d2p_dw2 = Eigen::Vector3d(0.0, -0.4 * std::sin(w), 0.0);
    state.vel = state.dp_dw;
    state.valid = true;
    return true;
  };
}

ContinuousPhasePath MakeStraightPath(const std::string& label = "mapped_bspline") {
  ContinuousPhasePath path;
  path.appendSegment(0.0, 3.0, label, MakeStraightEvaluator());
  return path;
}

phase_offset_navigation::PathCellBoundQuery
MakeStraightPathCellBoundQuery() {
  return [](const double w0, const double w1,
            phase_offset_core::PathCellGeometryCertificate& certificate) {
    certificate = phase_offset_core::PathCellGeometryCertificate();
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.segment_w0 = 0.0;
    certificate.segment_w1 = 3.0;
    certificate.segment_identity = 1U;
    certificate.inf_p_w_norm = 1.0;
    certificate.inf_horizontal_p_w_norm = 1.0;
    certificate.sup_p_w_norm = 1.0;
    certificate.sup_p_ww_norm = 0.0;
    certificate.sup_p_www_norm = 0.0;
    certificate.sup_horizontal_p_ww_norm = 0.0;
    certificate.horizontal_acceleration_bound_complete = true;
    certificate.sup_N_w_norm = 0.0;
    certificate.sup_abs_curvature = 0.0;
    certificate.normal_variation_bound = 0.0;
    certificate.tangent_variation_bound = 0.0;
    certificate.curvature_variation_bound = 0.0;
    certificate.midpoint_position_variation_bound = 0.5 * (w1 - w0);
    certificate.chord_deviation_bound = 0.0;
    certificate.valid = std::isfinite(w0) && std::isfinite(w1) &&
        w1 > w0 && w0 >= 0.0 && w1 <= 3.0;
    certificate.complete = certificate.valid;
    return certificate.valid;
  };
}

ContinuousPhasePath MakeHeightVaryingPath() {
  ContinuousPhasePath path;
  path.appendSegment(0.0, 3.0, "mapped_bspline", MakeHeightVaryingEvaluator());
  return path;
}

phase_offset_navigation::DistanceQuery MakeConstantQuery(
    const double distance) {
  return [distance](const Eigen::Vector3d&) {
    phase_offset_navigation::DistanceQueryResult result;
    result.status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
    result.signed_distance = distance;
    return result;
  };
}

phase_offset_navigation::DistanceQuery MakeStatusQuery(
    const phase_offset_navigation::DistanceStatus status) {
  return [status](const Eigen::Vector3d&) {
    phase_offset_navigation::DistanceQueryResult result;
    result.status = status;
    result.signed_distance =
        status == phase_offset_navigation::DistanceStatus::KNOWN_FREE ? 0.8 : 0.0;
    return result;
  };
}

PhaseOffsetClearanceAuditInput MakeInput(
    const ContinuousPhasePath& path,
    const phase_offset_navigation::DistanceQuery& query,
    const double current_w = 1.5) {
  PhaseOffsetClearanceAuditInput input;
  input.path = &path;
  input.current_w = current_w;
  input.back_w = 0.2;
  input.lookahead_w = 2.0;
  input.sample_step_w = 0.5;
  input.lateral_probe_half_width = 0.10;
  input.required_reference_clearance = 0.70;
  input.planner_safe_distance = 0.4;
  input.configured_search_margin = 0.5;
  input.source_revision = 7U;
  input.tube_revision = 11U;
  input.tube_source = 2;
  input.zero_gate_open = true;
  input.failure_latched = true;
  input.profile_complete = true;
  input.obstacle_certified = true;
  input.tube_first_invalid_w = 1.1;
  input.tube_first_stop_reason = 6;
  input.tube_insufficient_clearance_count = 3.0;
  input.distance_query = query;
  return input;
}

phase_offset_navigation::TubeProfile MakeCertifiedProfile() {
  phase_offset_navigation::TubeProfile profile;
  profile.source = phase_offset_navigation::TubeSource::ESDF;
  profile.raw_complete = true;
  profile.filtered_complete = true;
  profile.complete = true;
  profile.obstacle_certified = true;
  profile.source_revision = 7U;
  profile.tube_revision = 11U;
  profile.snapshot_sequence = 19U;
  profile.snapshot_resolution = 0.10;
  profile.snapshot_provenance_is_immutable = true;
  profile.preview_start_w = 0.0;
  profile.preview_end_w = 2.0;
  for (double w = 0.0; w <= 2.0; w += 1.0) {
    phase_offset_navigation::TubeRawSample sample;
    sample.w = w;
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.full_effective_radius = 0.55;
    sample.preincluded_map_uncertainty = 0.10;
    sample.residual_effective_radius = 0.40;
    sample.obstacle_lower = -0.20;
    sample.obstacle_upper = 0.20;
    sample.pre_inset_lower = -0.20;
    sample.pre_inset_upper = 0.20;
    sample.continuous_inset = 0.10;
    sample.raw_lower = -0.10;
    sample.raw_upper = 0.10;
    sample.filtered_lower = -0.08;
    sample.filtered_upper = 0.08;
    sample.complete = true;
    profile.samples.push_back(sample);
    profile.raw_build_samples.push_back(sample);
    phase_offset_navigation::TubeValidatorKnotEvidence evidence;
    evidence.w = w;
    evidence.observed = true;
    evidence.max_cover_radius = 0.15;
    evidence.max_requested_clearance = 0.60;
    evidence.filtered_contains_zero = true;
    evidence.zero_surface_covered = true;
    profile.validator_knot_evidence.push_back(evidence);
  }
  profile.zero_centerline_continuously_certified = true;
  return profile;
}

phase_offset_navigation::ClearanceQuery MakeCertifiedClearance(
    const double clearance) {
  return [clearance](const Eigen::Vector3d&, const double required) {
    phase_offset_navigation::ClearanceQueryResult result;
    result.status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
    result.clearance = clearance;
    result.clearance_certified = clearance >= required;
    return result;
  };
}

std::shared_ptr<const plan_env::SDFMapCaptureV2> MakeMetricCapture() {
  std::shared_ptr<plan_env::SDFMapCaptureV2> capture(
      new plan_env::SDFMapCaptureV2());
  capture->valid = true;
  capture->map_instance_id = 51U;
  capture->configuration_generation = 2U;
  capture->configuration_key = 31U;
  capture->frame_id = "world";
  capture->accepted_state_sequence = 7U;
  capture->accepted_time_ticks = 10U;
  capture->map_min = Eigen::Vector3d::Constant(-1.0);
  capture->map_max = Eigen::Vector3d::Constant(3.0);
  capture->grid_origin = capture->map_min;
  capture->capture_min = capture->map_min;
  capture->capture_max = capture->map_max;
  capture->source_min_index = Eigen::Vector3i::Zero();
  capture->source_max_index = Eigen::Vector3i::Constant(3);
  capture->voxel_count = Eigen::Vector3i::Constant(4);
  capture->resolution = 1.0;
  capture->included_map_inflation = 0.0;
  capture->occupied.assign(64U, 0U);
  // Native index (1,1,1) is the closed voxel [0,1]^3.
  capture->occupied[(1U * 4U + 1U) * 4U + 1U] = 1U;
  capture->support.valid = true;
  capture->support.complete = true;
  capture->support.evidence_basis =
      plan_env::kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  capture->support.evidence_sequence = 7U;
  capture->support.evidence_accepted_ticks = 10U;
  capture->support.map_instance_id = 51U;
  capture->support.configuration_generation = 2U;
  capture->support.configuration_key = 31U;
  capture->support.frame_id = "world";
  capture->support.support_min = capture->map_min;
  capture->support.support_max = capture->map_max;
  capture->support.required_halo = 0.0;
  capture->support.halo_reconciled = true;
  capture->support.mask.assign(64U, 1U);
  return capture;
}

// Two disconnected robust components: nominal delta=0 is safe, while the
// preferred nonzero component is selected by the actual Builder.  This is the
// Branch-B contract case, not an inset regression.
phase_offset_navigation::ClearanceQuery MakeSplitRobustClearance() {
  return [](const Eigen::Vector3d& point, const double required) {
    phase_offset_navigation::ClearanceQueryResult result;
    const double delta = point.y();
    const bool safe = std::abs(delta) <= 0.18 ||
        (delta >= 0.60 && delta <= 0.90);
    if (!safe) {
      result.status = phase_offset_navigation::DistanceStatus::OCCUPIED;
      return result;
    }
    result.status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(1.0, required);
    result.clearance_certified = true;
    return result;
  };
}

PhaseOffsetClearanceAuditInput MakeMarginInput(
    const ContinuousPhasePath& path,
    const phase_offset_navigation::TubeProfile& profile,
    const phase_offset_navigation::ClearanceQuery& clearance) {
  PhaseOffsetClearanceAuditInput input = MakeInput(path, MakeConstantQuery(0.8),
                                                    1.0);
  input.back_w = 1.0;
  input.lookahead_w = 1.0;
  input.snapshot_sequence = 19U;
  input.snapshot_stamp = 12.5;
  input.snapshot_resolution = 0.10;
  input.snapshot_included_map_inflation = 0.10;
  input.continuous_inset = -1.0;
  input.validator_cover_radius = -1.0;
  input.margins.uav_radius = 0.25;
  input.margins.map_uncertainty = 0.10;
  input.margins.localization_uncertainty = 0.05;
  input.margins.tracking_error_bound = 0.15;
  input.margins.preincluded_map_uncertainty = 0.10;
  input.raw_clearance_query = clearance;
  input.tube_profile = &profile;
  input.profile_complete = profile.complete;
  input.obstacle_certified = profile.obstacle_certified;
  return input;
}

double Value(const PhaseOffsetClearanceAuditResult& result,
             const PhaseOffsetClearanceAuditIndex index) {
  return result.values[index];
}

TEST(ClearanceAuditTest, OpenSpaceHasNoContractFailure) {
  const auto path = MakeStraightPath();
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeInput(path, MakeConstantQuery(0.9)));
  EXPECT_EQ(Value(result, kAuditSchemaVersion), 1.0);
  EXPECT_EQ(Value(result, kAuditValid), 1.0);
  EXPECT_GT(Value(result, kAuditSampleCount), 0.0);
  EXPECT_DOUBLE_EQ(Value(result, kAuditBaseKnownFreeCount),
                   Value(result, kAuditSampleCount));
  EXPECT_DOUBLE_EQ(Value(result, kAuditCountDgeRequired),
                   Value(result, kAuditSampleCount));
  EXPECT_EQ(Value(result, kAuditCountDltPlannerSafe), 0.0);
  EXPECT_EQ(Value(result, kAuditFirstContractFailurePresent), 0.0);
  EXPECT_EQ(Value(result, kAuditMinBaseDistanceValid), 1.0);
  EXPECT_DOUBLE_EQ(Value(result, kAuditMinBaseDistance), 0.9);
}

TEST(ClearanceAuditTest, SamePathSnapshotAccountsEveryMarginAndZero) {
  const auto path = MakeStraightPath();
  const auto profile = MakeCertifiedProfile();
  auto input = MakeMarginInput(path, profile, MakeCertifiedClearance(0.8));
  phase_offset_navigation::TubeSurfaceValidationResult surface;
  surface.complete = true;
  surface.max_cover_radius = 0.15;
  surface.max_requested_clearance = 0.60;
  input.surface_validation = &surface;
  const auto result = RunPhaseOffsetClearanceAudit(input);
  ASSERT_EQ(result.points.size(), 3U);
  EXPECT_TRUE(result.margin_accounting_complete);
  EXPECT_TRUE(result.same_path_snapshot);
  EXPECT_TRUE(result.zero_contained_raw_all_points);
  EXPECT_TRUE(result.zero_contained_filter_all_points);
  EXPECT_TRUE(result.zero_contained_validator_all_points);
  EXPECT_TRUE(result.map_inflation_alignment_evidence);
  EXPECT_FALSE(result.duplicate_margin_proven);
  for (const auto& point : result.points) {
    EXPECT_EQ(point.source_revision, 7U);
    EXPECT_EQ(point.tube_revision, 11U);
    EXPECT_EQ(point.snapshot_sequence, 19U);
    EXPECT_NEAR(point.snapshot_stamp, 12.5, 1e-12);
    EXPECT_TRUE(point.profile_sample_found);
    EXPECT_NEAR(point.full_effective_radius, 0.55, 1e-12);
    EXPECT_NEAR(point.residual_effective_radius, 0.40, 1e-12);
    EXPECT_NEAR(point.snapshot_included_map_inflation, 0.10, 1e-12);
    EXPECT_NEAR(point.continuous_inset, 0.10, 1e-12);
    EXPECT_NEAR(point.validator_cover_radius, 0.15, 1e-12);
    EXPECT_TRUE(point.pre_inset_contains_zero);
    EXPECT_TRUE(point.raw_contains_zero);
    EXPECT_TRUE(point.filtered_contains_zero);
    EXPECT_TRUE(point.validator_contains_zero);
    EXPECT_TRUE(point.residual_matches_full_minus_preincluded);
    EXPECT_TRUE(point.raw_clearance_valid);
  }
}

TEST(ClearanceAuditTest, ActualBuilderFilterValidatorBuildFeedsOneAuditRecord) {
  const auto path = MakeStraightPath();
  std::vector<phase_offset_core::PathDifferentialState,
              Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>
      preview;
  for (double w = 0.0; w <= 2.0 + 1e-12; w += 0.1) {
    phase_offset_core::PathDifferentialState state;
    state.p = Eigen::Vector3d(w, 0.0, 1.0);
    state.p_w = Eigen::Vector3d::UnitX();
    state.p_ww = Eigen::Vector3d::Zero();
    state.w = w;
    state.valid = true;
    preview.push_back(state);
  }
  const phase_offset_navigation::PathStateQuery path_query =
      [](const double w, phase_offset_core::PathDifferentialState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.p_w = Eigen::Vector3d::UnitX();
        state.p_ww = Eigen::Vector3d::Zero();
        state.w = w;
        state.valid = std::isfinite(w);
        return state.valid;
      };
  const phase_offset_navigation::ClearanceQuery clearance =
      [](const Eigen::Vector3d&, const double required) {
        phase_offset_navigation::ClearanceQueryResult result;
        result.status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
        result.clearance = std::max(10.0, required);
        result.clearance_certified = true;
        return result;
      };
  const auto path_cell_bound_query = MakeStraightPathCellBoundQuery();
  phase_offset_navigation::TubeProfile profile;
  ASSERT_TRUE(phase_offset_navigation::TubeBuilder().buildCloudClearance(
      phase_offset_navigation::TubeSource::ESDF, preview, clearance,
      path_query, path_cell_bound_query, 0.10, 1.0, 7U, 11U, profile));
  profile.snapshot_sequence = 19U;
  profile.snapshot_provenance_is_immutable = true;
  ASSERT_TRUE(phase_offset_navigation::TubeFilter().filter(profile, 1.0));
  phase_offset_navigation::TubeSurfaceValidationResult surface;
  ASSERT_TRUE(phase_offset_navigation::TubeSurfaceValidator().validate(
      profile, 1.0, path_query, path_cell_bound_query, clearance, 0.10, 0.40,
      0.10, surface));

  auto input = MakeMarginInput(path, profile, clearance);
  input.surface_validation = &surface;
  input.snapshot_sequence = 19U;
  input.snapshot_resolution = 0.10;
  const auto result = RunPhaseOffsetClearanceAudit(input);
  EXPECT_TRUE(result.margin_accounting_complete);
  EXPECT_TRUE(result.same_path_snapshot);
  EXPECT_TRUE(result.zero_contained_raw_all_points);
  EXPECT_TRUE(result.zero_contained_filter_all_points);
  EXPECT_TRUE(result.zero_contained_validator_all_points);
  EXPECT_FALSE(result.points.empty());
  for (const auto& point : result.points) {
    EXPECT_TRUE(point.profile_sample_found);
    EXPECT_TRUE(point.filter_sample_found);
    EXPECT_TRUE(point.validator_sample_found);
    EXPECT_TRUE(point.path_profile_match);
    EXPECT_TRUE(point.validator_contains_zero);
  }
}

TEST(ClearanceAuditTest,
     ActualBuilderIgnoresDisconnectedNonzeroComponentAndKeepsZero) {
  const auto path = MakeStraightPath();
  std::vector<phase_offset_core::PathDifferentialState,
              Eigen::aligned_allocator<phase_offset_core::PathDifferentialState>>
      preview;
  for (double w = 0.0; w <= 2.0 + 1e-12; w += 0.1) {
    phase_offset_core::PathDifferentialState state;
    state.p = Eigen::Vector3d(w, 0.0, 1.0);
    state.p_w = Eigen::Vector3d::UnitX();
    state.p_ww = Eigen::Vector3d::Zero();
    state.w = w;
    state.valid = true;
    preview.push_back(state);
  }
  const phase_offset_navigation::PathStateQuery path_query =
      [](const double w, phase_offset_core::PathDifferentialState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.p_w = Eigen::Vector3d::UnitX();
        state.p_ww = Eigen::Vector3d::Zero();
        state.w = w;
        state.valid = std::isfinite(w);
        return state.valid;
      };
  const auto split_clearance = MakeSplitRobustClearance();
  const auto path_cell_bound_query = MakeStraightPathCellBoundQuery();
  phase_offset_navigation::TubeProfile profile;
  ASSERT_TRUE(phase_offset_navigation::TubeBuilder().buildCloudClearance(
      phase_offset_navigation::TubeSource::ESDF, preview, split_clearance,
      path_query, path_cell_bound_query, 0.10, 1.0, 7U, 11U, profile));
  profile.snapshot_sequence = 19U;
  profile.snapshot_provenance_is_immutable = true;
  ASSERT_TRUE(phase_offset_navigation::TubeFilter().filter(profile, 1.0));
  phase_offset_navigation::TubeSurfaceValidationResult surface;
  ASSERT_TRUE(phase_offset_navigation::TubeSurfaceValidator().validate(
      profile, 1.0, path_query, path_cell_bound_query, split_clearance, 0.10,
      0.40, 0.10, surface));

  auto input = MakeMarginInput(path, profile, split_clearance);
  input.planner_distance_query = MakeConstantQuery(0.40);
  input.surface_validation = &surface;
  input.snapshot_sequence = 19U;
  input.snapshot_resolution = 0.10;
  const auto result = RunPhaseOffsetClearanceAudit(input);
  ASSERT_FALSE(result.points.empty());
  EXPECT_TRUE(result.margin_accounting_complete);
  EXPECT_TRUE(result.same_path_snapshot);
  EXPECT_NE(result.disposition,
            PhaseOffsetClearanceDisposition::
                PLANNER_TUBE_ROBUST_CONTRACT_MISMATCH);
  EXPECT_FALSE(result.planner_tube_robust_contract_mismatch_observed);
  EXPECT_TRUE(result.zero_contained_raw_all_points);
  EXPECT_TRUE(result.zero_contained_filter_all_points);
  EXPECT_TRUE(profile.zero_centerline_continuously_certified);
  for (const auto& point : result.points) {
    EXPECT_TRUE(point.path_profile_match);
    EXPECT_TRUE(point.pre_inset_contains_zero);
    EXPECT_TRUE(point.pre_inset_interval_valid);
    EXPECT_TRUE(point.pre_inset_cross_section_valid);
    EXPECT_TRUE(point.raw_contains_zero);
    EXPECT_TRUE(point.filtered_contains_zero);
    EXPECT_FALSE(point.planner_tube_robust_contract_mismatch);
  }
}

TEST(ClearanceAuditTest,
     UnknownPreInsetCrossSectionIsNotLabelledAsPlannerTubeMismatch) {
  const auto path = MakeStraightPath();
  const phase_offset_navigation::TubeCrossSectionReason invalid_reasons[] = {
      phase_offset_navigation::TubeCrossSectionReason::CENTER_UNKNOWN,
      phase_offset_navigation::TubeCrossSectionReason::INVALID_GEOMETRY,
      phase_offset_navigation::TubeCrossSectionReason::CURVATURE_NUMERICAL_FAILURE,
  };
  for (const auto reason : invalid_reasons) {
    auto profile = MakeCertifiedProfile();
    for (auto& sample : profile.samples) {
      sample.pre_inset_lower = 0.20;
      sample.pre_inset_upper = 0.40;
      sample.raw_lower = 0.20;
      sample.raw_upper = 0.40;
      sample.filtered_lower = 0.20;
      sample.filtered_upper = 0.40;
      sample.cross_section_reason = reason;
    }
    for (auto& sample : profile.raw_build_samples) {
      sample.pre_inset_lower = 0.20;
      sample.pre_inset_upper = 0.40;
      sample.raw_lower = 0.20;
      sample.raw_upper = 0.40;
      sample.filtered_lower = 0.20;
      sample.filtered_upper = 0.40;
      sample.cross_section_reason = reason;
    }

    auto input = MakeMarginInput(path, profile, MakeCertifiedClearance(0.8));
    input.planner_distance_query = MakeConstantQuery(0.40);
    const auto result = RunPhaseOffsetClearanceAudit(input);

    ASSERT_FALSE(result.points.empty());
    EXPECT_EQ(result.disposition, PhaseOffsetClearanceDisposition::UNKNOWN);
    EXPECT_FALSE(result.planner_tube_robust_contract_mismatch_observed);
    for (const auto& point : result.points) {
      EXPECT_FALSE(point.pre_inset_contains_zero);
      EXPECT_TRUE(point.pre_inset_interval_valid);
      EXPECT_FALSE(point.pre_inset_cross_section_valid);
      EXPECT_FALSE(point.planner_tube_robust_contract_mismatch);
    }
  }
}

TEST(ClearanceAuditTest, CurrentPlannerSearchAndResidualOrderingIsAccepted) {
  const auto path = MakeStraightPath();
  const auto profile = MakeCertifiedProfile();
  auto input = MakeMarginInput(path, profile, MakeCertifiedClearance(0.8));
  input.planner_safe_distance = 0.40;
  input.configured_search_margin = 0.50;
  input.required_reference_clearance = 0.45;
  const auto result = RunPhaseOffsetClearanceAudit(input);
  EXPECT_EQ(Value(result, kAuditValid), 1.0);
  EXPECT_TRUE(result.margin_accounting_complete);
  EXPECT_TRUE(result.same_path_snapshot);
}

TEST(ClearanceAuditTest, CenterlineInsufficientClearanceIsNotRescued) {
  const auto path = MakeStraightPath();
  const auto profile = MakeCertifiedProfile();
  const auto input = MakeMarginInput(path, profile,
                                      MakeCertifiedClearance(0.35));
  const auto result = RunPhaseOffsetClearanceAudit(input);
  ASSERT_FALSE(result.points.empty());
  EXPECT_FALSE(result.margin_accounting_complete);
  EXPECT_FALSE(result.points.front().raw_clearance_valid);
  EXPECT_FALSE(result.points.front().centerline_clearance_sufficient);
  EXPECT_TRUE(result.zero_contained_raw_all_points);
}

TEST(ClearanceAuditTest, UnknownSnapshotIsDistinctFromInsufficientDistance) {
  const auto path = MakeStraightPath();
  const auto profile = MakeCertifiedProfile();
  auto unknown = [](const Eigen::Vector3d&, const double) {
    phase_offset_navigation::ClearanceQueryResult result;
    result.status = phase_offset_navigation::DistanceStatus::UNKNOWN;
    return result;
  };
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeMarginInput(path, profile, unknown));
  ASSERT_FALSE(result.points.empty());
  EXPECT_TRUE(result.points.front().unknown_or_out_of_map);
  EXPECT_FALSE(result.points.front().raw_clearance_valid);
}

TEST(ClearanceAuditTest, SnapshotInflationIsNotAssumedToBeMapUncertainty) {
  const auto path = MakeStraightPath();
  const auto profile = MakeCertifiedProfile();
  auto input = MakeMarginInput(path, profile, MakeCertifiedClearance(0.8));
  auto result = RunPhaseOffsetClearanceAudit(input);
  EXPECT_FALSE(result.duplicate_margin_proven);
  input.snapshot_inflation_is_map_uncertainty = true;
  input.snapshot_included_map_inflation = 0.20;
  result = RunPhaseOffsetClearanceAudit(input);
  EXPECT_TRUE(result.duplicate_margin_proven);
}

TEST(ClearanceAuditTest, SnapshotInflationBelowPreincludedFailsAlignment) {
  const auto path = MakeStraightPath();
  const auto profile = MakeCertifiedProfile();
  auto input = MakeMarginInput(path, profile, MakeCertifiedClearance(0.8));
  input.snapshot_included_map_inflation = 0.05;
  const auto result = RunPhaseOffsetClearanceAudit(input);
  ASSERT_FALSE(result.points.empty());
  EXPECT_FALSE(result.map_inflation_alignment_evidence);
  EXPECT_FALSE(result.points.front().snapshot_inflation_covers_preincluded);
}

TEST(ClearanceAuditTest, MissingSnapshotProvenanceCannotClaimCompleteAccounting) {
  const auto path = MakeStraightPath();
  const auto profile = MakeCertifiedProfile();
  auto input = MakeMarginInput(path, profile, MakeCertifiedClearance(0.8));
  input.snapshot_sequence = 0U;
  const auto result = RunPhaseOffsetClearanceAudit(input);
  EXPECT_FALSE(result.margin_accounting_complete);
  EXPECT_FALSE(result.same_path_snapshot);
}

TEST(ClearanceAuditTest, InsetExcludingZeroIsSeparatedFromFilterRegression) {
  const auto path = MakeStraightPath();
  auto profile = MakeCertifiedProfile();
  for (auto& sample : profile.samples) {
    sample.obstacle_lower = -0.05;
    sample.obstacle_upper = 0.20;
    sample.pre_inset_lower = -0.05;
    sample.pre_inset_upper = 0.20;
    sample.raw_lower = 0.05;
    sample.raw_upper = 0.10;
    sample.filtered_lower = 0.05;
    sample.filtered_upper = 0.10;
  }
  for (auto& sample : profile.raw_build_samples) {
    sample.obstacle_lower = -0.05;
    sample.obstacle_upper = 0.20;
    sample.pre_inset_lower = -0.05;
    sample.pre_inset_upper = 0.20;
    sample.raw_lower = 0.05;
    sample.raw_upper = 0.10;
    sample.filtered_lower = 0.05;
    sample.filtered_upper = 0.10;
  }
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeMarginInput(path, profile, MakeCertifiedClearance(0.8)));
  ASSERT_FALSE(result.points.empty());
  EXPECT_TRUE(result.points.front().pre_inset_contains_zero);
  EXPECT_TRUE(result.points.front().zero_excluded_by_inset);
  EXPECT_FALSE(result.zero_contained_raw_all_points);
  EXPECT_FALSE(result.zero_contained_filter_all_points);
}

TEST(ClearanceAuditTest, HalfToSevenTenthsIsDiagnosticOnlyAbovePlannerClearance) {
  const auto path = MakeStraightPath();
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeInput(path, MakeConstantQuery(0.6)));
  EXPECT_DOUBLE_EQ(Value(result, kAuditCountSearchMarginLeDltRequired),
                   Value(result, kAuditSampleCount));
  EXPECT_EQ(Value(result, kAuditFirstContractFailurePresent), 0.0);
  EXPECT_EQ(Value(result, kAuditFirstContractFailureDeficitValid), 0.0);
  EXPECT_DOUBLE_EQ(Value(result, kAuditFirstContractFailureDeficit), 0.0);
}

TEST(ClearanceAuditTest, FourToFiveTenthsFillsOptimizerZeroCostBin) {
  const auto path = MakeStraightPath();
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeInput(path, MakeConstantQuery(0.45)));
  EXPECT_DOUBLE_EQ(Value(result, kAuditCountPlannerSafeLeDltSearchMargin),
                   Value(result, kAuditSampleCount));
  EXPECT_DOUBLE_EQ(Value(result, kAuditFirstContractFailureDeficit), 0.0);
}

TEST(ClearanceAuditTest, BelowFourTenthsFillsFirstDistanceBin) {
  const auto path = MakeStraightPath();
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeInput(path, MakeConstantQuery(0.3)));
  EXPECT_DOUBLE_EQ(Value(result, kAuditCountDltPlannerSafe),
                   Value(result, kAuditSampleCount));
  EXPECT_NEAR(Value(result, kAuditFirstContractFailureDeficit), 0.1, 1e-12);
}

TEST(ClearanceAuditTest, DistanceStatusesAreDistinguished) {
  using phase_offset_navigation::DistanceStatus;
  const auto path = MakeStraightPath();
  struct Case {
    DistanceStatus status;
    double reason;
    PhaseOffsetClearanceAuditIndex count_index;
  };
  const Case cases[] = {
      {DistanceStatus::UNAVAILABLE, 2.0, kAuditBaseUnavailableCount},
      {DistanceStatus::OUT_OF_MAP, 3.0, kAuditBaseOutOfMapCount},
      {DistanceStatus::UNKNOWN, 4.0, kAuditBaseUnknownCount},
      {DistanceStatus::OCCUPIED, 5.0, kAuditBaseOccupiedCount},
  };
  for (const Case& test_case : cases) {
    const auto result = RunPhaseOffsetClearanceAudit(
        MakeInput(path, MakeStatusQuery(test_case.status)));
    EXPECT_GT(Value(result, test_case.count_index), 0.0);
    EXPECT_EQ(Value(result, kAuditFirstContractFailurePresent), 1.0);
    EXPECT_EQ(Value(result, kAuditFirstContractFailureReason),
              test_case.reason);
    EXPECT_EQ(Value(result, kAuditFirstContractFailureDistanceValid), 0.0);
    EXPECT_EQ(Value(result, kAuditFirstContractFailureDeficitValid), 0.0);
  }
}

TEST(ClearanceAuditTest, FirstFailureInC2QuinticIsDetected) {
  ContinuousPhasePath path;
  path.appendSegment(0.0, 1.0, "mapped_bspline", MakeStraightEvaluator());
  path.appendSegment(1.0, 2.0, "c2_quintic", MakeStraightEvaluator());
  auto query = [](const Eigen::Vector3d& point) {
    phase_offset_navigation::DistanceQueryResult result;
    result.status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
    result.signed_distance = point.x() > 1.0 ? 0.3 : 0.9;
    return result;
  };
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeInput(path, query, 0.8));
  EXPECT_EQ(Value(result, kAuditFirstContractFailurePresent), 1.0);
  EXPECT_EQ(Value(result, kAuditFirstContractFailureSegmentCode), 2.0);
  EXPECT_GE(Value(result, kAuditFirstContractFailureW), 1.0 - 1e-9);
}

TEST(ClearanceAuditTest, FirstFailureInMappedBsplineIsDetected) {
  ContinuousPhasePath path;
  path.appendSegment(0.0, 1.0, "mapped_bspline", MakeStraightEvaluator());
  path.appendSegment(1.0, 2.0, "c2_quintic", MakeStraightEvaluator());
  auto query = [](const Eigen::Vector3d& point) {
    phase_offset_navigation::DistanceQueryResult result;
    result.status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
    result.signed_distance = point.x() < 1.0 ? 0.3 : 0.9;
    return result;
  };
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeInput(path, query, 0.8));
  EXPECT_EQ(Value(result, kAuditFirstContractFailureSegmentCode), 1.0);
  EXPECT_LT(Value(result, kAuditFirstContractFailureW), 1.0);
}

TEST(ClearanceAuditTest, HeightVaryingPathUsesFullThreeDimensionalPoint) {
  const auto path = MakeHeightVaryingPath();
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeInput(path, MakeConstantQuery(0.9), 1.5));
  EXPECT_EQ(Value(result, kAuditGeometryInvalidCount), 0.0);
  EXPECT_EQ(Value(result, kAuditFirstContractFailurePresent), 0.0);
  EXPECT_GT(Value(result, kAuditMinBaseZ), 1.0);
  EXPECT_GT(Value(result, kAuditMinBaseX), 1.0);
}

TEST(ClearanceAuditTest, PositiveAndNegativeProbesAreNotSwapped) {
  const auto path = MakeStraightPath();
  auto query = [](const Eigen::Vector3d& point) {
    phase_offset_navigation::DistanceQueryResult result;
    result.status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
    // Asymmetric probe query: +y (positive probe) is farther than -y
    // (negative probe).  Swapping the probes would therefore fail.
    result.signed_distance = point.y() > 0.0 ? 0.8 : 0.4;
    return result;
  };
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeInput(path, query, 1.5));
  EXPECT_GT(Value(result, kAuditPositiveProbeKnownFreeCount), 0.0);
  EXPECT_GT(Value(result, kAuditNegativeProbeKnownFreeCount), 0.0);
  EXPECT_EQ(Value(result, kAuditPositiveProbeMinDistanceValid), 1.0);
  EXPECT_EQ(Value(result, kAuditNegativeProbeMinDistanceValid), 1.0);
  EXPECT_NEAR(Value(result, kAuditPositiveProbeMinDistance), 0.8, 1e-12);
  EXPECT_NEAR(Value(result, kAuditNegativeProbeMinDistance), 0.4, 1e-12);
  EXPECT_NEAR(Value(result, kAuditMinBaseDistance), 0.4, 1e-12);
}

TEST(ClearanceAuditTest,
     CenterDistanceAndClosedVolumeRemainDistinctAtUnchangedEpsilon) {
  // The immutable backing contains one occupied voxel [0,1]^3.  At q=(1.25,
  // .5,.5), its center is 0.75 m away while its closed volume is 0.25 m away.
  // Keep the planner epsilon fixed at 0.40 for both metrics.
  const double epsilon = 0.40;
  const Eigen::Vector3d witness(1.25, 0.5, 0.5);
  const std::shared_ptr<const plan_env::SDFMapCaptureV2> capture =
      MakeMetricCapture();
  const SDFMapCaptureQueryBridgeV2 bridge =
      makeSDFMapCaptureQueryV2(capture);
  ASSERT_TRUE(bridge.usable());
  EXPECT_TRUE(bridge.descriptor.closed_inflated_voxel_volume_metric);
  EXPECT_EQ(bridge.descriptor.clearance_metric,
            "closed_inflated_voxel_volume");
  EXPECT_DOUBLE_EQ(bridge.descriptor.included_map_inflation, 0.0);

  // Mirror the same occupied backing through the unchanged legacy
  // planner-ESDF-base center query.  Its requested radius is deliberately
  // larger only to expose the uncapped 0.75 m center distance; admission
  // still compares that evidence against the unchanged epsilon above.
  plan_env::CloudOccupancySnapshot legacy;
  legacy.valid = true;
  legacy.observation_sequence = 7U;
  legacy.map_min = Eigen::Vector3d::Constant(-1.0);
  legacy.map_max = Eigen::Vector3d::Constant(3.0);
  legacy.observed_min = legacy.map_min;
  legacy.observed_max = legacy.map_max;
  legacy.grid_origin = legacy.map_min;
  legacy.voxel_count = Eigen::Vector3i::Constant(4);
  legacy.resolution = 1.0;
  legacy.included_map_inflation = 0.0;
  legacy.occupied.assign(64U, 0U);
  legacy.occupied[(1U * 4U + 1U) * 4U + 1U] = 1U;
  const auto center_at_epsilon =
      plan_env::queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
          legacy, witness, epsilon);
  EXPECT_EQ(center_at_epsilon.status,
            plan_env::CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_TRUE(center_at_epsilon.clearance_certified);
  EXPECT_DOUBLE_EQ(
      center_at_epsilon.nearest_inflated_occupied_voxel_center_distance,
      epsilon);
  const auto center = plan_env::queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
      legacy, witness, 1.0);
  EXPECT_EQ(center.status, plan_env::CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_TRUE(center.clearance_certified);
  EXPECT_NEAR(center.nearest_inflated_occupied_voxel_center_distance,
              0.75, 1e-12);
  EXPECT_GT(center.nearest_inflated_occupied_voxel_center_distance, epsilon);

  const auto native = plan_env::certifySDFMapCaptureFreeBallV2(
      *capture, witness, epsilon);
  EXPECT_EQ(native.status,
            plan_env::SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE);
  const auto volume = bridge.free_ball_query(witness, epsilon);
  EXPECT_EQ(volume.status,
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);
  EXPECT_FALSE(volume.certified);
  EXPECT_FALSE(volume.exact);
  EXPECT_NE(volume.provenance.find("closed-inflated-voxel-volume"),
            std::string::npos);
  EXPECT_NE(volume.provenance.find("inconclusive"), std::string::npos);
  EXPECT_DOUBLE_EQ(epsilon, 0.40);
}

TEST(ClearanceAuditTest, V2AuditSchemaIsStrictSixtyAndFinite) {
  EXPECT_EQ(kAuditCount, 60U);
  EXPECT_EQ(kAuditSchemaVersion, 0U);
  EXPECT_EQ(kAuditCount - 1U, kAuditTubeInsufficientClearanceCount);
  const auto path = MakeStraightPath();
  const auto result = RunPhaseOffsetClearanceAudit(
      MakeInput(path, MakeConstantQuery(0.9)));
  for (const double value : result.values) {
    EXPECT_TRUE(std::isfinite(value));
  }
}

TEST(ClearanceAuditTest, InputIsNotModified) {
  const auto path = MakeStraightPath();
  PhaseOffsetClearanceAuditInput input = MakeInput(path, MakeConstantQuery(0.9));
  const PhaseOffsetClearanceAuditInput before = input;
  const auto result = RunPhaseOffsetClearanceAudit(input);
  (void)result;
  EXPECT_EQ(input.path, before.path);
  EXPECT_DOUBLE_EQ(input.current_w, before.current_w);
  EXPECT_DOUBLE_EQ(input.sample_step_w, before.sample_step_w);
  EXPECT_DOUBLE_EQ(input.lateral_probe_half_width,
                   before.lateral_probe_half_width);
  EXPECT_DOUBLE_EQ(input.required_reference_clearance,
                   before.required_reference_clearance);
  EXPECT_EQ(input.source_revision, before.source_revision);
  EXPECT_EQ(input.distance_query ? 1 : 0, before.distance_query ? 1 : 0);
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
