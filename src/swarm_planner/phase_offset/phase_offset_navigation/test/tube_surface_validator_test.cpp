#include <gtest/gtest.h>

#include "phase_offset_navigation/tube_surface_validator.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

namespace phase_offset_navigation {
namespace {

TubeProfile MakeProfile(const std::vector<double>& w,
                        const double lower = -0.20,
                        const double upper = 0.20,
                        const bool certified = true) {
  TubeProfile profile;
  profile.source = TubeSource::ESDF;
  profile.raw_complete = true;
  profile.filtered_complete = true;
  profile.complete = true;
  profile.obstacle_certified = true;
  profile.cell_geometry_certified = certified;
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
    sample.complete = true;
    sample.positive_certified = true;
    sample.negative_certified = true;
    sample.regularity_intersection = true;
    profile.samples.push_back(sample);
  }
  return profile;
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

ClearanceQuery OpenLowerBound() {
  return [](const Eigen::Vector3d&, const double required) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(5.0, required);
    result.clearance_certified = true;
    result.clearance_is_exact = false;
    return result;
  };
}

ClearanceQuery ExactClearance(const double value) {
  return [value](const Eigen::Vector3d&, const double) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = value;
    result.clearance_certified = true;
    result.clearance_is_exact = true;
    return result;
  };
}

ClearanceQuery ExactPointObstacle(const Eigen::Vector3d& obstacle) {
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
    result.clearance_is_exact = true;
    return result;
  };
}

ClearanceQuery PrimaryUnknownThen(
    const ClearanceQueryResult& base_result, std::size_t& calls,
    std::vector<Eigen::Vector3d>* points = nullptr,
    std::vector<double>* required_radii = nullptr) {
  return [base_result, &calls, points, required_radii](
      const Eigen::Vector3d& point, const double required) {
    if (points != nullptr) points->push_back(point);
    if (required_radii != nullptr) required_radii->push_back(required);
    const std::size_t call = calls++;
    if (call == 0U) {
      ClearanceQueryResult result;
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    if (call == 1U) return base_result;
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::max(5.0, required);
    result.clearance_certified = true;
    result.clearance_is_exact = true;
    return result;
  };
}

ClearanceQueryResult BaseResult(const DistanceStatus status,
                                const double clearance = 0.0,
                                const bool certified = false,
                                const bool exact = false) {
  ClearanceQueryResult result;
  result.status = status;
  result.clearance = clearance;
  result.clearance_certified = certified;
  result.clearance_is_exact = exact;
  return result;
}

struct ClearanceCallRecord {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double required = 0.0;
  ClearanceQueryResult result;
};

bool IsBaseProbeRadius(const double required) {
  return std::abs(required - 0.40) <= 1e-12;
}

const TubeSurfaceCellEvidence* FindCellEvidence(
    const TubeSurfaceValidationResult& result, const double w0,
    const double w1, const double v0 = 0.0, const double v1 = 1.0) {
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    if (std::abs(evidence.w0 - w0) <= 1e-12 &&
        std::abs(evidence.w1 - w1) <= 1e-12 &&
        std::abs(evidence.v0 - v0) <= 1e-12 &&
        std::abs(evidence.v1 - v1) <= 1e-12) {
      return &evidence;
    }
  }
  return nullptr;
}

PathCellBoundQuery CertifiedLineCells(const double inf_speed = 1.0,
                                      const double sup_normal_w = 0.0,
                                      const double midpoint_scale = 0.0) {
  return [inf_speed, sup_normal_w, midpoint_scale](
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
    certificate.sup_horizontal_p_ww_norm = sup_normal_w * inf_speed;
    certificate.horizontal_acceleration_bound_complete = true;
    certificate.sup_N_w_norm = sup_normal_w;
    certificate.sup_abs_curvature = 0.0;
    certificate.normal_variation_bound = sup_normal_w * (w1 - w0);
    certificate.tangent_variation_bound = 0.0;
    certificate.curvature_variation_bound = 0.0;
    certificate.midpoint_position_variation_bound = midpoint_scale * (w1 - w0);
    certificate.chord_deviation_bound = 0.0;
    certificate.valid = std::isfinite(w0) && std::isfinite(w1) && w1 > w0;
    certificate.complete = certificate.valid;
    return certificate.valid;
  };
}

TEST(TubeSurfaceValidatorTest, OpenRibbonIsCertifiedWithGeometricOnlyCover) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20});
  std::size_t calls = 0U;
  const ClearanceQuery query = [base = OpenLowerBound(), &calls](
      const Eigen::Vector3d& point, const double required) {
    ++calls;
    return base(point, required);
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_EQ(result.query_sample_count, result.clearance_leaf_cell_count);
  EXPECT_EQ(calls, result.query_sample_count);
  EXPECT_DOUBLE_EQ(result.support_alignment_bound, 0.0);
  EXPECT_TRUE(result.zero_centerline_continuously_certified);
  EXPECT_EQ(profile.proof_level, TubeProofLevel::CONTINUOUS_COVER_PROOF);
  EXPECT_NEAR(result.max_cover_radius, 0.20, 1e-12);
}

TEST(TubeSurfaceValidatorTest, ConfiguredMinimumReferenceSpeedIsEnforced) {
  TubeSurfaceValidatorConfig config;
  config.minimum_reference_speed = 0.50;
  TubeProfile profile = MakeProfile({0.0, 0.20});
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(0.1, 2.0),
      OpenLowerBound(), 0.05, 0.0, 0.1, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::REGULARITY_UNPROVEN);
}

TEST(TubeSurfaceValidatorTest, ExactContractUnsafeWitnessIsRejected) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), ExactClearance(0.39),
      0.05, 0.40, 0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
  EXPECT_EQ(result.first_failure_reason, TubeStopReason::INSUFFICIENT_CLEARANCE);
}

TEST(TubeSurfaceValidatorTest, CertifiedLowerBoundBelowContractIsInconclusive) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  const ClearanceQuery query = [](const Eigen::Vector3d&, const double) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 0.39;
    result.clearance_certified = true;
    result.clearance_is_exact = false;
    return result;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_NE(result.outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
}

TEST(TubeSurfaceValidatorTest, ExactBoundaryUsesOutwardRequirement) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  const ClearanceQuery boundary = [](const Eigen::Vector3d&, const double required) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = required;
    result.clearance_certified = true;
    return result;
  };
  TubeSurfaceValidationResult result;
  EXPECT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), boundary, 0.05, 0.40,
      0.10, result));
  TubeProfile below_profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  const ClearanceQuery below = [](const Eigen::Vector3d&, const double required) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = std::nextafter(required, 0.0);
    result.clearance_certified = true;
    return result;
  };
  TubeSurfaceValidationResult below_result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      below_profile, 0.0, LinePath(), CertifiedLineCells(), below, 0.05, 0.40,
      0.10, below_result));
  EXPECT_EQ(below_result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
}

TEST(TubeSurfaceValidatorTest,
     PrimaryUnknownCappedBaseClearanceRefinesWithoutParentSafe) {
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 0;
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.20, 0.20);
  std::size_t calls = 0U;
  std::vector<Eigen::Vector3d> points;
  std::vector<double> required_radii;
  const ClearanceQuery query = PrimaryUnknownThen(
      BaseResult(DistanceStatus::KNOWN_FREE, 0.40, true, false), calls,
      &points, &required_radii);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(calls, 3U);
  EXPECT_EQ(result.query_sample_count, calls);
  EXPECT_EQ(result.clearance_leaf_cell_count, 2U);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::DEPTH_GUARD);
  EXPECT_GT(result.max_requested_clearance, 0.40);

  const TubeSurfaceCellEvidence* parent = nullptr;
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    if (std::abs(evidence.w0) <= 1e-12 &&
        std::abs(evidence.w1) <= 1e-12) {
      parent = &evidence;
      break;
    }
  }
  ASSERT_NE(parent, nullptr);
  EXPECT_EQ(parent->outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(parent->inconclusive_reason,
            TubeSurfaceInconclusiveReason::DEPTH_GUARD);
  EXPECT_EQ(parent->clearance_status, DistanceStatus::UNKNOWN);
  EXPECT_FALSE(parent->witness_clearance_valid);
  EXPECT_FALSE(parent->witness_clearance_certified);
  EXPECT_FALSE(parent->witness_clearance_exact);
  EXPECT_GT(parent->requested_clearance, 0.40);
  ASSERT_GE(required_radii.size(), 2U);
  EXPECT_GT(required_radii[0], 0.40);
  EXPECT_DOUBLE_EQ(required_radii[1], 0.40);
  EXPECT_EQ(0, std::memcmp(&points[0].x(), &points[1].x(), sizeof(double)));
  EXPECT_EQ(0, std::memcmp(&points[0].y(), &points[1].y(), sizeof(double)));
  EXPECT_EQ(0, std::memcmp(&points[0].z(), &points[1].z(), sizeof(double)));
}

TEST(TubeSurfaceValidatorTest,
     DomainLimitedUnknownUsesCoverReducingMoveAndIndependentChildren) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.20, 0.20);
  std::size_t calls = 0U;
  const ClearanceQuery query = [&calls](const Eigen::Vector3d& point,
                                         const double required) {
    ++calls;
    ClearanceQueryResult result;
    if (std::abs(point.x()) < 0.05 || std::abs(point.y()) > 1e-9) {
      result.status = DistanceStatus::KNOWN_FREE;
      result.clearance = std::max(5.0, required);
      result.clearance_certified = true;
      result.clearance_is_exact = true;
      return result;
    }
    if (required > 0.45) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 0.40;
    result.clearance_certified = true;
    result.clearance_is_exact = false;
    return result;
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_EQ(result.query_sample_count, calls);
  EXPECT_EQ(result.clearance_leaf_cell_count + 1U,
            result.query_sample_count);
  EXPECT_GT(result.split_v_count, 0U);
  EXPECT_EQ(result.split_w_count, 0U);
  EXPECT_EQ(result.split_both_count, 0U);

  bool lower_child_safe = false;
  bool upper_child_safe = false;
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    if (std::abs(evidence.w0) > 1e-12 ||
        std::abs(evidence.w1 - 0.20) > 1e-12) {
      continue;
    }
    if (std::abs(evidence.v0) <= 1e-12 &&
        std::abs(evidence.v1 - 0.5) <= 1e-12) {
      lower_child_safe = evidence.outcome == TubeSurfaceOutcome::SAFE;
    }
    if (std::abs(evidence.v0 - 0.5) <= 1e-12 &&
        std::abs(evidence.v1 - 1.0) <= 1e-12) {
      upper_child_safe = evidence.outcome == TubeSurfaceOutcome::SAFE;
    }
  }
  EXPECT_TRUE(lower_child_safe);
  EXPECT_TRUE(upper_child_safe);
}

TEST(TubeSurfaceValidatorTest, PrimaryUnknownBaseUnknownIsTerminal) {
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 0;
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::size_t calls = 0U;
  const ClearanceQuery query = PrimaryUnknownThen(
      BaseResult(DistanceStatus::UNKNOWN), calls);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(calls, 3U);
  EXPECT_EQ(result.query_sample_count, 3U);
  EXPECT_EQ(result.clearance_leaf_cell_count, 2U);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_UNKNOWN);
  EXPECT_EQ(result.split_w_count + result.split_v_count +
                result.split_both_count,
            0U);
}

TEST(TubeSurfaceValidatorTest, PrimaryUnknownBaseOutOfMapIsTerminal) {
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 0;
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::size_t calls = 0U;
  const ClearanceQuery query = PrimaryUnknownThen(
      BaseResult(DistanceStatus::OUT_OF_MAP), calls);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(calls, 3U);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_OUT_OF_MAP);
}

TEST(TubeSurfaceValidatorTest, PrimaryUnknownBaseUnavailableIsTerminal) {
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 0;
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::size_t calls = 0U;
  const ClearanceQuery query = PrimaryUnknownThen(
      BaseResult(DistanceStatus::UNAVAILABLE), calls);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(calls, 3U);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_UNAVAILABLE);
}

TEST(TubeSurfaceValidatorTest,
     PrimaryUnknownBaseOccupiedUsesBaseTerminalEvidence) {
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 0;
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::size_t calls = 0U;
  const ClearanceQuery query = PrimaryUnknownThen(
      BaseResult(DistanceStatus::OCCUPIED), calls);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
  EXPECT_EQ(result.inconclusive_reason, TubeSurfaceInconclusiveReason::NONE);
  EXPECT_EQ(result.first_failure_reason, TubeStopReason::OCCUPIED);
  const TubeSurfaceCellEvidence* evidence = nullptr;
  for (const TubeSurfaceCellEvidence& candidate : result.cell_evidence) {
    if (std::abs(candidate.w0) <= 1e-12 &&
        std::abs(candidate.w1) <= 1e-12) {
      evidence = &candidate;
      break;
    }
  }
  ASSERT_NE(evidence, nullptr);
  EXPECT_EQ(evidence->outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
  EXPECT_EQ(evidence->clearance_status, DistanceStatus::OCCUPIED);
  EXPECT_TRUE(evidence->clearance_query_attempted);
  EXPECT_FALSE(evidence->witness_clearance_valid);
  EXPECT_EQ(evidence->witness_legacy_reason, TubeStopReason::OCCUPIED);
  EXPECT_DOUBLE_EQ(evidence->requested_clearance, 0.40);
}

TEST(TubeSurfaceValidatorTest,
     PrimaryUnknownBaseExactInsufficientClearanceIsContractUnsafe) {
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 0;
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::size_t calls = 0U;
  const ClearanceQuery query = PrimaryUnknownThen(
      BaseResult(DistanceStatus::KNOWN_FREE, 0.39, true, true), calls);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
  EXPECT_EQ(result.first_failure_reason, TubeStopReason::INSUFFICIENT_CLEARANCE);
  const TubeSurfaceCellEvidence* evidence = nullptr;
  for (const TubeSurfaceCellEvidence& candidate : result.cell_evidence) {
    if (std::abs(candidate.w0) <= 1e-12 &&
        std::abs(candidate.w1) <= 1e-12) {
      evidence = &candidate;
      break;
    }
  }
  ASSERT_NE(evidence, nullptr);
  EXPECT_EQ(evidence->clearance_status, DistanceStatus::KNOWN_FREE);
  EXPECT_TRUE(evidence->witness_clearance_valid);
  EXPECT_TRUE(evidence->witness_clearance_certified);
  EXPECT_TRUE(evidence->witness_clearance_exact);
  EXPECT_DOUBLE_EQ(evidence->witness_clearance, 0.39);
  EXPECT_DOUBLE_EQ(evidence->requested_clearance, 0.40);
}

TEST(TubeSurfaceValidatorTest,
     PrimaryUnknownBaseNonExactInsufficientClearanceRemainsInconclusive) {
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 0;
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::size_t calls = 0U;
  const ClearanceQuery query = PrimaryUnknownThen(
      BaseResult(DistanceStatus::KNOWN_FREE, 0.39, true, false), calls);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_UNCERTIFIED);
  EXPECT_NE(result.outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
  const TubeSurfaceCellEvidence* evidence = nullptr;
  for (const TubeSurfaceCellEvidence& candidate : result.cell_evidence) {
    if (std::abs(candidate.w0) <= 1e-12 &&
        std::abs(candidate.w1) <= 1e-12) {
      evidence = &candidate;
      break;
    }
  }
  ASSERT_NE(evidence, nullptr);
  EXPECT_EQ(evidence->clearance_status, DistanceStatus::KNOWN_FREE);
  EXPECT_TRUE(evidence->witness_clearance_valid);
  EXPECT_TRUE(evidence->witness_clearance_certified);
  EXPECT_FALSE(evidence->witness_clearance_exact);
  EXPECT_DOUBLE_EQ(evidence->witness_clearance, 0.39);
}

TEST(TubeSurfaceValidatorTest,
     PrimaryUnknownAtBudgetBoundarySkipsBaseProbeAndChildren) {
  TubeSurfaceValidatorConfig config;
  config.max_query_samples = 1U;
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::vector<ClearanceCallRecord> calls;
  const ClearanceQuery query = [&calls](const Eigen::Vector3d& point,
                                         const double required) {
    ClearanceQueryResult output = BaseResult(DistanceStatus::UNKNOWN);
    ClearanceCallRecord call;
    call.point = point;
    call.required = required;
    call.result = output;
    calls.push_back(call);
    return output;
  };

  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));

  // The single callback is the primary UNKNOWN.  Since it consumed the
  // configured budget, no callback at required_clearance may be issued.
  ASSERT_EQ(calls.size(), 1U);
  EXPECT_EQ(calls.front().result.status, DistanceStatus::UNKNOWN);
  EXPECT_GT(calls.front().required, 0.40);
  EXPECT_EQ(result.query_sample_count, 1U);
  EXPECT_EQ(result.clearance_leaf_cell_count, 1U);
  EXPECT_TRUE(result.query_budget_reached);
  EXPECT_TRUE(result.limit_exceeded);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::QUERY_BUDGET);
  std::size_t base_callback_count = 0U;
  for (const ClearanceCallRecord& call : calls) {
    if (IsBaseProbeRadius(call.required)) ++base_callback_count;
  }
  EXPECT_EQ(base_callback_count, 0U);
  EXPECT_EQ(result.split_w_count + result.split_v_count +
                result.split_both_count,
            0U);
  ASSERT_FALSE(result.cell_evidence.empty());
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    EXPECT_EQ(evidence.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
    EXPECT_EQ(evidence.inconclusive_reason,
              TubeSurfaceInconclusiveReason::QUERY_BUDGET);
  }
}

TEST(TubeSurfaceValidatorTest,
     DomainLimitedRefinementRecomputesChildInputsIndependently) {
  // A zero-width profile makes the midpoint-position certificate the sole
  // cover term.  Splitting in w therefore has a strictly larger certified
  // reduction than v-only, while the existing tie order still remains in
  // charge of the choice.
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::vector<ClearanceCallRecord> clearance_calls;
  std::vector<double> path_state_w;
  std::vector<double> certificate_w0;
  std::vector<double> certificate_w1;
  const PathStateQuery path_state = [base = LinePath(), &path_state_w](
      const double w, phase_offset_core::PathDifferentialState& state) {
    path_state_w.push_back(w);
    return base(w, state);
  };
  const PathCellBoundQuery base_cells = CertifiedLineCells(1.0, 0.0, 2.0);
  const PathCellBoundQuery cells = [base_cells, &certificate_w0,
                                   &certificate_w1](
      const double w0, const double w1,
      phase_offset_core::PathCellGeometryCertificate& certificate) {
    certificate_w0.push_back(w0);
    certificate_w1.push_back(w1);
    return base_cells(w0, w1, certificate);
  };
  const ClearanceQuery query = [&clearance_calls](
      const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult output;
    if (IsBaseProbeRadius(required) &&
        std::abs(point.x() - 0.10) <= 1e-12) {
      output = BaseResult(DistanceStatus::KNOWN_FREE, 0.40, true, false);
    } else if (std::abs(point.x() - 0.10) <= 1e-12) {
      output = BaseResult(DistanceStatus::UNKNOWN);
    } else {
      output = BaseResult(DistanceStatus::KNOWN_FREE, 5.0, true, true);
    }
    ClearanceCallRecord call;
    call.point = point;
    call.required = required;
    call.result = output;
    clearance_calls.push_back(call);
    return output;
  };

  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, path_state, cells, query, 0.05, 0.40, 0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_DOUBLE_EQ(result.support_alignment_bound, 0.0);
  EXPECT_GT(result.split_w_count, 0U);
  EXPECT_EQ(result.split_v_count, 0U);
  EXPECT_EQ(result.split_both_count, 0U);

  std::size_t parent_primary_count = 0U;
  std::size_t base_callback_count = 0U;
  std::size_t left_child_callback_count = 0U;
  std::size_t right_child_callback_count = 0U;
  double parent_requested_radius = 0.0;
  for (const ClearanceCallRecord& call : clearance_calls) {
    if (std::abs(call.point.x() - 0.10) <= 1e-12) {
      if (IsBaseProbeRadius(call.required)) {
        ++base_callback_count;
        EXPECT_EQ(call.result.status, DistanceStatus::KNOWN_FREE);
      } else {
        ++parent_primary_count;
        parent_requested_radius = call.required;
        EXPECT_EQ(call.result.status, DistanceStatus::UNKNOWN);
      }
    }
    if (std::abs(call.point.x() - 0.05) <= 1e-12) {
      ++left_child_callback_count;
      EXPECT_EQ(call.result.status, DistanceStatus::KNOWN_FREE);
    }
    if (std::abs(call.point.x() - 0.15) <= 1e-12) {
      ++right_child_callback_count;
      EXPECT_EQ(call.result.status, DistanceStatus::KNOWN_FREE);
    }
  }
  EXPECT_EQ(clearance_calls.size(), 5U);
  EXPECT_EQ(result.query_sample_count, clearance_calls.size());
  EXPECT_EQ(result.clearance_leaf_cell_count + 1U,
            result.query_sample_count);
  EXPECT_EQ(parent_primary_count, 1U);
  EXPECT_EQ(base_callback_count, 1U);
  EXPECT_EQ(left_child_callback_count, 1U);
  EXPECT_EQ(right_child_callback_count, 1U);
  EXPECT_GT(parent_requested_radius, 0.40);

  const TubeSurfaceCellEvidence* left =
      FindCellEvidence(result, 0.0, 0.10);
  const TubeSurfaceCellEvidence* right =
      FindCellEvidence(result, 0.10, 0.20);
  ASSERT_NE(left, nullptr);
  ASSERT_NE(right, nullptr);
  EXPECT_EQ(left->outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_EQ(right->outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_TRUE(left->cell_certificate_attempted);
  EXPECT_TRUE(left->cell_certificate_complete);
  EXPECT_TRUE(right->cell_certificate_attempted);
  EXPECT_TRUE(right->cell_certificate_complete);
  EXPECT_TRUE(left->witness_valid);
  EXPECT_TRUE(right->witness_valid);
  EXPECT_DOUBLE_EQ(left->center_w, 0.05);
  EXPECT_DOUBLE_EQ(right->center_w, 0.15);
  EXPECT_DOUBLE_EQ(left->witness_x, 0.05);
  EXPECT_DOUBLE_EQ(right->witness_x, 0.15);
  EXPECT_TRUE(left->requested_clearance_valid);
  EXPECT_TRUE(right->requested_clearance_valid);
  EXPECT_GT(left->requested_clearance, 0.40);
  EXPECT_GT(right->requested_clearance, 0.40);
  EXPECT_LT(left->requested_clearance, parent_requested_radius);
  EXPECT_LT(right->requested_clearance, parent_requested_radius);
  EXPECT_GT(result.max_cover_radius, left->geometric_cover);
  EXPECT_GT(result.max_cover_radius, right->geometric_cover);
  EXPECT_GT(left->geometric_cover, 0.0);
  EXPECT_GT(right->geometric_cover, 0.0);
  EXPECT_EQ(left->clearance_status, DistanceStatus::KNOWN_FREE);
  EXPECT_EQ(right->clearance_status, DistanceStatus::KNOWN_FREE);
  EXPECT_TRUE(left->witness_clearance_certified);
  EXPECT_TRUE(right->witness_clearance_certified);

  bool parent_certificate_seen = false;
  bool left_certificate_seen = false;
  bool right_certificate_seen = false;
  for (std::size_t index = 0U; index < certificate_w0.size(); ++index) {
    parent_certificate_seen = parent_certificate_seen ||
        (std::abs(certificate_w0[index]) <= 1e-12 &&
         std::abs(certificate_w1[index] - 0.20) <= 1e-12);
    left_certificate_seen = left_certificate_seen ||
        (std::abs(certificate_w0[index]) <= 1e-12 &&
         std::abs(certificate_w1[index] - 0.10) <= 1e-12);
    right_certificate_seen = right_certificate_seen ||
        (std::abs(certificate_w0[index] - 0.10) <= 1e-12 &&
         std::abs(certificate_w1[index] - 0.20) <= 1e-12);
  }
  EXPECT_TRUE(parent_certificate_seen);
  EXPECT_TRUE(left_certificate_seen);
  EXPECT_TRUE(right_certificate_seen);

  bool child_path_state_left = false;
  bool child_path_state_right = false;
  for (const double w : path_state_w) {
    child_path_state_left = child_path_state_left ||
        std::abs(w - 0.05) <= 1e-12;
    child_path_state_right = child_path_state_right ||
        std::abs(w - 0.15) <= 1e-12;
  }
  EXPECT_TRUE(child_path_state_left);
  EXPECT_TRUE(child_path_state_right);
  const ClearanceCallRecord* left_callback = nullptr;
  const ClearanceCallRecord* right_callback = nullptr;
  for (const ClearanceCallRecord& call : clearance_calls) {
    if (std::abs(call.point.x() - 0.05) <= 1e-12) {
      left_callback = &call;
    }
    if (std::abs(call.point.x() - 0.15) <= 1e-12) {
      right_callback = &call;
    }
  }
  ASSERT_NE(left_callback, nullptr);
  ASSERT_NE(right_callback, nullptr);
  EXPECT_EQ(0, std::memcmp(&left->witness_x, &left_callback->point.x(),
                           sizeof(double)));
  EXPECT_EQ(0, std::memcmp(&right->witness_x, &right_callback->point.x(),
                           sizeof(double)));
  EXPECT_NE(0, std::memcmp(&left->witness_x, &right->witness_x,
                           sizeof(double)));
}

TEST(TubeSurfaceValidatorTest,
     RecursiveProofInducedUnknownRefinesAgainBeforeSafeLeaves) {
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 4;
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::vector<ClearanceCallRecord> calls;
  const PathCellBoundQuery cells = CertifiedLineCells(1.0, 0.0, 2.0);
  const ClearanceQuery query = [&calls](const Eigen::Vector3d& point,
                                         const double required) {
    ClearanceQueryResult output;
    if (IsBaseProbeRadius(required)) {
      output = BaseResult(DistanceStatus::KNOWN_FREE, 0.40, true, false);
    } else if (std::abs(point.x() - 0.10) <= 1e-12 ||
               std::abs(point.x() - 0.05) <= 1e-12) {
      output = BaseResult(DistanceStatus::UNKNOWN);
    } else {
      output = BaseResult(DistanceStatus::KNOWN_FREE, 5.0, true, true);
    }
    ClearanceCallRecord call;
    call.point = point;
    call.required = required;
    call.result = output;
    calls.push_back(call);
    return output;
  };

  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), cells, query, 0.05, 0.40, 0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_DOUBLE_EQ(result.support_alignment_bound, 0.0);
  EXPECT_EQ(result.split_w_count, 2U);
  EXPECT_EQ(result.split_v_count, 0U);
  EXPECT_EQ(result.split_both_count, 0U);
  EXPECT_EQ(result.query_sample_count, calls.size());
  EXPECT_EQ(calls.size(), 8U);
  EXPECT_EQ(result.clearance_leaf_cell_count, 6U);

  // The callback sequence is: anchor SAFE; parent UNKNOWN/base REFINE;
  // one child UNKNOWN/base REFINE; the other child and both grandchildren
  // are independently SAFE leaves.
  ASSERT_EQ(calls[0].result.status, DistanceStatus::KNOWN_FREE);
  ASSERT_EQ(calls[1].result.status, DistanceStatus::UNKNOWN);
  ASSERT_EQ(calls[2].result.status, DistanceStatus::KNOWN_FREE);
  ASSERT_EQ(calls[3].result.status, DistanceStatus::UNKNOWN);
  ASSERT_EQ(calls[4].result.status, DistanceStatus::KNOWN_FREE);
  ASSERT_EQ(calls[5].result.status, DistanceStatus::KNOWN_FREE);
  ASSERT_EQ(calls[6].result.status, DistanceStatus::KNOWN_FREE);
  ASSERT_EQ(calls[7].result.status, DistanceStatus::KNOWN_FREE);
  EXPECT_NEAR(calls[0].point.x(), 0.0, 1e-12);
  EXPECT_NEAR(calls[1].point.x(), 0.10, 1e-12);
  EXPECT_NEAR(calls[2].point.x(), 0.10, 1e-12);
  EXPECT_NEAR(calls[3].point.x(), 0.05, 1e-12);
  EXPECT_NEAR(calls[4].point.x(), 0.05, 1e-12);
  EXPECT_NEAR(calls[5].point.x(), 0.15, 1e-12);
  EXPECT_NEAR(calls[6].point.x(), 0.025, 1e-12);
  EXPECT_NEAR(calls[7].point.x(), 0.075, 1e-12);
  EXPECT_GT(calls[1].required, calls[3].required);
  EXPECT_DOUBLE_EQ(calls[2].required, 0.40);
  EXPECT_DOUBLE_EQ(calls[4].required, 0.40);
  EXPECT_EQ(calls[2].result.clearance, 0.40);
  EXPECT_EQ(calls[4].result.clearance, 0.40);
  EXPECT_TRUE(calls[2].result.clearance_certified);
  EXPECT_TRUE(calls[4].result.clearance_certified);
  EXPECT_FALSE(calls[2].result.clearance_is_exact);
  EXPECT_FALSE(calls[4].result.clearance_is_exact);
  EXPECT_EQ(0, std::memcmp(&calls[1].point.x(), &calls[2].point.x(),
                           sizeof(double)));
  EXPECT_EQ(0, std::memcmp(&calls[3].point.x(), &calls[4].point.x(),
                           sizeof(double)));

  std::size_t base_callback_count = 0U;
  for (const ClearanceCallRecord& call : calls) {
    if (IsBaseProbeRadius(call.required)) ++base_callback_count;
  }
  EXPECT_EQ(base_callback_count, 2U);
  ASSERT_FALSE(result.cell_evidence.empty());
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    EXPECT_EQ(evidence.outcome, TubeSurfaceOutcome::SAFE);
    EXPECT_NE(evidence.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
    EXPECT_TRUE(evidence.terminal);
  }
}

TEST(TubeSurfaceValidatorTest,
     PrimaryOutOfMapDoesNotEnterUnknownBaseProbePath) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::vector<ClearanceCallRecord> calls;
  const ClearanceQuery query = [&calls](const Eigen::Vector3d& point,
                                         const double required) {
    ClearanceQueryResult output = BaseResult(DistanceStatus::OUT_OF_MAP);
    ClearanceCallRecord call;
    call.point = point;
    call.required = required;
    call.result = output;
    calls.push_back(call);
    return output;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_OUT_OF_MAP);
  EXPECT_EQ(calls.size(), 2U);
  EXPECT_EQ(result.query_sample_count, calls.size());
  EXPECT_EQ(result.clearance_leaf_cell_count, calls.size());
  std::size_t base_callback_count = 0U;
  for (const ClearanceCallRecord& call : calls) {
    EXPECT_EQ(call.result.status, DistanceStatus::OUT_OF_MAP);
    EXPECT_GT(call.required, 0.40);
    if (IsBaseProbeRadius(call.required)) ++base_callback_count;
  }
  EXPECT_EQ(base_callback_count, 0U);
  EXPECT_EQ(result.split_w_count + result.split_v_count +
                result.split_both_count,
            0U);
}

TEST(TubeSurfaceValidatorTest,
     PrimaryUnavailableDoesNotEnterUnknownBaseProbePath) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::vector<ClearanceCallRecord> calls;
  const ClearanceQuery query = [&calls](const Eigen::Vector3d& point,
                                         const double required) {
    ClearanceQueryResult output = BaseResult(DistanceStatus::UNAVAILABLE);
    ClearanceCallRecord call;
    call.point = point;
    call.required = required;
    call.result = output;
    calls.push_back(call);
    return output;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_UNAVAILABLE);
  EXPECT_EQ(calls.size(), 2U);
  EXPECT_EQ(result.query_sample_count, calls.size());
  EXPECT_EQ(result.clearance_leaf_cell_count, calls.size());
  std::size_t base_callback_count = 0U;
  for (const ClearanceCallRecord& call : calls) {
    EXPECT_EQ(call.result.status, DistanceStatus::UNAVAILABLE);
    EXPECT_GT(call.required, 0.40);
    if (IsBaseProbeRadius(call.required)) ++base_callback_count;
  }
  EXPECT_EQ(base_callback_count, 0U);
  EXPECT_EQ(result.split_w_count + result.split_v_count +
                result.split_both_count,
            0U);
}

TEST(TubeSurfaceValidatorTest,
     PrimaryUncertifiedKnownFreeDoesNotEnterUnknownBaseProbePath) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::vector<ClearanceCallRecord> calls;
  const ClearanceQuery query = [&calls](const Eigen::Vector3d& point,
                                         const double required) {
    ClearanceQueryResult output =
        BaseResult(DistanceStatus::KNOWN_FREE, 5.0, false, true);
    ClearanceCallRecord call;
    call.point = point;
    call.required = required;
    call.result = output;
    calls.push_back(call);
    return output;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_UNCERTIFIED);
  EXPECT_EQ(calls.size(), 2U);
  EXPECT_EQ(result.query_sample_count, calls.size());
  EXPECT_EQ(result.clearance_leaf_cell_count, calls.size());
  std::size_t base_callback_count = 0U;
  for (const ClearanceCallRecord& call : calls) {
    EXPECT_EQ(call.result.status, DistanceStatus::KNOWN_FREE);
    EXPECT_FALSE(call.result.clearance_certified);
    EXPECT_GT(call.required, 0.40);
    if (IsBaseProbeRadius(call.required)) ++base_callback_count;
  }
  EXPECT_EQ(base_callback_count, 0U);
  EXPECT_EQ(result.split_w_count + result.split_v_count +
                result.split_both_count,
            0U);
}

TEST(TubeSurfaceValidatorTest,
     CertificateFailureStopsBeforeAnyClearanceOrBaseProbe) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::size_t certificate_calls = 0U;
  std::vector<ClearanceCallRecord> clearance_calls;
  const PathCellBoundQuery cells = [&certificate_calls](
      const double, const double,
      phase_offset_core::PathCellGeometryCertificate&) {
    ++certificate_calls;
    return false;
  };
  const ClearanceQuery query = [&clearance_calls](
      const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult output =
        BaseResult(DistanceStatus::KNOWN_FREE, 5.0, true, true);
    ClearanceCallRecord call;
    call.point = point;
    call.required = required;
    call.result = output;
    clearance_calls.push_back(call);
    return output;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), cells, query, 0.05, 0.40, 0.10, result));
  EXPECT_GT(certificate_calls, 0U);
  ASSERT_EQ(clearance_calls.size(), 1U);
  EXPECT_GT(clearance_calls.front().required, 0.40);
  EXPECT_FALSE(IsBaseProbeRadius(clearance_calls.front().required));
  EXPECT_EQ(result.query_sample_count, clearance_calls.size());
  EXPECT_EQ(result.clearance_leaf_cell_count, clearance_calls.size());
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MALFORMED);
  const TubeSurfaceCellEvidence* interval =
      FindCellEvidence(result, 0.0, 0.20);
  ASSERT_NE(interval, nullptr);
  EXPECT_EQ(interval->outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(interval->inconclusive_reason,
            TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MALFORMED);
  EXPECT_FALSE(interval->clearance_query_attempted);
}

TEST(TubeSurfaceValidatorTest,
     RegularityFailureStopsBeforeAnyClearanceOrBaseProbe) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.20, 0.20);
  std::size_t certificate_calls = 0U;
  std::vector<ClearanceCallRecord> clearance_calls;
  const PathCellBoundQuery base_cells = CertifiedLineCells(0.1, 2.0, 0.0);
  const PathCellBoundQuery cells = [base_cells, &certificate_calls](
      const double w0, const double w1,
      phase_offset_core::PathCellGeometryCertificate& certificate) {
    ++certificate_calls;
    return base_cells(w0, w1, certificate);
  };
  const ClearanceQuery query = [&clearance_calls](
      const Eigen::Vector3d& point, const double required) {
    ClearanceQueryResult output =
        BaseResult(DistanceStatus::KNOWN_FREE, 5.0, true, true);
    ClearanceCallRecord call;
    call.point = point;
    call.required = required;
    call.result = output;
    clearance_calls.push_back(call);
    return output;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), cells, query, 0.05, 0.40, 0.10, result));
  EXPECT_GT(certificate_calls, 0U);
  ASSERT_EQ(clearance_calls.size(), 1U);
  EXPECT_GT(clearance_calls.front().required, 0.40);
  EXPECT_FALSE(IsBaseProbeRadius(clearance_calls.front().required));
  EXPECT_EQ(result.query_sample_count, clearance_calls.size());
  EXPECT_EQ(result.clearance_leaf_cell_count, clearance_calls.size());
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::REGULARITY_UNPROVEN);
  const TubeSurfaceCellEvidence* interval =
      FindCellEvidence(result, 0.0, 0.20);
  ASSERT_NE(interval, nullptr);
  EXPECT_EQ(interval->outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(interval->inconclusive_reason,
            TubeSurfaceInconclusiveReason::REGULARITY_UNPROVEN);
  EXPECT_FALSE(interval->clearance_query_attempted);
}

TEST(TubeSurfaceValidatorTest,
     NumericalFailureStopsBeforeAnyClearanceOrBaseProbe) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  std::size_t clearance_calls = 0U;
  const ClearanceQuery query = [&clearance_calls](
      const Eigen::Vector3d&, const double) {
    ++clearance_calls;
    return BaseResult(DistanceStatus::KNOWN_FREE, 5.0, true, true);
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05,
      std::numeric_limits<double>::max(), 0.10, result));
  EXPECT_EQ(clearance_calls, 0U);
  EXPECT_EQ(result.query_sample_count, 0U);
  EXPECT_EQ(result.clearance_leaf_cell_count, 0U);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::NUMERICAL_FAILURE);
  ASSERT_FALSE(result.cell_evidence.empty());
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    EXPECT_EQ(evidence.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
    EXPECT_EQ(evidence.inconclusive_reason,
              TubeSurfaceInconclusiveReason::NUMERICAL_FAILURE);
    EXPECT_FALSE(evidence.clearance_query_attempted);
  }
}

TEST(TubeSurfaceValidatorTest, RefineRecomputesChildrenAndTerminatesSafely) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.20, 0.20);
  const ClearanceQuery query = [](const Eigen::Vector3d&, const double required) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = required < 0.41 ? required : 0.41;
    result.clearance_certified = true;
    result.clearance_is_exact = true;
    return result;
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_GT(result.split_v_count + result.split_w_count +
                result.split_both_count,
            0U);
  EXPECT_GT(result.query_sample_count, result.clearance_leaf_cell_count - 1U);
}

TEST(TubeSurfaceValidatorTest, DepthGuardIsTypedAndNeverInsufficientClearance) {
  TubeSurfaceValidatorConfig config;
  config.max_subdivision_depth = 0;
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.20, 0.20);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(1.0, 0.0, 0.5),
      ExactClearance(0.41), 0.05, 0.40, 0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason, TubeSurfaceInconclusiveReason::DEPTH_GUARD);
  EXPECT_TRUE(result.limit_exceeded);
  EXPECT_TRUE(result.depth_guard_reached);
  EXPECT_NE(result.first_failure_reason, TubeStopReason::INSUFFICIENT_CLEARANCE);
}

TEST(TubeSurfaceValidatorTest, QueryBudgetIsTypedAndPreservesExactUnsafeEvidence) {
  TubeSurfaceValidatorConfig config;
  config.max_query_samples = 1U;
  TubeProfile profile = MakeProfile({0.0, 0.20}, 0.0, 0.0);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), ExactClearance(0.39),
      0.05, 0.40, 0.10, result));
  EXPECT_TRUE(result.limit_exceeded || result.query_budget_reached);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
}

TEST(TubeSurfaceValidatorTest,
     QueryBudgetDrainPreservesCachedTerminalInconclusiveReason) {
  TubeSurfaceValidatorConfig config;
  config.max_query_samples = 3U;
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.20, 0.20);
  const ClearanceQuery query = [](const Eigen::Vector3d& point, const double) {
    ClearanceQueryResult result;
    if (point.y() < -0.05) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 0.41;
    result.clearance_certified = true;
    result.clearance_is_exact = true;
    return result;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_TRUE(result.query_budget_reached);

  bool cached_unknown_preserved = false;
  bool cached_refine_drained = false;
  bool unevaluated_drained = false;
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    if (std::abs(evidence.w0) <= 1e-12 &&
        std::abs(evidence.w1) <= 1e-12 &&
        std::abs(evidence.v0) <= 1e-12 &&
        std::abs(evidence.v1 - 0.5) <= 1e-12) {
      EXPECT_EQ(evidence.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
      EXPECT_EQ(evidence.inconclusive_reason,
                TubeSurfaceInconclusiveReason::CLEARANCE_UNKNOWN);
      cached_unknown_preserved = true;
    }
    if (std::abs(evidence.w0) <= 1e-12 &&
        std::abs(evidence.w1) <= 1e-12 &&
        std::abs(evidence.v0 - 0.5) <= 1e-12 &&
        std::abs(evidence.v1 - 1.0) <= 1e-12) {
      EXPECT_EQ(evidence.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
      EXPECT_EQ(evidence.inconclusive_reason,
                TubeSurfaceInconclusiveReason::QUERY_BUDGET);
      cached_refine_drained = true;
    }
    if (std::abs(evidence.w0) <= 1e-12 &&
        std::abs(evidence.w1 - 0.20) <= 1e-12 &&
        std::abs(evidence.v0) <= 1e-12 &&
        std::abs(evidence.v1 - 1.0) <= 1e-12) {
      EXPECT_EQ(evidence.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
      EXPECT_EQ(evidence.inconclusive_reason,
                TubeSurfaceInconclusiveReason::QUERY_BUDGET);
      unevaluated_drained = true;
    }
  }
  EXPECT_TRUE(cached_unknown_preserved);
  EXPECT_TRUE(cached_refine_drained);
  EXPECT_TRUE(unevaluated_drained);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_UNKNOWN);
}

TEST(TubeSurfaceValidatorTest, QueryBudgetDrainConvertsCachedRefineEvidence) {
  TubeSurfaceValidatorConfig config;
  config.max_query_samples = 3U;
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.20, 0.20);
  TubeSurfaceValidationResult result;
  ASSERT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), ExactClearance(0.41),
      0.05, 0.40, 0.10, result));
  EXPECT_TRUE(result.query_budget_reached);

  bool lower_child_drained = false;
  bool upper_child_drained = false;
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    const bool anchor_child = std::abs(evidence.w0) <= 1e-12 &&
        std::abs(evidence.w1) <= 1e-12;
    if (!anchor_child) continue;
    if (std::abs(evidence.v0) <= 1e-12 &&
        std::abs(evidence.v1 - 0.5) <= 1e-12) {
      EXPECT_EQ(evidence.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
      EXPECT_EQ(evidence.inconclusive_reason,
                TubeSurfaceInconclusiveReason::QUERY_BUDGET);
      lower_child_drained = true;
    }
    if (std::abs(evidence.v0 - 0.5) <= 1e-12 &&
        std::abs(evidence.v1 - 1.0) <= 1e-12) {
      EXPECT_EQ(evidence.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
      EXPECT_EQ(evidence.inconclusive_reason,
                TubeSurfaceInconclusiveReason::QUERY_BUDGET);
      upper_child_drained = true;
    }
  }
  EXPECT_TRUE(lower_child_drained);
  EXPECT_TRUE(upper_child_drained);
}

TEST(TubeSurfaceValidatorTest,
     QueryBudgetDrainConvertsUnevaluatedWorkItems) {
  TubeSurfaceValidatorConfig config;
  config.max_query_samples = 1U;
  TubeProfile profile = MakeProfile({0.0, 0.20});
  TubeSurfaceValidationResult result;
  ASSERT_FALSE(TubeSurfaceValidator(config).validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), OpenLowerBound(), 0.05,
      0.40, 0.10, result));
  EXPECT_TRUE(result.query_budget_reached);

  bool anchor_safe = false;
  bool unevaluated_interval_drained = false;
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    if (std::abs(evidence.w0) <= 1e-12 &&
        std::abs(evidence.w1) <= 1e-12 &&
        std::abs(evidence.v0) <= 1e-12 &&
        std::abs(evidence.v1 - 1.0) <= 1e-12) {
      EXPECT_EQ(evidence.outcome, TubeSurfaceOutcome::SAFE);
      anchor_safe = true;
    }
    if (std::abs(evidence.w0) <= 1e-12 &&
        std::abs(evidence.w1 - 0.20) <= 1e-12 &&
        std::abs(evidence.v0) <= 1e-12 &&
        std::abs(evidence.v1 - 1.0) <= 1e-12) {
      EXPECT_EQ(evidence.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
      EXPECT_EQ(evidence.inconclusive_reason,
                TubeSurfaceInconclusiveReason::QUERY_BUDGET);
      unevaluated_interval_drained = true;
    }
  }
  EXPECT_TRUE(anchor_safe);
  EXPECT_TRUE(unevaluated_interval_drained);
}

TEST(TubeSurfaceValidatorTest, OneCentreWitnessPerLeaf) {
  TubeProfile profile = MakeProfile({0.0, 0.20});
  std::size_t calls = 0U;
  const ClearanceQuery query = [base = OpenLowerBound(), &calls](
      const Eigen::Vector3d& point, const double required) {
    ++calls;
    return base(point, required);
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(calls, result.clearance_leaf_cell_count);
  EXPECT_EQ(result.query_sample_count, result.clearance_leaf_cell_count);
}

TEST(TubeSurfaceValidatorTest, OffCentreUnsafeRegionCannotBeAccepted) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20}, -0.20, 0.20);
  bool root_centre_seen = false;
  bool root_centre_unsafe = false;
  bool off_centre_child_seen = false;
  const ClearanceQuery query = [&root_centre_seen, &root_centre_unsafe,
                                &off_centre_child_seen](
      const Eigen::Vector3d& point, const double) {
    ClearanceQueryResult result;
    const bool root_centre = std::abs(point.x() - 0.10) <= 1e-12 &&
        std::abs(point.y()) <= 1e-12;
    const bool off_centre_child = std::abs(point.x() - 0.10) <= 1e-12 &&
        std::abs(point.y() - 0.10) <= 1e-12;
    if (root_centre) {
      root_centre_seen = true;
      result.status = DistanceStatus::KNOWN_FREE;
      result.clearance = 0.41;
      result.clearance_certified = true;
      result.clearance_is_exact = true;
      return result;
    }
    if (off_centre_child) {
      off_centre_child_seen = true;
      result.status = DistanceStatus::OCCUPIED;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 0.41;
    result.clearance_certified = true;
    result.clearance_is_exact = true;
    return result;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.10, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_TRUE(root_centre_seen);
  EXPECT_FALSE(root_centre_unsafe);
  EXPECT_TRUE(off_centre_child_seen);
  EXPECT_GT(result.split_v_count + result.split_w_count +
                result.split_both_count,
            0U);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
  bool child_contract_unsafe = false;
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    if (std::abs(evidence.w0 - 0.10) <= 1e-12 &&
        std::abs(evidence.w1 - 0.10) <= 1e-12 &&
        std::abs(evidence.v0 - 0.50) <= 1e-12 &&
        std::abs(evidence.v1 - 1.00) <= 1e-12 &&
        evidence.outcome == TubeSurfaceOutcome::CONTRACT_UNSAFE &&
        evidence.witness_legacy_reason == TubeStopReason::OCCUPIED) {
      child_contract_unsafe = true;
    }
  }
  EXPECT_TRUE(child_contract_unsafe);
}

TEST(TubeSurfaceValidatorTest, KnotFlagsAloneCannotCertifyCentreline) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20, 0.30, 0.40});
  std::size_t certificate_calls = 0U;
  bool left_outer_seen = false;
  bool right_outer_seen = false;
  bool left_gap_seen = false;
  bool right_gap_seen = false;
  const PathCellBoundQuery cells = [&certificate_calls, &left_outer_seen,
                                    &right_outer_seen, &left_gap_seen,
                                    &right_gap_seen](
      const double w0, const double w1,
      phase_offset_core::PathCellGeometryCertificate& certificate) {
    ++certificate_calls;
    const bool left_outer = std::abs(w0 - 0.0) <= 1e-12 &&
        std::abs(w1 - 0.10) <= 1e-12;
    const bool right_outer = std::abs(w0 - 0.30) <= 1e-12 &&
        std::abs(w1 - 0.40) <= 1e-12;
    const bool left_gap = std::abs(w0 - 0.10) <= 1e-12 &&
        std::abs(w1 - 0.20) <= 1e-12;
    const bool right_gap = std::abs(w0 - 0.20) <= 1e-12 &&
        std::abs(w1 - 0.30) <= 1e-12;
    if (left_gap) left_gap_seen = true;
    if (right_gap) right_gap_seen = true;
    if (!left_outer && !right_outer) return false;
    if (left_outer) left_outer_seen = true;
    if (right_outer) right_outer_seen = true;
    certificate = phase_offset_core::PathCellGeometryCertificate();
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.segment_w0 = 0.0;
    certificate.segment_w1 = 0.40;
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
    certificate.midpoint_position_variation_bound = 0.0;
    certificate.chord_deviation_bound = 0.0;
    certificate.valid = std::isfinite(w0) && std::isfinite(w1) && w1 > w0;
    certificate.complete = certificate.valid;
    return certificate.valid;
  };
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.20, LinePath(), cells, OpenLowerBound(), 0.05, 0.40, 0.10,
      result));
  EXPECT_TRUE(left_outer_seen);
  EXPECT_TRUE(right_outer_seen);
  EXPECT_TRUE(left_gap_seen);
  EXPECT_TRUE(right_gap_seen);
  EXPECT_EQ(certificate_calls, 4U);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MALFORMED);
  EXPECT_FALSE(result.zero_centerline_continuously_certified);
  EXPECT_NE(profile.proof_level, TubeProofLevel::CONTINUOUS_COVER_PROOF);

  for (const double knot : {0.0, 0.10, 0.30, 0.40}) {
    const TubeValidatorKnotEvidence* evidence = nullptr;
    for (const TubeValidatorKnotEvidence& candidate :
             profile.validator_knot_evidence) {
      if (std::abs(candidate.w - knot) <= 1e-12) {
        evidence = &candidate;
        break;
      }
    }
    ASSERT_NE(evidence, nullptr);
    EXPECT_TRUE(evidence->observed);
    EXPECT_TRUE(evidence->filtered_contains_zero);
    EXPECT_TRUE(evidence->zero_surface_covered);
  }

  bool left_gap_evidence = false;
  bool right_gap_evidence = false;
  bool any_gap_safe = false;
  for (const TubeSurfaceCellEvidence& evidence : profile.surface_cell_evidence) {
    if (std::abs(evidence.w0 - 0.10) <= 1e-12 &&
        std::abs(evidence.w1 - 0.20) <= 1e-12) {
      left_gap_evidence = true;
      any_gap_safe = any_gap_safe || evidence.outcome == TubeSurfaceOutcome::SAFE;
    }
    if (std::abs(evidence.w0 - 0.20) <= 1e-12 &&
        std::abs(evidence.w1 - 0.30) <= 1e-12) {
      right_gap_evidence = true;
      any_gap_safe = any_gap_safe || evidence.outcome == TubeSurfaceOutcome::SAFE;
    }
  }
  EXPECT_TRUE(left_gap_evidence);
  EXPECT_TRUE(right_gap_evidence);
  EXPECT_FALSE(any_gap_safe);
}

TEST(TubeSurfaceValidatorTest, ContiguousCellEvidenceCertifiesCentreline) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20});
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), OpenLowerBound(), 0.05,
      0.40, 0.10, result));
  EXPECT_TRUE(result.zero_centerline_continuously_certified);
  EXPECT_DOUBLE_EQ(result.certified_start_w, 0.0);
  EXPECT_DOUBLE_EQ(result.certified_end_w, 0.20);
}

TEST(TubeSurfaceValidatorTest, SampledFallbackRemainsInconclusive) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.10, 0.10, false);
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), OpenLowerBound(), 0.05, 0.40, 0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(result.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MISSING);
  EXPECT_FALSE(profile.zero_centerline_continuously_certified);
}

TEST(TubeSurfaceValidatorTest, TraversalDeterminismUsesCanonicalEvidence) {
  const ClearanceQuery query = [](const Eigen::Vector3d&, const double required) {
    ClearanceQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = required < 0.41 ? required : 0.41;
    result.clearance_certified = true;
    result.clearance_is_exact = true;
    return result;
  };
  TubeProfile first = MakeProfile({0.0, 0.20}, -0.20, 0.20);
  TubeProfile second = first;
  TubeSurfaceValidationResult first_result;
  TubeSurfaceValidationResult second_result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      first, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40, 0.10,
      first_result));
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      second, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40, 0.10,
      second_result));
  EXPECT_EQ(first_result.outcome, second_result.outcome);
  EXPECT_EQ(first_result.truncation_outcome, second_result.truncation_outcome);
  EXPECT_EQ(first_result.first_failure_reason, second_result.first_failure_reason);
  EXPECT_EQ(first_result.split_w_count, second_result.split_w_count);
  EXPECT_EQ(first_result.split_v_count, second_result.split_v_count);
  EXPECT_EQ(first_result.split_both_count, second_result.split_both_count);
}

TEST(TubeSurfaceValidatorTest, FarUnsafeSuffixRetainsSafePrefixAndCanonicalTerminal) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20, 0.30});
  const ClearanceQuery query = [](const Eigen::Vector3d& point, const double) {
    ClearanceQueryResult result;
    if (point.x() >= 0.15) {
      result.status = DistanceStatus::OCCUPIED;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 5.0;
    result.clearance_certified = true;
    result.clearance_is_exact = true;
    return result;
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_TRUE(result.truncated_after);
  EXPECT_EQ(result.truncation_outcome, TubeSurfaceTruncationOutcome::SUFFIX);
  EXPECT_DOUBLE_EQ(result.terminal_w, 0.10);
  EXPECT_EQ(result.first_failure_reason, TubeStopReason::OCCUPIED);
  EXPECT_DOUBLE_EQ(profile.first_truncated_w, 0.10);
  EXPECT_EQ(profile.first_truncated_reason, TubeStopReason::OCCUPIED);
}

TEST(TubeSurfaceValidatorTest, StaleContinuousProofIsClearedOnFailure) {
  TubeProfile profile = MakeProfile({0.0, 0.20}, -0.10, 0.10, false);
  profile.proof_level = TubeProofLevel::CONTINUOUS_COVER_PROOF;
  TubeSurfaceValidationResult result;
  EXPECT_FALSE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), OpenLowerBound(), 0.05, 0.40, 0.10, result));
  EXPECT_NE(profile.proof_level, TubeProofLevel::CONTINUOUS_COVER_PROOF);
}

TEST(TubeSurfaceValidatorTest,
     ForwardExcludedSidecarDoesNotOverwriteBackwardCanonicalFailure) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20, 0.30}, 0.0, 0.0);
  profile.snapshot_sequence = 777U;
  profile.snapshot_provenance_is_immutable = true;
  std::size_t calls = 0U;
  std::size_t path_state_calls = 0U;
  std::size_t path_cell_bound_calls = 0U;
  Eigen::Vector3d forward_query_point = Eigen::Vector3d::Zero();
  const ClearanceQuery query = [&calls, &forward_query_point](
      const Eigen::Vector3d& point, const double) {
    ++calls;
    ClearanceQueryResult result;
    if (point.x() < 0.075) {
      result.status = DistanceStatus::UNKNOWN;
      return result;
    }
    if (point.x() >= 0.20) {
      forward_query_point = point;
      result.status = DistanceStatus::OCCUPIED;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 5.0;
    result.clearance_certified = true;
    result.clearance_is_exact = true;
    return result;
  };
  const PathStateQuery path_state = [base = LinePath(), &path_state_calls](
      const double w, phase_offset_core::PathDifferentialState& state) {
    ++path_state_calls;
    return base(w, state);
  };
  const PathCellBoundQuery path_cells = [base = CertifiedLineCells(),
                                         &path_cell_bound_calls](
      const double w0, const double w1,
      phase_offset_core::PathCellGeometryCertificate& certificate) {
    ++path_cell_bound_calls;
    return base(w0, w1, certificate);
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.10, path_state, path_cells, query, 0.05, 0.40, 0.10,
      result));

  // Existing canonical evidence remains the closer backward UNKNOWN cell.
  EXPECT_EQ(result.first_failure_reason, TubeStopReason::UNKNOWN);
  EXPECT_DOUBLE_EQ(result.first_failure_w, 0.0);
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_TRUE(result.complete);
  EXPECT_TRUE(result.current_anchor_valid);
  EXPECT_EQ(result.truncation_outcome,
            TubeSurfaceTruncationOutcome::PREFIX_AND_SUFFIX);
  EXPECT_TRUE(result.truncated_before);
  EXPECT_TRUE(result.truncated_after);
  EXPECT_TRUE(result.zero_centerline_continuously_certified);
  EXPECT_EQ(profile.proof_level, TubeProofLevel::CONTINUOUS_COVER_PROOF);
  ASSERT_EQ(profile.samples.size(), 2U);
  EXPECT_DOUBLE_EQ(profile.samples.front().w, 0.10);
  EXPECT_DOUBLE_EQ(profile.samples.back().w, 0.20);
  EXPECT_DOUBLE_EQ(profile.preview_start_w, 0.10);
  EXPECT_DOUBLE_EQ(profile.preview_end_w, 0.20);

  const TubeSurfaceForwardExcludedEvidence& forward =
      result.forward_excluded_evidence;
  ASSERT_TRUE(forward.valid);
  EXPECT_DOUBLE_EQ(forward.w0, 0.20);
  EXPECT_DOUBLE_EQ(forward.w1, 0.30);
  EXPECT_EQ(forward.outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
  EXPECT_EQ(forward.inconclusive_reason, TubeSurfaceInconclusiveReason::NONE);
  EXPECT_TRUE(forward.clearance_query_attempted);
  EXPECT_EQ(forward.clearance_status, DistanceStatus::OCCUPIED);
  EXPECT_FALSE(forward.witness_clearance_valid);
  EXPECT_FALSE(forward.witness_clearance_exact);
  EXPECT_FALSE(forward.exact_d_c_valid);
  EXPECT_TRUE(forward.geometric_evidence_valid);
  EXPECT_TRUE(forward.witness_valid);
  EXPECT_TRUE(forward.map_observation_sequence_valid);
  EXPECT_EQ(forward.map_observation_sequence, 777U);
  // The backward UNKNOWN primary query legitimately receives one required-
  // clearance base probe before remaining terminal evidence is serialized.
  EXPECT_EQ(calls, result.query_sample_count);
  EXPECT_EQ(calls, 5U);
  EXPECT_EQ(path_state_calls, 4U);
  EXPECT_EQ(path_cell_bound_calls, 3U);
  EXPECT_EQ(result.split_w_count, 0U);
  EXPECT_EQ(result.split_v_count, 0U);
  EXPECT_EQ(result.split_both_count, 0U);
  EXPECT_EQ(profile.surface_cell_evidence.size(), result.cell_evidence.size());
  EXPECT_EQ(profile.forward_excluded_evidence.w0, forward.w0);
  EXPECT_EQ(profile.forward_excluded_evidence.w1, forward.w1);
  const TubeSurfaceCellEvidence* selected_cell = nullptr;
  for (const TubeSurfaceCellEvidence& cell : result.cell_evidence) {
    if (cell.w0 == forward.w0 && cell.w1 == forward.w1 &&
        cell.v0 == forward.v0 && cell.v1 == forward.v1 &&
        cell.outcome == forward.outcome) {
      selected_cell = &cell;
      break;
    }
  }
  ASSERT_NE(selected_cell, nullptr);
  EXPECT_TRUE(selected_cell->witness_valid);
  EXPECT_DOUBLE_EQ(forward.witness_x, forward_query_point.x());
  EXPECT_DOUBLE_EQ(forward.witness_y, forward_query_point.y());
  EXPECT_DOUBLE_EQ(forward.witness_z, forward_query_point.z());
  EXPECT_DOUBLE_EQ(forward.center_w, selected_cell->center_w);
  EXPECT_DOUBLE_EQ(forward.center_v, selected_cell->center_v);
  EXPECT_DOUBLE_EQ(forward.witness_x, selected_cell->witness_x);
  EXPECT_DOUBLE_EQ(forward.witness_y, selected_cell->witness_y);
  EXPECT_DOUBLE_EQ(forward.witness_z, selected_cell->witness_z);

  // The sidecar carries the exact centre/witness produced by the selected
  // EvaluateCell invocation and the exact point received by the existing
  // clearance callback; no reconstruction from profile samples is allowed.
  EXPECT_DOUBLE_EQ(forward.center_w, 0.25);
  EXPECT_DOUBLE_EQ(forward.center_v, 0.50);
  EXPECT_DOUBLE_EQ(forward.witness_x, 0.25);
  EXPECT_DOUBLE_EQ(forward.witness_y, 0.0);
  EXPECT_DOUBLE_EQ(forward.witness_z, 0.0);
  const double callback_x = forward_query_point.x();
  const double callback_y = forward_query_point.y();
  const double callback_z = forward_query_point.z();
  EXPECT_EQ(0, std::memcmp(&forward.witness_x, &callback_x,
                           sizeof(forward.witness_x)));
  EXPECT_EQ(0, std::memcmp(&forward.witness_y, &callback_y,
                           sizeof(forward.witness_y)));
  EXPECT_EQ(0, std::memcmp(&forward.witness_z, &callback_z,
                           sizeof(forward.witness_z)));

  const auto expect_bitwise_equal = [](const double first,
                                       const double second) {
    EXPECT_EQ(0, std::memcmp(&first, &second, sizeof(double)));
  };
  expect_bitwise_equal(forward.w0, selected_cell->w0);
  expect_bitwise_equal(forward.w1, selected_cell->w1);
  expect_bitwise_equal(forward.v0, selected_cell->v0);
  expect_bitwise_equal(forward.v1, selected_cell->v1);
  expect_bitwise_equal(forward.witness_clearance,
                       selected_cell->witness_clearance);
  expect_bitwise_equal(forward.exact_d_c, selected_cell->witness_clearance);
  expect_bitwise_equal(forward.requested_clearance,
                       selected_cell->requested_clearance);
  expect_bitwise_equal(forward.midpoint_position_cover,
                       selected_cell->midpoint_position_cover);
  expect_bitwise_equal(forward.normal_variation_cover,
                       selected_cell->normal_variation_cover);
  expect_bitwise_equal(forward.delta_slope_cover,
                       selected_cell->delta_slope_cover);
  expect_bitwise_equal(forward.v_span_cover, selected_cell->v_span_cover);
  expect_bitwise_equal(forward.geometric_cover,
                       selected_cell->geometric_cover);
  expect_bitwise_equal(forward.support_alignment_bound,
                       selected_cell->support_alignment_bound);
  expect_bitwise_equal(forward.numerical_epsilon,
                       selected_cell->numerical_epsilon);
  expect_bitwise_equal(forward.allowable_cover,
                       selected_cell->allowable_cover);
  expect_bitwise_equal(forward.proof_residual, selected_cell->proof_residual);
  expect_bitwise_equal(forward.center_w, selected_cell->center_w);
  expect_bitwise_equal(forward.center_v, selected_cell->center_v);
  expect_bitwise_equal(forward.witness_x, selected_cell->witness_x);
  expect_bitwise_equal(forward.witness_y, selected_cell->witness_y);
  expect_bitwise_equal(forward.witness_z, selected_cell->witness_z);
}

TEST(TubeSurfaceValidatorTest,
     ForwardExcludedUnknownThenOccupiedSerializesBaseEvidence) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20, 0.30}, 0.0, 0.0);
  std::vector<ClearanceCallRecord> calls;
  const ClearanceQuery query = [&calls](const Eigen::Vector3d& point,
                                         const double required) {
    ClearanceQueryResult output;
    const bool forward_interval = point.x() > 0.10 + 1e-12 &&
        point.x() < 0.20 - 1e-12;
    if (!forward_interval) {
      output = BaseResult(DistanceStatus::KNOWN_FREE, 5.0, true, true);
    } else if (required > 0.40) {
      output = BaseResult(DistanceStatus::UNKNOWN);
    } else {
      output = BaseResult(DistanceStatus::OCCUPIED);
    }
    ClearanceCallRecord call;
    call.point = point;
    call.required = required;
    call.result = output;
    calls.push_back(call);
    return output;
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.10, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_TRUE(result.truncated_after);
  EXPECT_EQ(result.truncation_outcome, TubeSurfaceTruncationOutcome::SUFFIX);
  ASSERT_EQ(calls.size(), 5U);
  ASSERT_EQ(calls[2].result.status, DistanceStatus::UNKNOWN);
  ASSERT_EQ(calls[3].result.status, DistanceStatus::OCCUPIED);
  EXPECT_GT(calls[2].required, 0.40);
  EXPECT_DOUBLE_EQ(calls[3].required, 0.40);
  EXPECT_EQ(result.query_sample_count, calls.size());
  EXPECT_EQ(result.clearance_leaf_cell_count + 1U, calls.size());

  const TubeSurfaceForwardExcludedEvidence& forward =
      result.forward_excluded_evidence;
  ASSERT_TRUE(forward.valid);
  EXPECT_DOUBLE_EQ(forward.w0, 0.10);
  EXPECT_DOUBLE_EQ(forward.w1, 0.20);
  EXPECT_EQ(forward.outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
  EXPECT_EQ(forward.inconclusive_reason, TubeSurfaceInconclusiveReason::NONE);
  EXPECT_TRUE(forward.clearance_query_attempted);
  EXPECT_EQ(forward.clearance_status, DistanceStatus::OCCUPIED);
  EXPECT_FALSE(forward.witness_clearance_valid);
  EXPECT_FALSE(forward.witness_clearance_certified);
  EXPECT_FALSE(forward.witness_clearance_exact);
  EXPECT_FALSE(forward.exact_d_c_valid);
  EXPECT_EQ(forward.witness_clearance, 0.0);
  EXPECT_TRUE(forward.requested_clearance_valid);
  EXPECT_DOUBLE_EQ(forward.requested_clearance, 0.40);
  EXPECT_FALSE(forward.outcome == TubeSurfaceOutcome::CONTRACT_UNSAFE &&
               forward.clearance_status == DistanceStatus::UNKNOWN);

  const TubeSurfaceCellEvidence* selected = FindCellEvidence(
      result, 0.10, 0.20);
  ASSERT_NE(selected, nullptr);
  EXPECT_EQ(selected->outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
  EXPECT_EQ(selected->clearance_status, DistanceStatus::OCCUPIED);
  EXPECT_EQ(selected->witness_legacy_reason, TubeStopReason::OCCUPIED);
  EXPECT_DOUBLE_EQ(selected->requested_clearance, 0.40);
}

TEST(TubeSurfaceValidatorTest,
     ForwardExcludedUnknownThenBaseUnknownSerializesBaseEvidence) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20, 0.30}, 0.0, 0.0);
  std::vector<ClearanceCallRecord> calls;
  const ClearanceQuery query = [&calls](const Eigen::Vector3d& point,
                                         const double required) {
    ClearanceQueryResult output;
    const bool forward_interval = point.x() > 0.10 + 1e-12 &&
        point.x() < 0.20 - 1e-12;
    if (!forward_interval) {
      output = BaseResult(DistanceStatus::KNOWN_FREE, 5.0, true, true);
    } else {
      output = BaseResult(DistanceStatus::UNKNOWN);
    }
    ClearanceCallRecord call;
    call.point = point;
    call.required = required;
    call.result = output;
    calls.push_back(call);
    return output;
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.10, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  EXPECT_EQ(result.outcome, TubeSurfaceOutcome::SAFE);
  EXPECT_TRUE(result.truncated_after);
  EXPECT_EQ(result.truncation_outcome, TubeSurfaceTruncationOutcome::SUFFIX);
  ASSERT_EQ(calls.size(), 5U);
  ASSERT_EQ(calls[2].result.status, DistanceStatus::UNKNOWN);
  ASSERT_EQ(calls[3].result.status, DistanceStatus::UNKNOWN);
  EXPECT_GT(calls[2].required, 0.40);
  EXPECT_DOUBLE_EQ(calls[3].required, 0.40);
  EXPECT_EQ(result.query_sample_count, calls.size());
  EXPECT_EQ(result.clearance_leaf_cell_count + 1U, calls.size());

  const TubeSurfaceForwardExcludedEvidence& forward =
      result.forward_excluded_evidence;
  ASSERT_TRUE(forward.valid);
  EXPECT_DOUBLE_EQ(forward.w0, 0.10);
  EXPECT_DOUBLE_EQ(forward.w1, 0.20);
  EXPECT_EQ(forward.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(forward.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_UNKNOWN);
  EXPECT_TRUE(forward.clearance_query_attempted);
  EXPECT_EQ(forward.clearance_status, DistanceStatus::UNKNOWN);
  EXPECT_FALSE(forward.witness_clearance_valid);
  EXPECT_FALSE(forward.witness_clearance_certified);
  EXPECT_FALSE(forward.witness_clearance_exact);
  EXPECT_FALSE(forward.exact_d_c_valid);
  EXPECT_EQ(forward.witness_clearance, 0.0);
  EXPECT_TRUE(forward.requested_clearance_valid);
  EXPECT_DOUBLE_EQ(forward.requested_clearance, 0.40);

  const TubeSurfaceCellEvidence* selected = FindCellEvidence(
      result, 0.10, 0.20);
  ASSERT_NE(selected, nullptr);
  EXPECT_EQ(selected->outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(selected->inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_UNKNOWN);
  EXPECT_EQ(selected->clearance_status, DistanceStatus::UNKNOWN);
  EXPECT_DOUBLE_EQ(selected->requested_clearance, 0.40);
}

TEST(TubeSurfaceValidatorTest,
     ForwardExcludedSidecarUsesDirectBorderAndDeterministicTieOrder) {
  const ClearanceQuery query = [](const Eigen::Vector3d& point,
                                  const double required) {
    ClearanceQueryResult result;
    if (point.x() < 0.20) {
      result.status = DistanceStatus::KNOWN_FREE;
      result.clearance = 5.0;
      result.clearance_certified = true;
      result.clearance_is_exact = true;
      return result;
    }
    // Force the forward root to refine, then leave both direct-border
    // v-children terminal with distinct typed outcomes.
    if (std::abs(point.y()) < 1e-8 && required > 0.50) {
      result.status = DistanceStatus::KNOWN_FREE;
      result.clearance = 0.45;
      result.clearance_certified = true;
      result.clearance_is_exact = true;
      return result;
    }
    if (point.y() < 0.0) {
      result.status = DistanceStatus::OCCUPIED;
      return result;
    }
    result.status = DistanceStatus::UNKNOWN;
    return result;
  };
  TubeProfile first = MakeProfile({0.0, 0.20, 0.40});
  TubeProfile second = first;
  TubeSurfaceValidationResult first_result;
  TubeSurfaceValidationResult second_result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      first, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40, 0.10,
      first_result));
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      second, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, second_result));

  const TubeSurfaceForwardExcludedEvidence& forward =
      first_result.forward_excluded_evidence;
  ASSERT_TRUE(forward.valid);
  EXPECT_DOUBLE_EQ(forward.w0, first.certified_segment_end_w);
  EXPECT_DOUBLE_EQ(forward.w1, 0.40);
  EXPECT_DOUBLE_EQ(forward.v0, 0.0);
  EXPECT_DOUBLE_EQ(forward.v1, 0.5);
  EXPECT_EQ(forward.outcome, TubeSurfaceOutcome::CONTRACT_UNSAFE);
  EXPECT_EQ(forward.clearance_status, DistanceStatus::OCCUPIED);
  EXPECT_EQ(forward.w0, second_result.forward_excluded_evidence.w0);
  EXPECT_EQ(forward.w1, second_result.forward_excluded_evidence.w1);
  EXPECT_EQ(forward.v0, second_result.forward_excluded_evidence.v0);
  EXPECT_EQ(forward.v1, second_result.forward_excluded_evidence.v1);
  EXPECT_EQ(forward.outcome, second_result.forward_excluded_evidence.outcome);
  EXPECT_EQ(forward.inconclusive_reason,
            second_result.forward_excluded_evidence.inconclusive_reason);
  EXPECT_EQ(first_result.query_sample_count, second_result.query_sample_count);
}

TEST(TubeSurfaceValidatorTest,
     ForwardExcludedSidecarSequenceRequiresImmutableSnapshotProvenance) {
  const ClearanceQuery query = [](const Eigen::Vector3d& point,
                                  const double) {
    ClearanceQueryResult result;
    if (point.x() < 0.20) {
      result.status = DistanceStatus::KNOWN_FREE;
      result.clearance = 5.0;
      result.clearance_certified = true;
      result.clearance_is_exact = true;
      return result;
    }
    result.status = DistanceStatus::OCCUPIED;
    return result;
  };

  TubeProfile non_immutable = MakeProfile({0.0, 0.10, 0.20, 0.30}, 0.0, 0.0);
  non_immutable.snapshot_sequence = 777U;
  non_immutable.snapshot_provenance_is_immutable = false;
  TubeSurfaceValidationResult non_immutable_result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      non_immutable, 0.10, LinePath(), CertifiedLineCells(), query, 0.05,
      0.40, 0.10, non_immutable_result));
  ASSERT_TRUE(non_immutable_result.forward_excluded_evidence.valid);
  EXPECT_TRUE(non_immutable_result.forward_excluded_evidence.witness_valid);
  EXPECT_FALSE(non_immutable_result.forward_excluded_evidence
                   .map_observation_sequence_valid);
  EXPECT_EQ(non_immutable_result.forward_excluded_evidence
                .map_observation_sequence,
            0U);

  TubeProfile zero_sequence = MakeProfile({0.0, 0.10, 0.20, 0.30}, 0.0, 0.0);
  zero_sequence.snapshot_sequence = 0U;
  zero_sequence.snapshot_provenance_is_immutable = true;
  TubeSurfaceValidationResult zero_sequence_result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      zero_sequence, 0.10, LinePath(), CertifiedLineCells(), query, 0.05,
      0.40, 0.10, zero_sequence_result));
  ASSERT_TRUE(zero_sequence_result.forward_excluded_evidence.valid);
  EXPECT_TRUE(zero_sequence_result.forward_excluded_evidence.witness_valid);
  EXPECT_FALSE(zero_sequence_result.forward_excluded_evidence
                   .map_observation_sequence_valid);
  EXPECT_EQ(zero_sequence_result.forward_excluded_evidence
                .map_observation_sequence,
            0U);
}

TEST(TubeSurfaceValidatorTest, InvalidForwardExcludedSidecarUsesSentinels) {
  TubeProfile profile = MakeProfile({0.0, 0.20});
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), OpenLowerBound(), 0.05,
      0.40, 0.10, result));
  const TubeSurfaceForwardExcludedEvidence& forward =
      result.forward_excluded_evidence;
  EXPECT_FALSE(forward.valid);
  EXPECT_FALSE(forward.witness_valid);
  EXPECT_FALSE(forward.map_observation_sequence_valid);
  EXPECT_EQ(forward.map_observation_sequence, 0U);
  EXPECT_EQ(forward.depth, -1);
  EXPECT_EQ(forward.clearance_status, DistanceStatus::UNAVAILABLE);
  EXPECT_FALSE(forward.clearance_query_attempted);
  EXPECT_FALSE(forward.witness_clearance_valid);
  EXPECT_FALSE(forward.witness_clearance_certified);
  EXPECT_FALSE(forward.witness_clearance_exact);
  EXPECT_FALSE(forward.exact_d_c_valid);
  EXPECT_FALSE(forward.requested_clearance_valid);
  EXPECT_FALSE(forward.cell_certificate_attempted);
  EXPECT_FALSE(forward.cell_certificate_complete);
  EXPECT_FALSE(forward.cell_certificate_revision_match);
  EXPECT_FALSE(forward.query_budget_exhausted);
  EXPECT_FALSE(forward.max_depth_reached);
  EXPECT_FALSE(forward.geometric_evidence_valid);
  EXPECT_DOUBLE_EQ(forward.witness_clearance, 0.0);
  EXPECT_DOUBLE_EQ(forward.exact_d_c, 0.0);
  EXPECT_DOUBLE_EQ(forward.requested_clearance, 0.0);
  EXPECT_DOUBLE_EQ(forward.geometric_cover, 0.0);
  EXPECT_FALSE(profile.forward_excluded_evidence.valid);
  EXPECT_EQ(profile.forward_excluded_evidence.depth, -1);
}

TEST(TubeSurfaceValidatorTest,
     ForwardExcludedSidecarKeepsLowerBoundDistinctFromExactWitness) {
  TubeProfile profile = MakeProfile({0.0, 0.10, 0.20}, 0.0, 0.0);
  const ClearanceQuery query = [](const Eigen::Vector3d& point, const double) {
    ClearanceQueryResult result;
    if (point.x() >= 0.10) {
      result.status = DistanceStatus::KNOWN_FREE;
      result.clearance = 0.39;
      result.clearance_certified = true;
      result.clearance_is_exact = false;
      return result;
    }
    result.status = DistanceStatus::KNOWN_FREE;
    result.clearance = 5.0;
    result.clearance_certified = true;
    result.clearance_is_exact = true;
    return result;
  };
  TubeSurfaceValidationResult result;
  ASSERT_TRUE(TubeSurfaceValidator().validate(
      profile, 0.0, LinePath(), CertifiedLineCells(), query, 0.05, 0.40,
      0.10, result));
  const TubeSurfaceForwardExcludedEvidence& forward =
      result.forward_excluded_evidence;
  ASSERT_TRUE(forward.valid);
  EXPECT_EQ(forward.outcome, TubeSurfaceOutcome::INCONCLUSIVE);
  EXPECT_EQ(forward.inconclusive_reason,
            TubeSurfaceInconclusiveReason::CLEARANCE_UNCERTIFIED);
  EXPECT_TRUE(forward.clearance_query_attempted);
  EXPECT_EQ(forward.clearance_status, DistanceStatus::KNOWN_FREE);
  EXPECT_TRUE(forward.witness_clearance_valid);
  EXPECT_TRUE(forward.witness_clearance_certified);
  EXPECT_FALSE(forward.witness_clearance_exact);
  EXPECT_FALSE(forward.exact_d_c_valid);
  EXPECT_DOUBLE_EQ(forward.exact_d_c, 0.0);
  EXPECT_TRUE(forward.requested_clearance_valid);
  EXPECT_GT(forward.requested_clearance, 0.0);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
