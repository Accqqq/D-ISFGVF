#include "phase_offset_navigation/tube_surface_validator.h"

#include "phase_offset_navigation/tube_filter.h"

#include <phase_offset_core/geometry.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

namespace phase_offset_navigation {
namespace {

constexpr double kEpsilon = 1e-10;
constexpr double kAnchorTolerance = 1e-8;
constexpr double kCoverComparisonTolerance = 1e-15;

bool IsFinite(const double value) { return std::isfinite(value); }
bool IsFinite(const Eigen::Vector3d& value) { return value.allFinite(); }

double OutwardUpper(const double value) {
  if (!IsFinite(value)) return value;
  return std::nextafter(value, std::numeric_limits<double>::infinity());
}

double OutwardProduct(const double first, const double second) {
  if (!IsFinite(first) || !IsFinite(second)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return OutwardUpper(first * second);
}

double OutwardSum(const double first, const double second) {
  if (!IsFinite(first) || !IsFinite(second)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return OutwardUpper(first + second);
}

TubeSurfaceInconclusiveReason InconclusiveReason(const DistanceStatus status) {
  switch (status) {
    case DistanceStatus::UNAVAILABLE:
      return TubeSurfaceInconclusiveReason::CLEARANCE_UNAVAILABLE;
    case DistanceStatus::OUT_OF_MAP:
      return TubeSurfaceInconclusiveReason::CLEARANCE_OUT_OF_MAP;
    case DistanceStatus::UNKNOWN:
      return TubeSurfaceInconclusiveReason::CLEARANCE_UNKNOWN;
    case DistanceStatus::KNOWN_FREE:
      return TubeSurfaceInconclusiveReason::CLEARANCE_UNCERTIFIED;
    case DistanceStatus::OCCUPIED:
      break;
  }
  return TubeSurfaceInconclusiveReason::CLEARANCE_UNKNOWN;
}

TubeStopReason LegacyReason(const TubeSurfaceOutcome outcome,
                            const TubeSurfaceInconclusiveReason reason) {
  if (outcome == TubeSurfaceOutcome::CONTRACT_UNSAFE) {
    return TubeStopReason::INSUFFICIENT_CLEARANCE;
  }
  switch (reason) {
    case TubeSurfaceInconclusiveReason::CLEARANCE_UNAVAILABLE:
      return TubeStopReason::UNAVAILABLE;
    case TubeSurfaceInconclusiveReason::CLEARANCE_OUT_OF_MAP:
      return TubeStopReason::OUT_OF_MAP;
    case TubeSurfaceInconclusiveReason::CLEARANCE_UNKNOWN:
      return TubeStopReason::UNKNOWN;
    case TubeSurfaceInconclusiveReason::REGULARITY_UNPROVEN:
      return TubeStopReason::REGULARITY;
    case TubeSurfaceInconclusiveReason::INVALID_INPUT:
    case TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MISSING:
    case TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MALFORMED:
    case TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MISMATCH:
    case TubeSurfaceInconclusiveReason::CLEARANCE_UNCERTIFIED:
    case TubeSurfaceInconclusiveReason::NUMERICAL_FAILURE:
    case TubeSurfaceInconclusiveReason::DEPTH_GUARD:
    case TubeSurfaceInconclusiveReason::QUERY_BUDGET:
    case TubeSurfaceInconclusiveReason::NO_CERTIFIED_REFINEMENT_IMPROVEMENT:
      return TubeStopReason::REGULARITY;
    case TubeSurfaceInconclusiveReason::NONE:
      return TubeStopReason::UNKNOWN;
  }
  return TubeStopReason::UNKNOWN;
}

struct SurfacePoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double w = 0.0;
};

struct SurfaceCell {
  double w0 = 0.0;
  double w1 = 0.0;
  double v0 = 0.0;
  double v1 = 1.0;
  int depth = 0;
  std::size_t interval_index = 0U;
};

enum class CertificateState {
  DEGENERATE_EXACT,
  COMPLETE,
  MISSING,
  MALFORMED,
  MISMATCH,
  REGULARITY,
};

struct CellCoverBreakdown {
  CertificateState certificate_state = CertificateState::MISSING;
  bool certificate_attempted = false;
  bool certificate_complete = false;
  bool certificate_revision_match = false;
  bool decomposable = false;
  double midpoint_position_cover = 0.0;
  double normal_variation_cover = 0.0;
  double delta_slope_cover = 0.0;
  double v_span_cover = 0.0;
  double geometric_cover = 0.0;
  double support_alignment_bound = 0.0;
  double numerical_epsilon = 0.0;
  double maximum_delta = 0.0;
  double maximum_width = 0.0;
  double allowable_cover = 0.0;
  phase_offset_core::PathCellGeometryCertificate certificate;
};

struct CellEvaluation {
  TubeSurfaceOutcome outcome = TubeSurfaceOutcome::INCONCLUSIVE;
  TubeSurfaceInconclusiveReason reason = TubeSurfaceInconclusiveReason::NONE;
  TubeStopReason legacy_reason = TubeStopReason::NONE;
  CellCoverBreakdown breakdown;
  SurfacePoint center;
  double requested_radius = 0.0;
  double witness_clearance = 0.0;
  bool witness_clearance_exact = false;
  bool clearance_query_attempted = false;
  DistanceStatus clearance_status = DistanceStatus::UNAVAILABLE;
  bool witness_clearance_valid = false;
  bool witness_clearance_certified = false;
  bool requested_clearance_valid = false;
  bool cell_certificate_attempted = false;
  bool cell_certificate_complete = false;
  bool cell_certificate_revision_match = false;
  double proof_residual = 0.0;
  bool geometry_valid = false;
};

struct ValidationContext {
  TubeProfile* profile = nullptr;
  const PathStateQuery* path_state_query = nullptr;
  const PathCellBoundQuery* path_cell_bound_query = nullptr;
  const ClearanceQuery* clearance_query = nullptr;
  double snapshot_resolution = 0.0;
  double current_w = 0.0;
  double required_clearance = 0.0;
  double regularity_margin = 0.0;
  double minimum_reference_speed = 1e-8;
  double cover_epsilon = 0.0;
  const TubeSurfaceValidatorConfig* config = nullptr;
  phase_offset_core::GeometryEvaluator geometry_evaluator;
  TubeSurfaceValidationResult* result = nullptr;
};

bool IsProfileKnot(const TubeProfile& profile, const double w) {
  for (const TubeRawSample& sample : profile.samples) {
    if (IsFinite(sample.w) && std::abs(sample.w - w) <= kAnchorTolerance) {
      return true;
    }
  }
  return false;
}

std::size_t FindAnchor(const TubeProfile& profile, const double current_w) {
  std::size_t selected = profile.samples.size();
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < profile.samples.size(); ++index) {
    const double error = std::abs(profile.samples[index].w - current_w);
    if (IsFinite(error) && error <= kAnchorTolerance &&
        (error < best - kEpsilon ||
         (std::abs(error - best) <= kEpsilon && index < selected))) {
      selected = index;
      best = error;
    }
  }
  return selected;
}

bool QueryBounds(const TubeProfile& profile, const double w,
                 TubeBounds& bounds) {
  return TubeFilter::query(profile, w, bounds) && bounds.valid &&
      IsFinite(bounds.lower) && IsFinite(bounds.upper) &&
      bounds.lower <= bounds.upper + kEpsilon;
}

bool EvaluateSurfacePoint(ValidationContext& context, const double w,
                          const double v, SurfacePoint& output) {
  if (!IsFinite(w) || !IsFinite(v) || v < -kEpsilon || v > 1.0 + kEpsilon) {
    return false;
  }
  TubeBounds bounds;
  if (!QueryBounds(*context.profile, w, bounds)) return false;
  phase_offset_core::PathDifferentialState path;
  if (!(*context.path_state_query)(w, path) || !path.valid ||
      !IsFinite(path.w) || std::abs(path.w - w) > kAnchorTolerance) {
    return false;
  }
  phase_offset_core::PreparedPathGeometry prepared;
  context.geometry_evaluator.preparePath(path, prepared);
  const double clamped_v = std::max(0.0, std::min(1.0, v));
  const double delta = bounds.lower + clamped_v *
      (bounds.upper - bounds.lower);
  phase_offset_core::PreparedReferenceResult geometry;
  if (!context.geometry_evaluator.evaluatePreparedReference(
          prepared, path.p, delta, geometry) || !geometry.valid ||
      !IsFinite(geometry.r)) {
    return false;
  }
  output.point = geometry.r;
  output.w = w;
  return true;
}

bool MatchingCellCertificate(ValidationContext& context,
                             const SurfaceCell& cell,
                             CellCoverBreakdown& breakdown) {
  breakdown = CellCoverBreakdown();
  breakdown.numerical_epsilon = context.cover_epsilon;
  if (cell.w1 <= cell.w0 + kEpsilon) {
    TubeBounds bounds;
    if (!QueryBounds(*context.profile, cell.w0, bounds)) return false;
    breakdown.certificate_state = CertificateState::DEGENERATE_EXACT;
    breakdown.maximum_delta = std::max(std::abs(bounds.lower),
                                       std::abs(bounds.upper));
    breakdown.maximum_width = bounds.upper - bounds.lower;
    breakdown.v_span_cover = OutwardProduct(0.5 * breakdown.maximum_width,
                                            cell.v1 - cell.v0);
    breakdown.geometric_cover = breakdown.v_span_cover;
    breakdown.decomposable = IsFinite(breakdown.maximum_delta) &&
        IsFinite(breakdown.maximum_width) &&
        IsFinite(breakdown.v_span_cover) && breakdown.maximum_width >= 0.0 &&
        IsFinite(breakdown.geometric_cover) && breakdown.geometric_cover >= 0.0;
    return breakdown.decomposable;
  }
  if (!context.profile->cell_geometry_certified ||
      context.path_cell_bound_query == nullptr ||
      !(*context.path_cell_bound_query)) {
    breakdown.certificate_state = CertificateState::MISSING;
    return false;
  }
  phase_offset_core::PathCellGeometryCertificate certificate;
  breakdown.certificate_attempted = true;
  if (!(*context.path_cell_bound_query)(cell.w0, cell.w1, certificate)) {
    breakdown.certificate_state = CertificateState::MALFORMED;
    return false;
  }
  if (!phase_offset_core::pathCellGeometryCertificateIsComplete(certificate)) {
    breakdown.certificate_state = CertificateState::MALFORMED;
    return false;
  }
  if (std::abs(certificate.w0 - cell.w0) > kAnchorTolerance ||
      std::abs(certificate.w1 - cell.w1) > kAnchorTolerance) {
    breakdown.certificate_state = CertificateState::MALFORMED;
    return false;
  }
  breakdown.certificate_complete = true;
  if (!phase_offset_core::pathCellGeometryCertificateMatches(
          certificate, context.profile->path_revision,
          context.profile->frame_revision)) {
    breakdown.certificate_state = CertificateState::MISMATCH;
    return false;
  }
  breakdown.certificate_revision_match = true;
  TubeBounds first;
  TubeBounds second;
  if (!QueryBounds(*context.profile, cell.w0, first) ||
      !QueryBounds(*context.profile, cell.w1, second)) {
    breakdown.certificate_state = CertificateState::REGULARITY;
    return false;
  }
  const double h = cell.w1 - cell.w0;
  const double maximum_delta = std::max({std::abs(first.lower),
                                          std::abs(first.upper),
                                          std::abs(second.lower),
                                          std::abs(second.upper)});
  const double delta_slope = std::max(
      std::abs(second.lower - first.lower),
      std::abs(second.upper - first.upper)) / h;
  const double maximum_width = std::max(first.upper - first.lower,
                                        second.upper - second.lower);
  const double v_span = cell.v1 - cell.v0;
  const double active_speed_lower = certificate.inf_p_w_norm -
      certificate.sup_N_w_norm * maximum_delta;
  if (!IsFinite(active_speed_lower) ||
      active_speed_lower < context.minimum_reference_speed) {
    breakdown.certificate_state = CertificateState::REGULARITY;
    return false;
  }
  breakdown.certificate_state = CertificateState::COMPLETE;
  breakdown.certificate = certificate;
  breakdown.maximum_delta = maximum_delta;
  breakdown.maximum_width = maximum_width;
  breakdown.midpoint_position_cover = OutwardUpper(
      certificate.midpoint_position_variation_bound);
  breakdown.normal_variation_cover = OutwardProduct(
      OutwardProduct(0.5, maximum_delta), certificate.normal_variation_bound);
  breakdown.delta_slope_cover = OutwardProduct(
      OutwardProduct(0.5, delta_slope), h);
  breakdown.v_span_cover = OutwardProduct(
      OutwardProduct(0.5, maximum_width), v_span);
  double cover = OutwardSum(breakdown.midpoint_position_cover,
                            breakdown.normal_variation_cover);
  cover = OutwardSum(cover, breakdown.delta_slope_cover);
  cover = OutwardSum(cover, breakdown.v_span_cover);
  breakdown.geometric_cover = cover;
  breakdown.support_alignment_bound = 0.0;
  breakdown.decomposable = IsFinite(maximum_delta) &&
      IsFinite(maximum_width) && maximum_delta >= 0.0 &&
      maximum_width >= 0.0 && IsFinite(delta_slope) && delta_slope >= 0.0 &&
      IsFinite(cover) && cover >= 0.0;
  return breakdown.decomposable;
}

TubeSurfaceInconclusiveReason CertificateReason(const CertificateState state) {
  switch (state) {
    case CertificateState::MISSING:
      return TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MISSING;
    case CertificateState::MALFORMED:
      return TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MALFORMED;
    case CertificateState::MISMATCH:
      return TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MISMATCH;
    case CertificateState::REGULARITY:
      return TubeSurfaceInconclusiveReason::REGULARITY_UNPROVEN;
    case CertificateState::DEGENERATE_EXACT:
    case CertificateState::COMPLETE:
      break;
  }
  return TubeSurfaceInconclusiveReason::NUMERICAL_FAILURE;
}

void UpdateCoverSummary(ValidationContext& context,
                        const CellCoverBreakdown& breakdown) {
  TubeSurfaceValidationResult& result = *context.result;
  if (!IsFinite(breakdown.geometric_cover)) return;
  result.cover_accounting_observed = true;
  result.min_cover_radius = std::min(result.min_cover_radius,
                                     breakdown.geometric_cover);
  result.max_cover_radius = std::max(result.max_cover_radius,
                                     breakdown.geometric_cover);
  result.midpoint_position_cover = std::max(
      result.midpoint_position_cover, breakdown.midpoint_position_cover);
  result.normal_variation_cover = std::max(
      result.normal_variation_cover, breakdown.normal_variation_cover);
  result.delta_slope_cover = std::max(result.delta_slope_cover,
                                      breakdown.delta_slope_cover);
  result.v_span_cover = std::max(result.v_span_cover,
                                 breakdown.v_span_cover);
  result.geometric_cover = std::max(result.geometric_cover,
                                    breakdown.geometric_cover);
  result.support_alignment_bound = std::max(
      result.support_alignment_bound, breakdown.support_alignment_bound);
  result.numerical_epsilon = std::max(result.numerical_epsilon,
                                      breakdown.numerical_epsilon);
}

bool CellContainsZero(const TubeProfile& profile, const SurfaceCell& cell) {
  if (cell.w1 < cell.w0 - kEpsilon) return false;
  TubeBounds first;
  TubeBounds second;
  if (!QueryBounds(profile, cell.w0, first) ||
      !QueryBounds(profile, cell.w1, second)) return false;
  if (!(first.lower <= 0.0 && 0.0 <= first.upper &&
        second.lower <= 0.0 && 0.0 <= second.upper)) return false;
  TubeBounds middle;
  if (!QueryBounds(profile, 0.5 * (cell.w0 + cell.w1), middle)) return false;
  return middle.lower <= 0.0 && 0.0 <= middle.upper;
}

void RecordCellEvidence(ValidationContext& context, const SurfaceCell& cell,
                        const CellEvaluation& evaluation) {
  TubeSurfaceCellEvidence evidence;
  evidence.w0 = cell.w0;
  evidence.w1 = cell.w1;
  evidence.v0 = cell.v0;
  evidence.v1 = cell.v1;
  evidence.depth = cell.depth;
  evidence.path_revision = evaluation.breakdown.certificate_state ==
          CertificateState::COMPLETE
      ? evaluation.breakdown.certificate.path_revision
      : context.profile->path_revision;
  evidence.frame_revision = evaluation.breakdown.certificate_state ==
          CertificateState::COMPLETE
      ? evaluation.breakdown.certificate.frame_revision
      : context.profile->frame_revision;
  evidence.profile_revision = context.profile->profile_revision;
  evidence.outcome = evaluation.outcome;
  evidence.inconclusive_reason = evaluation.reason;
  evidence.witness_legacy_reason = evaluation.legacy_reason;
  evidence.terminal = true;
  evidence.witness_clearance = IsFinite(evaluation.witness_clearance)
      ? evaluation.witness_clearance : 0.0;
  evidence.witness_clearance_exact = evaluation.witness_clearance_exact;
  evidence.geometric_evidence_valid = evaluation.geometry_valid;
  evidence.clearance_query_attempted = evaluation.clearance_query_attempted;
  evidence.clearance_status = evaluation.clearance_status;
  evidence.witness_clearance_valid = evaluation.witness_clearance_valid;
  evidence.witness_clearance_certified = evaluation.witness_clearance_certified;
  evidence.requested_clearance_valid = evaluation.requested_clearance_valid;
  evidence.requested_clearance = evaluation.requested_clearance_valid &&
          IsFinite(evaluation.requested_radius)
      ? evaluation.requested_radius
      : 0.0;
  evidence.cell_certificate_attempted =
      evaluation.breakdown.certificate_attempted;
  evidence.cell_certificate_complete =
      evaluation.breakdown.certificate_complete;
  evidence.cell_certificate_revision_match =
      evaluation.breakdown.certificate_revision_match;
  evidence.allowable_cover = evaluation.breakdown.allowable_cover;
  evidence.midpoint_position_cover =
      evaluation.breakdown.midpoint_position_cover;
  evidence.normal_variation_cover = evaluation.breakdown.normal_variation_cover;
  evidence.delta_slope_cover = evaluation.breakdown.delta_slope_cover;
  evidence.v_span_cover = evaluation.breakdown.v_span_cover;
  evidence.geometric_cover = evaluation.breakdown.geometric_cover;
  evidence.support_alignment_bound =
      evaluation.breakdown.support_alignment_bound;
  evidence.numerical_epsilon = evaluation.breakdown.numerical_epsilon;
  evidence.proof_residual = evaluation.proof_residual;
  evidence.complete_filtered_pwl_contains_zero =
      evaluation.outcome == TubeSurfaceOutcome::SAFE &&
      CellContainsZero(*context.profile, cell);
  evidence.filtered_pwl_contains_zero =
      evidence.complete_filtered_pwl_contains_zero;
  evidence.contains_zero = evidence.complete_filtered_pwl_contains_zero;
  context.result->cell_evidence.push_back(evidence);
  context.profile->surface_cell_evidence.push_back(evidence);
}

void RecordKnotEvidence(ValidationContext& context, const SurfaceCell& cell,
                        const CellEvaluation& evaluation) {
  if (evaluation.outcome != TubeSurfaceOutcome::SAFE) return;
  if (!IsProfileKnot(*context.profile, cell.w0) &&
      !IsProfileKnot(*context.profile, cell.w1)) return;
  const double requested = evaluation.requested_radius;
  const double cover = evaluation.breakdown.geometric_cover;
  const bool contains_zero = CellContainsZero(*context.profile, cell);
  const double knots[] = {cell.w0, cell.w1};
  for (const double w : knots) {
    if (!IsProfileKnot(*context.profile, w)) continue;
    TubeValidatorKnotEvidence* target = nullptr;
    for (TubeValidatorKnotEvidence& existing :
             context.profile->validator_knot_evidence) {
      if (std::abs(existing.w - w) <= kAnchorTolerance) {
        target = &existing;
        break;
      }
    }
    if (target == nullptr) {
      TubeValidatorKnotEvidence created;
      created.w = w;
      context.profile->validator_knot_evidence.push_back(created);
      target = &context.profile->validator_knot_evidence.back();
    }
    target->observed = true;
    target->max_cover_radius = std::max(target->max_cover_radius, cover);
    target->max_requested_clearance = std::max(
        target->max_requested_clearance, requested);
    TubeBounds bounds;
    target->filtered_contains_zero = QueryBounds(
        *context.profile, w, bounds) && bounds.lower <= 0.0 &&
        0.0 <= bounds.upper;
    target->zero_surface_covered = target->zero_surface_covered ||
        contains_zero;
  }
}

CellEvaluation EvaluateCell(ValidationContext& context,
                            const SurfaceCell& cell) {
  CellEvaluation evaluation;
  ++context.result->geometry_cell_count;
  context.result->max_depth_observed = std::max(
      context.result->max_depth_observed, cell.depth);
  evaluation.breakdown.numerical_epsilon = context.cover_epsilon;
  const double center_w = 0.5 * (cell.w0 + cell.w1);
  const double center_v = 0.5 * (cell.v0 + cell.v1);
  if (!EvaluateSurfacePoint(context, center_w, center_v,
                            evaluation.center)) {
    evaluation.reason = TubeSurfaceInconclusiveReason::INVALID_INPUT;
    return evaluation;
  }
  evaluation.geometry_valid = MatchingCellCertificate(
      context, cell, evaluation.breakdown);
  evaluation.cell_certificate_attempted =
      evaluation.breakdown.certificate_attempted;
  evaluation.cell_certificate_complete =
      evaluation.breakdown.certificate_complete;
  evaluation.cell_certificate_revision_match =
      evaluation.breakdown.certificate_revision_match;
  UpdateCoverSummary(context, evaluation.breakdown);
  if (!evaluation.geometry_valid) {
    evaluation.reason = CertificateReason(
        evaluation.breakdown.certificate_state);
    return evaluation;
  }
  const double requirement = OutwardSum(
      OutwardSum(context.required_clearance,
                 evaluation.breakdown.geometric_cover),
      context.cover_epsilon);
  evaluation.requested_radius = requirement;
  if (!IsFinite(requirement) || requirement < context.required_clearance) {
    evaluation.reason = TubeSurfaceInconclusiveReason::NUMERICAL_FAILURE;
    return evaluation;
  }
  evaluation.requested_clearance_valid = IsFinite(requirement) &&
      requirement >= 0.0 && requirement >= context.required_clearance;
  if (context.result->query_sample_count >= context.config->max_query_samples) {
    context.result->limit_exceeded = true;
    context.result->query_budget_reached = true;
    evaluation.reason = TubeSurfaceInconclusiveReason::QUERY_BUDGET;
    return evaluation;
  }
  ++context.result->clearance_leaf_cell_count;
  ++context.result->query_sample_count;
  const ClearanceQueryResult query = (*context.clearance_query)(
      evaluation.center.point, requirement);
  evaluation.clearance_query_attempted = true;
  evaluation.clearance_status = query.status;
  evaluation.witness_clearance = query.clearance;
  evaluation.witness_clearance_certified = query.clearance_certified;
  evaluation.witness_clearance_valid = query.status == DistanceStatus::KNOWN_FREE &&
      query.clearance_certified && IsFinite(query.clearance);
  evaluation.witness_clearance_exact = evaluation.witness_clearance_valid &&
      query.clearance_is_exact;
  context.result->max_requested_clearance = std::max(
      context.result->max_requested_clearance, requirement);
  if (query.status == DistanceStatus::OCCUPIED) {
    evaluation.outcome = TubeSurfaceOutcome::CONTRACT_UNSAFE;
    evaluation.reason = TubeSurfaceInconclusiveReason::NONE;
    evaluation.legacy_reason = TubeStopReason::OCCUPIED;
    return evaluation;
  }
  if (query.status != DistanceStatus::KNOWN_FREE) {
    evaluation.reason = InconclusiveReason(query.status);
    return evaluation;
  }
  if (!query.clearance_certified || !IsFinite(query.clearance)) {
    evaluation.reason = !IsFinite(query.clearance)
        ? TubeSurfaceInconclusiveReason::NUMERICAL_FAILURE
        : TubeSurfaceInconclusiveReason::CLEARANCE_UNCERTIFIED;
    return evaluation;
  }
  if (query.clearance_is_exact && query.clearance < context.required_clearance) {
    evaluation.outcome = TubeSurfaceOutcome::CONTRACT_UNSAFE;
    evaluation.reason = TubeSurfaceInconclusiveReason::NONE;
    evaluation.legacy_reason = TubeStopReason::INSUFFICIENT_CLEARANCE;
    return evaluation;
  }
  if (query.clearance >= requirement) {
    evaluation.outcome = TubeSurfaceOutcome::SAFE;
    context.result->witness_evidence_present = true;
    context.result->witness_clearance = query.clearance;
    context.result->witness_clearance_exact = query.clearance_is_exact;
    context.result->min_clearance_margin = std::min(
        context.result->min_clearance_margin,
        query.clearance - context.required_clearance);
    return evaluation;
  }
  if (query.clearance_is_exact && query.clearance >= context.required_clearance) {
    evaluation.outcome = TubeSurfaceOutcome::REFINE;
    evaluation.breakdown.allowable_cover = query.clearance -
        context.required_clearance - context.cover_epsilon;
    evaluation.proof_residual = evaluation.breakdown.geometric_cover -
        evaluation.breakdown.allowable_cover;
    context.result->proof_residual = std::max(
        context.result->proof_residual, evaluation.proof_residual);
    return evaluation;
  }
  evaluation.reason = TubeSurfaceInconclusiveReason::CLEARANCE_UNCERTIFIED;
  return evaluation;
}

bool CoverOnly(ValidationContext& context, const SurfaceCell& cell,
               CellCoverBreakdown& breakdown) {
  if (!MatchingCellCertificate(context, cell, breakdown)) return false;
  UpdateCoverSummary(context, breakdown);
  return true;
}

struct SplitChoice {
  bool split_w = false;
  bool split_v = false;
  double reduction = -std::numeric_limits<double>::infinity();
  std::size_t child_count = 0U;
};

bool BetterSplit(const SplitChoice& candidate, const SplitChoice& current) {
  if (candidate.reduction > current.reduction + kCoverComparisonTolerance) {
    return true;
  }
  if (std::abs(candidate.reduction - current.reduction) >
      kCoverComparisonTolerance) {
    return false;
  }
  if (candidate.child_count != current.child_count) {
    return candidate.child_count < current.child_count;
  }
  if (candidate.split_w != current.split_w) return candidate.split_w;
  if (candidate.split_v != current.split_v) return !candidate.split_v;
  return false;
}

SplitChoice EvaluateSplitChoice(ValidationContext& context,
                                const SurfaceCell& cell, const bool split_w,
                                const bool split_v) {
  SplitChoice choice;
  choice.split_w = split_w;
  choice.split_v = split_v;
  const double w_mid = 0.5 * (cell.w0 + cell.w1);
  const double v_mid = 0.5 * (cell.v0 + cell.v1);
  std::vector<SurfaceCell> children;
  if (split_w && split_v) {
    children = {{cell.w0, w_mid, cell.v0, v_mid, cell.depth + 1,
                 cell.interval_index},
                {cell.w0, w_mid, v_mid, cell.v1, cell.depth + 1,
                 cell.interval_index},
                {w_mid, cell.w1, cell.v0, v_mid, cell.depth + 1,
                 cell.interval_index},
                {w_mid, cell.w1, v_mid, cell.v1, cell.depth + 1,
                 cell.interval_index}};
  } else if (split_w) {
    children = {{cell.w0, w_mid, cell.v0, cell.v1, cell.depth + 1,
                 cell.interval_index},
                {w_mid, cell.w1, cell.v0, cell.v1, cell.depth + 1,
                 cell.interval_index}};
  } else if (split_v) {
    children = {{cell.w0, cell.w1, cell.v0, v_mid, cell.depth + 1,
                 cell.interval_index},
                {cell.w0, cell.w1, v_mid, cell.v1, cell.depth + 1,
                 cell.interval_index}};
  }
  if (children.empty()) return choice;
  CellCoverBreakdown parent;
  if (!CoverOnly(context, cell, parent)) return choice;
  double maximum_child_cover = 0.0;
  for (const SurfaceCell& child : children) {
    CellCoverBreakdown child_breakdown;
    if (!CoverOnly(context, child, child_breakdown)) return choice;
    maximum_child_cover = std::max(maximum_child_cover,
                                   child_breakdown.geometric_cover);
  }
  choice.reduction = parent.geometric_cover - maximum_child_cover;
  choice.child_count = children.size();
  return choice;
}

void MakeChildren(const SurfaceCell& cell, const bool split_w,
                  const bool split_v, std::vector<SurfaceCell>& children) {
  children.clear();
  const double w_mid = 0.5 * (cell.w0 + cell.w1);
  const double v_mid = 0.5 * (cell.v0 + cell.v1);
  if (split_w && split_v) {
    children = {{cell.w0, w_mid, cell.v0, v_mid, cell.depth + 1,
                 cell.interval_index},
                {cell.w0, w_mid, v_mid, cell.v1, cell.depth + 1,
                 cell.interval_index},
                {w_mid, cell.w1, cell.v0, v_mid, cell.depth + 1,
                 cell.interval_index},
                {w_mid, cell.w1, v_mid, cell.v1, cell.depth + 1,
                 cell.interval_index}};
  } else if (split_w) {
    children = {{cell.w0, w_mid, cell.v0, cell.v1, cell.depth + 1,
                 cell.interval_index},
                {w_mid, cell.w1, cell.v0, cell.v1, cell.depth + 1,
                 cell.interval_index}};
  } else if (split_v) {
    children = {{cell.w0, cell.w1, cell.v0, v_mid, cell.depth + 1,
                 cell.interval_index},
                {cell.w0, cell.w1, v_mid, cell.v1, cell.depth + 1,
                 cell.interval_index}};
  }
}

struct WorkItem {
  SurfaceCell cell;
  double unresolved = 0.0;
  bool has_evaluation = false;
  CellEvaluation evaluation;
};

struct WorkItemLess {
  bool operator()(const WorkItem& first, const WorkItem& second) const {
    if (std::abs(first.unresolved - second.unresolved) >
        kCoverComparisonTolerance) {
      return first.unresolved < second.unresolved;
    }
    const auto first_key = std::make_tuple(first.cell.w0, first.cell.w1,
                                           first.cell.v0, first.cell.v1,
                                           first.cell.depth,
                                           first.cell.interval_index);
    const auto second_key = std::make_tuple(second.cell.w0, second.cell.w1,
                                            second.cell.v0, second.cell.v1,
                                            second.cell.depth,
                                            second.cell.interval_index);
    return first_key > second_key;
  }
};

void MarkPendingBudget(std::priority_queue<WorkItem, std::vector<WorkItem>,
                                           WorkItemLess>& queue,
                       ValidationContext& context) {
  while (!queue.empty()) {
    const SurfaceCell cell = queue.top().cell;
    const bool has_evaluation = queue.top().has_evaluation;
    const CellEvaluation cached = queue.top().evaluation;
    queue.pop();
    const bool convert_to_query_budget =
        !has_evaluation || cached.outcome == TubeSurfaceOutcome::REFINE;
    if (!convert_to_query_budget) {
      RecordCellEvidence(context, cell, cached);
      continue;
    }
    CellEvaluation pending = cached;
    pending.outcome = TubeSurfaceOutcome::INCONCLUSIVE;
    pending.reason = TubeSurfaceInconclusiveReason::QUERY_BUDGET;
    pending.breakdown.numerical_epsilon = context.cover_epsilon;
    RecordCellEvidence(context, cell, pending);
  }
  context.result->limit_exceeded = true;
  context.result->query_budget_reached = true;
}

void MarkPendingDepth(ValidationContext& context, const SurfaceCell& cell,
                      const CellEvaluation& cached) {
  // Preserve all facts from the already-evaluated REFINE leaf.  Depth guard
  // changes only the public terminal classification/reason for diagnostics;
  // it never re-evaluates the cell or fabricates missing provenance.
  CellEvaluation pending = cached;
  pending.outcome = TubeSurfaceOutcome::INCONCLUSIVE;
  pending.reason = TubeSurfaceInconclusiveReason::DEPTH_GUARD;
  RecordCellEvidence(context, cell, pending);
  context.result->limit_exceeded = true;
  context.result->depth_guard_reached = true;
}

bool CanonicalEvidenceLess(const TubeSurfaceCellEvidence& first,
                           const TubeSurfaceCellEvidence& second,
                           const double current_w) {
  const double first_distance = std::abs(0.5 * (first.w0 + first.w1) -
                                         current_w);
  const double second_distance = std::abs(0.5 * (second.w0 + second.w1) -
                                          current_w);
  if (std::abs(first_distance - second_distance) > kCoverComparisonTolerance) {
    return first_distance < second_distance;
  }
  if (std::abs(first.w0 - second.w0) > kCoverComparisonTolerance) {
    return first.w0 < second.w0;
  }
  if (std::abs(first.v0 - second.v0) > kCoverComparisonTolerance) {
    return first.v0 < second.v0;
  }
  const int first_precedence = first.outcome == TubeSurfaceOutcome::CONTRACT_UNSAFE
      ? 0 : first.outcome == TubeSurfaceOutcome::INCONCLUSIVE ? 1 : 2;
  const int second_precedence = second.outcome == TubeSurfaceOutcome::CONTRACT_UNSAFE
      ? 0 : second.outcome == TubeSurfaceOutcome::INCONCLUSIVE ? 1 : 2;
  if (first_precedence != second_precedence) {
    return first_precedence < second_precedence;
  }
  return static_cast<int>(first.inconclusive_reason) <
      static_cast<int>(second.inconclusive_reason);
}

TubeSurfaceOutcome AggregateInterval(
    const std::vector<TubeSurfaceCellEvidence>& evidence) {
  bool any_inconclusive = false;
  for (const TubeSurfaceCellEvidence& cell : evidence) {
    if (cell.outcome == TubeSurfaceOutcome::CONTRACT_UNSAFE) {
      return TubeSurfaceOutcome::CONTRACT_UNSAFE;
    }
    if (cell.outcome == TubeSurfaceOutcome::INCONCLUSIVE) any_inconclusive = true;
  }
  return any_inconclusive ? TubeSurfaceOutcome::INCONCLUSIVE
                          : TubeSurfaceOutcome::SAFE;
}

bool ContinuousCentrelineCoverage(const TubeProfile& profile,
                                  const std::vector<TubeSurfaceCellEvidence>&
                                      evidence,
                                  const double current_w,
                                  double& start_w, double& end_w) {
  std::vector<std::pair<double, double>> intervals;
  for (const TubeSurfaceCellEvidence& cell : evidence) {
    if (cell.outcome != TubeSurfaceOutcome::SAFE ||
        !cell.complete_filtered_pwl_contains_zero || !cell.terminal ||
        cell.profile_revision != profile.profile_revision ||
        cell.path_revision != profile.path_revision ||
        cell.frame_revision != profile.frame_revision ||
        !IsFinite(cell.w0) || !IsFinite(cell.w1) ||
        cell.w1 < cell.w0 - kEpsilon) {
      continue;
    }
    intervals.emplace_back(cell.w0, cell.w1);
  }
  if (intervals.empty()) return false;
  std::sort(intervals.begin(), intervals.end());
  std::vector<std::pair<double, double>> merged;
  for (const auto interval : intervals) {
    if (merged.empty() || interval.first > merged.back().second + kAnchorTolerance) {
      merged.push_back(interval);
    } else {
      merged.back().second = std::max(merged.back().second, interval.second);
    }
  }
  const double required_start = profile.samples.front().w;
  const double required_end = profile.samples.back().w;
  bool covers_start = false;
  bool covers_end = false;
  bool covers_anchor = false;
  for (const auto interval : merged) {
    covers_start = covers_start ||
        interval.first <= required_start + kAnchorTolerance;
    covers_end = covers_end || interval.second >= required_end - kAnchorTolerance;
    covers_anchor = covers_anchor ||
        (interval.first - kAnchorTolerance <= current_w &&
         current_w <= interval.second + kAnchorTolerance);
  }
  if (!covers_start || !covers_end || !covers_anchor) return false;
  start_w = required_start;
  end_w = required_end;
  return true;
}

void DowngradeStaleProofLevel(TubeProfile& profile) {
  if (profile.proof_level == TubeProofLevel::CONTINUOUS_COVER_PROOF) {
    profile.proof_level = profile.cell_geometry_certified
        ? TubeProofLevel::FRAME_CELL_PROOF
        : TubeProofLevel::SAMPLED_EVIDENCE;
  }
}

void DeriveLegacyFailure(const TubeSurfaceValidationResult& result,
                         const double current_w, TubeStopReason& reason,
                         double& w) {
  reason = TubeStopReason::NONE;
  w = 0.0;
  const TubeSurfaceCellEvidence* selected = nullptr;
  for (const TubeSurfaceCellEvidence& candidate : result.cell_evidence) {
    if (candidate.outcome == TubeSurfaceOutcome::SAFE) continue;
    if (selected == nullptr ||
        CanonicalEvidenceLess(candidate, *selected, current_w)) {
      selected = &candidate;
    }
  }
  if (selected == nullptr) return;
  reason = selected->witness_legacy_reason != TubeStopReason::NONE
      ? selected->witness_legacy_reason
      : LegacyReason(selected->outcome, selected->inconclusive_reason);
  w = IsFinite(selected->w0) ? selected->w0 : 0.0;
}

const TubeSurfaceCellEvidence* CanonicalExcludedEvidence(
    const std::vector<std::vector<TubeSurfaceCellEvidence>>& groups,
    const std::size_t first, const std::size_t last, const double current_w) {
  const TubeSurfaceCellEvidence* selected = nullptr;
  for (std::size_t group = 0U; group < groups.size(); ++group) {
    // Retained original interval groups are the half-open range [first,last).
    // The group at `last` is the first excluded interval and must remain
    // eligible as canonical terminal evidence (especially for a suffix).
    if (group >= first && group < last) continue;
    for (const TubeSurfaceCellEvidence& candidate : groups[group]) {
      if (candidate.outcome == TubeSurfaceOutcome::SAFE) continue;
      if (selected == nullptr ||
          CanonicalEvidenceLess(candidate, *selected, current_w)) {
        selected = &candidate;
      }
    }
  }
  return selected;
}

int ForwardOutcomePrecedence(const TubeSurfaceOutcome outcome) {
  switch (outcome) {
    case TubeSurfaceOutcome::CONTRACT_UNSAFE:
      return 0;
    case TubeSurfaceOutcome::INCONCLUSIVE:
      return 1;
    case TubeSurfaceOutcome::REFINE:
      return 2;
    case TubeSurfaceOutcome::SAFE:
      return 3;
  }
  return 3;
}

bool ForwardTieLess(const TubeSurfaceCellEvidence& first,
                    const TubeSurfaceCellEvidence& second) {
  const auto first_key = std::make_tuple(
      first.w0, first.w1, first.v0, first.v1, first.depth,
      ForwardOutcomePrecedence(first.outcome),
      static_cast<int>(first.inconclusive_reason));
  const auto second_key = std::make_tuple(
      second.w0, second.w1, second.v0, second.v1, second.depth,
      ForwardOutcomePrecedence(second.outcome),
      static_cast<int>(second.inconclusive_reason));
  return first_key < second_key;
}

const TubeSurfaceCellEvidence* ForwardExcludedEvidence(
    const std::vector<std::vector<TubeSurfaceCellEvidence>>& groups,
    const std::size_t last, const double current_w,
    const double retained_end_w) {
  if (last >= groups.size() || !IsFinite(current_w) ||
      !IsFinite(retained_end_w)) {
    return nullptr;
  }
  const TubeSurfaceCellEvidence* selected = nullptr;
  bool selected_direct = false;
  double selected_distance = std::numeric_limits<double>::infinity();
  for (const TubeSurfaceCellEvidence& candidate : groups[last]) {
    if (!candidate.terminal || candidate.outcome == TubeSurfaceOutcome::SAFE ||
        !IsFinite(candidate.w0) || !IsFinite(candidate.w1) ||
        !IsFinite(candidate.v0) || !IsFinite(candidate.v1) ||
        candidate.w0 < current_w - kAnchorTolerance ||
        candidate.w1 < candidate.w0 - kEpsilon) {
      continue;
    }
    const double distance = candidate.w0 - retained_end_w;
    const bool direct = std::abs(distance) <= kAnchorTolerance;
    if (!direct && distance < 0.0) continue;
    const double nonnegative_distance = direct ? 0.0 : distance;
    if (selected == nullptr) {
      selected = &candidate;
      selected_direct = direct;
      selected_distance = nonnegative_distance;
      continue;
    }
    if (direct != selected_direct) {
      if (direct) {
        selected = &candidate;
        selected_direct = true;
        selected_distance = nonnegative_distance;
      }
      continue;
    }
    if (!direct && nonnegative_distance < selected_distance -
            kCoverComparisonTolerance) {
      selected = &candidate;
      selected_distance = nonnegative_distance;
      continue;
    }
    if ((!direct && std::abs(nonnegative_distance - selected_distance) <=
             kCoverComparisonTolerance) || direct) {
      if (ForwardTieLess(candidate, *selected)) {
        selected = &candidate;
        selected_distance = nonnegative_distance;
      }
    }
  }
  return selected;
}

void CopyForwardExcludedEvidence(
    TubeSurfaceValidationResult& result, TubeProfile& profile,
    const std::vector<std::vector<TubeSurfaceCellEvidence>>& groups,
    const std::size_t first, const std::size_t last,
    const double current_w) {
  result.forward_excluded_evidence = TubeSurfaceForwardExcludedEvidence();
  profile.forward_excluded_evidence = TubeSurfaceForwardExcludedEvidence();
  if (first >= last || !result.truncated_after ||
      !IsFinite(profile.certified_segment_end_w)) {
    return;
  }
  const TubeSurfaceCellEvidence* selected = ForwardExcludedEvidence(
      groups, last, current_w, profile.certified_segment_end_w);
  if (selected == nullptr) return;

  TubeSurfaceForwardExcludedEvidence evidence;
  evidence.valid = true;
  evidence.w0 = selected->w0;
  evidence.w1 = selected->w1;
  evidence.v0 = selected->v0;
  evidence.v1 = selected->v1;
  evidence.depth = selected->depth;
  evidence.outcome = selected->outcome;
  evidence.inconclusive_reason = selected->inconclusive_reason;
  evidence.clearance_query_attempted = selected->clearance_query_attempted;
  evidence.clearance_status = selected->clearance_query_attempted
      ? selected->clearance_status : DistanceStatus::UNAVAILABLE;
  evidence.witness_clearance_valid = selected->witness_clearance_valid;
  evidence.witness_clearance_certified = selected->clearance_query_attempted &&
      selected->witness_clearance_certified;
  evidence.witness_clearance_exact = selected->witness_clearance_valid &&
      selected->witness_clearance_exact;
  evidence.witness_clearance = evidence.witness_clearance_valid &&
          IsFinite(selected->witness_clearance)
      ? selected->witness_clearance : 0.0;
  evidence.exact_d_c_valid = evidence.witness_clearance_valid &&
      evidence.witness_clearance_exact && IsFinite(evidence.witness_clearance);
  evidence.exact_d_c = evidence.exact_d_c_valid
      ? evidence.witness_clearance : 0.0;
  evidence.requested_clearance_valid = selected->requested_clearance_valid;
  evidence.requested_clearance = selected->requested_clearance_valid &&
          IsFinite(selected->requested_clearance)
      ? selected->requested_clearance : 0.0;
  evidence.cell_certificate_attempted = selected->cell_certificate_attempted;
  evidence.cell_certificate_complete = selected->cell_certificate_complete;
  evidence.cell_certificate_revision_match =
      selected->cell_certificate_revision_match;
  evidence.query_budget_exhausted = selected->inconclusive_reason ==
      TubeSurfaceInconclusiveReason::QUERY_BUDGET;
  evidence.max_depth_reached = selected->inconclusive_reason ==
      TubeSurfaceInconclusiveReason::DEPTH_GUARD;
  evidence.geometric_evidence_valid = selected->geometric_evidence_valid;
  evidence.midpoint_position_cover = selected->midpoint_position_cover;
  evidence.normal_variation_cover = selected->normal_variation_cover;
  evidence.delta_slope_cover = selected->delta_slope_cover;
  evidence.v_span_cover = selected->v_span_cover;
  evidence.geometric_cover = selected->geometric_cover;
  evidence.support_alignment_bound = selected->support_alignment_bound;
  evidence.numerical_epsilon = selected->numerical_epsilon;
  evidence.allowable_cover = selected->allowable_cover;
  evidence.proof_residual = selected->proof_residual;
  result.forward_excluded_evidence = evidence;
  profile.forward_excluded_evidence = evidence;
}

void CopyResultToDiagnostics(TubeProfile& profile,
                             const TubeSurfaceValidationResult& result) {
  TubeBuildDiagnostics& diagnostics = profile.diagnostics;
  diagnostics.surface_summary_present = true;
  diagnostics.surface_outcome = result.outcome;
  diagnostics.surface_inconclusive_reason = result.inconclusive_reason;
  diagnostics.surface_truncation_outcome = result.truncation_outcome;
  diagnostics.surface_terminal_w = result.terminal_w;
  diagnostics.surface_witness_clearance = result.witness_clearance;
  diagnostics.surface_witness_clearance_exact = result.witness_clearance_exact;
  diagnostics.surface_midpoint_position_cover = result.midpoint_position_cover;
  diagnostics.surface_normal_variation_cover = result.normal_variation_cover;
  diagnostics.surface_delta_slope_cover = result.delta_slope_cover;
  diagnostics.surface_v_span_cover = result.v_span_cover;
  diagnostics.surface_geometric_cover = result.geometric_cover;
  diagnostics.surface_support_alignment_bound = result.support_alignment_bound;
  diagnostics.surface_numerical_epsilon = result.numerical_epsilon;
  diagnostics.surface_proof_residual = result.proof_residual;
  diagnostics.surface_max_depth_observed = result.max_depth_observed;
  diagnostics.surface_query_sample_count = result.query_sample_count;
  diagnostics.surface_depth_guard_reached = result.depth_guard_reached;
  diagnostics.surface_query_budget_reached = result.query_budget_reached;
  diagnostics.surface_split_w_count = result.split_w_count;
  diagnostics.surface_split_v_count = result.split_v_count;
  diagnostics.surface_split_both_count = result.split_both_count;
  diagnostics.zero_centerline_contiguous =
      result.zero_centerline_continuously_certified;
  diagnostics.zero_centerline_start_w = result.certified_start_w;
  diagnostics.zero_centerline_end_w = result.certified_end_w;
}

}  // namespace

const char* tubeSurfaceOutcomeName(const TubeSurfaceOutcome outcome) {
  switch (outcome) {
    case TubeSurfaceOutcome::SAFE: return "safe";
    case TubeSurfaceOutcome::CONTRACT_UNSAFE: return "contract_unsafe";
    case TubeSurfaceOutcome::REFINE: return "refine";
    case TubeSurfaceOutcome::INCONCLUSIVE: return "inconclusive";
  }
  return "inconclusive";
}

const char* tubeSurfaceInconclusiveReasonName(
    const TubeSurfaceInconclusiveReason reason) {
  switch (reason) {
    case TubeSurfaceInconclusiveReason::NONE: return "none";
    case TubeSurfaceInconclusiveReason::INVALID_INPUT: return "invalid_input";
    case TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MISSING:
      return "cell_certificate_missing";
    case TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MALFORMED:
      return "cell_certificate_malformed";
    case TubeSurfaceInconclusiveReason::CELL_CERTIFICATE_MISMATCH:
      return "cell_certificate_mismatch";
    case TubeSurfaceInconclusiveReason::REGULARITY_UNPROVEN:
      return "regularity_unproven";
    case TubeSurfaceInconclusiveReason::CLEARANCE_UNAVAILABLE:
      return "clearance_unavailable";
    case TubeSurfaceInconclusiveReason::CLEARANCE_OUT_OF_MAP:
      return "clearance_out_of_map";
    case TubeSurfaceInconclusiveReason::CLEARANCE_UNKNOWN:
      return "clearance_unknown";
    case TubeSurfaceInconclusiveReason::CLEARANCE_UNCERTIFIED:
      return "clearance_uncertified";
    case TubeSurfaceInconclusiveReason::NUMERICAL_FAILURE:
      return "numerical_failure";
    case TubeSurfaceInconclusiveReason::DEPTH_GUARD:
      return "depth_guard";
    case TubeSurfaceInconclusiveReason::QUERY_BUDGET:
      return "query_budget";
    case TubeSurfaceInconclusiveReason::NO_CERTIFIED_REFINEMENT_IMPROVEMENT:
      return "no_certified_refinement_improvement";
  }
  return "none";
}

const char* tubeSurfaceTruncationOutcomeName(
    const TubeSurfaceTruncationOutcome outcome) {
  switch (outcome) {
    case TubeSurfaceTruncationOutcome::NONE: return "none";
    case TubeSurfaceTruncationOutcome::PREFIX: return "prefix";
    case TubeSurfaceTruncationOutcome::SUFFIX: return "suffix";
    case TubeSurfaceTruncationOutcome::PREFIX_AND_SUFFIX:
      return "prefix_and_suffix";
    case TubeSurfaceTruncationOutcome::ANCHOR_EXCLUDED:
      return "anchor_excluded";
    case TubeSurfaceTruncationOutcome::NO_NONDEGENERATE_COMPONENT:
      return "no_nondegenerate_component";
  }
  return "none";
}

TubeSurfaceValidator::TubeSurfaceValidator(
    const TubeSurfaceValidatorConfig& config)
    : config_(config) {}

bool TubeSurfaceValidator::configurationValid() const {
  return config_.max_subdivision_depth >= 0 &&
      config_.max_subdivision_depth <= 24 &&
      config_.max_query_samples >= 1U &&
      IsFinite(config_.minimum_reference_speed) &&
      config_.minimum_reference_speed > 0.0;
}

bool TubeSurfaceValidator::validate(
    TubeProfile& profile, const double current_w,
    const PathStateQuery& path_state_query,
    const ClearanceQuery& clearance_query, const double snapshot_resolution,
    const double required_clearance, const double regularity_margin,
    TubeSurfaceValidationResult& result) const {
  return validate(profile, current_w, path_state_query, PathCellBoundQuery(),
                  clearance_query, snapshot_resolution, required_clearance,
                  regularity_margin, result);
}

bool TubeSurfaceValidator::validate(
    TubeProfile& profile, const double current_w,
    const PathStateQuery& path_state_query,
    const PathCellBoundQuery& path_cell_bound_query,
    const ClearanceQuery& clearance_query, const double snapshot_resolution,
    const double required_clearance, const double regularity_margin,
    TubeSurfaceValidationResult& result) const {
  result = TubeSurfaceValidationResult();
  result.outcome = TubeSurfaceOutcome::INCONCLUSIVE;
  result.min_clearance_margin = std::numeric_limits<double>::infinity();
  result.min_cover_radius = std::numeric_limits<double>::infinity();
  DowngradeStaleProofLevel(profile);
  profile.validator_knot_evidence.clear();
  profile.surface_cell_evidence.clear();
  profile.forward_excluded_evidence = TubeSurfaceForwardExcludedEvidence();
  profile.zero_centerline_continuously_certified = false;
  profile.obstacle_certified = false;
  if (!configurationValid() || !profile.complete || profile.samples.size() < 2U ||
      !path_state_query || !clearance_query || !IsFinite(current_w) ||
      !IsFinite(snapshot_resolution) || snapshot_resolution <= 0.0 ||
      !IsFinite(required_clearance) || required_clearance < 0.0 ||
      !IsFinite(regularity_margin) || regularity_margin <= 0.0) {
    result.inconclusive_reason = TubeSurfaceInconclusiveReason::INVALID_INPUT;
    result.first_failure_reason = TubeStopReason::INVALID_PATH;
    result.first_failure_w = IsFinite(current_w) ? current_w : 0.0;
    result.min_cover_radius = 0.0;
    DowngradeStaleProofLevel(profile);
    CopyResultToDiagnostics(profile, result);
    return false;
  }

  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = regularity_margin;
  geometry_params.minimum_reference_speed = config_.minimum_reference_speed;
  ValidationContext context;
  context.profile = &profile;
  context.path_state_query = &path_state_query;
  context.path_cell_bound_query = &path_cell_bound_query;
  context.clearance_query = &clearance_query;
  context.snapshot_resolution = snapshot_resolution;
  context.current_w = current_w;
  context.required_clearance = required_clearance;
  context.regularity_margin = regularity_margin;
  context.minimum_reference_speed = config_.minimum_reference_speed;
  context.cover_epsilon = std::min(1e-6, 0.01 * snapshot_resolution);
  context.config = &config_;
  context.geometry_evaluator = phase_offset_core::GeometryEvaluator(
      geometry_params);
  context.result = &result;
  result.support_alignment_bound = 0.0;
  result.numerical_epsilon = context.cover_epsilon;

  const std::size_t anchor = FindAnchor(profile, current_w);
  if (anchor >= profile.samples.size()) {
    result.inconclusive_reason = TubeSurfaceInconclusiveReason::INVALID_INPUT;
    result.first_failure_reason = TubeStopReason::INVALID_PATH;
    result.first_failure_w = current_w;
    result.min_cover_radius = 0.0;
    DowngradeStaleProofLevel(profile);
    CopyResultToDiagnostics(profile, result);
    return false;
  }

  const std::size_t anchor_group = profile.samples.size();
  const std::size_t group_count = profile.samples.size() + 1U;
  std::priority_queue<WorkItem, std::vector<WorkItem>, WorkItemLess> queue;
  queue.push({{current_w, current_w, 0.0, 1.0, 0, anchor_group}, 0.0});
  for (std::size_t index = 0U; index + 1U < profile.samples.size(); ++index) {
    queue.push({{profile.samples[index].w, profile.samples[index + 1U].w,
                 0.0, 1.0, 0, index}, 0.0});
  }

  while (!queue.empty()) {
    if (result.query_sample_count >= config_.max_query_samples) {
      MarkPendingBudget(queue, context);
      break;
    }
    const WorkItem item = queue.top();
    queue.pop();
    const SurfaceCell cell = item.cell;
    const CellEvaluation evaluation = item.has_evaluation
        ? item.evaluation : EvaluateCell(context, cell);
    result.max_depth_observed = std::max(result.max_depth_observed, cell.depth);
    if (evaluation.outcome != TubeSurfaceOutcome::REFINE) {
      RecordCellEvidence(context, cell, evaluation);
      RecordKnotEvidence(context, cell, evaluation);
      continue;
    }
    if (cell.depth >= config_.max_subdivision_depth) {
      MarkPendingDepth(context, cell, evaluation);
      continue;
    }
    const bool can_split_w = cell.w1 - cell.w0 > kEpsilon;
    const bool can_split_v = cell.v1 - cell.v0 > kEpsilon;
    SplitChoice choice;
    if (can_split_w) choice = EvaluateSplitChoice(context, cell, true, false);
    if (can_split_v) {
      const SplitChoice candidate = EvaluateSplitChoice(context, cell, false, true);
      if (BetterSplit(candidate, choice)) choice = candidate;
    }
    if (can_split_w && can_split_v) {
      const SplitChoice candidate = EvaluateSplitChoice(context, cell, true, true);
      if (BetterSplit(candidate, choice)) choice = candidate;
    }
    if (choice.child_count == 0U ||
        !(choice.reduction > kCoverComparisonTolerance)) {
      CellEvaluation terminal = evaluation;
      terminal.outcome = TubeSurfaceOutcome::INCONCLUSIVE;
      terminal.reason = TubeSurfaceInconclusiveReason::
          NO_CERTIFIED_REFINEMENT_IMPROVEMENT;
      RecordCellEvidence(context, cell, terminal);
      continue;
    }
    if (choice.split_w && choice.split_v) {
      ++result.split_both_count;
    } else if (choice.split_w) {
      ++result.split_w_count;
    } else {
      ++result.split_v_count;
    }
    if (choice.split_w != choice.split_v) ++result.anisotropic_split_count;
    std::vector<SurfaceCell> children;
    MakeChildren(cell, choice.split_w, choice.split_v, children);
    for (const SurfaceCell& child : children) {
      const CellEvaluation child_evaluation = EvaluateCell(context, child);
      const double unresolved = child_evaluation.outcome ==
          TubeSurfaceOutcome::REFINE
          ? child_evaluation.proof_residual : 0.0;
      queue.push({child, unresolved, true, child_evaluation});
    }
  }

  std::sort(result.cell_evidence.begin(), result.cell_evidence.end(),
            [current_w](const TubeSurfaceCellEvidence& first,
                        const TubeSurfaceCellEvidence& second) {
              return CanonicalEvidenceLess(first, second, current_w);
            });
  profile.surface_cell_evidence = result.cell_evidence;
  std::vector<std::vector<TubeSurfaceCellEvidence>> groups(group_count);
  for (const TubeSurfaceCellEvidence& evidence : result.cell_evidence) {
    std::size_t group = anchor_group;
    if (!(std::abs(evidence.w0 - current_w) <= kAnchorTolerance &&
          std::abs(evidence.w1 - current_w) <= kAnchorTolerance)) {
      double best_distance = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0U; index + 1U < profile.samples.size(); ++index) {
        const double distance = std::abs(evidence.w0 - profile.samples[index].w) +
            std::abs(evidence.w1 - profile.samples[index + 1U].w);
        if (distance < best_distance) {
          best_distance = distance;
          group = index;
        }
      }
    }
    if (group < groups.size()) groups[group].push_back(evidence);
  }

  const TubeSurfaceOutcome anchor_outcome = AggregateInterval(groups[anchor_group]);
  result.current_anchor_valid = anchor_outcome == TubeSurfaceOutcome::SAFE;
  if (!result.current_anchor_valid) {
    result.outcome = anchor_outcome;
    result.truncation_outcome = TubeSurfaceTruncationOutcome::ANCHOR_EXCLUDED;
    if (anchor_outcome == TubeSurfaceOutcome::INCONCLUSIVE) {
      for (const TubeSurfaceCellEvidence& evidence : groups[anchor_group]) {
        if (evidence.outcome == TubeSurfaceOutcome::INCONCLUSIVE) {
          result.inconclusive_reason = evidence.inconclusive_reason;
          break;
        }
      }
    }
    DeriveLegacyFailure(result, current_w, result.first_failure_reason,
                        result.first_failure_w);
    if (!IsFinite(result.min_cover_radius)) result.min_cover_radius = 0.0;
    DowngradeStaleProofLevel(profile);
    CopyResultToDiagnostics(profile, result);
    return false;
  }

  std::vector<bool> safe_interval(profile.samples.size() - 1U, false);
  for (std::size_t index = 0U; index + 1U < profile.samples.size(); ++index) {
    safe_interval[index] = !groups[index].empty() &&
        AggregateInterval(groups[index]) == TubeSurfaceOutcome::SAFE;
  }
  std::size_t first = anchor;
  std::size_t last = anchor;
  while (first > 0U && safe_interval[first - 1U]) --first;
  while (last < safe_interval.size() && safe_interval[last]) ++last;
  if (first == last) {
    result.outcome = TubeSurfaceOutcome::INCONCLUSIVE;
    result.truncation_outcome =
        TubeSurfaceTruncationOutcome::NO_NONDEGENERATE_COMPONENT;
    const TubeSurfaceCellEvidence* excluded = CanonicalExcludedEvidence(
        groups, first, last, current_w);
    if (excluded != nullptr) {
      result.terminal_w = excluded->w0;
      result.inconclusive_reason = excluded->outcome ==
              TubeSurfaceOutcome::INCONCLUSIVE
          ? excluded->inconclusive_reason
          : TubeSurfaceInconclusiveReason::NO_CERTIFIED_REFINEMENT_IMPROVEMENT;
    } else {
      result.inconclusive_reason =
          TubeSurfaceInconclusiveReason::NO_CERTIFIED_REFINEMENT_IMPROVEMENT;
    }
    DeriveLegacyFailure(result, current_w, result.first_failure_reason,
                        result.first_failure_w);
    if (!IsFinite(result.min_cover_radius)) result.min_cover_radius = 0.0;
    DowngradeStaleProofLevel(profile);
    CopyResultToDiagnostics(profile, result);
    return false;
  }

  const bool truncated_before = first > 0U;
  const bool truncated_after = last + 1U < profile.samples.size();
  const std::vector<TubeRawSample, Eigen::aligned_allocator<TubeRawSample>>
      original_samples = profile.samples;
  profile.samples.assign(original_samples.begin() + first,
                         original_samples.begin() + last + 1U);
  profile.preview_start_w = profile.samples.front().w;
  profile.preview_end_w = profile.samples.back().w;
  profile.certified_segment_start_w = profile.preview_start_w;
  profile.certified_segment_end_w = profile.preview_end_w;
  profile.certified_segment_truncated_before =
      profile.certified_segment_truncated_before || truncated_before;
  profile.certified_segment_truncated_after =
      profile.certified_segment_truncated_after || truncated_after;
  if (truncated_before || truncated_after) {
    result.truncation_outcome = truncated_before && truncated_after
        ? TubeSurfaceTruncationOutcome::PREFIX_AND_SUFFIX
        : truncated_before ? TubeSurfaceTruncationOutcome::PREFIX
                           : TubeSurfaceTruncationOutcome::SUFFIX;
  }
  const TubeSurfaceCellEvidence* excluded = CanonicalExcludedEvidence(
      groups, first, last, current_w);
  if (excluded != nullptr) {
    result.terminal_w = excluded->w0;
    result.first_failure_reason = excluded->witness_legacy_reason !=
            TubeStopReason::NONE
        ? excluded->witness_legacy_reason
        : LegacyReason(excluded->outcome, excluded->inconclusive_reason);
    result.first_failure_w = excluded->w0;
    profile.first_truncated_w = excluded->w0;
    profile.first_truncated_reason = result.first_failure_reason;
  }
  profile.filtered_complete = true;
  profile.complete = true;
  profile.obstacle_certified = true;
  result.complete = true;
  result.outcome = TubeSurfaceOutcome::SAFE;
  result.truncated_before = truncated_before;
  result.truncated_after = truncated_after;
  CopyForwardExcludedEvidence(result, profile, groups, first, last, current_w);
  result.certified_start_w = profile.preview_start_w;
  result.certified_end_w = profile.preview_end_w;
  if (excluded == nullptr) result.terminal_w = result.certified_end_w;
  if (!IsFinite(result.min_clearance_margin)) result.min_clearance_margin = 0.0;
  if (!IsFinite(result.min_cover_radius)) result.min_cover_radius = 0.0;
  double centreline_start = 0.0;
  double centreline_end = 0.0;
  result.zero_centerline_continuously_certified =
      profile.cell_geometry_certified && ContinuousCentrelineCoverage(
          profile, result.cell_evidence, current_w, centreline_start,
          centreline_end);
  profile.zero_centerline_continuously_certified =
      result.zero_centerline_continuously_certified;
  if (profile.zero_centerline_continuously_certified) {
    result.certified_start_w = centreline_start;
    result.certified_end_w = centreline_end;
    profile.proof_level = TubeProofLevel::CONTINUOUS_COVER_PROOF;
  } else if (profile.proof_level == TubeProofLevel::CONTINUOUS_COVER_PROOF) {
    profile.proof_level = profile.cell_geometry_certified
        ? TubeProofLevel::FRAME_CELL_PROOF : TubeProofLevel::SAMPLED_EVIDENCE;
  }
  if (profile.diagnostics.min_safety_margin == 0.0 ||
      profile.diagnostics.min_safety_margin > result.min_clearance_margin) {
    profile.diagnostics.min_safety_margin = result.min_clearance_margin;
  }
  result.knot_evidence = profile.validator_knot_evidence;
  DeriveLegacyFailure(result, current_w, result.first_failure_reason,
                      result.first_failure_w);
  CopyResultToDiagnostics(profile, result);
  return true;
}

}  // namespace phase_offset_navigation
