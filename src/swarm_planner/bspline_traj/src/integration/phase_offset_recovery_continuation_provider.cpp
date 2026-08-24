#include "bspline_race/integration/phase_offset_recovery_continuation_provider.h"

#include "bspline_race/integration/phase_offset_executed_reference_query.h"

#include <cmath>

namespace FLAG_Race {
namespace {

bool Finite(const double value) { return std::isfinite(value); }

bool ValidFiniteDomain(const double start, const double end) {
  return Finite(start) && Finite(end) && end >= start;
}

bool AttachPathSecondDerivative(
    const std::shared_ptr<const ContinuousPhasePath>& path, const double w,
    const double delta,
    phase_offset_navigation::RecoveryReferenceJet& jet) {
  // The frozen frame contract does not expose N_ww.  Exact r_ww is therefore
  // optional and this helper only supplies the zero-offset capability; the
  // Batch-B first-order continuation path never requires it.
  if (!path || !Finite(w) || !Finite(delta) || std::abs(delta) > 1e-12) {
    return false;
  }
  ContinuousPhasePathState state;
  if (!path->evaluate(w, state, false) || !state.valid ||
      !state.d2p_dw2.allFinite()) {
    return false;
  }
  jet.r_ww = state.d2p_dw2;
  jet.r_ww_valid = true;
  return true;
}

}  // namespace

bool PhaseOffsetRecoveryContinuationProvider::propose(
    const PhaseOffsetRecoveryContinuationInput& input,
    PhaseOffsetRecoveryContinuationOutput& output) {
  output = PhaseOffsetRecoveryContinuationOutput();
  output.provenance =
      "bspline_race/phase_offset_recovery_continuation_provider";
  if (!input.source_path || !input.source_frame || !input.target_path ||
      !input.target_frame || !input.source_jet.valid ||
      input.recovery_session == 0U || !Finite(input.source_w) ||
      !Finite(input.target_w) || !Finite(input.delta) ||
      !ValidFiniteDomain(input.finite_domain_start_w,
                         input.finite_domain_end_w) ||
      input.source_w < input.source_path->startW() ||
      input.source_w > input.source_path->endW() ||
      input.target_w < input.target_path->startW() ||
      input.target_w > input.target_path->endW()) {
    output.invalid_reason = "continuation input is invalid";
    return false;
  }
  if (input.source_jet.path_revision != input.source_frame->pathRevision() ||
      input.source_jet.frame_revision != input.source_frame->frameRevision()) {
    output.invalid_reason = "source reference jet revision mismatch";
    return false;
  }
  ContinuousPhasePathState source_state;
  ContinuousPhasePathState target_state;
  phase_offset_core::NormalFrameQuery source_frame_query;
  phase_offset_core::NormalFrameQuery target_frame_query;
  if (!input.source_path->evaluate(input.source_w, source_state, false) ||
      !input.target_path->evaluate(input.target_w, target_state, false) ||
      !input.source_frame->query(input.source_w, source_frame_query) ||
      !input.target_frame->query(input.target_w, target_frame_query) ||
      !source_state.valid || !target_state.valid ||
      !source_frame_query.valid || !target_frame_query.valid) {
    output.invalid_reason = "base-path or frame seam query failed";
    return false;
  }
  // Reject a target with no usable phase speed before reporting seam
  // continuity.  The speed witness is a local precondition for every
  // continuation, and this ordering keeps the diagnostic specific even when
  // a degenerate target also cannot match the source tangent at the seam.
  const double geometric_speed = target_state.dp_dw.norm();
  if (!Finite(geometric_speed) || geometric_speed <= 1e-8) {
    output.invalid_reason = "target continuation has no finite phase speed";
    return false;
  }
  // C2_CONTINUATION names the inherited base path.  Validate its exact
  // p/p_w/p_ww seam and the first-order immutable frame seam independently of
  // the executed-reference jet; the provider does not construct a connector.
  constexpr double kBaseSeamTolerance = 1e-8;
  constexpr double kFrameSeamTolerance = 1e-7;
  if ((source_state.p - target_state.p).norm() > kBaseSeamTolerance ||
      (source_state.dp_dw - target_state.dp_dw).norm() > kBaseSeamTolerance ||
      (source_state.d2p_dw2 - target_state.d2p_dw2).norm() >
          kBaseSeamTolerance ||
      (source_frame_query.N - target_frame_query.N).norm() >
          kFrameSeamTolerance ||
      (source_frame_query.N_w - target_frame_query.N_w).norm() >
          kFrameSeamTolerance) {
    output.invalid_reason = "base-path or first-order frame seam is not continuous";
    return false;
  }
  phase_offset_navigation::RecoveryReferenceJet source_at_w;
  if (!phase_offset_navigation::makeRecoveryReferenceJet(
          *std::make_shared<PhaseOffsetExecutedReferenceQuery>(
              input.source_path, input.delta, input.source_frame,
              input.source_frame->pathRevision(),
              input.source_frame->frameRevision(), input.source_owner_revision,
              input.source_jet.query_revision),
          input.source_w, input.delta, source_at_w) ||
      (input.require_full_jet && !AttachPathSecondDerivative(
          input.source_path, input.source_w, input.delta, source_at_w)) ||
      !source_at_w.matches(input.source_jet, input.require_full_jet)) {
    output.invalid_reason = "source reference jet continuity contract failed";
    return false;
  }
  ContinuousPhasePathState state;
  phase_offset_core::NormalFrameQuery frame_query;
  if (!input.target_path->evaluate(input.target_w, state, false) ||
      !input.target_frame->query(input.target_w, frame_query) ||
      !state.valid || !frame_query.valid) {
    output.invalid_reason = "target continuation geometry query failed";
    return false;
  }
  phase_offset_navigation::RecoveryReferenceJet target_jet;
  auto target_query = std::make_shared<PhaseOffsetExecutedReferenceQuery>(
      input.target_path, input.delta, input.target_frame,
      input.target_frame->pathRevision(), input.target_frame->frameRevision(),
      input.target_owner_revision, input.source_jet.query_revision);
  if (!phase_offset_navigation::makeRecoveryReferenceJet(
          *target_query, input.target_w, input.delta, target_jet)) {
    output.invalid_reason = "target reference jet is invalid";
    return false;
  }
  output.full_jet_valid = AttachPathSecondDerivative(
      input.target_path, input.target_w, input.delta, target_jet) &&
      (input.source_jet.r_ww_valid || !input.require_full_jet);
  if (input.require_full_jet &&
      (!input.source_jet.r_ww_valid || !target_jet.r_ww_valid)) {
    output.invalid_reason =
        "complete source/target reference jet is unavailable";
    return false;
  }
  output.target_state = state;
  output.target_frame_query = frame_query;
  output.target_jet = target_jet;
  if (input.require_time_progress) {
    // A path derivative norm is not an actual time/phase progression fact.
    // Production C2 recovery must provide a validated, revision-bound
    // s_dot witness from the caller; otherwise fail closed.
    if (!input.actual_s_dot_valid || !Finite(input.validated_s_dot) ||
        input.validated_s_dot < input.v_s_min ||
        input.s_dot_revision != input.target_owner_revision ||
        input.s_dot_provenance.empty()) {
      output.invalid_reason =
          "validated actual s_dot provenance is unavailable";
      return false;
    }
    output.s_dot = input.validated_s_dot;
  } else {
    output.s_dot = geometric_speed;
  }
  output.time_progress = input.target_w - input.source_w;
  output.time_progress_valid = Finite(output.time_progress) &&
      (!input.require_time_progress ||
       (Finite(input.min_time_progress) && input.min_time_progress >= 0.0 &&
        output.time_progress >= input.min_time_progress));
  if (input.require_time_progress &&
      (!Finite(input.v_s_min) || input.v_s_min <= 0.0 ||
       output.s_dot < input.v_s_min || !output.time_progress_valid)) {
    output.invalid_reason =
        "validated v_s_min/time-progress contract failed";
    return false;
  }
  output.c2_continuous = (target_jet.r - input.source_jet.r).norm() < 1e-3 &&
      (target_jet.r_w - input.source_jet.r_w).norm() < 1e-2 &&
      (!input.require_full_jet ||
       (target_jet.r_ww - input.source_jet.r_ww).norm() < 1e-1);
  output.finite_domain = ValidFiniteDomain(input.finite_domain_start_w,
                                           input.finite_domain_end_w) &&
      input.target_w >= input.finite_domain_start_w &&
      input.target_w <= input.finite_domain_end_w;
  output.valid = output.c2_continuous && output.finite_domain;
  if (!output.valid) {
    output.invalid_reason = output.c2_continuous
        ? "continuation target lies outside finite domain"
        : "source/target reference jet is not C2-continuous";
  }
  return output.valid;
}

bool PhaseOffsetRecoveryContinuationProvider::validateSelectedTimeProgress(
    const PhaseOffsetRecoveryContinuationInput& input,
    const double actual_s_dot, const std::uint64_t actual_s_dot_revision,
    const std::string& actual_s_dot_provenance,
    PhaseOffsetRecoveryContinuationOutput& output) {
  output.s_dot = actual_s_dot;
  output.time_progress = input.target_w - input.source_w;
  output.time_progress_valid = Finite(actual_s_dot) &&
      Finite(output.time_progress) && Finite(input.v_s_min) &&
      input.v_s_min > 0.0 && actual_s_dot >= input.v_s_min &&
      actual_s_dot_revision != 0U &&
      actual_s_dot_revision == input.target_owner_revision &&
      !actual_s_dot_provenance.empty() &&
      Finite(input.min_time_progress) && input.min_time_progress >= 0.0 &&
      output.time_progress >= input.min_time_progress;
  if (!output.time_progress_valid) {
    output.invalid_reason =
        "selected actual s_dot revision/time-progress contract failed";
    return false;
  }
  output.invalid_reason.clear();
  return true;
}

}  // namespace FLAG_Race
