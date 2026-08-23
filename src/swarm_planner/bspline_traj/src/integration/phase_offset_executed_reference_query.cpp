#include <bspline_race/integration/phase_offset_executed_reference_query.h>

#include <cmath>

namespace FLAG_Race {

PhaseOffsetExecutedReferenceQuery::PhaseOffsetExecutedReferenceQuery(
    const std::shared_ptr<const ContinuousPhasePath>& path, double delta,
    std::uint64_t path_revision, std::uint64_t frame_revision,
    std::uint64_t owner_revision, std::uint64_t query_revision)
    : PhaseOffsetExecutedReferenceQuery(
          path, delta, std::shared_ptr<const ContinuousPhaseNormalFrame>(),
          path_revision, frame_revision, owner_revision, query_revision) {}

PhaseOffsetExecutedReferenceQuery::PhaseOffsetExecutedReferenceQuery(
    const std::shared_ptr<const ContinuousPhasePath>& path, double delta,
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& frame,
    std::uint64_t path_revision, std::uint64_t frame_revision,
    std::uint64_t owner_revision, std::uint64_t query_revision)
    : path_(path),
      // The fallback exists only for legacy synthetic callers of the old
      // constructor.  Production PathTubePair publication always supplies
      // the shared immutable frame_owner explicitly.
      frame_(frame ? frame : std::shared_ptr<const ContinuousPhaseNormalFrame>(
          new ContinuousPhaseNormalFrame(
              path, path_revision == 0U ? 1U : path_revision,
              frame_revision == 0U ? 1U : frame_revision))),
      delta_(delta), path_revision_(path_revision),
      frame_revision_(frame_revision), owner_revision_(owner_revision),
      query_revision_(query_revision) {}

bool PhaseOffsetExecutedReferenceQuery::query(
    double w, phase_offset_navigation::ExecutedReferenceQueryResult& result)
    const {
  result = phase_offset_navigation::ExecutedReferenceQueryResult();
  result.w = w;
  result.path_revision = path_revision_;
  result.frame_revision = frame_revision_;
  result.owner_revision = owner_revision_;
  result.query_revision = query_revision_;
  result.provenance = "PhaseOffsetExecutedReferenceQuery/immutable-path-frame";
  if (!path_ || !frame_ || !std::isfinite(w) ||
      w < path_->startW() || w > path_->endW() || !std::isfinite(delta_)) {
    result.invalid_reason = "executed reference input is invalid";
    return false;
  }
  ContinuousPhasePathState state;
  phase_offset_core::NormalFrameQuery frame;
  if (!path_->evaluate(w, state, false) || !frame_->query(w, frame)) {
    result.invalid_reason = "executed reference owner query failed";
    return false;
  }
  result.r = state.p + frame.N * delta_;
  result.r_w = state.dp_dw + frame.N_w * delta_;
  // The immutable frame supplies p, p_w, N, and N_w.  It does not claim a
  // second derivative of the transported normal, so r_ww remains explicitly
  // unavailable instead of fabricating a zero vector.
  result.r_ww.setZero();
  result.r_ww_valid = false;
  result.valid = result.r.allFinite() && result.r_w.allFinite();
  if (!result.valid) result.invalid_reason = "executed reference is not finite";
  return result.valid;
}

bool PhaseOffsetExecutedReferenceQuery::domain(double& w0, double& w1) const {
  w0 = path_ ? path_->startW() : 0.0;
  w1 = path_ ? path_->endW() : 0.0;
  return path_ && path_->endW() >= path_->startW();
}

}  // namespace FLAG_Race
