#include <bspline_race/continuous_phase_normal_frame.h>

#include <algorithm>
#include <cmath>

namespace FLAG_Race {
namespace {

constexpr double kFrameTangentEpsilon = 1e-12;
constexpr double kDomainEpsilon = 1e-10;

bool finite(const double value) { return std::isfinite(value); }
bool finite(const Eigen::Vector3d& value) { return value.allFinite(); }

}  // namespace

ContinuousPhaseNormalFrame::ContinuousPhaseNormalFrame(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    const std::uint64_t path_revision, const std::uint64_t frame_revision)
    : path_(path), path_revision_(path_revision ? path_revision : 1U),
      frame_revision_(frame_revision ? frame_revision : 1U) {}

// Retained only for source compatibility with older staging callers.  The
// Horizontal-N frame is uniquely determined by the successor path itself.
ContinuousPhaseNormalFrame::ContinuousPhaseNormalFrame(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    const std::uint64_t path_revision, const std::uint64_t frame_revision,
    const Eigen::Vector3d& /*successor_seed_normal*/)
    : ContinuousPhaseNormalFrame(path, path_revision, frame_revision) {}

double ContinuousPhaseNormalFrame::startW() const {
  return path_ ? path_->startW() : 0.0;
}

double ContinuousPhaseNormalFrame::endW() const {
  return path_ ? path_->endW() : 0.0;
}

bool ContinuousPhaseNormalFrame::tangentAt(
    const double w, Eigen::Vector3d& tangent,
    Eigen::Vector3d& tangent_w) const {
  if (!path_ || path_->empty() || !finite(w)) return false;
  ContinuousPhasePathState state;
  if (!path_->evaluate(w, state, false) || !state.valid ||
      !finite(state.dp_dw) || !finite(state.d2p_dw2)) {
    return false;
  }
  const double speed = state.dp_dw.norm();
  if (!finite(speed) || speed <= kFrameTangentEpsilon) return false;
  tangent = state.dp_dw / speed;
  tangent_w = (state.d2p_dw2 - tangent * tangent.dot(state.d2p_dw2)) /
      speed;
  return finite(tangent) && finite(tangent_w);
}

bool ContinuousPhaseNormalFrame::representedNormalAt(
    const double w, Eigen::Vector3d& tangent, Eigen::Vector3d& tangent_w,
    Eigen::Vector3d& normal, Eigen::Vector3d& normal_w) const {
  if (!path_ || path_->empty() || !finite(w)) return false;
  const double begin = startW();
  const double end = endW();
  if (!finite(begin) || !finite(end) || end < begin ||
      w < begin - kDomainEpsilon || w > end + kDomainEpsilon) {
    return false;
  }
  const double bounded_w = std::max(begin, std::min(end, w));
  ContinuousPhasePathState state;
  if (!path_->evaluate(bounded_w, state, false) || !state.valid ||
      !finite(state.dp_dw) || !finite(state.d2p_dw2)) {
    return false;
  }
  const double speed = state.dp_dw.norm();
  if (!finite(speed) || speed <= kFrameTangentEpsilon) return false;
  tangent = state.dp_dw / speed;
  tangent_w = (state.d2p_dw2 - tangent * tangent.dot(state.d2p_dw2)) /
      speed;

  const Eigen::Vector3d horizontal_cross =
      Eigen::Vector3d::UnitZ().cross(state.dp_dw);
  const double q = horizontal_cross.norm();
  if (!finite(horizontal_cross) || !finite(q) ||
      q <= phase_offset_core::kHorizontalNormalSpeedEpsilon) {
    return false;
  }
  normal = horizontal_cross / q;
  const Eigen::Vector3d horizontal_cross_w =
      Eigen::Vector3d::UnitZ().cross(state.d2p_dw2);
  const Eigen::Vector3d projected_horizontal_cross_w =
      horizontal_cross_w - normal * normal.dot(horizontal_cross_w);
  normal_w = projected_horizontal_cross_w / q;
  return finite(normal) && finite(normal_w) &&
      std::abs(normal.norm() - 1.0) <= 1e-8 &&
      std::abs(tangent.dot(normal)) <= 1e-8 &&
      std::abs(normal.z()) <= 1e-12 && std::abs(normal_w.z()) <= 1e-12;
}

bool ContinuousPhaseNormalFrame::query(
    const double w, phase_offset_core::NormalFrameQuery& result) const {
  result = phase_offset_core::NormalFrameQuery();
  result.w = w;
  result.path_revision = path_revision_;
  result.frame_revision = frame_revision_;
  result.provenance =
      "ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct";
  if (!path_ || path_->empty() || !finite(w) ||
      w < startW() - kDomainEpsilon || w > endW() + kDomainEpsilon) {
    result.invalid_reason = "normal frame query is outside immutable domain";
    return false;
  }
  Eigen::Vector3d tangent;
  Eigen::Vector3d tangent_w;
  Eigen::Vector3d normal;
  Eigen::Vector3d normal_w;
  if (!representedNormalAt(w, tangent, tangent_w, normal, normal_w)) {
    result.invalid_reason =
        "horizontal normal capability is unavailable at this path point";
    return false;
  }
  result.w = std::max(startW(), std::min(endW(), w));
  result.T = tangent;
  result.N = normal;
  result.N_w = normal_w;
  result.valid = true;
  return true;
}

bool ContinuousPhaseNormalFrame::evaluatePathState(
    const double w, ContinuousPhasePathState& state) const {
  state = ContinuousPhasePathState();
  if (!path_ || !path_->evaluate(w, state, false)) return false;
  phase_offset_core::NormalFrameQuery frame;
  if (!query(w, frame)) {
    // The planner path remains a valid full-3-D centerline when Horizontal-N
    // capability is unavailable.  Return its tangent/derivatives with an
    // explicitly unbound frame so neutral geometry can continue while any
    // nonzero PhaseOffset capability fails closed.
    if (!state.valid || !finite(state.dp_dw) || !finite(state.d2p_dw2)) {
      return false;
    }
    const double speed = state.dp_dw.norm();
    if (!finite(speed) || speed <= kFrameTangentEpsilon) return false;
    state.T = state.dp_dw / speed;
    state.N.setZero();
    state.N_w.setZero();
    state.path_revision = path_revision_;
    state.frame_revision = frame_revision_;
    state.frame_valid = false;
    state.frame_provenance =
        "ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct/Unavailable";
    state.valid = true;
    return true;
  }
  state.T = frame.T;
  state.N = frame.N;
  state.N_w = frame.N_w;
  state.path_revision = frame.path_revision;
  state.frame_revision = frame.frame_revision;
  state.frame_valid = true;
  state.frame_provenance = frame.provenance;
  state.valid = true;
  return true;
}

bool ContinuousPhaseNormalFrame::certifyCell(
    const double w0, const double w1,
    phase_offset_core::NormalFrameCellProof& proof) const {
  proof = phase_offset_core::NormalFrameCellProof();
  proof.w0 = w0;
  proof.w1 = w1;
  proof.path_revision = path_revision_;
  proof.frame_revision = frame_revision_;
  proof.provenance =
      "ContinuousPhaseNormalFrame/WorldHorizontalCrossProduct";
  phase_offset_core::PathCellGeometryCertificate path_proof;
  if (!path_ || !path_->cellBounds(w0, w1, path_proof) ||
      !phase_offset_core::pathCellGeometryCertificateIsComplete(path_proof)) {
    return false;
  }
  if (path_proof.path_revision != 0U &&
      path_proof.path_revision != path_revision_) return false;
  if (path_proof.frame_revision != 0U &&
      path_proof.frame_revision != frame_revision_) return false;
  if (path_proof.normal_frame_proof_complete &&
      !phase_offset_core::isWorldHorizontalCrossProductProvenance(
          path_proof.provenance)) {
    return false;
  }
  phase_offset_core::NormalFrameQuery left;
  phase_offset_core::NormalFrameQuery middle;
  phase_offset_core::NormalFrameQuery right;
  if (!query(w0, left) || !query(0.5 * (w0 + w1), middle) ||
      !query(w1, right)) {
    return false;
  }
  proof.inf_path_speed = path_proof.inf_p_w_norm;
  proof.sup_path_speed = path_proof.sup_p_w_norm;
  proof.sup_path_acceleration = path_proof.sup_p_ww_norm;
  proof.sup_path_jerk = path_proof.sup_p_www_norm;
  proof.inf_horizontal_path_speed = path_proof.inf_horizontal_p_w_norm;
  proof.sup_horizontal_p_ww_norm = path_proof.sup_horizontal_p_ww_norm;
  proof.horizontal_acceleration_bound_complete =
      path_proof.horizontal_acceleration_bound_complete;
  if (!finite(proof.inf_horizontal_path_speed) ||
      proof.inf_horizontal_path_speed <=
          phase_offset_core::kHorizontalNormalSpeedEpsilon ||
      !finite(proof.sup_horizontal_p_ww_norm) ||
      !proof.horizontal_acceleration_bound_complete) {
    return false;
  }
  // ContinuousPhasePath is the certificate producer.  Preserve its
  // outward-rounded Horizontal-N and tangent evidence exactly instead of
  // recomputing a/q or multiplying by the cell span with ordinary floating
  // point arithmetic, which could silently weaken the proof.
  proof.sup_normal_derivative = path_proof.sup_N_w_norm;
  proof.normal_variation_bound = path_proof.normal_variation_bound;
  proof.tangent_variation_bound = path_proof.tangent_variation_bound;
  proof.valid = finite(proof.sup_normal_derivative) &&
      proof.sup_normal_derivative >= 0.0 &&
      finite(proof.normal_variation_bound) &&
      proof.normal_variation_bound >= 0.0 &&
      finite(proof.tangent_variation_bound) &&
      proof.tangent_variation_bound >= 0.0 && proof.w1 > proof.w0;
  proof.complete = proof.valid;
  return phase_offset_core::normalFrameCellProofIsComplete(proof);
}

}  // namespace FLAG_Race
