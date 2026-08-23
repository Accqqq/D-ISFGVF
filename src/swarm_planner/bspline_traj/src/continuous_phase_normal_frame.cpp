#include <bspline_race/continuous_phase_normal_frame.h>

#include <algorithm>
#include <cmath>

namespace FLAG_Race {
namespace {
constexpr double kEpsilon = 1e-10;
constexpr double kTransportStep = 0.05;
bool finite(double x) { return std::isfinite(x); }
bool finite(const Eigen::Vector3d& x) { return x.allFinite(); }

Eigen::Vector3d leastParallelAxis(const Eigen::Vector3d& t) {
  const Eigen::Vector3d axes[] = {Eigen::Vector3d::UnitX(),
                                  Eigen::Vector3d::UnitY(),
                                  Eigen::Vector3d::UnitZ()};
  int best = 0;
  double alignment = std::abs(t.dot(axes[0]));
  for (int i = 1; i < 3; ++i) {
    const double next = std::abs(t.dot(axes[i]));
    if (next < alignment) { alignment = next; best = i; }
  }
  Eigen::Vector3d n = axes[best] - t * t.dot(axes[best]);
  const double norm = n.norm();
  if (!finite(norm) || norm <= kEpsilon) return Eigen::Vector3d::Zero();
  return n / norm;
}

bool tangentFromState(const ContinuousPhasePathState& state,
                      Eigen::Vector3d& t, Eigen::Vector3d& t_w) {
  if (!state.valid || !finite(state.dp_dw) || !finite(state.d2p_dw2)) return false;
  const double speed = state.dp_dw.norm();
  if (!finite(speed) || speed <= kEpsilon) return false;
  t = state.dp_dw / speed;
  t_w = (state.d2p_dw2 - t * t.dot(state.d2p_dw2)) / speed;
  return finite(t) && finite(t_w);
}
}  // namespace

ContinuousPhaseNormalFrame::ContinuousPhaseNormalFrame(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    std::uint64_t path_revision, std::uint64_t frame_revision)
    : path_(path), path_revision_(path_revision ? path_revision : 1U),
      frame_revision_(frame_revision ? frame_revision : 1U) {}

ContinuousPhaseNormalFrame::ContinuousPhaseNormalFrame(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    std::uint64_t path_revision, std::uint64_t frame_revision,
    const Eigen::Vector3d& successor_seed_normal)
    : ContinuousPhaseNormalFrame(path, path_revision, frame_revision) {
  if (finite(successor_seed_normal) && successor_seed_normal.norm() > kEpsilon) {
    seed_normal_ = successor_seed_normal.normalized();
  }
}

double ContinuousPhaseNormalFrame::startW() const { return path_ ? path_->startW() : 0.0; }
double ContinuousPhaseNormalFrame::endW() const { return path_ ? path_->endW() : 0.0; }

bool ContinuousPhaseNormalFrame::tangentAt(double w, Eigen::Vector3d& t,
                                           Eigen::Vector3d& t_w) const {
  if (!path_ || path_->empty() || !finite(w)) return false;
  ContinuousPhasePathState state;
  return path_->evaluate(w, state, false) && tangentFromState(state, t, t_w);
}

bool ContinuousPhaseNormalFrame::seedNormal(const Eigen::Vector3d& t,
                                            Eigen::Vector3d& n) const {
  if (seed_normal_.allFinite() && seed_normal_.norm() > kEpsilon) {
    n = seed_normal_ - t * t.dot(seed_normal_);
    const double norm = n.norm();
    if (finite(norm) && norm > kEpsilon) {
      n /= norm;
      return true;
    }
  }
  n = leastParallelAxis(t);
  return finite(n) && n.norm() > kEpsilon;
}

bool ContinuousPhaseNormalFrame::transportTo(double w, Eigen::Vector3d& t,
                                             Eigen::Vector3d& n) const {
  if (!path_ || path_->empty() || !finite(w)) return false;
  const double begin = startW();
  const double target = std::max(begin, std::min(endW(), w));
  Eigen::Vector3d previous_t;
  Eigen::Vector3d previous_tw;
  if (!tangentAt(begin, previous_t, previous_tw) || !seedNormal(previous_t, n)) return false;
  Eigen::Vector3d previous_n = n;
  const double span = std::max(0.0, target - begin);
  const int full_steps = static_cast<int>(std::floor(
      span / kTransportStep + 1e-12));
  for (int i = 1; i <= full_steps; ++i) {
    const double sample_w = begin + kTransportStep * static_cast<double>(i);
    Eigen::Vector3d current_t;
    Eigen::Vector3d current_tw;
    if (!tangentAt(sample_w, current_t, current_tw)) return false;
    n -= current_t * current_t.dot(n);
    const double norm = n.norm();
    if (!finite(norm) || norm <= kEpsilon) n = leastParallelAxis(current_t);
    else n /= norm;
    if (n.dot(previous_n) < 0.0) n = -n;
    if (!finite(n) || n.norm() <= kEpsilon) return false;
    previous_t = current_t;
    previous_n = n;
  }
  const double grid_w = begin + kTransportStep * static_cast<double>(full_steps);
  if (target > grid_w + 1e-12) {
    Eigen::Vector3d current_t;
    Eigen::Vector3d current_tw;
    if (!tangentAt(target, current_t, current_tw)) return false;
    n -= current_t * current_t.dot(n);
    const double norm = n.norm();
    if (!finite(norm) || norm <= kEpsilon) n = leastParallelAxis(current_t);
    else n /= norm;
    if (n.dot(previous_n) < 0.0) n = -n;
    if (!finite(n) || n.norm() <= kEpsilon) return false;
    previous_t = current_t;
  }
  t = previous_t;
  return finite(t) && finite(n);
}

bool ContinuousPhaseNormalFrame::query(
    double w, phase_offset_core::NormalFrameQuery& result) const {
  result = phase_offset_core::NormalFrameQuery();
  result.w = w;
  result.path_revision = path_revision_;
  result.frame_revision = frame_revision_;
  result.provenance = "ContinuousPhaseNormalFrame/BishopTransport";
  if (!path_ || path_->empty() || !finite(w) || w < startW() - kEpsilon || w > endW() + kEpsilon) {
    result.invalid_reason = "normal frame query is outside immutable domain";
    return false;
  }
  const double bounded_w = std::max(startW(), std::min(endW(), w));
  Eigen::Vector3d t;
  Eigen::Vector3d n;
  Eigen::Vector3d t_w;
  if (!transportTo(bounded_w, t, n) || !tangentAt(bounded_w, t, t_w)) {
    result.invalid_reason = "normal frame transport failed";
    return false;
  }
  const Eigen::Vector3d n_w = -t_w.dot(n) * t;
  if (!finite(t) || !finite(n) || !finite(n_w) || std::abs(t.norm() - 1.0) > 1e-6 ||
      std::abs(n.norm() - 1.0) > 1e-6 || std::abs(t.dot(n)) > 1e-6) {
    result.invalid_reason = "normal frame result is not orthonormal";
    return false;
  }
  result.w = bounded_w;
  result.T = t;
  result.N = n;
  result.N_w = n_w;
  result.valid = true;
  return true;
}

bool ContinuousPhaseNormalFrame::evaluatePathState(double w,
                                                   ContinuousPhasePathState& state) const {
  state = ContinuousPhasePathState();
  if (!path_ || !path_->evaluate(w, state, false)) return false;
  phase_offset_core::NormalFrameQuery frame;
  if (!query(w, frame)) return false;
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
    double w0, double w1, phase_offset_core::NormalFrameCellProof& proof) const {
  proof = phase_offset_core::NormalFrameCellProof();
  proof.w0 = w0;
  proof.w1 = w1;
  proof.path_revision = path_revision_;
  proof.frame_revision = frame_revision_;
  proof.provenance = "ContinuousPhaseNormalFrame/BishopTransport";
  phase_offset_core::PathCellGeometryCertificate path_proof;
  if (!path_ || !path_->cellBounds(w0, w1, path_proof) ||
      !phase_offset_core::pathCellGeometryCertificateIsComplete(path_proof)) return false;
  phase_offset_core::NormalFrameQuery left, middle, right;
  if (!query(w0, left) || !query(0.5 * (w0 + w1), middle) || !query(w1, right)) return false;
  proof.inf_path_speed = path_proof.inf_p_w_norm;
  proof.sup_path_speed = path_proof.sup_p_w_norm;
  proof.sup_path_acceleration = path_proof.sup_p_ww_norm;
  proof.sup_path_jerk = path_proof.sup_p_www_norm;
  // The path certificate supplies closed-cell derivative bounds.  Endpoint
  // samples are queried only to validate that this immutable frame is defined
  // on the cell; their maxima are never promoted to a continuous proof.
  proof.sup_normal_derivative = std::max(
      path_proof.sup_N_w_norm,
      path_proof.sup_p_ww_norm / path_proof.inf_p_w_norm);
  proof.normal_variation_bound = std::max(
      path_proof.normal_variation_bound,
      proof.sup_normal_derivative * (w1 - w0));
  proof.tangent_variation_bound = proof.normal_variation_bound;
  proof.valid = finite(proof.sup_normal_derivative) &&
      finite(proof.normal_variation_bound) &&
      finite(proof.tangent_variation_bound) &&
      proof.inf_path_speed > kEpsilon;
  proof.complete = proof.valid;
  return proof.complete;
}

}  // namespace FLAG_Race
