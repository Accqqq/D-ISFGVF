#include <bspline_race/continuous_phase_normal_frame.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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

bool ContinuousPhaseNormalFrame::representedNormalAt(
    const double w, Eigen::Vector3d& tangent, Eigen::Vector3d& tangent_w,
    Eigen::Vector3d& normal, Eigen::Vector3d& normal_w) const {
  if (!path_ || path_->empty() || !finite(w)) return false;
  const double begin = startW();
  const double end = endW();
  if (!finite(begin) || !finite(end) || end < begin) return false;
  const double target = std::max(begin, std::min(end, w));
  if (!tangentAt(target, tangent, tangent_w)) return false;

  // The transport nodes are immutable functions of the path and the
  // constructor seed.  Between nodes we represent the normal by a cubic
  // Hermite curve, then project/normalise that curve in the current tangent
  // plane.  N_w is differentiated from that exact represented curve, rather
  // than being an unrelated ideal Bishop derivative.
  const double span = std::max(0.0, end - begin);
  const std::size_t segment_count = span <= kEpsilon
      ? 1U : static_cast<std::size_t>(std::ceil(span / kTransportStep));
  const double h = span <= kEpsilon
      ? 1.0 : span / static_cast<double>(segment_count);
  std::vector<Eigen::Vector3d> nodes(segment_count + 1U,
                                     Eigen::Vector3d::Zero());
  Eigen::Vector3d node_t;
  Eigen::Vector3d node_tw;
  if (!tangentAt(begin, node_t, node_tw) || !seedNormal(node_t, nodes[0U])) {
    return false;
  }
  for (std::size_t index = 1U; index <= segment_count; ++index) {
    const double node_w = begin + h * static_cast<double>(index);
    Eigen::Vector3d current_t;
    Eigen::Vector3d current_tw;
    if (!tangentAt(node_w, current_t, current_tw)) return false;
    Eigen::Vector3d projected = nodes[index - 1U] - current_t *
        current_t.dot(nodes[index - 1U]);
    const double projected_norm = projected.norm();
    if (!finite(projected_norm) || projected_norm <= kEpsilon) {
      projected = leastParallelAxis(current_t);
    } else {
      projected /= projected_norm;
    }
    if (!finite(projected) || projected.norm() <= kEpsilon) return false;
    if (projected.dot(nodes[index - 1U]) < 0.0) projected = -projected;
    nodes[index] = projected;
  }

  std::size_t segment = 0U;
  double alpha = 0.0;
  if (span > kEpsilon) {
    const double coordinate = (target - begin) / h;
    segment = static_cast<std::size_t>(std::floor(coordinate));
    if (segment >= segment_count) segment = segment_count - 1U;
    alpha = std::max(0.0, std::min(1.0,
        coordinate - static_cast<double>(segment)));
  }
  const auto nodeDerivative = [&](const std::size_t index) -> Eigen::Vector3d {
    if (segment_count == 1U) return (nodes[1U] - nodes[0U]) / h;
    if (index == 0U) return (nodes[1U] - nodes[0U]) / h;
    if (index == segment_count) {
      return (nodes[segment_count] - nodes[segment_count - 1U]) / h;
    }
    return (nodes[index + 1U] - nodes[index - 1U]) / (2.0 * h);
  };
  const Eigen::Vector3d d0 = nodeDerivative(segment);
  const Eigen::Vector3d d1 = nodeDerivative(segment + 1U);
  const Eigen::Vector3d& n0 = nodes[segment];
  const Eigen::Vector3d& n1 = nodes[segment + 1U];
  const double a2 = alpha * alpha;
  const double a3 = a2 * alpha;
  const double h00 = 2.0 * a3 - 3.0 * a2 + 1.0;
  const double h10 = a3 - 2.0 * a2 + alpha;
  const double h01 = -2.0 * a3 + 3.0 * a2;
  const double h11 = a3 - a2;
  const Eigen::Vector3d raw = h00 * n0 + h10 * h * d0 +
      h01 * n1 + h11 * h * d1;
  const double dh00 = 6.0 * a2 - 6.0 * alpha;
  const double dh10 = 3.0 * a2 - 4.0 * alpha + 1.0;
  const double dh01 = -dh00;
  const double dh11 = 3.0 * a2 - 2.0 * alpha;
  const Eigen::Vector3d raw_w = (dh00 * n0 + dh10 * h * d0 +
      dh01 * n1 + dh11 * h * d1) / h;
  const double tangent_projection = raw.dot(tangent);
  const double tangent_projection_w = raw_w.dot(tangent) +
      raw.dot(tangent_w);
  const Eigen::Vector3d projected = raw - tangent * tangent_projection;
  const Eigen::Vector3d projected_w = raw_w - tangent_w * tangent_projection -
      tangent * tangent_projection_w;
  const double norm = projected.norm();
  if (!finite(norm) || norm <= kEpsilon || !finite(projected_w)) return false;
  normal = projected / norm;
  normal_w = (projected_w - normal * normal.dot(projected_w)) / norm;
  if (!finite(normal) || !finite(normal_w) ||
      std::abs(normal.norm() - 1.0) > 1e-6 ||
      std::abs(tangent.dot(normal)) > 1e-6) return false;
  // A sign flip here would change the represented N(w) after its derivative
  // was computed.  Preserve the immutable Hermite representation and fail
  // closed if its projected polynomial crossed the inherited orientation.
  if (normal.dot(n0) < -1e-8) return false;
  return true;
}

bool ContinuousPhaseNormalFrame::query(
    double w, phase_offset_core::NormalFrameQuery& result) const {
  result = phase_offset_core::NormalFrameQuery();
  result.w = w;
  result.path_revision = path_revision_;
  result.frame_revision = frame_revision_;
  result.provenance = "ContinuousPhaseNormalFrame/ProjectedHermiteTransport";
  if (!path_ || path_->empty() || !finite(w) || w < startW() - kEpsilon || w > endW() + kEpsilon) {
    result.invalid_reason = "normal frame query is outside immutable domain";
    return false;
  }
  const double bounded_w = std::max(startW(), std::min(endW(), w));
  Eigen::Vector3d t;
  Eigen::Vector3d n;
  Eigen::Vector3d t_w;
  Eigen::Vector3d n_w;
  if (!representedNormalAt(bounded_w, t, t_w, n, n_w)) {
    result.invalid_reason = "normal frame transport failed";
    return false;
  }
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
  proof.provenance =
      "ContinuousPhaseNormalFrame/ProjectedHermiteTransport/"
      "CertifiedProjectedRawNormLowerBound";
  phase_offset_core::PathCellGeometryCertificate path_proof;
  if (!path_ || !path_->cellBounds(w0, w1, path_proof) ||
      !phase_offset_core::pathCellGeometryCertificateIsComplete(path_proof)) return false;
  phase_offset_core::NormalFrameQuery left, middle, right;
  if (!query(w0, left) || !query(0.5 * (w0 + w1), middle) || !query(w1, right)) return false;
  proof.inf_path_speed = path_proof.inf_p_w_norm;
  proof.sup_path_speed = path_proof.sup_p_w_norm;
  proof.sup_path_acceleration = path_proof.sup_p_ww_norm;
  proof.sup_path_jerk = path_proof.sup_p_www_norm;
  // Prove the derivative of the exact projected-Hermite representation.  For
  // each transport cell, the cubic Hermite coefficients give a closed bound
  // on raw/raw_w.  The tangent derivative is bounded by the immutable path
  // certificate.  A deterministic quarter-cell cover plus the resulting
  // Lipschitz bound proves a positive lower bound for the projected raw norm;
  // without that lower bound this cell is not frame-proof complete.
  const double begin = startW();
  const double end = endW();
  if (!finite(begin) || !finite(end) || end < begin) return false;
  const double full_span = std::max(0.0, end - begin);
  const std::size_t segment_count = full_span <= kEpsilon
      ? 1U : static_cast<std::size_t>(std::ceil(full_span / kTransportStep));
  const double h = full_span <= kEpsilon
      ? 1.0 : full_span / static_cast<double>(segment_count);
  std::vector<Eigen::Vector3d> nodes(segment_count + 1U,
                                     Eigen::Vector3d::Zero());
  Eigen::Vector3d node_t;
  Eigen::Vector3d node_tw;
  if (!tangentAt(begin, node_t, node_tw) || !seedNormal(node_t, nodes[0U])) {
    return false;
  }
  for (std::size_t index = 1U; index <= segment_count; ++index) {
    Eigen::Vector3d current_t;
    Eigen::Vector3d current_tw;
    if (!tangentAt(begin + h * static_cast<double>(index), current_t,
                   current_tw)) {
      return false;
    }
    Eigen::Vector3d projected = nodes[index - 1U] - current_t *
        current_t.dot(nodes[index - 1U]);
    const double projected_norm = projected.norm();
    if (!finite(projected_norm) || projected_norm <= kEpsilon) {
      projected = leastParallelAxis(current_t);
    } else {
      projected /= projected_norm;
    }
    if (!finite(projected) || projected.norm() <= kEpsilon) return false;
    if (projected.dot(nodes[index - 1U]) < 0.0) projected = -projected;
    nodes[index] = projected;
  }
  const auto nodeDerivative = [&](const std::size_t index) {
    if (segment_count == 1U) return (nodes[1U] - nodes[0U]) / h;
    if (index == 0U) return (nodes[1U] - nodes[0U]) / h;
    if (index == segment_count) {
      return (nodes[segment_count] - nodes[segment_count - 1U]) / h;
    }
    return (nodes[index + 1U] - nodes[index - 1U]) / (2.0 * h);
  };
  double represented_sup_derivative = 0.0;
  const double tangent_derivative_bound =
      path_proof.sup_p_ww_norm / path_proof.inf_p_w_norm;
  for (std::size_t index = 0U; index < segment_count; ++index) {
    const double local_start = begin + h * static_cast<double>(index);
    const double local_end = local_start + h;
    const double overlap_start = std::max(w0, local_start);
    const double overlap_end = std::min(w1, local_end);
    if (overlap_end <= overlap_start + kEpsilon) continue;
    const Eigen::Vector3d d0 = nodeDerivative(index);
    const Eigen::Vector3d d1 = nodeDerivative(index + 1U);
    const Eigen::Vector3d c0 = nodes[index];
    const Eigen::Vector3d c1 = h * d0;
    const Eigen::Vector3d c2 = -3.0 * nodes[index] +
        3.0 * nodes[index + 1U] - 2.0 * h * d0 - h * d1;
    const Eigen::Vector3d c3 = 2.0 * nodes[index] -
        2.0 * nodes[index + 1U] + h * d0 + h * d1;
    const double raw_upper = c0.norm() + c1.norm() + c2.norm() + c3.norm();
    const double raw_w_upper = (c1.norm() + 2.0 * c2.norm() +
                                3.0 * c3.norm()) / h;
    const double projected_w_upper = 2.0 * raw_w_upper +
        2.0 * raw_upper * tangent_derivative_bound;
    const double local_span = overlap_end - overlap_start;
    double min_projected_norm = std::numeric_limits<double>::infinity();
    for (int sample_index = 0; sample_index <= 4; ++sample_index) {
      const double sample_w = overlap_start + local_span *
          static_cast<double>(sample_index) / 4.0;
      const double alpha = (sample_w - local_start) / h;
      const Eigen::Vector3d raw = c0 + alpha * c1 + alpha * alpha * c2 +
          alpha * alpha * alpha * c3;
      Eigen::Vector3d sample_t;
      Eigen::Vector3d sample_tw;
      if (!tangentAt(sample_w, sample_t, sample_tw)) return false;
      const Eigen::Vector3d projected = raw - sample_t * raw.dot(sample_t);
      min_projected_norm = std::min(min_projected_norm, projected.norm());
    }
    const double projected_lower = min_projected_norm -
        projected_w_upper * (local_span / 8.0);
    if (!finite(projected_lower) || projected_lower <= 1e-8) return false;
    represented_sup_derivative = std::max(represented_sup_derivative,
        projected_w_upper / projected_lower);
  }
  if (!finite(represented_sup_derivative) || represented_sup_derivative < 0.0) {
    return false;
  }
  proof.sup_normal_derivative = represented_sup_derivative;
  proof.normal_variation_bound = proof.sup_normal_derivative * (w1 - w0);
  proof.tangent_variation_bound = proof.normal_variation_bound;
  proof.valid = finite(proof.sup_normal_derivative) &&
      finite(proof.normal_variation_bound) &&
      finite(proof.tangent_variation_bound) &&
      proof.inf_path_speed > kEpsilon;
  proof.complete = proof.valid;
  return proof.complete;
}

}  // namespace FLAG_Race
