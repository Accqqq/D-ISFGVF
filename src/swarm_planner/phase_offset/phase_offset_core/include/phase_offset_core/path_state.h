#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <array>
#include <string>

#include "phase_offset_core/normal_frame.h"

namespace phase_offset_core {

// A closed binary64 interval used only by the proof-facing path DTOs.  The
// endpoints are values of the represented real operation, not sampled
// observations.  Producers must set valid only after every elementary
// operation has been rounded outward and all domains have been checked.
struct Binary64Interval {
  double lower = 0.0;
  double upper = 0.0;
  bool valid = false;
};

struct Binary64VectorInterval {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::array<Binary64Interval, 3U> component;
  bool valid = false;
};

// Proof-only value DTO for one closed structural path cell.  It deliberately
// contains no callback, map, ROS, or mutable owner.  The legacy
// PathCellGeometryCertificate below remains unchanged for existing callers;
// V2 producers use this stronger interval evidence and fail closed when any
// required field is unavailable.
struct CertifiedPathCellV2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w0 = 0.0;
  double w1 = 0.0;
  double anchor_w = 0.0;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t segment_identity = 0U;
  std::uint64_t proof_identity = 0U;

  Binary64VectorInterval anchor_position;
  Binary64VectorInterval anchor_p_w;
  Binary64VectorInterval anchor_p_ww;

  // Complete-cell derivative extrema.  The speed floors are strict lower
  // bounds; all upper bounds are finite closed-cell bounds.
  Binary64Interval inf_p_w_norm;
  Binary64Interval sup_p_w_norm;
  Binary64Interval inf_horizontal_p_w_norm;
  Binary64Interval sup_p_ww_norm;
  Binary64Interval sup_horizontal_p_ww_norm;
  Binary64Interval sup_p_www_norm;
  Binary64Interval sup_normal_derivative;
  Binary64Interval normal_variation;
  Binary64Interval tangent_variation;
  Binary64Interval curvature_variation;
  Binary64Interval midpoint_position_variation;
  Binary64Interval chord_deviation;

  bool horizontal_acceleration_bound_complete = false;
  bool normal_frame_proof_complete = false;
  bool phase_map_proof_complete = false;
  bool complete = false;
  bool valid = false;
  std::string provenance;
};

inline bool binary64IntervalIsComplete(const Binary64Interval& interval,
                                       const bool nonnegative = false) {
  return interval.valid && std::isfinite(interval.lower) &&
      std::isfinite(interval.upper) && interval.lower <= interval.upper &&
      (!nonnegative || interval.lower >= 0.0);
}

inline bool binary64VectorIntervalIsComplete(
    const Binary64VectorInterval& interval) {
  if (!interval.valid) return false;
  for (const Binary64Interval& component : interval.component) {
    if (!binary64IntervalIsComplete(component)) return false;
  }
  return true;
}

inline bool certifiedPathCellV2IsComplete(
    const CertifiedPathCellV2& cell) {
  if (!cell.valid || !cell.complete || !std::isfinite(cell.w0) ||
      !std::isfinite(cell.w1) || !(cell.w1 > cell.w0) ||
      !std::isfinite(cell.anchor_w) || cell.anchor_w < cell.w0 ||
      cell.anchor_w > cell.w1 || cell.segment_identity == 0U ||
      cell.proof_identity == 0U || !binary64VectorIntervalIsComplete(
          cell.anchor_position) || !binary64VectorIntervalIsComplete(
          cell.anchor_p_w) || !binary64VectorIntervalIsComplete(
          cell.anchor_p_ww) || !binary64IntervalIsComplete(cell.inf_p_w_norm,
                                                            true) ||
      !binary64IntervalIsComplete(cell.sup_p_w_norm, true) ||
      !binary64IntervalIsComplete(cell.inf_horizontal_p_w_norm, true) ||
      !binary64IntervalIsComplete(cell.sup_p_ww_norm, true) ||
      !binary64IntervalIsComplete(cell.sup_horizontal_p_ww_norm, true) ||
      !binary64IntervalIsComplete(cell.sup_p_www_norm, true) ||
      !binary64IntervalIsComplete(cell.sup_normal_derivative, true) ||
      !binary64IntervalIsComplete(cell.normal_variation, true) ||
      !binary64IntervalIsComplete(cell.tangent_variation, true) ||
      !binary64IntervalIsComplete(cell.curvature_variation, true) ||
      !binary64IntervalIsComplete(cell.midpoint_position_variation, true) ||
      !binary64IntervalIsComplete(cell.chord_deviation, true) ||
      !cell.horizontal_acceleration_bound_complete ||
      !cell.normal_frame_proof_complete || !cell.phase_map_proof_complete) {
    return false;
  }
  if ((cell.path_revision == 0U) != (cell.frame_revision == 0U) ||
      !isWorldHorizontalCrossProductProvenance(cell.provenance)) {
    return false;
  }
  return cell.inf_p_w_norm.upper <= cell.sup_p_w_norm.upper &&
      cell.inf_horizontal_p_w_norm.lower >
          kHorizontalNormalSpeedEpsilon;
}

struct PathDifferentialState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_ww = Eigen::Vector3d::Zero();
  double w = 0.0;
  // Immutable path/frame provenance. Zero denotes legacy synthetic input
  // which has not yet been bound to a path owner.
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  Eigen::Vector3d T = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  Eigen::Vector3d N_w = Eigen::Vector3d::Zero();
  bool frame_valid = false;
  std::string frame_provenance;
  // Optional owner-provided admissible range used only to scale adaptive
  // refinement. The Tube remains authoritative for final bounds.
  double admissible_delta_lower = 0.0;
  double admissible_delta_upper = 0.0;
  bool valid = false;
};

// A conservative, immutable-path fact for one closed phase cell [w0, w1].
// The producer owns the derivation; consumers must reject this value unless
// pathCellGeometryCertificateIsComplete() succeeds.  In particular, this is
// deliberately not a sampled-path hint: all extrema below bound the complete
// closed cell of one identified source segment.
struct PathCellGeometryCertificate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w0 = 0.0;
  double w1 = 0.0;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  // Identifies the immutable ContinuousPhasePath segment which produced the
  // certificate.  Zero is reserved for an absent/unknown identity.
  std::uint64_t segment_identity = 0U;
  double segment_w0 = 0.0;
  double segment_w1 = 0.0;

  // Explicit speed facts.  The horizontal value is needed because the active
  // normal is normalize(e_z x p_w), rather than normalize(p_w).
  double inf_p_w_norm = 0.0;
  double inf_horizontal_p_w_norm = 0.0;
  double sup_p_w_norm = 0.0;
  double sup_p_ww_norm = 0.0;
  double sup_p_www_norm = 0.0;
  double sup_horizontal_p_ww_norm = 0.0;
  // Optional component-wise absolute p_ww upper bounds supplied by a path
  // producer.  The capability bit is authoritative: a default zero vector
  // without the bit set does not claim zero acceleration.
  Eigen::Vector3d sup_abs_p_ww = Eigen::Vector3d::Zero();
  bool component_acceleration_bound_complete = false;
  bool horizontal_acceleration_bound_complete = false;

  // These are bounds for the same Horizontal-N convention used by
  // GeometryEvaluator: N = normalize(e_z x p_w), with signed planar
  // curvature retained only as compatibility metadata.  The variation values
  // bound the complete closed cell, not only sampled endpoints.
  double sup_N_w_norm = 0.0;
  double sup_abs_curvature = 0.0;
  double normal_variation_bound = 0.0;
  double tangent_variation_bound = 0.0;
  double curvature_variation_bound = 0.0;

  // Conservative position facts: max distance from p(w_mid) and from the
  // endpoint chord, respectively, over this complete cell.
  double midpoint_position_variation_bound = 0.0;
  double chord_deviation_bound = 0.0;

  // Frame-bound and combined regularity evidence carried with the cell.
  bool normal_frame_proof_complete = false;
  bool combined_regularity_proof_complete = false;
  double regularity_speed_min = 0.0;
  double regularity_speed_max = 0.0;
  double admissible_delta_lower = 0.0;
  double admissible_delta_upper = 0.0;
  std::string provenance;

  bool valid = false;
  bool complete = false;
};

inline bool pathCellGeometryCertificateIsComplete(
    const PathCellGeometryCertificate& certificate) {
  if (!certificate.valid || !certificate.complete ||
      certificate.segment_identity == 0U ||
      !std::isfinite(certificate.w0) || !std::isfinite(certificate.w1) ||
      !(certificate.w1 > certificate.w0) ||
      !std::isfinite(certificate.segment_w0) ||
      !std::isfinite(certificate.segment_w1) ||
      !(certificate.segment_w1 > certificate.segment_w0) ||
      certificate.w0 < certificate.segment_w0 ||
      certificate.w1 > certificate.segment_w1 ||
      !std::isfinite(certificate.inf_p_w_norm) ||
      certificate.inf_p_w_norm <= 0.0 ||
      !std::isfinite(certificate.inf_horizontal_p_w_norm) ||
      certificate.inf_horizontal_p_w_norm <=
          kHorizontalNormalSpeedEpsilon ||
      !std::isfinite(certificate.sup_p_w_norm) ||
      certificate.sup_p_w_norm < certificate.inf_p_w_norm ||
      !std::isfinite(certificate.sup_p_ww_norm) ||
      certificate.sup_p_ww_norm < 0.0 ||
      !std::isfinite(certificate.sup_p_www_norm) ||
      certificate.sup_p_www_norm < 0.0 ||
      !std::isfinite(certificate.sup_horizontal_p_ww_norm) ||
      certificate.sup_horizontal_p_ww_norm < 0.0 ||
      !certificate.horizontal_acceleration_bound_complete ||
      !std::isfinite(certificate.sup_N_w_norm) ||
      certificate.sup_N_w_norm < 0.0 ||
      !std::isfinite(certificate.sup_abs_curvature) ||
      certificate.sup_abs_curvature < 0.0 ||
      !std::isfinite(certificate.normal_variation_bound) ||
      certificate.normal_variation_bound < 0.0 ||
      !std::isfinite(certificate.tangent_variation_bound) ||
      certificate.tangent_variation_bound < 0.0 ||
      !std::isfinite(certificate.curvature_variation_bound) ||
      certificate.curvature_variation_bound < 0.0 ||
      !std::isfinite(certificate.midpoint_position_variation_bound) ||
      certificate.midpoint_position_variation_bound < 0.0 ||
      !std::isfinite(certificate.chord_deviation_bound) ||
      certificate.chord_deviation_bound < 0.0) {
    return false;
  }

  // Value-only path bounds may be path-revision bound, but they must not carry
  // a frame revision or a non-canonical frame provenance.  A composed frame
  // certificate may be unbound only in legacy/synthetic fixtures (both
  // revisions zero); otherwise the two identities must be present together.
  if (certificate.normal_frame_proof_complete) {
    if (!isWorldHorizontalCrossProductProvenance(certificate.provenance) ||
        ((certificate.path_revision == 0U) !=
         (certificate.frame_revision == 0U))) {
      return false;
    }
  } else if (certificate.frame_revision != 0U ||
             !certificate.provenance.empty()) {
    return false;
  }

  double required_normal_derivative = 0.0;
  if (!outwardUpperRatio(certificate.sup_horizontal_p_ww_norm,
                         certificate.inf_horizontal_p_w_norm,
                         required_normal_derivative) ||
      certificate.sup_N_w_norm < required_normal_derivative) {
    return false;
  }
  const double span = certificate.w1 - certificate.w0;
  double required_normal_variation = 0.0;
  if (!outwardUpperProduct(certificate.sup_N_w_norm, span,
                           required_normal_variation) ||
      certificate.normal_variation_bound < required_normal_variation) {
    return false;
  }
  double tangent_rate = 0.0;
  if (!outwardUpperRatio(certificate.sup_p_ww_norm,
                         certificate.inf_p_w_norm, tangent_rate)) {
    return false;
  }
  double required_tangent_variation = 0.0;
  if (!outwardUpperProduct(tangent_rate, span,
                           required_tangent_variation) ||
      certificate.tangent_variation_bound < required_tangent_variation) {
    return false;
  }
  return true;
}

inline bool pathCellGeometryCertificateMatches(
    const PathCellGeometryCertificate& certificate,
    const std::uint64_t path_revision,
    const std::uint64_t frame_revision) {
  if (!pathCellGeometryCertificateIsComplete(certificate)) return false;
  // Legacy value-only certificates use zero revisions. Once either side is
  // revision-bound, both identities must agree exactly.
  if (path_revision == 0U && frame_revision == 0U &&
      certificate.path_revision == 0U && certificate.frame_revision == 0U) {
    return true;
  }
  return certificate.path_revision == path_revision &&
      certificate.frame_revision == frame_revision;
}

}  // namespace phase_offset_core
