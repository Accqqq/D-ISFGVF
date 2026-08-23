#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <string>

namespace phase_offset_core {

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

  // These are bounds for the same normal/curvature convention used by
  // GeometryEvaluator: N = normalize(e_z x p_w), and signed planar
  // curvature.  The variation values bound the difference anywhere in the
  // cell, not only at sampled endpoints.
  double sup_N_w_norm = 0.0;
  double sup_abs_curvature = 0.0;
  double normal_variation_bound = 0.0;
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
  return certificate.valid && certificate.complete &&
      certificate.segment_identity != 0U &&
      std::isfinite(certificate.w0) && std::isfinite(certificate.w1) &&
      certificate.w1 > certificate.w0 &&
      std::isfinite(certificate.segment_w0) &&
      std::isfinite(certificate.segment_w1) &&
      certificate.segment_w1 >= certificate.segment_w0 &&
      certificate.w0 >= certificate.segment_w0 &&
      certificate.w1 <= certificate.segment_w1 &&
      std::isfinite(certificate.inf_p_w_norm) &&
      certificate.inf_p_w_norm > 0.0 &&
      std::isfinite(certificate.inf_horizontal_p_w_norm) &&
      certificate.inf_horizontal_p_w_norm >= 0.0 &&
      std::isfinite(certificate.sup_p_w_norm) &&
      certificate.sup_p_w_norm >= certificate.inf_p_w_norm &&
      std::isfinite(certificate.sup_p_ww_norm) &&
      certificate.sup_p_ww_norm >= 0.0 &&
      std::isfinite(certificate.sup_p_www_norm) &&
      certificate.sup_p_www_norm >= 0.0 &&
      std::isfinite(certificate.sup_N_w_norm) &&
      certificate.sup_N_w_norm >= 0.0 &&
      std::isfinite(certificate.sup_abs_curvature) &&
      certificate.sup_abs_curvature >= 0.0 &&
      std::isfinite(certificate.normal_variation_bound) &&
      certificate.normal_variation_bound >= 0.0 &&
      std::isfinite(certificate.curvature_variation_bound) &&
      certificate.curvature_variation_bound >= 0.0 &&
      std::isfinite(certificate.midpoint_position_variation_bound) &&
      certificate.midpoint_position_variation_bound >= 0.0 &&
      std::isfinite(certificate.chord_deviation_bound) &&
      certificate.chord_deviation_bound >= 0.0 &&
      (certificate.path_revision == 0U ||
       certificate.normal_frame_proof_complete);
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
