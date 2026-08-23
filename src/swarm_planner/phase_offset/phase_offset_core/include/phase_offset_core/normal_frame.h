#pragma once

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <string>

namespace phase_offset_core {

// Result of querying the one immutable normal frame associated with a path
// revision.  The query is deliberately a value type: consumers cannot retain
// a mutable frame or reconstruct a competing normal from the path derivative.
struct NormalFrameQuery {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w = 0.0;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  Eigen::Vector3d T = Eigen::Vector3d::Zero();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  Eigen::Vector3d N_w = Eigen::Vector3d::Zero();
  bool valid = false;
  std::string provenance;
  std::string invalid_reason;
};

// A closed-cell proof for the frame and the path derivatives used by the
// geometric Tube.  This is evidence, not a runtime gate.  A consumer must
// reject the proof unless complete is true and the revisions/range match the
// immutable path it is evaluating.
struct NormalFrameCellProof {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w0 = 0.0;
  double w1 = 0.0;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  double inf_path_speed = 0.0;
  double sup_path_speed = 0.0;
  double sup_path_acceleration = 0.0;
  double sup_path_jerk = 0.0;
  double sup_normal_derivative = 0.0;
  double normal_variation_bound = 0.0;
  double tangent_variation_bound = 0.0;
  bool valid = false;
  bool complete = false;
  std::string provenance;
};

inline bool normalFrameCellProofIsComplete(
    const NormalFrameCellProof& proof) {
  return proof.valid && proof.complete && proof.w1 > proof.w0 &&
      std::isfinite(proof.w0) && std::isfinite(proof.w1) &&
      std::isfinite(proof.inf_path_speed) && proof.inf_path_speed > 0.0 &&
      std::isfinite(proof.sup_path_speed) &&
      proof.sup_path_speed >= proof.inf_path_speed &&
      std::isfinite(proof.sup_path_acceleration) &&
      proof.sup_path_acceleration >= 0.0 &&
      std::isfinite(proof.sup_path_jerk) && proof.sup_path_jerk >= 0.0 &&
      std::isfinite(proof.sup_normal_derivative) &&
      proof.sup_normal_derivative >= 0.0 &&
      std::isfinite(proof.normal_variation_bound) &&
      proof.normal_variation_bound >= 0.0 &&
      std::isfinite(proof.tangent_variation_bound) &&
      proof.tangent_variation_bound >= 0.0;
}

// ImmutableNormalFrame is the only production-facing normal-frame contract.
// Implementations own their path/frame data and expose read-only queries.
class ImmutableNormalFrame {
 public:
  virtual ~ImmutableNormalFrame() = default;

  virtual bool query(double w, NormalFrameQuery& result) const = 0;
  virtual bool certifyCell(double w0, double w1,
                           NormalFrameCellProof& proof) const = 0;
  virtual double startW() const = 0;
  virtual double endW() const = 0;
  virtual std::uint64_t pathRevision() const = 0;
  virtual std::uint64_t frameRevision() const = 0;

  bool query(double w, NormalFrameQuery* result) const {
    return result != nullptr && query(w, *result);
  }
};

}  // namespace phase_offset_core
