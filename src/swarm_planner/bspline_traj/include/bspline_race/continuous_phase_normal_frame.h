#ifndef BSPLINE_RACE_CONTINUOUS_PHASE_NORMAL_FRAME_H
#define BSPLINE_RACE_CONTINUOUS_PHASE_NORMAL_FRAME_H

#include <phase_offset_core/normal_frame.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>

#include <bspline_race/continuous_phase_path.h>

namespace FLAG_Race {

// Deterministic Bishop-style frame transport for one immutable path owner.
// The frame is value/query-only; no production consumer needs to rebuild N or
// N_w from p_w. Query results are independent of call order and are bound to
// the path/frame revisions supplied at construction.
class ContinuousPhaseNormalFrame final
    : public phase_offset_core::ImmutableNormalFrame {
 public:
  explicit ContinuousPhaseNormalFrame(
      const std::shared_ptr<const ContinuousPhasePath>& path,
      std::uint64_t path_revision = 1U,
      std::uint64_t frame_revision = 1U);
  ContinuousPhaseNormalFrame(
      const std::shared_ptr<const ContinuousPhasePath>& path,
      std::uint64_t path_revision,
      std::uint64_t frame_revision,
      const Eigen::Vector3d& successor_seed_normal);

  bool query(double w, phase_offset_core::NormalFrameQuery& result) const override;
  bool certifyCell(double w0, double w1,
                   phase_offset_core::NormalFrameCellProof& proof) const override;
  double startW() const override;
  double endW() const override;
  std::uint64_t pathRevision() const override { return path_revision_; }
  std::uint64_t frameRevision() const override { return frame_revision_; }

  bool evaluatePathState(double w, ContinuousPhasePathState& state) const;

 private:
  bool tangentAt(double w, Eigen::Vector3d& tangent,
                Eigen::Vector3d& tangent_w) const;
  bool seedNormal(const Eigen::Vector3d& tangent,
                  Eigen::Vector3d& normal) const;
  // Builds the immutable represented N(w) and its derivative from the same
  // smooth, projected interpolation.  Keeping this calculation together is
  // essential: an analytically transported N_w cannot be paired with a
  // discretely projected N without violating the frame contract.
  bool representedNormalAt(double w, Eigen::Vector3d& tangent,
                           Eigen::Vector3d& tangent_w,
                           Eigen::Vector3d& normal,
                           Eigen::Vector3d& normal_w) const;
  bool transportTo(double w, Eigen::Vector3d& tangent,
                   Eigen::Vector3d& normal) const;

  std::shared_ptr<const ContinuousPhasePath> path_;
  std::uint64_t path_revision_ = 0U;
  std::uint64_t frame_revision_ = 0U;
  Eigen::Vector3d seed_normal_ = Eigen::Vector3d::Zero();
};

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_CONTINUOUS_PHASE_NORMAL_FRAME_H
