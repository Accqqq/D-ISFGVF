#ifndef BSPLINE_RACE_CONTINUOUS_PHASE_NORMAL_FRAME_H
#define BSPLINE_RACE_CONTINUOUS_PHASE_NORMAL_FRAME_H

#include <phase_offset_core/normal_frame.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>

#include <bspline_race/continuous_phase_path.h>

namespace FLAG_Race {

// Deterministic world-horizontal cross-product frame for one immutable path
// owner.  The frame is value/query-only; N and N_w are derived from the same
// full-3-D path query and are bound to the path/frame revisions supplied at
// construction.
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

  // Builds the deterministic world-horizontal frame directly from one path
  // state.  This is shared by the ordinary path query and cell-aware
  // integration callbacks so both use exactly the same arithmetic and
  // capability thresholds.
  static bool computeGeometry(const ContinuousPhasePathState& state,
                              Eigen::Vector3d& tangent,
                              Eigen::Vector3d& tangent_w,
                              Eigen::Vector3d& normal,
                              Eigen::Vector3d& normal_w);

  bool evaluatePathState(double w, ContinuousPhasePathState& state) const;

 private:
  bool tangentAt(double w, Eigen::Vector3d& tangent,
                 Eigen::Vector3d& tangent_w) const;
  // Builds the unique Horizontal-N representation and its exact analytic
  // derivative from p_w and p_ww at the same phase location.
  bool representedNormalAt(double w, Eigen::Vector3d& tangent,
                           Eigen::Vector3d& tangent_w,
                           Eigen::Vector3d& normal,
                           Eigen::Vector3d& normal_w) const;

  std::shared_ptr<const ContinuousPhasePath> path_;
  std::uint64_t path_revision_ = 0U;
  std::uint64_t frame_revision_ = 0U;
};

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_CONTINUOUS_PHASE_NORMAL_FRAME_H
