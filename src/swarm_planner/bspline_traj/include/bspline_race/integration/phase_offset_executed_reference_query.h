#pragma once

#include <phase_offset_navigation/immutable_executed_reference_query.h>

#include <bspline_race/continuous_phase_normal_frame.h>

namespace FLAG_Race {

class PhaseOffsetExecutedReferenceQuery final
    : public phase_offset_navigation::ImmutableExecutedReferenceQuery {
 public:
  PhaseOffsetExecutedReferenceQuery(
      const std::shared_ptr<const ContinuousPhasePath>& path,
      double delta,
      std::uint64_t path_revision,
      std::uint64_t frame_revision,
      std::uint64_t owner_revision,
      std::uint64_t query_revision);
  PhaseOffsetExecutedReferenceQuery(
      const std::shared_ptr<const ContinuousPhasePath>& path,
      double delta,
      const std::shared_ptr<const ContinuousPhaseNormalFrame>& frame,
      std::uint64_t path_revision,
      std::uint64_t frame_revision,
      std::uint64_t owner_revision,
      std::uint64_t query_revision);

  bool query(double w,
             phase_offset_navigation::ExecutedReferenceQueryResult& result)
      const override;
  bool domain(double& w0, double& w1) const override;
  std::uint64_t pathRevision() const override { return path_revision_; }
  std::uint64_t frameRevision() const override { return frame_revision_; }
  std::uint64_t ownerRevision() const override { return owner_revision_; }
  std::uint64_t queryRevision() const override { return query_revision_; }

 private:
  std::shared_ptr<const ContinuousPhasePath> path_;
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame_;
  double delta_ = 0.0;
  std::uint64_t path_revision_ = 0U;
  std::uint64_t frame_revision_ = 0U;
  std::uint64_t owner_revision_ = 0U;
  std::uint64_t query_revision_ = 0U;
};

}  // namespace FLAG_Race
