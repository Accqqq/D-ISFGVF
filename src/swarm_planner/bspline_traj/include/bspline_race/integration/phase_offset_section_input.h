#ifndef BSPLINE_RACE_PHASE_OFFSET_SECTION_INPUT_H
#define BSPLINE_RACE_PHASE_OFFSET_SECTION_INPUT_H

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/continuous_phase_normal_frame.h>
#include <phase_offset_navigation/section_tube.h>
#include <plan_env/local_obstacle_view.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

namespace FLAG_Race {

// Immutable integration-side composition of one path, its matching normal
// frame, and the Section profile built from that same path.  The bundle keeps
// only value facts needed by the planner/adapter boundary: local copied
// domains, build policy, task/frame facts, and the map-owned environment
// validity boundary.  It deliberately does not carry w, delta, previous_u,
// or a LocalObstacleCapture wrapper into navigation/core.
struct SectionPathBundle {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::shared_ptr<const ContinuousPhasePath> path;
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame;
  std::shared_ptr<const phase_offset_navigation::SectionTubeProfile> profile;

  // The local view is copied into the Section build input.  Retain its
  // status and exact domains as build-time facts so a candidate cannot be
  // relabelled with a later map/view revision.
  plan_env::LocalObstacleViewStatus local_view_status =
      plan_env::LocalObstacleViewStatus::INVALID_INPUT;
  plan_env::LocalObstacleBox reference_domain;
  plan_env::LocalObstacleBox obstacle_region;

  phase_offset_navigation::SectionBuildConfig build_config;
  std::string frame_id;
  std::uint64_t task_generation = 0U;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;

  // These pointers are map-owned and shared with LocalObstacleCapture.  A
  // publisher reads state->valid while holding environment_change_mutex;
  // the map is the sole writer and may replace/invalidate the state at a
  // real static-environment change.
  std::shared_ptr<plan_env::EnvironmentValidityState>
      environment_validity;
  std::shared_ptr<std::recursive_mutex> environment_change_mutex;
};

using SectionPathBundlePtr = std::shared_ptr<const SectionPathBundle>;

// Adapts one immutable ContinuousPhasePath (including a contiguous sequence
// of segments) and one complete copied LocalObstacleView into the navigation
// builder's value input.  The caller guarantees that both values use the same
// world frame and does not mutate the path through another mutable alias while
// callbacks may run.  Success only means that the callbacks and copied
// snapshot are available; it is not a tube safety or completeness result.
// When the path producer supplies its optional component-wise acceleration
// capability, the adapter uses that tighter bound; otherwise it conservatively
// falls back to the legacy norm-derived component bound.  The capability is
// value-only and does not alter path identity or the normal/point query
// contracts.
bool makeSectionBuildInput(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    double w_start,
    double w_end,
    const plan_env::LocalObstacleView& view,
    phase_offset_navigation::SectionBuildInput& output,
    std::string& reason);

// Computes a conservative XY-expanded reference ROI for one immutable path
// interval.  Each structural cell uses its exact endpoint positions and the
// producer's whole-cell acceleration bound to cover between-sample extrema;
// callers therefore do not infer map support from a discrete point envelope.
bool makeSectionReferenceRegion(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    double w_start,
    double w_end,
    double half_width,
    double max_step_w,
    std::size_t max_cells,
    plan_env::LocalObstacleBox& region,
    std::string& reason);

// SUPER-style horizontal scan.  Instead of turning every occupied native voxel
// into a separation-plane constraint for every structural cell -- which makes
// the build cost scale with (cells x occupied voxels) and subdivides whenever
// the analytic cell proof fails -- walk the horizontal normal at each path
// sample and stop at the first occupied or unknown voxel.  The cost is
// (samples x scan steps) and does not depend on the occupied-voxel count.
//
// The result is the same kind of value the plane-based builder produced: one
// signed [lower, upper] offset interval per phase knot, in the path's own
// horizontal normal.  It is a truthful prefix: a sample that leaves the known
// domain, or a scan budget that runs out, ends the usable prefix instead of
// silently widening or narrowing it.
struct SectionScanConfig {
  double sample_step_w = 0.10;
  double half_width = 3.0;
  double clearance = 0.4;
  double minimum_reference_speed = 1e-8;
  // M4F: guaranteed lower bound on |1 - kappa*delta| along the cross-section,
  // so the corridor never offers an offset that collapses the active
  // reference.  Values outside [0, 1) disable the cap.
  double min_regularity_ratio = 0.35;
  double curvature_epsilon = 1e-9;
  std::size_t max_scan_checks = 4000000U;
  std::size_t max_knots = 20000U;
};

// Horizontal-section cross-section cap.
//
// For a planar curve the parallel (offset) curve at signed distance d has
// derivative norm |1 - kappa*d| times the path speed, so the active reference
// r(w) = p(w) + N(w)*d degenerates as d approaches the local curvature radius
// 1/kappa: rho_w = p_w + N_w*d -> 0 and the phase rate k1*(alpha+sigma)/|rho_w|
// diverges.  A corridor that claims such an offset is not a usable
// cross-section, so the collapsing side is capped at the offset that keeps the
// factor at or above `min_regularity_ratio`.
//
// With kappa > 0 the curve turns towards +N, so only the +N side collapses and
// only that side is limited; the outer side keeps its obstacle free distance.
// `min_regularity_ratio` outside [0, 1) (or a non-finite input) disables the
// cap, which is reported as an infinite magnitude on both sides.
inline void sectionCurvatureOffsetCaps(const Eigen::Vector3d& p_w,
                                       const Eigen::Vector3d& p_ww,
                                       const double min_regularity_ratio,
                                       const double curvature_epsilon,
                                       double& cap_negative_side,
                                       double& cap_positive_side) {
  cap_negative_side = std::numeric_limits<double>::infinity();
  cap_positive_side = std::numeric_limits<double>::infinity();
  if (!(min_regularity_ratio >= 0.0 && min_regularity_ratio < 1.0) ||
      !std::isfinite(curvature_epsilon) || curvature_epsilon <= 0.0 ||
      !p_w.allFinite() || !p_ww.allFinite()) {
    return;
  }
  const Eigen::Vector3d horizontal_cross = Eigen::Vector3d::UnitZ().cross(p_w);
  const double horizontal_speed = horizontal_cross.norm();
  if (!std::isfinite(horizontal_speed) ||
      horizontal_speed <= curvature_epsilon) {
    return;
  }
  const double speed_cubed =
      horizontal_speed * horizontal_speed * horizontal_speed;
  const double curvature =
      (p_w.x() * p_ww.y() - p_w.y() * p_ww.x()) / speed_cubed;
  if (!std::isfinite(curvature) || std::abs(curvature) <= curvature_epsilon) {
    return;
  }
  const double limit = (1.0 - min_regularity_ratio) / std::abs(curvature);
  if (curvature > 0.0) {
    cap_positive_side = limit;
  } else {
    cap_negative_side = limit;
  }
}

// Compact lookup over one immutable LocalObstacleView.  Building it is
// O(occupied voxels); each query is O(1).
class SectionScanEnvironment {
 public:
  bool build(const plan_env::LocalObstacleView& view, std::string& reason);

  // Returns false when the point is outside the view's known reference domain,
  // i.e. the view cannot say whether the point is free.  On success `occupied`
  // reports the union of the inflated/manual/static layers.
  bool query(const Eigen::Vector3d& point, bool& occupied) const;

  double resolution() const { return resolution_; }

 private:
  bool valid_ = false;
  Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
  Eigen::Vector3i voxel_count_ = Eigen::Vector3i::Zero();
  double resolution_ = 0.0;
  plan_env::LocalObstacleBox known_;
  std::unordered_set<std::int64_t> occupied_;
};

phase_offset_navigation::SectionTubeProfile buildSectionTubeByScan(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    double w_start,
    double w_end,
    const SectionScanEnvironment& environment,
    const SectionScanConfig& config);

}  // namespace FLAG_Race

#endif  // BSPLINE_RACE_PHASE_OFFSET_SECTION_INPUT_H
