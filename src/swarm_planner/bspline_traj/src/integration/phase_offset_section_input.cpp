#include <bspline_race/integration/phase_offset_section_input.h>

#include <bspline_race/continuous_phase_normal_frame.h>
#include <phase_offset_core/normal_frame.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace FLAG_Race {
namespace {

using phase_offset_navigation::SectionBox;
using phase_offset_navigation::SectionBuildInput;
using phase_offset_navigation::SectionCellBounds;
using phase_offset_navigation::SectionPathSample;
using phase_offset_navigation::SectionEnvironment;

bool finite(const double value) { return std::isfinite(value); }

bool finite(const Eigen::Vector3d& value) { return value.allFinite(); }

bool finitePathState(const ContinuousPhasePathState& state) {
  return finite(state.p) && finite(state.dp_dw) &&
      finite(state.d2p_dw2) && finite(state.vel);
}

using Segment = ContinuousPhasePath::Segment;

const Segment* findSegmentForCell(
    const ContinuousPhasePath& path, const double w0, const double w1) {
  if (!finite(w0) || !finite(w1) || !(w1 > w0)) return nullptr;
  const Segment* selected = nullptr;
  for (const Segment& segment : path.segments()) {
    if (w0 >= segment.w0 && w1 <= segment.w1) {
      if (selected != nullptr) return nullptr;
      selected = &segment;
    }
  }
  return selected;
}

bool collectRelevantSegments(const ContinuousPhasePath& path,
                             const double w_start, const double w_end,
                             std::vector<const Segment*>& relevant,
                             std::string& reason) {
  relevant.clear();
  reason.clear();
  for (const Segment& segment : path.segments()) {
    if (!finite(segment.w0) || !finite(segment.w1) ||
        !(segment.w1 > segment.w0)) {
      // Invalid segments outside the requested range cannot affect this
      // immutable query.  Invalid related segments fail closed below.
      if (segment.w1 >= w_start && segment.w0 <= w_end) {
        reason = "invalid_related_segment_domain";
        return false;
      }
      continue;
    }
    if (segment.w1 >= w_start && segment.w0 <= w_end) {
      relevant.push_back(&segment);
    }
  }
  if (relevant.empty() || relevant.front()->w0 > w_start ||
      relevant.back()->w1 < w_end) {
    reason = "phase_range_not_covered";
    return false;
  }
  for (std::size_t index = 1U; index < relevant.size(); ++index) {
    const Segment& previous = *relevant[index - 1U];
    const Segment& current = *relevant[index];
    if (previous.w1 != current.w0) {
      reason = previous.w1 < current.w0
          ? "phase_range_gap_between_segments"
          : "phase_range_overlap_between_segments";
      return false;
    }
  }
  return true;
}

bool seamC2Compatible(const Segment& left, const Segment& right,
                      std::string& reason) {
  reason.clear();
  const double seam = left.w1;
  ContinuousPhasePathState left_state;
  ContinuousPhasePathState right_state;
  if (!left.evaluate(seam, left_state) || !right.evaluate(seam, right_state) ||
      !left_state.valid || !right_state.valid ||
      !finitePathState(left_state) || !finitePathState(right_state)) {
    reason = "segment_seam_point_query_failed";
    return false;
  }
  const double p_jump = (left_state.p - right_state.p).norm();
  const double dp_jump = (left_state.dp_dw - right_state.dp_dw).norm();
  const double d2p_jump =
      (left_state.d2p_dw2 - right_state.d2p_dw2).norm();
  const double total = p_jump + dp_jump + d2p_jump;
  if (!finite(p_jump) || !finite(dp_jump) || !finite(d2p_jump) ||
      !finite(total) || total > 1e-6) {
    reason = "segment_seam_c2_discontinuity";
    return false;
  }
  return true;
}

bool orderedBox(const plan_env::LocalObstacleBox& box) {
  return finite(box.min) && finite(box.max) &&
      (box.min.array() <= box.max.array()).all();
}

bool safeUpperSum(const double lhs, const double rhs, double& result) {
  result = std::numeric_limits<double>::quiet_NaN();
  if (!finite(lhs) || !finite(rhs) || lhs < 0.0 || rhs < 0.0) return false;
  if (lhs == 0.0) {
    result = rhs;
    return true;
  }
  if (rhs == 0.0) {
    result = lhs;
    return true;
  }
  const double rounded = lhs + rhs;
  if (!finite(rounded)) return false;
  result = std::nextafter(rounded, std::numeric_limits<double>::infinity());
  return finite(result);
}

bool makeNormalSecondDerivativeBound(const double q,
                                     const double a_xy,
                                     const double jerk,
                                     double& bound) {
  bound = std::numeric_limits<double>::quiet_NaN();
  if (!finite(q) || q <= phase_offset_core::kHorizontalNormalSpeedEpsilon ||
      !finite(a_xy) || a_xy < 0.0 || !finite(jerk) || jerk < 0.0) {
    return false;
  }
  // A zero horizontal acceleration bound proves that the horizontal path
  // derivative is constant on the complete cell.  Its normal is therefore
  // constant even when the full 3-D jerk is nonzero.
  if (a_xy == 0.0) {
    bound = 0.0;
    return true;
  }
  double jerk_over_q = 0.0;
  if (!phase_offset_core::outwardUpperRatio(jerk, q, jerk_over_q)) {
    return false;
  }
  double acceleration_over_q = 0.0;
  if (!phase_offset_core::outwardUpperRatio(a_xy, q,
                                             acceleration_over_q)) {
    return false;
  }
  double square = 0.0;
  if (!phase_offset_core::outwardUpperProduct(acceleration_over_q,
                                               acceleration_over_q,
                                               square)) {
    return false;
  }
  double quadratic = 0.0;
  if (!phase_offset_core::outwardUpperProduct(3.0, square, quadratic) ||
      !safeUpperSum(jerk_over_q, quadratic, bound)) {
    return false;
  }
  return finite(bound);
}

void fail(SectionBuildInput& output, std::string& reason,
          std::string message) {
  output = SectionBuildInput();
  reason = std::move(message);
}

}  // namespace

bool makeSectionBuildInput(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    const double w_start,
    const double w_end,
    const plan_env::LocalObstacleView& view,
    SectionBuildInput& output,
    std::string& reason) {
  output = SectionBuildInput();
  reason.clear();

  if (view.status != plan_env::LocalObstacleViewStatus::VALID) {
    fail(output, reason, "local_obstacle_view_not_valid");
    return false;
  }
  if (view.frame_id.empty() || !finite(view.resolution) ||
      view.resolution <= 0.0 || !orderedBox(view.obstacle_region) ||
      !orderedBox(view.reference_domain)) {
    fail(output, reason, "invalid_local_obstacle_view_geometry");
    return false;
  }
  if (view.occupied_voxels != view.obstacles.size()) {
    fail(output, reason, "occupied_voxel_count_mismatch");
    return false;
  }
  for (const plan_env::LocalObstacleVoxel& obstacle : view.obstacles) {
    if (!orderedBox(obstacle.bounds)) {
      fail(output, reason, "invalid_local_obstacle_box");
      return false;
    }
  }

  if (!path || path->empty()) {
    fail(output, reason, "path_empty");
    return false;
  }
  if (!finite(w_start) || !finite(w_end) || !(w_end > w_start)) {
    fail(output, reason, "invalid_phase_range");
    return false;
  }

  std::vector<const Segment*> relevant_segments;
  if (!collectRelevantSegments(*path, w_start, w_end, relevant_segments,
                               reason)) {
    fail(output, reason, reason.c_str());
    return false;
  }
  for (std::size_t index = 1U; index < relevant_segments.size(); ++index) {
    if (!seamC2Compatible(*relevant_segments[index - 1U],
                          *relevant_segments[index], reason)) {
      fail(output, reason, reason.c_str());
      return false;
    }
  }

  SectionEnvironment environment;
  environment.available = true;
  environment.reference_domain.min = view.reference_domain.min;
  environment.reference_domain.max = view.reference_domain.max;
  environment.obstacle_region.min = view.obstacle_region.min;
  environment.obstacle_region.max = view.obstacle_region.max;
  environment.obstacles.reserve(view.obstacles.size());
  for (const plan_env::LocalObstacleVoxel& obstacle : view.obstacles) {
    SectionBox box;
    box.min = obstacle.bounds.min;
    box.max = obstacle.bounds.max;
    environment.obstacles.push_back(box);
  }

  output.w_start = w_start;
  output.w_end = w_end;
  output.environment = std::move(environment);
  for (const Segment* segment : relevant_segments) {
    output.structural_breakpoints.push_back(segment->w0);
    output.structural_breakpoints.push_back(segment->w1);
  }
  std::sort(output.structural_breakpoints.begin(),
            output.structural_breakpoints.end());
  output.structural_breakpoints.erase(
      std::remove_if(output.structural_breakpoints.begin(),
                     output.structural_breakpoints.end(),
                     [w_start, w_end](const double value) {
                       return value < w_start || value > w_end;
                     }),
      output.structural_breakpoints.end());
  output.structural_breakpoints.erase(
      std::unique(output.structural_breakpoints.begin(),
                  output.structural_breakpoints.end()),
      output.structural_breakpoints.end());

  const std::shared_ptr<const ContinuousPhasePath> captured_path = path;
  const std::shared_ptr<const ContinuousPhaseNormalFrame> frame(
      new ContinuousPhaseNormalFrame(captured_path,
                                     captured_path->pathRevision(),
                                     captured_path->pathRevision()));

  output.point_query = [captured_path, frame, w_start, w_end](
      const double w, SectionPathSample& sample) {
    sample = SectionPathSample();
    if (!finite(w) || w < w_start || w > w_end) return false;
    ContinuousPhasePathState state;
    if (!captured_path->evaluate(w, state, false) || !state.valid) return false;
    phase_offset_core::NormalFrameQuery normal;
    if (!frame->query(w, normal) || !normal.valid) return false;
    SectionPathSample candidate;
    candidate.p = state.p;
    candidate.p_w = state.dp_dw;
    candidate.p_ww = state.d2p_dw2;
    candidate.N = normal.N;
    candidate.N_w = normal.N_w;
    if (!finite(candidate.p) || !finite(candidate.p_w) ||
        !finite(candidate.p_ww) || !finite(candidate.N) ||
        !finite(candidate.N_w)) {
      return false;
    }
    sample = candidate;
    return true;
  };

  output.bounds_query = [captured_path, w_start, w_end](
      const double cell_w0, const double cell_w1,
      SectionCellBounds& bounds) {
    bounds = SectionCellBounds();
    if (!finite(cell_w0) || !finite(cell_w1) || !(cell_w1 > cell_w0) ||
        cell_w0 < w_start || cell_w1 > w_end) {
      return false;
    }
    const Segment* segment = findSegmentForCell(
        *captured_path, cell_w0, cell_w1);
    if (segment == nullptr) {
      return false;
    }
    phase_offset_core::PathCellGeometryCertificate certificate;
    if (!captured_path->cellBounds(cell_w0, cell_w1, certificate)) {
      return false;
    }
    const double q = certificate.inf_horizontal_p_w_norm;
    const double v = certificate.inf_p_w_norm;
    const double a_xy = certificate.sup_horizontal_p_ww_norm;
    const double a = certificate.sup_p_ww_norm;
    const double jerk = certificate.sup_p_www_norm;
    const Eigen::Vector3d norm_component_bound(a_xy, a_xy, a);
    Eigen::Vector3d component_bound = norm_component_bound;
    if (certificate.component_acceleration_bound_complete) {
      if (!finite(certificate.sup_abs_p_ww) ||
          (certificate.sup_abs_p_ww.array() < 0.0).any()) {
        return false;
      }
      // Both vectors are whole-cell upper bounds.  Taking their componentwise
      // minimum preserves soundness while recovering tight zero coordinates.
      component_bound = certificate.sup_abs_p_ww.cwiseMin(
          norm_component_bound);
    }
    double normal_second_derivative = 0.0;
    if (!makeNormalSecondDerivativeBound(q, a_xy, jerk,
                                         normal_second_derivative)) {
      return false;
    }
    SectionCellBounds candidate;
    candidate.horizontal_speed_lower = q;
    candidate.path_speed_lower = v;
    candidate.abs_p_ww = component_bound;
    candidate.abs_N_ww = Eigen::Vector3d(normal_second_derivative,
                                         normal_second_derivative, 0.0);
    candidate.valid = finite(candidate.horizontal_speed_lower) &&
        finite(candidate.path_speed_lower) && finite(candidate.abs_p_ww) &&
        finite(candidate.abs_N_ww) && candidate.horizontal_speed_lower >
            phase_offset_core::kHorizontalNormalSpeedEpsilon &&
        candidate.path_speed_lower > 0.0 &&
        (candidate.abs_p_ww.array() >= 0.0).all() &&
        (candidate.abs_N_ww.array() >= 0.0).all();
    if (!candidate.valid) return false;
    bounds = candidate;
    return true;
  };

  // A multi-segment path uses a cell-aware callback so a seam endpoint is
  // evaluated by the same immutable segment as its bounds certificate.  The
  // ordinary point callback above remains the execution/compatibility query
  // and intentionally keeps ContinuousPhasePath's left-at-seam semantics.
  if (relevant_segments.size() > 1U) {
    output.cell_point_query = [captured_path, w_start, w_end](
        const double cell_w0, const double cell_w1, const double w,
        SectionPathSample& sample) {
      sample = SectionPathSample();
      if (!finite(cell_w0) || !finite(cell_w1) || !(cell_w1 > cell_w0) ||
          !finite(w) || cell_w0 < w_start || cell_w1 > w_end ||
          w < cell_w0 || w > cell_w1) {
        return false;
      }
      const Segment* segment = findSegmentForCell(
          *captured_path, cell_w0, cell_w1);
      if (segment == nullptr) return false;
      ContinuousPhasePathState state;
      if (!segment->evaluate(w, state) || !state.valid ||
          !finitePathState(state)) {
        return false;
      }
      Eigen::Vector3d tangent;
      Eigen::Vector3d tangent_w;
      Eigen::Vector3d normal;
      Eigen::Vector3d normal_w;
      if (!ContinuousPhaseNormalFrame::computeGeometry(
              state, tangent, tangent_w, normal, normal_w)) {
        return false;
      }
      SectionPathSample candidate;
      candidate.p = state.p;
      candidate.p_w = state.dp_dw;
      candidate.p_ww = state.d2p_dw2;
      candidate.N = normal;
      candidate.N_w = normal_w;
      if (!finite(candidate.p) || !finite(candidate.p_w) ||
          !finite(candidate.p_ww) || !finite(candidate.N) ||
          !finite(candidate.N_w)) {
        return false;
      }
      sample = candidate;
      return true;
    };
  }

  return true;
}

bool makeSectionReferenceRegion(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    const double w_start,
    const double w_end,
    const double half_width,
    const double max_step_w,
    const std::size_t max_cells,
    plan_env::LocalObstacleBox& region,
    std::string& reason) {
  region = plan_env::LocalObstacleBox();
  reason.clear();
  if (!path || path->empty() || !finite(w_start) || !finite(w_end) ||
      !(w_end > w_start) || !finite(half_width) || half_width < 0.0 ||
      !finite(max_step_w) || max_step_w <= 0.0 || max_cells == 0U ||
      w_start < path->startW() || w_end > path->endW()) {
    reason = "invalid_reference_region_request";
    return false;
  }

  std::vector<double> breakpoints;
  breakpoints.push_back(w_start);
  breakpoints.push_back(w_end);
  for (const Segment& segment : path->segments()) {
    if (segment.w0 > w_start && segment.w0 < w_end) {
      breakpoints.push_back(segment.w0);
    }
    if (segment.w1 > w_start && segment.w1 < w_end) {
      breakpoints.push_back(segment.w1);
    }
  }
  std::sort(breakpoints.begin(), breakpoints.end());
  breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()),
                    breakpoints.end());
  if (breakpoints.size() < 2U) {
    reason = "reference_region_has_no_cells";
    return false;
  }

  Eigen::Vector3d lower =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d upper =
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  std::size_t visited_cells = 0U;
  for (std::size_t i = 0U; i + 1U < breakpoints.size(); ++i) {
    const double interval_w0 = breakpoints[i];
    const double interval_w1 = breakpoints[i + 1U];
    if (!(interval_w1 > interval_w0)) continue;
    const double interval_span = interval_w1 - interval_w0;
    const double ratio = interval_span / max_step_w;
    if (!finite(interval_span) || !(interval_span > 0.0) ||
        !finite(ratio) || !(ratio > 0.0)) {
      reason = "reference_region_cell_budget_exceeded";
      return false;
    }
    // A structural interval shorter than max_step_w still needs one cell.
    // Keep the count in a checked floating representation until every range
    // and budget condition has passed; converting an unchecked huge ceil to
    // size_t is implementation-defined (and can wrap to zero).
    const double count_ceil = std::max(1.0, std::ceil(ratio));
    const std::size_t remaining_cells = max_cells - visited_cells;
    const long double count_value = static_cast<long double>(count_ceil);
    const long double remaining_value =
        static_cast<long double>(remaining_cells);
    const long double size_max_value = static_cast<long double>(
        std::numeric_limits<std::size_t>::max());
    if (!std::isfinite(count_value) || count_value < 1.0L ||
        // Reject the rounded representation of size_t::max as well.  On
        // platforms where long double aliases double, that value is 2^N and
        // cannot be converted to size_t without overflow.
        !(count_value < size_max_value) || count_value > remaining_value) {
      reason = "reference_region_cell_budget_exceeded";
      return false;
    }
    const std::size_t count = static_cast<std::size_t>(count_value);
    if (count == 0U || count > remaining_cells) {
      reason = "reference_region_cell_budget_exceeded";
      return false;
    }
    double cell_w0 = interval_w0;
    for (std::size_t cell = 0U; cell < count; ++cell) {
      // Reuse the previous endpoint for the next cell so adjacent cells have
      // an exactly shared boundary.  The final endpoint is assigned directly
      // from the structural interval to avoid a rounded value crossing a
      // segment seam and becoming ineligible for findSegmentForCell().
      double cell_w1 = interval_w1;
      if (cell + 1U < count) {
        const double fraction = static_cast<double>(cell + 1U) /
            static_cast<double>(count);
        cell_w1 = interval_w0 + interval_span * fraction;
      }
      if (!finite(cell_w0) || !finite(cell_w1) || !(cell_w1 > cell_w0)) {
        reason = "reference_region_cell_domain_nonfinite";
        return false;
      }
      const Segment* segment = findSegmentForCell(*path, cell_w0, cell_w1);
      if (segment == nullptr) {
        reason = "reference_region_cell_owner_unavailable";
        return false;
      }
      ContinuousPhasePathState left_state;
      ContinuousPhasePathState right_state;
      if (!segment->evaluate(cell_w0, left_state) ||
          !segment->evaluate(cell_w1, right_state) ||
          !left_state.valid || !right_state.valid ||
          !finitePathState(left_state) || !finitePathState(right_state)) {
        reason = "reference_region_endpoint_query_failed";
        return false;
      }
      phase_offset_core::PathCellGeometryCertificate certificate;
      if (!path->cellBounds(cell_w0, cell_w1, certificate) ||
          !finite(certificate.sup_p_ww_norm) ||
          certificate.sup_p_ww_norm < 0.0) {
        reason = "reference_region_cell_bound_unavailable";
        return false;
      }
      Eigen::Vector3d acceleration = Eigen::Vector3d::Constant(
          certificate.sup_p_ww_norm);
      if (certificate.component_acceleration_bound_complete) {
        if (!finite(certificate.sup_abs_p_ww) ||
            (certificate.sup_abs_p_ww.array() < 0.0).any()) {
          reason = "reference_region_component_bound_invalid";
          return false;
        }
        acceleration = acceleration.cwiseMin(certificate.sup_abs_p_ww);
      }
      const double h = cell_w1 - cell_w0;
      if (!finite(h) || !(h > 0.0)) {
        reason = "reference_region_cell_variation_nonfinite";
        return false;
      }
      double h2_upper = 0.0;
      if (!phase_offset_core::outwardUpperProduct(h, h, h2_upper) ||
          !finite(h2_upper)) {
        reason = "reference_region_cell_variation_nonfinite";
        return false;
      }
      Eigen::Vector3d cell_lower = left_state.p.cwiseMin(right_state.p);
      Eigen::Vector3d cell_upper = left_state.p.cwiseMax(right_state.p);
      for (int axis = 0; axis < 3; ++axis) {
        // Preserve exact zero component bounds.  Applying nextafter(0,+inf)
        // would fabricate a one-denormal-thick Z slab for a planar path.
        if (acceleration(axis) == 0.0) continue;
        double product_upper = 0.0;
        double axis_deviation = 0.0;
        if (!phase_offset_core::outwardUpperProduct(
                acceleration(axis), h2_upper, product_upper) ||
            !phase_offset_core::outwardUpperRatio(product_upper, 8.0,
                                                  axis_deviation) ||
            !finite(product_upper) || !finite(axis_deviation) ||
            axis_deviation < 0.0) {
          reason = "reference_region_axis_variation_nonfinite";
          return false;
        }
        const double raw_lower = cell_lower(axis) - axis_deviation;
        const double raw_upper = cell_upper(axis) + axis_deviation;
        cell_lower(axis) = std::nextafter(
            raw_lower, -std::numeric_limits<double>::infinity());
        cell_upper(axis) = std::nextafter(
            raw_upper, std::numeric_limits<double>::infinity());
        if (!finite(cell_lower(axis)) || !finite(cell_upper(axis))) {
          reason = "reference_region_cell_bounds_nonfinite";
          return false;
        }
      }
      lower = lower.cwiseMin(cell_lower);
      upper = upper.cwiseMax(cell_upper);
      ++visited_cells;
      cell_w0 = cell_w1;
    }
  }
  if (!finite(lower) || !finite(upper) ||
      (upper.array() < lower.array()).any()) {
    reason = "reference_region_bounds_nonfinite";
    return false;
  }
  if (half_width > 0.0) {
    lower.x() = std::nextafter(
        lower.x() - half_width, -std::numeric_limits<double>::infinity());
    lower.y() = std::nextafter(
        lower.y() - half_width, -std::numeric_limits<double>::infinity());
    upper.x() = std::nextafter(
        upper.x() + half_width, std::numeric_limits<double>::infinity());
    upper.y() = std::nextafter(
        upper.y() + half_width, std::numeric_limits<double>::infinity());
  }
  if (!finite(lower) || !finite(upper) ||
      (upper.array() < lower.array()).any()) {
    reason = "reference_region_expansion_nonfinite";
    return false;
  }
  region.min = lower;
  region.max = upper;
  return true;
}

namespace {

bool finiteBox(const plan_env::LocalObstacleBox& box) {
  return box.min.allFinite() && box.max.allFinite() &&
      (box.max.array() >= box.min.array()).all();
}

bool closedContains(const plan_env::LocalObstacleBox& box,
                    const Eigen::Vector3d& point) {
  return (point.array() >= box.min.array()).all() &&
      (point.array() <= box.max.array()).all();
}

}  // namespace

bool SectionScanEnvironment::build(
    const plan_env::LocalObstacleView& view, std::string& reason) {
  valid_ = false;
  occupied_.clear();
  reason.clear();
  if (view.status != plan_env::LocalObstacleViewStatus::VALID) {
    reason = "scan_environment_view_invalid";
    return false;
  }
  if (!std::isfinite(view.resolution) || view.resolution <= 0.0 ||
      !view.origin.allFinite() ||
      (view.voxel_count.array() <= 0).any() ||
      !finiteBox(view.reference_domain)) {
    reason = "scan_environment_geometry_invalid";
    return false;
  }
  const std::int64_t nx = view.voxel_count.x();
  const std::int64_t ny = view.voxel_count.y();
  if (nx > 0 && ny > (std::numeric_limits<std::int64_t>::max() / nx)) {
    reason = "scan_environment_stride_overflow";
    return false;
  }
  const std::int64_t stride_y = nx;
  const std::int64_t stride_z = nx * ny;
  occupied_.reserve(view.obstacles.size() * 2U + 1U);
  for (const plan_env::LocalObstacleVoxel& voxel : view.obstacles) {
    const Eigen::Vector3i& index = voxel.index;
    if ((index.array() < 0).any() ||
        (index.array() >= view.voxel_count.array()).any()) {
      reason = "scan_environment_voxel_out_of_grid";
      occupied_.clear();
      return false;
    }
    if (voxel.layer_mask == 0U) continue;
    const std::int64_t key = static_cast<std::int64_t>(index.x()) +
        static_cast<std::int64_t>(index.y()) * stride_y +
        static_cast<std::int64_t>(index.z()) * stride_z;
    occupied_.insert(key);
  }
  origin_ = view.origin;
  voxel_count_ = view.voxel_count;
  resolution_ = view.resolution;
  known_ = view.reference_domain;
  valid_ = true;
  return true;
}

bool SectionScanEnvironment::query(const Eigen::Vector3d& point,
                                   bool& occupied) const {
  occupied = true;
  if (!valid_ || !point.allFinite()) return false;
  if (!closedContains(known_, point)) return false;
  const Eigen::Vector3d local = (point - origin_) / resolution_;
  if (!local.allFinite()) return false;
  const int ix = static_cast<int>(std::floor(local.x()));
  const int iy = static_cast<int>(std::floor(local.y()));
  const int iz = static_cast<int>(std::floor(local.z()));
  if (ix < 0 || iy < 0 || iz < 0 || ix >= voxel_count_.x() ||
      iy >= voxel_count_.y() || iz >= voxel_count_.z()) {
    return false;
  }
  const std::int64_t nx = voxel_count_.x();
  const std::int64_t ny = voxel_count_.y();
  const std::int64_t key = static_cast<std::int64_t>(ix) +
      static_cast<std::int64_t>(iy) * nx + static_cast<std::int64_t>(iz) * nx * ny;
  occupied = occupied_.find(key) != occupied_.end();
  return true;
}

phase_offset_navigation::SectionTubeProfile buildSectionTubeByScan(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    const double w_start,
    const double w_end,
    const SectionScanEnvironment& environment,
    const SectionScanConfig& config) {
  using phase_offset_navigation::SectionTubeKnot;
  using phase_offset_navigation::SectionTubeProfile;
  using phase_offset_navigation::SectionTubeStatus;

  SectionTubeProfile profile;
  profile.valid_start = w_start;
  profile.valid_end = w_start;
  const auto fail = [&profile, w_start](const SectionTubeStatus status,
                                        const char* reason,
                                        const double failed_w) {
    profile.status = status;
    profile.usable = profile.knots.size() >= 2U;
    profile.complete = false;
    profile.valid_end = profile.knots.empty() ? w_start
                                             : profile.knots.back().w;
    profile.first_failure_w = failed_w;
    profile.failure_reason = reason;
    return profile;
  };
  if (!path || path->empty() || path->pathRevision() == 0U) {
    profile.status = SectionTubeStatus::INVALID_INPUT;
    profile.usable = false;
    profile.complete = false;
    profile.failure_reason = "scan_path_invalid";
    return profile;
  }
  if (!std::isfinite(w_start) || !std::isfinite(w_end) ||
      !(w_end > w_start) || w_start < path->startW() ||
      w_end > path->endW()) {
    profile.status = SectionTubeStatus::INVALID_INPUT;
    profile.usable = false;
    profile.complete = false;
    profile.failure_reason = "scan_interval_invalid";
    return profile;
  }
  if (!std::isfinite(config.sample_step_w) || config.sample_step_w <= 0.0 ||
      !std::isfinite(config.half_width) || config.half_width < 0.0 ||
      !std::isfinite(config.clearance) || config.clearance < 0.0 ||
      !std::isfinite(config.minimum_reference_speed) ||
      config.minimum_reference_speed <= 0.0 ||
      !std::isfinite(config.min_regularity_ratio) ||
      !std::isfinite(config.curvature_epsilon) ||
      config.curvature_epsilon <= 0.0 || config.max_scan_checks == 0U ||
      config.max_knots < 2U) {
    profile.status = SectionTubeStatus::INVALID_INPUT;
    profile.usable = false;
    profile.complete = false;
    profile.failure_reason = "scan_config_invalid";
    return profile;
  }

  // Scan step: one native voxel is the natural granularity, and never finer
  // than a twentieth of the requested half width so the work stays bounded.
  double scan_step = environment.resolution() > 0.0
      ? environment.resolution() : config.sample_step_w;
  if (config.half_width > 0.0) {
    scan_step = std::max(scan_step, config.half_width / 64.0);
  }
  if (!std::isfinite(scan_step) || scan_step <= 0.0) {
    profile.status = SectionTubeStatus::INVALID_INPUT;
    profile.usable = false;
    profile.failure_reason = "scan_step_invalid";
    return profile;
  }

  const ContinuousPhaseNormalFrame frame(
      path, path->pathRevision(), path->pathRevision());
  profile.status = SectionTubeStatus::COMPLETE;
  profile.usable = false;
  profile.complete = true;
  std::size_t checks = 0U;
  double w = w_start;
  while (true) {
    if (profile.knots.size() >= config.max_knots) {
      return fail(SectionTubeStatus::BUDGET_EXCEEDED, "scan_knot_budget", w);
    }
    ContinuousPhasePathState state;
    if (!frame.evaluatePathState(w, state) || !state.valid ||
        !state.frame_valid || !state.p.allFinite() || !state.N.allFinite()) {
      return fail(SectionTubeStatus::FRAME_DEGENERATE,
                  "scan_frame_unavailable", w);
    }
    if (state.dp_dw.allFinite() &&
        state.dp_dw.norm() < config.minimum_reference_speed) {
      return fail(SectionTubeStatus::GEOMETRY_UNAVAILABLE,
                  "scan_reference_speed_too_low", w);
    }
    // The corridor follows the path.  While the path sample itself is inside
    // the known domain a cross-section always exists; the known boundary may
    // only clip how far the interval extends.  Leaving the known domain with
    // the path itself is the genuine end of the usable prefix.
    {
      bool path_point_occupied = true;
      if (!environment.query(state.p, path_point_occupied)) {
        // The view's claimable domain is inset by the clearance, so the very
        // last requested sample can land exactly on that inset face and fall
        // an ULP outside it.  That tip is the path's own terminal region: the
        // corridor simply ends there.  Keep the prefix built so far instead of
        // reporting a partial corridor for a sub-voxel remainder.
        if (profile.knots.size() >= 2U &&
            w >= w_end - config.sample_step_w - 1e-9) {
          break;
        }
        return fail(SectionTubeStatus::UNKNOWN_DOMAIN,
                    "scan_path_left_known_domain", w);
      }
    }

    double lower_limit = 0.0;
    double upper_limit = 0.0;
    bool unknown = false;
    // The configured half width is the transverse offset the tube is allowed
    // to claim.  The scan therefore reaches one clearance beyond it, so that
    // an unobstructed sample reports exactly the configured half width after
    // the clearance is taken off the free distance.
    const double scan_reach = config.half_width + config.clearance;
    // M4F: the cross-section is also a set of offsets whose active reference
    // stays regular.  The obstacle scan below cannot see that: on a curved
    // path the parallel offset collapses as delta approaches the local
    // curvature radius, which is what made the phase rate blow up right after
    // a C2 connector.  Cap the collapsing side before the obstacle limit is
    // applied.
    double curvature_cap_negative = std::numeric_limits<double>::infinity();
    double curvature_cap_positive = std::numeric_limits<double>::infinity();
    sectionCurvatureOffsetCaps(state.dp_dw, state.d2p_dw2,
                               config.min_regularity_ratio,
                               config.curvature_epsilon,
                               curvature_cap_negative,
                               curvature_cap_positive);
    for (int side = -1; side <= 1; side += 2) {
      double limit = 0.0;
      for (double offset = scan_step; offset <= scan_reach + 1e-12;
           offset += scan_step) {
        if (checks >= config.max_scan_checks) {
          return fail(SectionTubeStatus::BUDGET_EXCEEDED, "scan_check_budget",
                      w);
        }
        ++checks;
        const Eigen::Vector3d probe =
            state.p + state.N * (static_cast<double>(side) * offset);
        bool occupied = true;
        if (!environment.query(probe, occupied)) {
          unknown = true;
          break;
        }
        if (occupied) break;
        limit = offset;
      }
      // Stop the claimed free interval one clearance short of the first
      // occupied voxel, then cap it at the configured half width.  The
      // inflation stencil inside the map's inflated layer is already
      // accounted for by that layer itself.
      limit = std::max(0.0, limit - config.clearance);
      limit = std::min(limit, config.half_width);
      limit = std::min(limit, side < 0 ? curvature_cap_negative
                                       : curvature_cap_positive);
      if (side < 0) {
        lower_limit = limit;
      } else {
        upper_limit = limit;
      }
    }
    profile.visited_cells = profile.knots.size();
    profile.obstacle_checks = checks;
    // Both sides are stored as non-negative magnitudes, so the signed interval
    // [-lower_limit, +upper_limit] always contains the path point.  An
    // asymmetric sample (one side blocked, the other open) is a valid, useful
    // cross-section and must not be rejected as if it were empty.
    if (!std::isfinite(lower_limit) || !std::isfinite(upper_limit) ||
        lower_limit < 0.0 || upper_limit < 0.0) {
      return fail(SectionTubeStatus::GEOMETRY_UNAVAILABLE,
                  "scan_interval_unavailable", w);
    }

    SectionTubeKnot knot;
    knot.w = w;
    knot.lower = -lower_limit;
    knot.upper = upper_limit;
    profile.knots.push_back(knot);
    profile.valid_end = w;
    // `unknown` only means this sample's interval is clipped by the known
    // boundary; the prefix continues along the path.
    (void)unknown;
    if (w >= w_end - 1e-12) break;
    w = std::min(w_end, w + config.sample_step_w);
    if (!(w > profile.knots.back().w)) break;
  }

  profile.status = SectionTubeStatus::COMPLETE;
  profile.complete = true;
  profile.usable = profile.knots.size() >= 2U;
  profile.valid_end = profile.knots.empty() ? w_start : profile.knots.back().w;
  profile.visited_cells = profile.knots.size();
  profile.obstacle_checks = checks;
  return profile;
}

}  // namespace FLAG_Race
