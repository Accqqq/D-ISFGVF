#include "phase_offset_navigation/section_tube.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <sstream>
#include <tuple>
#include <utility>

namespace phase_offset_navigation {
namespace {

constexpr double kHorizontalSpeedEpsilon = 1e-8;
constexpr double kGeometryTolerance = 1e-8;
constexpr double kEight = 8.0;

bool finite(const double value) { return std::isfinite(value); }
bool finite(const Eigen::Vector3d& value) { return value.allFinite(); }

bool orderedBox(const SectionBox& box, const bool allow_degenerate) {
  if (!finite(box.min) || !finite(box.max)) return false;
  return allow_degenerate
      ? (box.min.array() <= box.max.array()).all()
      : (box.min.array() < box.max.array()).all();
}

bool safeSub(const double lhs, const double rhs, double& result) {
  result = lhs - rhs;
  return finite(result);
}

bool safeAdd(const double lhs, const double rhs, double& result) {
  result = lhs + rhs;
  return finite(result);
}

// Error-free rounded addition (for finite operands whose rounded sum is
// finite).  Comparing the rounded sum against a finite double bound is exact
// unless they are equal; in that case the residual gives the missing sign and
// prevents a sub-ulp positive clearance from disappearing.
bool twoSumFinite(const double lhs, const double rhs,
                  double& rounded, double& residual) {
  rounded = lhs + rhs;
  if (!finite(rounded)) return false;
  const double z = rounded - lhs;
  residual = (lhs - (rounded - z)) + (rhs - z);
  return finite(residual);
}

bool sumGreaterEqual(const double lhs, const double rhs,
                     const double bound) {
  if (!finite(lhs) || !finite(rhs) || !finite(bound)) return false;
  double rounded = 0.0;
  double residual = 0.0;
  if (!twoSumFinite(lhs, rhs, rounded, residual)) return false;
  if (rounded > bound) return true;
  if (rounded < bound) return false;
  return residual >= 0.0;
}

bool sumLessEqual(const double lhs, const double rhs,
                  const double bound) {
  if (!finite(lhs) || !finite(rhs) || !finite(bound)) return false;
  double rounded = 0.0;
  double residual = 0.0;
  if (!twoSumFinite(lhs, rhs, rounded, residual)) return false;
  if (rounded < bound) return true;
  if (rounded > bound) return false;
  return residual <= 0.0;
}

bool intervalContainsZero(const double lower, const double upper) {
  return finite(lower) && finite(upper) && lower <= upper &&
      lower <= 0.0 && upper >= 0.0;
}

bool vectorClose(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
  if (!finite(lhs) || !finite(rhs)) return false;
  const double scale = std::max(1.0, std::max(lhs.norm(), rhs.norm()));
  return (lhs - rhs).norm() <= kGeometryTolerance * scale;
}

bool validatePathSample(const SectionPathSample& sample,
                        std::string& reason) {
  reason.clear();
  if (!finite(sample.p) || !finite(sample.p_w) ||
      !finite(sample.p_ww) || !finite(sample.N) || !finite(sample.N_w)) {
    reason = "nonfinite_path_sample";
    return false;
  }
  const double horizontal_speed =
      std::hypot(sample.p_w.x(), sample.p_w.y());
  if (!finite(horizontal_speed) ||
      horizontal_speed <= kHorizontalSpeedEpsilon) {
    reason = "horizontal_normal_capability_degenerate";
    return false;
  }
  const Eigen::Vector3d expected_normal(
      -sample.p_w.y() / horizontal_speed,
      sample.p_w.x() / horizontal_speed, 0.0);
  const Eigen::Vector3d horizontal_acceleration(
      -sample.p_ww.y(), sample.p_ww.x(), 0.0);
  const Eigen::Vector3d expected_normal_w =
      (horizontal_acceleration - expected_normal *
       expected_normal.dot(horizontal_acceleration)) / horizontal_speed;
  if (!vectorClose(sample.N, expected_normal)) {
    reason = "normal_cross_product_mismatch";
    return false;
  }
  if (!vectorClose(sample.N_w, expected_normal_w)) {
    reason = "normal_cross_product_derivative_mismatch";
    return false;
  }
  if (std::abs(sample.N.norm() - 1.0) > kGeometryTolerance ||
      std::abs(sample.N.z()) > kGeometryTolerance ||
      std::abs(sample.N_w.z()) > kGeometryTolerance ||
      std::abs(sample.p_w.dot(sample.N)) > kGeometryTolerance *
          std::max(1.0, sample.p_w.norm()) ||
      std::abs(sample.N_w.dot(sample.N)) > kGeometryTolerance *
          std::max(1.0, sample.N_w.norm())) {
    reason = "normal_frame_orthogonality_mismatch";
    return false;
  }
  return true;
}

bool validateBounds(const SectionCellBounds& bounds, std::string& reason) {
  reason.clear();
  if (!bounds.valid || !finite(bounds.horizontal_speed_lower) ||
      !finite(bounds.path_speed_lower) ||
      !finite(bounds.abs_p_ww) || !finite(bounds.abs_N_ww) ||
      bounds.horizontal_speed_lower < 0.0 || bounds.path_speed_lower < 0.0 ||
      bounds.horizontal_speed_lower <= kHorizontalSpeedEpsilon ||
      (bounds.abs_p_ww.array() < 0.0).any() ||
      (bounds.abs_N_ww.array() < 0.0).any()) {
    reason = "invalid_cell_bounds";
    return false;
  }
  return true;
}

bool boundContainsSample(const SectionPathSample& sample,
                         const SectionCellBounds& bounds,
                         std::string& reason) {
  const double speed = sample.p_w.norm();
  const double horizontal_speed =
      std::hypot(sample.p_w.x(), sample.p_w.y());
  const double scale = std::max(1.0, std::max(speed, horizontal_speed));
  const double tolerance = kGeometryTolerance * scale;
  if (!finite(speed) || !finite(horizontal_speed) ||
      speed + tolerance < bounds.path_speed_lower ||
      horizontal_speed + tolerance < bounds.horizontal_speed_lower) {
    reason = "path_speed_lower_bound_violated_at_sample";
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    const double value = std::abs(sample.p_ww(axis));
    const double bound = bounds.abs_p_ww(axis);
    const double local_scale = std::max(1.0, std::max(value, bound));
    if (!finite(value) || !finite(bound) ||
        value > bound + kGeometryTolerance * local_scale) {
      reason = "p_ww_coordinate_bound_violated_at_sample";
      return false;
    }
  }
  return true;
}

struct Plane {
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
  double b = 0.0;
};

struct SeedSeparation {
  bool separated = false;
  Eigen::Vector3d s = Eigen::Vector3d::Zero();
  Eigen::Vector3d o = Eigen::Vector3d::Zero();
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
};

Eigen::Vector3d clampToBox(const Eigen::Vector3d& point,
                           const SectionBox& box) {
  return point.cwiseMax(box.min).cwiseMin(box.max);
}

bool considerSeedPoint(const Eigen::Vector3d& p0,
                       const Eigen::Vector3d& direction,
                       const SectionBox& box,
                       const double t,
                       double& best_distance_squared,
                       Eigen::Vector3d& best_s,
                       Eigen::Vector3d& best_o) {
  if (!finite(t) || t < 0.0 || t > 1.0) return true;
  const Eigen::Vector3d s = p0 + t * direction;
  if (!finite(s)) return false;
  const Eigen::Vector3d o = clampToBox(s, box);
  const Eigen::Vector3d difference = o - s;
  const double distance_squared = difference.squaredNorm();
  if (!finite(distance_squared)) return false;
  if (distance_squared < best_distance_squared) {
    best_distance_squared = distance_squared;
    best_s = s;
    best_o = o;
  }
  return true;
}

bool closestSeedPointToBox(const Eigen::Vector3d& p0,
                           const Eigen::Vector3d& p1,
                           const SectionBox& box,
                           SeedSeparation& result) {
  result = SeedSeparation();
  if (!finite(p0) || !finite(p1) || !orderedBox(box, true)) return false;
  const Eigen::Vector3d direction = p1 - p0;
  if (!finite(direction)) return false;

  std::vector<double> breakpoints;
  breakpoints.reserve(8U);
  breakpoints.push_back(0.0);
  breakpoints.push_back(1.0);
  for (int axis = 0; axis < 3; ++axis) {
    if (direction(axis) == 0.0) continue;
    const double t_min = (box.min(axis) - p0(axis)) / direction(axis);
    const double t_max = (box.max(axis) - p0(axis)) / direction(axis);
    if (finite(t_min) && t_min >= 0.0 && t_min <= 1.0) {
      breakpoints.push_back(t_min);
    }
    if (finite(t_max) && t_max >= 0.0 && t_max <= 1.0) {
      breakpoints.push_back(t_max);
    }
  }
  std::sort(breakpoints.begin(), breakpoints.end());
  breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()),
                    breakpoints.end());

  double best_distance_squared = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_s = Eigen::Vector3d::Zero();
  Eigen::Vector3d best_o = Eigen::Vector3d::Zero();
  if (!considerSeedPoint(p0, direction, box, 0.0, best_distance_squared,
                         best_s, best_o) ||
      !considerSeedPoint(p0, direction, box, 1.0, best_distance_squared,
                         best_s, best_o)) {
    return false;
  }

  for (std::size_t i = 0U; i + 1U < breakpoints.size(); ++i) {
    const double ta = breakpoints[i];
    const double tb = breakpoints[i + 1U];
    if (!(tb > ta)) continue;
    const double tm = ta + 0.5 * (tb - ta);
    const Eigen::Vector3d sm = p0 + tm * direction;
    if (!finite(sm)) return false;
    double quadratic_a = 0.0;
    double quadratic_b = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      double boundary = 0.0;
      bool active = false;
      if (sm(axis) < box.min(axis)) {
        boundary = box.min(axis);
        active = true;
      } else if (sm(axis) > box.max(axis)) {
        boundary = box.max(axis);
        active = true;
      }
      if (active) {
        const double constant = p0(axis) - boundary;
        quadratic_a += direction(axis) * direction(axis);
        quadratic_b += 2.0 * direction(axis) * constant;
      }
    }
    if (!finite(quadratic_a) || !finite(quadratic_b)) return false;
    double candidate = ta;
    if (quadratic_a > 0.0) {
      candidate = -quadratic_b / (2.0 * quadratic_a);
      if (!finite(candidate)) return false;
      candidate = std::max(ta, std::min(tb, candidate));
    }
    if (!considerSeedPoint(p0, direction, box, candidate,
                           best_distance_squared, best_s, best_o) ||
        !considerSeedPoint(p0, direction, box, ta,
                           best_distance_squared, best_s, best_o) ||
        !considerSeedPoint(p0, direction, box, tb,
                           best_distance_squared, best_s, best_o)) {
      return false;
    }
  }
  if (!finite(best_distance_squared) || !finite(best_s) ||
      !finite(best_o)) return false;
  if (best_distance_squared <= 0.0) {
    result.separated = false;
    return true;
  }
  const Eigen::Vector3d separation = best_o - best_s;
  const double distance = separation.norm();
  if (!finite(distance) || distance <= 0.0) return false;
  result.separated = true;
  result.s = best_s;
  result.o = best_o;
  result.a = separation / distance;
  return finite(result.a);
}

bool planeSupportMinimum(const Eigen::Vector3d& a,
                         const SectionBox& box,
                         double& result,
                         long double& absolute_term_scale) {
  long double support = 0.0L;
  long double absolute_terms = 0.0L;
  for (int axis = 0; axis < 3; ++axis) {
    const long double coefficient = static_cast<long double>(a(axis));
    const long double endpoint = static_cast<long double>(
        a(axis) >= 0.0 ? box.min(axis) : box.max(axis));
    const long double term = coefficient * endpoint;
    if (!std::isfinite(term)) return false;
    support += term;
    absolute_terms += std::abs(term);
    if (!std::isfinite(support) || !std::isfinite(absolute_terms)) {
      return false;
    }
  }
  if (support < -static_cast<long double>(std::numeric_limits<double>::max()) ||
      support > static_cast<long double>(std::numeric_limits<double>::max()) ||
      absolute_terms < 0.0L) {
    return false;
  }
  result = static_cast<double>(support);
  if (!finite(result)) return false;
  absolute_term_scale = absolute_terms;
  return true;
}

bool addPlaneForBox(const Eigen::Vector3d& a, const SectionBox& box,
                    const double clearance, Plane& plane) {
  if (!finite(a) || !orderedBox(box, true) || !finite(clearance) ||
      clearance < 0.0) return false;
  const double norm = a.norm();
  const double clearance_term = clearance * norm;
  double support = 0.0;
  long double absolute_term_scale = 0.0L;
  if (!finite(norm) || norm <= 0.0 || !finite(clearance_term) ||
      !planeSupportMinimum(a, box, support, absolute_term_scale)) {
    return false;
  }
  // A scale-relative inward repair protects the whole-box support inequality
  // from one rounded dot/product operation.  This is numerical conservatism,
  // not an additional physical clearance; domain planes intentionally have
  // no such margin.
  const long double scale = std::max(
      1.0L, absolute_term_scale +
          std::abs(static_cast<long double>(clearance_term)));
  const long double margin = 64.0L *
      static_cast<long double>(std::numeric_limits<double>::epsilon()) * scale;
  const long double plane_b = static_cast<long double>(support) -
      static_cast<long double>(clearance_term) - margin;
  if (!std::isfinite(scale) || !std::isfinite(margin) ||
      !std::isfinite(plane_b) ||
      plane_b < -static_cast<long double>(std::numeric_limits<double>::max()) ||
      plane_b > static_cast<long double>(std::numeric_limits<double>::max())) {
    return false;
  }
  double b = static_cast<double>(plane_b);
  if (!finite(b)) return false;
  // Casting can round an upper bound upward.  Keep the support half-space
  // closed and conservative after the scale-aware repair above.
  if (static_cast<long double>(b) > plane_b) {
    b = std::nextafter(b, -std::numeric_limits<double>::infinity());
  }
  if (!finite(b)) return false;
  plane.a = a;
  plane.b = b;
  return finite(plane.b);
}

bool addDomainPlanes(const SectionBox& domain,
                     std::vector<Plane>& planes) {
  if (!orderedBox(domain, true)) return false;
  for (int axis = 0; axis < 3; ++axis) {
    Plane lower;
    lower.a.setZero();
    lower.a(axis) = -1.0;
    lower.b = -domain.min(axis);
    Plane upper;
    upper.a.setZero();
    upper.a(axis) = 1.0;
    upper.b = domain.max(axis);
    if (!finite(lower.b) || !finite(upper.b)) return false;
    planes.push_back(lower);
    planes.push_back(upper);
  }
  return true;
}

bool inwardLowerFromLongDouble(const long double value, double& result) {
  if (!std::isfinite(value) ||
      value < -static_cast<long double>(std::numeric_limits<double>::max()) ||
      value > static_cast<long double>(std::numeric_limits<double>::max())) {
    return false;
  }
  result = static_cast<double>(value);
  if (!finite(result)) return false;
  if (static_cast<long double>(result) < value) {
    result = std::nextafter(result, std::numeric_limits<double>::infinity());
  }
  return finite(result);
}

bool inwardUpperFromLongDouble(const long double value, double& result) {
  if (!std::isfinite(value) ||
      value < -static_cast<long double>(std::numeric_limits<double>::max()) ||
      value > static_cast<long double>(std::numeric_limits<double>::max())) {
    return false;
  }
  result = static_cast<double>(value);
  if (!finite(result)) return false;
  if (static_cast<long double>(result) > value) {
    result = std::nextafter(result, -std::numeric_limits<double>::infinity());
  }
  return finite(result);
}

bool applyPlaneAtEndpoint(const Plane& plane,
                          const SectionPathSample& sample,
                          const SectionCellBounds& bounds,
                          const double w_span,
                          double& negative_low,
                          double& negative_high,
                          double& positive_low,
                          double& positive_high) {
  const double alpha = plane.a.dot(sample.N);
  const double beta = plane.b - plane.a.dot(sample.p);
  const double p_bound = plane.a.cwiseAbs().dot(bounds.abs_p_ww);
  const double n_bound = plane.a.cwiseAbs().dot(bounds.abs_N_ww);
  const double span_squared = w_span * w_span;
  const double c0 = p_bound * span_squared / kEight;
  const double c1 = n_bound * span_squared / kEight;
  if (!finite(alpha) || !finite(beta) || !finite(p_bound) ||
      !finite(n_bound) || !finite(span_squared) || !finite(c0) ||
      !finite(c1) || c0 < 0.0 || c1 < 0.0) return false;
  const long double rhs = static_cast<long double>(beta) -
      static_cast<long double>(c0);
  if (!std::isfinite(rhs) || rhs < 0.0L) return false;

  // The zero offset must satisfy this endpoint inequality.  Therefore a
  // negative-side coefficient can only constrain the lower endpoint, and a
  // positive-side coefficient can only constrain the upper endpoint.
  const long double negative_coefficient = static_cast<long double>(alpha) -
      static_cast<long double>(c1);
  if (!std::isfinite(negative_coefficient)) return false;
  if (negative_coefficient < 0.0L) {
    const long double root = rhs / negative_coefficient;
    double inward_root = 0.0;
    if (!std::isfinite(root) || !inwardLowerFromLongDouble(root,
                                                           inward_root)) {
      return false;
    }
    negative_low = std::max(negative_low, inward_root);
  }
  const long double positive_coefficient = static_cast<long double>(alpha) +
      static_cast<long double>(c1);
  if (!std::isfinite(positive_coefficient)) return false;
  if (positive_coefficient > 0.0L) {
    const long double root = rhs / positive_coefficient;
    double inward_root = 0.0;
    if (!std::isfinite(root) || !inwardUpperFromLongDouble(root,
                                                           inward_root)) {
      return false;
    }
    positive_high = std::min(positive_high, inward_root);
  }
  (void)negative_high;
  (void)positive_low;
  return true;
}

// Applies all endpoint inequalities with the explicit phase span; no
// path-derived quantity is inferred from positions.
bool computePlaneIntervalWithSpan(const std::vector<Plane>& planes,
                                  const SectionPathSample& start,
                                  const SectionPathSample& end,
                                  const SectionCellBounds& bounds,
                                  const double half_width,
                                  const double w_span,
                                  double& lower,
                                  double& upper,
                                  std::string& reason) {
  reason.clear();
  double negative_low = -half_width;
  double negative_high = 0.0;
  double positive_low = 0.0;
  double positive_high = half_width;
  for (const Plane& plane : planes) {
    const double p0_side = plane.a.dot(start.p);
    const double p1_side = plane.a.dot(end.p);
    if (!finite(p0_side) || !finite(p1_side) ||
        p0_side > plane.b || p1_side > plane.b) {
      reason = "seed_endpoint_outside_plane";
      return false;
    }
    auto applyEndpoint = [&](const SectionPathSample& sample) {
      return applyPlaneAtEndpoint(
          plane, sample, bounds, w_span,
          negative_low, negative_high, positive_low, positive_high);
    };
    if (!applyEndpoint(start) || !applyEndpoint(end)) {
      reason = "plane_interval_blocked";
      return false;
    }
  }
  if (negative_low > negative_high || positive_low > positive_high) {
    reason = "plane_interval_blocked";
    return false;
  }
  lower = negative_low;
  upper = positive_high;
  if (!intervalContainsZero(lower, upper)) {
    reason = "zero_offset_outside_plane_interval";
    return false;
  }
  return true;
}

bool quadraticAtZeroSafe(const Eigen::Vector3d& p,
                         const double threshold) {
  const double value = p.squaredNorm() - threshold * threshold;
  return finite(value) && value >= 0.0;
}

bool applyRegularity(const Eigen::Vector3d& p,
                     const Eigen::Vector3d& n,
                     const double threshold,
                     const double /*half_width*/,
                     double& lower,
                     double& upper,
                     bool& zero_safe) {
  zero_safe = quadraticAtZeroSafe(p, threshold);
  if (!zero_safe) return false;
  const long double a = static_cast<long double>(n.dot(n));
  const long double b = 2.0L * static_cast<long double>(p.dot(n));
  const long double c = static_cast<long double>(p.dot(p)) -
      static_cast<long double>(threshold) *
      static_cast<long double>(threshold);
  if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c) ||
      a < 0.0L) return false;

  double negative_low = lower;
  double negative_high = 0.0;
  double positive_low = 0.0;
  double positive_high = upper;
  if (a == 0.0L) {
    if (b > 0.0L) {
      long double root = -c / b;
      if (!std::isfinite(root)) return false;
      double rounded = 0.0;
      if (!inwardLowerFromLongDouble(root, rounded)) return false;
      negative_low = std::max(negative_low, rounded);
      positive_low = std::max(positive_low, rounded);
    } else if (b < 0.0L) {
      long double root = -c / b;
      if (!std::isfinite(root)) return false;
      double rounded = 0.0;
      if (!inwardUpperFromLongDouble(root, rounded)) return false;
      negative_high = std::min(negative_high, rounded);
      positive_high = std::min(positive_high, rounded);
    } else if (c < 0.0L) {
      return false;
    }
  } else {
    const long double discriminant = b * b - 4.0L * a * c;
    if (!std::isfinite(discriminant)) return false;
    if (discriminant > 0.0L) {
      const long double root_sqrt = std::sqrt(discriminant);
      const long double root1 = (-b - root_sqrt) / (2.0L * a);
      const long double root2 = (-b + root_sqrt) / (2.0L * a);
      if (!std::isfinite(root1) || !std::isfinite(root2)) return false;
      const long double r1 = std::min(root1, root2);
      const long double r2 = std::max(root1, root2);
      if (r1 < 0.0L && r2 > 0.0L) return false;
      if (r2 <= 0.0L) {
        double rounded = 0.0;
        if (!inwardLowerFromLongDouble(r2, rounded)) return false;
        negative_low = std::max(negative_low, rounded);
      } else if (r1 >= 0.0L) {
        double rounded = 0.0;
        if (!inwardUpperFromLongDouble(r1, rounded)) return false;
        positive_high = std::min(positive_high, rounded);
      }
    }
  }
  if (negative_low > negative_high || positive_low > positive_high) {
    return false;
  }
  lower = negative_low;
  upper = positive_high;
  return intervalContainsZero(lower, upper);
}

enum class CellFailure {
  NONE,
  BUDGET,
  GEOMETRY,
  FRAME,
  SEED,
  REGULARITY
};

struct CellResult {
  bool accepted = false;
  bool zero_only = false;
  // This is deliberately separate from accepted/zero_only.  It records only
  // the proof that the reference (zero-offset) path is safe on this complete
  // cell: all callbacks, bounds, seed/plane checks succeeded and the supplied
  // path-speed lower bound reaches the configured minimum.  A nonzero-width
  // cell may carry this proof while its regularity bound still needs finite
  // subdivision before we retain the zero-only fallback.
  bool zero_fallback_proven = false;
  bool geometric_zero = false;
  double lower = 0.0;
  double upper = 0.0;
  CellFailure failure = CellFailure::GEOMETRY;
  std::string reason;
};

struct AcceptedCell {
  double w0 = 0.0;
  double w1 = 0.0;
  double lower = 0.0;
  double upper = 0.0;
  bool zero_only = false;
};

class BuilderContext {
 public:
  BuilderContext(const SectionBuildInput& input,
                 const SectionBuildConfig& config,
                 SectionTubeProfile& profile)
      : input_(input), config_(config), profile_(profile) {}

  bool queryGlobalPoint(const double w, SectionPathSample& sample) {
    const auto found = global_point_cache_.find(w);
    if (found != global_point_cache_.end()) {
      sample = found->second.second;
      return found->second.first;
    }
    ++profile_.point_queries;
    const bool ok = input_.point_query(w, sample);
    global_point_cache_.emplace(w, std::make_pair(ok, sample));
    return ok;
  }

  bool queryPoint(const double w0, const double w1, const double w,
                  SectionPathSample& sample) {
    if (!input_.cell_point_query) return queryGlobalPoint(w, sample);
    const std::tuple<double, double, double> key(w0, w1, w);
    const auto found = cell_point_cache_.find(key);
    if (found != cell_point_cache_.end()) {
      sample = found->second.second;
      return found->second.first;
    }
    ++profile_.point_queries;
    const bool ok = input_.cell_point_query(w0, w1, w, sample);
    cell_point_cache_.emplace(key, std::make_pair(ok, sample));
    return ok;
  }

  bool queryBounds(const double w0, const double w1,
                   SectionCellBounds& bounds) {
    const std::pair<double, double> key(w0, w1);
    const auto found = bounds_cache_.find(key);
    if (found != bounds_cache_.end()) {
      bounds = found->second.second;
      return found->second.first;
    }
    ++profile_.bounds_queries;
    const bool ok = input_.bounds_query(w0, w1, bounds);
    bounds_cache_.emplace(key, std::make_pair(ok, bounds));
    return ok;
  }

  bool budgetCellsAvailable() const {
    return profile_.visited_cells < config_.max_cells;
  }

  bool budgetObstaclesAvailable() const {
    return profile_.obstacle_checks < config_.max_obstacle_checks;
  }

  void visitCell(const int depth) {
    ++profile_.visited_cells;
    profile_.max_depth_reached = std::max(profile_.max_depth_reached, depth);
  }

  bool consumeObstacleCheck() {
    if (!budgetObstaclesAvailable()) return false;
    ++profile_.obstacle_checks;
    return true;
  }

 private:
  const SectionBuildInput& input_;
  const SectionBuildConfig& config_;
  SectionTubeProfile& profile_;
  std::map<double, std::pair<bool, SectionPathSample>> global_point_cache_;
  std::map<std::tuple<double, double, double>,
           std::pair<bool, SectionPathSample>> cell_point_cache_;
  std::map<std::pair<double, double>,
           std::pair<bool, SectionCellBounds>> bounds_cache_;
};

SectionTubeStatus statusForFailure(const CellFailure failure) {
  switch (failure) {
    case CellFailure::BUDGET: return SectionTubeStatus::BUDGET_EXCEEDED;
    case CellFailure::FRAME: return SectionTubeStatus::FRAME_DEGENERATE;
    case CellFailure::SEED: return SectionTubeStatus::SEED_BLOCKED;
    case CellFailure::REGULARITY:
      return SectionTubeStatus::REGULARITY_LIMIT;
    case CellFailure::GEOMETRY:
      return SectionTubeStatus::GEOMETRY_UNAVAILABLE;
    case CellFailure::NONE: break;
  }
  return SectionTubeStatus::GEOMETRY_UNAVAILABLE;
}

bool samplesExactlyEqual(const SectionPathSample& lhs,
                         const SectionPathSample& rhs) {
  return (lhs.p.array() == rhs.p.array()).all() &&
      (lhs.p_w.array() == rhs.p_w.array()).all() &&
      (lhs.p_ww.array() == rhs.p_ww.array()).all() &&
      (lhs.N.array() == rhs.N.array()).all() &&
      (lhs.N_w.array() == rhs.N_w.array()).all();
}

// Computes the safe cross-section for one endpoint sample.  This deliberately
// uses zero span and zero second-derivative bounds: it proves only the
// endpoint geometry and instantaneous offset velocity, not a new complete
// cell.  The caller uses it only when the global execution sample differs
// from the cell-owned sample at a returned profile boundary.
bool computeSinglePointInterval(const SectionBuildInput& input,
                                const SectionBuildConfig& config,
                                BuilderContext& context,
                                const SectionPathSample& sample,
                                double& lower,
                                double& upper,
                                CellFailure& failure,
                                std::string& reason) {
  failure = CellFailure::GEOMETRY;
  reason.clear();
  if (!validatePathSample(sample, reason)) {
    failure = reason.find("normal_") == 0U ||
            reason.find("horizontal_") == 0U
        ? CellFailure::FRAME : CellFailure::GEOMETRY;
    return false;
  }

  std::vector<Plane> planes;
  planes.reserve(6U + input.environment.obstacles.size());
  if (!addDomainPlanes(input.environment.reference_domain, planes)) {
    reason = "invalid_reference_domain_planes";
    return false;
  }
  for (const SectionBox& obstacle : input.environment.obstacles) {
    if (!context.consumeObstacleCheck()) {
      failure = CellFailure::BUDGET;
      reason = "obstacle_check_budget_exceeded";
      return false;
    }
    if (!orderedBox(obstacle, true)) {
      reason = "invalid_obstacle_box";
      return false;
    }
    SeedSeparation separation;
    if (!closestSeedPointToBox(sample.p, sample.p, obstacle, separation)) {
      reason = "seed_distance_undetermined";
      return false;
    }
    if (!separation.separated) {
      failure = CellFailure::SEED;
      reason = "seed_intersects_obstacle";
      return false;
    }
    Plane plane;
    if (!addPlaneForBox(separation.a, obstacle, config.clearance, plane)) {
      failure = CellFailure::SEED;
      reason = "seed_separation_plane_undetermined";
      return false;
    }
    planes.push_back(plane);
  }

  SectionCellBounds point_bounds;
  point_bounds.abs_p_ww.setZero();
  point_bounds.abs_N_ww.setZero();
  point_bounds.valid = true;
  if (!computePlaneIntervalWithSpan(
          planes, sample, sample, point_bounds, config.half_width, 0.0,
          lower, upper, reason)) {
    failure = CellFailure::SEED;
    return false;
  }
  bool zero_regular = false;
  if (!applyRegularity(sample.p_w, sample.N_w,
                       config.minimum_reference_speed,
                       config.half_width, lower, upper, zero_regular) ||
      !intervalContainsZero(lower, upper)) {
    failure = CellFailure::REGULARITY;
    reason = "regularity_endpoint_bound_failed";
    return false;
  }
  return true;
}

CellResult evaluateCell(const SectionBuildInput& input,
                        const SectionBuildConfig& config,
                        BuilderContext& context,
                        const double w0,
                        const double w1,
                        const int depth) {
  CellResult result;
  if (!context.budgetCellsAvailable()) {
    result.failure = CellFailure::BUDGET;
    result.reason = "cell_budget_exceeded";
    return result;
  }
  const double span = w1 - w0;
  if (!finite(span) || span <= 0.0) {
    result.reason = "invalid_cell_span";
    return result;
  }
  context.visitCell(depth);
  SectionPathSample start;
  SectionPathSample end;
  SectionPathSample midpoint;
  if (!context.queryPoint(w0, w1, w0, start) ||
      !context.queryPoint(w0, w1, w1, end) ||
      !context.queryPoint(w0, w1, w0 + 0.5 * span, midpoint)) {
    result.failure = CellFailure::GEOMETRY;
    result.reason = "point_query_failed";
    return result;
  }
  std::string geometry_reason;
  if (!validatePathSample(start, geometry_reason) ||
      !validatePathSample(end, geometry_reason) ||
      !validatePathSample(midpoint, geometry_reason)) {
    result.failure = geometry_reason.find("normal_") == 0U ||
            geometry_reason.find("horizontal_") == 0U
        ? CellFailure::FRAME : CellFailure::GEOMETRY;
    result.reason = geometry_reason;
    return result;
  }
  SectionCellBounds bounds;
  if (!context.queryBounds(w0, w1, bounds) ||
      !validateBounds(bounds, geometry_reason)) {
    result.failure = CellFailure::GEOMETRY;
    result.reason = geometry_reason.empty() ? "bounds_query_failed"
                                            : geometry_reason;
    return result;
  }
  if (!boundContainsSample(start, bounds, geometry_reason) ||
      !boundContainsSample(end, bounds, geometry_reason) ||
      !boundContainsSample(midpoint, bounds, geometry_reason)) {
    result.failure = geometry_reason.find("speed") != std::string::npos
        ? CellFailure::FRAME : CellFailure::GEOMETRY;
    result.reason = geometry_reason;
    return result;
  }

  std::vector<Plane> planes;
  planes.reserve(6U + input.environment.obstacles.size());
  if (!addDomainPlanes(input.environment.reference_domain, planes)) {
    result.failure = CellFailure::GEOMETRY;
    result.reason = "invalid_reference_domain_planes";
    return result;
  }
  for (const SectionBox& obstacle : input.environment.obstacles) {
    if (!context.consumeObstacleCheck()) {
      result.failure = CellFailure::BUDGET;
      result.reason = "obstacle_check_budget_exceeded";
      return result;
    }
    if (!orderedBox(obstacle, true)) {
      result.failure = CellFailure::GEOMETRY;
      result.reason = "invalid_obstacle_box";
      return result;
    }
    SeedSeparation separation;
    if (!closestSeedPointToBox(start.p, end.p, obstacle, separation)) {
      result.failure = CellFailure::GEOMETRY;
      result.reason = "seed_distance_undetermined";
      return result;
    }
    if (!separation.separated) {
      result.failure = CellFailure::SEED;
      result.reason = "seed_intersects_obstacle";
      return result;
    }
    Plane plane;
    if (!addPlaneForBox(separation.a, obstacle, config.clearance, plane)) {
      result.failure = CellFailure::SEED;
      result.reason = "seed_separation_plane_undetermined";
      return result;
    }
    planes.push_back(plane);
  }

  double plane_lower = 0.0;
  double plane_upper = 0.0;
  if (!computePlaneIntervalWithSpan(
          planes, start, end, bounds, config.half_width, span,
          plane_lower, plane_upper, result.reason)) {
    result.failure = CellFailure::SEED;
    return result;
  }

  const bool explicitly_zero = config.half_width == 0.0;
  // At this point every geometric check for the complete cell has succeeded.
  // Keep the reference-path proof independent of the regularity calculation:
  // a conservative nonzero-offset bound may fail even though zero is known
  // safe from the supplied path-speed lower bound.
  result.zero_fallback_proven =
      finite(bounds.path_speed_lower) &&
      bounds.path_speed_lower >= config.minimum_reference_speed;
  result.geometric_zero = plane_lower == 0.0 && plane_upper == 0.0;

  // An explicit zero-width request, or a plane intersection that genuinely
  // leaves no transverse capacity, does not benefit from subdivision.  It is
  // accepted only when the independent zero-offset speed proof is present.
  if ((explicitly_zero || result.geometric_zero) &&
      result.zero_fallback_proven) {
    result.accepted = true;
    result.lower = 0.0;
    result.upper = 0.0;
    result.zero_only = true;
    result.failure = CellFailure::NONE;
    result.reason = explicitly_zero ? "zero_only_explicit_request"
                                    : "zero_only_geometric_zero";
    return result;
  }

  // If N_w is identically zero on the cell (the midpoint value is exactly
  // zero and its supplied second-derivative bound is zero), the regularity
  // expression is independent of delta.  The path-speed lower bound then
  // certifies the entire plane interval, even when p_ww makes the generic
  // anchor/Lipschitz variation bound too conservative.
  const bool normal_rate_identically_zero =
      (midpoint.N_w.array() == 0.0).all() &&
      (bounds.abs_N_ww.array() == 0.0).all();
  if (normal_rate_identically_zero && result.zero_fallback_proven) {
    result.accepted = true;
    result.lower = plane_lower;
    result.upper = plane_upper;
    result.zero_only = false;
    result.failure = CellFailure::NONE;
    result.reason = "regularity_normal_rate_identically_zero";
    return result;
  }

  double regularity_lower = plane_lower;
  double regularity_upper = plane_upper;
  const double D = std::max(std::abs(plane_lower), std::abs(plane_upper));
  const double A = bounds.abs_p_ww.norm();
  const double B2 = bounds.abs_N_ww.norm();
  const double variation = (A + D * B2) * span / 2.0;
  const double threshold = config.minimum_reference_speed + variation;
  if (!finite(D) || !finite(A) || !finite(B2) || !finite(variation) ||
      !finite(threshold) || threshold < 0.0) {
    result.failure = CellFailure::REGULARITY;
    result.reason = "regularity_bound_undetermined";
    return result;
  }
  bool zero_regular = false;
  const bool regularity_ok = applyRegularity(
      midpoint.p_w, midpoint.N_w, threshold, config.half_width,
      regularity_lower, regularity_upper, zero_regular);
  if (regularity_ok && intervalContainsZero(regularity_lower,
                                            regularity_upper)) {
    result.accepted = true;
    result.lower = regularity_lower;
    result.upper = regularity_upper;
    result.zero_only = regularity_lower == 0.0 && regularity_upper == 0.0;
    result.failure = CellFailure::NONE;
    return result;
  }
  // If the regularity proof fails, a complete path-speed lower bound may still
  // retain zero at a terminal cell; nonzero-width cells first try bounded
  // subdivision before that fallback is considered.
  result.failure = regularity_ok ? CellFailure::REGULARITY :
                                   CellFailure::REGULARITY;
  result.reason = regularity_ok ? "regularity_has_no_nonzero_capacity"
                                : "regularity_bound_failed";
  return result;
}

bool validateEnvironment(const SectionEnvironment& environment,
                         const SectionBuildConfig& config,
                         std::string& reason) {
  reason.clear();
  if (!environment.available) {
    reason = "environment_unknown";
    return false;
  }
  if (!orderedBox(environment.reference_domain, true) ||
      !orderedBox(environment.obstacle_region, true)) {
    reason = "invalid_environment_domain";
    return false;
  }
  if (!finite(config.clearance) || config.clearance < 0.0) {
    reason = "invalid_clearance";
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    double lower = 0.0;
    double upper = 0.0;
    if (!safeSub(environment.reference_domain.min(axis), config.clearance,
                 lower) ||
        !safeAdd(environment.reference_domain.max(axis), config.clearance,
                 upper) ||
        !sumGreaterEqual(environment.reference_domain.min(axis),
                         -config.clearance,
                         environment.obstacle_region.min(axis)) ||
        !sumLessEqual(environment.reference_domain.max(axis),
                      config.clearance,
                      environment.obstacle_region.max(axis))) {
      reason = "reference_domain_clearance_contract_invalid";
      return false;
    }
  }
  if (environment.obstacles.size() > config.max_obstacles) {
    reason = "obstacle_budget_exceeded";
    return false;
  }
  for (const SectionBox& obstacle : environment.obstacles) {
    if (!orderedBox(obstacle, true)) {
      reason = "invalid_obstacle_box";
      return false;
    }
  }
  return true;
}

bool validateConfig(const SectionBuildConfig& config, std::string& reason) {
  reason.clear();
  if (!finite(config.clearance) || config.clearance < 0.0 ||
      !finite(config.half_width) || config.half_width < 0.0 ||
      !finite(config.minimum_reference_speed) ||
      config.minimum_reference_speed <= 0.0 || !finite(config.max_step_w) ||
      !finite(config.min_step_w) || config.max_step_w <= 0.0 ||
      config.min_step_w <= 0.0 || config.min_step_w > config.max_step_w ||
      config.max_depth < 0 || config.max_cells == 0U ||
      config.max_obstacle_checks == 0U || config.max_obstacles == 0U) {
    reason = "invalid_build_config";
    return false;
  }
  return true;
}

bool validateInput(const SectionBuildInput& input,
                   const SectionBuildConfig& config,
                   std::string& reason) {
  reason.clear();
  if (!finite(input.w_start) || !finite(input.w_end) ||
      !(input.w_end > input.w_start) || !input.point_query ||
      !input.bounds_query) {
    reason = "invalid_path_input";
    return false;
  }
  double previous = -std::numeric_limits<double>::infinity();
  for (const double breakpoint : input.structural_breakpoints) {
    if (!finite(breakpoint) || breakpoint < input.w_start ||
        breakpoint > input.w_end || breakpoint <= previous) {
      reason = "invalid_structural_breakpoints";
      return false;
    }
    previous = breakpoint;
  }
  if (!validateConfig(config, reason)) return false;
  return true;
}

bool buildInitialCells(const SectionBuildInput& input,
                       const SectionBuildConfig& config,
                       std::vector<std::pair<double, double>>& cells) {
  cells.clear();
  const std::size_t size_max = std::numeric_limits<std::size_t>::max();
  // Count interior structural boundaries before reserving/allocating.  Each
  // such boundary creates another mandatory segment (and therefore at least
  // one cell), so an over-budget list fails closed without a large reserve.
  std::size_t interior_breakpoints = 0U;
  for (const double breakpoint : input.structural_breakpoints) {
    if (breakpoint > input.w_start && breakpoint < input.w_end) {
      if (interior_breakpoints == size_max - 2U) return false;
      ++interior_breakpoints;
    }
  }
  if (interior_breakpoints == size_max ||
      interior_breakpoints + 1U > config.max_cells) {
    return false;
  }

  std::vector<double> boundaries;
  boundaries.reserve(interior_breakpoints + 2U);
  boundaries.push_back(input.w_start);
  for (const double breakpoint : input.structural_breakpoints) {
    if (breakpoint > input.w_start && breakpoint < input.w_end) {
      boundaries.push_back(breakpoint);
    }
  }
  boundaries.push_back(input.w_end);

  const auto ceilToSize = [&](const double ratio,
                              std::size_t& count) -> bool {
    if (!finite(ratio) || ratio < 0.0) return false;
    const double rounded = std::ceil(ratio);
    if (!finite(rounded)) return false;
    // On 64-bit platforms size_t::max rounds to 2^64 in double, which is
    // outside the integer conversion range.  Reject that sentinel explicitly
    // when size_t has more integer bits than double's mantissa.
    const double max_as_double = static_cast<double>(size_max);
    if (rounded > max_as_double ||
        (std::numeric_limits<std::size_t>::digits >
             std::numeric_limits<double>::digits &&
         rounded >= max_as_double)) {
      return false;
    }
    count = static_cast<std::size_t>(rounded);
    return true;
  };

  std::size_t total = 0U;
  std::vector<std::size_t> segment_counts;
  segment_counts.reserve(interior_breakpoints + 1U);
  for (std::size_t segment = 0U; segment + 1U < boundaries.size(); ++segment) {
    const double span = boundaries[segment + 1U] - boundaries[segment];
    const double ratio = span / config.max_step_w;
    if (!finite(span) || !(span > 0.0) || !finite(ratio)) return false;
    std::size_t count = 0U;
    if (!ceilToSize(ratio, count)) return false;
    if (count == 0U) count = 1U;
    if (count > config.max_cells - total) return false;
    total += count;
    segment_counts.push_back(count);
  }
  cells.reserve(total);
  std::size_t segment_index = 0U;
  for (std::size_t segment = 0U; segment + 1U < boundaries.size(); ++segment,
                       ++segment_index) {
    const double segment_start = boundaries[segment];
    const double segment_end = boundaries[segment + 1U];
    const std::size_t count = segment_counts[segment_index];
    double cursor = segment_start;
    for (std::size_t i = 0U; i < count; ++i) {
      const double w0 = i == 0U ? segment_start : cursor;
      const double w1 = i + 1U == count
          ? segment_end
          : segment_start + (segment_end - segment_start) *
                static_cast<double>(i + 1U) / static_cast<double>(count);
      if (!finite(w0) || !finite(w1) || !(w1 > w0)) return false;
      cells.emplace_back(w0, w1);
      cursor = w1;
    }
  }
  return cells.size() == total;
}

void recordFailure(SectionTubeProfile& profile, const CellResult& result,
                   const double w0) {
  if (!finite(profile.first_failure_w)) {
    profile.first_failure_w = w0;
    profile.failure_reason = result.reason;
  }
}

bool processCellRecursive(const SectionBuildInput& input,
                          const SectionBuildConfig& config,
                          BuilderContext& context,
                          const double w0,
                          const double w1,
                          const int depth,
                          std::vector<AcceptedCell>& accepted,
                          SectionTubeProfile& profile) {
  if (!context.budgetCellsAvailable()) {
    CellResult budget;
    budget.failure = CellFailure::BUDGET;
    budget.reason = "cell_budget_exceeded";
    recordFailure(profile, budget, w0);
    profile.status = SectionTubeStatus::BUDGET_EXCEEDED;
    return false;
  }
  CellResult result = evaluateCell(input, config, context, w0, w1, depth);
  if (result.accepted) {
    accepted.push_back(AcceptedCell{w0, w1, result.lower, result.upper,
                                   result.zero_only});
    return true;
  }
  if (result.failure == CellFailure::BUDGET) {
    recordFailure(profile, result, w0);
    profile.status = SectionTubeStatus::BUDGET_EXCEEDED;
    return false;
  }
  const double span = w1 - w0;
  const double half = 0.5 * span;
  // Explicit zero requests and genuine geometric zero cells were already
  // accepted above when their reference proof was available.  If that proof
  // is missing, subdivision cannot create transverse capacity and must not
  // turn a failed speed/geometry check into ZERO_ONLY.
  const bool no_transverse_capacity = config.half_width == 0.0 ||
      result.geometric_zero;
  if (!no_transverse_capacity && depth < config.max_depth &&
      finite(half) && half >= config.min_step_w &&
      profile.visited_cells < config.max_cells) {
    const double mid = w0 + half;
    if (finite(mid) && mid > w0 && mid < w1) {
      if (processCellRecursive(input, config, context, w0, mid, depth + 1,
                                accepted, profile) &&
          processCellRecursive(input, config, context, mid, w1, depth + 1,
                               accepted, profile)) {
        return true;
      }
      return false;
    }
  }
  // A regularity-only failure at the terminal resolution may still retain the
  // independently proven zero-offset path.  This fallback is intentionally
  // after subdivision, and is never reached for callback, seed, or budget
  // failures because those carry no zero_fallback_proven bit.
  if (result.failure == CellFailure::REGULARITY &&
      result.zero_fallback_proven) {
    accepted.push_back(AcceptedCell{w0, w1, 0.0, 0.0, true});
    return true;
  }
  recordFailure(profile, result, w0);
  profile.status = statusForFailure(result.failure);
  return false;
}

bool assembleProfile(const std::vector<AcceptedCell>& accepted,
                     SectionTubeProfile& profile) {
  if (accepted.empty()) return false;
  profile.knots.clear();
  profile.knots.reserve(accepted.size() + 1U);
  SectionTubeKnot first;
  first.w = accepted.front().w0;
  first.lower = accepted.front().lower;
  first.upper = accepted.front().upper;
  profile.knots.push_back(first);
  for (std::size_t i = 0U; i + 1U < accepted.size(); ++i) {
    const AcceptedCell& left = accepted[i];
    const AcceptedCell& right = accepted[i + 1U];
    if (!(right.w0 > left.w0) || left.w1 != right.w0) {
      return false;
    }
    SectionTubeKnot shared;
    shared.w = right.w0;
    shared.lower = std::max(left.lower, right.lower);
    shared.upper = std::min(left.upper, right.upper);
    if (!intervalContainsZero(shared.lower, shared.upper)) return false;
    profile.knots.push_back(shared);
  }
  SectionTubeKnot last;
  last.w = accepted.back().w1;
  last.lower = accepted.back().lower;
  last.upper = accepted.back().upper;
  profile.knots.push_back(last);
  for (std::size_t i = 0U; i < profile.knots.size(); ++i) {
    const SectionTubeKnot& knot = profile.knots[i];
    if (!finite(knot.w) || !finite(knot.lower) || !finite(knot.upper) ||
        !intervalContainsZero(knot.lower, knot.upper) ||
        (i != 0U && !(knot.w > profile.knots[i - 1U].w))) {
      profile.knots.clear();
      return false;
    }
  }
  profile.valid_start = profile.knots.front().w;
  profile.valid_end = profile.knots.back().w;
  profile.usable = true;
  return true;
}

}  // namespace

bool SectionTubeProfile::evaluate(const double w, double& lower,
                                  double& upper) const {
  lower = 0.0;
  upper = 0.0;
  if (!usable || knots.empty() || !finite(w) || !finite(valid_start) ||
      !finite(valid_end) || w < valid_start || w > valid_end) {
    return false;
  }
  if (w == valid_start) {
    lower = knots.front().lower;
    upper = knots.front().upper;
    return intervalContainsZero(lower, upper);
  }
  if (w == valid_end) {
    lower = knots.back().lower;
    upper = knots.back().upper;
    return intervalContainsZero(lower, upper);
  }
  const auto it = std::lower_bound(
      knots.begin(), knots.end(), w,
      [](const SectionTubeKnot& knot, const double value) {
        return knot.w < value;
      });
  if (it != knots.end() && it->w == w) {
    lower = it->lower;
    upper = it->upper;
    return intervalContainsZero(lower, upper);
  }
  if (it == knots.begin() || it == knots.end()) return false;
  const SectionTubeKnot& left = *(it - 1);
  const SectionTubeKnot& right = *it;
  const double denominator = right.w - left.w;
  if (!finite(denominator) || denominator <= 0.0) return false;
  const double ratio = (w - left.w) / denominator;
  if (!finite(ratio) || ratio < 0.0 || ratio > 1.0) return false;
  lower = left.lower + ratio * (right.lower - left.lower);
  upper = left.upper + ratio * (right.upper - left.upper);
  return finite(lower) && finite(upper) && intervalContainsZero(lower, upper);
}

SectionTubeProfile buildSectionTube(const SectionBuildInput& input,
                                    const SectionBuildConfig& config) {
  SectionTubeProfile profile;
  std::string reason;
  if (!validateInput(input, config, reason)) {
    profile.status = SectionTubeStatus::INVALID_INPUT;
    profile.failure_reason = reason;
    return profile;
  }
  if (!input.environment.available) {
    profile.status = SectionTubeStatus::UNKNOWN_DOMAIN;
    profile.failure_reason = "environment_unknown";
    return profile;
  }
  if (!validateEnvironment(input.environment, config, reason)) {
    profile.status = input.environment.obstacles.size() > config.max_obstacles
        ? SectionTubeStatus::BUDGET_EXCEEDED
        : SectionTubeStatus::INVALID_INPUT;
    profile.failure_reason = reason;
    return profile;
  }

  std::vector<std::pair<double, double>> initial_cells;
  if (!buildInitialCells(input, config, initial_cells)) {
    profile.status = SectionTubeStatus::BUDGET_EXCEEDED;
    profile.failure_reason = "initial_cell_budget_exceeded";
    return profile;
  }
  BuilderContext context(input, config, profile);
  std::vector<AcceptedCell> accepted;
  accepted.reserve(initial_cells.size());
  bool all_initial_cells_processed = true;
  for (const auto& cell : initial_cells) {
    if (!processCellRecursive(input, config, context, cell.first, cell.second,
                              0, accepted, profile)) {
      all_initial_cells_processed = false;
      break;
    }
  }
  if (accepted.empty()) {
    profile.usable = false;
    profile.complete = false;
    profile.knots.clear();
    if (profile.status == SectionTubeStatus::INVALID_INPUT) {
      profile.status = SectionTubeStatus::GEOMETRY_UNAVAILABLE;
    }
    return profile;
  }
  if (!assembleProfile(accepted, profile)) {
    profile.usable = false;
    profile.complete = false;
    profile.knots.clear();
    profile.status = SectionTubeStatus::GEOMETRY_UNAVAILABLE;
    profile.failure_reason = "profile_node_assembly_failed";
    return profile;
  }

  // The execution query remains the global path query.  At a request
  // boundary that is also a segment seam it may intentionally select the
  // preceding closed segment, while the first/last accepted cell owns the
  // segment used for its proof.  Compare those values exactly and, only when
  // they differ, prove the global endpoint independently and intersect the
  // corresponding profile knot.
  if (input.cell_point_query) {
    const auto failEndpoint = [&](const double endpoint_w,
                                  const CellFailure failure,
                                  const std::string& endpoint_reason) {
      profile.usable = false;
      profile.complete = false;
      profile.knots.clear();
      profile.valid_start = 0.0;
      profile.valid_end = 0.0;
      profile.status = statusForFailure(failure);
      profile.failure_reason = endpoint_reason;
      if (!finite(profile.first_failure_w)) profile.first_failure_w = endpoint_w;
    };
    const auto checkEndpoint = [&](const std::size_t knot_index,
                                   const AcceptedCell& cell,
                                   const double endpoint_w) -> bool {
      SectionPathSample cell_sample;
      if (!context.queryPoint(cell.w0, cell.w1, endpoint_w, cell_sample)) {
        failEndpoint(endpoint_w, CellFailure::GEOMETRY,
                     "endpoint_cell_point_query_failed");
        return false;
      }
      SectionPathSample global_sample;
      if (!context.queryGlobalPoint(endpoint_w, global_sample)) {
        failEndpoint(endpoint_w, CellFailure::GEOMETRY,
                     "endpoint_global_point_query_failed");
        return false;
      }
      std::string sample_reason;
      if (!validatePathSample(cell_sample, sample_reason)) {
        failEndpoint(endpoint_w,
                     sample_reason.find("normal_") == 0U ||
                             sample_reason.find("horizontal_") == 0U
                         ? CellFailure::FRAME : CellFailure::GEOMETRY,
                     "endpoint_cell_" + sample_reason);
        return false;
      }
      if (!validatePathSample(global_sample, sample_reason)) {
        failEndpoint(endpoint_w,
                     sample_reason.find("normal_") == 0U ||
                             sample_reason.find("horizontal_") == 0U
                         ? CellFailure::FRAME : CellFailure::GEOMETRY,
                     "endpoint_global_" + sample_reason);
        return false;
      }
      if (samplesExactlyEqual(cell_sample, global_sample)) return true;

      double point_lower = 0.0;
      double point_upper = 0.0;
      CellFailure failure = CellFailure::GEOMETRY;
      std::string point_reason;
      if (!computeSinglePointInterval(input, config, context, global_sample,
                                      point_lower, point_upper, failure,
                                      point_reason)) {
        failEndpoint(endpoint_w, failure,
                     "endpoint_global_geometry_" + point_reason);
        return false;
      }
      SectionTubeKnot& knot = profile.knots[knot_index];
      knot.lower = std::max(knot.lower, point_lower);
      knot.upper = std::min(knot.upper, point_upper);
      if (!intervalContainsZero(knot.lower, knot.upper)) {
        failEndpoint(endpoint_w, CellFailure::GEOMETRY,
                     "endpoint_interval_intersection_empty");
        return false;
      }
      return true;
    };

    if (!checkEndpoint(0U, accepted.front(), profile.valid_start) ||
        !checkEndpoint(profile.knots.size() - 1U, accepted.back(),
                       profile.valid_end)) {
      return profile;
    }
  }

  const bool exact_coverage =
      !accepted.empty() && accepted.front().w0 == input.w_start &&
      accepted.back().w1 == input.w_end;
  profile.complete = all_initial_cells_processed && exact_coverage &&
      profile.status != SectionTubeStatus::BUDGET_EXCEEDED &&
      !finite(profile.first_failure_w);
  if (!profile.complete) {
    // Once a nonempty prefix has been assembled it is a usable partial
    // result even when traversal stopped at a budget boundary.  Keep the
    // budget diagnostic in failure_reason while exposing PARTIAL semantics.
    profile.status = SectionTubeStatus::PARTIAL;
  } else {
    bool all_zero = true;
    for (const SectionTubeKnot& knot : profile.knots) {
      if (knot.lower < 0.0 || knot.upper > 0.0) {
        all_zero = false;
        break;
      }
    }
    profile.status = all_zero ? SectionTubeStatus::ZERO_ONLY
                              : SectionTubeStatus::COMPLETE;
  }
  return profile;
}

}  // namespace phase_offset_navigation
