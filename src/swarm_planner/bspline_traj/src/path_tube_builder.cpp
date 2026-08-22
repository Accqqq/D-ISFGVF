#include "bspline_race/path_tube_builder.h"

#include <algorithm>
#include <cmath>

namespace FLAG_Race
{

namespace
{

// Ray-step along +/-N from the path point; return the offset where the
// distance to the nearest obstacle drops below the clearance.  The returned
// offset is the last ray position that still keeps reference_clearance; when
// the ray leaves the map or enters a voxel whose ESDF has not been computed
// (distance sentinel 10000) the sample cannot be certified and certified is
// set to false (offset keeps the last safe value).
//
// NOTE: this simulator has no depth camera, so the base occupancy buffer
// stays at its "unknown" initialization value even after ESDF updates; the
// ESDF distance itself (distance_buffer_all_) is the authoritative
// observed/unknown signal used below.
struct RayBound
{
  double offset = 0.0;
  bool certified = true;
};

namespace
{
constexpr double kEsdfUnobserved = 1e3;
}

RayBound
offsetBound(const Eigen::Vector3d& p, const Eigen::Vector3d& N,
            SDFMap* map, double ray_step, double max_offset,
            double clearance)
{
  RayBound out;
  if (!map || !map->isInMap(p) || map->getDistance(p) >= kEsdfUnobserved)
  {
    out.certified = false;
    return out;
  }
  for (double d = ray_step; d <= max_offset + 1e-9; d += ray_step)
  {
    const Eigen::Vector3d q = p + N * d;
    if (std::getenv("TUBE_DEBUG") && d <= ray_step + 1e-9)
      fprintf(stderr,
              "[tube-ray] d=%.2f inmap=%d obs=%d dist=%.3f\n",
              d, map->isInMap(q) ? 1 : 0,
              (map->isInMap(q) && map->getDistance(q) < kEsdfUnobserved) ? 1 : 0,
              map->isInMap(q) ? map->getDistance(q) : -1.0);
    if (!map->isInMap(q) || map->getDistance(q) >= kEsdfUnobserved)
    {
      // out-of-map / ESDF-unobserved is blocked, not free: stop and mark
      // uncertified.
      out.certified = false;
      break;
    }
    if (map->getDistance(q) < clearance)
      break;
    out.offset = d;
  }
  return out;
}

}  // namespace

bool
PathTubeBuilder::rebuild(const ContinuousPhasePath& path,
                         double current_w, double current_delta,
                         TubeProfile& profile)
{
  profile = TubeProfile();
  if (!map_ || !map_->mapReady())
    return false;

  const double w_end = current_w + params.lookahead_w;
  ContinuousPhasePathState st;
  const int steps = std::max(
    2, static_cast<int>(std::ceil(
         (w_end - current_w) / params.sample_step_w)));
  const double step = (w_end - current_w) / steps;

  for (int i = 0; i <= steps; ++i)
  {
    const double w = current_w + i * step;
    if (!path.evaluate(w, st, true))
      return false;
    const double v = st.dp_dw.norm();
    if (v < 1e-6 || st.dp_dw.head<2>().norm() < 1e-3)
      return false;
    const Eigen::Vector3d T = st.dp_dw / v;
    const Eigen::Vector3d N(-T.y(), T.x(), 0.0);

    TubeSample s;
    s.w = w;
    if (std::getenv("TUBE_DEBUG"))
      fprintf(stderr,
              "[tube] w=%.2f p=(%.2f,%.2f) center_dist=%.3f\n",
              w, st.p.x(), st.p.y(),
              map_ && map_->isInMap(st.p) ? map_->getDistance(st.p) : -1.0);
    const RayBound ub = offsetBound(st.p, N, map_.get(), params.ray_step,
                                    params.max_offset,
                                    params.reference_clearance);
    const RayBound lb = offsetBound(st.p, -N, map_.get(), params.ray_step,
                                    params.max_offset,
                                    params.reference_clearance);

    // Curvature regularity bound: |delta| <= (1 - mu) / max(kappa, eps).
    const double kappa =
      (st.dp_dw.x() * st.d2p_dw2.y() - st.dp_dw.y() * st.d2p_dw2.x()) /
      (v * v * v);
    const double reg_bound =
      std::abs(kappa) > 1e-9
        ? std::max(0.0, (1.0 - params.mu_regular) / std::abs(kappa))
        : params.max_offset;
    s.upper = std::min(ub.offset, reg_bound);
    s.lower = -std::min(lb.offset, reg_bound);
    s.certified = ub.certified && lb.certified;
    s.regularity = 1.0 - std::abs(kappa) * current_delta;
    s.beta = std::max(
      0.0, std::min(1.0, (s.upper - s.lower) / params.open_width));
    if (std::getenv("TUBE_DEBUG"))
      fprintf(stderr, "[tube]   upper=%.3f lower=%.3f beta=%.3f cert=%d\n",
              s.upper, s.lower, s.beta, s.certified ? 1 : 0);
    if (std::getenv("TUBE_DEBUG") && s.upper - s.lower < 0.2)
      fprintf(stderr,
              "[tube-narrow] w=%.3f p=(%.3f,%.3f) kappa=%.3f reg=%.3f "
              "up=%.3f lo=%.3f\n",
              w, st.p.x(), st.p.y(), kappa, reg_bound, ub.offset, lb.offset);
    profile.samples.push_back(s);
  }

  // Conservative slopes (one-sided at the boundaries, central inside) with
  // slope limiting.
  const int n = static_cast<int>(profile.samples.size());
  for (int i = 0; i < n; ++i)
  {
    TubeSample& s = profile.samples[i];
    double du = 0.0;
    double dl = 0.0;
    if (i > 0)
    {
      const double dw = std::max(1e-6, s.w - profile.samples[i - 1].w);
      du = (s.upper - profile.samples[i - 1].upper) / dw;
      dl = (s.lower - profile.samples[i - 1].lower) / dw;
    }
    if (i + 1 < n)
    {
      const double dw = std::max(1e-6, profile.samples[i + 1].w - s.w);
      du = (profile.samples[i + 1].upper - s.upper) / dw;
      dl = (profile.samples[i + 1].lower - s.lower) / dw;
    }
    s.upper_dw =
      std::max(-params.boundary_slope_max,
               std::min(params.boundary_slope_max, du));
    s.lower_dw =
      std::max(-params.boundary_slope_max,
               std::min(params.boundary_slope_max, dl));
  }

  profile.w0 = profile.samples.front().w;
  profile.w1 = profile.samples.back().w;
  profile.epoch = ++epoch_;
  profile.stamp = ros::Time::now();
  profile.valid = true;
  return true;
}

bool
PathTubeBuilder::query(const TubeProfile& profile, double w,
                       TubeBounds& bounds) const
{
  bounds = TubeBounds();
  if (!profile.valid || profile.samples.size() < 2)
    return false;
  const double wc = std::max(profile.w0, std::min(w, profile.w1));
  // Linear interpolation over the samples.
  size_t j = 0;
  for (size_t i = 1; i < profile.samples.size(); ++i)
  {
    if (profile.samples[i].w >= wc)
    {
      j = i;
      break;
    }
  }
  if (j == 0)
    j = 1;
  const TubeSample& a = profile.samples[j - 1];
  const TubeSample& b = profile.samples[j];
  const double dw = std::max(1e-9, b.w - a.w);
  const double s = (wc - a.w) / dw;
  bounds.query_w = wc;
  bounds.lower = a.lower + (b.lower - a.lower) * s;
  bounds.upper = a.upper + (b.upper - a.upper) * s;
  bounds.lower_dw = a.lower_dw + (b.lower_dw - a.lower_dw) * s;
  bounds.upper_dw = a.upper_dw + (b.upper_dw - a.upper_dw) * s;
  bounds.beta = a.beta + (b.beta - a.beta) * s;
  bounds.min_regularity = 1.0;
  for (const TubeSample& sm : profile.samples)
    bounds.min_regularity = std::min(bounds.min_regularity, sm.regularity);
  bounds.stamp = profile.stamp;
  bounds.epoch = profile.epoch;
  bounds.valid = true;
  return true;
}

bool
PathTubeBuilder::queryConservativeLookahead(const TubeProfile& profile,
                                            double w0, double w1,
                                            TubeBounds& bounds) const
{
  bounds = TubeBounds();
  if (!profile.valid || profile.samples.size() < 2)
    return false;
  const double a0 = std::max(profile.w0, w0);
  const double a1 = std::min(profile.w1, w1);
  if (a1 < a0)
    return false;

  double lower = -1e9;
  double upper = 1e9;
  double beta = 1.0;
  double min_reg = 1.0;
  bool any_nonempty = false;
  for (const TubeSample& s : profile.samples)
  {
    if (std::getenv("TUBE_DEBUG") && s.w < 1.5)
      fprintf(stderr, "[tube]   q s.w=%.6f\n", s.w);
    if (s.w < a0 - 1e-9 || s.w > a1 + 1e-9)
      continue;
    if (!s.certified)
      continue;  // unknown / out-of-map samples are not claimed passable
    // Conservative for the whole window: the delta must fit inside EVERY
    // sample, so lower is the MAXIMUM of the sample lowers and upper is the
    // MINIMUM of the sample uppers.
    lower = std::max(lower, s.lower);
    upper = std::min(upper, s.upper);
    beta = std::min(beta, s.beta);
    min_reg = std::min(min_reg, s.regularity);
    any_nonempty = true;
  }
  if (std::getenv("TUBE_DEBUG"))
    fprintf(stderr,
            "[tube] conservative a0=%.3f a1=%.3f w0=%.3f w1=%.3f "
            "lower=%.4f upper=%.4f n=%zu\n",
            a0, a1, profile.w0, profile.w1, lower, upper,
            profile.samples.size());
  if (!any_nonempty || lower > upper)
    return false;
  bounds.query_w = a0;
  bounds.lower = lower;
  bounds.upper = upper;
  bounds.lower_dw = 0.0;  // conservative: no slope credit over the window
  bounds.upper_dw = 0.0;
  bounds.beta = beta;
  bounds.min_regularity = min_reg;
  bounds.stamp = profile.stamp;
  bounds.epoch = profile.epoch;
  bounds.valid = true;
  return true;
}

bool
PathTubeBuilder::lineOfSight(const Eigen::Vector3d& a,
                             const Eigen::Vector3d& b) const
{
  if (!map_)
    return true;
  const Eigen::Vector3d dir = b - a;
  const double dist = dir.norm();
  if (dist < 1e-6)
    return true;
  const Eigen::Vector3d step = dir / dist;
  const double ds = 0.1;
  for (double s = 0.0; s <= dist; s += ds)
  {
    const Eigen::Vector3d p = a + step * s;
    if (!map_->isInMap(p) || map_->getInflateOccupancy(p) > 0)
      return false;
  }
  return true;
}

}  // namespace FLAG_Race
