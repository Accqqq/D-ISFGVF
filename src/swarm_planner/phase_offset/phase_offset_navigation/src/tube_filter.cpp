#include "phase_offset_navigation/tube_filter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace phase_offset_navigation {
namespace {

constexpr double kEpsilon = 1e-10;
constexpr double kAnchorPhaseMatchTolerance = 1e-8;

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool IsOrdered(const TubeProfile& profile) {
  for (std::size_t index = 1U; index < profile.samples.size(); ++index) {
    const double previous_w = profile.samples[index - 1U].w;
    const double current_w = profile.samples[index].w;
    if (!IsFinite(previous_w) || !IsFinite(current_w) ||
        current_w <= previous_w + kEpsilon) {
      return false;
    }
  }
  return true;
}

std::size_t FindAnchor(const TubeProfile& profile, const double current_w) {
  if (!IsFinite(current_w)) return profile.samples.size();
  std::size_t anchor = profile.samples.size();
  double best_error = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < profile.samples.size(); ++index) {
    const double w = profile.samples[index].w;
    if (!IsFinite(w)) continue;
    const double error = std::abs(w - current_w);
    if (error <= kAnchorPhaseMatchTolerance &&
        (error < best_error - kEpsilon ||
         (std::abs(error - best_error) <= kEpsilon && index < anchor))) {
      anchor = index;
      best_error = error;
    }
  }
  return anchor;
}

bool ValidRawSample(const TubeRawSample& sample) {
  return sample.complete && IsFinite(sample.w) &&
      IsFinite(sample.raw_lower) && IsFinite(sample.raw_upper) &&
      sample.raw_lower <= sample.raw_upper + kEpsilon;
}

}  // namespace

TubeFilter::TubeFilter(const TubeFilterConfig& config) : config_(config) {}

bool TubeFilter::configurationValid() const {
  // boundary_slope_max and dense_samples_per_segment remain compatibility
  // fields for older callers.  Local PWL geometry is analytic and deliberately
  // has no configuration gate based on either legacy field.
  return true;
}

bool TubeFilter::filter(TubeProfile& profile, const double current_w) const {
  profile.filtered_complete = false;
  profile.complete = false;
  if (!configurationValid() || !profile.raw_complete ||
      profile.samples.size() < 2U || !IsOrdered(profile) ||
      !IsFinite(current_w)) {
    return false;
  }
  for (const TubeRawSample& sample : profile.samples) {
    if (!ValidRawSample(sample)) return false;
  }
  if (FindAnchor(profile, current_w) >= profile.samples.size()) return false;

  // Each knot remains exactly the complete raw Builder interval.  A cell is
  // represented by the affine interpolation between its two endpoints; this
  // is conservative for the sampled/raw knot contract and has no dependence
  // on a remote bottleneck or a legacy global slope bound.
  std::vector<TubeRawSample, Eigen::aligned_allocator<TubeRawSample>> local;
  local = profile.samples;
  for (std::size_t index = 0U; index < local.size(); ++index) {
    TubeRawSample& sample = local[index];
    sample.filtered_lower = sample.raw_lower;
    sample.filtered_upper = sample.raw_upper;
    sample.filtered_contains_zero = sample.filtered_lower <= kEpsilon &&
        sample.filtered_upper >= -kEpsilon;
    if (index + 1U < local.size()) {
      const double dw = local[index + 1U].w - local[index].w;
      if (!IsFinite(dw) || dw <= kEpsilon) return false;
      sample.lower_w = (local[index + 1U].raw_lower - sample.raw_lower) / dw;
      sample.upper_w = (local[index + 1U].raw_upper - sample.raw_upper) / dw;
      if (!IsFinite(sample.lower_w) || !IsFinite(sample.upper_w)) return false;
    }
  }

  // The final knot has no right cell.  Expose the left one-sided derivative;
  // interior knots are queried from their right cell by query().
  local.back().lower_w = local[local.size() - 2U].lower_w;
  local.back().upper_w = local[local.size() - 2U].upper_w;

  profile.diagnostics.min_width = std::numeric_limits<double>::infinity();
  for (const TubeRawSample& sample : local) {
    if (!IsFinite(sample.filtered_lower) || !IsFinite(sample.filtered_upper) ||
        sample.filtered_lower > sample.filtered_upper + kEpsilon) {
      return false;
    }
    profile.diagnostics.min_width = std::min(
        profile.diagnostics.min_width,
        std::max(0.0, sample.filtered_upper - sample.filtered_lower));
  }
  if (!IsFinite(profile.diagnostics.min_width)) {
    profile.diagnostics.min_width = 0.0;
  }

  // The Filter does not create truncation provenance.  Any pre-existing
  // before/after truncation and reason belong to the Builder/Validator and are
  // intentionally preserved verbatim.
  profile.samples.swap(local);
  profile.preview_start_w = profile.samples.front().w;
  profile.preview_end_w = profile.samples.back().w;
  profile.certified_segment_start_w = profile.preview_start_w;
  profile.certified_segment_end_w = profile.preview_end_w;
  profile.filtered_complete = true;
  profile.complete = true;
  return true;
}

bool TubeFilter::query(const TubeProfile& profile,
                       const double w,
                       TubeBounds& bounds) {
  bounds = TubeBounds();
  if (!profile.complete || profile.samples.empty() || !IsFinite(w) ||
      w < profile.preview_start_w - kEpsilon ||
      w > profile.preview_end_w + kEpsilon) {
    return false;
  }
  const std::size_t count = profile.samples.size();
  if (count == 1U) {
    const TubeRawSample& sample = profile.samples.front();
    if (std::abs(w - sample.w) > kEpsilon) return false;
    bounds.query_w = sample.w;
    bounds.lower = sample.filtered_lower;
    bounds.upper = sample.filtered_upper;
    bounds.lower_w = sample.lower_w;
    bounds.upper_w = sample.upper_w;
    bounds.valid = IsFinite(bounds.lower) && IsFinite(bounds.upper) &&
        IsFinite(bounds.lower_w) && IsFinite(bounds.upper_w) &&
        bounds.lower <= bounds.upper + kEpsilon;
    return bounds.valid;
  }

  const double query_w = std::max(profile.preview_start_w,
      std::min(profile.preview_end_w, w));
  std::size_t first_index = 0U;
  while (first_index + 1U < count &&
         profile.samples[first_index + 1U].w <= query_w) {
    ++first_index;
  }
  if (first_index + 1U == count) --first_index;
  const TubeRawSample& first = profile.samples[first_index];
  const TubeRawSample& second = profile.samples[first_index + 1U];
  const double dw = second.w - first.w;
  if (!IsFinite(dw) || dw <= kEpsilon) return false;
  const double s = (query_w - first.w) / dw;
  if (!IsFinite(s) || s < -kEpsilon || s > 1.0 + kEpsilon) return false;
  bounds.query_w = query_w;
  bounds.lower = first.filtered_lower + s *
      (second.filtered_lower - first.filtered_lower);
  bounds.upper = first.filtered_upper + s *
      (second.filtered_upper - first.filtered_upper);
  bounds.lower_w = first.lower_w;
  bounds.upper_w = first.upper_w;
  bounds.valid = IsFinite(bounds.lower) && IsFinite(bounds.upper) &&
      IsFinite(bounds.lower_w) && IsFinite(bounds.upper_w) &&
      bounds.lower <= bounds.upper + kEpsilon;
  return bounds.valid;
}

}  // namespace phase_offset_navigation
