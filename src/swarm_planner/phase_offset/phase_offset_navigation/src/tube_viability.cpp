#include "phase_offset_navigation/tube_viability.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace phase_offset_navigation {
namespace {

constexpr double kDefaultTolerance = 1e-10;
constexpr double kMinimumSpacing = 1e-12;
constexpr std::size_t kMaximumSamples = 100000U;

bool Finite(const double value) { return std::isfinite(value); }

bool RevisionMatches(const std::uint64_t actual,
                     const std::uint64_t expected) {
  return expected == 0U || actual == expected;
}

TubeViabilityInterval MakeInterval(const double lower, const double upper) {
  TubeViabilityInterval interval;
  interval.lower = lower;
  interval.upper = upper;
  interval.valid = Finite(lower) && Finite(upper) && lower <= upper;
  return interval;
}

bool OrderedSections(const std::vector<TubeViabilityCrossSection>& sections) {
  if (sections.size() < 2U) return false;
  for (std::size_t i = 0U; i < sections.size(); ++i) {
    if (!Finite(sections[i].w) || !Finite(sections[i].lower) ||
        !Finite(sections[i].upper) || sections[i].lower > sections[i].upper) {
      return false;
    }
    if (i > 0U && sections[i].w <= sections[i - 1U].w + kMinimumSpacing) {
      return false;
    }
  }
  return true;
}

struct SourceSection {
  double w = 0.0;
  double lower = 0.0;
  double upper = 0.0;
};

bool SelectProfileBounds(const TubeProfile& profile,
                         const TubeRawSample& sample,
                         double& lower,
                         double& upper) {
  const bool filtered_valid = Finite(sample.filtered_lower) &&
      Finite(sample.filtered_upper) &&
      sample.filtered_lower <= sample.filtered_upper;
  // A complete filtered profile is authoritative.  Invalid filtered evidence
  // must fail closed rather than falling back to a broader raw interval.
  if (profile.filtered_complete) {
    if (!filtered_valid) return false;
    lower = sample.filtered_lower;
    upper = sample.filtered_upper;
    return true;
  }
  const bool raw_valid = Finite(sample.raw_lower) &&
      Finite(sample.raw_upper) && sample.raw_lower <= sample.raw_upper;
  if (profile.raw_complete && raw_valid) {
    lower = sample.raw_lower;
    upper = sample.raw_upper;
    return true;
  }
  if (filtered_valid) {
    lower = sample.filtered_lower;
    upper = sample.filtered_upper;
    return true;
  }
  if (raw_valid) {
    lower = sample.raw_lower;
    upper = sample.raw_upper;
    return true;
  }
  return false;
}

bool BuildProfileSource(const TubeProfile& profile,
                        std::vector<SourceSection>& source,
                        std::string& reason) {
  source.clear();
  if (!profile.complete || profile.samples.size() < 2U) {
    reason = "geometric Tube profile is incomplete";
    return false;
  }

  source.resize(profile.samples.size());
  for (std::size_t i = 0U; i < profile.samples.size(); ++i) {
    const TubeRawSample& sample = profile.samples[i];
    if (!Finite(sample.w) || (i > 0U &&
        sample.w <= profile.samples[i - 1U].w + kMinimumSpacing)) {
      reason = "profile samples are not strictly ordered in phase";
      return false;
    }
    double lower = 0.0;
    double upper = 0.0;
    if (!SelectProfileBounds(profile, sample, lower, upper)) {
      reason = "profile cross-section is invalid";
      return false;
    }
    source[i].w = sample.w;
    source[i].lower = lower;
    source[i].upper = upper;
  }

  // Native phase coordinate w is authoritative.  Position vectors are not
  // consulted because they cannot create a second production coordinate.
  if (!Finite(profile.preview_start_w) || !Finite(profile.preview_end_w) ||
      profile.preview_end_w <= profile.preview_start_w + kMinimumSpacing ||
      profile.preview_start_w < source.front().w - kDefaultTolerance ||
      profile.preview_end_w > source.back().w + kDefaultTolerance) {
    reason = "profile preview W domain is invalid";
    return false;
  }
  return true;
}

bool InterpolateSource(const std::vector<SourceSection>& source,
                       const double w,
                       SourceSection& value) {
  if (source.size() < 2U || !Finite(w) ||
      w < source.front().w - kDefaultTolerance ||
      w > source.back().w + kDefaultTolerance) {
    return false;
  }
  const double query_w = std::max(source.front().w,
      std::min(source.back().w, w));
  std::size_t index = 0U;
  while (index + 1U < source.size() &&
         source[index + 1U].w <= query_w) {
    ++index;
  }
  if (index + 1U >= source.size()) index = source.size() - 2U;
  const SourceSection& first = source[index];
  const SourceSection& second = source[index + 1U];
  const double dw = second.w - first.w;
  if (!Finite(dw) || dw <= kMinimumSpacing) return false;
  const double alpha = (query_w - first.w) / dw;
  if (!Finite(alpha)) return false;
  value.w = query_w;
  value.lower = first.lower + alpha * (second.lower - first.lower);
  value.upper = first.upper + alpha * (second.upper - first.upper);
  return Finite(value.lower) && Finite(value.upper) &&
      value.lower <= value.upper;
}

bool BuildSource(const TubeViabilityInput& input,
                 std::vector<SourceSection>& source,
                 std::string& reason) {
  source.clear();
  if (!input.cross_sections.empty()) {
    if (!OrderedSections(input.cross_sections)) {
      reason = "value-only cross-sections are not ordered or valid";
      return false;
    }
    source.reserve(input.cross_sections.size());
    for (const TubeViabilityCrossSection& section : input.cross_sections) {
      SourceSection value;
      value.w = section.w;
      value.lower = section.lower;
      value.upper = section.upper;
      source.push_back(value);
    }
    return true;
  }
  if (input.profile == nullptr) {
    reason = "no immutable geometric Tube profile was supplied";
    return false;
  }
  return BuildProfileSource(*input.profile, source, reason);
}

std::uint64_t ResolveRevision(const std::uint64_t supplied,
                              const std::uint64_t profile_revision) {
  return supplied != 0U ? supplied : profile_revision;
}

bool BindRevisions(const TubeViabilityInput& input,
                   TubeViabilityResult& output,
                   std::string& reason) {
  const TubeProfile* profile = input.profile;
  const std::uint64_t profile_path = profile == nullptr
      ? 0U : profile->path_revision;
  const std::uint64_t profile_frame = profile == nullptr
      ? 0U : profile->frame_revision;
  const std::uint64_t profile_profile = profile == nullptr
      ? 0U : profile->profile_revision;
  output.provenance.path_revision =
      ResolveRevision(input.path_revision, profile_path);
  output.provenance.frame_revision =
      ResolveRevision(input.frame_revision, profile_frame);
  output.provenance.profile_revision =
      ResolveRevision(input.profile_revision, profile_profile);
  if (profile != nullptr) {
    output.provenance.source_revision = profile->source_revision;
    output.provenance.tube_revision = profile->tube_revision;
    output.provenance.map_revision = profile->map_revision;
    output.provenance.obstacle_contract_id = profile->obstacle_contract_id;
    if ((input.path_revision != 0U && profile_path != 0U &&
         input.path_revision != profile_path) ||
        (input.frame_revision != 0U && profile_frame != 0U &&
         input.frame_revision != profile_frame) ||
        (input.profile_revision != 0U && profile_profile != 0U &&
         input.profile_revision != profile_profile)) {
      reason = "profile revision does not match the current immutable input";
      return false;
    }
  }
  if (!RevisionMatches(output.provenance.path_revision,
                       input.expected_path_revision) ||
      !RevisionMatches(output.provenance.frame_revision,
                       input.expected_frame_revision) ||
      !RevisionMatches(output.provenance.profile_revision,
                       input.expected_profile_revision)) {
    reason = "path/frame/profile revision mismatch";
    return false;
  }
  return true;
}

double SelectedSlope(const TubeViabilityKnot& knot,
                     const bool lower,
                     const bool prefer_right) {
  if (lower) {
    if (prefer_right && knot.lower_slope_right_valid) {
      return knot.lower_slope_right;
    }
    if (!prefer_right && knot.lower_slope_left_valid) {
      return knot.lower_slope_left;
    }
    if (knot.lower_slope_right_valid) return knot.lower_slope_right;
    if (knot.lower_slope_left_valid) return knot.lower_slope_left;
  } else {
    if (prefer_right && knot.upper_slope_right_valid) {
      return knot.upper_slope_right;
    }
    if (!prefer_right && knot.upper_slope_left_valid) {
      return knot.upper_slope_left;
    }
    if (knot.upper_slope_right_valid) return knot.upper_slope_right;
    if (knot.upper_slope_left_valid) return knot.upper_slope_left;
  }
  return 0.0;
}

bool BuildRateInterval(const TubeViabilityKnot& knot,
                       const double delta,
                       const double lower_nu,
                       const double upper_nu,
                       const double upper_u_delta,
                       const double tolerance,
                       TubeViabilityRateInterval& output) {
  output = TubeViabilityRateInterval();
  if (!knot.reachable.valid || !Finite(delta) || !Finite(lower_nu) ||
      !Finite(upper_nu) || lower_nu <= 0.0 || upper_nu < lower_nu ||
      !Finite(upper_u_delta) || upper_u_delta < 0.0) {
    return false;
  }
  if (!knot.reachable.contains(delta, tolerance)) return false;
  output.lower = -upper_u_delta;
  output.upper = upper_u_delta;
  const bool at_lower = std::abs(delta - knot.reachable.lower) <= tolerance;
  const bool at_upper = std::abs(delta - knot.reachable.upper) <= tolerance;
  output.lower_boundary_active = at_lower;
  output.upper_boundary_active = at_upper;
  if (at_lower) {
    output.lower_boundary_slope = SelectedSlope(knot, true, true);
    const double robust_lower = output.lower_boundary_slope >= 0.0
        ? output.lower_boundary_slope * upper_nu
        : output.lower_boundary_slope * lower_nu;
    if (!Finite(robust_lower)) return false;
    output.lower = std::max(output.lower, robust_lower);
  }
  if (at_upper) {
    output.upper_boundary_slope = SelectedSlope(knot, false, true);
    const double robust_upper = output.upper_boundary_slope >= 0.0
        ? output.upper_boundary_slope * lower_nu
        : output.upper_boundary_slope * upper_nu;
    if (!Finite(robust_upper)) return false;
    output.upper = std::min(output.upper, robust_upper);
  }
  output.valid = Finite(output.lower) && Finite(output.upper) &&
      output.lower <= output.upper;
  return output.valid;
}

void PopulateSlopes(std::vector<TubeViabilityKnot>& knots) {
  if (knots.size() < 2U) return;
  for (std::size_t i = 0U; i < knots.size(); ++i) {
    if (i > 0U && knots[i - 1U].reachable.valid &&
        knots[i].reachable.valid) {
      const double dw = knots[i].w - knots[i - 1U].w;
      if (Finite(dw) && dw > kMinimumSpacing) {
        knots[i].lower_slope_left =
            (knots[i].reachable.lower - knots[i - 1U].reachable.lower) / dw;
        knots[i].upper_slope_left =
            (knots[i].reachable.upper - knots[i - 1U].reachable.upper) / dw;
        knots[i].lower_slope_left_valid =
            Finite(knots[i].lower_slope_left);
        knots[i].upper_slope_left_valid =
            Finite(knots[i].upper_slope_left);
      }
    }
    if (i + 1U < knots.size() && knots[i].reachable.valid &&
        knots[i + 1U].reachable.valid) {
      const double dw = knots[i + 1U].w - knots[i].w;
      if (Finite(dw) && dw > kMinimumSpacing) {
        knots[i].lower_slope_right =
            (knots[i + 1U].reachable.lower - knots[i].reachable.lower) / dw;
        knots[i].upper_slope_right =
            (knots[i + 1U].reachable.upper - knots[i].reachable.upper) / dw;
        knots[i].lower_slope_right_valid =
            Finite(knots[i].lower_slope_right);
        knots[i].upper_slope_right_valid =
            Finite(knots[i].upper_slope_right);
      }
    }
  }
}

std::size_t KnotForW(const TubeViabilityResult& result, const double w) {
  if (result.knots.size() < 2U) return result.knots.size();
  std::size_t index = 0U;
  while (index + 1U < result.knots.size() &&
         result.knots[index + 1U].w <= w) {
    ++index;
  }
  if (index + 1U >= result.knots.size()) index = result.knots.size() - 2U;
  return index;
}

}  // namespace

bool NormalPreviewProductionPolicy::valid() const {
  return immutable && Finite(preview_horizon_w) && preview_horizon_w > 0.0 &&
      Finite(sample_spacing_w) && sample_spacing_w > 0.0 &&
      Finite(lower_nu) && lower_nu > 0.0 && Finite(upper_nu) &&
      upper_nu >= lower_nu && Finite(b_tight) && b_tight >= 0.0 &&
      Finite(b_open) && b_open > b_tight && policy_revision != 0U &&
      configuration_identity != 0U;
}

const char* tubeViabilityStatusName(const TubeViabilityStatus status) {
  switch (status) {
    case TubeViabilityStatus::NOT_EVALUATED: return "NOT_EVALUATED";
    case TubeViabilityStatus::FEASIBLE: return "FEASIBLE";
    case TubeViabilityStatus::RATE_INFEASIBLE: return "RATE_INFEASIBLE";
    case TubeViabilityStatus::PREVIEW_INFEASIBLE:
      return "PREVIEW_INFEASIBLE";
    case TubeViabilityStatus::CURRENT_DELTA_OUTSIDE:
      return "CURRENT_DELTA_OUTSIDE";
    case TubeViabilityStatus::STALE: return "STALE";
    case TubeViabilityStatus::INVALID_INPUT: return "INVALID_INPUT";
  }
  return "UNKNOWN";
}

double TubeViability::smoothstep(const double q) {
  if (q <= 0.0) return 0.0;
  if (q >= 1.0) return 1.0;
  return 3.0 * q * q - 2.0 * q * q * q;
}

double tubeViabilitySmoothstep(const double q) {
  return TubeViability::smoothstep(q);
}

bool TubeViability::evaluate(
    const std::vector<TubeViabilityCrossSection>& cross_sections,
    const TubeViabilityInput& input,
    TubeViabilityResult& output) {
  TubeViabilityInput value = input;
  value.cross_sections = cross_sections;
  return evaluate(value, output);
}

bool TubeViability::evaluate(const TubeViabilityInput& input,
                             TubeViabilityResult& output) {
  output = TubeViabilityResult();
  output.provenance.immutable = true;
  output.policy = input.policy;
  output.upper_u_delta = input.upper_u_delta;
  output.provenance.policy_revision = input.policy.policy_revision;
  output.provenance.policy_configuration_identity =
      input.policy.configuration_identity;
  output.provenance.policy_configuration_id = input.policy.configuration_id;

  std::string reason;
  if (!BindRevisions(input, output, reason)) {
    output.status = TubeViabilityStatus::STALE;
    output.reason = reason;
    return false;
  }
  if (!input.policy.valid() || !Finite(input.current_w) ||
      !Finite(input.current_delta) || !Finite(input.upper_u_delta) ||
      input.upper_u_delta < 0.0 || !Finite(input.boundary_tolerance) ||
      input.boundary_tolerance < 0.0) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.reason = "required W-domain NORMAL Preview policy or capability is invalid";
    return false;
  }

  std::vector<SourceSection> source;
  if (!BuildSource(input, source, reason)) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.reason = reason;
    return false;
  }
  const double source_start_w = source.front().w;
  const double source_end_w = source.back().w;
  if (input.current_w < source_start_w - kDefaultTolerance ||
      input.current_w > source_end_w + kDefaultTolerance) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.reason = "current phase is outside the native W source domain";
    return false;
  }
  output.current_w = std::max(source_start_w,
      std::min(source_end_w, input.current_w));
  if (input.profile != nullptr &&
      output.current_w < input.profile->preview_start_w -
          kDefaultTolerance) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.reason = "current phase precedes the immutable profile preview start";
    return false;
  }
  output.current_w_inside = true;

  const double requested_end = output.current_w + input.policy.preview_horizon_w;
  const double profile_end = input.profile != nullptr
      ? input.profile->preview_end_w : source_end_w;
  if (!Finite(requested_end) || !Finite(profile_end) ||
      profile_end <= output.current_w +
      kMinimumSpacing || profile_end > source_end_w + kDefaultTolerance) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.reason = "immutable profile preview_end_w is invalid";
    return false;
  }
  double effective_end = std::min(requested_end, profile_end);
  if (!Finite(effective_end) || effective_end <= output.current_w +
      kMinimumSpacing) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.reason = "effective W-domain preview is empty";
    return false;
  }
  if (requested_end > profile_end + kDefaultTolerance) {
    output.preview_truncated_after = true;
    effective_end = profile_end;
  }

  const double horizon_w = effective_end - output.current_w;
  const double ratio = horizon_w / input.policy.sample_spacing_w;
  if (!Finite(ratio) || ratio <= 0.0 ||
      ratio > static_cast<double>(kMaximumSamples - 1U)) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.reason = "derived W-domain preview sample count exceeds bounds";
    return false;
  }
  const std::size_t count = static_cast<std::size_t>(std::ceil(ratio)) + 1U;
  if (count < 2U || count > kMaximumSamples) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.reason = "derived W-domain preview sample count is invalid";
    return false;
  }

  output.preview_start_w = output.current_w;
  output.preview_end_w = effective_end;
  output.effective_horizon_w = horizon_w;
  output.delta_w = horizon_w / static_cast<double>(count - 1U);
  output.delta_delta = input.upper_u_delta * output.delta_w /
      input.policy.upper_nu;
  if (!Finite(output.delta_w) || output.delta_w <= kMinimumSpacing ||
      !Finite(output.delta_delta)) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.reason = "W-domain preview spacing or transverse reach is invalid";
    return false;
  }
  output.delta_reach = MakeInterval(-output.delta_delta,
                                    output.delta_delta);

  output.knots.resize(count);
  for (std::size_t i = 0U; i < count; ++i) {
    TubeViabilityKnot& knot = output.knots[i];
    knot.w = (i + 1U == count) ? output.preview_end_w :
        output.preview_start_w + output.delta_w * static_cast<double>(i);
    SourceSection section;
    if (!InterpolateSource(source, knot.w, section)) {
      output.status = TubeViabilityStatus::INVALID_INPUT;
      output.reason = "uniform W-domain preview sample is outside the source domain";
      output.knots.clear();
      return false;
    }
    knot.geometric = MakeInterval(section.lower, section.upper);
    if (!knot.geometric.valid) {
      output.status = TubeViabilityStatus::INVALID_INPUT;
      output.reason = "uniform preview cross-section is invalid";
      output.knots.clear();
      return false;
    }
  }

  // Exact backward viability recursion.  Minkowski addition with the
  // symmetric rate interval is scalar interval expansion by delta_delta.
  output.knots.back().reachable = output.knots.back().geometric;
  for (std::size_t i = count - 1U; i-- > 0U;) {
    const TubeViabilityInterval& geometric = output.knots[i].geometric;
    const TubeViabilityInterval& successor = output.knots[i + 1U].reachable;
    if (!successor.valid) {
      output.knots[i].reachable = TubeViabilityInterval();
      continue;
    }
    output.knots[i].reachable = MakeInterval(
        std::max(geometric.lower, successor.lower - output.delta_delta),
        std::min(geometric.upper, successor.upper + output.delta_delta));
  }

  PopulateSlopes(output.knots);
  output.feasible = true;
  for (const TubeViabilityKnot& knot : output.knots) {
    output.feasible = output.feasible && knot.reachable.valid;
  }
  if (output.feasible) {
    output.allowed_inward_contraction_slope =
        input.upper_u_delta / input.policy.upper_nu;
    output.max_inward_contraction_slope = 0.0;
    for (const TubeViabilityKnot& knot : output.knots) {
      if (knot.lower_slope_left_valid) {
        output.max_inward_contraction_slope = std::max(
            output.max_inward_contraction_slope,
            std::max(0.0, knot.lower_slope_left));
      }
      if (knot.lower_slope_right_valid) {
        output.max_inward_contraction_slope = std::max(
            output.max_inward_contraction_slope,
            std::max(0.0, knot.lower_slope_right));
      }
      if (knot.upper_slope_left_valid) {
        output.max_inward_contraction_slope = std::max(
            output.max_inward_contraction_slope,
            std::max(0.0, -knot.upper_slope_left));
      }
      if (knot.upper_slope_right_valid) {
        output.max_inward_contraction_slope = std::max(
            output.max_inward_contraction_slope,
            std::max(0.0, -knot.upper_slope_right));
      }
    }
    output.contraction_rate_feasible =
        output.max_inward_contraction_slope <=
        output.allowed_inward_contraction_slope + input.boundary_tolerance;
    if (!output.contraction_rate_feasible) {
      output.status = TubeViabilityStatus::RATE_INFEASIBLE;
      output.reason = "continuous envelope contracts faster than the rate bound";
    }
    output.b_pre = std::numeric_limits<double>::infinity();
    for (const TubeViabilityKnot& knot : output.knots) {
      output.b_pre = std::min(output.b_pre, knot.reachable.width());
    }
    if (!Finite(output.b_pre) || output.b_pre < 0.0) {
      output.status = TubeViabilityStatus::INVALID_INPUT;
      output.reason = "preview width is invalid";
      return false;
    }
    output.beta_argument = (output.b_pre - input.policy.b_tight) /
        (input.policy.b_open - input.policy.b_tight);
    output.beta = smoothstep(output.beta_argument);
    output.beta_i = output.beta;
    if (!Finite(output.beta)) {
      output.status = TubeViabilityStatus::INVALID_INPUT;
      output.reason = "preview beta is invalid";
      return false;
    }
    for (TubeViabilityKnot& knot : output.knots) {
      BuildRateInterval(knot, knot.reachable.lower, input.policy.lower_nu,
                        input.policy.upper_nu, input.upper_u_delta,
                        input.boundary_tolerance,
                        knot.lower_boundary_rate);
      BuildRateInterval(knot, knot.reachable.upper, input.policy.lower_nu,
                        input.policy.upper_nu, input.upper_u_delta,
                        input.boundary_tolerance,
                        knot.upper_boundary_rate);
    }
  } else {
    output.status = TubeViabilityStatus::PREVIEW_INFEASIBLE;
    output.reason = "backward rate-limited preview envelope is empty";
  }

  if (output.feasible) {
    const TubeViabilityKnot& first = output.knots.front();
    output.current_delta_inside = first.reachable.contains(
        input.current_delta, input.boundary_tolerance);
    if (output.current_delta_inside) {
      const bool local_rate_feasible = BuildRateInterval(
          first, input.current_delta, input.policy.lower_nu,
          input.policy.upper_nu, input.upper_u_delta, input.boundary_tolerance,
          output.current_rate_interval);
      output.rate_feasible = output.contraction_rate_feasible &&
          local_rate_feasible;
      if (!local_rate_feasible && output.contraction_rate_feasible) {
        output.status = TubeViabilityStatus::RATE_INFEASIBLE;
        output.reason = "current boundary rate interval is empty";
      }
    } else {
      output.status = TubeViabilityStatus::CURRENT_DELTA_OUTSIDE;
      output.reason = "current delta is outside K_0";
    }
  }

  output.valid = true;
  if (output.feasible && output.contraction_rate_feasible &&
      output.current_delta_inside && output.rate_feasible &&
      output.status != TubeViabilityStatus::RATE_INFEASIBLE) {
    output.status = TubeViabilityStatus::FEASIBLE;
    if (output.reason.empty()) output.reason = "preview is feasible";
  }
  return true;
}

bool TubeViability::queryEnvelope(const TubeViabilityResult& result,
                                  const double w,
                                  TubeViabilityInterval& interval) {
  interval = TubeViabilityInterval();
  if (!result.valid || !result.feasible || result.knots.size() < 2U ||
      !Finite(w) || w < result.preview_start_w - kDefaultTolerance ||
      w > result.preview_end_w + kDefaultTolerance) {
    return false;
  }
  const double query_w = std::max(result.preview_start_w,
      std::min(result.preview_end_w, w));
  const std::size_t index = KnotForW(result, query_w);
  if (index >= result.knots.size() - 1U) {
    interval = result.knots.back().reachable;
    return interval.valid;
  }
  const TubeViabilityKnot& first = result.knots[index];
  const TubeViabilityKnot& second = result.knots[index + 1U];
  const double dw = second.w - first.w;
  if (!Finite(dw) || dw <= kMinimumSpacing || !first.reachable.valid ||
      !second.reachable.valid) {
    return false;
  }
  const double alpha = (query_w - first.w) / dw;
  interval = MakeInterval(
      first.reachable.lower + alpha *
          (second.reachable.lower - first.reachable.lower),
      first.reachable.upper + alpha *
          (second.reachable.upper - first.reachable.upper));
  return interval.valid;
}

bool TubeViability::queryRateInterval(const TubeViabilityResult& result,
                                      const double w,
                                      const double delta,
                                      TubeViabilityRateInterval& interval) {
  return queryRateInterval(result, w, delta, result.policy.lower_nu, interval);
}

bool TubeViability::queryRateInterval(const TubeViabilityResult& result,
                                      const double w,
                                      const double delta,
                                      const double phase_rate,
                                      TubeViabilityRateInterval& interval) {
  interval = TubeViabilityRateInterval();
  if (!result.valid || !result.feasible || result.knots.size() < 2U ||
      !Finite(w) || !Finite(delta) || !Finite(phase_rate) ||
      phase_rate < result.policy.lower_nu - kDefaultTolerance ||
      phase_rate > result.policy.upper_nu + kDefaultTolerance ||
      w < result.preview_start_w - kDefaultTolerance ||
      w > result.preview_end_w + kDefaultTolerance ||
      !result.policy.valid()) {
    return false;
  }
  const double query_w = std::max(result.preview_start_w,
      std::min(result.preview_end_w, w));
  const std::size_t index = KnotForW(result, query_w);
  if (index >= result.knots.size() - 1U) {
    return BuildRateInterval(result.knots.back(), delta,
                             result.policy.lower_nu, result.policy.upper_nu,
                             result.upper_u_delta,
                             kDefaultTolerance, interval);
  }
  const TubeViabilityKnot& first = result.knots[index];
  const TubeViabilityKnot& second = result.knots[index + 1U];
  const double dw = second.w - first.w;
  if (!Finite(dw) || dw <= kMinimumSpacing || !first.reachable.valid ||
      !second.reachable.valid) {
    return false;
  }
  const double alpha = (query_w - first.w) / dw;
  const TubeViabilityInterval envelope = MakeInterval(
      first.reachable.lower + alpha *
          (second.reachable.lower - first.reachable.lower),
      first.reachable.upper + alpha *
          (second.reachable.upper - first.reachable.upper));
  if (!envelope.valid || !envelope.contains(delta, kDefaultTolerance)) {
    return false;
  }
  TubeViabilityKnot local = first;
  local.reachable = envelope;
  local.lower_slope_left_valid = false;
  local.upper_slope_left_valid = false;
  local.lower_slope_right =
      (second.reachable.lower - first.reachable.lower) / dw;
  local.upper_slope_right =
      (second.reachable.upper - first.reachable.upper) / dw;
  local.lower_slope_right_valid = Finite(local.lower_slope_right);
  local.upper_slope_right_valid = Finite(local.upper_slope_right);
  return BuildRateInterval(local, delta, result.policy.lower_nu,
                           result.policy.upper_nu, result.upper_u_delta,
                           kDefaultTolerance, interval);
}

bool TubeViabilityResult::envelopeAt(const double w,
                                     TubeViabilityInterval& interval) const {
  return TubeViability::queryEnvelope(*this, w, interval);
}

bool TubeViabilityResult::rateIntervalAt(
    const double w, const double delta,
    TubeViabilityRateInterval& interval) const {
  return TubeViability::queryRateInterval(*this, w, delta, interval);
}

bool TubeViabilityResult::rateIntervalAt(
    const double w, const double delta, const double phase_rate,
    TubeViabilityRateInterval& interval) const {
  return TubeViability::queryRateInterval(*this, w, delta, phase_rate,
                                          interval);
}

}  // namespace phase_offset_navigation
