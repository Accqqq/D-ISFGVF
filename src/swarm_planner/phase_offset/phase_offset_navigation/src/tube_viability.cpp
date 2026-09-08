#include "phase_offset_navigation/tube_viability.h"

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#if defined(__i386__) || defined(__x86_64__)
#include <immintrin.h>
#endif

#pragma STDC FP_CONTRACT OFF

namespace phase_offset_navigation {
namespace {

constexpr double kDefaultTolerance = 1e-10;
constexpr double kMinimumSpacing = 1e-12;
constexpr std::size_t kMaximumSamples = 100000U;
// S1-C permits a frozen certificate to contain up to 250,000 cells.  The
// live preview budget above applies only to the merged window, not to the
// already accepted immutable profile as a whole.
constexpr std::size_t kMaximumProfileCells = 250000U;
constexpr std::size_t kMaximumProfileKnots = kMaximumProfileCells + 1U;

bool Finite(const double value) { return std::isfinite(value); }

bool ConsumeWorkV2(const std::size_t amount, const std::size_t limit,
                   std::size_t& work) {
  if (amount > std::numeric_limits<std::size_t>::max() - work) return false;
  if (limit != 0U && (work > limit || amount > limit - work)) return false;
  work += amount;
  return true;
}

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

int CompareProductToDoubleV2(const double lhs, const double rhs,
                             const double candidate) {
  const phase_offset_core::Binary64Dyadic lhs_parts =
      phase_offset_core::decomposeBinary64(lhs);
  const phase_offset_core::Binary64Dyadic rhs_parts =
      phase_offset_core::decomposeBinary64(rhs);
  const phase_offset_core::Binary64Dyadic candidate_parts =
      phase_offset_core::decomposeBinary64(candidate);
  const unsigned __int128 product =
      static_cast<unsigned __int128>(lhs_parts.significand) *
      static_cast<unsigned __int128>(rhs_parts.significand);
  return phase_offset_core::compareDyadic(
      product, lhs_parts.exponent + rhs_parts.exponent,
      candidate_parts.significand, candidate_parts.exponent);
}

int CompareRatioToDoubleV2(const double numerator, const double denominator,
                           const double candidate) {
  const phase_offset_core::Binary64Dyadic numerator_parts =
      phase_offset_core::decomposeBinary64(numerator);
  const phase_offset_core::Binary64Dyadic denominator_parts =
      phase_offset_core::decomposeBinary64(denominator);
  const phase_offset_core::Binary64Dyadic candidate_parts =
      phase_offset_core::decomposeBinary64(candidate);
  const unsigned __int128 product =
      static_cast<unsigned __int128>(candidate_parts.significand) *
      static_cast<unsigned __int128>(denominator_parts.significand);
  // Compare candidate*denominator against numerator: <= 0 means the
  // candidate is a downward (safe) quotient bound.
  return phase_offset_core::compareDyadic(
      product, candidate_parts.exponent + denominator_parts.exponent,
      numerator_parts.significand, numerator_parts.exponent);
}

double Binary64FromBitsV2(const std::uint64_t bits) {
  double result = 0.0;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

std::uint64_t Binary64BitsV2(const double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

bool DifferenceDownwardV2(double lhs, double rhs, double& result);
bool SumFloorV2(double lhs, double rhs, double& result);

bool ProductFloorV2(const double lhs, const double rhs, double& result) {
  if (!Finite(lhs) || !Finite(rhs) || lhs < 0.0 || rhs < 0.0) return false;
  if (lhs == 0.0 || rhs == 0.0) {
    result = 0.0;
    return true;
  }
  double candidate = lhs * rhs;
  if (!Finite(candidate)) {
    const double maximum = std::numeric_limits<double>::max();
    if (CompareProductToDoubleV2(lhs, rhs, maximum) > 0) return false;
    candidate = maximum;
  }
  const int comparison = CompareProductToDoubleV2(lhs, rhs, candidate);
  if (comparison >= 0) {
    result = candidate;
    return true;
  }
  const double previous = std::nextafter(candidate, 0.0);
  if (Finite(previous) &&
      CompareProductToDoubleV2(lhs, rhs, previous) >= 0) {
    result = previous;
    return true;
  }
  std::uint64_t low = 0U;
  std::uint64_t high = Binary64BitsV2(candidate);
  while (low < high) {
    const std::uint64_t middle = low + (high - low + 1U) / 2U;
    if (CompareProductToDoubleV2(lhs, rhs,
                                 Binary64FromBitsV2(middle)) >= 0) {
      low = middle;
    } else {
      high = middle - 1U;
    }
  }
  result = Binary64FromBitsV2(low);
  return Finite(result);
}

bool RatioFloorV2(const double numerator, const double denominator,
                  double& result) {
  if (!Finite(numerator) || !Finite(denominator) || numerator < 0.0 ||
      denominator <= 0.0) return false;
  if (numerator == 0.0) {
    result = 0.0;
    return true;
  }
  double candidate = numerator / denominator;
  if (!Finite(candidate)) {
    const double maximum = std::numeric_limits<double>::max();
    // compare(candidate*denominator, numerator) < 0 means the exact
    // quotient is above DBL_MAX; saturating would violate the finite bound.
    if (CompareRatioToDoubleV2(numerator, denominator, maximum) < 0) {
      return false;
    }
    candidate = maximum;
  }
  if (CompareRatioToDoubleV2(numerator, denominator, candidate) <= 0) {
    result = candidate;
    return true;
  }
  const double previous = std::nextafter(candidate, 0.0);
  if (Finite(previous) &&
      CompareRatioToDoubleV2(numerator, denominator, previous) <= 0) {
    result = previous;
    return true;
  }
  std::uint64_t low = 0U;
  std::uint64_t high = Binary64BitsV2(candidate);
  while (low < high) {
    const std::uint64_t middle = low + (high - low + 1U) / 2U;
    if (CompareRatioToDoubleV2(numerator, denominator,
                               Binary64FromBitsV2(middle)) <= 0) {
      low = middle;
    } else {
      high = middle - 1U;
    }
  }
  result = Binary64FromBitsV2(low);
  return Finite(result);
}

// Smallest finite binary64 value no less than the exact non-negative ratio.
// This is used only for safe grid-count allocation; unlike the downward
// reachability ratio above, rounding upward may add one sample but can never
// omit a required cell.
bool RatioCeilV2(const double numerator, const double denominator,
                 double& result) {
  if (!Finite(numerator) || !Finite(denominator) || numerator < 0.0 ||
      denominator <= 0.0) return false;
  if (numerator == 0.0) {
    result = 0.0;
    return true;
  }
  double candidate = numerator / denominator;
  if (!Finite(candidate)) return false;
  if (CompareRatioToDoubleV2(numerator, denominator, candidate) >= 0) {
    result = candidate;
    return true;
  }
  result = std::nextafter(candidate,
                          std::numeric_limits<double>::infinity());
  return Finite(result);
}

bool CompareSumToDoubleV2(const double lhs, const double rhs,
                          const double candidate, int& comparison) {
  if (!Finite(lhs) || !Finite(rhs) || !Finite(candidate) || lhs < 0.0 ||
      rhs < 0.0 || candidate < 0.0) return false;
  const phase_offset_core::Binary64Dyadic lhs_parts =
      phase_offset_core::decomposeBinary64(lhs);
  const phase_offset_core::Binary64Dyadic rhs_parts =
      phase_offset_core::decomposeBinary64(rhs);
  const phase_offset_core::Binary64Dyadic candidate_parts =
      phase_offset_core::decomposeBinary64(candidate);
  const int common_exponent =
      std::min(lhs_parts.exponent, rhs_parts.exponent);
  const int lhs_shift = lhs_parts.exponent - common_exponent;
  const int rhs_shift = rhs_parts.exponent - common_exponent;
  // Keep the exact fixed-width path bounded.  Ordinary W-domain arithmetic
  // uses nearby exponents; a far-separated fallback below remains safely
  // outward rather than inventing an equality.
  if (lhs_shift > 74 || rhs_shift > 74) return false;
  const unsigned __int128 sum =
      (static_cast<unsigned __int128>(lhs_parts.significand) <<
       static_cast<unsigned>(lhs_shift)) +
      (static_cast<unsigned __int128>(rhs_parts.significand) <<
       static_cast<unsigned>(rhs_shift));
  comparison = phase_offset_core::compareDyadic(
      sum, common_exponent, candidate_parts.significand,
      candidate_parts.exponent);
  return true;
}

bool SumExactV2(const double lhs, const double rhs, const double raw) {
  if (!Finite(lhs) || !Finite(rhs) || !Finite(raw)) return false;
  if (lhs == 0.0 || rhs == 0.0) return true;
  const double lhs_abs = std::abs(lhs);
  const double rhs_abs = std::abs(rhs);
  const double raw_abs = std::abs(raw);
  if (std::signbit(lhs) == std::signbit(rhs)) {
    int comparison = 0;
    return CompareSumToDoubleV2(lhs_abs, rhs_abs, raw_abs, comparison) &&
        comparison == 0;
  }
  // Opposite-sign addition is subtraction of magnitudes.  Sterbenz's lemma
  // certifies exactness only within a factor of two; equal magnitudes give
  // exact cancellation.  Never treat a rounded zero as proof of equality.
  if (lhs_abs == rhs_abs) return raw == 0.0;
  const double hi = std::max(lhs_abs, rhs_abs);
  const double lo = std::min(lhs_abs, rhs_abs);
  const double ratio = lo > 0.0 ? hi / lo :
      std::numeric_limits<double>::infinity();
  return Finite(ratio) && ratio <= 2.0;
}

bool SumCeilV2(const double lhs, const double rhs, double& result) {
  if (!Finite(lhs) || !Finite(rhs)) return false;
  if (lhs == 0.0) {
    result = rhs;
    return true;
  }
  if (rhs == 0.0) {
    result = lhs;
    return true;
  }
  const double candidate = lhs + rhs;
  if (!Finite(candidate)) return false;
  if (SumExactV2(lhs, rhs, candidate)) {
    result = candidate;
  } else {
    result = std::nextafter(candidate,
                            std::numeric_limits<double>::infinity());
  }
  return Finite(result);
}

bool SumFloorV2(const double lhs, const double rhs, double& result) {
  if (!Finite(lhs) || !Finite(rhs)) return false;
  if (lhs == 0.0) {
    result = rhs;
    return true;
  }
  if (rhs == 0.0) {
    result = lhs;
    return true;
  }
  const double candidate = lhs + rhs;
  if (!Finite(candidate)) return false;
  if (SumExactV2(lhs, rhs, candidate)) {
    result = candidate;
  } else {
    // Floor toward negative infinity.  Moving toward zero would be upward
    // for a negative rounded sum and could exceed the exact grid endpoint.
    result = std::nextafter(candidate,
                            -std::numeric_limits<double>::infinity());
  }
  return Finite(result);
}

bool DifferenceDownwardV2(const double lhs, const double rhs,
                          double& result) {
  if (!Finite(lhs) || !Finite(rhs) || !(lhs > rhs)) return false;
  const double raw = lhs - rhs;
  if (!Finite(raw) || !(raw > 0.0)) return false;
  // Sterbenz's lemma guarantees exact subtraction for nearby same-sign
  // operands (including one-ULP seams).  Opposite-sign subtraction is an
  // addition and is exact only when the dyadic sum compares equal to the
  // rounded result.  Do not infer either property from a product: tiny
  // opposite-sign operands can underflow that product to -0.
  bool exact = rhs == 0.0;
  if (!exact && std::signbit(lhs) == std::signbit(rhs)) {
    const double lhs_abs = std::abs(lhs);
    const double rhs_abs = std::abs(rhs);
    const double hi = std::max(lhs_abs, rhs_abs);
    const double lo = std::min(lhs_abs, rhs_abs);
    const double ratio = lo > 0.0 ? hi / lo : std::numeric_limits<double>::infinity();
    exact = Finite(ratio) && ratio <= 2.0;
  } else if (!exact) {
    int comparison = 0;
    exact = CompareSumToDoubleV2(std::abs(lhs), std::abs(rhs),
                                 std::abs(raw), comparison) && comparison == 0;
  }
  if (exact) {
    result = raw;
  } else {
    result = std::nextafter(raw, 0.0);
  }
  return Finite(result) && result > 0.0;
}

bool BuildMergedNodesV2(const TubeProfileV2& profile, const double current_w,
                        const double horizon_w, const double spacing_w,
                        const std::size_t max_nodes,
                        std::vector<double>& nodes, std::string& reason) {
  nodes.clear();
  if (profile.knots.size() < 2U || max_nodes < 2U ||
      profile.knots.size() > kMaximumProfileKnots) {
    reason = "immutable V2 profile knot count is out of bounds";
    return false;
  }
  double horizon = 0.0;
  if (!DifferenceDownwardV2(horizon_w, current_w, horizon)) {
    reason = "live V2 horizon width is not representable";
    return false;
  }
  double ratio = 0.0;
  if (!Finite(horizon) || horizon <= 0.0 ||
      !RatioCeilV2(horizon, spacing_w, ratio) || !Finite(ratio) ||
      ratio <= 0.0 || ratio > static_cast<double>(max_nodes - 1U)) {
    reason = "live V2 sample-grid ratio is out of bounds";
    return false;
  }
  const double rounded_count = std::ceil(ratio);
  if (!Finite(rounded_count) || rounded_count < 1.0 ||
      rounded_count > static_cast<double>(max_nodes - 1U)) {
    reason = "live V2 sample-grid count is out of bounds";
    return false;
  }
  const std::size_t sample_count = static_cast<std::size_t>(rounded_count);
  const auto first_ge = std::lower_bound(
      profile.knots.begin(), profile.knots.end(), current_w,
      [](const TubePwlKnotV2& knot, const double value) {
        return knot.w < value;
      });
  const auto first_gt = std::upper_bound(
      profile.knots.begin(), profile.knots.end(), horizon_w,
      [](const double value, const TubePwlKnotV2& knot) {
        return value < knot.w;
      });
  const std::size_t geometric_count =
      static_cast<std::size_t>(first_gt - first_ge);
  if (geometric_count > std::numeric_limits<std::size_t>::max() -
          sample_count - 2U) {
    reason = "live V2 partition size overflow";
    return false;
  }
  const std::size_t raw_bound = geometric_count + sample_count + 2U;
  if (raw_bound > max_nodes) {
    reason = "live V2 merged partition exceeds finite bound";
    return false;
  }
  nodes.reserve(raw_bound);
  nodes.push_back(current_w);
  nodes.push_back(horizon_w);
  for (auto iterator = first_ge; iterator != first_gt; ++iterator) {
    nodes.push_back(iterator->w);
  }
  for (std::size_t k = 1U; k <= sample_count; ++k) {
    double offset = 0.0;
    if (!ProductFloorV2(static_cast<double>(k), spacing_w, offset)) {
      reason = "live V2 sample-grid product overflow";
      return false;
    }
    double candidate = 0.0;
    if (!SumFloorV2(current_w, offset, candidate)) {
      reason = "live V2 sample-grid sum is nonfinite";
      return false;
    }
    if (!Finite(candidate) || !(candidate > current_w)) {
      if (!Finite(candidate)) {
        reason = "live V2 sample-grid endpoint is nonfinite";
        return false;
      }
      break;
    }
    if (candidate >= horizon_w) break;
    nodes.push_back(candidate);
  }
  std::sort(nodes.begin(), nodes.end());
  nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
  if (nodes.size() < 2U || nodes.size() > max_nodes ||
      nodes.front() != current_w || nodes.back() != horizon_w) {
    reason = "live V2 merged partition is malformed";
    return false;
  }
  return true;
}

struct V2Interval {
  double lower = 0.0;
  double upper = 0.0;
  bool valid = false;
};

bool StrictTubeIntervalValidV2(const TubeViabilityInterval& value) {
  return value.valid && Finite(value.lower) && Finite(value.upper) &&
      value.lower <= value.upper;
}

V2Interval V2InvalidInterval() { return V2Interval(); }

V2Interval V2Point(const double value) {
  if (!Finite(value)) return V2InvalidInterval();
  V2Interval result;
  result.lower = value;
  result.upper = value;
  result.valid = true;
  return result;
}

V2Interval V2Bounds(const double lower, const double upper) {
  if (!Finite(lower) || !Finite(upper) || lower > upper) {
    return V2InvalidInterval();
  }
  V2Interval result;
  result.lower = lower;
  result.upper = upper;
  result.valid = true;
  return result;
}

bool V2ExactZero(const V2Interval& value) {
  return value.valid && value.lower == 0.0 && value.upper == 0.0;
}

V2Interval V2Add(const V2Interval& lhs, const V2Interval& rhs) {
  if (!lhs.valid || !rhs.valid) return V2InvalidInterval();
  if (V2ExactZero(lhs)) return rhs;
  if (V2ExactZero(rhs)) return lhs;
  const double lower = lhs.lower + rhs.lower;
  const double upper = lhs.upper + rhs.upper;
  if (!Finite(lower) || !Finite(upper)) return V2InvalidInterval();
  if (lhs.lower == lhs.upper && rhs.lower == rhs.upper) {
    const double raw = lhs.lower + rhs.lower;
    if (SumExactV2(lhs.lower, rhs.lower, raw)) return V2Point(raw);
  }
  const double outward_lower = std::nextafter(
      lower, -std::numeric_limits<double>::infinity());
  const double outward_upper = std::nextafter(
      upper, std::numeric_limits<double>::infinity());
  return V2Bounds(outward_lower, outward_upper);
}

V2Interval V2Subtract(const V2Interval& lhs, const V2Interval& rhs) {
  if (!lhs.valid || !rhs.valid) return V2InvalidInterval();
  if (V2ExactZero(rhs)) return lhs;
  if (V2ExactZero(lhs)) {
    return V2Bounds(-rhs.upper, -rhs.lower);
  }
  if (lhs.lower == lhs.upper && rhs.lower == rhs.upper &&
      lhs.lower == rhs.lower) {
    return V2Point(0.0);
  }
  const double lower = lhs.lower - rhs.upper;
  const double upper = lhs.upper - rhs.lower;
  if (!Finite(lower) || !Finite(upper)) return V2InvalidInterval();
  if (lhs.lower == lhs.upper && rhs.lower == rhs.upper) {
    const double raw = lhs.lower - rhs.lower;
    const double lhs_abs = std::abs(lhs.lower);
    const double rhs_abs = std::abs(rhs.lower);
    bool exact = false;
    const bool opposite_sign =
        std::signbit(lhs.lower) != std::signbit(rhs.lower);
    if (opposite_sign) {
      int comparison = 0;
      exact = CompareSumToDoubleV2(lhs_abs, rhs_abs, std::abs(raw),
                                   comparison) && comparison == 0;
    } else {
      const double hi = std::max(lhs_abs, rhs_abs);
      const double lo = std::min(lhs_abs, rhs_abs);
      const double ratio = lo > 0.0 ? hi / lo :
          std::numeric_limits<double>::infinity();
      exact = (lhs_abs == rhs_abs) || (Finite(ratio) && ratio <= 2.0);
    }
    if (exact) return V2Point(raw);
  }
  return V2Bounds(
      std::nextafter(lower, -std::numeric_limits<double>::infinity()),
      std::nextafter(upper, std::numeric_limits<double>::infinity()));
}

V2Interval V2Multiply(const V2Interval& lhs, const V2Interval& rhs) {
  if (!lhs.valid || !rhs.valid) return V2InvalidInterval();
  if (V2ExactZero(lhs) || V2ExactZero(rhs)) return V2Point(0.0);
  const double values[4] = {
      lhs.lower * rhs.lower, lhs.lower * rhs.upper,
      lhs.upper * rhs.lower, lhs.upper * rhs.upper};
  double lower = values[0];
  double upper = values[0];
  for (const double value : values) {
    if (!Finite(value)) return V2InvalidInterval();
    lower = std::min(lower, value);
    upper = std::max(upper, value);
  }
  if (lhs.lower == lhs.upper && rhs.lower == rhs.upper) {
    const double raw = lhs.lower * rhs.lower;
    if (CompareProductToDoubleV2(std::abs(lhs.lower), std::abs(rhs.lower),
                                 std::abs(raw)) == 0) {
      return V2Point(raw);
    }
  }
  return V2Bounds(
      std::nextafter(lower, -std::numeric_limits<double>::infinity()),
      std::nextafter(upper, std::numeric_limits<double>::infinity()));
}

V2Interval V2DividePositive(const V2Interval& numerator,
                            const V2Interval& denominator) {
  if (!numerator.valid || !denominator.valid ||
      !(denominator.lower > 0.0) || denominator.upper < denominator.lower) {
    return V2InvalidInterval();
  }
  if (V2ExactZero(numerator)) return V2Point(0.0);
  const double values[4] = {
      numerator.lower / denominator.lower,
      numerator.lower / denominator.upper,
      numerator.upper / denominator.lower,
      numerator.upper / denominator.upper};
  double lower = values[0];
  double upper = values[0];
  for (const double value : values) {
    if (!Finite(value)) return V2InvalidInterval();
    lower = std::min(lower, value);
    upper = std::max(upper, value);
  }
  if (numerator.lower == numerator.upper &&
      denominator.lower == denominator.upper) {
    const double raw = numerator.lower / denominator.lower;
    if (CompareRatioToDoubleV2(std::abs(numerator.lower),
                               denominator.lower, std::abs(raw)) == 0) {
      return V2Point(raw);
    }
  }
  return V2Bounds(
      std::nextafter(lower, -std::numeric_limits<double>::infinity()),
      std::nextafter(upper, std::numeric_limits<double>::infinity()));
}

// Return an inward (downward) width for a closed interval.  A zero-width
// interval is exact and therefore remains valid; a nonzero width uses the
// complete directed subtraction so b_pre never overstates certified capacity
// (including opposite-sign subnormal endpoints).
bool WidthDownwardV2(const TubeViabilityInterval& interval,
                     double& result) {
  if (!interval.valid || !Finite(interval.lower) ||
      !Finite(interval.upper) || interval.upper < interval.lower) {
    return false;
  }
  if (interval.upper == interval.lower) {
    result = 0.0;
    return true;
  }
  const V2Interval difference = V2Subtract(
      V2Point(interval.upper), V2Point(interval.lower));
  if (!difference.valid || difference.lower < 0.0) return false;
  result = difference.lower;
  return Finite(result);
}

bool ProductMagnitudeAtMostV2(const double lhs, const double rhs,
                              const double bound) {
  if (!Finite(lhs) || !Finite(rhs) || !Finite(bound) || bound < 0.0) {
    return false;
  }
  if (lhs == 0.0 || rhs == 0.0) return true;
  return CompareProductToDoubleV2(std::abs(lhs), std::abs(rhs), bound) <= 0;
}

// Check an exact endpoint product against a non-negative capability.  The
// directed interval product can sit one ULP outside a representable exact
// boundary; these checks permit clipping that harmless enclosure artifact
// while still rejecting an endpoint whose exact product exceeds the bound.
bool PositiveProductsAtMostV2(const TubeViabilityInterval& slope,
                              const V2Interval& phase,
                              const double bound) {
  const double slope_values[2] = {slope.lower, slope.upper};
  const double phase_values[2] = {phase.lower, phase.upper};
  for (const double slope_value : slope_values) {
    for (const double phase_value : phase_values) {
      if (std::signbit(slope_value) == std::signbit(phase_value) &&
          !ProductMagnitudeAtMostV2(slope_value, phase_value, bound)) {
        return false;
      }
    }
  }
  return true;
}

bool NegativeProductsMagnitudeAtMostV2(const TubeViabilityInterval& slope,
                                       const V2Interval& phase,
                                       const double bound) {
  const double slope_values[2] = {slope.lower, slope.upper};
  const double phase_values[2] = {phase.lower, phase.upper};
  for (const double slope_value : slope_values) {
    for (const double phase_value : phase_values) {
      if (std::signbit(slope_value) != std::signbit(phase_value) &&
          !ProductMagnitudeAtMostV2(slope_value, phase_value, bound)) {
        return false;
      }
    }
  }
  return true;
}

bool InwardLowerDifferenceV2(const double lhs, const double rhs,
                             double& result) {
  if (!Finite(lhs) || !Finite(rhs)) return false;
  const V2Interval difference = V2Subtract(V2Point(lhs), V2Point(rhs));
  if (!difference.valid) return false;
  // The upper endpoint is rounded upward, so the resulting lower bound stays
  // inside the exact backward-reachable set.
  result = difference.upper;
  return Finite(result);
}

bool InwardUpperSumV2(const double lhs, const double rhs, double& result) {
  if (!Finite(lhs) || !Finite(rhs)) return false;
  const V2Interval sum = V2Add(V2Point(lhs), V2Point(rhs));
  if (!sum.valid) return false;
  // The lower endpoint is rounded downward, keeping the upper bound inward.
  result = sum.lower;
  return Finite(result);
}

bool SupportedFloatingPointEnvironmentV2() {
#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && \
                               __FINITE_MATH_ONLY__)
  return false;
#endif
#if defined(__FMA__) || defined(__FP_FAST_FMA) || defined(__FP_FAST_FMAF)
  return false;
#endif
#if defined(__FLT_EVAL_METHOD__) && __FLT_EVAL_METHOD__ != 0
  return false;
#endif
  if (!std::numeric_limits<double>::is_iec559 ||
      std::numeric_limits<double>::radix != 2 ||
      std::numeric_limits<double>::digits != 53 ||
      std::numeric_limits<double>::has_denorm != std::denorm_present ||
      fegetround() != FE_TONEAREST) return false;
#if defined(__i386__) || defined(__x86_64__)
  const unsigned int csr = _mm_getcsr();
#ifdef _MM_FLUSH_ZERO_ON
  if ((csr & _MM_FLUSH_ZERO_ON) != 0U) return false;
#endif
  if ((csr & 0x0040U) != 0U) return false;
  return true;
#else
  return false;
#endif
}

bool InterpolateV2Interval(const TubeViabilityKnot& first,
                           const TubeViabilityKnot& second,
                           const double w,
                           TubeViabilityInterval& interval) {
  interval = TubeViabilityInterval();
  if (!StrictTubeIntervalValidV2(first.reachable) ||
      !StrictTubeIntervalValidV2(second.reachable) ||
      !Finite(first.w) || !Finite(second.w) || !Finite(w) ||
      !(second.w > first.w) || w < first.w || w > second.w) {
    return false;
  }
  if (w == first.w) {
    interval = first.reachable;
    return true;
  }
  if (w == second.w) {
    interval = second.reachable;
    return true;
  }
  const V2Interval span = V2Subtract(V2Point(second.w), V2Point(first.w));
  const V2Interval numerator = V2Subtract(V2Point(w), V2Point(first.w));
  if (!span.valid || !numerator.valid || !(span.lower > 0.0)) return false;
  const V2Interval alpha = V2DividePositive(numerator, span);
  if (!alpha.valid || alpha.lower <= 0.0 || alpha.upper >= 1.0) return false;
  const V2Interval lower_difference = V2Subtract(
      V2Point(second.reachable.lower), V2Point(first.reachable.lower));
  const V2Interval upper_difference = V2Subtract(
      V2Point(second.reachable.upper), V2Point(first.reachable.upper));
  const V2Interval lower_value = V2Add(
      V2Point(first.reachable.lower), V2Multiply(lower_difference, alpha));
  const V2Interval upper_value = V2Add(
      V2Point(first.reachable.upper), V2Multiply(upper_difference, alpha));
  if (!lower_value.valid || !upper_value.valid) return false;
  interval = MakeInterval(lower_value.upper, upper_value.lower);
  return interval.valid;
}

bool QueryEnvelopeV2Strict(const TubeViabilityResult& result,
                           const double w,
                           TubeViabilityInterval& interval) {
  interval = TubeViabilityInterval();
  if (!SupportedFloatingPointEnvironmentV2() ||
      result.proof_kind != TubeViabilityProofKind::V2_LIVE_PWL ||
      !result.valid || !result.feasible || result.knots.size() < 2U ||
      !Finite(w) || w < result.preview_start_w ||
      w > result.preview_end_w) {
    return false;
  }
  if (w == result.knots.front().w) {
    interval = result.knots.front().reachable;
    return StrictTubeIntervalValidV2(interval);
  }
  if (w == result.knots.back().w) {
    interval = result.knots.back().reachable;
    return StrictTubeIntervalValidV2(interval);
  }
  std::size_t index = 0U;
  while (index + 1U < result.knots.size() &&
         result.knots[index + 1U].w < w) {
    ++index;
  }
  if (index + 1U >= result.knots.size()) return false;
  return InterpolateV2Interval(result.knots[index], result.knots[index + 1U],
                               w, interval);
}

const TubeViabilityInterval* SelectSlopeIntervalV2(
    const TubeViabilityKnot& knot, const bool lower,
    const bool prefer_right) {
  if (lower) {
    if (prefer_right && knot.lower_slope_right_interval.valid) {
      return &knot.lower_slope_right_interval;
    }
    if (!prefer_right && knot.lower_slope_left_interval.valid) {
      return &knot.lower_slope_left_interval;
    }
    if (knot.lower_slope_right_interval.valid) {
      return &knot.lower_slope_right_interval;
    }
    if (knot.lower_slope_left_interval.valid) {
      return &knot.lower_slope_left_interval;
    }
  } else {
    if (prefer_right && knot.upper_slope_right_interval.valid) {
      return &knot.upper_slope_right_interval;
    }
    if (!prefer_right && knot.upper_slope_left_interval.valid) {
      return &knot.upper_slope_left_interval;
    }
    if (knot.upper_slope_right_interval.valid) {
      return &knot.upper_slope_right_interval;
    }
    if (knot.upper_slope_left_interval.valid) {
      return &knot.upper_slope_left_interval;
    }
  }
  return nullptr;
}

bool BuildRateIntervalV2Strict(const TubeViabilityKnot& knot,
                               const double delta,
                               const double lower_nu,
                               const double upper_nu,
                               const double upper_u_delta,
                               TubeViabilityRateInterval& output) {
  output = TubeViabilityRateInterval();
  if (!StrictTubeIntervalValidV2(knot.reachable) || !Finite(delta) ||
      !Finite(lower_nu) ||
      !Finite(upper_nu) || lower_nu <= 0.0 || upper_nu < lower_nu ||
      !Finite(upper_u_delta) || upper_u_delta < 0.0 ||
      delta < knot.reachable.lower || delta > knot.reachable.upper) {
    return false;
  }
  output.lower = -upper_u_delta;
  output.upper = upper_u_delta;
  const bool at_lower = delta == knot.reachable.lower;
  const bool at_upper = delta == knot.reachable.upper;
  output.lower_boundary_active = at_lower;
  output.upper_boundary_active = at_upper;
  const V2Interval phase_rate = V2Bounds(lower_nu, upper_nu);
  if (at_lower) {
    const TubeViabilityInterval* slope =
        SelectSlopeIntervalV2(knot, true, true);
    if (slope == nullptr) return false;
    const V2Interval product = V2Multiply(
        V2Bounds(slope->lower, slope->upper), phase_rate);
    if (!product.valid) return false;
    // At the lower boundary u_delta must be at least slope*nu; use the
    // largest directed product to keep this an inner, conservative bound.
    output.lower = std::max(output.lower, product.upper);
    // This scalar is diagnostic only; expose the worst-case endpoint rather
    // than the optimistic lower endpoint of the directed slope interval.
    output.lower_boundary_slope = slope->upper;
    if (output.lower > upper_u_delta &&
        PositiveProductsAtMostV2(*slope, phase_rate, upper_u_delta)) {
      output.lower = upper_u_delta;
    }
  }
  if (at_upper) {
    const TubeViabilityInterval* slope =
        SelectSlopeIntervalV2(knot, false, true);
    if (slope == nullptr) return false;
    const V2Interval product = V2Multiply(
        V2Bounds(slope->lower, slope->upper), phase_rate);
    if (!product.valid) return false;
    // At the upper boundary u_delta must be no greater than slope*nu; use the
    // smallest directed product for a conservative inner upper bound.
    output.upper = std::min(output.upper, product.lower);
    // As above, report the rigorous endpoint for the upper-face constraint.
    output.upper_boundary_slope = slope->lower;
    if (output.upper < -upper_u_delta &&
        NegativeProductsMagnitudeAtMostV2(*slope, phase_rate,
                                          upper_u_delta)) {
      output.upper = -upper_u_delta;
    }
  }
  output.valid = Finite(output.lower) && Finite(output.upper) &&
      output.lower <= output.upper;
  return output.valid;
}

bool QueryRateIntervalV2Strict(const TubeViabilityResult& result,
                               const double w, const double delta,
                               const double phase_rate,
                               TubeViabilityRateInterval& interval) {
  interval = TubeViabilityRateInterval();
  if (!SupportedFloatingPointEnvironmentV2() ||
      result.proof_kind != TubeViabilityProofKind::V2_LIVE_PWL ||
      !result.valid || !result.feasible || result.knots.size() < 2U ||
      !Finite(w) || !Finite(delta) || !Finite(phase_rate) ||
      !result.policy.valid() || phase_rate < result.policy.lower_nu ||
      phase_rate > result.policy.upper_nu || w < result.preview_start_w ||
      w > result.preview_end_w) {
    return false;
  }
  if (w == result.knots.front().w) {
    return BuildRateIntervalV2Strict(
        result.knots.front(), delta, result.policy.lower_nu,
        result.policy.upper_nu, result.upper_u_delta, interval);
  }
  if (w == result.knots.back().w) {
    return BuildRateIntervalV2Strict(
        result.knots.back(), delta, result.policy.lower_nu,
        result.policy.upper_nu, result.upper_u_delta, interval);
  }
  std::size_t index = 0U;
  while (index + 1U < result.knots.size() &&
         result.knots[index + 1U].w < w) {
    ++index;
  }
  if (index + 1U >= result.knots.size()) return false;
  // At an exact internal knot the frozen PWL contract selects the outgoing
  // (right) segment when present.  Do not manufacture a local knot from the
  // preceding cell, which would incorrectly use its left-side slope.
  if (result.knots[index + 1U].w == w) {
    return BuildRateIntervalV2Strict(
        result.knots[index + 1U], delta, result.policy.lower_nu,
        result.policy.upper_nu, result.upper_u_delta, interval);
  }
  TubeViabilityInterval envelope;
  if (!InterpolateV2Interval(result.knots[index], result.knots[index + 1U],
                             w, envelope) || !envelope.contains(delta, 0.0)) {
    return false;
  }
  const TubeViabilityKnot& first = result.knots[index];
  const TubeViabilityKnot& second = result.knots[index + 1U];
  const double span = second.w - first.w;
  if (!Finite(span) || !(span > 0.0)) return false;
  TubeViabilityKnot local = first;
  local.reachable = envelope;
  local.lower_slope_left_valid = false;
  local.upper_slope_left_valid = false;
  local.lower_slope_right = first.lower_slope_right;
  local.upper_slope_right = first.upper_slope_right;
  local.lower_slope_right_valid = first.lower_slope_right_valid;
  local.upper_slope_right_valid = first.upper_slope_right_valid;
  local.lower_slope_right_interval = first.lower_slope_right_interval;
  local.upper_slope_right_interval = first.upper_slope_right_interval;
  return BuildRateIntervalV2Strict(
      local, delta, result.policy.lower_nu, result.policy.upper_nu,
      result.upper_u_delta, interval);
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

namespace {

bool EvaluateV2(const TubeProfileV2& profile,
               const TubeViabilityInput& input,
               TubeViabilityResult& output) {
  output = TubeViabilityResult();
  output.proof_kind = TubeViabilityProofKind::V2_LIVE_PWL;
  output.policy = input.policy;
  output.upper_u_delta = input.upper_u_delta;
  output.provenance.immutable = true;
  output.provenance.path_revision = profile.path_key.path_revision;
  output.provenance.frame_revision = profile.path_key.frame_revision;
  output.provenance.profile_revision = profile.profile_id;
  output.provenance.source_revision = profile.request_id;
  output.provenance.tube_revision = profile.profile_id;
  output.provenance.map_revision = profile.map_capture_key.state_id;
  output.provenance.obstacle_contract_id =
      "phase_offset_navigation/tube_certificate_v2";
  output.provenance.policy_revision = input.policy.policy_revision;
  output.provenance.policy_configuration_identity =
      input.policy.configuration_identity;
  output.provenance.policy_configuration_id = input.policy.configuration_id;
  output.provenance.source =
      "phase_offset_navigation/tube_viability/live-v2-pwl";
  // Preserve the exact immutable authorities on every V2 result.  Scalar
  // revisions below are retained for compatibility with legacy callers, but
  // they are not a substitute for these value-owned keys.
  output.path_key = profile.path_key;
  output.configuration_key = profile.configuration_key;
  output.map_capture_key = profile.map_capture_key;
  output.profile_id = profile.profile_id;
  output.max_work = input.max_work;
  std::size_t work_count = 0U;

  const auto invalid = [&output, &work_count](const std::string& reason) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.valid = false;
    output.feasible = false;
    output.rate_feasible = false;
    output.contraction_rate_feasible = false;
    output.reason = reason;
    output.knots.clear();
    output.work_count = work_count;
    return false;
  };
  const auto stale = [&output](const std::string& reason) {
    output.status = TubeViabilityStatus::STALE;
    output.valid = false;
    output.feasible = false;
    output.rate_feasible = false;
    output.contraction_rate_feasible = false;
    output.reason = reason;
    output.knots.clear();
    return false;
  };

  if (!SupportedFloatingPointEnvironmentV2()) {
    return invalid("V2 preview requires nearest binary64 without FTZ/DAZ/FMA");
  }
  if (input.profile != nullptr) {
    return invalid("V2 preview cannot bind a legacy geometric profile");
  }
  if (input.v2_provenance_bound &&
      (profile.path_key != input.expected_v2_path_key ||
       profile.configuration_key != input.expected_v2_configuration_key ||
       profile.map_capture_key != input.expected_v2_map_capture_key)) {
    return stale("V2 immutable path/configuration/map provenance mismatch");
  }
  if (profile.knots.size() < 2U ||
      profile.knots.size() > kMaximumProfileKnots ||
      profile.cells.size() > kMaximumProfileCells ||
      !profile.structurallyValid() || profile.profile_id == 0U ||
      profile.request_id == 0U) {
    return invalid("immutable V2 profile is incomplete or malformed");
  }
  if (!input.policy.valid() ||
      input.policy.configuration_identity !=
          profile.configuration_key.configuration_id ||
      !Finite(input.current_w) || !Finite(input.current_delta) ||
      !Finite(input.upper_u_delta) || input.upper_u_delta < 0.0 ||
      !Finite(input.boundary_tolerance) || input.boundary_tolerance < 0.0 ||
      input.max_work == 0U) {
    return invalid("V2 preview policy or live values are invalid");
  }
  if ((input.path_revision != 0U &&
       input.path_revision != profile.path_key.path_revision) ||
      (input.frame_revision != 0U &&
       input.frame_revision != profile.path_key.frame_revision) ||
      (input.profile_revision != 0U &&
       input.profile_revision != profile.profile_id) ||
      (input.expected_path_revision != 0U &&
       input.expected_path_revision != profile.path_key.path_revision) ||
      (input.expected_frame_revision != 0U &&
       input.expected_frame_revision != profile.path_key.frame_revision) ||
      (input.expected_profile_revision != 0U &&
       input.expected_profile_revision != profile.profile_id)) {
    return stale("V2 path/frame/profile provenance does not match");
  }
  output.current_w = input.current_w;
  if (input.current_w < profile.certified_start ||
      input.current_w > profile.certified_end) {
    output.current_w_inside = false;
    output.valid = true;
    output.status = TubeViabilityStatus::PREVIEW_INFEASIBLE;
    output.reason = "live W is outside the immutable certified range";
    return true;
  }
  output.current_w_inside = true;
  if (!Finite(profile.certified_start) || !Finite(profile.certified_end) ||
      !(profile.certified_end > profile.certified_start)) {
    return invalid("immutable V2 certified range is invalid");
  }
  double horizon_end = 0.0;
  if (!SumCeilV2(input.current_w, input.policy.preview_horizon_w,
                 horizon_end)) {
    return invalid("live V2 horizon endpoint is nonfinite");
  }
  output.preview_start_w = input.current_w;
  output.preview_end_w = horizon_end;
  double effective_horizon = 0.0;
  if (!DifferenceDownwardV2(horizon_end, input.current_w,
                            effective_horizon)) {
    return invalid("live V2 horizon width is nonfinite");
  }
  output.effective_horizon_w = effective_horizon;
  if (!Finite(horizon_end) || horizon_end <= input.current_w ||
      horizon_end > profile.certified_end) {
    output.preview_truncated_after = horizon_end > profile.certified_end;
    output.valid = true;
    output.feasible = false;
    output.status = TubeViabilityStatus::PREVIEW_INFEASIBLE;
    output.reason = "full normal W horizon is not covered by the profile";
    return true;
  }

  std::vector<double> nodes;
  std::string reason;
  const std::size_t node_budget = std::min(
      kMaximumSamples, input.max_work);
  if (!BuildMergedNodesV2(profile, input.current_w, horizon_end,
                          input.policy.sample_spacing_w, node_budget, nodes,
                          reason)) {
    return invalid(reason);
  }
  if (nodes.size() < 2U || nodes.size() > kMaximumSamples) {
    return invalid("V2 merged partition is outside finite bounds");
  }
  if (!ConsumeWorkV2(nodes.size(), input.max_work, work_count)) {
    return invalid("V2 live preview work budget is exhausted");
  }
  output.preview_end_w = horizon_end;
  output.effective_horizon_w = effective_horizon;
  if (!DifferenceDownwardV2(nodes[1U], nodes[0U], output.delta_w)) {
    return invalid("V2 merged partition has a nonpositive first cell");
  }

  double delta_rate = 0.0;
  if (!RatioFloorV2(input.upper_u_delta, input.policy.upper_nu,
                    delta_rate)) {
    return invalid("V2 reachable-rate ratio is nonfinite");
  }
  output.allowed_inward_contraction_slope = delta_rate;
  output.knots.resize(nodes.size());
  for (std::size_t i = 0U; i < nodes.size(); ++i) {
    if (!ConsumeWorkV2(1U, input.max_work, work_count)) {
      return invalid("V2 live preview work budget is exhausted");
    }
    TubeViabilityKnot& knot = output.knots[i];
    knot.w = nodes[i];
    double lower = 0.0;
    double upper = 0.0;
    if (!profile.evaluate(nodes[i], lower, upper)) {
      return invalid("V2 profile PWL evaluation failed on merged partition");
    }
    knot.geometric = MakeInterval(lower, upper);
    if (!knot.geometric.valid) {
      return invalid("V2 geometric interval is invalid");
    }
  }

  output.knots.back().reachable = output.knots.back().geometric;
  output.delta_delta = 0.0;
  for (std::size_t i = nodes.size() - 1U; i-- > 0U;) {
    if (!ConsumeWorkV2(1U, input.max_work, work_count)) {
      return invalid("V2 live preview work budget is exhausted");
    }
    double span = 0.0;
    if (!DifferenceDownwardV2(nodes[i + 1U], nodes[i], span)) {
      return invalid("V2 merged cell width is invalid");
    }
    double reach = 0.0;
    if (!ProductFloorV2(delta_rate, span, reach)) {
      return invalid("V2 per-cell reachable width is nonfinite");
    }
    if (i == 0U) output.delta_delta = reach;
    const TubeViabilityInterval& geometric = output.knots[i].geometric;
    const TubeViabilityInterval& successor = output.knots[i + 1U].reachable;
    if (!successor.valid) {
      output.knots[i].reachable = TubeViabilityInterval();
      continue;
    }
    double lower = 0.0;
    double upper = 0.0;
    if (!InwardLowerDifferenceV2(successor.lower, reach, lower) ||
        !InwardUpperSumV2(successor.upper, reach, upper)) {
      return invalid("V2 inward recurrence arithmetic failed");
    }
    output.knots[i].reachable = MakeInterval(
        std::max(geometric.lower, lower),
        std::min(geometric.upper, upper));
  }
  output.delta_reach = MakeInterval(-output.delta_delta,
                                    output.delta_delta);

  output.feasible = true;
  for (TubeViabilityKnot& knot : output.knots) {
    if (!knot.reachable.valid ||
        knot.reachable.lower < knot.geometric.lower ||
        knot.reachable.upper > knot.geometric.upper) {
      output.feasible = false;
    }
  }
  if (!output.feasible) {
    output.valid = true;
    output.status = TubeViabilityStatus::PREVIEW_INFEASIBLE;
    output.reason = "inward V2 reachable envelope is empty or outside I";
    return true;
  }

  // Populate both one-sided slope values on the exact merged cells.  Unlike
  // the legacy path, no minimum-spacing tolerance is allowed to erase a
  // near-equal geometric knot.
  for (std::size_t i = 0U; i < output.knots.size(); ++i) {
    if (i > 0U) {
      if (!ConsumeWorkV2(2U, input.max_work, work_count)) {
        return invalid("V2 live preview work budget is exhausted");
      }
      // Slope division needs the complete directed interval for the exact
      // endpoint subtraction.  A downward scalar span is appropriate for
      // reachability recursion, but using it as a singleton denominator here
      // could silently understate a signed slope.
      const V2Interval span_interval = V2Subtract(
          V2Point(output.knots[i].w),
          V2Point(output.knots[i - 1U].w));
      if (!span_interval.valid || !(span_interval.lower > 0.0)) {
        return invalid("V2 left-cell slope span is invalid");
      }
      const V2Interval lower_difference = V2Subtract(
          V2Point(output.knots[i].reachable.lower),
          V2Point(output.knots[i - 1U].reachable.lower));
      const V2Interval upper_difference = V2Subtract(
          V2Point(output.knots[i].reachable.upper),
          V2Point(output.knots[i - 1U].reachable.upper));
      const V2Interval lower_slope = V2DividePositive(
          lower_difference, span_interval);
      const V2Interval upper_slope = V2DividePositive(
          upper_difference, span_interval);
      if (!lower_slope.valid || !upper_slope.valid) {
        return invalid("V2 left-cell slope ratio is invalid");
      }
      output.knots[i].lower_slope_left_interval = MakeInterval(
          lower_slope.lower, lower_slope.upper);
      output.knots[i].upper_slope_left_interval = MakeInterval(
          upper_slope.lower, upper_slope.upper);
      output.knots[i].lower_slope_left = lower_slope.lower;
      output.knots[i].upper_slope_left = upper_slope.upper;
      output.knots[i].lower_slope_left_valid =
          output.knots[i].lower_slope_left_interval.valid;
      output.knots[i].upper_slope_left_valid =
          output.knots[i].upper_slope_left_interval.valid;
    }
    if (i + 1U < output.knots.size()) {
      if (!ConsumeWorkV2(2U, input.max_work, work_count)) {
        return invalid("V2 live preview work budget is exhausted");
      }
      const V2Interval span_interval = V2Subtract(
          V2Point(output.knots[i + 1U].w), V2Point(output.knots[i].w));
      if (!span_interval.valid || !(span_interval.lower > 0.0)) {
        return invalid("V2 right-cell slope span is invalid");
      }
      const V2Interval lower_difference = V2Subtract(
          V2Point(output.knots[i + 1U].reachable.lower),
          V2Point(output.knots[i].reachable.lower));
      const V2Interval upper_difference = V2Subtract(
          V2Point(output.knots[i + 1U].reachable.upper),
          V2Point(output.knots[i].reachable.upper));
      const V2Interval lower_slope = V2DividePositive(
          lower_difference, span_interval);
      const V2Interval upper_slope = V2DividePositive(
          upper_difference, span_interval);
      if (!lower_slope.valid || !upper_slope.valid) {
        return invalid("V2 right-cell slope ratio is invalid");
      }
      output.knots[i].lower_slope_right_interval = MakeInterval(
          lower_slope.lower, lower_slope.upper);
      output.knots[i].upper_slope_right_interval = MakeInterval(
          upper_slope.lower, upper_slope.upper);
      output.knots[i].lower_slope_right = lower_slope.lower;
      output.knots[i].upper_slope_right = upper_slope.upper;
      output.knots[i].lower_slope_right_valid =
          output.knots[i].lower_slope_right_interval.valid;
      output.knots[i].upper_slope_right_valid =
          output.knots[i].upper_slope_right_interval.valid;
    }
  }
  output.max_inward_contraction_slope = 0.0;
  output.contraction_rate_feasible = true;
  for (const TubeViabilityKnot& knot : output.knots) {
    // The lower face moves inward only when its slope is positive.  The
    // directed upper endpoint is the rigorous worst case for that test.
    const TubeViabilityInterval* lower_slopes[] = {
        &knot.lower_slope_left_interval, &knot.lower_slope_right_interval};
    for (const TubeViabilityInterval* slope : lower_slopes) {
      if (!slope->valid) continue;
      if (slope->upper > delta_rate) {
        output.contraction_rate_feasible = false;
      }
      output.max_inward_contraction_slope = std::max(
          output.max_inward_contraction_slope, std::max(0.0, slope->upper));
    }
    // The upper face moves inward only when its slope is negative.  The
    // directed lower endpoint is the rigorous worst case for that test.
    const TubeViabilityInterval* upper_slopes[] = {
        &knot.upper_slope_left_interval, &knot.upper_slope_right_interval};
    for (const TubeViabilityInterval* slope : upper_slopes) {
      if (!slope->valid) continue;
      if (slope->lower < -delta_rate) {
        output.contraction_rate_feasible = false;
      }
      output.max_inward_contraction_slope = std::max(
          output.max_inward_contraction_slope,
          std::max(0.0, -slope->lower));
    }
  }

  output.b_pre = std::numeric_limits<double>::infinity();
  for (const TubeViabilityKnot& knot : output.knots) {
    if (!ConsumeWorkV2(1U, input.max_work, work_count)) {
      return invalid("V2 live preview work budget is exhausted");
    }
    double width = 0.0;
    if (!WidthDownwardV2(knot.reachable, width)) {
      return invalid("V2 reachable knot width is invalid");
    }
    output.b_pre = std::min(output.b_pre, width);
  }
  if (!Finite(output.b_pre)) return invalid("V2 b_pre is nonfinite");
  output.beta_argument = (output.b_pre - input.policy.b_tight) /
      (input.policy.b_open - input.policy.b_tight);
  output.beta = TubeViability::smoothstep(output.beta_argument);
  output.beta_i = output.beta;
  if (!Finite(output.beta)) return invalid("V2 beta is nonfinite");

  for (TubeViabilityKnot& knot : output.knots) {
    if (!ConsumeWorkV2(2U, input.max_work, work_count)) {
      return invalid("V2 live preview work budget is exhausted");
    }
    const bool lower_rate_ok = BuildRateIntervalV2Strict(
        knot, knot.reachable.lower, input.policy.lower_nu,
        input.policy.upper_nu, input.upper_u_delta,
        knot.lower_boundary_rate);
    const bool upper_rate_ok = BuildRateIntervalV2Strict(
        knot, knot.reachable.upper, input.policy.lower_nu,
        input.policy.upper_nu, input.upper_u_delta,
        knot.upper_boundary_rate);
    // These are diagnostics for hypothetical states at each knot.  They do
    // not veto the finite-horizon preview: only the actual current state's
    // rate and the per-cell inward-contraction proof determine rate_feasible.
    (void)lower_rate_ok;
    (void)upper_rate_ok;
  }
  const TubeViabilityKnot& first = output.knots.front();
  output.current_delta_inside = first.reachable.contains(
      input.current_delta, 0.0);
  output.rate_feasible = output.contraction_rate_feasible;
  if (output.current_delta_inside) {
    const bool current_rate_ok = BuildRateIntervalV2Strict(
        first, input.current_delta, input.policy.lower_nu,
        input.policy.upper_nu, input.upper_u_delta,
        output.current_rate_interval);
    output.rate_feasible = output.rate_feasible && current_rate_ok;
  } else {
    output.rate_feasible = false;
  }
  output.valid = true;
  if (!output.contraction_rate_feasible || !output.rate_feasible) {
    output.status = output.current_delta_inside
        ? TubeViabilityStatus::RATE_INFEASIBLE
        : TubeViabilityStatus::CURRENT_DELTA_OUTSIDE;
    output.reason = output.current_delta_inside
        ? "V2 reachable boundary contraction/rate is infeasible"
        : "current delta is outside live V2 K";
  } else {
    output.status = TubeViabilityStatus::FEASIBLE;
    output.reason = "live V2 preview is feasible";
  }
  output.work_count = work_count;
  return true;
}

}  // namespace

bool TubeViability::evaluate(const TubeProfileV2& profile,
                             const TubeViabilityInput& input,
                             TubeViabilityResult& output) {
  return EvaluateV2(profile, input, output);
}

bool TubeViability::evaluate(const TubeProfileV2& profile,
                             const double current_w,
                             const double current_delta,
                             const NormalPreviewProductionPolicy& policy,
                             const double upper_u_delta,
                             TubeViabilityResult& output) {
  TubeViabilityInput input;
  input.current_w = current_w;
  input.current_delta = current_delta;
  input.policy = policy;
  input.upper_u_delta = upper_u_delta;
  return EvaluateV2(profile, input, output);
}

bool TubeViability::queryEnvelope(const TubeViabilityResult& result,
                                  const double w,
                                  TubeViabilityInterval& interval) {
  if (result.proof_kind == TubeViabilityProofKind::V2_LIVE_PWL) {
    return QueryEnvelopeV2Strict(result, w, interval);
  }
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
  if (result.proof_kind == TubeViabilityProofKind::V2_LIVE_PWL) {
    return QueryRateIntervalV2Strict(result, w, delta, phase_rate, interval);
  }
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
