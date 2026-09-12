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

// A read-only view over either immutable PWL producer.  The strict viability
// core only needs ordered W knots and an interval evaluation callback; it does
// not copy a V2 profile or synthesize one for Section input.
struct StrictPwlSourceView {
  const TubeProfileV2* v2_profile = nullptr;
  const SectionTubeProfile* section_profile = nullptr;

  std::size_t knotCount() const {
    if (v2_profile != nullptr) return v2_profile->knots.size();
    return section_profile == nullptr ? 0U : section_profile->knots.size();
  }

  bool knotAt(const std::size_t index, double& w, double& lower,
             double& upper) const {
    if (v2_profile != nullptr) {
      if (index >= v2_profile->knots.size()) return false;
      const TubePwlKnotV2& knot = v2_profile->knots[index];
      w = knot.w;
      lower = knot.lower;
      upper = knot.upper;
      return true;
    }
    if (section_profile == nullptr || index >= section_profile->knots.size()) {
      return false;
    }
    const SectionTubeKnot& knot = section_profile->knots[index];
    w = knot.w;
    lower = knot.lower;
    upper = knot.upper;
    return true;
  }
};

std::size_t SourceLowerBound(const StrictPwlSourceView& source,
                             const double value) {
  std::size_t first = 0U;
  std::size_t last = source.knotCount();
  while (first < last) {
    const std::size_t middle = first + (last - first) / 2U;
    double w = 0.0;
    double lower = 0.0;
    double upper = 0.0;
    if (!source.knotAt(middle, w, lower, upper) || !(w >= value)) {
      first = middle + 1U;
    } else {
      last = middle;
    }
  }
  return first;
}

std::size_t SourceUpperBound(const StrictPwlSourceView& source,
                             const double value) {
  std::size_t first = 0U;
  std::size_t last = source.knotCount();
  while (first < last) {
    const std::size_t middle = first + (last - first) / 2U;
    double w = 0.0;
    double lower = 0.0;
    double upper = 0.0;
    if (!source.knotAt(middle, w, lower, upper) || !(w > value)) {
      first = middle + 1U;
    } else {
      last = middle;
    }
  }
  return first;
}

bool BuildMergedNodesStrict(const StrictPwlSourceView& source,
                            const double current_w, const double horizon_w,
                            const double spacing_w,
                            const std::size_t max_nodes,
                            std::vector<double>& nodes, std::string& reason) {
  nodes.clear();
  if (source.knotCount() < 2U || max_nodes < 2U ||
      source.knotCount() > kMaximumProfileKnots) {
    reason = "immutable PWL profile knot count is out of bounds";
    return false;
  }
  double horizon = 0.0;
  if (!DifferenceDownwardV2(horizon_w, current_w, horizon)) {
    reason = "live PWL horizon width is not representable";
    return false;
  }
  double ratio = 0.0;
  if (!Finite(horizon) || horizon <= 0.0 ||
      !RatioCeilV2(horizon, spacing_w, ratio) || !Finite(ratio) ||
      ratio <= 0.0 || ratio > static_cast<double>(max_nodes - 1U)) {
    reason = "live PWL sample-grid ratio is out of bounds";
    return false;
  }
  const double rounded_count = std::ceil(ratio);
  if (!Finite(rounded_count) || rounded_count < 1.0 ||
      rounded_count > static_cast<double>(max_nodes - 1U)) {
    reason = "live PWL sample-grid count is out of bounds";
    return false;
  }
  const std::size_t sample_count = static_cast<std::size_t>(rounded_count);
  const std::size_t first_ge = SourceLowerBound(source, current_w);
  const std::size_t first_gt = SourceUpperBound(source, horizon_w);
  if (first_ge > first_gt || first_gt > source.knotCount()) {
    reason = "live PWL source partition bounds are malformed";
    return false;
  }
  const std::size_t geometric_count = first_gt - first_ge;
  if (geometric_count > std::numeric_limits<std::size_t>::max() -
          sample_count - 2U) {
    reason = "live PWL partition size overflow";
    return false;
  }
  const std::size_t raw_bound = geometric_count + sample_count + 2U;
  if (raw_bound > max_nodes) {
    reason = "live PWL merged partition exceeds finite bound";
    return false;
  }
  nodes.reserve(raw_bound);
  nodes.push_back(current_w);
  nodes.push_back(horizon_w);
  for (std::size_t index = first_ge; index < first_gt; ++index) {
    double knot_w = 0.0;
    double lower = 0.0;
    double upper = 0.0;
    if (!source.knotAt(index, knot_w, lower, upper) || !Finite(knot_w)) {
      reason = "live PWL source knot is malformed";
      return false;
    }
    nodes.push_back(knot_w);
  }
  for (std::size_t k = 1U; k <= sample_count; ++k) {
    double offset = 0.0;
    if (!ProductFloorV2(static_cast<double>(k), spacing_w, offset)) {
      reason = "live PWL sample-grid product overflow";
      return false;
    }
    double candidate = 0.0;
    if (!SumFloorV2(current_w, offset, candidate)) {
      reason = "live PWL sample-grid sum is nonfinite";
      return false;
    }
    if (!Finite(candidate) || !(candidate > current_w)) {
      if (!Finite(candidate)) {
        reason = "live PWL sample-grid endpoint is nonfinite";
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
    reason = "live PWL merged partition is malformed";
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

bool InterpolateStrictBounds(const double first_w, const double first_lower,
                             const double first_upper, const double second_w,
                             const double second_lower,
                             const double second_upper, const double w,
                             TubeViabilityInterval& interval) {
  interval = TubeViabilityInterval();
  const TubeViabilityInterval first =
      MakeInterval(first_lower, first_upper);
  const TubeViabilityInterval second =
      MakeInterval(second_lower, second_upper);
  if (!StrictTubeIntervalValidV2(first) ||
      !StrictTubeIntervalValidV2(second) || !Finite(first_w) ||
      !Finite(second_w) || !Finite(w) || !(second_w > first_w) ||
      w < first_w || w > second_w) {
    return false;
  }
  if (w == first_w) {
    interval = first;
    return true;
  }
  if (w == second_w) {
    interval = second;
    return true;
  }
  const V2Interval span = V2Subtract(V2Point(second_w), V2Point(first_w));
  const V2Interval numerator = V2Subtract(V2Point(w), V2Point(first_w));
  if (!span.valid || !numerator.valid || !(span.lower > 0.0)) return false;
  const V2Interval alpha = V2DividePositive(numerator, span);
  if (!alpha.valid || alpha.lower <= 0.0 || alpha.upper >= 1.0) return false;
  const V2Interval lower_difference = V2Subtract(
      V2Point(second.lower), V2Point(first.lower));
  const V2Interval upper_difference = V2Subtract(
      V2Point(second.upper), V2Point(first.upper));
  const V2Interval lower_value = V2Add(
      V2Point(first.lower), V2Multiply(lower_difference, alpha));
  const V2Interval upper_value = V2Add(
      V2Point(first.upper), V2Multiply(upper_difference, alpha));
  if (!lower_value.valid || !upper_value.valid) return false;
  interval = MakeInterval(lower_value.upper, upper_value.lower);
  return interval.valid;
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
  return InterpolateStrictBounds(
      first.w, first.reachable.lower, first.reachable.upper, second.w,
      second.reachable.lower, second.reachable.upper, w, interval);
}

bool StrictPwlProof(const TubeViabilityProofKind proof_kind) {
  return proof_kind == TubeViabilityProofKind::V2_LIVE_PWL ||
      proof_kind == TubeViabilityProofKind::SECTION_PWL;
}

bool StrictPolicyValid(const TubeViabilityResult& result) {
  return result.proof_kind == TubeViabilityProofKind::SECTION_PWL
      ? result.policy.numericValid()
      : result.policy.valid();
}

bool QueryEnvelopeV2Strict(const TubeViabilityResult& result,
                           const double w,
                           TubeViabilityInterval& interval) {
  interval = TubeViabilityInterval();
  if (!SupportedFloatingPointEnvironmentV2() ||
      !StrictPwlProof(result.proof_kind) ||
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
      !StrictPwlProof(result.proof_kind) ||
      !result.valid || !result.feasible || result.knots.size() < 2U ||
      !Finite(w) || !Finite(delta) || !Finite(phase_rate) ||
      !StrictPolicyValid(result) || phase_rate < result.policy.lower_nu ||
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

bool NormalPreviewProductionPolicy::numericValid() const {
  return immutable && Finite(preview_horizon_w) && preview_horizon_w > 0.0 &&
      Finite(sample_spacing_w) && sample_spacing_w > 0.0 &&
      Finite(lower_nu) && lower_nu > 0.0 && Finite(upper_nu) &&
      upper_nu >= lower_nu && Finite(b_tight) && b_tight >= 0.0 &&
      Finite(b_open) && b_open > b_tight;
}

bool NormalPreviewProductionPolicy::valid() const {
  return numericValid() && policy_revision != 0U &&
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

bool EvaluateStrictSourceAt(const StrictPwlSourceView& source,
                            const double w, double& lower, double& upper) {
  lower = 0.0;
  upper = 0.0;
  const std::size_t count = source.knotCount();
  if (count < 2U || !Finite(w)) return false;
  double first_w = 0.0;
  double first_lower = 0.0;
  double first_upper = 0.0;
  double last_w = 0.0;
  double last_lower = 0.0;
  double last_upper = 0.0;
  if (!source.knotAt(0U, first_w, first_lower, first_upper) ||
      !source.knotAt(count - 1U, last_w, last_lower, last_upper) ||
      w < first_w || w > last_w) {
    return false;
  }
  if (w == first_w) {
    lower = first_lower;
    upper = first_upper;
    return Finite(lower) && Finite(upper) && lower <= upper;
  }
  if (w == last_w) {
    lower = last_lower;
    upper = last_upper;
    return Finite(lower) && Finite(upper) && lower <= upper;
  }
  if (source.v2_profile != nullptr) {
    // Keep the frozen V2 profile's established strict evaluator exactly as
    // the old entry point used it.  The merged partition/recurrence below is
    // shared with Section PWL, while this callback preserves V2 bit patterns.
    return source.v2_profile->evaluate(w, lower, upper);
  }
  const std::size_t right = SourceUpperBound(source, w);
  if (right == 0U || right >= count) return false;
  double left_w = 0.0;
  double left_lower = 0.0;
  double left_upper = 0.0;
  double right_w = 0.0;
  double right_lower = 0.0;
  double right_upper = 0.0;
  if (!source.knotAt(right - 1U, left_w, left_lower, left_upper) ||
      !source.knotAt(right, right_w, right_lower, right_upper)) return false;
  TubeViabilityInterval interval;
  if (!InterpolateStrictBounds(left_w, left_lower, left_upper, right_w,
                               right_lower, right_upper, w, interval) ||
      !interval.valid || interval.lower > 0.0 || interval.upper < 0.0) {
    return false;
  }
  lower = interval.lower;
  upper = interval.upper;
  return true;
}

bool EvaluateStrictPwlCore(const StrictPwlSourceView& source,
                           const TubeViabilityInput& input,
                           TubeViabilityResult& output,
                           const double profile_end,
                           const bool allow_terminal_truncate,
                           const char* label,
                           const std::size_t initial_work = 0U) {
  output.current_w = input.current_w;
  output.max_work = input.max_work;
  std::size_t work_count = initial_work;
  const auto clear_borrowed = [&output]() {
    output.section_profile = nullptr;
    output.evaluated_delta = 0.0;
  };
  const auto invalid = [&output, &work_count, &clear_borrowed](
      const std::string& reason) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.valid = false;
    output.feasible = false;
    output.rate_feasible = false;
    output.contraction_rate_feasible = false;
    output.reason = reason;
    output.knots.clear();
    clear_borrowed();
    output.work_count = work_count;
    return false;
  };
  const auto preview_infeasible = [&output, &work_count, &clear_borrowed](
      const std::string& reason) {
    output.valid = true;
    output.feasible = false;
    output.rate_feasible = false;
    output.contraction_rate_feasible = false;
    output.status = TubeViabilityStatus::PREVIEW_INFEASIBLE;
    output.reason = reason;
    // Preserve the historical V2 diagnostic knots on a geometrically
    // infeasible preview; Section failures must not retain borrowable
    // geometry, so that branch is cleared before returning.
    if (output.proof_kind == TubeViabilityProofKind::SECTION_PWL) {
      output.knots.clear();
    }
    clear_borrowed();
    output.work_count = work_count;
    return true;
  };

  if (!Finite(profile_end)) {
    return invalid(std::string(label) + " profile endpoint is nonfinite");
  }
  double horizon_end = 0.0;
  if (!SumCeilV2(input.current_w, input.policy.preview_horizon_w,
                 horizon_end)) {
    return invalid(std::string(label) + " horizon endpoint is nonfinite");
  }
  output.preview_start_w = input.current_w;
  output.preview_end_w = horizon_end;
  double effective_horizon = 0.0;
  if (!DifferenceDownwardV2(horizon_end, input.current_w,
                            effective_horizon)) {
    return invalid(std::string(label) + " horizon width is nonfinite");
  }
  output.effective_horizon_w = effective_horizon;
  if (!Finite(horizon_end) || horizon_end <= input.current_w) {
    return preview_infeasible(std::string(label) +
                              " has no positive preview horizon");
  }
  if (horizon_end > profile_end) {
    output.preview_truncated_after = true;
    if (!allow_terminal_truncate || !(profile_end > input.current_w)) {
      return preview_infeasible(std::string(label) +
                                " full normal W horizon is not covered");
    }
    horizon_end = profile_end;
    output.preview_end_w = horizon_end;
    if (!DifferenceDownwardV2(horizon_end, input.current_w,
                              effective_horizon) ||
        !(effective_horizon > 0.0)) {
      return preview_infeasible(std::string(label) +
                                " terminal endpoint has no positive preview");
    }
    output.effective_horizon_w = effective_horizon;
  }

  const std::size_t node_budget = std::min(kMaximumSamples, input.max_work);
  std::vector<double> nodes;
  std::string reason;
  if (!BuildMergedNodesStrict(source, input.current_w, horizon_end,
                              input.policy.sample_spacing_w, node_budget,
                              nodes, reason)) {
    return invalid(reason);
  }
  if (nodes.size() < 2U || nodes.size() > kMaximumSamples) {
    return invalid(std::string(label) + " merged partition is outside bounds");
  }
  if (!ConsumeWorkV2(nodes.size(), input.max_work, work_count)) {
    return invalid(std::string(label) + " live preview work budget is exhausted");
  }
  output.preview_end_w = horizon_end;
  output.effective_horizon_w = effective_horizon;
  if (!DifferenceDownwardV2(nodes[1U], nodes[0U], output.delta_w)) {
    return invalid(std::string(label) + " merged partition has a bad first cell");
  }

  double delta_rate = 0.0;
  if (!RatioFloorV2(input.upper_u_delta, input.policy.upper_nu, delta_rate)) {
    return invalid(std::string(label) + " reachable-rate ratio is nonfinite");
  }
  output.allowed_inward_contraction_slope = delta_rate;
  output.knots.resize(nodes.size());
  for (std::size_t i = 0U; i < nodes.size(); ++i) {
    if (!ConsumeWorkV2(1U, input.max_work, work_count)) {
      return invalid(std::string(label) + " live preview work budget is exhausted");
    }
    TubeViabilityKnot& knot = output.knots[i];
    knot.w = nodes[i];
    double lower = 0.0;
    double upper = 0.0;
    if (!EvaluateStrictSourceAt(source, nodes[i], lower, upper)) {
      return invalid(std::string(label) + " PWL evaluation failed on merged partition");
    }
    knot.geometric = MakeInterval(lower, upper);
    if (!knot.geometric.valid) {
      return invalid(std::string(label) + " geometric interval is invalid");
    }
  }

  output.knots.back().reachable = output.knots.back().geometric;
  output.delta_delta = 0.0;
  for (std::size_t i = nodes.size() - 1U; i-- > 0U;) {
    if (!ConsumeWorkV2(1U, input.max_work, work_count)) {
      return invalid(std::string(label) + " live preview work budget is exhausted");
    }
    double span = 0.0;
    if (!DifferenceDownwardV2(nodes[i + 1U], nodes[i], span)) {
      return invalid(std::string(label) + " merged cell width is invalid");
    }
    double reach = 0.0;
    if (!ProductFloorV2(delta_rate, span, reach)) {
      return invalid(std::string(label) + " per-cell reachable width is nonfinite");
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
      return invalid(std::string(label) + " inward recurrence arithmetic failed");
    }
    output.knots[i].reachable = MakeInterval(
        std::max(geometric.lower, lower),
        std::min(geometric.upper, upper));
  }
  output.delta_reach = MakeInterval(-output.delta_delta, output.delta_delta);

  output.feasible = true;
  for (TubeViabilityKnot& knot : output.knots) {
    if (!knot.reachable.valid ||
        knot.reachable.lower < knot.geometric.lower ||
        knot.reachable.upper > knot.geometric.upper) {
      output.feasible = false;
    }
  }
  if (!output.feasible) {
    return preview_infeasible(std::string(label) +
                              " reachable envelope is empty or outside I");
  }

  // Directed one-sided slope intervals on every merged cell.  This is shared
  // verbatim by V2 and Section PWL; the work accounting remains per endpoint
  // ratio as in the frozen V2 implementation.
  for (std::size_t i = 0U; i < output.knots.size(); ++i) {
    if (i > 0U) {
      if (!ConsumeWorkV2(2U, input.max_work, work_count)) {
        return invalid(std::string(label) + " live preview work budget is exhausted");
      }
      const V2Interval span_interval = V2Subtract(
          V2Point(output.knots[i].w), V2Point(output.knots[i - 1U].w));
      if (!span_interval.valid || !(span_interval.lower > 0.0)) {
        return invalid(std::string(label) + " left-cell slope span is invalid");
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
        return invalid(std::string(label) + " left-cell slope ratio is invalid");
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
        return invalid(std::string(label) + " live preview work budget is exhausted");
      }
      const V2Interval span_interval = V2Subtract(
          V2Point(output.knots[i + 1U].w), V2Point(output.knots[i].w));
      if (!span_interval.valid || !(span_interval.lower > 0.0)) {
        return invalid(std::string(label) + " right-cell slope span is invalid");
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
        return invalid(std::string(label) + " right-cell slope ratio is invalid");
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
  // `delta_rate` is a floored ratio (`RatioFloorV2(upper_u_delta, upper_nu)`)
  // and the reachable envelope is constructed to contract at or below it.  The
  // boundary slope recovered below is an outward-rounded interval, so a
  // corridor whose construction saturates the rate window reads one or two
  // ULPs *above* the floored ratio even though its exact slope is within the
  // window.  The geometric evaluator has always absorbed this rounding with
  // `boundary_tolerance`; the strict PWL core inherited the construction but
  // not the slack, which spuriously reported RATE_INFEASIBLE for corridors that
  // are exactly at the limit (a 1-ULP ceiling-vs-floor comparison).  Compare
  // against a limit that carries both the caller's tolerance and a few ULPs of
  // the ratio itself; the slack is ~1e-16, far below any physical scale.
  const double contraction_limit =
      delta_rate + std::max(input.boundary_tolerance,
                            16.0 * std::numeric_limits<double>::epsilon() *
                                std::abs(delta_rate));
  for (const TubeViabilityKnot& knot : output.knots) {
    const TubeViabilityInterval* lower_slopes[] = {
        &knot.lower_slope_left_interval, &knot.lower_slope_right_interval};
    for (const TubeViabilityInterval* slope : lower_slopes) {
      if (!slope->valid) continue;
      if (slope->upper > contraction_limit) {
        output.contraction_rate_feasible = false;
      }
      output.max_inward_contraction_slope = std::max(
          output.max_inward_contraction_slope, std::max(0.0, slope->upper));
    }
    const TubeViabilityInterval* upper_slopes[] = {
        &knot.upper_slope_left_interval, &knot.upper_slope_right_interval};
    for (const TubeViabilityInterval* slope : upper_slopes) {
      if (!slope->valid) continue;
      if (slope->lower < -contraction_limit) {
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
      return invalid(std::string(label) + " live preview work budget is exhausted");
    }
    double width = 0.0;
    if (!WidthDownwardV2(knot.reachable, width)) {
      return invalid(std::string(label) + " reachable knot width is invalid");
    }
    output.b_pre = std::min(output.b_pre, width);
  }
  if (!Finite(output.b_pre)) return invalid(std::string(label) + " b_pre is nonfinite");
  output.beta_argument = (output.b_pre - input.policy.b_tight) /
      (input.policy.b_open - input.policy.b_tight);
  output.beta = TubeViability::smoothstep(output.beta_argument);
  output.beta_i = output.beta;
  if (!Finite(output.beta)) return invalid(std::string(label) + " beta is nonfinite");

  for (TubeViabilityKnot& knot : output.knots) {
    if (!ConsumeWorkV2(2U, input.max_work, work_count)) {
      return invalid(std::string(label) + " live preview work budget is exhausted");
    }
    const bool lower_rate_ok = BuildRateIntervalV2Strict(
        knot, knot.reachable.lower, input.policy.lower_nu,
        input.policy.upper_nu, input.upper_u_delta, knot.lower_boundary_rate);
    const bool upper_rate_ok = BuildRateIntervalV2Strict(
        knot, knot.reachable.upper, input.policy.lower_nu,
        input.policy.upper_nu, input.upper_u_delta, knot.upper_boundary_rate);
    (void)lower_rate_ok;
    (void)upper_rate_ok;
  }
  const TubeViabilityKnot& first = output.knots.front();
  output.current_delta_inside = first.reachable.contains(input.current_delta, 0.0);
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
        ? std::string(label) + " reachable boundary contraction/rate is infeasible"
        : std::string("current delta is outside ") + label + " K";
    clear_borrowed();
  } else {
    output.status = TubeViabilityStatus::FEASIBLE;
    output.reason = std::string(label) + " preview is feasible";
  }
  output.work_count = work_count;
  return true;
}

bool ValidateSectionProfile(const SectionTubeProfile& profile,
                            const std::size_t max_work,
                            std::size_t& work_count, std::string& reason) {
  work_count = 0U;
  if (!profile.usable) {
    reason = "Section PWL profile is not usable";
    return false;
  }
  if (!Finite(profile.valid_start) || !Finite(profile.valid_end) ||
      !(profile.valid_end > profile.valid_start) || profile.knots.size() < 2U ||
      profile.knots.size() > kMaximumProfileKnots ||
      profile.knots.front().w != profile.valid_start ||
      profile.knots.back().w != profile.valid_end) {
    reason = "Section PWL profile domain or knot count is malformed";
    return false;
  }
  if (profile.status == SectionTubeStatus::COMPLETE && !profile.complete) {
    reason = "COMPLETE Section PWL profile is not marked complete";
    return false;
  }
  if (profile.status == SectionTubeStatus::ZERO_ONLY && !profile.complete) {
    reason = "ZERO_ONLY Section PWL profile is not marked complete";
    return false;
  }
  if (profile.status == SectionTubeStatus::PARTIAL && profile.complete) {
    reason = "PARTIAL Section PWL profile is marked complete";
    return false;
  }
  if (profile.status != SectionTubeStatus::COMPLETE &&
      profile.status != SectionTubeStatus::ZERO_ONLY &&
      profile.status != SectionTubeStatus::PARTIAL) {
    reason = "Section PWL profile status is not queryable";
    return false;
  }
  bool zero_only = true;
  for (std::size_t i = 0U; i < profile.knots.size(); ++i) {
    if (!ConsumeWorkV2(1U, max_work, work_count)) {
      reason = "Section PWL validation work budget is exhausted";
      return false;
    }
    const SectionTubeKnot& knot = profile.knots[i];
    if (!Finite(knot.w) || !Finite(knot.lower) || !Finite(knot.upper) ||
        knot.lower > knot.upper || knot.lower > 0.0 || knot.upper < 0.0 ||
        (i > 0U && !(knot.w > profile.knots[i - 1U].w))) {
      reason = "Section PWL knots are nonfinite, unordered or exclude zero";
      return false;
    }
    zero_only = zero_only && knot.lower == 0.0 && knot.upper == 0.0;
  }
  if (profile.status == SectionTubeStatus::ZERO_ONLY && !zero_only) {
    reason = "ZERO_ONLY Section PWL profile contains nonzero capacity";
    return false;
  }
  return true;
}

bool EvaluateSection(const SectionTubeProfile& profile,
                     const TubeViabilityInput& input,
                     TubeViabilityResult& output) {
  output = TubeViabilityResult();
  output.proof_kind = TubeViabilityProofKind::SECTION_PWL;
  output.policy = input.policy;
  output.upper_u_delta = input.upper_u_delta;
  output.provenance.immutable = true;
  output.provenance.path_revision = input.path_revision;
  output.provenance.frame_revision = input.frame_revision;
  output.provenance.source =
      "phase_offset_navigation/tube_viability/section-pwl";
  output.max_work = input.max_work;

  const auto invalid = [&output](const std::string& reason) {
    output.status = TubeViabilityStatus::INVALID_INPUT;
    output.valid = false;
    output.feasible = false;
    output.rate_feasible = false;
    output.contraction_rate_feasible = false;
    output.reason = reason;
    output.knots.clear();
    output.section_profile = nullptr;
    output.evaluated_delta = 0.0;
    return false;
  };
  if (!SupportedFloatingPointEnvironmentV2()) {
    return invalid("Section PWL preview requires nearest binary64 without FTZ/DAZ/FMA");
  }
  std::string reason;
  if (!input.policy.numericValid() || !Finite(input.current_w) ||
      !Finite(input.current_delta) || !Finite(input.upper_u_delta) ||
      input.upper_u_delta < 0.0 || !Finite(input.boundary_tolerance) ||
      input.boundary_tolerance < 0.0 || input.max_work == 0U) {
    return invalid("Section PWL policy, capability or live values are invalid");
  }
  if (input.profile != nullptr || !input.cross_sections.empty()) {
    return invalid("Section PWL preview cannot bind a legacy geometric source");
  }
  std::size_t validation_work = 0U;
  if (!ValidateSectionProfile(profile, input.max_work, validation_work,
                              reason)) {
    output.work_count = validation_work;
    return invalid(reason);
  }
  output.work_count = validation_work;
  // A Section profile has no embedded path/frame identity.  The caller's
  // nonzero revisions are therefore the only immutable binding available to
  // Preview and allocator consumers.
  if (input.path_revision == 0U || input.frame_revision == 0U ||
      (input.expected_path_revision != 0U &&
       input.expected_path_revision != input.path_revision) ||
      (input.expected_frame_revision != 0U &&
       input.expected_frame_revision != input.frame_revision)) {
    return invalid("Section PWL path/frame provenance is missing or stale");
  }
  if (input.current_w < profile.valid_start ||
      input.current_w > profile.valid_end) {
    return invalid("current phase is outside the Section PWL domain");
  }
  if (input.section_reaches_path_end &&
      (!profile.complete || profile.status == SectionTubeStatus::PARTIAL)) {
    return invalid("Section terminal flag requires a complete non-PARTIAL profile");
  }

  output.current_w = input.current_w;
  output.current_w_inside = true;
  StrictPwlSourceView source;
  source.section_profile = &profile;
  const bool evaluated = EvaluateStrictPwlCore(
      source, input, output, profile.valid_end, input.section_reaches_path_end,
      "Section PWL", validation_work);
  // A state-feasible evaluation (the corridor exists at the current phase)
  // borrows the immutable profile even when its *rate* window cannot host the
  // requested motion, or its offset sits outside the corridor and must be
  // recovered.  Callers bind the returned pointer to the profile they passed,
  // so leaving it null for those statuses would make the identity gate reject
  // every degraded tick and freeze the reference.
  if (evaluated && output.feasible) {
    output.section_profile = &profile;
    output.evaluated_delta = input.current_delta;
  }
  return evaluated;
}

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
  StrictPwlSourceView source;
  source.v2_profile = &profile;
  return EvaluateStrictPwlCore(source, input, output,
                               profile.certified_end, false, "live V2");
}

// Locate and conservatively interpolate one immutable Section K interval.
// This is deliberately independent from TubeViabilityResult::envelopeAt,
// whose observational legacy implementation performs a linear scan.  A held
// step only visits the small knot range touched by its terminal interval.
std::size_t HeldProbeCost(const std::size_t count) {
  std::size_t probes = 1U;
  std::size_t remaining = count;
  while (remaining > 1U) {
    ++probes;
    remaining = (remaining + 1U) / 2U;
  }
  return probes;
}

void SetHeldFailure(TubeHeldStepResult& output, const std::size_t work,
                    const std::string& reason) {
  output.valid = false;
  output.next_w = 0.0;
  output.next_delta = 0.0;
  output.work_count = work;
  output.reason = reason;
}

bool HeldEnvelopeAt(const TubeViabilityResult& preview, const double w,
                    const std::size_t max_work, std::size_t& work,
                    TubeHeldStepResult& output,
                    TubeViabilityInterval& interval) {
  interval = TubeViabilityInterval();
  if (!Finite(w) || preview.knots.size() < 2U ||
      w < preview.preview_start_w || w > preview.preview_end_w) {
    return false;
  }
  if (!ConsumeWorkV2(HeldProbeCost(preview.knots.size()), max_work, work)) {
    SetHeldFailure(output, work, "held-step work budget is exhausted");
    return false;
  }
  const std::vector<TubeViabilityKnot>& knots = preview.knots;
  std::size_t first = 0U;
  std::size_t last = knots.size();
  while (first < last) {
    const std::size_t middle = first + (last - first) / 2U;
    if (!Finite(knots[middle].w)) return false;
    if (knots[middle].w < w) {
      first = middle + 1U;
    } else {
      last = middle;
    }
  }
  if (first < knots.size() && knots[first].w == w) {
    interval = knots[first].reachable;
    return StrictTubeIntervalValidV2(interval);
  }
  if (first == 0U || first >= knots.size()) return false;
  const TubeViabilityKnot& left = knots[first - 1U];
  const TubeViabilityKnot& right = knots[first];
  if (!(right.w > left.w) || !StrictTubeIntervalValidV2(left.reachable) ||
      !StrictTubeIntervalValidV2(right.reachable)) {
    return false;
  }
  return InterpolateStrictBounds(
      left.w, left.reachable.lower, left.reachable.upper, right.w,
      right.reachable.lower, right.reachable.upper, w, interval);
}

bool HeldLowerBound(const TubeViabilityResult& preview, const double w,
                    const std::size_t max_work, std::size_t& work,
                    TubeHeldStepResult& output, std::size_t& result) {
  if (!ConsumeWorkV2(HeldProbeCost(preview.knots.size()), max_work, work)) {
    SetHeldFailure(output, work, "held-step work budget is exhausted");
    return false;
  }
  std::size_t first = 0U;
  std::size_t last = preview.knots.size();
  while (first < last) {
    const std::size_t middle = first + (last - first) / 2U;
    if (preview.knots[middle].w < w) {
      first = middle + 1U;
    } else {
      last = middle;
    }
  }
  result = first;
  return true;
}

bool HeldUpperBound(const TubeViabilityResult& preview, const double w,
                    const std::size_t max_work, std::size_t& work,
                    TubeHeldStepResult& output, std::size_t& result) {
  if (!ConsumeWorkV2(HeldProbeCost(preview.knots.size()), max_work, work)) {
    SetHeldFailure(output, work, "held-step work budget is exhausted");
    return false;
  }
  std::size_t first = 0U;
  std::size_t last = preview.knots.size();
  while (first < last) {
    const std::size_t middle = first + (last - first) / 2U;
    if (!(preview.knots[middle].w > w)) {
      first = middle + 1U;
    } else {
      last = middle;
    }
  }
  result = first;
  return true;
}

bool ConsumeHeldWork(const std::size_t max_work, std::size_t& work,
                     TubeHeldStepResult& output, const char* reason) {
  if (!ConsumeWorkV2(1U, max_work, work)) {
    SetHeldFailure(output, work, reason);
    return false;
  }
  return true;
}

bool IntersectHeld(const TubeViabilityInterval& first,
                   const TubeViabilityInterval& second,
                   TubeViabilityInterval& result) {
  if (!StrictTubeIntervalValidV2(first) ||
      !StrictTubeIntervalValidV2(second)) {
    result = TubeViabilityInterval();
    return false;
  }
  const TubeViabilityInterval candidate =
      MakeInterval(std::max(first.lower, second.lower),
                   std::min(first.upper, second.upper));
  result = candidate;
  return StrictTubeIntervalValidV2(candidate);
}

bool CheckHeldTerminalRange(const TubeViabilityResult& preview,
                            const V2Interval& terminal_w,
                            const V2Interval& terminal_delta,
                            const double next_w, const double next_delta,
                            const std::size_t max_work,
                            std::size_t& work,
                            TubeHeldStepResult& output) {
  if (!terminal_w.valid || !terminal_delta.valid ||
      terminal_w.lower < preview.preview_start_w ||
      terminal_w.upper > preview.preview_end_w ||
      terminal_w.lower > terminal_w.upper) {
    SetHeldFailure(output, work,
                   "held terminal multiply-add interval leaves the preview domain");
    return false;
  }

  TubeViabilityInterval common;
  bool have_common = false;
  const auto include = [&](const TubeViabilityInterval& envelope) -> bool {
    if (!ConsumeHeldWork(max_work, work, output,
                         "held-step work budget is exhausted")) {
      return false;
    }
    if (!StrictTubeIntervalValidV2(envelope)) {
      SetHeldFailure(output, work,
                     "held terminal PWL envelope is unavailable");
      return false;
    }
    if (!have_common) {
      common = envelope;
      have_common = true;
      return true;
    }
    return IntersectHeld(common, envelope, common);
  };

  TubeViabilityInterval terminal_lower;
  TubeViabilityInterval terminal_upper;
  if (!HeldEnvelopeAt(preview, terminal_w.lower, max_work, work, output,
                      terminal_lower) ||
      !include(terminal_lower) ||
      (terminal_w.upper != terminal_w.lower &&
       (!HeldEnvelopeAt(preview, terminal_w.upper, max_work, work, output,
                        terminal_upper) ||
        !include(terminal_upper)))) {
    if (output.reason.empty()) {
      SetHeldFailure(output, work,
                     "held terminal PWL envelope is unavailable");
    }
    return false;
  }
  std::size_t begin = 0U;
  std::size_t end = 0U;
  if (!HeldLowerBound(preview, terminal_w.lower, max_work, work, output,
                      begin) ||
      !HeldUpperBound(preview, terminal_w.upper, max_work, work, output,
                      end)) {
    return false;
  }
  if (begin > end || end > preview.knots.size()) {
    SetHeldFailure(output, work, "held terminal knot range is malformed");
    return false;
  }
  for (std::size_t index = begin; index < end; ++index) {
    const double knot_w = preview.knots[index].w;
    if (knot_w < terminal_w.lower || knot_w > terminal_w.upper) continue;
    if (!include(preview.knots[index].reachable)) return false;
  }
  if (!have_common || !StrictTubeIntervalValidV2(common) ||
      terminal_delta.lower > terminal_delta.upper ||
      terminal_delta.lower < common.lower ||
      terminal_delta.upper > common.upper ||
      next_delta < common.lower || next_delta > common.upper ||
      !Finite(next_w) || !Finite(next_delta)) {
    SetHeldFailure(output, work,
                   "held terminal delta interval is outside common PWL bounds");
    return false;
  }
  return true;
}

bool CheckHeldStepImpl(const TubeViabilityResult& preview,
                       const double current_w, const double current_delta,
                       const double phase_rate, const double u_delta,
                       const double dt, const std::size_t max_work,
                       TubeHeldStepResult& output) {
  output = TubeHeldStepResult();
  std::size_t work = 0U;
  const auto fail = [&](const std::string& reason) {
    SetHeldFailure(output, work, reason);
    return false;
  };
  if (!SupportedFloatingPointEnvironmentV2()) {
    return fail("held Section step requires nearest binary64 without FTZ/DAZ/FMA");
  }
  if (preview.proof_kind != TubeViabilityProofKind::SECTION_PWL ||
      preview.status != TubeViabilityStatus::FEASIBLE ||
      !preview.valid || !preview.feasible || !preview.rate_feasible ||
      !preview.contraction_rate_feasible || !preview.current_delta_inside ||
      preview.section_profile == nullptr ||
      !preview.policy.numericValid() || preview.knots.size() < 2U ||
      !Finite(preview.current_w) || !Finite(preview.evaluated_delta) ||
      !Finite(preview.preview_start_w) || !Finite(preview.preview_end_w) ||
      !(preview.preview_end_w > preview.preview_start_w) ||
      preview.current_w != preview.preview_start_w ||
      !Finite(preview.upper_u_delta) || preview.upper_u_delta < 0.0) {
    return fail("held Section preview is not a feasible immutable PWL result");
  }
  if (!Finite(current_w) || !Finite(current_delta) ||
      current_w != preview.current_w ||
      current_delta != preview.evaluated_delta) {
    return fail("held Section current W/delta does not match Preview state");
  }
  const SectionTubeProfile& profile = *preview.section_profile;
  if (!profile.usable || !Finite(profile.valid_start) ||
      !Finite(profile.valid_end) || profile.knots.size() < 2U ||
      profile.knots.front().w != profile.valid_start ||
      profile.knots.back().w != profile.valid_end ||
      (profile.status == SectionTubeStatus::COMPLETE && !profile.complete) ||
      (profile.status == SectionTubeStatus::ZERO_ONLY && !profile.complete) ||
      (profile.status == SectionTubeStatus::PARTIAL && profile.complete) ||
      (profile.status != SectionTubeStatus::COMPLETE &&
       profile.status != SectionTubeStatus::ZERO_ONLY &&
       profile.status != SectionTubeStatus::PARTIAL) ||
      preview.knots.front().w != preview.current_w ||
      preview.knots.back().w != preview.preview_end_w ||
      preview.preview_start_w < profile.valid_start ||
      preview.preview_end_w > profile.valid_end) {
    return fail("held Section profile binding or domain is invalid");
  }
  if (!Finite(phase_rate) || phase_rate < 0.0 ||
      phase_rate < preview.policy.lower_nu ||
      phase_rate > preview.policy.upper_nu) {
    return fail("held phase rate is outside the immutable Preview policy");
  }
  if (!Finite(u_delta) || std::abs(u_delta) > preview.upper_u_delta) {
    return fail("held transverse command exceeds the Preview capability");
  }
  if (!Finite(dt) || dt <= 0.0 || max_work == 0U) {
    return fail("held-step duration or work budget is invalid");
  }

  // The endpoint values below are the exact double state that Runtime may
  // later commit.  Their interval counterparts are used only for the safety
  // checks, never as a second successor state or as a clamped replacement.
  const double next_w = current_w + dt * phase_rate;
  const double next_delta = current_delta + dt * u_delta;
  if (!Finite(next_w) || !Finite(next_delta) || next_w < current_w) {
    return fail("held-step endpoint is nonfinite or moves phase backwards");
  }
  V2Interval terminal_w = V2Add(
      V2Point(current_w),
      V2Multiply(V2Point(dt), V2Point(phase_rate)));
  V2Interval terminal_delta = V2Add(
      V2Point(current_delta),
      V2Multiply(V2Point(dt), V2Point(u_delta)));
  if (!terminal_w.valid || !terminal_delta.valid) {
    return fail("held-step endpoint multiply-add is not representable");
  }

  if (!ConsumeHeldWork(max_work, work, output,
                       "held-step work budget is exhausted")) {
    return false;
  }
  TubeViabilityInterval current_envelope;
  if (!HeldEnvelopeAt(preview, current_w, max_work, work, output,
                      current_envelope)) {
    return false;
  }
  if (!current_envelope.contains(current_delta, 0.0)) {
    return fail("held current point is outside the Preview K envelope");
  }

  // Every K knot whose W may be reached in this tick is checked using an
  // outward time/delta interval.  A knot at the exact fixed terminal time is
  // still covered by CheckHeldTerminalRange and is not reverse-solved from
  // the rounded endpoint.
  std::size_t begin = 0U;
  std::size_t end = 0U;
  if (!HeldUpperBound(preview, current_w, max_work, work, output, begin) ||
      !HeldUpperBound(preview, terminal_w.upper, max_work, work, output,
                      end)) {
    return false;
  }
  if (begin > end || end > preview.knots.size()) {
    return fail("held crossed-knot range is malformed");
  }
  const V2Interval time_domain = V2Bounds(0.0, dt);
  for (std::size_t index = begin; index < end; ++index) {
    const double knot_w = preview.knots[index].w;
    if (!Finite(knot_w) || !(knot_w > current_w) ||
        knot_w > terminal_w.upper) {
      continue;
    }
    const bool exact_terminal_knot =
        terminal_w.lower == terminal_w.upper && terminal_w.lower == knot_w &&
        next_w == knot_w;
    if (exact_terminal_knot) continue;
    if (!ConsumeHeldWork(max_work, work, output,
                         "held-step work budget is exhausted")) {
      return false;
    }
    const V2Interval elapsed = V2DividePositive(
        V2Subtract(V2Point(knot_w), V2Point(current_w)),
        V2Point(phase_rate));
    if (!elapsed.valid) return fail("held crossed-knot time is not representable");
    TubeViabilityInterval elapsed_limited;
    if (!IntersectHeld(TubeViabilityInterval{elapsed.lower, elapsed.upper, true},
                       TubeViabilityInterval{time_domain.lower,
                                              time_domain.upper, true},
                       elapsed_limited)) {
      continue;
    }
    if (!ConsumeHeldWork(max_work, work, output,
                         "held-step work budget is exhausted")) {
      return false;
    }
    const V2Interval knot_delta = V2Add(
        V2Point(current_delta),
        V2Multiply(V2Point(u_delta),
                   V2Bounds(elapsed_limited.lower,
                            elapsed_limited.upper)));
    TubeViabilityInterval knot_envelope;
    knot_envelope = preview.knots[index].reachable;
    if (!knot_delta.valid || !StrictTubeIntervalValidV2(knot_envelope) ||
        knot_delta.lower < knot_envelope.lower ||
        knot_delta.upper > knot_envelope.upper) {
      return fail("held crossed-knot delta interval leaves the Preview K envelope");
    }
  }

  if (!CheckHeldTerminalRange(preview, terminal_w, terminal_delta, next_w,
                              next_delta, max_work, work, output)) {
    return false;
  }
  output.valid = true;
  output.next_w = next_w;
  output.next_delta = next_delta;
  output.work_count = work;
  output.reason = "held Section PWL step is feasible";
  return true;
}

}  // namespace

bool TubeViability::evaluate(const TubeProfileV2& profile,
                             const TubeViabilityInput& input,
                             TubeViabilityResult& output) {
  return EvaluateV2(profile, input, output);
}

bool TubeViability::evaluate(const SectionTubeProfile& profile,
                             const TubeViabilityInput& input,
                             TubeViabilityResult& output) {
  return EvaluateSection(profile, input, output);
}

bool TubeViability::checkHeldStep(const TubeViabilityResult& preview,
                                  const double current_w,
                                  const double current_delta,
                                  const double phase_rate,
                                  const double u_delta, const double dt,
                                  const std::size_t max_work,
                                  TubeHeldStepResult& output) {
  return CheckHeldStepImpl(preview, current_w, current_delta, phase_rate,
                           u_delta, dt, max_work, output);
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
  if (StrictPwlProof(result.proof_kind)) {
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
  if (StrictPwlProof(result.proof_kind)) {
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
