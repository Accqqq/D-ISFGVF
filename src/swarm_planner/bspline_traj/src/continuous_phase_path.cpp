#include <bspline_race/continuous_phase_path.h>
#include <phase_offset_core/normal_frame.h>

#include <algorithm>
#include <array>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <boost/multiprecision/cpp_int.hpp>

#if defined(__i386__) || defined(__x86_64__)
#include <immintrin.h>
#endif

namespace FLAG_Race
{
namespace
{
constexpr double kDomainEps = 1e-8;
constexpr int kArcLengthTableIntervals = 1024;
constexpr double kArcLengthSpeedEps = 1e-8;
constexpr double kCertificateSpeedEps = 1e-9;
constexpr double kCertificateRoundoff = 1e-11;

struct ArcLengthCell
{
    double t0 = 0.0;
    double h = 0.0;
    std::array<double, 6> a = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

struct ArcLengthMap
{
    std::vector<double> table_t;
    std::vector<double> table_s;
    std::vector<double> table_speed;
    std::vector<double> table_speed_derivative;
    std::vector<ArcLengthCell> cells;
};

struct VectorBounds
{
    double inf_norm = 0.0;
    double sup_norm = 0.0;
    // True only when every Bernstein control of the represented vector field
    // is bitwise identical.  In that case the polynomial is exactly
    // constant, so its norm has an exact lower bound and need not be eroded
    // by the non-constant roundoff margin.
    bool inf_norm_exact = false;
    bool valid = false;
};

struct DifferentialBounds
{
    double inf_speed = 0.0;
    double inf_horizontal_speed = 0.0;
    double sup_speed = 0.0;
    double sup_acceleration = 0.0;
    double sup_horizontal_acceleration = 0.0;
    double sup_jerk = 0.0;
    double sup_horizontal_jerk = 0.0;
    bool valid = false;
};

// V2 proof arithmetic.  Every elementary operation returns an interval for
// the exact real operation on its binary64 operands.  The old certificate
// helpers below intentionally remain for legacy callers; V2 producers never
// call those fixed-margin helpers.
struct ProofInterval {
    double lower = 0.0;
    double upper = 0.0;
    bool valid = false;
};

// The V2 producers rely on IEEE-754 binary64 operations rounded to nearest,
// gradual underflow, and separate multiply/add operations.  These properties
// are environmental inputs to the proof just like the stored coefficients;
// they cannot be repaired by widening an interval after the fact.  Read the
// environment only and fail closed when the compiler or execution mode does
// not provide the contract.  In particular, do not change MXCSR or the
// process rounding mode on behalf of a caller.
bool ProofFloatingPointEnvironmentSupported()
{
#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && \
                               __FINITE_MATH_ONLY__)
    return false;
#endif
#if defined(__FMA__) || defined(__FP_FAST_FMA) || defined(__FP_FAST_FMAF)
    // The proof arithmetic is deliberately written as separate operations;
    // a translation unit that permits unaccounted contraction is unsupported.
    return false;
#endif
#if defined(__FLT_EVAL_METHOD__) && __FLT_EVAL_METHOD__ != 0
    // Extended x87 evaluation would make the binary64 endpoint-underflow and
    // one-ULP reasoning below dependent on an unaccounted intermediate format.
    return false;
#endif
    if (!std::numeric_limits<double>::is_iec559 ||
        std::numeric_limits<double>::radix != 2 ||
        std::numeric_limits<double>::digits != 53 ||
        std::numeric_limits<double>::has_denorm !=
            std::denorm_present ||
        std::fegetround() != FE_TONEAREST) {
        return false;
    }
#if defined(__i386__) || defined(__x86_64__)
    const unsigned int csr = _mm_getcsr();
#ifdef _MM_FLUSH_ZERO_ON
    if ((csr & _MM_FLUSH_ZERO_ON) != 0U) return false;
#endif
#ifdef _MM_DENORMALS_ZERO_ON
    if ((csr & _MM_DENORMALS_ZERO_ON) != 0U) return false;
#endif
    return true;
#else
    // No portable C++ query proves that a non-x86 runtime has not enabled a
    // flush-to-zero mode (for example through an ARM FP control register).
    // The proof therefore remains unavailable instead of assuming gradual
    // underflow from the type traits alone.
    return false;
#endif
}

ProofInterval InvalidProofInterval()
{
    return ProofInterval();
}

ProofInterval PointProofInterval(const double value)
{
    if (!std::isfinite(value)) return InvalidProofInterval();
    ProofInterval result;
    result.lower = value;
    result.upper = value;
    result.valid = true;
    return result;
}

bool ProofFinite(const ProofInterval& value)
{
    return value.valid && std::isfinite(value.lower) &&
        std::isfinite(value.upper) && value.lower <= value.upper;
}

double ProofLower(const double value)
{
    if (!std::isfinite(value)) return std::numeric_limits<double>::quiet_NaN();
    if (value == 0.0) return 0.0;
    return std::nextafter(value, -std::numeric_limits<double>::infinity());
}

double ProofUpper(const double value)
{
    if (!std::isfinite(value)) return std::numeric_limits<double>::quiet_NaN();
    if (value == 0.0) return 0.0;
    return std::nextafter(value, std::numeric_limits<double>::infinity());
}

ProofInterval ProofIntervalFromBounds(const double lower, const double upper)
{
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
        return InvalidProofInterval();
    }
    ProofInterval result;
    result.lower = lower;
    result.upper = upper;
    result.valid = true;
    return result;
}

bool ProofSumEndpoint(const double lhs, const double rhs,
                      double& lower, double& upper)
{
    const double value = lhs + rhs;
    if (!std::isfinite(value)) return false;
    if (value == 0.0 && lhs != 0.0 && rhs != 0.0) {
        // Gradual-underflow addition can round a nonzero exact sum to zero.
        // Same-sign operands determine the sign; cancellation leaves the
        // sign unknown, so use the complete one-denorm hull.
        const double denorm = std::numeric_limits<double>::denorm_min();
        if (!(denorm > 0.0)) return false;
        if (std::signbit(lhs) == std::signbit(rhs)) {
            if (std::signbit(lhs)) {
                lower = -denorm;
                upper = 0.0;
            } else {
                lower = 0.0;
                upper = denorm;
            }
        } else {
            lower = -denorm;
            upper = denorm;
        }
        return true;
    }
    lower = ProofLower(value);
    upper = ProofUpper(value);
    return std::isfinite(lower) && std::isfinite(upper);
}

ProofInterval ProofAdd(const ProofInterval& lhs, const ProofInterval& rhs)
{
    if (!ProofFinite(lhs) || !ProofFinite(rhs)) return InvalidProofInterval();
    double lower_endpoint_lower = 0.0;
    double lower_endpoint_upper = 0.0;
    double upper_endpoint_lower = 0.0;
    double upper_endpoint_upper = 0.0;
    if (!ProofSumEndpoint(lhs.lower, rhs.lower,
                          lower_endpoint_lower, lower_endpoint_upper) ||
        !ProofSumEndpoint(lhs.upper, rhs.upper,
                          upper_endpoint_lower, upper_endpoint_upper)) {
        return InvalidProofInterval();
    }
    return ProofIntervalFromBounds(
        lower_endpoint_lower, upper_endpoint_upper);
}

// Return an outward enclosure for one endpoint product.  A rounded binary64
// product can underflow to zero even when the represented real product is
// nonzero (notably for subnormal inputs).  Treat that case explicitly rather
// than allowing a zero upper endpoint to become an under-approximation.
bool ProofProductEndpoint(const double lhs, const double rhs,
                          double& lower, double& upper)
{
    const double value = lhs * rhs;
    if (!std::isfinite(value)) return false;
    if (value == 0.0 && lhs != 0.0 && rhs != 0.0) {
        const double denorm = std::numeric_limits<double>::denorm_min();
        if (!(denorm > 0.0)) return false;
        const bool negative = std::signbit(lhs) != std::signbit(rhs);
        if (negative) {
            lower = -denorm;
            upper = 0.0;
        } else {
            lower = 0.0;
            upper = denorm;
        }
        return true;
    }
    lower = ProofLower(value);
    upper = ProofUpper(value);
    return std::isfinite(lower) && std::isfinite(upper);
}

bool ProofQuotientEndpoint(const double numerator, const double denominator,
                           double& lower, double& upper)
{
    if (denominator == 0.0) return false;
    const double value = numerator / denominator;
    if (!std::isfinite(value)) return false;
    if (value == 0.0 && numerator != 0.0) {
        const double denorm = std::numeric_limits<double>::denorm_min();
        if (!(denorm > 0.0)) return false;
        const bool negative = std::signbit(numerator) !=
            std::signbit(denominator);
        if (negative) {
            lower = -denorm;
            upper = 0.0;
        } else {
            lower = 0.0;
            upper = denorm;
        }
        return true;
    }
    lower = ProofLower(value);
    upper = ProofUpper(value);
    return std::isfinite(lower) && std::isfinite(upper);
}

ProofInterval ProofSub(const ProofInterval& lhs, const ProofInterval& rhs)
{
    if (!ProofFinite(lhs) || !ProofFinite(rhs)) return InvalidProofInterval();
    double lower_endpoint_lower = 0.0;
    double lower_endpoint_upper = 0.0;
    double upper_endpoint_lower = 0.0;
    double upper_endpoint_upper = 0.0;
    if (!ProofSumEndpoint(lhs.lower, -rhs.upper,
                          lower_endpoint_lower, lower_endpoint_upper) ||
        !ProofSumEndpoint(lhs.upper, -rhs.lower,
                          upper_endpoint_lower, upper_endpoint_upper)) {
        return InvalidProofInterval();
    }
    return ProofIntervalFromBounds(
        lower_endpoint_lower, upper_endpoint_upper);
}

ProofInterval ProofMul(const ProofInterval& lhs, const ProofInterval& rhs)
{
    if (!ProofFinite(lhs) || !ProofFinite(rhs)) return InvalidProofInterval();
    double lower = std::numeric_limits<double>::infinity();
    double upper = -std::numeric_limits<double>::infinity();
    const double lhs_values[] = {lhs.lower, lhs.upper};
    const double rhs_values[] = {rhs.lower, rhs.upper};
    for (const double lhs_value : lhs_values) {
        for (const double rhs_value : rhs_values) {
            double endpoint_lower = 0.0;
            double endpoint_upper = 0.0;
            if (!ProofProductEndpoint(lhs_value, rhs_value,
                                      endpoint_lower, endpoint_upper)) {
                return InvalidProofInterval();
            }
            lower = std::min(lower, endpoint_lower);
            upper = std::max(upper, endpoint_upper);
        }
    }
    return ProofIntervalFromBounds(lower, upper);
}

ProofInterval ProofDiv(const ProofInterval& lhs, const ProofInterval& rhs)
{
    if (!ProofFinite(lhs) || !ProofFinite(rhs) ||
        (rhs.lower <= 0.0 && rhs.upper >= 0.0)) {
        return InvalidProofInterval();
    }
    double lower = std::numeric_limits<double>::infinity();
    double upper = -std::numeric_limits<double>::infinity();
    const double lhs_values[] = {lhs.lower, lhs.upper};
    const double rhs_values[] = {rhs.lower, rhs.upper};
    for (const double lhs_value : lhs_values) {
        for (const double rhs_value : rhs_values) {
            double endpoint_lower = 0.0;
            double endpoint_upper = 0.0;
            if (!ProofQuotientEndpoint(lhs_value, rhs_value,
                                       endpoint_lower, endpoint_upper)) {
                return InvalidProofInterval();
            }
            lower = std::min(lower, endpoint_lower);
            upper = std::max(upper, endpoint_upper);
        }
    }
    return ProofIntervalFromBounds(lower, upper);
}

ProofInterval ProofSqrt(const ProofInterval& value)
{
    if (!ProofFinite(value) || value.lower < 0.0) {
        return InvalidProofInterval();
    }
    const double lower = std::sqrt(value.lower);
    const double upper = std::sqrt(value.upper);
    if (!std::isfinite(lower) || !std::isfinite(upper)) {
        return InvalidProofInterval();
    }
    return ProofIntervalFromBounds(ProofLower(lower), ProofUpper(upper));
}

ProofInterval ProofNorm(const std::array<ProofInterval, 3U>& components)
{
    ProofInterval lower_squared = PointProofInterval(0.0);
    ProofInterval upper_squared = PointProofInterval(0.0);
    for (const ProofInterval& component : components) {
        if (!ProofFinite(component)) return InvalidProofInterval();
        const double near_zero = component.lower <= 0.0 &&
            component.upper >= 0.0 ? 0.0 : std::min(
                std::abs(component.lower), std::abs(component.upper));
        const double farthest = std::max(std::abs(component.lower),
                                         std::abs(component.upper));
        const ProofInterval low_component = PointProofInterval(near_zero);
        const ProofInterval high_component = PointProofInterval(farthest);
        const ProofInterval low_square = ProofMul(low_component, low_component);
        const ProofInterval high_square = ProofMul(high_component, high_component);
        lower_squared = ProofAdd(lower_squared, low_square);
        upper_squared = ProofAdd(upper_squared, high_square);
        if (!ProofFinite(lower_squared) || !ProofFinite(upper_squared)) {
            return InvalidProofInterval();
        }
    }
    const ProofInterval lower = ProofSqrt(lower_squared);
    const ProofInterval upper = ProofSqrt(upper_squared);
    if (!ProofFinite(lower) || !ProofFinite(upper)) return InvalidProofInterval();
    return ProofIntervalFromBounds(lower.lower, upper.upper);
}

ProofInterval ProofHorner(const std::vector<ProofInterval>& coefficients,
                          const ProofInterval& x)
{
    if (coefficients.empty() || !ProofFinite(x)) return InvalidProofInterval();
    ProofInterval value = coefficients.back();
    for (std::size_t index = coefficients.size() - 1U; index > 0U; --index) {
        value = ProofAdd(ProofMul(value, x), coefficients[index - 1U]);
        if (!ProofFinite(value)) return InvalidProofInterval();
    }
    return value;
}

std::uint64_t ProofBinomialInteger(const int n, int k)
{
    if (k < 0 || k > n) return 0U;
    k = std::min(k, n - k);
    std::uint64_t result = 1U;
    for (int index = 1; index <= k; ++index) {
        const std::uint64_t numerator = static_cast<std::uint64_t>(
            n - k + index);
        const std::uint64_t denominator = static_cast<std::uint64_t>(index);
        // The proof producers only use degrees <= 5 here.  Keep the helper
        // fail-closed if a future caller exceeds exact uint64 arithmetic.
        if (result > std::numeric_limits<std::uint64_t>::max() /
            numerator) return 0U;
        result = result * numerator / denominator;
    }
    return result;
}

ProofInterval ProofRationalInterval(const std::uint64_t numerator,
                                    const std::uint64_t denominator)
{
    if (denominator == 0U) return InvalidProofInterval();
    if (numerator == 0U) return PointProofInterval(0.0);
    const double value = static_cast<double>(numerator) /
        static_cast<double>(denominator);
    if (!std::isfinite(value)) return InvalidProofInterval();
    return ProofIntervalFromBounds(ProofLower(value), ProofUpper(value));
}

// Enclose a power-basis polynomial over [0,1] by forming its Bernstein
// controls with interval arithmetic and taking their hull.  This is used for
// the stored phase-map quintic derivative: unlike interval Horner over the
// entire cell, the Bernstein hull preserves the map's positive derivative
// witness without introducing dependency-induced sign loss.
ProofInterval ProofPowerToBernsteinRange(
    const std::vector<ProofInterval>& coefficients)
{
    if (coefficients.empty()) return InvalidProofInterval();
    const int degree = static_cast<int>(coefficients.size()) - 1;
    ProofInterval result = InvalidProofInterval();
    for (int control = 0; control <= degree; ++control) {
        ProofInterval value = PointProofInterval(0.0);
        for (int order = 0; order <= control; ++order) {
            const std::uint64_t numerator = ProofBinomialInteger(control,
                                                                  order);
            const std::uint64_t denominator = ProofBinomialInteger(degree,
                                                                    order);
            const ProofInterval ratio = ProofRationalInterval(numerator,
                                                               denominator);
            value = ProofAdd(value, ProofMul(
                coefficients[static_cast<std::size_t>(order)], ratio));
            if (!ProofFinite(value)) return InvalidProofInterval();
        }
        if (!ProofFinite(value)) return InvalidProofInterval();
        if (!result.valid) {
            result = value;
        } else {
            result.lower = std::min(result.lower, value.lower);
            result.upper = std::max(result.upper, value.upper);
        }
    }
    return ProofFinite(result) ? result : InvalidProofInterval();
}

phase_offset_core::Binary64Interval ToPublicInterval(
    const ProofInterval& value)
{
    phase_offset_core::Binary64Interval result;
    result.lower = value.lower;
    result.upper = value.upper;
    result.valid = ProofFinite(value);
    return result;
}

void SetPublicVectorInterval(
    const std::array<ProofInterval, 3U>& source,
    phase_offset_core::Binary64VectorInterval& destination)
{
    destination = phase_offset_core::Binary64VectorInterval();
    for (std::size_t index = 0U; index < 3U; ++index) {
        destination.component[index] = ToPublicInterval(source[index]);
        if (!ProofFinite(source[index])) return;
    }
    destination.valid = true;
}

std::array<ProofInterval, 3U> ProofVectorConstant(const Eigen::Vector3d& value)
{
    return {{PointProofInterval(value.x()), PointProofInterval(value.y()),
             PointProofInterval(value.z())}};
}

using ProofVectorPolynomial =
    std::vector<std::array<ProofInterval, 3U>>;

ProofVectorPolynomial ProofPolynomialFromEigen(
    const std::vector<Eigen::Vector3d>& coefficients)
{
    ProofVectorPolynomial result;
    result.reserve(coefficients.size());
    for (const Eigen::Vector3d& coefficient : coefficients) {
        result.push_back(ProofVectorConstant(coefficient));
    }
    return result;
}

ProofVectorPolynomial ProofDifferentiatePolynomial(
    const ProofVectorPolynomial& coefficients)
{
    if (coefficients.size() <= 1U) {
        return ProofVectorPolynomial(1U, {{PointProofInterval(0.0),
                                          PointProofInterval(0.0),
                                          PointProofInterval(0.0)}});
    }
    ProofVectorPolynomial result(coefficients.size() - 1U);
    for (std::size_t order = 1U; order < coefficients.size(); ++order) {
        for (std::size_t component = 0U; component < 3U; ++component) {
            result[order - 1U][component] = ProofMul(
                coefficients[order][component],
                PointProofInterval(static_cast<double>(order)));
        }
    }
    return result;
}

std::array<ProofInterval, 3U> ProofVectorPolynomialEvaluate(
    const ProofVectorPolynomial& coefficients, const ProofInterval& x)
{
    std::array<ProofInterval, 3U> result;
    if (coefficients.empty()) {
        result = {{InvalidProofInterval(), InvalidProofInterval(),
                   InvalidProofInterval()}};
        return result;
    }
    for (std::size_t component = 0U; component < 3U; ++component) {
        ProofInterval value = coefficients.back()[component];
        for (std::size_t index = coefficients.size() - 1U; index > 0U;
             --index) {
            value = ProofAdd(ProofMul(value, x),
                             coefficients[index - 1U][component]);
        }
        result[component] = value;
    }
    return result;
}

std::array<ProofInterval, 3U> ProofVectorScale(
    const std::array<ProofInterval, 3U>& vector,
    const ProofInterval& scale)
{
    std::array<ProofInterval, 3U> result;
    for (std::size_t index = 0U; index < 3U; ++index) {
        result[index] = ProofMul(vector[index], scale);
    }
    return result;
}

std::array<ProofInterval, 3U> ProofVectorAdd(
    const std::array<ProofInterval, 3U>& lhs,
    const std::array<ProofInterval, 3U>& rhs)
{
    std::array<ProofInterval, 3U> result;
    for (std::size_t index = 0U; index < 3U; ++index) {
        result[index] = ProofAdd(lhs[index], rhs[index]);
    }
    return result;
}

bool ProofVectorFinite(const std::array<ProofInterval, 3U>& vector)
{
    for (const ProofInterval& component : vector) {
        if (!ProofFinite(component)) return false;
    }
    return true;
}

double UpperBound(const double value)
{
    if (!std::isfinite(value) || value < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (value == 0.0) return 0.0;
    return std::nextafter(value * (1.0 + kCertificateRoundoff) +
                              kCertificateRoundoff,
                          std::numeric_limits<double>::infinity());
}

double LowerBound(const double value)
{
    if (!std::isfinite(value) || value < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (value == 0.0) return 0.0;
    const double lowered = value * (1.0 - kCertificateRoundoff) -
        kCertificateRoundoff;
    return std::max(0.0, std::nextafter(
        lowered, -std::numeric_limits<double>::infinity()));
}

struct Binary64Parts
{
    std::uint64_t significand = 0U;
    int exponent = 0;
};

Binary64Parts DecomposeBinary64(const double value)
{
    Binary64Parts parts;
    const double magnitude = std::abs(value);
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &magnitude, sizeof(bits));
    const std::uint64_t fraction = bits & ((std::uint64_t(1) << 52U) - 1U);
    const std::uint64_t exponent_bits = (bits >> 52U) & 0x7ffU;
    if (exponent_bits == 0U) {
        parts.significand = fraction;
        parts.exponent = -1074;
    } else {
        parts.significand = (std::uint64_t(1) << 52U) | fraction;
        parts.exponent = static_cast<int>(exponent_bits) - 1023 - 52;
    }
    return parts;
}

// Compare two finite binary64 values in exact dyadic arithmetic.  A rounded
// subtraction can report `1.0` even when the represented real difference is
// slightly larger; the supported UniformBspline contract needs the latter
// exact fact because getDerivative() omits a non-unit knot denominator.
bool ExactBinary64DifferenceEqualsOne(const double upper, const double lower)
{
    if (!std::isfinite(upper) || !std::isfinite(lower)) return false;
    const Binary64Parts upper_parts = DecomposeBinary64(upper);
    const Binary64Parts lower_parts = DecomposeBinary64(lower);
    int base_exponent = 0;
    bool have_nonzero = false;
    if (upper_parts.significand != 0U) {
        base_exponent = upper_parts.exponent;
        have_nonzero = true;
    }
    if (lower_parts.significand != 0U) {
        base_exponent = have_nonzero
            ? std::min(base_exponent, lower_parts.exponent)
            : lower_parts.exponent;
        have_nonzero = true;
    }
    if (!have_nonzero) return false;
    base_exponent = std::min(base_exponent, 0);
    boost::multiprecision::cpp_int upper_integer = 0;
    boost::multiprecision::cpp_int lower_integer = 0;
    if (upper_parts.significand != 0U) {
        upper_integer = upper_parts.significand;
        upper_integer <<= upper_parts.exponent - base_exponent;
        if (std::signbit(upper)) upper_integer = -upper_integer;
    }
    if (lower_parts.significand != 0U) {
        lower_integer = lower_parts.significand;
        lower_integer <<= lower_parts.exponent - base_exponent;
        if (std::signbit(lower)) lower_integer = -lower_integer;
    }
    const boost::multiprecision::cpp_int difference =
        upper_integer - lower_integer;
    const boost::multiprecision::cpp_int one =
        boost::multiprecision::cpp_int(1) << (-base_exponent);
    return difference == one;
}

struct ExactSquaredMagnitude
{
    boost::multiprecision::cpp_int significand = 0;
    int exponent = 0;
    bool zero = true;
};

ExactSquaredMagnitude ExactSquaredMagnitudeFor(
    const Eigen::Vector3d& value, const bool horizontal)
{
    const double components[] = {value.x(), value.y(),
                                 horizontal ? 0.0 : value.z()};
    ExactSquaredMagnitude result;
    int minimum_exponent = std::numeric_limits<int>::max();
    Binary64Parts parts[3];
    for (int index = 0; index < 3; ++index) {
        parts[index] = DecomposeBinary64(components[index]);
        if (parts[index].significand != 0U) {
            result.zero = false;
            minimum_exponent = std::min(minimum_exponent,
                                        2 * parts[index].exponent);
        }
    }
    if (result.zero) return result;
    result.exponent = minimum_exponent;
    for (int index = 0; index < 3; ++index) {
        if (parts[index].significand == 0U) continue;
        const int shift = 2 * parts[index].exponent - minimum_exponent;
        boost::multiprecision::cpp_int term = parts[index].significand;
        term *= parts[index].significand;
        result.significand += term << shift;
    }
    return result;
}

bool Binary64SquaredLeq(const double candidate,
                        const ExactSquaredMagnitude& exact)
{
    if (candidate <= 0.0) return true;
    if (exact.zero) return false;
    const Binary64Parts parts = DecomposeBinary64(candidate);
    boost::multiprecision::cpp_int candidate_significand = parts.significand;
    candidate_significand *= parts.significand;
    const int candidate_exponent = 2 * parts.exponent;
    if (candidate_exponent >= exact.exponent) {
        candidate_significand <<= candidate_exponent - exact.exponent;
        return candidate_significand <= exact.significand;
    }
    boost::multiprecision::cpp_int exact_significand = exact.significand;
    exact_significand <<= exact.exponent - candidate_exponent;
    return candidate_significand <= exact_significand;
}

double DirectedLowerNorm(const Eigen::Vector3d& value, const bool horizontal)
{
    if (!value.allFinite()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const ExactSquaredMagnitude exact =
        ExactSquaredMagnitudeFor(value, horizontal);
    if (exact.zero) return 0.0;

    // Positive binary64 bit patterns are monotonically ordered by their
    // integer representation.  Binary-search the greatest finite value whose
    // exact square is no larger than the exact binary64 sum of squares.  This
    // avoids relying on long-double precision or on the final scale*root
    // multiplication being rounded in the desired direction.
    std::uint64_t lower = 0U;
    std::uint64_t upper = 0x7fefffffffffffffULL;  // DBL_MAX, finite
    while (lower < upper) {
        const std::uint64_t middle = lower +
            (upper - lower + 1U) / 2U;
        double candidate = 0.0;
        std::memcpy(&candidate, &middle, sizeof(candidate));
        if (Binary64SquaredLeq(candidate, exact)) {
            lower = middle;
        } else {
            upper = middle - 1U;
        }
    }
    double result = 0.0;
    std::memcpy(&result, &lower, sizeof(result));
    return result;
}

double ScaledLowerBound(const double value, const bool exact,
                        const double scale)
{
    if (!std::isfinite(value) || value < 0.0 ||
        !std::isfinite(scale) || scale <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double scaled = value / scale;
    if (!std::isfinite(scaled) || scaled < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (!exact) return LowerBound(scaled);
    // Preserve an exact constant quotient for every finite span, not only
    // binary powers.  The fused residual distinguishes a rounded-down
    // quotient (already a valid lower bound) from a rounded-up one; only the
    // latter is stepped toward zero.
    const double residual = std::fma(scaled, scale, -value);
    if (std::isfinite(residual) && residual <= 0.0) return scaled;
    return std::max(0.0, std::nextafter(
        scaled, -std::numeric_limits<double>::infinity()));
}

double ProductLowerBound(const double value, const bool exact,
                         const double scale)
{
    if (!std::isfinite(value) || value < 0.0 ||
        !std::isfinite(scale) || scale < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double product = value * scale;
    if (!std::isfinite(product) || product < 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (!exact) return LowerBound(product);
    // As above, retain exact constant products for arbitrary spans while
    // moving a rounded-up product one representable value downward.
    const double residual = std::fma(value, scale, -product);
    if (std::isfinite(residual) && residual <= 0.0) return product;
    return std::max(0.0, std::nextafter(
        product, -std::numeric_limits<double>::infinity()));
}

double Binomial(const int n, const int k)
{
    if (k < 0 || k > n) return 0.0;
    double result = 1.0;
    for (int index = 1; index <= k; ++index) {
        result *= static_cast<double>(n - k + index) /
            static_cast<double>(index);
    }
    return result;
}

std::vector<Eigen::Vector3d> RestrictPowerVector(
    const std::vector<Eigen::Vector3d>& coefficients,
    const double first, const double last)
{
    std::vector<Eigen::Vector3d> restricted(coefficients.size(),
                                             Eigen::Vector3d::Zero());
    if (!std::isfinite(first) || !std::isfinite(last) ||
        last < first || coefficients.empty()) {
        return std::vector<Eigen::Vector3d>();
    }
    const double scale = last - first;
    for (std::size_t order = 0U; order < coefficients.size(); ++order) {
        if (!coefficients[order].allFinite()) {
            return std::vector<Eigen::Vector3d>();
        }
        for (std::size_t output_order = 0U; output_order <= order;
             ++output_order) {
            restricted[output_order] += coefficients[order] *
                (Binomial(static_cast<int>(order),
                          static_cast<int>(output_order)) *
                 std::pow(first,
                          static_cast<int>(order - output_order)) *
                 std::pow(scale, static_cast<int>(output_order)));
        }
    }
    return restricted;
}

std::vector<Eigen::Vector3d> VectorPowerToBernstein(
    const std::vector<Eigen::Vector3d>& coefficients)
{
    if (coefficients.empty()) return std::vector<Eigen::Vector3d>();
    const int degree = static_cast<int>(coefficients.size()) - 1;
    std::vector<Eigen::Vector3d> controls(
        coefficients.size(), Eigen::Vector3d::Zero());
    for (int control = 0; control <= degree; ++control) {
        for (int order = 0; order <= control; ++order) {
            if (!coefficients[static_cast<std::size_t>(order)].allFinite()) {
                return std::vector<Eigen::Vector3d>();
            }
            controls[static_cast<std::size_t>(control)] +=
                coefficients[static_cast<std::size_t>(order)] *
                (Binomial(control, order) / Binomial(degree, order));
        }
    }
    return controls;
}

std::vector<double> ScalarPowerToBernstein(const std::vector<double>& coefficients)
{
    if (coefficients.empty()) return std::vector<double>();
    const int degree = static_cast<int>(coefficients.size()) - 1;
    std::vector<double> controls(coefficients.size(), 0.0);
    for (int control = 0; control <= degree; ++control) {
        for (int order = 0; order <= control; ++order) {
            if (!std::isfinite(coefficients[static_cast<std::size_t>(order)])) {
                return std::vector<double>();
            }
            controls[static_cast<std::size_t>(control)] +=
                coefficients[static_cast<std::size_t>(order)] *
                (Binomial(control, order) / Binomial(degree, order));
        }
    }
    return controls;
}

std::vector<Eigen::Vector3d> DifferentiateVectorPower(
    const std::vector<Eigen::Vector3d>& coefficients)
{
    if (coefficients.size() <= 1U) {
        return std::vector<Eigen::Vector3d>(1U, Eigen::Vector3d::Zero());
    }
    std::vector<Eigen::Vector3d> derivative(coefficients.size() - 1U,
                                              Eigen::Vector3d::Zero());
    for (std::size_t order = 1U; order < coefficients.size(); ++order) {
        derivative[order - 1U] = coefficients[order] *
            static_cast<double>(order);
    }
    return derivative;
}

std::vector<double> DifferentiateScalarPower(const std::vector<double>& coefficients)
{
    if (coefficients.size() <= 1U) return std::vector<double>(1U, 0.0);
    std::vector<double> derivative(coefficients.size() - 1U, 0.0);
    for (std::size_t order = 1U; order < coefficients.size(); ++order) {
        derivative[order - 1U] = coefficients[order] *
            static_cast<double>(order);
    }
    return derivative;
}

VectorBounds BoundsFromBernsteinControls(
    const std::vector<Eigen::Vector3d>& controls, const bool horizontal)
{
    VectorBounds bounds;
    if (controls.empty()) return bounds;
    const Eigen::Vector3d first = horizontal
        ? Eigen::Vector3d(controls.front().x(), controls.front().y(), 0.0)
        : controls.front();
    if (!first.allFinite()) return bounds;
    bool constant = true;
    Eigen::Vector3d center = first;
    for (std::size_t index = 1U; index < controls.size(); ++index) {
        const Eigen::Vector3d projected = horizontal
            ? Eigen::Vector3d(controls[index].x(), controls[index].y(), 0.0)
            : controls[index];
        if (!projected.allFinite()) return bounds;
        if ((projected.array() != first.array()).any()) constant = false;
        center += projected;
    }
    if (!constant) {
        center /= static_cast<double>(controls.size());
    }
    double radius = 0.0;
    double upper = 0.0;
    for (const Eigen::Vector3d& control : controls) {
        const Eigen::Vector3d projected = horizontal
            ? Eigen::Vector3d(control.x(), control.y(), 0.0) : control;
        radius = std::max(radius, (projected - center).norm());
        upper = std::max(upper, projected.norm());
    }
    const double lower = constant
        ? DirectedLowerNorm(first, horizontal)
        : std::max(0.0, center.norm() - radius);
    bounds.inf_norm = constant ? lower : LowerBound(lower);
    bounds.inf_norm_exact = constant;
    bounds.sup_norm = UpperBound(upper);
    bounds.valid = std::isfinite(bounds.inf_norm) &&
        std::isfinite(bounds.sup_norm) && bounds.sup_norm >= bounds.inf_norm;
    return bounds;
}

VectorBounds BoundsFromVectorPower(const std::vector<Eigen::Vector3d>& coefficients,
                                   const double first, const double last,
                                   const bool horizontal)
{
    return BoundsFromBernsteinControls(
        VectorPowerToBernstein(RestrictPowerVector(coefficients, first, last)),
        horizontal);
}

bool MakeCertificate(const double w0, const double w1,
                     const DifferentialBounds& differential,
                     phase_offset_core::PathCellGeometryCertificate& certificate)
{
    certificate = phase_offset_core::PathCellGeometryCertificate();
    const double h = w1 - w0;
    if (!differential.valid || !std::isfinite(w0) || !std::isfinite(w1) ||
        h <= kDomainEps ||
        differential.inf_speed <= kCertificateSpeedEps ||
        differential.inf_horizontal_speed <=
            phase_offset_core::kHorizontalNormalSpeedEpsilon ||
        !std::isfinite(differential.sup_speed) ||
        !std::isfinite(differential.sup_acceleration) ||
        !std::isfinite(differential.sup_horizontal_acceleration) ||
        !std::isfinite(differential.sup_jerk)) {
        return false;
    }
    // Horizontal-N uses the direct normalized-cross-product bound
    // ||N_w|| <= sup||p_ww,xy|| / inf||p_w,xy||.  Tangent variation remains a
    // separate full-3-D proof using sup||p_ww|| / inf||p_w||.
    const double normal_w = UpperBound(
        differential.sup_horizontal_acceleration /
            differential.inf_horizontal_speed);
    const double tangent_w = UpperBound(
        differential.sup_acceleration / differential.inf_speed);
    const double curvature = UpperBound(
        differential.sup_acceleration /
            (differential.inf_speed * differential.inf_speed));
    const double curvature_w = UpperBound(
        differential.sup_jerk /
            (differential.inf_speed * differential.inf_speed) +
        3.0 * differential.sup_acceleration * differential.sup_acceleration /
            (differential.inf_speed * differential.inf_speed *
             differential.inf_speed));
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.inf_p_w_norm = differential.inf_speed;
    certificate.inf_horizontal_p_w_norm = differential.inf_horizontal_speed;
    certificate.sup_p_w_norm = differential.sup_speed;
    certificate.sup_p_ww_norm = differential.sup_acceleration;
    certificate.sup_p_www_norm = differential.sup_jerk;
    certificate.sup_horizontal_p_ww_norm =
        differential.sup_horizontal_acceleration;
    certificate.horizontal_acceleration_bound_complete = true;
    certificate.sup_N_w_norm = normal_w;
    certificate.sup_abs_curvature = curvature;
    certificate.normal_variation_bound = UpperBound(normal_w * h);
    certificate.tangent_variation_bound = UpperBound(tangent_w * h);
    certificate.curvature_variation_bound = UpperBound(curvature_w * h);
    certificate.midpoint_position_variation_bound = UpperBound(
        0.5 * differential.sup_speed * h);
    certificate.chord_deviation_bound = UpperBound(
        differential.sup_acceleration * h * h / 8.0);
    certificate.valid = std::isfinite(normal_w) && std::isfinite(tangent_w) &&
        std::isfinite(curvature) && std::isfinite(curvature_w);
    certificate.complete = certificate.valid;
    return certificate.valid;
}

bool CollectSplineBernsteinControls(const UniformBspline& spline,
                                    const double t0, const double t1,
                                    std::vector<Eigen::Vector3d>& controls)
{
    controls.clear();
    if (!std::isfinite(t0) || !std::isfinite(t1) || t1 < t0 ||
        !std::isfinite(spline.beta_) || spline.beta_ <= 0.0 ||
        spline.p_ < 0 || spline.m_ <= spline.p_ ||
        spline.u_.size() <= spline.m_ || spline.control_points_.cols() != 3 ||
        spline.control_points_.rows() == 0) {
        return false;
    }
    const double u0 = t0 * spline.beta_ + spline.u_(spline.p_);
    const double u1 = t1 * spline.beta_ + spline.u_(spline.p_);
    const int first_span = spline.p_;
    const int last_span = spline.m_ - spline.p_ - 1;
    if (!std::isfinite(u0) || !std::isfinite(u1) || last_span < first_span ||
        u0 < spline.u_(first_span) - kDomainEps ||
        u1 > spline.u_(last_span + 1) + kDomainEps) {
        return false;
    }
    std::vector<bool> selected(
        static_cast<std::size_t>(spline.control_points_.rows()), false);
    for (int span = first_span; span <= last_span; ++span) {
        if (spline.u_(span + 1) < u0 - kDomainEps ||
            spline.u_(span) > u1 + kDomainEps) {
            continue;
        }
        const int control_first = span - spline.p_;
        const int control_last = span;
        if (control_first < 0 || control_last >= spline.control_points_.rows()) {
            return false;
        }
        for (int index = control_first; index <= control_last; ++index) {
            selected[static_cast<std::size_t>(index)] = true;
        }
    }
    for (std::size_t index = 0U; index < selected.size(); ++index) {
        if (!selected[index]) continue;
        const Eigen::Vector3d control = spline.control_points_.row(
            static_cast<Eigen::Index>(index));
        if (!control.allFinite()) return false;
        controls.push_back(control);
    }
    return !controls.empty();
}

bool BoundsFromSpline(const UniformBspline& spline, const double t0,
                      const double t1, VectorBounds& full,
                      VectorBounds& horizontal)
{
    std::vector<Eigen::Vector3d> controls;
    if (!CollectSplineBernsteinControls(spline, t0, t1, controls)) return false;
    full = BoundsFromBernsteinControls(controls, false);
    horizontal = BoundsFromBernsteinControls(controls, true);
    return full.valid && horizontal.valid;
}

bool MakeQuinticDifferentialBounds(
    const std::array<Eigen::Vector3d, 6>& a,
    const double segment_w0, const double segment_w1,
    const double w0, const double w1,
    const Eigen::Vector3d* constant_horizontal_dp_dw,
    DifferentialBounds& differential)
{
    differential = DifferentialBounds();
    if (!std::isfinite(segment_w0) || !std::isfinite(segment_w1) ||
        !std::isfinite(w0) || !std::isfinite(w1) ||
        w0 < segment_w0 - kDomainEps || w1 > segment_w1 + kDomainEps ||
        w1 <= w0 + kDomainEps) {
        return false;
    }
    const double h_segment = segment_w1 - segment_w0;
    if (h_segment <= kDomainEps) return false;
    const double first = std::max(0.0, std::min(1.0,
        (w0 - segment_w0) / h_segment));
    const double last = std::max(0.0, std::min(1.0,
        (w1 - segment_w0) / h_segment));
    std::vector<Eigen::Vector3d> position(a.begin(), a.end());
    const std::vector<Eigen::Vector3d> p_w = DifferentiateVectorPower(position);
    const std::vector<Eigen::Vector3d> p_ww = DifferentiateVectorPower(p_w);
    const std::vector<Eigen::Vector3d> p_www = DifferentiateVectorPower(p_ww);
    const VectorBounds speed = BoundsFromVectorPower(p_w, first, last, false);
    const VectorBounds horizontal_speed = BoundsFromVectorPower(
        p_w, first, last, true);
    const VectorBounds acceleration = BoundsFromVectorPower(
        p_ww, first, last, false);
    const VectorBounds horizontal_acceleration = BoundsFromVectorPower(
        p_ww, first, last, true);
    const VectorBounds jerk = BoundsFromVectorPower(p_www, first, last, false);
    const VectorBounds horizontal_jerk = BoundsFromVectorPower(
        p_www, first, last, true);
    if (!speed.valid || !horizontal_speed.valid || !acceleration.valid ||
        !horizontal_acceleration.valid || !jerk.valid ||
        !horizontal_jerk.valid) {
        return false;
    }
    differential.inf_speed = ScaledLowerBound(
        speed.inf_norm, speed.inf_norm_exact, h_segment);
    if (constant_horizontal_dp_dw != nullptr) {
        const Eigen::Vector3d horizontal_derivative(
            constant_horizontal_dp_dw->x(),
            constant_horizontal_dp_dw->y(), 0.0);
        const double direct_horizontal_speed =
            DirectedLowerNorm(horizontal_derivative, true);
        if (!std::isfinite(direct_horizontal_speed) ||
            direct_horizontal_speed < 0.0) {
            return false;
        }
        // The endpoint derivative is already expressed in phase-w units.
        // Preserve it directly instead of recovering q through a1/h, whose
        // arbitrary-span division can round an immediately-above-threshold
        // value down to the threshold.
        differential.inf_horizontal_speed = direct_horizontal_speed;
    } else {
        differential.inf_horizontal_speed = ScaledLowerBound(
            horizontal_speed.inf_norm, horizontal_speed.inf_norm_exact,
            h_segment);
    }
    differential.sup_speed = UpperBound(speed.sup_norm / h_segment);
    differential.sup_acceleration = UpperBound(
        acceleration.sup_norm / (h_segment * h_segment));
    differential.sup_horizontal_acceleration = UpperBound(
        horizontal_acceleration.sup_norm / (h_segment * h_segment));
    differential.sup_jerk = UpperBound(
        jerk.sup_norm / (h_segment * h_segment * h_segment));
    differential.sup_horizontal_jerk = UpperBound(
        horizontal_jerk.sup_norm / (h_segment * h_segment * h_segment));
    differential.valid = std::isfinite(differential.inf_speed) &&
        std::isfinite(differential.inf_horizontal_speed) &&
        std::isfinite(differential.sup_speed) &&
        differential.sup_speed >= differential.inf_speed;
    return differential.valid;
}

bool MakeQuinticCertificate(
    const std::array<Eigen::Vector3d, 6>& a, const double segment_w0,
    const double segment_w1, const double w0, const double w1,
    const Eigen::Vector3d* constant_horizontal_dp_dw,
    phase_offset_core::PathCellGeometryCertificate& certificate)
{
    if (!std::isfinite(segment_w0) || !std::isfinite(segment_w1) ||
        !std::isfinite(w0) || !std::isfinite(w1) ||
        w0 < segment_w0 - kDomainEps || w1 > segment_w1 + kDomainEps ||
        w1 <= w0 + kDomainEps) {
        return false;
    }
    DifferentialBounds differential;
    if (MakeQuinticDifferentialBounds(a, segment_w0, segment_w1,
                                      w0, w1, constant_horizontal_dp_dw,
                                      differential) &&
        MakeCertificate(w0, w1, differential, certificate)) {
        return true;
    }
    // Bernstein control polygons can contain the origin even when the
    // quintic's derivative is strictly nonzero on the requested cell.  Split
    // only the certificate calculation into deterministic subcells, aggregate
    // their valid bounds, and keep the same fail-closed semantics if any
    // subcell still has no positive speed proof.
    constexpr int kSubdivisions = 32;
    DifferentialBounds aggregate;
    aggregate.inf_speed = std::numeric_limits<double>::infinity();
    aggregate.inf_horizontal_speed = std::numeric_limits<double>::infinity();
    aggregate.sup_speed = 0.0;
    aggregate.sup_acceleration = 0.0;
    aggregate.sup_horizontal_acceleration = 0.0;
    aggregate.sup_jerk = 0.0;
    aggregate.sup_horizontal_jerk = 0.0;
    aggregate.valid = true;
    const double span = w1 - w0;
    for (int index = 0; index < kSubdivisions; ++index) {
        const double local_w0 = w0 + span * static_cast<double>(index) /
            static_cast<double>(kSubdivisions);
        const double local_w1 = w0 + span * static_cast<double>(index + 1) /
            static_cast<double>(kSubdivisions);
        DifferentialBounds local;
        if (!MakeQuinticDifferentialBounds(a, segment_w0, segment_w1,
                                           local_w0, local_w1,
                                           constant_horizontal_dp_dw,
                                           local)) {
            aggregate.valid = false;
            break;
        }
        aggregate.inf_speed = std::min(aggregate.inf_speed, local.inf_speed);
        aggregate.inf_horizontal_speed = std::min(
            aggregate.inf_horizontal_speed, local.inf_horizontal_speed);
        aggregate.sup_speed = std::max(aggregate.sup_speed, local.sup_speed);
        aggregate.sup_acceleration = std::max(
            aggregate.sup_acceleration, local.sup_acceleration);
        aggregate.sup_horizontal_acceleration = std::max(
            aggregate.sup_horizontal_acceleration,
            local.sup_horizontal_acceleration);
        aggregate.sup_jerk = std::max(aggregate.sup_jerk, local.sup_jerk);
        aggregate.sup_horizontal_jerk = std::max(
            aggregate.sup_horizontal_jerk, local.sup_horizontal_jerk);
    }
    if (!aggregate.valid || !std::isfinite(aggregate.inf_speed) ||
        !std::isfinite(aggregate.inf_horizontal_speed)) {
        return false;
    }
    return MakeCertificate(w0, w1, aggregate, certificate);
}

bool MakeV2Certificate(
    const double w0, const double w1, const double anchor_w,
    const std::array<ProofInterval, 3U>& anchor_position,
    const std::array<ProofInterval, 3U>& anchor_p_w,
    const std::array<ProofInterval, 3U>& anchor_p_ww,
    const ProofInterval& inf_speed, const ProofInterval& sup_speed,
    const ProofInterval& inf_horizontal_speed,
    const ProofInterval& sup_acceleration,
    const ProofInterval& sup_horizontal_acceleration,
    const ProofInterval& sup_jerk,
    const bool phase_map_complete,
    phase_offset_core::CertifiedPathCellV2& certificate)
{
    certificate = phase_offset_core::CertifiedPathCellV2();
    if (!ProofFloatingPointEnvironmentSupported() ||
        !std::isfinite(w0) || !std::isfinite(w1) || !(w1 > w0) ||
        !std::isfinite(anchor_w) || anchor_w < w0 || anchor_w > w1 ||
        !phase_map_complete || !ProofVectorFinite(anchor_position) ||
        !ProofVectorFinite(anchor_p_w) || !ProofVectorFinite(anchor_p_ww) ||
        !ProofFinite(inf_speed) || !ProofFinite(sup_speed) ||
        !ProofFinite(inf_horizontal_speed) || !ProofFinite(sup_acceleration) ||
        !ProofFinite(sup_horizontal_acceleration) || !ProofFinite(sup_jerk)) {
        return false;
    }
    const ProofInterval span = ProofSub(PointProofInterval(w1),
                                        PointProofInterval(w0));
    const ProofInterval half = ProofIntervalFromBounds(0.5, 0.5);
    const ProofInterval normal_rate = ProofDiv(
        sup_horizontal_acceleration, inf_horizontal_speed);
    const ProofInterval tangent_rate = ProofDiv(sup_acceleration, inf_speed);
    const ProofInterval curvature = ProofDiv(
        sup_acceleration, ProofMul(inf_speed, inf_speed));
    const ProofInterval curvature_rate = ProofAdd(
        ProofDiv(sup_jerk, ProofMul(inf_speed, inf_speed)),
        ProofMul(ProofIntervalFromBounds(3.0, 3.0),
                 ProofDiv(ProofMul(sup_acceleration, sup_acceleration),
                          ProofMul(ProofMul(inf_speed, inf_speed), inf_speed))));
    const ProofInterval normal_variation = ProofMul(normal_rate, span);
    const ProofInterval tangent_variation = ProofMul(tangent_rate, span);
    const ProofInterval curvature_variation = ProofMul(curvature_rate, span);
    const ProofInterval midpoint_variation = ProofMul(
        ProofMul(sup_speed, span), half);
    const ProofInterval chord_deviation = ProofDiv(
        ProofMul(ProofMul(sup_acceleration, ProofMul(span, span)), half),
        ProofIntervalFromBounds(4.0, 4.0));
    if (!ProofFinite(normal_rate) || !ProofFinite(tangent_rate) ||
        !ProofFinite(curvature) || !ProofFinite(curvature_rate) ||
        !ProofFinite(normal_variation) || !ProofFinite(tangent_variation) ||
        !ProofFinite(curvature_variation) || !ProofFinite(midpoint_variation) ||
        !ProofFinite(chord_deviation) || inf_speed.lower <= 0.0 ||
        inf_horizontal_speed.lower <=
            phase_offset_core::kHorizontalNormalSpeedEpsilon ||
        sup_speed.upper < inf_speed.lower) {
        return false;
    }

    SetPublicVectorInterval(anchor_position, certificate.anchor_position);
    SetPublicVectorInterval(anchor_p_w, certificate.anchor_p_w);
    SetPublicVectorInterval(anchor_p_ww, certificate.anchor_p_ww);
    if (!certificate.anchor_position.valid || !certificate.anchor_p_w.valid ||
        !certificate.anchor_p_ww.valid) {
        return false;
    }
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.anchor_w = anchor_w;
    certificate.inf_p_w_norm = ToPublicInterval(inf_speed);
    certificate.sup_p_w_norm = ToPublicInterval(sup_speed);
    certificate.inf_horizontal_p_w_norm = ToPublicInterval(
        inf_horizontal_speed);
    certificate.sup_p_ww_norm = ToPublicInterval(sup_acceleration);
    certificate.sup_horizontal_p_ww_norm = ToPublicInterval(
        sup_horizontal_acceleration);
    certificate.sup_p_www_norm = ToPublicInterval(sup_jerk);
    certificate.sup_normal_derivative = ToPublicInterval(normal_rate);
    certificate.normal_variation = ToPublicInterval(normal_variation);
    certificate.tangent_variation = ToPublicInterval(tangent_variation);
    certificate.curvature_variation = ToPublicInterval(curvature_variation);
    certificate.midpoint_position_variation = ToPublicInterval(
        midpoint_variation);
    certificate.chord_deviation = ToPublicInterval(chord_deviation);
    certificate.horizontal_acceleration_bound_complete = true;
    certificate.normal_frame_proof_complete = true;
    certificate.phase_map_proof_complete = phase_map_complete;
    certificate.provenance =
        phase_offset_core::kWorldHorizontalCrossProductProvenance;
    // The producer has not yet been bound to a ContinuousPhasePath segment;
    // the owner wrapper stamps the actual identity/revision after this
    // callback returns.  Use a nonzero local identity so the value checker can
    // validate all numerical fields before that binding step.
    certificate.segment_identity = 1U;
    certificate.proof_identity = 1U;
    certificate.valid = true;
    certificate.complete = true;
    certificate.complete = phase_offset_core::certifiedPathCellV2IsComplete(
        certificate);
    return certificate.complete;
}

bool MakeQuinticV2Certificate(
    const std::array<Eigen::Vector3d, 6U>& coefficients,
    const double segment_w0, const double segment_w1,
    const double w0, const double w1,
    const Eigen::Vector3d* constant_horizontal_dp_dw,
    phase_offset_core::CertifiedPathCellV2& certificate)
{
    certificate = phase_offset_core::CertifiedPathCellV2();
    if (!ProofFloatingPointEnvironmentSupported() ||
        !std::isfinite(segment_w0) || !std::isfinite(segment_w1) ||
        !std::isfinite(w0) || !std::isfinite(w1) || !(segment_w1 > segment_w0) ||
        w0 < segment_w0 || w1 > segment_w1 || !(w1 > w0)) {
        return false;
    }
    const ProofInterval segment_span = ProofSub(
        PointProofInterval(segment_w1), PointProofInterval(segment_w0));
    const ProofInterval inverse_span = ProofDiv(
        PointProofInterval(1.0), segment_span);
    const ProofInterval first = ProofMul(
        ProofSub(PointProofInterval(w0), PointProofInterval(segment_w0)),
        inverse_span);
    const ProofInterval last = ProofMul(
        ProofSub(PointProofInterval(w1), PointProofInterval(segment_w0)),
        inverse_span);
    if (!ProofFinite(first) || !ProofFinite(last)) return false;
    const std::vector<Eigen::Vector3d> position(coefficients.begin(),
                                                coefficients.end());
    const ProofVectorPolynomial position_poly =
        ProofPolynomialFromEigen(position);
    const ProofVectorPolynomial p_w_poly =
        ProofDifferentiatePolynomial(position_poly);
    const ProofVectorPolynomial p_ww_poly =
        ProofDifferentiatePolynomial(p_w_poly);
    const ProofVectorPolynomial p_www_poly =
        ProofDifferentiatePolynomial(p_ww_poly);
    // The proof contract anchors at the mathematical midpoint of the closed
    // cell.  Keep that midpoint as an outward interval even when the phase
    // coordinate itself is too large for the midpoint to be represented (the
    // owner exposes a rounded diagnostic double below).  Also hull the phase
    // obtained from that diagnostic value so existing consumers querying
    // certificate.anchor_w remain enclosed by the same evidence.
    const double anchor_w_value = w0 + 0.5 * (w1 - w0);
    const ProofInterval mathematical_anchor_s = ProofMul(
        PointProofInterval(0.5), ProofAdd(first, last));
    const ProofInterval rounded_anchor_s = ProofDiv(
        ProofSub(PointProofInterval(anchor_w_value),
                 PointProofInterval(segment_w0)),
        segment_span);
    if (!ProofFinite(mathematical_anchor_s) ||
        !ProofFinite(rounded_anchor_s)) return false;
    const ProofInterval anchor_s = ProofIntervalFromBounds(
        std::min(mathematical_anchor_s.lower, rounded_anchor_s.lower),
        std::max(mathematical_anchor_s.upper, rounded_anchor_s.upper));
    if (!ProofFinite(anchor_s)) return false;
    std::array<ProofInterval, 3U> anchor_position =
        ProofVectorPolynomialEvaluate(position_poly, anchor_s);
    std::array<ProofInterval, 3U> anchor_p_w =
        ProofVectorPolynomialEvaluate(p_w_poly, anchor_s);
    std::array<ProofInterval, 3U> anchor_p_ww =
        ProofVectorPolynomialEvaluate(p_ww_poly, anchor_s);
    anchor_p_w = ProofVectorScale(anchor_p_w, inverse_span);
    const ProofInterval inverse_span_squared = ProofMul(inverse_span,
                                                         inverse_span);
    anchor_p_ww = ProofVectorScale(anchor_p_ww, inverse_span_squared);

    // Restricting a power polynomial to [first,last] and evaluating it with
    // interval Horner is a complete closed-cell enclosure.  The subdivision
    // is proof-only and does not alter the nominal evaluator.
    const int subdivisions = 32;
    ProofInterval inf_speed = ProofIntervalFromBounds(
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity());
    ProofInterval inf_horizontal = inf_speed;
    ProofInterval sup_speed = PointProofInterval(0.0);
    ProofInterval sup_acceleration = PointProofInterval(0.0);
    ProofInterval sup_horizontal_acceleration = PointProofInterval(0.0);
    ProofInterval sup_jerk = PointProofInterval(0.0);
    bool first_bound = true;
    for (int index = 0; index < subdivisions; ++index) {
        const ProofInterval fraction0 = ProofDiv(
            PointProofInterval(static_cast<double>(index)),
            PointProofInterval(static_cast<double>(subdivisions)));
        const ProofInterval fraction1 = ProofDiv(
            PointProofInterval(static_cast<double>(index + 1)),
            PointProofInterval(static_cast<double>(subdivisions)));
        const ProofInterval x = ProofAdd(first,
            ProofMul(ProofSub(last, first), fraction0));
        const ProofInterval x_end = ProofAdd(first,
            ProofMul(ProofSub(last, first), fraction1));
        if (!ProofFinite(x) || !ProofFinite(x_end)) return false;
        const ProofInterval x_cell = ProofIntervalFromBounds(x.lower,
                                                              x_end.upper);
        std::array<ProofInterval, 3U> v = ProofVectorScale(
            ProofVectorPolynomialEvaluate(p_w_poly, x_cell), inverse_span);
        std::array<ProofInterval, 3U> a = ProofVectorScale(
            ProofVectorPolynomialEvaluate(p_ww_poly, x_cell), inverse_span_squared);
        const ProofInterval inverse_span_cubed = ProofMul(
            inverse_span_squared, inverse_span);
        std::array<ProofInterval, 3U> j = ProofVectorScale(
            ProofVectorPolynomialEvaluate(p_www_poly, x_cell), inverse_span_cubed);
        if (constant_horizontal_dp_dw != nullptr) {
            // The nominal evaluator replaces the generic quintic horizontal
            // branch with these exact stored components at every phase.  Do
            // the same substitution before *all* norm extrema (not only the
            // horizontal floor), so full speed/acceleration/jerk evidence
            // covers the executed branch rather than depending on algebraic
            // equivalence plus incidental rounding.
            v[0] = PointProofInterval(constant_horizontal_dp_dw->x());
            v[1] = PointProofInterval(constant_horizontal_dp_dw->y());
            a[0] = PointProofInterval(0.0);
            a[1] = PointProofInterval(0.0);
            j[0] = PointProofInterval(0.0);
            j[1] = PointProofInterval(0.0);
        }
        std::array<ProofInterval, 3U> vh = v;
        std::array<ProofInterval, 3U> ah = a;
        vh[2] = PointProofInterval(0.0);
        ah[2] = PointProofInterval(0.0);
        const ProofInterval speed = ProofNorm(v);
        const ProofInterval horizontal_speed = ProofNorm(vh);
        const ProofInterval acceleration = ProofNorm(a);
        const ProofInterval horizontal_acceleration = ProofNorm(ah);
        const ProofInterval jerk = ProofNorm(j);
        if (!ProofFinite(speed) || !ProofFinite(horizontal_speed) ||
            !ProofFinite(acceleration) || !ProofFinite(horizontal_acceleration) ||
            !ProofFinite(jerk)) return false;
        if (first_bound) {
            inf_speed = speed;
            inf_horizontal = horizontal_speed;
            first_bound = false;
        } else {
            inf_speed.lower = std::min(inf_speed.lower, speed.lower);
            inf_horizontal.lower = std::min(inf_horizontal.lower,
                                             horizontal_speed.lower);
        }
        sup_speed.upper = std::max(sup_speed.upper, speed.upper);
        sup_acceleration.upper = std::max(sup_acceleration.upper,
                                          acceleration.upper);
        sup_horizontal_acceleration.upper = std::max(
            sup_horizontal_acceleration.upper, horizontal_acceleration.upper);
        sup_jerk.upper = std::max(sup_jerk.upper, jerk.upper);
    }
    if (first_bound) return false;
    inf_speed.valid = true;
    inf_horizontal.valid = true;
    sup_speed.valid = true;
    sup_acceleration.valid = true;
    sup_horizontal_acceleration.valid = true;
    sup_jerk.valid = true;
    if (constant_horizontal_dp_dw != nullptr) {
        const Eigen::Vector3d horizontal(constant_horizontal_dp_dw->x(),
                                         constant_horizontal_dp_dw->y(), 0.0);
        // Preserve the exact direct-branch floor at the strict capability
        // threshold.  DirectedLowerNorm is an exact binary64-squared oracle
        // (not a guessed epsilon); its greatest representable value below
        // the mathematical norm is paired with the next representable upper
        // endpoint, so a one-ULP-above-threshold constant remains certifiable.
        const double exact_horizontal_lower = DirectedLowerNorm(horizontal, true);
        if (!std::isfinite(exact_horizontal_lower) ||
            exact_horizontal_lower < 0.0) return false;
        const double exact_horizontal_upper = exact_horizontal_lower == 0.0
            ? 0.0
            : std::nextafter(exact_horizontal_lower,
                             std::numeric_limits<double>::infinity());
        const ProofInterval exact_horizontal = ProofIntervalFromBounds(
            exact_horizontal_lower, exact_horizontal_upper);
        if (!ProofFinite(exact_horizontal)) return false;
        inf_horizontal = exact_horizontal;
        // The nominal point evaluator has an explicit horizontal derivative
        // override.  Its proof must use the same value for anchor and every
        // horizontal acceleration bound, rather than the generic quintic
        // coefficients that are algebraically equivalent only in exact real
        // arithmetic.
        anchor_position[0] = ProofAdd(PointProofInterval(coefficients[0].x()),
            ProofMul(PointProofInterval(constant_horizontal_dp_dw->x()),
                     ProofMul(segment_span, anchor_s)));
        anchor_position[1] = ProofAdd(PointProofInterval(coefficients[0].y()),
            ProofMul(PointProofInterval(constant_horizontal_dp_dw->y()),
                     ProofMul(segment_span, anchor_s)));
        anchor_p_w[0] = PointProofInterval(constant_horizontal_dp_dw->x());
        anchor_p_w[1] = PointProofInterval(constant_horizontal_dp_dw->y());
        anchor_p_ww[0] = PointProofInterval(0.0);
        anchor_p_ww[1] = PointProofInterval(0.0);
        sup_horizontal_acceleration = PointProofInterval(0.0);
    }
    return MakeV2Certificate(w0, w1, anchor_w_value, anchor_position,
        anchor_p_w, anchor_p_ww,
        inf_speed, sup_speed, inf_horizontal, sup_acceleration,
        sup_horizontal_acceleration, sup_jerk, true, certificate);
}

bool MakeMappedBsplineCertificate(
    const UniformBspline& p, const UniformBspline& dp_dt,
    const UniformBspline& d2p_dt2, const UniformBspline& d3p_dt3,
    const ArcLengthMap& map, const double spline_t_anchor,
    const double spline_t_end, const double phase_w_anchor,
    const double phase_w_end, const double arclength_per_phase,
    const double w0, const double w1,
    phase_offset_core::PathCellGeometryCertificate& certificate)
{
    (void)p;
    if (!std::isfinite(w0) || !std::isfinite(w1) ||
        w0 < phase_w_anchor - kDomainEps || w1 > phase_w_end + kDomainEps ||
        w1 <= w0 + kDomainEps || !std::isfinite(arclength_per_phase) ||
        arclength_per_phase <= 0.0 || map.cells.empty() ||
        map.table_s.size() != map.cells.size() + 1U ||
        map.table_t.size() != map.table_s.size()) {
        return false;
    }
    const double total_length = map.table_s.back();
    const double target_s0 = std::max(0.0, std::min(total_length,
        arclength_per_phase * (w0 - phase_w_anchor)));
    const double target_s1 = std::max(0.0, std::min(total_length,
        arclength_per_phase * (w1 - phase_w_anchor)));
    if (!std::isfinite(total_length) || !std::isfinite(target_s0) ||
        !std::isfinite(target_s1) || target_s1 < target_s0) {
        return false;
    }
    double enclosed_t0 = std::numeric_limits<double>::infinity();
    double enclosed_t1 = -std::numeric_limits<double>::infinity();
    double q_min = std::numeric_limits<double>::infinity();
    double q_max = 0.0;
    double q_prime_max = 0.0;
    double q_second_max = 0.0;
    bool has_cell = false;
    // Select exactly the arc-length cells intersected by [target_s0,target_s1].
    // `upper_bound(...)-1` gives the containing cell for an interior query and
    // the cell immediately preceding an exact table knot.  The previous code
    // used a lower_bound for the upper endpoint and then added neighbours on
    // both sides; that admitted unrelated cells (and their conservative
    // derivative controls) into the certificate, so one locally valid cell
    // could fail because a neighbouring map cell had a non-positive Bernstein
    // speed control.  Exact-boundary cells are still included through the
    // preceding-cell convention without weakening the interval proof.
    const auto first_table = std::upper_bound(
        map.table_s.begin(), map.table_s.end(), target_s0);
    const auto last_table = std::upper_bound(
        map.table_s.begin(), map.table_s.end(), target_s1);
    std::size_t begin_index = first_table == map.table_s.begin()
        ? 0U : static_cast<std::size_t>(
            std::distance(map.table_s.begin(), first_table) - 1);
    std::size_t end_index = last_table == map.table_s.begin()
        ? 0U : static_cast<std::size_t>(
            std::distance(map.table_s.begin(), last_table) - 1);
    begin_index = std::min(begin_index, map.cells.size() - 1U);
    end_index = std::min(end_index, map.cells.size() - 1U);
    for (std::size_t index = begin_index; index <= end_index; ++index) {
        const double s0 = map.table_s[index];
        const double s1 = map.table_s[index + 1U];
        if (!std::isfinite(s0) || !std::isfinite(s1) || s1 < target_s0 ||
            s0 > target_s1) {
            continue;
        }
        const ArcLengthCell& cell = map.cells[index];
        if (!std::isfinite(cell.h) || cell.h <= 0.0) return false;
        const std::vector<double> map_power(cell.a.begin(), cell.a.end());
        const std::vector<double> first = DifferentiateScalarPower(map_power);
        const std::vector<double> second = DifferentiateScalarPower(first);
        const std::vector<double> third = DifferentiateScalarPower(second);
        const std::vector<double> q_controls = ScalarPowerToBernstein(first);
        const std::vector<double> q_prime_controls =
            ScalarPowerToBernstein(second);
        const std::vector<double> q_second_controls =
            ScalarPowerToBernstein(third);
        if (q_controls.empty() || q_prime_controls.empty() ||
            q_second_controls.empty()) {
            return false;
        }
        for (const double coefficient : q_controls) {
            if (!std::isfinite(coefficient) || coefficient <= 0.0) return false;
            q_min = std::min(q_min, coefficient / cell.h);
            q_max = std::max(q_max, coefficient / cell.h);
        }
        for (const double coefficient : q_prime_controls) {
            if (!std::isfinite(coefficient)) return false;
            q_prime_max = std::max(q_prime_max,
                std::abs(coefficient) / (cell.h * cell.h));
        }
        for (const double coefficient : q_second_controls) {
            if (!std::isfinite(coefficient)) return false;
            q_second_max = std::max(q_second_max,
                std::abs(coefficient) / (cell.h * cell.h * cell.h));
        }
        enclosed_t0 = std::min(enclosed_t0, map.table_t[index]);
        enclosed_t1 = std::max(enclosed_t1, map.table_t[index + 1U]);
        has_cell = true;
    }
    if (!has_cell || !std::isfinite(enclosed_t0) ||
        !std::isfinite(enclosed_t1) || enclosed_t0 < spline_t_anchor - kDomainEps ||
        enclosed_t1 > spline_t_end + kDomainEps) {
        return false;
    }
    q_min = LowerBound(q_min);
    q_max = UpperBound(q_max);
    q_prime_max = UpperBound(q_prime_max);
    q_second_max = UpperBound(q_second_max);
    if (!std::isfinite(q_min) || !std::isfinite(q_max) ||
        !std::isfinite(q_prime_max) || !std::isfinite(q_second_max) ||
        q_min <= kCertificateSpeedEps || q_max < q_min) {
        return false;
    }
    VectorBounds p_t;
    VectorBounds p_t_h;
    VectorBounds p_tt;
    VectorBounds p_tt_h;
    VectorBounds p_ttt;
    VectorBounds p_ttt_h;
    if (!BoundsFromSpline(dp_dt, enclosed_t0, enclosed_t1, p_t, p_t_h) ||
        !BoundsFromSpline(d2p_dt2, enclosed_t0, enclosed_t1, p_tt, p_tt_h) ||
        !BoundsFromSpline(d3p_dt3, enclosed_t0, enclosed_t1, p_ttt, p_ttt_h)) {
        return false;
    }
    const double dt_dw_max = UpperBound(arclength_per_phase / q_min);
    const double dt_dw_min = LowerBound(arclength_per_phase / q_max);
    const double d2t_dw2_max = UpperBound(
        arclength_per_phase * arclength_per_phase * q_prime_max /
        (q_min * q_min * q_min));
    const double d3t_dw3_max = UpperBound(
        arclength_per_phase * arclength_per_phase * arclength_per_phase *
        (3.0 * q_prime_max * q_prime_max /
             (q_min * q_min * q_min * q_min * q_min) +
         q_second_max / (q_min * q_min * q_min * q_min)));
    if (!std::isfinite(dt_dw_max) || !std::isfinite(dt_dw_min) ||
        !std::isfinite(d2t_dw2_max) || !std::isfinite(d3t_dw3_max)) {
        return false;
    }
    DifferentialBounds differential;
    differential.inf_speed = ProductLowerBound(
        p_t.inf_norm, p_t.inf_norm_exact, dt_dw_min);
    differential.inf_horizontal_speed = ProductLowerBound(
        p_t_h.inf_norm, p_t_h.inf_norm_exact, dt_dw_min);
    differential.sup_speed = UpperBound(p_t.sup_norm * dt_dw_max);
    differential.sup_acceleration = UpperBound(
        p_tt.sup_norm * dt_dw_max * dt_dw_max +
        p_t.sup_norm * d2t_dw2_max);
    differential.sup_horizontal_acceleration = UpperBound(
        p_tt_h.sup_norm * dt_dw_max * dt_dw_max +
        p_t_h.sup_norm * d2t_dw2_max);
    differential.sup_jerk = UpperBound(
        p_ttt.sup_norm * dt_dw_max * dt_dw_max * dt_dw_max +
        3.0 * p_tt.sup_norm * dt_dw_max * d2t_dw2_max +
        p_t.sup_norm * d3t_dw3_max);
    differential.sup_horizontal_jerk = UpperBound(
        p_ttt_h.sup_norm * dt_dw_max * dt_dw_max * dt_dw_max +
        3.0 * p_tt_h.sup_norm * dt_dw_max * d2t_dw2_max +
        p_t_h.sup_norm * d3t_dw3_max);
    differential.valid = std::isfinite(differential.inf_speed) &&
        std::isfinite(differential.inf_horizontal_speed) &&
        std::isfinite(differential.sup_speed) &&
        std::isfinite(differential.sup_acceleration) &&
        std::isfinite(differential.sup_horizontal_acceleration) &&
        std::isfinite(differential.sup_jerk) &&
        std::isfinite(differential.sup_horizontal_jerk);
    return MakeCertificate(w0, w1, differential, certificate);
}

// The executable arclength map is preferred because it removes the planner's
// time-parameterization from phase speed.  Some valid planner splines still
// cannot produce that map (for example a stationary raw endpoint or a
// numerically non-positive speed at one table sample).  The point evaluator
// deliberately retains the historical linear t(w) fallback in that case.
// Keep the fallback generic and certifiable on cells whose actual mapped
// B-spline derivative has a positive lower bound; otherwise it remains
// fail-closed and the caller keeps the historical inset.
bool MakeLinearMappedBsplineCertificate(
    const UniformBspline& dp_dt, const UniformBspline& d2p_dt2,
    const UniformBspline& d3p_dt3, const double spline_t_anchor,
    const double spline_t_end, const double phase_w_anchor,
    const double phase_w_end, const double w0, const double w1,
    phase_offset_core::PathCellGeometryCertificate& certificate)
{
    certificate = phase_offset_core::PathCellGeometryCertificate();
    if (!std::isfinite(spline_t_anchor) || !std::isfinite(spline_t_end) ||
        !std::isfinite(phase_w_anchor) || !std::isfinite(phase_w_end) ||
        !std::isfinite(w0) || !std::isfinite(w1) ||
        spline_t_end <= spline_t_anchor + kDomainEps ||
        phase_w_end <= phase_w_anchor + kDomainEps ||
        w0 < phase_w_anchor - kDomainEps ||
        w1 > phase_w_end + kDomainEps ||
        w1 <= w0 + kDomainEps) {
        return false;
    }
    const double dt_dw = (spline_t_end - spline_t_anchor) /
        (phase_w_end - phase_w_anchor);
    const double t0 = spline_t_anchor + dt_dw * (w0 - phase_w_anchor);
    const double t1 = spline_t_anchor + dt_dw * (w1 - phase_w_anchor);
    if (!std::isfinite(dt_dw) || dt_dw <= 0.0 ||
        !std::isfinite(t0) || !std::isfinite(t1) ||
        t1 <= t0 + kDomainEps) {
        return false;
    }
    VectorBounds p_t;
    VectorBounds p_t_h;
    VectorBounds p_tt;
    VectorBounds p_tt_h;
    VectorBounds p_ttt;
    VectorBounds p_ttt_h;
    if (!BoundsFromSpline(dp_dt, t0, t1, p_t, p_t_h) ||
        !BoundsFromSpline(d2p_dt2, t0, t1, p_tt, p_tt_h) ||
        !BoundsFromSpline(d3p_dt3, t0, t1, p_ttt, p_ttt_h)) {
        return false;
    }
    DifferentialBounds differential;
    differential.inf_speed = ProductLowerBound(
        p_t.inf_norm, p_t.inf_norm_exact, dt_dw);
    differential.inf_horizontal_speed = ProductLowerBound(
        p_t_h.inf_norm, p_t_h.inf_norm_exact, dt_dw);
    differential.sup_speed = UpperBound(p_t.sup_norm * dt_dw);
    differential.sup_acceleration = UpperBound(
        p_tt.sup_norm * dt_dw * dt_dw);
    differential.sup_horizontal_acceleration = UpperBound(
        p_tt_h.sup_norm * dt_dw * dt_dw);
    differential.sup_jerk = UpperBound(
        p_ttt.sup_norm * dt_dw * dt_dw * dt_dw);
    differential.sup_horizontal_jerk = UpperBound(
        p_ttt_h.sup_norm * dt_dw * dt_dw * dt_dw);
    differential.valid = std::isfinite(differential.inf_speed) &&
        std::isfinite(differential.inf_horizontal_speed) &&
        std::isfinite(differential.sup_speed) &&
        std::isfinite(differential.sup_acceleration) &&
        std::isfinite(differential.sup_horizontal_acceleration) &&
        std::isfinite(differential.sup_jerk) &&
        std::isfinite(differential.sup_horizontal_jerk);
    return MakeCertificate(w0, w1, differential, certificate);
}

// UniformBspline::getDerivative() in the unchanged source stores
// beta*(P[i+1]-P[i]) and therefore assumes unit-spaced source knots.  The V2
// de Boor proof supports exactly that production representation; mutated,
// non-unit, duplicated, or metadata-inconsistent knot vectors are rejected
// rather than silently proving a different derivative than the nominal
// evaluator executes.
bool SupportedUniformSplineRepresentation(const UniformBspline& spline)
{
    if (spline.p_ < 0 || spline.n_ < 0 ||
        spline.m_ != spline.p_ + spline.n_ + 1 ||
        spline.control_points_.rows() != spline.n_ + 1 ||
        spline.control_points_.cols() != 3 ||
        spline.u_.size() != spline.m_ + 1) {
        return false;
    }
    for (int index = 0; index <= spline.m_; ++index) {
        const double knot = spline.u_(index);
        if (!std::isfinite(knot)) return false;
        if (index > 0 && !ExactBinary64DifferenceEqualsOne(
                spline.u_(index), spline.u_(index - 1))) {
            return false;
        }
    }
    return true;
}

bool SupportedSplineTimeDomain(const UniformBspline& spline,
                               const double t0, const double t1)
{
    if (!SupportedUniformSplineRepresentation(spline) ||
        !std::isfinite(spline.beta_) || spline.beta_ <= 0.0 ||
        !std::isfinite(t0) || !std::isfinite(t1) || t1 <= t0 || t0 < 0.0) {
        return false;
    }
    const int span_count = spline.m_ - 2 * spline.p_;
    const double span = static_cast<double>(span_count);
    return span_count > 0 && std::isfinite(span) &&
        phase_offset_core::binary64ProductLeq(t1, spline.beta_, span);
}

// Call-local preparation only: neither the path nor a shared evaluator is
// mutated. Derivatives still use the original binary64 controls and exactly
// the same outward operations, not the rounded nominal derivative splines.
struct OriginalSplineProofControls {
    const UniformBspline& spline;
    const bool representation_supported;
    std::vector<ProofVectorPolynomial> derivatives;

    explicit OriginalSplineProofControls(const UniformBspline& source)
        : spline(source),
          representation_supported(SupportedUniformSplineRepresentation(source)) {}

    bool prepare(const int derivative_order) {
        if (derivatives.empty()) {
            ProofVectorPolynomial controls;
            controls.reserve(static_cast<std::size_t>(spline.control_points_.rows()));
            for (Eigen::Index row = 0; row < spline.control_points_.rows(); ++row) {
                const Eigen::Vector3d value = spline.control_points_.row(row);
                if (!value.allFinite()) return false;
                controls.push_back(ProofVectorConstant(value));
            }
            derivatives.push_back(std::move(controls));
        }
        while (derivatives.size() <= static_cast<std::size_t>(derivative_order)) {
            const int order = static_cast<int>(derivatives.size()) - 1;
            const int degree = spline.p_ - order;
            const ProofVectorPolynomial& controls = derivatives.back();
            ProofVectorPolynomial next;
            next.resize(controls.size() - 1U);
            for (std::size_t index = 0U; index + 1U < controls.size(); ++index) {
                const int knot_offset = order;
                const ProofInterval left_knot = PointProofInterval(
                    spline.u_(static_cast<Eigen::Index>(index + 1U +
                                                        knot_offset)));
                const ProofInterval right_knot = PointProofInterval(
                    spline.u_(static_cast<Eigen::Index>(index + degree + 1 +
                                                        knot_offset)));
                const ProofInterval denominator = ProofSub(right_knot, left_knot);
                const ProofInterval scale = ProofMul(
                    PointProofInterval(spline.beta_),
                    PointProofInterval(static_cast<double>(degree)));
                const ProofInterval factor = ProofDiv(scale, denominator);
                if (!ProofFinite(factor)) return false;
                for (std::size_t component = 0U; component < 3U; ++component) {
                    next[index][component] = ProofMul(
                        ProofSub(controls[index + 1U][component],
                                 controls[index][component]), factor);
                }
            }
            derivatives.push_back(std::move(next));
        }
        return !derivatives[static_cast<std::size_t>(derivative_order)].empty();
    }
};

bool ProofBoundsFromOriginalSpline(
    OriginalSplineProofControls& prepared, const int derivative_order,
    const double t0, const double t1,
    std::array<ProofInterval, 3U>& full,
    std::array<ProofInterval, 3U>& horizontal)
{
    const UniformBspline& spline = prepared.spline;
    if (derivative_order < 0 || derivative_order > spline.p_ ||
        !std::isfinite(t0) || !std::isfinite(t1) || !(t1 > t0) ||
        !std::isfinite(spline.beta_) || spline.beta_ <= 0.0 ||
        spline.p_ < 0 || spline.m_ <= spline.p_ ||
        spline.u_.size() <= spline.m_ || spline.control_points_.rows() <= 0 ||
        spline.control_points_.cols() != 3 ||
        !prepared.representation_supported) return false;
    const int source_span_count = spline.m_ - 2 * spline.p_;
    const double source_span = static_cast<double>(source_span_count);
    if (source_span_count <= 0 || !std::isfinite(source_span) ||
        t0 < 0.0 ||
        !phase_offset_core::binary64ProductLeq(
            t1, spline.beta_, source_span)) {
        return false;
    }
    if (!prepared.prepare(derivative_order)) return false;
    const ProofVectorPolynomial& controls =
        prepared.derivatives[static_cast<std::size_t>(derivative_order)];
    const int degree = spline.p_ - derivative_order;
    if (controls.empty() || degree < 0) return false;
    const int knot_offset = derivative_order;
    // Each derivative trims one knot from both ends.  The source's derivative
    // constructor correspondingly reduces p_, n_, and m_; derivative_m is
    // the final source-backed knot index after those trims.
    const int derivative_m = spline.m_ - 2 * derivative_order;
    const ProofInterval u0 = ProofAdd(
        ProofMul(PointProofInterval(t0), PointProofInterval(spline.beta_)),
        PointProofInterval(spline.u_(spline.p_)));
    const ProofInterval u1 = ProofAdd(
        ProofMul(PointProofInterval(t1), PointProofInterval(spline.beta_)),
        PointProofInterval(spline.u_(spline.p_)));
    if (!ProofFinite(u0) || !ProofFinite(u1)) return false;
    std::array<ProofInterval, 3U> aggregate;
    std::array<ProofInterval, 3U> aggregate_horizontal;
    for (std::size_t component = 0U; component < 3U; ++component) {
        aggregate[component] = ProofIntervalFromBounds(
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity());
        aggregate_horizontal[component] = aggregate[component];
    }
    bool have_span = false;
    for (int span = degree; span <= derivative_m - degree - 1; ++span) {
        const ProofInterval knot_left = PointProofInterval(
            spline.u_(span + knot_offset));
        const ProofInterval knot_right = PointProofInterval(
            spline.u_(span + 1 + knot_offset));
        if (!ProofFinite(knot_left) || !ProofFinite(knot_right) ||
            !(knot_right.lower > knot_left.upper) ||
            knot_right.upper < u0.lower || knot_left.lower > u1.upper) {
            continue;
        }
        std::vector<std::array<ProofInterval, 3U>> deBoor(
            static_cast<std::size_t>(degree + 1));
        for (int index = 0; index <= degree; ++index) {
            const std::size_t control_index = static_cast<std::size_t>(
                span - degree + index);
            if (control_index >= controls.size()) return false;
            deBoor[static_cast<std::size_t>(index)] = controls[control_index];
        }
        // Restrict de Boor's query to the actual requested parameter
        // interval.  Using the entire knot span here is sound but needlessly
        // widens derivative controls (and can erase a strictly positive speed
        // floor for a small mapped cell).  The intersection endpoints are
        // already outward interval bounds from the source knot/map arithmetic.
        const double query_lower = std::max(knot_left.lower, u0.lower);
        const double query_upper = std::min(knot_right.upper, u1.upper);
        if (!std::isfinite(query_lower) || !std::isfinite(query_upper) ||
            query_lower > query_upper) {
            continue;
        }
        const ProofInterval query = ProofIntervalFromBounds(
            query_lower, query_upper);
        for (int round = 1; round <= degree; ++round) {
            for (int index = degree; index >= round; --index) {
                const int left_index = span + index - degree + knot_offset;
                const int right_index = span + index - round + 1 + knot_offset;
                const ProofInterval left = PointProofInterval(
                    spline.u_(left_index));
                const ProofInterval right = PointProofInterval(
                    spline.u_(right_index));
                const ProofInterval alpha = ProofDiv(ProofSub(query, left),
                                                     ProofSub(right, left));
                if (!ProofFinite(alpha)) return false;
                const ProofInterval one_minus_alpha = ProofSub(
                    PointProofInterval(1.0), alpha);
                for (std::size_t component = 0U; component < 3U; ++component) {
                    deBoor[static_cast<std::size_t>(index)][component] =
                        ProofAdd(
                            ProofMul(one_minus_alpha,
                                     deBoor[static_cast<std::size_t>(index - 1)]
                                         [component]),
                            ProofMul(alpha,
                                     deBoor[static_cast<std::size_t>(index)]
                                         [component]));
                }
            }
        }
        for (std::size_t component = 0U; component < 3U; ++component) {
            const ProofInterval value = deBoor.back()[component];
            if (!ProofFinite(value)) return false;
            if (!have_span) {
                aggregate[component] = value;
                aggregate_horizontal[component] = component < 2U
                    ? value : PointProofInterval(0.0);
            } else {
                aggregate[component].lower = std::min(
                    aggregate[component].lower, value.lower);
                aggregate[component].upper = std::max(
                    aggregate[component].upper, value.upper);
                if (component < 2U) {
                    aggregate_horizontal[component].lower = std::min(
                        aggregate_horizontal[component].lower, value.lower);
                    aggregate_horizontal[component].upper = std::max(
                        aggregate_horizontal[component].upper, value.upper);
                }
            }
        }
        have_span = true;
    }
    if (!have_span) return false;
    for (std::size_t component = 0U; component < 3U; ++component) {
        aggregate[component].valid = true;
        aggregate_horizontal[component].valid = true;
    }
    full = aggregate;
    horizontal = aggregate_horizontal;
    return ProofVectorFinite(full) && ProofVectorFinite(horizontal);
}

bool MakeLinearMappedBsplineV2Certificate(
    const UniformBspline& p, const UniformBspline& dp_dt,
    const UniformBspline& d2p_dt2, const UniformBspline& d3p_dt3,
    const double spline_t_anchor, const double spline_t_end,
    const double phase_w_anchor, const double phase_w_end,
    const double w0, const double w1,
    phase_offset_core::CertifiedPathCellV2& certificate)
{
    (void)dp_dt;
    (void)d2p_dt2;
    (void)d3p_dt3;
    certificate = phase_offset_core::CertifiedPathCellV2();
    if (!ProofFloatingPointEnvironmentSupported() ||
        !std::isfinite(spline_t_anchor) || !std::isfinite(spline_t_end) ||
        !std::isfinite(phase_w_anchor) || !std::isfinite(phase_w_end) ||
        !std::isfinite(w0) || !std::isfinite(w1) || !(spline_t_end > spline_t_anchor) ||
        !(phase_w_end > phase_w_anchor) || w0 < phase_w_anchor ||
        w1 > phase_w_end || !(w1 > w0)) return false;
    if (!SupportedSplineTimeDomain(p, spline_t_anchor, spline_t_end)) {
        return false;
    }
    const ProofInterval phase_span = ProofSub(
        PointProofInterval(phase_w_end), PointProofInterval(phase_w_anchor));
    const ProofInterval time_span = ProofSub(
        PointProofInterval(spline_t_end), PointProofInterval(spline_t_anchor));
    const ProofInterval dt_dw = ProofDiv(time_span, phase_span);
    const ProofInterval t0 = ProofAdd(PointProofInterval(spline_t_anchor),
        ProofMul(dt_dw, ProofSub(PointProofInterval(w0),
                                 PointProofInterval(phase_w_anchor))));
    const ProofInterval t1 = ProofAdd(PointProofInterval(spline_t_anchor),
        ProofMul(dt_dw, ProofSub(PointProofInterval(w1),
                                 PointProofInterval(phase_w_anchor))));
    if (!ProofFinite(dt_dw) || !ProofFinite(t0) || !ProofFinite(t1) ||
        !(t1.upper > t0.lower)) return false;
    std::array<ProofInterval, 3U> p_bounds;
    std::array<ProofInterval, 3U> p_h_bounds;
    std::array<ProofInterval, 3U> p_t;
    std::array<ProofInterval, 3U> p_t_h;
    std::array<ProofInterval, 3U> p_tt;
    std::array<ProofInterval, 3U> p_tt_h;
    std::array<ProofInterval, 3U> p_ttt;
    std::array<ProofInterval, 3U> p_ttt_h;
    OriginalSplineProofControls prepared(p);
    if (!ProofBoundsFromOriginalSpline(prepared, 0, t0.lower, t1.upper, p_bounds,
                               p_h_bounds) ||
        !ProofBoundsFromOriginalSpline(prepared, 1, t0.lower, t1.upper, p_t, p_t_h) ||
        !ProofBoundsFromOriginalSpline(prepared, 2, t0.lower, t1.upper, p_tt,
                               p_tt_h) ||
        !ProofBoundsFromOriginalSpline(prepared, 3, t0.lower, t1.upper, p_ttt,
                               p_ttt_h)) return false;
    const ProofInterval speed_t = ProofNorm(p_t);
    const ProofInterval horizontal_speed_t = ProofNorm(p_t_h);
    const ProofInterval acceleration_t = ProofNorm(p_tt);
    const ProofInterval horizontal_acceleration_t = ProofNorm(p_tt_h);
    const ProofInterval jerk_t = ProofNorm(p_ttt);
    const ProofInterval speed = ProofMul(speed_t, dt_dw);
    const ProofInterval horizontal_speed = ProofMul(horizontal_speed_t, dt_dw);
    const ProofInterval acceleration = ProofMul(
        acceleration_t, ProofMul(dt_dw, dt_dw));
    const ProofInterval horizontal_acceleration = ProofMul(
        horizontal_acceleration_t, ProofMul(dt_dw, dt_dw));
    const ProofInterval jerk = ProofMul(jerk_t,
        ProofMul(ProofMul(dt_dw, dt_dw), dt_dw));
    if (!ProofFinite(speed) || !ProofFinite(horizontal_speed) ||
        !ProofFinite(acceleration) || !ProofFinite(horizontal_acceleration) ||
        !ProofFinite(jerk)) return false;
    const double anchor_w_value = w0 + 0.5 * (w1 - w0);
    const ProofInterval mathematical_anchor_w = ProofMul(
        PointProofInterval(0.5), ProofAdd(PointProofInterval(w0),
                                          PointProofInterval(w1)));
    const ProofInterval rounded_anchor_w = PointProofInterval(anchor_w_value);
    if (!ProofFinite(mathematical_anchor_w) ||
        !ProofFinite(rounded_anchor_w)) return false;
    const ProofInterval anchor_w = ProofIntervalFromBounds(
        std::min(mathematical_anchor_w.lower, rounded_anchor_w.lower),
        std::max(mathematical_anchor_w.upper, rounded_anchor_w.upper));
    const ProofInterval anchor_t = ProofAdd(PointProofInterval(spline_t_anchor),
        ProofMul(dt_dw, ProofSub(anchor_w, PointProofInterval(phase_w_anchor))));
    std::array<ProofInterval, 3U> anchor_position;
    std::array<ProofInterval, 3U> anchor_p_w;
    std::array<ProofInterval, 3U> anchor_p_ww;
    if (!ProofBoundsFromOriginalSpline(prepared, 0, anchor_t.lower, anchor_t.upper,
                               anchor_position, p_h_bounds) ||
        !ProofBoundsFromOriginalSpline(prepared, 1, anchor_t.lower, anchor_t.upper,
                               anchor_p_w, p_t_h) ||
        !ProofBoundsFromOriginalSpline(prepared, 2, anchor_t.lower, anchor_t.upper,
                               anchor_p_ww, p_tt_h)) return false;
    anchor_p_w = ProofVectorScale(anchor_p_w, dt_dw);
    anchor_p_ww = ProofVectorScale(anchor_p_ww,
        ProofMul(dt_dw, dt_dw));
    return MakeV2Certificate(w0, w1, anchor_w_value, anchor_position,
                             anchor_p_w, anchor_p_ww, speed, speed,
                             horizontal_speed, acceleration,
                             horizontal_acceleration, jerk, true, certificate);
}

bool invertArcLengthMap(const ArcLengthMap&, double, double&, double&, double&);
bool MakeMappedBsplineV2Certificate(
    const UniformBspline& p, const UniformBspline& dp_dt,
    const UniformBspline& d2p_dt2, const UniformBspline& d3p_dt3,
    const ArcLengthMap& map, const double spline_t_anchor,
    const double spline_t_end, const double phase_w_anchor,
    const double phase_w_end, const double arclength_per_phase,
    const double w0, const double w1,
    phase_offset_core::CertifiedPathCellV2& certificate)
{
    (void)dp_dt;
    (void)d2p_dt2;
    (void)d3p_dt3;
    certificate = phase_offset_core::CertifiedPathCellV2();
    if (!ProofFloatingPointEnvironmentSupported() ||
        !std::isfinite(w0) || !std::isfinite(w1) || !(w1 > w0) ||
        w0 < phase_w_anchor || w1 > phase_w_end ||
        !std::isfinite(arclength_per_phase) || arclength_per_phase <= 0.0 ||
        map.cells.empty() || map.table_s.size() != map.cells.size() + 1U ||
        map.table_t.size() != map.table_s.size()) return false;
    if (!SupportedSplineTimeDomain(p, spline_t_anchor, spline_t_end)) {
        return false;
    }
    for (std::size_t index = 0U; index < map.table_s.size(); ++index) {
        if (!std::isfinite(map.table_s[index]) ||
            !std::isfinite(map.table_t[index])) {
            return false;
        }
        if (index > 0U &&
            !(map.table_s[index] > map.table_s[index - 1U]) ) {
            return false;
        }
    }
    const ProofInterval scale = PointProofInterval(arclength_per_phase);
    const ProofInterval target0 = ProofMul(scale,
        ProofSub(PointProofInterval(w0), PointProofInterval(phase_w_anchor)));
    const ProofInterval target1 = ProofMul(scale,
        ProofSub(PointProofInterval(w1), PointProofInterval(phase_w_anchor)));
    if (!ProofFinite(target0) || !ProofFinite(target1)) return false;
    const double total_map_s = map.table_s.back();
    if (!std::isfinite(total_map_s) || total_map_s <= 0.0 ||
        target0.upper < 0.0 || target1.lower > total_map_s) {
        return false;
    }
    // The phase-domain inequalities are authoritative for the represented
    // map.  Intersect their outward arithmetic with the known map domain
    // before selecting cells; this prevents a one-ULP endpoint excursion from
    // becoming an inverse-coverage claim outside the table.
    const double target0_lower = std::max(0.0, target0.lower);
    const double target1_upper = std::min(total_map_s, target1.upper);
    const auto first_table = std::upper_bound(map.table_s.begin(),
                                               map.table_s.end(),
                                               target0_lower);
    const auto last_table = std::upper_bound(map.table_s.begin(),
                                              map.table_s.end(),
                                              target1_upper);
    std::size_t begin_index = first_table == map.table_s.begin() ? 0U :
        static_cast<std::size_t>(std::distance(map.table_s.begin(), first_table) - 1);
    std::size_t end_index = last_table == map.table_s.begin() ? 0U :
        static_cast<std::size_t>(std::distance(map.table_s.begin(), last_table) - 1);
    begin_index = std::min(begin_index, map.cells.size() - 1U);
    end_index = std::min(end_index, map.cells.size() - 1U);
    double enclosed_t0 = std::numeric_limits<double>::infinity();
    double enclosed_t1 = -std::numeric_limits<double>::infinity();
    ProofInterval q_min = PointProofInterval(std::numeric_limits<double>::infinity());
    ProofInterval q_max = PointProofInterval(0.0);
    ProofInterval q_prime_abs = PointProofInterval(0.0);
    ProofInterval q_second_abs = PointProofInterval(0.0);
    bool any = false;
    for (std::size_t index = begin_index; index <= end_index; ++index) {
        const ArcLengthCell& cell = map.cells[index];
        if (!std::isfinite(cell.t0) || !std::isfinite(cell.h) || cell.h <= 0.0) {
            return false;
        }
        std::vector<ProofInterval> q_coefficients;
        std::vector<ProofInterval> qp_coefficients;
        std::vector<ProofInterval> qpp_coefficients;
        for (int order = 1; order <= 5; ++order) {
            q_coefficients.push_back(ProofMul(
                PointProofInterval(static_cast<double>(order)),
                PointProofInterval(cell.a[static_cast<std::size_t>(order)])));
        }
        for (int order = 2; order <= 5; ++order) {
            qp_coefficients.push_back(ProofMul(
                PointProofInterval(static_cast<double>(order * (order - 1))),
                PointProofInterval(cell.a[static_cast<std::size_t>(order)])));
        }
        for (int order = 3; order <= 5; ++order) {
            qpp_coefficients.push_back(ProofMul(
                PointProofInterval(static_cast<double>(order * (order - 1) *
                    (order - 2))),
                PointProofInterval(cell.a[static_cast<std::size_t>(order)])));
        }
        const ProofInterval q = ProofDiv(
            ProofPowerToBernsteinRange(q_coefficients),
            PointProofInterval(cell.h));
        const ProofInterval qp = ProofDiv(
            ProofPowerToBernsteinRange(qp_coefficients),
            ProofMul(PointProofInterval(cell.h), PointProofInterval(cell.h)));
        const ProofInterval qpp = ProofDiv(
            ProofPowerToBernsteinRange(qpp_coefficients),
            ProofMul(ProofMul(PointProofInterval(cell.h),
                              PointProofInterval(cell.h)),
                     PointProofInterval(cell.h)));
        if (!ProofFinite(q) || !ProofFinite(qp) || !ProofFinite(qpp) ||
            q.lower <= 0.0) {
            return false;
        }
        q_min.lower = any ? std::min(q_min.lower, q.lower) : q.lower;
        q_min.upper = any ? std::max(q_min.upper, q.upper) : q.upper;
        q_max.upper = std::max(q_max.upper, q.upper);
        q_prime_abs.upper = std::max(q_prime_abs.upper,
            std::max(std::abs(qp.lower), std::abs(qp.upper)));
        q_second_abs.upper = std::max(q_second_abs.upper,
            std::max(std::abs(qpp.lower), std::abs(qpp.upper)));
        enclosed_t0 = std::min(enclosed_t0, map.table_t[index]);
        enclosed_t1 = std::max(enclosed_t1, map.table_t[index + 1U]);
        any = true;
    }
    if (!any || !std::isfinite(enclosed_t0) || !std::isfinite(enclosed_t1) ||
        enclosed_t1 <= enclosed_t0) {
        return false;
    }
    q_min.valid = true;
    // q_max is used as a positive denominator bound for dt/dw.  It is not
    // the interval [0, max]; retaining zero as its lower endpoint would make
    // the otherwise valid division straddle zero and reject every mapped
    // certificate.  The stored upper bound itself is the conservative scalar
    // denominator bound.
    if (!std::isfinite(q_max.upper) || q_max.upper <= 0.0 ||
        !std::isfinite(q_min.lower) || q_min.lower <= 0.0) {
        return false;
    }
    q_max.lower = q_max.upper;
    q_max.valid = true;
    q_prime_abs.valid = true;
    q_second_abs.valid = true;
    std::array<ProofInterval, 3U> p_bounds;
    std::array<ProofInterval, 3U> p_h_bounds;
    std::array<ProofInterval, 3U> pt_bounds;
    std::array<ProofInterval, 3U> pt_h_bounds;
    std::array<ProofInterval, 3U> ptt_bounds;
    std::array<ProofInterval, 3U> ptt_h_bounds;
    std::array<ProofInterval, 3U> pttt_bounds;
    std::array<ProofInterval, 3U> pttt_h_bounds;
    OriginalSplineProofControls prepared(p);
    const bool bounds0 = ProofBoundsFromOriginalSpline(
        prepared, 0, enclosed_t0, enclosed_t1, p_bounds, p_h_bounds);
    const bool bounds1 = ProofBoundsFromOriginalSpline(
        prepared, 1, enclosed_t0, enclosed_t1, pt_bounds, pt_h_bounds);
    const bool bounds2 = ProofBoundsFromOriginalSpline(
        prepared, 2, enclosed_t0, enclosed_t1, ptt_bounds, ptt_h_bounds);
    const bool bounds3 = ProofBoundsFromOriginalSpline(
        prepared, 3, enclosed_t0, enclosed_t1, pttt_bounds, pttt_h_bounds);
    if (!(bounds0 && bounds1 && bounds2 && bounds3)) return false;
    const ProofInterval dt_dw_min = ProofDiv(scale, q_max);
    const ProofInterval dt_dw_max = ProofDiv(scale, q_min);
    const ProofInterval d2t_dw2 = ProofMul(
        ProofMul(ProofMul(scale, scale), q_prime_abs),
        ProofDiv(PointProofInterval(1.0),
                 ProofMul(ProofMul(q_min, q_min), q_min)));
    // d2t/dw2 = -scale^2*q'/q^3 has unknown sign because q' may be
    // positive or negative.  Keep the nonnegative magnitude bound for scalar
    // acceleration/jerk terms, but use a symmetric signed interval whenever
    // forming the anchor vector enclosure.
    if (!ProofFinite(d2t_dw2)) return false;
    const ProofInterval d2t_dw2_signed = ProofIntervalFromBounds(
        ProofLower(-d2t_dw2.upper), ProofUpper(d2t_dw2.upper));
    if (!ProofFinite(d2t_dw2_signed)) return false;
    // t' = S'/q, t'' = -S'^2 q'/q^3 and
    // |t'''| <= S'^3 (3|q'|^2/q_min^5 + |q''|/q_min^4).
    const ProofInterval q_min_squared = ProofMul(q_min, q_min);
    const ProofInterval q_min_cubed = ProofMul(q_min_squared, q_min);
    const ProofInterval q_min_fourth = ProofMul(q_min_cubed, q_min);
    const ProofInterval q_min_fifth = ProofMul(q_min_fourth, q_min);
    const ProofInterval d3t_dw3 = ProofMul(
        ProofMul(ProofMul(scale, scale), scale),
        ProofAdd(ProofDiv(ProofMul(
                                   ProofIntervalFromBounds(3.0, 3.0),
                                   ProofMul(q_prime_abs, q_prime_abs)),
                          q_min_fifth),
                 ProofDiv(q_second_abs, q_min_fourth)));
    if (!ProofFinite(dt_dw_min) || !ProofFinite(dt_dw_max) ||
        !ProofFinite(d2t_dw2) || !ProofFinite(d3t_dw3)) {
        return false;
    }
    const ProofInterval speed = ProofMul(ProofNorm(pt_bounds), dt_dw_min);
    const ProofInterval horizontal_speed = ProofMul(
        ProofNorm(pt_h_bounds), dt_dw_min);
    const ProofInterval acceleration = ProofAdd(
        ProofMul(ProofNorm(ptt_bounds), ProofMul(dt_dw_max, dt_dw_max)),
        ProofMul(ProofNorm(pt_bounds), d2t_dw2));
    const ProofInterval horizontal_acceleration = ProofAdd(
        ProofMul(ProofNorm(ptt_h_bounds), ProofMul(dt_dw_max, dt_dw_max)),
        ProofMul(ProofNorm(pt_h_bounds), d2t_dw2));
    const ProofInterval jerk = ProofAdd(
        ProofAdd(ProofMul(ProofNorm(pttt_bounds),
                          ProofMul(ProofMul(dt_dw_max, dt_dw_max), dt_dw_max)),
                 ProofMul(ProofMul(PointProofInterval(3.0), ProofNorm(ptt_bounds)),
                          ProofMul(dt_dw_max, d2t_dw2))),
        ProofMul(ProofNorm(pt_bounds), d3t_dw3));
    if (!ProofFinite(speed) || !ProofFinite(horizontal_speed) ||
        !ProofFinite(acceleration) || !ProofFinite(horizontal_acceleration) ||
        !ProofFinite(jerk)) {
        return false;
    }
    const double anchor_w_value = w0 + 0.5 * (w1 - w0);
    const ProofInterval mathematical_anchor_w = ProofMul(
        PointProofInterval(0.5), ProofAdd(PointProofInterval(w0),
                                          PointProofInterval(w1)));
    const ProofInterval rounded_anchor_w = PointProofInterval(anchor_w_value);
    if (!ProofFinite(mathematical_anchor_w) ||
        !ProofFinite(rounded_anchor_w)) return false;
    const ProofInterval anchor_w = ProofIntervalFromBounds(
        std::min(mathematical_anchor_w.lower, rounded_anchor_w.lower),
        std::max(mathematical_anchor_w.upper, rounded_anchor_w.upper));
    const ProofInterval anchor_target_s = ProofMul(
        scale, ProofSub(anchor_w, PointProofInterval(phase_w_anchor)));
    if (!ProofFinite(anchor_target_s) ||
        anchor_target_s.upper < target0.lower ||
        anchor_target_s.lower > target1.upper) {
        return false;
    }
    // Invert the STORED monotone quintic, not the nominal Newton estimate.
    // Interval Newton uses a positive derivative enclosure over [0,1].
    // At ambiguous table/polynomial endpoints retain the original cell hull;
    // do not assert that a rounded endpoint or nominal iterate is an inverse.
    // This is anchor-only work: all complete-cell bounds above are unchanged.
    const double anchor_s0 = std::max(target0_lower,
        std::min(target1_upper, anchor_target_s.lower));
    const double anchor_s1 = std::max(target0_lower,
        std::min(target1_upper, anchor_target_s.upper));
    ProofInterval anchor_t = InvalidProofInterval();
    ProofInterval anchor_q = InvalidProofInterval();
    ProofInterval anchor_qp = InvalidProofInterval();
    // Keep the rigorous inverse enclosure and also cover the executable
    // inverse's binary64 result; its residual stopping rule is unchanged.
    double nominal_t, nominal_q, nominal_qp;
    const double nominal_s = std::max(0.0, std::min(map.table_s.back(),
        arclength_per_phase * (anchor_w_value - phase_w_anchor)));
    if (!invertArcLengthMap(map, nominal_s, nominal_t, nominal_q, nominal_qp)) return false;
    const auto hull = [](ProofInterval& out, const ProofInterval& value) {
        if (!out.valid) out = value;
        else {
            out.lower = std::min(out.lower, value.lower);
            out.upper = std::max(out.upper, value.upper);
        }
    };
    for (std::size_t index = begin_index; index <= end_index; ++index) {
        const double s0 = std::max(anchor_s0, map.table_s[index]);
        const double s1 = std::min(anchor_s1, map.table_s[index + 1U]);
        if (s0 > s1) continue;
        const ArcLengthCell& local = map.cells[index];
        std::vector<ProofInterval> coefficients, first, second;
        for (std::size_t order = 0U; order < local.a.size(); ++order) {
            coefficients.push_back(PointProofInterval(local.a[order]));
            if (order > 0U) first.push_back(ProofMul(
                PointProofInterval(static_cast<double>(order)), coefficients.back()));
            if (order > 1U) second.push_back(ProofMul(
                PointProofInterval(static_cast<double>(order * (order - 1U))), coefficients.back()));
        }
        const ProofInterval derivative = ProofPowerToBernsteinRange(first);
        if (!ProofFinite(derivative) || derivative.lower <= 0.0) return false;
        const ProofInterval target = ProofIntervalFromBounds(s0, s1);
        ProofInterval x = ProofIntervalFromBounds(0.0, 1.0);
        const ProofInterval at_zero = ProofHorner(coefficients, PointProofInterval(0.0));
        const ProofInterval at_one = ProofHorner(coefficients, PointProofInterval(1.0));
        if (!ProofFinite(at_zero) || !ProofFinite(at_one)) return false;
        const bool inverse_bracketed = at_zero.upper <= s0 && s1 <= at_one.lower;
        // This guess only selects the evaluation point. Every contraction
        // below is justified by outward arithmetic and monotonicity.
        double center = std::max(0.0, std::min(1.0,
            ((s0 + 0.5 * (s1 - s0)) - map.table_s[index]) /
            (map.table_s[index + 1U] - map.table_s[index])));
        for (int iteration = 0; inverse_bracketed &&
             iteration < std::numeric_limits<double>::digits; ++iteration) {
            if (!std::isfinite(center)) return false;
            const ProofInterval value = ProofHorner(coefficients, PointProofInterval(center));
            const ProofInterval inverse = ProofSub(PointProofInterval(center),
                ProofDiv(ProofSub(value, target), derivative));
            if (!ProofFinite(inverse)) return false;
            const double lower = std::max(x.lower, inverse.lower);
            const double upper = std::min(x.upper, inverse.upper);
            if (lower > upper) return false;
            if (lower == x.lower && upper == x.upper) break;
            x = ProofIntervalFromBounds(lower, upper);
            center = lower + 0.5 * (upper - lower);
            if (!(center > lower && center < upper)) break;
        }
        if (nominal_s >= map.table_s[index] && nominal_s <= map.table_s[index + 1U]) {
            const ProofInterval nominal_x = ProofDiv(
                ProofSub(PointProofInterval(nominal_t), PointProofInterval(local.t0)),
                PointProofInterval(local.h));
            if (!ProofFinite(nominal_x)) return false;
            x.lower = std::min(x.lower, std::max(0.0, nominal_x.lower));
            x.upper = std::max(x.upper, std::min(1.0, nominal_x.upper));
        }
        const ProofInterval time = ProofAdd(PointProofInterval(local.t0),
            ProofMul(PointProofInterval(local.h), x));
        ProofInterval q = ProofDiv(ProofHorner(first, x), PointProofInterval(local.h));
        ProofInterval qp = ProofDiv(ProofHorner(second, x),
            ProofMul(PointProofInterval(local.h), PointProofInterval(local.h)));
        if (!ProofFinite(time) || !ProofFinite(q) ||
            !ProofFinite(qp)) return false;
        // Intersect with the already-proved full-cell bounds; these remain
        // untouched and keep dependency widening from destroying positivity.
        q = ProofIntervalFromBounds(std::max(q.lower, q_min.lower),
                                     std::min(q.upper, q_max.upper));
        qp = ProofIntervalFromBounds(std::max(qp.lower, -q_prime_abs.upper),
                                      std::min(qp.upper, q_prime_abs.upper));
        if (!ProofFinite(q) || q.lower <= 0.0 || !ProofFinite(qp)) return false;
        hull(anchor_t, time);
        hull(anchor_q, q);
        hull(anchor_qp, qp);
    }
    hull(anchor_t, PointProofInterval(nominal_t));
    hull(anchor_q, PointProofInterval(nominal_q));
    hull(anchor_qp, PointProofInterval(nominal_qp));
    if (!ProofFinite(anchor_t) || !ProofFinite(anchor_q) || !ProofFinite(anchor_qp)) return false;
    // Preserve the authoritative native time domain, including its endpoints.
    // One outward ULP also lets a singleton inverse use the interval evaluator.
    anchor_t.lower = std::max(spline_t_anchor,
        std::nextafter(anchor_t.lower, -std::numeric_limits<double>::infinity()));
    anchor_t.upper = std::min(spline_t_end,
        std::nextafter(anchor_t.upper, std::numeric_limits<double>::infinity()));
    std::array<ProofInterval, 3U> anchor_position, anchor_pt, anchor_ptt, unused_horizontal;
    if (!ProofBoundsFromOriginalSpline(prepared, 0, anchor_t.lower, anchor_t.upper,
                                      anchor_position, unused_horizontal) ||
        !ProofBoundsFromOriginalSpline(prepared, 1, anchor_t.lower, anchor_t.upper,
                                      anchor_pt, unused_horizontal) ||
        !ProofBoundsFromOriginalSpline(prepared, 2, anchor_t.lower, anchor_t.upper,
                                      anchor_ptt, unused_horizontal)) return false;
    const ProofInterval anchor_dt = ProofDiv(scale, anchor_q);
    const ProofInterval anchor_d2t = ProofDiv(
        ProofMul(ProofMul(PointProofInterval(-1.0), ProofMul(scale, scale)), anchor_qp),
        ProofMul(ProofMul(anchor_q, anchor_q), anchor_q));
    if (!ProofFinite(anchor_dt) || !ProofFinite(anchor_d2t)) return false;
    std::array<ProofInterval, 3U> anchor_p_w = ProofVectorScale(anchor_pt, anchor_dt);
    std::array<ProofInterval, 3U> anchor_p_ww = ProofVectorAdd(
        ProofVectorScale(anchor_ptt, ProofMul(anchor_dt, anchor_dt)),
        ProofVectorScale(anchor_pt, anchor_d2t));
    return MakeV2Certificate(w0, w1, anchor_w_value, anchor_position, anchor_p_w,
                             anchor_p_ww, speed, ProofMul(ProofNorm(pt_bounds),
                             dt_dw_max), horizontal_speed, acceleration,
                             horizontal_acceleration, jerk, true, certificate);
}

bool finiteState(const ContinuousPhasePathState& state)
{
    return state.p.allFinite() && state.dp_dw.allFinite() &&
           state.d2p_dw2.allFinite() && state.vel.allFinite();
}

Eigen::Vector3d evaluateSplineAtTime(const UniformBspline& spline, double t)
{
    const double u = t * spline.beta_ + spline.u_(spline.p_);
    return spline.singleDeboor(u);
}

bool buildArcLengthTable(const UniformBspline& position,
                         double t_start,
                         double t_end,
                         bool require_positive_speed,
                         std::vector<double>& table_t,
                         std::vector<double>& table_s,
                         std::vector<double>* table_speed)
{
    table_t.assign(kArcLengthTableIntervals + 1, 0.0);
    table_s.assign(kArcLengthTableIntervals + 1, 0.0);
    if (table_speed) table_speed->assign(kArcLengthTableIntervals + 1, 0.0);
    if (!std::isfinite(t_start) || !std::isfinite(t_end) ||
        t_end <= t_start + kDomainEps) {
        return false;
    }
    const UniformBspline velocity_spline = position.getDerivative();
    const double dt = (t_end - t_start) / kArcLengthTableIntervals;
    for (int i = 0; i <= kArcLengthTableIntervals; ++i) {
        const double t = (i == kArcLengthTableIntervals)
            ? t_end
            : t_start + dt * static_cast<double>(i);
        const Eigen::Vector3d velocity = evaluateSplineAtTime(velocity_spline, t);
        const double speed = velocity.norm();
        if (!velocity.allFinite() || !std::isfinite(speed) ||
            (require_positive_speed && speed <= kArcLengthSpeedEps)) {
            return false;
        }
        table_t[i] = t;
        if (table_speed) (*table_speed)[i] = speed;
    }
    for (int i = 1; i <= kArcLengthTableIntervals; ++i) {
        const double midpoint_t = 0.5 * (table_t[i - 1] + table_t[i]);
        const double midpoint_speed =
            evaluateSplineAtTime(velocity_spline, midpoint_t).norm();
        if (!std::isfinite(midpoint_speed)) return false;
        table_s[i] = table_s[i - 1] +
            (dt / 6.0) * ((table_speed ? (*table_speed)[i - 1]
                                       : evaluateSplineAtTime(
                                             velocity_spline, table_t[i - 1]).norm()) +
                          4.0 * midpoint_speed +
                          (table_speed ? (*table_speed)[i]
                                       : evaluateSplineAtTime(
                                             velocity_spline, table_t[i]).norm()));
    }
    return std::isfinite(table_s.back()) && table_s.back() > kDomainEps;
}

bool buildExecutableArcLengthMap(const UniformBspline& position,
                                 double t_start,
                                 double t_end,
                                 ArcLengthMap& map)
{
    map = ArcLengthMap();
    if (!buildArcLengthTable(position, t_start, t_end, true,
                             map.table_t, map.table_s, &map.table_speed)) {
        return false;
    }

    const UniformBspline velocity_spline = position.getDerivative();
    const UniformBspline acceleration_spline = velocity_spline.getDerivative();
    map.table_speed_derivative.assign(kArcLengthTableIntervals + 1, 0.0);
    for (int i = 0; i <= kArcLengthTableIntervals; ++i) {
        const Eigen::Vector3d velocity =
            evaluateSplineAtTime(velocity_spline, map.table_t[i]);
        const Eigen::Vector3d acceleration =
            evaluateSplineAtTime(acceleration_spline, map.table_t[i]);
        const double speed = map.table_speed[i];
        if (!velocity.allFinite() || !acceleration.allFinite() ||
            !std::isfinite(speed) || speed <= kArcLengthSpeedEps) {
            return false;
        }
        map.table_speed_derivative[i] = velocity.dot(acceleration) / speed;
        if (!std::isfinite(map.table_speed_derivative[i])) return false;
    }

    map.cells.resize(kArcLengthTableIntervals);
    for (int i = 0; i < kArcLengthTableIntervals; ++i) {
        ArcLengthCell& cell = map.cells[i];
        cell.t0 = map.table_t[i];
        cell.h = map.table_t[i + 1] - map.table_t[i];
        const double h = cell.h;
        if (!std::isfinite(h) || h <= 0.0) return false;
        const double y0 = map.table_s[i];
        const double y1 = map.table_s[i + 1];
        const double d0 = map.table_speed[i];
        const double d1 = map.table_speed[i + 1];
        const double dd0 = map.table_speed_derivative[i];
        const double dd1 = map.table_speed_derivative[i + 1];
        cell.a[0] = y0;
        cell.a[1] = h * d0;
        cell.a[2] = 0.5 * h * h * dd0;
        const double r0 = y1 - cell.a[0] - cell.a[1] - cell.a[2];
        const double r1 = h * d1 - cell.a[1] - 2.0 * cell.a[2];
        const double r2 = h * h * dd1 - 2.0 * cell.a[2];
        cell.a[3] = 10.0 * r0 - 4.0 * r1 + 0.5 * r2;
        cell.a[4] = -15.0 * r0 + 7.0 * r1 - r2;
        cell.a[5] = 6.0 * r0 - 3.0 * r1 + 0.5 * r2;

        // A positive Bernstein control polygon is a sufficient monotonicity
        // proof for ds/dx across the complete cell.
        const double q0 = cell.a[1];
        const double q1 = 2.0 * cell.a[2];
        const double q2 = 3.0 * cell.a[3];
        const double q3 = 4.0 * cell.a[4];
        const double q4 = 5.0 * cell.a[5];
        const std::array<double, 5> bernstein = {{
            q0,
            q0 + 0.25 * q1,
            q0 + 0.5 * q1 + q2 / 6.0,
            q0 + 0.75 * q1 + 0.5 * q2 + 0.25 * q3,
            q0 + q1 + q2 + q3 + q4}};
        for (const double coefficient : bernstein) {
            if (!std::isfinite(coefficient) ||
                coefficient <= cell.h * kArcLengthSpeedEps) {
                return false;
            }
        }
    }
    return std::isfinite(map.table_s.back()) &&
           map.table_s.back() > kDomainEps;
}

double refinedArcLengthOnInterval(const UniformBspline& velocity_spline,
                                  const double t0,
                                  const double t1)
{
    if (!std::isfinite(t0) || !std::isfinite(t1) || t1 <= t0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double midpoint = 0.5 * (t0 + t1);
    const Eigen::Vector3d v0 = evaluateSplineAtTime(velocity_spline, t0);
    const Eigen::Vector3d vm = evaluateSplineAtTime(velocity_spline, midpoint);
    const Eigen::Vector3d v1 = evaluateSplineAtTime(velocity_spline, t1);
    if (!v0.allFinite() || !vm.allFinite() || !v1.allFinite()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return (t1 - t0) * (v0.norm() + 4.0 * vm.norm() + v1.norm()) / 6.0;
}

bool invertRawArcLengthTable(const UniformBspline& position,
                             const std::vector<double>& table_t,
                             const std::vector<double>& table_s,
                             const double target_s,
                             double& t)
{
    t = 0.0;
    if (table_t.size() != kArcLengthTableIntervals + 1U ||
        table_s.size() != table_t.size() || !std::isfinite(target_s)) {
        return false;
    }
    const double total_length = table_s.back();
    const double bounded_s = std::max(0.0, std::min(target_s, total_length));
    const auto upper = std::lower_bound(
        table_s.begin(), table_s.end(), bounded_s);
    if (upper == table_s.begin()) {
        t = table_t.front();
        return true;
    }
    if (upper == table_s.end()) {
        t = table_t.back();
        return true;
    }
    const size_t hi = static_cast<size_t>(std::distance(table_s.begin(), upper));
    const size_t lo = hi - 1U;
    const double s0 = table_s[lo];
    const double s1 = table_s[hi];
    if (!std::isfinite(s0) || !std::isfinite(s1) || s1 <= s0) return false;

    const UniformBspline velocity_spline = position.getDerivative();
    double left_t = table_t[lo];
    double right_t = table_t[hi];
    double left_s = s0;
    for (int iteration = 0; iteration < 48; ++iteration) {
        const double middle_t = 0.5 * (left_t + right_t);
        const double partial = refinedArcLengthOnInterval(
            velocity_spline, table_t[lo], middle_t);
        if (!std::isfinite(partial)) return false;
        const double middle_s = s0 + partial;
        if (middle_s < bounded_s) {
            left_t = middle_t;
            left_s = middle_s;
        } else {
            right_t = middle_t;
        }
        if (std::abs(middle_s - bounded_s) <=
            1e-13 * std::max(1.0, total_length)) {
            t = middle_t;
            return true;
        }
    }
    (void)left_s;
    t = 0.5 * (left_t + right_t);
    return std::isfinite(t);
}

void evaluateArcLengthCell(const ArcLengthCell& cell,
                           const double x,
                           double& value,
                           double& first,
                           double& second)
{
    const double x2 = x * x;
    const double x3 = x2 * x;
    const double x4 = x3 * x;
    const double x5 = x4 * x;
    value = cell.a[0] + cell.a[1] * x + cell.a[2] * x2 +
            cell.a[3] * x3 + cell.a[4] * x4 + cell.a[5] * x5;
    first = (cell.a[1] + 2.0 * cell.a[2] * x +
             3.0 * cell.a[3] * x2 + 4.0 * cell.a[4] * x3 +
             5.0 * cell.a[5] * x4) / cell.h;
    second = (2.0 * cell.a[2] + 6.0 * cell.a[3] * x +
              12.0 * cell.a[4] * x2 + 20.0 * cell.a[5] * x3) /
             (cell.h * cell.h);
}

// Enclose the represented quintic S(t) stored in an ArcLengthMap.  The map's
// cells are the executable representation (the Simpson samples only define
// their coefficients), so structural knot mapping must evaluate these power
// polynomials rather than linearly interpolating table_s.  A time interval is
// accepted because spline-knot arithmetic itself is rounded; if it straddles
// a map-cell boundary, both represented cells contribute to the hull.
bool ProofArcLengthValueAtTime(const ArcLengthMap& map,
                               const ProofInterval& time,
                               ProofInterval& value)
{
    value = InvalidProofInterval();
    if (!ProofFloatingPointEnvironmentSupported() || !ProofFinite(time) ||
        map.cells.empty() || map.table_t.size() != map.cells.size() + 1U ||
        map.table_s.size() != map.table_t.size()) {
        return false;
    }
    bool have_cell = false;
    for (std::size_t index = 0U; index < map.cells.size(); ++index) {
        const double table_t0 = map.table_t[index];
        const double table_t1 = map.table_t[index + 1U];
        const ArcLengthCell& cell = map.cells[index];
        if (!std::isfinite(table_t0) || !std::isfinite(table_t1) ||
            !(table_t1 > table_t0) || !std::isfinite(cell.t0) ||
            !std::isfinite(cell.h) || cell.h <= 0.0 ||
            cell.t0 != table_t0 || cell.h != table_t1 - table_t0) {
            return false;
        }
        if (table_t1 < time.lower || table_t0 > time.upper) continue;
        const double clipped_lower = std::max(table_t0, time.lower);
        const double clipped_upper = std::min(table_t1, time.upper);
        if (!std::isfinite(clipped_lower) || !std::isfinite(clipped_upper) ||
            clipped_lower > clipped_upper) {
            continue;
        }
        const ProofInterval query = ProofIntervalFromBounds(
            clipped_lower, clipped_upper);
        const ProofInterval x = ProofDiv(
            ProofSub(query, PointProofInterval(cell.t0)),
            PointProofInterval(cell.h));
        if (!ProofFinite(x)) return false;
        std::vector<ProofInterval> coefficients;
        coefficients.reserve(cell.a.size());
        for (const double coefficient : cell.a) {
            coefficients.push_back(PointProofInterval(coefficient));
        }
        const ProofInterval local = ProofHorner(coefficients, x);
        if (!ProofFinite(local)) return false;
        if (!have_cell) {
            value = local;
            have_cell = true;
        } else {
            value.lower = std::min(value.lower, local.lower);
            value.upper = std::max(value.upper, local.upper);
        }
    }
    return have_cell && ProofFinite(value);
}

bool invertArcLengthMap(const ArcLengthMap& map,
                        double target_s,
                        double& t,
                        double& ds_dt,
                        double& d2s_dt2)
{
    if (map.table_t.size() != kArcLengthTableIntervals + 1U ||
        map.table_s.size() != map.table_t.size() ||
        map.table_speed.size() != map.table_t.size() ||
        map.table_speed_derivative.size() != map.table_t.size() ||
        map.cells.size() != kArcLengthTableIntervals ||
        !std::isfinite(target_s)) {
        return false;
    }
    const double total_length = map.table_s.back();
    const double bounded_s = std::max(0.0, std::min(target_s, total_length));
    const auto upper = std::lower_bound(
        map.table_s.begin(), map.table_s.end(), bounded_s);
    if (upper == map.table_s.begin()) {
        t = map.table_t.front();
        ds_dt = map.table_speed.front();
        d2s_dt2 = map.table_speed_derivative.front();
        return true;
    }
    if (upper == map.table_s.end()) {
        t = map.table_t.back();
        ds_dt = map.table_speed.back();
        d2s_dt2 = map.table_speed_derivative.back();
        return true;
    }
    const size_t hi = static_cast<size_t>(
        std::distance(map.table_s.begin(), upper));
    const size_t lo = hi - 1U;
    const double s0 = map.table_s[lo];
    const double s1 = map.table_s[hi];
    const double ds = s1 - s0;
    const ArcLengthCell& cell = map.cells[lo];
    if (!std::isfinite(ds) || ds <= 0.0 || !std::isfinite(cell.h) ||
        cell.h <= 0.0) {
        return false;
    }
    double lower_x = 0.0;
    double upper_x = 1.0;
    double x = std::max(0.0, std::min(1.0, (bounded_s - s0) / ds));
    for (int iteration = 0; iteration < 24; ++iteration) {
        double value = 0.0;
        evaluateArcLengthCell(cell, x, value, ds_dt, d2s_dt2);
        const double residual = value - bounded_s;
        if (std::abs(residual) <= 1e-13 * std::max(1.0, total_length)) {
            break;
        }
        if (residual < 0.0) lower_x = x;
        else upper_x = x;
        double next_x = 0.5 * (lower_x + upper_x);
        if (std::isfinite(ds_dt) && ds_dt > kArcLengthSpeedEps) {
            const double newton = x - residual / (cell.h * ds_dt);
            if (newton > lower_x && newton < upper_x) next_x = newton;
        }
        x = next_x;
    }
    double value = 0.0;
    evaluateArcLengthCell(cell, x, value, ds_dt, d2s_dt2);
    t = cell.t0 + x * cell.h;
    return std::isfinite(t) && std::isfinite(ds_dt) &&
           ds_dt > kArcLengthSpeedEps && std::isfinite(d2s_dt2);
}
}  // namespace

bool ContinuousPhasePath::appendSegment(
    double w0,
    double w1,
    const std::string& label,
    const Evaluator& evaluator)
{
    if (!std::isfinite(w0) || !std::isfinite(w1) ||
        w1 <= w0 + kDomainEps || !evaluator) {
        return false;
    }
    if (!segments_.empty() && w0 < segments_.back().w1 - kDomainEps) {
        return false;
    }

    Segment segment;
    segment.w0 = w0;
    segment.w1 = w1;
    segment.identity = next_segment_identity_++;
    if (segment.identity == 0U) return false;
    segment.label = label;
    segment.evaluate = evaluator;
    segments_.push_back(segment);
    return true;
}

bool ContinuousPhasePath::appendSlice(
    const ContinuousPhasePath& source,
    double slice_w0,
    double slice_w1)
{
    if (!std::isfinite(slice_w0) || !std::isfinite(slice_w1) ||
        slice_w1 <= slice_w0 + kDomainEps) {
        return false;
    }

    bool appended = false;
    for (const auto& segment : source.segments_) {
        const double w0 = std::max(slice_w0, segment.w0);
        const double w1 = std::min(slice_w1, segment.w1);
        if (w1 <= w0 + kDomainEps) continue;
        if (!appendSegment(w0, w1, segment.label, segment.evaluate)) {
            return false;
        }
        appended = true;
    }
    return appended;
}

bool ContinuousPhasePath::cellBounds(
    const double w0, const double w1,
    phase_offset_core::PathCellGeometryCertificate& certificate) const
{
    certificate = phase_offset_core::PathCellGeometryCertificate();
    if (segments_.empty() || !std::isfinite(w0) || !std::isfinite(w1) ||
        w1 <= w0 + kDomainEps) {
        return false;
    }
    const Segment* selected = nullptr;
    for (const Segment& segment : segments_) {
        if (w0 >= segment.w0 - kDomainEps && w1 <= segment.w1 + kDomainEps) {
            selected = &segment;
            break;
        }
    }
    if (selected == nullptr || !selected->evaluate.hasCellBounds() ||
        !selected->evaluate.cellBounds(w0, w1, certificate)) {
        certificate = phase_offset_core::PathCellGeometryCertificate();
        return false;
    }
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.path_revision = path_revision_;
    certificate.segment_identity = selected->identity;
    certificate.segment_w0 = selected->w0;
    certificate.segment_w1 = selected->w1;
    if (certificate.path_revision == 0U) {
        certificate.path_revision = path_revision_;
    }
    if (!phase_offset_core::pathCellGeometryCertificateIsComplete(certificate)) {
        certificate = phase_offset_core::PathCellGeometryCertificate();
        return false;
    }
    return true;
}

bool ContinuousPhasePath::tubeCellBoundsV2(
    const double w0, const double w1,
    phase_offset_core::CertifiedPathCellV2& certificate) const
{
    certificate = phase_offset_core::CertifiedPathCellV2();
    if (segments_.empty() || !std::isfinite(w0) || !std::isfinite(w1) ||
        !(w1 > w0)) {
        return false;
    }
    const Segment* selected = nullptr;
    for (const Segment& segment : segments_) {
        if (w0 >= segment.w0 && w1 <= segment.w1) {
            selected = &segment;
            break;
        }
    }
    if (selected == nullptr || !selected->evaluate.hasTubeCellBoundsV2() ||
        !selected->evaluate.tubeCellBoundsV2(w0, w1, certificate)) {
        certificate = phase_offset_core::CertifiedPathCellV2();
        return false;
    }
    // A callback is not allowed to return evidence for a different phase
    // cell and rely on the owner stamp below to relabel it.  Keep the
    // requested binary64 endpoints as an exact identity check; numerical
    // slack here would make a structural seam ambiguous.
    if (certificate.w0 != w0 || certificate.w1 != w1) {
        certificate = phase_offset_core::CertifiedPathCellV2();
        return false;
    }
    certificate.w0 = w0;
    certificate.w1 = w1;
    certificate.path_revision = path_revision_;
    certificate.frame_revision = path_revision_;
    certificate.segment_identity = selected->identity;
    certificate.proof_identity = selected->identity == 0U
        ? 1U : selected->identity;
    if (!phase_offset_core::certifiedPathCellV2IsComplete(certificate)) {
        certificate = phase_offset_core::CertifiedPathCellV2();
        return false;
    }
    return true;
}

bool ContinuousPhasePath::certificateBreakpointsV2(
    std::vector<double>& breakpoints) const
{
    breakpoints.clear();
    if (segments_.empty()) return false;
    for (const Segment& segment : segments_) {
        if (!std::isfinite(segment.w0) || !std::isfinite(segment.w1) ||
            !(segment.w1 > segment.w0)) {
            breakpoints.clear();
            return false;
        }
        if (!segment.evaluate.hasTubeCellBoundsV2() &&
            !segment.evaluate.hasCertificateBreakpointsV2()) {
            // A point-only/legacy evaluator has no certified structural
            // capability.  Do not present its owner endpoints as a V2 proof
            // partition merely because the segment metadata is finite.
            breakpoints.clear();
            return false;
        }
        breakpoints.push_back(segment.w0);
        breakpoints.push_back(segment.w1);
        if (segment.evaluate.hasCertificateBreakpointsV2()) {
            std::vector<double> internal;
            if (!segment.evaluate.certificateBreakpointsV2(internal)) {
                breakpoints.clear();
                return false;
            }
            for (const double value : internal) {
                if (!std::isfinite(value)) {
                    breakpoints.clear();
                    return false;
                }
                // appendSlice deliberately reuses the immutable evaluator
                // from its source segment.  Its callback may therefore list
                // structural points owned by the unsliced prefix/suffix;
                // retain only points that lie in this copied segment while
                // still rejecting non-finite evidence.
                if (value < segment.w0 || value > segment.w1) continue;
                breakpoints.push_back(value);
            }
        }
    }
    std::sort(breakpoints.begin(), breakpoints.end());
    breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()),
                      breakpoints.end());
    return !breakpoints.empty();
}

std::vector<double> ContinuousPhasePath::certificateBreakpointsV2() const
{
    std::vector<double> breakpoints;
    if (!certificateBreakpointsV2(breakpoints)) breakpoints.clear();
    return breakpoints;
}

bool ContinuousPhasePath::evaluate(
    double w,
    ContinuousPhasePathState& state,
    bool clamp_to_domain) const
{
    state = ContinuousPhasePathState();
    if (segments_.empty() || !std::isfinite(w)) return false;

    double query_w = w;
    if (query_w < startW()) {
        if (!clamp_to_domain) return false;
        query_w = startW();
    } else if (query_w > endW()) {
        if (!clamp_to_domain) return false;
        query_w = endW();
    }

    const Segment* selected = nullptr;
    for (const auto& segment : segments_) {
        if (query_w >= segment.w0 - kDomainEps &&
            query_w <= segment.w1 + kDomainEps) {
            selected = &segment;
            break;
        }
    }
    if (!selected) return false;

    const double bounded_w = std::max(selected->w0, std::min(query_w, selected->w1));
    if (!selected->evaluate(bounded_w, state)) return false;
    state.valid = finiteState(state);
    state.path_revision = path_revision_;
    return state.valid;
}

bool ContinuousPhasePath::sample(
    double step_w,
    std::vector<double>& w,
    std::vector<ContinuousPhasePathState>& states) const
{
    w.clear();
    states.clear();
    if (empty() || !std::isfinite(step_w) || step_w <= 0.0) return false;

    const double span = endW() - startW();
    const int count = std::max(2, static_cast<int>(std::ceil(span / step_w)) + 1);
    w.reserve(count);
    states.reserve(count);
    for (int i = 0; i < count; ++i) {
        const double ratio = static_cast<double>(i) /
            static_cast<double>(count - 1);
        const double query_w = startW() + ratio * span;
        ContinuousPhasePathState state;
        if (!evaluate(query_w, state, false)) {
            w.clear();
            states.clear();
            return false;
        }
        w.push_back(query_w);
        states.push_back(state);
    }
    return true;
}

double ContinuousPhasePath::startW() const
{
    return segments_.empty() ? 0.0 : segments_.front().w0;
}

double ContinuousPhasePath::endW() const
{
    return segments_.empty() ? 0.0 : segments_.back().w1;
}

bool ContinuousPhasePath::measureBsplineArcLength(
    const UniformBspline& position,
    double spline_t_start,
    double spline_t_end,
    double& total_length)
{
    total_length = 0.0;
    std::vector<double> table_t;
    std::vector<double> table_s;
    if (!buildArcLengthTable(position, spline_t_start, spline_t_end, false,
                             table_t, table_s, nullptr)) {
        return false;
    }
    total_length = table_s.back();
    return std::isfinite(total_length) && total_length > kDomainEps;
}

ContinuousPhasePath::Evaluator ContinuousPhasePath::makeQuinticHermite(
    double w0,
    double w1,
    const ContinuousPhasePathState& start,
    const ContinuousPhasePathState& end)
{
    if (!std::isfinite(w0) || !std::isfinite(w1) ||
        w1 <= w0 + kDomainEps || !finiteState(start) || !finiteState(end)) {
        return Evaluator();
    }

    const double h = w1 - w0;
    const Eigen::Vector3d a0 = start.p;
    const Eigen::Vector3d a1 = h * start.dp_dw;
    const Eigen::Vector3d a2 = 0.5 * h * h * start.d2p_dw2;
    const Eigen::Vector3d r0 = end.p - a0 - a1 - a2;
    const Eigen::Vector3d r1 = h * end.dp_dw - a1 - 2.0 * a2;
    const Eigen::Vector3d r2 = h * h * end.d2p_dw2 - 2.0 * a2;
    const Eigen::Vector3d a3 = 10.0 * r0 - 4.0 * r1 + 0.5 * r2;
    const Eigen::Vector3d a4 = -15.0 * r0 + 7.0 * r1 - r2;
    const Eigen::Vector3d a5 = 6.0 * r0 - 3.0 * r1 + 0.5 * r2;
    const double speed0 = std::max(0.0, start.vel.norm());
    const double speed1 = std::max(0.0, end.vel.norm());
    const std::array<Eigen::Vector3d, 6> coefficients = {{
        a0, a1, a2, a3, a4, a5}};
    bool constant_horizontal_derivative =
        start.dp_dw.x() == end.dp_dw.x() &&
        start.dp_dw.y() == end.dp_dw.y();
    for (std::size_t index = 2U; index < 6U; ++index) {
        constant_horizontal_derivative = constant_horizontal_derivative &&
            coefficients[index].x() == 0.0 && coefficients[index].y() == 0.0;
    }
    const Eigen::Vector3d constant_horizontal_dp_dw = start.dp_dw;

    const auto point_evaluator = [=](double w, ContinuousPhasePathState& state) {
        const double s = std::max(0.0, std::min(1.0, (w - w0) / h));
        const double s2 = s * s;
        const double s3 = s2 * s;
        const double s4 = s3 * s;
        const double s5 = s4 * s;
        state.p = a0 + a1 * s + a2 * s2 + a3 * s3 + a4 * s4 + a5 * s5;
        state.dp_dw = (a1 + 2.0 * a2 * s + 3.0 * a3 * s2 +
                       4.0 * a4 * s3 + 5.0 * a5 * s4) / h;
        state.d2p_dw2 = (2.0 * a2 + 6.0 * a3 * s + 12.0 * a4 * s2 +
                         20.0 * a5 * s3) / (h * h);
        if (constant_horizontal_derivative) {
            const double dw = std::max(0.0, std::min(h, w - w0));
            state.p.x() = a0.x() + constant_horizontal_dp_dw.x() * dw;
            state.p.y() = a0.y() + constant_horizontal_dp_dw.y() * dw;
            state.dp_dw.x() = constant_horizontal_dp_dw.x();
            state.dp_dw.y() = constant_horizontal_dp_dw.y();
            state.d2p_dw2.x() = 0.0;
            state.d2p_dw2.y() = 0.0;
        }
        const double blend = s * s * (3.0 - 2.0 * s);
        const double speed = (1.0 - blend) * speed0 + blend * speed1;
        if (state.dp_dw.norm() > 1e-9) {
            state.vel = state.dp_dw.normalized() * speed;
        } else {
            state.vel.setZero();
        }
        state.valid = finiteState(state);
        return state.valid;
    };
    const CellBoundEvaluator cell_bound_evaluator =
        [coefficients, w0, w1, constant_horizontal_derivative,
         constant_horizontal_dp_dw](const double cell_w0, const double cell_w1,
                                phase_offset_core::PathCellGeometryCertificate& certificate) {
          const Eigen::Vector3d* direct_horizontal_dp_dw =
              constant_horizontal_derivative
                  ? &constant_horizontal_dp_dw : nullptr;
          return MakeQuinticCertificate(coefficients, w0, w1, cell_w0,
                                        cell_w1, direct_horizontal_dp_dw,
                                        certificate);
        };
    const TubeCellBoundsV2Evaluator tube_cell_bounds_v2 =
        [coefficients, w0, w1, constant_horizontal_derivative,
         constant_horizontal_dp_dw](
            const double cell_w0, const double cell_w1,
            phase_offset_core::CertifiedPathCellV2& certificate) {
          const Eigen::Vector3d* direct_horizontal_dp_dw =
              constant_horizontal_derivative
                  ? &constant_horizontal_dp_dw : nullptr;
          return MakeQuinticV2Certificate(
              coefficients, w0, w1, cell_w0, cell_w1,
              direct_horizontal_dp_dw, certificate);
        };
    return Evaluator(point_evaluator, cell_bound_evaluator,
                     tube_cell_bounds_v2);
}

ContinuousPhasePath::Evaluator ContinuousPhasePath::makeMappedBspline(
    const UniformBspline& position,
    double spline_t_anchor,
    double spline_t_end,
    double phase_w_anchor,
    double phase_w_end)
{
    if (!std::isfinite(spline_t_anchor) || !std::isfinite(spline_t_end) ||
        !std::isfinite(phase_w_anchor) || !std::isfinite(phase_w_end) ||
        spline_t_end <= spline_t_anchor + kDomainEps ||
        phase_w_end <= phase_w_anchor + kDomainEps) {
        return Evaluator();
    }

    const UniformBspline p = position;
    const UniformBspline dp_dt = p.getDerivative();
    const UniformBspline d2p_dt2 = dp_dt.getDerivative();
    const double phase_span = phase_w_end - phase_w_anchor;

    CellBoundEvaluator linear_cell_bound_evaluator;
    TubeCellBoundsV2Evaluator linear_tube_cell_bounds_v2;
    CertificateBreakpointsV2Evaluator linear_breakpoints_v2;
    if (p.p_ >= 3) {
        const UniformBspline d3p_dt3 = d2p_dt2.getDerivative();
        linear_cell_bound_evaluator =
            [dp_dt, d2p_dt2, d3p_dt3, spline_t_anchor, spline_t_end,
             phase_w_anchor, phase_w_end](
                const double cell_w0, const double cell_w1,
                phase_offset_core::PathCellGeometryCertificate& certificate) {
              return MakeLinearMappedBsplineCertificate(
                  dp_dt, d2p_dt2, d3p_dt3, spline_t_anchor, spline_t_end,
                  phase_w_anchor, phase_w_end, cell_w0, cell_w1,
                  certificate);
            };
        linear_tube_cell_bounds_v2 =
            [p, dp_dt, d2p_dt2, d3p_dt3, spline_t_anchor, spline_t_end,
             phase_w_anchor, phase_w_end](
                const double cell_w0, const double cell_w1,
                phase_offset_core::CertifiedPathCellV2& certificate) {
              return MakeLinearMappedBsplineV2Certificate(
                  p, dp_dt, d2p_dt2, d3p_dt3, spline_t_anchor, spline_t_end,
                  phase_w_anchor, phase_w_end, cell_w0, cell_w1,
                  certificate);
            };
        // The strict arclength map may be unavailable (for example when a
        // raw endpoint is stationary), but the linear fallback still has a
        // represented phase map.  Retain its structural knots so callers do
        // not silently merge proof cells across a spline seam.
        linear_breakpoints_v2 =
            [p, spline_t_anchor, spline_t_end, phase_w_anchor, phase_w_end](
                std::vector<double>& breakpoints) {
              breakpoints.clear();
              if (!ProofFloatingPointEnvironmentSupported() ||
                  !std::isfinite(spline_t_anchor) ||
                  !std::isfinite(spline_t_end) ||
                  !(spline_t_end > spline_t_anchor) ||
                  !std::isfinite(phase_w_anchor) ||
                  !std::isfinite(phase_w_end) ||
                  !(phase_w_end > phase_w_anchor) ||
                  !std::isfinite(p.beta_) || p.beta_ <= 0.0 ||
                  !SupportedSplineTimeDomain(p, spline_t_anchor,
                                             spline_t_end)) {
                  return false;
              }
              breakpoints.push_back(phase_w_anchor);
              breakpoints.push_back(phase_w_end);
              const ProofInterval phase_span = ProofSub(
                  PointProofInterval(phase_w_end),
                  PointProofInterval(phase_w_anchor));
              const ProofInterval time_span = ProofSub(
                  PointProofInterval(spline_t_end),
                  PointProofInterval(spline_t_anchor));
              if (!ProofFinite(phase_span) || !ProofFinite(time_span)) {
                  breakpoints.clear();
                  return false;
              }
              for (int index = p.p_; index <= p.m_ - p.p_; ++index) {
                  const ProofInterval knot_t = ProofDiv(
                      ProofSub(PointProofInterval(p.u_(index)),
                               PointProofInterval(p.u_(p.p_))),
                      PointProofInterval(p.beta_));
                  if (!ProofFinite(knot_t)) return false;
                  if (knot_t.upper < spline_t_anchor ||
                      knot_t.lower > spline_t_end) {
                      continue;
                  }
                  const ProofInterval phase = ProofAdd(
                      PointProofInterval(phase_w_anchor),
                      ProofDiv(ProofMul(ProofSub(knot_t,
                                                   PointProofInterval(
                                                       spline_t_anchor)),
                                             phase_span),
                              time_span));
                  if (!ProofFinite(phase)) return false;
                  const double phase_lower = std::max(
                      phase_w_anchor, phase.lower);
                  const double phase_upper = std::min(
                      phase_w_end, phase.upper);
                  if (!std::isfinite(phase_lower) ||
                      !std::isfinite(phase_upper) || phase_lower > phase_upper) {
                      return false;
                  }
                  breakpoints.push_back(phase_lower);
                  breakpoints.push_back(phase_upper);
              }
              std::sort(breakpoints.begin(), breakpoints.end());
              breakpoints.erase(std::unique(breakpoints.begin(),
                                             breakpoints.end()),
                                breakpoints.end());
              return !breakpoints.empty();
            };
    }

    // A certificate/map proof is optional metadata.  Preserve a finite
    // legacy point evaluator even when the strict executable Hermite map
    // cannot be built; navigation will then receive no cell certificate and
    // retain the historical fixed inset.  This fallback intentionally uses
    // the original linear t(w) contract and does not claim arclength bounds.
    const auto linear_point_evaluator = [=](double w,
                                            ContinuousPhasePathState& state) {
        const double bounded_w = std::max(
            phase_w_anchor, std::min(w, phase_w_end));
        const double dt_dw = (spline_t_end - spline_t_anchor) / phase_span;
        const double t = spline_t_anchor +
            dt_dw * (bounded_w - phase_w_anchor);
        state.p = evaluateSplineAtTime(p, t);
        state.vel = evaluateSplineAtTime(dp_dt, t);
        const Eigen::Vector3d acceleration = evaluateSplineAtTime(d2p_dt2, t);
        state.dp_dw = state.vel * dt_dw;
        state.d2p_dw2 = acceleration * dt_dw * dt_dw;
        state.valid = finiteState(state);
        return state.valid;
    };
    // This producer certifies the *implemented* monotone quintic-Hermite
    // map S(t) below and its inverse.  It intentionally does not claim that
    // the Simpson table is a validated exact integral of ||p_t||.

    // The planner B-spline is time-parameterized and may have stationary raw
    // endpoints.  The caller must first select a strictly executable time
    // interval.  Inside that interval, build a deterministic normalized-
    // arclength map so phase speed cannot inherit a small |p_t| from the time
    // parameterization.
    ArcLengthMap executable_map;
    if (!buildExecutableArcLengthMap(
            p, spline_t_anchor, spline_t_end, executable_map)) {
        return Evaluator(linear_point_evaluator, linear_cell_bound_evaluator,
                         linear_tube_cell_bounds_v2, linear_breakpoints_v2);
    }
    const double total_length = executable_map.table_s.back();
    if (!std::isfinite(total_length) || total_length <= kDomainEps) {
        return Evaluator(linear_point_evaluator, linear_cell_bound_evaluator,
                         linear_tube_cell_bounds_v2, linear_breakpoints_v2);
    }
    const double arclength_per_phase = total_length / phase_span;

    const auto point_evaluator = [=](double w, ContinuousPhasePathState& state) {
        const double bounded_w = std::max(
            phase_w_anchor, std::min(w, phase_w_end));
        const double target_s = std::max(
            0.0, std::min(total_length,
                arclength_per_phase * (bounded_w - phase_w_anchor)));
        double t = spline_t_anchor;
        double ds_dt = 0.0;
        double d2s_dt2 = 0.0;
        if (!invertArcLengthMap(executable_map, target_s, t, ds_dt,
                                d2s_dt2)) {
            return false;
        }
        state.p = evaluateSplineAtTime(p, t);
        state.vel = evaluateSplineAtTime(dp_dt, t);
        const Eigen::Vector3d acceleration = evaluateSplineAtTime(d2p_dt2, t);
        const double speed = state.vel.norm();
        if (!state.vel.allFinite() || !acceleration.allFinite() ||
            !std::isfinite(speed) || speed <= kArcLengthSpeedEps ||
            !std::isfinite(ds_dt) || ds_dt <= kArcLengthSpeedEps ||
            !std::isfinite(d2s_dt2)) {
            return false;
        }
        const double dt_dw = arclength_per_phase / ds_dt;
        const double d2t_dw2 = -d2s_dt2 * arclength_per_phase *
            arclength_per_phase / (ds_dt * ds_dt * ds_dt);
        state.dp_dw = state.vel * dt_dw;
        state.d2p_dw2 = acceleration * dt_dw * dt_dw +
                        state.vel * d2t_dw2;
        state.valid = finiteState(state);
        return state.valid;
    };
    CellBoundEvaluator cell_bound_evaluator;
    TubeCellBoundsV2Evaluator tube_cell_bounds_v2;
    if (p.p_ >= 3) {
        const UniformBspline d3p_dt3 = d2p_dt2.getDerivative();
        cell_bound_evaluator =
            [p, dp_dt, d2p_dt2, d3p_dt3, executable_map, spline_t_anchor,
             spline_t_end, phase_w_anchor, phase_w_end, arclength_per_phase](
                const double cell_w0, const double cell_w1,
                phase_offset_core::PathCellGeometryCertificate& certificate) {
              return MakeMappedBsplineCertificate(
                  p, dp_dt, d2p_dt2, d3p_dt3, executable_map,
                  spline_t_anchor, spline_t_end, phase_w_anchor,
                  phase_w_end, arclength_per_phase, cell_w0, cell_w1,
                  certificate);
            };
        tube_cell_bounds_v2 =
            [p, dp_dt, d2p_dt2, d3p_dt3, executable_map,
             spline_t_anchor, spline_t_end, phase_w_anchor, phase_w_end,
             arclength_per_phase](
                const double cell_w0, const double cell_w1,
                phase_offset_core::CertifiedPathCellV2& certificate) {
              return MakeMappedBsplineV2Certificate(
                  p, dp_dt, d2p_dt2, d3p_dt3, executable_map,
                  spline_t_anchor, spline_t_end, phase_w_anchor,
                  phase_w_end, arclength_per_phase, cell_w0, cell_w1,
                  certificate);
            };
        linear_breakpoints_v2 =
            [executable_map, phase_w_anchor, arclength_per_phase, p,
             spline_t_anchor, spline_t_end, phase_w_end](
                std::vector<double>& breakpoints) {
              breakpoints.clear();
              if (!ProofFloatingPointEnvironmentSupported() ||
                  !std::isfinite(arclength_per_phase) ||
                  arclength_per_phase <= 0.0 ||
                  !SupportedSplineTimeDomain(p, spline_t_anchor,
                                             spline_t_end)) return false;
              breakpoints.push_back(phase_w_anchor);
              const ProofInterval phase_scale =
                  PointProofInterval(arclength_per_phase);
              // Every stored map-cell boundary is a structural breakpoint of
              // the represented quintic map.  Convert it with outward
              // arithmetic and retain the enclosure endpoints; no rounded
              // linear interpolation is used as proof evidence.
              for (std::size_t table_index = 1U;
                   table_index + 1U < executable_map.table_s.size();
                   ++table_index) {
                  const ProofInterval map_s = PointProofInterval(
                      executable_map.table_s[table_index]);
                  const ProofInterval phase = ProofAdd(
                      PointProofInterval(phase_w_anchor),
                      ProofDiv(map_s, phase_scale));
                  if (!ProofFinite(phase)) return false;
                  const double phase_lower = std::max(
                      phase_w_anchor, phase.lower);
                  const double phase_upper = std::min(
                      phase_w_end, phase.upper);
                  if (!std::isfinite(phase_lower) ||
                      !std::isfinite(phase_upper) || phase_lower > phase_upper) {
                      return false;
                  }
                  breakpoints.push_back(phase_lower);
                  breakpoints.push_back(phase_upper);
              }
              breakpoints.push_back(phase_w_end);
              // Preserve every stored spline knot as a structural identity.
              // Evaluate the represented quintic S(t) over an outward time
              // enclosure.  If the rounded phase is not a singleton, retain
              // both enclosure endpoints so the uncertainty is explicit in
              // the partition instead of being presented as an exact linear
              // seam.
              for (int index = p.p_; index <= p.m_ - p.p_; ++index) {
                  const ProofInterval knot_t = ProofDiv(
                      ProofSub(PointProofInterval(p.u_(index)),
                               PointProofInterval(p.u_(p.p_))),
                      PointProofInterval(p.beta_));
                  if (!ProofFinite(knot_t)) return false;
                  if (knot_t.upper < spline_t_anchor ||
                      knot_t.lower > spline_t_end) {
                      continue;
                  }
                  ProofInterval map_s;
                  if (!ProofArcLengthValueAtTime(executable_map, knot_t,
                                                 map_s)) return false;
                  const ProofInterval phase = ProofAdd(
                      PointProofInterval(phase_w_anchor),
                      ProofDiv(map_s, phase_scale));
                  if (!ProofFinite(phase)) return false;
                  const double phase_lower = std::max(
                      phase_w_anchor, phase.lower);
                  const double phase_upper = std::min(
                      phase_w_end, phase.upper);
                  if (!std::isfinite(phase_lower) ||
                      !std::isfinite(phase_upper) || phase_lower > phase_upper) {
                      return false;
                  }
                  breakpoints.push_back(phase_lower);
                  breakpoints.push_back(phase_upper);
              }
              std::sort(breakpoints.begin(), breakpoints.end());
              breakpoints.erase(std::unique(breakpoints.begin(),
                                             breakpoints.end()),
                                breakpoints.end());
              return !breakpoints.empty();
            };
    }
    return Evaluator(point_evaluator, cell_bound_evaluator,
                     tube_cell_bounds_v2, linear_breakpoints_v2);
}

bool ContinuousPhasePath::trimBsplineTimeDomainByArcLength(
    const UniformBspline& position,
    double spline_t_start,
    double spline_t_end,
    double trim_start_length,
    double trim_end_length,
    double& executable_t_start,
    double& executable_t_end,
    double& total_length)
{
    executable_t_start = 0.0;
    executable_t_end = 0.0;
    total_length = 0.0;
    if (!std::isfinite(trim_start_length) || !std::isfinite(trim_end_length) ||
        trim_start_length < 0.0 || trim_end_length < 0.0) {
        return false;
    }
    std::vector<double> table_t;
    std::vector<double> table_s;
    // Raw endpoint trim accepts stationary planner endpoint constraints;
    // makeMappedBspline later builds the strict-positive executable map.
    if (!buildArcLengthTable(position, spline_t_start, spline_t_end, false,
                             table_t, table_s, nullptr)) {
        return false;
    }
    total_length = table_s.back();
    if (trim_start_length + trim_end_length >= total_length - kDomainEps) {
        return false;
    }
    if (!invertRawArcLengthTable(position, table_t, table_s, trim_start_length,
                                 executable_t_start) ||
        !invertRawArcLengthTable(position, table_t, table_s,
                                 total_length - trim_end_length,
                                 executable_t_end)) {
        return false;
    }
    return std::isfinite(executable_t_start) && std::isfinite(executable_t_end) &&
           executable_t_end > executable_t_start + kDomainEps;
}

ContinuousPhasePath::Evaluator ContinuousPhasePath::makePeriodicCircle(
    const Eigen::Vector3d& center,
    double radius,
    double period_w,
    double nominal_speed)
{
    if (!center.allFinite() || !std::isfinite(radius) || radius <= 0.0 ||
        !std::isfinite(period_w) || period_w <= kDomainEps) {
        return Evaluator();
    }
    const double omega = 2.0 * M_PI / period_w;
    const double speed = std::max(0.0, nominal_speed);
    return [=](double w, ContinuousPhasePathState& state) {
        const double theta = omega * w;
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        state.p = center + Eigen::Vector3d(radius * c, radius * s, 0.0);
        state.dp_dw = Eigen::Vector3d(-radius * omega * s,
                                      radius * omega * c, 0.0);
        state.d2p_dw2 = Eigen::Vector3d(-radius * omega * omega * c,
                                        -radius * omega * omega * s, 0.0);
        state.vel = state.dp_dw.normalized() * speed;
        state.valid = finiteState(state);
        return state.valid;
    };
}

ContinuousPhasePath::Evaluator ContinuousPhasePath::makePeriodicFigureEight(
    const Eigen::Vector3d& center,
    double radius,
    double period_w,
    double nominal_speed)
{
    if (!center.allFinite() || !std::isfinite(radius) || radius <= 0.0 ||
        !std::isfinite(period_w) || period_w <= kDomainEps) {
        return Evaluator();
    }
    const double omega = 2.0 * M_PI / period_w;
    const double speed = std::max(0.0, nominal_speed);
    return [=](double w, ContinuousPhasePathState& state) {
        const double theta = omega * w;
        const double sin_theta = std::sin(theta);
        const double cos_theta = std::cos(theta);
        const double sin_2theta = std::sin(2.0 * theta);
        const double cos_2theta = std::cos(2.0 * theta);
        state.p = center + Eigen::Vector3d(0.5 * radius * sin_2theta,
                                           radius * sin_theta, 0.0);
        state.dp_dw = Eigen::Vector3d(radius * omega * cos_2theta,
                                      radius * omega * cos_theta, 0.0);
        state.d2p_dw2 = Eigen::Vector3d(-2.0 * radius * omega * omega * sin_2theta,
                                        -radius * omega * omega * sin_theta, 0.0);
        state.vel = state.dp_dw.normalized() * speed;
        state.valid = finiteState(state);
        return state.valid;
    };
}

}  // namespace FLAG_Race
