#include "phase_offset_navigation/tube_certificate_v2.h"

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <unordered_set>
#include <utility>

#if defined(__i386__) || defined(__x86_64__)
#include <immintrin.h>
#endif

#pragma STDC FP_CONTRACT OFF

namespace phase_offset_navigation {
namespace {

const double kInf = std::numeric_limits<double>::infinity();

bool Finite(const double value) {
  return std::isfinite(value);
}

double Down(const double value) {
  if (!Finite(value) || value == -kInf) return value;
  return std::nextafter(value, -kInf);
}

double Up(const double value) {
  if (!Finite(value) || value == kInf) return value;
  return std::nextafter(value, kInf);
}

bool SupportedFloatingPointEnvironment() {
  // The proof uses binary64 operations followed by one nextafter expansion.
  // A non-nearest rounding mode or an unavailable fenv result is not an
  // alternative proof mode; it is an explicit fail-closed condition.
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
  // DAZ is bit 6 of MXCSR.  Some immintrin versions omit the convenience
  // macro, so check the architectural bit explicitly.
  if ((csr & 0x0040U) != 0U) return false;
  return true;
#else
  return false;
#endif
}

struct Interval {
  double lower = 0.0;
  double upper = 0.0;
  bool valid = false;
};

Interval InvalidInterval() {
  return Interval();
}

Interval Point(const double value) {
  if (!Finite(value)) return InvalidInterval();
  Interval result;
  result.lower = value;
  result.upper = value;
  result.valid = true;
  return result;
}

Interval Bounds(const double lower, const double upper) {
  if (!Finite(lower) || !Finite(upper) || lower > upper) {
    return InvalidInterval();
  }
  Interval result;
  result.lower = lower;
  result.upper = upper;
  result.valid = true;
  return result;
}

bool ExactZero(const Interval& value) {
  return value.valid && value.lower == 0.0 && value.upper == 0.0;
}

Interval Negate(const Interval& value) {
  if (!value.valid) return InvalidInterval();
  return Bounds(-value.upper, -value.lower);
}

Interval FromCore(const phase_offset_core::Binary64Interval& value) {
  if (!phase_offset_core::binary64IntervalIsComplete(value)) {
    return InvalidInterval();
  }
  return Bounds(value.lower, value.upper);
}

Interval Add(const Interval& lhs, const Interval& rhs) {
  if (!lhs.valid || !rhs.valid) return InvalidInterval();
  if (ExactZero(lhs)) return rhs;
  if (ExactZero(rhs)) return lhs;
  const double lower = lhs.lower + rhs.lower;
  const double upper = lhs.upper + rhs.upper;
  if (!Finite(lower) || !Finite(upper)) return InvalidInterval();
  return Bounds(Down(lower), Up(upper));
}

Interval Subtract(const Interval& lhs, const Interval& rhs) {
  if (!lhs.valid || !rhs.valid) return InvalidInterval();
  // Equal singleton endpoints denote the exact same binary64 value.  Their
  // mathematical difference is therefore exactly zero; expanding the
  // rounded subtraction by one ULP would incorrectly turn a zero numerator
  // into a sign-changing interval (notably in the regularity cap when
  // m == minimum_reference_speed).
  if (lhs.lower == lhs.upper && rhs.lower == rhs.upper &&
      lhs.lower == rhs.lower) {
    return Point(0.0);
  }
  if (ExactZero(rhs)) return lhs;
  if (ExactZero(lhs)) return Negate(rhs);
  const double lower = lhs.lower - rhs.upper;
  const double upper = lhs.upper - rhs.lower;
  if (!Finite(lower) || !Finite(upper)) return InvalidInterval();
  return Bounds(Down(lower), Up(upper));
}

Interval Multiply(const Interval& lhs, const Interval& rhs) {
  if (!lhs.valid || !rhs.valid) return InvalidInterval();
  if (ExactZero(lhs) || ExactZero(rhs)) return Point(0.0);
  const double values[4] = {
      lhs.lower * rhs.lower, lhs.lower * rhs.upper,
      lhs.upper * rhs.lower, lhs.upper * rhs.upper};
  double lower = values[0];
  double upper = values[0];
  for (const double value : values) {
    if (!Finite(value)) return InvalidInterval();
    lower = std::min(lower, value);
    upper = std::max(upper, value);
  }
  return Bounds(Down(lower), Up(upper));
}

Interval DividePositive(const Interval& numerator,
                        const Interval& denominator) {
  if (!numerator.valid || !denominator.valid ||
      !(denominator.lower > 0.0) || denominator.upper < denominator.lower) {
    return InvalidInterval();
  }
  if (ExactZero(numerator)) return Point(0.0);
  const double values[4] = {
      numerator.lower / denominator.lower, numerator.lower / denominator.upper,
      numerator.upper / denominator.lower, numerator.upper / denominator.upper};
  double lower = values[0];
  double upper = values[0];
  for (const double value : values) {
    if (!Finite(value)) return InvalidInterval();
    lower = std::min(lower, value);
    upper = std::max(upper, value);
  }
  return Bounds(Down(lower), Up(upper));
}

Interval SqrtInterval(const Interval& value) {
  if (!value.valid || value.lower < 0.0) return InvalidInterval();
  const double lower = std::sqrt(value.lower);
  const double upper = std::sqrt(value.upper);
  if (!Finite(lower) || !Finite(upper)) return InvalidInterval();
  return Bounds(std::max(0.0, Down(lower)), Up(upper));
}

Interval Square(const Interval& value) {
  if (!value.valid) return InvalidInterval();
  if (value.lower <= 0.0 && value.upper >= 0.0) {
    const double upper = std::max(value.lower * value.lower,
                                  value.upper * value.upper);
    if (!Finite(upper)) return InvalidInterval();
    return Bounds(0.0, Up(upper));
  }
  return Multiply(value, value);
}

bool FiniteVectorInterval(const phase_offset_core::Binary64VectorInterval& value) {
  return phase_offset_core::binary64VectorIntervalIsComplete(value);
}

Interval VectorComponent(const phase_offset_core::Binary64VectorInterval& value,
                         const std::size_t index) {
  if (index >= value.component.size()) return InvalidInterval();
  return FromCore(value.component[index]);
}

// A subdivision midpoint must be a representable strict interior value so
// that both child cells retain nonzero width.  Path-cell anchors, by
// contrast, are supplied as proof data and may be either endpoint of an
// adjacent-ULP cell when the mathematical midpoint is not representable.
bool StrictSplitMidpoint(const double lower, const double upper,
                         double& result) {
  if (!Finite(lower) || !Finite(upper) || !(upper > lower)) return false;
  const double half = (upper - lower) * 0.5;
  if (!Finite(half)) return false;
  result = lower + half;
  return Finite(result) && result > lower && result < upper;
}

int CompareProductToDouble(const double lhs, const double rhs,
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

double Binary64FromBits(const std::uint64_t bits) {
  double result = 0.0;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

std::uint64_t Binary64Bits(const double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

// Return the largest representable non-negative binary64 value no greater
// than the exact dyadic product lhs*rhs.  Unlike an outward interval bound,
// this is used for sample indexing: a k*s endpoint must never overshoot the
// mathematical grid.  Exact products are preserved verbatim; only a truly
// nonrepresentable product is floored by one ULP (with a bounded exact-search
// fallback for an implementation that rounds by more than one ULP).
bool ProductFloor(const double lhs, const double rhs, double& result) {
  if (!Finite(lhs) || !Finite(rhs) || lhs < 0.0 || rhs < 0.0) return false;
  if (lhs == 0.0 || rhs == 0.0) {
    result = 0.0;
    return true;
  }
  double candidate = lhs * rhs;
  if (!Finite(candidate)) {
    const double maximum = std::numeric_limits<double>::max();
    if (CompareProductToDouble(lhs, rhs, maximum) > 0) return false;
    candidate = maximum;
  }
  const int comparison = CompareProductToDouble(lhs, rhs, candidate);
  if (comparison >= 0) {
    // Candidate is exact or rounded below the product, hence it is already
    // the required floor under the non-negative binary64 ordering.
    result = candidate;
    return true;
  }
  const double previous = std::nextafter(candidate, 0.0);
  if (Finite(previous) &&
      CompareProductToDouble(lhs, rhs, previous) >= 0) {
    result = previous;
    return true;
  }

  // Defensive exact fallback.  The ordinary operation should reach the
  // branch above; this search keeps the endpoint contract independent of a
  // nonconforming evaluator without introducing a tolerance.
  std::uint64_t low = 0U;
  std::uint64_t high = Binary64Bits(candidate);
  while (low < high) {
    const std::uint64_t middle = low + (high - low + 1U) / 2U;
    if (CompareProductToDouble(lhs, rhs, Binary64FromBits(middle)) >= 0) {
      low = middle;
    } else {
      high = middle - 1U;
    }
  }
  result = Binary64FromBits(low);
  return Finite(result);
}

bool VectorFinite(const Eigen::Vector3d& value) {
  return value.allFinite();
}

bool ComputeNormInterval(const phase_offset_core::Binary64VectorInterval& value,
                         Interval& norm) {
  if (!FiniteVectorInterval(value)) return false;
  const Interval x = VectorComponent(value, 0U);
  const Interval y = VectorComponent(value, 1U);
  const Interval z = VectorComponent(value, 2U);
  (void)z;
  const Interval squared = Add(Square(x), Square(y));
  norm = SqrtInterval(squared);
  return norm.valid;
}

bool ComputeHorizontalNormal(
    const phase_offset_core::CertifiedPathCellV2& cell,
    phase_offset_core::Binary64VectorInterval& normal,
    double& q_min) {
  if (!FiniteVectorInterval(cell.anchor_p_w) ||
      !phase_offset_core::binary64IntervalIsComplete(
          cell.inf_horizontal_p_w_norm, true)) return false;
  const Interval p_w_norm = [&cell]() {
    Interval result;
    return ComputeNormInterval(cell.anchor_p_w, result) ? result
                                                        : InvalidInterval();
  }();
  if (!p_w_norm.valid || !Finite(p_w_norm.upper)) return false;
  q_min = cell.inf_horizontal_p_w_norm.lower;
  if (!Finite(q_min) ||
      !(q_min > phase_offset_core::kHorizontalNormalSpeedEpsilon) ||
      p_w_norm.upper < q_min) return false;
  const Interval denominator = Bounds(q_min, p_w_norm.upper);
  if (!denominator.valid) return false;
  const Interval px = VectorComponent(cell.anchor_p_w, 0U);
  const Interval py = VectorComponent(cell.anchor_p_w, 1U);
  const Interval nx = DividePositive(
      Bounds(-py.upper, -py.lower), denominator);
  const Interval ny = DividePositive(px, denominator);
  if (!nx.valid || !ny.valid) return false;
  normal = phase_offset_core::Binary64VectorInterval();
  normal.component[0].lower = nx.lower;
  normal.component[0].upper = nx.upper;
  normal.component[0].valid = true;
  normal.component[1].lower = ny.lower;
  normal.component[1].upper = ny.upper;
  normal.component[1].valid = true;
  normal.component[2].lower = 0.0;
  normal.component[2].upper = 0.0;
  normal.component[2].valid = true;
  normal.valid = true;
  return true;
}

bool VectorIntervalAtOffset(
    const phase_offset_core::CertifiedPathCellV2& cell,
    const phase_offset_core::Binary64VectorInterval& normal,
    const Interval& delta,
    phase_offset_core::Binary64VectorInterval& position) {
  if (!delta.valid || !FiniteVectorInterval(cell.anchor_position) ||
      !phase_offset_core::binary64VectorIntervalIsComplete(normal)) return false;
  position = phase_offset_core::Binary64VectorInterval();
  for (std::size_t index = 0U; index < 3U; ++index) {
    const Interval p = VectorComponent(cell.anchor_position, index);
    const Interval n = VectorComponent(normal, index);
    const Interval value = Add(p, Multiply(n, delta));
    if (!value.valid) return false;
    position.component[index].lower = value.lower;
    position.component[index].upper = value.upper;
    position.component[index].valid = true;
  }
  position.valid = true;
  return true;
}

bool VectorIntervalAtOffset(
    const phase_offset_core::CertifiedPathCellV2& cell,
    const phase_offset_core::Binary64VectorInterval& normal,
    const double delta,
    phase_offset_core::Binary64VectorInterval& position) {
  return VectorIntervalAtOffset(cell, normal, Point(delta), position);
}

bool MidpointWitness(const phase_offset_core::Binary64VectorInterval& value,
                     Eigen::Vector3d& witness, double& error) {
  if (!FiniteVectorInterval(value)) return false;
  witness = Eigen::Vector3d::Zero();
  double squared_upper = 0.0;
  for (std::size_t index = 0U; index < 3U; ++index) {
    const Interval component = VectorComponent(value, index);
    const Interval midpoint_interval = Multiply(
        Add(Point(component.lower), Point(component.upper)), Point(0.5));
    if (!midpoint_interval.valid) return false;
    double midpoint = midpoint_interval.lower;
    if (midpoint < component.lower) midpoint = component.lower;
    if (midpoint > component.upper) midpoint = component.upper;
    if (!Finite(midpoint)) return false;
    witness(static_cast<Eigen::Index>(index)) = midpoint;
    const Interval lower_distance =
        Subtract(Point(midpoint), Point(component.lower));
    const Interval upper_distance =
        Subtract(Point(component.upper), Point(midpoint));
    if (!lower_distance.valid || !upper_distance.valid) return false;
    const double radius = std::max(lower_distance.upper, upper_distance.upper);
    if (!Finite(radius) || radius < 0.0) return false;
    const Interval square = Multiply(Point(radius), Point(radius));
    const Interval sum = Add(Point(squared_upper), square);
    if (!square.valid || !sum.valid) return false;
    squared_upper = std::max(0.0, sum.upper);
  }
  const Interval root = SqrtInterval(Point(squared_upper));
  if (!root.valid) return false;
  error = root.upper;
  return Finite(error) && VectorFinite(witness);
}

bool BoundsContainZero(const double lower, const double upper) {
  return Finite(lower) && Finite(upper) && lower <= 0.0 && upper >= 0.0;
}

bool ValidSupport(const TubeSupportFootprintV2& support,
                  const Eigen::Vector3d& witness, const double radius,
                  const std::uint64_t map_instance_id,
                  const std::uint64_t map_state_id,
                  const std::uint64_t support_provenance_id,
                  const std::uint64_t accepted_time_ticks,
                  const std::uint64_t support_expiry_ticks,
                  const bool support_expiry_timeless,
                  const TubeMapCaptureKey& map_key) {
  if (!support.valid || !VectorFinite(support.lower) ||
      !VectorFinite(support.upper) || !VectorFinite(support.witness) ||
      !Finite(support.radius) || support.radius < 0.0 ||
      // The retained footprint is evidence for exactly this requested ball.
      // A producer must not smuggle an unproved larger radius into the
      // immutable certificate; callers wanting a larger ball issue another
      // query with that radius.
      support.radius != radius ||
      (support.witness.array() != witness.array()).any() ||
      support.map_instance_id != map_instance_id ||
      support.map_state_id != map_state_id ||
      support.support_provenance_id != support_provenance_id ||
      support.accepted_time_ticks != accepted_time_ticks ||
      support.support_expiry_timeless != support_expiry_timeless ||
      (!support_expiry_timeless &&
       support.support_expiry_ticks != support_expiry_ticks) ||
      (support.lower.array() > support.upper.array()).any() ||
      (witness.array() < support.lower.array()).any() ||
      (witness.array() > support.upper.array()).any()) {
    return false;
  }
  Eigen::Vector3d ball_lower = Eigen::Vector3d::Zero();
  Eigen::Vector3d ball_upper = Eigen::Vector3d::Zero();
  for (std::size_t i = 0U; i < 3U; ++i) {
    const double lower = witness(static_cast<Eigen::Index>(i)) - radius;
    const double upper = witness(static_cast<Eigen::Index>(i)) + radius;
    if (!Finite(lower) || !Finite(upper)) return false;
    ball_lower(static_cast<Eigen::Index>(i)) =
        std::nextafter(lower, -std::numeric_limits<double>::infinity());
    ball_upper(static_cast<Eigen::Index>(i)) =
        std::nextafter(upper, std::numeric_limits<double>::infinity());
  }
  if (!VectorFinite(ball_lower) || !VectorFinite(ball_upper) ||
      (ball_lower.array() < support.lower.array()).any() ||
      (ball_upper.array() > support.upper.array()).any()) {
    return false;
  }
  if (support.voxel_footprint.size() != 1U) return false;
  const TubeVoxelFootprintV2& voxel = support.voxel_footprint.front();
  if (!voxel.valid || voxel.min_index_x > voxel.max_index_x ||
      voxel.min_index_y > voxel.max_index_y ||
      voxel.min_index_z > voxel.max_index_z ||
      voxel.native_index != map_key.grid_native_index ||
      !VectorFinite(voxel.native_origin) ||
      !VectorFinite(voxel.voxel_resolution) ||
      (voxel.voxel_resolution.array() <= 0.0).any() ||
      (voxel.native_origin.array() != map_key.grid_native_origin.array()).any() ||
      (voxel.voxel_resolution.array() !=
           map_key.grid_voxel_resolution.array()).any() ||
      voxel.source_offset_x != map_key.grid_source_offset_x ||
      voxel.source_offset_y != map_key.grid_source_offset_y ||
      voxel.source_offset_z != map_key.grid_source_offset_z ||
      voxel.min_index_x < map_key.grid_min_index_x ||
      voxel.min_index_y < map_key.grid_min_index_y ||
      voxel.min_index_z < map_key.grid_min_index_z ||
      voxel.max_index_x > map_key.grid_max_index_x ||
      voxel.max_index_y > map_key.grid_max_index_y ||
      voxel.max_index_z > map_key.grid_max_index_z) return false;
  return true;
}

TubeCertificateFailureReason QueryFailureReason(
    const TubeFreeBallQueryResult& result, const double required) {
  if (!result.complete_support || !result.certified) {
    switch (result.status) {
      case DistanceStatus::OUT_OF_MAP:
        return TubeCertificateFailureReason::CLEARANCE_OUT_OF_MAP;
      case DistanceStatus::UNKNOWN:
        return TubeCertificateFailureReason::CLEARANCE_UNKNOWN;
      case DistanceStatus::OCCUPIED:
        return TubeCertificateFailureReason::CLEARANCE_OCCUPIED;
      case DistanceStatus::UNAVAILABLE:
        return TubeCertificateFailureReason::CLEARANCE_UNAVAILABLE;
      case DistanceStatus::KNOWN_FREE:
        return TubeCertificateFailureReason::CLEARANCE_UNCERTIFIED;
    }
  }
  switch (result.status) {
    case DistanceStatus::OUT_OF_MAP:
      return TubeCertificateFailureReason::CLEARANCE_OUT_OF_MAP;
    case DistanceStatus::UNKNOWN:
      return TubeCertificateFailureReason::CLEARANCE_UNKNOWN;
    case DistanceStatus::OCCUPIED:
      return TubeCertificateFailureReason::CLEARANCE_OCCUPIED;
    case DistanceStatus::UNAVAILABLE:
      return TubeCertificateFailureReason::CLEARANCE_UNAVAILABLE;
    case DistanceStatus::KNOWN_FREE:
      if (!Finite(result.certified_radius) ||
          result.certified_radius < required) {
        return TubeCertificateFailureReason::CLEARANCE_INSUFFICIENT;
      }
      return TubeCertificateFailureReason::NONE;
  }
  return TubeCertificateFailureReason::CLEARANCE_UNAVAILABLE;
}

struct CellAttempt {
  TubeProofCellV2 proof;
  bool anchor_safe = false;
  bool nonzero = false;
  TubeCertificateFailureReason failure = TubeCertificateFailureReason::NONE;
  double failure_delta = 0.0;
  bool valid = false;
};

struct QueryState {
  const TubeBuildInputV2& input;
  const TubeCertificateConfigV2& config;
  TubeCertificateBuildStatsV2& stats;
  TubeFailureAttribution& failure;
  std::size_t cell_samples = 0U;
};

bool QueryFreeBall(QueryState& state, const Eigen::Vector3d& witness,
                  const double required, const int depth,
                  TubeSupportFootprintV2& support,
                  TubeCertificateFailureReason& failure) {
  if (state.stats.query_count >= state.config.budgets.max_queries) {
    state.stats.query_budget_reached = true;
    failure = TubeCertificateFailureReason::QUERY_BUDGET;
    return false;
  }
  if (state.stats.witness_count >= state.config.budgets.max_witnesses) {
    state.stats.witness_budget_reached = true;
    failure = TubeCertificateFailureReason::WITNESS_BUDGET;
    return false;
  }
  if (state.cell_samples >= state.config.budgets.max_samples_per_cell) {
    state.stats.sample_budget_reached = true;
    failure = TubeCertificateFailureReason::SAMPLE_BUDGET;
    return false;
  }
  if (!state.input.free_ball_query || !VectorFinite(witness) ||
      !Finite(required) || required < 0.0) {
    failure = TubeCertificateFailureReason::UNAVAILABLE_PRODUCER;
    return false;
  }
  ++state.stats.query_count;
  ++state.stats.witness_count;
  ++state.cell_samples;
  if (depth > 0) ++state.stats.child_query_count;
  const TubeFreeBallQueryResult result = state.input.free_ball_query(
      witness, required);
  const TubeCertificateFailureReason result_failure =
      QueryFailureReason(result, required);
  if (result_failure != TubeCertificateFailureReason::NONE ||
      result.map_instance_id != state.input.map_capture_key.map_instance_id ||
      result.map_state_id != state.input.map_capture_key.state_id ||
      result.configuration_id != state.input.map_capture_key.configuration_id ||
      result.frame_provenance_id !=
          state.input.map_capture_key.frame_provenance_id ||
      result.frame_provenance != state.input.map_capture_key.frame_provenance ||
      result.accepted_sequence != state.input.map_capture_key.accepted_sequence ||
      result.configuration_generation !=
          state.input.map_capture_key.configuration_generation ||
      result.support_provenance_id !=
          state.input.map_capture_key.support_provenance_id ||
      result.accepted_time_ticks !=
          state.input.map_capture_key.accepted_time_ticks ||
      result.support_expiry_timeless !=
          state.input.map_capture_key.support_expiry_timeless ||
      (!result.support_expiry_timeless &&
       result.support_expiry_ticks !=
           state.input.map_capture_key.support_expiry_ticks) ||
      result.halo_reconciled != state.input.map_capture_key.halo_reconciled ||
      !ValidSupport(result.support, witness, required,
                    state.input.map_capture_key.map_instance_id,
                    state.input.map_capture_key.state_id,
                    state.input.map_capture_key.support_provenance_id,
                    state.input.map_capture_key.accepted_time_ticks,
                    state.input.map_capture_key.support_expiry_ticks,
                    state.input.map_capture_key.support_expiry_timeless,
                    state.input.map_capture_key)) {
    ++state.stats.failed_query_count;
    failure = result_failure == TubeCertificateFailureReason::NONE
        ? TubeCertificateFailureReason::MAP_CAPTURE_MISMATCH : result_failure;
    return false;
  }
  support = result.support;
  return true;
}

bool PreparePathCell(QueryState& state,
                     const phase_offset_core::CertifiedPathCellV2& fallback,
                     const double w0, const double w1,
                     phase_offset_core::CertifiedPathCellV2& result) {
  if (state.input.path_cell_query) {
    if (!state.input.path_owner) return false;
    if (state.stats.query_count + state.stats.path_cell_query_count >=
        state.config.budgets.max_queries) {
      state.stats.query_budget_reached = true;
      return false;
    }
    ++state.stats.path_cell_query_count;
    if (!state.input.path_cell_query(w0, w1, result) || result.w0 != w0 ||
        result.w1 != w1 ||
        result.w0 < state.input.path_key.domain_start ||
        result.w1 > state.input.path_key.domain_end ||
        result.path_revision != state.input.path_key.path_revision ||
        result.frame_revision != state.input.path_key.frame_revision ||
        !phase_offset_core::certifiedPathCellV2IsComplete(result)) {
      ++state.stats.failed_path_cell_query_count;
      return false;
    }
    return true;
  }
  // The producer's immutable proof owns the exact endpoints.  Do not impose
  // a representable midpoint requirement here: a valid one-ULP cell may only
  // have an endpoint-valued rounded anchor.  The core completeness predicate
  // below remains authoritative for the anchor-in-cell and interval checks.
  if (fallback.w0 != w0 || fallback.w1 != w1 ||
      fallback.anchor_w < w0 || fallback.anchor_w > w1) {
    return false;
  }
  result = fallback;
  return result.path_revision == state.input.path_key.path_revision &&
      result.frame_revision == state.input.path_key.frame_revision &&
      phase_offset_core::certifiedPathCellV2IsComplete(result);
}

bool BuildRegularity(const phase_offset_core::CertifiedPathCellV2& cell,
                     const TubeCertificateConfigV2& config, double& q_min,
                     double& p_bound, double& a_xy, double& q_regular,
                     double& cap) {
  if (!phase_offset_core::certifiedPathCellV2IsComplete(cell)) return false;
  q_min = cell.inf_horizontal_p_w_norm.lower;
  p_bound = cell.sup_p_w_norm.upper;
  a_xy = cell.sup_horizontal_p_ww_norm.upper;
  const double m = cell.inf_p_w_norm.lower;
  if (!Finite(q_min) || !Finite(p_bound) || !Finite(a_xy) || !Finite(m) ||
      q_min <= phase_offset_core::kHorizontalNormalSpeedEpsilon ||
      p_bound < 0.0 || a_xy < 0.0 || m < 0.0 ||
      !Finite(config.minimum_reference_speed) ||
      config.minimum_reference_speed <= 0.0) return false;
  if (a_xy == 0.0) {
    q_regular = 0.0;
  } else {
    const Interval ratio = DividePositive(Point(a_xy), Point(q_min));
    if (!ratio.valid) return false;
    q_regular = ratio.upper;
  }
  if (q_regular == 0.0) {
    if (m < config.minimum_reference_speed) return false;
    cap = config.nominal_half_width;
  } else {
    const Interval numerator = Subtract(Point(m),
                                        Point(config.minimum_reference_speed));
    const Interval quotient = DividePositive(numerator, Point(q_regular));
    if (!quotient.valid) return false;
    cap = std::min(config.nominal_half_width, quotient.lower);
    if (!Finite(cap) || cap < 0.0) return false;
  }
  const Interval check = Subtract(Point(m), Multiply(Point(q_regular), Point(cap)));
  if (!check.valid || check.lower < config.minimum_reference_speed) return false;
  return Finite(cap);
}

bool MotionCover(const double p_bound, const double q_regular,
                 const double nominal_half_width, const double w0,
                 const double w1, double& cover) {
  if (!Finite(p_bound) || !Finite(q_regular) || !Finite(nominal_half_width) ||
      !Finite(w0) || !Finite(w1) || !(w1 > w0)) return false;
  const Interval sum = Add(Point(p_bound),
                           Multiply(Point(nominal_half_width),
                                    Point(q_regular)));
  const Interval width = Subtract(Point(w1), Point(w0));
  const Interval span = Multiply(width, Point(0.5));
  Interval result = Multiply(sum, span);
  if (!result.valid || !Finite(result.upper)) return false;
  result.lower = std::max(0.0, result.lower);
  cover = result.upper;
  return true;
}

bool QuerySegment(QueryState& state,
                  const phase_offset_core::CertifiedPathCellV2& cell,
                  const phase_offset_core::Binary64VectorInterval& normal,
                  const double motion_cover, const double epsilon,
                  const double d0, const double d1, const int depth,
                  TubeSupportFootprintV2& support, double& extent,
                  TubeCertificateFailureReason& failure) {
  if (!Finite(d0) || !Finite(d1) || !(d1 > d0)) {
    failure = TubeCertificateFailureReason::WITNESS_FAILURE;
    return false;
  }
  const Interval segment_width = Subtract(Point(d1), Point(d0));
  const Interval delta_mid = Multiply(
      Add(Point(d0), Point(d1)), Point(0.5));
  if (!segment_width.valid || !delta_mid.valid) {
    failure = TubeCertificateFailureReason::INTERVAL_OVERFLOW;
    return false;
  }
  const double eta = Multiply(segment_width, Point(0.5)).upper;
  if (!Finite(eta) || eta < 0.0) {
    failure = TubeCertificateFailureReason::INTERVAL_OVERFLOW;
    return false;
  }
  phase_offset_core::Binary64VectorInterval position;
  if (!VectorIntervalAtOffset(cell, normal, delta_mid, position)) {
    failure = TubeCertificateFailureReason::NONFINITE_INTERVAL;
    return false;
  }
  Eigen::Vector3d witness;
  double error = 0.0;
  if (!MidpointWitness(position, witness, error)) {
    failure = TubeCertificateFailureReason::NONFINITE_INTERVAL;
    return false;
  }
  const Interval required_interval = Add(
      Point(epsilon), Add(Point(motion_cover),
                          Add(Point(eta), Point(error))));
  if (!required_interval.valid || required_interval.upper < 0.0) {
    failure = TubeCertificateFailureReason::INTERVAL_OVERFLOW;
    return false;
  }
  const double required = required_interval.upper;
  if (!QueryFreeBall(state, witness, required, depth, support, failure)) {
    return false;
  }
  extent = d1;
  return true;
}

CellAttempt EvaluateCell(QueryState& state,
                         const phase_offset_core::CertifiedPathCellV2& input,
                         const double w0, const double w1,
                         const int depth) {
  CellAttempt attempt;
  state.cell_samples = 0U;
  if (!phase_offset_core::certifiedPathCellV2IsComplete(input) ||
      input.w0 != w0 || input.w1 != w1) {
    attempt.failure = TubeCertificateFailureReason::MALFORMED_PATH_CELL;
    return attempt;
  }
  // Certified path cells carry an enclosing anchor interval; the rounded
  // anchor may coincide with an endpoint for adjacent-ULP cells.  Core
  // completeness already verifies finiteness and anchor_w in [w0,w1].
  if (!Finite(input.anchor_w) || input.anchor_w < w0 || input.anchor_w > w1) {
    attempt.failure = TubeCertificateFailureReason::MALFORMED_PATH_CELL;
    return attempt;
  }
  const phase_offset_core::CertifiedPathCellV2& cell = input;
  TubeProofCellV2 proof;
  proof.w0 = w0;
  proof.w1 = w1;
  proof.depth = depth;
  proof.segment_identity = cell.segment_identity;
  proof.proof_identity = cell.proof_identity;
  proof.path_cell = cell;
  double q_min = 0.0;
  double p_bound = 0.0;
  double a_xy = 0.0;
  double q_regular = 0.0;
  double cap = 0.0;
  if (!BuildRegularity(cell, state.config, q_min, p_bound, a_xy,
                       q_regular, cap)) {
    attempt.failure = TubeCertificateFailureReason::REGULARITY;
    return attempt;
  }
  double motion_cover = 0.0;
  if (!MotionCover(p_bound, q_regular, state.config.nominal_half_width,
                   w0, w1, motion_cover)) {
    attempt.failure = TubeCertificateFailureReason::MOTION_COVER;
    return attempt;
  }
  phase_offset_core::Binary64VectorInterval normal;
  if (!ComputeHorizontalNormal(cell, normal, q_min)) {
    attempt.failure = TubeCertificateFailureReason::NORMAL_FLOOR;
    return attempt;
  }
  proof.q_min = q_min;
  proof.p_bound = p_bound;
  proof.a_xy = a_xy;
  proof.q_regular = q_regular;
  proof.regularity_cap = cap;
  proof.motion_cover = motion_cover;

  phase_offset_core::Binary64VectorInterval zero_position;
  if (!VectorIntervalAtOffset(cell, normal, 0.0, zero_position)) {
    attempt.failure = TubeCertificateFailureReason::NONFINITE_INTERVAL;
    return attempt;
  }
  Eigen::Vector3d zero_witness;
  double zero_error = 0.0;
  if (!MidpointWitness(zero_position, zero_witness, zero_error)) {
    attempt.failure = TubeCertificateFailureReason::NONFINITE_INTERVAL;
    return attempt;
  }
  const Interval zero_required_interval = Add(
      Point(state.config.epsilon),
      Add(Point(motion_cover), Point(zero_error)));
  if (!zero_required_interval.valid || zero_required_interval.upper < 0.0) {
    attempt.failure = TubeCertificateFailureReason::INTERVAL_OVERFLOW;
    return attempt;
  }
  TubeSupportFootprintV2 zero_support;
  TubeCertificateFailureReason failure = TubeCertificateFailureReason::NONE;
  if (!QueryFreeBall(state, zero_witness, zero_required_interval.upper, depth,
                     zero_support,
                     failure)) {
    attempt.failure = failure == TubeCertificateFailureReason::NONE
        ? TubeCertificateFailureReason::ZERO_ANCHOR : failure;
    attempt.failure_delta = 0.0;
    return attempt;
  }
  proof.supports.push_back(zero_support);
  proof.zero_anchor_certified = true;
  attempt.anchor_safe = true;

  const double resolution_half = state.config.snapshot_resolution * 0.5;
  const double transverse_step = std::min(
      state.config.nominal_half_width,
      std::min(state.config.ray_step, resolution_half));
  if (!Finite(transverse_step) || !(transverse_step > 0.0)) {
    attempt.failure = TubeCertificateFailureReason::WITNESS_FAILURE;
    return attempt;
  }

  double positive_extent = 0.0;
  for (std::size_t k = 1U; positive_extent < cap; ++k) {
    if (k > state.config.budgets.max_samples_per_cell ||
        !Finite(static_cast<double>(k))) {
      failure = TubeCertificateFailureReason::SAMPLE_BUDGET;
      state.stats.sample_budget_reached = true;
      break;
    }
    double indexed_endpoint = 0.0;
    if (!ProductFloor(static_cast<double>(k), transverse_step,
                      indexed_endpoint)) {
      failure = TubeCertificateFailureReason::INTERVAL_OVERFLOW;
      break;
    }
    const double d0 = positive_extent;
    const double d1 = std::min(cap, indexed_endpoint);
    if (!(d1 > d0) || !Finite(d1)) {
      failure = TubeCertificateFailureReason::WITNESS_FAILURE;
      break;
    }
    double accepted = d0;
    TubeSupportFootprintV2 support;
    if (!QuerySegment(state, cell, normal, motion_cover, state.config.epsilon,
                      d0, d1, depth, support, accepted, failure)) {
      break;
    }
    proof.supports.push_back(support);
    positive_extent = accepted;
  }

  double negative_extent = 0.0;
  for (std::size_t k = 1U; negative_extent < cap; ++k) {
    if (k > state.config.budgets.max_samples_per_cell ||
        !Finite(static_cast<double>(k))) {
      failure = TubeCertificateFailureReason::SAMPLE_BUDGET;
      state.stats.sample_budget_reached = true;
      break;
    }
    double indexed_endpoint = 0.0;
    if (!ProductFloor(static_cast<double>(k), transverse_step,
                      indexed_endpoint)) {
      failure = TubeCertificateFailureReason::INTERVAL_OVERFLOW;
      break;
    }
    const double d0 = negative_extent;
    const double d1 = std::min(cap, indexed_endpoint);
    if (!(d1 > d0) || !Finite(d1)) {
      failure = TubeCertificateFailureReason::WITNESS_FAILURE;
      break;
    }
    double accepted = d0;
    TubeSupportFootprintV2 support;
    const double signed_d0 = -d0;
    const double signed_d1 = -d1;
    if (!QuerySegment(state, cell, normal, motion_cover, state.config.epsilon,
                      signed_d1, signed_d0, depth, support, accepted, failure)) {
      break;
    }
    proof.supports.push_back(support);
    negative_extent = d1;
  }

  proof.lower = -negative_extent;
  proof.upper = positive_extent;
  proof.complete = true;
  proof.valid = BoundsContainZero(proof.lower, proof.upper);
  if (!proof.valid) {
    attempt.failure = failure == TubeCertificateFailureReason::NONE
        ? TubeCertificateFailureReason::WITNESS_FAILURE : failure;
    return attempt;
  }
  attempt.proof = proof;
  attempt.nonzero = positive_extent > 0.0 || negative_extent > 0.0;
  attempt.valid = true;
  return attempt;
}

bool IntersectIntervals(const double lhs_lower, const double lhs_upper,
                        const double rhs_lower, const double rhs_upper,
                        double& lower, double& upper) {
  lower = std::max(lhs_lower, rhs_lower);
  upper = std::min(lhs_upper, rhs_upper);
  return BoundsContainZero(lower, upper) && lower <= upper;
}

bool BuildInitialPoints(const TubeBuildInputV2& input,
                        const TubeCertificateConfigV2& config,
                        std::vector<double>& points) {
  if (input.producer_breakpoints.size() >
          std::numeric_limits<std::size_t>::max() -
              input.sample_grid.size() ||
      input.producer_breakpoints.size() + input.sample_grid.size() >
          std::numeric_limits<std::size_t>::max() - input.path_cells.size()) {
    return false;
  }
  const std::size_t structural_count = input.producer_breakpoints.size() +
      input.sample_grid.size() + input.path_cells.size();
  const std::size_t raw_slack = 8U;
  if (structural_count > std::numeric_limits<std::size_t>::max() - raw_slack ||
      config.budgets.max_cells >
          std::numeric_limits<std::size_t>::max() - raw_slack ||
      structural_count > config.budgets.max_cells + raw_slack) {
    return false;
  }
  const std::size_t raw_limit = config.budgets.max_cells + raw_slack;
  points.clear();
  points.reserve(std::min(raw_limit, structural_count + raw_slack));
  const auto add_bounded = [&points, raw_limit](const double value) {
    if (points.size() >= raw_limit) return false;
    points.push_back(value);
    return true;
  };
  if (!add_bounded(input.requested_start) ||
      !add_bounded(input.requested_end) ||
      !add_bounded(input.anchor_w)) {
    return false;
  }
  for (const double value : input.producer_breakpoints) {
    if (!Finite(value) || value < input.path_key.domain_start ||
        value > input.path_key.domain_end) return false;
    if (value >= input.requested_start && value <= input.requested_end &&
        !add_bounded(value)) return false;
  }
  if (input.producer_breakpoint_query) {
    if (!input.query_owner) return false;
    std::vector<double> producer_points;
    if (!input.producer_breakpoint_query(producer_points)) return false;
    if (producer_points.size() > raw_limit - points.size()) return false;
    for (const double value : producer_points) {
      if (!Finite(value) || value < input.path_key.domain_start ||
          value > input.path_key.domain_end) return false;
      if (value >= input.requested_start && value <= input.requested_end &&
          !add_bounded(value)) return false;
    }
  }
  for (const double value : input.sample_grid) {
    if (!Finite(value) || value < input.requested_start ||
        value > input.requested_end || !add_bounded(value)) return false;
  }
  for (const phase_offset_core::CertifiedPathCellV2& cell : input.path_cells) {
    if (!Finite(cell.w0) || cell.w0 < input.path_key.domain_start ||
        cell.w0 > input.path_key.domain_end ||
        !Finite(cell.w1) || cell.w1 < input.path_key.domain_start ||
        cell.w1 > input.path_key.domain_end) return false;
    if (cell.w0 >= input.requested_start && cell.w0 <= input.requested_end &&
        !add_bounded(cell.w0)) return false;
    if (cell.w1 >= input.requested_start && cell.w1 <= input.requested_end &&
        !add_bounded(cell.w1)) return false;
  }
  if (config.sample_step_w > 0.0) {
    for (std::size_t k = 1U; ; ++k) {
      if (k > config.budgets.max_cells || !Finite(static_cast<double>(k))) {
        return false;
      }
      const double offset = static_cast<double>(k) * config.sample_step_w;
      const double value = input.requested_start + offset;
      if (!Finite(offset) || !Finite(value)) return false;
      if (!(value > input.requested_start)) return false;
      if (value >= input.requested_end) break;
      if (!add_bounded(value)) return false;
    }
  }
  std::sort(points.begin(), points.end());
  points.erase(std::unique(points.begin(), points.end()), points.end());
  // max_cells bounds intervals/work items; the sorted partition may contain
  // one additional endpoint.  Structural duplicates are removed before this
  // check so they do not consume the unique-cell budget.
  if (points.size() > config.budgets.max_cells + 1U) return false;
  return points.size() >= 2U && points.front() == input.requested_start &&
      points.back() == input.requested_end;
}

bool FindSourceCell(const CertifiedPathCellsV2& cells, const double w0,
                    const double w1, std::size_t& index) {
  bool found = false;
  for (std::size_t i = 0U; i < cells.size(); ++i) {
    const phase_offset_core::CertifiedPathCellV2& cell = cells[i];
    if (cell.w0 <= w0 && cell.w1 >= w1) {
      if (found && cells[index].segment_identity != cell.segment_identity) {
        return false;
      }
      index = i;
      found = true;
    }
  }
  return found;
}

bool ApplicabilityMetadataComplete(const TubeBuildInputV2& input) {
  if (input.applicability_assumptions.empty()) return false;
  const TubeMapCaptureKey& map = input.map_capture_key;
  if (map.support_expiry_timeless) {
    return map.support_expiry_ticks == 0U &&
        input.applicability_deadline_timeless &&
        input.applicability_deadline_ticks == 0U;
  }
  return !input.applicability_deadline_timeless &&
      input.applicability_deadline_ticks >= map.accepted_time_ticks &&
      input.applicability_deadline_ticks <= map.support_expiry_ticks;
}

struct AcceptedCell {
  TubeProofCellV2 proof;
};

bool BuildPwl(const std::vector<AcceptedCell>& accepted,
              TubeProfileV2& profile) {
  if (accepted.empty()) return false;
  std::unordered_set<std::uint64_t> cell_ids;
  cell_ids.reserve(accepted.size());
  for (const AcceptedCell& cell : accepted) {
    if (cell.proof.cell_id == 0U ||
        !cell_ids.insert(cell.proof.cell_id).second) {
      return false;
    }
  }
  profile.knots.clear();
  profile.knots.reserve(accepted.size() + 1U);
  for (std::size_t i = 0U; i <= accepted.size(); ++i) {
    TubePwlKnotV2 knot;
    if (i == 0U) {
      knot.w = accepted.front().proof.w0;
      knot.lower = accepted.front().proof.lower;
      knot.upper = accepted.front().proof.upper;
      knot.right_cell_id = accepted.front().proof.cell_id;
    } else if (i == accepted.size()) {
      knot.w = accepted.back().proof.w1;
      knot.lower = accepted.back().proof.lower;
      knot.upper = accepted.back().proof.upper;
      knot.left_cell_id = accepted.back().proof.cell_id;
    } else {
      const TubeProofCellV2& left = accepted[i - 1U].proof;
      const TubeProofCellV2& right = accepted[i].proof;
      if (left.w1 != right.w0 ||
          !IntersectIntervals(left.lower, left.upper, right.lower,
                              right.upper, knot.lower, knot.upper)) {
        return false;
      }
      knot.w = left.w1;
      knot.left_cell_id = left.cell_id;
      knot.right_cell_id = right.cell_id;
    }
    if (!Finite(knot.w) || !BoundsContainZero(knot.lower, knot.upper)) {
      return false;
    }
    knot.valid = true;
    profile.knots.push_back(knot);
  }
  for (std::size_t i = 1U; i < profile.knots.size(); ++i) {
    TubePwlKnotV2& previous = profile.knots[i - 1U];
    TubePwlKnotV2& current = profile.knots[i];
    const Interval span = Subtract(Point(current.w), Point(previous.w));
    if (!span.valid || !(span.lower > 0.0)) return false;
    const Interval lower_numerator =
        Subtract(Point(current.lower), Point(previous.lower));
    const Interval upper_numerator =
        Subtract(Point(current.upper), Point(previous.upper));
    const Interval lower_slope_interval =
        DividePositive(lower_numerator, span);
    const Interval upper_slope_interval =
        DividePositive(upper_numerator, span);
    if (!lower_slope_interval.valid || !upper_slope_interval.valid) return false;
    const TubeDirectedRatioV2 lower_ratio{
        lower_slope_interval.lower, lower_slope_interval.upper, true};
    const TubeDirectedRatioV2 upper_ratio{
        upper_slope_interval.lower, upper_slope_interval.upper, true};
    previous.right_lower_slope_interval = lower_ratio;
    previous.right_upper_slope_interval = upper_ratio;
    current.left_lower_slope_interval = lower_ratio;
    current.left_upper_slope_interval = upper_ratio;
    previous.right_lower_slope = lower_slope_interval.upper;
    previous.right_upper_slope = upper_slope_interval.lower;
    current.left_lower_slope = previous.right_lower_slope;
    current.left_upper_slope = previous.right_upper_slope;
    if (!Finite(previous.right_lower_slope) ||
        !Finite(previous.right_upper_slope) || !lower_ratio.valid ||
        !upper_ratio.valid) return false;
  }
  profile.contains_zero_everywhere = true;
  for (const AcceptedCell& cell : accepted) {
    profile.contains_zero_everywhere = profile.contains_zero_everywhere &&
        BoundsContainZero(cell.proof.lower, cell.proof.upper);
  }
  // Capability is a property of the actually assembled immutable PWL, not
  // of an individual cell before adjacent-knot intersections.  A broad
  // interior cell can collapse to zero at both seams (for example
  // {0},[0,.2],{0}); report ZERO_ONLY unless some stored knot still carries
  // nonzero width.
  profile.nonzero_capacity = false;
  for (const TubePwlKnotV2& knot : profile.knots) {
    profile.nonzero_capacity = profile.nonzero_capacity ||
        knot.lower < 0.0 || knot.upper > 0.0;
  }
  return profile.contains_zero_everywhere;
}

void CopyCounters(const TubeCertificateBuildStatsV2& source,
                  TubeProfileV2::BuildCounters& destination) {
  destination.path_cell_query_count = source.path_cell_query_count;
  destination.failed_path_cell_query_count = source.failed_path_cell_query_count;
  destination.query_count = source.query_count;
  destination.failed_query_count = source.failed_query_count;
  destination.child_query_count = source.child_query_count;
  destination.cell_count = source.cell_count;
  destination.scheduled_cell_count = source.scheduled_cell_count;
  destination.accepted_cell_count = source.accepted_cell_count;
  destination.witness_count = source.witness_count;
  destination.max_depth_observed = source.max_depth_observed;
  destination.query_budget_reached = source.query_budget_reached;
  destination.cell_budget_reached = source.cell_budget_reached;
  destination.witness_budget_reached = source.witness_budget_reached;
  destination.sample_budget_reached = source.sample_budget_reached;
}

struct WorkItem {
  double w0 = 0.0;
  double w1 = 0.0;
  std::size_t source_index = 0U;
  int depth = 0;
};

bool SplitItem(const WorkItem& item, WorkItem& left, WorkItem& right) {
  double midpoint = 0.0;
  if (!StrictSplitMidpoint(item.w0, item.w1, midpoint)) return false;
  left = item;
  left.w1 = midpoint;
  left.depth = item.depth + 1;
  right = item;
  right.w0 = midpoint;
  right.depth = item.depth + 1;
  return left.w1 > left.w0 && right.w1 > right.w0;
}

}  // namespace

bool TubePathKey::complete() const {
  return execution_generation != 0U && path_instance_id != 0U &&
      path_revision != 0U && frame_revision != 0U &&
      frame_convention_id != 0U && !frame_convention.empty() &&
      frame_convention == phase_offset_core::kWorldHorizontalCrossProductProvenance &&
      (phase_orientation == 1 ||
                                    phase_orientation == -1) &&
      Finite(domain_start) && Finite(domain_end) && domain_end > domain_start;
}

bool TubePathKey::operator==(const TubePathKey& other) const {
  return execution_generation == other.execution_generation &&
      path_instance_id == other.path_instance_id &&
      path_revision == other.path_revision && frame_revision == other.frame_revision &&
      frame_convention_id == other.frame_convention_id &&
      frame_convention == other.frame_convention &&
      phase_orientation == other.phase_orientation &&
      domain_start == other.domain_start && domain_end == other.domain_end;
}

bool TubeConfigurationKey::complete() const {
  return configuration_id != 0U && Finite(epsilon) && epsilon > 0.0 &&
      Finite(nominal_half_width) && nominal_half_width > 0.0 &&
      Finite(ray_step) && ray_step > 0.0 &&
      Finite(snapshot_resolution) && snapshot_resolution > 0.0 &&
      Finite(minimum_reference_speed) && minimum_reference_speed > 0.0;
}

bool TubeConfigurationKey::operator==(const TubeConfigurationKey& other) const {
  return configuration_id == other.configuration_id &&
      epsilon == other.epsilon && nominal_half_width == other.nominal_half_width &&
      ray_step == other.ray_step &&
      snapshot_resolution == other.snapshot_resolution &&
      minimum_reference_speed == other.minimum_reference_speed;
}

bool TubeMapCaptureKey::complete() const {
  return map_instance_id != 0U && state_id != 0U && accepted_sequence != 0U &&
      configuration_generation != 0U && configuration_id != 0U &&
      frame_provenance_id != 0U &&
      !frame_provenance.empty() &&
      support_provenance_id != 0U &&
      accepted_time_ticks != 0U && (support_expiry_timeless ||
                                    support_expiry_ticks >= accepted_time_ticks) &&
      Finite(support_halo) && support_halo >= 0.0 && halo_reconciled &&
      grid_min_index_x <= grid_max_index_x &&
      grid_min_index_y <= grid_max_index_y &&
      grid_min_index_z <= grid_max_index_z &&
      grid_native_origin.allFinite() &&
      (grid_voxel_resolution.array() > 0.0).all() &&
      grid_voxel_resolution.allFinite() && complete_support;
}

bool TubeMapCaptureKey::operator==(const TubeMapCaptureKey& other) const {
  return map_instance_id == other.map_instance_id &&
      state_id == other.state_id && accepted_sequence == other.accepted_sequence &&
      configuration_generation == other.configuration_generation &&
      configuration_id == other.configuration_id &&
      frame_provenance_id == other.frame_provenance_id &&
      frame_provenance == other.frame_provenance &&
      support_provenance_id == other.support_provenance_id &&
      accepted_time_ticks == other.accepted_time_ticks &&
      support_expiry_ticks == other.support_expiry_ticks &&
      support_expiry_timeless == other.support_expiry_timeless &&
      support_halo == other.support_halo &&
      halo_reconciled == other.halo_reconciled &&
      grid_min_index_x == other.grid_min_index_x &&
      grid_min_index_y == other.grid_min_index_y &&
      grid_min_index_z == other.grid_min_index_z &&
      grid_max_index_x == other.grid_max_index_x &&
      grid_max_index_y == other.grid_max_index_y &&
      grid_max_index_z == other.grid_max_index_z &&
      (grid_native_origin.array() == other.grid_native_origin.array()).all() &&
      (grid_voxel_resolution.array() ==
           other.grid_voxel_resolution.array()).all() &&
      grid_source_offset_x == other.grid_source_offset_x &&
      grid_source_offset_y == other.grid_source_offset_y &&
      grid_source_offset_z == other.grid_source_offset_z &&
      grid_native_index == other.grid_native_index &&
      complete_support == other.complete_support;
}

bool TubeCertificateConfigV2::complete() const {
  return configuration_id != 0U && Finite(epsilon) && epsilon > 0.0 &&
      Finite(nominal_half_width) && nominal_half_width > 0.0 &&
      Finite(ray_step) && ray_step > 0.0 && Finite(snapshot_resolution) &&
      snapshot_resolution > 0.0 && Finite(minimum_reference_speed) &&
      minimum_reference_speed > 0.0 && Finite(sample_step_w) &&
      sample_step_w > 0.0 && budgets.max_w_depth >= 0 &&
      budgets.max_w_depth <= 12 && budgets.max_queries > 0U &&
      budgets.max_queries <= 250000U && budgets.max_cells > 0U &&
      budgets.max_cells <= 250000U && budgets.max_samples_per_cell > 0U &&
      budgets.max_samples_per_cell <= 100000U && budgets.max_witnesses > 0U &&
      budgets.max_witnesses <= budgets.max_queries &&
      // Every scheduled build cell must have room in the captured query
      // budget even before path/map callback costs are accounted for.
      budgets.max_cells <= budgets.max_queries;
}

TubeConfigurationKey TubeCertificateConfigV2::key() const {
  TubeConfigurationKey result;
  result.configuration_id = configuration_id;
  result.epsilon = epsilon;
  result.nominal_half_width = nominal_half_width;
  result.ray_step = ray_step;
  result.snapshot_resolution = snapshot_resolution;
  result.minimum_reference_speed = minimum_reference_speed;
  return result;
}

bool TubeBuildInputV2::complete() const {
  if (request_id == 0U || !path_key.complete() ||
      !configuration_key.complete() || !map_capture_key.complete() ||
      !Finite(requested_start) || !Finite(requested_end) ||
      !(requested_end > requested_start) || !Finite(anchor_w) ||
      anchor_w < requested_start || anchor_w > requested_end ||
      (path_cells.empty() && !path_cell_query) || !free_ball_query ||
      !path_owner || !capture_owner || !query_owner ||
      (applicability_deadline_ticks == 0U && !applicability_deadline_timeless) ||
      !ApplicabilityMetadataComplete(*this)) {
    return false;
  }
  // S1-B captures an isotropic voxel grid under the same scalar resolution
  // named by the immutable configuration.  Reject stale or anisotropic map
  // metadata before any witness query can consume it.
  const Eigen::Vector3d expected_resolution =
      Eigen::Vector3d::Constant(configuration_key.snapshot_resolution);
  if ((map_capture_key.grid_voxel_resolution.array() !=
       expected_resolution.array()).any()) {
    return false;
  }
  if (path_cell_query && !path_owner) return false;
  if (producer_breakpoint_query && !query_owner) return false;
  for (const phase_offset_core::CertifiedPathCellV2& cell : path_cells) {
    if (!phase_offset_core::certifiedPathCellV2IsComplete(cell) ||
        cell.w0 < path_key.domain_start || cell.w1 > path_key.domain_end) return false;
  }
  return true;
}

bool TubeProfileV2::evaluate(const double w, double& lower, double& upper) const {
  if (!SupportedFloatingPointEnvironment() || !valid || knots.size() < 2U ||
      !Finite(w) || w < certified_start ||
      w > certified_end) return false;
  if (w == knots.front().w) {
    lower = knots.front().lower;
    upper = knots.front().upper;
    return true;
  }
  if (w == knots.back().w) {
    lower = knots.back().lower;
    upper = knots.back().upper;
    return true;
  }
  const auto it = std::upper_bound(
      knots.begin(), knots.end(), w,
      [](const double value, const TubePwlKnotV2& knot) {
        return value < knot.w;
      });
  if (it == knots.begin() || it == knots.end()) return false;
  const TubePwlKnotV2& left = *(it - 1);
  const TubePwlKnotV2& right = *it;
  const Interval span = Subtract(Point(right.w), Point(left.w));
  const Interval numerator = Subtract(Point(w), Point(left.w));
  const Interval raw_t = DividePositive(numerator, span);
  const Interval t = raw_t.valid
      ? Bounds(std::max(0.0, raw_t.lower), std::min(1.0, raw_t.upper))
      : InvalidInterval();
  if (!t.valid) return false;
  const Interval lower_difference =
      Subtract(Point(right.lower), Point(left.lower));
  const Interval upper_difference =
      Subtract(Point(right.upper), Point(left.upper));
  const Interval lower_value = Add(Point(left.lower),
                                   Multiply(lower_difference, t));
  const Interval upper_value = Add(Point(left.upper),
                                   Multiply(upper_difference, t));
  if (!lower_value.valid || !upper_value.valid ||
      !Finite(lower_value.upper) || !Finite(upper_value.lower) ||
      lower_value.upper > upper_value.lower) return false;
  // The endpoint selection is inward: lower moves toward zero and upper moves
  // toward zero.  Every arithmetic elementary operation above is enclosed;
  // no final post-hoc ULP is used to mask a lost intermediate.
  lower = lower_value.upper;
  upper = upper_value.lower;
  return Finite(lower) && Finite(upper) && lower <= upper;
}

bool TubeProfileV2::contains(const double w, const double delta) const {
  double lower = 0.0;
  double upper = 0.0;
  return Finite(delta) && evaluate(w, lower, upper) && delta >= lower &&
      delta <= upper;
}

bool TubeProfileV2::structurallyValid() const {
  if (!valid || !complete || !path_key.complete() ||
      !configuration_key.complete() || !map_capture_key.complete() ||
      !path_owner || !capture_owner || !query_owner ||
      (applicability_deadline_ticks == 0U && !applicability_deadline_timeless) ||
      applicability_assumptions.empty() ||
      (map_capture_key.support_expiry_timeless
           ? (map_capture_key.support_expiry_ticks != 0U ||
              !applicability_deadline_timeless ||
              applicability_deadline_ticks != 0U)
           : (applicability_deadline_timeless ||
              applicability_deadline_ticks < map_capture_key.accepted_time_ticks ||
              applicability_deadline_ticks > map_capture_key.support_expiry_ticks)) ||
      knots.size() < 2U || cells.empty() ||
      certified_start != knots.front().w || certified_end != knots.back().w ||
      !(certified_end > certified_start)) return false;
  if (knots.front().left_cell_id != 0U ||
      knots.back().right_cell_id != 0U) return false;
  for (std::size_t i = 0U; i < knots.size(); ++i) {
    const TubePwlKnotV2& knot = knots[i];
    if (!knot.valid || !Finite(knot.w) ||
        (i > 0U && !(knot.w > knots[i - 1U].w)) ||
        !BoundsContainZero(knot.lower, knot.upper)) return false;
    if (i > 0U && (!knot.left_lower_slope_interval.valid ||
                   !knot.left_upper_slope_interval.valid)) return false;
    if (i + 1U < knots.size() &&
        (!knot.right_lower_slope_interval.valid ||
         !knot.right_upper_slope_interval.valid)) return false;
  }
  if (cells.size() + 1U != knots.size()) return false;
  std::unordered_set<std::uint64_t> cell_ids;
  cell_ids.reserve(cells.size());
  for (std::size_t i = 0U; i < cells.size(); ++i) {
    const TubeProofCellV2& cell = cells[i];
    if (!cell.valid || !cell.complete || cell.w0 != knots[i].w ||
        cell.w1 != knots[i + 1U].w || !BoundsContainZero(cell.lower, cell.upper)) {
      return false;
    }
    if (cell.cell_id == 0U || !cell_ids.insert(cell.cell_id).second ||
        knots[i].right_cell_id != cell.cell_id ||
        knots[i + 1U].left_cell_id != cell.cell_id) {
      return false;
    }
    if (cell.path_cell.path_revision != path_key.path_revision ||
        cell.path_cell.frame_revision != path_key.frame_revision ||
        cell.path_cell.w0 != cell.w0 || cell.path_cell.w1 != cell.w1 ||
        !phase_offset_core::certifiedPathCellV2IsComplete(cell.path_cell)) {
      return false;
    }
    if (i > 0U && cells[i - 1U].w1 != cell.w0) return false;
    if (knots[i].lower < cell.lower || knots[i].upper > cell.upper ||
        knots[i + 1U].lower < cell.lower ||
        knots[i + 1U].upper > cell.upper) return false;
  }
  return true;
}

const char* tubeCertificateFailureReasonName(
    const TubeCertificateFailureReason reason) {
  switch (reason) {
    case TubeCertificateFailureReason::NONE: return "NONE";
    case TubeCertificateFailureReason::INVALID_INPUT: return "INVALID_INPUT";
    case TubeCertificateFailureReason::UNAVAILABLE_PRODUCER: return "UNAVAILABLE_PRODUCER";
    case TubeCertificateFailureReason::MALFORMED_PATH_CELL: return "MALFORMED_PATH_CELL";
    case TubeCertificateFailureReason::PATH_KEY_MISMATCH: return "PATH_KEY_MISMATCH";
    case TubeCertificateFailureReason::CONFIGURATION_MISMATCH: return "CONFIGURATION_MISMATCH";
    case TubeCertificateFailureReason::MAP_CAPTURE_MISMATCH: return "MAP_CAPTURE_MISMATCH";
    case TubeCertificateFailureReason::UNSUPPORTED_FP_MODE: return "UNSUPPORTED_FP_MODE";
    case TubeCertificateFailureReason::NONFINITE_INTERVAL: return "NONFINITE_INTERVAL";
    case TubeCertificateFailureReason::INTERVAL_OVERFLOW: return "INTERVAL_OVERFLOW";
    case TubeCertificateFailureReason::DENOMINATOR_CROSSES_ZERO: return "DENOMINATOR_CROSSES_ZERO";
    case TubeCertificateFailureReason::NORMAL_FLOOR: return "NORMAL_FLOOR";
    case TubeCertificateFailureReason::REGULARITY: return "REGULARITY";
    case TubeCertificateFailureReason::MOTION_COVER: return "MOTION_COVER";
    case TubeCertificateFailureReason::ZERO_ANCHOR: return "ZERO_ANCHOR";
    case TubeCertificateFailureReason::CLEARANCE_UNAVAILABLE: return "CLEARANCE_UNAVAILABLE";
    case TubeCertificateFailureReason::CLEARANCE_OUT_OF_MAP: return "CLEARANCE_OUT_OF_MAP";
    case TubeCertificateFailureReason::CLEARANCE_UNKNOWN: return "CLEARANCE_UNKNOWN";
    case TubeCertificateFailureReason::CLEARANCE_OCCUPIED: return "CLEARANCE_OCCUPIED";
    case TubeCertificateFailureReason::CLEARANCE_UNCERTIFIED: return "CLEARANCE_UNCERTIFIED";
    case TubeCertificateFailureReason::CLEARANCE_INSUFFICIENT: return "CLEARANCE_INSUFFICIENT";
    case TubeCertificateFailureReason::STRUCTURAL_GAP: return "STRUCTURAL_GAP";
    case TubeCertificateFailureReason::WITNESS_FAILURE: return "WITNESS_FAILURE";
    case TubeCertificateFailureReason::SUBDIVISION_DEPTH: return "SUBDIVISION_DEPTH";
    case TubeCertificateFailureReason::QUERY_BUDGET: return "QUERY_BUDGET";
    case TubeCertificateFailureReason::CELL_BUDGET: return "CELL_BUDGET";
    case TubeCertificateFailureReason::WITNESS_BUDGET: return "WITNESS_BUDGET";
    case TubeCertificateFailureReason::SAMPLE_BUDGET: return "SAMPLE_BUDGET";
    case TubeCertificateFailureReason::COMPONENT_GAP: return "COMPONENT_GAP";
    case TubeCertificateFailureReason::PWL_MALFORMED: return "PWL_MALFORMED";
  }
  return "UNKNOWN";
}

const char* tubeCertificateTruncationName(const TubeCertificateTruncation outcome) {
  switch (outcome) {
    case TubeCertificateTruncation::NONE: return "NONE";
    case TubeCertificateTruncation::PREFIX: return "PREFIX";
    case TubeCertificateTruncation::SUFFIX: return "SUFFIX";
    case TubeCertificateTruncation::PREFIX_AND_SUFFIX: return "PREFIX_AND_SUFFIX";
    case TubeCertificateTruncation::ANCHOR_EXCLUDED: return "ANCHOR_EXCLUDED";
    case TubeCertificateTruncation::NO_CERTIFIED_COMPONENT: return "NO_CERTIFIED_COMPONENT";
    case TubeCertificateTruncation::BUDGET: return "BUDGET";
    case TubeCertificateTruncation::DEPTH: return "DEPTH";
  }
  return "UNKNOWN";
}

TubeCertificateBuilderV2::TubeCertificateBuilderV2(
    const TubeCertificateConfigV2& config)
    : config_(config) {}

bool TubeCertificateBuilderV2::configurationValid() const {
  return config_.complete();
}

bool TubeCertificateBuilderV2::build(
    const TubeBuildInputV2& input,
    TubeCertificateBuildResultV2& result) const {
  result = TubeCertificateBuildResultV2();
  TubeProfileV2& profile = result.profile;
  profile.request_id = input.request_id;
  profile.path_key = input.path_key;
  profile.configuration_key = input.configuration_key;
  profile.map_capture_key = input.map_capture_key;
  profile.requested_start = input.requested_start;
  profile.requested_end = input.requested_end;
  profile.anchor_w = input.anchor_w;
  profile.path_owner = input.path_owner;
  profile.capture_owner = input.capture_owner;
  profile.query_owner = input.query_owner;
  profile.applicability_assumptions = input.applicability_assumptions;
  profile.applicability_deadline_ticks = input.applicability_deadline_ticks;
  profile.applicability_deadline_timeless = input.applicability_deadline_timeless;

  auto fail = [&profile](const TubeCertificateFailureReason reason,
                         const double w0 = 0.0, const double w1 = 0.0,
                         const int depth = 0, const std::string& detail = "") {
    if (profile.failure_reason != TubeCertificateFailureReason::NONE) return;
    profile.failure_reason = reason;
    profile.failure.reason = reason;
    profile.failure.w0 = w0;
    profile.failure.w1 = w1;
    profile.failure.depth = depth;
    profile.failure.detail = detail;
  };

  if (!configurationValid()) {
    fail(TubeCertificateFailureReason::INVALID_INPUT, 0.0, 0.0, 0,
         "invalid V2 certificate configuration");
    return false;
  }
  if (!SupportedFloatingPointEnvironment()) {
    fail(TubeCertificateFailureReason::UNSUPPORTED_FP_MODE);
    return false;
  }
  if (!input.complete() || !(input.configuration_key == config_.key()) ||
      input.path_key.domain_start > input.requested_start ||
      input.path_key.domain_end < input.requested_end) {
    fail(TubeCertificateFailureReason::INVALID_INPUT);
    return false;
  }

  std::vector<double> points;
  if (!BuildInitialPoints(input, config_, points)) {
    fail(TubeCertificateFailureReason::STRUCTURAL_GAP);
    return false;
  }

  TubeCertificateBuildStatsV2 stats;
  const std::size_t effective_cell_budget = std::min(
      config_.budgets.max_cells, config_.budgets.max_queries);
  std::vector<WorkItem> initial;
  initial.reserve(points.size() - 1U);
  for (std::size_t i = 1U; i < points.size(); ++i) {
    WorkItem item;
    item.w0 = points[i - 1U];
    item.w1 = points[i];
    if (!FindSourceCell(input.path_cells, item.w0, item.w1,
                        item.source_index)) {
      if (!input.path_cell_query) {
        fail(TubeCertificateFailureReason::STRUCTURAL_GAP, item.w0, item.w1);
        return false;
      }
      item.source_index = 0U;
    }
    initial.push_back(item);
  }

  // Initial intervals are already scheduled work cells.  Endpoints are not
  // cells, hence BuildInitialPoints permits max_cells+1 partition points;
  // enforce the unique-cell budget before any producer callback runs.
  stats.scheduled_cell_count = initial.size();
  if (initial.size() > effective_cell_budget) {
    stats.cell_budget_reached = true;
    fail(TubeCertificateFailureReason::CELL_BUDGET,
         initial[effective_cell_budget].w0,
         initial[effective_cell_budget].w1);
    profile.truncation = TubeCertificateTruncation::NO_CERTIFIED_COMPONENT;
    profile.failure.truncation = profile.truncation;
    profile.failure.query_count = 0U;
    CopyCounters(stats, profile.counters);
    result.stats = stats;
    return false;
  }

  TubeFailureAttribution failure;
  QueryState state{input, config_, stats, failure, 0U};
  std::vector<AcceptedCell> accepted;
  accepted.reserve(initial.size());
  bool anchor_component_closed = false;
  // Work is visited in increasing w (left child first).  Once a terminal
  // unproved cell separates the anchored component from the remaining work,
  // none of that work can contribute to this profile.  This also holds if
  // the anchor itself is unproved: later cells cannot recover it across a
  // gap.  A failed cell ENDING exactly at the anchor must not stop work,
  // because the next closed cell may still certify that shared endpoint.
  const auto close_anchor_component = [&](const WorkItem& item) {
    if (item.w1 > input.anchor_w) {
      anchor_component_closed = true;
    }
  };

  std::function<void(const WorkItem&)> process;
  process = [&](const WorkItem& item) {
    if (anchor_component_closed) return;
    if (stats.cell_budget_reached || stats.query_budget_reached ||
        stats.witness_budget_reached || stats.sample_budget_reached) return;
    if (stats.cell_count >= effective_cell_budget) {
      stats.cell_budget_reached = true;
      fail(TubeCertificateFailureReason::CELL_BUDGET, item.w0, item.w1,
           item.depth);
      return;
    }
    ++stats.cell_count;
    stats.max_depth_observed = std::max(stats.max_depth_observed, item.depth);

    phase_offset_core::CertifiedPathCellV2 fallback;
    if (!input.path_cells.empty()) fallback = input.path_cells[item.source_index];
    phase_offset_core::CertifiedPathCellV2 cell;
    if (!PreparePathCell(state, fallback, item.w0, item.w1, cell)) {
      if (item.depth < config_.budgets.max_w_depth) {
        WorkItem left;
        WorkItem right;
        if (SplitItem(item, left, right)) {
          if (stats.scheduled_cell_count > effective_cell_budget ||
              effective_cell_budget - stats.scheduled_cell_count < 2U) {
            stats.cell_budget_reached = true;
            fail(TubeCertificateFailureReason::CELL_BUDGET, item.w0, item.w1,
                 item.depth);
            return;
          }
          stats.scheduled_cell_count += 2U;
          process(left);
          process(right);
          return;
        }
      }
      fail(TubeCertificateFailureReason::MALFORMED_PATH_CELL, item.w0, item.w1,
           item.depth);
      close_anchor_component(item);
      return;
    }

    const CellAttempt attempt = EvaluateCell(state, cell, item.w0, item.w1,
                                             item.depth);
    if (attempt.valid && (attempt.nonzero ||
                          item.depth >= config_.budgets.max_w_depth)) {
      AcceptedCell retained;
      retained.proof = attempt.proof;
      // cell_count is incremented exactly once for every visited work item,
      // so using it here yields a deterministic, nonzero identity that stays
      // distinct even when several subdivisions share one producer segment.
      retained.proof.cell_id = static_cast<std::uint64_t>(stats.cell_count);
      accepted.push_back(retained);
      ++stats.accepted_cell_count;
      return;
    }

    if (item.depth < config_.budgets.max_w_depth) {
      WorkItem left;
      WorkItem right;
      if (SplitItem(item, left, right)) {
        if (stats.scheduled_cell_count > effective_cell_budget ||
            effective_cell_budget - stats.scheduled_cell_count < 2U) {
          stats.cell_budget_reached = true;
          fail(TubeCertificateFailureReason::CELL_BUDGET, item.w0, item.w1,
               item.depth);
          return;
        }
        stats.scheduled_cell_count += 2U;
        process(left);
        process(right);
        return;
      }
    }
    if (!attempt.valid) {
      fail(attempt.failure == TubeCertificateFailureReason::NONE
               ? TubeCertificateFailureReason::SUBDIVISION_DEPTH
               : attempt.failure,
           item.w0, item.w1, item.depth);
    } else {
      fail(TubeCertificateFailureReason::SUBDIVISION_DEPTH, item.w0, item.w1,
           item.depth);
    }
    close_anchor_component(item);
  };

  for (const WorkItem& item : initial) {
    process(item);
    if (anchor_component_closed) break;
    if (stats.cell_budget_reached || stats.query_budget_reached ||
        stats.witness_budget_reached || stats.sample_budget_reached) break;
  }

  if (accepted.empty()) {
    profile.truncation = TubeCertificateTruncation::NO_CERTIFIED_COMPONENT;
    fail(profile.failure_reason == TubeCertificateFailureReason::NONE
             ? TubeCertificateFailureReason::COMPONENT_GAP
             : profile.failure_reason);
    profile.failure.truncation = profile.truncation;
    profile.failure.query_count = stats.query_count;
    CopyCounters(stats, profile.counters);
    result.stats = stats;
    return false;
  }

  std::sort(accepted.begin(), accepted.end(),
            [](const AcceptedCell& lhs, const AcceptedCell& rhs) {
              return lhs.proof.w0 < rhs.proof.w0;
            });
  std::size_t anchor_index = accepted.size();
  for (std::size_t i = 0U; i < accepted.size(); ++i) {
    if (accepted[i].proof.w0 <= input.anchor_w &&
        accepted[i].proof.w1 >= input.anchor_w) {
      anchor_index = i;
      break;
    }
  }
  if (anchor_index == accepted.size()) {
    profile.truncation = TubeCertificateTruncation::ANCHOR_EXCLUDED;
    fail(TubeCertificateFailureReason::COMPONENT_GAP);
    profile.failure.truncation = profile.truncation;
    profile.failure.query_count = stats.query_count;
    CopyCounters(stats, profile.counters);
    result.stats = stats;
    return false;
  }

  std::size_t first = anchor_index;
  while (first > 0U &&
         accepted[first - 1U].proof.w1 == accepted[first].proof.w0) {
    --first;
  }
  std::size_t last = anchor_index;
  while (last + 1U < accepted.size() &&
         accepted[last].proof.w1 == accepted[last + 1U].proof.w0) {
    ++last;
  }
  std::vector<AcceptedCell> component;
  component.reserve(last - first + 1U);
  for (std::size_t i = first; i <= last; ++i) {
    component.push_back(accepted[i]);
    profile.cells.push_back(accepted[i].proof);
  }
  const bool has_prefix = component.front().proof.w0 > input.requested_start;
  const bool has_suffix = component.back().proof.w1 < input.requested_end;
  if (has_prefix && has_suffix) {
    profile.truncation = TubeCertificateTruncation::PREFIX_AND_SUFFIX;
  } else if (has_prefix) {
    profile.truncation = TubeCertificateTruncation::PREFIX;
  } else if (has_suffix) {
    profile.truncation = TubeCertificateTruncation::SUFFIX;
  } else {
    profile.truncation = TubeCertificateTruncation::NONE;
  }
  profile.certified_start = component.front().proof.w0;
  profile.certified_end = component.back().proof.w1;
  profile.contains_anchor = profile.certified_start <= input.anchor_w &&
      profile.certified_end >= input.anchor_w;
  if (!BuildPwl(component, profile)) {
    profile.valid = false;
    fail(TubeCertificateFailureReason::PWL_MALFORMED);
    profile.failure.truncation = profile.truncation;
    profile.failure.query_count = stats.query_count;
    CopyCounters(stats, profile.counters);
    result.stats = stats;
    return false;
  }
  profile.valid = profile.contains_anchor && profile.contains_zero_everywhere;
  profile.complete = profile.valid;
  profile.capability = profile.nonzero_capacity
      ? TubeProfileV2Capability::OFFSET_CERTIFIED
      : TubeProfileV2Capability::ZERO_ONLY;
  profile.failure.truncation = profile.truncation;
  profile.failure.query_count = stats.query_count;
  if (!profile.valid) {
    profile.capability = TubeProfileV2Capability::UNAVAILABLE;
    fail(TubeCertificateFailureReason::PWL_MALFORMED);
  }
  // request_id is the caller-assigned non-reused build identity.  It is
  // carried unchanged; installation may not rebind or replace this proof ID.
  profile.profile_id = input.request_id;
  CopyCounters(stats, profile.counters);
  result.stats = stats;
  result.success = profile.structurallyValid();
  return result.success;
}

bool TubeCertificateBuilderV2::build(const TubeBuildInputV2& input,
                                     TubeProfileV2& profile) const {
  TubeCertificateBuildResultV2 result;
  const bool success = build(input, result);
  profile = result.profile;
  return success;
}

}  // namespace phase_offset_navigation
