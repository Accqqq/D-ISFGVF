#include <gtest/gtest.h>

#include "phase_offset_navigation/tube_viability.h"

#include <boost/multiprecision/cpp_int.hpp>
#include <cfenv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "phase_offset_navigation/tube_profile_v2.h"

#if defined(__i386__) || defined(__x86_64__)
#include <immintrin.h>
#endif

namespace phase_offset_navigation {
namespace {

using ExactRational = boost::multiprecision::cpp_rational;

ExactRational DecodeExact(const double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  const bool negative = (bits >> 63U) != 0U;
  const std::uint64_t fraction =
      bits & ((std::uint64_t(1) << 52U) - std::uint64_t(1));
  const std::uint64_t exponent_bits = (bits >> 52U) & std::uint64_t(0x7ff);
  boost::multiprecision::cpp_int significand = 0;
  int exponent = 0;
  if (exponent_bits == 0U) {
    significand = fraction;
    exponent = -1074;
  } else {
    significand = (std::uint64_t(1) << 52U) | fraction;
    exponent = static_cast<int>(exponent_bits) - 1023 - 52;
  }
  if (negative) significand = -significand;
  ExactRational result = significand;
  if (exponent >= 0) {
    result *= (boost::multiprecision::cpp_int(1) <<
               static_cast<unsigned>(exponent));
  } else {
    result /= (boost::multiprecision::cpp_int(1) <<
               static_cast<unsigned>(-exponent));
  }
  return result;
}

TubeProfile MakeProfile(const std::vector<std::pair<double, double>>& bounds,
                        const std::uint64_t path_revision = 7U,
                        const std::uint64_t frame_revision = 8U,
                        const std::uint64_t profile_revision = 9U) {
  TubeProfile profile;
  profile.complete = true;
  profile.filtered_complete = true;
  profile.path_revision = path_revision;
  profile.frame_revision = frame_revision;
  profile.profile_revision = profile_revision;
  profile.preview_start_w = 0.0;
  profile.preview_end_w = static_cast<double>(bounds.size() - 1U);
  for (std::size_t i = 0U; i < bounds.size(); ++i) {
    TubeRawSample sample;
    sample.w = static_cast<double>(i);
    sample.p = Eigen::Vector3d(static_cast<double>(i), 0.0, 0.0);
    sample.filtered_lower = bounds[i].first;
    sample.filtered_upper = bounds[i].second;
    sample.complete = true;
    profile.samples.push_back(sample);
  }
  return profile;
}

TubeViabilityInput BaseInput(const TubeProfile& profile) {
  TubeViabilityInput input;
  input.profile = &profile;
  input.current_w = 0.0;
  input.current_delta = 0.0;
  input.upper_u_delta = 1.0;
  input.policy.preview_horizon_w = profile.preview_end_w;
  input.policy.sample_spacing_w = 1.0;
  input.policy.lower_nu = 0.5;
  input.policy.upper_nu = 1.0;
  input.policy.b_tight = 0.2;
  input.policy.b_open = 0.8;
  input.policy.policy_revision = 1U;
  input.policy.configuration_identity = 1U;
  input.policy.configuration_id = "test-normal-preview-w";
  input.path_revision = profile.path_revision;
  input.frame_revision = profile.frame_revision;
  input.profile_revision = profile.profile_revision;
  input.expected_path_revision = profile.path_revision;
  input.expected_frame_revision = profile.frame_revision;
  input.expected_profile_revision = profile.profile_revision;
  input.max_work = 100000U;
  return input;
}

TubeProfile MakeProfileWithRawAndFiltered(
    const std::vector<std::pair<double, double>>& filtered_bounds,
    const std::vector<std::pair<double, double>>& raw_bounds,
    const bool filtered_complete = true,
    const bool raw_complete = true) {
  TubeProfile profile;
  profile.complete = true;
  profile.filtered_complete = filtered_complete;
  profile.raw_complete = raw_complete;
  profile.path_revision = 7U;
  profile.frame_revision = 8U;
  profile.profile_revision = 9U;
  profile.preview_start_w = 0.0;
  profile.preview_end_w =
      static_cast<double>(filtered_bounds.size() - 1U);
  for (std::size_t i = 0U; i < filtered_bounds.size(); ++i) {
    TubeRawSample sample;
    sample.w = static_cast<double>(i);
    sample.p = Eigen::Vector3d(static_cast<double>(i), 0.0, 0.0);
    sample.filtered_lower = filtered_bounds[i].first;
    sample.filtered_upper = filtered_bounds[i].second;
    sample.raw_lower = raw_bounds[i].first;
    sample.raw_upper = raw_bounds[i].second;
    sample.complete = true;
    profile.samples.push_back(sample);
  }
  return profile;
}

std::shared_ptr<const void> V2Owner() {
  return std::static_pointer_cast<const void>(std::make_shared<int>(11));
}

void SetV2Interval(phase_offset_core::Binary64Interval& value,
                   const double lower, const double upper) {
  value.lower = lower;
  value.upper = upper;
  value.valid = true;
}

void SetV2VectorInterval(phase_offset_core::Binary64VectorInterval& value,
                         const Eigen::Vector3d& lower,
                         const Eigen::Vector3d& upper) {
  value.valid = true;
  for (std::size_t i = 0U; i < 3U; ++i) {
    SetV2Interval(value.component[i],
                  lower(static_cast<Eigen::Index>(i)),
                  upper(static_cast<Eigen::Index>(i)));
  }
}

phase_offset_core::CertifiedPathCellV2 MakeV2Cell(
    const double w0, const double w1, const std::uint64_t identity) {
  phase_offset_core::CertifiedPathCellV2 cell;
  cell.w0 = w0;
  cell.w1 = w1;
  cell.anchor_w = w0 + (w1 - w0) * 0.5;
  cell.path_revision = 7U;
  cell.frame_revision = 8U;
  cell.segment_identity = identity;
  cell.proof_identity = identity + 1000U;
  // The mathematical midpoint of an adjacent-ULP cell may not be a distinct
  // binary64 value.  Retain a sound affine enclosure of the whole cell
  // instead of collapsing the anchor position to a rounded singleton.
  SetV2VectorInterval(cell.anchor_position,
                      Eigen::Vector3d(w0, 0.0, 0.0),
                      Eigen::Vector3d(w1, 0.0, 0.0));
  SetV2VectorInterval(cell.anchor_p_w, Eigen::Vector3d(1.0, 0.0, 0.0),
                      Eigen::Vector3d(1.0, 0.0, 0.0));
  SetV2VectorInterval(cell.anchor_p_ww, Eigen::Vector3d::Zero(),
                      Eigen::Vector3d::Zero());
  SetV2Interval(cell.inf_p_w_norm, 1.0, 1.0);
  SetV2Interval(cell.sup_p_w_norm, 1.0, 1.0);
  SetV2Interval(cell.inf_horizontal_p_w_norm, 1.0, 1.0);
  SetV2Interval(cell.sup_p_ww_norm, 0.0, 0.0);
  SetV2Interval(cell.sup_horizontal_p_ww_norm, 0.0, 0.0);
  SetV2Interval(cell.sup_p_www_norm, 0.0, 0.0);
  SetV2Interval(cell.sup_normal_derivative, 0.0, 0.0);
  SetV2Interval(cell.normal_variation, 0.0, 0.0);
  SetV2Interval(cell.tangent_variation, 0.0, 0.0);
  SetV2Interval(cell.curvature_variation, 0.0, 0.0);
  SetV2Interval(cell.midpoint_position_variation, 0.0, w1 - w0);
  SetV2Interval(cell.chord_deviation, 0.0, 0.0);
  cell.horizontal_acceleration_bound_complete = true;
  cell.normal_frame_proof_complete = true;
  cell.phase_map_proof_complete = true;
  cell.provenance = phase_offset_core::kWorldHorizontalCrossProductProvenance;
  cell.valid = true;
  cell.complete = true;
  return cell;
}

TubePathKey V2PathKey(const double end_w, const double start_w = 0.0) {
  TubePathKey key;
  key.execution_generation = 1U;
  key.path_instance_id = 2U;
  key.path_revision = 7U;
  key.frame_revision = 8U;
  key.frame_convention_id = 1U;
  key.frame_convention =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  key.phase_orientation = 1;
  key.domain_start = start_w;
  key.domain_end = end_w;
  return key;
}

TubeConfigurationKey V2ConfigurationKey() {
  TubeConfigurationKey key;
  key.configuration_id = 17U;
  key.epsilon = 0.1;
  // The fixture's final PWL intervals reach +/-1; metadata D must cover that
  // declared transverse domain rather than retaining the old +/-0.2 value.
  key.nominal_half_width = 1.0;
  key.ray_step = 0.1;
  key.snapshot_resolution = 0.2;
  key.minimum_reference_speed = 0.5;
  return key;
}

TubeMapCaptureKey V2MapKey() {
  TubeMapCaptureKey key;
  key.map_instance_id = 3U;
  key.state_id = 4U;
  key.accepted_sequence = 5U;
  key.configuration_generation = 6U;
  // This captured map uses its own immutable configuration namespace; it is
  // intentionally distinct from the geometry configuration id (17).
  key.configuration_id = 99U;
  key.frame_provenance_id = 8U;
  key.frame_provenance = "world";
  key.support_provenance_id = 9U;
  key.accepted_time_ticks = 10U;
  key.support_expiry_ticks = 20U;
  key.support_halo = 0.0;
  key.halo_reconciled = true;
  key.grid_min_index_x = -100;
  key.grid_min_index_y = -100;
  key.grid_min_index_z = -100;
  key.grid_max_index_x = 100;
  key.grid_max_index_y = 100;
  key.grid_max_index_z = 100;
  key.grid_native_origin = Eigen::Vector3d::Zero();
  key.grid_voxel_resolution = Eigen::Vector3d::Constant(0.2);
  key.complete_support = true;
  return key;
}

TubeProfileV2 MakeV2Profile(
    const std::vector<double>& knots,
    const std::vector<std::pair<double, double>>& bounds) {
  EXPECT_EQ(knots.size(), bounds.size());
  TubeProfileV2 profile;
  const double end_w = knots.empty() ? 0.0 : knots.back();
  const double start_w = knots.empty() ? 0.0 : knots.front();
  profile.path_key = V2PathKey(end_w, start_w);
  profile.configuration_key = V2ConfigurationKey();
  profile.map_capture_key = V2MapKey();
  profile.profile_id = 123U;
  profile.request_id = 456U;
  profile.requested_start = start_w;
  profile.requested_end = end_w;
  profile.anchor_w = profile.requested_start;
  profile.certified_start = profile.requested_start;
  profile.certified_end = end_w;
  profile.path_owner = V2Owner();
  profile.capture_owner = V2Owner();
  profile.query_owner = V2Owner();
  profile.applicability_assumptions = "immutable V2 test capture";
  profile.applicability_deadline_ticks = 15U;
  profile.valid = true;
  profile.complete = true;
  profile.contains_anchor = true;
  profile.contains_zero_everywhere = true;
  profile.nonzero_capacity = false;
  for (const std::pair<double, double>& bound : bounds) {
    profile.contains_zero_everywhere = profile.contains_zero_everywhere &&
        std::isfinite(bound.first) && std::isfinite(bound.second) &&
        bound.first <= 0.0 && bound.second >= 0.0;
    profile.nonzero_capacity = profile.nonzero_capacity ||
        (std::isfinite(bound.first) && std::isfinite(bound.second) &&
         bound.second > bound.first);
  }
  profile.capability = profile.nonzero_capacity
      ? TubeProfileV2Capability::OFFSET_CERTIFIED
      : TubeProfileV2Capability::ZERO_ONLY;
  for (std::size_t i = 0U; i < knots.size(); ++i) {
    TubePwlKnotV2 knot;
    knot.w = knots[i];
    knot.lower = bounds[i].first;
    knot.upper = bounds[i].second;
    knot.valid = true;
    if (i > 0U) {
      knot.left_cell_id = static_cast<std::uint64_t>(100U + i);
      const double dw = knots[i] - knots[i - 1U];
      const double lower_slope =
          (bounds[i].first - bounds[i - 1U].first) / dw;
      const double upper_slope =
          (bounds[i].second - bounds[i - 1U].second) / dw;
      knot.left_lower_slope = lower_slope;
      knot.left_upper_slope = upper_slope;
      knot.left_lower_slope_interval = {lower_slope, lower_slope, true};
      knot.left_upper_slope_interval = {upper_slope, upper_slope, true};
    }
    if (i + 1U < knots.size()) {
      knot.right_cell_id = static_cast<std::uint64_t>(100U + i + 1U);
      const double dw = knots[i + 1U] - knots[i];
      const double lower_slope =
          (bounds[i + 1U].first - bounds[i].first) / dw;
      const double upper_slope =
          (bounds[i + 1U].second - bounds[i].second) / dw;
      knot.right_lower_slope = lower_slope;
      knot.right_upper_slope = upper_slope;
      knot.right_lower_slope_interval = {lower_slope, lower_slope, true};
      knot.right_upper_slope_interval = {upper_slope, upper_slope, true};
    }
    profile.knots.push_back(knot);
  }
  for (std::size_t i = 0U; i + 1U < knots.size(); ++i) {
    TubeProofCellV2 cell;
    cell.w0 = knots[i];
    cell.w1 = knots[i + 1U];
    // A PWL cell's certified interval must contain both endpoint intervals;
    // use their closed hull rather than silently dropping a widening knot.
    cell.lower = std::min(bounds[i].first, bounds[i + 1U].first);
    cell.upper = std::max(bounds[i].second, bounds[i + 1U].second);
    cell.cell_id = static_cast<std::uint64_t>(100U + i + 1U);
    cell.segment_identity = 77U;
    cell.proof_identity = 700U + i;
    cell.depth = 0;
    cell.zero_anchor_certified = true;
    cell.complete = true;
    cell.valid = true;
    cell.path_cell = MakeV2Cell(knots[i], knots[i + 1U],
                                static_cast<std::uint64_t>(1000U + i));
    profile.cells.push_back(cell);
  }
  if (profile.knots.size() >= 2U && !profile.cells.empty()) {
    profile.knots.front().right_cell_id = profile.cells.front().cell_id;
    profile.knots.back().left_cell_id = profile.cells.back().cell_id;
    for (std::size_t i = 1U; i + 1U < profile.knots.size(); ++i) {
      profile.knots[i].left_cell_id = profile.cells[i - 1U].cell_id;
      profile.knots[i].right_cell_id = profile.cells[i].cell_id;
    }
  }
  return profile;
}

NormalPreviewProductionPolicy V2Policy(const double horizon = 2.0,
                                        const double spacing = 1.0) {
  NormalPreviewProductionPolicy policy;
  policy.preview_horizon_w = horizon;
  policy.sample_spacing_w = spacing;
  policy.lower_nu = 0.5;
  policy.upper_nu = 1.0;
  policy.b_tight = 0.2;
  policy.b_open = 0.8;
  policy.policy_revision = 1U;
  policy.configuration_identity = 17U;
  policy.configuration_id = "v2-test-config";
  return policy;
}

TubeViabilityInput V2Input(const double current_w, const double current_delta,
                           const NormalPreviewProductionPolicy& policy,
                           const double upper_u_delta = 1.0) {
  TubeViabilityInput input;
  input.current_w = current_w;
  input.current_delta = current_delta;
  input.policy = policy;
  input.upper_u_delta = upper_u_delta;
  input.max_work = 100000U;
  return input;
}

TEST(TubeViabilityTest, CompleteValidFilteredBoundsAreAuthoritative) {
  TubeProfile profile = MakeProfileWithRawAndFiltered(
      {{-0.2, 0.3}, {-0.4, 0.5}}, {{-0.9, 0.9}, {-0.8, 0.8}});
  TubeViabilityInput input = BaseInput(profile);
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.feasible);
  ASSERT_EQ(result.knots.size(), 2U);
  EXPECT_DOUBLE_EQ(result.knots[0].geometric.lower, -0.2);
  EXPECT_DOUBLE_EQ(result.knots[0].geometric.upper, 0.3);
  EXPECT_DOUBLE_EQ(result.knots[1].geometric.lower, -0.4);
  EXPECT_DOUBLE_EQ(result.knots[1].geometric.upper, 0.5);
}

TEST(TubeViabilityTest, CompleteNonFiniteFilteredBoundsFailClosed) {
  TubeProfile profile = MakeProfileWithRawAndFiltered(
      {{std::numeric_limits<double>::quiet_NaN(), 0.3}, {-0.4, 0.5}},
      {{-0.9, 0.9}, {-0.8, 0.8}});
  TubeViabilityInput input = BaseInput(profile);
  TubeViabilityResult result;
  EXPECT_FALSE(TubeViability::evaluate(input, result));
  EXPECT_EQ(result.status, TubeViabilityStatus::INVALID_INPUT);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.knots.empty());
}

TEST(TubeViabilityTest, CompleteInvertedFilteredBoundsDoNotUseValidRawBounds) {
  TubeProfile profile = MakeProfileWithRawAndFiltered(
      {{0.4, -0.3}, {-0.4, 0.5}}, {{-0.9, 0.9}, {-0.8, 0.8}});
  TubeViabilityInput input = BaseInput(profile);
  TubeViabilityResult result;
  EXPECT_FALSE(TubeViability::evaluate(input, result));
  EXPECT_EQ(result.status, TubeViabilityStatus::INVALID_INPUT);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.knots.empty());
}

TEST(TubeViabilityTest, IncompleteFilteredProfilePreservesRawFallbackOrdering) {
  TubeProfile raw_authoritative = MakeProfileWithRawAndFiltered(
      {{std::numeric_limits<double>::quiet_NaN(), 0.3},
       {-0.4, 0.5}},
      {{-0.9, 0.9}, {-0.8, 0.8}}, false, true);
  TubeViabilityInput input = BaseInput(raw_authoritative);
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.feasible);
  EXPECT_DOUBLE_EQ(result.knots[0].geometric.lower, -0.9);
  EXPECT_DOUBLE_EQ(result.knots[0].geometric.upper, 0.9);

  TubeProfile filtered_fallback = MakeProfileWithRawAndFiltered(
      {{-0.2, 0.3}, {-0.4, 0.5}},
      {{std::numeric_limits<double>::quiet_NaN(), 0.0},
       {std::numeric_limits<double>::quiet_NaN(), 0.0}},
      false, false);
  input = BaseInput(filtered_fallback);
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.feasible);
  EXPECT_DOUBLE_EQ(result.knots[0].geometric.lower, -0.2);
  EXPECT_DOUBLE_EQ(result.knots[0].geometric.upper, 0.3);
}

TEST(TubeViabilityTest, OpenNarrowingAndAsymmetricIntervalsArePreserved) {
  TubeProfile open = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}});
  TubeViabilityInput input = BaseInput(open);
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.feasible);
  ASSERT_EQ(result.knots.size(), 3U);
  EXPECT_DOUBLE_EQ(result.knots.front().reachable.lower, -1.0);
  EXPECT_DOUBLE_EQ(result.knots.front().reachable.upper, 1.0);

  TubeProfile narrowing = MakeProfile({{-1.0, 1.0}, {-0.5, 0.5},
                                       {-0.2, 0.2}});
  input = BaseInput(narrowing);
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.feasible);
  EXPECT_DOUBLE_EQ(result.knots[2].reachable.lower, -0.2);
  EXPECT_DOUBLE_EQ(result.knots[2].reachable.upper, 0.2);
  EXPECT_DOUBLE_EQ(result.knots[1].reachable.lower, -0.5);
  EXPECT_DOUBLE_EQ(result.knots[1].reachable.upper, 0.5);
  TubeViabilityInterval middle;
  ASSERT_TRUE(result.envelopeAt(0.5, middle));
  EXPECT_NEAR(middle.lower, -0.75, 1e-12);
  EXPECT_NEAR(middle.upper, 0.75, 1e-12);

  TubeProfile asymmetric = MakeProfile({{-0.8, 0.3}, {-0.6, 0.25},
                                         {-0.4, 0.2}});
  input = BaseInput(asymmetric);
  input.current_delta = -0.4;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.feasible);
  EXPECT_LT(result.knots[0].reachable.lower,
            result.knots[0].reachable.upper);
  EXPECT_NE(result.knots[0].reachable.lower,
            -result.knots[0].reachable.upper);
}

TEST(TubeViabilityTest, BackwardRecursionSeparatesRateFeasibleAndInfeasible) {
  TubeProfile feasible = MakeProfile({{-1.0, 1.0}, {-0.1, 0.1},
                                      {-0.1, 0.1}});
  TubeViabilityInput input = BaseInput(feasible);
  input.upper_u_delta = 1.0;
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  EXPECT_TRUE(result.feasible);
  EXPECT_NEAR(result.knots[0].reachable.lower, -1.0, 1e-12);
  EXPECT_NEAR(result.knots[0].reachable.upper, 1.0, 1e-12);

  TubeProfile infeasible = MakeProfile({{-1.0, -0.5}, {0.5, 1.0},
                                        {0.5, 1.0}});
  input = BaseInput(infeasible);
  input.upper_u_delta = 0.1;
  input.current_delta = -0.75;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.status, TubeViabilityStatus::PREVIEW_INFEASIBLE);
  EXPECT_FALSE(result.knots[0].reachable.valid);
}

TEST(TubeViabilityTest, ExactBPreAndCubicSmoothstepSaturation) {
  TubeProfile profile = MakeProfile({{-0.4, 0.8}, {-0.2, 0.5},
                                     {-0.1, 0.3}});
  TubeViabilityInput input = BaseInput(profile);
  input.upper_u_delta = 0.0;
  input.policy.b_tight = 0.3;
  input.policy.b_open = 0.9;
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.feasible);
  EXPECT_DOUBLE_EQ(result.b_pre, 0.4);
  EXPECT_DOUBLE_EQ(result.beta,
                   TubeViability::smoothstep((0.4 - 0.3) / (0.9 - 0.3)));
  EXPECT_DOUBLE_EQ(result.beta_i, result.beta);
  EXPECT_DOUBLE_EQ(TubeViability::smoothstep(-1.0), 0.0);
  EXPECT_DOUBLE_EQ(TubeViability::smoothstep(0.0), 0.0);
  EXPECT_DOUBLE_EQ(TubeViability::smoothstep(0.5), 0.5);
  EXPECT_DOUBLE_EQ(TubeViability::smoothstep(1.0), 1.0);
  EXPECT_DOUBLE_EQ(TubeViability::smoothstep(2.0), 1.0);

  input.policy.b_tight = 0.5;
  input.policy.b_open = 0.6;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  EXPECT_DOUBLE_EQ(result.beta, 0.0);
  input.policy.b_tight = 0.1;
  input.policy.b_open = 0.2;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  EXPECT_DOUBLE_EQ(result.beta, 1.0);
}

TEST(TubeViabilityTest, BoundaryRateIntervalsUseExactOneSidedSlopes) {
  TubeProfile profile = MakeProfile({{-0.2, 0.4}, {-0.1, 0.3},
                                     {0.0, 0.2}});
  TubeViabilityInput input = BaseInput(profile);
  input.current_delta = -0.2;
  input.upper_u_delta = 0.5;
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.feasible);
  ASSERT_TRUE(result.current_rate_interval.valid);
  EXPECT_TRUE(result.current_rate_interval.lower_boundary_active);
  EXPECT_NEAR(result.current_rate_interval.lower, 0.1, 1e-12);
  EXPECT_NEAR(result.knots.front().lower_slope_right, 0.1, 1e-12);

  input.current_delta = 0.4;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.feasible);
  ASSERT_TRUE(result.current_rate_interval.valid);
  EXPECT_TRUE(result.current_rate_interval.upper_boundary_active);
  EXPECT_NEAR(result.current_rate_interval.upper, -0.1, 1e-12);
}

TEST(TubeViabilityTest, CurrentDeltaOutsideK0IsReportedWithoutPlannerVeto) {
  TubeProfile profile = MakeProfile({{-0.2, 0.2}, {-0.2, 0.2},
                                     {-0.2, 0.2}});
  TubeViabilityInput input = BaseInput(profile);
  input.current_delta = 0.8;
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.feasible);
  EXPECT_FALSE(result.current_delta_inside);
  EXPECT_EQ(result.status, TubeViabilityStatus::CURRENT_DELTA_OUTSIDE);
}

TEST(TubeViabilityTest, StalePathFrameProfileMismatchIsRejected) {
  TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  TubeViabilityInput input = BaseInput(profile);
  input.expected_path_revision = profile.path_revision + 1U;
  TubeViabilityResult result;
  EXPECT_FALSE(TubeViability::evaluate(input, result));
  EXPECT_EQ(result.status, TubeViabilityStatus::STALE);

  input = BaseInput(profile);
  input.expected_frame_revision = profile.frame_revision + 1U;
  EXPECT_FALSE(TubeViability::evaluate(input, result));
  EXPECT_EQ(result.status, TubeViabilityStatus::STALE);
  EXPECT_FALSE(result.valid);

  input = BaseInput(profile);
  input.profile_revision = profile.profile_revision + 1U;
  EXPECT_FALSE(TubeViability::evaluate(input, result));
  EXPECT_EQ(result.status, TubeViabilityStatus::STALE);
}

TEST(TubeViabilityTest, RepeatedInputIsBitwiseDeterministicInValues) {
  TubeProfile profile = MakeProfile({{-0.7, 0.8}, {-0.5, 0.6},
                                     {-0.2, 0.4}, {-0.1, 0.3}});
  TubeViabilityInput input = BaseInput(profile);
  input.current_delta = -0.1;
  input.policy.preview_horizon_w = 3.0;
  input.policy.sample_spacing_w = 0.5;
  TubeViabilityResult first;
  TubeViabilityResult second;
  ASSERT_TRUE(TubeViability::evaluate(input, first));
  ASSERT_TRUE(TubeViability::evaluate(input, second));
  EXPECT_EQ(first.status, second.status);
  EXPECT_DOUBLE_EQ(first.b_pre, second.b_pre);
  EXPECT_DOUBLE_EQ(first.beta, second.beta);
  EXPECT_DOUBLE_EQ(first.delta_w, second.delta_w);
  EXPECT_DOUBLE_EQ(first.delta_delta, second.delta_delta);
  ASSERT_EQ(first.knots.size(), second.knots.size());
  for (std::size_t i = 0U; i < first.knots.size(); ++i) {
    EXPECT_DOUBLE_EQ(first.knots[i].w, second.knots[i].w);
    EXPECT_DOUBLE_EQ(first.knots[i].geometric.lower,
                     second.knots[i].geometric.lower);
    EXPECT_DOUBLE_EQ(first.knots[i].geometric.upper,
                     second.knots[i].geometric.upper);
    EXPECT_DOUBLE_EQ(first.knots[i].reachable.lower,
                     second.knots[i].reachable.lower);
    EXPECT_DOUBLE_EQ(first.knots[i].reachable.upper,
                     second.knots[i].reachable.upper);
  }
}

TEST(TubeViabilityTest, MissingRequiredPolicyFailsClosedWithoutDefaults) {
  TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  TubeViabilityInput input;
  input.profile = &profile;
  input.current_w = 0.0;
  input.current_delta = 0.0;
  input.upper_u_delta = 1.0;
  TubeViabilityResult result;
  EXPECT_FALSE(TubeViability::evaluate(input, result));
  EXPECT_EQ(result.status, TubeViabilityStatus::INVALID_INPUT);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.knots.empty());
}

TEST(TubeViabilityTest, FixedWLookaheadClipsOnlyAtProfilePreviewEnd) {
  TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0},
                                     {-0.8, 0.8}, {-0.6, 0.6},
                                     {-0.5, 0.5}});
  TubeViabilityInput input = BaseInput(profile);
  input.current_w = 0.5;
  input.policy.preview_horizon_w = 10.0;
  input.policy.sample_spacing_w = 0.75;
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  EXPECT_TRUE(result.preview_truncated_after);
  EXPECT_DOUBLE_EQ(result.preview_start_w, 0.5);
  EXPECT_DOUBLE_EQ(result.preview_end_w, profile.preview_end_w);
  EXPECT_DOUBLE_EQ(result.knots.front().w, 0.5);
  EXPECT_DOUBLE_EQ(result.knots.back().w, profile.preview_end_w);
  EXPECT_EQ(result.knots.size(), 6U);
  EXPECT_DOUBLE_EQ(result.delta_w, 0.7);
}

TEST(TubeViabilityTest, RobustBoundaryRateUsesFullPhaseEnvelope) {
  TubeProfile profile = MakeProfile({{-0.2, 0.4}, {-0.1, 0.3},
                                     {0.0, 0.2}});
  TubeViabilityInput input = BaseInput(profile);
  input.policy.lower_nu = 0.5;
  input.policy.upper_nu = 2.0;
  input.upper_u_delta = 1.0;
  input.current_delta = -0.2;
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.feasible);
  ASSERT_TRUE(result.current_rate_interval.valid);
  EXPECT_NEAR(result.current_rate_interval.lower, 0.2, 1e-12);

  input.current_delta = 0.4;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  ASSERT_TRUE(result.current_rate_interval.valid);
  EXPECT_NEAR(result.current_rate_interval.upper, -0.2, 1e-12);
}

TEST(TubeViabilityTest, QueryUsesWCoordinateAndOneSidedKnotRules) {
  TubeProfile profile = MakeProfile({{-0.8, 0.3}, {-0.6, 0.25},
                                     {-0.4, 0.2}});
  TubeViabilityInput input = BaseInput(profile);
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(input, result));
  TubeViabilityInterval interval;
  ASSERT_TRUE(result.envelopeAt(1.0, interval));
  EXPECT_DOUBLE_EQ(interval.lower, result.knots[1].reachable.lower);
  EXPECT_DOUBLE_EQ(interval.upper, result.knots[1].reachable.upper);
  TubeViabilityRateInterval rate;
  ASSERT_TRUE(result.rateIntervalAt(1.0, interval.lower, 1.0, rate));
  EXPECT_TRUE(rate.lower_boundary_active);
}

TEST(TubeViabilityV2Test, ExactProvenanceIsRetainedAndMapConfigMayDiffer) {
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 2.0}, {{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}});
  // Geometry and map captures are independent immutable authorities.  The
  // map key's 99 namespace is captured with the profile; this test does not
  // relabel old map evidence after a geometry configuration change.
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy());
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.proof_kind, TubeViabilityProofKind::V2_LIVE_PWL);
  EXPECT_TRUE(result.path_key == profile.path_key);
  EXPECT_TRUE(result.configuration_key == profile.configuration_key);
  EXPECT_TRUE(result.map_capture_key == profile.map_capture_key);
  EXPECT_EQ(result.profile_id, profile.profile_id);

  input.v2_provenance_bound = true;
  input.expected_v2_path_key = profile.path_key;
  input.expected_v2_configuration_key = profile.configuration_key;
  input.expected_v2_map_capture_key = profile.map_capture_key;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  profile.map_capture_key.state_id += 1U;
  EXPECT_FALSE(TubeViability::evaluate(profile, input, result));
  EXPECT_EQ(result.status, TubeViabilityStatus::STALE);
}

TEST(TubeViabilityV2Test, GeometricBottleneckKnotSurvivesCoarseSampling) {
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 4.0}, {{-1.0, 1.0}, {-0.1, 0.1}, {-1.0, 1.0}});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy(4.0, 3.0));
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  ASSERT_TRUE(result.feasible);
  bool retained = false;
  for (const TubeViabilityKnot& knot : result.knots) {
    if (knot.w == 1.0) {
      retained = true;
      EXPECT_DOUBLE_EQ(knot.geometric.lower, -0.1);
      EXPECT_DOUBLE_EQ(knot.geometric.upper, 0.1);
    }
  }
  EXPECT_TRUE(retained);
}

TEST(TubeViabilityV2Test, AdjacentUlpKnotsAreRetainedAndEndpointIsExact) {
  const double adjacent = std::nextafter(1.0, 2.0);
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, adjacent, 2.0},
      {{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy(2.0, 2.0));
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  ASSERT_TRUE(result.feasible);
  bool saw_one = false;
  bool saw_adjacent = false;
  for (const TubeViabilityKnot& knot : result.knots) {
    saw_one = saw_one || knot.w == 1.0;
    saw_adjacent = saw_adjacent || knot.w == adjacent;
  }
  EXPECT_TRUE(saw_one);
  EXPECT_TRUE(saw_adjacent);
  EXPECT_DOUBLE_EQ(result.preview_end_w, 2.0);
  EXPECT_FALSE(result.preview_truncated_after);

  input.policy.preview_horizon_w = std::nextafter(2.0, 3.0);
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.feasible);
  EXPECT_TRUE(result.preview_truncated_after);
  EXPECT_EQ(result.status, TubeViabilityStatus::PREVIEW_INFEASIBLE);
}

TEST(TubeViabilityV2Test, NonuniformFinalCellUsesItsActualWidth) {
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 2.0, 2.5},
      {{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}, {-0.1, 0.1}});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy(2.5, 1.0));
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  ASSERT_TRUE(result.feasible);
  std::size_t index = result.knots.size();
  for (std::size_t i = 0U; i < result.knots.size(); ++i) {
    if (result.knots[i].w == 2.0) index = i;
  }
  ASSERT_LT(index, result.knots.size());
  ASSERT_LT(index + 1U, result.knots.size());
  EXPECT_DOUBLE_EQ(result.knots[index + 1U].w - result.knots[index].w,
                   0.5);
  EXPECT_NEAR(result.knots[index].reachable.lower, -0.6, 1e-14);
  EXPECT_NEAR(result.knots[index].reachable.upper, 0.6, 1e-14);
}

TEST(TubeViabilityV2Test, LargeImmutablePrefixDoesNotConsumeLivePartitionBudget) {
  constexpr std::size_t kPrefixKnots = 100001U;
  std::vector<double> knots;
  knots.reserve(kPrefixKnots + 1U);
  std::vector<std::pair<double, double>> bounds;
  bounds.reserve(kPrefixKnots + 1U);
  for (std::size_t i = 0U; i <= kPrefixKnots; ++i) {
    knots.push_back(static_cast<double>(i));
    bounds.emplace_back(0.0, 0.0);
  }
  TubeProfileV2 profile = MakeV2Profile(knots, bounds);
  TubeViabilityInput input = V2Input(100000.0, 0.0, V2Policy(1.0, 1.0),
                                     0.0);
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  ASSERT_TRUE(result.feasible);
  ASSERT_EQ(result.knots.size(), 2U);
  EXPECT_DOUBLE_EQ(result.knots.front().w, 100000.0);
  EXPECT_DOUBLE_EQ(result.knots.back().w, 100001.0);
}

TEST(TubeViabilityV2Test, EmptyMalformedFailsClosedButZeroProfileIsValid) {
  TubeProfileV2 empty = MakeV2Profile({}, {});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy());
  TubeViabilityResult result;
  EXPECT_FALSE(TubeViability::evaluate(empty, input, result));
  EXPECT_EQ(result.status, TubeViabilityStatus::INVALID_INPUT);
  EXPECT_FALSE(result.valid);

  TubeProfileV2 malformed = MakeV2Profile(
      {0.0, 1.0}, {{-1.0, 1.0}, {-1.0, 1.0}});
  malformed.knots[1].lower = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(TubeViability::evaluate(malformed, input, result));
  EXPECT_EQ(result.status, TubeViabilityStatus::INVALID_INPUT);
  EXPECT_FALSE(result.valid);

  TubeProfileV2 zero = MakeV2Profile({0.0, 1.0, 2.0},
                                     {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}});
  ASSERT_TRUE(TubeViability::evaluate(zero, input, result))
      << result.reason;
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.feasible);
  EXPECT_EQ(result.status, TubeViabilityStatus::FEASIBLE);
  EXPECT_DOUBLE_EQ(result.b_pre, 0.0);
}

TEST(TubeViabilityV2Test, WorkBudgetIsBoundedAndIdentityMismatchIsStale) {
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 2.0}, {{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy(2.0, 0.5));
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  ASSERT_TRUE(result.valid);
  ASSERT_GT(result.work_count, 0U);
  EXPECT_EQ(result.max_work, input.max_work);
  EXPECT_LE(result.work_count, result.max_work);

  TubeViabilityInput exhausted = input;
  exhausted.max_work = result.work_count - 1U;
  TubeViabilityResult exhausted_result;
  EXPECT_FALSE(TubeViability::evaluate(profile, exhausted, exhausted_result));
  EXPECT_EQ(exhausted_result.status, TubeViabilityStatus::INVALID_INPUT);
  EXPECT_FALSE(exhausted_result.valid);
  EXPECT_LE(exhausted_result.work_count, exhausted.max_work);

  TubeViabilityInput stale = input;
  stale.path_revision = profile.path_key.path_revision + 1U;
  TubeViabilityResult stale_result;
  EXPECT_FALSE(TubeViability::evaluate(profile, stale, stale_result));
  EXPECT_EQ(stale_result.status, TubeViabilityStatus::STALE);
  EXPECT_FALSE(stale_result.valid);
}

TEST(TubeViabilityV2Test, ExactAndAboveContractionRateBoundariesUseBothFaces) {
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 2.0}, {{-1.0, 1.0}, {-0.5, 0.5}, {0.0, 0.0}});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy(), 0.5);
  TubeViabilityResult exact;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, exact))
      << exact.reason;
  ASSERT_TRUE(exact.feasible);
  EXPECT_TRUE(exact.contraction_rate_feasible);
  EXPECT_NEAR(exact.max_inward_contraction_slope, 0.5, 1e-14);
  ASSERT_TRUE(exact.knots[1].lower_slope_right_interval.valid);
  ASSERT_TRUE(exact.knots[1].upper_slope_right_interval.valid);
  EXPECT_LE(exact.knots[1].lower_slope_right_interval.upper, 0.5);
  EXPECT_GE(exact.knots[1].upper_slope_right_interval.lower, -0.5);

  input.upper_u_delta = std::nextafter(0.5, 0.0);
  TubeViabilityResult above;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, above))
      << above.reason;
  EXPECT_TRUE(above.valid);
  EXPECT_TRUE(above.feasible);
  // K is recomputed for the narrower capability, so its incoming slopes can
  // contract with the reduced rate instead of reusing the old K certificate.
  EXPECT_TRUE(above.contraction_rate_feasible);
  EXPECT_EQ(above.status, TubeViabilityStatus::FEASIBLE);

  // Reusing the old K with a too-small capability must not be rescued by an
  // outward-product ULP clip: the unchanged boundary query is rejected.
  TubeViabilityResult unchanged = exact;
  unchanged.upper_u_delta = input.upper_u_delta;
  TubeViabilityRateInterval rate;
  EXPECT_FALSE(unchanged.rateIntervalAt(
      unchanged.knots.front().w, unchanged.knots.front().reachable.lower,
      1.0, rate));
}

TEST(TubeViabilityV2Test, AsymmetricMultipleKinkEnvelopeUsesMinimumKWidth) {
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 2.0, 3.0},
      {{-1.0, 1.0}, {-0.8, 0.7}, {-0.2, 0.35}, {-0.6, 1.0}});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy(3.0, 2.0));
  input.policy.b_tight = 0.1;
  input.policy.b_open = 0.9;
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  ASSERT_TRUE(result.feasible);
  for (const TubeViabilityKnot& knot : result.knots) {
    EXPECT_GE(knot.reachable.lower, knot.geometric.lower);
    EXPECT_LE(knot.reachable.upper, knot.geometric.upper);
  }
  EXPECT_NEAR(result.b_pre, 0.55, 1e-14);
  EXPECT_DOUBLE_EQ(
      result.beta,
      TubeViability::smoothstep((result.b_pre - input.policy.b_tight) /
                                (input.policy.b_open - input.policy.b_tight)));
}

TEST(TubeViabilityV2Test, ExactRationalOracleBoundsReachAndInteriorQueries) {
  // Negative live phases and a nonuniform, signed PWL envelope exercise the
  // exact dyadic reach products independently of the implementation's sampled
  // doubles.  The coarse grid contributes no extra geometric knot here; every
  // frozen breakpoint remains in the merged partition.
  const std::vector<double> phases = {-1.0, -0.75, 0.0, 0.5, 1.0};
  const std::vector<std::pair<double, double>> bounds = {
      {-1.0, 1.0}, {-0.75, 0.75}, {-0.5, 0.5}, {-0.25, 0.25},
      {-0.5, 1.0}};
  TubeProfileV2 profile = MakeV2Profile(phases, bounds);
  TubeViabilityInput input = V2Input(-1.0, 0.0, V2Policy(2.0, 2.0), 0.75);
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  ASSERT_TRUE(result.feasible);
  ASSERT_EQ(result.knots.size(), phases.size());

  std::vector<ExactRational> exact_lower(result.knots.size());
  std::vector<ExactRational> exact_upper(result.knots.size());
  const ExactRational exact_rate =
      DecodeExact(input.upper_u_delta) /
      DecodeExact(input.policy.upper_nu);
  for (std::size_t i = result.knots.size(); i-- > 0U;) {
    // Build the independent geometric oracle from the fixture's original
    // PWL endpoints, never from the implementation's rounded output.
    const ExactRational geometric_lower = DecodeExact(bounds[i].first);
    const ExactRational geometric_upper = DecodeExact(bounds[i].second);
    if (i + 1U == result.knots.size()) {
      exact_lower[i] = geometric_lower;
      exact_upper[i] = geometric_upper;
      continue;
    }
    const ExactRational span = DecodeExact(result.knots[i + 1U].w) -
        DecodeExact(result.knots[i].w);
    const ExactRational exact_reach = exact_rate * span;
    const ExactRational backward_lower = exact_lower[i + 1U] - exact_reach;
    const ExactRational backward_upper = exact_upper[i + 1U] + exact_reach;
    exact_lower[i] = geometric_lower > backward_lower
        ? geometric_lower : backward_lower;
    exact_upper[i] = geometric_upper < backward_upper
        ? geometric_upper : backward_upper;
  }
  for (std::size_t i = 0U; i + 1U < result.knots.size(); ++i) {
    const ExactRational span = DecodeExact(phases[i + 1U]) -
        DecodeExact(phases[i]);
    const ExactRational exact_reach = exact_rate * span;
    EXPECT_TRUE(exact_lower[i + 1U] - exact_lower[i] <= exact_reach)
        << "lower reach cell " << i;
    EXPECT_TRUE(exact_upper[i] - exact_upper[i + 1U] <= exact_reach)
        << "upper reach cell " << i;
  }
  for (std::size_t i = 0U; i < result.knots.size(); ++i) {
    EXPECT_TRUE(DecodeExact(result.knots[i].reachable.lower) >= exact_lower[i])
        << "lower knot " << i;
    EXPECT_TRUE(DecodeExact(result.knots[i].reachable.upper) <= exact_upper[i])
        << "upper knot " << i;
    EXPECT_TRUE(result.knots[i].reachable.lower >=
                result.knots[i].geometric.lower);
    EXPECT_TRUE(result.knots[i].reachable.upper <=
                result.knots[i].geometric.upper);
    if (i > 0U) {
      const ExactRational span = DecodeExact(phases[i]) -
          DecodeExact(phases[i - 1U]);
      const ExactRational lower_slope =
          (DecodeExact(result.knots[i].reachable.lower) -
           DecodeExact(result.knots[i - 1U].reachable.lower)) / span;
      const ExactRational upper_slope =
          (DecodeExact(result.knots[i].reachable.upper) -
           DecodeExact(result.knots[i - 1U].reachable.upper)) / span;
      EXPECT_TRUE(DecodeExact(result.knots[i].lower_slope_left_interval.lower) <=
                  lower_slope);
      EXPECT_TRUE(DecodeExact(result.knots[i].lower_slope_left_interval.upper) >=
                  lower_slope);
      EXPECT_TRUE(DecodeExact(result.knots[i].upper_slope_left_interval.lower) <=
                  upper_slope);
      EXPECT_TRUE(DecodeExact(result.knots[i].upper_slope_left_interval.upper) >=
                  upper_slope);
    }
    if (i + 1U < result.knots.size()) {
      const ExactRational span = DecodeExact(phases[i + 1U]) -
          DecodeExact(phases[i]);
      const ExactRational lower_slope =
          (DecodeExact(result.knots[i + 1U].reachable.lower) -
           DecodeExact(result.knots[i].reachable.lower)) / span;
      const ExactRational upper_slope =
          (DecodeExact(result.knots[i + 1U].reachable.upper) -
           DecodeExact(result.knots[i].reachable.upper)) / span;
      EXPECT_TRUE(DecodeExact(result.knots[i].lower_slope_right_interval.lower) <=
                  lower_slope);
      EXPECT_TRUE(DecodeExact(result.knots[i].lower_slope_right_interval.upper) >=
                  lower_slope);
      EXPECT_TRUE(DecodeExact(result.knots[i].upper_slope_right_interval.lower) <=
                  upper_slope);
      EXPECT_TRUE(DecodeExact(result.knots[i].upper_slope_right_interval.upper) >=
                  upper_slope);
    }
  }

  const double query_w = -0.25;
  TubeViabilityInterval queried;
  ASSERT_TRUE(result.envelopeAt(query_w, queried));
  const std::size_t left = 1U;
  const ExactRational t = (DecodeExact(query_w) - DecodeExact(phases[left])) /
      (DecodeExact(phases[left + 1U]) - DecodeExact(phases[left]));
  const ExactRational expected_lower = exact_lower[left] +
      t * (exact_lower[left + 1U] - exact_lower[left]);
  const ExactRational expected_upper = exact_upper[left] +
      t * (exact_upper[left + 1U] - exact_upper[left]);
  EXPECT_TRUE(DecodeExact(queried.lower) >= expected_lower);
  EXPECT_TRUE(DecodeExact(queried.upper) <= expected_upper);
  const ExactRational produced_lower =
      DecodeExact(result.knots[left].reachable.lower) +
      t * (DecodeExact(result.knots[left + 1U].reachable.lower) -
           DecodeExact(result.knots[left].reachable.lower));
  const ExactRational produced_upper =
      DecodeExact(result.knots[left].reachable.upper) +
      t * (DecodeExact(result.knots[left + 1U].reachable.upper) -
           DecodeExact(result.knots[left].reachable.upper));
  EXPECT_TRUE(DecodeExact(queried.lower) >= produced_lower);
  EXPECT_TRUE(DecodeExact(queried.upper) <= produced_upper);

  TubeViabilityInterval near_query;
  ASSERT_TRUE(result.envelopeAt(std::nextafter(-0.5, 0.0), near_query));
}

TEST(TubeViabilityV2Test, LiveSuffixReuseKeepsProfileIdentityButChangesKAndBeta) {
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 0.5, 1.0, 2.0, 3.0, 4.0},
      {{-1.0, 1.0}, {-0.1, 0.1}, {-1.0, 1.0}, {-1.0, 1.0},
       {-1.0, 1.0}, {-1.0, 1.0}});
  TubeViabilityInput first_input = V2Input(0.0, 0.0, V2Policy(3.0, 1.0));
  TubeViabilityResult first;
  ASSERT_TRUE(TubeViability::evaluate(profile, first_input, first))
      << first.reason;
  ASSERT_TRUE(first.feasible);

  TubeViabilityInput suffix_input = V2Input(1.0, 0.0, V2Policy(3.0, 1.0));
  TubeViabilityResult suffix;
  ASSERT_TRUE(TubeViability::evaluate(profile, suffix_input, suffix))
      << suffix.reason;
  ASSERT_TRUE(suffix.feasible);
  EXPECT_EQ(first.profile_id, profile.profile_id);
  EXPECT_EQ(suffix.profile_id, profile.profile_id);
  EXPECT_TRUE(first.path_key == suffix.path_key);
  EXPECT_TRUE(first.configuration_key == suffix.configuration_key);
  EXPECT_TRUE(first.map_capture_key == suffix.map_capture_key);
  EXPECT_LT(first.b_pre, suffix.b_pre);
  EXPECT_NE(first.beta, suffix.beta);
}

TEST(TubeViabilityV2Test, InsufficientFullHorizonIsExplicitlyInfeasible) {
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 2.0}, {{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy(3.0, 1.0));
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.feasible);
  EXPECT_TRUE(result.preview_truncated_after);
  EXPECT_EQ(result.status, TubeViabilityStatus::PREVIEW_INFEASIBLE);
  EXPECT_TRUE(result.knots.empty());
}

TEST(TubeViabilityV2Test, ChangedLiveHorizonExposesNewBottleneck) {
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 2.0, 3.0},
      {{-1.0, 1.0}, {-1.0, 1.0}, {-0.1, 0.1}, {-1.0, 1.0}});
  TubeViabilityResult short_result;
  ASSERT_TRUE(TubeViability::evaluate(
      profile, V2Input(0.0, 0.0, V2Policy(1.0, 1.0)), short_result))
      << short_result.reason;
  ASSERT_TRUE(short_result.feasible);
  TubeViabilityResult long_result;
  ASSERT_TRUE(TubeViability::evaluate(
      profile, V2Input(1.0, 0.0, V2Policy(1.0, 1.0)), long_result))
      << long_result.reason;
  ASSERT_TRUE(long_result.feasible);
  EXPECT_GT(short_result.b_pre, long_result.b_pre);
  EXPECT_NE(short_result.beta, long_result.beta);
}

TEST(TubeViabilityV2Test, StrictQueriesHonorKnotSidesAndRejectMalformedQueries) {
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 2.0}, {{-0.5, 1.0}, {-1.0, 1.0}, {-0.5, 1.0}});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy(), 0.5);
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  ASSERT_TRUE(result.feasible);

  TubeViabilityRateInterval right_rate;
  ASSERT_TRUE(result.rateIntervalAt(1.0, result.knots[1].reachable.lower,
                                    1.0, right_rate));
  EXPECT_TRUE(right_rate.lower_boundary_active);
  EXPECT_NEAR(right_rate.lower_boundary_slope, 0.5, 1e-14);
  EXPECT_NEAR(result.knots[1].lower_slope_left, -0.5, 1e-14);
  EXPECT_NEAR(result.knots[1].lower_slope_right, 0.5, 1e-14);

  TubeViabilityResult too_small = result;
  too_small.upper_u_delta = std::nextafter(0.5, 0.0);
  EXPECT_FALSE(too_small.rateIntervalAt(
      1.0, too_small.knots[1].reachable.lower, 1.0, right_rate));

  TubeViabilityInterval interval;
  ASSERT_TRUE(result.envelopeAt(std::nextafter(1.0, 0.0), interval));
  ASSERT_TRUE(result.envelopeAt(std::nextafter(1.0, 2.0), interval));
  EXPECT_FALSE(result.envelopeAt(std::numeric_limits<double>::quiet_NaN(),
                                 interval));
  TubeViabilityRateInterval malformed_rate;
  EXPECT_FALSE(result.rateIntervalAt(
      1.0, 0.0, std::numeric_limits<double>::quiet_NaN(), malformed_rate));

  TubeViabilityResult malformed = result;
  malformed.knots.front().reachable.lower =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(malformed.envelopeAt(malformed.preview_start_w, interval));
  EXPECT_FALSE(malformed.rateIntervalAt(
      malformed.preview_start_w, 0.0, 1.0, malformed_rate));

  struct RoundingRestore {
    int value;
    ~RoundingRestore() { fesetround(value); }
  } rounding_restore{fegetround()};
  ASSERT_EQ(fesetround(FE_DOWNWARD), 0);
  EXPECT_FALSE(result.envelopeAt(0.5, interval));
  EXPECT_FALSE(result.rateIntervalAt(0.5, 0.0, 1.0, malformed_rate));
}

TEST(TubeViabilityV2Test, StrictEvaluationRejectsFtzAndDazModes) {
#if defined(__i386__) || defined(__x86_64__)
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 2.0}, {{-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy());
  TubeViabilityResult result;
  struct CsrRestore {
    unsigned int value;
    ~CsrRestore() { _mm_setcsr(value); }
  } restore{_mm_getcsr()};
  _mm_setcsr(restore.value & ~0x8040U);
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  for (const unsigned int mask : {0x8000U, 0x0040U, 0x8040U}) {
    _mm_setcsr((_mm_getcsr() & ~0x8040U) | mask);
    EXPECT_FALSE(TubeViability::evaluate(profile, input, result));
    EXPECT_EQ(result.status, TubeViabilityStatus::INVALID_INPUT);
  }
#else
  SUCCEED();
#endif
}

TEST(TubeViabilityV2Test, SignedTinyWidthDoesNotBecomeFalseSingleton) {
  // For a=2^-664 and b=nextafter(a,0), the exact positive width a+b is
  // halfway below 2*a, so nearest binary64 rounds it upward to 2*a.  The
  // product a*(-b) underflows to -0; sign-product shortcuts must not turn the
  // width subtraction into that optimistic singleton.
  const double a = std::ldexp(1.0, -664);
  const double b = std::nextafter(a, 0.0);
  TubeProfileV2 profile = MakeV2Profile(
      {0.0, 1.0, 2.0}, {{-b, a}, {-b, a}, {-b, a}});
  TubeViabilityInput input = V2Input(0.0, 0.0, V2Policy(), 0.0);
  TubeViabilityResult result;
  ASSERT_TRUE(TubeViability::evaluate(profile, input, result))
      << result.reason;
  ASSERT_TRUE(result.feasible);
  // The expected bound is derived from the exact dyadic construction above,
  // not from the rounded runtime expression a+b.
  EXPECT_DOUBLE_EQ(result.b_pre, std::nextafter(2.0 * a, 0.0));
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
