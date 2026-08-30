#include <gtest/gtest.h>

#include "phase_offset_navigation/tube_viability.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace phase_offset_navigation {
namespace {

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

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
