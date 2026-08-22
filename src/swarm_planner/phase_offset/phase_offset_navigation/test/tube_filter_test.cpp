#include <gtest/gtest.h>

#include "phase_offset_navigation/tube_filter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace phase_offset_navigation {
namespace {

TubeProfile MakeRawProfile(
    const std::vector<std::pair<double, double>>& bounds) {
  TubeProfile profile;
  profile.source = TubeSource::FIXED;
  profile.raw_complete = true;
  profile.preview_start_w = 0.0;
  profile.preview_end_w = bounds.empty()
      ? 0.0 : static_cast<double>(bounds.size() - 1U);
  profile.requested_preview_start_w = profile.preview_start_w;
  profile.requested_preview_end_w = profile.preview_end_w;
  profile.certified_segment_start_w = profile.preview_start_w;
  profile.certified_segment_end_w = profile.preview_end_w;
  for (std::size_t index = 0U; index < bounds.size(); ++index) {
    TubeRawSample sample;
    sample.w = static_cast<double>(index);
    sample.raw_lower = bounds[index].first;
    sample.raw_upper = bounds[index].second;
    sample.complete = std::isfinite(sample.raw_lower) &&
        std::isfinite(sample.raw_upper) &&
        sample.raw_lower <= sample.raw_upper;
    profile.samples.push_back(sample);
  }
  return profile;
}

void ExpectRawSubsetEquality(const TubeProfile& profile) {
  ASSERT_EQ(profile.samples.size(), profile.raw_build_samples.empty()
      ? profile.samples.size() : profile.raw_build_samples.size());
  for (const TubeRawSample& sample : profile.samples) {
    EXPECT_DOUBLE_EQ(sample.filtered_lower, sample.raw_lower);
    EXPECT_DOUBLE_EQ(sample.filtered_upper, sample.raw_upper);
    EXPECT_LE(sample.filtered_lower, sample.filtered_upper + 1e-12);
  }
}

TEST(TubeFilterTest, HighSlopeRetainsEveryRawKnotAndUsesLocalSlope) {
  TubeFilter filter;
  TubeProfile profile = MakeRawProfile({
      {-0.20, 0.20}, {-0.20, 1.40}, {-0.20, 0.20}});
  ASSERT_TRUE(filter.filter(profile, 0.0));
  ASSERT_EQ(profile.samples.size(), 3U);
  ExpectRawSubsetEquality(profile);
  EXPECT_FALSE(profile.certified_segment_truncated_before);
  EXPECT_FALSE(profile.certified_segment_truncated_after);
  EXPECT_EQ(profile.first_truncated_reason, TubeStopReason::NONE);
  EXPECT_NEAR(profile.samples[0].upper_w, 1.20, 1e-12);
  EXPECT_NEAR(profile.samples[1].upper_w, -1.20, 1e-12);
  TubeBounds query;
  ASSERT_TRUE(TubeFilter::query(profile, 0.5, query));
  EXPECT_NEAR(query.upper, 0.80, 1e-12);
  EXPECT_NEAR(query.upper_w, 1.20, 1e-12);
}

TEST(TubeFilterTest, DistantBottleneckDoesNotContractEarlierCells) {
  TubeFilter filter;
  TubeProfile profile = MakeRawProfile({
      {-0.80, 0.80}, {-0.80, 0.80}, {-0.12, 0.12}, {-0.80, 0.80}});
  ASSERT_TRUE(filter.filter(profile, 0.0));
  ExpectRawSubsetEquality(profile);
  TubeBounds early;
  TubeBounds narrow;
  ASSERT_TRUE(TubeFilter::query(profile, 0.5, early));
  ASSERT_TRUE(TubeFilter::query(profile, 2.0, narrow));
  EXPECT_NEAR(early.lower, -0.80, 1e-12);
  EXPECT_NEAR(early.upper, 0.80, 1e-12);
  EXPECT_NEAR(narrow.lower, -0.12, 1e-12);
  EXPECT_NEAR(narrow.upper, 0.12, 1e-12);
}

TEST(TubeFilterTest, InteriorQueriesUseRightDerivativeAndEndUsesLeftDerivative) {
  TubeFilter filter;
  TubeProfile profile = MakeRawProfile({
      {-1.0, 0.0}, {-0.5, 1.0}, {-0.25, 0.25}});
  ASSERT_TRUE(filter.filter(profile, 0.0));
  TubeBounds left;
  TubeBounds at_knot;
  TubeBounds right;
  TubeBounds end;
  ASSERT_TRUE(TubeFilter::query(profile, 1.0 - 1e-6, left));
  ASSERT_TRUE(TubeFilter::query(profile, 1.0, at_knot));
  ASSERT_TRUE(TubeFilter::query(profile, 1.0 + 1e-6, right));
  ASSERT_TRUE(TubeFilter::query(profile, 2.0, end));
  EXPECT_NEAR(left.lower_w, 0.50, 1e-12);
  EXPECT_NEAR(left.upper_w, 1.00, 1e-12);
  EXPECT_NEAR(right.lower_w, 0.25, 1e-12);
  EXPECT_NEAR(right.upper_w, -0.75, 1e-12);
  EXPECT_NEAR(at_knot.lower_w, right.lower_w, 1e-12);
  EXPECT_NEAR(at_knot.upper_w, right.upper_w, 1e-12);
  EXPECT_NEAR(end.lower_w, right.lower_w, 1e-12);
  EXPECT_NEAR(end.upper_w, right.upper_w, 1e-12);
}

TEST(TubeFilterTest, BoundarySlopeCompatibilityFieldNeverChangesGeometry) {
  const std::vector<double> compatibility_values = {0.80, 1e-12, 0.0, -1.0};
  TubeProfile reference = MakeRawProfile({
      {-0.60, 0.70}, {-0.20, 1.80}, {-0.40, 0.30}, {-0.10, 0.60}});
  TubeFilter reference_filter;
  ASSERT_TRUE(reference_filter.filter(reference, 0.0));
  for (const double value : compatibility_values) {
    TubeFilterConfig config;
    config.boundary_slope_max = value;
    TubeFilter filter(config);
    EXPECT_TRUE(filter.configurationValid());
    TubeProfile candidate = MakeRawProfile({
        {-0.60, 0.70}, {-0.20, 1.80}, {-0.40, 0.30}, {-0.10, 0.60}});
    ASSERT_TRUE(filter.filter(candidate, 0.0));
    ASSERT_EQ(candidate.samples.size(), reference.samples.size());
    for (std::size_t index = 0U; index < candidate.samples.size(); ++index) {
      EXPECT_DOUBLE_EQ(candidate.samples[index].filtered_lower,
                       reference.samples[index].filtered_lower);
      EXPECT_DOUBLE_EQ(candidate.samples[index].filtered_upper,
                       reference.samples[index].filtered_upper);
      EXPECT_DOUBLE_EQ(candidate.samples[index].lower_w,
                       reference.samples[index].lower_w);
      EXPECT_DOUBLE_EQ(candidate.samples[index].upper_w,
                       reference.samples[index].upper_w);
    }
    EXPECT_FALSE(candidate.certified_segment_truncated_after);
    EXPECT_EQ(candidate.first_truncated_reason, TubeStopReason::NONE);
  }
}

TEST(TubeFilterTest, BuilderTruncationProvenanceIsPreservedVerbatim) {
  TubeFilter filter;
  TubeProfile profile = MakeRawProfile({{-0.2, 0.2}, {-0.1, 0.3}, {-0.1, 0.3}});
  profile.certified_segment_truncated_before = true;
  profile.certified_segment_truncated_after = false;
  profile.first_truncated_w = -1.0;
  profile.first_truncated_reason = TubeStopReason::UNKNOWN;
  ASSERT_TRUE(filter.filter(profile, 0.0));
  EXPECT_TRUE(profile.certified_segment_truncated_before);
  EXPECT_FALSE(profile.certified_segment_truncated_after);
  EXPECT_DOUBLE_EQ(profile.first_truncated_w, -1.0);
  EXPECT_EQ(profile.first_truncated_reason, TubeStopReason::UNKNOWN);
  ExpectRawSubsetEquality(profile);
}

TEST(TubeFilterTest, DenseQueriesAreRawEndpointPwlAndNeverInvertBounds) {
  TubeFilter filter;
  TubeProfile profile = MakeRawProfile({
      {-0.60, 0.70}, {-0.20, 0.40}, {-0.40, 0.30}, {-0.10, 0.60}});
  ASSERT_TRUE(filter.filter(profile, 0.0));
  for (int index = 0; index <= 600; ++index) {
    const double w = 3.0 * static_cast<double>(index) / 600.0;
    TubeBounds bounds;
    ASSERT_TRUE(TubeFilter::query(profile, w, bounds));
    EXPECT_LE(bounds.lower, bounds.upper + 1e-12);
    const std::size_t cell = std::min<std::size_t>(
        2U, static_cast<std::size_t>(std::floor(std::min(2.999999, w))));
    const double s = w - static_cast<double>(cell);
    const TubeRawSample& first = profile.samples[cell];
    const TubeRawSample& second = profile.samples[cell + 1U];
    const double expected_lower = first.raw_lower +
        s * (second.raw_lower - first.raw_lower);
    const double expected_upper = first.raw_upper +
        s * (second.raw_upper - first.raw_upper);
    EXPECT_NEAR(bounds.lower, expected_lower, 1e-10);
    EXPECT_NEAR(bounds.upper, expected_upper, 1e-10);
  }
}

TEST(TubeFilterTest, InvalidInputsFailClosed) {
  TubeFilter filter;
  TubeProfile empty = MakeRawProfile({});
  EXPECT_FALSE(filter.filter(empty, 0.0));

  TubeProfile single = MakeRawProfile({{-0.2, 0.2}});
  EXPECT_FALSE(filter.filter(single, 0.0));

  TubeProfile incomplete = MakeRawProfile({{-0.2, 0.2}, {-0.2, 0.2}});
  incomplete.raw_complete = false;
  EXPECT_FALSE(filter.filter(incomplete, 0.0));

  TubeProfile missing_anchor = MakeRawProfile({{-0.2, 0.2}, {-0.2, 0.2}});
  EXPECT_FALSE(filter.filter(missing_anchor, 0.5));

  TubeProfile nonfinite_current = MakeRawProfile({{-0.2, 0.2}, {-0.2, 0.2}});
  EXPECT_FALSE(filter.filter(
      nonfinite_current, std::numeric_limits<double>::quiet_NaN()));

  TubeProfile incomplete_sample = MakeRawProfile({{-0.2, 0.2}, {-0.2, 0.2}});
  incomplete_sample.samples[1].complete = false;
  EXPECT_FALSE(filter.filter(incomplete_sample, 0.0));

  TubeProfile unordered = MakeRawProfile({{-0.2, 0.2}, {-0.2, 0.2}});
  unordered.samples[1].w = unordered.samples[0].w;
  EXPECT_FALSE(filter.filter(unordered, 0.0));

  TubeProfile decreasing = MakeRawProfile({{-0.2, 0.2}, {-0.2, 0.2},
                                           {-0.2, 0.2}});
  decreasing.samples[1].w = -0.5;
  EXPECT_FALSE(filter.filter(decreasing, 0.0));

  TubeProfile nonfinite_w = MakeRawProfile({{-0.2, 0.2}, {-0.2, 0.2}});
  nonfinite_w.samples[1].w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(filter.filter(nonfinite_w, 0.0));

  TubeProfile nonfinite_bound = MakeRawProfile({{-0.2, 0.2}, {-0.2, 0.2}});
  nonfinite_bound.samples[1].raw_upper =
      std::numeric_limits<double>::infinity();
  nonfinite_bound.samples[1].complete = false;
  EXPECT_FALSE(filter.filter(nonfinite_bound, 0.0));

  TubeProfile reversed = MakeRawProfile({{0.2, -0.2}, {0.2, -0.2}});
  EXPECT_FALSE(filter.filter(reversed, 0.0));
}

TEST(TubeFilterTest, ZeroConnectedAndZeroOnlyIntervalsRemainRawFacts) {
  TubeFilter filter;
  TubeProfile connected = MakeRawProfile({
      {-0.02, 0.30}, {-0.40, 0.01}, {-0.01, 0.20}, {-0.25, 0.02}});
  ASSERT_TRUE(filter.filter(connected, 0.0));
  for (const TubeRawSample& sample : connected.samples) {
    EXPECT_TRUE(sample.filtered_contains_zero);
  }

  TubeProfile zero_only = MakeRawProfile({{0.0, 0.0}, {0.0, 0.0},
                                          {0.0, 0.0}});
  ASSERT_TRUE(filter.filter(zero_only, 0.0));
  for (const TubeRawSample& sample : zero_only.samples) {
    EXPECT_TRUE(sample.filtered_contains_zero);
    EXPECT_DOUBLE_EQ(sample.filtered_lower, 0.0);
    EXPECT_DOUBLE_EQ(sample.filtered_upper, 0.0);
  }
}

TEST(TubeFilterTest, QueryRoundoffIsClampedButOutsideDomainRejected) {
  TubeFilter filter;
  TubeProfile profile = MakeRawProfile({{-0.3, 0.3}, {-0.2, 0.2}});
  ASSERT_TRUE(filter.filter(profile, 0.0));
  TubeBounds bounds;
  ASSERT_TRUE(TubeFilter::query(profile, -5e-11, bounds));
  EXPECT_DOUBLE_EQ(bounds.query_w, profile.preview_start_w);
  ASSERT_TRUE(TubeFilter::query(profile, 1.0 + 5e-11, bounds));
  EXPECT_DOUBLE_EQ(bounds.query_w, profile.preview_end_w);
  EXPECT_FALSE(TubeFilter::query(profile, -2e-10, bounds));
  EXPECT_FALSE(TubeFilter::query(profile, 1.0 + 2e-10, bounds));
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
