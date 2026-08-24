#include <gtest/gtest.h>

#include "phase_offset_navigation/preview_feasibility.h"

namespace phase_offset_navigation {
namespace {

TubeProfile Profile() {
  TubeProfile profile;
  profile.complete = true;
  profile.filtered_complete = true;
  profile.preview_start_w = 0.0;
  profile.preview_end_w = 2.0;
  for (int i = 0; i <= 4; ++i) {
    TubeRawSample sample;
    sample.w = 0.5 * i;
    sample.filtered_lower = -0.2;
    sample.filtered_upper = 0.2;
    sample.complete = true;
    profile.samples.push_back(sample);
  }
  return profile;
}

TEST(PreviewFeasibilityTest, DoesNotMutateProfileAndReportsOverlap) {
  TubeProfile profile = Profile();
  PreviewFeasibilityInput input;
  input.profile = &profile;
  input.current_w = 0.0;
  input.current_delta = 0.0;
  input.target_w = 1.0;
  input.target_delta = 0.1;
  input.dt = 0.1;
  input.horizon = 2.0;
  input.max_phase_rate = 1.0;
  input.max_delta_rate = 1.0;
  input.max_delta_slew = 1.0;
  PreviewFeasibilityResult result;
  ASSERT_TRUE(PreviewFeasibility::evaluate(input, result));
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.target_overlap);
  EXPECT_EQ(result.status, PreviewStatus::TARGET_OVERLAP);
  EXPECT_EQ(profile.samples.size(), 5U);
}

TEST(PreviewFeasibilityTest, StaleAndUnsafeAreObservationalStatuses) {
  TubeProfile profile = Profile();
  PreviewFeasibilityInput input;
  input.profile = &profile;
  input.current_w = 0.0;
  input.current_delta = 0.0;
  input.target_w = 1.0;
  input.dt = 0.1;
  input.horizon = 2.0;
  input.max_phase_rate = 1.0;
  input.max_delta_rate = 1.0;
  input.expected_path_revision = 4U;
  input.path_revision = 3U;
  PreviewFeasibilityResult result;
  EXPECT_FALSE(PreviewFeasibility::evaluate(input, result));
  EXPECT_EQ(result.status, PreviewStatus::STALE);
  input.expected_path_revision = 0U;
  input.current_state_safe = false;
  EXPECT_FALSE(PreviewFeasibility::evaluate(input, result));
  EXPECT_EQ(result.status, PreviewStatus::CURRENT_STATE_UNSAFE);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
