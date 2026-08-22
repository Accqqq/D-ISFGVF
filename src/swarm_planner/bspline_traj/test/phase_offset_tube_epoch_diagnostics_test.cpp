#include <gtest/gtest.h>

#include "bspline_race/integration/phase_offset_tube_epoch_diagnostics.h"

#include <cmath>
#include <limits>
#include <set>

namespace FLAG_Race {
namespace {

phase_offset_navigation::TubeProfile MakeProfile(bool obstacle_certified) {
  phase_offset_navigation::TubeProfile profile;
  profile.source = obstacle_certified ? phase_offset_navigation::TubeSource::ESDF
                                      : phase_offset_navigation::TubeSource::FIXED;
  profile.complete = true;
  profile.raw_complete = true;
  profile.filtered_complete = true;
  profile.obstacle_certified = obstacle_certified;
  profile.samples.resize(3U);
  return profile;
}

TubeEpochDiagnosticsInput MakeInput() {
  static phase_offset_navigation::TubeProfile candidate = MakeProfile(true);
  static phase_offset_navigation::TubeProfile active = MakeProfile(true);
  TubeEpochDiagnosticsInput input;
  input.source = phase_offset_navigation::TubeSource::ESDF;
  input.epoch.state = phase_offset_navigation::TubeEpochState::ROLLING;
  input.epoch.disposition = phase_offset_navigation::TubeInstallDisposition::INITIAL_INSTALL;
  input.epoch.reason = phase_offset_navigation::TubeEpochReason::NONE;
  input.epoch.current_safety_status = phase_offset_navigation::CurrentSafetyStatus::SAFE;
  input.epoch.candidate_sequence = 17U;
  input.epoch.active_tube_epoch = 4U;
  input.epoch.candidate_path_source_revision = 9U;
  input.epoch.active_path_source_revision = 9U;
  input.epoch.candidate_map_observation_sequence = 23U;
  input.epoch.active_map_observation_sequence = 23U;
  input.epoch.candidate_raw_complete = true;
  input.epoch.candidate_filtered_complete = true;
  input.epoch.candidate_complete = true;
  input.epoch.active_available = true;
  input.epoch.active_current_validation_valid = true;
  input.epoch.current_state_admissible = true;
  input.epoch.reference_clearance_sufficient = true;
  input.epoch.actual_clearance_sufficient = true;
  input.epoch.tracking_within_bound = true;
  input.epoch.forward_horizon_sufficient = true;
  input.epoch.tracking_error_norm = 0.03;
  input.epoch.tracking_error_bound = 0.15;
  input.epoch.reference_signed_distance = 1.2;
  input.epoch.actual_signed_distance = 1.1;
  input.epoch.required_reference_clearance = 0.70;
  input.epoch.required_actual_clearance = 0.55;
  input.epoch.certified_forward_w = 1.8;
  input.epoch.equivalent_refresh_count = 5U;
  input.epoch.install_count = 4U;
  input.epoch.reject_count = 2U;
  input.epoch.wait_count = 1U;
  input.runtime.mode = phase_offset_navigation::RuntimeExecutionMode::NORMAL;
  input.runtime.current_geometry_valid = true;
  input.runtime.current_bounds_valid = true;
  input.runtime.retained_delta_current_inside = true;
  input.runtime.tracking_within_bound = true;
  input.runtime.tracking_error_norm = 0.02;
  input.runtime.tracking_error_bound = 0.15;
  input.candidate_profile = &candidate;
  input.active_profile = &active;
  input.retained_delta = -0.04;
  input.active_display_certified = true;
  input.tube_update_due_this_cycle = true;
  input.control_selected = false;
  return input;
}

TEST(TubeEpochDiagnosticsTest, SchemaContainsOnlyFiftyLiveFields) {
  static_assert(kTubeEpochDiagnosticCount == 50U, "schema count changed");
  static_assert(kEpochControlSelected == 45U, "live index moved");
  const auto& names = tubeEpochDiagnosticFieldNames();
  ASSERT_EQ(names.size(), kTubeEpochDiagnosticCount);
  std::set<std::string> unique;
  for (const char* name : names) {
    ASSERT_NE(name, nullptr);
    EXPECT_TRUE(unique.insert(name).second);
  }
  EXPECT_EQ(std::string(names[kEpochMapObservationIsSnapshot]),
            "map_observation_is_snapshot");
}

TEST(TubeEpochDiagnosticsTest, MapsAuthoritativeEpochAndNonSnapshotObservation) {
  const auto values = makeTubeEpochDiagnostics(MakeInput());
  ASSERT_EQ(values.size(), kTubeEpochDiagnosticCount);
  EXPECT_DOUBLE_EQ(values[kEpochSchemaVersion], 3.0);
  EXPECT_DOUBLE_EQ(values[kEpochCandidateSequence], 17.0);
  EXPECT_DOUBLE_EQ(values[kEpochActiveTubeEpoch], 4.0);
  EXPECT_DOUBLE_EQ(values[kEpochCandidateMapObservationSequence], 23.0);
  EXPECT_DOUBLE_EQ(values[kEpochActiveMapObservationSequence], 23.0);
  EXPECT_DOUBLE_EQ(values[kEpochMapObservationIsSnapshot], 0.0);
  EXPECT_DOUBLE_EQ(values[kEpochCandidateSampleCount], 3.0);
  EXPECT_DOUBLE_EQ(values[kEpochActiveSampleCount], 3.0);
  EXPECT_DOUBLE_EQ(values[kEpochCertificateDenied], 0.0);
}

TEST(TubeEpochDiagnosticsTest, SnapshotIdentityUsesTheExistingSchemaField) {
  TubeEpochDiagnosticsInput input = MakeInput();
  input.epoch.map_observation_is_snapshot = true;
  const auto values = makeTubeEpochDiagnostics(input);
  EXPECT_DOUBLE_EQ(values[kEpochMapObservationIsSnapshot], 1.0);
  EXPECT_DOUBLE_EQ(values[kEpochCandidateMapObservationSequence], 23.0);
}

TEST(TubeEpochDiagnosticsTest, RuntimeCertificateAndGateSelectionStayIndependent) {
  TubeEpochDiagnosticsInput input = MakeInput();
  const auto values = makeTubeEpochDiagnostics(input);
  EXPECT_DOUBLE_EQ(values[kEpochActiveDisplayCertified], 1.0);
  EXPECT_DOUBLE_EQ(values[kEpochTubeUpdateDueThisCycle], 1.0);
  EXPECT_DOUBLE_EQ(values[kEpochControlSelected], 0.0);
  EXPECT_DOUBLE_EQ(values[kEpochRuntimeExecutionMode],
                   static_cast<int>(phase_offset_navigation::RuntimeExecutionMode::NORMAL));
}

TEST(TubeEpochDiagnosticsTest, PayloadIsFiniteAndDeterministicForUnavailableProfiles) {
  TubeEpochDiagnosticsInput input = MakeInput();
  input.candidate_profile = nullptr;
  input.active_profile = nullptr;
  input.epoch.reference_signed_distance = std::numeric_limits<double>::quiet_NaN();
  input.runtime.tracking_error_norm = std::numeric_limits<double>::infinity();
  const auto first = makeTubeEpochDiagnostics(input);
  const auto second = makeTubeEpochDiagnostics(input);
  EXPECT_EQ(first, second);
  for (double value : first) EXPECT_TRUE(std::isfinite(value));
  EXPECT_DOUBLE_EQ(first[kEpochCandidateSampleCount], 0.0);
  EXPECT_DOUBLE_EQ(first[kEpochActiveSampleCount], 0.0);
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
