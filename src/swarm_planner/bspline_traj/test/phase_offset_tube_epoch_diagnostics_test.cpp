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

TEST(TubeEpochDiagnosticsTest, SchemaAppendsSurfaceFieldsWithoutMovingLegacyFields) {
  static_assert(kTubeEpochDiagnosticCount == 74U, "schema count changed");
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
  EXPECT_DOUBLE_EQ(values[kEpochSchemaVersion], 4.0);
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

TEST(TubeEpochDiagnosticsTest, ForwardExcludedStructuredLogMapsEveryField) {
  phase_offset_navigation::TubeSurfaceForwardExcludedEvidence evidence;
  evidence.valid = true;
  evidence.w0 = 1.2345678901234567;
  evidence.w1 = 1.3456789012345678;
  evidence.v0 = 0.125;
  evidence.v1 = 0.875;
  evidence.depth = 7;
  evidence.outcome = phase_offset_navigation::TubeSurfaceOutcome::CONTRACT_UNSAFE;
  evidence.inconclusive_reason =
      phase_offset_navigation::TubeSurfaceInconclusiveReason::NONE;
  evidence.clearance_query_attempted = true;
  evidence.clearance_status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
  evidence.witness_clearance_valid = true;
  evidence.witness_clearance_certified = true;
  evidence.witness_clearance_exact = true;
  evidence.witness_clearance = 0.98765432101234567;
  evidence.exact_d_c_valid = true;
  evidence.exact_d_c = evidence.witness_clearance;
  evidence.requested_clearance_valid = true;
  evidence.requested_clearance = 0.87654321098765432;
  evidence.cell_certificate_attempted = true;
  evidence.cell_certificate_complete = true;
  evidence.cell_certificate_revision_match = true;
  evidence.query_budget_exhausted = false;
  evidence.max_depth_reached = false;
  evidence.geometric_evidence_valid = true;
  evidence.midpoint_position_cover = 0.11;
  evidence.normal_variation_cover = 0.22;
  evidence.delta_slope_cover = 0.33;
  evidence.v_span_cover = 0.44;
  evidence.geometric_cover = 0.55;
  evidence.support_alignment_bound = 0.0;
  evidence.numerical_epsilon = 1e-10;
  evidence.allowable_cover = 0.66;
  evidence.proof_residual = 0.77;

  TubeSurfaceForwardExcludedLogInput input;
  input.build_sequence = 101U;
  input.candidate_sequence = 102U;
  input.task_generation = 103U;
  input.authority_session = 104U;
  input.source_revision = 105U;
  input.path_revision = 106U;
  input.frame_revision = 107U;
  input.map_observation_sequence = 108U;
  input.current_w = 1.1122334455667789;
  input.certified_segment_end_w = 1.2233445566778899;
  input.evidence = &evidence;
  const std::string log = formatTubeSurfaceForwardExcludedLog(input);
  EXPECT_NE(log.find("[PHASE_OFFSET][TUBE][FORWARD_EXCLUDED]"),
            std::string::npos);
  EXPECT_NE(log.find("schema=1 valid=1 build_sequence=101"),
            std::string::npos);
  EXPECT_NE(log.find("candidate_sequence=102 task_generation=103"),
            std::string::npos);
  EXPECT_NE(log.find("authority_session=104 source_revision=105"),
            std::string::npos);
  EXPECT_NE(log.find("path_revision=106 frame_revision=107 map_observation_sequence=108"),
            std::string::npos);
  EXPECT_NE(log.find("depth=7 outcome=1 reason=0"), std::string::npos);
  EXPECT_NE(log.find("clearance_query_attempted=1 clearance_status=3"),
            std::string::npos);
  EXPECT_NE(log.find("witness_clearance_valid=1 witness_clearance_certified=1 witness_clearance_exact=1"),
            std::string::npos);
  EXPECT_NE(log.find("exact_d_c_valid=1"), std::string::npos);
  EXPECT_NE(log.find("requested_clearance_valid=1"), std::string::npos);
  EXPECT_NE(log.find("cell_certificate_attempted=1 cell_certificate_complete=1 cell_certificate_revision_match=1"),
            std::string::npos);
  EXPECT_NE(log.find("geometric_evidence_valid=1 midpoint_position_cover=0.11"),
            std::string::npos);
  EXPECT_NE(log.find("proof_residual=0.77000000000000002"),
            std::string::npos);
}

TEST(TubeEpochDiagnosticsTest, ForwardExcludedStructuredLogUsesUnavailableSentinels) {
  phase_offset_navigation::TubeSurfaceForwardExcludedEvidence evidence;
  const TubeSurfaceForwardExcludedLogInput input = [&evidence]() {
    TubeSurfaceForwardExcludedLogInput value;
    value.build_sequence = 201U;
    value.evidence = &evidence;
    return value;
  }();
  const std::string log = formatTubeSurfaceForwardExcludedLog(input);
  EXPECT_NE(log.find("schema=1 valid=0 build_sequence=201"),
            std::string::npos);
  EXPECT_NE(log.find("w0=0 w1=0 v0=0 v1=0 depth=-1 outcome=-1 reason=-1"),
            std::string::npos);
  EXPECT_NE(log.find("clearance_query_attempted=0 clearance_status=-1"),
            std::string::npos);
  EXPECT_NE(log.find("witness_clearance_valid=0 witness_clearance_certified=0 witness_clearance_exact=0 witness_clearance=0"),
            std::string::npos);
  EXPECT_NE(log.find("exact_d_c_valid=0 exact_d_c=0 requested_clearance_valid=0 requested_clearance=0"),
            std::string::npos);
  EXPECT_NE(log.find("cell_certificate_attempted=0 cell_certificate_complete=0 cell_certificate_revision_match=0"),
            std::string::npos);
  EXPECT_NE(log.find("geometric_evidence_valid=0"), std::string::npos);
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
