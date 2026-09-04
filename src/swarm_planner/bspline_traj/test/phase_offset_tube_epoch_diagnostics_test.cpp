#include <gtest/gtest.h>

#include "bspline_race/integration/phase_offset_raw_candidate_diagnostics.h"
#include "bspline_race/integration/phase_offset_tube_epoch_diagnostics.h"

#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>

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

phase_offset_navigation::TubeRawSample MakeRawRepresentativeSample() {
  phase_offset_navigation::TubeRawSample sample;
  sample.w = 1.0;
  sample.p = Eigen::Vector3d(1.0, 2.0, 1.0);
  sample.N = Eigen::Vector3d(0.0, 2.0, 0.0);
  sample.complete = true;
  sample.c_plus_raw = 0.75;
  sample.c_minus_raw = 1.25;
  sample.effective_radius = 0.55;
  sample.obstacle_lower = -0.70;
  sample.obstacle_upper = 0.20;
  sample.curvature_lower = -3.0;
  sample.curvature_upper = 2.0;
  sample.environment_lower = -0.70;
  sample.environment_upper = 0.20;
  sample.environment_width = 0.90;
  sample.environment_interval_nonempty = true;
  sample.environment_contains_zero = true;
  sample.cross_section_reason =
      phase_offset_navigation::TubeCrossSectionReason::NONE;
  sample.positive_ray_termination =
      phase_offset_navigation::TubeRayTermination::OCCUPIED;
  sample.negative_ray_termination =
      phase_offset_navigation::TubeRayTermination::SEARCH_EXTENT;
  return sample;
}

phase_offset_navigation::TubeProfile MakeRawRepresentativeProfile() {
  phase_offset_navigation::TubeProfile profile;
  profile.source = phase_offset_navigation::TubeSource::ESDF;
  profile.raw_complete = true;
  profile.filtered_complete = true;
  profile.complete = true;
  profile.requested_preview_start_w = 0.8;
  profile.requested_preview_end_w = 2.2;
  profile.certified_segment_start_w = 0.8;
  profile.certified_segment_end_w = 2.0;
  profile.preview_start_w = profile.certified_segment_start_w;
  profile.preview_end_w = profile.certified_segment_end_w;
  profile.diagnostics.invalid_count = 0U;
  profile.diagnostics.unavailable_count = 2U;
  profile.diagnostics.out_of_map_count = 3U;
  profile.diagnostics.unknown_count = 4U;
  profile.diagnostics.occupied_count = 5U;
  profile.diagnostics.insufficient_clearance_count = 6U;
  profile.samples.push_back(MakeRawRepresentativeSample());
  return profile;
}

RawCandidateDiagnosticsInput MakeRawRepresentativeInput(
    const phase_offset_navigation::TubeProfile* profile) {
  RawCandidateDiagnosticsInput input;
  input.raw_source_configured = true;
  input.raw_storage_ready = true;
  input.raw_query_injected = true;
  input.tube_update_due = true;
  input.epoch_status.state =
      phase_offset_navigation::TubeEpochState::WAITING_FOR_CANDIDATE;
  input.epoch_status.reason =
      phase_offset_navigation::TubeEpochReason::CANDIDATE_INCOMPLETE;
  input.epoch_status.candidate_sequence = 11U;
  input.epoch_status.candidate_raw_complete = false;
  input.epoch_status.candidate_filtered_complete = false;
  input.epoch_status.candidate_complete = false;
  input.candidate_profile = profile;
  input.current_w = 1.0;
  input.current_path.w = 1.0;
  input.current_path.p = Eigen::Vector3d(1.0, 2.0, 1.0);
  input.actual_position = Eigen::Vector3d(9.0, 2.0, 1.0);
  input.raw_occupancy_query = [](const Eigen::Vector3d& point) {
    if (std::abs(point.x() - 9.0) <= 1e-12) {
      return phase_offset_navigation::DistanceStatus::OCCUPIED;
    }
    if (std::abs(point.y() - 2.05) <= 1e-12) {
      return phase_offset_navigation::DistanceStatus::UNKNOWN;
    }
    if (std::abs(point.y() - 1.95) <= 1e-12) {
      return phase_offset_navigation::DistanceStatus::OUT_OF_MAP;
    }
    return phase_offset_navigation::DistanceStatus::KNOWN_FREE;
  };
  input.raw_ray_step = 0.05;
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

TEST(TubeEpochDiagnosticsTest,
     RawAndEpochSchemasKeepCompleteNamesVersionsAndRepresentativeValues) {
  static_assert(kRawCandidateDiagnosticCount == 49U,
                "raw candidate schema count changed");
  static_assert(kTubeEpochDiagnosticCount == 74U,
                "tube epoch schema count changed");

  const std::array<const char*, kRawCandidateDiagnosticCount>
      expected_raw_names = {{
          "schema_version",
          "raw_source_configured",
          "raw_storage_ready",
          "raw_query_injected",
          "tube_update_due",
          "candidate_sequence",
          "epoch_state",
          "epoch_reason",
          "candidate_raw_complete",
          "candidate_filtered_complete",
          "candidate_complete",
          "candidate_sample_count",
          "candidate_invalid_count",
          "candidate_unavailable_count",
          "candidate_out_of_map_count",
          "candidate_unknown_count",
          "candidate_occupied_count",
          "candidate_insufficient_clearance_count",
          "requested_preview_start_w",
          "requested_preview_end_w",
          "certified_segment_start_w",
          "certified_segment_end_w",
          "certified_segment_truncated_before",
          "certified_segment_truncated_after",
          "first_truncated_w",
          "first_truncated_reason",
          "current_w",
          "current_sample_found",
          "current_sample_complete",
          "current_cross_section_reason",
          "current_positive_ray_termination",
          "current_negative_ray_termination",
          "current_base_raw_status",
          "actual_position_raw_status",
          "current_plus_step_raw_status",
          "current_minus_step_raw_status",
          "current_c_plus_raw",
          "current_c_minus_raw",
          "current_effective_radius",
          "current_obstacle_lower",
          "current_obstacle_upper",
          "current_curvature_lower",
          "current_curvature_upper",
          "current_environment_lower",
          "current_environment_upper",
          "current_environment_width",
          "current_contains_zero",
          "deprecated_current_contains_preferred_delta",
          "current_base_to_actual_norm"}};
  const auto& raw_names = rawCandidateDiagnosticFieldNames();
  ASSERT_EQ(raw_names.size(), expected_raw_names.size());
  for (std::size_t index = 0U; index < raw_names.size(); ++index) {
    EXPECT_STREQ(raw_names[index], expected_raw_names[index]);
  }

  const TubeProfile raw_profile = MakeRawRepresentativeProfile();
  const RawCandidateDiagnosticsInput raw_input =
      MakeRawRepresentativeInput(&raw_profile);
  const auto raw_values = makeRawCandidateDiagnostics(raw_input);
  const std::array<double, kRawCandidateDiagnosticCount> expected_raw_values = {{
      /* 0 schema */ 1.0,
      /* 1 configured */ 1.0,
      /* 2 storage */ 1.0,
      /* 3 query */ 1.0,
      /* 4 due */ 1.0,
      /* 5 sequence */ 11.0,
      /* 6 state */ 1.0,
      /* 7 reason */ 3.0,
      /* 8 raw complete */ 0.0,
      /* 9 filtered complete */ 0.0,
      /* 10 complete */ 0.0,
      /* 11 samples */ 1.0,
      /* 12 invalid */ 0.0,
      /* 13 unavailable */ 2.0,
      /* 14 out of map */ 3.0,
      /* 15 unknown */ 4.0,
      /* 16 occupied */ 5.0,
      /* 17 insufficient */ 6.0,
      /* 18 requested start */ 0.8,
      /* 19 requested end */ 2.2,
      /* 20 certified start */ 0.8,
      /* 21 certified end */ 2.0,
      /* 22 truncated before */ 0.0,
      /* 23 truncated after */ 0.0,
      /* 24 first truncated w */ 0.0,
      /* 25 first truncated reason */ 0.0,
      /* 26 current w */ 1.0,
      /* 27 current found */ 1.0,
      /* 28 current complete */ 1.0,
      /* 29 cross-section reason */ 0.0,
      /* 30 positive termination */ 1.0,
      /* 31 negative termination */ 0.0,
      /* 32 base raw status */ 3.0,
      /* 33 actual raw status */ 4.0,
      /* 34 plus-step status */ 2.0,
      /* 35 minus-step status */ 1.0,
      /* 36 c+ */ 0.75,
      /* 37 c- */ 1.25,
      /* 38 effective radius */ 0.55,
      /* 39 obstacle lower */ -0.70,
      /* 40 obstacle upper */ 0.20,
      /* 41 curvature lower */ -3.0,
      /* 42 curvature upper */ 2.0,
      /* 43 environment lower */ -0.70,
      /* 44 environment upper */ 0.20,
      /* 45 environment width */ 0.90,
      /* 46 contains zero */ 1.0,
      /* 47 deprecated preferred delta */ 0.0,
      /* 48 base-to-actual norm */ 8.0}};
  ASSERT_EQ(raw_values.size(), expected_raw_values.size());
  for (std::size_t index = 0U; index < raw_values.size(); ++index) {
    EXPECT_DOUBLE_EQ(raw_values[index], expected_raw_values[index]);
  }

  const auto epoch_values = makeTubeEpochDiagnostics(MakeInput());
  const std::array<double, kTubeEpochDiagnosticCount> expected_epoch_values = {{
      /* 0 schema */ 4.0,
      /* 1 source */ 2.0,
      /* 2 state */ 2.0,
      /* 3 disposition */ 1.0,
      /* 4 reason */ 0.0,
      /* 5 candidate sequence */ 17.0,
      /* 6 active epoch */ 4.0,
      /* 7 candidate path revision */ 9.0,
      /* 8 active path revision */ 9.0,
      /* 9 candidate map sequence */ 23.0,
      /* 10 active map sequence */ 23.0,
      /* 11 map snapshot */ 0.0,
      /* 12 candidate raw complete */ 1.0,
      /* 13 candidate filtered complete */ 1.0,
      /* 14 candidate complete */ 1.0,
      /* 15 active available */ 1.0,
      /* 16 active current validation */ 1.0,
      /* 17 current geometry */ 1.0,
      /* 18 current bounds */ 1.0,
      /* 19 retained delta inside */ 1.0,
      /* 20 state admissible */ 1.0,
      /* 21 safety status */ 1.0,
      /* 22 reference clearance */ 1.0,
      /* 23 actual clearance */ 1.0,
      /* 24 tracking bound */ 1.0,
      /* 25 forward horizon */ 1.0,
      /* 26 retained delta */ -0.04,
      /* 27 tracking norm */ 0.02,
      /* 28 tracking bound */ 0.15,
      /* 29 reference distance */ 1.2,
      /* 30 actual distance */ 1.1,
      /* 31 required reference */ 0.70,
      /* 32 required actual */ 0.55,
      /* 33 certified forward */ 1.8,
      /* 34 refresh count */ 5.0,
      /* 35 install count */ 4.0,
      /* 36 reject count */ 2.0,
      /* 37 wait count */ 1.0,
      /* 38 candidate samples */ 3.0,
      /* 39 active samples */ 3.0,
      /* 40 active profile complete */ 1.0,
      /* 41 active obstacle certified */ 1.0,
      /* 42 active display certified */ 1.0,
      /* 43 update due */ 1.0,
      /* 44 runtime mode */ 1.0,
      /* 45 control selected */ 0.0,
      /* 46 certificate denied */ 0.0,
      /* 47 transient blocked */ 0.0,
      /* 48 fatal invariant */ 0.0,
      /* 49 runtime failure reason */ 0.0,
      /* 50 surface outcome */ 0.0,
      /* 51 surface inconclusive reason */ 0.0,
      /* 52 surface truncation */ 0.0,
      /* 53 terminal w */ 0.0,
      /* 54 witness clearance */ 0.0,
      /* 55 witness exact */ 0.0,
      /* 56 midpoint cover */ 0.0,
      /* 57 normal cover */ 0.0,
      /* 58 delta slope cover */ 0.0,
      /* 59 v-span cover */ 0.0,
      /* 60 geometric cover */ 0.0,
      /* 61 support alignment */ 0.0,
      /* 62 numerical epsilon */ 0.0,
      /* 63 proof residual */ 0.0,
      /* 64 max depth */ 0.0,
      /* 65 query samples */ 0.0,
      /* 66 depth guard */ 0.0,
      /* 67 query budget */ 0.0,
      /* 68 split w */ 0.0,
      /* 69 split v */ 0.0,
      /* 70 split both */ 0.0,
      /* 71 zero contiguous */ 0.0,
      /* 72 zero start */ 0.0,
      /* 73 zero end */ 0.0}};
  ASSERT_EQ(epoch_values.size(), expected_epoch_values.size());
  for (std::size_t index = 0U; index < epoch_values.size(); ++index) {
    EXPECT_DOUBLE_EQ(epoch_values[index], expected_epoch_values[index]);
  }
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
  EXPECT_NE(log.find("schema=2 valid=1 build_sequence=101"),
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

  // Reconstruct the complete legacy record independently and compare it as
  // one contiguous prefix.  This proves every schema-1 key/value and its
  // relative order remain unchanged while the schema-2 operands append only
  // after proof_residual.
  std::ostringstream legacy;
  legacy << std::setprecision(17) << std::defaultfloat;
  legacy << "[PHASE_OFFSET][TUBE][FORWARD_EXCLUDED] schema=2 valid=1";
  legacy << " build_sequence=101 candidate_sequence=102 task_generation=103";
  legacy << " authority_session=104 source_revision=105 path_revision=106";
  legacy << " frame_revision=107 map_observation_sequence=108";
  legacy << " current_w=" << input.current_w;
  legacy << " certified_segment_end_w=" << input.certified_segment_end_w;
  legacy << " w0=" << evidence.w0 << " w1=" << evidence.w1;
  legacy << " v0=" << evidence.v0 << " v1=" << evidence.v1;
  legacy << " depth=7 outcome=1 reason=0";
  legacy << " clearance_query_attempted=1 clearance_status=3";
  legacy << " witness_clearance_valid=1 witness_clearance_certified=1"
         << " witness_clearance_exact=1 witness_clearance="
         << evidence.witness_clearance;
  legacy << " exact_d_c_valid=1 exact_d_c=" << evidence.exact_d_c;
  legacy << " requested_clearance_valid=1 requested_clearance="
         << evidence.requested_clearance;
  legacy << " cell_certificate_attempted=1 cell_certificate_complete=1"
         << " cell_certificate_revision_match=1";
  legacy << " query_budget_exhausted=0 max_depth_reached=0";
  legacy << " geometric_evidence_valid=1";
  legacy << " midpoint_position_cover=" << evidence.midpoint_position_cover;
  legacy << " normal_variation_cover=" << evidence.normal_variation_cover;
  legacy << " delta_slope_cover=" << evidence.delta_slope_cover;
  legacy << " v_span_cover=" << evidence.v_span_cover;
  legacy << " geometric_cover=" << evidence.geometric_cover;
  legacy << " support_alignment_bound=" << evidence.support_alignment_bound;
  legacy << " numerical_epsilon=" << evidence.numerical_epsilon;
  legacy << " allowable_cover=" << evidence.allowable_cover;
  legacy << " proof_residual=" << evidence.proof_residual;
  const std::string legacy_prefix = legacy.str();
  ASSERT_EQ(log.compare(0U, legacy_prefix.size(), legacy_prefix), 0);
  const std::size_t proof_end = log.find(" proof_residual=");
  const std::size_t schema2_append = log.find(" witness_valid=");
  ASSERT_NE(proof_end, std::string::npos);
  ASSERT_NE(schema2_append, std::string::npos);
  EXPECT_EQ(schema2_append, legacy_prefix.size());
  EXPECT_LT(proof_end, schema2_append);
}

TEST(TubeEpochDiagnosticsTest,
     ForwardExcludedStructuredLogCarriesObservedDomainOperands) {
  phase_offset_navigation::TubeSurfaceForwardExcludedEvidence evidence;
  evidence.valid = true;
  evidence.w0 = 1.0;
  evidence.w1 = 1.5;
  evidence.v0 = 0.0;
  evidence.v1 = 1.0;
  evidence.depth = 2;
  evidence.witness_valid = true;
  evidence.center_w = 1.2345678901234567;
  evidence.center_v = 0.5;
  evidence.witness_x = 1.0;
  evidence.witness_y = 1.0;
  evidence.witness_z = 1.0;
  evidence.requested_clearance_valid = true;
  evidence.requested_clearance = 1.5;
  evidence.map_observation_sequence_valid = true;
  evidence.map_observation_sequence = 108U;

  plan_env::CloudOccupancySnapshot snapshot;
  snapshot.valid = true;
  snapshot.observation_sequence = 108U;
  snapshot.observed_min = Eigen::Vector3d::Zero();
  snapshot.observed_max = Eigen::Vector3d::Constant(2.0);

  TubeSurfaceForwardExcludedLogInput input;
  input.map_observation_sequence = 108U;
  input.evidence = &evidence;
  input.snapshot = &snapshot;
  input.cloud_status.snapshot_available = true;
  input.cloud_status.snapshot_valid = true;
  input.cloud_status.usable = true;
  input.cloud_status.observation_sequence = 108U;

  const std::string log = formatTubeSurfaceForwardExcludedLog(input);
  EXPECT_NE(log.find("schema=2"), std::string::npos);
  EXPECT_NE(log.find("witness_valid=1 center_w=1.2345678901234567 center_v=0.5"),
            std::string::npos);
  EXPECT_NE(log.find("witness_x=1 witness_y=1 witness_z=1"),
            std::string::npos);
  EXPECT_NE(log.find("snapshot_available=1 snapshot_valid=1 snapshot_usable=1"),
            std::string::npos);
  EXPECT_NE(log.find("snapshot_observation_sequence_valid=1 snapshot_observation_sequence=108 snapshot_identity_match=1"),
            std::string::npos);
  EXPECT_NE(log.find("observed_bounds_valid=1 observed_min_x=0 observed_min_y=0 observed_min_z=0 observed_max_x=2 observed_max_y=2 observed_max_z=2"),
            std::string::npos);
  EXPECT_NE(log.find("requested_radius_valid=1 requested_radius=1.5 base_radius_valid=1"),
            std::string::npos);
  EXPECT_NE(log.find("base_radius=0.40000000000000002"), std::string::npos);
  EXPECT_NE(log.find("domain_predicates_valid=1 point_in_observed_box=1 requested_ball_in_observed_box=0 base_0p4_ball_in_observed_box=1"),
            std::string::npos);
  EXPECT_NE(log.find("margins_valid=1 x_minus_margin=1 x_plus_margin=1 y_minus_margin=1 y_plus_margin=1 z_minus_margin=1 z_plus_margin=1 min_observed_margin=1"),
            std::string::npos);
  EXPECT_NE(log.find("limiting_face_valid=1 limiting_axis=0 limiting_side=0 requested_ball_deficit_valid=1 requested_ball_deficit=0.5"),
            std::string::npos);
  const std::size_t proof = log.find(" proof_residual=");
  const std::size_t witness = log.find(" witness_valid=");
  const std::size_t derived = log.find(" snapshot_available=");
  ASSERT_NE(proof, std::string::npos);
  ASSERT_NE(witness, std::string::npos);
  ASSERT_NE(derived, std::string::npos);
  EXPECT_LT(proof, witness);
  EXPECT_LT(witness, derived);
}

TEST(TubeEpochDiagnosticsTest,
     ForwardExcludedStructuredLogSuppressesMismatchedSnapshotDomain) {
  phase_offset_navigation::TubeSurfaceForwardExcludedEvidence evidence;
  evidence.valid = true;
  evidence.witness_valid = true;
  evidence.center_w = 1.0;
  evidence.center_v = 0.5;
  evidence.witness_x = 1.0;
  evidence.witness_y = 1.0;
  evidence.witness_z = 1.0;
  evidence.requested_clearance_valid = true;
  evidence.requested_clearance = 0.5;
  evidence.map_observation_sequence_valid = true;
  evidence.map_observation_sequence = 108U;

  plan_env::CloudOccupancySnapshot snapshot;
  snapshot.valid = true;
  snapshot.observation_sequence = 108U;
  snapshot.observed_min = Eigen::Vector3d::Zero();
  snapshot.observed_max = Eigen::Vector3d::Constant(2.0);
  TubeSurfaceForwardExcludedLogInput input;
  input.map_observation_sequence = 108U;
  input.snapshot = &snapshot;
  input.cloud_status.snapshot_available = true;
  input.cloud_status.snapshot_valid = true;
  input.cloud_status.usable = true;
  input.cloud_status.observation_sequence = 109U;
  input.evidence = &evidence;

  const std::string log = formatTubeSurfaceForwardExcludedLog(input);
  EXPECT_NE(log.find("snapshot_observation_sequence_valid=1 snapshot_observation_sequence=108 snapshot_identity_match=0"),
            std::string::npos);
  EXPECT_NE(log.find("observed_bounds_valid=0 observed_min_x=0 observed_min_y=0 observed_min_z=0 observed_max_x=0 observed_max_y=0 observed_max_z=0"),
            std::string::npos);
  EXPECT_NE(log.find("domain_predicates_valid=0 point_in_observed_box=0 requested_ball_in_observed_box=0 base_0p4_ball_in_observed_box=0"),
            std::string::npos);
  EXPECT_NE(log.find("margins_valid=0"), std::string::npos);
  EXPECT_NE(log.find("limiting_face_valid=0 limiting_axis=-1 limiting_side=-1 requested_ball_deficit_valid=0 requested_ball_deficit=0"),
            std::string::npos);
}

TEST(TubeEpochDiagnosticsTest,
     ForwardExcludedStructuredLogDistinguishesPointAndBallDomainCases) {
  phase_offset_navigation::TubeSurfaceForwardExcludedEvidence evidence;
  evidence.valid = true;
  evidence.witness_valid = true;
  evidence.center_w = 1.0;
  evidence.center_v = 0.5;
  evidence.witness_x = 1.0;
  evidence.witness_y = 1.0;
  evidence.witness_z = 1.0;
  evidence.requested_clearance_valid = true;
  evidence.requested_clearance = 1.5;
  evidence.map_observation_sequence_valid = true;
  evidence.map_observation_sequence = 401U;

  plan_env::CloudOccupancySnapshot snapshot;
  snapshot.valid = true;
  snapshot.observation_sequence = 401U;
  snapshot.observed_min = Eigen::Vector3d::Zero();
  snapshot.observed_max = Eigen::Vector3d::Constant(2.0);
  TubeSurfaceForwardExcludedLogInput input;
  input.map_observation_sequence = 401U;
  input.snapshot = &snapshot;
  input.cloud_status.snapshot_available = true;
  input.cloud_status.snapshot_valid = true;
  input.cloud_status.usable = true;
  input.cloud_status.observation_sequence = 401U;
  input.evidence = &evidence;

  const std::string inside_point_outside_ball =
      formatTubeSurfaceForwardExcludedLog(input);
  EXPECT_NE(inside_point_outside_ball.find(
      "domain_predicates_valid=1 point_in_observed_box=1 requested_ball_in_observed_box=0 base_0p4_ball_in_observed_box=1"),
            std::string::npos);

  evidence.witness_x = 2.25;
  evidence.requested_clearance = 0.25;
  const std::string outside_point =
      formatTubeSurfaceForwardExcludedLog(input);
  EXPECT_NE(outside_point.find(
      "domain_predicates_valid=1 point_in_observed_box=0 requested_ball_in_observed_box=0 base_0p4_ball_in_observed_box=0"),
            std::string::npos);
  EXPECT_NE(outside_point.find("margins_valid=1"), std::string::npos);
}

TEST(TubeEpochDiagnosticsTest,
     ForwardExcludedStructuredLogUsesInclusiveObservedBoxArithmetic) {
  phase_offset_navigation::TubeSurfaceForwardExcludedEvidence evidence;
  evidence.valid = true;
  evidence.witness_valid = true;
  evidence.center_w = 0.75;
  evidence.center_v = 0.5;
  evidence.witness_x = 1.0;
  evidence.witness_y = 1.0;
  evidence.witness_z = 1.0;
  evidence.requested_clearance_valid = true;
  evidence.requested_clearance = 0.5;
  evidence.map_observation_sequence_valid = true;
  evidence.map_observation_sequence = 402U;

  plan_env::CloudOccupancySnapshot snapshot;
  snapshot.valid = true;
  snapshot.observation_sequence = 402U;
  snapshot.observed_min = Eigen::Vector3d::Constant(0.5);
  snapshot.observed_max = Eigen::Vector3d::Constant(1.5);
  TubeSurfaceForwardExcludedLogInput input;
  input.map_observation_sequence = 402U;
  input.snapshot = &snapshot;
  input.cloud_status.snapshot_available = true;
  input.cloud_status.snapshot_valid = true;
  input.cloud_status.usable = true;
  input.cloud_status.observation_sequence = 402U;
  input.evidence = &evidence;

  const std::string boundary = formatTubeSurfaceForwardExcludedLog(input);
  EXPECT_NE(boundary.find(
      "domain_predicates_valid=1 point_in_observed_box=1 requested_ball_in_observed_box=1 base_0p4_ball_in_observed_box=1"),
            std::string::npos);
  EXPECT_NE(boundary.find(
      "margins_valid=1 x_minus_margin=0.5 x_plus_margin=0.5 y_minus_margin=0.5 y_plus_margin=0.5 z_minus_margin=0.5 z_plus_margin=0.5 min_observed_margin=0.5"),
            std::string::npos);
  EXPECT_NE(boundary.find(
      "limiting_face_valid=1 limiting_axis=0 limiting_side=0 requested_ball_deficit_valid=1 requested_ball_deficit=0"),
            std::string::npos);

  const auto format17 = [](const char* key, const double value) {
    std::ostringstream stream;
    stream << std::setprecision(17) << std::defaultfloat << " " << key << "="
           << value;
    return stream.str();
  };
  evidence.witness_x = 1.2345678901234567;
  evidence.witness_y = 1.3456789012345678;
  evidence.witness_z = 1.4567890123456789;
  snapshot.observed_min = Eigen::Vector3d(
      0.12345678901234567, 0.23456789012345678, 0.34567890123456789);
  snapshot.observed_max = Eigen::Vector3d(
      2.1234567890123457, 2.2345678901234568, 2.3456789012345679);
  const std::string precise = formatTubeSurfaceForwardExcludedLog(input);
  EXPECT_NE(precise.find(format17("witness_x", evidence.witness_x)),
            std::string::npos);
  EXPECT_NE(precise.find(format17("witness_y", evidence.witness_y)),
            std::string::npos);
  EXPECT_NE(precise.find(format17("witness_z", evidence.witness_z)),
            std::string::npos);
  EXPECT_NE(precise.find(format17("observed_min_x", snapshot.observed_min.x())),
            std::string::npos);
  EXPECT_NE(precise.find(format17("observed_max_z", snapshot.observed_max.z())),
            std::string::npos);
}

TEST(TubeEpochDiagnosticsTest,
     ForwardExcludedStructuredLogInvalidatesOverflowedMarginGroup) {
  const double max_finite = std::numeric_limits<double>::max();
  phase_offset_navigation::TubeSurfaceForwardExcludedEvidence evidence;
  evidence.valid = true;
  evidence.witness_valid = true;
  evidence.center_w = 0.5;
  evidence.center_v = 0.5;
  evidence.witness_x = max_finite;
  evidence.witness_y = 0.0;
  evidence.witness_z = 0.0;
  evidence.requested_clearance_valid = true;
  evidence.requested_clearance = 0.5;
  evidence.map_observation_sequence_valid = true;
  evidence.map_observation_sequence = 501U;

  plan_env::CloudOccupancySnapshot snapshot;
  snapshot.valid = true;
  snapshot.observation_sequence = 501U;
  snapshot.observed_min = Eigen::Vector3d(-max_finite, -1.0, -1.0);
  snapshot.observed_max = Eigen::Vector3d(max_finite, 1.0, 1.0);

  TubeSurfaceForwardExcludedLogInput input;
  input.map_observation_sequence = 501U;
  input.evidence = &evidence;
  input.snapshot = &snapshot;
  input.cloud_status.snapshot_available = true;
  input.cloud_status.snapshot_valid = true;
  input.cloud_status.usable = true;
  input.cloud_status.observation_sequence = 501U;

  const std::string log = formatTubeSurfaceForwardExcludedLog(input);
  EXPECT_NE(log.find("domain_predicates_valid=1"), std::string::npos);
  EXPECT_NE(log.find(
      "margins_valid=0 x_minus_margin=0 x_plus_margin=0 y_minus_margin=0"
      " y_plus_margin=0 z_minus_margin=0 z_plus_margin=0"
      " min_observed_margin=0"), std::string::npos);
  EXPECT_NE(log.find(
      "limiting_face_valid=0 limiting_axis=-1 limiting_side=-1"
      " requested_ball_deficit_valid=0 requested_ball_deficit=0"),
            std::string::npos);
}

TEST(TubeEpochDiagnosticsTest,
     ForwardExcludedStructuredLogInvalidatesOverflowedRequestedDeficit) {
  const double max_finite = std::numeric_limits<double>::max();
  phase_offset_navigation::TubeSurfaceForwardExcludedEvidence evidence;
  evidence.valid = true;
  evidence.witness_valid = true;
  evidence.center_w = 0.5;
  evidence.center_v = 0.5;
  evidence.witness_x = 0.0;
  evidence.witness_y = 0.0;
  evidence.witness_z = 0.0;
  evidence.requested_clearance_valid = true;
  evidence.requested_clearance = max_finite;
  evidence.map_observation_sequence_valid = true;
  evidence.map_observation_sequence = 502U;

  plan_env::CloudOccupancySnapshot snapshot;
  snapshot.valid = true;
  snapshot.observation_sequence = 502U;
  snapshot.observed_min = Eigen::Vector3d(max_finite, 0.0, 0.0);
  snapshot.observed_max = Eigen::Vector3d(max_finite, 0.0, 0.0);

  TubeSurfaceForwardExcludedLogInput input;
  input.map_observation_sequence = 502U;
  input.evidence = &evidence;
  input.snapshot = &snapshot;
  input.cloud_status.snapshot_available = true;
  input.cloud_status.snapshot_valid = true;
  input.cloud_status.usable = true;
  input.cloud_status.observation_sequence = 502U;

  const std::string log = formatTubeSurfaceForwardExcludedLog(input);
  EXPECT_NE(log.find("domain_predicates_valid=1"), std::string::npos);
  EXPECT_NE(log.find("margins_valid=1"), std::string::npos);
  EXPECT_NE(log.find(
      "limiting_face_valid=1 limiting_axis=0 limiting_side=0"
      " requested_ball_deficit_valid=0 requested_ball_deficit=0"),
            std::string::npos);
}

TEST(TubeEpochDiagnosticsTest,
     ForwardExcludedStructuredLogInvalidDomainInputsUsePlaceholders) {
  phase_offset_navigation::TubeSurfaceForwardExcludedEvidence evidence;
  evidence.valid = true;
  evidence.witness_valid = true;
  evidence.center_w = 1.0;
  evidence.center_v = 0.5;
  evidence.witness_x = 1.0;
  evidence.witness_y = 1.0;
  evidence.witness_z = 1.0;
  evidence.requested_clearance_valid = true;
  evidence.requested_clearance = 0.5;
  evidence.map_observation_sequence_valid = true;
  evidence.map_observation_sequence = 403U;

  plan_env::CloudOccupancySnapshot snapshot;
  snapshot.valid = true;
  snapshot.observation_sequence = 403U;
  snapshot.observed_min = Eigen::Vector3d::Zero();
  snapshot.observed_max = Eigen::Vector3d::Constant(2.0);
  TubeSurfaceForwardExcludedLogInput input;
  input.map_observation_sequence = 403U;
  input.snapshot = &snapshot;
  input.cloud_status.snapshot_available = true;
  input.cloud_status.snapshot_valid = true;
  input.cloud_status.usable = true;
  input.cloud_status.observation_sequence = 403U;
  input.evidence = &evidence;

  const auto expect_suppressed = [](const std::string& log,
                                    const bool bounds_unavailable) {
    EXPECT_NE(log.find("domain_predicates_valid=0 point_in_observed_box=0 requested_ball_in_observed_box=0 base_0p4_ball_in_observed_box=0"),
              std::string::npos);
    EXPECT_NE(log.find("base_radius_valid=0 base_radius=0"), std::string::npos);
    if (bounds_unavailable) {
      EXPECT_NE(log.find("observed_bounds_valid=0 observed_min_x=0 observed_min_y=0 observed_min_z=0 observed_max_x=0 observed_max_y=0 observed_max_z=0"),
                std::string::npos);
    } else {
      EXPECT_NE(log.find("observed_bounds_valid=1"), std::string::npos);
    }
    EXPECT_NE(log.find("margins_valid=0 x_minus_margin=0 x_plus_margin=0 y_minus_margin=0 y_plus_margin=0 z_minus_margin=0 z_plus_margin=0 min_observed_margin=0"),
              std::string::npos);
    EXPECT_NE(log.find("limiting_face_valid=0 limiting_axis=-1 limiting_side=-1 requested_ball_deficit_valid=0 requested_ball_deficit=0"),
              std::string::npos);
  };

  evidence.witness_valid = false;
  expect_suppressed(formatTubeSurfaceForwardExcludedLog(input), false);

  evidence.witness_valid = true;
  input.snapshot = nullptr;
  input.cloud_status.snapshot_available = false;
  input.cloud_status.snapshot_valid = false;
  input.cloud_status.usable = false;
  input.cloud_status.observation_sequence = 0U;
  expect_suppressed(formatTubeSurfaceForwardExcludedLog(input), true);

  input.snapshot = &snapshot;
  input.cloud_status.snapshot_available = true;
  input.cloud_status.snapshot_valid = true;
  input.cloud_status.usable = true;
  input.cloud_status.observation_sequence = 403U;
  snapshot.observed_min = Eigen::Vector3d(2.0, 0.0, 0.0);
  snapshot.observed_max = Eigen::Vector3d(1.0, 2.0, 2.0);
  expect_suppressed(formatTubeSurfaceForwardExcludedLog(input), true);

  snapshot.observed_min = Eigen::Vector3d::Zero();
  snapshot.observed_max = Eigen::Vector3d::Constant(2.0);
  input.cloud_status.observation_sequence = 404U;
  expect_suppressed(formatTubeSurfaceForwardExcludedLog(input), true);
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
  EXPECT_NE(log.find("schema=2 valid=0 build_sequence=201"),
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
