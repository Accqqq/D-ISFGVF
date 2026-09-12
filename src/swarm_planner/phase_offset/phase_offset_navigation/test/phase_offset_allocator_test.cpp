#include <gtest/gtest.h>

#include "phase_offset_navigation/phase_offset_allocator.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace phase_offset_navigation {
namespace {

TubeProfile MakeProfile(
    const std::vector<std::pair<double, double>>& bounds,
    const std::uint64_t path_revision = 11U,
    const std::uint64_t frame_revision = 12U,
    const std::uint64_t profile_revision = 13U) {
  TubeProfile profile;
  profile.complete = true;
  profile.filtered_complete = true;
  profile.path_revision = path_revision;
  profile.frame_revision = frame_revision;
  profile.profile_revision = profile_revision;
  profile.source_revision = 14U;
  profile.tube_revision = 15U;
  profile.map_revision = 16U;
  profile.obstacle_contract_id = "test-obstacle-contract";
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

NormalPreviewResult MakePreview(const TubeProfile& profile,
                                const double current_delta = 0.0) {
  NormalPreviewInput input;
  input.profile = &profile;
  input.current_w = 0.0;
  input.current_delta = current_delta;
  input.upper_u_delta = 1.0;
  input.policy.preview_horizon_w = profile.preview_end_w;
  input.policy.sample_spacing_w = 1.0;
  input.policy.lower_nu = 0.5;
  input.policy.upper_nu = 2.0;
  input.policy.b_tight = 0.1;
  input.policy.b_open = 0.9;
  input.policy.policy_revision = 1U;
  input.policy.configuration_identity = 1U;
  input.policy.configuration_id = "test-normal-preview-w";
  input.path_revision = profile.path_revision;
  input.frame_revision = profile.frame_revision;
  input.profile_revision = profile.profile_revision;
  input.expected_path_revision = profile.path_revision;
  input.expected_frame_revision = profile.frame_revision;
  input.expected_profile_revision = profile.profile_revision;

  NormalPreviewResult result;
  EXPECT_TRUE(NormalPreview::evaluate(input, result));
  EXPECT_EQ(result.status, TubeViabilityStatus::FEASIBLE);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.feasible);
  EXPECT_TRUE(result.current_rate_interval.valid);
  return result;
}

SectionTubeProfile MakeSectionProfile() {
  SectionTubeProfile profile;
  profile.valid_start = 0.0;
  profile.valid_end = 2.0;
  profile.status = SectionTubeStatus::COMPLETE;
  profile.usable = true;
  profile.complete = true;
  const double knots[] = {0.0, 1.0, 2.0};
  const double lowers[] = {-1.0, -0.8, -0.7};
  const double uppers[] = {1.0, 0.8, 0.7};
  for (std::size_t i = 0U; i < 3U; ++i) {
    SectionTubeKnot knot;
    knot.w = knots[i];
    knot.lower = lowers[i];
    knot.upper = uppers[i];
    profile.knots.push_back(knot);
  }
  return profile;
}

NormalPreviewResult MakeSectionPreview(const SectionTubeProfile& profile,
                                       const double delta = 0.0) {
  NormalPreviewInput input;
  input.current_w = 0.0;
  input.current_delta = delta;
  input.upper_u_delta = 1.0;
  input.policy.preview_horizon_w = 2.0;
  input.policy.sample_spacing_w = 1.0;
  input.policy.lower_nu = 0.5;
  input.policy.upper_nu = 2.0;
  input.policy.b_tight = 0.1;
  input.policy.b_open = 0.9;
  input.path_revision = 71U;
  input.frame_revision = 72U;
  input.expected_path_revision = 71U;
  input.expected_frame_revision = 72U;
  input.max_work = 100000U;
  NormalPreviewResult result;
  EXPECT_TRUE(NormalPreview::evaluate(profile, input, result))
      << result.reason;
  EXPECT_EQ(result.status, TubeViabilityStatus::FEASIBLE);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.feasible);
  EXPECT_EQ(result.section_profile, &profile);
  return result;
}

phase_offset_core::PhaseOffsetGeometryState MakeGeometry(
    const std::uint64_t path_revision = 11U,
    const std::uint64_t frame_revision = 12U) {
  phase_offset_core::PhaseOffsetGeometryState geometry;
  geometry.valid = true;
  geometry.path_revision = path_revision;
  geometry.frame_revision = frame_revision;
  geometry.r_w = Eigen::Vector3d(2.0, 0.0, 0.0);
  geometry.N = Eigen::Vector3d(0.0, 1.0, 0.0);
  geometry.r = Eigen::Vector3d(1.0, 0.0, 0.0);
  return geometry;
}

PhaseOffsetAllocatorBounds BaseBounds() {
  PhaseOffsetAllocatorBounds bounds;
  bounds.lower_nu = 0.5;
  bounds.upper_nu = 2.0;
  bounds.u_w_abs_max = 10.0;
  bounds.upper_u_delta = 1.0;
  bounds.u_w_slew_rate = 100.0;
  bounds.u_delta_slew_rate = 100.0;
  return bounds;
}

PhaseOffsetAllocatorInput BaseInput(const NormalPreviewResult& preview) {
  PhaseOffsetAllocatorInput input;
  input.geometry = MakeGeometry(preview.provenance.path_revision,
                                preview.provenance.frame_revision);
  input.preview = &preview;
  input.f_w0 = 1.0;
  input.dt = 0.1;
  input.bounds = BaseBounds();
  input.expected_path_revision = preview.provenance.path_revision;
  input.expected_frame_revision = preview.provenance.frame_revision;
  input.expected_profile_revision = preview.provenance.profile_revision;
  input.expected_source_revision = preview.provenance.source_revision;
  input.expected_tube_revision = preview.provenance.tube_revision;
  input.expected_map_revision = preview.provenance.map_revision;
  input.expected_obstacle_contract_id =
      preview.provenance.obstacle_contract_id;
  return input;
}

void ExpectFailClosed(const PhaseOffsetAllocatorInput& input,
                      const PhaseOffsetAllocatorStatus status) {
  PhaseOffsetAllocatorResult output;
  EXPECT_FALSE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_EQ(output.status, status);
  EXPECT_FALSE(output.valid);
  EXPECT_FALSE(output.feasible);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 0.0);
  EXPECT_DOUBLE_EQ(output.selected_u.u_delta, 0.0);
  EXPECT_TRUE(output.selected_u_owner.empty());
  EXPECT_TRUE(output.selectedUConsistent());
}

TEST(PhaseOffsetAllocatorTest, FullyBoundMatchingProvenanceAllowsSelection) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.g_des = Eigen::Vector3d(0.4, 0.2, 0.0);

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_EQ(output.status, PhaseOffsetAllocatorStatus::SELECTED);
  EXPECT_EQ(output.preview_provenance.path_revision,
            preview.provenance.path_revision);
  EXPECT_EQ(output.preview_provenance.frame_revision,
            preview.provenance.frame_revision);
  EXPECT_EQ(output.preview_provenance.profile_revision,
            preview.provenance.profile_revision);
  EXPECT_EQ(output.preview_provenance.source_revision,
            preview.provenance.source_revision);
  EXPECT_EQ(output.preview_provenance.tube_revision,
            preview.provenance.tube_revision);
  EXPECT_EQ(output.preview_provenance.map_revision,
            preview.provenance.map_revision);
  EXPECT_EQ(output.preview_provenance.obstacle_contract_id,
            preview.provenance.obstacle_contract_id);
}

TEST(PhaseOffsetAllocatorSectionTest,
     SectionPreviewUsesPointerAndGeometryStateOnly) {
  const SectionTubeProfile profile = MakeSectionProfile();
  const NormalPreviewResult preview = MakeSectionPreview(profile);
  PhaseOffsetAllocatorInput input;
  input.geometry = MakeGeometry(preview.provenance.path_revision,
                                preview.provenance.frame_revision);
  input.geometry.w = preview.current_w;
  input.geometry.delta = preview.evaluated_delta;
  input.preview = &preview;
  input.expected_section_profile = &profile;
  input.expected_path_revision = preview.provenance.path_revision;
  input.expected_frame_revision = preview.provenance.frame_revision;
  input.f_w0 = 1.0;
  input.dt = 0.1;
  input.bounds = BaseBounds();
  input.g_des = Eigen::Vector3d(0.4, 0.2, 0.0);
  // Legacy identity expectations intentionally remain empty for SECTION.
  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output)) << output.reason;
  EXPECT_EQ(output.status, PhaseOffsetAllocatorStatus::SELECTED);
  EXPECT_TRUE(output.selectedUConsistent(0.0));
  EXPECT_EQ(output.preview_provenance.profile_revision, 0U);
  EXPECT_EQ(output.preview_provenance.map_revision, 0U);
}

TEST(PhaseOffsetAllocatorSectionTest, SectionAssociationMismatchesFailClosed) {
  const SectionTubeProfile profile = MakeSectionProfile();
  const SectionTubeProfile other = MakeSectionProfile();
  const NormalPreviewResult preview = MakeSectionPreview(profile);
  PhaseOffsetAllocatorInput input;
  input.geometry = MakeGeometry(preview.provenance.path_revision,
                                preview.provenance.frame_revision);
  input.geometry.w = preview.current_w;
  input.geometry.delta = preview.evaluated_delta;
  input.preview = &preview;
  input.expected_section_profile = &other;
  input.expected_path_revision = preview.provenance.path_revision;
  input.expected_frame_revision = preview.provenance.frame_revision;
  input.f_w0 = 1.0;
  input.dt = 0.1;
  input.bounds = BaseBounds();
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);

  input.expected_section_profile = &profile;
  input.geometry.delta = 0.1;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);

  input.geometry.delta = preview.evaluated_delta;
  ++input.geometry.path_revision;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);

  NormalPreviewResult wrong_kind = preview;
  wrong_kind.proof_kind = TubeViabilityProofKind::V2_LIVE_PWL;
  input.geometry.path_revision = preview.provenance.path_revision;
  input.preview = &wrong_kind;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::INVALID_INPUT);
}

TEST(PhaseOffsetAllocatorSectionTest,
     SectionGeometryAndFrameMismatchesAreRejected) {
  const SectionTubeProfile profile = MakeSectionProfile();
  const NormalPreviewResult preview = MakeSectionPreview(profile);
  PhaseOffsetAllocatorInput input;
  input.geometry = MakeGeometry(preview.provenance.path_revision,
                                preview.provenance.frame_revision);
  input.geometry.w = preview.current_w;
  input.geometry.delta = preview.evaluated_delta;
  input.preview = &preview;
  input.expected_section_profile = &profile;
  input.expected_path_revision = preview.provenance.path_revision;
  input.expected_frame_revision = preview.provenance.frame_revision;
  input.f_w0 = 1.0;
  input.dt = 0.1;
  input.bounds = BaseBounds();

  input.geometry.w = std::nextafter(preview.current_w, 1.0);
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);
  input.geometry.w = preview.current_w;
  ++input.expected_frame_revision;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);
}

TEST(PhaseOffsetAllocatorSectionTest,
     EmptySectionDiagnosticSourceDoesNotGateValidAssociation) {
  const SectionTubeProfile profile = MakeSectionProfile();
  NormalPreviewResult preview = MakeSectionPreview(profile);
  preview.provenance.source.clear();
  PhaseOffsetAllocatorInput input;
  input.geometry = MakeGeometry(preview.provenance.path_revision,
                                preview.provenance.frame_revision);
  input.geometry.w = preview.current_w;
  input.geometry.delta = preview.evaluated_delta;
  input.preview = &preview;
  input.expected_section_profile = &profile;
  input.expected_path_revision = preview.provenance.path_revision;
  input.expected_frame_revision = preview.provenance.frame_revision;
  input.f_w0 = 1.0;
  input.dt = 0.1;
  input.bounds = BaseBounds();
  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output)) << output.reason;
  EXPECT_EQ(output.status, PhaseOffsetAllocatorStatus::SELECTED);
}

TEST(PhaseOffsetAllocatorSectionTest,
     FailedSectionPreviewIsClassifiedInfeasibleWithNonzeroDelta) {
  SectionTubeProfile partial;
  partial.valid_start = 0.0;
  partial.valid_end = 1.0;
  partial.status = SectionTubeStatus::PARTIAL;
  partial.usable = true;
  partial.complete = false;
  partial.knots.push_back(SectionTubeKnot{0.0, -1.0, 1.0});
  partial.knots.push_back(SectionTubeKnot{1.0, -1.0, 1.0});
  NormalPreviewInput preview_input;
  preview_input.current_w = 0.0;
  preview_input.current_delta = 0.3;
  preview_input.upper_u_delta = 1.0;
  preview_input.policy.preview_horizon_w = 2.0;
  preview_input.policy.sample_spacing_w = 1.0;
  preview_input.policy.lower_nu = 0.5;
  preview_input.policy.upper_nu = 2.0;
  preview_input.policy.b_tight = 0.1;
  preview_input.policy.b_open = 0.9;
  preview_input.path_revision = 71U;
  preview_input.frame_revision = 72U;
  preview_input.expected_path_revision = 71U;
  preview_input.expected_frame_revision = 72U;
  preview_input.max_work = 100000U;
  NormalPreviewResult preview;
  ASSERT_TRUE(NormalPreview::evaluate(partial, preview_input, preview));
  ASSERT_EQ(preview.status, TubeViabilityStatus::PREVIEW_INFEASIBLE);
  EXPECT_EQ(preview.section_profile, nullptr);

  PhaseOffsetAllocatorInput input;
  input.geometry = MakeGeometry(71U, 72U);
  input.geometry.w = 0.0;
  input.geometry.delta = 0.3;
  input.preview = &preview;
  input.expected_section_profile = &partial;
  input.expected_path_revision = 71U;
  input.expected_frame_revision = 72U;
  input.f_w0 = 1.0;
  input.dt = 0.1;
  input.bounds = BaseBounds();
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::PREVIEW_INFEASIBLE);
}

TEST(PhaseOffsetAllocatorSectionTest,
     LegacyAndSectionSelectionMatchForSameGeometryAndLimits) {
  const TubeProfile legacy_profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult legacy_preview = MakePreview(legacy_profile);
  const SectionTubeProfile section_profile = MakeSectionProfile();
  const NormalPreviewResult section_preview = MakeSectionPreview(section_profile);
  PhaseOffsetAllocatorInput legacy_input = BaseInput(legacy_preview);
  legacy_input.g_des = Eigen::Vector3d(0.6, -0.3, 0.0);
  legacy_input.previous_u.u_w = 0.1;
  legacy_input.previous_u.u_delta = -0.1;
  legacy_input.bounds.u_w_abs_max = 0.4;
  legacy_input.bounds.upper_u_delta = 0.5;
  legacy_input.dt = 0.2;
  PhaseOffsetAllocatorInput section_input;
  section_input.geometry = MakeGeometry(section_preview.provenance.path_revision,
                                         section_preview.provenance.frame_revision);
  section_input.geometry.w = section_preview.current_w;
  section_input.geometry.delta = section_preview.evaluated_delta;
  section_input.preview = &section_preview;
  section_input.expected_section_profile = &section_profile;
  section_input.expected_path_revision = section_preview.provenance.path_revision;
  section_input.expected_frame_revision = section_preview.provenance.frame_revision;
  section_input.f_w0 = legacy_input.f_w0;
  section_input.g_des = legacy_input.g_des;
  section_input.previous_u = legacy_input.previous_u;
  section_input.dt = legacy_input.dt;
  section_input.bounds = legacy_input.bounds;
  PhaseOffsetAllocatorResult legacy_output;
  PhaseOffsetAllocatorResult section_output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(legacy_input, legacy_output))
      << legacy_output.reason;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(section_input, section_output))
      << section_output.reason;
  EXPECT_DOUBLE_EQ(section_output.selected_u.u_w, legacy_output.selected_u.u_w);
  EXPECT_DOUBLE_EQ(section_output.selected_u.u_delta,
                   legacy_output.selected_u.u_delta);
}

TEST(PhaseOffsetAllocatorTest, MissingExpectedSourceFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.expected_source_revision = 0U;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::INVALID_INPUT);
}

TEST(PhaseOffsetAllocatorTest, MissingExpectedTubeFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.expected_tube_revision = 0U;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::INVALID_INPUT);
}

TEST(PhaseOffsetAllocatorTest, MissingExpectedMapFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.expected_map_revision = 0U;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::INVALID_INPUT);
}

TEST(PhaseOffsetAllocatorTest, MissingExpectedObstacleContractFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.expected_obstacle_contract_id.clear();
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::INVALID_INPUT);
}

TEST(PhaseOffsetAllocatorTest, SourceMismatchFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  ++input.expected_source_revision;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);
}

TEST(PhaseOffsetAllocatorTest, TubeMismatchFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  ++input.expected_tube_revision;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);
}

TEST(PhaseOffsetAllocatorTest, MapMismatchFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  ++input.expected_map_revision;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);
}

TEST(PhaseOffsetAllocatorTest, ObstacleContractMismatchFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.expected_obstacle_contract_id = "different-obstacle-contract";
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);
}

TEST(PhaseOffsetAllocatorTest, OmittedExpectationsCannotBypassStaleValidation) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.expected_path_revision = 0U;
  input.expected_frame_revision = 0U;
  input.expected_profile_revision = 0U;
  input.expected_source_revision = 0U;
  input.expected_tube_revision = 0U;
  input.expected_map_revision = 0U;
  input.expected_obstacle_contract_id.clear();
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::INVALID_INPUT);
}

TEST(PhaseOffsetAllocatorTest, PathMismatchFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  ++input.expected_path_revision;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);
}

TEST(PhaseOffsetAllocatorTest, ProfileMismatchFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  ++input.expected_profile_revision;
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::STALE_PREVIEW);
}

TEST(PhaseOffsetAllocatorTest, PositiveAmplitudeClipsExactPositiveUw) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.25;
  input.g_des = Eigen::Vector3d(4.0, 0.0, 0.0);

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_DOUBLE_EQ(output.u_w_nom, 2.0);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 0.25);
}

TEST(PhaseOffsetAllocatorTest, PositiveAmplitudeClipsExactNegativeUw) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.25;
  input.g_des = Eigen::Vector3d(-4.0, 0.0, 0.0);

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_DOUBLE_EQ(output.u_w_nom, -2.0);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, -0.25);
}

TEST(PhaseOffsetAllocatorTest, ZeroAmplitudeWithPositiveNominalSelectsZero) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.0;
  input.g_des = Eigen::Vector3d(4.0, 0.0, 0.0);

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_DOUBLE_EQ(output.u_w_nom, 2.0);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 0.0);
}

TEST(PhaseOffsetAllocatorTest, ZeroAmplitudeWithNegativeNominalSelectsZero) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.0;
  input.g_des = Eigen::Vector3d(-4.0, 0.0, 0.0);

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_DOUBLE_EQ(output.u_w_nom, -2.0);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 0.0);
}

TEST(PhaseOffsetAllocatorTest, ZeroAmplitudeIsNeverUnbounded) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.lower_nu = 0.0;
  input.bounds.upper_nu = 100.0;
  input.bounds.u_w_abs_max = 0.0;
  input.g_des = Eigen::Vector3d(400.0, 0.0, 0.0);

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_GT(output.u_w_nom, 100.0);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 0.0);
  EXPECT_TRUE(output.u_w.amplitude_limited);
}

TEST(PhaseOffsetAllocatorTest, ZeroAmplitudeZeroSlewAtZeroHoldsZero) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.0;
  input.bounds.u_w_slew_rate = 0.0;
  input.previous_u.u_w = 0.0;
  input.g_des = Eigen::Vector3d(4.0, 0.0, 0.0);

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 0.0);
  EXPECT_DOUBLE_EQ(output.next_u_prev.u_w, 0.0);
}

TEST(PhaseOffsetAllocatorTest,
     PhaseSlewSubEpsilonAboveAmplitudeFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.25;
  input.bounds.u_w_slew_rate = 0.0;
  input.previous_u.u_w = 0.2500000000005;
  input.g_des = Eigen::Vector3d::Zero();

  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND);
}

TEST(PhaseOffsetAllocatorTest,
     ZeroAmplitudeNegativeSubEpsilonPreviousFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.0;
  input.bounds.u_w_slew_rate = 0.0;
  input.previous_u.u_w = -5e-13;
  input.g_des = Eigen::Vector3d::Zero();

  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND);
}

TEST(PhaseOffsetAllocatorTest,
     NegativePhaseSlewSubEpsilonBelowAmplitudeFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.25;
  input.bounds.u_w_slew_rate = 0.0;
  input.previous_u.u_w = -0.2500000000005;
  input.g_des = Eigen::Vector3d::Zero();

  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND);
}

TEST(PhaseOffsetAllocatorTest,
     PhaseSlewRampTowardsAnUnreachableWindowStillSelects) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  // Base rate above the window, and the requested correction is further away
  // than one tick of slew authority.  The tick must still produce the
  // closest slew-reachable rate and ramp over the following ticks; failing
  // here would freeze the phase permanently because a dropped tick never
  // commits (`phase scalar slew interval is empty`).
  input.f_w0 = 2.5;
  input.bounds.lower_nu = 0.5;
  input.bounds.upper_nu = 2.0;
  input.bounds.u_w_abs_max = 0.2;
  input.bounds.u_w_slew_rate = 0.2;
  input.previous_u.u_w = 0.0;
  input.g_des = Eigen::Vector3d::Zero();

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output)) << output.reason;
  EXPECT_NEAR(output.selected_u.u_w, -0.02, 1e-12);
  EXPECT_TRUE(output.phase_window_clipped);
  EXPECT_NEAR(output.phase_rate_selected,
              input.f_w0 + output.selected_u.u_w, 1e-12);
  EXPECT_GT(output.phase_rate_selected, input.bounds.upper_nu);
}

TEST(PhaseOffsetAllocatorTest,
     ExactTouchingClosedPhaseIntervalsRemainFeasible) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.25;
  input.bounds.u_w_slew_rate = 0.0;
  input.previous_u.u_w = 0.25;
  input.g_des = Eigen::Vector3d::Zero();

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 0.25);
  EXPECT_DOUBLE_EQ(output.u_w.selected_lower, 0.25);
  EXPECT_DOUBLE_EQ(output.u_w.selected_upper, 0.25);
}

TEST(PhaseOffsetAllocatorTest,
     NarrowTruePhaseIntersectionBelowToleranceIsPreserved) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.25;
  input.bounds.lower_nu = 1.2499999999996;
  input.bounds.upper_nu = 1.25;
  input.bounds.u_w_slew_rate = 2e-13;
  input.previous_u.u_w = 0.2499999999998;
  input.dt = 1.0;
  input.g_des = Eigen::Vector3d::Zero();

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  const double width = output.u_w.selected_upper -
      output.u_w.selected_lower;
  EXPECT_GT(width, 0.0);
  EXPECT_LT(width, 1e-12);
  EXPECT_GE(output.selected_u.u_w, output.u_w.selected_lower);
  EXPECT_LE(output.selected_u.u_w, output.u_w.selected_upper);
}

TEST(PhaseOffsetAllocatorTest, ZeroAmplitudeSlewThatCannotReachZeroFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.0;
  input.bounds.u_w_slew_rate = 0.0;
  input.previous_u.u_w = 0.1;
  input.g_des = Eigen::Vector3d(0.0, 0.0, 0.0);
  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND);
}

TEST(PhaseOffsetAllocatorTest, AnalyticProjectionIsExactAndValueOnly) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.upper_nu = 3.0;
  input.g_des = Eigen::Vector3d(4.0, 0.6, 3.0);

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_DOUBLE_EQ(output.u_w_nom, 2.0);
  EXPECT_DOUBLE_EQ(output.u_delta_nom, 0.6);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 2.0);
  EXPECT_DOUBLE_EQ(output.selected_u.u_delta, 0.6);
  EXPECT_TRUE(output.selectedUConsistent());
  EXPECT_EQ(output.selected_u_owner, "PhaseOffsetAllocator");
  EXPECT_TRUE(output.piecewise_constant);
  EXPECT_DOUBLE_EQ(output.zoh_dt, input.dt);
}

TEST(PhaseOffsetAllocatorTest, PhaseClippingPreservesPositiveProgression) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);

  input.g_des = Eigen::Vector3d(-20.0, 0.0, 0.0);
  PhaseOffsetAllocatorResult lower;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, lower));
  EXPECT_DOUBLE_EQ(lower.selected_u.u_w, -0.5);
  EXPECT_DOUBLE_EQ(input.f_w0 + lower.selected_u.u_w,
                   input.bounds.lower_nu);
  EXPECT_GE(input.f_w0 + lower.selected_u.u_w, 0.0);

  input.g_des = Eigen::Vector3d(20.0, 0.0, 0.0);
  PhaseOffsetAllocatorResult upper;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, upper));
  EXPECT_DOUBLE_EQ(upper.selected_u.u_w, 1.0);
  EXPECT_DOUBLE_EQ(input.f_w0 + upper.selected_u.u_w,
                   input.bounds.upper_nu);
  EXPECT_GT(input.f_w0 + upper.selected_u.u_w, 0.0);
}

TEST(PhaseOffsetAllocatorTest, TransverseInteriorUsesOnlyItsOwnProjection) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.g_des = Eigen::Vector3d(0.0, 0.35, 0.0);

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_DOUBLE_EQ(output.u_delta_nom, 0.35);
  EXPECT_DOUBLE_EQ(output.selected_u.u_delta, 0.35);
  EXPECT_FALSE(output.u_delta.envelope_limited);
  EXPECT_FALSE(output.u_delta.slew_limited);
}

TEST(PhaseOffsetAllocatorTest, LowerAndUpperMovingBoundaryRatesAreConsumed) {
  const TubeProfile profile = MakeProfile({{-0.2, 0.4}, {-0.1, 0.3},
                                           {0.0, 0.2}});

  const NormalPreviewResult lower_preview = MakePreview(profile, -0.2);
  PhaseOffsetAllocatorInput lower_input = BaseInput(lower_preview);
  lower_input.g_des = Eigen::Vector3d(0.0, 0.0, 0.0);
  PhaseOffsetAllocatorResult lower;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(lower_input, lower));
  EXPECT_TRUE(lower.preview_rate_interval.lower_boundary_active);
  EXPECT_NEAR(lower.preview_rate_interval.lower, 0.2, 1e-12);
  EXPECT_NEAR(lower.selected_u.u_delta, 0.2, 1e-12);

  const NormalPreviewResult upper_preview = MakePreview(profile, 0.4);
  PhaseOffsetAllocatorInput upper_input = BaseInput(upper_preview);
  upper_input.g_des = Eigen::Vector3d(0.0, 0.0, 0.0);
  PhaseOffsetAllocatorResult upper;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(upper_input, upper));
  EXPECT_TRUE(upper.preview_rate_interval.upper_boundary_active);
  EXPECT_NEAR(upper.preview_rate_interval.upper, -0.2, 1e-12);
  EXPECT_NEAR(upper.selected_u.u_delta, -0.2, 1e-12);
}

TEST(PhaseOffsetAllocatorTest,
     TransverseAmplitudeSubEpsilonDisjointClipsToBoundary) {
  // M4C: a transverse window that excludes every admissible rate no longer
  // invalidates the whole command.  Losing transverse capacity must not by
  // itself drop a planner-valid path, so the command takes the boundary of the
  // window that the amplitude can actually reach.
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  NormalPreviewResult preview = MakePreview(profile);
  preview.current_rate_interval.lower = 0.2500000000005;
  preview.current_rate_interval.upper = 0.2500000000005;
  preview.current_rate_interval.valid = true;

  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.upper_u_delta = 0.25;
  input.g_des = Eigen::Vector3d::Zero();

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.transverse_interval_clipped);
  EXPECT_FALSE(output.phase_window_clipped);
  EXPECT_DOUBLE_EQ(output.selected_u.u_delta, 0.25);
  EXPECT_DOUBLE_EQ(output.u_delta.selected, 0.25);
}

TEST(PhaseOffsetAllocatorTest,
     PhaseWindowClipsOntoAuthorityBoundaryInsteadOfFailing) {
  // M4C: when the base phase rate sits outside the allowed window and the
  // correction authority cannot reach the window, take the authority boundary
  // that reduces the violation the most (here: slow down as much as allowed).
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.12;
  input.f_w0 = 3.0;  // window is [0.5, 2.0]; f_w0 is far above it
  input.g_des = Eigen::Vector3d::Zero();

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.phase_window_clipped);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, -0.12);
  EXPECT_DOUBLE_EQ(output.u_w.selected, -0.12);
  EXPECT_DOUBLE_EQ(output.phase_rate_selected, input.f_w0 - 0.12);
}

TEST(PhaseOffsetAllocatorTest,
     RateDegradedPreviewStillAllocatesAClippedCommand) {
  // M4C: a RATE_INFEASIBLE preview whose current state is still inside the
  // corridor stays usable; the allocator clips instead of returning
  // PREVIEW_INFEASIBLE.
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  NormalPreviewResult preview = MakePreview(profile);
  preview.status = TubeViabilityStatus::RATE_INFEASIBLE;
  preview.rate_feasible = false;
  // The corridor window is unusable for allocation, but the current state is
  // still inside the corridor, so the tick must not be dropped.
  preview.current_rate_interval.lower = 0.0;
  preview.current_rate_interval.upper = 0.0;
  preview.current_rate_interval.valid = false;

  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_w_abs_max = 0.12;
  input.f_w0 = 3.0;
  input.g_des = Eigen::Vector3d(0.0, 0.2, 0.0);

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.phase_window_clipped);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, -0.12);
  EXPECT_DOUBLE_EQ(output.selected_u.u_delta, 0.0);
}

TEST(PhaseOffsetAllocatorTest,
     TransverseSlewSubEpsilonDisjointFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.bounds.u_delta_slew_rate = 0.0;
  input.previous_u.u_delta = 1.0000000000005;
  input.g_des = Eigen::Vector3d::Zero();

  ExpectFailClosed(input, PhaseOffsetAllocatorStatus::NO_ADMISSIBLE_COMMAND);
}

TEST(PhaseOffsetAllocatorTest, AmplitudeAndSlewSaturationArePerScalar) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.g_des = Eigen::Vector3d(10.0, 0.9, 0.0);
  input.bounds.u_w_abs_max = 0.2;
  input.bounds.upper_u_delta = 0.3;
  input.bounds.u_w_slew_rate = 0.1;
  input.bounds.u_delta_slew_rate = 0.2;
  input.dt = 1.0;

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_DOUBLE_EQ(output.u_w.pre_slew, 0.2);
  EXPECT_DOUBLE_EQ(output.u_delta.pre_slew, 0.3);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 0.1);
  EXPECT_DOUBLE_EQ(output.selected_u.u_delta, 0.2);
  EXPECT_TRUE(output.u_w.amplitude_limited);
  EXPECT_TRUE(output.u_delta.amplitude_limited);
  EXPECT_TRUE(output.u_w.slew_limited);
  EXPECT_TRUE(output.u_delta.slew_limited);
}

TEST(PhaseOffsetAllocatorTest, InterSampleSlewUsesPreviousCommandForEachScalar) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.g_des = Eigen::Vector3d(0.0, 0.0, 0.0);
  input.previous_u.u_w = 0.4;
  input.previous_u.u_delta = 0.2;
  input.bounds.u_w_slew_rate = 0.1;
  input.bounds.u_delta_slew_rate = 0.05;
  input.dt = 1.0;

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 0.3);
  EXPECT_DOUBLE_EQ(output.selected_u.u_delta, 0.15);
  EXPECT_NEAR(output.selected_u.u_w - input.previous_u.u_w, -0.1, 1e-12);
  EXPECT_NEAR(output.selected_u.u_delta - input.previous_u.u_delta,
              -0.05, 1e-12);
}

TEST(PhaseOffsetAllocatorTest, RepeatedCallsProduceIdenticalZohCommand) {
  const TubeProfile profile = MakeProfile({{-0.7, 0.8}, {-0.5, 0.6},
                                           {-0.2, 0.4}});
  const NormalPreviewResult preview = MakePreview(profile, -0.1);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.g_des = Eigen::Vector3d(3.2, -0.25, 0.0);

  PhaseOffsetAllocatorResult first;
  PhaseOffsetAllocatorResult second;
  ASSERT_TRUE(PhaseOffsetAllocator::evaluate(input, first));
  ASSERT_TRUE(PhaseOffsetAllocator::evaluate(input, second));
  EXPECT_EQ(first.status, second.status);
  EXPECT_DOUBLE_EQ(first.u_w_nom, second.u_w_nom);
  EXPECT_DOUBLE_EQ(first.u_delta_nom, second.u_delta_nom);
  EXPECT_DOUBLE_EQ(first.selected_u.u_w, second.selected_u.u_w);
  EXPECT_DOUBLE_EQ(first.selected_u.u_delta, second.selected_u.u_delta);
  EXPECT_DOUBLE_EQ(first.next_u_prev.u_w, first.selected_u.u_w);
  EXPECT_DOUBLE_EQ(first.next_u_prev.u_delta, first.selected_u.u_delta);
  EXPECT_TRUE(first.piecewise_constant);
  EXPECT_TRUE(second.piecewise_constant);
}

TEST(PhaseOffsetAllocatorTest, ZohDurationBoundsAreFailClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.g_des = Eigen::Vector3d(0.2, 0.1, 0.0);
  input.bounds.zoh_min_dt = 0.2;

  PhaseOffsetAllocatorResult short_tick;
  EXPECT_FALSE(PhaseOffsetAllocator::allocate(input, short_tick));
  EXPECT_EQ(short_tick.status, PhaseOffsetAllocatorStatus::INVALID_INPUT);

  input.bounds.zoh_min_dt = 0.1;
  input.bounds.zoh_max_dt = 0.1;
  input.bounds.zoh_dt = 0.1;
  PhaseOffsetAllocatorResult exact_tick;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, exact_tick));
  EXPECT_DOUBLE_EQ(exact_tick.zoh_dt, input.dt);
}

TEST(PhaseOffsetAllocatorTest, InvalidOrStalePreviewFailsClosedWithoutOwner) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.g_des = Eigen::Vector3d(1.0, 0.1, 0.0);

  input.preview = nullptr;
  PhaseOffsetAllocatorResult missing;
  EXPECT_FALSE(PhaseOffsetAllocator::allocate(input, missing));
  EXPECT_EQ(missing.status, PhaseOffsetAllocatorStatus::STALE_PREVIEW);
  EXPECT_FALSE(missing.valid);
  EXPECT_TRUE(missing.selected_u_owner.empty());

  input.preview = &preview;
  input.expected_frame_revision = preview.provenance.frame_revision + 1U;
  PhaseOffsetAllocatorResult stale;
  EXPECT_FALSE(PhaseOffsetAllocator::allocate(input, stale));
  EXPECT_EQ(stale.status, PhaseOffsetAllocatorStatus::STALE_PREVIEW);
  EXPECT_FALSE(stale.valid);
  EXPECT_TRUE(stale.selected_u_owner.empty());
}

TEST(PhaseOffsetAllocatorTest, NonFeasiblePreviewFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, -0.5}, {0.5, 1.0},
                                           {0.5, 1.0}});
  NormalPreviewResult preview;
  NormalPreviewInput preview_input;
  preview_input.profile = &profile;
  preview_input.current_w = 0.0;
  preview_input.current_delta = -0.75;
  preview_input.upper_u_delta = 0.1;
  preview_input.policy.preview_horizon_w = profile.preview_end_w;
  preview_input.policy.sample_spacing_w = 1.0;
  preview_input.policy.lower_nu = 0.5;
  preview_input.policy.upper_nu = 2.0;
  preview_input.policy.b_tight = 0.1;
  preview_input.policy.b_open = 0.9;
  preview_input.policy.policy_revision = 1U;
  preview_input.policy.configuration_identity = 1U;
  preview_input.policy.configuration_id = "test-normal-preview-w";
  preview_input.path_revision = profile.path_revision;
  preview_input.frame_revision = profile.frame_revision;
  preview_input.profile_revision = profile.profile_revision;
  preview_input.expected_path_revision = profile.path_revision;
  preview_input.expected_frame_revision = profile.frame_revision;
  preview_input.expected_profile_revision = profile.profile_revision;
  ASSERT_TRUE(NormalPreview::evaluate(preview_input, preview));
  ASSERT_EQ(preview.status, TubeViabilityStatus::PREVIEW_INFEASIBLE);

  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.g_des = Eigen::Vector3d(1.0, 0.1, 0.0);
  PhaseOffsetAllocatorResult output;
  EXPECT_FALSE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_EQ(output.status, PhaseOffsetAllocatorStatus::PREVIEW_INFEASIBLE);
  EXPECT_FALSE(output.valid);
  EXPECT_TRUE(output.selected_u_owner.empty());
}

TEST(PhaseOffsetAllocatorTest, NonFiniteInputFailsClosed) {
  const TubeProfile profile = MakeProfile({{-1.0, 1.0}, {-1.0, 1.0}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.g_des.x() = std::numeric_limits<double>::quiet_NaN();

  PhaseOffsetAllocatorResult output;
  EXPECT_FALSE(PhaseOffsetAllocator::allocate(input, output));
  EXPECT_EQ(output.status, PhaseOffsetAllocatorStatus::INVALID_INPUT);
  EXPECT_FALSE(output.valid);
  EXPECT_TRUE(output.selected_u_owner.empty());
}

TEST(PhaseOffsetAllocatorTest, ScalarsAreClippedIndependentlyWithoutReplacement) {
  const TubeProfile profile = MakeProfile({{-0.4, 0.4}, {-0.4, 0.4}});
  const NormalPreviewResult preview = MakePreview(profile);
  PhaseOffsetAllocatorInput input = BaseInput(preview);
  input.g_des = Eigen::Vector3d(100.0, 100.0, 0.0);
  input.bounds.lower_nu = 0.8;
  input.bounds.upper_nu = 1.2;
  input.bounds.u_w_abs_max = 0.25;
  input.bounds.upper_u_delta = 0.3;

  PhaseOffsetAllocatorResult output;
  ASSERT_TRUE(PhaseOffsetAllocator::allocate(input, output));
  // u_w_nom=50 and u_delta_nom=100 are each clipped only by their own
  // scalar admissible intervals.  Neither result is chosen by a joint cost.
  EXPECT_DOUBLE_EQ(output.u_w_nom, 50.0);
  EXPECT_DOUBLE_EQ(output.u_delta_nom, 100.0);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w, 0.2);
  EXPECT_DOUBLE_EQ(output.selected_u.u_delta, 0.3);
  EXPECT_DOUBLE_EQ(input.f_w0 + output.selected_u.u_w,
                   input.bounds.upper_nu);
  EXPECT_DOUBLE_EQ(output.selected_u.u_delta,
                   input.bounds.upper_u_delta);
  EXPECT_DOUBLE_EQ(output.selected_u.u_w,
                   output.u_w.selected_upper);
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
