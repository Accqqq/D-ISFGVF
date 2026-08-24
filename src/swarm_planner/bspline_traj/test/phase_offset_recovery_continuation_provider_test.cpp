#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "bspline_race/integration/phase_offset_executed_reference_query.h"
#include "bspline_race/integration/phase_offset_recovery_continuation_provider.h"

namespace FLAG_Race {
namespace {

std::shared_ptr<ContinuousPhasePath> MakeLinePath(const double speed,
                                                  const std::uint64_t revision) {
  auto path = std::make_shared<ContinuousPhasePath>();
  if (!path->appendSegment(
      0.0, 10.0, "line",
      [speed](double w, ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(speed * w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d(speed, 0.0, 0.0);
        state.d2p_dw2 = Eigen::Vector3d::Zero();
        state.valid = true;
        return true;
      })) {
    return std::shared_ptr<ContinuousPhasePath>();
  }
  path->setPathRevision(revision);
  return path;
}

TEST(RecoveryContinuationProviderTest, PreservesReferenceJetOnSameOwner) {
  auto path = std::make_shared<ContinuousPhasePath>();
  ASSERT_TRUE(path->appendSegment(
      0.0, 10.0, "circle",
      ContinuousPhasePath::makePeriodicCircle(Eigen::Vector3d::Zero(), 1.0,
                                              10.0, 1.0)));
  path->setPathRevision(11U);
  auto frame = std::make_shared<ContinuousPhaseNormalFrame>(path, 11U, 12U);
  auto query = std::make_shared<PhaseOffsetExecutedReferenceQuery>(
      path, 0.0, frame, 11U, 12U, 13U, 14U);
  phase_offset_navigation::RecoveryReferenceJet source;
  ASSERT_TRUE(phase_offset_navigation::makeRecoveryReferenceJet(
      *query, 1.0, 0.0, source));
  PhaseOffsetRecoveryContinuationInput input;
  input.source_path = path;
  input.source_frame = frame;
  input.target_path = path;
  input.target_frame = frame;
  input.source_jet = source;
  input.source_w = 1.0;
  input.target_w = 1.0;
  input.finite_domain_start_w = 0.0;
  input.finite_domain_end_w = 10.0;
  input.delta = 0.0;
  input.recovery_session = 1U;
  input.source_owner_revision = 13U;
  input.target_owner_revision = 13U;
  PhaseOffsetRecoveryContinuationOutput output;
  EXPECT_TRUE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_TRUE(output.c2_continuous);
  EXPECT_TRUE(output.finite_domain);
}

TEST(RecoveryContinuationProviderTest, RejectsSourceReferenceRevisionMismatch) {
  auto path = MakeLinePath(1.0, 11U);
  auto frame = std::make_shared<ContinuousPhaseNormalFrame>(path, 11U, 12U);
  auto query = std::make_shared<PhaseOffsetExecutedReferenceQuery>(
      path, 0.0, frame, 11U, 12U, 13U, 14U);
  phase_offset_navigation::RecoveryReferenceJet source;
  ASSERT_TRUE(phase_offset_navigation::makeRecoveryReferenceJet(
      *query, 1.0, 0.0, source));
  source.owner_revision = 99U;

  PhaseOffsetRecoveryContinuationInput input;
  input.source_path = path;
  input.source_frame = frame;
  input.target_path = path;
  input.target_frame = frame;
  input.source_jet = source;
  input.source_w = 1.0;
  input.target_w = 1.0;
  input.finite_domain_start_w = 0.0;
  input.finite_domain_end_w = 10.0;
  input.recovery_session = 1U;
  input.source_owner_revision = 13U;
  input.target_owner_revision = 13U;
  PhaseOffsetRecoveryContinuationOutput output;
  EXPECT_FALSE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_EQ(output.invalid_reason, "source reference jet continuity contract failed");
  EXPECT_FALSE(output.valid);
}

TEST(RecoveryContinuationProviderTest, RejectsNonFiniteAndOutOfDomainInputs) {
  auto path = MakeLinePath(1.0, 11U);
  auto frame = std::make_shared<ContinuousPhaseNormalFrame>(path, 11U, 12U);
  auto query = std::make_shared<PhaseOffsetExecutedReferenceQuery>(
      path, 0.0, frame, 11U, 12U, 13U, 14U);
  phase_offset_navigation::RecoveryReferenceJet source;
  ASSERT_TRUE(phase_offset_navigation::makeRecoveryReferenceJet(
      *query, 1.0, 0.0, source));

  PhaseOffsetRecoveryContinuationInput input;
  input.source_path = path;
  input.source_frame = frame;
  input.target_path = path;
  input.target_frame = frame;
  input.source_jet = source;
  input.source_w = 1.0;
  input.target_w = 1.0;
  input.finite_domain_start_w = 0.0;
  input.finite_domain_end_w =
      std::numeric_limits<double>::quiet_NaN();
  input.recovery_session = 1U;
  input.source_owner_revision = 13U;
  input.target_owner_revision = 13U;
  PhaseOffsetRecoveryContinuationOutput output;
  EXPECT_FALSE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_EQ(output.invalid_reason, "continuation input is invalid");

  input.finite_domain_end_w = 10.0;
  input.source_w = -1.0;
  EXPECT_FALSE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_EQ(output.invalid_reason, "continuation input is invalid");
}

TEST(RecoveryContinuationProviderTest, RejectsNonPositiveContinuationSpeed) {
  auto source_path = MakeLinePath(1.0, 11U);
  auto source_frame = std::make_shared<ContinuousPhaseNormalFrame>(
      source_path, 11U, 12U);
  auto query = std::make_shared<PhaseOffsetExecutedReferenceQuery>(
      source_path, 0.0, source_frame, 11U, 12U, 13U, 14U);
  phase_offset_navigation::RecoveryReferenceJet source;
  ASSERT_TRUE(phase_offset_navigation::makeRecoveryReferenceJet(
      *query, 1.0, 0.0, source));

  // The frame accepts a nonzero tangent above its numerical epsilon, while
  // the continuation contract still rejects a phase speed <= 1e-8.
  auto target_path = MakeLinePath(5e-9, 21U);
  auto target_frame = std::make_shared<ContinuousPhaseNormalFrame>(
      target_path, 21U, 22U);
  PhaseOffsetRecoveryContinuationInput input;
  input.source_path = source_path;
  input.source_frame = source_frame;
  input.target_path = target_path;
  input.target_frame = target_frame;
  input.source_jet = source;
  input.source_w = 1.0;
  input.target_w = 1.0;
  input.finite_domain_start_w = 0.0;
  input.finite_domain_end_w = 10.0;
  input.recovery_session = 1U;
  input.source_owner_revision = 13U;
  input.target_owner_revision = 13U;
  PhaseOffsetRecoveryContinuationOutput output;
  EXPECT_FALSE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_EQ(output.invalid_reason,
            "target continuation has no finite phase speed");
  EXPECT_FALSE(output.valid);
}

TEST(RecoveryContinuationProviderTest, ValidSourceAndTargetJetsRemainC2Continuous) {
  auto path = MakeLinePath(1.0, 31U);
  auto frame = std::make_shared<ContinuousPhaseNormalFrame>(path, 31U, 32U);
  auto query = std::make_shared<PhaseOffsetExecutedReferenceQuery>(
      path, 0.15, frame, 31U, 32U, 41U, 42U);
  phase_offset_navigation::RecoveryReferenceJet source;
  ASSERT_TRUE(phase_offset_navigation::makeRecoveryReferenceJet(
      *query, 2.0, 0.15, source));

  PhaseOffsetRecoveryContinuationInput input;
  input.source_path = path;
  input.source_frame = frame;
  input.target_path = path;
  input.target_frame = frame;
  input.source_jet = source;
  input.source_w = 2.0;
  input.target_w = 2.0;
  input.delta = 0.15;
  input.finite_domain_start_w = 0.0;
  input.finite_domain_end_w = 10.0;
  input.recovery_session = 7U;
  input.source_owner_revision = 41U;
  input.target_owner_revision = 41U;
  PhaseOffsetRecoveryContinuationOutput output;
  ASSERT_TRUE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.c2_continuous);
  EXPECT_TRUE(output.finite_domain);
  // Batch-B production intentionally exercises only the first-order
  // continuation contract; the optional full-jet capability remains
  // disabled unless a future caller explicitly requests it.
  EXPECT_FALSE(output.full_jet_valid);
  EXPECT_GT(output.s_dot, 0.0);
  EXPECT_TRUE(output.target_jet.matches(source));
}

TEST(RecoveryContinuationProviderTest,
     StrictC2RequiresFullJetAndRevisionBoundActualTimeProgress) {
  auto path = MakeLinePath(1.0, 51U);
  auto frame = std::make_shared<ContinuousPhaseNormalFrame>(path, 51U, 52U);
  auto query = std::make_shared<PhaseOffsetExecutedReferenceQuery>(
      path, 0.0, frame, 51U, 52U, 53U, 54U);
  phase_offset_navigation::RecoveryReferenceJet source;
  ASSERT_TRUE(phase_offset_navigation::makeRecoveryReferenceJet(
      *query, 1.0, 0.0, source));
  source.r_ww = Eigen::Vector3d::Zero();
  source.r_ww_valid = true;

  PhaseOffsetRecoveryContinuationInput input;
  input.source_path = path;
  input.source_frame = frame;
  input.target_path = path;
  input.target_frame = frame;
  input.source_jet = source;
  input.source_w = 1.0;
  input.target_w = 1.0;
  input.delta = 0.0;
  input.finite_domain_start_w = 0.0;
  input.finite_domain_end_w = 10.0;
  input.recovery_session = 2U;
  input.source_owner_revision = 53U;
  input.target_owner_revision = 53U;
  input.require_full_jet = true;
  input.require_time_progress = true;
  input.v_s_min = 0.5;
  input.min_time_progress = 0.0;
  input.validated_s_dot = 0.75;
  input.s_dot_revision = 53U;
  input.s_dot_provenance = "validated-test-sdot";
  input.actual_s_dot_valid = true;

  PhaseOffsetRecoveryContinuationOutput output;
  ASSERT_TRUE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_TRUE(output.full_jet_valid);
  EXPECT_TRUE(output.time_progress_valid);
  EXPECT_DOUBLE_EQ(output.s_dot, 0.75);

  // The frozen lower-bound contract is inclusive: an actual witness equal to
  // v_s_min is valid when it is revision/provenance bound.
  input.validated_s_dot = input.v_s_min;
  ASSERT_TRUE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_DOUBLE_EQ(output.s_dot, input.v_s_min);
  ASSERT_TRUE(PhaseOffsetRecoveryContinuationProvider::
              validateSelectedTimeProgress(
                  input, input.v_s_min, input.target_owner_revision,
                  "selected-recovery-sdot", output));
  input.target_owner_revision = 99U;
  EXPECT_FALSE(PhaseOffsetRecoveryContinuationProvider::
               validateSelectedTimeProgress(
                   input, input.v_s_min, 53U, "selected-recovery-sdot", output));
  input.target_owner_revision = 53U;

  input.actual_s_dot_valid = false;
  EXPECT_FALSE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_EQ(output.invalid_reason,
            "validated actual s_dot provenance is unavailable");
  input.actual_s_dot_valid = true;
  input.s_dot_revision = 99U;
  EXPECT_FALSE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_EQ(output.invalid_reason,
            "validated actual s_dot provenance is unavailable");
}

TEST(RecoveryContinuationProviderTest, StrictC2RejectsMissingFullJet) {
  auto path = MakeLinePath(1.0, 61U);
  auto frame = std::make_shared<ContinuousPhaseNormalFrame>(path, 61U, 62U);
  auto query = std::make_shared<PhaseOffsetExecutedReferenceQuery>(
      path, 0.0, frame, 61U, 62U, 63U, 64U);
  phase_offset_navigation::RecoveryReferenceJet source;
  ASSERT_TRUE(phase_offset_navigation::makeRecoveryReferenceJet(
      *query, 1.0, 0.0, source));
  PhaseOffsetRecoveryContinuationInput input;
  input.source_path = path;
  input.source_frame = frame;
  input.target_path = path;
  input.target_frame = frame;
  input.source_jet = source;
  input.source_w = 1.0;
  input.target_w = 1.0;
  input.finite_domain_start_w = 0.0;
  input.finite_domain_end_w = 10.0;
  input.recovery_session = 3U;
  input.source_owner_revision = 63U;
  input.target_owner_revision = 63U;
  input.require_full_jet = true;
  PhaseOffsetRecoveryContinuationOutput output;
  EXPECT_FALSE(PhaseOffsetRecoveryContinuationProvider::propose(input, output));
  EXPECT_EQ(output.invalid_reason,
            "source reference jet continuity contract failed");
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
