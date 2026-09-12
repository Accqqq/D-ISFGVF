#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

// The adapter owns the publication transaction.  Tests inspect its private
// value slots to assert that failed publication/reset never commits Runtime;
// no retired worker, certificate, map-id, or reserve API is exercised here.
#define private public
#include "bspline_race/integration/phase_offset_matched_adapter.h"
#undef private

namespace FLAG_Race {
namespace {

constexpr double kDt = 0.02;

guidance::IsfGains MakeGains() {
  guidance::IsfGains gains;
  gains.k1 = 2.0;
  gains.k2 = -2.2;
  gains.convergence_bandwidth = 0.1;
  gains.progress_rho0 = 0.5;
  gains.progress_delta = 0.3;
  gains.alpha_min = 0.05;
  return gains;
}

phase_offset_core::PathDifferentialState MakeStraightState(
    const double w, const std::uint64_t revision = 7U) {
  phase_offset_core::PathDifferentialState state;
  state.p = Eigen::Vector3d(w, 0.0, 1.0);
  state.p_w = Eigen::Vector3d::UnitX();
  state.p_ww = Eigen::Vector3d::Zero();
  state.T = Eigen::Vector3d::UnitX();
  state.N = Eigen::Vector3d::UnitY();
  state.N_w = Eigen::Vector3d::Zero();
  state.path_revision = revision;
  state.frame_revision = revision;
  state.frame_valid = true;
  state.frame_provenance =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  state.w = w;
  state.valid = true;
  return state;
}

bool RefreshLegacyGuidance(MatchedAdapterInput& input) {
  PhaseOffsetActiveAdapter active;
  ActiveAdapterInput active_input;
  active_input.path = input.path;
  active_input.position = input.position;
  active_input.gains = input.gains;
  ActiveAdapterOutput active_output;
  if (!active.evaluate(active_input, active_output)) return false;
  input.legacy = LegacyGuidanceSnapshot(
      active_output.guidance.v_cmd, active_output.guidance.w_dot,
      active_output.guidance.e_parallel, active_output.guidance.e_perp,
      active_output.guidance.ref_pt, active_output.guidance.tangent,
      active_output.guidance.valid);
  return true;
}

std::shared_ptr<const ContinuousPhasePath> MakeStraightPath(
    const double start_w = 0.0, const double end_w = 3.0,
    const std::uint64_t revision = 7U) {
  std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  if (!path->appendSegment(
          start_w, end_w, "section-test-straight",
          [start_w, end_w](const double w, ContinuousPhasePathState& state) {
            state.p = Eigen::Vector3d(w, 0.0, 1.0);
            state.dp_dw = Eigen::Vector3d::UnitX();
            state.d2p_dw2 = Eigen::Vector3d::Zero();
            state.vel = state.dp_dw;
            state.valid = std::isfinite(w) && w >= start_w && w <= end_w;
            return state.valid;
          })) {
    return std::shared_ptr<const ContinuousPhasePath>();
  }
  path->setPathRevision(revision);
  return path;
}

std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>
MakeSectionProfile(const double end_w) {
  std::shared_ptr<phase_offset_navigation::SectionTubeProfile> profile(
      new phase_offset_navigation::SectionTubeProfile());
  profile->valid_start = 0.0;
  profile->valid_end = end_w;
  profile->status = phase_offset_navigation::SectionTubeStatus::COMPLETE;
  profile->usable = true;
  profile->complete = true;
  const double samples[] = {0.0, 0.5, 1.0, 2.0, 3.0};
  for (const double w : samples) {
    if (w > end_w) continue;
    phase_offset_navigation::SectionTubeKnot knot;
    knot.w = w;
    knot.lower = -1.0;
    knot.upper = 1.0;
    profile->knots.push_back(knot);
  }
  return std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>(
      std::move(profile));
}

SectionPathBundlePtr MakeSectionBundle(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    const std::uint64_t generation,
    const std::shared_ptr<plan_env::EnvironmentValidityState>& validity =
        std::shared_ptr<plan_env::EnvironmentValidityState>()) {
  if (!path || path->empty()) return SectionPathBundlePtr();
  const std::uint64_t revision = path->pathRevision();
  std::shared_ptr<SectionPathBundle> bundle(new SectionPathBundle());
  bundle->path = path;
  bundle->frame = std::shared_ptr<const ContinuousPhaseNormalFrame>(
      new ContinuousPhaseNormalFrame(path, revision, revision));
  bundle->profile = MakeSectionProfile(path->endW());
  bundle->local_view_status = plan_env::LocalObstacleViewStatus::VALID;
  bundle->frame_id = "world";
  bundle->task_generation = generation;
  bundle->path_revision = revision;
  bundle->frame_revision = revision;
  bundle->environment_validity = validity
      ? validity : std::make_shared<plan_env::EnvironmentValidityState>();
  bundle->environment_change_mutex = std::make_shared<std::recursive_mutex>();
  return std::shared_ptr<const SectionPathBundle>(std::move(bundle));
}

MatchedAdapterInput MakeSectionInput(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    const double w = 0.4) {
  MatchedAdapterInput input;
  input.path = MakeStraightState(w, path ? path->pathRevision() : 7U);
  input.semantic_path_owner = path;
  input.semantic_path_start_w = path ? path->startW() : 0.0;
  input.semantic_path_end_w = path ? path->endW() : 3.0;
  input.position = input.path.p + Eigen::Vector3d(0.0, 0.1, 0.1);
  input.gains = MakeGains();
  input.dt = kDt;
  EXPECT_TRUE(RefreshLegacyGuidance(input));
  return input;
}

PhaseOffsetMatchedAdapterConfig MakeManualConfig() {
  PhaseOffsetMatchedAdapterConfig config;
  config.mode = PhaseOffsetMatchedMode::MANUAL;
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.observe_only = false;
  config.warmup_cycles = 100;
  config.u_w_rate_max = 100.0;
  config.u_delta_rate_max = 100.0;
  config.u_delta_abs_max = 0.40;
  config.tube_update_period = 0.10;
  config.normal_preview_policy.preview_horizon_w = 2.0;
  config.normal_preview_policy.sample_spacing_w = 0.10;
  config.normal_preview_policy.lower_nu = 0.02;
  config.normal_preview_policy.upper_nu = 2.0;
  config.normal_preview_policy.b_tight = 0.10;
  config.normal_preview_policy.b_open = 0.90;
  config.normal_preview_policy_explicit = true;
  config.tube_source = phase_offset_navigation::TubeSource::ESDF;
  config.tube.cross_section.search_extent = 3.0;
  config.tube.cross_section.minimum_reference_speed = 1e-8;
  config.section_build.max_obstacle_checks = 100000U;
  config.section_build.max_obstacles = 1000U;
  return config;
}

bool StageAndUpdate(PhaseOffsetMatchedAdapter& adapter,
                    const SectionPathBundlePtr& bundle,
                    const std::shared_ptr<const ContinuousPhasePath>& path,
                    MatchedAdapterOutput& output,
                    const double w = 0.4,
                    const std::shared_ptr<const ContinuousPhasePath>& source =
                        std::shared_ptr<const ContinuousPhasePath>(),
                    const double prefix_start =
                        std::numeric_limits<double>::quiet_NaN(),
                    const double prefix_end =
                        std::numeric_limits<double>::quiet_NaN()) {
  if (!adapter.stageSectionBundle(bundle, source, prefix_start, prefix_end)) {
    return false;
  }
  MatchedAdapterInput input = MakeSectionInput(path, w);
  input.section_bundle = bundle;
  input.g_des = Eigen::Vector3d::Zero();
  input.g_des_valid = true;
  return adapter.update(input, output);
}

TEST(PhaseOffsetMatchedAdapterActive, ZeroPortWarmupAndMismatchLatch) {
  PhaseOffsetMatchedAdapterConfig config;
  config.mode = PhaseOffsetMatchedMode::ACTIVE;
  config.warmup_cycles = 100;
  config.equivalence_tolerance = 1e-10;
  PhaseOffsetMatchedAdapter adapter(config);
  ASSERT_TRUE(adapter.configurationValid());

  MatchedAdapterInput input;
  input.path = MakeStraightState(0.4);
  input.position = input.path.p + Eigen::Vector3d(0.0, 0.1, 0.1);
  input.gains = MakeGains();
  ASSERT_TRUE(RefreshLegacyGuidance(input));

  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < config.warmup_cycles; ++cycle) {
    const bool selected = adapter.update(input, output);
    if (cycle + 1 < config.warmup_cycles) EXPECT_FALSE(selected);
  }
  EXPECT_TRUE(output.zero_comparison.valid);
  EXPECT_TRUE(output.zero_comparison.equivalent);
  EXPECT_TRUE(output.zero_gate_open);
  EXPECT_TRUE(output.selected);

  input.legacy.v_cmd.x() += 0.25;
  EXPECT_FALSE(adapter.update(input, output));
  EXPECT_TRUE(output.failure_latched);
  EXPECT_EQ(output.control_failure_reason,
            phase_offset_navigation::ControlFailureReason::ZERO_PORT_EQUIVALENCE);
  EXPECT_FALSE(output.selected);
}

TEST(PhaseOffsetMatchedAdapterSection,
     FailedPublishPreservesPendingAndRuntimeThenCommits) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  ASSERT_TRUE(adapter.configurationValid());
  adapter.zero_gate_open_ = true;
  const auto path = MakeStraightPath();
  const auto bundle = MakeSectionBundle(path, adapter.executionGeneration());
  MatchedAdapterOutput output;
  ASSERT_TRUE(StageAndUpdate(adapter, bundle, path, output));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const PendingPositionCommandCapture pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(pending.pending);
  ASSERT_TRUE(pending.valid);
  const double delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  const double next_delta = output.section_prepared_step->nextDelta();

  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      []() { return false; }, pending.identity));
  EXPECT_TRUE(adapter.hasPendingPositionCommand());
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w,
                   previous_before.u_w);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   previous_before.u_delta);

  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, pending.identity));
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  EXPECT_EQ(adapter.captureSectionBundle(), bundle);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), next_delta);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterSection,
     InvalidUpdateRetiresOlderPendingButPreservesCurrentAndStaged) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  ASSERT_TRUE(adapter.configurationValid());
  adapter.zero_gate_open_ = true;
  const auto path = MakeStraightPath();
  const auto bundle = MakeSectionBundle(path, adapter.executionGeneration());
  MatchedAdapterOutput output;
  ASSERT_TRUE(StageAndUpdate(adapter, bundle, path, output));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const PendingPositionCommandCapture before =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(before.pending);

  MatchedAdapterInput invalid = MakeSectionInput(path);
  invalid.section_bundle.reset();
  EXPECT_FALSE(adapter.update(invalid, output));
  const PendingPositionCommandCapture after =
      adapter.capturePendingPositionCommand();
  EXPECT_FALSE(after.pending);
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      []() { return true; }, before.identity));
  EXPECT_EQ(adapter.captureSectionBundle(), nullptr);
  EXPECT_EQ(adapter.capturePendingSectionBundle(), bundle);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterSection,
     StagingReplacementDoesNotReplaceAnAlreadyPreparedCommand) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  ASSERT_TRUE(adapter.configurationValid());
  adapter.zero_gate_open_ = true;
  const auto first_path = MakeStraightPath(0.0, 3.0, 7U);
  const auto first_bundle =
      MakeSectionBundle(first_path, adapter.executionGeneration());
  MatchedAdapterOutput output;
  ASSERT_TRUE(StageAndUpdate(adapter, first_bundle, first_path, output));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const PendingPositionCommandCapture first_pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(first_pending.pending);

  const auto staged_path = MakeStraightPath(0.0, 3.0, 8U);
  const auto staged_bundle =
      MakeSectionBundle(staged_path, adapter.executionGeneration());
  ASSERT_TRUE(adapter.stageSectionBundle(
      staged_bundle, std::shared_ptr<const ContinuousPhasePath>(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(adapter.capturePendingSectionBundle(), staged_bundle);
  const PendingPositionCommandCapture still_pending =
      adapter.capturePendingPositionCommand();
  EXPECT_EQ(still_pending.identity, first_pending.identity);
  EXPECT_EQ(still_pending.section_bundle, first_bundle);
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, first_pending.identity));
  EXPECT_EQ(adapter.captureSectionBundle(), first_bundle);
  EXPECT_EQ(adapter.capturePendingSectionBundle(), staged_bundle);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterSection,
     ObserveOnlyOrClosedZeroGateNeverCreatesPendingCommand) {
  for (const bool observe_only : {true, false}) {
    PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
    config.observe_only = observe_only;
    PhaseOffsetMatchedAdapter adapter(config);
    ASSERT_TRUE(adapter.configurationValid());
    adapter.zero_gate_open_ = false;
    const auto path = MakeStraightPath();
    const auto bundle = MakeSectionBundle(path, adapter.executionGeneration());
    MatchedAdapterOutput output;
    ASSERT_TRUE(adapter.stageSectionBundle(
        bundle, std::shared_ptr<const ContinuousPhasePath>(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()));
    MatchedAdapterInput input = MakeSectionInput(path);
    input.section_bundle = bundle;
    input.g_des_valid = true;
    EXPECT_TRUE(adapter.update(input, output));
    EXPECT_FALSE(output.selected);
    EXPECT_FALSE(adapter.hasPendingPositionCommand());
    EXPECT_EQ(adapter.captureSectionBundle(), nullptr);
    EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
    adapter.shutdown();
  }
}

TEST(PhaseOffsetMatchedAdapterSection,
     ValidReplacementMovesPendingButRetainsStagedCandidateUntilCommit) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  ASSERT_TRUE(adapter.configurationValid());
  adapter.zero_gate_open_ = true;
  const auto first_path = MakeStraightPath(0.0, 3.0, 7U);
  const auto first_bundle =
      MakeSectionBundle(first_path, adapter.executionGeneration());
  MatchedAdapterOutput output;
  ASSERT_TRUE(StageAndUpdate(adapter, first_bundle, first_path, output));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const PendingPositionCommandCapture first_pending =
      adapter.capturePendingPositionCommand();

  const auto second_path = MakeStraightPath(0.0, 3.0, 8U);
  const auto second_bundle =
      MakeSectionBundle(second_path, adapter.executionGeneration());
  ASSERT_TRUE(StageAndUpdate(adapter, second_bundle, second_path, output));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const PendingPositionCommandCapture second_pending =
      adapter.capturePendingPositionCommand();
  EXPECT_NE(second_pending.identity, first_pending.identity);
  EXPECT_EQ(second_pending.section_bundle, second_bundle);
  EXPECT_EQ(adapter.capturePendingSectionBundle(), second_bundle);
  EXPECT_EQ(adapter.captureSectionBundle(), nullptr);

  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, second_pending.identity));
  EXPECT_EQ(adapter.captureSectionBundle(), second_bundle);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterSection,
     TerminalGoalOverrideCancelsStaleEnvironmentOnlyAtZeroDelta) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  ASSERT_TRUE(adapter.configurationValid());
  adapter.zero_gate_open_ = true;
  const auto validity = std::make_shared<plan_env::EnvironmentValidityState>();
  const auto path = MakeStraightPath();
  const auto bundle =
      MakeSectionBundle(path, adapter.executionGeneration(), validity);
  MatchedAdapterOutput output;
  ASSERT_TRUE(StageAndUpdate(adapter, bundle, path, output));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const PendingPositionCommandCapture pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(pending.pending);
  {
    const std::lock_guard<std::recursive_mutex> lock(
        *bundle->environment_change_mutex);
    validity->valid = false;
  }
  int publish_count = 0;
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      [&publish_count]() {
        ++publish_count;
        return true;
      }, pending.identity, true));
  EXPECT_EQ(publish_count, 1);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  EXPECT_EQ(adapter.captureSectionBundle(), nullptr);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);

  // A nonzero retained offset cannot use the terminal cancellation escape.
  const auto valid_again =
      std::make_shared<plan_env::EnvironmentValidityState>();
  const auto next_bundle =
      MakeSectionBundle(path, adapter.executionGeneration(), valid_again);
  ASSERT_TRUE(StageAndUpdate(adapter, next_bundle, path, output));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const PendingPositionCommandCapture nonzero_pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(nonzero_pending.pending);
  adapter.runtime_->delta_ = 0.1;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      []() { return true; }, nonzero_pending.identity, true));
  EXPECT_TRUE(adapter.hasPendingPositionCommand());
  adapter.discardPendingPositionCommand();
  int bypass_publish_count = 0;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      [&bypass_publish_count]() {
        ++bypass_publish_count;
        return true;
      }, 0U, true));
  EXPECT_EQ(bypass_publish_count, 0);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterSection,
     ResetRetiresTaskAndPendingWithoutCommittingRuntime) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  ASSERT_TRUE(adapter.configurationValid());
  adapter.zero_gate_open_ = true;
  const auto path = MakeStraightPath();
  const auto bundle = MakeSectionBundle(path, adapter.executionGeneration());
  MatchedAdapterOutput output;
  ASSERT_TRUE(StageAndUpdate(adapter, bundle, path, output));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const std::uint64_t old_generation = adapter.executionGeneration();

  ASSERT_TRUE(adapter.resetForNewNavigationTask());
  EXPECT_GT(adapter.executionGeneration(), old_generation);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  EXPECT_EQ(adapter.captureSectionBundle(), nullptr);
  EXPECT_EQ(adapter.capturePendingSectionBundle(), nullptr);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterSection,
     CopiedPrefixCrossingRejectsPublishAndKeepsCurrentBundle) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  ASSERT_TRUE(adapter.configurationValid());
  adapter.zero_gate_open_ = true;
  const auto source_path = MakeStraightPath(0.0, 3.0, 7U);
  const auto source_bundle =
      MakeSectionBundle(source_path, adapter.executionGeneration());
  MatchedAdapterOutput output;
  ASSERT_TRUE(StageAndUpdate(adapter, source_bundle, source_path, output));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const PendingPositionCommandCapture source_pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, source_pending.identity));
  ASSERT_EQ(adapter.captureSectionBundle(), source_bundle);

  const auto replacement_path = MakeStraightPath(0.0, 3.0, 8U);
  const auto replacement_bundle =
      MakeSectionBundle(replacement_path, adapter.executionGeneration());
  ASSERT_TRUE(StageAndUpdate(adapter, replacement_bundle, replacement_path,
                             output, 0.4, source_path, 0.0, 0.4));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const PendingPositionCommandCapture pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(pending.pending);
  ASSERT_GT(pending.section_next_w, pending.section_copied_prefix_end_w);
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      []() { return true; }, pending.identity));
  EXPECT_EQ(adapter.captureSectionBundle(), source_bundle);
  EXPECT_TRUE(adapter.hasPendingPositionCommand());
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterSection,
     ShutdownRequestedBetweenCaptureAndPublishRejectsTransaction) {
  PhaseOffsetMatchedAdapter adapter(MakeManualConfig());
  ASSERT_TRUE(adapter.configurationValid());
  adapter.zero_gate_open_ = true;
  const auto path = MakeStraightPath();
  const auto bundle = MakeSectionBundle(path, adapter.executionGeneration());
  MatchedAdapterOutput output;
  ASSERT_TRUE(StageAndUpdate(adapter, bundle, path, output));
  ASSERT_TRUE(output.selected) << output.invalid_reason;
  const PendingPositionCommandCapture pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(pending.pending);

  // Model shutdown winning the task-publication lock after capture but before
  // the local publish callback.  No command callback may run in that window.
  adapter.shutdown_requested_.store(true, std::memory_order_release);
  int publish_count = 0;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      [&publish_count]() {
        ++publish_count;
        return true;
      }, pending.identity));
  EXPECT_EQ(publish_count, 0);
  adapter.shutdown();
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
