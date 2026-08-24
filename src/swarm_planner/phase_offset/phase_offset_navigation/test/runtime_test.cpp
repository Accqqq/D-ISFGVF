#include <gtest/gtest.h>

#include "phase_offset_navigation/phase_offset_runtime.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <type_traits>
#include <vector>

namespace phase_offset_navigation {
namespace {

RuntimePathSamples MakePath() {
  RuntimePathSamples samples;
  for (int index = 0; index <= 20; ++index) {
    phase_offset_core::PathDifferentialState state;
    state.w = 0.1 * index;
    state.p = Eigen::Vector3d(state.w, 0.0, 1.0 + 0.1 * state.w);
    state.p_w = Eigen::Vector3d(1.0, 0.0, 0.1);
    state.p_ww = Eigen::Vector3d::Zero();
    state.valid = true;
    samples.push_back(state);
  }
  return samples;
}

RuntimePathSamples MakeFrameBoundPath(const double tangent_speed) {
  RuntimePathSamples samples;
  for (int index = 0; index <= 4; ++index) {
    phase_offset_core::PathDifferentialState state;
    state.w = 0.1 * index;
    state.p = Eigen::Vector3d(state.w, 0.0, 1.0);
    state.p_w = Eigen::Vector3d(tangent_speed, 0.0, 0.0);
    state.p_ww = Eigen::Vector3d::Zero();
    state.T = Eigen::Vector3d::UnitX();
    state.N = Eigen::Vector3d::UnitY();
    state.N_w = Eigen::Vector3d::Zero();
    state.frame_valid = true;
    state.valid = true;
    samples.push_back(state);
  }
  return samples;
}

std::shared_ptr<const TubeProfile> FinalizeProfile(
    const std::shared_ptr<TubeProfile>& profile) {
  if (profile && !profile->samples.empty()) {
    profile->certified_segment_start_w = profile->samples.front().w;
    profile->certified_segment_end_w = profile->samples.back().w;
  }
  return profile;
}

RuntimeFutureStepContract MakeFutureContract(
    const std::shared_ptr<const TubeProfile>& profile,
    const double min_certified_forward_w = 0.0) {
  RuntimeFutureStepContract contract;
  if (!profile || profile->samples.size() < 2U) return contract;
  contract.tube_update_period = 0.10;
  contract.min_certified_forward_w = min_certified_forward_w;
  contract.profile_domain_end_w = profile->certified_segment_end_w;
  contract.evaluate = [](
      const RuntimeFutureStepInput& input,
      RuntimeFutureStepResult& output) {
    output = RuntimeFutureStepResult();
    if (!std::isfinite(input.phase) || !std::isfinite(input.delta) ||
        !input.matched_position.allFinite()) {
      return false;
    }
    output.path.w = input.phase;
    output.path.p = Eigen::Vector3d(
        input.phase, 0.0, 1.0 + 0.1 * input.phase);
    output.path.p_w = Eigen::Vector3d(1.0, 0.0, 0.1);
    output.path.p_ww = Eigen::Vector3d::Zero();
    output.path.valid = true;
    phase_offset_core::PhaseOffsetGeometryState geometry;
    if (!phase_offset_core::GeometryEvaluator().evaluate(
            output.path, input.matched_position, input.delta, geometry)) {
      return false;
    }
    output.matched_reference = geometry.r;
    output.matched_tangent = geometry.T;
    output.matched_derivative_norm = geometry.r_w.norm();
    output.base_v_cmd = geometry.T * 0.5;
    output.base_w_dot = 0.5;
    output.base_guidance_valid = true;
    output.valid = true;
    return true;
  };
  return contract;
}

RuntimeFutureStepContract MakePhaseLimitedFutureContract(
    const std::shared_ptr<const TubeProfile>& profile,
    const Eigen::Vector3d& base_v_cmd, const double base_w_dot,
    const double maximum_phase,
    const std::shared_ptr<std::vector<RuntimeFutureStepInput>>& calls) {
  RuntimeFutureStepContract contract;
  if (!profile || profile->samples.size() < 2U ||
      !base_v_cmd.allFinite() || !std::isfinite(base_w_dot)) {
    return contract;
  }
  contract.tube_update_period = 0.10;
  contract.profile_domain_end_w = profile->certified_segment_end_w;
  contract.evaluate = [base_v_cmd, base_w_dot, maximum_phase, calls](
      const RuntimeFutureStepInput& input,
      RuntimeFutureStepResult& output) {
    output = RuntimeFutureStepResult();
    if (calls) calls->push_back(input);
    if (!std::isfinite(input.phase) || !std::isfinite(input.delta) ||
        !std::isfinite(input.dt) || input.dt <= 0.0 ||
        !input.matched_position.allFinite() ||
        input.phase > maximum_phase + 1e-12) {
      return false;
    }
    output.path.w = input.phase;
    output.path.p = Eigen::Vector3d(
        input.phase, 0.0, 1.0 + 0.1 * input.phase);
    output.path.p_w = Eigen::Vector3d(1.0, 0.0, 0.1);
    output.path.p_ww = Eigen::Vector3d::Zero();
    output.path.valid = true;
    phase_offset_core::PhaseOffsetGeometryState geometry;
    if (!phase_offset_core::GeometryEvaluator().evaluate(
            output.path, input.matched_position, input.delta, geometry)) {
      return false;
    }
    output.matched_reference = geometry.r;
    output.matched_tangent = geometry.T;
    output.matched_derivative_norm = geometry.r_w.norm();
    output.base_v_cmd = base_v_cmd;
    output.base_w_dot = base_w_dot;
    output.base_guidance_valid = true;
    output.valid = true;
    return true;
  };
  return contract;
}

std::shared_ptr<const TubeProfile> MakeActiveProfile(double lower = -0.08,
                                                     double upper = 0.08) {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 2.0;
  profile->raw_complete = profile->filtered_complete = profile->complete = true;
  for (int index = 0; index <= 20; ++index) {
    TubeRawSample sample;
    sample.w = 0.1 * index;
    sample.p = Eigen::Vector3d(sample.w, 0.0, 1.0);
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.raw_lower = sample.filtered_lower = lower;
    sample.raw_upper = sample.filtered_upper = upper;
    sample.complete = true;
    profile->samples.push_back(sample);
  }
  return FinalizeProfile(profile);
}

std::shared_ptr<const TubeProfile> MakePosthocRejectingProfile() {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 2.0;
  profile->raw_complete = profile->filtered_complete = profile->complete = true;
  TubeRawSample current;
  current.w = 0.0;
  current.raw_lower = current.filtered_lower = -0.08;
  current.raw_upper = current.filtered_upper = 0.08;
  current.complete = true;
  TubeRawSample narrowing;
  narrowing.w = 0.015;
  narrowing.raw_lower = narrowing.filtered_lower = 0.07;
  narrowing.raw_upper = narrowing.filtered_upper = 0.08;
  narrowing.complete = true;
  TubeRawSample tail = narrowing;
  tail.w = 2.0;
  profile->samples.push_back(current);
  profile->samples.push_back(narrowing);
  profile->samples.push_back(tail);
  return FinalizeProfile(profile);
}

std::shared_ptr<const TubeProfile> MakeCrossKnotProfile() {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 2.0;
  profile->raw_complete = profile->filtered_complete = profile->complete = true;
  const double upper[] = {0.08, 0.08, 0.00, 0.08};
  for (int index = 0; index <= 3; ++index) {
    TubeRawSample sample;
    sample.w = 0.05 * index;
    sample.raw_lower = sample.filtered_lower = -0.08;
    sample.raw_upper = sample.filtered_upper = upper[index];
    sample.complete = true;
    profile->samples.push_back(sample);
  }
  return FinalizeProfile(profile);
}

std::shared_ptr<const TubeProfile> MakeTightCrossKnotProfile() {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 2.0;
  profile->raw_complete = profile->filtered_complete = profile->complete = true;
  const double upper[] = {0.08, 0.00, 0.08};
  for (int index = 0; index <= 2; ++index) {
    TubeRawSample sample;
    sample.w = 0.05 * index;
    sample.raw_lower = sample.filtered_lower = -0.08;
    sample.raw_upper = sample.filtered_upper = upper[index];
    sample.complete = true;
    profile->samples.push_back(sample);
  }
  return FinalizeProfile(profile);
}

std::shared_ptr<const TubeProfile> MakeImpossibleCrossKnotProfile() {
  auto profile = std::make_shared<TubeProfile>();
  profile->source = TubeSource::FIXED;
  profile->preview_start_w = 0.0;
  profile->preview_end_w = 0.10;
  profile->raw_complete = profile->filtered_complete = profile->complete = true;
  const double w[] = {0.0, 0.005, 0.10};
  const double lower[] = {-0.08, 0.05, 0.05};
  const double upper[] = {0.08, 0.08, 0.08};
  for (int index = 0; index <= 2; ++index) {
    TubeRawSample sample;
    sample.w = w[index];
    sample.raw_lower = sample.filtered_lower = lower[index];
    sample.raw_upper = sample.filtered_upper = upper[index];
    sample.complete = true;
    profile->samples.push_back(sample);
  }
  return FinalizeProfile(profile);
}

PhaseOffsetRuntimeConfig MakeConfig(TubeSource source) {
  PhaseOffsetRuntimeConfig config;
  config.manual.profile_period = 0.4;
  config.manual.u_delta_abs_max = 0.5;
  config.manual.u_w_rate_max = 100.0;
  config.manual.u_delta_rate_max = 100.0;
  config.tube.source = source;
  config.tube.tracking_error_bound = 0.15;
  return config;
}

RuntimeInstalledTubeView MakeView(TubeEpochState state = TubeEpochState::ROLLING,
                                  std::shared_ptr<const TubeProfile> profile = MakeActiveProfile()) {
  RuntimeInstalledTubeView view;
  view.active_profile = std::move(profile);
  view.epoch_status.state = state;
  view.epoch_status.active_available = static_cast<bool>(view.active_profile);
  view.epoch_status.active_current_validation_valid =
      static_cast<bool>(view.active_profile) &&
      state != TubeEpochState::CERTIFICATE_DENIED;
  view.epoch_status.certificate_denied =
      state == TubeEpochState::CERTIFICATE_DENIED;
  view.epoch_status.active_tube_epoch = view.active_profile ? 1U : 0U;
  view.epoch_status.current_state_admissible = true;
  return view;
}

RuntimePrepareInput MakeInput(const RuntimePathSamples& path,
                              const RuntimeInstalledTubeView& view,
                              bool gate_open = true) {
  RuntimePrepareInput input;
  input.current_path = path.front();
  input.position = path.front().p;
  input.tube_view = view;
  input.dt = 0.02;
  if (view.active_profile) {
    input.future_step = MakeFutureContract(view.active_profile);
  }
  input.zero_gate_open = gate_open;
  return input;
}

void Refresh(PhaseOffsetRuntime& runtime, const RuntimePathSamples& path,
             std::uint64_t revision = 1U) {
  RuntimePreflightInput input;
  input.path = path;
  input.position = path.front().p;
  input.path_source_revision = revision;
  ASSERT_TRUE(runtime.refreshPreflight(input));
}

bool Complete(PhaseOffsetRuntime& runtime, const RuntimePreparedStep& prepared,
              RuntimeStepOutput& output) {
  return runtime.complete(prepared, prepared.geometry.T * 0.5, 0.5, true,
                          output);
}

RuntimeDryRunInput MakeDryRunInput(const RuntimePathSamples& path,
                                   const RuntimeInstalledTubeView& view) {
  RuntimeDryRunInput input;
  input.preflight.path = path;
  input.preflight.position = path.front().p;
  input.preflight.path_source_revision = 2U;
  input.prepare = MakeInput(path, view, true);
  input.base_v_cmd = input.prepare.current_path.p_w.normalized() * 0.5;
  input.base_w_dot = 0.5;
  input.base_guidance_valid = true;
  return input;
}

TEST(PhaseOffsetRuntimeTest, NoneKeepsA4FinalPortDeltaIntegration) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  ASSERT_TRUE(runtime.configurationValid());
  Refresh(runtime, path);
  RuntimePreparedStep first;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), first));
  EXPECT_EQ(first.execution.mode, RuntimeExecutionMode::NO_TUBE_REQUIRED);
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, first, output));
  RuntimePreparedStep next;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), next));
  EXPECT_NEAR(next.delta, output.delta + 0.02 * output.projection.final_port.u_delta, 1e-12);
  ASSERT_TRUE(Complete(runtime, next, output));
  EXPECT_NEAR(output.matched.delta_dot, output.projection.final_port.u_delta, 1e-15);
  EXPECT_LE(output.matched.matched_residual_norm, 1e-12);
}

TEST(PhaseOffsetRuntimeTest,
     NewNavigationTaskResetDropsOnlyTaskScopedExecutionHistory) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  ASSERT_TRUE(runtime.configurationValid());
  Refresh(runtime, path, 41U);

  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                              prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                              prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_TRUE(runtime.hasExecutedOffsetAuthority());
  EXPECT_NE(0.0, runtime.retainedDelta());
  EXPECT_TRUE(std::abs(runtime.previousFinalPort().u_delta) > 0.0 ||
              std::abs(runtime.previousFinalPort().u_w) > 0.0);

  runtime.resetForNewNavigationTask();
  EXPECT_TRUE(runtime.configurationValid());
  EXPECT_DOUBLE_EQ(0.0, runtime.retainedDelta());
  EXPECT_DOUBLE_EQ(0.0, runtime.previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(0.0, runtime.previousFinalPort().u_delta);
  EXPECT_FALSE(runtime.hasExecutedOffsetAuthority());
  EXPECT_TRUE(runtime.hasPendingOrActiveOffsetIntent());
  // The old path's preflight may not authorize the new task.
  EXPECT_FALSE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                               prepared));

  // The operation is idempotent and preserves configuration validity.
  runtime.resetForNewNavigationTask();
  EXPECT_TRUE(runtime.configurationValid());
  EXPECT_DOUBLE_EQ(0.0, runtime.retainedDelta());
  EXPECT_FALSE(runtime.hasExecutedOffsetAuthority());
}

TEST(PhaseOffsetRuntimeTest, OrdinaryPrepareDoesNotImplicitlyResetTaskState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  Refresh(runtime, path, 42U);
  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                              prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  const double retained_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand port_before = runtime.previousFinalPort();
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()),
                              prepared));
  EXPECT_DOUBLE_EQ(retained_before, prepared.delta);
  EXPECT_DOUBLE_EQ(port_before.u_w, runtime.previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(port_before.u_delta, runtime.previousFinalPort().u_delta);
}

TEST(PhaseOffsetRuntimeTest, PreflightIsExplicitAndRevisionScoped) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  Refresh(runtime, path, 4U);
  RuntimePreflightInput same_revision;
  same_revision.position = path.front().p;
  same_revision.path_source_revision = 4U;
  EXPECT_TRUE(runtime.refreshPreflight(same_revision));
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  EXPECT_TRUE(prepared.preflight.complete);
  RuntimePreflightInput changed = same_revision;
  changed.path_source_revision = 5U;
  EXPECT_FALSE(runtime.refreshPreflight(changed));
  EXPECT_FALSE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  EXPECT_FALSE(prepared.execution.fatal_control_failure);
}

TEST(PhaseOffsetRuntimeTest, ConfiguredMinimumReferenceSpeedAgreesWithFramePreflight) {
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::NONE);
  config.tube.minimum_reference_speed = 0.50;
  PhaseOffsetRuntime slow(config);
  RuntimePreflightInput slow_input;
  slow_input.path = MakeFrameBoundPath(0.20);
  slow_input.position = slow_input.path.front().p;
  slow_input.path_source_revision = 1U;
  EXPECT_FALSE(slow.refreshPreflight(slow_input));

  PhaseOffsetRuntime fast(config);
  RuntimePreflightInput fast_input = slow_input;
  fast_input.path = MakeFrameBoundPath(0.80);
  fast_input.position = fast_input.path.front().p;
  EXPECT_TRUE(fast.refreshPreflight(fast_input));
}

TEST(PhaseOffsetRuntimeTest, GateClosedEvaluatesExactPortButCannotSelectOrCommit) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView(), false), prepared));
  EXPECT_TRUE(prepared.execution.current_geometry_valid);
  EXPECT_TRUE(prepared.execution.current_bounds_valid);
  EXPECT_TRUE(prepared.execution.retained_delta_current_inside);
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_TRUE(prepared.requires_base_guidance);
  EXPECT_DOUBLE_EQ(prepared.delta, 0.0);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  EXPECT_TRUE(output.valid);
  EXPECT_FALSE(output.selected);
  EXPECT_TRUE(output.projection.valid);
  EXPECT_TRUE(output.matched.valid);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest,
     DryRunFailureDoesNotMutateLivePreflightDeltaOrPreviousPort) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path, 1U);
  RuntimePreparedStep prepared;
  RuntimeStepOutput committed;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, committed));
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();

  RuntimeDryRunInput staged = MakeDryRunInput(path, MakeView());
  staged.preflight.path.clear();
  RuntimeDryRunResult result;
  EXPECT_FALSE(runtime.dryRun(staged, result));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);

  // The failed staging revision must not replace the live revision's
  // preflight.  A normal live prepare is still executable afterwards.
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  EXPECT_TRUE(prepared.preflight.complete);
}

TEST(PhaseOffsetRuntimeTest,
     DryRunUsesExactPortWithoutCommittingLiveRuntimeState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path, 1U);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();

  RuntimeDryRunResult result;
  ASSERT_TRUE(runtime.dryRun(MakeDryRunInput(path, MakeView()), result));
  EXPECT_TRUE(result.step.valid);
  EXPECT_TRUE(result.step.projection.valid);
  EXPECT_TRUE(result.step.matched.valid);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest,
     DryRunLeavesFollowupLivePrepareAndCompleteBitwiseEquivalentToBaseline) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path, 1U);
  // Start the profile once so this covers copied profile lifecycle state as
  // well as retained delta and the previous final port.
  RuntimePreparedStep initial;
  RuntimeStepOutput initial_output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), initial));
  ASSERT_TRUE(Complete(runtime, initial, initial_output));
  PhaseOffsetRuntime baseline(runtime);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();

  RuntimeDryRunResult dry_run;
  ASSERT_TRUE(runtime.dryRun(MakeDryRunInput(path, MakeView()), dry_run));
  const double delta_after_dry_run = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_after_dry_run =
      runtime.previousFinalPort();
  EXPECT_EQ(std::memcmp(&delta_before, &delta_after_dry_run, sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&previous_before.u_w, &previous_after_dry_run.u_w,
                        sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&previous_before.u_delta, &previous_after_dry_run.u_delta,
                        sizeof(double)), 0);

  RuntimePreparedStep actual;
  RuntimePreparedStep expected;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), actual));
  ASSERT_TRUE(baseline.prepare(MakeInput(path, MakeView()), expected));
  RuntimeStepOutput actual_output;
  RuntimeStepOutput expected_output;
  ASSERT_TRUE(Complete(runtime, actual, actual_output));
  ASSERT_TRUE(Complete(baseline, expected, expected_output));
  EXPECT_EQ(std::memcmp(&actual_output.projection.final_port.u_w,
                        &expected_output.projection.final_port.u_w,
                        sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&actual_output.projection.final_port.u_delta,
                        &expected_output.projection.final_port.u_delta,
                        sizeof(double)), 0);
  const double actual_delta = runtime.retainedDelta();
  const double expected_delta = baseline.retainedDelta();
  EXPECT_EQ(std::memcmp(&actual_delta, &expected_delta, sizeof(double)), 0);
  const phase_offset_core::PortCommand actual_previous = runtime.previousFinalPort();
  const phase_offset_core::PortCommand expected_previous =
      baseline.previousFinalPort();
  EXPECT_EQ(std::memcmp(&actual_previous.u_w, &expected_previous.u_w,
                        sizeof(double)), 0);
  EXPECT_EQ(std::memcmp(&actual_previous.u_delta, &expected_previous.u_delta,
                        sizeof(double)), 0);
}

TEST(PhaseOffsetRuntimeTest, CompleteCommitsSelectedPortImmediately) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_TRUE(output.selected);
  EXPECT_DOUBLE_EQ(output.projection.next_delta, runtime.retainedDelta());
  EXPECT_DOUBLE_EQ(output.projection.final_port.u_w,
                   runtime.previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(output.projection.final_port.u_delta,
                   runtime.previousFinalPort().u_delta);

  RuntimePreparedStep next;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), next));
  EXPECT_DOUBLE_EQ(output.projection.next_delta, next.delta);
}

TEST(PhaseOffsetRuntimeTest, InvalidBaseGuidanceDoesNotAdvanceRuntimeState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  RuntimeStepOutput committed;
  ASSERT_TRUE(Complete(runtime, prepared, committed));
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  RuntimeStepOutput invalid;
  EXPECT_FALSE(runtime.complete(prepared, prepared.geometry.T * 0.5, 0.5,
                                false, invalid));
  EXPECT_EQ(invalid.execution.mode, RuntimeExecutionMode::FATAL_CONTROL_FAILURE);
  EXPECT_EQ(invalid.execution.failure_reason,
            ControlFailureReason::BASE_GUIDANCE_INVARIANT);
  EXPECT_DOUBLE_EQ(delta_before, runtime.retainedDelta());
  EXPECT_DOUBLE_EQ(previous_before.u_w, runtime.previousFinalPort().u_w);
  EXPECT_DOUBLE_EQ(previous_before.u_delta, runtime.previousFinalPort().u_delta);
}

TEST(PhaseOffsetRuntimeTest, GateClosedTrackingExcursionKeepsCertificateSeparateFromSelection) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePrepareInput input = MakeInput(path, MakeView(), false);
  input.position.y() = 0.16;
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(prepared.execution.tracking_within_bound);
  EXPECT_FALSE(prepared.execution.certificate_denied);
  EXPECT_TRUE(prepared.requires_base_guidance);
  RuntimeStepOutput output;
  EXPECT_TRUE(Complete(runtime, prepared, output));
  EXPECT_FALSE(output.selected);
  EXPECT_FALSE(output.execution.certificate_denied);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, RollingFixedViewExecutesAndUsesSameFinalPortEverywhere) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_TRUE(output.selected);
  EXPECT_NEAR(output.matched.delta_dot, output.projection.final_port.u_delta, 1e-15);
  EXPECT_LE(output.matched.matched_residual_norm, 1e-12);
}

TEST(PhaseOffsetRuntimeTest, ActivePointerReplacementDoesNotResetDeltaOrPreviousPort) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  const double expected_delta = output.delta + 0.02 * output.projection.final_port.u_delta;
  auto replacement = MakeActiveProfile(-0.06, 0.06);
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView(TubeEpochState::ROLLING, replacement)), prepared));
  EXPECT_NEAR(prepared.delta, expected_delta, 1e-12);
  ASSERT_TRUE(Complete(runtime, prepared, output));
  EXPECT_LE(output.matched.matched_residual_norm, 1e-12);
}

TEST(PhaseOffsetRuntimeTest, WaitingCandidateWithValidActiveProfileKeepsExecution) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(
      MakeInput(path, MakeView(TubeEpochState::WAITING_FOR_CANDIDATE)), prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(prepared.execution.transient_blocked);
  RuntimeStepOutput output;
  EXPECT_TRUE(Complete(runtime, prepared, output));
  EXPECT_TRUE(output.selected);
}

TEST(PhaseOffsetRuntimeTest, WaitingBeforeStartBlocksWithoutPermanentLatch) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  EXPECT_FALSE(runtime.prepare(MakeInput(path, MakeView(TubeEpochState::WAITING_FOR_CANDIDATE,
                                                         nullptr)), prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::WAITING_FOR_CANDIDATE);
  EXPECT_FALSE(prepared.execution.fatal_control_failure);
}

TEST(PhaseOffsetRuntimeTest,
     LatestCandidateExcludingRetainedDeltaBlocksOldActiveExecution) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimeInstalledTubeView view = MakeView();
  // The old active profile still contains zero, but TubeEpochManager has
  // invalidated its current certificate because the latest Candidate does
  // not.  Runtime must wait rather than use the old profile to side-step.
  view.epoch_status.active_current_validation_valid = false;
  RuntimePreparedStep prepared;
  EXPECT_FALSE(runtime.prepare(MakeInput(path, view), prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::WAITING_FOR_CANDIDATE);
  EXPECT_TRUE(prepared.execution.transient_blocked);
  EXPECT_FALSE(prepared.requires_base_guidance);
  EXPECT_DOUBLE_EQ(prepared.delta, 0.0);
}

TEST(PhaseOffsetRuntimeTest, IncompleteCandidateDoesNotDropRetainedActiveControl) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_TRUE(runtime.prepare(
      MakeInput(path, MakeView(TubeEpochState::WAITING_FOR_CANDIDATE)), prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(prepared.execution.transient_blocked);
  EXPECT_NEAR(prepared.delta, output.delta +
      0.02 * output.projection.final_port.u_delta, 1e-12);
}

TEST(PhaseOffsetRuntimeTest, LivePositiveFailureReprojectsSameRawPortAsSafe) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_w_rate_max = 0.0;
  config.tube.invariant_gain = 10.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  prepared.raw_port.u_w = 0.1;
  prepared.raw_port.u_delta = 0.1;
  RuntimeStepOutput output;
  EXPECT_TRUE(runtime.complete(prepared, Eigen::Vector3d::Zero(), 0.0, true,
                               output));
  EXPECT_TRUE(output.selected);
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::SAFETY_PRIORITY);
  EXPECT_GE(output.projection.final_w_dot, -1e-12);
  EXPECT_GE(output.projection.final_tangent_speed, -1e-12);
  EXPECT_FALSE(output.execution.certificate_denied);
  EXPECT_DOUBLE_EQ(output.raw_port.u_delta, 0.1);
  EXPECT_DOUBLE_EQ(output.projection.final_port.u_delta, 0.1);
}

TEST(PhaseOffsetRuntimeTest,
     ExactPwlSingleStepWithoutShortHorizonContinuationIsDenied) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(
      MakeInput(path, MakeView(TubeEpochState::ROLLING,
                               MakePosthocRejectingProfile())), prepared));
  RuntimeStepOutput output;
  EXPECT_FALSE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_TRUE(output.execution.certificate_denied);
  EXPECT_NE(output.invalid_reason.find(
                "U+ first_failure_step=1 category=projector"),
            std::string::npos);
  EXPECT_NE(output.invalid_reason.find(
                "U_safe first_failure_step=1 category=projector"),
            std::string::npos);
}

TEST(PhaseOffsetRuntimeTest, ExactPwlCrossKnotProjectionKeepsHeldStepInsideTube) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_w_rate_max = 100.0;
  config.manual.u_delta_rate_max = 100.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, MakeCrossKnotProfile()));
  input.dt = 0.02;
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  // w+ crosses 0.05 and 0.10.  The PWL projector must use the actual narrow
  // knot rather than accept the current upper tangent of zero.
  prepared.raw_port.u_w = 0.0;
  prepared.raw_port.u_delta = 0.25;
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  EXPECT_TRUE(output.next_bounds.valid);
  EXPECT_LE(output.projection.next_delta, output.next_bounds.upper + 1e-12);
  EXPECT_GE(output.projection.next_delta, output.next_bounds.lower - 1e-12);
}

TEST(PhaseOffsetRuntimeTest, ExactPwlFailureDoesNotCommitLiveState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_delta_abs_max = 0.01;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, MakeImpossibleCrossKnotProfile()));
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  prepared.raw_port.u_delta = 0.25;
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  RuntimeStepOutput output;
  EXPECT_FALSE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, ExactPwlFallsBackFromPositiveToSafeProgress) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.phase_dot_min = 0.02;
  config.manual.tangent_speed_min = 0.02;
  config.manual.u_w_abs_max = 0.0;
  config.manual.u_w_rate_max = 0.0;
  config.manual.u_delta_abs_max = 0.0;
  config.manual.u_delta_rate_max = 0.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(
      path, MakeView(TubeEpochState::ROLLING, MakeTightCrossKnotProfile())), prepared));
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.complete(prepared, Eigen::Vector3d::Zero(), 0.0, true, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::SAFETY_PRIORITY);
  EXPECT_DOUBLE_EQ(output.projection.final_w_dot, 0.0);
  EXPECT_FALSE(output.execution.certificate_denied);
}

TEST(PhaseOffsetRuntimeTest,
     PositiveSingleStepCanFailViabilityThenSafeHoldProvidesWitness) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_w_abs_max = 0.12;
  config.manual.u_w_rate_max = 100.0;
  config.manual.u_delta_rate_max = 100.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  const std::shared_ptr<const TubeProfile> profile = MakeActiveProfile();
  const std::shared_ptr<std::vector<RuntimeFutureStepInput>> calls(
      new std::vector<RuntimeFutureStepInput>());
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, profile));
  // U+ must advance by the positive phase/tangent margins.  Its first held
  // step is locally exact-feasible, but the immutable future evaluator rejects
  // that positive phase.  U_safe may hold at w=0 until the next refresh.
  input.future_step = MakePhaseLimitedFutureContract(
      profile, Eigen::Vector3d::Zero(), 0.0, 1e-4, calls);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  prepared.raw_port = phase_offset_core::PortCommand();
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.complete(prepared, Eigen::Vector3d::Zero(), 0.0, true,
                               output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::SAFETY_PRIORITY);
  EXPECT_NEAR(output.projection.final_w_dot, 0.0, 1e-12);
  EXPECT_TRUE(output.selected);
  bool saw_rejected_positive_phase = false;
  for (const RuntimeFutureStepInput& call : *calls) {
    saw_rejected_positive_phase =
        saw_rejected_positive_phase || call.phase > 1e-4;
  }
  EXPECT_TRUE(saw_rejected_positive_phase);
}

TEST(PhaseOffsetRuntimeTest,
     DeterministicSearchUsesNonGreedyRootPortWhenItIsTheOnlyContinuation) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_w_abs_max = 0.12;
  config.manual.u_w_rate_max = 100.0;
  config.manual.u_delta_rate_max = 100.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  const std::shared_ptr<const TubeProfile> profile = MakeActiveProfile();
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, profile));
  // The nearest root projection is u_w=0.10.  It is exact-feasible for one
  // held step, but reaches a phase from which the next positive step exceeds
  // this immutable profile's future contract.  The lower polygon vertex
  // u_w=phase_dot_min remains viable through the full refresh horizon.
  input.future_step = MakePhaseLimitedFutureContract(
      profile, Eigen::Vector3d::Zero(), 0.0, 0.0021,
      std::shared_ptr<std::vector<RuntimeFutureStepInput>>());
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  prepared.raw_port = phase_offset_core::PortCommand();
  prepared.raw_port.u_w = 0.10;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.complete(prepared, Eigen::Vector3d::Zero(), 0.0, true,
                               output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_LT(output.projection.final_port.u_w, 0.05);
  EXPECT_GE(output.projection.final_port.u_w,
            config.manual.phase_dot_min - 1e-12);
}

TEST(PhaseOffsetRuntimeTest,
     MissingOrMismatchedFutureContractDeniesWithoutCommittingRuntimeState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();

  RuntimePreparedStep missing;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), missing));
  missing.future_step = RuntimeFutureStepContract();
  RuntimeStepOutput missing_output;
  EXPECT_FALSE(Complete(runtime, missing, missing_output));
  EXPECT_EQ(missing_output.execution.mode,
            RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_NE(missing_output.invalid_reason.find(
                "U+ first_failure_step=0 category=contract"),
            std::string::npos);
  EXPECT_NE(missing_output.invalid_reason.find(
                "U_safe first_failure_step=0 category=contract"),
            std::string::npos);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);

  RuntimePreparedStep mismatched;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), mismatched));
  mismatched.future_step.profile_domain_end_w += 0.01;
  RuntimeStepOutput mismatched_output;
  EXPECT_FALSE(Complete(runtime, mismatched, mismatched_output));
  EXPECT_EQ(mismatched_output.execution.mode,
            RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_NE(mismatched_output.invalid_reason.find(
                "U+ first_failure_step=0 category=contract"),
            std::string::npos);
  EXPECT_NE(mismatched_output.invalid_reason.find(
                "U_safe first_failure_step=0 category=contract"),
            std::string::npos);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest,
     InvalidFutureContractDryRunDeniesWithoutCommittingRuntimeState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();
  RuntimeDryRunInput dry = MakeDryRunInput(path, MakeView());
  dry.prepare.future_step.profile_domain_end_w += 0.01;
  RuntimeDryRunResult result;
  EXPECT_FALSE(runtime.dryRun(dry, result));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest,
     FutureEvaluatorIsCalledForEachPredictedStepWithEvolvingState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  const std::shared_ptr<const TubeProfile> profile = MakeActiveProfile();
  const std::shared_ptr<std::vector<RuntimeFutureStepInput>> calls(
      new std::vector<RuntimeFutureStepInput>());
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, profile));
  RuntimeFutureStepContract contract = MakeFutureContract(profile);
  const RuntimeFutureStepEvaluator evaluator = contract.evaluate;
  contract.evaluate = [calls, evaluator](const RuntimeFutureStepInput& step,
                                         RuntimeFutureStepResult& result) {
    calls->push_back(step);
    return evaluator(step, result);
  };
  input.future_step = contract;
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  RuntimeStepOutput output;
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_EQ(calls->size(), 4U);
  for (std::size_t index = 0U; index < 4U; ++index) {
    EXPECT_EQ((*calls)[index].step_index, index + 1U);
    EXPECT_TRUE((*calls)[index].matched_position.allFinite());
    EXPECT_TRUE((*calls)[index].previous_matched_reference.allFinite());
    if (index > 0U) {
      EXPECT_GT((*calls)[index].phase, (*calls)[index - 1U].phase);
    }
  }
}

TEST(PhaseOffsetRuntimeTest,
     FutureEvaluatorStepTwoFailureReportsDeepestWitnessFrontier) {
  const RuntimePathSamples path = MakePath();
  const std::shared_ptr<const TubeProfile> profile = MakeActiveProfile();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, profile));
  RuntimeFutureStepContract contract = MakeFutureContract(profile);
  const RuntimeFutureStepEvaluator evaluator = contract.evaluate;
  contract.evaluate = [evaluator](const RuntimeFutureStepInput& step,
                                  RuntimeFutureStepResult& result) {
    if (step.step_index == 2U) {
      result = RuntimeFutureStepResult();
      result.invalid_reason = "fixture rejects step two";
      return false;
    }
    return evaluator(step, result);
  };
  input.future_step = contract;
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      runtime.previousFinalPort();
  RuntimeStepOutput output;
  EXPECT_FALSE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_NE(output.invalid_reason.find(
                "U+ first_failure_step=2 category=future evaluator "
                "detail=fixture rejects step two"),
            std::string::npos);
  EXPECT_NE(output.invalid_reason.find(
                "U_safe first_failure_step=2 category=future evaluator "
                "detail=fixture rejects step two"),
            std::string::npos);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta,
                   previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, ExactPwlBothProgressSetsEmptyDoesNotCommit) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_delta_abs_max = 0.0;
  config.manual.u_delta_rate_max = 0.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  RuntimePrepareInput input = MakeInput(
      path, MakeView(TubeEpochState::ROLLING, MakeImpossibleCrossKnotProfile()));
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(input, prepared));
  prepared.raw_port.u_delta = 0.25;
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  RuntimeStepOutput output;
  EXPECT_FALSE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_NE(output.invalid_reason.find(
                "U+ first_failure_step=0 category=projector"),
            std::string::npos);
  EXPECT_NE(output.invalid_reason.find(
                "U_safe first_failure_step=0 category=projector"),
            std::string::npos);
  EXPECT_NE(output.invalid_reason.find(
                "constraint=joint port polygon; "
                "joint port feasible polygon is empty"),
            std::string::npos);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, ExactPwlDryRunFailureDoesNotCommitState) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntimeConfig config = MakeConfig(TubeSource::FIXED);
  config.manual.u_delta_abs_max = 0.0;
  config.manual.u_delta_rate_max = 0.0;
  PhaseOffsetRuntime runtime(config);
  Refresh(runtime, path);
  const double delta_before = runtime.retainedDelta();
  const phase_offset_core::PortCommand previous_before = runtime.previousFinalPort();
  RuntimeDryRunInput dry = MakeDryRunInput(
      path, MakeView(TubeEpochState::ROLLING, MakeImpossibleCrossKnotProfile()));
  dry.prepare.dt = 0.02;
  RuntimeDryRunResult result;
  EXPECT_FALSE(runtime.dryRun(dry, result));
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_w, previous_before.u_w);
  EXPECT_DOUBLE_EQ(runtime.previousFinalPort().u_delta, previous_before.u_delta);
}

TEST(PhaseOffsetRuntimeTest, TrackingBoundRevokesOnlyCertificateAndFatalGeometryStaysFatal) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePrepareInput tracking = MakeInput(path, MakeView());
  tracking.position.y() = 1.0;
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(tracking, prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(prepared.execution.tracking_within_bound);
  EXPECT_FALSE(prepared.execution.certificate_denied);
  EXPECT_FALSE(prepared.execution.fatal_control_failure);
  RuntimeStepOutput recovery;
  ASSERT_TRUE(Complete(runtime, prepared, recovery));
  EXPECT_TRUE(recovery.selected);
  EXPECT_EQ(recovery.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(recovery.execution.certificate_denied);
  RuntimePrepareInput invalid = MakeInput(path, MakeView());
  invalid.current_path.valid = false;
  EXPECT_FALSE(runtime.prepare(invalid, prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::FATAL_CONTROL_FAILURE);
  EXPECT_TRUE(prepared.execution.fatal_control_failure);
}

TEST(PhaseOffsetRuntimeTest, CertificateDenialDoesNotMutateDeltaOrPreviousPort) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  const double retained_before = output.delta +
      0.02 * output.projection.final_port.u_delta;

  EXPECT_FALSE(runtime.prepare(
      MakeInput(path, MakeView(TubeEpochState::CERTIFICATE_DENIED)), prepared));
  EXPECT_EQ(prepared.execution.mode, RuntimeExecutionMode::CERTIFICATE_DENIED);
  EXPECT_TRUE(prepared.execution.certificate_denied);
  EXPECT_FALSE(prepared.execution.fatal_control_failure);
  EXPECT_NEAR(prepared.delta, retained_before, 1e-12);
}

TEST(PhaseOffsetRuntimeTest,
     DiagnosticInvariantConflictDoesNotRejectExactPwlWitness) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView()), prepared));
  // The current tangent invariant is deliberately contradictory, while the
  // exact PWL profile remains wide and continuously viable.  G2h makes the
  // former diagnostic-only in this exact-PWL path.
  prepared.current_bounds.lower = -0.001;
  prepared.current_bounds.upper = 0.08;
  prepared.current_bounds.lower_w = 100.0;
  prepared.current_bounds.upper_w = 0.0;
  RuntimeStepOutput output;
  EXPECT_TRUE(Complete(runtime, prepared, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(output.execution.certificate_denied);
  EXPECT_FALSE(output.execution.fatal_control_failure);
  EXPECT_FALSE(output.execution.genuine_fatal_invariant);
  EXPECT_EQ(output.execution.failure_reason, ControlFailureReason::NONE);
}

TEST(PhaseOffsetRuntimeTest,
     ChangedLiveBaseFactsRecomputeExactPwlWithoutInvariantLatch) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  const RuntimeInstalledTubeView installed = MakeView();
  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, installed), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  // The high-rate base phase fact has changed.  Exact PWL viability is
  // recomputed from the current base facts; a contradictory tangent
  // invariant remains diagnostic and cannot create a false empty set.
  ASSERT_TRUE(runtime.prepare(MakeInput(path, installed), prepared));
  prepared.current_bounds.lower = -0.001;
  prepared.current_bounds.upper = 0.08;
  prepared.current_bounds.lower_w = 100.0;
  prepared.current_bounds.upper_w = 0.0;
  EXPECT_TRUE(runtime.complete(prepared, prepared.geometry.T * 0.5, 0.25,
                               true, output));
  EXPECT_EQ(output.execution.mode, RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(output.execution.certificate_denied);
  EXPECT_FALSE(output.execution.fatal_control_failure);
  EXPECT_FALSE(output.execution.genuine_fatal_invariant);
  EXPECT_EQ(output.execution.failure_reason, ControlFailureReason::NONE);
  EXPECT_LT(output.projection.lower_invariant_residual, 0.0);
  EXPECT_DOUBLE_EQ(runtime.retainedDelta(), output.projection.next_delta);
}

TEST(PhaseOffsetRuntimeTest, ImmutableActiveViewIsConsumedReadOnly) {
  static_assert(std::is_same<decltype(RuntimeInstalledTubeView().active_profile),
                             std::shared_ptr<const TubeProfile>>::value,
                "Runtime must consume immutable active profiles");
  const RuntimePathSamples path = MakePath();
  const auto profile = MakeActiveProfile();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::FIXED));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, MakeView(TubeEpochState::ROLLING, profile)), prepared));
  EXPECT_EQ(prepared.active_profile.get(), profile.get());
}

TEST(PhaseOffsetRuntimeTest, RecenterIsContinuousAndDoesNotResetBeforeCommit) {
  const RuntimePathSamples path = MakePath();
  PhaseOffsetRuntime runtime(MakeConfig(TubeSource::NONE));
  Refresh(runtime, path);
  RuntimePreparedStep prepared;
  RuntimeStepOutput output;
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  ASSERT_TRUE(Complete(runtime, prepared, output));
  const double nonzero = runtime.retainedDelta();
  ASSERT_GT(std::abs(nonzero), 0.0);
  runtime.requestRecenter();
  EXPECT_TRUE(runtime.recenterRequested());
  ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
  EXPECT_DOUBLE_EQ(nonzero, prepared.delta);
  ASSERT_TRUE(Complete(runtime, prepared, output));
  EXPECT_LT(std::abs(runtime.retainedDelta()), std::abs(nonzero));
  for (int index = 0; index < 80 && runtime.hasExecutedOffsetAuthority(); ++index) {
    ASSERT_TRUE(runtime.prepare(MakeInput(path, RuntimeInstalledTubeView()), prepared));
    ASSERT_TRUE(Complete(runtime, prepared, output));
  }
  EXPECT_NEAR(runtime.retainedDelta(), 0.0, 1e-3);
  EXPECT_FALSE(runtime.recenterRequested());
  EXPECT_FALSE(runtime.hasExecutedOffsetAuthority());
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
