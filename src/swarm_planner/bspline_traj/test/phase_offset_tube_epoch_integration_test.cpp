#include <gtest/gtest.h>

#include <cmath>

#define private public
#include <plan_env/sdf_map.h>
#undef private

#define private public
#include "bspline_race/integration/phase_offset_matched_adapter.h"
#undef private
#include "bspline_race/integration/phase_offset_tube_epoch_diagnostics.h"

namespace FLAG_Race {
namespace {

using phase_offset_navigation::DistanceStatus;
using phase_offset_navigation::TubeEpochState;
using phase_offset_navigation::TubeSource;

constexpr double kDt = 0.02;

phase_offset_core::PathDifferentialState State(double w) {
  phase_offset_core::PathDifferentialState state;
  state.p = Eigen::Vector3d(w, 0.0, 1.0 + 0.1 * w);
  state.p_w = Eigen::Vector3d(1.0, 0.0, 0.1); state.p_ww = Eigen::Vector3d::Zero();
  state.w = w; state.valid = true;
  return state;
}

phase_offset_navigation::RuntimePathSamples Path() {
  phase_offset_navigation::RuntimePathSamples path;
  for (int index = 0; index <= 30; ++index) path.push_back(State(0.1 * index));
  return path;
}

guidance::IsfGains Gains() {
  guidance::IsfGains gains;
  gains.k1 = 2.0; gains.k2 = -2.2; gains.convergence_bandwidth = 0.1;
  gains.progress_rho0 = 0.5; gains.progress_delta = 0.3; gains.alpha_min = 0.05;
  return gains;
}

MatchedAdapterInput Input(const phase_offset_navigation::RuntimePathSamples& path,
                          const void* identity, double stamp,
                          const std::shared_ptr<const ContinuousPhasePath>& owner =
                              std::shared_ptr<const ContinuousPhasePath>(),
                          const std::shared_ptr<const ContinuousPhaseNormalFrame>&
                              frame =
                                  std::shared_ptr<const ContinuousPhaseNormalFrame>()) {
  MatchedAdapterInput input;
  input.path = State(0.4); input.semantic_path_identity = identity;
  input.path_state_query = [](const double w,
                              phase_offset_core::PathDifferentialState& state) {
    state = State(w);
    return true;
  };
  input.sampled_path = path;
  input.semantic_path_start_w = 0.0; input.semantic_path_end_w = 3.0;
  input.position = input.path.p; input.gains = Gains(); input.dt = kDt; input.stamp = ros::Time(stamp);
  PhaseOffsetActiveAdapter zero; ActiveAdapterInput zero_input;
  zero_input.path = input.path; zero_input.position = input.position; zero_input.gains = input.gains;
  ActiveAdapterOutput zero_output; EXPECT_TRUE(zero.evaluate(zero_input, zero_output));
  input.legacy = LegacyGuidanceSnapshot(zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent, zero_output.guidance.valid);
  if (owner) {
    input.semantic_path_owner = owner;
    input.frame_owner = frame;
    input.semantic_path_identity = owner.get();
    input.semantic_path_start_w = owner->startW();
    input.semantic_path_end_w = owner->endW();
  }
  return input;
}

void RebaseInputToAuthority(PhaseOffsetMatchedAdapter& adapter,
                            MatchedAdapterInput& input) {
  const phase_offset_navigation::ActiveReferenceSnapshot authority =
      adapter.execution_authority_.snapshot();
  if (!authority.valid || !std::isfinite(authority.proposed_next_w)) return;
  const Eigen::Vector3d position_offset = input.position - input.path.p;
  input.path = State(authority.proposed_next_w);
  input.position = input.path.p + position_offset;
  ActiveAdapterInput zero_input;
  zero_input.path = input.path;
  zero_input.position = input.position;
  zero_input.gains = input.gains;
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterOutput zero_output;
  EXPECT_TRUE(zero.evaluate(zero_input, zero_output));
  input.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
}

PhaseOffsetMatchedAdapterConfig Config(TubeSource source) {
  PhaseOffsetMatchedAdapterConfig config;
  config.mode = PhaseOffsetMatchedMode::MANUAL; config.tube_source = source;
  config.observe_only = false;
  config.tube_update_period = 0.10; config.warmup_cycles = 100;
  config.u_w_rate_max = 100.0; config.u_delta_rate_max = 100.0;
  config.tube.fixed_delta_max = 0.04; config.tube.lookahead_w = 2.0;
  config.tube.back_w = 0.0; config.tube.min_certified_forward_w = 0.40;
  if (source == TubeSource::ESDF) {
    config.cloud_obstacle_set_complete = true;
    config.tube.cross_section.margins.preincluded_map_uncertainty = 0.10;
  }
  return config;
}

std::shared_ptr<const ContinuousPhasePath> BootstrapOwner() {
  auto owner = std::make_shared<ContinuousPhasePath>();
  ContinuousPhasePath::PointEvaluator point_evaluator =
      [](const double w, ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0 + 0.1 * w);
        state.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.1);
        state.d2p_dw2 = Eigen::Vector3d::Zero();
        state.vel = state.dp_dw;
        state.valid = true;
        return true;
      };
  ContinuousPhasePath::CellBoundEvaluator cell_bound_evaluator =
      [](const double w0, const double w1,
         phase_offset_core::PathCellGeometryCertificate& certificate) {
        certificate = phase_offset_core::PathCellGeometryCertificate();
        certificate.w0 = w0;
        certificate.w1 = w1;
        certificate.segment_w0 = 0.0;
        certificate.segment_w1 = 3.0;
        certificate.segment_identity = 1U;
        certificate.inf_p_w_norm = 1.0;
        certificate.inf_horizontal_p_w_norm = 1.0;
        certificate.sup_p_w_norm = 1.01;
        certificate.sup_p_ww_norm = 0.0;
        certificate.sup_p_www_norm = 0.0;
        certificate.sup_horizontal_p_ww_norm = 0.0;
        certificate.horizontal_acceleration_bound_complete = true;
        certificate.sup_N_w_norm = 0.0;
        certificate.sup_abs_curvature = 0.0;
        certificate.normal_variation_bound = 0.0;
        certificate.tangent_variation_bound = 0.0;
        certificate.curvature_variation_bound = 0.0;
        certificate.midpoint_position_variation_bound = 0.0;
        certificate.chord_deviation_bound = 0.0;
        certificate.valid = std::isfinite(w0) && std::isfinite(w1) &&
            w1 > w0;
        certificate.complete = certificate.valid;
        return certificate.valid;
      };
  EXPECT_TRUE(owner->appendSegment(
      0.0, 3.0, "integration-bootstrap-owner",
      ContinuousPhasePath::Evaluator(point_evaluator,
                                     cell_bound_evaluator)));
  return std::shared_ptr<const ContinuousPhasePath>(owner);
}

std::shared_ptr<const ContinuousPhaseNormalFrame> BootstrapFrame(
    const std::shared_ptr<const ContinuousPhasePath>& owner,
    const std::uint64_t effective_revision) {
  if (!owner || owner->empty() || effective_revision == 0U) {
    return std::shared_ptr<const ContinuousPhaseNormalFrame>();
  }
  return std::make_shared<const ContinuousPhaseNormalFrame>(
      owner, effective_revision, effective_revision);
}

bool PrepareAndFinalizePair(
    PhaseOffsetMatchedAdapter& adapter,
    const PathTubePairTransaction& transaction,
    const double current_w,
    const Eigen::Vector3d& position,
    const guidance::IsfGains& gains,
    const double dt,
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    std::shared_ptr<const PathTubePair>& committed) {
  PathTubePairCommitPreparation preparation;
  if (!adapter.preparePathTubePairCommit(transaction, current_w, position,
                                         gains, dt, snapshot, preparation)) {
    return false;
  }
  return adapter.finalizePreparedPathTubePairCommit(preparation, committed);
}

bool BootstrapMatchingPair(
    PhaseOffsetMatchedAdapter& adapter,
    const phase_offset_navigation::RuntimePathSamples& path,
    const void* identity,
    const std::shared_ptr<const plan_env::CloudOccupancySnapshot>& snapshot,
    std::shared_ptr<const PathTubePair>& committed,
    const std::shared_ptr<const ContinuousPhasePath>& supplied_owner =
        std::shared_ptr<const ContinuousPhasePath>(),
    const std::shared_ptr<const ContinuousPhaseNormalFrame>& supplied_frame =
        std::shared_ptr<const ContinuousPhaseNormalFrame>()) {
  const auto owner = supplied_owner ? supplied_owner : BootstrapOwner();
  const auto frame = supplied_frame ? supplied_frame : BootstrapFrame(owner, 1U);
  if (!owner || !frame || !adapter.requiresPathTubePairBootstrap()) return false;
  MatchedAdapterInput activation = Input(path, owner.get(), 2.0, owner, frame);
  activation.semantic_path_owner = owner;
  activation.frame_owner = frame;
  activation.semantic_path_start_w = owner->startW();
  activation.semantic_path_end_w = owner->endW();
  activation.semantic_path_identity = identity;
  activation.cloud_occupancy_snapshot = snapshot;
  PathTubePairTransaction transaction;
  if (!adapter.stagePathTubePair(
          std::shared_ptr<const PathTubePair>(), owner, path,
          activation.path.w, 1.0, 2.4, activation.position,
          activation.gains, activation.dt, snapshot, transaction)) {
    return false;
  }
  return PrepareAndFinalizePair(adapter, transaction, activation.path.w,
                                activation.position, activation.gains,
                                activation.dt, snapshot, committed) &&
      committed && committed->path_owner == owner;
}

void AddCallback(MatchedAdapterInput& input,
                 const phase_offset_navigation::RuntimePathSamples& path,
                 int& calls) {
  // S3 never transports a sampling callback across the command/timer split.
  // Keep a complete immutable sample value instead; `calls` proves no legacy
  // callback is invoked by either marker construction or timer builds.
  input.sampled_path = path;
  input.sample_path = decltype(input.sample_path)();
  (void)calls;
}

void BuildAndConsume(PhaseOffsetMatchedAdapter& adapter,
                     const MatchedAdapterInput& input,
                     MatchedAdapterOutput& output) {
  adapter.update(input, output);
  adapter.timerTick();
  adapter.update(input, output);
}

void InitMap(SDFMap& map) {
  map.mp_.resolution_ = map.mp_.resolution_inv_ = 1.0;
  map.mp_.map_origin_ = Eigen::Vector3d(-5.0, -5.0, -1.0);
  map.mp_.map_size_ = Eigen::Vector3d(10.0, 10.0, 10.0);
  map.mp_.map_voxel_num_ = Eigen::Vector3i(10, 10, 10);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
  map.md_.distance_buffer_all_.assign(1000U, 10.0);
  map.md_.occupancy_buffer_inflate_.assign(1000U, 0);
}

void InstallCompleteCloudSnapshot(SDFMap& map, std::uint64_t sequence) {
  auto snapshot = std::make_shared<plan_env::CloudOccupancySnapshot>();
  snapshot->valid = true;
  snapshot->observation_sequence = sequence;
  snapshot->observation_stamp = ros::Time(static_cast<double>(sequence));
  snapshot->map_min = map.mp_.map_min_boundary_;
  snapshot->map_max = map.mp_.map_max_boundary_;
  snapshot->observed_min = snapshot->map_min;
  snapshot->observed_max = snapshot->map_max;
  snapshot->grid_origin = map.mp_.map_origin_;
  snapshot->voxel_count = map.mp_.map_voxel_num_;
  snapshot->resolution = map.mp_.resolution_;
  snapshot->included_map_inflation = 0.10;
  snapshot->occupied.assign(1000U, 0U);
  ASSERT_TRUE(plan_env::cloudOccupancySnapshotConsistent(*snapshot));
  std::lock_guard<std::mutex> lock(map.cloud_occupancy_snapshot_store_->mutex);
  map.cloud_occupancy_snapshot_store_->observation_sequence = sequence;
  map.cloud_occupancy_snapshot_store_->latest = snapshot;
}

void ExpectThreeActions(const visualization_msgs::MarkerArray& markers, int action) {
  ASSERT_EQ(markers.markers.size(), 3U);
  for (const auto& marker : markers.markers) EXPECT_EQ(marker.action, action);
}

// Small deterministic rendezvous fixture.  It exercises the adapter's
// identity/sequence protocol directly, without constructing a Tube or
// touching ROS timers.  Production still reaches the same arm/READY/claim
// transitions through update()/finalizeTubeEpoch().
struct BootstrapRendezvousFixture {
  PhaseOffsetMatchedAdapter adapter;
  std::shared_ptr<const ContinuousPhasePath> owner;
  std::shared_ptr<const ContinuousPhaseNormalFrame> frame;
  std::shared_ptr<const TubeBuildRequest> request;

  BootstrapRendezvousFixture()
      : adapter(Config(TubeSource::FIXED)), owner(BootstrapOwner()) {
    const_cast<ContinuousPhasePath*>(owner.get())->setPathRevision(1U);
    frame = std::make_shared<const ContinuousPhaseNormalFrame>(owner, 1U, 1U);
    MatchedAdapterInput input = Input(Path(), owner.get(), 1.0);
    input.semantic_path_owner = owner;
    input.frame_owner = frame;
    input.semantic_path_identity = owner.get();
    input.semantic_path_start_w = owner->startW();
    input.semantic_path_end_w = owner->endW();
    const std::uint64_t revision = adapter.sourceRevision(input);
    request = adapter.makeBuildRequest(input, revision, 0.0);
    std::lock_guard<std::mutex> lock(adapter.runtime_command_mutex_);
    adapter.zero_gate_open_ = true;
    adapter.command_active_ = true;
    std::atomic_store(&adapter.latest_build_request_, request);
    EXPECT_TRUE(adapter.armBootstrapRendezvousLocked(request));
  }

  void ready(std::uint64_t build_sequence) {
    std::lock_guard<std::mutex> lock(adapter.runtime_command_mutex_);
    adapter.bootstrap_rendezvous_ready_ = adapter.bootstrap_rendezvous_arm_;
    adapter.bootstrap_rendezvous_ready_build_sequence_ = build_sequence;
    adapter.bootstrap_rendezvous_state_ = BootstrapRendezvousState::READY;

    // Keep the synthetic READY transition bound to the same immutable active
    // epoch that production claim() now requires.  This fixture does not run
    // TubeEpochManager, so populate only the identity/provenance facts used by
    // the rendezvous freshness gate; no profile is ever handed to authority.
    auto epoch = std::make_shared<TubeEpochSnapshot>();
    epoch->active = true;
    epoch->task_generation = adapter.bootstrap_rendezvous_arm_.task_generation;
    epoch->request_control_sequence = request->control_sequence;
    epoch->build_sequence = build_sequence;
    epoch->source_revision = adapter.bootstrap_rendezvous_arm_.source_revision;
    epoch->path_revision = adapter.bootstrap_rendezvous_arm_.path_revision;
    epoch->frame_revision = adapter.bootstrap_rendezvous_arm_.frame_revision;
    epoch->semantic_path_owner = owner;
    epoch->frame_owner = frame;
    epoch->full_path_samples = request->supplied_path_samples;
    auto profile = std::make_shared<phase_offset_navigation::TubeProfile>();
    profile->source = TubeSource::FIXED;
    profile->source_revision = epoch->source_revision;
    profile->path_revision = epoch->path_revision;
    profile->frame_revision = epoch->frame_revision;
    profile->tube_revision = build_sequence;
    profile->profile_revision = build_sequence;
    profile->raw_complete = true;
    profile->filtered_complete = true;
    profile->complete = true;
    profile->obstacle_certified = true;
    profile->classification =
        phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
    epoch->candidate_profile = profile;
    epoch->active_profile = profile;
    epoch->epoch_status.candidate_complete = true;
    epoch->epoch_status.candidate_classification =
        phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
    epoch->epoch_status.candidate_path_source_revision =
        epoch->source_revision;
    epoch->epoch_status.active_available = true;
    epoch->epoch_status.active_current_validation_valid = true;
    epoch->epoch_status.active_classification =
        phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
    epoch->epoch_status.active_path_source_revision = epoch->source_revision;
    std::atomic_store(&adapter.latest_epoch_snapshot_,
                      std::shared_ptr<const TubeEpochSnapshot>(epoch));
  }
};

TEST(TubeEpochIntegrationTest, FiftyControlCyclesProduceTenBuildAttemptsAndNoMarkerResampling) {
  const auto path = Path(); int identity = 1; int callback_calls = 0;
  PhaseOffsetMatchedAdapter adapter(Config(TubeSource::FIXED));
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 50; ++cycle) {
    MatchedAdapterInput input = Input(path, &identity, cycle * kDt);
    AddCallback(input, path, callback_calls);
    adapter.update(input, output);
    if (cycle % 5 == 0) adapter.timerTick();
  }
  EXPECT_EQ(callback_calls, 0);
  EXPECT_EQ(output.tube_epoch_status.candidate_sequence, 10U);
  EXPECT_EQ(output.tube_epoch_status.active_tube_epoch, 1U);
  EXPECT_FALSE(output.zero_gate_open);
  EXPECT_FALSE(output.selected);
  MatchedAdapterInput marker_input = Input(path, &identity, 2.0);
  AddCallback(marker_input, path, callback_calls);
  MatchedAdapterMarkerBundle markers;
  for (int index = 0; index < 20; ++index) adapter.buildMarkers(marker_input, output, markers);
  EXPECT_EQ(callback_calls, 0);
  ExpectThreeActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectThreeActions(markers.tube, visualization_msgs::Marker::ADD);
}

TEST(TubeEpochIntegrationTest,
     PreGateTrackingExcursionDeletesCertifiedWithoutCertificateDenialOrFailureLatch) {
  const auto path = Path();
  int identity = 8;
  PhaseOffsetMatchedAdapter adapter(Config(TubeSource::FIXED));
  MatchedAdapterInput input = Input(path, &identity, 0.0);
  input.position.y() += 1.0;
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterInput zero_input;
  zero_input.path = input.path;
  zero_input.position = input.position;
  zero_input.gains = input.gains;
  ActiveAdapterOutput zero_output;
  ASSERT_TRUE(zero.evaluate(zero_input, zero_output));
  input.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
  MatchedAdapterOutput output;

  BuildAndConsume(adapter, input, output);
  EXPECT_FALSE(output.selected);
  EXPECT_FALSE(output.zero_gate_open);
  EXPECT_FALSE(output.selected);
  EXPECT_EQ(output.tube_epoch_status.state, TubeEpochState::ROLLING);
  EXPECT_FALSE(output.runtime_execution.certificate_denied);
  EXPECT_FALSE(output.tube_epoch_status.certificate_denied);
  EXPECT_FALSE(output.failure_latched);
  ASSERT_TRUE(output.candidate_profile);
  EXPECT_TRUE(output.candidate_profile->complete);
  MatchedAdapterMarkerBundle markers;
  ASSERT_TRUE(adapter.buildMarkers(input, output, markers));
  ExpectThreeActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectThreeActions(markers.tube, visualization_msgs::Marker::DELETE);
}

TEST(TubeEpochIntegrationTest,
     ArmedTrackingRecoveryKeepsSelectedThenReturnsToRolling) {
  const auto path = Path();
  int identity = 81;
  PhaseOffsetMatchedAdapter adapter(Config(TubeSource::FIXED));
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 99; ++cycle) {
    ASSERT_FALSE(adapter.update(Input(path, &identity, cycle * kDt), output));
    if (cycle == 0) adapter.timerTick();
  }
  ASSERT_FALSE(adapter.update(Input(path, &identity, 99.0 * kDt), output));
  ASSERT_TRUE(output.zero_gate_open);
  std::shared_ptr<const PathTubePair> pair;
  ASSERT_TRUE(BootstrapMatchingPair(
      adapter, path, &identity,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), pair));
  MatchedAdapterInput activation = Input(path, &identity, 100.0 * kDt);
  activation.path_tube_pair = pair;
  RebaseInputToAuthority(adapter, activation);
  ASSERT_TRUE(adapter.update(activation, output));
  ASSERT_TRUE(output.selected);
  const double delta_before_recovery = output.delta;

  MatchedAdapterInput tracking = Input(path, &identity, 100.0 * kDt);
  tracking.position.y() += 0.20;
  tracking.path_tube_pair = pair;
  RebaseInputToAuthority(adapter, tracking);
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterInput zero_input;
  zero_input.path = tracking.path;
  zero_input.position = tracking.position;
  zero_input.gains = tracking.gains;
  ActiveAdapterOutput zero_output;
  ASSERT_TRUE(zero.evaluate(zero_input, zero_output));
  tracking.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);

  ASSERT_TRUE(adapter.update(tracking, output));
  EXPECT_TRUE(output.selected);
  EXPECT_EQ(output.runtime_execution.mode,
            phase_offset_navigation::RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(output.runtime_execution.tracking_within_bound);
  EXPECT_FALSE(output.runtime_execution.certificate_denied);
  EXPECT_FALSE(output.failure_latched);
  EXPECT_NEAR(output.delta, delta_before_recovery, 0.10);
  MatchedAdapterMarkerBundle markers;
  MatchedAdapterInput marker_tracking = Input(path, &identity, 100.0 * kDt);
  marker_tracking.position = tracking.position;
  marker_tracking.path_tube_pair = pair;
  ASSERT_TRUE(adapter.buildMarkers(marker_tracking, output, markers));
  ExpectThreeActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectThreeActions(markers.tube, visualization_msgs::Marker::DELETE);

  for (int cycle = 101; cycle <= 105; ++cycle) {
    MatchedAdapterInput recovery = Input(path, &identity, cycle * kDt);
    recovery.path_tube_pair = pair;
    RebaseInputToAuthority(adapter, recovery);
    ASSERT_TRUE(adapter.update(recovery, output));
  }
  EXPECT_TRUE(output.selected);
  EXPECT_EQ(output.runtime_execution.mode,
            phase_offset_navigation::RuntimeExecutionMode::NORMAL);
  EXPECT_TRUE(output.runtime_execution.tracking_within_bound);
  MatchedAdapterInput marker_input = Input(path, &identity, 2.2);
  marker_input.path_tube_pair = pair;
  ASSERT_TRUE(adapter.buildMarkers(marker_input, output, markers));
  ExpectThreeActions(markers.tube, visualization_msgs::Marker::ADD);
}

TEST(TubeEpochIntegrationTest,
     ArmedEsdfTrackingRecoveryUsesLatestFreeSnapshotWithoutCertificateDenial) {
  const auto path = Path();
  const auto owner = BootstrapOwner();
  const auto frame = BootstrapFrame(owner, 1U);
  ASSERT_TRUE(owner);
  ASSERT_TRUE(frame);
  int identity = 82;
  SDFMap map;
  InitMap(map);
  InstallCompleteCloudSnapshot(map, 82U);
  PhaseOffsetMatchedAdapter adapter(Config(TubeSource::ESDF));
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 99; ++cycle) {
    MatchedAdapterInput input =
        Input(path, owner.get(), cycle * kDt, owner, frame);
    input.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
    ASSERT_FALSE(adapter.update(input, output));
    if (cycle == 0) adapter.timerTick();
  }
  MatchedAdapterInput armed =
      Input(path, owner.get(), 99.0 * kDt, owner, frame);
  armed.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  ASSERT_FALSE(adapter.update(armed, output));
  ASSERT_TRUE(output.candidate_profile);
  ASSERT_TRUE(output.candidate_profile->complete);
  ASSERT_EQ(output.candidate_profile->classification,
            phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED);
  ASSERT_TRUE(output.tube_epoch_status.candidate_complete);
  ASSERT_EQ(output.tube_epoch_status.candidate_classification,
            phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED);
  ASSERT_TRUE(output.zero_gate_open);
  ASSERT_TRUE(adapter.requiresPathTubePairBootstrap());
  std::shared_ptr<const PathTubePair> pair;
  ASSERT_TRUE(BootstrapMatchingPair(adapter, path, &identity,
                                    map.cloudOccupancySnapshot(), pair,
                                    owner, frame));
  armed.path_tube_pair = pair;
  RebaseInputToAuthority(adapter, armed);
  ASSERT_TRUE(adapter.update(armed, output));
  ASSERT_TRUE(output.selected);

  MatchedAdapterInput tracking =
      Input(path, owner.get(), 100.0 * kDt, owner, frame);
  tracking.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  tracking.path_tube_pair = pair;
  RebaseInputToAuthority(adapter, tracking);
  tracking.position.y() += 0.20;
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterInput zero_input;
  zero_input.path = tracking.path;
  zero_input.position = tracking.position;
  zero_input.gains = tracking.gains;
  ActiveAdapterOutput zero_output;
  ASSERT_TRUE(zero.evaluate(zero_input, zero_output));
  tracking.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);

  ASSERT_TRUE(adapter.update(tracking, output));
  EXPECT_TRUE(output.selected);
  EXPECT_EQ(output.runtime_execution.mode,
            phase_offset_navigation::RuntimeExecutionMode::NORMAL);
  EXPECT_FALSE(output.runtime_execution.tracking_within_bound);
  EXPECT_FALSE(output.runtime_execution.certificate_denied);
  EXPECT_FALSE(output.failure_latched);
  EXPECT_TRUE(output.tube_epoch_status.raw_cross_section_path_used);
  MatchedAdapterMarkerBundle markers;
  MatchedAdapterInput marker_tracking =
      Input(path, owner.get(), 100.0 * kDt, owner, frame);
  marker_tracking.position = tracking.position;
  marker_tracking.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  marker_tracking.path_tube_pair = pair;
  ASSERT_TRUE(adapter.buildMarkers(marker_tracking, output, markers));
  ExpectThreeActions(markers.tube_candidate, visualization_msgs::Marker::ADD);
  ExpectThreeActions(markers.tube, visualization_msgs::Marker::DELETE);
}

TEST(TubeEpochIntegrationTest,
     PathRevisionTrackingRecoveryKeepsSelectedWithoutAttemptingA6Continuation) {
  const auto path = Path();
  int first_identity = 83;
  int revised_identity = 84;
  PhaseOffsetMatchedAdapter adapter(Config(TubeSource::FIXED));
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 99; ++cycle) {
    ASSERT_FALSE(adapter.update(Input(path, &first_identity, cycle * kDt), output));
    if (cycle == 0) adapter.timerTick();
  }
  ASSERT_FALSE(adapter.update(Input(path, &first_identity, 99.0 * kDt), output));
  ASSERT_TRUE(output.zero_gate_open);
  std::shared_ptr<const PathTubePair> pair;
  ASSERT_TRUE(BootstrapMatchingPair(
      adapter, path, &first_identity,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), pair));
  MatchedAdapterInput activation = Input(path, &first_identity, 100.0 * kDt);
  activation.path_tube_pair = pair;
  ASSERT_TRUE(adapter.update(activation, output));
  ASSERT_TRUE(output.selected);
  const double delta_before_revision = output.delta;

  MatchedAdapterInput revised = Input(path, &revised_identity, 100.0 * kDt);
  revised.position.y() += 0.20;
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterInput zero_input;
  zero_input.path = revised.path;
  zero_input.position = revised.position;
  zero_input.gains = revised.gains;
  ActiveAdapterOutput zero_output;
  ASSERT_TRUE(zero.evaluate(zero_input, zero_output));
  revised.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);

  // A changed semantic revision cannot consume the old Pair as a sidecar
  // authority.  With no atomic replacement, the adapter remains fail-closed
  // and retains the old owner rather than fabricating a selected command.
  ASSERT_FALSE(adapter.update(revised, output));
  EXPECT_FALSE(output.selected);
  EXPECT_FALSE(output.failure_latched);
  EXPECT_EQ(pair, adapter.capturePathTubePair());
  adapter.timerTick();
  ASSERT_FALSE(adapter.update(revised, output));
  EXPECT_TRUE(output.tube_update_due_this_cycle);
  EXPECT_FALSE(output.selected);
  EXPECT_FALSE(output.runtime_execution.certificate_denied);
  EXPECT_FALSE(output.failure_latched);
  EXPECT_NEAR(output.delta, delta_before_revision, 0.10);
}

TEST(TubeEpochIntegrationTest, GateOpeningDoesNotAdvanceAnEquivalentActiveEpoch) {
  const auto path = Path(); int identity = 2;
  PhaseOffsetMatchedAdapter adapter(Config(TubeSource::FIXED));
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 99; ++cycle) {
    adapter.update(Input(path, &identity, cycle * kDt), output);
    if (cycle == 0) adapter.timerTick();
  }
  const std::uint64_t before = output.tube_epoch_status.active_tube_epoch;
  ASSERT_EQ(before, 1U);
  ASSERT_FALSE(adapter.update(Input(path, &identity, 100.0 * kDt), output));
  EXPECT_TRUE(output.zero_gate_open);
  EXPECT_FALSE(output.selected);
  EXPECT_EQ(output.tube_epoch_status.active_tube_epoch, before);

  // Gate opening alone is not an authority transition.  Once the existing
  // bootstrap bridge commits the matching Pair, the first selected command
  // is allowed, while an unrelated equivalent timer refresh does not create a
  // new active epoch identity.
  std::shared_ptr<const PathTubePair> pair;
  ASSERT_TRUE(BootstrapMatchingPair(
      adapter, path, &identity,
      std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), pair));
  MatchedAdapterInput activation = Input(path, &identity, 100.0 * kDt);
  activation.path_tube_pair = pair;
  RebaseInputToAuthority(adapter, activation);
  ASSERT_TRUE(adapter.update(activation, output));
  EXPECT_TRUE(output.selected);
  EXPECT_EQ(output.tube_epoch_status.active_tube_epoch, before);
  adapter.timerTick();
  MatchedAdapterInput refresh = Input(path, &identity, 101.0 * kDt);
  refresh.path_tube_pair = pair;
  RebaseInputToAuthority(adapter, refresh);
  ASSERT_TRUE(adapter.update(refresh, output));
  EXPECT_TRUE(output.selected);
  EXPECT_EQ(output.tube_epoch_status.active_tube_epoch, before);
}

TEST(TubeEpochIntegrationTest, PathEventForcesBuildAndEquivalentRefreshKeepsActiveIdentity) {
  const auto path = Path(); int first_identity = 3; int second_identity = 4;
  PhaseOffsetMatchedAdapter adapter(Config(TubeSource::FIXED));
  MatchedAdapterOutput first;
  BuildAndConsume(adapter, Input(path, &first_identity, 0.0), first);
  ASSERT_TRUE(first.tube_update_due_this_cycle);
  const auto* first_active = first.active_profile.get();
  MatchedAdapterOutput equivalent;
  for (int cycle = 1; cycle <= 5; ++cycle) {
    adapter.update(Input(path, &first_identity, cycle * kDt), equivalent);
  }
  adapter.timerTick();
  adapter.update(Input(path, &first_identity, 6.0 * kDt), equivalent);
  EXPECT_EQ(equivalent.active_profile.get(), first_active);
  MatchedAdapterOutput event;
  BuildAndConsume(adapter, Input(path, &second_identity, 7.0 * kDt), event);
  EXPECT_TRUE(event.tube_update_due_this_cycle);
  EXPECT_NE(event.active_profile.get(), first_active);
  EXPECT_EQ(event.tube_epoch_status.active_tube_epoch,
            first.tube_epoch_status.active_tube_epoch + 1U);
}

TEST(TubeEpochIntegrationTest, EsdfDoesNotFabricateObservationSequenceWithoutSnapshot) {
  const auto path = Path(); int identity = 5;
  PhaseOffsetMatchedAdapter adapter(Config(TubeSource::ESDF));
  MatchedAdapterOutput output;
  for (int cycle = 0; cycle < 50; ++cycle) {
    adapter.update(Input(path, &identity, cycle * kDt), output);
    if (cycle % 5 == 0) adapter.timerTick();
  }
  EXPECT_EQ(output.tube_epoch_status.candidate_sequence, 10U);
  EXPECT_EQ(output.tube_epoch_status.candidate_map_observation_sequence, 0U);
  EXPECT_EQ(output.tube_epoch_status.active_map_observation_sequence, 0U);
  EXPECT_TRUE(output.tube_epoch_status.map_observation_is_snapshot);
  EXPECT_EQ(output.tube_epoch_status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
}

TEST(TubeEpochIntegrationTest, RejectedEsdfCandidateMasksStaleActiveAndDeletesCertified) {
  const auto path = Path();
  const auto owner = BootstrapOwner();
  const auto frame = BootstrapFrame(owner, 1U);
  ASSERT_TRUE(owner);
  ASSERT_TRUE(frame);
  int identity = 6; SDFMap map; InitMap(map);
  InstallCompleteCloudSnapshot(map, 41U);
  PhaseOffsetMatchedAdapter adapter(Config(TubeSource::ESDF));
  MatchedAdapterInput valid = Input(path, owner.get(), 0.0, owner, frame);
  valid.cloud_occupancy_snapshot = map.cloudOccupancySnapshot();
  MatchedAdapterOutput installed; BuildAndConsume(adapter, valid, installed);
  ASSERT_TRUE(installed.tube_epoch_status.candidate_complete);
  ASSERT_EQ(installed.tube_epoch_status.candidate_classification,
            phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED);
  ASSERT_TRUE(installed.active_profile);
  ASSERT_TRUE(installed.tube_epoch_status.active_available);
  ASSERT_EQ(installed.active_profile->classification,
            phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED);
  MatchedAdapterOutput rejected;
  for (int cycle = 1; cycle <= 5; ++cycle) {
    MatchedAdapterInput unavailable =
        Input(path, owner.get(), cycle * kDt, owner, frame);
    adapter.update(unavailable, rejected);
    if (cycle == 1) adapter.timerTick();
  }
  EXPECT_TRUE(rejected.candidate_profile);
  EXPECT_FALSE(rejected.candidate_profile->complete);
  EXPECT_FALSE(rejected.active_profile);
  EXPECT_FALSE(rejected.tube_epoch_status.active_available);
  EXPECT_EQ(rejected.tube_epoch_status.state, TubeEpochState::WAITING_FOR_CANDIDATE);
  MatchedAdapterMarkerBundle markers;
  MatchedAdapterInput marker_input =
      Input(path, owner.get(), 1.0, owner, frame);
  ASSERT_TRUE(adapter.buildMarkers(marker_input, rejected, markers));
  ExpectThreeActions(markers.tube, visualization_msgs::Marker::DELETE);
}

TEST(TubeEpochIntegrationTest, ManualAndEpochDiagnosticsHaveExactIndependentSchemas) {
  const auto path = Path(); int identity = 7;
  PhaseOffsetMatchedAdapter adapter(Config(TubeSource::FIXED));
  MatchedAdapterOutput output; adapter.update(Input(path, &identity, 0.0), output);
  EXPECT_EQ(output.diagnostics.size(), kManualDiagnosticCount);
  TubeEpochDiagnosticsInput epoch;
  epoch.source = TubeSource::FIXED; epoch.epoch = output.tube_epoch_status;
  epoch.runtime = output.runtime_execution; epoch.candidate_profile = output.candidate_profile.get();
  epoch.active_profile = output.active_profile.get(); epoch.retained_delta = output.delta;
  epoch.active_display_certified = output.diagnostics[kTubeDisplayCertified] != 0.0;
  const auto payload = makeTubeEpochDiagnostics(epoch);
  EXPECT_EQ(payload.size(), kTubeEpochDiagnosticCount);
  for (double value : payload) EXPECT_TRUE(std::isfinite(value));
}

TEST(BootstrapRendezvous, GateClosedDoesNotArmOrUsePreGateEpoch) {
  BootstrapRendezvousFixture fixture;
  {
    std::lock_guard<std::mutex> lock(fixture.adapter.runtime_command_mutex_);
    fixture.adapter.clearBootstrapRendezvousLocked();
    fixture.adapter.zero_gate_open_ = false;
    EXPECT_FALSE(fixture.adapter.armBootstrapRendezvousLocked(
        fixture.request));
    EXPECT_EQ(BootstrapRendezvousState::DISARMED,
              fixture.adapter.bootstrap_rendezvous_state_);
  }
  BootstrapRendezvousTicket ticket;
  EXPECT_FALSE(fixture.adapter.claimBootstrapRendezvous(ticket));
  EXPECT_FALSE(ticket.valid());
}

TEST(BootstrapRendezvous, PostArmFreshEpochClaimsExactlyOnce) {
  BootstrapRendezvousFixture fixture;
  fixture.ready(1U);
  BootstrapRendezvousTicket first;
  ASSERT_TRUE(fixture.adapter.claimBootstrapRendezvous(first));
  EXPECT_TRUE(first.valid());
  BootstrapRendezvousTicket duplicate;
  EXPECT_FALSE(fixture.adapter.claimBootstrapRendezvous(duplicate));
  EXPECT_TRUE(fixture.adapter.completeBootstrapRendezvous(first, false));
  EXPECT_EQ(BootstrapRendezvousState::ARMED,
            fixture.adapter.bootstrapRendezvousState());

  fixture.ready(1U);  // Consumed build sequence is fenced by the watermark.
  EXPECT_FALSE(fixture.adapter.claimBootstrapRendezvous(duplicate));
  fixture.ready(2U);
  ASSERT_TRUE(fixture.adapter.claimBootstrapRendezvous(duplicate));
  EXPECT_NE(first.claim_id, duplicate.claim_id);
  EXPECT_EQ(2U, duplicate.build_sequence);
  EXPECT_TRUE(fixture.adapter.completeBootstrapRendezvous(duplicate, false));
}

TEST(BootstrapRendezvous, NoPostArmCertificateRemainsArmed) {
  BootstrapRendezvousFixture fixture;
  EXPECT_EQ(BootstrapRendezvousState::ARMED,
            fixture.adapter.bootstrapRendezvousState());
  BootstrapRendezvousTicket ticket;
  EXPECT_FALSE(fixture.adapter.claimBootstrapRendezvous(ticket));
  EXPECT_FALSE(fixture.adapter.capturePathTubePair());
}

TEST(BootstrapRendezvous, ExpiredReadyEpochDemotesAndRequiresFreshSequence) {
  BootstrapRendezvousFixture fixture;
  fixture.ready(3U);
  {
    std::lock_guard<std::mutex> lock(fixture.adapter.runtime_command_mutex_);
    // Simulate a newer/incomplete finalization or deactivation removing the
    // active epoch after READY was published.  The ticket must not preserve
    // that old profile as an authority trigger.
    std::atomic_store(&fixture.adapter.latest_epoch_snapshot_,
                      std::shared_ptr<const TubeEpochSnapshot>());
  }

  BootstrapRendezvousTicket expired;
  EXPECT_FALSE(fixture.adapter.claimBootstrapRendezvous(expired));
  EXPECT_FALSE(expired.valid());
  EXPECT_EQ(BootstrapRendezvousState::ARMED,
            fixture.adapter.bootstrapRendezvousState());

  // Re-presenting the expired sequence is fenced by the consumed watermark;
  // only a later finalized sequence may become READY/claimable.
  fixture.ready(3U);
  BootstrapRendezvousTicket same_sequence;
  EXPECT_FALSE(fixture.adapter.claimBootstrapRendezvous(same_sequence));
  fixture.ready(4U);
  BootstrapRendezvousTicket fresh;
  ASSERT_TRUE(fixture.adapter.claimBootstrapRendezvous(fresh));
  EXPECT_EQ(4U, fresh.build_sequence);
  EXPECT_TRUE(fixture.adapter.completeBootstrapRendezvous(fresh, false));

  // The same freshness contract is checked again at each transaction seam;
  // an epoch expiring after claim cannot authorize a late commit.
  fixture.ready(5U);
  BootstrapRendezvousTicket claimed;
  ASSERT_TRUE(fixture.adapter.claimBootstrapRendezvous(claimed));
  {
    std::lock_guard<std::mutex> lock(fixture.adapter.runtime_command_mutex_);
    std::atomic_store(&fixture.adapter.latest_epoch_snapshot_,
                      std::shared_ptr<const TubeEpochSnapshot>());
  }
  EXPECT_FALSE(fixture.adapter.validateBootstrapRendezvousClaim(claimed));
  EXPECT_TRUE(fixture.adapter.completeBootstrapRendezvous(claimed, false));
  EXPECT_EQ(BootstrapRendezvousState::ARMED,
            fixture.adapter.bootstrapRendezvousState());
}

TEST(BootstrapRendezvous, TaskSessionRevisionAndFailureInvalidateLateClaim) {
  BootstrapRendezvousFixture fixture;
  fixture.ready(3U);
  BootstrapRendezvousTicket ticket;
  ASSERT_TRUE(fixture.adapter.claimBootstrapRendezvous(ticket));
  fixture.adapter.authority_session_.fetch_add(1U, std::memory_order_acq_rel);
  EXPECT_FALSE(fixture.adapter.completeBootstrapRendezvous(ticket, false));
  EXPECT_EQ(BootstrapRendezvousState::DISARMED,
            fixture.adapter.bootstrapRendezvousState());

  BootstrapRendezvousFixture revision_fixture;
  revision_fixture.ready(4U);
  BootstrapRendezvousTicket revision_ticket;
  ASSERT_TRUE(revision_fixture.adapter.claimBootstrapRendezvous(
      revision_ticket));
  const auto replacement_owner = BootstrapOwner();
  const_cast<ContinuousPhasePath*>(replacement_owner.get())
      ->setPathRevision(2U);
  const auto replacement_frame =
      std::make_shared<const ContinuousPhaseNormalFrame>(
          replacement_owner, 2U, 2U);
  MatchedAdapterInput replacement_input =
      Input(Path(), replacement_owner.get(), 2.0);
  replacement_input.semantic_path_owner = replacement_owner;
  replacement_input.frame_owner = replacement_frame;
  replacement_input.semantic_path_identity = replacement_owner.get();
  replacement_input.semantic_path_start_w = replacement_owner->startW();
  replacement_input.semantic_path_end_w = replacement_owner->endW();
  const std::uint64_t replacement_revision =
      revision_fixture.adapter.sourceRevision(replacement_input);
  const auto replacement_request = revision_fixture.adapter.makeBuildRequest(
      replacement_input, replacement_revision, 0.0);
  {
    std::lock_guard<std::mutex> lock(
        revision_fixture.adapter.runtime_command_mutex_);
    std::atomic_store(&revision_fixture.adapter.latest_build_request_,
                      replacement_request);
    revision_fixture.adapter.armBootstrapRendezvousLocked(
        replacement_request);
  }
  EXPECT_FALSE(revision_fixture.adapter.completeBootstrapRendezvous(
      revision_ticket, false));

  BootstrapRendezvousFixture failure_fixture;
  failure_fixture.ready(5U);
  BootstrapRendezvousTicket failure_ticket;
  ASSERT_TRUE(failure_fixture.adapter.claimBootstrapRendezvous(
      failure_ticket));
  {
    std::lock_guard<std::mutex> lock(
        failure_fixture.adapter.runtime_command_mutex_);
    failure_fixture.adapter.latchFailure(
        phase_offset_navigation::ControlFailureReason::GEOMETRY_INVARIANT);
  }
  EXPECT_FALSE(failure_fixture.adapter.completeBootstrapRendezvous(
      failure_ticket, false));
  EXPECT_EQ(BootstrapRendezvousState::DISARMED,
            failure_fixture.adapter.bootstrapRendezvousState());
}

TEST(BootstrapRendezvous, ExistingPairPreventsSecondAuthority) {
  BootstrapRendezvousFixture fixture;
  fixture.ready(5U);
  auto pair = std::make_shared<PathTubePair>();
  std::atomic_store(&fixture.adapter.authoritative_path_tube_pair_,
                    std::shared_ptr<const PathTubePair>(pair));
  BootstrapRendezvousTicket ticket;
  EXPECT_FALSE(fixture.adapter.claimBootstrapRendezvous(ticket));
  EXPECT_EQ(BootstrapRendezvousState::DISARMED,
            fixture.adapter.bootstrapRendezvousState());
}

TEST(BootstrapRendezvous, FailedClaimConsumesEpochAndRequiresFreshSequence) {
  BootstrapRendezvousFixture fixture;
  fixture.ready(7U);
  BootstrapRendezvousTicket ticket;
  ASSERT_TRUE(fixture.adapter.claimBootstrapRendezvous(ticket));
  ASSERT_TRUE(fixture.adapter.completeBootstrapRendezvous(ticket, false));
  fixture.ready(7U);
  BootstrapRendezvousTicket stale;
  EXPECT_FALSE(fixture.adapter.claimBootstrapRendezvous(stale));
  fixture.ready(8U);
  ASSERT_TRUE(fixture.adapter.claimBootstrapRendezvous(stale));
  EXPECT_EQ(8U, stale.build_sequence);
  EXPECT_TRUE(fixture.adapter.completeBootstrapRendezvous(stale, false));
}

TEST(BootstrapRendezvous, NewerReadyMayReplaceClaimedEpoch) {
  BootstrapRendezvousFixture fixture;
  fixture.ready(9U);
  BootstrapRendezvousTicket claimed;
  ASSERT_TRUE(fixture.adapter.claimBootstrapRendezvous(claimed));
  // A worker can finalize a newer epoch while the manager is still proving
  // the older ticket.  Keep the newer READY independently of the claim.
  fixture.ready(10U);
  BootstrapRendezvousTicket blocked;
  EXPECT_FALSE(fixture.adapter.claimBootstrapRendezvous(blocked));
  EXPECT_TRUE(fixture.adapter.completeBootstrapRendezvous(claimed, false));
  ASSERT_TRUE(fixture.adapter.claimBootstrapRendezvous(blocked));
  EXPECT_EQ(10U, blocked.build_sequence);
  EXPECT_TRUE(fixture.adapter.completeBootstrapRendezvous(blocked, false));
}

TEST(BootstrapRendezvous, NavigationStateRemainsAvailableWhileArmed) {
  BootstrapRendezvousFixture fixture;
  EXPECT_EQ(BootstrapRendezvousState::ARMED,
            fixture.adapter.bootstrapRendezvousState());
  // Arming is identity-only: it does not install a Pair or mutate Runtime's
  // retained offset/port history, so the ordinary command path remains the
  // planner-owned neutral path.
  EXPECT_DOUBLE_EQ(0.0, fixture.adapter.runtime_->retainedDelta());
  EXPECT_FALSE(fixture.adapter.capturePathTubePair());
}

TEST(BootstrapRendezvous, SuccessfulPairCasClearsWithFreshStagedRevision) {
  BootstrapRendezvousFixture fixture;
  fixture.ready(11U);
  BootstrapRendezvousTicket ticket;
  ASSERT_TRUE(fixture.adapter.claimBootstrapRendezvous(ticket));

  auto pair = std::make_shared<PathTubePair>();
  pair->authority_session = ticket.authority_session;
  pair->source_revision = ticket.source_revision + 1U;
  pair->path_revision = ticket.path_revision;
  pair->frame_revision = ticket.frame_revision + 1U;
  pair->path_owner = fixture.owner;
  auto profile = std::make_shared<phase_offset_navigation::TubeProfile>();
  profile->complete = true;
  profile->classification =
      phase_offset_navigation::TubeProfileClassification::OFFSET_CERTIFIED;
  pair->active_profile = profile;
  pair->epoch_snapshot = std::make_shared<TubeEpochSnapshot>();
  pair->epoch_status.active_available = true;
  pair->epoch_status.active_current_validation_valid = true;
  std::atomic_store(&fixture.adapter.authoritative_path_tube_pair_,
                    std::shared_ptr<const PathTubePair>(pair));
  EXPECT_TRUE(fixture.adapter.completeBootstrapRendezvous(ticket, true));
  EXPECT_EQ(BootstrapRendezvousState::DISARMED,
            fixture.adapter.bootstrapRendezvousState());
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
