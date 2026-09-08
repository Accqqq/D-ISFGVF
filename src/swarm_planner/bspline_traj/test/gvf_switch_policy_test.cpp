#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#define private public
#include <plan_env/sdf_map.h>
#undef private
#include <bspline_race/gvf_manager.h>
#include <phase_offset_core/geometry.h>


namespace FLAG_Race {

// Narrow test-only access for current production phase/governor and V2
// publish-first seams.  Retired Pair/READY/mailbox helpers are intentionally
// absent from this Stage-7 fixture.
class GvfManagerS4AnchorTestAccess {
 public:
  struct PhaseSnapshotProbe {
    double w = 0.0;
    bool initialized = false;
    bool closed_acquired = false;
    std::uint64_t generation = 0U;
  };

  static PhaseSnapshotProbe capturePhase(const gvf_manager& manager) {
    const gvf_manager::AuthoritativePhaseSnapshot snapshot =
        manager.captureAuthoritativePhase();
    PhaseSnapshotProbe probe;
    probe.w = snapshot.w;
    probe.initialized = snapshot.initialized;
    probe.closed_acquired = snapshot.closed_acquired;
    probe.generation = snapshot.generation;
    return probe;
  }

  static void publishPhase(gvf_manager& manager, const double w,
                           const bool initialized,
                           const bool closed_acquired) {
    manager.publishAuthoritativePhase(w, initialized, closed_acquired);
  }

  static bool commitPhase(gvf_manager& manager,
                          const PhaseSnapshotProbe& captured,
                          const double w,
                          const bool acquire_closed_phase,
                          PhaseSnapshotProbe& committed) {
    gvf_manager::AuthoritativePhaseSnapshot internal;
    internal.w = captured.w;
    internal.initialized = captured.initialized;
    internal.closed_acquired = captured.closed_acquired;
    internal.generation = captured.generation;
    gvf_manager::AuthoritativePhaseSnapshot internal_committed;
    const bool accepted = manager.commitAuthoritativePhase(
        internal, w, acquire_closed_phase, internal_committed);
    committed.w = internal_committed.w;
    committed.initialized = internal_committed.initialized;
    committed.closed_acquired = internal_committed.closed_acquired;
    committed.generation = internal_committed.generation;
    return accepted;
  }

  static bool resetForNewNavigationTask(gvf_manager& manager) {
    return manager.resetForNewNavigationTask();
  }

  static gvf_manager::GovernorCommandResult invalidHold(
      gvf_manager& manager, const Eigen::Vector3d& position,
      const std::string& reason) {
    gvf_manager::GovernorCommandDebug debug;
    return manager.makeGovernorInvalidHold(position, reason, debug);
  }

  static bool publishGovernorCandidate(gvf_manager& manager,
                                        const Eigen::Vector3d& position,
                                        const Eigen::Vector3d& yaw) {
    return manager.publishGovernorPositionCommand(position, yaw);
  }

  struct GovernorProbe {
    Eigen::Vector3d cmd_pos = Eigen::Vector3d::Zero();
    bool command_valid = false;
  };

  static PendingPositionCommandCapture pendingCommand(gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_
        ? manager.phase_offset_matched_adapter_->capturePendingPositionCommand()
        : PendingPositionCommandCapture();
  }

  static bool publishPendingCommand(gvf_manager& manager,
                                    const std::function<bool()>& local_publish,
                                    const std::uint64_t identity) {
    return manager.phase_offset_matched_adapter_ &&
        manager.phase_offset_matched_adapter_->publishPendingPositionCommand(
            local_publish, identity);
  }

  struct RuntimeProbe {
    bool present = false;
    double retained_delta = 0.0;
    phase_offset_core::PortCommand previous_port;
  };

  static RuntimeProbe runtimeState(gvf_manager& manager) {
    RuntimeProbe probe;
    if (!manager.phase_offset_matched_adapter_ ||
        !manager.phase_offset_matched_adapter_->runtime_) {
      return probe;
    }
    std::lock_guard<std::mutex> lock(
        manager.phase_offset_matched_adapter_->runtime_command_mutex_);
    probe.present = true;
    probe.retained_delta =
        manager.phase_offset_matched_adapter_->runtime_->retainedDelta();
    probe.previous_port =
        manager.phase_offset_matched_adapter_->runtime_->previousFinalPort();
    return probe;
  }

  static guidance::IsfGains v2Gains() {
    guidance::IsfGains gains;
    gains.k1 = 2.0;
    gains.k2 = -2.2;
    gains.convergence_bandwidth = 0.1;
    gains.progress_rho0 = 0.5;
    gains.progress_delta = 0.3;
    gains.alpha_min = 0.05;
    return gains;
  }

  static PhaseOffsetMatchedAdapterConfig v2Config() {
    PhaseOffsetMatchedAdapterConfig config;
    config.mode = PhaseOffsetMatchedMode::MANUAL;
    config.tube_source = phase_offset_navigation::TubeSource::ESDF;
    config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
    config.observe_only = false;
    config.profile_period = 2.0;
    config.warmup_cycles = 100;
    config.u_w_rate_max = 100.0;
    config.u_delta_rate_max = 100.0;
    config.u_delta_abs_max = 0.40;
    config.tube_update_period = 0.10;
    config.normal_preview_policy.preview_horizon_w = 1.0;
    config.normal_preview_policy.sample_spacing_w = 0.25;
    config.normal_preview_policy.lower_nu = 0.02;
    config.normal_preview_policy.upper_nu = 2.0;
    config.normal_preview_policy.b_tight = 0.10;
    config.normal_preview_policy.b_open = 0.90;
    config.normal_preview_policy.policy_revision = 1U;
    config.normal_preview_policy.configuration_identity = 31U;
    config.normal_preview_policy.configuration_id = "s7-t08-preview";
    config.normal_preview_policy_explicit = true;
    config.tube.cross_section.regularity_margin = 0.10;
    config.tube.cross_section.minimum_reference_speed = 1e-8;
    config.tube.cross_section.margins.tracking_error_bound = 0.15;
    return config;
  }

  static MatchedAdapterInput v2GateInput(const double stamp) {
    MatchedAdapterInput input;
    input.path.p = Eigen::Vector3d(0.25, 0.0, 1.0);
    input.path.p_w = Eigen::Vector3d::UnitX();
    input.path.p_ww.setZero();
    input.path.path_revision = 11U;
    input.path.frame_revision = 12U;
    input.path.T = Eigen::Vector3d::UnitX();
    input.path.N = Eigen::Vector3d::UnitY();
    input.path.N_w.setZero();
    input.path.frame_valid = true;
    input.path.frame_provenance =
        phase_offset_core::kWorldHorizontalCrossProductProvenance;
    input.path.w = 0.25;
    input.path.valid = true;
    input.semantic_path_start_w = 0.0;
    input.semantic_path_end_w = 2.0;
    input.position = input.path.p;
    input.gains = v2Gains();
    input.dt = 0.10;
    input.stamp = ros::Time(stamp);
    PhaseOffsetActiveAdapter zero;
    ActiveAdapterInput zero_input;
    zero_input.path = input.path;
    zero_input.position = input.position;
    zero_input.gains = input.gains;
    ActiveAdapterOutput zero_output;
    if (!zero.evaluate(zero_input, zero_output)) return MatchedAdapterInput();
    input.legacy = LegacyGuidanceSnapshot(
        zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
        zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
        zero_output.guidance.ref_pt, zero_output.guidance.tangent,
        zero_output.guidance.valid);
    return input;
  }

  static bool installNeutralAdapter(gvf_manager& manager) {
    PhaseOffsetMatchedAdapterConfig config = v2Config();
    config.tube_certificate_v2.configuration_id =
        config.normal_preview_policy.configuration_identity;
    config.tube_certificate_v2.epsilon = 0.10;
    config.tube_certificate_v2.nominal_half_width = 0.50;
    config.tube_certificate_v2.ray_step = 0.05;
    config.tube_certificate_v2.snapshot_resolution = 0.05;
    config.tube_certificate_v2.minimum_reference_speed = 1e-8;
    config.tube_certificate_v2.sample_step_w = 0.25;
    std::unique_ptr<PhaseOffsetMatchedAdapter> adapter(
        new PhaseOffsetMatchedAdapter(config));
    if (!adapter->configurationValid()) return false;
    manager.matched_config_ = config;
    manager.phase_offset_matched_adapter_ = std::move(adapter);
    return true;
  }

  static bool initializePlannerFrontend(
      gvf_manager& manager,
      const std::shared_ptr<const ContinuousPhasePath>& owner) {
    if (!owner || owner->empty()) return false;
    gvf_manager::gvfManager frontend;
    frontend.gvf_.reset(new gvf());
    frontend.gvf_->setAuthoritativePhaseMode(true);
    frontend.gvf_->setContinuousPhasePath(owner);
    manager.swarmParticlesManager.clear();
    manager.swarmParticlesManager.push_back(frontend);
    return true;
  }

  static bool installPlannerOnly(
      gvf_manager& manager,
      const std::shared_ptr<const ContinuousPhasePath>& owner,
      const std::shared_ptr<const ContinuousPhasePath>& copied_prefix_source =
          std::shared_ptr<const ContinuousPhasePath>(),
      double copied_prefix_end_w =
          std::numeric_limits<double>::quiet_NaN(),
      const std::uint64_t expected_execution_generation = 0U,
      double live_phase_before_install =
          std::numeric_limits<double>::quiet_NaN(),
      const bool reset_adapter_task_before_install = false) {
    if (!owner || manager.swarmParticlesManager.empty()) return false;
    const gvf_manager::AuthoritativePhaseSnapshot expected_phase =
        manager.captureAuthoritativePhase();
    if (!std::isfinite(copied_prefix_end_w)) {
      copied_prefix_end_w = expected_phase.w;
    }
    if (std::isfinite(live_phase_before_install)) {
      manager.publishAuthoritativePhase(
          live_phase_before_install, expected_phase.initialized,
          expected_phase.closed_acquired);
    }
    if (reset_adapter_task_before_install &&
        (!manager.phase_offset_matched_adapter_ ||
         !manager.phase_offset_matched_adapter_->resetForNewNavigationTask())) {
      return false;
    }
    const std::vector<double> w{
        owner->startW(),
        0.5 * (owner->startW() + owner->endW()), owner->endW()};
    Eigen::MatrixXd traj(3, 3);
    Eigen::MatrixXd vel(3, 3);
    Eigen::VectorXd time(3);
    for (int index = 0; index < 3; ++index) {
      ContinuousPhasePathState state;
      if (!owner->evaluate(w[static_cast<std::size_t>(index)], state,
                           false)) {
        return false;
      }
      traj.row(index) = state.p.transpose();
      vel.row(index) = state.dp_dw.transpose();
      time(index) = static_cast<double>(index);
    }
    nav_msgs::Path path_msg;
    return manager.installPlannerOnlyFrontendV2(
        manager.swarmParticlesManager.front(), traj, vel, time, w, owner,
        ros::Time(7, 0), expected_phase, copied_prefix_source,
        copied_prefix_end_w, expected_execution_generation, path_msg);
  }

  static std::uint64_t executionGenerationV2(const gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_
        ? manager.phase_offset_matched_adapter_->executionGenerationV2() : 0U;
  }

  static int currentTrajectoryIndex(const gvf_manager& manager) {
    return manager.current_traj_index_;
  }

  static bool plannerOnlyFutureSeam(
      const std::shared_ptr<const ContinuousPhasePath>& source,
      const double phase_at_switch,
      const double construction_lead_w,
      double& seam_w) {
    return gvf_manager::plannerOnlyFutureSeamV2(
        source, phase_at_switch, construction_lead_w, seam_w);
  }

  static void clearPendingPathReferenceV2(gvf_manager& manager) {
    std::lock_guard<std::mutex> lock(manager.path_reference_handoff_mutex_);
    manager.pending_path_reference_handoff_v2_.reset();
  }

  struct V2ManagerTransactionFixture {
    std::shared_ptr<const ContinuousPhasePath> source_owner;
    std::shared_ptr<const ContinuousPhasePath> successor_owner;
    std::shared_ptr<const TubeV2ExecutionBinding> source_binding;
    std::shared_ptr<const TubeV2ExecutionBinding> proposed_binding;
    std::shared_ptr<const PathReferenceHandoffV2> request_handoff;
    std::shared_ptr<const TubeV2ShadowAdmissionCandidate> candidate;
    PendingPositionCommandCapture pending_command;
    double phase_before = 0.0;
    double phase_after = 0.0;
  };

  struct V2PublishProbe {
    bool attempted = false;
    bool published = false;
    bool phase_token_prepared = false;
    int local_publish_count = 0;
    int post_publish_count = 0;
    bool local_saw_source_state = false;
    bool post_saw_atomic_adapter_commit = false;
    bool post_saw_old_phase = false;
  };

  static bool prepareV2ManagerTransaction(
      gvf_manager& manager, V2ManagerTransactionFixture& fixture);
  static V2PublishProbe publishV2ManagerTransaction(
      gvf_manager& manager, const V2ManagerTransactionFixture& fixture,
      bool local_publish_success);
  static std::shared_ptr<const PathReferenceHandoffV2>
      pendingPathReferenceV2(gvf_manager& manager);
  static std::shared_ptr<const ContinuousPhasePath> commandPathForV2(
      gvf_manager& manager,
      const std::shared_ptr<const TubeV2ExecutionBinding>& binding);
  static std::shared_ptr<const TubeV2ExecutionBinding>
      executionBindingV2(gvf_manager& manager);
  static bool consumeCommittedPathReferenceV2(gvf_manager& manager);
  static void installPrematureCommittedMirrorV2(
      gvf_manager& manager,
      const V2ManagerTransactionFixture& fixture);
  static void discardPendingV2Command(gvf_manager& manager);

  static bool point(
                    const gvf_manager& manager,
                    const std::shared_ptr<const ContinuousPhasePath>& path,
                    double w, double delta, Eigen::Vector3d& point,
                    bool& clamped, double& start_w, double& end_w) {
    return manager.pathPointAtW(path, w, delta, point, clamped, start_w, end_w);
  }

  static bool tangent(
                      const gvf_manager& manager,
                      const std::shared_ptr<const ContinuousPhasePath>& path,
                      double w, double delta, Eigen::Vector3d& tangent) {
    return manager.pathTangentAtW(path, w, delta, tangent);
  }

  static GovernorProbe runSingleAnchorCandidate(
      gvf_manager& manager,
      const std::shared_ptr<const ContinuousPhasePath>& path,
      double progress_w, double reference_delta) {
    manager.cmd_governor_l_min_ = 0.50;
    manager.cmd_governor_l_max_ = 0.50;
    manager.cmd_governor_l_step_ = 0.05;
    manager.cmd_governor_normal_max_ = 0.0;
    manager.cmd_governor_initialized_ = false;
    manager.cmd_governor_normal_state_.setZero();
    manager.cmd_governor_last_l_ = 0.0;
    gvf::LiftedGuidanceResult out;
    out.valid = true;
    ContinuousPhasePathState state;
    if (!path || !path->evaluate(progress_w, state, false) ||
        state.dp_dw.norm() <= 1e-6) {
      return GovernorProbe();
    }
    out.tangent = state.dp_dw.normalized();
    out.v_cmd = 0.50 * out.tangent;
    out.w_dot = 0.50;
    out.e_perp.setZero();
    gvf_manager::GovernorCommandDebug debug;
    const gvf_manager::GovernorCommandResult result =
        manager.runVelocityMatchingGovernor(
            path, out, Eigen::Vector3d::Zero(), progress_w,
            reference_delta, 0.02, 1.0, debug);
    GovernorProbe probe;
    probe.cmd_pos = result.cmd_pos;
    probe.command_valid = result.command_valid;
    return probe;
  }
};
bool GvfManagerS4AnchorTestAccess::prepareV2ManagerTransaction(
    gvf_manager& manager, V2ManagerTransactionFixture& fixture) {
  fixture = V2ManagerTransactionFixture();

  const auto interval = [](const double lower, const double upper) {
    phase_offset_core::Binary64Interval value;
    value.lower = lower;
    value.upper = upper;
    value.valid = true;
    return value;
  };
  const auto vector_interval = [&interval]() {
    phase_offset_core::Binary64VectorInterval value;
    value.valid = true;
    for (phase_offset_core::Binary64Interval& component : value.component) {
      component = interval(0.0, 0.0);
    }
    return value;
  };
  const auto path_cell = [&interval, &vector_interval](
      const phase_offset_navigation::TubePathKey& key, const double w0,
      const double w1, const std::uint64_t segment) {
    phase_offset_core::CertifiedPathCellV2 cell;
    cell.w0 = w0;
    cell.w1 = w1;
    cell.anchor_w = 0.5 * (w0 + w1);
    cell.path_revision = key.path_revision;
    cell.frame_revision = key.frame_revision;
    cell.segment_identity = segment;
    cell.proof_identity = 100U + segment;
    cell.anchor_position = vector_interval();
    cell.anchor_p_w = vector_interval();
    cell.anchor_p_ww = vector_interval();
    cell.inf_p_w_norm = interval(1.0, 1.0);
    cell.sup_p_w_norm = interval(1.0, 1.0);
    cell.inf_horizontal_p_w_norm = interval(1.0, 1.0);
    cell.sup_p_ww_norm = interval(0.0, 0.0);
    cell.sup_horizontal_p_ww_norm = interval(0.0, 0.0);
    cell.sup_p_www_norm = interval(0.0, 0.0);
    cell.sup_normal_derivative = interval(0.0, 0.0);
    cell.normal_variation = interval(0.0, 0.0);
    cell.tangent_variation = interval(0.0, 0.0);
    cell.curvature_variation = interval(0.0, 0.0);
    cell.midpoint_position_variation = interval(0.0, 0.0);
    cell.chord_deviation = interval(0.0, 0.0);
    cell.horizontal_acceleration_bound_complete = true;
    cell.normal_frame_proof_complete = true;
    cell.phase_map_proof_complete = true;
    cell.provenance =
        phase_offset_core::kWorldHorizontalCrossProductProvenance;
    cell.complete = true;
    cell.valid = true;
    return cell;
  };

  phase_offset_navigation::TubeConfigurationKey configuration_key;
  configuration_key.configuration_id = 31U;
  configuration_key.epsilon = 0.1;
  configuration_key.nominal_half_width = 0.5;
  configuration_key.ray_step = 0.05;
  configuration_key.snapshot_resolution = 0.05;
  configuration_key.minimum_reference_speed = 1e-8;
  phase_offset_navigation::TubeMapCaptureKey map_key;
  map_key.map_instance_id = 41U;
  map_key.state_id = 43U;
  map_key.accepted_sequence = 43U;
  map_key.configuration_generation = 44U;
  map_key.configuration_id = configuration_key.configuration_id;
  map_key.frame_provenance_id = 45U;
  map_key.frame_provenance = "s6c-t08-world";
  map_key.support_provenance_id = 46U;
  map_key.accepted_time_ticks = 100U;
  map_key.support_expiry_ticks = 10000U;
  map_key.support_halo = 0.1;
  map_key.halo_reconciled = true;
  map_key.grid_min_index_x = -10;
  map_key.grid_min_index_y = -10;
  map_key.grid_min_index_z = -10;
  map_key.grid_max_index_x = 10;
  map_key.grid_max_index_y = 10;
  map_key.grid_max_index_z = 10;
  map_key.grid_voxel_resolution = Eigen::Vector3d::Constant(0.05);
  map_key.complete_support = true;

  PhaseOffsetMatchedAdapterConfig config = v2Config();
  config.coordination_backend = PhaseOffsetCoordinationBackend::D1B;
  config.observe_only = false;
  config.tube_certificate_v2.configuration_id =
      configuration_key.configuration_id;
  config.tube_certificate_v2.epsilon = configuration_key.epsilon;
  config.tube_certificate_v2.nominal_half_width =
      configuration_key.nominal_half_width;
  config.tube_certificate_v2.ray_step = configuration_key.ray_step;
  config.tube_certificate_v2.snapshot_resolution =
      configuration_key.snapshot_resolution;
  config.tube_certificate_v2.minimum_reference_speed =
      configuration_key.minimum_reference_speed;
  config.tube_certificate_v2.sample_step_w = 0.25;
  config.normal_preview_policy.preview_horizon_w = 1.0;
  config.normal_preview_policy.sample_spacing_w = 0.25;
  config.normal_preview_policy.lower_nu = 0.02;
  config.normal_preview_policy.upper_nu = 2.0;
  config.normal_preview_policy.b_tight = 0.1;
  config.normal_preview_policy.b_open = 0.9;
  config.normal_preview_policy.configuration_identity =
      configuration_key.configuration_id;
  config.normal_preview_policy.configuration_id = "s6c-t08-preview";

  std::unique_ptr<PhaseOffsetMatchedAdapter> adapter(
      new PhaseOffsetMatchedAdapter(config));
  if (!adapter->configurationValid()) {
    return false;
  }
  const std::uint64_t execution_generation =
      adapter->task_generation_.load(std::memory_order_acquire);

  const auto make_path = [](const std::uint64_t revision) {
    std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
    if (!path->appendSegment(
            0.0, 2.0, "s6c-t08-straight",
            [](const double w, ContinuousPhasePathState& state) {
              state.p = Eigen::Vector3d(w, 0.0, 1.0);
              state.dp_dw = Eigen::Vector3d::UnitX();
              state.d2p_dw2.setZero();
              state.vel = state.dp_dw;
              state.valid = std::isfinite(w);
              return state.valid;
            })) {
      return std::shared_ptr<const ContinuousPhasePath>();
    }
    path->setPathRevision(revision);
    return std::shared_ptr<const ContinuousPhasePath>(path);
  };
  fixture.source_owner = make_path(11U);
  if (!fixture.source_owner) return false;

  const auto make_profile = [&](const phase_offset_navigation::TubePathKey& key,
                                const std::uint64_t profile_id,
                                const std::uint64_t request_id,
                                const double requested_start,
                                const std::shared_ptr<const ContinuousPhasePath>&
                                    owner) {
    std::shared_ptr<phase_offset_navigation::TubeProfileV2> profile(
        new phase_offset_navigation::TubeProfileV2());
    profile->path_key = key;
    profile->configuration_key = configuration_key;
    profile->map_capture_key = map_key;
    profile->profile_id = profile_id;
    profile->request_id = request_id;
    profile->requested_start = requested_start;
    profile->requested_end = 2.0;
    profile->anchor_w = requested_start;
    profile->certified_start = 0.0;
    profile->certified_end = 2.0;
    profile->valid = true;
    profile->complete = true;
    profile->contains_anchor = true;
    profile->contains_zero_everywhere = true;
    profile->nonzero_capacity = true;
    profile->capability =
        phase_offset_navigation::TubeProfileV2Capability::OFFSET_CERTIFIED;
    profile->path_owner = std::static_pointer_cast<const void>(owner);
    profile->capture_owner =
        std::static_pointer_cast<const void>(std::make_shared<int>(2));
    profile->query_owner =
        std::static_pointer_cast<const void>(std::make_shared<int>(3));
    profile->applicability_assumptions = "s6c-t08-complete-support";
    profile->applicability_deadline_ticks = 10000U;

    phase_offset_navigation::TubePwlKnotV2 first;
    first.w = 0.0;
    first.lower = -0.5;
    first.upper = 0.5;
    first.right_cell_id = 61U;
    first.right_lower_slope_interval = {0.0, 0.0, true};
    first.right_upper_slope_interval = {0.0, 0.0, true};
    first.valid = true;
    phase_offset_navigation::TubePwlKnotV2 middle = first;
    middle.w = 1.0;
    middle.left_cell_id = 61U;
    middle.right_cell_id = 62U;
    middle.left_lower_slope_interval = {0.0, 0.0, true};
    middle.left_upper_slope_interval = {0.0, 0.0, true};
    middle.right_lower_slope_interval = {0.0, 0.0, true};
    middle.right_upper_slope_interval = {0.0, 0.0, true};
    phase_offset_navigation::TubePwlKnotV2 last = middle;
    last.w = 2.0;
    last.left_cell_id = 62U;
    last.right_cell_id = 0U;
    last.right_lower_slope_interval = {};
    last.right_upper_slope_interval = {};
    profile->knots = {first, middle, last};

    phase_offset_navigation::TubeProofCellV2 cell0;
    cell0.w0 = 0.0;
    cell0.w1 = 1.0;
    cell0.lower = -0.5;
    cell0.upper = 0.5;
    cell0.cell_id = 61U;
    cell0.valid = true;
    cell0.complete = true;
    cell0.path_cell = path_cell(key, 0.0, 1.0, 71U);
    phase_offset_navigation::TubeProofCellV2 cell1 = cell0;
    cell1.w0 = 1.0;
    cell1.w1 = 2.0;
    cell1.cell_id = 62U;
    cell1.path_cell = path_cell(key, 1.0, 2.0, 72U);
    profile->cells = {cell0, cell1};
    return profile;
  };
  const auto make_input = [](
      const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>&
          profile) {
    std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> input(
        new phase_offset_navigation::TubeBuildInputV2());
    input->request_id = profile->request_id;
    input->path_key = profile->path_key;
    input->configuration_key = profile->configuration_key;
    input->map_capture_key = profile->map_capture_key;
    input->requested_start = profile->requested_start;
    input->requested_end = profile->requested_end;
    input->anchor_w = profile->anchor_w;
    for (const phase_offset_navigation::TubeProofCellV2& cell :
         profile->cells) {
      input->path_cells.push_back(cell.path_cell);
    }
    input->producer_breakpoints = {0.0, 1.0, 2.0};
    input->free_ball_query = [](
        const Eigen::Vector3d&, const double) {
      return phase_offset_navigation::TubeFreeBallQueryResult();
    };
    input->path_owner = profile->path_owner;
    input->capture_owner = profile->capture_owner;
    input->query_owner = profile->query_owner;
    input->applicability_assumptions = profile->applicability_assumptions;
    input->applicability_deadline_ticks =
        profile->applicability_deadline_ticks;
    input->applicability_deadline_timeless =
        profile->applicability_deadline_timeless;
    return std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
        input);
  };
  const auto completion = [](
      const TubeWorkerPurposeV2 purpose,
      const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>&
          profile) {
    std::shared_ptr<TubeWorkerCompletionV2> value(
        new TubeWorkerCompletionV2());
    value->status = TubeWorkerCompletionStatusV2::BUILT;
    value->purpose = purpose;
    value->request_id = profile->request_id;
    value->execution_generation = profile->path_key.execution_generation;
    value->accepted_state_demand =
        profile->map_capture_key.accepted_sequence;
    value->useful_start = profile->requested_start;
    value->useful_end = profile->requested_end;
    value->path_key = profile->path_key;
    value->configuration_key = profile->configuration_key;
    value->map_capture_key = profile->map_capture_key;
    value->heavy_build_entered = true;
    value->build.profile = *profile;
    value->build.success = true;
    return std::shared_ptr<const TubeWorkerCompletionV2>(value);
  };
  const auto populate_admission = [&config, &map_key](
      MatchedAdapterInput& input, const std::uint64_t binding_sequence) {
    input.tube_v2_admission.valid = true;
    input.tube_v2_admission.binding_sequence = binding_sequence;
    input.tube_v2_admission.accepted_state_sequence = map_key.accepted_sequence;
    input.tube_v2_admission.accepted_state_notification_sequence =
        map_key.accepted_sequence;
    input.tube_v2_admission.accepted_time_ticks = map_key.accepted_time_ticks;
    input.tube_v2_admission.support_expiry_ticks =
        map_key.support_expiry_ticks;
    input.tube_v2_admission.map_instance_id = map_key.map_instance_id;
    input.tube_v2_admission.configuration_generation =
        map_key.configuration_generation;
    input.tube_v2_admission.configuration_key = map_key.configuration_id;
    input.tube_v2_admission.support_provenance_id =
        map_key.support_provenance_id;
    input.tube_v2_admission.frame_provenance = map_key.frame_provenance;
    input.tube_v2_admission.latest_accepted_state_sequence =
        map_key.accepted_sequence;
    input.tube_v2_admission.latest_accepted_state_notification_sequence =
        map_key.accepted_sequence;
    input.tube_v2_admission.latest_accepted_time_ticks =
        map_key.accepted_time_ticks;
    input.tube_v2_admission.latest_map_instance_id = map_key.map_instance_id;
    input.tube_v2_admission.latest_configuration_generation =
        map_key.configuration_generation;
    input.tube_v2_admission.latest_configuration_key =
        map_key.configuration_id;
    input.tube_v2_admission.latest_support_provenance_id =
        map_key.support_provenance_id;
    input.tube_v2_admission.latest_frame_provenance =
        map_key.frame_provenance;
    input.tube_v2_admission.selected_u = phase_offset_core::PortCommand();
    input.tube_v2_admission.selected_u.u_delta = 0.05;
    input.tube_v2_admission.base_phase_rate = 0.10;
    input.tube_v2_admission.phase_rate_lower = 0.05;
    input.tube_v2_admission.phase_rate_upper = 0.50;
    input.tube_v2_admission.upper_u_delta = 0.20;
    input.tube_v2_admission.now = 100.0;
    input.tube_v2_admission.applicability_deadline = 10000.0;
    input.tube_v2_admission.applicability_deadline_valid = true;
    input.tube_v2_admission.preview_policy = config.normal_preview_policy;
    input.tube_v2_admission.limits.lower_phase_rate = 0.05;
    input.tube_v2_admission.limits.upper_phase_rate = 0.50;
    input.tube_v2_admission.limits.upper_nu = 0.50;
    input.tube_v2_admission.limits.max_u_w = 0.50;
    input.tube_v2_admission.limits.max_u_delta = 1.0;
    input.tube_v2_admission.limits.u_w_slew_rate = 1.0;
    input.tube_v2_admission.limits.u_delta_slew_rate = 1.0;
    input.tube_v2_admission.limits.return_u_delta_max = 0.50;
    input.tube_v2_admission.limits.return_u_delta_slew_rate = 0.50;
    input.tube_v2_admission.limits.max_schedule_steps = 200U;
    input.tube_v2_admission.limits.max_work = 1000U;
    input.tube_v2_admission.limits.valid = true;
    input.tube_v2_admission.tracking.valid = true;
    input.tube_v2_admission.tracking.error_norm = 0.0;
    input.tube_v2_admission.tracking.error_bound = 0.15;
    input.tube_v2_admission.tracking.physical_tangent_valid = true;
    input.tube_v2_admission.max_work = 1000U;
    input.tube_v2_admission.provenance = "S6C/T08/manager-transaction";
  };

  phase_offset_navigation::TubePathKey source_key;
  source_key.execution_generation = execution_generation;
  source_key.path_instance_id = 22U;
  source_key.path_revision = fixture.source_owner->pathRevision();
  source_key.frame_revision = 12U;
  source_key.frame_convention_id = 13U;
  source_key.frame_convention =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  source_key.phase_orientation = 1;
  source_key.domain_start = 0.0;
  source_key.domain_end = 2.0;
  const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>
      source_profile = make_profile(
          source_key, 51U, 52U, 0.0, fixture.source_owner);
  const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      source_input = make_input(source_profile);
  if (!source_profile->structurallyValid() || !source_input->complete()) {
    return false;
  }
  MatchedAdapterInput source_command = v2GateInput(100.0);
  ContinuousPhasePathState source_state;
  if (!fixture.source_owner->evaluate(0.25, source_state, false)) {
    return false;
  }
  source_command.path = ConvertContinuousPhasePathStateForActive(
      source_state, 0.25);
  source_command.path.path_revision = source_key.path_revision;
  source_command.path.frame_revision = source_key.frame_revision;
  source_command.path.T = Eigen::Vector3d::UnitX();
  source_command.path.N = Eigen::Vector3d::UnitY();
  source_command.path.N_w.setZero();
  source_command.path.frame_valid = true;
  source_command.path.frame_provenance = source_key.frame_convention;
  source_command.position = source_state.p;
  source_command.g_des = source_command.path.N * 0.05;
  source_command.g_des_valid = true;
  source_command.semantic_path_owner = fixture.source_owner;
  source_command.semantic_path_start_w = fixture.source_owner->startW();
  source_command.semantic_path_end_w = fixture.source_owner->endW();
  source_command.dt = 0.10;
  source_command.tube_worker_input_v2 = source_input;
  populate_admission(source_command, 81U);

  // Exercise the unchanged production zero-port equivalence gate.  Stage 7
  // has no bootstrap-only bypass: the V2 candidate may become selectable
  // only after one hundred coherent command observations.
  ActiveAdapterInput zero_input;
  zero_input.path = source_command.path;
  zero_input.position = source_command.position;
  zero_input.gains = source_command.gains;
  ActiveAdapterOutput zero_output;
  if (!adapter->zero_port_adapter_.evaluate(zero_input, zero_output)) {
    return false;
  }
  source_command.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
  for (int cycle = 0; cycle < config.warmup_cycles; ++cycle) {
    MatchedAdapterOutput gate_output;
    if (!adapter->updateGate(source_command, gate_output)) {
      return false;
    }
  }
  if (!adapter->zero_gate_open_) return false;

  const std::shared_ptr<const TubeWorkerCompletionV2> source_completion =
      completion(TubeWorkerPurposeV2::CURRENT, source_profile);
  {
    std::lock_guard<std::mutex> worker_lock(adapter->worker_state_mutex_);
    adapter->latest_v2_shadow_current_completion_ = source_completion;
  }
  MatchedAdapterOutput source_output;
  if (!adapter->update(source_command, source_output) ||
      !source_output.selected || !source_output.valid ||
      !source_output.v2_shadow_admission_candidate) {
    return false;
  }
  const std::shared_ptr<const TubeV2ShadowAdmissionCandidate> source_candidate =
      source_output.v2_shadow_admission_candidate;
  const PendingPositionCommandCapture source_pending =
      adapter->capturePendingPositionCommand();
  if (!source_pending.pending || !source_pending.valid ||
      !adapter->publishPendingPositionCommand(
          []() { return true; }, source_pending.identity)) {
    return false;
  }
  fixture.source_binding = adapter->captureV2ExecutionBinding();
  if (!fixture.source_binding || !fixture.source_binding->complete()) {
    return false;
  }

  const double live_w = source_candidate->prepared_step.successor.w;
  std::shared_ptr<ContinuousPhasePath> successor(new ContinuousPhasePath());
  if (!successor->appendSlice(*fixture.source_owner, 0.0, 1.0) ||
      !successor->appendSegment(
          1.0, 2.0, "s6c-t08-successor-tail",
          [](const double w, ContinuousPhasePathState& state) {
            state.p = Eigen::Vector3d(w, 0.0, 1.0);
            state.dp_dw = Eigen::Vector3d::UnitX();
            state.d2p_dw2.setZero();
            state.vel = state.dp_dw;
            state.valid = std::isfinite(w);
            return state.valid;
          })) {
    return false;
  }
  successor->setPathRevision(14U);
  fixture.successor_owner = successor;
  phase_offset_navigation::TubePathKey successor_key = source_key;
  ++successor_key.path_instance_id;
  successor_key.path_revision = fixture.successor_owner->pathRevision();
  successor_key.frame_revision = 15U;
  const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>
      successor_profile = make_profile(
          successor_key, 53U, 54U, live_w, fixture.successor_owner);
  const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      successor_input = make_input(successor_profile);
  if (!successor_profile->structurallyValid() ||
      !successor_input->complete()) {
    return false;
  }

  std::shared_ptr<TubeV2SuccessorHandoffEvidence> admission(
      new TubeV2SuccessorHandoffEvidence());
  admission->valid = true;
  admission->structurally_copied_prefix = true;
  admission->expected_execution_generation = execution_generation;
  admission->source_path_key = source_key;
  admission->source_path_owner = fixture.source_owner;
  admission->successor_path_key = successor_key;
  admission->successor_path_owner = fixture.successor_owner;
  admission->phase_after_w = live_w;
  admission->copied_prefix_start_w = live_w;
  admission->copied_prefix_end_w = 1.0;
  admission->successor_request = successor_input;
  admission->provenance = "S6C/T08/appendSlice";

  MatchedAdapterInput successor_command = source_command;
  if (!fixture.source_owner->evaluate(live_w, source_state, false)) return false;
  successor_command.path = ConvertContinuousPhasePathStateForActive(
      source_state, live_w);
  successor_command.path.path_revision = source_key.path_revision;
  successor_command.path.frame_revision = source_key.frame_revision;
  successor_command.path.T = Eigen::Vector3d::UnitX();
  successor_command.path.N = Eigen::Vector3d::UnitY();
  successor_command.path.N_w.setZero();
  successor_command.path.frame_valid = true;
  successor_command.path.frame_provenance = source_key.frame_convention;
  successor_command.position = source_state.p;
  ActiveAdapterInput successor_zero_input;
  successor_zero_input.path = successor_command.path;
  successor_zero_input.position = successor_command.position;
  successor_zero_input.gains = successor_command.gains;
  ActiveAdapterOutput successor_zero_output;
  if (!adapter->zero_port_adapter_.evaluate(
          successor_zero_input, successor_zero_output)) {
    return false;
  }
  successor_command.legacy = LegacyGuidanceSnapshot(
      successor_zero_output.guidance.v_cmd,
      successor_zero_output.guidance.w_dot,
      successor_zero_output.guidance.e_parallel,
      successor_zero_output.guidance.e_perp,
      successor_zero_output.guidance.ref_pt,
      successor_zero_output.guidance.tangent,
      successor_zero_output.guidance.valid);
  successor_command.tube_v2_successor_handoff = admission;
  populate_admission(successor_command, 81U);
  const std::shared_ptr<const TubeWorkerCompletionV2> successor_completion =
      completion(TubeWorkerPurposeV2::SUCCESSOR, successor_profile);
  {
    std::lock_guard<std::mutex> worker_lock(adapter->worker_state_mutex_);
    adapter->latest_v2_shadow_successor_completion_ = successor_completion;
  }
  MatchedAdapterOutput successor_output;
  if (!adapter->update(successor_command, successor_output) ||
      !successor_output.selected || !successor_output.valid ||
      !successor_output.v2_shadow_admission_candidate) {
    return false;
  }
  fixture.candidate = successor_output.v2_shadow_admission_candidate;
  fixture.pending_command = adapter->capturePendingPositionCommand();
  fixture.proposed_binding = fixture.pending_command.proposed_v2_binding;
  if (!fixture.pending_command.pending || !fixture.pending_command.valid ||
      !fixture.pending_command.v2_binding_transition ||
      !fixture.proposed_binding || !fixture.proposed_binding->complete()) {
    return false;
  }

  std::shared_ptr<PathReferenceFrontendMirrorV2> mirror(
      new PathReferenceFrontendMirrorV2());
  const std::vector<double> mirror_w{0.0, live_w, 1.0, 2.0};
  mirror->traj.resize(4, 3);
  mirror->vel.resize(4, 3);
  mirror->time.resize(4);
  mirror->w = mirror_w;
  mirror->anchor_idx = 1;
  for (int index = 0; index < 4; ++index) {
    ContinuousPhasePathState state;
    if (!fixture.successor_owner->evaluate(
            mirror_w[static_cast<std::size_t>(index)], state, false)) {
      return false;
    }
    mirror->traj.row(index) = state.p.transpose();
    mirror->vel.row(index) = state.dp_dw.transpose();
    mirror->time(index) = static_cast<double>(index);
  }
  std::shared_ptr<PathReferenceHandoffV2> handoff(
      new PathReferenceHandoffV2());
  handoff->expected_execution_generation = execution_generation;
  handoff->source_path_key = source_key;
  handoff->successor_path_owner = fixture.successor_owner;
  handoff->successor_path_key = successor_key;
  handoff->successor_phase_after_w = live_w;
  handoff->copied_prefix_start_w = live_w;
  handoff->copied_prefix_end_w = 1.0;
  handoff->successor_request = successor_input;
  handoff->admission_evidence = admission;
  handoff->frontend_mirror = mirror;
  if (!handoff->requestComplete()) return false;
  fixture.request_handoff = handoff;

  manager.matched_config_ = config;
  manager.phase_offset_matched_adapter_ = std::move(adapter);
  gvf_manager::gvfManager frontend;
  frontend.gvf_.reset(new gvf());
  frontend.gvf_->setAuthoritativePhaseMode(true);
  frontend.gvf_->setContinuousPhasePath(fixture.source_owner);
  const std::vector<double> source_w{0.0, 1.0, 2.0};
  frontend.last_traj.resize(3, 3);
  frontend.last_vel.resize(3, 3);
  frontend.last_traj_time_.resize(3);
  for (int index = 0; index < 3; ++index) {
    ContinuousPhasePathState state;
    if (!fixture.source_owner->evaluate(
            source_w[static_cast<std::size_t>(index)], state, false)) {
      return false;
    }
    frontend.last_traj.row(index) = state.p.transpose();
    frontend.last_vel.row(index) = state.dp_dw.transpose();
    frontend.last_traj_time_(index) = static_cast<double>(index);
  }
  frontend.gvf_->setNextPathWSamples(source_w);
  manager.swarmParticlesManager.clear();
  manager.swarmParticlesManager.push_back(frontend);
  manager.publishAuthoritativePhase(live_w, true, false);
  {
    std::lock_guard<std::mutex> handoff_lock(
        manager.path_reference_handoff_mutex_);
    manager.pending_path_reference_handoff_v2_ = fixture.request_handoff;
  }
  fixture.phase_before = live_w;
  fixture.phase_after = fixture.candidate->prepared_step.successor.w;
  return true;
}

GvfManagerS4AnchorTestAccess::V2PublishProbe
GvfManagerS4AnchorTestAccess::publishV2ManagerTransaction(
    gvf_manager& manager, const V2ManagerTransactionFixture& fixture,
    const bool local_publish_success) {
  V2PublishProbe probe;
  if (!manager.phase_offset_matched_adapter_ ||
      manager.swarmParticlesManager.empty() ||
      !fixture.source_binding || !fixture.proposed_binding ||
      !fixture.request_handoff || !fixture.candidate ||
      !fixture.pending_command.pending ||
      !fixture.pending_command.valid ||
      !fixture.pending_command.v2_binding_transition ||
      fixture.pending_command.proposed_v2_binding !=
          fixture.proposed_binding) {
    return probe;
  }
  PhaseOffsetMatchedAdapter& adapter =
      *manager.phase_offset_matched_adapter_;
  const gvf_manager::AuthoritativePhaseSnapshot captured_phase =
      manager.captureAuthoritativePhase();
  std::shared_ptr<const PathReferenceHandoffV2> expected_handoff;
  {
    std::lock_guard<std::mutex> handoff_lock(
        manager.path_reference_handoff_mutex_);
    expected_handoff = manager.pending_path_reference_handoff_v2_;
  }
  if (expected_handoff != fixture.request_handoff ||
      !expected_handoff->requestComplete() ||
      expected_handoff->successor_profile ||
      fixture.proposed_binding->source_input !=
          expected_handoff->successor_request ||
      fixture.proposed_binding->profile->path_key !=
          expected_handoff->successor_path_key) {
    return probe;
  }
  std::shared_ptr<PathReferenceHandoffV2> enriched_mutable(
      new PathReferenceHandoffV2(*expected_handoff));
  enriched_mutable->successor_profile = fixture.proposed_binding->profile;
  const std::shared_ptr<const PathReferenceHandoffV2> enriched_handoff =
      enriched_mutable;
  if (!enriched_handoff->committedComplete()) return probe;

  std::unique_lock<std::mutex> frontend_lock(manager.frontend_apply_mutex_);
  std::unique_lock<std::mutex> handoff_lock(
      manager.path_reference_handoff_mutex_);
  if (manager.pending_path_reference_handoff_v2_ != expected_handoff) {
    return probe;
  }
  std::unique_lock<std::mutex> phase_lock(
      manager.authoritative_phase_mutex_);
  gvf_manager::AuthoritativePhaseCommitToken phase_token;
  probe.phase_token_prepared =
      manager.prepareAuthoritativePhaseCommitLocked(
          captured_phase, fixture.phase_after, false, phase_token);
  if (!probe.phase_token_prepared || !phase_token.valid) return probe;

  probe.attempted = true;
  probe.published = adapter.publishPendingPositionCommand(
      [&]() {
        ++probe.local_publish_count;
        const phase_offset_core::PortCommand previous =
            adapter.runtime_->previousFinalPort();
        probe.local_saw_source_state =
            adapter.v2_execution_binding_ == fixture.source_binding &&
            adapter.runtime_->retainedDelta() ==
                fixture.candidate->prepared_step.expected_current.delta &&
            previous.u_w == fixture.candidate->prepared_step
                                .expected_current.previous_u.u_w &&
            previous.u_delta == fixture.candidate->prepared_step
                                    .expected_current.previous_u.u_delta &&
            manager.phase_w_ == captured_phase.w &&
            manager.authoritative_phase_generation_ ==
                captured_phase.generation &&
            manager.pending_path_reference_handoff_v2_ ==
                expected_handoff &&
            manager.swarmParticlesManager.front().gvf_ &&
            manager.swarmParticlesManager.front().gvf_
                    ->getContinuousPhasePath() == fixture.source_owner;
        return local_publish_success;
      },
      fixture.pending_command.identity, false,
      [&]() {
        ++probe.post_publish_count;
        const phase_offset_core::PortCommand previous =
            adapter.runtime_->previousFinalPort();
        probe.post_saw_atomic_adapter_commit =
            adapter.v2_execution_binding_ == fixture.proposed_binding &&
            adapter.runtime_->retainedDelta() ==
                fixture.candidate->prepared_step.successor.delta &&
            previous.u_w == fixture.candidate->prepared_step
                                .successor.previous_u.u_w &&
            previous.u_delta == fixture.candidate->prepared_step
                                    .successor.previous_u.u_delta;
        probe.post_saw_old_phase =
            manager.phase_w_ == captured_phase.w &&
            manager.authoritative_phase_generation_ ==
                captured_phase.generation;
        manager.commitAuthoritativePhaseNoFailLocked(phase_token);
        manager.pending_path_reference_handoff_v2_ = enriched_handoff;
      });
  return probe;
}

std::shared_ptr<const PathReferenceHandoffV2>
GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
    gvf_manager& manager) {
  std::lock_guard<std::mutex> handoff_lock(
      manager.path_reference_handoff_mutex_);
  return manager.pending_path_reference_handoff_v2_;
}

std::shared_ptr<const ContinuousPhasePath>
GvfManagerS4AnchorTestAccess::commandPathForV2(
    gvf_manager& manager,
    const std::shared_ptr<const TubeV2ExecutionBinding>& binding) {
  return manager.swarmParticlesManager.empty()
      ? std::shared_ptr<const ContinuousPhasePath>()
      : manager.captureCommandPathForV2Binding(
            manager.swarmParticlesManager.front(), binding);
}

std::shared_ptr<const TubeV2ExecutionBinding>
GvfManagerS4AnchorTestAccess::executionBindingV2(gvf_manager& manager) {
  return manager.phase_offset_matched_adapter_
      ? manager.phase_offset_matched_adapter_->captureV2ExecutionBinding()
      : std::shared_ptr<const TubeV2ExecutionBinding>();
}

bool GvfManagerS4AnchorTestAccess::consumeCommittedPathReferenceV2(
    gvf_manager& manager) {
  return !manager.swarmParticlesManager.empty() &&
      manager.consumeCommittedPathReferenceHandoffV2(
          manager.swarmParticlesManager.front(), ros::Time(7, 0));
}

void GvfManagerS4AnchorTestAccess::installPrematureCommittedMirrorV2(
    gvf_manager& manager,
    const V2ManagerTransactionFixture& fixture) {
  if (!fixture.request_handoff || !fixture.proposed_binding) return;
  std::shared_ptr<PathReferenceHandoffV2> enriched(
      new PathReferenceHandoffV2(*fixture.request_handoff));
  enriched->successor_profile = fixture.proposed_binding->profile;
  if (!enriched->committedComplete()) return;
  std::lock_guard<std::mutex> handoff_lock(
      manager.path_reference_handoff_mutex_);
  if (manager.pending_path_reference_handoff_v2_ ==
      fixture.request_handoff) {
    manager.pending_path_reference_handoff_v2_ = enriched;
  }
}

void GvfManagerS4AnchorTestAccess::discardPendingV2Command(
    gvf_manager& manager) {
  if (manager.phase_offset_matched_adapter_) {
    manager.phase_offset_matched_adapter_->discardPendingPositionCommand();
  }
}

}  // namespace FLAG_Race

namespace {
std::shared_ptr<FLAG_Race::gvf> makeS4AnchorPath() {
  auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  EXPECT_TRUE(path->appendSegment(
      0.0, 2.0, "s4_anchor_test",
      [](const double w, FLAG_Race::ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.20 * w * w, 1.0 + 0.10 * w);
        state.dp_dw = Eigen::Vector3d(1.0, 0.40 * w, 0.10);
        state.d2p_dw2 = Eigen::Vector3d(0.0, 0.40, 0.0);
        state.vel = state.dp_dw;
        return true;
      }));
  auto g = std::make_shared<FLAG_Race::gvf>();
  g->setAuthoritativePhaseMode(true);
  g->setContinuousPhasePath(path);
  return g;
}

std::shared_ptr<const FLAG_Race::ContinuousPhasePath> makeH2ReplacementPath() {
  auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  EXPECT_TRUE(path->appendSegment(
      0.0, 2.0, "h2_replacement_path",
      [](const double w, FLAG_Race::ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(10.0 + w, -0.30 * w * w, 2.0);
        state.dp_dw = Eigen::Vector3d(1.0, -0.60 * w, 0.0);
        state.d2p_dw2 = Eigen::Vector3d(0.0, -0.60, 0.0);
        state.vel = state.dp_dw;
        return true;
      }));
  return path;
}

std::shared_ptr<const FLAG_Race::ContinuousPhasePath> makeH2OldSeamPath() {
  auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  EXPECT_TRUE(path->appendSegment(
      0.0, 4.0, "h2_old_seam",
      [](const double w, FLAG_Race::ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.10 * w * w, 1.0);
        state.dp_dw = Eigen::Vector3d(1.0, 0.20 * w, 0.0);
        state.d2p_dw2 = Eigen::Vector3d(0.0, 0.20, 0.0);
        state.vel = state.dp_dw;
        state.valid = true;
        return true;
      }));
  return path;
}

std::shared_ptr<const FLAG_Race::ContinuousPhasePath>
makeH2CopiedPrefixReplacement(
    const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>& source,
    const double seam_w = 1.20,
    const double end_w = 3.0) {
  if (!source || source->empty()) {
    return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  }
  FLAG_Race::ContinuousPhasePathState seam;
  if (!source->evaluate(seam_w, seam, false) || !seam.valid) {
    return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  }
  std::shared_ptr<FLAG_Race::ContinuousPhasePath> replacement(
      new FLAG_Race::ContinuousPhasePath());
  if (!replacement->appendSlice(*source, source->startW(), seam_w) ||
      !replacement->appendSegment(
          seam_w, end_w, "h2_copied_prefix_replacement_tail",
          [seam, seam_w, end_w](
              const double w, FLAG_Race::ContinuousPhasePathState& state) {
            const double dw = w - seam_w;
            state.p = seam.p + dw * seam.dp_dw +
                0.5 * dw * dw * seam.d2p_dw2 +
                Eigen::Vector3d(0.0, 0.05 * dw * dw * dw, 0.0);
            state.dp_dw = seam.dp_dw + dw * seam.d2p_dw2 +
                Eigen::Vector3d(0.0, 0.15 * dw * dw, 0.0);
            state.d2p_dw2 = seam.d2p_dw2 +
                Eigen::Vector3d(0.0, 0.30 * dw, 0.0);
            state.vel = state.dp_dw;
            state.valid = std::isfinite(w) && w >= seam_w && w <= end_w;
            return state.valid;
          })) {
    return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  }
  return replacement;
}

TEST(GvfSwitchPolicy, ForcesAcceptWhenAcceptedPathCannotSupportGovernorLookahead)
{
  EXPECT_TRUE(FLAG_Race::gvf_manager::shouldForceAcceptForGovernorPathShort(
      28.95, 29.40, 1.6, 0.2));
}

TEST(GvfSwitchPolicy, KeepsScoreBasedSwitchWhenAcceptedPathHasEnoughGovernorLookahead)
{
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldForceAcceptForGovernorPathShort(
      27.40, 29.40, 1.6, 0.2));
}

TEST(GvfInitialAcquisition, ConvertsGuidanceVelocityToNearbyPositionCommand)
{
  const Eigen::Vector3d delta =
      FLAG_Race::gvf_manager::boundedInitialAcquisitionDelta(
          Eigen::Vector3d(3.0, 4.0, 0.0), 2.0, 2.0, 1.6);
  EXPECT_NEAR(1.0, delta.norm(), 1e-9);
  EXPECT_NEAR(0.6, delta.x(), 1e-9);
  EXPECT_NEAR(0.8, delta.y(), 1e-9);
}

TEST(GvfInitialAcquisition, ScalesCommandInsteadOfRejectingLeadLimitViolation)
{
  const Eigen::Vector3d delta =
      FLAG_Race::gvf_manager::boundedInitialAcquisitionDelta(
          Eigen::Vector3d(4.0, 0.0, 0.0), 4.0, 1.0, 1.6);
  EXPECT_NEAR(1.6, delta.norm(), 1e-9);
  EXPECT_GT(delta.x(), 0.0);
}

TEST(GvfInitialAcquisition, RejectsInvalidCommandInputs)
{
  const Eigen::Vector3d delta =
      FLAG_Race::gvf_manager::boundedInitialAcquisitionDelta(
          Eigen::Vector3d::Ones(), 2.0, 0.0, 1.6);
  EXPECT_NEAR(0.0, delta.norm(), 1e-12);
}

TEST(GvfS4GovernorAnchor,
     ZeroDeltaIsBitwiseOriginalPathAndUsesContinuousPathDomain)
{
  FLAG_Race::gvf_manager manager;
  const std::shared_ptr<FLAG_Race::gvf> g = makeS4AnchorPath();
  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath> path =
      g->getContinuousPhasePath();
  constexpr double kQueryW = 0.75;
  const Eigen::Vector3d expected_point = g->evalPathByW(kQueryW);
  const Eigen::Vector3d expected_tangent = g->evalTangentByW(kQueryW);
  Eigen::Vector3d point;
  Eigen::Vector3d tangent;
  bool clamped = false;
  double start_w = 0.0;
  double end_w = 0.0;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::point(
      manager, path, kQueryW, 0.0, point, clamped, start_w, end_w));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::tangent(
      manager, path, kQueryW, 0.0, tangent));
  EXPECT_FALSE(clamped);
  EXPECT_DOUBLE_EQ(0.0, start_w);
  EXPECT_DOUBLE_EQ(2.0, end_w);
  EXPECT_EQ(0, std::memcmp(point.data(), expected_point.data(),
                           3U * sizeof(double)));
  EXPECT_EQ(0, std::memcmp(tangent.data(), expected_tangent.data(),
                           3U * sizeof(double)));

  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::point(
      manager, path, 4.0, 0.0, point, clamped, start_w, end_w));
  EXPECT_TRUE(clamped);
  EXPECT_DOUBLE_EQ(0.0, start_w);
  EXPECT_DOUBLE_EQ(2.0, end_w);
  const Eigen::Vector3d expected_end = g->evalPathByW(4.0);
  EXPECT_EQ(0, std::memcmp(point.data(), expected_end.data(),
                           3U * sizeof(double)));
}

TEST(GvfS4GovernorAnchor, NonzeroDeltaAnchorsAtActiveReference)
{
  FLAG_Race::gvf_manager manager;
  const std::shared_ptr<FLAG_Race::gvf> g = makeS4AnchorPath();
  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath> path =
      g->getContinuousPhasePath();
  constexpr double kQueryW = 0.75;
  constexpr double kDelta = 0.05;
  Eigen::Vector3d point;
  Eigen::Vector3d tangent;
  bool clamped = false;
  double start_w = 0.0;
  double end_w = 0.0;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::point(
      manager, path, kQueryW, kDelta, point, clamped, start_w, end_w));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::tangent(
      manager, path, kQueryW, kDelta, tangent));

  ASSERT_TRUE(path);
  FLAG_Race::ContinuousPhasePathState source;
  ASSERT_TRUE(path->evaluate(kQueryW, source, false));
  const phase_offset_core::PathDifferentialState active_path =
      FLAG_Race::ConvertContinuousPhasePathStateForActive(source, kQueryW);
  phase_offset_core::PhaseOffsetGeometryState expected;
  phase_offset_core::GeometryEvaluator evaluator;
  ASSERT_TRUE(evaluator.evaluate(active_path, Eigen::Vector3d::Zero(),
                                 kDelta, expected));
  EXPECT_EQ(0, std::memcmp(point.data(), expected.r.data(),
                           3U * sizeof(double)));
  EXPECT_EQ(0, std::memcmp(tangent.data(), expected.T.data(),
                           3U * sizeof(double)));
  EXPECT_GT((point - source.p).norm(), 0.0);
}

TEST(GvfS4GovernorAnchor,
     ZeroDeltaGovernorPositionCommandIsBitwiseBaselineAndActiveMovesAnchor)
{
  constexpr double kProgressW = 0.20;
  constexpr double kLookaheadW = 0.70;
  FLAG_Race::gvf_manager zero_manager;
  const std::shared_ptr<FLAG_Race::gvf> g = makeS4AnchorPath();
  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath> path =
      g->getContinuousPhasePath();
  const auto zero = FLAG_Race::GvfManagerS4AnchorTestAccess::
      runSingleAnchorCandidate(zero_manager, path, kProgressW, 0.0);
  ASSERT_TRUE(zero.command_valid);
  const Eigen::Vector3d baseline_command = g->evalPathByW(kLookaheadW);
  EXPECT_EQ(0, std::memcmp(zero.cmd_pos.data(), baseline_command.data(),
                           3U * sizeof(double)));

  FLAG_Race::gvf_manager active_manager;
  const auto active = FLAG_Race::GvfManagerS4AnchorTestAccess::
      runSingleAnchorCandidate(active_manager, path, kProgressW, 0.05);
  ASSERT_TRUE(active.command_valid);
  Eigen::Vector3d active_anchor;
  bool clamped = false;
  double start_w = 0.0;
  double end_w = 0.0;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::point(
      active_manager, path, kLookaheadW, 0.05, active_anchor, clamped,
      start_w, end_w));
  EXPECT_EQ(0, std::memcmp(active.cmd_pos.data(), active_anchor.data(),
                           3U * sizeof(double)));
  EXPECT_GT((active.cmd_pos - baseline_command).norm(), 0.0);
}

TEST(GvfH2CapturedPath,
     GuidanceAndGovernorKeepCapturedOwnerAfterMirrorPublicationChanges)
{
  constexpr double kPhaseW = 0.50;
  constexpr double kGovernorProgressW = 0.20;
  constexpr double kGovernorLookaheadW = 0.70;
  const std::shared_ptr<FLAG_Race::gvf> g = makeS4AnchorPath();
  g->gvf_.K1_ = 1.0;
  g->gvf_.K2_ = -1.0;
  g->gvf_.convergence_bandwidth_ = 0.5;
  g->progress_rho0_ = 0.5;
  g->progress_delta_ = 0.5;
  g->alpha_min_ = 0.05;

  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath> captured_path =
      g->getContinuousPhasePath();
  ASSERT_TRUE(captured_path);
  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath> replacement_path =
      makeH2ReplacementPath();
  ASSERT_TRUE(replacement_path);

  // This models a later publication after the command has already captured
  // its authority.  The explicit-owner calls below must remain on the old
  // immutable owner throughout this command.
  g->setContinuousPhasePath(replacement_path);

  FLAG_Race::ContinuousPhasePathState captured_state;
  FLAG_Race::ContinuousPhasePathState replacement_state;
  ASSERT_TRUE(captured_path->evaluate(kPhaseW, captured_state, false));
  ASSERT_TRUE(replacement_path->evaluate(kPhaseW, replacement_state, false));
  ASSERT_GT((captured_state.p - replacement_state.p).norm(), 1e-3);

  // Mirror the command callback's adapter input construction.  The published
  // slot has already changed above, but this command's semantic owner and
  // differential state must still be the captured immutable path.
  FLAG_Race::MatchedAdapterInput matched_input;
  matched_input.path = FLAG_Race::ConvertContinuousPhasePathStateForActive(
      captured_state, kPhaseW);
  matched_input.semantic_path_owner = captured_path;
  matched_input.semantic_path_start_w = captured_path->startW();
  matched_input.semantic_path_end_w = captured_path->endW();
  EXPECT_EQ(matched_input.semantic_path_owner.get(), captured_path.get());
  EXPECT_NE(matched_input.semantic_path_owner.get(), replacement_path.get());
  EXPECT_EQ(0, std::memcmp(matched_input.path.p.data(), captured_state.p.data(),
                           3U * sizeof(double)));
  EXPECT_EQ(0, std::memcmp(matched_input.path.p_w.data(),
                           captured_state.dp_dw.data(), 3U * sizeof(double)));
  EXPECT_EQ(0, std::memcmp(matched_input.path.p_ww.data(),
                           captured_state.d2p_dw2.data(), 3U * sizeof(double)));
  EXPECT_DOUBLE_EQ(matched_input.path.w, kPhaseW);
  EXPECT_GT((matched_input.path.p - replacement_state.p).norm(), 1e-3);
  EXPECT_GT((matched_input.path.p_w - replacement_state.dp_dw).norm(), 1e-3);

  const auto captured_guidance =
      g->calcLiftedGuidanceAtPhase(captured_state.p, kPhaseW, captured_path);
  ASSERT_TRUE(captured_guidance.valid);
  EXPECT_NEAR(0.0, (captured_guidance.ref_pt - captured_state.p).norm(), 1e-12);
  EXPECT_GT((captured_guidance.ref_pt - replacement_state.p).norm(), 1e-3);

  FLAG_Race::gvf_manager manager;
  const auto governor = FLAG_Race::GvfManagerS4AnchorTestAccess::
      runSingleAnchorCandidate(manager, captured_path, kGovernorProgressW, 0.0);
  ASSERT_TRUE(governor.command_valid);
  FLAG_Race::ContinuousPhasePathState expected_governor_state;
  ASSERT_TRUE(captured_path->evaluate(kGovernorLookaheadW,
                                      expected_governor_state, false));
  EXPECT_EQ(0, std::memcmp(governor.cmd_pos.data(),
                           expected_governor_state.p.data(),
                           3U * sizeof(double)));

  FLAG_Race::ContinuousPhasePathState replacement_governor_state;
  ASSERT_TRUE(replacement_path->evaluate(kGovernorLookaheadW,
                                         replacement_governor_state, false));
  EXPECT_GT((governor.cmd_pos - replacement_governor_state.p).norm(), 1e-3);
}

TEST(GvfH2FutureSeam,
     ProductionQuinticKeepsOldPrefixAndHasC2SeamWithDistinctInterior) {
  const auto old_path = makeH2OldSeamPath();
  ASSERT_TRUE(old_path);
  constexpr double kW0 = 0.40;
  constexpr double kFutureSeam = 1.20;
  constexpr double kJoin = 1.80;
  constexpr double kEnd = 3.00;
  FLAG_Race::ContinuousPhasePathState old_seam;
  ASSERT_TRUE(old_path->evaluate(kFutureSeam, old_seam, false));

  // A deliberately different mapped tail endpoint makes the Hermite
  // connector's strict interior distinguishable from the old owner.
  FLAG_Race::ContinuousPhasePathState tail_join;
  tail_join.p = Eigen::Vector3d(kJoin, 1.60, 1.0);
  tail_join.dp_dw = Eigen::Vector3d(1.0, -0.40, 0.0);
  tail_join.d2p_dw2 = Eigen::Vector3d(0.0, -0.20, 0.0);
  tail_join.vel = tail_join.dp_dw;
  tail_join.valid = true;
  const auto connector = FLAG_Race::ContinuousPhasePath::makeQuinticHermite(
      kFutureSeam, kJoin, old_seam, tail_join);
  ASSERT_TRUE(connector);
  auto new_path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  ASSERT_TRUE(new_path->appendSlice(*old_path, kW0, kFutureSeam));
  ASSERT_TRUE(new_path->appendSegment(kFutureSeam, kJoin, "c2_quintic",
                                      connector));
  ASSERT_TRUE(new_path->appendSegment(
      kJoin, kEnd, "tail",
      [tail_join](const double w, FLAG_Race::ContinuousPhasePathState& state) {
        const double dw = w - kJoin;
        state = tail_join;
        state.p = tail_join.p + dw * tail_join.dp_dw +
            0.5 * dw * dw * tail_join.d2p_dw2;
        state.dp_dw = tail_join.dp_dw + dw * tail_join.d2p_dw2;
        state.valid = true;
        return true;
      }));

  FLAG_Race::ContinuousPhasePathState new_seam;
  ASSERT_TRUE(new_path->evaluate(kFutureSeam, new_seam, false));
  EXPECT_NEAR(0.0, (new_seam.p - old_seam.p).norm(), 1e-12);
  EXPECT_NEAR(0.0, (new_seam.dp_dw - old_seam.dp_dw).norm(), 1e-12);
  EXPECT_NEAR(0.0, (new_seam.d2p_dw2 - old_seam.d2p_dw2).norm(), 1e-12);

  FLAG_Race::ContinuousPhasePathState old_prefix;
  FLAG_Race::ContinuousPhasePathState new_prefix;
  ASSERT_TRUE(old_path->evaluate(0.80, old_prefix, false));
  ASSERT_TRUE(new_path->evaluate(0.80, new_prefix, false));
  EXPECT_EQ(0, std::memcmp(old_prefix.p.data(), new_prefix.p.data(),
                           3U * sizeof(double)));
  EXPECT_EQ(0, std::memcmp(old_prefix.dp_dw.data(), new_prefix.dp_dw.data(),
                           3U * sizeof(double)));

  FLAG_Race::ContinuousPhasePathState old_interior;
  FLAG_Race::ContinuousPhasePathState new_interior;
  ASSERT_TRUE(old_path->evaluate(1.50, old_interior, false));
  ASSERT_TRUE(new_path->evaluate(1.50, new_interior, false));
  EXPECT_GT((new_interior.p - old_interior.p).norm(), 1e-4);
}

TEST(GvfH2FutureSeam,
     SelectsExactRepresentedMapInteriorBreakpointAndNeverPathEndpoint) {
  FLAG_Race::ContinuousPhasePath::Evaluator represented(
      [](const double w, FLAG_Race::ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.10 * w * w, 1.0);
        state.dp_dw = Eigen::Vector3d(1.0, 0.20 * w, 0.0);
        state.d2p_dw2 = Eigen::Vector3d(0.0, 0.20, 0.0);
        state.vel = state.dp_dw;
        state.valid = std::isfinite(w) && w >= 0.0 && w <= 4.0;
        return state.valid;
      },
      FLAG_Race::ContinuousPhasePath::CellBoundEvaluator(),
      [](const double, const double,
         phase_offset_core::CertifiedPathCellV2&) { return false; },
      [](std::vector<double>& breakpoints) {
        breakpoints = {0.0, 1.25, 2.50, 4.0};
        return true;
      });
  auto source = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  ASSERT_TRUE(source->appendSegment(
      0.0, 4.0, "represented_map", represented));

  double seam_w = -1.0;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
      plannerOnlyFutureSeam(source, 0.40, 0.60, seam_w));
  EXPECT_DOUBLE_EQ(seam_w, 1.25);

  seam_w = -1.0;
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
      plannerOnlyFutureSeam(source, 3.30, 0.40, seam_w));
  EXPECT_DOUBLE_EQ(seam_w, 0.0);
}

TEST(GvfManagerGDes,
     ProductionValueSeamProvidesUpdatesAndClearWithoutPrivateDefault) {
  FLAG_Race::gvf_manager manager;
  Eigen::Vector3d captured = Eigen::Vector3d::Constant(9.0);
  EXPECT_FALSE(manager.capturePhaseOffsetGDes(captured));
  EXPECT_TRUE(captured.isZero());

  const Eigen::Vector3d supplied(0.21, -0.34, 0.56);
  ASSERT_TRUE(manager.setPhaseOffsetGDes(supplied));
  ASSERT_TRUE(manager.capturePhaseOffsetGDes(captured));
  EXPECT_TRUE(captured.isApprox(supplied, 0.0));

  const Eigen::Vector3d updated(-0.11, 0.22, -0.33);
  ASSERT_TRUE(manager.setPhaseOffsetGDes(updated));
  ASSERT_TRUE(manager.capturePhaseOffsetGDes(captured));
  EXPECT_TRUE(captured.isApprox(updated, 0.0));

  EXPECT_FALSE(manager.setPhaseOffsetGDes(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)));
  EXPECT_FALSE(manager.capturePhaseOffsetGDes(captured));
  EXPECT_TRUE(captured.isZero());

  ASSERT_TRUE(manager.setPhaseOffsetGDes(supplied));
  manager.clearPhaseOffsetGDes();
  EXPECT_FALSE(manager.capturePhaseOffsetGDes(captured));
  EXPECT_TRUE(captured.isZero());
}

TEST(GvfPlannerOnlyV2,
     NeutralContinuationInstallsWithoutBindingAndBindingBlocksReplacement) {
  ros::Time::init();
  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath> original =
      makeH2OldSeamPath();
  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath> replacement =
      makeH2ReplacementPath();
  ASSERT_TRUE(original);
  ASSERT_TRUE(replacement);

  FLAG_Race::gvf_manager neutral_manager;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::installNeutralAdapter(
      neutral_manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::initializePlannerFrontend(
      neutral_manager, original));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      neutral_manager, 0.40, true, false);
  const auto neutral_phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(neutral_manager);
  const auto neutral_runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(neutral_manager);
  ASSERT_TRUE(neutral_runtime_before.present);
  ASSERT_DOUBLE_EQ(neutral_runtime_before.retained_delta, 0.0);
  ASSERT_DOUBLE_EQ(neutral_runtime_before.previous_port.u_w, 0.0);
  ASSERT_DOUBLE_EQ(neutral_runtime_before.previous_port.u_delta, 0.0);
  ASSERT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::executionBindingV2(
      neutral_manager));
  ASSERT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
      neutral_manager));

  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      neutral_manager, replacement));
  const auto neutral_phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(neutral_manager);
  const auto neutral_runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(neutral_manager);
  EXPECT_DOUBLE_EQ(neutral_phase_after.w, neutral_phase_before.w);
  EXPECT_EQ(neutral_phase_after.initialized, neutral_phase_before.initialized);
  EXPECT_EQ(neutral_phase_after.closed_acquired,
            neutral_phase_before.closed_acquired);
  EXPECT_EQ(neutral_phase_after.generation, neutral_phase_before.generation);
  EXPECT_DOUBLE_EQ(neutral_runtime_after.retained_delta, 0.0);
  EXPECT_DOUBLE_EQ(neutral_runtime_after.previous_port.u_w, 0.0);
  EXPECT_DOUBLE_EQ(neutral_runtime_after.previous_port.u_delta, 0.0);
  ASSERT_FALSE(neutral_manager.swarmParticlesManager.empty());
  ASSERT_TRUE(neutral_manager.swarmParticlesManager.front().gvf_);
  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath> installed =
      neutral_manager.swarmParticlesManager.front().gvf_
          ->getContinuousPhasePath();
  ASSERT_TRUE(installed);
  EXPECT_NE(installed, original);
  EXPECT_NE(installed->pathRevision(), 0U);
  FLAG_Race::ContinuousPhasePathState installed_state;
  FLAG_Race::ContinuousPhasePathState replacement_state;
  ASSERT_TRUE(installed->evaluate(1.0, installed_state, false));
  ASSERT_TRUE(replacement->evaluate(1.0, replacement_state, false));
  EXPECT_TRUE(installed_state.p.isApprox(replacement_state.p, 0.0));
  EXPECT_TRUE(installed_state.dp_dw.isApprox(replacement_state.dp_dw, 0.0));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::executionBindingV2(
      neutral_manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
      neutral_manager));

  FLAG_Race::gvf_manager bound_manager;
  FLAG_Race::GvfManagerS4AnchorTestAccess::V2ManagerTransactionFixture fixture;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  prepareV2ManagerTransaction(bound_manager, fixture));
  FLAG_Race::GvfManagerS4AnchorTestAccess::discardPendingV2Command(
      bound_manager);
  FLAG_Race::GvfManagerS4AnchorTestAccess::clearPendingPathReferenceV2(
      bound_manager);
  const auto binding_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionBindingV2(
          bound_manager);
  ASSERT_TRUE(binding_before);
  const auto bound_phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(bound_manager);
  const auto bound_runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(bound_manager);
  ASSERT_TRUE(bound_runtime_before.present);
  ASSERT_FALSE(bound_manager.swarmParticlesManager.empty());
  ASSERT_TRUE(bound_manager.swarmParticlesManager.front().gvf_);
  const auto bound_owner_before =
      bound_manager.swarmParticlesManager.front().gvf_
          ->getContinuousPhasePath();

  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      bound_manager, replacement));
  const auto bound_phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(bound_manager);
  const auto bound_runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(bound_manager);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::executionBindingV2(
                bound_manager),
            binding_before);
  EXPECT_EQ(bound_manager.swarmParticlesManager.front().gvf_
                ->getContinuousPhasePath(),
            bound_owner_before);
  EXPECT_DOUBLE_EQ(bound_phase_after.w, bound_phase_before.w);
  EXPECT_EQ(bound_phase_after.initialized, bound_phase_before.initialized);
  EXPECT_EQ(bound_phase_after.closed_acquired,
            bound_phase_before.closed_acquired);
  EXPECT_EQ(bound_phase_after.generation, bound_phase_before.generation);
  EXPECT_DOUBLE_EQ(bound_runtime_after.retained_delta,
                   bound_runtime_before.retained_delta);
  EXPECT_DOUBLE_EQ(bound_runtime_after.previous_port.u_w,
                   bound_runtime_before.previous_port.u_w);
  EXPECT_DOUBLE_EQ(bound_runtime_after.previous_port.u_delta,
                   bound_runtime_before.previous_port.u_delta);
}

TEST(GvfPlannerOnlyV2,
     MovingPhaseInsideCopiedPrefixInstallsWithoutRollbackOrIndexJump) {
  ros::Time::init();
  const auto source = makeH2OldSeamPath();
  const auto replacement = makeH2CopiedPrefixReplacement(source);
  ASSERT_TRUE(source);
  ASSERT_TRUE(replacement);

  FLAG_Race::gvf_manager manager;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::installNeutralAdapter(
      manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::initializePlannerFrontend(
      manager, source));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.40, true, false);
  const auto captured =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const std::uint64_t execution_generation =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGenerationV2(manager);
  ASSERT_NE(execution_generation, 0U);

  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      manager, replacement, source, 1.20, execution_generation, 0.65));
  const auto installed_phase =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  EXPECT_DOUBLE_EQ(installed_phase.w, 0.65);
  EXPECT_GT(installed_phase.w, captured.w);
  EXPECT_EQ(installed_phase.generation, captured.generation + 1U);
  EXPECT_TRUE(installed_phase.initialized);
  EXPECT_FALSE(installed_phase.closed_acquired);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::currentTrajectoryIndex(
                manager),
            1);

  const auto installed = manager.swarmParticlesManager.front().gvf_
                             ->getContinuousPhasePath();
  ASSERT_TRUE(installed);
  EXPECT_NE(installed, source);
  FLAG_Race::ContinuousPhasePathState source_live;
  FLAG_Race::ContinuousPhasePathState installed_live;
  ASSERT_TRUE(source->evaluate(installed_phase.w, source_live, false));
  ASSERT_TRUE(installed->evaluate(installed_phase.w, installed_live, false));
  EXPECT_EQ(0, std::memcmp(source_live.p.data(), installed_live.p.data(),
                          3U * sizeof(double)));
  EXPECT_EQ(0, std::memcmp(source_live.dp_dw.data(),
                           installed_live.dp_dw.data(),
                           3U * sizeof(double)));
  EXPECT_EQ(0, std::memcmp(source_live.d2p_dw2.data(),
                           installed_live.d2p_dw2.data(),
                           3U * sizeof(double)));
}

TEST(GvfPlannerOnlyV2,
     CopiedPrefixExpiryTaskResetAndExactCasStillRejectReplacement) {
  ros::Time::init();
  const auto source = makeH2OldSeamPath();
  const auto replacement = makeH2CopiedPrefixReplacement(source);
  ASSERT_TRUE(source);
  ASSERT_TRUE(replacement);

  const auto initialize = [&source](FLAG_Race::gvf_manager& manager) {
    return FLAG_Race::GvfManagerS4AnchorTestAccess::installNeutralAdapter(
               manager) &&
        FLAG_Race::GvfManagerS4AnchorTestAccess::initializePlannerFrontend(
               manager, source);
  };

  FLAG_Race::gvf_manager expired;
  ASSERT_TRUE(initialize(expired));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      expired, 0.40, true, false);
  const std::uint64_t expired_generation =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGenerationV2(expired);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      expired, replacement, source, 1.20, expired_generation, 1.21));
  EXPECT_EQ(expired.swarmParticlesManager.front().gvf_
                ->getContinuousPhasePath(),
            source);
  EXPECT_DOUBLE_EQ(
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(expired).w, 1.21);

  FLAG_Race::gvf_manager reset;
  ASSERT_TRUE(initialize(reset));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      reset, 0.40, true, false);
  const std::uint64_t stale_task_generation =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGenerationV2(reset);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      reset, replacement, source, 1.20, stale_task_generation, 0.65, true));
  EXPECT_EQ(reset.swarmParticlesManager.front().gvf_
                ->getContinuousPhasePath(),
            source);

  FLAG_Race::gvf_manager exact;
  ASSERT_TRUE(initialize(exact));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      exact, 0.40, true, false);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      exact, replacement,
      std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(), 0.40, 0U,
      0.41));
  EXPECT_EQ(exact.swarmParticlesManager.front().gvf_
                ->getContinuousPhasePath(),
            source);
}

TEST(GvfAuthoritativePhaseCommit, CommitsValidGovernorCandidate)
{
  const auto decision = FLAG_Race::gvf_manager::decideAuthoritativePhaseCommit(
      true, true, false, false, 12.0, 12.35, true, false);

  EXPECT_TRUE(decision.commit);
  EXPECT_FALSE(decision.acquire_closed_phase);
  EXPECT_DOUBLE_EQ(12.35, decision.phase_after);
  EXPECT_NEAR(0.35, decision.applied_delta_w, 1e-12);
}

TEST(GvfAuthoritativePhaseCommit, HoldsForEveryGovernorInvalidReason)
{
  const std::vector<const char*> hold_reasons{
      "missing_gvf", "guidance_invalid", "initial_acquisition_zero_command",
      "path_invalid", "tangent_invalid", "all_candidates_path_end_clamped",
      "no_valid_candidate"};
  for (const char* reason : hold_reasons) {
    SCOPED_TRACE(reason);
    const auto decision = FLAG_Race::gvf_manager::decideAuthoritativePhaseCommit(
        true, false, false, false, 7.0, 7.25, false, true);
    EXPECT_FALSE(decision.commit);
    EXPECT_FALSE(decision.acquire_closed_phase);
    EXPECT_DOUBLE_EQ(7.0, decision.phase_after);
    EXPECT_DOUBLE_EQ(0.0, decision.applied_delta_w);
  }
}

TEST(GvfGovernorPublication, ActiveFailureProducesPositionHoldNotCenterline)
{
  FLAG_Race::gvf_manager manager;
  manager.cmd_governor_initialized_ = true;
  manager.cmd_governor_normal_state_ = Eigen::Vector3d(0.3, -0.2, 0.1);
  manager.cmd_governor_last_l_ = 0.25;
  const Eigen::Vector3d position(4.0, -1.0, 2.0);
  const auto result = FLAG_Race::GvfManagerS4AnchorTestAccess::invalidHold(
      manager, position, "active_nonzero_recovery_denied");
  EXPECT_FALSE(result.command_valid);
  EXPECT_FALSE(result.selected_valid_for_state);
  EXPECT_TRUE(result.reset_state_after_publish);
  EXPECT_EQ(result.final_cmd_source, "GOVERNOR_INVALID_HOLD");
  EXPECT_EQ(result.fallback_reason, "active_nonzero_recovery_denied");
  EXPECT_EQ(0, std::memcmp(result.cmd_pos.data(), position.data(),
                           3U * sizeof(double)));
  // Constructing the invalid result is side-effect free; reset/last-yaw
  // mutation belongs after successful PositionCommand publication.
  EXPECT_TRUE(manager.cmd_governor_initialized_);
  EXPECT_NEAR(manager.cmd_governor_normal_state_.norm(),
              Eigen::Vector3d(0.3, -0.2, 0.1).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(manager.cmd_governor_last_l_, 0.25);
}

TEST(GvfGovernorPublication, FailedPublicationDoesNotAdvanceYawCandidate)
{
  FLAG_Race::gvf_manager manager;
  manager.last_yaw = 0.35;
  manager.pending_last_yaw_valid_ = false;
  const bool published = FLAG_Race::GvfManagerS4AnchorTestAccess::
      publishGovernorCandidate(manager, Eigen::Vector3d(1.0, 2.0, 3.0),
                                Eigen::Vector3d(0.0, 1.0, 0.0));
  EXPECT_FALSE(published);
  EXPECT_DOUBLE_EQ(manager.last_yaw, 0.35);
  EXPECT_FALSE(manager.pending_last_yaw_valid_);
}

TEST(GvfAuthoritativePhaseCommit, GoalPositionOverrideFreezesCandidate)
{
  const auto decision = FLAG_Race::gvf_manager::decideAuthoritativePhaseCommit(
      true, true, true, false, 4.0, 4.2, false, true);

  EXPECT_FALSE(decision.commit);
  EXPECT_FALSE(decision.acquire_closed_phase);
  EXPECT_DOUBLE_EQ(4.0, decision.phase_after);
  EXPECT_DOUBLE_EQ(0.0, decision.applied_delta_w);
}

TEST(GvfAuthoritativePhaseCommit, ValidInitialAcquisitionFreezesPhase)
{
  const auto decision = FLAG_Race::gvf_manager::decideAuthoritativePhaseCommit(
      true, true, false, true, 3.0, 3.1, false, false);

  EXPECT_FALSE(decision.commit);
  EXPECT_FALSE(decision.acquire_closed_phase);
  EXPECT_DOUBLE_EQ(3.0, decision.phase_after);
  EXPECT_DOUBLE_EQ(0.0, decision.applied_delta_w);
}

TEST(GvfAuthoritativePhaseCommit, WillAcquireUsesGovernorWithoutOneCycleDelay)
{
  EXPECT_TRUE(FLAG_Race::gvf_manager::shouldRunInitialClosedPhaseAcquisition(
      true, false, false));
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldRunInitialClosedPhaseAcquisition(
      true, false, true));
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldRunInitialClosedPhaseAcquisition(
      true, true, false));
  EXPECT_FALSE(FLAG_Race::gvf_manager::shouldRunInitialClosedPhaseAcquisition(
      false, false, false));
}

TEST(GvfAuthoritativePhaseCommit, ZeroInitialAcquisitionFreezesPhase)
{
  const auto decision = FLAG_Race::gvf_manager::decideAuthoritativePhaseCommit(
      true, false, false, true, 3.0, 3.1, false, false);

  EXPECT_FALSE(decision.commit);
  EXPECT_DOUBLE_EQ(3.0, decision.phase_after);
  EXPECT_DOUBLE_EQ(0.0, decision.applied_delta_w);
}

TEST(GvfAuthoritativePhaseCommit, ClampsPointEndpointCandidateExactly)
{
  EXPECT_DOUBLE_EQ(20.0,
      FLAG_Race::gvf_manager::clampPointPhaseCandidate(99.0, 10.0, 20.0));
  EXPECT_DOUBLE_EQ(10.0,
      FLAG_Race::gvf_manager::clampPointPhaseCandidate(-4.0, 10.0, 20.0));
  EXPECT_DOUBLE_EQ(14.5,
      FLAG_Race::gvf_manager::clampPointPhaseCandidate(14.5, 10.0, 20.0));
}

TEST(GvfAuthoritativePhaseCommit, ClosedAcquisitionChangesOnlyWithFinalCommit)
{
  const auto hold = FLAG_Race::gvf_manager::decideAuthoritativePhaseCommit(
      true, false, false, false, 8.0, 8.2, false, true);
  EXPECT_FALSE(hold.commit);
  EXPECT_FALSE(hold.acquire_closed_phase);

  const auto overridden = FLAG_Race::gvf_manager::decideAuthoritativePhaseCommit(
      true, true, true, false, 8.0, 8.2, false, true);
  EXPECT_FALSE(overridden.commit);
  EXPECT_FALSE(overridden.acquire_closed_phase);

  const auto committed = FLAG_Race::gvf_manager::decideAuthoritativePhaseCommit(
      true, true, false, false, 8.0, 8.2, false, true);
  EXPECT_TRUE(committed.commit);
  EXPECT_TRUE(committed.acquire_closed_phase);
}

TEST(GvfAuthoritativePhaseSnapshot,
     CapturesCoherentTupleAndRejectsStaleCommandCommitAfterReset)
{
  FLAG_Race::gvf_manager manager;
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 4.0, true, false);
  const auto command = FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(
      manager);
  ASSERT_DOUBLE_EQ(command.w, 4.0);
  ASSERT_TRUE(command.initialized);
  ASSERT_FALSE(command.closed_acquired);

  // This models an FSM reset/initialization transition completing while the
  // command callback is between its initial capture and final phase commit.
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.0, false, false);
  FLAG_Race::GvfManagerS4AnchorTestAccess::PhaseSnapshotProbe committed;
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::commitPhase(
      manager, command, 4.1, true, committed));
  const auto after = FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(
      manager);
  EXPECT_DOUBLE_EQ(after.w, 0.0);
  EXPECT_FALSE(after.initialized);
  EXPECT_FALSE(after.closed_acquired);
  EXPECT_GT(after.generation, command.generation);
}

TEST(GvfAuthoritativePhaseSnapshot,
     ConcurrentCaptureNeverObservesMixedPhaseTuple)
{
  FLAG_Race::gvf_manager manager;
  // Make the pre-start tuple satisfy the reader invariant as well.  Without
  // this, a scheduled reader can observe the default {0,false,false} before
  // the writer's first publication and report a spurious mixed tuple.
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, -1.0, false, false);
  std::atomic<bool> start(false);
  std::atomic<bool> bad_snapshot(false);
  std::thread writer([&]() {
    while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
    for (int iteration = 0; iteration < 2000; ++iteration) {
      const bool initialized = (iteration % 2) == 0;
      const bool acquired = initialized && (iteration % 4) == 0;
      const double w = initialized ? static_cast<double>(iteration) : -1.0;
      FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
          manager, w, initialized, acquired);
    }
  });
  std::thread reader([&]() {
    while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
    for (int iteration = 0; iteration < 2000; ++iteration) {
      const auto snapshot =
          FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
      if ((!snapshot.initialized &&
           (snapshot.closed_acquired || snapshot.w != -1.0)) ||
          (snapshot.closed_acquired && !snapshot.initialized)) {
        bad_snapshot.store(true, std::memory_order_release);
      }
    }
  });
  start.store(true, std::memory_order_release);
  writer.join();
  reader.join();
  EXPECT_FALSE(bad_snapshot.load(std::memory_order_acquire));
}

TEST(PathReferenceHandoffV2Test,
     ExactPhaseAfterAndCopiedPrefixIdentityAreImmutableRequestEvidence) {
  auto mutable_successor =
      std::make_shared<FLAG_Race::ContinuousPhasePath>();
  ASSERT_TRUE(mutable_successor->appendSegment(
      0.25, 2.0, "s6a_successor",
      [](const double w, FLAG_Race::ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.vel = state.dp_dw;
        state.valid = std::isfinite(w);
        return state.valid;
      }));
  mutable_successor->setPathRevision(12U);
  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath> successor =
      mutable_successor;

  phase_offset_navigation::TubePathKey source_key;
  source_key.execution_generation = 4U;
  source_key.path_instance_id = 10U;
  source_key.path_revision = 11U;
  source_key.frame_revision = 11U;
  source_key.frame_convention_id = 91U;
  source_key.frame_convention =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  source_key.phase_orientation = -1;
  source_key.domain_start = 0.0;
  source_key.domain_end = 2.0;
  phase_offset_navigation::TubePathKey successor_key = source_key;
  successor_key.path_instance_id = 12U;
  successor_key.path_revision = 12U;
  successor_key.frame_revision = 12U;
  successor_key.domain_start = 0.25;

  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> request(
      new phase_offset_navigation::TubeBuildInputV2());
  request->request_id = 19U;
  request->path_key = successor_key;
  request->configuration_key.configuration_id = 21U;
  request->configuration_key.epsilon = 0.4;
  request->configuration_key.nominal_half_width = 1.0;
  request->configuration_key.ray_step = 0.05;
  request->configuration_key.snapshot_resolution = 0.1;
  request->configuration_key.minimum_reference_speed = 1e-8;
  request->map_capture_key.map_instance_id = 31U;
  request->map_capture_key.state_id = 32U;
  request->map_capture_key.accepted_sequence = 32U;
  request->map_capture_key.configuration_generation = 33U;
  request->map_capture_key.configuration_id = 34U;
  request->map_capture_key.frame_provenance_id = 35U;
  request->map_capture_key.frame_provenance = "world";
  request->map_capture_key.support_provenance_id = 36U;
  request->map_capture_key.accepted_time_ticks = 37U;
  request->map_capture_key.support_expiry_ticks = 40U;
  request->map_capture_key.halo_reconciled = true;
  request->map_capture_key.grid_min_index_x = 0;
  request->map_capture_key.grid_min_index_y = 0;
  request->map_capture_key.grid_min_index_z = 0;
  request->map_capture_key.grid_max_index_x = 9;
  request->map_capture_key.grid_max_index_y = 9;
  request->map_capture_key.grid_max_index_z = 9;
  request->map_capture_key.grid_voxel_resolution =
      Eigen::Vector3d::Constant(0.1);
  request->map_capture_key.complete_support = true;
  request->requested_start = 0.25;
  request->requested_end = 1.5;
  request->anchor_w = 0.25;
  request->path_cell_query = [](
      double, double, phase_offset_core::CertifiedPathCellV2&) {
        return false;
      };
  request->producer_breakpoints = {0.25, 0.75, 2.0};
  request->path_owner = std::static_pointer_cast<const void>(successor);
  request->query_owner = std::static_pointer_cast<const void>(
      std::make_shared<int>(1));
  request->capture_owner = std::static_pointer_cast<const void>(
      std::make_shared<int>(2));
  request->applicability_assumptions = "immutable S6-A transport fixture";
  request->applicability_deadline_ticks = 40U;
  request->free_ball_query = [](
      const Eigen::Vector3d&, double) {
        return phase_offset_navigation::TubeFreeBallQueryResult();
      };
  ASSERT_TRUE(request->complete());

  std::shared_ptr<FLAG_Race::PathReferenceFrontendMirrorV2> mirror(
      new FLAG_Race::PathReferenceFrontendMirrorV2());
  mirror->traj.resize(2, 3);
  mirror->traj << 0.25, 0.0, 1.0, 2.0, 0.0, 1.0;
  mirror->vel.resize(2, 3);
  mirror->vel << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0;
  mirror->time.resize(2);
  mirror->time << 0.0, 1.0;
  mirror->w = {0.25, 2.0};
  mirror->anchor_idx = 0;

  FLAG_Race::PathReferenceHandoffV2 handoff;
  handoff.expected_execution_generation = 4U;
  handoff.source_path_key = source_key;
  handoff.successor_path_owner = successor;
  handoff.successor_path_key = successor_key;
  handoff.successor_phase_after_w = 0.25;
  handoff.copied_prefix_start_w = 0.25;
  handoff.copied_prefix_end_w = 0.75;
  handoff.successor_request = request;
  handoff.frontend_mirror = mirror;
  EXPECT_TRUE(handoff.requestComplete());
  EXPECT_DOUBLE_EQ(handoff.successor_phase_after_w,
                   handoff.copied_prefix_start_w);
  EXPECT_EQ(handoff.successor_request->path_owner.get(), successor.get());
  EXPECT_NE(handoff.source_path_key, handoff.successor_path_key);
  EXPECT_EQ(handoff.successor_path_key.phase_orientation, -1);
  EXPECT_EQ(handoff.successor_path_key.frame_convention_id, 91U);

  handoff.successor_path_key.phase_orientation = 1;
  EXPECT_FALSE(handoff.requestComplete());
  handoff.successor_path_key.phase_orientation = -1;
  request->anchor_w = std::nextafter(
      0.25, std::numeric_limits<double>::infinity());
  EXPECT_FALSE(handoff.requestComplete());
  request->anchor_w = 0.25;
  request->requested_end = 0.5;
  EXPECT_FALSE(handoff.requestComplete());
  request->requested_end = 1.5;

  handoff.successor_phase_after_w =
      std::nextafter(0.25, std::numeric_limits<double>::infinity());
  EXPECT_FALSE(handoff.requestComplete());
}

TEST(GvfStage6V2Handoff, PublishFirstBindingPrecedesDeferredMirror) {
  ros::Time::init();
  FLAG_Race::gvf_manager manager;
  FLAG_Race::GvfManagerS4AnchorTestAccess::V2ManagerTransactionFixture fixture;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  prepareV2ManagerTransaction(manager, fixture));
  ASSERT_TRUE(fixture.source_binding);
  ASSERT_TRUE(fixture.proposed_binding);
  ASSERT_TRUE(fixture.request_handoff);
  ASSERT_TRUE(fixture.candidate);

  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  const auto request_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
          manager);
  ASSERT_TRUE(runtime_before.present);
  ASSERT_NE(runtime_before.retained_delta, 0.0);
  ASSERT_DOUBLE_EQ(runtime_before.previous_port.u_delta, 0.05);
  ASSERT_DOUBLE_EQ(runtime_before.retained_delta,
                   0.10 * runtime_before.previous_port.u_delta);
  ASSERT_EQ(request_before, fixture.request_handoff);
  ASSERT_FALSE(request_before->successor_profile);
  ASSERT_FALSE(manager.swarmParticlesManager.empty());
  ASSERT_TRUE(manager.swarmParticlesManager.front().gvf_);
  const Eigen::MatrixXd display_before =
      manager.swarmParticlesManager.front().last_traj;
  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>
      display_owner_before = manager.swarmParticlesManager.front().gvf_
                                 ->getContinuousPhasePath();
  ASSERT_EQ(display_owner_before, fixture.source_owner);

  const auto failed = FLAG_Race::GvfManagerS4AnchorTestAccess::
      publishV2ManagerTransaction(manager, fixture, false);
  EXPECT_TRUE(failed.attempted);
  EXPECT_FALSE(failed.published);
  EXPECT_TRUE(failed.phase_token_prepared);
  EXPECT_EQ(failed.local_publish_count, 1);
  EXPECT_EQ(failed.post_publish_count, 0);
  EXPECT_TRUE(failed.local_saw_source_state);
  EXPECT_FALSE(failed.post_saw_atomic_adapter_commit);
  EXPECT_FALSE(failed.post_saw_old_phase);
  const auto phase_after_failure =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_after_failure =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  EXPECT_DOUBLE_EQ(phase_after_failure.w, phase_before.w);
  EXPECT_EQ(phase_after_failure.initialized, phase_before.initialized);
  EXPECT_EQ(phase_after_failure.closed_acquired,
            phase_before.closed_acquired);
  EXPECT_EQ(phase_after_failure.generation, phase_before.generation);
  EXPECT_DOUBLE_EQ(runtime_after_failure.retained_delta,
                   runtime_before.retained_delta);
  EXPECT_DOUBLE_EQ(runtime_after_failure.previous_port.u_w,
                   runtime_before.previous_port.u_w);
  EXPECT_DOUBLE_EQ(runtime_after_failure.previous_port.u_delta,
                   runtime_before.previous_port.u_delta);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::executionBindingV2(
                manager),
            fixture.source_binding);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
                manager),
            request_before);
  EXPECT_EQ(manager.swarmParticlesManager.front().gvf_
                ->getContinuousPhasePath(),
            fixture.source_owner);
  EXPECT_TRUE((manager.swarmParticlesManager.front().last_traj.array() ==
               display_before.array()).all());

  const auto succeeded = FLAG_Race::GvfManagerS4AnchorTestAccess::
      publishV2ManagerTransaction(manager, fixture, true);
  EXPECT_TRUE(succeeded.attempted);
  EXPECT_TRUE(succeeded.published);
  EXPECT_TRUE(succeeded.phase_token_prepared);
  EXPECT_EQ(succeeded.local_publish_count, 1);
  EXPECT_EQ(succeeded.post_publish_count, 1);
  EXPECT_TRUE(succeeded.local_saw_source_state);
  EXPECT_TRUE(succeeded.post_saw_atomic_adapter_commit);
  EXPECT_TRUE(succeeded.post_saw_old_phase);
  const auto phase_after_success =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_after_success =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  EXPECT_DOUBLE_EQ(phase_after_success.w, fixture.phase_after);
  EXPECT_TRUE(phase_after_success.initialized);
  EXPECT_EQ(phase_after_success.generation, phase_before.generation + 1U);
  EXPECT_DOUBLE_EQ(runtime_after_success.retained_delta,
                   fixture.candidate->prepared_step.successor.delta);
  EXPECT_DOUBLE_EQ(runtime_after_success.previous_port.u_w,
                   fixture.candidate->prepared_step.successor.previous_u.u_w);
  EXPECT_DOUBLE_EQ(
      runtime_after_success.previous_port.u_delta,
      fixture.candidate->prepared_step.successor.previous_u.u_delta);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::executionBindingV2(
                manager),
            fixture.proposed_binding);
  const auto enriched =
      FLAG_Race::GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
          manager);
  ASSERT_TRUE(enriched);
  EXPECT_TRUE(enriched->committedComplete());
  EXPECT_EQ(enriched->successor_profile, fixture.proposed_binding->profile);

  // Command ownership changes at publication; the FSM/display mirror does
  // not.  The next command must nevertheless capture the committed owner.
  EXPECT_EQ(manager.swarmParticlesManager.front().gvf_
                ->getContinuousPhasePath(),
            fixture.source_owner);
  EXPECT_TRUE((manager.swarmParticlesManager.front().last_traj.array() ==
               display_before.array()).all());
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::commandPathForV2(
                manager, fixture.proposed_binding),
            fixture.successor_owner);

  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  consumeCommittedPathReferenceV2(manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
      manager));
  EXPECT_EQ(manager.swarmParticlesManager.front().gvf_
                ->getContinuousPhasePath(),
            fixture.successor_owner);
  ASSERT_EQ(manager.swarmParticlesManager.front().last_traj.rows(),
            enriched->frontend_mirror->traj.rows());
  EXPECT_TRUE(manager.swarmParticlesManager.front().last_traj.isApprox(
      enriched->frontend_mirror->traj, 0.0));
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::commandPathForV2(
                manager, fixture.proposed_binding),
            fixture.successor_owner);
}

TEST(GvfStage6V2Handoff,
     PrematureMirrorAndTaskResetCannotRebindOrReplay) {
  ros::Time::init();

  FLAG_Race::gvf_manager stale_manager;
  FLAG_Race::GvfManagerS4AnchorTestAccess::V2ManagerTransactionFixture
      stale_fixture;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  prepareV2ManagerTransaction(stale_manager, stale_fixture));
  const auto stale_phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(stale_manager);
  const auto stale_runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(stale_manager);
  ASSERT_TRUE(stale_runtime_before.present);
  ASSERT_FALSE(stale_manager.swarmParticlesManager.empty());
  ASSERT_TRUE(stale_manager.swarmParticlesManager.front().gvf_);
  const Eigen::MatrixXd stale_display_before =
      stale_manager.swarmParticlesManager.front().last_traj;

  FLAG_Race::GvfManagerS4AnchorTestAccess::installPrematureCommittedMirrorV2(
      stale_manager, stale_fixture);
  const auto premature =
      FLAG_Race::GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
          stale_manager);
  ASSERT_TRUE(premature);
  ASSERT_TRUE(premature->committedComplete());
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   consumeCommittedPathReferenceV2(stale_manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
      stale_manager));
  const auto stale_phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(stale_manager);
  const auto stale_runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(stale_manager);
  EXPECT_DOUBLE_EQ(stale_phase_after.w, stale_phase_before.w);
  EXPECT_EQ(stale_phase_after.generation, stale_phase_before.generation);
  EXPECT_DOUBLE_EQ(stale_runtime_after.retained_delta,
                   stale_runtime_before.retained_delta);
  EXPECT_DOUBLE_EQ(stale_runtime_after.previous_port.u_w,
                   stale_runtime_before.previous_port.u_w);
  EXPECT_DOUBLE_EQ(stale_runtime_after.previous_port.u_delta,
                   stale_runtime_before.previous_port.u_delta);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::executionBindingV2(
                stale_manager),
            stale_fixture.source_binding);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::commandPathForV2(
                stale_manager, stale_fixture.source_binding),
            stale_fixture.source_owner);
  EXPECT_EQ(stale_manager.swarmParticlesManager.front().gvf_
                ->getContinuousPhasePath(),
            stale_fixture.source_owner);
  EXPECT_TRUE((stale_manager.swarmParticlesManager.front().last_traj.array() ==
               stale_display_before.array()).all());
  FLAG_Race::GvfManagerS4AnchorTestAccess::discardPendingV2Command(
      stale_manager);

  FLAG_Race::gvf_manager reset_manager;
  FLAG_Race::GvfManagerS4AnchorTestAccess::V2ManagerTransactionFixture
      reset_fixture;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  prepareV2ManagerTransaction(reset_manager, reset_fixture));
  ASSERT_FALSE(reset_manager.swarmParticlesManager.empty());
  ASSERT_TRUE(reset_manager.swarmParticlesManager.front().gvf_);
  const Eigen::MatrixXd reset_display_before =
      reset_manager.swarmParticlesManager.front().last_traj;
  const auto reset_phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(reset_manager);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingCommand(
                  reset_manager).pending);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
      reset_manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::executionBindingV2(
      reset_manager));

  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  resetForNewNavigationTask(reset_manager));
  const auto reset_phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(reset_manager);
  const auto reset_runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(reset_manager);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingCommand(
      reset_manager).pending);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingPathReferenceV2(
      reset_manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::executionBindingV2(
      reset_manager));
  EXPECT_DOUBLE_EQ(reset_phase_after.w, 0.0);
  EXPECT_FALSE(reset_phase_after.initialized);
  EXPECT_FALSE(reset_phase_after.closed_acquired);
  EXPECT_GT(reset_phase_after.generation, reset_phase_before.generation);
  ASSERT_TRUE(reset_runtime_after.present);
  EXPECT_DOUBLE_EQ(reset_runtime_after.retained_delta, 0.0);
  EXPECT_DOUBLE_EQ(reset_runtime_after.previous_port.u_w, 0.0);
  EXPECT_DOUBLE_EQ(reset_runtime_after.previous_port.u_delta, 0.0);

  int stale_local_publish_count = 0;
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::publishPendingCommand(
      reset_manager, [&]() {
        ++stale_local_publish_count;
        return true;
      }, reset_fixture.pending_command.identity));
  EXPECT_EQ(stale_local_publish_count, 0);
  const auto stale_replay = FLAG_Race::GvfManagerS4AnchorTestAccess::
      publishV2ManagerTransaction(reset_manager, reset_fixture, true);
  EXPECT_FALSE(stale_replay.attempted);
  EXPECT_FALSE(stale_replay.published);
  EXPECT_EQ(stale_replay.local_publish_count, 0);
  EXPECT_EQ(stale_replay.post_publish_count, 0);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::executionBindingV2(
      reset_manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   consumeCommittedPathReferenceV2(reset_manager));
  EXPECT_EQ(reset_manager.swarmParticlesManager.front().gvf_
                ->getContinuousPhasePath(),
            reset_fixture.source_owner);
  EXPECT_TRUE((reset_manager.swarmParticlesManager.front().last_traj.array() ==
               reset_display_before.array()).all());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
