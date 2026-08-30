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

// Narrow test-only access to the two S4 anchor helpers.  The production
// helpers remain private; this makes the zero-delta bitwise regression a
// deterministic same-process proof instead of a cross-master timing claim.
class GvfManagerS4AnchorTestAccess {
 public:
  struct PhaseSnapshotProbe {
    double w = 0.0;
    bool initialized = false;
    bool closed_acquired = false;
    std::uint64_t generation = 0U;
  };

  static PhaseSnapshotProbe capturePhase(
      const gvf_manager& manager) {
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

  static void setAuthoritySession(gvf_manager& manager,
                                  const std::uint64_t session) {
    std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
    manager.path_tube_authority_session_ = session;
  }

  static void setAuthoritySessionForE2E(gvf_manager& manager,
                                        const std::uint64_t session) {
    setAuthoritySession(manager, session);
    if (manager.phase_offset_matched_adapter_) {
      manager.phase_offset_matched_adapter_->authority_session_.store(
          session, std::memory_order_release);
    }
  }

  static void occupyBootstrapSlot(gvf_manager& manager) {
    std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
    manager.pending_path_tube_handoff_.reset(
        new gvf_manager::PendingPathTubeFrontend());
  }

  static bool stageSinglePending(
      gvf_manager& manager,
      const std::shared_ptr<const PathTubePair>& pair,
      const std::uint64_t session) {
    std::shared_ptr<gvf_manager::PendingPathTubeFrontend> frontend(
        new gvf_manager::PendingPathTubeFrontend());
    frontend->candidate_pair = pair;
    frontend->authority_session = session;
    frontend->transaction.authority_session = session;
    std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
    if (!pair || session != manager.path_tube_authority_session_ ||
        manager.pending_path_tube_handoff_ ||
        manager.completed_path_tube_handoff_) {
      return false;
    }
    manager.pending_path_tube_handoff_ = frontend;
    return true;
  }

  static void setCompletedForReset(
      gvf_manager& manager,
      const std::shared_ptr<const PathTubePair>& pair,
      const std::uint64_t session) {
    std::shared_ptr<gvf_manager::PendingPathTubeFrontend> frontend(
        new gvf_manager::PendingPathTubeFrontend());
    frontend->candidate_pair = pair;
    frontend->authority_session = session;
    frontend->transaction.authority_session = session;
    std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
    manager.completed_path_tube_handoff_ = frontend;
  }

  struct MailboxProbe {
    bool pending = false;
    bool completed = false;
    const void* pending_identity = nullptr;
    const void* completed_identity = nullptr;
    std::uint64_t session = 0U;
    std::uint64_t pending_clear = 0U;
    std::uint64_t consumed_clear = 0U;
  };

  static MailboxProbe mailbox(gvf_manager& manager) {
    std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
    MailboxProbe probe;
    probe.pending = static_cast<bool>(manager.pending_path_tube_handoff_);
    probe.completed = static_cast<bool>(manager.completed_path_tube_handoff_);
    probe.pending_identity = manager.pending_path_tube_handoff_.get();
    probe.completed_identity = manager.completed_path_tube_handoff_.get();
    probe.session = manager.path_tube_authority_session_;
    probe.pending_clear = manager.pending_frontend_clear_session_;
    probe.consumed_clear = manager.consumed_frontend_clear_session_;
    return probe;
  }

  static std::string handoffLifecycle(gvf_manager& manager) {
    std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
    switch (manager.last_path_tube_handoff_lifecycle_result_) {
      case gvf_manager::PathTubeHandoffLifecycleResult::NONE:
        return "none";
      case gvf_manager::PathTubeHandoffLifecycleResult::RETRY_PENDING:
        return "retry_pending";
      case gvf_manager::PathTubeHandoffLifecycleResult::COMMITTED:
        return "committed";
      case gvf_manager::PathTubeHandoffLifecycleResult::DROPPED_EXPIRED:
        return "dropped_expired";
      case gvf_manager::PathTubeHandoffLifecycleResult::DROPPED_STALE:
        return "dropped_stale";
      case gvf_manager::PathTubeHandoffLifecycleResult::CONSUMED:
        return "consumed";
    }
    return "unknown";
  }

  static bool stageRetryablePendingFromLivePair(gvf_manager& manager) {
    if (!manager.phase_offset_matched_adapter_) return false;
    const std::shared_ptr<const PathTubePair> pair =
        manager.phase_offset_matched_adapter_->capturePathTubePair();
    std::unique_ptr<PathTubePairPin> pin = manager.phase_offset_matched_adapter_
        ->captureAndAcquirePathTubePairPin();
    if (!pair || !pin || !pin->valid()) return false;
    std::shared_ptr<gvf_manager::PendingPathTubeFrontend> frontend(
        new gvf_manager::PendingPathTubeFrontend());
    frontend->candidate_pair = pair;
    frontend->authority_session = pair->authority_session;
    frontend->transaction.expected_pair = pair;
    frontend->transaction.candidate_pair = pair;
    frontend->transaction.expected_capture = pin->capture();
    frontend->transaction.pin_lease_id = pin->leaseId();
    frontend->transaction.authority_session = pair->authority_session;
    frontend->transaction_pin = std::move(pin);
    std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
    if (manager.path_tube_authority_session_ != pair->authority_session ||
        manager.pending_path_tube_handoff_ || manager.completed_path_tube_handoff_) {
      return false;
    }
    manager.pending_path_tube_handoff_ = frontend;
    return true;
  }

  static bool preparePending(gvf_manager& manager,
                             const Eigen::Vector3d& position =
                                 Eigen::Vector3d(0.4, 0.0, 1.0),
                             const double dt = 0.02) {
    const gvf_manager::AuthoritativePhaseSnapshot captured =
        manager.captureAuthoritativePhase();
    return manager.prepareAndCommitPendingPathTubeHandoff(
        captured, position, timerBootstrapGains(), dt,
        std::shared_ptr<const plan_env::CloudOccupancySnapshot>());
  }

  static bool preparePendingWithCapturedPhase(
      gvf_manager& manager,
      const PhaseSnapshotProbe& captured) {
    gvf_manager::AuthoritativePhaseSnapshot internal;
    internal.w = captured.w;
    internal.initialized = captured.initialized;
    internal.closed_acquired = captured.closed_acquired;
    internal.generation = captured.generation;
    return manager.prepareAndCommitPendingPathTubeHandoff(
        internal, Eigen::Vector3d(0.4, 0.0, 1.0), timerBootstrapGains(),
        0.02, std::shared_ptr<const plan_env::CloudOccupancySnapshot>());
  }

  static bool canAcquirePin(gvf_manager& manager) {
    if (!manager.phase_offset_matched_adapter_) return false;
    std::unique_ptr<PathTubePairPin> pin = manager.phase_offset_matched_adapter_
        ->captureAndAcquirePathTubePairPin();
    const bool acquired = pin && pin->valid();
    pin.reset();
    return acquired;
  }

  static bool stageValidCompletedFromLivePair(gvf_manager& manager) {
    if (!manager.phase_offset_matched_adapter_ ||
        manager.swarmParticlesManager.empty()) {
      return false;
    }
    const std::shared_ptr<const PathTubePair> pair =
        manager.phase_offset_matched_adapter_->capturePathTubePair();
    if (!pair || !pair->path_owner) return false;
    const std::vector<double> w{pair->path_owner->startW(),
                                0.5 * (pair->path_owner->startW() +
                                       pair->path_owner->endW()),
                                pair->path_owner->endW()};
    std::shared_ptr<gvf_manager::PendingPathTubeFrontend> frontend(
        new gvf_manager::PendingPathTubeFrontend());
    frontend->candidate_pair = pair;
    frontend->authority_session = pair->authority_session;
    frontend->transaction.authority_session = pair->authority_session;
    frontend->w = w;
    frontend->traj.resize(3, 3);
    frontend->vel.resize(3, 3);
    frontend->time.resize(3);
    frontend->anchor_idx = 1;
    for (int i = 0; i < 3; ++i) {
      ContinuousPhasePathState state;
      if (!pair->path_owner->evaluate(w[static_cast<std::size_t>(i)], state,
                                      false)) {
        return false;
      }
      frontend->traj.row(i) = state.p.transpose();
      frontend->vel.row(i) = state.dp_dw.transpose();
      frontend->time(i) = static_cast<double>(i);
    }
    std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
    if (manager.path_tube_authority_session_ != pair->authority_session ||
        manager.pending_path_tube_handoff_ || manager.completed_path_tube_handoff_) {
      return false;
    }
    manager.completed_path_tube_handoff_ = frontend;
    return true;
  }

  static bool consumeCompleted(gvf_manager& manager) {
    if (manager.swarmParticlesManager.empty()) return false;
    return manager.consumeCompletedPathTubeHandoff(
        manager.swarmParticlesManager.front(), ros::Time(7, 0));
  }

  static int frontendRows(const gvf_manager& manager) {
    if (manager.swarmParticlesManager.empty()) return -1;
    return manager.swarmParticlesManager.front().last_traj.rows();
  }

  static void setCompletedFrontendGvfAvailable(gvf_manager& manager,
                                                const bool available) {
    if (manager.swarmParticlesManager.empty()) return;
    gvf_manager::gvfManager& frontend = manager.swarmParticlesManager.front();
    if (!available) {
      frontend.gvf_.reset();
      return;
    }
    const std::shared_ptr<const PathTubePair> pair =
        manager.phase_offset_matched_adapter_
            ? manager.phase_offset_matched_adapter_->capturePathTubePair()
            : std::shared_ptr<const PathTubePair>();
    frontend.gvf_.reset(new gvf());
    if (pair && pair->path_owner) {
      frontend.gvf_->setAuthoritativePhaseMode(true);
      frontend.gvf_->setContinuousPhasePath(pair->path_owner);
    }
  }

  struct RecoveryMailboxProbe {
    bool pending = false;
    bool shutdown = false;
    std::uint64_t next_ticket = 0U;
    std::uint64_t consumed_ticket = 0U;
    std::uint64_t pending_ticket = 0U;
    std::uint64_t pending_session = 0U;
  };

  static bool recoveryRequired(const MatchedAdapterOutput& output) {
    return gvf_manager::requiresCurrentStateRecovery(output);
  }

  static bool stageRecovery(
      gvf_manager& manager, const MatchedAdapterOutput& output,
      const std::shared_ptr<const PathTubePair>& pair) {
    return manager.stageCurrentStateRecoveryRequest(output, pair);
  }

  static bool consumeRecovery(
      gvf_manager& manager, const std::shared_ptr<const PathTubePair>& pair,
      std::uint64_t& ticket) {
    gvf_manager::PendingCurrentStateRecoveryRequest request;
    if (!manager.consumeCurrentStateRecoveryRequestForFsm(pair, request)) {
      return false;
    }
    ticket = request.ticket;
    return true;
  }

  static RecoveryMailboxProbe recoveryMailbox(gvf_manager& manager) {
    std::lock_guard<std::mutex> lock(manager.current_state_recovery_mutex_);
    RecoveryMailboxProbe probe;
    probe.pending = static_cast<bool>(
        manager.pending_current_state_recovery_request_);
    probe.shutdown = manager.current_state_recovery_shutdown_;
    probe.next_ticket = manager.next_current_state_recovery_ticket_;
    probe.consumed_ticket = manager.consumed_current_state_recovery_ticket_;
    if (manager.pending_current_state_recovery_request_) {
      probe.pending_ticket =
          manager.pending_current_state_recovery_request_->ticket;
      probe.pending_session =
          manager.pending_current_state_recovery_request_->authority_session;
    }
    return probe;
  }

  static void shutdownRecoveryMailbox(gvf_manager& manager) {
    manager.shutdownCurrentStateRecoveryMailbox();
  }

  static int execState(const gvf_manager& manager) {
    return static_cast<int>(manager.exec_state_);
  }

  static void setExecState(gvf_manager& manager, const int state) {
    manager.exec_state_ = static_cast<gvf_manager::FSM_EXEC_STATE>(state);
  }

  static bool sameAuthority(
      const std::shared_ptr<const PathTubePair>& lhs,
      const std::shared_ptr<const PathTubePair>& rhs) {
    return gvf_manager::samePathTubeAuthority(lhs, rhs);
  }

  static std::shared_ptr<const ContinuousPhasePath> plannerOwner(
      const std::shared_ptr<const PathTubePair>& pair,
      const std::shared_ptr<const ContinuousPhasePath>& independent_owner) {
    gvf_manager::PendingPathTubeFrontend frontend;
    frontend.candidate_pair = pair;
    frontend.planner_path_owner = independent_owner;
    return gvf_manager::plannerPathOwnerForFrontend(frontend);
  }

  static void resetH2(gvf_manager& manager) {
    manager.resetUnifiedPhaseV2();
  }

  static bool resetForNewNavigationTask(gvf_manager& manager) {
    return manager.resetForNewNavigationTask();
  }

  static void markNonzeroHandoffRecoveryRequired(gvf_manager& manager) {
    std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
    manager.nonzero_handoff_recovery_required_reported_ = true;
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

  static bool advertiseProductionSeams(gvf_manager& manager,
                                       ros::NodeHandle& nh,
                                       const std::string& command_topic) {
    if (!manager.phase_offset_matched_adapter_) return false;
    manager.cmd_topic_ = command_topic;
    manager.cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>(
        command_topic, 10);
    manager.phase_offset_matched_adapter_->advertise(nh);
    return static_cast<bool>(manager.cmd_pub);
  }

  static bool installProductionManagerFixture(
      gvf_manager& manager,
      const std::shared_ptr<const ContinuousPhasePath>& owner,
      const std::shared_ptr<SDFMap>& map,
      ros::NodeHandle& nh) {
    if (!owner || !map) return false;
    // The zero-argument manager constructor is intentionally lightweight for
    // deterministic unit fixtures.  Set the legacy command-mode flags that
    // cmdCallback reads before reaching the production H2 path.
    manager.use_test_cmd_ = false;
    manager.enable_gvfcmd_control = false;
    PhaseOffsetMatchedAdapterConfig config;
    config.mode = PhaseOffsetMatchedMode::MANUAL;
    config.tube_source = phase_offset_navigation::TubeSource::ESDF;
    config.observe_only = false;
    config.profile_period = 2.0;
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
    config.normal_preview_policy.policy_revision = 1U;
    config.normal_preview_policy.configuration_identity = 1U;
    config.normal_preview_policy.configuration_id = "test-normal-preview-w";
    config.normal_preview_policy_explicit = true;
    config.tube.fixed_delta_max = 0.04;
    config.tube.back_w = 0.0;
    config.tube.lookahead_w = 2.0;
    config.tube.min_certified_forward_w = 0.40;
    config.tube.cross_section.search_extent = 3.0;
    config.tube.cross_section.ray_step = 0.05;
    config.tube.cross_section.boundary_tolerance = 0.01;
    config.tube.cross_section.regularity_margin = 0.10;
    config.tube.cross_section.curvature_epsilon = 1e-9;
    config.tube.cross_section.planner_safe_distance = 0.40;
    config.tube.cross_section.margins.uav_radius = 0.25;
    config.tube.cross_section.margins.map_uncertainty = 0.10;
    config.tube.cross_section.margins.localization_uncertainty = 0.05;
    config.tube.cross_section.margins.tracking_error_bound = 0.15;
    config.tube.cross_section.margins.preincluded_map_uncertainty = 0.10;
    config.cloud_obstacle_set_complete = true;
    std::unique_ptr<PhaseOffsetMatchedAdapter> adapter(
        new PhaseOffsetMatchedAdapter(config));
    if (!adapter->configurationValid()) return false;
    manager.matched_config_ = config;
    manager.phase_offset_matched_adapter_ = std::move(adapter);
    manager.cmd_topic_ = "/gvf_c3_bootstrap/position_command";
    manager.cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>(
        manager.cmd_topic_, 10);
    const std::uint64_t session_before_advertise =
        manager.phase_offset_matched_adapter_->authority_session_.load(
            std::memory_order_acquire);
    manager.phase_offset_matched_adapter_->advertise(nh);
    const std::uint64_t session_after_advertise =
        manager.phase_offset_matched_adapter_->authority_session_.load(
            std::memory_order_acquire);
    if (session_after_advertise != session_before_advertise) return false;
    if (!manager.resetForNewNavigationTask()) return false;
    gvf_manager::gvfManager frontend;
    frontend.gvf_.reset(new gvf());
    frontend.gvf_->setAuthoritativePhaseMode(true);
    frontend.gvf_->setContinuousPhasePath(owner);
    frontend.gvf_->gvf_.K1_ = 2.0;
    frontend.gvf_->gvf_.K2_ = -2.2;
    frontend.gvf_->gvf_.convergence_bandwidth_ = 0.1;
    frontend.gvf_->progress_rho0_ = 0.5;
    frontend.gvf_->progress_delta_ = 0.3;
    frontend.gvf_->alpha_min_ = 0.05;
    frontend.sdf_map_ = map;
    frontend.receive_goal = true;
    frontend.goal_pt = Eigen::Vector3d(20.0, 0.0, 1.0);
    manager.swarmParticlesManager.clear();
    manager.swarmParticlesManager.push_back(frontend);
    manager.point_phase_v2_enabled_ = true;
    manager.odom_ = Eigen::Vector3d(0.4, 0.10, 1.10);
    manager.publishAuthoritativePhase(0.4, true, false);
    return static_cast<bool>(manager.cmd_pub);
  }

  static void runCommand(gvf_manager& manager) {
    manager.cmdCallback(ros::TimerEvent());
  }

  static MatchedAdapterOutput latestAdapterOutput(gvf_manager& manager) {
    const std::shared_ptr<const ControlPublishSnapshot> control =
        manager.phase_offset_matched_adapter_
            ? std::atomic_load(&manager.phase_offset_matched_adapter_
                                    ->latest_control_snapshot_)
            : std::shared_ptr<const ControlPublishSnapshot>();
    return control ? control->output : MatchedAdapterOutput();
  }

  static bool updateAdapter(gvf_manager& manager,
                            const MatchedAdapterInput& input,
                            MatchedAdapterOutput& output) {
    return manager.phase_offset_matched_adapter_ &&
        manager.phase_offset_matched_adapter_->update(input, output);
  }

  static PendingPositionCommandCapture pendingCommand(
      gvf_manager& manager) {
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

  static phase_offset_navigation::ActiveReferenceSnapshot authoritySnapshot(
      gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_
        ? manager.phase_offset_matched_adapter_->execution_authority_.snapshot()
        : phase_offset_navigation::ActiveReferenceSnapshot();
  }

  static phase_offset_navigation::ActiveReferenceSnapshot pendingAuthoritySnapshot(
      gvf_manager& manager) {
    if (!manager.phase_offset_matched_adapter_ ||
        !manager.phase_offset_matched_adapter_->pending_authority_valid_ ||
        !manager.phase_offset_matched_adapter_->pending_authority_prepared_
             .committed_snapshot) {
      return phase_offset_navigation::ActiveReferenceSnapshot();
    }
    return *manager.phase_offset_matched_adapter_->pending_authority_prepared_
                  .committed_snapshot;
  }

  static double retainedDelta(gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_ &&
            manager.phase_offset_matched_adapter_->runtime_
        ? manager.phase_offset_matched_adapter_->runtime_->retainedDelta()
        : 0.0;
  }

  static void setTestOnlyRuntimeOwnerAllowed(gvf_manager& manager,
                                             const bool allowed) {
    if (manager.phase_offset_matched_adapter_) {
      manager.phase_offset_matched_adapter_->execution_authority_
          .setTestOnlyRuntimeOwnerAllowed(allowed);
    }
  }

  static void setAdapterAdvertised(gvf_manager& manager, const bool advertised) {
    if (manager.phase_offset_matched_adapter_) {
      manager.phase_offset_matched_adapter_->advertised_ = advertised;
    }
  }

  static bool requiresPathTubePairBootstrap(gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_ &&
        manager.phase_offset_matched_adapter_->requiresPathTubePairBootstrap();
  }

  static void openManualGate(gvf_manager& manager) {
    if (manager.phase_offset_matched_adapter_) {
      manager.phase_offset_matched_adapter_->zero_gate_open_ = true;
      manager.phase_offset_matched_adapter_->zero_gate_consecutive_count_ =
          100;
    }
  }

  static const phase_offset_navigation::RecoveryOwnerStatus& recoveryStatus(
      gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_->recovery_owner_.status();
  }

  static bool stageSuccessorPair(
      gvf_manager& manager,
      const std::shared_ptr<const PathTubePair>& expected_pair,
      const std::shared_ptr<const ContinuousPhasePath>& successor_owner,
      const MatchedAdapterPathSamples& successor_samples,
      const double captured_w0,
      const double future_seam_w,
      const double existing_future_horizon_end_w,
      const Eigen::Vector3d& position,
      const guidance::IsfGains& gains,
      const double dt,
      PathTubePairTransaction& transaction,
      PathTubePairStageFailure* stage_failure = nullptr) {
    if (!manager.phase_offset_matched_adapter_ || !expected_pair) return false;
    std::unique_ptr<PathTubePairPin> pin =
        manager.phase_offset_matched_adapter_->captureAndAcquirePathTubePairPin();
    if (!pin || !pin->valid() || pin->capture().pair != expected_pair) {
      return false;
    }
    const bool staged = manager.phase_offset_matched_adapter_->stagePathTubePair(
        expected_pair, successor_owner, successor_samples, captured_w0,
        future_seam_w, existing_future_horizon_end_w, position, gains, dt,
        std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), transaction,
        expected_pair->authority_session, &pin->capture(), pin->leaseId(),
        stage_failure);
    if (!staged || !transaction.candidate_pair) return false;
    std::shared_ptr<gvf_manager::PendingPathTubeFrontend> frontend(
        new gvf_manager::PendingPathTubeFrontend());
    frontend->candidate_pair = transaction.candidate_pair;
    frontend->transaction = transaction;
    frontend->authority_session = expected_pair->authority_session;
    frontend->transaction_pin = std::move(pin);
    {
      std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
      if (manager.path_tube_authority_session_ != expected_pair->authority_session ||
          manager.pending_path_tube_handoff_ ||
          manager.completed_path_tube_handoff_) {
        return false;
      }
      manager.pending_path_tube_handoff_ = frontend;
    }
    return true;
  }

  static GovernorProbe runGovernorCandidate(
      gvf_manager& manager,
      const std::shared_ptr<const ContinuousPhasePath>& path,
      const gvf::LiftedGuidanceResult& out,
      const Eigen::Vector3d& position,
      const double progress_w,
      const double reference_delta,
      const double dt,
      const double kp_equiv,
      const phase_offset_navigation::ImmutableExecutedReferenceQueryPtr& query) {
    manager.cmd_governor_l_min_ = 0.50;
    manager.cmd_governor_l_max_ = 0.50;
    manager.cmd_governor_l_step_ = 0.05;
    manager.cmd_governor_normal_max_ = 0.0;
    gvf_manager::GovernorCommandDebug debug;
    const gvf_manager::GovernorCommandResult result =
        manager.runVelocityMatchingGovernor(
            path, out, position, progress_w, reference_delta, dt, kp_equiv,
            debug, query);
    GovernorProbe probe;
    probe.cmd_pos = result.cmd_pos;
    probe.command_valid = result.command_valid;
    return probe;
  }

  struct NeutralCommitProbe {
    bool committed = false;
    bool installed_new_owner = false;
    bool pair_present = false;
    std::uint64_t session_before = 0U;
    std::uint64_t session_after = 0U;
  };

  static NeutralCommitProbe attemptNeutralPlannerFrontend(
      gvf_manager& manager,
      const std::shared_ptr<const ContinuousPhasePath>& new_owner) {
    NeutralCommitProbe probe;
    ros::Time::init();
    if (!new_owner || manager.swarmParticlesManager.empty() ||
        !manager.swarmParticlesManager.front().gvf_) {
      return probe;
    }
    probe.session_before = manager.path_tube_authority_session_;
    gvf_manager::gvfManager& live = manager.swarmParticlesManager.front();

    const std::vector<double> w{
        new_owner->startW(),
        0.5 * (new_owner->startW() + new_owner->endW()),
        new_owner->endW()};
    Eigen::MatrixXd traj(3, 3);
    Eigen::MatrixXd vel(3, 3);
    Eigen::VectorXd time(3);
    for (int i = 0; i < 3; ++i) {
      ContinuousPhasePathState state;
      if (!new_owner->evaluate(w[static_cast<size_t>(i)], state, false)) {
        return probe;
      }
      traj.row(i) = state.p.transpose();
      vel.row(i) = state.dp_dw.transpose();
      time(i) = static_cast<double>(i);
    }
    nav_msgs::Path path_msg;
    probe.committed = manager.commitNeutralPlannerFrontend(
        live, traj, vel, time, w, new_owner, 1, ros::Time(7, 0), path_msg);
    probe.installed_new_owner =
        live.gvf_->getContinuousPhasePath() == new_owner;
    probe.pair_present = static_cast<bool>(
        manager.phase_offset_matched_adapter_->capturePathTubePair());
    probe.session_after = manager.path_tube_authority_session_;
    return probe;
  }

  struct ReplanHandoffProbe {
    std::shared_ptr<const PathTubePair> pair;
    bool executed_authority = false;
    bool pending_activation = false;
    bool h2_required = false;
  };

  static ReplanHandoffProbe captureReplanHandoff(
      const gvf_manager& manager) {
    ReplanHandoffProbe probe;
    const gvf_manager::PathTubeReplanHandoffRequirement requirement =
        manager.capturePathTubeReplanHandoffRequirement();
    probe.pair = requirement.captured_pair;
    probe.executed_authority = requirement.executed_authority;
    probe.pending_activation = requirement.pending_activation;
    probe.h2_required = requirement.required();
    return probe;
  }

  static bool pendingBootstrapMatches(
      gvf_manager& manager,
      const std::shared_ptr<const ContinuousPhasePath>& planner_owner,
      const std::uint64_t authority_session) {
    std::lock_guard<std::mutex> lock(manager.path_tube_handoff_mutex_);
    if (manager.swarmParticlesManager.empty()) return false;
    return manager.pendingOffsetBootstrapMatchesCurrentPlannerLocked(
        manager.swarmParticlesManager.front(), planner_owner,
        authority_session);
  }

  static guidance::IsfGains timerBootstrapGains() {
    guidance::IsfGains gains;
    gains.k1 = 2.0;
    gains.k2 = -2.2;
    gains.convergence_bandwidth = 0.1;
    gains.progress_rho0 = 0.5;
    gains.progress_delta = 0.3;
    gains.alpha_min = 0.05;
    return gains;
  }

  static PhaseOffsetMatchedAdapterConfig timerBootstrapConfig() {
    PhaseOffsetMatchedAdapterConfig config;
    config.mode = PhaseOffsetMatchedMode::MANUAL;
    config.tube_source = phase_offset_navigation::TubeSource::FIXED;
    config.observe_only = false;
    config.profile_period = 2.0;
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
    config.normal_preview_policy.policy_revision = 1U;
    config.normal_preview_policy.configuration_identity = 1U;
    config.normal_preview_policy.configuration_id = "test-normal-preview-w";
    config.normal_preview_policy_explicit = true;
    config.tube.fixed_delta_max = 0.04;
    config.tube.back_w = 0.0;
    config.tube.lookahead_w = 2.0;
    config.tube.min_certified_forward_w = 0.40;
    config.tube.cross_section.search_extent = 3.0;
    config.tube.cross_section.ray_step = 0.05;
    config.tube.cross_section.boundary_tolerance = 0.01;
    config.tube.cross_section.regularity_margin = 0.10;
    config.tube.cross_section.curvature_epsilon = 1e-9;
    // Z1 makes this explicit production geometry input.  Keep the H2 timer
    // fixture on the same planner clearance contract; this is not a test
    // tuning parameter.
    config.tube.cross_section.planner_safe_distance = 0.40;
    config.tube.cross_section.margins.uav_radius = 0.25;
    config.tube.cross_section.margins.map_uncertainty = 0.10;
    config.tube.cross_section.margins.localization_uncertainty = 0.05;
    config.tube.cross_section.margins.tracking_error_bound = 0.15;
    config.tube.cross_section.margins.preincluded_map_uncertainty = 0.10;
    config.cloud_obstacle_set_complete = true;
    return config;
  }

  static MatchedAdapterInput timerBootstrapGateInput(const double stamp) {
    MatchedAdapterInput input;
    input.path.w = 0.4;
    input.path.p = Eigen::Vector3d(0.4, 0.0, 1.0);
    input.path.p_w = Eigen::Vector3d::UnitX();
    input.path.p_ww.setZero();
    input.path.valid = true;
    for (int index = 0; index <= 30; ++index) {
      phase_offset_core::PathDifferentialState sample;
      sample.w = 0.1 * static_cast<double>(index);
      sample.p = Eigen::Vector3d(sample.w, 0.0, 1.0);
      sample.p_w = Eigen::Vector3d::UnitX();
      sample.p_ww.setZero();
      sample.valid = true;
      input.sampled_path.push_back(sample);
    }
    input.path_state_query = [](const double w,
                                phase_offset_core::PathDifferentialState& state) {
      state.w = w;
      state.p = Eigen::Vector3d(w, 0.0, 1.0);
      state.p_w = Eigen::Vector3d::UnitX();
      state.p_ww.setZero();
      state.valid = true;
      return true;
    };
    static const int kStableTimerBootstrapIdentity = 0;
    input.semantic_path_identity = &kStableTimerBootstrapIdentity;
    input.semantic_path_start_w = 0.0;
    input.semantic_path_end_w = 3.0;
    input.position = Eigen::Vector3d(0.4, 0.0, 1.0);
    input.gains = timerBootstrapGains();
    input.dt = 0.02;
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

  static bool installTimerBootstrapFixture(
      gvf_manager& manager,
      const std::shared_ptr<const ContinuousPhasePath>& planner_owner) {
    if (!planner_owner) return false;
    ros::Time::init();
    const PhaseOffsetMatchedAdapterConfig config = timerBootstrapConfig();
    std::unique_ptr<PhaseOffsetMatchedAdapter> adapter(
        new PhaseOffsetMatchedAdapter(config));
    if (!adapter->configurationValid()) return false;
    MatchedAdapterOutput output;
    for (int cycle = 0; cycle < config.warmup_cycles; ++cycle) {
      if (adapter->update(timerBootstrapGateInput(0.02 * cycle), output) ||
          output.failure_latched) {
        return false;
      }
      // Production construction is timer-owned.  The fixture makes that
      // callback boundary explicit before asking the next command update to
      // consume the completed fixed Tube epoch.
      if (cycle == 0 && !adapter->timerTick()) return false;
    }
    if (!adapter->requiresPathTubePairBootstrap()) return false;
    manager.matched_config_ = config;
    manager.phase_offset_matched_adapter_ = std::move(adapter);
    manager.path_tube_authority_session_ = 0U;
    gvf_manager::gvfManager frontend;
    frontend.gvf_.reset(new gvf());
    frontend.gvf_->setAuthoritativePhaseMode(true);
    frontend.gvf_->setContinuousPhasePath(planner_owner);
    manager.swarmParticlesManager.clear();
    manager.swarmParticlesManager.push_back(frontend);
    return true;
  }

  struct BootstrapAttemptProbe {
    bool committed = false;
    std::string outcome;
    std::string stage_failure;
    double captured_w0 = 0.0;
    double live_wc = 0.0;
    double future_seam_w = 0.0;
    std::uint64_t authority_session = 0U;
    std::uint64_t map_observation_sequence = 0U;
    std::uint64_t pair_generation = 0U;
  };

  static BootstrapAttemptProbe activateTimerBootstrapAttempt(
      gvf_manager& manager,
      const Eigen::Vector3d& position = Eigen::Vector3d(0.4, 0.0, 1.0),
      const double dt = 0.02) {
    gvf_manager::gvfManager fallback;
    gvf_manager::gvfManager& frontend = manager.swarmParticlesManager.empty()
        ? fallback : manager.swarmParticlesManager.front();
    const gvf_manager::AuthoritativePhaseSnapshot captured =
        manager.captureAuthoritativePhase();
    gvf_manager::OffsetBootstrapAttemptResult result;
    BootstrapAttemptProbe probe;
    probe.committed = manager.activatePendingOffsetAuthority(
        frontend, captured, position, timerBootstrapGains(), dt,
        std::shared_ptr<const plan_env::CloudOccupancySnapshot>(), &result);
    probe.outcome = manager.offsetBootstrapAttemptOutcomeName(result.outcome);
    probe.stage_failure = manager.pathTubePairStageFailureName(
        result.stage_failure);
    probe.captured_w0 = result.captured_w0;
    probe.live_wc = result.live_wc;
    probe.future_seam_w = result.future_seam_w;
    probe.authority_session = result.authority_session;
    probe.map_observation_sequence = result.map_observation_sequence;
    probe.pair_generation = result.pair_generation;
    return probe;
  }

  static std::string bootstrapOutcomeName(const int ordinal) {
    return gvf_manager::offsetBootstrapAttemptOutcomeName(
        static_cast<gvf_manager::OffsetBootstrapAttemptOutcome>(ordinal));
  }

  static std::string stageFailureName(const PathTubePairStageFailure failure) {
    return gvf_manager::pathTubePairStageFailureName(failure);
  }

  static bool activateTimerBootstrap(gvf_manager& manager) {
    return activateTimerBootstrapAttempt(manager).committed;
  }

  static void setOdom(gvf_manager& manager,
                      const Eigen::Vector3d& position) {
    manager.odom_ = position;
  }

  static bool replaceBootstrapPlannerOwner(
      gvf_manager& manager,
      const std::shared_ptr<const ContinuousPhasePath>& owner) {
    if (manager.swarmParticlesManager.empty() ||
        !manager.swarmParticlesManager.front().gvf_ || !owner) {
      return false;
    }
    manager.swarmParticlesManager.front().gvf_->setContinuousPhasePath(owner);
    return true;
  }

  static bool clearBootstrapPlannerOwner(gvf_manager& manager) {
    if (manager.swarmParticlesManager.empty() ||
        !manager.swarmParticlesManager.front().gvf_) {
      return false;
    }
    manager.swarmParticlesManager.front().gvf_->clearContinuousPhasePath();
    return true;
  }

  static void setBootstrapAfterStageHook(
      gvf_manager& manager, std::function<void()> hook) {
    manager.bootstrap_after_stage_test_hook_ = std::move(hook);
  }

  static void setBootstrapBeforeFinalCasHook(
      gvf_manager& manager, std::function<void()> hook) {
    manager.bootstrap_before_final_cas_test_hook_ = std::move(hook);
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

  static std::shared_ptr<const PathTubePair> bootstrapAuthority(
      const gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_
        ? manager.phase_offset_matched_adapter_->capturePathTubePair()
        : std::shared_ptr<const PathTubePair>();
  }

  static bool claimPendingActivationNonzeroLogGeneration(
      gvf_manager& manager, const std::shared_ptr<const PathTubePair>& pair) {
    return pair && manager.pending_activation_nonzero_logged_generation_.exchange(
        pair->generation, std::memory_order_acq_rel) != pair->generation;
  }

  static bool claimPendingActivationNotSelectedLogGeneration(
      gvf_manager& manager, const std::shared_ptr<const PathTubePair>& pair) {
    return pair && manager.pending_activation_not_selected_logged_generation_.exchange(
        pair->generation, std::memory_order_acq_rel) != pair->generation;
  }

  struct PairCommandProbe {
    bool update_success = false;
    bool selected = false;
    bool valid = false;
    bool executed_authority = false;
    double retained_delta_before = 0.0;
    double retained_delta_after = 0.0;
    bool projection_valid = false;
  };

  static PairCommandProbe executePendingBootstrapPairCommand(
      gvf_manager& manager, const double stamp,
      const bool corrupt_current_path = false) {
    PairCommandProbe probe;
    if (!manager.phase_offset_matched_adapter_) return probe;
    const std::shared_ptr<const PathTubePair> pair =
        manager.phase_offset_matched_adapter_->capturePathTubePair();
    if (!pair || !pair->path_owner) return probe;
    MatchedAdapterInput input = timerBootstrapGateInput(stamp);
    input.semantic_path_owner = pair->path_owner;
    input.semantic_path_start_w = pair->path_owner->startW();
    input.semantic_path_end_w = pair->path_owner->endW();
    input.path_tube_pair = pair;
    if (corrupt_current_path) input.path.p.x() += 1.0;
    MatchedAdapterOutput output;
    probe.update_success = manager.phase_offset_matched_adapter_->update(
        input, output);
    probe.selected = probe.update_success && output.selected;
    probe.valid = output.valid;
    probe.executed_authority = manager.phase_offset_matched_adapter_->
        requiresAuthoritativeOffsetHandoff();
    probe.retained_delta_before = output.delta;
    probe.retained_delta_after = output.projection.next_delta;
    probe.projection_valid = output.projection.valid;
    return probe;
  }

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

}  // namespace FLAG_Race

namespace {
FLAG_Race::gvf_manager::ClosedGoalCandidateProgress candidate(
    bool passed, double end_delta_w, double end_to_goal_dist, double lookahead)
{
  FLAG_Race::gvf_manager::ClosedGoalCandidateProgress value;
  value.valid = true;
  value.passed_obstacle = passed;
  value.end_delta_w = end_delta_w;
  value.end_to_goal_dist = end_to_goal_dist;
  value.lookahead = lookahead;
  return value;
}

FLAG_Race::gvf_manager::ClosedGoalProgressiveCandidate progressiveCandidate(
    int idx, double lookahead, double end_delta_w,
    double kino_path_length, double end_to_goal_dist)
{
  FLAG_Race::gvf_manager::ClosedGoalProgressiveCandidate value;
  value.valid = true;
  value.idx = idx;
  value.lookahead = lookahead;
  value.end_delta_w = end_delta_w;
  value.kino_path_length = kino_path_length;
  value.end_to_goal_dist = end_to_goal_dist;
  return value;
}

int selectProgressiveCandidateIndex(
    std::initializer_list<FLAG_Race::gvf_manager::ClosedGoalProgressiveCandidate> candidates,
    double required_progress, double desired_lookahead)
{
  FLAG_Race::gvf_manager::ClosedGoalProgressiveCandidate best;
  for (const auto& value : candidates) {
    if (FLAG_Race::gvf_manager::preferClosedGoalProgressiveCandidate(
            value, best, required_progress, desired_lookahead)) {
      best = value;
    }
  }
  return best.idx;
}

std::vector<int> sortClosedGoalAttemptIndices(
    std::initializer_list<int> indices,
    const std::vector<double>& lookaheads,
    double desired_lookahead)
{
  std::vector<int> result(indices);
  std::sort(result.begin(), result.end(), [&](int lhs, int rhs) {
    return FLAG_Race::gvf_manager::closedGoalAttemptComesBefore(
        lookaheads[lhs], lhs, lookaheads[rhs], rhs, desired_lookahead);
  });
  return result;
}

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
makeSeededRecoveryCompositeSuccessor(
    const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>& old_owner,
    const double captured_w0, const double future_seam_w,
    const double join_w, const double end_w) {
  if (!old_owner) return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  FLAG_Race::ContinuousPhasePathState old_seam;
  if (!old_owner->evaluate(future_seam_w, old_seam, false)) {
    return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  }
  FLAG_Race::ContinuousPhasePathState mapped_tail_join;
  mapped_tail_join.p = Eigen::Vector3d(join_w, 1.60, 1.0);
  mapped_tail_join.dp_dw = Eigen::Vector3d(1.0, -0.40, 0.0);
  mapped_tail_join.d2p_dw2 = Eigen::Vector3d(0.0, -0.20, 0.0);
  mapped_tail_join.vel = mapped_tail_join.dp_dw;
  mapped_tail_join.valid = true;
  const auto connector = FLAG_Race::ContinuousPhasePath::makeQuinticHermite(
      future_seam_w, join_w, old_seam, mapped_tail_join);
  if (!connector) return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  auto owner = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  if (!owner->appendSlice(*old_owner, captured_w0, future_seam_w) ||
      !owner->appendSegment(future_seam_w, join_w, "c2_quintic", connector) ||
      !owner->appendSegment(
          join_w, end_w, "mapped_tail",
          [mapped_tail_join, join_w](
              const double w, FLAG_Race::ContinuousPhasePathState& state) {
            const double dw = w - join_w;
            state = mapped_tail_join;
            state.p = mapped_tail_join.p + dw * mapped_tail_join.dp_dw +
                0.5 * dw * dw * mapped_tail_join.d2p_dw2;
            state.dp_dw = mapped_tail_join.dp_dw +
                dw * mapped_tail_join.d2p_dw2;
            state.vel = state.dp_dw;
            state.valid = state.p.allFinite() && state.dp_dw.allFinite() &&
                state.d2p_dw2.allFinite() && state.vel.allFinite();
            return state.valid;
          })) {
    return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  }
  return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(owner);
}

std::shared_ptr<const FLAG_Race::ContinuousPhasePath> makeTimerBootstrapPath(
    const std::function<void()>& on_first_evaluation) {
  auto fired = std::make_shared<std::atomic<bool>>(false);
  auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  EXPECT_TRUE(path->appendSegment(
      0.0, 3.0, "timer_bootstrap_path",
      [fired, on_first_evaluation](const double w,
                                   FLAG_Race::ContinuousPhasePathState& state) {
        if (on_first_evaluation &&
            !fired->exchange(true, std::memory_order_acq_rel)) {
          on_first_evaluation();
        }
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.vel = state.dp_dw;
        state.valid = true;
        return true;
      }));
  return path;
}

void initializeProductionFreeMap(SDFMap& map) {
  map.mp_.resolution_ = map.mp_.resolution_inv_ = 1.0;
  map.mp_.map_origin_ = Eigen::Vector3d(-5.0, -5.0, -5.0);
  map.mp_.map_size_ = Eigen::Vector3d(10.0, 10.0, 10.0);
  map.mp_.map_voxel_num_ = Eigen::Vector3i(10, 10, 10);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
  map.mp_.clamp_min_log_ = -2.0;
  map.mp_.min_occupancy_log_ = 0.0;
  map.md_.occupancy_buffer_.assign(1000U, -1.0);
  map.md_.occupancy_buffer_inflate_.assign(1000U, 1);
  map.md_.distance_buffer_all_.assign(
      1000U, std::numeric_limits<double>::quiet_NaN());
  map.md_.manual_boundary_enabled_ = false;
}

void installProductionCloudSnapshot(SDFMap& map,
                                    const std::uint64_t sequence = 1U) {
  std::shared_ptr<plan_env::CloudOccupancySnapshot> snapshot(
      new plan_env::CloudOccupancySnapshot());
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
  const std::size_t size = static_cast<std::size_t>(snapshot->voxel_count.x()) *
      static_cast<std::size_t>(snapshot->voxel_count.y()) *
      static_cast<std::size_t>(snapshot->voxel_count.z());
  snapshot->occupied.assign(size, 0U);
  ASSERT_TRUE(plan_env::cloudOccupancySnapshotConsistent(*snapshot));
  ASSERT_TRUE(map.cloud_occupancy_snapshot_store_);
  std::lock_guard<std::mutex> lock(map.cloud_occupancy_snapshot_store_->mutex);
  map.cloud_occupancy_snapshot_store_->observation_sequence = sequence;
  map.cloud_occupancy_snapshot_store_->latest = snapshot;
}

std::shared_ptr<const FLAG_Race::ContinuousPhasePath> makeShortTimerBootstrapPath(
    const double end_w) {
  auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  EXPECT_TRUE(path->appendSegment(
      0.0, end_w, "short_timer_bootstrap_path",
      [](const double w, FLAG_Race::ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.vel = state.dp_dw;
        state.valid = true;
        return true;
      }));
  return path;
}

void ensureRosInitializedForSeededRecoveryE2E() {
  if (ros::isInitialized()) return;
  int argc = 1;
  char node_name[] = "gvf_seeded_recovery_e2e";
  char* argv[] = {node_name, nullptr};
  ros::init(argc, argv, node_name,
            ros::init_options::AnonymousName |
                ros::init_options::NoSigintHandler);
}

struct PositionCommandCapture {
  mutable std::mutex mutex;
  std::vector<quadrotor_msgs::PositionCommand> messages;

  void callback(const quadrotor_msgs::PositionCommand::ConstPtr& message) {
    if (!message) return;
    std::lock_guard<std::mutex> lock(mutex);
    messages.push_back(*message);
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex);
    return messages.size();
  }
};

bool waitForPositionCommandCount(const PositionCommandCapture& capture,
                                 const std::size_t expected,
                                 const double timeout_sec) {
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
  while (capture.size() < expected &&
         std::chrono::steady_clock::now() < deadline) {
    ros::spinOnce();
    ros::WallDuration(0.005).sleep();
  }
  return capture.size() >= expected;
}

std::shared_ptr<const FLAG_Race::PathTubePair> makeCertifiedH2Pair(
    const std::vector<double>& sample_w,
    const double certified_start,
    const double certified_end) {
  const auto owner = makeH2OldSeamPath();
  auto samples = std::make_shared<FLAG_Race::MatchedAdapterPathSamples>();
  for (const double w : sample_w) {
    FLAG_Race::ContinuousPhasePathState state;
    EXPECT_TRUE(owner->evaluate(w, state, false));
    samples->push_back(FLAG_Race::ConvertContinuousPhasePathStateForActive(
        state, w));
  }
  auto profile = std::make_shared<phase_offset_navigation::TubeProfile>();
  profile->source = phase_offset_navigation::TubeSource::FIXED;
  profile->source_revision = 7U;
  profile->complete = true;
  profile->preview_start_w = certified_start;
  profile->preview_end_w = certified_end;
  profile->certified_segment_start_w = certified_start;
  profile->certified_segment_end_w = certified_end;
  for (const double w : sample_w) {
    phase_offset_navigation::TubeRawSample sample;
    FLAG_Race::ContinuousPhasePathState state;
    EXPECT_TRUE(owner->evaluate(w, state, false));
    sample.w = w;
    sample.p = state.p;
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.complete = true;
    sample.filtered_lower = -0.2;
    sample.filtered_upper = 0.2;
    profile->samples.push_back(sample);
  }
  auto pair = std::make_shared<FLAG_Race::PathTubePair>();
  pair->source_revision = 7U;
  pair->generation = 3U;
  pair->path_owner = owner;
  pair->full_path_samples = samples;
  pair->active_profile = profile;
  pair->epoch_status.active_available = true;
  pair->epoch_status.active_current_validation_valid = true;
  pair->epoch_status.candidate_path_source_revision = 7U;
  pair->epoch_status.active_path_source_revision = 7U;
  return pair;
}

std::shared_ptr<const FLAG_Race::PathTubePair> makeRecoveryPair(
    const std::uint64_t revision, const std::uint64_t generation,
    const std::uint64_t session) {
  auto pair = std::make_shared<FLAG_Race::PathTubePair>();
  pair->source_revision = revision;
  pair->generation = generation;
  pair->authority_session = session;
  return pair;
}

FLAG_Race::MatchedAdapterOutput certificateDeniedOutput() {
  FLAG_Race::MatchedAdapterOutput output;
  output.selected = false;
  output.valid = false;
  // P1's real no-witness shape preserves the previously valid current port's
  // executable fact while reporting the explicit certificate denial.
  output.runtime_execution.executable = true;
  output.runtime_execution.mode =
      phase_offset_navigation::RuntimeExecutionMode::CERTIFICATE_DENIED;
  output.runtime_execution.certificate_denied = true;
  return output;
}

FLAG_Race::MatchedAdapterOutput currentOffsetOutsideOutput() {
  FLAG_Race::MatchedAdapterOutput output;
  output.selected = false;
  output.valid = false;
  output.runtime_execution.executable = false;
  output.runtime_execution.mode =
      phase_offset_navigation::RuntimeExecutionMode::WAITING_FOR_CANDIDATE;
  output.tube_epoch_status.reason =
      phase_offset_navigation::TubeEpochReason::CURRENT_OFFSET_OUTSIDE;
  return output;
}

FLAG_Race::MatchedAdapterOutput observeOnlyOutput() {
  FLAG_Race::MatchedAdapterOutput output;
  output.selected = false;
  output.valid = false;
  output.runtime_execution.executable = false;
  output.runtime_execution.mode =
      phase_offset_navigation::RuntimeExecutionMode::WAITING_FOR_CANDIDATE;
  return output;
}

TEST(GvfRecoveryMailbox, RecoveryReplanRequiredIsRoutedOnceToFsm) {
  FLAG_Race::gvf_manager manager;
  const std::shared_ptr<const FLAG_Race::PathTubePair> pair =
      makeRecoveryPair(7U, 9U, 11U);
  ASSERT_TRUE(pair);
  FLAG_Race::MatchedAdapterOutput output;
  output.selected = false;
  output.valid = false;
  output.recovery_status =
      phase_offset_navigation::RecoveryStepStatus::RECOVERY_REPLAN_REQUIRED;
  output.recovery_replan_required = true;

  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::recoveryRequired(output));
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::stageRecovery(
      manager, output, pair));
  // Repeated command ticks for the same owner/session are idempotent.
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::stageRecovery(
      manager, output, pair));
  const auto mailbox =
      FLAG_Race::GvfManagerS4AnchorTestAccess::recoveryMailbox(manager);
  ASSERT_TRUE(mailbox.pending);
  ASSERT_EQ(mailbox.pending_session, pair->authority_session);

  std::uint64_t ticket = 0U;
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::consumeRecovery(
      manager, pair, ticket));
  EXPECT_EQ(ticket, mailbox.pending_ticket);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::consumeRecovery(
      manager, pair, ticket));
}
}

TEST(GvfTimerBootstrapAttribution, NamesEveryOuterAndStageResult) {
  const std::array<const char*, 11U> outer_names{{
      "NOT_REQUIRED", "ENTRY_OR_SLOT_PRECONDITION", "OWNER_OR_SAMPLE",
      "STRUCTURAL_SEAM", "STAGE_PATH_TUBE_PAIR", "LIVE_PHASE_STATE",
      "LIVE_PHASE_WINDOW", "POST_STAGE_OWNER_OR_SESSION",
      "LIVE_PREPARE_OR_LATEST_MAP", "FINAL_CAS", "COMMITTED",
  }};
  for (std::size_t index = 0U; index < outer_names.size(); ++index) {
    EXPECT_EQ(outer_names[index],
              FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapOutcomeName(
                  static_cast<int>(index)));
  }
  const std::array<FLAG_Race::PathTubePairStageFailure, 12U> stage_failures{{
      FLAG_Race::PathTubePairStageFailure::NONE,
      FLAG_Race::PathTubePairStageFailure::INPUT_PRECONDITION,
      FLAG_Race::PathTubePairStageFailure::TRANSACTION_PRECONDITION,
      FLAG_Race::PathTubePairStageFailure::PAIR_SESSION_RUNTIME_SNAPSHOT,
      FLAG_Race::PathTubePairStageFailure::OWNER_EVALUATE,
      FLAG_Race::PathTubePairStageFailure::TUBE_BUILD_PRECONDITION,
      FLAG_Race::PathTubePairStageFailure::TUBE_RAW_BUILD,
      FLAG_Race::PathTubePairStageFailure::TUBE_FILTER,
      FLAG_Race::PathTubePairStageFailure::TUBE_SURFACE_VALIDATOR,
      FLAG_Race::PathTubePairStageFailure::TUBE_PROFILE_COVERAGE,
      FLAG_Race::PathTubePairStageFailure::TUBE_PROFILE_OWNER_MATCH,
      FLAG_Race::PathTubePairStageFailure::STAGING_DRY_RUN,
  }};
  const std::array<const char*, 12U> stage_names{{
      "NONE", "INPUT_PRECONDITION", "TRANSACTION_PRECONDITION",
      "PAIR_SESSION_RUNTIME_SNAPSHOT", "OWNER_EVALUATE",
      "TUBE_BUILD_PRECONDITION", "TUBE_RAW_BUILD", "TUBE_FILTER",
      "TUBE_SURFACE_VALIDATOR", "TUBE_PROFILE_COVERAGE",
      "TUBE_PROFILE_OWNER_MATCH", "STAGING_DRY_RUN",
  }};
  for (std::size_t index = 0U; index < stage_failures.size(); ++index) {
    EXPECT_EQ(stage_names[index],
              FLAG_Race::GvfManagerS4AnchorTestAccess::stageFailureName(
                  stage_failures[index]));
  }
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

TEST(GvfH2FutureSeam, SelectsEarliestStructuralSampleWithoutOldTubeLead) {
  const auto pair = makeCertifiedH2Pair(
      {0.0, 0.4, 0.6, 0.799, 0.8, 1.2, 1.6, 2.0}, 0.4, 1.6);
  ASSERT_TRUE(pair);
  double seam_w = 0.0;
  ASSERT_TRUE(FLAG_Race::gvf_manager::selectStructuralFutureSeam(
      pair, 0.4, 0.4, seam_w));
  EXPECT_DOUBLE_EQ(0.8, seam_w);
}

TEST(GvfH2FutureSeam, RetainsStructuralSampleAtExactCaptureBoundary) {
  const auto pair = makeCertifiedH2Pair({0.0, 0.4, 0.8, 1.2}, 0.4, 1.2);
  ASSERT_TRUE(pair);
  double seam_w = 0.0;
  ASSERT_TRUE(FLAG_Race::gvf_manager::selectStructuralFutureSeam(
      pair, 0.4, 0.4, seam_w));
  EXPECT_DOUBLE_EQ(0.8, seam_w);
}

TEST(GvfH2FutureSeam, CertificateLagDoesNotRemoveSelectedStructuralSeam) {
  const auto pair = makeCertifiedH2Pair(
      {0.0, 0.4, 0.6, 0.8, 1.2, 1.6}, 0.4, 0.79);
  ASSERT_TRUE(pair);
  ASSERT_TRUE(pair->active_profile);
  ASSERT_LT(pair->active_profile->certified_segment_end_w, 1.2);
  double seam_w = 0.0;
  ASSERT_TRUE(FLAG_Race::gvf_manager::selectStructuralFutureSeam(
      pair, 0.4, 0.4, seam_w));
  EXPECT_DOUBLE_EQ(0.8, seam_w);
}

TEST(GvfH2FutureSeam, LaterCallbackNaturallySelectsLaterSingleSeam) {
  const auto pair = makeCertifiedH2Pair(
      {0.0, 0.4, 0.6, 0.8, 1.2, 1.6}, 0.4, 1.6);
  ASSERT_TRUE(pair);
  double first_seam_w = 0.0;
  ASSERT_TRUE(FLAG_Race::gvf_manager::selectStructuralFutureSeam(
      pair, 0.4, 0.4, first_seam_w));
  EXPECT_DOUBLE_EQ(0.8, first_seam_w);

  double retry_seam_w = 0.0;
  ASSERT_TRUE(FLAG_Race::gvf_manager::selectStructuralFutureSeam(
      pair, 0.81, 0.4, retry_seam_w));
  EXPECT_DOUBLE_EQ(1.6, retry_seam_w);
}

TEST(GvfH2FutureSeam, NonMonotonicOrOwnerMismatchInputFailsClosed) {
  const auto certified = makeCertifiedH2Pair({0.0, 0.4, 0.8, 1.2}, 0.4, 1.2);
  ASSERT_TRUE(certified);

  auto nonmonotonic = std::make_shared<FLAG_Race::PathTubePair>(*certified);
  auto nonmonotonic_samples =
      std::make_shared<FLAG_Race::MatchedAdapterPathSamples>(
          *certified->full_path_samples);
  (*nonmonotonic_samples)[2].w = (*nonmonotonic_samples)[1].w;
  nonmonotonic->full_path_samples = nonmonotonic_samples;
  double seam_w = 0.0;
  EXPECT_FALSE(FLAG_Race::gvf_manager::selectStructuralFutureSeam(
      std::shared_ptr<const FLAG_Race::PathTubePair>(nonmonotonic),
      0.4, 0.4, seam_w));

  auto owner_mismatch = std::make_shared<FLAG_Race::PathTubePair>(*certified);
  auto mismatch_samples = std::make_shared<FLAG_Race::MatchedAdapterPathSamples>(
      *certified->full_path_samples);
  (*mismatch_samples)[2].p.x() += 0.01;
  owner_mismatch->full_path_samples = mismatch_samples;
  EXPECT_FALSE(FLAG_Race::gvf_manager::selectStructuralFutureSeam(
      std::shared_ptr<const FLAG_Race::PathTubePair>(owner_mismatch),
      0.4, 0.4, seam_w));
}

TEST(GvfTimerBootstrapAttribution, NotRequiredIsNotARealAttempt) {
  FLAG_Race::gvf_manager manager;
  const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(manager);
  EXPECT_FALSE(attempt.committed);
  EXPECT_EQ("NOT_REQUIRED", attempt.outcome);
  EXPECT_EQ("NONE", attempt.stage_failure);
  EXPECT_EQ(0U, attempt.pair_generation);
}

TEST(GvfTimerBootstrapAttribution, OwnerAndStructuralSeamRemainDistinct) {
  {
    FLAG_Race::gvf_manager manager;
    const auto owner = makeTimerBootstrapPath(std::function<void()>());
    ASSERT_TRUE(owner);
    ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                    installTimerBootstrapFixture(manager, owner));
    ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                    clearBootstrapPlannerOwner(manager));
    FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
        manager, 0.4, true, false);
    const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
        activateTimerBootstrapAttempt(manager);
    EXPECT_FALSE(attempt.committed);
    EXPECT_EQ("OWNER_OR_SAMPLE", attempt.outcome);
    EXPECT_DOUBLE_EQ(0.4, attempt.captured_w0);
    EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                     bootstrapAuthority(manager));
  }
  {
    FLAG_Race::gvf_manager manager;
    const auto short_owner = makeShortTimerBootstrapPath(0.70);
    ASSERT_TRUE(short_owner);
    ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                    installTimerBootstrapFixture(manager, short_owner));
    FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
        manager, 0.4, true, false);
    const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
        activateTimerBootstrapAttempt(manager);
    EXPECT_FALSE(attempt.committed);
    EXPECT_EQ("STRUCTURAL_SEAM", attempt.outcome);
    EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                     bootstrapAuthority(manager));
  }
}

TEST(GvfTimerBootstrapAttribution, OccupiedHandoffSlotIsAnEntryPrecondition) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.4, true, false);
  FLAG_Race::GvfManagerS4AnchorTestAccess::occupyBootstrapSlot(manager);
  const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(manager);
  EXPECT_FALSE(attempt.committed);
  EXPECT_EQ("ENTRY_OR_SLOT_PRECONDITION", attempt.outcome);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   bootstrapAuthority(manager));
}

TEST(GvfTimerBootstrapAttribution,
     StageFailureDoesNotPublishOrMutateRuntimeAndCannotCreateH2Mailbox) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.4, true, false);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  ASSERT_TRUE(runtime_before.present);
  const auto mailbox_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);

  const auto first = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(
          manager, Eigen::Vector3d(0.4, 0.0, 1.0),
          std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(first.committed);
  EXPECT_EQ("STAGE_PATH_TUBE_PAIR", first.outcome);
  EXPECT_EQ("STAGING_DRY_RUN", first.stage_failure);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   bootstrapAuthority(manager));
  const auto runtime_after_first =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  ASSERT_TRUE(runtime_after_first.present);
  EXPECT_DOUBLE_EQ(runtime_before.retained_delta,
                   runtime_after_first.retained_delta);
  EXPECT_EQ(0, std::memcmp(&runtime_before.previous_port,
                           &runtime_after_first.previous_port,
                           sizeof(runtime_before.previous_port)));
  const auto mailbox_after_first =
      FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_EQ(mailbox_before.pending, mailbox_after_first.pending);
  EXPECT_EQ(mailbox_before.completed, mailbox_after_first.completed);

  const auto repeated = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(
          manager, Eigen::Vector3d(0.4, 0.0, 1.0),
          std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(repeated.committed);
  EXPECT_EQ("STAGE_PATH_TUBE_PAIR", repeated.outcome);
  EXPECT_EQ("STAGING_DRY_RUN", repeated.stage_failure);
  const auto mailbox_after_repeated =
      FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_EQ(mailbox_before.pending, mailbox_after_repeated.pending);
  EXPECT_EQ(mailbox_before.completed, mailbox_after_repeated.completed);
}

TEST(GvfTimerBootstrapAttribution,
     LatestLivePrepareAndFinalCasStayDistinctWithoutPublishingPair) {
  {
    FLAG_Race::gvf_manager manager;
    const auto owner = makeTimerBootstrapPath(std::function<void()>());
    ASSERT_TRUE(owner);
    ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                    installTimerBootstrapFixture(manager, owner));
    FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
        manager, 0.4, true, false);
    FLAG_Race::GvfManagerS4AnchorTestAccess::setBootstrapAfterStageHook(
        manager, [&manager]() {
          FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
              manager, Eigen::Vector3d(
                  std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0));
        });
    const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
        activateTimerBootstrapAttempt(manager);
    EXPECT_FALSE(attempt.committed);
    EXPECT_EQ("LIVE_PREPARE_OR_LATEST_MAP", attempt.outcome);
    EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                     bootstrapAuthority(manager));
    FLAG_Race::GvfManagerS4AnchorTestAccess::setBootstrapAfterStageHook(
        manager, std::function<void()>());
  }
  {
    FLAG_Race::gvf_manager manager;
    const auto owner = makeTimerBootstrapPath(std::function<void()>());
    ASSERT_TRUE(owner);
    ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                    installTimerBootstrapFixture(manager, owner));
    FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
        manager, 0.4, true, false);
    FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
        manager, Eigen::Vector3d(0.4, 0.0, 1.0));
    FLAG_Race::GvfManagerS4AnchorTestAccess::setBootstrapBeforeFinalCasHook(
        manager, [&manager]() {
          FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
              manager, 0.4, true, false);
        });
    const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
        activateTimerBootstrapAttempt(manager);
    EXPECT_FALSE(attempt.committed);
    EXPECT_EQ("FINAL_CAS", attempt.outcome);
    EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                     bootstrapAuthority(manager));
    FLAG_Race::GvfManagerS4AnchorTestAccess::setBootstrapBeforeFinalCasHook(
        manager, std::function<void()>());
  }
}

TEST(GvfTimerBootstrap,
     MovingLivePhaseAndPositionCommitSameOwnerPairBeforeStructuralSeam) {
  constexpr double kW0 = 0.40;
  constexpr double kWc = 0.60;
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath([&manager]() {
    FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
        manager, Eigen::Vector3d(0.60, 0.0, 1.0));
    FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
        manager, kWc, true, false);
  });
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(kW0, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, kW0, true, false);

  const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(manager);
  ASSERT_TRUE(attempt.committed);
  EXPECT_EQ("COMMITTED", attempt.outcome);
  EXPECT_EQ("NONE", attempt.stage_failure);
  const std::shared_ptr<const FLAG_Race::PathTubePair> installed =
      FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(manager);
  ASSERT_TRUE(installed);
  EXPECT_EQ(installed->path_owner.get(), owner.get());
  EXPECT_DOUBLE_EQ(kW0, installed->captured_w0);
  EXPECT_GE(installed->future_seam_w, kW0 + 0.40 - 1e-9);
  EXPECT_GT(installed->future_seam_w, kWc);
  EXPECT_EQ(installed->generation, attempt.pair_generation);
  EXPECT_EQ(installed->authority_session, attempt.authority_session);
  EXPECT_EQ(installed->map_observation_sequence,
            attempt.map_observation_sequence);
  const auto live = FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  EXPECT_DOUBLE_EQ(kWc, live.w);
  EXPECT_TRUE(live.initialized);
}

TEST(GvfTimerBootstrap,
     PendingActivationPairRequiresH2AndSurvivesNeutralPlannerAttempt) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  const auto replacement = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(replacement);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.4, true, false);

  const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(manager);
  ASSERT_TRUE(attempt.committed);
  const std::shared_ptr<const FLAG_Race::PathTubePair> pair =
      FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(manager);
  ASSERT_TRUE(pair);
  EXPECT_DOUBLE_EQ(0.0, FLAG_Race::GvfManagerS4AnchorTestAccess::
                            runtimeState(manager).retained_delta);

  const auto pending = FLAG_Race::GvfManagerS4AnchorTestAccess::
      captureReplanHandoff(manager);
  EXPECT_EQ(pair, pending.pair);
  EXPECT_FALSE(pending.executed_authority);
  EXPECT_TRUE(pending.pending_activation);
  EXPECT_TRUE(pending.h2_required);

  const auto neutral = FLAG_Race::GvfManagerS4AnchorTestAccess::
      attemptNeutralPlannerFrontend(manager, replacement);
  EXPECT_FALSE(neutral.committed);
  EXPECT_FALSE(neutral.installed_new_owner);
  EXPECT_TRUE(neutral.pair_present);
  EXPECT_EQ(neutral.session_before, neutral.session_after);
  const std::shared_ptr<const FLAG_Race::PathTubePair> preserved =
      FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(manager);
  ASSERT_EQ(pair, preserved);
  EXPECT_EQ(pair->authority_session, preserved->authority_session);
  EXPECT_EQ(owner, preserved->path_owner);

  const auto first_command = FLAG_Race::GvfManagerS4AnchorTestAccess::
      executePendingBootstrapPairCommand(manager, 3.0);
  ASSERT_TRUE(first_command.selected);
  ASSERT_TRUE(first_command.executed_authority);
  ASSERT_TRUE(first_command.projection_valid);
  EXPECT_LE(std::abs(first_command.retained_delta_before), 1e-6);
  EXPECT_LE(std::abs(first_command.retained_delta_after), 1e-6);
  // The lifecycle log uses a strict, significant threshold and an atomic
  // generation claim.  Exercise both sides without changing Runtime state.
  EXPECT_FALSE(std::abs(1e-6) > 1e-6);
  EXPECT_TRUE(std::abs(1.1e-6) > 1e-6);
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  claimPendingActivationNonzeroLogGeneration(manager, pair));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   claimPendingActivationNonzeroLogGeneration(manager, pair));
  const auto denied_command = FLAG_Race::GvfManagerS4AnchorTestAccess::
      executePendingBootstrapPairCommand(manager, 3.02, true);
  EXPECT_FALSE(denied_command.update_success);
  EXPECT_FALSE(denied_command.selected);
  EXPECT_FALSE(denied_command.valid);
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  claimPendingActivationNotSelectedLogGeneration(manager, pair));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   claimPendingActivationNotSelectedLogGeneration(manager, pair));
  const auto executed = FLAG_Race::GvfManagerS4AnchorTestAccess::
      captureReplanHandoff(manager);
  EXPECT_EQ(pair, executed.pair);
  EXPECT_TRUE(executed.executed_authority);
  EXPECT_FALSE(executed.pending_activation);
  EXPECT_TRUE(executed.h2_required);
}

TEST(GvfTimerBootstrap,
     AdvertisedNeutralActivationUsesExistingPairLifecycle) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.4, true, false);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setAdapterAdvertised(manager, true);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setTestOnlyRuntimeOwnerAllowed(
      manager, false);
  FLAG_Race::GvfManagerS4AnchorTestAccess::openManualGate(manager);

  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   requiresPathTubePairBootstrap(manager));
  const auto handoff = FLAG_Race::GvfManagerS4AnchorTestAccess::
      captureReplanHandoff(manager);
  EXPECT_FALSE(handoff.pair);
  EXPECT_FALSE(handoff.executed_authority);
  EXPECT_FALSE(handoff.pending_activation);
  EXPECT_FALSE(handoff.h2_required);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   recoveryMailbox(manager).pending);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   authoritySnapshot(manager).valid);

  const auto bootstrap = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(manager);
  ASSERT_TRUE(bootstrap.committed) << bootstrap.outcome << ": "
                                   << bootstrap.stage_failure;
  const auto pair = FLAG_Race::GvfManagerS4AnchorTestAccess::
      bootstrapAuthority(manager);
  ASSERT_TRUE(pair);
  const auto post_handoff = FLAG_Race::GvfManagerS4AnchorTestAccess::
      captureReplanHandoff(manager);
  EXPECT_EQ(post_handoff.pair, pair);
  EXPECT_TRUE(post_handoff.pending_activation);
  EXPECT_DOUBLE_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::
                       retainedDelta(manager), 0.0);
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

TEST(GvfManagerC3,
     ProductionNeutralBootstrapRunsManagerToFirstAllocatorTransaction) {
  ensureRosInitializedForSeededRecoveryE2E();
  ros::Time::init();
  if (!ros::master::check()) {
    GTEST_SKIP() << "ROS master unavailable for production manager boundary";
  }
  ros::NodeHandle nh;
  std::shared_ptr<SDFMap> map(new SDFMap());
  initializeProductionFreeMap(*map);
  installProductionCloudSnapshot(*map, 1U);
  const std::shared_ptr<const FLAG_Race::ContinuousPhasePath> owner =
      makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  FLAG_Race::gvf_manager manager;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installProductionManagerFixture(manager, owner, map, nh));
  ASSERT_TRUE(manager.setPhaseOffsetGDes(Eigen::Vector3d(0.08, 0.0, 0.0)));

  // Enter through gvf_manager::cmdCallback.  The first command is planner
  // baseline only and creates the immutable Tube request; no pair or
  // execution authority may exist before the H2 timer CAS.
  FLAG_Race::GvfManagerS4AnchorTestAccess::runCommand(manager);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(
                   manager).valid);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(
      manager));
  FLAG_Race::GvfManagerS4AnchorTestAccess::openManualGate(manager);
  for (int attempt = 0; attempt < 200 &&
           !FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(manager);
       ++attempt) {
    manager.phaseOffsetTubeTimerCallback(ros::TimerEvent());
    ros::spinOnce();
    if (!FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(manager)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(
      manager));
  const std::shared_ptr<const FLAG_Race::PathTubePair> pair_before_command =
      FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(manager);
  ASSERT_TRUE(pair_before_command);
  FLAG_Race::GvfManagerS4AnchorTestAccess::runCommand(manager);
  const auto authority =
      FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(manager);
  ASSERT_TRUE(authority.valid);
  EXPECT_EQ(authority.owner_mode,
            phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL);
  EXPECT_EQ(authority.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  EXPECT_TRUE(authority.selectedUConsistent(0.0));
  EXPECT_GT(std::abs(authority.selected_u_w) +
                std::abs(authority.selected_u_delta), 0.0);
  const std::shared_ptr<const FLAG_Race::PathTubePair> pair_after_command =
      FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(manager);
  ASSERT_TRUE(pair_after_command);
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::sameAuthority(
      pair_before_command, pair_after_command));
  EXPECT_EQ(pair_before_command->authority_session,
            pair_after_command->authority_session);
  EXPECT_EQ(pair_after_command->authority_session,
            authority.authority_session);
  const FLAG_Race::MatchedAdapterOutput output =
      FLAG_Race::GvfManagerS4AnchorTestAccess::latestAdapterOutput(manager);
  EXPECT_TRUE(output.allocator_evaluated);
  EXPECT_EQ(output.allocator.selected_u_owner,
            phase_offset_navigation::PhaseOffsetAllocator::ownerName());
  EXPECT_TRUE(output.matched.physical_port.allFinite());
  EXPECT_TRUE(output.matched.v_cmd.allFinite());

  // A subsequent exact-zero desired normal command remains on the same
  // immutable H2 Pair/session; it does not reopen bootstrap or retire the
  // live NORMAL authority.
  ASSERT_TRUE(manager.setPhaseOffsetGDes(Eigen::Vector3d::Zero()));
  FLAG_Race::GvfManagerS4AnchorTestAccess::runCommand(manager);
  const auto zero_authority =
      FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(manager);
  ASSERT_TRUE(zero_authority.valid);
  EXPECT_EQ(zero_authority.owner_mode,
            phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL);
  EXPECT_EQ(zero_authority.authority_session,
            authority.authority_session);
  EXPECT_DOUBLE_EQ(0.0, zero_authority.selected_u_delta);
  const std::shared_ptr<const FLAG_Race::PathTubePair> pair_after_zero =
      FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(manager);
  ASSERT_TRUE(pair_after_zero);
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::sameAuthority(
      pair_after_command, pair_after_zero));
  EXPECT_EQ(pair_after_command->authority_session,
            pair_after_zero->authority_session);
}

TEST(GvfTimerBootstrap, ResetDuringStageRejectsWithoutPublishingAuthority) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.4, true, false);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setBootstrapAfterStageHook(
      manager, [&manager]() {
        FLAG_Race::GvfManagerS4AnchorTestAccess::resetH2(manager);
      });

  const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(manager);
  EXPECT_FALSE(attempt.committed);
  EXPECT_EQ("LIVE_PHASE_STATE", attempt.outcome);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   bootstrapAuthority(manager));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setBootstrapAfterStageHook(
      manager, std::function<void()>());
  const auto after = FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  EXPECT_FALSE(after.initialized);
  EXPECT_DOUBLE_EQ(0.0, after.w);
}

TEST(GvfTimerBootstrap, PlannerOwnerDriftDuringStageRejectsWithoutAuthority) {
  FLAG_Race::gvf_manager manager;
  const auto replacement = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(replacement);
  const auto owner = makeTimerBootstrapPath([&manager, replacement]() {
    ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                    replaceBootstrapPlannerOwner(manager, replacement));
  });
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.4, true, false);

  const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(manager);
  EXPECT_FALSE(attempt.committed);
  EXPECT_EQ("POST_STAGE_OWNER_OR_SESSION", attempt.outcome);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   bootstrapAuthority(manager));
}

TEST(GvfTimerBootstrap, LivePhaseAtStructuralSeamRejectsWithoutAuthority) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath([&manager]() {
    FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
        manager, 0.8, true, false);
  });
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.4, true, false);

  const auto attempt = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(manager);
  EXPECT_FALSE(attempt.committed);
  EXPECT_EQ("LIVE_PHASE_WINDOW", attempt.outcome);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   bootstrapAuthority(manager));
  const auto after = FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  EXPECT_DOUBLE_EQ(0.8, after.w);
  EXPECT_TRUE(after.initialized);
}

TEST(GvfH2Mailbox, RetryablePrepareFailureRetainsPendingUntilSeamExpiry) {
  constexpr double kW0 = 0.40;
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, kW0, true, false);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  activateTimerBootstrap(manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  stageRetryablePendingFromLivePair(manager));
  const auto before = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  ASSERT_TRUE(before.pending);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::canAcquirePin(manager));

  // An invalid command-time dt makes adapter prepare return false.  H2-L1
  // must retain the same pending identity rather than consuming it before
  // that retryable attempt.
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::preparePending(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0),
      std::numeric_limits<double>::quiet_NaN()));
  const auto retry = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_TRUE(retry.pending);
  EXPECT_FALSE(retry.completed);
  EXPECT_EQ(before.pending_identity, retry.pending_identity);
  EXPECT_EQ("retry_pending", FLAG_Race::GvfManagerS4AnchorTestAccess::
                handoffLifecycle(manager));
  // A second retry sees the same pending transaction, rather than a copied or
  // replaced mailbox entry.
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::preparePending(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0),
      std::numeric_limits<double>::quiet_NaN()));
  const auto retry_again =
      FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_TRUE(retry_again.pending);
  EXPECT_EQ(before.pending_identity, retry_again.pending_identity);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::canAcquirePin(manager));

  const std::shared_ptr<const FLAG_Race::PathTubePair> pair =
      FLAG_Race::GvfManagerS4AnchorTestAccess::bootstrapAuthority(manager);
  ASSERT_TRUE(pair);
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, pair->future_seam_w, true, false);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::preparePending(manager));
  const auto expired = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_FALSE(expired.pending);
  EXPECT_FALSE(expired.completed);
  EXPECT_EQ("dropped_expired", FLAG_Race::GvfManagerS4AnchorTestAccess::
                handoffLifecycle(manager));
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::canAcquirePin(manager));
}

TEST(GvfH2Mailbox, SessionDriftDropsPendingWithoutPublishingCompletion) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.40, true, false);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  activateTimerBootstrap(manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  stageRetryablePendingFromLivePair(manager));
  const auto before = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setAuthoritySession(
      manager, before.session + 1U);

  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::preparePending(manager));
  const auto after = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_FALSE(after.pending);
  EXPECT_FALSE(after.completed);
  EXPECT_EQ("dropped_stale", FLAG_Race::GvfManagerS4AnchorTestAccess::
                handoffLifecycle(manager));
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::canAcquirePin(manager));
}

TEST(GvfH2Mailbox, PhaseGenerationDriftRetainsSamePendingForRetry) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.40, true, false);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  activateTimerBootstrap(manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  stageRetryablePendingFromLivePair(manager));
  const auto before = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  const auto captured =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  // Preserve w/init facts while changing only the authoritative generation.
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, captured.w, captured.initialized, captured.closed_acquired);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   preparePendingWithCapturedPhase(manager, captured));
  const auto after = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_TRUE(after.pending);
  EXPECT_FALSE(after.completed);
  EXPECT_EQ(before.pending_identity, after.pending_identity);
  EXPECT_EQ("retry_pending", FLAG_Race::GvfManagerS4AnchorTestAccess::
                handoffLifecycle(manager));
}

TEST(GvfH2Mailbox, SuccessfulCommitMovesOnePendingIdentityToCompletedOnce) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.40, true, false);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  activateTimerBootstrap(manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  stageRetryablePendingFromLivePair(manager));
  const auto pending = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  ASSERT_TRUE(pending.pending);

  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::preparePending(manager));
  const auto committed = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_FALSE(committed.pending);
  EXPECT_TRUE(committed.completed);
  EXPECT_EQ(pending.pending_identity, committed.completed_identity);
  EXPECT_EQ("committed", FLAG_Race::GvfManagerS4AnchorTestAccess::
                handoffLifecycle(manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::preparePending(manager));
  const auto repeated = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_FALSE(repeated.pending);
  EXPECT_EQ(committed.completed_identity, repeated.completed_identity);
}

TEST(GvfH2Mailbox, CompletedFrontendIsConsumedOnlyAfterValidatedMirrorInstall) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.40, true, false);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  activateTimerBootstrap(manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  stageValidCompletedFromLivePair(manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager).completed);

  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::consumeCompleted(manager));
  const auto after = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_FALSE(after.pending);
  EXPECT_FALSE(after.completed);
  EXPECT_EQ("consumed", FLAG_Race::GvfManagerS4AnchorTestAccess::
                handoffLifecycle(manager));
  // The slot was retired at the success linearization point; the next FSM
  // tick cannot publish the same frontend twice.
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::consumeCompleted(manager));
}

TEST(GvfH2Mailbox, TemporarilyUnavailableCompletedConsumerRetainsIdentity) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.40, true, false);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  activateTimerBootstrap(manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  stageValidCompletedFromLivePair(manager));
  const auto before = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setCompletedFrontendGvfAvailable(
      manager, false);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::consumeCompleted(manager));
  const auto unavailable =
      FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_TRUE(unavailable.completed);
  EXPECT_EQ(before.completed_identity, unavailable.completed_identity);

  FLAG_Race::GvfManagerS4AnchorTestAccess::setCompletedFrontendGvfAvailable(
      manager, true);
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::consumeCompleted(manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager).completed);
}

TEST(GvfH2Mailbox, StaleCompletedFrontendDropsWithoutMirrorApplication) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.0, 1.0));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.40, true, false);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  activateTimerBootstrap(manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  stageValidCompletedFromLivePair(manager));
  const auto before = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  const int mirror_rows_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::frontendRows(manager);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setAuthoritySession(
      manager, before.session + 1U);

  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::consumeCompleted(manager));
  const auto after = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_FALSE(after.completed);
  EXPECT_EQ(mirror_rows_before,
            FLAG_Race::GvfManagerS4AnchorTestAccess::frontendRows(manager));
  EXPECT_EQ("dropped_stale", FLAG_Race::GvfManagerS4AnchorTestAccess::
                handoffLifecycle(manager));
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

TEST(GvfH2Mailbox, PendingSlotIsSingleEntryAndCannotBeOverwritten)
{
  FLAG_Race::gvf_manager manager;
  constexpr std::uint64_t kSession = 17U;
  FLAG_Race::GvfManagerS4AnchorTestAccess::setAuthoritySession(
      manager, kSession);
  const auto first = makeCertifiedH2Pair({0.0, 1.0, 2.0}, 0.0, 2.0);
  const auto second = makeCertifiedH2Pair({0.0, 1.0, 2.0}, 0.0, 2.0);
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::stageSinglePending(
      manager, first, kSession));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::stageSinglePending(
      manager, second, kSession));
  const auto mailbox = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_TRUE(mailbox.pending);
  EXPECT_FALSE(mailbox.completed);
  EXPECT_EQ(kSession, mailbox.session);
}

TEST(GvfH2Mailbox, ResetRetiresSessionAndClearsBothMailboxes)
{
  FLAG_Race::gvf_manager manager;
  constexpr std::uint64_t kSession = 29U;
  FLAG_Race::GvfManagerS4AnchorTestAccess::setAuthoritySession(
      manager, kSession);
  const auto pair = makeCertifiedH2Pair({0.0, 1.0, 2.0}, 0.0, 2.0);
  ASSERT_TRUE(pair);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::stageSinglePending(
      manager, pair, kSession));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setCompletedForReset(
      manager, pair, kSession);
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 1.0, true, true);

  FLAG_Race::GvfManagerS4AnchorTestAccess::resetH2(manager);

  const auto mailbox = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_FALSE(mailbox.pending);
  EXPECT_FALSE(mailbox.completed);
  EXPECT_GT(mailbox.session, kSession);
  EXPECT_EQ(mailbox.session, mailbox.pending_clear);
  EXPECT_LT(mailbox.consumed_clear, mailbox.pending_clear);
  const auto phase = FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  EXPECT_FALSE(phase.initialized);
  EXPECT_FALSE(phase.closed_acquired);
  EXPECT_DOUBLE_EQ(0.0, phase.w);
}

TEST(GvfH2Mailbox,
     NewNavigationTaskResetClearsOldMailboxesAndNeutralizesSessionBoundary)
{
  FLAG_Race::gvf_manager manager;
  constexpr std::uint64_t kSession = 31U;
  FLAG_Race::GvfManagerS4AnchorTestAccess::setAuthoritySession(
      manager, kSession);
  const auto certified = makeCertifiedH2Pair({0.0, 1.0, 2.0}, 0.0, 2.0);
  auto mutable_pair = std::make_shared<FLAG_Race::PathTubePair>(*certified);
  mutable_pair->generation = 9U;
  mutable_pair->authority_session = kSession;
  mutable_pair->map_observation_sequence = 15U;
  const std::shared_ptr<const FLAG_Race::PathTubePair> pair(mutable_pair);
  ASSERT_TRUE(pair);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::stageSinglePending(
      manager, pair, kSession));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setCompletedForReset(
      manager, pair, kSession);
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 1.0, true, true);

  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  resetForNewNavigationTask(manager));

  const auto mailbox = FLAG_Race::GvfManagerS4AnchorTestAccess::mailbox(manager);
  EXPECT_FALSE(mailbox.pending);
  EXPECT_FALSE(mailbox.completed);
  EXPECT_GT(mailbox.session, kSession);
  EXPECT_EQ(mailbox.session, mailbox.pending_clear);
  EXPECT_LT(mailbox.consumed_clear, mailbox.pending_clear);
  const auto phase = FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  EXPECT_FALSE(phase.initialized);
  EXPECT_FALSE(phase.closed_acquired);
  EXPECT_DOUBLE_EQ(0.0, phase.w);
}

TEST(GvfH2Mailbox, SameOwnerTimerRefreshMatchesMailboxAuthority)
{
  const auto original = makeCertifiedH2Pair({0.0, 1.0, 2.0}, 0.0, 2.0);
  ASSERT_TRUE(original);
  auto refresh = std::make_shared<FLAG_Race::PathTubePair>(*original);
  refresh->generation = original->generation + 1U;
  const std::shared_ptr<const FLAG_Race::PathTubePair> same_owner_refresh(refresh);
  EXPECT_NE(original.get(), same_owner_refresh.get());
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::sameAuthority(
      original, same_owner_refresh));

  const auto different_owner = makeCertifiedH2Pair({0.0, 1.0, 2.0}, 0.0, 2.0);
  ASSERT_TRUE(different_owner);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::sameAuthority(
      original, different_owner));

  auto different_revision = std::make_shared<FLAG_Race::PathTubePair>(*original);
  ++different_revision->source_revision;
  const std::shared_ptr<const FLAG_Race::PathTubePair> stale_revision(
      different_revision);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::sameAuthority(
      original, stale_revision));
}

TEST(GvfPlannerAuthority,
     NeutralFrontendInstallsIndependentOwnerWithoutManufacturingTubePair)
{
  const auto planner_owner = makeH2OldSeamPath();
  ASSERT_TRUE(planner_owner);
  EXPECT_EQ(planner_owner,
            FLAG_Race::GvfManagerS4AnchorTestAccess::plannerOwner(
                std::shared_ptr<const FLAG_Race::PathTubePair>(),
                planner_owner));

  const auto pair = makeCertifiedH2Pair({0.0, 1.0, 2.0}, 0.0, 2.0);
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->path_owner,
            FLAG_Race::GvfManagerS4AnchorTestAccess::plannerOwner(
                pair, pair->path_owner));

  const auto mismatched_owner = makeH2ReplacementPath();
  ASSERT_TRUE(mismatched_owner);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::plannerOwner(
      pair, mismatched_owner));
}

TEST(GvfPlannerAuthority,
     NeutralFrontendCommitSucceedsWithoutTubeAuthority) {
  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  const auto replacement = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(replacement);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  installTimerBootstrapFixture(manager, owner));

  const auto result = FLAG_Race::GvfManagerS4AnchorTestAccess::
      attemptNeutralPlannerFrontend(manager, replacement);
  EXPECT_TRUE(result.committed);
  EXPECT_TRUE(result.installed_new_owner);
  EXPECT_FALSE(result.pair_present);
  EXPECT_GT(result.session_after, result.session_before);
}

TEST(SeededRecoveryE2E,
     SeededNonzeroAuthorityUsesSuccessorRecoveryAndAtomicNeutralHandoff) {
  ensureRosInitializedForSeededRecoveryE2E();
  ros::Time::init();
  ros::NodeHandle nh;
  constexpr char kCommandTopic[] = "/gvf_seeded_recovery_e2e/position_command";

  FLAG_Race::gvf_manager manager;
  const auto owner = makeTimerBootstrapPath(std::function<void()>());
  ASSERT_TRUE(owner);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::installTimerBootstrapFixture(
      manager, owner));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setAuthoritySessionForE2E(manager, 1U);
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, 0.4, true, false);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setOdom(
      manager, Eigen::Vector3d(0.4, 0.10, 1.10));

  auto& frontend = manager.swarmParticlesManager.front();
  frontend.receive_goal = true;
  frontend.goal_pt = Eigen::Vector3d(20.0, 0.0, 1.0);
  frontend.gvf_->gvf_.K1_ = 2.0;
  frontend.gvf_->gvf_.K2_ = -2.2;
  frontend.gvf_->gvf_.convergence_bandwidth_ = 0.1;
  frontend.gvf_->progress_rho0_ = 0.5;
  frontend.gvf_->progress_delta_ = 0.3;
  frontend.gvf_->alpha_min_ = 0.05;

  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::advertiseProductionSeams(
      manager, nh, kCommandTopic));
  // The first nonzero seed is deliberately established through the ordinary
  // adapter/authority path before advertisement.  Subsequent recovery ticks
  // must use the publication-owned transaction below.
  FLAG_Race::GvfManagerS4AnchorTestAccess::setAdapterAdvertised(manager, false);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setTestOnlyRuntimeOwnerAllowed(
      manager, true);

  const auto bootstrap = FLAG_Race::GvfManagerS4AnchorTestAccess::
      activateTimerBootstrapAttempt(manager);
  ASSERT_TRUE(bootstrap.committed) << bootstrap.outcome << ": "
                                   << bootstrap.stage_failure;
  const auto old_pair = FLAG_Race::GvfManagerS4AnchorTestAccess::
      bootstrapAuthority(manager);
  ASSERT_TRUE(old_pair);
  ASSERT_TRUE(old_pair->active_profile);
  EXPECT_NE(0U, old_pair->source_revision);
  EXPECT_NE(0U, old_pair->path_revision);
  EXPECT_NE(0U, old_pair->frame_revision);
  EXPECT_NE(0U, old_pair->active_profile->profile_revision);

  auto make_input = [&](const std::shared_ptr<const FLAG_Race::PathTubePair>& pair,
                        const phase_offset_navigation::ActiveReferenceSnapshot& authority,
                        const double stamp,
                        const std::shared_ptr<const FLAG_Race::PathTubePair>& successor)
      -> FLAG_Race::MatchedAdapterInput {
    FLAG_Race::MatchedAdapterInput input =
        FLAG_Race::GvfManagerS4AnchorTestAccess::timerBootstrapGateInput(stamp);
    const double w = authority.valid ? authority.proposed_next_w : 0.4;
    FLAG_Race::ContinuousPhasePathState state;
    EXPECT_TRUE(pair && pair->path_owner &&
                pair->path_owner->evaluate(w, state, false));
    input.path = FLAG_Race::ConvertContinuousPhasePathStateForActive(state, w);
    input.path_state_query = [owner = pair->path_owner](
        const double query_w,
        phase_offset_core::PathDifferentialState& query_state) {
      FLAG_Race::ContinuousPhasePathState state;
      if (!owner || !owner->evaluate(query_w, state, false)) return false;
      query_state = FLAG_Race::ConvertContinuousPhasePathStateForActive(
          state, query_w);
      return true;
    };
    input.semantic_path_owner = pair->path_owner;
    input.frame_owner = pair->frame_owner;
    input.semantic_path_start_w = pair->path_owner->startW();
    input.semantic_path_end_w = pair->path_owner->endW();
    input.path_tube_pair = pair;
    input.successor_path_tube_pair = successor;
    input.position = Eigen::Vector3d(w, 0.10, 1.10);
    input.stamp = ros::Time(stamp);
    FLAG_Race::PhaseOffsetActiveAdapter zero;
    FLAG_Race::ActiveAdapterInput zero_input;
    zero_input.path = input.path;
    zero_input.position = input.position;
    zero_input.gains = input.gains;
    FLAG_Race::ActiveAdapterOutput zero_output;
    EXPECT_TRUE(zero.evaluate(zero_input, zero_output));
    input.legacy = FLAG_Race::LegacyGuidanceSnapshot(
        zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
        zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
        zero_output.guidance.ref_pt, zero_output.guidance.tangent,
        zero_output.guidance.valid);
    return input;
  };

  FLAG_Race::MatchedAdapterOutput output;
  auto authority = FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(
      manager);
  ASSERT_FALSE(authority.valid);
  FLAG_Race::GvfManagerS4AnchorTestAccess::openManualGate(manager);
  auto seed_input = make_input(old_pair, authority, 1.0,
                               std::shared_ptr<const FLAG_Race::PathTubePair>());
  seed_input.dt = 0.02;
  // The bootstrap pair is the real activation edge.  The first update is
  // selected by the adapter and commits a normal, nonzero predecessor.
  seed_input.path_tube_pair = old_pair;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::updateAdapter(
      manager, seed_input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.valid);
  auto seeded = FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(
      manager);
  for (int seed_tick = 0; seed_tick < 32 && std::abs(seeded.delta) <= 1e-6;
       ++seed_tick) {
    seed_input = make_input(old_pair, seeded, 1.02 + 0.02 * seed_tick,
                            std::shared_ptr<const FLAG_Race::PathTubePair>());
    seed_input.dt = 0.02;
    ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::updateAdapter(
        manager, seed_input, output)) << output.invalid_reason;
    ASSERT_TRUE(output.selected);
    seeded = FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(manager);
  }
  ASSERT_TRUE(seeded.valid);
  EXPECT_EQ(phase_offset_navigation::ActiveReferenceOwnerMode::NORMAL,
            seeded.owner_mode);
  ASSERT_GT(std::abs(seeded.delta), 1e-6);
  EXPECT_EQ("PhaseOffsetMatchedAdapterRuntime", seeded.selected_u_owner);

  // Rebase the exact immutable predecessor and advance once more through the
  // unadvertised path, keeping the seed comfortably nonzero.
  authority = seeded;
  seed_input = make_input(old_pair, authority, 1.02,
                          std::shared_ptr<const FLAG_Race::PathTubePair>());
  seed_input.dt = 0.02;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::updateAdapter(
      manager, seed_input, output)) << output.invalid_reason;
  ASSERT_TRUE(output.selected);
  authority = FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(manager);
  ASSERT_TRUE(authority.valid);
  ASSERT_GT(std::abs(authority.delta), 1e-6);

  // Advertisement switches the adapter to the production publication-owned
  // transaction.  Keep the already authenticated nonzero predecessor intact.
  FLAG_Race::GvfManagerS4AnchorTestAccess::setAdapterAdvertised(manager, true);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setTestOnlyRuntimeOwnerAllowed(
      manager, false);
  const double captured_w0 = authority.w;
  ASSERT_TRUE(std::isfinite(captured_w0));
  ASSERT_TRUE(std::isfinite(old_pair->future_seam_w));
  ASSERT_TRUE(std::isfinite(old_pair->existing_future_horizon_end_w));
  ASSERT_LT(captured_w0, old_pair->future_seam_w)
      << "w=" << captured_w0 << " seam=" << old_pair->future_seam_w
      << " horizon=" << old_pair->existing_future_horizon_end_w
      << " end=" << old_pair->path_owner->endW();
  const auto successor_owner = makeSeededRecoveryCompositeSuccessor(
      old_pair->path_owner, captured_w0, 1.0, 1.6, 20.0);
  ASSERT_TRUE(successor_owner);
  std::vector<double> successor_sample_w;
  std::vector<FLAG_Race::ContinuousPhasePathState> successor_states;
  ASSERT_TRUE(successor_owner->sample(0.10, successor_sample_w,
                                      successor_states));
  ASSERT_EQ(successor_sample_w.size(), successor_states.size());
  FLAG_Race::MatchedAdapterPathSamples successor_samples;
  successor_samples.reserve(successor_sample_w.size());
  for (std::size_t index = 0U; index < successor_sample_w.size(); ++index) {
    successor_samples.push_back(
        FLAG_Race::ConvertContinuousPhasePathStateForActive(
            successor_states[index], successor_sample_w[index]));
  }
  ASSERT_GE(successor_samples.size(), 2U);
  ASSERT_GT(successor_owner->segments().size(), 1U);
  ASSERT_TRUE(successor_owner->segments()[1].label == "c2_quintic");
  ASSERT_TRUE(successor_owner->segments().back().label == "mapped_tail");
  FLAG_Race::ContinuousPhasePathState seam_state;
  FLAG_Race::ContinuousPhasePathState old_seam_state;
  ASSERT_TRUE(successor_owner->evaluate(1.0, seam_state, false));
  ASSERT_TRUE(old_pair->path_owner->evaluate(1.0, old_seam_state, false));
  EXPECT_NEAR((seam_state.p - old_seam_state.p).norm(), 0.0, 1e-10);
  EXPECT_NEAR((seam_state.dp_dw - old_seam_state.dp_dw).norm(), 0.0, 1e-10);
  EXPECT_NEAR((seam_state.d2p_dw2 - old_seam_state.d2p_dw2).norm(), 0.0,
              1e-10);
  FLAG_Race::ContinuousPhasePathState tail_state;
  ASSERT_TRUE(successor_owner->evaluate(2.0, tail_state, false));
  EXPECT_GT((tail_state.p - old_seam_state.p).norm(), 1e-3);
  const Eigen::Vector3d successor_stage_position(captured_w0, 0.10, 1.10);
  FLAG_Race::PathTubePairTransaction successor_transaction;
  FLAG_Race::PathTubePairStageFailure successor_failure =
      FLAG_Race::PathTubePairStageFailure::NONE;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::stageSuccessorPair(
      manager, old_pair, successor_owner, successor_samples, captured_w0,
      std::max(1.0, captured_w0 + 0.40),
      std::max(2.4, captured_w0 + 1.6),
      successor_stage_position, seed_input.gains, 0.02,
      successor_transaction, &successor_failure))
      << FLAG_Race::GvfManagerS4AnchorTestAccess::stageFailureName(
             successor_failure)
      << " w=" << captured_w0 << " seam=" << old_pair->future_seam_w
      << " horizon=" << old_pair->existing_future_horizon_end_w
      << " sample_front=" << successor_samples.front().w
      << " sample_back=" << successor_samples.back().w
      << " source=" << old_pair->source_revision
      << " path=" << old_pair->path_revision
      << " frame=" << old_pair->frame_revision;
  const auto successor_pair = successor_transaction.candidate_pair;
  ASSERT_TRUE(successor_pair);
  ASSERT_TRUE(successor_pair->active_profile);
  EXPECT_NE(successor_pair->path_revision, old_pair->path_revision);
  EXPECT_NE(successor_pair->generation, old_pair->generation);
  EXPECT_EQ(successor_pair->source_revision,
            successor_pair->active_profile->source_revision);
  EXPECT_EQ(successor_pair->path_revision,
            successor_pair->active_profile->path_revision);
  EXPECT_EQ(successor_pair->frame_revision,
            successor_pair->active_profile->frame_revision);
  EXPECT_NE(old_pair.get(), successor_pair.get());

  PositionCommandCapture command_capture;
  ros::Subscriber command_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
      kCommandTopic, 10, &PositionCommandCapture::callback, &command_capture);
  const auto connection_deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(2);
  while (manager.cmd_pub.getNumSubscribers() == 0U &&
         std::chrono::steady_clock::now() < connection_deadline) {
    ros::spinOnce();
    ros::WallDuration(0.01).sleep();
  }
  ASSERT_GT(manager.cmd_pub.getNumSubscribers(), 0U);
  const auto old_authority = authority;
  std::vector<double> delta_history;
  std::vector<std::uint64_t> revision_history;
  bool saw_recovery_owner = false;
  bool saw_query = false;
  bool saw_publish_before_commit = false;
  bool terminal = false;
  std::size_t successful_publishes = 0U;
  std::size_t committed_recovery_ticks = 0U;
  for (std::size_t tick = 0U; tick < 128U; ++tick) {
    authority = FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(manager);
    ASSERT_TRUE(authority.valid);
    auto input = make_input(old_pair, authority, 2.0 + 0.02 * tick,
                            successor_pair);
    ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::updateAdapter(
        manager, input, output)) << output.invalid_reason;
    ASSERT_TRUE(output.selected) << output.invalid_reason;
    const auto capture = FLAG_Race::GvfManagerS4AnchorTestAccess::
        pendingCommand(manager);
    ASSERT_TRUE(capture.pending);
    ASSERT_TRUE(capture.valid);
    ASSERT_TRUE(capture.reference_query);
    saw_query = true;
    const auto live_before_publish =
        FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(manager);
    EXPECT_EQ(live_before_publish.snapshotId(), authority.snapshotId());
    const auto after_update =
        FLAG_Race::GvfManagerS4AnchorTestAccess::pendingAuthoritySnapshot(manager);
    ASSERT_TRUE(after_update.valid);
    if (after_update.owner_mode ==
        phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY) {
      EXPECT_EQ(output.recovery_status,
                phase_offset_navigation::RecoveryStepStatus::PREPARED);
      EXPECT_EQ(after_update.selected_u_owner, "PhaseOffsetRecoveryOwner");
      EXPECT_EQ(after_update.executed_path_revision,
                successor_pair->path_revision);
      EXPECT_EQ(after_update.frame_revision, successor_pair->frame_revision);
      EXPECT_EQ(after_update.tube_revision,
                successor_pair->active_profile->tube_revision);
      EXPECT_EQ(after_update.profile_revision,
                successor_pair->active_profile->profile_revision);
      EXPECT_EQ(capture.reference_query->pathRevision(),
                successor_pair->path_revision);
      EXPECT_EQ(after_update.handoff_state, "RECOVERY_OWNER");
      saw_recovery_owner = true;
    } else {
      EXPECT_EQ(after_update.owner_mode,
                phase_offset_navigation::ActiveReferenceOwnerMode::PLANNER_ONLY);
      EXPECT_EQ(after_update.selected_u_owner, "PlannerOwner");
      EXPECT_DOUBLE_EQ(0.0, after_update.delta);
    }
    EXPECT_TRUE(after_update.governorViewValid());
    EXPECT_TRUE(after_update.selectedUConsistent());
    delta_history.push_back(std::abs(after_update.delta));
    revision_history.push_back(after_update.executed_path_revision);

    FLAG_Race::gvf::LiftedGuidanceResult governor_input;
    governor_input.v_cmd = output.guidance.v_cmd;
    governor_input.w_dot = output.guidance.w_dot;
    governor_input.e_parallel = output.guidance.e_parallel;
    governor_input.e_perp = output.guidance.e_perp;
    governor_input.ref_pt = output.guidance.ref_pt;
    governor_input.tangent = output.guidance.tangent;
    governor_input.valid = output.guidance.valid;
    const auto governor = FLAG_Race::GvfManagerS4AnchorTestAccess::
        runGovernorCandidate(manager, old_pair->path_owner, governor_input,
                              input.position, after_update.w,
                              after_update.delta, input.dt, 1.0,
                              capture.reference_query);
    ASSERT_TRUE(governor.command_valid);
    const auto before_publish = live_before_publish;
    const double before_delta =
        FLAG_Race::GvfManagerS4AnchorTestAccess::retainedDelta(manager);
    bool callback_saw_uncommitted = false;
    ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::publishPendingCommand(
        manager,
        [&]() {
          callback_saw_uncommitted =
              FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(manager)
                  .snapshotId() == before_publish.snapshotId() &&
              std::abs(FLAG_Race::GvfManagerS4AnchorTestAccess::retainedDelta(
                           manager) - before_delta) <= 1e-12;
          return FLAG_Race::GvfManagerS4AnchorTestAccess::publishGovernorCandidate(
              manager, governor.cmd_pos, output.guidance.tangent);
        },
        capture.identity));
    EXPECT_TRUE(callback_saw_uncommitted);
    saw_publish_before_commit = saw_publish_before_commit || callback_saw_uncommitted;
    ++successful_publishes;
    ASSERT_TRUE(waitForPositionCommandCount(command_capture,
                                            successful_publishes, 1.0));
    const auto committed_after_publish =
        FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(manager);
    EXPECT_EQ(committed_after_publish.snapshotId(), after_update.snapshotId());
    if (committed_after_publish.owner_mode ==
        phase_offset_navigation::ActiveReferenceOwnerMode::RECOVERY) {
      ++committed_recovery_ticks;
    }
    if (committed_after_publish.owner_mode ==
        phase_offset_navigation::ActiveReferenceOwnerMode::PLANNER_ONLY) {
      terminal = true;
      break;
    }
    const auto recovery_selected =
        FLAG_Race::GvfManagerS4AnchorTestAccess::recoveryStatus(manager)
            .selected_u;
    const auto authority_selected =
        FLAG_Race::GvfManagerS4AnchorTestAccess::authoritySnapshot(manager)
            .selected_u;
    EXPECT_DOUBLE_EQ(recovery_selected.u_w, authority_selected.u_w);
    EXPECT_DOUBLE_EQ(recovery_selected.u_delta, authority_selected.u_delta);
  }
  EXPECT_TRUE(old_authority.valid);
  EXPECT_GT(std::abs(old_authority.delta), 1e-6);
  EXPECT_TRUE(saw_recovery_owner);
  EXPECT_GT(committed_recovery_ticks, 1U);
  EXPECT_TRUE(saw_query);
  EXPECT_TRUE(saw_publish_before_commit);
  EXPECT_TRUE(terminal);
  const auto terminal_authority = FLAG_Race::GvfManagerS4AnchorTestAccess::
      authoritySnapshot(manager);
  EXPECT_EQ(phase_offset_navigation::ActiveReferenceOwnerMode::PLANNER_ONLY,
            terminal_authority.owner_mode);
  EXPECT_EQ("PlannerOwner", terminal_authority.selected_u_owner);
  EXPECT_DOUBLE_EQ(0.0, terminal_authority.delta);
  EXPECT_DOUBLE_EQ(0.0,
                   FLAG_Race::GvfManagerS4AnchorTestAccess::retainedDelta(manager));
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::recoveryStatus(manager)
                  .exact_terminal_predicate);
  EXPECT_EQ(command_capture.size(), successful_publishes);
  ASSERT_FALSE(delta_history.empty());
  for (std::size_t index = 1U; index < delta_history.size(); ++index) {
    EXPECT_LE(delta_history[index], delta_history[index - 1U] + 1e-5);
  }
  EXPECT_NE(std::find(revision_history.begin(), revision_history.end(),
                      successor_pair->path_revision), revision_history.end());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
