#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
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
#include <ros/master.h>


namespace FLAG_Race {

// Narrow test-only access for current production phase/governor and Section
// publish-first seams.  Retired Pair/READY/mailbox helpers are intentionally
// absent from this fixture.
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

  static guidance::IsfGains sectionGains() {
    guidance::IsfGains gains;
    gains.k1 = 2.0;
    gains.k2 = -2.2;
    gains.convergence_bandwidth = 0.1;
    gains.progress_rho0 = 0.5;
    gains.progress_delta = 0.3;
    gains.alpha_min = 0.05;
    return gains;
  }

  static PhaseOffsetMatchedAdapterConfig sectionConfig() {
    PhaseOffsetMatchedAdapterConfig config;
    config.mode = PhaseOffsetMatchedMode::MANUAL;
    config.tube_source = phase_offset_navigation::TubeSource::ESDF;
    config.coordination_backend = PhaseOffsetCoordinationBackend::D1B;
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

  static bool installNeutralAdapter(gvf_manager& manager) {
    PhaseOffsetMatchedAdapterConfig config = sectionConfig();
    config.section_build.clearance = config.tube.cross_section.planner_safe_distance;
    config.section_build.half_width = config.tube.cross_section.search_extent;
    config.section_build.minimum_reference_speed =
        config.tube.cross_section.minimum_reference_speed;
    config.section_build.max_step_w = 0.25;
    config.section_build.max_obstacle_checks = 100000U;
    config.section_build.max_obstacles = 1000U;
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
    return manager.installPlannerOnlyFrontend(
        manager.swarmParticlesManager.front(), traj, vel, time, w, owner,
        ros::Time(7, 0), expected_phase, copied_prefix_source,
        copied_prefix_end_w, expected_execution_generation, path_msg);
  }

  static std::uint64_t executionGeneration(const gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_
        ? manager.phase_offset_matched_adapter_->executionGeneration() : 0U;
  }

  static bool finalizeSectionTerminal(
      gvf_manager& manager, const std::uint64_t expected_task_generation,
      double& retained_manual_reference) {
    if (manager.swarmParticlesManager.empty()) return false;
    return manager.finalizeSectionTerminal(
        manager.swarmParticlesManager.front(), expected_task_generation,
        retained_manual_reference);
  }

  static int currentTrajectoryIndex(const gvf_manager& manager) {
    return manager.current_traj_index_;
  }

  static void configureTerminalExecution(gvf_manager& manager,
                                         const double phase_w,
                                         const ros::Time& now) {
    // This fixture bypasses gvf_manager::init(), so initialize the command
    // mode flags that init() normally loads before exercising cmdCallback.
    // The isolated governor tests must enter the ordinary command path, not
    // the legacy test-command or gain-test early returns.
    manager.use_test_cmd_ = false;
    manager.cmd_gain_test_enable_ = false;
    manager.enable_gvfcmd_control = true;
    manager.test_traj_index_ = 0;
    manager.enable_trajectory_concatenation_ = false;
    manager.exec_state_ = gvf_manager::EXEC_TRAJ;
    manager.point_phase_v2_enabled_ = true;
    manager.closed_tracking_mode_ = "legacy";
    manager.enable_circle_reference_test_ = false;
    manager.collision_threshold_ = 0.0;
    manager.planInterval = 1e9;
    manager.last_replan_time_ = now;
    manager.last_section_build_time_ = now;
    manager.progress_w_ = phase_w;
    manager.progress_initialized_ = true;
    manager.current_traj_index_ = 0;
  }

  static bool isExecuting(const gvf_manager& manager) {
    return manager.exec_state_ == gvf_manager::EXEC_TRAJ;
  }

  static bool isWaitingForTarget(const gvf_manager& manager) {
    return manager.exec_state_ == gvf_manager::WAIT_TARGET;
  }

  static std::shared_ptr<const SectionPathHandoff>
      pendingSectionPathHandoff(gvf_manager& manager) {
    std::lock_guard<std::mutex> lock(manager.path_reference_handoff_mutex_);
    return manager.pending_section_path_handoff_;
  }

  static void setPendingSectionPathHandoff(
      gvf_manager& manager,
      const std::shared_ptr<const SectionPathHandoff>& handoff) {
    std::lock_guard<std::mutex> lock(manager.path_reference_handoff_mutex_);
    manager.pending_section_path_handoff_ = handoff;
  }

  static bool consumeCommittedSectionPathHandoff(gvf_manager& manager) {
    return !manager.swarmParticlesManager.empty() &&
        manager.consumeCommittedSectionPathHandoff(
            manager.swarmParticlesManager.front(), ros::Time(7, 0));
  }

  static SectionPathBundlePtr currentSectionBundle(gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_
        ? manager.phase_offset_matched_adapter_->captureSectionBundle()
        : SectionPathBundlePtr();
  }

  static SectionPathBundlePtr pendingSectionBundle(gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_
        ? manager.phase_offset_matched_adapter_->capturePendingSectionBundle()
        : SectionPathBundlePtr();
  }

  static void openSectionGate(gvf_manager& manager) {
    if (manager.phase_offset_matched_adapter_) {
      manager.phase_offset_matched_adapter_->zero_gate_open_ = true;
    }
  }

  static bool stageSectionBundle(
      gvf_manager& manager, const SectionPathBundlePtr& bundle,
      const std::shared_ptr<const ContinuousPhasePath>& source,
      double prefix_start, double prefix_end) {
    return manager.phase_offset_matched_adapter_ &&
        manager.phase_offset_matched_adapter_->stageSectionBundle(
            bundle, source, prefix_start, prefix_end);
  }

  static bool updateSection(gvf_manager& manager,
                            const MatchedAdapterInput& input,
                            MatchedAdapterOutput& output) {
    return manager.phase_offset_matched_adapter_ &&
        manager.phase_offset_matched_adapter_->update(input, output);
  }

  static PendingPositionCommandCapture capturePending(gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_
        ? manager.phase_offset_matched_adapter_->capturePendingPositionCommand()
        : PendingPositionCommandCapture();
  }

  static bool publishPending(gvf_manager& manager,
                             const std::function<bool()>& local_publish,
                             const std::uint64_t identity) {
    return manager.phase_offset_matched_adapter_ &&
        manager.phase_offset_matched_adapter_->publishPendingPositionCommand(
            local_publish, identity);
  }

  static bool hasPending(gvf_manager& manager) {
    return manager.phase_offset_matched_adapter_ &&
        manager.phase_offset_matched_adapter_->hasPendingPositionCommand();
  }

  static void discardPendingCommand(gvf_manager& manager) {
    if (manager.phase_offset_matched_adapter_) {
      manager.phase_offset_matched_adapter_->discardPendingPositionCommand();
    }
  }

  // Test-only replacement used to exercise the production command callback's
  // candidate retry boundary.  The planner-facing API still stages immutable
  // Section values; this helper only prepares a current/staged fixture under
  // the adapter's normal command mutex.
  static void replaceCurrentSectionBundle(
      gvf_manager& manager, const SectionPathBundlePtr& bundle) {
    if (!manager.phase_offset_matched_adapter_) return;
    std::lock_guard<std::mutex> lock(
        manager.phase_offset_matched_adapter_->runtime_command_mutex_);
    manager.phase_offset_matched_adapter_->section_bundle_ = bundle;
    manager.phase_offset_matched_adapter_->staged_section_bundle_.reset();
    manager.phase_offset_matched_adapter_->staged_section_source_path_.reset();
    manager.phase_offset_matched_adapter_->staged_section_copied_prefix_start_w_ =
        std::numeric_limits<double>::quiet_NaN();
    manager.phase_offset_matched_adapter_->staged_section_copied_prefix_end_w_ =
        std::numeric_limits<double>::quiet_NaN();
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

std::shared_ptr<const FLAG_Race::ContinuousPhasePath> makeSectionPath(
    const double start_w = 0.0, const double end_w = 4.0,
    const std::uint64_t revision = 11U) {
  std::shared_ptr<FLAG_Race::ContinuousPhasePath> path(
      new FLAG_Race::ContinuousPhasePath());
  if (!path->appendSegment(
          start_w, end_w, "section_straight",
          [start_w, end_w](const double w,
                           FLAG_Race::ContinuousPhasePathState& state) {
            state.p = Eigen::Vector3d(w, 0.0, 1.0);
            state.dp_dw = Eigen::Vector3d::UnitX();
            state.d2p_dw2 = Eigen::Vector3d::Zero();
            state.vel = state.dp_dw;
            state.valid = std::isfinite(w) && w >= start_w && w <= end_w;
            return state.valid;
          })) {
    return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  }
  path->setPathRevision(revision);
  return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(path);
}

// Deliberately non-evaluable owner for the current-bundle fallback test.  The
// path remains structurally nonempty so it can occupy the committed Section
// slot, while every state query fails closed.  This models a temporary current
// frame/path evaluation failure without weakening the candidate's valid
// guidance setup.
std::shared_ptr<const FLAG_Race::ContinuousPhasePath>
makeInvalidSectionPath(const double start_w = 0.0,
                       const double end_w = 4.0,
                       const std::uint64_t revision = 71U) {
  std::shared_ptr<FLAG_Race::ContinuousPhasePath> path(
      new FLAG_Race::ContinuousPhasePath());
  if (!path->appendSegment(
          start_w, end_w, "section_invalid_current",
          [](const double /*w*/, FLAG_Race::ContinuousPhasePathState& state) {
            state.valid = false;
            return false;
          })) {
    return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  }
  path->setPathRevision(revision);
  return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(path);
}

std::shared_ptr<const FLAG_Race::ContinuousPhasePath>
makeSectionSuccessor(
    const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>& source,
    const double seam_w = 1.20, const double end_w = 4.0,
    const std::uint64_t revision = 12U) {
  if (!source || source->empty() || seam_w <= source->startW() ||
      seam_w >= source->endW() || end_w <= seam_w) {
    return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  }
  std::shared_ptr<FLAG_Race::ContinuousPhasePath> successor(
      new FLAG_Race::ContinuousPhasePath());
  if (!successor->appendSlice(*source, source->startW(), seam_w) ||
      !successor->appendSegment(
          seam_w, end_w, "section_successor_tail",
          [seam_w, end_w](const double w,
                          FLAG_Race::ContinuousPhasePathState& state) {
            const double dw = w - seam_w;
            state.p = Eigen::Vector3d(w, 0.04 * dw * dw, 1.0);
            state.dp_dw = Eigen::Vector3d(1.0, 0.08 * dw, 0.0);
            state.d2p_dw2 = Eigen::Vector3d(0.0, 0.08, 0.0);
            state.vel = state.dp_dw;
            state.valid = std::isfinite(w) && w >= seam_w && w <= end_w;
            return state.valid;
          })) {
    return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>();
  }
  successor->setPathRevision(revision);
  return std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(successor);
}

std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>
makeSectionProfile(
    const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>& path) {
  if (!path || path->empty()) {
    return std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>();
  }
  std::shared_ptr<phase_offset_navigation::SectionTubeProfile> profile(
      new phase_offset_navigation::SectionTubeProfile());
  profile->valid_start = path->startW();
  profile->valid_end = path->endW();
  profile->status = phase_offset_navigation::SectionTubeStatus::COMPLETE;
  profile->usable = true;
  profile->complete = true;
  const double middle = 0.5 * (path->startW() + path->endW());
  for (const double w : {path->startW(), middle, path->endW()}) {
    phase_offset_navigation::SectionTubeKnot knot;
    knot.w = w;
    knot.lower = -1.0;
    knot.upper = 1.0;
    profile->knots.push_back(knot);
  }
  return std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>(
      profile);
}

FLAG_Race::SectionPathBundlePtr makeSectionBundle(
    const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>& path,
    const std::uint64_t generation) {
  if (!path || path->empty() || path->pathRevision() == 0U ||
      generation == 0U) {
    return FLAG_Race::SectionPathBundlePtr();
  }
  std::shared_ptr<FLAG_Race::SectionPathBundle> bundle(
      new FLAG_Race::SectionPathBundle());
  bundle->path = path;
  bundle->frame = std::shared_ptr<const FLAG_Race::ContinuousPhaseNormalFrame>(
      new FLAG_Race::ContinuousPhaseNormalFrame(
          path, path->pathRevision(), path->pathRevision()));
  bundle->profile = makeSectionProfile(path);
  bundle->local_view_status = plan_env::LocalObstacleViewStatus::VALID;
  bundle->frame_id = "world";
  bundle->task_generation = generation;
  bundle->path_revision = path->pathRevision();
  bundle->frame_revision = path->pathRevision();
  bundle->environment_validity =
      std::make_shared<plan_env::EnvironmentValidityState>();
  bundle->environment_change_mutex = std::make_shared<std::recursive_mutex>();
  bundle->build_config.clearance = 0.4;
  bundle->build_config.half_width = 1.0;
  bundle->build_config.minimum_reference_speed = 1e-8;
  bundle->build_config.max_obstacle_checks = 100000U;
  bundle->build_config.max_obstacles = 1000U;
  return FLAG_Race::SectionPathBundlePtr(bundle);
}

FLAG_Race::SectionPathBundlePtr makePartialSectionBundle(
    const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>& path,
    const std::uint64_t generation, const double valid_end_w) {
  const FLAG_Race::SectionPathBundlePtr bundle =
      makeSectionBundle(path, generation);
  if (!bundle || !bundle->profile || !std::isfinite(valid_end_w) ||
      !(valid_end_w > path->startW()) || valid_end_w >= path->endW()) {
    return FLAG_Race::SectionPathBundlePtr();
  }
  std::shared_ptr<phase_offset_navigation::SectionTubeProfile> profile(
      new phase_offset_navigation::SectionTubeProfile(*bundle->profile));
  profile->status = phase_offset_navigation::SectionTubeStatus::PARTIAL;
  profile->complete = false;
  profile->valid_end = valid_end_w;
  profile->knots.clear();
  for (const double w : {path->startW(),
                         0.5 * (path->startW() + valid_end_w), valid_end_w}) {
    phase_offset_navigation::SectionTubeKnot knot;
    knot.w = w;
    knot.lower = -1.0;
    knot.upper = 1.0;
    profile->knots.push_back(knot);
  }
  std::shared_ptr<FLAG_Race::SectionPathBundle> mutable_bundle =
      std::const_pointer_cast<FLAG_Race::SectionPathBundle>(bundle);
  mutable_bundle->profile =
      std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>(
          profile);
  return bundle;
}

bool makeSectionInput(
    const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>& path,
    const double w, FLAG_Race::MatchedAdapterInput& input) {
  input = FLAG_Race::MatchedAdapterInput();
  if (!path || path->empty()) return false;
  FLAG_Race::ContinuousPhasePathState state;
  const FLAG_Race::ContinuousPhaseNormalFrame frame(
      path, path->pathRevision(), path->pathRevision());
  if (!frame.evaluatePathState(w, state) || !state.valid ||
      !state.frame_valid) return false;
  input.path = FLAG_Race::ConvertContinuousPhasePathStateForActive(state, w);
  input.semantic_path_owner = path;
  input.semantic_path_start_w = path->startW();
  input.semantic_path_end_w = path->endW();
  input.position = state.p + Eigen::Vector3d(0.0, 0.05, 0.0);
  input.gains = FLAG_Race::GvfManagerS4AnchorTestAccess::sectionGains();
  input.dt = 0.10;
  input.stamp = ros::Time(7, 0);
  input.g_des = Eigen::Vector3d::Zero();
  input.g_des_valid = true;

  FLAG_Race::PhaseOffsetActiveAdapter zero;
  FLAG_Race::ActiveAdapterInput zero_input;
  zero_input.path = input.path;
  zero_input.position = input.position;
  zero_input.gains = input.gains;
  FLAG_Race::ActiveAdapterOutput zero_output;
  if (!zero.evaluate(zero_input, zero_output)) return false;
  input.legacy = FLAG_Race::LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
  return true;
}

FLAG_Race::PhaseOffsetMatchedAdapterConfig makeSectionConfig() {
  FLAG_Race::PhaseOffsetMatchedAdapterConfig config =
      FLAG_Race::GvfManagerS4AnchorTestAccess::sectionConfig();
  config.section_build.clearance = config.tube.cross_section.planner_safe_distance;
  config.section_build.half_width = config.tube.cross_section.search_extent;
  config.section_build.minimum_reference_speed =
      config.tube.cross_section.minimum_reference_speed;
  config.section_build.max_step_w = 0.25;
  config.section_build.max_obstacle_checks = 100000U;
  config.section_build.max_obstacles = 1000U;
  return config;
}

bool populateSectionFrontend(
    FLAG_Race::gvf_manager& manager,
    const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>& path) {
  if (manager.swarmParticlesManager.empty() || !path || path->empty()) {
    return false;
  }
  auto& frontend = manager.swarmParticlesManager.front();
  const std::vector<double> w{path->startW(),
                              0.5 * (path->startW() + path->endW()),
                              path->endW()};
  frontend.last_traj.resize(3, 3);
  frontend.last_vel.resize(3, 3);
  frontend.last_traj_time_.resize(3);
  for (int index = 0; index < 3; ++index) {
    FLAG_Race::ContinuousPhasePathState state;
    if (!path->evaluate(w[static_cast<std::size_t>(index)], state, false)) {
      return false;
    }
    frontend.last_traj.row(index) = state.p.transpose();
    frontend.last_vel.row(index) = state.dp_dw.transpose();
    frontend.last_traj_time_(index) = static_cast<double>(index);
  }
  frontend.gvf_->setNextPathWSamples(w);
  return true;
}

bool installSectionManager(
    FLAG_Race::gvf_manager& manager,
    const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>& path,
    const FLAG_Race::SectionPathBundlePtr& bundle,
    const double phase_w = 0.40) {
  if (!path || !bundle || !FLAG_Race::GvfManagerS4AnchorTestAccess::
      installNeutralAdapter(manager) ||
      !FLAG_Race::GvfManagerS4AnchorTestAccess::initializePlannerFrontend(
          manager, path) || !populateSectionFrontend(manager, path)) {
    return false;
  }
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, phase_w, true, false);
  FLAG_Race::GvfManagerS4AnchorTestAccess::openSectionGate(manager);
  if (!FLAG_Race::GvfManagerS4AnchorTestAccess::stageSectionBundle(
          manager, bundle,
          std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN())) {
    return false;
  }
  FLAG_Race::MatchedAdapterInput input;
  if (!makeSectionInput(path, phase_w, input)) return false;
  input.section_bundle = bundle;
  FLAG_Race::MatchedAdapterOutput output;
  if (!FLAG_Race::GvfManagerS4AnchorTestAccess::updateSection(
          manager, input, output) ||
      !output.selected || !output.section_prepared_step) {
    return false;
  }
  const FLAG_Race::PendingPositionCommandCapture pending =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePending(manager);
  if (!pending.pending || !pending.valid ||
      !FLAG_Race::GvfManagerS4AnchorTestAccess::publishPending(
          manager, []() { return true; }, pending.identity)) {
    return false;
  }
  return FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(
             manager) == bundle;
}

struct SectionTransitionFixture {
  std::shared_ptr<const FLAG_Race::ContinuousPhasePath> source;
  std::shared_ptr<const FLAG_Race::ContinuousPhasePath> successor;
  FLAG_Race::SectionPathBundlePtr source_bundle;
  FLAG_Race::SectionPathBundlePtr successor_bundle;
  FLAG_Race::PendingPositionCommandCapture pending;
  std::shared_ptr<const FLAG_Race::SectionPathHandoff> handoff;
  double prepared_next_delta = 0.0;
  double phase_before = 0.0;
  double phase_after = 0.0;
};

bool prepareSectionTransition(FLAG_Race::gvf_manager& manager,
                              SectionTransitionFixture& fixture,
                              const bool source_nonzero = true) {
  const auto fail = [](const std::string& reason) {
    ADD_FAILURE() << "Section transition fixture: " << reason;
    return false;
  };
  fixture = SectionTransitionFixture();
  fixture.source = makeSectionPath(0.0, 4.0, 11U);
  fixture.successor = makeSectionSuccessor(fixture.source, 1.20, 4.0, 12U);
  if (!fixture.source || !fixture.successor ||
      !FLAG_Race::GvfManagerS4AnchorTestAccess::installNeutralAdapter(manager)) {
    return fail("paths or adapter setup");
  }
  const std::uint64_t generation =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(manager);
  fixture.source_bundle = makeSectionBundle(fixture.source, generation);
  fixture.successor_bundle = makeSectionBundle(fixture.successor, generation);
  if (!fixture.source_bundle || !fixture.successor_bundle ||
      !FLAG_Race::GvfManagerS4AnchorTestAccess::initializePlannerFrontend(
          manager, fixture.source) || !populateSectionFrontend(manager, fixture.source)) {
    return fail("bundle/frontend setup");
  }
  const double initial_phase_w = 0.40;
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, initial_phase_w, true, false);
  FLAG_Race::GvfManagerS4AnchorTestAccess::openSectionGate(manager);
  if (!FLAG_Race::GvfManagerS4AnchorTestAccess::stageSectionBundle(
          manager, fixture.source_bundle,
          std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(),
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN())) {
    return fail("stage source bundle");
  }
  FLAG_Race::MatchedAdapterInput source_input;
  if (!makeSectionInput(fixture.source, initial_phase_w, source_input)) {
    return fail("source input");
  }
  // Establish the source command through the real Section D1B path.  The
  // normal transition fixture uses a lateral intent so Runtime owns a real
  // nonzero delta; terminal-cancellation tests may opt into the same
  // production path with a neutral source and a prepared successor.
  if (source_nonzero) {
    source_input.g_des = Eigen::Vector3d(0.0, 0.30, 0.0);
  }
  source_input.section_bundle = fixture.source_bundle;
  FLAG_Race::MatchedAdapterOutput output;
  if (!FLAG_Race::GvfManagerS4AnchorTestAccess::updateSection(
      manager, source_input, output) ||
      !output.selected) {
    return fail(std::string("source update: ") + output.invalid_reason);
  }
  const FLAG_Race::PendingPositionCommandCapture source_pending =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePending(manager);
  if (!source_pending.pending || !source_pending.valid ||
      !FLAG_Race::GvfManagerS4AnchorTestAccess::publishPending(
          manager, []() { return true; }, source_pending.identity)) {
    return fail("source publish");
  }
  const auto source_runtime =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  if (!source_runtime.present || !std::isfinite(source_runtime.retained_delta) ||
      (source_nonzero && source_runtime.retained_delta == 0.0) ||
      (!source_nonzero && source_runtime.retained_delta != 0.0)) {
    return fail(source_nonzero
                    ? "source publish did not establish nonzero Runtime delta"
                    : "neutral source publish changed Runtime delta");
  }
  if (!std::isfinite(source_pending.section_next_w) ||
      source_pending.section_next_w <= initial_phase_w ||
      source_pending.section_next_w >= fixture.source->endW()) {
    return fail("source publish produced invalid successor phase");
  }
  // The Section commit advances Runtime's retained offset, while the
  // authoritative phase remains manager-owned.  Mirror the exact committed
  // next-W before staging the successor so every prefix witness is tied to
  // the live phase actually observed by the next command.
  const double live_w = source_pending.section_next_w;
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, live_w, true, false);
  if (!FLAG_Race::GvfManagerS4AnchorTestAccess::stageSectionBundle(
          manager, fixture.successor_bundle, fixture.source, live_w, 1.20)) {
    return fail("stage successor bundle");
  }
  FLAG_Race::MatchedAdapterInput successor_input;
  if (!makeSectionInput(fixture.successor, live_w, successor_input)) {
    return fail("successor input");
  }
  successor_input.g_des = Eigen::Vector3d(0.0, 0.30, 0.0);
  successor_input.section_bundle = fixture.successor_bundle;
  if (!FLAG_Race::GvfManagerS4AnchorTestAccess::updateSection(
      manager, successor_input, output) ||
      !output.selected || !output.section_prepared_step) {
    return fail(std::string("successor update: ") + output.invalid_reason);
  }
  fixture.prepared_next_delta = output.section_prepared_step->nextDelta();
  fixture.pending =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePending(manager);
  if (!fixture.pending.pending || !fixture.pending.valid ||
      !fixture.pending.section_transition ||
      fixture.pending.section_bundle != fixture.successor_bundle) {
    return false;
  }

  std::shared_ptr<FLAG_Race::SectionPathHandoff> handoff(
      new FLAG_Race::SectionPathHandoff());
  const auto phase =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  handoff->task_generation = generation;
  handoff->phase_generation = phase.generation;
  handoff->source_path = fixture.source;
  handoff->candidate_path = fixture.successor;
  handoff->bundle = fixture.successor_bundle;
  handoff->copied_prefix_start_w = live_w;
  handoff->copied_prefix_end_w = 1.20;
  handoff->w = {0.0, live_w, 1.20, 4.0};
  handoff->traj.resize(4, 3);
  handoff->vel.resize(4, 3);
  handoff->time.resize(4);
  handoff->path_msg.header.frame_id = "world";
  handoff->path_msg.poses.reserve(handoff->w.size());
  for (std::size_t index = 0; index < handoff->w.size(); ++index) {
    FLAG_Race::ContinuousPhasePathState state;
    if (!fixture.successor->evaluate(handoff->w[index], state, false)) {
      return fail("handoff sample");
    }
    handoff->traj.row(static_cast<int>(index)) = state.p.transpose();
    handoff->vel.row(static_cast<int>(index)) = state.dp_dw.transpose();
    handoff->time(static_cast<int>(index)) = static_cast<double>(index);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = state.p.x();
    pose.pose.position.y = state.p.y();
    pose.pose.position.z = state.p.z();
    pose.pose.orientation.x = state.dp_dw.x();
    pose.pose.orientation.y = state.dp_dw.y();
    pose.pose.orientation.z = state.dp_dw.z();
    pose.pose.orientation.w = 1.0;
    handoff->path_msg.poses.push_back(pose);
  }
  if (!handoff->complete()) return fail("handoff completeness");
  fixture.handoff = std::shared_ptr<const FLAG_Race::SectionPathHandoff>(
      std::move(handoff));
  FLAG_Race::GvfManagerS4AnchorTestAccess::setPendingSectionPathHandoff(
      manager, fixture.handoff);
  fixture.phase_before = phase.w;
  fixture.phase_after = fixture.pending.section_next_w;
  return true;
}

bool configureTerminalExecution(FLAG_Race::gvf_manager& manager,
                                const SectionTransitionFixture& fixture) {
  if (manager.swarmParticlesManager.empty() || !fixture.source ||
      !std::isfinite(fixture.phase_before)) {
    return false;
  }
  FLAG_Race::ContinuousPhasePathState phase_state;
  if (!fixture.source->evaluate(fixture.phase_before, phase_state, false) ||
      !phase_state.valid) {
    return false;
  }
  auto& frontend = manager.swarmParticlesManager.front();
  manager.odom_ = phase_state.p;
  FLAG_Race::GvfManagerS4AnchorTestAccess::configureTerminalExecution(
      manager, fixture.phase_before, ros::Time::now());
  frontend.receive_startpt = false;
  frontend.receive_goal = true;
  frontend.is_first_goal = false;
  frontend.goal_pt = manager.odom_;
  return true;
}

bool stageSuccessorPending(FLAG_Race::gvf_manager& manager,
                           const SectionTransitionFixture& fixture) {
  if (!fixture.source || !fixture.successor || !fixture.successor_bundle ||
      !std::isfinite(fixture.phase_before) ||
      !(fixture.phase_before < 1.20)) {
    return false;
  }
  if (!FLAG_Race::GvfManagerS4AnchorTestAccess::stageSectionBundle(
          manager, fixture.successor_bundle, fixture.source,
          fixture.phase_before, 1.20)) {
    return false;
  }
  FLAG_Race::MatchedAdapterInput input;
  if (!makeSectionInput(fixture.successor, fixture.phase_before, input)) {
    return false;
  }
  input.g_des = Eigen::Vector3d(0.0, 0.30, 0.0);
  input.section_bundle = fixture.successor_bundle;
  FLAG_Race::MatchedAdapterOutput output;
  if (!FLAG_Race::GvfManagerS4AnchorTestAccess::updateSection(
          manager, input, output) ||
      !output.selected || !output.section_prepared_step) {
    return false;
  }
  const auto pending =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePending(manager);
  return pending.pending && pending.valid && pending.section_transition &&
      pending.section_bundle == fixture.successor_bundle;
}

bool installTestCommandPublisher(FLAG_Race::gvf_manager& manager) {
  if (!ros::isInitialized()) {
    int argc = 1;
    char name[] = "gvf_switch_policy_test";
    char* argv[] = {name, nullptr};
    ros::init(argc, argv, "gvf_switch_policy_test",
              ros::init_options::AnonymousName |
                  ros::init_options::NoSigintHandler);
  }
  if (!ros::master::check()) return false;
  try {
    ros::NodeHandle nh;
    manager.cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>(
        "/gvf_switch_policy_test/position_cmd", 1);
  } catch (const std::exception&) {
    return false;
  }
  return static_cast<bool>(manager.cmd_pub);
}

bool prepareNonzeroSectionCommandManager(
    FLAG_Race::gvf_manager& manager,
    const std::shared_ptr<const FLAG_Race::ContinuousPhasePath>& path,
    const FLAG_Race::SectionPathBundlePtr& bundle) {
  if (!path || !bundle || !installSectionManager(manager, path, bundle)) {
    return false;
  }
  if (manager.swarmParticlesManager.empty() ||
      !manager.swarmParticlesManager.front().gvf_) {
    return false;
  }
  auto& pm = manager.swarmParticlesManager.front();
  pm.gvf_->gvf_.K1_ = 2.0;
  pm.gvf_->gvf_.K2_ = -2.2;
  pm.gvf_->gvf_.convergence_bandwidth_ = 0.1;
  pm.gvf_->progress_rho0_ = 0.5;
  pm.gvf_->progress_delta_ = 0.3;
  pm.gvf_->alpha_min_ = 0.05;

  FLAG_Race::ContinuousPhasePathState state;
  if (!path->evaluate(0.40, state, false) || !state.valid) return false;
  manager.odom_ = state.p + Eigen::Vector3d(0.0, 0.05, 0.0);
  pm.receive_startpt = false;
  pm.receive_goal = true;
  pm.is_first_goal = false;
  pm.goal_pt = manager.odom_ + Eigen::Vector3d(10.0, 0.0, 0.0);
  FLAG_Race::GvfManagerS4AnchorTestAccess::configureTerminalExecution(
      manager, 0.40, ros::Time::now());
  manager.cmd_governor_l_min_ = 0.50;
  manager.cmd_governor_l_max_ = 0.50;
  manager.cmd_governor_l_step_ = 0.05;
  manager.cmd_governor_lead_max_ = 1.6;
  manager.cmd_governor_normal_max_ = 0.0;
  manager.cmd_governor_initialized_ = false;
  manager.cmd_governor_normal_state_.setZero();
  manager.cmd_governor_last_l_ = 0.0;

  FLAG_Race::MatchedAdapterInput input;
  if (!makeSectionInput(path, 0.40, input)) return false;
  input.g_des = Eigen::Vector3d(0.0, 0.30, 0.0);
  input.section_bundle = bundle;
  FLAG_Race::MatchedAdapterOutput output;
  if (!FLAG_Race::GvfManagerS4AnchorTestAccess::updateSection(
          manager, input, output) ||
      !output.selected) {
    return false;
  }
  const FLAG_Race::PendingPositionCommandCapture pending =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePending(manager);
  return pending.pending && pending.valid &&
      FLAG_Race::GvfManagerS4AnchorTestAccess::publishPending(
          manager, []() { return true; }, pending.identity) &&
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager)
              .retained_delta !=
          0.0;
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
  ASSERT_TRUE(FLAG_Race::gvf_manager::plannerOnlyFutureSeam(
      source, 0.40, 0.60, seam_w));
  EXPECT_DOUBLE_EQ(seam_w, 1.25);

  seam_w = -1.0;
  EXPECT_FALSE(FLAG_Race::gvf_manager::plannerOnlyFutureSeam(
      source, 3.30, 0.40, seam_w));
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

TEST(GvfPlannerOnly,
     NeutralContinuationInstallsWithoutSectionBundleAndNonzeroBlocksReplacement) {
  ros::Time::init();
  const auto original = makeH2OldSeamPath();
  const auto replacement = makeH2ReplacementPath();
  ASSERT_TRUE(original);
  ASSERT_TRUE(replacement);

  FLAG_Race::gvf_manager neutral_manager;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::installNeutralAdapter(
      neutral_manager));
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::initializePlannerFrontend(
      neutral_manager, original));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      neutral_manager, 0.40, true, false);
  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(neutral_manager);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(neutral_manager);
  ASSERT_TRUE(runtime_before.present);
  EXPECT_DOUBLE_EQ(runtime_before.retained_delta, 0.0);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(
      neutral_manager));

  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      neutral_manager, replacement));
  const auto phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(neutral_manager);
  EXPECT_DOUBLE_EQ(phase_after.w, phase_before.w);
  EXPECT_EQ(phase_after.initialized, phase_before.initialized);
  EXPECT_EQ(phase_after.closed_acquired, phase_before.closed_acquired);
  EXPECT_EQ(phase_after.generation, phase_before.generation);
  EXPECT_DOUBLE_EQ(
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(neutral_manager)
          .retained_delta,
      0.0);
  ASSERT_TRUE(neutral_manager.swarmParticlesManager.front().gvf_);
  const auto installed =
      neutral_manager.swarmParticlesManager.front().gvf_->getContinuousPhasePath();
  ASSERT_TRUE(installed);
  EXPECT_NE(installed, original);
  EXPECT_NE(installed->pathRevision(), 0U);
  FLAG_Race::ContinuousPhasePathState installed_state;
  FLAG_Race::ContinuousPhasePathState replacement_state;
  ASSERT_TRUE(installed->evaluate(1.0, installed_state, false));
  ASSERT_TRUE(replacement->evaluate(1.0, replacement_state, false));
  EXPECT_TRUE(installed_state.p.isApprox(replacement_state.p, 0.0));
  EXPECT_TRUE(installed_state.dp_dw.isApprox(replacement_state.dp_dw, 0.0));

  FLAG_Race::gvf_manager nonzero_manager;
  SectionTransitionFixture transition;
  ASSERT_TRUE(prepareSectionTransition(nonzero_manager, transition));
  FLAG_Race::GvfManagerS4AnchorTestAccess::discardPendingCommand(nonzero_manager);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setPendingSectionPathHandoff(
      nonzero_manager, std::shared_ptr<const FLAG_Race::SectionPathHandoff>());
  const auto nonzero_phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(nonzero_manager);
  const auto nonzero_runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(nonzero_manager);
  const auto nonzero_owner_before =
      nonzero_manager.swarmParticlesManager.front().gvf_->getContinuousPhasePath();
  ASSERT_NE(nonzero_runtime_before.retained_delta, 0.0);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      nonzero_manager, replacement));
  EXPECT_EQ(nonzero_manager.swarmParticlesManager.front().gvf_
                ->getContinuousPhasePath(),
            nonzero_owner_before);
  const auto nonzero_phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(nonzero_manager);
  const auto nonzero_runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(nonzero_manager);
  EXPECT_DOUBLE_EQ(nonzero_phase_after.w, nonzero_phase_before.w);
  EXPECT_EQ(nonzero_phase_after.generation, nonzero_phase_before.generation);
  EXPECT_DOUBLE_EQ(nonzero_runtime_after.retained_delta,
                   nonzero_runtime_before.retained_delta);
}

TEST(GvfPlannerOnly,
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
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(manager);
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

TEST(GvfPlannerOnly,
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
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(expired);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      expired, replacement, source, 1.20, expired_generation, 1.21));
  EXPECT_EQ(expired.swarmParticlesManager.front().gvf_->getContinuousPhasePath(),
            source);
  EXPECT_DOUBLE_EQ(
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(expired).w, 1.21);

  FLAG_Race::gvf_manager reset;
  ASSERT_TRUE(initialize(reset));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      reset, 0.40, true, false);
  const std::uint64_t stale_task_generation =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(reset);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      reset, replacement, source, 1.20, stale_task_generation, 0.65, true));
  EXPECT_EQ(reset.swarmParticlesManager.front().gvf_->getContinuousPhasePath(),
            source);

  FLAG_Race::gvf_manager exact;
  ASSERT_TRUE(initialize(exact));
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      exact, 0.40, true, false);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::installPlannerOnly(
      exact, replacement,
      std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(), 0.40, 0U,
      0.41));
  EXPECT_EQ(exact.swarmParticlesManager.front().gvf_->getContinuousPhasePath(),
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
TEST(GvfSectionPathHandoffTest,
     ExactPhaseAndCopiedPrefixEvidenceRemainImmutableUntilCommit) {
  ros::Time::init();
  FLAG_Race::gvf_manager manager;
  SectionTransitionFixture fixture;
  ASSERT_TRUE(prepareSectionTransition(manager, fixture));
  ASSERT_TRUE(fixture.handoff);
  EXPECT_TRUE(fixture.handoff->complete());
  EXPECT_EQ(fixture.handoff->source_path, fixture.source);
  EXPECT_EQ(fixture.handoff->candidate_path, fixture.successor);
  EXPECT_EQ(fixture.handoff->bundle, fixture.successor_bundle);
  EXPECT_DOUBLE_EQ(fixture.handoff->copied_prefix_start_w,
                   fixture.phase_before);
  EXPECT_DOUBLE_EQ(fixture.handoff->copied_prefix_end_w, 1.20);
  EXPECT_DOUBLE_EQ(fixture.handoff->phase_generation,
                   FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager)
                       .generation);

  FLAG_Race::SectionPathHandoff malformed(*fixture.handoff);
  malformed.copied_prefix_end_w = 0.30;
  EXPECT_FALSE(malformed.complete());
  malformed.copied_prefix_end_w = 1.20;
  malformed.bundle = fixture.source_bundle;
  EXPECT_FALSE(malformed.complete());
  malformed.bundle = fixture.successor_bundle;
  malformed.task_generation = 0U;
  EXPECT_FALSE(malformed.complete());
}

TEST(GvfSectionHandoff,
     PublishFirstCommitThenEnvironmentEditStillConsumesDeferredMirror) {
  ros::Time::init();
  FLAG_Race::gvf_manager manager;
  SectionTransitionFixture fixture;
  ASSERT_TRUE(prepareSectionTransition(manager, fixture));
  ASSERT_TRUE(fixture.handoff);
  ASSERT_TRUE(fixture.pending.pending);
  ASSERT_TRUE(fixture.pending.valid);

  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  const auto handoff_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::pendingSectionPathHandoff(
          manager);
  ASSERT_TRUE(runtime_before.present);
  ASSERT_NE(runtime_before.retained_delta, 0.0);
  ASSERT_EQ(handoff_before, fixture.handoff);
  ASSERT_EQ(manager.swarmParticlesManager.front().gvf_->getContinuousPhasePath(),
            fixture.source);

  int failed_publish_count = 0;
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::publishPending(
      manager,
      [&failed_publish_count]() {
        ++failed_publish_count;
        return false;
      }, fixture.pending.identity));
  EXPECT_EQ(failed_publish_count, 1);
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::hasPending(manager));
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(manager),
            fixture.source_bundle);
  const auto runtime_after_failure =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  EXPECT_DOUBLE_EQ(runtime_after_failure.retained_delta,
                   runtime_before.retained_delta);
  EXPECT_DOUBLE_EQ(runtime_after_failure.previous_port.u_w,
                   runtime_before.previous_port.u_w);
  EXPECT_DOUBLE_EQ(runtime_after_failure.previous_port.u_delta,
                   runtime_before.previous_port.u_delta);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingSectionPathHandoff(
                manager),
            handoff_before);

  int successful_publish_count = 0;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::publishPending(
      manager,
      [&successful_publish_count]() {
        ++successful_publish_count;
        return true;
      }, fixture.pending.identity));
  EXPECT_EQ(successful_publish_count, 1);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::hasPending(manager));
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(manager),
            fixture.successor_bundle);
  const auto runtime_after_success =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  EXPECT_DOUBLE_EQ(runtime_after_success.retained_delta,
                   fixture.prepared_next_delta);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingSectionPathHandoff(
                manager),
            handoff_before);
  // The command/runtime commit is already authoritative.  A subsequent
  // static-map edit invalidates the captured environment witness, but the
  // deferred frontend mirror still consumes the exact committed Section
  // handoff rather than rebinding to a later map snapshot.
  {
    std::lock_guard<std::recursive_mutex> environment_lock(
        *fixture.successor_bundle->environment_change_mutex);
    fixture.successor_bundle->environment_validity->valid = false;
  }
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, fixture.phase_after, true, false);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  consumeCommittedSectionPathHandoff(manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingSectionPathHandoff(
      manager));
  EXPECT_EQ(manager.swarmParticlesManager.front().gvf_->getContinuousPhasePath(),
            fixture.successor);
  EXPECT_GT(manager.swarmParticlesManager.front().last_traj.rows(), 1);
  FLAG_Race::ContinuousPhasePathState display_state;
  ASSERT_TRUE(fixture.successor->evaluate(1.20, display_state, false));
  EXPECT_NEAR(manager.swarmParticlesManager.front().last_traj(2, 0),
              display_state.p.x(), 1e-12);
}

TEST(GvfStage6Handoff,
     PrematureMirrorAndTaskResetCannotRebindOrReplay) {
  ros::Time::init();
  FLAG_Race::gvf_manager manager;
  SectionTransitionFixture fixture;
  ASSERT_TRUE(prepareSectionTransition(manager, fixture));
  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  const Eigen::MatrixXd display_before =
      manager.swarmParticlesManager.front().last_traj;
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   consumeCommittedSectionPathHandoff(manager));
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingSectionPathHandoff(
                manager),
            fixture.handoff);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(manager),
            fixture.source_bundle);
  EXPECT_EQ(manager.swarmParticlesManager.front().gvf_->getContinuousPhasePath(),
            fixture.source);
  EXPECT_TRUE((manager.swarmParticlesManager.front().last_traj.array() ==
               display_before.array()).all());

  FLAG_Race::GvfManagerS4AnchorTestAccess::discardPendingCommand(manager);
  const auto stale_identity = fixture.pending.identity;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                  resetForNewNavigationTask(manager));
  const auto phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingCommand(manager)
                   .pending);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   pendingSectionPathHandoff(manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(
      manager));
  EXPECT_DOUBLE_EQ(phase_after.w, 0.0);
  EXPECT_FALSE(phase_after.initialized);
  EXPECT_FALSE(phase_after.closed_acquired);
  EXPECT_GT(phase_after.generation, phase_before.generation);
  ASSERT_TRUE(runtime_after.present);
  EXPECT_DOUBLE_EQ(runtime_after.retained_delta, 0.0);
  EXPECT_DOUBLE_EQ(runtime_after.previous_port.u_w, 0.0);
  EXPECT_DOUBLE_EQ(runtime_after.previous_port.u_delta, 0.0);

  int stale_publish_count = 0;
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::publishPending(
      manager,
      [&stale_publish_count]() {
        ++stale_publish_count;
        return true;
      }, stale_identity));
  EXPECT_EQ(stale_publish_count, 0);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::
                   consumeCommittedSectionPathHandoff(manager));
  EXPECT_EQ(manager.swarmParticlesManager.front().gvf_->getContinuousPhasePath(),
            fixture.source);
  EXPECT_TRUE((manager.swarmParticlesManager.front().last_traj.array() ==
               display_before.array()).all());
}

TEST(GvfSectionGovernor,
     RejectedGovernorPublishesHoldWithoutRuntimeOrPhaseCommit) {
  ros::Time::init();
  if (std::getenv("M3C_ISOLATED_ROS_TESTS") == nullptr) {
    GTEST_SKIP() << "set M3C_ISOLATED_ROS_TESTS=1 under a task-owned ROS master";
  }
  FLAG_Race::gvf_manager manager;
  const auto source = makeSectionPath(0.0, 4.0, 11U);
  const auto bundle = makeSectionBundle(source, 1U);
  ASSERT_TRUE(source);
  ASSERT_TRUE(bundle);
  ASSERT_TRUE(installSectionManager(manager, source, bundle));
  ASSERT_TRUE(installTestCommandPublisher(manager));

  ASSERT_FALSE(manager.swarmParticlesManager.empty());
  auto& frontend = manager.swarmParticlesManager.front();
  ASSERT_TRUE(frontend.gvf_);
  frontend.gvf_->gvf_.K1_ = 2.0;
  frontend.gvf_->gvf_.K2_ = -2.2;
  frontend.gvf_->gvf_.convergence_bandwidth_ = 0.1;
  frontend.gvf_->progress_rho0_ = 0.5;
  frontend.gvf_->progress_delta_ = 0.3;
  frontend.gvf_->alpha_min_ = 0.05;

  FLAG_Race::ContinuousPhasePathState phase_state;
  ASSERT_TRUE(source->evaluate(0.40, phase_state, false));
  manager.odom_ = phase_state.p;
  FLAG_Race::GvfManagerS4AnchorTestAccess::configureTerminalExecution(
      manager, 0.40, ros::Time::now());
  manager.cmd_governor_l_min_ = 5.0;
  manager.cmd_governor_l_max_ = 5.0;
  manager.cmd_governor_l_step_ = 0.10;
  manager.cmd_governor_lead_max_ = 1.0;
  manager.cmd_governor_initialized_ = false;
  manager.cmd_governor_normal_state_.setZero();
  manager.cmd_governor_last_l_ = 0.0;
  manager.has_last_governor_cmd_ = false;
  frontend.receive_startpt = false;
  frontend.receive_goal = true;
  frontend.is_first_goal = false;
  frontend.goal_pt = manager.odom_ + Eigen::Vector3d(10.0, 0.0, 0.0);

  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  ASSERT_TRUE(runtime_before.present);
  ASSERT_DOUBLE_EQ(runtime_before.retained_delta, 0.0);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(
      manager));

  ros::TimerEvent event;
  manager.cmdCallback(event);

  // Governor rejection still publishes the position hold, but no prepared
  // Section step or phase candidate may cross the Runtime commit boundary.
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::hasPending(manager));
  const auto phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  EXPECT_DOUBLE_EQ(phase_after.w, phase_before.w);
  EXPECT_EQ(phase_after.generation, phase_before.generation);
  const auto runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  EXPECT_DOUBLE_EQ(runtime_after.retained_delta,
                   runtime_before.retained_delta);
  EXPECT_FALSE(manager.pending_last_yaw_valid_);
  EXPECT_TRUE(manager.has_last_governor_cmd_);
  EXPECT_TRUE(manager.last_cmd_pos_.isApprox(manager.odom_, 1e-12));
}

TEST(GvfSectionCommand,
     NonzeroCurrentProfileFailureHoldsWithoutCentrelineFallback) {
  ros::Time::init();
  if (std::getenv("M3C_ISOLATED_ROS_TESTS") == nullptr) {
    GTEST_SKIP() << "set M3C_ISOLATED_ROS_TESTS=1 under a task-owned ROS master";
  }
  FLAG_Race::gvf_manager manager;
  const auto source = makeSectionPath(0.0, 4.0, 41U);
  const auto source_bundle = makeSectionBundle(source, 1U);
  ASSERT_TRUE(source);
  ASSERT_TRUE(source_bundle);
  ASSERT_TRUE(installTestCommandPublisher(manager));
  ASSERT_TRUE(prepareNonzeroSectionCommandManager(
      manager, source, source_bundle));

  const std::uint64_t generation =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(manager);
  const auto partial_bundle =
      makePartialSectionBundle(source, generation, 3.0);
  ASSERT_TRUE(partial_bundle);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::stageSectionBundle(
      manager, partial_bundle,
      std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()));

  FLAG_Race::MatchedAdapterInput partial_input;
  ASSERT_TRUE(makeSectionInput(source, 0.40, partial_input));
  partial_input.g_des = Eigen::Vector3d(0.0, 0.30, 0.0);
  partial_input.section_bundle = partial_bundle;
  FLAG_Race::MatchedAdapterOutput partial_output;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::updateSection(
      manager, partial_input, partial_output));
  ASSERT_TRUE(partial_output.selected);
  const auto partial_pending =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePending(manager);
  ASSERT_TRUE(partial_pending.pending);
  ASSERT_TRUE(partial_pending.valid);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::publishPending(
      manager, []() { return true; }, partial_pending.identity));

  // The partial profile covers the current phase but not the next normal
  // preview horizon.  Advance only the authoritative phase to just before
  // that endpoint; the planner path remains valid and the current Section
  // owner is otherwise unchanged.
  const double phase_before = 0.999;
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, phase_before, true, false);
  FLAG_Race::ContinuousPhasePathState state;
  ASSERT_TRUE(source->evaluate(phase_before, state, false));
  manager.odom_ = state.p + Eigen::Vector3d(0.0, 0.05, 0.0);
  const auto phase_snapshot =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  ASSERT_NE(runtime_before.retained_delta, 0.0);

  ros::TimerEvent event;
  manager.cmdCallback(event);

  const auto phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  EXPECT_DOUBLE_EQ(phase_after.w, phase_snapshot.w);
  EXPECT_DOUBLE_EQ(runtime_after.retained_delta,
                   runtime_before.retained_delta);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(
                manager),
            partial_bundle);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::hasPending(manager));
  // Invalid Section preparation must publish the existing-position hold, not
  // a zero-offset centreline command generated from the planner mirror.
  EXPECT_TRUE(manager.last_cmd_pos_.isApprox(manager.odom_, 1e-9));
}

TEST(GvfSectionCommand,
     RejectedNarrowCandidateRetriesCommittedCurrentBundle) {
  ros::Time::init();
  if (std::getenv("M3C_ISOLATED_ROS_TESTS") == nullptr) {
    GTEST_SKIP() << "set M3C_ISOLATED_ROS_TESTS=1 under a task-owned ROS master";
  }
  FLAG_Race::gvf_manager manager;
  const auto source = makeSectionPath(0.0, 4.0, 51U);
  const auto current_bundle = makeSectionBundle(source, 1U);
  ASSERT_TRUE(source);
  ASSERT_TRUE(current_bundle);
  ASSERT_TRUE(installTestCommandPublisher(manager));
  ASSERT_TRUE(prepareNonzeroSectionCommandManager(
      manager, source, current_bundle));

  const std::uint64_t generation =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(manager);
  const auto narrow_candidate =
      makePartialSectionBundle(source, generation, 0.5);
  ASSERT_TRUE(narrow_candidate);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::stageSectionBundle(
      manager, narrow_candidate,
      std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()));

  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  ASSERT_NE(runtime_before.retained_delta, 0.0);
  ros::TimerEvent event;
  manager.cmdCallback(event);

  const auto phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  // The staged profile cannot cover the normal horizon at w=.40.  The
  // callback must retire only that candidate, evaluate/publish the committed
  // current bundle, and then advance the ordinary Section transaction.
  EXPECT_GT(phase_after.w, phase_before.w);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(
                manager),
            current_bundle);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingSectionBundle(
      manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::hasPending(manager));
  EXPECT_TRUE(std::isfinite(runtime_after.retained_delta));
  EXPECT_GT((manager.last_cmd_pos_ - manager.odom_).norm(), 1e-6);
}

TEST(GvfSectionCommand,
     InvalidCurrentGuidanceCannotReuseRejectedCandidateWhenDeltaIsZero) {
  ros::Time::init();
  if (std::getenv("M3C_ISOLATED_ROS_TESTS") == nullptr) {
    GTEST_SKIP() << "set M3C_ISOLATED_ROS_TESTS=1 under a task-owned ROS master";
  }
  FLAG_Race::gvf_manager manager;
  const auto planner_path = makeSectionPath(0.0, 4.0, 61U);
  const auto planner_bundle = makeSectionBundle(planner_path, 1U);
  ASSERT_TRUE(planner_path);
  ASSERT_TRUE(planner_bundle);
  ASSERT_TRUE(installTestCommandPublisher(manager));
  // Establish a committed zero-delta Section owner and the planner frontend,
  // then replace only the committed bundle with an owner whose state query
  // fails.  The staged candidate still uses the valid planner path, so its
  // nominal guidance is valid before the Section preview rejects it.
  ASSERT_TRUE(installSectionManager(manager, planner_path, planner_bundle));
  const auto invalid_current_path = makeInvalidSectionPath();
  const auto invalid_current_bundle = makeSectionBundle(
      invalid_current_path,
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(manager));
  ASSERT_TRUE(invalid_current_path);
  ASSERT_TRUE(invalid_current_bundle);
  FLAG_Race::GvfManagerS4AnchorTestAccess::replaceCurrentSectionBundle(
      manager, invalid_current_bundle);

  const std::uint64_t generation =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(manager);
  const auto rejected_candidate =
      makePartialSectionBundle(planner_path, generation, 0.50);
  ASSERT_TRUE(rejected_candidate);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::stageSectionBundle(
      manager, rejected_candidate,
      std::shared_ptr<const FLAG_Race::ContinuousPhasePath>(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()));

  ASSERT_FALSE(manager.swarmParticlesManager.empty());
  auto& frontend = manager.swarmParticlesManager.front();
  ASSERT_TRUE(frontend.gvf_);
  frontend.gvf_->gvf_.K1_ = 2.0;
  frontend.gvf_->gvf_.K2_ = -2.2;
  frontend.gvf_->gvf_.convergence_bandwidth_ = 0.1;
  frontend.gvf_->progress_rho0_ = 0.5;
  frontend.gvf_->progress_delta_ = 0.3;
  frontend.gvf_->alpha_min_ = 0.05;
  FLAG_Race::ContinuousPhasePathState planner_state;
  ASSERT_TRUE(planner_path->evaluate(0.40, planner_state, false));
  const FLAG_Race::gvf::LiftedGuidanceResult candidate_guidance =
      frontend.gvf_->calcLiftedGuidanceAtPhase(
          planner_state.p + Eigen::Vector3d(0.0, 0.05, 0.0), 0.40,
          planner_path);
  ASSERT_TRUE(candidate_guidance.valid);
  const FLAG_Race::gvf::LiftedGuidanceResult current_guidance =
      frontend.gvf_->calcLiftedGuidanceAtPhase(
          planner_state.p + Eigen::Vector3d(0.0, 0.05, 0.0), 0.40,
          invalid_current_path);
  EXPECT_FALSE(current_guidance.valid);
  manager.odom_ = planner_state.p + Eigen::Vector3d(0.0, 0.05, 0.0);
  FLAG_Race::GvfManagerS4AnchorTestAccess::configureTerminalExecution(
      manager, 0.40, ros::Time::now());
  manager.cmd_governor_l_min_ = 0.50;
  manager.cmd_governor_l_max_ = 0.50;
  manager.cmd_governor_l_step_ = 0.05;
  manager.cmd_governor_lead_max_ = 1.6;
  manager.cmd_governor_normal_max_ = 0.0;
  manager.cmd_governor_initialized_ = false;
  manager.cmd_governor_normal_state_.setZero();
  manager.cmd_governor_last_l_ = 0.0;
  manager.has_last_governor_cmd_ = false;
  frontend.receive_startpt = false;
  frontend.receive_goal = true;
  frontend.is_first_goal = false;
  frontend.goal_pt = manager.odom_ + Eigen::Vector3d(10.0, 0.0, 0.0);

  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  ASSERT_TRUE(runtime_before.present);
  ASSERT_DOUBLE_EQ(runtime_before.retained_delta, 0.0);

  ros::TimerEvent event;
  manager.cmdCallback(event);

  const auto phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  EXPECT_DOUBLE_EQ(phase_after.w, phase_before.w);
  EXPECT_EQ(phase_after.generation, phase_before.generation);
  EXPECT_DOUBLE_EQ(runtime_after.retained_delta,
                   runtime_before.retained_delta);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(
                manager),
            invalid_current_bundle);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingSectionBundle(
      manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::hasPending(manager));
  // The candidate's guidance was valid, but the committed current owner was
  // not evaluable.  The callback must therefore publish the existing-position
  // hold and never advance phase using the rejected candidate's command.
  EXPECT_TRUE(manager.last_cmd_pos_.isApprox(manager.odom_, 1e-9));
}

TEST(GvfSectionTerminal, NonzeroRuntimeReferenceKeepsGoalTaskActive) {
  ros::Time::init();
  FLAG_Race::gvf_manager manager;
  SectionTransitionFixture fixture;
  ASSERT_TRUE(prepareSectionTransition(manager, fixture, true));
  ASSERT_TRUE(configureTerminalExecution(manager, fixture));

  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  ASSERT_TRUE(runtime_before.present);
  ASSERT_NE(runtime_before.retained_delta, 0.0);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingCommand(manager)
                  .pending);

  ros::TimerEvent event;
  manager.FSMCallback(event);

  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::isExecuting(manager));
  EXPECT_TRUE(manager.swarmParticlesManager.front().receive_goal);
  const auto phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  EXPECT_DOUBLE_EQ(phase_after.w, phase_before.w);
  EXPECT_EQ(phase_after.generation, phase_before.generation);
  EXPECT_DOUBLE_EQ(runtime_after.retained_delta,
                   runtime_before.retained_delta);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(
                manager),
            fixture.source_bundle);
  EXPECT_EQ(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingSectionPathHandoff(
                manager),
            fixture.handoff);
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingCommand(manager)
                  .pending);
}

TEST(GvfSectionTerminal, ZeroRuntimeReferenceRetiresPendingAndEndsTask) {
  ros::Time::init();
  FLAG_Race::gvf_manager manager;
  SectionTransitionFixture fixture;
  ASSERT_TRUE(prepareSectionTransition(manager, fixture, false));
  ASSERT_TRUE(configureTerminalExecution(manager, fixture));

  const std::uint64_t task_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(manager);
  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const auto runtime_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  ASSERT_TRUE(runtime_before.present);
  ASSERT_DOUBLE_EQ(runtime_before.retained_delta, 0.0);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingCommand(manager)
                  .pending);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingSectionPathHandoff(
                  manager));

  ros::TimerEvent event;
  manager.FSMCallback(event);

  EXPECT_TRUE(
      FLAG_Race::GvfManagerS4AnchorTestAccess::isWaitingForTarget(manager));
  EXPECT_FALSE(manager.swarmParticlesManager.front().receive_goal);
  EXPECT_FALSE(manager.swarmParticlesManager.front().is_first_goal);
  EXPECT_GT(FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(
                manager),
            task_before);
  const auto phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  EXPECT_DOUBLE_EQ(phase_after.w, phase_before.w);
  EXPECT_TRUE(phase_after.initialized);
  EXPECT_GT(phase_after.generation, phase_before.generation);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingCommand(manager)
                   .pending);
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::pendingSectionPathHandoff(
      manager));
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::currentSectionBundle(
      manager));
  const auto runtime_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  ASSERT_TRUE(runtime_after.present);
  EXPECT_DOUBLE_EQ(runtime_after.retained_delta, 0.0);
  EXPECT_DOUBLE_EQ(runtime_after.previous_port.u_w, 0.0);
  EXPECT_DOUBLE_EQ(runtime_after.previous_port.u_delta, 0.0);
}

TEST(GvfSectionTerminal, FinalZeroSnapshotFollowedByNonzeroCommitStaysActive) {
  ros::Time::init();
  FLAG_Race::gvf_manager manager;
  SectionTransitionFixture fixture;
  ASSERT_TRUE(prepareSectionTransition(manager, fixture, false));
  ASSERT_TRUE(configureTerminalExecution(manager, fixture));

  FLAG_Race::GvfManagerS4AnchorTestAccess::discardPendingCommand(manager);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setPendingSectionPathHandoff(
      manager, std::shared_ptr<const FLAG_Race::SectionPathHandoff>());
  const std::uint64_t terminal_task_generation =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(manager);
  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager)
                  .present);
  ASSERT_DOUBLE_EQ(
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager)
          .retained_delta,
      0.0);

  // A real Section prepare/publish commits the nonzero successor after the
  // caller captured its zero terminal task witness.
  ASSERT_TRUE(stageSuccessorPending(manager, fixture));
  const auto pending =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePending(manager);
  ASSERT_TRUE(pending.pending);
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::publishPending(
      manager, []() { return true; }, pending.identity));
  const auto runtime_after_commit =
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager);
  ASSERT_NE(runtime_after_commit.retained_delta, 0.0);

  double retained_at_terminal = 0.0;
  EXPECT_FALSE(FLAG_Race::GvfManagerS4AnchorTestAccess::finalizeSectionTerminal(
      manager, terminal_task_generation, retained_at_terminal));
  EXPECT_DOUBLE_EQ(retained_at_terminal,
                   runtime_after_commit.retained_delta);
  EXPECT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::isExecuting(manager));
  EXPECT_TRUE(manager.swarmParticlesManager.front().receive_goal);
  const auto phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  EXPECT_DOUBLE_EQ(phase_after.w, phase_before.w);
  EXPECT_EQ(phase_after.generation, phase_before.generation);
  EXPECT_DOUBLE_EQ(
      FLAG_Race::GvfManagerS4AnchorTestAccess::runtimeState(manager)
          .retained_delta,
      runtime_after_commit.retained_delta);
}

TEST(GvfSectionTerminal, SameTaskPhaseProgressStillAllowsZeroTerminal) {
  ros::Time::init();
  FLAG_Race::gvf_manager manager;
  SectionTransitionFixture fixture;
  ASSERT_TRUE(prepareSectionTransition(manager, fixture, false));
  ASSERT_TRUE(configureTerminalExecution(manager, fixture));

  FLAG_Race::GvfManagerS4AnchorTestAccess::discardPendingCommand(manager);
  FLAG_Race::GvfManagerS4AnchorTestAccess::setPendingSectionPathHandoff(
      manager, std::shared_ptr<const FLAG_Race::SectionPathHandoff>());
  const std::uint64_t terminal_task_generation =
      FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(manager);
  const auto phase_before =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  const double progressed_w = phase_before.w + 0.10;
  ASSERT_LT(progressed_w, fixture.successor->endW());
  FLAG_Race::GvfManagerS4AnchorTestAccess::publishPhase(
      manager, progressed_w, true, false);

  double retained_at_terminal = 0.0;
  ASSERT_TRUE(FLAG_Race::GvfManagerS4AnchorTestAccess::finalizeSectionTerminal(
      manager, terminal_task_generation, retained_at_terminal));
  EXPECT_DOUBLE_EQ(retained_at_terminal, 0.0);
  EXPECT_TRUE(
      FLAG_Race::GvfManagerS4AnchorTestAccess::isWaitingForTarget(manager));
  EXPECT_FALSE(manager.swarmParticlesManager.front().receive_goal);
  const auto phase_after =
      FLAG_Race::GvfManagerS4AnchorTestAccess::capturePhase(manager);
  EXPECT_DOUBLE_EQ(phase_after.w, progressed_w);
  EXPECT_GT(phase_after.generation, phase_before.generation);
  EXPECT_GT(FLAG_Race::GvfManagerS4AnchorTestAccess::executionGeneration(
                manager),
            terminal_task_generation);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
