#include "bspline_race/gvf_manager.h"
#include "bspline_race/integration/phase_offset_tube_markers.h"
#include "bspline_race/continuous_phase_normal_frame.h"
#include <visualization_msgs/Marker.h>

#include <phase_offset_core/geometry.h>

#include <chrono>
#include <exception>
#include <fstream>

namespace FLAG_Race
{
    namespace {
    constexpr double kPathReferenceHandoffTolerance = 1e-10;
    constexpr std::uint64_t kWorldHorizontalFrameConventionId = 1U;
    // Exhausted-frontend handoffs use a backdated seam and a small phase
    // overshoot allowance: the command thread may re-anchor the live phase by
    // a few centimetres after the FSM captured its snapshot.
    constexpr double kExhaustedFrontendSeamBackW = 0.5;
    constexpr double kExhaustedFrontendSeamSlackW = 0.10;

    double steadyTicksAsDouble()
    {
        return static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    PhaseOffsetCoordinationBackend parseCoordinationBackend(
        const std::string& value, bool& valid)
    {
        valid = true;
        if (value == "disabled") {
            return PhaseOffsetCoordinationBackend::DISABLED;
        }
        if (value == "d1b") {
            return PhaseOffsetCoordinationBackend::D1B;
        }
        if (value == "sph") {
            return PhaseOffsetCoordinationBackend::SPH;
        }
        valid = false;
        return PhaseOffsetCoordinationBackend::DISABLED;
    }

    bool coordinationFlagsConsistent(
        const PhaseOffsetCoordinationBackend backend,
        const bool enable_neighbor_transport,
        const bool enable_sph_provider)
    {
        if (enable_neighbor_transport != enable_sph_provider) return false;
        if (backend == PhaseOffsetCoordinationBackend::SPH) {
            return enable_neighbor_transport && enable_sph_provider;
        }
        return !enable_neighbor_transport && !enable_sph_provider;
    }

    int robotIdFromNamespace(const ros::NodeHandle& nh)
    {
        std::string namespace_name = nh.getNamespace();
        std::smatch match;
        const std::regex pattern("/uav_([0-9]+)(?:/|$)");
        if (std::regex_search(namespace_name, match, pattern) &&
            match.size() > 1U) {
            try {
                const long parsed = std::stol(match[1].str());
                if (parsed >= 0L && parsed <= 65535L) {
                    return static_cast<int>(parsed);
                }
            } catch (const std::exception&) {
            }
        }
        return -1;
    }

    bool c2ConnectorEndForSeam(
        const std::shared_ptr<const ContinuousPhasePath>& path,
        const double seam_w,
        double& connector_end_w)
    {
        connector_end_w = 0.0;
        if (!path || path->empty() || !std::isfinite(seam_w)) return false;
        for (const ContinuousPhasePath::Segment& segment : path->segments()) {
            if (segment.label == "c2_quintic" &&
                std::abs(segment.w0 - seam_w) <=
                    kPathReferenceHandoffTolerance &&
                std::isfinite(segment.w1) &&
                segment.w1 > seam_w + kPathReferenceHandoffTolerance) {
                connector_end_w = segment.w1;
                return true;
            }
        }
        return false;
    }

    }


bool gvf_manager::plannerOnlyFutureSeam(
    const std::shared_ptr<const ContinuousPhasePath>& source_path,
    const double phase_at_switch,
    const double construction_lead_w,
    double& seam_w)
{
    seam_w = 0.0;
    if (!source_path || source_path->empty() ||
        !std::isfinite(phase_at_switch) ||
        !std::isfinite(construction_lead_w) ||
        phase_at_switch < source_path->startW() ||
        phase_at_switch >= source_path->endW()) {
        return false;
    }
    const double earliest_seam_w =
        phase_at_switch + std::max(0.0, construction_lead_w);
    if (!std::isfinite(earliest_seam_w)) return false;

    // A mapped B-spline is one outer Segment whose represented map owns many
    // exact internal breakpoints.  Segment::w1 alone sees only the path end
    // and unnecessarily falls back to a current-w CAS.  Use the complete
    // structural partition and require a genuinely interior future seam.
    std::vector<double> breakpoints;
    if (!source_path->certificateBreakpointsV2(breakpoints)) return false;
    for (const double candidate : breakpoints) {
        if (!std::isfinite(candidate) ||
            candidate <= phase_at_switch + kPathReferenceHandoffTolerance ||
            candidate + kPathReferenceHandoffTolerance < earliest_seam_w ||
            candidate >= source_path->endW() - kPathReferenceHandoffTolerance) {
            continue;
        }
        ContinuousPhasePathState seam_state;
        if (source_path->evaluate(candidate, seam_state, false) &&
            seam_state.valid) {
            seam_w = candidate;
            return true;
        }
    }
    return false;
}


bool SectionPathHandoff::complete() const
{
    if (task_generation == 0U || phase_generation == 0U ||
        !candidate_path || candidate_path->empty() || traj.rows() < 2 ||
        traj.cols() != 3 || vel.rows() != traj.rows() || vel.cols() != 3 ||
        time.size() != traj.rows() ||
        w.size() != static_cast<std::size_t>(traj.rows()) ||
        !traj.allFinite() || !vel.allFinite() || !time.allFinite() ||
        path_msg.poses.size() != static_cast<std::size_t>(traj.rows())) {
        return false;
    }
    if (!bundle || bundle->path != candidate_path ||
        bundle->task_generation != task_generation || !bundle->frame ||
        !bundle->profile || !bundle->profile->usable ||
        bundle->path_revision != candidate_path->pathRevision() ||
        bundle->path_revision == 0U) {
        return false;
    }
    for (std::size_t index = 0U; index < w.size(); ++index) {
        if (!std::isfinite(w[index]) ||
            (index != 0U && !(w[index] > w[index - 1U]))) {
            return false;
        }
    }
    const bool start_finite = std::isfinite(copied_prefix_start_w);
    const bool end_finite = std::isfinite(copied_prefix_end_w);
    if (start_finite != end_finite ||
        (start_finite &&
         (!(copied_prefix_end_w > copied_prefix_start_w) ||
          !source_path || source_path->empty() ||
          copied_prefix_start_w < source_path->startW() ||
          copied_prefix_end_w > source_path->endW()))) {
        return false;
    }
    return true;
}

    gvf_manager::AuthoritativePhaseSnapshot
    gvf_manager::captureAuthoritativePhase() const
    {
        std::lock_guard<std::mutex> lock(authoritative_phase_mutex_);
        AuthoritativePhaseSnapshot snapshot;
        snapshot.w = phase_w_;
        snapshot.initialized = phase_initialized_;
        snapshot.closed_acquired = closed_phase_acquired_;
        snapshot.generation = authoritative_phase_generation_;
        return snapshot;
    }

    void gvf_manager::publishAuthoritativePhaseLocked(
        const double w, const bool initialized, const bool closed_acquired)
    {
        phase_w_ = w;
        phase_initialized_ = initialized;
        closed_phase_acquired_ = closed_acquired;
        ++authoritative_phase_generation_;
    }

    void gvf_manager::publishAuthoritativePhase(
        const double w, const bool initialized, const bool closed_acquired)
    {
        std::lock_guard<std::mutex> lock(authoritative_phase_mutex_);
        publishAuthoritativePhaseLocked(w, initialized, closed_acquired);
    }

    bool gvf_manager::commitAuthoritativePhase(
        const AuthoritativePhaseSnapshot& captured, const double w,
        const bool acquire_closed_phase,
        AuthoritativePhaseSnapshot& committed)
    {
        std::lock_guard<std::mutex> lock(authoritative_phase_mutex_);
        if (authoritative_phase_generation_ != captured.generation ||
            phase_initialized_ != captured.initialized ||
            closed_phase_acquired_ != captured.closed_acquired) {
            return false;
        }
        phase_w_ = w;
        phase_initialized_ = captured.initialized;
        closed_phase_acquired_ = captured.closed_acquired ||
            acquire_closed_phase;
        ++authoritative_phase_generation_;
        committed.w = phase_w_;
        committed.initialized = phase_initialized_;
        committed.closed_acquired = closed_phase_acquired_;
        committed.generation = authoritative_phase_generation_;
        return true;
    }

    bool gvf_manager::prepareAuthoritativePhaseCommitLocked(
        const AuthoritativePhaseSnapshot& captured, const double w,
        const bool acquire_closed_phase,
        AuthoritativePhaseCommitToken& token) const
    {
        token = AuthoritativePhaseCommitToken();
        if (!std::isfinite(w) ||
            authoritative_phase_generation_ != captured.generation ||
            phase_w_ != captured.w ||
            phase_initialized_ != captured.initialized ||
            closed_phase_acquired_ != captured.closed_acquired) {
            return false;
        }
        token.expected = captured;
        token.committed.w = w;
        token.committed.initialized = captured.initialized;
        token.committed.closed_acquired = captured.closed_acquired ||
            acquire_closed_phase;
        token.committed.generation = captured.generation + 1U;
        token.valid = true;
        return true;
    }

    void gvf_manager::commitAuthoritativePhaseNoFailLocked(
        const AuthoritativePhaseCommitToken& token) noexcept
    {
        phase_w_ = token.committed.w;
        phase_initialized_ = token.committed.initialized;
        closed_phase_acquired_ = token.committed.closed_acquired;
        authoritative_phase_generation_ = token.committed.generation;
    }

    bool gvf_manager::setPhaseOffsetGDes(const Eigen::Vector3d& g_des)
    {
        std::lock_guard<std::mutex> lock(phase_offset_g_des_mutex_);
        if (!g_des.allFinite()) {
            phase_offset_g_des_.setZero();
            phase_offset_g_des_valid_ = false;
            return false;
        }
        phase_offset_g_des_ = g_des;
        phase_offset_g_des_valid_ = true;
        return true;
    }

    void gvf_manager::clearPhaseOffsetGDes()
    {
        std::lock_guard<std::mutex> lock(phase_offset_g_des_mutex_);
        phase_offset_g_des_.setZero();
        phase_offset_g_des_valid_ = false;
    }

    bool gvf_manager::capturePhaseOffsetGDes(Eigen::Vector3d& g_des) const
    {
        std::lock_guard<std::mutex> lock(phase_offset_g_des_mutex_);
        if (!phase_offset_g_des_valid_ || !phase_offset_g_des_.allFinite()) {
            g_des.setZero();
            return false;
        }
        g_des = phase_offset_g_des_;
        return true;
    }

    gvf_manager::gvf_manager(ros::NodeHandle &nh)
    {
        nh.param("gvf/planInterval", planInterval, -1.0);
        nh.param<std::string>("gvf/cloud_topic", cloud_topic_, "click_map");
        nh.param<std::string>("gvf/odom_topic", odom_topic_, "odom");
        nh.param<std::string>("gvf/cmd_topic", cmd_topic_, "/drone_1_planning/pos_cmd");
        nh.param("gvf/init_bias_x", init_bias_x, -1.0);
        nh.param("gvf/init_bias_y", init_bias_y, -1.0);
        nh.param("gvf/gvf_use_kinopath", use_kinopath_, false);
        nh.param("gvf/num_points_to_take", num_points_to_take_, 10);  // 默认值为10
        nh.param("gvf/exec_timer_interval", exec_timer_interval, 0.2);  // 默认值为0.2秒
        nh.param("gvf/kino_timer_interval", kino_timer_interval, 0.2);  // 默认值为0.2秒
        nh.param("gvf/kino_sample_ts", kino_sample_ts_, 0.2);
        nh.param("gvf/kino_sample_ts_min", kino_sample_ts_min_, 0.05);
        kino_sample_ts_min_ = std::max(1e-3, kino_sample_ts_min_);
        kino_sample_ts_ = std::max(kino_sample_ts_min_, kino_sample_ts_);
        nh.param("planning/safe_distance", safe_distance_, 0.5);  // 添加安全距离参数读取
        nh.param("gvf/collision_threshold", collision_threshold_, 0.05);  // 添加碰撞检测阈值参数读取
        nh.param("gvf/use_test_cmd", use_test_cmd_, false);  // 是否使用测试命令模式


        nh.param("gvf/enable_trajectory_concatenation", enable_trajectory_concatenation_, false);
        nh.param("gvf/max_trajectory_concatenation_points", max_trajectory_concatenation_points_, 50);

        nh.param<std::string>("gvf/closed_tracking_mode", closed_tracking_mode_, std::string("legacy"));
        nh.param("gvf/point_phase_v2/enable", point_phase_v2_enabled_, false);
        nh.param("gvf/point_phase_v2/enable_c2_connector", point_phase_c2_enabled_, false);
        nh.param("gvf/point_phase_v2/endpoint_margin_w", point_phase_endpoint_margin_w_, 0.05);
        point_phase_endpoint_margin_w_ = std::max(1e-3, point_phase_endpoint_margin_w_);
        // Bounded wait before a non-decaying Section offset at the goal is
        // accepted as the terminal equilibrium.  Negative disables the bound.
        nh.param("gvf/point_goal/max_terminal_residual_hold",
                 max_terminal_residual_hold_s_, 2.0);
        nh.param("gvf/point_goal/terminal_offset_slack",
                 point_goal_terminal_offset_slack_, 0.25);
        point_goal_terminal_offset_slack_ =
            std::max(0.0, point_goal_terminal_offset_slack_);
        nh.param("gvf/circle_test/acquire_distance", closed_phase_acquire_distance_, 0.5);
        closed_phase_acquire_distance_ = std::max(0.0, closed_phase_acquire_distance_);
        nh.param("gvf/closed_phase_v2/enable_c2_connector", closed_phase_c2_enabled_, false);
        nh.param("gvf/closed_phase_v2/c2_join_min_w", closed_phase_c2_join_min_w_, 0.5);
        nh.param("gvf/closed_phase_v2/c2_join_max_w", closed_phase_c2_join_max_w_, 1.0);
        nh.param("gvf/closed_phase_v2/c2_join_step_w", closed_phase_c2_join_step_w_, 0.25);
        nh.param("gvf/closed_phase_v2/c2_sample_step_w", closed_phase_c2_sample_step_w_, 0.05);
        nh.param("gvf/closed_phase_v2/back_margin_w", closed_phase_back_margin_w_, 1.0);
        closed_phase_c2_join_min_w_ = std::max(0.05, closed_phase_c2_join_min_w_);
        closed_phase_c2_join_max_w_ = std::max(closed_phase_c2_join_min_w_, closed_phase_c2_join_max_w_);
        closed_phase_c2_join_step_w_ = std::max(0.05, closed_phase_c2_join_step_w_);
        closed_phase_c2_sample_step_w_ = std::max(0.01, closed_phase_c2_sample_step_w_);
        closed_phase_back_margin_w_ = std::max(0.0, closed_phase_back_margin_w_);
        nh.param("gvf/circle_test/enable", enable_circle_reference_test_, false);
        nh.param("gvf/circle_test/auto_start", circle_reference_auto_start_, false);
        nh.param<std::string>("gvf/circle_test/shape", reference_shape_, std::string("circle"));
        nh.param("gvf/circle_test/radius", circle_reference_radius_, 4.0);
        nh.param("gvf/circle_test/figure8_radius", figure8_reference_radius_, 4.0);
        nh.param("gvf/circle_test/height", circle_reference_height_, 1.0);
        nh.param("gvf/circle_test/points", circle_reference_points_, 240);

        nh.param("gvf/circle_test/search_back_w", closed_ref_search_back_w_, 0.3);
        nh.param("gvf/circle_test/search_forward_w", closed_ref_search_forward_w_, 1.5);
        nh.param("gvf/circle_test/lookahead_w", closed_ref_lookahead_w_, 1.5);
        nh.param("gvf/circle_test/lookahead_min_w", closed_ref_lookahead_min_w_, 1.0);
        nh.param("gvf/circle_test/lookahead_max_w", closed_ref_lookahead_max_w_, 3.0);
        nh.param("gvf/circle_test/lookahead_step_w", closed_ref_lookahead_step_w_, 0.5);
        nh.param("gvf/circle_test/enable_global_realign", closed_ref_enable_global_realign_, false);
        nh.param("gvf/circle_test/enable_recover", closed_ref_enable_recover_, false);
        nh.param("gvf/circle_test/ref_lost_radius", closed_ref_lost_radius_, 1.5);
        nh.param("gvf/circle_test/ref_recover_radius", closed_ref_recover_radius_, 1.0);
        nh.param("gvf/circle_test/initial_phase_w", closed_ref_initial_phase_w_, -1.0);
        nh.param("gvf/circle_test/ref_phase_k1", ref_phase_k1_, 2.0);
        nh.param("gvf/circle_test/ref_alpha_rho", ref_alpha_rho_, 1.0);
        nh.param("gvf/circle_test/ref_sigma_scale", ref_sigma_scale_, 1.0);
        nh.param("gvf/circle_test/ref_wdot_forward_max", ref_wdot_forward_max_, 3.0);
        nh.param("gvf/circle_test/ref_wdot_backward_max", ref_wdot_backward_max_, 1.5);
        nh.param("gvf/circle_test/ref_project_blend", ref_project_blend_, 0.2);
        nh.param("gvf/circle_test/ref_project_snap_max", ref_project_snap_max_, 0.5);
        nh.param("gvf/circle_test/ref_project_boundary_eps", ref_project_boundary_eps_, 0.03);
        nh.param("gvf/circle_test/goal_full_success_tol", closed_goal_full_success_tol_, 0.3);
        nh.param("gvf/circle_test/goal_prefer_lookahead_w", closed_goal_prefer_lookahead_w_, 2.0);
        nh.param("gvf/circle_test/progressive_lookahead_extra_w",
                 closed_goal_progressive_lookahead_extra_w_, 0.75);
        nh.param("gvf/circle_test/progressive_min_progress_w",
                 closed_goal_progressive_min_progress_w_, 0.6);
        nh.param("gvf/circle_test/progressive_max_progress_w",
                 closed_goal_progressive_max_progress_w_, 0.8);
        nh.param("gvf/circle_test/progressive_progress_time",
                 closed_goal_progressive_progress_time_, 1.0);
        nh.param("gvf/circle_test/goal_lookahead_weight", closed_goal_lookahead_weight_, 5.0);
        nh.param("gvf/circle_test/goal_end_dist_weight", closed_goal_end_dist_weight_, 20.0);
        nh.param("gvf/circle_test/goal_push_past_obstacle", closed_goal_push_past_obstacle_, false);
        nh.param("gvf/circle_test/goal_obstacle_check_step_w", closed_goal_obstacle_check_step_w_, 0.1);
        nh.param("gvf/circle_test/goal_obstacle_pass_margin_w", closed_goal_obstacle_pass_margin_w_, 0.8);

        nh.param("gvf/circle_test/center_x", circle_reference_center_x_, 0.0);
        nh.param("gvf/circle_test/center_y", circle_reference_center_y_, 0.0);
        nh.param("gvf/circle_test/center_z", circle_reference_center_z_, 0.0);


        nh.param("gvf/slow_radius", slow_radius, 1.0);
        nh.param("gvf/stop_radius", stop_radius, 0.3);
        nh.param("gvf/cmd/vel_max", cmd_vel_max_, 1.5);
        nh.param("gvf/cmd/acc_max", cmd_acc_max_, 1.5);
        nh.param("gvf/cmd/pos_gain_equiv", cmd_pos_gain_equiv_, 1.10);
        nh.param("gvf/cmd/switch_motion_limit_time", cmd_switch_motion_limit_time_, 0.25);
        nh.param("gvf/cmd/tangent_vel_max", cmd_tangent_vel_max_, 2.0);
        nh.param("gvf/cmd/governor_l_min", cmd_governor_l_min_, 0.0);
        nh.param("gvf/cmd/governor_l_max", cmd_governor_l_max_, 1.6);
        nh.param("gvf/cmd/governor_l_step", cmd_governor_l_step_, 0.05);
        nh.param("gvf/cmd/governor_l_rate_max", cmd_governor_l_rate_max_, 4.0);
        nh.param("gvf/cmd/governor_l_ff_weight", cmd_governor_l_ff_weight_, 0.8);
        nh.param("gvf/cmd/governor_lead_max", cmd_governor_lead_max_, 1.6);
        nh.param("phase_offset/recovery_from_hold_enable",
                 recovery_from_hold_enable_, false);
        nh.param("phase_offset/recovery_delta_step",
                 recovery_delta_step_, 0.05);
        nh.param("gvf/cmd/governor_unlock_timeout",
                 cmd_governor_unlock_timeout_s_, 1.0);
        nh.param("gvf/cmd/governor_normal_cross_max", cmd_governor_normal_cross_max_, 0.08);
        nh.param("gvf/cmd/governor_normal_deadband", cmd_governor_normal_deadband_, 0.05);
        nh.param("gvf/cmd/governor_normal_full_error", cmd_governor_normal_full_error_, 0.35);
        nh.param("gvf/cmd/governor_normal_max", cmd_governor_normal_max_, 0.0);
        nh.param("gvf/cmd/governor_normal_rate_max", cmd_governor_normal_rate_max_, 0.6);
        nh.param("gvf/cmd/governor_l_rate_weight", cmd_governor_l_rate_weight_, 0.005);
        nh.param("gvf/cmd/governor_normal_weight", cmd_governor_normal_weight_, 0.60);
        nh.param("gvf/cmd/governor_normal_rate_weight", cmd_governor_normal_rate_weight_, 0.10);
        nh.param("gvf/cmd/governor_tau_vel_weight", cmd_governor_tau_vel_weight_, 1.0);
        nh.param("gvf/cmd/governor_normal_vel_weight", cmd_governor_normal_vel_weight_, 1.0);
        nh.param("gvf/cmd/governor_normal_vel_error_cap", cmd_governor_normal_vel_error_cap_, 2.0);
        nh.param("gvf/switch/governor_path_margin_w", switch_governor_path_margin_w_, 0.2);

        nh.param("gvf/cmd/gain_test_enable", cmd_gain_test_enable_, false);
        nh.param("gvf/cmd/gain_test_lead", cmd_gain_test_lead_, 0.4);
        nh.param("gvf/cmd/gain_test_axis", cmd_gain_test_axis_, 0);
        nh.param("gvf/odom_vel_est_window", odom_vel_est_window_, 0.3);
        nh.param("gvf/odom_vel_lpf_hz", odom_vel_lpf_hz_, 2.0);

        bool phase_offset_measurement_requested = false;
        nh.param("phase_offset/measurement/enable",
                 phase_offset_measurement_requested, false);
        nh.param<std::string>("phase_offset/measurement/callback_csv_path",
                              phase_offset_measurement_callback_csv_path_,
                              std::string());
        phase_offset_measurement_callback_enabled_ =
            phase_offset_measurement_requested &&
            !phase_offset_measurement_callback_csv_path_.empty();
        if (phase_offset_measurement_callback_enabled_)
        {
            phase_offset_measurement_callback_samples_.reserve(20000U);
        }

        // Resolve the sole main-side coordination selector before constructing
        // any phase-offset adapter.  Missing/invalid selectors and any
        // mismatch with the two Provider/transport flags fail closed to the
        // disabled production coordination path.
        std::string coordination_backend_name = "disabled";
        nh.param<std::string>("phase_offset/coordination_backend",
                              coordination_backend_name, "disabled");
        nh.param("enable_neighbor_transport", enable_neighbor_transport_,
                 false);
        nh.param("enable_sph_provider", enable_sph_provider_, false);
        bool selector_valid = false;
        coordination_backend_ = parseCoordinationBackend(
            coordination_backend_name, selector_valid);
        coordination_backend_config_valid_ = selector_valid &&
            coordinationFlagsConsistent(coordination_backend_,
                                        enable_neighbor_transport_,
                                        enable_sph_provider_);
        if (!coordination_backend_config_valid_) {
            ROS_ERROR(
                "[phase_offset] invalid coordination selector/Provider flags "
                "(backend='%s' neighbor=%d provider=%d); coordination "
                "failed closed to disabled",
                coordination_backend_name.c_str(),
                enable_neighbor_transport_ ? 1 : 0,
                enable_sph_provider_ ? 1 : 0);
            coordination_backend_ = PhaseOffsetCoordinationBackend::DISABLED;
        }

        std::string phase_offset_mode = "disabled";
        nh.param<std::string>("phase_offset/mode", phase_offset_mode, "disabled");
        if (phase_offset_mode == "shadow")
        {
            if (coordination_backend_ == PhaseOffsetCoordinationBackend::SPH) {
                ROS_ERROR(
                    "[phase_offset] SPH coordination requires MANUAL mode; "
                    "coordination failed closed to disabled");
                coordination_backend_ = PhaseOffsetCoordinationBackend::DISABLED;
                coordination_backend_config_valid_ = false;
            }
            ShadowConfig shadow_config;
            shadow_config.mode = PhaseOffsetShadowMode::SHADOW;
            nh.param("phase_offset/shadow_delta", shadow_config.delta, 0.0);
            nh.param("phase_offset/shadow_sample_step_w", shadow_config.sample_step_w, 0.10);
            nh.param("phase_offset/shadow_publish_rate", shadow_config.publish_rate, 5.0);
            nh.param<std::string>("phase_offset/frame_id", shadow_config.frame_id, "world");
            phase_offset_shadow_adapter_.reset(
                new PhaseOffsetShadowAdapter(shadow_config));
            phase_offset_shadow_adapter_->advertise(nh);
            ROS_INFO("[phase_offset_shadow] enabled delta=%.3f sample_step_w=%.3f publish_rate=%.2f",
                     shadow_config.delta,
                     shadow_config.sample_step_w,
                     shadow_config.publish_rate);
        }
        else if (phase_offset_mode == "active" || phase_offset_mode == "manual")
        {
            const PhaseOffsetMatchedMode matched_mode = phase_offset_mode == "active"
                ? PhaseOffsetMatchedMode::ACTIVE : PhaseOffsetMatchedMode::MANUAL;
            PhaseOffsetMatchedAdapterConfig matched_config =
                PhaseOffsetMatchedAdapter::loadConfig(nh, matched_mode);
            // SPH is a real C1/C2 production path only through MANUAL with a
            // non-observe-only adapter.  Any other phase-offset mode is
            // retained for compatibility but cannot activate SPH control.
            if (coordination_backend_ == PhaseOffsetCoordinationBackend::SPH &&
                (matched_mode != PhaseOffsetMatchedMode::MANUAL ||
                 matched_config.observe_only)) {
                ROS_ERROR(
                    "[phase_offset] SPH coordination requires MANUAL "
                    "observe_only=false; coordination failed closed to disabled");
                coordination_backend_ = PhaseOffsetCoordinationBackend::DISABLED;
                coordination_backend_config_valid_ = false;
            }
            matched_config.coordination_backend = coordination_backend_;
            matched_config_ = matched_config;
            phase_offset_matched_adapter_.reset(
                new PhaseOffsetMatchedAdapter(matched_config));
            if (!phase_offset_matched_adapter_->configurationValid())
            {
                phase_offset_matched_adapter_.reset();
                if (coordination_backend_ ==
                    PhaseOffsetCoordinationBackend::SPH) {
                    phase_offset_sph_ros_bridge_.reset();
                    coordination_backend_ =
                        PhaseOffsetCoordinationBackend::DISABLED;
                    coordination_backend_config_valid_ = false;
                    matched_config_.coordination_backend =
                        PhaseOffsetCoordinationBackend::DISABLED;
                }
                ROS_ERROR("[phase_offset_%s] invalid gate or manual-port configuration; mode disabled",
                          phase_offset_mode.c_str());
            }
            else
            {
                if (coordination_backend_ == PhaseOffsetCoordinationBackend::SPH) {
                    nh.param("robot_id", phase_offset_robot_id_, -1);
                    if (phase_offset_robot_id_ < 0) {
                        phase_offset_robot_id_ = robotIdFromNamespace(nh);
                    }
                    if (phase_offset_robot_id_ < 0) {
                        ROS_ERROR(
                            "[phase_offset] SPH coordination requires a "
                            "non-negative robot_id; coordination failed "
                            "closed to disabled");
                        coordination_backend_ =
                            PhaseOffsetCoordinationBackend::DISABLED;
                        coordination_backend_config_valid_ = false;
                    } else {
                        bspline_race::integration::PhaseOffsetSphRosBridgeConfig
                            bridge_config;
                        bridge_config.robot_id = phase_offset_robot_id_;
                        bridge_config.frame_id = matched_config.frame_id;
                        nh.param<std::string>(
                            "phase_offset/sph_beta_topic",
                            bridge_config.beta_topic, std::string());
                        nh.param<std::string>(
                            "phase_offset/sph_g_coord_topic",
                            bridge_config.g_coord_topic, std::string());
                        nh.param("phase_offset/g_coord_fresh_timeout",
                                 bridge_config.g_coord_fresh_timeout,
                                 bridge_config.g_coord_fresh_timeout);
                        nh.param("phase_offset/future_timestamp_tolerance",
                                 bridge_config.future_timestamp_tolerance,
                                 bridge_config.future_timestamp_tolerance);
                        try {
                            phase_offset_sph_ros_bridge_.reset(
                                new bspline_race::integration::PhaseOffsetSphRosBridge(
                                    nh, bridge_config));
                        } catch (const std::exception& error) {
                            ROS_ERROR(
                                "[phase_offset] SPH bridge construction failed: %s; "
                                "coordination failed closed to disabled",
                                error.what());
                            coordination_backend_ =
                                PhaseOffsetCoordinationBackend::DISABLED;
                            coordination_backend_config_valid_ = false;
                        }
                    }
                }
                // A robot-id/bridge failure can downgrade the effective
                // backend after the initial adapter was constructed. Rebuild
                // it with the disabled value before advertising so no SPH
                // configuration remains live behind a disabled manager.
                if (coordination_backend_ !=
                    matched_config.coordination_backend) {
                    matched_config.coordination_backend = coordination_backend_;
                    matched_config_ = matched_config;
                    phase_offset_matched_adapter_.reset(
                        new PhaseOffsetMatchedAdapter(matched_config));
                }
                if (!phase_offset_matched_adapter_ ||
                    !phase_offset_matched_adapter_->configurationValid()) {
                    phase_offset_matched_adapter_.reset();
                    ROS_ERROR(
                        "[phase_offset_%s] downgraded coordination adapter "
                        "configuration is invalid; mode disabled",
                        phase_offset_mode.c_str());
                } else {
                    phase_offset_matched_adapter_->advertise(nh);
                if (matched_config.mode == PhaseOffsetMatchedMode::ACTIVE)
                {
                ROS_INFO("[phase_offset_active] zero-port mode enabled tolerance=%.3e cycles=%d",
                         matched_config.equivalence_tolerance, matched_config.warmup_cycles);
                }
                else
                {
                    ROS_INFO("[phase_offset_manual] enabled amplitude=%.3f profile_period=%.2f cycles=%d",
                             matched_config.amplitude, matched_config.profile_period,
                             matched_config.warmup_cycles);
                }
                }
            }
        }
        else if (phase_offset_mode != "disabled")
        {
            ROS_ERROR("[phase_offset] unsupported mode '%s'; phase-offset disabled",
                      phase_offset_mode.c_str());
            if (coordination_backend_ == PhaseOffsetCoordinationBackend::SPH) {
                coordination_backend_ = PhaseOffsetCoordinationBackend::DISABLED;
                coordination_backend_config_valid_ = false;
            }
        }
        else if (coordination_backend_ == PhaseOffsetCoordinationBackend::SPH)
        {
            ROS_ERROR(
                "[phase_offset] SPH coordination requires MANUAL mode; "
                "coordination failed closed to disabled");
            coordination_backend_ = PhaseOffsetCoordinationBackend::DISABLED;
            coordination_backend_config_valid_ = false;
        }

	        nh.param("gvf/collision_check_horizon_pts", collision_check_horizon_pts_, 120);
	        nh.param("gvf/collision_consecutive_hits", collision_consecutive_hits_, 3);

        nh.param("gvf/goal_reach_radius", goal_reach_radius_, 2.0);
        nh.param("gvf/start_pt_change_threshold", start_pt_change_threshold_, 1.0);


        // nh.param("gvf/flight_height", flight_height_, 1.0);  // 设定飞行高度
        last_replan_time_ = ros::Time(0);  // 初始化上次重规划时间
        cmd_switch_motion_limit_until_ = ros::Time(0);
        current_traj_index_ = 0;  // 初始化当前轨迹索引
        test_traj_index_ = 0;  // 初始化测试轨迹索引
        last_yaw = 0.0;  // 初始化yaw角度

        exec_state_ = WAIT_TARGET;

        initCallback(nh);
        InitGvf(nh);
    }
    gvf_manager::~gvf_manager()
    {
        if (phase_offset_matched_adapter_)
        {
            phase_offset_matched_adapter_->requestShutdown();
        }
        // Stop every manager-owned timer before detaching the Section handoff.
        exec_fsm_timer.stop();
        kino_timer.stop();
        test_cmd_timer.stop();
        exec_timer.stop();
        cmd_timer.stop();
        {
            std::lock_guard<std::mutex> apply_lock(frontend_apply_mutex_);
            std::lock_guard<std::mutex> handoff_lock(path_reference_handoff_mutex_);
            pending_section_path_handoff_.reset();
        }
        if (phase_offset_matched_adapter_)
        {
            phase_offset_matched_adapter_->shutdown();
        }
        flushCallbackTiming();
    }

    void gvf_manager::recordCallbackTiming(
        const std::uint64_t steady_duration_ns,
        const std::uint64_t ros_stamp_ns)
    {
        if (!phase_offset_measurement_callback_enabled_) return;
        std::lock_guard<std::mutex> lock(phase_offset_measurement_callback_mutex_);
        CallbackTimingSample sample;
        sample.sequence = phase_offset_measurement_callback_sequence_++;
        sample.steady_duration_ns = steady_duration_ns;
        sample.ros_stamp_ns = ros_stamp_ns;
        phase_offset_measurement_callback_samples_.push_back(sample);
    }

    void gvf_manager::flushCallbackTiming()
    {
        if (!phase_offset_measurement_callback_enabled_) return;
        std::vector<CallbackTimingSample> samples;
        {
            std::lock_guard<std::mutex> lock(phase_offset_measurement_callback_mutex_);
            samples.swap(phase_offset_measurement_callback_samples_);
        }
        if (samples.empty()) return;
        std::ofstream stream(phase_offset_measurement_callback_csv_path_.c_str(),
                             std::ios::out | std::ios::trunc);
        if (!stream.is_open()) return;
        stream << "kind,sequence,steady_duration_ns,ros_stamp_ns\n";
        for (const CallbackTimingSample& sample : samples)
        {
            stream << "callback," << sample.sequence << ','
                   << sample.steady_duration_ns << ',' << sample.ros_stamp_ns
                   << '\n';
        }
    }

    void gvf_manager::initCallback(ros::NodeHandle &nh)
    {
        // exec_timer = nh.createTimer(ros::Duration(0.02), &gvf_manager::execTimerCallback, this);  // 修改为execTimerCallback，频率0.02s
        force_pub  = nh.advertise<common_msgs::Swarm_particles>("/gvf_force", 10);
        goal_sub = nh.subscribe("/move_base_simple/goal", 1000, &gvf_manager::goalCallback, this);
        path_vis = nh.advertise<visualization_msgs::Marker>("/path_vis", 10);
        odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 10, &gvf_manager::odomCallback, this);
        cmd_pub    = nh.advertise<quadrotor_msgs::PositionCommand>(cmd_topic_, 10);
        cmd_timer  = nh.createTimer(ros::Duration(0.02), &gvf_manager::cmdCallback, this);  // 50Hz
        test_cmd_timer = nh.createTimer(ros::Duration(0.02), &gvf_manager::test_cmdCallback, this);  // 50Hz 测试轨迹跟踪
        path_pub = nh.advertise<nav_msgs::Path>("/particle0/path", 1);
        kino_path_pub = nh.advertise<nav_msgs::Path>("/particle0/kinopath", 1);  // 初始化新发布者
        kino_timer = nh.createTimer(ros::Duration(0.2), &gvf_manager::KinoPathCallback, this);  // 初始化新定时器
        goal_vis_pub = nh.advertise<visualization_msgs::Marker>("/goal_vis", 10);  // 初始化目标点可视化发布者

        exec_fsm_timer = nh.createTimer(ros::Duration(0.02), &gvf_manager::FSMCallback, this);  // FSM 状态机定时器

        circle_ref_pub_ = nh.advertise<nav_msgs::Path>("/particle0/circle_reference", 1, true);
        // Display-only Section tube boundaries.  The topic is refreshed on
        // bundle change plus a slow heartbeat from the planning thread.  It
        // stays a fixed name; the per-agent SIM-B launch remaps it to
        // /uav_N/phase_offset_section_tube so independent UAVs cannot
        // overwrite each other's corridor markers.
        section_tube_marker_pub_ =
            nh.advertise<visualization_msgs::MarkerArray>(
                "/phase_offset_section_tube", 1);

        // Display-only executed reference for the Section offset: p(w) +
        // N(w) * delta.  The per-agent launch remaps this fixed name to
        // /uav_N/phase_offset_adjusted_path so independent UAVs do not
        // overwrite each other's intent-adjusted centreline.
        adjusted_path_pub_ =
            nh.advertise<visualization_msgs::Marker>(
                "/phase_offset_adjusted_path", 1);

	        // nh.param("gvf/debug_gate", debug_gate_, false);

	        // ROS_INFO("[GVF] debug_gate=%s (param: ~gvf/debug_gate)", debug_gate_ ? "true" : "false");
	    } 

void gvf_manager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("[GVF] receive goal (%.2f, %.2f, %.2f)",
             msg->pose.position.x,
             msg->pose.position.y,
             msg->pose.position.z);

    // 发布目标点可视化
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_visualization";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置目标点的位置
    marker.pose.position.x = msg->pose.position.x;
    marker.pose.position.y = msg->pose.position.y;
    marker.pose.position.z = msg->pose.position.z+1.0;
    
    // 设置目标点的方向
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    
    // 设置目标点的尺寸
    marker.scale.x = 0.2;  // 球体直径
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    
    // 设置目标点的颜色为红色
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;  // 不透明
    
    // 发布标记
    goal_vis_pub.publish(marker);

    Eigen::Vector3d start_pt(odom_.x()+0.000001, odom_.y()+0.000001,1.0);
    Eigen::Vector3d goal_pt(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z+1.0
    );


    progress_w_ = 0.0;
    progress_initialized_ = false;
    if (!resetForNewNavigationTask()) {
        ROS_ERROR("[GVF][NEW_TASK_AUTHORITY_RESET] rejected; goal is not installed");
        return;
    }
    // The reset queued the existing FSM-owned clear mailbox; goalCallback
    // does not install or clear a frontend directly.
    closed_ref_w_ = 0.0;
    closed_ref_initialized_ = false;
    closed_ref_recover_ = false;
    resetClosedGoalCandidateState();
    ref_pos = start_pt;
    last_curve_vel_.setZero();
    has_last_curve_vel_ = false;
    cmd_switch_motion_limit_until_ = ros::Time(0);
    last_governor_cmd_pos_ = start_pt;
    last_governor_cmd_vel_.setZero();
    has_last_governor_cmd_ = false;
    cmd_governor_normal_state_.setZero();
    cmd_governor_initialized_ = false;
    cmd_governor_last_l_ = 0.0;
    ref_initialized = false;
    last_cmd_pos_ = start_pt;

    if (enable_circle_reference_test_) {
        if (reference_shape_ == "figure8" || reference_shape_ == "8" || reference_shape_ == "lemniscate") {
            generateFigureEightReference(goal_pt);
        } else {
            generateCircleReference(goal_pt);
        }
        if (circle_reference_ready_ && !closedPhaseV2Enabled()) {
            auto circle_goal = getCircleReferenceGoal(start_pt);
            goal_pt = circle_goal.first;
        }
    } else {
        circle_reference_ready_ = false;
        circle_reference_traj_.resize(0, 0);
        circle_reference_vel_.resize(0, 0);
        circle_reference_w_.clear();
        circle_reference_total_w_ = 0.0;
        circle_reference_progress_anchor_w_ = 0.0;
        circle_reference_index_ = 0;
        closed_ref_w_ = 0.0;
        closed_ref_initialized_ = false;
        closed_ref_recover_ = false;
        resetClosedGoalCandidateState();
    }

    for (auto& manager : swarmParticlesManager) {
        manager.receive_startpt = true;
        manager.start_pt = start_pt;
        manager.goal_pt = goal_pt;
        manager.is_first_goal = true;  // 重置标志位
        manager.receive_goal = true;
    }

}

bool gvf_manager::pathPointAtW(
                               const std::shared_ptr<const ContinuousPhasePath>& path,
                               double query_w,
                               double reference_delta,
                               Eigen::Vector3d& point,
                               bool& clamped_to_end,
                               double& path_w_start,
                               double& path_w_end) const
{
    point.setZero();
    clamped_to_end = false;
    path_w_start = 0.0;
    path_w_end = 0.0;

    if (!path || !std::isfinite(query_w) || !std::isfinite(reference_delta))
    {
        return false;
    }

    if (path->empty())
    {
        return false;
    }
    path_w_start = path->startW();
    path_w_end = path->endW();
    if (!std::isfinite(path_w_start) || !std::isfinite(path_w_end) ||
        path_w_end <= path_w_start)
    {
        return false;
    }
    const double clamped_w = std::max(path_w_start, std::min(query_w, path_w_end));
    clamped_to_end = query_w > path_w_end;
    ContinuousPhasePathState continuous_state;
    if (!path->evaluate(clamped_w, continuous_state, false))
    {
        return false;
    }
    const phase_offset_core::PathDifferentialState active_path =
        ConvertContinuousPhasePathStateForActive(continuous_state, clamped_w);
    phase_offset_core::PhaseOffsetGeometryState geometry;
    phase_offset_core::GeometryEvaluator evaluator;
    if (!evaluator.evaluate(active_path, Eigen::Vector3d::Zero(),
                            reference_delta, geometry))
    {
        return false;
    }
    // Geometry validation is mandatory even at zero delta.  The direct base
    // result preserves the original calculation exactly for observe-only.
    point = reference_delta == 0.0 ? active_path.p : geometry.r;
    return true;
}

bool gvf_manager::pathTangentAtW(
                                 const std::shared_ptr<const ContinuousPhasePath>& path,
                                 double query_w,
                                 double reference_delta,
                                 Eigen::Vector3d& tangent) const
{
    tangent.setZero();
    if (!path || !std::isfinite(query_w) || !std::isfinite(reference_delta))
    {
        return false;
    }

    if (path->empty())
    {
        return false;
    }
    const double path_w_start = path->startW();
    const double path_w_end = path->endW();
    if (!std::isfinite(path_w_start) || !std::isfinite(path_w_end) ||
        path_w_end <= path_w_start)
    {
        return false;
    }
    const double clamped_w = std::max(path_w_start, std::min(query_w, path_w_end));
    ContinuousPhasePathState continuous_state;
    if (!path->evaluate(clamped_w, continuous_state, false))
    {
        return false;
    }
    const phase_offset_core::PathDifferentialState active_path =
        ConvertContinuousPhasePathStateForActive(continuous_state, clamped_w);
    phase_offset_core::PhaseOffsetGeometryState geometry;
    phase_offset_core::GeometryEvaluator evaluator;
    if (!evaluator.evaluate(active_path, Eigen::Vector3d::Zero(),
                            reference_delta, geometry))
    {
        return false;
    }
    if (reference_delta == 0.0)
    {
        // Match gvf::evalTangentByW's zero-port normalization exactly.
        const double path_speed = active_path.p_w.norm();
        if (path_speed <= 1e-6)
        {
            return false;
        }
        tangent = active_path.p_w.normalized();
    }
    else
    {
        tangent = geometry.T;
    }
    return tangent.norm() > 1e-6;
}

void gvf_manager::resetGovernorState()
{
    cmd_governor_normal_state_.setZero();
    cmd_governor_initialized_ = false;
    cmd_governor_last_l_ = 0.0;
}

void gvf_manager::clearActiveTrajectory(bool publish_empty_path,
                                        bool clear_gvf_path)
{
    {
        std::lock_guard<std::mutex> lock(path_reference_handoff_mutex_);
        pending_section_path_handoff_.reset();
    }
    // Retire the published Section tube display with the trajectory it
    // belonged to.  The FSM republishes (or DELETEs) on its next tick.
    last_section_marker_bundle_.reset();
    last_section_marker_time_ = ros::Time(0);
    // In active H2 execution the asynchronous goal/reset callback must not
    // write frontend mirrors concurrently with FSM.  Reset has retired the
    // handoff above; FSM will own the ensuing mirror
    // application/clear.  Legacy modes retain their original immediate clear.
    if (unifiedPhaseV2Active() && phase_offset_matched_adapter_ &&
        phase_offset_matched_adapter_->requiresAuthoritativeOffsetHandoff()) {
        return;
    }
    for (auto& manager : swarmParticlesManager) {
        manager.last_traj.resize(0, 3);
        manager.last_vel.resize(0, 3);
        manager.last_traj_time_.resize(0);
        if (clear_gvf_path && manager.gvf_) {
            manager.gvf_->clearPathReparamState();
        }
    }

    current_traj_index_ = 0;
    test_traj_index_ = 0;
    resetGovernorState();

    if (publish_empty_path) {
        nav_msgs::Path empty_path;
        empty_path.header.frame_id = "world";
        empty_path.header.stamp = ros::Time::now();
        path_pub.publish(empty_path);
        kino_path_pub.publish(empty_path);
    }
}

gvf_manager::GovernorCommandResult
gvf_manager::makeGovernorInvalidHold(const Eigen::Vector3d& pos,
                                     const std::string& reason,
                                     GovernorCommandDebug& dbg)
{
    GovernorCommandResult result;
    result.cmd_pos = pos;
    result.yaw_cmd_vec.setZero();
    result.final_cmd_source = "GOVERNOR_INVALID_HOLD";
    result.fallback_reason = reason;
    result.reset_state_after_publish = true;
    dbg.fallback_hold_pos = true;
    dbg.normal_state_norm = 0.0;
    return result;
}

gvf_manager::GovernorCommandResult
gvf_manager::runVelocityMatchingGovernor(
                                         const std::shared_ptr<const ContinuousPhasePath>& authoritative_path,
                                         const gvf::LiftedGuidanceResult& out,
                                         const Eigen::Vector3d& pos,
                                         double progress_w_after,
                                         double reference_delta,
                                         double dt,
                                         double kp_equiv,
                                         GovernorCommandDebug& dbg,
                                         const phase_offset_navigation::ImmutableExecutedReferenceQueryPtr& executed_reference_query)
{
    GovernorCommandResult result;
    result.cmd_pos = pos;
    result.yaw_cmd_vec.setZero();
    dbg.guidance_valid = true;
    dbg.e_perp_norm = out.e_perp.norm();

    const Eigen::Vector3d raw_v = out.v_cmd;
    dbg.raw_v_norm = raw_v.norm();
    const double tangent_norm = out.tangent.norm();
    double executed_domain_start = 0.0;
    double executed_domain_end = 0.0;
    // A retained Section bundle can leave its domain behind the advancing
    // phase: when the local obstacle view is momentarily unknown the profile is
    // not rebuilt, and its predecessor's window eventually stops covering the
    // live phase.  Every candidate query then falls outside the domain, the
    // governor holds with no candidate, and the task never resumes.  Treat
    // "domain does not cover the live phase" as "no executed domain" so the
    // planner path (with the retained reference offset) carries the tick.
    constexpr double kExecutedDomainPhaseEps = 1e-6;
    const bool executed_domain_ready = executed_reference_query &&
        executed_reference_query->domain(executed_domain_start,
                                         executed_domain_end) &&
        std::isfinite(executed_domain_start) &&
        std::isfinite(executed_domain_end) &&
        executed_domain_end > executed_domain_start &&
        progress_w_after >= executed_domain_start - kExecutedDomainPhaseEps &&
        progress_w_after <= executed_domain_end + kExecutedDomainPhaseEps;
    const bool path_ready = executed_domain_ready || (authoritative_path &&
        !authoritative_path->empty() &&
        std::isfinite(authoritative_path->startW()) &&
        std::isfinite(authoritative_path->endW()) &&
        authoritative_path->endW() > authoritative_path->startW());
    if (!path_ready)
    {
        return makeGovernorInvalidHold(pos, "path_invalid", dbg);
    }
    if (tangent_norm <= 1e-6)
    {
        return makeGovernorInvalidHold(pos, "tangent_invalid", dbg);
    }

    const Eigen::Vector3d t_current = out.tangent / tangent_norm;
    dbg.raw_v_tau = raw_v.dot(t_current);
    const Eigen::Vector3d raw_v_t = dbg.raw_v_tau * t_current;
    const Eigen::Vector3d v_n_intent = raw_v - raw_v_t;
    dbg.raw_v_normal_norm = v_n_intent.norm();
    dbg.v_tau_intent = std::max(0.0, dbg.raw_v_tau);
    const double tangent_vel_limit = std::max(0.0, cmd_tangent_vel_max_);
    if (tangent_vel_limit > 1e-6)
    {
        dbg.v_tau_intent = std::min(dbg.v_tau_intent, tangent_vel_limit);
    }
    dbg.v_n_intent_norm = v_n_intent.norm();

    auto projectNormal = [](const Eigen::Vector3d& v, const Eigen::Vector3d& t) {
        return v - v.dot(t) * t;
    };
    auto limitNormalMagnitude = [](Eigen::Vector3d v, double max_norm) {
        const double n = v.norm();
        if (max_norm <= 1e-6)
        {
            v.setZero();
        }
        else if (n > max_norm && n > 1e-6)
        {
            v *= max_norm / n;
        }
        return v;
    };
    auto enforceCrossInPlane = [&](Eigen::Vector3d v,
                                   const Eigen::Vector3d& t,
                                   const Eigen::Vector3d& e_hat,
                                   bool has_e_hat,
                                   double cross_max) {
        if (has_e_hat)
        {
            const double min_dot = -std::max(0.0, cross_max);
            const double d = v.dot(e_hat);
            if (d < min_dot)
            {
                const Eigen::Vector3d e_hat_plane = projectNormal(e_hat, t);
                const double denom = e_hat_plane.dot(e_hat);
                if (denom > 1e-6)
                {
                    v += ((min_dot - d) / denom) * e_hat_plane;
                }
            }
        }
        return v;
    };
    auto constrainNormal = [&](Eigen::Vector3d n,
                               const Eigen::Vector3d& t,
                               const Eigen::Vector3d& e_hat,
                               bool has_e_hat,
                               double normal_max) {
        n = projectNormal(n, t);
        n = limitNormalMagnitude(n, normal_max);
        n = enforceCrossInPlane(n, t, e_hat, has_e_hat, cmd_governor_normal_cross_max_);
        n = projectNormal(n, t);
        n = limitNormalMagnitude(n, normal_max);
        return n;
    };

    const double l_min = std::max(0.0, cmd_governor_l_min_);
    const double l_max = std::max(l_min, cmd_governor_l_max_);
    const double l_step = std::max(1e-3, cmd_governor_l_step_);
    dbg.l_ff = std::max(l_min, std::min(dbg.v_tau_intent / kp_equiv, l_max));

    const bool was_initialized = cmd_governor_initialized_;
    // Governor candidate generation is side-effect free.  These local
    // values are committed only after the corresponding PositionCommand has
    // published successfully at the command boundary.
    const double prior_governor_l = was_initialized
        ? cmd_governor_last_l_ : dbg.l_ff;
    const Eigen::Vector3d prior_governor_normal = was_initialized
        ? cmd_governor_normal_state_ : Eigen::Vector3d::Zero();

    const double l_rate_max = std::max(0.0, cmd_governor_l_rate_max_);
    const double rate_lower = std::max(l_min, prior_governor_l - l_rate_max * dt);
    const double rate_upper = std::min(l_max, prior_governor_l + l_rate_max * dt);
    std::vector<double> l_candidates;
    for (double l = l_min; l <= l_max + 0.5 * l_step; l += l_step)
    {
        l_candidates.push_back(std::max(l_min, std::min(l, l_max)));
    }
    l_candidates.push_back(dbg.l_ff);
    l_candidates.push_back(prior_governor_l);
    l_candidates.push_back(rate_lower);
    l_candidates.push_back(rate_upper);
    for (double& l : l_candidates)
    {
        l = std::max(l_min, std::min(l, l_max));
    }
    std::sort(l_candidates.begin(), l_candidates.end());
    l_candidates.erase(std::unique(l_candidates.begin(), l_candidates.end(),
                                   [](double a, double b) {
                                       return std::abs(a - b) < 1e-4;
                                   }),
                       l_candidates.end());

    const Eigen::Vector3d delta_des = raw_v / kp_equiv;
    const double rho = out.e_perp.norm();
    const bool has_e_hat = rho > 1e-6;
    Eigen::Vector3d e_hat = Eigen::Vector3d::Zero();
    if (has_e_hat)
    {
        e_hat = out.e_perp / rho;
    }
    const double normal_deadband = std::max(0.0, cmd_governor_normal_deadband_);
    const double normal_full_error = std::max(normal_deadband + 1e-6,
                                              cmd_governor_normal_full_error_);
    const double normal_max_cfg = std::max(0.0, cmd_governor_normal_max_);
    if (rho <= normal_deadband)
    {
        dbg.active_normal_max = 0.0;
    }
    else if (rho < normal_full_error)
    {
        dbg.active_normal_max = normal_max_cfg *
            (rho - normal_deadband) / (normal_full_error - normal_deadband);
    }
    else
    {
        dbg.active_normal_max = normal_max_cfg;
    }

    GovernorCandidate best;
    const double lead_max = std::max(0.0, cmd_governor_lead_max_);
    const double normal_rate_max = std::max(0.0, cmd_governor_normal_rate_max_);
    const double path_end_eps = 1e-6;
    double phase_per_meter = 1.0;
    if (closedPhaseV2Active()) {
        if (executed_domain_ready) {
            phase_offset_navigation::ExecutedReferenceQueryResult phase_reference;
            if (executed_reference_query->query(progress_w_after,
                                                phase_reference) &&
                phase_reference.valid && phase_reference.r_w.norm() > 1e-6) {
                phase_per_meter = 1.0 / phase_reference.r_w.norm();
            }
        } else {
        ContinuousPhasePathState phase_state;
        if (authoritative_path->evaluate(progress_w_after, phase_state, true)) {
            const double dpdw_norm = phase_state.dp_dw.norm();
            if (dpdw_norm > 1e-6) {
                phase_per_meter = 1.0 / dpdw_norm;
            }
        }
        }
    }

    // The phase must keep describing where the UAV actually is on the path.
    // With the coordination offset r = p + N*delta and a curved path, the
    // stored phase can lag the vehicle's projection by more than a metre; the
    // lookahead then resolves onto the vehicle itself and every candidate
    // collapses to a hold (measured cmd_dist = 0.023 m with a 1.6 m lookahead).
    // This is a correction, not a restriction: if the projection is ahead of
    // the phase and closer to the vehicle, adopt it.
    if (authoritative_path && std::isfinite(progress_w_after)) {
        double best_w = progress_w_after;
        double best_d = std::numeric_limits<double>::infinity();
        const double lo = progress_w_after - 1.0;
        const double hi = progress_w_after + 4.0;
        for (double w = lo; w <= hi + 1e-9; w += 0.05) {
            ContinuousPhasePathState probe;
            if (!authoritative_path->evaluate(w, probe)) continue;
            const double d = (probe.p - pos).norm();
            if (d < best_d) {
                best_d = d;
                best_w = w;
            }
        }
        if (best_w > progress_w_after + 0.05) {
            publishAuthoritativePhase(best_w, true, closed_phase_acquired_);
            progress_w_ = best_w;
            progress_w_after = best_w;
        }
    }

    for (double l : l_candidates)
    {
        if (was_initialized &&
            (l < rate_lower - 1e-6 || l > rate_upper + 1e-6))
        {
            continue;
        }

        ++dbg.candidate_count;
        Eigen::Vector3d reference_l = Eigen::Vector3d::Zero();
        bool clamped_to_end = false;
        double candidate_path_w_start = 0.0;
        double candidate_path_w_end = 0.0;
        const double query_w = progress_w_after + l * phase_per_meter;
        if (executed_domain_ready) {
            candidate_path_w_start = executed_domain_start;
            candidate_path_w_end = executed_domain_end;
            clamped_to_end = query_w > candidate_path_w_end;
            phase_offset_navigation::ExecutedReferenceQueryResult reference;
            if (query_w < candidate_path_w_start ||
                query_w > candidate_path_w_end ||
                !executed_reference_query->query(query_w, reference) ||
                !reference.valid || !reference.r.allFinite() ||
                !reference.r_w.allFinite()) {
                continue;
            }
            reference_l = reference.r;
        } else if (!pathPointAtW(authoritative_path, query_w, reference_delta,
                                 reference_l, clamped_to_end,
                                 candidate_path_w_start, candidate_path_w_end)) {
            continue;
        }
        dbg.path_w_start = candidate_path_w_start;
        dbg.path_w_end = candidate_path_w_end;
        if (clamped_to_end || query_w > candidate_path_w_end - path_end_eps)
        {
            ++dbg.path_end_clamped_count;
            continue;
        }

        Eigen::Vector3d t_l = t_current;
        Eigen::Vector3d target_tangent = Eigen::Vector3d::Zero();
        if (executed_domain_ready) {
            phase_offset_navigation::ExecutedReferenceQueryResult reference;
            if (executed_reference_query->query(query_w, reference) &&
                reference.valid && reference.r_w.allFinite() &&
                reference.r_w.norm() > 1e-6) {
                target_tangent = reference.r_w;
            }
        } else if (pathTangentAtW(authoritative_path, query_w, reference_delta,
                                  target_tangent)) {
            // Legacy neutral/observe-only path.  Production offset commands
            // use the immutable executed query branch above.
        }
        if (target_tangent.norm() > 1e-6) {
            t_l = target_tangent.normalized();
        }

        GovernorCandidate c;
        c.have = true;
        c.l = l;
        c.query_w = query_w;
        c.path_w_start = candidate_path_w_start;
        c.path_w_end = candidate_path_w_end;
        c.base_delta = reference_l - pos;
        c.n_raw = delta_des - c.base_delta;
        c.n = constrainNormal(c.n_raw, t_l, e_hat, has_e_hat, dbg.active_normal_max);

        if (was_initialized && normal_rate_max > 1e-6 && dt > 1e-6)
        {
            Eigen::Vector3d d_n = c.n - prior_governor_normal;
            const double max_dn = normal_rate_max * dt;
            const double dn_norm = d_n.norm();
            if (dn_norm > max_dn && dn_norm > 1e-6)
            {
                c.n = prior_governor_normal + d_n * (max_dn / dn_norm);
                c.normal_rate_limited = true;
            }
        }
        c.n = projectNormal(c.n, t_l);
        c.n = enforceCrossInPlane(c.n, t_l, e_hat, has_e_hat, cmd_governor_normal_cross_max_);
        c.n = projectNormal(c.n, t_l);
        c.n = limitNormalMagnitude(c.n, dbg.active_normal_max);

        c.cmd = reference_l + c.n;
        double candidate_lead = (c.cmd - pos).norm();
        if (lead_max > 1e-6 && candidate_lead > lead_max && candidate_lead > 1e-6)
        {
            // Bounded-step semantics.  A candidate further ahead than the lead
            // leash is pulled back along the same ray instead of being dropped.
            // Dropping all of them froze the vehicle (GOVERNOR_INVALID_HOLD /
            // no_valid_candidate) whenever the reference ran ahead of it, and
            // the resulting hold preserved exactly the condition that caused
            // it.  Clamping keeps the command bounded while still stepping
            // toward the reference, so the gap can close again.
            const Eigen::Vector3d lead_direction =
                (c.cmd - pos) / candidate_lead;
            c.cmd = pos + lead_direction * lead_max;
            c.n = c.cmd - reference_l;
            candidate_lead = lead_max;
            dbg.lead_limit_violation = true;
            ++dbg.lead_clamped_count;
        }

        c.v_model = kp_equiv * (c.cmd - pos);
        const double v_model_tau = c.v_model.dot(t_current);
        const Eigen::Vector3d v_model_n = c.v_model - v_model_tau * t_current;
        c.tau_vel_error = v_model_tau - dbg.v_tau_intent;
        const Eigen::Vector3d n_err = v_model_n - v_n_intent;
        c.normal_vel_error_norm = n_err.norm();
        const double normal_vel_error_cap = std::max(0.0, cmd_governor_normal_vel_error_cap_);
        double normal_vel_error_for_cost = c.normal_vel_error_norm;
        if (normal_vel_error_cap > 1e-6 &&
            normal_vel_error_for_cost > normal_vel_error_cap)
        {
            normal_vel_error_for_cost = normal_vel_error_cap;
            c.normal_vel_error_capped = true;
        }
        c.vel_cost = std::max(0.0, cmd_governor_tau_vel_weight_) *
                         c.tau_vel_error * c.tau_vel_error +
                     std::max(0.0, cmd_governor_normal_vel_weight_) *
                         normal_vel_error_for_cost * normal_vel_error_for_cost;
        c.normal_cost = std::max(0.0, cmd_governor_normal_weight_) *
                        (kp_equiv * c.n).squaredNorm();
        c.l_ff_cost = std::max(0.0, cmd_governor_l_ff_weight_) *
                      std::pow(kp_equiv * (l - dbg.l_ff), 2);
        if (was_initialized)
        {
            c.normal_rate_cost = std::max(0.0, cmd_governor_normal_rate_weight_) *
                                 (kp_equiv * (c.n - prior_governor_normal)).squaredNorm();
            c.l_rate_cost = std::max(0.0, cmd_governor_l_rate_weight_) *
                            std::pow(kp_equiv * (l - prior_governor_l), 2);
        }
        c.cost = c.vel_cost + c.normal_cost + c.l_ff_cost +
                 c.normal_rate_cost + c.l_rate_cost;

        ++dbg.valid_count;
        if (!best.have || c.cost < best.cost)
        {
            best = c;
        }
    }

    // A surviving but zero-step candidate is the same lock as an empty
    // candidate set: the reference has been consumed while the vehicle is not
    // at its goal, so the command pins it in place.  Both feed one unlock
    // timer; after the timeout the phase is re-anchored to the vehicle.
    // 5 cm rather than 2 cm: a "creeping" command of a couple of centimetres per
    // tick (a few cm/s of commanded speed) is a stall in practice, and the
    // phase/position desync that produces it was measured at ~2.3 cm.
    const bool zero_step_candidate =
        best.have && (best.cmd - pos).norm() < 0.05;
    if (cmd_governor_unlock_timeout_s_ > 0.0 &&
        (!best.have || zero_step_candidate))
    {
        const ros::Time now = ros::Time::now();
        if (governor_no_valid_since_.isZero())
        {
            governor_no_valid_since_ = now;
        }
        else if ((now - governor_no_valid_since_).toSec() >=
                 cmd_governor_unlock_timeout_s_)
        {
            governor_no_valid_since_ = ros::Time(0);
            resetGovernorState();
            // Re-project the phase onto the vehicle.  The stall is produced by
            // a phase that no longer describes where the UAV actually is on
            // the path, so merely invalidating the initialization flag is not
            // enough (measured: 78 unlocks without recovery).  Search a window
            // around the stored phase for the closest path point and adopt its
            // w, which puts the lookahead back in front of the UAV.
            double recovered_w = progress_w_;
            if (authoritative_path) {
                double best_d = std::numeric_limits<double>::infinity();
                const double lo = progress_w_ - 3.0;
                const double hi = progress_w_ + 4.0;
                for (double w = lo; w <= hi + 1e-9; w += 0.05) {
                    ContinuousPhasePathState state_probe;
                    if (!authoritative_path->evaluate(w, state_probe)) continue;
                    const double d = (state_probe.p - pos).norm();
                    if (d < best_d) {
                        best_d = d;
                        recovered_w = w;
                    }
                }
            }
            publishAuthoritativePhase(
                recovered_w, true, closed_phase_acquired_);
            progress_w_ = recovered_w;
            progress_initialized_ = true;
            dbg.state_reset_due_to_lead_limit = true;
            ROS_WARN(
                "[GVF][GOV_UNLOCK] stalled command for %.2fs; phase %.2f -> %.2f "
                "(pos=(%.2f,%.2f), dom=[%.2f,%.2f])",
                cmd_governor_unlock_timeout_s_, phase_w_, recovered_w, pos.x(),
                pos.y(), executed_domain_start, executed_domain_end);
        }
    }
    else
    {
        governor_no_valid_since_ = ros::Time(0);
    }

    if (!best.have)
    {
        if (dbg.candidate_count > 0 &&
            dbg.path_end_clamped_count == dbg.candidate_count)
        {
            return makeGovernorInvalidHold(pos, "all_candidates_path_end_clamped", dbg);
        }
        return makeGovernorInvalidHold(pos, "no_valid_candidate", dbg);
    }

    result.cmd_pos = best.cmd;
    result.yaw_cmd_vec = best.cmd - pos;
    result.final_cmd_source = "VEL_MATCH_GOVERNOR";
    result.fallback_reason = "none";
    result.command_valid = true;
    result.selected_valid_for_state = true;
    result.selected_l = best.l;
    result.selected_n = best.n;

    dbg.fallback_hold_pos = false;
    dbg.best_l = best.l;
    dbg.best_query_w = best.query_w;
    dbg.best_cost = best.cost;
    dbg.base_delta_norm = best.base_delta.norm();
    dbg.normal_raw_norm = best.n_raw.norm();
    dbg.normal_state_norm = best.n.norm();
    dbg.normal_rate_limited = best.normal_rate_limited;
    dbg.selected_v_model_norm = best.v_model.norm();
    dbg.tau_vel_error = best.tau_vel_error;
    dbg.normal_vel_error_norm = best.normal_vel_error_norm;
    dbg.normal_vel_error_capped = best.normal_vel_error_capped;
    dbg.vel_error_norm = std::sqrt(best.tau_vel_error * best.tau_vel_error +
                                   best.normal_vel_error_norm * best.normal_vel_error_norm);
    dbg.vel_cost = best.vel_cost;
    dbg.l_ff_cost = best.l_ff_cost;
    dbg.l_rate_cost = best.l_rate_cost;
    dbg.normal_cost = best.normal_cost;
    dbg.normal_rate_cost = best.normal_rate_cost;
    dbg.cmd_dist = (result.cmd_pos - pos).norm();
    dbg.path_w_start = best.path_w_start;
    dbg.path_w_end = best.path_w_end;
    dbg.best_clamped_to_end = false;
    return result;
}

void gvf_manager::updateGovernorCommandHistory(const Eigen::Vector3d& pos,
                                               const Eigen::Vector3d& cmd_pos,
                                               double dt,
                                               GovernorCommandDebug& dbg)
{
    dbg.cmd_dist = (cmd_pos - pos).norm();
    if (has_last_governor_cmd_ && dt > 1e-6)
    {
        const Eigen::Vector3d cmd_vel_est = (cmd_pos - last_governor_cmd_pos_) / dt;
        dbg.cmd_delta_rate = cmd_vel_est.norm();
        dbg.estimated_acc = (cmd_vel_est - last_governor_cmd_vel_).norm() / dt;
        last_governor_cmd_vel_ = cmd_vel_est;
    }
    else
    {
        last_governor_cmd_vel_.setZero();
    }
    last_governor_cmd_pos_ = cmd_pos;
    has_last_governor_cmd_ = true;
    last_cmd_pos_ = cmd_pos;
}

void gvf_manager::logGovernorCommand(const GovernorCommandResult& result,
                                     const GovernorCommandDebug& dbg,
                                     const Eigen::Vector3d& cmd_pos,
                                     double real_dis_to_goal,
                                     double kp_equiv,
                                     bool switch_active) const
{
    ROS_WARN_THROTTLE(1.0,
      "[GVF_CMD] source=%s reason=%s raw=%.2f odom_v=%.2f cmd_dist=%.2f K=%.2f cmd=(%.2f %.2f %.2f) d_goal=%.2f",
      result.final_cmd_source.c_str(), result.fallback_reason.c_str(), dbg.raw_v_norm,
      odom_vel_lpf_.head<2>().norm(), dbg.cmd_dist, kp_equiv,
      cmd_pos.x(), cmd_pos.y(), cmd_pos.z(), real_dis_to_goal);

    ROS_WARN_THROTTLE(
        0.2,
        "[GVF][CMD_VEL_MATCH_GOV] final_cmd_source=%s fallback_reason=%s raw_v_norm=%.3f raw_v_tau=%.3f raw_v_normal_norm=%.3f v_tau_intent=%.3f v_n_intent_norm=%.3f cmd_vel_max=%.3f tangent_vel_max=%.3f L_ff=%.3f best_L=%.3f best_query_w=%.3f path_w_start=%.3f path_w_end=%.3f best_clamped_to_end=%d best_cost=%.3f candidate_count=%d valid_count=%d path_end_clamped_count=%d skipped_lead_count=%d lead_clamped_count=%d base_delta_norm=%.3f normal_raw_norm=%.3f normal_state_norm=%.3f normal_max=%.3f normal_cross_max=%.3f normal_rate_limited=%d lead_limit_violation=%d selected_v_model_norm=%.3f vel_error_norm=%.3f tau_vel_error=%.3f normal_vel_error_norm=%.3f normal_vel_error_capped=%d vel_cost=%.3f l_ff_cost=%.3f l_rate_cost=%.3f normal_cost=%.3f normal_rate_cost=%.3f cmd_dist=%.3f cmd_delta_rate=%.3f estimated_acc=%.3f acc_max=%.3f switch_active=%d K_eq=%.3f e_perp_norm=%.3f fallback_hold_pos=%d initialized=%d final_cmd_overridden=%d state_reset_due_to_override=%d state_reset_due_to_lead_limit=%d actual_stored_normal_norm=%.3f",
        result.final_cmd_source.c_str(),
        result.fallback_reason.c_str(),
        dbg.raw_v_norm,
        dbg.raw_v_tau,
        dbg.raw_v_normal_norm,
        dbg.v_tau_intent,
        dbg.v_n_intent_norm,
        cmd_vel_max_,
        cmd_tangent_vel_max_,
        dbg.l_ff,
        dbg.best_l,
        dbg.best_query_w,
        dbg.path_w_start,
        dbg.path_w_end,
        dbg.best_clamped_to_end ? 1 : 0,
        dbg.best_cost,
        dbg.candidate_count,
        dbg.valid_count,
        dbg.path_end_clamped_count,
        dbg.skipped_lead_count,
        dbg.lead_clamped_count,
        dbg.base_delta_norm,
        dbg.normal_raw_norm,
        dbg.normal_state_norm,
        dbg.active_normal_max,
        std::max(0.0, cmd_governor_normal_cross_max_),
        dbg.normal_rate_limited ? 1 : 0,
        dbg.lead_limit_violation ? 1 : 0,
        dbg.selected_v_model_norm,
        dbg.vel_error_norm,
        dbg.tau_vel_error,
        dbg.normal_vel_error_norm,
        dbg.normal_vel_error_capped ? 1 : 0,
        dbg.vel_cost,
        dbg.l_ff_cost,
        dbg.l_rate_cost,
        dbg.normal_cost,
        dbg.normal_rate_cost,
        dbg.cmd_dist,
        dbg.cmd_delta_rate,
        dbg.estimated_acc,
        std::max(0.0, cmd_acc_max_),
        switch_active ? 1 : 0,
        kp_equiv,
        dbg.e_perp_norm,
        dbg.fallback_hold_pos ? 1 : 0,
        cmd_governor_initialized_ ? 1 : 0,
        dbg.final_cmd_overridden ? 1 : 0,
        dbg.state_reset_due_to_override ? 1 : 0,
        dbg.state_reset_due_to_lead_limit ? 1 : 0,
        cmd_governor_normal_state_.norm());
}

bool gvf_manager::publishGovernorPositionCommand(const Eigen::Vector3d& cmd_pos,
                                                 const Eigen::Vector3d& yaw_cmd_vec)
{
    if (!cmd_pub) return false;
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.position.x = cmd_pos.x();
    cmd.position.y = cmd_pos.y();
    cmd.position.z = cmd_pos.z();
    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = 0.0;

    double arg_ = last_yaw;
    if (yaw_cmd_vec.norm() > 0.1)
    {
        arg_ = atan2(-yaw_cmd_vec.x(), yaw_cmd_vec.y()) + (PI/2.0f);
    }
    std::pair<double, double> yaw_all = calculate_yaw(last_yaw, arg_);
    cmd.yaw = yaw_all.first;
    cmd.yaw_dot = 0.0f;

    try {
        cmd_pub.publish(cmd);
    } catch (const std::exception&) {
        return false;
    }
    // Keep manager state transactional with the software publication.  The
    // callback installs this candidate only after the adapter/authority
    // commit has succeeded; a rejected/throwing publisher leaves last_yaw
    // untouched.
    pending_last_yaw_ = yaw_all.first;
    pending_last_yaw_valid_ = true;
    return true;
}

bool gvf_manager::assignPathIdentity(
    const std::shared_ptr<const ContinuousPhasePath>& candidate,
    std::shared_ptr<const ContinuousPhasePath>& assigned)
{
    assigned.reset();
    if (!candidate || candidate->empty()) return false;
    std::uint64_t identity = 0U;
    {
        std::lock_guard<std::mutex> lock(path_reference_handoff_mutex_);
        const std::uint64_t floor = std::max(
            next_path_identity_, candidate->pathRevision());
        if (floor == std::numeric_limits<std::uint64_t>::max()) return false;
        identity = floor + 1U;
        next_path_identity_ = identity;
    }
    std::shared_ptr<ContinuousPhasePath> revised(
        new ContinuousPhasePath(*candidate));
    revised->setPathRevision(identity);
    assigned = std::shared_ptr<const ContinuousPhasePath>(std::move(revised));
    return assigned && assigned->pathRevision() == identity;
}

bool gvf_manager::buildAndStageSectionBundle(
    gvfManager& pm,
    const AuthoritativePhaseSnapshot& phase,
    const std::shared_ptr<const ContinuousPhasePath>& path,
    const std::shared_ptr<const ContinuousPhasePath>& source_path,
    const double copied_prefix_start_w,
    const double copied_prefix_end_w) {
    if (!phase_offset_matched_adapter_ || !pm.sdf_map_ || !pm.gvf_ ||
        matched_config_.mode != PhaseOffsetMatchedMode::MANUAL ||
        !phase.initialized || !std::isfinite(phase.w) || !path ||
        path->empty() || path->pathRevision() == 0U ||
        !std::isfinite(path->startW()) || !std::isfinite(path->endW()) ||
        !(path->endW() > path->startW())) {
        ROS_WARN_THROTTLE(
            1.0,
            "[phase_offset_section] build guard rejected adapter=%d map=%d gvf=%d mode_manual=%d phase_init=%d path=%d rev=%llu",
            phase_offset_matched_adapter_ ? 1 : 0, pm.sdf_map_ ? 1 : 0,
            pm.gvf_ ? 1 : 0,
            matched_config_.mode == PhaseOffsetMatchedMode::MANUAL ? 1 : 0,
            phase.initialized ? 1 : 0, path ? 1 : 0,
            static_cast<unsigned long long>(
                path ? path->pathRevision() : 0U));
        return false;
    }
    const std::uint64_t generation =
        phase_offset_matched_adapter_->executionGeneration();
    if (generation == 0U ||
        (source_path && (source_path->empty() ||
                         source_path->pathRevision() == 0U))) {
        return false;
    }

    // Build only the live section needed by NORMAL Preview.  Keep a small
    // backward margin for the governor while never reading outside the
    // immutable path domain.
    const double back_w = std::max(0.0, matched_config_.tube.back_w);
    const double refresh_slack_w = std::max(
        0.0, matched_config_.tube.min_certified_forward_w);
    const double horizon_w = std::max(
        matched_config_.normal_preview_policy.preview_horizon_w,
        matched_config_.tube.lookahead_w) + refresh_slack_w;
    if (!std::isfinite(back_w) || !std::isfinite(refresh_slack_w) ||
        !std::isfinite(horizon_w) || horizon_w <= 0.0) {
        return false;
    }
    const double w_start = std::max(path->startW(), phase.w - back_w);
    const double w_end = std::min(path->endW(), phase.w + horizon_w);
    if (!std::isfinite(w_start) || !std::isfinite(w_end) ||
        w_start < path->startW() || w_end > path->endW() ||
        !(w_end > w_start + 1e-6) || phase.w < w_start || phase.w > w_end) {
        return false;
    }

    const double half_width = matched_config_.section_build.half_width;
    const double clearance = matched_config_.section_build.clearance;
    if (!std::isfinite(half_width) || half_width < 0.0 ||
        !std::isfinite(clearance) || clearance < 0.0 ||
        matched_config_.section_build.max_obstacle_checks == 0U ||
        matched_config_.section_build.max_obstacles == 0U) {
        return false;
    }
    plan_env::LocalObstacleRequest request;
    std::string region_reason;
    // The requested reference region is expanded by the configured clearance
    // as well, because the map view insets its claimable reference domain by
    // that same clearance.  Without this the scan could never reach the
    // configured half width even in open space.
    //
    // The same inset applies along the path: the view only lets a consumer
    // rely on the region that is at least one clearance inside the requested
    // box in every direction.  Requesting a region that ends exactly at the
    // scanned interval therefore clipped the far end of every prefix.  Ask for
    // one clearance more on both sides, clamped to the immutable path domain.
    const double roi_w_start = std::max(path->startW(), w_start - clearance);
    const double roi_w_end = std::min(path->endW(), w_end + clearance);
    if (!(roi_w_end > roi_w_start)) {
        return false;
    }
    if (!makeSectionReferenceRegion(
            path, roi_w_start, roi_w_end, half_width + clearance,
            matched_config_.section_build.max_step_w,
            matched_config_.section_build.max_cells,
            request.reference_region, region_reason)) {
        ROS_WARN_THROTTLE(
            1.0,
            "[phase_offset_section] reference ROI unavailable: %s",
            region_reason.c_str());
        return false;
    }
    request.clearance = clearance;
    request.max_voxel_checks = matched_config_.section_build.max_obstacle_checks;
    request.max_occupied_voxels = matched_config_.section_build.max_obstacles;
    const plan_env::LocalObstacleCapture capture =
        pm.sdf_map_->readLocalObstacleView(request);
    if (capture.view.status != plan_env::LocalObstacleViewStatus::VALID ||
        !capture.environment_validity || !capture.environment_change_mutex) {
        ROS_WARN_THROTTLE(
            1.0,
            "[phase_offset_section] local obstacle view unavailable status=%d validity=%d mutex=%d",
            static_cast<int>(capture.view.status),
            capture.environment_validity ? 1 : 0,
            capture.environment_change_mutex ? 1 : 0);
        return false;
    }

    // SUPER-style horizontal scan.  The local view exposes one box per
    // occupied native voxel; feeding all of them into a per-cell separation
    // plane builder made the cost grow with (cells x voxels) and subdivided
    // whenever the analytic cell proof failed.  Walk the horizontal normal
    // instead: for each path sample step outward in both directions and stop
    // at the first occupied or unknown voxel.  This is the same
    // path-relative cross-section the paper needs, at a cost that depends
    // only on the number of samples and the half width.
    std::string reason;
    SectionScanEnvironment scan_environment;
    if (!scan_environment.build(capture.view, reason)) {
        ROS_WARN_THROTTLE(1.0,
            "[phase_offset_section] local view scan environment unavailable: %s",
            reason.c_str());
        return false;
    }
    SectionScanConfig scan_config;
    scan_config.sample_step_w = matched_config_.section_build.max_step_w;
    scan_config.half_width = half_width;
    scan_config.clearance = clearance;
    scan_config.minimum_reference_speed =
        matched_config_.section_build.minimum_reference_speed;
    scan_config.min_regularity_ratio =
        matched_config_.section_build.min_regularity_ratio;
    scan_config.curvature_epsilon =
        matched_config_.section_build.curvature_epsilon;
    scan_config.max_scan_checks =
        matched_config_.section_build.max_obstacle_checks;
    scan_config.max_knots = matched_config_.section_build.max_cells;
    const phase_offset_navigation::SectionTubeProfile profile =
        buildSectionTubeByScan(path, w_start, w_end, scan_environment,
                               scan_config);
    if (!profile.usable || profile.valid_end < phase.w - 1e-9) {
        ROS_WARN_THROTTLE(1.0,
            "[phase_offset_section] profile unavailable status=%d usable=%d "
            "valid=[%.3f,%.3f] phase=%.3f first_failure_w=%.3f reason=%s "
            "knots=%zu budget(cells=%zu checks=%zu)",
            static_cast<int>(profile.status), profile.usable ? 1 : 0,
            profile.valid_start, profile.valid_end, phase.w,
            profile.first_failure_w, profile.failure_reason.c_str(),
            profile.knots.size(), profile.visited_cells, profile.obstacle_checks);
        return false;
    }
    if (!profile.complete) {
        // The profile is usable and covers the live phase, but the scan
        // stopped before the requested forward end.  Report the cause once per
        // second; the display renders such a prefix translucent so a partial
        // corridor is never mistaken for a full one.
        ROS_WARN_THROTTLE(1.0,
            "[phase_offset_section] partial prefix status=%d valid_end=%.3f "
            "reason=%s knots=%zu request=[%.3f,%.3f] first_failure_w=%.3f",
            static_cast<int>(profile.status), profile.valid_end,
            profile.failure_reason.c_str(), profile.knots.size(), w_start,
            w_end, profile.first_failure_w);
    }

    // One throttled geometry summary per build: the numbers an operator needs
    // to tell whether the configured half-width cap or the clearance is the
    // binding limit at this place along the path.
    {
        double left_min = std::numeric_limits<double>::infinity();
        double left_max = 0.0, left_sum = 0.0;
        double right_min = std::numeric_limits<double>::infinity();
        double right_max = 0.0, right_sum = 0.0;
        for (const auto& knot : profile.knots) {
            const double left = -knot.lower;
            const double right = knot.upper;
            if (!std::isfinite(left) || !std::isfinite(right)) continue;
            left_min = std::min(left_min, left);
            left_max = std::max(left_max, left);
            left_sum += left;
            right_min = std::min(right_min, right);
            right_max = std::max(right_max, right);
            right_sum += right;
        }
        if (!profile.knots.empty() &&
            std::isfinite(left_min) && std::isfinite(right_min)) {
            const double count = static_cast<double>(profile.knots.size());
            ROS_INFO_THROTTLE(2.0,
                "[phase_offset_section] half width left[min=%.3f mean=%.3f "
                "max=%.3f] right[min=%.3f mean=%.3f max=%.3f] knots=%zu "
                "cap=%.2f clearance=%.2f",
                left_min, left_sum / count, left_max,
                right_min, right_sum / count, right_max,
                profile.knots.size(),
                matched_config_.section_build.half_width,
                matched_config_.section_build.clearance);
        }
    }

    const std::shared_ptr<const ContinuousPhaseNormalFrame> frame(
        new ContinuousPhaseNormalFrame(path, path->pathRevision(),
                                       path->pathRevision()));
    std::shared_ptr<SectionPathBundle> mutable_bundle(new SectionPathBundle());
    mutable_bundle->path = path;
    mutable_bundle->frame = frame;
    mutable_bundle->profile =
        std::shared_ptr<const phase_offset_navigation::SectionTubeProfile>(
            new phase_offset_navigation::SectionTubeProfile(profile));
    mutable_bundle->local_view_status = capture.view.status;
    mutable_bundle->reference_domain = capture.view.reference_domain;
    mutable_bundle->obstacle_region = capture.view.obstacle_region;
    mutable_bundle->build_config = matched_config_.section_build;
    mutable_bundle->frame_id = capture.view.frame_id;
    mutable_bundle->task_generation = generation;
    mutable_bundle->path_revision = path->pathRevision();
    mutable_bundle->frame_revision = frame->frameRevision();
    mutable_bundle->environment_validity = capture.environment_validity;
    mutable_bundle->environment_change_mutex = capture.environment_change_mutex;
    const SectionPathBundlePtr bundle =
        std::shared_ptr<const SectionPathBundle>(std::move(mutable_bundle));
    if (!phase_offset_matched_adapter_->stageSectionBundle(
            bundle, source_path, copied_prefix_start_w,
            copied_prefix_end_w)) {
        ROS_WARN_THROTTLE(
            1.0,
            "[phase_offset_section] stage rejected generation=%llu path_rev=%llu",
            static_cast<unsigned long long>(generation),
            static_cast<unsigned long long>(path->pathRevision()));
        return false;
    }
    return true;
}

void gvf_manager::phaseOffsetTubeTimerCallback(const ros::TimerEvent& event)
{
    (void)event;
    // Section production is driven synchronously by FSMCallback on the
    // planning thread.  The historical callback remains as a compatibility
    // symbol for external timer handles, but intentionally performs no V2
    // map capture, worker scheduling, or geometry construction.
}

void gvf_manager::cmdCallback(const ros::TimerEvent& event)
{
    const ros::Time now = ros::Time::now();
    bool sph_beta_published = false;
    const auto publish_sph_beta =
        [this, &sph_beta_published, &now](
            const MatchedAdapterOutput* output, const bool authoritative) {
            if (sph_beta_published || !phase_offset_sph_ros_bridge_) return;
            bool valid = false;
            double beta_i = 0.0;
            if (authoritative && output != nullptr &&
                output->allocator_evaluated &&
                output->runtime_execution.mode ==
                    phase_offset_navigation::RuntimeExecutionMode::NORMAL) {
                const auto& preview = output->normal_preview;
                valid = preview.valid && preview.feasible &&
                    preview.contraction_rate_feasible &&
                    preview.current_delta_inside && preview.rate_feasible &&
                    preview.current_rate_interval.valid &&
                    preview.status ==
                        phase_offset_navigation::TubeViabilityStatus::FEASIBLE &&
                    std::isfinite(preview.beta_i) && preview.beta_i >= 0.0 &&
                    preview.beta_i <= 1.0;
                if (valid) beta_i = preview.beta_i;
            }
            phase_offset_sph_ros_bridge_->publishBeta(now, beta_i, valid);
            sph_beta_published = true;
        };
    const bool measure_callback = phase_offset_measurement_callback_enabled_;
    const std::uint64_t callback_ros_stamp_ns = measure_callback
        ? ros::Time::now().toNSec() : 0U;
    const auto callback_start = measure_callback
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point();
    const auto finish_callback_timing = [this, measure_callback,
                                         callback_ros_stamp_ns, callback_start]() {
        if (!measure_callback) return;
        const auto callback_end = std::chrono::steady_clock::now();
        const std::uint64_t duration_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                callback_end - callback_start).count());
        recordCallbackTiming(duration_ns, callback_ros_stamp_ns);
    };
    const auto deactivate_phase_offset = [this]() {
        if (phase_offset_matched_adapter_ &&
            matched_config_.mode == PhaseOffsetMatchedMode::MANUAL)
        {
            phase_offset_matched_adapter_->deactivate(ros::Time::now());
        }
    };

    if (use_test_cmd_)
    {
        deactivate_phase_offset();
        publish_sph_beta(nullptr, false);
        finish_callback_timing();
        return;
    }

    if (cmd_gain_test_enable_)
    {
        deactivate_phase_offset();
        const Eigen::Vector3d pos = odom_;
        Eigen::Vector3d lead = Eigen::Vector3d::Zero();
        if (cmd_gain_test_axis_ == 1)
        {
            lead.y() = cmd_gain_test_lead_;
        }
        else
        {
            lead.x() = cmd_gain_test_lead_;
        }

        const Eigen::Vector3d cmd_pos = pos + lead;

        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        cmd.position.x = cmd_pos.x();
        cmd.position.y = cmd_pos.y();
        cmd.position.z = cmd_pos.z();
        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;
        cmd.yaw = last_yaw;
        cmd.yaw_dot = 0.0f;
        cmd_pub.publish(cmd);

        const Eigen::Vector2d lead_xy = lead.head<2>();
        const Eigen::Vector2d vel_xy = odom_vel_lpf_.head<2>();
        const double lead_sq = lead_xy.squaredNorm();
        const double k_meas = lead_sq > 1e-9 ? vel_xy.dot(lead_xy) / lead_sq : 0.0;

        ROS_WARN_THROTTLE(0.5,
                          "[GVF][GAIN_TEST] lead=%.3f vel_est_x=%.3f vel_est_y=%.3f k_meas=%.3f",
                          lead_xy.norm(), odom_vel_lpf_.x(), odom_vel_lpf_.y(), k_meas);
        publish_sph_beta(nullptr, false);
        finish_callback_timing();
        return;
    }

    if (swarmParticlesManager.empty())
    {
        deactivate_phase_offset();
        publish_sph_beta(nullptr, false);
        finish_callback_timing();
        return;
    }
    if (!swarmParticlesManager[0].receive_goal)
    {
        deactivate_phase_offset();
        ROS_WARN_THROTTLE(1.0, "[GVF] DO NOT RECEIVE GOAL");
        publish_sph_beta(nullptr, false);
        finish_callback_timing();
        return;
    }

    auto& pm = swarmParticlesManager[0];
    const Eigen::Vector3d pos = odom_;
    const Eigen::Vector3d goal = pm.goal_pt;
    const double dt = 0.02;
    pending_last_yaw_valid_ = false;
    const double kp_equiv = std::max(0.1, cmd_pos_gain_equiv_);
    const double real_dis_to_goal = (goal - pos).head<2>().norm();
    // One immutable H2 phase tuple for this whole command.  Guidance,
    // adapter input, governor and the pending-pair boundary below must all
    // use this same snapshot rather than independently rereading phase_w_.
    const AuthoritativePhaseSnapshot command_phase =
        captureAuthoritativePhase();
    const double phase_before = command_phase.w;
    const guidance::IsfGains command_gains(
        pm.gvf_ ? pm.gvf_->gvf_.K1_ : 0.0,
        pm.gvf_ ? pm.gvf_->gvf_.K2_ : 0.0,
        pm.gvf_ ? pm.gvf_->gvf_.convergence_bandwidth_ : 0.0,
        pm.gvf_ ? pm.gvf_->progress_rho0_ : 0.0,
        pm.gvf_ ? pm.gvf_->progress_delta_ : 0.0,
        pm.gvf_ ? pm.gvf_->alpha_min_ : 0.0);
    bool have_phase_candidate = false;
    double phase_candidate = phase_before;
    bool legacy_progress_candidate_valid = false;
    bool will_acquire_closed_phase = false;
    bool initial_closed_phase_acquisition_used = false;

    GovernorCommandDebug dbg;
    dbg.normal_state_norm = cmd_governor_normal_state_.norm();
    dbg.best_query_w = unifiedPhaseV2Active() && command_phase.initialized
        ? phase_before : activeTrackingPhase();

    GovernorCommandResult result;
    result.cmd_pos = pos;
    result.yaw_cmd_vec.setZero();

    gvf::LiftedGuidanceResult out;
    MatchedAdapterOutput matched_output;
    MatchedAdapterInput matched_input_snapshot;
    bool matched_input_snapshot_valid = false;
    double governor_reference_delta = 0.0;
    std::uint64_t pending_position_command_identity = 0U;
    // A Section pending command carries the map-owned environment boundary
    // captured by the planning/update seam.  Keep this owner outside the
    // manager publication locks so the environment mutex is always acquired
    // first, before frontend/handoff/phase and adapter publication locks.
    SectionPathBundlePtr pending_section_environment_bundle;
    SectionPathBundlePtr current_section_bundle_for_publication;
    SectionPathBundlePtr staged_section_bundle_for_publication;
    std::shared_ptr<const SectionPathHandoff>
        command_section_path_handoff;
    std::shared_ptr<const ContinuousPhasePath> command_planner_path;
    bool command_published_and_committed = false;
    SectionPathBundlePtr rejected_section_candidate_for_retirement;
    const auto retire_rejected_section_candidate =
        [this](const SectionPathBundlePtr& rejected_bundle) {
            if (!phase_offset_matched_adapter_ || !rejected_bundle) return;
            std::shared_ptr<const SectionPathHandoff> retired_handoff;
            {
                // Keep the candidate/current check and both owner retirements
                // in one publication-order critical section.  A lock-free
                // capture followed by a separate discard could race a
                // successful commit and accidentally clear its deferred FSM
                // mirror handoff.
                std::lock_guard<std::mutex> frontend_lock(
                    frontend_apply_mutex_);
                std::lock_guard<std::mutex> handoff_lock(
                    path_reference_handoff_mutex_);
                std::lock_guard<std::mutex> task_lock(
                    phase_offset_matched_adapter_->task_publication_mutex_);
                std::lock_guard<std::mutex> runtime_lock(
                    phase_offset_matched_adapter_->runtime_command_mutex_);
                // This helper retires only an uncommitted candidate.  If the
                // bundle has become the committed current owner, leave its
                // current command/pending step and deferred mirror untouched;
                // a later adapter update will naturally replace a failed
                // pending step.  The identity check must precede every
                // retirement mutation to avoid clearing a newly committed
                // command from a stale rejection witness.
                if (phase_offset_matched_adapter_->section_bundle_ ==
                    rejected_bundle) {
                    return;
                }
                const bool staged_matches =
                    phase_offset_matched_adapter_->staged_section_bundle_ ==
                    rejected_bundle;
                const bool handoff_matches = pending_section_path_handoff_ &&
                    pending_section_path_handoff_->bundle == rejected_bundle;
                if (!staged_matches && !handoff_matches) {
                    return;
                }
                if (staged_matches) {
                    phase_offset_matched_adapter_->staged_section_bundle_.reset();
                    phase_offset_matched_adapter_->staged_section_source_path_.reset();
                    phase_offset_matched_adapter_->staged_section_copied_prefix_start_w_ =
                        std::numeric_limits<double>::quiet_NaN();
                    phase_offset_matched_adapter_->staged_section_copied_prefix_end_w_ =
                        std::numeric_limits<double>::quiet_NaN();
                }
                if (handoff_matches) {
                    retired_handoff.swap(pending_section_path_handoff_);
                }
            }
        };
    if (!pm.gvf_)
    {
        deactivate_phase_offset();
        result = makeGovernorInvalidHold(pos, "missing_gvf", dbg);
    }
    else
    {
        const double shadow_semantic_w =
            unifiedPhaseV2Active() && command_phase.initialized
                ? phase_before : activeTrackingPhase();
        const std::shared_ptr<const ContinuousPhasePath> planner_path =
            pm.gvf_->getContinuousPhasePath();
        command_planner_path = planner_path;
        SectionPathBundlePtr section_candidate;
        SectionPathBundlePtr section_current;
        SectionPathBundlePtr section_for_command;
        SectionBundleCandidateCapture section_candidate_capture;
        std::shared_ptr<const SectionPathHandoff> section_path_handoff;
        if (matched_config_.mode == PhaseOffsetMatchedMode::MANUAL &&
            phase_offset_matched_adapter_) {
            // Capture the complete candidate tuple, not only its bundle
            // pointer.  A future-prefix candidate is executable before its
            // planner mirror is installed, but only when this manager has
            // already registered the matching publish-first handoff.
            section_candidate_capture =
                phase_offset_matched_adapter_->capturePendingSectionCandidate();
            section_candidate = section_candidate_capture.bundle;
            section_current =
                phase_offset_matched_adapter_->captureSectionBundle();
            {
                std::lock_guard<std::mutex> handoff_lock(
                    path_reference_handoff_mutex_);
                section_path_handoff = pending_section_path_handoff_;
            }
            command_section_path_handoff = section_path_handoff;

            const auto candidate_prefix_usable = [&]() {
                if (!section_candidate || !section_candidate->path ||
                    section_candidate->path->empty() ||
                    section_candidate->task_generation !=
                        phase_offset_matched_adapter_->executionGeneration() ||
                    !std::isfinite(phase_before) ||
                    phase_before < section_candidate->path->startW() ||
                    phase_before > section_candidate->path->endW()) {
                    return false;
                }
                const bool start_finite = std::isfinite(
                    section_candidate_capture.copied_prefix_start_w);
                const bool end_finite = std::isfinite(
                    section_candidate_capture.copied_prefix_end_w);
                if (start_finite != end_finite) return false;
                if (!start_finite) {
                    // A same-path refresh has no copied-prefix handoff.  A
                    // path replacement without that evidence cannot be
                    // selected merely because its bundle is well formed.
                    return section_candidate->path == planner_path;
                }
                if (!section_candidate_capture.source_path ||
                    section_candidate_capture.source_path->empty() ||
                    !(section_candidate_capture.copied_prefix_end_w >
                        section_candidate_capture.copied_prefix_start_w) ||
                    phase_before <
                        section_candidate_capture.copied_prefix_start_w ||
                    phase_before >
                        section_candidate_capture.copied_prefix_end_w +
                            ((section_current && section_current->path &&
                              section_candidate_capture.copied_prefix_end_w >=
                                  section_current->path->endW() -
                                      kPathReferenceHandoffTolerance)
                                 ? kExhaustedFrontendSeamSlackW
                                 : 0.0)) {
                    return false;
                }
                if (!section_current || !section_current->path ||
                    section_candidate_capture.source_path !=
                        section_current->path ||
                    section_candidate_capture.source_path != planner_path) {
                    return false;
                }
                if (section_candidate->path == planner_path) return true;
                // A path replacement must be paired with the manager DTO
                // before cmdCallback may consume it.  This closes the window
                // between adapter staging and handoff registration.
                return section_path_handoff &&
                    section_path_handoff->task_generation ==
                        section_candidate->task_generation &&
                    section_path_handoff->phase_generation != 0U &&
                    section_path_handoff->bundle == section_candidate &&
                    section_path_handoff->candidate_path ==
                        section_candidate->path &&
                    section_path_handoff->source_path ==
                        section_candidate_capture.source_path &&
                    section_path_handoff->copied_prefix_start_w ==
                        section_candidate_capture.copied_prefix_start_w &&
                    section_path_handoff->copied_prefix_end_w ==
                        section_candidate_capture.copied_prefix_end_w;
            };

            if (candidate_prefix_usable()) {
                section_for_command = section_candidate;
            } else if (section_current && section_current->path) {
                // Candidate validation is deliberately fail-closed, but an
                // already committed current bundle remains eligible for this
                // tick.  A rejected future candidate must not consume it or
                // clear its nonzero Runtime delta.
                section_for_command = section_current;
            }
        }
        // Neutral navigation evaluates the current planner path.  MANUAL
        // Section mode may select an immutable staged/current bundle for the
        // same command; no alternate worker/profile owner is consulted.
        std::shared_ptr<const ContinuousPhasePath> command_path =
            section_for_command ? section_for_command->path : planner_path;
        if (unifiedPhaseV2Active() && command_phase.initialized)
        {
            out = pm.gvf_->calcLiftedGuidanceAtPhase(
                pos, phase_before, command_path);
            if (phase_offset_matched_adapter_)
            {
                MatchedAdapterInput matched_input;
                matched_input.position = pos;
                matched_input.gains = command_gains;
                matched_input.legacy = LegacyGuidanceSnapshot(
                    out.v_cmd, out.w_dot, out.e_parallel, out.e_perp,
                    out.ref_pt, out.tangent, out.valid);
                matched_input.dt = dt;
                matched_input.stamp = now;
                ContinuousPhasePathState command_path_state;
                bool command_path_state_valid = false;
                if (matched_config_.mode == PhaseOffsetMatchedMode::MANUAL &&
                    section_for_command && section_for_command->path ==
                        command_path && section_for_command->frame) {
                    // Section geometry must be evaluated through the exact
                    // immutable frame bundled with the selected profile. A
                    // bare ContinuousPhasePath query has no bound N/N_w and
                    // therefore (correctly) fails the production frame gate.
                    command_path_state_valid =
                        section_for_command->frame->evaluatePathState(
                            phase_before, command_path_state);
                } else if (command_path && !command_path->empty()) {
                    command_path_state_valid = command_path->evaluate(
                        phase_before, command_path_state, false);
                }
                const auto retained_nonzero_section = [this]() {
                    if (!phase_offset_matched_adapter_ ||
                        matched_config_.mode !=
                            PhaseOffsetMatchedMode::MANUAL) {
                        return false;
                    }
                    std::lock_guard<std::mutex> runtime_lock(
                        phase_offset_matched_adapter_->runtime_command_mutex_);
                    return phase_offset_matched_adapter_->runtime_ &&
                        phase_offset_matched_adapter_->runtime_->retainedDelta() !=
                            0.0;
                };
                if (command_path_state_valid)
                {
                    matched_input.path = ConvertContinuousPhasePathStateForActive(
                        command_path_state, phase_before);
                    matched_input.semantic_path_owner = command_path;
                    matched_input.semantic_path_start_w = command_path->startW();
                    matched_input.semantic_path_end_w = command_path->endW();
                    if (coordination_backend_ ==
                        PhaseOffsetCoordinationBackend::SPH &&
                        phase_offset_sph_ros_bridge_) {
                        matched_input.sph_bridge =
                            phase_offset_sph_ros_bridge_.get();
                        // Capture exactly one immutable value for this
                        // command.  Freshness is deliberately resolved only
                        // after NORMAL Preview in evaluateNormalAllocator().
                        matched_input.captured_gcoord =
                            phase_offset_sph_ros_bridge_->captureGCoord();
                        matched_input.g_des_valid = false;
                    } else if (coordination_backend_ ==
                               PhaseOffsetCoordinationBackend::D1B) {
                        // Legacy complete-g_des remains available only for
                        // the explicitly selected D1B compatibility path.
                        matched_input.g_des_valid =
                            capturePhaseOffsetGDes(matched_input.g_des);
                    } else {
                        matched_input.g_des_valid = false;
                    }
                    if (matched_config_.mode ==
                        PhaseOffsetMatchedMode::MANUAL) {
                        matched_input.section_bundle = section_for_command;
                    }
                    matched_input_snapshot = matched_input;
                    matched_input_snapshot_valid = true;
                    matched_output = MatchedAdapterOutput();
                    const bool adapter_update_success =
                        phase_offset_matched_adapter_->update(
                            matched_input, matched_output);
                    last_recovery_status_ = matched_output.recovery_status;
                    bool section_output_selected = adapter_update_success &&
                        matched_output.selected;
                    // Diagnostic only: name why the Section port refused this
                    // tick.  The governor reports "guidance_invalid" without
                    // this string, so the dominant cause of a HOLD could not be
                    // told apart from a stale bundle, a non-finite motion, or a
                    // rejected selected step.
                    if (!section_output_selected) {
                        ROS_WARN_THROTTLE(
                            1.0,
                            "[branch-debug] section port unavailable "
                            "update_ok=%d selected=%d reason='%s'",
                            (int)adapter_update_success,
                            (int)matched_output.selected,
                            matched_output.invalid_reason.c_str());
                    }

                    // A freshly staged Section candidate is only a proposal.
                    // If its profile/preview cannot support this tick, retry
                    // the still-committed immutable bundle before falling
                    // back to the nominal centreline.  This is especially
                    // important with a nonzero retained delta: using the
                    // baseline guidance after a failed Section update would
                    // silently command delta=0 while Runtime still owns the
                    // old offset.
                    if (!section_output_selected &&
                        matched_config_.mode == PhaseOffsetMatchedMode::MANUAL &&
                        section_for_command && section_current &&
                        section_for_command != section_current &&
                        section_current->path &&
                        !section_current->path->empty()) {
                        // Keep the existing nonzero reference's recenter
                        // intent when retrying the committed profile.  The
                        // request is advisory Runtime state; the actual
                        // step still has to pass the normal Section preview
                        // and publish transaction below.
                        if (retained_nonzero_section()) {
                            phase_offset_matched_adapter_->requestRecenter();
                        }
                        ContinuousPhasePathState fallback_state;
                        const bool fallback_state_valid =
                            section_current->frame &&
                            section_current->frame->evaluatePathState(
                                phase_before, fallback_state);
                        const gvf::LiftedGuidanceResult fallback_guidance =
                            fallback_state_valid
                                ? pm.gvf_->calcLiftedGuidanceAtPhase(
                                      pos, phase_before,
                                      section_current->path)
                                : pm.gvf_->calcLiftedGuidanceAtPhase(
                                      pos, phase_before,
                                      section_current->path);
                        // Once the candidate fails, the committed current
                        // path is the only legal zero-delta fallback.  Never
                        // continue with guidance evaluated on an uncommitted
                        // candidate just because the adapter rejected it.
                        section_for_command = section_current;
                        command_path = section_current->path;
                        // Clear the candidate guidance unconditionally.  A
                        // current-frame evaluation failure must become the
                        // existing invalid-hold path; retaining a valid
                        // candidate `out` while switching `command_path` would
                        // issue guidance from an uncommitted path (and could
                        // advance phase with delta zero).
                        out = fallback_guidance;
                        matched_input_snapshot_valid = false;
                        if (fallback_guidance.valid) {
                            MatchedAdapterInput fallback_input = matched_input;
                            if (!fallback_state_valid) {
                                fallback_input.path =
                                    phase_offset_core::PathDifferentialState();
                            } else {
                                fallback_input.path =
                                    ConvertContinuousPhasePathStateForActive(
                                        fallback_state, phase_before);
                            }
                            fallback_input.semantic_path_owner =
                                section_current->path;
                            fallback_input.semantic_path_start_w =
                                section_current->path->startW();
                            fallback_input.semantic_path_end_w =
                                section_current->path->endW();
                            fallback_input.section_bundle = section_current;
                            fallback_input.legacy = LegacyGuidanceSnapshot(
                                fallback_guidance.v_cmd,
                                fallback_guidance.w_dot,
                                fallback_guidance.e_parallel,
                                fallback_guidance.e_perp,
                                fallback_guidance.ref_pt,
                                fallback_guidance.tangent,
                                fallback_guidance.valid);
                            if (fallback_state_valid) {
                                matched_output = MatchedAdapterOutput();
                                const bool fallback_update_success =
                                    phase_offset_matched_adapter_->update(
                                        fallback_input, matched_output);
                                last_recovery_status_ =
                                    matched_output.recovery_status;
                            section_output_selected =
                                fallback_update_success &&
                                matched_output.selected;
                                if (section_output_selected) {
                                    // The governor and the publication tuple
                                    // must use the same committed path that
                                    // succeeded in the adapter retry.
                                    matched_input_snapshot = fallback_input;
                                    matched_input_snapshot_valid = true;
                                }
                            }
                        }
                    } else if (!section_output_selected &&
                               matched_config_.mode ==
                                   PhaseOffsetMatchedMode::MANUAL &&
                               section_for_command && section_candidate &&
                               section_for_command == section_candidate) {
                        // There is no committed Section owner to retry.  A
                        // neutral command may still follow the planner's
                        // currently committed path, but a nonzero Runtime
                        // reference must not jump to the candidate/centerline.
                        section_for_command = section_current;
                        command_path = section_current && section_current->path
                            ? section_current->path : planner_path;
                        out = pm.gvf_->calcLiftedGuidanceAtPhase(
                            pos, phase_before, command_path);
                    }
                    if (!adapter_update_success && section_candidate &&
                        matched_input.section_bundle == section_candidate &&
                        section_candidate != section_current) {
                        retire_rejected_section_candidate(section_candidate);
                    }
                    if (section_output_selected)
                    {
                        out.v_cmd = matched_output.guidance.v_cmd;
                        out.w_proj = phase_before;
                        out.w_dot = matched_output.guidance.w_dot;
                        out.e_parallel = matched_output.guidance.e_parallel;
                        out.e_perp = matched_output.guidance.e_perp;
                        out.ref_pt = matched_output.guidance.ref_pt;
                        out.tangent = matched_output.guidance.tangent;
                        out.valid = matched_output.guidance.valid;
                        // The governor consumes only the Runtime state that
                        // was authoritative at this callback's start.  A
                        // selected output is the sole source of a nonzero
                        // reference; observe-only and all fallback paths use
                        // the zero-port anchor.
                        governor_reference_delta = matched_output.delta;
                    }
                    else if (retained_nonzero_section()) {
                        // Optional: keep the offset alive (never drop it) and
                        // follow the coordinated reference while no certified
                        // Section exists.  Off by default -> previous hold.
                        bool recovered = false;
                        if (recovery_from_hold_enable_) {
                            double delta_now = 0.0;
                            {
                                std::lock_guard<std::mutex> recovery_lock(
                                    phase_offset_matched_adapter_
                                        ->runtime_command_mutex_);
                                if (phase_offset_matched_adapter_->runtime_) {
                                    phase_offset_matched_adapter_->runtime_
                                        ->applyBoundedDeltaDecayNoFail(
                                            recovery_delta_step_);
                                    delta_now = phase_offset_matched_adapter_
                                                    ->runtime_->retainedDelta();
                                }
                            }
                            const SectionPathBundlePtr nominal_bundle =
                                (section_current && section_current->path)
                                    ? section_current : section_for_command;
                            if (nominal_bundle && nominal_bundle->path &&
                                !nominal_bundle->path->empty()) {
                                section_for_command = nominal_bundle;
                                command_path = nominal_bundle->path;
                                out = pm.gvf_->calcLiftedGuidanceAtPhaseWithDelta(
                                    pos, phase_before, command_path, delta_now);
                                governor_reference_delta = delta_now;
                                recovered = out.valid;
                            }
                        }
                        if (!recovered) {
                            phase_offset_matched_adapter_->requestRecenter();
                            out.valid = false;
                        }
                    }
                }
                else
                {
                    bool retain_nonzero_reference =
                        retained_nonzero_section();
                    if (retain_nonzero_reference && recovery_from_hold_enable_) {
                        // Keep the offset (bounded decay) instead of dropping it:
                        // the committed path is still planner-valid, so fly it
                        // on the coordinated reference rather than HOLDing.
                        double delta_now = 0.0;
                        {
                            std::lock_guard<std::mutex> recovery_lock(
                                phase_offset_matched_adapter_
                                    ->runtime_command_mutex_);
                            if (phase_offset_matched_adapter_->runtime_) {
                                phase_offset_matched_adapter_->runtime_
                                    ->applyBoundedDeltaDecayNoFail(
                                        recovery_delta_step_);
                                delta_now = phase_offset_matched_adapter_
                                                ->runtime_->retainedDelta();
                            }
                        }
                        const SectionPathBundlePtr recovery_bundle =
                            (section_current && section_current->path)
                                ? section_current : section_for_command;
                        if (recovery_bundle && recovery_bundle->path &&
                            !recovery_bundle->path->empty()) {
                            section_for_command = recovery_bundle;
                            command_path = recovery_bundle->path;
                            out = pm.gvf_->calcLiftedGuidanceAtPhaseWithDelta(
                                pos, phase_before, command_path, delta_now);
                            governor_reference_delta = delta_now;
                            retain_nonzero_reference = !out.valid;
                        }
                    }
                    if (retain_nonzero_reference) {
                        phase_offset_matched_adapter_->requestRecenter();
                        out.valid = false;
                    } else {
                        const SectionPathBundlePtr nominal_bundle =
                            (section_current && section_current->path &&
                             section_for_command != section_current)
                                ? section_current : section_for_command;
                        if (nominal_bundle && nominal_bundle->path &&
                            !nominal_bundle->path->empty()) {
                        // A temporary frame/evaluation failure must not make
                        // an already committed Section owner disappear.  For
                        // neutral Runtime state, retain nominal guidance on
                        // that committed path and let the normal command
                        // transaction decide whether phase may advance.
                        section_for_command = nominal_bundle;
                        command_path = nominal_bundle->path;
                        out = pm.gvf_->calcLiftedGuidanceAtPhase(
                            pos, phase_before, command_path);
                        } else {
                            deactivate_phase_offset();
                        }
                    }
                    if (ros::isInitialized()) {
                        ROS_WARN_THROTTLE(
                        1.0,
                        retain_nonzero_reference
                            ? "[phase_offset_matched] current semantic path unavailable; retain nonzero Section reference and hold"
                            : "[phase_offset_matched] current semantic path unavailable; committed nominal guidance retained");
                    }
                }
            }
        }
        else
        {
            deactivate_phase_offset();
            out = pm.gvf_->calcLiftedGuidance3D(pos, progress_w_);
        }
        if (phase_offset_shadow_adapter_ &&
            phase_offset_shadow_adapter_->isPublishDue(now))
        {
            const auto shadow_path = pm.gvf_->getContinuousPhasePath();
            ContinuousPhasePathState current_shadow_state;
            std::vector<double> shadow_sample_w;
            std::vector<ContinuousPhasePathState> shadow_sample_states;
            if (shadow_path && !shadow_path->empty() &&
                shadow_path->evaluate(shadow_semantic_w, current_shadow_state, false) &&
                shadow_path->sample(phase_offset_shadow_adapter_->sampleStepW(),
                                    shadow_sample_w,
                                    shadow_sample_states) &&
                shadow_sample_w.size() == shadow_sample_states.size())
            {
                ShadowUpdateInput shadow_input;
                shadow_input.current_path = ConvertContinuousPhasePathState(
                    current_shadow_state, shadow_semantic_w);
                shadow_input.position = pos;
                shadow_input.stamp = now;
                shadow_input.sampled_path.reserve(shadow_sample_states.size());
                for (size_t index = 0; index < shadow_sample_states.size(); ++index)
                {
                    shadow_input.sampled_path.push_back(ConvertContinuousPhasePathState(
                        shadow_sample_states[index], shadow_sample_w[index]));
                }
                phase_offset_shadow_adapter_->update(shadow_input);
            }
            else
            {
                ROS_WARN_THROTTLE(1.0,
                                  "[phase_offset_shadow] current semantic path unavailable");
            }
        }
        if (!out.valid)
        {
            result = makeGovernorInvalidHold(pos, "guidance_invalid", dbg);
        }
        else
        {
            PendingPositionCommandCapture pending_capture =
                phase_offset_matched_adapter_
                    ? phase_offset_matched_adapter_->capturePendingPositionCommand()
                    : PendingPositionCommandCapture();
            // A future-prefix Section candidate is selected optimistically so
            // its immutable path/frame drive the same geometry as guidance.
            // Once Runtime preparation reveals the exact next-w witness, a
            // prefix crossing is rejected and retried against the still
            // committed current bundle in this same command tick.
            const bool selected_future_section =
                matched_config_.mode == PhaseOffsetMatchedMode::MANUAL &&
                section_for_command && section_candidate &&
                section_for_command == section_candidate && planner_path &&
                section_candidate->path &&
                section_candidate->path != planner_path &&
                pending_capture.section_transition &&
                pending_capture.section_source_path &&
                std::isfinite(pending_capture.section_copied_prefix_start_w) &&
                std::isfinite(pending_capture.section_copied_prefix_end_w);
            const bool future_section_prefix_crossed =
                selected_future_section &&
                (!std::isfinite(pending_capture.section_current_w) ||
                 !std::isfinite(pending_capture.section_next_w) ||
                 pending_capture.section_current_w <
                     pending_capture.section_copied_prefix_start_w ||
                 pending_capture.section_current_w >
                     pending_capture.section_copied_prefix_end_w ||
                 pending_capture.section_next_w >
                     pending_capture.section_copied_prefix_end_w);
            if (future_section_prefix_crossed) {
                if (phase_offset_matched_adapter_) {
                    phase_offset_matched_adapter_->discardPendingPositionCommand();
                }
                retire_rejected_section_candidate(section_candidate);
                if (section_current && section_current->path &&
                    matched_input_snapshot_valid) {
                    section_for_command = section_current;
                    command_path = section_current->path;
                    MatchedAdapterInput fallback_input = matched_input_snapshot;
                    fallback_input.section_bundle = section_current;
                    fallback_input.semantic_path_owner = command_path;
                    fallback_input.semantic_path_start_w = command_path->startW();
                    fallback_input.semantic_path_end_w = command_path->endW();
                    ContinuousPhasePathState fallback_state;
                    bool fallback_state_valid = false;
                    if (section_current->frame &&
                        section_current->path == command_path) {
                        fallback_state_valid =
                            section_current->frame->evaluatePathState(
                                phase_before, fallback_state);
                    } else if (command_path && !command_path->empty()) {
                        fallback_state_valid = command_path->evaluate(
                            phase_before, fallback_state, false);
                    }
                    const gvf::LiftedGuidanceResult fallback_guidance =
                        fallback_state_valid
                            ? pm.gvf_->calcLiftedGuidanceAtPhase(
                                  pos, phase_before, command_path)
                            : gvf::LiftedGuidanceResult();
                    if (fallback_guidance.valid &&
                        phase_offset_matched_adapter_) {
                        fallback_input.path =
                            ConvertContinuousPhasePathStateForActive(
                                fallback_state, phase_before);
                        fallback_input.legacy = LegacyGuidanceSnapshot(
                            fallback_guidance.v_cmd,
                            fallback_guidance.w_dot,
                            fallback_guidance.e_parallel,
                            fallback_guidance.e_perp,
                            fallback_guidance.ref_pt,
                            fallback_guidance.tangent,
                            fallback_guidance.valid);
                        matched_output = MatchedAdapterOutput();
                        const bool fallback_update_success =
                            phase_offset_matched_adapter_->update(
                                fallback_input, matched_output);
                        last_recovery_status_ = matched_output.recovery_status;
                        out = fallback_guidance;
                        if (fallback_update_success &&
                            matched_output.selected) {
                            out.v_cmd = matched_output.guidance.v_cmd;
                            out.w_proj = phase_before;
                            out.w_dot = matched_output.guidance.w_dot;
                            out.e_parallel = matched_output.guidance.e_parallel;
                            out.e_perp = matched_output.guidance.e_perp;
                            out.ref_pt = matched_output.guidance.ref_pt;
                            out.tangent = matched_output.guidance.tangent;
                            out.valid = matched_output.guidance.valid;
                            governor_reference_delta = matched_output.delta;
                        } else {
                            out.valid = false;
                        }
                    } else {
                        out.valid = false;
                    }
                } else {
                    out.valid = false;
                }
                pending_capture = phase_offset_matched_adapter_
                    ? phase_offset_matched_adapter_->capturePendingPositionCommand()
                    : PendingPositionCommandCapture();
            }
            pending_section_environment_bundle = pending_capture.section_bundle;
            if (phase_offset_matched_adapter_) {
                current_section_bundle_for_publication =
                    phase_offset_matched_adapter_->captureSectionBundle();
                staged_section_bundle_for_publication =
                    phase_offset_matched_adapter_->capturePendingSectionBundle();
            }
            pending_position_command_identity = pending_capture.pending
                ? pending_capture.identity : 0U;
            if (unifiedPhaseV2Active() && command_phase.initialized)
            {
                const double requested_phase = phase_before + out.w_dot * dt;
                phase_candidate = requested_phase;
                if (pointPhaseV2Active()) {
                    double path_start_w = 0.0;
                    double path_end_w = 0.0;
                    if (command_path && !command_path->empty()) {
                        path_start_w = command_path->startW();
                        path_end_w = command_path->endW();
                    } else {
                        const gvf::ReparamCacheSnapshot cache =
                            pm.gvf_->captureReparamCacheSnapshot();
                        if (!cache.w.empty()) {
                            path_start_w = cache.w.front();
                            path_end_w = cache.w.back();
                        }
                    }
                    phase_candidate = clampPointPhaseCandidate(
                        requested_phase, path_start_w, path_end_w);
                }
                have_phase_candidate = true;
                will_acquire_closed_phase = closedPhaseV2Active() &&
                    !command_phase.closed_acquired &&
                    out.e_perp.norm() <= closed_phase_acquire_distance_;
            }
            else
            {
                phase_candidate = out.w_proj + out.w_dot * dt;
                legacy_progress_candidate_valid = true;
            }
            if (shouldRunInitialClosedPhaseAcquisition(
                    closedPhaseV2Active(), command_phase.closed_acquired,
                    will_acquire_closed_phase))
            {
                initial_closed_phase_acquisition_used = true;
                const Eigen::Vector3d delta = boundedInitialAcquisitionDelta(
                    out.v_cmd, cmd_vel_max_, kp_equiv, cmd_governor_lead_max_);
                if (delta.norm() <= 1e-9)
                {
                    result = makeGovernorInvalidHold(
                        pos, "initial_acquisition_zero_command", dbg);
                }
                else
                {
                    result.cmd_pos = pos + delta;
                    result.yaw_cmd_vec = delta;
                    result.final_cmd_source = "GVF_INITIAL_ACQUISITION";
                    result.fallback_reason = "none";
                    result.command_valid = true;
                    result.selected_valid_for_state = false;
                    result.reset_state_after_publish = true;

                    dbg.guidance_valid = true;
                    dbg.fallback_hold_pos = false;
                    dbg.raw_v_norm = out.v_cmd.norm();
                    dbg.e_perp_norm = out.e_perp.norm();
                    if (out.tangent.norm() > 1e-6)
                    {
                        const Eigen::Vector3d tangent = out.tangent.normalized();
                        dbg.raw_v_tau = out.v_cmd.dot(tangent);
                        dbg.raw_v_normal_norm =
                            (out.v_cmd - dbg.raw_v_tau * tangent).norm();
                    }
                    dbg.v_tau_intent = std::max(0.0, dbg.raw_v_tau);
                    dbg.v_n_intent_norm = dbg.raw_v_normal_norm;
                    dbg.selected_v_model_norm = (kp_equiv * delta).norm();
                    dbg.cmd_dist = delta.norm();
                }
            }
            else
            {
                const bool pending_ready = !pending_capture.pending ||
                    pending_capture.valid;
                if (!pending_ready) {
                    if (phase_offset_matched_adapter_ && pending_capture.pending) {
                        phase_offset_matched_adapter_->discardPendingPositionCommand();
                    }
                    out.valid = false;
                    result = makeGovernorInvalidHold(
                        pos, "phase_offset_final_validation_failed", dbg);
                } else {
                    // Final validated ImmutableExecutedReferenceQuery is now
                    // the input boundary for the existing governor call.
                    const phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
                        pending_reference_query = pending_capture.reference_query;
                    result = runVelocityMatchingGovernor(
                        command_path, out, pos,
                        have_phase_candidate ? phase_candidate : phase_before,
                        governor_reference_delta,
                        dt, kp_equiv, dbg, pending_reference_query);
                }
            }
        }
    }

    const bool circle_mode_active = enable_circle_reference_test_ && circle_reference_ready_;
    bool force_goal_position = !circle_mode_active && real_dis_to_goal < stop_radius;
    bool preserve_nonzero_reference = false;
    if (force_goal_position && phase_offset_matched_adapter_ &&
        matched_config_.mode == PhaseOffsetMatchedMode::MANUAL &&
        phase_offset_matched_adapter_->runtime_ &&
        phase_offset_matched_adapter_->runtime_->retainedDelta() != 0.0) {
        // Reaching the geometric goal while an offset is still authoritative
        // is not a license to jump to the baseline goal.  Keep the command's
        // existing reference/hold and request the bounded Runtime recenter;
        // only a true zero-offset state may use terminal cancellation.
        preserve_nonzero_reference = true;
        force_goal_position = false;
        phase_offset_matched_adapter_->requestRecenter();
    }
    if (force_goal_position)
    {
        // The explicit goal override replaces any staged offset result.  The
        // Keep the staged offset transaction intact until the replacement
        // PositionCommand has actually published.  The adapter prepares an
        // immutable deactivation token and commits it only on publish success.
        if (phase_offset_matched_adapter_) {
            // Preserve the exact pending command witness for terminal
            // cancellation.  The adapter still checks task generation and
            // retained delta; replacing it with zero would make a real
            // pending candidate impossible to cancel safely.
            const PendingPositionCommandCapture terminal_pending =
                phase_offset_matched_adapter_->capturePendingPositionCommand();
            pending_position_command_identity = terminal_pending.pending
                ? terminal_pending.identity : 0U;
        }
        dbg.final_cmd_overridden = result.command_valid;
        dbg.state_reset_due_to_override = true;
        result.selected_valid_for_state = false;
        result.reset_state_after_publish = true;
        result.cmd_pos = goal;
        result.yaw_cmd_vec.setZero();
        dbg.cmd_dist = (result.cmd_pos - pos).norm();
        if (!result.command_valid)
        {
            result.final_cmd_source = "GOVERNOR_INVALID_HOLD";
            result.fallback_reason = "goal_override_after_invalid_governor";
            dbg.fallback_hold_pos = true;
        }
    }

    AuthoritativePhaseCommitDecision phase_decision =
        decideAuthoritativePhaseCommit(
            have_phase_candidate, result.command_valid, force_goal_position,
            initial_closed_phase_acquisition_used, phase_before, phase_candidate,
            command_phase.closed_acquired,
            will_acquire_closed_phase);
    if (preserve_nonzero_reference &&
        pending_position_command_identity == 0U) {
        // A hold with no prepared Section step may still be published, but it
        // must not advance the manager phase witness without a matching
        // Runtime transaction.
        have_phase_candidate = false;
        phase_decision.commit = false;
        phase_decision.acquire_closed_phase = false;
        phase_decision.phase_after = phase_before;
        phase_decision.applied_delta_w = 0.0;
    }
    // A hold/initial-acquisition command is allowed to publish its position,
    // but it is not a Runtime step.  Retire any prepared Section command
    // before entering the publication lock sequence so no stale delta or
    // executed-reference query can be committed by the hold publication.
    if (!force_goal_position && !phase_decision.commit &&
        phase_offset_matched_adapter_ && pending_position_command_identity != 0U) {
        phase_offset_matched_adapter_->discardPendingPositionCommand();
        pending_position_command_identity = 0U;
        pending_section_environment_bundle.reset();
    }
    // Prepare the phase publication token before any PositionCommand can be
    // emitted.  Holding this existing phase mutex through publication and the
    // no-fail commit serializes the token against reset/goal retirement.
    if (!pending_section_environment_bundle && phase_offset_matched_adapter_) {
        const PendingPositionCommandCapture late_pending_capture =
            phase_offset_matched_adapter_->capturePendingPositionCommand();
        pending_section_environment_bundle = late_pending_capture.section_bundle;
        pending_position_command_identity = late_pending_capture.pending
            ? late_pending_capture.identity : 0U;
        if (!current_section_bundle_for_publication) {
            current_section_bundle_for_publication =
                phase_offset_matched_adapter_->captureSectionBundle();
        }
        if (!staged_section_bundle_for_publication) {
            staged_section_bundle_for_publication =
                phase_offset_matched_adapter_->capturePendingSectionBundle();
        }
    }
    std::unique_lock<std::recursive_mutex> environment_transaction_lock;
    if (pending_section_environment_bundle &&
        pending_section_environment_bundle->environment_change_mutex) {
        environment_transaction_lock = std::unique_lock<std::recursive_mutex>(
            *pending_section_environment_bundle->environment_change_mutex);
    }
    std::unique_lock<std::mutex> frontend_transaction_lock;
    std::unique_lock<std::mutex> handoff_transaction_lock;
    std::unique_lock<std::mutex> phase_transaction_lock;
    AuthoritativePhaseCommitToken phase_commit_token;
    bool phase_commit_ready = true;
    const bool section_transaction_pending =
        static_cast<bool>(pending_section_environment_bundle);
    if (section_transaction_pending) {
        // Section publication is a path/frontend handoff as well: keep the
        // same manager-side serialization boundary even though it carries no
        // legacy V2 authority DTO.
        frontend_transaction_lock = std::unique_lock<std::mutex>(
            frontend_apply_mutex_);
        handoff_transaction_lock = std::unique_lock<std::mutex>(
            path_reference_handoff_mutex_);
    }
    if (section_transaction_pending && pending_section_environment_bundle &&
        pending_section_environment_bundle->path && command_planner_path &&
        pending_section_environment_bundle->path != command_planner_path &&
        (!command_section_path_handoff ||
         command_section_path_handoff->bundle !=
             pending_section_environment_bundle ||
         command_section_path_handoff != pending_section_path_handoff_)) {
        // A cross-path Section command is valid only while the exact manager
        // DTO selected before update() is still registered.  If reset or a
        // newer handoff retired it, leave Runtime/phase untouched and let the
        // next callback use the committed current bundle.
        phase_commit_ready = false;
        phase_offset_matched_adapter_->discardPendingPositionCommand();
        rejected_section_candidate_for_retirement =
            pending_section_environment_bundle;
        pending_section_environment_bundle.reset();
        pending_position_command_identity = 0U;
    }
    if (phase_decision.commit && phase_commit_ready) {
        phase_transaction_lock = std::unique_lock<std::mutex>(
            authoritative_phase_mutex_);
        phase_commit_ready = prepareAuthoritativePhaseCommitLocked(
            command_phase, phase_decision.phase_after,
            phase_decision.acquire_closed_phase, phase_commit_token);
        if (!phase_commit_ready && phase_offset_matched_adapter_) {
            phase_offset_matched_adapter_->discardPendingPositionCommand();
            rejected_section_candidate_for_retirement =
                pending_section_environment_bundle;
            pending_section_environment_bundle.reset();
            pending_position_command_identity = 0U;
            if (phase_transaction_lock.owns_lock()) {
                phase_transaction_lock.unlock();
            }
        }
    }
    // Phase, governor-state and command-history mutation is intentionally
    // deferred until the local PositionCommand publication and any pending
    // authority/runtime transaction have both committed.
    const bool switch_active = now < cmd_switch_motion_limit_until_;
    logGovernorCommand(result, dbg, result.cmd_pos, real_dis_to_goal, kp_equiv, switch_active);
    bool phase_committed_in_publication = false;
    std::function<void()> post_publish_no_fail;
    if (phase_decision.commit && phase_commit_token.valid) {
        post_publish_no_fail = [this, &phase_commit_token,
                                &phase_committed_in_publication]() {
            commitAuthoritativePhaseNoFailLocked(phase_commit_token);
            phase_committed_in_publication = true;
        };
    }
    if (phase_commit_ready && phase_offset_matched_adapter_) {
        // The adapter serializes final validation, the actual cmd_pub.publish
        // invocation, and the no-fail authority/token commit against task
        // reset and pair retirement.
        command_published_and_committed =
            phase_offset_matched_adapter_->publishPendingPositionCommand(
            [this, &result]() {
                return publishGovernorPositionCommand(
                    result.cmd_pos, result.yaw_cmd_vec);
            }, pending_position_command_identity, force_goal_position,
            post_publish_no_fail);
        if (!command_published_and_committed && !force_goal_position &&
            pending_section_environment_bundle) {
            // publishPendingPositionCommand() fails closed on an invalidated
            // environment witness or a crossed copied prefix.  Retire the
            // exact failed pending bundle after the publication locks unwind;
            // the helper rechecks current/staged/handoff ownership under the
            // lock order and will not erase a concurrently committed mirror.
            rejected_section_candidate_for_retirement =
                pending_section_environment_bundle;
        }
    } else if (phase_commit_ready) {
        command_published_and_committed =
            publishGovernorPositionCommand(result.cmd_pos, result.yaw_cmd_vec);
    }
    if (command_published_and_committed && force_goal_position &&
        handoff_transaction_lock.owns_lock()) {
        // A terminal goal override may cancel a staged Section candidate
        // without a phase commit.  Retire its manager DTO now so FSM cannot
        // wait forever for a mirror whose adapter transaction was cancelled.
        pending_section_path_handoff_.reset();
    }
    if (handoff_transaction_lock.owns_lock()) {
        handoff_transaction_lock.unlock();
    }
    if (frontend_transaction_lock.owns_lock()) {
        frontend_transaction_lock.unlock();
    }
    if (rejected_section_candidate_for_retirement) {
        retire_rejected_section_candidate(
            rejected_section_candidate_for_retirement);
    }
    if (command_published_and_committed) {
        if (pending_last_yaw_valid_) {
            last_yaw = pending_last_yaw_;
            pending_last_yaw_valid_ = false;
        }
        if (result.reset_state_after_publish) {
            resetGovernorState();
        }
        if (phase_decision.commit && phase_commit_token.valid) {
            if (!phase_committed_in_publication) {
                commitAuthoritativePhaseNoFailLocked(phase_commit_token);
            }
            progress_w_ = phase_commit_token.committed.w;
            progress_initialized_ = true;
            if (phase_decision.acquire_closed_phase) {
                last_replan_time_ = now;
                ROS_WARN(
                    "[GVF][CLOSED_PHASE_V2][ACQUIRED] phase_w=%.3f error=%.3f",
                    phase_commit_token.committed.w, out.e_perp.norm());
            }
        } else if (!unifiedPhaseV2Active() &&
                   legacy_progress_candidate_valid) {
            // Legacy progress is also a command-owned state transition.
            progress_w_ = phase_candidate;
            progress_initialized_ = true;
        }
        if (out.valid && pm.gvf_) {
            pm.gvf_->setVisualizationProgressW(
                unifiedPhaseV2Active() && command_phase.initialized
                    ? phase_before : activeTrackingPhase());
            // Give the RViz vector field the offset the controller is actually
            // following, so the drawn field is built on the coordinated
            // reference r = p + N*delta.  Display-only: nothing here feeds back
            // into guidance, the tube or the command path.
            double display_delta = 0.0;
            if (phase_offset_matched_adapter_ &&
                matched_config_.mode == PhaseOffsetMatchedMode::MANUAL &&
                phase_offset_matched_adapter_->runtime_) {
                std::lock_guard<std::mutex> runtime_lock(
                    phase_offset_matched_adapter_->runtime_command_mutex_);
                if (phase_offset_matched_adapter_->runtime_) {
                    display_delta = phase_offset_matched_adapter_->runtime_->
                        retainedDelta();
                }
            }
            pm.gvf_->setVisualizationDelta(display_delta);
        }
        if (result.selected_valid_for_state) {
            cmd_governor_normal_state_ = result.selected_n;
            cmd_governor_last_l_ = result.selected_l;
            cmd_governor_initialized_ = true;
            dbg.normal_state_norm = cmd_governor_normal_state_.norm();
        }
        updateGovernorCommandHistory(pos, result.cmd_pos, dt, dbg);
    }
    // Publish SPH beta only after the command's final Section/current retry,
    // prefix decision, goal override, and PositionCommand transaction have
    // settled.  Publishing from an earlier candidate update can expose that
    // rejected candidate's beta even when the committed bundle ultimately
    // supplies the command.  A hold, override, or failed publication keeps the
    // original invalid beta behavior.
    const bool final_sph_beta_authoritative =
        command_published_and_committed && !force_goal_position &&
        !initial_closed_phase_acquisition_used && result.command_valid &&
        matched_output.selected;
    publish_sph_beta(final_sph_beta_authoritative ? &matched_output : nullptr,
                     final_sph_beta_authoritative);
    finish_callback_timing();
}

void gvf_manager::test_cmdCallback(const ros::TimerEvent& event)
{
    if (!use_test_cmd_) return;  // 如果不使用测试命令模式，则跳过
    
    if (swarmParticlesManager.empty()) return;
    
    auto& pm = swarmParticlesManager[0];
    
    // 检查是否有轨迹数据
    if (pm.last_traj.rows() == 0) {
        ROS_WARN_THROTTLE(1.0, "[TEST_CMD] No trajectory data available");
        return;
    }
    
    // 检查索引是否超出范围
    if (test_traj_index_ >= pm.last_traj.rows()) {
        ROS_INFO_THROTTLE(1.0, "[TEST_CMD] Trajectory completed, index: %d, total points: %d", 
                         test_traj_index_, static_cast<int>(pm.last_traj.rows()));
        return;
    }
    
    // 获取当前轨迹点
    Eigen::Vector3d pos = pm.last_traj.row(test_traj_index_).transpose();
    Eigen::Vector3d vel = pm.last_vel.row(test_traj_index_).transpose();
    
    // 构造 PositionCommand 消息
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    
    // 设置位置控制
    cmd.position.x = pos.x();
    cmd.position.y = pos.y();
    cmd.position.z = pos.z();
    
    // 设置速度控制
    cmd.velocity.x = vel.x();
    cmd.velocity.y = vel.y();
    cmd.velocity.z = vel.z();
    
    // 计算yaw角度
    double arg_ = atan2(-cmd.velocity.x, cmd.velocity.y) + (PI/2.0f);
    double vel_len = sqrt(pow(cmd.velocity.x, 2) + pow(cmd.velocity.y, 2));
    if (vel_len <= 0.1) arg_ = last_yaw;
    std::pair<double, double> yaw_all = calculate_yaw(last_yaw, arg_);
    
    cmd.yaw = yaw_all.first;
    cmd.yaw_dot = yaw_all.second;
    
    // 发布控制指令
    cmd_pub.publish(cmd);
    
    // 更新索引
    test_traj_index_++;
    
    ROS_INFO_THROTTLE(0.5, "[TEST_CMD] Following trajectory point %d/%d, pos: (%.3f, %.3f, %.3f)", 
             test_traj_index_, static_cast<int>(pm.last_traj.rows()),
             pos.x(), pos.y(), pos.z());
}

    void gvf_manager::InitGvf(ros::NodeHandle &nh)
{
        try {
            std::string particle_base = "/particle0";
            // //EDT & MAP
            auto sdf_map_ = std::make_shared<SDFMap>();
            sdf_map_->initMap(nh, particle_base, odom_topic_, cloud_topic_);
            auto edt_environment_ = std::make_shared<EDTEnvironment>();
            edt_environment_->setMap(sdf_map_);
            
            //ASTAR
            auto geo_path_finder_ = std::make_shared<AstarTopo>();
            geo_path_finder_->setParam(nh);
            geo_path_finder_->setEnvironment(edt_environment_);
            geo_path_finder_->init();

            // dynamic a*
            auto kino_path_finder_ = std::make_shared<KinodynamicAstar>();
            kino_path_finder_->setParam(nh);
            kino_path_finder_->setEnvironment(edt_environment_);
            kino_path_finder_->init();

            //OPT
            auto bspline_opt_ = std::make_shared<bspline_optimizer>();
            bspline_opt_->init(nh);
            bspline_opt_->setEnvironment(edt_environment_);

            //UNIFORM BSPLINE
            auto spline_ = std::make_shared<UniformBspline>();
            spline_->init(nh);

            // gvf
            auto gvf_ = std::make_shared<gvf>();
            gvf_->init(nh, particle_base, odom_topic_, cloud_topic_);

            gvfManager pm {
                particle_base,
                sdf_map_,
                edt_environment_,
                geo_path_finder_,
                kino_path_finder_,
                bspline_opt_,
                spline_,
                gvf_,
                ros::Time::now(),     // curr_time
                ros::Time(0),         // last_time
                true, 
            };

            swarmParticlesManager.push_back(pm);  // 将实例存入向量

            std::cout << "\033[1;33m" << "-----------------------------------------" << "\033[0m" << std::endl;

        } catch (const std::exception& e) {
            ROS_ERROR("Exception caught while initializing environments for %s", e.what());
        }  
    }

void gvf_manager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    Eigen::Vector3d curr_pos(
        msg->pose.pose.position.x + 0.000001,
        msg->pose.pose.position.y + 0.000001,
        msg->pose.pose.position.z);

    ros::Time curr_time = msg->header.stamp;
    if (curr_time.isZero()) {
        curr_time = ros::Time::now();
    }

    odom_pos_history_.push_back({curr_time, curr_pos});
    const double history_keep_time = std::max(odom_vel_est_window_ + 1.0, 1.0);
    while (odom_pos_history_.size() > 2 &&
           (curr_time - odom_pos_history_.front().t).toSec() > history_keep_time)
    {
        odom_pos_history_.pop_front();
    }

    if (odom_pos_history_.size() >= 2)
    {
        const double target_window = std::max(0.0, odom_vel_est_window_);
        const OdomPosSample* old_sample = &odom_pos_history_.front();
        for (const auto& sample : odom_pos_history_)
        {
            if ((curr_time - sample.t).toSec() >= target_window)
            {
                old_sample = &sample;
            }
            else
            {
                break;
            }
        }

        const double dt = (curr_time - old_sample->t).toSec();
        if (dt > 1e-3 && dt < 2.0)
        {
            const Eigen::Vector3d raw_vel = (curr_pos - old_sample->p) / dt;
            if (raw_vel.norm() <= 5.0)
            {
                odom_vel_est_ = raw_vel;

                if (!odom_vel_initialized_)
                {
                    odom_vel_lpf_ = raw_vel;
                    odom_vel_initialized_ = true;
                }
                else
                {
                    const double lpf_hz = std::max(0.0, odom_vel_lpf_hz_);
                    double alpha = 1.0;
                    if (lpf_hz > 1e-6)
                    {
                        const double tau = 1.0 / (2.0 * PI * lpf_hz);
                        alpha = dt / (tau + dt);
                    }
                    odom_vel_lpf_ = odom_vel_lpf_ + alpha * (raw_vel - odom_vel_lpf_);
                }

                odom_vel_est_ = odom_vel_lpf_;
            }
        }
    }

    last_odom_pos_ = curr_pos;
    last_odom_time_ = curr_time;
    has_last_odom_ = true;
    this->odom_ = curr_pos;
    
    if (!enable_circle_reference_test_ || !circle_reference_auto_start_ || circle_reference_auto_started_) {
        return;
    }
    if (swarmParticlesManager.empty()) {
        return;
    }

    Eigen::Vector3d start_pt(odom_.x(), odom_.y(), odom_.z());
    Eigen::Vector3d center(circle_reference_center_x_, circle_reference_center_y_, circle_reference_center_z_);

    progress_w_ = 0.0;
    progress_initialized_ = false;
    if (!resetForNewNavigationTask()) {
        ROS_ERROR("[GVF][NEW_TASK_AUTHORITY_RESET] auto-start reset rejected");
        return;
    }
    closed_ref_w_ = 0.0;
    closed_ref_initialized_ = false;
    closed_ref_recover_ = false;
    resetClosedGoalCandidateState();
    ref_pos = start_pt;
    last_curve_vel_.setZero();
    has_last_curve_vel_ = false;
    cmd_switch_motion_limit_until_ = ros::Time(0);
    last_governor_cmd_pos_ = start_pt;
    last_governor_cmd_vel_.setZero();
    has_last_governor_cmd_ = false;
    cmd_governor_normal_state_.setZero();
    cmd_governor_initialized_ = false;
    cmd_governor_last_l_ = 0.0;
    ref_initialized = false;
    last_cmd_pos_ = start_pt;

    if (reference_shape_ == "figure8" || reference_shape_ == "8" || reference_shape_ == "lemniscate") {
        generateFigureEightReference(center);
    } else {
        generateCircleReference(center);
    }

    Eigen::Vector3d goal_pt = start_pt;
    if (circle_reference_ready_ && !closedPhaseV2Enabled()) {
        goal_pt = getCircleReferenceGoal(start_pt).first;
    }

    for (auto& manager : swarmParticlesManager) {
        if (manager.gvf_ &&
            !(unifiedPhaseV2Active() && phase_offset_matched_adapter_ &&
              phase_offset_matched_adapter_->requiresAuthoritativeOffsetHandoff())) {
            manager.gvf_->clearPathReparamState();
        }
        manager.receive_startpt = true;
        manager.start_pt = start_pt;
        manager.goal_pt = goal_pt;
        manager.is_first_goal = true;
        manager.receive_goal = true;
    }

    circle_reference_auto_started_ = true;
    ROS_INFO("[GVF] auto start %s reference centered at (%.2f, %.2f, %.2f)",
             reference_shape_.c_str(), center.x(), center.y(), center.z());
}

void gvf_manager::publishReferencePathMsg(const Eigen::MatrixXd& traj, const Eigen::MatrixXd& vel, ros::Publisher& pub)
{
    if (traj.rows() <= 0) return;
    if (vel.rows() != traj.rows()) return;

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();

    for (int i = 0; i < traj.rows(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = traj(i, 0);
        pose.pose.position.y = traj(i, 1);
        pose.pose.position.z = traj(i, 2);
        pose.pose.orientation.x = vel(i, 0);
        pose.pose.orientation.y = vel(i, 1);
        pose.pose.orientation.z = vel(i, 2);
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    pub.publish(path_msg);
}

bool gvf_manager::closedPhaseV2Enabled() const
{
    return closed_tracking_mode_ == "closed_phase_v2";
}

bool gvf_manager::closedPhaseV2Active() const
{
    return closedPhaseV2Enabled() && enable_circle_reference_test_ && circle_reference_ready_;
}

bool gvf_manager::pointPhaseV2Active() const
{
    return point_phase_v2_enabled_ &&
           !(enable_circle_reference_test_ && circle_reference_ready_);
}

bool gvf_manager::unifiedPhaseV2Active() const
{
    return closedPhaseV2Active() || pointPhaseV2Active();
}

double gvf_manager::activeTrackingPhase() const
{
    const AuthoritativePhaseSnapshot phase = captureAuthoritativePhase();
    return unifiedPhaseV2Active() && phase.initialized ? phase.w : progress_w_;
}

bool gvf_manager::resetForNewNavigationTask()
{
    {
        // A new goal owns a new path coordinate.  Retire the adapter task and
        // manager-side Section handoff under the established publication lock order.
        std::lock_guard<std::mutex> apply_lock(frontend_apply_mutex_);
        std::lock_guard<std::mutex> handoff_lock(path_reference_handoff_mutex_);
        std::lock_guard<std::mutex> phase_lock(authoritative_phase_mutex_);
        if (phase_offset_matched_adapter_ &&
            !phase_offset_matched_adapter_->resetForNewNavigationTask()) {
            ROS_ERROR("[GVF][NEW_TASK_AUTHORITY_RESET] adapter task reset failed");
            return false;
        }
        pending_section_path_handoff_.reset();
        publishAuthoritativePhaseLocked(0.0, false, false);
    }
    closed_phase_pending_path_end_w_ = 0.0;
    closed_phase_has_pending_path_end_w_ = false;
    last_section_build_time_ = ros::Time(0);
    // A new task owns a new path, so the previous tube geometry must be
    // retired rather than left visible until the next successful build.
    last_section_marker_bundle_.reset();
    last_section_marker_time_ = ros::Time(0);
    // Hand the frontend retirement to the FSM thread: it owns pm.last_* and the
    // executor state, and the phase origin published above only becomes
    // consistent with a freshly installed frontend once that thread has run.
    new_task_pending_.store(true, std::memory_order_release);
    return true;
}

double gvf_manager::findInitialClosedPhaseV2(const Eigen::Vector3d& curr_pos) const
{
    const int N = static_cast<int>(circle_reference_traj_.rows());
    if (N <= 1 || circle_reference_w_.size() != static_cast<size_t>(N) ||
        circle_reference_total_w_ <= 1e-9) {
        return 0.0;
    }

    const bool circle_shape =
        reference_shape_ != "figure8" && reference_shape_ != "8" &&
        reference_shape_ != "lemniscate";
    const double center_distance =
        (curr_pos - circle_reference_center_).head<2>().norm();
    if (circle_shape && center_distance <= 1e-3) {
        return closed_ref_initial_phase_w_ >= 0.0
            ? wrapClosedW(closed_ref_initial_phase_w_)
            : 0.0;
    }

    struct PhaseCandidate {
        double w = 0.0;
        double dist_sq = std::numeric_limits<double>::infinity();
        double tangent_dot_odom = -1.0;
    };

    const Eigen::Vector2d odom_v_xy = odom_vel_lpf_.head<2>();
    const double odom_v_norm = odom_v_xy.norm();
    const bool use_direction = odom_v_norm > 0.2;
    Eigen::Vector2d odom_dir = Eigen::Vector2d::Zero();
    if (use_direction) {
        odom_dir = odom_v_xy / odom_v_norm;
    }

    std::vector<PhaseCandidate> candidates;
    candidates.reserve(N);
    double min_dist_sq = std::numeric_limits<double>::infinity();
    for (int i = 0; i < N; ++i) {
        const int j = (i + 1) % N;
        const Eigen::Vector3d p0 = circle_reference_traj_.row(i).transpose();
        const Eigen::Vector3d p1 = circle_reference_traj_.row(j).transpose();
        const Eigen::Vector3d seg = p1 - p0;
        const double seg_len_sq = seg.squaredNorm();
        if (seg_len_sq <= 1e-12) continue;

        const double u = std::max(0.0, std::min(1.0,
            (curr_pos - p0).dot(seg) / seg_len_sq));
        const Eigen::Vector3d projection = p0 + u * seg;
        PhaseCandidate candidate;
        candidate.dist_sq = (curr_pos - projection).squaredNorm();
        const double w0 = circle_reference_w_[i];
        const double w1 = i == N - 1
            ? circle_reference_total_w_
            : circle_reference_w_[j];
        candidate.w = w0 + u * (w1 - w0);

        const Eigen::Vector2d tangent_xy = seg.head<2>();
        if (use_direction && tangent_xy.norm() > 1e-9) {
            candidate.tangent_dot_odom = tangent_xy.normalized().dot(odom_dir);
        }
        candidates.push_back(candidate);
        min_dist_sq = std::min(min_dist_sq, candidate.dist_sq);
    }

    if (candidates.empty()) return 0.0;

    const double sample_step = circle_reference_total_w_ / static_cast<double>(N);
    const double tie_distance = std::max(1e-4, 0.05 * sample_step);
    const double tie_dist_sq = tie_distance * tie_distance;
    PhaseCandidate best;
    bool have_best = false;
    for (const auto& candidate : candidates) {
        if (candidate.dist_sq > min_dist_sq + tie_dist_sq) continue;
        if (!have_best ||
            (use_direction && candidate.tangent_dot_odom > best.tangent_dot_odom + 1e-6) ||
            ((!use_direction || std::abs(candidate.tangent_dot_odom - best.tangent_dot_odom) <= 1e-6) &&
             candidate.dist_sq < best.dist_sq - 1e-12) ||
            (std::abs(candidate.dist_sq - best.dist_sq) <= 1e-12 && candidate.w < best.w)) {
            best = candidate;
            have_best = true;
        }
    }
    return have_best ? best.w : 0.0;
}

bool gvf_manager::buildNominalClosedFrontend(
    double start_w,
    Eigen::MatrixXd& traj,
    Eigen::MatrixXd& vel,
    Eigen::VectorXd& time,
    std::vector<double>& global_w) const
{
    const int reference_points = static_cast<int>(circle_reference_traj_.rows());
    if (!circle_reference_ready_ || reference_points < 2 ||
        circle_reference_total_w_ <= 1e-9) {
        return false;
    }

    const double frontend_length = std::max(
        1.0,
        std::max(closed_goal_prefer_lookahead_w_,
                 cmd_governor_l_max_ + std::max(0.0, switch_governor_path_margin_w_)));
    const double nominal_step = std::max(
        1e-3, circle_reference_total_w_ / static_cast<double>(reference_points));
    const int count = std::max(2, static_cast<int>(std::ceil(frontend_length / nominal_step)) + 1);

    traj.resize(count, 3);
    vel.resize(count, 3);
    time.resize(count);
    global_w.resize(count);
    time(0) = 0.0;
    const double nominal_speed = std::max(0.1, cmd_tangent_vel_max_);

    for (int i = 0; i < count; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(count - 1);
        const double w = start_w + ratio * frontend_length;
        global_w[i] = w;
        traj.row(i) = pointFromClosedW(w).transpose();
        vel.row(i) = (nominal_speed * tangentFromClosedW(w)).transpose();
        if (i > 0) {
            const double ds = (traj.row(i) - traj.row(i - 1)).norm();
            time(i) = time(i - 1) + ds / nominal_speed;
        }
    }
    return true;
}

bool gvf_manager::buildMappedPhaseFrontend(
    double phase_anchor,
    double path_end_w,
    bool trim_start_endpoint,
    int candidate_anchor_idx,
    const UniformBspline& candidate_spline,
    const Eigen::MatrixXd& candidate_traj,
    const Eigen::VectorXd& candidate_time,
    Eigen::MatrixXd& mapped_traj,
    Eigen::MatrixXd& mapped_vel,
    Eigen::VectorXd& mapped_time,
    std::vector<double>& mapped_w,
    std::shared_ptr<const ContinuousPhasePath>& continuous_path)
{
    continuous_path.reset();
    const int rows = static_cast<int>(candidate_traj.rows());
    if (rows < 2 || candidate_time.size() != rows ||
        !candidate_time.allFinite() ||
        !std::isfinite(phase_anchor) || !std::isfinite(path_end_w) ||
        path_end_w <= phase_anchor + 1e-6) {
        return false;
    }

    candidate_anchor_idx = std::max(0, std::min(candidate_anchor_idx, rows - 2));
    const double raw_t_anchor = candidate_time(candidate_anchor_idx);
    const double raw_t_end = candidate_spline.t_range(1);
    if (!std::isfinite(raw_t_anchor) || !std::isfinite(raw_t_end) ||
        raw_t_anchor < candidate_spline.t_range(0) - 1e-8 ||
        raw_t_anchor >= raw_t_end - 1e-8) {
        return false;
    }
    double raw_length = 0.0;
    double spline_t_anchor = 0.0;
    double spline_t_end = 0.0;
    const double requested_span = path_end_w - phase_anchor;
    const double endpoint_margin = std::min(
        point_phase_endpoint_margin_w_, 0.25 * requested_span);
    const double trim_start = trim_start_endpoint ? endpoint_margin : 0.0;
    const double trim_end = endpoint_margin;
    if (!ContinuousPhasePath::trimBsplineTimeDomainByArcLength(
            candidate_spline, raw_t_anchor, raw_t_end, trim_start, trim_end,
            spline_t_anchor, spline_t_end, raw_length)) {
        return false;
    }
    const double semantic_start_w = phase_anchor + trim_start;
    const double semantic_end_w = path_end_w - trim_end;
    if (semantic_end_w <= semantic_start_w + 1e-6) return false;
    const auto evaluator = ContinuousPhasePath::makeMappedBspline(
        candidate_spline, spline_t_anchor, spline_t_end,
        semantic_start_w, semantic_end_w);
    if (!evaluator) return false;

    auto path = std::make_shared<ContinuousPhasePath>();
    if (!path->appendSegment(
            semantic_start_w, semantic_end_w, "mapped_bspline", evaluator)) {
        return false;
    }

    const std::shared_ptr<const ContinuousPhasePath> immutable_path = path;
    if (!sampleContinuousPhasePath(
            immutable_path, mapped_traj, mapped_vel, mapped_time, mapped_w)) {
        return false;
    }
    // Stamp the path identity exactly like every other constructed frontend
    // does.  Without it pathRevision() stays 0 and the Section bundle builder
    // rejects the path ("build guard rejected ... rev=0"), which deadlocked
    // the switch: the tube could not be rebuilt and navigation stayed on the
    // exhausted frontend forever.
    if (!assignPathIdentity(immutable_path, continuous_path)) {
        return false;
    }
    return true;
}

bool gvf_manager::buildNominalContinuousPhasePath(
    double start_w,
    double end_w,
    std::shared_ptr<const ContinuousPhasePath>& path) const
{
    path.reset();
    if (!circle_reference_ready_ || circle_reference_total_w_ <= 1e-9 ||
        !std::isfinite(start_w) || !std::isfinite(end_w) ||
        end_w <= start_w + 1e-6) {
        return false;
    }

    const bool figure8 = reference_shape_ == "figure8" ||
                         reference_shape_ == "8" ||
                         reference_shape_ == "lemniscate";
    const double radius = figure8
        ? std::max(0.3, figure8_reference_radius_)
        : std::max(0.3, circle_reference_radius_);
    const double nominal_speed = std::max(0.1, cmd_tangent_vel_max_);
    const auto evaluator = figure8
        ? ContinuousPhasePath::makePeriodicFigureEight(
              circle_reference_center_, radius, circle_reference_total_w_, nominal_speed)
        : ContinuousPhasePath::makePeriodicCircle(
              circle_reference_center_, radius, circle_reference_total_w_, nominal_speed);
    if (!evaluator) return false;

    auto result = std::make_shared<ContinuousPhasePath>();
    if (!result->appendSegment(start_w, end_w,
                               figure8 ? "nominal_figure8" : "nominal_circle",
                               evaluator)) {
        return false;
    }
    path = result;
    return true;
}

bool gvf_manager::sampleContinuousPhasePath(
    const std::shared_ptr<const ContinuousPhasePath>& path,
    Eigen::MatrixXd& traj,
    Eigen::MatrixXd& vel,
    Eigen::VectorXd& time,
    std::vector<double>& global_w) const
{
    traj.resize(0, 0);
    vel.resize(0, 0);
    time.resize(0);
    global_w.clear();
    if (!path || path->empty()) return false;

    std::vector<ContinuousPhasePathState> states;
    if (!path->sample(std::max(0.01, closed_phase_c2_sample_step_w_),
                      global_w, states) || states.size() < 2) {
        return false;
    }

    const int rows = static_cast<int>(states.size());
    traj.resize(rows, 3);
    vel.resize(rows, 3);
    time.resize(rows);
    time(0) = 0.0;
    for (int i = 0; i < rows; ++i) {
        traj.row(i) = states[i].p.transpose();
        Eigen::Vector3d sample_vel = states[i].vel;
        if (!sample_vel.allFinite() || sample_vel.norm() <= 0.1) {
            if (states[i].dp_dw.norm() > 1e-9) {
                sample_vel = states[i].dp_dw.normalized() *
                             std::max(0.1, cmd_tangent_vel_max_);
            } else {
                sample_vel.setZero();
            }
        }
        vel.row(i) = sample_vel.transpose();
        if (i > 0) {
            const double ds = (states[i].p - states[i - 1].p).norm();
            const double speed = std::max(
                0.1, 0.5 * (sample_vel.norm() + vel.row(i - 1).norm()));
            time(i) = time(i - 1) + ds / speed;
        }
    }
    return true;
}

bool gvf_manager::buildGlobalPhaseSamples(
    const Eigen::MatrixXd& traj,
    int anchor_idx,
    double anchor_w,
    double end_w,
    std::vector<double>& global_w) const
{
    const int rows = static_cast<int>(traj.rows());
    if (rows < 2 || !std::isfinite(anchor_w) || !std::isfinite(end_w)) return false;

    anchor_idx = std::max(0, std::min(anchor_idx, rows - 2));
    if (end_w <= anchor_w + 1e-6) return false;

    std::vector<double> cumulative(rows, 0.0);
    for (int i = 1; i < rows; ++i) {
        const double ds = (traj.row(i) - traj.row(i - 1)).norm();
        cumulative[i] = cumulative[i - 1] + std::max(1e-6, ds);
    }

    const double future_length = cumulative.back() - cumulative[anchor_idx];
    if (future_length <= 1e-9) return false;
    const double scale = (end_w - anchor_w) / future_length;

    global_w.resize(rows);
    for (int i = 0; i < rows; ++i) {
        global_w[i] = anchor_w + (cumulative[i] - cumulative[anchor_idx]) * scale;
    }
    return true;
}

bool gvf_manager::evaluateC2QuinticHermite(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& dp0,
    const Eigen::Vector3d& d2p0,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& dp1,
    const Eigen::Vector3d& d2p1,
    double interval_w,
    double u,
    Eigen::Vector3d& p,
    Eigen::Vector3d& dp_dw,
    Eigen::Vector3d& d2p_dw2)
{
    if (!std::isfinite(interval_w) || interval_w <= 1e-6 ||
        !std::isfinite(u)) {
        return false;
    }

    const double s = std::max(0.0, std::min(1.0, u));
    const double h = interval_w;
    const Eigen::Vector3d a0 = p0;
    const Eigen::Vector3d a1 = h * dp0;
    const Eigen::Vector3d a2 = 0.5 * h * h * d2p0;
    const Eigen::Vector3d r0 = p1 - a0 - a1 - a2;
    const Eigen::Vector3d r1 = h * dp1 - a1 - 2.0 * a2;
    const Eigen::Vector3d r2 = h * h * d2p1 - 2.0 * a2;
    const Eigen::Vector3d a3 = 10.0 * r0 - 4.0 * r1 + 0.5 * r2;
    const Eigen::Vector3d a4 = -15.0 * r0 + 7.0 * r1 - r2;
    const Eigen::Vector3d a5 = 6.0 * r0 - 3.0 * r1 + 0.5 * r2;

    const double s2 = s * s;
    const double s3 = s2 * s;
    const double s4 = s3 * s;
    const double s5 = s4 * s;
    p = a0 + a1 * s + a2 * s2 + a3 * s3 + a4 * s4 + a5 * s5;
    dp_dw = (a1 + 2.0 * a2 * s + 3.0 * a3 * s2 +
             4.0 * a4 * s3 + 5.0 * a5 * s4) / h;
    d2p_dw2 = (2.0 * a2 + 6.0 * a3 * s + 12.0 * a4 * s2 +
                20.0 * a5 * s3) / (h * h);
    return p.allFinite() && dp_dw.allFinite() && d2p_dw2.allFinite();
}

bool gvf_manager::evaluateSampledPhasePathState(
    const Eigen::MatrixXd& traj,
    const Eigen::MatrixXd& vel,
    const std::vector<double>& global_w,
    double query_w,
    PhasePathState& state) const
{
    state = PhasePathState();
    const int rows = static_cast<int>(traj.rows());
    if (rows < 3 || traj.cols() < 3 ||
        global_w.size() != static_cast<size_t>(rows) ||
        !std::isfinite(query_w)) {
        return false;
    }
    for (int i = 0; i < rows; ++i) {
        if (!std::isfinite(global_w[i]) ||
            (i > 0 && global_w[i] <= global_w[i - 1])) {
            return false;
        }
    }

    auto pointAt = [&](int i) {
        return Eigen::Vector3d(traj(i, 0), traj(i, 1), traj(i, 2));
    };
    auto firstDerivativeAt = [&](int i) {
        Eigen::Vector3d derivative = Eigen::Vector3d::Zero();
        if (i <= 0) {
            const double dw = std::max(1e-9, global_w[1] - global_w[0]);
            derivative = (pointAt(1) - pointAt(0)) / dw;
        } else if (i >= rows - 1) {
            const double dw = std::max(1e-9, global_w[rows - 1] - global_w[rows - 2]);
            derivative = (pointAt(rows - 1) - pointAt(rows - 2)) / dw;
        } else {
            const double dw = std::max(1e-9, global_w[i + 1] - global_w[i - 1]);
            derivative = (pointAt(i + 1) - pointAt(i - 1)) / dw;
        }
        return derivative;
    };
    auto secondDerivativeAt = [&](int i) {
        Eigen::Vector3d derivative = Eigen::Vector3d::Zero();
        if (i <= 0) {
            const double dw = std::max(1e-9, global_w[1] - global_w[0]);
            derivative = (firstDerivativeAt(1) - firstDerivativeAt(0)) / dw;
        } else if (i >= rows - 1) {
            const double dw = std::max(1e-9, global_w[rows - 1] - global_w[rows - 2]);
            derivative = (firstDerivativeAt(rows - 1) - firstDerivativeAt(rows - 2)) / dw;
        } else {
            const double dw = std::max(1e-9, global_w[i + 1] - global_w[i - 1]);
            derivative = (firstDerivativeAt(i + 1) - firstDerivativeAt(i - 1)) / dw;
        }
        return derivative;
    };

    int i0 = 0;
    int i1 = 0;
    double ratio = 0.0;
    if (query_w <= global_w.front()) {
        i0 = i1 = 0;
    } else if (query_w >= global_w.back()) {
        i0 = i1 = rows - 1;
    } else {
        auto it = std::lower_bound(global_w.begin(), global_w.end(), query_w);
        i1 = static_cast<int>(std::distance(global_w.begin(), it));
        i0 = i1 - 1;
        ratio = (query_w - global_w[i0]) /
            std::max(1e-9, global_w[i1] - global_w[i0]);
    }

    state.p = (1.0 - ratio) * pointAt(i0) + ratio * pointAt(i1);
    state.dp_dw = (1.0 - ratio) * firstDerivativeAt(i0) +
                  ratio * firstDerivativeAt(i1);
    state.d2p_dw2 = (1.0 - ratio) * secondDerivativeAt(i0) +
                    ratio * secondDerivativeAt(i1);
    if (vel.rows() == rows && vel.cols() >= 3) {
        const Eigen::Vector3d v0(vel(i0, 0), vel(i0, 1), vel(i0, 2));
        const Eigen::Vector3d v1(vel(i1, 0), vel(i1, 1), vel(i1, 2));
        state.vel = (1.0 - ratio) * v0 + ratio * v1;
    }
    state.valid = state.p.allFinite() && state.dp_dw.allFinite() &&
                  state.d2p_dw2.allFinite() && state.dp_dw.norm() > 1e-6;
    return state.valid;
}

bool gvf_manager::buildPhaseC2Frontend(
    gvfManager& pm,
    double phase_at_switch,
    double future_switch_w,
    const std::shared_ptr<const ContinuousPhasePath>& old_path_owner,
    double path_end_w,
    int candidate_anchor_idx,
    const UniformBspline& candidate_spline,
    const Eigen::MatrixXd& candidate_traj,
    const Eigen::MatrixXd& candidate_vel,
    const Eigen::VectorXd& candidate_time,
    const std::vector<double>& candidate_w,
    Eigen::MatrixXd& stitched_traj,
    Eigen::MatrixXd& stitched_vel,
    Eigen::VectorXd& stitched_time,
    std::vector<double>& stitched_w,
    std::shared_ptr<const ContinuousPhasePath>& continuous_path) const
{
    continuous_path.reset();
    const bool c2_enabled = closedPhaseV2Active()
        ? closed_phase_c2_enabled_
        : (pointPhaseV2Active() ? point_phase_c2_enabled_ : false);
    // The caller selects the seam contract explicitly: H2 passes a future
    // phase, while neutral/legacy C2 passes the current phase.  Do not reread
    // mutable Runtime authority during expensive connector construction.
    const bool h2_timer_handoff =
        future_switch_w > phase_at_switch + 1e-6;
    const char* mode_name = closedPhaseV2Active() ? "closed" : "point";
    if (!c2_enabled || !pm.gvf_ ||
        candidate_traj.rows() < 3 ||
        candidate_vel.rows() != candidate_traj.rows() ||
        candidate_time.size() != candidate_traj.rows() ||
        !candidate_time.allFinite() ||
        candidate_w.size() != static_cast<size_t>(candidate_traj.rows()) ||
        !std::isfinite(phase_at_switch) || !std::isfinite(future_switch_w) ||
        !std::isfinite(path_end_w) ||
        // H2's timer-backed path is necessarily a future seam.  The legacy
        // non-timer frontend retains its established current-seam C2 path.
        (h2_timer_handoff
            ? future_switch_w <= phase_at_switch + 1e-6
            : future_switch_w < phase_at_switch - 1e-6) ||
        path_end_w <= future_switch_w + 1e-3) {
        return false;
    }

    const std::shared_ptr<const ContinuousPhasePath> old_path = old_path_owner;
    ContinuousPhasePathState old_state;
    if (!old_path || old_path->empty() ||
        future_switch_w < old_path->startW() ||
        future_switch_w > old_path->endW() ||
        !old_path->evaluate(future_switch_w, old_state, false)) {
        return false;
    }

    candidate_anchor_idx = std::max(
        0, std::min(candidate_anchor_idx, static_cast<int>(candidate_time.size()) - 2));
    const double raw_t_anchor = candidate_time(candidate_anchor_idx);
    const double raw_t_end = candidate_spline.t_range(1);
    if (!std::isfinite(raw_t_anchor) || !std::isfinite(raw_t_end) ||
        raw_t_anchor < candidate_spline.t_range(0) - 1e-8 ||
        raw_t_anchor >= raw_t_end - 1e-8) {
        return false;
    }
    const double endpoint_margin = std::min(
        point_phase_endpoint_margin_w_, 0.25 * (path_end_w - future_switch_w));
    double spline_t_anchor = 0.0;
    double spline_t_end = 0.0;
    double raw_length = 0.0;
    if (!ContinuousPhasePath::trimBsplineTimeDomainByArcLength(
            candidate_spline, raw_t_anchor, raw_t_end, 0.0, endpoint_margin,
            spline_t_anchor, spline_t_end, raw_length)) {
        return false;
    }
    const double semantic_path_end_w = path_end_w - endpoint_margin;
    if (semantic_path_end_w <= future_switch_w + 1e-3) return false;
    // The w anchor of the candidate spline must follow the spline's own
    // geometric start (the planner anchor, whose old-path phase is
    // phase_at_switch), not the future seam.  Anchoring it at the seam lets the
    // spline's w origin run ahead of where its geometry actually starts, so the
    // C2 connector has to bridge backwards along the seam tangent; with forward
    // boundary tangents at both ends that forces a loop inside the span and
    // collapses |dp/dw| locally (measured down to 0.28), which makes every
    // w-unit rate window unsatisfiable.  Keeping the anchor aligned keeps the
    // connector's endpoints ordered forwards.
    const double mapped_start_w = std::min(future_switch_w, phase_at_switch);
    const auto mapped_bspline = ContinuousPhasePath::makeMappedBspline(
        candidate_spline, spline_t_anchor, spline_t_end,
        mapped_start_w, semantic_path_end_w);
    if (!mapped_bspline) return false;

    bool have_best = false;
    double best_cost = std::numeric_limits<double>::infinity();
    double best_join_delta_w = 0.0;
    Eigen::MatrixXd best_traj;
    Eigen::MatrixXd best_vel;
    Eigen::VectorXd best_time;
    std::vector<double> best_w;
    std::shared_ptr<const ContinuousPhasePath> best_path;

    const double join_min = std::max(0.05, closed_phase_c2_join_min_w_);
    const double join_max = std::max(join_min, closed_phase_c2_join_max_w_);
    const double join_step = std::max(0.05, closed_phase_c2_join_step_w_);
    const double sample_step = std::max(0.01, closed_phase_c2_sample_step_w_);

    // A connector whose w span is wider than its own arc length is squeezed:
    // |dp/dw| collapses locally (measured down to 0.28), and every rate window
    // expressed in w units becomes unsatisfiable there, which stalls the command
    // and freezes the phase inside the connector.  Measured behaviour: the ratio
    // (connector arc length) / (w span) converges towards 1 from both sides as
    // the span grows, so when the tuned bracket contains no unsqueezed span the
    // consistent one lies above it rather than below.
    // Allowed residual squeeze.  The PhaseOffset rate window must still contain
    // the base guidance rate on the connector, so this is traded against the
    // forward gain: with k1 = 1.2 the nominal need is 1.2*(1+sigma)/ratio, which
    // stays inside the 2.12 window while the ratio stays above ~0.6.
    const double kJoinSqueezeTolerance = 0.10;
    const double kJoinSpanCeiling = 2.5;
    const auto join_span_unsqueezed =
        [kJoinSqueezeTolerance](
            const ContinuousPhasePath::Evaluator& connector, const double w0,
            const double h, double* measured_length) {
            if (measured_length != nullptr) *measured_length = 0.0;
            const int steps = 32;
            double length = 0.0;
            ContinuousPhasePathState previous;
            ContinuousPhasePathState cursor;
            if (h <= 0.0 || !connector(w0, previous)) return false;
            for (int i = 1; i <= steps; ++i) {
                const double w =
                    w0 + h * (static_cast<double>(i) / static_cast<double>(steps));
                if (!connector(w, cursor)) return false;
                length += (cursor.p - previous.p).norm();
                previous = cursor;
            }
            if (measured_length != nullptr) *measured_length = length;
            return std::isfinite(length) &&
                length / h >= 1.0 - kJoinSqueezeTolerance;
        };

    double best_join_ratio = 0.0;
    for (int pass = 0;
         pass < 2 && !have_best;
         ++pass) {
        const double pass_low = pass == 0 ? join_min : join_max + join_step;
        const double pass_high =
            pass == 0 ? join_max : std::max(pass_low, kJoinSpanCeiling);
        if (pass == 1 && !(pass_high >= pass_low - 1e-9)) break;
        if (pass == 1) {
            ROS_WARN_THROTTLE(
                1.0,
                "[GVF][PHASE_V2][C2] mode=%s no unsqueezed join span in [%.3f,%.3f]; extending up to %.3f",
                mode_name, join_min, join_max, pass_high);
        }
    for (double join_delta_w = pass_low;
         join_delta_w <= pass_high + 1e-9;
         join_delta_w += join_step) {
        const double join_w = future_switch_w + join_delta_w;
        if (join_w >= semantic_path_end_w - 1e-4) continue;

        ContinuousPhasePathState new_state;
        if (!mapped_bspline(join_w, new_state)) {
            continue;
        }

        const auto connector = ContinuousPhasePath::makeQuinticHermite(
            future_switch_w, join_w, old_state, new_state);
        if (!connector) continue;
        double connector_own_length = 0.0;
        if (!join_span_unsqueezed(connector, future_switch_w, join_delta_w,
                                  &connector_own_length)) {
            continue;
        }
        ContinuousPhasePathState connector_start;
        ContinuousPhasePathState connector_end;
        if (!connector(future_switch_w, connector_start) ||
            !connector(join_w, connector_end)) {
            continue;
        }
        const double boundary_error =
            (connector_start.p - old_state.p).norm() +
            (connector_start.dp_dw - old_state.dp_dw).norm() +
            (connector_start.d2p_dw2 - old_state.d2p_dw2).norm() +
            (connector_end.p - new_state.p).norm() +
            (connector_end.dp_dw - new_state.dp_dw).norm() +
            (connector_end.d2p_dw2 - new_state.d2p_dw2).norm();
        if (!std::isfinite(boundary_error) || boundary_error > 1e-6) continue;

        auto trial_path = std::make_shared<ContinuousPhasePath>();
        const double prefix_start = std::max(
            old_path->startW(), h2_timer_handoff
                ? phase_at_switch
                : phase_at_switch - closed_phase_back_margin_w_);
        if (future_switch_w > prefix_start + 1e-6 &&
            !trial_path->appendSlice(*old_path, prefix_start, future_switch_w)) {
            continue;
        }
        if (!trial_path->appendSegment(
                future_switch_w, join_w, "c2_quintic", connector) ||
            !trial_path->appendSegment(
                join_w, semantic_path_end_w, "mapped_bspline", mapped_bspline)) {
            continue;
        }

        std::vector<double> trial_sample_w;
        std::vector<ContinuousPhasePathState> trial_states;
        if (!trial_path->sample(sample_step, trial_sample_w, trial_states) ||
            trial_states.size() < 3) {
            continue;
        }

        bool connector_valid = true;
        double smoothness_cost = 0.0;
        double connector_length = 0.0;
        Eigen::Vector3d previous_connector_p = old_state.p;

        for (size_t i = 0; i < trial_states.size(); ++i) {
            const auto& state = trial_states[i];
            const bool terminal_sample = i + 1 == trial_states.size();
            if (!state.valid || (!terminal_sample && state.dp_dw.norm() <= 1e-6)) {
                connector_valid = false;
                break;
            }
            if (i > 0 && trial_states[i - 1].dp_dw.norm() > 1e-6 &&
                state.dp_dw.norm() > 1e-6) {
                const double tangent_dot = trial_states[i - 1].dp_dw.normalized().dot(
                    state.dp_dw.normalized());
                if (tangent_dot < -0.2) {
                    connector_valid = false;
                    break;
                }
            }
            if (trial_sample_w[i] > future_switch_w + 1e-6 &&
                pm.sdf_map_ && pm.sdf_map_->isInMap(state.p) &&
                pm.sdf_map_->getInflateOccupancy(state.p) != 0) {
                connector_valid = false;
                break;
            }
            if (trial_sample_w[i] >= future_switch_w - 1e-9 &&
                trial_sample_w[i] <= join_w + 1e-9) {
                if (trial_sample_w[i] > future_switch_w + 1e-9) {
                    connector_length += (state.p - previous_connector_p).norm();
                }
                previous_connector_p = state.p;
                smoothness_cost += state.d2p_dw2.squaredNorm() * sample_step;
            }
        }
        if (!connector_valid) continue;

        Eigen::MatrixXd trial_traj;
        Eigen::MatrixXd trial_vel;
        Eigen::VectorXd trial_time;
        std::vector<double> trial_w;
        const std::shared_ptr<const ContinuousPhasePath> immutable_trial = trial_path;
        if (!sampleContinuousPhasePath(
                immutable_trial, trial_traj, trial_vel, trial_time, trial_w)) {
            continue;
        }

        const double chord_length = (new_state.p - old_state.p).norm();
        const double excess_length = std::max(0.0, connector_length - chord_length);
        const double cost = smoothness_cost + 0.2 * excess_length * excess_length;
        if (!have_best || cost < best_cost) {
            have_best = true;
            best_cost = cost;
            best_join_delta_w = join_delta_w;
            best_join_ratio = connector_own_length / join_delta_w;
            best_traj = trial_traj;
            best_vel = trial_vel;
            best_time = trial_time;
            best_w = trial_w;
            best_path = immutable_trial;
        }
    }
    }

    if (!have_best) {
        ROS_WARN("[GVF][PHASE_V2][C2] mode=%s connector_failed phase_w=%.3f join_range=[%.3f,%.3f]",
                 mode_name, future_switch_w, join_min, join_max);
        return false;
    }

    stitched_traj = best_traj;
    stitched_vel = best_vel;
    stitched_time = best_time;
    stitched_w = best_w;
    continuous_path = best_path;
    ROS_WARN("[GVF][PHASE_V2][C2] mode=%s connector_success phase_w=%.3f join_delta_w=%.3f cost=%.6f points=%d exact_path=1",
             mode_name, future_switch_w, best_join_delta_w, best_cost,
             static_cast<int>(stitched_traj.rows()));
    ROS_WARN("[GVF][PHASE_V2][C2] mode=%s join_span_ratio=%.4f",
             mode_name, best_join_ratio);
    return true;
}


bool gvf_manager::stageSectionPathHandoff(
    const AuthoritativePhaseSnapshot& phase,
    gvfManager& pm,
    const std::shared_ptr<const ContinuousPhasePath>& candidate_path,
    const Eigen::MatrixXd& candidate_traj,
    const Eigen::MatrixXd& candidate_vel,
    const Eigen::VectorXd& candidate_time,
    const std::vector<double>& candidate_w,
    const std::shared_ptr<const ContinuousPhasePath>& source_path,
    const double copied_prefix_start_w,
    const double copied_prefix_end_w) {
    if (!phase.initialized || !pm.gvf_ || !candidate_path ||
        candidate_path->empty() || candidate_traj.rows() < 2 ||
        candidate_traj.cols() != 3 || candidate_vel.rows() != candidate_traj.rows() ||
        candidate_vel.cols() != 3 || candidate_time.size() != candidate_traj.rows() ||
        candidate_w.size() != static_cast<std::size_t>(candidate_traj.rows()) ||
        !candidate_traj.allFinite() || !candidate_vel.allFinite() ||
        !candidate_time.allFinite()) {
        return false;
    }
    if (source_path && source_path->empty()) return false;
    const SectionPathBundlePtr committed_section_bundle =
        phase_offset_matched_adapter_
            ? phase_offset_matched_adapter_->captureSectionBundle()
            : SectionPathBundlePtr();
    if (source_path &&
        (!pm.gvf_ || pm.gvf_->getContinuousPhasePath() != source_path ||
         !committed_section_bundle || !committed_section_bundle->path ||
         committed_section_bundle->path != source_path)) {
        // A copied-prefix candidate may only replace the bundle that is
        // currently committed and the planner path from which that prefix was
        // actually copied.  This is a planning-time check; publication
        // repeats the source/prefix validation under the serialized locks.
        return false;
    }
    const bool prefix_start_finite = std::isfinite(copied_prefix_start_w);
    const bool prefix_end_finite = std::isfinite(copied_prefix_end_w);
    if (prefix_start_finite != prefix_end_finite ||
        (prefix_start_finite &&
         (!(copied_prefix_end_w > copied_prefix_start_w) ||
          !source_path || copied_prefix_start_w < source_path->startW() ||
          copied_prefix_end_w > source_path->endW()))) {
        return false;
    }
    {
        std::lock_guard<std::mutex> handoff_lock(path_reference_handoff_mutex_);
        if (pending_section_path_handoff_) {
            std::lock_guard<std::mutex> phase_lock(
                authoritative_phase_mutex_);
            const bool source_exhausted =
                source_path && !source_path->empty() &&
                std::isfinite(phase_w_) &&
                phase_w_ >=
                    source_path->endW() - kExhaustedFrontendSeamSlackW;
            const bool candidate_not_committed =
                !committed_section_bundle ||
                committed_section_bundle->path !=
                    pending_section_path_handoff_->candidate_path;
            if (!(source_exhausted && candidate_not_committed)) {
                return false;
            }
            pending_section_path_handoff_.reset();
        }
    }

    std::shared_ptr<const ContinuousPhasePath> assigned_path;
    if (!assignPathIdentity(candidate_path, assigned_path) ||
        !assigned_path || assigned_path->empty()) {
        return false;
    }
    if (!buildAndStageSectionBundle(
            pm, phase, assigned_path, source_path,
            copied_prefix_start_w, copied_prefix_end_w)) {
        return false;
    }
    const SectionBundleCandidateCapture staged_candidate =
        phase_offset_matched_adapter_->capturePendingSectionCandidate();
    const bool requested_prefix_start_finite =
        std::isfinite(copied_prefix_start_w);
    const bool requested_prefix_end_finite =
        std::isfinite(copied_prefix_end_w);
    const bool staged_prefix_start_finite =
        std::isfinite(staged_candidate.copied_prefix_start_w);
    const bool staged_prefix_end_finite =
        std::isfinite(staged_candidate.copied_prefix_end_w);
    if (!staged_candidate.bundle || staged_candidate.bundle->path != assigned_path ||
        staged_candidate.source_path != source_path ||
        requested_prefix_start_finite != staged_prefix_start_finite ||
        requested_prefix_end_finite != staged_prefix_end_finite ||
        (requested_prefix_start_finite &&
         staged_candidate.copied_prefix_start_w != copied_prefix_start_w) ||
        (requested_prefix_end_finite &&
         staged_candidate.copied_prefix_end_w != copied_prefix_end_w)) {
        return false;
    }

    std::shared_ptr<SectionPathHandoff> mutable_handoff(
        new SectionPathHandoff());
    mutable_handoff->task_generation =
        phase_offset_matched_adapter_
            ? phase_offset_matched_adapter_->executionGeneration()
            : 0U;
    mutable_handoff->phase_generation = phase.generation;
    mutable_handoff->source_path = source_path;
    mutable_handoff->candidate_path = assigned_path;
    mutable_handoff->bundle = staged_candidate.bundle;
    mutable_handoff->copied_prefix_start_w = copied_prefix_start_w;
    mutable_handoff->copied_prefix_end_w = copied_prefix_end_w;
    mutable_handoff->traj = candidate_traj;
    mutable_handoff->vel = candidate_vel;
    mutable_handoff->time = candidate_time;
    mutable_handoff->w = candidate_w;
    mutable_handoff->path_msg.header.frame_id = "world";
    mutable_handoff->path_msg.header.stamp = ros::Time::now();
    mutable_handoff->path_msg.poses.reserve(candidate_w.size());
    for (int index = 0; index < candidate_traj.rows(); ++index) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = candidate_traj(index, 0);
        pose.pose.position.y = candidate_traj(index, 1);
        pose.pose.position.z = candidate_traj(index, 2);
        pose.pose.orientation.x = candidate_vel(index, 0);
        pose.pose.orientation.y = candidate_vel(index, 1);
        pose.pose.orientation.z = candidate_vel(index, 2);
        pose.pose.orientation.w = 1.0;
        mutable_handoff->path_msg.poses.push_back(pose);
    }
    const std::shared_ptr<const SectionPathHandoff> handoff =
        std::shared_ptr<const SectionPathHandoff>(std::move(mutable_handoff));
    if (!handoff->complete()) return false;
    SectionPathBundlePtr retired_unregistered_bundle;
    bool registered = false;
    {
        // Recheck task/source/prefix/staged ownership in the same lock order
        // used by command publication.  Expensive planner/profile work above
        // may have raced a command commit, reset, or phase advance.
        std::lock_guard<std::mutex> frontend_lock(frontend_apply_mutex_);
        std::lock_guard<std::mutex> handoff_lock(path_reference_handoff_mutex_);
        std::lock_guard<std::mutex> phase_lock(authoritative_phase_mutex_);
        std::lock_guard<std::mutex> task_lock(
            phase_offset_matched_adapter_->task_publication_mutex_);
        std::lock_guard<std::mutex> runtime_lock(
            phase_offset_matched_adapter_->runtime_command_mutex_);
        const bool task_matches =
            phase_offset_matched_adapter_->task_generation_.load(
                std::memory_order_acquire) == handoff->task_generation;
        const bool source_matches =
            !source_path ||
            (pm.gvf_->getContinuousPhasePath() == source_path &&
             phase_offset_matched_adapter_->section_bundle_ &&
             phase_offset_matched_adapter_->section_bundle_->path ==
                 source_path);
        const bool prefix_start_finite =
            std::isfinite(copied_prefix_start_w);
        const bool prefix_end_finite =
            std::isfinite(copied_prefix_end_w);
        const bool prefix_reaches_source_end =
            source_path && !source_path->empty() &&
            prefix_end_finite &&
            copied_prefix_end_w >=
                source_path->endW() - kPathReferenceHandoffTolerance;
        const double phase_upper =
            copied_prefix_end_w +
            (prefix_reaches_source_end ? kExhaustedFrontendSeamSlackW : 0.0);
        const bool latest_phase_matches =
            phase_initialized_ && std::isfinite(phase_w_) &&
            ((prefix_start_finite && prefix_end_finite)
                 ? (phase_w_ >= copied_prefix_start_w &&
                    phase_w_ <= phase_upper)
                 : (!prefix_start_finite && !prefix_end_finite &&
                    authoritative_phase_generation_ == phase.generation &&
                    phase_w_ == phase.w));
        const bool staged_matches =
            phase_offset_matched_adapter_->staged_section_bundle_ ==
            staged_candidate.bundle;
        if (!pending_section_path_handoff_ && task_matches && source_matches &&
            latest_phase_matches && staged_matches &&
            staged_candidate.bundle &&
            staged_candidate.bundle->task_generation == handoff->task_generation &&
            staged_candidate.bundle->path == handoff->candidate_path) {
            pending_section_path_handoff_ = handoff;
            registered = true;
        } else if (staged_matches &&
                   phase_offset_matched_adapter_->section_bundle_ !=
                       staged_candidate.bundle) {
            // No command can have committed this candidate under the same
            // serialized check.  Retire it here so a failed registration does
            // not leave the FSM permanently blocked by an orphan stage.
            retired_unregistered_bundle.swap(
                phase_offset_matched_adapter_->staged_section_bundle_);
            phase_offset_matched_adapter_->staged_section_source_path_.reset();
            phase_offset_matched_adapter_->staged_section_copied_prefix_start_w_ =
                std::numeric_limits<double>::quiet_NaN();
            phase_offset_matched_adapter_->staged_section_copied_prefix_end_w_ =
                std::numeric_limits<double>::quiet_NaN();
        }
    }
    return registered;
}

bool gvf_manager::installPlannerOnlyFrontend(
    gvfManager& pm,
    const Eigen::MatrixXd& traj,
    const Eigen::MatrixXd& vel,
    const Eigen::VectorXd& time,
    const std::vector<double>& w,
    const std::shared_ptr<const ContinuousPhasePath>& path_owner,
    const ros::Time& now,
    const AuthoritativePhaseSnapshot& expected_phase,
    const std::shared_ptr<const ContinuousPhasePath>& copied_prefix_source,
    const double copied_prefix_end_w,
    const std::uint64_t expected_execution_generation,
    nav_msgs::Path& path_msg,
    const bool allow_retained_delta,
    const SectionPathBundlePtr& section_bundle_to_commit)
{
    path_msg = nav_msgs::Path();
    if (!pm.gvf_ || !path_owner || path_owner->empty() ||
        traj.rows() <= 0 || vel.rows() != traj.rows() ||
        time.size() != traj.rows() ||
        w.size() != static_cast<std::size_t>(traj.rows()) ||
        !traj.allFinite() || !vel.allFinite() || !time.allFinite() ||
        !std::isfinite(expected_phase.w)) {
        return false;
    }
    for (std::size_t index = 0U; index < w.size(); ++index) {
        if (!std::isfinite(w[index]) ||
            (index != 0U && !(w[index] > w[index - 1U]))) {
            return false;
        }
    }
    const bool has_copied_prefix = static_cast<bool>(copied_prefix_source);
    const bool copied_prefix_reaches_source_end =
        has_copied_prefix && !copied_prefix_source->empty() &&
        copied_prefix_end_w >=
            copied_prefix_source->endW() - kPathReferenceHandoffTolerance;
    const double copied_prefix_phase_slack =
        copied_prefix_reaches_source_end ? kExhaustedFrontendSeamSlackW : 0.0;
    if (has_copied_prefix) {
        if (!expected_phase.initialized ||
            expected_execution_generation == 0U ||
            // A current-seam handoff copies the last prefix up to the live
            // phase and starts the new path there; equality is the exhausted-
            // frontend case, not an already-consumed prefix.
            !(copied_prefix_end_w >=
              expected_phase.w - copied_prefix_phase_slack) ||
            copied_prefix_source->empty() ||
            expected_phase.w < copied_prefix_source->startW() ||
            copied_prefix_end_w > copied_prefix_source->endW() ||
            expected_phase.w < path_owner->startW() ||
            copied_prefix_end_w > path_owner->endW()) {
            return false;
        }
    } else if (std::isfinite(copied_prefix_end_w) &&
               copied_prefix_end_w != expected_phase.w) {
        return false;
    }

    std::shared_ptr<const ContinuousPhasePath> identified_path;
    if (section_bundle_to_commit) {
        if (!section_bundle_to_commit->path ||
            section_bundle_to_commit->path->empty()) {
            return false;
        }
        identified_path = section_bundle_to_commit->path;
    } else if (!assignPathIdentity(path_owner, identified_path) ||
               !identified_path || identified_path->empty()) {
        return false;
    }
    // Keep retired Section values alive until after all publication locks are
    // released.  They may own a large immutable profile/environment snapshot.
    SectionPathBundlePtr retired_current_bundle;
    SectionPathBundlePtr retired_staged_bundle;
    SectionPathBundlePtr retired_pending_bundle;
    std::shared_ptr<const ContinuousPhasePath> retired_staged_source_path;
    std::shared_ptr<const ContinuousPhasePath> retired_pending_source_path;
    std::shared_ptr<const phase_offset_navigation::RuntimeSectionPreparedStep>
        retired_pending_step;
    phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
        retired_pending_query;
    std::shared_ptr<const SectionPathHandoff> retired_manager_handoff;
    std::lock_guard<std::mutex> apply_lock(frontend_apply_mutex_);
    std::lock_guard<std::mutex> handoff_lock(path_reference_handoff_mutex_);
    std::lock_guard<std::mutex> phase_lock(authoritative_phase_mutex_);
    // Planner-only installation is the neutral path.  Once MANUAL Runtime
    // owns a nonzero offset, a path replacement must carry the Section bundle
    // and copied-prefix evidence through the publish-first handoff; installing
    // a neutral frontend here would silently detach the active reference.
    std::unique_lock<std::mutex> runtime_lock;
    std::unique_lock<std::mutex> task_lock;
    if (phase_offset_matched_adapter_ &&
        matched_config_.mode == PhaseOffsetMatchedMode::MANUAL) {
        task_lock = std::unique_lock<std::mutex>(
            phase_offset_matched_adapter_->task_publication_mutex_);
        runtime_lock = std::unique_lock<std::mutex>(
            phase_offset_matched_adapter_->runtime_command_mutex_);
        if (!phase_offset_matched_adapter_->runtime_ ||
            (!allow_retained_delta &&
             phase_offset_matched_adapter_->runtime_->retainedDelta() != 0.0)) {
            return false;
        }
    }
    const std::uint64_t live_generation = phase_offset_matched_adapter_
        ? phase_offset_matched_adapter_->executionGeneration() : 0U;
    if (phase_initialized_ != expected_phase.initialized ||
        closed_phase_acquired_ != expected_phase.closed_acquired ||
        ((expected_execution_generation != 0U ||
          phase_offset_matched_adapter_ == nullptr) &&
         live_generation != expected_execution_generation)) {
        return false;
    }
    // A neutral install is an exact compare-and-swap against the phase tuple.
    // A copied-prefix install is allowed to observe command progress made
    // while C2 construction was in flight, provided the task/flags remain the
    // same and the phase has only advanced within the copied source prefix.
    if ((!has_copied_prefix &&
         (authoritative_phase_generation_ != expected_phase.generation ||
          phase_w_ != expected_phase.w)) ||
        (has_copied_prefix &&
         (authoritative_phase_generation_ < expected_phase.generation ||
          phase_w_ > copied_prefix_end_w + copied_prefix_phase_slack))) {
        return false;
    }
    if (has_copied_prefix) {
        if (pm.gvf_->getContinuousPhasePath() != copied_prefix_source ||
            phase_w_ > copied_prefix_end_w + copied_prefix_phase_slack) {
            return false;
        }
    }
    if (phase_w_ < identified_path->startW() ||
        phase_w_ > identified_path->endW() ||
        phase_w_ < w.front() || phase_w_ > w.back()) {
        return false;
    }

    const int live_anchor_idx = static_cast<int>(std::distance(
        w.begin(), std::lower_bound(w.begin(), w.end(), phase_w_)));
    pm.last_traj = traj;
    pm.last_vel = vel;
    pm.last_traj_time_ = time;
    pm.is_first_goal = false;
    current_traj_index_ = std::max(
        0, std::min(live_anchor_idx, static_cast<int>(w.size()) - 1));
    last_switch_time_ = now;
    cmd_switch_motion_limit_until_ = now + ros::Duration(
        std::max(0.0, cmd_switch_motion_limit_time_));
    pm.gvf_->setAuthoritativePhaseMode(true);
    pm.gvf_->setContinuousPhasePath(identified_path);
    pm.gvf_->setNextPathWSamples(w);
    resetGovernorState();
    if (!installAuthoritativePathMirrorLocked(traj, vel, w, path_msg)) {
        return false;
    }
    if (phase_offset_matched_adapter_ &&
        matched_config_.mode == PhaseOffsetMatchedMode::MANUAL) {
        // A successful zero-offset neutral replacement supersedes every old
        // Section candidate and manager handoff.  The Runtime itself remains
        // at delta==0; only the immutable bundle owners are retired here.
        if (section_bundle_to_commit) {
            retired_current_bundle.swap(
                phase_offset_matched_adapter_->section_bundle_);
            phase_offset_matched_adapter_->section_bundle_ =
                section_bundle_to_commit;
        } else {
            retired_current_bundle.swap(
                phase_offset_matched_adapter_->section_bundle_);
        }
        retired_staged_bundle.swap(
            phase_offset_matched_adapter_->staged_section_bundle_);
        retired_staged_source_path.swap(
            phase_offset_matched_adapter_->staged_section_source_path_);
        phase_offset_matched_adapter_->staged_section_copied_prefix_start_w_ =
            std::numeric_limits<double>::quiet_NaN();
        phase_offset_matched_adapter_->staged_section_copied_prefix_end_w_ =
            std::numeric_limits<double>::quiet_NaN();
        retired_pending_bundle.swap(
            phase_offset_matched_adapter_->pending_section_bundle_);
        retired_pending_source_path.swap(
            phase_offset_matched_adapter_->pending_section_source_path_);
        retired_pending_step.swap(
            phase_offset_matched_adapter_->pending_section_prepared_step_);
        retired_pending_query.swap(
            phase_offset_matched_adapter_->pending_section_reference_query_);
        phase_offset_matched_adapter_->clearPendingPositionCommandLocked();
        retired_manager_handoff.swap(pending_section_path_handoff_);
    }
    return true;
}

bool gvf_manager::consumeCommittedSectionPathHandoff(
    gvfManager& pm, const ros::Time& now)
{
    if (!phase_offset_matched_adapter_ || !pm.gvf_) {
        return false;
    }

    std::shared_ptr<const SectionPathHandoff> section_handoff;
    {
        std::lock_guard<std::mutex> handoff_lock(
            path_reference_handoff_mutex_);
        section_handoff = pending_section_path_handoff_;
    }
    if (!section_handoff || !section_handoff->bundle ||
        !section_handoff->bundle->environment_change_mutex) {
        return false;
    }

    std::unique_lock<std::recursive_mutex> environment_lock(
        *section_handoff->bundle->environment_change_mutex);
    nav_msgs::Path path_msg;
    {
        std::lock_guard<std::mutex> apply_lock(frontend_apply_mutex_);
        std::lock_guard<std::mutex> handoff_lock(
            path_reference_handoff_mutex_);
        if (pending_section_path_handoff_ != section_handoff ||
            section_handoff->task_generation == 0U ||
            section_handoff->phase_generation == 0U ||
            !section_handoff->candidate_path ||
            section_handoff->candidate_path->empty()) {
            return false;
        }
        const std::shared_ptr<const ContinuousPhasePath> planner_path =
            pm.gvf_->getContinuousPhasePath();
        if ((section_handoff->source_path &&
             planner_path != section_handoff->source_path) ||
            (!section_handoff->source_path &&
             planner_path != section_handoff->candidate_path)) {
            return false;
        }
        std::lock_guard<std::mutex> phase_lock(authoritative_phase_mutex_);
        if (phase_offset_matched_adapter_->executionGeneration() !=
                section_handoff->task_generation) {
            return false;
        }
        const SectionPathBundlePtr current_bundle =
            phase_offset_matched_adapter_->captureSectionBundle();
        if (current_bundle != section_handoff->bundle ||
            !current_bundle->path ||
            current_bundle->path != section_handoff->candidate_path) {
            return false;
        }
        if (!phase_initialized_ || !std::isfinite(phase_w_) ||
            phase_w_ < section_handoff->candidate_path->startW() ||
            phase_w_ > section_handoff->candidate_path->endW()) {
            return false;
        }
        if (!installAuthoritativePathMirrorLocked(
                section_handoff->traj, section_handoff->vel,
                section_handoff->w, path_msg)) {
            return false;
        }
        pm.last_traj = section_handoff->traj;
        pm.last_vel = section_handoff->vel;
        pm.last_traj_time_ = section_handoff->time;
        pm.is_first_goal = false;
        const int anchor_idx = static_cast<int>(std::distance(
            section_handoff->w.begin(),
            std::lower_bound(section_handoff->w.begin(),
                             section_handoff->w.end(), phase_w_)));
        current_traj_index_ = std::max(
            0, std::min(anchor_idx,
                        static_cast<int>(section_handoff->w.size()) - 1));
        last_switch_time_ = now;
        cmd_switch_motion_limit_until_ = now + ros::Duration(
            std::max(0.0, cmd_switch_motion_limit_time_));
        pm.gvf_->setAuthoritativePhaseMode(true);
        pm.gvf_->setContinuousPhasePath(section_handoff->candidate_path);
        pm.gvf_->setNextPathWSamples(section_handoff->w);
        resetGovernorState();
        if (pending_section_path_handoff_ != section_handoff) {
            return false;
        }
        pending_section_path_handoff_.reset();
    }
    path_pub.publish(path_msg);
    return true;
}
bool gvf_manager::installAuthoritativePathMirrorLocked(
    const Eigen::MatrixXd& traj,
    const Eigen::MatrixXd& vel,
    const std::vector<double>& w,
    nav_msgs::Path& path_msg)
{
    if (traj.rows() <= 0 || vel.rows() != traj.rows() ||
        w.size() != static_cast<size_t>(traj.rows())) {
        return false;
    }

    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < traj.rows(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = traj(i, 0);
        pose.pose.position.y = traj(i, 1);
        pose.pose.position.z = traj(i, 2);
        pose.pose.orientation.x = vel(i, 0);
        pose.pose.orientation.y = vel(i, 1);
        pose.pose.orientation.z = vel(i, 2);
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    // This is invoked only by the H2 FSM after it has serialized frontend
    // application.  Install first; subsequent ROS delivery is topic input
    // only and is rejected by gvf::pathCallback while authoritative.
    if (!swarmParticlesManager.empty() && swarmParticlesManager[0].gvf_) {
        swarmParticlesManager[0].gvf_->installAuthoritativePathMirror(
            path_msg, w);
    }
    return true;
}

bool gvf_manager::installInitialClosedPhaseFrontend(
    gvfManager& pm,
    const Eigen::Vector3d& current_pos,
    const ros::Time& current_time)
{
    if (!closedPhaseV2Active() || !pm.gvf_) return false;

    // Capture reset-retirement evidence before path sampling and C2 work.  A
    // later reset changes this session, so stale planner work can only discard.
    const AuthoritativePhaseSnapshot bootstrap_phase =
        captureAuthoritativePhase();
    const double initial_phase_w = findInitialClosedPhaseV2(current_pos);
    // Keep the phase local until the complete planner frontend is ready so no
    // callback observes a partially initialized authoritative phase tuple.

    Eigen::MatrixXd traj;
    Eigen::MatrixXd vel;
    Eigen::VectorXd time;
    std::vector<double> global_w;
    const double frontend_length = std::max(
        1.0,
        std::max(closed_goal_prefer_lookahead_w_,
                 cmd_governor_l_max_ + std::max(0.0, switch_governor_path_margin_w_)));
    std::shared_ptr<const ContinuousPhasePath> continuous_path;
    if (!buildNominalContinuousPhasePath(
            initial_phase_w - closed_phase_back_margin_w_,
            initial_phase_w + frontend_length,
            continuous_path)) {
        return false;
    }
    std::shared_ptr<const ContinuousPhasePath> identified_path;
    if (!assignPathIdentity(continuous_path, identified_path)) return false;
    continuous_path = identified_path;
    if (!sampleContinuousPhasePath(
            continuous_path, traj, vel, time, global_w)) {
        return false;
    }

    ContinuousPhasePathState phase_state;
    ContinuousPhasePathState end_state;
    if (!continuous_path->evaluate(initial_phase_w, phase_state, false) ||
        !continuous_path->evaluate(continuous_path->endW(), end_state, false)) {
        return false;
    }

    int bootstrap_anchor_idx = static_cast<int>(std::distance(
        global_w.begin(), std::lower_bound(global_w.begin(), global_w.end(),
                                           initial_phase_w)));
    bootstrap_anchor_idx = std::max(
        0, std::min(bootstrap_anchor_idx,
                    static_cast<int>(global_w.size()) - 1));
    const bool closed_phase_acquired =
        (current_pos - phase_state.p).norm() <= closed_phase_acquire_distance_;
    bool bootstrap_applied = false;
    nav_msgs::Path bootstrap_path_msg;
    {
        std::lock_guard<std::mutex> apply_lock(frontend_apply_mutex_);
        std::lock_guard<std::mutex> handoff_lock(path_reference_handoff_mutex_);
        if (!pending_section_path_handoff_) {
            std::lock_guard<std::mutex> phase_lock(authoritative_phase_mutex_);
            if (authoritative_phase_generation_ == bootstrap_phase.generation &&
                phase_w_ == bootstrap_phase.w &&
                phase_initialized_ == bootstrap_phase.initialized &&
                closed_phase_acquired_ == bootstrap_phase.closed_acquired) {
                pm.last_traj = traj;
                pm.last_vel = vel;
                pm.last_traj_time_ = time;
                pm.is_first_goal = true;
                pm.goal_pt = end_state.p;
                current_traj_index_ = bootstrap_anchor_idx;
                last_switch_time_ = current_time;
                cmd_switch_motion_limit_until_ = current_time + ros::Duration(
                    std::max(0.0, cmd_switch_motion_limit_time_));
                pm.gvf_->setAuthoritativePhaseMode(true);
                pm.gvf_->setContinuousPhasePath(continuous_path);
                pm.gvf_->setNextPathWSamples(global_w);
                resetGovernorState();
                if (!installAuthoritativePathMirrorLocked(
                        pm.last_traj, pm.last_vel, global_w,
                        bootstrap_path_msg)) {
                    return false;
                }
                closed_phase_pending_path_end_w_ = global_w.back();
                closed_phase_has_pending_path_end_w_ = true;
                publishAuthoritativePhaseLocked(
                    initial_phase_w, true, closed_phase_acquired);
                progress_w_ = initial_phase_w;
                progress_initialized_ = true;
                pm.gvf_->setVisualizationProgressW(initial_phase_w);
                bootstrap_applied = true;
            }
        }
    }
    if (!bootstrap_applied) return false;
    if (phase_offset_matched_adapter_ &&
        matched_config_.mode == PhaseOffsetMatchedMode::MANUAL &&
        phase_offset_matched_adapter_->configurationValid()) {
        buildAndStageSectionBundle(
            pm, captureAuthoritativePhase(), continuous_path);
        last_section_build_time_ = current_time;
    }
    path_pub.publish(bootstrap_path_msg);
    ROS_WARN("[GVF][CLOSED_PHASE_V2][INIT] phase_w=%.3f phase_mod=%.3f frontend_start_w=%.3f frontend_end_w=%.3f points=%d acquired=%d distance=%.3f exact_path=1",
             initial_phase_w, wrapClosedW(initial_phase_w), global_w.front(),
             global_w.back(), static_cast<int>(traj.rows()),
             closed_phase_acquired ? 1 : 0,
             (current_pos - phase_state.p).norm());
    return true;

}

bool gvf_manager::installInitialPointPhaseFrontend(
    gvfManager& pm,
    const Eigen::Vector3d& current_pos,
    const ros::Time& current_time,
    const Eigen::MatrixXd& candidate_traj,
    const Eigen::MatrixXd& candidate_vel,
    const Eigen::VectorXd& candidate_time,
    int candidate_anchor_idx,
    const UniformBspline& candidate_spline)
{
    if (!pointPhaseV2Active() || !pm.gvf_ || candidate_traj.rows() < 2 ||
        candidate_vel.rows() != candidate_traj.rows()) {
        ROS_WARN_THROTTLE(1.0,
            "[GVF][POINT_PHASE_V2][MAPPED_CONTINUOUS_PATH] invalid candidate inputs");
        return false;
    }

    // See the closed bootstrap: capture before the expensive frontend build,
    // never after it.  Session retirement is the reset/replan invalidation
    // edge for this otherwise lock-free preparation.
    const AuthoritativePhaseSnapshot bootstrap_phase =
        captureAuthoritativePhase();
    candidate_anchor_idx = std::max(
        0, std::min(candidate_anchor_idx, static_cast<int>(candidate_traj.rows()) - 2));
    const double raw_t_anchor = candidate_time(candidate_anchor_idx);
    double remaining_length = 0.0;
    if (candidate_time.size() != candidate_traj.rows() ||
        !candidate_time.allFinite() ||
        !std::isfinite(raw_t_anchor) ||
        raw_t_anchor < candidate_spline.t_range(0) - 1e-8 ||
        raw_t_anchor >= candidate_spline.t_range(1) - 1e-8 ||
        !ContinuousPhasePath::measureBsplineArcLength(
            candidate_spline, raw_t_anchor, candidate_spline.t_range(1),
            remaining_length) || remaining_length <= 1e-3) {
        ROS_WARN_THROTTLE(1.0,
            "[GVF][POINT_PHASE_V2][MAPPED_CONTINUOUS_PATH] invalid candidate arc-length domain");
        return false;
    }

    const double path_start_w = 0.0;
    const double path_end_w = path_start_w + remaining_length;
    Eigen::MatrixXd install_traj;
    Eigen::MatrixXd install_vel;
    Eigen::VectorXd install_time;
    std::vector<double> install_w;
    std::shared_ptr<const ContinuousPhasePath> continuous_path;
    if (!buildMappedPhaseFrontend(
            path_start_w, path_end_w, true, candidate_anchor_idx, candidate_spline,
            candidate_traj, candidate_time, install_traj, install_vel,
            install_time, install_w, continuous_path)) {
        ROS_WARN_THROTTLE(1.0,
            "[GVF][POINT_PHASE_V2][MAPPED_CONTINUOUS_PATH] frontend mapping failed");
        return false;
    }
    std::shared_ptr<const ContinuousPhasePath> identified_path;
    if (!assignPathIdentity(continuous_path, identified_path)) return false;
    continuous_path = identified_path;

    const double initial_phase_w = continuous_path->startW();

    int bootstrap_anchor_idx = static_cast<int>(std::distance(
        install_w.begin(),
        std::lower_bound(install_w.begin(), install_w.end(), initial_phase_w)));
    bootstrap_anchor_idx = std::max(
        0, std::min(bootstrap_anchor_idx,
                    static_cast<int>(install_w.size()) - 1));

    bool bootstrap_applied = false;
    nav_msgs::Path bootstrap_path_msg;
    {
        std::lock_guard<std::mutex> apply_lock(frontend_apply_mutex_);
        std::lock_guard<std::mutex> handoff_lock(path_reference_handoff_mutex_);
        if (!pending_section_path_handoff_) {
            std::lock_guard<std::mutex> phase_lock(authoritative_phase_mutex_);
            if (authoritative_phase_generation_ == bootstrap_phase.generation &&
                phase_w_ == bootstrap_phase.w &&
                phase_initialized_ == bootstrap_phase.initialized &&
                closed_phase_acquired_ == bootstrap_phase.closed_acquired &&
                !phase_initialized_) {
                pm.last_traj = install_traj;
                pm.last_vel = install_vel;
                pm.last_traj_time_ = install_time;
                pm.is_first_goal = false;
                current_traj_index_ = bootstrap_anchor_idx;
                last_switch_time_ = current_time;
                cmd_switch_motion_limit_until_ = current_time + ros::Duration(
                    std::max(0.0, cmd_switch_motion_limit_time_));
                pm.gvf_->setAuthoritativePhaseMode(true);
                pm.gvf_->setContinuousPhasePath(continuous_path);
                pm.gvf_->setNextPathWSamples(install_w);
                resetGovernorState();
                if (!installAuthoritativePathMirrorLocked(
                        pm.last_traj, pm.last_vel, install_w,
                        bootstrap_path_msg)) {
                    return false;
                }
                publishAuthoritativePhaseLocked(initial_phase_w, true, true);
                progress_w_ = initial_phase_w;
                progress_initialized_ = true;
                pm.gvf_->setVisualizationProgressW(initial_phase_w);
                bootstrap_applied = true;
            }
        }
    }
    if (!bootstrap_applied) {
        ROS_WARN_THROTTLE(1.0,
            "[GVF][POINT_PHASE_V2][NEUTRAL_FRONTEND_INSTALL] session or frontend CAS rejected");
        return false;
    }
    if (phase_offset_matched_adapter_ &&
        matched_config_.mode == PhaseOffsetMatchedMode::MANUAL &&
        phase_offset_matched_adapter_->configurationValid()) {
        buildAndStageSectionBundle(
            pm, captureAuthoritativePhase(), continuous_path);
        last_section_build_time_ = current_time;
    }
    path_pub.publish(bootstrap_path_msg);

    last_replan_time_ = current_time;
    ROS_WARN("[GVF][POINT_PHASE_V2][INIT] phase_w=%.3f path_start_w=%.3f path_end_w=%.3f points=%d start_error=%.3f exact_path=1",
             initial_phase_w, install_w.front(), install_w.back(),
             static_cast<int>(install_traj.rows()),
             (current_pos - install_traj.row(0).transpose()).norm());
    return true;
}

void gvf_manager::generateCircleReference(const Eigen::Vector3d& center)
{
    const int N = std::max(24, circle_reference_points_);
    const double radius = std::max(0.3, circle_reference_radius_);
    circle_reference_center_ = Eigen::Vector3d(center.x(), center.y(), circle_reference_height_);
    circle_reference_traj_.resize(N, 3);
    circle_reference_vel_.resize(N, 3);

    for (int i = 0; i < N; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(N);
        const double cth = std::cos(theta);
        const double sth = std::sin(theta);
        circle_reference_traj_.row(i) << circle_reference_center_.x() + radius * cth,
                                          circle_reference_center_.y() + radius * sth,
                                          circle_reference_center_.z();
        circle_reference_vel_.row(i) << 0.0,
                                          0.0,
                                          0.0;
    }

    circle_reference_w_.assign(N, 0.0);
    for (int i = 1; i < N; ++i) {
        circle_reference_w_[i] = circle_reference_w_[i - 1]
                               + (circle_reference_traj_.row(i) - circle_reference_traj_.row(i - 1)).norm();
    }
    circle_reference_total_w_ = 0.0;
    for (int i = 0; i < N; ++i) {
        const int j = (i + 1) % N;
        circle_reference_total_w_ += (circle_reference_traj_.row(j) - circle_reference_traj_.row(i)).norm();
    }
    circle_reference_progress_anchor_w_ = progress_w_;
    circle_reference_index_ = 0;
    closed_ref_w_ = 0.0;
    closed_ref_initialized_ = false;
    closed_ref_recover_ = false;
    resetClosedGoalCandidateState();
    circle_reference_ready_ = true;
    publishReferencePathMsg(circle_reference_traj_, circle_reference_vel_, circle_ref_pub_);
}


void gvf_manager::generateFigureEightReference(const Eigen::Vector3d& center)
{
    const int N = std::max(48, circle_reference_points_);
    const double radius = std::max(0.3, figure8_reference_radius_);
    circle_reference_center_ = Eigen::Vector3d(center.x(), center.y(), circle_reference_height_);
    circle_reference_traj_.resize(N, 3);
    circle_reference_vel_.resize(N, 3);

    for (int i = 0; i < N; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(N);
        const double sx = std::sin(theta);
        const double s2 = std::sin(2.0 * theta);

        Eigen::Vector3d pos(circle_reference_center_.x() + 0.5 * radius * s2,
                            circle_reference_center_.y() + radius * sx,
                            circle_reference_center_.z());
        circle_reference_traj_.row(i) = pos.transpose();
        circle_reference_vel_.row(i) << 0.0,
                                          0.0,
                                          0.0;
    }

    circle_reference_w_.assign(N, 0.0);
    for (int i = 1; i < N; ++i) {
        circle_reference_w_[i] = circle_reference_w_[i - 1]
                               + (circle_reference_traj_.row(i) - circle_reference_traj_.row(i - 1)).norm();
    }
    circle_reference_total_w_ = 0.0;
    for (int i = 0; i < N; ++i) {
        const int j = (i + 1) % N;
        circle_reference_total_w_ += (circle_reference_traj_.row(j) - circle_reference_traj_.row(i)).norm();
    }
    circle_reference_progress_anchor_w_ = progress_w_;
    circle_reference_index_ = 0;
    closed_ref_w_ = 0.0;
    closed_ref_initialized_ = false;
    closed_ref_recover_ = false;
    resetClosedGoalCandidateState();
    circle_reference_ready_ = true;
    publishReferencePathMsg(circle_reference_traj_, circle_reference_vel_, circle_ref_pub_);
}

double gvf_manager::wrapClosedW(double w) const
{
    if (circle_reference_total_w_ <= 1e-9) {
        return 0.0;
    }

    double s = std::fmod(w, circle_reference_total_w_);
    if (s < 0.0) {
        s += circle_reference_total_w_;
    }
    return s;
}

int gvf_manager::indexFromClosedW(double w) const
{
    const int N = static_cast<int>(circle_reference_traj_.rows());
    if (N <= 0 || circle_reference_w_.size() != static_cast<size_t>(N) ||
        circle_reference_total_w_ <= 1e-9) {
        return 0;
    }

    const double s = wrapClosedW(w);
    auto it = std::upper_bound(circle_reference_w_.begin(), circle_reference_w_.end(), s);
    if (it == circle_reference_w_.begin()) {
        return 0;
    }
    if (it == circle_reference_w_.end()) {
        return N - 1;
    }
    return static_cast<int>(std::distance(circle_reference_w_.begin(), it)) - 1;
}

Eigen::Vector3d gvf_manager::pointFromClosedW(double w) const
{
    const int N = static_cast<int>(circle_reference_traj_.rows());
    if (N <= 0 || circle_reference_w_.size() != static_cast<size_t>(N) ||
        circle_reference_total_w_ <= 1e-9) {
        return Eigen::Vector3d::Zero();
    }

    const double s = wrapClosedW(w);
    const int i = indexFromClosedW(w);
    const int j = (i + 1) % N;
    const double w0 = circle_reference_w_[i];
    const double w1 = (i == N - 1) ? circle_reference_total_w_ : circle_reference_w_[j];
    const double denom = std::max(1e-9, w1 - w0);
    const double u = std::max(0.0, std::min(1.0, (s - w0) / denom));
    const Eigen::Vector3d p0 = circle_reference_traj_.row(i).transpose();
    const Eigen::Vector3d p1 = circle_reference_traj_.row(j).transpose();
    return (1.0 - u) * p0 + u * p1;
}

Eigen::Vector3d gvf_manager::tangentFromClosedW(double w) const
{
    const int N = static_cast<int>(circle_reference_traj_.rows());
    if (N <= 1 || circle_reference_w_.size() != static_cast<size_t>(N) ||
        circle_reference_total_w_ <= 1e-9) {
        return Eigen::Vector3d::Zero();
    }

    const int i = indexFromClosedW(w);
    const int j = (i + 1) % N;
    Eigen::Vector3d tangent = circle_reference_traj_.row(j).transpose() -
                              circle_reference_traj_.row(i).transpose();
    const double norm = tangent.norm();
    if (norm <= 1e-9) {
        return Eigen::Vector3d::Zero();
    }
    return tangent / norm;
}

double gvf_manager::findInitialClosedPhase(const Eigen::Vector3d& curr_pos) const
{
    const int N = static_cast<int>(circle_reference_traj_.rows());
    if (N <= 1 || circle_reference_w_.size() != static_cast<size_t>(N) ||
        circle_reference_total_w_ <= 1e-9) {
        return 0.0;
    }

    if (closed_ref_initial_phase_w_ >= 0.0) {
        return closed_ref_initial_phase_w_;
    }

    const Eigen::Vector2d odom_v_xy = odom_vel_lpf_.head<2>();
    const double odom_v_norm = odom_v_xy.norm();
    const bool use_odom_direction = odom_v_norm > 0.2;
    Eigen::Vector2d odom_dir = Eigen::Vector2d::Zero();
    if (use_odom_direction) {
        odom_dir = odom_v_xy / odom_v_norm;
    }
    const double dir_weight = 0.25;

    double best_w = 0.0;
    double best_score = std::numeric_limits<double>::infinity();
    for (int i = 0; i < N; ++i) {
        const int j = (i + 1) % N;
        const Eigen::Vector3d p0 = circle_reference_traj_.row(i).transpose();
        const Eigen::Vector3d p1 = circle_reference_traj_.row(j).transpose();
        const Eigen::Vector3d seg = p1 - p0;
        const double seg_len_sq = seg.squaredNorm();
        if (seg_len_sq <= 1e-12) {
            continue;
        }

        const double u = std::max(0.0, std::min(1.0, (curr_pos - p0).dot(seg) / seg_len_sq));
        const Eigen::Vector3d proj = p0 + u * seg;
        const double dist_sq = (curr_pos - proj).squaredNorm();
        const double w0 = circle_reference_w_[i];
        const double w1 = (i == N - 1) ? circle_reference_total_w_ : circle_reference_w_[j];
        const double cand_w = w0 + u * (w1 - w0);

        double tangent_dot_odom = 0.0;
        if (use_odom_direction) {
            const Eigen::Vector2d tangent_xy = seg.head<2>();
            const double tangent_norm = tangent_xy.norm();
            if (tangent_norm > 1e-9) {
                tangent_dot_odom = (tangent_xy / tangent_norm).dot(odom_dir);
            }
        }

        const double score = dist_sq + (use_odom_direction ? dir_weight * (1.0 - tangent_dot_odom) : 0.0);
        if (score < best_score) {
            best_score = score;
            best_w = cand_w;
        }
    }

    return best_w;
}

double gvf_manager::projectClosedLocal(const Eigen::Vector3d& curr_pos,
                                       double w_prev,
                                       double back_window,
                                       double forward_window) const
{
    const int N = static_cast<int>(circle_reference_traj_.rows());
    if (N <= 1 || circle_reference_w_.size() != static_cast<size_t>(N) ||
        circle_reference_total_w_ <= 1e-9) {
        return w_prev;
    }

    const double w_min = w_prev - std::max(0.0, back_window);
    const double w_max = w_prev + std::max(0.0, forward_window);
    const double step = std::max(0.02, circle_reference_total_w_ / static_cast<double>(std::max(200, 4 * N)));

    double best_w = w_prev;
    double best_dist_sq = std::numeric_limits<double>::infinity();
    for (double cand_w = w_min; cand_w <= w_max + 1e-9; cand_w += step) {
        const Eigen::Vector3d p = pointFromClosedW(cand_w);
        const double dist_sq = (curr_pos - p).squaredNorm();
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_w = cand_w;
        }
    }

    return best_w;
}

double gvf_manager::closedRefAlpha(double rho) const
{
    const double r = std::max(1e-6, ref_alpha_rho_);
    const double s = rho / r;
    return 1.0 / (1.0 + s * s);
}

double gvf_manager::closedRefSigma(double e_parallel) const
{
    const double scale = std::max(1e-6, ref_sigma_scale_);
    return std::tanh(e_parallel / scale);
}

double gvf_manager::updateClosedRefPhaseByDynamics(const Eigen::Vector3d& curr_pos, double dt)
{
    const double old_w = closed_ref_w_;
    const Eigen::Vector3d p = pointFromClosedW(old_w);
    Eigen::Vector3d tau = tangentFromClosedW(old_w);
    if (tau.norm() > 1e-6) {
        tau.normalize();
    } else {
        tau.setZero();
    }

    const Eigen::Vector3d e = curr_pos - p;
    const double e_parallel = tau.dot(e);
    const Eigen::Vector3d e_perp = e - e_parallel * tau;
    const double rho = e_perp.head<2>().norm();
    const double alpha = closedRefAlpha(rho);
    const double sigma = closedRefSigma(e_parallel);

    double w_dot = ref_phase_k1_ * (alpha + sigma);
    w_dot = std::max(-std::max(0.0, ref_wdot_backward_max_),
                     std::min(w_dot, std::max(0.0, ref_wdot_forward_max_)));

    const double safe_dt = std::max(0.0, dt);
    const double w_dyn = old_w + w_dot * safe_dt;

    double effective_forward_w = closed_ref_search_forward_w_;
    if (closed_ref_has_accepted_goal_) {
        effective_forward_w = std::max(effective_forward_w,
                                       closed_ref_accepted_lookahead_w_ + 0.5);
    }

    const double w_proj = projectClosedLocal(curr_pos, w_dyn,
                                             closed_ref_search_back_w_,
                                             effective_forward_w);
    const double project_delta = w_proj - w_dyn;
    const double blend = std::max(0.0, std::min(1.0, ref_project_blend_));
    const double boundary_eps = std::max(0.0, ref_project_boundary_eps_);
    const bool projected_on_boundary =
        std::abs(project_delta + std::max(0.0, closed_ref_search_back_w_)) < boundary_eps ||
        std::abs(project_delta - std::max(0.0, effective_forward_w)) < boundary_eps;
    const bool used_project_blend =
        !projected_on_boundary &&
        std::abs(project_delta) < std::max(0.0, ref_project_snap_max_);
    const double new_w = used_project_blend ? (1.0 - blend) * w_dyn + blend * w_proj : w_dyn;

    closed_ref_w_ = new_w;
    closed_ref_dbg_e_parallel_ = e_parallel;
    closed_ref_dbg_rho_ = rho;
    closed_ref_dbg_alpha_ = alpha;
    closed_ref_dbg_sigma_ = sigma;
    closed_ref_dbg_w_dot_ = w_dot;
    closed_ref_dbg_dt_ = safe_dt;
    closed_ref_dbg_w_dyn_ = w_dyn;
    closed_ref_dbg_w_proj_ = w_proj;
    closed_ref_dbg_project_delta_ = project_delta;
    closed_ref_dbg_used_project_blend_ = used_project_blend;
    closed_ref_dbg_projected_on_boundary_ = projected_on_boundary;
    closed_ref_dbg_boundary_eps_ = boundary_eps;
    if (used_project_blend) {
        closed_ref_dbg_project_blend_skipped_reason_ = "none";
    } else if (projected_on_boundary) {
        closed_ref_dbg_project_blend_skipped_reason_ = "boundary";
    } else {
        closed_ref_dbg_project_blend_skipped_reason_ = "snap_limit";
    }
    closed_ref_dbg_phase_delta_ = new_w - old_w;

    return closed_ref_w_;
}

std::vector<double> gvf_manager::buildClosedLookaheadCandidates() const
{
    std::vector<double> candidates;
    const double min_w = std::max(0.0, std::min(closed_ref_lookahead_min_w_, closed_ref_lookahead_max_w_));
    const double max_w = std::max(min_w, std::max(closed_ref_lookahead_min_w_, closed_ref_lookahead_max_w_));
    const double step_w = closed_ref_lookahead_step_w_ > 1e-6 ? closed_ref_lookahead_step_w_ : 0.5;

    for (double lookahead = min_w; lookahead <= max_w + 1e-9; lookahead += step_w) {
        candidates.push_back(lookahead);
    }

    if (candidates.empty() || std::abs(candidates.back() - max_w) > 1e-6) {
        candidates.push_back(max_w);
    }

    if (candidates.empty()) {
        candidates.push_back(2.0);
    }

    return candidates;
}

void gvf_manager::resetClosedGoalCandidateState()
{
    last_selected_goal_w_ = 0.0;
    last_selected_lookahead_w_ = 0.0;
    last_selected_goal_idx_ = -1;
    last_failed_goal_idx_ = -1;
    has_last_selected_goal_ = false;
    last_closed_goal_plan_success_ = true;
    closed_ref_last_selected_goal_w_ = 0.0;
    closed_ref_last_selected_lookahead_w_ = 0.0;
    closed_ref_has_selected_goal_ = false;
    closed_ref_pending_goal_w_ = 0.0;
    closed_ref_pending_lookahead_w_ = 0.0;
    closed_ref_has_pending_goal_ = false;
    closed_ref_pending_from_bypass_ = false;
    closed_ref_accepted_goal_w_ = 0.0;
    closed_ref_accepted_lookahead_w_ = 0.0;
    closed_ref_has_accepted_goal_ = false;
    closed_ref_accepted_from_bypass_ = false;
    closed_ref_last_goal_pos_.setZero();
    closed_ref_last_goal_dist_xy_ = 0.0;
    closed_ref_last_candidate_idx_ = -1;
    closed_ref_last_update_time_ = ros::Time(0);
    closed_ref_dbg_e_parallel_ = 0.0;
    closed_ref_dbg_rho_ = 0.0;
    closed_ref_dbg_alpha_ = 0.0;
    closed_ref_dbg_sigma_ = 0.0;
    closed_ref_dbg_w_dot_ = 0.0;
    closed_ref_dbg_dt_ = 0.0;
    closed_ref_dbg_w_dyn_ = closed_ref_w_;
    closed_ref_dbg_w_proj_ = closed_ref_w_;
    closed_ref_dbg_project_delta_ = 0.0;
    closed_ref_dbg_phase_delta_ = 0.0;
    closed_ref_dbg_used_project_blend_ = false;
    closed_ref_dbg_projected_on_boundary_ = false;
    closed_ref_dbg_boundary_eps_ = ref_project_boundary_eps_;
    closed_ref_dbg_project_blend_skipped_reason_ = "none";
}

void gvf_manager::ensureProgressInCurrentPathRange(double start_w, double end_w)
{
    if (end_w < start_w) {
        std::swap(start_w, end_w);
    }

    double window = 1.0;
    if (!swarmParticlesManager.empty() && swarmParticlesManager[0].gvf_) {
        window = swarmParticlesManager[0].gvf_->progress_window_;
    }

    const bool out_of_range =
        progress_w_ < start_w - window ||
        progress_w_ > end_w + window;

    if (!out_of_range) {
        return;
    }

    const double old_progress = progress_w_;
    progress_w_ = start_w;
    progress_initialized_ = true;
    if (!swarmParticlesManager.empty() && swarmParticlesManager[0].gvf_) {
        swarmParticlesManager[0].gvf_->setVisualizationProgressW(progress_w_);
    }
    ROS_WARN("[GVF][PROGRESS_RESET] old_progress=%.3f new_progress=%.3f start_w=%.3f end_w=%.3f window=%.3f reason=out_of_new_path_range",
             old_progress, progress_w_, start_w, end_w, window);
}

void gvf_manager::logReplanReason(const std::string& reason)
{
    if (swarmParticlesManager.empty()) {
        return;
    }

    const auto& pm = swarmParticlesManager[0];
    const int rows = static_cast<int>(pm.last_traj.rows());
    int curr_i = rows > 0 ? std::max(0, std::min(current_traj_index_, rows - 1)) : -1;
    int progress_i = -1;
    int start_i = curr_i;
    int end_i = curr_i;
    double remaining_w = 0.0;

    const gvf::ReparamCacheSnapshot cache = pm.gvf_
        ? pm.gvf_->captureReparamCacheSnapshot()
        : gvf::ReparamCacheSnapshot();
    if (cache.ready && !cache.w.empty()) {
        remaining_w = cache.w.back() - progress_w_;

        if (rows > 0 && cache.w.size() == static_cast<size_t>(rows)) {
            auto it = std::lower_bound(cache.w.begin(), cache.w.end(), progress_w_);
            if (it == cache.w.end()) {
                progress_i = rows - 1;
            } else {
                progress_i = static_cast<int>(std::distance(cache.w.begin(), it));
                if (progress_i > 0) {
                    const double w_hi = cache.w[progress_i];
                    const double w_lo = cache.w[progress_i - 1];
                    if (std::abs(progress_w_ - w_lo) <= std::abs(w_hi - progress_w_)) {
                        progress_i -= 1;
                    }
                }
            }
            progress_i = std::max(0, std::min(progress_i, rows - 1));
            start_i = std::max(curr_i, progress_i);
            end_i = std::min(rows - 1, start_i + std::max(1, collision_check_horizon_pts_));
        }
    }

    ROS_WARN("[GVF][REPLAN_REASON] reason=%s curr_i=%d progress_i=%d start_i=%d end_i=%d rows=%d remaining_w=%.3f",
             reason.c_str(), curr_i, progress_i, start_i, end_i, rows, remaining_w);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> gvf_manager::getCircleReferenceGoal(const Eigen::Vector3d& curr_pos)
{
    if (!circle_reference_ready_ || circle_reference_traj_.rows() <= 0 ||
        circle_reference_traj_.rows() != circle_reference_vel_.rows()) {
        return {curr_pos, Eigen::Vector3d::Zero()};
    }

    const int N = static_cast<int>(circle_reference_traj_.rows());
    if (N <= 1 || circle_reference_w_.size() != static_cast<size_t>(N) ||
        circle_reference_total_w_ <= 1e-9) {
        return {curr_pos, Eigen::Vector3d::Zero()};
    }

    std::string mode = "TRACK";
    const ros::Time now = ros::Time::now();
    double dt = 0.0;
    if (!closed_ref_last_update_time_.isZero()) {
        dt = std::max(0.0, (now - closed_ref_last_update_time_).toSec());
    }
    closed_ref_last_update_time_ = now;

    if (!closed_ref_initialized_) {
        closed_ref_w_ = findInitialClosedPhase(curr_pos);
        closed_ref_initialized_ = true;
        closed_ref_recover_ = false;
        mode = "INIT";
        closed_ref_dbg_e_parallel_ = 0.0;
        closed_ref_dbg_rho_ = 0.0;
        closed_ref_dbg_alpha_ = 0.0;
        closed_ref_dbg_sigma_ = 0.0;
        closed_ref_dbg_w_dot_ = 0.0;
        closed_ref_dbg_dt_ = 0.0;
        closed_ref_dbg_w_dyn_ = closed_ref_w_;
        closed_ref_dbg_w_proj_ = closed_ref_w_;
        closed_ref_dbg_project_delta_ = 0.0;
        closed_ref_dbg_phase_delta_ = 0.0;
        closed_ref_dbg_used_project_blend_ = false;
        closed_ref_dbg_projected_on_boundary_ = false;
        closed_ref_dbg_boundary_eps_ = ref_project_boundary_eps_;
        closed_ref_dbg_project_blend_skipped_reason_ = "none";
    }

    double effective_forward_w = closed_ref_search_forward_w_;
    std::string effective_forward_source = "default";
    if (closed_ref_has_accepted_goal_) {
        effective_forward_w = std::max(effective_forward_w,
                                       closed_ref_accepted_lookahead_w_ + 0.5);
        effective_forward_source = "accepted";
    }

    double candidate_w = closed_ref_w_;
    double local_d = (curr_pos - pointFromClosedW(closed_ref_w_)).head<2>().norm();
    double local_d_project = local_d;
    if (!closed_ref_enable_recover_) {
        closed_ref_recover_ = false;
    } else if (local_d > closed_ref_lost_radius_) {
        closed_ref_recover_ = true;
        mode = "RECOVER";
    }

    if (closed_ref_enable_recover_ && closed_ref_recover_ && mode != "INIT") {
        updateClosedRefPhaseByDynamics(curr_pos, dt);
        const double w_after_dyn = closed_ref_w_;
        const double phase_delta_after_dyn = closed_ref_dbg_phase_delta_;
        candidate_w = projectClosedLocal(curr_pos, closed_ref_w_,
                                         closed_ref_search_back_w_,
                                         effective_forward_w);
        local_d_project = (curr_pos - pointFromClosedW(candidate_w)).head<2>().norm();
        if (local_d_project < closed_ref_recover_radius_) {
            closed_ref_w_ = candidate_w;
            closed_ref_dbg_phase_delta_ = phase_delta_after_dyn + (candidate_w - w_after_dyn);
            closed_ref_dbg_w_proj_ = candidate_w;
            closed_ref_dbg_project_delta_ = candidate_w - closed_ref_dbg_w_dyn_;
            closed_ref_dbg_used_project_blend_ = true;
            closed_ref_recover_ = false;
            mode = "TRACK";
            local_d = local_d_project;
        } else {
            mode = "RECOVER";
        }
    } else if (mode != "INIT") {
        updateClosedRefPhaseByDynamics(curr_pos, dt);
        candidate_w = closed_ref_dbg_w_proj_;
        mode = "TRACK";
        local_d = (curr_pos - pointFromClosedW(closed_ref_w_)).head<2>().norm();
        local_d_project = (curr_pos - pointFromClosedW(candidate_w)).head<2>().norm();
    }

    const std::vector<double> candidates = buildClosedLookaheadCandidates();
    int selected_idx = selectDefaultClosedLookaheadIndex(
        candidates, closed_goal_prefer_lookahead_w_);
    if (closed_ref_has_accepted_goal_ && !closed_ref_accepted_from_bypass_) {
        double best_diff = std::numeric_limits<double>::infinity();
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
            const double diff = std::abs(candidates[i] - closed_ref_accepted_lookahead_w_);
            if (diff < best_diff) {
                best_diff = diff;
                selected_idx = i;
            }
        }
    }
    if (selected_idx < 0 || selected_idx >= static_cast<int>(candidates.size())) {
        selected_idx = selectDefaultClosedLookaheadIndex(
            candidates, closed_goal_prefer_lookahead_w_);
    }
    const double selected_lookahead = candidates.empty() ? 0.0 : candidates[selected_idx];
    const double goal_w = closed_ref_w_ + selected_lookahead;
    const Eigen::Vector3d goal_pt = pointFromClosedW(goal_w);
    const int idx = indexFromClosedW(closed_ref_w_);
    const int goal_idx = indexFromClosedW(goal_w);
    circle_reference_index_ = idx;

    double tangent_dot_odom = 0.0;
    const Eigen::Vector2d odom_v_xy = odom_vel_lpf_.head<2>();
    const double odom_v_norm = odom_v_xy.norm();
    if (odom_v_norm > 1e-6) {
        const Eigen::Vector2d tangent_xy = tangentFromClosedW(closed_ref_w_).head<2>();
        const double tangent_norm = tangent_xy.norm();
        if (tangent_norm > 1e-6) {
            tangent_dot_odom = (tangent_xy / tangent_norm).dot(odom_v_xy / odom_v_norm);
        }
    }

    ROS_WARN_THROTTLE(0.2,
                      "[GVF][CLOSED_REF] mode=%s closed_ref_w=%.3f w_mod=%.3f total_w=%.3f goal_w=%.3f idx=%d goal_idx=%d local_d=%.3f tangent_dot_odom=%.3f search_back_w=%.3f search_forward_w=%.3f effective_forward_w=%.3f effective_forward_source=%s pending_goal_w=%.3f pending_lookahead_w=%.3f accepted_goal_w=%.3f accepted_lookahead_w=%.3f has_pending_goal=%d has_accepted_goal=%d candidate_w=%.3f local_d_project=%.3f e_parallel=%.3f rho=%.3f alpha=%.3f sigma=%.3f w_dot=%.3f dt=%.3f w_dyn=%.3f w_proj=%.3f project_delta=%.3f used_project_blend=%d projected_on_boundary=%d boundary_eps=%.3f project_blend_skipped_reason=%s phase_delta=%.3f initialized=%d recover=%d recover_enabled=%d lost_radius=%.3f recover_radius=%.3f",
                      mode.c_str(), closed_ref_w_, wrapClosedW(closed_ref_w_), circle_reference_total_w_, goal_w,
                      idx, goal_idx, local_d, tangent_dot_odom, closed_ref_search_back_w_,
                      closed_ref_search_forward_w_, effective_forward_w, effective_forward_source.c_str(),
                      closed_ref_pending_goal_w_, closed_ref_pending_lookahead_w_,
                      closed_ref_accepted_goal_w_, closed_ref_accepted_lookahead_w_,
                      closed_ref_has_pending_goal_ ? 1 : 0,
                      closed_ref_has_accepted_goal_ ? 1 : 0,
                      candidate_w, local_d_project,
                      closed_ref_dbg_e_parallel_, closed_ref_dbg_rho_,
                      closed_ref_dbg_alpha_, closed_ref_dbg_sigma_, closed_ref_dbg_w_dot_,
                      closed_ref_dbg_dt_, closed_ref_dbg_w_dyn_, closed_ref_dbg_w_proj_,
                      closed_ref_dbg_project_delta_, closed_ref_dbg_used_project_blend_ ? 1 : 0,
                      closed_ref_dbg_projected_on_boundary_ ? 1 : 0,
                      closed_ref_dbg_boundary_eps_,
                      closed_ref_dbg_project_blend_skipped_reason_.c_str(),
                      closed_ref_dbg_phase_delta_,
                      closed_ref_initialized_ ? 1 : 0, closed_ref_recover_ ? 1 : 0,
                      closed_ref_enable_recover_ ? 1 : 0,
                      closed_ref_lost_radius_, closed_ref_recover_radius_);

    return {goal_pt, Eigen::Vector3d::Zero()};
}

void gvf_manager::visualizePath(const std::vector<Eigen::Vector3d>& path_points, 
                    ros::Publisher& marker_pub, const std::string& particle_index) 
{
    // 定义一个Marker消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";  // 根据实际使用的坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "path_visualization";
    
    // 使用 particle_index 作为 marker 的唯一 ID
    std::hash<std::string> hash_fn;
    marker.id = static_cast<int>(hash_fn(particle_index));
    
    // 使用POINTS类型以便可以设置每个点的颜色
    marker.type = visualization_msgs::Marker::POINTS; 
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1; 
    marker.scale.y = 0.1;

    // 遍历路径点并将其添加到Marker中
    for (size_t i = 0; i < path_points.size(); ++i) {
        geometry_msgs::Point p;
        p.x = path_points[i].x();
        p.y = path_points[i].y();
        p.z = path_points[i].z();

        // 将当前点添加到Marker
        marker.points.push_back(p);

        // 设置每个点的颜色
        std_msgs::ColorRGBA color;
        if (i == path_points.size() - 1) {
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 1.0;

        } else {
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 1.0;
        }
        // 将颜色添加到Marker中
        marker.colors.push_back(color);
    }

    // 发布Marker消息
    marker_pub.publish(marker);
}

bool gvf_manager::checkCollision()
{
    if (swarmParticlesManager.empty()) return false;

    auto& pm = swarmParticlesManager[0];
    if (pm.last_traj.rows() == 0) return false;  // 没有轨迹时不检查碰撞
    if (collision_threshold_ <= 0.0) return false;

    const int traj_rows = static_cast<int>(pm.last_traj.rows());
    const int horizon_pts = std::max(1, collision_check_horizon_pts_);

    int start_idx = std::max(0, std::min(current_traj_index_, traj_rows - 1));
    int progress_idx_log = -1;
    const gvf::ReparamCacheSnapshot cache = pm.gvf_
        ? pm.gvf_->captureReparamCacheSnapshot()
        : gvf::ReparamCacheSnapshot();
    if (cache.ready && cache.w.size() == static_cast<size_t>(traj_rows)) {
        auto it = std::lower_bound(cache.w.begin(), cache.w.end(), progress_w_);
        int progress_idx = traj_rows - 1;
        if (it != cache.w.end()) {
            progress_idx = static_cast<int>(std::distance(cache.w.begin(), it));
            if (progress_idx > 0) {
                const double w_hi = cache.w[progress_idx];
                const double w_lo = cache.w[progress_idx - 1];
                if (std::abs(progress_w_ - w_lo) <= std::abs(w_hi - progress_w_)) {
                    progress_idx -= 1;
                }
            }
        }
        progress_idx = std::max(0, std::min(progress_idx, traj_rows - 1));
        progress_idx_log = progress_idx;
        start_idx = std::max(start_idx, progress_idx);
    }

    const int end_idx = std::min(traj_rows - 1, start_idx + horizon_pts);
    ROS_WARN_THROTTLE(0.5, "[GVF][COLL] curr_i=%d progress_i=%d start_i=%d end_i=%d progress=%.3f rows=%d",
                      current_traj_index_, progress_idx_log, start_idx, end_idx, progress_w_, traj_rows);
    int consecutive_hits = 0;

    // 优先使用 lifted progress 对应的未来段做碰撞检查；若 progress 不可用则退化为当前索引。
    for (int i = start_idx; i <= end_idx; ++i) {
        Eigen::Vector3d point(pm.last_traj(i, 0), pm.last_traj(i, 1), pm.last_traj(i, 2));
        const int occ = pm.sdf_map_->getInflateOccupancy(point);
        const double distance = (occ != 0) ? 0.0 : pm.sdf_map_->getDistance(point);

        if (distance < collision_threshold_) {
            consecutive_hits++;
            if (consecutive_hits >= std::max(1, collision_consecutive_hits_)) {
                if (ros::isInitialized()) {
                    ROS_WARN_THROTTLE(1.0,
                        "[GVF][COLL_HIT] i=%d start_i=%d end_i=%d rows=%d occ=%d "
                        "dist=%.3f thr=%.3f pts=(%.2f,%.2f,%.2f) progress_w=%.3f "
                        "d_self=%.2f",
                        i, start_idx, end_idx, traj_rows, occ, distance,
                        collision_threshold_, point.x(), point.y(), point.z(),
                        progress_w_, (point - odom_).norm());
                }
                return true;
            }
        } else {
            consecutive_hits = 0;
        }
    }
    return false;
}

void gvf_manager::KinoPathCallback(const ros::TimerEvent& event)
{
    if (!use_kinopath_) return;  // 如果不使用动力学路径，则不处理
    auto& pm = swarmParticlesManager[0];
    if (!pm.receive_startpt) return;
    // 确定起点和初始速度
    Eigen::Vector3d start_pt;
    Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();  // 初始速度默认为0
    
    if (pm.is_first_kinogoal) {
        start_pt = Eigen::Vector3d(odom_.x(), odom_.y(), odom_.z());
        pm.is_first_kinogoal = false;
    } else {
        // 获取上一次的动力学轨迹
        std::vector<Eigen::Vector3d> last_path = pm.last_path;
        if (!last_path.empty()) {
            double min_dist = std::numeric_limits<double>::max();
            int nearest_idx = 0;
            Eigen::Vector3d current_pos(odom_.x(), odom_.y(), odom_.z());
            
            // 找到当前位置在路径上的最近点
            for (size_t i = 0; i < last_path.size(); ++i) {
                double dist = (last_path[i] - current_pos).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest_idx = i;
                }
            }
            
            // 设置起点和初始速度
            start_pt = last_path[nearest_idx];
            
            // 计算路径方向（使用下一个点）
            if (nearest_idx < last_path.size() - 1) {
                Eigen::Vector3d direction = (last_path[nearest_idx + 1] - last_path[nearest_idx]).normalized();
                // 获取GVF增益
                double gvf_gain = pm.gvf_->gvf_.K1_;
                // 设置速度大小和方向
                start_vel = direction * gvf_gain;
            }
        } else {
            start_pt = Eigen::Vector3d(odom_.x(), odom_.y(), odom_.z());
        }
    }
    // 设置初始状态
    Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();  // 初始加速度
    Eigen::Vector3d original_goal = pm.goal_pt;
    Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();    // 目标速度

    // 调整目标点，确保在horizon_范围内
    Eigen::Vector3d end_pt = original_goal;
    double path_dist_to_goal = (original_goal - start_pt).norm();
    double horizon = pm.kino_path_finder_->horizon_;  // 获取horizon_参数

    // 添加终点衰减机制
    const double decay_start_dist = 1.0;  // 开始衰减的距离阈值
    const double stop_dist = 0.1;         // 停止的距离阈值
    
    if (path_dist_to_goal < decay_start_dist) {
        // 计算衰减系数
        double decay_factor = std::max(0.0, (path_dist_to_goal - stop_dist) / (decay_start_dist - stop_dist));
        
        // 对速度进行衰减
        start_vel *= decay_factor;
        
        // 如果非常接近终点，直接设置速度为零
        if (path_dist_to_goal < stop_dist) {
            start_vel = Eigen::Vector3d::Zero();
            end_vel = Eigen::Vector3d::Zero();
        }
    }

    if (path_dist_to_goal > horizon) {
        // 计算方向向量
        Eigen::Vector3d direction = (original_goal - start_pt).normalized();
        // 计算horizon范围内的点
        end_pt = start_pt + direction * horizon;
        
        // 检查该点是否被占据
        if (pm.sdf_map_->getInflateOccupancy(end_pt) == 1) {
            // 如果被占据，在horizon范围内寻找最近的可达点
            double search_radius = horizon;
            double angle_step = M_PI / 8;  // 22.5度
            bool found_valid_point = false;
            
            for (double angle = 0; angle < 2 * M_PI; angle += angle_step) {
                for (double r = search_radius; r > 0; r -= 0.1) {  // 从外向内搜索
                    Eigen::Vector3d test_point = start_pt + Eigen::Vector3d(
                        r * cos(angle),
                        r * sin(angle),
                        0
                    );
                    
                    if (pm.sdf_map_->getInflateOccupancy(test_point) == 0) {
                        end_pt = test_point;
                        found_valid_point = true;
                        break;
                    }
                }
                if (found_valid_point) break;
            }
        }
    }

    int result = pm.kino_path_finder_->search(start_pt, start_vel, start_acc, 
                                            end_pt, end_vel, false);
    
    // 检查搜索结果
    if (result == KinodynamicAstar::NO_PATH) {
        ROS_WARN("[GVF] No valid path found, skipping trajectory generation"); 
        return;
    }

    std::vector<Eigen::Vector3d> kino_path = pm.kino_path_finder_->getKinoTraj(0.01);  // 0.01s的时间间隔
    pm.last_path = kino_path;
    if (!kino_path.empty()) {
        // Diagnostic only: the A*/kino plan is expected to end at the task
        // goal.  A stalled agent's path has been observed to stop several
        // metres short of it, so record start/end/goal and the path extent.
        double path_len = 0.0;
        for (std::size_t i = 1; i < kino_path.size(); ++i) {
            path_len += (kino_path[i] - kino_path[i - 1]).norm();
        }
        if (ros::isInitialized()) {
            ROS_INFO_THROTTLE(
            1.0,
            "[GVF][PATH] cmd=%d start=(%.2f,%.2f) end=(%.2f,%.2f) "
            "path_last=(%.2f,%.2f) len=%.2f end_to_goal=%.2f "
            "last_to_goal=%.2f goal=(%.2f,%.2f) d0_to_goal=%.2f horizon=%.2f "
            "pts=%zu",
            result, start_pt.x(), start_pt.y(), end_pt.x(), end_pt.y(),
            kino_path.back().x(), kino_path.back().y(), path_len,
            (end_pt - original_goal).norm(),
            (kino_path.back() - original_goal).norm(), original_goal.x(),
            original_goal.y(), path_dist_to_goal, horizon, kino_path.size());
        }
    }

    // 发布动力学路径
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();

    for (const auto& pt : kino_path) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pt.x();
        pose.pose.position.y = pt.y();
        pose.pose.position.z = pt.z();
        path_msg.poses.push_back(pose);
    }
    kino_path_pub.publish(path_msg);
}

// void gvf_manager::execTimerCallback(const ros::TimerEvent& event)
// {
//     if (use_kinopath_) return;
//     if (swarmParticlesManager.empty()) return;

//     ros::Time current_time = ros::Time::now();
//     bool need_replan = false;
//     std::string trigger_reason = "None";

//     // auto& pm = swarmParticlesManager[0];

//     auto& pm = swarmParticlesManager[0];
//     if (!pm.receive_goal) return;   // 没有目标就别生成轨迹

//     auto clearPublishedTraj = [&]() {
//         nav_msgs::Path empty_path;
//         empty_path.header.frame_id = "world";
//         empty_path.header.stamp = ros::Time::now();
//         path_pub.publish(empty_path);
//         kino_path_pub.publish(empty_path);

//         visualization_msgs::Marker del;
//         del.header.frame_id = "world";
//         del.header.stamp = ros::Time::now();
//         del.ns = "path_visualization";
//         std::hash<std::string> hash_fn;
//         del.id = static_cast<int>(hash_fn(pm.index));
//         del.action = visualization_msgs::Marker::DELETE;
//         path_vis.publish(del);

//         pm.last_traj.resize(0, 0);
//         pm.last_path.clear();
//     };

//     // 到达目标附近时停止重规划：否则 start≈goal 会导致 kino 采样 ts→很小、guide_pts 过多、轨迹长度爆炸
//     double replan_stop_radius;
//     ros::param::param("~gvf/replan_stop_radius", replan_stop_radius, stop_radius);
//     const double dist_to_goal_xy = (pm.goal_pt.head<2>() - odom_.head<2>()).norm();
//     if (!pm.receive_startpt && dist_to_goal_xy < replan_stop_radius) {
//         clearPublishedTraj();
//         return;
//     }


//     // 1) 首次触发
//     if (pm.receive_startpt) {
//         need_replan = true;
//         trigger_reason = "receive_startpt";
//     }

//     // 2) 碰撞触发（带冷却，避免阈值附近来回触发导致抽动）
//     if (!need_replan && checkCollision()) {
//         const double since_last_collision_replan = (current_time - last_collision_replan_time_).toSec();
//         if (since_last_collision_replan >= collision_replan_cooldown_) {
//             need_replan = true;
//             trigger_reason = "collision detection";
//             last_collision_replan_time_ = current_time;
//         } else {
//             trigger_reason = "collision detection (cooldown)";
//         }
//     }

//     // 3) 定时 + 尾段触发
//     if (!need_replan && (current_time - last_replan_time_).toSec() >= planInterval) {
//         // const int rows = pm.last_traj.rows();
//         // if (rows > 0) {
//         //     const int tail_margin = std::min(200, rows-1);
//         //     const int tail_threshold = rows - 1 - tail_margin;
//         //     if (current_traj_index_ >= tail_threshold) {
//         //         need_replan = true;
//         //         trigger_reason = "trajectory near end after planInterval";
//         //     } else {
//         //         last_replan_time_ = current_time;
//         //         trigger_reason = "planInterval reached but not near end";
//         //     }
//         // } else {
//         //     need_replan = true;
//         //     trigger_reason = "no trajectory yet";
//         // }
//         need_replan = true;
//         trigger_reason = "planInterval reached";
//     }
    
//     // 4) 执行或不执行重规划
//     if (need_replan) {
//         // 重规划前先在旧轨迹上更新当前索引，保证 remain_old / old_err 等评估更稳定
//         if (pm.last_traj.rows() > 0) {
//             Eigen::Vector3d current_pos(odom_.x(), odom_.y(), odom_.z());
//             double min_dist = std::numeric_limits<double>::max();
//             for (int i = current_traj_index_; i < pm.last_traj.rows(); ++i) {
//                 Eigen::Vector3d traj_point(pm.last_traj(i,0), pm.last_traj(i,1), pm.last_traj(i,2));
//                 double dist = (traj_point - current_pos).norm();
//                 if (dist < min_dist) {
//                     min_dist = dist;
//                     current_traj_index_ = i;
//                 }
//             }
//         } else {
//             current_traj_index_ = 0;
//         }
//         astaropt();
//         last_replan_time_ = current_time;

//         ROS_INFO_THROTTLE(2.0,
//         "\033[36m[GVF] ExecTimer:\033[0m need_replan=%d, reason=%s, idx=%d, rows=%d",
//         need_replan, trigger_reason.c_str(),
//         current_traj_index_, (int)pm.last_traj.rows());

//     } else {
//         if (pm.last_traj.rows() > 0) {
//             Eigen::Vector3d current_pos(odom_.x(), odom_.y(), odom_.z());
//             double min_dist = std::numeric_limits<double>::max();
//             for (int i = current_traj_index_; i < pm.last_traj.rows(); ++i) {
//                 Eigen::Vector3d traj_point(pm.last_traj(i,0), pm.last_traj(i,1), pm.last_traj(i,2));
//                 double dist = (traj_point - current_pos).norm();
//                 if (dist < min_dist) {
//                     min_dist = dist;
//                     current_traj_index_ = i;
//                 }
//             }
//         }
//     }
// }

    // void gvf_manager::execTimerCallback(const ros::TimerEvent& event)
    // {
    //     if (swarmParticlesManager.empty()) return;

    //     ros::Time current_time = ros::Time::now();
    //     bool need_replan = false;
    //     std::string trigger_reason = "None";

    //     auto& pm = swarmParticlesManager[0];

    //     // 1) 首次触发
    //     if (pm.receive_startpt) {
    //         need_replan = true;
    //         trigger_reason = "receive_startpt";
    //     }

    //     // 2) 碰撞触发
    //     if (!need_replan && checkCollision()) {
    //         need_replan = true;
    //         trigger_reason = "collision detection";
    //     }

    //     // 3) 定时 + 尾段触发
    //     if (!need_replan && (current_time - last_replan_time_).toSec() >= planInterval) {
    //         // const int rows = pm.last_traj.rows();
    //         // if (rows > 0) {
    //         //     const int tail_margin = std::min(200, rows-1);
    //         //     const int tail_threshold = rows - 1 - tail_margin;
    //         //     if (current_traj_index_ >= tail_threshold) {
    //         //         need_replan = true;
    //         //         trigger_reason = "trajectory near end after planInterval";
    //         //     } else {
    //         //         last_replan_time_ = current_time;
    //         //         trigger_reason = "planInterval reached but not near end";
    //         //     }
    //         // } else {
    //         //     need_replan = true;
    //         //     trigger_reason = "no trajectory yet";
    //         // }
    //         need_replan = true;
    //         trigger_reason = "planInterval reached";
    //     }
        
	//         // 4) 执行或不执行重规划
	//         if (need_replan) {
    //         // 重规划前先在旧轨迹上更新当前索引
    //         if (pm.last_traj.rows() > 0) {
    //             Eigen::Vector3d current_pos(odom_.x(), odom_.y(), odom_.z());
    //             double min_dist = std::numeric_limits<double>::max();
    //             for (int i = current_traj_index_; i < pm.last_traj.rows(); ++i) {
    //                 Eigen::Vector3d traj_point(pm.last_traj(i,0), pm.last_traj(i,1), pm.last_traj(i,2));
    //                 double dist = (traj_point - current_pos).norm();
    //                 if (dist < min_dist) {
    //                     min_dist = dist;
    //                     current_traj_index_ = i;
    //                 }
    //             }
    //         } else {
    //             current_traj_index_ = 0;
    //         }
	//             Eigen::MatrixXd cand_traj, cand_vel;
	//             int new_i0 = 0;
	//             if (astaropt(Eigen::Vector3d(odom_.x(), odom_.y(), odom_.z()), cand_traj, cand_vel, new_i0)) {
	//                 // execTimerCallback 模式：保持旧行为，直接覆盖
	//                 pm.last_traj = cand_traj;
	//                 pm.last_vel = cand_vel;
	//                 current_traj_index_ = new_i0;
	//                 last_switch_time_ = current_time;
	//                 publishPathMsg(pm.last_traj, pm.last_vel);
	//             } else {
	//                 ROS_WARN_THROTTLE(1.0, "[GVF] execTimer: plan failed, keep old");
	//                 publishPathMsg(pm.last_traj, pm.last_vel);
	//             }
	//             last_replan_time_ = current_time;

    //         ROS_INFO_THROTTLE(2.0,
    //         "\033[36m[GVF] ExecTimer:\033[0m need_replan=%d, reason=%s, idx=%d, rows=%d",
    //         need_replan, trigger_reason.c_str(),
    //         current_traj_index_, (int)pm.last_traj.rows());

    //     } else {
    //         if (pm.last_traj.rows() > 0) {
    //             Eigen::Vector3d current_pos(odom_.x(), odom_.y(), odom_.z());
    //             double min_dist = std::numeric_limits<double>::max();
    //             for (int i = current_traj_index_; i < pm.last_traj.rows(); ++i) {
    //                 Eigen::Vector3d traj_point(pm.last_traj(i,0), pm.last_traj(i,1), pm.last_traj(i,2));
    //                 double dist = (traj_point - current_pos).norm();
    //                 if (dist < min_dist) {
    //                     min_dist = dist;
    //                     current_traj_index_ = i;
    //                 }
    //             }
    //         }
    //     }
        
    // }
// void gvf_manager::astaropt() 
// {   
//     ros::Time t1 = ros::Time::now();
    
//     auto& pm = swarmParticlesManager[0];  
//     // auto& pm = swarmParticlesManager[0];
//     if (!pm.receive_goal) { pm.receive_startpt = false; return; }

//     /*----------- ① A* 搜索路径 -----------*/
//     // Eigen::Vector3d start_pt;
    
//     // if (pm.is_first_goal) {
//     //     // 第一次接收到目标点时，使用当前位置作为起点
//     //     start_pt = Eigen::Vector3d(odom_.x()+0.000001, odom_.y()+0.000001, 1.0);
//     //     pm.is_first_goal = false;
//     // } else {
//     //     // 不是第一次时，找到上一次路径上与当前位置最近的点作为起点
//     //     std::vector<Eigen::Vector3d> last_path = pm.geo_path_finder_->getPath();
//     //     if (!last_path.empty()) {
//     //         double min_dist = std::numeric_limits<double>::max();
//     //         Eigen::Vector3d current_pos(odom_.x(), odom_.y(), odom_.z());
            
//     //         for (const auto& pt : last_path) {
//     //             double dist = (pt - current_pos).norm();
//     //             if (dist < min_dist) {
//     //                 min_dist = dist;
//     //                 start_pt = pt;
//     //             }
//     //         }
//     //     } else {
//     //         // 如果没有上一次路径，使用当前位置
//     //         start_pt = Eigen::Vector3d(odom_.x()+0.000001, odom_.y()+0.000001, 1.0);
//     //     }
//     // }

//     //20260126修改
//     /****************************** */
//     auto nearestIdxInTraj = [&](const Eigen::MatrixXd& traj, const Eigen::Vector3d& pos) {
//         int best = 0;
//         double best_d = std::numeric_limits<double>::max();
//         for (int i = 0; i < traj.rows(); ++i) {
//             Eigen::Vector3d p(traj(i, 0), traj(i, 1), traj(i, 2));
//             double d = (p - pos).squaredNorm();
//             if (d < best_d) { best_d = d; best = i; }
//         }
//         return best;
//     };

//     Eigen::Vector3d curr_pos(odom_.x(), odom_.y(), odom_.z());
//     Eigen::Vector3d start_pt, start_vel = Eigen::Vector3d::Zero(), start_acc = Eigen::Vector3d::Zero();

//     if (pm.is_first_goal || pm.last_traj.rows() == 0) {
//         start_pt = Eigen::Vector3d(odom_.x() + 1e-6, odom_.y() + 1e-6, 1.0);
//         pm.is_first_goal = false;
//     } else {
//         int i0 = std::max(0, std::min(current_traj_index_, (int)pm.last_traj.rows() - 1));
//         start_pt = pm.last_traj.row(i0).transpose();
//         start_vel = pm.last_vel.row(i0).transpose();

//         double dt = 1.0 / std::max(1, pm.spline_->TrajSampleRate);
//         if (i0 + 1 < pm.last_vel.rows()) {
//             start_acc = (pm.last_vel.row(i0 + 1) - pm.last_vel.row(i0)).transpose() / dt;
//         }
//     }

//     // 计算轨迹未来 K 个点内的最小 ESDF 距离（含碰撞检测：若占据则直接返回 0）
// 	    auto minDistFutureTraj = [&](const Eigen::MatrixXd& traj, int start_idx, int K) {
// 	        double md = std::numeric_limits<double>::infinity();
// 	        int end = std::min((int)traj.rows(), start_idx + K);
// 	        for (int i = std::max(0, start_idx); i < end; ++i) {
// 	            Eigen::Vector3d pt(traj(i, 0), traj(i, 1), traj(i, 2));
// 	            if (!pm.sdf_map_->isInMap(pt)) return 0.0;
// 	            if (pm.sdf_map_->isUnknown(pt)) return 0.0;
// 	            if (pm.sdf_map_->getInflateOccupancy(pt) != 0) return 0.0;
// 	            md = std::min(md, pm.sdf_map_->getDistance(pt));
// 	        }
// 	        return md;
// 	    };

//     // 轨迹拼接：只用于“发布段融合”（让控制更平滑），不允许覆盖论文的起点选择逻辑。
//     // 注意：无论是否启用拼接，本函数的 KinoA* 搜索起点始终按“最近点/odom”逻辑得到 start_pt/start_vel/start_acc。
// 	    int concat_old_i0 = -1;
// 	    int concat_keep = 0;
// 	    if (enable_trajectory_concatenation_ && pm.last_traj.rows() >= 2) {
// 	        concat_old_i0 = std::max(0, std::min(current_traj_index_, (int)pm.last_traj.rows() - 1));
// 	        int max_keep = std::min(max_trajectory_concatenation_points_, (int)pm.last_traj.rows() - concat_old_i0);

//         double keep_time;
//         ros::param::param("~gvf/trajectory_keep_time", keep_time, -1.0);  // 秒；<=0 则用点数
//         if (keep_time > 0.0) {
//             concat_keep = (int)llround(keep_time * std::max(1, pm.spline_->TrajSampleRate));
//             concat_keep = std::min(concat_keep, max_keep);
//         } else {
//             concat_keep = max_keep;
//         }

//         if (concat_keep >= 2) {
//             // 旧轨迹保留段必须“足够安全”，否则不要保留（避免把飞机继续送进墙边/障碍）
//             const double md_keep = minDistFutureTraj(pm.last_traj, concat_old_i0, concat_keep);
//             if (md_keep < safe_distance_) {
//                 concat_keep = 0;
//             }
//         }
//     }
//     /************************************************ */

//     //修改：每次重归划用当前位置当起点
//     // start_pt = Eigen::Vector3d(odom_.x()+0.000001, odom_.y()+0.000001, odom_.z());

//     // Eigen::Vector3d goal_pt = pm.goal_pt;  
         
//     //do traj opt here
//     Eigen::MatrixXd initial_state(3,3),terminal_state(3,3);//初始，结束P V A  
//     Eigen::Vector3d start_v, end_v, start_a;
//     // Eigen::Vector3d end_pt, start_v, end_v, start_a;
//     // std::vector<Eigen::Vector3d> initial_ctrl_ps;

//     ros::Time t2 = ros::Time::now();

//     // pm.geo_path_finder_->reset();
//     // pm.geo_path_finder_->search(start_pt, goal_pt, false, -1.0);
//     // std::vector<Eigen::Vector3d> path_points = pm.geo_path_finder_->getprunePath();   
//     // std::vector<Eigen::Vector3d> raw_path = pm.geo_path_finder_->getPath();   
//     // visualizePath(path_points, path_vis, pm.index); 
//     // ros::Time t3 = ros::Time::now();

//     // int num_points_to_take = std::min(static_cast<int>(path_points.size()), this->num_points_to_take_);
//     // for (int i = 0; i < num_points_to_take; ++i) {
//     //     initial_ctrl_ps.push_back(path_points[i]);
//     // }
//     // if (initial_ctrl_ps.empty()) {
//     //     ROS_INFO("[DEBUG] No new control points, publishing last trajectory");
//     //     return;
//     // }
//     // end_pt = initial_ctrl_ps.back();   

//     /********************************** */
//     Eigen::Vector3d goal_pt = pm.goal_pt;
//     Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();

//     // 如果已经到达目标附近，停止规划（避免 start≈goal 时 kino 输出退化、反复重规划产生圈/长轨迹）
//     double replan_stop_radius;
//     ros::param::param("~gvf/replan_stop_radius", replan_stop_radius, stop_radius);
//     double replan_stop_vel;
//     ros::param::param("~gvf/replan_stop_vel", replan_stop_vel, 0.3);
//     const double start_goal_xy = (goal_pt.head<2>() - start_pt.head<2>()).norm();
//     if (!pm.receive_startpt && start_goal_xy < replan_stop_radius && start_vel.head<2>().norm() < replan_stop_vel) {
//         pm.receive_startpt = false;
//         nav_msgs::Path empty_path;
//         empty_path.header.frame_id = "world";
//         empty_path.header.stamp = ros::Time::now();
//         path_pub.publish(empty_path);
//         kino_path_pub.publish(empty_path);

//         visualization_msgs::Marker del;
//         del.header.frame_id = "world";
//         del.header.stamp = ros::Time::now();
//         del.ns = "path_visualization";
//         std::hash<std::string> hash_fn;
//         del.id = static_cast<int>(hash_fn(pm.index));
//         del.action = visualization_msgs::Marker::DELETE;
//         path_vis.publish(del);

//         pm.last_traj.resize(0, 0);
//         pm.last_path.clear();
//         return;
//     }

//     pm.kino_path_finder_->reset();

//     ROS_WARN("[GVF][KINOCHK] start=(%.2f %.2f %.2f) occ=%d dist=%.3f",
//          start_pt.x(), start_pt.y(), start_pt.z(),
//          pm.sdf_map_->getInflateOccupancy(start_pt),
//          pm.sdf_map_->getDistance(start_pt));

//     ROS_WARN("[GVF][KINOCHK] goal =(%.2f %.2f %.2f) occ=%d dist=%.3f",
//             goal_pt.x(), goal_pt.y(), goal_pt.z(),
//             pm.sdf_map_->getInflateOccupancy(goal_pt),
//             pm.sdf_map_->getDistance(goal_pt));


//     int ret = pm.kino_path_finder_->search(start_pt, start_vel, start_acc,
//                                         goal_pt, end_vel,
//                                         /*init=*/false, /*dynamic=*/false);

//     if (ret == KinodynamicAstar::NO_PATH) {
//         pm.receive_startpt = false;
//         ROS_WARN("[GVF] KinoA*: NO_PATH");
//         return;
//     }

//     double ts;
//     ros::param::param("~gvf/kino_sample_ts", ts, 0.2);
//     double ts_min;
//     ros::param::param("~gvf/kino_sample_ts_min", ts_min, 0.05);
//     if (ts < ts_min) ts = ts_min;

//     std::vector<Eigen::Vector3d> guide_pts;
//     std::vector<Eigen::Vector3d> boundary;  // [v0, vT, a0, aT]
//     pm.kino_path_finder_->getSamples(ts, guide_pts, boundary);

//     ROS_INFO("[GVF][KINO] ts=%.3f, guide_pts=%zu, v0=(%.2f %.2f %.2f), vT=(%.2f %.2f %.2f), a0=(%.2f %.2f %.2f), aT=(%.2f %.2f %.2f)",
//          ts, guide_pts.size(),
//          boundary[0].x(), boundary[0].y(), boundary[0].z(),
//          boundary[1].x(), boundary[1].y(), boundary[1].z(),
//          boundary[2].x(), boundary[2].y(), boundary[2].z(),
//          boundary[3].x(), boundary[3].y(), boundary[3].z());


//     if (guide_pts.size() < 3 || boundary.size() < 4) {
//         ROS_WARN("[GVF] KinoA*: samples too small");
//         return;
//     }

//     // 限制 guide points 数量，避免 B 样条时间跨度/采样点数爆炸
//     int max_guide_pts;
//     ros::param::param("~gvf/kino_max_guide_pts", max_guide_pts, 60);
//     if ((int)guide_pts.size() > max_guide_pts && max_guide_pts >= 3) {
//         std::vector<Eigen::Vector3d> ds;
//         ds.reserve(max_guide_pts);
//         ds.push_back(guide_pts.front());
//         for (int i = 1; i < max_guide_pts - 1; ++i) {
//             double r = (double)i / (double)(max_guide_pts - 1);
//             int idx = std::min((int)guide_pts.size() - 2,
//                                std::max(1, (int)std::llround(r * (guide_pts.size() - 1))));
//             ds.push_back(guide_pts[idx]);
//         }
//         ds.push_back(guide_pts.back());
//         guide_pts.swap(ds);
//     }

//     visualizePath(guide_pts, path_vis, pm.index);

//     std::vector<Eigen::Vector3d> initial_ctrl_ps = guide_pts;
//     Eigen::Vector3d end_pt = initial_ctrl_ps.back();

//     initial_state.row(0) = start_pt.transpose();
//     initial_state.row(1) = boundary[0].transpose();  // v0
//     initial_state.row(2) = boundary[2].transpose();  // a0

//     terminal_state.row(0) = end_pt.transpose();
//     terminal_state.row(1) = boundary[1].transpose(); // vT
//     terminal_state.row(2) = boundary[3].transpose(); // aT
//     /********************************************************************/

//     // initial_state <<    start_pt(0), start_pt(1), start_pt(2),
//     //                         0.0, 0.0,0.0,
//     //                         0.0, 0.0,0.0;
//     // terminal_state <<   end_pt(0), end_pt(1),end_pt(2),
//     //                         0.0, 0.0,0.0,
//     //                         0.0, 0.0,0.0;
//     pm.bspline_opt_->set3DPath2(initial_ctrl_ps);
//     pm.spline_->setIniandTerandCpsnum(initial_state,terminal_state,pm.bspline_opt_->cps_num_);


//     if(pm.bspline_opt_->cps_num_ <= 2*pm.spline_->p_)
//     {
//         // 如果控制点数量不足，发布path
//         if (pm.last_traj.rows() > 0) {
//             nav_msgs::Path path_msg;
//             path_msg.header.frame_id = "world";
//             path_msg.header.stamp = ros::Time::now();

//             for (const auto& pt : guide_pts) {
//                 geometry_msgs::PoseStamped pose;
//                 pose.pose.position.x = pt.x();
//                 pose.pose.position.y = pt.y();
//                 pose.pose.position.z = pt.z();
//                 path_msg.poses.push_back(pose);
//             }
//             path_pub.publish(path_msg);
//         }
//         return;
//     }
    
//     UniformBspline spline = *pm.spline_;
//     pm.bspline_opt_->setSplineParam(spline);
//     pm.bspline_opt_->optimize();

//     pm.spline_->setControlPoints(pm.bspline_opt_->control_points_);
//     pm.spline_->getT();
//     UniformBspline p = *pm.spline_;
//     UniformBspline v = p.getDerivative();
//     //traj
//     Eigen::MatrixXd p_ = p.getTrajectory(p.time_);
// 	Eigen::MatrixXd v_ = v.getTrajectory(p.time_);

// 		    /*********************************** */
// 			    struct MinDistEval {
// 			        double md;
// 			        bool reliable;        // 评估窗口内是否“完全可靠”（没有 unknown/out-of-map）
// 			        bool hit_occ;         // 是否碰到占据/膨胀占据（危险，直接当 md=0）
// 			        int reliable_pts;     // 窗口内可用 ESDF 的点数（unknown/out-of-map 不计入）
// 			    };
//                 // out-of-map：默认只标记 unreliable，避免局部地图边界制造“假紧急”
//                 int oob_hard_N;
//                 ros::param::param("~gvf/oob_hard_N", oob_hard_N, 4);
//                 double oob_hard_ratio;
//                 ros::param::param("~gvf/oob_hard_ratio", oob_hard_ratio, 0.35);

// 				    auto minDistFuture = [&](const Eigen::MatrixXd& traj, int start_idx, int K) -> MinDistEval {
// 				    // 重要：不要在遇到 unknown/out-of-map 时立刻 return。
// 				    // 真实场景里常见“前缀段是可靠的，远端进入未知区”，若直接 return 会导致 md=inf、new_unreliable=1，
// 				    // 进而触发 reject_unknown_keep_old / reject_unknown_in_caution_new_not_safe，最后卡死在旧轨迹尾段撞墙。
// 				    MinDistEval out{std::numeric_limits<double>::infinity(), true, false, 0};
// 				    int end = std::min((int)traj.rows(), start_idx + K);
//                     int oob_cnt = 0;
//                     int oob_total = 0;
//                     int total = 0;
// 				    for (int i = start_idx; i < end; ++i) {
//                         total++;
// 				        Eigen::Vector3d pt(traj(i,0), traj(i,1), traj(i,2));
// 				        if (!pm.sdf_map_->isInMap(pt)) {
// 				            out.reliable = false;
//                             oob_cnt++;
//                             oob_total++;
//                             const bool too_many_oob =
//                                 (oob_cnt >= std::max(1, oob_hard_N)) ||
//                                 (total > 0 && (double)oob_total / (double)total >= oob_hard_ratio);
//                             if (too_many_oob) {
// 				                out.hit_occ = true;
// 				                out.md = 0.0;
// 				                return out;
//                             }
//                             continue;
// 				        }
//                         oob_cnt = 0;

// 				        // 先用膨胀占据做硬判据（unknown 区域的距离值不可信，但占据是可信的）
// 				        const int occ = pm.sdf_map_->getInflateOccupancy(pt);
// 				        if (occ != 0) {
// 				            out.hit_occ = true;
// 			            out.md = 0.0;
// 			            return out;
// 			        }

// 			        // unknown：不更新 md，但允许把“前缀无占据”作为应急切换依据
// 			        if (pm.sdf_map_->isUnknown(pt)) { out.reliable = false; continue; }

// 			        out.reliable_pts++;
// 			        out.md = std::min(out.md, pm.sdf_map_->getDistance(pt));
// 			    }
// 			    if (out.reliable_pts == 0) {
// 			        out.md = std::numeric_limits<double>::infinity();
// 			    }
// 				    return out;
// 			    };

//                 // unknown/不可靠场景：仅 “前缀 occ-free” 太松，可能放行贴墙走廊导致擦墙/大曲率
//                 double safe_distance_small;
//                 ros::param::param("~gvf/switch_safe_distance_small", safe_distance_small, 0.28);
//                 double switch_occ_neigh_radius;
//                 ros::param::param("~gvf/switch_occ_neigh_radius", switch_occ_neigh_radius, 0.12);
//                 int switch_occ_neigh_samples;
//                 ros::param::param("~gvf/switch_occ_neigh_samples", switch_occ_neigh_samples, 6);

//                 auto occNeighborhoodClear = [&](const Eigen::Vector3d& pt) -> bool {
//                     if (!pm.sdf_map_) return true;
//                     if (!pm.sdf_map_->isInMap(pt)) return false;
//                     static const Eigen::Vector3d dirs[6] = {
//                         Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(-1, 0, 0),
//                         Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, -1, 0),
//                         Eigen::Vector3d(1, 1, 0).normalized(), Eigen::Vector3d(1, -1, 0).normalized()
//                     };
//                     const int n = std::max(0, std::min(6, switch_occ_neigh_samples));
//                     for (int i = 0; i < n; ++i) {
//                         Eigen::Vector3d q = pt + switch_occ_neigh_radius * dirs[i];
//                         if (!pm.sdf_map_->isInMap(q)) return false;
//                         if (pm.sdf_map_->getInflateOccupancy(q) != 0) return false;
//                     }
//                     return true;
//                 };

//                 auto prefixOccNeighborhoodClear = [&](const Eigen::MatrixXd& traj, int start_idx, int prefix_pts) -> bool {
//                     if (!pm.sdf_map_) return true;
//                     int end = std::min((int)traj.rows(), start_idx + std::max(1, prefix_pts));
//                     for (int i = start_idx; i < end; ++i) {
//                         Eigen::Vector3d pt(traj(i, 0), traj(i, 1), traj(i, 2));
//                         if (!occNeighborhoodClear(pt)) return false;
//                     }
//                     return true;
//                 };

//     // 发布“当前位置之后”的一段轨迹给 GVF/可视化（用于刷新 gvf::pathCallback 的局部缓冲区）
//     // 注意：即便拒绝换轨，也需要周期性发布旧轨迹，否则 GVF 的局部 buffer 会落在“上一帧相机位置”，
//     // 飞机走远后切向量取不到，表现为“飞着飞着飘离轨迹”。
//     auto publishTrajSegment = [&](const Eigen::MatrixXd& traj, const Eigen::MatrixXd& vel) {
//         if (traj.rows() <= 0) return;
//         if (vel.rows() != traj.rows()) return;

//         int future_pts;
//         ros::param::param("~gvf/path_pub_future_pts", future_pts, 200);
//         bool from_current;
//         ros::param::param("~gvf/path_pub_from_current", from_current, true);

//         int pub_i0 = 0;
//         if (from_current) {
//             pub_i0 = nearestIdxInTraj(traj, curr_pos);
//             pub_i0 = std::max(0, std::min(pub_i0, (int)traj.rows() - 1));
//         }
//         int pub_i1 = std::min((int)traj.rows(), pub_i0 + std::max(2, future_pts));

//         nav_msgs::Path path_msg;
//         path_msg.header.frame_id = "world";
//         path_msg.header.stamp = ros::Time::now();

//         for (int i = pub_i0; i < pub_i1; ++i) {
//             geometry_msgs::PoseStamped pose;
//             pose.pose.position.x = traj(i, 0);
//             pose.pose.position.y = traj(i, 1);
//             pose.pose.position.z = traj(i, 2);
//             pose.pose.orientation.x = vel(i, 0);
//             pose.pose.orientation.y = vel(i, 1);
//             pose.pose.orientation.z = vel(i, 2);
//             pose.pose.orientation.w = 1.0;
//             path_msg.poses.push_back(pose);
//         }

//         path_pub.publish(path_msg);
//     };

//     int K;
//     ros::param::param("~gvf/switch_eval_horizon_pts", K, 200);
//     double hyster;
//     ros::param::param("~gvf/switch_clearance_hysteresis", hyster, 0.15);
//     double switch_max_track_err;
//     ros::param::param("~gvf/switch_max_track_error", switch_max_track_err, 0.8);
//     double switch_unknown_dist;
//     ros::param::param("~gvf/switch_unknown_distance", switch_unknown_dist, 50.0);
//     double switch_goal_progress_margin;
//     ros::param::param("~gvf/switch_goal_progress_margin", switch_goal_progress_margin, 0.3);
//     double switch_min_hold_time;
//     ros::param::param("~gvf/switch_min_hold_time", switch_min_hold_time, 0.8);

//     int confirm_goal_progress;
//     ros::param::param("~gvf/switch_confirm_goal_progress", confirm_goal_progress, 2);
// 	    int confirm_track_error;
// 	    ros::param::param("~gvf/switch_confirm_track_error", confirm_track_error, 2);
// 		    int confirm_near_end;
// 		    ros::param::param("~gvf/switch_confirm_near_end", confirm_near_end, 1);
// 		    int switch_prefix_min_reliable_pts;
// 		    ros::param::param("~gvf/switch_prefix_min_reliable_pts", switch_prefix_min_reliable_pts, 6);
// 		    int switch_near_end_hard_pts;
// 		    ros::param::param("~gvf/switch_near_end_hard_pts", switch_near_end_hard_pts, 5);
// 	    double switch_topo_dev_thr;
// 	    ros::param::param("~gvf/switch_topo_dev_thr", switch_topo_dev_thr, 0.8);
//     double switch_topo_clearance_margin;
//     ros::param::param("~gvf/switch_topo_clearance_margin", switch_topo_clearance_margin, 0.25);
//     int switch_topo_cmp_pts;
//     ros::param::param("~gvf/switch_topo_cmp_pts", switch_topo_cmp_pts, 60);
//     double switch_caution_margin;
//     ros::param::param("~gvf/switch_caution_margin", switch_caution_margin, 0.10);
// 		    double switch_progress_clearance_slack;
// 		    ros::param::param("~gvf/switch_progress_clearance_slack", switch_progress_clearance_slack, 0.05);
// 		    int switch_emergency_prefix_pts;
// 		    ros::param::param("~gvf/switch_emergency_prefix_pts", switch_emergency_prefix_pts, 20);
// 		    int switch_topo_side_ahead_pts;
// 		    ros::param::param("~gvf/switch_topo_side_ahead_pts", switch_topo_side_ahead_pts, 25);
// 		    double switch_emergency_improve_delta;
// 		    ros::param::param("~gvf/switch_emergency_improve_delta", switch_emergency_improve_delta, 0.05);

// 	    bool accept_new = true;
// 	    bool force_accept = pm.receive_startpt;   // goalCallback 会置 true
// 	    std::string switch_reason = "no_old_traj";

//     // 迟滞“永远生效”：只要存在旧轨迹，就用稳定优先的判据决定是否接受新轨迹；
//     // enable_trajectory_concatenation 只影响“发布段拼接”，不影响换轨判据。
// 	    if (pm.last_traj.rows() > 0) {
// 	        switch_reason = "default_accept";
// 	        int old_i0 = 0;
// 	        int new_i0 = 0;
// 	        int remain_old = 0;

// 		        // old_i0 使用单调的 current_traj_index_（execTimerCallback 已在重规划前更新），避免最近点来回跳
// 		        old_i0 = std::max(0, std::min(current_traj_index_, (int)pm.last_traj.rows() - 1));
//                 // new_i0 用“窗口最近点”去抖：避免对称/平坦段 nearestIdxInTraj() 在相邻点间抖动
//                 int new_i0_window;
//                 ros::param::param("~gvf/switch_new_i0_window", new_i0_window, 20);
//                 auto nearestIdxInTrajWindow = [&](const Eigen::MatrixXd& traj, const Eigen::Vector3d& pos, int center, int window) {
//                     int c = std::max(0, std::min(center, (int)traj.rows() - 1));
//                     int l = std::max(0, c - std::max(0, window));
//                     int r = std::min((int)traj.rows() - 1, c + std::max(0, window));
//                     int best = c;
//                     double best_d = std::numeric_limits<double>::max();
//                     for (int i = l; i <= r; ++i) {
//                         Eigen::Vector3d p(traj(i, 0), traj(i, 1), traj(i, 2));
//                         double d = (p - pos).squaredNorm();
//                         if (d < best_d) { best_d = d; best = i; }
//                     }
//                     return best;
//                 };
//                 if (has_last_new_i0_) new_i0 = nearestIdxInTrajWindow(p_, curr_pos, last_new_i0_, new_i0_window);
//                 else new_i0 = nearestIdxInTraj(p_, curr_pos);
//                 last_new_i0_ = new_i0;
//                 has_last_new_i0_ = true;
// 		        remain_old = pm.last_traj.rows() - old_i0;

// 	        MinDistEval old_eval = minDistFuture(pm.last_traj, old_i0, K);
// 	        MinDistEval new_eval = minDistFuture(p_,          new_i0, K);
// 	        double old_md = old_eval.md;
// 	        double new_md = new_eval.md;
// 	        double old_track_err = (pm.last_traj.row(old_i0).transpose() - curr_pos).norm();
// 	        double old_end_goal = (pm.last_traj.row(pm.last_traj.rows() - 1).transpose() - goal_pt).head<2>().norm();
// 	        double new_end_goal = (p_.row(p_.rows() - 1).transpose() - goal_pt).head<2>().norm();

// 		        const bool hold_time_passed =
// 		            (last_switch_time_.isZero() || (ros::Time::now() - last_switch_time_).toSec() >= switch_min_hold_time);
// 		        const bool old_unreliable = (!old_eval.reliable) || (old_md > switch_unknown_dist);
// 		        const bool new_unreliable = (!new_eval.reliable) || (new_md > switch_unknown_dist);
// 		        // unsafe/caution 判据不要依赖 “完全可靠”，否则出现“部分 unknown 导致 reliable=false，unsafe 被绕过”
// 		        const bool old_unsafe = old_eval.hit_occ || (old_eval.reliable_pts > 0 && old_md < safe_distance_);
// 		        const bool new_unsafe = new_eval.hit_occ || (new_eval.reliable_pts > 0 && new_md < safe_distance_);
// 		        const bool old_in_caution = old_eval.hit_occ || (old_eval.reliable_pts > 0 && old_md < safe_distance_ + switch_caution_margin);
// 		        const double dist_curr = pm.sdf_map_ ? pm.sdf_map_->getDistance(curr_pos) : std::numeric_limits<double>::infinity();
// 		        const int occ_curr = pm.sdf_map_ ? pm.sdf_map_->getInflateOccupancy(curr_pos) : 0;
// 		        const bool in_caution_now = (occ_curr != 0) || (dist_curr < safe_distance_ + switch_caution_margin);
// 		        const bool allow_count = (hold_time_passed || old_in_caution || old_unsafe || in_caution_now);
// 		        const bool in_collision_now = (occ_curr != 0) || (dist_curr < std::max(1e-3, collision_threshold_));
// 		        const double prefix_safe_thr = safe_distance_;

// 			        auto prefixOk = [&](const MinDistEval& e, double thr) -> bool {
// 			            return (!e.hit_occ) &&
// 			                   (e.reliable_pts >= std::max(1, switch_prefix_min_reliable_pts)) &&
// 			                   (e.md >= thr);
// 			        };

//                     // unknown/unreliable 下：occ-free 必须，再加一层（可靠距离>=safe_small 或 occ 邻域也为空）
//                     auto prefixOkUnreliableTight = [&](const Eigen::MatrixXd& traj, int i0, int prefix_pts, const MinDistEval& pref) -> bool {
//                         if (pref.hit_occ) return false;
//                         const bool dist_ok_small =
//                             (pref.reliable_pts >= std::max(1, switch_prefix_min_reliable_pts)) &&
//                             (pref.md >= safe_distance_small);
//                         const bool neigh_ok = prefixOccNeighborhoodClear(traj, i0, prefix_pts);
//                         return dist_ok_small || neigh_ok;
//                     };

// 	        // “换边/拓扑切换”启发式：如果新轨迹在未来一段与旧轨迹偏差很大，
// 	        // 视作可能左右换边，此时需要更强证据才允许切换（显著更安全/确认的进度更好）。
// 	        double topo_max_dev = 0.0;
// 	        bool topo_flip = false;
// 	        int topo_side_old = 0;
// 	        int topo_side_new = 0;
// 	        {
// 	            int max_cmp = std::min({switch_topo_cmp_pts, remain_old, (int)p_.rows() - new_i0, (int)pm.last_traj.rows() - old_i0});
// 	            for (int k = 0; k < max_cmp; ++k) {
// 	                Eigen::Vector2d old_pt(pm.last_traj(old_i0 + k, 0), pm.last_traj(old_i0 + k, 1));
// 	                Eigen::Vector2d new_pt(p_(new_i0 + k, 0), p_(new_i0 + k, 1));
// 	                topo_max_dev = std::max(topo_max_dev, (old_pt - new_pt).norm());
// 	            }
// 	            topo_flip = (topo_max_dev > switch_topo_dev_thr);

// 	            auto sideSign = [&](const Eigen::MatrixXd& traj, int i0) -> int {
// 	                int ia = std::min((int)traj.rows() - 1, i0 + std::max(1, switch_topo_side_ahead_pts));
// 	                Eigen::Vector2d d(traj(ia, 0) - curr_pos.x(), traj(ia, 1) - curr_pos.y());
// 	                Eigen::Vector2d g(goal_pt.x() - curr_pos.x(), goal_pt.y() - curr_pos.y());
// 	                if (g.norm() < 1e-3 || d.norm() < 1e-3) return 0;
// 	                double z = g.x() * d.y() - g.y() * d.x();
// 	                if (std::abs(z) < 1e-6) return 0;
// 	                return z > 0.0 ? 1 : -1;
// 	            };
// 	            topo_side_old = sideSign(pm.last_traj, old_i0);
// 	            topo_side_new = sideSign(p_, new_i0);
// 	            if (topo_side_old != 0 && topo_side_new != 0 && topo_side_old != topo_side_new) {
// 	                topo_flip = true;
// 	            }
// 	        }

// 		        if (force_accept) {
// 		            accept_new = true;
// 		            switch_reason = "force_accept(receive_startpt)";
// 		        } else {
// 			            // 旧轨迹快结束时的硬兜底：不要卡在 remain_old≈0 导致“无轨迹可跟 → 撞墙/停住”
// 			            if (remain_old <= std::max(1, switch_near_end_hard_pts)) {
// 			                MinDistEval new_pref = minDistFuture(p_, new_i0, std::max(1, switch_emergency_prefix_pts));
// 			                const bool new_pref_clear = (!new_pref.hit_occ);
// 			                const bool new_pref_ok =
// 			                    (!new_unsafe) &&
// 			                    (new_unreliable
// 			                        ? (new_pref_clear && prefixOkUnreliableTight(p_, new_i0, switch_emergency_prefix_pts, new_pref))
// 			                        : prefixOk(new_pref, safe_distance_ + switch_caution_margin));
// 			                // near_end 解决“必须有轨迹”，但仍限制换边：旧轨迹仍安全时不要硬兜底直接 topo_flip
// 			                const bool allow_topo_in_near_end = (!topo_flip) || old_unsafe || old_in_caution || in_caution_now;
// 			                if (new_pref_ok && allow_topo_in_near_end) {
// 			                    accept_new = true;
// 			                    switch_reason = "accept_old_near_end_emergency";
// 			                }
// 			            }

// 		            if (switch_reason == "accept_old_near_end_emergency") {
// 		                // 已经决定切换，不再进入后续分支
// 		            } else
// 			            if (old_unsafe) {
// 			                // 旧轨迹已不安全：必须重规划，但不允许切到“同样不安全/不可靠”的新轨迹
// 			                // 同时避免在贴墙时刻突然“换边”引发大曲率/撞墙：要求新轨迹前缀段足够安全
// 				                MinDistEval new_pref = minDistFuture(p_, new_i0, std::max(1, switch_emergency_prefix_pts));
// 						                const bool new_pref_clear = (!new_pref.hit_occ);
// 						                // old_unsafe 时，ESDF 可能仍然不可靠；此时用“前缀不占据(occ)”作为主要安全判据，
// 						                // 距离只在可靠时作为附加条件（避免卡死在墙边持续 NO_PATH）。
// 						                const bool new_pref_ok =
// 						                    (!new_unsafe) &&
// 						                    (new_unreliable
// 						                        ? (new_pref_clear && prefixOkUnreliableTight(p_, new_i0, switch_emergency_prefix_pts, new_pref))
// 						                        : prefixOk(new_pref, prefix_safe_thr));
// 			                // 这里不再用 new_unreliable 直接拒绝：很多时候“远处未知”，但前缀段是可靠且安全的，
// 			                // 此时继续死守旧轨迹反而会把飞机送进墙边（尤其 collision-trigger 的时刻）。
// 			                if (!new_pref_ok) {
// 			                    accept_new = false;
// 			                    switch_reason = "reject_new_unsafe_even_old_unsafe";
// 			                } else if (topo_flip && new_md < safe_distance_ + switch_caution_margin) {
// 			                    accept_new = false;
// 			                    switch_reason = "reject_topo_flip_when_old_unsafe";
// 	                } else {
// 	                    accept_new = true;
// 	                    switch_reason = "accept_old_unsafe";
// 	                }
// 		            } else if (remain_old <= 30) {
// 			                // 旧轨迹快结束：即便新轨迹远端存在 unknown，
// 			                // 只要“前缀段”在 ESDF 里有足够可靠点且足够安全，也允许切换。
// 			                MinDistEval new_pref = minDistFuture(p_, new_i0, std::max(1, switch_emergency_prefix_pts));
// 					                const bool new_pref_clear = (!new_pref.hit_occ);
// 					                const bool new_pref_ok =
// 					                    (!new_unsafe) &&
// 					                    (new_unreliable
// 					                        ? (new_pref_clear && prefixOkUnreliableTight(p_, new_i0, switch_emergency_prefix_pts, new_pref))
// 					                        : prefixOk(new_pref, prefix_safe_thr));
// 					                if (allow_count && new_pref_ok) {
// 					                    switch_confirm_near_end_cnt_++;
// 					                } else {
// 					                    switch_confirm_near_end_cnt_ = 0;
// 					                }
// 	                if (switch_confirm_near_end_cnt_ >= std::max(1, confirm_near_end)) {
// 	                    accept_new = true;                 // 旧的快结束：允许换（但要求新轨迹不低于 safe_distance_）
// 	                    switch_reason = "accept_old_near_end";
// 	                } else {
// 	                    accept_new = false;
// 	                    switch_reason = "reject_near_end_wait_confirm";
// 	                }
// 	            } else if (old_track_err > switch_max_track_err) {
// 	                if (allow_count && !new_unreliable && !new_unsafe) {
// 	                    switch_confirm_track_error_cnt_++;
// 	                } else {
// 	                    switch_confirm_track_error_cnt_ = 0;
// 	                }
// 	                if (switch_confirm_track_error_cnt_ >= std::max(1, confirm_track_error)) {
// 	                    accept_new = true;                 // 已经明显偏离旧轨迹：允许换（但要求新轨迹不低于 safe_distance_）
// 	                    switch_reason = "accept_track_error_too_large";
// 	                } else {
// 	                    accept_new = false;
// 	                    switch_reason = "reject_track_error_wait_confirm";
// 	                }
// 			            } else if (old_unreliable || new_unreliable) {
// 			                // ESDF距离失真/未知区域：不要无条件切换；只在“显著收益”或“确认触发”时切换
// 				                if (in_caution_now) {
// 				                    // 已经贴墙/进入预警区：unknown 下不能死守旧轨迹，否则会“贴墙→NO_PATH→继续走→撞墙”
// 					                    MinDistEval old_pref = minDistFuture(pm.last_traj, old_i0, std::max(1, switch_emergency_prefix_pts));
// 					                    MinDistEval new_pref = minDistFuture(p_, new_i0, std::max(1, switch_emergency_prefix_pts));
// 					                    const bool new_pref_clear = (!new_pref.hit_occ);
// 					                    // unknown/不可靠区域：以“前缀不占据(occ)”作为主要判据，避免因为 reliable_pts/距离不足而卡死不换
// 					                    const bool new_pref_ok =
// 					                        (!new_unsafe) &&
// 					                        (new_unreliable
// 					                            ? (new_pref_clear && prefixOkUnreliableTight(p_, new_i0, switch_emergency_prefix_pts, new_pref))
// 					                            : prefixOk(new_pref, prefix_safe_thr));
// 				                    // 贴墙/碰撞时必须“更安全”才换：至少比当前位置距离更大，且（若可比）比旧前缀更大
// 				                    const bool improves_over_curr =
// 				                        (new_pref.reliable_pts > 0) ? (new_pref.md >= dist_curr + switch_emergency_improve_delta) : new_pref_clear;
// 				                    const bool improves_over_old =
// 				                        (old_pref.reliable_pts > 0 && new_pref.reliable_pts > 0)
// 				                            ? (new_pref.md >= old_pref.md + switch_emergency_improve_delta)
// 				                            : improves_over_curr;
// 				                    const bool emergency_allow = (in_collision_now || old_in_caution || old_unsafe);
// 				                    if (new_pref_ok) {
// 				                        if (emergency_allow && improves_over_old) {
// 				                            accept_new = true;
// 			                            switch_reason = "accept_unknown_in_caution";
// 			                        } else if (!emergency_allow && hold_time_passed) {
// 			                            // 非紧急：也允许切，但仍要求不比当前位置更差
// 			                            accept_new = improves_over_curr;
// 			                            switch_reason = accept_new ? "accept_unknown_in_caution" : "reject_unknown_in_caution_new_not_safe";
// 			                        } else {
// 			                            accept_new = false;
// 			                            switch_reason = "reject_unknown_in_caution_new_not_safe";
// 			                        }
// 			                    } else {
// 		                        accept_new = false;
// 		                        switch_reason = "reject_unknown_in_caution_new_not_safe";
// 		                    }
// 				                } else {
// 				                    const bool progress_better = (new_end_goal + switch_goal_progress_margin < old_end_goal);
// 					                    // unknown 下的进度切换：要求前缀安全（否则会“进度更好但贴墙/擦墙”）
// 					                    MinDistEval new_pref = minDistFuture(p_, new_i0, std::max(1, switch_emergency_prefix_pts));
// 					                    const bool new_pref_clear = (!new_pref.hit_occ);
// 					                    const bool new_pref_ok =
// 					                        (!new_unsafe) &&
// 					                        (new_unreliable
// 					                            ? (new_pref_clear && prefixOkUnreliableTight(p_, new_i0, switch_emergency_prefix_pts, new_pref))
// 					                            : prefixOk(new_pref, prefix_safe_thr));
// 				                    if (allow_count && progress_better && new_pref_ok) {
// 				                        switch_confirm_goal_progress_cnt_++;
// 				                    } else {
// 				                        switch_confirm_goal_progress_cnt_ = 0;
// 		                    }
// 	                    if (progress_better && switch_confirm_goal_progress_cnt_ >= std::max(1, confirm_goal_progress) && hold_time_passed) {
// 	                        accept_new = true;
// 	                        switch_reason = "accept_unknown_progress_confirmed";
// 	                    } else {
// 	                        accept_new = false;
// 	                        switch_reason = "reject_unknown_keep_old";
// 	                    }
// 	                }
// 	            } else if (new_end_goal + switch_goal_progress_margin < old_end_goal) {
// 	                const bool clearance_not_worse = (new_md + switch_progress_clearance_slack >= old_md);
// 	                if (allow_count && !new_unsafe && clearance_not_worse) {
// 	                    switch_confirm_goal_progress_cnt_++;
// 	                } else {
// 	                    switch_confirm_goal_progress_cnt_ = 0;
// 	                }
// 	                if (switch_confirm_goal_progress_cnt_ >= std::max(1, confirm_goal_progress)) {
// 	                    accept_new = true;                 // 新轨迹明显更接近目标：允许换（要求新轨迹不低于 safe_distance_）
// 	                    switch_reason = "accept_goal_progress_better";
// 	                } else {
// 	                    accept_new = false;
// 	                    switch_reason = "reject_goal_progress_wait_confirm";
// 	                }
// 	            } else if (new_md < old_md + hyster) {
// 	                accept_new = false;                // 旧的够安全且新的没“明显更安全”：不换（抑制左右来回切）
// 	                switch_reason = "reject_hysteresis_keep_old";
// 	            } else {
// 	                accept_new = true;
// 	                switch_reason = "accept_new_clearer_than_old";
// 	            }
// 	        }

// 	        // 非紧急情况下的最小保持时间（抗左右来回切换）
// 	        // 注意：当旧轨迹已经进入“预警区”（离墙很近但尚未 unsafe）时，允许提前换以避免撞墙
// 	        if (accept_new && !old_unsafe && !hold_time_passed && !old_in_caution) {
// 	            accept_new = false;
// 	            switch_reason = "reject_hold_time_keep_old";
// 	        }
// 	        // hold time 阻止时不累积确认计数（避免“延迟触发”）
// 	        if (!hold_time_passed && !old_in_caution && !old_unsafe) {
// 	            switch_confirm_goal_progress_cnt_ = 0;
// 	            switch_confirm_track_error_cnt_ = 0;
// 	            switch_confirm_near_end_cnt_ = 0;
// 	        }

// 	        // 拓扑一致性门控：旧轨迹仍安全时，不允许轻易换到“差异很大”的拓扑，
// 	        // 除非新轨迹显著更安全（clearance）或属于“已确认”的目标进度更好。
// 			        if (accept_new && topo_flip && !old_unsafe) {
// 			            // 注意：old_md/new_md 可能为 inf（大量 unknown），此时不能用 “inf >= inf” 绕过门控
// 			            const bool clearer_enough =
// 			                (!new_unsafe &&
// 			                 old_eval.reliable_pts > 0 && new_eval.reliable_pts > 0 &&
// 			                 std::isfinite(old_md) && std::isfinite(new_md) &&
// 			                 (new_md >= old_md + switch_topo_clearance_margin));
// 			            const bool progress_accept =
// 			                (switch_reason == "accept_goal_progress_better" ||
// 			                 switch_reason == "accept_unknown_progress_confirmed" ||
// 			                 switch_reason == "accept_unknown_in_caution" ||
// 			                 switch_reason == "accept_old_near_end_emergency");
// 			            // unknown/unreliable 下用 md=inf 会过度放行 topo flip，这里只允许“可靠距离”参与 topo 放行
// 			            const bool progress_accept_safe =
// 			                (progress_accept && !new_unsafe && !new_unreliable &&
// 			                 std::isfinite(new_md) && new_md >= safe_distance_ + switch_caution_margin);
// 			            if (!clearer_enough && !progress_accept_safe) {
// 			                accept_new = false;
// 			                switch_reason = "reject_topology_flip_keep_old";
// 		            }
// 		        }

// 	        ROS_INFO_THROTTLE(
// 	            1.0,
// 	            "[GVF][SWITCHDBG] accept=%d reason=%s old_md=%.3f new_md=%.3f safe=%.3f hyster=%.3f "
// 	            "old_end=%.3f new_end=%.3f prog_margin=%.3f remain_old=%d old_err=%.3f err_th=%.3f K=%d topo_flip=%d topo_dev=%.3f topo_side=%d->%d",
// 	            (int)accept_new, switch_reason.c_str(), old_md, new_md, safe_distance_, hyster, old_end_goal,
// 	            new_end_goal, switch_goal_progress_margin, remain_old, old_track_err, switch_max_track_err, K,
// 	            (int)topo_flip, topo_max_dev, topo_side_old, topo_side_new);
// 	    }

//     if (!accept_new) {
//         pm.receive_startpt = false;

//         ROS_WARN_THROTTLE(1.0, "[GVF][SWITCH] reject new traj (old safe), keep old");

//         // 迟滞拒绝时：保持旧轨迹，但仍然发布旧轨迹段以刷新 GVF 的局部缓存
//         publishTrajSegment(pm.last_traj, pm.last_vel);

//         return;
//     }
//     // 接受新轨迹：重置确认计数
//     switch_confirm_goal_progress_cnt_ = 0;
//     switch_confirm_track_error_cnt_ = 0;
//     switch_confirm_near_end_cnt_ = 0;

//     // ===== 发布段融合：保留旧轨迹未来 M 点，再接新轨迹（仅用于发布/控制平滑）=====
//     int blend_pts;
//     ros::param::param("~gvf/trajectory_blend_points", blend_pts, 10);

//     if (enable_trajectory_concatenation_ && pm.last_traj.rows() > 0 && concat_old_i0 >= 0 && concat_keep >= 2 &&
//         pm.last_vel.rows() == pm.last_traj.rows() && p_.rows() > 1 && v_.rows() == p_.rows()) {
//         int old_i0 = std::max(0, concat_old_i0);
//         int keep = std::min(concat_keep, (int)pm.last_traj.rows() - old_i0);

//         Eigen::Vector3d keep_end = pm.last_traj.row(std::min((int)pm.last_traj.rows() - 1, old_i0 + keep - 1)).transpose();
//         int new_j0 = nearestIdxInTraj(p_, keep_end);
//         new_j0 = std::max(0, std::min(new_j0, (int)p_.rows() - 1));

//         double join_dist = (p_.row(new_j0).transpose() - keep_end).norm();
//         double max_join_dist;
//         ros::param::param("~gvf/concat_max_join_dist", max_join_dist, 0.8);

//         int append_len = (int)p_.rows() - new_j0;
//         if (keep >= 2 && append_len >= 2 && join_dist <= max_join_dist) {
//             Eigen::MatrixXd fused_p(keep + append_len, 3);
//             Eigen::MatrixXd fused_v(keep + append_len, 3);

//             fused_p.topRows(keep) = pm.last_traj.middleRows(old_i0, keep);
//             fused_v.topRows(keep) = pm.last_vel.middleRows(old_i0, keep);
//             fused_p.bottomRows(append_len) = p_.middleRows(new_j0, append_len);
//             fused_v.bottomRows(append_len) = v_.middleRows(new_j0, append_len);

//             int b = std::min({blend_pts, keep, append_len});
//             for (int i = 0; i < b; ++i) {
//                 double a = (i + 1.0) / (b + 1.0);
//                 fused_p.row(keep - b + i) = (1 - a) * fused_p.row(keep - b + i) + a * fused_p.row(keep + i);
//                 fused_v.row(keep - b + i) = (1 - a) * fused_v.row(keep - b + i) + a * fused_v.row(keep + i);
//             }

//             p_ = fused_p;
//             v_ = fused_v;

//             ROS_INFO("[GVF][SPLICE] splice_mode=0 keep=%d new_j0=%d fused_rows=%d join=%.3f",
//                      keep, new_j0, (int)p_.rows(), join_dist);
//         } else {
//             ROS_WARN_THROTTLE(1.0, "[GVF][SPLICE] skip fusion: keep=%d append=%d join=%.3f (max=%.3f)",
//                               keep, append_len, join_dist, max_join_dist);
//         }
//     }
//     /*********************************** */


//     // //修改：防止实际左右摇摆，如果旧的路径还安全，并且新旧路径起始段偏差太大，就拒绝切换
//     // const double switch_thr = 0.30; //允许切换的最大偏差
//     // const double wz = 0.1; //z方向权重，考虑3D的距离
//     // const int L = 40; //比较前L个点


//     // auto weightedDist = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
//     //     Eigen::Vector3d d = a - b;
//     //     return std::sqrt(d.x()*d.x() + d.y()*d.y() + (wz*d.z())*(wz*d.z()));
//     // };

//     // auto nearestIdxInOldPath = [&](const nav_msgs::Path& path, const Eigen::Vector3d& pos) {
//     //     int best = 0;
//     //     double best_d = std::numeric_limits<double>::max();
//     //     for (int i = 0; i < (int)path.poses.size(); ++i) {
//     //         const auto& p = path.poses[i].pose.position;
//     //         Eigen::Vector3d pt(p.x, p.y, p.z);
//     //         double d = (pt - pos).squaredNorm();
//     //         if (d < best_d) { best_d = d; best = i; }
//     //     }
//     //     return best;
//     // };

//     // auto nearestIdxInTraj = [&](const Eigen::MatrixXd& traj, const Eigen::Vector3d& pos) {
//     //     int best = 0;
//     //     double best_d = std::numeric_limits<double>::max();
//     //     for (int i = 0; i < traj.rows(); ++i) {
//     //         Eigen::Vector3d pt(traj(i,0), traj(i,1), traj(i,2));
//     //         double d = (pt - pos).squaredNorm();
//     //         if (d < best_d) { best_d = d; best = i; }
//     //     }
//     //     return best;
//     // };

//     // // auto oldPathSafe = [&](const nav_msgs::Path& path, int start_idx) {
//     // //     // 旧路如果未来一小段要撞，就允许切换
//     // //     const int K = 40;
//     // //     const double safe_thr = collision_threshold_; // 你已有的阈值
//     // //     int end = std::min((int)path.poses.size(), start_idx + K);
//     // //     for (int i = start_idx; i < end; ++i) {
//     // //         const auto& p = path.poses[i].pose.position;
//     // //         Eigen::Vector3d pt(p.x, p.y, p.z);
//     // //         if (pm.sdf_map_->getDistance(pt) < safe_thr) return false;
//     // //     }
//     // //     return true;
//     // // };

//     // // auto oldPathSafe = [&](const nav_msgs::Path& path, int start_idx) {
//     // //     // 看更远一点，避免“末端在障碍里还被判安全”
//     // //     const int K = 200;  // 最多检查未来200个点（够用了，path一般不长）
//     // //     const double safe_thr = safe_distance_; // 用规划安全距离，而不是 collision_threshold_
//     // //     int end = std::min((int)path.poses.size(), start_idx + K);

//     // //     for (int i = start_idx; i < end; ++i) {
//     // //         const auto& p = path.poses[i].pose.position;
//     // //         Eigen::Vector3d pt(p.x, p.y, p.z);

//     // //         // inflate占据直接判不安全（更保守）
//     // //         if (pm.sdf_map_->getInflateOccupancy(pt) == 1) return false;

//     // //         // 距离场小于安全距离也判不安全
//     // //         if (pm.sdf_map_->getDistance(pt) < safe_thr) return false;
//     // //     }
//     // //     return true;
//     // // };
//     // auto oldPathMinDist = [&](const nav_msgs::Path& path, int start_idx) {
//     //     const int K = 200;
//     //     int end = std::min((int)path.poses.size(), start_idx + K);

//     //     double min_dist = std::numeric_limits<double>::infinity();
//     //     for (int i = start_idx; i < end; ++i) {
//     //         const auto& p = path.poses[i].pose.position;
//     //         Eigen::Vector3d pt(p.x, p.y, p.z);

//     //         if (pm.sdf_map_->getInflateOccupancy(pt) == 1) return 0.0;
//     //         min_dist = std::min(min_dist, pm.sdf_map_->getDistance(pt));
//     //     }
//     //     return min_dist;
//     // };



//     // const Eigen::Vector3d curr_pos(odom_.x(), odom_.y(), odom_.z());
//     // const nav_msgs::Path& old_path = pm.gvf_->last_path_;

//     // bool accept_new = true;
//     // if (!old_path.poses.empty()) {

//     //     int old_i0 = nearestIdxInOldPath(old_path, curr_pos);
//     //     int remain_old = (int)old_path.poses.size() - old_i0;

//     //     // 快到旧path末端时，不要死守旧路，强制接受新路
//     //     const int end_margin_pts = 20;
//     //     bool old_near_end = remain_old <= end_margin_pts;

//     //     const double safe_thr = safe_distance_;
//     //     const double caution_thr = safe_distance_ + 0.3;

//     //     double min_dist = oldPathMinDist(old_path, old_i0);

//     //     // 旧路不安全 or 快到末端：必须接受新路
//     //     if (min_dist < safe_thr || old_near_end) {
//     //         accept_new = true;
//     //     }
//     //     // 进入预警区：允许切换（不启用“max_dev 拒绝”）
//     //     else if (min_dist < caution_thr) {
//     //         accept_new = true;
//     //     }
//     //     // 足够安全：才启用“稳定优先”的 max_dev 拒绝逻辑
//     //     else {
//     //         int new_i0 = nearestIdxInTraj(p_, curr_pos);

//     //         int max_cmp = L;
//     //         max_cmp = std::min(max_cmp, remain_old);
//     //         max_cmp = std::min(max_cmp, (int)p_.rows() - new_i0);
//     //         if (max_cmp < 0) max_cmp = 0;

//     //         double max_dev = 0.0;
//     //         for (int k = 0; k < max_cmp; ++k) {
//     //             const auto& op = old_path.poses[old_i0 + k].pose.position;
//     //             Eigen::Vector3d old_pt(op.x, op.y, op.z);
//     //             Eigen::Vector3d new_pt(p_(new_i0 + k,0), p_(new_i0 + k,1), p_(new_i0 + k,2));
//     //             max_dev = std::max(max_dev, weightedDist(old_pt, new_pt));
//     //         }

//     //         if (max_dev > switch_thr) {
//     //             accept_new = false;
//     //             ROS_WARN_THROTTLE(1.0, "[GVF] Reject new path (max_dev=%.3f > %.3f), keep old", max_dev, switch_thr);
//     //         }
//     //     }


//     // }

//     // if (!accept_new) {
//     //     // 不覆盖 pm.last_traj / pm.last_vel，也不发布新 path
//     //     pm.receive_startpt = false;
//     //     return;
//     // }

//     ref_pos = odom_;
//     ref_initialized = true;

//     // 保存当前轨迹
//     pm.last_traj = p_;
//     pm.last_vel = v_;
//     current_traj_index_ = 0;

//     // 重置测试轨迹索引，从新轨迹的头开始执行
//     test_traj_index_ = 0;
//     ros::Time t4 = ros::Time::now();

//     // 只发布“当前位置之后”的一段轨迹用于 GVF/可视化。
//     // 否则 RViz 会一直显示“很长的轨迹”，即使已经接近目标点。
//     publishTrajSegment(p_, v_);
//     ros::Time t5 = ros::Time::now();

//     // ROS_INFO("Timing: A* search: %.3f ms, Traj opt: %.3f ms, Publish: %.3f ms",
//     //          (t3-t2).toSec()*1000, (t4-t3).toSec()*1000, (t5-t4).toSec()*1000);
    
//     // 重规划完成后，重置receive_startpt标志位
//     pm.receive_startpt = false;
//     last_switch_time_ = ros::Time::now();
// }
    void gvf_manager::publishPathMsg(
        const Eigen::MatrixXd& traj,
        const Eigen::MatrixXd& vel,
        const std::vector<double>* authoritative_w_samples)
    {
        if (traj.rows() <= 0) return;
        if (vel.rows() != traj.rows()) return;

        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "world";
        path_msg.header.stamp = ros::Time::now();

        for (int i = 0; i < traj.rows(); ++i) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = traj(i, 0);
            pose.pose.position.y = traj(i, 1);
            pose.pose.position.z = traj(i, 2);
            pose.pose.orientation.x = vel(i, 0);
            pose.pose.orientation.y = vel(i, 1);
            pose.pose.orientation.z = vel(i, 2);
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        if (!swarmParticlesManager.empty() && swarmParticlesManager[0].gvf_) {
            nav_msgs::Path::ConstPtr path_ptr(new nav_msgs::Path(path_msg));
            gvf* path_owner = swarmParticlesManager[0].gvf_.get();
            if (path_owner->authoritativePhaseMode()) {
                if (!authoritative_w_samples ||
                    authoritative_w_samples->size() !=
                        static_cast<size_t>(traj.rows())) {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[GVF][PATH_VIS] authoritative path publish missing matching w samples; keep existing mirror");
                    return;
                }
                // The legacy topic callback is intentionally rejected while
                // the manager owns the authoritative frontend.  Update the
                // visualization/reparam mirror through the explicit manager
                // API instead, preserving the same global phase samples used
                // by control and Tube ownership.
                path_owner->installAuthoritativePathMirror(
                    path_msg, *authoritative_w_samples);
            } else {
                path_owner->pathCallback(path_ptr);
            }
            const gvf::ReparamCacheSnapshot cache =
                path_owner->captureReparamCacheSnapshot();
            if (cache.ready && !cache.w.empty()) {
                const double start_w = cache.w.front();
                const double end_w = cache.w.back();
                const Eigen::Vector3d path_end = traj.row(traj.rows() - 1).transpose();
                const double pending_end_to_goal_dist = closed_ref_has_pending_goal_ ?
                    (path_end - pointFromClosedW(closed_ref_pending_goal_w_)).norm() : -1.0;
                const double accepted_end_to_goal_dist = closed_ref_has_accepted_goal_ ?
                    (path_end - pointFromClosedW(closed_ref_accepted_goal_w_)).norm() : -1.0;
                ROS_WARN("[GVF][REPARAM] start_w=%.3f end_w=%.3f path_w_len=%.3f path_points=%zu pending_goal_w=%.3f pending_lookahead_w=%.3f accepted_goal_w=%.3f accepted_lookahead_w=%.3f has_pending_goal=%d has_accepted_goal=%d pending_end_to_goal_dist=%.3f accepted_end_to_goal_dist=%.3f",
                         start_w, end_w, end_w - start_w,
                         cache.w.size(),
                         closed_ref_pending_goal_w_,
                         closed_ref_pending_lookahead_w_,
                         closed_ref_accepted_goal_w_,
                         closed_ref_accepted_lookahead_w_,
                         closed_ref_has_pending_goal_ ? 1 : 0,
                         closed_ref_has_accepted_goal_ ? 1 : 0,
                         pending_end_to_goal_dist,
                         accepted_end_to_goal_dist);
                const AuthoritativePhaseSnapshot phase =
                    captureAuthoritativePhase();
                if (unifiedPhaseV2Active() && phase.initialized) {
                    path_owner->setVisualizationProgressW(
                        phase.w);
                    ROS_WARN("[GVF][PHASE_V2][PATH] mode=%s phase_w=%.3f start_w=%.3f end_w=%.3f points=%zu",
                             closedPhaseV2Active() ? "closed" : "point",
                             phase.w, start_w, end_w,
                             cache.w.size());
                } else {
                    ensureProgressInCurrentPathRange(start_w, end_w);
                }
            }
        }

        path_pub.publish(path_msg);
    }

    // 计算轨迹切换评分
    double gvf_manager::cul_score(const Eigen::MatrixXd& traj,
                                    const Eigen::MatrixXd& vel,
                                    const Eigen::Vector3d& goal_pt,
                                    int i0,
                                    const Eigen::VectorXd& time)
    {
        auto& pm = swarmParticlesManager[0];
        const int N = std::min((int)traj.rows(), std::min((int)vel.rows(), (int)time.size()));
        if (N < 2) return std::numeric_limits<double>::infinity();

        const int start = std::max(1, std::min(i0 + 1, N - 1));

        double J_smooth = 0.0;
        double J_dist   = 0.0;

        Eigen::Vector3d traj_end(traj(traj.rows() - 1, 0), traj(traj.rows() - 1, 1),
                                      traj(traj.rows() - 1, 2));
        double J_goal = (traj_end - goal_pt).norm();

        for (int i = start; i < N; ++i) {
            double dt = time(i) - time(i - 1);
            if (dt <= 1e-6) continue;
            Eigen::Vector3d dv = (vel.row(i) - vel.row(i - 1)).transpose();
            Eigen::Vector3d a  = dv / dt;
            J_smooth += a.squaredNorm() * dt;
        }

        constexpr double d_eps = 0.05; // 5cm 防数值爆炸
        for (int i = start; i < N; ++i) {
            double dt = time(i) - time(i - 1);
            if (dt <= 1e-6) continue;

            Eigen::Vector3d pt = traj.row(i).transpose();
            if (!pm.sdf_map_ || !pm.sdf_map_->isInMap(pt)) {
                continue;
            }
            if (pm.sdf_map_->isUnknown(pt)) {
                continue;
            }

            double dist = pm.sdf_map_->getDistance(pt);
            double d = std::max(d_eps, dist);
            J_dist += (1.0 / d) * dt;
        }

        double J_total = 0.5 * J_smooth + 1.0 * J_dist + 0.05 * J_goal;
        return J_total;
    }


    bool gvf_manager::shouldAcceptCandidate(const Eigen::MatrixXd& old_traj, const Eigen::MatrixXd& old_vel,
                                            const Eigen::VectorXd& old_time, int old_i0,
                                            const Eigen::MatrixXd& new_traj, const Eigen::MatrixXd& new_vel,
                                            const Eigen::VectorXd& new_time, int new_i0,
                                            const Eigen::Vector3d& goal_pt, std::string& reason_out,
                                            double accepted_progress_w, double accepted_path_w_end)
    {
        bool accept_new = true;
        std::string switch_reason = "accept_default";
        double J_old = 0.0;
        double J_new = 0.0;

        reason_out = "accept_default";
        if (new_traj.rows() <= 0 || new_vel.rows() != new_traj.rows()) {
            switch_reason = "reject_empty_new";
            reason_out = switch_reason;
            return false;
        }
        if (old_traj.rows() <= 0 || old_vel.rows() != old_traj.rows()) {
            switch_reason = "accept_no_old_traj";
            reason_out = switch_reason;
            return true;
        }

        //检查旧轨迹是否发生碰撞，或者旧轨迹快结束
        const bool old_collision = checkCollision();
        const bool old_near_end = ((old_traj.rows() - 1 - old_i0) <= (old_traj.rows() - 1) / 2);
        const bool old_governor_path_short = shouldForceAcceptForGovernorPathShort(
            accepted_progress_w, accepted_path_w_end, cmd_governor_l_max_, switch_governor_path_margin_w_);
        if (old_collision || old_near_end)
        {
            logReplanReason(old_collision ? "collision" : "near_end");
            switch_reason = "accept_collision&timout";
            accept_new = true;
        } else if (old_governor_path_short) {
            logReplanReason("governor_path_short");
            switch_reason = "accept_governor_path_short";
            accept_new = true;
        } else {
            //旧轨迹仍然安全，检查新旧轨迹代价
            J_old = cul_score(old_traj, old_vel, goal_pt, old_i0, old_time);
            J_new = cul_score(new_traj, new_vel, goal_pt, new_i0, new_time);

            const double eps = 1e-6;
            const double rel_improve = (J_old - J_new) / std::max(eps, std::abs(J_old));
            // Accept anything that is not worse.  The old rule demanded a 10%
            // improvement, but a periodic replan of the *same* mission can only
            // manage a few percent (measured: J_old=1.685 vs J_new=1.601, 5%),
            // so ~99% of replans were rejected (850+ reject_worse_new per run,
            // only 5-7 installs) and the frontend ran dry -> the UAV froze with
            // a few metres of stale path left.  Accepting ties keeps the
            // frontend fresh; clearly better candidates still switch.
            if (rel_improve >= 0.0) {
                accept_new = true;
                switch_reason = (rel_improve > 0.1) ? "accept_better_new"
                                                    : "accept_not_worse";
            } else {
                accept_new = false;
                switch_reason = "reject_worse_new";
            }
        }

        ROS_INFO_THROTTLE(1.0,
            "\033[36m[GVF][SWITCH]\033[0m accept=%d reason=%s J_old=%.3f J_new=%.3f "
            "rel=%.3f old_coll=%d old_near_end=%d gov_short=%d "
            "progress_w=%.3f path_w_end=%.3f remain_w=%.3f d_goal=%.2f "
            "pos=(%.2f,%.2f) goal=(%.2f,%.2f) old_i0=%d new_i0=%d",
            (int)accept_new, switch_reason.c_str(), J_old, J_new,
            (J_old != 0.0) ? (J_old - J_new) / J_old : 0.0,
            (int)old_collision, (int)old_near_end,
            (int)old_governor_path_short,
            accepted_progress_w, accepted_path_w_end,
            accepted_path_w_end - accepted_progress_w,
            (goal_pt - odom_).norm(),
            odom_.x(), odom_.y(), goal_pt.x(), goal_pt.y(),
            old_i0, new_i0);

        reason_out = switch_reason;
        return accept_new;
    }

bool gvf_manager::planKinoToGoal(gvfManager& pm,
                                 const Eigen::Vector3d& start_pt,
                                 const Eigen::Vector3d& start_vel,
                                 const Eigen::Vector3d& start_acc,
                                 const Eigen::Vector3d& goal_pt,
                                 const Eigen::Vector3d& end_vel,
                                 KinoPlanSamples& samples)
{
    pm.kino_path_finder_->reset();
    int status = pm.kino_path_finder_->search(start_pt, start_vel, start_acc,
                                              goal_pt, end_vel,
                                              /*init=*/false, /*dynamic=*/false);
    if (status != KinodynamicAstar::NO_PATH) {
        cout << "[kino replan]: kinodynamic search success." << endl;
        samples.ts = std::max(kino_sample_ts_min_, kino_sample_ts_);
        pm.kino_path_finder_->getSamples(samples.ts, samples.point_set, samples.start_end_derivatives);
        return true;
    }

    cout << "[kino replan]: kinodynamic search fail!" << endl;
    pm.kino_path_finder_->reset();
    status = pm.kino_path_finder_->search(start_pt, start_vel, start_acc,
                                          goal_pt, end_vel,
                                          false, false);
    if (status == KinodynamicAstar::NO_PATH) {
        cout << "[kino replan]: Can't find path." << endl;
        return false;
    }

    cout << "[kino replan]: retry search success." << endl;
    samples.ts = std::max(kino_sample_ts_min_, kino_sample_ts_);
    pm.kino_path_finder_->getSamples(samples.ts, samples.point_set, samples.start_end_derivatives);
    return true;
}

bool gvf_manager::selectClosedPhaseGoal(
    gvfManager& pm,
    const Eigen::Vector3d& curr_pos,
    const Eigen::Vector3d& start_pt,
    const Eigen::Vector3d& start_vel,
    const Eigen::Vector3d& start_acc,
    Eigen::Vector3d& goal_pt,
    Eigen::Vector3d& end_vel,
    KinoPlanSamples& samples)
{
    const AuthoritativePhaseSnapshot phase = captureAuthoritativePhase();
    if (!closedPhaseV2Active() || !phase.initialized ||
        !circle_reference_ready_ || circle_reference_total_w_ <= 1e-9) {
        return false;
    }

    const double planning_phase_w = phase.w;
    const double min_delta_w = std::max(
        0.0, std::min(closed_ref_lookahead_min_w_, closed_ref_lookahead_max_w_));
    const double max_delta_w = std::max(
        min_delta_w, std::max(closed_ref_lookahead_min_w_, closed_ref_lookahead_max_w_));
    const double base_delta_w = std::max(
        min_delta_w, std::min(closed_goal_prefer_lookahead_w_, max_delta_w));

    const double check_step_w = std::max(1e-3, closed_goal_obstacle_check_step_w_);
    ClosedGoalObstacleInterval obstacle_interval;
    if (closed_goal_push_past_obstacle_ && pm.sdf_map_ && max_delta_w > 1e-6) {
        std::vector<int> occupancy;
        occupancy.reserve(static_cast<size_t>(std::ceil(max_delta_w / check_step_w)));
        for (double delta_w = check_step_w;
             delta_w <= max_delta_w + 1e-9;
             delta_w += check_step_w) {
            const Eigen::Vector3d ref_pt = pointFromClosedW(planning_phase_w + delta_w);
            if (!pm.sdf_map_->isInMap(ref_pt)) {
                occupancy.push_back(-1);
            } else {
                occupancy.push_back(pm.sdf_map_->getInflateOccupancy(ref_pt));
            }
        }
        obstacle_interval = detectClosedGoalObstacleInterval(occupancy, check_step_w, 3);
    }

    const double selected_delta_w = closedPhaseV2SelectedDeltaW(
        base_delta_w, max_delta_w, obstacle_interval.found_end,
        obstacle_interval.end_delta_w, closed_goal_obstacle_pass_margin_w_);
    const bool pushed_past_obstacle = selected_delta_w > base_delta_w + 1e-6;

    const double selected_goal_w = planning_phase_w + selected_delta_w;
    goal_pt = pointFromClosedW(selected_goal_w);
    end_vel = Eigen::Vector3d::Zero();

    if (!planKinoToGoal(pm, start_pt, start_vel, start_acc,
                        goal_pt, end_vel, samples)) {
        last_closed_goal_plan_success_ = false;
        closed_ref_has_pending_goal_ = false;
        closed_phase_has_pending_path_end_w_ = false;
        ROS_WARN("[GVF][CLOSED_PHASE_V2][GOAL] planner_success=0 phase_w=%.3f selected_delta_w=%.3f goal_w=%.3f base_delta_w=%.3f max_delta_w=%.3f obstacle_start_w=%.3f obstacle_end_w=%.3f pushed=%d reason=kino_failed",
                 planning_phase_w, selected_delta_w, selected_goal_w,
                 base_delta_w, max_delta_w,
                 obstacle_interval.found_start ? obstacle_interval.start_delta_w : -1.0,
                 obstacle_interval.found_end ? obstacle_interval.end_delta_w : -1.0,
                 pushed_past_obstacle ? 1 : 0);
        return false;
    }

    if (samples.point_set.empty()) {
        last_closed_goal_plan_success_ = false;
        closed_ref_has_pending_goal_ = false;
        closed_phase_has_pending_path_end_w_ = false;
        return false;
    }

    const Eigen::Vector3d actual_end = samples.point_set.back();
    const double projected_end_w = projectClosedLocal(
        actual_end, planning_phase_w, 0.0, max_delta_w);
    const double actual_end_delta_w = std::max(0.0, projected_end_w - planning_phase_w);
    const double end_to_goal_dist = (actual_end - goal_pt).norm();

    pm.goal_pt = goal_pt;
    closed_ref_pending_goal_w_ = selected_goal_w;
    closed_ref_pending_lookahead_w_ = selected_delta_w;
    closed_ref_has_pending_goal_ = true;
    closed_ref_pending_from_bypass_ = pushed_past_obstacle;
    closed_ref_last_goal_pos_ = goal_pt;
    closed_ref_last_goal_dist_xy_ = (goal_pt.head<2>() - curr_pos.head<2>()).norm();
    closed_ref_last_candidate_idx_ = -1;
    closed_ref_last_selected_goal_w_ = selected_goal_w;
    closed_ref_last_selected_lookahead_w_ = selected_delta_w;
    closed_ref_has_selected_goal_ = true;
    last_selected_goal_w_ = selected_goal_w;
    last_selected_lookahead_w_ = selected_delta_w;
    last_selected_goal_idx_ = -1;
    last_failed_goal_idx_ = -1;
    last_closed_goal_plan_success_ = true;

    closed_phase_pending_path_end_w_ =
        actual_end_delta_w > 1e-3 ? projected_end_w : selected_goal_w;
    closed_phase_has_pending_path_end_w_ = true;

    ROS_WARN("[GVF][CLOSED_PHASE_V2][GOAL] planner_success=1 phase_w=%.3f selected_delta_w=%.3f goal_w=%.3f base_delta_w=%.3f max_delta_w=%.3f obstacle_start_w=%.3f obstacle_end_w=%.3f pushed=%d actual_end_delta_w=%.3f end_to_goal_dist=%.3f kino_points=%zu",
             planning_phase_w, selected_delta_w, selected_goal_w,
             base_delta_w, max_delta_w,
             obstacle_interval.found_start ? obstacle_interval.start_delta_w : -1.0,
             obstacle_interval.found_end ? obstacle_interval.end_delta_w : -1.0,
             pushed_past_obstacle ? 1 : 0,
             actual_end_delta_w, end_to_goal_dist, samples.point_set.size());
    return true;
}

bool gvf_manager::selectClosedGoalCandidate(gvfManager& pm,
                                            const Eigen::Vector3d& curr_pos,
                                            const Eigen::Vector3d& start_pt,
                                            const Eigen::Vector3d& start_vel,
                                            const Eigen::Vector3d& start_acc,
                                            Eigen::Vector3d& goal_pt,
                                            Eigen::Vector3d& end_vel,
                                            KinoPlanSamples& samples)
{
    if (closedPhaseV2Active()) {
        return selectClosedPhaseGoal(pm, curr_pos, start_pt, start_vel,
                                       start_acc, goal_pt, end_vel, samples);
    }

    double planning_phase_w = closed_ref_w_;
    getCircleReferenceGoal(curr_pos);
    planning_phase_w = closed_ref_w_;
    end_vel = Eigen::Vector3d::Zero();

    const std::vector<double> candidates = buildClosedLookaheadCandidates();
    const int candidate_count = static_cast<int>(candidates.size());

    const double local_d_for_goal = (curr_pos - pointFromClosedW(planning_phase_w)).head<2>().norm();
    double tangent_dot_odom_for_goal = 0.0;
    const Eigen::Vector2d odom_v_xy = odom_vel_lpf_.head<2>();
    const double odom_v_norm = odom_v_xy.norm();
    if (odom_v_norm > 1e-6) {
        const Eigen::Vector2d tangent_xy = tangentFromClosedW(planning_phase_w).head<2>();
        const double tangent_norm = tangent_xy.norm();
        if (tangent_norm > 1e-6) {
            tangent_dot_odom_for_goal = (tangent_xy / tangent_norm).dot(odom_v_xy / odom_v_norm);
        }
    }

    const bool goal_recover_mode = closed_ref_enable_recover_ && closed_ref_recover_;
    const std::string mode_for_goal = goal_recover_mode ? "RECOVER" : "TRACK";
    double desired_lookahead = closed_goal_prefer_lookahead_w_;
    if (!candidates.empty()) {
        desired_lookahead = std::max(candidates.front(), std::min(desired_lookahead, candidates.back()));
    }
    double first_obstacle_delta_w = std::numeric_limits<double>::infinity();
    double obstacle_end_delta_w = std::numeric_limits<double>::infinity();
    double bypass_delta_w = std::numeric_limits<double>::infinity();
    ClosedGoalObstacleInterval obstacle_interval;

    if (closed_goal_push_past_obstacle_ && pm.sdf_map_ && !candidates.empty()) {
        const double check_step = std::max(1e-3, closed_goal_obstacle_check_step_w_);
        const double max_check_w = candidates.back();
        std::vector<int> occupancy;
        occupancy.reserve(static_cast<size_t>(std::ceil(max_check_w / check_step)));
        for (double delta_w = check_step; delta_w <= max_check_w + 1e-9; delta_w += check_step) {
            const Eigen::Vector3d ref_pt = pointFromClosedW(planning_phase_w + delta_w);
            if (!pm.sdf_map_->isInMap(ref_pt)) {
                occupancy.push_back(-1);
                continue;
            }
            occupancy.push_back(pm.sdf_map_->getInflateOccupancy(ref_pt));
        }

        obstacle_interval = detectClosedGoalObstacleInterval(occupancy, check_step, 3);
        first_obstacle_delta_w = obstacle_interval.start_delta_w;
        obstacle_end_delta_w = obstacle_interval.end_delta_w;
        if (obstacle_interval.found_end) {
            bypass_delta_w = std::min(max_check_w,
                obstacle_end_delta_w + std::max(0.0, closed_goal_obstacle_pass_margin_w_));
        }
    }
    const bool bypass_mode = obstacle_interval.found_end;
    const bool desired_pushed_by_obstacle = false;

    const double progressive_max_lookahead = closedGoalProgressiveMaxLookahead(
        closed_goal_prefer_lookahead_w_, candidates.empty() ? 0.0 : candidates.back(),
        closed_goal_progressive_lookahead_extra_w_, goal_recover_mode);
    const double required_progress_w = closedGoalRequiredProgress(
        odom_v_xy.norm(), closed_goal_progressive_progress_time_,
        closed_goal_progressive_min_progress_w_,
        closed_goal_progressive_max_progress_w_);

    std::string candidate_order_reason = closedGoalCandidateOrderReason(goal_recover_mode);
    std::vector<int> order;
    if (desired_pushed_by_obstacle) {
        candidate_order_reason += "_push_obstacle";
    }

    order.reserve(candidate_count);
    for (int i = 0; i < candidate_count; ++i) {
        if (goal_recover_mode || candidates[i] <= progressive_max_lookahead + 1e-6) {
            order.push_back(i);
        }
    }
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        return closedGoalAttemptComesBefore(
            candidates[lhs], lhs, candidates[rhs], rhs, desired_lookahead);
    });
    const int considered_candidate_count = static_cast<int>(order.size());

    std::ostringstream candidate_order_ss;
    candidate_order_ss << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < order.size(); ++i) {
        if (i > 0) candidate_order_ss << ",";
        candidate_order_ss << candidates[order[i]];
    }

    if (order.empty()) {
        const Eigen::Vector3d failed_goal_pos = pointFromClosedW(planning_phase_w);
        const double failed_goal_dist_xy = (failed_goal_pos.head<2>() - curr_pos.head<2>()).norm();
        ROS_WARN("[GVF][CLOSED_GOAL] curr_pos=(%.3f,%.3f,%.3f) goal_pos=(%.3f,%.3f,%.3f) goal_dist_xy=%.3f selected_lookahead=%.3f selected_delta_w=%.3f desired_lookahead=%.3f obstacle_delta_w=%.3f desired_pushed=%d selected_score=-1.000 closed_ref_w=%.3f selected_goal_w=%.3f candidate_count=%d considered_candidate_count=%d selected_idx=-1 planner_success=0 reason=no_candidates mode_for_goal=%s candidate_order_reason=%s local_d=%.3f tangent_dot_odom=%.3f candidate_order=\"\" full_success_tol=%.3f accepted_full_goal=0 accepted_partial_goal=0 selected_end_to_goal_dist=-1.000 tried_lookaheads=\"\" tried_end_to_goal_dists=\"\" obstacle_end_delta_w=%.3f bypass_delta_w=%.3f bypass_mode=%d selected_end_delta_w=-1.000 selected_passed_obstacle=0 tried_end_delta_ws=\"\" selection_mode=progressive progressive_max_lookahead_w=%.3f required_progress_w=%.3f selected_kino_path_length=-1.000 selected_progress_sufficient=0 tried_kino_path_lengths=\"\"",
                 curr_pos.x(), curr_pos.y(), curr_pos.z(),
                 failed_goal_pos.x(), failed_goal_pos.y(), failed_goal_pos.z(),
                 failed_goal_dist_xy, 0.0, 0.0, desired_lookahead,
                 std::isfinite(first_obstacle_delta_w) ? first_obstacle_delta_w : -1.0,
                 desired_pushed_by_obstacle ? 1 : 0,
                 planning_phase_w, planning_phase_w,
                 candidate_count, considered_candidate_count,
                 mode_for_goal.c_str(), candidate_order_reason.c_str(),
                 local_d_for_goal, tangent_dot_odom_for_goal,
                 closed_goal_full_success_tol_,
                 std::isfinite(obstacle_end_delta_w) ? obstacle_end_delta_w : -1.0,
                 std::isfinite(bypass_delta_w) ? bypass_delta_w : -1.0,
                 bypass_mode ? 1 : 0,
                 progressive_max_lookahead, required_progress_w);
        last_closed_goal_plan_success_ = false;
        return false;
    }

    bool planner_success = false;
    int selected_idx = order.front();
    double selected_lookahead = candidates[selected_idx];
    double selected_goal_w = planning_phase_w + selected_lookahead;
    double selected_end_to_goal_dist = std::numeric_limits<double>::infinity();
    bool accepted_full_goal = false;
    bool accepted_partial_goal = false;
    std::string selected_reason = "all_failed";
    double selected_end_delta_w = std::numeric_limits<double>::infinity();
    double selected_kino_path_length = std::numeric_limits<double>::infinity();
    bool selected_progress_sufficient = false;
    bool selected_passed_obstacle = false;

    struct ClosedGoalCandidateResult
    {
        int idx = -1;
        double lookahead = 0.0;
        Eigen::Vector3d goal = Eigen::Vector3d::Zero();
        double goal_w = 0.0;
        KinoPlanSamples samples;
        bool full_success = false;
        double kino_path_length = std::numeric_limits<double>::infinity();
        double end_to_goal_dist = std::numeric_limits<double>::infinity();
        double projected_end_w = 0.0;
        ClosedGoalCandidateProgress progress;
        ClosedGoalProgressiveCandidate progressive;
    };
    std::vector<ClosedGoalCandidateResult> valid_candidates;
    valid_candidates.reserve(order.size());

    std::ostringstream tried_lookaheads_ss;
    std::ostringstream tried_end_dists_ss;
    std::ostringstream tried_end_delta_ws_ss;
    std::ostringstream tried_kino_path_lengths_ss;
    tried_lookaheads_ss << std::fixed << std::setprecision(3);
    tried_end_dists_ss << std::fixed << std::setprecision(3);
    tried_end_delta_ws_ss << std::fixed << std::setprecision(3);
    tried_kino_path_lengths_ss << std::fixed << std::setprecision(3);

    for (int idx : order) {
        const double lookahead = candidates[idx];
        const double goal_w = planning_phase_w + lookahead;
        const Eigen::Vector3d candidate_goal = pointFromClosedW(goal_w);
        if (tried_lookaheads_ss.tellp() > 0) tried_lookaheads_ss << ",";
        tried_lookaheads_ss << lookahead;

        KinoPlanSamples candidate_samples;
        if (!planKinoToGoal(pm, start_pt, start_vel, start_acc, candidate_goal, end_vel, candidate_samples)) {
            if (tried_end_dists_ss.tellp() > 0) tried_end_dists_ss << ",";
            tried_end_dists_ss << "fail";
            if (tried_end_delta_ws_ss.tellp() > 0) tried_end_delta_ws_ss << ",";
            tried_end_delta_ws_ss << "fail";
            if (tried_kino_path_lengths_ss.tellp() > 0) tried_kino_path_lengths_ss << ",";
            tried_kino_path_lengths_ss << "fail";
            continue;
        }

        const bool sample_valid = !candidate_samples.point_set.empty() &&
                                  candidate_samples.start_end_derivatives.size() >= 3;
        const Eigen::Vector3d actual_end = sample_valid ?
            candidate_samples.point_set.back() :
            Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
        const double end_to_goal_dist = sample_valid ?
            (actual_end - candidate_goal).norm() :
            std::numeric_limits<double>::infinity();

        if (tried_end_dists_ss.tellp() > 0) tried_end_dists_ss << ",";
        tried_end_dists_ss << end_to_goal_dist;

        if (!sample_valid || !isFiniteClosedGoalCandidate(
                actual_end.x(), actual_end.y(), actual_end.z(),
                end_to_goal_dist, 0.0)) {
            if (tried_end_delta_ws_ss.tellp() > 0) tried_end_delta_ws_ss << ",";
            tried_end_delta_ws_ss << "invalid";
            if (tried_kino_path_lengths_ss.tellp() > 0) tried_kino_path_lengths_ss << ",";
            tried_kino_path_lengths_ss << "invalid";
            continue;
        }

        double kino_path_length = 0.0;
        for (size_t i = 1; i < candidate_samples.point_set.size(); ++i) {
            kino_path_length +=
                (candidate_samples.point_set[i] - candidate_samples.point_set[i - 1]).norm();
        }
        if (tried_kino_path_lengths_ss.tellp() > 0) tried_kino_path_lengths_ss << ",";
        tried_kino_path_lengths_ss << kino_path_length;
        if (!std::isfinite(kino_path_length)) {
            if (tried_end_delta_ws_ss.tellp() > 0) tried_end_delta_ws_ss << ",";
            tried_end_delta_ws_ss << "invalid";
            continue;
        }

        const double projected_end_w = projectClosedLocal(
            actual_end, planning_phase_w, 0.0, candidates.back());
        const double raw_end_delta_w = projected_end_w - planning_phase_w;
        if (!isFiniteClosedGoalCandidate(
                actual_end.x(), actual_end.y(), actual_end.z(),
                end_to_goal_dist, raw_end_delta_w)) {
            if (tried_end_delta_ws_ss.tellp() > 0) tried_end_delta_ws_ss << ",";
            tried_end_delta_ws_ss << "invalid";
            continue;
        }
        const double end_delta_w = std::max(0.0, raw_end_delta_w);
        const bool passed_obstacle = bypass_mode &&
            end_delta_w >= bypass_delta_w - std::max(1e-3, closed_goal_obstacle_check_step_w_);
        if (tried_end_delta_ws_ss.tellp() > 0) tried_end_delta_ws_ss << ",";
        tried_end_delta_ws_ss << end_delta_w;

        ClosedGoalCandidateResult result;
        result.idx = idx;
        result.lookahead = lookahead;
        result.goal = candidate_goal;
        result.goal_w = goal_w;
        result.samples = candidate_samples;
        result.full_success = end_to_goal_dist < closed_goal_full_success_tol_;
        result.kino_path_length = kino_path_length;
        result.end_to_goal_dist = end_to_goal_dist;
        result.projected_end_w = projected_end_w;
        result.progress.valid = true;
        result.progress.passed_obstacle = passed_obstacle;
        result.progress.end_delta_w = end_delta_w;
        result.progress.end_to_goal_dist = end_to_goal_dist;
        result.progress.lookahead = lookahead;
        result.progressive.valid = true;
        result.progressive.idx = idx;
        result.progressive.lookahead = lookahead;
        result.progressive.end_delta_w = end_delta_w;
        result.progressive.kino_path_length = kino_path_length;
        result.progressive.end_to_goal_dist = end_to_goal_dist;
        valid_candidates.push_back(result);
    }

    int selected_candidate = -1;
    for (size_t i = 0; i < valid_candidates.size(); ++i) {
        if (selected_candidate < 0 || preferClosedGoalProgressiveCandidate(
                valid_candidates[i].progressive,
                valid_candidates[selected_candidate].progressive,
                required_progress_w, desired_lookahead)) {
            selected_candidate = static_cast<int>(i);
        }
    }

    if (selected_candidate >= 0) {
        const ClosedGoalCandidateResult& selected = valid_candidates[selected_candidate];
        planner_success = true;
        selected_idx = selected.idx;
        selected_lookahead = selected.lookahead;
        selected_goal_w = selected.goal_w;
        goal_pt = selected.goal;
        pm.goal_pt = goal_pt;
        samples = selected.samples;
        selected_end_to_goal_dist = selected.end_to_goal_dist;
        selected_end_delta_w = selected.progress.end_delta_w;
        selected_kino_path_length = selected.kino_path_length;
        selected_passed_obstacle = selected.progress.passed_obstacle;
        accepted_full_goal = selected.full_success;
        accepted_partial_goal = !selected.full_success;
        selected_progress_sufficient = closedGoalProgressSufficient(
            selected_end_delta_w, required_progress_w);
        selected_reason = selected_progress_sufficient
            ? "progressive_sufficient"
            : "progressive_farthest_fallback";
    }

    const Eigen::Vector3d selected_goal_pos = pointFromClosedW(selected_goal_w);
    const double goal_dist_xy = (selected_goal_pos.head<2>() - curr_pos.head<2>()).norm();
    const double selected_score = planner_success ?
        closedGoalCandidateScore(selected_lookahead, desired_lookahead, selected_end_to_goal_dist,
                                 closed_goal_lookahead_weight_, closed_goal_end_dist_weight_) :
        -1.0;
    ROS_WARN("[GVF][CLOSED_GOAL] curr_pos=(%.3f,%.3f,%.3f) goal_pos=(%.3f,%.3f,%.3f) goal_dist_xy=%.3f selected_lookahead=%.3f selected_delta_w=%.3f desired_lookahead=%.3f obstacle_delta_w=%.3f desired_pushed=%d selected_score=%.3f closed_ref_w=%.3f selected_goal_w=%.3f candidate_count=%d considered_candidate_count=%d selected_idx=%d planner_success=%d reason=%s mode_for_goal=%s candidate_order_reason=%s local_d=%.3f tangent_dot_odom=%.3f candidate_order=\"%s\" full_success_tol=%.3f accepted_full_goal=%d accepted_partial_goal=%d selected_end_to_goal_dist=%.3f tried_lookaheads=\"%s\" tried_end_to_goal_dists=\"%s\" obstacle_end_delta_w=%.3f bypass_delta_w=%.3f bypass_mode=%d selected_end_delta_w=%.3f selected_passed_obstacle=%d tried_end_delta_ws=\"%s\" selection_mode=progressive progressive_max_lookahead_w=%.3f required_progress_w=%.3f selected_kino_path_length=%.3f selected_progress_sufficient=%d tried_kino_path_lengths=\"%s\"",
             curr_pos.x(), curr_pos.y(), curr_pos.z(),
             selected_goal_pos.x(), selected_goal_pos.y(), selected_goal_pos.z(),
             goal_dist_xy, selected_lookahead, selected_goal_w - planning_phase_w,
             desired_lookahead,
             std::isfinite(first_obstacle_delta_w) ? first_obstacle_delta_w : -1.0,
             desired_pushed_by_obstacle ? 1 : 0,
             selected_score,
             planning_phase_w, selected_goal_w, candidate_count, considered_candidate_count,
             selected_idx, planner_success ? 1 : 0, selected_reason.c_str(),
             mode_for_goal.c_str(), candidate_order_reason.c_str(),
             local_d_for_goal, tangent_dot_odom_for_goal,
             candidate_order_ss.str().c_str(), closed_goal_full_success_tol_,
             accepted_full_goal ? 1 : 0, accepted_partial_goal ? 1 : 0,
             selected_end_to_goal_dist,
             tried_lookaheads_ss.str().c_str(), tried_end_dists_ss.str().c_str(),
             std::isfinite(obstacle_end_delta_w) ? obstacle_end_delta_w : -1.0,
             std::isfinite(bypass_delta_w) ? bypass_delta_w : -1.0,
             bypass_mode ? 1 : 0,
             std::isfinite(selected_end_delta_w) ? selected_end_delta_w : -1.0,
             selected_passed_obstacle ? 1 : 0,
             tried_end_delta_ws_ss.str().c_str(),
             progressive_max_lookahead, required_progress_w,
             std::isfinite(selected_kino_path_length) ? selected_kino_path_length : -1.0,
             selected_progress_sufficient ? 1 : 0,
             tried_kino_path_lengths_ss.str().c_str());

    if (!planner_success) {
        last_failed_goal_idx_ = selected_idx;
        last_closed_goal_plan_success_ = false;
        return false;
    }

    closed_ref_pending_goal_w_ = selected_goal_w;
    closed_ref_pending_lookahead_w_ = selected_lookahead;
    closed_ref_has_pending_goal_ = true;
    closed_ref_pending_from_bypass_ = false;
    closed_ref_last_goal_pos_ = selected_goal_pos;
    closed_ref_last_goal_dist_xy_ = goal_dist_xy;
    closed_ref_last_candidate_idx_ = selected_idx;

    last_selected_goal_idx_ = selected_idx;
    last_failed_goal_idx_ = -1;
    last_closed_goal_plan_success_ = true;
    return true;
}

bool gvf_manager::astaropt(const Eigen::Vector3d& curr_pos, Eigen::MatrixXd& pos_out, Eigen::MatrixXd& vel_out,
                           int& new_i0_out, Eigen::VectorXd& time,
                           UniformBspline* continuous_spline_out)
{
    auto& pm = swarmParticlesManager[0];
    /*----------- ① Kino A* 搜索路径 + B 样条 -----------*/

    Eigen::Vector3d start_pt, start_vel = Eigen::Vector3d::Zero(), start_acc = Eigen::Vector3d::Zero();

    if (pm.is_first_goal || pm.last_traj.rows() == 0) {
        start_pt = Eigen::Vector3d(odom_.x() + 1e-6, odom_.y() + 1e-6, 1.0);
    } else {

        int i0 = std::max(0, std::min(current_traj_index_, (int)pm.last_traj.rows() - 1));

        start_pt = pm.last_traj.row(i0).transpose();
        start_vel = pm.last_vel.row(i0).transpose();

        double dt = 1.0 / std::max(1, pm.spline_->TrajSampleRate);
        if (i0 + 1 < pm.last_vel.rows()) {
            start_acc = (pm.last_vel.row(i0 + 1) - pm.last_vel.row(i0)).transpose() / dt;
        }

    }

    Eigen::Vector3d goal_pt = pm.goal_pt;
    Eigen::Vector3d end_vel = Eigen::Vector3d::Zero();
    const bool use_closed_goal_candidates = enable_circle_reference_test_ && circle_reference_ready_;
    KinoPlanSamples plan_samples;

    if (use_closed_goal_candidates) {
        if (!selectClosedGoalCandidate(pm, curr_pos, start_pt, start_vel, start_acc,
                                       goal_pt, end_vel, plan_samples)) {
            return false;
        }
    } else {
        if (!planKinoToGoal(pm, start_pt, start_vel, start_acc, goal_pt, end_vel, plan_samples)) {
            return false;
        }
        pm.goal_pt = goal_pt;
    }

    vector<Eigen::Vector3d> point_set = plan_samples.point_set;
    vector<Eigen::Vector3d> start_end_derivatives = plan_samples.start_end_derivatives;
    double ts = plan_samples.ts;

    if (use_closed_goal_candidates && !point_set.empty()) {
        const Eigen::Vector3d kino_end_pos = point_set.back();
        const double kino_duration = ts * static_cast<double>(std::max(0, static_cast<int>(point_set.size()) - 1));
        const double kino_end_to_goal_dist = (kino_end_pos - closed_ref_last_goal_pos_).norm();
        ROS_WARN("[GVF][KINO_RESULT] pending_goal_w=%.3f pending_lookahead=%.3f accepted_goal_w=%.3f accepted_lookahead=%.3f goal_dist_xy=%.3f kino_start_pos=(%.3f,%.3f,%.3f) kino_end_pos=(%.3f,%.3f,%.3f) kino_end_to_goal_dist=%.3f kino_duration=%.3f kino_success=1 candidate_idx=%d",
                 closed_ref_pending_goal_w_, closed_ref_pending_lookahead_w_,
                 closed_ref_accepted_goal_w_, closed_ref_accepted_lookahead_w_,
                 closed_ref_last_goal_dist_xy_,
                 start_pt.x(), start_pt.y(), start_pt.z(),
                 kino_end_pos.x(), kino_end_pos.y(), kino_end_pos.z(),
                 kino_end_to_goal_dist, kino_duration,
                 closed_ref_last_candidate_idx_);
    }

    if (point_set.size() < 2 || start_end_derivatives.size() != 4 ||
        !std::isfinite(ts) || ts <= 0.0) {
        ROS_WARN("[gvf kino replan] invalid Kino samples: points=%zu derivatives=%zu ts=%.6f",
                 point_set.size(), start_end_derivatives.size(), ts);
        return false;
    }

    Eigen::MatrixXd initial_control_points;
    if (!UniformBspline::parameterizeToBspline(
            ts, point_set, start_end_derivatives, initial_control_points)) {
        ROS_WARN("[gvf kino replan] Fast-Planner parameterization failed");
        return false;
    }

    if (!pm.bspline_opt_->setInitialControlPoints(initial_control_points, ts)) {
        ROS_WARN("[gvf kino replan] optimizer rejected parameterized control points");
        return false;
    }

    pm.bspline_opt_->optimize();

    if (!pm.spline_->setControlPointsAndInterval(
            pm.bspline_opt_->control_points_, 3, ts)) {
        ROS_WARN("[gvf kino replan] spline initialization failed");
        return false;
    }

    const double feasibility_ratio = pm.spline_->getFeasibilityRatio(
        pm.bspline_opt_->max_vel_, pm.bspline_opt_->max_acc_);
    if (!std::isfinite(feasibility_ratio)) {
        ROS_WARN("[gvf kino replan] invalid feasibility ratio");
        return false;
    }
    if (feasibility_ratio > 1.0 &&
        !pm.spline_->scaleTime(feasibility_ratio * 1.01)) {
        ROS_WARN("[gvf kino replan] time scaling failed");
        return false;
    }

    pm.spline_->getT();

    ROS_INFO_THROTTLE(
        1.0,
        "[GVF][BSPLINE_PARAM] kino_points=%zu control_points=%d kino_ts=%.4f final_interval=%.4f ratio=%.3f",
        point_set.size(), static_cast<int>(pm.bspline_opt_->control_points_.rows()),
        ts, pm.spline_->interval_, feasibility_ratio);

    UniformBspline p = *pm.spline_;
    UniformBspline v = p.getDerivative();

    Eigen::MatrixXd p_ = p.getTrajectory(p.time_);
    Eigen::MatrixXd v_ = v.getTrajectory(p.time_);

    // 找候选轨迹上离当前 odom 最近的索引（用于后续切换/跟踪）
    int best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (int i = 0; i < p_.rows(); ++i) {
        Eigen::Vector3d pt(p_(i, 0), p_(i, 1), p_(i, 2));
        double d = (pt - curr_pos).squaredNorm();
        if (d < best_d) { best_d = d; best = i; }
    }

    pos_out = p_;
    vel_out = v_;
    time = p.time_;
    new_i0_out = std::max(0, std::min(best, (int)p_.rows() - 1));
    if (continuous_spline_out) {
        *continuous_spline_out = p;
    }
    return true;
}


    void gvf_manager::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
        string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
        int    pre_s        = int(exec_state_);
        exec_state_         = new_state;
        cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    }

bool gvf_manager::finalizeSectionTerminal(
    gvfManager& pm, const std::uint64_t expected_task_generation,
    double& retained_manual_reference, bool allow_nonzero_reference) {
    retained_manual_reference = 0.0;
    if (!pm.gvf_ || !phase_offset_matched_adapter_ ||
        matched_config_.mode != PhaseOffsetMatchedMode::MANUAL) {
        return false;
    }

    // Keep task-end state mutation in the same lock domain as the final
    // Runtime read.  In particular, a command that committed nonzero after
    // the caller captured its zero snapshot cannot race this transition.
    SectionPathBundlePtr retired_current_bundle;
    SectionPathBundlePtr retired_staged_bundle;
    SectionPathBundlePtr retired_pending_bundle;
    std::shared_ptr<const ContinuousPhasePath> retired_staged_source;
    std::shared_ptr<const ContinuousPhasePath> retired_pending_source;
    std::shared_ptr<const phase_offset_navigation::RuntimeSectionPreparedStep>
        retired_pending_step;
    phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
        retired_pending_query;
    std::shared_ptr<const SectionPathHandoff> retired_manager_handoff;
    std::lock_guard<std::mutex> frontend_lock(frontend_apply_mutex_);
    std::lock_guard<std::mutex> handoff_lock(path_reference_handoff_mutex_);
    std::lock_guard<std::mutex> phase_lock(authoritative_phase_mutex_);
    std::lock_guard<std::mutex> task_lock(
        phase_offset_matched_adapter_->task_publication_mutex_);
    std::lock_guard<std::mutex> runtime_lock(
        phase_offset_matched_adapter_->runtime_command_mutex_);

    if (phase_offset_matched_adapter_->executionGeneration() !=
            expected_task_generation ||
        !phase_offset_matched_adapter_->runtime_) {
        return false;
    }
    retained_manual_reference =
        phase_offset_matched_adapter_->runtime_->retainedDelta();
    if (retained_manual_reference != 0.0 && !allow_nonzero_reference) {
        return false;
    }

    const std::uint64_t generation =
        phase_offset_matched_adapter_->task_generation_.load(
            std::memory_order_acquire);
    if (generation == std::numeric_limits<std::uint64_t>::max()) return false;

    retired_current_bundle.swap(phase_offset_matched_adapter_->section_bundle_);
    retired_staged_bundle.swap(phase_offset_matched_adapter_->staged_section_bundle_);
    retired_staged_source.swap(
        phase_offset_matched_adapter_->staged_section_source_path_);
    phase_offset_matched_adapter_->staged_section_copied_prefix_start_w_ =
        std::numeric_limits<double>::quiet_NaN();
    phase_offset_matched_adapter_->staged_section_copied_prefix_end_w_ =
        std::numeric_limits<double>::quiet_NaN();
    retired_pending_bundle.swap(
        phase_offset_matched_adapter_->pending_section_bundle_);
    retired_pending_source.swap(
        phase_offset_matched_adapter_->pending_section_source_path_);
    retired_pending_step.swap(
        phase_offset_matched_adapter_->pending_section_prepared_step_);
    retired_pending_query.swap(
        phase_offset_matched_adapter_->pending_section_reference_query_);
    phase_offset_matched_adapter_->clearPendingPositionCommandLocked();
    retired_manager_handoff.swap(pending_section_path_handoff_);

    phase_offset_matched_adapter_->task_generation_.store(
        generation + 1U, std::memory_order_release);
    phase_offset_matched_adapter_->zero_gate_consecutive_count_ = 0;
    phase_offset_matched_adapter_->zero_gate_open_ = false;
    phase_offset_matched_adapter_->failure_latched_ = false;
    phase_offset_matched_adapter_->control_failure_reason_ =
        phase_offset_navigation::ControlFailureReason::NONE;
    phase_offset_matched_adapter_->runtime_->resetForNewNavigationTask();

    // Preserve the final phase/path visualization, while advancing the
    // existing phase generation so an already-captured command token cannot
    // publish after this task has ended.
    publishAuthoritativePhaseLocked(
        phase_w_, phase_initialized_, closed_phase_acquired_);
    pm.receive_goal = false;
    pm.is_first_goal = false;
    changeFSMExecState(WAIT_TARGET, "reach_goal");
    return true;
}

void gvf_manager::FSMCallback(const ros::TimerEvent& event)
{
    auto& pm = swarmParticlesManager[0];
    Eigen::Vector3d current_pos(odom_.x(), odom_.y(), odom_.z());
    ros::Time current_time = ros::Time::now();
    // New-task handoff from goalCallback.  The goal reset republishes the phase
    // origin, but the previously accepted frontend still carries the old task's
    // w domain.  Keeping both made projectToPathLocal()'s window empty on every
    // cycle (w_prev=0 versus start_w~35), so no governor candidate was valid and
    // the vehicle held position forever.  Retire the stale frontend here, on the
    // FSM thread, and let WAIT_TARGET -> GEN_NEW_TRAJ install a fresh one.
    if (new_task_pending_.exchange(false, std::memory_order_acq_rel)) {
        for (auto& manager : swarmParticlesManager) {
            if (manager.gvf_) {
                manager.gvf_->clearContinuousPhasePath();
                manager.gvf_->clearPathReparamState();
            }
            manager.last_traj.resize(0, 3);
            manager.last_vel.resize(0, 3);
            manager.is_first_goal = true;
            manager.receive_goal = true;
        }
        resetGovernorState();
        current_traj_index_ = 0;
        last_switch_time_ = ros::Time(0);
        terminal_residual_start_ = ros::Time(0);
        changeFSMExecState(WAIT_TARGET, "new_task");
    }
    // FSM is the single writer for pm.last_* and the GVF display mirror, so
    // consume a publish-committed immutable V2 handoff before planning.
    consumeCommittedSectionPathHandoff(pm, current_time);
    // A replan intentionally retains this publish-committed phase-after
    // witness through its expensive planner and request preparation work.
    // Capture the existing task generation while holding the same phase lock
    // used by resetForNewNavigationTask().  The terminal helper later needs
    // only this task witness; normal zero-delta phase progress is allowed.
    AuthoritativePhaseSnapshot fsm_phase;
    std::uint64_t fsm_task_generation = 0U;
    {
        std::lock_guard<std::mutex> phase_lock(authoritative_phase_mutex_);
        fsm_phase.w = phase_w_;
        fsm_phase.initialized = phase_initialized_;
        fsm_phase.closed_acquired = closed_phase_acquired_;
        fsm_phase.generation = authoritative_phase_generation_;
        if (phase_offset_matched_adapter_) {
            fsm_task_generation =
                phase_offset_matched_adapter_->executionGeneration();
        }
    }

    // MANUAL Section production is deliberately synchronous and planning
    // owned.  Capture the immutable path/phase snapshot first, then copy a
    // bounded local map view and build the profile outside every command or
    // Runtime lock.  A failed attempt leaves the committed bundle untouched;
    // the next FSM period retries without forcing the nominal planner into a
    // permanent hold.
    if (phase_offset_matched_adapter_ &&
        phase_offset_matched_adapter_->configurationValid() &&
        matched_config_.mode == PhaseOffsetMatchedMode::MANUAL &&
        fsm_phase.initialized && pm.gvf_) {
        const ros::Duration section_period(
            std::max(0.0, matched_config_.tube_update_period));
        const bool section_due = last_section_build_time_.isZero() ||
            (current_time - last_section_build_time_) >= section_period;
        bool section_handoff_pending = false;
        {
            std::lock_guard<std::mutex> handoff_lock(
                path_reference_handoff_mutex_);
            section_handoff_pending = static_cast<bool>(
                pending_section_path_handoff_);
        }
        if (section_due && !section_handoff_pending) {
            const std::shared_ptr<const ContinuousPhasePath> section_path =
                pm.gvf_->getContinuousPhasePath();
            const bool section_built = buildAndStageSectionBundle(
                pm, fsm_phase, section_path);
            last_section_build_time_ = current_time;
            if (!section_built) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[phase_offset_section] synchronous profile unavailable; retain current bundle");
            }
        }
    }

    // Display-only Section tube boundaries.  Geometry comes from the committed
    // immutable bundle and is built here, on the planning thread; the command
    // callback never samples the path for visualization.  Publishing on
    // bundle change (including retirement to DELETE) plus a slow heartbeat
    // keeps RViz correct without a 50 Hz marker stream.
    if (section_tube_marker_pub_) {
        SectionPathBundlePtr marker_bundle;
        if (phase_offset_matched_adapter_ &&
            matched_config_.mode == PhaseOffsetMatchedMode::MANUAL) {
            // Prefer the committed bundle.  In observe-only mode nothing is
            // ever committed, so fall back to the candidate the planning
            // thread just built; both are immutable path+profile values from
            // the same builder and this display has no execution authority.
            marker_bundle =
                phase_offset_matched_adapter_->captureSectionBundle();
            if (!marker_bundle) {
                marker_bundle = phase_offset_matched_adapter_
                                    ->capturePendingSectionCandidate()
                                    .bundle;
            }
        }
        const bool bundle_changed =
            marker_bundle != last_section_marker_bundle_;
        const bool heartbeat_due = last_section_marker_time_.isZero() ||
            (current_time - last_section_marker_time_) >= ros::Duration(1.0);
        // A retired tube needs exactly one DELETE; an active tube is refreshed
        // on bundle change and on a slow heartbeat so RViz keeps showing it.
        const bool need_delete =
            !marker_bundle && section_tube_marker_active_;
        const bool need_publish =
            (marker_bundle && (bundle_changed || heartbeat_due)) || need_delete;
        if (need_publish) {
            const std::string marker_frame =
                (marker_bundle && !marker_bundle->frame_id.empty())
                    ? marker_bundle->frame_id
                    : matched_config_.frame_id;
            const std::shared_ptr<const ContinuousPhasePath> marker_path =
                marker_bundle ? marker_bundle->path
                              : std::shared_ptr<const ContinuousPhasePath>();
            const std::shared_ptr<
                const phase_offset_navigation::SectionTubeProfile>
                marker_profile =
                    marker_bundle
                        ? marker_bundle->profile
                        : std::shared_ptr<
                              const phase_offset_navigation::SectionTubeProfile>();
            section_tube_marker_pub_.publish(MakeSectionTubeMarkers(
                current_time, marker_frame, marker_path, marker_profile));
            last_section_marker_bundle_ = marker_bundle;
            last_section_marker_time_ = current_time;
            section_tube_marker_active_ = static_cast<bool>(marker_bundle);
        }
    }

    // Display-only executed reference: p(w) + N(w) * delta.  This is the
    // centreline the Section offset actually asks the vehicle to follow, so
    // RViz can show the intent-adjusted path next to the nominal one.  It is
    // published from the planning thread at a low rate and never feeds back
    // into control.
    if (adjusted_path_pub_ && phase_offset_matched_adapter_ &&
        matched_config_.mode == PhaseOffsetMatchedMode::MANUAL &&
        (last_adjusted_path_time_.isZero() ||
         (current_time - last_adjusted_path_time_) >= ros::Duration(0.5))) {
        last_adjusted_path_time_ = current_time;
        SectionPathBundlePtr adjusted_bundle =
            phase_offset_matched_adapter_->captureSectionBundle();
        if (adjusted_bundle && adjusted_bundle->path) {
            double delta = 0.0;
            {
                std::lock_guard<std::mutex> runtime_lock(
                    phase_offset_matched_adapter_->runtime_command_mutex_);
                if (phase_offset_matched_adapter_->runtime_) {
                    delta = phase_offset_matched_adapter_->runtime_->
                        retainedDelta();
                }
            }
            const std::shared_ptr<const ContinuousPhasePath> path =
                adjusted_bundle->path;
            const double w0 = path->startW();
            const double w1 = path->endW();
            if (std::isfinite(w0) && std::isfinite(w1) && w1 > w0 &&
                std::isfinite(delta)) {
                visualization_msgs::Marker marker;
                marker.header.frame_id =
                    matched_config_.frame_id.empty()
                        ? std::string("world")
                        : matched_config_.frame_id;
                marker.header.stamp = current_time;
                marker.ns = "phase_offset_adjusted_path";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.06;
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
                marker.color.a = 0.9;
                ContinuousPhaseNormalFrame frame(path, 1U, 1U);
                phase_offset_core::NormalFrameQuery frame_query;
                constexpr int kMaxSamples = 200;
                const double step =
                    std::max(0.05, (w1 - w0) / static_cast<double>(kMaxSamples));
                for (double w = w0; w <= w1 + 1e-9; w += step) {
                    ContinuousPhasePathState state;
                    if (!path->evaluate(w, state, true)) break;
                    Eigen::Vector3d point = state.p;
                    if (delta != 0.0 && frame.query(w, frame_query) &&
                        frame_query.valid) {
                        point += frame_query.N * delta;
                    }
                    geometry_msgs::Point ros_point;
                    ros_point.x = point.x();
                    ros_point.y = point.y();
                    ros_point.z = point.z();
                    marker.points.push_back(ros_point);
                }
                if (marker.points.size() >= 2U) {
                    adjusted_path_pub_.publish(marker);
                }
            }
        }
    }

    switch (exec_state_)
    {
        case WAIT_TARGET:
            if(pm.receive_goal || pm.is_first_goal)
            {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            else return;
        break;

        case GEN_NEW_TRAJ:{
            if (closedPhaseV2Active()) {
                if (installInitialClosedPhaseFrontend(pm, current_pos, current_time)) {
                    changeFSMExecState(EXEC_TRAJ, "closed_phase_v2 init");
                } else {
                    ROS_WARN_THROTTLE(1.0,
                        "[GVF][CLOSED_PHASE_V2] nominal frontend initialization failed; retry in GEN_NEW_TRAJ");
                }
                break;
            }

            if (pointPhaseV2Active()) {
                Eigen::MatrixXd cand_traj, cand_vel;
                Eigen::VectorXd cand_time;
                UniformBspline cand_spline;
                int new_i0 = 0;
                if (!astaropt(current_pos, cand_traj, cand_vel, new_i0,
                              cand_time, &cand_spline)) {
                    ROS_WARN_THROTTLE(1.0,
                        "[GVF][POINT_PHASE_V2][PLANNER_BSPLINE_CANDIDATE] "
                        "Kino/B-spline candidate failed; retry in GEN_NEW_TRAJ");
                } else if (installInitialPointPhaseFrontend(
                               pm, current_pos, current_time, cand_traj,
                               cand_vel, cand_time, new_i0, cand_spline)) {
                    changeFSMExecState(EXEC_TRAJ, "point_phase_v2 init");
                } else {
                    ROS_WARN_THROTTLE(1.0,
                        "[GVF][POINT_PHASE_V2][NEUTRAL_FRONTEND_INSTALL] "
                        "initial planner frontend install failed; retry in GEN_NEW_TRAJ");
                }
                break;
            }

            Eigen::MatrixXd cand_traj, cand_vel;
            Eigen::VectorXd cand_time;
            int new_i0 = 0;
            if (astaropt(current_pos, cand_traj, cand_vel, new_i0, cand_time)) {
                Eigen::Vector3d v_ref_start = Eigen::Vector3d::Zero();
                if (cand_vel.rows() > 0) {
                    const int v_idx = std::max(0, std::min(new_i0, (int)cand_vel.rows() - 1));
                    v_ref_start = cand_vel.row(v_idx).transpose();
                }
                const Eigen::Vector3d v_odom = odom_vel_est_;
                double cos_start_odom = 0.0;
                if (v_ref_start.norm() > 1e-6 && v_odom.norm() > 1e-6) {
                    cos_start_odom = v_ref_start.normalized().dot(v_odom.normalized());
                }
                if (enable_circle_reference_test_ && circle_reference_ready_ && closed_ref_has_pending_goal_) {
                    closed_ref_accepted_goal_w_ = closed_ref_pending_goal_w_;
                    closed_ref_accepted_lookahead_w_ = closed_ref_pending_lookahead_w_;
                    closed_ref_has_accepted_goal_ = true;
                    closed_ref_accepted_from_bypass_ = closed_ref_pending_from_bypass_;
                }
                ROS_WARN("[GVF][SWITCH_OBS] accept_reason=gen_new_traj accepted_new=1 pending_goal_w=%.3f accepted_goal_w=%.3f pending_lookahead=%.3f accepted_lookahead=%.3f v_ref_start=(%.3f,%.3f,%.3f) v_odom=(%.3f,%.3f,%.3f) cos_start_odom=%.3f",
                         closed_ref_pending_goal_w_, closed_ref_accepted_goal_w_,
                         closed_ref_pending_lookahead_w_, closed_ref_accepted_lookahead_w_,
                         v_ref_start.x(), v_ref_start.y(), v_ref_start.z(),
                         v_odom.x(), v_odom.y(), v_odom.z(), cos_start_odom);
                pm.last_traj = cand_traj;
                pm.last_vel = cand_vel;
                pm.last_traj_time_ = cand_time;
                pm.is_first_goal = false;
                current_traj_index_ = new_i0;
                last_switch_time_ = current_time;
                cmd_switch_motion_limit_until_ = ros::Time::now() + ros::Duration(std::max(0.0, cmd_switch_motion_limit_time_));
                cmd_governor_normal_state_.setZero();
                cmd_governor_initialized_ = false;
                cmd_governor_last_l_ = 0.0;
                publishPathMsg(pm.last_traj, pm.last_vel);
            } else {
                ROS_WARN_THROTTLE(1.0, "[GVF] GEN_NEW_TRAJ: plan failed, keep old");
                publishPathMsg(pm.last_traj, pm.last_vel);
            }
            last_replan_time_ = current_time;
            changeFSMExecState(EXEC_TRAJ, "FSM");
            break;
        }

        case EXEC_TRAJ:{
            if (pm.last_traj.rows() > 0) {
                const gvf::ReparamCacheSnapshot cache = pm.gvf_
                    ? pm.gvf_->captureReparamCacheSnapshot()
                    : gvf::ReparamCacheSnapshot();
                if (unifiedPhaseV2Active() && fsm_phase.initialized &&
                    cache.w.size() == static_cast<size_t>(pm.last_traj.rows())) {
                    auto it = std::lower_bound(
                        cache.w.begin(), cache.w.end(),
                        fsm_phase.w);
                    current_traj_index_ = it == cache.w.end()
                        ? pm.last_traj.rows() - 1
                        : static_cast<int>(std::distance(cache.w.begin(), it));
                } else {
                    double min_dist = std::numeric_limits<double>::max();
                    for (int i = current_traj_index_; i < pm.last_traj.rows(); ++i) {
                        Eigen::Vector3d traj_point(
                            pm.last_traj(i,0), pm.last_traj(i,1), pm.last_traj(i,2));
                        const double dist = (traj_point - current_pos).norm();
                        if (dist < min_dist) {
                            min_dist = dist;
                            current_traj_index_ = i;
                        }
                    }
                }
            }

            const bool circle_mode_active = enable_circle_reference_test_ && circle_reference_ready_;
            const double dist_xy = (pm.goal_pt.head<2>() - current_pos.head<2>()).norm();
            // The executed Section reference is r = p + N*delta, so a retained
            // offset parks it at goal + N*delta.  While the swarm intent holds a
            // nonzero delta, the strict 0.2 m arrival test can never be met: the
            // vehicle stops |delta| short, this branch never runs, no recenter
            // is ever requested, and the FSM never reaches WAIT_TARGET.
            // Enlarging the neighbourhood by the retained offset makes the
            // recenter start while the vehicle is still |delta| away; once the
            // offset decays the strict radius applies again and ordinary
            // arrival fires.  With no retained offset the radius is unchanged.
            double retained_offset_magnitude = 0.0;
            if (phase_offset_matched_adapter_ &&
                matched_config_.mode == PhaseOffsetMatchedMode::MANUAL) {
                std::lock_guard<std::mutex> runtime_lock(
                    phase_offset_matched_adapter_->runtime_command_mutex_);
                if (phase_offset_matched_adapter_->runtime_) {
                    retained_offset_magnitude = std::abs(
                        phase_offset_matched_adapter_->runtime_->retainedDelta());
                }
            }
            const double terminal_radius =
                retained_offset_magnitude > 0.0
                    ? 0.2 + retained_offset_magnitude +
                          point_goal_terminal_offset_slack_
                    : 0.2;
            const bool terminal_candidate =
                shouldDeclarePointGoalReached(circle_mode_active, dist_xy,
                                              terminal_radius);
            bool terminal_zero_reference = terminal_candidate;
            double retained_manual_reference = 0.0;
            const bool manual_section_terminal = terminal_candidate &&
                phase_offset_matched_adapter_ &&
                matched_config_.mode == PhaseOffsetMatchedMode::MANUAL;
            const bool terminal_finalized = manual_section_terminal &&
                finalizeSectionTerminal(pm, fsm_task_generation,
                                        retained_manual_reference);
            if (manual_section_terminal) {
                terminal_zero_reference = terminal_finalized;
            }
            const bool nonzero_manual_reference = terminal_candidate &&
                manual_section_terminal &&
                retained_manual_reference != 0.0;
            if (terminal_finalized ||
                (terminal_candidate && !manual_section_terminal &&
                 terminal_zero_reference)) {
                // 到达后停止控制，但完整保留最终轨迹、相位路径和向量场。
                // 下一个目标进入 goalCallback 时再统一清空，避免跨任务 C2 拼接。
                resetGovernorState();
                if (pm.gvf_) {
                    pm.gvf_->setTerminalGoalVisualization(pm.goal_pt);
                }
                ROS_WARN("[GVF][POINT_GOAL][REACHED] distance=%.3f retained_final_traj=1 terminal_attractor_field=1 clear_on_next_goal=1",
                         dist_xy);
                if (!manual_section_terminal) {
                    changeFSMExecState(WAIT_TARGET, "reach_goal");
                    pm.receive_goal = false;
                    pm.is_first_goal = false;
                }
                return;
            }
            if (terminal_candidate && nonzero_manual_reference) {
                // Reaching the geometric goal while an active Section
                // reference is nonzero is not terminal.  Keep the task alive
                // so cmdCallback can publish the bounded recenter command and
                // periodic/collision replanning can produce a compatible
                // continuation; only a true zero-offset state takes the
                // legacy WAIT_TARGET branch above.
                if (phase_offset_matched_adapter_) {
                    phase_offset_matched_adapter_->requestRecenter();
                }
                // The recenter cannot always win against a persistent swarm
                // intent, and an exhausted path then leaves the governor with
                // no candidate at all.  Bound the wait: once the window
                // expires, accept the current intent equilibrium as the
                // terminal state instead of holding forever.
                if (terminal_residual_start_.isZero()) {
                    terminal_residual_start_ = current_time;
                }
                const bool residual_window_expired =
                    max_terminal_residual_hold_s_ >= 0.0 &&
                    (current_time - terminal_residual_start_).toSec() >=
                        max_terminal_residual_hold_s_;
                if (residual_window_expired &&
                    finalizeSectionTerminal(
                        pm, fsm_task_generation, retained_manual_reference,
                        /*allow_nonzero_reference=*/true)) {
                    resetGovernorState();
                    if (pm.gvf_) {
                        pm.gvf_->setTerminalGoalVisualization(pm.goal_pt);
                    }
                    ROS_WARN("[GVF][POINT_GOAL][REACHED] distance=%.3f "
                             "retained_final_traj=1 terminal_attractor_field=1 "
                             "clear_on_next_goal=1 residual_forced=1 delta=%.6f",
                             dist_xy, retained_manual_reference);
                    return;
                }
                ROS_WARN_THROTTLE(
                    1.0,
                    "[GVF][POINT_GOAL] nonzero Section reference retained; continue recenter/replan delta=%.6f",
                    retained_manual_reference);
            } else {
                terminal_residual_start_ = ros::Time(0);
            }

            const bool collision_detected = checkCollision();
            const bool periodic_due =
                (current_time - last_replan_time_).toSec() >= planInterval;
            const bool periodic_phase_ready =
                !closedPhaseV2Active() || fsm_phase.closed_acquired;
            const ReplanTriggerDecision replan_trigger = selectReplanTrigger(
                collision_detected, periodic_due, periodic_phase_ready,
                // A failed nonzero handoff is a transaction result, not a
                // permanent frontend gate.  Keep periodic replanning alive
                // so a later valid Path+Tube candidate can be retried while
                // collision-triggered replanning retains its priority.
                false);
            if (replan_trigger == ReplanTriggerDecision::COLLISION) {
                logReplanReason("collision");
                changeFSMExecState(REPLAN_TRAJ, "collision detection");
            }
            else if (replan_trigger == ReplanTriggerDecision::PERIODIC) {
                logReplanReason("plan_interval");
                changeFSMExecState(REPLAN_TRAJ, "planInterval reached");
            }
            else if (closedPhaseV2Active() && !fsm_phase.closed_acquired) {
                const Eigen::Vector3d phase_point = pm.gvf_
                    ? pm.gvf_->evalPathByW(fsm_phase.w)
                    : pointFromClosedW(fsm_phase.w);
                ROS_INFO_THROTTLE(1.0,
                    "[GVF][CLOSED_PHASE_V2] initial GVF acquisition: phase_w=%.3f distance=%.3f threshold=%.3f",
                    fsm_phase.w, (current_pos - phase_point).norm(),
                    closed_phase_acquire_distance_);
            }

            break;
        }

        case REPLAN_TRAJ:{
            // Preserve the original closed-phase planner/C2 switching path.
            // Section MANUAL candidates are staged with the immutable source
            // prefix and consumed only after a command publication commits the
            // matching Runtime step; zero-offset candidates retain the
            // planner-only install path.
            if (closedPhaseV2Active()) {
                Eigen::MatrixXd cand_traj, cand_vel;
                Eigen::VectorXd cand_time;
                UniformBspline cand_spline;
                int new_i0 = 0;
                bool installed = false;

                if (astaropt(current_pos, cand_traj, cand_vel, new_i0,
                             cand_time, &cand_spline)) {
                    const double phase_at_switch = fsm_phase.w;
                    double path_end_w = closed_phase_has_pending_path_end_w_
                        ? closed_phase_pending_path_end_w_
                        : closed_ref_pending_goal_w_;
                    if (path_end_w <= phase_at_switch + 1e-3) {
                        path_end_w = std::max(closed_ref_pending_goal_w_,
                            phase_at_switch +
                                std::max(0.1, closed_ref_lookahead_min_w_));
                    }

                    std::vector<double> global_w;
                    if (pm.gvf_ && buildGlobalPhaseSamples(
                            cand_traj, new_i0, phase_at_switch, path_end_w,
                            global_w)) {
                        Eigen::MatrixXd install_traj = cand_traj;
                        Eigen::MatrixXd install_vel = cand_vel;
                        Eigen::VectorXd install_time = cand_time;
                        std::vector<double> install_w = global_w;
                        std::shared_ptr<const ContinuousPhasePath>
                            install_continuous_path;
                        std::shared_ptr<const ContinuousPhasePath>
                            planner_only_prefix_source;
                        double planner_only_prefix_end_w = phase_at_switch;
                        std::uint64_t planner_only_execution_generation = 0U;

                        bool section_handoff_required = false;
                        if (phase_offset_matched_adapter_ &&
                            matched_config_.mode ==
                                PhaseOffsetMatchedMode::MANUAL) {
                            std::lock_guard<std::mutex> runtime_lock(
                                phase_offset_matched_adapter_->runtime_command_mutex_);
                            section_handoff_required =
                                phase_offset_matched_adapter_->runtime_ &&
                                phase_offset_matched_adapter_->runtime_->retainedDelta() != 0.0;
                        }

                        const std::uint64_t captured_execution_generation =
                            phase_offset_matched_adapter_
                                ? phase_offset_matched_adapter_->executionGeneration()
                                : 0U;
                        bool frontend_ready = false;
                        bool section_source_valid = true;
                        std::shared_ptr<const ContinuousPhasePath> old_path =
                            pm.gvf_->getContinuousPhasePath();
                        if (section_handoff_required) {
                            const SectionPathBundlePtr committed_bundle =
                                phase_offset_matched_adapter_
                                    ? phase_offset_matched_adapter_->captureSectionBundle()
                                    : SectionPathBundlePtr();
                            if (!committed_bundle ||
                                committed_bundle->task_generation !=
                                    captured_execution_generation ||
                                !committed_bundle->path ||
                                committed_bundle->path->empty()) {
                                section_source_valid = false;
                            } else {
                                old_path = committed_bundle->path;
                            }
                        }
                        double future_seam_w = 0.0;
                        const bool future_seam_available =
                            section_source_valid &&
                            captured_execution_generation != 0U &&
                            plannerOnlyFutureSeam(
                                old_path, phase_at_switch,
                                matched_config_.tube.min_certified_forward_w,
                                future_seam_w);

                        if (section_handoff_required) {
                            double handoff_switch_w = future_seam_w;
                            double copied_prefix_start_w = phase_at_switch;
                            double copied_prefix_end_w = future_seam_w;
                            bool handoff_seam_available =
                                future_seam_available;
                            if (!handoff_seam_available && old_path &&
                                !old_path->empty() &&
                                phase_at_switch >=
                                    old_path->startW() - 1e-9 &&
                                phase_at_switch <= old_path->endW() + 1e-9) {
                                // The old frontend is exhausted at the live
                                // phase, so no future seam exists.  Start the
                                // connector a little behind the live phase and
                                // keep the whole source tail as the prefix
                                // window, so command-thread re-anchoring cannot
                                // fall outside the staged handoff.
                                const double seam_anchor_w = std::min(
                                    phase_at_switch, old_path->endW());
                                handoff_switch_w = std::max(
                                    old_path->startW(),
                                    seam_anchor_w -
                                        kExhaustedFrontendSeamBackW);
                                copied_prefix_start_w = handoff_switch_w;
                                copied_prefix_end_w = old_path->endW();
                                copied_prefix_start_w = std::max(
                                    copied_prefix_start_w,
                                    old_path->startW());
                                handoff_seam_available =
                                    copied_prefix_end_w >
                                    copied_prefix_start_w + 1e-6;
                            }
                            if (handoff_seam_available) {
                                frontend_ready = buildPhaseC2Frontend(
                                    pm, phase_at_switch, handoff_switch_w,
                                    old_path, path_end_w, new_i0, cand_spline,
                                    cand_traj, cand_vel, cand_time, global_w,
                                    install_traj, install_vel, install_time,
                                    install_w, install_continuous_path);
                            }
                            if (frontend_ready) {
                                frontend_ready = stageSectionPathHandoff(
                                    fsm_phase, pm, install_continuous_path,
                                    install_traj, install_vel, install_time,
                                    install_w, old_path,
                                    copied_prefix_start_w,
                                    copied_prefix_end_w);
                            }
                        } else if (closed_phase_c2_enabled_) {
                            frontend_ready = future_seam_available &&
                                buildPhaseC2Frontend(
                                    pm, phase_at_switch, future_seam_w,
                                    old_path, path_end_w, new_i0, cand_spline,
                                    cand_traj, cand_vel, cand_time, global_w,
                                    install_traj, install_vel, install_time,
                                    install_w, install_continuous_path);
                            if (frontend_ready) {
                                planner_only_prefix_source = old_path;
                                planner_only_prefix_end_w = future_seam_w;
                                planner_only_execution_generation =
                                    captured_execution_generation;
                            } else {
                                planner_only_prefix_source = old_path;
                                planner_only_prefix_end_w =
                                    old_path && !old_path->empty()
                                        ? old_path->endW()
                                        : phase_at_switch;
                                planner_only_execution_generation =
                                    captured_execution_generation;
                                frontend_ready = buildPhaseC2Frontend(
                                    pm, phase_at_switch, phase_at_switch,
                                    old_path, path_end_w, new_i0, cand_spline,
                                    cand_traj, cand_vel, cand_time, global_w,
                                    install_traj, install_vel, install_time,
                                    install_w, install_continuous_path);
                            }
                        } else {
                            frontend_ready = buildMappedPhaseFrontend(
                                phase_at_switch, path_end_w, false, new_i0,
                                cand_spline, cand_traj, cand_time, install_traj,
                                install_vel, install_time, install_w,
                                install_continuous_path);
                        }

                        if (!frontend_ready || install_w.empty() ||
                            !install_continuous_path) {
                            if (section_handoff_required &&
                                phase_offset_matched_adapter_ &&
                                phase_offset_matched_adapter_->requestRecenter()) {
                                ROS_WARN_THROTTLE(
                                    1.0,
                                    "[GVF][H2][RECOVERY] closed successor stage denied; continuous recenter requested");
                            }
                            ROS_WARN_THROTTLE(
                                1.0,
                                "[GVF][CLOSED_PHASE_V2] C2/Section connector unavailable; keep old frontend");
                        } else if (section_handoff_required) {
                            installed = true;
                        } else {
                            nav_msgs::Path committed_path_msg;
                            if (installPlannerOnlyFrontend(
                                    pm, install_traj, install_vel, install_time,
                                    install_w, install_continuous_path,
                                    current_time, fsm_phase,
                                    planner_only_prefix_source,
                                    planner_only_prefix_end_w,
                                    planner_only_execution_generation,
                                    committed_path_msg)) {
                                path_pub.publish(committed_path_msg);
                                installed = true;
                                if (closed_ref_has_pending_goal_) {
                                    closed_ref_accepted_goal_w_ =
                                        closed_ref_pending_goal_w_;
                                    closed_ref_accepted_lookahead_w_ =
                                        closed_ref_pending_lookahead_w_;
                                    closed_ref_has_accepted_goal_ = true;
                                    closed_ref_accepted_from_bypass_ =
                                        closed_ref_pending_from_bypass_;
                                }
                                ROS_WARN(
                                    "[GVF][CLOSED_PHASE_V2][SWITCH] accepted_new=1 phase_before=%.6f phase_after=%.6f path_start_w=%.3f path_end_w=%.3f anchor_idx=%d points=%d c2=%d exact_path=1",
                                    phase_at_switch, fsm_phase.w,
                                    install_w.front(), install_w.back(),
                                    current_traj_index_,
                                    static_cast<int>(install_traj.rows()),
                                    closed_phase_c2_enabled_ ? 1 : 0);
                            }
                        }
                    }
                }

                if (!installed) {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[GVF][CLOSED_PHASE_V2] replan failed; keep current frontend and phase_w=%.3f",
                        fsm_phase.w);
                }
                closed_phase_has_pending_path_end_w_ = false;
                last_replan_time_ = current_time;
                changeFSMExecState(EXEC_TRAJ, "closed_phase_v2 replan");
                break;
            }

            // Point-phase replanning keeps the original acceptance, arc
            // length mapping, future seam/C2 connector and anchor bookkeeping.
            // A live nonzero Section offset takes the staged handoff path;
            // neutral candidates use the planner-only CAS above.
            if (pointPhaseV2Active()) {
                const Eigen::MatrixXd old_traj = pm.last_traj;
                const Eigen::MatrixXd old_vel = pm.last_vel;
                Eigen::MatrixXd cand_traj, cand_vel;
                Eigen::VectorXd cand_time;
                UniformBspline cand_spline;
                int new_i0 = 0;
                bool installed = false;

                if (astaropt(current_pos, cand_traj, cand_vel, new_i0,
                             cand_time, &cand_spline)) {
                    bool accept_new = true;
                    std::string reason = "accept_default";
                    std::shared_ptr<const ContinuousPhasePath> old_path =
                        pm.gvf_ ? pm.gvf_->getContinuousPhasePath()
                                : std::shared_ptr<const ContinuousPhasePath>();
                    if (old_traj.rows() > 0 && old_vel.rows() == old_traj.rows()) {
                        double accepted_path_w_end =
                            old_path && !old_path->empty()
                                ? old_path->endW()
                                : std::numeric_limits<double>::quiet_NaN();
                        accept_new = shouldAcceptCandidate(
                            old_traj, old_vel, pm.last_traj_time_,
                            current_traj_index_, cand_traj, cand_vel,
                            cand_time, new_i0, pm.goal_pt, reason,
                            fsm_phase.w, accepted_path_w_end);
                    }

                    if (accept_new) {
                        new_i0 = std::max(
                            0, std::min(new_i0,
                                static_cast<int>(cand_traj.rows()) - 2));
                        const double raw_t_anchor = cand_time(new_i0);
                        double remaining_length = 0.0;
                        if (cand_time.size() == cand_traj.rows() &&
                            cand_time.allFinite() &&
                            std::isfinite(raw_t_anchor) &&
                            raw_t_anchor >= cand_spline.t_range(0) - 1e-8 &&
                            raw_t_anchor < cand_spline.t_range(1) - 1e-8 &&
                            ContinuousPhasePath::measureBsplineArcLength(
                                cand_spline, raw_t_anchor,
                                cand_spline.t_range(1), remaining_length) &&
                            remaining_length > 1e-3) {
                            const double phase_at_switch = fsm_phase.w;
                            const double path_end_w =
                                phase_at_switch + remaining_length;
                            std::vector<double> candidate_w;
                            const bool build_samples_ok =
                                pm.gvf_ && buildGlobalPhaseSamples(
                                    cand_traj, new_i0, phase_at_switch,
                                    path_end_w, candidate_w);
                            if (build_samples_ok) {
                                Eigen::MatrixXd install_traj;
                                Eigen::MatrixXd install_vel;
                                Eigen::VectorXd install_time;
                                std::vector<double> install_w;
                                std::shared_ptr<const ContinuousPhasePath>
                                    install_continuous_path;

                                bool section_handoff_required = false;
                                if (phase_offset_matched_adapter_ &&
                                    matched_config_.mode ==
                                        PhaseOffsetMatchedMode::MANUAL) {
                                    std::lock_guard<std::mutex> runtime_lock(
                                        phase_offset_matched_adapter_->runtime_command_mutex_);
                                    section_handoff_required =
                                        phase_offset_matched_adapter_->runtime_ &&
                                        phase_offset_matched_adapter_->runtime_->retainedDelta() != 0.0;
                                }
                                const std::uint64_t captured_execution_generation =
                                    phase_offset_matched_adapter_
                                        ? phase_offset_matched_adapter_->executionGeneration()
                                        : 0U;
                                if (section_handoff_required) {
                                    const SectionPathBundlePtr committed_bundle =
                                        phase_offset_matched_adapter_
                                            ? phase_offset_matched_adapter_->captureSectionBundle()
                                            : SectionPathBundlePtr();
                                    if (!committed_bundle ||
                                        committed_bundle->task_generation !=
                                            captured_execution_generation ||
                                        !committed_bundle->path ||
                                        committed_bundle->path->empty()) {
                                        old_path.reset();
                                    } else {
                                        old_path = committed_bundle->path;
                                    }
                                }
                                double future_seam_w = 0.0;
                                const bool future_seam_available =
                                    captured_execution_generation != 0U &&
                                    plannerOnlyFutureSeam(
                                        old_path, phase_at_switch,
                                        matched_config_.tube.min_certified_forward_w,
                                        future_seam_w);
                                const bool frontend_exhausted_w =
                                    !future_seam_available ||
                                    (old_path && !old_path->empty() &&
                                     old_path->endW() - phase_at_switch <=
                                         cmd_governor_l_max_ +
                                             std::max(
                                                 0.0,
                                                 switch_governor_path_margin_w_));
                                bool frontend_ready = false;
                                bool used_mapped_fallback = false;
                                std::shared_ptr<const ContinuousPhasePath>
                                    planner_only_prefix_source;
                                double planner_only_prefix_end_w = phase_at_switch;
                                std::uint64_t planner_only_execution_generation = 0U;

                                if (section_handoff_required) {
                                    double handoff_switch_w = future_seam_w;
                                    double copied_prefix_start_w =
                                        phase_at_switch;
                                    double copied_prefix_end_w = future_seam_w;
                                    bool handoff_seam_available =
                                        future_seam_available;
                                    if (!handoff_seam_available && old_path &&
                                        !old_path->empty() &&
                                        phase_at_switch >=
                                            old_path->startW() - 1e-9 &&
                                        phase_at_switch <=
                                            old_path->endW() +
                                                kExhaustedFrontendSeamSlackW) {
                                        // Exhausted frontend: use a backdated
                                        // C2 seam and the full source tail as
                                        // the copied-prefix window while
                                        // retaining the Section-owned delta.
                                        const double seam_anchor_w = std::min(
                                            phase_at_switch,
                                            old_path->endW());
                                        handoff_switch_w = std::max(
                                            old_path->startW(),
                                            seam_anchor_w -
                                                kExhaustedFrontendSeamBackW);
                                        copied_prefix_start_w =
                                            handoff_switch_w;
                                        copied_prefix_end_w =
                                            old_path->endW();
                                        handoff_seam_available =
                                            copied_prefix_end_w >
                                            copied_prefix_start_w + 1e-6;
                                    }
                                    if (handoff_seam_available) {
                                        frontend_ready =
                                            buildPhaseC2Frontend(
                                                pm, phase_at_switch,
                                                handoff_switch_w, old_path,
                                                path_end_w, new_i0,
                                                cand_spline, cand_traj,
                                                cand_vel, cand_time,
                                                candidate_w, install_traj,
                                                install_vel, install_time,
                                                install_w,
                                                install_continuous_path);
                                    }
                                    if (!frontend_ready &&
                                        frontend_exhausted_w &&
                                        old_path && !old_path->empty()) {
                                        frontend_ready =
                                            buildMappedPhaseFrontend(
                                                phase_at_switch, path_end_w,
                                                false, new_i0, cand_spline,
                                                cand_traj, cand_time,
                                                install_traj, install_vel,
                                                install_time, install_w,
                                                install_continuous_path);
                                        used_mapped_fallback = frontend_ready;
                                        if (!frontend_ready) {
                                            ROS_WARN_THROTTLE(
                                                1.0,
                                                "[GVF][POINT_PHASE_V2] mapped fallback frontend unavailable phase=%.3f; keeping current frontend",
                                                phase_at_switch);
                                        }
                                    }
                                    if (frontend_ready) {
                                        if (used_mapped_fallback) {
                                            const bool bundle_built =
                                                buildAndStageSectionBundle(
                                                    pm, fsm_phase,
                                                    install_continuous_path,
                                                    old_path,
                                                    copied_prefix_start_w,
                                                    copied_prefix_end_w);
                                            SectionPathBundlePtr bundle;
                                            if (bundle_built) {
                                                bundle = phase_offset_matched_adapter_
                                                             ->capturePendingSectionCandidate()
                                                             .bundle;
                                            }
                                            if (bundle && bundle->path) {
                                                nav_msgs::Path fallback_msg;
                                                if (installPlannerOnlyFrontend(
                                                        pm, install_traj,
                                                        install_vel,
                                                        install_time,
                                                        install_w,
                                                        bundle->path,
                                                        current_time,
                                                        fsm_phase, old_path,
                                                        old_path->endW(),
                                                        captured_execution_generation,
                                                        fallback_msg, true,
                                                        bundle)) {
                                                    path_pub.publish(
                                                        fallback_msg);
                                                    installed = true;
                                                    ROS_WARN(
                                                        "[GVF][POINT_PHASE_V2][MAPPED_SECTION_FALLBACK] "
                                                        "installed mapped path+bundle "
                                                        "phase=%.3f path=[%.3f,%.3f]",
                                                        fsm_phase.w,
                                                        install_w.front(),
                                                        install_w.back());
                                                } else {
                                                    frontend_ready = false;
                                                    if (phase_offset_matched_adapter_) {
                                                        // A tube that cannot be
                                                        // rebuilt must release its
                                                        // authority, not freeze
                                                        // navigation: the next
                                                        // cycle then lands on the
                                                        // neutral mapped frontend.
                                                        phase_offset_matched_adapter_
                                                            ->requestRecenter();
                                                    }
                                                    ROS_WARN_THROTTLE(
                                                        1.0,
                                                        "[GVF][POINT_PHASE_V2] mapped fallback install rejected phase=%.3f; tube authority released",
                                                        fsm_phase.w);
                                                }
                                            } else {
                                                frontend_ready = false;
                                                if (phase_offset_matched_adapter_) {
                                                    phase_offset_matched_adapter_
                                                        ->requestRecenter();
                                                }
                                                ROS_WARN_THROTTLE(
                                                    1.0,
                                                    "[GVF][POINT_PHASE_V2] section bundle unavailable phase=%.3f bundle_built=%d; tube authority released",
                                                    fsm_phase.w,
                                                    (int)bundle_built);
                                            }
                                        } else {
                                            frontend_ready =
                                                stageSectionPathHandoff(
                                                    fsm_phase, pm,
                                                    install_continuous_path,
                                                    install_traj, install_vel,
                                                    install_time, install_w,
                                                    old_path,
                                                    copied_prefix_start_w,
                                                    copied_prefix_end_w);
                                        }
                                    }
                                    if (!frontend_ready &&
                                        phase_offset_matched_adapter_) {
                                        phase_offset_matched_adapter_->requestRecenter();
                                    }
                                } else if (point_phase_c2_enabled_) {
                                    frontend_ready = future_seam_available &&
                                        buildPhaseC2Frontend(
                                            pm, phase_at_switch, future_seam_w,
                                            old_path, path_end_w, new_i0,
                                            cand_spline, cand_traj, cand_vel,
                                            cand_time, candidate_w, install_traj,
                                            install_vel, install_time, install_w,
                                            install_continuous_path);
                                    if (frontend_ready) {
                                        planner_only_prefix_source = old_path;
                                        planner_only_prefix_end_w = future_seam_w;
                                        planner_only_execution_generation =
                                            captured_execution_generation;
                                    } else {
                                        planner_only_prefix_source = old_path;
                                        planner_only_prefix_end_w =
                                            old_path && !old_path->empty()
                                                ? old_path->endW()
                                                : phase_at_switch;
                                        frontend_ready = buildPhaseC2Frontend(
                                            pm, phase_at_switch, phase_at_switch,
                                            old_path, path_end_w, new_i0,
                                            cand_spline, cand_traj, cand_vel,
                                            cand_time, candidate_w, install_traj,
                                            install_vel, install_time, install_w,
                                            install_continuous_path);
                                        planner_only_execution_generation =
                                            captured_execution_generation;
                                        if (!frontend_ready &&
                                            frontend_exhausted_w) {
                                            // No future seam and no C2 connector:
                                            // the frontend is exhausted, and this
                                            // switch carries no lateral tube
                                            // authority (retained delta is zero),
                                            // so the mapped frontend is the safe
                                            // landing.  Install it rather than
                                            // holding on an exhausted frontend.
                                            frontend_ready =
                                                buildMappedPhaseFrontend(
                                                    phase_at_switch, path_end_w,
                                                    false, new_i0, cand_spline,
                                                    cand_traj, cand_time,
                                                    install_traj, install_vel,
                                                    install_time, install_w,
                                                    install_continuous_path);
                                            if (frontend_ready) {
                                                planner_only_prefix_source =
                                                    old_path;
                                                planner_only_prefix_end_w =
                                                    path_end_w;
                                            }
                                        }
                                    }
                                } else {
                                    frontend_ready = buildMappedPhaseFrontend(
                                        phase_at_switch, path_end_w, false, new_i0,
                                        cand_spline, cand_traj, cand_time,
                                        install_traj, install_vel, install_time,
                                        install_w, install_continuous_path);
                                    planner_only_execution_generation =
                                        captured_execution_generation;
                                }

                                if (frontend_ready && !install_w.empty() &&
                                    install_continuous_path) {
                                    if (section_handoff_required) {
                                        installed = true;
                                    } else {
                                        nav_msgs::Path committed_path_msg;
                                        if (installPlannerOnlyFrontend(
                                                pm, install_traj, install_vel,
                                                install_time, install_w,
                                                install_continuous_path,
                                                current_time, fsm_phase,
                                                planner_only_prefix_source,
                                                planner_only_prefix_end_w,
                                                planner_only_execution_generation,
                                                committed_path_msg)) {
                                            path_pub.publish(committed_path_msg);
                                            installed = true;
                                            ROS_WARN(
                                                "[GVF][POINT_PHASE_V2][SWITCH] accepted_new=1 reason=%s phase_before=%.6f phase_after=%.6f path_start_w=%.3f path_end_w=%.3f points=%d c2=%d exact_path=1",
                                                reason.c_str(), phase_at_switch,
                                                fsm_phase.w, install_w.front(),
                                                install_w.back(),
                                                static_cast<int>(install_traj.rows()),
                                                point_phase_c2_enabled_ ? 1 : 0);
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        ROS_WARN(
                            "[GVF][POINT_PHASE_V2][SWITCH] accepted_new=0 reason=%s phase_w=%.3f; keep current frontend",
                            reason.c_str(), fsm_phase.w);
                    }
                }

                if (!installed) {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[GVF][POINT_PHASE_V2] replan not installed; keep current frontend and phase_w=%.3f",
                        fsm_phase.w);
                }
                last_replan_time_ = current_time;
                changeFSMExecState(EXEC_TRAJ, "point_phase_v2 replan");
                break;
            }

            const Eigen::MatrixXd old_traj = pm.last_traj;
            const Eigen::MatrixXd old_vel  = pm.last_vel;
            Eigen::MatrixXd cand_traj, cand_vel;
            Eigen::VectorXd cand_time;
            int new_i0 = 0;
            const gvf::ReparamCacheSnapshot legacy_cache = pm.gvf_
                ? pm.gvf_->captureReparamCacheSnapshot()
                : gvf::ReparamCacheSnapshot();

            if (pm.last_traj.rows() > 0) {
                int progress_anchor_idx = std::max(0, std::min(current_traj_index_, (int)pm.last_traj.rows() - 1));
                if (legacy_cache.ready &&
                    legacy_cache.w.size() == static_cast<size_t>(pm.last_traj.rows())) {
                    auto it = std::lower_bound(
                        legacy_cache.w.begin(), legacy_cache.w.end(), progress_w_);
                    if (it == legacy_cache.w.end()) {
                        progress_anchor_idx = pm.last_traj.rows() - 1;
                    } else {
                        progress_anchor_idx = std::distance(legacy_cache.w.begin(), it);
                        if (progress_anchor_idx > 0) {
                            const double w_hi = legacy_cache.w[progress_anchor_idx];
                            const double w_lo = legacy_cache.w[progress_anchor_idx - 1];
                            if (std::abs(progress_w_ - w_lo) <= std::abs(w_hi - progress_w_)) {
                                progress_anchor_idx -= 1;
                            }
                        }
                    }
                }

                int best = progress_anchor_idx;
                double best_d = std::numeric_limits<double>::infinity();
                for (int i = progress_anchor_idx; i < pm.last_traj.rows(); ++i) {
                    Eigen::Vector3d traj_point(pm.last_traj(i,0), pm.last_traj(i,1), pm.last_traj(i,2));
                    double d = (traj_point - current_pos).squaredNorm();
                    if (d < best_d) { best_d = d; best = i; }
                }
                current_traj_index_ = best;
                new_i0 = best;
            } else {
                current_traj_index_ = 0;
            }

            auto computeOldPathAnchor = [&](double fallback_w) {
                std::pair<double, int> result(fallback_w, std::max(0, current_traj_index_));
                if (!(legacy_cache.ready &&
                      legacy_cache.w.size() == static_cast<size_t>(old_traj.rows()) &&
                      old_traj.rows() > 0)) {
                    return result;
                }

                int anchor_idx = std::max(0, std::min(current_traj_index_, (int)old_traj.rows() - 1));
                double best_d = std::numeric_limits<double>::infinity();
                for (int i = anchor_idx; i < old_traj.rows(); ++i) {
                    Eigen::Vector3d traj_point(old_traj(i,0), old_traj(i,1), old_traj(i,2));
                    double d = (traj_point - current_pos).squaredNorm();
                    if (d < best_d) {
                        best_d = d;
                        anchor_idx = i;
                    }
                }
                result.first = legacy_cache.w[anchor_idx];
                result.second = anchor_idx;
                return result;
            };

            auto computeKeepPathAnchor = [&]() {
                std::pair<double, int> result(0.0, std::max(0, current_traj_index_));
                if (!(legacy_cache.ready && !legacy_cache.w.empty())) {
                    return result;
                }
                result.first = legacy_cache.w.front();
                return result;
            };

            if (astaropt(current_pos, cand_traj, cand_vel, new_i0, cand_time)) {
                bool accept_new = true;
                std::string reason = "accept_default";

                if (old_traj.rows() > 0 && old_vel.rows() == old_traj.rows()) {
                    double accepted_path_w_end = std::numeric_limits<double>::quiet_NaN();
                    if (legacy_cache.ready && !legacy_cache.w.empty()) {
                        accepted_path_w_end = legacy_cache.w.back();
                    }
                    accept_new = shouldAcceptCandidate(old_traj, old_vel, pm.last_traj_time_, current_traj_index_, cand_traj, cand_vel,
                                                       cand_time, new_i0, pm.goal_pt, reason,
                                                       progress_w_, accepted_path_w_end);
                }
                Eigen::Vector3d v_ref_start = Eigen::Vector3d::Zero();
                if (cand_vel.rows() > 0) {
                    const int v_idx = std::max(0, std::min(new_i0, (int)cand_vel.rows() - 1));
                    v_ref_start = cand_vel.row(v_idx).transpose();
                }
                const Eigen::Vector3d v_odom = odom_vel_est_;
                double cos_start_odom = 0.0;
                if (v_ref_start.norm() > 1e-6 && v_odom.norm() > 1e-6) {
                    cos_start_odom = v_ref_start.normalized().dot(v_odom.normalized());
                }
                if (accept_new && enable_circle_reference_test_ && circle_reference_ready_ && closed_ref_has_pending_goal_) {
                    closed_ref_accepted_goal_w_ = closed_ref_pending_goal_w_;
                    closed_ref_accepted_lookahead_w_ = closed_ref_pending_lookahead_w_;
                    closed_ref_has_accepted_goal_ = true;
                    closed_ref_accepted_from_bypass_ = closed_ref_pending_from_bypass_;
                }
                ROS_WARN("[GVF][SWITCH_OBS] accept_reason=%s accepted_new=%d pending_goal_w=%.3f accepted_goal_w=%.3f pending_lookahead=%.3f accepted_lookahead=%.3f v_ref_start=(%.3f,%.3f,%.3f) v_odom=(%.3f,%.3f,%.3f) cos_start_odom=%.3f",
                         reason.c_str(), accept_new ? 1 : 0,
                         closed_ref_pending_goal_w_, closed_ref_accepted_goal_w_,
                         closed_ref_pending_lookahead_w_, closed_ref_accepted_lookahead_w_,
                         v_ref_start.x(), v_ref_start.y(), v_ref_start.z(),
                         v_odom.x(), v_odom.y(), v_odom.z(), cos_start_odom);
                if (accept_new) {
                    const auto anchor = computeOldPathAnchor(progress_w_);
                    const double w_anchor = anchor.first;
                    const int anchor_idx = anchor.second;

                    ROS_WARN("[GVF][ANCHOR] curr_i0=%d anchor_idx=%d w_anchor=%.3f old_rows=%d new_i0=%d", 
                             current_traj_index_, anchor_idx, w_anchor, (int)old_traj.rows(), new_i0);
                    ROS_INFO("[GVF][SWITCH] progress_w=%.3f w_anchor=%.3f delta=%.3f curr_i0=%d anchor_idx=%d new_i0=%d",
                             progress_w_, w_anchor, w_anchor - progress_w_, current_traj_index_, anchor_idx, new_i0);

                    pm.last_traj = cand_traj;
                    pm.last_vel = cand_vel;
                    pm.last_traj_time_ = cand_time;
                    current_traj_index_ = new_i0;
                    last_switch_time_ = current_time;
                    cmd_switch_motion_limit_until_ = ros::Time::now() + ros::Duration(std::max(0.0, cmd_switch_motion_limit_time_));
                    if (pm.gvf_) {
                        pm.gvf_->setNextPathWAnchor(w_anchor);
                    }
                    // The frontend just changed, so the stored phase must move
                    // into the NEW path's w domain.  Without this realignment
                    // the manager keeps a phase that refers to the old
                    // trajectory; the next lookahead then resolves to a point
                    // essentially on the vehicle (measured cmd_dist = 0.023 m
                    // with a 1.6 m lookahead) and the UAV freezes in place.
                    // This mirrors what the phase-V2 install path does with
                    // initial_phase_w.
                    publishAuthoritativePhase(
                        w_anchor, true, closed_phase_acquired_);
                    progress_w_ = w_anchor;
                    progress_initialized_ = true;
                    if (pm.gvf_) {
                        pm.gvf_->setVisualizationProgressW(w_anchor);
                    }
                    cmd_governor_normal_state_.setZero();
                    cmd_governor_initialized_ = false;
                    cmd_governor_last_l_ = 0.0;
                } else {
                    const auto anchor = computeKeepPathAnchor();
                    const double w_anchor = anchor.first;
                    const int anchor_idx = anchor.second;
                    if (pm.gvf_) {
                        pm.gvf_->setNextPathWAnchor(w_anchor);
                    }
                    ROS_WARN("[GVF][ANCHOR][KEEP] curr_i0=%d anchor_idx=%d start_w=%.3f old_rows=%d", 
                             current_traj_index_, anchor_idx, w_anchor, (int)old_traj.rows());
                }
            }
            else {
                ROS_WARN_THROTTLE(1.0, "[GVF] REPLAN_TRAJ: plan failed, keep old");
                const auto anchor = computeKeepPathAnchor();
                const double w_anchor = anchor.first;
                const int anchor_idx = anchor.second;
                if (pm.gvf_) {
                    pm.gvf_->setNextPathWAnchor(w_anchor);
                }
                ROS_WARN("[GVF][ANCHOR][KEEP] curr_i0=%d anchor_idx=%d start_w=%.3f old_rows=%d", 
                         current_traj_index_, anchor_idx, w_anchor, (int)old_traj.rows());
            }

            publishPathMsg(pm.last_traj, pm.last_vel);
            last_replan_time_ = current_time;

            changeFSMExecState(EXEC_TRAJ, "FSM");

            break;
        }
    }
}
}
