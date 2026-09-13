#include "bspline_race/integration/phase_offset_matched_adapter.h"
#include <bspline_race/integration/phase_offset_executed_reference_query.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

namespace FLAG_Race {
namespace {

bool IsFinite(const double value) { return std::isfinite(value); }
bool IsFinite(const Eigen::Vector3d& value) { return value.allFinite(); }

bool HasUsablePathInput(
    const phase_offset_core::PathDifferentialState& path) {
  return path.valid && IsFinite(path.w) && IsFinite(path.p) &&
      IsFinite(path.p_w) && IsFinite(path.p_ww);
}

bool HasUsableLegacyInput(const LegacyGuidanceSnapshot& legacy) {
  return legacy.valid && IsFinite(legacy.v_cmd) && IsFinite(legacy.w_dot) &&
      IsFinite(legacy.e_parallel) && IsFinite(legacy.e_perp) &&
      IsFinite(legacy.ref_pt) && IsFinite(legacy.tangent);
}

phase_offset_navigation::PhaseOffsetRuntimeConfig MakeRuntimeConfig(
    const PhaseOffsetMatchedAdapterConfig& config) {
  phase_offset_navigation::PhaseOffsetRuntimeConfig runtime;
  auto& manual = runtime.manual;
  manual.amplitude = config.amplitude;
  manual.profile_period = config.profile_period;
  manual.delta_tracking_gain = config.delta_tracking_gain;
  manual.u_w_amplitude = config.u_w_amplitude;
  manual.u_w_abs_max = config.u_w_abs_max;
  manual.u_delta_abs_max = config.u_delta_abs_max;
  manual.u_w_rate_max = config.u_w_rate_max;
  manual.u_delta_rate_max = config.u_delta_rate_max;
  manual.phase_dot_min = config.phase_dot_min;
  manual.tangent_speed_min = config.tangent_speed_min;
  manual.preflight_sample_step_w = config.preflight_sample_step_w;
  runtime.tube.source = config.tube_source;
  runtime.tube.invariant_gain = config.tube.invariant_gain;
  runtime.tube.interior_margin = config.tube.interior_margin;
  runtime.tube.tracking_error_bound =
      config.tube.cross_section.margins.tracking_error_bound;
  runtime.tube.regularity_margin =
      config.tube.cross_section.regularity_margin;
  runtime.tube.minimum_reference_speed =
      config.tube.cross_section.minimum_reference_speed;
  return runtime;
}

void SetGuidance(const guidance::IsfGuidance& base,
                 const phase_offset_core::MatchedPortOutput& matched,
                 guidance::IsfGuidance& output) {
  output = base;
  output.v_cmd = matched.v_cmd;
  output.w_dot = matched.w_dot;
  output.valid = matched.valid;
  output.invalid_reason = matched.invalid_reason;
}

}  // namespace

const char* phaseOffsetCoordinationBackendName(
    const PhaseOffsetCoordinationBackend backend) {
  switch (backend) {
    case PhaseOffsetCoordinationBackend::DISABLED:
      return "disabled";
    case PhaseOffsetCoordinationBackend::D1B:
      return "d1b";
    case PhaseOffsetCoordinationBackend::SPH:
      return "sph";
  }
  return "disabled";
}

PhaseOffsetMatchedAdapterConfig PhaseOffsetMatchedAdapter::loadConfig(
    ros::NodeHandle& nh, const PhaseOffsetMatchedMode mode) {
  PhaseOffsetMatchedAdapterConfig config;
  config.mode = mode;
  nh.param("phase_offset/active_equivalence_tolerance",
           config.equivalence_tolerance, 1e-10);
  nh.param("phase_offset/active_required_consecutive_cycles",
           config.warmup_cycles, 100);
  nh.param("phase_offset/measurement/enable",
           config.measurement_enabled, false);
  nh.param<std::string>("phase_offset/measurement/tube_due_csv_path",
                        config.measurement_tube_due_csv_path,
                        std::string());
  if (mode == PhaseOffsetMatchedMode::ACTIVE) return config;

  nh.param("phase_offset/manual/amplitude", config.amplitude, 0.10);
  nh.param("phase_offset/manual/observe_only", config.observe_only, true);
  nh.param("phase_offset/manual/profile_period", config.profile_period, 10.0);
  nh.param("phase_offset/manual/warmup_cycles", config.warmup_cycles, 100);
  nh.param("phase_offset/manual/delta_tracking_gain",
           config.delta_tracking_gain, 3.0);
  nh.param("phase_offset/manual/u_w_amplitude", config.u_w_amplitude, 0.05);
  nh.param("phase_offset/manual/u_w_abs_max", config.u_w_abs_max, 0.12);
  nh.param("phase_offset/manual/u_delta_abs_max",
           config.u_delta_abs_max, 0.25);
  nh.param("phase_offset/manual/u_w_rate_max",
           config.u_w_rate_max, 0.60);
  nh.param("phase_offset/manual/u_delta_rate_max",
           config.u_delta_rate_max, 1.20);
  nh.param("phase_offset/manual/phase_dot_min", config.phase_dot_min, 0.02);
  nh.param("phase_offset/manual/tangent_speed_min",
           config.tangent_speed_min, 0.02);
  nh.param("phase_offset/manual/preflight_sample_step_w",
           config.preflight_sample_step_w, 0.10);

  const auto read_required_double = [&nh](const std::string& key,
                                           double& value) {
    return nh.hasParam(key) && nh.getParam(key, value);
  };
  bool policy_complete = true;
  policy_complete = read_required_double(
      "phase_offset/normal_preview/preview_horizon_w",
      config.normal_preview_policy.preview_horizon_w) && policy_complete;
  policy_complete = read_required_double(
      "phase_offset/normal_preview/sample_spacing_w",
      config.normal_preview_policy.sample_spacing_w) && policy_complete;
  policy_complete = read_required_double(
      "phase_offset/normal_preview/lower_nu",
      config.normal_preview_policy.lower_nu) && policy_complete;
  policy_complete = read_required_double(
      "phase_offset/normal_preview/upper_nu",
      config.normal_preview_policy.upper_nu) && policy_complete;
  policy_complete = read_required_double(
      "phase_offset/normal_preview/b_tight",
      config.normal_preview_policy.b_tight) && policy_complete;
  policy_complete = read_required_double(
      "phase_offset/normal_preview/b_open",
      config.normal_preview_policy.b_open) && policy_complete;
  if (policy_complete && config.normal_preview_policy.numericValid()) {
    config.normal_preview_policy_explicit = true;
  }

  // One production Section policy.  The source selector remains a value-only
  // compatibility field for existing launch files; Section construction does
  // not dispatch through a legacy tube backend.
  config.tube_source = phase_offset_navigation::TubeSource::ESDF;
  const std::string nominal_key = "phase_offset/tube/nominal_half_width";
  if (nh.hasParam(nominal_key)) {
    nh.param(nominal_key, config.tube.cross_section.nominal_half_width, 1.0);
    config.tube.cross_section.nominal_width_source =
        phase_offset_navigation::TubeNominalWidthSource::EXPLICIT_PARAMETER;
  }
  nh.param("phase_offset/tube/sample_step_w", config.tube.sample_step_w, 0.10);
  nh.param("phase_offset/tube/lookahead_w", config.tube.lookahead_w, 2.0);
  nh.param("phase_offset/tube/back_w", config.tube.back_w, 0.20);
  nh.param("phase_offset/tube/min_certified_forward_w",
           config.tube.min_certified_forward_w, 0.40);
  nh.param("phase_offset/tube/invariant_gain", config.tube.invariant_gain, 1.0);
  nh.param("phase_offset/tube/interior_margin",
           config.tube.interior_margin, 0.0);
  nh.param("phase_offset/tube/update_period", config.tube_update_period, 0.10);
  nh.param("phase_offset/tube/environment_search_extent",
           config.tube.cross_section.search_extent, 3.0);
  nh.param("phase_offset/tube/raw_regularity_margin",
           config.tube.cross_section.regularity_margin, 0.10);
  nh.param("planning/safe_distance",
           config.tube.cross_section.planner_safe_distance, 0.4);
  nh.param("phase_offset/tube/raw_tracking_error_bound",
           config.tube.cross_section.margins.tracking_error_bound, 0.15);
  nh.param("phase_offset/tube/cloud_obstacle_set_complete",
           config.cloud_obstacle_set_complete, false);
  nh.param<std::string>("phase_offset/frame_id", config.frame_id,
                        std::string("world"));

  // The Section corridor reserves its own clearance between the tube wall and
  // the first occupied voxel.  It defaults to the planner's own safe distance
  // so an existing deployment keeps its previous geometry, but it can be
  // lowered on its own when the corridor should reach closer to the raw free
  // space without relaxing the planner's B-spline obstacle preference.
  double section_clearance = config.tube.cross_section.planner_safe_distance;
  nh.param("phase_offset/tube/section_clearance", section_clearance,
           section_clearance);
  if (!std::isfinite(section_clearance) || section_clearance < 0.0) {
    section_clearance = config.tube.cross_section.planner_safe_distance;
  }
  config.section_build.clearance = section_clearance;
  config.section_build.half_width =
      config.tube.cross_section.search_extent;
  config.section_build.minimum_reference_speed =
      config.tube.cross_section.minimum_reference_speed;
  nh.param("phase_offset/tube/min_regularity_ratio",
           config.section_build.min_regularity_ratio, 0.35);
  nh.param("phase_offset/tube/curvature_epsilon",
           config.section_build.curvature_epsilon,
           config.tube.cross_section.curvature_epsilon);
  config.section_build.max_step_w =
      std::max(config.section_build.min_step_w, config.tube.sample_step_w);
  // Section work budgets are exposed so the deployment can match its local
  // view size without touching the accepted geometry defaults.  They are
  // work limits only: exceeding them reports a truthful PARTIAL prefix and
  // never widens or narrows the tube geometry itself.
  int section_max_obstacle_checks = static_cast<int>(
      config.section_build.max_obstacle_checks);
  if (nh.param("phase_offset/tube/section_max_obstacle_checks",
               section_max_obstacle_checks, section_max_obstacle_checks) &&
      section_max_obstacle_checks > 0) {
    config.section_build.max_obstacle_checks =
        static_cast<std::size_t>(section_max_obstacle_checks);
  }
  int section_max_obstacles =
      static_cast<int>(config.section_build.max_obstacles);
  if (nh.param("phase_offset/tube/section_max_obstacles",
               section_max_obstacles, section_max_obstacles) &&
      section_max_obstacles > 0) {
    config.section_build.max_obstacles =
        static_cast<std::size_t>(section_max_obstacles);
  }
  int section_max_cells = static_cast<int>(config.section_build.max_cells);
  if (nh.param("phase_offset/tube/section_max_cells", section_max_cells,
               section_max_cells) &&
      section_max_cells > 0) {
    config.section_build.max_cells =
        static_cast<std::size_t>(section_max_cells);
  }
  return config;
}

PhaseOffsetMatchedAdapter::PhaseOffsetMatchedAdapter(
    const PhaseOffsetMatchedAdapterConfig& config)
    : config_(config) {
  if (config_.frame_id.empty()) config_.frame_id = "world";
  configuration_valid_ =
      (config_.mode == PhaseOffsetMatchedMode::ACTIVE ||
       config_.mode == PhaseOffsetMatchedMode::MANUAL) &&
      IsFinite(config_.equivalence_tolerance) &&
      config_.equivalence_tolerance >= 0.0 &&
      config_.warmup_cycles >= 100;
  if (config_.mode == PhaseOffsetMatchedMode::MANUAL) {
    runtime_.reset(new phase_offset_navigation::PhaseOffsetRuntime(
        MakeRuntimeConfig(config_)));
    configuration_valid_ = configuration_valid_ &&
        runtime_->sectionConfigurationValid() &&
        config_.normal_preview_policy_explicit &&
        config_.normal_preview_policy.numericValid() &&
        config_.section_build.max_obstacle_checks > 0U &&
        config_.section_build.max_obstacles > 0U &&
        IsFinite(config_.tube_update_period) &&
        config_.tube_update_period > 0.0;
  }
}

PhaseOffsetMatchedAdapter::~PhaseOffsetMatchedAdapter() { shutdown(); }

bool PhaseOffsetMatchedAdapter::configurationValid() const {
  return configuration_valid_;
}

bool PhaseOffsetMatchedAdapter::requiresPathSamples() const { return false; }

double PhaseOffsetMatchedAdapter::sampleStepW() const {
  return config_.preflight_sample_step_w;
}

bool PhaseOffsetMatchedAdapter::requiresTubeTimer() const { return false; }

bool PhaseOffsetMatchedAdapter::requiresAuthoritativeOffsetHandoff() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return requiresAuthoritativeOffsetHandoffLocked();
}

bool PhaseOffsetMatchedAdapter::requiresAuthoritativeOffsetHandoffLocked() const {
  return config_.mode == PhaseOffsetMatchedMode::MANUAL && runtime_ &&
      (static_cast<bool>(section_bundle_) ||
       static_cast<bool>(pending_section_prepared_step_) ||
       runtime_->hasExecutedOffsetAuthority());
}

bool PhaseOffsetMatchedAdapter::requestRecenter() {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  if (!runtime_ || config_.mode != PhaseOffsetMatchedMode::MANUAL ||
      (!section_bundle_ && !pending_section_prepared_step_ &&
       !runtime_->hasExecutedOffsetAuthority())) {
    return false;
  }
  runtime_->requestRecenter();
  return true;
}

bool PhaseOffsetMatchedAdapter::recenterRequested() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return runtime_ && runtime_->recenterRequested();
}

void PhaseOffsetMatchedAdapter::latchFailure(
    const phase_offset_navigation::ControlFailureReason reason) {
  if (!failure_latched_) control_failure_reason_ = reason;
  failure_latched_ = true;
  clearPendingPositionCommandLocked();
}

void PhaseOffsetMatchedAdapter::clearPendingPositionCommandLocked() {
  pending_section_prepared_step_.reset();
  pending_section_reference_query_.reset();
  pending_section_bundle_.reset();
  pending_section_source_path_.reset();
  pending_section_copied_prefix_start_w_ =
      std::numeric_limits<double>::quiet_NaN();
  pending_section_copied_prefix_end_w_ =
      std::numeric_limits<double>::quiet_NaN();
  pending_section_current_w_ =
      std::numeric_limits<double>::quiet_NaN();
  section_pending_identity_ = 0U;
}

bool PhaseOffsetMatchedAdapter::updateGate(
    const MatchedAdapterInput& input, MatchedAdapterOutput& output) {
  const bool path_available = HasUsablePathInput(input.path);
  const bool legacy_available = HasUsableLegacyInput(input.legacy);
  if (!path_available || !legacy_available) {
    const std::string reason = !path_available
        ? "matched adapter path input is unavailable"
        : "matched adapter legacy guidance input is unavailable";
    output.zero_port.invalid_reason = reason;
    output.zero_comparison.invalid_reason = reason;
    output.invalid_reason = reason;
    output.zero_gate_open = zero_gate_open_;
    output.zero_gate_consecutive_count = zero_gate_consecutive_count_;
    output.failure_latched = failure_latched_;
    output.control_failure_reason = control_failure_reason_;
    return false;
  }
  ActiveAdapterInput zero_input;
  zero_input.path = input.path;
  zero_input.position = input.position;
  zero_input.gains = input.gains;
  zero_port_adapter_.evaluate(zero_input, output.zero_port);
  PhaseOffsetActiveAdapter::compareWithLegacy(
      output.zero_port, input.legacy, config_.equivalence_tolerance,
      output.zero_comparison);
  const bool equivalent = output.zero_comparison.valid &&
      output.zero_comparison.equivalent;
  if (output.zero_comparison.valid) {
    if (!failure_latched_ && !zero_gate_open_) {
      zero_gate_consecutive_count_ = equivalent
          ? zero_gate_consecutive_count_ + 1 : 0;
      zero_gate_open_ = zero_gate_consecutive_count_ >=
          config_.warmup_cycles;
    } else if (!failure_latched_ && !equivalent) {
      latchFailure(
          phase_offset_navigation::ControlFailureReason::ZERO_PORT_EQUIVALENCE);
    }
  }
  output.zero_gate_open = zero_gate_open_;
  output.zero_gate_consecutive_count = zero_gate_consecutive_count_;
  output.failure_latched = failure_latched_;
  output.geometry = output.zero_port.geometry;
  output.control_failure_reason = control_failure_reason_;
  output.base_guidance = output.zero_port.guidance;
  output.guidance = output.zero_port.guidance;
  return true;
}

bool PhaseOffsetMatchedAdapter::updateSectionLocked(
    const MatchedAdapterInput& input, MatchedAdapterOutput& output) {
  output.section_bundle = input.section_bundle;
  if (!runtime_ || !input.section_bundle ||
      !input.section_bundle->path || !input.section_bundle->frame ||
      !input.section_bundle->profile ||
      !input.section_bundle->profile->usable ||
      input.section_bundle->local_view_status !=
          plan_env::LocalObstacleViewStatus::VALID) {
    output.invalid_reason = "Section bundle is unavailable";
    return false;
  }
  const SectionPathBundle& bundle = *input.section_bundle;
  if (input.section_bundle != staged_section_bundle_ &&
      input.section_bundle != section_bundle_) {
    output.invalid_reason = "Section bundle is no longer staged/current";
    return false;
  }
  const std::uint64_t path_revision = bundle.path_revision != 0U
      ? bundle.path_revision : bundle.path->pathRevision();
  const std::uint64_t frame_revision = bundle.frame_revision != 0U
      ? bundle.frame_revision : bundle.frame->frameRevision();
  if (bundle.task_generation != task_generation_.load(
          std::memory_order_acquire) || bundle.path->empty() ||
      path_revision == 0U || frame_revision == 0U ||
      bundle.path->pathRevision() != path_revision ||
      bundle.frame->pathRevision() != path_revision ||
      bundle.frame->frameRevision() != frame_revision ||
      bundle.frame_id.empty() || bundle.frame_id != config_.frame_id ||
      (!input.path.valid ||
       (input.semantic_path_owner &&
        input.semantic_path_owner != bundle.path))) {
    output.invalid_reason = "Section bundle path/frame/task facts are stale";
    return false;
  }

  phase_offset_core::GeometryParams geometry_params;
  geometry_params.regularity_margin = runtime_->config().tube.regularity_margin;
  geometry_params.minimum_reference_speed =
      runtime_->config().tube.minimum_reference_speed;
  geometry_params.require_frame_binding = true;
  phase_offset_core::GeometryEvaluator evaluator(geometry_params);
  phase_offset_core::PhaseOffsetGeometryState geometry;
  const double retained_delta = runtime_->retainedDelta();
  if (!evaluator.evaluate(input.path, input.position, retained_delta,
                          geometry) || !geometry.valid ||
      geometry.path_revision != path_revision ||
      geometry.frame_revision != frame_revision) {
    output.invalid_reason = "Section live geometry is unavailable or stale";
    return false;
  }

  guidance::ReferenceGeometry reference;
  reference.point = geometry.r;
  reference.tangent = geometry.T;
  reference.derivative_norm = geometry.r_w.norm();
  reference.valid = true;
  guidance::IsfGuidance base;
  if (!guidance::IsfReferenceKernel::evaluate(
          input.position, reference, input.gains, base) || !base.valid) {
    output.invalid_reason = "Section base guidance is unavailable";
    return false;
  }

  // Diagnostic only: decompose the ISF base rate.  w_dot = k1 (alpha + sigma) /
  // |r_w|, so a collapsed rate is either a large lateral error rho (alpha ->
  // alpha_min), a negative longitudinal error e_parallel (sigma -> -1, "wait
  // for the UAV"), or an inflated reference derivative norm |r_w|.
  {
    const double rho_dbg = base.e_perp.norm();
    const double rho0_dbg = std::max(1e-6, input.gains.progress_rho0);
    const double delta_dbg = std::max(1e-6, input.gains.progress_delta);
    const double alpha_dbg = input.gains.alpha_min +
        (1.0 - input.gains.alpha_min) /
            (1.0 + (rho_dbg / rho0_dbg) * (rho_dbg / rho0_dbg));
    const double sigma_dbg = std::tanh(base.e_parallel / delta_dbg);
    // Guarded: the unit tests exercise this path without ros::init(), and the
    // THROTTLE macros call ros::Time::now() internally.
    if (ros::isInitialized()) {
    ROS_INFO_THROTTLE(
        1.0,
        "[phase_offset_base] k1=%.3f k2=%.3f rho=%.3f e_par=%.3f "
        "alpha=%.3f sigma=%.3f r_w_norm=%.4f w_dot=%.3f v_tau=%.3f "
        "|v_cmd|=%.3f delta=%.3f w=%.3f",
        input.gains.k1, input.gains.k2, rho_dbg, base.e_parallel, alpha_dbg,
        sigma_dbg, reference.derivative_norm, base.w_dot,
        base.v_cmd.dot(reference.tangent), base.v_cmd.norm(), retained_delta,
        geometry.w);
    }
  }

  phase_offset_navigation::TubeViabilityInput preview_input;
  preview_input.current_w = geometry.w;
  preview_input.current_delta = retained_delta;
  preview_input.policy = config_.normal_preview_policy;
  preview_input.upper_u_delta = std::max(0.0, config_.u_delta_abs_max);
  preview_input.boundary_tolerance = 0.0;
  preview_input.path_revision = path_revision;
  preview_input.frame_revision = frame_revision;
  preview_input.expected_path_revision = path_revision;
  preview_input.expected_frame_revision = frame_revision;
  preview_input.max_work = config_.section_build.max_obstacle_checks;
  if (preview_input.max_work == 0U) {
    output.invalid_reason = "Section obstacle-check budget is zero";
    output.runtime_execution.mode =
        phase_offset_navigation::RuntimeExecutionMode::WAITING_FOR_CANDIDATE;
    output.runtime_execution.transient_blocked = true;
    return false;
  }
  preview_input.section_reaches_path_end = bundle.profile->complete &&
      bundle.profile->valid_end == bundle.path->endW();
  phase_offset_navigation::NormalPreviewResult preview;
  // M4C: a rate-degraded Preview (the transverse rate window cannot host the
  // requested motion) stays allocatable.  Losing transverse capacity must not
  // by itself drop a planner-valid path; the allocator clips the request onto
  // the window boundary.  Only states that are unusable right now stay closed.
  if (!phase_offset_navigation::TubeViability::evaluate(
          *bundle.profile, preview_input, preview) || !preview.valid ||
      !preview.feasible) {
    if (ros::isInitialized()) {
        ROS_WARN_THROTTLE(
        1.0,
        "[branch-debug] preview status=%d valid=%d feasible=%d inside=%d reason='%s'",
        static_cast<int>(preview.status), preview.valid ? 1 : 0,
        preview.feasible ? 1 : 0, preview.current_delta_inside ? 1 : 0,
        preview.reason.c_str());
    }
    output.normal_preview = preview;
    output.allocator_value_failure = true;
    output.invalid_reason = "selected_step_unavailable";
    output.geometry = geometry;
    output.base_guidance = base;
    output.guidance = base;
    output.delta = retained_delta;
    output.runtime_execution.mode =
        phase_offset_navigation::RuntimeExecutionMode::WAITING_FOR_CANDIDATE;
    output.runtime_execution.transient_blocked = true;
    output.runtime_execution.current_geometry_valid = geometry.valid;
    return false;
  }

  const Eigen::Vector3d recenter = -config_.delta_tracking_gain *
      retained_delta * geometry.N;
  if (!recenter.allFinite()) {
    output.invalid_reason = "Section recenter motion is nonfinite";
    return false;
  }
  Eigen::Vector3d g_des = recenter;
  // The coordination and the centerline recovery are summed, exactly as the
  // paper writes g_i^des = sat(g_i^coord) - k_rec * delta_i * N_i: the recovery
  // term is a spring on delta, not a switch that mutes the swarm.
  //
  // This used to gate the SPH branch on runtime_->recenterRequested(), which
  // latches as soon as |delta| > tol and stays set until delta reaches zero.
  // In flight that muted the entire coordination intent (measured: 51% of ticks
  // in the pillar scene) and left the inter-UAV repulsion with only a few
  // centimetres of realized lateral offset.  The terminal rejoin that motivated
  // the gate is handled by the arrival test, which already tolerates
  // |delta| + terminal_offset_slack, so the gate is not needed.
  switch (config_.coordination_backend) {
    case PhaseOffsetCoordinationBackend::SPH:
      if (input.sph_bridge != nullptr) {
        const bspline_race::integration::GCoordResolveResult resolved =
            input.sph_bridge->resolveGCoord(
                input.captured_gcoord, ros::Time::now().toSec(),
                ros::SteadyTime::now().toSec());
        if (resolved.usable()) {
          g_des += Eigen::Vector3d(resolved.sample.g_coord.x,
                                   resolved.sample.g_coord.y, 0.0);
        }
      }
      break;
    case PhaseOffsetCoordinationBackend::D1B:
      if (input.g_des_valid) g_des = input.g_des;
      break;
    case PhaseOffsetCoordinationBackend::DISABLED:
      break;
  }
  if (!g_des.allFinite()) {
    output.invalid_reason = "Section desired motion is nonfinite";
    return false;
  }

  phase_offset_navigation::PhaseOffsetAllocatorInput allocator_input;
  allocator_input.geometry = geometry;
  allocator_input.preview = &preview;
  allocator_input.g_des = g_des;
  allocator_input.f_w0 = base.w_dot;
  // Same tangential-speed floor the port projector applies, so the selected
  // u_w cannot spend the tangential budget below the minimum and be rejected by
  // the runtime audit (which would HOLD a planner-valid task).
  allocator_input.base_tangent_speed =
      base.v_cmd.dot(reference.tangent);
  allocator_input.previous_u = runtime_->previousFinalPort();
  allocator_input.dt = input.dt;
  allocator_input.bounds.lower_nu = config_.normal_preview_policy.lower_nu;
  allocator_input.bounds.upper_nu = config_.normal_preview_policy.upper_nu;
  allocator_input.bounds.u_w_abs_max = std::max(0.0, config_.u_w_abs_max);
  allocator_input.bounds.tangent_speed_min =
      std::max(0.0, config_.tangent_speed_min);
  allocator_input.bounds.upper_u_delta =
      std::max(0.0, config_.u_delta_abs_max);
  allocator_input.bounds.u_w_slew_rate = std::max(0.0, config_.u_w_rate_max);
  allocator_input.bounds.u_delta_slew_rate =
      std::max(0.0, config_.u_delta_rate_max);
  allocator_input.bounds.zoh_dt = input.dt;
  allocator_input.expected_path_revision = path_revision;
  allocator_input.expected_frame_revision = frame_revision;
  allocator_input.expected_section_profile = bundle.profile.get();
  phase_offset_navigation::PhaseOffsetAllocatorResult allocator;
  if (!phase_offset_navigation::PhaseOffsetAllocator::allocate(
          allocator_input, allocator) || !allocator.valid ||
      !allocator.feasible ||
      !allocator.selectedUConsistent(0.0)) {
    ROS_WARN_THROTTLE(
        1.0,
        "[branch-debug] allocator status=%d valid=%d feasible=%d reason='%s'",
        static_cast<int>(allocator.status), allocator.valid ? 1 : 0,
        allocator.feasible ? 1 : 0, allocator.reason.c_str());
    output.normal_preview = preview;
    output.allocator = allocator;
    output.allocator_evaluated = true;
    output.allocator_value_failure = true;
    output.geometry = geometry;
    output.base_guidance = base;
    output.guidance = base;
    output.delta = retained_delta;
    output.invalid_reason = "selected_step_unavailable";
    output.runtime_execution.mode =
        phase_offset_navigation::RuntimeExecutionMode::WAITING_FOR_CANDIDATE;
    output.runtime_execution.transient_blocked = true;
    output.runtime_execution.current_geometry_valid = geometry.valid;
    return false;
  }

  // Diagnostic only: records the allocator's phase-rate decision so a stalled
  // agent can be attributed to a clipped phase window / clipped transverse
  // interval rather than guessed at.  Logs nothing new; it only reports values
  // the allocator already computed.
  if (ros::isInitialized()) {
    ROS_INFO_THROTTLE(
      1.0,
      "[phase_offset_alloc] f_w0=%.3f u_w_nom=%.3f u_delta_nom=%.3f "
      "u_w=%.3f[%.3f,%.3f] u_delta=%.3f[%.3f,%.3f] "
      "phase_rate_nom=%.3f phase_rate_sel=%.3f "
      "phase_clipped=%d transverse_clipped=%d preview_valid=%d "
      "preview=[%.3f,%.3f] delta=%.3f base_w_dot=%.3f",
      allocator_input.f_w0, allocator.u_w_nom, allocator.u_delta_nom,
      allocator.selected_u.u_w, allocator.u_w.lower, allocator.u_w.upper,
      allocator.selected_u.u_delta, allocator.u_delta.lower,
      allocator.u_delta.upper, allocator.phase_rate_nom,
      allocator.phase_rate_selected, (int)allocator.phase_window_clipped,
      (int)allocator.transverse_interval_clipped,
      (int)allocator.preview_rate_interval.valid,
      allocator.preview_rate_interval.lower,
      allocator.preview_rate_interval.upper, retained_delta, base.w_dot);
  }

  phase_offset_core::MatchedPortInput matched_input;
  matched_input.geometry = geometry;
  matched_input.base_v_cmd = base.v_cmd;
  matched_input.base_w_dot = base.w_dot;
  matched_input.final_port = allocator.selected_u;
  phase_offset_navigation::RuntimeSectionPrepareInput runtime_input;
  runtime_input.profile = bundle.profile;
  runtime_input.preview = &preview;
  runtime_input.matched = matched_input;
  runtime_input.previous_u = runtime_->previousFinalPort();
  runtime_input.limits = allocator_input.bounds;
  runtime_input.dt = input.dt;
  runtime_input.max_work = preview_input.max_work;
  // M4E: the allocator owns the phase window.  When it had to saturate on the
  // u_w authority boundary because the base guidance rate left the window, the
  // Section seam must execute that saturated port instead of re-applying the
  // same window (which would freeze the phase and hold the vehicle).
  runtime_input.phase_window_saturated = allocator.phase_window_clipped;
  phase_offset_navigation::RuntimeSectionPreparedStep prepared;
  if (!runtime_->prepareSection(runtime_input, prepared) || !prepared.valid()) {
    if (ros::isInitialized()) {
        ROS_WARN_THROTTLE(
        1.0,
        "[branch-debug] prepare valid=%d reason='%s'",
        prepared.valid() ? 1 : 0, prepared.reason().c_str());
    }
    output.normal_preview = preview;
    output.allocator = allocator;
    output.allocator_evaluated = true;
    output.allocator_value_failure = true;
    output.geometry = geometry;
    output.base_guidance = base;
    output.guidance = base;
    output.delta = retained_delta;
    output.invalid_reason = "selected_step_unavailable";
    output.runtime_execution.mode =
        phase_offset_navigation::RuntimeExecutionMode::WAITING_FOR_CANDIDATE;
    output.runtime_execution.transient_blocked = true;
    output.runtime_execution.current_geometry_valid = geometry.valid;
    return false;
  }

  // Allocate a fresh command identity before constructing the executed
  // reference query; reusing an earlier id would allow an ABA-style stale
  // publication to masquerade as this prepared step.
  if (next_section_pending_identity_ ==
          std::numeric_limits<std::uint64_t>::max()) {
    output.invalid_reason = "Section pending command identity saturated";
    return false;
  }
  const std::uint64_t prepared_identity = ++next_section_pending_identity_;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr query;
  try {
    query.reset(new PhaseOffsetExecutedReferenceQuery(
        bundle.path, prepared.nextDelta(), bundle.frame, path_revision,
        frame_revision, bundle.task_generation, prepared_identity,
        bundle.profile->valid_start, bundle.profile->valid_end));
  } catch (const std::exception&) {
    output.invalid_reason = "Section executed-reference query allocation failed";
    return false;
  }
  output.section_bundle = input.section_bundle;
  output.section_prepared_step =
      std::make_shared<const phase_offset_navigation::RuntimeSectionPreparedStep>(
          prepared);
  output.geometry = geometry;
  output.raw_port = allocator.selected_u;
  output.projection.final_port = allocator.selected_u;
  output.projection.final_w_dot = prepared.matchedOutput().w_dot;
  output.projection.final_tangent_speed =
      geometry.T.dot(prepared.matchedOutput().v_cmd);
  output.projection.next_delta = prepared.nextDelta();
  output.projection.next_regularity = geometry.regularity;
  output.projection.valid = IsFinite(output.projection.final_w_dot) &&
      IsFinite(output.projection.final_tangent_speed) &&
      IsFinite(output.projection.next_delta) &&
      IsFinite(output.projection.next_regularity);
  output.matched = prepared.matchedOutput();
  output.base_guidance = base;
  SetGuidance(base, output.matched, output.guidance);
  output.g_des = g_des;
  output.g_des_valid = true;
  output.normal_preview = preview;
  output.allocator = allocator;
  output.allocator_evaluated = true;
  output.delta = retained_delta;
  output.delta_ref = prepared.nextDelta();
  output.profile_active = true;
  output.runtime_execution.mode =
      phase_offset_navigation::RuntimeExecutionMode::NORMAL;
  output.runtime_execution.active_profile_available = true;
  output.runtime_execution.current_geometry_valid = true;
  output.runtime_execution.current_bounds_valid = true;
  output.runtime_execution.retained_delta_current_inside = true;
  output.runtime_execution.tracking_within_bound = true;
  output.runtime_execution.executable = true;
  output.selected = !config_.observe_only && zero_gate_open_ &&
      !failure_latched_ && output.projection.valid;
  output.valid = output.projection.valid && output.matched.valid &&
      output.guidance.valid;
  if (!output.selected) {
    // Observe-only/arming-gate denial is value-only.  The prepared value is
    // diagnostic evidence only; do not install it in the publishable pending
    // tuple.  This keeps a zero-gate/observe-only tick from committing a
    // Runtime step merely because preparation happened to succeed.
    output.invalid_reason = config_.observe_only
        ? "Section candidate is observe-only"
        : (zero_gate_open_ ? "Section candidate is not selectable"
                           : "Section zero gate is not open");
  } else {
    // Only a candidate still occupying the staged slot carries copied-prefix
    // authority.  A command using the committed current bundle deliberately
    // has no source/prefix handoff; otherwise a concurrent planner refresh
    // could relabel this command with metadata from a newer candidate.
    const bool input_is_staged = staged_section_bundle_ == input.section_bundle;
    pending_section_bundle_ = input.section_bundle;
    pending_section_source_path_ = input_is_staged
        ? staged_section_source_path_
        : std::shared_ptr<const ContinuousPhasePath>();
    pending_section_copied_prefix_start_w_ = input_is_staged
        ? staged_section_copied_prefix_start_w_
        : std::numeric_limits<double>::quiet_NaN();
    pending_section_copied_prefix_end_w_ = input_is_staged
        ? staged_section_copied_prefix_end_w_
        : std::numeric_limits<double>::quiet_NaN();
    pending_section_current_w_ = input.path.w;
    section_pending_identity_ = prepared_identity;
    pending_section_prepared_step_ =
        std::make_shared<const phase_offset_navigation::RuntimeSectionPreparedStep>(
            prepared);
    pending_section_reference_query_ = query;
    output.section_prepared_step = pending_section_prepared_step_;
    output.invalid_reason.clear();
  }
  return output.valid;
}


bool PhaseOffsetMatchedAdapter::hasPendingPositionCommand() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return static_cast<bool>(pending_section_prepared_step_);
}

bool PhaseOffsetMatchedAdapter::validatePendingPositionCommandLocked(
    std::string* reason) const {
  if (reason) reason->clear();
  if (!pending_section_prepared_step_ || !pending_section_bundle_ ||
      section_pending_identity_ == 0U || !runtime_ ||
      !runtime_->validateSectionBeforePublish(
          *pending_section_prepared_step_)) {
    if (reason) *reason = "pending Section transaction is unavailable";
    return false;
  }
  return true;
}

void PhaseOffsetMatchedAdapter::discardPendingPositionCommand() {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  clearPendingPositionCommandLocked();
}

bool PhaseOffsetMatchedAdapter::validatePendingPositionCommand() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return validatePendingPositionCommandLocked(nullptr);
}

PendingPositionCommandCapture
PhaseOffsetMatchedAdapter::capturePendingPositionCommand() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  PendingPositionCommandCapture capture;
  if (!pending_section_prepared_step_) {
    capture.valid = true;
    return capture;
  }
  capture.pending = true;
  capture.section_transition = true;
  capture.section_bundle = pending_section_bundle_;
  capture.section_source_path = pending_section_source_path_;
  capture.section_copied_prefix_start_w =
      pending_section_copied_prefix_start_w_;
  capture.section_copied_prefix_end_w =
      pending_section_copied_prefix_end_w_;
  capture.section_current_w = pending_section_current_w_;
  capture.section_next_w = pending_section_prepared_step_->nextW();
  capture.identity = section_pending_identity_;
  capture.reference_query = pending_section_reference_query_;
  capture.valid = capture.identity != 0U &&
      static_cast<bool>(capture.reference_query) &&
      validatePendingPositionCommandLocked(nullptr);
  return capture;
}

phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
PhaseOffsetMatchedAdapter::pendingExecutedReferenceQuery() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  return pending_section_prepared_step_
      ? pending_section_reference_query_
      : phase_offset_navigation::ImmutableExecutedReferenceQueryPtr();
}

bool PhaseOffsetMatchedAdapter::publishPendingPositionCommand(
    const std::function<bool()>& local_publish,
    const std::uint64_t expected_identity,
    const bool cancel_pending_for_goal_override,
    const std::function<void()>& post_publish_no_fail) {
  SectionPathBundlePtr environment_bundle;
  {
    std::lock_guard<std::mutex> probe_lock(runtime_command_mutex_);
    environment_bundle = pending_section_bundle_;
  }
  SectionPathBundlePtr retired_current_bundle;
  SectionPathBundlePtr retired_staged_bundle;
  std::unique_lock<std::recursive_mutex> environment_lock;
  if (environment_bundle && environment_bundle->environment_change_mutex) {
    environment_lock = std::unique_lock<std::recursive_mutex>(
        *environment_bundle->environment_change_mutex);
  }
  std::unique_lock<std::mutex> task_lock(task_publication_mutex_);
  std::unique_lock<std::mutex> runtime_lock(runtime_command_mutex_);
  if (shutdown_requested_.load(std::memory_order_acquire)) {
    return false;
  }
  if (pending_section_prepared_step_) {
    const SectionPathBundlePtr pending_bundle = pending_section_bundle_;
    const auto pending_step = pending_section_prepared_step_;
    const std::uint64_t pending_identity = section_pending_identity_;
    if (!pending_bundle || !pending_step || pending_identity == 0U ||
        pending_bundle->task_generation !=
            task_generation_.load(std::memory_order_acquire)) {
      return false;
    }
    // A terminal goal command at zero retained offset is allowed to retire a
    // stale pending candidate even when its map validity or copied-prefix
    // witness has expired.  It still must name this exact pending command and
    // current task; nonzero offset commands remain on the normal Section path.
    if (cancel_pending_for_goal_override) {
      if (!runtime_ || runtime_->retainedDelta() != 0.0 ||
          expected_identity == 0U || expected_identity != pending_identity ||
          !local_publish || !local_publish()) {
        return false;
      }
      if (staged_section_bundle_ == pending_bundle) {
        retired_staged_bundle.swap(staged_section_bundle_);
        staged_section_source_path_.reset();
      }
      clearPendingPositionCommandLocked();
      if (post_publish_no_fail) post_publish_no_fail();
      return true;
    }
    if (pending_bundle != environment_bundle ||
        !environment_lock.owns_lock() ||
        !pending_bundle->environment_validity ||
        !pending_bundle->environment_change_mutex ||
        !pending_bundle->environment_validity->valid) {
      return false;
    }
    if (pending_section_source_path_) {
      const auto& source_path = pending_section_source_path_;
      if (!source_path || source_path->empty()) return false;
      const bool start_finite =
          std::isfinite(pending_section_copied_prefix_start_w_);
      const bool end_finite =
          std::isfinite(pending_section_copied_prefix_end_w_);
      if (start_finite != end_finite ||
          (start_finite &&
           (pending_section_copied_prefix_end_w_ <
                pending_section_copied_prefix_start_w_ ||
            pending_section_copied_prefix_start_w_ < source_path->startW() ||
            pending_section_copied_prefix_end_w_ > source_path->endW()))) {
        return false;
      }
      if (start_finite) {
        const double live_w = pending_section_current_w_;
        const double next_w = pending_step->nextW();
        if (!std::isfinite(live_w) || !std::isfinite(next_w) ||
            live_w < pending_section_copied_prefix_start_w_ ||
            live_w > pending_section_copied_prefix_end_w_ ||
            next_w > pending_section_copied_prefix_end_w_) {
          return false;
        }
      }
      if (section_bundle_ && section_bundle_->path &&
          section_bundle_->path != source_path) {
        return false;
      }
    }
    if (expected_identity == 0U || expected_identity != pending_identity ||
        !runtime_ || !runtime_->validateSectionBeforePublish(*pending_step) ||
        !local_publish || !local_publish()) {
      return false;
    }
    runtime_->commitSectionNoFail(*pending_step);
    retired_current_bundle.swap(section_bundle_);
    section_bundle_ = pending_bundle;
    if (staged_section_bundle_ == pending_bundle) {
      retired_staged_bundle.swap(staged_section_bundle_);
      staged_section_source_path_.reset();
    }
    clearPendingPositionCommandLocked();
    if (post_publish_no_fail) post_publish_no_fail();
    return true;
  }
  if (expected_identity != 0U) return false;
  if (cancel_pending_for_goal_override) {
    if (!runtime_ || runtime_->retainedDelta() != 0.0 ||
        !local_publish || !local_publish()) {
      return false;
    }
    clearPendingPositionCommandLocked();
    if (post_publish_no_fail) post_publish_no_fail();
    return true;
  }
  if (!local_publish || !local_publish()) return false;
  if (post_publish_no_fail) post_publish_no_fail();
  return true;
}

void PhaseOffsetMatchedAdapter::requestShutdown() {
  {
    std::lock_guard<std::mutex> publication_lock(task_publication_mutex_);
    shutdown_requested_.store(true, std::memory_order_release);
  }
  SectionPathBundlePtr retired_current_bundle;
  SectionPathBundlePtr retired_staged_bundle;
  SectionPathBundlePtr retired_pending_bundle;
  std::shared_ptr<const ContinuousPhasePath> retired_pending_source_path;
  std::shared_ptr<const phase_offset_navigation::RuntimeSectionPreparedStep>
      retired_pending_step;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
      retired_pending_query;
  {
    std::lock_guard<std::mutex> lock(runtime_command_mutex_);
    retired_pending_bundle = pending_section_bundle_;
    retired_pending_source_path = pending_section_source_path_;
    retired_pending_step = pending_section_prepared_step_;
    retired_pending_query = pending_section_reference_query_;
    clearPendingPositionCommandLocked();
    retired_current_bundle.swap(section_bundle_);
    retired_staged_bundle.swap(staged_section_bundle_);
    staged_section_source_path_.reset();
    staged_section_copied_prefix_start_w_ =
        std::numeric_limits<double>::quiet_NaN();
    staged_section_copied_prefix_end_w_ =
        std::numeric_limits<double>::quiet_NaN();
  }
}

void PhaseOffsetMatchedAdapter::shutdown() {
  requestShutdown();
  {
    std::lock_guard<std::mutex> lock(runtime_command_mutex_);
    active_diagnostics_pub_ = ros::Publisher();
  }
}

bool PhaseOffsetMatchedAdapter::resetForNewNavigationTask() {
  SectionPathBundlePtr retired_current_bundle;
  SectionPathBundlePtr retired_staged_bundle;
  SectionPathBundlePtr retired_pending_bundle;
  std::shared_ptr<const ContinuousPhasePath> retired_pending_source_path;
  std::shared_ptr<const phase_offset_navigation::RuntimeSectionPreparedStep>
      retired_pending_step;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
      retired_pending_query;
  std::lock_guard<std::mutex> publication_lock(task_publication_mutex_);
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  if (shutdown_requested_.load(std::memory_order_acquire)) return false;
  const std::uint64_t previous =
      task_generation_.load(std::memory_order_acquire);
  if (previous == std::numeric_limits<std::uint64_t>::max()) return false;
  task_generation_.store(previous + 1U, std::memory_order_release);
  retired_pending_bundle = pending_section_bundle_;
  retired_pending_source_path = pending_section_source_path_;
  retired_pending_step = pending_section_prepared_step_;
  retired_pending_query = pending_section_reference_query_;
  retired_current_bundle.swap(section_bundle_);
  retired_staged_bundle.swap(staged_section_bundle_);
  clearPendingPositionCommandLocked();
  staged_section_source_path_.reset();
  staged_section_copied_prefix_start_w_ =
      std::numeric_limits<double>::quiet_NaN();
  staged_section_copied_prefix_end_w_ =
      std::numeric_limits<double>::quiet_NaN();
  zero_gate_consecutive_count_ = 0;
  zero_gate_open_ = false;
  failure_latched_ = false;
  control_failure_reason_ =
      phase_offset_navigation::ControlFailureReason::NONE;
  if (runtime_) runtime_->resetForNewNavigationTask();
  return true;
}

void PhaseOffsetMatchedAdapter::deactivate(const ros::Time&) {
  const std::uint64_t generation =
      task_generation_.load(std::memory_order_acquire);
  SectionPathBundlePtr retired_current_bundle;
  SectionPathBundlePtr retired_staged_bundle;
  SectionPathBundlePtr retired_pending_bundle;
  std::shared_ptr<const ContinuousPhasePath> retired_pending_source_path;
  std::shared_ptr<const phase_offset_navigation::RuntimeSectionPreparedStep>
      retired_pending_step;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
      retired_pending_query;
  std::lock_guard<std::mutex> publication_lock(task_publication_mutex_);
  std::lock_guard<std::mutex> command_lock(runtime_command_mutex_);
  if (shutdown_requested_.load(std::memory_order_acquire) ||
      generation != task_generation_.load(std::memory_order_acquire)) return;
  retired_pending_bundle = pending_section_bundle_;
  retired_pending_source_path = pending_section_source_path_;
  retired_pending_step = pending_section_prepared_step_;
  retired_pending_query = pending_section_reference_query_;
  retired_current_bundle.swap(section_bundle_);
  retired_staged_bundle.swap(staged_section_bundle_);
  clearPendingPositionCommandLocked();
  staged_section_source_path_.reset();
  staged_section_copied_prefix_start_w_ =
      std::numeric_limits<double>::quiet_NaN();
  staged_section_copied_prefix_end_w_ =
      std::numeric_limits<double>::quiet_NaN();
}

void PhaseOffsetMatchedAdapter::advertise(ros::NodeHandle& nh) {
  if (!configuration_valid_ ||
      shutdown_requested_.load(std::memory_order_acquire)) return;
  if (config_.mode == PhaseOffsetMatchedMode::ACTIVE &&
      !active_diagnostics_pub_) {
    active_diagnostics_pub_ =
        nh.advertise<std_msgs::Float64MultiArray>(
            "phase_offset_active/diagnostics", 1);
  }
}

bool PhaseOffsetMatchedAdapter::stageSectionBundle(
    const SectionPathBundlePtr& bundle,
    const std::shared_ptr<const ContinuousPhasePath>& source_path,
    const double copied_prefix_start_w,
    const double copied_prefix_end_w) {
  if (config_.mode != PhaseOffsetMatchedMode::MANUAL ||
      !configuration_valid_ ||
      shutdown_requested_.load(std::memory_order_acquire) ||
      !bundle || !bundle->path || !bundle->frame || !bundle->profile ||
      bundle->path->empty() || !bundle->profile->usable ||
      bundle->frame_id.empty() ||
      bundle->path->pathRevision() == 0U ||
      bundle->frame->pathRevision() != bundle->path->pathRevision() ||
      bundle->frame->frameRevision() == 0U ||
      bundle->task_generation !=
          task_generation_.load(std::memory_order_acquire) ||
      !bundle->environment_validity ||
      !bundle->environment_change_mutex) {
    return false;
  }
  if (source_path && source_path->empty()) return false;
  const bool start_finite = std::isfinite(copied_prefix_start_w);
  const bool end_finite = std::isfinite(copied_prefix_end_w);
  if (start_finite != end_finite ||
      (start_finite && !(copied_prefix_end_w > copied_prefix_start_w))) {
    return false;
  }
  SectionPathBundlePtr retired;
  {
    std::lock_guard<std::mutex> lock(runtime_command_mutex_);
    if (shutdown_requested_.load(std::memory_order_acquire) ||
        bundle->task_generation !=
            task_generation_.load(std::memory_order_acquire)) return false;
    retired.swap(staged_section_bundle_);
    staged_section_bundle_ = bundle;
    staged_section_source_path_ = source_path;
    staged_section_copied_prefix_start_w_ = copied_prefix_start_w;
    staged_section_copied_prefix_end_w_ = copied_prefix_end_w;
  }
  return true;
}

SectionPathBundlePtr PhaseOffsetMatchedAdapter::captureSectionBundle() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  if (shutdown_requested_.load(std::memory_order_acquire)) {
    return SectionPathBundlePtr();
  }
  return section_bundle_;
}

SectionPathBundlePtr PhaseOffsetMatchedAdapter::capturePendingSectionBundle() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  if (shutdown_requested_.load(std::memory_order_acquire)) {
    return SectionPathBundlePtr();
  }
  return staged_section_bundle_;
}

SectionBundleCandidateCapture
PhaseOffsetMatchedAdapter::capturePendingSectionCandidate() const {
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  SectionBundleCandidateCapture capture;
  if (shutdown_requested_.load(std::memory_order_acquire)) return capture;
  capture.bundle = staged_section_bundle_;
  capture.source_path = staged_section_source_path_;
  capture.copied_prefix_start_w = staged_section_copied_prefix_start_w_;
  capture.copied_prefix_end_w = staged_section_copied_prefix_end_w_;
  return capture;
}

std::uint64_t PhaseOffsetMatchedAdapter::executionGeneration() const {
  return shutdown_requested_.load(std::memory_order_acquire)
      ? 0U : task_generation_.load(std::memory_order_acquire);
}

bool PhaseOffsetMatchedAdapter::update(
    const MatchedAdapterInput& input, MatchedAdapterOutput& output) {
  output = MatchedAdapterOutput();
  // Keep replaced immutable owners alive until the command mutex is released;
  // profile/path destruction can otherwise run on the 50 Hz control lock.
  SectionPathBundlePtr retired_pending_bundle;
  std::shared_ptr<const ContinuousPhasePath> retired_pending_source_path;
  std::shared_ptr<const phase_offset_navigation::RuntimeSectionPreparedStep>
      retired_pending_step;
  phase_offset_navigation::ImmutableExecutedReferenceQueryPtr
      retired_pending_query;
  const std::uint64_t generation =
      task_generation_.load(std::memory_order_acquire);
  if (shutdown_requested_.load(std::memory_order_acquire) ||
      !configuration_valid_) {
    output.invalid_reason =
        "matched adapter configuration is invalid or shutting down";
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_command_mutex_);
  if (generation != task_generation_.load(std::memory_order_acquire)) {
    output.invalid_reason = "matched adapter task retired";
    return false;
  }
  retired_pending_bundle = pending_section_bundle_;
  retired_pending_source_path = pending_section_source_path_;
  retired_pending_step = pending_section_prepared_step_;
  retired_pending_query = pending_section_reference_query_;
  // Each update starts a new command transaction.  Retire the previous
  // prepared step before evaluating the new input; staged/current bundles are
  // deliberately preserved so an invalid refresh cannot erase the active
  // planner owner.  The retired owners remain held by the locals above until
  // runtime_command_mutex_ is released.
  clearPendingPositionCommandLocked();
  if (!updateGate(input, output)) return false;

  if (config_.mode == PhaseOffsetMatchedMode::ACTIVE) {
    const bool equivalent = output.zero_comparison.valid &&
        output.zero_comparison.equivalent;
    output.selected = zero_gate_open_ && !failure_latched_ && equivalent;
    output.valid = output.zero_port.valid &&
        output.zero_comparison.valid;
    output.invalid_reason = output.selected
        ? std::string() : output.zero_comparison.invalid_reason;
    if (active_diagnostics_pub_) {
      const auto values = PhaseOffsetActiveAdapter::makeDiagnostics(
          output.zero_port, output.zero_comparison, zero_gate_open_,
          zero_gate_consecutive_count_, failure_latched_, output.selected);
      std_msgs::Float64MultiArray diagnostics;
      diagnostics.data.assign(values.begin(), values.end());
      active_diagnostics_pub_.publish(diagnostics);
    }
    return output.selected;
  }
  if (config_.mode == PhaseOffsetMatchedMode::MANUAL) {
    return updateSectionLocked(input, output);
  }
  output.invalid_reason = "matched adapter mode is invalid";
  return false;
}

}  // namespace FLAG_Race
