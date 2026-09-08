#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#define private public
#include "bspline_race/integration/phase_offset_matched_adapter.h"
#undef private

namespace FLAG_Race {
namespace {

constexpr double kDt = 0.02;

guidance::IsfGains MakeGains() {
  guidance::IsfGains gains;
  gains.k1 = 2.0; gains.k2 = -2.2; gains.convergence_bandwidth = 0.1;
  gains.progress_rho0 = 0.5; gains.progress_delta = 0.3; gains.alpha_min = 0.05;
  return gains;
}


struct SyntheticPath {
  phase_offset_navigation::RuntimePathSamples samples;
  phase_offset_core::PathDifferentialState current;
  double start_w = 0.0;
  double end_w = 3.0;
};


phase_offset_core::PathDifferentialState MakeStraightState(double w) {
  phase_offset_core::PathDifferentialState state;
  state.p = Eigen::Vector3d(w, 0.0, 1.0);
  state.p_w = Eigen::Vector3d::UnitX();
  state.p_ww.setZero();
  state.path_revision = 7U;
  state.frame_revision = 7U;
  state.T = Eigen::Vector3d::UnitX();
  state.N = Eigen::Vector3d::UnitY();
  state.N_w.setZero();
  state.frame_valid = true;
  state.frame_provenance =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  state.w = w;
  state.valid = true;
  return state;
}


SyntheticPath MakeStraightSyntheticPath() {
  SyntheticPath path;
  path.current = MakeStraightState(0.4);
  for (int index = 0; index <= 30; ++index) {
    path.samples.push_back(MakeStraightState(0.1 * index));
  }
  return path;
}

bool RefreshLegacyGuidance(MatchedAdapterInput& input) {
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterInput zero_input;
  zero_input.path = input.path;
  zero_input.position = input.position;
  zero_input.gains = input.gains;
  ActiveAdapterOutput zero_output;
  if (!zero.evaluate(zero_input, zero_output)) return false;
  input.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
  return true;
}


MatchedAdapterInput MakeInput(const SyntheticPath& path) {
  MatchedAdapterInput input;
  input.path = path.current;
  input.semantic_path_start_w = path.start_w; input.semantic_path_end_w = path.end_w;
  input.position = Eigen::Vector3d(0.4, 0.1, 1.1); input.gains = MakeGains();
  input.dt = kDt;
  EXPECT_TRUE(RefreshLegacyGuidance(input));
  return input;
}

std::shared_ptr<const ContinuousPhasePath> MakeStraightPathOwner(
    const double start_w, const double end_w,
    const std::uint64_t path_revision = 7U) {
  std::shared_ptr<ContinuousPhasePath> owner(new ContinuousPhasePath());
  if (!owner->appendSegment(
          start_w, end_w, "v2-test-straight-owner",
          [start_w, end_w](const double w,
                           ContinuousPhasePathState& state) {
            state.p = Eigen::Vector3d(w, 0.0, 1.0);
            state.dp_dw = Eigen::Vector3d::UnitX();
            state.d2p_dw2.setZero();
            state.vel = state.dp_dw;
            state.valid = std::isfinite(w) && w >= start_w && w <= end_w;
            return state.valid;
          })) {
    return std::shared_ptr<const ContinuousPhasePath>();
  }
  owner->setPathRevision(path_revision);
  return owner;
}


PhaseOffsetMatchedAdapterConfig MakeManualConfig() {
  PhaseOffsetMatchedAdapterConfig config;
  config.mode = PhaseOffsetMatchedMode::MANUAL;
  config.tube_source = phase_offset_navigation::TubeSource::ESDF;
  config.observe_only = false;
  config.profile_period = 2.0; config.warmup_cycles = 100;
  config.u_w_rate_max = 100.0; config.u_delta_rate_max = 100.0;
  config.u_delta_abs_max = 0.40; config.tube_update_period = 0.10;
  config.normal_preview_policy.preview_horizon_w = 2.0;
  config.normal_preview_policy.sample_spacing_w = 0.10;
  config.normal_preview_policy.lower_nu = 0.02;
  config.normal_preview_policy.upper_nu = 2.0;
  config.normal_preview_policy.b_tight = 0.10;
  config.normal_preview_policy.b_open = 0.90;
  config.normal_preview_policy.policy_revision = 1U;
  config.normal_preview_policy.configuration_identity = 99U;
  config.normal_preview_policy.configuration_id = "test-normal-preview-w";
  config.normal_preview_policy_explicit = true;
  config.tube.fixed_delta_max = 0.04; config.tube.back_w = 0.0;
  config.tube.lookahead_w = 2.0; config.tube.min_certified_forward_w = 0.40;
  config.tube.cross_section.search_extent = 3.0;
  config.tube.cross_section.ray_step = 0.05;
  config.tube.cross_section.boundary_tolerance = 0.01;
  config.tube.cross_section.regularity_margin = 0.10;
  config.tube.cross_section.curvature_epsilon = 1e-9;
  config.tube.cross_section.margins.uav_radius = 0.25;
  config.tube.cross_section.margins.map_uncertainty = 0.10;
  config.tube.cross_section.margins.localization_uncertainty = 0.05;
  config.tube.cross_section.margins.tracking_error_bound = 0.15;
  config.tube.cross_section.margins.preincluded_map_uncertainty = 0.10;
  config.cloud_obstacle_set_complete = true;
  return config;
}

// S2-B V2 shadow transport fixtures.  They are immutable value owners and
// exercise the real TubeCertificateBuilderV2 through the adapter's selected
// worker; no retired integration proof manager is used by these tests.
void SetShadowInterval(phase_offset_core::Binary64Interval& interval,
                       const double lower, const double upper) {
  // Keep synthetic certificates conservative under round-to-nearest: all
  // nonzero endpoints are expanded by one representable value.  Zero lower
  // bounds remain exactly zero where the proof requires nonnegative norms.
  interval.lower = (lower >= 0.0 && lower != 0.0)
      ? std::nextafter(lower, -std::numeric_limits<double>::infinity())
      : lower;
  interval.upper = (upper >= 0.0 && upper != 0.0)
      ? std::nextafter(upper, std::numeric_limits<double>::infinity())
      : upper;
  interval.valid = true;
}

void SetShadowVector(phase_offset_core::Binary64VectorInterval& interval,
                     const Eigen::Vector3d& lower,
                     const Eigen::Vector3d& upper) {
  interval.valid = true;
  for (int index = 0; index < 3; ++index) {
    SetShadowInterval(interval.component[static_cast<std::size_t>(index)],
                      lower(index), upper(index));
  }
}

phase_offset_core::CertifiedPathCellV2 MakeShadowCell(double w0, double w1) {
  phase_offset_core::CertifiedPathCellV2 cell;
  cell.w0 = w0;
  cell.w1 = w1;
  cell.anchor_w = static_cast<double>(
      (static_cast<long double>(w0) + static_cast<long double>(w1)) * 0.5L);
  cell.path_revision = 7U;
  cell.frame_revision = 7U;
  cell.segment_identity = 17U;
  cell.proof_identity = 117U;
  SetShadowVector(cell.anchor_position,
                  Eigen::Vector3d(2.0 + cell.anchor_w, 2.0, 2.0),
                  Eigen::Vector3d(2.0 + cell.anchor_w, 2.0, 2.0));
  SetShadowVector(cell.anchor_p_w, Eigen::Vector3d(1.0, 0.0, 0.0),
                  Eigen::Vector3d(1.0, 0.0, 0.0));
  SetShadowVector(cell.anchor_p_ww, Eigen::Vector3d::Zero(),
                  Eigen::Vector3d::Zero());
  SetShadowInterval(cell.inf_p_w_norm, 1.0, 1.0);
  SetShadowInterval(cell.sup_p_w_norm, 1.0, 1.0);
  SetShadowInterval(cell.inf_horizontal_p_w_norm, 1.0, 1.0);
  SetShadowInterval(cell.sup_p_ww_norm, 0.0, 0.0);
  SetShadowInterval(cell.sup_horizontal_p_ww_norm, 0.0, 0.0);
  SetShadowInterval(cell.sup_p_www_norm, 0.0, 0.0);
  SetShadowInterval(cell.sup_normal_derivative, 0.0, 0.0);
  SetShadowInterval(cell.normal_variation, 0.0, 0.0);
  SetShadowInterval(cell.tangent_variation, 0.0, 0.0);
  SetShadowInterval(cell.curvature_variation, 0.0, 0.0);
  const long double half_width =
      (static_cast<long double>(w1) - static_cast<long double>(w0)) * 0.5L;
  SetShadowInterval(cell.midpoint_position_variation, 0.0,
                    std::nextafter(static_cast<double>(half_width),
                                   std::numeric_limits<double>::infinity()));
  SetShadowInterval(cell.chord_deviation, 0.0, 0.0);
  cell.horizontal_acceleration_bound_complete = true;
  cell.normal_frame_proof_complete = true;
  cell.phase_map_proof_complete = true;
  cell.provenance = phase_offset_core::kWorldHorizontalCrossProductProvenance;
  cell.valid = true;
  cell.complete = true;
  return cell;
}

phase_offset_navigation::TubeCertificateConfigV2
MakeShadowCertificateConfig() {
  // This is an explicit test capture, not a conversion from the legacy Tube
  // builder configuration.  Production V2 selection requires this complete
  // value to be supplied by the adapter config before construction.
  phase_offset_navigation::TubeCertificateConfigV2 config;
  config.configuration_id = 99U;
  config.epsilon = 0.40;
  config.nominal_half_width = 1.0;
  config.ray_step = 0.05;
  config.snapshot_resolution = 0.05;
  config.minimum_reference_speed = 1e-8;
  config.sample_step_w = 0.10;
  return config;
}

phase_offset_navigation::TubeMapCaptureKey MakeShadowMapKey(
    const phase_offset_navigation::TubeCertificateConfigV2& config) {
  phase_offset_navigation::TubeMapCaptureKey key;
  key.map_instance_id = 3U;
  // The accepted map state and its accepted notification sequence are one
  // coherent identity in the S3-C admission evidence.  Keep the callback's
  // captured map copy aligned with the evidence supplied by the test below.
  key.state_id = 5U;
  key.accepted_sequence = 5U;
  key.configuration_generation = 6U;
  key.configuration_id = 99U;
  key.frame_provenance_id = 8U;
  key.frame_provenance = "world";
  key.support_provenance_id = 9U;
  key.accepted_time_ticks = 10U;
  key.support_expiry_ticks = 20U;
  key.support_halo = 0.0;
  key.halo_reconciled = true;
  key.grid_min_index_x = -100;
  key.grid_min_index_y = -100;
  key.grid_min_index_z = -100;
  key.grid_max_index_x = 100;
  key.grid_max_index_y = 100;
  key.grid_max_index_z = 100;
  key.grid_voxel_resolution = Eigen::Vector3d::Constant(
      config.snapshot_resolution);
  key.complete_support = true;
  return key;
}

std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
MakeShadowInput(const phase_offset_navigation::TubeCertificateConfigV2& config,
                const std::uint64_t request_id,
                const std::uint64_t generation = 1U,
                std::shared_ptr<const void>* path_owner_out = nullptr) {
  const phase_offset_navigation::TubeMapCaptureKey map =
      MakeShadowMapKey(config);
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> input(
      new phase_offset_navigation::TubeBuildInputV2());
  input->request_id = request_id;
  input->path_key.execution_generation = generation;
  input->path_key.path_instance_id = 2U;
  input->path_key.path_revision = 7U;
  input->path_key.frame_revision = 7U;
  input->path_key.frame_convention_id = 1U;
  input->path_key.frame_convention =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  input->path_key.phase_orientation = 1;
  input->path_key.domain_start = 0.0;
  input->path_key.domain_end = 1.0;
  input->configuration_key = config.key();
  input->map_capture_key = map;
  input->requested_start = 0.0;
  input->requested_end = 1.0;
  input->anchor_w = 0.5;
  // The worker may ask for arbitrary proof partitions.  Return a fresh,
  // exact immutable certificate for each requested child interval; this
  // callback captures no mutable path owner and never mutates its owner.
  input->path_cell_query = [](const double w0, const double w1,
                              phase_offset_core::CertifiedPathCellV2& cell) {
    if (!std::isfinite(w0) || !std::isfinite(w1) || !(w1 > w0) ||
        w0 < 0.0 || w1 > 1.0) {
      return false;
    }
    cell = MakeShadowCell(w0, w1);
    return phase_offset_core::certifiedPathCellV2IsComplete(cell);
  };
  input->producer_breakpoints = {0.0, 1.0};
  const std::shared_ptr<const ContinuousPhasePath> path_owner =
      MakeStraightPathOwner(0.0, 1.0, 7U);
  if (!path_owner) {
    return std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>();
  }
  input->path_owner = std::static_pointer_cast<const void>(path_owner);
  if (path_owner_out != nullptr) *path_owner_out = input->path_owner;
  input->query_owner = std::static_pointer_cast<const void>(
      std::make_shared<int>(request_id + 1U));
  input->capture_owner = std::static_pointer_cast<const void>(
      std::make_shared<int>(request_id + 2U));
  input->applicability_assumptions = "immutable shadow capture";
  input->applicability_deadline_ticks = 20U;
  input->free_ball_query = [map](const Eigen::Vector3d& witness,
                                 const double required) {
    phase_offset_navigation::TubeFreeBallQueryResult result;
    result.status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
    result.certified_radius = required;
    result.certified = true;
    result.complete_support = true;
    result.map_instance_id = map.map_instance_id;
    result.map_state_id = map.state_id;
    result.configuration_id = map.configuration_id;
    result.frame_provenance_id = map.frame_provenance_id;
    result.frame_provenance = map.frame_provenance;
    result.accepted_sequence = map.accepted_sequence;
    result.configuration_generation = map.configuration_generation;
    result.support_provenance_id = map.support_provenance_id;
    result.accepted_time_ticks = map.accepted_time_ticks;
    result.support_expiry_ticks = map.support_expiry_ticks;
    result.support_expiry_timeless = map.support_expiry_timeless;
    result.halo_reconciled = map.halo_reconciled;
    result.support.lower = witness - Eigen::Vector3d::Constant(5.0);
    result.support.upper = witness + Eigen::Vector3d::Constant(5.0);
    result.support.witness = witness;
    result.support.radius = required;
    result.support.map_instance_id = map.map_instance_id;
    result.support.map_state_id = map.state_id;
    result.support.support_provenance_id = map.support_provenance_id;
    result.support.accepted_time_ticks = map.accepted_time_ticks;
    result.support.support_expiry_ticks = map.support_expiry_ticks;
    result.support.support_expiry_timeless = map.support_expiry_timeless;
    result.support.valid = true;
    phase_offset_navigation::TubeVoxelFootprintV2 voxel;
    voxel.min_index_x = map.grid_min_index_x;
    voxel.min_index_y = map.grid_min_index_y;
    voxel.min_index_z = map.grid_min_index_z;
    voxel.max_index_x = map.grid_max_index_x;
    voxel.max_index_y = map.grid_max_index_y;
    voxel.max_index_z = map.grid_max_index_z;
    voxel.native_index = map.grid_native_index;
    voxel.native_origin = map.grid_native_origin;
    voxel.voxel_resolution = map.grid_voxel_resolution;
    voxel.valid = true;
    result.support.voxel_footprint.push_back(voxel);
    return result;
  };
  return std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
      input);
}

void PopulateS4ShadowAdmissionInput(
    const PhaseOffsetMatchedAdapterConfig& config,
    MatchedAdapterInput& command,
    const std::uint64_t request_id = 701U) {
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> input(
      new phase_offset_navigation::TubeBuildInputV2(
          *MakeShadowInput(config.tube_certificate_v2, request_id)));
  input->path_key.domain_end = 3.0;
  input->requested_end = 3.0;
  input->producer_breakpoints = {0.0, 3.0};
  input->path_cell_query =
      [](const double w0, const double w1,
         phase_offset_core::CertifiedPathCellV2& cell) {
        if (!std::isfinite(w0) || !std::isfinite(w1) || !(w1 > w0) ||
            w0 < 0.0 || w1 > 3.0) {
          return false;
        }
        cell = MakeShadowCell(w0, w1);
        return phase_offset_core::certifiedPathCellV2IsComplete(cell);
      };
  const std::shared_ptr<const ContinuousPhasePath> path_owner =
      MakeStraightPathOwner(0.0, 3.0, 7U);
  ASSERT_TRUE(path_owner);
  input->path_owner = std::static_pointer_cast<const void>(path_owner);
  command.tube_worker_input_v2 =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(input);
  command.semantic_path_owner = path_owner;
  command.semantic_path_start_w = path_owner->startW();
  command.semantic_path_end_w = path_owner->endW();
  command.path = MakeStraightState(0.45);
  command.position = command.path.p + Eigen::Vector3d(0.0, 0.0, 0.1);
  ASSERT_TRUE(RefreshLegacyGuidance(command));
  command.tube_v2_admission.valid = true;
  command.tube_v2_admission.binding_sequence = request_id;
  command.tube_v2_admission.accepted_state_sequence = 5U;
  command.tube_v2_admission.accepted_state_notification_sequence = 5U;
  command.tube_v2_admission.accepted_time_ticks = 10U;
  command.tube_v2_admission.support_expiry_ticks = 20U;
  command.tube_v2_admission.map_instance_id = 3U;
  command.tube_v2_admission.configuration_generation = 6U;
  command.tube_v2_admission.configuration_key = 99U;
  command.tube_v2_admission.support_provenance_id = 9U;
  command.tube_v2_admission.frame_provenance = "world";
  command.tube_v2_admission.latest_accepted_state_sequence = 5U;
  command.tube_v2_admission.latest_accepted_state_notification_sequence = 5U;
  command.tube_v2_admission.latest_accepted_time_ticks = 10U;
  command.tube_v2_admission.latest_map_instance_id = 3U;
  command.tube_v2_admission.latest_configuration_generation = 6U;
  command.tube_v2_admission.latest_configuration_key = 99U;
  command.tube_v2_admission.latest_support_provenance_id = 9U;
  command.tube_v2_admission.latest_frame_provenance = "world";
  command.tube_v2_admission.selected_u = phase_offset_core::PortCommand();
  command.tube_v2_admission.base_phase_rate = 0.02;
  command.tube_v2_admission.phase_rate_lower = 0.02;
  command.tube_v2_admission.phase_rate_upper = 2.0;
  command.tube_v2_admission.upper_u_delta = 0.40;
  command.tube_v2_admission.now = 10.0;
  command.tube_v2_admission.applicability_deadline = 20.0;
  command.tube_v2_admission.applicability_deadline_valid = true;
  command.tube_v2_admission.preview_policy = config.normal_preview_policy;
  command.tube_v2_admission.preview_policy.configuration_identity =
      config.tube_certificate_v2.configuration_id;
  command.tube_v2_admission.limits.lower_phase_rate = 0.02;
  command.tube_v2_admission.limits.upper_phase_rate = 2.0;
  command.tube_v2_admission.limits.upper_nu = 2.0;
  command.tube_v2_admission.limits.max_u_w = config.u_w_abs_max;
  command.tube_v2_admission.limits.max_u_delta = config.u_delta_abs_max;
  command.tube_v2_admission.limits.u_w_slew_rate = 100.0;
  command.tube_v2_admission.limits.u_delta_slew_rate = 100.0;
  command.tube_v2_admission.limits.return_u_delta_max =
      config.u_delta_abs_max;
  command.tube_v2_admission.limits.return_u_delta_slew_rate = 100.0;
  command.tube_v2_admission.limits.max_schedule_steps = 1000U;
  command.tube_v2_admission.limits.max_work = 250000U;
  command.tube_v2_admission.limits.valid = true;
  command.tube_v2_admission.tracking.valid = true;
  command.tube_v2_admission.tracking.error_norm = 0.0;
  command.tube_v2_admission.tracking.error_bound = 0.15;
  command.tube_v2_admission.tracking.physical_tangent_valid = true;
  command.tube_v2_admission.max_work = 250000U;
  command.tube_v2_admission.provenance = "S4A/test/bootstrap";
}

bool WaitShadowDelivery(PhaseOffsetMatchedAdapter& adapter,
                        const std::uint64_t minimum) {
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(3);
  while (adapter.v2_shadow_worker_ &&
         adapter.v2_shadow_worker_->stats().delivery_count < minimum) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::yield();
  }
  adapter.consumeV2ShadowCompletions();
  return adapter.v2_shadow_worker_ != nullptr;
}

bool WaitShadowIdle(PhaseOffsetMatchedAdapter& adapter) {
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(3);
  while (adapter.v2_shadow_worker_ &&
         adapter.v2_shadow_worker_->running()) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::yield();
  }
  return adapter.v2_shadow_worker_ != nullptr;
}

std::shared_ptr<const phase_offset_navigation::TubeProfileV2>
InstalledV2Profile(const PhaseOffsetMatchedAdapter& adapter) {
  return adapter.v2_execution_binding_
      ? adapter.v2_execution_binding_->profile
      : std::shared_ptr<const phase_offset_navigation::TubeProfileV2>();
}

std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
InstalledV2SourceInput(const PhaseOffsetMatchedAdapter& adapter) {
  return adapter.v2_execution_binding_
      ? adapter.v2_execution_binding_->source_input
      : std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>();
}

bool RefreshV2BaseRateFromCommittedState(
    const PhaseOffsetMatchedAdapter& adapter,
    MatchedAdapterInput& command) {
  if (!adapter.runtime_) return false;
  phase_offset_core::GeometryParams params;
  params.regularity_margin =
      adapter.config_.tube.cross_section.regularity_margin;
  params.minimum_reference_speed =
      adapter.config_.tube.cross_section.minimum_reference_speed;
  phase_offset_core::GeometryEvaluator evaluator(params);
  phase_offset_core::PhaseOffsetGeometryState geometry;
  if (!evaluator.evaluate(command.path, command.position,
                          adapter.runtime_->retainedDelta(), geometry) ||
      !geometry.valid) {
    return false;
  }
  guidance::ReferenceGeometry reference;
  reference.point = geometry.r;
  reference.tangent = geometry.T;
  reference.derivative_norm = geometry.r_w.norm();
  reference.valid = true;
  guidance::IsfGuidance base;
  if (!guidance::IsfReferenceKernel::evaluate(
          command.position, reference, command.gains, base) || !base.valid) {
    return false;
  }
  command.tube_v2_admission.base_phase_rate = base.w_dot;
  return RefreshLegacyGuidance(command);
}

struct ShadowCallbackBarrier {
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false;
  bool release = false;
};

bool WaitShadowCallbackEntered(
    const std::shared_ptr<ShadowCallbackBarrier>& barrier) {
  std::unique_lock<std::mutex> lock(barrier->mutex);
  return barrier->condition.wait_for(lock, std::chrono::seconds(3),
                                     [&barrier]() {
                                       return barrier->entered;
                                     });
}

bool InstallS4ShadowProfileForTest(
    PhaseOffsetMatchedAdapter& adapter,
    const PhaseOffsetMatchedAdapterConfig& config,
    MatchedAdapterInput& command,
    MatchedAdapterOutput& output,
    const phase_offset_core::PortCommand& selected_u =
        phase_offset_core::PortCommand(),
    const std::shared_ptr<const ContinuousPhasePath>& source_owner =
        std::shared_ptr<const ContinuousPhasePath>()) {
  command = MakeInput(MakeStraightSyntheticPath());
  PopulateS4ShadowAdmissionInput(config, command);
  const std::shared_ptr<const ContinuousPhasePath> effective_source_owner =
      source_owner ? source_owner : MakeStraightPathOwner(0.0, 3.0, 7U);
  if (effective_source_owner) {
    std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> owned_input(
        new phase_offset_navigation::TubeBuildInputV2(
            *command.tube_worker_input_v2));
    owned_input->path_key.path_revision = effective_source_owner->pathRevision();
    owned_input->path_key.frame_revision = effective_source_owner->pathRevision();
    owned_input->path_key.domain_start = effective_source_owner->startW();
    owned_input->path_key.domain_end = effective_source_owner->endW();
    owned_input->path_owner =
        std::static_pointer_cast<const void>(effective_source_owner);
    command.tube_worker_input_v2 =
        std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
            owned_input);
    command.semantic_path_owner = effective_source_owner;
    command.semantic_path_start_w = effective_source_owner->startW();
    command.semantic_path_end_w = effective_source_owner->endW();
  }
  command.path = MakeStraightState(0.45);
  command.position = command.path.p + Eigen::Vector3d(0.0, 0.0, 0.1);
  const bool request_nonzero_port =
      selected_u.u_w != 0.0 || selected_u.u_delta != 0.0;
  if (request_nonzero_port) {
    // Stage-7 NORMAL authority belongs to the live allocator.  Drive the
    // existing D1B value-semantic test seam so the requested straight-path
    // port is actually selected and committed through publish-first; merely
    // writing the admission evidence's selected_u field would be inert.
    if (config.coordination_backend != PhaseOffsetCoordinationBackend::D1B) {
      return false;
    }
    command.g_des = command.path.p_w * selected_u.u_w +
        command.path.N * selected_u.u_delta;
    command.g_des_valid = command.g_des.allFinite();
    if (!command.g_des_valid) return false;
  }
  {
    PhaseOffsetActiveAdapter zero;
    ActiveAdapterInput zero_input;
    zero_input.path = command.path;
    zero_input.position = command.position;
    zero_input.gains = command.gains;
    ActiveAdapterOutput zero_output;
    if (!zero.evaluate(zero_input, zero_output)) return false;
    command.tube_v2_admission.base_phase_rate =
        zero_output.guidance.w_dot;
    command.legacy = LegacyGuidanceSnapshot(
        zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
        zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
        zero_output.guidance.ref_pt, zero_output.guidance.tangent,
        zero_output.guidance.valid);
  }
  if (adapter.update(command, output) || !adapter.scheduleTubeBuild() ||
      !WaitShadowDelivery(adapter, 1U) ||
      !adapter.latest_v2_shadow_current_completion_ ||
      !adapter.latest_v2_shadow_current_completion_->built()) {
    return false;
  }
  bool selected = false;
  for (int cycle = 0;
       cycle <= config.warmup_cycles &&
           !adapter.pending_v2_shadow_bootstrap_candidate_;
       ++cycle) {
    selected = adapter.update(command, output);
  }
  if (!selected || !output.selected ||
      !adapter.pending_v2_shadow_bootstrap_candidate_ ||
      !adapter.pending_v2_shadow_bootstrap_candidate_->profile) {
    return false;
  }
  if (request_nonzero_port &&
      (output.raw_port.u_w != selected_u.u_w ||
       output.raw_port.u_delta != selected_u.u_delta)) {
    return false;
  }
  const PendingPositionCommandCapture pending =
      adapter.capturePendingPositionCommand();
  return pending.pending && pending.valid && pending.identity != 0U &&
      pending.reference_query &&
      adapter.publishPendingPositionCommand([]() { return true; },
                                            pending.identity) &&
      static_cast<bool>(adapter.v2_execution_binding_);
}

void SetS5ShadowRecoveryState(
    const phase_offset_navigation::TubeExecutionStateV2& state,
    MatchedAdapterInput& command) {
  command.path = MakeStraightState(state.w);
  // Keep the actual offset-reference tracking error fixed while delta
  // changes, so the unchanged ISF kernel reproduces the base phase rate
  // captured in the immutable reserve.
  command.position = command.path.p +
      Eigen::Vector3d(0.0, state.delta, 0.1);
  PhaseOffsetActiveAdapter zero;
  ActiveAdapterInput zero_input;
  zero_input.path = command.path;
  zero_input.position = command.position;
  zero_input.gains = command.gains;
  ActiveAdapterOutput zero_output;
  ASSERT_TRUE(zero.evaluate(zero_input, zero_output));
  command.legacy = LegacyGuidanceSnapshot(
      zero_output.guidance.v_cmd, zero_output.guidance.w_dot,
      zero_output.guidance.e_parallel, zero_output.guidance.e_perp,
      zero_output.guidance.ref_pt, zero_output.guidance.tangent,
      zero_output.guidance.valid);
}

std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
MakeAcceptedMapRefreshInput(
    const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>&
        source,
    const std::uint64_t request_id,
    const std::uint64_t state_id,
    const std::uint64_t accepted_time_ticks) {
  if (!source) return std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>();
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> refreshed(
      new phase_offset_navigation::TubeBuildInputV2(*source));
  refreshed->request_id = request_id;
  refreshed->map_capture_key.state_id = state_id;
  refreshed->map_capture_key.accepted_sequence = state_id;
  refreshed->map_capture_key.accepted_time_ticks = accepted_time_ticks;
  // Keep this fixture intentionally tied to the old query owner.  The
  // replacement is still a coherent immutable request identity for the
  // scheduler, but its failed proof must not erase the installed incumbent.
  // A production producer supplies the matching coherent map capture/query.
  return std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
      refreshed);
}

std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
MakeCoherentAcceptedMapRefreshInput(
    const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>&
        source,
    const std::uint64_t request_id,
    const std::uint64_t state_id,
    const std::uint64_t accepted_time_ticks) {
  if (!source || !source->free_ball_query) {
    return std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>();
  }
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> refreshed(
      new phase_offset_navigation::TubeBuildInputV2(*source));
  refreshed->request_id = request_id;
  refreshed->map_capture_key.state_id = state_id;
  refreshed->map_capture_key.accepted_sequence = state_id;
  refreshed->map_capture_key.accepted_time_ticks = accepted_time_ticks;
  const phase_offset_navigation::TubeMapCaptureKey map =
      refreshed->map_capture_key;
  const phase_offset_navigation::TubeFreeBallQuery source_query =
      source->free_ball_query;
  refreshed->free_ball_query =
      [source_query, map](const Eigen::Vector3d& witness,
                          const double required) {
        phase_offset_navigation::TubeFreeBallQueryResult result =
            source_query(witness, required);
        result.map_state_id = map.state_id;
        result.accepted_sequence = map.accepted_sequence;
        result.accepted_time_ticks = map.accepted_time_ticks;
        result.support.map_state_id = map.state_id;
        result.support.accepted_time_ticks = map.accepted_time_ticks;
        return result;
      };
  return std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
      refreshed);
}

std::shared_ptr<phase_offset_navigation::TubeBuildInputV2>
MakeBlockedShadowInput(
    const phase_offset_navigation::TubeCertificateConfigV2& config,
    const std::uint64_t request_id,
    const std::shared_ptr<ShadowCallbackBarrier>& barrier) {
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> input(
      new phase_offset_navigation::TubeBuildInputV2(*MakeShadowInput(
          config, request_id)));
  input->path_cell_query = [barrier](
      const double w0, const double w1,
      phase_offset_core::CertifiedPathCellV2& cell) {
    {
      std::unique_lock<std::mutex> lock(barrier->mutex);
      barrier->entered = true;
      barrier->condition.notify_all();
      if (!barrier->condition.wait_for(lock, std::chrono::seconds(3),
                                      [&barrier]() {
                                        return barrier->release;
                                      })) {
        return false;
      }
    }
    if (!std::isfinite(w0) || !std::isfinite(w1) || !(w1 > w0) ||
        w0 < 0.0 || w1 > 1.0) {
      return false;
    }
    cell = MakeShadowCell(w0, w1);
    return phase_offset_core::certifiedPathCellV2IsComplete(cell);
  };
  return input;
}

TEST(PhaseOffsetMatchedAdapterS2B, V2ShadowSelectsOneWorkerAndObservesCompletion) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  ASSERT_TRUE(adapter.v2_shadow_worker_);
  EXPECT_FALSE(adapter.v2_shadow_worker_->joined());
  // Seed nonzero Runtime history to prove the observational backend never
  // consumes or rewrites it while transporting a proof request/completion.
  adapter.runtime_->delta_ = 0.125;
  adapter.runtime_->previous_final_port_.u_w = 0.031;
  adapter.runtime_->previous_final_port_.u_delta = -0.017;
  const double retained_delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_port_before =
      adapter.runtime_->previousFinalPort();
  const auto authority_before = adapter.execution_authority_.snapshotPtr();
  const auto binding_before = adapter.v2_execution_binding_;
  const auto pending_before = adapter.pending_v2_shadow_bootstrap_candidate_;
  const auto retained_before = std::atomic_load_explicit(
      &adapter.retained_v2_shadow_admission_candidate_,
      std::memory_order_acquire);
  const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2> input =
      MakeShadowInput(adapter.v2_shadow_worker_->config(), 1U);
  MatchedAdapterInput command = MakeInput(MakeStraightSyntheticPath());
  command.tube_worker_input_v2 = input;
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(command, output));
  EXPECT_EQ(adapter.runtime_->retainedDelta(), retained_delta_before);
  EXPECT_EQ(adapter.runtime_->previousFinalPort().u_w, previous_port_before.u_w);
  EXPECT_EQ(adapter.runtime_->previousFinalPort().u_delta,
            previous_port_before.u_delta);
  EXPECT_EQ(adapter.execution_authority_.snapshotPtr(), authority_before);
  EXPECT_EQ(adapter.v2_execution_binding_, binding_before);
  EXPECT_EQ(adapter.pending_v2_shadow_bootstrap_candidate_, pending_before);
  EXPECT_EQ(std::atomic_load_explicit(
                &adapter.retained_v2_shadow_admission_candidate_,
                std::memory_order_acquire),
            retained_before);
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 1U);
  // Completion transport remains observational too: no Runtime history,
  // authority snapshot, binding, or pending publication is changed by
  // scheduling and consuming the V2 result.
  EXPECT_EQ(adapter.runtime_->retainedDelta(), retained_delta_before);
  EXPECT_EQ(adapter.runtime_->previousFinalPort().u_w, previous_port_before.u_w);
  EXPECT_EQ(adapter.runtime_->previousFinalPort().u_delta,
            previous_port_before.u_delta);
  EXPECT_EQ(adapter.execution_authority_.snapshotPtr(), authority_before);
  EXPECT_EQ(adapter.v2_execution_binding_, binding_before);
  EXPECT_EQ(adapter.pending_v2_shadow_bootstrap_candidate_, pending_before);
  EXPECT_FALSE(adapter.v2_execution_binding_);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS2B, V2ShadowIdentityCoalescesAndRetainsCompletion) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  const auto v2config = adapter.v2_shadow_worker_->config();
  const auto v2input = MakeShadowInput(v2config, 7U);
  MatchedAdapterInput command = MakeInput(MakeStraightSyntheticPath());
  command.tube_worker_input_v2 = v2input;
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  EXPECT_FALSE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, 1U));
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, 1U);
  // Adapter consumes the worker's retained slot into its observational value;
  // no second adapter-side proof is started by delayed timer calls.
  EXPECT_TRUE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_FALSE(adapter.v2_execution_binding_);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS2B, V2ShadowResetAndPathReplacementDiscardStale) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  const auto v2config = adapter.v2_shadow_worker_->config();
  MatchedAdapterInput command = MakeInput(MakeStraightSyntheticPath());
  command.tube_worker_input_v2 = MakeShadowInput(v2config, 9U, 1U);
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, 1U));
  const std::uint64_t deliveries_before_replacement =
      adapter.v2_shadow_worker_->stats().delivery_count;
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> replacement(
      new phase_offset_navigation::TubeBuildInputV2(*MakeShadowInput(
          v2config, 10U, 1U)));
  replacement->path_key.path_instance_id = 4U;
  command.tube_worker_input_v2 =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
          replacement);
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(
      adapter, deliveries_before_replacement + 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 10U);
  const std::uint64_t generation_before_reset =
      adapter.executionGenerationV2();
  ASSERT_TRUE(adapter.resetForNewNavigationTask());
  EXPECT_EQ(adapter.executionGenerationV2(), generation_before_reset + 1U);
  EXPECT_FALSE(adapter.latest_v2_shadow_current_completion_);
  const std::uint64_t deliveries_before_reset =
      adapter.v2_shadow_worker_->stats().delivery_count;
  command.tube_worker_input_v2 = MakeShadowInput(v2config, 10U, 2U);
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, deliveries_before_reset + 1U));
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->execution_generation, 2U);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS2B, DelayedCommandNotificationsDoNotDuplicateV2Proof) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  const auto v2config = adapter.v2_shadow_worker_->config();
  MatchedAdapterInput command = MakeInput(MakeStraightSyntheticPath());
  command.tube_worker_input_v2 = MakeShadowInput(v2config, 21U);
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, 1U));
  // Replaying the same immutable command (as a delayed live-w/raw-cloud
  // notification would) has no new request identity and cannot recertify.
  ASSERT_FALSE(adapter.update(command, output));
  EXPECT_FALSE(adapter.scheduleTubeBuild());
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, 1U);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS6C,
     SuccessfulSuccessorPublishAtomicallySwitchesRuntimeAndBinding) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::D1B;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = false;

  std::shared_ptr<ContinuousPhasePath> mutable_source(
      new ContinuousPhasePath());
  ASSERT_TRUE(mutable_source->appendSegment(
      0.0, 3.0, "s6c-publish-source",
      [](const double w, ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.vel = state.dp_dw;
        state.valid = std::isfinite(w);
        return state.valid;
      }));
  mutable_source->setPathRevision(7U);
  const std::shared_ptr<const ContinuousPhasePath> source_owner =
      mutable_source;

  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterInput command;
  MatchedAdapterOutput output;
  phase_offset_core::PortCommand bootstrap_u;
  bootstrap_u.u_delta = 0.20;
  ASSERT_TRUE(InstallS4ShadowProfileForTest(
      adapter, config, command, output, bootstrap_u, source_owner));
  ASSERT_NE(adapter.runtime_->retainedDelta(), 0.0);
  ASSERT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   bootstrap_u.u_delta);
  const std::shared_ptr<const TubeV2ExecutionBinding> source_binding =
      adapter.v2_execution_binding_;
  ASSERT_TRUE(source_binding);
  ASSERT_TRUE(source_binding->complete());
  const phase_offset_navigation::TubeFiniteReserveV2* source_reserve =
      adapter.runtime_->committedV2Reserve();
  ASSERT_TRUE(source_reserve);
  ASSERT_LT(source_reserve->cursor, source_reserve->steps.size());
  const double live_w = source_reserve->steps[source_reserve->cursor].before.w;

  std::shared_ptr<ContinuousPhasePath> mutable_successor(
      new ContinuousPhasePath());
  const double seam_w = live_w + 0.35;
  ASSERT_TRUE(mutable_successor->appendSlice(
      *source_owner, live_w, seam_w));
  ASSERT_TRUE(mutable_successor->appendSegment(
      seam_w, 3.0, "s6c-publish-tail",
      [](const double w, ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.vel = state.dp_dw;
        state.valid = std::isfinite(w);
        return state.valid;
      }));
  mutable_successor->setPathRevision(8U);
  const std::shared_ptr<const ContinuousPhasePath> successor_owner =
      mutable_successor;

  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> successor_input(
      new phase_offset_navigation::TubeBuildInputV2(
          *source_binding->source_input));
  successor_input->request_id = source_binding->source_input->request_id + 1U;
  successor_input->path_key.path_instance_id += 1U;
  successor_input->path_key.path_revision = 8U;
  successor_input->path_key.frame_revision = 8U;
  successor_input->path_key.domain_start = live_w;
  successor_input->requested_start = live_w;
  successor_input->anchor_w = live_w;
  successor_input->producer_breakpoints = {live_w, seam_w, 3.0};
  successor_input->path_cell_query = [](
      const double w0, const double w1,
      phase_offset_core::CertifiedPathCellV2& cell) {
    if (!std::isfinite(w0) || !std::isfinite(w1) || !(w1 > w0) ||
        w0 < 0.0 || w1 > 3.0) {
      return false;
    }
    cell = MakeShadowCell(w0, w1);
    cell.path_revision = 8U;
    cell.frame_revision = 8U;
    return phase_offset_core::certifiedPathCellV2IsComplete(cell);
  };
  successor_input->path_owner =
      std::static_pointer_cast<const void>(successor_owner);
  ASSERT_TRUE(successor_input->complete());

  std::shared_ptr<TubeV2SuccessorHandoffEvidence> handoff(
      new TubeV2SuccessorHandoffEvidence());
  handoff->valid = true;
  handoff->structurally_copied_prefix = true;
  handoff->expected_execution_generation =
      source_binding->profile->path_key.execution_generation;
  handoff->source_path_key = source_binding->profile->path_key;
  handoff->source_path_owner = source_owner;
  handoff->successor_path_key = successor_input->path_key;
  handoff->successor_path_owner = successor_owner;
  handoff->phase_after_w = live_w;
  handoff->copied_prefix_start_w = live_w;
  handoff->copied_prefix_end_w = seam_w;
  handoff->successor_request = successor_input;
  handoff->provenance = "test/S6C/appendSlice";
  command.tube_v2_successor_handoff = handoff;
  command.path = MakeStraightState(live_w);
  command.position = command.path.p + Eigen::Vector3d(
      0.0, adapter.runtime_->retainedDelta(), 0.1);
  ASSERT_TRUE(RefreshV2BaseRateFromCommittedState(adapter, command));

  const std::uint64_t deliveries_before =
      adapter.v2_shadow_worker_->stats().delivery_count;
  ASSERT_TRUE(adapter.enqueueV2SuccessorRequest(
      handoff->source_path_key, *successor_input));
  ASSERT_TRUE(WaitShadowDelivery(adapter, deliveries_before + 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_);
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_->built())
      << adapter.latest_v2_shadow_successor_completion_->error_message;
  const bool successor_selected = adapter.update(command, output);
  ASSERT_TRUE(output.v2_shadow_admission_candidate)
      << output.invalid_reason;
  ASSERT_TRUE(successor_selected)
      << output.v2_shadow_admission_candidate->reason << " status="
      << static_cast<int>(output.v2_shadow_admission_candidate->status);
  ASSERT_TRUE(output.selected);
  const std::shared_ptr<const TubeV2ShadowAdmissionCandidate> candidate =
      output.v2_shadow_admission_candidate;
  ASSERT_TRUE(candidate);
  ASSERT_TRUE(candidate->prepared_step.binding_transition);
  ASSERT_TRUE(candidate->proposed_binding);
  const PendingPositionCommandCapture pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(pending.pending);
  ASSERT_TRUE(pending.valid);
  ASSERT_TRUE(pending.v2_binding_transition);
  ASSERT_EQ(pending.proposed_v2_binding, candidate->proposed_binding);

  int callback_count = 0;
  bool callback_saw_atomic_successor = false;
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, pending.identity, false,
      [&]() {
        ++callback_count;
        callback_saw_atomic_successor =
            adapter.v2_execution_binding_ == pending.proposed_v2_binding &&
            adapter.runtime_->retainedDelta() ==
                candidate->prepared_step.successor.delta &&
            adapter.runtime_->previousFinalPort().u_w ==
                candidate->prepared_step.successor.previous_u.u_w &&
            adapter.runtime_->previousFinalPort().u_delta ==
                candidate->prepared_step.successor.previous_u.u_delta;
      }));
  EXPECT_EQ(callback_count, 1);
  EXPECT_TRUE(callback_saw_atomic_successor);
  ASSERT_TRUE(adapter.v2_execution_binding_);
  EXPECT_NE(adapter.v2_execution_binding_, source_binding);
  EXPECT_EQ(adapter.v2_execution_binding_, pending.proposed_v2_binding);
  EXPECT_EQ(InstalledV2Profile(adapter)->path_key,
            successor_input->path_key);
  EXPECT_EQ(InstalledV2SourceInput(adapter), handoff->successor_request);
  EXPECT_EQ(adapter.runtime_->v2_identity_.path_instance_id,
            successor_input->path_key.path_instance_id);
  EXPECT_EQ(adapter.runtime_->v2_identity_.profile_id,
            InstalledV2Profile(adapter)->profile_id);
  EXPECT_EQ(adapter.runtime_->v2_identity_.binding_sequence,
            candidate->prepared_step.identity.binding_sequence);
  EXPECT_FALSE(adapter.hasPendingPositionCommand());

  int replay_callback_count = 0;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      []() { return true; }, pending.identity, false,
      [&replay_callback_count]() { ++replay_callback_count; }));
  EXPECT_EQ(replay_callback_count, 0);
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> replay_request(
      new phase_offset_navigation::TubeBuildInputV2(*successor_input));
  ++replay_request->request_id;
  ASSERT_TRUE(replay_request->complete());
  EXPECT_FALSE(adapter.enqueueV2SuccessorRequest(
      source_binding->profile->path_key, *replay_request));
  EXPECT_EQ(adapter.v2_execution_binding_, pending.proposed_v2_binding);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS2B,
     V2ShadowSuccessorPurposeAndUsefulRetention) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  const auto v2config = adapter.v2_shadow_worker_->config();
  MatchedAdapterInput command = MakeInput(MakeStraightSyntheticPath());
  command.tube_worker_input_v2 = MakeShadowInput(v2config, 31U);
  command.tube_worker_purpose_v2 = TubeWorkerPurposeV2::SUCCESSOR;
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_);
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_->built());
  EXPECT_EQ(adapter.latest_v2_shadow_successor_completion_->purpose,
            TubeWorkerPurposeV2::SUCCESSOR);
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, 1U);

  // The purpose slots are independent: a CURRENT completion must coexist with
  // the retained SUCCESSOR proof rather than overwrite it.
  command.tube_worker_input_v2 = MakeShadowInput(v2config, 30U);
  command.tube_worker_purpose_v2 = TubeWorkerPurposeV2::CURRENT;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, 2U));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_->built());
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_successor_completion_->request_id, 31U);
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, 2U);

  // A later request with the same immutable path/config/map work but a
  // narrower useful interval is covered by the retained successor proof.
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> narrower(
      new phase_offset_navigation::TubeBuildInputV2(*MakeShadowInput(
          v2config, 32U)));
  narrower->requested_end = 0.5;
  narrower->anchor_w = 0.25;
  command.tube_worker_input_v2 =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
          narrower);
  command.tube_worker_purpose_v2 = TubeWorkerPurposeV2::SUCCESSOR;
  ASSERT_FALSE(adapter.update(command, output));
  EXPECT_FALSE(adapter.scheduleTubeBuild());
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, 2U);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS6A,
     CopiedPrefixSuccessorEnqueuePreservesInstalledSourceAndRuntime) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = false;
  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterInput command;
  MatchedAdapterOutput output;
  ASSERT_TRUE(InstallS4ShadowProfileForTest(
      adapter, config, command, output));
  ASSERT_TRUE(adapter.v2_execution_binding_);

  phase_offset_navigation::TubePathKey source_key;
  std::uint64_t generation = 0U;
  std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      source_input;
  ASSERT_TRUE(adapter.captureV2SuccessorSource(
      source_key, generation, source_input));
  ASSERT_TRUE(source_input);
  EXPECT_EQ(source_key, InstalledV2Profile(adapter)->path_key);
  EXPECT_EQ(source_input.get(),
            InstalledV2SourceInput(adapter).get());
  const std::shared_ptr<const TubeBuildRequestV2> current_request =
      std::atomic_load(&adapter.latest_build_request_);
  ASSERT_TRUE(current_request);
  EXPECT_EQ(current_request->tube_worker_purpose_v2,
            TubeWorkerPurposeV2::CURRENT);

  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> successor(
      new phase_offset_navigation::TubeBuildInputV2(*source_input));
  successor->request_id = source_input->request_id + 1U;
  successor->path_key.path_instance_id =
      source_input->path_key.path_instance_id + 1U;
  successor->path_owner = std::static_pointer_cast<const void>(
      std::make_shared<int>(73));
  ASSERT_TRUE(successor->complete());

  const double delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand port_before =
      adapter.runtime_->previousFinalPort();
  const std::uint64_t deliveries_before =
      adapter.v2_shadow_worker_->stats().delivery_count;
  ASSERT_TRUE(adapter.enqueueV2SuccessorRequest(source_key, *successor));
  ASSERT_TRUE(WaitShadowDelivery(adapter, deliveries_before + 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_);
  EXPECT_TRUE(adapter.latest_v2_shadow_successor_completion_->built());
  EXPECT_EQ(adapter.latest_v2_shadow_successor_completion_->purpose,
            TubeWorkerPurposeV2::SUCCESSOR);
  EXPECT_EQ(adapter.latest_v2_shadow_successor_completion_->request_id,
            successor->request_id);
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().peak_in_flight, 1U);
  EXPECT_EQ(std::atomic_load(&adapter.latest_build_request_), current_request);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w,
                   port_before.u_w);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   port_before.u_delta);
  EXPECT_FALSE(adapter.enqueueV2SuccessorRequest(
      phase_offset_navigation::TubePathKey(), *successor));
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS6C,
     FailedSuccessorPublishAndPrefixExpiryPreserveSourceBinding) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::D1B;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = false;
  std::shared_ptr<ContinuousPhasePath> mutable_source(
      new ContinuousPhasePath());
  ASSERT_TRUE(mutable_source->appendSegment(
      0.0, 3.0, "s6b-copied-source",
      [](const double w, ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.vel = state.dp_dw;
        state.valid = std::isfinite(w);
        return state.valid;
      }));
  mutable_source->setPathRevision(7U);
  const std::shared_ptr<const ContinuousPhasePath> source_owner =
      mutable_source;
  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterInput command;
  MatchedAdapterOutput output;
  phase_offset_core::PortCommand bootstrap_u;
  bootstrap_u.u_delta = 0.20;
  ASSERT_TRUE(InstallS4ShadowProfileForTest(
      adapter, config, command, output, bootstrap_u, source_owner));
  ASSERT_TRUE(adapter.v2_execution_binding_);
  const double delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  ASSERT_NE(delta_before, 0.0);
  ASSERT_DOUBLE_EQ(previous_before.u_delta, bootstrap_u.u_delta);
  const phase_offset_navigation::TubeFiniteReserveV2* reserve_before =
      adapter.runtime_->committedV2Reserve();
  ASSERT_TRUE(reserve_before);
  ASSERT_LT(reserve_before->cursor, reserve_before->steps.size());
  const std::uint64_t reserve_id_before = reserve_before->reserve_id;
  const std::size_t reserve_cursor_before = reserve_before->cursor;
  const phase_offset_navigation::TubeReserveStepV2 live_reserve_step =
      reserve_before->steps[reserve_cursor_before];
  ASSERT_DOUBLE_EQ(live_reserve_step.before.delta, delta_before);
  ASSERT_DOUBLE_EQ(live_reserve_step.before.previous_u.u_w,
                   previous_before.u_w);
  ASSERT_DOUBLE_EQ(live_reserve_step.before.previous_u.u_delta,
                   previous_before.u_delta);
  std::shared_ptr<ContinuousPhasePath> mutable_successor(
      new ContinuousPhasePath());
  const double kPrefixStart = live_reserve_step.before.w;
  const double kPrefixEnd = kPrefixStart + 0.35;
  ASSERT_TRUE(mutable_successor->appendSlice(
      *source_owner, kPrefixStart, kPrefixEnd));
  ASSERT_TRUE(mutable_successor->appendSegment(
      kPrefixEnd, 3.0, "s6b-successor-tail",
      [](const double w, ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.vel = state.dp_dw;
        state.valid = std::isfinite(w);
        return state.valid;
      }));
  mutable_successor->setPathRevision(7U);
  const std::shared_ptr<const ContinuousPhasePath> successor_owner =
      mutable_successor;

  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> source_input(
      new phase_offset_navigation::TubeBuildInputV2(
          *InstalledV2SourceInput(adapter)));
  ASSERT_EQ(source_input->path_owner.get(), source_owner.get());
  ASSERT_EQ(InstalledV2Profile(adapter)->path_owner.get(),
            source_owner.get());

  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> successor_input(
      new phase_offset_navigation::TubeBuildInputV2(*source_input));
  successor_input->request_id = source_input->request_id + 1U;
  successor_input->path_key.path_instance_id += 1U;
  successor_input->path_key.domain_start = kPrefixStart;
  successor_input->requested_start = kPrefixStart;
  successor_input->anchor_w = kPrefixStart;
  successor_input->producer_breakpoints = {
      kPrefixStart, kPrefixEnd, 3.0};
  successor_input->path_owner =
      std::static_pointer_cast<const void>(successor_owner);
  ASSERT_TRUE(successor_input->complete());

  std::shared_ptr<TubeV2SuccessorHandoffEvidence> handoff(
      new TubeV2SuccessorHandoffEvidence());
  handoff->valid = true;
  handoff->structurally_copied_prefix = true;
  handoff->expected_execution_generation =
      source_input->path_key.execution_generation;
  handoff->source_path_key = source_input->path_key;
  handoff->source_path_owner = source_owner;
  handoff->successor_path_key = successor_input->path_key;
  handoff->successor_path_owner = successor_owner;
  handoff->phase_after_w = kPrefixStart;
  handoff->copied_prefix_start_w = kPrefixStart;
  handoff->copied_prefix_end_w = kPrefixEnd;
  handoff->successor_request =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
          successor_input);
  handoff->provenance =
      "test/buildPhaseV2C2Frontend/appendSlice";
  const std::shared_ptr<const TubeV2SuccessorHandoffEvidence> valid_handoff(
      handoff);
  command.tube_v2_successor_handoff = valid_handoff;
  command.path = MakeStraightState(kPrefixStart);
  command.position = command.path.p +
      Eigen::Vector3d(0.0, delta_before, 0.1);
  const std::shared_ptr<const TubeV2ExecutionBinding> binding_before =
      adapter.v2_execution_binding_;
  ASSERT_TRUE(RefreshV2BaseRateFromCommittedState(adapter, command));
  const std::uint64_t deliveries_before =
      adapter.v2_shadow_worker_->stats().delivery_count;
  ASSERT_TRUE(adapter.enqueueV2SuccessorRequest(
      handoff->source_path_key, *successor_input));
  ASSERT_TRUE(WaitShadowDelivery(adapter, deliveries_before + 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_);
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_->built());

  ASSERT_TRUE(adapter.update(command, output));
  const std::shared_ptr<const TubeV2ShadowAdmissionCandidate> admitted =
      output.v2_shadow_admission_candidate;
  ASSERT_TRUE(admitted) << output.invalid_reason;
  EXPECT_EQ(admitted->purpose, TubeWorkerPurposeV2::SUCCESSOR);
  EXPECT_TRUE(admitted->applicable) << admitted->reason;
  EXPECT_TRUE(admitted->prepared);
  EXPECT_FALSE(admitted->nonselecting);
  EXPECT_TRUE(admitted->copied_prefix_continuity_valid);
  EXPECT_TRUE(admitted->prepared_step.binding_transition);
  EXPECT_EQ(admitted->prepared_step.expected_identity.path_instance_id,
            source_input->path_key.path_instance_id);
  EXPECT_EQ(admitted->prepared_step.identity.path_instance_id,
            successor_input->path_key.path_instance_id);
  EXPECT_EQ(admitted->prepared_step.identity.binding_sequence,
            admitted->prepared_step.expected_identity.binding_sequence + 1U);
  EXPECT_DOUBLE_EQ(admitted->path_position_residual, 0.0);
  EXPECT_DOUBLE_EQ(admitted->path_derivative_residual, 0.0);
  EXPECT_DOUBLE_EQ(admitted->reference_position_residual, 0.0);
  EXPECT_DOUBLE_EQ(admitted->reference_derivative_residual, 0.0);
  EXPECT_EQ(admitted->source_path_key, source_input->path_key);
  EXPECT_EQ(admitted->path_key, successor_input->path_key);
  ASSERT_TRUE(admitted->prepared_step.admission.successor_reserve.valid);
  EXPECT_EQ(admitted->prepared_step.admission.successor_reserve.identity
                .path_instance_id,
            successor_input->path_key.path_instance_id);
  EXPECT_DOUBLE_EQ(admitted->prepared_step.expected_current.delta,
                   delta_before);
  EXPECT_DOUBLE_EQ(admitted->prepared_step.expected_current.previous_u.u_w,
                   previous_before.u_w);
  EXPECT_DOUBLE_EQ(
      admitted->prepared_step.expected_current.previous_u.u_delta,
      previous_before.u_delta);
  EXPECT_LE(std::abs(admitted->prepared_step.selected_u.u_w -
                     previous_before.u_w),
            command.dt * command.tube_v2_admission.limits.u_w_slew_rate);
  EXPECT_LE(std::abs(admitted->prepared_step.selected_u.u_delta -
                     previous_before.u_delta),
            command.dt *
                command.tube_v2_admission.limits.u_delta_slew_rate);
  EXPECT_LE(admitted->prepared_step.successor.w, kPrefixEnd);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w,
                   previous_before.u_w);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   previous_before.u_delta);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve(), reserve_before);
  EXPECT_EQ(InstalledV2Profile(adapter)->path_key,
            source_input->path_key);
  ASSERT_TRUE(adapter.pending_v2_shadow_bootstrap_candidate_);
  const PendingPositionCommandCapture pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(pending.pending);
  ASSERT_TRUE(pending.valid);
  ASSERT_TRUE(pending.v2_binding_transition);
  ASSERT_EQ(pending.proposed_v2_binding, admitted->proposed_binding);
  int post_publish_count = 0;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      []() { return false; }, pending.identity, false,
      [&post_publish_count]() { ++post_publish_count; }));
  EXPECT_EQ(post_publish_count, 0);
  EXPECT_EQ(adapter.v2_execution_binding_, binding_before);
  EXPECT_EQ(InstalledV2Profile(adapter)->path_key, source_input->path_key);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   previous_before.u_delta);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve(), reserve_before);
  EXPECT_TRUE(adapter.hasPendingPositionCommand());
  adapter.discardPendingPositionCommand();
  EXPECT_FALSE(adapter.hasPendingPositionCommand());

  // Adversarial handoff substitutions replace the immutable command value;
  // they never mutate the already-published const evidence through an alias.
  std::shared_ptr<ContinuousPhasePath> substituted_source(
      new ContinuousPhasePath());
  ASSERT_TRUE(substituted_source->appendSegment(
      0.0, 3.0, "s6b-substituted-source",
      [](const double w, ContinuousPhasePathState& state) {
        state.p = Eigen::Vector3d(w, 0.0, 1.0);
        state.dp_dw = Eigen::Vector3d::UnitX();
        state.d2p_dw2.setZero();
        state.vel = state.dp_dw;
        state.valid = std::isfinite(w);
        return state.valid;
      }));
  substituted_source->setPathRevision(7U);
  const auto expect_malformed_handoff =
      [&](const std::function<void(TubeV2SuccessorHandoffEvidence&)>& mutate) {
        std::shared_ptr<TubeV2SuccessorHandoffEvidence> malformed(
            new TubeV2SuccessorHandoffEvidence(*valid_handoff));
        mutate(*malformed);
        command.tube_v2_successor_handoff =
            std::shared_ptr<const TubeV2SuccessorHandoffEvidence>(malformed);
        // Rejecting malformed successor evidence must not suppress an
        // independently admissible command on the installed source binding.
        // The staged command is therefore CURRENT, never a transition to the
        // substituted successor.
        EXPECT_TRUE(adapter.update(command, output));
        EXPECT_TRUE(output.selected);
        EXPECT_TRUE(output.valid);
        ASSERT_TRUE(output.v2_shadow_admission_candidate);
        EXPECT_EQ(output.v2_shadow_admission_candidate->purpose,
                  TubeWorkerPurposeV2::CURRENT);
        EXPECT_TRUE(output.v2_shadow_admission_candidate->applicable)
            << output.v2_shadow_admission_candidate->reason;
        EXPECT_FALSE(output.v2_shadow_admission_candidate->prepared_step
                         .binding_transition);
        EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), delta_before);
        EXPECT_EQ(adapter.runtime_->committedV2Reserve(), reserve_before);
        const PendingPositionCommandCapture source_pending =
            adapter.capturePendingPositionCommand();
        ASSERT_TRUE(source_pending.pending);
        ASSERT_TRUE(source_pending.valid);
        EXPECT_FALSE(source_pending.v2_binding_transition);
        adapter.discardPendingPositionCommand();
        command.tube_v2_successor_handoff = valid_handoff;
        EXPECT_TRUE(adapter.update(command, output));
        ASSERT_TRUE(output.v2_shadow_admission_candidate);
        EXPECT_EQ(output.v2_shadow_admission_candidate->purpose,
                  TubeWorkerPurposeV2::SUCCESSOR);
        EXPECT_TRUE(output.v2_shadow_admission_candidate->applicable)
            << output.v2_shadow_admission_candidate->reason;
        const PendingPositionCommandCapture successor_pending =
            adapter.capturePendingPositionCommand();
        ASSERT_TRUE(successor_pending.pending);
        ASSERT_TRUE(successor_pending.valid);
        EXPECT_TRUE(successor_pending.v2_binding_transition);
        adapter.discardPendingPositionCommand();
      };
  expect_malformed_handoff([&](TubeV2SuccessorHandoffEvidence& malformed) {
    malformed.source_path_owner = substituted_source;
  });
  expect_malformed_handoff([](TubeV2SuccessorHandoffEvidence& malformed) {
    malformed.source_path_key.domain_end = std::nextafter(
        malformed.source_path_key.domain_end,
        -std::numeric_limits<double>::infinity());
  });
  expect_malformed_handoff([](TubeV2SuccessorHandoffEvidence& malformed) {
    std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> request(
        new phase_offset_navigation::TubeBuildInputV2(
            *malformed.successor_request));
    request->requested_start = std::nextafter(
        malformed.copied_prefix_start_w,
        -std::numeric_limits<double>::infinity());
    malformed.successor_request = request;
  });
  expect_malformed_handoff([](TubeV2SuccessorHandoffEvidence& malformed) {
    std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> request(
        new phase_offset_navigation::TubeBuildInputV2(
            *malformed.successor_request));
    request->anchor_w = std::nextafter(
        malformed.phase_after_w, std::numeric_limits<double>::infinity());
    malformed.successor_request = request;
  });
  expect_malformed_handoff([](TubeV2SuccessorHandoffEvidence& malformed) {
    std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> request(
        new phase_offset_navigation::TubeBuildInputV2(
            *malformed.successor_request));
    request->requested_end = std::nextafter(
        malformed.copied_prefix_end_w,
        -std::numeric_limits<double>::infinity());
    malformed.successor_request = request;
  });
  EXPECT_TRUE(valid_handoff->structurally_copied_prefix);

  // An untrusted structural claim remains a non-mutating availability
  // denial; it cannot replace the source binding or its reserve.
  std::shared_ptr<TubeV2SuccessorHandoffEvidence> untrusted_handoff(
      new TubeV2SuccessorHandoffEvidence(*valid_handoff));
  untrusted_handoff->structurally_copied_prefix = false;
  command.tube_v2_successor_handoff = untrusted_handoff;
  ASSERT_TRUE(adapter.update(command, output));
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.valid);
  ASSERT_TRUE(output.v2_shadow_admission_candidate);
  EXPECT_EQ(output.v2_shadow_admission_candidate->purpose,
            TubeWorkerPurposeV2::CURRENT);
  EXPECT_TRUE(output.v2_shadow_admission_candidate->applicable)
      << output.v2_shadow_admission_candidate->reason;
  EXPECT_FALSE(output.v2_shadow_admission_candidate->prepared_step
                   .binding_transition);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), delta_before);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   previous_before.u_delta);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve(), reserve_before);
  EXPECT_EQ(adapter.v2_execution_binding_, binding_before);
  const PendingPositionCommandCapture untrusted_source_pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(untrusted_source_pending.pending);
  ASSERT_TRUE(untrusted_source_pending.valid);
  EXPECT_FALSE(untrusted_source_pending.v2_binding_transition);
  adapter.discardPendingPositionCommand();

  // Use the exact committed reserve state and make the certified copied
  // prefix expire inside its next held ZOH step.  The successor cannot be
  // selected; the incumbent's first finite-reserve step must be selected
  // through the same publish-first transaction instead.
  ASSERT_GT(live_reserve_step.after.w, live_reserve_step.before.w);
  const double expiring_prefix_end =
      0.5 * (live_reserve_step.before.w + live_reserve_step.after.w);
  ASSERT_GT(expiring_prefix_end, kPrefixStart);
  ASSERT_LT(expiring_prefix_end, kPrefixEnd);
  std::shared_ptr<TubeV2SuccessorHandoffEvidence> expiring_handoff(
      new TubeV2SuccessorHandoffEvidence(*valid_handoff));
  expiring_handoff->copied_prefix_end_w = expiring_prefix_end;
  command.tube_v2_successor_handoff = expiring_handoff;
  ASSERT_TRUE(RefreshV2BaseRateFromCommittedState(adapter, command));
  ASSERT_TRUE(adapter.update(command, output));
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(output.valid);
  ASSERT_TRUE(output.v2_shadow_admission_candidate);
  EXPECT_TRUE(output.v2_shadow_admission_candidate->recovery_step.valid);
  EXPECT_EQ(output.recovery_status,
            phase_offset_navigation::RecoveryStepStatus::PREPARED);
  EXPECT_EQ(adapter.v2_execution_binding_, binding_before);
  const PendingPositionCommandCapture reserve_pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(reserve_pending.pending);
  ASSERT_TRUE(reserve_pending.valid);
  EXPECT_FALSE(reserve_pending.v2_binding_transition);
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      []() { return false; }, reserve_pending.identity));
  EXPECT_EQ(adapter.v2_execution_binding_, binding_before);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve(), reserve_before);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve()->cursor,
            reserve_cursor_before);
  ASSERT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, reserve_pending.identity));
  EXPECT_EQ(adapter.v2_execution_binding_, binding_before);
  ASSERT_TRUE(adapter.runtime_->committedV2Reserve());
  EXPECT_EQ(adapter.runtime_->committedV2Reserve()->reserve_id,
            reserve_id_before);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve()->cursor,
            reserve_cursor_before + 1U);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(),
                   live_reserve_step.after.delta);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w,
                   live_reserve_step.after.previous_u.u_w);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   live_reserve_step.after.previous_u.u_delta);

  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS2B,
     V2ShadowPathReplacementCancelsOnlyIncomingPurpose) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  const auto v2config = adapter.v2_shadow_worker_->config();
  MatchedAdapterInput command = MakeInput(MakeStraightSyntheticPath());
  MatchedAdapterOutput output;

  command.tube_worker_input_v2 = MakeShadowInput(v2config, 50U);
  command.tube_worker_purpose_v2 = TubeWorkerPurposeV2::SUCCESSOR;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_);

  const std::shared_ptr<ShadowCallbackBarrier> barrier(
      new ShadowCallbackBarrier());
  command.tube_worker_input_v2 = MakeBlockedShadowInput(v2config, 51U, barrier);
  command.tube_worker_purpose_v2 = TubeWorkerPurposeV2::CURRENT;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowCallbackEntered(barrier));

  // Replace only CURRENT while SUCCESSOR still uses the old path key.  The
  // adapter must cancel the blocked CURRENT request without globally
  // invalidating or overwriting the retained SUCCESSOR proof.
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> replacement(
      new phase_offset_navigation::TubeBuildInputV2(*MakeShadowInput(
          v2config, 52U)));
  replacement->path_key.path_instance_id = 4U;
  command.tube_worker_input_v2 =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
          replacement);
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  {
    std::lock_guard<std::mutex> lock(barrier->mutex);
    barrier->release = true;
  }
  barrier->condition.notify_all();
  ASSERT_TRUE(WaitShadowDelivery(adapter, 2U));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 52U);
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_successor_completion_->request_id, 50U);
  EXPECT_GT(adapter.v2_shadow_worker_->stats().cancellation_discarded, 0U);
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, 3U);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS2B,
     V2ShadowShutdownDuringBlockedCallbackDoesNotReinsertCompletion) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  const auto v2config = adapter.v2_shadow_worker_->config();
  const std::shared_ptr<ShadowCallbackBarrier> barrier(
      new ShadowCallbackBarrier());
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> input =
      MakeBlockedShadowInput(v2config, 61U, barrier);
  const std::weak_ptr<const void> weak_owner(input->path_owner);
  MatchedAdapterInput command = MakeInput(MakeStraightSyntheticPath());
  command.tube_worker_input_v2 =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(input);
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowCallbackEntered(barrier));

  adapter.requestShutdown();
  EXPECT_TRUE(adapter.shutdown_requested_.load(std::memory_order_acquire));
  {
    std::lock_guard<std::mutex> lock(barrier->mutex);
    barrier->release = true;
  }
  barrier->condition.notify_all();
  adapter.shutdown();
  EXPECT_TRUE(adapter.v2_shadow_worker_->joined());
  EXPECT_FALSE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_FALSE(adapter.latest_v2_shadow_successor_completion_);
  input.reset();
  command.tube_worker_input_v2.reset();
  const auto expiry_deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(3);
  while (!weak_owner.expired() &&
         std::chrono::steady_clock::now() < expiry_deadline) {
    std::this_thread::yield();
  }
  EXPECT_TRUE(weak_owner.expired());
}

TEST(PhaseOffsetMatchedAdapterS2B, V2ShadowCompletionRetainsReleasedInputOwner) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  const auto v2config = adapter.v2_shadow_worker_->config();
  std::shared_ptr<const void> caller_owner;
  std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2> input =
      MakeShadowInput(v2config, 41U, 1U, &caller_owner);
  const std::weak_ptr<const void> weak_owner(caller_owner);
  MatchedAdapterInput command = MakeInput(MakeStraightSyntheticPath());
  command.tube_worker_input_v2 = input;
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(command, output));
  input.reset();
  caller_owner.reset();
  command.tube_worker_input_v2.reset();
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  // Drop the adapter's request transport; the copied completion/profile must
  // still retain the immutable path owner until its observational value is
  // released.
  ASSERT_TRUE(WaitShadowIdle(adapter));
  std::atomic_store(&adapter.latest_build_request_,
                    std::shared_ptr<const TubeBuildRequestV2>());
  EXPECT_FALSE(weak_owner.expired());
  adapter.latest_v2_shadow_current_completion_.reset();
  const auto expiry_deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(3);
  while (!weak_owner.expired() &&
         std::chrono::steady_clock::now() < expiry_deadline) {
    std::this_thread::yield();
  }
  EXPECT_TRUE(weak_owner.expired());
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS3C,
     CurrentCompletionPreparesNonSelectingAdmissionWithoutCommit) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = true;
  PhaseOffsetMatchedAdapter adapter(config);
  const auto v2config = adapter.v2_shadow_worker_->config();
  MatchedAdapterInput command = MakeInput(MakeStraightSyntheticPath());
  // Keep the frozen 2.0-w preview horizon from the production policy intact:
  // this S3-C profile must cover the complete [0.4, 2.4] live window rather
  // than silently shortening the policy to fit the small S2-B fixture.
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> s3c_input(
      new phase_offset_navigation::TubeBuildInputV2(
          *MakeShadowInput(v2config, 701U)));
  s3c_input->path_key.domain_end = 3.0;
  s3c_input->requested_end = 3.0;
  s3c_input->producer_breakpoints = {0.0, 3.0};
  const std::shared_ptr<const ContinuousPhasePath> s3c_path_owner =
      MakeStraightPathOwner(0.0, 3.0, 7U);
  ASSERT_TRUE(s3c_path_owner);
  s3c_input->path_owner =
      std::static_pointer_cast<const void>(s3c_path_owner);
  command.semantic_path_owner = s3c_path_owner;
  command.semantic_path_start_w = s3c_path_owner->startW();
  command.semantic_path_end_w = s3c_path_owner->endW();
  s3c_input->path_cell_query =
      [](const double w0, const double w1,
         phase_offset_core::CertifiedPathCellV2& cell) {
        if (!std::isfinite(w0) || !std::isfinite(w1) || !(w1 > w0) ||
            w0 < 0.0 || w1 > 3.0) {
          return false;
        }
        cell = MakeShadowCell(w0, w1);
        return phase_offset_core::certifiedPathCellV2IsComplete(cell);
      };
  command.tube_worker_input_v2 =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
          s3c_input);
  command.tube_v2_admission.valid = true;
  command.tube_v2_admission.binding_sequence = 701U;
  command.tube_v2_admission.accepted_state_sequence = 5U;
  command.tube_v2_admission.accepted_state_notification_sequence = 5U;
  command.tube_v2_admission.accepted_time_ticks = 10U;
  command.tube_v2_admission.support_expiry_ticks = 20U;
  command.tube_v2_admission.map_instance_id = 3U;
  command.tube_v2_admission.configuration_generation = 6U;
  command.tube_v2_admission.configuration_key = 99U;
  command.tube_v2_admission.support_provenance_id = 9U;
  command.tube_v2_admission.frame_provenance = "world";
  command.tube_v2_admission.latest_accepted_state_sequence = 5U;
  command.tube_v2_admission.latest_accepted_state_notification_sequence = 5U;
  command.tube_v2_admission.latest_accepted_time_ticks = 10U;
  command.tube_v2_admission.latest_map_instance_id = 3U;
  command.tube_v2_admission.latest_configuration_generation = 6U;
  command.tube_v2_admission.latest_configuration_key = 99U;
  command.tube_v2_admission.latest_support_provenance_id = 9U;
  command.tube_v2_admission.latest_frame_provenance = "world";
  command.tube_v2_admission.selected_u = phase_offset_core::PortCommand();
  command.tube_v2_admission.base_phase_rate = 0.02;
  command.tube_v2_admission.phase_rate_lower = 0.02;
  command.tube_v2_admission.phase_rate_upper = 2.0;
  command.tube_v2_admission.upper_u_delta = 0.40;
  command.tube_v2_admission.now = 10.0;
  command.tube_v2_admission.applicability_deadline = 20.0;
  command.tube_v2_admission.applicability_deadline_valid = true;
  command.tube_v2_admission.preview_policy = config.normal_preview_policy;
  // The V2 preview policy is evidence for this frozen certificate
  // configuration, so its identity must match the captured V2 key while the
  // production control values above remain unchanged.
  command.tube_v2_admission.preview_policy.configuration_identity =
      config.tube_certificate_v2.configuration_id;
  command.tube_v2_admission.limits.lower_phase_rate = 0.02;
  command.tube_v2_admission.limits.upper_phase_rate = 2.0;
  command.tube_v2_admission.limits.upper_nu = 2.0;
  command.tube_v2_admission.limits.max_u_w = config.u_w_abs_max;
  command.tube_v2_admission.limits.max_u_delta = config.u_delta_abs_max;
  command.tube_v2_admission.limits.u_w_slew_rate = 100.0;
  command.tube_v2_admission.limits.u_delta_slew_rate = 100.0;
  command.tube_v2_admission.limits.return_u_delta_max =
      config.u_delta_abs_max;
  command.tube_v2_admission.limits.return_u_delta_slew_rate = 100.0;
  command.tube_v2_admission.limits.max_schedule_steps = 1000U;
  command.tube_v2_admission.limits.max_work = 250000U;
  command.tube_v2_admission.limits.valid = true;
  command.tube_v2_admission.tracking.valid = true;
  command.tube_v2_admission.tracking.error_norm = 0.0;
  command.tube_v2_admission.tracking.error_bound = 0.15;
  command.tube_v2_admission.tracking.physical_tangent_valid = true;
  command.tube_v2_admission.max_work = 250000U;
  command.tube_v2_admission.provenance = "S3C/test/nonselecting";

  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, 1U));
  const double retained_delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_before =
      adapter.runtime_->previousFinalPort();
  const phase_offset_navigation::TubeFiniteReserveV2* reserve_before =
      adapter.runtime_->committedV2Reserve();
  const auto authority_before = adapter.execution_authority_.snapshotPtr();
  const auto binding_before = adapter.v2_execution_binding_;
  const auto pending_before = adapter.pending_v2_shadow_bootstrap_candidate_;
  const auto expect_unchanged = [&]() {
    EXPECT_EQ(adapter.runtime_->retainedDelta(), retained_delta_before);
    EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w,
                     previous_before.u_w);
    EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                     previous_before.u_delta);
    EXPECT_EQ(adapter.runtime_->committedV2Reserve(), reserve_before);
    EXPECT_EQ(adapter.execution_authority_.snapshotPtr(), authority_before);
    EXPECT_EQ(adapter.v2_execution_binding_, binding_before);
    EXPECT_EQ(adapter.pending_v2_shadow_bootstrap_candidate_, pending_before);
    EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, 1U);
  };
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->nonselecting);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->prepared);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->commit_token.valid);
  EXPECT_EQ(adapter.runtime_->retainedDelta(), retained_delta_before);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w,
                   previous_before.u_w);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   previous_before.u_delta);
  EXPECT_EQ(adapter.execution_authority_.snapshotPtr(), authority_before);
  EXPECT_FALSE(adapter.v2_execution_binding_);
  EXPECT_FALSE(adapter.pending_v2_shadow_bootstrap_candidate_);
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, 1U);

  // Simulate a failed publication by retaining the prepared value without
  // calling any commit/publication seam, then retry the same immutable input.
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  expect_unchanged();

  // Tracking, deadline, and binding substitutions fail closed and are
  // retryable after the exact evidence is restored.
  const Eigen::Vector3d nominal_position = command.position;
  command.position = command.path.p + Eigen::Vector3d(0.0, 0.0, 0.30);
  ASSERT_TRUE(RefreshLegacyGuidance(command));
  ASSERT_FALSE(adapter.update(command, output));
  EXPECT_FALSE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  EXPECT_EQ(adapter.latest_v2_shadow_admission_candidate_->status,
            phase_offset_navigation::TubeExecutionStatusV2::
                TRACKING_UNAVAILABLE);
  expect_unchanged();
  command.position = nominal_position;
  ASSERT_TRUE(RefreshLegacyGuidance(command));
  command.tube_v2_admission.now = 20.0;
  ASSERT_FALSE(adapter.update(command, output));
  EXPECT_FALSE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  expect_unchanged();
  command.tube_v2_admission.now = 10.0;
  command.dt = 0.0;
  ASSERT_FALSE(adapter.update(command, output));
  expect_unchanged();
  command.dt = kDt;
  command.tube_v2_admission.binding_sequence = 702U;
  ASSERT_FALSE(adapter.update(command, output));
  EXPECT_FALSE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  expect_unchanged();
  command.tube_v2_admission.binding_sequence = 701U;
  ASSERT_FALSE(adapter.update(command, output));
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  expect_unchanged();

  // A newer accepted notification is carried as bounded value evidence and
  // remains admissible only while the old profile's support/deadline contract
  // is still valid; no map query or geometry proof is restarted.
  command.tube_v2_admission.latest_accepted_state_sequence = 6U;
  command.tube_v2_admission.latest_accepted_state_notification_sequence = 6U;
  command.tube_v2_admission.latest_accepted_time_ticks = 11U;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->accepted_update_visible);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->accepted_update_compatible);
  expect_unchanged();
  command.tube_v2_admission.latest_accepted_state_sequence = 5U;
  command.tube_v2_admission.latest_accepted_state_notification_sequence = 5U;
  command.tube_v2_admission.latest_accepted_time_ticks = 10U;

  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> stale_path(
      new phase_offset_navigation::TubeBuildInputV2(
          *command.tube_worker_input_v2));
  stale_path->path_key.path_instance_id += 1U;
  command.tube_worker_input_v2 =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(stale_path);
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
  EXPECT_FALSE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  expect_unchanged();
  command.tube_worker_input_v2 =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(
          s3c_input);

  // A substituted accepted-map identity is rejected as stale evidence and
  // leaves the prepared/non-authoritative state and live Runtime untouched.
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> stale(
      new phase_offset_navigation::TubeBuildInputV2(
          *command.tube_worker_input_v2));
  stale->map_capture_key.state_id += 1U;
  stale->map_capture_key.accepted_sequence += 1U;
  command.tube_worker_input_v2 =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(stale);
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
  EXPECT_FALSE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  EXPECT_EQ(adapter.runtime_->retainedDelta(), retained_delta_before);
  EXPECT_EQ(adapter.execution_authority_.snapshotPtr(), authority_before);
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, 1U);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS4A,
     V2ShadowBootstrapPublishesBeforeCommitAndRetriesExactCompletion) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = false;
  PhaseOffsetMatchedAdapter adapter(config);
  ASSERT_TRUE(adapter.configurationValid());
  ASSERT_TRUE(adapter.v2_shadow_worker_);

  MatchedAdapterInput command = MakeInput(MakeStraightSyntheticPath());
  PopulateS4ShadowAdmissionInput(config, command);
  const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      original_source = command.tube_worker_input_v2;
  MatchedAdapterOutput output;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_->built());
  const std::uint64_t build_count =
      adapter.v2_shadow_worker_->stats().build_started;
  ASSERT_EQ(build_count, 1U);

  // A delayed live-w update reuses the completed immutable cohort; it must
  // not invoke the worker a second time or create a second lifecycle token.
  command.path = MakeStraightState(0.45);
  ASSERT_TRUE(RefreshLegacyGuidance(command));
  bool selected = false;
  for (int cycle = 0;
       cycle <= config.warmup_cycles &&
           !adapter.pending_v2_shadow_bootstrap_candidate_;
       ++cycle) {
    selected = adapter.update(command, output);
  }
  ASSERT_TRUE(selected)
      << "gate_open=" << adapter.zero_gate_open_
      << " count=" << adapter.zero_gate_consecutive_count_
      << " output_reason=" << output.invalid_reason
      << " admission_reason="
      << (output.v2_shadow_admission_candidate
              ? output.v2_shadow_admission_candidate->reason
              : std::string("<none>"));
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, build_count);
  ASSERT_TRUE(adapter.pending_v2_shadow_bootstrap_candidate_);
  ASSERT_TRUE(adapter.pending_v2_shadow_bootstrap_candidate_->profile);
  EXPECT_FALSE(adapter.v2_execution_binding_);
  EXPECT_FALSE(adapter.runtime_->committedV2Reserve());
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w, 0.0);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta, 0.0);
  EXPECT_TRUE(output.selected);
  EXPECT_TRUE(output.profile_active);

  // A newer accepted-map notification that remains inside the immutable
  // support/deadline contract is carried as evidence only; it neither starts
  // another build nor invalidates the already staged activation.
  command.tube_v2_admission.latest_accepted_state_sequence = 6U;
  command.tube_v2_admission.latest_accepted_state_notification_sequence = 6U;
  command.tube_v2_admission.latest_accepted_time_ticks = 11U;
  ASSERT_TRUE(adapter.update(command, output));
  ASSERT_TRUE(adapter.pending_v2_shadow_bootstrap_candidate_);
  ASSERT_TRUE(output.v2_shadow_admission_candidate);
  EXPECT_EQ(output.v2_shadow_admission_candidate,
            adapter.pending_v2_shadow_bootstrap_candidate_);
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, build_count);

  // An evidence-only map identity substitution (with the same immutable
  // builder input) is incompatible and must retire the pending token.
  command.tube_v2_admission.latest_map_instance_id = 4U;
  ASSERT_FALSE(adapter.update(command, output));
  EXPECT_FALSE(adapter.pending_v2_shadow_bootstrap_candidate_);
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, build_count);
  command.tube_v2_admission.latest_map_instance_id = 3U;
  command.tube_v2_admission.latest_accepted_state_sequence = 5U;
  command.tube_v2_admission.latest_accepted_state_notification_sequence = 5U;
  command.tube_v2_admission.latest_accepted_time_ticks = 10U;
  ASSERT_TRUE(adapter.update(command, output));
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(adapter.pending_v2_shadow_bootstrap_candidate_);

  // Substituting the captured map identity is incompatible and retires the
  // pending token; restoring the exact source cohort allows the same retained
  // completion to be staged again without a worker rebuild.
  std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> stale_map(
      new phase_offset_navigation::TubeBuildInputV2(
          *command.tube_worker_input_v2));
  stale_map->map_capture_key.state_id = 6U;
  stale_map->map_capture_key.accepted_sequence = 6U;
  command.tube_worker_input_v2 =
      std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(stale_map);
  ASSERT_FALSE(adapter.update(command, output));
  EXPECT_FALSE(adapter.pending_v2_shadow_bootstrap_candidate_);
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, build_count);
  // Reuse the original immutable S3-C source payload (including its owner
  // callbacks) rather than deriving a new map/path certificate here.
  command.tube_worker_input_v2 = original_source;
  command.tube_v2_admission.latest_accepted_state_sequence = 5U;
  command.tube_v2_admission.latest_accepted_state_notification_sequence = 5U;
  command.tube_v2_admission.latest_accepted_time_ticks = 10U;
  ASSERT_TRUE(adapter.update(command, output));
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(adapter.pending_v2_shadow_bootstrap_candidate_);
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, build_count);

  const std::uint64_t profile_id =
      adapter.pending_v2_shadow_bootstrap_candidate_->profile->profile_id;
  EXPECT_TRUE(adapter.hasPendingPositionCommand());
  const PendingPositionCommandCapture pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(pending.pending);
  ASSERT_TRUE(pending.valid);
  ASSERT_NE(pending.identity, 0U);
  ASSERT_TRUE(pending.reference_query);
  EXPECT_EQ(adapter.pendingExecutedReferenceQuery(), pending.reference_query);
  bool wrong_identity_publish_called = false;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      [&wrong_identity_publish_called]() {
        wrong_identity_publish_called = true;
        return true;
      }, 0U));
  EXPECT_FALSE(wrong_identity_publish_called);
  EXPECT_TRUE(adapter.hasPendingPositionCommand());
  bool mismatched_identity_publish_called = false;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      [&mismatched_identity_publish_called]() {
        mismatched_identity_publish_called = true;
        return true;
      }, pending.identity + 1U));
  EXPECT_FALSE(mismatched_identity_publish_called);
  EXPECT_TRUE(adapter.hasPendingPositionCommand());
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      []() { return false; }, pending.identity));
  EXPECT_TRUE(adapter.hasPendingPositionCommand());
  EXPECT_FALSE(adapter.v2_execution_binding_);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), 0.0);
  EXPECT_FALSE(adapter.runtime_->committedV2Reserve());

  EXPECT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, pending.identity));
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  ASSERT_TRUE(adapter.v2_execution_binding_);
  EXPECT_EQ(InstalledV2Profile(adapter)->profile_id,
            adapter.latest_v2_shadow_current_completion_->build.profile.profile_id);
  EXPECT_EQ(InstalledV2Profile(adapter)->profile_id, profile_id);
  // The installed pointer aliases the retained completion's profile member,
  // proving activation consumed the completed object rather than a clone.
  EXPECT_EQ(InstalledV2Profile(adapter).get(),
            &adapter.latest_v2_shadow_current_completion_->build.profile);
  EXPECT_TRUE(adapter.runtime_->committedV2Reserve());
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, build_count);
  // The output is the exact command whose successful publication committed
  // the binding; publication does not retroactively deselect that command.
  EXPECT_TRUE(output.selected);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS5A,
     AcceptedMapRefreshRetainsInstalledIncumbentAndRejectsRebinds) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = false;

  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterInput command;
  MatchedAdapterOutput output;
  ASSERT_TRUE(InstallS4ShadowProfileForTest(adapter, config, command, output));
  const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>
      incumbent = InstalledV2Profile(adapter);
  ASSERT_TRUE(incumbent);
  const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      original_source = command.tube_worker_input_v2;
  const std::uint64_t builds_before_refresh =
      adapter.v2_shadow_worker_->stats().build_started;

  // A coherent accepted-state advance keeps path/configuration and all
  // support identity fields fixed.  The fixture intentionally leaves the
  // old query callback attached, so this refresh's failed completion is
  // evidence that must not erase either the useful completion or incumbent.
  command.tube_worker_input_v2 = MakeAcceptedMapRefreshInput(
      original_source, incumbent->request_id + 1U, 6U, 11U);
  command.tube_v2_admission.accepted_state_sequence = 6U;
  command.tube_v2_admission.accepted_state_notification_sequence = 6U;
  command.tube_v2_admission.accepted_time_ticks = 11U;
  command.tube_v2_admission.latest_accepted_state_sequence = 6U;
  command.tube_v2_admission.latest_accepted_state_notification_sequence = 6U;
  command.tube_v2_admission.latest_accepted_time_ticks = 11U;
  ASSERT_FALSE(adapter.update(command, output));
  EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  const auto refresh_deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(3);
  while (adapter.v2_shadow_worker_ &&
         adapter.v2_shadow_worker_->stats().build_finished <
             builds_before_refresh + 1U &&
         std::chrono::steady_clock::now() < refresh_deadline) {
    std::this_thread::yield();
  }
  ASSERT_TRUE(adapter.v2_shadow_worker_);
  ASSERT_GE(adapter.v2_shadow_worker_->stats().build_finished,
            builds_before_refresh + 1U);
  adapter.consumeV2ShadowCompletions();
  EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started,
            builds_before_refresh + 1U);
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id,
            incumbent->request_id);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->map_capture_key,
            incumbent->map_capture_key);
  EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());

  // Reusing the installed profile's request ID with a different map key is
  // an identity contradiction, not an accepted refresh.  Fail closed and do
  // not allow the old profile to be rebound to that capture.
  command.tube_worker_input_v2 = MakeAcceptedMapRefreshInput(
      original_source, incumbent->request_id, 7U, 12U);
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.v2_execution_binding_);
  EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());
  adapter.shutdown();

  // A refresh gap may retain the incumbent only before its own finite
  // applicability deadline.  Missing replacement input does not grant an
  // unbounded grace period.
  PhaseOffsetMatchedAdapter deadline_adapter(config);
  MatchedAdapterInput deadline_command;
  MatchedAdapterOutput deadline_output;
  ASSERT_TRUE(InstallS4ShadowProfileForTest(
      deadline_adapter, config, deadline_command, deadline_output));
  ASSERT_TRUE(deadline_adapter.v2_execution_binding_);
  deadline_command.tube_worker_input_v2.reset();
  deadline_command.tube_v2_admission.now = 19.0;
  ASSERT_FALSE(deadline_adapter.update(deadline_command, deadline_output));
  EXPECT_TRUE(deadline_adapter.v2_execution_binding_);
  deadline_command.tube_v2_admission.now = 20.0;
  ASSERT_FALSE(deadline_adapter.update(deadline_command, deadline_output));
  EXPECT_TRUE(deadline_adapter.v2_execution_binding_);
  deadline_adapter.shutdown();

  const auto expect_incompatible_retains = [&config, &original_source](
      const std::function<void(phase_offset_navigation::TubeBuildInputV2&)>&
          mutate) {
    PhaseOffsetMatchedAdapter separate(config);
    MatchedAdapterInput separate_command;
    MatchedAdapterOutput separate_output;
    EXPECT_TRUE(InstallS4ShadowProfileForTest(
        separate, config, separate_command, separate_output));
    ASSERT_TRUE(separate.v2_execution_binding_);
    const auto installed = InstalledV2Profile(separate);
    std::shared_ptr<phase_offset_navigation::TubeBuildInputV2> changed(
        new phase_offset_navigation::TubeBuildInputV2(*original_source));
    changed->request_id = installed->request_id + 1U;
    mutate(*changed);
    separate_command.tube_worker_input_v2 =
        std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>(changed);
    EXPECT_FALSE(separate.update(separate_command, separate_output));
    EXPECT_TRUE(separate.v2_execution_binding_);
    EXPECT_EQ(InstalledV2Profile(separate).get(), installed.get());
    separate.shutdown();
  };

  expect_incompatible_retains(
      [](phase_offset_navigation::TubeBuildInputV2& changed) {
        changed.path_key.path_instance_id += 1U;
      });
  expect_incompatible_retains(
      [](phase_offset_navigation::TubeBuildInputV2& changed) {
        changed.configuration_key.configuration_id += 1U;
      });
  expect_incompatible_retains(
      [](phase_offset_navigation::TubeBuildInputV2& changed) {
        changed.map_capture_key.map_instance_id += 1U;
        changed.map_capture_key.state_id += 1U;
        changed.map_capture_key.accepted_sequence += 1U;
        changed.map_capture_key.accepted_time_ticks += 1U;
      });
}

TEST(PhaseOffsetMatchedAdapterS5B,
     IncumbentRefreshGapRechecksSuccessorAndPreparesCommittedReserve) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::D1B;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = false;

  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterInput command;
  MatchedAdapterOutput output;
  phase_offset_core::PortCommand bootstrap_selected_u;
  bootstrap_selected_u.u_delta = 0.20;
  ASSERT_TRUE(InstallS4ShadowProfileForTest(
      adapter, config, command, output, bootstrap_selected_u));
  const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>
      incumbent = InstalledV2Profile(adapter);
  ASSERT_TRUE(incumbent);
  const phase_offset_navigation::TubeFiniteReserveV2* reserve =
      adapter.runtime_->committedV2Reserve();
  ASSERT_TRUE(reserve);
  ASSERT_TRUE(reserve->valid);
  ASSERT_GT(reserve->steps.size(), 1U);
  EXPECT_NE(adapter.runtime_->retainedDelta(), 0.0);
  const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      original_source = command.tube_worker_input_v2;
  ASSERT_TRUE(original_source);

  // A worker-starvation/refresh gap has no replacement input.  The installed
  // profile is still checked through the exact V2 admission kernel, including
  // its complete next command and successor reserve, without a rebuild.
  command.tube_worker_input_v2.reset();
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->prepared_step
                  .admission.successor_reserve.valid);
  EXPECT_EQ(adapter.latest_v2_shadow_admission_candidate_->profile.get(),
            incumbent.get());
  EXPECT_EQ(adapter.runtime_->committedV2Reserve(), reserve);
  EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());

  // A coherent accepted-map advance remains compatible with the incumbent;
  // the old profile is not rebound to the newer map and the worker is not
  // required to produce a completion before the old reserve is checked.
  command.tube_v2_admission.accepted_state_sequence = 6U;
  command.tube_v2_admission.accepted_state_notification_sequence = 6U;
  command.tube_v2_admission.accepted_time_ticks = 11U;
  command.tube_v2_admission.latest_accepted_state_sequence = 6U;
  command.tube_v2_admission.latest_accepted_state_notification_sequence = 6U;
  command.tube_v2_admission.latest_accepted_time_ticks = 11U;
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->applicable);
  EXPECT_TRUE(adapter.latest_v2_shadow_admission_candidate_->accepted_update_visible);
  EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());

  // A tracking/physical applicability violation is categorically different
  // from an infeasible NORMAL command.  P10 denies it as stale offset
  // execution, so the adapter must not launder the retained reserve into a
  // certified PREPARED recovery claim.  All committed state remains intact.
  const std::uint64_t tracking_recovery_sequence_before =
      adapter.recovery_owner_.status().committed_sequence;
  const double tracking_delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand tracking_port_before =
      adapter.runtime_->previousFinalPort();
  SetS5ShadowRecoveryState(reserve->initial, command);
  // Tracking evidence is recomputed from live geometry in Stage 7.  Move the
  // actual vehicle position outside the configured bound and refresh the
  // independent zero-port snapshot; mutating the incoming evidence field
  // would be inert and would not exercise P10.
  command.position = command.path.p +
      command.path.N * reserve->initial.delta +
      Eigen::Vector3d(
          0.0, 0.0,
          config.tube.cross_section.margins.tracking_error_bound + 0.01);
  ASSERT_TRUE(RefreshLegacyGuidance(command));
  ASSERT_FALSE(adapter.update(command, output));
  ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
  EXPECT_EQ(adapter.latest_v2_shadow_admission_candidate_->status,
            phase_offset_navigation::TubeExecutionStatusV2::
                TRACKING_UNAVAILABLE);
  EXPECT_FALSE(
      adapter.latest_v2_shadow_admission_candidate_->recovery_step.valid);
  EXPECT_NE(output.recovery_status,
            phase_offset_navigation::RecoveryStepStatus::PREPARED);
  EXPECT_FALSE(output.selected);
  EXPECT_EQ(adapter.recovery_owner_.status().committed_sequence,
            tracking_recovery_sequence_before);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), tracking_delta_before);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w,
                   tracking_port_before.u_w);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   tracking_port_before.u_delta);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve(), reserve);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve()->reserve_id,
            reserve->reserve_id);
  EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());

  // Drive only through real allocator-owned NORMAL commands and the existing
  // publish-first seam.  Eventually the next outward NORMAL successor can no
  // longer carry a complete common-corridor return, while the reserve already
  // committed by the preceding command remains applicable.  That exact
  // boundary must stage cursor zero of the immutable reserve; no incoming
  // selected_u field is used as authority.
  SetS5ShadowRecoveryState(reserve->initial, command);
  command.g_des = command.path.N * config.u_delta_abs_max;
  command.g_des_valid = true;
  bool recovery_prepared = false;
  for (int cycle = 0; cycle < 128 && !recovery_prepared; ++cycle) {
    ASSERT_TRUE(adapter.update(command, output)) << output.invalid_reason;
    ASSERT_TRUE(output.selected);
    ASSERT_TRUE(output.valid);
    ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
    const std::shared_ptr<const TubeV2ShadowAdmissionCandidate> staged =
        adapter.latest_v2_shadow_admission_candidate_;
    recovery_prepared = staged->recovery_step.valid;
    if (recovery_prepared) break;
    ASSERT_TRUE(staged->applicable) << staged->reason;
    EXPECT_EQ(staged->purpose, TubeWorkerPurposeV2::CURRENT);
    EXPECT_FALSE(staged->prepared_step.binding_transition);
    const phase_offset_navigation::TubeExecutionStateV2 next =
        staged->prepared_step.successor;
    const PendingPositionCommandCapture outward_pending =
        adapter.capturePendingPositionCommand();
    ASSERT_TRUE(outward_pending.pending);
    ASSERT_TRUE(outward_pending.valid);
    EXPECT_FALSE(outward_pending.v2_binding_transition);
    ASSERT_TRUE(adapter.publishPendingPositionCommand(
        []() { return true; }, outward_pending.identity));
    reserve = adapter.runtime_->committedV2Reserve();
    ASSERT_TRUE(reserve);
    ASSERT_TRUE(reserve->valid);
    SetS5ShadowRecoveryState(next, command);
    command.g_des = command.path.N * config.u_delta_abs_max;
    command.g_des_valid = true;
  }
  ASSERT_TRUE(recovery_prepared)
      << "outward NORMAL sequence never reached the certified reserve boundary";
  reserve = adapter.runtime_->committedV2Reserve();
  ASSERT_TRUE(reserve);
  ASSERT_TRUE(reserve->valid);
  ASSERT_GT(reserve->steps.size(), 1U);
  const std::uint64_t recovery_committed_sequence_before =
      adapter.recovery_owner_.status().committed_sequence;
  const double retained_delta_before = adapter.runtime_->retainedDelta();
  const phase_offset_core::PortCommand previous_port_before =
      adapter.runtime_->previousFinalPort();
  EXPECT_TRUE(output.selected);
  EXPECT_TRUE(output.valid);
  EXPECT_TRUE(output.matched.valid);
  EXPECT_TRUE(output.projection.valid);
  EXPECT_EQ(output.recovery_status,
            phase_offset_navigation::RecoveryStepStatus::PREPARED);
  EXPECT_FALSE(output.recovery_replan_required);
  ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
  const phase_offset_navigation::RecoveryPreparedStep prepared_recovery =
      adapter.latest_v2_shadow_admission_candidate_->recovery_step;
  ASSERT_TRUE(prepared_recovery.valid);
  EXPECT_EQ(prepared_recovery.proof_kind,
            phase_offset_navigation::RecoveryStepProofKind::FINITE_RESERVE_V2);
  EXPECT_EQ(prepared_recovery.reserve_id, reserve->reserve_id);
  EXPECT_EQ(prepared_recovery.reserve_cursor, reserve->cursor);
  EXPECT_EQ(prepared_recovery.reserve_size, reserve->steps.size());
  ASSERT_LT(prepared_recovery.reserve_cursor, reserve->steps.size());
  EXPECT_DOUBLE_EQ(
      prepared_recovery.selected_u.u_w,
      reserve->steps[prepared_recovery.reserve_cursor].command.u_w);
  EXPECT_DOUBLE_EQ(
      prepared_recovery.selected_u.u_delta,
      reserve->steps[prepared_recovery.reserve_cursor].command.u_delta);
  EXPECT_TRUE(prepared_recovery.selectedUExact());
  EXPECT_EQ(prepared_recovery.selected_u_owner, "PhaseOffsetRecoveryOwner");
  EXPECT_DOUBLE_EQ(output.raw_port.u_w, prepared_recovery.selected_u.u_w);
  EXPECT_DOUBLE_EQ(output.raw_port.u_delta,
                   prepared_recovery.selected_u.u_delta);
  EXPECT_DOUBLE_EQ(output.matched.w_dot,
                   adapter.latest_v2_shadow_admission_candidate_
                           ->commit_token.sealed_prepared->base_phase_rate +
                       prepared_recovery.selected_u.u_w);
  EXPECT_DOUBLE_EQ(output.matched.delta_dot,
                   prepared_recovery.selected_u.u_delta);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve(), reserve);
  EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());

  const PendingPositionCommandCapture first_pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(first_pending.pending);
  ASSERT_TRUE(first_pending.valid);
  EXPECT_NE(first_pending.identity, 0U);
  ASSERT_TRUE(first_pending.reference_query);
  EXPECT_EQ(adapter.pendingExecutedReferenceQuery(),
            first_pending.reference_query);
  bool failed_publish_called = false;
  EXPECT_FALSE(adapter.publishPendingPositionCommand(
      [&failed_publish_called]() {
        failed_publish_called = true;
        return false;
      }, first_pending.identity));
  EXPECT_TRUE(failed_publish_called);
  EXPECT_TRUE(adapter.hasPendingPositionCommand());
  EXPECT_EQ(adapter.runtime_->committedV2Reserve(), reserve);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(), retained_delta_before);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w,
                   previous_port_before.u_w);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   previous_port_before.u_delta);
  EXPECT_EQ(adapter.recovery_owner_.status().committed_sequence,
            recovery_committed_sequence_before);
  EXPECT_EQ(adapter.recovery_owner_.status().reserve_cursor, 0U);

  EXPECT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, first_pending.identity));
  EXPECT_FALSE(adapter.hasPendingPositionCommand());
  const phase_offset_navigation::TubeFiniteReserveV2* advanced_reserve =
      adapter.runtime_->committedV2Reserve();
  ASSERT_TRUE(advanced_reserve);
  EXPECT_EQ(advanced_reserve->reserve_id, prepared_recovery.reserve_id);
  EXPECT_EQ(advanced_reserve->cursor, prepared_recovery.reserve_cursor + 1U);
  EXPECT_EQ(advanced_reserve->steps.size(), prepared_recovery.reserve_size);
  EXPECT_DOUBLE_EQ(adapter.runtime_->retainedDelta(),
                   prepared_recovery.next_delta);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_w,
                   prepared_recovery.selected_u.u_w);
  EXPECT_DOUBLE_EQ(adapter.runtime_->previousFinalPort().u_delta,
                   prepared_recovery.selected_u.u_delta);
  EXPECT_EQ(adapter.recovery_owner_.status().committed_sequence,
            recovery_committed_sequence_before + 1U);
  EXPECT_EQ(adapter.recovery_owner_.status().reserve_id,
            prepared_recovery.reserve_id);
  EXPECT_EQ(adapter.recovery_owner_.status().reserve_cursor,
            prepared_recovery.reserve_cursor + 1U);
  EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());

  // A running recovery keeps the same immutable reserve and takes its next
  // cursor even when a genuinely newer coherent CURRENT completion arrives.
  // That completion may be retained for later use, but cannot replace the
  // incumbent profile or rebind the already-running recovery transaction.
  ASSERT_LT(advanced_reserve->cursor, advanced_reserve->steps.size());
  const std::size_t second_cursor = advanced_reserve->cursor;
  SetS5ShadowRecoveryState(
      advanced_reserve->steps[second_cursor].before, command);
  command.tube_v2_admission.now = 11.25;
  const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      newer_source = MakeCoherentAcceptedMapRefreshInput(
          original_source, original_source->request_id + 1U, 6U, 11U);
  ASSERT_TRUE(newer_source);
  command.tube_worker_input_v2 = newer_source;
  const std::uint64_t delivery_before_newer =
      adapter.v2_shadow_worker_->stats().delivery_count;
  ASSERT_TRUE(adapter.update(command, output));
  ASSERT_TRUE(output.selected);
  ASSERT_TRUE(adapter.latest_v2_shadow_admission_candidate_);
  const std::shared_ptr<const TubeV2ShadowAdmissionCandidate>
      second_pending_candidate =
          adapter.pending_v2_shadow_bootstrap_candidate_;
  ASSERT_TRUE(second_pending_candidate);
  const phase_offset_navigation::RecoveryPreparedStep second_recovery =
      adapter.latest_v2_shadow_admission_candidate_->recovery_step;
  ASSERT_TRUE(second_recovery.valid);
  EXPECT_EQ(second_recovery.reserve_id, prepared_recovery.reserve_id);
  EXPECT_EQ(second_recovery.reserve_cursor, second_cursor);
  EXPECT_EQ(second_recovery.reserve_size, prepared_recovery.reserve_size);
  EXPECT_DOUBLE_EQ(second_recovery.selected_u.u_w,
                   advanced_reserve->steps[second_cursor].command.u_w);
  EXPECT_DOUBLE_EQ(second_recovery.selected_u.u_delta,
                   advanced_reserve->steps[second_cursor].command.u_delta);
  EXPECT_DOUBLE_EQ(output.raw_port.u_w, second_recovery.selected_u.u_w);
  EXPECT_DOUBLE_EQ(output.raw_port.u_delta, second_recovery.selected_u.u_delta);
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter, delivery_before_newer + 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_->built());
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id,
            newer_source->request_id);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->map_capture_key,
            newer_source->map_capture_key);
  EXPECT_NE(&adapter.latest_v2_shadow_current_completion_->build.profile,
            incumbent.get());
  EXPECT_EQ(adapter.pending_v2_shadow_bootstrap_candidate_,
            second_pending_candidate);
  EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());
  EXPECT_EQ(adapter.runtime_->committedV2Reserve(), advanced_reserve);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve()->reserve_id,
            prepared_recovery.reserve_id);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve()->cursor, second_cursor);

  // Observe the newer completion on another command update.  The pending
  // exact recovery command remains the identical immutable transaction.
  ASSERT_TRUE(adapter.update(command, output));
  EXPECT_TRUE(output.selected);
  EXPECT_EQ(adapter.pending_v2_shadow_bootstrap_candidate_,
            second_pending_candidate);
  EXPECT_EQ(adapter.latest_v2_shadow_admission_candidate_,
            second_pending_candidate);
  EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());
  EXPECT_EQ(adapter.runtime_->committedV2Reserve(), advanced_reserve);
  const PendingPositionCommandCapture second_pending =
      adapter.capturePendingPositionCommand();
  ASSERT_TRUE(second_pending.pending);
  ASSERT_TRUE(second_pending.valid);
  EXPECT_TRUE(adapter.publishPendingPositionCommand(
      []() { return true; }, second_pending.identity));
  ASSERT_TRUE(adapter.runtime_->committedV2Reserve());
  EXPECT_EQ(adapter.runtime_->committedV2Reserve()->reserve_id,
            prepared_recovery.reserve_id);
  EXPECT_EQ(adapter.runtime_->committedV2Reserve()->cursor,
            second_cursor + 1U);
  EXPECT_EQ(adapter.recovery_owner_.status().reserve_id,
            prepared_recovery.reserve_id);
  EXPECT_EQ(adapter.recovery_owner_.status().reserve_cursor,
            second_cursor + 1U);
  EXPECT_EQ(adapter.recovery_owner_.status().committed_sequence,
            recovery_committed_sequence_before + 2U);
  adapter.shutdown();

  // Once the incumbent's own finite deadline is reached, the reserve cannot
  // be claimed as applicable; its immutable binding evidence remains retained
  // until an explicit neutral retirement or publish-first replacement.
  PhaseOffsetMatchedAdapter deadline_adapter(config);
  MatchedAdapterInput deadline_command;
  MatchedAdapterOutput deadline_output;
  ASSERT_TRUE(InstallS4ShadowProfileForTest(
      deadline_adapter, config, deadline_command, deadline_output));
  deadline_command.tube_worker_input_v2.reset();
  deadline_command.tube_v2_admission.now = 20.0;
  ASSERT_FALSE(deadline_adapter.update(deadline_command, deadline_output));
  EXPECT_TRUE(deadline_adapter.v2_execution_binding_);
  deadline_adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterS5C,
     AcceptedNotificationWithoutCaptureDoesNotRestartOrCommitV2) {
  PhaseOffsetMatchedAdapterConfig config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  config.observe_only = false;

  PhaseOffsetMatchedAdapter adapter(config);
  MatchedAdapterInput command;
  MatchedAdapterOutput output;
  ASSERT_TRUE(InstallS4ShadowProfileForTest(adapter, config, command, output));
  ASSERT_TRUE(adapter.v2_shadow_worker_);
  const std::uint64_t build_count =
      adapter.v2_shadow_worker_->stats().build_started;
  const std::shared_ptr<const phase_offset_navigation::TubeProfileV2>
      incumbent = InstalledV2Profile(adapter);
  ASSERT_TRUE(incumbent);
  const phase_offset_navigation::TubeFiniteReserveV2* const reserve =
      adapter.runtime_->committedV2Reserve();
  ASSERT_TRUE(reserve);
  const std::uint64_t reserve_id = reserve->reserve_id;
  const auto binding_before = adapter.v2_execution_binding_;
  const auto authority_before = adapter.execution_authority_.snapshotPtr();
  const std::shared_ptr<const phase_offset_navigation::TubeBuildInputV2>
      v2_source_before = command.tube_worker_input_v2;
  ASSERT_TRUE(v2_source_before);
  const phase_offset_navigation::TubeMapCaptureKey accepted_map_before =
      v2_source_before->map_capture_key;
  const std::uint64_t accepted_sequence_before =
      command.tube_v2_admission.latest_accepted_state_sequence;
  const std::uint64_t accepted_time_before =
      command.tube_v2_admission.latest_accepted_time_ticks;

  const auto verify_unchanged = [&]() {
    EXPECT_EQ(adapter.v2_shadow_worker_->stats().build_started, build_count);
    EXPECT_EQ(InstalledV2Profile(adapter).get(), incumbent.get());
    ASSERT_EQ(adapter.runtime_->committedV2Reserve(), reserve);
    EXPECT_EQ(adapter.runtime_->committedV2Reserve()->reserve_id, reserve_id);
    EXPECT_EQ(adapter.v2_execution_binding_, binding_before);
    EXPECT_EQ(adapter.execution_authority_.snapshotPtr(), authority_before);
    const auto request = std::atomic_load(&adapter.latest_build_request_);
    ASSERT_TRUE(request);
    EXPECT_EQ(request->tube_worker_input_v2.get(), v2_source_before.get());
    ASSERT_TRUE(request->tube_worker_input_v2);
    EXPECT_EQ(request->tube_worker_input_v2->map_capture_key,
              accepted_map_before);
    EXPECT_EQ(command.tube_v2_admission.latest_accepted_state_sequence,
              accepted_sequence_before);
    EXPECT_EQ(command.tube_v2_admission.latest_accepted_time_ticks,
              accepted_time_before);
  };

  // Raw-cloud transport is intentionally absent from MatchedAdapterInput.
  // Notification visibility without a new coherent immutable V2 capture may
  // affect only value-level admission; it cannot create another heavy proof
  // or commit Runtime/binding/authority state by itself.
  for (std::uint64_t notification = accepted_sequence_before + 1U;
       notification <= accepted_sequence_before + 2U; ++notification) {
    command.tube_v2_admission.latest_accepted_state_notification_sequence =
        notification;
    adapter.update(command, output);
    EXPECT_FALSE(adapter.scheduleTubeBuild());
    if (adapter.hasPendingPositionCommand()) {
      adapter.discardPendingPositionCommand();
    }
    verify_unchanged();
  }

  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterRefresh, ForwardWindowReplacesOldBackwardPrefix) {
  auto config = MakeManualConfig();
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  PhaseOffsetMatchedAdapter adapter(config);
  auto input = MakeShadowInput(config.tube_certificate_v2, 1U);
  TubeWorkerCompletionV2 old;
  old.status = TubeWorkerCompletionStatusV2::BUILT;
  old.build.success = true;
  old.execution_generation = input->path_key.execution_generation;
  old.request_id = 1U;
  old.path_key = input->path_key;
  old.configuration_key = input->configuration_key;
  old.map_capture_key = input->map_capture_key;
  old.accepted_state_demand = 1U;
  old.useful_start = 0.0;
  old.useful_end = 0.4;
  old.build.profile.certified_start = 0.0;
  old.build.profile.certified_end = 0.4;
  adapter.retainV2ShadowCompletionLocked(old);
  auto newer = old;
  newer.request_id = 2U;
  newer.useful_start = 0.2;
  newer.useful_end = 0.6;
  newer.build.profile.certified_start = 0.2;
  newer.build.profile.certified_end = 0.6;
  adapter.retainV2ShadowCompletionLocked(newer);
  ASSERT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 2U);
  newer.request_id = 3U;
  ++newer.accepted_state_demand;
  ++newer.map_capture_key.accepted_sequence;
  ++newer.map_capture_key.state_id;
  newer.useful_start = 0.4;
  newer.useful_end = 0.8;
  newer.build.profile.certified_start = 0.4;
  newer.build.profile.certified_end = 0.8;
  adapter.retainV2ShadowCompletionLocked(newer);
  ASSERT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 3U);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->map_capture_key, newer.map_capture_key);
  auto shorter = newer;
  shorter.request_id = 4U;
  shorter.useful_start = 0.5;
  shorter.useful_end = 0.7;
  adapter.retainV2ShadowCompletionLocked(shorter);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 3U);
  auto failed = newer;
  failed.request_id = 5U;
  failed.status = TubeWorkerCompletionStatusV2::FAILED;
  adapter.retainV2ShadowCompletionLocked(failed);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 3U);
  auto wrong_path = newer;
  wrong_path.request_id = 6U;
  ++wrong_path.path_key.path_instance_id;
  wrong_path.useful_start = 0.6;
  wrong_path.useful_end = 1.0;
  adapter.retainV2ShadowCompletionLocked(wrong_path);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 3U);
  auto live = std::make_shared<TubeBuildRequestV2>();
  live->active = true;
  live->task_generation = newer.execution_generation;
  live->current_w = 0.95;
  live->tube_worker_input_v2 = input;
  adapter.latest_build_request_ = live;
  auto after_gap = newer;
  after_gap.request_id = 7U;
  after_gap.useful_start = after_gap.build.profile.certified_start = 0.9;
  after_gap.useful_end = after_gap.build.profile.certified_end = 1.0;
  adapter.retainV2ShadowCompletionLocked(after_gap);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 7U);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterRefresh, CandidateClearsWhenProgressOrIdentityExpires) {
  auto config = MakeShadowCertificateConfig();
  auto input = MakeShadowInput(config, 1U);
  TubeWorkerCompletionV2 completion;
  completion.status = TubeWorkerCompletionStatusV2::BUILT;
  completion.build.success = true;
  completion.execution_generation = input->path_key.execution_generation;
  auto& profile = completion.build.profile;
  profile.path_key = input->path_key;
  profile.configuration_key = input->configuration_key;
  profile.map_capture_key = input->map_capture_key;
  profile.certified_start = 0.2;
  profile.certified_end = 0.8;
  profile.applicability_deadline_ticks = 100U;
  profile.applicability_deadline_timeless = false;
  TubeBuildRequestV2 request;
  request.active = true;
  request.task_generation = completion.execution_generation;
  request.tube_worker_input_v2 = input;
  const auto visible = [&]() {
    return PhaseOffsetMatchedAdapter::candidateMarkerApplicableV2(
        completion, request, completion.execution_generation, 50U);
  };
  request.current_w = 0.5;
  EXPECT_TRUE(visible());
  request.current_w = 0.8;
  EXPECT_FALSE(visible());
  request.current_w = 0.9;
  EXPECT_FALSE(visible());
  request.current_w = 0.1;
  EXPECT_FALSE(visible());
  request.current_w = 0.5;
  profile.applicability_deadline_ticks = 49U;
  EXPECT_FALSE(visible());
  profile.applicability_deadline_ticks = 100U;
  ++profile.path_key.path_instance_id;
  EXPECT_FALSE(visible());
  profile.path_key = input->path_key;
  ++profile.map_capture_key.configuration_generation;
  EXPECT_FALSE(visible());
  profile.map_capture_key = input->map_capture_key;
  request.active = false;
  EXPECT_FALSE(visible());
}

TEST(PhaseOffsetMatchedAdapterRefresh, MarkerHeartbeatReusesGeometryAndDeletesPassedTube) {
  auto config = MakeManualConfig();
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  PhaseOffsetMatchedAdapter adapter(config);
  auto input = std::make_shared<phase_offset_navigation::TubeBuildInputV2>(
      *MakeShadowInput(config.tube_certificate_v2, 1U, adapter.executionGenerationV2()));
  input->map_capture_key.support_expiry_ticks = 0U;
  input->map_capture_key.support_expiry_timeless = true;
  input->applicability_deadline_ticks = 0U;
  input->applicability_deadline_timeless = true;
  const auto original_query = input->free_ball_query;
  input->free_ball_query = [original_query](const Eigen::Vector3d& point, double radius) {
    auto result = original_query(point, radius);
    result.support_expiry_ticks = result.support.support_expiry_ticks = 0U;
    result.support_expiry_timeless = result.support.support_expiry_timeless = true;
    return result;
  };
  auto completion = std::make_shared<TubeWorkerCompletionV2>();
  phase_offset_navigation::TubeCertificateBuilderV2 builder(config.tube_certificate_v2);
  ASSERT_TRUE(builder.build(*input, completion->build));
  completion->status = TubeWorkerCompletionStatusV2::BUILT;
  completion->execution_generation = input->path_key.execution_generation;
  adapter.latest_v2_shadow_current_completion_ = completion;
  auto request = std::make_shared<TubeBuildRequestV2>();
  request->active = true;
  request->task_generation = input->path_key.execution_generation;
  request->current_w = 0.25;
  request->tube_worker_input_v2 = input;
  adapter.latest_build_request_ = request;
  adapter.publishV2TubeMarkers(ros::Time(10.0));
  ASSERT_EQ(adapter.cached_candidate_markers_v2_.markers.size(), 3U);
  const auto& line = adapter.cached_candidate_markers_v2_.markers[0];
  ASSERT_EQ(line.action, visualization_msgs::Marker::ADD);
  ASSERT_FALSE(line.points.empty());
  const auto* points = line.points.data();
  const double previous_first_x = line.points.front().x;
  const std::size_t previous_size = line.points.size();
  EXPECT_DOUBLE_EQ(line.lifetime.toSec(), 2.0 * config.tube_update_period);
  request->current_w = 0.5;
  adapter.publishV2TubeMarkers(ros::Time(10.1));
  EXPECT_EQ(adapter.cached_candidate_markers_v2_.markers[0].points.data(), points);
  EXPECT_GT(adapter.cached_candidate_markers_v2_.markers[0].points.front().x, previous_first_x);
  EXPECT_LT(adapter.cached_candidate_markers_v2_.markers[0].points.size(), previous_size);
  EXPECT_EQ(adapter.cached_candidate_markers_v2_.markers[2].points.size(),
      6U * (adapter.cached_candidate_markers_v2_.markers[0].points.size() - 1U));
  EXPECT_EQ(adapter.cached_candidate_markers_v2_.markers[0].header.stamp, ros::Time(10.1));
  const std::size_t trimmed_size = adapter.cached_candidate_markers_v2_.markers[0].points.size();
  adapter.publishV2TubeMarkers(ros::Time(10.15));
  EXPECT_EQ(adapter.cached_candidate_markers_v2_.markers[0].points.data(), points);
  EXPECT_EQ(adapter.cached_candidate_markers_v2_.markers[0].points.size(), trimmed_size);
  EXPECT_EQ(adapter.cached_candidate_markers_v2_.markers[0].header.stamp, ros::Time(10.15));
  request->current_w = completion->build.profile.certified_end;
  adapter.publishV2TubeMarkers(ros::Time(10.2));
  for (const auto& marker : adapter.cached_candidate_markers_v2_.markers) {
    EXPECT_EQ(marker.action, visualization_msgs::Marker::DELETE);
    EXPECT_TRUE(marker.points.empty());
  }
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterPinned, CurrentDemandCoalescesThroughPublish) {
  auto config = MakeManualConfig();
  config.coordination_backend = PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2 = MakeShadowCertificateConfig();
  PhaseOffsetMatchedAdapter adapter(config);
  auto command = MakeInput(MakeStraightSyntheticPath());
  PopulateS4ShadowAdmissionInput(config, command, 1U);
  auto original = command.tube_worker_input_v2;
  auto barrier = std::make_shared<ShadowCallbackBarrier>();
  auto controlled = std::make_shared<phase_offset_navigation::TubeBuildInputV2>(*original);
  auto query = controlled->path_cell_query;
  controlled->path_cell_query = [barrier, query](double a, double b, phase_offset_core::CertifiedPathCellV2& cell) {
    { std::unique_lock<std::mutex> lock(barrier->mutex); barrier->entered = true;
      barrier->condition.notify_all();
      barrier->condition.wait_for(lock, std::chrono::seconds(3), [&]{return barrier->release;}); }
    return query(a,b,cell);
  };
  command.tube_worker_input_v2 = controlled;
  MatchedAdapterOutput output;
  adapter.update(command, output);
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowCallbackEntered(barrier));
  for (std::uint64_t id = 2; id <= 8; ++id) {
    auto next = std::make_shared<phase_offset_navigation::TubeBuildInputV2>(*original);
    next->request_id = id;
    next->map_capture_key.state_id = 6U;
    next->map_capture_key.accepted_sequence = 6U;
    next->map_capture_key.accepted_time_ticks = 11U;
    command.tube_worker_input_v2 = next;
    command.tube_v2_admission.accepted_state_sequence = 6U;
    command.tube_v2_admission.accepted_state_notification_sequence = 6U;
    command.tube_v2_admission.accepted_time_ticks = 11U;
    command.tube_v2_admission.latest_accepted_state_sequence = 6U;
    command.tube_v2_admission.latest_accepted_state_notification_sequence = 6U;
    command.tube_v2_admission.latest_accepted_time_ticks = 11U;
    adapter.update(command, output);
    ASSERT_TRUE(adapter.latest_build_request_->tube_worker_input_v2);
    EXPECT_EQ(adapter.latest_build_request_->tube_worker_input_v2->request_id, 1U);
    EXPECT_FALSE(adapter.scheduleTubeBuild());
  }
  {std::lock_guard<std::mutex> lock(barrier->mutex);barrier->release=true;barrier->condition.notify_all();}
  ASSERT_TRUE(WaitShadowDelivery(adapter, 1U));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_->built());
  for(int k=0;k<=config.warmup_cycles && !adapter.pending_v2_shadow_bootstrap_candidate_;++k)
    adapter.update(command,output);
  ASSERT_TRUE(adapter.pending_v2_shadow_bootstrap_candidate_) << output.invalid_reason;
  EXPECT_EQ(adapter.pending_v2_shadow_bootstrap_candidate_->request_id,1U);
  EXPECT_EQ(adapter.latest_build_request_->tube_worker_input_v2->request_id,1U);
  ASSERT_TRUE(adapter.update(command,output));
  std::string reason;
  EXPECT_TRUE(adapter.validatePendingV2ShadowBootstrapLocked(&reason)) << reason;
  auto pending=adapter.capturePendingPositionCommand();
  ASSERT_TRUE(pending.valid);
  ASSERT_TRUE(adapter.publishPendingPositionCommand([]{return true;},pending.identity));
  EXPECT_FALSE(adapter.pinned_current_cohort_);
  adapter.update(command,output);
  EXPECT_EQ(adapter.latest_build_request_->tube_worker_input_v2->request_id,8U);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterPinned, PathReplacementRetiresAndPartialRangeRemainsSafe) {
  auto config=MakeManualConfig();
  config.coordination_backend=PhaseOffsetCoordinationBackend::DISABLED;
  config.tube_certificate_v2=MakeShadowCertificateConfig();
  PhaseOffsetMatchedAdapter adapter(config);
  auto command=MakeInput(MakeStraightSyntheticPath());
  PopulateS4ShadowAdmissionInput(config,command,1U);
  auto source=command.tube_worker_input_v2;
  MatchedAdapterOutput output;
  adapter.update(command,output);
  ASSERT_TRUE(adapter.scheduleTubeBuild());
  ASSERT_TRUE(WaitShadowDelivery(adapter,1U));
  auto complete=adapter.latest_v2_shadow_current_completion_;
  ASSERT_TRUE(complete && complete->built());
  auto partial=std::make_shared<TubeWorkerCompletionV2>(*complete);
  auto prefix=*source;prefix.requested_end=2.9;
  ASSERT_TRUE(phase_offset_navigation::TubeCertificateBuilderV2(config.tube_certificate_v2).build(prefix,partial->build));
  partial->useful_start=partial->build.profile.certified_start;
  partial->useful_end=partial->build.profile.certified_end;
  adapter.zero_gate_open_=true;
  std::shared_ptr<const TubeV2ShadowAdmissionCandidate> candidate;
  EXPECT_TRUE(adapter.prepareV2ShadowAdmission(command,source,partial,candidate));
  ASSERT_TRUE(candidate);EXPECT_TRUE(candidate->applicable)<<candidate->reason;
  command.tube_v2_admission.latest_map_instance_id = 999U;
  EXPECT_FALSE(adapter.prepareV2ShadowAdmission(command,source,partial,candidate));
  EXPECT_EQ(candidate->reason,"latest accepted map identity is not coherent");
  command.tube_v2_admission.latest_map_instance_id = source->map_capture_key.map_instance_id;
  command.tube_v2_admission.now = 21.0;
  EXPECT_FALSE(adapter.prepareV2ShadowAdmission(command,source,partial,candidate));
  EXPECT_EQ(candidate->status,phase_offset_navigation::TubeExecutionStatusV2::DEADLINE_EXPIRED);
  command.tube_v2_admission.now = 10.0;
  command.path.w=2.95;
  EXPECT_FALSE(adapter.prepareV2ShadowAdmission(command,source,partial,candidate));
  ASSERT_TRUE(candidate);
  EXPECT_NE(candidate->reason,"CURRENT completion is stale or identity-mismatched");
  partial->useful_end=3.1;
  EXPECT_FALSE(adapter.prepareV2ShadowAdmission(command,source,partial,candidate));
  EXPECT_EQ(candidate->reason,"V2 certified range is inconsistent with original source");
  auto replacement=std::make_shared<phase_offset_navigation::TubeBuildInputV2>(*source);
  replacement->request_id=8U;replacement->path_key.path_instance_id+=1U;
  command.tube_worker_input_v2=replacement;
  adapter.makeBuildRequestV2(command);
  EXPECT_FALSE(adapter.pinned_current_cohort_);
  EXPECT_FALSE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_EQ(adapter.latest_build_request_->tube_worker_input_v2,replacement);
  adapter.shutdown();
}

TEST(PhaseOffsetMatchedAdapterPinned, DefinitiveRejectionReleasesOnNextTick) {
  auto config=MakeManualConfig();config.tube_certificate_v2=MakeShadowCertificateConfig();
  PhaseOffsetMatchedAdapter adapter(config);
  auto command=MakeInput(MakeStraightSyntheticPath());
  PopulateS4ShadowAdmissionInput(config,command,1U);
  MatchedAdapterOutput output;adapter.update(command,output);
  ASSERT_TRUE(adapter.scheduleTubeBuild());ASSERT_TRUE(WaitShadowDelivery(adapter,1U));
  auto next=std::make_shared<phase_offset_navigation::TubeBuildInputV2>(*command.tube_worker_input_v2);
  next->request_id=8U;command.tube_worker_input_v2=next;
  command.tube_v2_admission.now=21.0;
  EXPECT_FALSE(adapter.update(command,output));
  ASSERT_TRUE(output.v2_shadow_admission_candidate);
  EXPECT_EQ(output.v2_shadow_admission_candidate->status,phase_offset_navigation::TubeExecutionStatusV2::DEADLINE_EXPIRED);
  EXPECT_FALSE(adapter.pinned_current_cohort_);
  EXPECT_EQ(adapter.latest_build_request_->tube_worker_input_v2->request_id,1U);
  adapter.update(command,output);
  EXPECT_EQ(adapter.latest_build_request_->tube_worker_input_v2->request_id,8U);
  adapter.shutdown();
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
