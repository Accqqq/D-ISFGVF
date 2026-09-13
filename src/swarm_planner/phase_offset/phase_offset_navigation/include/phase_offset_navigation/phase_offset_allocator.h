#pragma once

#include "phase_offset_navigation/tube_viability.h"

#include <phase_offset_core/geometry_types.h>
#include <phase_offset_core/port_types.h>

#include <Eigen/Core>

#include <cstdint>
#include <string>

namespace phase_offset_navigation {

// Failure/selection status for the value-only NORMAL/COORDINATION allocator.
// A failure is deliberately a value result; it does not install an owner,
// mutate Runtime state or invalidate the planner path.
enum class PhaseOffsetAllocatorStatus {
  NOT_EVALUATED,
  SELECTED,
  INVALID_INPUT,
  STALE_PREVIEW,
  PREVIEW_INFEASIBLE,
  NO_ADMISSIBLE_COMMAND,
  ALLOCATOR_INFEASIBLE = NO_ADMISSIBLE_COMMAND,
  STALE = STALE_PREVIEW,
};

const char* phaseOffsetAllocatorStatusName(
    PhaseOffsetAllocatorStatus status);

// Frozen scalar limits.  Bounds are applied independently to u_w and
// u_delta.  The phase-rate envelope, scalar amplitude bounds and slew limits
// are explicit.  zoh_min_dt/zoh_max_dt are optional duration checks (zero
// means unbound); zoh_dt, when nonzero, requests the exact tick duration.
struct PhaseOffsetAllocatorBounds {
  double lower_nu = 0.0;
  double upper_nu = 0.0;

  // Literal phase-offset amplitude bound: |u_w| <= u_w_abs_max.  Zero
  // permits only u_w == 0; it never denotes an unbounded amplitude.
  double u_w_abs_max = 0.0;
  double upper_u_delta = 0.0;

  double u_w_slew_rate = 0.0;
  double u_delta_slew_rate = 0.0;

  double zoh_min_dt = 0.0;
  double zoh_max_dt = 0.0;
  double zoh_dt = 0.0;

  // Minimum physical tangential speed of the realised command.  The port
  // projector already carries this as a constraint on w_lower
  // (port_projector.cpp: "w_lower = max(w_lower, (tangent_speed_min -
  // base_tangent_speed) / r_w_norm)"); the allocator owns the phase window and
  // the amplitude, so it must carry the same lower bound or it can select a
  // command that the runtime's tangential-speed audit then rejects, turning a
  // reachable tick into a HOLD.  Zero disables the constraint (unchanged
  // behaviour for every caller that does not set it).
  double tangent_speed_min = 0.0;
};

struct PhaseOffsetAllocatorInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // The geometry and Preview are caller-owned immutable facts for this tick.
  phase_offset_core::PhaseOffsetGeometryState geometry;
  const NormalPreviewResult* preview = nullptr;
  Eigen::Vector3d g_des = Eigen::Vector3d::Zero();

  // f_w0 is the nominal phase rate before the selected port is added.
  double f_w0 = 0.0;
  // Tangential component of the base (pre-port) command, i.e. T . base_v_cmd.
  // Used with bounds.tangent_speed_min to keep the selected u_w from spending
  // the tangential budget below its floor.  Zero means "not provided", which
  // leaves the tangential constraint inactive together with a zero minimum.
  double base_tangent_speed = 0.0;
  phase_offset_core::PortCommand previous_u;
  double dt = 0.0;
  PhaseOffsetAllocatorBounds bounds;

  // Legacy/V2 production allocation requires every expected revision below to
  // be nonzero and the obstacle contract ID to be nonempty.  Each expectation
  // must exactly match immutable Normal Preview provenance; zero/empty is an
  // unbound expectation and is rejected rather than disabling validation.  A
  // SECTION_PWL preview dispatches separately and uses only its profile pointer
  // plus path/frame and geometry-state association.
  std::uint64_t expected_path_revision = 0U;
  std::uint64_t expected_frame_revision = 0U;
  std::uint64_t expected_profile_revision = 0U;
  std::uint64_t expected_source_revision = 0U;
  std::uint64_t expected_tube_revision = 0U;
  std::uint64_t expected_map_revision = 0U;
  std::string expected_obstacle_contract_id;

  // SECTION_PWL previews borrow the exact immutable SectionTubeProfile rather
  // than carrying legacy map/source/tube identities.  The allocator requires
  // this pointer to match preview.section_profile for the current tick.
  const SectionTubeProfile* expected_section_profile = nullptr;
};

struct PhaseOffsetScalarSelection {
  double nominal = 0.0;
  double pre_slew = 0.0;
  double selected = 0.0;
  double lower = 0.0;
  double upper = 0.0;
  double slew_lower = 0.0;
  double slew_upper = 0.0;
  double selected_lower = 0.0;
  double selected_upper = 0.0;
  bool amplitude_limited = false;
  bool envelope_limited = false;
  bool slew_limited = false;
  bool valid = false;
};

struct PhaseOffsetAllocatorResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PhaseOffsetAllocatorStatus status =
      PhaseOffsetAllocatorStatus::NOT_EVALUATED;
  bool valid = false;
  bool feasible = false;
  bool preview_bound = false;
  bool piecewise_constant = false;
  // Set when the requested motion could not fit the admissible interval and
  // the command was clipped onto that interval's boundary instead of being
  // rejected.  The corresponding hard bound in FinalCommandValid is then the
  // bound that was provably unreachable for this tick, so it is not re-applied;
  // every other bound still is.
  bool phase_window_clipped = false;
  bool transverse_interval_clipped = false;

  double u_w_nom = 0.0;
  double u_delta_nom = 0.0;
  double phase_rate_nom = 0.0;
  double phase_rate_selected = 0.0;
  double zoh_dt = 0.0;

  PhaseOffsetScalarSelection u_w;
  PhaseOffsetScalarSelection u_delta;
  TubeViabilityRateInterval preview_rate_interval;

  // selected_u is the exact command to hold for this tick.  The aliases are
  // value copies for diagnostics and selected-u ownership audits.
  phase_offset_core::PortCommand selected_u;
  phase_offset_core::PortCommand next_u_prev;
  double selected_u_w = 0.0;
  double selected_u_delta = 0.0;
  std::string selected_u_owner;

  TubeViabilityProvenance preview_provenance;
  std::string provenance;
  std::string reason;

  bool selectedUConsistent(double tolerance = 1e-12) const;
};

class PhaseOffsetAllocator {
 public:
  static const char* ownerName();

  // Side-effect-free value/selection entry point.
  static bool allocate(const PhaseOffsetAllocatorInput& input,
                       PhaseOffsetAllocatorResult& output);

  // Naming aliases keep the value kernel convenient at call sites while all
  // aliases execute the same one-pass scalar algorithm.
  static bool evaluate(const PhaseOffsetAllocatorInput& input,
                       PhaseOffsetAllocatorResult& output);
  static bool select(const PhaseOffsetAllocatorInput& input,
                     PhaseOffsetAllocatorResult& output);
  static bool solve(const PhaseOffsetAllocatorInput& input,
                    PhaseOffsetAllocatorResult& output);

  static PhaseOffsetAllocatorResult allocate(
      const PhaseOffsetAllocatorInput& input);

  static bool analyticProjection(
      const phase_offset_core::PhaseOffsetGeometryState& geometry,
      const Eigen::Vector3d& g_des, double& u_w_nom, double& u_delta_nom);

  static Eigen::Vector2d analyticRawPort(
      const phase_offset_core::PhaseOffsetGeometryState& geometry,
      const Eigen::Vector3d& g_des);
};

using NoQpPhaseOffsetAllocator = PhaseOffsetAllocator;
using PhaseOffsetAllocatorLimits = PhaseOffsetAllocatorBounds;
using PhaseOffsetAllocatorOutput = PhaseOffsetAllocatorResult;
using AllocatorInput = PhaseOffsetAllocatorInput;
using AllocatorOutput = PhaseOffsetAllocatorResult;
using AllocatorBounds = PhaseOffsetAllocatorBounds;

}  // namespace phase_offset_navigation
