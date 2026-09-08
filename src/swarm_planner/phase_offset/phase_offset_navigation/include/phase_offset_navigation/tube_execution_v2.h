#pragma once

#include "phase_offset_navigation/tube_viability.h"
#include "phase_offset_navigation/tube_profile_v2.h"

#include <phase_offset_core/port_types.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace phase_offset_navigation {

// This file is a side-effect-free admission/reserve kernel.  All values are
// copied by the caller from one immutable execution binding and no method
// mutates a path, profile, Runtime, authority, or command owner.

enum class TubeExecutionStatusV2 {
  UNAVAILABLE = 0,
  ADMISSIBLE,
  INVALID_INPUT,
  PROFILE_UNAVAILABLE,
  RANGE_UNAVAILABLE,
  K_INFEASIBLE,
  COMMAND_INFEASIBLE,
  RESERVE_UNAVAILABLE,
  TRACKING_UNAVAILABLE,
  DEADLINE_EXPIRED,
  BUDGET_EXHAUSTED,
};

const char* tubeExecutionStatusName(TubeExecutionStatusV2 status);

enum class TubeReserveSegmentKindV2 {
  BRAKE = 0,
  RETURN,
  SETTLE,
};

const char* tubeReserveSegmentKindName(TubeReserveSegmentKindV2 kind);

struct TubeExecutionStateV2 {
  double w = std::numeric_limits<double>::quiet_NaN();
  double delta = std::numeric_limits<double>::quiet_NaN();
  phase_offset_core::PortCommand previous_u;

  bool finite() const;
};

struct TubeExecutionIdentityV2 {
  std::uint64_t execution_generation = 0U;
  std::uint64_t path_instance_id = 0U;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t frame_convention_id = 0U;
  std::uint64_t configuration_id = 0U;
  std::uint64_t map_instance_id = 0U;
  std::uint64_t map_state_id = 0U;
  std::uint64_t accepted_sequence = 0U;
  std::uint64_t profile_id = 0U;
  std::uint64_t binding_sequence = 0U;

  bool complete() const;
};

struct TubeExecutionLimitsV2 {
  double lower_phase_rate = 0.0;
  double upper_phase_rate = 0.0;
  double upper_nu = 0.0;
  double max_u_w = 0.0;
  double max_u_delta = 0.0;
  double u_w_slew_rate = 0.0;
  double u_delta_slew_rate = 0.0;
  double return_u_delta_max = 0.0;
  double return_u_delta_slew_rate = 0.0;
  std::size_t max_schedule_steps = 0U;
  std::size_t max_work = 0U;
  bool valid = false;

  bool complete() const;
};

struct TubeExecutionTrackingEvidenceV2 {
  bool valid = true;
  double error_norm = 0.0;
  double error_bound = 0.0;
  bool physical_tangent_valid = true;
};

struct TubeExecutionIntervalV2 {
  double w = 0.0;
  double lower = 0.0;
  double upper = 0.0;
  bool valid = false;

  bool contains(double value) const;
};

struct TubeLiveKPreviewV2 {
  bool valid = false;
  // P07/P08 own the authoritative V2 live-K partition and query arithmetic.
  // `nodes` is a value-only compatibility view used by the execution seam;
  // no second interpolation or contraction proof is stored here.
  TubeViabilityResult viability;
  std::vector<TubeExecutionIntervalV2> nodes;
  double min_width = std::numeric_limits<double>::quiet_NaN();
  std::size_t work_count = 0U;
  std::string reason;
};

struct TubeReserveStepV2 {
  std::size_t ordinal = 0U;
  TubeReserveSegmentKindV2 segment = TubeReserveSegmentKindV2::BRAKE;
  TubeExecutionStateV2 before;
  TubeExecutionStateV2 after;
  phase_offset_core::PortCommand command;
  double base_phase_rate = 0.0;
  double phase_rate_lower = 0.0;
  double phase_rate_upper = 0.0;
  bool valid = false;
};

struct TubeFiniteReserveV2 {
  bool valid = false;
  std::uint64_t reserve_id = 0U;
  TubeExecutionIdentityV2 identity;
  TubeExecutionStateV2 initial;
  TubeExecutionStateV2 terminal;
  std::vector<TubeReserveStepV2> steps;
  double dt = 0.0;
  double w_max = std::numeric_limits<double>::quiet_NaN();
  double common_lower = std::numeric_limits<double>::quiet_NaN();
  double common_upper = std::numeric_limits<double>::quiet_NaN();
  std::size_t cursor = 0U;
  std::size_t work_count = 0U;
  std::string provenance;
  std::string reason;

  bool terminalExact() const;
};

struct TubeExecutionReserveInputV2 {
  const TubeProfileV2* profile = nullptr;
  TubeExecutionIdentityV2 identity;
  TubeExecutionStateV2 initial;
  double base_phase_rate = 0.0;
  double phase_rate_lower = 0.0;
  double phase_rate_upper = 0.0;
  double approved_upper_nu = 0.0;
  double dt = 0.0;
  double now = 0.0;
  double deadline = std::numeric_limits<double>::quiet_NaN();
  bool deadline_valid = false;
  TubeExecutionLimitsV2 limits;
  std::size_t max_schedule_steps = 0U;
  std::size_t max_work = 0U;
  std::string provenance;
};

struct TubeExecutionAdmissionInputV2 {
  const TubeProfileV2* profile = nullptr;
  TubeExecutionIdentityV2 identity;
  TubeExecutionStateV2 current;
  phase_offset_core::PortCommand selected_u;
  std::string selected_u_owner = "PhaseOffsetAllocator";
  double base_phase_rate = 0.0;
  double phase_rate_lower = 0.0;
  double phase_rate_upper = 0.0;
  double horizon_w = std::numeric_limits<double>::quiet_NaN();
  // Positive spacing for the deterministic live-K preview partition.  The
  // partition merges these uniform samples with every immutable I knot.
  double sample_spacing_w = std::numeric_limits<double>::quiet_NaN();
  // Captured immutable P07/P08 normal-preview policy.  P10 does not invent
  // beta thresholds or a second live-K policy.
  NormalPreviewProductionPolicy preview_policy;
  // Optional precomputed P07/P08 value.  When supplied, P10 verifies exact
  // profile/policy/work identity and never re-evaluates the preview; the
  // returned P07/P08 endpoint remains the sole window authority.
  const TubeViabilityResult* preview_result = nullptr;
  double upper_u_delta = 0.0;
  double dt = 0.0;
  double now = 0.0;
  double applicability_deadline = std::numeric_limits<double>::quiet_NaN();
  bool applicability_deadline_valid = false;
  TubeExecutionLimitsV2 limits;
  TubeExecutionTrackingEvidenceV2 tracking;
  std::size_t max_work = 0U;
  std::string provenance;
};

struct TubeStepAdmissionV2 {
  bool valid = false;
  TubeExecutionStatusV2 status = TubeExecutionStatusV2::UNAVAILABLE;
  TubeExecutionIdentityV2 identity;
  TubeExecutionStateV2 current;
  TubeExecutionStateV2 successor;
  TubeLiveKPreviewV2 live_k;
  TubeFiniteReserveV2 successor_reserve;
  std::size_t crossed_breakpoint_count = 0U;
  std::size_t work_count = 0U;
  std::string reason;
  std::string provenance;
};

class TubeExecutionGuardV2 {
 public:
  // Build the moving-window K and validate one actual selected ZOH command.
  // The exact successor's complete finite reserve is built before returning
  // ADMISSIBLE.  No method here changes authoritative state or publishes.
  static bool prepareAdmission(const TubeExecutionAdmissionInputV2& input,
                               TubeStepAdmissionV2& output);
  static bool admit(const TubeExecutionAdmissionInputV2& input,
                    TubeStepAdmissionV2& output) {
    return prepareAdmission(input, output);
  }
  static bool evaluate(const TubeExecutionAdmissionInputV2& input,
                       TubeStepAdmissionV2& output) {
    return prepareAdmission(input, output);
  }

  static bool prepareReserve(const TubeExecutionReserveInputV2& input,
                             TubeFiniteReserveV2& output);
  static bool buildReserve(const TubeExecutionReserveInputV2& input,
                           TubeFiniteReserveV2& output) {
    return prepareReserve(input, output);
  }

  static bool validateReserve(const TubeFiniteReserveV2& reserve,
                              std::string* reason = nullptr);
};

using TubeExecutionBindingV2 = TubeExecutionIdentityV2;
using TubeExecutionStepInputV2 = TubeExecutionAdmissionInputV2;
using TubeExecutionStepOutputV2 = TubeStepAdmissionV2;
using TubeReserveInputV2 = TubeExecutionReserveInputV2;
using TubeReserveV2 = TubeFiniteReserveV2;

}  // namespace phase_offset_navigation
