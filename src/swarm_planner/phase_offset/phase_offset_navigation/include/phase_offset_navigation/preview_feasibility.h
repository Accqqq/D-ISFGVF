#pragma once

#include "phase_offset_navigation/tube_types.h"

#include <phase_offset_core/port_types.h>

#include <cstdint>
#include <string>

namespace phase_offset_navigation {

// Preview is rate-limited reachability evidence.  It is intentionally a
// value-only API: evaluating a preview cannot install/clear an owner, mutate a
// TubeProfile, invalidate a planner path, or publish a command.
enum class PreviewStatus {
  NOT_EVALUATED,
  FEASIBLE,
  TARGET_OVERLAP,
  RATE_LIMITED,
  INFEASIBLE,
  STALE,
  DENIED,
  CURRENT_STATE_UNSAFE,
};

const char* previewStatusName(PreviewStatus status);

struct PreviewFeasibilityInput {
  const TubeProfile* profile = nullptr;
  double current_w = 0.0;
  double current_delta = 0.0;
  double target_w = 0.0;
  double target_delta = 0.0;
  double dt = 0.0;
  double horizon = 0.0;
  double max_phase_rate = 0.0;
  double max_delta_rate = 0.0;
  double max_delta_slew = 0.0;
  phase_offset_core::PortCommand previous_u;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t profile_revision = 0U;
  std::uint64_t expected_path_revision = 0U;
  std::uint64_t expected_frame_revision = 0U;
  std::uint64_t expected_profile_revision = 0U;
  bool current_state_safe = true;
  bool temporary_deny = false;
  bool successor_zero_only = false;
};

struct PreviewFeasibilityResult {
  PreviewStatus status = PreviewStatus::NOT_EVALUATED;
  bool valid = false;
  bool reachable = false;
  bool target_overlap = false;
  bool rate_limited = false;
  bool slowing_required = false;
  bool zero_only = false;
  double reachable_w_lower = 0.0;
  double reachable_w_upper = 0.0;
  double reachable_delta_lower = 0.0;
  double reachable_delta_upper = 0.0;
  double required_time = 0.0;
  double horizon_remaining = 0.0;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t profile_revision = 0U;
  std::string provenance;
  std::string reason;

  bool plannerVeto() const { return false; }
};

class PreviewFeasibility {
 public:
  static bool evaluate(const PreviewFeasibilityInput& input,
                       PreviewFeasibilityResult& output);
};

}  // namespace phase_offset_navigation
