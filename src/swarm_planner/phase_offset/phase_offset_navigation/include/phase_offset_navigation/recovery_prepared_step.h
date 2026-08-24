#pragma once

#include "phase_offset_navigation/recovery_reference.h"

#include <phase_offset_core/port_types.h>

#include <cstdint>
#include <cmath>
#include <memory>
#include <string>

namespace phase_offset_navigation {

struct RecoveryOwnerStatus;

enum class RecoveryStepStatus {
  NONE,
  PREPARED,
  RECOVERY_REPLAN_REQUIRED,
  DEADLINE_EXPIRED,
  STALE,
  CURRENT_STATE_UNSAFE,
  REFERENCE_MISMATCH,
  INVALID,
};

const char* recoveryStepStatusName(RecoveryStepStatus status);

struct RecoveryPreparedStep {
  std::uint64_t recovery_session = 0U;
  std::uint64_t source_path_revision = 0U;
  std::uint64_t target_path_revision = 0U;
  std::uint64_t source_frame_revision = 0U;
  std::uint64_t target_frame_revision = 0U;
  std::uint64_t source_owner_revision = 0U;
  std::uint64_t target_owner_revision = 0U;
  RecoveryReferenceJet reference_jet;

  // selected_u is the actual ZOH command for this step.  The scalar aliases
  // are kept explicit to make ownership auditable in diagnostics and tests.
  phase_offset_core::PortCommand selected_u;
  double selected_u_w = 0.0;
  double selected_u_delta = 0.0;
  std::string selected_u_owner;
  phase_offset_core::PortCommand u_prev;

  double u_w_lower = 0.0;
  double u_w_upper = 0.0;
  double u_delta_lower = 0.0;
  double u_delta_upper = 0.0;
  double current_w = 0.0;
  double current_delta = 0.0;
  double next_w = 0.0;
  double next_delta = 0.0;
  double dt = 0.0;
  double measure = 0.0;
  double next_measure_upper_bound = 0.0;
  double progress = 0.0;
  double required_progress = 0.0;
  double deadline = 0.0;
  double now = 0.0;
  double remaining_domain_duration = 0.0;
  bool deadline_valid = false;
  bool terminal_predicate = false;
  // Exact selected-ZOH arrival proof.  This is deliberately distinct from
  // tolerance-based diagnostics: neutral handoff may consume only a proof
  // that the recurrence arrived at the immutable target exactly.
  bool exact_terminal_predicate = false;
  bool proof_valid = false;
  bool valid = false;
  bool owner_committed = false;
  RecoveryStepStatus status = RecoveryStepStatus::NONE;
  std::string proof;
  std::string provenance;
  // Fully materialized status for the post-publication no-fail seam.  The
  // owner prepares this before publication; commitNoFail only swaps it.
  std::shared_ptr<const RecoveryOwnerStatus> committed_status;

  bool selectedUConsistent(double tolerance = 1e-12) const {
    return std::abs(selected_u.u_w - selected_u_w) <= tolerance &&
        std::abs(selected_u.u_delta - selected_u_delta) <= tolerance;
  }

  bool finite() const {
    return std::isfinite(selected_u.u_w) &&
        std::isfinite(selected_u.u_delta) &&
        std::isfinite(selected_u_w) && std::isfinite(selected_u_delta) &&
        std::isfinite(current_w) && std::isfinite(current_delta) &&
        std::isfinite(next_w) && std::isfinite(next_delta) &&
        std::isfinite(dt) && dt > 0.0 && std::isfinite(measure) &&
        std::isfinite(next_measure_upper_bound) &&
        std::isfinite(progress) && std::isfinite(required_progress) &&
        std::isfinite(deadline) && std::isfinite(now) &&
        std::isfinite(remaining_domain_duration) &&
        std::isfinite(u_w_lower) && std::isfinite(u_w_upper) &&
        std::isfinite(u_delta_lower) && std::isfinite(u_delta_upper) &&
        u_w_lower <= u_w_upper && u_delta_lower <= u_delta_upper &&
        reference_jet.valid;
  }
};

}  // namespace phase_offset_navigation
