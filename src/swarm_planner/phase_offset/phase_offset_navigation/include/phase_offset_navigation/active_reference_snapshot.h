#pragma once

#include "phase_offset_navigation/immutable_executed_reference_query.h"

#include <phase_offset_core/port_types.h>

#include <Eigen/Core>

#include <cstdint>
#include <cmath>
#include <string>

namespace phase_offset_navigation {

enum class ActiveReferenceOwnerMode {
  NONE,
  NORMAL,
  COORDINATION,
  RECOVERY,
  PLANNER_ONLY,
};

struct ActiveReferenceSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t authority_session = 0U;
  std::uint64_t sequence = 0U;
  std::uint64_t planner_path_revision = 0U;
  std::uint64_t executed_path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t tube_revision = 0U;
  std::uint64_t profile_revision = 0U;
  std::uint64_t map_revision = 0U;
  std::uint64_t neighbor_revision = 0U;
  std::string obstacle_contract_id;
  ActiveReferenceOwnerMode owner_mode = ActiveReferenceOwnerMode::NONE;
  std::string selected_u_owner;
  double w = 0.0;
  double delta = 0.0;
  double dt = 0.0;
  // Previous command and selected command are separate facts.  selected_u is
  // the exact ZOH command owned by the mode owner; the authority never
  // projects or replaces it.
  phase_offset_core::PortCommand u_prev;
  phase_offset_core::PortCommand selected_u;
  double selected_u_w = 0.0;
  double selected_u_delta = 0.0;
  double proposed_next_w = 0.0;
  double proposed_next_delta = 0.0;
  phase_offset_core::PortCommand proposed_next_u_prev;
  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_ww = Eigen::Vector3d::Zero();
  bool r_ww_valid = false;
  Eigen::Vector3d matched_base_v_cmd = Eigen::Vector3d::Zero();
  double matched_base_w_dot = 0.0;
  // Stored only for successor-frame construction. Governor-facing code must
  // consume executed_reference_query and never this seed normal.
  Eigen::Vector3d executed_N = Eigen::Vector3d::Zero();
  ImmutableExecutedReferenceQueryPtr executed_reference_query;
  std::string provenance;
  std::string failure_provenance;
  std::string safety_status;
  std::string handoff_state;
  std::uint64_t reference_query_revision = 0U;
  bool current_state_unsafe = false;
  bool planner_invalid = false;
  bool stale = false;
  bool valid = false;

  std::uint64_t snapshotId() const {
    // The pair is intentionally value-derived and stable across diagnostics;
    // sequence is unique within one authority session.
    return authority_session == 0U ? sequence
        : (authority_session * 0x9e3779b97f4a7c15ULL) ^ sequence;
  }

  bool selectedUConsistent(double tolerance = 1e-12) const {
    return std::isfinite(selected_u.u_w) && std::isfinite(selected_u.u_delta) &&
        std::isfinite(selected_u_w) && std::isfinite(selected_u_delta) &&
        std::abs(selected_u.u_w - selected_u_w) <= tolerance &&
        std::abs(selected_u.u_delta - selected_u_delta) <= tolerance;
  }

  bool governorViewValid() const {
    return valid && executed_reference_query &&
        executed_reference_query->pathRevision() == executed_path_revision &&
        executed_reference_query->frameRevision() == frame_revision &&
        (reference_query_revision == 0U ||
         executed_reference_query->queryRevision() == reference_query_revision);
  }
};

}  // namespace phase_offset_navigation
