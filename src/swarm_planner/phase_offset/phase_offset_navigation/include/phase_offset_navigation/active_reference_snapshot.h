#pragma once

#include "phase_offset_navigation/immutable_executed_reference_query.h"

#include <phase_offset_core/port_types.h>

#include <Eigen/Core>

#include <cstdint>
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
  phase_offset_core::PortCommand selected_u;
  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_ww = Eigen::Vector3d::Zero();
  // Stored only for successor-frame construction. Governor-facing code must
  // consume executed_reference_query and never this seed normal.
  Eigen::Vector3d executed_N = Eigen::Vector3d::Zero();
  ImmutableExecutedReferenceQueryPtr executed_reference_query;
  std::string provenance;
  bool valid = false;

  bool governorViewValid() const {
    return valid && executed_reference_query &&
        executed_reference_query->pathRevision() == executed_path_revision &&
        executed_reference_query->frameRevision() == frame_revision;
  }
};

}  // namespace phase_offset_navigation
