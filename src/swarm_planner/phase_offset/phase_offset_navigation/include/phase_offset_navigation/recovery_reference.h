#pragma once

#include "phase_offset_navigation/immutable_executed_reference_query.h"

#include <phase_offset_core/port_types.h>

#include <Eigen/Core>

#include <cstdint>
#include <cmath>
#include <memory>
#include <string>

namespace phase_offset_navigation {

struct RecoveryReferenceJet {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w = 0.0;
  double delta = 0.0;
  Eigen::Vector3d r = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_ww = Eigen::Vector3d::Zero();
  bool r_ww_valid = false;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t owner_revision = 0U;
  std::uint64_t query_revision = 0U;
  std::string provenance;
  bool valid = false;

  bool matches(const RecoveryReferenceJet& other,
               bool require_full_jet = false) const {
    return valid && other.valid && path_revision == other.path_revision &&
        frame_revision == other.frame_revision &&
        owner_revision == other.owner_revision &&
        query_revision == other.query_revision &&
        (r - other.r).norm() <= 1e-10 &&
        (r_w - other.r_w).norm() <= 1e-10 &&
        (!require_full_jet || (r_ww_valid && other.r_ww_valid &&
                               (r_ww - other.r_ww).norm() <= 1e-10)) &&
        (require_full_jet || !r_ww_valid || !other.r_ww_valid ||
         (r_ww - other.r_ww).norm() <= 1e-10);
  }
};

using RecoveryReferenceJetPtr = std::shared_ptr<const RecoveryReferenceJet>;

struct RecoveryReference {
  RecoveryReferenceJet source;
  RecoveryReferenceJet target;
  ImmutableExecutedReferenceQueryPtr source_query;
  ImmutableExecutedReferenceQueryPtr target_query;
  std::uint64_t recovery_session = 0U;
  bool source_target_continuous = false;
  bool valid = false;
  std::string provenance;

  bool validJetContract() const {
    return valid && source.valid && target.valid &&
        source_query && target_query && source_query->pathRevision() ==
            source.path_revision && target_query->pathRevision() ==
            target.path_revision;
  }
};

inline bool makeRecoveryReferenceJet(const ImmutableExecutedReferenceQuery& query,
                                     double w, double delta,
                                     RecoveryReferenceJet& jet) {
  jet = RecoveryReferenceJet();
  ExecutedReferenceQueryResult result;
  if (!query.query(w, result) || !result.valid || !std::isfinite(delta) ||
      !result.r.allFinite() || !result.r_w.allFinite()) {
    return false;
  }
  jet.w = w;
  jet.delta = delta;
  jet.r = result.r;
  jet.r_w = result.r_w;
  jet.r_ww = result.r_ww;
  jet.r_ww_valid = result.r_ww_valid && result.r_ww.allFinite();
  jet.path_revision = result.path_revision;
  jet.frame_revision = result.frame_revision;
  jet.owner_revision = result.owner_revision;
  jet.query_revision = result.query_revision;
  jet.provenance = result.provenance;
  jet.valid = true;
  return true;
}

}  // namespace phase_offset_navigation
