#pragma once

#include "phase_offset_navigation/distance_query.h"
#include "phase_offset_navigation/path_state_query.h"
#include "phase_offset_navigation/tube_types.h"

#include <cstddef>
#include <vector>

namespace phase_offset_navigation {

// Fixed implementation limits, not launch parameters.  A limit exhaustion is
// a failed certificate, never a reason to loosen the clearance predicate.
struct TubeSurfaceValidatorConfig {
  int max_subdivision_depth = 12;
  std::size_t max_query_samples = 250000U;
};

struct TubeSurfaceValidationResult {
  bool complete = false;
  bool current_anchor_valid = false;
  bool truncated_before = false;
  bool truncated_after = false;
  bool limit_exceeded = false;
  std::size_t query_sample_count = 0U;
  // In-memory proof-capacity evidence.  query_sample_count remains the exact
  // number of underlying ClearanceQuery calls; these counters describe only
  // geometry/scheduling work in this invocation.
  std::size_t geometry_cell_count = 0U;
  std::size_t clearance_leaf_cell_count = 0U;
  std::size_t prequery_cover_split_count = 0U;
  int max_depth_observed = 0;
  std::size_t split_w_count = 0U;
  std::size_t split_v_count = 0U;
  std::size_t split_both_count = 0U;
  std::size_t anisotropic_split_count = 0U;
  // Cover accounting is exposed as evidence for the clearance audit.  It does
  // not alter validation or Runtime semantics.
  bool cover_accounting_observed = false;
  double min_cover_radius = 0.0;
  double max_cover_radius = 0.0;
  double max_requested_clearance = 0.0;
  // Per-knot evidence from this validator invocation.  It lets a caller
  // correlate Builder pre/post-inset facts with the actual continuous proof
  // without reconstructing coverage from a diagnostics payload.
  std::vector<TubeValidatorKnotEvidence> knot_evidence;
  bool zero_centerline_continuously_certified = false;
  double certified_start_w = 0.0;
  double certified_end_w = 0.0;
  double min_clearance_margin = 0.0;
  TubeStopReason first_failure_reason = TubeStopReason::NONE;
  double first_failure_w = 0.0;
};

// Validates the complete filtered ribbon surface with a conservative
// Euclidean-Lipschitz cover.  It only changes Candidate geometry (by trimming
// an unsafe remote suffix/prefix); it has no active-tube, Runtime, or command
// ownership.
class TubeSurfaceValidator {
 public:
  explicit TubeSurfaceValidator(
      const TubeSurfaceValidatorConfig& config = TubeSurfaceValidatorConfig());

  bool configurationValid() const;
  bool validate(TubeProfile& profile,
                double current_w,
                const PathStateQuery& path_state_query,
                const ClearanceQuery& clearance_query,
                double snapshot_resolution,
                double required_clearance,
                double regularity_margin,
                TubeSurfaceValidationResult& result) const;

  // Certified-cell variant.  A missing/invalid callback preserves the
  // established fixed-inset sampled cover; it is never treated as a proof.
  bool validate(TubeProfile& profile,
                double current_w,
                const PathStateQuery& path_state_query,
                const PathCellBoundQuery& path_cell_bound_query,
                const ClearanceQuery& clearance_query,
                double snapshot_resolution,
                double required_clearance,
                double regularity_margin,
                TubeSurfaceValidationResult& result) const;

 private:
  TubeSurfaceValidatorConfig config_;
};

}  // namespace phase_offset_navigation
