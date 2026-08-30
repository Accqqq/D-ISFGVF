#pragma once

#include "phase_offset_navigation/tube_types.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace phase_offset_navigation {

// The existing TubeProfile is the immutable geometric profile consumed by the
// C1 value core.  This alias keeps the NORMAL Preview contract explicit without
// adding a second mutable geometry representation.
using GeometricTubeProfile = TubeProfile;

enum class TubeViabilityStatus {
  NOT_EVALUATED,
  FEASIBLE,
  RATE_INFEASIBLE,
  PREVIEW_INFEASIBLE,
  CURRENT_DELTA_OUTSIDE,
  STALE,
  INVALID_INPUT,
};

// Immutable production NORMAL Preview policy.  The policy is deliberately a
// value type with invalid (NaN/zero identity) construction defaults: a caller
// must capture a complete configuration before a production preview can run.
// All distances are in the native phase coordinate w; no alternate
// coordinate conversion is part of this contract.
struct NormalPreviewProductionPolicy {
  double preview_horizon_w = std::numeric_limits<double>::quiet_NaN();
  double sample_spacing_w = std::numeric_limits<double>::quiet_NaN();
  double lower_nu = std::numeric_limits<double>::quiet_NaN();
  double upper_nu = std::numeric_limits<double>::quiet_NaN();
  double b_tight = std::numeric_limits<double>::quiet_NaN();
  double b_open = std::numeric_limits<double>::quiet_NaN();

  // Stable identity captured with the immutable value.  The numerical
  // revision and configuration identity are intentionally separate fields so
  // a policy can be traced without introducing a lifecycle/session owner.
  std::uint64_t policy_revision = 0U;
  std::uint64_t configuration_identity = 0U;
  std::string configuration_id;
  bool immutable = true;

  bool valid() const;
};

const char* tubeViabilityStatusName(TubeViabilityStatus status);

struct TubeViabilityInterval {
  double lower = 0.0;
  double upper = 0.0;
  bool valid = false;

  double width() const { return upper - lower; }
  bool contains(double value, double tolerance = 1e-10) const {
    return valid && value >= lower - tolerance && value <= upper + tolerance;
  }
};

// Optional value-only source section.  It is useful for deterministic core
// tests and does not carry map, ROS, neighbour or controller state.
struct TubeViabilityCrossSection {
  double w = 0.0;
  double lower = 0.0;
  double upper = 0.0;
};

struct TubeViabilityProvenance {
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t profile_revision = 0U;
  std::uint64_t source_revision = 0U;
  std::uint64_t tube_revision = 0U;
  std::uint64_t map_revision = 0U;
  std::string obstacle_contract_id;
  std::uint64_t policy_revision = 0U;
  std::uint64_t policy_configuration_identity = 0U;
  std::string policy_configuration_id;
  std::string source = "phase_offset_navigation/tube_viability/normal-v1";
  bool immutable = true;
};

struct TubeViabilityRateInterval {
  double lower = 0.0;
  double upper = 0.0;
  bool valid = false;
  bool lower_boundary_active = false;
  bool upper_boundary_active = false;
  double lower_boundary_slope = 0.0;
  double upper_boundary_slope = 0.0;
};

struct TubeViabilityKnot {
  // Native phase-coordinate knot.  `w` is the coordinate used for planning,
  // execution and all C1 queries.
  double w = 0.0;
  TubeViabilityInterval geometric;
  TubeViabilityInterval reachable;

  // Exact PWL one-sided slopes.  A missing side is marked by the matching
  // *_slope_*_valid bit; no smoothing or extrapolation is performed.
  double lower_slope_left = 0.0;
  double lower_slope_right = 0.0;
  double upper_slope_left = 0.0;
  double upper_slope_right = 0.0;
  bool lower_slope_left_valid = false;
  bool lower_slope_right_valid = false;
  bool upper_slope_left_valid = false;
  bool upper_slope_right_valid = false;

  TubeViabilityRateInterval lower_boundary_rate;
  TubeViabilityRateInterval upper_boundary_rate;
};

struct TubeViabilityInput {
  // Production callers bind an immutable geometric profile.  The optional
  // cross_sections vector is a pure value-only source for standalone tests;
  // when present it takes precedence over profile samples.
  const GeometricTubeProfile* profile = nullptr;
  std::vector<TubeViabilityCrossSection> cross_sections;

  double current_w = 0.0;
  double current_delta = 0.0;

  // Required immutable W-domain policy.  A value-only cross-section fixture
  // may supply the same policy, but production callers must also bind an
  // immutable GeometricTubeProfile and its revisions below.
  NormalPreviewProductionPolicy policy;
  double upper_u_delta = 0.0;
  double boundary_tolerance = 1e-10;

  // Expected immutable revisions.  Zero means that the corresponding
  // expectation is intentionally unbound (legacy synthetic tests only).
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t profile_revision = 0U;
  std::uint64_t expected_path_revision = 0U;
  std::uint64_t expected_frame_revision = 0U;
  std::uint64_t expected_profile_revision = 0U;
};

struct TubeViabilityResult {
  TubeViabilityStatus status = TubeViabilityStatus::NOT_EVALUATED;
  bool valid = false;
  bool feasible = false;
  bool rate_feasible = false;
  bool contraction_rate_feasible = false;
  bool current_w_inside = false;
  bool current_delta_inside = false;
  bool preview_truncated_after = false;

  double current_w = 0.0;
  double preview_start_w = 0.0;
  double preview_end_w = 0.0;
  double effective_horizon_w = 0.0;
  double delta_w = 0.0;
  double delta_delta = 0.0;
  TubeViabilityInterval delta_reach;
  double upper_u_delta = 0.0;
  NormalPreviewProductionPolicy policy;

  double b_pre = 0.0;
  double beta_argument = 0.0;
  double beta = 0.0;
  // Explicit upstream-facing spelling retained alongside beta for the C2
  // handoff contract.
  double beta_i = 0.0;
  double max_inward_contraction_slope = 0.0;
  double allowed_inward_contraction_slope = 0.0;

  TubeViabilityRateInterval current_rate_interval;
  TubeViabilityProvenance provenance;
  std::vector<TubeViabilityKnot> knots;
  std::string reason;

  // Values are the exact piecewise-linear inner envelope.  At a knot the
  // right segment is selected when available; the final knot uses its left
  // one-sided segment.  These helpers are observational and side-effect free.
  bool envelopeAt(double w, TubeViabilityInterval& interval) const;
  bool rateIntervalAt(double w, double delta,
                      TubeViabilityRateInterval& interval) const;
  bool rateIntervalAt(double w, double delta, double phase_rate,
                      TubeViabilityRateInterval& interval) const;
};

class TubeViability {
 public:
  // Returns true when the immutable value input was evaluated.  A valid result
  // may still report PREVIEW_INFEASIBLE, RATE_INFEASIBLE or
  // CURRENT_DELTA_OUTSIDE; those statuses never invalidate the planner path.
  static bool evaluate(const TubeViabilityInput& input,
                       TubeViabilityResult& output);

  // Convenience overload for an explicit value-only cross-section sequence.
  static bool evaluate(
      const std::vector<TubeViabilityCrossSection>& cross_sections,
      const TubeViabilityInput& input,
      TubeViabilityResult& output);

  static double smoothstep(double q);

  static bool queryEnvelope(const TubeViabilityResult& result,
                            double w, TubeViabilityInterval& interval);
  static bool queryRateInterval(const TubeViabilityResult& result,
                                double w, double delta,
                                TubeViabilityRateInterval& interval);
  static bool queryRateInterval(const TubeViabilityResult& result,
                                double w, double delta, double phase_rate,
                                TubeViabilityRateInterval& interval);
};

using NormalPreviewInput = TubeViabilityInput;
using NormalPreviewResult = TubeViabilityResult;
using NormalPreview = TubeViability;

double tubeViabilitySmoothstep(double q);

}  // namespace phase_offset_navigation
