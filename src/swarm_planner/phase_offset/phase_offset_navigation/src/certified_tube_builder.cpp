#include "phase_offset_navigation/certified_tube_builder.h"

#include <cmath>

namespace phase_offset_navigation {
namespace {

bool IsFinite(const double value) {
  return std::isfinite(value);
}

double BoundedConstructionDelta(const TubeBuilderConfig& config,
                                const double retained_delta) {
  const double rho = config.cross_section.nominal_half_width;
  if (!IsFinite(retained_delta) || !IsFinite(rho) || rho <= 0.0) return 0.0;
  return retained_delta >= -rho && retained_delta <= rho
      ? retained_delta : 0.0;
}

bool HasNonzeroCapacity(const TubeProfile& profile) {
  for (const TubeRawSample& sample : profile.samples) {
    if (!IsFinite(sample.filtered_lower) || !IsFinite(sample.filtered_upper)) {
      return false;
    }
    if (sample.filtered_lower < 0.0 || sample.filtered_upper > 0.0) {
      return true;
    }
  }
  return false;
}

bool ZeroCentrelineCoveredEvidence(const TubeProfile& profile,
                                   const TubeSurfaceValidationResult& validation) {
  if (validation.outcome != TubeSurfaceOutcome::SAFE ||
      !validation.zero_centerline_continuously_certified) return false;
  if (profile.samples.empty()) return false;
  // TubeSurfaceValidator records per-knot evidence before returning a failed
  // proof.  Treat the planner centreline as continuously covered only when
  // every retained knot has an observed, zero-containing successful witness;
  // no inward geometry is synthesized from a partial set.
  for (const TubeRawSample& sample : profile.samples) {
    bool found = false;
    for (const TubeValidatorKnotEvidence& evidence :
         profile.validator_knot_evidence) {
      if (std::abs(evidence.w - sample.w) <= 1e-8) {
        found = evidence.observed && evidence.filtered_contains_zero &&
            evidence.zero_surface_covered;
        break;
      }
    }
    if (!found) return false;
  }
  return true;
}

void PreserveSurfaceValidationEvidence(
    TubeProfile& profile, const TubeSurfaceValidationResult& validation) {
  profile.surface_cell_evidence = validation.cell_evidence;
  profile.diagnostics.surface_summary_present = true;
  profile.diagnostics.surface_outcome = validation.outcome;
  profile.diagnostics.surface_inconclusive_reason =
      validation.inconclusive_reason;
  profile.diagnostics.surface_truncation_outcome =
      validation.truncation_outcome;
  profile.diagnostics.surface_terminal_w = validation.terminal_w;
  profile.diagnostics.surface_witness_clearance = validation.witness_clearance;
  profile.diagnostics.surface_witness_clearance_exact =
      validation.witness_clearance_exact;
  profile.diagnostics.surface_midpoint_position_cover =
      validation.midpoint_position_cover;
  profile.diagnostics.surface_normal_variation_cover =
      validation.normal_variation_cover;
  profile.diagnostics.surface_delta_slope_cover = validation.delta_slope_cover;
  profile.diagnostics.surface_v_span_cover = validation.v_span_cover;
  profile.diagnostics.surface_geometric_cover = validation.geometric_cover;
  profile.diagnostics.surface_support_alignment_bound =
      validation.support_alignment_bound;
  profile.diagnostics.surface_numerical_epsilon = validation.numerical_epsilon;
  profile.diagnostics.surface_proof_residual = validation.proof_residual;
  profile.diagnostics.surface_max_depth_observed =
      validation.max_depth_observed;
  profile.diagnostics.surface_query_sample_count =
      validation.query_sample_count;
  profile.diagnostics.surface_depth_guard_reached =
      validation.depth_guard_reached;
  profile.diagnostics.surface_query_budget_reached =
      validation.query_budget_reached;
  profile.diagnostics.surface_split_w_count = validation.split_w_count;
  profile.diagnostics.surface_split_v_count = validation.split_v_count;
  profile.diagnostics.surface_split_both_count = validation.split_both_count;
  profile.diagnostics.zero_centerline_contiguous =
      validation.zero_centerline_continuously_certified;
  profile.diagnostics.zero_centerline_start_w = validation.certified_start_w;
  profile.diagnostics.zero_centerline_end_w = validation.certified_end_w;
}

TubeSurfaceValidatorConfig NormalizeValidatorConfig(
    const TubeBuilderConfig& builder_config,
    TubeSurfaceValidatorConfig validator_config) {
  // A single configured m_r governs GeometryEvaluator, cross-section, Builder,
  // and continuous validation.  Keep the validator's public config usable for
  // standalone callers, but normalize the production composition boundary.
  validator_config.minimum_reference_speed =
      builder_config.cross_section.minimum_reference_speed;
  return validator_config;
}

void PreserveValidationFailureProvenance(
    TubeProfile& profile, const TubeSurfaceValidationResult& validation) {
  profile.diagnostics.first_stop_reason =
      static_cast<int>(validation.first_failure_reason);
  profile.diagnostics.first_invalid_w = validation.first_failure_w;
}

void CollapseToPlannerZeroBaseline(
    TubeProfile& profile,
    const bool preserve_raw_environment_evidence = false) {
  bool had_nonzero_capacity = false;
  for (const TubeRawSample& sample : profile.samples) {
    had_nonzero_capacity = had_nonzero_capacity ||
        sample.filtered_lower < 0.0 || sample.filtered_upper > 0.0;
  }
  for (TubeRawSample& sample : profile.samples) {
    if (!preserve_raw_environment_evidence) {
      sample.raw_lower = 0.0;
      sample.raw_upper = 0.0;
      sample.environment_lower = 0.0;
      sample.environment_upper = 0.0;
      sample.environment_width = 0.0;
      sample.environment_interval_nonempty = true;
      sample.environment_contains_zero = true;
      sample.pre_inset_contains_zero = true;
      sample.post_inset_contains_zero = true;
      sample.filter_input_contains_zero = true;
    }
    sample.filtered_lower = 0.0;
    sample.filtered_upper = 0.0;
    sample.lower_w = 0.0;
    sample.upper_w = 0.0;
    sample.filtered_contains_zero = true;
    sample.complete = true;
  }
  if (!preserve_raw_environment_evidence) {
    profile.raw_complete = !profile.samples.empty();
  } else {
    profile.diagnostics.min_width = 0.0;
  }
  profile.filtered_complete = profile.raw_complete;
  profile.complete = profile.raw_complete;
  profile.obstacle_certified = false;
  profile.current_component_contains_delta =
      profile.current_delta_valid && profile.current_delta == 0.0;
  profile.zero_component_contains_zero = profile.current_component_contains_delta;
  profile.zero_only = profile.current_component_contains_delta &&
      !had_nonzero_capacity;
  profile.zero_centerline_continuously_certified = false;
  profile.classification = TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE;
}

}  // namespace

CertifiedTubeBuilder::CertifiedTubeBuilder(
    const TubeBuilderConfig& builder_config,
    const TubeFilterConfig& filter_config,
    const TubeSurfaceValidatorConfig& validator_config)
    : builder_(builder_config), filter_(filter_config),
      surface_validator_(NormalizeValidatorConfig(builder_config,
                                                  validator_config)),
      builder_config_(builder_config),
      validator_config_(NormalizeValidatorConfig(builder_config,
                                                 validator_config)) {}

bool CertifiedTubeBuilder::configurationValid() const {
  return builder_.configurationValid() && filter_.configurationValid() &&
      surface_validator_.configurationValid();
}

bool CertifiedTubeBuilder::configurationValidForSource(
    const TubeSource source) const {
  return builder_.configurationValidForSource(source) &&
      filter_.configurationValid() && surface_validator_.configurationValid();
}

bool CertifiedTubeBuilder::build(const CertifiedTubeBuildInput& input,
                                 CertifiedTubeBuildResult& result) const {
  result = CertifiedTubeBuildResult();
  if (!configurationValidForSource(input.source)) return false;

  TubeProfile candidate;
  const bool cloud_clearance_source = input.source == TubeSource::ESDF;
  const double construction_delta = cloud_clearance_source
      ? BoundedConstructionDelta(builder_config_, input.current_delta)
      : input.current_delta;
  const bool raw_complete = cloud_clearance_source
      ? builder_.buildCloudClearance(
            input.source, input.preview_path, input.cloud_clearance_query,
            input.path_state_query, input.path_cell_bound_query,
            input.cloud_snapshot_resolution, input.current_w,
            input.path_source_revision, input.tube_revision, candidate,
            construction_delta)
      : builder_.build(input.source, input.preview_path, input.distance_query,
                       input.path_source_revision, input.tube_revision, candidate,
                       construction_delta);
  if (cloud_clearance_source) {
    // These labels describe the exact immutable observation used by the raw
    // build.  They are provenance, never a second query or profile identity.
    candidate.snapshot_sequence = input.map_observation_sequence;
    candidate.snapshot_resolution = input.cloud_snapshot_resolution;
    candidate.snapshot_provenance_is_immutable =
        input.map_observation_is_snapshot && input.map_observation_sequence != 0U;
    candidate.map_revision = input.map_observation_sequence;
  }

  candidate.current_delta = input.current_delta;
  candidate.current_delta_valid = IsFinite(input.current_delta);
  bool filtered_complete = raw_complete &&
      filter_.filter(candidate, input.current_w, input.current_delta);
  if (filtered_complete) {
    const double rho = builder_config_.cross_section.nominal_half_width;
    const bool retained_in_nominal = IsFinite(input.current_delta) &&
        IsFinite(rho) && input.current_delta >= -rho &&
        input.current_delta <= rho;
    // TubeFilter's legacy selected-component label is intentionally refined at
    // this owner boundary: an out-of-range retained delta uses exact-zero
    // construction evidence and must never be advertised as connected.
    if (!retained_in_nominal || !candidate.current_component_contains_delta) {
      candidate.selected_component = TubeComponentSelection::ZERO_CONNECTED;
      candidate.current_component_contains_delta = false;
    } else if (input.current_delta == 0.0) {
      candidate.selected_component = TubeComponentSelection::ZERO_CONNECTED;
    } else {
      candidate.selected_component = TubeComponentSelection::CURRENT_DELTA_CONNECTED;
    }
  }
  if (filtered_complete && !HasNonzeroCapacity(candidate)) {
    CollapseToPlannerZeroBaseline(candidate, cloud_clearance_source);
  } else if (filtered_complete && cloud_clearance_source) {
    const TubeProfile full_width_profile = candidate;
    result.surface_validation_attempted = true;
    // Count every actual Validator proof callback separately from Builder's
    // cell-bound callbacks.  The wrapper has no geometry semantics; it only
    // routes the immutable callback invocation to the P1 diagnostic leaf.
    PathCellBoundQuery validator_cell_query = input.path_cell_bound_query;
    if (validator_cell_query) {
      const PathCellBoundQuery original = validator_cell_query;
      validator_cell_query = [&candidate, original](
          const double w0, const double w1,
          phase_offset_core::PathCellGeometryCertificate& certificate) {
        ++candidate.diagnostics.validator_certified_cell_bound_query_count;
        return original(w0, w1, certificate);
      };
    }
    ++candidate.diagnostics.surface_validator_invocation_count;
    const bool full_width_complete = surface_validator_.validate(
        candidate, input.current_w, input.path_state_query,
        validator_cell_query, input.cloud_clearance_query,
        input.cloud_snapshot_resolution,
        builder_config_.cross_section.planner_safe_distance,
        builder_config_.cross_section.regularity_margin,
        result.surface_validation);
    filtered_complete = full_width_complete;
    candidate.diagnostics.validator_surface_query_count =
        result.surface_validation.query_sample_count;
    candidate.diagnostics.certified_cell_bound_query_count =
        candidate.diagnostics.builder_certified_cell_bound_query_count +
        candidate.diagnostics.validator_certified_cell_bound_query_count;
    candidate.diagnostics.total_tube_construction_clearance_query_count =
        candidate.diagnostics.cross_section_directional_query_count +
        candidate.diagnostics.adaptive_refinement_centerline_query_count +
        candidate.diagnostics.adaptive_sample_base_clearance_query_count +
        candidate.diagnostics.validator_surface_query_count;
      candidate.diagnostics.total_tube_construction_query_count =
          candidate.diagnostics.total_tube_construction_clearance_query_count +
        candidate.diagnostics.certified_cell_bound_query_count;
    const std::size_t validator_cell_bound_count =
        candidate.diagnostics.validator_certified_cell_bound_query_count;
    const std::size_t validator_surface_count =
        result.surface_validation.query_sample_count;
    if (full_width_complete) {
      candidate.classification = TubeProfileClassification::OFFSET_CERTIFIED;
    } else {
      // P1 is intentionally one-pass: a failed Validator target is not a
      // signal to search alternate inward families or halve a normal scale.
      // Preserve the exact raw/proof failure provenance, then expose only the
      // frozen complete-zero fallback when the Validator itself established
      // continuous zero-centreline cover.  Otherwise leave an incomplete,
      // fail-closed Candidate for EpochManager diagnostics.
      const bool zero_centerline_covered =
          ZeroCentrelineCoveredEvidence(candidate, result.surface_validation);
      const std::vector<TubeValidatorKnotEvidence> validator_knot_evidence =
          candidate.validator_knot_evidence;
      candidate = full_width_profile;
      candidate.validator_knot_evidence = validator_knot_evidence;
      if (candidate.proof_level == TubeProofLevel::CONTINUOUS_COVER_PROOF) {
        candidate.proof_level = candidate.cell_geometry_certified
            ? TubeProofLevel::FRAME_CELL_PROOF
            : TubeProofLevel::SAMPLED_EVIDENCE;
      }
      PreserveSurfaceValidationEvidence(candidate, result.surface_validation);
      PreserveValidationFailureProvenance(candidate,
                                          result.surface_validation);
      candidate.diagnostics.validator_certified_cell_bound_query_count =
          validator_cell_bound_count;
      candidate.diagnostics.validator_surface_query_count =
          validator_surface_count;
      candidate.diagnostics.surface_validator_invocation_count = 1U;
      candidate.diagnostics.certified_cell_bound_query_count =
          candidate.diagnostics.builder_certified_cell_bound_query_count +
          validator_cell_bound_count;
      candidate.diagnostics.total_tube_construction_clearance_query_count =
          candidate.diagnostics.cross_section_directional_query_count +
          candidate.diagnostics.adaptive_refinement_centerline_query_count +
          candidate.diagnostics.adaptive_sample_base_clearance_query_count +
          validator_surface_count;
      candidate.diagnostics.total_tube_construction_query_count =
          candidate.diagnostics.total_tube_construction_clearance_query_count +
          candidate.diagnostics.certified_cell_bound_query_count;
      if (zero_centerline_covered) {
        candidate.diagnostics.invalid_reason =
            "full-width surface failed; retained complete zero planner baseline";
        CollapseToPlannerZeroBaseline(candidate, true);
        candidate.zero_centerline_continuously_certified = true;
        filtered_complete = candidate.complete;
      } else {
        candidate.diagnostics.invalid_reason =
            "bounded surface validation failed; incomplete fail-closed candidate";
        candidate.filtered_complete = false;
        candidate.complete = false;
        candidate.obstacle_certified = false;
        candidate.classification = TubeProfileClassification::NONE;
        filtered_complete = false;
      }
    }
  } else if (filtered_complete) {
    candidate.classification = HasNonzeroCapacity(candidate)
        ? TubeProfileClassification::OFFSET_CERTIFIED
        : TubeProfileClassification::ZERO_ONLY_PLANNER_BASELINE;
  }

  result.profile = candidate;
  result.raw_complete = candidate.raw_complete;
  result.filtered_complete = candidate.filtered_complete;
  result.complete = filtered_complete && candidate.complete;
  return result.complete;
}

}  // namespace phase_offset_navigation
