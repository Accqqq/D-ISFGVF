#include "phase_offset_navigation/certified_tube_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace phase_offset_navigation {
namespace {

constexpr double kCapacityTolerance = 1e-10;
constexpr double kSelectionTolerance = 1e-12;

bool IsFinite(const double value) {
  return std::isfinite(value);
}

bool HasNonzeroCapacity(const TubeProfile& profile) {
  for (const TubeRawSample& sample : profile.samples) {
    if (!IsFinite(sample.filtered_lower) || !IsFinite(sample.filtered_upper)) {
      return false;
    }
    if (sample.filtered_lower < -kCapacityTolerance ||
        sample.filtered_upper > kCapacityTolerance) {
      return true;
    }
  }
  return false;
}

bool AuthorityRequestValid(const TubeBounds& request) {
  return request.valid && IsFinite(request.query_w) &&
      IsFinite(request.lower) && IsFinite(request.upper) &&
      IsFinite(request.lower_w) && IsFinite(request.upper_w) &&
      request.query_w == 0.0 && request.lower_w == 0.0 &&
      request.upper_w == 0.0 && request.lower <= request.upper &&
      request.lower <= 0.0 && request.upper >= 0.0;
}

enum class InwardFamily {
  BOTH_SIDED,
  POSITIVE_ONLY,
  NEGATIVE_ONLY,
};

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

struct InwardCandidateSelection {
  TubeProfile profile;
  TubeSurfaceValidationResult validation;
  double current_width = 0.0;
  double certified_forward_horizon = 0.0;
  int family_order = std::numeric_limits<int>::max();
  std::size_t level = std::numeric_limits<std::size_t>::max();
  bool valid = false;
};

bool QueryBudgetExhausted(const TubeSurfaceValidationResult& validation,
                          const TubeSurfaceValidatorConfig& config) {
  // Every Validator cell consumes at most the fixed 3x3 surface cover.  Use
  // the configured budget rather than the production default, and subtract
  // before comparing so the check remains safe at the minimum valid budget.
  constexpr std::size_t kSurfacePointsPerCell = 9U;
  if (config.max_query_samples < kSurfacePointsPerCell) return true;
  return validation.query_sample_count >
      config.max_query_samples - kSurfacePointsPerCell;
}

bool RetryableValidationFailure(
    const TubeSurfaceValidationResult& validation,
    const TubeSurfaceValidatorConfig& config) {
  // A subdivision-depth exhaustion is still a rejected full-width geometry;
  // it may be followed by a fresh inward candidate.  Actual query-budget
  // exhaustion, malformed path/query input, and configuration failures are
  // terminal and remain fail-closed.
  if (QueryBudgetExhausted(validation, config) ||
      validation.first_failure_reason == TubeStopReason::INVALID_PATH ||
      validation.first_failure_reason == TubeStopReason::NONE) {
    return false;
  }
  return true;
}

void MarkInwardRepairOutcome(TubeProfile& profile,
                             const TubeSurfaceValidationResult& full_width) {
  profile.diagnostics.invalid_reason =
      std::string("full-width nonzero surface validation failed (") +
      tubeStopReasonName(full_width.first_failure_reason) +
      "); retained inward certified ribbon";
}

std::string ValidationEvidence(const TubeSurfaceValidationResult& validation) {
  return std::string("reason=") +
      tubeStopReasonName(validation.first_failure_reason) +
      " queries=" + std::to_string(validation.query_sample_count) +
      " limit_exceeded=" + (validation.limit_exceeded ? "1" : "0");
}

std::string FullWidthTerminalReason(
    const TubeSurfaceValidationResult& validation,
    const TubeSurfaceValidatorConfig& config) {
  if (QueryBudgetExhausted(validation, config)) {
    return std::string("full-width query-budget exhaustion; ") +
        ValidationEvidence(validation) +
        " max_queries=" + std::to_string(config.max_query_samples) +
        "; using planner zero baseline";
  }
  return std::string(
             "full-width nonretryable invalid/configuration failure; ") +
      ValidationEvidence(validation) + "; using planner zero baseline";
}

std::string InwardSearchFailureReason(
    const std::size_t attempt_count,
    const bool have_last_validation,
    const TubeSurfaceValidationResult& last_validation) {
  const TubeStopReason last_reason = have_last_validation
      ? last_validation.first_failure_reason : TubeStopReason::NONE;
  const std::size_t last_queries = have_last_validation
      ? last_validation.query_sample_count : 0U;
  const bool last_limit_exceeded = have_last_validation &&
      last_validation.limit_exceeded;
  return std::string(
             "inward search attempted but no candidate certified; attempts=") +
      std::to_string(attempt_count) + " last_reason=" +
      tubeStopReasonName(last_reason) + " last_queries=" +
      std::to_string(last_queries) + " last_limit_exceeded=" +
      (last_limit_exceeded ? "1" : "0") + "; using planner zero baseline";
}

std::string InwardSearchSkippedForZeroCurrentReason(
    const double current_width) {
  return std::string(
             "inward search skipped after exact-current zero-capacity proof; ") +
      "current_width=" + std::to_string(current_width) +
      "; using planner zero baseline";
}

std::string InwardSearchSkippedForZeroLimitReason(
    const TubeSurfaceValidationResult& zero_limit_validation) {
  return std::string(
             "inward search skipped because same-build zero-width limit was "
             "not continuously certifiable; zero-limit ") +
      ValidationEvidence(zero_limit_validation) +
      "; using planner zero baseline";
}

bool PrepareNarrowedCandidate(const TubeProfile& original,
                              const TubeBounds* constant_bounds,
                              const double scale,
                              const InwardFamily family,
                              const bool allow_zero_only,
                              TubeProfile& candidate) {
  const bool exact_zero_limit = scale == 0.0 &&
      family == InwardFamily::BOTH_SIDED && allow_zero_only;
  if (!IsFinite(scale) || scale < 0.0 || scale > 1.0 ||
      (scale == 0.0 && !exact_zero_limit) ||
      original.samples.size() < 2U || !original.raw_complete ||
      !original.filtered_complete || !original.complete ||
      (constant_bounds != nullptr &&
       !AuthorityRequestValid(*constant_bounds))) {
    return false;
  }

  candidate = original;
  candidate.validator_knot_evidence.clear();
  candidate.obstacle_certified = false;
  candidate.zero_centerline_continuously_certified = false;
  candidate.classification = TubeProfileClassification::NONE;

  bool has_positive = false;
  bool has_negative = false;
  for (std::size_t index = 0U; index < candidate.samples.size(); ++index) {
    TubeRawSample& sample = candidate.samples[index];
    const double original_lower = sample.filtered_lower;
    const double original_upper = sample.filtered_upper;
    if (!IsFinite(original_lower) || !IsFinite(original_upper) ||
        original_lower > original_upper + kCapacityTolerance ||
        original_lower > kCapacityTolerance ||
        original_upper < -kCapacityTolerance) {
      // Inward search may remove capacity, never manufacture the planner
      // centreline when the original filtered interval did not contain it.
      return false;
    }

    const double available_lower = constant_bounds == nullptr
        ? original_lower : std::max(original_lower, constant_bounds->lower);
    const double available_upper = constant_bounds == nullptr
        ? original_upper : std::min(original_upper, constant_bounds->upper);
    if (!IsFinite(available_lower) || !IsFinite(available_upper) ||
        available_lower > available_upper + kCapacityTolerance ||
        available_lower > 0.0 || available_upper < 0.0) {
      return false;
    }

    double lower = 0.0;
    double upper = 0.0;
    if (!exact_zero_limit) {
      switch (family) {
        case InwardFamily::BOTH_SIDED:
          lower = available_lower * scale;
          upper = available_upper * scale;
          break;
        case InwardFamily::POSITIVE_ONLY:
          lower = 0.0;
          upper = available_upper * scale;
          break;
        case InwardFamily::NEGATIVE_ONLY:
          lower = available_lower * scale;
          upper = 0.0;
          break;
      }
    }

    // Clamp only for roundoff at the original interval boundary.  These
    // operations are monotone toward zero and therefore cannot widen the
    // original Filter output.
    lower = std::max(available_lower, std::min(0.0, lower));
    upper = std::min(available_upper, std::max(0.0, upper));
    if (!IsFinite(lower) || !IsFinite(upper) ||
        lower > upper + kCapacityTolerance) {
      return false;
    }

    sample.filtered_lower = lower;
    sample.filtered_upper = upper;
    sample.filtered_contains_zero = lower <= 0.0 && 0.0 <= upper;
    has_positive = has_positive || upper > kCapacityTolerance;
    has_negative = has_negative || lower < -kCapacityTolerance;
  }

  if ((family == InwardFamily::POSITIVE_ONLY && !has_positive) ||
      (family == InwardFamily::NEGATIVE_ONLY && !has_negative) ||
      (family == InwardFamily::BOTH_SIDED && !allow_zero_only &&
       !has_positive && !has_negative)) {
    return false;
  }

  // Recompute the exact local PWL slopes and minimum width.  The original raw
  // and environment fields remain untouched as Builder evidence; only the
  // narrowed Filter bounds and their local one-sided slopes change.
  for (std::size_t index = 0U; index + 1U < candidate.samples.size(); ++index) {
    TubeRawSample& sample = candidate.samples[index];
    const TubeRawSample& next = candidate.samples[index + 1U];
    const double dw = next.w - sample.w;
    if (!IsFinite(dw) || dw <= kCapacityTolerance) return false;
    sample.lower_w = (next.filtered_lower - sample.filtered_lower) / dw;
    sample.upper_w = (next.filtered_upper - sample.filtered_upper) / dw;
    if (!IsFinite(sample.lower_w) || !IsFinite(sample.upper_w)) return false;
  }
  candidate.samples.back().lower_w =
      candidate.samples[candidate.samples.size() - 2U].lower_w;
  candidate.samples.back().upper_w =
      candidate.samples[candidate.samples.size() - 2U].upper_w;
  candidate.diagnostics.min_width = std::numeric_limits<double>::infinity();
  for (const TubeRawSample& sample : candidate.samples) {
    if (!sample.complete || !IsFinite(sample.filtered_lower) ||
        !IsFinite(sample.filtered_upper) ||
        sample.filtered_lower > sample.filtered_upper + kCapacityTolerance) {
      return false;
    }
    candidate.diagnostics.min_width = std::min(
        candidate.diagnostics.min_width,
        std::max(0.0, sample.filtered_upper - sample.filtered_lower));
  }
  if (!IsFinite(candidate.diagnostics.min_width)) {
    candidate.diagnostics.min_width = 0.0;
  }
  candidate.filtered_complete = true;
  candidate.complete = true;
  candidate.preview_start_w = candidate.samples.front().w;
  candidate.preview_end_w = candidate.samples.back().w;
  candidate.certified_segment_start_w = candidate.preview_start_w;
  candidate.certified_segment_end_w = candidate.preview_end_w;
  return true;
}

bool CurrentIntervalWidth(const TubeProfile& profile,
                          const double current_w,
                          double& width) {
  TubeBounds bounds;
  if (!TubeFilter::query(profile, current_w, bounds) || !bounds.valid ||
      !IsFinite(bounds.lower) || !IsFinite(bounds.upper) ||
      bounds.upper < bounds.lower - kCapacityTolerance) {
    return false;
  }
  width = std::max(0.0, bounds.upper - bounds.lower);
  return IsFinite(width);
}

double CertifiedForwardHorizon(const TubeProfile& profile,
                               const double current_w) {
  const double horizon = profile.certified_segment_end_w - current_w;
  return IsFinite(horizon) ? std::max(0.0, horizon) : 0.0;
}

bool BetterInwardCandidate(const InwardCandidateSelection& candidate,
                           const InwardCandidateSelection& best) {
  if (!candidate.valid) return false;
  if (!best.valid) return true;
  if (candidate.current_width > best.current_width + kSelectionTolerance) {
    return true;
  }
  if (std::abs(candidate.current_width - best.current_width) <=
      kSelectionTolerance) {
    if (candidate.certified_forward_horizon >
        best.certified_forward_horizon + kSelectionTolerance) {
      return true;
    }
    if (std::abs(candidate.certified_forward_horizon -
                 best.certified_forward_horizon) <= kSelectionTolerance) {
      if (candidate.family_order != best.family_order) {
        return candidate.family_order < best.family_order;
      }
      return candidate.level < best.level;
    }
  }
  return false;
}

void PreserveFullRibbonFailureProvenance(
    TubeProfile& profile, const TubeProfile& full_width_failure) {
  profile.diagnostics.invalid_reason =
      full_width_failure.diagnostics.invalid_reason;
  profile.diagnostics.first_invalid_w =
      full_width_failure.diagnostics.first_invalid_w;
  profile.diagnostics.first_invalid_side =
      full_width_failure.diagnostics.first_invalid_side;
  profile.diagnostics.first_stop_reason =
      full_width_failure.diagnostics.first_stop_reason;
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
        sample.filtered_lower < -kCapacityTolerance ||
        sample.filtered_upper > kCapacityTolerance;
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
      profile.current_delta_valid && std::abs(profile.current_delta) <= 1e-10;
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

bool CertifiedTubeBuilder::build(const CertifiedTubeBuildInput& input,
                                 CertifiedTubeBuildResult& result) const {
  result = CertifiedTubeBuildResult();
  if (!configurationValid()) return false;

  TubeProfile candidate;
  const bool cloud_clearance_source = input.source == TubeSource::ESDF;
  const bool raw_complete = cloud_clearance_source
      ? builder_.buildCloudClearance(
            input.source, input.preview_path, input.cloud_clearance_query,
            input.path_state_query, input.path_cell_bound_query,
            input.cloud_snapshot_resolution, input.current_w,
            input.path_source_revision, input.tube_revision, candidate,
            input.current_delta)
      : builder_.build(input.source, input.preview_path, input.distance_query,
                       input.path_source_revision, input.tube_revision, candidate,
                       input.current_delta);
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
  if (filtered_complete && !HasNonzeroCapacity(candidate)) {
    CollapseToPlannerZeroBaseline(candidate, cloud_clearance_source);
  } else if (filtered_complete && cloud_clearance_source) {
    const TubeProfile full_width_profile = candidate;
    result.surface_validation_attempted = true;
    const bool full_width_complete = surface_validator_.validate(
        candidate, input.current_w, input.path_state_query,
        input.path_cell_bound_query, input.cloud_clearance_query,
        input.cloud_snapshot_resolution,
        builder_config_.cross_section.planner_safe_distance,
        builder_config_.cross_section.regularity_margin,
        result.surface_validation);
    filtered_complete = full_width_complete;
    if (full_width_complete) {
      candidate.classification = TubeProfileClassification::OFFSET_CERTIFIED;
    } else {
      PreserveValidationFailureProvenance(candidate,
                                          result.surface_validation);
      const TubeSurfaceValidationResult full_width_validation =
          result.surface_validation;
      const bool retryable = RetryableValidationFailure(
          result.surface_validation, validator_config_);
      InwardCandidateSelection best;
      const InwardFamily families[] = {InwardFamily::BOTH_SIDED,
                                       InwardFamily::POSITIVE_ONLY,
                                       InwardFamily::NEGATIVE_ONLY};
      std::size_t attempts = 0U;
      TubeSurfaceValidationResult last_validation;
      bool have_last = false;
      if (retryable) {
        for (int family_index = 0; family_index < 3; ++family_index) {
          const InwardFamily family = families[family_index];
          double scale = family == InwardFamily::BOTH_SIDED ? 0.5 : 1.0;
          std::size_t level = 0U;
          while (scale > kCapacityTolerance) {
            TubeProfile inward;
            if (!PrepareNarrowedCandidate(full_width_profile, nullptr, scale,
                                           family, false, inward)) {
              break;
            }
            TubeSurfaceValidationResult validation;
            ++attempts;
            const bool valid = surface_validator_.validate(
                inward, input.current_w, input.path_state_query,
                input.path_cell_bound_query, input.cloud_clearance_query,
                input.cloud_snapshot_resolution,
                builder_config_.cross_section.planner_safe_distance,
                builder_config_.cross_section.regularity_margin, validation);
            last_validation = validation;
            have_last = true;
            if (valid) {
              double width = 0.0;
              if (CurrentIntervalWidth(inward, input.current_w, width) &&
                  width > kCapacityTolerance && HasNonzeroCapacity(inward) &&
                  inward.zero_centerline_continuously_certified) {
                InwardCandidateSelection selection;
                selection.profile = inward;
                selection.validation = validation;
                selection.current_width = width;
                selection.certified_forward_horizon =
                    CertifiedForwardHorizon(inward, input.current_w);
                selection.family_order = family_index;
                selection.level = level;
                selection.valid = true;
                if (BetterInwardCandidate(selection, best)) best = selection;
                break;
              }
            } else if (!RetryableValidationFailure(validation,
                                                   validator_config_)) {
              break;
            }
            scale *= 0.5;
            ++level;
          }
        }
      }
      if (best.valid) {
        candidate = best.profile;
        result.surface_validation = best.validation;
        PreserveFullRibbonFailureProvenance(candidate, full_width_profile);
        PreserveValidationFailureProvenance(candidate,
                                            full_width_validation);
        candidate.diagnostics.invalid_reason =
            "full-width surface failed; retained inward certified ribbon";
        candidate.classification = TubeProfileClassification::OFFSET_CERTIFIED;
        filtered_complete = true;
      } else {
        candidate = full_width_profile;
        PreserveValidationFailureProvenance(candidate,
                                            result.surface_validation);
        candidate.diagnostics.invalid_reason = retryable
            ? InwardSearchFailureReason(attempts, have_last, last_validation)
            : FullWidthTerminalReason(result.surface_validation,
                                       validator_config_);
        CollapseToPlannerZeroBaseline(candidate, true);
        filtered_complete = candidate.complete;
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
