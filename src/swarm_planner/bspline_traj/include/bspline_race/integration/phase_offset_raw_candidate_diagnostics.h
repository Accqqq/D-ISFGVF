#pragma once

#include <phase_offset_navigation/tube_epoch_types.h>

#include <array>
#include <cstddef>

namespace FLAG_Race {

// This independent payload deliberately does not change either existing
// diagnostics schema.  It describes the most recent due raw-candidate attempt.
enum RawCandidateDiagnosticIndex : std::size_t {
  kRawCandidateSchemaVersion = 0U,
  kRawCandidateRawSourceConfigured,
  kRawCandidateRawStorageReady,
  kRawCandidateRawQueryInjected,
  kRawCandidateTubeUpdateDue,
  kRawCandidateSequence,
  kRawCandidateEpochState,
  kRawCandidateEpochReason,
  kRawCandidateProfileRawComplete,
  kRawCandidateProfileFilteredComplete,
  kRawCandidateProfileComplete,
  kRawCandidateSampleCount,
  kRawCandidateInvalidCount,
  kRawCandidateUnavailableCount,
  kRawCandidateOutOfMapCount,
  kRawCandidateUnknownCount,
  kRawCandidateOccupiedCount,
  kRawCandidateInsufficientClearanceCount,
  kRawCandidateRequestedPreviewStartW,
  kRawCandidateRequestedPreviewEndW,
  kRawCandidateCertifiedSegmentStartW,
  kRawCandidateCertifiedSegmentEndW,
  kRawCandidateCertifiedSegmentTruncatedBefore,
  kRawCandidateCertifiedSegmentTruncatedAfter,
  kRawCandidateFirstTruncatedW,
  kRawCandidateFirstTruncatedReason,
  kRawCandidateCurrentW,
  kRawCandidateCurrentSampleFound,
  kRawCandidateCurrentSampleComplete,
  kRawCandidateCurrentCrossSectionReason,
  kRawCandidateCurrentPositiveRayTermination,
  kRawCandidateCurrentNegativeRayTermination,
  kRawCandidateCurrentBaseRawStatus,
  kRawCandidateActualPositionRawStatus,
  kRawCandidateCurrentPlusStepRawStatus,
  kRawCandidateCurrentMinusStepRawStatus,
  kRawCandidateCurrentCPlusRaw,
  kRawCandidateCurrentCMinusRaw,
  kRawCandidateCurrentEffectiveRadius,
  kRawCandidateCurrentObstacleLower,
  kRawCandidateCurrentObstacleUpper,
  kRawCandidateCurrentCurvatureLower,
  kRawCandidateCurrentCurvatureUpper,
  kRawCandidateCurrentEnvironmentLower,
  kRawCandidateCurrentEnvironmentUpper,
  kRawCandidateCurrentEnvironmentWidth,
  kRawCandidateCurrentContainsZero,
  kRawCandidateCurrentContainsPreferredDelta,
  kRawCandidateCurrentBaseToActualNorm,
  kRawCandidateDiagnosticCount,
};

static_assert(kRawCandidateDiagnosticCount == 49U,
              "raw candidate diagnostics must remain exactly 49 fields");

struct RawCandidateDiagnosticsInput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool raw_source_configured = false;
  bool raw_storage_ready = false;
  bool raw_query_injected = false;
  bool tube_update_due = false;
  phase_offset_navigation::TubeEpochStatus epoch_status;
  const phase_offset_navigation::TubeProfile* candidate_profile = nullptr;
  phase_offset_core::PathDifferentialState current_path;
  double current_w = 0.0;
  Eigen::Vector3d actual_position = Eigen::Vector3d::Zero();
  phase_offset_navigation::RawOccupancyQuery raw_occupancy_query;
  double raw_ray_step = 0.0;
};

const std::array<const char*, kRawCandidateDiagnosticCount>&
rawCandidateDiagnosticFieldNames();

std::array<double, kRawCandidateDiagnosticCount> makeRawCandidateDiagnostics(
    const RawCandidateDiagnosticsInput& input);

}  // namespace FLAG_Race
