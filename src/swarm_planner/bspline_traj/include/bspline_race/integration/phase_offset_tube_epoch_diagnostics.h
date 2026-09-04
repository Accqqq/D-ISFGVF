#pragma once

#include <phase_offset_navigation/phase_offset_runtime.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace FLAG_Race {

enum TubeEpochDiagnosticIndex : std::size_t {
  kEpochSchemaVersion = 0U,
  kEpochTubeSource,
  kEpochState,
  kEpochInstallDisposition,
  kEpochReason,
  kEpochCandidateSequence,
  kEpochActiveTubeEpoch,
  kEpochCandidatePathSourceRevision,
  kEpochActivePathSourceRevision,
  kEpochCandidateMapObservationSequence,
  kEpochActiveMapObservationSequence,
  kEpochMapObservationIsSnapshot,
  kEpochCandidateRawComplete,
  kEpochCandidateFilteredComplete,
  kEpochCandidateComplete,
  kEpochActiveAvailable,
  kEpochActiveCurrentValidationValid,
  kEpochCurrentGeometryValid,
  kEpochCurrentBoundsValid,
  kEpochRetainedDeltaCurrentInside,
  kEpochCurrentStateAdmissible,
  kEpochCurrentSafetyStatus,
  kEpochReferenceClearanceSufficient,
  kEpochActualClearanceSufficient,
  kEpochTrackingWithinBound,
  kEpochForwardHorizonSufficient,
  kEpochRetainedDelta,
  kEpochTrackingErrorNorm,
  kEpochTrackingErrorBound,
  kEpochReferenceSignedDistance,
  kEpochActualSignedDistance,
  kEpochRequiredReferenceClearance,
  kEpochRequiredActualClearance,
  kEpochCertifiedForwardW,
  kEpochEquivalentRefreshCount,
  kEpochInstallCount,
  kEpochRejectCount,
  kEpochWaitCount,
  kEpochCandidateSampleCount,
  kEpochActiveSampleCount,
  kEpochActiveProfileComplete,
  kEpochActiveObstacleCertified,
  kEpochActiveDisplayCertified,
  kEpochTubeUpdateDueThisCycle,
  kEpochRuntimeExecutionMode,
  kEpochControlSelected,
  kEpochCertificateDenied,
  kEpochTransientBlocked,
  kEpochGenuineFatalInvariant,
  kEpochRuntimeFailureReason,
  // SurfaceValidator v4 fields.  Append only: the first 50 indices above are
  // permanent live schema and must never move.
  kCandidateSurfaceOutcome,
  kCandidateSurfaceInconclusiveReason,
  kCandidateSurfaceTruncationOutcome,
  kCandidateSurfaceTerminalW,
  kCandidateSurfaceWitnessClearance,
  kCandidateSurfaceWitnessClearanceExact,
  kCandidateSurfaceMidpointPositionCover,
  kCandidateSurfaceNormalVariationCover,
  kCandidateSurfaceDeltaSlopeCover,
  kCandidateSurfaceVSpanCover,
  kCandidateSurfaceGeometricCover,
  kCandidateSurfaceSupportAlignmentBound,
  kCandidateSurfaceNumericalEpsilon,
  kCandidateSurfaceProofResidual,
  kCandidateSurfaceMaxDepthObserved,
  kCandidateSurfaceQuerySampleCount,
  kCandidateSurfaceDepthGuardReached,
  kCandidateSurfaceQueryBudgetReached,
  kCandidateSurfaceSplitWCount,
  kCandidateSurfaceSplitVCount,
  kCandidateSurfaceSplitBothCount,
  kCandidateZeroCenterlineContiguous,
  kCandidateZeroCenterlineStartW,
  kCandidateZeroCenterlineEndW,
  kTubeEpochDiagnosticCount,
};

static_assert(kTubeEpochDiagnosticCount == 74U,
              "S6 tube epoch diagnostics must contain exactly 74 live fields");

struct TubeEpochDiagnosticsInput {
  phase_offset_navigation::TubeSource source =
      phase_offset_navigation::TubeSource::NONE;
  phase_offset_navigation::TubeEpochStatus epoch;
  phase_offset_navigation::RuntimeExecutionStatus runtime;
  const phase_offset_navigation::TubeProfile* candidate_profile = nullptr;
  const phase_offset_navigation::TubeProfile* active_profile = nullptr;
  double retained_delta = 0.0;
  bool active_display_certified = false;
  bool tube_update_due_this_cycle = false;
  bool control_selected = false;
  bool failure_latched = false;
  phase_offset_navigation::ControlFailureReason control_failure_reason =
      phase_offset_navigation::ControlFailureReason::NONE;
};

const std::array<const char*, kTubeEpochDiagnosticCount>&
tubeEpochDiagnosticFieldNames();

std::array<double, kTubeEpochDiagnosticCount> makeTubeEpochDiagnostics(
    const TubeEpochDiagnosticsInput& input);

// Independent structured-log transport for the diagnostic-only nearest
// forward excluded leaf.  This value never participates in the 74-field
// epoch diagnostics array or any Runtime/authority decision.
struct TubeSurfaceForwardExcludedLogInput {
  std::uint64_t build_sequence = 0U;
  std::uint64_t candidate_sequence = 0U;
  std::uint64_t task_generation = 0U;
  std::uint64_t authority_session = 0U;
  std::uint64_t source_revision = 0U;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t map_observation_sequence = 0U;
  double current_w = 0.0;
  double certified_segment_end_w = 0.0;
  const phase_offset_navigation::TubeSurfaceForwardExcludedEvidence*
      evidence = nullptr;
};

std::string formatTubeSurfaceForwardExcludedLog(
    const TubeSurfaceForwardExcludedLogInput& input);

}  // namespace FLAG_Race
