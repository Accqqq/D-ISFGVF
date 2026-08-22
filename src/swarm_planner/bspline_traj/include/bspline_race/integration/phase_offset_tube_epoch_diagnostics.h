#pragma once

#include <phase_offset_navigation/phase_offset_runtime.h>

#include <array>
#include <cstddef>

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
  kTubeEpochDiagnosticCount,
};

static_assert(kTubeEpochDiagnosticCount == 50U,
              "S6 tube epoch diagnostics must contain exactly 50 live fields");

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

}  // namespace FLAG_Race
