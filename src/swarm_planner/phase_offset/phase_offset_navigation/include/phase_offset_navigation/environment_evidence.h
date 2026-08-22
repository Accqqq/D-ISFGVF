#pragma once

#include "phase_offset_navigation/distance_query.h"

#include <Eigen/Core>

#include <cstdint>
#include <functional>

namespace phase_offset_navigation {

// Uninflated environmental layers that may contribute to a future distance
// backing.  A layer bit identifies provenance; it is not a pre-applied
// occupancy-inflation or robust-clearance term.
enum EnvironmentEvidenceLayer : std::uint32_t {
  kEnvironmentLayerNone = 0U,
  kEnvironmentLayerSensorUninflated = 1U << 0,
  kEnvironmentLayerManualUninflated = 1U << 1,
  kEnvironmentLayerStaticUninflated = 1U << 2,
  kEnvironmentLayerGlobalStaticTest = 1U << 3,
  kEnvironmentLayerAll = kEnvironmentLayerSensorUninflated |
      kEnvironmentLayerManualUninflated |
      kEnvironmentLayerStaticUninflated |
      kEnvironmentLayerGlobalStaticTest,
};

// A finite, fail-closed answer about distance to the nearest included,
// uninflated environmental occupied set.  It intentionally carries no tube
// margin, pre-applied occupancy inflation, or signed tracking clearance.
struct EnvironmentEvidenceResult {
  DistanceStatus status = DistanceStatus::UNAVAILABLE;
  double uninflated_distance = 0.0;
  bool distance_valid = false;
  std::uint32_t included_layer_mask = kEnvironmentLayerNone;
  std::uint32_t unsupported_active_layer_mask = kEnvironmentLayerNone;
};

using EnvironmentEvidenceQuery =
    std::function<EnvironmentEvidenceResult(const Eigen::Vector3d&)>;

// Verifies the status, finite numeric, validity, and provenance rules of an
// environment-evidence answer.  An unsupported active layer may remain a
// consistent fail-closed diagnostic fact, but it can never be usable.
bool environmentEvidenceConsistent(const EnvironmentEvidenceResult& result);

// True only for a consistent OCCUPIED or KNOWN_FREE result with a usable,
// explicitly supported uninflated distance.
bool environmentEvidenceHasUsableDistance(
    const EnvironmentEvidenceResult& result);

}  // namespace phase_offset_navigation
