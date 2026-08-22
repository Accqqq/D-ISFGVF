#pragma once

#include <cstdint>

namespace plan_env {

// SDFMap-local provenance values.  They deliberately mirror the abstract
// contract's information without depending on a navigation package.
enum SDFMapEnvironmentEvidenceLayer : std::uint32_t {
  kSDFMapEnvironmentLayerNone = 0U,
  kSDFMapEnvironmentLayerSensorUninflated = 1U << 0,
  kSDFMapEnvironmentLayerManualUninflated = 1U << 1,
  kSDFMapEnvironmentLayerStaticUninflated = 1U << 2,
  kSDFMapEnvironmentLayerGlobalStaticTest = 1U << 3,
  kSDFMapEnvironmentLayerAll = kSDFMapEnvironmentLayerSensorUninflated |
      kSDFMapEnvironmentLayerManualUninflated |
      kSDFMapEnvironmentLayerStaticUninflated |
      kSDFMapEnvironmentLayerGlobalStaticTest,
};

enum class SDFMapEnvironmentEvidenceStatus {
  UNAVAILABLE,
  OUT_OF_MAP,
  UNKNOWN,
  KNOWN_FREE,
  OCCUPIED,
};

struct SDFMapEnvironmentEvidenceResult {
  SDFMapEnvironmentEvidenceStatus status =
      SDFMapEnvironmentEvidenceStatus::UNAVAILABLE;
  double uninflated_distance = 0.0;
  bool distance_valid = false;
  std::uint32_t included_layer_mask = kSDFMapEnvironmentLayerNone;
  std::uint32_t unsupported_active_layer_mask =
      kSDFMapEnvironmentLayerNone;
};

struct SDFMapEnvironmentEvidenceCapabilities {
  bool observation_status_available = false;
  bool uninflated_distance_available = false;
  std::uint32_t supported_layer_mask = kSDFMapEnvironmentLayerNone;
};

constexpr std::uint32_t kSDFMapCurrentUnsupportedEnvironmentLayerMask =
    kSDFMapEnvironmentLayerSensorUninflated |
    kSDFMapEnvironmentLayerManualUninflated |
    kSDFMapEnvironmentLayerStaticUninflated;

bool sdfMapEnvironmentEvidenceConsistent(
    const SDFMapEnvironmentEvidenceResult& result);

}  // namespace plan_env
