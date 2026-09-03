#include "phase_offset_swarm/swarm_backend.h"

namespace phase_offset_swarm {

BackendSelection parseBackend(const std::string& text) {
  if (text == "disabled") {
    return {BackendKind::DISABLED, BackendParseStatus::VALID};
  }
  if (text == "d1b") {
    return {BackendKind::D1B, BackendParseStatus::VALID};
  }
  if (text == "sph") {
    return {BackendKind::SPH, BackendParseStatus::VALID};
  }
  return {BackendKind::DISABLED, BackendParseStatus::INVALID_SELECTOR};
}

}  // namespace phase_offset_swarm
