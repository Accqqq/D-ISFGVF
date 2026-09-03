#pragma once

#include <string>

namespace phase_offset_swarm {

enum class BackendKind { DISABLED = 0, D1B = 1, SPH = 2 };
enum class BackendParseStatus { VALID = 0, INVALID_SELECTOR = 1 };

struct BackendSelection {
  BackendKind kind = BackendKind::DISABLED;
  BackendParseStatus status = BackendParseStatus::VALID;
};

BackendSelection parseBackend(const std::string& text);

}  // namespace phase_offset_swarm
