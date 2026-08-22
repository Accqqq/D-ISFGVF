#pragma once

#include "phase_offset_navigation/certified_tube_builder.h"
#include "phase_offset_navigation/tube_epoch_types.h"

#include <phase_offset_core/geometry.h>

#include <cstdint>

namespace phase_offset_navigation {

// Pure C++ owner of latest candidate and independently installed active tube
// profiles.  It has no runtime-control, map-ownership, or ROS responsibilities.
class TubeEpochManager {
 public:
  explicit TubeEpochManager(
      const TubeEpochManagerConfig& config = TubeEpochManagerConfig());

  bool configurationValid() const;
  // true only means this update reached a complete state decision; callers
  // must inspect result.status.state, since CERTIFICATE_DENIED and SOURCE_NONE
  // may also return true and do not mean certified rolling operation.
  bool update(const TubeEpochUpdateInput& input, TubeEpochUpdateResult& result);

  static bool profilesEquivalent(const TubeProfile& first,
                                 const TubeProfile& second,
                                 double tolerance);
  const TubeEpochManagerConfig& config() const { return config_; }

 private:
  void copyPersistentStatus(TubeEpochStatus& status) const;
  void copyResult(const TubeEpochStatus& status, TubeEpochUpdateResult& result) const;
  void installActive(const TubeProfile& candidate,
                     const TubeEpochUpdateInput& input,
                     bool safety_replacement,
                     TubeEpochStatus& status);
  void markWaiting(TubeEpochReason reason,
                   TubeEpochStatus& status);

  TubeEpochManagerConfig config_;
  CertifiedTubeBuilder certified_builder_;
  phase_offset_core::GeometryEvaluator geometry_evaluator_;
  bool configuration_valid_ = false;

  TubeProfile candidate_profile_;
  TubeProfile active_profile_;
  bool active_available_ = false;
  bool active_current_validation_valid_ = false;
  std::uint64_t candidate_sequence_ = 0U;
  std::uint64_t active_tube_epoch_ = 0U;
  std::uint64_t active_path_source_revision_ = 0U;
  std::uint64_t active_map_observation_sequence_ = 0U;
  std::uint64_t equivalent_refresh_count_ = 0U;
  std::uint64_t install_count_ = 0U;
  std::uint64_t reject_count_ = 0U;
  std::uint64_t wait_count_ = 0U;
};

}  // namespace phase_offset_navigation
