#pragma once

#include <phase_offset_navigation/recovery_reference.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <string>

#include <bspline_race/continuous_phase_normal_frame.h>
#include <bspline_race/continuous_phase_path.h>

namespace FLAG_Race {

struct PhaseOffsetRecoveryContinuationInput {
  std::shared_ptr<const ContinuousPhasePath> source_path;
  std::shared_ptr<const ContinuousPhaseNormalFrame> source_frame;
  std::shared_ptr<const ContinuousPhasePath> target_path;
  std::shared_ptr<const ContinuousPhaseNormalFrame> target_frame;
  phase_offset_navigation::RecoveryReferenceJet source_jet;
  double source_w = 0.0;
  double target_w = 0.0;
  double delta = 0.0;
  double finite_domain_start_w = 0.0;
  double finite_domain_end_w = 0.0;
  // Optional higher-order capability.  Batch-B production recovery remains a
  // first-order PhaseOffset contract and leaves this false; a future caller
  // may opt into exact r_ww evidence independently.
  double v_s_min = 0.0;
  double min_time_progress = 0.0;
  double validated_s_dot = 0.0;
  std::uint64_t s_dot_revision = 0U;
  std::string s_dot_provenance;
  bool actual_s_dot_valid = false;
  bool require_full_jet = false;
  bool require_time_progress = false;
  std::uint64_t recovery_session = 0U;
  std::uint64_t source_owner_revision = 0U;
  std::uint64_t target_owner_revision = 0U;
};

struct PhaseOffsetRecoveryContinuationOutput {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  phase_offset_navigation::RecoveryReferenceJet target_jet;
  ContinuousPhasePathState target_state;
  phase_offset_core::NormalFrameQuery target_frame_query;
  double s_dot = 0.0;
  double time_progress = 0.0;
  bool full_jet_valid = false;
  bool time_progress_valid = false;
  bool c2_continuous = false;
  bool finite_domain = false;
  bool valid = false;
  std::string provenance;
  std::string invalid_reason;
};

// Trusted C2 continuation geometry provider.  It proposes only immutable
// geometry/reference evidence; RecoveryOwner remains the selected-u owner.
class PhaseOffsetRecoveryContinuationProvider {
 public:
  static bool propose(const PhaseOffsetRecoveryContinuationInput& input,
                      PhaseOffsetRecoveryContinuationOutput& output);

  // Bind the actual selected-ZOH phase-rate witness after RecoveryOwner has
  // selected the command. This validates only immutable evidence; it does not
  // construct a connector or select/replace any command.
  static bool validateSelectedTimeProgress(
      const PhaseOffsetRecoveryContinuationInput& input,
      double actual_s_dot, std::uint64_t actual_s_dot_revision,
      const std::string& actual_s_dot_provenance,
      PhaseOffsetRecoveryContinuationOutput& output);
};

}  // namespace FLAG_Race
