#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>

#define private public
#include "bspline_race/integration/phase_offset_matched_adapter.h"
#undef private

namespace FLAG_Race {
namespace {

PhaseOffsetMatchedAdapterConfig ActiveConfig() {
  PhaseOffsetMatchedAdapterConfig config;
  config.mode = PhaseOffsetMatchedMode::ACTIVE;
  config.equivalence_tolerance = 1e-10;
  config.warmup_cycles = 100;
  return config;
}

phase_offset_navigation::TubePathKey PathKey(
    const std::uint64_t generation = 1U,
    const std::uint64_t revision = 7U) {
  phase_offset_navigation::TubePathKey key;
  key.execution_generation = generation;
  key.path_instance_id = revision;
  key.path_revision = revision;
  key.frame_revision = revision;
  key.frame_convention_id = 1U;
  key.frame_convention =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  key.phase_orientation = 1;
  key.domain_start = 0.0;
  key.domain_end = 4.0;
  return key;
}

phase_offset_navigation::TubeConfigurationKey ConfigurationKey() {
  phase_offset_navigation::TubeConfigurationKey key;
  key.configuration_id = 17U;
  key.epsilon = 0.4;
  key.nominal_half_width = 1.0;
  key.ray_step = 0.05;
  key.snapshot_resolution = 0.2;
  key.minimum_reference_speed = 1e-8;
  return key;
}

phase_offset_navigation::TubeMapCaptureKey MapKey(
    const std::uint64_t state = 11U) {
  phase_offset_navigation::TubeMapCaptureKey key;
  key.map_instance_id = 3U;
  key.state_id = state;
  key.accepted_sequence = state;
  key.configuration_generation = 5U;
  key.configuration_id = 19U;
  key.frame_provenance_id = 19U;
  key.frame_provenance = "world";
  key.support_provenance_id = 23U;
  key.accepted_time_ticks = state;
  key.support_expiry_ticks = 1000U;
  key.support_halo = 0.0;
  key.halo_reconciled = true;
  key.grid_min_index_x = -10;
  key.grid_min_index_y = -10;
  key.grid_min_index_z = -10;
  key.grid_max_index_x = 10;
  key.grid_max_index_y = 10;
  key.grid_max_index_z = 10;
  key.grid_native_origin = Eigen::Vector3d::Zero();
  key.grid_voxel_resolution = Eigen::Vector3d::Constant(0.2);
  key.complete_support = true;
  return key;
}

TubeWorkerCompletionV2 Completion(
    const TubeWorkerPurposeV2 purpose,
    const std::uint64_t request_id,
    const double useful_start,
    const double useful_end,
    const std::uint64_t map_state = 11U,
    const TubeWorkerCompletionStatusV2 status =
        TubeWorkerCompletionStatusV2::BUILT) {
  TubeWorkerCompletionV2 completion;
  completion.status = status;
  completion.purpose = purpose;
  completion.request_id = request_id;
  completion.execution_generation = 1U;
  completion.accepted_state_demand = map_state;
  completion.useful_start = useful_start;
  completion.useful_end = useful_end;
  completion.path_key = PathKey();
  completion.configuration_key = ConfigurationKey();
  completion.map_capture_key = MapKey(map_state);
  completion.build.success =
      status == TubeWorkerCompletionStatusV2::BUILT;
  return completion;
}

TEST(TubeCompletionBindingIntegrationTest,
     FailedCompletionCannotEraseUsefulCurrentCompletion) {
  PhaseOffsetMatchedAdapter adapter(ActiveConfig());
  const TubeWorkerCompletionV2 useful = Completion(
      TubeWorkerPurposeV2::CURRENT, 10U, 0.0, 2.0);
  adapter.retainV2ShadowCompletionLocked(useful);
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 10U);

  const TubeWorkerCompletionV2 failed = Completion(
      TubeWorkerPurposeV2::CURRENT, 11U, 0.0, 3.0, 12U,
      TubeWorkerCompletionStatusV2::FAILED);
  adapter.retainV2ShadowCompletionLocked(failed);
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 10U);
  EXPECT_TRUE(adapter.latest_v2_shadow_current_completion_->built());
}

TEST(TubeCompletionBindingIntegrationTest,
     NarrowerSameAuthorityCompletionDoesNotEraseWiderEvidence) {
  PhaseOffsetMatchedAdapter adapter(ActiveConfig());
  adapter.retainV2ShadowCompletionLocked(Completion(
      TubeWorkerPurposeV2::CURRENT, 20U, 0.0, 3.0));
  adapter.retainV2ShadowCompletionLocked(Completion(
      TubeWorkerPurposeV2::CURRENT, 21U, 0.5, 2.5));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 20U);
  EXPECT_DOUBLE_EQ(adapter.latest_v2_shadow_current_completion_->useful_start,
                   0.0);
  EXPECT_DOUBLE_EQ(adapter.latest_v2_shadow_current_completion_->useful_end,
                   3.0);

  adapter.retainV2ShadowCompletionLocked(Completion(
      TubeWorkerPurposeV2::CURRENT, 22U, 0.0, 4.0));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 22U);
  EXPECT_DOUBLE_EQ(adapter.latest_v2_shadow_current_completion_->useful_end,
                   4.0);
}

TEST(TubeCompletionBindingIntegrationTest,
     CurrentAndSuccessorCompletionSlotsRemainIndependent) {
  PhaseOffsetMatchedAdapter adapter(ActiveConfig());
  adapter.retainV2ShadowCompletionLocked(Completion(
      TubeWorkerPurposeV2::CURRENT, 30U, 0.0, 2.0));
  adapter.retainV2ShadowCompletionLocked(Completion(
      TubeWorkerPurposeV2::SUCCESSOR, 31U, 1.0, 3.0));
  ASSERT_TRUE(adapter.latest_v2_shadow_current_completion_);
  ASSERT_TRUE(adapter.latest_v2_shadow_successor_completion_);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->purpose,
            TubeWorkerPurposeV2::CURRENT);
  EXPECT_EQ(adapter.latest_v2_shadow_successor_completion_->purpose,
            TubeWorkerPurposeV2::SUCCESSOR);
  EXPECT_EQ(adapter.latest_v2_shadow_current_completion_->request_id, 30U);
  EXPECT_EQ(adapter.latest_v2_shadow_successor_completion_->request_id, 31U);
}

TEST(TubeCompletionBindingIntegrationTest,
     ExactRequestIdentityIncludesPurposeRangeAndEveryAuthorityKey) {
  PhaseOffsetMatchedAdapter adapter(ActiveConfig());
  phase_offset_navigation::TubeBuildInputV2 input;
  input.request_id = 40U;
  input.path_key = PathKey();
  input.configuration_key = ConfigurationKey();
  input.map_capture_key = MapKey();
  input.requested_start = 0.25;
  input.requested_end = 2.75;
  const auto current = adapter.makeV2ShadowRequestIdentity(
      TubeWorkerPurposeV2::CURRENT, input);
  const auto same = adapter.makeV2ShadowRequestIdentity(
      TubeWorkerPurposeV2::CURRENT, input);
  EXPECT_EQ(current, same);

  const auto successor = adapter.makeV2ShadowRequestIdentity(
      TubeWorkerPurposeV2::SUCCESSOR, input);
  EXPECT_NE(current, successor);
  ++input.map_capture_key.state_id;
  ++input.map_capture_key.accepted_sequence;
  ++input.map_capture_key.accepted_time_ticks;
  const auto refreshed = adapter.makeV2ShadowRequestIdentity(
      TubeWorkerPurposeV2::CURRENT, input);
  EXPECT_NE(current, refreshed);
}

TEST(TubeCompletionBindingIntegrationTest,
     FailedLocalPublicationDoesNotMutateBindingOrTaskGeneration) {
  PhaseOffsetMatchedAdapter adapter(ActiveConfig());
  std::shared_ptr<TubeV2ExecutionBinding> staged(
      new TubeV2ExecutionBinding());
  adapter.v2_execution_binding_ = staged;
  const std::uint64_t generation = adapter.executionGenerationV2();
  int publish_calls = 0;
  EXPECT_FALSE(adapter.publishPendingPositionCommand([&publish_calls]() {
    ++publish_calls;
    return false;
  }));
  EXPECT_EQ(publish_calls, 1);
  EXPECT_EQ(adapter.v2_execution_binding_.get(), staged.get());
  EXPECT_EQ(adapter.executionGenerationV2(), generation);
}

TEST(TubeCompletionBindingIntegrationTest,
     NewTaskResetRetiresCompletionBindingAndPendingStateOnce) {
  PhaseOffsetMatchedAdapter adapter(ActiveConfig());
  adapter.retainV2ShadowCompletionLocked(Completion(
      TubeWorkerPurposeV2::CURRENT, 50U, 0.0, 2.0));
  adapter.retainV2ShadowCompletionLocked(Completion(
      TubeWorkerPurposeV2::SUCCESSOR, 51U, 1.0, 3.0));
  adapter.v2_execution_binding_ =
      std::make_shared<const TubeV2ExecutionBinding>();
  adapter.pending_v2_shadow_bootstrap_candidate_ =
      std::make_shared<const TubeV2ShadowAdmissionCandidate>();
  adapter.latest_v2_shadow_admission_candidate_ =
      adapter.pending_v2_shadow_bootstrap_candidate_;
  const std::uint64_t before = adapter.executionGenerationV2();

  ASSERT_TRUE(adapter.resetForNewNavigationTask());
  EXPECT_EQ(adapter.executionGenerationV2(), before + 1U);
  EXPECT_FALSE(adapter.latest_v2_shadow_current_completion_);
  EXPECT_FALSE(adapter.latest_v2_shadow_successor_completion_);
  EXPECT_FALSE(adapter.latest_v2_shadow_admission_candidate_);
  EXPECT_FALSE(adapter.pending_v2_shadow_bootstrap_candidate_);
  EXPECT_FALSE(adapter.v2_execution_binding_);
  EXPECT_FALSE(std::atomic_load(&adapter.latest_build_request_));
}

TEST(TubeCompletionBindingIntegrationTest,
     EmptyPendingCaptureIsExplicitlyValidAndNonAuthoritative) {
  PhaseOffsetMatchedAdapter adapter(ActiveConfig());
  const PendingPositionCommandCapture capture =
      adapter.capturePendingPositionCommand();
  EXPECT_TRUE(capture.valid);
  EXPECT_FALSE(capture.pending);
  EXPECT_EQ(capture.identity, 0U);
  EXPECT_FALSE(capture.reference_query);
  EXPECT_FALSE(capture.v2_binding_transition);
  EXPECT_FALSE(capture.proposed_v2_binding);
}

TEST(TubeCompletionBindingIntegrationTest,
     EventVocabularyContainsNoReadyOrArmedLifecycle) {
  const TubeWorkerEventTypeV2 events[] = {
      TubeWorkerEventTypeV2::SUBMITTED,
      TubeWorkerEventTypeV2::COALESCED,
      TubeWorkerEventTypeV2::REJECTED,
      TubeWorkerEventTypeV2::BUILD_STARTED,
      TubeWorkerEventTypeV2::BUILD_FINISHED,
      TubeWorkerEventTypeV2::CANCEL_REQUESTED,
      TubeWorkerEventTypeV2::COMPLETION_DISCARDED,
      TubeWorkerEventTypeV2::COMPLETION_DELIVERED,
      TubeWorkerEventTypeV2::RESET,
      TubeWorkerEventTypeV2::PATH_INVALIDATED,
      TubeWorkerEventTypeV2::SHUTDOWN,
  };
  for (const TubeWorkerEventTypeV2 event : events) {
    const std::string name = tubeWorkerEventTypeName(event);
    EXPECT_FALSE(name.empty());
    EXPECT_EQ(name.find("READY"), std::string::npos);
    EXPECT_EQ(name.find("ARMED"), std::string::npos);
    EXPECT_EQ(name.find("EPOCH"), std::string::npos);
  }
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
