#include "bspline_race/integration/phase_offset_tube_runtime_v2.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace FLAG_Race {
namespace {

using phase_offset_core::CertifiedPathCellV2;
using phase_offset_navigation::TubeBuildInputV2;
using phase_offset_navigation::TubeCertificateConfigV2;
using phase_offset_navigation::TubeFreeBallQuery;
using phase_offset_navigation::TubeFreeBallQueryResult;
using phase_offset_navigation::TubeMapCaptureKey;
using phase_offset_navigation::TubePathKey;
using phase_offset_navigation::TubeSupportFootprintV2;
using phase_offset_navigation::TubeVoxelFootprintV2;

void SetInterval(phase_offset_core::Binary64Interval& interval,
                 const double lower, const double upper) {
  interval.lower = lower;
  interval.upper = upper;
  interval.valid = true;
}

void SetVectorInterval(phase_offset_core::Binary64VectorInterval& interval,
                       const Eigen::Vector3d& lower,
                       const Eigen::Vector3d& upper) {
  interval.valid = true;
  for (int i = 0; i < 3; ++i) {
    SetInterval(interval.component[static_cast<std::size_t>(i)],
                lower(i), upper(i));
  }
}

// Immutable analytic path p(w)=(2+w,2,2), w in [0,1].  The owner is retained
// in the input so a queued job never observes a mutable planner path object.
struct AnalyticPathOwner {
  CertifiedPathCellV2 cell;
};

CertifiedPathCellV2 AnalyticCell(const double w0, const double w1) {
  CertifiedPathCellV2 cell;
  cell.w0 = w0;
  cell.w1 = w1;
  cell.anchor_w = (w0 + w1) * 0.5;
  cell.path_revision = 7U;
  cell.frame_revision = 7U;
  cell.segment_identity = 100U +
      static_cast<std::uint64_t>(w0 * 1000.0 + 0.5);
  cell.proof_identity = 1000U + cell.segment_identity;
  const double x = 2.0 + cell.anchor_w;
  SetVectorInterval(cell.anchor_position, Eigen::Vector3d(x, 2.0, 2.0),
                    Eigen::Vector3d(x, 2.0, 2.0));
  SetVectorInterval(cell.anchor_p_w, Eigen::Vector3d(1.0, 0.0, 0.0),
                    Eigen::Vector3d(1.0, 0.0, 0.0));
  SetVectorInterval(cell.anchor_p_ww, Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());
  SetInterval(cell.inf_p_w_norm, 1.0, 1.0);
  SetInterval(cell.sup_p_w_norm, 1.0, 1.0);
  SetInterval(cell.inf_horizontal_p_w_norm, 1.0, 1.0);
  SetInterval(cell.sup_p_ww_norm, 0.0, 0.0);
  SetInterval(cell.sup_horizontal_p_ww_norm, 0.0, 0.0);
  SetInterval(cell.sup_p_www_norm, 0.0, 0.0);
  SetInterval(cell.sup_normal_derivative, 0.0, 0.0);
  SetInterval(cell.normal_variation, 0.0, 0.0);
  SetInterval(cell.tangent_variation, 0.0, 0.0);
  SetInterval(cell.curvature_variation, 0.0, 0.0);
  SetInterval(cell.midpoint_position_variation, 0.0,
              (w1 - w0) * 0.5);
  SetInterval(cell.chord_deviation, 0.0, 0.0);
  cell.horizontal_acceleration_bound_complete = true;
  cell.normal_frame_proof_complete = true;
  cell.phase_map_proof_complete = true;
  cell.provenance = phase_offset_core::kWorldHorizontalCrossProductProvenance;
  cell.valid = true;
  cell.complete = true;
  return cell;
}

TubeCertificateConfigV2 Config() {
  TubeCertificateConfigV2 config;
  config.configuration_id = 17U;
  config.epsilon = 0.05;
  config.nominal_half_width = 0.2;
  config.ray_step = 0.1;
  config.snapshot_resolution = 0.2;
  config.minimum_reference_speed = 0.5;
  config.sample_step_w = 1.0;
  config.budgets.max_w_depth = 4;
  config.budgets.max_queries = 1000U;
  config.budgets.max_cells = 1000U;
  config.budgets.max_samples_per_cell = 1000U;
  config.budgets.max_witnesses = 1000U;
  return config;
}

TubeMapCaptureKey MapKey() {
  TubeMapCaptureKey key;
  key.map_instance_id = 3U;
  key.state_id = 4U;
  key.accepted_sequence = 5U;
  key.configuration_generation = 6U;
  key.configuration_id = 99U;
  key.frame_provenance_id = 8U;
  key.frame_provenance = "world";
  key.support_provenance_id = 9U;
  key.accepted_time_ticks = 10U;
  key.support_expiry_ticks = 20U;
  key.support_halo = 0.0;
  key.halo_reconciled = true;
  key.grid_min_index_x = -100;
  key.grid_min_index_y = -100;
  key.grid_min_index_z = -100;
  key.grid_max_index_x = 100;
  key.grid_max_index_y = 100;
  key.grid_max_index_z = 100;
  key.grid_native_origin = Eigen::Vector3d::Zero();
  key.grid_voxel_resolution = Eigen::Vector3d::Constant(0.2);
  key.complete_support = true;
  return key;
}

TubeSupportFootprintV2 Support(const TubeMapCaptureKey& key,
                               const Eigen::Vector3d& witness,
                               const double radius) {
  TubeSupportFootprintV2 support;
  support.lower = witness - Eigen::Vector3d::Constant(2.0);
  support.upper = witness + Eigen::Vector3d::Constant(2.0);
  support.witness = witness;
  support.radius = radius;
  support.map_instance_id = key.map_instance_id;
  support.map_state_id = key.state_id;
  support.support_provenance_id = key.support_provenance_id;
  support.accepted_time_ticks = key.accepted_time_ticks;
  support.support_expiry_ticks = key.support_expiry_ticks;
  support.support_expiry_timeless = key.support_expiry_timeless;
  support.valid = true;
  TubeVoxelFootprintV2 voxel;
  voxel.min_index_x = key.grid_min_index_x;
  voxel.min_index_y = key.grid_min_index_y;
  voxel.min_index_z = key.grid_min_index_z;
  voxel.max_index_x = key.grid_max_index_x;
  voxel.max_index_y = key.grid_max_index_y;
  voxel.max_index_z = key.grid_max_index_z;
  voxel.native_origin = key.grid_native_origin;
  voxel.voxel_resolution = key.grid_voxel_resolution;
  voxel.native_index = key.grid_native_index;
  voxel.valid = true;
  support.voxel_footprint.push_back(voxel);
  return support;
}

TubeFreeBallQuery OpenQuery(const TubeMapCaptureKey& key) {
  return [key](const Eigen::Vector3d& witness, const double required) {
    TubeFreeBallQueryResult result;
    result.status = phase_offset_navigation::DistanceStatus::KNOWN_FREE;
    result.certified_radius = required;
    result.certified = true;
    result.complete_support = true;
    result.map_instance_id = key.map_instance_id;
    result.map_state_id = key.state_id;
    result.configuration_id = key.configuration_id;
    result.frame_provenance_id = key.frame_provenance_id;
    result.frame_provenance = key.frame_provenance;
    result.accepted_sequence = key.accepted_sequence;
    result.configuration_generation = key.configuration_generation;
    result.support_provenance_id = key.support_provenance_id;
    result.accepted_time_ticks = key.accepted_time_ticks;
    result.support_expiry_ticks = key.support_expiry_ticks;
    result.support_expiry_timeless = key.support_expiry_timeless;
    result.halo_reconciled = key.halo_reconciled;
    result.support = Support(key, witness, required);
    return result;
  };
}

TubeBuildInputV2 Input(const TubeCertificateConfigV2& config,
                       const std::uint64_t generation = 1U,
                       const TubeFreeBallQuery& query = TubeFreeBallQuery(),
                       std::shared_ptr<const AnalyticPathOwner>* owner_out =
                           nullptr) {
  const std::shared_ptr<AnalyticPathOwner> owner(new AnalyticPathOwner());
  owner->cell = AnalyticCell(0.0, 1.0);
  const std::shared_ptr<const AnalyticPathOwner> immutable_owner = owner;
  if (owner_out != nullptr) *owner_out = immutable_owner;
  TubeBuildInputV2 input;
  input.request_id = 1U;
  input.path_key.execution_generation = generation;
  input.path_key.path_instance_id = 2U;
  input.path_key.path_revision = 7U;
  input.path_key.frame_revision = 7U;
  input.path_key.frame_convention_id = 1U;
  input.path_key.frame_convention =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  input.path_key.phase_orientation = 1;
  input.path_key.domain_start = 0.0;
  input.path_key.domain_end = 1.0;
  input.configuration_key = config.key();
  input.map_capture_key = MapKey();
  input.requested_start = 0.0;
  input.requested_end = 1.0;
  input.anchor_w = 0.5;
  input.path_cells.push_back(owner->cell);
  input.producer_breakpoints = {0.0, 1.0};
  input.path_cell_query = [immutable_owner](const double w0,
                                             const double w1,
                                             CertifiedPathCellV2& cell) {
    (void)immutable_owner;
    cell = AnalyticCell(w0, w1);
    return true;
  };
  input.path_owner = std::static_pointer_cast<const void>(immutable_owner);
  input.query_owner = std::static_pointer_cast<const void>(
      std::make_shared<int>(1));
  input.capture_owner = std::static_pointer_cast<const void>(
      std::make_shared<int>(2));
  input.applicability_assumptions = "immutable analytic path and map capture";
  // Keep applicability within the captured support's finite expiry.
  input.applicability_deadline_ticks = 20U;
  input.free_ball_query = query ? query : OpenQuery(input.map_capture_key);
  return input;
}

TubeWorkerRequestV2 Request(const TubeCertificateConfigV2& config,
                            const TubeWorkerPurposeV2 purpose,
                            const std::uint64_t id,
                            const std::uint64_t generation = 1U,
                            const double start = 0.0,
                            const double end = 1.0,
                            std::shared_ptr<const AnalyticPathOwner>* owner_out =
                                nullptr) {
  TubeWorkerRequestV2 request;
  request.purpose = purpose;
  request.request_id = id;
  request.execution_generation = generation;
  request.accepted_state_demand = 5U;
  request.useful_start = start;
  request.useful_end = end;
  request.input = Input(config, generation, TubeFreeBallQuery(), owner_out);
  request.input.request_id = id;
  request.input.requested_start = start;
  request.input.requested_end = end;
  request.input.anchor_w = (start + end) * 0.5;
  request.input.path_key.domain_start = 0.0;
  request.input.path_key.domain_end = 1.0;
  return request;
}

struct Gate {
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false;
  bool release = false;
  std::size_t completions = 0U;
};

bool WaitFor(const std::function<bool()>& predicate,
             const std::chrono::milliseconds timeout =
                 std::chrono::milliseconds(3000)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::yield();
  }
  return true;
}

TubeWorkerHooksV2 Hooks(const std::shared_ptr<Gate>& gate,
                        const bool block_before = false) {
  TubeWorkerHooksV2 hooks;
  hooks.before_build = [gate, block_before](const TubeWorkerRequestV2&) {
    std::unique_lock<std::mutex> lock(gate->mutex);
    gate->entered = true;
    gate->condition.notify_all();
    if (block_before) {
      gate->condition.wait(lock, [gate]() { return gate->release; });
    }
  };
  hooks.after_build = [gate](const TubeWorkerRequestV2&,
                             const TubeWorkerCompletionV2&) {
    std::lock_guard<std::mutex> lock(gate->mutex);
    ++gate->completions;
    gate->condition.notify_all();
  };
  return hooks;
}

bool WaitForGate(const std::shared_ptr<Gate>& gate, const bool completion) {
  std::unique_lock<std::mutex> lock(gate->mutex);
  return gate->condition.wait_for(
      lock, std::chrono::seconds(3), [gate, completion]() {
        return completion ? gate->completions > 0U : gate->entered;
      });
}

TEST(PhaseOffsetTubeWorkerV2Test, RequestCompletenessAndIdentity) {
  const TubeCertificateConfigV2 config = Config();
  TubeWorkerRequestV2 request = Request(config, TubeWorkerPurposeV2::CURRENT, 1U);
  EXPECT_TRUE(request.complete());
  request.accepted_state_demand = 6U;
  EXPECT_FALSE(request.complete());
  request.accepted_state_demand = 5U;
  request.input.configuration_key.configuration_id += 1U;
  EXPECT_TRUE(request.complete());
  PhaseOffsetTubeWorkerV2 worker(config);
  EXPECT_FALSE(worker.submit(request));
  EXPECT_FALSE(worker.cancel(static_cast<TubeWorkerPurposeV2>(77)));
  TubeWorkerCompletionV2 completion;
  EXPECT_FALSE(worker.tryTake(static_cast<TubeWorkerPurposeV2>(77), completion));
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, ActualBuilderInvocationAndCompletionDelivery) {
  const TubeCertificateConfigV2 config = Config();
  auto gate = std::make_shared<Gate>();
  PhaseOffsetTubeWorkerV2 worker(config, Hooks(gate));
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
  ASSERT_TRUE(WaitForGate(gate, true));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTake(TubeWorkerPurposeV2::CURRENT, completion));
  EXPECT_TRUE(completion.built());
  EXPECT_TRUE(completion.heavy_build_entered);
  EXPECT_GT(completion.build_duration_ns, 0U);
  EXPECT_GT(completion.path_callback_count, 0U);
  EXPECT_GT(completion.free_ball_callback_count, 0U);
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, ExactlyOneHeavyBuildAndInflightBound) {
  const TubeCertificateConfigV2 config = Config();
  auto gate = std::make_shared<Gate>();
  PhaseOffsetTubeWorkerV2 worker(config, Hooks(gate, true));
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
  ASSERT_TRUE(WaitForGate(gate, false));
  EXPECT_FALSE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 2U,
                                     1U, 0.0, 0.8)));
  {
    std::lock_guard<std::mutex> lock(gate->mutex);
    gate->release = true;
  }
  gate->condition.notify_all();
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  worker.shutdown();
  const TubeWorkerStatsV2 stats = worker.stats();
  EXPECT_EQ(stats.build_started, 1U);
  EXPECT_EQ(stats.peak_in_flight, 1U);
}

TEST(PhaseOffsetTubeWorkerV2Test, PurposeSlotsAndFairScheduling) {
  const TubeCertificateConfigV2 config = Config();
  auto gate = std::make_shared<Gate>();
  PhaseOffsetTubeWorkerV2 worker(config, Hooks(gate, true));
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
  ASSERT_TRUE(WaitForGate(gate, false));
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::SUCCESSOR, 2U)));
  TubeWorkerRequestV2 current_replacement = Request(
      config, TubeWorkerPurposeV2::CURRENT, 3U, 1U, 0.0, 0.8);
  // Use a distinct immutable path identity so this pending current work is
  // not covered by the already-running full-range current request.
  current_replacement.input.path_key.path_instance_id = 3U;
  ASSERT_TRUE(worker.submit(current_replacement));
  EXPECT_FALSE(worker.submit(Request(config, TubeWorkerPurposeV2::SUCCESSOR, 4U,
                                     1U, 0.0, 0.8)));
  {
    std::lock_guard<std::mutex> lock(gate->mutex);
    gate->release = true;
  }
  gate->condition.notify_all();
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 3U; }));
  const TubeWorkerStatsV2 stats = worker.stats();
  EXPECT_EQ(stats.pending_current + stats.pending_successor, 0U);
  EXPECT_EQ(stats.peak_in_flight, 1U);
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test,
     SuccessorPurposeCancellationLeavesRunningCurrentUntouched) {
  const TubeCertificateConfigV2 config = Config();
  auto gate = std::make_shared<Gate>();
  PhaseOffsetTubeWorkerV2 worker(config, Hooks(gate, true));
  ASSERT_TRUE(worker.submit(Request(
      config, TubeWorkerPurposeV2::CURRENT, 1U)));
  ASSERT_TRUE(WaitForGate(gate, false));
  ASSERT_TRUE(worker.submit(Request(
      config, TubeWorkerPurposeV2::SUCCESSOR, 2U)));
  EXPECT_EQ(worker.stats().pending_successor, 1U);
  ASSERT_TRUE(worker.cancel(TubeWorkerPurposeV2::SUCCESSOR));
  EXPECT_EQ(worker.stats().pending_successor, 0U);
  {
    std::lock_guard<std::mutex> lock(gate->mutex);
    gate->release = true;
  }
  gate->condition.notify_all();
  ASSERT_TRUE(WaitFor([&worker]() {
    return worker.stats().delivery_count >= 1U;
  }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTake(TubeWorkerPurposeV2::CURRENT, completion));
  EXPECT_TRUE(completion.built());
  EXPECT_FALSE(worker.tryTake(TubeWorkerPurposeV2::SUCCESSOR, completion));
  EXPECT_EQ(worker.stats().build_started, 1U);
  EXPECT_EQ(worker.stats().peak_in_flight, 1U);
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, PendingCoalescingAndCoverageDominance) {
  const TubeCertificateConfigV2 config = Config();
  auto gate = std::make_shared<Gate>();
  PhaseOffsetTubeWorkerV2 worker(config, Hooks(gate, true));
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
  ASSERT_TRUE(WaitForGate(gate, false));
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::SUCCESSOR, 2U,
                                    1U, 0.0, 0.4)));
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::SUCCESSOR, 3U,
                                    1U, 0.0, 1.0)));
  EXPECT_FALSE(worker.submit(Request(config, TubeWorkerPurposeV2::SUCCESSOR, 4U,
                                     1U, 0.0, 0.2)));
  {
    std::lock_guard<std::mutex> lock(gate->mutex);
    gate->release = true;
  }
  gate->condition.notify_all();
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 2U; }));
  worker.shutdown();
  EXPECT_GE(worker.stats().coalesced, 1U);
}

TEST(PhaseOffsetTubeWorkerV2Test, ShorterSameStartRejectedAndSuffixAccepted) {
  const TubeCertificateConfigV2 config = Config();
  PhaseOffsetTubeWorkerV2 worker(config);
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U,
                                    1U, 0.0, 0.6)));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTake(TubeWorkerPurposeV2::CURRENT, completion));
  EXPECT_FALSE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 2U,
                                     1U, 0.0, 0.5)));
  EXPECT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 3U,
                                    1U, 0.5, 1.0)));
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, ImmutableWorkDifferentIdAndConsumedRetryDoNotRecertify) {
  const TubeCertificateConfigV2 config = Config();
  PhaseOffsetTubeWorkerV2 worker(config);
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTake(TubeWorkerPurposeV2::CURRENT, completion));
  EXPECT_FALSE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 2U)));
  EXPECT_FALSE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
  EXPECT_EQ(worker.stats().build_started, 1U);
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, ActualTruncatedCertifiedRangeControlsUsefulness) {
  const TubeCertificateConfigV2 config = Config();
  const TubeMapCaptureKey key = MapKey();
  const TubeFreeBallQuery query = [key](const Eigen::Vector3d& witness,
                                        const double required) {
    TubeFreeBallQueryResult result = OpenQuery(key)(witness, required);
    if (witness.x() > 2.5) {
      result.status = phase_offset_navigation::DistanceStatus::OCCUPIED;
      result.certified = false;
      result.complete_support = false;
    }
    return result;
  };
  PhaseOffsetTubeWorkerV2 worker(config);
  TubeWorkerRequestV2 request = Request(config, TubeWorkerPurposeV2::CURRENT, 1U);
  request.input.path_cells.clear();
  request.input.path_cells.push_back(AnalyticCell(0.0, 0.5));
  request.input.path_cells.push_back(AnalyticCell(0.5, 1.0));
  request.input.producer_breakpoints = {0.0, 0.5, 1.0};
  request.input.anchor_w = 0.25;
  request.input.free_ball_query = query;
  ASSERT_TRUE(worker.submit(request));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTake(TubeWorkerPurposeV2::CURRENT, completion));
  ASSERT_TRUE(completion.built());
  EXPECT_LT(completion.useful_end, request.useful_end);
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, CallbackBoundaryCancellationAndRetry) {
  const TubeCertificateConfigV2 config = Config();
  auto gate = std::make_shared<Gate>();
  PhaseOffsetTubeWorkerV2 worker(config, Hooks(gate, true));
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
  ASSERT_TRUE(WaitForGate(gate, false));
  ASSERT_TRUE(worker.cancel(TubeWorkerPurposeV2::CURRENT));
  {
    std::lock_guard<std::mutex> lock(gate->mutex);
    gate->release = true;
  }
  gate->condition.notify_all();
  ASSERT_TRUE(WaitFor([&worker]() {
    return worker.stats().cancellation_discarded >= 1U;
  }));
  EXPECT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 2U)));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTakeAny(completion));
  EXPECT_TRUE(completion.built());
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, FailedCompletionDoesNotSuppressRetry) {
  const TubeCertificateConfigV2 config = Config();
  const std::shared_ptr<std::atomic<bool>> fail_once(
      new std::atomic<bool>(true));
  TubeWorkerHooksV2 hooks;
  hooks.after_build = [fail_once](const TubeWorkerRequestV2&,
                                  const TubeWorkerCompletionV2&) {
    if (fail_once->exchange(false)) {
      throw std::runtime_error("deterministic test callback failure");
    }
  };
  PhaseOffsetTubeWorkerV2 worker(config, hooks);
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  TubeWorkerCompletionV2 failed;
  ASSERT_TRUE(worker.tryTakeAny(failed));
  EXPECT_FALSE(failed.built());
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 2U)));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 2U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 2U; }));
  TubeWorkerCompletionV2 retried;
  ASSERT_TRUE(worker.tryTakeAny(retried));
  EXPECT_TRUE(retried.built());
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, ResetCancelsAndDiscardsStaleCompletion) {
  const TubeCertificateConfigV2 config = Config();
  auto gate = std::make_shared<Gate>();
  PhaseOffsetTubeWorkerV2 worker(config, Hooks(gate, true));
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
  ASSERT_TRUE(WaitForGate(gate, false));
  ASSERT_TRUE(worker.reset(2U));
  {
    std::lock_guard<std::mutex> lock(gate->mutex);
    gate->release = true;
  }
  gate->condition.notify_all();
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().stale_discarded >= 1U; }));
  TubeWorkerCompletionV2 completion;
  EXPECT_FALSE(worker.tryTakeAny(completion));
  EXPECT_FALSE(worker.reset(2U));
  EXPECT_FALSE(worker.reset(1U));
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, ExactPathInvalidationAndGenerationScope) {
  const TubeCertificateConfigV2 config = Config();
  PhaseOffsetTubeWorkerV2 worker(config);
  TubeWorkerRequestV2 request = Request(config, TubeWorkerPurposeV2::CURRENT, 1U);
  ASSERT_TRUE(worker.submit(request));
  TubePathKey different = request.input.path_key;
  different.frame_revision += 1U;
  EXPECT_TRUE(worker.invalidatePath(different));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTake(TubeWorkerPurposeV2::CURRENT, completion));
  // The different frame key did not invalidate the submitted path.
  EXPECT_TRUE(completion.built());
  const TubePathKey old_generation = request.input.path_key;
  ASSERT_TRUE(worker.reset(2U));
  TubeWorkerRequestV2 fresh = Request(config, TubeWorkerPurposeV2::CURRENT,
                                      1U, 2U);
  ASSERT_TRUE(worker.submit(fresh));
  EXPECT_FALSE(worker.invalidatePath(old_generation));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 2U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 2U; }));
  ASSERT_TRUE(worker.tryTake(TubeWorkerPurposeV2::CURRENT, completion));
  EXPECT_TRUE(completion.built());
  EXPECT_FALSE(worker.invalidatePath(old_generation));
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, MonotoneGenerationAndInvalidPurposeBoundaries) {
  const TubeCertificateConfigV2 config = Config();
  PhaseOffsetTubeWorkerV2 worker(config);
  TubeWorkerRequestV2 request = Request(config, TubeWorkerPurposeV2::CURRENT, 1U);
  ASSERT_TRUE(worker.submit(request));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  ASSERT_TRUE(worker.reset(2U));
  EXPECT_FALSE(worker.reset(2U));
  EXPECT_FALSE(worker.reset(1U));
  EXPECT_FALSE(worker.cancel(static_cast<TubeWorkerPurposeV2>(-1)));
  TubeWorkerCompletionV2 completion;
  EXPECT_FALSE(worker.tryTake(static_cast<TubeWorkerPurposeV2>(-1), completion));
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, ShutdownJoinAndOwnerLifetime) {
  const TubeCertificateConfigV2 config = Config();
  std::shared_ptr<const AnalyticPathOwner> owner;
  TubeWorkerRequestV2 request = Request(config, TubeWorkerPurposeV2::CURRENT,
                                        1U, 1U, 0.0, 1.0, &owner);
  const std::weak_ptr<const AnalyticPathOwner> weak_owner(owner);
  PhaseOffsetTubeWorkerV2 worker(config);
  ASSERT_TRUE(worker.submit(request));
  request = TubeWorkerRequestV2();
  owner.reset();
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTakeAny(completion));
  worker.shutdown();
  EXPECT_TRUE(worker.joined());
  EXPECT_FALSE(worker.running());
  completion = TubeWorkerCompletionV2();
  EXPECT_TRUE(weak_owner.expired());
  EXPECT_FALSE(worker.tryTakeAny(completion));
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, ConcurrentShutdownSafety) {
  const TubeCertificateConfigV2 config = Config();
  PhaseOffsetTubeWorkerV2 worker(config);
  ASSERT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
  std::thread first([&worker]() { worker.shutdown(); });
  std::thread second([&worker]() { worker.shutdown(); });
  first.join();
  second.join();
  EXPECT_TRUE(worker.joined());
}

TEST(PhaseOffsetTubeWorkerV2Test, EventRingBoundedAndDropCountReported) {
  const TubeCertificateConfigV2 config = Config();
  PhaseOffsetTubeWorkerV2 worker(config);
  for (std::uint64_t id = 1U; id <= 700U; ++id) {
    TubeWorkerRequestV2 request = Request(
        config, TubeWorkerPurposeV2::CURRENT, id, 1U, 0.0, 0.1);
    worker.submit(request);
  }
  worker.shutdown();
  const std::vector<TubeWorkerEventV2> events = worker.events();
  EXPECT_LE(events.size(), 512U);
  EXPECT_GT(worker.stats().event_log_dropped, 0U);
}

TEST(PhaseOffsetTubeWorkerV2Test, DeterministicReplayAndTimingFields) {
  const TubeCertificateConfigV2 config = Config();
  auto run_once = [&config]() {
    PhaseOffsetTubeWorkerV2 worker(config);
    EXPECT_TRUE(worker.submit(Request(config, TubeWorkerPurposeV2::CURRENT, 1U)));
    EXPECT_TRUE(WaitFor([&worker]() {
      return worker.stats().build_finished >= 1U;
    }));
    EXPECT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
    TubeWorkerCompletionV2 completion;
    EXPECT_TRUE(worker.tryTakeAny(completion));
    EXPECT_TRUE(completion.built());
    EXPECT_GE(completion.build_end_ticks, completion.build_start_ticks);
    EXPECT_GE(completion.path_callback_duration_ns, 0U);
    const std::vector<TubeWorkerEventV2> events = worker.events();
    EXPECT_FALSE(events.empty());
    worker.shutdown();
    return std::make_pair(completion.path_callback_count,
                          completion.free_ball_callback_count);
  };
  const std::pair<std::uint64_t, std::uint64_t> first = run_once();
  const std::pair<std::uint64_t, std::uint64_t> second = run_once();
  EXPECT_EQ(first, second);
}

TEST(PhaseOffsetTubeWorkerV2Test,
     AcceptedStateDemandRemainsBoundToCompletionIdentity) {
  const TubeCertificateConfigV2 config = Config();
  PhaseOffsetTubeWorkerV2 worker(config);
  TubeWorkerRequestV2 request =
      Request(config, TubeWorkerPurposeV2::CURRENT, 901U);
  request.accepted_state_demand = request.input.map_capture_key.accepted_sequence;
  ASSERT_TRUE(worker.submit(request));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTakeAny(completion));
  EXPECT_EQ(completion.accepted_state_demand,
            request.input.map_capture_key.accepted_sequence);
  EXPECT_EQ(completion.map_capture_key, request.input.map_capture_key);
  EXPECT_EQ(completion.path_key, request.input.path_key);
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test,
     LiveAnchorRefreshDoesNotRestartRetainedUsefulCurrentProof) {
  const TubeCertificateConfigV2 config = Config();
  PhaseOffsetTubeWorkerV2 worker(config);
  ASSERT_TRUE(worker.submit(
      Request(config, TubeWorkerPurposeV2::CURRENT, 902U, 1U, 0.0, 1.0)));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 1U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTakeAny(completion));
  ASSERT_TRUE(completion.built());

  // A live phase/raw-cloud notification is represented only by a changed
  // immutable anchor here; it does not invalidate the retained useful
  // completion or launch a second geometry/map proof.
  TubeWorkerRequestV2 refresh =
      Request(config, TubeWorkerPurposeV2::CURRENT, 903U, 1U, 0.0, 1.0);
  refresh.input.anchor_w = 0.75;
  EXPECT_FALSE(worker.submit(refresh));
  EXPECT_EQ(worker.stats().build_started, 1U);
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test,
     AcceptedMapRefreshCoalescesAndRetainsWiderOlderCompletion) {
  const TubeCertificateConfigV2 config = Config();
  auto gate = std::make_shared<Gate>();
  PhaseOffsetTubeWorkerV2 worker(config, Hooks(gate, true));

  TubeWorkerRequestV2 initial =
      Request(config, TubeWorkerPurposeV2::CURRENT, 1001U, 1U, 0.0, 1.0);
  ASSERT_TRUE(worker.submit(initial));
  ASSERT_TRUE(WaitForGate(gate, false));

  const auto refresh_request = [&config](const std::uint64_t id,
                                          const std::uint64_t state_id,
                                          const std::uint64_t accepted,
                                          const std::uint64_t accepted_time,
                                          const double end) {
    TubeWorkerRequestV2 request = Request(
        config, TubeWorkerPurposeV2::CURRENT, id, 1U, 0.0, end);
    request.accepted_state_demand = accepted;
    request.input.map_capture_key.state_id = state_id;
    request.input.map_capture_key.accepted_sequence = accepted;
    request.input.map_capture_key.accepted_time_ticks = accepted_time;
    request.input.applicability_deadline_ticks = 20U;
    request.input.free_ball_query = OpenQuery(request.input.map_capture_key);
    return request;
  };

  // The accepted map advances twice while the first profile is running.  The
  // second refresh is intentionally narrower: it supersedes the pending old
  // capture, but may not erase the wider useful completion once delivered.
  TubeWorkerRequestV2 refresh =
      refresh_request(1002U, 6U, 6U, 11U, 0.8);
  ASSERT_TRUE(worker.submit(refresh));
  TubeWorkerRequestV2 latest =
      refresh_request(1003U, 7U, 7U, 12U, 0.4);
  ASSERT_TRUE(worker.submit(latest));
  TubeWorkerRequestV2 rollback =
      refresh_request(1004U, 6U, 6U, 11U, 0.4);
  EXPECT_FALSE(worker.submit(rollback));
  TubeWorkerRequestV2 reused_id =
      refresh_request(1003U, 8U, 8U, 13U, 0.4);
  EXPECT_FALSE(worker.submit(reused_id));

  {
    std::lock_guard<std::mutex> lock(gate->mutex);
    gate->release = true;
  }
  gate->condition.notify_all();
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().build_finished >= 2U; }));
  ASSERT_TRUE(WaitFor([&worker]() { return worker.stats().delivery_count >= 1U; }));
  EXPECT_GE(worker.stats().coalesced, 1U);
  EXPECT_EQ(worker.stats().build_started, 2U);

  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTake(TubeWorkerPurposeV2::CURRENT, completion));
  ASSERT_TRUE(completion.built());
  EXPECT_EQ(completion.request_id, initial.request_id);
  EXPECT_EQ(completion.map_capture_key, initial.input.map_capture_key);
  worker.shutdown();
}

TEST(PhaseOffsetTubeWorkerV2Test, NewMapForwardWindowReplacesUndeliveredOldPrefix) {
  const auto config = Config();
  PhaseOffsetTubeWorkerV2 worker(config);
  auto old = Request(config, TubeWorkerPurposeV2::CURRENT, 1U, 1U, 0.0, 0.6);
  ASSERT_TRUE(worker.submit(old));
  ASSERT_TRUE(WaitFor([&]() { return worker.stats().delivery_count >= 1U; }));
  auto next = Request(config, TubeWorkerPurposeV2::CURRENT, 2U, 1U, 0.5, 1.0);
  ++next.input.map_capture_key.state_id;
  ++next.input.map_capture_key.accepted_sequence;
  ++next.accepted_state_demand;
  next.input.free_ball_query = OpenQuery(next.input.map_capture_key);
  ASSERT_TRUE(worker.submit(next));
  ASSERT_TRUE(WaitFor([&]() { return worker.stats().build_finished >= 2U; }));
  ASSERT_TRUE(WaitFor([&]() { return worker.stats().delivery_count >= 2U; }));
  TubeWorkerCompletionV2 completion;
  ASSERT_TRUE(worker.tryTake(TubeWorkerPurposeV2::CURRENT, completion));
  ASSERT_TRUE(completion.built());
  EXPECT_EQ(completion.request_id, next.request_id);
  EXPECT_EQ(completion.map_capture_key, next.input.map_capture_key);
  EXPECT_GE(completion.useful_start, 0.5);
  EXPECT_DOUBLE_EQ(completion.useful_end, 1.0);
  worker.shutdown();
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
