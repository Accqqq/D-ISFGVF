#pragma once

#include "phase_offset_navigation/distance_query.h"
#include "phase_offset_navigation/tube_profile_v2.h"

#include <Eigen/StdVector>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace phase_offset_navigation {

// This is the only environmental authority accepted by the V2 proof.  It is
// intentionally a new DTO rather than a reinterpretation of the legacy
// center-distance query: a successful answer must identify complete support
// for the whole requested closed ball and the immutable map state that
// supplied it.
struct TubeFreeBallQueryResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DistanceStatus status = DistanceStatus::UNAVAILABLE;
  double certified_radius = 0.0;
  bool certified = false;
  bool exact = false;
  bool complete_support = false;
  std::uint64_t map_instance_id = 0U;
  std::uint64_t map_state_id = 0U;
  std::uint64_t configuration_id = 0U;
  std::uint64_t frame_provenance_id = 0U;
  std::string frame_provenance;
  std::uint64_t accepted_sequence = 0U;
  std::uint64_t configuration_generation = 0U;
  std::uint64_t support_provenance_id = 0U;
  std::uint64_t accepted_time_ticks = 0U;
  std::uint64_t support_expiry_ticks = 0U;
  bool support_expiry_timeless = false;
  bool halo_reconciled = false;
  TubeSupportFootprintV2 support;
  std::string provenance;
};

using TubeFreeBallQuery = std::function<TubeFreeBallQueryResult(
    const Eigen::Vector3d&, double requested_radius)>;
using AuthoritativeFreeBallQuery = TubeFreeBallQuery;
using TubePathCellQueryV2 = std::function<bool(
    double, double, phase_offset_core::CertifiedPathCellV2&)>;

using CertifiedPathCellsV2 = std::vector<
    phase_offset_core::CertifiedPathCellV2,
    Eigen::aligned_allocator<phase_offset_core::CertifiedPathCellV2>>;

struct TubeCertificateBudgetsV2 {
  int max_w_depth = 12;
  std::size_t max_queries = 250000U;
  std::size_t max_cells = 250000U;
  // This is the captured per-cross-section sample-count ceiling.  Aggregate
  // retained witnesses share the query/cell budget and are not silently
  // reduced to this per-cell value.
  std::size_t max_samples_per_cell = 100000U;
  std::size_t max_witnesses = 250000U;
};

struct TubeCertificateConfigV2 {
  // epsilon is the unchanged planner safe-distance authority.  A nonpositive
  // value disables V2 rather than becoming an implicit default.
  std::uint64_t configuration_id = 0U;
  double epsilon = 0.0;
  double nominal_half_width = 1.0;
  double ray_step = 0.05;
  double snapshot_resolution = 0.0;
  double minimum_reference_speed = 1e-8;
  double sample_step_w = 0.1;
  TubeCertificateBudgetsV2 budgets;

  bool complete() const;
  TubeConfigurationKey key() const;
};

struct TubeBuildInputV2 {
  std::uint64_t request_id = 0U;
  TubePathKey path_key;
  TubeConfigurationKey configuration_key;
  TubeMapCaptureKey map_capture_key;

  // The requested interval and useful anchor are copied into the worker job;
  // no current delta, previous port, live map, or lifecycle state is allowed.
  double requested_start = 0.0;
  double requested_end = 0.0;
  double anchor_w = 0.0;

  CertifiedPathCellsV2 path_cells;
  // Structural producer breakpoints are identity-bearing.  The builder only
  // removes exact duplicates; near-equal values remain distinct.
  std::vector<double> producer_breakpoints;
  std::vector<double> sample_grid;
  // The authoritative producer callback must return a certificate for the
  // exact requested child interval and is tied to path_key by its owner.  The
  // value vector remains useful for immutable fixtures and unsplit cells.
  TubePathCellQueryV2 path_cell_query;
  std::function<bool(std::vector<double>&)> producer_breakpoint_query;
  std::shared_ptr<const void> path_owner;
  std::shared_ptr<const void> query_owner;
  std::shared_ptr<const void> capture_owner;
  std::string applicability_assumptions;
  std::uint64_t applicability_deadline_ticks = 0U;
  bool applicability_deadline_timeless = false;
  TubeFreeBallQuery free_ball_query;

  bool complete() const;
};

struct TubeCertificateBuildStatsV2 {
  std::size_t path_cell_query_count = 0U;
  std::size_t failed_path_cell_query_count = 0U;
  std::size_t query_count = 0U;
  std::size_t failed_query_count = 0U;
  std::size_t child_query_count = 0U;
  std::size_t cell_count = 0U;
  // Number of unique work cells admitted to the deterministic queue/tree,
  // including scheduled children that may remain unvisited after a budget
  // stop.  cell_count is the corresponding visited count.
  std::size_t scheduled_cell_count = 0U;
  std::size_t accepted_cell_count = 0U;
  std::size_t witness_count = 0U;
  int max_depth_observed = 0;
  bool query_budget_reached = false;
  bool cell_budget_reached = false;
  bool witness_budget_reached = false;
  bool sample_budget_reached = false;
};

struct TubeCertificateBuildResultV2 {
  TubeProfileV2 profile;
  TubeCertificateBuildStatsV2 stats;
  bool success = false;
};

class TubeCertificateBuilderV2 {
 public:
  explicit TubeCertificateBuilderV2(
      const TubeCertificateConfigV2& config = TubeCertificateConfigV2());

  bool configurationValid() const;
  const TubeCertificateConfigV2& config() const { return config_; }

  bool build(const TubeBuildInputV2& input,
             TubeCertificateBuildResultV2& result) const;
  bool build(const TubeBuildInputV2& input, TubeProfileV2& profile) const;

 private:
  TubeCertificateConfigV2 config_;
};

// Naming aliases make the value contract convenient for callers without
// introducing a second implementation or a legacy compatibility path.
using TubeCertificateInputV2 = TubeBuildInputV2;
using TubeCertificateOutputV2 = TubeCertificateBuildResultV2;

}  // namespace phase_offset_navigation
