#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <phase_offset_core/path_state.h>

namespace phase_offset_navigation {

// These value keys are deliberately independent of the legacy epoch/profile
// objects.  A V2 profile is useful only with the exact immutable authorities
// named by all three keys.
struct TubePathKey {
  std::uint64_t execution_generation = 0U;
  std::uint64_t path_instance_id = 0U;
  std::uint64_t path_revision = 0U;
  std::uint64_t frame_revision = 0U;
  std::uint64_t frame_convention_id = 0U;
  std::string frame_convention;
  // +1 and -1 are the only supported native phase orientations.
  int phase_orientation = 0;
  double domain_start = 0.0;
  double domain_end = 0.0;

  bool complete() const;
  bool operator==(const TubePathKey& other) const;
  bool operator!=(const TubePathKey& other) const {
    return !(*this == other);
  }
};

struct TubeConfigurationKey {
  std::uint64_t configuration_id = 0U;
  double epsilon = 0.0;
  double nominal_half_width = 0.0;
  double ray_step = 0.0;
  double snapshot_resolution = 0.0;
  double minimum_reference_speed = 0.0;

  bool complete() const;
  bool operator==(const TubeConfigurationKey& other) const;
  bool operator!=(const TubeConfigurationKey& other) const {
    return !(*this == other);
  }
};

struct TubeMapCaptureKey {
  // state_id alone is not an identity: map copies can reuse an observation
  // sequence.  Every field below is captured from the same accepted state.
  std::uint64_t map_instance_id = 0U;
  std::uint64_t state_id = 0U;
  std::uint64_t accepted_sequence = 0U;
  std::uint64_t configuration_generation = 0U;
  std::uint64_t configuration_id = 0U;
  std::uint64_t frame_provenance_id = 0U;
  std::string frame_provenance;
  std::uint64_t support_provenance_id = 0U;
  std::uint64_t accepted_time_ticks = 0U;
  std::uint64_t support_expiry_ticks = 0U;
  bool support_expiry_timeless = false;
  double support_halo = 0.0;
  bool halo_reconciled = false;
  std::int64_t grid_min_index_x = 0;
  std::int64_t grid_min_index_y = 0;
  std::int64_t grid_min_index_z = 0;
  std::int64_t grid_max_index_x = 0;
  std::int64_t grid_max_index_y = 0;
  std::int64_t grid_max_index_z = 0;
  Eigen::Vector3d grid_native_origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d grid_voxel_resolution = Eigen::Vector3d::Zero();
  std::int64_t grid_source_offset_x = 0;
  std::int64_t grid_source_offset_y = 0;
  std::int64_t grid_source_offset_z = 0;
  bool grid_native_index = true;
  bool complete_support = false;

  bool complete() const;
  bool operator==(const TubeMapCaptureKey& other) const;
  bool operator!=(const TubeMapCaptureKey& other) const {
    return !(*this == other);
  }
};

enum class TubeCertificateFailureReason {
  NONE,
  INVALID_INPUT,
  UNAVAILABLE_PRODUCER,
  MALFORMED_PATH_CELL,
  PATH_KEY_MISMATCH,
  CONFIGURATION_MISMATCH,
  MAP_CAPTURE_MISMATCH,
  UNSUPPORTED_FP_MODE,
  NONFINITE_INTERVAL,
  INTERVAL_OVERFLOW,
  DENOMINATOR_CROSSES_ZERO,
  NORMAL_FLOOR,
  REGULARITY,
  MOTION_COVER,
  ZERO_ANCHOR,
  CLEARANCE_UNAVAILABLE,
  CLEARANCE_OUT_OF_MAP,
  CLEARANCE_UNKNOWN,
  CLEARANCE_OCCUPIED,
  CLEARANCE_UNCERTIFIED,
  CLEARANCE_INSUFFICIENT,
  STRUCTURAL_GAP,
  WITNESS_FAILURE,
  SUBDIVISION_DEPTH,
  QUERY_BUDGET,
  CELL_BUDGET,
  WITNESS_BUDGET,
  SAMPLE_BUDGET,
  COMPONENT_GAP,
  PWL_MALFORMED,
};

enum class TubeCertificateTruncation {
  NONE,
  PREFIX,
  SUFFIX,
  PREFIX_AND_SUFFIX,
  ANCHOR_EXCLUDED,
  NO_CERTIFIED_COMPONENT,
  BUDGET,
  DEPTH,
};

enum class TubeProfileV2Capability {
  UNAVAILABLE,
  ZERO_ONLY,
  OFFSET_CERTIFIED,
};

struct TubeFailureAttribution {
  TubeCertificateFailureReason reason = TubeCertificateFailureReason::NONE;
  TubeCertificateTruncation truncation = TubeCertificateTruncation::NONE;
  double w = 0.0;
  double w0 = 0.0;
  double w1 = 0.0;
  double delta = 0.0;
  std::size_t query_count = 0U;
  int depth = 0;
  std::string detail;
};

struct TubeVoxelFootprintV2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Inclusive native/crop index range returned by the authoritative map
  // enumerator.  This is compact evidence, not one copied record per voxel.
  std::int64_t min_index_x = 0;
  std::int64_t min_index_y = 0;
  std::int64_t min_index_z = 0;
  std::int64_t max_index_x = 0;
  std::int64_t max_index_y = 0;
  std::int64_t max_index_z = 0;
  Eigen::Vector3d native_origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d voxel_resolution = Eigen::Vector3d::Zero();
  std::int64_t source_offset_x = 0;
  std::int64_t source_offset_y = 0;
  std::int64_t source_offset_z = 0;
  bool native_index = true;
  bool valid = false;
};

struct TubeSupportFootprintV2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d lower = Eigen::Vector3d::Zero();
  Eigen::Vector3d upper = Eigen::Vector3d::Zero();
  Eigen::Vector3d witness = Eigen::Vector3d::Zero();
  double radius = 0.0;
  std::uint64_t map_instance_id = 0U;
  std::uint64_t map_state_id = 0U;
  std::uint64_t support_provenance_id = 0U;
  std::uint64_t accepted_time_ticks = 0U;
  std::uint64_t support_expiry_ticks = 0U;
  bool support_expiry_timeless = false;
  bool valid = false;
  // The ball AABB is not a replacement for the producer's native/crop
  // enumeration.  Retain the exact closed footprint used to establish the
  // support so a later consumer can audit applicability without a map read.
  std::vector<TubeVoxelFootprintV2,
              Eigen::aligned_allocator<TubeVoxelFootprintV2>> voxel_footprint;
};

struct TubeDirectedRatioV2 {
  double lower = 0.0;
  double upper = 0.0;
  bool valid = false;
};

struct TubePwlKnotV2 {
  double w = 0.0;
  double lower = 0.0;
  double upper = 0.0;
  // Slopes are one-sided: left_* belongs to [previous,w], right_* to
  // [w,next].  At an outer knot the absent side remains zero.
  double left_lower_slope = 0.0;
  double left_upper_slope = 0.0;
  double right_lower_slope = 0.0;
  double right_upper_slope = 0.0;
  TubeDirectedRatioV2 left_lower_slope_interval;
  TubeDirectedRatioV2 left_upper_slope_interval;
  TubeDirectedRatioV2 right_lower_slope_interval;
  TubeDirectedRatioV2 right_upper_slope_interval;
  std::uint64_t left_cell_id = 0U;
  std::uint64_t right_cell_id = 0U;
  bool valid = false;
};

struct TubeProofCellV2 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double w0 = 0.0;
  double w1 = 0.0;
  double lower = 0.0;
  double upper = 0.0;
  double q_min = 0.0;
  double p_bound = 0.0;
  double a_xy = 0.0;
  double q_regular = 0.0;
  double regularity_cap = 0.0;
  double motion_cover = 0.0;
  // Deterministic identity assigned by the V2 builder to this exact
  // visited/retained build cell.  This is deliberately distinct from the
  // producer's segment_identity, which may be shared by subdivisions.
  std::uint64_t cell_id = 0U;
  std::uint64_t segment_identity = 0U;
  std::uint64_t proof_identity = 0U;
  int depth = 0;
  bool zero_anchor_certified = false;
  bool complete = false;
  bool valid = false;
  phase_offset_core::CertifiedPathCellV2 path_cell;
  std::vector<TubeSupportFootprintV2> supports;
};

// Immutable after TubeCertificateBuilderV2::build returns.  Geometry and
// provenance are value-owned; type-erased shared owners retain the exact
// immutable path/query/capture lifetimes without exposing ROS/map types or
// mutable navigation state.
struct TubeProfileV2 {
  TubePathKey path_key;
  TubeConfigurationKey configuration_key;
  TubeMapCaptureKey map_capture_key;
  std::uint64_t profile_id = 0U;
  std::uint64_t request_id = 0U;
  double requested_start = 0.0;
  double requested_end = 0.0;
  double anchor_w = 0.0;
  double certified_start = 0.0;
  double certified_end = 0.0;
  bool valid = false;
  bool complete = false;
  bool contains_anchor = false;
  bool contains_zero_everywhere = false;
  bool nonzero_capacity = false;
  TubeProfileV2Capability capability = TubeProfileV2Capability::UNAVAILABLE;
  TubeCertificateFailureReason failure_reason =
      TubeCertificateFailureReason::NONE;
  TubeCertificateTruncation truncation = TubeCertificateTruncation::NONE;
  TubeFailureAttribution failure;
  // Type-erased shared ownership keeps the exact immutable capture/query
  // alive while keeping this pure navigation DTO free of ROS/SDFMap types.
  std::shared_ptr<const void> path_owner;
  std::shared_ptr<const void> capture_owner;
  std::shared_ptr<const void> query_owner;
  std::string applicability_assumptions;
  std::uint64_t applicability_deadline_ticks = 0U;
  bool applicability_deadline_timeless = false;
  struct BuildCounters {
    std::size_t path_cell_query_count = 0U;
    std::size_t failed_path_cell_query_count = 0U;
    std::size_t query_count = 0U;
    std::size_t failed_query_count = 0U;
    std::size_t child_query_count = 0U;
    std::size_t cell_count = 0U;
    std::size_t scheduled_cell_count = 0U;
    std::size_t accepted_cell_count = 0U;
    std::size_t witness_count = 0U;
    int max_depth_observed = 0;
    bool query_budget_reached = false;
    bool cell_budget_reached = false;
    bool witness_budget_reached = false;
    bool sample_budget_reached = false;
  } counters;
  std::vector<TubePwlKnotV2> knots;
  std::vector<TubeProofCellV2> cells;

  bool evaluate(double w, double& lower, double& upper) const;
  bool contains(double w, double delta) const;
  bool structurallyValid() const;
};

const char* tubeCertificateFailureReasonName(
    TubeCertificateFailureReason reason);
const char* tubeCertificateTruncationName(TubeCertificateTruncation outcome);

}  // namespace phase_offset_navigation
