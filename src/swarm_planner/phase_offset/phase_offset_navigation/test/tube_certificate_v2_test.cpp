#include "phase_offset_navigation/tube_certificate_v2.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <boost/multiprecision/cpp_int.hpp>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include <cfenv>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if defined(__i386__) || defined(__x86_64__)
#include <immintrin.h>
#endif

namespace phase_offset_navigation {
namespace {

std::shared_ptr<const void> Owner() {
  return std::static_pointer_cast<const void>(std::make_shared<int>(7));
}

void SetInterval(phase_offset_core::Binary64Interval& value,
                 const double lower, const double upper) {
  value.lower = lower;
  value.upper = upper;
  value.valid = true;
}

void SetVectorInterval(phase_offset_core::Binary64VectorInterval& value,
                       const Eigen::Vector3d& lower,
                       const Eigen::Vector3d& upper) {
  value.valid = true;
  for (std::size_t i = 0U; i < 3U; ++i) {
    SetInterval(value.component[i], lower(static_cast<Eigen::Index>(i)),
                upper(static_cast<Eigen::Index>(i)));
  }
}

struct Dyadic {
  boost::multiprecision::cpp_int numerator = 0;
  int exponent = 0;
};

Dyadic Decode(const double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  const bool negative = (bits >> 63U) != 0U;
  const std::uint64_t fraction = bits & ((std::uint64_t(1) << 52U) - 1U);
  const std::uint64_t exponent_bits = (bits >> 52U) & 0x7ffU;
  Dyadic result;
  if (exponent_bits == 0U) {
    result.numerator = fraction;
    result.exponent = -1074;
  } else {
    result.numerator = (std::uint64_t(1) << 52U) | fraction;
    result.exponent = static_cast<int>(exponent_bits) - 1023 - 52;
  }
  if (negative) result.numerator = -result.numerator;
  return result;
}

Dyadic Negate(const Dyadic& value) {
  Dyadic result = value;
  result.numerator = -result.numerator;
  return result;
}

Dyadic Add(const Dyadic& lhs, const Dyadic& rhs) {
  Dyadic result;
  result.exponent = std::min(lhs.exponent, rhs.exponent);
  result.numerator = (lhs.numerator <<
      static_cast<unsigned>(lhs.exponent - result.exponent)) +
      (rhs.numerator << static_cast<unsigned>(rhs.exponent - result.exponent));
  return result;
}

Dyadic Subtract(const Dyadic& lhs, const Dyadic& rhs) {
  return Add(lhs, Negate(rhs));
}

Dyadic Multiply(const Dyadic& lhs, const Dyadic& rhs) {
  Dyadic result;
  result.numerator = lhs.numerator * rhs.numerator;
  result.exponent = lhs.exponent + rhs.exponent;
  return result;
}

int Compare(const Dyadic& lhs, const Dyadic& rhs) {
  const Dyadic difference = Add(lhs, Negate(rhs));
  return difference.numerator == 0 ? 0 : (difference.numerator < 0 ? -1 : 1);
}

std::pair<double, double> Outward(const Dyadic& value) {
  using Big = boost::multiprecision::cpp_dec_float_100;
  Big exact = Big(value.numerator.convert_to<std::string>());
  if (value.exponent >= 0) {
    for (int i = 0; i < value.exponent; ++i) exact *= 2;
  } else {
    for (int i = 0; i < -value.exponent; ++i) exact /= 2;
  }
  double estimate = exact.convert_to<double>();
  double lower = estimate;
  double upper = estimate;
  while (Compare(Decode(lower), value) > 0) {
    lower = std::nextafter(lower, -std::numeric_limits<double>::infinity());
  }
  while (Compare(Decode(upper), value) < 0) {
    upper = std::nextafter(upper, std::numeric_limits<double>::infinity());
  }
  return std::make_pair(lower, upper);
}

phase_offset_core::CertifiedPathCellV2 Cell(const double w0,
                                             const double w1,
                                             const double x = 0.0,
                                             const double speed = 1.0,
                                             const double horizontal_accel = 0.0,
                                             const double horizontal_floor = 1.0) {
  phase_offset_core::CertifiedPathCellV2 cell;
  cell.w0 = w0;
  cell.w1 = w1;
  cell.anchor_w = w0 + (w1 - w0) * 0.5;
  cell.path_revision = 7U;
  cell.frame_revision = 7U;
  cell.segment_identity = 11U;
  cell.proof_identity = 111U;
  const Dyadic exact_midpoint = Multiply(
      Add(Decode(w0), Decode(w1)), Decode(0.5));
  const Dyadic exact_anchor_x = Add(
      Decode(x), Multiply(Decode(speed), exact_midpoint));
  const std::pair<double, double> anchor_bounds = Outward(exact_anchor_x);
  SetVectorInterval(cell.anchor_position,
                    Eigen::Vector3d(anchor_bounds.first, 0.0, 0.0),
                    Eigen::Vector3d(anchor_bounds.second, 0.0, 0.0));
  SetVectorInterval(cell.anchor_p_w, Eigen::Vector3d(speed, 0.0, 0.0),
                    Eigen::Vector3d(speed, 0.0, 0.0));
  SetVectorInterval(cell.anchor_p_ww, Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::Zero());
  SetInterval(cell.inf_p_w_norm, speed, speed);
  SetInterval(cell.sup_p_w_norm, speed, speed);
  SetInterval(cell.inf_horizontal_p_w_norm, horizontal_floor,
              horizontal_floor);
  SetInterval(cell.sup_p_ww_norm, horizontal_accel, horizontal_accel);
  SetInterval(cell.sup_horizontal_p_ww_norm, horizontal_accel,
              horizontal_accel);
  SetInterval(cell.sup_p_www_norm, 0.0, 0.0);
  SetInterval(cell.sup_normal_derivative, 0.0, 0.0);
  SetInterval(cell.normal_variation, 0.0, 0.0);
  SetInterval(cell.tangent_variation, 0.0, 0.0);
  SetInterval(cell.curvature_variation, 0.0, 0.0);
  const Dyadic exact_variation = Multiply(
      Multiply(Decode(speed), Subtract(Decode(w1), Decode(w0))),
      Decode(0.5));
  const double variation = Outward(exact_variation).second;
  SetInterval(cell.midpoint_position_variation, 0.0, variation);
  SetInterval(cell.chord_deviation, 0.0, 0.0);
  cell.horizontal_acceleration_bound_complete = true;
  cell.normal_frame_proof_complete = true;
  cell.phase_map_proof_complete = true;
  cell.provenance = phase_offset_core::kWorldHorizontalCrossProductProvenance;
  cell.valid = true;
  cell.complete = true;
  return cell;
}

phase_offset_core::CertifiedPathCellV2 RotatingPolynomialCell(
    const double w0, const double w1) {
  phase_offset_core::CertifiedPathCellV2 cell = Cell(w0, w1);
  const Dyadic midpoint = Multiply(
      Add(Decode(w0), Decode(w1)), Decode(0.5));
  const Dyadic t = Subtract(midpoint, Decode(0.1));
  const std::pair<double, double> px = Outward(t);
  const std::pair<double, double> py = Outward(
      Multiply(Multiply(t, t), Decode(0.5)));
  SetVectorInterval(cell.anchor_position,
                    Eigen::Vector3d(px.first, py.first, 0.0),
                    Eigen::Vector3d(px.second, py.second, 0.0));
  const std::pair<double, double> py_w = Outward(t);
  SetVectorInterval(cell.anchor_p_w,
                    Eigen::Vector3d(1.0, py_w.first, 0.0),
                    Eigen::Vector3d(1.0, py_w.second, 0.0));
  SetVectorInterval(cell.anchor_p_ww, Eigen::Vector3d(0.0, 1.0, 0.0),
                    Eigen::Vector3d(0.0, 1.0, 0.0));
  SetInterval(cell.inf_p_w_norm, 1.0, 1.0);
  SetInterval(cell.sup_p_w_norm, 1.0, 1.01);
  SetInterval(cell.inf_horizontal_p_w_norm, 1.0, 1.0);
  SetInterval(cell.sup_p_ww_norm, 1.0, 1.0);
  SetInterval(cell.sup_horizontal_p_ww_norm, 1.0, 1.0);
  SetInterval(cell.sup_p_www_norm, 0.0, 0.0);
  SetInterval(cell.sup_normal_derivative, 2.0, 2.0);
  SetInterval(cell.normal_variation, 0.0, 0.5);
  SetInterval(cell.tangent_variation, 0.0, 0.5);
  SetInterval(cell.curvature_variation, 0.0, 1.0);
  const Dyadic width = Subtract(Decode(w1), Decode(w0));
  const double midpoint_variation =
      Outward(Multiply(width, Decode(1.01))).second;
  const double chord_deviation = Outward(
      Multiply(Multiply(width, width), Decode(0.125))).second;
  SetInterval(cell.midpoint_position_variation, 0.0, midpoint_variation);
  SetInterval(cell.chord_deviation, 0.0, chord_deviation);
  cell.segment_identity = static_cast<std::uint64_t>(w0 * 1000.0 + 1001.0);
  cell.proof_identity = cell.segment_identity + 10000U;
  return cell;
}

TubeCertificateConfigV2 Config() {
  TubeCertificateConfigV2 config;
  config.configuration_id = 17U;
  config.epsilon = 0.1;
  config.nominal_half_width = 0.2;
  config.ray_step = 0.1;
  config.snapshot_resolution = 0.2;
  config.minimum_reference_speed = 0.5;
  config.sample_step_w = 1.0;
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
  // The map frame is independent from the path's fixed horizontal-normal
  // convention.  Production captures normally use the configured ROS frame.
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
  key.grid_source_offset_x = 0;
  key.grid_source_offset_y = 0;
  key.grid_source_offset_z = 0;
  key.complete_support = true;
  return key;
}

TubeSupportFootprintV2 Support(const TubeMapCaptureKey& key,
                               const Eigen::Vector3d& witness,
                               const double radius) {
  TubeSupportFootprintV2 support;
  support.lower = (witness.array() - 2.0).matrix();
  support.upper = (witness.array() + 2.0).matrix();
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
  voxel.min_index_x = -100;
  voxel.min_index_y = -100;
  voxel.min_index_z = -100;
  voxel.max_index_x = 100;
  voxel.max_index_y = 100;
  voxel.max_index_z = 100;
  voxel.native_origin = Eigen::Vector3d::Zero();
  voxel.voxel_resolution = Eigen::Vector3d::Constant(0.2);
  voxel.source_offset_x = 0;
  voxel.source_offset_y = 0;
  voxel.source_offset_z = 0;
  voxel.native_index = true;
  voxel.valid = true;
  support.voxel_footprint.push_back(voxel);
  return support;
}

TubeFreeBallQuery OpenQuery(const TubeMapCaptureKey& key) {
  return [key](const Eigen::Vector3d& witness, const double required) {
    // Keep the fixture's fixed map domain an independent exact dyadic oracle;
    // ordinary Eigen multiply/add rounding must not silently move a closed
    // boundary by one ULP.
    for (std::size_t i = 0U; i < 3U; ++i) {
      const std::int64_t min_index =
          (i == 0U ? key.grid_min_index_x :
           (i == 1U ? key.grid_min_index_y : key.grid_min_index_z)) +
          (i == 0U ? key.grid_source_offset_x :
           (i == 1U ? key.grid_source_offset_y : key.grid_source_offset_z));
      const std::int64_t max_index =
          (i == 0U ? key.grid_max_index_x :
           (i == 1U ? key.grid_max_index_y : key.grid_max_index_z)) +
          (i == 0U ? key.grid_source_offset_x :
           (i == 1U ? key.grid_source_offset_y : key.grid_source_offset_z));
      const Dyadic origin = Decode(key.grid_native_origin(
          static_cast<Eigen::Index>(i)));
      const Dyadic resolution = Decode(key.grid_voxel_resolution(
          static_cast<Eigen::Index>(i)));
      const Dyadic domain_lower = Add(
          origin, Multiply(resolution, Decode(static_cast<double>(min_index))));
      const Dyadic domain_upper = Add(
          origin, Multiply(resolution,
                           Decode(static_cast<double>(max_index + 1))));
      const Dyadic ball_lower = Subtract(
          Decode(witness(static_cast<Eigen::Index>(i))), Decode(required));
      const Dyadic ball_upper = Add(
          Decode(witness(static_cast<Eigen::Index>(i))), Decode(required));
      if (Compare(ball_lower, domain_lower) < 0 ||
          Compare(ball_upper, domain_upper) > 0) {
        TubeFreeBallQueryResult result;
        result.status = DistanceStatus::OUT_OF_MAP;
        result.map_instance_id = key.map_instance_id;
        result.map_state_id = key.state_id;
        result.configuration_id = key.configuration_id;
        return result;
      }
    }
    TubeFreeBallQueryResult result;
    result.status = DistanceStatus::KNOWN_FREE;
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

struct ClosedBox {
  Eigen::Vector3d lower = Eigen::Vector3d::Zero();
  Eigen::Vector3d upper = Eigen::Vector3d::Zero();
};

bool BallIntersectsClosedBox(const Eigen::Vector3d& point,
                             const ClosedBox& box, const double radius) {
  Dyadic squared_distance;
  squared_distance.numerator = 0;
  squared_distance.exponent = 0;
  for (std::size_t i = 0U; i < 3U; ++i) {
    const Dyadic value = Decode(point(static_cast<Eigen::Index>(i)));
    const Dyadic lower = Decode(box.lower(static_cast<Eigen::Index>(i)));
    const Dyadic upper = Decode(box.upper(static_cast<Eigen::Index>(i)));
    Dyadic separation;
    if (Compare(value, lower) < 0) separation = Subtract(lower, value);
    else if (Compare(value, upper) > 0) separation = Subtract(value, upper);
    else separation = Decode(0.0);
    squared_distance = Add(squared_distance, Multiply(separation, separation));
  }
  return Compare(squared_distance,
                 Multiply(Decode(radius), Decode(radius))) < 0;
}

TubeFreeBallQuery BoxQuery(const TubeMapCaptureKey& key,
                           const std::vector<ClosedBox>& obstacles) {
  return [key, obstacles](const Eigen::Vector3d& witness,
                          const double required) {
    for (const ClosedBox& obstacle : obstacles) {
      if (BallIntersectsClosedBox(witness, obstacle, required)) {
        TubeFreeBallQueryResult result;
        result.status = DistanceStatus::OCCUPIED;
        result.map_instance_id = key.map_instance_id;
        result.map_state_id = key.state_id;
        result.configuration_id = key.configuration_id;
        return result;
      }
    }
    return OpenQuery(key)(witness, required);
  };
}

ClosedBox Box(const double x0, const double x1, const double y0,
              const double y1) {
  ClosedBox result;
  result.lower = Eigen::Vector3d(x0, y0, -100.0);
  result.upper = Eigen::Vector3d(x1, y1, 100.0);
  return result;
}

TubeBuildInputV2 Input(const TubeCertificateConfigV2& config,
                       const TubeFreeBallQuery& query = TubeFreeBallQuery()) {
  TubeBuildInputV2 input;
  input.request_id = 101U;
  input.path_key.execution_generation = 1U;
  input.path_key.path_instance_id = 2U;
  input.path_key.path_revision = 7U;
  input.path_key.frame_revision = 7U;
  input.path_key.frame_convention_id = 1U;
  input.path_key.frame_convention =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  input.path_key.phase_orientation = 1;
  input.path_key.domain_start = 0.0;
  input.path_key.domain_end = 2.0;
  input.configuration_key = config.key();
  input.map_capture_key = MapKey();
  input.requested_start = 0.0;
  input.requested_end = 2.0;
  input.anchor_w = 1.0;
  input.path_cells.push_back(Cell(0.0, 1.0));
  input.path_cells.push_back(Cell(1.0, 2.0));
  input.producer_breakpoints = {0.0, 1.0, 2.0};
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = Cell(w0, w1);
    return true;
  };
  input.path_owner = Owner();
  input.query_owner = Owner();
  input.capture_owner = Owner();
  input.applicability_assumptions = "complete immutable support";
  input.applicability_deadline_ticks = 15U;
  input.free_ball_query = query ? query : OpenQuery(input.map_capture_key);
  return input;
}

TubeCertificateBuildResultV2 Build(
    TubeBuildInputV2 input,
    const TubeCertificateConfigV2& config = Config()) {
  TubeCertificateBuilderV2 builder(config);
  TubeCertificateBuildResultV2 result;
  builder.build(input, result);
  return result;
}

TEST(TubeCertificateV2Test, G01StraightOpenProducesZeroAnchoredPwl) {
  const TubeCertificateConfigV2 config = Config();
  const TubeCertificateBuildResultV2 result = Build(Input(config), config);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.profile.structurallyValid());
  EXPECT_EQ(result.profile.capability, TubeProfileV2Capability::OFFSET_CERTIFIED);
  EXPECT_TRUE(result.profile.contains(1.0, 0.1));
  EXPECT_TRUE(result.profile.contains(1.0, -0.1));
  EXPECT_GE(result.stats.query_count, 1U);
}

TEST(TubeCertificateV2Test, G02AsymmetricObstacleStopsOnlyPositiveSide) {
  const TubeCertificateConfigV2 config = Config();
  const TubeMapCaptureKey key = MapKey();
  const TubeFreeBallQuery query = BoxQuery(
      key, {Box(-100.0, 100.0, 0.52, 100.0)});
  const TubeCertificateBuildResultV2 result = Build(Input(config, query), config);
  ASSERT_TRUE(result.success);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(result.profile.evaluate(1.0, lower, upper));
  EXPECT_LT(upper, 0.2);
  EXPECT_LT(lower, -0.19);
}

TEST(TubeCertificateV2Test, G03RemoteIslandIsNeverJoined) {
  TubeCertificateConfigV2 config = Config();
  config.nominal_half_width = 0.8;
  config.ray_step = 0.2;
  const TubeMapCaptureKey key = MapKey();
  const TubeFreeBallQuery query = BoxQuery(
      key, {Box(-100.0, 100.0, 0.42, 0.50)});
  const TubeCertificateBuildResultV2 result = Build(Input(config, query), config);
  ASSERT_TRUE(result.success);
  EXPECT_FALSE(result.profile.contains(1.0, 0.15));
  EXPECT_TRUE(result.profile.contains(1.0, 0.0));
}

TEST(TubeCertificateV2Test, G03BothSidesRemainIndependent) {
  const TubeCertificateConfigV2 config = Config();
  const TubeMapCaptureKey key = MapKey();
  const TubeFreeBallQuery query = BoxQuery(
      key, {Box(-100.0, 100.0, 0.52, 100.0),
            Box(-100.0, 100.0, -100.0, -0.52)});
  const TubeCertificateBuildResultV2 result = Build(Input(config, query), config);
  ASSERT_TRUE(result.success);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(result.profile.evaluate(1.0, lower, upper));
  EXPECT_LT(lower, 0.0);
  EXPECT_GT(upper, 0.0);
  EXPECT_NEAR(-lower, upper, 0.03);
}

TEST(TubeCertificateV2Test, G04NominalCapAndPartialLastSegmentAreRespected) {
  TubeCertificateConfigV2 config = Config();
  config.nominal_half_width = 0.23;
  config.ray_step = 0.1;
  const TubeCertificateBuildResultV2 result = Build(Input(config), config);
  ASSERT_TRUE(result.success);
  bool reaches_positive_cap = false;
  bool reaches_negative_cap = false;
  for (const TubeProofCellV2& cell : result.profile.cells) {
    EXPECT_LE(cell.upper, config.nominal_half_width);
    EXPECT_GE(cell.lower, -config.nominal_half_width);
    reaches_positive_cap = reaches_positive_cap ||
        cell.upper == config.nominal_half_width;
    reaches_negative_cap = reaches_negative_cap ||
        cell.lower == -config.nominal_half_width;
  }
  EXPECT_TRUE(reaches_positive_cap);
  EXPECT_TRUE(reaches_negative_cap);
}

TEST(TubeCertificateV2Test, C05IndexedEndpointsPreserveExactProductsAndFloor) {
  for (int k = 1; k <= 3; ++k) {
    SCOPED_TRACE(k);
    TubeCertificateConfigV2 config = Config();
    config.nominal_half_width = 0.4;
    config.ray_step = 0.1;
    config.budgets.max_queries = static_cast<std::size_t>(k + 1);
    config.budgets.max_witnesses = static_cast<std::size_t>(k + 1);
    config.budgets.max_cells = 2U;
    TubeBuildInputV2 input = Input(config);
    input.anchor_w = 1.0;
    // The authoritative callback and immutable producer breakpoints already
    // provide the standard [0,1],[1,2] partition; avoid duplicating fallback
    // path-cell endpoints in this intentionally tiny raw-work budget.
    input.path_cells.clear();
    const TubeCertificateBuildResultV2 result = Build(input, config);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.profile.cells.size(), 1U);
    ASSERT_EQ(result.stats.query_count, static_cast<std::size_t>(k + 1));
    const TubeProofCellV2& first = result.profile.cells.front();
    const Dyadic exact_product = Multiply(
        Decode(static_cast<double>(k)), Decode(config.ray_step));
    const std::pair<double, double> exact_bounds = Outward(exact_product);
    // The first retained cell was stopped by the query budget after exactly
    // k positive transverse segments; its upper extent is the floored
    // indexed endpoint, never an outward-interval overshoot.
    EXPECT_DOUBLE_EQ(first.upper, exact_bounds.first);
    EXPECT_GT(Compare(Decode(std::nextafter(
                       first.upper, std::numeric_limits<double>::infinity())),
                      exact_product), 0);
  }
}

TEST(TubeCertificateV2Test, G05MalformedOrUncertifiedBallFailsClosed) {
  TubeCertificateConfigV2 config = Config();
  const TubeMapCaptureKey key = MapKey();
  TubeFreeBallQuery query = [key](const Eigen::Vector3d& witness,
                                  const double required) {
    TubeFreeBallQueryResult result = OpenQuery(key)(witness, required);
    result.complete_support = false;
    result.support.valid = false;
    return result;
  };
  const TubeCertificateBuildResultV2 result = Build(Input(config, query), config);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.profile.failure_reason,
            TubeCertificateFailureReason::CLEARANCE_UNCERTIFIED);
}

TEST(TubeCertificateV2Test, G05ThinClosedStripIsCaughtByBallOracle) {
  const TubeCertificateConfigV2 config = Config();
  const TubeMapCaptureKey key = MapKey();
  const TubeFreeBallQuery query = BoxQuery(
      key, {Box(-100.0, 100.0, 0.52, 0.54)});
  const TubeCertificateBuildResultV2 result = Build(Input(config, query), config);
  ASSERT_TRUE(result.success);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(result.profile.evaluate(1.0, lower, upper));
  EXPECT_LE(upper, 0.1);
}

TEST(TubeCertificateV2Test, G06NarrowWCellIsSubdividedLeftFirst) {
  TubeCertificateConfigV2 config = Config();
  config.sample_step_w = 2.0;
  const TubeMapCaptureKey key = MapKey();
  std::size_t calls = 0U;
  const TubeFreeBallQuery base = BoxQuery(
      key, {Box(0.9, 1.1, -100.0, 100.0)});
  const TubeFreeBallQuery query = [base, &calls](const Eigen::Vector3d& witness,
                                                const double required) {
    ++calls;
    return base(witness, required);
  };
  TubeBuildInputV2 input = Input(config, query);
  input.path_cells.clear();
  input.path_cells.push_back(Cell(0.0, 2.0, 0.0));
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = Cell(w0, w1, 0.0);
    return true;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  EXPECT_GT(calls, 1U);
  EXPECT_TRUE(result.success || result.profile.truncation !=
      TubeCertificateTruncation::NONE);
}

TEST(TubeCertificateV2Test, G07OccupiedGapPreventsRemoteSuffixBridge) {
  TubeCertificateConfigV2 config = Config();
  const TubeMapCaptureKey key = MapKey();
  const TubeFreeBallQuery query = BoxQuery(
      key, {Box(0.8, 1.2, -100.0, 100.0)});
  TubeBuildInputV2 input = Input(config, query);
  input.anchor_w = 0.25;
  input.path_cells.clear();
  input.path_cells.push_back(Cell(0.0, 2.0, 0.0, 2.0, 0.0, 2.0));
  input.anchor_w = 0.25;
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = Cell(w0, w1, 0.0, 2.0, 0.0, 2.0);
    return true;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.profile.contains(0.25, 0.0));
  EXPECT_LE(result.profile.certified_end, 0.4);
  EXPECT_FALSE(result.profile.contains(0.5, 0.0));
  EXPECT_NE(result.profile.truncation, TubeCertificateTruncation::NONE);
}

TEST(TubeCertificateV2Test, G08ZeroAnchorUnknownCannotBecomeFakeZeroProfile) {
  TubeCertificateConfigV2 config = Config();
  config.budgets.max_w_depth = 1;
  const TubeMapCaptureKey key = MapKey();
  const TubeFreeBallQuery query = [key](const Eigen::Vector3d& witness,
                                        const double required) {
    TubeFreeBallQueryResult result;
    result.status = DistanceStatus::UNKNOWN;
    result.map_instance_id = key.map_instance_id;
    result.map_state_id = key.state_id;
    result.configuration_id = key.configuration_id;
    return result;
  };
  const TubeCertificateBuildResultV2 result = Build(Input(config, query), config);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.profile.capability, TubeProfileV2Capability::UNAVAILABLE);
}

TEST(TubeCertificateV2Test, G09HorizontalNormalFloorIsStrict) {
  TubeCertificateConfigV2 config = Config();
  TubeBuildInputV2 input = Input(config);
  input.path_cells.clear();
  input.path_cells.push_back(Cell(0.0, 2.0, 0.0, 1.0, 0.0, 1e-8));
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = Cell(w0, w1, 0.0, 1.0, 0.0, 1e-8);
    return true;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  EXPECT_FALSE(result.success);
}

TEST(TubeCertificateV2Test, G10RegularityCapIsVerifiedIndependently) {
  TubeCertificateConfigV2 config = Config();
  TubeBuildInputV2 input = Input(config);
  input.path_cells.clear();
  input.path_cells.push_back(Cell(0.0, 2.0, 0.0, 0.1, 1.0, 1.0));
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = Cell(w0, w1, 0.0, 0.1, 1.0, 1.0);
    return true;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.profile.failure_reason, TubeCertificateFailureReason::REGULARITY);
}

TEST(TubeCertificateV2Test, G11SubrangeKeepsOwnerBreakpoints) {
  TubeCertificateConfigV2 config = Config();
  config.sample_step_w = 10.0;
  TubeBuildInputV2 input = Input(config);
  input.requested_start = 0.25;
  input.requested_end = 1.75;
  input.anchor_w = 1.0;
  input.sample_grid.clear();
  input.producer_breakpoints = {0.0, 0.5, 1.0, 1.5, 2.0};
  input.path_cells.clear();
  input.path_cells.push_back(Cell(0.0, 2.0));
  const TubeCertificateBuildResultV2 result = Build(input, config);
  ASSERT_TRUE(result.success);
  ASSERT_GE(result.profile.knots.size(), 4U);
  EXPECT_DOUBLE_EQ(result.profile.knots[1].w, 0.5);
  EXPECT_DOUBLE_EQ(result.profile.knots[2].w, 1.0);
  EXPECT_DOUBLE_EQ(result.profile.knots[3].w, 1.5);
}

TEST(TubeCertificateV2Test, G12MalformedPathIdentityRejected) {
  TubeCertificateConfigV2 config = Config();
  TubeBuildInputV2 input = Input(config);
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = Cell(w0, w1);
    cell.path_revision = 999U;
    return true;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.profile.failure_reason,
            TubeCertificateFailureReason::MALFORMED_PATH_CELL);
}

TEST(TubeCertificateV2Test, G11RequestedRangeCannotExtrapolatePastOwnerDomain) {
  TubeCertificateConfigV2 config = Config();
  TubeBuildInputV2 input = Input(config);
  input.requested_end = 2.1;
  input.anchor_w = 1.0;
  const TubeCertificateBuildResultV2 result = Build(input, config);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.profile.failure_reason,
            TubeCertificateFailureReason::INVALID_INPUT);
}

TEST(TubeCertificateV2Test, G12AdjacentCertifiedCellsHaveDistinctNodeIds) {
  TubeCertificateConfigV2 config = Config();
  config.sample_step_w = 1.0;
  TubeBuildInputV2 input = Input(config);
  const TubeCertificateBuildResultV2 result = Build(input, config);
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.profile.cells.size(), 2U);
  ASSERT_EQ(result.profile.knots.size(), 3U);
  EXPECT_NE(result.profile.cells[0].cell_id,
            result.profile.cells[1].cell_id);
  EXPECT_NE(result.profile.knots[0].right_cell_id,
            result.profile.knots[1].right_cell_id);
  EXPECT_EQ(result.profile.knots[0].right_cell_id,
            result.profile.cells[0].cell_id);
  EXPECT_EQ(result.profile.knots[1].left_cell_id,
            result.profile.cells[0].cell_id);
  EXPECT_EQ(result.profile.knots[1].right_cell_id,
            result.profile.cells[1].cell_id);
  EXPECT_EQ(result.profile.knots[2].left_cell_id,
            result.profile.cells[1].cell_id);
  EXPECT_EQ(result.profile.cells[0].segment_identity,
            result.profile.cells[1].segment_identity);
}

TEST(TubeCertificateV2Test, G12PwlIntersectionControlsNonzeroCapability) {
  TubeCertificateConfigV2 config = Config();
  config.budgets.max_w_depth = 0;
  config.sample_step_w = 10.0;
  TubeBuildInputV2 input = Input(config);
  input.requested_end = 2.0;
  input.anchor_w = 1.5;
  input.path_cells.clear();
  input.path_cells.push_back(Cell(0.0, 0.5));
  input.path_cells.push_back(Cell(0.5, 1.5));
  input.path_cells.push_back(Cell(1.5, 2.0));
  input.producer_breakpoints = {0.0, 0.5, 1.5, 2.0};
  input.path_cell_query = TubePathCellQueryV2();
  const TubeMapCaptureKey key = MapKey();
  input.free_ball_query = [key](const Eigen::Vector3d& witness,
                                const double required) {
    // Algebraic/API fixture: the two outer cells certify only their zero
    // anchor; the middle cell is fully open.  The test targets the resulting
    // PWL intersection metadata, not physical map-oracle evidence.
    if (witness.y() != 0.0 &&
        (witness.x() < 0.75 || witness.x() > 1.25)) {
      TubeFreeBallQueryResult result;
      result.status = DistanceStatus::OCCUPIED;
      result.map_instance_id = key.map_instance_id;
      result.map_state_id = key.state_id;
      result.configuration_id = key.configuration_id;
      return result;
    }
    return OpenQuery(key)(witness, required);
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.profile.cells.size(), 3U);
  EXPECT_EQ(result.profile.capability, TubeProfileV2Capability::ZERO_ONLY);
  EXPECT_FALSE(result.profile.nonzero_capacity);
  for (const TubePwlKnotV2& knot : result.profile.knots) {
    EXPECT_DOUBLE_EQ(knot.lower, 0.0);
    EXPECT_DOUBLE_EQ(knot.upper, 0.0);
  }
}

TEST(TubeCertificateV2Test, G12AdjacentUlpCellsUseEnclosingAnchors) {
  const double w0 = 1.0;
  const double w1 = std::nextafter(w0, std::numeric_limits<double>::infinity());
  const double w2 = std::nextafter(w1, std::numeric_limits<double>::infinity());
  TubeCertificateConfigV2 config = Config();
  config.sample_step_w = 1.0;
  TubeBuildInputV2 input = Input(config);
  input.requested_start = w0;
  input.requested_end = w2;
  input.anchor_w = w1;
  input.sample_grid.clear();
  input.producer_breakpoints = {w0, w1, w2};
  input.path_cells.clear();
  input.path_cells.push_back(Cell(w0, w1));
  input.path_cells.push_back(Cell(w1, w2));
  input.path_cell_query = TubePathCellQueryV2();
  const TubeCertificateBuildResultV2 result = Build(input, config);
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.profile.cells.size(), 2U);
  ASSERT_EQ(result.profile.knots.size(), 3U);
  EXPECT_DOUBLE_EQ(result.profile.cells[0].w1, w1);
  EXPECT_DOUBLE_EQ(result.profile.cells[1].w0, w1);
  EXPECT_DOUBLE_EQ(result.profile.knots[1].w, w1);
  EXPECT_NE(result.profile.cells[0].cell_id,
            result.profile.cells[1].cell_id);
}

TEST(TubeCertificateV2Test, C01WitnessRadiusIncludesEtaAndCoordinateError) {
  struct Observation {
    double witness_x = 0.0;
    double witness_y = 0.0;
    double witness_z = 0.0;
    double required = 0.0;
  };
  TubeCertificateConfigV2 config = Config();
  TubeBuildInputV2 input = Input(config);
  std::vector<Observation> observations;
  const TubeMapCaptureKey key = MapKey();
  input.free_ball_query = [key, &observations](const Eigen::Vector3d& witness,
                                                const double required) {
    observations.push_back(
        Observation{witness.x(), witness.y(), witness.z(), required});
    return OpenQuery(key)(witness, required);
  };
  // Use a binary-exact, deliberately wide coordinate interval so omitting
  // e_q is separated by many ULPs from the independent lower oracle.
  const double coordinate_error = 0.125;
  input.path_cell_query = [coordinate_error](
      const double w0, const double w1,
      phase_offset_core::CertifiedPathCellV2& cell) {
    cell = Cell(w0, w1);
    const double lo = cell.anchor_position.component[0].lower -
        coordinate_error;
    const double hi = cell.anchor_position.component[0].upper +
        coordinate_error;
    SetInterval(cell.anchor_position.component[0], lo, hi);
    return true;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.profile.cells.size(), 2U);
  ASSERT_EQ(observations.size(), 10U);

  std::size_t observations_per_cell[2] = {0U, 0U};
  bool saw_anchor = false;
  bool saw_transverse = false;
  for (const Observation& observation : observations) {
    const std::size_t cell_index = observation.witness_x < 1.0 ? 0U : 1U;
    ASSERT_LT(cell_index, result.profile.cells.size());
    ++observations_per_cell[cell_index];
    const TubeProofCellV2& proof = result.profile.cells[cell_index];

    // Independent exact dyadic lower bound for
    //   epsilon + (P + D*Q)*(w1-w0)/2 + eta + e_q.
    const Dyadic p = Decode(proof.path_cell.sup_p_w_norm.upper);
    const Dyadic q = Decode(proof.q_regular);
    const Dyadic width = Subtract(Decode(proof.w1), Decode(proof.w0));
    const Dyadic motion = Multiply(
        Multiply(Add(p, Multiply(Decode(config.nominal_half_width), q)), width),
        Decode(0.5));
    const bool is_anchor = observation.witness_y == 0.0;
    const Dyadic eta = is_anchor
        ? Decode(0.0)
        : Multiply(Decode(config.ray_step), Decode(0.5));
    const Dyadic coordinate_width = Subtract(
        Decode(proof.path_cell.anchor_position.component[0].upper),
        Decode(proof.path_cell.anchor_position.component[0].lower));
    const Dyadic e_q = Multiply(coordinate_width, Decode(0.5));
    const Dyadic expected = Add(
        Decode(config.epsilon), Add(motion, Add(eta, e_q)));
    EXPECT_GE(Compare(Decode(observation.required), expected), 0)
        << "cell=" << cell_index << " witness_y=" << observation.witness_y;
    saw_anchor = saw_anchor || is_anchor;
    saw_transverse = saw_transverse || !is_anchor;
  }
  EXPECT_EQ(observations_per_cell[0], 5U);
  EXPECT_EQ(observations_per_cell[1], 5U);
  EXPECT_TRUE(saw_anchor);
  EXPECT_TRUE(saw_transverse);
}

TEST(TubeCertificateV2Test, C02MotionCoverUsesWholeCellDerivativeBound) {
  TubeCertificateConfigV2 config = Config();
  TubeBuildInputV2 input = Input(config);
  input.path_cells.clear();
  input.path_cells.push_back(Cell(0.0, 2.0));
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = Cell(w0, w1, 0.0, 3.0);
    return true;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  ASSERT_TRUE(result.success);
  for (const TubeProofCellV2& cell : result.profile.cells) {
    const Dyadic exact_sum = Add(
        Decode(cell.p_bound),
        Multiply(Decode(config.nominal_half_width), Decode(cell.q_regular)));
    const Dyadic exact_expected = Multiply(
        Multiply(exact_sum, Subtract(Decode(cell.w1), Decode(cell.w0))),
        Decode(0.5));
    EXPECT_GE(Compare(Decode(cell.motion_cover), exact_expected), 0);
  }
}

TEST(TubeCertificateV2Test, C02RotatingPolynomialRequiresQTerm) {
  TubeCertificateConfigV2 config = Config();
  config.sample_step_w = 0.2;
  TubeBuildInputV2 input = Input(config);
  input.requested_end = 0.2;
  input.anchor_w = 0.1;
  input.path_cells.clear();
  input.path_cells.push_back(RotatingPolynomialCell(0.0, 0.2));
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = RotatingPolynomialCell(w0, w1);
    return true;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  ASSERT_TRUE(result.success);
  bool saw_q_term = false;
  for (const TubeProofCellV2& cell : result.profile.cells) {
    EXPECT_GT(cell.q_regular, 0.0);
    const Dyadic width = Subtract(Decode(cell.w1), Decode(cell.w0));
    EXPECT_GE(Compare(
        Decode(cell.path_cell.chord_deviation.upper),
        Multiply(Multiply(width, width), Decode(0.125))), 0);
    EXPECT_GE(Compare(
        Decode(cell.path_cell.midpoint_position_variation.upper),
        Multiply(width, Decode(1.01))), 0);
    const Dyadic full_sum = Add(
        Decode(cell.p_bound),
        Multiply(Decode(config.nominal_half_width), Decode(cell.q_regular)));
    const Dyadic expected = Multiply(Multiply(full_sum, width), Decode(0.5));
    const Dyadic straight = Multiply(
        Multiply(Decode(cell.p_bound), width), Decode(0.5));
    EXPECT_GT(Compare(expected, straight), 0);
    EXPECT_GE(Compare(Decode(cell.motion_cover), expected), 0);
    if (Compare(Decode(cell.motion_cover), straight) > 0) saw_q_term = true;
  }
  EXPECT_TRUE(saw_q_term);
}

TEST(TubeCertificateV2Test, C02PhysicalNegativeOracleNeedsQTerm) {
  TubeCertificateConfigV2 config = Config();
  // The reference-speed equality makes the regularity cap exactly zero, so
  // this test exercises the anchor witness only.  With q omitted the motion
  // cover is 0.101; the exact Q-inclusive cover is 0.121, and the fixed
  // closed strip below lies between the resulting required radii.
  config.minimum_reference_speed = 1.0;
  config.budgets.max_w_depth = 0;
  config.sample_step_w = 1.0;
  TubeBuildInputV2 input = Input(config);
  input.requested_end = 0.2;
  input.anchor_w = 0.0;
  input.path_cells.clear();
  input.path_cells.push_back(RotatingPolynomialCell(0.0, 0.2));
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = RotatingPolynomialCell(w0, w1);
    return true;
  };
  const TubeMapCaptureKey key = MapKey();
  // At the cell's polynomial midpoint the witness is (0,0,0).  This closed
  // horizontal obstacle has exact clearance 0.21; eps + B_Q = 0.221, while
  // eps + B_P = 0.201.  A certificate omitting D*Q would therefore accept a
  // ball that this independent dyadic box oracle correctly rejects.
  input.free_ball_query = BoxQuery(key, {Box(-100.0, 100.0, 0.21, 100.0)});
  const TubeFreeBallQueryResult oracle = input.free_ball_query(
      Eigen::Vector3d::Zero(), 0.221);
  ASSERT_EQ(oracle.status, DistanceStatus::OCCUPIED);
  const TubeCertificateBuildResultV2 result = Build(input, config);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.profile.failure_reason,
            TubeCertificateFailureReason::CLEARANCE_OCCUPIED);
}

TEST(TubeCertificateV2Test, C02ReferenceSpeedEqualityIsZeroOnlyNotFailure) {
  TubeCertificateConfigV2 equal = Config();
  equal.minimum_reference_speed = 1.0;
  equal.budgets.max_w_depth = 0;
  equal.sample_step_w = 1.0;
  TubeBuildInputV2 input = Input(equal);
  input.requested_end = 0.2;
  input.anchor_w = 0.0;
  input.path_cells.clear();
  input.path_cells.push_back(RotatingPolynomialCell(0.0, 0.2));
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = RotatingPolynomialCell(w0, w1);
    return true;
  };
  const TubeCertificateBuildResultV2 known_free = Build(input, equal);
  ASSERT_TRUE(known_free.success);
  EXPECT_EQ(known_free.profile.capability,
            TubeProfileV2Capability::ZERO_ONLY);

  TubeCertificateConfigV2 below = equal;
  below.minimum_reference_speed = std::nextafter(
      equal.minimum_reference_speed, std::numeric_limits<double>::infinity());
  TubeBuildInputV2 below_input = input;
  below_input.configuration_key = below.key();
  const TubeCertificateBuildResultV2 rejected = Build(below_input, below);
  EXPECT_FALSE(rejected.success);
  EXPECT_EQ(rejected.profile.failure_reason,
            TubeCertificateFailureReason::REGULARITY);
}

TEST(TubeCertificateV2Test, C03UlpAndSignedCoordinatesRemainFinite) {
  TubeCertificateConfigV2 config = Config();
  config.epsilon = std::nextafter(0.1, std::numeric_limits<double>::infinity());
  TubeBuildInputV2 input = Input(config);
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = Cell(w0, w1, -1.0);
    return true;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.profile.structurallyValid());
  EXPECT_TRUE(result.profile.contains(1.0, 0.0));
}

TEST(TubeCertificateV2Test, C03SubnormalWidthsRemainDirectedAndFinite) {
  const double tiny = std::numeric_limits<double>::denorm_min();
  TubeCertificateConfigV2 config = Config();
  config.epsilon = tiny;
  config.nominal_half_width = tiny;
  config.ray_step = tiny;
  // Keep the captured map/configuration resolution consistent with MapKey;
  // the transverse step is still denorm_min through nominal_half_width and
  // ray_step.
  config.snapshot_resolution = 0.2;
  config.minimum_reference_speed = 1e-8;
  config.sample_step_w = 1.0;
  TubeBuildInputV2 input = Input(config);
  input.requested_end = tiny;
  input.anchor_w = 0.0;
  input.path_cells.clear();
  input.path_cells.push_back(Cell(0.0, tiny, 0.0, 2e-8, 0.0, 2e-8));
  input.path_cell_query = TubePathCellQueryV2();
  input.producer_breakpoints.clear();
  input.sample_grid.clear();
  std::vector<double> requested;
  const TubeMapCaptureKey key = MapKey();
  input.free_ball_query = [key, &requested](const Eigen::Vector3d& witness,
                                            const double required) {
    requested.push_back(required);
    return OpenQuery(key)(witness, required);
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  ASSERT_TRUE(result.success);
  ASSERT_FALSE(requested.empty());
  for (const double required : requested) {
    EXPECT_TRUE(std::isfinite(required));
    EXPECT_GT(required, 0.0);
  }
  EXPECT_TRUE(result.profile.structurallyValid());
}

TEST(TubeCertificateV2Test, C03FixedDomainOracleHasClosedDyadicBounds) {
  TubeMapCaptureKey key = MapKey();
  // Use an exactly representable one-unit grid for a boundary test; the
  // production fixture above intentionally uses a non-dyadic resolution.
  key.grid_min_index_x = 0;
  key.grid_min_index_y = 0;
  key.grid_min_index_z = 0;
  key.grid_max_index_x = 1;
  key.grid_max_index_y = 1;
  key.grid_max_index_z = 1;
  key.grid_voxel_resolution = Eigen::Vector3d::Ones();
  const TubeFreeBallQuery query = OpenQuery(key);
  const TubeFreeBallQueryResult on_boundary = query(
      Eigen::Vector3d::Ones(), 1.0);
  EXPECT_EQ(on_boundary.status, DistanceStatus::KNOWN_FREE);
  const TubeFreeBallQueryResult outside = query(
      Eigen::Vector3d::Ones(),
      std::nextafter(1.0, std::numeric_limits<double>::infinity()));
  EXPECT_EQ(outside.status, DistanceStatus::OUT_OF_MAP);
}

TEST(TubeCertificateV2Test, C03MismatchedGridResolutionFailsClosed) {
  TubeCertificateConfigV2 config = Config();
  TubeBuildInputV2 input = Input(config);
  input.map_capture_key.grid_voxel_resolution.x() = std::nextafter(
      config.snapshot_resolution, std::numeric_limits<double>::infinity());
  const TubeCertificateBuildResultV2 result = Build(input, config);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.profile.failure_reason,
            TubeCertificateFailureReason::INVALID_INPUT);
}

TEST(TubeCertificateV2Test, C03EmptyMapFrameFailsClosed) {
  TubeMapCaptureKey key = MapKey();
  key.frame_provenance.clear();
  EXPECT_FALSE(key.complete());
}

TEST(TubeCertificateV2Test, C03ReturnedMapFrameMismatchFailsClosed) {
  TubeCertificateConfigV2 config = Config();
  const TubeMapCaptureKey key = MapKey();
  const TubeFreeBallQuery query = [key](const Eigen::Vector3d& witness,
                                        const double required) {
    TubeFreeBallQueryResult result = OpenQuery(key)(witness, required);
    result.frame_provenance = "calibrated_frame_changed";
    return result;
  };
  const TubeCertificateBuildResultV2 result = Build(Input(config, query), config);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.profile.failure_reason,
            TubeCertificateFailureReason::MAP_CAPTURE_MISMATCH);
}

TEST(TubeCertificateV2Test, C04SupportBoundaryRequiresCompleteFootprint) {
  TubeCertificateConfigV2 config = Config();
  const TubeMapCaptureKey key = MapKey();
  TubeFreeBallQuery query = [key](const Eigen::Vector3d& witness,
                                  const double required) {
    TubeFreeBallQueryResult result = OpenQuery(key)(witness, required);
    result.support.voxel_footprint.front().voxel_resolution.x() = 2.0;
    return result;
  };
  const TubeCertificateBuildResultV2 result = Build(Input(config, query), config);
  EXPECT_FALSE(result.success);
}

TEST(TubeCertificateV2Test, C04OversizedSupportRadiusIsNotRetained) {
  TubeCertificateConfigV2 config = Config();
  const TubeMapCaptureKey key = MapKey();
  TubeFreeBallQuery query = [key](const Eigen::Vector3d& witness,
                                  const double required) {
    TubeFreeBallQueryResult result = OpenQuery(key)(witness, required);
    result.support.radius = std::nextafter(
        required, std::numeric_limits<double>::infinity());
    return result;
  };
  const TubeCertificateBuildResultV2 result = Build(Input(config, query), config);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.profile.failure_reason,
            TubeCertificateFailureReason::MAP_CAPTURE_MISMATCH);
}

TEST(TubeCertificateV2Test, C05QueryBudgetStopsWithoutManufacturedChildren) {
  TubeCertificateConfigV2 config = Config();
  config.budgets.max_queries = 1U;
  config.budgets.max_witnesses = 1U;
  config.budgets.max_cells = 1U;
  config.sample_step_w = 10.0;
  TubeBuildInputV2 input = Input(config);
  input.anchor_w = 0.0;
  input.path_cells.clear();
  input.path_cells.push_back(Cell(0.0, 2.0));
  input.path_cell_query = TubePathCellQueryV2();
  input.producer_breakpoints = {0.0, 2.0};
  input.sample_grid.clear();
  const TubeCertificateBuildResultV2 result = Build(input, config);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.stats.query_budget_reached ||
              result.stats.witness_budget_reached);
}

TEST(TubeCertificateV2Test, C05ScheduledCellBudgetStopsBeforeChildCallbacks) {
  TubeCertificateConfigV2 config = Config();
  config.budgets.max_cells = 3U;
  config.budgets.max_queries = 32U;
  config.budgets.max_witnesses = 32U;
  config.budgets.max_w_depth = 12;
  config.sample_step_w = 10.0;
  TubeBuildInputV2 input = Input(config);
  input.anchor_w = 0.0;
  input.path_cells.clear();
  input.path_cells.push_back(Cell(0.0, 2.0));
  input.producer_breakpoints = {0.0, 2.0};
  input.sample_grid.clear();
  std::size_t path_callbacks = 0U;
  input.path_cell_query = [&path_callbacks](
      const double w0, const double w1,
      phase_offset_core::CertifiedPathCellV2& cell) {
    ++path_callbacks;
    cell = Cell(w0, w1);
    return true;
  };
  const TubeMapCaptureKey key = MapKey();
  input.free_ball_query = [key](const Eigen::Vector3d& witness,
                                const double required) {
    (void)witness;
    (void)required;
    TubeFreeBallQueryResult result;
    result.status = DistanceStatus::UNKNOWN;
    result.map_instance_id = key.map_instance_id;
    result.map_state_id = key.state_id;
    result.configuration_id = key.configuration_id;
    return result;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.stats.cell_budget_reached);
  EXPECT_EQ(result.stats.scheduled_cell_count, 3U);
  EXPECT_EQ(result.stats.cell_count, 2U);
  EXPECT_EQ(result.stats.query_count, 2U);
  EXPECT_EQ(path_callbacks, 2U);
  EXPECT_LE(result.stats.cell_count, config.budgets.max_cells);
  EXPECT_LE(result.stats.scheduled_cell_count, config.budgets.max_cells);
}

TEST(TubeCertificateV2Test, C06InvalidIntervalsAndDenominatorFailClosed) {
  TubeCertificateConfigV2 config = Config();
  TubeBuildInputV2 input = Input(config);
  input.path_cell_query = [](const double w0, const double w1,
                             phase_offset_core::CertifiedPathCellV2& cell) {
    cell = Cell(w0, w1);
    cell.anchor_p_w.component[0].lower = -1.0;
    return true;
  };
  const TubeCertificateBuildResultV2 result = Build(input, config);
  EXPECT_FALSE(result.success);
}

TEST(TubeCertificateV2Test, C07ClosedVolumeSupportIsNotCenterOnly) {
  TubeCertificateConfigV2 config = Config();
  const TubeMapCaptureKey key = MapKey();
  TubeFreeBallQuery query = [key](const Eigen::Vector3d& witness,
                                  const double required) {
    TubeFreeBallQueryResult result = OpenQuery(key)(witness, required);
    result.support.lower = witness;
    result.support.upper = witness;
    return result;
  };
  const TubeCertificateBuildResultV2 result = Build(Input(config, query), config);
  EXPECT_FALSE(result.success);
}

TEST(TubeCertificateV2Test, FPGuardRejectsNonNearestRounding) {
  const int original = fegetround();
  ASSERT_EQ(fesetround(FE_DOWNWARD), 0);
  const TubeCertificateConfigV2 config = Config();
  const TubeCertificateBuildResultV2 result = Build(Input(config), config);
  EXPECT_FALSE(result.success);
  ASSERT_EQ(fesetround(original), 0);
}

#if defined(__i386__) || defined(__x86_64__)
TEST(TubeCertificateV2Test, FPGuardRejectsFlushToZeroAndDenormalsAreRestored) {
  const unsigned int original = _mm_getcsr();
  const unsigned int adversaries[] = {0x8000U, 0x0040U, 0x8040U};
  for (const unsigned int mode : adversaries) {
    SCOPED_TRACE(mode);
    _mm_setcsr(original | mode);  // FTZ (bit 15) and/or DAZ (bit 6).
    const TubeCertificateConfigV2 config = Config();
    const TubeCertificateBuildResultV2 result = Build(Input(config), config);
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.profile.failure_reason,
              TubeCertificateFailureReason::UNSUPPORTED_FP_MODE);
    _mm_setcsr(original);
    EXPECT_EQ(_mm_getcsr(), original);
  }
}
#endif

TEST(TubeCertificateV2Test, StopsWorkBeyondFirstTerminalGapAfterAnchor) {
  const auto config = Config();
  auto input = Input(config);
  input.anchor_w = 0.0;
  std::size_t path_calls = 0U;
  input.path_cell_query = [&path_calls](double lo, double hi,
      phase_offset_core::CertifiedPathCellV2& cell) {
    ++path_calls;
    cell = Cell(lo, hi);
    return true;
  };
  const auto open = OpenQuery(MapKey());
  input.free_ball_query = [open](const Eigen::Vector3d& point, double radius) {
    // Availability fixture: the entire second cell is unsupported.  No
    // disconnected suffix can contribute to the anchored component.
    if (point.x() >= 1.0) {
      TubeFreeBallQueryResult unavailable;
      unavailable.status = DistanceStatus::UNKNOWN;
      return unavailable;
    }
    return open(point, radius);
  };
  const auto result = Build(input, config);
  ASSERT_TRUE(result.success);
  ASSERT_TRUE(result.profile.structurallyValid());
  EXPECT_DOUBLE_EQ(result.profile.certified_start, 0.0);
  EXPECT_DOUBLE_EQ(result.profile.certified_end, 1.0);
  EXPECT_EQ(result.profile.truncation, TubeCertificateTruncation::SUFFIX);
  EXPECT_EQ(result.stats.max_depth_observed, config.budgets.max_w_depth);
  // Still perform every required refinement on the first failing branch,
  // but not the thousands of siblings beyond the now-established gap.
  EXPECT_LE(path_calls, static_cast<std::size_t>(config.budgets.max_w_depth + 3));
  EXPECT_EQ(path_calls, result.stats.path_cell_query_count);
  for (const auto& cell : result.profile.cells) EXPECT_LE(cell.w1, 1.0);
  input.requested_end = 1.0;
  const auto prefix_only = Build(input, config);
  ASSERT_TRUE(prefix_only.success);
  ASSERT_EQ(prefix_only.profile.knots.size(), result.profile.knots.size());
  for (std::size_t i = 0; i < result.profile.knots.size(); ++i) {
    EXPECT_DOUBLE_EQ(prefix_only.profile.knots[i].w, result.profile.knots[i].w);
    EXPECT_DOUBLE_EQ(prefix_only.profile.knots[i].lower, result.profile.knots[i].lower);
    EXPECT_DOUBLE_EQ(prefix_only.profile.knots[i].upper, result.profile.knots[i].upper);
  }
}

TEST(TubeCertificateV2Test, TerminalProducerFailureAlsoClosesAnchoredComponent) {
  const auto config = Config();
  auto input = Input(config);
  input.anchor_w = 0.0;
  std::size_t path_calls = 0U;
  input.path_cell_query = [&path_calls](double lo, double hi,
      phase_offset_core::CertifiedPathCellV2& cell) {
    ++path_calls;
    if (lo >= 1.0) return false;
    cell = Cell(lo, hi);
    return true;
  };
  const auto result = Build(input, config);
  ASSERT_TRUE(result.success);
  EXPECT_DOUBLE_EQ(result.profile.certified_end, 1.0);
  EXPECT_EQ(result.profile.failure_reason, TubeCertificateFailureReason::MALFORMED_PATH_CELL);
  EXPECT_LE(path_calls, static_cast<std::size_t>(config.budgets.max_w_depth + 3));
}

TEST(TubeCertificateV2Test, GapBeforeAnchorDoesNotSuppressLaterAnchoredComponent) {
  auto config = Config();
  config.budgets.max_w_depth = 2;
  auto input = Input(config);
  input.anchor_w = 1.5;
  const auto open = OpenQuery(MapKey());
  input.free_ball_query = [open](const Eigen::Vector3d& point, double radius) {
    if (point.x() < 1.0) {
      TubeFreeBallQueryResult unavailable;
      unavailable.status = DistanceStatus::UNKNOWN;
      return unavailable;
    }
    return open(point, radius);
  };
  const auto result = Build(input, config);
  ASSERT_TRUE(result.success);
  EXPECT_DOUBLE_EQ(result.profile.certified_start, 1.0);
  EXPECT_DOUBLE_EQ(result.profile.certified_end, 2.0);
  EXPECT_TRUE(result.profile.contains_anchor);
  EXPECT_EQ(result.profile.truncation, TubeCertificateTruncation::PREFIX);
  // The previous failed cell's closed upper endpoint is shared with the
  // following successful component, so equality must not stop the search.
  input.anchor_w = 1.0;
  const auto shared_endpoint = Build(input, config);
  ASSERT_TRUE(shared_endpoint.success);
  EXPECT_DOUBLE_EQ(shared_endpoint.profile.certified_start, 1.0);
  EXPECT_DOUBLE_EQ(shared_endpoint.profile.certified_end, 2.0);
}

TEST(TubeCertificateV2Test, UnprovedAnchorDoesNotBuildUnusableLaterComponents) {
  const auto config = Config();
  auto input = Input(config);
  input.anchor_w = 0.0;
  input.free_ball_query = [](const Eigen::Vector3d&, double) {
    TubeFreeBallQueryResult unavailable;
    unavailable.status = DistanceStatus::UNKNOWN;
    return unavailable;
  };
  const auto result = Build(input, config);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.profile.truncation, TubeCertificateTruncation::NO_CERTIFIED_COMPONENT);
  EXPECT_EQ(result.stats.max_depth_observed, config.budgets.max_w_depth);
  EXPECT_LE(result.stats.path_cell_query_count,
      static_cast<std::size_t>(config.budgets.max_w_depth + 1));
  EXPECT_TRUE(result.profile.cells.empty());
}

}  // namespace
}  // namespace phase_offset_navigation

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
