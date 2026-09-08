#include <gtest/gtest.h>

#include "bspline_race/integration/phase_offset_cloud_occupancy_query.h"

#include <cfenv>
#include <cmath>
#include <limits>
#include <memory>
#include <set>

#if defined(__i386__) || defined(__x86_64__)
#include <immintrin.h>
#endif

namespace FLAG_Race {
namespace {

std::shared_ptr<const plan_env::CloudOccupancySnapshot> makeSnapshot() {
  plan_env::CloudOccupancySnapshotBuildInput input;
  input.odom_valid = true;
  input.observation_sequence = 7U;
  input.observation_stamp = ros::Time(21.0);
  input.camera_position = Eigen::Vector3d::Zero();
  input.map_min = Eigen::Vector3d(-5.0, -5.0, -5.0);
  input.map_max = Eigen::Vector3d(5.0, 5.0, 5.0);
  input.grid_origin = input.map_min;
  input.voxel_count = Eigen::Vector3i(10, 10, 10);
  input.local_update_range = Eigen::Vector3d(2.0, 2.0, 2.0);
  input.resolution = 1.0;
  input.obstacles_inflation = 0.99;
  input.cloud_points.emplace_back(1.1, 0.1, 0.1);
  return std::shared_ptr<const plan_env::CloudOccupancySnapshot>(
      new plan_env::CloudOccupancySnapshot(
          plan_env::buildCloudOccupancySnapshot(input)));
}

std::shared_ptr<const plan_env::SDFMapCaptureV2> makeV2Capture() {
  std::shared_ptr<plan_env::SDFMapCaptureV2> capture(
      new plan_env::SDFMapCaptureV2());
  capture->valid = true;
  capture->map_instance_id = 77U;
  capture->configuration_generation = 3U;
  capture->configuration_key = 19U;
  capture->frame_id = "world";
  capture->accepted_state_sequence = 11U;
  capture->accepted_state_notification_sequence = 11U;
  capture->accepted_time_ticks = 101U;
  capture->map_min = Eigen::Vector3d::Zero();
  capture->map_max = Eigen::Vector3d::Constant(4.0);
  capture->grid_origin = Eigen::Vector3d::Zero();
  capture->capture_min = Eigen::Vector3d::Zero();
  capture->capture_max = Eigen::Vector3d::Constant(4.0);
  capture->source_min_index = Eigen::Vector3i::Zero();
  capture->source_max_index = Eigen::Vector3i::Constant(3);
  capture->voxel_count = Eigen::Vector3i::Constant(4);
  capture->resolution = 1.0;
  capture->occupied.assign(64U, 0U);
  capture->support.valid = true;
  capture->support.complete = true;
  capture->support.evidence_basis =
      plan_env::kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  capture->support.evidence_sequence = 11U;
  capture->support.evidence_accepted_ticks = 101U;
  capture->support.map_instance_id = 77U;
  capture->support.configuration_generation = 3U;
  capture->support.configuration_key = 19U;
  capture->support.frame_id = "world";
  capture->support.support_min = capture->map_min;
  capture->support.support_max = capture->map_max;
  capture->support.halo_reconciled = true;
  capture->support.mask.assign(64U, 1U);
  return capture;
}

std::shared_ptr<const plan_env::CloudOccupancySnapshot>
makeCenterMetricSnapshot() {
  std::shared_ptr<plan_env::CloudOccupancySnapshot> snapshot(
      new plan_env::CloudOccupancySnapshot());
  snapshot->valid = true;
  snapshot->observation_sequence = 8U;
  snapshot->map_min = Eigen::Vector3d(-5.0, -5.0, -5.0);
  snapshot->map_max = Eigen::Vector3d(5.0, 5.0, 5.0);
  snapshot->observed_min = Eigen::Vector3d(-2.0, -2.0, -2.0);
  snapshot->observed_max = Eigen::Vector3d(2.0, 2.0, 2.0);
  snapshot->grid_origin = snapshot->map_min;
  snapshot->voxel_count = Eigen::Vector3i(10, 10, 10);
  snapshot->resolution = 1.0;
  snapshot->included_map_inflation = 0.10;
  snapshot->occupied.assign(1000U, 0U);
  snapshot->occupied[(6U * 10U + 5U) * 10U + 5U] = 1U;
  return std::shared_ptr<const plan_env::CloudOccupancySnapshot>(snapshot);
}

std::shared_ptr<const plan_env::CloudOccupancySnapshot>
makeFrozenGapSnapshot() {
  std::shared_ptr<plan_env::CloudOccupancySnapshot> snapshot(
      new plan_env::CloudOccupancySnapshot());
  snapshot->valid = true;
  snapshot->observation_sequence = 904U;
  snapshot->map_min = Eigen::Vector3d(-10.0, -15.0, -0.01);
  snapshot->map_max = Eigen::Vector3d(6.0, 3.0, 2.99);
  // The frozen query and its complete requested ball must be observed.  Keep
  // the production fail-closed domain checks unchanged and make only this
  // regression fixture's observation domain complete.
  snapshot->observed_min = snapshot->map_min;
  snapshot->observed_max = snapshot->map_max;
  snapshot->grid_origin = snapshot->map_min;
  snapshot->voxel_count = Eigen::Vector3i(160, 180, 30);
  snapshot->resolution = 0.1;
  snapshot->included_map_inflation = 0.1;
  snapshot->occupied.assign(160U * 180U * 30U, 0U);
  snapshot->occupied[(128U * 180U + 148U) * 30U + 10U] = 1U;
  return std::shared_ptr<const plan_env::CloudOccupancySnapshot>(snapshot);
}

CloudOccupancyQueryConfig usableConfig() {
  CloudOccupancyQueryConfig config;
  config.obstacle_set_complete = true;
  config.required_preincluded_map_uncertainty = 0.10;
  return config;
}

phase_offset_navigation::RobustTubeMargins margins() {
  phase_offset_navigation::RobustTubeMargins value;
  value.uav_radius = 0.25;
  value.map_uncertainty = 0.10;
  value.localization_uncertainty = 0.05;
  value.tracking_error_bound = 0.15;
  value.preincluded_map_uncertainty = 0.10;
  return value;
}

void setV2Interval(phase_offset_core::Binary64Interval& value,
                   const double lower, const double upper) {
  value.lower = lower;
  value.upper = upper;
  value.valid = true;
}

void setV2VectorInterval(phase_offset_core::Binary64VectorInterval& value,
                         const Eigen::Vector3d& lower,
                         const Eigen::Vector3d& upper) {
  value.valid = true;
  for (std::size_t index = 0U; index < 3U; ++index) {
    setV2Interval(value.component[index],
                  lower(static_cast<Eigen::Index>(index)),
                  upper(static_cast<Eigen::Index>(index)));
  }
}

phase_offset_core::CertifiedPathCellV2 makeStraightV2PathCell() {
  phase_offset_core::CertifiedPathCellV2 cell;
  cell.w0 = 0.0;
  cell.w1 = 1.0;
  cell.anchor_w = 0.5;
  cell.path_revision = 1U;
  cell.frame_revision = 1U;
  cell.segment_identity = 1U;
  cell.proof_identity = 2U;
  setV2VectorInterval(cell.anchor_position,
                      Eigen::Vector3d(2.5, 2.0, 2.0),
                      Eigen::Vector3d(2.5, 2.0, 2.0));
  setV2VectorInterval(cell.anchor_p_w,
                      Eigen::Vector3d(1.0, 0.0, 0.0),
                      Eigen::Vector3d(1.0, 0.0, 0.0));
  setV2VectorInterval(cell.anchor_p_ww, Eigen::Vector3d::Zero(),
                      Eigen::Vector3d::Zero());
  setV2Interval(cell.inf_p_w_norm, 1.0, 1.0);
  setV2Interval(cell.sup_p_w_norm, 1.0, 1.0);
  setV2Interval(cell.inf_horizontal_p_w_norm, 1.0, 1.0);
  setV2Interval(cell.sup_p_ww_norm, 0.0, 0.0);
  setV2Interval(cell.sup_horizontal_p_ww_norm, 0.0, 0.0);
  setV2Interval(cell.sup_p_www_norm, 0.0, 0.0);
  setV2Interval(cell.sup_normal_derivative, 0.0, 0.0);
  setV2Interval(cell.normal_variation, 0.0, 0.0);
  setV2Interval(cell.tangent_variation, 0.0, 0.0);
  setV2Interval(cell.curvature_variation, 0.0, 0.0);
  setV2Interval(cell.midpoint_position_variation, 0.0, 0.5);
  setV2Interval(cell.chord_deviation, 0.0, 0.0);
  cell.horizontal_acceleration_bound_complete = true;
  cell.normal_frame_proof_complete = true;
  cell.phase_map_proof_complete = true;
  cell.provenance = phase_offset_core::kWorldHorizontalCrossProductProvenance;
  cell.valid = true;
  cell.complete = true;
  return cell;
}

class ScopedRoundingMode {
 public:
  explicit ScopedRoundingMode(const int mode)
      : original_(std::fegetround()), changed_(std::fesetround(mode) == 0) {}

  ~ScopedRoundingMode() {
    if (changed_) std::fesetround(original_);
  }

  bool changed() const { return changed_; }

 private:
  int original_;
  bool changed_;
};

#if defined(__i386__) || defined(__x86_64__)
class ScopedFlushToZeroMode {
 public:
  explicit ScopedFlushToZeroMode(const unsigned int unsupported_bits)
      : original_(_mm_getcsr()), changed_(false) {
    const unsigned int unsupported = original_ | unsupported_bits;
    changed_ = unsupported != original_;
    _mm_setcsr(unsupported);
  }

  ~ScopedFlushToZeroMode() {
    if (changed_) _mm_setcsr(original_);
  }

  bool changed() const { return changed_; }

 private:
  unsigned int original_;
  bool changed_;
};
#endif

TEST(CloudOccupancyQueryTest, CompleteCloudContractMapsSnapshotStatuses) {
  const auto snapshot = makeSnapshot();
  const CloudOccupancyQueryStatus status =
      inspectCloudOccupancyQuery(snapshot, usableConfig());
  ASSERT_TRUE(status.usable);
  EXPECT_EQ(status.observation_sequence, 7U);
  EXPECT_NEAR(status.included_map_inflation, 1.0, 1e-12);
  const auto query = makeCloudOccupancyQuery(snapshot, usableConfig());
  EXPECT_EQ(query(Eigen::Vector3d(-1.9, 0.0, 0.0)),
            phase_offset_navigation::DistanceStatus::KNOWN_FREE);
  EXPECT_EQ(query(Eigen::Vector3d(1.1, 0.1, 0.1)),
            phase_offset_navigation::DistanceStatus::OCCUPIED);
  EXPECT_EQ(query(Eigen::Vector3d(2.5, 0.0, 0.0)),
            phase_offset_navigation::DistanceStatus::UNKNOWN);
  EXPECT_EQ(query(Eigen::Vector3d(5.1, 0.0, 0.0)),
            phase_offset_navigation::DistanceStatus::OUT_OF_MAP);
}

TEST(CloudOccupancyQueryTest, DisabledOrInsufficientContractFailsClosed) {
  const auto snapshot = makeSnapshot();
  CloudOccupancyQueryConfig disabled = usableConfig();
  disabled.obstacle_set_complete = false;
  EXPECT_FALSE(inspectCloudOccupancyQuery(snapshot, disabled).usable);
  EXPECT_EQ(makeCloudOccupancyQuery(snapshot, disabled)(Eigen::Vector3d::Zero()),
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);

  CloudOccupancyQueryConfig insufficient = usableConfig();
  insufficient.required_preincluded_map_uncertainty = 1.01;
  const CloudOccupancyQueryStatus insufficient_status =
      inspectCloudOccupancyQuery(snapshot, insufficient);
  EXPECT_FALSE(insufficient_status.usable);
  EXPECT_FALSE(insufficient_status.included_map_inflation_sufficient);
  EXPECT_EQ(makeCloudOccupancyQuery(snapshot, insufficient)(Eigen::Vector3d::Zero()),
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);
}

TEST(CloudOccupancyQueryTest, V2BridgeRetainsImmutableCaptureAndMetadata) {
  std::shared_ptr<const plan_env::SDFMapCaptureV2> capture = makeV2Capture();
  SDFMapCaptureQueryBridgeV2 bridge = makeSDFMapCaptureQueryV2(capture);
  ASSERT_TRUE(bridge.usable());
  ASSERT_TRUE(bridge.capture_owner);
  ASSERT_TRUE(bridge.query_owner);
  EXPECT_EQ(bridge.descriptor.map_capture_key.map_instance_id, 77U);
  EXPECT_EQ(bridge.descriptor.map_capture_key.state_id, 11U);
  EXPECT_EQ(bridge.descriptor.map_capture_key.accepted_sequence, 11U);
  EXPECT_EQ(bridge.status.accepted_state_notification_sequence, 11U);
  EXPECT_EQ(bridge.descriptor.accepted_state_notification_sequence, 11U);
  EXPECT_EQ(bridge.descriptor.map_capture_key.configuration_generation, 3U);
  EXPECT_EQ(bridge.descriptor.map_capture_key.configuration_id, 19U);
  EXPECT_EQ(bridge.descriptor.map_capture_key.frame_provenance, "world");
  EXPECT_EQ(bridge.descriptor.map_capture_key.grid_source_offset_x, 0);
  EXPECT_EQ(bridge.descriptor.map_capture_key.grid_max_index_x, 3);
  EXPECT_FALSE(bridge.descriptor.map_capture_key.grid_native_index);
  EXPECT_TRUE(bridge.descriptor.map_capture_key.complete());
  EXPECT_TRUE(bridge.descriptor.closed_inflated_voxel_volume_metric);
  EXPECT_EQ(bridge.descriptor.clearance_metric,
            "closed_inflated_voxel_volume");

  capture.reset();
  const phase_offset_navigation::TubeFreeBallQueryResult result =
      bridge.free_ball_query(Eigen::Vector3d::Constant(2.0), 0.5);
  EXPECT_EQ(result.status,
            phase_offset_navigation::DistanceStatus::KNOWN_FREE);
  EXPECT_TRUE(result.certified);
  EXPECT_FALSE(result.exact);
  EXPECT_TRUE(result.complete_support);
  EXPECT_DOUBLE_EQ(result.certified_radius, 0.5);
  EXPECT_DOUBLE_EQ(result.support.radius, 0.5);
  ASSERT_EQ(result.support.voxel_footprint.size(), 1U);
  EXPECT_EQ(result.support.voxel_footprint.front().min_index_x, 1);
  EXPECT_EQ(result.support.voxel_footprint.front().max_index_x, 2);
  EXPECT_EQ(result.support.voxel_footprint.front().source_offset_x, 0);
  EXPECT_FALSE(result.support.voxel_footprint.front().native_index);
}

TEST(CloudOccupancyQueryTest, V2BridgeRejectsUnsupportedOrMismatchedSupport) {
  const std::shared_ptr<const plan_env::SDFMapCaptureV2> capture =
      makeV2Capture();
  const SDFMapCaptureQueryStatusV2 valid =
      inspectSDFMapCaptureQueryV2(capture);
  EXPECT_TRUE(valid.usable);
  EXPECT_TRUE(valid.closed_inflated_voxel_volume_metric);

  std::shared_ptr<plan_env::SDFMapCaptureV2> notification_regressed(
      new plan_env::SDFMapCaptureV2(*capture));
  notification_regressed->accepted_state_notification_sequence = 10U;
  EXPECT_FALSE(inspectSDFMapCaptureQueryV2(notification_regressed).usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> unsupported(
      new plan_env::SDFMapCaptureV2(*capture));
  unsupported->support.evidence_basis =
      plan_env::kSDFMapCaptureSupportEvidenceDepthRaycast;
  EXPECT_FALSE(inspectSDFMapCaptureQueryV2(unsupported).usable);
  EXPECT_EQ(makeSDFMapCaptureQueryV2(unsupported).free_ball_query(
                Eigen::Vector3d::Constant(2.0), 0.5).status,
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);

  std::shared_ptr<plan_env::SDFMapCaptureV2> frame_changed(
      new plan_env::SDFMapCaptureV2(*capture));
  frame_changed->support.frame_id = "other_frame";
  EXPECT_FALSE(inspectSDFMapCaptureQueryV2(frame_changed).usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> configuration_changed(
      new plan_env::SDFMapCaptureV2(*capture));
  configuration_changed->support.configuration_key = 20U;
  EXPECT_FALSE(inspectSDFMapCaptureQueryV2(configuration_changed).usable);
}

TEST(CloudOccupancyQueryTest, V2BridgePreservesFiniteSupportExpiryTicks) {
  std::shared_ptr<plan_env::SDFMapCaptureV2> capture(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  capture->support.valid_until = ros::Time(22.0);
  capture->support.valid_until_accepted_ticks = 150U;
  const SDFMapCaptureQueryBridgeV2 bridge =
      makeSDFMapCaptureQueryV2(capture);
  ASSERT_TRUE(bridge.usable());
  EXPECT_FALSE(bridge.status.support_expiry_timeless);
  EXPECT_EQ(bridge.status.support_expiry_ticks, 150U);
  EXPECT_EQ(bridge.descriptor.map_capture_key.support_expiry_ticks, 150U);
  EXPECT_EQ(bridge.descriptor.support_valid_until_ticks, 150U);
  const auto result = bridge.free_ball_query(Eigen::Vector3d::Constant(2.0),
                                             0.25);
  EXPECT_TRUE(result.certified);
  EXPECT_FALSE(result.support.support_expiry_timeless);
  EXPECT_EQ(result.support.support_expiry_ticks, 150U);
}

TEST(CloudOccupancyQueryTest, V2BridgeCropKeepsNativeOriginAndLocalRange) {
  const std::shared_ptr<const plan_env::SDFMapCaptureV2> full =
      makeV2Capture();
  std::shared_ptr<plan_env::SDFMapCaptureV2> crop(
      new plan_env::SDFMapCaptureV2(*full));
  crop->capture_min = Eigen::Vector3d::Constant(1.0);
  crop->capture_max = Eigen::Vector3d::Constant(3.0);
  crop->source_min_index = Eigen::Vector3i::Constant(1);
  crop->source_max_index = Eigen::Vector3i::Constant(2);
  crop->voxel_count = Eigen::Vector3i::Constant(2);
  crop->occupied.assign(8U, 0U);
  crop->support.support_min = crop->capture_min;
  crop->support.support_max = crop->capture_max;
  crop->support.mask.assign(8U, 1U);
  const SDFMapCaptureQueryBridgeV2 full_bridge =
      makeSDFMapCaptureQueryV2(full);
  const SDFMapCaptureQueryBridgeV2 crop_bridge =
      makeSDFMapCaptureQueryV2(crop);
  ASSERT_TRUE(full_bridge.usable());
  ASSERT_TRUE(crop_bridge.usable());
  const Eigen::Vector3d witness = Eigen::Vector3d::Constant(2.0);
  const auto full_result = full_bridge.free_ball_query(witness, 0.25);
  const auto crop_result = crop_bridge.free_ball_query(witness, 0.25);
  EXPECT_EQ(crop_result.status, full_result.status);
  EXPECT_TRUE(crop_result.certified);
  ASSERT_EQ(crop_result.support.voxel_footprint.size(), 1U);
  EXPECT_EQ(crop_result.support.voxel_footprint.front().min_index_x, 0);
  EXPECT_EQ(crop_result.support.voxel_footprint.front().max_index_x, 1);
  EXPECT_EQ(crop_result.support.voxel_footprint.front().source_offset_x, 1);
  EXPECT_TRUE(crop_result.support.voxel_footprint.front().native_origin.isApprox(
      full->grid_origin));
  EXPECT_FALSE(crop_result.support.voxel_footprint.front().native_index);
}

TEST(CloudOccupancyQueryTest,
     V2BridgeNonUnitAndNondyadicCropRangesAgreeWithFullCapture) {
  for (const double resolution : {2.0, 0.3}) {
    auto make_capture = [resolution](const bool cropped) {
      std::shared_ptr<plan_env::SDFMapCaptureV2> capture(
          new plan_env::SDFMapCaptureV2());
      capture->valid = true;
      capture->map_instance_id = 77U;
      capture->configuration_generation = 3U;
      capture->configuration_key = 19U;
      capture->frame_id = "world";
      capture->accepted_state_sequence = 11U;
      capture->accepted_time_ticks = 101U;
      capture->map_min = Eigen::Vector3d::Zero();
      capture->map_max = Eigen::Vector3d::Constant(4.0 * resolution);
      capture->grid_origin = Eigen::Vector3d::Zero();
      capture->capture_min = cropped
          ? Eigen::Vector3d::Constant(resolution)
          : Eigen::Vector3d::Zero();
      capture->capture_max = Eigen::Vector3d::Constant(4.0 * resolution);
      capture->source_min_index = cropped
          ? Eigen::Vector3i::Constant(1) : Eigen::Vector3i::Zero();
      capture->source_max_index = Eigen::Vector3i::Constant(3);
      capture->voxel_count = cropped
          ? Eigen::Vector3i::Constant(3) : Eigen::Vector3i::Constant(4);
      capture->resolution = resolution;
      const std::size_t voxel_total = cropped ? 27U : 64U;
      capture->occupied.assign(voxel_total, 0U);
      capture->support.valid = true;
      capture->support.complete = true;
      capture->support.evidence_basis =
          plan_env::kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
      capture->support.evidence_sequence = 11U;
      capture->support.evidence_accepted_ticks = 101U;
      capture->support.map_instance_id = 77U;
      capture->support.configuration_generation = 3U;
      capture->support.configuration_key = 19U;
      capture->support.frame_id = "world";
      capture->support.support_min = capture->map_min;
      capture->support.support_max = capture->map_max;
      capture->support.halo_reconciled = true;
      capture->support.mask.assign(voxel_total, 1U);
      // The occupied source voxel is (1,1,3), which is local (0,0,2) in the
      // crop.  The free witness below is separated from it in z.
      if (cropped) {
        capture->occupied[(0U * 3U + 0U) * 3U + 2U] = 1U;
      } else {
        capture->occupied[(1U * 4U + 1U) * 4U + 3U] = 1U;
      }
      return std::shared_ptr<const plan_env::SDFMapCaptureV2>(capture);
    };

    const auto full_bridge = makeSDFMapCaptureQueryV2(make_capture(false));
    const auto crop_bridge = makeSDFMapCaptureQueryV2(make_capture(true));
    ASSERT_TRUE(full_bridge.usable());
    ASSERT_TRUE(crop_bridge.usable());
    const Eigen::Vector3d free_witness =
        Eigen::Vector3d::Constant(2.0 * resolution);
    const double radius = resolution / 4.0;
    const auto full_free = full_bridge.free_ball_query(free_witness, radius);
    const auto crop_free = crop_bridge.free_ball_query(free_witness, radius);
    EXPECT_EQ(crop_free.status, full_free.status);
    EXPECT_EQ(crop_free.status,
              phase_offset_navigation::DistanceStatus::KNOWN_FREE);
    EXPECT_TRUE(full_free.certified);
    EXPECT_TRUE(crop_free.certified);
    ASSERT_EQ(full_free.support.voxel_footprint.size(), 1U);
    ASSERT_EQ(crop_free.support.voxel_footprint.size(), 1U);
    EXPECT_EQ(full_free.support.voxel_footprint.front().min_index_x, 1);
    EXPECT_EQ(full_free.support.voxel_footprint.front().max_index_x, 2);
    EXPECT_EQ(crop_free.support.voxel_footprint.front().min_index_x, 0);
    EXPECT_EQ(crop_free.support.voxel_footprint.front().max_index_x, 1);
    EXPECT_EQ(crop_free.support.voxel_footprint.front().source_offset_x, 1);
    EXPECT_TRUE(crop_free.support.voxel_footprint.front().native_origin.isApprox(
        Eigen::Vector3d::Zero()));

    const Eigen::Vector3d occupied_witness(
        1.5 * resolution, 1.5 * resolution, 3.5 * resolution);
    const auto full_occupied =
        full_bridge.free_ball_query(occupied_witness, 0.0);
    const auto crop_occupied =
        crop_bridge.free_ball_query(occupied_witness, 0.0);
    EXPECT_EQ(full_occupied.status,
              phase_offset_navigation::DistanceStatus::OCCUPIED);
    EXPECT_EQ(crop_occupied.status, full_occupied.status);
    ASSERT_EQ(crop_occupied.support.voxel_footprint.size(), 0U);
  }
}

TEST(CloudOccupancyQueryTest,
     V2BridgeUsesExactRadiusAndClosedVoxelTangentSemantics) {
  std::shared_ptr<plan_env::SDFMapCaptureV2> mutable_capture(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  // Occupied voxel [0,1] x [0,1] x [0,1], with a query point 0.25 m away
  // in x.  The closed-volume tangent at radius=.25 is free; one ULP above it
  // is not certified by the exact predicate.
  mutable_capture->occupied[0U] = 1U;
  const SDFMapCaptureQueryBridgeV2 bridge =
      makeSDFMapCaptureQueryV2(mutable_capture);
  const Eigen::Vector3d tangent(1.25, 0.5, 0.5);
  const auto at_tangent = bridge.free_ball_query(tangent, 0.25);
  EXPECT_EQ(at_tangent.status,
            phase_offset_navigation::DistanceStatus::KNOWN_FREE);
  EXPECT_TRUE(at_tangent.certified);
  EXPECT_DOUBLE_EQ(at_tangent.support.radius, 0.25);

  const double above =
      std::nextafter(0.25, std::numeric_limits<double>::infinity());
  const auto above_tangent = bridge.free_ball_query(tangent, above);
  EXPECT_NE(above_tangent.status,
            phase_offset_navigation::DistanceStatus::KNOWN_FREE);
  EXPECT_FALSE(above_tangent.certified);
}

TEST(CloudOccupancyQueryTest,
     V2BridgeRejectsNullDefaultAndMalformedCaptureValues) {
  const std::shared_ptr<const plan_env::SDFMapCaptureV2> absent;
  const SDFMapCaptureQueryStatusV2 absent_status =
      inspectSDFMapCaptureQueryV2(absent);
  EXPECT_FALSE(absent_status.capture_available);
  EXPECT_FALSE(absent_status.usable);
  const SDFMapCaptureQueryBridgeV2 absent_bridge =
      makeSDFMapCaptureQueryV2(absent);
  EXPECT_FALSE(absent_bridge.usable());
  EXPECT_EQ(absent_bridge.free_ball_query(
                Eigen::Vector3d::Constant(2.0), 0.25).status,
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);

  const std::shared_ptr<const plan_env::SDFMapCaptureV2> defaults(
      new plan_env::SDFMapCaptureV2());
  const SDFMapCaptureQueryStatusV2 default_status =
      inspectSDFMapCaptureQueryV2(defaults);
  EXPECT_TRUE(default_status.capture_available);
  EXPECT_FALSE(default_status.capture_consistent);
  EXPECT_FALSE(default_status.usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> nonfinite_resolution(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  nonfinite_resolution->resolution = std::numeric_limits<double>::quiet_NaN();
  const SDFMapCaptureQueryStatusV2 nonfinite_status =
      inspectSDFMapCaptureQueryV2(nonfinite_resolution);
  EXPECT_FALSE(nonfinite_status.geometry_valid);
  EXPECT_FALSE(nonfinite_status.capture_consistent);
  EXPECT_FALSE(nonfinite_status.usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> empty_axis(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  empty_axis->voxel_count.x() = 0;
  EXPECT_FALSE(inspectSDFMapCaptureQueryV2(empty_axis).geometry_valid);
  EXPECT_FALSE(inspectSDFMapCaptureQueryV2(empty_axis).usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> reversed_map(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  reversed_map->map_max.x() = reversed_map->map_min.x();
  EXPECT_FALSE(inspectSDFMapCaptureQueryV2(reversed_map).geometry_valid);
  EXPECT_FALSE(inspectSDFMapCaptureQueryV2(reversed_map).usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> missing_occupancy(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  missing_occupancy->occupied.pop_back();
  const SDFMapCaptureQueryStatusV2 occupancy_status =
      inspectSDFMapCaptureQueryV2(missing_occupancy);
  EXPECT_FALSE(occupancy_status.capture_consistent);
  EXPECT_FALSE(occupancy_status.support_metadata_valid);
  EXPECT_FALSE(occupancy_status.usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> missing_mask(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  missing_mask->support.mask.pop_back();
  const SDFMapCaptureQueryStatusV2 mask_status =
      inspectSDFMapCaptureQueryV2(missing_mask);
  EXPECT_FALSE(mask_status.support_metadata_valid);
  EXPECT_FALSE(mask_status.capture_consistent);
  EXPECT_FALSE(mask_status.usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> unknown_layer(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  unknown_layer->effective_layer_mask = (std::uint32_t(1U) << 31U);
  const SDFMapCaptureQueryStatusV2 layer_status =
      inspectSDFMapCaptureQueryV2(unknown_layer);
  EXPECT_FALSE(layer_status.geometry_valid);
  EXPECT_FALSE(layer_status.usable);
}

TEST(CloudOccupancyQueryTest,
     V2BridgeRejectsInvalidSupportEvidenceAndExpiryMetadata) {
  std::shared_ptr<plan_env::SDFMapCaptureV2> invalid_support(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  invalid_support->support.valid = false;
  const SDFMapCaptureQueryStatusV2 invalid_status =
      inspectSDFMapCaptureQueryV2(invalid_support);
  EXPECT_FALSE(invalid_status.support_valid);
  EXPECT_FALSE(invalid_status.support_metadata_valid);
  EXPECT_FALSE(invalid_status.usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> incomplete_support(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  incomplete_support->support.complete = false;
  const SDFMapCaptureQueryStatusV2 incomplete_status =
      inspectSDFMapCaptureQueryV2(incomplete_support);
  EXPECT_FALSE(incomplete_status.support_complete);
  EXPECT_FALSE(incomplete_status.support_metadata_valid);
  EXPECT_FALSE(incomplete_status.usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> missing_evidence(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  missing_evidence->support.evidence_sequence = 0U;
  const SDFMapCaptureQueryStatusV2 evidence_status =
      inspectSDFMapCaptureQueryV2(missing_evidence);
  EXPECT_FALSE(evidence_status.support_metadata_valid);
  EXPECT_FALSE(evidence_status.usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> unsupported_evidence(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  unsupported_evidence->support.evidence_basis =
      plan_env::kSDFMapCaptureSupportEvidenceNone;
  EXPECT_FALSE(inspectSDFMapCaptureQueryV2(unsupported_evidence).usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> ungrounded_expiry(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  ungrounded_expiry->support.valid_until = ros::Time();
  ungrounded_expiry->support.valid_until_accepted_ticks = 150U;
  const SDFMapCaptureQueryStatusV2 ungrounded_status =
      inspectSDFMapCaptureQueryV2(ungrounded_expiry);
  EXPECT_FALSE(ungrounded_status.expiry_metadata_valid);
  EXPECT_FALSE(ungrounded_status.capture_consistent);
  EXPECT_FALSE(ungrounded_status.usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> unticked_expiry(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  unticked_expiry->support.valid_until = ros::Time(22.0);
  unticked_expiry->support.valid_until_accepted_ticks = 0U;
  const SDFMapCaptureQueryStatusV2 unticked_status =
      inspectSDFMapCaptureQueryV2(unticked_expiry);
  EXPECT_FALSE(unticked_status.expiry_metadata_valid);
  EXPECT_FALSE(unticked_status.capture_consistent);
  EXPECT_FALSE(unticked_status.usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> expired_at_capture(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  expired_at_capture->support.valid_until = ros::Time(22.0);
  expired_at_capture->support.valid_until_accepted_ticks =
      expired_at_capture->accepted_time_ticks - 1U;
  const SDFMapCaptureQueryStatusV2 expired_status =
      inspectSDFMapCaptureQueryV2(expired_at_capture);
  EXPECT_FALSE(expired_status.expiry_metadata_valid);
  EXPECT_FALSE(expired_status.capture_consistent);
  EXPECT_FALSE(expired_status.usable);

  std::shared_ptr<plan_env::SDFMapCaptureV2> expiry_before_evidence(
      new plan_env::SDFMapCaptureV2(*makeV2Capture()));
  expiry_before_evidence->support.valid_until = ros::Time(22.0);
  expiry_before_evidence->support.valid_until_accepted_ticks =
      expiry_before_evidence->support.evidence_accepted_ticks - 1U;
  EXPECT_FALSE(inspectSDFMapCaptureQueryV2(expiry_before_evidence).usable);
}

TEST(CloudOccupancyQueryTest, V2BridgeRejectsNonfiniteAndNegativeQueries) {
  const SDFMapCaptureQueryBridgeV2 bridge =
      makeSDFMapCaptureQueryV2(makeV2Capture());
  ASSERT_TRUE(bridge.usable());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto nan_witness = bridge.free_ball_query(
      Eigen::Vector3d(nan, 2.0, 2.0), 0.25);
  EXPECT_EQ(nan_witness.status,
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);
  EXPECT_FALSE(nan_witness.certified);
  EXPECT_FALSE(nan_witness.complete_support);

  const auto nan_radius = bridge.free_ball_query(
      Eigen::Vector3d::Constant(2.0), nan);
  EXPECT_EQ(nan_radius.status,
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);
  EXPECT_FALSE(nan_radius.certified);

  const auto negative_radius = bridge.free_ball_query(
      Eigen::Vector3d::Constant(2.0), -0.01);
  EXPECT_EQ(negative_radius.status,
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);
  EXPECT_FALSE(negative_radius.certified);
}

TEST(CloudOccupancyQueryTest,
     V2BridgeRejectsUnsupportedRoundingAtConstructionAndQuery) {
  ASSERT_EQ(std::fegetround(), FE_TONEAREST);
  const std::shared_ptr<const plan_env::SDFMapCaptureV2> capture =
      makeV2Capture();
  ScopedRoundingMode construction_mode(FE_DOWNWARD);
  ASSERT_TRUE(construction_mode.changed());
  const SDFMapCaptureQueryStatusV2 status =
      inspectSDFMapCaptureQueryV2(capture);
  EXPECT_FALSE(status.floating_point_environment_supported);
  EXPECT_FALSE(status.usable);
  const SDFMapCaptureQueryBridgeV2 bridge =
      makeSDFMapCaptureQueryV2(capture);
  EXPECT_FALSE(bridge.usable());
  EXPECT_EQ(bridge.free_ball_query(Eigen::Vector3d::Constant(2.0), 0.25).status,
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);

  ASSERT_EQ(std::fesetround(FE_TONEAREST), 0);
  const SDFMapCaptureQueryBridgeV2 valid_bridge =
      makeSDFMapCaptureQueryV2(capture);
  ASSERT_TRUE(valid_bridge.usable());
  ASSERT_EQ(std::fesetround(FE_DOWNWARD), 0);
  const auto postconstruction = valid_bridge.free_ball_query(
      Eigen::Vector3d::Constant(2.0), 0.25);
  EXPECT_EQ(postconstruction.status,
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);
  EXPECT_FALSE(postconstruction.certified);
}

#if defined(__i386__) || defined(__x86_64__)
TEST(CloudOccupancyQueryTest, V2BridgeRejectsFTZAndDAZWithRestoredEnvironment) {
  ASSERT_TRUE(plan_env::sdfMapCaptureV2FloatingPointEnvironmentSupported());
  const std::shared_ptr<const plan_env::SDFMapCaptureV2> capture =
      makeV2Capture();
  const SDFMapCaptureQueryBridgeV2 valid_bridge =
      makeSDFMapCaptureQueryV2(capture);
  ASSERT_TRUE(valid_bridge.usable());
  const unsigned int flush_zero = 0x8000U;
  const unsigned int denormals_are_zero = 0x0040U;
  for (const unsigned int unsupported_bits :
       {flush_zero, denormals_are_zero, flush_zero | denormals_are_zero}) {
    ScopedFlushToZeroMode unsupported_mode(unsupported_bits);
    ASSERT_TRUE(unsupported_mode.changed());
    const SDFMapCaptureQueryStatusV2 status =
        inspectSDFMapCaptureQueryV2(capture);
    EXPECT_FALSE(status.floating_point_environment_supported);
    EXPECT_FALSE(status.usable);
    const SDFMapCaptureQueryBridgeV2 bridge =
        makeSDFMapCaptureQueryV2(capture);
    EXPECT_FALSE(bridge.usable());
    EXPECT_EQ(bridge.free_ball_query(
                  Eigen::Vector3d::Constant(2.0), 0.25).status,
              phase_offset_navigation::DistanceStatus::UNAVAILABLE);
    const auto postconstruction = valid_bridge.free_ball_query(
        Eigen::Vector3d::Constant(2.0), 0.25);
    EXPECT_EQ(postconstruction.status,
              phase_offset_navigation::DistanceStatus::UNAVAILABLE);
    EXPECT_FALSE(postconstruction.certified);
  }
}
#endif

TEST(CloudOccupancyQueryTest,
     V2BridgeFeedsImmutableCaptureIntoTubeCertificateBuilder) {
  const SDFMapCaptureQueryBridgeV2 bridge =
      makeSDFMapCaptureQueryV2(makeV2Capture());
  ASSERT_TRUE(bridge.usable());

  phase_offset_navigation::TubeCertificateConfigV2 config;
  config.configuration_id = 19U;
  config.epsilon = 0.10;
  config.nominal_half_width = 0.20;
  config.ray_step = 0.10;
  config.snapshot_resolution = 1.0;
  config.minimum_reference_speed = 0.50;
  config.sample_step_w = 1.0;
  ASSERT_TRUE(config.complete());

  phase_offset_navigation::TubeBuildInputV2 input;
  input.request_id = 1U;
  input.path_key.execution_generation = 1U;
  input.path_key.path_instance_id = 2U;
  input.path_key.path_revision = 1U;
  input.path_key.frame_revision = 1U;
  input.path_key.frame_convention_id = 1U;
  input.path_key.frame_convention =
      phase_offset_core::kWorldHorizontalCrossProductProvenance;
  input.path_key.phase_orientation = 1;
  input.path_key.domain_start = 0.0;
  input.path_key.domain_end = 1.0;
  input.configuration_key = config.key();
  input.map_capture_key = bridge.descriptor.map_capture_key;
  input.requested_start = 0.0;
  input.requested_end = 1.0;
  input.anchor_w = 1.0;
  // Immutable analytic path p(w)=(2+w,2,2), w in [0,1], with a timeless
  // complete preknown static-domain capture; no operational timeless claim
  // is inferred from this test fixture.
  const std::shared_ptr<const phase_offset_core::CertifiedPathCellV2>
      path_owner(new phase_offset_core::CertifiedPathCellV2(
          makeStraightV2PathCell()));
  input.path_cells.push_back(*path_owner);
  input.producer_breakpoints = {0.0, 1.0};
  input.path_owner = std::static_pointer_cast<const void>(path_owner);
  input.query_owner = bridge.query_owner;
  input.capture_owner = bridge.capture_owner;
  input.applicability_assumptions = "immutable authoritative V2 capture";
  input.applicability_deadline_timeless = true;
  input.free_ball_query = bridge.free_ball_query;

  phase_offset_navigation::TubeCertificateBuilderV2 builder(config);
  phase_offset_navigation::TubeCertificateBuildResultV2 build;
  ASSERT_TRUE(builder.build(input, build));
  ASSERT_TRUE(build.success);
  ASSERT_TRUE(build.profile.structurallyValid());
  EXPECT_EQ(build.profile.map_capture_key,
            bridge.descriptor.map_capture_key);
  EXPECT_EQ(build.profile.map_capture_key.frame_provenance, "world");
  EXPECT_EQ(build.profile.map_capture_key.configuration_id, 19U);
  EXPECT_GT(build.stats.query_count, 0U);
  EXPECT_EQ(build.stats.failed_query_count, 0U);
}

TEST(CloudOccupancyQueryTest,
     ClearanceBridgeUsesPlannerEsdfBaseCentersAndObservedBall) {
  const auto snapshot = makeSnapshot();
  const auto query = makeCloudOccupancyClearanceQuery(snapshot, usableConfig());
  const auto occupied = query(Eigen::Vector3d(1.1, 0.1, 0.1), 0.45);
  EXPECT_EQ(occupied.status, phase_offset_navigation::DistanceStatus::OCCUPIED);
  EXPECT_DOUBLE_EQ(occupied.clearance, 0.0);
  EXPECT_FALSE(occupied.clearance_certified);

  const auto unknown = query(Eigen::Vector3d(1.8, 0.0, 0.0), 0.30);
  EXPECT_EQ(unknown.status, phase_offset_navigation::DistanceStatus::UNKNOWN);
  EXPECT_FALSE(unknown.clearance_certified);

  const auto free = query(Eigen::Vector3d(-1.5, 0.0, 0.0), 0.20);
  EXPECT_EQ(free.status, phase_offset_navigation::DistanceStatus::KNOWN_FREE);
  EXPECT_TRUE(free.clearance_certified);
  EXPECT_GE(free.clearance, 0.20);

  CloudOccupancyQueryConfig disabled = usableConfig();
  disabled.obstacle_set_complete = false;
  EXPECT_EQ(makeCloudOccupancyClearanceQuery(snapshot, disabled)(
                Eigen::Vector3d::Zero(), 0.45).status,
            phase_offset_navigation::DistanceStatus::UNAVAILABLE);
}

TEST(CloudOccupancyQueryTest, ClearanceBridgeSelectsPlannerEsdfBasePrimitive) {
  const auto snapshot = makeCenterMetricSnapshot();
  const auto query = makeCloudOccupancyClearanceQuery(snapshot, usableConfig());
  const auto result = query(Eigen::Vector3d::Zero(), 1.80);
  EXPECT_EQ(result.status, phase_offset_navigation::DistanceStatus::KNOWN_FREE);
  EXPECT_TRUE(result.clearance_certified);
  // The occupied voxel is [1,2] x [0,1] x [0,1], so its planner-ESDF-base
  // centre is (1.5, 0.5, 0.5), 1.658... m from the query point.  The adapter
  // must no longer route to the old closed-volume distance of 1 m.
  EXPECT_NEAR(result.clearance, std::sqrt(2.75), 1e-12);
}

TEST(CloudOccupancyQueryTest,
     ClearanceBridgeFeedsOneCoherentMetricIntoCrossSection) {
  const auto snapshot = makeFrozenGapSnapshot();
  const Eigen::Vector3d frozen_point(2.6643275039478231,
                                     0.27533414135136092,
                                     0.99994409891574243);
  const plan_env::CloudOccupancySnapshotClearanceResult old_clearance =
      plan_env::queryCloudOccupancySnapshotClearance(
          *snapshot, frozen_point, 0.50);
  const plan_env::CloudOccupancySnapshotPlannerEsdfBaseClearanceResult
      planner_esdf_base_clearance =
          plan_env::queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
              *snapshot, frozen_point, 0.50);
  ASSERT_EQ(old_clearance.status, plan_env::CloudOccupancyStatus::KNOWN_FREE);
  ASSERT_EQ(planner_esdf_base_clearance.status,
            plan_env::CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_NEAR(old_clearance.nearest_occupied_voxel_volume_distance,
              0.39910242275510055, 1e-12);
  EXPECT_NEAR(
      planner_esdf_base_clearance
          .nearest_inflated_occupied_voxel_center_distance,
      0.46581958181362049, 1e-12);
  EXPECT_LT(old_clearance.nearest_occupied_voxel_volume_distance, 0.40);
  EXPECT_GT(
      planner_esdf_base_clearance
          .nearest_inflated_occupied_voxel_center_distance,
      0.40);

  const auto query = makeCloudOccupancyClearanceQuery(snapshot, usableConfig());
  phase_offset_navigation::TubeCrossSectionConfig cross_section_config;
  cross_section_config.nominal_half_width = 0.50;
  cross_section_config.ray_step = 0.05;
  cross_section_config.planner_safe_distance = 0.40;

  phase_offset_navigation::TubeCrossSectionInput input;
  input.p = frozen_point;
  input.N = Eigen::Vector3d::UnitY();
  input.clearance_query = query;
  const phase_offset_navigation::TubeCrossSectionResult result =
      phase_offset_navigation::TubeCrossSectionSolver(cross_section_config)
          .solve(input);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.contains_zero);
  EXPECT_GT(result.width, 0.0);
  EXPECT_NE(result.reason,
            phase_offset_navigation::TubeCrossSectionReason::
                EMPTY_AFTER_OBSTACLE_BOUNDS);
}

TEST(CloudOccupancyQueryTest, DiagnosticsKeepFullAndResidualAccountingSeparate) {
  const auto snapshot = makeSnapshot();
  const CloudOccupancyQueryConfig config = usableConfig();
  const auto query = makeCloudOccupancyQuery(snapshot, config);
  phase_offset_navigation::TubeProfile candidate;
  phase_offset_navigation::TubeRawSample sample;
  sample.w = 1.0;
  sample.p = Eigen::Vector3d::Zero();
  sample.N = Eigen::Vector3d::UnitY();
  candidate.samples.push_back(sample);

  CloudSnapshotDiagnosticsInput input;
  input.query_status = inspectCloudOccupancyQuery(snapshot, config);
  input.query_config = config;
  input.margins = margins();
  input.candidate_profile = &candidate;
  input.current_w = 1.0;
  input.current_path.w = 1.0;
  input.current_path.p = sample.p;
  input.actual_position = sample.p;
  input.categorical_query = query;
  input.ray_step = 0.05;
  const auto values = makeCloudSnapshotDiagnostics(input);
  ASSERT_EQ(values.size(), kCloudSnapshotDiagnosticCount);
  EXPECT_NEAR(values[kCloudSnapshotFullEffectiveRadius], 0.55, 1e-12);
  EXPECT_NEAR(values[kCloudSnapshotConfiguredPreincludedMapUncertainty], 0.10,
              1e-12);
  EXPECT_NEAR(values[kCloudSnapshotResidualEffectiveRadius], 0.45, 1e-12);
  EXPECT_DOUBLE_EQ(values[kCloudSnapshotRawStorageAccessed], 0.0);
  EXPECT_DOUBLE_EQ(values[kCloudSnapshotSelfFreeSeedUsed], 0.0);
  for (const double value : values) EXPECT_TRUE(std::isfinite(value));

  std::set<std::string> names;
  for (const char* name : cloudSnapshotDiagnosticFieldNames()) {
    ASSERT_NE(name, nullptr);
    EXPECT_TRUE(names.insert(name).second);
  }
}

}  // namespace
}  // namespace FLAG_Race

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
