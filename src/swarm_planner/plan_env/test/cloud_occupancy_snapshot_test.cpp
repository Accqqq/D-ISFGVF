#include <gtest/gtest.h>

#include <plan_env/cloud_occupancy_snapshot.h>

#define private public
#include <plan_env/sdf_map.h>
#undef private

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/make_shared.hpp>

#include <algorithm>
#include <cmath>
#include <cfenv>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <thread>
#include <atomic>

#if defined(__i386__) || defined(__x86_64__)
#include <immintrin.h>
#endif

// Exercise the authorized producer's actual predicate in T03, including empty
// observations.  No external smoke target and no copied test-only algorithm.
#define main local_sensing_node_main_for_t03
#include "../../../uav_simulator/dynamic_map_generator/src/local_sensing.cpp"
#undef main

namespace plan_env {
namespace {

CloudOccupancySnapshotBuildInput makeInput(
    std::uint64_t sequence = 1U) {
  CloudOccupancySnapshotBuildInput input;
  input.odom_valid = true;
  input.observation_sequence = sequence;
  input.observation_stamp = ros::Time(12.5 + static_cast<double>(sequence));
  input.camera_position = Eigen::Vector3d::Zero();
  input.map_min = Eigen::Vector3d(-5.0, -5.0, -5.0);
  input.map_max = Eigen::Vector3d(5.0, 5.0, 5.0);
  input.grid_origin = input.map_min;
  input.voxel_count = Eigen::Vector3i(10, 10, 10);
  input.local_update_range = Eigen::Vector3d(2.0, 2.0, 2.0);
  input.resolution = 1.0;
  input.obstacles_inflation = 0.99;
  return input;
}

void initializeCloudCallbackMap(SDFMap& map) {
  map.mp_.resolution_ = 1.0;
  map.mp_.resolution_inv_ = 1.0;
  map.mp_.map_origin_ = Eigen::Vector3d(-5.0, -5.0, -5.0);
  map.mp_.map_size_ = Eigen::Vector3d(10.0, 10.0, 10.0);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
  map.mp_.map_voxel_num_ = Eigen::Vector3i(10, 10, 10);
  map.mp_.local_update_range_ = Eigen::Vector3d(2.0, 2.0, 2.0);
  map.mp_.obstacles_inflation_ = 0.99;
  map.mp_.ground_height_ = -4.9;
  map.mp_.enable_manual_map_ = false;
  map.mp_.static_preinflated_map_enable_ = false;
  const std::size_t size = 1000U;
  map.md_.occupancy_buffer_.assign(size, -3.0);
  map.md_.occupancy_buffer_inflate_.assign(size, 0);
  map.md_.distance_buffer_all_.assign(size, 10000.0);
  map.md_.manual_occupancy_buffer_.assign(size, 0);
  map.md_.static_preinflated_buffer_.assign(size, 0);
  map.md_.has_odom_ = true;
  map.md_.camera_pos_ = Eigen::Vector3d::Zero();
}

sensor_msgs::PointCloud2 cloudMessage(const std::vector<Eigen::Vector3d>& points,
                                      const ros::Time& stamp) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const Eigen::Vector3d& point : points) {
    cloud.emplace_back(point.x(), point.y(), point.z());
  }
  cloud.width = cloud.size();
  cloud.height = 1U;
  cloud.is_dense = true;
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.stamp = stamp;
  return message;
}

CloudOccupancySnapshot clearanceSnapshot() {
  CloudOccupancySnapshot snapshot;
  snapshot.valid = true;
  snapshot.observation_sequence = 41U;
  snapshot.map_min = Eigen::Vector3d(-5.0, -5.0, -5.0);
  snapshot.map_max = Eigen::Vector3d(5.0, 5.0, 5.0);
  snapshot.observed_min = Eigen::Vector3d(-2.0, -2.0, -2.0);
  snapshot.observed_max = Eigen::Vector3d(2.0, 2.0, 2.0);
  snapshot.grid_origin = snapshot.map_min;
  snapshot.voxel_count = Eigen::Vector3i(10, 10, 10);
  snapshot.resolution = 1.0;
  snapshot.included_map_inflation = 0.10;
  snapshot.occupied.assign(1000U, 0U);
  return snapshot;
}

std::size_t clearanceAddress(const int x, const int y, const int z) {
  return (static_cast<std::size_t>(x) * 10U +
          static_cast<std::size_t>(y)) * 10U +
      static_cast<std::size_t>(z);
}

CloudOccupancySnapshot denseOnlySnapshot(
    const CloudOccupancySnapshot& source) {
  CloudOccupancySnapshot dense;
  dense.valid = source.valid;
  dense.observation_sequence = source.observation_sequence;
  dense.observation_stamp = source.observation_stamp;
  dense.map_min = source.map_min;
  dense.map_max = source.map_max;
  dense.observed_min = source.observed_min;
  dense.observed_max = source.observed_max;
  dense.grid_origin = source.grid_origin;
  dense.voxel_count = source.voxel_count;
  dense.resolution = source.resolution;
  dense.included_map_inflation = source.included_map_inflation;
  dense.occupied = source.occupied;
  return dense;
}

std::uint64_t doubleBits(const double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

void expectClearanceBitwiseEqual(
    const CloudOccupancySnapshotClearanceResult& dense,
    const CloudOccupancySnapshotClearanceResult& indexed) {
  EXPECT_EQ(indexed.status, dense.status);
  EXPECT_EQ(indexed.clearance_certified, dense.clearance_certified);
  EXPECT_EQ(doubleBits(indexed.nearest_occupied_voxel_volume_distance),
            doubleBits(dense.nearest_occupied_voxel_volume_distance));
}

void expectDenseAndIndexedClearanceEqual(
    const CloudOccupancySnapshot& dense,
    const CloudOccupancySnapshot& indexed, const Eigen::Vector3d& point,
    const double radius) {
  expectClearanceBitwiseEqual(
      queryCloudOccupancySnapshotClearance(dense, point, radius),
      queryCloudOccupancySnapshotClearance(indexed, point, radius));
}

void expectDenseAndIndexedPlannerClearanceEqual(
    const CloudOccupancySnapshot& dense,
    const CloudOccupancySnapshot& indexed, const Eigen::Vector3d& point,
    const double radius) {
  const CloudOccupancySnapshotPlannerEsdfBaseClearanceResult dense_result =
      queryCloudOccupancySnapshotPlannerEsdfBaseClearance(dense, point, radius);
  const CloudOccupancySnapshotPlannerEsdfBaseClearanceResult indexed_result =
      queryCloudOccupancySnapshotPlannerEsdfBaseClearance(indexed, point, radius);
  EXPECT_EQ(indexed_result.status, dense_result.status);
  EXPECT_EQ(indexed_result.clearance_certified,
            dense_result.clearance_certified);
  EXPECT_EQ(
      doubleBits(indexed_result
                     .nearest_inflated_occupied_voxel_center_distance),
      doubleBits(
          dense_result.nearest_inflated_occupied_voxel_center_distance));
}

CloudOccupancySnapshot plannerFixtureSnapshot() {
  CloudOccupancySnapshot snapshot;
  snapshot.valid = true;
  snapshot.observation_sequence = 904U;
  snapshot.map_min = Eigen::Vector3d(-10.0, -15.0, -0.01);
  snapshot.map_max = Eigen::Vector3d(6.0, 3.0, 2.99);
  snapshot.observed_min = snapshot.map_min;
  snapshot.observed_max = snapshot.map_max;
  snapshot.grid_origin = snapshot.map_min;
  snapshot.voxel_count = Eigen::Vector3i(160, 180, 30);
  snapshot.resolution = 0.1;
  snapshot.included_map_inflation = 0.1;
  snapshot.occupied.assign(160U * 180U * 30U, 0U);
  snapshot.occupied[(128U * 180U + 148U) * 30U + 10U] = 1U;
  return snapshot;
}

CloudOccupancySnapshotBuildInput randomizedBuildInput(
    const Eigen::Vector3i& voxel_count, const std::uint64_t sequence,
    std::mt19937& random) {
  CloudOccupancySnapshotBuildInput input;
  input.odom_valid = true;
  input.observation_sequence = sequence;
  input.observation_stamp = ros::Time(100.0 + static_cast<double>(sequence));
  input.voxel_count = voxel_count;
  input.resolution = 0.25;
  const Eigen::Vector3d map_size = input.resolution *
      voxel_count.cast<double>();
  input.map_min = -0.5 * map_size;
  input.map_max = 0.5 * map_size;
  input.grid_origin = input.map_min;
  input.camera_position = Eigen::Vector3d::Zero();
  input.local_update_range = 0.42 * map_size;
  input.obstacles_inflation = 0.249;
  std::uniform_real_distribution<double> coordinate(-0.95, 0.95);
  const int point_count = 16 + static_cast<int>(sequence % 17U);
  for (int index = 0; index < point_count; ++index) {
    input.cloud_points.emplace_back(
        coordinate(random) * input.local_update_range.x(),
        coordinate(random) * input.local_update_range.y(),
        coordinate(random) * input.local_update_range.z());
  }
  return input;
}

SDFMapCaptureV2 makeAuthoritativeCaptureFixture() {
  SDFMapCaptureV2 capture;
  capture.valid = true;
  capture.map_instance_id = 77U;
  capture.configuration_generation = 3U;
  capture.configuration_key = 19U;
  capture.frame_id = "world";
  capture.accepted_state_sequence = 11U;
  capture.accepted_time_ticks = 101U;
  capture.map_min = Eigen::Vector3d::Zero();
  capture.map_max = Eigen::Vector3d::Constant(4.0);
  capture.grid_origin = Eigen::Vector3d::Zero();
  capture.capture_min = Eigen::Vector3d::Zero();
  capture.capture_max = Eigen::Vector3d::Constant(4.0);
  capture.voxel_count = Eigen::Vector3i::Constant(4);
  capture.resolution = 1.0;
  capture.occupied.assign(64U, 0U);
  capture.support.valid = true;
  capture.support.complete = true;
  capture.support.evidence_basis =
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  capture.support.evidence_sequence = 11U;
  capture.support.evidence_accepted_ticks = capture.accepted_time_ticks;
  capture.support.map_instance_id = capture.map_instance_id;
  capture.support.configuration_generation = capture.configuration_generation;
  capture.support.configuration_key = capture.configuration_key;
  capture.support.frame_id = capture.frame_id;
  capture.support.support_min = capture.map_min;
  capture.support.support_max = capture.map_max;
  capture.support.required_halo = 0.0;
  capture.support.halo_reconciled = true;
  capture.support.mask.assign(64U, 1U);
  return capture;
}

void bindMapSupportToCurrentConfiguration(SDFMap& map) {
  const std::shared_ptr<const SDFMapCaptureV2> metadata =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(metadata);
  map.md_.authoritative_support_map_instance_id_v2_ =
      metadata->map_instance_id;
  map.md_.authoritative_support_configuration_generation_v2_ =
      metadata->configuration_generation;
  map.md_.authoritative_support_configuration_key_v2_ =
      metadata->configuration_key;
  map.md_.authoritative_support_frame_id_v2_ = metadata->frame_id;
  map.md_.authoritative_support_accepted_ticks_v2_ =
      metadata->accepted_time_ticks;
  map.authoritative_capture_mutex_v2_.support_binding_invalidated = false;
}

TEST(CloudOccupancySnapshotTest, NoOdomAndNoObservationAreInvalid) {
  CloudOccupancySnapshotBuildInput input = makeInput();
  input.odom_valid = false;
  const CloudOccupancySnapshot snapshot = buildCloudOccupancySnapshot(input);
  EXPECT_FALSE(snapshot.valid);
  EXPECT_FALSE(cloudOccupancySnapshotConsistent(snapshot));

  const CloudOccupancySnapshot absent;
  EXPECT_FALSE(cloudOccupancySnapshotConsistent(absent));
}

TEST(CloudOccupancySnapshotTest,
     DefaultAuthoritativeCaptureAndPredicateFailClosed) {
  SDFMap map;
  EXPECT_FALSE(map.captureAuthoritativeSDFMapV2());

  const SDFMapCaptureV2 absent;
  EXPECT_FALSE(sdfMapCaptureV2Consistent(absent));
  const SDFMapCaptureFreeBallResultV2 result =
      certifySDFMapCaptureFreeBallV2(
          absent, Eigen::Vector3d::Zero(), 0.0);
  EXPECT_EQ(result.status, SDFMapCaptureFreeBallStatusV2::UNAVAILABLE);
  EXPECT_FALSE(result.clearance_certified);
}

TEST(CloudOccupancySnapshotTest, EmptyCloudIsAValidAllFreeObservedBox) {
  const CloudOccupancySnapshot snapshot = buildCloudOccupancySnapshot(makeInput());
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(snapshot));
  EXPECT_EQ(queryCloudOccupancySnapshot(snapshot, Eigen::Vector3d::Zero()).status,
            CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_EQ(queryCloudOccupancySnapshot(snapshot, Eigen::Vector3d(2.5, 0.0, 0.0)).status,
            CloudOccupancyStatus::UNKNOWN);
  EXPECT_EQ(queryCloudOccupancySnapshot(snapshot, Eigen::Vector3d(5.1, 0.0, 0.0)).status,
            CloudOccupancyStatus::OUT_OF_MAP);
  EXPECT_EQ(snapshot.observation_sequence, 1U);
  EXPECT_TRUE(snapshot.observation_stamp == ros::Time(13.5));
}

TEST(CloudOccupancySnapshotTest, PointCloudUsesOneVoxelInflatedOccupancy) {
  CloudOccupancySnapshotBuildInput input = makeInput();
  input.cloud_points.emplace_back(1.1, 0.1, 0.1);
  const CloudOccupancySnapshot snapshot = buildCloudOccupancySnapshot(input);
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(snapshot));
  EXPECT_NEAR(snapshot.included_map_inflation, 1.0, 1e-12);
  EXPECT_EQ(queryCloudOccupancySnapshot(snapshot, Eigen::Vector3d(1.1, 0.1, 0.1)).status,
            CloudOccupancyStatus::OCCUPIED);
  EXPECT_EQ(queryCloudOccupancySnapshot(snapshot, Eigen::Vector3d(2.0, 0.1, 0.1)).status,
            CloudOccupancyStatus::OCCUPIED);
  EXPECT_EQ(queryCloudOccupancySnapshot(snapshot, Eigen::Vector3d(-1.9, 0.1, 0.1)).status,
            CloudOccupancyStatus::KNOWN_FREE);
}

TEST(CloudOccupancySnapshotTest, SequenceAndOldImmutableContentAreIndependent) {
  CloudOccupancySnapshotBuildInput first_input = makeInput(1U);
  first_input.cloud_points.emplace_back(1.1, 0.1, 0.1);
  const std::shared_ptr<const CloudOccupancySnapshot> first(
      new CloudOccupancySnapshot(buildCloudOccupancySnapshot(first_input)));
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(*first));
  const CloudOccupancyStatus old_status =
      queryCloudOccupancySnapshot(*first, Eigen::Vector3d(1.1, 0.1, 0.1)).status;

  CloudOccupancySnapshotBuildInput second_input = makeInput(2U);
  second_input.cloud_points.emplace_back(-1.1, 0.1, 0.1);
  const CloudOccupancySnapshot second = buildCloudOccupancySnapshot(second_input);
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(second));
  EXPECT_EQ(second.observation_sequence, 2U);
  EXPECT_EQ(queryCloudOccupancySnapshot(*first, Eigen::Vector3d(1.1, 0.1, 0.1)).status,
            old_status);
  EXPECT_EQ(queryCloudOccupancySnapshot(second, Eigen::Vector3d(1.1, 0.1, 0.1)).status,
            CloudOccupancyStatus::KNOWN_FREE);
}

TEST(CloudOccupancySnapshotTest, NonFiniteInputsFailClosed) {
  CloudOccupancySnapshotBuildInput camera_nan = makeInput();
  camera_nan.camera_position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(buildCloudOccupancySnapshot(camera_nan).valid);

  CloudOccupancySnapshotBuildInput cloud_nan = makeInput();
  cloud_nan.cloud_points.emplace_back(std::numeric_limits<double>::quiet_NaN(),
                                      0.0, 0.0);
  EXPECT_FALSE(buildCloudOccupancySnapshot(cloud_nan).valid);

  const CloudOccupancySnapshot snapshot = buildCloudOccupancySnapshot(makeInput());
  EXPECT_EQ(queryCloudOccupancySnapshot(
                snapshot,
                Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0))
                .status,
            CloudOccupancyStatus::OUT_OF_MAP);
}

TEST(CloudOccupancySnapshotTest, ClearanceQueryFailsClosedForInvalidAndDomainCases) {
  const CloudOccupancySnapshot invalid;
  EXPECT_EQ(queryCloudOccupancySnapshotClearance(
                invalid, Eigen::Vector3d::Zero(), 0.45).status,
            CloudOccupancyStatus::UNAVAILABLE);

  const CloudOccupancySnapshot snapshot = clearanceSnapshot();
  EXPECT_EQ(queryCloudOccupancySnapshotClearance(
                snapshot, Eigen::Vector3d(5.1, 0.0, 0.0), 0.10).status,
            CloudOccupancyStatus::OUT_OF_MAP);
  EXPECT_EQ(queryCloudOccupancySnapshotClearance(
                snapshot, Eigen::Vector3d(2.1, 0.0, 0.0), 0.10).status,
            CloudOccupancyStatus::UNKNOWN);
  EXPECT_EQ(queryCloudOccupancySnapshotClearance(
                snapshot, Eigen::Vector3d(1.8, 0.0, 0.0), 0.30).status,
            CloudOccupancyStatus::UNKNOWN);
  EXPECT_EQ(queryCloudOccupancySnapshotClearance(
                snapshot,
                Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0,
                                0.0),
                0.10).status,
            CloudOccupancyStatus::UNAVAILABLE);
  EXPECT_EQ(queryCloudOccupancySnapshotClearance(
                snapshot, Eigen::Vector3d::Zero(),
                std::numeric_limits<double>::infinity()).status,
            CloudOccupancyStatus::UNAVAILABLE);
}

TEST(CloudOccupancySnapshotTest, ClearanceUsesClosedVoxelVolumesNotCenters) {
  CloudOccupancySnapshot snapshot = clearanceSnapshot();
  // Index (6,5,5) is the closed box [1,2] x [0,1] x [0,1].
  snapshot.occupied[clearanceAddress(6, 5, 5)] = 1U;
  const CloudOccupancySnapshotClearanceResult result =
      queryCloudOccupancySnapshotClearance(
          snapshot, Eigen::Vector3d(0.0, 0.5, 0.5), 1.20);
  ASSERT_EQ(result.status, CloudOccupancyStatus::KNOWN_FREE);
  ASSERT_TRUE(result.clearance_certified);
  EXPECT_NEAR(result.nearest_occupied_voxel_volume_distance, 1.0, 1e-12);

  const CloudOccupancySnapshotClearanceResult occupied =
      queryCloudOccupancySnapshotClearance(
          snapshot, Eigen::Vector3d(1.0, 0.5, 0.5), 0.10);
  EXPECT_EQ(occupied.status, CloudOccupancyStatus::OCCUPIED);
  EXPECT_DOUBLE_EQ(occupied.nearest_occupied_voxel_volume_distance, 0.0);
}

TEST(CloudOccupancySnapshotTest, ClearanceFindsDiagonalVoxelAndCapsCertifiedFree) {
  CloudOccupancySnapshot snapshot = clearanceSnapshot();
  // [1,2] x [1,2] x [0,1]: the closest point from the query is its corner.
  snapshot.occupied[clearanceAddress(6, 6, 5)] = 1U;
  const CloudOccupancySnapshotClearanceResult diagonal =
      queryCloudOccupancySnapshotClearance(
          snapshot, Eigen::Vector3d(0.0, 0.0, 0.5), 1.50);
  ASSERT_EQ(diagonal.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_NEAR(diagonal.nearest_occupied_voxel_volume_distance,
              std::sqrt(2.0), 1e-12);

  snapshot.occupied.assign(snapshot.occupied.size(), 0U);
  const CloudOccupancySnapshotClearanceResult empty =
      queryCloudOccupancySnapshotClearance(
          snapshot, Eigen::Vector3d::Zero(), 0.45);
  ASSERT_EQ(empty.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_TRUE(empty.clearance_certified);
  EXPECT_NEAR(empty.nearest_occupied_voxel_volume_distance, 0.45, 1e-12);
}

TEST(CloudOccupancySnapshotTest,
     PlannerEsdfBaseClearanceUsesOccupiedVoxelCenters) {
  CloudOccupancySnapshot snapshot = clearanceSnapshot();
  snapshot.occupied[clearanceAddress(6, 5, 5)] = 1U;

  const CloudOccupancySnapshotPlannerEsdfBaseClearanceResult free =
      queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
          snapshot, Eigen::Vector3d::Zero(), 1.80);
  ASSERT_EQ(free.status, CloudOccupancyStatus::KNOWN_FREE);
  ASSERT_TRUE(free.clearance_certified);
  EXPECT_NEAR(free.nearest_inflated_occupied_voxel_center_distance,
              std::sqrt(2.75), 1e-12);

  const CloudOccupancySnapshotPlannerEsdfBaseClearanceResult occupied =
      queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
          snapshot, Eigen::Vector3d(1.5, 0.5, 0.5), 0.10);
  EXPECT_EQ(occupied.status, CloudOccupancyStatus::OCCUPIED);
  EXPECT_DOUBLE_EQ(
      occupied.nearest_inflated_occupied_voxel_center_distance, 0.0);
  EXPECT_FALSE(occupied.clearance_certified);

  snapshot.occupied.assign(snapshot.occupied.size(), 0U);
  const CloudOccupancySnapshotPlannerEsdfBaseClearanceResult empty =
      queryCloudOccupancySnapshotPlannerEsdfBaseClearance(
          snapshot, Eigen::Vector3d::Zero(), 0.45);
  EXPECT_EQ(empty.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_TRUE(empty.clearance_certified);
  EXPECT_DOUBLE_EQ(
      empty.nearest_inflated_occupied_voxel_center_distance, 0.45);
}

TEST(CloudOccupancySnapshotTest,
     PlannerEsdfBaseClearanceMatchesSdfMapPositiveEdtAtGridCenters) {
  SDFMap map;
  map.mp_.resolution_ = 1.0;
  map.mp_.resolution_inv_ = 1.0;
  map.mp_.map_origin_ = Eigen::Vector3d(-5.0, -5.0, -5.0);
  map.mp_.map_size_ = Eigen::Vector3d(10.0, 10.0, 10.0);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
  map.mp_.map_voxel_num_ = Eigen::Vector3i(10, 10, 10);
  map.md_.occupancy_buffer_inflate_.assign(1000U, 0);
  map.md_.occupancy_buffer_neg.assign(1000U, 0);
  map.md_.distance_buffer_.assign(1000U, 0.0);
  map.md_.distance_buffer_neg_.assign(1000U, 0.0);
  map.md_.distance_buffer_all_.assign(1000U, 0.0);
  map.md_.tmp_buffer1_.assign(1000U, 0.0);
  map.md_.tmp_buffer2_.assign(1000U, 0.0);
  map.md_.local_bound_min_ = Eigen::Vector3i::Zero();
  map.md_.local_bound_max_ = Eigen::Vector3i::Constant(9);
  map.md_.occupancy_buffer_inflate_[
      (5U * 10U + 5U) * 10U + 5U] = 1;
  map.updateESDF3d();

  CloudOccupancySnapshot snapshot;
  snapshot.valid = true;
  snapshot.observation_sequence = 905U;
  snapshot.map_min = map.mp_.map_min_boundary_;
  snapshot.map_max = map.mp_.map_max_boundary_;
  snapshot.observed_min = snapshot.map_min;
  snapshot.observed_max = snapshot.map_max;
  snapshot.grid_origin = map.mp_.map_origin_;
  snapshot.voxel_count = map.mp_.map_voxel_num_;
  snapshot.resolution = map.mp_.resolution_;
  snapshot.occupied.assign(1000U, 0U);
  snapshot.occupied[(5U * 10U + 5U) * 10U + 5U] = 1U;

  const Eigen::Vector3i free_index(3, 5, 5);
  const Eigen::Vector3d center = snapshot.grid_origin +
      snapshot.resolution *
          (free_index.cast<double>() + Eigen::Vector3d::Constant(0.5));
  const CloudOccupancySnapshotPlannerEsdfBaseClearanceResult result =
      queryCloudOccupancySnapshotPlannerEsdfBaseClearance(snapshot, center,
                                                           2.20);
  ASSERT_EQ(result.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_TRUE(result.clearance_certified);
  EXPECT_NEAR(result.nearest_inflated_occupied_voxel_center_distance,
              map.getDistance(free_index), 1e-12);
  EXPECT_NEAR(result.nearest_inflated_occupied_voxel_center_distance, 2.0,
              1e-12);
}

TEST(CloudOccupancySnapshotTest,
     PlannerEsdfBaseClearanceFrozenMismatchPointsExceedSafeDistance) {
  const CloudOccupancySnapshot snapshot = plannerFixtureSnapshot();
  const Eigen::Vector3d point_one(2.6643275039478231,
                                  0.27533414135136092,
                                  0.99994409891574243);
  const Eigen::Vector3d point_two(2.7627822162288762,
                                  0.28778254721111168,
                                  0.99995064617094009);

  const CloudOccupancySnapshotClearanceResult old_one =
      queryCloudOccupancySnapshotClearance(snapshot, point_one, 0.50);
  const CloudOccupancySnapshotPlannerEsdfBaseClearanceResult new_one =
      queryCloudOccupancySnapshotPlannerEsdfBaseClearance(snapshot, point_one,
                                                           0.50);
  EXPECT_EQ(old_one.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_NEAR(old_one.nearest_occupied_voxel_volume_distance,
              0.39910242275510055, 1e-12);
  EXPECT_EQ(new_one.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_NEAR(new_one.nearest_inflated_occupied_voxel_center_distance,
              0.46581958181362049, 1e-12);
  EXPECT_GT(new_one.nearest_inflated_occupied_voxel_center_distance, 0.4);

  const CloudOccupancySnapshotClearanceResult old_two =
      queryCloudOccupancySnapshotClearance(snapshot, point_two, 0.50);
  const CloudOccupancySnapshotPlannerEsdfBaseClearanceResult new_two =
      queryCloudOccupancySnapshotPlannerEsdfBaseClearance(snapshot, point_two,
                                                           0.50);
  EXPECT_EQ(old_two.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_NEAR(old_two.nearest_occupied_voxel_volume_distance,
              0.38956445853076954, 1e-12);
  EXPECT_EQ(new_two.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_NEAR(new_two.nearest_inflated_occupied_voxel_center_distance,
              0.44817903921392843, 1e-12);
  EXPECT_GT(new_two.nearest_inflated_occupied_voxel_center_distance, 0.4);
}

TEST(CloudOccupancySnapshotTest, ClearanceDoesNotApplyIncludedInflationTwice) {
  CloudOccupancySnapshot snapshot = clearanceSnapshot();
  snapshot.occupied[clearanceAddress(6, 5, 5)] = 1U;
  const CloudOccupancySnapshotClearanceResult included =
      queryCloudOccupancySnapshotClearance(
          snapshot, Eigen::Vector3d(0.0, 0.5, 0.5), 1.20);
  snapshot.included_map_inflation = 0.0;
  const CloudOccupancySnapshotClearanceResult unincluded =
      queryCloudOccupancySnapshotClearance(
          snapshot, Eigen::Vector3d(0.0, 0.5, 0.5), 1.20);
  EXPECT_EQ(included.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_EQ(unincluded.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_DOUBLE_EQ(included.nearest_occupied_voxel_volume_distance,
                   unincluded.nearest_occupied_voxel_volume_distance);
}

TEST(CloudOccupancySnapshotTest, ClearanceMetadataFailuresAreUnavailable) {
  CloudOccupancySnapshot snapshot = clearanceSnapshot();
  snapshot.occupied.pop_back();
  EXPECT_EQ(queryCloudOccupancySnapshotClearance(
                snapshot, Eigen::Vector3d::Zero(), 0.10).status,
            CloudOccupancyStatus::UNAVAILABLE);
  snapshot = clearanceSnapshot();
  snapshot.voxel_count.x() = std::numeric_limits<int>::max();
  EXPECT_EQ(queryCloudOccupancySnapshotClearance(
                snapshot, Eigen::Vector3d::Zero(), 0.10).status,
            CloudOccupancyStatus::UNAVAILABLE);
}

TEST(CloudOccupancySnapshotTest,
     BuilderColumnIndexMatchesDenseReferenceAndSnapshotCopy) {
  CloudOccupancySnapshotBuildInput input = makeInput(61U);
  input.cloud_points.emplace_back(1.1, 0.1, 0.1);
  input.cloud_points.emplace_back(-1.1, 0.1, 0.1);
  input.cloud_points.emplace_back(0.1, -1.1, 0.1);
  const CloudOccupancySnapshot indexed = buildCloudOccupancySnapshot(input);
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(indexed));

  const CloudOccupancySnapshot dense = denseOnlySnapshot(indexed);
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(dense));

  // SDFMap copies the builder result into a const shared snapshot.  A normal
  // snapshot copy must retain a usable opaque index rather than falling back
  // solely because std::vector storage is copied.
  const CloudOccupancySnapshot copied = indexed;
  EXPECT_TRUE(cloudOccupancySnapshotConsistent(copied));

  const Eigen::Vector3d queries[] = {
      Eigen::Vector3d(0.0, 0.5, 0.5),
      Eigen::Vector3d(1.0, 0.5, 0.5),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.8, 0.0, 0.0),
      Eigen::Vector3d(2.1, 0.0, 0.0),
      Eigen::Vector3d(5.1, 0.0, 0.0),
  };
  const double radii[] = {1.20, 0.10, 0.45, 0.30, 0.30, 0.10};
  for (std::size_t index = 0U; index < sizeof(radii) / sizeof(radii[0]);
       ++index) {
    expectDenseAndIndexedClearanceEqual(dense, indexed, queries[index],
                                        radii[index]);
    expectDenseAndIndexedClearanceEqual(dense, copied, queries[index],
                                        radii[index]);
  }

  CloudOccupancySnapshot binding_mismatch = indexed;
  ++binding_mismatch.observation_sequence;
  const CloudOccupancySnapshot binding_mismatch_dense =
      denseOnlySnapshot(binding_mismatch);
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(binding_mismatch));
  expectDenseAndIndexedClearanceEqual(
      binding_mismatch_dense, binding_mismatch,
      Eigen::Vector3d(0.0, 0.5, 0.5), 1.20);
}

TEST(CloudOccupancySnapshotTest,
     EmptyBuilderColumnIndexAndManualNonbinaryDenseFallbackAreEquivalent) {
  const CloudOccupancySnapshot empty_indexed =
      buildCloudOccupancySnapshot(makeInput(62U));
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(empty_indexed));
  const CloudOccupancySnapshot empty_dense = denseOnlySnapshot(empty_indexed);
  expectDenseAndIndexedClearanceEqual(
      empty_dense, empty_indexed, Eigen::Vector3d::Zero(), 0.45);

  CloudOccupancySnapshot manual = clearanceSnapshot();
  manual.occupied[clearanceAddress(6, 5, 5)] = 2U;
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(manual));
  const CloudOccupancySnapshotClearanceResult face =
      queryCloudOccupancySnapshotClearance(
          manual, Eigen::Vector3d(0.0, 0.5, 0.5), 1.20);
  EXPECT_EQ(face.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_TRUE(face.clearance_certified);
  EXPECT_DOUBLE_EQ(face.nearest_occupied_voxel_volume_distance, 1.0);
  CloudOccupancySnapshot manual_diagonal = clearanceSnapshot();
  manual_diagonal.occupied[clearanceAddress(6, 6, 5)] = 255U;
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(manual_diagonal));
  const CloudOccupancySnapshotClearanceResult diagonal =
      queryCloudOccupancySnapshotClearance(
          manual_diagonal, Eigen::Vector3d(0.0, 0.0, 0.5), 1.50);
  EXPECT_EQ(diagonal.status, CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_NEAR(diagonal.nearest_occupied_voxel_volume_distance, std::sqrt(2.0),
              1e-12);

  CloudOccupancySnapshotBuildInput nonbinary_input = makeInput(63U);
  nonbinary_input.cloud_points.emplace_back(1.1, 0.1, 0.1);
  CloudOccupancySnapshot nonbinary_indexed =
      buildCloudOccupancySnapshot(nonbinary_input);
  for (std::size_t index = 0U; index < nonbinary_indexed.occupied.size();
       ++index) {
    if (nonbinary_indexed.occupied[index] != 0U) {
      nonbinary_indexed.occupied[index] = index % 2U == 0U ? 2U : 255U;
    }
  }
  const CloudOccupancySnapshot nonbinary_dense =
      denseOnlySnapshot(nonbinary_indexed);
  expectDenseAndIndexedClearanceEqual(
      nonbinary_dense, nonbinary_indexed, Eigen::Vector3d(0.0, 0.5, 0.5),
      1.20);
}

TEST(CloudOccupancySnapshotTest,
     RandomizedBuilderIndexedVsFreshDenseDifferentialIsBitwiseExact) {
  std::mt19937 random(20260812U);
  std::uniform_real_distribution<double> fraction(0.02, 0.98);
  const double radii[] = {0.0, 0.10, 0.25, 0.45};
  for (int snapshot_index = 0; snapshot_index < 24; ++snapshot_index) {
    const Eigen::Vector3i dimensions(7 + snapshot_index % 3,
                                     8 + snapshot_index % 4,
                                     6 + snapshot_index % 3);
    CloudOccupancySnapshot indexed = buildCloudOccupancySnapshot(
        randomizedBuildInput(dimensions,
                             static_cast<std::uint64_t>(100U + snapshot_index),
                             random));
    ASSERT_TRUE(cloudOccupancySnapshotConsistent(indexed));
    for (std::size_t occupied_index = 0U;
         occupied_index < indexed.occupied.size(); ++occupied_index) {
      if (indexed.occupied[occupied_index] != 0U) {
        indexed.occupied[occupied_index] = occupied_index % 2U == 0U ? 2U :
            255U;
      }
    }
    // denseOnlySnapshot explicitly copies only public metadata and dense
    // occupancy into a new value, so it cannot retain the private CSR.
    const CloudOccupancySnapshot dense = denseOnlySnapshot(indexed);
    for (int query_index = 0; query_index < 128; ++query_index) {
      const Eigen::Vector3d extent = indexed.map_max - indexed.map_min;
      const Eigen::Vector3d point = indexed.map_min + Eigen::Vector3d(
          fraction(random) * extent.x(), fraction(random) * extent.y(),
          fraction(random) * extent.z());
      expectDenseAndIndexedClearanceEqual(
          dense, indexed, point,
          radii[(snapshot_index + query_index) %
                 (sizeof(radii) / sizeof(radii[0]))]);
      expectDenseAndIndexedPlannerClearanceEqual(
          dense, indexed, point,
          radii[(snapshot_index + query_index) %
                 (sizeof(radii) / sizeof(radii[0]))]);
    }
  }
}

TEST(CloudOccupancySnapshotTest,
     SdfMapCloudCallbackPublishesImmutableSequenceIncludingEmptyCloud) {
  SDFMap map;
  EXPECT_FALSE(map.cloudOccupancySnapshot());
  initializeCloudCallbackMap(map);

  const sensor_msgs::PointCloud2 empty = cloudMessage({}, ros::Time(31.0));
  map.cloudCallback(boost::make_shared<sensor_msgs::PointCloud2>(empty));
  const std::shared_ptr<const CloudOccupancySnapshot> first =
      map.cloudOccupancySnapshot();
  ASSERT_TRUE(first);
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(*first));
  EXPECT_EQ(first->observation_sequence, 1U);
  EXPECT_EQ(queryCloudOccupancySnapshot(*first, Eigen::Vector3d::Zero()).status,
            CloudOccupancyStatus::KNOWN_FREE);

  const sensor_msgs::PointCloud2 obstacle = cloudMessage(
      {Eigen::Vector3d(1.1, 0.1, 0.1)}, ros::Time(32.0));
  map.cloudCallback(boost::make_shared<sensor_msgs::PointCloud2>(obstacle));
  const std::shared_ptr<const CloudOccupancySnapshot> second =
      map.cloudOccupancySnapshot();
  ASSERT_TRUE(second);
  ASSERT_TRUE(cloudOccupancySnapshotConsistent(*second));
  EXPECT_EQ(second->observation_sequence, 2U);
  EXPECT_EQ(queryCloudOccupancySnapshot(*first, Eigen::Vector3d(1.1, 0.1, 0.1)).status,
            CloudOccupancyStatus::KNOWN_FREE);
  EXPECT_EQ(queryCloudOccupancySnapshot(*second, Eigen::Vector3d(1.1, 0.1, 0.1)).status,
            CloudOccupancyStatus::OCCUPIED);
}

TEST(CloudOccupancySnapshotTest,
     AuthoritativeV2PredicateUsesClosedBoundaryAndSupport) {
  SDFMapCaptureV2 capture = makeAuthoritativeCaptureFixture();
  ASSERT_TRUE(sdfMapCaptureV2Consistent(capture));

  const SDFMapCaptureFreeBallResultV2 free =
      certifySDFMapCaptureFreeBallV2(capture, Eigen::Vector3d::Zero(), 0.0);
  EXPECT_EQ(free.status, SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE);
  EXPECT_TRUE(free.clearance_certified);

  // A dyadic native grid admits a mathematically exact closed-ball tangency
  // at its true upper boundary; the exact containment path must preserve it.
  const SDFMapCaptureV2 grid_edge_capture = makeAuthoritativeCaptureFixture();
  const SDFMapCaptureFreeBallResultV2 grid_edge_tangent =
      certifySDFMapCaptureFreeBallV2(
          grid_edge_capture, Eigen::Vector3d(3.75, 2.0, 2.0), 0.25);
  EXPECT_EQ(grid_edge_tangent.status,
            SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE);
  EXPECT_TRUE(grid_edge_tangent.clearance_certified);

  capture.occupied[0U] = 1U;
  const SDFMapCaptureFreeBallResultV2 occupied =
      certifySDFMapCaptureFreeBallV2(capture, Eigen::Vector3d::Zero(), 0.0);
  EXPECT_EQ(occupied.status, SDFMapCaptureFreeBallStatusV2::OCCUPIED);
  EXPECT_FALSE(occupied.clearance_certified);

  capture.occupied[0U] = 0U;
  capture.occupied[16U] = 1U;  // closed x-face at x=1
  const SDFMapCaptureFreeBallResultV2 contact =
      certifySDFMapCaptureFreeBallV2(
          capture, Eigen::Vector3d(1.0, 0.5, 0.5), 0.0);
  EXPECT_EQ(contact.status, SDFMapCaptureFreeBallStatusV2::OCCUPIED);

  // A positive-radius free centre tangent to the closed x-face is certifiable
  // when exact arithmetic proves d == R.  This is distinct from the point
  // occupancy case above and preserves the frozen d >= R contract.
  const SDFMapCaptureFreeBallResultV2 tangent =
      certifySDFMapCaptureFreeBallV2(
          capture, Eigen::Vector3d(0.75, 0.5, 0.5), 0.25);
  EXPECT_EQ(tangent.status, SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE);
  EXPECT_TRUE(tangent.clearance_certified);

  capture.support.mask[0U] = 0U;
  const SDFMapCaptureFreeBallResultV2 unsupported =
      certifySDFMapCaptureFreeBallV2(capture, Eigen::Vector3d::Zero(), 0.0);
  EXPECT_EQ(unsupported.status, SDFMapCaptureFreeBallStatusV2::UNKNOWN);

  capture.support.mask[0U] = 1U;
  capture.occupied[16U] = 0U;
  capture.support.evidence_basis = kSDFMapCaptureSupportEvidenceDepthRaycast;
  const SDFMapCaptureFreeBallResultV2 depth_only =
      certifySDFMapCaptureFreeBallV2(capture, Eigen::Vector3d::Zero(), 0.0);
  EXPECT_EQ(depth_only.status, SDFMapCaptureFreeBallStatusV2::UNAVAILABLE);

  capture.support.evidence_basis =
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  capture.accepted_state_stamp = ros::Time(6.0);
  capture.support.evidence_stamp = ros::Time(4.0);
  capture.support.valid_until = ros::Time(5.0);
  capture.support.valid_until_accepted_ticks = 5U;
  const SDFMapCaptureFreeBallResultV2 expired =
      certifySDFMapCaptureFreeBallV2(capture, Eigen::Vector3d::Zero(), 0.0);
  EXPECT_EQ(expired.status, SDFMapCaptureFreeBallStatusV2::UNAVAILABLE);

  capture.support.valid_until = ros::Time(7.0);
  capture.support.valid_until_accepted_ticks = 200U;
  const SDFMapCaptureFreeBallResultV2 fresh =
      certifySDFMapCaptureFreeBallV2(capture, Eigen::Vector3d::Zero(), 0.0);
  EXPECT_EQ(fresh.status, SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE);

  capture.support.valid_until = ros::Time();
  capture.support.valid_until_accepted_ticks = 0U;
  capture.occupied[16U] = 1U;
  const SDFMapCaptureFreeBallResultV2 tangent_below =
      certifySDFMapCaptureFreeBallV2(
          capture,
          Eigen::Vector3d(std::nextafter(0.75, 0.0), 0.5, 0.5), 0.25);
  EXPECT_EQ(tangent_below.status,
            SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE);
  const SDFMapCaptureFreeBallResultV2 tangent_above =
      certifySDFMapCaptureFreeBallV2(
          capture,
          Eigen::Vector3d(std::nextafter(0.75, 1.0), 0.5, 0.5), 0.25);
  EXPECT_EQ(tangent_above.status,
            SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE);
}

TEST(CloudOccupancySnapshotTest,
     AuthoritativeV2CaptureCopiesEffectiveBackingAndUsesFreshIdentity) {
  SDFMap map;
  // This test intentionally exercises the legacy SDFMap copy constructor.
  // Value-initialize both aggregate payloads first so sanitizer runs do not
  // copy indeterminate scalar/bool fields that this focused fixture never
  // uses; all fields relevant to the capture are assigned below.
  map.mp_ = MappingParameters();
  map.md_ = MappingData();
  map.mp_.map_origin_ = Eigen::Vector3d::Zero();
  map.mp_.map_size_ = Eigen::Vector3d::Constant(8.0);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_size_;
  map.mp_.map_voxel_num_ = Eigen::Vector3i::Constant(4);
  map.mp_.resolution_ = 2.0;
  map.mp_.resolution_inv_ = 0.5;
  map.mp_.local_update_range_ = Eigen::Vector3d::Constant(1.0);
  map.mp_.fx_ = 1.0;
  map.mp_.fy_ = 1.0;
  map.mp_.cx_ = 0.0;
  map.mp_.cy_ = 0.0;
  map.mp_.frame_id_ = "world";
  map.mp_.obstacles_inflation_ = 0.1;
  map.mp_.enable_manual_map_ = true;
  map.mp_.static_preinflated_map_enable_ = true;
  map.mp_.virtual_ceil_height_ = 6.0;
  map.mp_.min_occupancy_log_ = 0.0;
  map.mp_.local_map_margin_ = 0;
  map.mp_.ground_height_ = 0.0;
  map.md_.occupancy_buffer_inflate_.assign(64U, 0);
  map.md_.occupancy_buffer_.assign(64U, -1.0);
  map.md_.occupancy_buffer_[42U] = 1.0;
  map.md_.distance_buffer_all_.assign(64U, 10000.0);
  map.md_.distance_buffer_.assign(64U, 10000.0);
  map.md_.manual_occupancy_buffer_.assign(64U, 0);
  map.md_.manual_occupancy_buffer_[1U] = 1;
  map.md_.static_preinflated_buffer_.assign(64U, 0);
  map.md_.static_preinflated_buffer_[2U] = 1;
  map.md_.static_preinflated_map_ready_ = true;
  map.md_.static_preinflated_map_loaded_ = true;
  map.md_.local_bound_min_ = Eigen::Vector3i::Zero();
  map.md_.local_bound_max_ = Eigen::Vector3i::Constant(3);
  // Seed the ceiling through the native clear/inflate path rather than
  // synthesizing it in captureAuthoritativeSDFMapV2().
  map.clearAndInflateLocalMap();
  map.applyManualLayer();
  map.applyStaticPreinflatedLayer();
  map.md_.has_cloud_ = false;
  map.md_.has_first_depth_ = false;
  map.md_.authoritative_support_buffer_v2_.assign(64U, 1);
  map.md_.authoritative_support_valid_v2_ = true;
  map.md_.authoritative_support_complete_v2_ = true;
  map.md_.authoritative_support_evidence_basis_v2_ =
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  map.md_.authoritative_support_sequence_v2_ = 1U;
  map.md_.authoritative_support_min_v2_ = map.mp_.map_min_boundary_;
  map.md_.authoritative_support_max_v2_ = map.mp_.map_max_boundary_;
  map.md_.authoritative_support_halo_reconciled_v2_ = true;
  map.md_.authoritative_state_sequence_v2_ = 1U;
  map.md_.authoritative_state_notification_sequence_v2_ = 1U;
  map.md_.authoritative_state_time_ticks_v2_ = 99U;
  map.md_.authoritative_configuration_generation_v2_ = 1U;
  bindMapSupportToCurrentConfiguration(map);

  const std::shared_ptr<const SDFMapCaptureV2> first =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(first);
  ASSERT_TRUE(sdfMapCaptureV2Consistent(*first));
  const SDFMapAcceptedStateVisibilityV2 accepted_before =
      map.acceptedStateVisibilityV2();
  ASSERT_TRUE(accepted_before.valid);
  map.md_.occupancy_buffer_inflate_[10U] =
      map.md_.occupancy_buffer_inflate_[10U] == 0 ? 1 : 0;
  const SDFMapAcceptedStateVisibilityV2 unaccepted_mutation =
      map.acceptedStateVisibilityV2();
  EXPECT_EQ(unaccepted_mutation.accepted_state_sequence,
            accepted_before.accepted_state_sequence);
  EXPECT_EQ(unaccepted_mutation.accepted_state_notification_sequence,
            accepted_before.accepted_state_notification_sequence);
  EXPECT_TRUE(first->grid_origin.isApprox(map.mp_.map_origin_));
  EXPECT_TRUE(first->source_min_index.isZero());
  EXPECT_NE(first->effective_layer_mask & kSDFMapCaptureLayerManual, 0U);
  EXPECT_NE(first->effective_layer_mask & kSDFMapCaptureLayerStatic, 0U);
  EXPECT_NE(first->effective_layer_mask & kSDFMapCaptureLayerCeiling, 0U);
  EXPECT_EQ(first->occupied[1U], 1U);
  EXPECT_EQ(first->occupied[2U], 1U);
  EXPECT_EQ(first->occupied[3U], 1U);
  EXPECT_EQ(first->occupied[42U], 1U);

  const std::string original_frame = map.mp_.frame_id_;
  map.mp_.frame_id_ = "calibrated_frame_changed";
  const std::shared_ptr<const SDFMapCaptureV2> frame_changed =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(frame_changed);
  EXPECT_FALSE(frame_changed->support.valid);
  map.mp_.frame_id_ = original_frame;
  map.mp_.fx_ += 1.0;
  const std::shared_ptr<const SDFMapCaptureV2> calibration_changed =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(calibration_changed);
  EXPECT_FALSE(calibration_changed->support.valid);
  map.mp_.fx_ -= 1.0;
  const std::shared_ptr<const SDFMapCaptureV2> restored_configuration =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(restored_configuration);
  EXPECT_FALSE(restored_configuration->support.valid);

  // Merely configuring a virtual ceiling on a cloud-only map must not create
  // a phantom plane when the native inflate backing has no such voxels.
  for (int x = 0; x < 4; ++x)
    for (int y = 0; y < 4; ++y)
      map.md_.occupancy_buffer_inflate_[
          (static_cast<std::size_t>(x) * 4U +
           static_cast<std::size_t>(y)) * 4U + 3U] = 0;
  const std::shared_ptr<const SDFMapCaptureV2> no_ceiling =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(no_ceiling);
  EXPECT_EQ(no_ceiling->occupied[3U], 0U);
  EXPECT_EQ(no_ceiling->effective_layer_mask &
            kSDFMapCaptureLayerCeiling,
            0U);
  for (int x = 0; x < 4; ++x)
    for (int y = 0; y < 4; ++y)
      map.md_.occupancy_buffer_inflate_[
          (static_cast<std::size_t>(x) * 4U +
           static_cast<std::size_t>(y)) * 4U + 3U] = 1;

  map.mp_.obstacles_inflation_ = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(map.captureAuthoritativeSDFMapV2());
  map.mp_.obstacles_inflation_ = -0.1;
  EXPECT_FALSE(map.captureAuthoritativeSDFMapV2());
  map.mp_.obstacles_inflation_ = 0.1;
  map.md_.manual_occupancy_buffer_.assign(3U, 1);
  EXPECT_FALSE(map.captureAuthoritativeSDFMapV2());
  map.md_.manual_occupancy_buffer_.assign(64U, 0);
  map.md_.manual_occupancy_buffer_[1U] = 1;
  map.md_.static_preinflated_buffer_.assign(3U, 1);
  EXPECT_FALSE(map.captureAuthoritativeSDFMapV2());
  map.md_.static_preinflated_buffer_.assign(64U, 0);
  map.md_.static_preinflated_buffer_[2U] = 1;

  // The frame/calibration mutations above deliberately latch the prior
  // support binding invalid.  Rebind the restored configuration with a fresh
  // complete-domain evidence event before comparing the cropped capture with
  // the immutable full capture; restoring the old parameter values alone must
  // not resurrect stale support.
  map.noteAuthoritativeMapMutationV2Locked(ros::Time(8.0));
  map.markAuthoritativeSupportV2Locked(
      Eigen::Vector3i::Zero(),
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain,
      ros::Time(8.0));
  // Reconcile the synthetic complete-domain halo separately from the fresh
  // voxel declaration; invalidateAuthoritativeSupportV2Locked deliberately
  // clears this proof bit and markAuthoritativeSupportV2Locked does not invent
  // it for a single supplied cell.
  map.md_.authoritative_support_halo_reconciled_v2_ = true;

  // The fresh declaration above names only voxel A.  It must not resurrect
  // the prior complete mask's voxel B until the producer explicitly supplies
  // that broader domain again.
  const std::shared_ptr<const SDFMapCaptureV2> rebound_only =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(rebound_only);
  ASSERT_TRUE(sdfMapCaptureV2Consistent(*rebound_only));
  EXPECT_EQ(rebound_only->support.mask[0U], 1U);
  EXPECT_EQ(rebound_only->support.mask[16U], 0U);
  EXPECT_EQ(certifySDFMapCaptureFreeBallV2(
                *rebound_only, Eigen::Vector3d(3.0, 1.0, 1.0), 0.0)
                .status,
            SDFMapCaptureFreeBallStatusV2::UNKNOWN);

  // This fixture intentionally models a newly supplied complete domain for
  // the crop comparison; establish every support cell explicitly rather than
  // relying on the single-cell declaration used by the regression above.
  std::fill(map.md_.authoritative_support_buffer_v2_.begin(),
            map.md_.authoritative_support_buffer_v2_.end(), 1);
  map.rebuildAuthoritativeSupportBoundsV2Locked();

  SDFMapCaptureRegionV2 crop_region;
  crop_region.valid = true;
  crop_region.min = Eigen::Vector3d::Constant(2.2);
  crop_region.max = Eigen::Vector3d::Constant(5.8);
  crop_region.halo = 0.0;
  const std::shared_ptr<const SDFMapCaptureV2> cropped =
      map.captureAuthoritativeSDFMapV2(crop_region);
  ASSERT_TRUE(cropped);
  ASSERT_TRUE(sdfMapCaptureV2Consistent(*cropped));
  EXPECT_TRUE(cropped->grid_origin.isApprox(first->grid_origin));
  EXPECT_TRUE(cropped->source_min_index.isApprox(Eigen::Vector3i::Constant(1)));
  EXPECT_TRUE(cropped->source_max_index.isApprox(Eigen::Vector3i::Constant(2)));
  EXPECT_TRUE(cropped->voxel_count.isApprox(Eigen::Vector3i::Constant(2)));
  crop_region.halo = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(map.captureAuthoritativeSDFMapV2(crop_region));
  crop_region.halo = 0.0;
  for (int x = 0; x < cropped->voxel_count.x(); ++x) {
    for (int y = 0; y < cropped->voxel_count.y(); ++y) {
      for (int z = 0; z < cropped->voxel_count.z(); ++z) {
        const std::size_t cropped_address =
            (static_cast<std::size_t>(x) * 2U +
             static_cast<std::size_t>(y)) * 2U +
            static_cast<std::size_t>(z);
        const Eigen::Vector3i source =
            cropped->source_min_index + Eigen::Vector3i(x, y, z);
        const std::size_t full_address =
            (static_cast<std::size_t>(source.x()) * 4U +
             static_cast<std::size_t>(source.y())) * 4U +
            static_cast<std::size_t>(source.z());
        EXPECT_EQ(cropped->occupied[cropped_address],
                  first->occupied[full_address]);
        EXPECT_EQ(cropped->support.mask[cropped_address],
                  first->support.mask[full_address]);
      }
    }
  }
  const SDFMapCaptureFreeBallResultV2 full_free =
      certifySDFMapCaptureFreeBallV2(
          *first, Eigen::Vector3d(3.0, 3.0, 3.0), 0.25);
  const SDFMapCaptureFreeBallResultV2 crop_free =
      certifySDFMapCaptureFreeBallV2(
          *cropped, Eigen::Vector3d(3.0, 3.0, 3.0), 0.25);
  EXPECT_EQ(crop_free.status, full_free.status);
  EXPECT_EQ(crop_free.clearance_certified, full_free.clearance_certified);
  const SDFMapCaptureFreeBallResultV2 full_occupied =
      certifySDFMapCaptureFreeBallV2(
          *first, Eigen::Vector3d(4.5, 4.5, 4.5), 0.0);
  const SDFMapCaptureFreeBallResultV2 crop_occupied =
      certifySDFMapCaptureFreeBallV2(
          *cropped, Eigen::Vector3d(4.5, 4.5, 4.5), 0.0);
  EXPECT_EQ(crop_occupied.status, full_occupied.status);
  EXPECT_EQ(crop_occupied.clearance_certified,
            full_occupied.clearance_certified);

  // Reset/reinflation changes the current backing and invalidates support,
  // while the prior immutable capture remains unchanged and certifiable.
  map.resetBuffer();
  const std::shared_ptr<const SDFMapCaptureV2> reset_capture =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(reset_capture);
  ASSERT_TRUE(sdfMapCaptureV2Consistent(*reset_capture));
  EXPECT_FALSE(reset_capture->support.valid);
  EXPECT_EQ(reset_capture->occupied[1U], 1U);  // manual overlay
  EXPECT_EQ(reset_capture->occupied[2U], 1U);  // static reinflation
  EXPECT_EQ(first->occupied[42U], 1U);
  EXPECT_TRUE(first->support.valid);

  const std::uint64_t prior_sequence = first->accepted_state_sequence;
  map.setOccupied(Eigen::Vector3d(0.5, 0.5, 0.5));
  const std::shared_ptr<const SDFMapCaptureV2> mutated =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(mutated);
  EXPECT_GT(mutated->accepted_state_sequence, prior_sequence);
  EXPECT_EQ(mutated->occupied[0U], 1U);

  SDFMap copied = map;
  const std::shared_ptr<const SDFMapCaptureV2> second =
      copied.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(second);
  EXPECT_NE(first->map_instance_id, second->map_instance_id);

  map.md_.authoritative_state_sequence_v2_ =
      std::numeric_limits<std::uint64_t>::max();
  map.md_.authoritative_state_identity_exhausted_v2_ = false;
  map.noteAuthoritativeMapMutationV2Locked();
  EXPECT_TRUE(map.md_.authoritative_state_identity_exhausted_v2_);
  EXPECT_FALSE(map.captureAuthoritativeSDFMapV2());
}

TEST(CloudOccupancySnapshotTest,
     AuthoritativeV2ExactArithmeticCoversSubnormalLargeAndOverflow) {
  const double denorm = std::numeric_limits<double>::denorm_min();
  const Eigen::Vector3d tiny_min = Eigen::Vector3d::Constant(-4.0 * denorm);
  const Eigen::Vector3d tiny_max = Eigen::Vector3d::Constant(4.0 * denorm);
  SDFMapCaptureV2 tiny;
  tiny.valid = true;
  tiny.map_instance_id = 301U;
  tiny.configuration_generation = 1U;
  tiny.configuration_key = 1U;
  tiny.frame_id = "world";
  tiny.accepted_state_sequence = 1U;
  tiny.accepted_time_ticks = 1U;
  tiny.map_min = tiny_min;
  tiny.map_max = tiny_max;
  tiny.grid_origin = tiny_min;
  tiny.capture_min = tiny_min;
  tiny.capture_max = tiny_max;
  tiny.voxel_count = Eigen::Vector3i::Constant(8);
  tiny.resolution = denorm;
  tiny.included_map_inflation = 0.0;
  tiny.occupied.assign(512U, 0U);
  tiny.occupied[(4U * 8U + 4U) * 8U + 4U] = 1U;
  tiny.support.valid = true;
  tiny.support.complete = true;
  tiny.support.evidence_basis =
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  tiny.support.evidence_sequence = 1U;
  tiny.support.evidence_accepted_ticks = 1U;
  tiny.support.map_instance_id = tiny.map_instance_id;
  tiny.support.configuration_generation = tiny.configuration_generation;
  tiny.support.configuration_key = tiny.configuration_key;
  tiny.support.frame_id = tiny.frame_id;
  tiny.support.support_min = tiny_min;
  tiny.support.support_max = tiny_max;
  tiny.support.halo_reconciled = true;
  tiny.support.mask.assign(512U, 1U);
  ASSERT_TRUE(sdfMapCaptureV2Consistent(tiny));
  const SDFMapCaptureFreeBallResultV2 subnormal_gap =
      certifySDFMapCaptureFreeBallV2(
          tiny, Eigen::Vector3d(-denorm, 0.0, 0.0), 2.0 * denorm);
  // The true gap is denorm_min while the requested radius is 2*denorm_min;
  // squaring both in binary64 underflows, so the exact dyadic fallback must
  // refuse certification rather than silently treating the ball as free.
  EXPECT_EQ(subnormal_gap.status,
            SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE);
  EXPECT_FALSE(subnormal_gap.clearance_certified);

  SDFMapCaptureV2 large = makeAuthoritativeCaptureFixture();
  const double large_origin = std::ldexp(1.0, 40);
  large.map_min = Eigen::Vector3d::Constant(large_origin);
  // Keep the planner/map domain a little larger than the native four-voxel
  // capture so the ULP-above query is rejected by exact grid containment,
  // rather than being classified as OUT_OF_MAP first.
  large.map_max = Eigen::Vector3d::Constant(large_origin + 4.5);
  large.grid_origin = large.map_min;
  large.capture_min = large.map_min;
  large.capture_max = Eigen::Vector3d::Constant(large_origin + 4.0);
  large.occupied.assign(64U, 0U);
  large.support.support_min = large.map_min;
  large.support.support_max = large.map_max;
  ASSERT_TRUE(sdfMapCaptureV2Consistent(large));
  const SDFMapCaptureFreeBallResultV2 large_tangent =
      certifySDFMapCaptureFreeBallV2(
          large, Eigen::Vector3d::Constant(large_origin + 3.75), 0.25);
  EXPECT_EQ(large_tangent.status,
            SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE);
  const Eigen::Vector3d one_ulp_above(
      std::nextafter(large_origin + 3.75, std::numeric_limits<double>::infinity()),
      large_origin + 2.0, large_origin + 2.0);
  const SDFMapCaptureFreeBallResultV2 large_above =
      certifySDFMapCaptureFreeBallV2(large, one_ulp_above, 0.25);
  EXPECT_EQ(large_above.status, SDFMapCaptureFreeBallStatusV2::UNKNOWN);
  EXPECT_FALSE(large_above.clearance_certified);

  SDFMapCaptureV2 overflow = large;
  const double huge_origin = std::numeric_limits<double>::max() / 2.0;
  const double huge_resolution = std::numeric_limits<double>::max() / 2.0;
  overflow.map_min = Eigen::Vector3d::Constant(huge_origin);
  overflow.map_max = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::max());
  overflow.grid_origin = overflow.map_min;
  overflow.capture_min = overflow.map_min;
  overflow.capture_max = overflow.map_max;
  overflow.resolution = huge_resolution;
  overflow.support.support_min = overflow.map_min;
  overflow.support.support_max = overflow.map_max;
  EXPECT_FALSE(sdfMapCaptureV2Consistent(overflow));
  EXPECT_EQ(certifySDFMapCaptureFreeBallV2(
                overflow, overflow.map_min, 0.0)
                .status,
            SDFMapCaptureFreeBallStatusV2::UNAVAILABLE);
}

TEST(CloudOccupancySnapshotTest,
     AuthoritativeV2RejectsUnsupportedFloatingPointModes) {
  SDFMapCaptureV2 capture = makeAuthoritativeCaptureFixture();
  ASSERT_TRUE(sdfMapCaptureV2FloatingPointEnvironmentSupported());
  ASSERT_TRUE(sdfMapCaptureV2Consistent(capture));

  const int original_rounding = std::fegetround();
  ASSERT_EQ(original_rounding, FE_TONEAREST);
  ASSERT_EQ(std::fesetround(FE_DOWNWARD), 0);
  EXPECT_FALSE(sdfMapCaptureV2FloatingPointEnvironmentSupported());
  EXPECT_FALSE(sdfMapCaptureV2Consistent(capture));
  EXPECT_EQ(certifySDFMapCaptureFreeBallV2(
                capture, Eigen::Vector3d::Zero(), 0.0)
                .status,
            SDFMapCaptureFreeBallStatusV2::UNAVAILABLE);
  ASSERT_EQ(std::fesetround(original_rounding), 0);
  ASSERT_TRUE(sdfMapCaptureV2FloatingPointEnvironmentSupported());

#if defined(__i386__) || defined(__x86_64__)
  const unsigned int original_csr = _mm_getcsr();
  unsigned int unsupported_csr = original_csr;
#ifdef _MM_FLUSH_ZERO_ON
  unsupported_csr |= _MM_FLUSH_ZERO_ON;
#endif
#ifdef _MM_DENORMALS_ZERO_ON
  unsupported_csr |= _MM_DENORMALS_ZERO_ON;
#endif
  _mm_setcsr(unsupported_csr);
  EXPECT_FALSE(sdfMapCaptureV2FloatingPointEnvironmentSupported());
  EXPECT_FALSE(sdfMapCaptureV2Consistent(capture));
  _mm_setcsr(original_csr);
  ASSERT_TRUE(sdfMapCaptureV2FloatingPointEnvironmentSupported());
#endif
}

TEST(CloudOccupancySnapshotTest,
     AuthoritativeV2ManualBoundaryRestrictsDomainAndInvalidatesClear) {
  SDFMap map;
  map.mp_ = MappingParameters();
  map.md_ = MappingData();
  map.mp_.map_origin_ = Eigen::Vector3d::Zero();
  map.mp_.map_size_ = Eigen::Vector3d::Constant(4.0);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_size_;
  map.mp_.map_voxel_num_ = Eigen::Vector3i::Constant(4);
  map.mp_.resolution_ = 1.0;
  map.mp_.resolution_inv_ = 1.0;
  map.mp_.local_update_range_ = Eigen::Vector3d::Constant(1.0);
  map.mp_.fx_ = 1.0;
  map.mp_.fy_ = 1.0;
  map.mp_.cx_ = 0.0;
  map.mp_.cy_ = 0.0;
  map.mp_.frame_id_ = "world";
  map.mp_.obstacles_inflation_ = 0.0;
  map.mp_.enable_manual_map_ = true;
  map.mp_.manual_boundary_padding_ = 0.1;
  map.mp_.manual_boundary_z_min_ = 1.0;
  map.mp_.manual_boundary_z_max_ = 3.0;
  map.mp_.clamp_min_log_ = -1.0;
  map.mp_.clamp_max_log_ = 1.0;
  map.mp_.unknown_flag_ = 1.0;
  map.mp_.map_max_idx_ = Eigen::Vector3i::Constant(3);
  map.mp_.manual_obstacle_radius_ = 0.0;
  map.mp_.manual_obstacle_inflate_ = 0.0;
  map.mp_.manual_obstacle_height_ = 4.0;
  map.mp_.manual_map_auto_save_ = false;
  map.mp_.manual_map_auto_load_ = false;
  map.mp_.static_preinflated_map_enable_ = false;
  map.mp_.virtual_ceil_height_ = -1.0;
  map.md_.occupancy_buffer_.assign(64U, -1.0);
  map.md_.occupancy_buffer_inflate_.assign(64U, 0);
  map.md_.manual_occupancy_buffer_.assign(64U, 0);
  map.md_.static_preinflated_buffer_.assign(64U, 0);
  map.md_.local_bound_min_ = Eigen::Vector3i::Zero();
  map.md_.local_bound_max_ = Eigen::Vector3i::Constant(3);
  map.md_.manual_boundary_enabled_ = true;
  map.md_.manual_boundary_min_ = Eigen::Vector3d::Constant(1.0);
  map.md_.manual_boundary_max_ = Eigen::Vector3d::Constant(3.0);
  map.md_.authoritative_support_buffer_v2_.assign(64U, 1);
  map.md_.authoritative_support_valid_v2_ = true;
  map.md_.authoritative_support_complete_v2_ = true;
  map.md_.authoritative_support_evidence_basis_v2_ =
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  map.md_.authoritative_support_sequence_v2_ = 1U;
  map.md_.authoritative_support_min_v2_ = map.md_.manual_boundary_min_;
  map.md_.authoritative_support_max_v2_ = map.md_.manual_boundary_max_;
  map.md_.authoritative_support_halo_reconciled_v2_ = true;
  map.md_.authoritative_state_sequence_v2_ = 1U;
  map.md_.authoritative_state_time_ticks_v2_ = 1U;
  map.md_.authoritative_configuration_generation_v2_ = 1U;
  bindMapSupportToCurrentConfiguration(map);

  const std::shared_ptr<const SDFMapCaptureV2> bounded =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(bounded);
  ASSERT_TRUE(sdfMapCaptureV2Consistent(*bounded));
  EXPECT_TRUE(bounded->map_min.isApprox(Eigen::Vector3d::Constant(1.0)));
  EXPECT_TRUE(bounded->map_max.isApprox(Eigen::Vector3d::Constant(3.0)));
  EXPECT_EQ(certifySDFMapCaptureFreeBallV2(
                *bounded, Eigen::Vector3d::Constant(0.5), 0.0)
                .status,
            SDFMapCaptureFreeBallStatusV2::OUT_OF_MAP);
  EXPECT_EQ(certifySDFMapCaptureFreeBallV2(
                *bounded, Eigen::Vector3d::Constant(2.0), 0.0)
                .status,
            SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE);
  ASSERT_TRUE(map.md_.authoritative_support_valid_v2_);

  // Exercise the real callback clearing seam: an old manual voxel is removed
  // before the new boundary walls are built, and its complete support cannot
  // survive that destructive update.
  map.md_.manual_occupancy_buffer_[63U] = 1;
  map.md_.occupancy_buffer_inflate_[63U] = 1;
  geometry_msgs::PointStamped first_point;
  first_point.point.x = 1.0;
  first_point.point.y = 1.0;
  first_point.point.z = 1.0;
  map.manualBoundaryCallback(
      boost::make_shared<geometry_msgs::PointStamped>(first_point));
  EXPECT_FALSE(map.md_.authoritative_support_valid_v2_);
  EXPECT_EQ(map.md_.manual_occupancy_buffer_[63U], 0);
  EXPECT_EQ(map.md_.occupancy_buffer_inflate_[63U], 0);
  geometry_msgs::PointStamped second_point = first_point;
  second_point.point.x = 2.0;
  second_point.point.y = 2.0;
  map.manualBoundaryCallback(
      boost::make_shared<geometry_msgs::PointStamped>(second_point));
  EXPECT_FALSE(map.md_.authoritative_support_valid_v2_);
  ASSERT_TRUE(map.md_.manual_boundary_enabled_);
  EXPECT_TRUE(map.md_.manual_boundary_min_.allFinite());
  EXPECT_TRUE(map.md_.manual_boundary_max_.allFinite());

  // Re-establish a genuinely fresh complete declaration before checking the
  // separate configuration-generation/exhaustion gates; those checks must not
  // pass merely because the callback already invalidated support.
  std::fill(map.md_.authoritative_support_buffer_v2_.begin(),
            map.md_.authoritative_support_buffer_v2_.end(), 1);
  map.md_.authoritative_support_valid_v2_ = true;
  map.md_.authoritative_support_complete_v2_ = true;
  map.md_.authoritative_support_evidence_basis_v2_ =
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  map.md_.authoritative_support_sequence_v2_ = 1U;
  map.md_.authoritative_support_min_v2_ = map.md_.manual_boundary_min_;
  map.md_.authoritative_support_max_v2_ = map.md_.manual_boundary_max_;
  map.md_.authoritative_support_halo_reconciled_v2_ = true;
  map.md_.authoritative_configuration_identity_exhausted_v2_ = false;
  map.md_.authoritative_configuration_generation_v2_ = 1U;
  bindMapSupportToCurrentConfiguration(map);
  ASSERT_TRUE(map.md_.authoritative_support_valid_v2_);
  const std::uint64_t original_generation =
      map.md_.authoritative_configuration_generation_v2_;
  map.md_.authoritative_configuration_generation_v2_ =
      original_generation + 1U;
  const std::shared_ptr<const SDFMapCaptureV2> generation_changed =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(generation_changed);
  EXPECT_FALSE(generation_changed->support.valid);
  map.md_.authoritative_configuration_identity_exhausted_v2_ = true;
  EXPECT_FALSE(map.captureAuthoritativeSDFMapV2());
}

TEST(CloudOccupancySnapshotTest,
     AuthoritativeV2CaptureSerializesConcurrentMutationsCoherently) {
  SDFMap map;
  map.mp_ = MappingParameters();
  map.md_ = MappingData();
  map.mp_.map_origin_ = Eigen::Vector3d::Zero();
  map.mp_.map_size_ = Eigen::Vector3d::Constant(2.0);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_size_;
  map.mp_.map_voxel_num_ = Eigen::Vector3i::Constant(2);
  map.mp_.resolution_ = 1.0;
  map.mp_.resolution_inv_ = 1.0;
  map.mp_.local_update_range_ = Eigen::Vector3d::Constant(1.0);
  map.mp_.fx_ = 1.0;
  map.mp_.fy_ = 1.0;
  map.mp_.cx_ = 0.0;
  map.mp_.cy_ = 0.0;
  map.mp_.frame_id_ = "world";
  map.mp_.obstacles_inflation_ = 0.0;
  map.mp_.enable_manual_map_ = false;
  map.mp_.static_preinflated_map_enable_ = false;
  map.mp_.virtual_ceil_height_ = -1.0;
  map.md_.occupancy_buffer_inflate_.assign(8U, 0);
  map.md_.manual_occupancy_buffer_.assign(8U, 0);
  map.md_.static_preinflated_buffer_.assign(8U, 0);
  map.md_.local_bound_min_ = Eigen::Vector3i::Zero();
  map.md_.local_bound_max_ = Eigen::Vector3i::Constant(1);
  map.md_.authoritative_support_buffer_v2_.assign(8U, 1);
  map.md_.authoritative_support_valid_v2_ = true;
  map.md_.authoritative_support_complete_v2_ = true;
  map.md_.authoritative_support_evidence_basis_v2_ =
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  map.md_.authoritative_support_sequence_v2_ = 1U;
  map.md_.authoritative_support_min_v2_ = map.mp_.map_min_boundary_;
  map.md_.authoritative_support_max_v2_ = map.mp_.map_max_boundary_;
  map.md_.authoritative_support_halo_reconciled_v2_ = true;
  map.md_.authoritative_state_sequence_v2_ = 1U;
  map.md_.authoritative_state_time_ticks_v2_ = 1U;
  map.md_.authoritative_configuration_generation_v2_ = 1U;
  bindMapSupportToCurrentConfiguration(map);

  const std::shared_ptr<const SDFMapCaptureV2> before =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(before);
  std::atomic<bool> mutation_ready(false);
  std::atomic<bool> release_mutation(false);
  std::atomic<bool> capture_started(false);
  std::atomic<bool> capture_done(false);
  std::shared_ptr<const SDFMapCaptureV2> captured;
  std::thread mutator([&]() {
    const std::lock_guard<std::recursive_mutex> lock(
        map.authoritative_capture_mutex_v2_.mutex);
    map.md_.occupancy_buffer_inflate_[0U] = 1;
    mutation_ready.store(true, std::memory_order_release);
    while (!release_mutation.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    map.md_.occupancy_buffer_inflate_[1U] = 1;
    map.noteAuthoritativeMapMutationV2Locked();
  });

  while (!mutation_ready.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  std::thread capturer([&]() {
    capture_started.store(true, std::memory_order_release);
    captured = map.captureAuthoritativeSDFMapV2();
    capture_done.store(true, std::memory_order_release);
  });
  while (!capture_started.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  for (int spin = 0; spin < 10000 &&
       !capture_done.load(std::memory_order_acquire); ++spin) {
    std::this_thread::yield();
  }
  EXPECT_FALSE(capture_done.load(std::memory_order_acquire));
  release_mutation.store(true, std::memory_order_release);
  mutator.join();
  capturer.join();
  ASSERT_TRUE(captured);
  ASSERT_TRUE(sdfMapCaptureV2Consistent(*captured));
  EXPECT_GT(captured->accepted_state_sequence,
            before->accepted_state_sequence);
  EXPECT_GT(captured->accepted_state_notification_sequence,
            before->accepted_state_notification_sequence);
  const SDFMapAcceptedStateVisibilityV2 visibility =
      map.acceptedStateVisibilityV2();
  EXPECT_TRUE(visibility.valid);
  EXPECT_EQ(visibility.accepted_state_sequence,
            captured->accepted_state_sequence);
  EXPECT_EQ(visibility.accepted_state_notification_sequence,
            captured->accepted_state_notification_sequence);
  EXPECT_EQ(captured->occupied[0U], 1U);
  EXPECT_EQ(captured->occupied[1U], 1U);
  EXPECT_EQ(before->occupied[0U], 0U);
  EXPECT_EQ(before->occupied[1U], 0U);
}

TEST(CloudOccupancySnapshotTest,
     AuthoritativeV2SupportDoesNotMixDepthIntoCompleteDomain) {
  SDFMap map;
  map.mp_ = MappingParameters();
  map.md_ = MappingData();
  map.mp_.map_origin_ = Eigen::Vector3d::Zero();
  map.mp_.map_size_ = Eigen::Vector3d::Constant(2.0);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_size_;
  map.mp_.map_voxel_num_ = Eigen::Vector3i::Constant(2);
  map.mp_.resolution_ = 1.0;
  map.mp_.resolution_inv_ = 1.0;
  map.mp_.local_update_range_ = Eigen::Vector3d::Constant(1.0);
  map.mp_.fx_ = 1.0;
  map.mp_.fy_ = 1.0;
  map.mp_.cx_ = 0.0;
  map.mp_.cy_ = 0.0;
  map.mp_.frame_id_ = "world";
  map.mp_.obstacles_inflation_ = 0.0;
  map.mp_.enable_manual_map_ = false;
  map.mp_.static_preinflated_map_enable_ = false;
  map.mp_.virtual_ceil_height_ = -1.0;
  map.md_.occupancy_buffer_inflate_.assign(8U, 0);
  map.md_.manual_occupancy_buffer_.assign(8U, 0);
  map.md_.static_preinflated_buffer_.assign(8U, 0);
  map.md_.authoritative_support_buffer_v2_.assign(8U, 0);
  map.md_.authoritative_state_sequence_v2_ = 1U;
  map.md_.authoritative_state_time_ticks_v2_ = 1U;
  map.md_.authoritative_configuration_generation_v2_ = 1U;
  map.md_.authoritative_support_halo_reconciled_v2_ = true;

  map.markAuthoritativeSupportV2Locked(
      Eigen::Vector3i(0, 0, 0),
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain);
  ASSERT_TRUE(map.md_.authoritative_support_valid_v2_);
  map.md_.authoritative_support_halo_reconciled_v2_ = true;
  map.markAuthoritativeSupportV2Locked(
      Eigen::Vector3i(1, 1, 1),
      kSDFMapCaptureSupportEvidenceDepthRaycast);
  EXPECT_EQ(map.md_.authoritative_support_buffer_v2_[0U], 1);
  EXPECT_EQ(map.md_.authoritative_support_buffer_v2_[7U], 0);

  const std::shared_ptr<const SDFMapCaptureV2> capture =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(capture);
  ASSERT_TRUE(sdfMapCaptureV2Consistent(*capture));
  EXPECT_EQ(capture->support.mask[0U], 1U);
  EXPECT_EQ(capture->support.mask[7U], 0U);
  EXPECT_EQ(certifySDFMapCaptureFreeBallV2(
                *capture, Eigen::Vector3d(0.5, 0.5, 0.5), 0.0)
                .status,
            SDFMapCaptureFreeBallStatusV2::CERTIFIED_FREE);
  EXPECT_EQ(certifySDFMapCaptureFreeBallV2(
                *capture, Eigen::Vector3d(1.5, 1.5, 1.5), 0.0)
                .status,
            SDFMapCaptureFreeBallStatusV2::UNKNOWN);
}

TEST(CloudOccupancySnapshotTest,
     AuthoritativeV2PredicateFailsClosedAtWorkBudget) {
  const int extent = 162;
  const std::size_t voxel_total = static_cast<std::size_t>(extent) *
      static_cast<std::size_t>(extent) * static_cast<std::size_t>(extent);
  SDFMapCaptureV2 capture;
  capture.valid = true;
  capture.map_instance_id = 91U;
  capture.configuration_generation = 2U;
  capture.configuration_key = 23U;
  capture.frame_id = "world";
  capture.accepted_state_sequence = 4U;
  capture.accepted_time_ticks = 8U;
  capture.map_min = Eigen::Vector3d::Zero();
  capture.map_max = Eigen::Vector3d::Constant(extent);
  capture.grid_origin = Eigen::Vector3d::Zero();
  capture.capture_min = capture.map_min;
  capture.capture_max = capture.map_max;
  capture.source_min_index = Eigen::Vector3i::Zero();
  capture.source_max_index = Eigen::Vector3i::Constant(extent - 1);
  capture.voxel_count = Eigen::Vector3i::Constant(extent);
  capture.resolution = 1.0;
  capture.included_map_inflation = 0.0;
  capture.occupied.assign(voxel_total, 0U);
  capture.support.valid = true;
  capture.support.complete = true;
  capture.support.evidence_basis =
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  capture.support.evidence_sequence = 4U;
  capture.support.evidence_accepted_ticks = capture.accepted_time_ticks;
  capture.support.map_instance_id = capture.map_instance_id;
  capture.support.configuration_generation = capture.configuration_generation;
  capture.support.configuration_key = capture.configuration_key;
  capture.support.frame_id = capture.frame_id;
  capture.support.support_min = capture.map_min;
  capture.support.support_max = capture.map_max;
  capture.support.halo_reconciled = true;
  capture.support.mask.assign(voxel_total, 1U);

  ASSERT_TRUE(sdfMapCaptureV2Consistent(capture));
  const SDFMapCaptureFreeBallResultV2 result =
      certifySDFMapCaptureFreeBallV2(
          capture, Eigen::Vector3d::Constant(extent / 2.0),
          extent / 2.0);
  EXPECT_EQ(result.status, SDFMapCaptureFreeBallStatusV2::INCONCLUSIVE);
  EXPECT_FALSE(result.clearance_certified);
}

TEST(CloudOccupancySnapshotTest,
     AuthoritativeV2CropEquivalenceUsesNonDyadicResolution) {
  SDFMap map;
  map.mp_ = MappingParameters();
  map.md_ = MappingData();
  map.mp_.map_origin_ = Eigen::Vector3d::Zero();
  map.mp_.map_size_ = Eigen::Vector3d::Constant(3.0);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_size_;
  map.mp_.map_voxel_num_ = Eigen::Vector3i::Constant(10);
  map.mp_.resolution_ = 0.3;
  map.mp_.resolution_inv_ = 1.0 / map.mp_.resolution_;
  map.mp_.local_update_range_ = Eigen::Vector3d::Constant(0.9);
  map.mp_.fx_ = 1.0;
  map.mp_.fy_ = 1.0;
  map.mp_.cx_ = 0.0;
  map.mp_.cy_ = 0.0;
  map.mp_.frame_id_ = "world";
  map.mp_.obstacles_inflation_ = 0.1;
  map.mp_.enable_manual_map_ = false;
  map.mp_.static_preinflated_map_enable_ = false;
  map.mp_.virtual_ceil_height_ = -1.0;
  const std::size_t voxel_total = 1000U;
  map.md_.occupancy_buffer_inflate_.assign(voxel_total, 0);
  map.md_.occupancy_buffer_inflate_[555U] = 1;
  map.md_.manual_occupancy_buffer_.assign(voxel_total, 0);
  map.md_.static_preinflated_buffer_.assign(voxel_total, 0);
  map.md_.local_bound_min_ = Eigen::Vector3i::Zero();
  map.md_.local_bound_max_ = Eigen::Vector3i::Constant(9);
  map.md_.has_cloud_ = false;
  map.md_.has_first_depth_ = false;
  map.md_.authoritative_support_buffer_v2_.assign(voxel_total, 1);
  map.md_.authoritative_support_valid_v2_ = true;
  map.md_.authoritative_support_complete_v2_ = true;
  map.md_.authoritative_support_evidence_basis_v2_ =
      kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  map.md_.authoritative_support_sequence_v2_ = 2U;
  map.md_.authoritative_support_min_v2_ = map.mp_.map_min_boundary_;
  map.md_.authoritative_support_max_v2_ = map.mp_.map_max_boundary_;
  map.md_.authoritative_support_halo_reconciled_v2_ = true;
  map.md_.authoritative_state_sequence_v2_ = 2U;
  map.md_.authoritative_state_time_ticks_v2_ = 22U;
  map.md_.authoritative_configuration_generation_v2_ = 1U;
  bindMapSupportToCurrentConfiguration(map);

  map.md_.authoritative_support_valid_until_v2_ = ros::Time(10.0);
  map.md_.authoritative_support_valid_until_ticks_v2_ = 1U;
  const std::shared_ptr<const SDFMapCaptureV2> quiet_expired =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(quiet_expired);
  EXPECT_FALSE(quiet_expired->support.valid);
  map.md_.authoritative_support_valid_until_v2_ = ros::Time();
  map.md_.authoritative_support_valid_until_ticks_v2_ = 0U;

  const std::shared_ptr<const SDFMapCaptureV2> full =
      map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(full);
  ASSERT_TRUE(sdfMapCaptureV2Consistent(*full));
  // The stored binary64 resolution is just below 0.3, so the exact native
  // grid upper face is below the nominal map boundary 3.0 by one sliver.
  // A ball that reaches the nominal boundary must therefore remain UNKNOWN;
  // an outward-rounded display bound must not certify it as inside the grid.
  const SDFMapCaptureFreeBallResultV2 native_edge_sliver =
      certifySDFMapCaptureFreeBallV2(
          *full, Eigen::Vector3d(2.75, 1.5, 1.5), 0.25);
  EXPECT_EQ(native_edge_sliver.status,
            SDFMapCaptureFreeBallStatusV2::UNKNOWN);
  EXPECT_FALSE(native_edge_sliver.clearance_certified);
  SDFMapCaptureRegionV2 region;
  region.valid = true;
  region.min = Eigen::Vector3d::Constant(0.75);
  region.max = Eigen::Vector3d::Constant(2.25);
  const std::shared_ptr<const SDFMapCaptureV2> cropped =
      map.captureAuthoritativeSDFMapV2(region);
  ASSERT_TRUE(cropped);
  ASSERT_TRUE(sdfMapCaptureV2Consistent(*cropped));
  EXPECT_TRUE(cropped->grid_origin.isApprox(full->grid_origin));
  EXPECT_TRUE(cropped->source_min_index.isApprox(Eigen::Vector3i::Constant(2)));
  EXPECT_TRUE(cropped->source_max_index.isApprox(Eigen::Vector3i::Constant(7)));
  EXPECT_TRUE(cropped->voxel_count.isApprox(Eigen::Vector3i::Constant(6)));

  const SDFMapCaptureFreeBallResultV2 full_free =
      certifySDFMapCaptureFreeBallV2(
          *full, Eigen::Vector3d::Constant(1.05), 0.05);
  const SDFMapCaptureFreeBallResultV2 crop_free =
      certifySDFMapCaptureFreeBallV2(
          *cropped, Eigen::Vector3d::Constant(1.05), 0.05);
  EXPECT_TRUE(full_free.enumerated_min_index.isApprox(Eigen::Vector3i::Constant(3)));
  EXPECT_TRUE(full_free.enumerated_max_index.isApprox(Eigen::Vector3i::Constant(3)));
  EXPECT_TRUE(crop_free.enumerated_min_index.isApprox(Eigen::Vector3i::Constant(1)));
  EXPECT_TRUE(crop_free.enumerated_max_index.isApprox(Eigen::Vector3i::Constant(1)));
  EXPECT_EQ(crop_free.status, full_free.status);
  EXPECT_EQ(crop_free.clearance_certified, full_free.clearance_certified);
  const SDFMapCaptureFreeBallResultV2 full_occupied =
      certifySDFMapCaptureFreeBallV2(
          *full, Eigen::Vector3d::Constant(1.65), 0.0);
  const SDFMapCaptureFreeBallResultV2 crop_occupied =
      certifySDFMapCaptureFreeBallV2(
          *cropped, Eigen::Vector3d::Constant(1.65), 0.0);
  EXPECT_EQ(crop_occupied.status, full_occupied.status);
  EXPECT_EQ(crop_occupied.clearance_certified,
            full_occupied.clearance_certified);
}

void initializeCompleteCloudProducer(SDFMap& map) {
  ros::Time::init();
  ros::Time::setNow(ros::Time(100.02));
  map.mp_ = MappingParameters();
  map.md_ = MappingData();
  initializeCloudCallbackMap(map);
  map.mp_.resolution_ = 0.5;
  map.mp_.resolution_inv_ = 2.0;
  map.mp_.map_voxel_num_ = Eigen::Vector3i::Constant(20);
  map.mp_.local_update_range_ = Eigen::Vector3d::Constant(4.0);
  map.mp_.obstacles_inflation_ = 0.49;
  map.mp_.frame_id_ = "world";
  map.mp_.virtual_ceil_height_ = -1.0;
  map.md_.occupancy_buffer_.assign(8000U, -3.0);
  map.md_.occupancy_buffer_inflate_.assign(8000U, 0);
  map.md_.distance_buffer_all_.assign(8000U, 10000.0);
  map.md_.distance_buffer_.assign(8000U, 10000.0);
  map.md_.distance_buffer_neg_.assign(8000U, 10000.0);
  map.md_.manual_occupancy_buffer_.assign(8000U, 0);
  map.md_.static_preinflated_buffer_.assign(8000U, 0);
  map.md_.authoritative_support_buffer_v2_.assign(8000U, 0);
  map.md_.esdf_need_update_ = false;
  map.cloud_complete_declared_v2_ = true;
  map.cloud_source_contract_v2_ = true;
  map.cloud_source_name_v2_ = "/map_generator";
  map.cloud_source_range_v2_ = Eigen::Vector3d::Constant(4.0);
  map.cloud_source_period_v2_ = 0.1;
  map.cloud_odom_history_v2_.clear();
  map.cloud_support_last_stamp_v2_ = ros::Time();
  auto odom = boost::make_shared<nav_msgs::Odometry>();
  odom->header.frame_id = "/simulator";
  odom->header.stamp = ros::Time(100.0);
  map.odomCallback(odom);
}

sensor_msgs::PointCloud2ConstPtr completeCloud(
    const std::vector<Eigen::Vector3d>& points = {}) {
  auto message = boost::make_shared<sensor_msgs::PointCloud2>(
      cloudMessage(points, ros::Time(100.0)));
  message->header.frame_id = "world";
  return message;
}

TEST(CloudOccupancySnapshotTest, CompleteEmptyObservationPreservesPlannerBuffers) {
  SDFMap map;
  initializeCompleteCloudProducer(map);
  const auto raw = map.md_.occupancy_buffer_;
  const auto inflated = map.md_.occupancy_buffer_inflate_;
  const auto edt = map.md_.distance_buffer_all_;
  map.cloudCallback(completeCloud());
  const auto capture = map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(capture);
  ASSERT_TRUE(capture->support.valid);
  EXPECT_TRUE(sdfMapCaptureV2Consistent(*capture));
  EXPECT_TRUE(certifySDFMapCaptureFreeBallV2(*capture,
      Eigen::Vector3d::Constant(0.25), 0.2).clearance_certified);
  EXPECT_FALSE(certifySDFMapCaptureFreeBallV2(*capture,
      Eigen::Vector3d(3.25,0.25,0.25), 0.2).clearance_certified);
  EXPECT_EQ(map.md_.occupancy_buffer_, raw);
  EXPECT_EQ(map.md_.occupancy_buffer_inflate_, inflated);
  EXPECT_EQ(map.md_.distance_buffer_all_, edt);
  EXPECT_FALSE(map.md_.esdf_need_update_);
}

TEST(CloudOccupancySnapshotTest, CompleteCloudHitsKeepLegacyInflationAndLayers) {
  SDFMap enabled, legacy;
  initializeCompleteCloudProducer(enabled);
  initializeCompleteCloudProducer(legacy);
  legacy.cloud_complete_declared_v2_ = false;
  const auto cloud = completeCloud({Eigen::Vector3d(0.25,0.25,0.25)});
  enabled.cloudCallback(cloud);
  legacy.cloudCallback(cloud);
  ASSERT_TRUE(enabled.md_.authoritative_support_valid_v2_);
  EXPECT_FALSE(legacy.md_.authoritative_support_valid_v2_);
  EXPECT_EQ(enabled.md_.occupancy_buffer_, legacy.md_.occupancy_buffer_);
  EXPECT_EQ(enabled.md_.occupancy_buffer_inflate_, legacy.md_.occupancy_buffer_inflate_);
  EXPECT_EQ(enabled.md_.distance_buffer_all_, legacy.md_.distance_buffer_all_);
  EXPECT_EQ(enabled.md_.esdf_need_update_, legacy.md_.esdf_need_update_);
  EXPECT_EQ(enabled.md_.local_bound_min_, legacy.md_.local_bound_min_);
  EXPECT_EQ(enabled.md_.local_bound_max_, legacy.md_.local_bound_max_);
  auto capture = enabled.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(capture);
  EXPECT_FALSE(certifySDFMapCaptureFreeBallV2(*capture,
      Eigen::Vector3d::Constant(0.25), 0.2).clearance_certified);
}

TEST(CloudOccupancySnapshotTest, CompleteCloudIntersectsSourceAndReceiverBoxes) {
  SDFMap map;
  initializeCompleteCloudProducer(map);
  auto latest = boost::make_shared<nav_msgs::Odometry>();
  latest->header.frame_id = "/simulator";
  latest->header.stamp = ros::Time(100.01);
  latest->pose.pose.position.x = 2.0;
  map.odomCallback(latest);
  map.cloud_source_range_v2_.x() = 3.0;
  map.cloudCallback(completeCloud());
  const auto capture = map.captureAuthoritativeSDFMapV2();
  ASSERT_TRUE(capture);
  ASSERT_TRUE(capture->support.valid);
  EXPECT_GT(capture->support.support_min.x(), -1.0);
  EXPECT_LT(capture->support.support_max.x(), 2.0);
  EXPECT_EQ(map.md_.camera_pos_.x(), 2.0);
  EXPECT_TRUE(certifySDFMapCaptureFreeBallV2(*capture,
      Eigen::Vector3d::Constant(0.25), 0.1).clearance_certified);
  EXPECT_FALSE(certifySDFMapCaptureFreeBallV2(*capture,
      Eigen::Vector3d(2.25,0.25,0.25), 0.1).clearance_certified);
}

TEST(CloudOccupancySnapshotTest, CompleteCloudRejectsMissingOrInvalidProvenance) {
  for (int fault = 0; fault != 8; ++fault) {
    SCOPED_TRACE(fault);
    SDFMap map;
    initializeCompleteCloudProducer(map);
    auto cloud = boost::make_shared<sensor_msgs::PointCloud2>(*completeCloud());
    switch (fault) {
      case 0: map.cloud_complete_declared_v2_ = false; break;
      case 1: map.cloud_source_contract_v2_ = false; break;
      case 2: cloud->header.frame_id = "other"; break;
      case 3: cloud->header.stamp = ros::Time(); break;
      case 4: cloud->header.stamp = ros::Time(100.005); break;
      case 5: map.cloud_odom_history_v2_.clear(); break;
      case 6: ros::Time::setNow(ros::Time(101.0)); break;
      case 7: map.cloud_source_range_v2_.x() = -1.0; break;
    }
    map.cloudCallback(cloud);
    EXPECT_FALSE(map.md_.authoritative_support_valid_v2_);
    EXPECT_FALSE(map.md_.authoritative_support_complete_v2_);
  }
}

TEST(CloudOccupancySnapshotTest, CompleteCloudClearingCannotBeUndoneByReplay) {
  SDFMap map;
  initializeCompleteCloudProducer(map);
  map.cloudCallback(completeCloud());
  ASSERT_TRUE(map.md_.authoritative_support_valid_v2_);
  map.resetBuffer(Eigen::Vector3d(-1,-1,-1), Eigen::Vector3d(1,1,1));
  EXPECT_FALSE(map.md_.authoritative_support_valid_v2_);
  map.cloudCallback(completeCloud());
  EXPECT_FALSE(map.md_.authoritative_support_valid_v2_);
  auto odom = boost::make_shared<nav_msgs::Odometry>();
  odom->header.frame_id = "/simulator";
  odom->header.stamp = ros::Time(100.01);
  map.odomCallback(odom);
  auto newer = boost::make_shared<sensor_msgs::PointCloud2>(*completeCloud());
  newer->header.stamp = odom->header.stamp;
  map.cloudCallback(newer);
  EXPECT_TRUE(map.md_.authoritative_support_valid_v2_);
}

TEST(CloudOccupancySnapshotTest, CompleteCloudNumericalAndFrameFaultsFailClosed) {
  SDFMap map;
  initializeCompleteCloudProducer(map);
  pcl::PointCloud<pcl::PointXYZ> invalid;
  invalid.push_back(pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(),0,0));
  map.acceptCloudSupportV2Locked(*completeCloud(), invalid);
  EXPECT_FALSE(map.md_.authoritative_support_valid_v2_);
  initializeCompleteCloudProducer(map);
  map.cloud_support_last_stamp_v2_ = ros::Time();
  const int rounding = std::fegetround();
  ASSERT_EQ(0, std::fesetround(FE_DOWNWARD));
  map.cloudCallback(completeCloud());
  EXPECT_FALSE(map.md_.authoritative_support_valid_v2_);
  ASSERT_EQ(0, std::fesetround(rounding));
  auto changed = boost::make_shared<nav_msgs::Odometry>();
  changed->header.stamp = ros::Time(100.01);
  changed->header.frame_id = "changed";
  map.odomCallback(changed);
  EXPECT_TRUE(map.cloud_odom_history_v2_.empty());
}

TEST(CloudOccupancySnapshotTest, SourceCompletenessChecksActualBackingEvenWhenEmpty) {
  pcl::PointCloud<pcl::PointXYZ> backing;
  const Eigen::Vector3d center = Eigen::Vector3d::Zero();
  const Eigen::Vector3d range = Eigen::Vector3d::Ones();
  EXPECT_TRUE(localSensingBoxComplete(backing, center, range, 0U));
  backing.emplace_back(1.0F,1.0F,1.0F);
  EXPECT_FALSE(localSensingBoxComplete(backing, center, range, 0U));
  EXPECT_TRUE(localSensingBoxComplete(backing, center, range, 1U));
  backing.emplace_back(std::nextafter(1.0F,2.0F),0.0F,0.0F);
  EXPECT_TRUE(localSensingBoxComplete(backing, center, range, 1U));
  EXPECT_FALSE(localSensingBoxComplete(backing, center, range, 2U));
  backing.emplace_back(std::numeric_limits<float>::quiet_NaN(),0.0F,0.0F);
  EXPECT_FALSE(localSensingBoxComplete(backing, center, range, 1U));
}

}  // namespace
}  // namespace plan_env

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
