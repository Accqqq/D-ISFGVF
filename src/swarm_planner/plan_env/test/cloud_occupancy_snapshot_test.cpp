#include <gtest/gtest.h>

#include <plan_env/cloud_occupancy_snapshot.h>

#define private public
#include <plan_env/sdf_map.h>
#undef private

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/make_shared.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <random>

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

TEST(CloudOccupancySnapshotTest, NoOdomAndNoObservationAreInvalid) {
  CloudOccupancySnapshotBuildInput input = makeInput();
  input.odom_valid = false;
  const CloudOccupancySnapshot snapshot = buildCloudOccupancySnapshot(input);
  EXPECT_FALSE(snapshot.valid);
  EXPECT_FALSE(cloudOccupancySnapshotConsistent(snapshot));

  const CloudOccupancySnapshot absent;
  EXPECT_FALSE(cloudOccupancySnapshotConsistent(absent));
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

}  // namespace
}  // namespace plan_env

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
