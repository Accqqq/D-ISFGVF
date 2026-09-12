#include <gtest/gtest.h>

#define private public
#include "plan_env/sdf_map.h"
#undef private

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

namespace {

// The complete-cloud declaration is conservatively eroded by the receiver
// strict-open box and an occupancy halo before native voxels are exposed.  A
// four-metre source/receiver range leaves at least two complete cells at the
// fixture's one-metre resolution; callers can request the original coarse
// three-metre range for an explicit UNKNOWN negative test below.
void initializeFixture(SDFMap& map, const double sensing_range = 4.0,
                       const double receiver_range = 4.0,
                       const double resolution = 1.0) {
  // SDFMap keeps the historical value-copyable fixture surface, so make the
  // aggregate parameter/data records fully initialized before assigning the
  // fields relevant to this test.  This avoids reading an indeterminate
  // legacy bool when the copied-map validity test invokes SDFMap's copy.
  map.mp_ = MappingParameters();
  map.md_ = MappingData();
  map.mp_.resolution_ = resolution;
  map.mp_.resolution_inv_ = 1.0 / resolution;
  map.mp_.map_origin_ = Eigen::Vector3d::Zero();
  map.mp_.map_size_ = Eigen::Vector3d::Constant(8.0);
  const int voxel_count = static_cast<int>(std::lround(8.0 / resolution));
  map.mp_.map_voxel_num_ = Eigen::Vector3i::Constant(voxel_count);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_size_;
  map.mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  map.mp_.map_max_idx_ = map.mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  map.mp_.local_update_range_ = Eigen::Vector3d::Constant(receiver_range);
  map.mp_.obstacles_inflation_ = 0.0;
  map.mp_.frame_id_ = "world";
  map.mp_.ground_height_ = 0.0;
  map.mp_.enable_manual_map_ = true;
  map.md_.manual_boundary_enabled_ = false;
  map.mp_.static_preinflated_map_enable_ = false;
  map.mp_.clamp_min_log_ = -1.0;
  map.mp_.clamp_max_log_ = 1.0;
  map.mp_.min_occupancy_log_ = 0.5;
  map.mp_.unknown_flag_ = 0.01;

  const std::size_t size = static_cast<std::size_t>(voxel_count) *
      static_cast<std::size_t>(voxel_count) *
      static_cast<std::size_t>(voxel_count);
  map.md_.occupancy_buffer_.assign(size, 0.0);
  map.md_.occupancy_buffer_inflate_.assign(size, 0);
  map.md_.manual_occupancy_buffer_.assign(size, 0);
  map.md_.static_preinflated_buffer_.assign(size, 0);
  map.md_.distance_buffer_.assign(size, 10000.0);
  map.md_.distance_buffer_neg_.assign(size, 10000.0);
  map.md_.distance_buffer_all_.assign(size, 10000.0);
  map.md_.camera_pos_ = Eigen::Vector3d::Constant(4.0);
  map.md_.has_odom_ = true;

  map.static_cloud_complete_declared_ = true;
  map.static_cloud_contract_valid_ = true;
  map.static_cloud_source_name_ = "/local_sensing";
  map.static_cloud_source_range_ = Eigen::Vector3d::Constant(sensing_range);
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time(42.0);
  odom.header.frame_id = "world";
  odom.pose.pose.position.x = 4.0;
  odom.pose.pose.position.y = 4.0;
  odom.pose.pose.position.z = 4.0;
  map.static_cloud_odom_history_.push_back(odom);
}

sensor_msgs::PointCloud2::ConstPtr emptyCloud() {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2::Ptr message(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(cloud, *message);
  message->header.frame_id = "world";
  message->header.stamp = ros::Time(42.0);
  return message;
}

sensor_msgs::PointCloud2::ConstPtr emptyCloudAt(const ros::Time& stamp) {
  sensor_msgs::PointCloud2::Ptr message(
      new sensor_msgs::PointCloud2(*emptyCloud()));
  message->header.stamp = stamp;
  return message;
}

bool captureIsEnvironmentValid(const plan_env::LocalObstacleCapture& capture) {
  if (!capture.environment_validity ||
      !capture.environment_change_mutex) {
    return false;
  }
  const std::lock_guard<std::recursive_mutex> lock(
      *capture.environment_change_mutex);
  return capture.environment_validity->valid;
}

class CloudEventRecorder {
public:
  using Event = ros::MessageEvent<sensor_msgs::PointCloud2 const>;

  void callback(const Event& event) {
    const std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset(new Event(event));
    ++count_;
  }

  std::size_t count() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return count_;
  }

  std::shared_ptr<Event> latestAfter(const std::size_t previous_count) const {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (count_ <= previous_count || !latest_) return nullptr;
    return latest_;
  }

private:
  mutable std::mutex mutex_;
  std::size_t count_ = 0U;
  std::shared_ptr<Event> latest_;
};

std::shared_ptr<CloudEventRecorder::Event> publishAndWaitForEvent(
    ros::Publisher& publisher, CloudEventRecorder& recorder,
    const sensor_msgs::PointCloud2::ConstPtr& cloud) {
  const std::size_t previous_count = recorder.count();
  publisher.publish(cloud);
  for (int attempt = 0; attempt < 200; ++attempt) {
    ros::spinOnce();
    const std::shared_ptr<CloudEventRecorder::Event> event =
        recorder.latestAfter(previous_count);
    if (event) return event;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return nullptr;
}

plan_env::LocalObstacleRequest request(const double reference_min = 2.0,
                                       const double reference_max = 6.0) {
  plan_env::LocalObstacleRequest result;
  result.reference_region.min = Eigen::Vector3d::Constant(reference_min);
  result.reference_region.max = Eigen::Vector3d::Constant(reference_max);
  // These values must be ignored by SDFMap::readLocalObstacleView().
  result.known_region.min = Eigen::Vector3d::Constant(-100.0);
  result.known_region.max = Eigen::Vector3d::Constant(100.0);
  result.known_region_valid = true;
  return result;
}

}  // namespace

TEST(SDFMapLocalObstacleCapture, EmptyCompleteCloudEstablishesKnownDomain) {
  SDFMap map;
  initializeFixture(map);
  map.cloudCallback(emptyCloud());

  const plan_env::LocalObstacleCapture capture =
      map.readLocalObstacleView(request());
  EXPECT_EQ(capture.view.status, plan_env::LocalObstacleViewStatus::VALID);
  EXPECT_TRUE(capture.view.obstacles.empty());
  ASSERT_TRUE(capture.environment_validity);
  ASSERT_TRUE(capture.environment_change_mutex);
  const std::lock_guard<std::recursive_mutex> lock(
      *capture.environment_change_mutex);
  EXPECT_TRUE(capture.environment_validity->valid);
}

TEST(SDFMapLocalObstacleCapture, CoarseStrictHaloLeavesUnknownDomain) {
  SDFMap map;
  initializeFixture(map, 3.0, 3.0);
  map.cloudCallback(emptyCloud());

  // The caller's broad known_region is ignored.  With a one-metre grid, the
  // strict (1,7) receiver interval minus the two-metre Z halo has no whole
  // native voxel after conservative inward alignment.
  EXPECT_EQ(map.readLocalObstacleView(request()).view.status,
            plan_env::LocalObstacleViewStatus::UNKNOWN_DOMAIN);
}

TEST(SDFMapLocalObstacleCapture, ClosedSourceWithLargerReceiverRetainsBoundaryCells) {
  SDFMap map;
  initializeFixture(map, 3.0, 4.0);
  map.cloudCallback(emptyCloud());

  const plan_env::LocalObstacleCapture capture =
      map.readLocalObstacleView(request());
  ASSERT_EQ(capture.view.status, plan_env::LocalObstacleViewStatus::VALID);
  // The source box is closed and dominates the larger strict-open receiver;
  // exact one-metre faces therefore retain the native [2,6] XY and [3,5] Z
  // reference domain instead of losing a whole cell to nextafter rounding.
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.min.x(), 2.0);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.max.x(), 6.0);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.min.y(), 2.0);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.max.y(), 6.0);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.min.z(), 3.0);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.max.z(), 5.0);
}

TEST(SDFMapLocalObstacleCapture, ExtendedPrecisionKeepsSubVoxelStrictFace) {
  SDFMap map;
  const double receiver_range =
      std::nextafter(0.5, std::numeric_limits<double>::infinity());
  initializeFixture(map, 2.0, receiver_range, 0.125);
  map.md_.camera_pos_ = Eigen::Vector3d::Constant(1.5);
  map.static_cloud_source_range_ = Eigen::Vector3d::Constant(2.0);
  map.static_cloud_odom_history_.front().pose.pose.position.x = 1.5;
  map.static_cloud_odom_history_.front().pose.pose.position.y = 1.5;
  map.static_cloud_odom_history_.front().pose.pose.position.z = 1.5;
  map.cloudCallback(emptyCloud());

  const plan_env::LocalObstacleCapture capture =
      map.readLocalObstacleView(request(1.0, 2.0));
  ASSERT_EQ(capture.view.status, plan_env::LocalObstacleViewStatus::VALID);
  // center + nextafter(.5) is just above 2.0 in exact arithmetic.  Forming
  // that face and subtracting the .125 halo in long double keeps the final
  // strict upper face just above 1.875, so the [1.125,1.875] native box is
  // retained instead of being eroded to [1.125,1.75].
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.min.x(), 1.125);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.max.x(), 1.875);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.min.y(), 1.125);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.max.y(), 1.875);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.min.z(), 1.25);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.max.z(), 1.75);
}

TEST(SDFMapLocalObstacleCapture, ManualAndStaticLayersAreCopiedIntoView) {
  SDFMap map;
  initializeFixture(map);
  map.md_.manual_occupancy_buffer_[map.toAddress(Eigen::Vector3i(3, 3, 3))] = 1;
  map.md_.static_preinflated_buffer_[map.toAddress(Eigen::Vector3i(4, 4, 4))] = 1;
  map.cloudCallback(emptyCloud());

  const plan_env::LocalObstacleCapture capture =
      map.readLocalObstacleView(request());
  ASSERT_EQ(capture.view.status, plan_env::LocalObstacleViewStatus::VALID);
  ASSERT_EQ(capture.view.obstacles.size(), 2U);
  EXPECT_TRUE(std::any_of(
      capture.view.obstacles.begin(), capture.view.obstacles.end(),
      [](const plan_env::LocalObstacleVoxel& voxel) {
        return voxel.index == Eigen::Vector3i(3, 3, 3) &&
            (voxel.layer_mask & 0x02U) != 0U;
      }));
  EXPECT_TRUE(std::any_of(
      capture.view.obstacles.begin(), capture.view.obstacles.end(),
      [](const plan_env::LocalObstacleVoxel& voxel) {
        return voxel.index == Eigen::Vector3i(4, 4, 4) &&
            (voxel.layer_mask & 0x04U) != 0U;
      }));
}

TEST(SDFMapLocalObstacleCapture, ResetClearsFutureKnownDomain) {
  SDFMap map;
  initializeFixture(map);
  map.cloudCallback(emptyCloud());
  const plan_env::LocalObstacleCapture before =
      map.readLocalObstacleView(request());
  ASSERT_EQ(before.view.status, plan_env::LocalObstacleViewStatus::VALID);
  ASSERT_TRUE(captureIsEnvironmentValid(before));
  const Eigen::Vector3d before_min = before.view.reference_domain.min;
  const Eigen::Vector3d before_max = before.view.reference_domain.max;
  const std::size_t before_obstacle_count = before.view.obstacles.size();

  map.resetBuffer();
  const plan_env::LocalObstacleCapture capture =
      map.readLocalObstacleView(request());
  EXPECT_EQ(capture.view.status,
            plan_env::LocalObstacleViewStatus::UNKNOWN_DOMAIN);

  // Buffer reset only clears the declaration used by future captures.  The
  // immutable view already handed to a planner remains a coherent static
  // observation and its map-owned validity state is not invalidated.
  EXPECT_EQ(before.view.status, plan_env::LocalObstacleViewStatus::VALID);
  EXPECT_TRUE(captureIsEnvironmentValid(before));
  EXPECT_TRUE(before.view.reference_domain.min.isApprox(before_min));
  EXPECT_TRUE(before.view.reference_domain.max.isApprox(before_max));
  EXPECT_EQ(before.view.obstacles.size(), before_obstacle_count);
}

TEST(SDFMapLocalObstacleCapture, CaptureAndOccupancyWriteRemainCoherent) {
  SDFMap map;
  initializeFixture(map);
  map.cloudCallback(emptyCloud());

  std::atomic<bool> failed(false);
  std::thread writer([&map]() {
    const Eigen::Vector3d point(4.0, 4.0, 4.0);
    for (int i = 0; i < 256; ++i) {
      map.setOccupancy(point, (i & 1) == 0 ? 1.0 : 0.0);
    }
  });
  std::thread reader([&map, &failed]() {
    for (int i = 0; i < 256; ++i) {
      const plan_env::LocalObstacleCapture capture =
          map.readLocalObstacleView(request());
      if (capture.view.status != plan_env::LocalObstacleViewStatus::VALID &&
          capture.view.status !=
              plan_env::LocalObstacleViewStatus::UNKNOWN_DOMAIN) {
        failed.store(true);
        continue;
      }
      if (capture.view.status != plan_env::LocalObstacleViewStatus::VALID) {
        continue;
      }
      for (const plan_env::LocalObstacleVoxel& voxel :
           capture.view.obstacles) {
        if ((voxel.index.array() < 0).any() ||
            (voxel.index.array() >= Eigen::Vector3i::Constant(8).array()).any() ||
            !voxel.bounds.min.allFinite() || !voxel.bounds.max.allFinite() ||
            (voxel.bounds.min.array() > voxel.bounds.max.array()).any()) {
          failed.store(true);
          break;
        }
      }
    }
  });
  writer.join();
  reader.join();

  // The reader either sees the complete static declaration or the
  // fail-closed post-write state; it must never observe a torn voxel buffer
  // or an invented status while the map writer holds the same data lock.
  EXPECT_FALSE(failed.load());
}

TEST(SDFMapLocalObstacleCapture,
     RepeatedCloudDoesNotAcquireEnvironmentPublicationLock) {
  SDFMap map;
  initializeFixture(map);
  map.cloudCallback(emptyCloud());
  ASSERT_TRUE(map.environment_validity_store_.environment_change_mutex);

  std::unique_lock<std::recursive_mutex> environment_lock(
      *map.environment_validity_store_.environment_change_mutex);
  std::atomic<bool> callback_finished(false);
  std::thread callback_thread([&map, &callback_finished]() {
    // The duplicate timestamp is intentionally ordinary/repeated input.  It
    // may take the map-data lock but must not wait for this publication lock.
    map.cloudCallback(emptyCloud());
    callback_finished.store(true);
  });

  for (int attempt = 0; attempt < 100 && !callback_finished.load(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(callback_finished.load());
  environment_lock.unlock();
  callback_thread.join();
}

TEST(SDFMapLocalObstacleCapture, EnvironmentChangeInvalidatesPriorCaptureOnly) {
  SDFMap map;
  initializeFixture(map);
  map.cloudCallback(emptyCloud());
  const plan_env::LocalObstacleCapture before =
      map.readLocalObstacleView(request());
  ASSERT_EQ(before.view.status, plan_env::LocalObstacleViewStatus::VALID);

  geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped);
  point->header.frame_id = "world";
  point->point.x = 4.0;
  point->point.y = 4.0;
  point->point.z = 4.0;
  map.manualObstacleCallback(point);

  {
    const std::lock_guard<std::recursive_mutex> lock(
        *before.environment_change_mutex);
    EXPECT_FALSE(before.environment_validity->valid);
  }
  EXPECT_EQ(map.readLocalObstacleView(request()).view.status,
            plan_env::LocalObstacleViewStatus::UNKNOWN_DOMAIN);
}

TEST(SDFMapLocalObstacleCapture,
     ManualBoundaryTwoPointUpdateInvalidatesAndConstrainsCloud) {
  SDFMap map;
  initializeFixture(map, 4.0, 4.0);
  map.mp_.manual_boundary_padding_ = 0.0;
  // The boundary callback uses these explicit z limits for the manual box;
  // keeping one-cell room on either side leaves a nonempty post-halo domain.
  map.mp_.manual_boundary_z_min_ = 1.0;
  map.mp_.manual_boundary_z_max_ = 7.0;

  map.cloudCallback(emptyCloud());
  const plan_env::LocalObstacleCapture before =
      map.readLocalObstacleView(request());
  ASSERT_EQ(before.view.status, plan_env::LocalObstacleViewStatus::VALID);
  ASSERT_TRUE(captureIsEnvironmentValid(before));

  geometry_msgs::PointStamped::Ptr first(new geometry_msgs::PointStamped);
  first->header.frame_id = "world";
  first->point.x = 2.0;
  first->point.y = 2.0;
  first->point.z = 4.0;
  geometry_msgs::PointStamped::Ptr second(new geometry_msgs::PointStamped);
  second->header.frame_id = "world";
  second->point.x = 6.0;
  second->point.y = 6.0;
  second->point.z = 4.0;

  // Two real manual-boundary points form [2,6] in XY.  Each callback is an
  // environment change, so the old immutable capture is invalid and the
  // previous complete-cloud declaration cannot be reused for a future view.
  map.manualBoundaryCallback(first);
  map.manualBoundaryCallback(second);
  {
    const std::lock_guard<std::recursive_mutex> lock(
        *before.environment_change_mutex);
    EXPECT_FALSE(before.environment_validity->valid);
  }
  ASSERT_TRUE(map.md_.manual_boundary_enabled_);
  ASSERT_EQ(map.md_.manual_boundary_points_.size(), 2U);

  const plan_env::LocalObstacleCapture missing_refresh =
      map.readLocalObstacleView(request());
  EXPECT_EQ(missing_refresh.view.status,
            plan_env::LocalObstacleViewStatus::UNKNOWN_DOMAIN);

  // A fresh complete cloud is required after the boundary mutation.  The
  // source/receiver intersection is still [0,8], but the manual box clips it
  // to [2,6] XY and [1,7] Z before the native one-cell/two-cell halos.  The
  // resulting valid reference domain is therefore exactly [3,5]^3.
  map.cloudCallback(emptyCloud());
  const plan_env::LocalObstacleCapture after =
      map.readLocalObstacleView(request());
  ASSERT_EQ(after.view.status, plan_env::LocalObstacleViewStatus::VALID);
  ASSERT_TRUE(captureIsEnvironmentValid(after));
  EXPECT_DOUBLE_EQ(after.view.reference_domain.min.x(), 3.0);
  EXPECT_DOUBLE_EQ(after.view.reference_domain.max.x(), 5.0);
  EXPECT_DOUBLE_EQ(after.view.reference_domain.min.y(), 3.0);
  EXPECT_DOUBLE_EQ(after.view.reference_domain.max.y(), 5.0);
  EXPECT_DOUBLE_EQ(after.view.reference_domain.min.z(), 3.0);
  EXPECT_DOUBLE_EQ(after.view.reference_domain.max.z(), 5.0);
}

TEST(SDFMapLocalObstacleCapture, CopiedMapOwnsIndependentValidityState) {
  SDFMap map;
  initializeFixture(map);
  map.cloudCallback(emptyCloud());
  const plan_env::LocalObstacleCapture original =
      map.readLocalObstacleView(request());
  ASSERT_TRUE(original.environment_validity);
  ASSERT_TRUE(original.environment_change_mutex);
  {
    const std::lock_guard<std::recursive_mutex> lock(
        *original.environment_change_mutex);
    ASSERT_TRUE(original.environment_validity->valid);
  }
  SDFMap copy = map;
  const plan_env::LocalObstacleCapture copied_before =
      copy.readLocalObstacleView(request());
  ASSERT_TRUE(copied_before.environment_validity);
  ASSERT_TRUE(copied_before.environment_change_mutex);
  {
    const std::lock_guard<std::recursive_mutex> lock(
        *copied_before.environment_change_mutex);
    ASSERT_TRUE(copied_before.environment_validity->valid);
  }

  geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped);
  point->header.frame_id = "world";
  point->point.x = 4.0;
  point->point.y = 4.0;
  point->point.z = 4.0;
  copy.manualObstacleCallback(point);

  const std::lock_guard<std::recursive_mutex> lock(
      *original.environment_change_mutex);
  ASSERT_TRUE(original.environment_validity->valid);
  EXPECT_TRUE(original.environment_validity->valid);
  const std::lock_guard<std::recursive_mutex> copied_lock(
      *copied_before.environment_change_mutex);
  EXPECT_FALSE(copied_before.environment_validity->valid);
  const plan_env::LocalObstacleCapture copied_after =
      copy.readLocalObstacleView(request());
  ASSERT_TRUE(copied_after.environment_validity);
  ASSERT_TRUE(copied_after.environment_change_mutex);
  EXPECT_EQ(copied_after.view.status,
            plan_env::LocalObstacleViewStatus::UNKNOWN_DOMAIN);
  {
    const std::lock_guard<std::recursive_mutex> new_lock(
        *copied_after.environment_change_mutex);
    EXPECT_TRUE(copied_after.environment_validity->valid);
  }
}

TEST(SDFMapLocalObstacleCapture, SourceReceiverOffsetStillHonorsHalo) {
  SDFMap map;
  initializeFixture(map, 4.0, 4.0);
  map.md_.camera_pos_ = Eigen::Vector3d(4.0, 4.0, 4.0);
  map.static_cloud_odom_history_.clear();
  nav_msgs::Odometry source_odom;
  source_odom.header.stamp = ros::Time(42.0);
  source_odom.header.frame_id = "world";
  source_odom.pose.pose.position.x = 3.5;
  source_odom.pose.pose.position.y = 3.5;
  source_odom.pose.pose.position.z = 3.5;
  map.static_cloud_odom_history_.push_back(source_odom);

  map.cloudCallback(emptyCloud());
  const plan_env::LocalObstacleCapture capture =
      map.readLocalObstacleView(request());
  ASSERT_EQ(capture.view.status, plan_env::LocalObstacleViewStatus::VALID);
  // The source/receiver intersection is [0,7.5] (up to strict-open ULPs),
  // then the XY one-cell and Z two-cell halos are aligned inward to native
  // boxes [2,6] and [3,5], respectively.  This checks both offset handling
  // and the non-binary strict/halo boundary without demanding an inessential
  // extra shrink of the already-safe native domain.
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.min.x(), 2.0);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.max.x(), 6.0);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.min.y(), 2.0);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.max.y(), 6.0);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.min.z(), 3.0);
  EXPECT_DOUBLE_EQ(capture.view.reference_domain.max.z(), 5.0);
}

TEST(SDFMapLocalObstacleCapture, MissingDeclarationNeverTurnsCallerKnownIntoFree) {
  SDFMap map;
  initializeFixture(map);
  map.static_cloud_complete_declared_ = false;
  map.static_cloud_contract_valid_ = false;
  map.cloudCallback(emptyCloud());
  EXPECT_EQ(map.readLocalObstacleView(request()).view.status,
            plan_env::LocalObstacleViewStatus::UNKNOWN_DOMAIN);
}

TEST(SDFMapLocalObstacleCapture,
     UnmatchedProducerStampFallsBackToLatestOdomLikeBaseline) {
  // local_sensing stamps its complete cloud with the odom sample it had
  // already processed, while this map's own odom callback can still be a few
  // samples behind.  Requiring that exact stamp made the complete observation
  // unusable in practice.  The trusted baseline never compared stamps: it
  // attributed each cloud to the map's own latest odometry sample.  Keep that
  // behaviour, with the explicit complete-cloud declaration and the
  // frame/contract checks still mandatory.
  SDFMap map;
  initializeFixture(map);
  map.cloudCallback(emptyCloudAt(ros::Time(43.0)));
  EXPECT_EQ(map.readLocalObstacleView(request()).view.status,
            plan_env::LocalObstacleViewStatus::VALID);
}

TEST(SDFMapLocalObstacleCapture,
     ZeroStampOrFrameMismatchNeverEstablishesKnownDomain) {
  SDFMap zero_stamp_map;
  initializeFixture(zero_stamp_map);
  zero_stamp_map.cloudCallback(emptyCloudAt(ros::Time()));
  EXPECT_EQ(zero_stamp_map.readLocalObstacleView(request()).view.status,
            plan_env::LocalObstacleViewStatus::UNKNOWN_DOMAIN);

  SDFMap wrong_frame_map;
  initializeFixture(wrong_frame_map);
  sensor_msgs::PointCloud2::Ptr wrong_frame(
      new sensor_msgs::PointCloud2(*emptyCloudAt(ros::Time(43.0))));
  wrong_frame->header.frame_id = "not_world";
  wrong_frame_map.cloudCallback(wrong_frame);
  EXPECT_EQ(wrong_frame_map.readLocalObstacleView(request()).view.status,
            plan_env::LocalObstacleViewStatus::UNKNOWN_DOMAIN);
}

TEST(SDFMapLocalObstacleCapture,
     MessageEventSourceDefaultsAndContractChangesRequireIsolatedMaster) {
  if (std::getenv("M3C_ISOLATED_ROS_TESTS") == nullptr) {
    GTEST_SKIP() << "set M3C_ISOLATED_ROS_TESTS=1 under a task-owned ROS master";
  }
  if (!ros::isInitialized()) {
    int argc = 1;
    char name[] = "local_obstacle_capture_test";
    char* argv[] = {name, nullptr};
    ros::init(argc, argv, "local_obstacle_capture_test",
              ros::init_options::AnonymousName |
                  ros::init_options::NoSigintHandler);
  }
  if (!ros::master::check()) {
    GTEST_SKIP() << "task-owned ROS master is unavailable";
  }

  ros::NodeHandle nh;
  const std::string source_name = ros::this_node::getName();
  ASSERT_FALSE(source_name.empty());
  ros::NodeHandle source_nh(source_name);
  source_nh.setParam("sdf_map/local_update_range_x", 4.0);
  source_nh.setParam("sdf_map/local_update_range_y", 4.0);
  source_nh.setParam("sdf_map/local_update_range_z", 4.0);
  // Exercise the production defaults (world and /sim/odom) first.  The
  // source node's explicit output_frame/odom_topic values are deleted rather
  // than copied into a synthetic event header.
  source_nh.deleteParam("output_frame");
  source_nh.deleteParam("odom_topic");

  SDFMap map;
  initializeFixture(map);
  map.static_cloud_complete_declared_ = false;
  map.static_cloud_contract_valid_ = false;
  map.static_cloud_source_name_.clear();
  map.static_cloud_source_range_.setZero();
  map.static_cloud_odom_topic_ = nh.resolveName("/sim/odom");
  map.static_cloud_complete_param_ =
      nh.resolveName("phase_offset/tube/cloud_obstacle_set_complete");
  ros::param::set(map.static_cloud_complete_param_, true);

  CloudEventRecorder recorder;
  const std::string topic = "/local_obstacle_capture_test/cloud";
  // Let roscpp deduce the MessageEvent callback's transport type.  Explicitly
  // naming PointCloud2 here selects the ConstPtr overload and drops the
  // publisher/source metadata that this regression intentionally inspects.
  ros::Subscriber subscriber = nh.subscribe(
      topic, 1, &CloudEventRecorder::callback, &recorder);
  ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>(
      topic, 1, false);
  for (int attempt = 0; attempt < 200 && publisher.getNumSubscribers() == 0;
       ++attempt) {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(publisher.getNumSubscribers(), 0U);

  const std::shared_ptr<CloudEventRecorder::Event> first_event =
      publishAndWaitForEvent(publisher, recorder,
                             emptyCloudAt(ros::Time(42.0)));
  ASSERT_TRUE(first_event);
  ASSERT_FALSE(first_event->getPublisherName().empty());
  map.cloudObservationCallback(*first_event);
  const plan_env::LocalObstacleCapture first_capture =
      map.readLocalObstacleView(request());
  ASSERT_EQ(first_capture.view.status,
            plan_env::LocalObstacleViewStatus::VALID);
  EXPECT_EQ(first_event->getPublisherName(), map.static_cloud_source_name_);
  EXPECT_TRUE(map.static_cloud_contract_valid_);
  EXPECT_DOUBLE_EQ(map.static_cloud_source_range_.x(), 4.0);
  EXPECT_TRUE(captureIsEnvironmentValid(first_capture));

  // A real contract change from the same MessageEvent source invalidates the
  // old immutable environment witness.  The cloud remains a valid ROS event,
  // but a frame mismatch cannot establish a known free domain.
  source_nh.setParam("output_frame", "map");
  nav_msgs::Odometry odom43;
  odom43.header.stamp = ros::Time(43.0);
  odom43.header.frame_id = "world";
  odom43.pose.pose.position.x = 4.0;
  odom43.pose.pose.position.y = 4.0;
  odom43.pose.pose.position.z = 4.0;
  nav_msgs::Odometry::Ptr odom43_ptr(new nav_msgs::Odometry(odom43));
  map.odomCallback(odom43_ptr);
  const std::shared_ptr<CloudEventRecorder::Event> bad_event =
      publishAndWaitForEvent(publisher, recorder,
                             emptyCloudAt(ros::Time(43.0)));
  ASSERT_TRUE(bad_event);
  map.cloudObservationCallback(*bad_event);
  EXPECT_FALSE(captureIsEnvironmentValid(first_capture));
  EXPECT_EQ(map.readLocalObstacleView(request()).view.status,
            plan_env::LocalObstacleViewStatus::UNKNOWN_DOMAIN);
  EXPECT_FALSE(map.static_cloud_contract_valid_);

  // Removing the override restores the documented defaults; a new exact
  // source-time odometry sample and cloud establish a fresh valid state.
  source_nh.deleteParam("output_frame");
  nav_msgs::Odometry odom44 = odom43;
  odom44.header.stamp = ros::Time(44.0);
  nav_msgs::Odometry::Ptr odom44_ptr(new nav_msgs::Odometry(odom44));
  map.odomCallback(odom44_ptr);
  const std::shared_ptr<CloudEventRecorder::Event> restored_event =
      publishAndWaitForEvent(publisher, recorder,
                             emptyCloudAt(ros::Time(44.0)));
  ASSERT_TRUE(restored_event);
  map.cloudObservationCallback(*restored_event);
  const plan_env::LocalObstacleCapture restored_capture =
      map.readLocalObstacleView(request());
  EXPECT_EQ(restored_capture.view.status,
            plan_env::LocalObstacleViewStatus::VALID);
  EXPECT_TRUE(map.static_cloud_contract_valid_);
  EXPECT_TRUE(captureIsEnvironmentValid(restored_capture));

  subscriber.shutdown();
  publisher.shutdown();
  ros::param::del(map.static_cloud_complete_param_);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
