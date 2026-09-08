/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include "plan_env/sdf_map.h"

#include <climits>
#include <chrono>
#include <cmath>
#include <limits>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

namespace {

void loadManualMapParams(ros::NodeHandle& nh, MappingParameters& mp) {
  nh.param("sdf_map/enable_manual_map", mp.enable_manual_map_, true);
  nh.param("sdf_map/manual_click_direct", mp.manual_click_direct_, false);
  nh.param("sdf_map/manual_obstacle_radius", mp.manual_obstacle_radius_, 0.35);
  nh.param("sdf_map/manual_obstacle_height", mp.manual_obstacle_height_, 2.5);
  nh.param("sdf_map/manual_obstacle_inflate", mp.manual_obstacle_inflate_, 0.10);
  nh.param("sdf_map/manual_boundary_padding", mp.manual_boundary_padding_, 0.10);
  nh.param("sdf_map/manual_boundary_z_min", mp.manual_boundary_z_min_, mp.ground_height_);
  nh.param("sdf_map/manual_boundary_z_max", mp.manual_boundary_z_max_, mp.virtual_ceil_height_);
  nh.param("sdf_map/manual_map_file", mp.manual_map_file_, std::string(""));
  nh.param("sdf_map/manual_map_auto_load", mp.manual_map_auto_load_, false);
  nh.param("sdf_map/manual_map_auto_save", mp.manual_map_auto_save_, false);
  nh.param("sdf_map/static_preinflated_map_enable",
           mp.static_preinflated_map_enable_, false);
  nh.param("sdf_map/static_preinflated_map_file",
           mp.static_preinflated_map_file_, std::string(""));
}

bool isInBaseMapBounds(const MappingParameters& mp, const Eigen::Vector3d& pos) {
  return pos(0) >= mp.map_min_boundary_(0) + 1e-4 &&
         pos(1) >= mp.map_min_boundary_(1) + 1e-4 &&
         pos(2) >= mp.map_min_boundary_(2) + 1e-4 &&
         pos(0) <= mp.map_max_boundary_(0) - 1e-4 &&
         pos(1) <= mp.map_max_boundary_(1) - 1e-4 &&
         pos(2) <= mp.map_max_boundary_(2) - 1e-4;
}

bool finiteVector(const Eigen::Vector3d& value) { return value.allFinite(); }

bool validCaptureMapGeometry(const MappingParameters& mp) {
  return finiteVector(mp.map_origin_) && finiteVector(mp.map_size_) &&
      finiteVector(mp.map_min_boundary_) && finiteVector(mp.map_max_boundary_) &&
      !mp.frame_id_.empty() && finiteVector(mp.local_update_range_) &&
      (mp.local_update_range_.array() > 0.0).all() &&
      std::isfinite(mp.resolution_) && mp.resolution_ > 0.0 &&
      std::isfinite(mp.obstacles_inflation_) &&
      mp.obstacles_inflation_ >= 0.0 &&
      mp.map_voxel_num_.x() > 0 && mp.map_voxel_num_.y() > 0 &&
      mp.map_voxel_num_.z() > 0 &&
      (mp.map_max_boundary_.array() > mp.map_min_boundary_.array()).all();
}

bool effectiveCaptureDomainV2(const MappingParameters& mp,
                             const MappingData& md,
                             Eigen::Vector3d& lower,
                             Eigen::Vector3d& upper) {
  lower = mp.map_min_boundary_;
  upper = mp.map_max_boundary_;
  if (!md.manual_boundary_enabled_) {
    return finiteVector(lower) && finiteVector(upper) &&
        (upper.array() > lower.array()).all();
  }
  // SDFMap::isInMap() treats this mutable manual box as an additional
  // admissible domain.  Use the unexpanded box here so the immutable capture
  // is a conservative subset of that planner domain; never widen it by the
  // legacy 1e-4 point threshold.
  if (!finiteVector(md.manual_boundary_min_) ||
      !finiteVector(md.manual_boundary_max_) ||
      (md.manual_boundary_max_.array() <
       md.manual_boundary_min_.array()).any()) {
    return false;
  }
  lower = lower.cwiseMax(md.manual_boundary_min_);
  upper = upper.cwiseMin(md.manual_boundary_max_);
  return finiteVector(lower) && finiteVector(upper) &&
      (upper.array() > lower.array()).all();
}

struct DirectedBoundsV2 {
  double lower = 0.0;
  double upper = 0.0;
};

bool finiteBoundsV2(const DirectedBoundsV2& bounds) {
  return std::isfinite(bounds.lower) && std::isfinite(bounds.upper) &&
      bounds.lower <= bounds.upper;
}

bool directedAddV2(const DirectedBoundsV2& left,
                   const DirectedBoundsV2& right,
                   DirectedBoundsV2& result) {
  const double lower = left.lower + right.lower;
  const double upper = left.upper + right.upper;
  if (!std::isfinite(lower) || !std::isfinite(upper)) return false;
  result.lower = std::nextafter(lower, -std::numeric_limits<double>::infinity());
  result.upper = std::nextafter(upper, std::numeric_limits<double>::infinity());
  return finiteBoundsV2(result);
}

bool directedSubV2(const DirectedBoundsV2& left,
                   const DirectedBoundsV2& right,
                   DirectedBoundsV2& result) {
  const double lower = left.lower - right.upper;
  const double upper = left.upper - right.lower;
  if (!std::isfinite(lower) || !std::isfinite(upper)) return false;
  result.lower = std::nextafter(lower, -std::numeric_limits<double>::infinity());
  result.upper = std::nextafter(upper, std::numeric_limits<double>::infinity());
  return finiteBoundsV2(result);
}

bool directedDivV2(const DirectedBoundsV2& left, const double divisor,
                   DirectedBoundsV2& result) {
  if (!std::isfinite(divisor) || divisor <= 0.0) return false;
  const double lower = left.lower / divisor;
  const double upper = left.upper / divisor;
  if (!std::isfinite(lower) || !std::isfinite(upper)) return false;
  result.lower = std::nextafter(lower, -std::numeric_limits<double>::infinity());
  result.upper = std::nextafter(upper, std::numeric_limits<double>::infinity());
  return finiteBoundsV2(result);
}

DirectedBoundsV2 scalarBoundsV2(const double value) {
  DirectedBoundsV2 result;
  result.lower = value;
  result.upper = value;
  return result;
}

bool captureIndexRangeForRegion(const DirectedBoundsV2& lower_coordinate,
                                const DirectedBoundsV2& upper_coordinate,
                                const int count, int& first, int& last) {
  if (!finiteBoundsV2(lower_coordinate) ||
      !finiteBoundsV2(upper_coordinate) || count <= 0 ||
      lower_coordinate.lower > upper_coordinate.upper) {
    return false;
  }
  // The interval endpoints already enclose every operation used to form the
  // requested region and map-space coordinates.  Use the outer endpoints
  // directly; no long-double-to-double narrowing or ordinary arithmetic is
  // allowed to shrink the set of intersected closed voxels.
  const double lower_value = lower_coordinate.lower;
  const double upper_value = upper_coordinate.upper;
  const double lower_out = std::nextafter(
      lower_value, -std::numeric_limits<double>::infinity());
  const double upper_out = std::nextafter(
      upper_value, std::numeric_limits<double>::infinity());
  const double first_value = std::ceil(std::nextafter(
      lower_out - 1.0, -std::numeric_limits<double>::infinity()));
  const double last_value = std::floor(upper_out);
  if (!std::isfinite(first_value) || !std::isfinite(last_value) ||
      first_value < static_cast<double>(INT_MIN) ||
      first_value > static_cast<double>(INT_MAX) ||
      last_value < static_cast<double>(INT_MIN) ||
      last_value > static_cast<double>(INT_MAX)) return false;
  first = std::max(0, static_cast<int>(first_value));
  last = std::min(count - 1, static_cast<int>(last_value));
  return first <= last;
}

std::size_t mapAddressChecked(const Eigen::Vector3i& index,
                              const Eigen::Vector3i& count) {
  return (static_cast<std::size_t>(index.x()) *
              static_cast<std::size_t>(count.y()) +
          static_cast<std::size_t>(index.y())) *
             static_cast<std::size_t>(count.z()) +
      static_cast<std::size_t>(index.z());
}

bool anyNonzero(const std::vector<char>& values) {
  for (const char value : values) {
    if (value != 0) return true;
  }
  return false;
}

bool validMapVoxelCount(const Eigen::Vector3i& count, std::size_t& size) {
  if (count.x() <= 0 || count.y() <= 0 || count.z() <= 0) return false;
  const std::size_t x = static_cast<std::size_t>(count.x());
  const std::size_t y = static_cast<std::size_t>(count.y());
  const std::size_t z = static_cast<std::size_t>(count.z());
  if (x > std::numeric_limits<std::size_t>::max() / y) return false;
  const std::size_t xy = x * y;
  if (xy > std::numeric_limits<std::size_t>::max() / z) return false;
  size = xy * z;
  return true;
}

std::uint64_t steadyTicksV2() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::uint64_t captureConfigurationKeyV2(
    const MappingParameters& mp, const bool manual_boundary_enabled,
    const Eigen::Vector3d& manual_boundary_min,
    const Eigen::Vector3d& manual_boundary_max) {
  std::uint64_t key = 1469598103934665603ULL;
  const auto mix = [&key](const void* data, const std::size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0U; i < size; ++i) {
      key ^= static_cast<std::uint64_t>(bytes[i]);
      key *= 1099511628211ULL;
    }
  };
  mix(mp.map_origin_.data(), sizeof(double) * 3U);
  mix(mp.map_size_.data(), sizeof(double) * 3U);
  mix(mp.map_min_boundary_.data(), sizeof(double) * 3U);
  mix(mp.map_max_boundary_.data(), sizeof(double) * 3U);
  mix(mp.map_voxel_num_.data(), sizeof(int) * 3U);
  mix(&mp.resolution_, sizeof(mp.resolution_));
  mix(&mp.obstacles_inflation_, sizeof(mp.obstacles_inflation_));
  mix(mp.local_update_range_.data(), sizeof(double) * 3U);
  mix(mp.frame_id_.data(), mp.frame_id_.size());
  mix(&mp.fx_, sizeof(mp.fx_));
  mix(&mp.fy_, sizeof(mp.fy_));
  mix(&mp.cx_, sizeof(mp.cx_));
  mix(&mp.cy_, sizeof(mp.cy_));
  mix(&mp.virtual_ceil_height_, sizeof(mp.virtual_ceil_height_));
  mix(&mp.enable_manual_map_, sizeof(mp.enable_manual_map_));
  mix(&mp.static_preinflated_map_enable_,
      sizeof(mp.static_preinflated_map_enable_));
  mix(&manual_boundary_enabled, sizeof(manual_boundary_enabled));
  // Disabled manual boundaries may have never been initialized in a synthetic
  // SDFMap fixture.  Do not read or hash those vectors unless the domain is
  // actually enabled and its values are part of the active map identity.
  if (manual_boundary_enabled) {
    mix(manual_boundary_min.data(), sizeof(double) * 3U);
    mix(manual_boundary_max.data(), sizeof(double) * 3U);
  }
  return key == 0U ? 1U : key;
}

}  // namespace

namespace plan_env {

namespace {

std::uint64_t allocateCaptureInstanceIdV2() {
  static std::atomic<std::uint64_t> next_id(1U);
  std::uint64_t current = next_id.load(std::memory_order_relaxed);
  for (;;) {
    if (current == 0U || current == std::numeric_limits<std::uint64_t>::max()) {
      return 0U;
    }
    if (next_id.compare_exchange_weak(current, current + 1U,
                                      std::memory_order_relaxed,
                                      std::memory_order_relaxed)) {
      return current;
    }
  }
}

}  // namespace

SDFMapCaptureMutexV2::SDFMapCaptureMutexV2()
    : instance_id(allocateCaptureInstanceIdV2()) {}

SDFMapCaptureMutexV2::SDFMapCaptureMutexV2(const SDFMapCaptureMutexV2&)
    : instance_id(allocateCaptureInstanceIdV2()) {}

SDFMapCaptureMutexV2& SDFMapCaptureMutexV2::operator=(
    const SDFMapCaptureMutexV2&) {
  instance_id = allocateCaptureInstanceIdV2();
  observed_configuration_key = 0U;
  support_binding_invalidated = false;
  return *this;
}

}  // namespace plan_env

void SDFMap::initMap(ros::NodeHandle& nh,const std::string& particle, const std::string& odom, const std::string& cloud) {

  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);

  /* get parameter */
  cloud_complete_param_v2_ = nh.resolveName("phase_offset/tube/cloud_obstacle_set_complete");
  cloud_odom_topic_v2_ = nh.resolveName(odom);
  cloud_odom_history_v2_.clear();
  cloud_support_last_stamp_v2_ = ros::Time();
  cloud_source_contract_v2_ = false;
  cloud_source_name_v2_.clear();
  cloud_complete_declared_v2_ = false;
  double x_size, y_size, z_size;
  nh.param("sdf_map/resolution", mp_.resolution_, -1.0);
  nh.param("sdf_map/map_size_x", x_size, -1.0);
  nh.param("sdf_map/map_size_y", y_size, -1.0);
  nh.param("sdf_map/map_size_z", z_size, -1.0);
  nh.param("sdf_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  nh.param("sdf_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  nh.param("sdf_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  nh.param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  nh.param("sdf_map/fx", mp_.fx_, -1.0);
  nh.param("sdf_map/fy", mp_.fy_, -1.0);
  nh.param("sdf_map/cx", mp_.cx_, -1.0);
  nh.param("sdf_map/cy", mp_.cy_, -1.0);

  nh.param("sdf_map/use_depth_filter", mp_.use_depth_filter_, true);
  nh.param("sdf_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  nh.param("sdf_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  nh.param("sdf_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  nh.param("sdf_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  nh.param("sdf_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  nh.param("sdf_map/skip_pixel", mp_.skip_pixel_, -1);

  nh.param("sdf_map/p_hit", mp_.p_hit_, 0.70);
  nh.param("sdf_map/p_miss", mp_.p_miss_, 0.35);
  nh.param("sdf_map/p_min", mp_.p_min_, 0.12);
  nh.param("sdf_map/p_max", mp_.p_max_, 0.97);
  nh.param("sdf_map/p_occ", mp_.p_occ_, 0.80);
  nh.param("sdf_map/min_ray_length", mp_.min_ray_length_, -0.1);
  nh.param("sdf_map/max_ray_length", mp_.max_ray_length_, -0.1);

  nh.param("sdf_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);
  nh.param("sdf_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  nh.param("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);

  nh.param("sdf_map/show_occ_time", mp_.show_occ_time_, false);
  nh.param("sdf_map/show_esdf_time", mp_.show_esdf_time_, false);
  nh.param("sdf_map/pose_type", mp_.pose_type_, 1);

  nh.param("sdf_map/frame_id", mp_.frame_id_, string("world"));
  nh.param("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);
  nh.param("sdf_map/local_map_margin", mp_.local_map_margin_, 1);
  nh.param("sdf_map/ground_height", mp_.ground_height_, 1.0);
  nh.param("sdf_map/buffer_refresh_period", mp_.buffer_refresh_period_, 0.0);
  loadManualMapParams(nh, mp_);

  mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  // cout << "hit: " << mp_.prob_hit_log_ << endl;
  // cout << "miss: " << mp_.prob_miss_log_ << endl;
  // cout << "min log: " << mp_.clamp_min_log_ << endl;
  // cout << "max: " << mp_.clamp_max_log_ << endl;
  // cout << "thresh log: " << mp_.min_occupancy_log_ << endl;
  // cout << "unknown_flag_: " << mp_.unknown_flag_ << endl;

  for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  // initialize data buffers

 int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_.resize(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);

  md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.manual_occupancy_buffer_ = vector<char>(buffer_size, 0);
  md_.static_preinflated_buffer_ = vector<char>(buffer_size, 0);
  md_.static_preinflated_map_loaded_ = false;
  md_.static_preinflated_map_ready_ = false;
  md_.static_preinflated_voxel_count_ = 0;
  md_.authoritative_support_buffer_v2_.assign(buffer_size, 0);
  md_.authoritative_support_valid_v2_ = false;
  md_.authoritative_support_complete_v2_ = false;
  md_.authoritative_support_evidence_basis_v2_ =
      plan_env::kSDFMapCaptureSupportEvidenceNone;
  md_.authoritative_support_sequence_v2_ = 0U;
  md_.authoritative_support_map_instance_id_v2_ = 0U;
  md_.authoritative_support_configuration_generation_v2_ = 0U;
  md_.authoritative_support_configuration_key_v2_ = 0U;
  md_.authoritative_support_frame_id_v2_.clear();
  md_.authoritative_support_accepted_ticks_v2_ = 0U;
  md_.authoritative_support_valid_until_ticks_v2_ = 0U;
  md_.authoritative_support_stamp_v2_ = ros::Time();
  md_.authoritative_support_valid_until_v2_ = ros::Time();
  md_.authoritative_support_min_v2_ = Eigen::Vector3d::Zero();
  md_.authoritative_support_max_v2_ = Eigen::Vector3d::Zero();
  md_.authoritative_support_halo_v2_ = 0.0;
  md_.authoritative_support_halo_reconciled_v2_ = false;
  md_.authoritative_state_sequence_v2_ = 0U;
  md_.authoritative_state_notification_sequence_v2_ = 0U;
  md_.authoritative_state_time_ticks_v2_ = 0U;
  md_.authoritative_state_stamp_v2_ = ros::Time();
  if (!md_.authoritative_configuration_identity_exhausted_v2_) {
    if (md_.authoritative_configuration_generation_v2_ ==
        std::numeric_limits<std::uint64_t>::max()) {
      md_.authoritative_configuration_identity_exhausted_v2_ = true;
      md_.authoritative_configuration_generation_v2_ = 0U;
    } else {
      ++md_.authoritative_configuration_generation_v2_;
      if (md_.authoritative_configuration_generation_v2_ == 0U) {
        md_.authoritative_configuration_generation_v2_ = 1U;
      }
    }
  }
  md_.manual_boundary_enabled_ = false;
  md_.manual_obstacle_centers_.clear();
  md_.manual_boundary_points_.clear();

  md_.distance_buffer_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_.tmp_buffer2_ = vector<double>(buffer_size, 0);
  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /* init callback */

  // use odometry and point cloud
  
  indep_cloud_sub_ = nh.subscribe(cloud, 10, &SDFMap::cloudObservationCallbackV2, this);
  indep_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom, 10, &SDFMap::odomCallback, this);

  occ_timer_ = nh.createTimer(ros::Duration(0.10), &SDFMap::updateOccupancyCallback, this);
  esdf_timer_ = nh.createTimer(ros::Duration(0.10), &SDFMap::updateESDFCallback, this);
  vis_timer_ = nh.createTimer(ros::Duration(0.10), &SDFMap::visCallback, this);
  if (mp_.buffer_refresh_period_ > 0.0) {
    buffer_timer_ = nh.createTimer(ros::Duration(mp_.buffer_refresh_period_), &SDFMap::bufferRefreshCallback, this);
  }

  map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"sdf_map/occupancy", 10);
  map_inf_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"sdf_map/occupancy_inflate", 10);
  esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"sdf_map/esdf", 10);
  update_range_pub_ = nh.advertise<visualization_msgs::Marker>(particle +"sdf_map/update_range", 10);
  //map_boundary_pub_ = nh.advertise<visualization_msgs::MarkerArray>(particle +"sdf_map/map_boundary", 10);

  unknown_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"sdf_map/unknown", 10);
  depth_pub_ = nh.advertise<sensor_msgs::PointCloud2>(particle +"sdf_map/depth_cloud", 10);
  if (mp_.enable_manual_map_) {
    manual_obstacle_sub_ = nh.subscribe<geometry_msgs::PointStamped>(
        "/manual_map/add_obstacle_center", 10, &SDFMap::manualObstacleCallback, this);
    manual_boundary_sub_ = nh.subscribe<geometry_msgs::PointStamped>(
        "/manual_map/add_boundary_point", 10, &SDFMap::manualBoundaryCallback, this);
    if (mp_.manual_click_direct_) {
      manual_click_sub_ = nh.subscribe<geometry_msgs::PointStamped>(
          "/clicked_point", 10, &SDFMap::manualObstacleCallback, this);
    }
  }
  manual_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/manual_map/occupancy", 1, true);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.esdf_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;

  md_.esdf_time_ = 0.0;
  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_esdf_time_ = 0.0;
  md_.max_fuse_time_ = 0.0;

  loadManualMapFile();
  loadStaticPreinflatedMapFile();

  rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  rand_noise2_ = normal_distribution<double>(0, 0.2);
  random_device rd;
  eng_ = default_random_engine(rd());
  
  std::cout << "\033[1;32m" << "success init SDF Map module" << "\033[0m" << std::endl;
}

void SDFMap::initMap(ros::NodeHandle& nh) {

  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  
  /* get parameter */
  double x_size, y_size, z_size;
  nh.param("sdf_map/resolution", mp_.resolution_, -1.0);
  nh.param("sdf_map/map_size_x", x_size, -1.0);
  nh.param("sdf_map/map_size_y", y_size, -1.0);
  nh.param("sdf_map/map_size_z", z_size, -1.0);
  nh.param("sdf_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  nh.param("sdf_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  nh.param("sdf_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  nh.param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  nh.param("sdf_map/fx", mp_.fx_, -1.0);
  nh.param("sdf_map/fy", mp_.fy_, -1.0);
  nh.param("sdf_map/cx", mp_.cx_, -1.0);
  nh.param("sdf_map/cy", mp_.cy_, -1.0);

  nh.param("sdf_map/use_depth_filter", mp_.use_depth_filter_, true);
  nh.param("sdf_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  nh.param("sdf_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  nh.param("sdf_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  nh.param("sdf_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  nh.param("sdf_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  nh.param("sdf_map/skip_pixel", mp_.skip_pixel_, -1);

  nh.param("sdf_map/p_hit", mp_.p_hit_, 0.70);
  nh.param("sdf_map/p_miss", mp_.p_miss_, 0.35);
  nh.param("sdf_map/p_min", mp_.p_min_, 0.12);
  nh.param("sdf_map/p_max", mp_.p_max_, 0.97);
  nh.param("sdf_map/p_occ", mp_.p_occ_, 0.80);
  nh.param("sdf_map/min_ray_length", mp_.min_ray_length_, -0.1);
  nh.param("sdf_map/max_ray_length", mp_.max_ray_length_, -0.1);

  nh.param("sdf_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);
  nh.param("sdf_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  nh.param("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);

  nh.param("sdf_map/show_occ_time", mp_.show_occ_time_, false);
  nh.param("sdf_map/show_esdf_time", mp_.show_esdf_time_, false);
  nh.param("sdf_map/pose_type", mp_.pose_type_, 1);

  nh.param("sdf_map/frame_id", mp_.frame_id_, string("world"));
  nh.param("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);
  nh.param("sdf_map/local_map_margin", mp_.local_map_margin_, 1);
  nh.param("sdf_map/ground_height", mp_.ground_height_, 1.0);
  nh.param("sdf_map/buffer_refresh_period", mp_.buffer_refresh_period_, 0.0);
  loadManualMapParams(nh, mp_);

  mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  // cout << "hit: " << mp_.prob_hit_log_ << endl;
  // cout << "miss: " << mp_.prob_miss_log_ << endl;
  // cout << "min log: " << mp_.clamp_min_log_ << endl;
  // cout << "max: " << mp_.clamp_max_log_ << endl;
  // cout << "thresh log: " << mp_.min_occupancy_log_ << endl;
  // cout << "unknown_flag_: " << mp_.unknown_flag_ << endl;

  for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.manual_occupancy_buffer_ = vector<char>(buffer_size, 0);
  md_.static_preinflated_buffer_ = vector<char>(buffer_size, 0);
  md_.static_preinflated_map_loaded_ = false;
  md_.static_preinflated_map_ready_ = false;
  md_.static_preinflated_voxel_count_ = 0;
  md_.authoritative_support_buffer_v2_.assign(buffer_size, 0);
  md_.authoritative_support_valid_v2_ = false;
  md_.authoritative_support_complete_v2_ = false;
  md_.authoritative_support_evidence_basis_v2_ =
      plan_env::kSDFMapCaptureSupportEvidenceNone;
  md_.authoritative_support_sequence_v2_ = 0U;
  md_.authoritative_support_map_instance_id_v2_ = 0U;
  md_.authoritative_support_configuration_generation_v2_ = 0U;
  md_.authoritative_support_configuration_key_v2_ = 0U;
  md_.authoritative_support_frame_id_v2_.clear();
  md_.authoritative_support_accepted_ticks_v2_ = 0U;
  md_.authoritative_support_valid_until_ticks_v2_ = 0U;
  md_.authoritative_support_stamp_v2_ = ros::Time();
  md_.authoritative_support_valid_until_v2_ = ros::Time();
  md_.authoritative_support_min_v2_ = Eigen::Vector3d::Zero();
  md_.authoritative_support_max_v2_ = Eigen::Vector3d::Zero();
  md_.authoritative_support_halo_v2_ = 0.0;
  md_.authoritative_support_halo_reconciled_v2_ = false;
  md_.authoritative_state_sequence_v2_ = 0U;
  md_.authoritative_state_notification_sequence_v2_ = 0U;
  md_.authoritative_state_time_ticks_v2_ = 0U;
  md_.authoritative_state_stamp_v2_ = ros::Time();
  if (!md_.authoritative_configuration_identity_exhausted_v2_) {
    if (md_.authoritative_configuration_generation_v2_ ==
        std::numeric_limits<std::uint64_t>::max()) {
      md_.authoritative_configuration_identity_exhausted_v2_ = true;
      md_.authoritative_configuration_generation_v2_ = 0U;
    } else {
      ++md_.authoritative_configuration_generation_v2_;
      if (md_.authoritative_configuration_generation_v2_ == 0U) {
        md_.authoritative_configuration_generation_v2_ = 1U;
      }
    }
  }
  md_.manual_boundary_enabled_ = false;
  md_.manual_obstacle_centers_.clear();
  md_.manual_boundary_points_.clear();

  md_.distance_buffer_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_.tmp_buffer2_ = vector<double>(buffer_size, 0);
  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /* init callback */
  // use odometry and point cloud

  indep_cloud_sub_ =
      nh.subscribe<sensor_msgs::PointCloud2>("/mock_map", 10, &SDFMap::cloudCallback, this);
  indep_odom_sub_ =
      nh.subscribe<nav_msgs::Odometry>("/particle0/odom", 10, &SDFMap::odomCallback, this);

  occ_timer_ = nh.createTimer(ros::Duration(0.10), &SDFMap::updateOccupancyCallback, this);
  esdf_timer_ = nh.createTimer(ros::Duration(0.10), &SDFMap::updateESDFCallback, this);
  vis_timer_ = nh.createTimer(ros::Duration(0.10), &SDFMap::visCallback, this);
  if (mp_.buffer_refresh_period_ > 0.0) {
    buffer_timer_ = nh.createTimer(ros::Duration(mp_.buffer_refresh_period_), &SDFMap::bufferRefreshCallback, this);
  }

  map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy", 10);
  map_inf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate", 10);
  esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = nh.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);
  map_boundary_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/sdf_map/map_boundary", 10);

  unknown_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  depth_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);
  if (mp_.enable_manual_map_) {
    manual_obstacle_sub_ = nh.subscribe<geometry_msgs::PointStamped>(
        "/manual_map/add_obstacle_center", 10, &SDFMap::manualObstacleCallback, this);
    manual_boundary_sub_ = nh.subscribe<geometry_msgs::PointStamped>(
        "/manual_map/add_boundary_point", 10, &SDFMap::manualBoundaryCallback, this);
    if (mp_.manual_click_direct_) {
      manual_click_sub_ = nh.subscribe<geometry_msgs::PointStamped>(
          "/clicked_point", 10, &SDFMap::manualObstacleCallback, this);
    }
  }
  manual_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/manual_map/occupancy", 1, true);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.esdf_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;

  md_.esdf_time_ = 0.0;
  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_esdf_time_ = 0.0;
  md_.max_fuse_time_ = 0.0;

  loadManualMapFile();
  loadStaticPreinflatedMapFile();

  rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  rand_noise2_ = normal_distribution<double>(0, 0.2);
  random_device rd;
  eng_ = default_random_engine(rd());

  ROS_INFO("success init!");
}

void SDFMap::noteAuthoritativeMapMutationV2Locked(const ros::Time& stamp) {
  if (md_.authoritative_configuration_generation_v2_ == 0U &&
      !md_.authoritative_configuration_identity_exhausted_v2_) {
    md_.authoritative_configuration_generation_v2_ = 1U;
  }
  if (md_.authoritative_state_identity_exhausted_v2_) return;
  if (md_.authoritative_state_sequence_v2_ ==
      std::numeric_limits<std::uint64_t>::max()) {
    md_.authoritative_state_identity_exhausted_v2_ = true;
    return;
  }
  ++md_.authoritative_state_sequence_v2_;
  // The notification is a copied accepted-state fact, not a raw-cloud
  // arrival counter.  Keep it monotone with the accepted identity and never
  // advance it on rejected/unaccepted callbacks.
  md_.authoritative_state_notification_sequence_v2_ =
      md_.authoritative_state_sequence_v2_;
  md_.authoritative_state_time_ticks_v2_ = steadyTicksV2();
  if (!stamp.isZero() &&
      (md_.authoritative_state_stamp_v2_.isZero() ||
       stamp >= md_.authoritative_state_stamp_v2_)) {
    md_.authoritative_state_stamp_v2_ = stamp;
  }
}

void SDFMap::invalidateAuthoritativeSupportV2Locked(
    const Eigen::Vector3i& min_id, const Eigen::Vector3i& max_id) {
  if (md_.authoritative_support_buffer_v2_.empty() ||
      mp_.map_voxel_num_.x() <= 0 || mp_.map_voxel_num_.y() <= 0 ||
      mp_.map_voxel_num_.z() <= 0) {
    md_.authoritative_support_valid_v2_ = false;
    md_.authoritative_support_complete_v2_ = false;
    md_.authoritative_support_evidence_basis_v2_ =
        plan_env::kSDFMapCaptureSupportEvidenceNone;
    return;
  }
  Eigen::Vector3i lower = min_id;
  Eigen::Vector3i upper = max_id;
  for (int axis = 0; axis < 3; ++axis) {
    lower(axis) = std::max(0, std::min(lower(axis), mp_.map_voxel_num_(axis) - 1));
    upper(axis) = std::max(0, std::min(upper(axis), mp_.map_voxel_num_(axis) - 1));
  }
  if ((lower.array() > upper.array()).any()) return;
  for (int x = lower.x(); x <= upper.x(); ++x)
    for (int y = lower.y(); y <= upper.y(); ++y)
      for (int z = lower.z(); z <= upper.z(); ++z) {
        const std::size_t address = mapAddressChecked(
            Eigen::Vector3i(x, y, z), mp_.map_voxel_num_);
        if (address < md_.authoritative_support_buffer_v2_.size()) {
          md_.authoritative_support_buffer_v2_[address] = 0;
        }
      }
  // A complete support declaration over a box is no longer true after any
  // relied-on cell is cleared.  Preserve the mask for diagnostics but require
  // fresh attributed evidence before another V2 certificate.
  md_.authoritative_support_valid_v2_ = false;
  md_.authoritative_support_complete_v2_ = false;
  md_.authoritative_support_evidence_basis_v2_ =
      plan_env::kSDFMapCaptureSupportEvidenceNone;
  md_.authoritative_support_sequence_v2_ = 0U;
  md_.authoritative_support_map_instance_id_v2_ = 0U;
  md_.authoritative_support_configuration_generation_v2_ = 0U;
  md_.authoritative_support_configuration_key_v2_ = 0U;
  md_.authoritative_support_frame_id_v2_.clear();
  md_.authoritative_support_accepted_ticks_v2_ = 0U;
  md_.authoritative_support_valid_until_ticks_v2_ = 0U;
  md_.authoritative_support_stamp_v2_ = ros::Time();
  md_.authoritative_support_valid_until_v2_ = ros::Time();
  md_.authoritative_support_min_v2_ = Eigen::Vector3d::Zero();
  md_.authoritative_support_max_v2_ = Eigen::Vector3d::Zero();
  md_.authoritative_support_halo_reconciled_v2_ = false;
}

void SDFMap::markAuthoritativeSupportV2Locked(
    const Eigen::Vector3i& index, const std::uint32_t evidence_basis,
    const ros::Time& stamp) {
  if (md_.authoritative_support_buffer_v2_.empty() ||
      !isInMap(index) || evidence_basis ==
          plan_env::kSDFMapCaptureSupportEvidenceNone) {
    return;
  }
  const std::size_t address = mapAddressChecked(index, mp_.map_voxel_num_);
  if (address >= md_.authoritative_support_buffer_v2_.size()) return;
  const bool complete_evidence =
      (evidence_basis &
       plan_env::kSDFMapCaptureSupportEvidenceCompletePreknownDomain) != 0U;
  const bool depth_evidence =
      (evidence_basis & plan_env::kSDFMapCaptureSupportEvidenceDepthRaycast) != 0U;
  const std::uint64_t current_configuration_key =
      captureConfigurationKeyV2(
          mp_, md_.manual_boundary_enabled_, md_.manual_boundary_min_,
          md_.manual_boundary_max_);
  const bool support_binding_mismatch =
      md_.authoritative_support_valid_v2_ &&
      md_.authoritative_support_complete_v2_ &&
      (md_.authoritative_support_map_instance_id_v2_ !=
           authoritative_capture_mutex_v2_.instance_id ||
       md_.authoritative_support_configuration_generation_v2_ !=
           md_.authoritative_configuration_generation_v2_ ||
       md_.authoritative_support_configuration_key_v2_ !=
           current_configuration_key ||
       md_.authoritative_support_frame_id_v2_ != mp_.frame_id_);
  const bool support_expiry_invalid_or_expired =
      (md_.authoritative_support_valid_until_v2_.isZero() !=
       (md_.authoritative_support_valid_until_ticks_v2_ == 0U)) ||
      (md_.authoritative_support_valid_until_ticks_v2_ != 0U &&
       (md_.authoritative_support_valid_until_ticks_v2_ <
            md_.authoritative_support_accepted_ticks_v2_ ||
        steadyTicksV2() > md_.authoritative_support_valid_until_ticks_v2_));
  if (support_binding_mismatch) {
    invalidateAuthoritativeSupportV2Locked(
        Eigen::Vector3i::Zero(),
        mp_.map_voxel_num_ - Eigen::Vector3i::Ones());
  }
  // Once a complete support domain is authoritative, ordinary depth/raycast
  // provenance outside that declared domain must not enlarge or relabel the
  // certified mask.  It remains ordinary map evidence, not V2 support.
  if (depth_evidence && !complete_evidence &&
      md_.authoritative_support_valid_v2_ &&
      md_.authoritative_support_complete_v2_) {
    return;
  }
  // A complete declaration after a provenance/configuration invalidation is a
  // new support cohort.  Clear every old mask cell before recording the
  // supplied voxel; otherwise restoring an old frame/configuration (or
  // clearing a prior invalid/expired declaration) could resurrect support for
  // cells that this fresh declaration never covered.  The producer must
  // explicitly re-establish any broader complete domain after this reset.
  if (complete_evidence &&
      (authoritative_capture_mutex_v2_.support_binding_invalidated ||
       support_expiry_invalid_or_expired ||
       !md_.authoritative_support_valid_v2_ ||
       (md_.authoritative_support_evidence_basis_v2_ &
        plan_env::kSDFMapCaptureSupportEvidenceCompletePreknownDomain) == 0U)) {
    invalidateAuthoritativeSupportV2Locked(
        Eigen::Vector3i::Zero(),
        mp_.map_voxel_num_ - Eigen::Vector3i::Ones());
  }
  // Do not scan the full map for every traversed ray voxel.  The producer's
  // support-valid bit is the serialization-boundary fact; bounds are updated
  // incrementally below and rebuilt only by explicit maintenance paths.
  const bool had_support = md_.authoritative_support_valid_v2_;
  md_.authoritative_support_buffer_v2_[address] = 1;
  // Per-voxel depth/raycast observations are recorded as provenance but do
  // not by themselves establish a complete, persistent forward domain.  Only
  // an explicitly attributed complete-domain declaration may make the support
  // usable by V2; ordinary source updates leave these false.
  if (complete_evidence) {
    md_.authoritative_support_valid_v2_ = true;
    md_.authoritative_support_complete_v2_ = true;
    md_.authoritative_support_map_instance_id_v2_ =
        authoritative_capture_mutex_v2_.instance_id;
    md_.authoritative_support_configuration_generation_v2_ =
        md_.authoritative_configuration_generation_v2_;
    md_.authoritative_support_configuration_key_v2_ =
        current_configuration_key;
    md_.authoritative_support_frame_id_v2_ = mp_.frame_id_;
    md_.authoritative_support_accepted_ticks_v2_ =
        md_.authoritative_state_time_ticks_v2_;
    authoritative_capture_mutex_v2_.support_binding_invalidated = false;
  }
  md_.authoritative_support_evidence_basis_v2_ |= evidence_basis;
  md_.authoritative_support_sequence_v2_ =
      std::max(md_.authoritative_support_sequence_v2_,
               md_.authoritative_state_sequence_v2_);
  if (md_.authoritative_support_sequence_v2_ == 0U) {
    md_.authoritative_support_sequence_v2_ = 1U;
  }
  if (!stamp.isZero() &&
      (md_.authoritative_support_stamp_v2_.isZero() ||
       stamp >= md_.authoritative_support_stamp_v2_)) {
    md_.authoritative_support_stamp_v2_ = stamp;
  }
  Eigen::Vector3d lower;
  Eigen::Vector3d upper;
  indexToPos(index, lower);
  lower.array() -= 0.5 * mp_.resolution_;
  upper = lower.array() + mp_.resolution_;
  if (!had_support) {
    md_.authoritative_support_min_v2_ = lower;
    md_.authoritative_support_max_v2_ = upper;
  } else {
    md_.authoritative_support_min_v2_ =
        md_.authoritative_support_min_v2_.cwiseMin(lower);
    md_.authoritative_support_max_v2_ =
        md_.authoritative_support_max_v2_.cwiseMax(upper);
  }
}

void SDFMap::rebuildAuthoritativeSupportBoundsV2Locked() {
  if (md_.authoritative_support_buffer_v2_.empty() ||
      !anyNonzero(md_.authoritative_support_buffer_v2_)) {
    md_.authoritative_support_valid_v2_ = false;
    md_.authoritative_support_complete_v2_ = false;
    md_.authoritative_support_evidence_basis_v2_ =
        plan_env::kSDFMapCaptureSupportEvidenceNone;
    md_.authoritative_support_min_v2_ = Eigen::Vector3d::Zero();
    md_.authoritative_support_max_v2_ = Eigen::Vector3d::Zero();
    return;
  }
  Eigen::Vector3d lower = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d upper = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
  for (int x = 0; x < mp_.map_voxel_num_.x(); ++x)
    for (int y = 0; y < mp_.map_voxel_num_.y(); ++y)
      for (int z = 0; z < mp_.map_voxel_num_.z(); ++z) {
        const Eigen::Vector3i index(x, y, z);
        const std::size_t address = mapAddressChecked(index, mp_.map_voxel_num_);
        if (address >= md_.authoritative_support_buffer_v2_.size() ||
            md_.authoritative_support_buffer_v2_[address] == 0) continue;
        Eigen::Vector3d cell_lower;
        indexToPos(index, cell_lower);
        cell_lower.array() -= 0.5 * mp_.resolution_;
        const Eigen::Vector3d cell_upper =
            cell_lower.array() + mp_.resolution_;
        lower = lower.cwiseMin(cell_lower);
        upper = upper.cwiseMax(cell_upper);
      }
  if (!finiteVector(lower) || !finiteVector(upper) ||
      (upper.array() < lower.array()).any()) {
    md_.authoritative_support_valid_v2_ = false;
    md_.authoritative_support_complete_v2_ = false;
    md_.authoritative_support_evidence_basis_v2_ =
        plan_env::kSDFMapCaptureSupportEvidenceNone;
    return;
  }
  md_.authoritative_support_min_v2_ = lower;
  md_.authoritative_support_max_v2_ = upper;
}

void SDFMap::resetBuffer() {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_.distance_buffer_[toAddress(x, y, z)] = 10000;
      }

  applyStaticPreinflatedLayer();
  invalidateAuthoritativeSupportV2Locked(min_id, max_id);
  noteAuthoritativeMapMutationV2Locked();
}

void SDFMap::gradualResetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  // 渐进式清空缓冲区，避免突然清空造成闪烁
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);
  
  boundIndex(min_id);
  boundIndex(max_id);
  
  // 计算清空区域的大小
  int total_voxels = (max_id(0) - min_id(0) + 1) * 
                    (max_id(1) - min_id(1) + 1) * 
                    (max_id(2) - min_id(2) + 1);
  
  // 使用静态变量跟踪清空进度
  static int clear_progress = 0;
  static bool is_clearing = false;
  
  // 如果不在清空过程中，开始新的清空周期
  if (!is_clearing) {
    clear_progress = 0;
    is_clearing = true;
  }
  
  // 每次清空一小批体素，实现平滑的淡出效果
  int clear_batch_size = std::max(1, total_voxels / 20); // 每次清空5%的体素，更平滑
  int cleared_count = 0;
  
  // 从外向内清空，创造更自然的淡出效果
  int center_x = (min_id(0) + max_id(0)) / 2;
  int center_y = (min_id(1) + max_id(1)) / 2;
  int center_z = (min_id(2) + max_id(2)) / 2;
  
  for (int x = min_id(0); x <= max_id(0) && cleared_count < clear_batch_size; ++x) {
    for (int y = min_id(1); y <= max_id(1) && cleared_count < clear_batch_size; ++y) {
      for (int z = min_id(2); z <= max_id(2) && cleared_count < clear_batch_size; ++z) {
        // 计算到中心的距离，优先清空边缘的体素
        int dist_to_center = abs(x - center_x) + abs(y - center_y) + abs(z - center_z);
        int max_dist = abs(max_id(0) - center_x) + abs(max_id(1) - center_y) + abs(max_id(2) - center_z);
        
        // 根据距离和进度决定是否清空
        bool should_clear = false;
        if (clear_progress < total_voxels / 4) {
          // 前25%：清空最外层的体素
          should_clear = (dist_to_center >= max_dist * 0.7);
        } else if (clear_progress < total_voxels / 2) {
          // 25%-50%：清空中间层的体素
          should_clear = (dist_to_center >= max_dist * 0.4 && dist_to_center < max_dist * 0.7);
        } else if (clear_progress < total_voxels * 3 / 4) {
          // 50%-75%：清空内层的体素
          should_clear = (dist_to_center >= max_dist * 0.1 && dist_to_center < max_dist * 0.4);
        } else {
          // 最后25%：清空中心区域的体素
          should_clear = (dist_to_center < max_dist * 0.1);
        }
        
        if (should_clear) {
          int idx = toAddress(x, y, z);
          md_.occupancy_buffer_inflate_[idx] = 0;
          md_.distance_buffer_[idx] = 10000;
          cleared_count++;
        }
        clear_progress++;
      }
    }
  }
  
  // 如果已经清空完所有体素，重置状态
  if (clear_progress >= total_voxels) {
    clear_progress = 0;
    is_clearing = false;
  }
  
  // 如果这次没有清空任何体素且不在清空过程中，说明需要开始新的清空周期
  if (cleared_count == 0 && !is_clearing) {
    resetBuffer(min_pos, max_pos);
  }
  if (cleared_count > 0) {
    invalidateAuthoritativeSupportV2Locked(min_id, max_id);
    noteAuthoritativeMapMutationV2Locked();
  }
}

void SDFMap::manualObstacleCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  if (!mp_.enable_manual_map_) return;

  Eigen::Vector3d center(msg->point.x, msg->point.y, msg->point.z);
  if (!isInBaseMapBounds(mp_, center)) {
    ROS_WARN("[MANUAL_MAP] ignore obstacle outside base map: %.3f %.3f %.3f",
             center(0), center(1), center(2));
    return;
  }

  md_.manual_obstacle_centers_.push_back(center);
  addManualCylinder(center);
  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_max_idx_;
  applyManualLayer();
  applyStaticPreinflatedLayer();
  noteAuthoritativeMapMutationV2Locked();
  md_.esdf_need_update_ = true;
  publishManualMap();
  saveManualMapFile();

  ROS_WARN("[MANUAL_MAP] add obstacle center: %.3f %.3f %.3f",
           center(0), center(1), center(2));
}

void SDFMap::manualBoundaryCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  if (!mp_.enable_manual_map_) return;

  Eigen::Vector3d point(msg->point.x, msg->point.y, msg->point.z);
  if (!isInBaseMapBounds(mp_, point)) {
    ROS_WARN("[MANUAL_MAP] ignore boundary point outside base map: %.3f %.3f %.3f",
             point(0), point(1), point(2));
    return;
  }

  // Boundary updates first clear the prior manual occupancy and may shrink
  // the planner-admissible domain.  Neither operation is evidence that the
  // cleared cells are free, so invalidate all relied-on support before the
  // destructive update rather than allowing an old complete mask to survive.
  invalidateAuthoritativeSupportV2Locked(
      Eigen::Vector3i::Zero(),
      mp_.map_voxel_num_ - Eigen::Vector3i::Ones());
  for (size_t i = 0; i < md_.manual_occupancy_buffer_.size(); ++i) {
    if (md_.manual_occupancy_buffer_[i] == 0) continue;
    md_.occupancy_buffer_inflate_[i] = 0;
    if (md_.occupancy_buffer_[i] >= mp_.clamp_max_log_) {
      md_.occupancy_buffer_[i] = mp_.clamp_min_log_ - mp_.unknown_flag_;
    }
  }
  std::fill(md_.manual_occupancy_buffer_.begin(), md_.manual_occupancy_buffer_.end(), 0);

  md_.manual_boundary_points_.push_back(point);
  md_.manual_boundary_enabled_ = md_.manual_boundary_points_.size() >= 2;

  if (md_.manual_boundary_enabled_) {
    Eigen::Vector3d min_pt = md_.manual_boundary_points_.front();
    Eigen::Vector3d max_pt = md_.manual_boundary_points_.front();
    for (const auto& p : md_.manual_boundary_points_) {
      min_pt = min_pt.cwiseMin(p);
      max_pt = max_pt.cwiseMax(p);
    }

    md_.manual_boundary_min_ = Eigen::Vector3d(
        std::max(mp_.map_min_boundary_(0), min_pt(0) - mp_.manual_boundary_padding_),
        std::max(mp_.map_min_boundary_(1), min_pt(1) - mp_.manual_boundary_padding_),
        std::max(mp_.map_min_boundary_(2), mp_.manual_boundary_z_min_));
    md_.manual_boundary_max_ = Eigen::Vector3d(
        std::min(mp_.map_max_boundary_(0), max_pt(0) + mp_.manual_boundary_padding_),
        std::min(mp_.map_max_boundary_(1), max_pt(1) + mp_.manual_boundary_padding_),
        std::min(mp_.map_max_boundary_(2), mp_.manual_boundary_z_max_));
  }

  for (const auto& center : md_.manual_obstacle_centers_) {
    addManualCylinder(center);
  }
  addManualBoundaryWalls();

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_max_idx_;
  applyManualLayer();
  applyStaticPreinflatedLayer();
  noteAuthoritativeMapMutationV2Locked();
  md_.esdf_need_update_ = true;
  publishManualMap();

  ROS_WARN("[MANUAL_MAP] add boundary point: %.3f %.3f %.3f, count: %zu",
           point(0), point(1), point(2), md_.manual_boundary_points_.size());
}

void SDFMap::addManualCylinder(const Eigen::Vector3d& center) {
  if (md_.manual_occupancy_buffer_.empty()) return;

  const double radius = std::max(0.0, mp_.manual_obstacle_radius_ + mp_.manual_obstacle_inflate_);
  const double z_min = std::max(mp_.map_min_boundary_(2), mp_.manual_boundary_z_min_);
  const double ceil_limit = mp_.virtual_ceil_height_ > -0.5 ? mp_.virtual_ceil_height_ : mp_.map_max_boundary_(2);
  const double z_max = std::min(std::min(mp_.manual_obstacle_height_, ceil_limit), mp_.map_max_boundary_(2));
  if (radius <= 0.0 || z_max < z_min) return;

  Eigen::Vector3i min_id, max_id;
  posToIndex(Eigen::Vector3d(center(0) - radius, center(1) - radius, z_min), min_id);
  posToIndex(Eigen::Vector3d(center(0) + radius, center(1) + radius, z_max), max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  const double radius_sq = radius * radius;
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        const double dx = pos(0) - center(0);
        const double dy = pos(1) - center(1);
        if (dx * dx + dy * dy <= radius_sq) {
          md_.manual_occupancy_buffer_[toAddress(x, y, z)] = 1;
        }
      }
}

void SDFMap::addManualBoundaryWalls() {
  if (!md_.manual_boundary_enabled_ || md_.manual_occupancy_buffer_.empty()) return;

  const double thickness = std::max(mp_.resolution_, mp_.manual_obstacle_inflate_);
  Eigen::Vector3i min_id, max_id;
  posToIndex(md_.manual_boundary_min_, min_id);
  posToIndex(md_.manual_boundary_max_, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        const bool on_x_wall = pos(0) <= md_.manual_boundary_min_(0) + thickness ||
                               pos(0) >= md_.manual_boundary_max_(0) - thickness;
        const bool on_y_wall = pos(1) <= md_.manual_boundary_min_(1) + thickness ||
                               pos(1) >= md_.manual_boundary_max_(1) - thickness;
        if (on_x_wall || on_y_wall) {
          md_.manual_occupancy_buffer_[toAddress(x, y, z)] = 1;
        }
      }
}

void SDFMap::applyManualLayer() {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  if (!mp_.enable_manual_map_ || md_.manual_occupancy_buffer_.empty()) return;

  bool has_manual_voxel = false;
  const size_t buffer_size = md_.manual_occupancy_buffer_.size();
  for (size_t addr = 0; addr < buffer_size; ++addr) {
    if (md_.manual_occupancy_buffer_[addr] == 0) continue;
    md_.occupancy_buffer_[addr] = mp_.clamp_max_log_;
    md_.occupancy_buffer_inflate_[addr] = 1;
    has_manual_voxel = true;
  }

  if (has_manual_voxel) {
    md_.esdf_need_update_ = true;
    noteAuthoritativeMapMutationV2Locked();
  }
}

void SDFMap::applyStaticPreinflatedLayer() {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  if (!mp_.static_preinflated_map_enable_ ||
      md_.static_preinflated_buffer_.empty()) {
    return;
  }

  const size_t buffer_size = std::min(
      md_.static_preinflated_buffer_.size(),
      md_.occupancy_buffer_inflate_.size());
  for (size_t addr = 0; addr < buffer_size; ++addr) {
    if (md_.static_preinflated_buffer_[addr] != 0) {
      md_.occupancy_buffer_inflate_[addr] = 1;
    }
  }
  noteAuthoritativeMapMutationV2Locked();
}

void SDFMap::publishManualMap() {
  if (!manual_map_pub_) return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
        if (md_.manual_occupancy_buffer_.empty() ||
            md_.manual_occupancy_buffer_[toAddress(x, y, z)] == 0) {
          continue;
        }

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (mp_.visualization_truncate_height_ > -0.5 &&
            pos(2) > mp_.visualization_truncate_height_) {
          continue;
        }

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  manual_map_pub_.publish(cloud_msg);
}

void SDFMap::loadManualMapFile() {
  if (!mp_.enable_manual_map_ || !mp_.manual_map_auto_load_) return;
  if (mp_.manual_map_file_.empty()) return;

  std::ifstream file(mp_.manual_map_file_);
  if (!file.good()) {
    ROS_WARN("[MANUAL_MAP] manual map file not found, skip load: %s",
             mp_.manual_map_file_.c_str());
    return;
  }

  size_t loaded_count = 0;
  size_t skipped_count = 0;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;

    std::istringstream iss(line);
    double x, y, z;
    if (!(iss >> x >> y >> z)) {
      ++skipped_count;
      continue;
    }

    Eigen::Vector3d center(x, y, z);
    if (!isInBaseMapBounds(mp_, center)) {
      ++skipped_count;
      ROS_WARN("[MANUAL_MAP] skip saved obstacle outside base map: %.3f %.3f %.3f",
               center(0), center(1), center(2));
      continue;
    }

    md_.manual_obstacle_centers_.push_back(center);
    addManualCylinder(center);
    ++loaded_count;
  }

  if (loaded_count > 0) {
    md_.local_bound_min_ = Eigen::Vector3i::Zero();
    md_.local_bound_max_ = mp_.map_max_idx_;
    applyManualLayer();
    applyStaticPreinflatedLayer();
    md_.esdf_need_update_ = true;
    publishManualMap();
  }

  ROS_WARN("[MANUAL_MAP] loaded %zu obstacle centers from %s, skipped %zu",
           loaded_count, mp_.manual_map_file_.c_str(), skipped_count);
}

void SDFMap::saveManualMapFile() {
  if (!mp_.enable_manual_map_ || !mp_.manual_map_auto_save_) return;
  if (mp_.manual_map_file_.empty()) return;

  std::ofstream file(mp_.manual_map_file_);
  if (!file.good()) {
    ROS_WARN("[MANUAL_MAP] failed to save manual map file: %s",
             mp_.manual_map_file_.c_str());
    return;
  }

  file << "# manual obstacle centers: x y z\n";
  file << std::setprecision(17);
  for (const auto& center : md_.manual_obstacle_centers_) {
    file << center(0) << " " << center(1) << " " << center(2) << "\n";
  }

  ROS_WARN("[MANUAL_MAP] saved %zu obstacle centers to %s",
           md_.manual_obstacle_centers_.size(), mp_.manual_map_file_.c_str());
}

void SDFMap::loadStaticPreinflatedMapFile() {
  md_.static_preinflated_map_loaded_ = false;
  md_.static_preinflated_map_ready_ = false;
  md_.static_preinflated_voxel_count_ = 0;

  if (!mp_.static_preinflated_map_enable_) return;
  if (mp_.static_preinflated_map_file_.empty()) {
    ROS_ERROR("[STATIC_MAP] enabled but file path is empty");
    return;
  }
  if (md_.static_preinflated_buffer_.empty()) {
    ROS_ERROR("[STATIC_MAP] map buffer is not initialized");
    return;
  }

  std::ifstream file(mp_.static_preinflated_map_file_);
  if (!file.good()) {
    ROS_ERROR("[STATIC_MAP] file not found: %s",
              mp_.static_preinflated_map_file_.c_str());
    return;
  }

  std::fill(md_.static_preinflated_buffer_.begin(),
            md_.static_preinflated_buffer_.end(), 0);
  Eigen::Vector3i min_id = mp_.map_max_idx_;
  Eigen::Vector3i max_id = Eigen::Vector3i::Zero();
  size_t skipped_count = 0;
  size_t duplicate_count = 0;
  std::string line;
  while (std::getline(file, line)) {
    const size_t first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') continue;

    std::istringstream input(line);
    double x, y, z;
    if (!(input >> x >> y >> z) || !std::isfinite(x) ||
        !std::isfinite(y) || !std::isfinite(z)) {
      ++skipped_count;
      continue;
    }

    const Eigen::Vector3d position(x, y, z);
    if (!isInBaseMapBounds(mp_, position)) {
      ++skipped_count;
      continue;
    }

    Eigen::Vector3i id;
    posToIndex(position, id);
    boundIndex(id);
    const int address = toAddress(id);
    if (md_.static_preinflated_buffer_[address] != 0) {
      ++duplicate_count;
      continue;
    }

    md_.static_preinflated_buffer_[address] = 1;
    ++md_.static_preinflated_voxel_count_;
    min_id = min_id.cwiseMin(id);
    max_id = max_id.cwiseMax(id);
  }

  if (md_.static_preinflated_voxel_count_ == 0) {
    ROS_ERROR("[STATIC_MAP] no usable voxels loaded from %s, skipped=%zu",
              mp_.static_preinflated_map_file_.c_str(), skipped_count);
    return;
  }

  const Eigen::Vector3i esdf_padding = Eigen::Vector3i::Ones();
  md_.local_bound_min_ = min_id - esdf_padding;
  md_.local_bound_max_ = max_id + esdf_padding;
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
  applyManualLayer();
  applyStaticPreinflatedLayer();
  updateESDF3d();
  md_.esdf_need_update_ = false;
  md_.static_preinflated_map_loaded_ = true;
  md_.static_preinflated_map_ready_ = true;

  Eigen::Vector3d min_position;
  Eigen::Vector3d max_position;
  indexToPos(min_id, min_position);
  indexToPos(max_id, max_position);
  ROS_WARN(
      "[STATIC_MAP] ready=1 voxels=%zu skipped=%zu duplicates=%zu "
      "bounds_min=(%.3f,%.3f,%.3f) bounds_max=(%.3f,%.3f,%.3f) file=%s",
      md_.static_preinflated_voxel_count_, skipped_count, duplicate_count,
      min_position.x(), min_position.y(), min_position.z(),
      max_position.x(), max_position.y(), max_position.z(),
      mp_.static_preinflated_map_file_.c_str());
}

bool SDFMap::staticPreinflatedMapReady() const {
  return !mp_.static_preinflated_map_enable_ ||
         md_.static_preinflated_map_ready_;
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_.map_voxel_num_(dim)];
  double z[mp_.map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d() {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  Eigen::Vector3i min_esdf = md_.local_bound_min_;
  Eigen::Vector3i max_esdf = md_.local_bound_max_;
    // ROS_INFO_STREAM("min_esdf: " << min_esdf.transpose() << ", max_esdf: " << max_esdf.transpose());
  /* ========== compute positive DT ========== */

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 //  min(mp_.resolution_ * std::sqrt(val),
                 //      md_.distance_buffer_[toAddress(x, y, z)]);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        if (md_.occupancy_buffer_inflate_[idx] == 0) {
          md_.occupancy_buffer_neg[idx] = 1;

        } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
          md_.occupancy_buffer_neg[idx] = 0;
        } else {
          ROS_ERROR("what?");
        }
      }

  ros::Time t1, t2;

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_neg[x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                            y * mp_.map_voxel_num_(2) + z] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
      }
}

// void SDFMap::updateESDF3d() {
//   Eigen::Vector3i min_esdf = md_.local_bound_min_;
//   Eigen::Vector3i max_esdf = md_.local_bound_max_;

//   ros::Time t_start = ros::Time::now();

//   /* ========== compute positive DT ========== */
//   ros::Time t_pos_start = ros::Time::now();

//   for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
//     for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
//       fillESDF(
//           [&](int z) {
//             return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
//                 0 : std::numeric_limits<double>::max();
//           },
//           [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, 
//           min_esdf[2], max_esdf[2], 2);
//     }
//   }

//   for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
//     for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
//       fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
//                [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, 
//                min_esdf[1], max_esdf[1], 1);
//     }
//   }

//   for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
//     for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
//       fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
//                [&](int x, double val) {
//                  md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
//                },
//                min_esdf[0], max_esdf[0], 0);
//     }
//   }

//   ros::Duration pos_dt = ros::Time::now() - t_pos_start;
//   ROS_INFO_STREAM("Positive DT time: " << pos_dt.toSec() << " s");

//   /* ========== compute negative occupancy ========= */
//   ros::Time t_neg_occ_start = ros::Time::now();

//   for (int x = min_esdf(0); x <= max_esdf(0); ++x)
//     for (int y = min_esdf(1); y <= max_esdf(1); ++y)
//       for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
//         int idx = toAddress(x, y, z);
//         if (md_.occupancy_buffer_inflate_[idx] == 0) {
//           md_.occupancy_buffer_neg[idx] = 1;
//         } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
//           md_.occupancy_buffer_neg[idx] = 0;
//         } else {
//           ROS_ERROR("what?");
//         }
//       }

//   ros::Duration neg_occ_dt = ros::Time::now() - t_neg_occ_start;
//   ROS_INFO_STREAM("Negative occupancy assignment time: " << neg_occ_dt.toSec() << " s");

//   /* ========== compute negative DT ========== */
//   ros::Time t_neg_dt_start = ros::Time::now();

//   for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
//     for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
//       fillESDF(
//           [&](int z) {
//             return md_.occupancy_buffer_neg[
//                 x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
//                 y * mp_.map_voxel_num_(2) + z] == 1 ?
//                 0 : std::numeric_limits<double>::max();
//           },
//           [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, 
//           min_esdf[2], max_esdf[2], 2);
//     }
//   }

//   for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
//     for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
//       fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
//                [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, 
//                min_esdf[1], max_esdf[1], 1);
//     }
//   }

//   for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
//     for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
//       fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
//                [&](int x, double val) {
//                  md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
//                },
//                min_esdf[0], max_esdf[0], 0);
//     }
//   }

//   ros::Duration neg_dt = ros::Time::now() - t_neg_dt_start;
//   ROS_INFO_STREAM("Negative DT time: " << neg_dt.toSec() << " s");

//   /* ========== combine positive and negative DT ========== */
//   ros::Time t_combine_start = ros::Time::now();

//   for (int x = min_esdf(0); x <= max_esdf(0); ++x)
//     for (int y = min_esdf(1); y <= max_esdf(1); ++y)
//       for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
//         int idx = toAddress(x, y, z);
//         md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];
//         if (md_.distance_buffer_neg_[idx] > 0.0)
//           md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
//       }

//   ros::Duration combine_dt = ros::Time::now() - t_combine_start;
//   ROS_INFO_STREAM("Combine DT time: " << combine_dt.toSec() << " s");

//   ros::Duration total_dt = ros::Time::now() - t_start;
//   ROS_INFO_STREAM("Total SDFMap::updateESDF3d time: " << total_dt.toSec() << " s");
// }

int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  if (occ != 1 && occ != 0) return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1) md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void SDFMap::projectDepthImage() {
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t* row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

  // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;

  if (!mp_.use_depth_filter_) {
    for (int v = 0; v < rows; v++) {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u++) {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240) std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_q_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_) {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_) {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0) {
            depth = mp_.max_ray_length_ + 0.1;
          } else if (depth < mp_.depth_filter_mindist_) {
            continue;
          } else if (depth > mp_.depth_filter_maxdist_) {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false) {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows) {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_) {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            } else {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

void SDFMap::raycastProcess() {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0) return;

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w)) {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);

    } else {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      } else {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX) {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_) {
        continue;
      } else {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt)) {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX) {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) {
          break;
        } else {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  // determine the local bounding box for updating ESDF
  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  md_.local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
  md_.local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty()) {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ?
        mp_.prob_hit_log_ :
        mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      markAuthoritativeSupportV2Locked(
          idx, plan_env::kSDFMapCaptureSupportEvidenceDepthRaycast);
      continue;
    } else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      markAuthoritativeSupportV2Locked(
          idx, plan_env::kSDFMapCaptureSupportEvidenceDepthRaycast);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
        idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    markAuthoritativeSupportV2Locked(
        idx, plan_env::kSDFMapCaptureSupportEvidenceDepthRaycast);
  }
  noteAuthoritativeMapMutationV2Locked();
}

Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap() {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);
   std::cout << "min_cut_m: (" << min_cut_m.x() << ", " << min_cut_m.y() << ", " << min_cut_m.z() << ")\n";
  std::cout << "max_cut_m: (" << max_cut_m.x() << ", " << max_cut_m.y() << ", " << max_cut_m.z() << ")\n";

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

      for (int z = min_cut_m(2); z < min_cut(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

      for (int y = min_cut_m(1); y < min_cut(1); ++y) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

      for (int x = min_cut_m(0); x < min_cut(0); ++x) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k) {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor(
        (mp_.virtual_ceil_height_ - mp_.map_origin_(2)) *
        mp_.resolution_inv_);
    if (ceil_id < 0 || ceil_id >= mp_.map_voxel_num_(2)) {
      ROS_WARN_THROTTLE(
          5.0,
          "virtual ceiling %.3f is outside map z voxels [0,%d); skip ceiling layer",
          mp_.virtual_ceil_height_, mp_.map_voxel_num_(2));
    } else {
      for (int x = md_.local_bound_min_(0);
           x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1);
             y <= md_.local_bound_max_(1); ++y) {
          md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
        }
    }
  }
  invalidateAuthoritativeSupportV2Locked(md_.local_bound_min_,
                                         md_.local_bound_max_);
  noteAuthoritativeMapMutationV2Locked();
}

void SDFMap::visCallback(const ros::TimerEvent& /*event*/) {
  publishMap();
  publishMapInflate(false);
  publishUpdateRange();
  //publishMapBoundary();
  publishESDF();
  publishManualMap();

  // publishUnknown();
  // publishDepth();
}

void SDFMap::bufferRefreshCallback(const ros::TimerEvent& /*event*/){
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  if (!md_.has_odom_) return;
  
  // 检查无人机是否在移动，如果移动速度过快则延迟清空
  static Eigen::Vector3d last_camera_pos = md_.camera_pos_;
  static ros::Time last_check_time = ros::Time::now();
  
  ros::Time current_time = ros::Time::now();
  double time_diff = (current_time - last_check_time).toSec();
  
  if (time_diff > 0.1) { // 每0.1秒检查一次移动速度
    Eigen::Vector3d velocity = (md_.camera_pos_ - last_camera_pos) / time_diff;
    double speed = velocity.norm();
    
    // 如果移动速度过快（>1.0 m/s），延迟清空
    if (speed > 1.0) {
      last_camera_pos = md_.camera_pos_;
      last_check_time = current_time;
      return;
    }
    
    last_camera_pos = md_.camera_pos_;
    last_check_time = current_time;
  }
  
  // 渐进式清空缓冲区，避免突然清空造成闪烁
  this->gradualResetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                          md_.camera_pos_ + mp_.local_update_range_);
  applyManualLayer();
  applyStaticPreinflatedLayer();
}

void SDFMap::updateOccupancyCallback(const ros::TimerEvent& /*event*/) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  if (!md_.occ_need_update_) return;

  /* update occupancy */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  projectDepthImage();
  raycastProcess();

  if (md_.local_updated_) {
    clearAndInflateLocalMap();
    applyManualLayer();
    applyStaticPreinflatedLayer();
  }

  t2 = ros::Time::now();

  md_.fuse_time_ += (t2 - t1).toSec();
  md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  if (mp_.show_occ_time_)
    ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  if (md_.local_updated_) md_.esdf_need_update_ = true;
  md_.local_updated_ = false;
}

void SDFMap::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!md_.esdf_need_update_) return;

  /* esdf */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  updateESDF3d();

  t2 = ros::Time::now();

  md_.esdf_time_ += (t2 - t1).toSec();
  md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).toSec());

  if (mp_.show_esdf_time_)
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

  md_.esdf_need_update_ = false;
}

void SDFMap::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                     pose->pose.orientation.y, pose->pose.orientation.z);
  if (isInMap(md_.camera_pos_)) {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  } else {
    md_.occ_need_update_ = false;
  }
}

void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  if (md_.has_first_depth_) return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;

  // Keep source-time evidence separately: the planner still consumes exactly
  // the latest camera position above.  Out-of-order or ambiguous identities
  // cannot be used to establish a sensing box.
  if (!cloud_odom_history_v2_.empty() &&
      (odom->header.stamp <= cloud_odom_history_v2_.back().header.stamp ||
       odom->header.frame_id != cloud_odom_history_v2_.back().header.frame_id)) {
    cloud_odom_history_v2_.clear();
    return;
  }
  if (odom->header.stamp.isZero() || odom->header.frame_id.empty() ||
      !md_.camera_pos_.allFinite()) {
    cloud_odom_history_v2_.clear();
    return;
  }
  cloud_odom_history_v2_.push_back(*odom);
  // Storage bound, not an interpolation or a timing/safety margin.
  if (cloud_odom_history_v2_.size() > 256U) cloud_odom_history_v2_.pop_front();
}

void SDFMap::cloudObservationCallbackV2(
    const ros::MessageEvent<sensor_msgs::PointCloud2 const>& event) {
  // The explicit deployment declaration applies only to the immutable static
  // local_sensing producer and its fixed launch configuration, not arbitrary
  // registered obstacle clouds.  Inspect that publisher's actual namespace;
  // never guess a source box from the receiver's camera or point extrema.
  bool declared = false;
  ros::param::getCached(cloud_complete_param_v2_, declared);
  Eigen::Vector3d range = Eigen::Vector3d::Zero();
  std::string frame = "world", odom_topic = "/sim/odom";
  double rate = 10.0;
  const std::string source = event.getPublisherName();
  bool contract = false;
  if (declared && !source.empty()) {
    ros::NodeHandle source_nh(source);
    contract = source_nh.getParamCached("sdf_map/local_update_range_x", range.x()) &&
        source_nh.getParamCached("sdf_map/local_update_range_y", range.y()) &&
        source_nh.getParamCached("sdf_map/local_update_range_z", range.z());
    source_nh.getParamCached("output_frame", frame);
    source_nh.getParamCached("odom_topic", odom_topic);
    source_nh.getParamCached("sensing_rate", rate);
    contract = contract && source_nh.resolveName(odom_topic) == cloud_odom_topic_v2_ &&
        std::isfinite(rate) && rate >= 1.0 && range.allFinite() &&
        (range.array() > 0.0).all();
  }
  const std::lock_guard<std::recursive_mutex> lock(authoritative_capture_mutex_v2_.mutex);
  cloud_complete_declared_v2_ = declared;
  cloud_source_contract_v2_ = contract && frame == mp_.frame_id_ &&
      (cloud_source_name_v2_.empty() ||
       (cloud_source_contract_v2_ && cloud_source_name_v2_ == source &&
        (range.array() == cloud_source_range_v2_.array()).all() &&
        cloud_source_period_v2_ == 1.0 / rate));
  // This source holds a fixed static map and launch configuration.  A changed
  // source/configuration cannot relabel an in-flight cloud or resurrect its
  // support; reinitialization is required to establish a new source contract.
  if (cloud_source_contract_v2_) cloud_source_name_v2_ = source;
  cloud_source_range_v2_ = range;
  cloud_source_period_v2_ = contract ? 1.0 / rate : 0.0;
  cloudCallback(event.getConstMessage());
}

void SDFMap::acceptCloudSupportV2Locked(
    const sensor_msgs::PointCloud2& cloud,
    const pcl::PointCloud<pcl::PointXYZ>& points) {
  if (!cloud_complete_declared_v2_ && cloud_source_name_v2_.empty()) return;
  const auto invalidate = [this]() {
    invalidateAuthoritativeSupportV2Locked(Eigen::Vector3i::Zero(),
        mp_.map_voxel_num_ - Eigen::Vector3i::Ones());
  };
  if (!cloud_complete_declared_v2_ || !cloud_source_contract_v2_ ||
      !plan_env::sdfMapCaptureV2FloatingPointEnvironmentSupported() ||
      !validCaptureMapGeometry(mp_) || !md_.camera_pos_.allFinite() ||
      cloud.header.frame_id != mp_.frame_id_ || cloud.header.stamp.isZero() ||
      !cloud_source_range_v2_.allFinite() ||
      (cloud_source_range_v2_.array() <= 0.0).any() ||
      !std::isfinite(cloud_source_period_v2_) || cloud_source_period_v2_ <= 0.0) {
    invalidate();
    return;
  }
  // A late/duplicate raw observation does not invalidate an incumbent static
  // capture, nor can it resurrect support after clearing.
  if (cloud.header.stamp <= cloud_support_last_stamp_v2_) return;
  cloud_support_last_stamp_v2_ = cloud.header.stamp;
  const auto found = std::find_if(cloud_odom_history_v2_.begin(),
      cloud_odom_history_v2_.end(), [&cloud](const nav_msgs::Odometry& odom) {
        return odom.header.stamp == cloud.header.stamp;
      });
  const ros::Time now = ros::Time::now();
  if (found == cloud_odom_history_v2_.end() || now < cloud.header.stamp ||
      (now - cloud.header.stamp).toSec() > cloud_source_period_v2_) {
    invalidate();
    return;
  }
  const Eigen::Vector3d source(found->pose.pose.position.x,
      found->pose.pose.position.y, found->pose.pose.position.z);
  // local_sensing copies the numerical odometry coordinates without a frame
  // transform (legacy simulator odom uses /simulator, output uses world).
  // Exact matching on the SAME configured odom stream preserves that existing
  // convention; a changed odom frame clears the history in odomCallback.
  if (!source.allFinite()) { invalidate(); return; }
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      invalidate(); return;
    }
  }
  std::size_t size = 0U;
  if (!validMapVoxelCount(mp_.map_voxel_num_, size) ||
      md_.occupancy_buffer_inflate_.size() != size) { invalidate(); return; }
  Eigen::Vector3d lower, upper;
  if (!effectiveCaptureDomainV2(mp_, md_, lower, upper)) { invalidate(); return; }
  const double inf_step = std::ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  if (!std::isfinite(inf_step) || inf_step > INT_MAX - 2) { invalidate(); return; }
  Eigen::Vector3i first, last;
  double max_halo = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    DirectedBoundsV2 sl, su, rl, ru, lo, hi, index_lo, index_hi, offset;
    // Include one seed-voxel width beyond the actual XY/z inflation stencil.
    const double halo = std::nextafter((axis == 2 ? 2.0 : inf_step + 1.0) *
        mp_.resolution_, std::numeric_limits<double>::infinity());
    const DirectedBoundsV2 total_halo = scalarBoundsV2(halo);
    if (!directedSubV2(scalarBoundsV2(source(axis)), scalarBoundsV2(cloud_source_range_v2_(axis)), sl) ||
        !directedAddV2(scalarBoundsV2(source(axis)), scalarBoundsV2(cloud_source_range_v2_(axis)), su) ||
        !directedSubV2(scalarBoundsV2(md_.camera_pos_(axis)), scalarBoundsV2(mp_.local_update_range_(axis)), rl) ||
        !directedAddV2(scalarBoundsV2(md_.camera_pos_(axis)), scalarBoundsV2(mp_.local_update_range_(axis)), ru)) {
      invalidate(); return;
    }
    lower(axis) = std::max(lower(axis), std::max(sl.upper, rl.upper));
    upper(axis) = std::min(upper(axis), std::min(su.lower, ru.lower));
    if (!directedAddV2(scalarBoundsV2(lower(axis)), total_halo, lo) ||
        !directedSubV2(scalarBoundsV2(upper(axis)), total_halo, hi) ||
        !directedSubV2(scalarBoundsV2(lo.upper), scalarBoundsV2(mp_.map_origin_(axis)), offset) ||
        !directedDivV2(offset, mp_.resolution_, index_lo)) { invalidate(); return; }
    lower(axis) = lo.upper;
    upper(axis) = hi.lower;
    if (!directedSubV2(scalarBoundsV2(hi.lower), scalarBoundsV2(mp_.map_origin_(axis)), offset) ||
        !directedDivV2(offset, mp_.resolution_, index_hi)) { invalidate(); return; }
    const double start = std::ceil(index_lo.upper);
    const double end = std::floor(index_hi.lower) - 1.0;
    if (start < 0.0 || end >= mp_.map_voxel_num_(axis) || start > end) {
      invalidate(); return;
    }
    first(axis) = static_cast<int>(start);
    last(axis) = static_cast<int>(end);
    max_halo = std::max(max_halo, total_halo.upper);
  }
  std::vector<char> mask(size, 0);
  for (int x = first.x(); x <= last.x(); ++x)
    for (int y = first.y(); y <= last.y(); ++y)
      for (int z = first.z(); z <= last.z(); ++z)
        mask[mapAddressChecked(Eigen::Vector3i(x,y,z), mp_.map_voxel_num_)] = 1;
  // Empty complete observations also advance the authoritative evidence state
  // without touching occupancy, layers, local bounds, or the ESDF update flag.
  if (points.empty()) noteAuthoritativeMapMutationV2Locked(cloud.header.stamp);
  if (md_.authoritative_state_identity_exhausted_v2_) { invalidate(); return; }
  md_.authoritative_support_buffer_v2_.swap(mask);
  md_.authoritative_support_valid_v2_ = true;
  md_.authoritative_support_complete_v2_ = true;
  md_.authoritative_support_evidence_basis_v2_ =
      plan_env::kSDFMapCaptureSupportEvidenceCompletePreknownDomain;
  md_.authoritative_support_sequence_v2_ = md_.authoritative_state_sequence_v2_;
  md_.authoritative_support_accepted_ticks_v2_ = md_.authoritative_state_time_ticks_v2_;
  md_.authoritative_support_stamp_v2_ = cloud.header.stamp;
  // The selected producer retains one immutable static map.  This is a
  // timeless physical-domain assumption, NOT a grace period for dynamic data.
  // Native clearing/reset still invalidates the current backing's support.
  md_.authoritative_support_valid_until_v2_ = ros::Time();
  md_.authoritative_support_valid_until_ticks_v2_ = 0U;
  md_.authoritative_support_map_instance_id_v2_ = authoritative_capture_mutex_v2_.instance_id;
  md_.authoritative_support_configuration_generation_v2_ = md_.authoritative_configuration_generation_v2_;
  md_.authoritative_support_configuration_key_v2_ = captureConfigurationKeyV2(
      mp_, md_.manual_boundary_enabled_, md_.manual_boundary_min_, md_.manual_boundary_max_);
  md_.authoritative_support_frame_id_v2_ = mp_.frame_id_;
  md_.authoritative_support_min_v2_ = lower;
  md_.authoritative_support_max_v2_ = upper;
  md_.authoritative_support_halo_v2_ = max_halo;
  md_.authoritative_support_halo_reconciled_v2_ = true;
  authoritative_capture_mutex_v2_.support_binding_invalidated = false;
}

void SDFMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img) {

  const std::lock_guard<std::recursive_mutex> map_lock(
      authoritative_capture_mutex_v2_.mutex);

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  // PCL 1.10's empty PointCloud2 conversion dereferences &cloud[0].  Keep
  // empty clouds as genuine observations, but avoid that library bug; the
  // remainder of the callback still publishes the valid all-free snapshot.
  if (img->width != 0U && img->height != 0U) {
    pcl::fromROSMsg(*img, latest_cloud);
  }

  md_.has_cloud_ = true;

  if (!md_.has_odom_) {
    // std::cout << "no odom!" << std::endl;
    return;
  }

  // The cloud path intentionally does not write raw log odds.  Build a
  // separate immutable categorical snapshot from exactly this observation so
  // phase-offset tube construction never reads a mutable, unobserved raw
  // buffer while the planner keeps using its original inflated map / ESDF.
  plan_env::CloudOccupancySnapshotBuildInput snapshot_input;
  snapshot_input.odom_valid = true;
  snapshot_input.observation_stamp = img->header.stamp;
  snapshot_input.camera_position = md_.camera_pos_;
  snapshot_input.map_min = mp_.map_min_boundary_;
  snapshot_input.map_max = mp_.map_max_boundary_;
  snapshot_input.grid_origin = mp_.map_origin_;
  snapshot_input.voxel_count = mp_.map_voxel_num_;
  snapshot_input.local_update_range = mp_.local_update_range_;
  snapshot_input.resolution = mp_.resolution_;
  snapshot_input.obstacles_inflation = mp_.obstacles_inflation_;
  snapshot_input.cloud_points.reserve(latest_cloud.points.size());
  for (const pcl::PointXYZ& point : latest_cloud.points) {
    snapshot_input.cloud_points.emplace_back(point.x, point.y, point.z);
  }
  const std::shared_ptr<plan_env::CloudOccupancySnapshotStore> snapshot_store =
      cloud_occupancy_snapshot_store_;
  if (snapshot_store) {
    {
      std::lock_guard<std::mutex> lock(snapshot_store->mutex);
      snapshot_input.observation_sequence =
          ++snapshot_store->observation_sequence;
    }
    const plan_env::CloudOccupancySnapshot snapshot =
        plan_env::buildCloudOccupancySnapshot(snapshot_input);
    {
      std::lock_guard<std::mutex> lock(snapshot_store->mutex);
      // cloudCallback may run on the AsyncSpinner.  A slower older callback
      // must never overwrite a snapshot already published by a newer cloud
      // observation.  The immutable pointer is therefore monotonic in the
      // real observation sequence, while any build which captured the old
      // pointer remains internally consistent.
      if (!snapshot_store->latest ||
          snapshot.observation_sequence >=
              snapshot_store->latest->observation_sequence) {
        snapshot_store->latest =
            std::make_shared<const plan_env::CloudOccupancySnapshot>(snapshot);
      }
    }
  }

  if (latest_cloud.points.size() == 0) {
    acceptCloudSupportV2Locked(*img, latest_cloud);
    return;
  }

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))) return;

  // buffer will be refreshed periodically by timer if enabled

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2)) {

      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z) {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt)) continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  applyManualLayer();
  applyStaticPreinflatedLayer();
  noteAuthoritativeMapMutationV2Locked(img->header.stamp);
  acceptCloudSupportV2Locked(*img, latest_cloud);
  md_.esdf_need_update_ = true;
}

void SDFMap::publishMap() {
  // pcl::PointXYZ pt;
  // pcl::PointCloud<pcl::PointXYZ> cloud;

  // Eigen::Vector3i min_cut = md_.local_bound_min_ -
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  // Eigen::Vector3i max_cut = md_.local_bound_max_ +
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);

  // boundIndex(min_cut);
  // boundIndex(max_cut);

  // for (int x = min_cut(0); x <= max_cut(0); ++x)
  //   for (int y = min_cut(1); y <= max_cut(1); ++y)
  //     for (int z = min_cut(2); z <= max_cut(2); ++z) {

  //       if (md_.occupancy_buffer_[toAddress(x, y, z)] <= mp_.min_occupancy_log_) continue;

  //       Eigen::Vector3d pos;
  //       indexToPos(Eigen::Vector3i(x, y, z), pos);
  //       if (pos(2) > mp_.visualization_truncate_height_) continue;

  //       pt.x = pos(0);
  //       pt.y = pos(1);
  //       pt.z = pos(2);
  //       cloud.points.push_back(pt);
  //     }

  // cloud.width = cloud.points.size();
  // cloud.height = 1;
  // cloud.is_dense = true;
  // cloud.header.frame_id = mp_.frame_id_;

  // sensor_msgs::PointCloud2 cloud_msg;
  // pcl::toROSMsg(cloud, cloud_msg);
  // map_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void SDFMap::publishMapInflate(bool all_info) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info) {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

void SDFMap::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  // auto sz = max_cut - min_cut;
  // std::cout << "unknown ratio: " << cloud.width << "/" << sz(0) * sz(1) * sz(2) << "="
  //           << double(cloud.width) / (sz(0) * sz(1) * sz(2)) << std::endl;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void SDFMap::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt.x = md_.proj_points_[i][0];
    pt.y = md_.proj_points_[i][1];
    pt.z = md_.proj_points_[i][2];
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}

void SDFMap::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  indexToPos(md_.local_bound_min_, esdf_min_pos);
  indexToPos(md_.local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void SDFMap::publishMapBoundary() {
  // 创建红色矩形框显示地图范围（只显示边框，不填充）
  visualization_msgs::MarkerArray boundary_markers;
  
  // 获取地图边界
  Eigen::Vector3d min_bound = mp_.map_origin_;
  Eigen::Vector3d max_bound = mp_.map_origin_ + mp_.map_size_;
  
  // 创建12条边来构成矩形框
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;
  
  // 底面的4条边
  edges.push_back({Eigen::Vector3d(min_bound(0), min_bound(1), min_bound(2)), 
                   Eigen::Vector3d(max_bound(0), min_bound(1), min_bound(2))});
  edges.push_back({Eigen::Vector3d(max_bound(0), min_bound(1), min_bound(2)), 
                   Eigen::Vector3d(max_bound(0), max_bound(1), min_bound(2))});
  edges.push_back({Eigen::Vector3d(max_bound(0), max_bound(1), min_bound(2)), 
                   Eigen::Vector3d(min_bound(0), max_bound(1), min_bound(2))});
  edges.push_back({Eigen::Vector3d(min_bound(0), max_bound(1), min_bound(2)), 
                   Eigen::Vector3d(min_bound(0), min_bound(1), min_bound(2))});
  
  // 顶面的4条边
  edges.push_back({Eigen::Vector3d(min_bound(0), min_bound(1), max_bound(2)), 
                   Eigen::Vector3d(max_bound(0), min_bound(1), max_bound(2))});
  edges.push_back({Eigen::Vector3d(max_bound(0), min_bound(1), max_bound(2)), 
                   Eigen::Vector3d(max_bound(0), max_bound(1), max_bound(2))});
  edges.push_back({Eigen::Vector3d(max_bound(0), max_bound(1), max_bound(2)), 
                   Eigen::Vector3d(min_bound(0), max_bound(1), max_bound(2))});
  edges.push_back({Eigen::Vector3d(min_bound(0), max_bound(1), max_bound(2)), 
                   Eigen::Vector3d(min_bound(0), min_bound(1), max_bound(2))});
  
  // 4条垂直边
  edges.push_back({Eigen::Vector3d(min_bound(0), min_bound(1), min_bound(2)), 
                   Eigen::Vector3d(min_bound(0), min_bound(1), max_bound(2))});
  edges.push_back({Eigen::Vector3d(max_bound(0), min_bound(1), min_bound(2)), 
                   Eigen::Vector3d(max_bound(0), min_bound(1), max_bound(2))});
  edges.push_back({Eigen::Vector3d(max_bound(0), max_bound(1), min_bound(2)), 
                   Eigen::Vector3d(max_bound(0), max_bound(1), max_bound(2))});
  edges.push_back({Eigen::Vector3d(min_bound(0), max_bound(1), min_bound(2)), 
                   Eigen::Vector3d(min_bound(0), max_bound(1), max_bound(2))});
  
  // 为每条边创建一个LINE_LIST标记
  for (size_t i = 0; i < edges.size(); ++i) {
    visualization_msgs::Marker edge_marker;
    edge_marker.header.frame_id = mp_.frame_id_;
    edge_marker.header.stamp = ros::Time::now();
    edge_marker.ns = "map_boundary";
    edge_marker.id = i;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::Marker::ADD;
    
    // 设置线条的两个端点
    geometry_msgs::Point start_point, end_point;
    start_point.x = edges[i].first(0);
    start_point.y = edges[i].first(1);
    start_point.z = edges[i].first(2);
    end_point.x = edges[i].second(0);
    end_point.y = edges[i].second(1);
    end_point.z = edges[i].second(2);
    
    edge_marker.points.push_back(start_point);
    edge_marker.points.push_back(end_point);
    
    // 设置线条属性
    edge_marker.scale.x = 0.05; // 线条宽度
    edge_marker.color.r = 1.0;  // 红色
    edge_marker.color.g = 0.0;
    edge_marker.color.b = 0.0;
    edge_marker.color.a = 1.0;  // 不透明
    
    boundary_markers.markers.push_back(edge_marker);
  }
  
  map_boundary_pub_.publish(boundary_markers);
}

void SDFMap::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = mp_.esdf_slice_height_;

      dist = getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void SDFMap::getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                          vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad, int sign) {
  double dist;
  Eigen::Vector3d gd;
  for (double x = range(0); x <= range(1); x += res)
    for (double y = range(2); y <= range(3); y += res) {

      dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
      slice.push_back(Eigen::Vector3d(x, y, dist));
      grad.push_back(gd);
    }
}

void SDFMap::checkDist() {
  for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        Eigen::Vector3d grad;
        double dist = getDistWithGradTrilinear(pos, grad);

        if (fabs(dist) > 10.0) {
        }
      }
}

bool SDFMap::odomValid() { return md_.has_odom_; }

bool SDFMap::hasDepthObservation() { return md_.has_first_depth_; }

std::shared_ptr<const plan_env::CloudOccupancySnapshot>
SDFMap::cloudOccupancySnapshot() const {
  const std::shared_ptr<plan_env::CloudOccupancySnapshotStore> snapshot_store =
      cloud_occupancy_snapshot_store_;
  if (!snapshot_store) return std::shared_ptr<const plan_env::CloudOccupancySnapshot>();
  std::lock_guard<std::mutex> lock(snapshot_store->mutex);
  return snapshot_store->latest;
}

plan_env::SDFMapAcceptedStateVisibilityV2
SDFMap::acceptedStateVisibilityV2() const {
  plan_env::SDFMapAcceptedStateVisibilityV2 visibility;
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  visibility.map_instance_id = authoritative_capture_mutex_v2_.instance_id;
  visibility.accepted_state_sequence = md_.authoritative_state_sequence_v2_;
  visibility.accepted_state_notification_sequence =
      md_.authoritative_state_notification_sequence_v2_;
  visibility.accepted_time_ticks = md_.authoritative_state_time_ticks_v2_;
  visibility.configuration_generation =
      md_.authoritative_configuration_generation_v2_;
  visibility.configuration_key = captureConfigurationKeyV2(
      mp_, md_.manual_boundary_enabled_, md_.manual_boundary_min_,
      md_.manual_boundary_max_);
  visibility.frame_id = mp_.frame_id_;
  visibility.accepted_state_stamp = md_.authoritative_state_stamp_v2_;
  visibility.support.valid = md_.authoritative_support_valid_v2_;
  visibility.support.complete = md_.authoritative_support_complete_v2_;
  visibility.support.evidence_basis =
      md_.authoritative_support_evidence_basis_v2_;
  visibility.support.evidence_sequence =
      md_.authoritative_support_sequence_v2_;
  visibility.support.evidence_accepted_ticks =
      md_.authoritative_support_accepted_ticks_v2_;
  visibility.support.map_instance_id =
      md_.authoritative_support_map_instance_id_v2_;
  visibility.support.configuration_generation =
      md_.authoritative_support_configuration_generation_v2_;
  visibility.support.configuration_key =
      md_.authoritative_support_configuration_key_v2_;
  visibility.support.frame_id = md_.authoritative_support_frame_id_v2_;
  visibility.support.evidence_stamp = md_.authoritative_support_stamp_v2_;
  visibility.support.valid_until = md_.authoritative_support_valid_until_v2_;
  visibility.support.valid_until_accepted_ticks =
      md_.authoritative_support_valid_until_ticks_v2_;
  visibility.support.support_min = md_.authoritative_support_min_v2_;
  visibility.support.support_max = md_.authoritative_support_max_v2_;
  visibility.support.required_halo = md_.authoritative_support_halo_v2_;
  visibility.support.halo_reconciled =
      md_.authoritative_support_halo_reconciled_v2_;
  visibility.valid = visibility.map_instance_id != 0U &&
      visibility.accepted_state_sequence != 0U &&
      visibility.accepted_state_notification_sequence >=
          visibility.accepted_state_sequence &&
      visibility.accepted_time_ticks != 0U &&
      visibility.configuration_generation != 0U &&
      visibility.configuration_key != 0U && !visibility.frame_id.empty();
  return visibility;
}

std::shared_ptr<const plan_env::SDFMapCaptureV2>
SDFMap::captureAuthoritativeSDFMapV2() const {
  return captureAuthoritativeSDFMapV2(plan_env::SDFMapCaptureRegionV2());
}

std::shared_ptr<const plan_env::SDFMapCaptureV2>
SDFMap::captureAuthoritativeSDFMapV2(
    const plan_env::SDFMapCaptureRegionV2& requested_region) const {
  if (!plan_env::sdfMapCaptureV2FloatingPointEnvironmentSupported()) {
    return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
  }
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  // The default-constructed legacy MappingParameters contains no valid map
  // geometry.  Check the V2 accepted-state gate first so an uninitialized
  // legacy buffer is never read merely to answer “unavailable”.
  if (md_.authoritative_state_sequence_v2_ == 0U ||
      md_.authoritative_state_identity_exhausted_v2_ ||
      md_.authoritative_configuration_generation_v2_ == 0U ||
      md_.authoritative_configuration_identity_exhausted_v2_ ||
      authoritative_capture_mutex_v2_.instance_id == 0U ||
      !validCaptureMapGeometry(mp_)) {
    return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
  }

  const Eigen::Vector3i map_count = mp_.map_voxel_num_;
  std::size_t expected_size = 0U;
  if (!validMapVoxelCount(map_count, expected_size)) {
    return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
  }
  if (md_.occupancy_buffer_inflate_.size() != expected_size) {
    return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
  }
  Eigen::Vector3d effective_map_min;
  Eigen::Vector3d effective_map_max;
  if (!effectiveCaptureDomainV2(mp_, md_, effective_map_min,
                                effective_map_max)) {
    return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
  }
  const std::uint64_t current_configuration_key =
      captureConfigurationKeyV2(
          mp_, md_.manual_boundary_enabled_, md_.manual_boundary_min_,
          md_.manual_boundary_max_);
  if (md_.authoritative_support_valid_v2_ &&
      md_.authoritative_support_complete_v2_ &&
      (md_.authoritative_support_map_instance_id_v2_ !=
           authoritative_capture_mutex_v2_.instance_id ||
       md_.authoritative_support_configuration_generation_v2_ !=
           md_.authoritative_configuration_generation_v2_ ||
       md_.authoritative_support_configuration_key_v2_ !=
           current_configuration_key ||
       md_.authoritative_support_frame_id_v2_ != mp_.frame_id_)) {
    // A const capture cannot rewrite map buffers, but it must latch the
    // provenance mismatch so restoring an old frame/configuration cannot
    // resurrect stale support.  Only fresh complete evidence clears it.
    authoritative_capture_mutex_v2_.support_binding_invalidated = true;
  }

  Eigen::Vector3d requested_min = effective_map_min;
  Eigen::Vector3d requested_max = effective_map_max;
  if (requested_region.valid) {
    if (!finiteVector(requested_region.min) ||
        !finiteVector(requested_region.max) ||
        !std::isfinite(requested_region.halo) ||
        requested_region.halo < 0.0 ||
        (requested_region.max.array() < requested_region.min.array()).any()) {
      return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
    }
    // Expand and clip in directed intervals.  The source range must enclose
    // every closed voxel volume that could intersect the mathematical region;
    // composed long-double expressions followed by an implicit double
    // conversion are not a sound enclosure at ULP-scale boundaries.
    for (int axis = 0; axis < 3; ++axis) {
      DirectedBoundsV2 region_min = scalarBoundsV2(requested_region.min(axis));
      DirectedBoundsV2 region_max = scalarBoundsV2(requested_region.max(axis));
      const DirectedBoundsV2 halo = scalarBoundsV2(requested_region.halo);
      DirectedBoundsV2 expanded_min;
      DirectedBoundsV2 expanded_max;
      if (!directedSubV2(region_min, halo, expanded_min) ||
          !directedAddV2(region_max, halo, expanded_max)) {
        return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
      }

      expanded_min.lower = std::max(expanded_min.lower,
                                    effective_map_min(axis));
      expanded_min.upper = std::max(expanded_min.upper,
                                    effective_map_min(axis));
      expanded_max.lower = std::min(expanded_max.lower,
                                    effective_map_max(axis));
      expanded_max.upper = std::min(expanded_max.upper,
                                    effective_map_max(axis));
      if (!finiteBoundsV2(expanded_min) || !finiteBoundsV2(expanded_max) ||
          expanded_min.lower > expanded_max.upper) {
        return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
      }
      // Keep the directed clipped request as metadata as well as using it to
      // select source indices.  It remains in native map coordinates.
      requested_min(axis) = expanded_min.lower;
      requested_max(axis) = expanded_max.upper;
    }
  }
  if ((requested_max.array() < requested_min.array()).any()) {
    return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
  }

  Eigen::Vector3i source_min = Eigen::Vector3i::Zero();
  Eigen::Vector3i source_max = map_count - Eigen::Vector3i::Ones();
  if (requested_region.valid) {
    for (int axis = 0; axis < 3; ++axis) {
      DirectedBoundsV2 lower_offset;
      DirectedBoundsV2 upper_offset;
      DirectedBoundsV2 lower_coordinate;
      DirectedBoundsV2 upper_coordinate;
      if (!directedSubV2(scalarBoundsV2(requested_min(axis)),
                         scalarBoundsV2(mp_.map_origin_(axis)),
                         lower_offset) ||
          !directedSubV2(scalarBoundsV2(requested_max(axis)),
                         scalarBoundsV2(mp_.map_origin_(axis)),
                         upper_offset) ||
          !directedDivV2(lower_offset, mp_.resolution_, lower_coordinate) ||
          !directedDivV2(upper_offset, mp_.resolution_, upper_coordinate)) {
        return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
      }
      int first = 0;
      int last = -1;
      if (!captureIndexRangeForRegion(lower_coordinate, upper_coordinate,
                                      map_count(axis), first, last)) {
        return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
      }
      source_min(axis) = first;
      source_max(axis) = last;
    }
  }

  const Eigen::Vector3i capture_count = source_max - source_min +
      Eigen::Vector3i::Ones();
  std::size_t capture_size = 0U;
  if (!validMapVoxelCount(capture_count, capture_size)) {
    return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
  }
  if (capture_size == 0U || capture_size > expected_size) {
    return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
  }

  const std::shared_ptr<plan_env::SDFMapCaptureV2> capture(
      new plan_env::SDFMapCaptureV2());
  capture->valid = true;
  capture->map_instance_id = authoritative_capture_mutex_v2_.instance_id;
  if (capture->map_instance_id == 0U) {
    return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
  }
  capture->configuration_generation =
      md_.authoritative_configuration_generation_v2_ == 0U
      ? 1U : md_.authoritative_configuration_generation_v2_;
  capture->configuration_key = captureConfigurationKeyV2(
      mp_, md_.manual_boundary_enabled_, md_.manual_boundary_min_,
      md_.manual_boundary_max_);
  capture->frame_id = mp_.frame_id_;
  capture->accepted_state_sequence = md_.authoritative_state_sequence_v2_;
  capture->accepted_state_notification_sequence =
      md_.authoritative_state_notification_sequence_v2_;
  capture->accepted_time_ticks = md_.authoritative_state_time_ticks_v2_;
  capture->observation_stamp = md_.authoritative_state_stamp_v2_;
  capture->accepted_state_stamp = md_.authoritative_state_stamp_v2_;
  // map_min/map_max are the effective planner-admissible domain.  The native
  // grid origin/count remain the full SDFMap geometry; manual boundaries are
  // therefore represented as a conservative domain restriction, not by
  // rebasing or rewriting voxel indices.
  capture->map_min = effective_map_min;
  capture->map_max = effective_map_max;
  capture->source_min_index = source_min;
  capture->source_max_index = source_max;
  capture->voxel_count = capture_count;
  capture->resolution = mp_.resolution_;
  // Retain the native SDFMap grid origin.  A cropped capture is identified by
  // its integer source offsets, never by rebasing the origin through rounded
  // floating-point multiplication.
  capture->grid_origin = mp_.map_origin_;
  capture->capture_min = requested_min;
  capture->capture_max = requested_max;
  capture->included_map_inflation = mp_.obstacles_inflation_;
  // A present layer with a mismatched backing cannot be safely treated as
  // absent: its effective planner contribution is unknown at this boundary.
  if ((!md_.manual_occupancy_buffer_.empty() &&
       md_.manual_occupancy_buffer_.size() != expected_size) ||
      (!md_.static_preinflated_buffer_.empty() &&
       md_.static_preinflated_buffer_.size() != expected_size)) {
    return std::shared_ptr<const plan_env::SDFMapCaptureV2>();
  }
  const bool manual_layer_present =
      md_.manual_occupancy_buffer_.size() == expected_size &&
      anyNonzero(md_.manual_occupancy_buffer_);
  const bool static_layer_present =
      mp_.static_preinflated_map_enable_ &&
      md_.static_preinflated_map_ready_ &&
      md_.static_preinflated_buffer_.size() == expected_size;
  int ceiling_index = -1;
  const bool ceiling_configured =
      std::isfinite(mp_.virtual_ceil_height_) &&
      mp_.virtual_ceil_height_ > -0.5;
  if (ceiling_configured) {
    const double ceiling_coordinate =
        (mp_.virtual_ceil_height_ - mp_.map_origin_(2)) /
        mp_.resolution_;
    if (std::isfinite(ceiling_coordinate) &&
        ceiling_coordinate >= static_cast<double>(INT_MIN) &&
        ceiling_coordinate <= static_cast<double>(INT_MAX)) {
      const double floored = std::floor(ceiling_coordinate);
      if (floored >= 0.0 &&
          floored < static_cast<double>(map_count.z())) {
        ceiling_index = static_cast<int>(floored);
      }
    }
  }
  if (md_.has_cloud_ || md_.has_first_depth_) {
    capture->effective_layer_mask |= plan_env::kSDFMapCaptureLayerSensor;
  }
  if (manual_layer_present) {
    capture->effective_layer_mask |= plan_env::kSDFMapCaptureLayerManual;
  }
  if (static_layer_present) {
    capture->effective_layer_mask |= plan_env::kSDFMapCaptureLayerStatic;
  }
  capture->occupied.assign(capture_size, 0U);
  if (md_.authoritative_support_buffer_v2_.size() == expected_size) {
    capture->support.mask.assign(capture_size, 0U);
  }

  bool ceiling_backing_present = false;
  for (int x = source_min.x(); x <= source_max.x(); ++x)
    for (int y = source_min.y(); y <= source_max.y(); ++y)
      for (int z = source_min.z(); z <= source_max.z(); ++z) {
        const Eigen::Vector3i source_index(x, y, z);
        const Eigen::Vector3i capture_index = source_index - source_min;
        const std::size_t source_address = mapAddressChecked(
            source_index, map_count);
        const std::size_t capture_address = mapAddressChecked(
            capture_index, capture_count);
        bool occupied = md_.occupancy_buffer_inflate_[source_address] != 0;
        // `getInflateOccupancy` treats the manual overlay as authoritative
        // even if a caller has not yet rerun applyManualLayer().  Preserve
        // that effective planner semantics at the serialization boundary.
        if (manual_layer_present &&
            md_.manual_occupancy_buffer_[source_address] != 0) {
          occupied = true;
        }
        // A ready static/preinflated layer is likewise effective even if its
        // bits have not yet been re-applied to the mutable inflate buffer.
        if (static_layer_present &&
            md_.static_preinflated_buffer_[source_address] != 0) {
          occupied = true;
        }
        // The ceiling layer is effective only when its native planner backing
        // actually contains the configured ceiling plane.  A cloud-only map
        // with the parameter set but no clear/inflate application must not
        // synthesize a phantom occupied plane in this read-only capture.
        if (ceiling_index >= 0 && source_index.z() == ceiling_index &&
            md_.occupancy_buffer_inflate_[source_address] != 0) {
          ceiling_backing_present = true;
        }
        capture->occupied[capture_address] = occupied ? 1U : 0U;
        if (!capture->support.mask.empty()) {
          capture->support.mask[capture_address] =
              md_.authoritative_support_buffer_v2_[source_address] != 0 ? 1U : 0U;
        }
      }

  if (ceiling_backing_present) {
    capture->effective_layer_mask |= plan_env::kSDFMapCaptureLayerCeiling;
  }

  const bool support_binding_matches =
      md_.authoritative_support_map_instance_id_v2_ ==
          capture->map_instance_id &&
      md_.authoritative_support_configuration_generation_v2_ ==
          capture->configuration_generation &&
      md_.authoritative_support_configuration_key_v2_ ==
          capture->configuration_key &&
      md_.authoritative_support_frame_id_v2_ == capture->frame_id;
  const bool support_expiry_grounded =
      md_.authoritative_support_valid_until_v2_.isZero()
      ? md_.authoritative_support_valid_until_ticks_v2_ == 0U
      : md_.authoritative_support_valid_until_ticks_v2_ != 0U;
  const bool support_not_expired =
      support_expiry_grounded &&
      (md_.authoritative_support_valid_until_ticks_v2_ == 0U ||
       steadyTicksV2() <= md_.authoritative_support_valid_until_ticks_v2_);
  Eigen::Vector3d support_min = md_.authoritative_support_min_v2_;
  Eigen::Vector3d support_max = md_.authoritative_support_max_v2_;
  bool support_domain_valid = finiteVector(support_min) &&
      finiteVector(support_max) &&
      (support_max.array() >= support_min.array()).all();
  if (support_domain_valid && md_.manual_boundary_enabled_) {
    support_min = support_min.cwiseMax(capture->map_min);
    support_max = support_max.cwiseMin(capture->map_max);
    support_domain_valid =
        (support_max.array() >= support_min.array()).all();
  }
  if (support_domain_valid) {
    support_domain_valid =
        (support_min.array() >= capture->map_min.array()).all() &&
        (support_max.array() <= capture->map_max.array()).all();
  }
  capture->support.valid = md_.authoritative_support_valid_v2_ &&
      md_.authoritative_support_complete_v2_ &&
      support_binding_matches && support_not_expired &&
      !authoritative_capture_mutex_v2_.support_binding_invalidated &&
      support_domain_valid &&
      !capture->support.mask.empty();
  capture->support.complete = capture->support.valid;
  capture->support.evidence_basis =
      md_.authoritative_support_evidence_basis_v2_;
  capture->support.evidence_sequence =
      md_.authoritative_support_sequence_v2_;
  capture->support.evidence_accepted_ticks =
      md_.authoritative_support_accepted_ticks_v2_;
  capture->support.map_instance_id =
      md_.authoritative_support_map_instance_id_v2_;
  capture->support.configuration_generation =
      md_.authoritative_support_configuration_generation_v2_;
  capture->support.configuration_key =
      md_.authoritative_support_configuration_key_v2_;
  capture->support.frame_id = md_.authoritative_support_frame_id_v2_;
  capture->support.evidence_stamp = md_.authoritative_support_stamp_v2_;
  capture->support.valid_until = md_.authoritative_support_valid_until_v2_;
  capture->support.valid_until_accepted_ticks =
      md_.authoritative_support_valid_until_ticks_v2_;
  capture->support.support_min = support_domain_valid
      ? support_min : Eigen::Vector3d::Zero();
  capture->support.support_max = support_domain_valid
      ? support_max : Eigen::Vector3d::Zero();
  capture->support.required_halo = md_.authoritative_support_halo_v2_;
  capture->support.halo_reconciled =
      md_.authoritative_support_halo_reconciled_v2_;
  return std::shared_ptr<const plan_env::SDFMapCaptureV2>(capture);
}

double SDFMap::getResolution() { return mp_.resolution_; }

Eigen::Vector3d SDFMap::getOrigin() { return mp_.map_origin_; }

int SDFMap::getVoxelNum() {
  return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void SDFMap::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2],
                            Eigen::Vector3d& diff) {
  if (!isInMap(pos)) {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y][z] = current_pos;
      }
    }
  }
}

void SDFMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& img,
                               const nav_msgs::OdometryConstPtr& odom) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  /* get pose */
  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
}

void SDFMap::depthCallback(const sensor_msgs::ImageConstPtr& img) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  std::cout << "depth: " << img->header.stamp << std::endl;
}

void SDFMap::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
  const std::lock_guard<std::recursive_mutex> lock(
      authoritative_capture_mutex_v2_.mutex);
  std::cout << "pose: " << pose->header.stamp << std::endl;

  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
}

// SDFMap
