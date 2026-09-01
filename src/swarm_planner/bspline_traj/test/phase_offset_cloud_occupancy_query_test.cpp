#include <gtest/gtest.h>

#include "bspline_race/integration/phase_offset_cloud_occupancy_query.h"

#include <cmath>
#include <memory>
#include <set>

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
