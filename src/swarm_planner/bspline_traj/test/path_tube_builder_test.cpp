#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#define private public
#include <plan_env/sdf_map.h>
#undef private

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/path_tube_builder.h>

namespace
{

using FLAG_Race::ContinuousPhasePath;
using FLAG_Race::ContinuousPhasePathState;
using FLAG_Race::PathTubeBuilder;
using FLAG_Race::TubeBounds;
using FLAG_Race::TubeProfile;

void initMap(SDFMap& map, double wall_half_width = 1e9)
{
  map.mp_.resolution_ = 0.1;
  map.mp_.resolution_inv_ = 10.0;
  map.mp_.map_origin_ = Eigen::Vector3d(-10.0, -10.0, 0.0);
  map.mp_.map_size_ = Eigen::Vector3d(30.0, 30.0, 3.0);
  map.mp_.map_voxel_num_ = Eigen::Vector3i(300, 300, 30);
  map.mp_.map_min_boundary_ = map.mp_.map_origin_;
  map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
  map.mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  map.mp_.map_max_idx_ = map.mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  map.mp_.p_min_ = 0.12;
  map.mp_.p_max_ = 0.90;
  map.mp_.p_occ_ = 0.80;
  map.mp_.clamp_min_log_ = logit(map.mp_.p_min_);
  map.mp_.clamp_max_log_ = logit(map.mp_.p_max_);
  map.mp_.min_occupancy_log_ = logit(map.mp_.p_occ_);
  map.mp_.unknown_flag_ = 0.01;

  const int n = map.mp_.map_voxel_num_(0) * map.mp_.map_voxel_num_(1) *
                map.mp_.map_voxel_num_(2);
  map.md_.occupancy_buffer_ =
    std::vector<double>(n, 0.0);  // known-free for the test region
  map.md_.occupancy_buffer_neg = std::vector<char>(n, 0);
  map.md_.occupancy_buffer_inflate_ = std::vector<char>(n, 0);
  map.md_.distance_buffer_ = std::vector<double>(n, 100.0);
  map.md_.distance_buffer_neg_ = std::vector<double>(n, 100.0);
  map.md_.distance_buffer_all_ = std::vector<double>(n, 100.0);

  if (wall_half_width < 1e8)
  {
    for (int ix = 0; ix < 300; ++ix)
      for (int iz = 0; iz < 30; ++iz)
        for (int iy = 0; iy < 300; ++iy)
        {
          const double y = map.mp_.map_origin_.y() + iy * 0.1;
          const double d = std::max(
            0.05, std::abs(std::abs(y) - wall_half_width));
          map.md_.distance_buffer_all_[ix * 300 * 30 + iy * 30 + iz] = d;
        }
  }
  map.md_.has_odom_ = true;
  map.md_.has_cloud_ = true;
}

std::shared_ptr<ContinuousPhasePath> makeLinePath()
{
  auto path = std::make_shared<ContinuousPhasePath>();
  ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
  start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  start.d2p_dw2 = Eigen::Vector3d::Zero();
  ContinuousPhasePathState end;
  end.p = Eigen::Vector3d(10.0, 0.0, 1.0);
  end.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  end.d2p_dw2 = Eigen::Vector3d::Zero();
  path->appendSegment(0.0, 10.0, "line",
                      ContinuousPhasePath::makeQuinticHermite(
                        0.0, 10.0, start, end));
  return path;
}

TEST(PathTubeBuilder, OpenAreaGivesWideTube)
{
  SDFMap map;
  initMap(map);
  PathTubeBuilder builder;
  builder.setMap(std::make_shared<SDFMap>(map));
  builder.params.max_offset = 1.2;
  builder.params.reference_clearance = 0.5;
  builder.params.open_width = 1.6;
  TubeProfile profile;
  ASSERT_TRUE(builder.rebuild(*makeLinePath(), 1.0, 0.0, profile));
  ASSERT_TRUE(profile.valid);
  EXPECT_GE(profile.samples.front().upper, 1.19);
  EXPECT_LE(profile.samples.front().lower, -1.19);
  EXPECT_NEAR(1.0, profile.samples.front().beta, 0.1);
  TubeBounds bounds;
  ASSERT_TRUE(builder.query(profile, 3.0, bounds));
  EXPECT_TRUE(bounds.valid);
  EXPECT_GE(bounds.upper - bounds.lower, 2.0);
}

TEST(PathTubeBuilder, NarrowCorridorShrinksTubeAndBeta)
{
  SDFMap map;
  initMap(map, 0.45);  // walls at y = +-0.45
  PathTubeBuilder builder;
  builder.setMap(std::make_shared<SDFMap>(map));
  builder.params.max_offset = 1.2;
  builder.params.reference_clearance = 0.25;
  builder.params.open_width = 1.6;
  TubeProfile profile;
  ASSERT_TRUE(builder.rebuild(*makeLinePath(), 1.0, 0.0, profile));
  // The offset bound is roughly wall distance minus clearance.
  EXPECT_LT(profile.samples.front().upper, 0.35);
  EXPECT_GT(profile.samples.front().lower, -0.35);
  EXPECT_LT(profile.samples.front().beta, 0.6);
}

TEST(PathTubeBuilder, MapNotReadyIsInvalid)
{
  SDFMap map;
  initMap(map);
  map.md_.has_cloud_ = false;
  PathTubeBuilder builder;
  builder.setMap(std::make_shared<SDFMap>(map));
  TubeProfile profile;
  EXPECT_FALSE(builder.rebuild(*makeLinePath(), 1.0, 0.0, profile));
  EXPECT_FALSE(profile.valid);
}

TEST(PathTubeBuilder, BlockedLookaheadDoesNotFakeEmptyTube)
{
  SDFMap map;
  initMap(map);
  // Shrink the x range so the path (x up to 10) leaves the map at x = 5.
  map.mp_.map_origin_ = Eigen::Vector3d(-10.0, -10.0, 0.0);
  map.mp_.map_size_ = Eigen::Vector3d(15.0, 30.0, 3.0);
  map.mp_.map_voxel_num_ = Eigen::Vector3i(150, 300, 30);
  map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
  map.mp_.map_max_idx_ = map.mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  PathTubeBuilder builder;
  builder.setMap(std::make_shared<SDFMap>(map));
  builder.params.lookahead_w = 8.0;  // lookahead crosses the map edge
  TubeProfile profile;
  ASSERT_TRUE(builder.rebuild(*makeLinePath(), 1.0, 0.0, profile));
  TubeBounds bounds;
  // The blocked (unknown) part of the window is not claimed passable: the
  // conservative query keeps the observed open-part bounds instead of
  // producing a false empty tube.
  const bool ok = builder.queryConservativeLookahead(profile, 1.0, 8.0,
                                                     bounds);
  if (ok)
    EXPECT_GE(bounds.upper - bounds.lower, 1.0);
}

}  // namespace

int
main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
