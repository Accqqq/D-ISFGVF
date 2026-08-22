#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <memory>

#define private public
#include <plan_env/sdf_map.h>
#undef private

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/path_tube_builder.h>
#include <bspline_race/phase_offset_geometry.h>

namespace
{

using FLAG_Race::C2ConnectorCheckResult;
using FLAG_Race::checkC2ConnectorSamples;
using FLAG_Race::ContinuousPhasePath;
using FLAG_Race::ContinuousPhasePathState;
using FLAG_Race::PathTubeBuilder;
using FLAG_Race::PhaseOffsetGeometry;
using FLAG_Race::PhaseOffsetGeometryEvaluator;
using FLAG_Race::PhaseOffsetGeometryParams;
using FLAG_Race::TubeBounds;
using FLAG_Race::TubeProfile;

ContinuousPhasePathState makeState(const Eigen::Vector3d& p,
                                   const Eigen::Vector3d& dp,
                                   const Eigen::Vector3d& d2p)
{
  ContinuousPhasePathState s;
  s.p = p;
  s.dp_dw = dp;
  s.d2p_dw2 = d2p;
  s.valid = true;
  return s;
}

// A straight old path from x=0 to x=8 at y=0 and a candidate path that
// begins identically but then shifts to y=1 (laterally offset replan).
std::shared_ptr<ContinuousPhasePath> makeOldPath()
{
  auto path = std::make_shared<ContinuousPhasePath>();
  const auto line = [](double w, ContinuousPhasePathState& s) {
    s.p = Eigen::Vector3d(w, 0.0, 1.0);
    s.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
    s.d2p_dw2 = Eigen::Vector3d::Zero();
    s.valid = true;
    return true;
  };
  EXPECT_TRUE(path->appendSegment(0.0, 8.0, "line", line));
  return path;
}

TEST(PhaseOffsetContinuation, C2ConnectorSeamIsContinuous)
{
  const auto old_path = makeOldPath();
  ContinuousPhasePathState old_state;
  ASSERT_TRUE(old_path->evaluate(3.0, old_state, false));

  // New path state at the switch: same position/derivatives (C2 matched),
  // then a lateral shift at the far end.
  const double w_switch = 3.0;
  const double w_join = 4.0;
  ContinuousPhasePathState new_state = makeState(
    Eigen::Vector3d(4.0, 0.5, 1.0),
    Eigen::Vector3d(1.0, 0.5, 0.0),
    Eigen::Vector3d::Zero());

  const auto connector = ContinuousPhasePath::makeQuinticHermite(
    w_switch, w_join, old_state, new_state);
  ASSERT_TRUE(connector);

  auto trial = std::make_shared<ContinuousPhasePath>();
  ASSERT_TRUE(trial->appendSlice(*old_path, 0.0, w_switch));
  ASSERT_TRUE(trial->appendSegment(w_switch, w_join, "c2", connector));

  // Evaluate on both sides of the seam and compare p / p_w / p_ww.
  ContinuousPhasePathState left, right;
  ASSERT_TRUE(old_path->evaluate(w_switch, left, false));
  ASSERT_TRUE(trial->evaluate(w_switch, right, false));
  EXPECT_NEAR((left.p - right.p).norm(), 0.0, 1e-9);
  EXPECT_NEAR((left.dp_dw - right.dp_dw).norm(), 0.0, 1e-9);
  EXPECT_NEAR((left.d2p_dw2 - right.d2p_dw2).norm(), 0.0, 1e-9);

  // With the SAME delta kept, r and r_w are continuous across the seam.
  const double delta = 0.30;
  PhaseOffsetGeometryParams params;
  PhaseOffsetGeometryEvaluator eval;
  const Eigen::Vector3d pos = left.p;
  PhaseOffsetGeometry g_left, g_right;
  ASSERT_TRUE(eval.evaluate(left, pos, delta, params, g_left));
  ASSERT_TRUE(eval.evaluate(right, pos, delta, params, g_right));
  EXPECT_NEAR((g_left.r - g_right.r).norm(), 0.0, 1e-9);
  EXPECT_NEAR((g_left.r_w - g_right.r_w).norm(), 0.0, 1e-9);
  EXPECT_NEAR((g_left.e_perp - g_right.e_perp).norm(), 0.0, 1e-9);
  EXPECT_NEAR((g_left.base_v - g_right.base_v).norm(), 0.0, 1e-9);
}

TEST(PhaseOffsetContinuation, ConnectorCheckRejectsBadRegularity)
{
  // Sharp turn: kappa large, delta large -> 1-kappa*delta < mu.
  std::vector<ContinuousPhasePathState> samples;
  samples.push_back(makeState(
    Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 40.0, 0.0)));  // kappa = 40
  C2ConnectorCheckResult out;
  EXPECT_FALSE(checkC2ConnectorSamples(
    samples, 0.30, 0.20, std::function<double(const Eigen::Vector3d&)>(),
    0.5, out));
  EXPECT_EQ(out.reason, "regularity_below_mu");

  // Same sample with delta=0 is fine.
  EXPECT_TRUE(checkC2ConnectorSamples(
    samples, 0.0, 0.20, std::function<double(const Eigen::Vector3d&)>(),
    0.5, out));
}

TEST(PhaseOffsetContinuation, ConnectorCheckRejectsActiveReferenceCollision)
{
  std::vector<ContinuousPhasePathState> samples;
  samples.push_back(makeState(
    Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d::Zero()));
  C2ConnectorCheckResult out;
  // r = p + N*delta = (0, 0.3, 1); obstacle distance 0.2 < 0.5 clearance.
  const auto dist_fn = [](const Eigen::Vector3d& q) {
    return 0.5 - std::abs(q.y());  // wall at y = +-0.5
  };
  EXPECT_FALSE(checkC2ConnectorSamples(
    samples, 0.30, 0.20, dist_fn, 0.5, out));
  EXPECT_EQ(out.reason, "active_reference_too_close");

  EXPECT_TRUE(checkC2ConnectorSamples(
    samples, 0.0, 0.20, dist_fn, 0.5, out));
  EXPECT_GE(out.min_obstacle_distance, 0.5 - 1e-9);
}

// Reuse the SDFMap init from path_tube_builder_test.
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
  map.md_.occupancy_buffer_ = std::vector<double>(n, 0.0);
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

TEST(PhaseOffsetContinuation, TubeNotContainingDeltaRejectsInstall)
{
  SDFMap map;
  initMap(map, 0.35);  // corridor: max lateral offset 0.35 - clearance
  PathTubeBuilder builder;
  builder.setMap(std::make_shared<SDFMap>(map));
  builder.params.max_offset = 1.2;
  builder.params.reference_clearance = 0.25;
  builder.params.open_width = 1.6;

  auto path = std::make_shared<ContinuousPhasePath>();
  const auto line = [](double w, ContinuousPhasePathState& s) {
    s.p = Eigen::Vector3d(w, 0.0, 1.0);
    s.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
    s.d2p_dw2 = Eigen::Vector3d::Zero();
    s.valid = true;
    return true;
  };
  ASSERT_TRUE(path->appendSegment(0.0, 8.0, "line", line));

  TubeProfile profile;
  ASSERT_TRUE(builder.rebuild(*path, 1.0, 0.0, profile));
  TubeBounds bounds;
  ASSERT_TRUE(builder.queryConservativeLookahead(profile, 1.0, 3.0, bounds));

  // delta=0 fits the narrow tube -> install would be accepted.
  EXPECT_GE(bounds.upper, 0.0 - 1e-9);
  EXPECT_LE(bounds.lower, 0.0 + 1e-9);
  // delta=0.5 cannot fit -> install must be rejected.
  const double delta = 0.5;
  const bool fits = delta <= bounds.upper + 1e-9 &&
                    delta >= bounds.lower - 1e-9;
  EXPECT_FALSE(fits);
}

}  // namespace

int
main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
