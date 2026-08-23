#include <gtest/gtest.h>

#include <bspline_race/continuous_phase_normal_frame.h>

namespace {

TEST(ContinuousPhaseNormalFrameTest, DeterministicQueryAndOrthonormality) {
  const auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  ASSERT_TRUE(path->appendSegment(
      0.0, 4.0,
      "helix",
      FLAG_Race::ContinuousPhasePath::makePeriodicCircle(
          Eigen::Vector3d(0.0, 0.0, 1.0), 2.0, 4.0, 1.0)));
  FLAG_Race::ContinuousPhaseNormalFrame frame(path, 7U, 11U);
  phase_offset_core::NormalFrameQuery first;
  phase_offset_core::NormalFrameQuery second;
  ASSERT_TRUE(frame.query(2.0, first));
  ASSERT_TRUE(frame.query(2.0, second));
  EXPECT_EQ(first.path_revision, 7U);
  EXPECT_EQ(first.frame_revision, 11U);
  EXPECT_LT((first.T - second.T).norm(), 1e-12);
  EXPECT_LT((first.N - second.N).norm(), 1e-12);
  EXPECT_LT((first.N_w - second.N_w).norm(), 1e-12);
  EXPECT_NEAR(first.T.norm(), 1.0, 1e-10);
  EXPECT_NEAR(first.N.norm(), 1.0, 1e-10);
  EXPECT_NEAR(first.T.dot(first.N), 0.0, 1e-10);
}

TEST(ContinuousPhaseNormalFrameTest, NearVerticalTangentHasFiniteNormal) {
  const auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  FLAG_Race::ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(0.0, 0.0, 0.0);
  start.dp_dw = Eigen::Vector3d(1e-3, 0.0, 1.0);
  start.d2p_dw2 = Eigen::Vector3d::Zero();
  start.vel = start.dp_dw.normalized();
  start.valid = true;
  FLAG_Race::ContinuousPhasePathState end = start;
  end.p = Eigen::Vector3d(2e-3, 0.0, 2.0);
  ASSERT_TRUE(path->appendSegment(
      0.0, 2.0,
      "near_vertical",
      FLAG_Race::ContinuousPhasePath::makeQuinticHermite(0.0, 2.0, start, end)));
  FLAG_Race::ContinuousPhaseNormalFrame frame(path, 3U, 5U);
  phase_offset_core::NormalFrameQuery query;
  ASSERT_TRUE(frame.query(1.0, query));
  EXPECT_TRUE(query.T.allFinite());
  EXPECT_TRUE(query.N.allFinite());
  EXPECT_TRUE(query.N_w.allFinite());
  EXPECT_NEAR(query.T.norm(), 1.0, 1e-8);
  EXPECT_NEAR(query.N.norm(), 1.0, 1e-8);
  EXPECT_NEAR(query.T.dot(query.N), 0.0, 1e-8);
  phase_offset_core::NormalFrameCellProof proof;
  ASSERT_TRUE(frame.certifyCell(0.25, 1.75, proof));
  EXPECT_TRUE(phase_offset_core::normalFrameCellProofIsComplete(proof));
  EXPECT_GT(proof.inf_path_speed, 0.0);
  EXPECT_GE(proof.sup_path_speed, proof.inf_path_speed);
}

TEST(ContinuousPhaseNormalFrameTest, QueryOrderAndFiniteDifferenceDerivative) {
  const auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  ASSERT_TRUE(path->appendSegment(
      0.0, 4.0, "circle",
      FLAG_Race::ContinuousPhasePath::makePeriodicCircle(
          Eigen::Vector3d::Zero(), 2.0, 4.0, 1.0)));
  FLAG_Race::ContinuousPhaseNormalFrame frame(path, 19U, 23U);
  phase_offset_core::NormalFrameQuery a;
  phase_offset_core::NormalFrameQuery b;
  phase_offset_core::NormalFrameQuery c;
  ASSERT_TRUE(frame.query(0.05, a));
  ASSERT_TRUE(frame.query(2.35, b));
  ASSERT_TRUE(frame.query(0.05, c));
  EXPECT_LT((a.N - c.N).norm(), 1e-12);
  EXPECT_LT((a.N_w - c.N_w).norm(), 1e-12);
  const double w = 1.25;
  const double h = 1e-5;
  phase_offset_core::NormalFrameQuery left;
  phase_offset_core::NormalFrameQuery right;
  phase_offset_core::NormalFrameQuery center;
  ASSERT_TRUE(frame.query(w - h, left));
  ASSERT_TRUE(frame.query(w + h, right));
  ASSERT_TRUE(frame.query(w, center));
  const Eigen::Vector3d finite_difference = (right.N - left.N) / (2.0 * h);
  EXPECT_LT((finite_difference - center.N_w).norm(), 2e-3);
  phase_offset_core::NormalFrameQuery previous;
  ASSERT_TRUE(frame.query(0.05, previous));
  for (int sample_index = 1; sample_index <= 77; ++sample_index) {
    const double sample_w = 0.05 + 0.05 * sample_index;
    phase_offset_core::NormalFrameQuery query;
    ASSERT_TRUE(frame.query(sample_w, query));
    EXPECT_GT(query.N.dot(previous.N), 0.0);
    previous = query;
  }
  for (double boundary : {0.05, 0.10, 0.15, 1.0, 2.0, 3.0}) {
    phase_offset_core::NormalFrameQuery left_boundary;
    phase_offset_core::NormalFrameQuery right_boundary;
    phase_offset_core::NormalFrameQuery at_boundary;
    ASSERT_TRUE(frame.query(boundary - 1e-5, left_boundary));
    ASSERT_TRUE(frame.query(boundary, at_boundary));
    ASSERT_TRUE(frame.query(boundary + 1e-5, right_boundary));
    EXPECT_GT(at_boundary.N.dot(left_boundary.N), 0.0);
    EXPECT_GT(right_boundary.N.dot(at_boundary.N), 0.0);
    const Eigen::Vector3d boundary_derivative =
        (right_boundary.N - left_boundary.N) / (2.0e-5);
    EXPECT_LT((boundary_derivative - at_boundary.N_w).norm(), 5e-2);
  }
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
