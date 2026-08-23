#include <gtest/gtest.h>

#include <bspline_race/integration/phase_offset_executed_reference_query.h>

TEST(PhaseOffsetExecutedReferenceQueryTest, BindsPathAndFrameRevisions) {
  const auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  ASSERT_TRUE(path->appendSegment(
      0.0, 2.0, "circle",
      FLAG_Race::ContinuousPhasePath::makePeriodicCircle(
          Eigen::Vector3d::Zero(), 2.0, 2.0, 1.0)));
  FLAG_Race::PhaseOffsetExecutedReferenceQuery query(path, 0.2, 13U, 17U,
                                                     19U, 23U);
  phase_offset_navigation::ExecutedReferenceQueryResult result;
  ASSERT_TRUE(query.query(1.0, result));
  EXPECT_EQ(result.path_revision, 13U);
  EXPECT_EQ(result.frame_revision, 17U);
  EXPECT_EQ(result.owner_revision, 19U);
  EXPECT_EQ(result.query_revision, 23U);
  EXPECT_TRUE(result.r.allFinite());
  EXPECT_TRUE(result.r_w.allFinite());
  EXPECT_FALSE(result.r_ww_valid);
  double w0 = 0.0;
  double w1 = 0.0;
  ASSERT_TRUE(query.domain(w0, w1));
  EXPECT_DOUBLE_EQ(w0, 0.0);
  EXPECT_DOUBLE_EQ(w1, 2.0);
}

TEST(PhaseOffsetExecutedReferenceQueryTest, SharedFrameAndFiniteDifference) {
  const auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  ASSERT_TRUE(path->appendSegment(
      0.0, 3.0, "circle",
      FLAG_Race::ContinuousPhasePath::makePeriodicCircle(
          Eigen::Vector3d::Zero(), 1.5, 3.0, 1.0)));
  const auto frame = std::make_shared<const FLAG_Race::ContinuousPhaseNormalFrame>(
      path, 31U, 37U);
  FLAG_Race::PhaseOffsetExecutedReferenceQuery query(
      path, 0.35, frame, 31U, 37U, 41U, 43U);
  const double w = 1.2;
  const double h = 1e-5;
  phase_offset_navigation::ExecutedReferenceQueryResult left;
  phase_offset_navigation::ExecutedReferenceQueryResult right;
  phase_offset_navigation::ExecutedReferenceQueryResult center;
  ASSERT_TRUE(query.query(w - h, left));
  ASSERT_TRUE(query.query(w + h, right));
  ASSERT_TRUE(query.query(w, center));
  EXPECT_LT((((right.r - left.r) / (2.0 * h)) - center.r_w).norm(), 2e-3);
  EXPECT_EQ(center.path_revision, frame->pathRevision());
  EXPECT_EQ(center.frame_revision, frame->frameRevision());
  EXPECT_FALSE(center.r_ww_valid);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
