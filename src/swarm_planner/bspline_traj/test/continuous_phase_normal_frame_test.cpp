#include <gtest/gtest.h>

#include <string>
#include <limits>
#include <cmath>

#include <bspline_race/continuous_phase_path.h>
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

TEST(ContinuousPhaseNormalFrameTest, NearVerticalTangentFailsHorizontalCapabilityClosed) {
  const auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  FLAG_Race::ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(0.0, 0.0, 0.0);
  start.dp_dw = Eigen::Vector3d(1e-10, 0.0, 1.0);
  start.d2p_dw2 = Eigen::Vector3d::Zero();
  start.vel = start.dp_dw.normalized();
  start.valid = true;
  FLAG_Race::ContinuousPhasePathState end = start;
  end.p = Eigen::Vector3d(2e-10, 0.0, 2.0);
  ASSERT_TRUE(path->appendSegment(
      0.0, 2.0,
      "near_vertical",
      FLAG_Race::ContinuousPhasePath::makeQuinticHermite(0.0, 2.0, start, end)));
  FLAG_Race::ContinuousPhaseNormalFrame frame(path, 3U, 5U);
  phase_offset_core::NormalFrameQuery query;
  EXPECT_FALSE(frame.query(1.0, query));
  EXPECT_FALSE(query.valid);
  EXPECT_NE(query.invalid_reason.find("horizontal"), std::string::npos);
  phase_offset_core::NormalFrameCellProof proof;
  EXPECT_FALSE(frame.certifyCell(0.25, 1.75, proof));
}

TEST(ContinuousPhaseNormalFrameTest, SlopedThreeDimensionalHorizontalInvariants) {
  const auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  FLAG_Race::ContinuousPhasePathState start;
  start.p = Eigen::Vector3d::Zero();
  start.dp_dw = Eigen::Vector3d(1.0, 2.0, 0.5);
  start.d2p_dw2 = Eigen::Vector3d(0.1, -0.2, 0.3);
  start.vel = start.dp_dw;
  start.valid = true;
  auto end = start;
  end.p = Eigen::Vector3d(4.0, 8.0, 2.0);
  ASSERT_TRUE(path->appendSegment(
      0.0, 4.0, "sloped", FLAG_Race::ContinuousPhasePath::makeQuinticHermite(
          0.0, 4.0, start, end)));
  FLAG_Race::ContinuousPhaseNormalFrame frame(path, 31U, 37U);
  const double w = 1.7;
  phase_offset_core::NormalFrameQuery q;
  ASSERT_TRUE(frame.query(w, q));
  EXPECT_NEAR(q.N.z(), 0.0, 1e-12);
  EXPECT_NEAR(q.N_w.z(), 0.0, 1e-12);
  EXPECT_NEAR(q.N.norm(), 1.0, 1e-10);
  EXPECT_NEAR(q.N.dot(q.T), 0.0, 1e-10);
  FLAG_Race::ContinuousPhasePathState state;
  ASSERT_TRUE(path->evaluate(w, state, false));
  EXPECT_NEAR(q.N.dot(state.dp_dw), 0.0, 1e-10);
  const double h = 1e-5;
  phase_offset_core::NormalFrameQuery left;
  phase_offset_core::NormalFrameQuery right;
  ASSERT_TRUE(frame.query(w - h, left));
  ASSERT_TRUE(frame.query(w + h, right));
  EXPECT_LT(((right.N - left.N) / (2.0 * h) - q.N_w).norm(), 2e-3);
  phase_offset_core::NormalFrameCellProof proof;
  ASSERT_TRUE(frame.certifyCell(0.5, 3.5, proof));
  EXPECT_TRUE(phase_offset_core::normalFrameCellProofIsComplete(proof));
  EXPECT_NE(proof.provenance.find("WorldHorizontalCrossProduct"),
            std::string::npos);
  EXPECT_GT(proof.inf_horizontal_path_speed,
            phase_offset_core::kHorizontalNormalSpeedEpsilon);
  EXPECT_TRUE(proof.horizontal_acceleration_bound_complete);
  EXPECT_GT(proof.tangent_variation_bound, 0.0);
  phase_offset_core::PathCellGeometryCertificate path_certificate;
  ASSERT_TRUE(path->cellBounds(0.5, 3.5, path_certificate));
  EXPECT_DOUBLE_EQ(proof.sup_normal_derivative,
                   path_certificate.sup_N_w_norm);
  EXPECT_DOUBLE_EQ(proof.normal_variation_bound,
                   path_certificate.normal_variation_bound);
  EXPECT_DOUBLE_EQ(proof.tangent_variation_bound,
                   path_certificate.tangent_variation_bound);
  phase_offset_core::NormalFrameCellProof incompatible = proof;
  incompatible.provenance =
      "ContinuousPhaseNormalFrame/ProjectedHermiteTransport";
  EXPECT_FALSE(phase_offset_core::normalFrameCellProofIsComplete(incompatible));

  phase_offset_core::NormalFrameCellProof understated_normal = proof;
  understated_normal.sup_normal_derivative = 0.0;
  EXPECT_FALSE(phase_offset_core::normalFrameCellProofIsComplete(
      understated_normal));
  phase_offset_core::NormalFrameCellProof understated_variation = proof;
  understated_variation.normal_variation_bound = 0.0;
  EXPECT_FALSE(phase_offset_core::normalFrameCellProofIsComplete(
      understated_variation));
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

TEST(ContinuousPhaseNormalFrameTest,
     ConstantHorizontalDirectionStillCarriesIndependentTangentVariation) {
  const auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
  FLAG_Race::ContinuousPhasePathState start;
  start.p = Eigen::Vector3d::Zero();
  start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  start.d2p_dw2 = Eigen::Vector3d(0.0, 0.0, 1.0);
  start.vel = start.dp_dw;
  start.valid = true;
  FLAG_Race::ContinuousPhasePathState end = start;
  end.p = Eigen::Vector3d(2.0, 0.0, 2.0);
  end.dp_dw = Eigen::Vector3d(1.0, 0.0, 2.0);
  ASSERT_TRUE(path->appendSegment(
      0.0, 2.0, "varying_height", FLAG_Race::ContinuousPhasePath::
          makeQuinticHermite(0.0, 2.0, start, end)));

  FLAG_Race::ContinuousPhaseNormalFrame frame(path, 53U, 59U);
  phase_offset_core::NormalFrameQuery query;
  ASSERT_TRUE(frame.query(1.0, query));
  EXPECT_NEAR(query.N_w.norm(), 0.0, 1e-12);

  phase_offset_core::NormalFrameCellProof proof;
  ASSERT_TRUE(frame.certifyCell(0.25, 1.75, proof));
  EXPECT_DOUBLE_EQ(proof.sup_horizontal_p_ww_norm, 0.0);
  EXPECT_DOUBLE_EQ(proof.sup_normal_derivative, 0.0);
  EXPECT_DOUBLE_EQ(proof.normal_variation_bound, 0.0);
  EXPECT_GT(proof.tangent_variation_bound, 0.0);
  EXPECT_TRUE(phase_offset_core::normalFrameCellProofIsComplete(proof));
}

TEST(ContinuousPhaseNormalFrameTest, HorizontalNormalThresholdIsStrict) {
  const double epsilon = phase_offset_core::kHorizontalNormalSpeedEpsilon;
  const double values[] = {
      std::nextafter(epsilon, 0.0), epsilon,
      std::nextafter(epsilon, std::numeric_limits<double>::infinity())};
  for (const double q : values) {
    const auto path = std::make_shared<FLAG_Race::ContinuousPhasePath>();
    FLAG_Race::ContinuousPhasePathState start;
    start.p = Eigen::Vector3d::Zero();
    start.dp_dw = Eigen::Vector3d(q, 0.0, 1.0);
    start.d2p_dw2 = Eigen::Vector3d::Zero();
    start.vel = start.dp_dw;
    start.valid = true;
    auto end = start;
    end.p = Eigen::Vector3d(2.0 * q, 0.0, 2.0);
    ASSERT_TRUE(path->appendSegment(
        0.0, 2.0, "threshold", FLAG_Race::ContinuousPhasePath::
            makeQuinticHermite(0.0, 2.0, start, end)));
    FLAG_Race::ContinuousPhaseNormalFrame frame(path, 41U, 43U);
    phase_offset_core::NormalFrameQuery query;
    EXPECT_EQ(frame.query(1.0, query), q > epsilon);
    phase_offset_core::CertifiedPathCellV2 v2_certificate;
    const bool v2_ok = path->tubeCellBoundsV2(0.25, 1.75, v2_certificate);
    EXPECT_EQ(v2_ok, q > epsilon);
    if (v2_ok) {
      EXPECT_GT(v2_certificate.inf_horizontal_p_w_norm.lower, epsilon);
      EXPECT_TRUE(phase_offset_core::certifiedPathCellV2IsComplete(
          v2_certificate));
      FLAG_Race::ContinuousPhasePathState state;
      ASSERT_TRUE(path->evaluate(1.0, state, false));
      EXPECT_LE(v2_certificate.inf_horizontal_p_w_norm.lower,
                std::hypot(state.dp_dw.x(), state.dp_dw.y()));
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
