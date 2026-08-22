#include <gtest/gtest.h>

#include <bspline_race/continuous_phase_path.h>

namespace
{

void expectVectorNear(const Eigen::Vector3d& actual,
                      const Eigen::Vector3d& expected,
                      double tolerance)
{
    EXPECT_NEAR((actual - expected).norm(), 0.0, tolerance);
}

TEST(ContinuousPhasePath, QuinticMatchesPositionAndTwoPhaseDerivatives)
{
    FLAG_Race::ContinuousPhasePathState start;
    start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
    start.dp_dw = Eigen::Vector3d(1.0, 0.2, 0.0);
    start.d2p_dw2 = Eigen::Vector3d(0.1, -0.3, 0.0);
    start.vel = Eigen::Vector3d(1.5, 0.3, 0.0);
    start.valid = true;

    FLAG_Race::ContinuousPhasePathState end;
    end.p = Eigen::Vector3d(1.2, 0.8, 1.0);
    end.dp_dw = Eigen::Vector3d(0.4, 1.1, 0.0);
    end.d2p_dw2 = Eigen::Vector3d(-0.2, 0.15, 0.0);
    end.vel = Eigen::Vector3d(0.6, 1.65, 0.0);
    end.valid = true;

    const double w0 = 2.0;
    const double w1 = 2.8;
    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeQuinticHermite(
        w0, w1, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));

    FLAG_Race::ContinuousPhasePathState actual_start;
    FLAG_Race::ContinuousPhasePathState actual_end;
    ASSERT_TRUE(evaluator(w0, actual_start));
    ASSERT_TRUE(evaluator(w1, actual_end));
    expectVectorNear(actual_start.p, start.p, 1e-10);
    expectVectorNear(actual_start.dp_dw, start.dp_dw, 1e-10);
    expectVectorNear(actual_start.d2p_dw2, start.d2p_dw2, 1e-10);
    expectVectorNear(actual_end.p, end.p, 1e-9);
    expectVectorNear(actual_end.dp_dw, end.dp_dw, 1e-9);
    expectVectorNear(actual_end.d2p_dw2, end.d2p_dw2, 1e-8);
}

TEST(ContinuousPhasePath, MappedBsplineUsesExactChainRuleDerivatives)
{
    Eigen::MatrixXd control_points(7, 3);
    control_points <<
        0.0, 0.0, 1.0,
        0.4, 0.0, 1.0,
        0.8, 0.2, 1.0,
        1.2, 0.5, 1.0,
        1.6, 0.9, 1.0,
        2.0, 1.1, 1.0,
        2.4, 1.2, 1.0;
    FLAG_Race::UniformBspline spline;
    ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, 0.2));

    const double t0 = spline.t_range(0);
    const double t1 = spline.t_range(1);
    const double w0 = 4.0;
    const double w1 = 7.0;
    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeMappedBspline(
        spline, t0, t1, w0, w1);
    ASSERT_TRUE(static_cast<bool>(evaluator));

    const double w = 5.2;
    const double eps = 1e-5;
    FLAG_Race::ContinuousPhasePathState state;
    FLAG_Race::ContinuousPhasePathState plus;
    FLAG_Race::ContinuousPhasePathState minus;
    ASSERT_TRUE(evaluator(w, state));
    ASSERT_TRUE(evaluator(w + eps, plus));
    ASSERT_TRUE(evaluator(w - eps, minus));

    const Eigen::Vector3d numerical_dp = (plus.p - minus.p) / (2.0 * eps);
    const Eigen::Vector3d numerical_d2p =
        (plus.p - 2.0 * state.p + minus.p) / (eps * eps);
    expectVectorNear(state.dp_dw, numerical_dp, 1e-5);
    expectVectorNear(state.d2p_dw2, numerical_d2p, 5e-3);

    const double expected_phase_speed =
        (plus.p - minus.p).norm() / (2.0 * eps);
    EXPECT_NEAR(state.dp_dw.norm(), expected_phase_speed, 1e-5);

    const double t = t0 + (t1 - t0) * 0.4;
    const FLAG_Race::UniformBspline velocity_spline = spline.getDerivative();
    const Eigen::Vector3d expected_velocity = velocity_spline.singleDeboor(
        t * velocity_spline.beta_ + velocity_spline.u_(velocity_spline.p_));
    // The arclength map changes p_w, but the velocity field remains the raw
    // planner B-spline's physical-time derivative at the mapped world point.
    EXPECT_GT(state.vel.norm(), 0.0);
    EXPECT_TRUE(expected_velocity.allFinite());
}

TEST(ContinuousPhasePath, StationaryRawEndpointsAreNotExecutable)
{
    Eigen::MatrixXd control_points(9, 3);
    control_points <<
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.5, 0.1, 1.0,
        1.0, 0.2, 1.0,
        1.5, 0.1, 1.0,
        2.0, 0.0, 1.0,
        2.0, 0.0, 1.0,
        2.0, 0.0, 1.0;
    FLAG_Race::UniformBspline spline;
    ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, 0.2));

    const double t0 = spline.t_range(0);
    const double t1 = spline.t_range(1);
    const auto raw_endpoint_evaluator =
        FLAG_Race::ContinuousPhasePath::makeMappedBspline(
        spline, t0, t1, 0.0, 2.0);
    // Strict arclength certification rejects the stationary endpoints, but
    // point navigation keeps a finite legacy linear-t(w) evaluator.
    ASSERT_TRUE(static_cast<bool>(raw_endpoint_evaluator));
    FLAG_Race::ContinuousPhasePathState raw_midpoint;
    ASSERT_TRUE(raw_endpoint_evaluator(1.0, raw_midpoint));
    EXPECT_TRUE(raw_midpoint.p.allFinite());
    EXPECT_TRUE(raw_midpoint.dp_dw.allFinite());
    FLAG_Race::ContinuousPhasePath raw_path;
    ASSERT_TRUE(raw_path.appendSegment(0.0, 2.0, "legacy_point_only",
                                       raw_endpoint_evaluator));
    phase_offset_core::PathCellGeometryCertificate raw_certificate;
    EXPECT_FALSE(raw_path.cellBounds(0.2, 1.8, raw_certificate));

    const double interior_t0 = t0 + 0.02;
    const double interior_t1 = t1 - 0.02;
    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeMappedBspline(
        spline, interior_t0, interior_t1, 0.05, 1.95);
    ASSERT_TRUE(static_cast<bool>(evaluator));

    FLAG_Race::ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.05, 1.95, "point_bspline", evaluator));
    FLAG_Race::ContinuousPhasePathState initial;
    FLAG_Race::ContinuousPhasePathState terminal;
    ASSERT_TRUE(path.evaluate(0.05, initial, false));
    ASSERT_TRUE(path.evaluate(1.95, terminal, false));
    EXPECT_TRUE(initial.p.allFinite());
    EXPECT_TRUE(initial.dp_dw.allFinite());
    EXPECT_GT(initial.dp_dw.norm(), 1e-3);
    EXPECT_TRUE(terminal.p.allFinite());
    EXPECT_TRUE(terminal.dp_dw.allFinite());
    EXPECT_GT(terminal.dp_dw.norm(), 1e-3);

    std::vector<double> phase;
    std::vector<FLAG_Race::ContinuousPhasePathState> states;
    ASSERT_TRUE(path.sample(0.05, phase, states));
    ASSERT_FALSE(states.empty());
    EXPECT_NEAR(phase.front(), 0.05, 1e-12);
    EXPECT_NEAR(phase.back(), 1.95, 1e-12);
    const double expected_speed = states.front().dp_dw.norm();
    ASSERT_GT(expected_speed, 1e-3);
    for (const auto& state : states) {
        EXPECT_NEAR(state.dp_dw.norm(), expected_speed, 2e-6);
        EXPECT_TRUE(state.d2p_dw2.allFinite());
    }
}

TEST(ContinuousPhasePath, MappedBsplineKeepsSecondDerivativeContinuous)
{
    Eigen::MatrixXd control_points(8, 3);
    control_points <<
        0.0, 0.0, 1.0,
        0.3, 0.0, 1.0,
        0.7, 0.3, 1.0,
        1.0, 0.8, 1.0,
        1.3, 1.1, 1.0,
        1.7, 1.0, 1.0,
        2.1, 0.6, 1.0,
        2.5, 0.5, 1.0;
    FLAG_Race::UniformBspline spline;
    ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, 0.2));
    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeMappedBspline(
        spline, spline.t_range(0), spline.t_range(1), 2.0, 5.0);
    ASSERT_TRUE(static_cast<bool>(evaluator));

    // The evaluator has 1024 arclength cells.  Sweep every internal map knot
    // from both sides: p, p_w and p_ww must remain continuous there.
    const double eps = 1e-8;
    for (int i = 1; i < 1024; ++i) {
        const double w = 2.0 + 3.0 * static_cast<double>(i) / 1024.0;
        FLAG_Race::ContinuousPhasePathState left;
        FLAG_Race::ContinuousPhasePathState right;
        ASSERT_TRUE(evaluator(w - eps, left));
        ASSERT_TRUE(evaluator(w + eps, right));
        EXPECT_LT((left.p - right.p).norm(), 5e-8);
        EXPECT_LT((left.dp_dw - right.dp_dw).norm(), 5e-7);
        EXPECT_LT((left.d2p_dw2 - right.d2p_dw2).norm(), 5e-5);
    }
}

TEST(ContinuousPhasePath, RawEndpointTrimUsesAccurateArcLengthInversion)
{
    Eigen::MatrixXd control_points(9, 3);
    control_points <<
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.5, 0.2, 1.0,
        1.2, 0.8, 1.0,
        1.8, 1.2, 1.0,
        2.5, 1.1, 1.0,
        2.5, 1.1, 1.0,
        2.5, 1.1, 1.0;
    FLAG_Race::UniformBspline spline;
    ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, 0.2));

    const double raw_t0 = spline.t_range(0);
    const double raw_t1 = spline.t_range(1);
    double full_length = 0.0;
    ASSERT_TRUE(FLAG_Race::ContinuousPhasePath::measureBsplineArcLength(
        spline, raw_t0, raw_t1, full_length));
    ASSERT_GT(full_length, 0.4);

    const double trim_start = 0.12;
    const double trim_end = 0.18;
    double exec_t0 = 0.0;
    double exec_t1 = 0.0;
    double trimmed_total = 0.0;
    ASSERT_TRUE(FLAG_Race::ContinuousPhasePath::trimBsplineTimeDomainByArcLength(
        spline, raw_t0, raw_t1, trim_start, trim_end,
        exec_t0, exec_t1, trimmed_total));
    EXPECT_NEAR(trimmed_total, full_length, 1e-10);
    EXPECT_GT(exec_t0, raw_t0);
    EXPECT_LT(exec_t1, raw_t1);

    double start_length = 0.0;
    double interior_length = 0.0;
    double end_length = 0.0;
    ASSERT_TRUE(FLAG_Race::ContinuousPhasePath::measureBsplineArcLength(
        spline, raw_t0, exec_t0, start_length));
    ASSERT_TRUE(FLAG_Race::ContinuousPhasePath::measureBsplineArcLength(
        spline, exec_t0, exec_t1, interior_length));
    ASSERT_TRUE(FLAG_Race::ContinuousPhasePath::measureBsplineArcLength(
        spline, exec_t1, raw_t1, end_length));
    EXPECT_NEAR(start_length, trim_start, 2e-5);
    EXPECT_NEAR(end_length, trim_end, 2e-5);
    EXPECT_NEAR(interior_length, full_length - trim_start - trim_end, 4e-5);

    const double w0 = 3.0;
    const double w1 = w0 + interior_length;
    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeMappedBspline(
        spline, exec_t0, exec_t1, w0, w1);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    FLAG_Race::ContinuousPhasePathState start;
    FLAG_Race::ContinuousPhasePathState end;
    ASSERT_TRUE(evaluator(w0, start));
    ASSERT_TRUE(evaluator(w1, end));
    EXPECT_GT(start.dp_dw.norm(), 1e-5);
    EXPECT_GT(end.dp_dw.norm(), 1e-5);
    EXPECT_NEAR(start.dp_dw.norm(), 1.0, 5e-5);
    EXPECT_NEAR(end.dp_dw.norm(), 1.0, 5e-5);
}

TEST(ContinuousPhasePath, CompositeRetainsOldPrefixAndC2Join)
{
    const Eigen::Vector3d center(0.0, 0.0, 1.0);
    const auto nominal = FLAG_Race::ContinuousPhasePath::makePeriodicCircle(
        center, 2.0, 4.0 * M_PI, 1.0);
    ASSERT_TRUE(static_cast<bool>(nominal));

    FLAG_Race::ContinuousPhasePath old_path;
    ASSERT_TRUE(old_path.appendSegment(-1.0, 1.0, "old", nominal));
    FLAG_Race::ContinuousPhasePathState old_state;
    ASSERT_TRUE(old_path.evaluate(0.0, old_state, false));

    FLAG_Race::ContinuousPhasePathState new_state;
    new_state.p = Eigen::Vector3d(2.2, 0.7, 1.0);
    new_state.dp_dw = Eigen::Vector3d(-0.3, 1.0, 0.0);
    new_state.d2p_dw2 = Eigen::Vector3d(-0.2, -0.1, 0.0);
    new_state.vel = new_state.dp_dw.normalized();
    new_state.valid = true;

    const double join_w = 0.8;
    const auto connector = FLAG_Race::ContinuousPhasePath::makeQuinticHermite(
        0.0, join_w, old_state, new_state);
    ASSERT_TRUE(static_cast<bool>(connector));

    FLAG_Race::ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSlice(old_path, -0.5, 0.0));
    ASSERT_TRUE(path.appendSegment(0.0, join_w, "connector", connector));
    ASSERT_TRUE(path.appendSegment(
        join_w, 1.8, "new",
        [new_state, join_w](double w, FLAG_Race::ContinuousPhasePathState& state) {
            const double dw = w - join_w;
            state = new_state;
            state.p = new_state.p + new_state.dp_dw * dw +
                      0.5 * new_state.d2p_dw2 * dw * dw;
            state.dp_dw = new_state.dp_dw + new_state.d2p_dw2 * dw;
            state.vel = state.dp_dw.normalized();
            state.valid = state.p.allFinite() && state.dp_dw.norm() > 1e-9;
            return state.valid;
        }));

    FLAG_Race::ContinuousPhasePathState at_switch;
    FLAG_Race::ContinuousPhasePathState at_join;
    ASSERT_TRUE(path.evaluate(0.0, at_switch, false));
    ASSERT_TRUE(path.evaluate(join_w, at_join, false));
    expectVectorNear(at_switch.p, old_state.p, 1e-10);
    expectVectorNear(at_switch.dp_dw, old_state.dp_dw, 1e-10);
    expectVectorNear(at_switch.d2p_dw2, old_state.d2p_dw2, 1e-10);
    expectVectorNear(at_join.p, new_state.p, 1e-9);
    expectVectorNear(at_join.dp_dw, new_state.dp_dw, 1e-9);
    expectVectorNear(at_join.d2p_dw2, new_state.d2p_dw2, 1e-8);
}

TEST(ContinuousPhasePath, QuinticCellCertificateContainsAdversarialQueries)
{
    FLAG_Race::ContinuousPhasePathState start;
    start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
    start.dp_dw = Eigen::Vector3d(1.0, 0.2, 0.0);
    start.d2p_dw2 = Eigen::Vector3d(0.1, -0.3, 0.0);
    start.vel = start.dp_dw;
    start.valid = true;
    FLAG_Race::ContinuousPhasePathState end;
    end.p = Eigen::Vector3d(1.2, 0.8, 1.0);
    end.dp_dw = Eigen::Vector3d(0.4, 1.1, 0.0);
    end.d2p_dw2 = Eigen::Vector3d(-0.2, 0.15, 0.0);
    end.vel = end.dp_dw;
    end.valid = true;
    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeQuinticHermite(
        2.0, 2.8, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    FLAG_Race::ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(2.0, 2.8, "certified_quintic", evaluator));
    phase_offset_core::PathCellGeometryCertificate certificate;
    ASSERT_TRUE(path.cellBounds(2.1, 2.7, certificate));
    ASSERT_TRUE(phase_offset_core::pathCellGeometryCertificateIsComplete(
        certificate));
    EXPECT_EQ(certificate.segment_identity, 1U);
    EXPECT_GT(certificate.inf_p_w_norm, 0.0);
    EXPECT_GT(certificate.inf_horizontal_p_w_norm, 0.0);
    for (int index = 0; index <= 1000; ++index) {
        const double w = 2.1 + 0.6 * static_cast<double>(index) / 1000.0;
        FLAG_Race::ContinuousPhasePathState state;
        ASSERT_TRUE(path.evaluate(w, state, false));
        EXPECT_GE(state.dp_dw.norm() + 1e-10, certificate.inf_p_w_norm);
        EXPECT_LE(state.dp_dw.norm(), certificate.sup_p_w_norm + 1e-10);
        EXPECT_LE(state.d2p_dw2.norm(), certificate.sup_p_ww_norm + 1e-10);
    }
    EXPECT_GT(certificate.chord_deviation_bound, 0.0);
}

TEST(ContinuousPhasePath, CellCertificateFailsClosedAcrossSegmentsAndUnsupported)
{
    const auto first = FLAG_Race::ContinuousPhasePath::makePeriodicCircle(
        Eigen::Vector3d::Zero(), 1.0, 4.0 * M_PI, 1.0);
    const auto second = FLAG_Race::ContinuousPhasePath::makePeriodicCircle(
        Eigen::Vector3d::Zero(), 1.0, 4.0 * M_PI, 1.0);
    FLAG_Race::ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, 1.0, "unsupported_circle", first));
    ASSERT_TRUE(path.appendSegment(1.0, 2.0, "unsupported_circle_2", second));
    phase_offset_core::PathCellGeometryCertificate certificate;
    EXPECT_FALSE(path.cellBounds(0.2, 0.8, certificate));
    EXPECT_FALSE(path.cellBounds(0.8, 1.2, certificate));
    EXPECT_FALSE(path.cellBounds(1.2, 1.8, certificate));
}

TEST(ContinuousPhasePath, MappedBsplineCellCertificateUsesMonotoneMapCells)
{
    Eigen::MatrixXd control_points(8, 3);
    control_points <<
        0.0, 0.0, 1.0,
        0.3, 0.0, 1.0,
        0.7, 0.3, 1.0,
        1.0, 0.8, 1.0,
        1.3, 1.1, 1.0,
        1.7, 1.0, 1.0,
        2.1, 0.6, 1.0,
        2.5, 0.5, 1.0;
    FLAG_Race::UniformBspline spline;
    ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, 0.2));
    const double t0 = spline.t_range(0);
    const double t1 = spline.t_range(1);
    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeMappedBspline(
        spline, t0, t1, 2.0, 5.0);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    FLAG_Race::ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(2.0, 5.0, "mapped_bspline", evaluator));
    phase_offset_core::PathCellGeometryCertificate certificate;
    ASSERT_TRUE(path.cellBounds(2.2, 2.8, certificate));
    ASSERT_TRUE(phase_offset_core::pathCellGeometryCertificateIsComplete(
        certificate));
    EXPECT_GT(certificate.inf_p_w_norm, 0.0);
    EXPECT_GT(certificate.sup_p_www_norm, 0.0);
    for (int index = 0; index <= 400; ++index) {
        const double w = 2.2 + 0.6 * static_cast<double>(index) / 400.0;
        FLAG_Race::ContinuousPhasePathState state;
        ASSERT_TRUE(path.evaluate(w, state, false));
        EXPECT_GE(state.dp_dw.norm() + 1e-8, certificate.inf_p_w_norm);
        EXPECT_LE(state.dp_dw.norm(), certificate.sup_p_w_norm + 1e-8);
        EXPECT_LE(state.d2p_dw2.norm(), certificate.sup_p_ww_norm + 1e-7);
    }
}

TEST(ContinuousPhasePath,
     MappedBsplineLinearFallbackRetainsCellCertificateWhenMapCannotBuild)
{
    // The repeated first controls make the raw spline endpoint stationary.
    // The strict arclength map consequently cannot be constructed, while the
    // interior of the same planner curve remains executable.  The fallback
    // t(w) evaluator must expose a cell certificate for that interior rather
    // than silently forcing the historical fixed inset for the whole segment.
    Eigen::MatrixXd control_points(8, 3);
    control_points <<
        0.0, 0.0, 1.0,
        0.0, 0.0, 1.0,
        0.4, 0.0, 1.0,
        0.8, 0.2, 1.0,
        1.2, 0.5, 1.0,
        1.6, 0.7, 1.0,
        2.0, 0.6, 1.0,
        2.4, 0.5, 1.0;
    FLAG_Race::UniformBspline spline;
    ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, 0.2));
    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeMappedBspline(
        spline, spline.t_range(0), spline.t_range(1), 0.0, 2.0);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    FLAG_Race::ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, 2.0, "mapped_linear_fallback",
                                   evaluator));
    phase_offset_core::PathCellGeometryCertificate certificate;
    ASSERT_TRUE(path.cellBounds(0.5, 1.5, certificate));
    EXPECT_TRUE(phase_offset_core::pathCellGeometryCertificateIsComplete(
        certificate));
    EXPECT_GT(certificate.inf_p_w_norm, 0.0);
    FLAG_Race::ContinuousPhasePathState state;
    EXPECT_TRUE(path.evaluate(1.0, state, false));
    EXPECT_TRUE(state.valid);
}

TEST(ContinuousPhasePath,
     MappedBsplinePointEvaluatorSurvivesUnsupportedHighOrderCertificate)
{
    Eigen::MatrixXd control_points(3, 3);
    control_points <<
        0.0, 0.0, 1.0,
        0.8, 0.2, 1.0,
        1.6, 0.0, 1.0;
    FLAG_Race::UniformBspline spline;
    ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 2, 0.2));
    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeMappedBspline(
        spline, spline.t_range(0), spline.t_range(1), 0.0, 2.0);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    FLAG_Race::ContinuousPhasePathState state;
    ASSERT_TRUE(evaluator(1.0, state));
    EXPECT_TRUE(state.valid);
    FLAG_Race::ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, 2.0, "order2_mapped", evaluator));
    phase_offset_core::PathCellGeometryCertificate certificate;
    EXPECT_FALSE(path.cellBounds(0.5, 1.5, certificate));
    ASSERT_TRUE(path.evaluate(1.0, state, false));
}

}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
