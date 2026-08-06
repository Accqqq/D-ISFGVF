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
    expectVectorNear(state.d2p_dw2, numerical_d2p, 2e-4);
}

TEST(ContinuousPhasePath, FinitePointPathKeepsStationaryTerminalState)
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
    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeMappedBspline(
        spline, t0, t1, 0.0, 2.0);
    ASSERT_TRUE(static_cast<bool>(evaluator));

    FLAG_Race::ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, 2.0, "point_bspline", evaluator));
    FLAG_Race::ContinuousPhasePathState terminal;
    ASSERT_TRUE(path.evaluate(2.0, terminal, false));
    EXPECT_TRUE(terminal.p.allFinite());
    EXPECT_TRUE(terminal.dp_dw.allFinite());
    EXPECT_LT(terminal.dp_dw.norm(), 1e-8);

    std::vector<double> phase;
    std::vector<FLAG_Race::ContinuousPhasePathState> states;
    ASSERT_TRUE(path.sample(0.05, phase, states));
    ASSERT_FALSE(states.empty());
    EXPECT_NEAR(phase.back(), 2.0, 1e-12);
    EXPECT_LT(states.back().dp_dw.norm(), 1e-8);
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

}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
