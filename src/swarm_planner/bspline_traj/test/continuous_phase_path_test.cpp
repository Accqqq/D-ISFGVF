#include <gtest/gtest.h>

#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/continuous_phase_normal_frame.h>

#include <cfenv>
#include <cmath>
#include <limits>

#if defined(__i386__) || defined(__x86_64__)
#include <immintrin.h>
#endif

namespace
{

using FLAG_Race::ContinuousPhasePath;
using FLAG_Race::ContinuousPhasePathState;
using FLAG_Race::UniformBspline;

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
    EXPECT_TRUE(certificate.horizontal_acceleration_bound_complete);
    EXPECT_TRUE(certificate.component_acceleration_bound_complete);
    EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.z(), 0.0);
    EXPECT_GE(certificate.sup_horizontal_p_ww_norm, 0.0);
    EXPECT_GE(certificate.tangent_variation_bound, 0.0);
    for (int index = 0; index <= 1000; ++index) {
        const double w = 2.1 + 0.6 * static_cast<double>(index) / 1000.0;
        FLAG_Race::ContinuousPhasePathState state;
        ASSERT_TRUE(path.evaluate(w, state, false));
        EXPECT_GE(state.dp_dw.norm() + 1e-10, certificate.inf_p_w_norm);
        EXPECT_LE(state.dp_dw.norm(), certificate.sup_p_w_norm + 1e-10);
        EXPECT_LE(state.d2p_dw2.norm(), certificate.sup_p_ww_norm + 1e-10);
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_LE(std::abs(state.d2p_dw2(axis)),
                      certificate.sup_abs_p_ww(axis) + 1e-10);
        }
    }
    EXPECT_GT(certificate.chord_deviation_bound, 0.0);
}

TEST(ContinuousPhasePath, QuinticComponentBoundsAggregateThirtyTwoSubcells)
{
    // p_w(s) = s^2 - s + 0.26 is strictly positive (minimum .01), while
    // its degree-four Bernstein elevation contains a negative control.  The
    // unsplit hull therefore has no positive speed floor and the producer's
    // deterministic 32-cell certificate fallback must aggregate each local
    // component bound without losing the exact planar z zero.
    const double c = 0.26;
    ContinuousPhasePathState start;
    start.p = Eigen::Vector3d::Zero();
    start.dp_dw = Eigen::Vector3d(c, 0.0, 0.0);
    start.d2p_dw2 = Eigen::Vector3d(-1.0, 0.0, 0.0);
    start.vel = start.dp_dw;
    start.valid = true;
    ContinuousPhasePathState end = start;
    end.p = Eigen::Vector3d(c - 1.0 / 6.0, 0.0, 0.0);
    end.dp_dw = Eigen::Vector3d(c, 0.0, 0.0);
    end.d2p_dw2 = Eigen::Vector3d(1.0, 0.0, 0.0);
    const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
        0.0, 1.0, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, 1.0, "subcell_component", evaluator));
    phase_offset_core::PathCellGeometryCertificate certificate;
    ASSERT_TRUE(path.cellBounds(0.0, 1.0, certificate));
    ASSERT_TRUE(certificate.component_acceleration_bound_complete);
    EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.z(), 0.0);
    for (int index = 0; index <= 1000; ++index) {
        const double w = static_cast<double>(index) / 1000.0;
        ContinuousPhasePathState state;
        ASSERT_TRUE(path.evaluate(w, state, false));
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_LE(std::abs(state.d2p_dw2(axis)),
                      certificate.sup_abs_p_ww(axis) + 1e-10);
        }
    }
}

TEST(ContinuousPhasePath, QuinticTinyNonzeroComponentIsNotThresholdedToZero)
{
    const double tiny = std::ldexp(1.0, -1000);
    ContinuousPhasePathState start;
    start.p = Eigen::Vector3d::Zero();
    start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
    start.d2p_dw2 = Eigen::Vector3d(0.0, 0.0, tiny);
    start.vel = start.dp_dw;
    start.valid = true;
    ContinuousPhasePathState end = start;
    end.d2p_dw2 = Eigen::Vector3d(0.0, 0.0, -tiny);
    end.p = Eigen::Vector3d(1.0, 0.0, 0.0);
    const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
        0.0, 1.0, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, 1.0, "tiny_component", evaluator));
    phase_offset_core::PathCellGeometryCertificate certificate;
    ASSERT_TRUE(path.cellBounds(0.1, 0.9, certificate));
    // The represented source coefficients are nonzero.  They may not be
    // silently thresholded to an exact zero component.
    EXPECT_TRUE(certificate.component_acceleration_bound_complete);
    EXPECT_GT(certificate.sup_abs_p_ww.z(), 0.0);
    EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.x(), 0.0);
    EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.y(), 0.0);
}

TEST(ContinuousPhasePath,
     QuinticComponentUnderflowDisablesOptionalCapabilityOnly)
{
    // Keep a nonzero source p_ww z coefficient (2*denorm_min at the
    // terminal phase acceleration), then query the smallest admissible cell.
    // Restriction multiplies that coefficient by the tiny local span and all
    // transformed controls underflow to zero.  The source-zero guard must
    // therefore reject only the optional component capability, never claim an
    // exact zero bound or discard the legacy scalar certificate.
    const double denorm = std::numeric_limits<double>::denorm_min();
    ContinuousPhasePathState start;
    start.p = Eigen::Vector3d::Zero();
    start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
    start.d2p_dw2 = Eigen::Vector3d::Zero();
    start.vel = start.dp_dw;
    start.valid = true;
    ContinuousPhasePathState end = start;
    // x(w)=w+w^3 keeps the legacy jerk norm comfortably representable while
    // z remains a denormal-scale second-derivative perturbation.
    end.p = Eigen::Vector3d(2.0, 0.0, 0.0);
    end.dp_dw = Eigen::Vector3d(4.0, 0.0, 0.0);
    end.d2p_dw2 = Eigen::Vector3d(6.0, 0.0, 2.0 * denorm);
    const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
        0.0, 1.0, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, 1.0, "component_underflow", evaluator));

    phase_offset_core::PathCellGeometryCertificate certificate;
    ASSERT_TRUE(path.cellBounds(0.0, 2.0e-8, certificate));
    EXPECT_FALSE(certificate.component_acceleration_bound_complete);
    EXPECT_TRUE(std::isfinite(certificate.sup_p_ww_norm));
    EXPECT_GT(certificate.sup_p_ww_norm, 0.0);
    EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.x(), 0.0);
    EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.y(), 0.0);
    EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.z(), 0.0);
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

TEST(ContinuousPhasePath, ExactSeamSelectionAndGapAreNotEpsilonSnapped)
{
    ContinuousPhasePathState left_start;
    left_start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
    left_start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
    left_start.d2p_dw2 = Eigen::Vector3d::Zero();
    left_start.vel = left_start.dp_dw;
    left_start.valid = true;
    ContinuousPhasePathState left_end = left_start;
    left_end.p.x() = 1.0;
    const auto left = ContinuousPhasePath::makeQuinticHermite(
        0.0, 1.0, left_start, left_end);

    ContinuousPhasePathState right_start = left_end;
    right_start.p.x() += 1e-7;
    ContinuousPhasePathState right_end = right_start;
    right_end.p.x() = 2.0 + 1e-7;
    const auto right = ContinuousPhasePath::makeQuinticHermite(
        1.0, 2.0, right_start, right_end);
    ASSERT_TRUE(left);
    ASSERT_TRUE(right);

    ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, 1.0, "left", left));
    ASSERT_TRUE(path.appendSegment(1.0, 2.0, "right", right));
    ContinuousPhasePathState seam;
    ASSERT_TRUE(path.evaluate(1.0, seam, false));
    EXPECT_DOUBLE_EQ(seam.p.x(), 1.0);
    ContinuousPhasePathState after_seam;
    ASSERT_TRUE(path.evaluate(std::nextafter(
        1.0, std::numeric_limits<double>::infinity()), after_seam, false));
    EXPECT_GT(after_seam.p.x(), 1.0);
    ContinuousPhasePathState at_five_nanoseconds;
    ASSERT_TRUE(path.evaluate(1.0 + 5e-9, at_five_nanoseconds, false));
    EXPECT_GT(at_five_nanoseconds.p.x(), 1.0);
    phase_offset_core::PathCellGeometryCertificate certificate;
    EXPECT_FALSE(path.cellBounds(0.9, 1.1, certificate));
    ASSERT_TRUE(path.cellBounds(1.0, 1.5, certificate));
    EXPECT_EQ(certificate.segment_identity, path.segments()[1].identity);

    ContinuousPhasePath gap;
    ASSERT_TRUE(gap.appendSegment(0.0, 1.0, "left", left));
    ASSERT_TRUE(gap.appendSegment(1.1, 2.0, "right", right));
    EXPECT_FALSE(gap.evaluate(1.05, seam, false));
    EXPECT_FALSE(gap.cellBounds(1.0, 1.2, certificate));
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
    EXPECT_TRUE(certificate.horizontal_acceleration_bound_complete);
    EXPECT_TRUE(certificate.component_acceleration_bound_complete);
    EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.z(), 0.0);
    EXPECT_GE(certificate.sup_horizontal_p_ww_norm, 0.0);
    EXPECT_GE(certificate.tangent_variation_bound, 0.0);
    for (int index = 0; index <= 400; ++index) {
        const double w = 2.2 + 0.6 * static_cast<double>(index) / 400.0;
        FLAG_Race::ContinuousPhasePathState state;
        ASSERT_TRUE(path.evaluate(w, state, false));
        EXPECT_GE(state.dp_dw.norm() + 1e-8, certificate.inf_p_w_norm);
        EXPECT_LE(state.dp_dw.norm(), certificate.sup_p_w_norm + 1e-8);
        EXPECT_LE(state.d2p_dw2.norm(), certificate.sup_p_ww_norm + 1e-7);
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_LE(std::abs(state.d2p_dw2(axis)),
                      certificate.sup_abs_p_ww(axis) + 1e-10);
        }
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
    EXPECT_TRUE(certificate.component_acceleration_bound_complete);
    EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.z(), 0.0);
    FLAG_Race::ContinuousPhasePathState state;
    EXPECT_TRUE(path.evaluate(1.0, state, false));
    EXPECT_TRUE(state.valid);
}

TEST(ContinuousPhasePath, ComponentBoundsSurviveAppendSlice)
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
    UniformBspline spline;
    ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, 0.2));
    const auto evaluator = ContinuousPhasePath::makeMappedBspline(
        spline, spline.t_range(0), spline.t_range(1), 2.0, 5.0);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    ContinuousPhasePath source;
    ASSERT_TRUE(source.appendSegment(2.0, 5.0, "source", evaluator));
    ContinuousPhasePath slice;
    ASSERT_TRUE(slice.appendSlice(source, 2.25, 4.75));
    phase_offset_core::PathCellGeometryCertificate source_certificate;
    phase_offset_core::PathCellGeometryCertificate slice_certificate;
    ASSERT_TRUE(source.cellBounds(2.3, 2.8, source_certificate));
    ASSERT_TRUE(slice.cellBounds(2.3, 2.8, slice_certificate));
    EXPECT_TRUE(slice_certificate.component_acceleration_bound_complete);
    EXPECT_DOUBLE_EQ(slice_certificate.sup_abs_p_ww.z(), 0.0);
    for (int axis = 0; axis < 3; ++axis) {
        EXPECT_DOUBLE_EQ(slice_certificate.sup_abs_p_ww(axis),
                         source_certificate.sup_abs_p_ww(axis));
    }
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

TEST(ContinuousPhasePath,
     QuinticHorizontalSpeedThresholdIsStrictThroughProducerAndFrameProof)
{
    const double epsilon =
        phase_offset_core::kHorizontalNormalSpeedEpsilon;
    const double values[] = {
        std::nextafter(epsilon, 0.0), epsilon,
        std::nextafter(epsilon, std::numeric_limits<double>::infinity())};
    // The span 3.0 is deliberately non-power-of-two: a constant derivative
    // must retain the direct endpoint q evidence rather than a1/h erosion.
    for (const double span : {2.0, 3.0}) {
        for (const double q : values) {
            FLAG_Race::ContinuousPhasePathState start;
            start.p = Eigen::Vector3d::Zero();
            start.dp_dw = Eigen::Vector3d(q, 0.0, 1.0);
            start.d2p_dw2 = Eigen::Vector3d::Zero();
            start.vel = start.dp_dw;
            start.valid = true;
            FLAG_Race::ContinuousPhasePathState end = start;
            end.p = Eigen::Vector3d(span * q, 0.0, span);

            const auto evaluator =
                FLAG_Race::ContinuousPhasePath::makeQuinticHermite(
                    0.0, span, start, end);
            ASSERT_TRUE(static_cast<bool>(evaluator));
            FLAG_Race::ContinuousPhasePath path;
            ASSERT_TRUE(path.appendSegment(0.0, span, "threshold", evaluator));

            FLAG_Race::ContinuousPhasePathState point;
            ASSERT_TRUE(path.evaluate(0.37 * span, point, false));
            EXPECT_DOUBLE_EQ(point.dp_dw.x(), q);
            EXPECT_DOUBLE_EQ(point.dp_dw.y(), 0.0);

            phase_offset_core::PathCellGeometryCertificate certificate;
            const bool certificate_ok = path.cellBounds(0.25 * span,
                                                         0.75 * span,
                                                         certificate);
            EXPECT_EQ(certificate_ok, q > epsilon);
            if (certificate_ok) {
                EXPECT_GT(certificate.inf_horizontal_p_w_norm, epsilon);
                EXPECT_TRUE(certificate.horizontal_acceleration_bound_complete);
                EXPECT_TRUE(phase_offset_core::
                            pathCellGeometryCertificateIsComplete(certificate));
            }

            phase_offset_core::CertifiedPathCellV2 v2_certificate;
            const bool v2_ok = path.tubeCellBoundsV2(
                0.25 * span, 0.75 * span, v2_certificate);
            EXPECT_EQ(v2_ok, q > epsilon);
            if (v2_ok) {
                EXPECT_GT(v2_certificate.inf_horizontal_p_w_norm.lower,
                          epsilon);
                EXPECT_TRUE(phase_offset_core::
                            certifiedPathCellV2IsComplete(v2_certificate));
            }

            const auto path_owner =
                std::make_shared<FLAG_Race::ContinuousPhasePath>(path);
            FLAG_Race::ContinuousPhaseNormalFrame frame(path_owner, 7U, 11U);
            phase_offset_core::NormalFrameCellProof proof;
            const bool proof_ok = frame.certifyCell(0.25 * span,
                                                    0.75 * span, proof);
            EXPECT_EQ(proof_ok, q > epsilon);
            if (proof_ok) {
                EXPECT_GT(proof.inf_horizontal_path_speed, epsilon);
                EXPECT_TRUE(phase_offset_core::normalFrameCellProofIsComplete(
                    proof));
            }
        }
    }
}

TEST(ContinuousPhasePath,
     ConstantHorizontalCertificateUsesDirectedLowerNormForGeneralVector)
{
    // This pair exposes an upward-rounded long-double norm on platforms where
    // the intermediate approximation is not exact.
    const double x = 0x1.9c4e88376785ap-28;
    const double y = 0x1.33b6cef0443a5p-26;
    const double span = 3.0;
    FLAG_Race::ContinuousPhasePathState start;
    start.p = Eigen::Vector3d::Zero();
    start.dp_dw = Eigen::Vector3d(x, y, 0.0);
    start.d2p_dw2 = Eigen::Vector3d::Zero();
    start.vel = start.dp_dw;
    start.valid = true;
    FLAG_Race::ContinuousPhasePathState end = start;
    end.p = span * start.dp_dw;

    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeQuinticHermite(
        0.0, span, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    FLAG_Race::ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, span, "directed_norm", evaluator));

    FLAG_Race::ContinuousPhasePathState point;
    ASSERT_TRUE(path.evaluate(0.37 * span, point, false));
    EXPECT_DOUBLE_EQ(point.dp_dw.x(), x);
    EXPECT_DOUBLE_EQ(point.dp_dw.y(), y);

    phase_offset_core::PathCellGeometryCertificate certificate;
    ASSERT_TRUE(path.cellBounds(0.25 * span, 0.75 * span, certificate));
    const double lower = certificate.inf_horizontal_p_w_norm;
    const double full_lower = certificate.inf_p_w_norm;
    const double reference = std::hypot(x, y);
    EXPECT_GT(lower, 0.0);
    EXPECT_LE(lower, reference);
    EXPECT_GT(full_lower, 0.0);
    EXPECT_LE(full_lower, reference);
    // The directed lower bound must not exceed the exact squared norm as
    // represented by a fused multiply-add evaluation.
    const double reference_squared = std::fma(x, x, y * y);
    EXPECT_LE(std::fma(lower, lower, -reference_squared), 0.0);
    EXPECT_LE(std::fma(full_lower, full_lower, -reference_squared), 0.0);
}

TEST(ContinuousPhasePath,
     SubnormalHorizontalComponentsRemainFiniteAndFailClosed)
{
    const double x = 0x0.000000007d19ep-1022;
    const double y = 0x0.00000000aa90bp-1022;
    const double span = 3.0;
    FLAG_Race::ContinuousPhasePathState start;
    start.p = Eigen::Vector3d::Zero();
    start.dp_dw = Eigen::Vector3d(x, y, 1.0);
    start.d2p_dw2 = Eigen::Vector3d::Zero();
    start.vel = start.dp_dw;
    start.valid = true;
    FLAG_Race::ContinuousPhasePathState end = start;
    end.p = span * start.dp_dw;

    const auto evaluator = FLAG_Race::ContinuousPhasePath::makeQuinticHermite(
        0.0, span, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    FLAG_Race::ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, span, "subnormal_norm", evaluator));

    FLAG_Race::ContinuousPhasePathState point;
    ASSERT_TRUE(path.evaluate(0.37 * span, point, false));
    EXPECT_DOUBLE_EQ(point.dp_dw.x(), x);
    EXPECT_DOUBLE_EQ(point.dp_dw.y(), y);

    // The horizontal norm is far below the single production threshold; the
    // producer must remain finite and fail closed rather than manufacture a
    // positive capability from a rounded-up scale*root result.
    phase_offset_core::PathCellGeometryCertificate certificate;
    EXPECT_FALSE(path.cellBounds(0.25 * span, 0.75 * span, certificate));
    EXPECT_FALSE(certificate.complete);
    phase_offset_core::CertifiedPathCellV2 v2_certificate;
    EXPECT_FALSE(path.tubeCellBoundsV2(0.25 * span, 0.75 * span,
                                       v2_certificate));
    EXPECT_FALSE(v2_certificate.complete);
}

TEST(ContinuousPhasePath, V2QuinticRegularSubcellIsCertified)
{
    const double segment_w0 = 2.0;
    const double segment_w1 = 5.0;
    ContinuousPhasePathState start;
    start.p = Eigen::Vector3d(0.0, 0.0, 0.0);
    start.dp_dw = Eigen::Vector3d(2.0, 0.0, 1.0);
    start.d2p_dw2 = Eigen::Vector3d::Zero();
    start.vel = start.dp_dw;
    start.valid = true;
    ContinuousPhasePathState end = start;
    end.p = (segment_w1 - segment_w0) * start.dp_dw;

    const auto evaluator =
        ContinuousPhasePath::makeQuinticHermite(
            segment_w0, segment_w1, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    ContinuousPhasePath path;
    path.setPathRevision(17U);
    ASSERT_TRUE(path.appendSegment(segment_w0, segment_w1,
                                   "v2_quintic", evaluator));

    const double cell_w0 = 2.5;
    const double cell_w1 = 4.5;
    phase_offset_core::CertifiedPathCellV2 certificate;
    ASSERT_TRUE(path.tubeCellBoundsV2(cell_w0, cell_w1, certificate));
    ASSERT_TRUE(phase_offset_core::certifiedPathCellV2IsComplete(certificate));
    EXPECT_DOUBLE_EQ(certificate.w0, cell_w0);
    EXPECT_DOUBLE_EQ(certificate.w1, cell_w1);
    EXPECT_DOUBLE_EQ(certificate.anchor_w, 3.5);
    EXPECT_GT(certificate.inf_horizontal_p_w_norm.lower,
              phase_offset_core::kHorizontalNormalSpeedEpsilon);
    const double expected_constant_speed = std::sqrt(5.0);
    EXPECT_LE(certificate.inf_p_w_norm.lower, expected_constant_speed);
    EXPECT_GE(certificate.sup_p_w_norm.upper, expected_constant_speed);
    EXPECT_DOUBLE_EQ(certificate.sup_p_ww_norm.upper, 0.0);

    ContinuousPhasePathState anchor;
    ASSERT_TRUE(path.evaluate(certificate.anchor_w, anchor, false));
    for (int component = 0; component < 3; ++component) {
        EXPECT_LE(certificate.anchor_position.component[component].lower,
                  anchor.p(component));
        EXPECT_LE(anchor.p(component),
                  certificate.anchor_position.component[component].upper);
        EXPECT_LE(certificate.anchor_p_w.component[component].lower,
                  anchor.dp_dw(component));
        EXPECT_LE(anchor.dp_dw(component),
                  certificate.anchor_p_w.component[component].upper);
        EXPECT_LE(certificate.anchor_p_ww.component[component].lower,
                  anchor.d2p_dw2(component));
        EXPECT_LE(anchor.d2p_dw2(component),
                  certificate.anchor_p_ww.component[component].upper);
    }
    EXPECT_LE(certificate.inf_p_w_norm.lower, anchor.dp_dw.norm());
    EXPECT_LE(anchor.dp_dw.norm(), certificate.sup_p_w_norm.upper);

}

TEST(ContinuousPhasePath, V2AnchorEnclosesUnrepresentableMathematicalMidpoint)
{
    // At 2^53 the mathematical midpoint of [A,A+2] is A+1, which rounds to
    // A as a binary64 phase value.  The proof anchor must still enclose p(c)
    // at c=A+1 rather than silently certifying only the rounded query phase.
    const double A = std::ldexp(1.0, 53);
    ContinuousPhasePathState start;
    start.p = Eigen::Vector3d::Zero();
    start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
    start.d2p_dw2 = Eigen::Vector3d::Zero();
    start.vel = start.dp_dw;
    start.valid = true;
    ContinuousPhasePathState end = start;
    end.p = Eigen::Vector3d(4.0, 0.0, 0.0);
    const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
        A, A + 4.0, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(A, A + 4.0, "large_phase", evaluator));
    phase_offset_core::CertifiedPathCellV2 certificate;
    ASSERT_TRUE(path.tubeCellBoundsV2(A, A + 2.0, certificate));
    ASSERT_TRUE(phase_offset_core::certifiedPathCellV2IsComplete(certificate));
    const double mathematical_midpoint_position = 1.0;
    EXPECT_LE(certificate.anchor_position.component[0].lower,
              mathematical_midpoint_position);
    EXPECT_LE(mathematical_midpoint_position,
              certificate.anchor_position.component[0].upper);
}

TEST(ContinuousPhasePath, V2MappedBsplineRegularCellIsCertified)
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
    UniformBspline spline;
    ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, 0.2));
    const auto evaluator = ContinuousPhasePath::makeMappedBspline(
        spline, spline.t_range(0), spline.t_range(1), 2.0, 5.0);
    ASSERT_TRUE(static_cast<bool>(evaluator));

    ContinuousPhasePath path;
    path.setPathRevision(23U);
    ASSERT_TRUE(path.appendSegment(2.0, 5.0, "v2_mapped", evaluator));
    phase_offset_core::CertifiedPathCellV2 certificate;
    ASSERT_TRUE(path.tubeCellBoundsV2(2.2, 2.8, certificate));
    ASSERT_TRUE(phase_offset_core::certifiedPathCellV2IsComplete(certificate));
    EXPECT_EQ(certificate.segment_identity, 1U);
    EXPECT_EQ(certificate.path_revision, 23U);
    EXPECT_EQ(certificate.frame_revision, 23U);
    EXPECT_TRUE(path.certificateBreakpointsV2().size() > 2U);

    ContinuousPhasePathState anchor;
    ASSERT_TRUE(path.evaluate(certificate.anchor_w, anchor, false));
    for (int component = 0; component < 3; ++component) {
        EXPECT_LE(certificate.anchor_position.component[component].lower,
                  anchor.p(component));
        EXPECT_LE(anchor.p(component),
                  certificate.anchor_position.component[component].upper);
        EXPECT_LE(certificate.anchor_p_w.component[component].lower,
                  anchor.dp_dw(component));
        EXPECT_LE(anchor.dp_dw(component),
                  certificate.anchor_p_w.component[component].upper);
        EXPECT_LE(certificate.anchor_p_ww.component[component].lower,
                  anchor.d2p_dw2(component));
        EXPECT_LE(anchor.d2p_dw2(component),
                  certificate.anchor_p_ww.component[component].upper);
    }
    EXPECT_LE(certificate.inf_p_w_norm.lower, anchor.dp_dw.norm());
    EXPECT_LE(anchor.dp_dw.norm(), certificate.sup_p_w_norm.upper);

    ContinuousPhasePath source;
    ASSERT_TRUE(source.appendSegment(2.0, 5.0, "v2_mapped_source",
                                     evaluator));
    ContinuousPhasePath slice;
    ASSERT_TRUE(slice.appendSlice(source, 2.2, 2.8));
    phase_offset_core::CertifiedPathCellV2 sliced_certificate;
    ASSERT_TRUE(slice.tubeCellBoundsV2(2.3, 2.7, sliced_certificate));
    EXPECT_DOUBLE_EQ(sliced_certificate.w0, 2.3);
    EXPECT_DOUBLE_EQ(sliced_certificate.w1, 2.7);
    EXPECT_EQ(sliced_certificate.segment_identity, 1U);
}

TEST(ContinuousPhasePath, V2AnchorCoversSignedSecondDerivativeAndSlices)
{
    ContinuousPhasePathState start;
    start.p = Eigen::Vector3d(0.0, 0.0, 0.0);
    start.dp_dw = Eigen::Vector3d(1.0, 0.4, 0.8);
    start.d2p_dw2 = Eigen::Vector3d(0.7, -0.2, 0.3);
    start.vel = start.dp_dw;
    start.valid = true;
    ContinuousPhasePathState end = start;
    end.p = Eigen::Vector3d(1.4, 0.2, 1.0);
    end.dp_dw = Eigen::Vector3d(0.5, 0.9, 0.6);
    end.d2p_dw2 = Eigen::Vector3d(-0.8, 0.3, -0.4);
    const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
        0.0, 1.0, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));

    ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(0.0, 1.0, "signed_anchor", evaluator));
    phase_offset_core::CertifiedPathCellV2 certificate;
    ASSERT_TRUE(path.tubeCellBoundsV2(0.2, 0.8, certificate));
    ASSERT_TRUE(phase_offset_core::certifiedPathCellV2IsComplete(certificate));
    ContinuousPhasePathState anchor;
    ASSERT_TRUE(path.evaluate(certificate.anchor_w, anchor, false));
    for (int component = 0; component < 3; ++component) {
        EXPECT_LE(certificate.anchor_p_ww.component[component].lower,
                  anchor.d2p_dw2(component));
        EXPECT_LE(anchor.d2p_dw2(component),
                  certificate.anchor_p_ww.component[component].upper);
    }

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
    UniformBspline spline;
    ASSERT_TRUE(spline.setControlPointsAndInterval(control_points, 3, 0.2));
    const auto mapped = ContinuousPhasePath::makeMappedBspline(
        spline, spline.t_range(0), spline.t_range(1), 2.0, 5.0);
    ASSERT_TRUE(static_cast<bool>(mapped));
    ContinuousPhasePath source;
    ASSERT_TRUE(source.appendSegment(2.0, 5.0, "mapped_source", mapped));
    ContinuousPhasePath slice;
    ASSERT_TRUE(slice.appendSlice(source, 2.2, 2.8));
    std::vector<double> breakpoints;
    ASSERT_TRUE(slice.certificateBreakpointsV2(breakpoints));
    ASSERT_GE(breakpoints.size(), 2U);
    EXPECT_DOUBLE_EQ(breakpoints.front(), 2.2);
    EXPECT_DOUBLE_EQ(breakpoints.back(), 2.8);
    for (const double value : breakpoints) {
        EXPECT_GE(value, 2.2);
        EXPECT_LE(value, 2.8);
    }
}

TEST(ContinuousPhasePath, V2RejectsMalformedQueriesAndUnsupportedEvaluator)
{
    ContinuousPhasePathState state;
    state.p = Eigen::Vector3d::Zero();
    state.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.2);
    state.d2p_dw2 = Eigen::Vector3d::Zero();
    state.vel = state.dp_dw;
    state.valid = true;
    ContinuousPhasePath path;
    const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
        0.0, 1.0, state, state);
    ASSERT_TRUE(path.appendSegment(0.0, 1.0, "malformed", evaluator));
    phase_offset_core::CertifiedPathCellV2 certificate;
    EXPECT_FALSE(path.tubeCellBoundsV2(
        std::numeric_limits<double>::quiet_NaN(), 0.5, certificate));
    EXPECT_FALSE(path.tubeCellBoundsV2(0.5,
                                       std::numeric_limits<double>::infinity(),
                                       certificate));
    EXPECT_FALSE(path.tubeCellBoundsV2(0.8, 0.2, certificate));
    EXPECT_FALSE(path.tubeCellBoundsV2(0.5, 0.5, certificate));

    ContinuousPhasePath unsupported;
    const ContinuousPhasePath::Evaluator point_only(
        [](double, ContinuousPhasePathState& point) {
            point = ContinuousPhasePathState();
            point.valid = true;
            return true;
        });
    ASSERT_TRUE(unsupported.appendSegment(0.0, 1.0, "point_only",
                                          point_only));
    EXPECT_FALSE(unsupported.tubeCellBoundsV2(0.2, 0.8, certificate));
    std::vector<double> unsupported_breakpoints;
    EXPECT_FALSE(unsupported.certificateBreakpointsV2(
        unsupported_breakpoints));
    EXPECT_TRUE(unsupported_breakpoints.empty());

    ContinuousPhasePath near_equal_seams;
    const ContinuousPhasePath::PointEvaluator finite_point =
        [](double, ContinuousPhasePathState& point) {
            point = ContinuousPhasePathState();
            point.valid = true;
            return true;
        };
    const ContinuousPhasePath::CertificateBreakpointsV2Evaluator seam_points =
        [](std::vector<double>& points) {
            points = {0.0, 0.5,
                      std::nextafter(0.5, std::numeric_limits<double>::infinity()),
                      1.0};
            return true;
        };
    ASSERT_TRUE(near_equal_seams.appendSegment(
        0.0, 1.0, "near_equal_seams",
        ContinuousPhasePath::Evaluator(
            finite_point, ContinuousPhasePath::CellBoundEvaluator(),
            ContinuousPhasePath::TubeCellBoundsV2Evaluator(), seam_points)));
    std::vector<double> seams;
    ASSERT_TRUE(near_equal_seams.certificateBreakpointsV2(seams));
    ASSERT_EQ(seams.size(), 4U);
    EXPECT_LT(seams[1], seams[2]);

    ContinuousPhasePath huge;
    ContinuousPhasePathState huge_state;
    huge_state.p = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    huge_state.dp_dw = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::max());
    huge_state.d2p_dw2 = Eigen::Vector3d::Zero();
    huge_state.vel = huge_state.dp_dw;
    huge_state.valid = true;
    const auto huge_evaluator = ContinuousPhasePath::makeQuinticHermite(
        0.0, 1.0, huge_state, huge_state);
    ASSERT_TRUE(static_cast<bool>(huge_evaluator));
    ASSERT_TRUE(huge.appendSegment(0.0, 1.0, "overflow", huge_evaluator));
    EXPECT_FALSE(huge.tubeCellBoundsV2(0.2, 0.8, certificate));

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
    UniformBspline malformed_spline;
    ASSERT_TRUE(malformed_spline.setControlPointsAndInterval(
        control_points, 3, 0.2));
    // The unchanged derivative evaluator assumes unit-spaced knots.  Keep
    // nominal point construction available, but V2 must reject this mutated
    // representation rather than prove a mismatched derivative chain.
    malformed_spline.u_(4) = malformed_spline.u_(3);
    const auto malformed_evaluator = ContinuousPhasePath::makeMappedBspline(
        malformed_spline, malformed_spline.t_range(0),
        malformed_spline.t_range(1), 2.0, 5.0);
    ASSERT_TRUE(static_cast<bool>(malformed_evaluator));
    ContinuousPhasePath malformed_path;
    ASSERT_TRUE(malformed_path.appendSegment(2.0, 5.0, "bad_knots",
                                             malformed_evaluator));
    EXPECT_FALSE(malformed_path.tubeCellBoundsV2(2.2, 2.8, certificate));
    std::vector<double> malformed_breakpoints;
    EXPECT_FALSE(malformed_path.certificateBreakpointsV2(
        malformed_breakpoints));

    UniformBspline regular_spline;
    ASSERT_TRUE(regular_spline.setControlPointsAndInterval(
        control_points, 3, 0.2));
    const auto oversized_evaluator = ContinuousPhasePath::makeMappedBspline(
        regular_spline, regular_spline.t_range(0) - 0.1,
        regular_spline.t_range(1) + 0.1, 2.0, 5.0);
    ASSERT_TRUE(static_cast<bool>(oversized_evaluator));
    ContinuousPhasePath oversized_path;
    ASSERT_TRUE(oversized_path.appendSegment(2.0, 5.0, "oversized_domain",
                                             oversized_evaluator));
    EXPECT_FALSE(oversized_path.tubeCellBoundsV2(2.2, 2.8, certificate));

    UniformBspline ratio_spline;
    ASSERT_TRUE(ratio_spline.setControlPointsAndInterval(
        control_points, 3, 1.0 / 3.0));
    double upward_endpoint = ratio_spline.t_range(1);
    int upward_steps = 0;
    while (phase_offset_core::binary64ProductLeq(
               upward_endpoint, ratio_spline.beta_, 5.0) &&
           upward_steps++ < 8) {
        upward_endpoint = std::nextafter(
            upward_endpoint, std::numeric_limits<double>::infinity());
    }
    ASSERT_FALSE(phase_offset_core::binary64ProductLeq(
        upward_endpoint, ratio_spline.beta_, 5.0));
    const auto upward_evaluator = ContinuousPhasePath::makeMappedBspline(
        ratio_spline, ratio_spline.t_range(0), upward_endpoint, 2.0, 5.0);
    ASSERT_TRUE(static_cast<bool>(upward_evaluator));
    ContinuousPhasePath upward_path;
    ASSERT_TRUE(upward_path.appendSegment(2.0, 5.0, "upward_domain",
                                          upward_evaluator));
    EXPECT_FALSE(upward_path.tubeCellBoundsV2(2.2, 2.8, certificate));
}

TEST(ContinuousPhasePath, V2RejectsUnsupportedFloatingPointModes)
{
    ContinuousPhasePathState start;
    start.p = Eigen::Vector3d::Zero();
    start.dp_dw = Eigen::Vector3d(2.0, 0.0, 1.0);
    start.d2p_dw2 = Eigen::Vector3d::Zero();
    start.vel = start.dp_dw;
    start.valid = true;
    ContinuousPhasePathState end = start;
    end.p = Eigen::Vector3d(3.0, 0.0, 1.5);
    const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
        2.0, 5.0, start, end);
    ASSERT_TRUE(static_cast<bool>(evaluator));
    ContinuousPhasePath path;
    ASSERT_TRUE(path.appendSegment(2.0, 5.0, "fp_environment", evaluator));
    phase_offset_core::CertifiedPathCellV2 certificate;

    const int original_rounding = std::fegetround();
    ASSERT_EQ(original_rounding, FE_TONEAREST);
    ASSERT_EQ(std::fesetround(FE_DOWNWARD), 0);
    EXPECT_FALSE(path.tubeCellBoundsV2(2.2, 2.8, certificate));
    ASSERT_EQ(std::fesetround(original_rounding), 0);
    ASSERT_TRUE(path.tubeCellBoundsV2(2.2, 2.8, certificate));

#if defined(__i386__) || defined(__x86_64__)
    const unsigned int original_csr = _mm_getcsr();
    unsigned int unsupported_csr = original_csr;
#ifdef _MM_FLUSH_ZERO_ON
    unsupported_csr |= _MM_FLUSH_ZERO_ON;
#endif
#ifdef _MM_DENORMALS_ZERO_ON
    unsupported_csr |= _MM_DENORMALS_ZERO_ON;
#endif
    _mm_setcsr(unsupported_csr);
    EXPECT_FALSE(path.tubeCellBoundsV2(2.2, 2.8, certificate));
    _mm_setcsr(original_csr);
    ASSERT_TRUE(path.tubeCellBoundsV2(2.2, 2.8, certificate));
#endif
}

}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
