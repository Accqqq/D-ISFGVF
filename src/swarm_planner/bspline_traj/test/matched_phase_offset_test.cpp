#include <gtest/gtest.h>

#include <cmath>

#include <bspline_race/phase_offset_geometry.h>

namespace
{

using FLAG_Race::ContinuousPhasePathState;
using FLAG_Race::PhaseOffsetGeometry;
using FLAG_Race::PhaseOffsetGeometryEvaluator;
using FLAG_Race::PhaseOffsetGeometryParams;

ContinuousPhasePathState makeLineState(double w)
{
  ContinuousPhasePathState s;
  s.p = Eigen::Vector3d(w, 0.0, 1.0);
  s.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  s.d2p_dw2 = Eigen::Vector3d::Zero();
  return s;
}

ContinuousPhasePathState makeCircleState(double w, double R)
{
  ContinuousPhasePathState s;
  s.p = Eigen::Vector3d(R * std::cos(w / R), R * std::sin(w / R), 1.0);
  s.dp_dw = Eigen::Vector3d(-std::sin(w / R), std::cos(w / R), 0.0);
  s.d2p_dw2 = Eigen::Vector3d(-(1.0 / R) * std::cos(w / R),
                              -(1.0 / R) * std::sin(w / R), 0.0);
  return s;
}

PhaseOffsetGeometryParams makeParams()
{
  PhaseOffsetGeometryParams p;
  p.mu_regular = 0.20;
  p.max_delta = 5.0;
  return p;
}

struct PortedResult
{
  Eigen::Vector3d v_match;
  Eigen::Vector3d v_final;
  double w_dot_final;
  double delta_dot_final;
  double epsilon_match;
  Eigen::Vector3d e_dot;  // error derivative under the port
};

PortedResult applyPort(const PhaseOffsetGeometry& g, double u_w,
                       double u_delta)
{
  PortedResult r;
  r.v_match = g.r_w * u_w + g.N * u_delta;
  r.v_final = g.base_v + r.v_match;
  r.w_dot_final = g.base_w_dot + u_w;
  r.delta_dot_final = u_delta;
  r.epsilon_match = (r.v_final - g.base_v - r.v_match).norm();
  // Perfect tracking assumption: x_dot = v_final.
  r.e_dot = r.v_final - g.r_w * r.w_dot_final - g.N * r.delta_dot_final;
  return r;
}

void expectCancellation(const PhaseOffsetGeometry& g, double u_w,
                        double u_delta)
{
  const PortedResult ported = applyPort(g, u_w, u_delta);
  // Unported error derivative: base_v - r_w * base_w_dot.
  const Eigen::Vector3d e_dot_unported = g.base_v - g.r_w * g.base_w_dot;
  EXPECT_LT(ported.epsilon_match, 1e-12);
  EXPECT_LT((ported.e_dot - e_dot_unported).norm(), 1e-9);
}

TEST(MatchedPhaseOffset, UWOnlyCancels)
{
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometry g;
  ASSERT_TRUE(ev.evaluate(makeLineState(2.0), Eigen::Vector3d(2.3, 0.1, 1.0),
                          0.3, makeParams(), g));
  expectCancellation(g, 0.4, 0.0);
}

TEST(MatchedPhaseOffset, UDeltaOnlyCancels)
{
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometry g;
  ASSERT_TRUE(ev.evaluate(makeLineState(2.0), Eigen::Vector3d(2.3, 0.1, 1.0),
                          0.3, makeParams(), g));
  expectCancellation(g, 0.0, 0.25);
}

TEST(MatchedPhaseOffset, BothPortsCancel)
{
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometry g;
  ASSERT_TRUE(ev.evaluate(makeCircleState(1.5, 5.0),
                          Eigen::Vector3d(4.0, 0.2, 1.0), 0.4, makeParams(),
                          g));
  expectCancellation(g, 0.2, -0.15);
}

TEST(MatchedPhaseOffset, ClampedPortStillCancels)
{
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometry g;
  ASSERT_TRUE(ev.evaluate(makeLineState(2.0), Eigen::Vector3d(2.3, 0.1, 1.0),
                          0.3, makeParams(), g));
  // Simulate a clamped port (e.g. slew / tube limits applied externally).
  expectCancellation(g, 0.9, -0.7);
}

TEST(MatchedPhaseOffset, PortDirectionalDecomposition)
{
  PhaseOffsetGeometryEvaluator ev;
  PhaseOffsetGeometry g;
  ASSERT_TRUE(ev.evaluate(makeLineState(2.0), Eigen::Vector3d(2.3, 0.1, 1.0),
                          0.3, makeParams(), g));
  const PortedResult p = applyPort(g, 0.5, 0.2);
  // u_w only changes the tangential projection; u_delta only the normal one.
  EXPECT_NEAR(p.v_match.dot(g.T), g.r_w.norm() * 0.5, 1e-9);
  EXPECT_NEAR(p.v_match.dot(g.N), 0.2, 1e-9);
  // The lateral port does not change the tangential projection of the base
  // velocity and vice versa.
  EXPECT_NEAR((g.base_v + p.v_match).dot(g.T) - g.base_v.dot(g.T),
              g.r_w.norm() * 0.5, 1e-9);
  EXPECT_NEAR((g.base_v + p.v_match).dot(g.N) - g.base_v.dot(g.N), 0.2,
              1e-9);
}

}  // namespace

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
