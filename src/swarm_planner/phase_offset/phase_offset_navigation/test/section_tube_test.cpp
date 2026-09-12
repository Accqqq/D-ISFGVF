#include <gtest/gtest.h>

#include "phase_offset_navigation/section_tube.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace {

using phase_offset_navigation::SectionBox;
using phase_offset_navigation::SectionBuildConfig;
using phase_offset_navigation::SectionBuildInput;
using phase_offset_navigation::SectionCellBounds;
using phase_offset_navigation::SectionEnvironment;
using phase_offset_navigation::SectionPathSample;
using phase_offset_navigation::SectionTubeKnot;
using phase_offset_navigation::SectionTubeProfile;
using phase_offset_navigation::SectionTubeStatus;

constexpr double kPi = 3.141592653589793238462643383279502884;

SectionBox box(const Eigen::Vector3d& lower, const Eigen::Vector3d& upper) {
  SectionBox result;
  result.min = lower;
  result.max = upper;
  return result;
}

SectionEnvironment environment(const SectionBox& reference,
                               const SectionBox& obstacle_region) {
  SectionEnvironment result;
  result.available = true;
  result.reference_domain = reference;
  result.obstacle_region = obstacle_region;
  return result;
}

SectionBuildConfig defaultConfig() {
  SectionBuildConfig config;
  config.clearance = 0.2;
  config.half_width = 1.0;
  config.minimum_reference_speed = 0.2;
  config.max_step_w = 0.1;
  config.min_step_w = 1e-4;
  config.max_depth = 8;
  config.max_cells = 2048U;
  config.max_obstacle_checks = 200000U;
  config.max_obstacles = 20000U;
  return config;
}

SectionBuildInput lineInput(const SectionEnvironment& env,
                            const double w_start = 0.0,
                            const double w_end = 1.0) {
  SectionBuildInput input;
  input.w_start = w_start;
  input.w_end = w_end;
  input.environment = env;
  input.point_query = [](double w, SectionPathSample& sample) {
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    sample.p_ww.setZero();
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.N_w.setZero();
    return true;
  };
  input.bounds_query = [](double, double, SectionCellBounds& bounds) {
    bounds = SectionCellBounds();
    bounds.horizontal_speed_lower = 1.0;
    bounds.path_speed_lower = 1.0;
    bounds.abs_p_ww.setZero();
    bounds.abs_N_ww.setZero();
    bounds.valid = true;
    return true;
  };
  return input;
}

SectionBuildInput circleInput(const SectionEnvironment& env,
                              const double w_start = 0.0,
                              const double w_end = 1.0) {
  SectionBuildInput input;
  input.w_start = w_start;
  input.w_end = w_end;
  input.environment = env;
  input.point_query = [](double w, SectionPathSample& sample) {
    sample = SectionPathSample();
    const double c = std::cos(w);
    const double s = std::sin(w);
    sample.p = Eigen::Vector3d(c, s, 1.0);
    sample.p_w = Eigen::Vector3d(-s, c, 0.0);
    sample.p_ww = Eigen::Vector3d(-c, -s, 0.0);
    sample.N = Eigen::Vector3d(-c, -s, 0.0);
    sample.N_w = Eigen::Vector3d(s, -c, 0.0);
    return true;
  };
  input.bounds_query = [](double, double, SectionCellBounds& bounds) {
    bounds = SectionCellBounds();
    bounds.horizontal_speed_lower = 1.0;
    bounds.path_speed_lower = 1.0;
    bounds.abs_p_ww = Eigen::Vector3d(1.0, 1.0, 0.0);
    bounds.abs_N_ww = Eigen::Vector3d(1.0, 1.0, 0.0);
    bounds.valid = true;
    return true;
  };
  return input;
}

double pointBoxDistance(const Eigen::Vector3d& point,
                        const SectionBox& obstacle) {
  double squared = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    double gap = 0.0;
    if (point(axis) < obstacle.min(axis)) gap = obstacle.min(axis) - point(axis);
    if (point(axis) > obstacle.max(axis)) gap = point(axis) - obstacle.max(axis);
    squared += gap * gap;
  }
  return std::sqrt(squared);
}

void fillCanonicalHorizontalFrame(SectionPathSample& sample) {
  const double horizontal_speed =
      std::hypot(sample.p_w.x(), sample.p_w.y());
  sample.N = Eigen::Vector3d(
      -sample.p_w.y() / horizontal_speed,
      sample.p_w.x() / horizontal_speed, 0.0);
  const Eigen::Vector3d horizontal_acceleration(
      -sample.p_ww.y(), sample.p_ww.x(), 0.0);
  sample.N_w = (horizontal_acceleration - sample.N *
                sample.N.dot(horizontal_acceleration)) / horizontal_speed;
}

void expectProfileSafety(const SectionBuildInput& input,
                         const SectionTubeProfile& profile,
                         const SectionBuildConfig& config) {
  ASSERT_TRUE(profile.usable);
  ASSERT_GE(profile.knots.size(), 2U);
  // Check each accepted knot cell independently.  Sampling only the global
  // prefix would miss narrow intervals introduced by structural breaks or
  // recursive subdivision (and this helper is also valid for PARTIAL output).
  for (std::size_t cell = 0U; cell + 1U < profile.knots.size(); ++cell) {
    const double cell_start = profile.knots[cell].w;
    const double cell_end = profile.knots[cell + 1U].w;
    ASSERT_LT(cell_start, cell_end);
    for (int wi = 0; wi <= 20; ++wi) {
      const double w = cell_start + (cell_end - cell_start) *
          static_cast<double>(wi) / 20.0;
      double lower = 0.0;
      double upper = 0.0;
      ASSERT_TRUE(profile.evaluate(w, lower, upper));
      SectionPathSample sample;
      ASSERT_TRUE(input.point_query(w, sample));
      const auto checkDelta = [&](const double delta) {
        const Eigen::Vector3d reference = sample.p + sample.N * delta;
        EXPECT_GE(reference.x(), input.environment.reference_domain.min.x() -
                  1e-10);
        EXPECT_LE(reference.x(), input.environment.reference_domain.max.x() +
                  1e-10);
        EXPECT_GE(reference.y(), input.environment.reference_domain.min.y() -
                  1e-10);
        EXPECT_LE(reference.y(), input.environment.reference_domain.max.y() +
                  1e-10);
        EXPECT_GE(reference.z(), input.environment.reference_domain.min.z() -
                  1e-10);
        EXPECT_LE(reference.z(), input.environment.reference_domain.max.z() +
                  1e-10);
        for (const SectionBox& obstacle : input.environment.obstacles) {
          EXPECT_GE(pointBoxDistance(reference, obstacle),
                    config.clearance - 1e-10);
        }
        const Eigen::Vector3d offset_velocity =
            sample.p_w + sample.N_w * delta;
        EXPECT_GE(offset_velocity.norm(), config.minimum_reference_speed -
                  1e-10);
      };
      for (int di = 0; di <= 10; ++di) {
        checkDelta(lower + (upper - lower) *
                   static_cast<double>(di) / 10.0);
      }
      // Uniform endpoint sampling need not hit the distinguished zero offset
      // in an asymmetric interval; check it explicitly as the required
      // connected reference branch.
      checkDelta(0.0);
    }
  }
}

void expectUsableComplete(const SectionTubeProfile& profile,
                          const double start, const double end,
                          const SectionTubeStatus expected_status =
                              SectionTubeStatus::COMPLETE) {
  EXPECT_TRUE(profile.usable);
  EXPECT_TRUE(profile.complete);
  EXPECT_EQ(profile.status, expected_status);
  EXPECT_DOUBLE_EQ(profile.valid_start, start);
  EXPECT_DOUBLE_EQ(profile.valid_end, end);
  ASSERT_FALSE(profile.knots.empty());
  EXPECT_DOUBLE_EQ(profile.knots.front().w, start);
  EXPECT_DOUBLE_EQ(profile.knots.back().w, end);
  for (const auto& knot : profile.knots) {
    EXPECT_TRUE(std::isfinite(knot.w));
    EXPECT_TRUE(std::isfinite(knot.lower));
    EXPECT_TRUE(std::isfinite(knot.upper));
    EXPECT_LE(knot.lower, 0.0);
    EXPECT_GE(knot.upper, 0.0);
  }
}

TEST(SectionTube, SpaciousStraightLineIsCompleteWithNominalWidth) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionBuildInput input = lineInput(
      environment(domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                              Eigen::Vector3d(1.4, 3.4, 1.4))));
  SectionBuildConfig config = defaultConfig();
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  expectUsableComplete(profile, 0.0, 1.0);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(profile.evaluate(0.5, lower, upper));
  EXPECT_NEAR(lower, -1.0, 1e-7);
  EXPECT_NEAR(upper, 1.0, 1e-7);
}

TEST(SectionTube, SingleWallProducesAnAsymmetricClearanceLimit) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                 Eigen::Vector3d(1.4, 3.4, 1.4)));
  env.obstacles.push_back(box(Eigen::Vector3d(-0.5, 0.9, 0.5),
                              Eigen::Vector3d(1.5, 1.4, 1.5)));
  SectionBuildConfig config = defaultConfig();
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(lineInput(env), config);
  expectUsableComplete(profile, 0.0, 1.0);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(profile.evaluate(0.5, lower, upper));
  EXPECT_NEAR(lower, -1.0, 1e-7);
  EXPECT_NEAR(upper, 0.7, 1e-7);
  EXPECT_LT(upper - lower, 2.0);
}

TEST(SectionTube, DoubleWallsKeepBothSidesAtTheRequestedClearance) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                 Eigen::Vector3d(1.4, 3.4, 1.4)));
  env.obstacles.push_back(box(Eigen::Vector3d(-0.5, 0.9, 0.5),
                              Eigen::Vector3d(1.5, 1.4, 1.5)));
  env.obstacles.push_back(box(Eigen::Vector3d(-0.5, -1.4, 0.5),
                              Eigen::Vector3d(1.5, -0.9, 1.5)));
  SectionBuildConfig config = defaultConfig();
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(lineInput(env), config);
  expectUsableComplete(profile, 0.0, 1.0);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(profile.evaluate(0.5, lower, upper));
  EXPECT_NEAR(lower, -0.7, 1e-7);
  EXPECT_NEAR(upper, 0.7, 1e-7);
}

TEST(SectionTube, HalfWidthZeroIsImmediateZeroOnly) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionBuildConfig config = defaultConfig();
  config.half_width = 0.0;
  const SectionTubeProfile profile = phase_offset_navigation::buildSectionTube(
      lineInput(environment(domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                                        Eigen::Vector3d(1.4, 3.4, 1.4)))),
      config);
  expectUsableComplete(profile, 0.0, 1.0, SectionTubeStatus::ZERO_ONLY);
  for (const auto& knot : profile.knots) {
    EXPECT_DOUBLE_EQ(knot.lower, 0.0);
    EXPECT_DOUBLE_EQ(knot.upper, 0.0);
  }
}

TEST(SectionTube, ExplicitZeroOnlyDomainDoesNotRecurse) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, 0.0, 1.0),
                                Eigen::Vector3d(1.0, 0.0, 1.0));
  SectionBuildConfig config = defaultConfig();
  config.half_width = 1.0;
  const SectionTubeProfile profile = phase_offset_navigation::buildSectionTube(
      lineInput(environment(domain, box(Eigen::Vector3d(-0.4, -0.4, 0.6),
                                        Eigen::Vector3d(1.4, 0.4, 1.4)))),
      config);
  expectUsableComplete(profile, 0.0, 1.0, SectionTubeStatus::ZERO_ONLY);
  EXPECT_EQ(profile.max_depth_reached, 0);
}

TEST(SectionTube, UnknownAndInvalidEnvironmentAreDistinct) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionBuildInput unknown = lineInput(
      environment(domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                              Eigen::Vector3d(1.4, 3.4, 1.4))));
  unknown.environment.available = false;
  SectionTubeProfile profile = phase_offset_navigation::buildSectionTube(unknown);
  EXPECT_EQ(profile.status, SectionTubeStatus::UNKNOWN_DOMAIN);
  EXPECT_FALSE(profile.usable);
  SectionBuildInput invalid = lineInput(unknown.environment);
  invalid.environment.available = true;
  invalid.environment.reference_domain.max.x() = -1.0;
  profile = phase_offset_navigation::buildSectionTube(invalid);
  EXPECT_EQ(profile.status, SectionTubeStatus::INVALID_INPUT);
  EXPECT_TRUE(profile.knots.empty());
}

TEST(SectionTube, SubUlpClearanceCannotDisappearFromEnvironmentContract) {
  const double tiny = std::numeric_limits<double>::denorm_min();
  const SectionBox domain = box(Eigen::Vector3d(1.0, -1.0, 0.0),
                                Eigen::Vector3d(2.0, 1.0, 0.0));
  SectionEnvironment tight = environment(
      domain, box(Eigen::Vector3d(1.0, -1.0, -1.0),
                 Eigen::Vector3d(2.0, 1.0, 1.0)));
  SectionBuildInput input = lineInput(tight);
  input.point_query = [](double w, SectionPathSample& sample) {
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(1.0 + w, 0.0, 0.0);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    return true;
  };
  SectionBuildConfig config = defaultConfig();
  config.clearance = tiny;
  SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_EQ(profile.status, SectionTubeStatus::INVALID_INPUT);
  EXPECT_FALSE(profile.usable);

  const double x_min = std::nextafter(
      domain.min.x(), -std::numeric_limits<double>::infinity());
  const double x_max = std::nextafter(
      domain.max.x(), std::numeric_limits<double>::infinity());
  const double y_min = std::nextafter(
      domain.min.y(), -std::numeric_limits<double>::infinity());
  const double y_max = std::nextafter(
      domain.max.y(), std::numeric_limits<double>::infinity());
  SectionEnvironment one_ulp = environment(
      domain, box(Eigen::Vector3d(x_min, y_min, -1.0),
                 Eigen::Vector3d(x_max, y_max, 1.0)));
  input.environment = one_ulp;
  profile = phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_TRUE(profile.usable);
}

TEST(SectionTube, MissingCallbacksAndInvalidBreakpointsFailClosed) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionBuildInput input = lineInput(
      environment(domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                              Eigen::Vector3d(1.4, 3.4, 1.4))));
  input.point_query = nullptr;
  SectionTubeProfile profile = phase_offset_navigation::buildSectionTube(input);
  EXPECT_EQ(profile.status, SectionTubeStatus::INVALID_INPUT);
  input = lineInput(input.environment);
  input.structural_breakpoints = {0.4, 0.4};
  profile = phase_offset_navigation::buildSectionTube(input);
  EXPECT_EQ(profile.status, SectionTubeStatus::INVALID_INPUT);
  input.structural_breakpoints = {0.0, 0.5, 1.0};
  profile = phase_offset_navigation::buildSectionTube(input, defaultConfig());
  expectUsableComplete(profile, 0.0, 1.0);
  EXPECT_TRUE(std::any_of(profile.knots.begin(), profile.knots.end(),
                          [](const auto& knot) { return knot.w == 0.5; }));
}

TEST(SectionTube, CurvedPathLosesInsideRegularityButKeepsOutsideCapacity) {
  const SectionBox domain = box(Eigen::Vector3d(-3.0, -3.0, 1.0),
                                Eigen::Vector3d(3.0, 3.0, 1.0));
  const SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-3.4, -3.4, 0.6),
                 Eigen::Vector3d(3.4, 3.4, 1.4)));
  SectionBuildConfig config = defaultConfig();
  config.half_width = 1.0;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(circleInput(env), config);
  expectUsableComplete(profile, 0.0, 1.0);
  expectProfileSafety(circleInput(env), profile, config);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(profile.evaluate(0.5, lower, upper));
  EXPECT_LT(upper, 1.0);
  EXPECT_LT(lower, -0.9);
  EXPECT_GT(upper - lower, 0.0);
}

TEST(SectionTube, HorizontalZeroThicknessReferencePlaneIsSupported) {
  const SectionBox domain = box(Eigen::Vector3d(-3.0, -3.0, 1.0),
                                Eigen::Vector3d(3.0, 3.0, 1.0));
  const SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-3.4, -3.4, 0.6),
                 Eigen::Vector3d(3.4, 3.4, 1.4)));
  const SectionTubeProfile profile = phase_offset_navigation::buildSectionTube(
      circleInput(env), defaultConfig());
  expectUsableComplete(profile, 0.0, 1.0);
}

TEST(SectionTube, ThinObstacleBetweenNodesIsNotSilentlySkipped) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -2.0, 1.0),
                                Eigen::Vector3d(1.0, 2.0, 1.0));
  SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-0.4, -2.4, 0.6),
                 Eigen::Vector3d(1.4, 2.4, 1.4)));
  env.obstacles.push_back(box(Eigen::Vector3d(0.47, -0.01, 0.5),
                              Eigen::Vector3d(0.53, 0.01, 1.5)));
  SectionBuildConfig config = defaultConfig();
  config.max_step_w = 0.5;
  config.max_depth = 6;
  SectionBuildInput input = lineInput(env);
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_TRUE(profile.usable);
  EXPECT_FALSE(profile.complete);
  EXPECT_EQ(profile.status, SectionTubeStatus::PARTIAL);
  // The reference path first loses the requested 0.2 m clearance at
  // x = box.min.x() - clearance = 0.27.
  EXPECT_LT(profile.valid_end, 0.27);
  EXPECT_TRUE(profile.obstacle_checks > 0U);
  expectProfileSafety(input, profile, config);
}

TEST(SectionTube, CurvedSeedMayRequireFiniteSubdivisionInsteadOfGlobalRejection) {
  const SectionBox domain = box(Eigen::Vector3d(-3.0, -3.0, 1.0),
                                Eigen::Vector3d(3.0, 3.0, 1.0));
  SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-3.4, -3.4, 0.6),
                 Eigen::Vector3d(3.4, 3.4, 1.4)));
  env.obstacles.push_back(box(Eigen::Vector3d(0.5, -0.1, 0.5),
                              Eigen::Vector3d(0.7, 0.1, 1.5)));
  SectionBuildConfig config = defaultConfig();
  config.half_width = 0.6;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(circleInput(env, -0.5, 0.5),
                                                config);
  EXPECT_TRUE(profile.visited_cells <= config.max_cells);
  EXPECT_TRUE(profile.max_depth_reached <= config.max_depth);
  EXPECT_TRUE(profile.status != SectionTubeStatus::BUDGET_EXCEEDED);
}

TEST(SectionTube, CurvedArcAroundCenterObstacleRecoversAfterChordSplit) {
  const SectionBox domain = box(Eigen::Vector3d(-2.0, -2.0, 1.0),
                                Eigen::Vector3d(2.0, 2.0, 1.0));
  SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-2.3, -2.3, 0.7),
                 Eigen::Vector3d(2.3, 2.3, 1.3)));
  env.obstacles.push_back(box(Eigen::Vector3d(-0.1, -0.1, 0.5),
                              Eigen::Vector3d(0.1, 0.1, 1.5)));
  SectionBuildInput input = circleInput(env, -0.5 * kPi, 0.5 * kPi);
  SectionBuildConfig config = defaultConfig();
  config.clearance = 0.2;
  config.half_width = 0.1;
  config.max_step_w = 4.0;
  config.min_step_w = 1e-3;
  config.max_depth = 8;
  config.max_cells = 1024U;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  ASSERT_TRUE(profile.usable);
  EXPECT_TRUE(profile.complete);
  EXPECT_EQ(profile.status, SectionTubeStatus::COMPLETE);
  EXPECT_GT(profile.visited_cells, 1U);
  expectProfileSafety(input, profile, config);
}

TEST(SectionTube, DomainPlaneAndAdjacentSegmentSwitchIntersectAtNodes) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionBuildInput input = lineInput(
      environment(domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                              Eigen::Vector3d(1.4, 3.4, 1.4))));
  input.structural_breakpoints = {0.5};
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, defaultConfig());
  expectUsableComplete(profile, 0.0, 1.0);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(profile.evaluate(0.5, lower, upper));
  EXPECT_LE(lower, 0.0);
  EXPECT_GE(upper, 0.0);
}

TEST(SectionTube, HorizontalNormalDegeneracyIsReported) {
  SectionBuildInput input = lineInput(environment(
      box(Eigen::Vector3d(0.0, -1.0, 1.0), Eigen::Vector3d(1.0, 1.0, 1.0)),
      box(Eigen::Vector3d(-0.4, -1.4, 0.6),
          Eigen::Vector3d(1.4, 1.4, 1.4))));
  input.point_query = [](double w, SectionPathSample& sample) {
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.p_w = Eigen::Vector3d(0.0, 0.0, 1.0);
    sample.N.setZero();
    sample.N_w.setZero();
    return true;
  };
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, defaultConfig());
  EXPECT_EQ(profile.status, SectionTubeStatus::FRAME_DEGENERATE);
  EXPECT_FALSE(profile.usable);
}

TEST(SectionTube, BoundsMustCoverPointSamples) {
  SectionBuildInput input = lineInput(environment(
      box(Eigen::Vector3d(0.0, -1.0, 1.0), Eigen::Vector3d(1.0, 1.0, 1.0)),
      box(Eigen::Vector3d(-0.4, -1.4, 0.6),
          Eigen::Vector3d(1.4, 1.4, 1.4))));
  input.bounds_query = [](double, double, SectionCellBounds& bounds) {
    bounds = SectionCellBounds();
    bounds.horizontal_speed_lower = 2.0;
    bounds.path_speed_lower = 2.0;
    bounds.valid = true;
    return true;
  };
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, defaultConfig());
  EXPECT_EQ(profile.status, SectionTubeStatus::FRAME_DEGENERATE);
  EXPECT_FALSE(profile.usable);
}

TEST(SectionTube, InitialAndObstacleBudgetsFailWithoutPartialSafetyClaim) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  const SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                 Eigen::Vector3d(1.4, 3.4, 1.4)));
  SectionBuildConfig config = defaultConfig();
  config.max_cells = 2U;
  SectionTubeProfile profile = phase_offset_navigation::buildSectionTube(
      lineInput(env), config);
  EXPECT_EQ(profile.status, SectionTubeStatus::BUDGET_EXCEEDED);
  EXPECT_FALSE(profile.usable);
  EXPECT_TRUE(profile.knots.empty());

  config = defaultConfig();
  config.max_obstacle_checks = 1U;
  SectionBuildInput obstacle_input = lineInput(env);
  obstacle_input.environment.obstacles.push_back(
      box(Eigen::Vector3d(-0.5, 1.0, 0.5), Eigen::Vector3d(1.5, 1.2, 1.5)));
  obstacle_input.environment.obstacles.push_back(
      box(Eigen::Vector3d(-0.5, -1.2, 0.5), Eigen::Vector3d(1.5, -1.0, 1.5)));
  profile = phase_offset_navigation::buildSectionTube(obstacle_input, config);
  EXPECT_EQ(profile.status, SectionTubeStatus::BUDGET_EXCEEDED);
  EXPECT_TRUE(profile.knots.empty());
}

TEST(SectionTube, BudgetTerminationPreservesUsablePrefixAsPartial) {
  const SectionBox domain = box(Eigen::Vector3d(-3.0, -3.0, 1.0),
                                Eigen::Vector3d(3.0, 3.0, 1.0));
  const SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-3.4, -3.4, 0.6),
                 Eigen::Vector3d(3.4, 3.4, 1.4)));
  SectionBuildInput input = circleInput(env, 0.0, 1.0);
  SectionBuildConfig config = defaultConfig();
  config.minimum_reference_speed = 1.0;
  config.half_width = 1.0;
  config.max_step_w = 1.0;
  config.max_depth = 1;
  config.max_cells = 2U;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_TRUE(profile.usable);
  EXPECT_FALSE(profile.complete);
  EXPECT_EQ(profile.status, SectionTubeStatus::PARTIAL);
  EXPECT_EQ(profile.failure_reason, "cell_budget_exceeded");
  EXPECT_LT(profile.valid_end, input.w_end);
  EXPECT_EQ(profile.visited_cells, config.max_cells);
  expectProfileSafety(input, profile, config);
}

TEST(SectionTube, LaterFailureReturnsOnlyAContinuousPrefix) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionBuildInput input = lineInput(
      environment(domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                              Eigen::Vector3d(1.4, 3.4, 1.4))));
  input.point_query = [](double w, SectionPathSample& sample) {
    if (w >= 0.5) return false;
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    return true;
  };
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, defaultConfig());
  EXPECT_TRUE(profile.usable);
  EXPECT_FALSE(profile.complete);
  EXPECT_EQ(profile.status, SectionTubeStatus::PARTIAL);
  // The callback rejects w >= 0.5, so the unknown endpoint itself is not
  // inserted into the usable profile.  The returned prefix must remain
  // strictly before that boundary and evaluate must not extrapolate across it.
  EXPECT_LT(profile.valid_end, 0.5);
  double lower = 0.0;
  double upper = 0.0;
  EXPECT_FALSE(profile.evaluate(0.5001, lower, upper));
  EXPECT_FALSE(profile.failure_reason.empty());
}

TEST(SectionTube, ClosedCurveZeroLengthSeedSubdividesWithoutNaN) {
  const SectionBox domain = box(Eigen::Vector3d(-2.0, -2.0, 1.0),
                                Eigen::Vector3d(2.0, 2.0, 1.0));
  SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-2.3, -2.3, 0.7),
                 Eigen::Vector3d(2.3, 2.3, 1.3)));
  env.obstacles.push_back(box(Eigen::Vector3d(0.5, 0.5, 0.5),
                              Eigen::Vector3d(0.6, 0.6, 1.5)));

  SectionBuildInput input;
  input.w_start = 0.0;
  input.w_end = 1.0;
  input.environment = env;
  input.point_query = [](double w, SectionPathSample& sample) {
    sample = SectionPathSample();
    const double x = w * (1.0 - w);
    const double y = x * (2.0 * w - 1.0);
    sample.p = Eigen::Vector3d(x, y, 1.0);
    sample.p_w = Eigen::Vector3d(1.0 - 2.0 * w,
                                 -6.0 * w * w + 6.0 * w - 1.0, 0.0);
    sample.p_ww = Eigen::Vector3d(-2.0, 6.0 - 12.0 * w, 0.0);
    fillCanonicalHorizontalFrame(sample);
    return true;
  };
  input.bounds_query = [](double, double, SectionCellBounds& bounds) {
    bounds = SectionCellBounds();
    bounds.horizontal_speed_lower = 0.45;
    bounds.path_speed_lower = 0.45;
    bounds.abs_p_ww = Eigen::Vector3d(2.0, 6.0, 0.0);
    bounds.abs_N_ww = Eigen::Vector3d(640.0, 640.0, 0.0);
    bounds.valid = true;
    return true;
  };
  SectionBuildConfig config = defaultConfig();
  config.clearance = 0.01;
  config.half_width = 0.01;
  config.minimum_reference_speed = 0.4;
  config.max_step_w = 1.0;
  config.min_step_w = 1e-3;
  config.max_depth = 8;
  config.max_cells = 1024U;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_GT(profile.visited_cells, 1U);
  EXPECT_LE(profile.visited_cells, config.max_cells);
  EXPECT_LE(profile.max_depth_reached, config.max_depth);
  EXPECT_NE(profile.status, SectionTubeStatus::BUDGET_EXCEEDED);
  EXPECT_TRUE(profile.usable);
  EXPECT_TRUE(profile.complete);
  expectProfileSafety(input, profile, config);
  for (const auto& knot : profile.knots) {
    EXPECT_TRUE(std::isfinite(knot.w));
    EXPECT_TRUE(std::isfinite(knot.lower));
    EXPECT_TRUE(std::isfinite(knot.upper));
  }
}

TEST(SectionTube, ProfileOutputPassesIndependentPointBoxClearanceOracle) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                 Eigen::Vector3d(1.4, 3.4, 1.4)));
  const SectionBox obstacle = box(Eigen::Vector3d(-0.5, 1.2, 0.5),
                                  Eigen::Vector3d(1.5, 1.6, 1.5));
  env.obstacles.push_back(obstacle);
  SectionBuildConfig config = defaultConfig();
  config.clearance = 0.2;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(lineInput(env), config);
  ASSERT_TRUE(profile.usable);
  expectProfileSafety(lineInput(env), profile, config);
}

TEST(SectionTube, RegularityQuadraticSpecialCaseKeepsZeroComponent) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 0.0),
                                Eigen::Vector3d(1.0, 3.0, 2.0));
  SectionBuildInput input = lineInput(
      environment(domain, box(Eigen::Vector3d(-0.4, -3.4, -0.4),
                              Eigen::Vector3d(1.4, 3.4, 2.4))));
  input.point_query = [](double w, SectionPathSample& sample) {
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w, 0.0, 0.5 * w + 0.5);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.5);
    sample.p_ww.setZero();
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.N_w = Eigen::Vector3d::Zero();
    return true;
  };
  SectionBuildConfig config = defaultConfig();
  config.minimum_reference_speed = 0.2;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_TRUE(profile.usable);
  EXPECT_TRUE(profile.complete);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(profile.evaluate(0.5, lower, upper));
  EXPECT_LE(lower, 0.0);
  EXPECT_GE(upper, 0.0);
}

TEST(SectionTube, VerticalAccelerationNormalRateZeroKeepsGeometryCapacity) {
  const SectionBox zero_domain = box(Eigen::Vector3d(-1.0, 0.0, -1.0),
                                     Eigen::Vector3d(1.0, 0.0, 1.0));
  const SectionBox wide_domain = box(Eigen::Vector3d(-1.0, -1.0, -1.0),
                                     Eigen::Vector3d(1.0, 1.0, 1.0));
  const auto makeInput = [](const SectionBox& domain,
                            const SectionBox& obstacle_region) {
    SectionBuildInput input = lineInput(environment(domain, obstacle_region));
    input.w_start = 0.0;
    input.w_end = 0.1;
    input.point_query = [](double w, SectionPathSample& sample) {
      sample = SectionPathSample();
      sample.p = Eigen::Vector3d(w, 0.0, w * w);
      sample.p_w = Eigen::Vector3d(1.0, 0.0, 2.0 * w);
      sample.p_ww = Eigen::Vector3d(0.0, 0.0, 2.0);
      sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
      sample.N_w = Eigen::Vector3d::Zero();
      return true;
    };
    input.bounds_query = [](double, double, SectionCellBounds& bounds) {
      bounds = SectionCellBounds();
      bounds.horizontal_speed_lower = 1.0;
      bounds.path_speed_lower = 1.0;
      bounds.abs_p_ww = Eigen::Vector3d(0.0, 0.0, 2.0);
      bounds.abs_N_ww = Eigen::Vector3d::Zero();
      bounds.valid = true;
      return true;
    };
    return input;
  };
  const SectionBox zero_region = box(Eigen::Vector3d(-1.3, -0.3, -1.3),
                                     Eigen::Vector3d(1.3, 0.3, 1.3));
  const SectionBox wide_region = box(Eigen::Vector3d(-1.3, -1.3, -1.3),
                                     Eigen::Vector3d(1.3, 1.3, 1.3));
  SectionBuildConfig config = defaultConfig();
  config.minimum_reference_speed = 1.0;
  config.half_width = 1.0;
  config.max_step_w = 1.0;
  config.max_depth = 0;

  const SectionBuildInput zero_input = makeInput(zero_domain, zero_region);
  const SectionTubeProfile zero_profile =
      phase_offset_navigation::buildSectionTube(zero_input, config);
  expectUsableComplete(zero_profile, 0.0, 0.1,
                       SectionTubeStatus::ZERO_ONLY);
  EXPECT_EQ(zero_profile.visited_cells, 1U);
  expectProfileSafety(zero_input, zero_profile, config);

  const SectionBuildInput wide_input = makeInput(wide_domain, wide_region);
  const SectionTubeProfile wide_profile =
      phase_offset_navigation::buildSectionTube(wide_input, config);
  expectUsableComplete(wide_profile, 0.0, 0.1);
  EXPECT_EQ(wide_profile.visited_cells, 1U);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(wide_profile.evaluate(0.05, lower, upper));
  EXPECT_DOUBLE_EQ(lower, -1.0);
  EXPECT_DOUBLE_EQ(upper, 1.0);
  expectProfileSafety(wide_input, wide_profile, config);
}

TEST(SectionTube, ZeroSourcesMayUseAnchorRegularityWithoutSpeedLowerProof) {
  const SectionBox wide_domain = box(Eigen::Vector3d(0.0, -1.0, 1.0),
                                     Eigen::Vector3d(1.0, 1.0, 1.0));
  const SectionBox zero_domain = box(Eigen::Vector3d(0.0, 0.0, 1.0),
                                     Eigen::Vector3d(1.0, 0.0, 1.0));
  const SectionBox wide_region = box(Eigen::Vector3d(-0.4, -1.4, 0.6),
                                     Eigen::Vector3d(1.4, 1.4, 1.4));
  const SectionBox zero_region = box(Eigen::Vector3d(-0.4, -0.4, 0.6),
                                     Eigen::Vector3d(1.4, 0.4, 1.4));
  const auto makeInput = [](const SectionBox& domain,
                            const SectionBox& region) {
    SectionBuildInput input = lineInput(environment(domain, region));
    input.w_end = 0.1;
    input.bounds_query = [](double, double, SectionCellBounds& bounds) {
      bounds = SectionCellBounds();
      // Deliberately supply a valid but insufficient lower bound; the
      // midpoint quadratic regularity proof must establish zero itself.
      bounds.horizontal_speed_lower = 0.1;
      bounds.path_speed_lower = 0.1;
      bounds.valid = true;
      return true;
    };
    return input;
  };
  SectionBuildConfig config = defaultConfig();
  config.minimum_reference_speed = 0.2;
  config.max_step_w = 1.0;
  config.max_depth = 0;

  const SectionBuildInput wide_input = makeInput(wide_domain, wide_region);
  config.half_width = 0.0;
  const SectionTubeProfile wide_profile =
      phase_offset_navigation::buildSectionTube(wide_input, config);
  expectUsableComplete(wide_profile, 0.0, 0.1,
                       SectionTubeStatus::ZERO_ONLY);
  EXPECT_EQ(wide_profile.visited_cells, 1U);

  const SectionBuildInput zero_input = makeInput(zero_domain, zero_region);
  config.half_width = 1.0;
  const SectionTubeProfile zero_profile =
      phase_offset_navigation::buildSectionTube(zero_input, config);
  expectUsableComplete(zero_profile, 0.0, 0.1,
                       SectionTubeStatus::ZERO_ONLY);
  EXPECT_EQ(zero_profile.visited_cells, 1U);
}

TEST(SectionTube, RegularityQuadraticPositiveAndNegativeBranches) {
  const SectionBox domain = box(Eigen::Vector3d(-3.0, -3.0, 1.0),
                                Eigen::Vector3d(3.0, 3.0, 1.0));
  const SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-3.4, -3.4, 0.6),
                 Eigen::Vector3d(3.4, 3.4, 1.4)));
  for (const bool reverse : {false, true}) {
    SectionBuildInput input;
    input.w_start = -0.01;
    input.w_end = 0.01;
    input.environment = env;
    input.point_query = [reverse](double w, SectionPathSample& sample) {
      sample = SectionPathSample();
      const double c = std::cos(w);
      const double s = std::sin(w);
      if (!reverse) {
        sample.p = Eigen::Vector3d(c, s, 1.0);
        sample.p_w = Eigen::Vector3d(-s, c, 0.0);
        sample.p_ww = Eigen::Vector3d(-c, -s, 0.0);
        sample.N = Eigen::Vector3d(-c, -s, 0.0);
        sample.N_w = Eigen::Vector3d(s, -c, 0.0);
      } else {
        sample.p = Eigen::Vector3d(c, -s, 1.0);
        sample.p_w = Eigen::Vector3d(-s, -c, 0.0);
        sample.p_ww = Eigen::Vector3d(-c, s, 0.0);
        sample.N = Eigen::Vector3d(c, -s, 0.0);
        sample.N_w = Eigen::Vector3d(-s, -c, 0.0);
      }
      return true;
    };
    input.bounds_query = [](double, double, SectionCellBounds& bounds) {
      bounds = SectionCellBounds();
      bounds.horizontal_speed_lower = 1.0;
      bounds.path_speed_lower = 1.0;
      bounds.abs_p_ww = Eigen::Vector3d(1.0, 1.0, 0.0);
      bounds.abs_N_ww = Eigen::Vector3d(1.0, 1.0, 0.0);
      bounds.valid = true;
      return true;
    };
    SectionBuildConfig config = defaultConfig();
    config.minimum_reference_speed = 0.9;
    config.half_width = 1.0;
    config.max_step_w = 0.1;
    config.max_depth = 0;
    const SectionTubeProfile profile =
        phase_offset_navigation::buildSectionTube(input, config);
    expectUsableComplete(profile, -0.01, 0.01);
    expectProfileSafety(input, profile, config);
    double lower = 0.0;
    double upper = 0.0;
    ASSERT_TRUE(profile.evaluate(0.0, lower, upper));
    if (!reverse) {
      EXPECT_DOUBLE_EQ(lower, -1.0);
      EXPECT_LT(upper, 0.2);
    } else {
      EXPECT_GT(lower, -0.2);
      EXPECT_DOUBLE_EQ(upper, 1.0);
    }
  }
}

TEST(SectionTube, MaxDepthZeroRetainsProvenZeroFallback) {
  const SectionBox domain = box(Eigen::Vector3d(-3.0, -3.0, 1.0),
                                Eigen::Vector3d(3.0, 3.0, 1.0));
  const SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-3.4, -3.4, 0.6),
                 Eigen::Vector3d(3.4, 3.4, 1.4)));
  SectionBuildInput input = circleInput(env, -0.01, 0.01);
  SectionBuildConfig config = defaultConfig();
  config.minimum_reference_speed = 1.0;
  config.half_width = 1.0;
  config.max_step_w = 0.1;
  config.max_depth = 0;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  expectUsableComplete(profile, -0.01, 0.01, SectionTubeStatus::ZERO_ONLY);
  EXPECT_EQ(profile.visited_cells, 1U);
  for (const SectionTubeKnot& knot : profile.knots) {
    EXPECT_DOUBLE_EQ(knot.lower, 0.0);
    EXPECT_DOUBLE_EQ(knot.upper, 0.0);
  }
}

TEST(SectionTube, HelixRegularityNearDoubleRootKeepsZeroBranch) {
  const SectionBox domain = box(Eigen::Vector3d(-3.0, -3.0, 0.0),
                                Eigen::Vector3d(3.0, 3.0, 2.0));
  const SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-3.3, -3.3, -0.3),
                 Eigen::Vector3d(3.3, 3.3, 2.3)));
  SectionBuildInput input;
  input.w_start = -0.01;
  input.w_end = 0.01;
  input.environment = env;
  input.point_query = [](double w, SectionPathSample& sample) {
    sample = SectionPathSample();
    const double c = std::cos(w);
    const double s = std::sin(w);
    sample.p = Eigen::Vector3d(c, s, 1.0 + 0.5 * w);
    sample.p_w = Eigen::Vector3d(-s, c, 0.5);
    sample.p_ww = Eigen::Vector3d(-c, -s, 0.0);
    sample.N = Eigen::Vector3d(-c, -s, 0.0);
    sample.N_w = Eigen::Vector3d(s, -c, 0.0);
    return true;
  };
  input.bounds_query = [](double, double, SectionCellBounds& bounds) {
    bounds = SectionCellBounds();
    bounds.horizontal_speed_lower = 1.0;
    bounds.path_speed_lower = 1.0;
    bounds.abs_p_ww = Eigen::Vector3d(1.0, 1.0, 0.0);
    bounds.abs_N_ww = Eigen::Vector3d(1.0, 1.0, 0.0);
    bounds.valid = true;
    return true;
  };
  const double variation = (std::sqrt(2.0) + std::sqrt(2.0)) * 0.02 / 2.0;
  const double center_minimum = 0.5 - variation;
  const std::vector<double> minima = {
      std::nextafter(center_minimum, -std::numeric_limits<double>::infinity()),
      center_minimum,
      std::nextafter(center_minimum, std::numeric_limits<double>::infinity())};
  for (const double minimum : minima) {
    SectionBuildConfig config = defaultConfig();
    config.minimum_reference_speed = minimum;
    config.half_width = 1.0;
    config.max_step_w = 0.1;
    config.max_depth = 0;
    const SectionTubeProfile profile =
        phase_offset_navigation::buildSectionTube(input, config);
    expectUsableComplete(profile, -0.01, 0.01);
    expectProfileSafety(input, profile, config);
    double lower = 0.0;
    double upper = 0.0;
    ASSERT_TRUE(profile.evaluate(0.0, lower, upper));
    EXPECT_LE(lower, 0.0);
    EXPECT_GE(upper, 0.0);
    EXPECT_TRUE(std::isfinite(lower));
    EXPECT_TRUE(std::isfinite(upper));
  }
}

TEST(SectionTube, RepeatedBuildIsDeterministicAndUsesBoundedCounters) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                 Eigen::Vector3d(1.4, 3.4, 1.4)));
  env.obstacles.push_back(box(Eigen::Vector3d(-0.5, 0.8, 0.5),
                              Eigen::Vector3d(1.5, 1.2, 1.5)));
  SectionBuildInput input = lineInput(env);
  SectionBuildConfig config = defaultConfig();
  const SectionTubeProfile first =
      phase_offset_navigation::buildSectionTube(input, config);
  const SectionTubeProfile second =
      phase_offset_navigation::buildSectionTube(input, config);
  ASSERT_EQ(first.status, second.status);
  ASSERT_EQ(first.knots.size(), second.knots.size());
  EXPECT_EQ(first.visited_cells, second.visited_cells);
  EXPECT_EQ(first.point_queries, second.point_queries);
  EXPECT_EQ(first.bounds_queries, second.bounds_queries);
  EXPECT_EQ(first.obstacle_checks, second.obstacle_checks);
  for (std::size_t i = 0U; i < first.knots.size(); ++i) {
    EXPECT_DOUBLE_EQ(first.knots[i].w, second.knots[i].w);
    EXPECT_DOUBLE_EQ(first.knots[i].lower, second.knots[i].lower);
    EXPECT_DOUBLE_EQ(first.knots[i].upper, second.knots[i].upper);
  }
}

TEST(SectionTube, CellPointCacheSeparatesAdjacentSeamCells) {
  const SectionBox domain = box(Eigen::Vector3d(-1.0, -3.0, 1.0),
                                Eigen::Vector3d(3.0, 3.0, 1.0));
  SectionBuildInput input = lineInput(
      environment(domain, box(Eigen::Vector3d(-1.4, -3.4, 0.6),
                              Eigen::Vector3d(3.4, 3.4, 1.4))), 0.0, 2.0);
  input.structural_breakpoints = {1.0};
  const auto global = input.point_query;
  int global_calls = 0;
  int cell_calls = 0;
  input.point_query = [&global, &global_calls](double w,
                                                SectionPathSample& sample) {
    ++global_calls;
    if (!global(w, sample)) return false;
    if (w > 1.0) sample.p.x() += 1e-3;
    return true;
  };
  input.cell_point_query = [&cell_calls](double w0, double w1, double w,
                                         SectionPathSample& sample) {
    ++cell_calls;
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w + (w0 >= 1.0 ? 1e-3 : 0.0), 0.0, 1.0);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    sample.p_ww.setZero();
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.N_w.setZero();
    return w1 > w0 && w >= w0 && w <= w1;
  };
  SectionBuildConfig config = defaultConfig();
  config.max_step_w = 2.0;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  ASSERT_TRUE(profile.usable) << profile.failure_reason;
  ASSERT_TRUE(profile.complete) << profile.failure_reason;
  EXPECT_EQ(global_calls, 2);
  EXPECT_EQ(cell_calls, 6);
  EXPECT_EQ(profile.point_queries, 8U);
  double lower = 0.0;
  double upper = 0.0;
  ASSERT_TRUE(profile.evaluate(1.0, lower, upper));
  EXPECT_LE(lower, 0.0);
  EXPECT_GE(upper, 0.0);
}

TEST(SectionTube, CellCallbackFailureDoesNotFallbackToGlobalPointQuery) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionBuildInput input = lineInput(
      environment(domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                              Eigen::Vector3d(1.4, 3.4, 1.4))));
  int global_calls = 0;
  const auto global = input.point_query;
  input.point_query = [&global_calls, global](double w,
                                               SectionPathSample& sample) {
    ++global_calls;
    return global(w, sample);
  };
  input.cell_point_query = [](double, double, double,
                              SectionPathSample& sample) {
    sample = SectionPathSample();
    return false;
  };
  SectionBuildConfig config = defaultConfig();
  config.max_step_w = 2.0;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_FALSE(profile.usable);
  EXPECT_FALSE(profile.complete);
  EXPECT_TRUE(profile.knots.empty());
  EXPECT_EQ(profile.status, SectionTubeStatus::GEOMETRY_UNAVAILABLE);
  EXPECT_EQ(global_calls, 0);
  EXPECT_GT(profile.point_queries, 0U);
}

TEST(SectionTube, EndpointExtraObstacleBudgetClearsAcceptedProfile) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                 Eigen::Vector3d(1.4, 3.4, 1.4)));
  env.obstacles.push_back(box(Eigen::Vector3d(10.0, 10.0, 10.0),
                              Eigen::Vector3d(11.0, 11.0, 11.0)));
  SectionBuildInput input = lineInput(env);
  const auto global = input.point_query;
  input.point_query = [global](double w, SectionPathSample& sample) {
    if (!global(w, sample)) return false;
    if (w == 0.0) {
      sample.p_ww = Eigen::Vector3d(0.0, -1.0, 0.0);
      sample.N_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    return true;
  };
  input.cell_point_query = [](double w0, double w1, double w,
                              SectionPathSample& sample) {
    if (w0 != 0.0 || w1 != 1.0 || w < w0 || w > w1) return false;
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    sample.p_ww.setZero();
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.N_w.setZero();
    return true;
  };
  SectionBuildConfig config = defaultConfig();
  config.max_step_w = 2.0;
  config.max_obstacle_checks = 1U;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_FALSE(profile.usable);
  EXPECT_FALSE(profile.complete);
  EXPECT_TRUE(profile.knots.empty());
  EXPECT_EQ(profile.status, SectionTubeStatus::BUDGET_EXCEEDED);
  EXPECT_NE(profile.failure_reason.find("endpoint"), std::string::npos);
  EXPECT_EQ(profile.obstacle_checks, 1U);
}

TEST(SectionTube, PartialTailUsesActualValidEndForEndpointComparison) {
  const SectionBox domain = box(Eigen::Vector3d(0.0, -3.0, 1.0),
                                Eigen::Vector3d(1.0, 3.0, 1.0));
  SectionEnvironment env = environment(
      domain, box(Eigen::Vector3d(-0.4, -3.4, 0.6),
                 Eigen::Vector3d(1.4, 3.4, 1.4)));
  env.obstacles.push_back(box(Eigen::Vector3d(0.75, -0.1, 0.5),
                              Eigen::Vector3d(0.85, 0.1, 1.5)));
  SectionBuildInput input = lineInput(env);
  const auto global = input.point_query;
  std::vector<double> global_queries;
  input.point_query = [global, &global_queries](double w,
                                                SectionPathSample& sample) {
    global_queries.push_back(w);
    if (!global(w, sample)) return false;
    if (w == 0.5) {
      sample.p_ww = Eigen::Vector3d(0.0, -1.0, 0.0);
      sample.N_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    return true;
  };
  input.cell_point_query = [](double w0, double w1, double w,
                              SectionPathSample& sample) {
    if (w0 < 0.0 || w1 > 1.0 || !(w1 > w0) || w < w0 || w > w1) {
      return false;
    }
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    sample.p_ww.setZero();
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.N_w.setZero();
    return true;
  };
  SectionBuildConfig config = defaultConfig();
  config.max_step_w = 0.5;
  config.max_depth = 0;
  config.minimum_reference_speed = 0.2;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  ASSERT_TRUE(profile.usable) << profile.failure_reason;
  EXPECT_FALSE(profile.complete);
  EXPECT_EQ(profile.status, SectionTubeStatus::PARTIAL);
  EXPECT_DOUBLE_EQ(profile.valid_end, 0.5);
  EXPECT_DOUBLE_EQ(profile.knots.back().w, 0.5);
  ASSERT_EQ(global_queries.size(), 2U);
  EXPECT_DOUBLE_EQ(global_queries[0], 0.0);
  EXPECT_DOUBLE_EQ(global_queries[1], 0.5);
  EXPECT_GT(profile.knots.back().lower, -1.0);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
