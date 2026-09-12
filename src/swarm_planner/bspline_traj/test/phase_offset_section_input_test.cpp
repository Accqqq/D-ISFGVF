#include <gtest/gtest.h>

#include <bspline_race/continuous_phase_normal_frame.h>
#include <bspline_race/continuous_phase_path.h>
#include <bspline_race/integration/phase_offset_section_input.h>
#include <phase_offset_navigation/section_tube.h>
#include <plan_env/local_obstacle_view.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <algorithm>
#include <utility>
#include <vector>

namespace {

using FLAG_Race::ContinuousPhaseNormalFrame;
using FLAG_Race::ContinuousPhasePath;
using FLAG_Race::ContinuousPhasePathState;
using FLAG_Race::UniformBspline;
using phase_offset_navigation::SectionBox;
using phase_offset_navigation::SectionBuildConfig;
using phase_offset_navigation::SectionBuildInput;
using phase_offset_navigation::SectionCellBounds;
using phase_offset_navigation::SectionPathSample;
using phase_offset_navigation::SectionTubeProfile;
using plan_env::LocalObstacleBox;
using plan_env::LocalObstacleView;
using plan_env::LocalObstacleViewStatus;
using plan_env::LocalObstacleVoxel;

LocalObstacleBox localBox(const Eigen::Vector3d& lower,
                          const Eigen::Vector3d& upper) {
  LocalObstacleBox box;
  box.min = lower;
  box.max = upper;
  return box;
}

struct GridFixture {
  GridFixture(const Eigen::Vector3i& count,
              const Eigen::Vector3d& origin,
              const double resolution)
      : inflated(static_cast<std::size_t>(count.x()) *
                     static_cast<std::size_t>(count.y()) *
                     static_cast<std::size_t>(count.z()), 0),
        manual(inflated.size(), 0), static_layer(inflated.size(), 0) {
    grid.origin = origin;
    grid.voxel_count = count;
    grid.resolution = resolution;
    grid.map_bounds = localBox(origin,
                                origin + resolution * count.cast<double>());
    grid.frame_id = "world";
    grid.inflated = &inflated;
    grid.manual = &manual;
    grid.static_layer = &static_layer;
  }

  std::size_t address(const Eigen::Vector3i& index) const {
    return (static_cast<std::size_t>(index.x()) *
                static_cast<std::size_t>(grid.voxel_count.y()) +
            static_cast<std::size_t>(index.y())) *
                static_cast<std::size_t>(grid.voxel_count.z()) +
            static_cast<std::size_t>(index.z());
  }

  void setInflated(const Eigen::Vector3i& index) {
    inflated[address(index)] = 1;
  }

  plan_env::LocalObstacleGridView grid;
  std::vector<char> inflated;
  std::vector<char> manual;
  std::vector<char> static_layer;
};

LocalObstacleView validView(const Eigen::Vector3d& domain_lower =
                                Eigen::Vector3d(-3.0, -3.0, 0.0),
                            const Eigen::Vector3d& domain_upper =
                                Eigen::Vector3d(5.0, 3.0, 4.0)) {
  LocalObstacleView view;
  view.status = LocalObstacleViewStatus::VALID;
  view.frame_id = "world";
  view.resolution = 0.1;
  view.reference_domain = localBox(domain_lower, domain_upper);
  view.obstacle_region = localBox(domain_lower - Eigen::Vector3d(1.0, 1.0, 1.0),
                                  domain_upper + Eigen::Vector3d(1.0, 1.0, 1.0));
  view.occupied_voxels = 0U;
  return view;
}

void addObstacle(LocalObstacleView& view, const Eigen::Vector3d& lower,
                 const Eigen::Vector3d& upper) {
  LocalObstacleVoxel voxel;
  voxel.bounds = localBox(lower, upper);
  voxel.index = Eigen::Vector3i(1, 2, 3);
  voxel.layer_mask = 1U;
  view.obstacles.push_back(voxel);
  view.occupied_voxels = view.obstacles.size();
}

std::shared_ptr<ContinuousPhasePath> straightPath(
    const double w0 = 0.0, const double w1 = 2.0) {
  ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(w0, 0.0, 1.0);
  start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  start.d2p_dw2 = Eigen::Vector3d::Zero();
  start.vel = start.dp_dw;
  start.valid = true;
  ContinuousPhasePathState end = start;
  end.p = Eigen::Vector3d(w1, 0.0, 1.0);
  const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
      w0, w1, start, end);
  std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  if (!evaluator || !path->appendSegment(w0, w1, "straight", evaluator)) {
    return std::shared_ptr<ContinuousPhasePath>();
  }
  return path;
}

std::shared_ptr<ContinuousPhasePath> liftedPath(
    const double w0 = 0.0, const double w1 = 2.0) {
  ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(w0, 0.0, 1.0);
  start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  start.d2p_dw2 = Eigen::Vector3d(0.0, 0.0, 1.0);
  start.vel = start.dp_dw;
  start.valid = true;
  ContinuousPhasePathState end = start;
  end.p = Eigen::Vector3d(w1, 0.0, 3.0);
  end.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  end.d2p_dw2 = Eigen::Vector3d(0.0, 0.0, -1.0);
  const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
      w0, w1, start, end);
  std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  if (!evaluator || !path->appendSegment(w0, w1, "lifted", evaluator)) {
    return std::shared_ptr<ContinuousPhasePath>();
  }
  return path;
}

std::shared_ptr<ContinuousPhasePath> mappedPath(const bool lifted) {
  Eigen::MatrixXd control_points(7, 3);
  control_points <<
      0.0, 0.0, 1.0,
      0.4, 0.0, lifted ? 1.1 : 1.0,
      0.8, 0.2, lifted ? 1.3 : 1.0,
      1.2, 0.5, lifted ? 1.6 : 1.0,
      1.6, 0.9, lifted ? 1.9 : 1.0,
      2.0, 1.1, lifted ? 2.1 : 1.0,
      2.4, 1.2, lifted ? 2.2 : 1.0;
  UniformBspline spline;
  if (!spline.setControlPointsAndInterval(control_points, 3, 0.2)) {
    return std::shared_ptr<ContinuousPhasePath>();
  }
  const auto evaluator = ContinuousPhasePath::makeMappedBspline(
      spline, spline.t_range(0), spline.t_range(1), 4.0, 7.0);
  std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  if (!evaluator || !path->appendSegment(4.0, 7.0,
                                         lifted ? "mapped_lifted" : "mapped",
                                         evaluator)) {
    return std::shared_ptr<ContinuousPhasePath>();
  }
  return path;
}

std::shared_ptr<ContinuousPhasePath> multiSegmentPath() {
  std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  const auto first = straightPath(0.0, 1.0);
  const auto second = straightPath(1.0, 2.0);
  if (!first || !second || first->segments().size() != 1U ||
      second->segments().size() != 1U ||
      !path->appendSegment(0.0, 1.0, "first",
                           first->segments().front().evaluate) ||
      !path->appendSegment(1.0, 2.0, "second",
                           second->segments().front().evaluate)) {
    return std::shared_ptr<ContinuousPhasePath>();
  }
  return path;
}

std::shared_ptr<ContinuousPhasePath> compositePrefixQuinticMappedPath(
    const bool lifted = false) {
  // The first two pieces form an immutable old-prefix/quintic continuation;
  // the final piece is a genuine mapped B-spline.  Its first endpoint is
  // reused as the quintic's terminal state, giving an exact C2 seam while
  // retaining each producer's native evaluator and certificate.
  Eigen::MatrixXd control_points(7, 3);
  control_points <<
      1.0, 0.0, 1.0,
      1.4, 0.0, lifted ? 1.01 : 1.0,
      1.8, 0.02, lifted ? 1.02 : 1.0,
      2.2, 0.05, lifted ? 1.04 : 1.0,
      2.6, 0.08, lifted ? 1.06 : 1.0,
      3.0, 0.10, lifted ? 1.08 : 1.0,
      3.4, 0.12, lifted ? 1.10 : 1.0;
  UniformBspline spline;
  if (!spline.setControlPointsAndInterval(control_points, 3, 0.2)) {
    return std::shared_ptr<ContinuousPhasePath>();
  }
  const auto mapped = ContinuousPhasePath::makeMappedBspline(
      spline, spline.t_range(0), spline.t_range(1), 2.0, 4.0);
  if (!mapped) return std::shared_ptr<ContinuousPhasePath>();
  ContinuousPhasePathState mapped_start;
  if (!mapped(2.0, mapped_start) || !mapped_start.valid) {
    return std::shared_ptr<ContinuousPhasePath>();
  }

  ContinuousPhasePathState prefix_start;
  ContinuousPhasePathState prefix_end;
  prefix_end = mapped_start;
  prefix_end.p -= prefix_end.dp_dw;
  prefix_end.vel = prefix_end.dp_dw;
  prefix_end.valid = true;
  prefix_start = prefix_end;
  prefix_start.p -= prefix_start.dp_dw;
  prefix_start.vel = prefix_start.dp_dw;
  prefix_start.valid = true;
  const auto prefix = ContinuousPhasePath::makeQuinticHermite(
      0.0, 1.0, prefix_start, prefix_end);
  const auto quintic = ContinuousPhasePath::makeQuinticHermite(
      1.0, 2.0, prefix_end, mapped_start);
  if (!prefix || !quintic) return std::shared_ptr<ContinuousPhasePath>();

  const std::shared_ptr<ContinuousPhasePath> old_prefix(
      new ContinuousPhasePath());
  if (!old_prefix->appendSegment(0.0, 1.0, "old_prefix", prefix)) {
    return std::shared_ptr<ContinuousPhasePath>();
  }
  const std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  if (!path->appendSlice(*old_prefix, 0.0, 1.0) ||
      !path->appendSegment(1.0, 2.0, "quintic_join", quintic) ||
      !path->appendSegment(2.0, 4.0, "mapped_tail", mapped)) {
    return std::shared_ptr<ContinuousPhasePath>();
  }
  return path;
}

bool finiteVector(const Eigen::Vector3d& value) { return value.allFinite(); }

double pointBoxDistance(const Eigen::Vector3d& point,
                        const SectionBox& box) {
  double squared = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    double gap = 0.0;
    if (point(axis) < box.min(axis)) gap = box.min(axis) - point(axis);
    else if (point(axis) > box.max(axis)) gap = point(axis) - box.max(axis);
    squared += gap * gap;
  }
  return std::sqrt(squared);
}

void expectFull3DOracle(const SectionBuildInput& input,
                        const SectionTubeProfile& profile,
                        const double clearance,
                        const double minimum_speed) {
  ASSERT_TRUE(profile.usable);
  ASSERT_TRUE(profile.complete);
  ASSERT_GE(profile.knots.size(), 2U);
  for (std::size_t cell = 0U; cell + 1U < profile.knots.size(); ++cell) {
    const double w0 = profile.knots[cell].w;
    const double w1 = profile.knots[cell + 1U].w;
    ASSERT_LT(w0, w1);
    for (int wi = 0; wi <= 8; ++wi) {
      const double w = w0 + (w1 - w0) * static_cast<double>(wi) / 8.0;
      double lower = 0.0;
      double upper = 0.0;
      ASSERT_TRUE(profile.evaluate(w, lower, upper));
      SectionPathSample sample;
      ASSERT_TRUE(input.point_query(w, sample));
      for (const double delta : {lower, 0.0, upper,
                                 0.5 * (lower + upper)}) {
        const Eigen::Vector3d reference = sample.p + sample.N * delta;
        EXPECT_TRUE((reference.array() >=
                     input.environment.reference_domain.min.array() - 1e-10).all());
        EXPECT_TRUE((reference.array() <=
                     input.environment.reference_domain.max.array() + 1e-10).all());
        for (const SectionBox& obstacle : input.environment.obstacles) {
          EXPECT_GE(pointBoxDistance(reference, obstacle),
                    clearance - 1e-10);
        }
        EXPECT_GE((sample.p_w + sample.N_w * delta).norm(),
                  minimum_speed - 1e-12);
      }
    }
  }
}

void expectDefaultInput(const SectionBuildInput& input) {
  EXPECT_DOUBLE_EQ(input.w_start, 0.0);
  EXPECT_DOUBLE_EQ(input.w_end, 0.0);
  EXPECT_FALSE(input.point_query);
  EXPECT_FALSE(input.bounds_query);
  EXPECT_FALSE(input.environment.available);
  EXPECT_TRUE(input.structural_breakpoints.empty());
}

bool makeInput(const std::shared_ptr<const ContinuousPhasePath>& path,
               const double w_start, const double w_end,
               const LocalObstacleView& view, SectionBuildInput& input,
               std::string& reason) {
  return FLAG_Race::makeSectionBuildInput(
      path, w_start, w_end, view, input, reason);
}

TEST(PhaseOffsetSectionInput, RejectsEmptyPathAndInvalidRangesFailClosed) {
  SectionBuildInput input;
  std::string reason;
  const LocalObstacleView view = validView();
  EXPECT_FALSE(makeInput(std::shared_ptr<const ContinuousPhasePath>(), 0.0,
                         1.0, view, input, reason));
  EXPECT_EQ(reason, "path_empty");
  expectDefaultInput(input);

  const auto path = straightPath();
  ASSERT_TRUE(path);
  for (const std::pair<double, double>& range : {
           std::make_pair(-1.0, 1.0), std::make_pair(0.0, 3.0),
           std::make_pair(1.0, 1.0), std::make_pair(1.0, 0.0),
           std::make_pair(std::numeric_limits<double>::quiet_NaN(), 1.0)}) {
    EXPECT_FALSE(makeInput(path, range.first, range.second, view, input,
                           reason));
    EXPECT_FALSE(reason.empty());
    expectDefaultInput(input);
  }
}

TEST(PhaseOffsetSectionInput, RejectsInvalidViewsAndCountMismatch) {
  const auto path = straightPath();
  ASSERT_TRUE(path);
  const LocalObstacleView base = validView();
  SectionBuildInput input;
  std::string reason;

  LocalObstacleView invalid = base;
  invalid.status = LocalObstacleViewStatus::UNKNOWN_DOMAIN;
  EXPECT_FALSE(makeInput(path, 0.0, 1.0, invalid, input, reason));
  EXPECT_EQ(reason, "local_obstacle_view_not_valid");
  expectDefaultInput(input);

  invalid = base;
  invalid.frame_id.clear();
  EXPECT_FALSE(makeInput(path, 0.0, 1.0, invalid, input, reason));
  EXPECT_EQ(reason, "invalid_local_obstacle_view_geometry");
  invalid = base;
  invalid.resolution = 0.0;
  EXPECT_FALSE(makeInput(path, 0.0, 1.0, invalid, input, reason));
  invalid = base;
  invalid.reference_domain.max.x() = invalid.reference_domain.min.x() - 1.0;
  EXPECT_FALSE(makeInput(path, 0.0, 1.0, invalid, input, reason));

  invalid = base;
  addObstacle(invalid, Eigen::Vector3d(1.0, 1.0, 1.0),
              Eigen::Vector3d(1.1, 1.1, 1.1));
  ++invalid.occupied_voxels;
  EXPECT_FALSE(makeInput(path, 0.0, 1.0, invalid, input, reason));
  EXPECT_EQ(reason, "occupied_voxel_count_mismatch");
  expectDefaultInput(input);

  invalid = base;
  addObstacle(invalid, Eigen::Vector3d(1.0, 1.0, 1.0),
              Eigen::Vector3d(0.9, 1.1, 1.1));
  EXPECT_FALSE(makeInput(path, 0.0, 1.0, invalid, input, reason));
  EXPECT_EQ(reason, "invalid_local_obstacle_box");
  expectDefaultInput(input);
}

TEST(PhaseOffsetSectionInput, CopiesCompleteViewWithoutInflationOrFiltering) {
  const auto path = straightPath();
  ASSERT_TRUE(path);
  LocalObstacleView view = validView();
  addObstacle(view, Eigen::Vector3d(1.0, 1.0, 1.0),
              Eigen::Vector3d(1.1, 1.2, 1.3));
  addObstacle(view, Eigen::Vector3d(-1.0, -1.2, 0.2),
              Eigen::Vector3d(-0.8, -0.9, 0.4));
  const LocalObstacleBox original_domain = view.reference_domain;
  const LocalObstacleBox original_region = view.obstacle_region;
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.0, 2.0, view, input, reason)) << reason;
  ASSERT_TRUE(input.environment.available);
  EXPECT_EQ(input.environment.obstacles.size(), view.obstacles.size());
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_DOUBLE_EQ(input.environment.reference_domain.min(axis),
                     original_domain.min(axis));
    EXPECT_DOUBLE_EQ(input.environment.reference_domain.max(axis),
                     original_domain.max(axis));
    EXPECT_DOUBLE_EQ(input.environment.obstacle_region.min(axis),
                     original_region.min(axis));
    EXPECT_DOUBLE_EQ(input.environment.obstacle_region.max(axis),
                     original_region.max(axis));
    EXPECT_DOUBLE_EQ(input.environment.obstacles[0].min(axis),
                     view.obstacles[0].bounds.min(axis));
    EXPECT_DOUBLE_EQ(input.environment.obstacles[0].max(axis),
                     view.obstacles[0].bounds.max(axis));
    EXPECT_DOUBLE_EQ(input.environment.obstacles[1].min(axis),
                     view.obstacles[1].bounds.min(axis));
    EXPECT_DOUBLE_EQ(input.environment.obstacles[1].max(axis),
                     view.obstacles[1].bounds.max(axis));
  }

  view.reference_domain.min.x() = 100.0;
  view.obstacles.clear();
  view.occupied_voxels = 0U;
  EXPECT_EQ(input.environment.obstacles.size(), 2U);
  EXPECT_DOUBLE_EQ(input.environment.reference_domain.min.x(),
                  original_domain.min.x());
}

TEST(PhaseOffsetSectionInput, AdaptsRealLocalObstacleViewAndRejectsUnknown) {
  GridFixture fixture(Eigen::Vector3i(12, 12, 8),
                      Eigen::Vector3d(-5.0, -5.0, -1.0), 1.0);
  const Eigen::Vector3i occupied_index(7, 7, 3);
  fixture.setInflated(occupied_index);
  plan_env::LocalObstacleRequest request;
  request.reference_region = localBox(Eigen::Vector3d(-1.0, -2.0, 0.0),
                                      Eigen::Vector3d(3.0, 2.0, 3.0));
  request.clearance = 0.4;
  request.known_region = fixture.grid.map_bounds;
  request.known_region_valid = true;
  const LocalObstacleView view =
      plan_env::buildLocalObstacleView(fixture.grid, request);
  ASSERT_EQ(view.status, LocalObstacleViewStatus::VALID);
  ASSERT_EQ(view.occupied_voxels, view.obstacles.size());
  ASSERT_FALSE(view.obstacles.empty());
  const auto path = straightPath(-1.0, 3.0);
  ASSERT_TRUE(path);
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, -1.0, 3.0, view, input, reason)) << reason;
  EXPECT_EQ(input.environment.obstacles.size(), view.obstacles.size());
  EXPECT_DOUBLE_EQ(input.environment.reference_domain.min.x(),
                   view.reference_domain.min.x());
  EXPECT_DOUBLE_EQ(input.environment.reference_domain.max.y(),
                   view.reference_domain.max.y());
  for (std::size_t i = 0; i < view.obstacles.size(); ++i) {
    EXPECT_EQ(input.environment.obstacles[i].min, view.obstacles[i].bounds.min);
    EXPECT_EQ(input.environment.obstacles[i].max, view.obstacles[i].bounds.max);
  }

  request.known_region_valid = false;
  const LocalObstacleView unknown =
      plan_env::buildLocalObstacleView(fixture.grid, request);
  EXPECT_EQ(unknown.status, LocalObstacleViewStatus::UNKNOWN_DOMAIN);
  EXPECT_FALSE(makeInput(path, -1.0, 3.0, unknown, input, reason));
  EXPECT_EQ(reason, "local_obstacle_view_not_valid");
  expectDefaultInput(input);
}

TEST(PhaseOffsetSectionInput, CapturedPathAndViewSurviveCallerLifetime) {
  SectionBuildInput input;
  std::string reason;
  {
    std::shared_ptr<ContinuousPhasePath> mutable_path = straightPath();
    ASSERT_TRUE(mutable_path);
    LocalObstacleView view = validView();
    addObstacle(view, Eigen::Vector3d(3.0, 3.0, 3.0),
                Eigen::Vector3d(3.1, 3.1, 3.1));
    ASSERT_TRUE(makeInput(mutable_path, 0.0, 2.0, view, input, reason))
        << reason;
    mutable_path.reset();
  }
  ASSERT_TRUE(input.point_query);
  FLAG_Race::ContinuousPhasePathState state;
  SectionPathSample sample;
  ASSERT_TRUE(input.point_query(0.5, sample));
  EXPECT_NEAR(sample.p.x(), 0.5, 1e-12);
  EXPECT_NEAR(sample.N.y(), 1.0, 1e-12);
  EXPECT_EQ(input.environment.obstacles.size(), 1U);
  SectionCellBounds bounds;
  ASSERT_TRUE(input.bounds_query(0.25, 0.75, bounds));
  (void)state;
}

TEST(PhaseOffsetSectionInput, PointAndBoundsCallbacksUseStrictRequestDomain) {
  const auto path = straightPath();
  ASSERT_TRUE(path);
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.25, 1.75, validView(), input, reason))
      << reason;

  SectionPathSample sample;
  sample.p = Eigen::Vector3d::Constant(9.0);
  sample.p_w = Eigen::Vector3d::Constant(9.0);
  sample.p_ww = Eigen::Vector3d::Constant(9.0);
  sample.N = Eigen::Vector3d::Constant(9.0);
  sample.N_w = Eigen::Vector3d::Constant(9.0);
  EXPECT_FALSE(input.point_query(0.25 - 1e-12, sample));
  EXPECT_TRUE(sample.p.isZero());
  EXPECT_TRUE(sample.p_w.isZero());
  EXPECT_TRUE(sample.p_ww.isZero());
  EXPECT_TRUE(sample.N.isZero());
  EXPECT_TRUE(sample.N_w.isZero());
  sample.p = Eigen::Vector3d::Constant(9.0);
  sample.p_w = Eigen::Vector3d::Constant(9.0);
  sample.p_ww = Eigen::Vector3d::Constant(9.0);
  sample.N = Eigen::Vector3d::Constant(9.0);
  sample.N_w = Eigen::Vector3d::Constant(9.0);
  EXPECT_FALSE(input.point_query(1.75 + 1e-12, sample));
  EXPECT_TRUE(sample.p.isZero());
  EXPECT_TRUE(sample.p_w.isZero());
  EXPECT_TRUE(sample.p_ww.isZero());
  EXPECT_TRUE(sample.N.isZero());
  EXPECT_TRUE(sample.N_w.isZero());
  EXPECT_FALSE(input.point_query(std::numeric_limits<double>::quiet_NaN(),
                                 sample));
  EXPECT_TRUE(sample.p.isZero());
  EXPECT_TRUE(sample.p_w.isZero());
  EXPECT_TRUE(sample.p_ww.isZero());
  EXPECT_TRUE(sample.N.isZero());
  EXPECT_TRUE(sample.N_w.isZero());
  ASSERT_TRUE(input.point_query(0.5, sample));
  EXPECT_NEAR(sample.p.x(), 0.5, 1e-12);

  SectionCellBounds bounds;
  bounds.horizontal_speed_lower = 9.0;
  bounds.path_speed_lower = 9.0;
  bounds.abs_p_ww = Eigen::Vector3d::Constant(9.0);
  bounds.abs_N_ww = Eigen::Vector3d::Constant(9.0);
  bounds.valid = true;
  EXPECT_FALSE(input.bounds_query(0.25 - 1e-12, 0.5, bounds));
  EXPECT_FALSE(bounds.valid);
  EXPECT_DOUBLE_EQ(bounds.horizontal_speed_lower, 0.0);
  EXPECT_DOUBLE_EQ(bounds.path_speed_lower, 0.0);
  EXPECT_TRUE(bounds.abs_p_ww.isZero());
  EXPECT_TRUE(bounds.abs_N_ww.isZero());
  bounds.horizontal_speed_lower = 9.0;
  bounds.path_speed_lower = 9.0;
  bounds.abs_p_ww = Eigen::Vector3d::Constant(9.0);
  bounds.abs_N_ww = Eigen::Vector3d::Constant(9.0);
  bounds.valid = true;
  EXPECT_FALSE(input.bounds_query(0.5, 1.75 + 1e-12, bounds));
  EXPECT_FALSE(bounds.valid);
  EXPECT_DOUBLE_EQ(bounds.horizontal_speed_lower, 0.0);
  EXPECT_DOUBLE_EQ(bounds.path_speed_lower, 0.0);
  EXPECT_TRUE(bounds.abs_p_ww.isZero());
  EXPECT_TRUE(bounds.abs_N_ww.isZero());
  ASSERT_TRUE(input.bounds_query(0.5, 1.0, bounds));
  EXPECT_TRUE(bounds.valid);
}

TEST(PhaseOffsetSectionInput, AcceptsContiguousMultiSegmentPathAndKeepsSeamAwareMode) {
  const auto path = multiSegmentPath();
  ASSERT_TRUE(path);
  ASSERT_EQ(path->segments().size(), 2U);
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.1, 0.9, validView(), input, reason)) << reason;
  EXPECT_FALSE(input.cell_point_query);
  ASSERT_TRUE(makeInput(path, 0.1, 1.9, validView(), input, reason)) << reason;
  ASSERT_TRUE(input.cell_point_query);
  ASSERT_EQ(input.structural_breakpoints.size(), 1U);
  EXPECT_DOUBLE_EQ(input.structural_breakpoints[0], 1.0);
  SectionPathSample left;
  SectionPathSample right;
  ASSERT_TRUE(input.cell_point_query(0.1, 1.0, 1.0, left));
  ASSERT_TRUE(input.cell_point_query(1.0, 1.9, 1.0, right));
  EXPECT_DOUBLE_EQ(left.p.x(), right.p.x());
}

TEST(PhaseOffsetSectionInput, CompositePrefixQuinticMappedBuildsWithSeamCells) {
  for (const bool lifted : {false, true}) {
    const auto path = compositePrefixQuinticMappedPath(lifted);
    ASSERT_TRUE(path);
    ASSERT_EQ(path->segments().size(), 3U);
    SectionBuildInput input;
    std::string reason;
    ASSERT_TRUE(makeInput(path, 0.0, 4.0, validView(), input, reason)) << reason;
    ASSERT_TRUE(input.cell_point_query);
    ASSERT_EQ(input.structural_breakpoints.size(), 4U);
    const SectionBuildConfig config;
    const SectionTubeProfile profile =
        phase_offset_navigation::buildSectionTube(input, config);
    ASSERT_TRUE(profile.usable) << profile.failure_reason;
    EXPECT_TRUE(profile.complete) << profile.failure_reason;
    EXPECT_EQ(profile.status, phase_offset_navigation::SectionTubeStatus::COMPLETE);
    EXPECT_DOUBLE_EQ(profile.valid_start, 0.0);
    EXPECT_DOUBLE_EQ(profile.valid_end, 4.0);
    expectFull3DOracle(input, profile, config.clearance,
                       config.minimum_reference_speed);
    for (const double seam : {1.0, 2.0}) {
      SectionPathSample left;
      SectionPathSample right;
      ASSERT_TRUE(input.cell_point_query(seam - 0.05, seam, seam, left));
      ASSERT_TRUE(input.cell_point_query(seam, seam + 0.05, seam, right));
      EXPECT_TRUE(finiteVector(left.p));
      EXPECT_TRUE(finiteVector(right.p));
      double lower = 0.0;
      double upper = 0.0;
      ASSERT_TRUE(profile.evaluate(seam, lower, upper));
      for (const SectionPathSample* sample : {&left, &right}) {
        for (const double delta : {lower, 0.0, upper}) {
          const Eigen::Vector3d reference = sample->p + sample->N * delta;
          EXPECT_TRUE((reference.array() >=
                       input.environment.reference_domain.min.array() - 1e-10).all());
          EXPECT_TRUE((reference.array() <=
                       input.environment.reference_domain.max.array() + 1e-10).all());
          EXPECT_GE((sample->p_w + sample->N_w * delta).norm(),
                    config.minimum_reference_speed - 1e-12);
        }
      }
    }
  }
}

TEST(PhaseOffsetSectionInput, SmallC2SeamDifferenceChecksBothGeometricSides) {
  ContinuousPhasePathState first_start;
  first_start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
  first_start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  first_start.d2p_dw2.setZero();
  first_start.vel = first_start.dp_dw;
  first_start.valid = true;
  ContinuousPhasePathState first_end = first_start;
  first_end.p.x() = 1.0;
  const auto first = ContinuousPhasePath::makeQuinticHermite(
      0.0, 1.0, first_start, first_end);
  ContinuousPhasePathState second_start = first_end;
  second_start.p.x() += 2e-7;
  ContinuousPhasePathState second_end = second_start;
  second_end.p.x() = 2.0 + 2e-7;
  const auto second = ContinuousPhasePath::makeQuinticHermite(
      1.0, 2.0, second_start, second_end);
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  const std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  ASSERT_TRUE(path->appendSegment(0.0, 1.0, "left", first));
  ASSERT_TRUE(path->appendSegment(1.0, 2.0, "right", second));
  LocalObstacleView view = validView(
      Eigen::Vector3d(-1.0, -3.0, 1.0),
      Eigen::Vector3d(4.0, 3.0, 1.0));
  addObstacle(view, Eigen::Vector3d(-1.0, 2.2, 0.5),
              Eigen::Vector3d(4.0, 2.5, 1.5));
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.0, 2.0, view, input, reason)) << reason;
  ASSERT_TRUE(input.cell_point_query);
  SectionPathSample left;
  SectionPathSample right;
  ASSERT_TRUE(input.cell_point_query(0.0, 1.0, 1.0, left));
  ASSERT_TRUE(input.cell_point_query(1.0, 2.0, 1.0, right));
  EXPECT_NE(left.p.x(), right.p.x());
  EXPECT_NEAR(std::abs(left.p.x() - right.p.x()), 2e-7, 1e-12);
  SectionBuildConfig config;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  ASSERT_TRUE(profile.usable) << profile.failure_reason;
  ASSERT_TRUE(profile.complete) << profile.failure_reason;
  expectFull3DOracle(input, profile, config.clearance,
                     config.minimum_reference_speed);

  const LocalObstacleView seam_view = validView(
      Eigen::Vector3d(1.0 + 1e-7, -3.0, 1.0),
      Eigen::Vector3d(3.0, 3.0, 1.0));
  ASSERT_TRUE(makeInput(path, 1.0, 2.0, seam_view, input, reason)) << reason;
  const SectionTubeProfile seam_profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_FALSE(seam_profile.usable);
  EXPECT_FALSE(seam_profile.complete);
  EXPECT_TRUE(seam_profile.knots.empty());
  EXPECT_NE(seam_profile.failure_reason.find("endpoint_global_geometry"),
            std::string::npos);
}

TEST(PhaseOffsetSectionInput, SeamStartGlobalLeftEndpointIsCheckedAndClearsProfile) {
  SectionBuildInput input;
  input.w_start = 1.0;
  input.w_end = 2.0;
  input.structural_breakpoints = {1.0};
  input.environment = phase_offset_navigation::SectionEnvironment();
  input.environment.available = true;
  input.environment.reference_domain = SectionBox();
  input.environment.reference_domain.min = Eigen::Vector3d(1.0, -2.0, 1.0);
  input.environment.reference_domain.max = Eigen::Vector3d(3.0, 2.0, 1.0);
  input.environment.obstacle_region.min = Eigen::Vector3d(0.0, -3.0, 0.0);
  input.environment.obstacle_region.max = Eigen::Vector3d(4.0, 3.0, 2.0);
  input.point_query = [](const double w, SectionPathSample& sample) {
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w == 1.0 ? 0.0 : w + 0.5, 0.0, 1.0);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.N_w.setZero();
    return true;
  };
  input.cell_point_query = [](const double w0, const double w1,
                              const double w, SectionPathSample& sample) {
    if (w0 != 1.0 || w1 != 2.0 || w < w0 || w > w1) return false;
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w + 0.5, 0.0, 1.0);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
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
  SectionBuildConfig config;
  config.max_step_w = 2.0;
  config.clearance = 0.2;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_FALSE(profile.usable);
  EXPECT_FALSE(profile.complete);
  EXPECT_TRUE(profile.knots.empty());
  EXPECT_EQ(profile.status,
            phase_offset_navigation::SectionTubeStatus::SEED_BLOCKED);
  EXPECT_NE(profile.failure_reason.find("endpoint_global_geometry"),
            std::string::npos);
}

TEST(PhaseOffsetSectionInput, EndpointNormalRateDifferenceIntersectsFirstKnot) {
  SectionBuildInput input;
  input.w_start = 0.0;
  input.w_end = 1.0;
  input.environment.available = true;
  input.environment.reference_domain =
      SectionBox{Eigen::Vector3d(0.0, -2.0, 1.0),
                 Eigen::Vector3d(1.0, 2.0, 1.0)};
  input.environment.obstacle_region =
      SectionBox{Eigen::Vector3d(-1.0, -3.0, 0.0),
                 Eigen::Vector3d(2.0, 3.0, 2.0)};
  input.point_query = [](const double w, SectionPathSample& sample) {
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    sample.p_ww = w == 0.0
        ? Eigen::Vector3d(0.0, -1.0, 0.0)
        : Eigen::Vector3d::Zero();
    sample.N = Eigen::Vector3d(0.0, 1.0, 0.0);
    sample.N_w = w == 0.0
        ? Eigen::Vector3d(1.0, 0.0, 0.0)
        : Eigen::Vector3d::Zero();
    return true;
  };
  input.cell_point_query = [](const double w0, const double w1,
                              const double w, SectionPathSample& sample) {
    if (w0 != 0.0 || w1 != 1.0 || w < w0 || w > w1) return false;
    sample = SectionPathSample();
    sample.p = Eigen::Vector3d(w, 0.0, 1.0);
    sample.p_w = Eigen::Vector3d(1.0, 0.0, 0.0);
    sample.p_ww = Eigen::Vector3d::Zero();
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
  SectionBuildConfig config;
  config.max_step_w = 2.0;
  config.half_width = 1.0;
  config.minimum_reference_speed = 0.2;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  ASSERT_TRUE(profile.usable) << profile.failure_reason;
  ASSERT_TRUE(profile.complete) << profile.failure_reason;
  ASSERT_FALSE(profile.knots.empty());
  EXPECT_GT(profile.knots.front().lower, -1.0);
}

TEST(PhaseOffsetSectionInput, RejectsGapOverlapAndBadSeamMultiSegmentOwners) {
  const auto segment_a = straightPath(0.0, 1.0);
  ASSERT_TRUE(segment_a);
  const auto segment_b = straightPath(1.0, 2.0);
  ASSERT_TRUE(segment_b);
  SectionBuildInput input;
  std::string reason;

  const auto makeTwoSegment = [&segment_a](const double second_start,
                                 const double second_end,
                                 const ContinuousPhasePath::Evaluator& second,
                                 std::shared_ptr<ContinuousPhasePath>& path) {
    path.reset(new ContinuousPhasePath());
    return path->appendSegment(0.0, 1.0, "a",
                               segment_a->segments().front().evaluate) &&
        path->appendSegment(second_start, second_end, "b", second);
  };

  std::shared_ptr<ContinuousPhasePath> path;
  ASSERT_TRUE(makeTwoSegment(1.1, 2.0,
                             segment_b->segments().front().evaluate, path));
  ASSERT_EQ(path->segments().size(), 2U);
  EXPECT_FALSE(makeInput(path, 0.2, 1.8, validView(), input, reason));
  EXPECT_EQ(reason, "phase_range_gap_between_segments");
  expectDefaultInput(input);

  ASSERT_TRUE(makeTwoSegment(1.0 - 1e-9, 2.0,
                             segment_b->segments().front().evaluate, path));
  ASSERT_EQ(path->segments().size(), 2U);
  EXPECT_FALSE(makeInput(path, 0.2, 1.8, validView(), input, reason));
  EXPECT_EQ(reason, "phase_range_overlap_between_segments");
  expectDefaultInput(input);

  ContinuousPhasePathState bad_start;
  bad_start.p = Eigen::Vector3d(1.0 + 2e-6, 0.0, 1.0);
  bad_start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  bad_start.d2p_dw2 = Eigen::Vector3d::Zero();
  bad_start.vel = bad_start.dp_dw;
  bad_start.valid = true;
  ContinuousPhasePathState bad_end = bad_start;
  bad_end.p.x() = 2.0 + 2e-6;
  const auto bad_evaluator = ContinuousPhasePath::makeQuinticHermite(
      1.0, 2.0, bad_start, bad_end);
  ASSERT_TRUE(bad_evaluator);
  ASSERT_TRUE(makeTwoSegment(1.0, 2.0, bad_evaluator, path));
  EXPECT_FALSE(makeInput(path, 0.2, 1.8, validView(), input, reason));
  EXPECT_EQ(reason, "segment_seam_c2_discontinuity");
  expectDefaultInput(input);
}

TEST(PhaseOffsetSectionInput, SingleSegmentAppendSliceRetainsRealBounds) {
  const auto source = straightPath(0.0, 2.0);
  ASSERT_TRUE(source);
  std::shared_ptr<ContinuousPhasePath> slice(new ContinuousPhasePath());
  ASSERT_TRUE(slice->appendSlice(*source, 0.5, 1.5));
  ASSERT_EQ(slice->segments().size(), 1U);
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(slice, 0.5, 1.5, validView(), input, reason)) << reason;
  ASSERT_EQ(input.structural_breakpoints.size(), 2U);
  EXPECT_DOUBLE_EQ(input.structural_breakpoints[0], 0.5);
  EXPECT_DOUBLE_EQ(input.structural_breakpoints[1], 1.5);
  SectionCellBounds bounds;
  EXPECT_FALSE(input.bounds_query(0.49, 0.6, bounds));
  EXPECT_FALSE(input.bounds_query(1.4, 1.51, bounds));
  EXPECT_TRUE(input.bounds_query(0.5, 1.5, bounds));
}

TEST(PhaseOffsetSectionInput, MappedPointMatchesPathAndNormalFrameQueries) {
  const auto path = mappedPath(true);
  ASSERT_TRUE(path);
  const std::shared_ptr<const ContinuousPhasePath> owner = path;
  ContinuousPhaseNormalFrame frame(owner);
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(owner, 4.0, 7.0, validView(), input, reason)) << reason;
  for (const double w : {4.0, 4.73, 5.5, 6.91, 7.0}) {
    ContinuousPhasePathState state;
    phase_offset_core::NormalFrameQuery normal;
    SectionPathSample sample;
    ASSERT_TRUE(path->evaluate(w, state, false));
    ASSERT_TRUE(frame.query(w, normal));
    ASSERT_TRUE(input.point_query(w, sample));
    EXPECT_DOUBLE_EQ((sample.p - state.p).norm(), 0.0);
    EXPECT_DOUBLE_EQ((sample.p_w - state.dp_dw).norm(), 0.0);
    EXPECT_DOUBLE_EQ((sample.p_ww - state.d2p_dw2).norm(), 0.0);
    EXPECT_DOUBLE_EQ((sample.N - normal.N).norm(), 0.0);
    EXPECT_DOUBLE_EQ((sample.N_w - normal.N_w).norm(), 0.0);
  }
  SectionPathSample sample;
  EXPECT_FALSE(input.point_query(4.0 - 1e-12, sample));
  EXPECT_TRUE(sample.p.isZero());
  EXPECT_FALSE(input.point_query(7.0 + 1e-12, sample));
  EXPECT_TRUE(sample.p.isZero());
}

TEST(PhaseOffsetSectionInput, NoBoundsPathAdaptsButBuilderDoesNotClaimZeroOnly) {
  const auto evaluator = ContinuousPhasePath::makePeriodicCircle(
      Eigen::Vector3d(0.0, 0.0, 1.0), 1.0, 4.0, 1.0);
  ASSERT_TRUE(static_cast<bool>(evaluator));
  const std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  ASSERT_TRUE(path->appendSegment(0.0, 1.0, "circle", evaluator));
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.0, 1.0, validView(), input, reason)) << reason;
  SectionCellBounds bounds;
  EXPECT_FALSE(input.bounds_query(0.2, 0.8, bounds));
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input);
  EXPECT_NE(profile.status, phase_offset_navigation::SectionTubeStatus::ZERO_ONLY);
  EXPECT_EQ(profile.status,
            phase_offset_navigation::SectionTubeStatus::GEOMETRY_UNAVAILABLE);
  EXPECT_FALSE(profile.usable);
}

TEST(PhaseOffsetSectionInput, BoundsUseComponentAndNormFallbackMinimum) {
  const auto path = mappedPath(true);
  ASSERT_TRUE(path);
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 4.0, 7.0, validView(), input, reason)) << reason;
  phase_offset_core::PathCellGeometryCertificate certificate;
  ASSERT_TRUE(path->cellBounds(4.25, 6.75, certificate));
  SectionCellBounds bounds;
  ASSERT_TRUE(input.bounds_query(4.25, 6.75, bounds));
  ASSERT_TRUE(certificate.component_acceleration_bound_complete);
  EXPECT_DOUBLE_EQ(bounds.horizontal_speed_lower,
                   certificate.inf_horizontal_p_w_norm);
  EXPECT_DOUBLE_EQ(bounds.path_speed_lower, certificate.inf_p_w_norm);
  const Eigen::Vector3d norm_bound(
      certificate.sup_horizontal_p_ww_norm,
      certificate.sup_horizontal_p_ww_norm,
      certificate.sup_p_ww_norm);
  const Eigen::Vector3d expected = certificate.sup_abs_p_ww.cwiseMin(
      norm_bound);
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.x(), expected.x());
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.y(), expected.y());
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.z(), expected.z());
  EXPECT_DOUBLE_EQ(bounds.abs_N_ww.z(), 0.0);
  EXPECT_LE(bounds.abs_p_ww.x(), norm_bound.x());
  EXPECT_LE(bounds.abs_p_ww.y(), norm_bound.y());
  EXPECT_LE(bounds.abs_p_ww.z(), norm_bound.z());
  EXPECT_TRUE(finiteVector(bounds.abs_p_ww));
  EXPECT_TRUE(finiteVector(bounds.abs_N_ww));
}

TEST(PhaseOffsetSectionInput, ScalarOnlyCertificateFallsBackConservatively) {
  ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
  start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  start.d2p_dw2 = Eigen::Vector3d(0.0, 1.0, 0.0);
  start.vel = start.dp_dw;
  start.valid = true;
  ContinuousPhasePathState end = start;
  end.p = Eigen::Vector3d(1.5, 0.8, 1.0);
  end.dp_dw = Eigen::Vector3d(0.3, 1.0, 0.0);
  end.d2p_dw2 = Eigen::Vector3d(-0.5, 0.2, 0.0);
  const ContinuousPhasePath::Evaluator source =
      ContinuousPhasePath::makeQuinticHermite(0.0, 1.5, start, end);
  ASSERT_TRUE(static_cast<bool>(source));
  const ContinuousPhasePath::PointEvaluator point =
      [source](const double w, ContinuousPhasePathState& state) {
        return source(w, state);
      };
  const ContinuousPhasePath::CellBoundEvaluator scalar_only =
      [source](const double w0, const double w1,
               phase_offset_core::PathCellGeometryCertificate& certificate) {
        if (!source.cellBounds(w0, w1, certificate)) return false;
        certificate.sup_abs_p_ww = Eigen::Vector3d::Zero();
        certificate.component_acceleration_bound_complete = false;
        return true;
      };
  const ContinuousPhasePath::Evaluator evaluator(point, scalar_only);
  const std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  ASSERT_TRUE(path->appendSegment(0.0, 1.5, "scalar_only", evaluator));

  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.0, 1.5, validView(), input, reason)) << reason;
  phase_offset_core::PathCellGeometryCertificate certificate;
  ASSERT_TRUE(path->cellBounds(0.1, 1.4, certificate));
  EXPECT_FALSE(certificate.component_acceleration_bound_complete);
  SectionCellBounds bounds;
  ASSERT_TRUE(input.bounds_query(0.1, 1.4, bounds));
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.x(),
                   certificate.sup_horizontal_p_ww_norm);
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.y(),
                   certificate.sup_horizontal_p_ww_norm);
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.z(), certificate.sup_p_ww_norm);
  EXPECT_GT(bounds.abs_p_ww.z(), 0.0);
}

TEST(PhaseOffsetSectionInput, FlatMappedTubeBuildsInZeroThicknessDomain) {
  const auto path = mappedPath(false);
  ASSERT_TRUE(path);
  const LocalObstacleView view = validView(
      Eigen::Vector3d(-2.0, -3.0, 1.0),
      Eigen::Vector3d(5.0, 3.5, 1.0));
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 4.0, 7.0, view, input, reason)) << reason;
  const SectionBuildConfig config;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_TRUE(profile.usable);
  EXPECT_TRUE(profile.complete);
  EXPECT_EQ(profile.status, phase_offset_navigation::SectionTubeStatus::COMPLETE);
  EXPECT_DOUBLE_EQ(profile.valid_start, 4.0);
  EXPECT_DOUBLE_EQ(profile.valid_end, 7.0);
  expectFull3DOracle(input, profile, config.clearance,
                     config.minimum_reference_speed);
  double min_width = std::numeric_limits<double>::infinity();
  for (const auto& knot : profile.knots) {
    min_width = std::min(min_width, knot.upper - knot.lower);
  }
  RecordProperty("min_width", std::to_string(min_width));
  RecordProperty("coverage", std::to_string(profile.valid_end - profile.valid_start));
  SectionCellBounds bounds;
  ASSERT_TRUE(input.bounds_query(4.25, 6.75, bounds));
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.z(), 0.0);
}

TEST(PhaseOffsetSectionInput, InvalidComponentCapabilityFailsClosed) {
  const auto source = liftedPath();
  ASSERT_TRUE(source);
  const ContinuousPhasePath::Evaluator original =
      source->segments().front().evaluate;
  const ContinuousPhasePath::PointEvaluator point =
      [original](const double w, ContinuousPhasePathState& state) {
        return original(w, state);
      };
  const ContinuousPhasePath::CellBoundEvaluator malformed =
      [original](const double w0, const double w1,
                 phase_offset_core::PathCellGeometryCertificate& certificate) {
        if (!original.cellBounds(w0, w1, certificate)) return false;
        certificate.component_acceleration_bound_complete = true;
        certificate.sup_abs_p_ww.x() = std::numeric_limits<double>::quiet_NaN();
        return true;
      };
  const ContinuousPhasePath::Evaluator evaluator(point, malformed);
  const std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  ASSERT_TRUE(path->appendSegment(0.0, 2.0, "malformed_component", evaluator));

  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.0, 2.0, validView(), input, reason)) << reason;
  SectionCellBounds bounds;
  EXPECT_FALSE(input.bounds_query(0.25, 1.75, bounds));
  EXPECT_FALSE(bounds.valid);
  EXPECT_TRUE(bounds.abs_p_ww.isZero());
}

TEST(PhaseOffsetSectionInput, DenseLiftedMappedBoundsContainDerivatives) {
  const auto path = mappedPath(true);
  ASSERT_TRUE(path);
  const std::shared_ptr<const ContinuousPhasePath> owner = path;
  ContinuousPhaseNormalFrame frame(owner);
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(owner, 4.25, 6.75, validView(), input, reason))
      << reason;
  phase_offset_core::PathCellGeometryCertificate certificate;
  ASSERT_TRUE(path->cellBounds(4.25, 6.75, certificate));
  SectionCellBounds bounds;
  ASSERT_TRUE(input.bounds_query(4.25, 6.75, bounds));
  ASSERT_TRUE(certificate.component_acceleration_bound_complete);
  EXPECT_GT(certificate.sup_abs_p_ww.z(), 0.0);
  for (int index = 0; index <= 500; ++index) {
    const double w = 4.25 + 2.5 * static_cast<double>(index) / 500.0;
    ContinuousPhasePathState state;
    ASSERT_TRUE(path->evaluate(w, state, false));
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_LE(std::abs(state.d2p_dw2(axis)),
                bounds.abs_p_ww(axis) + 1e-10);
    }
    EXPECT_LE(std::hypot(state.d2p_dw2.x(), state.d2p_dw2.y()),
              certificate.sup_horizontal_p_ww_norm + 1e-10);
    if (w > 4.25 + 2e-4 && w < 6.75 - 2e-4) {
      const double h = 1e-4;
      phase_offset_core::NormalFrameQuery left;
      phase_offset_core::NormalFrameQuery right;
      ASSERT_TRUE(frame.query(w - h, left));
      ASSERT_TRUE(frame.query(w + h, right));
      // Central differencing N_w estimates N_ww.  The 1e-6 relative/absolute
      // allowance covers O(h^2) truncation and binary64 subtraction at h=1e-4;
      // it is far below the conservative norm-derived component bound.
      const Eigen::Vector3d normal_ww = (right.N_w - left.N_w) / (2.0 * h);
      for (int axis = 0; axis < 3; ++axis) {
        const double tolerance = 1e-6 *
            std::max(1.0, std::abs(bounds.abs_N_ww(axis)));
        EXPECT_LE(std::abs(normal_ww(axis)),
                  bounds.abs_N_ww(axis) + tolerance);
      }
    }
  }
}

TEST(PhaseOffsetSectionInput, DenseCurvedQuinticBoundsAndNormalOracle) {
  ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
  start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  start.d2p_dw2 = Eigen::Vector3d(0.0, 1.0, 0.0);
  start.vel = start.dp_dw;
  start.valid = true;
  ContinuousPhasePathState end = start;
  end.p = Eigen::Vector3d(1.5, 0.8, 1.0);
  end.dp_dw = Eigen::Vector3d(0.3, 1.0, 0.0);
  end.d2p_dw2 = Eigen::Vector3d(-0.5, 0.2, 0.0);
  const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
      0.0, 1.5, start, end);
  const std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  ASSERT_TRUE(evaluator);
  ASSERT_TRUE(path->appendSegment(0.0, 1.5, "curved", evaluator));
  const std::shared_ptr<const ContinuousPhasePath> owner = path;
  ContinuousPhaseNormalFrame frame(owner);
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(owner, 0.1, 1.4, validView(), input, reason)) << reason;
  phase_offset_core::PathCellGeometryCertificate certificate;
  ASSERT_TRUE(path->cellBounds(0.1, 1.4, certificate));
  SectionCellBounds bounds;
  ASSERT_TRUE(input.bounds_query(0.1, 1.4, bounds));
  ASSERT_TRUE(certificate.component_acceleration_bound_complete);
  EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.z(), 0.0);
  for (int index = 0; index <= 400; ++index) {
    const double w = 0.1 + 1.3 * static_cast<double>(index) / 400.0;
    ContinuousPhasePathState state;
    ASSERT_TRUE(path->evaluate(w, state, false));
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_LE(std::abs(state.d2p_dw2(axis)),
                bounds.abs_p_ww(axis) + 1e-10);
    }
    if (index >= 2 && index <= 398) {
      const double h = 1e-4;
      phase_offset_core::NormalFrameQuery left;
      phase_offset_core::NormalFrameQuery right;
      ASSERT_TRUE(frame.query(w - h, left));
      ASSERT_TRUE(frame.query(w + h, right));
      const Eigen::Vector3d normal_ww = (right.N_w - left.N_w) / (2.0 * h);
      for (int axis = 0; axis < 3; ++axis) {
        const double tolerance = 1e-6 *
            std::max(1.0, std::abs(bounds.abs_N_ww(axis)));
        EXPECT_LE(std::abs(normal_ww(axis)),
                  bounds.abs_N_ww(axis) + tolerance);
      }
    }
  }
}

TEST(PhaseOffsetSectionInput, ConstantHorizontalDerivativeKeepsNormalSecondBoundZero) {
  const auto path = liftedPath();
  ASSERT_TRUE(path);
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.25, 1.75, validView(), input, reason)) << reason;
  phase_offset_core::PathCellGeometryCertificate certificate;
  ASSERT_TRUE(path->cellBounds(0.25, 1.75, certificate));
  EXPECT_DOUBLE_EQ(certificate.sup_horizontal_p_ww_norm, 0.0);
  EXPECT_TRUE(certificate.component_acceleration_bound_complete);
  EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.x(), 0.0);
  EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.y(), 0.0);
  EXPECT_GT(certificate.sup_abs_p_ww.z(), 0.0);
  EXPECT_GT(certificate.sup_p_ww_norm, 0.0);
  EXPECT_GT(certificate.sup_p_www_norm, 0.0);
  SectionCellBounds bounds;
  ASSERT_TRUE(input.bounds_query(0.25, 1.75, bounds));
  EXPECT_DOUBLE_EQ(bounds.abs_N_ww.x(), 0.0);
  EXPECT_DOUBLE_EQ(bounds.abs_N_ww.y(), 0.0);
  EXPECT_DOUBLE_EQ(bounds.abs_N_ww.z(), 0.0);
  EXPECT_GT(bounds.abs_p_ww.z(), 0.0);
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.x(), 0.0);
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.y(), 0.0);
}

TEST(PhaseOffsetSectionInput, LiftedTubeIsCheckedInFullThreeDimensionalDomain) {
  const auto path = liftedPath();
  ASSERT_TRUE(path);
  LocalObstacleView view = validView(Eigen::Vector3d(-2.0, -3.0, 0.0),
                                     Eigen::Vector3d(4.0, 3.5, 4.0));
  addObstacle(view, Eigen::Vector3d(3.0, 2.0, 3.0),
              Eigen::Vector3d(3.2, 2.2, 3.2));
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.0, 2.0, view, input, reason)) << reason;
  SectionBuildConfig config;
  config.clearance = 0.2;
  config.half_width = 0.6;
  config.minimum_reference_speed = 1e-8;
  config.max_step_w = 0.2;
  config.min_step_w = 1e-4;
  config.max_depth = 8;
  config.max_cells = 2048U;
  config.max_obstacle_checks = 200000U;
  config.max_obstacles = 20000U;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  expectFull3DOracle(input, profile, config.clearance,
                     config.minimum_reference_speed);
}

TEST(PhaseOffsetSectionInput, MappedLiftedTubeBuildsInThreeDimensionalDomain) {
  const auto path = mappedPath(true);
  ASSERT_TRUE(path);
  LocalObstacleView view = validView(Eigen::Vector3d(-2.0, -3.0, 0.0),
                                     Eigen::Vector3d(5.0, 3.5, 4.0));
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 4.0, 7.0, view, input, reason)) << reason;
  // Keep the production defaults for the required mapped-path acceptance
  // case, notably clearance=.4 and half_width=1.
  const SectionBuildConfig config;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_EQ(profile.status, phase_offset_navigation::SectionTubeStatus::COMPLETE);
  EXPECT_DOUBLE_EQ(profile.valid_start, 4.0);
  EXPECT_DOUBLE_EQ(profile.valid_end, 7.0);
  expectFull3DOracle(input, profile, config.clearance,
                     config.minimum_reference_speed);
}

TEST(PhaseOffsetSectionInput, PlanarCurvatureUsesExactZeroZComponent) {
  ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
  start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  start.d2p_dw2 = Eigen::Vector3d(0.0, 1.0, 0.0);
  start.vel = start.dp_dw;
  start.valid = true;
  ContinuousPhasePathState end = start;
  end.p = Eigen::Vector3d(1.5, 0.8, 1.0);
  end.dp_dw = Eigen::Vector3d(0.3, 1.0, 0.0);
  end.d2p_dw2 = Eigen::Vector3d(-0.5, 0.2, 0.0);
  const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
      0.0, 1.5, start, end);
  const std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  ASSERT_TRUE(evaluator);
  ASSERT_TRUE(path->appendSegment(0.0, 1.5, "planar", evaluator));
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.0, 1.5,
                        validView(Eigen::Vector3d(-2.0, -2.0, 1.0),
                                  Eigen::Vector3d(3.0, 3.0, 1.0)),
                        input, reason)) << reason;
  SectionCellBounds bounds;
  ASSERT_TRUE(input.bounds_query(0.1, 1.4, bounds));
  phase_offset_core::PathCellGeometryCertificate certificate;
  ASSERT_TRUE(path->cellBounds(0.1, 1.4, certificate));
  EXPECT_TRUE(certificate.component_acceleration_bound_complete);
  EXPECT_DOUBLE_EQ(certificate.sup_abs_p_ww.z(), 0.0);
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.z(), 0.0);
  EXPECT_DOUBLE_EQ(bounds.abs_N_ww.z(), 0.0);
  EXPECT_GT(bounds.abs_p_ww.x(), 0.0);
  EXPECT_GT(bounds.abs_p_ww.y(), 0.0);
}

TEST(PhaseOffsetSectionInput, PlanarZeroThicknessBuildsWithComponentBound) {
  ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
  start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  start.d2p_dw2 = Eigen::Vector3d(0.0, 1.0, 0.0);
  start.vel = start.dp_dw;
  start.valid = true;
  ContinuousPhasePathState end = start;
  end.p = Eigen::Vector3d(1.5, 0.8, 1.0);
  end.dp_dw = Eigen::Vector3d(0.3, 1.0, 0.0);
  end.d2p_dw2 = Eigen::Vector3d(-0.5, 0.2, 0.0);
  const auto evaluator = ContinuousPhasePath::makeQuinticHermite(
      0.0, 1.5, start, end);
  const std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  ASSERT_TRUE(evaluator);
  ASSERT_TRUE(path->appendSegment(0.0, 1.5, "planar_zero_z", evaluator));
  SectionBuildInput input;
  std::string reason;
  ASSERT_TRUE(makeInput(path, 0.0, 1.5,
                        validView(Eigen::Vector3d(-2.0, -2.0, 1.0),
                                  Eigen::Vector3d(3.0, 3.0, 1.0)),
                        input, reason)) << reason;
  const SectionBuildConfig config;
  const SectionTubeProfile profile =
      phase_offset_navigation::buildSectionTube(input, config);
  EXPECT_TRUE(profile.usable);
  EXPECT_TRUE(profile.complete);
  EXPECT_EQ(profile.status,
            phase_offset_navigation::SectionTubeStatus::COMPLETE);
  EXPECT_DOUBLE_EQ(profile.valid_start, 0.0);
  EXPECT_DOUBLE_EQ(profile.valid_end, 1.5);
  expectFull3DOracle(input, profile, config.clearance,
                     config.minimum_reference_speed);
  double min_width = std::numeric_limits<double>::infinity();
  for (const auto& knot : profile.knots) {
    min_width = std::min(min_width, knot.upper - knot.lower);
  }
  RecordProperty("min_width", std::to_string(min_width));
  RecordProperty("coverage", std::to_string(profile.valid_end - profile.valid_start));
  SectionCellBounds bounds;
  ASSERT_TRUE(input.bounds_query(0.1, 1.4, bounds));
  EXPECT_DOUBLE_EQ(bounds.abs_p_ww.z(), 0.0);
}

TEST(PhaseOffsetSectionInput,
     ReferenceRegionAcceptsShortStructuralIntervalAsOneCell) {
  const auto path = straightPath(0.0, 2.0);
  ASSERT_TRUE(path);
  LocalObstacleBox region;
  std::string reason;
  ASSERT_TRUE(FLAG_Race::makeSectionReferenceRegion(
      path, 0.25, 0.5, 0.0, 2.0, 1U, region, reason)) << reason;
  EXPECT_LE(region.min.x(), 0.25);
  EXPECT_GE(region.max.x(), 0.5);
  // A flat path must not acquire a denormal-thick Z enclosure merely because
  // its single structural interval is shorter than max_step_w.
  EXPECT_DOUBLE_EQ(region.min.z(), 1.0);
  EXPECT_DOUBLE_EQ(region.max.z(), 1.0);
}

TEST(PhaseOffsetSectionInput, ReferenceRegionHonorsTotalCellBudget) {
  const auto path = straightPath(0.0, 2.0);
  ASSERT_TRUE(path);
  LocalObstacleBox region;
  std::string reason;
  EXPECT_FALSE(FLAG_Race::makeSectionReferenceRegion(
      path, 0.0, 2.0, 0.0, 0.25, 7U, region, reason));
  EXPECT_EQ(reason, "reference_region_cell_budget_exceeded");
}

TEST(PhaseOffsetSectionInput,
     ReferenceRegionDoesNotAccumulateExpansionAcrossCells) {
  const auto path = straightPath(0.0, 2.0);
  ASSERT_TRUE(path);
  LocalObstacleBox one_cell;
  LocalObstacleBox many_cells;
  std::string reason;
  ASSERT_TRUE(FLAG_Race::makeSectionReferenceRegion(
      path, 0.0, 2.0, 0.0, 2.0, 1U, one_cell, reason)) << reason;
  ASSERT_TRUE(FLAG_Race::makeSectionReferenceRegion(
      path, 0.0, 2.0, 0.0, 0.25, 32U, many_cells, reason)) << reason;
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_DOUBLE_EQ(many_cells.min(axis), one_cell.min(axis));
    EXPECT_DOUBLE_EQ(many_cells.max(axis), one_cell.max(axis));
  }
}

TEST(PhaseOffsetSectionInput,
     ReferenceRegionKeepsCurvedPlanarPathExactlyFlatInZ) {
  const auto path = mappedPath(false);
  ASSERT_TRUE(path);
  LocalObstacleBox region;
  std::string reason;
  ASSERT_TRUE(FLAG_Race::makeSectionReferenceRegion(
      path, 4.0, 7.0, 0.8, 0.15, 256U, region, reason)) << reason;
  EXPECT_DOUBLE_EQ(region.min.z(), 1.0);
  EXPECT_DOUBLE_EQ(region.max.z(), 1.0);
}

TEST(PhaseOffsetSectionInput,
     ReferenceRegionContainsDenseLiftedPathNormalOffsets) {
  const auto path = liftedPath();
  ASSERT_TRUE(path);
  const std::shared_ptr<const ContinuousPhasePath> owner = path;
  ContinuousPhaseNormalFrame frame(owner);
  const double half_width = 0.6;
  LocalObstacleBox region;
  std::string reason;
  ASSERT_TRUE(FLAG_Race::makeSectionReferenceRegion(
      owner, 0.0, 2.0, half_width, 0.2, 64U, region, reason)) << reason;
  for (int index = 0; index <= 1000; ++index) {
    const double w = 2.0 * static_cast<double>(index) / 1000.0;
    ContinuousPhasePathState state;
    ASSERT_TRUE(path->evaluate(w, state, false));
    phase_offset_core::NormalFrameQuery normal;
    ASSERT_TRUE(frame.query(w, normal));
    for (const double sign : {-1.0, 1.0}) {
      const Eigen::Vector3d reference =
          state.p + normal.N * (sign * half_width);
      for (int axis = 0; axis < 3; ++axis) {
        EXPECT_GE(reference(axis), region.min(axis) - 1e-12);
        EXPECT_LE(reference(axis), region.max(axis) + 1e-12);
      }
    }
  }
}

TEST(PhaseOffsetSectionInput,
     ReferenceRegionUsesStructuralSeamAsExactCellEndpoint) {
  const auto path = multiSegmentPath();
  ASSERT_TRUE(path);
  LocalObstacleBox region;
  std::string reason;
  ASSERT_TRUE(FLAG_Race::makeSectionReferenceRegion(
      path, 0.0, 2.0, 0.0, 0.3, 16U, region, reason)) << reason;
  EXPECT_DOUBLE_EQ(region.min.z(), 1.0);
  EXPECT_DOUBLE_EQ(region.max.z(), 1.0);
}

double horizontalCurvature(const ContinuousPhasePathState& state) {
  const double horizontal_speed =
      Eigen::Vector3d::UnitZ().cross(state.dp_dw).norm();
  if (!(horizontal_speed > 1e-9)) return 0.0;
  return (state.dp_dw.x() * state.d2p_dw2.y() -
          state.dp_dw.y() * state.d2p_dw2.x()) /
      (horizontal_speed * horizontal_speed * horizontal_speed);
}

// `SectionScanEnvironment` needs the native occupancy grid facts (origin and
// voxel counts) that the plane-builder fixtures never had to provide.
LocalObstacleView scanView(const Eigen::Vector3d& lower,
                           const Eigen::Vector3d& upper) {
  LocalObstacleView view = validView(lower, upper);
  view.origin = lower;
  view.obstacle_region = localBox(lower, upper);
  view.voxel_count = Eigen::Vector3i(
      static_cast<int>(std::ceil((upper.x() - lower.x()) / view.resolution)),
      static_cast<int>(std::ceil((upper.y() - lower.y()) / view.resolution)),
      static_cast<int>(std::ceil((upper.z() - lower.z()) / view.resolution)));
  return view;
}

std::shared_ptr<ContinuousPhasePath> turningPath(const double w0 = 0.0,
                                                const double w1 = 2.0) {
  ContinuousPhasePathState start;
  start.p = Eigen::Vector3d(0.0, 0.0, 1.0);
  start.dp_dw = Eigen::Vector3d(1.0, 0.0, 0.0);
  start.d2p_dw2 = Eigen::Vector3d::Zero();
  start.vel = start.dp_dw;
  start.valid = true;
  ContinuousPhasePathState end = start;
  end.p = Eigen::Vector3d(1.2, 1.2, 1.0);
  end.dp_dw = Eigen::Vector3d(-0.2, 1.0, 0.0);
  end.vel = end.dp_dw;
  const auto evaluator =
      ContinuousPhasePath::makeQuinticHermite(w0, w1, start, end);
  std::shared_ptr<ContinuousPhasePath> path(new ContinuousPhasePath());
  if (!evaluator || !path->appendSegment(w0, w1, "turning", evaluator)) {
    return std::shared_ptr<ContinuousPhasePath>();
  }
  path->setPathRevision(1U);
  return path;
}

// M4F: the horizontal parallel-offset curve degenerates at delta -> 1/kappa.
// Only the side towards the centre of curvature collapses, so only that side is
// capped and the outer side keeps its full authority.
TEST(PhaseOffsetSectionInput, CurvatureOffsetCapsLimitOnlyTheCollapsingSide) {
  double cap_negative = 0.0;
  double cap_positive = 0.0;
  const Eigen::Vector3d p_w(1.0, 0.0, 0.0);

  // Straight path: no curvature, no cap.
  FLAG_Race::sectionCurvatureOffsetCaps(p_w, Eigen::Vector3d::Zero(), 0.35,
                                        1e-9, cap_negative, cap_positive);
  EXPECT_TRUE(std::isinf(cap_negative));
  EXPECT_TRUE(std::isinf(cap_positive));

  // kappa = +0.5 (turning towards +N): the +N side collapses, so the +N side is
  // capped at (1 - 0.35) / 0.5 = 1.3 and the -N side is untouched.
  FLAG_Race::sectionCurvatureOffsetCaps(p_w, Eigen::Vector3d(0.0, 0.5, 0.0),
                                        0.35, 1e-9, cap_negative,
                                        cap_positive);
  EXPECT_TRUE(std::isinf(cap_negative));
  EXPECT_NEAR(cap_positive, 1.3, 1e-12);
  // At the cap the regularity factor is exactly the requested floor.
  EXPECT_NEAR(1.0 - 0.5 * cap_positive, 0.35, 1e-12);

  // Mirror image: kappa = -0.5 caps the -N side instead.
  FLAG_Race::sectionCurvatureOffsetCaps(p_w, Eigen::Vector3d(0.0, -0.5, 0.0),
                                        0.35, 1e-9, cap_negative,
                                        cap_positive);
  EXPECT_NEAR(cap_negative, 1.3, 1e-12);
  EXPECT_TRUE(std::isinf(cap_positive));

  // A disabled ratio (outside [0, 1)) leaves both sides to the obstacle scan.
  FLAG_Race::sectionCurvatureOffsetCaps(p_w, Eigen::Vector3d(0.0, 0.5, 0.0),
                                        1.5, 1e-9, cap_negative,
                                        cap_positive);
  EXPECT_TRUE(std::isinf(cap_negative));
  EXPECT_TRUE(std::isinf(cap_positive));
}

// The scan itself must apply the cap: with no obstacle anywhere the obstacle
// limit is the configured half width, so a shorter knot is the curvature cap.
TEST(PhaseOffsetSectionInput, ScanCorridorIsCappedByLocalCurvature) {
  const std::shared_ptr<ContinuousPhasePath> path = turningPath();
  ASSERT_TRUE(path);
  LocalObstacleView view = scanView(Eigen::Vector3d(-2.0, -2.0, 0.0),
                                    Eigen::Vector3d(4.0, 3.5, 4.0));
  FLAG_Race::SectionScanEnvironment environment;
  std::string reason;
  ASSERT_TRUE(environment.build(view, reason)) << reason;

  FLAG_Race::SectionScanConfig config;
  config.sample_step_w = 0.05;
  config.half_width = 1.5;
  config.clearance = 0.15;
  config.min_regularity_ratio = 0.35;
  config.curvature_epsilon = 1e-9;
  const auto profile = FLAG_Race::buildSectionTubeByScan(
      path, 0.0, path->endW(), environment, config);
  ASSERT_TRUE(profile.usable) << profile.failure_reason;
  ASSERT_GE(profile.knots.size(), 2U);

  double max_curvature = 0.0;
  bool capped = false;
  for (const auto& knot : profile.knots) {
    ContinuousPhasePathState state;
    ASSERT_TRUE(path->evaluate(knot.w, state, false));
    const double curvature = horizontalCurvature(state);
    max_curvature = std::max(max_curvature, std::abs(curvature));
    ASSERT_TRUE(std::isfinite(knot.lower));
    ASSERT_TRUE(std::isfinite(knot.upper));
    if (std::abs(curvature) <= 1e-6) continue;
    const double cap =
        (1.0 - config.min_regularity_ratio) / std::abs(curvature);
    if (curvature > 0.0) {
      EXPECT_LE(knot.upper, cap + 1e-9);
      if (knot.upper < config.half_width - 1e-6) capped = true;
    } else {
      EXPECT_LE(-knot.lower, cap + 1e-9);
      if (-knot.lower < config.half_width - 1e-6) capped = true;
    }
  }
  // The fixture must actually contain curvature, otherwise the test proves
  // nothing about the cap.
  EXPECT_GT(max_curvature, 0.2);
  EXPECT_TRUE(capped);
}

// Open space with a straight path keeps exactly the obstacle-limited half
// width: the curvature term must not shrink a corridor that cannot collapse.
TEST(PhaseOffsetSectionInput, StraightScanCorridorKeepsTheConfiguredHalfWidth) {
  const std::shared_ptr<ContinuousPhasePath> path = straightPath();
  ASSERT_TRUE(path);
  path->setPathRevision(1U);
  LocalObstacleView view = scanView(Eigen::Vector3d(-3.0, -3.0, 0.0),
                                    Eigen::Vector3d(5.0, 3.0, 4.0));
  FLAG_Race::SectionScanEnvironment environment;
  std::string reason;
  ASSERT_TRUE(environment.build(view, reason)) << reason;

  FLAG_Race::SectionScanConfig config;
  config.sample_step_w = 0.05;
  config.half_width = 1.5;
  config.clearance = 0.15;
  config.min_regularity_ratio = 0.35;
  const auto profile = FLAG_Race::buildSectionTubeByScan(
      path, 0.0, path->endW(), environment, config);
  ASSERT_TRUE(profile.usable) << profile.failure_reason;
  for (const auto& knot : profile.knots) {
    // Without curvature the corridor is exactly the obstacle/step-limited
    // half width: never wider than configured, and never shrunk below it by
    // more than one native scan step.
    EXPECT_LE(-knot.lower, config.half_width + 1e-9);
    EXPECT_GE(-knot.lower, config.half_width - 0.11);
    EXPECT_LE(knot.upper, config.half_width + 1e-9);
    EXPECT_GE(knot.upper, config.half_width - 0.11);
  }
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
