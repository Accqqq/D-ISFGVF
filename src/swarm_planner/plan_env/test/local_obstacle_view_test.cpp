#include <gtest/gtest.h>

#include "plan_env/local_obstacle_view.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <tuple>
#include <vector>

namespace {

using plan_env::LocalObstacleBox;
using plan_env::LocalObstacleGridView;
using plan_env::LocalObstacleRequest;
using plan_env::LocalObstacleView;
using plan_env::LocalObstacleViewStatus;
using plan_env::LocalObstacleVoxel;

struct GridFixture {
  GridFixture(const Eigen::Vector3i& count,
              const Eigen::Vector3d& origin = Eigen::Vector3d::Zero(),
              const double resolution = 1.0)
      : inflated(product(count), 0),
        manual(product(count), 0),
        static_layer(product(count), 0) {
    grid.origin = origin;
    grid.voxel_count = count;
    grid.resolution = resolution;
    grid.map_bounds.min = origin;
    grid.map_bounds.max = origin +
        resolution * count.cast<double>();
    grid.frame_id = "world";
    grid.inflated = &inflated;
    grid.manual = &manual;
    grid.static_layer = &static_layer;
  }

  static std::size_t product(const Eigen::Vector3i& count) {
    if ((count.array() <= 0).any()) return 0U;
    return static_cast<std::size_t>(count.x()) *
        static_cast<std::size_t>(count.y()) *
        static_cast<std::size_t>(count.z());
  }

  std::size_t address(const Eigen::Vector3i& index) const {
    return (static_cast<std::size_t>(index.x()) *
                static_cast<std::size_t>(grid.voxel_count.y()) +
            static_cast<std::size_t>(index.y())) *
                static_cast<std::size_t>(grid.voxel_count.z()) +
            static_cast<std::size_t>(index.z());
  }

  void set(const Eigen::Vector3i& index, const char inflated_value,
           const char manual_value = 0, const char static_value = 0) {
    const std::size_t i = address(index);
    inflated[i] = inflated_value;
    manual[i] = manual_value;
    static_layer[i] = static_value;
  }

  LocalObstacleRequest request(const LocalObstacleBox& reference,
                               const double clearance = 0.0) const {
    LocalObstacleRequest result;
    result.reference_region = reference;
    result.clearance = clearance;
    result.known_region = grid.map_bounds;
    result.known_region_valid = true;
    return result;
  }

  LocalObstacleGridView grid;
  std::vector<char> inflated;
  std::vector<char> manual;
  std::vector<char> static_layer;
};

LocalObstacleBox box(const Eigen::Vector3d& lower,
                     const Eigen::Vector3d& upper) {
  LocalObstacleBox result;
  result.min = lower;
  result.max = upper;
  return result;
}

bool boxesIntersect(const LocalObstacleBox& lhs, const LocalObstacleBox& rhs) {
  return (lhs.min.array() <= rhs.max.array()).all() &&
      (rhs.min.array() <= lhs.max.array()).all();
}

std::tuple<int, int, int> key(const Eigen::Vector3i& index) {
  return std::make_tuple(index.x(), index.y(), index.z());
}

// Independent small-grid oracle: it scans complete native voxel boxes and
// applies closed-box intersection directly, without using the implementation
// index-range/padding code.
std::map<std::tuple<int, int, int>, std::uint8_t> oracle(
    const GridFixture& fixture, const LocalObstacleBox& region) {
  std::map<std::tuple<int, int, int>, std::uint8_t> result;
  for (int x = 0; x < fixture.grid.voxel_count.x(); ++x) {
    for (int y = 0; y < fixture.grid.voxel_count.y(); ++y) {
      for (int z = 0; z < fixture.grid.voxel_count.z(); ++z) {
        const Eigen::Vector3i index(x, y, z);
        const Eigen::Vector3d lower = fixture.grid.origin +
            fixture.grid.resolution * index.cast<double>();
        LocalObstacleBox voxel = box(
            lower, lower + Eigen::Vector3d::Constant(fixture.grid.resolution));
        if (!boxesIntersect(voxel, region)) continue;
        const std::size_t address = fixture.address(index);
        std::uint8_t mask = 0U;
        if (fixture.inflated[address] != 0) mask |= 0x01U;
        if (fixture.manual[address] != 0) mask |= 0x02U;
        if (fixture.static_layer[address] != 0) mask |= 0x04U;
        if (mask != 0U) result[key(index)] = mask;
      }
    }
  }
  return result;
}

void expectObstacleSet(const GridFixture& fixture,
                       const LocalObstacleView& view,
                       const LocalObstacleBox& region) {
  const auto expected = oracle(fixture, region);
  std::map<std::tuple<int, int, int>, std::uint8_t> actual;
  for (const LocalObstacleVoxel& obstacle : view.obstacles) {
    ASSERT_TRUE(actual.emplace(key(obstacle.index), obstacle.layer_mask).second)
        << "duplicate output voxel";
    EXPECT_TRUE(boxesIntersect(obstacle.bounds, region));
    const Eigen::Vector3d native_min = fixture.grid.origin +
        fixture.grid.resolution * obstacle.index.cast<double>();
    EXPECT_LE(obstacle.bounds.min.x(), native_min.x());
    EXPECT_LE(obstacle.bounds.min.y(), native_min.y());
    EXPECT_LE(obstacle.bounds.min.z(), native_min.z());
    EXPECT_GE(obstacle.bounds.max.x(),
              native_min.x() + fixture.grid.resolution);
    EXPECT_GE(obstacle.bounds.max.y(),
              native_min.y() + fixture.grid.resolution);
    EXPECT_GE(obstacle.bounds.max.z(),
              native_min.z() + fixture.grid.resolution);
  }
  EXPECT_EQ(expected, actual);
  EXPECT_EQ(view.occupied_voxels, view.obstacles.size());
}

TEST(LocalObstacleView, KnownEmptyGridIsValidAndFreeListIsExplicit) {
  GridFixture fixture(Eigen::Vector3i(3, 2, 2));
  const LocalObstacleBox reference = box(Eigen::Vector3d(0.25, 0.25, 0.25),
                                         Eigen::Vector3d(1.25, 1.25, 1.25));
  const LocalObstacleView view =
      plan_env::buildLocalObstacleView(fixture.grid, fixture.request(reference));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  EXPECT_TRUE(view.obstacles.empty());
  EXPECT_EQ(view.occupied_voxels, 0U);
  EXPECT_FALSE(view.request_clipped);
  EXPECT_EQ(view.reference_domain.min, reference.min);
  EXPECT_EQ(view.reference_domain.max, reference.max);
  expectObstacleSet(fixture, view, reference);
}

TEST(LocalObstacleView, MissingKnownDomainNeverPretendsFree) {
  GridFixture fixture(Eigen::Vector3i(2, 2, 2));
  fixture.set(Eigen::Vector3i(0, 0, 0), 1);
  LocalObstacleRequest request = fixture.request(
      box(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0)));
  request.known_region_valid = false;
  const LocalObstacleView view =
      plan_env::buildLocalObstacleView(fixture.grid, request);
  EXPECT_EQ(view.status, LocalObstacleViewStatus::UNKNOWN_DOMAIN);
  EXPECT_TRUE(view.obstacles.empty());
  EXPECT_EQ(view.visited_voxels, 0U);
  EXPECT_TRUE(view.reference_domain.min.isZero());
  EXPECT_TRUE(view.reference_domain.max.isZero());
}

TEST(LocalObstacleView, SingleOccupiedVoxelUsesNativeClosedBoxAndIndex) {
  GridFixture fixture(Eigen::Vector3i(3, 3, 3));
  const Eigen::Vector3i index(1, 2, 0);
  fixture.set(index, 1);
  const LocalObstacleBox reference = box(
      Eigen::Vector3d(1.1, 1.9, 0.1), Eigen::Vector3d(1.8, 2.7, 0.8));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(reference));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  ASSERT_EQ(view.obstacles.size(), 1U);
  EXPECT_EQ(view.obstacles.front().index, index);
  EXPECT_EQ(view.obstacles.front().layer_mask, 0x01U);
  expectObstacleSet(fixture, view, reference);
}

TEST(LocalObstacleView, OverlayLayersUnionAndOverlapAreDeduplicated) {
  GridFixture fixture(Eigen::Vector3i(3, 3, 2));
  fixture.set(Eigen::Vector3i(0, 0, 0), -1, 0, 0);
  fixture.set(Eigen::Vector3i(1, 1, 0), 0, 2, 0);
  fixture.set(Eigen::Vector3i(2, 2, 1), 3, -4, 5);
  const LocalObstacleBox reference = box(Eigen::Vector3d::Zero(),
                                         Eigen::Vector3d::Constant(3.0));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(reference));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  expectObstacleSet(fixture, view, reference);
  ASSERT_EQ(view.obstacles.size(), 3U);
  EXPECT_EQ(view.obstacles[0].layer_mask, 0x01U);
  EXPECT_EQ(view.obstacles[1].layer_mask, 0x02U);
  EXPECT_EQ(view.obstacles[2].layer_mask, 0x07U);
}

TEST(LocalObstacleView, ClosedFaceEdgeAndCornerContactsAreNotDropped) {
  GridFixture fixture(Eigen::Vector3i(3, 3, 3));
  const Eigen::Vector3d contact(1.0, 1.0, 1.0);
  for (int x = 0; x <= 1; ++x) {
    for (int y = 0; y <= 1; ++y) {
      for (int z = 0; z <= 1; ++z) {
        fixture.set(Eigen::Vector3i(x, y, z), 1);
      }
    }
  }
  const LocalObstacleBox reference = box(contact, contact);
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(reference));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  expectObstacleSet(fixture, view, reference);
  EXPECT_EQ(view.obstacles.size(), 8U);
}

TEST(LocalObstacleView, NonzeroOriginAndNonintegerResolutionPreserveGeometry) {
  const Eigen::Vector3d origin(1.1, -2.3, 0.7);
  GridFixture fixture(Eigen::Vector3i(3, 2, 2), origin, 0.3);
  const Eigen::Vector3i index(2, 1, 0);
  fixture.set(index, 1);
  const Eigen::Vector3d lower = origin + 0.3 * index.cast<double>();
  const LocalObstacleBox reference = box(lower + Eigen::Vector3d::Constant(0.01),
                                         lower + Eigen::Vector3d::Constant(0.29));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(reference));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  expectObstacleSet(fixture, view, reference);
  ASSERT_EQ(view.obstacles.size(), 1U);
  EXPECT_NEAR(view.obstacles.front().bounds.min.x(), lower.x(), 1e-12);
  EXPECT_NEAR(view.obstacles.front().bounds.max.z(), lower.z() + 0.3, 1e-12);
}

TEST(LocalObstacleView, LargeGridScansOnlyLocalCandidateRange) {
  GridFixture fixture(Eigen::Vector3i(100, 100, 100));
  const Eigen::Vector3i index(50, 50, 50);
  fixture.set(index, 1);
  LocalObstacleBox reference = box(Eigen::Vector3d(50.1, 50.1, 50.1),
                                   Eigen::Vector3d(50.2, 50.2, 50.2));
  LocalObstacleRequest request = fixture.request(reference);
  request.max_voxel_checks = 1000U;
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, request);
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  EXPECT_LT(view.visited_voxels, 1000U);
  EXPECT_LT(view.visited_voxels,
            static_cast<std::size_t>(100 * 100 * 100));
  expectObstacleSet(fixture, view, reference);
}

TEST(LocalObstacleView, PartialKnownDomainClipsAndShrinksReferenceSafely) {
  GridFixture fixture(Eigen::Vector3i(6, 6, 6));
  LocalObstacleRequest request = fixture.request(
      box(Eigen::Vector3d::Constant(-1.0), Eigen::Vector3d::Constant(7.0)),
      0.2);
  request.known_region = box(Eigen::Vector3d::Constant(1.0),
                             Eigen::Vector3d::Constant(5.0));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, request);
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  EXPECT_TRUE(view.request_clipped);
  EXPECT_TRUE((view.obstacle_region.min.array() >= 1.0).all());
  EXPECT_TRUE((view.obstacle_region.max.array() <= 5.0).all());
  EXPECT_TRUE((view.reference_domain.min.array() >= 1.2).all());
  EXPECT_TRUE((view.reference_domain.max.array() <= 4.8).all());
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_GE(view.reference_domain.min(axis) - request.clearance,
               view.obstacle_region.min(axis));
    EXPECT_LE(view.reference_domain.max(axis) + request.clearance,
               view.obstacle_region.max(axis));
  }
}

TEST(LocalObstacleView, CompletelyOutsideKnownDomainIsOutOfDomain) {
  GridFixture fixture(Eigen::Vector3i(3, 3, 3));
  const LocalObstacleRequest request = fixture.request(
      box(Eigen::Vector3d::Constant(4.0), Eigen::Vector3d::Constant(5.0)));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, request);
  EXPECT_EQ(view.status, LocalObstacleViewStatus::OUT_OF_DOMAIN);
  EXPECT_TRUE(view.obstacles.empty());
  EXPECT_EQ(view.visited_voxels, 0U);
}

TEST(LocalObstacleView, ClearanceCanMakeTheReferenceDomainEmpty) {
  GridFixture fixture(Eigen::Vector3i(3, 3, 3));
  LocalObstacleRequest request = fixture.request(
      box(Eigen::Vector3d::Constant(0.5), Eigen::Vector3d::Constant(0.5)),
      0.6);
  request.known_region = box(Eigen::Vector3d::Zero(),
                             Eigen::Vector3d::Constant(1.0));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, request);
  EXPECT_EQ(view.status, LocalObstacleViewStatus::OUT_OF_DOMAIN);
  EXPECT_TRUE(view.obstacles.empty());
}

TEST(LocalObstacleView, ClearanceHaloReturnsObstaclesOutsideReferenceBox) {
  GridFixture fixture(Eigen::Vector3i(5, 5, 5), Eigen::Vector3d::Zero(), 0.5);
  const Eigen::Vector3i halo_index(1, 2, 2);
  fixture.set(halo_index, 1);
  const LocalObstacleBox reference = box(Eigen::Vector3d::Constant(1.0),
                                         Eigen::Vector3d::Constant(1.0));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(reference, 0.5));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  expectObstacleSet(fixture, view, view.obstacle_region);
  EXPECT_TRUE(std::find_if(view.obstacles.begin(), view.obstacles.end(),
                           [&](const LocalObstacleVoxel& obstacle) {
                             return obstacle.index == halo_index;
                           }) != view.obstacles.end());
}

TEST(LocalObstacleView, SmallerMapBoundsNeverExpandReferenceToExtraGridCells) {
  GridFixture fixture(Eigen::Vector3i(5, 5, 5));
  fixture.grid.map_bounds = box(Eigen::Vector3d::Constant(1.0),
                                Eigen::Vector3d::Constant(3.5));
  fixture.set(Eigen::Vector3i(0, 0, 0), 1);
  fixture.set(Eigen::Vector3i(4, 4, 4), 1);
  fixture.set(Eigen::Vector3i(2, 2, 2), 1);
  const LocalObstacleBox reference = box(Eigen::Vector3d::Constant(1.5),
                                         Eigen::Vector3d::Constant(2.5));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(reference));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  EXPECT_TRUE((view.reference_domain.min.array() >= 1.5).all());
  EXPECT_TRUE((view.reference_domain.max.array() <= 2.5).all());
  for (const LocalObstacleVoxel& obstacle : view.obstacles) {
    EXPECT_GE(obstacle.index.x(), 1);
    EXPECT_LE(obstacle.index.x(), 2);
    EXPECT_GE(obstacle.index.y(), 1);
    EXPECT_LE(obstacle.index.y(), 2);
    EXPECT_GE(obstacle.index.z(), 1);
    EXPECT_LE(obstacle.index.z(), 2);
  }
}

TEST(LocalObstacleView, InvalidNumericsAndDimensionsFailBeforeBufferAccess) {
  GridFixture fixture(Eigen::Vector3i(2, 2, 2));
  LocalObstacleRequest request = fixture.request(
      box(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0)));
  fixture.grid.resolution = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(plan_env::buildLocalObstacleView(fixture.grid, request).status,
            LocalObstacleViewStatus::INVALID_INPUT);
  fixture.grid.resolution = 1.0;
  fixture.grid.voxel_count.x() = 0;
  EXPECT_EQ(plan_env::buildLocalObstacleView(fixture.grid, request).status,
            LocalObstacleViewStatus::INVALID_INPUT);
  fixture.grid.voxel_count.x() = 2;
  request.reference_region.min.x() = 2.0;
  request.reference_region.max.x() = 1.0;
  EXPECT_EQ(plan_env::buildLocalObstacleView(fixture.grid, request).status,
            LocalObstacleViewStatus::INVALID_INPUT);
}

TEST(LocalObstacleView, BadBufferLengthsAndReversedKnownBoxAreInvalid) {
  GridFixture fixture(Eigen::Vector3i(2, 2, 2));
  LocalObstacleRequest request = fixture.request(
      box(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0)));
  fixture.inflated.pop_back();
  EXPECT_EQ(plan_env::buildLocalObstacleView(fixture.grid, request).status,
            LocalObstacleViewStatus::INVALID_INPUT);
  fixture.inflated.resize(8U, 0);
  request.known_region.min.x() = 2.0;
  request.known_region.max.x() = 1.0;
  EXPECT_EQ(plan_env::buildLocalObstacleView(fixture.grid, request).status,
            LocalObstacleViewStatus::INVALID_INPUT);
}

TEST(LocalObstacleView, VoxelAndObstacleBudgetsFailClosed) {
  GridFixture fixture(Eigen::Vector3i(4, 4, 4));
  for (char& value : fixture.inflated) value = 1;
  LocalObstacleRequest request = fixture.request(
      box(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(4.0)));
  request.max_voxel_checks = 10U;
  LocalObstacleView view = plan_env::buildLocalObstacleView(fixture.grid,
                                                              request);
  EXPECT_EQ(view.status, LocalObstacleViewStatus::BUDGET_EXCEEDED);
  EXPECT_TRUE(view.obstacles.empty());
  EXPECT_EQ(view.visited_voxels, 0U);
  request.max_voxel_checks = 100U;
  request.max_occupied_voxels = 3U;
  view = plan_env::buildLocalObstacleView(fixture.grid, request);
  EXPECT_EQ(view.status, LocalObstacleViewStatus::BUDGET_EXCEEDED);
  EXPECT_TRUE(view.obstacles.empty());
  EXPECT_TRUE(view.reference_domain.min.isZero());
}

TEST(LocalObstacleView, DegenerateReferenceAndZeroClearanceRemainFinite) {
  GridFixture fixture(Eigen::Vector3i(3, 3, 3));
  fixture.set(Eigen::Vector3i(1, 1, 1), 1);
  const LocalObstacleBox line = box(Eigen::Vector3d(1.0, 0.5, 1.0),
                                    Eigen::Vector3d(2.0, 0.5, 1.0));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(line, 0.0));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  EXPECT_EQ(view.reference_domain.min, line.min);
  EXPECT_EQ(view.reference_domain.max, line.max);
  EXPECT_TRUE(view.obstacle_region.min.allFinite());
  EXPECT_TRUE(view.obstacle_region.max.allFinite());
}

TEST(LocalObstacleView, ReturnedViewIsIndependentOfBorrowedBuffers) {
  GridFixture fixture(Eigen::Vector3i(3, 3, 3));
  const Eigen::Vector3i index(1, 1, 1);
  fixture.set(index, 1, 2, 3);
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(
          box(Eigen::Vector3d::Constant(1.0),
              Eigen::Vector3d::Constant(2.0))));
  ASSERT_EQ(view.status, LocalObstacleViewStatus::VALID);
  ASSERT_EQ(view.obstacles.size(), 1U);
  fixture.set(index, 0, 0, 0);
  EXPECT_EQ(view.obstacles.front().index, index);
  EXPECT_EQ(view.obstacles.front().layer_mask, 0x07U);
}

TEST(LocalObstacleView, RepeatedCallsAreDeterministicAndOrdered) {
  GridFixture fixture(Eigen::Vector3i(4, 3, 2));
  std::mt19937 generator(20260909U);
  for (char& value : fixture.inflated) value =
      static_cast<char>(generator() % 3U == 0U ? 1 : 0);
  for (char& value : fixture.manual) value =
      static_cast<char>(generator() % 5U == 0U ? -2 : 0);
  for (char& value : fixture.static_layer) value =
      static_cast<char>(generator() % 7U == 0U ? 4 : 0);
  const LocalObstacleRequest request = fixture.request(
      box(Eigen::Vector3d(0.2, 0.1, 0.0), Eigen::Vector3d(3.1, 2.1, 1.2)));
  const LocalObstacleView first = plan_env::buildLocalObstacleView(
      fixture.grid, request);
  const LocalObstacleView second = plan_env::buildLocalObstacleView(
      fixture.grid, request);
  ASSERT_EQ(first.status, LocalObstacleViewStatus::VALID);
  ASSERT_EQ(second.status, LocalObstacleViewStatus::VALID);
  ASSERT_EQ(first.obstacles.size(), second.obstacles.size());
  ASSERT_EQ(first.visited_voxels, second.visited_voxels);
  for (std::size_t i = 0; i < first.obstacles.size(); ++i) {
    EXPECT_EQ(first.obstacles[i].index, second.obstacles[i].index);
    EXPECT_EQ(first.obstacles[i].layer_mask, second.obstacles[i].layer_mask);
    if (i != 0U) {
      EXPECT_LT(key(first.obstacles[i - 1U].index),
                key(first.obstacles[i].index));
    }
  }
}

TEST(LocalObstacleView, OverflowAndZeroBudgetsAreRejected) {
  GridFixture fixture(Eigen::Vector3i(2, 2, 2));
  LocalObstacleRequest request = fixture.request(
      box(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(1.0)));
  request.max_voxel_checks = 0U;
  EXPECT_EQ(plan_env::buildLocalObstacleView(fixture.grid, request).status,
            LocalObstacleViewStatus::INVALID_INPUT);
  request.max_voxel_checks = 100U;
  request.max_occupied_voxels = 0U;
  EXPECT_EQ(plan_env::buildLocalObstacleView(fixture.grid, request).status,
            LocalObstacleViewStatus::INVALID_INPUT);
  fixture.grid.voxel_count = Eigen::Vector3i::Constant(
      std::numeric_limits<int>::max());
  fixture.grid.inflated = nullptr;
  EXPECT_EQ(plan_env::buildLocalObstacleView(fixture.grid, request).status,
            LocalObstacleViewStatus::INVALID_INPUT);
}

TEST(LocalObstacleView, TinyPositiveClearanceUsesSafeOutwardRepresentables) {
  const double origin_value = 1.0e12;
  GridFixture fixture(Eigen::Vector3i(4, 4, 4),
                      Eigen::Vector3d::Constant(origin_value), 0.5);
  const double tiny_clearance = std::ldexp(1.0, -100);
  const LocalObstacleBox reference = box(
      Eigen::Vector3d::Constant(origin_value + 0.75),
      Eigen::Vector3d::Constant(origin_value + 0.75));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(reference, tiny_clearance));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  EXPECT_FALSE(view.request_clipped);
  EXPECT_EQ(view.reference_domain.min, reference.min);
  EXPECT_EQ(view.reference_domain.max, reference.max);
  EXPECT_TRUE(view.obstacle_region.min.allFinite());
  EXPECT_TRUE(view.obstacle_region.max.allFinite());
  EXPECT_LT(view.obstacle_region.min.x(), reference.min.x());
  EXPECT_GT(view.obstacle_region.max.x(), reference.max.x());
}

TEST(LocalObstacleView, SingleFaceClipPreservesUnclippedDegenerateAxes) {
  GridFixture fixture(Eigen::Vector3i(4, 4, 4));
  const LocalObstacleBox reference = box(
      Eigen::Vector3d(-0.2, 1.0, 1.0),
      Eigen::Vector3d(1.2, 1.0, 1.0));
  const double clearance = 0.4;
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(reference, clearance));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  EXPECT_TRUE(view.request_clipped);
  EXPECT_NEAR(view.reference_domain.min.x(), 0.4, 1e-12);
  EXPECT_DOUBLE_EQ(view.reference_domain.max.x(), reference.max.x());
  EXPECT_DOUBLE_EQ(view.reference_domain.min.y(), reference.min.y());
  EXPECT_DOUBLE_EQ(view.reference_domain.max.y(), reference.max.y());
  EXPECT_DOUBLE_EQ(view.reference_domain.min.z(), reference.min.z());
  EXPECT_DOUBLE_EQ(view.reference_domain.max.z(), reference.max.z());
  EXPECT_TRUE(view.reference_domain.min.allFinite());
  EXPECT_TRUE(view.reference_domain.max.allFinite());
}

TEST(LocalObstacleView, NativeBoundsSurviveOriginProductCancellation) {
  // The index product is large while the native origin cancels it.  A
  // two-step rounded computation can lose the residual; the returned box
  // must still enclose the higher-precision reference expression.
  const int extent = 1000001;
  const double origin = -100000.0;
  const double resolution = 0.1;
  GridFixture fixture(Eigen::Vector3i(extent, 1, 1),
                      Eigen::Vector3d(origin, 0.0, 0.0), resolution);
  const Eigen::Vector3i index(extent - 2, 0, 0);
  fixture.set(index, 1);
  const LocalObstacleBox reference = box(
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
  const LocalObstacleView view = plan_env::buildLocalObstacleView(
      fixture.grid, fixture.request(reference));
  EXPECT_EQ(view.status, LocalObstacleViewStatus::VALID);
  ASSERT_FALSE(view.obstacles.empty());
  const auto found = std::find_if(
      view.obstacles.begin(), view.obstacles.end(),
      [&](const LocalObstacleVoxel& obstacle) { return obstacle.index == index; });
  ASSERT_NE(found, view.obstacles.end());
  const long double expected_lower = static_cast<long double>(origin) +
      static_cast<long double>(index.x()) *
      static_cast<long double>(resolution);
  const long double expected_upper = expected_lower +
      static_cast<long double>(resolution);
  EXPECT_LE(static_cast<long double>(found->bounds.min.x()), expected_lower);
  EXPECT_GE(static_cast<long double>(found->bounds.max.x()), expected_upper);
}

TEST(LocalObstacleView, UnrepresentableNativeGridSpanIsInvalid) {
  const double origin = 1.0e12;
  GridFixture fixture(Eigen::Vector3i(4, 4, 4),
                      Eigen::Vector3d::Constant(origin), 1.0e-20);
  // Give the input a nominally non-empty map box even though one voxel step
  // cannot be represented at this origin.
  fixture.grid.map_bounds.max = Eigen::Vector3d::Constant(
      std::nextafter(origin, std::numeric_limits<double>::infinity()));
  const LocalObstacleRequest request = fixture.request(
      box(Eigen::Vector3d::Constant(origin),
          Eigen::Vector3d::Constant(origin)));
  EXPECT_EQ(plan_env::buildLocalObstacleView(fixture.grid, request).status,
            LocalObstacleViewStatus::INVALID_INPUT);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
