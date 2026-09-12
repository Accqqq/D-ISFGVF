#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "phase_offset_swarm/sph_intent.h"

namespace {

using phase_offset_swarm::NeighborFreshness;
using phase_offset_swarm::NeighborSnapshot;
using phase_offset_swarm::NeighborState;
using phase_offset_swarm::SphIntentOutput;
using phase_offset_swarm::SphIntentProvider;
using phase_offset_swarm::SphIntentStatus;
using phase_offset_swarm::SphParameters;
using phase_offset_swarm::SwarmAgentState;

SphParameters params() { return SphParameters(); }

SwarmAgentState self(double vx = 0.0, double vy = 0.0) {
  SwarmAgentState result;
  result.id = 0;
  result.position = Eigen::Vector2d(0.0, 0.0);
  result.velocity = Eigen::Vector2d(vx, vy);
  result.stamp = 1.0;
  result.frame_id = "world";
  return result;
}

NeighborState neighbor(int id, double x, double y, double vx = 0.0,
                       double vy = 0.0,
                       NeighborFreshness freshness =
                           NeighborFreshness::FRESH) {
  NeighborState result;
  result.id = id;
  result.position = Eigen::Vector2d(x, y);
  result.predicted_position = result.position;
  result.velocity = Eigen::Vector2d(vx, vy);
  result.source_stamp = 1.0;
  result.freshness = freshness;
  return result;
}

NeighborSnapshot snapshot(std::initializer_list<NeighborState> states) {
  NeighborSnapshot result;
  result.self_id = 0;
  for (const NeighborState& state : states) {
    result.all.push_back(state);
  }
  return result;
}

double kernel(double q) {
  if (q >= 0.0 && q <= 1.0) {
    return 1.0 - 1.5 * q * q + 0.75 * q * q * q;
  }
  if (q > 1.0 && q < 2.0) {
    const double tail = 2.0 - q;
    return 0.25 * tail * tail * tail;
  }
  return 0.0;
}

// Six neighbours on a regular hexagon of radius d_star.
NeighborSnapshot nominal_hexagon(double spacing) {
  NeighborSnapshot result;
  result.self_id = 0;
  for (int index = 0; index < 6; ++index) {
    const double angle = index * M_PI / 3.0;
    result.all.push_back(neighbor(1 + index, spacing * std::cos(angle),
                                  spacing * std::sin(angle)));
  }
  return result;
}

TEST(SphIntent, NominalLatticeSitsExactlyAtTheReferenceDensity) {
  SphIntentProvider provider(params());
  const SphIntentOutput output =
      provider.compute(self(), nominal_hexagon(1.5), 1.0);
  ASSERT_TRUE(output.valid);
  EXPECT_EQ(SphIntentStatus::VALID, output.status);
  EXPECT_EQ(6, output.support_neighbor_count);
  EXPECT_NEAR(output.rho_reference, output.rho_i, 1e-12);
  EXPECT_NEAR(0.0, output.eta_i, 1e-12);
  EXPECT_NEAR(0.0, output.g_density_repulsive.norm(), 1e-12);
  EXPECT_NEAR(0.0, output.g_density_attractive.norm(), 1e-12);
  // d_star = 1.5 m is outside the 0.9 m repulsion band and the hexagon is
  // symmetric, so the whole coordination velocity vanishes.
  EXPECT_NEAR(0.0, output.g_explicit_repulsive.norm(), 1e-12);
  EXPECT_NEAR(0.0, output.g_damping.norm(), 1e-12);
  EXPECT_NEAR(0.0, output.g_coord.norm(), 1e-12);
}

TEST(SphIntent, ShortRangeRepulsionIsBoundedAndInactiveAtNominalSpacing) {
  const SphParameters defaults = params();
  SphIntentProvider provider(defaults);
  // d = 0.4 m: [1 - 0.4/d_rep]^2 * k_rep, directed away from the neighbour.
  const SphIntentOutput close =
      provider.compute(self(), snapshot({neighbor(1, 0.4, 0.0)}), 1.0);
  ASSERT_TRUE(close.valid);
  const double expected =
      defaults.k_rep * std::pow(1.0 - 0.4 / defaults.d_rep, 2.0);
  EXPECT_NEAR(-expected, close.g_explicit_repulsive.x(), 1e-12);
  EXPECT_NEAR(0.0, close.g_explicit_repulsive.y(), 1e-12);
  // At d_rep and beyond the bounded term is exactly zero (so the nominal
  // configuration, which sits at d_star > d_rep, feels no extra push).
  const SphIntentOutput at_activation =
      provider.compute(self(),
                       snapshot({neighbor(1, defaults.d_rep, 0.0)}), 1.0);
  EXPECT_NEAR(0.0, at_activation.g_explicit_repulsive.norm(), 1e-12);
  const SphIntentOutput diluted =
      provider.compute(self(),
                       snapshot({neighbor(1, defaults.d_rep + 0.1, 0.0)}), 1.0);
  EXPECT_NEAR(0.0, diluted.g_explicit_repulsive.norm(), 1e-12);
  // Just inside the activation distance the term is already non-zero, which
  // is what removes the dead band between d_rep and d_star.
  const SphIntentOutput just_inside = provider.compute(
      self(), snapshot({neighbor(1, defaults.d_rep - 0.05, 0.0)}), 1.0);
  EXPECT_GT(just_inside.g_explicit_repulsive.norm(), 0.0);
}

TEST(SphIntent, CompressedNeighborhoodRepelsAwayFromTheConcentration) {
  SphIntentProvider provider(params());
  const SphIntentOutput output = provider.compute(
      self(),
      snapshot({neighbor(1, 1.5, 0.0), neighbor(2, 1.5, 0.2),
                neighbor(3, 0.7, -0.1)}),
      1.0);
  ASSERT_TRUE(output.valid);
  EXPECT_GT(output.eta_i, 0.0);
  EXPECT_GT(output.rho_i, output.rho_reference);
  // The cluster sits on +x, so the density branch pushes the vehicle to -x.
  EXPECT_LT(output.g_density_repulsive.x(), 0.0);
  EXPECT_NEAR(0.0, output.g_density_attractive.norm(), 1e-12);
}

TEST(SphIntent, DilutedNeighborhoodAttractsAndBetaModulatesOnlyThatBranch) {
  SphIntentProvider provider(params());
  const NeighborSnapshot dilute =
      snapshot({neighbor(1, 2.2, 0.0), neighbor(2, -1.9, 0.0)});
  const SphIntentOutput with_beta = provider.compute(self(), dilute, 1.0);
  const SphIntentOutput without_beta = provider.compute(self(), dilute, 0.0);
  ASSERT_TRUE(with_beta.valid);
  ASSERT_TRUE(without_beta.valid);
  EXPECT_LT(with_beta.eta_i, 0.0);
  EXPECT_LT(with_beta.rho_i, with_beta.rho_reference);
  // Attraction pulls toward the locally concentrated (-x) side.
  EXPECT_LT(with_beta.g_density_attractive.x(), 0.0);
  EXPECT_NEAR(0.0, with_beta.g_density_repulsive.norm(), 1e-12);
  // beta only scales the attractive branch.
  EXPECT_NEAR(with_beta.g_explicit_repulsive.x(),
              without_beta.g_explicit_repulsive.x(), 1e-12);
  EXPECT_NEAR(with_beta.g_damping.x(), without_beta.g_damping.x(), 1e-12);
  EXPECT_NEAR((with_beta.g_density_attractive + with_beta.g_explicit_repulsive +
               with_beta.g_damping).x(),
              with_beta.g_coord.x(), 1e-12);
  EXPECT_NEAR((without_beta.g_explicit_repulsive + without_beta.g_damping).x(),
              without_beta.g_coord.x(), 1e-12);
}

TEST(SphIntent, DampingOpposesRelativeRadialMotionAndVanishesUnderTranslation) {
  SphIntentProvider provider(params());
  // UAV i flies +x, neighbour j flies -x at d = 2 m: the pair closes, so the
  // damping term must push i away from j (negative x here).
  const SphIntentOutput closing = provider.compute(
      self(1.0, 0.0), snapshot({neighbor(1, 2.0, 0.0, -1.0, 0.0)}), 1.0);
  ASSERT_TRUE(closing.valid);
  const double weight = kernel(2.0 / 2.0);
  const double rho = 1.0 + weight;
  EXPECT_NEAR(-(weight / rho) * 2.0, closing.g_damping.x(), 1e-12);
  EXPECT_NEAR(0.0, closing.g_explicit_repulsive.norm(), 1e-12);

  // Common translation: identical velocities leave the term exactly zero.
  const SphIntentOutput translated = provider.compute(
      self(1.0, 0.0), snapshot({neighbor(1, 2.0, 0.0, 1.0, 0.0)}), 1.0);
  ASSERT_TRUE(translated.valid);
  EXPECT_NEAR(0.0, translated.g_damping.norm(), 1e-15);
}

TEST(SphIntent, TotalOutputIsSaturatedAtGMax) {
  SphParameters limited = params();
  limited.g_max = 0.05;
  SphIntentProvider provider(limited);
  const SphIntentOutput output =
      provider.compute(self(), snapshot({neighbor(1, 0.2, 0.0)}), 1.0);
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.output_saturated);
  EXPECT_NEAR(0.05, output.g_coord.norm(), 1e-12);
  EXPECT_GT(output.g_coord.norm(), limited.g_max - 1e-12);
}

TEST(SphIntent, CoincidentNeighborIsCountedWithoutProducingNaN) {
  SphIntentProvider provider(params());
  const SphIntentOutput output =
      provider.compute(self(), snapshot({neighbor(1, 0.0, 0.0)}), 1.0);
  ASSERT_TRUE(output.valid);
  EXPECT_EQ(1, output.near_zero_count);
  EXPECT_EQ(1, output.support_neighbor_count);
  EXPECT_TRUE(output.g_coord.allFinite());
}

TEST(SphIntent, InvalidInputsFailClosed) {
  SphIntentProvider provider(params());
  const SphIntentOutput duplicate = provider.compute(
      self(), snapshot({neighbor(1, 1.0, 0.0), neighbor(1, 1.0, 0.2)}), 1.0);
  EXPECT_FALSE(duplicate.valid);
  EXPECT_EQ(SphIntentStatus::DUPLICATE_FRESH_ID, duplicate.status);

  const SphIntentOutput bad_beta =
      provider.compute(self(), snapshot({neighbor(1, 1.0, 0.0)}), 1.5);
  EXPECT_FALSE(bad_beta.valid);
  EXPECT_EQ(SphIntentStatus::INVALID_BETA, bad_beta.status);

  const SphIntentOutput stale = provider.compute(
      self(),
      snapshot({neighbor(1, 1.0, 0.0, 0.0, 0.0, NeighborFreshness::STALE)}),
      1.0);
  ASSERT_TRUE(stale.valid);
  EXPECT_EQ(1, stale.ignored_nonfresh_count);
  EXPECT_EQ(0, stale.support_neighbor_count);
  EXPECT_NEAR(0.0, stale.g_coord.norm(), 1e-12);
}

TEST(SphParameters, ValidateEnforcesThePaperOrdering) {
  std::string error;
  EXPECT_TRUE(params().validate(&error)) << error;

  SphParameters equal = params();
  equal.d_rep = equal.reference_spacing;
  EXPECT_FALSE(equal.validate(&error));

  SphParameters too_wide = params();
  too_wide.reference_spacing = 2.0 * too_wide.h;
  EXPECT_FALSE(too_wide.validate(&error));

  SphParameters no_sensitivity = params();
  no_sensitivity.gamma = 0.0;
  EXPECT_FALSE(no_sensitivity.validate(&error));

  SphParameters negative_gain = params();
  negative_gain.k_rho = -1.0;
  EXPECT_FALSE(negative_gain.validate(&error));

  SphParameters negative_reference = params();
  negative_reference.reference_density = -1.0;
  EXPECT_FALSE(negative_reference.validate(&error));
}

// Paper rho_0 of a configuration: 1 + sum_j Wbar(d_ij/h) over its neighbours.
// Computed from the same snapshot in the same order the provider uses, so the
// nominal configuration must come out exactly neutral.
double reference_density_of(const NeighborSnapshot& snap, double h) {
  double total = 1.0;
  for (const NeighborState& state : snap.all) {
    const double distance =
        std::hypot(state.position.x(), state.position.y());
    total += kernel(distance / h);
  }
  return total;
}

NeighborSnapshot mixed_rings() {
  return snapshot({neighbor(1, 1.5, 0.0), neighbor(2, -1.5, 0.0),
                   neighbor(3, 0.0, 1.5), neighbor(4, 2.4, 0.0),
                   neighbor(5, -1.2, -2.08), neighbor(6, -1.2, 2.08)});
}

TEST(SphIntent, ExplicitReferenceDensityMatchesTheNominalFormation) {
  // Paper rho_0 for this cluster: three neighbours at d_star and three farther
  // out.  With it the nominal configuration is exactly neutral.
  SphParameters nominal = params();
  nominal.reference_density = reference_density_of(mixed_rings(), 2.0);
  SphIntentProvider nominal_provider(nominal);
  const SphIntentOutput at_nominal =
      nominal_provider.compute(self(), mixed_rings(), 1.0);
  ASSERT_TRUE(at_nominal.valid);
  EXPECT_NEAR(0.0, at_nominal.eta_i, 1e-12);
  EXPECT_NEAR(0.0, at_nominal.g_density_repulsive.norm(), 1e-12);
  EXPECT_NEAR(0.0, at_nominal.g_density_attractive.norm(), 1e-12);

  // Same configuration with a compressed inner ring: rho rises above rho_0,
  // so the paper branch must now repel.
  const SphIntentOutput compressed = nominal_provider.compute(
      self(),
      snapshot({neighbor(1, 1.1, 0.0), neighbor(2, -1.1, 0.0),
                neighbor(3, 0.0, 1.1), neighbor(4, 2.4, 0.0),
                neighbor(5, -1.2, -2.08), neighbor(6, -1.2, 2.08)}),
      1.0);
  ASSERT_TRUE(compressed.valid);
  EXPECT_GT(compressed.eta_i, 0.0);
  EXPECT_GT(compressed.g_density_repulsive.norm(), 0.0);

  // Diluted: rho below rho_0, so the (beta-modulated) attraction fires.
  const SphIntentOutput diluted = nominal_provider.compute(
      self(),
      snapshot({neighbor(1, 1.9, 0.0), neighbor(2, -1.9, 0.0),
                neighbor(3, 0.0, 1.9), neighbor(4, 2.4, 0.0),
                neighbor(5, -1.2, -2.08), neighbor(6, -1.2, 2.08)}),
      1.0);
  ASSERT_TRUE(diluted.valid);
  EXPECT_LT(diluted.eta_i, 0.0);
  EXPECT_GT(diluted.g_density_attractive.norm(), 0.0);
}

TEST(SphIntent, LocalEstimateBiasIsRemovedByTheExplicitReference) {
  // The fallback formula places every visible neighbour at d_star; for a
  // formation whose outer ring sits farther out that over-estimates rho_0 and
  // turns the nominal configuration into a persistent attraction.  The
  // explicit paper value removes exactly that bias.
  SphIntentProvider fallback(params());
  const SphIntentOutput estimated =
      fallback.compute(self(), mixed_rings(), 1.0);
  ASSERT_TRUE(estimated.valid);
  EXPECT_LT(estimated.eta_i, -0.05);
  EXPECT_GT(estimated.g_density_attractive.norm(), 0.0);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
