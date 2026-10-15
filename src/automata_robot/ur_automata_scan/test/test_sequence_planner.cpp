#include <gtest/gtest.h>

#include "ur_automata_scan/scan_sequence_planner.hpp"

// The DP picks the globally shortest chain, not the greedy one: via the near
// A the total is 1.20, via the far B it is 1.10.
TEST(SequencePlanner, PrefersGlobalPathOverGreedy)
{
  std::vector<std::vector<SeqCandidate>> layers = {
    { {{0.1, 0.0}}, {{0.0, 1.0}} },
    { {{0.0, 1.1}} },
  };
  SeqResult r = choose_sequence({0.0, 0.0}, layers);
  ASSERT_EQ(r.chosen.size(), 2u);
  EXPECT_EQ(r.chosen[0], 1);
  EXPECT_EQ(r.chosen[1], 0);
  EXPECT_NEAR(r.total_cost, 1.1, 1e-9);
}

// An empty layer is skipped (-1) and the chain goes on.
TEST(SequencePlanner, SkipsEmptyLayers)
{
  std::vector<std::vector<SeqCandidate>> layers = {
    { {{1.0}} },
    { },
    { {{5.0}}, {{1.5}} },
  };
  SeqResult r = choose_sequence({0.0}, layers);
  EXPECT_EQ(r.chosen[0], 0);
  EXPECT_EQ(r.chosen[1], -1);
  EXPECT_EQ(r.chosen[2], 1);
  EXPECT_NEAR(r.total_cost, 1.5, 1e-9);
}

// With identical joints, the lower extra_cost wins.
TEST(SequencePlanner, UsesExtraCost)
{
  std::vector<std::vector<SeqCandidate>> layers = {
    { {{1.0}, 0.5}, {{1.0}, 0.0} },
  };
  SeqResult r = choose_sequence({0.0}, layers);
  EXPECT_EQ(r.chosen[0], 1);
}

// A blocked segment is avoided when there is an alternative.
TEST(SequencePlanner, AvoidsBlockedEdges)
{
  std::vector<std::vector<SeqCandidate>> layers = {
    { {{0.1}}, {{0.5}} },
  };
  auto edge_ok = [](int prev_layer, int /*prev_c*/, int layer, int c) {
    return !(prev_layer == -1 && layer == 0 && c == 0);
  };
  SeqResult r = choose_sequence({0.0}, layers, edge_ok);
  EXPECT_EQ(r.chosen[0], 1);
}

// If every segment to a layer is blocked, the penalty is paid, the waypoint kept.
TEST(SequencePlanner, BlockedEdgesAreNotForbidden)
{
  std::vector<std::vector<SeqCandidate>> layers = { { {{1.0}} } };
  auto edge_ok = [](int, int, int, int) { return false; };
  SeqCost cost;
  cost.blocked_penalty = 1000.0;
  SeqResult r = choose_sequence({0.0}, layers, edge_ok, cost);
  EXPECT_EQ(r.chosen[0], 0);
  EXPECT_NEAR(r.total_cost, 1001.0, 1e-9);
}

// A custom edge cost replaces joint_l2.
TEST(SequencePlanner, UsesCustomEdgeCost)
{
  std::vector<std::vector<SeqCandidate>> layers = {
    { {{0.1}}, {{0.5}} },
  };
  EXPECT_EQ(choose_sequence({0.0}, layers).chosen[0], 0);

  SeqCost cost;
  cost.edge_cost = [](int /*prev_layer*/, int /*prev_c*/, int /*layer*/, int c) {
    return (c == 0) ? 2.0 : 0.7;
  };
  SeqResult r = choose_sequence({0.0}, layers, EdgeFilter(), cost);
  EXPECT_EQ(r.chosen[0], 1);
  EXPECT_NEAR(r.total_cost, 0.7, 1e-9);
}

// No wrap: +179 -> -179 degrees is ~358 degrees.
TEST(JointL2, NoWrapAround)
{
  double d = joint_l2({3.1}, {-3.1});
  EXPECT_NEAR(d, 6.2, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
