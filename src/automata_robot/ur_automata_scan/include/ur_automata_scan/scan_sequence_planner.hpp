#pragma once

#include <functional>
#include <vector>

// One joint configuration that reaches a waypoint, plus an optional extra cost.
struct SeqCandidate {
  std::vector<double> joints;
  double extra_cost = 0.0;
};

struct SeqResult {
  std::vector<int> chosen;   // per layer: chosen candidate, -1 = empty layer
  double total_cost = 0.0;
};

// Euclidean joint distance WITHOUT wrapping at +-pi: UR joints span +-2pi, so
// +179 -> -179 degrees really is almost a full turn.
double joint_l2(const std::vector<double> & a, const std::vector<double> & b);

// Returns false if the segment (prev_layer, prev_c) -> (layer, c) is blocked
// (prev_layer = -1 = start). A blocked segment is not forbidden: it costs
// blocked_penalty more, so the DP avoids it without dropping the waypoint.
using EdgeFilter = std::function<bool(int prev_layer, int prev_c, int layer, int c)>;

// Cost of the segment (prev_layer, prev_c) -> (layer, c); unset = joint_l2.
using EdgeCost = std::function<double(int prev_layer, int prev_c, int layer, int c)>;

struct SeqCost {
  EdgeCost edge_cost;              // empty = joint_l2
  double blocked_penalty = 1000.0; // added to a blocked segment (see EdgeFilter)
};

// Layered DP: layer i = candidates of waypoint i. Picks one candidate per layer
// minimizing the total segment cost from start_joints; empty layers are skipped.
SeqResult choose_sequence(const std::vector<double> & start_joints,
                          const std::vector<std::vector<SeqCandidate>> & layers,
                          const EdgeFilter & edge_ok = EdgeFilter(),
                          const SeqCost & cost = SeqCost());
