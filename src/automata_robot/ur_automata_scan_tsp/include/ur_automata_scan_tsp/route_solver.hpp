#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Visit order found by the OR-Tools routing solver.
struct RouteResult {
  bool ok = false;
  std::vector<int> nodes;     // visited nodes, in order, start and end excluded
  int64_t objective = 0;      // total cost of the route, same unit as the matrix
  std::string status;         // OR-Tools search status, e.g. ROUTING_SUCCESS
};

// Open route from `start` to `end` on an n x n cost matrix (row-major,
// cost[a * n + b]). No clusters: TSP, every node once. Clusters: one node per
// cluster (generalized TSP). Heuristic (guided local search for time_limit_s),
// not proven optimal. `initial`: optional starting route, never worsened.
RouteResult solve_route(const std::vector<int64_t> & cost, int n, int start, int end,
                        const std::vector<std::vector<int>> & clusters,
                        double time_limit_s,
                        const std::vector<int> & initial = {});
