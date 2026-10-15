#pragma once

#include <vector>

#include "ur_automata_scan/scan_sequence_planner.hpp"

// Estimated duration (s) of a Pilz PTP motion from a to b: trapezoidal (or
// triangular) profile per joint, synchronized on the slowest. Limits per joint,
// already scaled. Pilz samples every 0.1 s, so the real time can be a bit longer.
double ptp_time(const std::vector<double> & a, const std::vector<double> & b,
                const std::vector<double> & max_velocity,
                const std::vector<double> & max_acceleration);

// Layered DP (choose_sequence) with lazy segment checks: the segments of the
// chosen chain are checked with `check_segment` (true = straight joint line
// free); a blocked one makes every pair between those two layers checked, then
// the DP runs again. `known_free`: answer from past checks (unknown = free);
// known blocked segments cost `cost.blocked_penalty` more.
SeqResult choose_sequence_lazy(const std::vector<double> & start_joints,
                               const std::vector<std::vector<SeqCandidate>> & layers,
                               const SeqCost & cost,
                               const EdgeFilter & known_free,
                               const EdgeFilter & check_segment,
                               int max_iterations,
                               int * iterations = nullptr);
