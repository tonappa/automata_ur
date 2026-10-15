#pragma once

#include <map>
#include <string>
#include <vector>

#include "ur_automata_scan/scan_recording.hpp"

// Saved plan of scan_tsp_node: a scan recording of ur_automata_scan plus a
// `plan:` section (waypoints, robot setup, scene) used to check it still fits.

// Bounding box of one scene object (world frame).
struct SceneObjectBox {
  std::string id;
  std::vector<double> box;   // min x, y, z, max x, y, z (m)
};

struct PlanData {
  std::string method;                          // visit order: tsp | gtsp | sectors | alternate
  std::vector<std::string> joint_names;        // planning group order
  std::vector<double> start_joints;            // robot state the plan starts from
  std::vector<int> visit_order;                // waypoint indices, in visit order
  std::vector<std::vector<double>> waypoints;  // every waypoint: x y z qx qy qz qw
  std::map<std::string, std::string> waypoint_params;  // scan settings that decide the waypoints
  double scaling = 0.0;                        // trajectory_scaling_factor
  std::vector<double> max_velocity;            // joint limits of the model (unscaled)
  std::vector<double> max_acceleration;
  std::vector<SceneObjectBox> scene;
  std::map<std::string, double> metrics;       // planning time, trajectory time, ...
};

struct ScanPlan {
  ScanRecording recording;
  PlanData data;
};

// Writes the plan. Returns false (and fills `error`) on failure.
bool save_plan(const std::string & path, const ScanPlan & plan, std::string & error);

// A plain recording without the `plan:` section is refused.
bool load_plan(const std::string & path, ScanPlan & plan, std::string & error);
