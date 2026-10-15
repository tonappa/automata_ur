#pragma once

#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#define HEMISPHERE_UPPER 0
#define HEMISPHERE_LOWER 1
#define HEMISPHERE_FULL  2

#define SCAN_LATITUDINAL  0   // ring by ring (horizontal circles)
#define SCAN_LONGITUDINAL 1   // meridian by meridian (vertical slices)

struct ScanConfig {
  Eigen::Vector3d center;
  double radius;                // m
  int hemisphere;               // HEMISPHERE_*
  int direction;                // SCAN_*

  int num_rings;                // latitudinal: rings between pole and equator
  int points_per_ring;

  int num_arc;                  // longitudinal: meridians
  int points_per_arc;           // excluding the pole

  double equator_exclusion_upper_rad; // band skipped near the equator
  double equator_exclusion_lower_rad;
  bool stagger_rings;                 // even rings offset by half a step
  bool adaptive_rings;                // points per ring scaled by sin(theta)
};

// Scan poses; at every pose the end-effector Y axis points at the center.
std::vector<geometry_msgs::msg::Pose> generate_waypoints(const ScanConfig & cfg);

// An azimuth slice of one hemisphere (see docs/scan.md, sectors).
struct SectorBlock {
  size_t first_wp = 0;        // first waypoint of the sector, in visit order
  int sector = 0;             // 0 = front (robot side), then counterclockwise from above
  bool upper = true;
};

// Reorders ONE hemisphere by sectors and appends to `out`. phi_front (rad) =
// azimuth of center -> robot base, the middle of sector 0. Rings go pole to
// equator (reversed in odd sectors) in serpentine order.
void order_by_sectors(const std::vector<geometry_msgs::msg::Pose> & pts,
                      const Eigen::Vector3d & center, double phi_front,
                      int num_sectors, bool upper,
                      std::vector<geometry_msgs::msg::Pose> & out,
                      std::vector<SectorBlock> & blocks);
