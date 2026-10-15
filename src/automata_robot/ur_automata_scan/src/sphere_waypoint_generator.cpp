#include "ur_automata_scan/sphere_waypoint_generator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

// theta = polar angle (0 = north pole, PI = south pole), phi = azimuth.
static Eigen::Vector3d sphere_point(
  const Eigen::Vector3d & center, double radius, double theta, double phi)
{
  return center + Eigen::Vector3d(
    radius * std::sin(theta) * std::cos(phi),
    radius * std::sin(theta) * std::sin(phi),
    radius * std::cos(theta));
}

// Pose at `position` with the Y axis (camera optical axis) toward `center`.
static geometry_msgs::msg::Pose make_look_at_pose(
  const Eigen::Vector3d & position, const Eigen::Vector3d & center)
{
  Eigen::Vector3d y_dir = (center - position).normalized();

  // Reference for the other axes: world Z, or X at the poles.
  Eigen::Vector3d ref = Eigen::Vector3d::UnitZ();
  if (y_dir.cross(ref).norm() < 1e-6) {
    ref = Eigen::Vector3d::UnitX();
  }

  Eigen::Vector3d x_dir = (y_dir.cross(ref)).normalized();
  Eigen::Vector3d z_dir = x_dir.cross(y_dir);

  Eigen::Matrix3d rot;
  rot.col(0) = x_dir;
  rot.col(1) = y_dir;
  rot.col(2) = z_dir;

  Eigen::Quaterniond q(rot);

  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

// One hemisphere, from its pole toward the equator.
static std::vector<geometry_msgs::msg::Pose> generate_hemisphere(
  const Eigen::Vector3d & center,
  double radius,
  bool upper,
  int direction,
  int num_rings,
  int points_per_ring,
  int num_arc,
  int points_per_arc,
  double equator_exclusion,
  bool stagger_rings,
  bool adaptive_rings)
{
  std::vector<geometry_msgs::msg::Pose> poses;

  double pole_theta = upper ? 0.0 : M_PI;
  double sign       = upper ? 1.0 : -1.0;  // +1 grows theta, -1 shrinks it

  double range = (M_PI / 2.0) - equator_exclusion;

  poses.push_back(make_look_at_pose(sphere_point(center, radius, pole_theta, 0.0), center));

  if (direction == SCAN_LATITUDINAL) {
    // Serpentine: even rings are walked backward, no jump back to phi 0.
    double d_theta = sign * range / num_rings;

    for (int ring = 1; ring <= num_rings; ++ring) {
      double theta = pole_theta + ring * d_theta;
      bool reverse = (ring % 2 == 0);

      // points_per_ring is the density at the equator.
      int n_points = points_per_ring;
      if (adaptive_rings) {
        n_points = std::max(1, (int)std::round(points_per_ring * std::sin(theta)));
      }

      double half_step     = M_PI / n_points;
      double stagger_offset = (stagger_rings && ring % 2 == 0) ? half_step : 0.0;

      for (int pt = 0; pt < n_points; ++pt) {
        int actual_pt = reverse ? (n_points - 1 - pt) : pt;
        double phi = actual_pt * 2.0 * M_PI / n_points + stagger_offset;
        poses.push_back(make_look_at_pose(sphere_point(center, radius, theta, phi), center));
      }
    }

  } else {
    // Serpentine: odd meridians go equator -> pole.
    double d_theta = sign * range / points_per_arc;

    for (int meridian = 0; meridian < num_arc; ++meridian) {
      double phi = meridian * 2.0 * M_PI / num_arc;
      bool reverse = (meridian % 2 == 1);

      for (int pt = 1; pt <= points_per_arc; ++pt) {
        int actual_pt = reverse ? (points_per_arc + 1 - pt) : pt;
        double theta = pole_theta + actual_pt * d_theta;
        poses.push_back(make_look_at_pose(sphere_point(center, radius, theta, phi), center));
      }
    }
  }

  return poses;
}

std::vector<geometry_msgs::msg::Pose> generate_waypoints(const ScanConfig & cfg)
{
  std::vector<geometry_msgs::msg::Pose> poses;

  if (cfg.hemisphere == HEMISPHERE_UPPER || cfg.hemisphere == HEMISPHERE_FULL) {
    std::vector<geometry_msgs::msg::Pose> upper = generate_hemisphere(
      cfg.center, cfg.radius, true, cfg.direction,
      cfg.num_rings, cfg.points_per_ring,
      cfg.num_arc,   cfg.points_per_arc,
      cfg.equator_exclusion_upper_rad, cfg.stagger_rings, cfg.adaptive_rings);

    poses.insert(poses.end(), upper.begin(), upper.end());
  }

  if (cfg.hemisphere == HEMISPHERE_LOWER || cfg.hemisphere == HEMISPHERE_FULL) {
    std::vector<geometry_msgs::msg::Pose> lower = generate_hemisphere(
      cfg.center, cfg.radius, false, cfg.direction,
      cfg.num_rings, cfg.points_per_ring,
      cfg.num_arc,   cfg.points_per_arc,
      cfg.equator_exclusion_lower_rad, cfg.stagger_rings, cfg.adaptive_rings);

    poses.insert(poses.end(), lower.begin(), lower.end());
  }

  return poses;
}

void order_by_sectors(const std::vector<geometry_msgs::msg::Pose> & pts,
                      const Eigen::Vector3d & center, double phi_front,
                      int num_sectors, bool upper,
                      std::vector<geometry_msgs::msg::Pose> & out,
                      std::vector<SectorBlock> & blocks)
{
  const double sector_width = 2.0 * M_PI / num_sectors;

  // Per point: sector, angle from the pole (= ring), azimuth from sector 0 start.
  std::vector<int> sector(pts.size());
  std::vector<double> pole_dist(pts.size());
  std::vector<double> rel_phi(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    Eigen::Vector3d v(pts[i].position.x - center.x(),
                      pts[i].position.y - center.y(),
                      pts[i].position.z - center.z());
    pole_dist[i] = std::acos(std::min(1.0, std::abs(v.z()) / v.norm()));
    double phi = std::atan2(v.y(), v.x()) - phi_front + sector_width / 2.0;
    while (phi < 0.0) phi += 2.0 * M_PI;
    while (phi >= 2.0 * M_PI) phi -= 2.0 * M_PI;
    rel_phi[i] = phi;
    sector[i] = std::min(num_sectors - 1, static_cast<int>(phi / sector_width));
    if (pole_dist[i] < 1e-6) sector[i] = 0;   // the pole has no azimuth: first sector
  }

  // Distinct rings, sorted from the pole toward the equator.
  std::vector<double> rings;
  for (double d : pole_dist) {
    bool found = false;
    for (double r : rings) {
      if (std::abs(r - d) < 1e-4) found = true;
    }
    if (!found) rings.push_back(d);
  }
  std::sort(rings.begin(), rings.end());

  for (int s = 0; s < num_sectors; ++s) {
    SectorBlock block;
    block.first_wp = out.size();
    block.sector = s;
    block.upper = upper;
    int count = 0;
    bool forward = true;   // walking direction along the current ring

    for (size_t k = 0; k < rings.size(); ++k) {
      size_t ring_idx = (s % 2 == 0) ? k : rings.size() - 1 - k;
      std::vector<size_t> ring_pts;
      for (size_t i = 0; i < pts.size(); ++i) {
        if (sector[i] == s && std::abs(pole_dist[i] - rings[ring_idx]) < 1e-4) {
          ring_pts.push_back(i);
        }
      }
      if (ring_pts.empty()) continue;
      std::sort(ring_pts.begin(), ring_pts.end(), [&](size_t a, size_t b) {
        return forward ? rel_phi[a] < rel_phi[b] : rel_phi[a] > rel_phi[b];
      });
      for (size_t i : ring_pts) {
        out.push_back(pts[i]);
        ++count;
      }
      forward = !forward;
    }
    if (count > 0) blocks.push_back(block);   // empty sector: no block
  }
}
