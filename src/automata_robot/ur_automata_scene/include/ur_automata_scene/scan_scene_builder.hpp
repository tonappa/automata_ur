#ifndef UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_
#define UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_

#include <string>

#include <Eigen/Core>
#include <rclcpp/time.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace ur_automata_scene {

// Scene options, from the `scan` section of automata_config.yaml. Lengths in m.
struct SceneOptions {
  bool platform_sim = true;               // true: disk + 3 legs, false: STL platform_mesh
  std::string platform_mesh = "disk.stl"; // in meshes/, in mm

  double table_margin = 0.0;              // keep-out slab on the table, 0 = none

  // Keep-outs around the real platform (platform_sim false only).
  bool   stem_base_enabled = false;
  double stem_base_diameter = 0.14;
  double stem_base_height = 0.10;
  bool   stem_enabled = false;
  double stem_diameter = 0.08;
  double stem_height = 0.30;
  bool   disk_enabled = false;
  double disk_diameter = 0.36;
  double disk_height = 0.01;
  bool   support_enabled = false;
  double support_margin = 0.01;

  // Cell walls and ceiling in the global frame, 0 = none.
  double wall_back_y = 0.0;
  double wall_left_x = 0.0;
  double wall_right_x = 0.0;
  double wall_top_z = 0.0;

  // Robot base pose (robot.base_xyz / base_rpy): non-zero adds a mounting wall.
  Eigen::Vector3d base_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector3d base_rpy = Eigen::Vector3d::Zero();
};

// Table, platform, object and the optional keep-outs and walls, in
// `global_frame`. `center` = top face of the platform disk (m).
moveit_msgs::msg::PlanningScene build_scan_scene(
    const std::string & global_frame,
    const Eigen::Vector3d & center,
    const rclcpp::Time & stamp,
    const SceneOptions & opt);

}  // namespace ur_automata_scene

#endif  // UR_AUTOMATA_SCENE__SCAN_SCENE_BUILDER_HPP_
