import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_config():
    """automata_config.yaml from the installed share of ur_automata_bringup."""
    cfg_path = os.path.join(
        get_package_share_directory("ur_automata_bringup"),
        "config",
        "automata_config.yaml",
    )
    with open(cfg_path, "r") as f:
        return yaml.safe_load(f)


def _load_kinematics_yaml():
    """kinematics.yaml: without it the node's setFromIK fails with
    'No kinematics solver instantiated for group ur_manipulator'."""
    path = os.path.join(
        get_package_share_directory("ur_automata_moveit_config"),
        "config",
        "kinematics.yaml",
    )
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_config()
    p = cfg["planning"]
    s = cfg["scan"]
    kinematics_yaml = _load_kinematics_yaml()

    # TRAC-IK Speed for the enumeration (10-50x faster than Distance, which uses
    # the whole timeout); move_group keeps its own kinematics.yaml.
    kinematics_yaml[p["group"]]["solve_type"] = "Speed"

    # Not passed when empty: ROS cannot guess the type of an empty list.
    retry_param = {}
    retry_times = [float(t) for t in (s.get("retry_planning_times") or [])]
    if retry_times:
        retry_param["scan_retry_planning_times"] = retry_times

    scan_node = Node(
        package="ur_automata_scan",
        executable="scan_sequence_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"robot_description_kinematics": kinematics_yaml},
            retry_param,
            {
            "global_frame":              p["global_frame"],
            "planning_group":            p["group"],
            "end_effector_link":         p["end_effector_link"],
            "home_pose_name":            p["home_pose_name"],
            "lower_home_pose_name":      p["lower_home_pose_name"],
            "trajectory_scaling_factor": p["trajectory_scaling_factor"],
            "scan_center":                 s["center"],
            "scan_radius":                 s["radius"],
            "scan_hemisphere":             s["hemisphere"],
            "scan_direction":              s["direction"],
            "scan_num_rings":              s["num_rings"],
            "scan_points_per_ring":        s["points_per_ring"],
            "scan_num_arc":                s["num_arc"],
            "scan_points_per_arc":         s["points_per_arc"],
            "scan_equator_exclusion_upper_deg": s["equator_exclusion_upper_deg"],
            "scan_equator_exclusion_lower_deg": s["equator_exclusion_lower_deg"],
            "scan_lock_pitch":                  s["lock_pitch"],
            "scan_stagger_rings":               s["stagger_rings"],
            "scan_adaptive_rings":              s["adaptive_rings"],
            "scan_occlusion_check":            s["occlusion_check"],
            "scan_occlusion_disk_radius":      float(s.get("occlusion_disk_radius", 0.15)),
            "scan_occlusion_margin":           float(s.get("occlusion_margin", 0.01)),
            "scan_fallback_search":            s["fallback_search"],
            "scan_fallback_radius_mm":         s["fallback_radius_mm"],
            "scan_planners":               s["planners"],
            "scan_ompl_algorithm":         s["ompl_algorithm"],
            "scan_retry_ompl_algorithm":   str(s.get("retry_ompl_algorithm", "RRTConnect")),
            "scan_pitch_search_range_deg": s["pitch_search_range_deg"],
            "scan_pitch_search_step_deg":  s["pitch_search_step_deg"],
            "scan_pitch_xparallel_bias":   s["pitch_xparallel_bias"],
            "scan_enum_ik_timeout":        s["enum_ik_timeout"],
            "scan_planning_time":          s["planning_time"],
            "scan_planning_attempts":      s["planning_attempts"],
            # Optional keys: defaults = old behavior
            "scan_sectors":                int(s.get("sectors", 0)),
            "scan_sector_offset_deg":      float(s.get("sector_offset_deg", 0.0)),
            "scan_joint_cost_weight":      float(s.get("joint_cost_weight", 0.1)),
            "scan_swing_threshold_m":      float(s.get("swing_threshold_m", 0.25)),
            "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
            # record:=true saves the executed motions to scan.recording_file
            "record": ParameterValue(LaunchConfiguration("record"), value_type=bool),
            "recording_file":              str(s.get("recording_file", "")),
            },
        ],
    )

    return LaunchDescription([
        # dry_run:=true: plan, print the forecast and exit without moving
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("record", default_value="false"),
        scan_node,
    ])
