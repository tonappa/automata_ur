import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(package, *path):
    with open(os.path.join(get_package_share_directory(package), *path), "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_yaml("ur_automata_bringup", "config", "automata_config.yaml")
    p = cfg["planning"]
    s = cfg["scan"]
    h = cfg["hsi"]

    # The node solves IK itself (TRAC-IK, Distance mode: the solution closest
    # to home), so it needs kinematics.yaml.
    kinematics_yaml = _load_yaml("ur_automata_moveit_config", "config", "kinematics.yaml")

    hsi_node = Node(
        package="ur_automata_hsi",
        executable="hsi_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"robot_description_kinematics": kinematics_yaml},
            {
                "planning_group":    p["group"],
                "end_effector_link": p["end_effector_link"],
                "global_frame":      p["global_frame"],
                "home_pose_name":    p["home_pose_name"],
                "scan_center":       [float(v) for v in s["center"]],
                "hsi_link":          h["link"],
                "distance":          float(h["distance"]),
                # Same meaning as the speed of the scan (scan_tsp_node).
                "trajectory_scaling_factor": float(p["trajectory_scaling_factor"]),
                "speed":             ParameterValue(LaunchConfiguration("speed"), value_type=float),
                "start_topic":       h["start_topic"],
                "acquire_topic":     h["acquire_topic"],
                "status_topic":      h["status_topic"],
                "done_topic":        h["done_topic"],
                "wait_for_hsi":      bool(h["wait_for_hsi"]),
                "wait_s":            float(h["wait_s"]),
            },
        ],
    )

    return LaunchDescription([
        # fraction of planning.trajectory_scaling_factor, as the speed of the scan (max 1.0)
        DeclareLaunchArgument("speed", default_value="0.4"),
        hsi_node,
    ])
