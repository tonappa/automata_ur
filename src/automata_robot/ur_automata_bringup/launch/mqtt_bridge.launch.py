import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# The single ROS <-> MQTT bridge of the cell (config/mqtt_bridge.yaml).
def generate_launch_description():
    mqtt_bridge = Node(
        package="mqtt_client",
        executable="mqtt_client",
        name="mqtt_client",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("ur_automata_bringup"),
                                 "config", "mqtt_bridge.yaml")],
    )
    return LaunchDescription([mqtt_bridge])
