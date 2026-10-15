import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# ROS <-> MQTT bridge of the whole cell (3D scan and HSI), config in
# config/mqtt_bridge.yaml. Included by ur_automata_bringup.launch.py; start it
# by hand only when the scan or HSI nodes are launched on their own.
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
