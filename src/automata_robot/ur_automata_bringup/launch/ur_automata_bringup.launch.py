import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

# Plan executed when plan_file is not given (mode:=execute only).
DEFAULT_PLAN_FILE = "/home/ros/ur/recordings/scan_tsp_real.yaml"


def _load_config():
    cfg_path = os.path.join(
        get_package_share_directory("ur_automata_bringup"),
        "config",
        "automata_config.yaml",
    )
    with open(cfg_path, "r") as f:
        return yaml.safe_load(f)


def launch_setup(context):
    pkg_share_dir = get_package_share_directory('ur_automata_bringup')

    ur_automata_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, 'launch', 'ur_automata_control.launch.py')
        )
    )

    ur_automata_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, 'launch', 'ur_automata_moveit.launch.py')
        ),
        launch_arguments={
            'use_rviz':  LaunchConfiguration('use_rviz'),
            'use_scene': LaunchConfiguration('use_scene'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_moveit'))
    )

    actions = [ur_automata_control_launch, ur_automata_moveit_launch]

    use_scan = LaunchConfiguration('use_scan').perform(context) == 'true'
    use_hsi = LaunchConfiguration('use_hsi').perform(context) == 'true'
    if not use_scan and not use_hsi:
        return actions
    if (LaunchConfiguration('use_moveit').perform(context) != 'true' or
            LaunchConfiguration('use_scene').perform(context) != 'true'):
        raise RuntimeError("use_scan:=true and use_hsi:=true need use_moveit:=true and use_scene:=true "
                           "(or use_scan:=false use_hsi:=false).")

    # One ROS <-> MQTT bridge for the scan and the HSI.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share_dir, 'launch', 'mqtt_bridge.launch.py'))
    ))

    # The nodes started after the scene is ready.
    after_scene = []
    message = "[ur_automata_bringup] scene ready: starting"

    # --- 3D scan (scan_tsp_node) ---
    if use_scan:
        mode = LaunchConfiguration('mode').perform(context)
        plan_file = LaunchConfiguration('plan_file').perform(context)
        if mode not in ('execute', 'plan'):
            raise RuntimeError("mode must be execute or plan, not '" + mode + "'.")
        if plan_file == '':
            if mode == 'plan':
                # mode:=plan writes plan_file: no default, so a working plan is
                # never overwritten by mistake.
                raise RuntimeError("mode:=plan needs plan_file:=/home/ros/ur/recordings/<new name>.yaml "
                                   "(the plan is written there).")
            plan_file = DEFAULT_PLAN_FILE
        after_scene.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ur_automata_scan_tsp'),
                             'launch', 'scan_tsp.launch.py')
            ),
            launch_arguments={
                'mode':      mode,
                'plan_file': plan_file,
                'order':     LaunchConfiguration('order'),
                'speed':     LaunchConfiguration('speed'),
            }.items(),
        ))
        message += " the scan (mode " + mode + ", plan " + plan_file + ")"

    # --- Hyperspectral measurement (hsi_node) ---
    if use_hsi:
        after_scene.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ur_automata_hsi'), 'launch', 'hsi.launch.py')
            ),
            launch_arguments={'speed': LaunchConfiguration('speed')}.items(),
        ))
        message += " the HSI node"

    # The scan node checks the plan against the MoveIt scene as soon as it
    # starts, and the HSI node reads the scene for its collision checks, so
    # they must not start before the scene is complete. The scene node waits
    # for move_group, adds the objects and exits: they start then.
    def start_after_scene(event, context):
        if event.process_name.startswith('scene_publisher_node'):
            print(message)
            return after_scene
        return None

    actions.append(RegisterEventHandler(OnProcessExit(on_exit=start_after_scene)))
    return actions


def generate_launch_description():
    cfg = _load_config()

    use_moveit_value = cfg['planning']['use_moveit']
    use_moveit_str_condition = 'true' if use_moveit_value else 'false'

    use_rviz_value = cfg['planning']['use_rviz']
    use_rviz_str_condition = 'true' if use_rviz_value else 'false'

    use_scene_value = cfg['planning']['use_scene']
    use_scene_str_condition = 'true' if use_scene_value else 'false'

    declared_arguments = [
        DeclareLaunchArgument(
            'use_moveit',
            default_value=use_moveit_str_condition,
            description='Launch MoveIt if true. Default is read from automata_config.yaml.'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value=use_rviz_str_condition,
            description='Launch RViz if true. Default is read from automata_config.yaml.'
        ),
        DeclareLaunchArgument(
            'use_scene',
            default_value=use_scene_str_condition,
            description='Spawn the planning-scene objects if true. Default is read from automata_config.yaml.'
        ),
        DeclareLaunchArgument(
            'use_scan',
            default_value='true',
            description='Start scan_tsp_node after the scene is ready (needs use_moveit and use_scene). '
                        'false: robot bring-up and MoveIt only.'
        ),
        DeclareLaunchArgument(
            'use_hsi',
            default_value='true',
            description='Start hsi_node (hyperspectral measurement) after the scene is ready '
                        '(needs use_moveit and use_scene).'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='execute',
            description='execute: load plan_file and wait for the object on /scan3d/start (MQTT bridge on) | '
                        'plan: plan without moving the robot and write plan_file, then the scan node exits.'
        ),
        DeclareLaunchArgument(
            'plan_file',
            default_value='',
            description='Plan to execute (default ' + DEFAULT_PLAN_FILE + ') or to write '
                        '(required with mode:=plan).'
        ),
        DeclareLaunchArgument(
            'order',
            default_value='alternate',
            description='mode:=plan only: alternate | tsp | gtsp | sectors.'
        ),
        DeclareLaunchArgument(
            'speed',
            default_value='0.4',
            description='Speed of the scan (mode:=execute) and of the HSI motions: fraction of '
                        'planning.trajectory_scaling_factor (1.0 = as planned, max 1.0).'
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
