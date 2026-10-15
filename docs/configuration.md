# Configuration and launch files

[← back to the README](../README.md)

## The config file

[`src/automata_robot/ur_automata_bringup/config/automata_config.yaml`](../src/automata_robot/ur_automata_bringup/config/automata_config.yaml)
is the only file to edit for a normal setup. Every launch file reads it.

```yaml
robot:
  ip: 192.168.1.97                       # UR controller (URSim: 192.168.56.101)
  type: ur5                              # ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e
  calibration_file: ur5_calibration.yaml # "" = nominal kinematics (URSim)
  base_xyz: [0.0, 0.0, 0.0]              # base pose in world (m)
  base_rpy: [0.0, 0.0, 0.0]              # (rad)

planning:
  group: ur_manipulator
  global_frame: world
  end_effector_link: ee_automata_tcp     # camera lens center
  trajectory_scaling_factor: 0.4         # speed used when planning
  home_pose_name: home                   # SRDF poses: start and recovery
  lower_home_pose_name: lower_scan_ready
  use_moveit: true                       # defaults of the launch arguments
  use_scene: true
  use_rviz: true

scan:                                    # sphere, waypoints, IK, planners, scene: see scan.md
  center: [0.133, 0.375, 0.455]
  radius: 0.25
  ...

scan3d:                                  # ROS topics of the photo handshake: see mqtt.md
  start_topic: /scan3d/start
  ...
  wait_for_next: true                    # false = do not wait for the camera
  light_topic: /scan3d/light             # scan lights (std_msgs/Bool); "" = no lights
```

Notes:

- `home_pose_name` / `lower_home_pose_name` must exist as `group_state` in
  `ur_automata_moveit_config/config/ur_automata.srdf` (`home`, `up`,
  `lower_scan_ready`, `home_wall`). They are joint values, so they depend on the
  base pose: `home_wall` for a wall-mounted robot.
- A non-zero `base_xyz` / `base_rpy` also adds a mounting wall behind the base
  in the scene. Example of a wall mount: `base_xyz: [0.133, -0.25, 0.455]`,
  `base_rpy: [-1.5708, 0, 0]`.
- The YAML comments keep the history of the positions and settings that were
  tried, with their results.

## Digital I/O and scan lights

`ur_io_ros2_manager` (started with the robot) maps robot digital pins to
`std_msgs/Bool` topics, as listed in
[`src/utils/ur_io_ros2_manager/config/config.yaml`](../src/utils/ur_io_ros2_manager/config/config.yaml):
inputs are published when the pin changes, outputs are set by publishing on
their topic.

```yaml
digital_io:
  - topic_name: /scan3d/light   # scan lights
    io_index: 0                 # standard digital output DO0
    io_type: output
```

`scan_tsp_node` publishes `true` on `scan3d.light_topic` when a start message
arrives (before the robot moves) and `false` when the robot is back in `home`,
right before `done`, also when the scan ends with `ERROR`. When the node starts
it switches the lights off, in case a previous scan was stopped with Ctrl+C.
Switch them by hand with
`ros2 topic pub --once /scan3d/light std_msgs/msg/Bool "{data: true}"`.
The topic name must be the same in both files. After a change in `config.yaml`
rebuild `ur_io_ros2_manager` and restart the bring-up.

## The camera TCP

The end effector is `ee_automata_V2.stl` on `tool0`. The TCP `ee_automata_tcp`
is the **lens center**: `xyz = (0, 0.052, 0.1745)` from `tool0`, optical axis
along its **Y** axis. A new end effector means changing both the mesh and this
offset in `ur_automata_description/urdf/ur_automata.urdf.xacro`, then planning
again.

## What to rebuild after a change

Launch files read the **installed** copies, not `src/`.

| you changed | then |
|---|---|
| `automata_config.yaml`, `mqtt_bridge.yaml`, calibration file | `colcon build --packages-select ur_automata_bringup`, restart the launch |
| digital I/O mapping (`ur_io_ros2_manager/config/config.yaml`) | `ur_io_ros2_manager`, restart the bring-up |
| URDF / xacro, end-effector mesh | `ur_automata_description` |
| SRDF, MoveIt configs | `ur_automata_moveit_config` |
| scene code or meshes | `ur_automata_scene` |
| scan code | `ur_automata_scan`, then `ur_automata_scan_tsp` |
| `docker/Dockerfile` | `./run.sh build` on the host, then a new container |

Always `source install/setup.bash` after a build. After deleting files from a
package, also remove `build/<package>` and `install/<package>`: colcon does not
delete old installed files.

Full build:

```bash
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" -DBUILD_TESTING=OFF --executor sequential
```

`-DBUILD_TESTING=OFF` is needed: the test dependencies of the UR driver are not
in the image. Unit tests of the scan packages:

```bash
colcon build --packages-select ur_automata_scan ur_automata_scan_tsp --cmake-args -DCMAKE_CXX_FLAGS="-w" -DBUILD_TESTING=ON
colcon test --packages-select ur_automata_scan ur_automata_scan_tsp && colcon test-result --verbose
```

## Launch files

| launch file | starts |
|---|---|
| `ur_automata_bringup ur_automata_bringup.launch.py` | **everything**: robot, MoveIt, scene, RViz and the scan |
| `ur_automata_bringup ur_automata_control.launch.py` | robot only: UR driver, controllers, digital I/O (`ur_io_ros2_manager`) |
| `ur_automata_bringup ur_automata_moveit.launch.py` | MoveIt (`move_group`), RViz, collision scene |
| `ur_automata_scan_tsp scan_tsp.launch.py` | scan node (+ MQTT bridge in execute), when the rest already runs |
| `ur_automata_moveit_config setup_assistant.launch.py` | MoveIt Setup Assistant, to edit the MoveIt configuration |

### `ur_automata_bringup.launch.py`

| argument | default | |
|---|---|---|
| `use_scan` | `true` | start the scan when the scene is ready; `false` = robot and MoveIt only |
| `mode` | `execute` | `execute` = load the plan and wait for objects · `plan` = compute a plan, the robot does not move |
| `plan_file` | `recordings/scan_tsp_real.yaml` | plan to execute; **required** with `mode:=plan` (so an existing plan is never overwritten) |
| `order` | `alternate` | `plan` only: `alternate`, `tsp`, `gtsp`, `sectors` |
| `speed` | `0.3` | `execute` only: fraction of the planned speed (max 1.0) |
| `use_moveit`, `use_rviz`, `use_scene` | from the config | the scan needs MoveIt and the scene |

The scan node starts only when the scene node has finished loading the
collision objects in MoveIt: it checks the plan against the scene right away.

### `ur_automata_control.launch.py`

| argument | default | |
|---|---|---|
| `ur_type`, `robot_ip` | from the config | |
| `kinematics_params_file` | `robot.calibration_file` | also an argument of the MoveIt launch: override both or neither |
| `headless_mode` | `false` | `true` = no External Control program on the pendant |
| `initial_joint_controller` | `scaled_joint_trajectory_controller` | MoveIt expects this one |

`ur_type` and the calibration must be the same for driver and MoveIt, otherwise
the plans are computed on a different robot than the real one.

### `scan_tsp.launch.py`

`mode`, `order`, `plan_file`, `speed` as above (here `plan_file` defaults to
`recordings/scan_tsp.yaml` and `speed` to 1.0), plus `tsp_time_limit`,
`refine_iterations`, `blocked_penalty_s`, `tsp_check_neighbours`,
`start_tolerance_rad`: see the
[package README](../src/automata_robot/ur_automata_scan_tsp/README.md).

## Docker

`run.sh` wraps `docker compose` and derives the names from the folder:

| | value |
|---|---|
| image | `ur_ws:jazzy` |
| container | `ur_container` |
| workspace in the container | `/home/ros/ur` (the repo, mounted) |
| network | `host` (ROS discovery, robot link, MQTT on localhost) |
| `ROS_DOMAIN_ID` | `44` |

```bash
./run.sh build     # build the image (cached)
./run.sh rebuild   # rebuild from scratch
./run.sh run       # shell in a new container
./run.sh down      # stop and remove it
```

The container installs nothing at startup: system dependencies go in
`docker/Dockerfile`, then `./run.sh build`.
