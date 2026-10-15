# Troubleshooting

[← back to the README](../README.md)

## Build

| Symptom | Cause / fix |
|---|---|
| `Could not find ... ros2_control_test_assets` while building `ur_controllers` | tests are built by default: add `-DBUILD_TESTING=OFF` to `colcon build` |
| `OR-Tools not found in /opt/ortools: ur_automata_scan_tsp is not built` | the image is older than the OR-Tools step of the Dockerfile: `./run.sh build`, then a new container |
| I edit `automata_config.yaml` and nothing changes | the launch files read the installed copy: `colcon build --packages-select ur_automata_bringup` + `source install/setup.bash` |
| Deleted files still show up | colcon does not remove installed files: delete `build/<pkg>` and `install/<pkg>` and build again |
| `ros2 control` is not a known command | the CLI is not in the image: `ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {}` |

## Robot and driver

| Symptom | Cause / fix |
|---|---|
| The driver starts but the robot does not move | **Play** on External Control is missing: press it again after every bring-up restart (or `headless_mode:=true`) |
| `Action client not connected to action server: scaled_joint_trajectory_controller/...` | the controller is not active: check Play, then the controller list (see [real_robot.md](real_robot.md)). If it is not even loaded, the controllers file and the driver are out of sync: `ur_automata_controllers.yaml` must contain every controller of the driver's `ur_controllers.yaml` |
| `Undefined parameters [ur_type]` from xacro | the URDF must pass `ur_type` to the driver's `ur_ros2_control` macro (driver ≥ upstream e9ac441) |
| `Can't accept new action goals. Controller is not running` | the program on the controller is not running: Play |
| Plausible trajectories but wrong positions | `ur_type` or the calibration differ between driver and MoveIt, or the calibration file is missing on the real robot |
| `ur_calibration` loops on `Stream is invalid` | the driver is older than the `ur_calibration` fix: keep the submodule at the pinned commit |

## URSim

| Symptom | Cause / fix |
|---|---|
| URSim does not start, `URControl` errors | kernel ≥ 6.15: use `./start_ursim_seccomp.sh`, not the official script |
| PolyScope does not show the URCap | the `.jar` must be in `~/.ursim/e-series/urcaps` |
| External Control does not connect | Host IP must be `192.168.56.1` (the `ursim_net` gateway), not `127.0.0.1` |

## Scan

| Symptom | Cause / fix |
|---|---|
| `Piano NON riutilizzabile` and the scan node exits | the cell changed since the plan was made (it lists what): fix it or plan again with `mode:=plan` |
| The scan stays on a waypoint | no `measure_complete` from the camera: is the camera app running and `ready`? Is Mosquitto up? Watch with `mosquitto_sub -t 'vimbax/#' -v` |
| `Nessuno ascolta su /scan3d/light` and the lights stay off | `ur_io_ros2_manager` is not running, or the topic names in `scan3d.light_topic` and in its `config.yaml` differ |
| A start message does nothing | a scan is already running (start ignored), or the name is only a number / `true` (the bridge turns it into another type) |
| A scan starts by itself when the launch starts | the orchestrator sent `start` with the retain flag: clear it with `mosquitto_pub -t 'automata/scan3d/start' -r -n` |
| `mode:=plan needs plan_file:=...` | intended: give a new file name so the working plan is not overwritten |
| `No kinematics solver instantiated for group ur_manipulator` | a hand-written node did not load `kinematics.yaml` under `robot_description_kinematics` |
| Many waypoints fail IK | `radius` too large or `center` too far from the base |
| Some waypoints are `OCCLUDED` | a scene object hides the center from that waypoint: expected (see [scan.md](scan.md)) |
| Plan fails with `INVALID_MOTION_PLAN` / `Computed path is not valid` | the path grazes a thin obstacle: lower `longest_valid_segment_fraction` in `ompl_planning.yaml` |
| `Cannot find planning configuration ... kConfigDefault` | the OMPL algorithm is missing from `planner_configs` in `ompl_planning.yaml` |
| Only a generic FAILURE is reported | the real cause is in the move_group log: `ls -t ~/.ros/log/move_group_*.log \| head -1` in the container |

## Display

| Symptom | Cause / fix |
|---|---|
| RViz does not open | `xhost +local:docker` (done by `./run.sh run`) and `DISPLAY` set on the host |
| No waypoint markers in RViz | RViz must use `automata.rviz` (the bring-up does) with the **Scan waypoints** display on `/scan_waypoints_markers` |
