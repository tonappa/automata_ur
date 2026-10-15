# Real robot

[← back to the README](../README.md)

The cell runs a **UR5 (CB3)** at `192.168.1.97`. Everything below also applies to
another UR arm: change the values in `automata_config.yaml`.

## 1. Network

Connect the PC to the UR controller over Ethernet, on the same subnet. On the
teach pendant: *Settings* → *System* → *Network*, static IP. From the host:

```bash
ping 192.168.1.97
```

## 2. External Control program (once)

1. Install the [External Control URCap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap)
   on the robot.
2. *Installation* → *URCaps* → **External Control**: *Host IP* = IP of the PC
   running ROS, *Custom port* = `50002`.
3. *Program* → add the **External Control** node, save the program
   (e.g. `externalcontrol.urp`).

## 3. Kinematic calibration (once per robot)

Every UR leaves the factory with its own measured kinematic parameters: without
them the error at the tool can reach several millimeters. Do this **once for
each physical robot**, with the controller on and reachable. The robot does not
move and External Control is not needed.

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.97 \
  target_filename:="/home/ros/ur/src/automata_robot/ur_automata_bringup/config/<robot>_calibration.yaml"
```

The cell's UR5 is already calibrated: `ur5_calibration.yaml`.

> The UR driver is a git submodule pinned to a commit that contains the
> `ur_calibration` fix (upstream `0089621`): the 3.9.0 `.deb` package lacks it
> and loops on `Stream is invalid. Connect it before trying to read!`.

## 4. Configuration

In [`automata_config.yaml`](../src/automata_robot/ur_automata_bringup/config/automata_config.yaml):

```yaml
robot:
  ip: 192.168.1.97
  type: ur5                                # the actual model
  calibration_file: ur5_calibration.yaml   # in ur_automata_bringup/config; "" = nominal
```

Then:

```bash
colcon build --packages-select ur_automata_bringup && source install/setup.bash
```

Both the driver **and** MoveIt read `calibration_file`, so planning, collision
checks and the real robot use the same model (the MoveIt launch prints
`[ur_automata_moveit] kinematics: <file>`). After changing the calibration,
compute the scan plans again (`mode:=plan`).

## 5. Power-on sequence

1. Turn on the controller and release the brakes (*Power on* → *Start*).
2. Load the program with the **External Control** node.
3. Start ROS:

   ```bash
   ros2 launch ur_automata_bringup ur_automata_bringup.launch.py
   ```

4. Press **Play** on the pendant. The driver prints
   `Robot connected to reverse interface. Ready to receive control commands.`

> **Every time the bring-up is restarted, press Play again**: when the driver
> stops, the program on the controller ends.

The robot controllers can be checked with:

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers {} \
  | grep -o "name='[a-z_]*'\|state='[a-z]*'"
```

`scaled_joint_trajectory_controller` must be `active`.

## 6. First motion

- Set the PolyScope **speed slider** to 20–30 %: the
  `scaled_joint_trajectory_controller` follows it and slows the motion without
  changing its path.
- Start with `speed:=0.3` (the default of the launcher) or lower.
- Keep the emergency stop within reach and check in RViz that the scene (table,
  platform, object) really matches the cell before a full scan.
- Planning a new scan (`mode:=plan`) never moves the robot.

## Simulation vs real robot

| | URSim | Real robot |
|---|---|---|
| `robot.ip` | `192.168.56.101` | controller IP (`192.168.1.97`) |
| `robot.type` | `ur5` | the real model (`ur5`, CB3) |
| `robot.calibration_file` | `""` (nominal) | **required** (`ur5_calibration.yaml`) |
| Play | in PolyScope (browser) | on the teach pendant, after every bring-up |
| plan file | one planned in URSim (see [ursim.md](ursim.md)) | `recordings/scan_tsp_real.yaml` (launcher default) |

Controllers, MoveIt, scene and scan nodes are the same.
