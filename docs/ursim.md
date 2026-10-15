# Simulation with URSim

[← back to the README](../README.md)

URSim is the official Universal Robots simulator: a complete PolyScope
controller in its own Docker container. There is no physics of the environment,
but programs, External Control and safety behave as on the real robot. It is the
safest way to try the whole cell.

## 1. Start URSim (host, not in the ROS container)

```bash
cd ~/automata_ur      # the repo folder
./start_ursim_seccomp.sh
```

The script:

- downloads the *External Control* URCap (`.jar`) into `~/.ursim/e-series/urcaps`;
- creates the Docker network `ursim_net` (`192.168.56.0/24`, gateway `192.168.56.1`);
- starts `universalrobots/ursim_e-series:5.25.1` with the fixed IP **192.168.56.101**;
- keeps programs and settings in `~/.ursim/e-series`;
- uses `--security-opt seccomp=unconfined`: needed on Linux kernels ≥ 6.15,
  where the official `start_ursim.sh` fails (`URControl` gets ENOSYS on `socket()`).

PolyScope: **http://localhost:6080** in a browser (or VNC on `localhost:5900`).
Ctrl+C in this terminal stops and removes the simulator.

## 2. Prepare PolyScope (once)

1. *Power on* → *Start* → *OK* (the arm must show `RUNNING`).
2. *Installation* → *URCaps* → **External Control**: *Host IP* `192.168.56.1`
   (the gateway of `ursim_net`), *Custom port* `50002`.
3. *Program* → add the **External Control** node and save the program.

## 3. Point the cell to URSim

In `automata_config.yaml`:

```yaml
robot:
  ip: 192.168.56.101
  calibration_file: ""
```

```bash
colcon build --packages-select ur_automata_bringup && source install/setup.bash
```

## 4. Plan the scan for URSim (once)

The real-robot plan does not fit URSim: the kinematics differ (nominal vs
calibrated) and the execute mode would refuse it. Compute a plan for the
simulator (6–7 minutes, nothing moves, Play not needed). The plan starts from
the current pose, so first bring the arm to `home` (RViz → *Planning* → goal
state `home` → *Plan & Execute*, with Play on):

```bash
ros2 launch ur_automata_bringup ur_automata_bringup.launch.py mode:=plan plan_file:=/home/ros/ur/recordings/scan_tsp_ursim.yaml
```

(`recordings/scan_tsp.yaml` is an older URSim plan with a 0.30 m sphere: it no
longer matches the config.)

## 5. Run

```bash
ros2 launch ur_automata_bringup ur_automata_bringup.launch.py plan_file:=/home/ros/ur/recordings/scan_tsp_ursim.yaml
```

Press **Play** in PolyScope, then start a scan as in the README
(`mosquitto_pub -t 'automata/scan3d/start' -q 1 -m 'object_1'`).

The HSI measurement works in URSim as well and needs no plan: after the scan
`done`, `mosquitto_pub -t 'automata/hsi/start' -q 1 -m 'object_1'` and answer
by hand on `hsi/status` (see [mqtt.md](mqtt.md)).

Quick check without scanning: `use_scan:=false`, then in RViz drag the
interactive marker, *Plan*, *Execute*: the arm moves in PolyScope too.

Back to the real robot: restore `robot.ip` and `robot.calibration_file`.
