# SO-101 Arm — MoveIt2 Control

MoveIt2 control of a SO-ARM100/SO-101 arm, in simulation and on real
hardware.

## Branches

| Branch | What it runs |
|---|---|
| `main` | MoveIt2 + MuJoCo simulation |
| `so101_hardware` | Real hardware, via a lerobot-backed `ros2_control` bridge (this doc) |


### `main` — simulation

<video src="https://github.com/user-attachments/assets/d9211cfa-1904-4dcd-8ea7-1023dfa931cb" width="480" controls></video>

```bash
mkdir -p so_100_ws/src && cd so_100_ws/src
git clone https://github.com/iAdityaDev/so_101_arm.git .
cd ..
colcon build --symlink-install
ros2 launch so101_mujoco mujoco.launch.py
```
Brings up MuJoCo, `robot_state_publisher`, `move_group`, RViz, and spawns
`joint_state_broadcaster` + `arm_controller` + `gripper_controller`
automatically

---

## `so101_hardware` branch — real hardware bridge

<video src="https://github.com/user-attachments/assets/4afae117-0115-45f9-b2b0-257acea641ea" width="480" controls></video>

```bash
git checkout so101_hardware   # for everything below this point
```

Drives the real arm through MoveIt2 by bridging `ros2_control` to
[lerobot](https://github.com/huggingface/lerobot)'s `SO101Follower`, instead
of a native serial driver. Drop-in replacement for the `main` branch's
MuJoCo plugin — MoveIt, SRDF, and controllers config are unchanged between
branches.

**Hardware:** 5 motors installed (`shoulder_pan`, `shoulder_lift`,
`elbow_flex`, `wrist_flex`, `wrist_roll`). Gripper (motor 6) not installed.

## Architecture

```
MoveIt -> joint_trajectory_controller -> controller_manager
              |
   SO101HardwareInterface (C++, so101_hardware)
   write()/read() <-> JointState, radians
              |
   /so101/hardware_commands, /so101/hardware_states  (ROS2 topics, matched by name)
              |
   feetech_bridge_node (Python, so101_bridge)
   renames joints, rad<->deg, safety clamps
              |
   lerobot SO101Follower / FeetechMotorsBus
   deg<->raw ticks via calibration JSON
              |
   Feetech STS3215 servos (serial)
```

| Package | Type | Role |
|---|---|---|
| `so101_hardware` | `ament_cmake` (C++) | `SystemInterface` plugin. Topics only, no serial. |
| `so101_bridge` | `ament_python` | Owns the serial connection via lerobot. |

URDF/xacro, MoveIt config, `controllers.yaml`, SRDF live in `so_arm_100_description`
and `so_arm_100_moveit_config` (both present on every branch — only the
`<ros2_control>` hardware plugin block differs between `main` and
`so101_hardware`).

## Prerequisites

- ROS2 + `ros-<distro>-ros2-control ros-<distro>-ros2-controllers ros-<distro>-pluginlib`
- Completed `lerobot-calibrate` for this 5-motor config, JSON at
  `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<robot_id>.json`
- Local lerobot clone with `so_follower.py`'s gripper `Motor(6, ...)` entry
  commented out (5 motors remain)

## Environment

`rclpy` and `lerobot` must share one Python env. Conda conflicts with
`rclpy`'s system-linked libraries — use a venv instead:

```bash
python3 -m venv ~/ros2_lerobot_env --system-site-packages
source ~/ros2_lerobot_env/bin/activate
python3 -m pip install --upgrade pip setuptools wheel
cd ~/lerobot && pip install -e '.[feetech]'
python3 -c "import rclpy; import lerobot; import scservo_sdk; print('OK')"
```

`feetech-servo-sdk` (imports as `scservo_sdk`) only comes via the `[feetech]`
extra — not a base dependency.

If `pandas`/`PIL`/`scipy`/etc. throw binary-incompatibility errors:
`--system-site-packages` is shadowing venv packages with older apt ones.
Fix per-package: `pip install --upgrade --force-reinstall <package>`.

Every terminal, every time, in this order, no conda active:
```bash
source /opt/ros/<distro>/setup.bash
source ~/ros2_lerobot_env/bin/activate
```
`colcon build` must run with the venv active — the active interpreter gets
baked into each package's launcher shebang. If `ros2 run` fails with
`ModuleNotFoundError: lerobot`, check `head -1
install/so101_bridge/lib/so101_bridge/feetech_bridge_node`; rebuild clean if
it points at system Python instead of the venv.

## Build

```bash
cd ~/so101_moveit_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## URDF integration

```xml
<plugin>so101_hardware/SO101HardwareInterface</plugin>
<param name="commands_topic">/so101/hardware_commands</param>
<param name="states_topic">/so101/hardware_states</param>
```
No serial port/baud/speed params in the URDF — those live in the bridge
node's launch params and constants below. Command-interface min/max should
match the link URDF's real `<limit>` values.


## Launch (4 terminals, in order, all left running)

**1 — bridge:**
```bash
python3 /so101_bridge/feetech_bridge_node.py
```

**2 — robot_state_publisher:**
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro $(ros2 pkg prefix so_arm_100_moveit_config)/share/so_arm_100_moveit_config/config/so_arm_100.urdf.xacro use_sim:=false use_fake_hardware:=false)"
```

**3 — controller_manager** 
```bash
ros2 run controller_manager ros2_control_node --ros-args \
  --remap '~/robot_description:=/robot_description' \
  --params-file $(ros2 pkg prefix so_arm_100_moveit_config)/share/so_arm_100_moveit_config/config/hardware_controllers.yaml
```

**4 — controllers + MoveIt:**
```bash
ros2 launch so_arm_100_moveit_config demo.launch.py
```
`arm_controller` (`joint_trajectory_controller`) covers all 5 real joints —
`Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll`.
`gripper_controller` also exists in `hardware_controllers.yaml`, unused
until motor 6 is installed.


## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `pluginlib::LibraryLoadException ... does not exist` | Stale build / not sourced in this terminal | `rm -rf build/so101_hardware install/so101_hardware log && colcon build --packages-select so101_hardware` |
| `ModuleNotFoundError: lerobot` via `ros2 run` | Built without venv active | Check shebang, rebuild with venv active |
| `FeetechMotorsBus motor check failed ... Missing motor IDs` | `so_follower.py` motor list doesn't match installed motors | Fix the list, recalibrate |
| `Bridge node never reported a position for joint 'X'` | Joint name not in `joint_name_map` | Fix the mapping |
| `name 'prefix' is not defined` (xacro) | `prefix` used but not declared as macro param | Add `prefix:=^|''` to `params=` |
| `controller_manager` hangs on `Subscribing to '~/robot_description'` | Unquoted tilde remap, bash-expanded | Quote it: `'~/robot_description:=/robot_description'` |
| `numpy.dtype size changed` / similar | apt package shadowing venv package | `pip install --upgrade --force-reinstall <pkg>` |

## Future Contributions

- Native C++ Feetech serial driver, once this pipeline is fully trusted.
- Add gripper (motor 6): uncomment in `so_follower.py`, recalibrate, add its
  `<joint>` block to the xacro. `feetech_bridge_node.py` needs no changes —
  it iterates `self._robot.bus.motors` dynamically and setup gripper controller for same.
