# UNO Q Braccio Gazebo Simulation

This package provides a lightweight Gazebo Harmonic simulation for the UNO Q
Braccio project.

It includes:

- Six Braccio command joints: `base`, `shoulder`, `elbow`, `wrist_vertical`,
  `wrist_rotation`, and `gripper` (plus a mirrored `left_gripper` finger that
  the trajectory bridge drives).
- Visual STL meshes for the Braccio base, links, wrist, and gripper, adapted
  from Will Stedden's GPL-3.0 Braccio MoveIt/Gazebo package.
- A gripper-mounted camera (`/vision/gripper/image_raw`) looking along the
  gripper, and a fixed overhead camera (`/vision/overhead/image_raw`).
- Red, blue and yellow 30 mm cubes in a pick sector, and green, cyan and
  magenta drop bins. Bin colors differ from cube colors so the camera cannot
  mistake one for the other. Red cubes go to green, blue to cyan, yellow to
  magenta.
- `ros2_control` metadata and controller configuration.
- A `ros_gz_bridge` for `/clock` and both camera streams.
- `sim_cube_detector`: the overhead camera. It is the only source of cube and
  bin positions, and it is request-driven (idle until asked).
- `sim_gripper_detector`: the gripper camera. Detection only (is a cube of
  this colour in view, how much of the image it fills); never positions.
- `workspace_markers` and `rviz/braccio.rviz`: RViz view of the robot, both
  camera feeds, the sectors, detected cubes and the task state.

## Install Dependencies

On Ubuntu with ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro
```

## Build

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch unoq_braccio_bringup sim.launch.py
```

Then, in another terminal:

```bash
ros2 run unoq_braccio_driver pose_demo --ros-args -p pose:=ready
ros2 run unoq_braccio_driver pose_demo --ros-args -p pose:=wave
ros2 run unoq_braccio_driver ik_pose_demo --ros-args   -p x:=0.30 -p y:=0.00 -p z:=0.06 -p gripper:=25
ros2 run unoq_braccio_driver pick_place_demo
ros2 topic echo /task/state
```

Launch arguments: `rviz:=false` skips RViz; `detector:=false` skips both
camera detectors;
`fallback_sim:=true` also runs `joint_state_simulator` (debug only, it fights
the controller's `/joint_states`).

If the arm does not move, check the clock first. Every node runs with
`use_sim_time`, so nothing advances without `/clock`:

```bash
ros2 topic hz /clock
ros2 control list_controllers
```

## Servo convention

`ready` (90, 90, 90, 90, 90) is the arm standing straight up; base 90 faces
+x. The URDF joint zeros differ, so `braccio_kinematics.servo_to_urdf`
converts, and `braccio_kinematics.solve_ik` is checked against the URDF chain
(`forward_kinematics`) to a few millimetres. This convention is assumed for
the physical arm too; verify shoulder/elbow directions on hardware before
trusting IK poses there.

## Workspace layout and accuracy

`ros2_ws/src/unoq_braccio_driver/unoq_braccio_driver/braccio_workspace.py` is
the single source of truth for the layout: arm base at the origin facing +x,
overhead camera at (0.20, 0, 0.60) looking straight down (640x480, 1.0 rad
HFOV, about 1 mm per pixel at the cubes, so a 30 mm cube is about 30 px wide),
the pick sector, and the bin sectors. `worlds/workspace.world` must match it.

`python ros2_ws/src/unoq_braccio_driver/test/test_workspace.py` checks, without
ROS or Gazebo, that the world file matches the module, every pick/place target
is reachable and the IK agrees with the URDF forward kinematics to a few
millimetres, the camera sees the whole workspace, cube and bin hues do not
overlap, and pixel-to-table error is under 2 mm. Cube centres are projected at
mid-height because a perspective silhouette centroid is only exact there
(under 1 mm, against up to about 6 mm at the top face).

The overhead detector rejects blobs whose size does not fit a 30 mm cube, and
only averages a cube after several agreeing frames.

## Task states

`pick_place_demo` publishes its state on `/task/state` (details in
`/task/current`): `IDLE`, `GO_HOME`, `DETECTING`, `TARGET_CONFIRMED`,
`MOVE_ABOVE_CUBE`, `VERIFY_CUBE`, `DESCEND`, `GRASP`, `LIFT`, `VERIFY_GRASP`,
`MOVE_TO_BIN`, `LOWER`, `RELEASE`, `RETREAT`, `VERIFY_PLACEMENT`, `COMPLETE`,
and the error states `DETECTION_FAILED`, `TARGET_UNREACHABLE`, `VERIFY_FAILED`.
The two `VERIFY_*` gripper-camera states only warn by default; set
`-p strict_gripper_verify:=true` to abort a cube on a failed check. The final
`VERIFY_PLACEMENT` re-scans with the overhead camera and checks each cube is
inside its bin sector.

## Current Scope

This is a practical development simulation, not a calibrated digital twin. Link
dimensions and inertias are approximate. The pick blocks and bins are there for
vision and workflow testing; grasp physics still needs tuning before relying on
it for realistic pick-and-place contact.

Grasp physics (finger friction on a 30 mm cube) has not been tuned; if a
block slips, adjust the finger geometry or `GRIPPER_CLOSED` in
`braccio_kinematics.py`.

## Reference

This simulation direction is inspired by Will Stedden's Braccio Gazebo/MoveIt
writeup, especially the practical point that the Braccio has a constrained
workspace and benefits from a simple 2D IK approach before full motion planning:

```text
https://opus.stedden.org/2020/08/braccio-moveit-gazebo/
```

That project targeted ROS Melodic and Gazebo Classic. This repository keeps the
same pick/drop playground idea but uses ROS 2 Jazzy and Gazebo Harmonic.

The Braccio STL visual meshes are copied from:

```text
https://github.com/lots-of-things/braccio_moveit_gazebo
```

Those mesh files are isolated under
`ros2_ws/src/unoq_braccio_sim/meshes/braccio_stedden/` with the original GPL-3.0
license text.
