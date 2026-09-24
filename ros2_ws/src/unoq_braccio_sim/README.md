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
- Red, blue, and yellow pick blocks and three colored drop bins, all inside
  the arm's reach.
- `ros2_control` metadata and controller configuration.
- A `ros_gz_bridge` for `/clock` and both camera streams.
- `sim_cube_detector`: request-driven cube detection on the overhead camera.

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
```

Launch arguments: `detector:=false` skips the cube detector;
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

## Current Scope

This is a practical development simulation, not a calibrated digital twin. Link
dimensions and inertias are approximate. The pick blocks and bins are there for
vision and workflow testing; grasp physics still needs tuning before relying on
it for realistic pick-and-place contact.

Grasp physics (finger friction on a 50 mm block) has not been tuned; if a
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
