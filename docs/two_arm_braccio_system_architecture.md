# Two-Arm Braccio Robotics System --- Detailed Architecture

## 1. System Overview

This document defines the event-ready architecture for a two-arm Braccio
robotic system built around:

-   Raspberry Pi 5 as the main ROS 2 computer.
-   Two TinkerKit Braccio robotic arms.
-   Arduino UNO Q controlling Arm 1.
-   Arduino UNO controlling Arm 2.
-   Two Grove Vision AI camera systems, each using a Raspberry Pi
    camera.
-   One camera mounted near the gripper of Arm 1 for live/first-person
    visualization.
-   One fixed camera mounted above the workspace for authoritative cube
    detection.
-   25 × 25 mm colored cubes.
-   A QR-code handoff station at the center of the workspace.
-   ROS 2 as the main middleware.
-   A web dashboard accessed from a second computer.
-   Gazebo/URDF simulation.
-   A USB control pad/joystick connected to the Raspberry Pi 5.

The main workflow is:

**Fixed overhead vision → Arm 1 picks cube → Arm 1 places cube at QR
handoff → Arm 2 picks cube → Arm 2 places cube at destination.**

The system is divided into perception, calibration, planning,
coordination, low-level control, visualization, and operator-interface
layers.

------------------------------------------------------------------------

## 2. High-Level Architecture

``` text
                         ┌───────────────────────────────┐
                         │       SECOND COMPUTER          │
                         │ Web Browser / Dashboard       │
                         │ Camera feeds                  │
                         │ Robot status                  │
                         │ Digital robot visualization  │
                         └───────────────┬───────────────┘
                                         │ Wi-Fi / Ethernet
                                         ▼
┌───────────────────────────────────────────────────────────────────────┐
│                         RASPBERRY PI 5                                │
│                         ROS 2 HOST                                    │
│                                                                       │
│  ┌────────────────┐   ┌──────────────────┐   ┌────────────────────┐ │
│  │ Fixed Camera   │──►│ Cube Detection   │──►│ Coordinate         │ │
│  │ Grove + Pi Cam │   │ / Vision Node    │   │ Transformation      │ │
│  └────────────────┘   └──────────────────┘   └─────────┬──────────┘ │
│                                                         │            │
│                                                         ▼            │
│                                                ┌─────────────────┐   │
│                                                │ Task Manager /  │   │
│                                                │ State Machine   │   │
│                                                └───────┬─────────┘   │
│                                                        │             │
│                              ┌─────────────────────────┴──────────┐  │
│                              ▼                                    ▼  │
│                    ┌─────────────────┐                    ┌─────────┐│
│                    │ IK / Arm 1      │                    │ IK /    ││
│                    │ Pick + Handoff  │                    │ Arm 2   ││
│                    └────────┬────────┘                    └────┬────┘│
│                             │                                  │     │
│                             ▼                                  ▼     │
│                    USB → Arduino UNO Q                  USB → Arduino│
└─────────────────────────────┼──────────────────────────────────┼─────┘
                              ▼                                  ▼
                         ┌──────────┐                       ┌──────────┐
                         │ Braccio 1│                       │ Braccio 2│
                         │ Pick     │                       │ Handoff  │
                         │ Handoff  │                       │ Place    │
                         └──────────┘                       └──────────┘
```

------------------------------------------------------------------------

## 3. Physical System

### 3.1 Arm 1

Arm 1 is the primary perception/manipulation robot.

Hardware:

-   TinkerKit Braccio.
-   Arduino UNO Q.
-   Braccio servo interface.
-   Grove Vision AI system.
-   Raspberry Pi camera connected to the Grove Vision AI system.
-   Camera mounted near/on the gripper.
-   USB connection between UNO Q and Raspberry Pi 5.
-   3D-printed camera mount and cable management as required.

Responsibilities:

1.  Receive cube target coordinates from ROS 2.
2.  Move to the cube.
3.  Pick the cube.
4.  Move to the central QR handoff position.
5.  Place the cube.
6.  Signal that handoff is complete.
7.  Return to READY.
8.  Provide the gripper-camera feed to the dashboard.

The gripper camera is **not the authoritative source for cube
coordinates**. Its primary role is visual presentation, with optional
future verification.

### 3.2 Arm 2

Arm 2 is the receiving/manipulation robot.

Hardware:

-   TinkerKit Braccio.
-   Arduino UNO.
-   Braccio servo interface.
-   USB serial connection to Raspberry Pi 5.

Responsibilities:

1.  Remain in READY/WAIT state while Arm 1 works.
2.  Wait for `HANDOFF_READY`.
3.  Move to the known QR handoff pose.
4.  Pick the cube.
5.  Move to its destination.
6.  Place the cube.
7.  Return to READY.
8.  Signal completion.

Arm 2 does not require the same vision system as Arm 1 because the
handoff location is deterministic.

------------------------------------------------------------------------

## 4. Vision Architecture

### 4.1 Fixed Overhead Camera

The fixed overhead camera is the authoritative perception sensor.

Hardware:

-   Grove Vision AI.
-   Raspberry Pi camera.
-   Rigid overhead mount.
-   Fixed height above the table.

It detects:

-   Cube color.
-   Cube position in image coordinates.
-   Detection confidence.
-   Optionally the QR marker.

The camera should not move during the event.

### 4.2 Gripper Camera

The second Grove Vision AI + Raspberry Pi camera is mounted on Arm 1
near the gripper.

Its primary purpose is:

-   First-person visual demonstration.
-   Live feed to the web dashboard.
-   Showing the audience what the robot sees.
-   Optional future close-range verification.

Because this camera moves with the robot, it is intentionally excluded
from the primary cube-coordinate calculation.

This avoids continuously recalculating the camera pose as the arm moves.

------------------------------------------------------------------------

## 5. Cube Detection

The physical objects are 25 × 25 mm colored cubes.

The fixed camera should produce detections containing:

``` text
color
image_x
image_y
confidence
timestamp
```

A detection should not immediately trigger a pick.

Instead:

``` text
READY
  ↓
DETECTION_REQUEST
  ↓
COLLECT DETECTIONS
  ↓
FILTER LOW-CONFIDENCE RESULTS
  ↓
REMOVE OUTLIERS
  ↓
AVERAGE POSITION
  ↓
TARGET_CONFIRMED
```

Example configuration:

``` yaml
detection:
  sample_duration_ms: 1500
  minimum_confidence: 0.70
  minimum_samples: 10
  maximum_samples: 50
  position_outlier_threshold_mm: 20
```

The exact values must be experimentally calibrated.

------------------------------------------------------------------------

## 6. Detection Averaging

Suppose the camera reports:

``` text
X = 210
X = 212
X = 208
X = 211
X = 209
```

The system calculates the representative position:

``` text
X ≈ 210 mm
```

The same procedure is applied to Y.

If the measurements are too widely dispersed, the target should be
rejected and a new detection window started.

This prevents one bad frame from generating a dangerous robot command.

------------------------------------------------------------------------

## 7. Camera-to-Table Calibration

The fixed camera establishes a stable relationship:

``` text
Camera image coordinates
        ↓
Calibration transform
        ↓
Table/world coordinates
        ↓
Robot coordinates
```

A calibration procedure should use several known physical points on the
table.

For a flat workspace, a planar homography is a practical first
implementation.

The calibration should be stored in a configuration file rather than
hard-coded into the detection node.

------------------------------------------------------------------------

## 8. Robot Calibration

Each Braccio needs independent calibration.

For each arm calibrate:

-   Base zero.
-   Shoulder zero.
-   Elbow zero.
-   Wrist vertical zero.
-   Wrist rotation zero.
-   Gripper limits.
-   Physical workspace.
-   Table height.
-   Pickup height.
-   Handoff pose.
-   Destination poses.

Do not assume the two physical arms have identical zero positions.

------------------------------------------------------------------------

## 9. QR Handoff Station

A QR code is placed near the center of the workspace.

It represents the shared handoff station:

``` text
             ARM 1
               ↓
        ┌─────────────┐
        │     QR      │
        │   HANDOFF   │
        └─────────────┘
               ↑
             ARM 2
```

The QR marker provides a physical reference for humans and cameras.

The robot should nevertheless have a predefined handoff pose:

``` yaml
handoff:
  x: ...
  y: ...
  z: ...
  wrist_vertical: ...
  wrist_rotation: ...
  gripper_open: ...
```

Thus the QR code is both:

1.  A visible physical marker.
2.  A calibration/reference feature.

------------------------------------------------------------------------

## 10. Complete Pick-and-Handoff Workflow

### Step 1 --- READY

Arm 1:

``` text
READY
```

Arm 2:

``` text
WAITING_FOR_HANDOFF
```

### Step 2 --- Detect

The overhead camera becomes authoritative.

The system collects detections for a configurable window.

Example:

``` text
Detection window = 1.5 seconds
```

Result:

``` text
target_color = BLUE
target_x = 182 mm
target_y = 96 mm
target_z = calibrated_pick_height
```

### Step 3 --- Plan

The target is sent to the Arm 1 IK solver.

Before moving, check:

-   Joint limits.
-   Reachability.
-   Workspace limits.
-   Safe approach height.
-   Collision constraints.

### Step 4 --- Pick

Arm 1:

1.  Moves above the cube.
2.  Moves down to pickup height.
3.  Closes the gripper.
4.  Waits briefly for settling.
5.  Raises the cube.

### Step 5 --- Handoff

Arm 1:

1.  Approaches the QR station.
2.  Descends to handoff height.
3.  Releases the cube.
4.  Retreats.
5.  Publishes `HANDOFF_READY`.

------------------------------------------------------------------------

## 11. Arm 2 Handoff

Arm 2 receives:

``` text
HANDOFF_READY
```

It then:

1.  Moves to the known handoff pose.
2.  Opens the gripper.
3.  Moves to cube pickup height.
4.  Closes the gripper.
5.  Raises the cube.
6.  Leaves the handoff zone.

Arm 1 must not enter the handoff zone while Arm 2 owns it.

------------------------------------------------------------------------

## 12. Arm 2 Placement

Destinations can be selected according to cube color.

Example:

``` yaml
destinations:
  red:
    x: ...
    y: ...
    z: ...
  blue:
    x: ...
    y: ...
    z: ...
  yellow:
    x: ...
    y: ...
    z: ...
```

Therefore:

``` text
RED    → destination A
BLUE   → destination B
YELLOW → destination C
```

The detected color becomes part of the task data.

------------------------------------------------------------------------

## 13. ROS 2 Node Architecture

Recommended nodes:

``` text
grove_overhead_detector
grove_gripper_streamer
cube_coordinate_transform
robot_calibration
task_manager
arm1_ik
arm2_ik
arm1_controller
arm2_controller
handoff_manager
control_pad_node
camera_stream_server
web_dashboard
robot_state_publisher
joint_state_publisher
gazebo_bridge
data_capture
```

------------------------------------------------------------------------

## 14. ROS 2 Topics

### Arm 1

``` text
/braccio1/joint_command
/braccio1/joint_states
/braccio1/status
/braccio1/state
```

### Arm 2

``` text
/braccio2/joint_command
/braccio2/joint_states
/braccio2/status
/braccio2/state
```

### Vision

``` text
/vision/overhead/image_raw
/vision/gripper/image_raw
/vision/cube_detection
/vision/cube_target
/vision/detection_status
```

### Handoff

``` text
/handoff/qr_pose
/handoff/state
/handoff/ready
/handoff/complete
```

### System

``` text
/robot/mode
/robot/state
/task/current
/task/result
```

------------------------------------------------------------------------

## 15. Suggested Cube Detection Message

A custom ROS 2 message is preferable to encoding detections in a String.

Example:

``` text
CubeDetection.msg

string color
float32 x
float32 y
float32 z
float32 confidence
uint32 sample_count
builtin_interfaces/Time timestamp
```

Later, support multiple objects using:

``` text
CubeDetectionArray
```

------------------------------------------------------------------------

## 16. Task Manager

The task manager is the central coordinator.

It owns the high-level workflow:

``` text
Vision
  ↓
Target
  ↓
Arm 1
  ↓
Handoff
  ↓
Arm 2
  ↓
Destination
  ↓
Complete
```

Independent nodes should not be allowed to issue conflicting autonomous
commands.

------------------------------------------------------------------------

## 17. State Machine

Recommended state machine:

``` text
IDLE
  ↓
READY
  ↓
SEARCHING
  ↓
DETECTING
  ↓
TARGET_CONFIRMED
  ↓
ARM1_PICKING
  ↓
ARM1_CARRYING
  ↓
ARM1_HANDOFF
  ↓
HANDOFF_READY
  ↓
ARM2_PICKING
  ↓
ARM2_CARRYING
  ↓
ARM2_PLACING
  ↓
TASK_COMPLETE
  ↓
READY
```

Error states:

``` text
DETECTION_FAILED
TARGET_UNREACHABLE
PICK_FAILED
HANDOFF_FAILED
ARM_LIMIT_ERROR
COMMUNICATION_ERROR
EMERGENCY_STOP
```

------------------------------------------------------------------------

## 18. Control Pad

The control pad connects to Raspberry Pi 5.

It should control high-level modes rather than directly commanding
individual servos.

Suggested modes:

### AUTO

Full workflow:

``` text
Detect → Pick → Handoff → Pick → Place
```

### TELEOP

Manual control for testing, calibration, and recovery.

### CALIBRATION

Move robots to known points and save calibration data.

### READY

Move robots to predefined safe positions.

### DEMO

Execute a predefined presentation sequence.

### STOP

Stop autonomous operation.

------------------------------------------------------------------------

## 19. Web Dashboard

A second computer connects to the Raspberry Pi through Wi-Fi/Ethernet.

The dashboard should provide:

### Camera views

``` text
OVERHEAD CAMERA
GRIPPER CAMERA
```

The overhead view can show:

-   Cube bounding boxes.
-   Cube colors.
-   Target coordinates.
-   QR handoff location.
-   Detection confidence.

The gripper view shows the first-person perspective.

### Robot state

Show:

``` text
Arm 1: READY / PICKING / HANDOFF / ...
Arm 2: WAITING / PICKING / PLACING / ...
System: AUTO / TELEOP / CALIBRATION / STOP
```

### Task information

Show:

``` text
Current cube
Color
Target position
Current state
Handoff status
Task result
```

------------------------------------------------------------------------

## 20. Digital Robot Visualization

The existing URDF/Gazebo work should be retained and expanded to two
arms.

The digital visualization should show:

-   Arm 1.
-   Arm 2.
-   Joint positions.
-   Target cube.
-   QR handoff point.
-   Destination zones.

The real robot's ROS joint states can drive the digital model:

``` text
REAL ARM
   ↓
ROS joint state
   ↓
Digital robot
   ↓
Web/Gazebo visualization
```

The first event version does not need to be a perfectly calibrated
digital twin. The priority is visibly following the real robots.

------------------------------------------------------------------------

## 21. Gazebo Simulation

Expand the original single-arm model to:

-   Arm 1.
-   Arm 2.
-   Separate joint namespaces.
-   Shared table.
-   Cube objects.
-   QR handoff marker.
-   Destination zones.
-   Optional camera models.
-   ROS 2 control interfaces.

Use namespaces such as:

``` text
/braccio1/
/braccio2/
```

The high-level task logic should ideally be usable in both simulation
and hardware.

------------------------------------------------------------------------

## 22. Real/Simulation Interface

The desired architecture is:

``` text
Task Manager
     ↓
Joint Command
     ↓
Hardware Bridge OR Gazebo Controller
```

This makes the high-level software independent of the physical transport
layer.

------------------------------------------------------------------------

## 23. Arm 1 Communication

``` text
Raspberry Pi 5
     │ USB
     ▼
Arduino UNO Q
     │
     ▼
Braccio 1
```

The existing serial command protocol can be retained:

``` text
M <base> <shoulder> <elbow> <wrist_vertical> <wrist_rotation> <gripper>
```

The existing `/braccio/joint_command` abstraction should be extended to
`/braccio1/joint_command`.

------------------------------------------------------------------------

## 24. Arm 2 Communication

``` text
Raspberry Pi 5
     │ USB
     ▼
Arduino UNO
     │
     ▼
Braccio 2
```

The UNO firmware should expose an equivalent command interface so ROS 2
does not need to care which Arduino is controlling the arm.

------------------------------------------------------------------------

## 25. Why the Two Arduino Controllers Differ

Arm 1 uses UNO Q because it is already integrated into the original
project.

Arm 2 uses a standard Arduino UNO because the second arm only requires
low-level servo control.

Raspberry Pi 5 handles:

-   ROS 2.
-   Vision.
-   Calibration.
-   IK.
-   Task planning.
-   Coordination.
-   Web interface.
-   Logging.

Arduino boards handle:

-   Servo commands.
-   Low-level Braccio control.
-   Hardware-specific timing.

------------------------------------------------------------------------

## 26. Grove Vision AI Device Management

Each Grove Vision AI unit communicates with Raspberry Pi 5 over USB
Type-C.

The software must distinguish the two devices reliably.

Avoid relying only on changing `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc.

Use persistent USB/device identification where possible.

Logical roles should be:

``` text
GROVE_OVERHEAD
GROVE_GRIPPER
```

------------------------------------------------------------------------

## 27. Gripper Camera Streaming

The gripper camera follows:

``` text
Gripper Grove Vision AI
        ↓
Raspberry Pi 5
        ↓
Camera streaming node
        ↓
Web server
        ↓
Second computer
```

It is primarily a visualization channel.

------------------------------------------------------------------------

## 28. Safety

Because two physical robots share a workspace, safety must be explicit.

Minimum requirements:

-   Physical emergency stop.
-   Software STOP state.
-   Joint limits.
-   Workspace limits.
-   Maximum joint velocity.
-   Safe READY poses.
-   Handoff exclusion zone.
-   Communication timeout handling.
-   Controlled servo power shutdown where practical.

If the Raspberry Pi loses communication, autonomous operation must stop
or enter a predefined safe state.

------------------------------------------------------------------------

## 29. Handoff Collision Avoidance

Define a shared resource:

``` text
HANDOFF_ZONE
```

Arm 1 owns it during:

``` text
ARM1_HANDOFF
```

Arm 2 owns it during:

``` text
ARM2_PICKUP
```

The task manager enforces mutual exclusion.

Conceptually:

``` text
if handoff_zone == ARM1:
    Arm2 cannot enter

if handoff_zone == ARM2:
    Arm1 cannot enter
```

This should be implemented as a software state/lock, not just an
assumption.

------------------------------------------------------------------------

## 30. Coordinate Frames

Recommended conceptual frames:

``` text
world
 └── table
      ├── overhead_camera
      ├── handoff_qr
      ├── arm1_base
      └── arm2_base
```

The overhead camera produces coordinates in its own image frame.

The calibration system converts these into table/world coordinates.

The IK layer converts the target into the appropriate arm's base frame.

This is necessary because the two arm bases will occupy different
physical positions.

------------------------------------------------------------------------

## 31. Cube Pickup Height

The cubes are 25 mm tall.

Do not simply assume that the robot's Z coordinate is 25 mm.

Instead define:

``` text
table_z
cube_height = 25 mm
pickup_z = calibrated value
```

The pickup Z must be calibrated against each arm's coordinate system.

------------------------------------------------------------------------

## 32. Handoff Height

The QR code defines the handoff X/Y reference.

A separate calibrated Z is required:

``` text
handoff_x
handoff_y
handoff_z
```

The gripper orientation must also be calibrated.

------------------------------------------------------------------------

## 33. Destination System

Destinations can be fixed physical zones.

Example:

``` text
RED_ZONE
BLUE_ZONE
YELLOW_ZONE
```

Each has a calibrated position for Arm 2.

This makes the event repeatable and reduces planning complexity.

------------------------------------------------------------------------

## 34. Event Demonstration Sequence

A polished demonstration can be:

1.  Both robots start in READY.
2.  Dashboard displays both digital robots.
3.  Overhead camera shows the colored cubes.
4.  Operator selects AUTO.
5.  System detects a cube.
6.  Detection is averaged.
7.  Target appears on dashboard.
8.  Arm 1 approaches the cube.
9.  Gripper camera shows the robot's view.
10. Arm 1 picks the cube.
11. Arm 1 moves to the QR station.
12. Arm 1 places the cube.
13. Dashboard shows `HANDOFF READY`.
14. Arm 2 moves to the QR station.
15. Arm 2 picks the cube.
16. Arm 2 moves to the color destination.
17. Arm 2 places the cube.
18. Dashboard shows `TASK COMPLETE`.
19. Both arms return to READY.

------------------------------------------------------------------------

## 35. Repository Evolution

The existing repository should be extended instead of replaced.

Suggested structure:

``` text
unoq-braccio/
│
├── firmware/
│   ├── unoq_braccio_firmware/
│   └── uno_braccio_firmware/
│
├── app_lab/
│
├── web_app/
│
├── ros2_ws/
│   └── src/
│       ├── unoq_braccio_driver/
│       ├── unoq_braccio_bringup/
│       ├── unoq_braccio_sim/
│       ├── braccio_vision/
│       ├── braccio_calibration/
│       ├── braccio_task_manager/
│       ├── braccio_handoff/
│       ├── braccio_teleop/
│       └── braccio_web/
│
├── edge_impulse/
│
├── config/
│   ├── camera.yaml
│   ├── calibration.yaml
│   ├── arm1.yaml
│   ├── arm2.yaml
│   ├── handoff.yaml
│   └── destinations.yaml
│
├── models/
│   ├── arm1/
│   └── arm2/
│
├── scripts/
│
└── docs/
    ├── architecture.md
    ├── calibration.md
    ├── event_setup.md
    ├── wiring.md
    └── troubleshooting.md
```

------------------------------------------------------------------------

## 36. Configuration

Calibration values should not be hard-coded into Python.

Example:

``` yaml
camera:
  height_mm: 850
  image_width: 640
  image_height: 480

workspace:
  x_min: 0
  x_max: 400
  y_min: 0
  y_max: 300

cube:
  size_mm: 25
  detection_confidence: 0.70
  averaging_time_ms: 1500
```

Arm-specific parameters should live in separate files.

------------------------------------------------------------------------

## 37. Logging

Log every task.

Useful fields:

``` text
timestamp
task_id
cube_color
detected_x
detected_y
confidence
sample_count
arm1_target
arm1_joint_command
handoff_start
handoff_complete
arm2_target
arm2_joint_command
task_result
error_code
```

This provides useful post-event debugging data.

------------------------------------------------------------------------

## 38. Failure Handling

### Cube not detected

Return to `SEARCHING`.

### Detection unstable

Collect another detection window.

### Cube unreachable

Report:

``` text
TARGET_UNREACHABLE
```

and do not move.

### Pick fails

Return to a safe pose and request another detection.

### Handoff fails

Prevent either robot from entering the handoff zone until the state is
resolved.

### Communication failure

Stop autonomous execution.

### Camera failure

Disable autonomous picking and notify the operator.

------------------------------------------------------------------------

## 39. Development Plan

Build incrementally.

### Phase 1 --- Arm 1

Verify:

``` text
Pi 5 → ROS 2 → UNO Q → Braccio 1
```

### Phase 2 --- Arm 2

Verify:

``` text
Pi 5 → ROS 2 → Arduino UNO → Braccio 2
```

### Phase 3 --- Fixed Camera

Implement:

``` text
camera → cube detection → image coordinates
```

without moving the robot.

### Phase 4 --- Calibration

Implement:

``` text
image coordinates → table coordinates
```

and validate against physical measurements.

### Phase 5 --- Single-Arm Pick

Implement:

``` text
detect → transform → IK → pick
```

### Phase 6 --- QR Handoff

Implement:

``` text
Arm 1 → QR → release
```

### Phase 7 --- Arm 2

Implement:

``` text
QR → Arm 2 → destination
```

### Phase 8 --- State Machine

Combine the full workflow.

### Phase 9 --- Dashboard

Add:

-   Camera feeds.
-   Robot states.
-   Target coordinates.
-   Handoff status.
-   Digital robot.
-   Logs.
-   Controls.

### Phase 10 --- Gazebo

Synchronize the digital system with real joint states and test workflows
before the event.

------------------------------------------------------------------------

## 40. Final Architecture

The final system is a small multi-robot ROS 2 manipulation platform:

``` text
                  PERCEPTION
                      │
                      ▼
             Fixed Grove Camera
                      │
                      ▼
               Cube Detection
                      │
                      ▼
             Coordinate Transform
                      │
                      ▼
                 Task Manager
                  /                          /                           ▼             ▼
             ARM 1          ARM 2
             Pick           Wait
                │             │
                ▼             │
             Handoff ─────────┘
                │
                ▼
             QR Station
                │
                ▼
             ARM 2 Pick
                │
                ▼
          Color Destination
```

The gripper-mounted camera operates alongside this architecture as a
visual demonstration channel, while the fixed overhead camera remains
the authoritative perception source.

The Raspberry Pi 5 is the central computational platform. The Arduino
UNO Q and Arduino UNO are low-level arm controllers. ROS 2 provides the
common software interface. The QR station provides a deterministic
physical handoff point. The web dashboard provides the operator/audience
interface. Gazebo/URDF provides the digital representation.

------------------------------------------------------------------------

## 41. Core Design Principle

The architecture should maintain a strict separation:

**Perception ≠ Calibration ≠ Planning ≠ Coordination ≠ Servo Control ≠
Visualization**

Each layer has a clear responsibility:

-   **Fixed Grove camera:** see the workspace and cubes.
-   **Gripper Grove camera:** show what Arm 1 sees.
-   **ROS 2:** transport structured information between components.
-   **Calibration:** convert camera coordinates into robot coordinates.
-   **IK:** convert target positions into joint targets.
-   **Task manager:** decide which robot acts and when.
-   **Handoff manager:** control access to the shared QR station.
-   **Arduino controllers:** execute low-level joint commands.
-   **Web dashboard:** present the system and provide operator controls.
-   **Gazebo/URDF:** represent the robots digitally.
-   **QR station:** provide a deterministic physical handoff location.

This separation makes the event system easier to test, debug,
demonstrate, and extend.
