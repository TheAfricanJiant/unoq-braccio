"""Vision-driven pick and place for the simulated (or real) Braccio.

For each colour: request a detection from the overhead camera, wait for the
averaged cube position, solve IK, pick the cube and drop it in the bin for
that colour. Commands go out on /braccio/joint_command, so the same node
drives Gazebo or the hardware bridge.

    ros2 run unoq_braccio_driver pick_place_demo
    ros2 run unoq_braccio_driver pick_place_demo --ros-args -p colors:="[blue]"
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from unoq_braccio_driver.braccio_kinematics import GRIPPER_CLOSED, GRIPPER_OPEN, solve_ik
from unoq_braccio_driver.braccio_model import JOINT_NAMES, POSES

# Drop bin centres (world xy, metres); must match worlds/workspace.world.
BINS = {
    "red": (0.169, 0.141),
    "blue": (0.093, 0.199),
    "yellow": (0.0, 0.22),
}
CUBE_CENTRE_Z = 0.025   # fingertip height when grasping a 50 mm cube
HOVER_Z = 0.14          # travel height above the table
DROP_Z = 0.07           # release height above a bin


class PickPlaceDemo(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_demo")
        self.declare_parameter("colors", ["red", "blue", "yellow"])
        self.declare_parameter("step_wait", 2.0)     # >= joint_trajectory_bridge move_time
        self.declare_parameter("gripper_wait", 1.0)
        self.declare_parameter("detect_timeout", 6.0)

        self.command = self.create_publisher(JointState, "/braccio/joint_command", 10)
        self.request = self.create_publisher(String, "/vision/detect_request", 10)
        self.create_subscription(String, "/vision/cube_target", self.on_target, 10)
        self.target = None
        self.target_event = threading.Event()

    def on_target(self, msg: String) -> None:
        try:
            self.target = json.loads(msg.data).get("cubes", {})
        except json.JSONDecodeError:
            self.target = {}
        self.target_event.set()

    # -- helpers -------------------------------------------------------

    def move(self, pose, wait=None) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(v) for v in pose]
        self.command.publish(msg)
        time.sleep(float(self.get_parameter("step_wait").value) if wait is None else wait)

    def ik(self, x, y, z, gripper):
        pose = solve_ik(x, y, z, gripper)
        if pose is None:
            raise RuntimeError(f"Unreachable target ({x:.3f}, {y:.3f}, {z:.3f})")
        return pose

    def detect(self, color: str):
        timeout = float(self.get_parameter("detect_timeout").value)
        self.target_event.clear()
        self.request.publish(String(data=color))
        if not self.target_event.wait(timeout):
            return None
        return (self.target or {}).get(color)

    # -- sequence ------------------------------------------------------

    def pick_and_place(self, color: str) -> bool:
        grip_wait = float(self.get_parameter("gripper_wait").value)
        found = self.detect(color)
        if found is None:
            self.get_logger().warning(f"No {color} cube detected; skipping")
            return False
        x, y = found["x"], found["y"]
        self.get_logger().info(
            f"{color}: cube at ({x:.3f}, {y:.3f}) confidence {found['confidence']:.2f}"
        )
        bin_x, bin_y = BINS[color]

        above = self.ik(x, y, HOVER_Z, GRIPPER_OPEN)
        down = self.ik(x, y, CUBE_CENTRE_Z, GRIPPER_OPEN)
        grip = self.ik(x, y, CUBE_CENTRE_Z, GRIPPER_CLOSED)
        lift = self.ik(x, y, HOVER_Z, GRIPPER_CLOSED)
        over_bin = self.ik(bin_x, bin_y, HOVER_Z, GRIPPER_CLOSED)
        into_bin = self.ik(bin_x, bin_y, DROP_Z, GRIPPER_CLOSED)
        release = self.ik(bin_x, bin_y, DROP_Z, GRIPPER_OPEN)
        retreat = self.ik(bin_x, bin_y, HOVER_Z, GRIPPER_OPEN)

        self.move(above)
        self.move(down)
        self.move(grip, wait=grip_wait)
        self.move(lift)
        self.move(over_bin)
        self.move(into_bin)
        self.move(release, wait=grip_wait)
        self.move(retreat)
        return True

    def run(self) -> None:
        # Let the publishers and detector discover each other.
        time.sleep(1.0)
        ready = list(POSES["ready"])
        ready[5] = GRIPPER_OPEN
        self.move(ready)

        done = []
        for color in [str(c).lower() for c in self.get_parameter("colors").value]:
            if color not in BINS:
                self.get_logger().error(f"No bin defined for '{color}'")
                continue
            try:
                if self.pick_and_place(color):
                    done.append(color)
            except RuntimeError as exc:
                self.get_logger().error(str(exc))
        self.move(ready)
        self.get_logger().info(f"Finished. Placed: {', '.join(done) or 'nothing'}")


def main() -> None:
    rclpy.init()
    node = PickPlaceDemo()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
