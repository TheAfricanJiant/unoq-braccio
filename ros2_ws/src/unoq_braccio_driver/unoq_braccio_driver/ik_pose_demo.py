import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from unoq_braccio_driver.braccio_kinematics import solve_ik
from unoq_braccio_driver.braccio_model import JOINT_NAMES


def solve_reachable(x: float, y: float, z: float, gripper: int):
    """IK for the fingertip; pulls the target in towards the base until reachable."""
    angle = math.atan2(y, x)
    radius = math.hypot(x, y)
    while radius > 0.05:
        pose = solve_ik(radius * math.cos(angle), radius * math.sin(angle), z, gripper)
        if pose is not None:
            return pose, radius
        radius -= 0.005
    return None, radius


class IkPoseDemo(Node):
    def __init__(self) -> None:
        super().__init__("unoq_braccio_ik_pose_demo")
        self.declare_parameter("x", 0.30)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.06)
        self.declare_parameter("gripper", 25)
        self.publisher = self.create_publisher(JointState, "/braccio/joint_command", 10)
        self.timer = self.create_timer(0.5, self.publish_once)
        self.sent = False

    def publish_once(self) -> None:
        if self.sent:
            rclpy.shutdown()
            return
        self.sent = True

        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        z = float(self.get_parameter("z").value)
        pose, radius = solve_reachable(x, y, z, int(self.get_parameter("gripper").value))
        if pose is None:
            self.get_logger().error(f"({x:.3f}, {y:.3f}, {z:.3f}) is out of reach; not moving")
            return
        if abs(radius - math.hypot(x, y)) > 1e-6:
            self.get_logger().warning(f"Target out of reach; pulled in to radius {radius:.3f} m")

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(value) for value in pose]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published IK pose: {pose}")


def main() -> None:
    rclpy.init()
    node = IkPoseDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
