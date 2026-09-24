import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from unoq_braccio_driver.braccio_kinematics import (
    URDF_JOINT_NAMES,
    servo_positions_to_urdf,
)
from unoq_braccio_driver.braccio_model import JOINT_NAMES, POSES


class JointTrajectoryBridge(Node):
    """/braccio/joint_command (servo degrees) -> /arm_controller/joint_trajectory."""

    def __init__(self) -> None:
        super().__init__("unoq_braccio_joint_trajectory_bridge")
        self.declare_parameter("move_time", 1.5)
        self.last_servo = {name: float(POSES["ready"][i]) for i, name in enumerate(JOINT_NAMES)}
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )
        self.subscription = self.create_subscription(
            JointState,
            "/braccio/joint_command",
            self.on_command,
            10,
        )

    def on_command(self, msg: JointState) -> None:
        # Joints missing from the message keep their last commanded value.
        self.last_servo.update(dict(zip(msg.name, (float(v) for v in msg.position))))

        trajectory = JointTrajectory()
        # A zero stamp means "start now" for the controller, whatever clock it uses.
        trajectory.joint_names = URDF_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = servo_positions_to_urdf(self.last_servo)
        move_time = float(self.get_parameter("move_time").value)
        point.time_from_start = Duration(
            sec=int(move_time), nanosec=int((move_time % 1.0) * 1e9)
        )
        trajectory.points = [point]
        self.publisher.publish(trajectory)


def main() -> None:
    rclpy.init()
    node = JointTrajectoryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
