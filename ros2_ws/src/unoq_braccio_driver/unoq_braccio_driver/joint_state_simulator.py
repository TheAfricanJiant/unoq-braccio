import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from unoq_braccio_driver.braccio_kinematics import (
    URDF_JOINT_NAMES,
    servo_positions_to_urdf,
)
from unoq_braccio_driver.braccio_model import JOINT_NAMES, POSES


class JointStateSimulator(Node):
    """Debug fallback: turns commands straight into /joint_states (no physics)."""

    def __init__(self) -> None:
        super().__init__("unoq_braccio_joint_state_simulator")
        self.last_servo = {name: float(POSES["ready"][i]) for i, name in enumerate(JOINT_NAMES)}
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.subscription = self.create_subscription(
            JointState,
            "/braccio/joint_command",
            self.on_command,
            10,
        )

    def on_command(self, msg: JointState) -> None:
        self.last_servo.update(dict(zip(msg.name, (float(v) for v in msg.position))))
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = URDF_JOINT_NAMES
        state.position = servo_positions_to_urdf(self.last_servo)
        self.publisher.publish(state)


def main() -> None:
    rclpy.init()
    node = JointStateSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
