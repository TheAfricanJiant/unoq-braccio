"""Overhead-camera cube detector for the Gazebo scene.

Detection is request-driven, mirroring the real Grove Vision AI design: the
node stays idle until it receives a request, then averages a short burst of
frames and publishes one result. Nothing is published between requests.

    /vision/detect_request   std_msgs/String   optional colour filter ("" = all)
    /vision/cube_target      std_msgs/String   JSON, see below

Result JSON::

    {"cubes": {"red": {"x": 0.21, "y": -0.10, "confidence": 0.98, "samples": 8}}}

x/y are table (world) coordinates in metres.
"""

import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

# HSV ranges (OpenCV: H 0-179). Colours match the blocks in workspace.world.
COLOR_RANGES = {
    "red": [((0, 120, 70), (8, 255, 255)), ((172, 120, 70), (179, 255, 255))],
    "blue": [((100, 120, 70), (130, 255, 255))],
    "yellow": [((20, 120, 70), (35, 255, 255))],
}

_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}


def pixel_to_table(u, v, fx, fy, cx, cy, cam_x, cam_y, height):
    """Pixel -> table xy for a camera looking straight down.

    The camera is pitched +90 deg, so image right = world -y and image
    down = world -x. ``height`` is the camera height above the plane the
    pixel lies on.
    """
    return (
        cam_x - (v - cy) / fy * height,
        cam_y - (u - cx) / fx * height,
    )


def image_to_rgb(msg: Image):
    channels = _CHANNELS.get(msg.encoding)
    if channels is None:
        return None
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    frame = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.encoding.startswith("bgr"):
        frame = frame[:, :, [2, 1, 0]]
    return frame[:, :, :3]


class SimCubeDetector(Node):
    def __init__(self) -> None:
        super().__init__("sim_cube_detector")
        # Must match the overhead_camera model in worlds/workspace.world.
        self.declare_parameter("camera_x", 0.20)
        self.declare_parameter("camera_y", -0.02)
        self.declare_parameter("camera_z", 0.80)
        self.declare_parameter("cube_height", 0.05)
        # Only blocks inside this table area count (the coloured bins are outside).
        self.declare_parameter("roi", [0.12, 0.34, -0.18, 0.05])  # xmin xmax ymin ymax
        self.declare_parameter("min_area_px", 150)
        self.declare_parameter("window_s", 1.5)
        self.declare_parameter("min_samples", 5)
        self.declare_parameter("outlier_mm", 20.0)

        self.info = None
        self.window_end = 0.0
        self.color_filter = ""
        self.samples = {}

        self.create_subscription(CameraInfo, "/vision/overhead/camera_info", self.on_info, 10)
        self.create_subscription(Image, "/vision/overhead/image_raw", self.on_image, 5)
        self.create_subscription(String, "/vision/detect_request", self.on_request, 10)
        self.publisher = self.create_publisher(String, "/vision/cube_target", 10)
        self.create_timer(0.1, self.check_window)

    def on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def on_request(self, msg: String) -> None:
        self.color_filter = msg.data.strip().lower()
        self.samples = {}
        self.window_end = time.monotonic() + float(self.get_parameter("window_s").value)
        self.get_logger().info(f"Detection requested ({self.color_filter or 'all colours'})")

    def on_image(self, msg: Image) -> None:
        if time.monotonic() >= self.window_end or self.info is None:
            return  # idle between requests
        import cv2

        frame = image_to_rgb(msg)
        if frame is None:
            self.get_logger().warning(f"Unsupported image encoding {msg.encoding}")
            return
        hsv = cv2.cvtColor(np.ascontiguousarray(frame), cv2.COLOR_RGB2HSV)
        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        cam_x = float(self.get_parameter("camera_x").value)
        cam_y = float(self.get_parameter("camera_y").value)
        height = float(self.get_parameter("camera_z").value) - float(
            self.get_parameter("cube_height").value
        )
        xmin, xmax, ymin, ymax = (float(v) for v in self.get_parameter("roi").value)
        min_area = float(self.get_parameter("min_area_px").value)

        for color, ranges in COLOR_RANGES.items():
            if self.color_filter and color != self.color_filter:
                continue
            mask = None
            for low, high in ranges:
                part = cv2.inRange(hsv, np.array(low), np.array(high))
                mask = part if mask is None else cv2.bitwise_or(mask, part)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in sorted(contours, key=cv2.contourArea, reverse=True):
                if cv2.contourArea(contour) < min_area:
                    break
                m = cv2.moments(contour)
                u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
                x, y = pixel_to_table(u, v, fx, fy, cx, cy, cam_x, cam_y, height)
                if xmin <= x <= xmax and ymin <= y <= ymax:
                    self.samples.setdefault(color, []).append((x, y))
                    break

    def check_window(self) -> None:
        if self.window_end == 0.0 or time.monotonic() < self.window_end:
            return
        self.window_end = 0.0
        min_samples = int(self.get_parameter("min_samples").value)
        limit = float(self.get_parameter("outlier_mm").value) / 1000.0

        cubes = {}
        for color, points in self.samples.items():
            if len(points) < min_samples:
                continue
            mx = sorted(p[0] for p in points)[len(points) // 2]
            my = sorted(p[1] for p in points)[len(points) // 2]
            kept = [p for p in points if math.hypot(p[0] - mx, p[1] - my) <= limit]
            if len(kept) < min_samples:
                continue  # too scattered to trust
            cubes[color] = {
                "x": sum(p[0] for p in kept) / len(kept),
                "y": sum(p[1] for p in kept) / len(kept),
                "confidence": len(kept) / len(points),
                "samples": len(kept),
            }
        self.publisher.publish(String(data=json.dumps({"cubes": cubes})))
        self.get_logger().info(f"Published {sorted(cubes)}")


def main() -> None:
    rclpy.init()
    node = SimCubeDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
