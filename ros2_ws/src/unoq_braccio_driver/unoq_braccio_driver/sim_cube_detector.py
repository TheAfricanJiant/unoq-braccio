"""Overhead-camera detector: the authoritative source of cube and bin locations.

Request-driven, like the real Grove Vision AI setup: the node idles until it
receives a request, averages a short burst of frames and publishes one result.

    /vision/detect_request   std_msgs/String   "" (everything) or a cube colour
    /vision/cube_target      std_msgs/String   JSON, below

Result::

    {"cubes": [{"color": "red", "x": 0.20, "y": -0.14, "sector": "pick",
                "confidence": 1.0, "samples": 22}, ...],
     "bins":  {"green": {"x": 0.169, "y": 0.141, "cube_color": "red"}, ...}}

x/y are table coordinates in metres. ``sector`` is ``pick``, a bin name, or
``none``. Bins are found by their own colours (not cube colours).
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from unoq_braccio_driver import braccio_workspace as ws
from unoq_braccio_driver.color_vision import find_blobs, image_to_rgb, to_hsv

# Re-exported for older imports / tests.
pixel_to_table = ws.pixel_to_table


def cluster_samples(samples, radius):
    """Group (x, y) samples into clusters no wider than ``radius`` from their seed.

    Returns a list of lists of points.
    """
    clusters = []
    for point in samples:
        for cluster in clusters:
            if math.hypot(point[0] - cluster[0][0], point[1] - cluster[0][1]) <= radius:
                cluster.append(point)
                break
        else:
            clusters.append([point])
    return clusters


class SimCubeDetector(Node):
    def __init__(self) -> None:
        super().__init__("sim_cube_detector")
        cam_x, cam_y, cam_z = ws.CAMERA_XYZ
        self.declare_parameter("camera_x", cam_x)
        self.declare_parameter("camera_y", cam_y)
        self.declare_parameter("camera_z", cam_z)
        self.declare_parameter("window_s", 1.5)
        self.declare_parameter("min_samples", 5)
        self.declare_parameter("cluster_mm", 12.0)

        self.info = None
        self.window_end = 0.0
        self.color_filter = ""
        self.cube_samples = {}   # colour -> [(x, y)]
        self.bin_samples = {}    # bin name -> [(x, y)]

        self.create_subscription(CameraInfo, "/vision/overhead/camera_info", self.on_info, 10)
        self.create_subscription(Image, "/vision/overhead/image_raw", self.on_image, 5)
        self.create_subscription(String, "/vision/detect_request", self.on_request, 10)
        self.publisher = self.create_publisher(String, "/vision/cube_target", 10)
        self.create_timer(0.1, self.check_window)

    def on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def on_request(self, msg: String) -> None:
        self.color_filter = msg.data.strip().lower()
        self.cube_samples = {}
        self.bin_samples = {}
        self.window_end = time.monotonic() + float(self.get_parameter("window_s").value)
        self.get_logger().info(f"Detection requested ({self.color_filter or 'all'})")

    def on_image(self, msg: Image) -> None:
        if time.monotonic() >= self.window_end or self.info is None:
            return  # idle between requests
        rgb = image_to_rgb(msg)
        if rgb is None:
            self.get_logger().warning(f"Unsupported image encoding {msg.encoding}")
            return
        hsv = to_hsv(rgb)

        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        cam_x = float(self.get_parameter("camera_x").value)
        cam_y = float(self.get_parameter("camera_y").value)
        cam_z = float(self.get_parameter("camera_z").value)

        # Pixel area of a cube / bin top, from the actual intrinsics.
        cube_px = ws.CUBE_SIZE * fx / (cam_z - ws.CUBE_CENTRE_Z)
        cube_lo, cube_hi = 0.4 * cube_px ** 2, 2.5 * cube_px ** 2

        for color, ranges in ws.CUBE_HSV.items():
            if self.color_filter and color != self.color_filter:
                continue
            for u, v, _ in find_blobs(hsv, ranges, cube_lo, cube_hi):
                x, y = ws.pixel_to_table(
                    u, v, fx, fy, cx, cy, cam_x, cam_y, cam_z - ws.CUBE_CENTRE_Z
                )
                self.cube_samples.setdefault(color, []).append((x, y))

        for bin_ in ws.BINS:
            bin_px = bin_.size * fx / (cam_z - bin_.height)
            for u, v, _ in find_blobs(
                hsv, ws.BIN_HSV[bin_.name], 0.5 * bin_px ** 2, 1.6 * bin_px ** 2
            ):
                x, y = ws.pixel_to_table(u, v, fx, fy, cx, cy, cam_x, cam_y, cam_z - bin_.height)
                self.bin_samples.setdefault(bin_.name, []).append((x, y))

    def check_window(self) -> None:
        if self.window_end == 0.0 or time.monotonic() < self.window_end:
            return
        self.window_end = 0.0
        min_samples = int(self.get_parameter("min_samples").value)
        radius = float(self.get_parameter("cluster_mm").value) / 1000.0

        cubes = []
        for color, points in self.cube_samples.items():
            for cluster in cluster_samples(points, radius):
                if len(cluster) < min_samples:
                    continue  # flicker, not a cube
                x = sum(p[0] for p in cluster) / len(cluster)
                y = sum(p[1] for p in cluster) / len(cluster)
                cubes.append({
                    "color": color,
                    "x": x,
                    "y": y,
                    "sector": ws.sector_of(x, y),
                    "confidence": len(cluster) / max(1, len(points)),
                    "samples": len(cluster),
                })

        bins = {}
        for name, points in self.bin_samples.items():
            biggest = max(cluster_samples(points, radius), key=len)
            if len(biggest) >= min_samples:
                bins[name] = {
                    "x": sum(p[0] for p in biggest) / len(biggest),
                    "y": sum(p[1] for p in biggest) / len(biggest),
                    "cube_color": ws.BIN_BY_NAME[name].cube_color,
                }

        self.publisher.publish(String(data=json.dumps({"cubes": cubes, "bins": bins})))
        self.get_logger().info(
            f"Published {len(cubes)} cube(s), {len(bins)} bin(s)"
        )


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
