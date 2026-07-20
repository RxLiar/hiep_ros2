import os
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class MapSaver:

    def __init__(self, node: Node):
        self._node = node
        self._last_map: OccupancyGrid | None = None
        node.create_subscription(OccupancyGrid, "/map", self._cb, 10)

    def _cb(self, msg):
        self._last_map = msg

    def save(self, filepath: str) -> bool:
        if self._last_map is None:
            self._node.get_logger().error("MapSaver: chưa có map")
            return False
        msg = self._last_map
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        pgm  = np.full((h, w), 205, dtype=np.uint8)
        pgm[data == 0]   = 254
        pgm[data == 100] = 0
        pgm = np.flipud(pgm)
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        pgm_file = filepath + ".pgm"
        with open(pgm_file, "wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode())
            f.write(pgm.tobytes())
        with open(filepath + ".yaml", "w") as f:
            yaml.dump({
                "image": os.path.basename(pgm_file),
                "resolution": float(msg.info.resolution),
                "origin": [float(msg.info.origin.position.x),
                           float(msg.info.origin.position.y), 0.0],
                "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
            }, f)
        self._node.get_logger().info(f"Map saved: {filepath}.yaml")
        return True