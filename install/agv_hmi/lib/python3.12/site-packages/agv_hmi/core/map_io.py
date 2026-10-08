"""map_io.py - save / load the HMI's own map files.

A saved map is the standard Nav2 pair (<name>.pgm + <name>.yaml) so map_server
can load it, plus an optional <name>.walls.json with the vector wall segments
that the HMI draws on top.
"""
from __future__ import annotations

import json
import os

import numpy as np
import yaml


def save_map(path_no_ext: str, grid: np.ndarray, resolution: float,
             origin_x: float, origin_y: float, segments=None,
             wall_thickness_m: float = 0.10) -> str:
    """grid: (h, w), row 0 = lowest y, values -1/0/100. Returns the yaml path."""
    h, w = grid.shape
    pgm = np.full((h, w), 205, dtype=np.uint8)
    pgm[grid == 0] = 254
    pgm[grid >= 65] = 0
    pgm = np.flipud(pgm)
    os.makedirs(os.path.dirname(os.path.abspath(path_no_ext)), exist_ok=True)
    pgm_file = path_no_ext + ".pgm"
    with open(pgm_file, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(pgm.tobytes())
    with open(path_no_ext + ".yaml", "w") as f:
        yaml.dump({
            "image": os.path.basename(pgm_file),
            "resolution": float(resolution),
            "origin": [float(origin_x), float(origin_y), 0.0],
            "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
        }, f)
    walls = path_no_ext + ".walls.json"
    if segments:
        with open(walls, "w") as f:
            json.dump({"wall_thickness_m": wall_thickness_m,
                       "segments": [list(map(float, sg)) for sg in segments]}, f)
    elif os.path.exists(walls):
        os.remove(walls)
    return path_no_ext + ".yaml"


def load_walls(yaml_path: str):
    """(segments, wall_thickness_m) saved next to a map, or ([], 0.10)."""
    walls = os.path.splitext(yaml_path)[0] + ".walls.json"
    try:
        with open(walls) as f:
            d = json.load(f)
        return [tuple(s) for s in d.get("segments", [])], float(d.get("wall_thickness_m", 0.10))
    except Exception:
        return [], 0.10
