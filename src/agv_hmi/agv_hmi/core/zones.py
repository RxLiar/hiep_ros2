"""zones.py - keep-out / slow zones drawn on a map (world coordinates)."""
from __future__ import annotations

import json
import os

import numpy as np

KINDS = ("keepout", "slow")


def zones_path(path_no_ext: str) -> str:
    return path_no_ext + ".zones.json"


def load_zones(path_no_ext: str) -> list[dict]:
    try:
        with open(zones_path(path_no_ext), encoding="utf-8") as f:
            d = json.load(f)
        return [z for z in d if z.get("kind") in KINDS and len(z.get("pts", [])) >= 3]
    except (OSError, ValueError, TypeError):
        return []


def save_zones(path_no_ext: str, zones: list[dict]) -> None:
    if not zones:
        try:
            os.remove(zones_path(path_no_ext))
        except OSError:
            pass
        return
    with open(zones_path(path_no_ext), "w", encoding="utf-8") as f:
        json.dump(zones, f)


def _fill_polygon(mask: np.ndarray, poly_cells: np.ndarray) -> None:
    """Even-odd fill of a polygon given in (col,row) cell coordinates."""
    h, w = mask.shape
    x0, x1 = max(0, int(np.floor(poly_cells[:, 0].min()))), min(w - 1, int(np.ceil(poly_cells[:, 0].max())))
    y0, y1 = max(0, int(np.floor(poly_cells[:, 1].min()))), min(h - 1, int(np.ceil(poly_cells[:, 1].max())))
    if x1 < x0 or y1 < y0:
        return
    xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
    inside = np.zeros(xs.shape, bool)
    px, py = poly_cells[:, 0], poly_cells[:, 1]
    j = len(px) - 1
    for i in range(len(px)):
        cond = ((py[i] > ys) != (py[j] > ys)) & (xs < (px[j] - px[i]) * (ys - py[i]) / (py[j] - py[i] + 1e-12) + px[i])
        inside ^= cond
        j = i
    mask[y0:y1 + 1, x0:x1 + 1] |= inside


def zones_mask(zones: list[dict], kind: str, shape, res: float, ox: float, oy: float) -> np.ndarray:
    """Boolean mask (row 0 = lowest y, same layout as the grid) of the zones of one kind."""
    mask = np.zeros(shape, bool)
    for z in zones:
        if z["kind"] != kind:
            continue
        pts = np.array([[(x - ox) / res, (y - oy) / res] for x, y in z["pts"]], float)
        _fill_polygon(mask, pts)
    return mask


def export_keepout_mask(path_no_ext: str, zones: list[dict], shape, res: float, ox: float, oy: float) -> str | None:
    """Write <path>_keepout.pgm/.yaml for Nav2's keepout costmap filter (zones = black)."""
    mask = zones_mask(zones, "keepout", shape, res, ox, oy)
    if not mask.any():
        return None
    h, w = shape
    img = np.where(mask, 0, 254).astype(np.uint8)
    img = np.flipud(img)
    base = path_no_ext + "_keepout"
    with open(base + ".pgm", "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode()); f.write(img.tobytes())
    with open(base + ".yaml", "w") as f:
        f.write(f"image: {os.path.basename(base)}.pgm\nmode: trinary\nresolution: {res}\n"
                f"origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return base + ".yaml"
