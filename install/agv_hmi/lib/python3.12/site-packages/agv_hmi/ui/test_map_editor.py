import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from agv_hmi.core.map_editor import MapEditor, snap_angle, WALL, FREE, UNKNOWN


def make(h=200, w=300):
    g = np.zeros((h, w), dtype=np.int16)
    return g, MapEditor(g, 0.05)


def test_brush_stroke_is_one_undo_step():
    g, ed = make()
    ed.begin()
    for u in np.linspace(10, 120, 40):
        ed.line(u, 50, u + 3, 50, 4, WALL)            # a freehand stroke made of many small segments
    assert ed.end() and ed.can_undo and (g == WALL).sum() > 300
    assert ed.undo() and (g == WALL).sum() == 0, "ONE undo removes the whole stroke"
    assert ed.can_redo and ed.redo() and (g == WALL).sum() > 300
    assert not ed.redo()


def test_erase_and_unknown():
    g, ed = make(); g[:] = WALL
    ed.begin(); ed.line(20, 20, 60, 20, 6, FREE); assert ed.end()
    assert g[20, 40] == FREE and g[20, 100] == WALL
    ed.begin(); ed.rect(100, 100, 120, 110, UNKNOWN); assert ed.end()
    assert (g[100:110, 100:120] == UNKNOWN).all() and g[99, 100] == WALL


def test_width_is_respected():
    g, ed = make()
    ed.begin(); ed.line(20, 100, 120, 100, 6, WALL); ed.end()      # 6 cells thick = 30 cm at 5 cm/cell
    col = g[:, 70]
    assert (col == WALL).sum() == 6, (col == WALL).sum()


def test_edges_clip_and_noop():
    g, ed = make(50, 50)
    ed.begin(); ed.line(-30, -30, -10, -10, 4, WALL); assert not ed.end(), "outside the map = no change, no undo step"
    ed.begin(); ed.line(40, 40, 80, 80, 4, WALL); assert ed.end() and g[49, 49] == WALL


def test_thicken_and_thin():
    g, ed = make()
    g[100, 20:200] = WALL                                           # 1-cell wall
    assert ed.thicken(1) > 0 and (g[:, 100] == WALL).sum() == 3
    assert ed.thin(1) > 0 and (g[:, 100] == WALL).sum() == 1
    assert ed.undo() and (g[:, 100] == WALL).sum() == 3
    assert ed.undo() and (g[:, 100] == WALL).sum() == 1


def test_thin_keeps_walls_at_the_map_border():
    g, ed = make(40, 40); g[:, 0:3] = WALL                          # 3-cell thick wall on the map edge
    ed.thin(1)
    assert (g[:, 0] == WALL).all() and (g[:, 1] == WALL).all(), "the map edge must not eat the wall"
    assert (g[:, 2] == FREE).all(), "only the side facing free space gets thinner"


def test_clear_small_blobs():
    g, ed = make()
    g[10, 10] = WALL; g[30, 30:32] = WALL                           # specks of 1 and 2 cells
    g[100:104, 50:150] = WALL                                       # a real wall
    assert ed.clear_small_blobs(4) == 3
    assert g[10, 10] == FREE and (g[100:104, 50:150] == WALL).all()
    assert ed.undo() and g[10, 10] == WALL


def test_undo_limit_and_redo_cleared_by_new_edit():
    g, ed = make(); ed.max_undo = 3
    for i in range(6):
        ed.begin(); ed.line(10 + i * 20, 10, 12 + i * 20, 10, 2, WALL); ed.end()
    n = 0
    while ed.undo():
        n += 1
    assert n == 3
    ed.begin(); ed.line(5, 5, 8, 5, 2, WALL); ed.end()
    assert not ed.can_redo


def test_large_map_undo_uses_tiles_only():
    g = np.zeros((4000, 4000), dtype=np.int16); ed = MapEditor(g, 0.05)
    ed.begin(); ed.line(100, 100, 400, 100, 4, WALL); ed.end()
    stored = sum(b.nbytes for b, _ in ed._undo[0].values())
    assert stored < 2_000_000, stored                               # not a 32 MB full copy


def test_snap_angle():
    u, v = snap_angle(0, 0, 10, 1.5, 45.0); assert abs(v) < 1e-9 and abs(u - math.hypot(10, 1.5)) < 1e-9
    u, v = snap_angle(0, 0, 5, 6, 45.0); assert abs(u - v) < 1e-6


import math
if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f(); print("ok", n)
