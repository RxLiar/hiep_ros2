"""Run with: python3 -m pytest test/test_map_cleaner.py   (needs numpy + scipy only)."""
import numpy as np
from agv_hmi.core.map_cleaner import clean_map, CleanParams


def _room(rng):
    g = np.full((240, 320), -1, np.int8)
    g[20:220, 20:300] = 0
    for x in range(20, 300):
        for y in (20, 218):
            yy = int(round(y + rng.normal(0, 0.5))); g[yy:yy + 2, x] = 100
    for y in range(20, 220):
        for x in (20, 298):
            xx = int(round(x + rng.normal(0, 0.5))); g[y, xx:xx + 2] = 100
    g[120 - 6:120 + 6, 230 - 6:230 + 6] = 100                      # pillar
    g[19:23, 80:86] = 0                                            # 0.3 m gap in the top wall
    ys, xs = rng.integers(30, 210, 120), rng.integers(30, 290, 120)
    g[ys, xs] = 100                                                # speckle noise
    return g, ys, xs


def test_clean_room():
    g, ys, xs = _room(np.random.default_rng(0))
    r = clean_map(g, 0.05, CleanParams())
    occ = r.grid >= 65
    assert len(r.segments) == 4                      # four straight walls
    interior = occ.copy(); interior[:30], interior[210:], interior[:, :30], interior[:, 290:] = False, False, False, False
    interior[120 - 8:120 + 8, 230 - 8:230 + 8] = False
    assert interior.sum() == 0                       # noise gone, only the pillar remains inside
    assert occ[120, 230]                             # pillar kept
    assert occ[19:23, 80:86].any()                   # gap bridged
    assert np.where(occ[:40, 40:140].any(axis=1))[0].size <= 3   # wall is straight / uniform
