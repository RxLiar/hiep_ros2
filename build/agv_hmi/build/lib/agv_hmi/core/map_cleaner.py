"""map_cleaner.py - clean an occupancy grid and rebuild walls as straight lines.

Pipeline (pure numpy/scipy, no ROS, no Qt):
  1. threshold occupied cells
  2. remove noise: small isolated blobs
  3. extract straight wall segments with iterative RANSAC (+ PCA refinement)
  4. regularise: snap to dominant (perpendicular) directions, merge collinear
     pieces, bridge small gaps, join segment ends at corners / T-junctions
  5. re-rasterise: clean walls with uniform thickness; objects that are not
     walls (pillars, machines, curved things) are kept as they were

Everything is in grid-cell units internally (x = column, y = row, same layout
as nav_msgs/OccupancyGrid.data); `res` (m/cell) converts the metric params.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import List, Tuple

import numpy as np
from scipy import ndimage as ndi

UNKNOWN, FREE, OCC = -1, 0, 100


@dataclass
class CleanParams:
    occ_threshold: int = 65
    min_blob_m: float = 0.20          # blobs whose largest extent is below this are noise
    min_blob_cells: int = 6           # ... or that have fewer cells than this
    link_gap_m: float = 0.10          # speckle this close to a wall is joined to it before filtering
    ransac_tol_m: float = 0.06        # max distance of a point from its wall line
    min_wall_m: float = 0.60          # shortest segment kept as a wall
    max_gap_m: float = 0.40           # gaps up to this along one wall are bridged
    snap_deg: float = 7.0             # snap angles to the dominant directions within this
    merge_dist_m: float = 0.15        # parallel segments closer than this become one wall
    ghost_dist_m: float = 0.30        # short echo lines this close to a much longer parallel wall are dropped
    join_dist_m: float = 0.35         # corner / T-junction snapping distance
    wall_thickness_m: float = 0.10    # thickness of the redrawn walls
    max_wall_thickness_m: float = 0.30  # thicker clusters (pillars, machines) are objects, not walls
    keep_other_objects: bool = True   # keep non-wall clusters (pillars, machines)
    max_lines: int = 600
    seed: int = 1

    def to_dict(self):
        return asdict(self)


@dataclass
class CleanResult:
    grid: np.ndarray                                  # int8 (h, w) cleaned grid
    segments: List[Tuple[float, float, float, float]] # wall segments, cell coords
    stats: dict = field(default_factory=dict)


# --------------------------------------------------------------------------
def _components(mask: np.ndarray):
    st = np.ones((3, 3), bool)
    return ndi.label(mask, structure=st)


def _line_runs(t: np.ndarray, max_gap: float):
    """Split sorted 1-D positions into runs separated by gaps > max_gap."""
    if t.size == 0:
        return []
    cuts = np.where(np.diff(t) > max_gap)[0]
    starts = np.r_[0, cuts + 1]
    ends = np.r_[cuts, t.size - 1]
    return list(zip(starts, ends))


def _fit_line(pts: np.ndarray):
    c = pts.mean(axis=0)
    u, s, vt = np.linalg.svd(pts - c, full_matrices=False)
    d = vt[0]
    return c, d


def _ransac_segments(pts, tol, min_len, max_gap, max_lines, rng):
    """Iteratively peel straight segments off the point cloud."""
    remaining = pts.copy()
    segs = []
    aside = []
    min_inl = max(8, int(min_len * 0.6))
    for _ in range(max_lines):
        n = len(remaining)
        if n < min_inl:
            break
        iters = min(250, max(40, n // 4))
        i = rng.integers(0, n, size=iters)
        j = rng.integers(0, n, size=iters)
        ok = i != j
        i, j = i[ok], j[ok]
        if len(i) == 0:
            break
        p, q = remaining[i], remaining[j]
        d = q - p
        ln = np.hypot(d[:, 0], d[:, 1])
        good = ln > max(3.0, tol * 2)
        if not good.any():
            break
        p, d, ln = p[good], d[good], ln[good]
        nx, ny = -d[:, 1] / ln, d[:, 0] / ln
        # distance of sampled points to every candidate line, chunked. Large maps
        # are scored on a random sample (the winner is refined on all points).
        if n > 12000:
            sample = remaining[rng.integers(0, n, size=12000)]
        else:
            sample = remaining
        best_k, best_cnt = -1, 0
        for s in range(0, len(p), 64):
            ps, nxs, nys = p[s:s + 64], nx[s:s + 64], ny[s:s + 64]
            dist = np.abs((sample[:, 0][None, :] - ps[:, 0:1]) * nxs[:, None]
                          + (sample[:, 1][None, :] - ps[:, 1:2]) * nys[:, None])
            cnt = (dist <= tol).sum(axis=1)
            k = int(cnt.argmax())
            if cnt[k] > best_cnt:
                best_cnt, best_k = int(cnt[k]), s + k
        if best_cnt * (n / len(sample)) < min_inl:
            break
        c0, d0 = p[best_k], d[best_k] / ln[best_k]
        n0 = np.array([-d0[1], d0[0]])
        inl = np.abs((remaining - c0) @ n0) <= tol
        for _r in range(3):  # refine with least squares
            c0, d0 = _fit_line(remaining[inl])
            n0 = np.array([-d0[1], d0[0]])
            inl = np.abs((remaining - c0) @ n0) <= tol
        idx = np.where(inl)[0]
        t = (remaining[idx] - c0) @ d0
        order = np.argsort(t)
        idx, t = idx[order], t[order]
        used = np.zeros(n, bool)
        found = False
        for a, b in _line_runs(t, max_gap):
            length = t[b] - t[a]
            # a wall has a few cells per unit of length; a filled blob (pillar, machine)
            # fitted by a diagonal has many, so it is not a wall
            if length >= min_len and (b - a + 1) / max(length, 1.0) <= 6.5:
                segs.append((c0 + d0 * t[a], c0 + d0 * t[b], int(b - a + 1)))
                used[idx[a:b + 1]] = True
                found = True
        if not found:
            # only short pieces on this line: set those points aside so the
            # search does not keep finding them again (kept as leftovers)
            aside.append(remaining[idx])
            remaining = np.delete(remaining, idx, axis=0)
            continue
        remaining = remaining[~used]
    left = np.vstack([remaining] + aside) if aside else remaining
    return segs, left


def _angle(s):
    a = np.degrees(np.arctan2(s[1][1] - s[0][1], s[1][0] - s[0][0])) % 180.0
    return a


def _snap_directions(segs, snap_deg):
    """Rotate segments (about their midpoint) onto dominant, perpendicular families."""
    if not segs:
        return segs
    lens = np.array([np.hypot(*(s[1] - s[0])) for s in segs])
    ang = np.array([_angle(s) for s in segs])
    left = list(np.argsort(-lens))
    new_angle = ang.copy()
    while left:
        seed = left[0]
        base = ang[seed] % 90.0
        # weighted circular mean of the family around the seed (mod 90)
        members = []
        for k in left:
            dd = (ang[k] % 90.0) - base
            dd = (dd + 45.0) % 90.0 - 45.0
            if abs(dd) <= snap_deg:
                members.append((k, dd))
        w = np.array([lens[k] for k, _ in members])
        dev = np.array([dd for _, dd in members])
        base = (base + float((w * dev).sum() / w.sum())) % 90.0
        for k, _ in members:
            quad = int(round((ang[k] - base) / 90.0)) % 2
            new_angle[k] = (base + 90.0 * quad) % 180.0
        member_ids = {k for k, _ in members}
        left = [k for k in left if k not in member_ids]
    out = []
    for s, a_new, ln in zip(segs, new_angle, lens):
        mid = (s[0] + s[1]) / 2.0
        d = np.array([np.cos(np.radians(a_new)), np.sin(np.radians(a_new))])
        out.append((mid - d * ln / 2.0, mid + d * ln / 2.0, s[2]))
    return out


def _merge_collinear(segs, merge_dist, max_gap):
    segs = list(segs)
    changed = True
    while changed:
        changed = False
        for i in range(len(segs)):
            for j in range(i + 1, len(segs)):
                a, b = segs[i], segs[j]
                da, db = a[1] - a[0], b[1] - b[0]
                la, lb = np.hypot(*da), np.hypot(*db)
                ua, ub = da / la, db / lb
                if abs(ua[0] * ub[1] - ua[1] * ub[0]) > np.sin(np.radians(3.0)):
                    continue
                n = np.array([-ua[1], ua[0]])
                off_b = np.abs(((b[0] + b[1]) / 2 - a[0]) @ n)
                if off_b > merge_dist:
                    continue
                if ua @ ub < 0:
                    ub = -ub
                ta = sorted([0.0, la])
                tb0, tb1 = sorted([(b[0] - a[0]) @ ua, (b[1] - a[0]) @ ua])
                gap = max(tb0 - ta[1], ta[0] - tb1)
                if gap > max_gap:
                    continue
                # merged: weighted offset, union extent
                wa, wb = la, lb
                off = (wa * 0.0 + wb * ((b[0] + b[1]) / 2 - a[0]) @ n) / (wa + wb)
                lo, hi = min(ta[0], tb0), max(ta[1], tb1)
                p0 = a[0] + ua * lo + n * off
                p1 = a[0] + ua * hi + n * off
                segs[i] = (p0, p1, a[2] + b[2])
                segs.pop(j)
                changed = True
                break
            if changed:
                break
    return segs


def _drop_ghosts(segs, ghost_dist, ratio=0.5):
    """Remove short lines hugging a much longer, parallel, overlapping wall."""
    keep = []
    L = [np.hypot(*(s[1] - s[0])) for s in segs]
    for i, a in enumerate(segs):
        ua = (a[1] - a[0]) / L[i]
        na = np.array([-ua[1], ua[0]])
        ghost = False
        for j, b in enumerate(segs):
            if i == j or L[j] < L[i] / ratio:
                continue
            ub = (b[1] - b[0]) / L[j]
            if abs(ua[0] * ub[1] - ua[1] * ub[0]) > np.sin(np.radians(5.0)):
                continue
            off = abs(((a[0] + a[1]) / 2 - b[0]) @ np.array([-ub[1], ub[0]]))
            if off > ghost_dist:
                continue
            t0, t1 = sorted([(a[0] - b[0]) @ ub, (a[1] - b[0]) @ ub])
            if t1 >= -ghost_dist and t0 <= L[j] + ghost_dist:
                ghost = True
                break
        if not ghost:
            keep.append(a)
    return keep


def _join_corners(segs, join_dist):
    """Move segment ends onto the intersection with a neighbouring wall."""
    segs = [[s[0].copy(), s[1].copy(), s[2]] for s in segs]
    n = len(segs)
    for i in range(n):
        for end in (0, 1):
            best, best_d = None, join_dist
            e = segs[i][end]
            di = segs[i][1] - segs[i][0]
            di = di / np.hypot(*di)
            for j in range(n):
                if i == j:
                    continue
                dj = segs[j][1] - segs[j][0]
                lj = np.hypot(*dj)
                dj = dj / lj
                cross = di[0] * dj[1] - di[1] * dj[0]
                if abs(cross) < np.sin(np.radians(20.0)):
                    continue
                # intersection of the two infinite lines
                w = segs[j][0] - segs[i][0]
                t_i = (w[0] * dj[1] - w[1] * dj[0]) / cross
                x = segs[i][0] + di * t_i
                if np.hypot(*(x - e)) > best_d:
                    continue
                t_j = (x - segs[j][0]) @ dj
                if -join_dist <= t_j <= lj + join_dist:
                    best, best_d = x, np.hypot(*(x - e))
            if best is not None:
                segs[i][end] = best
    return [(s[0], s[1], s[2]) for s in segs]


def _draw_segments(shape, segs, thickness_cells):
    mask = np.zeros(shape, bool)
    for p0, p1, _ in segs:
        ln = float(np.hypot(*(p1 - p0)))
        k = max(2, int(ln * 2))
        t = np.linspace(0.0, 1.0, k)
        x = np.rint(p0[0] + (p1[0] - p0[0]) * t).astype(int)
        y = np.rint(p0[1] + (p1[1] - p0[1]) * t).astype(int)
        ok = (x >= 0) & (x < shape[1]) & (y >= 0) & (y < shape[0])
        mask[y[ok], x[ok]] = True
    r = max(0, int(round(thickness_cells / 2.0 - 0.5)))
    if r > 0:
        mask = ndi.binary_dilation(mask, structure=ndi.generate_binary_structure(2, 2), iterations=r)
    return mask


# --------------------------------------------------------------------------
def clean_map(grid: np.ndarray, res: float, params: CleanParams | None = None) -> CleanResult:
    """grid: int8/int array (h, w) with -1/0/100 (or 0..100) occupancy values."""
    p = params or CleanParams()
    g = np.asarray(grid)
    occ = g >= p.occ_threshold
    stats = {"occupied_before": int(occ.sum())}
    cell = lambda m: max(1.0, m / res)

    # --- 1. noise removal -------------------------------------------------
    link = int(round(cell(p.link_gap_m) / 2.0))
    linked = ndi.binary_dilation(occ, structure=np.ones((3, 3), bool), iterations=link) if link > 0 else occ
    lab, n = _components(linked)
    keep = np.zeros(n + 1, bool)
    if n:
        objs = ndi.find_objects(lab)
        cnt = ndi.sum(occ, lab, index=np.arange(1, n + 1))
        min_ext = cell(p.min_blob_m)
        for k, sl in enumerate(objs, start=1):
            ext = max(sl[0].stop - sl[0].start, sl[1].stop - sl[1].start)
            keep[k] = (ext >= min_ext) and (cnt[k - 1] >= p.min_blob_cells)
    kept = occ & keep[lab]
    removed = occ & ~kept
    stats["noise_removed_cells"] = int(removed.sum())

    # --- 2. straight wall extraction -----------------------------------------
    # Thick clusters (pillars, machines) are objects: keep them as they are and do
    # not let RANSAC cut diagonal "walls" through them.
    blob = np.zeros_like(kept)
    if kept.any():
        rad = max(1.5, cell(p.max_wall_thickness_m) / 2.0)
        core = ndi.distance_transform_edt(kept) > rad
        if core.any():
            blob = ndi.binary_dilation(core, structure=np.ones((3, 3), bool),
                                       iterations=int(np.ceil(rad)) + 1) & kept
    ys, xs = np.nonzero(kept & ~blob)
    pts = np.stack([xs, ys], axis=1).astype(float)
    rng = np.random.default_rng(p.seed)
    segs, leftover = _ransac_segments(
        pts, tol=cell(p.ransac_tol_m), min_len=cell(p.min_wall_m),
        max_gap=cell(p.max_gap_m), max_lines=p.max_lines, rng=rng)
    stats["lines_raw"] = len(segs)

    # --- 3. regularise ------------------------------------------------------
    segs = _snap_directions(segs, p.snap_deg)
    segs = _merge_collinear(segs, cell(p.merge_dist_m), cell(p.max_gap_m))
    segs = _snap_directions(segs, p.snap_deg)
    segs = _drop_ghosts(segs, cell(p.ghost_dist_m))
    segs = _join_corners(segs, cell(p.join_dist_m))
    stats["lines"] = len(segs)

    # --- 4. rebuild ---------------------------------------------------------
    walls = _draw_segments(g.shape, segs, cell(p.wall_thickness_m))
    other = np.zeros_like(kept)
    if blob.any():
        by, bx = np.nonzero(blob)
        leftover = np.vstack([leftover, np.stack([bx, by], axis=1).astype(float)]) if len(leftover) else \
            np.stack([bx, by], axis=1).astype(float)
    if p.keep_other_objects and len(leftover):
        # keep leftover cells that are part of a sizeable non-wall cluster
        lm = np.zeros_like(kept)
        lm[leftover[:, 1].astype(int), leftover[:, 0].astype(int)] = True
        near_wall = ndi.binary_dilation(walls, iterations=int(round(cell(p.ransac_tol_m) * 1.5)))
        lm &= ~near_wall
        lab2, n2 = _components(ndi.binary_dilation(lm, iterations=1))
        if n2:
            cnt2 = ndi.sum(lm, lab2, index=np.arange(1, n2 + 1))
            ok2 = np.zeros(n2 + 1, bool)
            ok2[1:] = cnt2 >= max(4, p.min_blob_cells)
            other = lm & ok2[lab2]
    stats["other_object_cells"] = int(other.sum())

    out = g.astype(np.int8).copy()
    # Old obstacle cells are cleared first. A cleared cell becomes free only if it
    # touches known free space (speckle inside a room); wobbly cells on the far
    # side of a wall touch only unknown space and go back to unknown.
    touches_free = ndi.binary_dilation(g == FREE, structure=np.ones((3, 3), bool))
    out[occ] = np.where(touches_free[occ], FREE, UNKNOWN).astype(np.int8)
    out[walls] = OCC
    out[other] = OCC
    # Cleared cells that ended up as free but are not connected (4-neighbour) to
    # any originally-free cell are leftovers on the far side of a wall -> unknown.
    lab_f, n_f = ndi.label(out == FREE)
    if n_f:
        has_orig = ndi.maximum((g == FREE).astype(np.uint8), lab_f, index=np.arange(1, n_f + 1)) > 0
        bad = np.zeros(n_f + 1, bool)
        bad[1:] = ~has_orig
        out[bad[lab_f] & (out == FREE)] = UNKNOWN
    stats["occupied_after"] = int((out >= p.occ_threshold).sum())
    seg_list = [(float(a[0]), float(a[1]), float(b[0]), float(b[1])) for a, b, _ in segs]
    return CleanResult(grid=out, segments=seg_list, stats=stats)


def rasterize_segments(shape, segments, thickness_cells: float) -> np.ndarray:
    """Public helper: boolean mask of (x0, y0, x1, y1) segments in cell coords."""
    segs = [(np.array([a, b], float), np.array([c, d], float), 0) for a, b, c, d in segments]
    return _draw_segments(shape, segs, thickness_cells)
