"""map_editor.py - manual editing of an occupancy grid (no Qt, only numpy, so it can be unit-tested).

Grid convention (same as MapWidget / map_io): int16 array [h, w], row 0 = LOWEST y, values
-1 = unknown, 0 = free, 100 = occupied.  Positions are "cell coordinates": u = column + fraction,
v = row + fraction (u = (world_x - origin_x) / resolution).  Cell (r, c) covers [c, c+1) x [r, r+1).

Every user action (one brush stroke, one line, one rectangle, one thicken...) is ONE undo step.
Undo data is stored per 64x64 tile that was really touched, so a big map costs little memory.
"""
from __future__ import annotations

import math

import numpy as np

WALL, FREE, UNKNOWN = 100, 0, -1
TILE = 64
OCC_THRESHOLD = 50


class MapEditor:
    def __init__(self, grid: np.ndarray, resolution: float, max_undo: int = 80):
        if grid.dtype != np.int16:
            raise ValueError("grid must be int16 (it is edited in place)")
        self.grid = grid
        self.res = float(resolution)
        self.max_undo = int(max_undo)
        self._undo: list[dict] = []
        self._redo: list[dict] = []
        self._stroke: dict | None = None          # (ti, tj) -> copy of the tile BEFORE the first change

    # -- undo bookkeeping ----------------------------------------------------------
    @property
    def can_undo(self) -> bool:
        return bool(self._undo)

    @property
    def can_redo(self) -> bool:
        return bool(self._redo)

    def begin(self) -> None:
        """Start a group of operations that undo as one step."""
        if self._stroke is None:
            self._stroke = {}

    def end(self) -> bool:
        """Close the group. True if the map really changed (then it is one undo step)."""
        stroke, self._stroke = self._stroke, None
        if not stroke:
            return False
        step = {}
        for key, before in stroke.items():
            r0, c0 = key[0] * TILE, key[1] * TILE
            after = self.grid[r0:r0 + before.shape[0], c0:c0 + before.shape[1]].copy()
            if not np.array_equal(before, after):
                step[key] = (before, after)
        if not step:
            return False
        self._undo.append(step)
        del self._undo[:-self.max_undo]
        self._redo.clear()
        return True

    def _touch(self, r0: int, r1: int, c0: int, c1: int) -> None:
        """Save the tiles of the (clipped) rectangle rows r0:r1, cols c0:c1 before they change."""
        if self._stroke is None:
            raise RuntimeError("call begin() before editing")
        for ti in range(r0 // TILE, (r1 - 1) // TILE + 1):
            for tj in range(c0 // TILE, (c1 - 1) // TILE + 1):
                if (ti, tj) not in self._stroke:
                    self._stroke[(ti, tj)] = self.grid[ti * TILE:(ti + 1) * TILE, tj * TILE:(tj + 1) * TILE].copy()

    def undo(self) -> bool:
        if not self._undo:
            return False
        step = self._undo.pop()
        for (ti, tj), (before, _after) in step.items():
            self.grid[ti * TILE:ti * TILE + before.shape[0], tj * TILE:tj * TILE + before.shape[1]] = before
        self._redo.append(step)
        return True

    def redo(self) -> bool:
        if not self._redo:
            return False
        step = self._redo.pop()
        for (ti, tj), (_before, after) in step.items():
            self.grid[ti * TILE:ti * TILE + after.shape[0], tj * TILE:tj * TILE + after.shape[1]] = after
        self._undo.append(step)
        return True

    # -- drawing primitives (cell coordinates) -------------------------------------
    def _clip(self, r0, r1, c0, c1):
        h, w = self.grid.shape
        return max(0, r0), min(h, r1), max(0, c0), min(w, c1)

    def line(self, u0: float, v0: float, u1: float, v1: float, width_cells: float, value: int) -> None:
        """Thick line with round ends (a 'capsule'). A zero length line is a round dab, i.e. a brush stamp."""
        rad = max(0.5, float(width_cells) / 2.0)
        r0, r1, c0, c1 = self._clip(int(math.floor(min(v0, v1) - rad)), int(math.ceil(max(v0, v1) + rad)) + 1,
                                    int(math.floor(min(u0, u1) - rad)), int(math.ceil(max(u0, u1) + rad)) + 1)
        if r0 >= r1 or c0 >= c1:
            return
        self._touch(r0, r1, c0, c1)
        cu = np.arange(c0, c1, dtype=np.float64) + 0.5            # cell centres
        cv = np.arange(r0, r1, dtype=np.float64)[:, None] + 0.5
        du, dv = u1 - u0, v1 - v0
        L2 = du * du + dv * dv
        if L2 < 1e-12:
            t = np.zeros((r1 - r0, c1 - c0))
        else:
            t = np.clip(((cu - u0) * du + (cv - v0) * dv) / L2, 0.0, 1.0)
        d2 = (cu - (u0 + t * du)) ** 2 + (cv - (v0 + t * dv)) ** 2
        self.grid[r0:r1, c0:c1][d2 <= rad * rad] = value

    def rect(self, u0: float, v0: float, u1: float, v1: float, value: int) -> None:
        """Fill every cell whose centre is inside the rectangle spanned by the two corners."""
        c0, c1 = int(math.floor(min(u0, u1))), int(math.ceil(max(u0, u1)))
        r0, r1 = int(math.floor(min(v0, v1))), int(math.ceil(max(v0, v1)))
        r0, r1, c0, c1 = self._clip(r0, r1, c0, c1)
        if r0 >= r1 or c0 >= c1:
            return
        self._touch(r0, r1, c0, c1)
        self.grid[r0:r1, c0:c1] = value

    # -- wall thickness ------------------------------------------------------------------
    @staticmethod
    def _grow(mask: np.ndarray, pad_value: bool) -> np.ndarray:
        """One 8-neighbour dilation step of a boolean mask (outside the map counts as pad_value)."""
        p = np.pad(mask, 1, constant_values=pad_value)
        h, w = mask.shape
        out = mask.copy()
        for dr in (0, 1, 2):
            for dc in (0, 1, 2):
                out |= p[dr:dr + h, dc:dc + w]
        return out

    def thicken(self, cells: int = 1) -> int:
        """Make every wall `cells` cells thicker on each side. One undo step. Returns changed cells."""
        occ = self.grid > OCC_THRESHOLD
        grown = occ
        for _ in range(max(1, int(cells))):
            grown = self._grow(grown, False)
        return self._apply_mask(grown & ~occ, WALL)

    def thin(self, cells: int = 1) -> int:
        """Make every wall `cells` cells thinner on each side (thin walls may disappear). One undo step."""
        occ = self.grid > OCC_THRESHOLD
        free = ~occ
        for _ in range(max(1, int(cells))):
            free = self._grow(free, False)           # the map border never eats into a wall
        return self._apply_mask(occ & free, FREE)

    def clear_small_blobs(self, min_cells: int = 4) -> int:
        """Remove isolated occupied specks of fewer than `min_cells` cells (8-connected). One undo step."""
        occ = self.grid > OCC_THRESHOLD
        h, w = occ.shape
        seen = np.zeros_like(occ)
        kill = np.zeros_like(occ)
        for r, c in zip(*np.nonzero(occ)):
            if seen[r, c]:
                continue
            comp, stack = [], [(r, c)]
            seen[r, c] = True
            while stack and len(comp) < min_cells:
                y, x = stack.pop()
                comp.append((y, x))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < h and 0 <= nx < w and occ[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            stack.append((ny, nx))
            if len(comp) < min_cells and not stack:
                for y, x in comp:
                    kill[y, x] = True
            else:                                    # big blob: flood the rest so it is not visited again
                while stack:
                    y, x = stack.pop()
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < h and 0 <= nx < w and occ[ny, nx] and not seen[ny, nx]:
                                seen[ny, nx] = True
                                stack.append((ny, nx))
        return self._apply_mask(kill, FREE)

    def _apply_mask(self, mask: np.ndarray, value: int) -> int:
        n = int(mask.sum())
        if n == 0:
            return 0
        rows, cols = np.nonzero(mask)
        self.begin()
        self._touch(int(rows.min()), int(rows.max()) + 1, int(cols.min()), int(cols.max()) + 1)
        self.grid[mask] = value
        self.end()
        return n


def snap_angle(u0: float, v0: float, u1: float, v1: float, step_deg: float = 45.0):
    """Move the end point so the line from (u0, v0) has an angle that is a multiple of step_deg."""
    dx, dy = u1 - u0, v1 - v0
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return u1, v1
    ang = round(math.degrees(math.atan2(dy, dx)) / step_deg) * step_deg
    return u0 + length * math.cos(math.radians(ang)), v0 + length * math.sin(math.radians(ang))
