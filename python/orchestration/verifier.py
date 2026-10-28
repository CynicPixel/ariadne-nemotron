"""The hypothesis verifier (D316): deterministic, from the OBSERVED slab only. The only code that sets a hypothesis's
status (CLAUDE.md §1): never the model's words, never simulator truth.

A hypothesis claims: the frontier at centroid C opens onto a passage a responder can use (clearance rho_responder,
0.40 m; invariant 5). Its anchor's standoff goal G is where an agent looks from, so the claim is tested as a CROSSING
of the disk of radius R around C, from the side facing G to the far side:

  u = (C - G) / |C - G|; for each disk cell, s = (cell - C) . u
  entry band: s <= -R/2      far band: s >= +R/2
  responder-clear cell: no OCCUPIED cell within rho (a disk), and the cell itself passable in the map at hand
  pessimistic map: passable = observed FREE           optimistic map: passable = FREE or UNKNOWN

  confirmed   an 8-connected responder-clear path from the entry band to the far band exists in the PESSIMISTIC map
              (observed free space alone proves the passage)
  refuted     no such path exists even in the OPTIMISTIC map (observed obstacles alone rule it out, whatever the
              unknown cells hold)
  supported   pessimistic: clear cells connected to the entry band reach within R/2 of C (the way in is seen open)
  unresolved  otherwise

Both bounds use only what was observed, so a confirmation or a refutation cannot be overturned by unseen space within
the disk (it can by a later map revision: the store logs that as a contradiction, it never flips silently; and a path
leaving the disk is out of this local test's scope). Cells outside the grid are unknown.

Slab convention (ariadne_live.cpp, occupancy_grid.hpp): uint8 0 unknown, 1 free, 2 occupied; row = y index (north),
col = x index (east), cell centre = origin + (index + 0.5) * res. The PNG is snapshots/slab_<ms>.png, written raw
(unflipped, unscaled) at the same replan as the request (D314); map_png is never read here.
"""
from __future__ import annotations

import math
from collections import deque
from pathlib import Path

import numpy as np

UNKNOWN, FREE, OCCUPIED = 0, 1, 2
OPEN = ("unresolved", "supported")
TERMINAL = ("confirmed", "refuted", "expired")


class SlabMismatch(ValueError):
    """The slab is not this decision's (D314 guard): evidence must never come from a later replan."""


def slab_ms(path: str) -> int:
    stem = Path(path).stem   # slab_<ms>
    if not stem.startswith("slab_"):
        raise SlabMismatch(f"not a slab snapshot: {path}")
    return int(stem[len("slab_"):])


def check_slab(req: dict) -> dict:
    """The request's slab block, checked against the request (FM condition on D314): same t_s, and a file stamped at
    or before the decision (decision ms is T truncated, the snapshot's T rounded: they differ by <= 1 at one replan)."""
    s = req.get("slab")
    if not isinstance(s, dict) or not s.get("path"):
        raise SlabMismatch("request has no slab")
    if abs(float(s["t_s"]) - float(req["t_s"])) > 1e-6:
        raise SlabMismatch(f"slab t_s {s['t_s']} != request t_s {req['t_s']}")
    g = s.get("grid_nx_ny")
    if g is not None and [int(g[0]), int(g[1])] != [int(s["cols"]), int(s["rows"])]:
        raise SlabMismatch(f"slab {s['cols']}x{s['rows']} is not the briefing grid {g[0]}x{g[1]}: origin would be wrong")
    did_ms = int(req["decision_id"][1:-2])
    if slab_ms(s["path"]) > did_ms + 1:
        raise SlabMismatch(f"slab {s['path']} is later than decision {req['decision_id']}")
    return s


def load_slab(s: dict) -> np.ndarray:
    from PIL import Image
    a = np.asarray(Image.open(s["path"]))
    if a.ndim != 2 or a.dtype != np.uint8:
        raise SlabMismatch(f"slab {s['path']} is not 8-bit single-channel")
    if a.shape != (int(s["rows"]), int(s["cols"])):
        raise SlabMismatch(f"slab shape {a.shape} != rows/cols {s['rows']}x{s['cols']}")
    if a.max(initial=0) > OCCUPIED:
        raise SlabMismatch(f"slab {s['path']} has values above {OCCUPIED}")
    return a


def _disk(r: int) -> list[tuple[int, int]]:
    return [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1) if dy * dy + dx * dx <= r * r]


def _reach(clear: np.ndarray, seeds: np.ndarray) -> np.ndarray:
    """8-connected flood fill over `clear` from the seed cells."""
    seen = seeds & clear
    q = deque(zip(*np.nonzero(seen)))
    h, w = clear.shape
    while q:
        y, x = q.popleft()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and clear[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    q.append((ny, nx))
    return seen


def verify(slab: np.ndarray, res_m: float, origin_xy_m, centroid_xy_m, goal_xy_m, region_radius_m: float = 1.5,
           rho_m: float = 0.40) -> dict:
    """One verdict for one hypothesis on one slab -> {verdict, reason, metrics}. Pure: same inputs, same answer."""
    ox, oy = origin_xy_m
    cx, cy = centroid_xy_m
    ux, uy = cx - goal_xy_m[0], cy - goal_xy_m[1]
    n = math.hypot(ux, uy)
    if n < res_m:
        return {"verdict": "unresolved", "reason": "degenerate anchor: goal at the centroid", "metrics": {}}
    ux, uy = ux / n, uy / n
    R, rho = max(1, round(region_radius_m / res_m)), max(1, math.ceil(rho_m / res_m - 1e-9))
    cr, cc = math.floor((cy - oy) / res_m), math.floor((cx - ox) / res_m)
    pad = R + rho
    # the window, with cells outside the grid as unknown
    win = np.full((2 * pad + 1, 2 * pad + 1), UNKNOWN, np.uint8)
    r0, c0 = cr - pad, cc - pad
    rs, re_ = max(0, r0), min(slab.shape[0], cr + pad + 1)
    cs, ce = max(0, c0), min(slab.shape[1], cc + pad + 1)
    if rs < re_ and cs < ce:
        win[rs - r0:re_ - r0, cs - c0:ce - c0] = slab[rs:re_, cs:ce]
    occ = win == OCCUPIED
    blocked = np.zeros_like(occ)
    for dy, dx in _disk(rho):   # any occupied cell within rho of a cell blocks it for a responder
        blocked |= np.roll(np.roll(occ, dy, 0), dx, 1)
    # np.roll wraps; the window is padded by rho beyond the disk, so wrapped cells never reach the disk
    yy, xx = np.mgrid[-pad:pad + 1, -pad:pad + 1]
    in_disk = yy * yy + xx * xx <= R * R
    s = (xx * ux + yy * uy)   # cells: x east = +col, y north = +row (row 0 = south)
    entry, far = in_disk & (s <= -R / 2), in_disk & (s >= R / 2)
    near_c = in_disk & (yy * yy + xx * xx <= (R / 2) ** 2)
    pess = in_disk & (win == FREE) & ~blocked
    opt = in_disk & (win != OCCUPIED) & ~blocked
    reach_p = _reach(pess, entry)
    reach_o = _reach(opt, entry)
    known = int((in_disk & (win != UNKNOWN)).sum())
    m = {"disk_cells": int(in_disk.sum()), "known_frac": round(known / max(1, int(in_disk.sum())), 3),
         "occupied_cells": int((in_disk & occ).sum()), "clear_pess": int(pess.sum()), "clear_opt": int(opt.sum()),
         "rho_cells": rho, "radius_cells": R}
    if (reach_p & far).any():
        return {"verdict": "confirmed", "reason": f"observed free crossing at {rho_m:.2f} m clearance", "metrics": m}
    if not (reach_o & far).any():
        why = "no clear cell on the entry side" if not (opt & entry).any() else "observed obstacles close every crossing"
        return {"verdict": "refuted", "reason": f"{why} at {rho_m:.2f} m clearance (unknown treated as free)",
                "metrics": m}
    if (reach_p & near_c).any():
        return {"verdict": "supported", "reason": "observed free way in reaches the frontier; far side unseen",
                "metrics": m}
    return {"verdict": "unresolved", "reason": "not enough observed space to decide", "metrics": m}


def next_status(current: str, verdict: str | None, unmatched_replans: int, expire_after: int) -> tuple[str, str | None]:
    """The status transition (D316) -> (status, event). Terminal states stay; a verdict that disagrees with a terminal
    status is reported as a contradiction (the caller logs it), never applied. An open hypothesis with no matching
    candidate for expire_after replans expires."""
    if current in TERMINAL:
        if current != "expired" and verdict in ("confirmed", "refuted") and verdict != current:
            return current, "contradiction"
        return current, None
    if verdict in ("confirmed", "refuted"):
        return verdict, "status"
    if unmatched_replans >= expire_after:
        return "expired", "status"
    if verdict in OPEN and verdict != current:
        return verdict, "status"
    return current, None
