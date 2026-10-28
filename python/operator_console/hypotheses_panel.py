"""The console's Hypotheses panel backend (fork F3.5; record schema: docs/design/nemotron-evidence-loop.md §3.1).

  GET /api/hypotheses        every hypothesis as its newest record says (last line per hypothesis_id), status counts,
                             the newest status changes, and the newest decision's model block from the runtime trace
  GET /api/hypotheses/file   one provenance image (decision map or keyframe PNG/JPEG) from inside the run directory

Read-only: it never writes to the run and never computes a status (statuses come only from orchestration/verifier.py,
D316; the panel shows what the records say). Sources:
  ARIADNE_LIVE=<run>:<scene>   <run>/hypotheses.jsonl (written beside decisions/ by orchestration/hypotheses.py) and
                               trace.jsonl from the ariadne_live directory inside <run>
  ARIADNE_HYPOTHESES=<file>    a recorded hypotheses.jsonl instead (its directory is then the run root). API only: `/`
                               serves index.html only with ARIADNE_LIVE; to show a finished run's panel, point
                               ARIADNE_LIVE at that run directory.
The reader is tolerant: a half-written last line, an unknown event or a missing field never fails a request. Both files
are re-read only when their size or mtime changes, and the trace is read from its tail (the console shares a CPU-bound
box with the simulator during a recording).
"""
from __future__ import annotations

import json
import os
from pathlib import Path

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, JSONResponse

router = APIRouter(prefix="/api/hypotheses")
STATUSES = ("unresolved", "supported", "confirmed", "refuted", "expired")
IMAGE_TYPES = {".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg"}
TRACE_TAIL_MIN, TRACE_TAIL_MAX = 256 << 10, 8 << 20   # a trace line carries every tool call's result: start with the
                                                     # last 256 KiB, double until a decision is found, stop at 8 MiB
_cache: dict = {}


def _cached(kind: str, f: Path, load):
    """load(f), recomputed only when f's (size, mtime) changes."""
    st = f.stat()
    key = (st.st_size, st.st_mtime_ns)
    hit = _cache.get((kind, f))
    if hit and hit[0] == key:
        return hit[1]
    val = load(f)
    _cache[(kind, f)] = (key, val)
    return val


def _sources() -> tuple[Path | None, Path | None]:
    """(run root, hypotheses.jsonl): None where there is nothing to show yet."""
    if os.environ.get("ARIADNE_HYPOTHESES"):
        f = Path(os.environ["ARIADNE_HYPOTHESES"])
        return f.parent, f
    if "ARIADNE_LIVE" not in os.environ:
        return None, None
    root = Path(os.environ["ARIADNE_LIVE"].split(":")[0])
    for f in [root / "hypotheses.jsonl", *sorted(root.glob("*/hypotheses.jsonl"))]:
        if f.is_file():
            return root, f
    return root, None


def _events(f: Path | None) -> list[dict]:
    if not f or not f.is_file():
        return []
    return _cached("events", f, _read_events)


def _read_events(f: Path) -> list[dict]:
    out = []
    for line in f.read_text(errors="replace").splitlines():
        try:
            r = json.loads(line)
        except ValueError:   # the writer may be mid-line
            continue
        if isinstance(r, dict) and r.get("hypothesis_id"):
            out.append(r)
    return out


def _run_relative(p: str | None) -> str | None:
    """A provenance path as run-relative. The layer writes them run-relative; older records carry the container's
    absolute path, so keep everything from the run's own decisions/ or snapshots/ directory onward."""
    if not p:
        return None
    if not p.startswith("/"):
        return p
    for marker in ("/decisions/", "/snapshots/"):
        if marker in p:
            return p[p.rindex(marker) + 1:]
    return None


def _resolve(root: Path, rel: str) -> Path | None:
    """rel inside the run root, or inside the ariadne_live directory one level down; never outside root."""
    base = root.resolve()
    for cand in [root / rel, *sorted(root.glob(f"*/{rel}"))]:
        p = cand.resolve()
        if p.is_file() and p.is_relative_to(base) and p.suffix.lower() in IMAGE_TYPES:
            return p
    return None


def _image(root: Path | None, p: str | None) -> str | None:
    rel = _run_relative(p)
    if root is None or rel is None or _resolve(root, rel) is None:
        return None
    return f"/api/hypotheses/file?path={rel}"


def fold(events: list[dict], root: Path | None) -> list[dict]:
    """The newest state of each hypothesis, open ones first, then by id."""
    state: dict[str, dict] = {}
    for r in events:
        hid = r["hypothesis_id"]
        h = state.setdefault(hid, {"hypothesis_id": hid, "history": [], "targeted": []})
        if r.get("event") == "targeted":
            h["targeted"].append({"sim_t_s": r.get("sim_t_s"), "agent": r.get("targeted_by_agent"),
                                  "candidate": r.get("candidate"), "decision_id": r.get("decision_id")})
        if r.get("status") and (not h["history"] or h["history"][-1]["status"] != r["status"]):
            h["history"].append({"sim_t_s": r.get("sim_t_s"), "status": r["status"], "source": r.get("status_source")})
        prov = r.get("provenance") or {}
        h.update({"agent_id": r.get("agent_id"), "status": r.get("status"), "status_source": r.get("status_source"),
                  "claim": r.get("claim"), "evidence_goal": r.get("evidence_goal") or h.get("evidence_goal"),
                  "sim_t_s": r.get("sim_t_s"), "decision_id": r.get("decision_id"), "anchor": r.get("anchor"),
                  "verifier": r.get("verifier") or h.get("verifier"), "source": prov.get("source"),
                  "map_url": _image(root, prov.get("map_png")), "keyframe_url": _image(root, prov.get("keyframe_path"))})
        if r.get("orchestrator"):   # present only on VL-proposer records (§3.1)
            h["orchestrator"] = r["orchestrator"]
    rank = {"unresolved": 0, "supported": 0}
    return sorted(state.values(), key=lambda h: (rank.get(h.get("status"), 1), h["hypothesis_id"]))


def _trace(root: Path | None) -> Path | None:
    if root is None:
        return None
    for f in [root / "trace.jsonl", *sorted(root.glob("*/trace.jsonl"))]:
        if f.is_file():
            return f
    return None


def latest_decision(f: Path | None) -> dict | None:
    """The newest trace record whose choice came back from the orchestrator: its model, latency and tokens, as logged."""
    return _cached("decision", f, _read_decision) if f else None


def _read_decision(f: Path) -> dict | None:
    size, n = f.stat().st_size, TRACE_TAIL_MIN
    while True:
        found = _newest_decision(f, size, n)
        if found or n >= min(size, TRACE_TAIL_MAX):
            return found
        n *= 2


def _newest_decision(f: Path, size: int, n: int) -> dict | None:
    with open(f, "rb") as fh:
        fh.seek(max(0, size - n))
        lines = fh.read(n).decode(errors="replace").splitlines()
    if size > n:
        lines = lines[1:]   # the first line of a tail is cut
    for line in reversed(lines):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        ch = r.get("choice") or {}
        o = (ch.get("answer") or {}).get("orchestrator")
        if o:
            return {"t_s": r.get("t_s"), "decision_id": ch.get("decision_id"), "source": ch.get("source"),
                    "validation": ch.get("validation"), "provider": o.get("provider"), "model": o.get("model"),
                    "latency_s": o.get("latency_s"), "input_tokens": o.get("input_tokens"),
                    "output_tokens": o.get("output_tokens"), "fallback_used": o.get("fallback_used"),
                    "errors": len(o.get("errors") or [])}
    return None


@router.get("")
def hypotheses() -> JSONResponse:
    root, f = _sources()
    ev = _events(f)
    hs = fold(ev, root)
    changes = [{"sim_t_s": r.get("sim_t_s"), "hypothesis_id": r["hypothesis_id"], "status": r.get("status"),
                "event": r.get("event")} for r in ev if r.get("event") in ("status", "contradiction")][-20:]
    return JSONResponse({"available": f is not None, "hypotheses": hs,
                         "counts": {s: sum(1 for h in hs if h.get("status") == s) for s in STATUSES},
                         "changes": changes, "decision": latest_decision(_trace(root))})


@router.get("/file")
def provenance_file(path: str) -> FileResponse:
    root, _ = _sources()
    p = _resolve(root, path) if root is not None and not path.startswith("/") else None
    if p is None:
        raise HTTPException(404, "no such image in this run")
    return FileResponse(p, media_type=IMAGE_TYPES[p.suffix.lower()])
