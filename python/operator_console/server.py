"""Operator console stub backend (arch §29.3; design live-runtime.md#operator-console): FastAPI, WebSocket telemetry at 5 Hz, REST
for metadata and map snapshots. Two sources, one frame schema:

  ARIADNE_CHAIN=collab_run:mission_run:scene ...   replay of a committed run chain (Phase 1)
  ARIADNE_LIVE=run_dir:scene ...                   a running ariadne_live (design live-runtime.md D154): tails its
                                                   telemetry; every frame served is logged to console_frames.jsonl
  uvicorn operator_console.server:app --host 127.0.0.1 --port 8080
"""
from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.gzip import GZipMiddleware
from .demo import router as demo_router, manager as demo_manager

from .live import FrameLog, LiveRun
from .replay import TICK_HZ, frame, load_chain

STATIC = Path(__file__).with_name("static")
@asynccontextmanager
async def lifespan(app):
    demo_manager.start()
    try:
        yield
    finally:
        demo_manager.close()

app = FastAPI(title="Ariadne mission console", lifespan=lifespan)
app.add_middleware(GZipMiddleware, minimum_size=1024)
app.include_router(demo_router)
app.mount("/static", StaticFiles(directory=STATIC), name="static")

@app.middleware("http")
async def protect_legacy(request, call_next):
    # The hosted controller has one mutation boundary; legacy file APIs cannot bypass it.
    if demo_manager.enabled and request.method == "POST" and not request.url.path.startswith("/api/demo/"):
        return JSONResponse({"detail": "Use the authenticated demo control API"}, status_code=403)
    return await call_next(request)

@app.get("/demo")
def demo_index():
    return HTMLResponse((STATIC / "demo.html").read_text())


# The splat panel (main 3f5e310): /splat page + chunk/table/ws + world.obj; it reads
# ARIADNE_SPLAT_DIR (trainer output) and ARIADNE_SCENE_DIR (worldgen scene, the GT viewport). Separate page,
# linked from the console header - never blended with the GT model (user rule: own viewport).
from operator_console.splat import router as splat_router  # noqa: E402
app.include_router(splat_router)
from operator_console.hypotheses_panel import router as hypotheses_router  # noqa: E402  fork F3.5: read-only panel
app.include_router(hypotheses_router)
_chain = None
_live = None
_live_log = None


def live(poll: bool = True):
    """The live run, or None when serving a replay chain. poll=False: the run directory is all the caller needs (the
    operator endpoints); the telemetry tail is advanced only by the streams."""
    global _live, _live_log
    if "ARIADNE_LIVE" not in os.environ:
        return None
    if _live is None:
        run, scene = os.environ["ARIADNE_LIVE"].split(":")
        _live = LiveRun(Path(run), Path(scene))
        _live_log = FrameLog(Path(run))
    if poll:
        _live.poll()
    return _live


def chain():
    global _chain
    if _chain is None:
        collab, mission, scene = os.environ["ARIADNE_CHAIN"].split(":")
        _chain = load_chain(Path(collab), Path(mission), Path(scene))
    return _chain


@app.get("/")
def index() -> HTMLResponse:
    return HTMLResponse((STATIC / ("demo.html" if demo_manager.enabled or not ("ARIADNE_LIVE" in os.environ or "ARIADNE_CHAIN" in os.environ) else "index.html")).read_text())


@app.get("/healthz")
def healthz() -> dict:
    return {"ok": True}


@app.get("/api/meta")
def meta() -> JSONResponse:
    lv = live()
    return JSONResponse(lv.meta() if lv else chain().meta)


@app.get("/api/map/{i}.png")
def map_png(i: int) -> FileResponse:
    lv = live()
    snaps = lv.snapshots if lv else chain().snapshots
    if not 0 <= i < len(snaps):
        raise HTTPException(404, "no such map version")
    return FileResponse(snaps[i][1], media_type="image/png")


@app.get("/api/cam/{k}.jpg")
def cam_jpg(k: int) -> FileResponse:
    """The agent's latest keyframe image (P8: the demo camera windows; the runtime overwrites it per keyframe)."""
    lv = live()
    if not lv:
        raise HTTPException(404, "live only")
    f = lv.src() / "snapshots" / f"cam_{k}.jpg"
    if not f.exists():
        raise HTTPException(404, "no camera frame yet")
    return FileResponse(f, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/api/cam/{k}.json")
def cam_features(k: int) -> FileResponse:
    """The tracked observations of that keyframe (pixel coords in cam_<k>.jpg): the feature overlay (item 6)."""
    lv = live()
    if not lv:
        raise HTTPException(404, "live only")
    f = lv.src() / "snapshots" / f"cam_{k}.json"
    if not f.exists():
        raise HTTPException(404, "no features yet")
    return FileResponse(f, media_type="application/json", headers={"Cache-Control": "no-store"})


@app.post("/api/collab/{action}")
def collab_action(action: str, groups: str = "") -> JSONResponse:
    """D221 panel buttons: cut_comms (groups like "0|1,2,3"), restore_comms, inject_false. The file lands in the
    run's collab_inbox and the runtime applies it at the next barrier (a recorded input, like directives)."""
    lv = live()
    if not lv:
        raise HTTPException(404, "live only")
    if action not in ("cut_comms", "restore_comms", "inject_false"):
        raise HTTPException(400, "action: cut_comms|restore_comms|inject_false")
    doc: dict = {"action": action}
    if action == "cut_comms":
        try:
            doc["groups"] = [[int(a) for a in g.split(",")] for g in groups.split("|") if g]
        except ValueError:
            raise HTTPException(400, "groups: like 0|1,2,3")
        if not doc["groups"]:
            raise HTTPException(400, "cut_comms needs groups")
    inbox = lv.src() / "collab_inbox"
    inbox.mkdir(exist_ok=True)
    import time as _t
    f = inbox / f"{int(_t.time() * 1000)}_{action}.json"
    f.write_text(json.dumps(doc) + "\n")
    return JSONResponse({"queued": f.name})


@app.get("/api/frame")
def frame_at(t: float, speed: float = 10.0) -> JSONResponse:
    """One frame at mission time t (static view, screenshots): events from 0..t included."""
    lv = live()
    if lv:
        f = lv.frame(-1, -1.0)   # live: the latest state, every event so far
        return JSONResponse(f)
    ch = chain()
    n = int(round(t * TICK_HZ / speed))
    f = frame(ch, n, speed)
    f["events"] = [e for e in ch.events if e["t_s"] <= f["t_s"]]
    return JSONResponse(f)


@app.websocket("/ws/telemetry")
async def telemetry(ws: WebSocket, speed: float = 10.0) -> None:
    await ws.accept()
    lv = live()
    if lv:
        client = f"{ws.client.host}:{ws.client.port}" if ws.client else "?"
        n, since, since_pose = 0, -1.0, -1.0
        try:
            while True:
                lv.poll()
                f = lv.frame(n, since, since_pose)
                await ws.send_json(f)
                _live_log.put(client, f)
                since = max(since, max((e["t_s"] for e in f["events"]), default=since))
                since_pose = max([since_pose] + [s[0] for a in f["agents"] for s in a.get("samples", [])])
                n += 1
                if lv.ended:
                    await ws.send_json({"end": True, "t_s": f["t_s"]})
                    await ws.close()
                    return
                await asyncio.sleep(1.0 / TICK_HZ)
        except WebSocketDisconnect:
            return
    ch = chain()
    n = 0
    try:
        while n * speed / TICK_HZ <= ch.end_s + speed / TICK_HZ:
            await ws.send_json(frame(ch, n, speed))
            n += 1
            await asyncio.sleep(1.0 / TICK_HZ)
        await ws.send_json({"end": True, "t_s": ch.end_s})
        await ws.close()
    except WebSocketDisconnect:
        return


# --- operator authority, live runs only (design agentic.md §0b; arch §23, §29.3) --------------------------------------
# Directives are written to the runtime's directives_inbox/; ariadne_live takes them at its next replan barrier, stamps
# them and records them (directives.jsonl), so a replay applies the same ones. Approvals are written where the runtime
# verifies them itself (approvals/<id>.json). LLM proposals wait in directives_proposed/ until a person confirms.
import json as _json
import time as _time
import uuid as _uuid

from fastapi import Body

_DIRECTIVE_KINDS = {"priority_region", "keep_out", "assign", "explore", "goto", "hold", "release", "cancel"}


def _live_src():
    lv = live(poll=False)
    if lv is None or lv.src() is None:
        raise HTTPException(409, "no live run")
    return lv.src()


def _inbox(src, d: dict, author: str) -> dict:
    if d.get("kind") not in _DIRECTIVE_KINDS:
        raise HTTPException(400, f"kind must be one of {sorted(_DIRECTIVE_KINDS)}")
    d = dict(d, author=author, wall_unix_s=round(_time.time(), 3))
    d.setdefault("id", f"op-{_uuid.uuid4().hex[:8]}")
    box = src / "directives_inbox"
    box.mkdir(exist_ok=True)
    tmp = box / f".{d['id']}.tmp"
    tmp.write_text(_json.dumps(d))
    tmp.rename(box / f"{int(_time.time() * 1e9)}-{d['id']}.json")   # atomic: the runtime never reads half a file
    return d


@app.post("/api/directive")
def post_directive(d: dict = Body(...)) -> JSONResponse:
    return JSONResponse(_inbox(_live_src(), d, "operator"))


@app.get("/api/directives/proposed")
def proposed() -> JSONResponse:
    src = _live_src()
    return JSONResponse([_json.loads(p.read_text()) for p in sorted((src / "directives_proposed").glob("*.json"))]
                        if (src / "directives_proposed").exists() else [])


@app.post("/api/directives/proposed/{pid}/{verdict}")
def confirm(pid: str, verdict: str, operator: str = "operator") -> JSONResponse:
    src = _live_src()
    p = src / "directives_proposed" / f"{pid}.json"
    if not p.exists() or verdict not in ("confirm", "reject"):
        raise HTTPException(404, "no such proposal, or verdict not confirm|reject")
    d = _json.loads(p.read_text())
    p.rename(p.with_suffix(f".{verdict}ed"))
    if verdict == "reject":
        return JSONResponse({"rejected": pid})
    return JSONResponse(_inbox(src, {k: v for k, v in d.items() if k not in ("author", "wall_unix_s")},
                               f"llm-proposed/confirmed-by:{operator}"))


@app.get("/api/approvals")
def approvals() -> JSONResponse:
    a = _live_src() / "approvals"
    out = []
    for r in sorted(a.glob("*.request.json")) if a.exists() else []:
        aid = r.name[: -len(".request.json")]
        if not (a / f"{aid}.json").exists():
            out.append(_json.loads(r.read_text()))
    return JSONResponse(out)


@app.post("/api/approval/{aid}/{verdict}")
def approve(aid: str, verdict: str, operator: str = "operator") -> JSONResponse:
    a = _live_src() / "approvals"
    if verdict not in ("approved", "rejected") or not (a / f"{aid}.request.json").exists():
        raise HTTPException(404, "no such pending request, or verdict not approved|rejected")
    (a / f"{aid}.json").write_text(_json.dumps({"verdict": verdict, "operator": operator,
                                                "wall_unix_s": round(_time.time(), 3)}))
    return JSONResponse({"approval_id": aid, "verdict": verdict})


@app.get("/api/directives")
def recorded() -> JSONResponse:
    """The directives the runtime took, as it recorded them (stamped effective_ns; what replay re-applies), and the
    ones it refused at intake (parse errors), so the operator sees why a directive did nothing."""
    src = _live_src()
    rd = lambda n: [_json.loads(l) for l in (src / n).read_text().splitlines() if l.strip()] if (src / n).exists() else []
    return JSONResponse({"recorded": rd("directives.jsonl"), "rejected": rd("directives_rejected.jsonl")})
