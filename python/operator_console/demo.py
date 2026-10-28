"""Single-worker demo control plane. Never imports ROS or runs client-supplied commands.

Run on the prewarmed worker with ARIADNE_DEMO=1, one uvicorn worker. The existing
live.sh owns containers; this module owns its session, controller lease and artifacts.
"""
from __future__ import annotations

import asyncio
import hashlib
import fcntl
import json
import math
import os
import re
import secrets
import signal
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

from fastapi import APIRouter, Body, Header, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response

from .live import LiveRun
from .world_mesh import merged_obj

ROOT = Path(__file__).resolve().parents[2]
ID = re.compile(r"^[A-Za-z0-9_-]{1,100}$")
ACTIVE = {"starting", "running", "stopping", "recovering"}
PRESETS = ("maze_demo", "maze_short", "ci_tiny")
router = APIRouter(prefix="/api/demo")
VIDEO_URL = None  # set ARIADNE_DEMO_VIDEO_URL to show the film link


def atomic(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    tmp.write_text(json.dumps(value, allow_nan=False))
    tmp.replace(path)


def identifier(value: str):
    if not ID.fullmatch(value):
        raise HTTPException(400, "Invalid identifier")
    return value


def finite_xy(value):
    if not isinstance(value, list) or len(value) != 2 or any(
        type(x) not in (int, float) or not math.isfinite(x) for x in value
    ):
        raise HTTPException(422, "Expected two finite world coordinates")
    return value


def decision_summary(record):
    """Bound the network payload; original evidence stays in trace.jsonl."""
    out = {k: record[k] for k in ("t_s", "agent", "result", "executed", "chooser", "vision", "command") if k in record}
    choice = record.get("choice", {})
    out["choice"] = {k: choice[k] for k in ("source", "decision_id", "validation") if k in choice}
    answer = choice.get("answer")
    if isinstance(answer, dict):
        assignments = answer.get("assignments") or []
        selected = next((a for a in assignments if a.get("agent") == record.get("agent")), {})
        out["choice"]["reason"] = str(selected.get("reason", ""))[:1000]
        log = answer.get("orchestrator", {})
        out["orchestrator"] = {k: log[k] for k in ("model", "latency_s", "calls", "input_tokens", "output_tokens", "fallback_used", "errors") if k in log}
        out["orchestrator"]["tool_calls"] = log.get("tool_calls", [])[-8:]
    return out


class Demo:
    def __init__(self, root=ROOT):
        self.root = Path(root)
        self.base = self.root / "build/demo"
        self.lock = threading.RLock()
        self.state = "idle"
        self.error = None
        self.scene = None
        self.scene_spec = None
        self.run = None
        self.session = None
        self.lv = None
        self.proc = None
        self.token = None
        self.heartbeat = 0.0
        self.lease_held = False
        self.seq = 0
        self.snapshot = {}
        self.commands = {}
        self.stop_event = threading.Event()
        self.thread = None
        self.enabled = os.environ.get("ARIADNE_DEMO") == "1"
        self.max_agents = max(1, min(3, int(os.environ.get("ARIADNE_DEMO_MAX_AGENTS", "2"))))
        self.recon_state = "idle"
        self.replay = None
        self.last_record = None
        self.scene_object = None
        self.world = None
        self.process_started = 0.0
        self.process_finished = False
        self.meta_cache = None
        self.progress_wall = time.time()
        self.latest_mission_time = None
        self.recon_proc = None
        self.worker_lock = None
        self.recovering = []

    def start(self):
        if self.thread:
            return
        if self.enabled:
            self.base.mkdir(parents=True, exist_ok=True)
            self.worker_lock = open(self.base / "worker.lock", "a")
            try:
                fcntl.flock(self.worker_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                self.worker_lock.close()
                raise RuntimeError("A demo controller already owns this worker; use one server process")
            # Do not adopt an unknown running process after a server restart. Request
            # recorded shutdown for any unfinished run before accepting another one.
            for manifest in (self.base / "runs").glob("*/session.json"):
                run = manifest.parent
                if (run / "finished.json").exists():
                    continue
                atomic(run / "stop.request.json", {"reason": "controller-restarted", "requested_at": time.time()})
                self.recovering.append(run)
            if self.recovering:
                self.state = "recovering"
                self.error = "Waiting for the previous worker to stop after controller restart"
        self.thread = threading.Thread(target=self.loop, daemon=True, name="demo-telemetry")
        self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
        # A server shutdown must not leave an unattended mission moving.
        if self.state in ACTIVE:
            self.hold_all("server-shutdown")
            self.request_stop()
        if self.worker_lock:
            self.worker_lock.close()

    def loop(self):
        while not self.stop_event.wait(.1):
            try:
                with self.lock:
                    self.tick()
            except Exception as exc:
                # Surface failed tailing instead of killing the only event producer.
                self.error = f"Telemetry: {type(exc).__name__}: {exc}"

    def authorize(self, token):
        if not self.enabled:
            raise HTTPException(503, "Live worker disabled; start with ARIADNE_DEMO=1")
        if not token or not self.token or not secrets.compare_digest(token, self.token):
            raise HTTPException(403, "This action requires the controller lease")
        self.heartbeat = time.monotonic()
        self.lease_held = True

    def claim(self, token=None):
        with self.lock:
            if not self.enabled:
                raise HTTPException(503, "Live worker disabled")
            if self.token and time.monotonic() - self.heartbeat < 30:
                self.authorize(token)
            else:
                self.token = secrets.token_urlsafe(32)
            self.heartbeat = time.monotonic()
            self.lease_held = True
            return {"token": self.token}

    def status(self):
        with self.lock:
            meta = self.meta_cache
            return {"state": self.state, "error": self.error, "session_id": self.session,
                    "enabled": self.enabled, "max_agents": self.max_agents, "presets": PRESETS,
                    "scene": self.scene_spec, "meta": meta, "seq": self.seq,
                    "controller_remaining_s": max(0, 30 - (time.monotonic() - self.heartbeat)) if self.token else 0,
                    "reconstruction": self.recon_state,
                    "links": {"github": "https://github.com/CynicPixel/ariadne-nemotron",
                              "paper": None,
                              "video": os.environ.get("ARIADNE_DEMO_VIDEO_URL", VIDEO_URL)},
                    "frame": self.snapshot, "commands": list(self.commands.values())[-20:]}

    def generate(self, spec):
        with self.lock:
            if self.state in ACTIVE or self.state == "generating" or self.recon_state == "training":
                raise HTTPException(409, "Worker busy")
            preset, seed = spec.get("preset", "maze_demo"), spec.get("seed", 6)
            if preset not in PRESETS or type(seed) is not int or not 0 <= seed <= 2**31 - 1:
                raise HTTPException(422, "Choose an allowed preset and a seed in 0..2147483647")
            self.state, self.error = "generating", None
            self.scene = None
            self.scene_spec = {"preset": preset, "seed": seed}
            self.snapshot, self.commands, self.world = {}, {}, None
            self.lv, self.run, self.session, self.replay = None, None, None, None
            threading.Thread(target=self._generate, args=(preset, seed), daemon=True).start()

    def _generate(self, preset, seed):
        try:
            from dataset_tools.worldgen.params import load_preset
            from dataset_tools.worldgen.scene import generate
            from dataset_tools.worldgen.build import write_scene
            from dataset_tools.worldgen.mission import write_mission
            from dataset_tools.worldgen.png import write_png
            import numpy as np
            p = load_preset(str(self.root / f"configs/worldgen/{preset}.yaml"))
            scene = generate(p, seed)
            # Every preview owns its mission files; cached geometry is never mutated across sessions.
            dest = self.base / "scenes" / uuid.uuid4().hex
            dest.mkdir(parents=True)
            write_scene(scene, str(dest))
            write_mission(scene, str(dest), str(self.root / "configs/sim/rig_stereo_v1.yaml"),
                          name="live", live=True, rtf=1, agents=list(range(min(2, len(scene.scripts)))))
            # Setup geometry only: no scripted trajectories or oracle egress route.
            image = np.zeros((*scene.free.shape, 3), np.uint8)
            image[:] = (17, 27, 35)
            image[scene.free] = (56, 78, 89)
            image[scene.trav_responder] = (95, 122, 128)
            write_png(str(dest / "preview.png"), image[::-1])
            world = merged_obj(str(dest), include_ceiling=False)
            from dataset_tools.worldgen import GENERATOR_VERSION
            manifest = {"preset": preset, "seed": seed, "preset_digest": p.digest(),
                        "generator_version": GENERATOR_VERSION, "scene_id": scene.scene_id, "preview_id": dest.name}
            atomic(dest / "demo_scene.json", manifest)
            with self.lock:
                self.scene, self.scene_object, self.world = dest, scene, world
                self.scene_spec, self.state = manifest, "ready"
                self.meta_cache = LiveRun(self.base, dest).meta()
        except Exception as exc:
            with self.lock:
                self.state, self.error = "failed", f"Generation failed: {exc}"

    def validate(self, spec):
        if not self.scene:
            raise HTTPException(409, "Generate a scene first")
        sc = json.loads((self.scene / "scene.json").read_text())
        agents = spec.get("entrances", [0, 1][:self.max_agents])
        if (not isinstance(agents, list) or not 1 <= len(agents) <= self.max_agents
                or any(type(a) is not int or not 0 <= a < len(sc["entrances"]) for a in agents)
                or agents != sorted(set(agents))):
            raise HTTPException(422, "Select distinct entrances within the qualified agent limit")
        target = finite_xy(spec.get("target", sc["target_xy_m"]))
        grid = sc["esdf"]
        ox, oy = grid["origin_m"]
        if not (ox <= target[0] < ox + grid["cols"] * grid["cell_m"] and
                oy <= target[1] < oy + grid["rows"] * grid["cell_m"]):
            raise HTTPException(422, "Target is outside the map")
        from dataset_tools.worldgen.mission import check_demo_target, MissionError
        try:
            check_demo_target(sc, str(self.scene), target,
                              [(a, sc["entrances"][a]["start_xy_m"]) for a in agents])
        except MissionError as exc:
            raise HTTPException(422, str(exc)) from exc
        return {"entrances": agents, "target": target, "valid": True}

    def launch(self, spec, key):
        with self.lock:
            identifier(key)
            if self.session and self.commands.get("start", {}).get("id") == key:
                return self.status()
            if self.state not in {"ready", "completed", "failed"} or self.recon_state == "training":
                raise HTTPException(409, "Worker is not ready")
            chosen = self.validate(spec)
            duration = spec.get("duration_s", 120)
            if type(duration) is not int or not 10 <= duration <= 300:
                raise HTTPException(422, "Duration must be 10..300 simulation seconds")
            from dataset_tools.worldgen.mission import write_mission
            write_mission(self.scene_object, str(self.scene), str(self.root / "configs/sim/rig_stereo_v1.yaml"),
                          name="live", live=True, rtf=1, agents=chosen["entrances"], target_xy=chosen["target"])
            self.session = uuid.uuid4().hex
            self.run = self.base / "runs" / self.session
            self.run.mkdir(parents=True)
            self.commands = {"start": {"id": key, "state": "queued"}}
            self.snapshot, self.replay, self.lv = {}, None, None
            self.last_record = None
            self.meta_cache = LiveRun(self.run, self.scene).meta()
            self.state, self.error, self.recon_state = "starting", None, "waiting"
            env = os.environ.copy()
            env.pop("ARIADNE_LOCKSTEP", None)
            env.update(ARIADNE_EXTERNAL_CONSOLE="1", ARIADNE_SPLAT="1",
                       ARIADNE_PLANNER_CONFIG="configs/mission/mission_v2.yaml",
                       ARIADNE_LIVE_CONFIG="configs/mission/live_web_demo_v1.yaml")
            manifest = dict(self.scene_spec, **chosen, duration_s=duration, session_id=self.session,
                            live_config=env["ARIADNE_LIVE_CONFIG"], mode="live-demo", scene_dir=str(self.scene.relative_to(self.root)))
            try:
                manifest["git_commit"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=self.root, timeout=5).decode().strip()
            except (OSError, subprocess.SubprocessError):
                manifest["git_commit"] = "unknown"
            manifest["live_config_sha256"] = hashlib.sha256((self.root / env["ARIADNE_LIVE_CONFIG"]).read_bytes()).hexdigest()
            atomic(self.run / "session.json", manifest)
            self.proc = None
            try:
                with open(self.run / "launch.log", "wb") as log:
                    self.proc = subprocess.Popen(["bash", "infra/scripts/live.sh", "run",
                        str(self.scene.relative_to(self.root)), str(self.run.relative_to(self.root)), str(duration)],
                        cwd=self.root, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as exc:
                self.state, self.error = "failed", f"Launch failed: {exc}"
            if self.proc:
                atomic(self.run / "launcher.json", {"pid": self.proc.pid})
            self.process_started, self.process_finished = time.monotonic(), False
            return self.status()

    def src(self):
        return self.lv.src() if self.lv else None

    def directive(self, doc, key):
        identifier(key)
        if key in self.commands:
            return self.commands[key]
        if self.state not in {"running", "stopping"} or not self.src():
            raise HTTPException(409, "Mission is not accepting commands")
        kind = doc.get("kind")
        if kind not in {"hold", "release", "priority_region", "keep_out", "assign", "cancel"}:
            raise HTTPException(422, "Unsupported map action")
        clean = {"id": key, "kind": kind, "author": "operator", "wall_unix_s": time.time()}
        n = len(json.loads((self.scene / "mission_live.json").read_text())["agents"])
        if kind in {"hold", "release", "assign"}:
            a = doc.get("agent")
            if type(a) is not int or not 0 <= a < n:
                raise HTTPException(422, "Unknown agent")
            clean["agent"] = a
        if kind in {"priority_region", "keep_out", "assign"}:
            polygon = doc.get("polygon")
            if not isinstance(polygon, list) or not 3 <= len(polygon) <= 32:
                raise HTTPException(422, "Draw a region with 3..32 vertices")
            grid = self.lv.meta()["grid"]
            for p in polygon:
                x, y = finite_xy(p)
                ox, oy = grid["origin_m"]
                if not ox <= x <= ox + grid["cols"] * grid["cell_m"] or not oy <= y <= oy + grid["rows"] * grid["cell_m"]:
                    raise HTTPException(422, "Region is outside the map")
            clean["polygon"] = polygon
            if kind == "priority_region":
                clean["weight"] = 2.0
        if kind == "cancel":
            clean["target"] = identifier(doc.get("target", ""))
        atomic(self.src() / "directives_inbox" / f"{time.time_ns()}-{key}.json", clean)
        self.commands[key] = dict(clean, state="queued")
        return self.commands[key]

    def hold_all(self, reason):
        if self.src():
            n = len(json.loads((self.scene / "mission_live.json").read_text())["agents"])
            for a in range(n):
                self.directive({"kind": "hold", "agent": a}, f"{reason}-{self.seq}-{a}")

    def request_stop(self):
        if self.state not in ACTIVE:
            return
        self.hold_all("end-run")
        self.state = "stopping"
        if self.run:
            # live.sh sees this even before runtime initialization finishes.
            atomic(self.run / "stop.request.json", {"requested_at": time.time()})
        if self.src():
            atomic(self.src() / "stop.request.json", {"requested_at": time.time()})

    def tick(self):
        if self.recovering:
            waiting = []
            for run in self.recovering:
                marker = run / "launcher.json"
                pid = json.loads(marker.read_text()).get("pid") if marker.exists() else None
                try:
                    cmd = Path(f"/proc/{pid}/cmdline").read_bytes() if pid else b""
                except OSError:
                    cmd = b""
                if b"infra/scripts/live.sh" in cmd and str(run.relative_to(self.root)).encode() in cmd:
                    waiting.append(run)
                else:
                    atomic(run / "finished.json", {"state": "interrupted", "reason": "controller restarted"})
            self.recovering = waiting
            if not waiting:
                self.state, self.error = "idle", None
        if self.token and self.lease_held and time.monotonic() - self.heartbeat >= 30:
            if self.state == "running":
                self.hold_all("controller-lost")
            self.lease_held = False
        if self.run and not self.lv:
            candidates = list(self.run.glob("live-*"))
            if candidates:
                self.lv = LiveRun(candidates[0], self.scene)
        if self.lv and self.replay is None:
            self.lv.poll()
            if self.lv.poses and self.state == "starting":
                self.state = "running"
                self.commands["start"]["state"] = "effective"
            self.seq += 1
            frame = self.lv.frame(self.seq, -1, self.lv.t_s - 3)
            frame["events"] = frame["events"][-40:]
            frame["decisions"] = [decision_summary(r) for r in getattr(self.lv, "decisions", [])[-20:]]
            if frame["t_s"] != self.latest_mission_time:
                self.progress_wall = time.time()
                self.latest_mission_time = frame["t_s"]
            frame["wall_unix_s"] = self.progress_wall
            frame["reconstruction"] = self.table()
            frame["session_id"] = self.session
            frame["map_url"] = f"/api/demo/map/{frame['map_version']}?session={self.session}" if frame["map_version"] is not None else None
            # Keep ground truth out of the operator data channel.
            for a in frame["agents"]:
                a.pop("gt", None)
            src = self.src()
            if src:
                for name, state in (("directives.jsonl", "effective"), ("directives_rejected.jsonl", "rejected")):
                    p = src / name
                    if p.exists():
                        for line in p.read_text().splitlines():
                            try:
                                d = json.loads(line)
                            except ValueError:
                                continue
                            for key, command in self.commands.items():
                                if d.get("id") == key or d.get("file", "").endswith(f"-{key}.json"):
                                    command.update(state=state, detail=d)
                frame["cameras"] = []
                for p in sorted((src / "snapshots").glob("cam_*.json")):
                    try:
                        d = json.loads(p.read_text())
                        if "image" in d:
                            d["url"] = f"/api/demo/media/{d['image']}?session={self.session}"
                            frame["cameras"].append(d)
                    except (ValueError, OSError):
                        pass
            frame["approvals"] = self.approvals()
            frame["mode"] = "live"
            self.snapshot = frame
            if self.state in ACTIVE and frame["t_s"] != self.last_record:
                with open(self.run / "playback.jsonl", "a") as f:
                    f.write(json.dumps(frame) + "\n")
                self.last_record = frame["t_s"]
        if self.proc and self.proc.poll() is not None and not self.process_finished:
            self.process_finished = True
            self.state = "completed" if self.proc.returncode == 0 else "failed"
            atomic(self.run / "finished.json", {"exit_code": self.proc.returncode, "state": self.state})
            if self.proc.returncode:
                self.error = "Simulation worker failed; inspect this session's launch.log"
            self.recon_state = "points available" if self.table().get("submaps") else "unavailable"
        if self.state == "starting" and self.proc and time.monotonic() - self.process_started > 360:
            self.request_stop()
            os.killpg(self.proc.pid, signal.SIGTERM)
            self.error = "Worker startup exceeded six minutes; stopped its launcher"
        if self.state == "stopping" and self.src():
            p = self.src() / "stop.request.json"
            if not p.exists():
                atomic(p, {"requested_at": time.time()})

    def approvals(self):
        src = self.src()
        if not src:
            return []
        out = []
        supervisor = src / "supervisor.json"
        pending = json.loads(supervisor.read_text()).get("decision_id") if supervisor.exists() else None
        for p in (src / "approvals").glob("*.request.json"):
            if p.with_name(p.name.replace(".request.json", ".json")).exists():
                continue
            try:
                d = json.loads(p.read_text())
                if pending is not None and d.get("decision_id") != pending:
                    continue
                if d.get("deadline_wall_s", d.get("deadline_unix_s", d.get("requested_unix_s", 0) + 60)) > time.time():
                    out.append(d)
            except (ValueError, OSError):
                pass
        return out

    def verdict(self, aid, verdict):
        identifier(aid)
        if verdict not in {"approved", "rejected"}:
            raise HTTPException(422, "Choose approve or reject")
        matches = [a for a in self.approvals() if a["approval_id"] == aid]
        if not matches or self.state != "running":
            raise HTTPException(409, "This request expired or was already answered")
        # The lock serializes verdicts. Atomic rename prevents partial JSON reaching C++.
        atomic(self.src() / "approvals" / f"{aid}.json",
               {"verdict": verdict, "operator": "controller", "wall_unix_s": time.time(),
                "session_id": self.session, "decision_id": matches[0].get("decision_id")})
        return {"id": aid, "state": "submitted", "verdict": verdict}

    def train(self):
        if self.state != "completed" or not self.lv or self.recon_state == "training":
            raise HTTPException(409, "Finish the run before training")
        recorded = self.lv.run / "recon.link"
        if not recorded.is_file():
            raise HTTPException(409, "This run has no recorded reconstruction stream")
        self.recon_state = "training"
        threading.Thread(target=self._train, args=(self.lv.run,), daemon=True).start()

    def _train(self, run):
        image = os.environ.get("ARIADNE_RECON_GPU_IMAGE", "ariadne/recon-gpu:build")
        env = os.environ.copy()
        try:
            # Only the explicitly provisioned local image: no implicit pulls or cloud provisioning.
            subprocess.run(["docker", "image", "inspect", image], check=True, capture_output=True, timeout=10)
            relative = str(run.relative_to(self.root))
            name = f"ariadne-demo-train-{self.session}"
            with open(run / "training.log", "wb") as log:
                try:
                    subprocess.run(["docker", "run", "--rm", "--name", name, "--gpus", "all", "--network", "none",
                        "-u", f"{os.getuid()}:{os.getgid()}", "-e", "HOME=/tmp", "-e", "PYTHONPATH=/work/python:/work/python/reconstruction",
                        "-v", f"{self.root}:/work", "-w", "/work", image,
                        "python", "-m", "ariadne_recon.live.service", "--recorded-link", f"{relative}/recon.link",
                        "--out", f"{relative}/trained", "--scene", str(self.scene.relative_to(self.root)),
                        "--backend", "gpu", "--training-image", "rgb", "--budget-ms", "100"],
                        env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=1800)
                except subprocess.TimeoutExpired:
                    subprocess.run(["docker", "stop", "-t", "10", name], capture_output=True, timeout=20)
                    raise
            with self.lock:
                self.recon_state = "trained"
        except (OSError, subprocess.SubprocessError) as exc:
            with self.lock:
                self.recon_state = "points available"
                self.error = f"Post-run training unavailable ({type(exc).__name__}); point cloud retained. Check GPU image and training.log."

    def table(self):
        if not self.lv:
            return {"version": 0, "submaps": [], "label": "Waiting for observed geometry"}
        p = self.lv.run / "splat/table.json"
        trained = self.lv.run / "trained/table.json"
        if self.recon_state == "trained" and trained.exists():
            p = trained
        return json.loads(p.read_text()) if p.exists() else {"version": 0, "submaps": []}


manager = Demo()


def check_session(session):
    if session is not None and session != manager.session:
        raise HTTPException(409, "Artifact belongs to another session; refresh the snapshot")


@router.get("/status")
def status():
    return manager.status()


@router.post("/lease")
def lease(x_controller_token: str | None = Header(None)):
    return manager.claim(x_controller_token)


@router.post("/heartbeat")
def heartbeat(x_controller_token: str | None = Header(None)):
    with manager.lock:
        manager.authorize(x_controller_token)
        return {"ok": True}


@router.post("/{action}")
def control(action: str, doc: dict = Body(default={}), x_controller_token: str | None = Header(None),
            idempotency_key: str | None = Header(None)):
    with manager.lock:
        manager.authorize(x_controller_token)
        if action == "generate":
            manager.generate(doc)
        elif action == "validate":
            return manager.validate(doc)
        elif action == "start":
            if not idempotency_key:
                raise HTTPException(400, "Idempotency-Key required")
            return manager.launch(doc, idempotency_key)
        elif action == "directive":
            if not idempotency_key:
                raise HTTPException(400, "Idempotency-Key required")
            return manager.directive(doc, idempotency_key)
        elif action == "stop":
            manager.request_stop()
        elif action == "hold":
            manager.hold_all("operator-hold")
        elif action == "train":
            manager.train()
        elif action == "approval":
            return manager.verdict(doc.get("id", ""), doc.get("verdict"))
        else:
            raise HTTPException(404, "Unknown action")
        return manager.status()


@router.get("/world.obj")
def world():
    if manager.world is None:
        raise HTTPException(404, "No scene yet")
    return Response(manager.world, media_type="text/plain")


@router.get("/preview.png")
def preview():
    if not manager.scene:
        raise HTTPException(404, "No scene yet")
    return FileResponse(manager.scene / "preview.png")


@router.get("/entrances")
def entrances():
    if not manager.scene:
        return []
    return json.loads((manager.scene / "scene.json").read_text())["entrances"]


@router.get("/map/{version}")
def map_image(version: int, session: str | None = None):
    check_session(session)
    if not manager.lv or not 0 <= version < len(manager.lv.snapshots):
        raise HTTPException(404, "Map unavailable")
    p = manager.lv.snapshots[version][1]
    if not p.is_file():
        raise HTTPException(404, "Map snapshot was not exported")
    return FileResponse(p)


@router.get("/media/{name}")
def media(name: str, session: str | None = None):
    check_session(session)
    if not re.fullmatch(r"cam_\d+_\d+\.jpg", name) or not manager.src():
        raise HTTPException(404, "Camera frame unavailable")
    p = manager.src() / "snapshots" / name
    if not p.is_file():
        raise HTTPException(404, "Camera frame unavailable")
    return FileResponse(p, headers={"Cache-Control": "private, max-age=31536000, immutable"})


@router.get("/cameras")
def cameras(agent: int, session: str):
    check_session(session)
    return next((c for c in manager.snapshot.get("cameras", []) if c["agent"] == agent), None)


@router.get("/table")
def table():
    return manager.table()


@router.get("/chunk/{name}")
def chunk(name: str, backend: str = "cpu_init", session: str | None = None):
    check_session(session)
    if not re.fullmatch(r"\d+_\d+\.splat", name) or not manager.lv:
        raise HTTPException(404, "Chunk unavailable")
    directory = "trained" if backend == "gpu" else "splat"
    p = manager.lv.run / directory / "chunks" / name
    if not p.is_file():
        raise HTTPException(404, "Chunk replaced; refresh table")
    return FileResponse(p, media_type="application/octet-stream", headers={"Cache-Control": "private, max-age=31536000, immutable"})


@router.get("/playback")
def playback():
    if not manager.run or manager.state in ACTIVE:
        raise HTTPException(409, "Playback becomes available when the run ends")
    p = manager.run / "playback.jsonl"
    if not p.is_file():
        raise HTTPException(404, "No recorded playback")
    return FileResponse(p, media_type="application/x-ndjson")


@router.websocket("/stream")
async def stream(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            # Single producer, latest snapshot: slow spectators never stall the worker.
            await asyncio.wait_for(ws.send_json(manager.status()), timeout=5)
            await asyncio.sleep(.2)
    except (WebSocketDisconnect, RuntimeError, TimeoutError):
        return
