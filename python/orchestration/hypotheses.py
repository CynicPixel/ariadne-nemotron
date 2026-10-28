"""Hypotheses for the evidence-seeking loop (D315, D317): a typed store, seeded deterministically from the planner's
candidates, re-anchored every replan, with status set ONLY through verifier.py from the observed slab.

Per decision request, before the model's turn (HypothesisLayer.before_turn):
  1. check the request's slab is this replan's (verifier.check_slab, D314) and load it;
  2. re-anchor every live hypothesis: the candidates (any agent) whose centroid_xy_m lies within match_radius_m of the
     hypothesis centroid. Cluster ids and indices renumber every replan, so geometry is the key, never the id;
  3. verify every live hypothesis on the slab (one verifier.jsonl line each) and apply verifier.next_status;
  4. seed: each agent's top seed_top_k ranked candidates (supervisor returns excluded) that no hypothesis covers yet
     become new hypotheses, verified at once on the same slab;
  5. write decisions/<did>.hypotheses.json, the view list_hypotheses serves for this decision.
After the turn (after_turn), an assignment naming a hypothesis_id is logged as a `targeted` event with its
evidence_goal. Nothing here reads model text as evidence or touches simulator truth: the inputs are the request
(candidates from the deterministic planner) and the slab the runtime observed.

Run-dir traces (append-only, joinable with trace.jsonl on decision_id / t_s):
  hypotheses.jsonl  one line per event: created | reanchored | status | contradiction | targeted (schema: design §3.1)
  verifier.jsonl    one line per hypothesis per replan: the verdict, its reason and metrics, and the slab it read
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

from . import verifier as V

CLAIM = "the frontier at ({x:.2f}, {y:.2f}) opens onto a passage a responder can use ({rho:.2f} m clearance)"
MAX_LISTED = 12   # hypotheses shown to the model per decision (open first): bounds the prompt


def _utc() -> str:
    t = time.time()
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t)) + f".{int(t % 1 * 1000):03d}Z"


class HypothesisLayer:
    def __init__(self, ddir: Path, cfg: dict):
        self.ddir = Path(ddir)
        self.run = self.ddir.parent
        self.top_k = int(cfg.get("seed_top_k", 2))
        self.match_m = float(cfg.get("match_radius_m", 1.0))
        self.radius_m = float(cfg.get("region_radius_m", 1.5))
        self.rho_m = float(cfg.get("rho_responder_m", 0.40))
        self.expire = int(cfg.get("expire_after_replans", 3))
        self.state_path = self.ddir / "hypotheses_state.json"
        self.h: dict[str, dict] = json.loads(self.state_path.read_text()) if self.state_path.exists() else {}

    def _rel(self, p: str | None) -> str | None:
        """A path under the run dir as run-relative (the console serves the run dir; container prefixes differ)."""
        if not p:
            return p
        try:
            return str(Path(p).relative_to(self.run.resolve()))
        except ValueError:
            return p

    # --- traces ---
    def _event(self, req: dict, h: dict, event: str, **extra) -> None:
        rec = {"t_utc": _utc(), "sim_t_s": req["t_s"], "decision_id": req["decision_id"], "event": event,
               "hypothesis_id": h["hypothesis_id"], "agent_id": h["agent_id"], "anchor": h["anchor"],
               "anchors": h["anchors"], "claim": h["claim"], "evidence_goal": h.get("evidence_goal"),
               "status": h["status"], "status_source": "verifier" if event in ("status", "contradiction") else "none",
               "verifier": h.get("verifier"), "provenance": h["provenance"], **extra}
        with open(self.run / "hypotheses.jsonl", "a") as f:
            f.write(json.dumps(rec) + "\n")

    def _verify(self, req: dict, slab, s: dict, h: dict) -> dict:
        v = V.verify(slab, float(s["res_m"]), s["origin_xy_m"], h["centroid_xy_m"], h["seed_goal_xy_m"],
                     self.radius_m, self.rho_m)
        rec = {"t_utc": _utc(), "sim_t_s": req["t_s"], "decision_id": req["decision_id"],
               "hypothesis_id": h["hypothesis_id"], "status_before": h["status"], **v,
               "slab": {"path": s["path"], "t_s": s["t_s"]}, "rho_m": self.rho_m, "region_radius_m": self.radius_m}
        with open(self.run / "verifier.jsonl", "a") as f:
            f.write(json.dumps(rec) + "\n")
        return v

    def _apply(self, req: dict, h: dict, v: dict | None) -> str | None:
        status, event = V.next_status(h["status"], v["verdict"] if v else None, h["unmatched"], self.expire)
        if v:
            h["verifier"] = {"slab": v.get("_slab"), "slab_t_s": req["t_s"], "verdict": v["verdict"],
                             "reason": v["reason"], "metrics": v["metrics"]}
        if event == "contradiction":
            self._event(req, h, "contradiction", contradicting_verdict=v["verdict"])
        elif event == "status":
            h["status"] = status
            if status in V.TERMINAL:
                h["resolved_t_s"] = req["t_s"]
            self._event(req, h, "status")
        return event

    # --- the per-decision step ---
    def anchors_for(self, req: dict, xy) -> dict:
        out = {}
        for a in req["agents"]:
            best, bi = self.match_m + 1e-9, None
            for i, c in enumerate(a["candidates"]):   # nearest centroid; a tie keeps the better-ranked candidate
                d = math.dist(c["centroid_xy_m"], xy)
                if d < best:
                    best, bi = d, i
            if bi is not None:
                out[str(a["agent"])] = bi
        return out

    def before_turn(self, req: dict) -> dict:
        did = req["decision_id"]
        summary = {"new": [], "changed": [], "slab": None, "error": None}
        slab = s = None
        try:
            s = V.check_slab(req)
            slab = V.load_slab(s)
            summary["slab"] = s["path"]
        except (V.SlabMismatch, OSError) as e:
            summary["error"] = f"{type(e).__name__}: {e}"
        # 2-3: re-anchor and verify what exists
        for h in self.h.values():
            if h["status"] == "expired":
                continue
            anchors = self.anchors_for(req, h["centroid_xy_m"])
            h["unmatched"] = 0 if anchors else h["unmatched"] + 1
            if anchors != h["anchors"]:
                h["anchors"] = anchors
                if anchors:
                    k, i = next(iter(anchors.items()))
                    c = next(a for a in req["agents"] if str(a["agent"]) == k)["candidates"][i]
                    h["anchor"] = {"kind": "candidate", "decision_id": did, "agent": int(k), "candidate": i,
                                   "cluster": c["cluster"], "goal_xy_m": c["goal_xy_m"],
                                   "centroid_xy_m": c["centroid_xy_m"]}
                if h["status"] in V.OPEN:
                    self._event(req, h, "reanchored")
            v = None
            if slab is not None:
                v = self._verify(req, slab, s, h)
                v["_slab"] = s["path"]
            before = h["status"]
            if self._apply(req, h, v) == "status":
                summary["changed"].append(f"{h['hypothesis_id']}:{before}->{h['status']}")
        # 4: seed from the planner's top-ranked candidates
        for a in req["agents"]:
            ranked = [(i, c) for i, c in enumerate(a["candidates"]) if not c.get("supervisor_return")][:self.top_k]
            for i, c in ranked:
                if any(math.dist(h["centroid_xy_m"], c["centroid_xy_m"]) <= self.match_m for h in self.h.values()):
                    continue
                hid = f"h{len(self.h) + 1:03d}"
                x, y = c["centroid_xy_m"]
                h = {"hypothesis_id": hid, "agent_id": a["agent"], "created_t_s": req["t_s"], "created_decision_id": did,
                     "claim": CLAIM.format(x=x, y=y, rho=self.rho_m), "centroid_xy_m": c["centroid_xy_m"],
                     "seed_goal_xy_m": c["goal_xy_m"], "status": "unresolved", "unmatched": 0,
                     "anchor": {"kind": "candidate", "decision_id": did, "agent": a["agent"], "candidate": i,
                                "cluster": c["cluster"], "goal_xy_m": c["goal_xy_m"], "centroid_xy_m": c["centroid_xy_m"]},
                     "anchors": self.anchors_for(req, c["centroid_xy_m"]), "evidence_goal": None, "verifier": None,
                     "provenance": {"source": "seed", "rank": c.get("rank", i),
                                    "keyframe_path": self._rel(a.get("keyframe_png")), "map_png": self._rel(req.get("map_png"))}}
                self.h[hid] = h
                self._event(req, h, "created")
                summary["new"].append(hid)
                if slab is not None:
                    v = self._verify(req, slab, s, h)
                    v["_slab"] = s["path"]
                    self._apply(req, h, v)
        # 5: the view for this decision
        view = sorted((h for h in self.h.values() if h["status"] != "expired"),
                      key=lambda h: (h["status"] not in V.OPEN, -h["created_t_s"]))[:MAX_LISTED]
        (self.ddir / f"{did}.hypotheses.json").write_text(json.dumps({
            "decision_id": did, "t_s": req["t_s"], "rho_responder_m": self.rho_m,
            "slab": summary["slab"], "verifier_error": summary["error"],
            "hypotheses": [{"hypothesis_id": h["hypothesis_id"], "claim": h["claim"], "status": h["status"],
                            "reason": (h.get("verifier") or {}).get("reason"),
                            "anchors": h["anchors"], "centroid_xy_m": h["centroid_xy_m"],
                            "seeded_by_agent": h["agent_id"], "created_t_s": h["created_t_s"],
                            "known_frac": ((h.get("verifier") or {}).get("metrics") or {}).get("known_frac")}
                           for h in view]}))
        tmp = self.state_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.h))
        tmp.replace(self.state_path)
        counts = {}
        for h in self.h.values():
            counts[h["status"]] = counts.get(h["status"], 0) + 1
        return {**summary, "counts": counts}

    def after_turn(self, req: dict, assignments) -> None:
        for e in assignments or []:
            h = self.h.get(e.get("hypothesis_id") or "")
            if h is None:
                continue
            h["evidence_goal"] = e.get("evidence_goal")
            self._event(req, h, "targeted", targeted_by_agent=e["agent"], candidate=e["candidate"],
                        hypothesis_anchored=e.get("hypothesis_anchored"))
        tmp = self.state_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.h))
        tmp.replace(self.state_path)
