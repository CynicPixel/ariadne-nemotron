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
    def __init__(self, ddir: Path, cfg: dict, vl=None):
        self.ddir = Path(ddir)
        self.run = self.ddir.parent
        self.top_k = int(cfg.get("seed_top_k", 2))
        self.match_m = float(cfg.get("match_radius_m", 1.0))
        self.radius_m = float(cfg.get("region_radius_m", 1.5))
        self.rho_m = float(cfg.get("rho_responder_m", 0.40))
        self.expire = int(cfg.get("expire_after_replans", 3))
        self.vl = vl   # VLProposer or None (vl_proposer off: deterministic seeding only)
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
            # a settled status that the current map disagrees with (G2: observed-free cells turning occupied under
            # map revisions). Logged once per change of the contradicting verdict, not every replan; surfaced in the
            # view (`contradicted`) so the model and the console see it. The status itself stays (first resolution).
            h["contradiction_replans"] = h.get("contradiction_replans", 0) + 1
            if (h.get("contradicted") or {}).get("verdict") != v["verdict"]:
                h["contradicted"] = {"since_t_s": req["t_s"], "verdict": v["verdict"], "reason": v["reason"]}
                self._event(req, h, "contradiction", contradicting_verdict=v["verdict"])
        elif h["status"] in ("confirmed", "refuted") and h.get("contradicted") and v and v["verdict"] == h["status"]:
            h["contradicted"] = None   # the map agrees again
            self._event(req, h, "contradiction_cleared")
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

    def _covered(self, xy) -> bool:
        return any(math.dist(h["centroid_xy_m"], xy) <= self.match_m for h in self.h.values())

    def _create(self, req: dict, agent: int, i: int, c: dict, slab, s, summary: dict, provenance: dict,
                orchestrator: dict | None = None, predicts: str | None = None) -> dict:
        did = req["decision_id"]
        hid = f"h{len(self.h) + 1:03d}"
        x, y = c["centroid_xy_m"]
        h = {"hypothesis_id": hid, "agent_id": agent, "created_t_s": req["t_s"], "created_decision_id": did,
             "claim": CLAIM.format(x=x, y=y, rho=self.rho_m), "centroid_xy_m": c["centroid_xy_m"],
             "seed_goal_xy_m": c["goal_xy_m"], "status": "unresolved", "unmatched": 0, "predicts": predicts,
             "anchor": {"kind": "candidate", "decision_id": did, "agent": agent, "candidate": i,
                        "cluster": c["cluster"], "goal_xy_m": c["goal_xy_m"], "centroid_xy_m": c["centroid_xy_m"]},
             "anchors": self.anchors_for(req, c["centroid_xy_m"]), "evidence_goal": None, "verifier": None,
             "provenance": provenance}
        if orchestrator:
            h["orchestrator"] = orchestrator
        self.h[hid] = h
        self._event(req, h, "created")
        summary["new"].append(hid)
        if slab is not None:
            v = self._verify(req, slab, s, h)
            v["_slab"] = s["path"]
            self._apply(req, h, v)
        return h

    def _fold_vl(self, req: dict, res: dict, slab, s, summary: dict) -> int:
        """One finished VL call -> hypotheses (source vl), or a VL prediction attached to an open hypothesis that already
        covers the frontier (event vl_annotated). Every call is logged as a `vl_call` event, whatever its outcome (FM:
        the vl/seed share must be measurable). Returns how many hypotheses it created or annotated."""
        seen = res["request"]
        made, skipped = [], []
        for p in res.get("proposals", []):
            a = next(x for x in seen["agents"] if x["agent"] == p["agent"])
            c = a["candidates"][p["candidate"]]
            cover = next((h for h in self.h.values()
                          if math.dist(h["centroid_xy_m"], c["centroid_xy_m"]) <= self.match_m), None)
            if cover is not None:
                # the VL predicts about a frontier a hypothesis already covers (typically the replan-0 seed): its
                # prediction is attached there, once, while that hypothesis is still open; the status stays the verifier's
                if cover["status"] in V.OPEN and cover.get("predicts") is None:
                    cover["predicts"] = p["predicts"]
                    cover["provenance"]["vl"] = {"model": res["log"].get("model"), "seen_decision_id": seen["decision_id"],
                                                 "seen_candidate": p["candidate"], "rationale": p["rationale"],
                                                 "prior": p["prior"], "predicts": p["predicts"]}
                    cover["orchestrator"] = {k2: res["log"].get(k2) for k2 in
                                             ("provider", "model", "latency_ms", "tokens_in", "tokens_out")}
                    self._event(req, cover, "vl_annotated")
                    made.append(cover["hypothesis_id"])
                else:
                    skipped.append({**p, "why": f"covered by {cover['hypothesis_id']} ({cover['status']}"
                                                f"{', already predicted' if cover.get('predicts') else ''})"})
                continue
            # re-anchor on THIS request: the VL saw an earlier candidate set (indices renumber every replan)
            anchors = self.anchors_for(req, c["centroid_xy_m"])
            k = str(p["agent"]) if str(p["agent"]) in anchors else next(iter(anchors), None)
            if k is not None:
                i = anchors[k]
                cur = next(x for x in req["agents"] if str(x["agent"]) == k)["candidates"][i]
                agent = int(k)
            else:   # gone from this replan's candidates: kept on the seen geometry, it may still resolve
                i, cur, agent = p["candidate"], c, p["agent"]
            prov = {"source": "vl", "model": res["log"].get("model"), "seen_decision_id": seen["decision_id"],
                    "seen_candidate": p["candidate"], "rationale": p["rationale"], "prior": p["prior"],
                    "predicts": p["predicts"], "keyframe_path": self._rel(a.get("keyframe_png")),
                    "map_png": self._rel(seen.get("map_png"))}
            orch = {k2: res["log"].get(k2) for k2 in ("provider", "model", "latency_ms", "tokens_in", "tokens_out")}
            h = self._create(req, agent, i, {**cur, "centroid_xy_m": c["centroid_xy_m"], "goal_xy_m": c["goal_xy_m"]},
                             slab, s, summary, prov, orch, p["predicts"])
            made.append(h["hypothesis_id"])
        rec = {"t_utc": _utc(), "sim_t_s": req["t_s"], "decision_id": req["decision_id"], "event": "vl_call",
               "hypothesis_id": None, "seen_decision_id": seen["decision_id"], "outcome": res["outcome"],
               "reason": res.get("reason"), "proposed": len(res.get("proposals", [])) + len(res.get("invalid", [])),
               "valid": len(res.get("proposals", [])), "invalid": res.get("invalid", []), "duplicates": skipped,
               "created_or_annotated": made, "orchestrator": {k2: res["log"].get(k2) for k2 in
                                                 ("provider", "model", "latency_ms", "tokens_in", "tokens_out",
                                                  "reasoning_tokens", "est_usd", "finish")}}
        with open(self.run / "hypotheses.jsonl", "a") as f:
            f.write(json.dumps(rec) + "\n")
        summary.setdefault("vl", []).append({"seen": seen["decision_id"], "outcome": res["outcome"], "created": made})
        return len(made)

    def hit_rate(self) -> dict:
        """VL predictions (created or annotated) against the verifier's terminal outcome. Only confirmed/refuted count (FM):
        expired, unresolved and supported ones are neither hits nor misses."""
        n = hits = 0
        for h in self.h.values():
            if h.get("predicts") not in ("passage", "dead_end") or h["status"] not in ("confirmed", "refuted"):
                continue
            n += 1
            hits += (h["predicts"] == "passage") == (h["status"] == "confirmed")
        return {"n": n, "hits": hits, "rate": round(hits / n, 3) if n else None}

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
        # 4a: the VL proposer's answer to an EARLIER request (async, one replan behind), validated against the request
        # it saw and re-anchored here by centroid; 4b: the deterministic seed fills in when no VL proposal landed now
        landed = 0
        if self.vl is not None:
            for res in self.vl.collect():
                landed += self._fold_vl(req, res, slab, s, summary)
        if self.vl is None or landed == 0:
            for a in req["agents"]:
                ranked = [(i, c) for i, c in enumerate(a["candidates"]) if not c.get("supervisor_return")][:self.top_k]
                for i, c in ranked:
                    if not self._covered(c["centroid_xy_m"]):
                        self._create(req, a["agent"], i, c, slab, s, summary,
                                     {"source": "seed", "rank": c.get("rank", i),
                                      "keyframe_path": self._rel(a.get("keyframe_png")),
                                      "map_png": self._rel(req.get("map_png"))})
        if self.vl is not None:
            self.vl.submit(req)
        # 5: the view for this decision
        view = sorted((h for h in self.h.values() if h["status"] != "expired"),
                      key=lambda h: (h["status"] not in V.OPEN, -h["created_t_s"]))[:MAX_LISTED]
        (self.ddir / f"{did}.hypotheses.json").write_text(json.dumps({
            "decision_id": did, "t_s": req["t_s"], "rho_responder_m": self.rho_m,
            "slab": summary["slab"], "verifier_error": summary["error"],
            "hypotheses": [{"hypothesis_id": h["hypothesis_id"], "claim": h["claim"], "status": h["status"],
                            "reason": (h.get("verifier") or {}).get("reason"),
                            "contradicted": h.get("contradicted"),
                            "anchors": h["anchors"], "centroid_xy_m": h["centroid_xy_m"],
                            "seeded_by_agent": h["agent_id"], "created_t_s": h["created_t_s"],
                            "source": h["provenance"].get("source"), "vl_predicts": h.get("predicts"),
                            "known_frac": ((h.get("verifier") or {}).get("metrics") or {}).get("known_frac")}
                           for h in view]}))
        tmp = self.state_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.h))
        tmp.replace(self.state_path)
        counts = {}
        for h in self.h.values():
            counts[h["status"]] = counts.get(h["status"], 0) + 1
        out = {**summary, "counts": counts}
        if self.vl is not None:
            out["vl_hit_rate"] = self.hit_rate()
        return out

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


VL_SYSTEM = (
    "You look at a search team's occupancy map (white = observed free, black = occupied, grey = unknown; blue line = "
    "responder egress route; circles = agents; labels aN:cK = agent N's candidate K) and the agents' latest camera "
    "keyframes, from a dark, partially collapsed building. Propose falsifiable hypotheses about which candidate "
    "frontiers open onto a passage a human responder could use (0.40 m clearance), or are dead ends. Only use the "
    "listed candidates. A deterministic verifier will later confirm or refute each from the map the robots build; "
    "your words are never evidence. Call propose_hypotheses once with at most 4 entries.")
VL_TOOL = [{"type": "function", "function": {
    "name": "propose_hypotheses",
    "description": "Hypotheses about candidate frontiers, each tied to one listed candidate.",
    "parameters": {"type": "object", "required": ["hypotheses"], "properties": {"hypotheses": {
        "type": "array", "maxItems": 4, "items": {"type": "object", "required": ["agent", "candidate", "predicts", "rationale"],
            "properties": {"agent": {"type": "integer"}, "candidate": {"type": "integer"},
                           "predicts": {"type": "string", "enum": ["passage", "dead_end"]},
                           "rationale": {"type": "string", "description": "one sentence: what in the images suggests it"},
                           "prior": {"type": "number", "description": "your probability that the prediction holds"}}}}}}}}]
MAX_PROPOSALS = 4


class VLProposer:
    """The Nemotron VL hypothesis worker (F2.4, D319): one call per replan in a background thread, at most one in
    flight. submit(req) starts a call on this request if none is running; collect() returns the finished results
    (folded in at the NEXT replan: the decision never waits for it). Inputs are the runtime's own rendered map and
    camera keyframes plus the planner's candidates: no simulator truth. Output is validated against the request the
    model saw; status is never the model's to set."""

    def __init__(self, provider, spec: dict):
        from concurrent.futures import ThreadPoolExecutor
        self.provider, self.spec = provider, spec
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vl")
        self.pending = None

    def submit(self, req: dict) -> bool:
        if self.pending is not None and not self.pending.done():
            return False
        if self.pending is not None:   # finished but not collected: keep it for collect()
            return False
        self.pending = self.pool.submit(self.call, json.loads(json.dumps(req)))
        return True

    def collect(self) -> list[dict]:
        if self.pending is None or not self.pending.done():
            return []
        f, self.pending = self.pending, None
        return [f.result()]

    @staticmethod
    def _image(path: str | None) -> dict | None:
        import base64
        if not path or not Path(path).exists():
            return None
        return {"type": "image_url", "image_url": {
            "url": "data:image/png;base64," + base64.b64encode(Path(path).read_bytes()).decode()}}

    def messages(self, req: dict) -> list:
        lines = [f"Mission time {req['t_s']:.1f} s. Candidates (from the deterministic planner):"]
        for a in req["agents"]:
            for i, c in enumerate(a["candidates"]):
                if c.get("supervisor_return"):
                    continue
                lines.append(f"a{a['agent']}:c{i} goal {c['goal_xy_m']} frontier centroid {c['centroid_xy_m']} "
                             f"path {c['path_m']:.1f} m, {c['cells']} frontier cells, risk {c['risk']:.2f}")
        content = [{"type": "text", "text": "\n".join(lines) + "\nImage 1 is the map; the next are agent keyframes."}]
        imgs = [self._image(req.get("map_png"))] + [self._image(a.get("keyframe_png")) for a in req["agents"]]
        content += [i for i in imgs if i]
        return [{"role": "system", "content": VL_SYSTEM}, {"role": "user", "content": content}]

    def validate(self, req: dict, raw: list) -> tuple[list, list]:
        ok, bad = [], []
        agents = {a["agent"]: a for a in req["agents"]}
        for p in raw[:MAX_PROPOSALS * 2]:
            try:
                k, i = int(p["agent"]), int(p["candidate"])
                why = None
                if k not in agents:
                    why = "unknown agent"
                elif not 0 <= i < len(agents[k]["candidates"]):
                    why = "candidate out of range"
                elif agents[k]["candidates"][i].get("supervisor_return"):
                    why = "supervisor return"
                elif p.get("predicts") not in ("passage", "dead_end"):
                    why = "predicts must be passage|dead_end"
                elif any(q["agent"] == k and q["candidate"] == i for q in ok):
                    why = "duplicate"
                elif len(ok) >= MAX_PROPOSALS:
                    why = "more than 4"
            except (KeyError, TypeError, ValueError):
                k = i = None
                why = "malformed entry"
            if why:
                bad.append({"entry": str(p)[:200], "why": why})
                continue
            prior = p.get("prior")
            prior = min(1.0, max(0.0, float(prior))) if isinstance(prior, (int, float)) else None
            ok.append({"agent": k, "candidate": i, "predicts": p["predicts"],
                       "rationale": str(p.get("rationale", ""))[:300], "prior": prior})
        return ok, bad

    def call(self, req: dict) -> dict:
        """One paid call (ledger-checked inside provider.chat). Never raises: every outcome is a result."""
        log = {"provider": self.provider.name, "model": self.spec["model"], "latency_ms": None, "tokens_in": 0,
               "tokens_out": 0, "reasoning_tokens": 0, "est_usd": 0.0, "finish": None}
        out = {"request": req, "proposals": [], "invalid": [], "outcome": "ok", "reason": None, "log": log}
        try:
            resp, dt, usd = self.provider.chat(self.spec, self.messages(req), VL_TOOL, f"hypothesis:{req['decision_id']}")
            u = resp.usage
            det = getattr(u, "completion_tokens_details", None)
            log.update(latency_ms=int(dt * 1000), tokens_in=u.prompt_tokens or 0, tokens_out=u.completion_tokens or 0,
                       reasoning_tokens=(getattr(det, "reasoning_tokens", 0) or 0) if det else 0,
                       est_usd=round(usd, 8), finish=resp.choices[0].finish_reason)
            calls = [c for c in resp.choices[0].message.tool_calls or [] if c.function.name == "propose_hypotheses"]
            if not calls:
                out.update(outcome="no_tool_call", reason=f"finish {log['finish']}")
                return out
            raw = json.loads(calls[0].function.arguments or "{}").get("hypotheses") or []
            out["proposals"], out["invalid"] = self.validate(req, raw if isinstance(raw, list) else [])
            if not out["proposals"]:
                out.update(outcome="zero_valid", reason=f"{len(out['invalid'])} invalid")
        except Exception as e:  # noqa: BLE001 - BudgetExceeded, transport, bad JSON: the seed fallback covers it
            name = "budget_exceeded" if type(e).__name__ == "BudgetExceeded" else type(e).__name__
            out.update(outcome="error", reason=f"{name}: {str(e)[:200]}")
        return out
