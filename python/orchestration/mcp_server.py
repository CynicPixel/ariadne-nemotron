"""The MCP tool surface over Ariadne's deterministic core (design agentic.md §0, D157; arch §25.2). mcp 2.3.0, stdio.

  python -m orchestration.mcp_server DECISION_DIR [--text-only]

One decision request per replan covers every agent (§0 change 1, Co-NavGPT's joint assignment). ariadne_live writes it
to DECISION_DIR/<decision_id>.json: per agent the map facts built from keyframe depth, the estimator's state and the
candidate set the planner proved reachable (with the deterministic §23 risk); the egress route at rho_responder; the
paths of the images it rendered with OpenCV. Every tool is a view of that request, plus two actions:
request_human_approval (§23) and assign (the decision). Fork (D317): list_hypotheses reads DECISION_DIR/<did>.hypotheses.json,
which the orchestrator writes before each turn (statuses set by verifier.py only); assign may carry hypothesis_id and
evidence_goal per agent (the C++ gate ignores both; they reach trace.jsonl in the answer). The LLM computes nothing numeric and cannot add a candidate
(§25.1); assign only records indices, which the C++ runtime validates again.

--text-only drops observe_image: the fallback model (DeepSeek V3.2 on Bedrock) accepts TEXT input only (checked with
bedrock get-foundation-model, 2026-10-23). Not exposed (no capability in this loop yet; not stubbed): estimate_pose,
compute_depth, retrieve_place_candidates, verify_loop, request_peer_observation, allocate_task, request_rendezvous.
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from mcp.server.mcpserver import Image, MCPServer

DIR = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
TEXT_ONLY = "--text-only" in sys.argv
APPROVAL_TIMEOUT_S = 60.0   # ariadne_live's llm.timeout_s must exceed this plus model latency (live config)
server = MCPServer("ariadne-core", instructions=(
    "Ariadne mission tools. Every value comes from the deterministic perception/SLAM/planning core at this replan. "
    "Inspect with the query tools, then call assign exactly once with one candidate per agent. A candidate whose "
    "risk exceeds the threshold needs request_human_approval first."))


def _req(decision_id: str) -> dict:
    p = DIR / f"{decision_id}.json"
    if not p.exists():
        raise ValueError(f"unknown decision_id {decision_id!r}")
    return json.loads(p.read_text())


def _agent(r: dict, agent: int) -> dict:
    for a in r["agents"]:
        if a["agent"] == agent:
            return a
    raise ValueError(f"agent {agent} has no decision at this replan")


@server.tool(description="Map facts for one agent at this replan, built from keyframe stereo depth: observed free, "
                         "occupied and unknown cell counts, frontier clusters, keyframes integrated, the agent's "
                         "estimated position.")
def observe_map(decision_id: str, agent: int) -> dict:
    return _agent(_req(decision_id), agent)["vision"]


if not TEXT_ONLY:
    @server.tool(description="Look at an image. what='map': the shared map rendered from keyframe depth (white = "
                             "observed free, black = occupied, grey = unknown; blue line = responder egress route; "
                             "circles = agents; labels aN:cK = agent N's candidate K). what='keyframe': the agent's "
                             "latest camera keyframe (left camera, rectified).")
    def observe_image(decision_id: str, what: str, agent: int = 0) -> Image:
        r = _req(decision_id)
        if what == "map":
            return Image(path=r["map_png"])
        if what == "keyframe":
            p = _agent(r, agent).get("keyframe_png")
            if not p:
                raise ValueError(f"agent {agent} has no keyframe yet")
            return Image(path=p)
        raise ValueError("what must be 'map' or 'keyframe'")


@server.tool(description="One agent's visual odometry state: frames, keyframes, and seconds since the last keyframe. "
                         "Few keyframes or a long gap mean a less certain pose.")
def query_uncertainty(decision_id: str, agent: int) -> dict:
    return _agent(_req(decision_id), agent)["estimator"]


@server.tool(description="The egress route for a human responder (0.40 m clearance) on the current map: found or "
                         "not, length, which exit. Exploration that finds this route serves the mission.")
def query_route(decision_id: str) -> dict:
    return _req(decision_id)["egress"]


@server.tool(description="Both agents' candidate actions, computed by the deterministic planner: frontier clusters "
                         "reachable at the agent's clearance, each with cluster id, path length, cluster size (unknown "
                         "boundary seen: an information proxy), risk in [0,1] with its reason, whether it needs human "
                         "approval, and the operator's intent at its goal (`directives` covering it, `priority_weight`, "
                         "`to_directive_m` = distance to the nearest operator region for this agent, 0 inside). "
                         "Candidate -1 means hold. `classical_choice` is what the classical planner would pick. "
                         "`current_task` is the goal the agent is driving to now (null if none): when it was chosen, "
                         "why, the straight-line distance left, and `candidate` = the index that continues it (-1 if "
                         "it is no longer a candidate). Cluster ids and indices are renumbered every replan; match by "
                         "goal, not id. `operator_directives` are the operator's standing orders. Do not send both "
                         "agents to the same cluster when another one is available.")
def list_candidates(decision_id: str) -> dict:
    r = _req(decision_id)
    return {"autonomous_threshold": r["autonomous_threshold"], "operator_directives": r.get("directives", []),
            "agents": [{"agent": a["agent"], "current_task": a.get("current_task"), "candidates": a["candidates"],
                        "classical_choice": a["classical_choice"]} for a in r["agents"]]}


@server.tool(description="Ask the human operator to approve one agent's candidate whose risk exceeds the autonomous "
                         "threshold (§23). Blocks until the operator answers or the request times out. Returns the "
                         "verdict (approved, rejected or timeout) and the approval_id to pass to assign.")
def request_human_approval(decision_id: str, agent: int, candidate: int, reason: str) -> dict:
    r = _req(decision_id)
    a = _agent(r, agent)
    if not 0 <= candidate < len(a["candidates"]):
        return {"verdict": "invalid", "detail": "candidate index out of range"}
    aid = f"{decision_id}-a{agent}-c{candidate}"
    adir = Path(r["approvals_dir"])
    adir.mkdir(parents=True, exist_ok=True)
    c = a["candidates"][candidate]
    alt = a["classical_choice"]
    packet = {  # §23 HUMAN_APPROVAL_REQUEST
        "approval_id": aid, "decision_id": decision_id, "t_s": r["t_s"], "agent": agent,
        "map_snapshot": r["map_png"], "agent_position_xy_m": a["vision"].get("agent_xy_m"),
        "intended_action": {"candidate": candidate, "cluster": c["cluster"], "goal_xy_m": c["goal_xy_m"],
                            "path_m": c["path_m"]},
        "evidence": {"cluster_cells": c["cells"], "standoff_m": c["standoff_m"], "keyframe": a.get("keyframe_png")},
        "uncertainty": a["estimator"], "predicted_benefit": f"{c['cells']} frontier cells",
        "risk_score": c["risk"], "risk_reason": c["risk_reason"],
        "alternative_action": {"candidate": alt} if alt >= 0 else {"hold": True},
        "orchestrator_reason": reason, "requested_unix_s": time.time(), "deadline_unix_s": time.time() + APPROVAL_TIMEOUT_S}
    (adir / f"{aid}.request.json").write_text(json.dumps(packet, indent=1))
    deadline = time.time() + APPROVAL_TIMEOUT_S
    while time.time() < deadline:
        v = adir / f"{aid}.json"
        if v.exists():
            d = json.loads(v.read_text())
            return {"approval_id": aid, "verdict": d.get("verdict"), "operator": d.get("operator")}
        time.sleep(0.5)
    return {"approval_id": aid, "verdict": "timeout"}


@server.tool(description="Propose an operator directive from the operator's words or from what you see: kind is "
                         "priority_region | keep_out | assign | explore (polygon = [[x,y],...] in metres, the map "
                         "frame) or hold | release (agent). It takes effect ONLY after a person confirms it in the "
                         "console; say why in `text`.")
def propose_directive(decision_id: str, kind: str, text: str, agent: int = -1, polygon: list[list[float]] | None = None,
                      weight: float = 1.0, time_budget_s: float = 0.0) -> dict:
    r = _req(decision_id)
    if kind not in ("priority_region", "keep_out", "assign", "explore", "hold", "release"):
        return {"proposed": False, "detail": "kind not allowed for proposals"}
    if kind in ("priority_region", "keep_out", "assign", "explore") and (not polygon or len(polygon) < 3):
        return {"proposed": False, "detail": "polygon needs >= 3 [x, y] points"}
    run = Path(r["approvals_dir"]).parent
    pid = f"llm-{decision_id}-{int(time.time() * 1000) % 100000}"
    d = {"id": pid, "kind": kind, "agent": agent, "polygon": polygon or [], "weight": weight,
         "time_budget_s": time_budget_s, "text": text}
    (run / "directives_proposed").mkdir(exist_ok=True)
    (run / "directives_proposed" / f"{pid}.json").write_text(json.dumps(d))
    return {"proposed": True, "id": pid, "status": "awaiting operator confirmation"}


@server.tool(description="Open hypotheses about where a responder-passable passage lies, anchored to this replan's "
                         "candidates (`anchors`: agent -> candidate index), with status set ONLY by a deterministic "
                         "verifier from the observed map (unresolved | supported | confirmed | refuted | expired) and "
                         "its reason. An unresolved or supported hypothesis is evidence worth seeking: sending an agent "
                         "to its candidate lets the next map confirm or refute it. Refuted ones are dead ends.")
def list_hypotheses(decision_id: str) -> dict:
    _req(decision_id)
    p = DIR / f"{decision_id}.hypotheses.json"
    if not p.exists():
        return {"enabled": False, "hypotheses": []}
    return {"enabled": True, **json.loads(p.read_text())}


def _open_hypotheses(decision_id: str) -> dict:
    p = DIR / f"{decision_id}.hypotheses.json"
    if not p.exists():
        return {}
    h = json.loads(p.read_text())
    return {x["hypothesis_id"]: x for x in h["hypotheses"] if x["status"] in ("unresolved", "supported")}


@server.tool(description="The decision: one entry per agent, {agent, candidate (-1 = hold), reason, approval_id "
                         "(only for a candidate that needed approval), hypothesis_id (optional: the open hypothesis this "
                         "move tests), evidence_goal (optional, one sentence: what the agent should observe)}. Call "
                         "exactly once, last.")
def assign(decision_id: str, assignments: list[dict]) -> dict:
    r = _req(decision_id)
    agents = {a["agent"]: a for a in r["agents"]}
    hyps = _open_hypotheses(decision_id)
    seen, clean, notes = set(), [], []
    for e in assignments:
        k, c = int(e.get("agent", -9)), int(e.get("candidate", -9))
        if k not in agents or k in seen:
            return {"recorded": False, "detail": f"agent {k} is not decided at this replan, or is listed twice"}
        if not -1 <= c < len(agents[k]["candidates"]):
            return {"recorded": False, "detail": f"agent {k}: candidate {c} is out of range; assign again"}
        hid = str(e.get("hypothesis_id") or "")
        seen.add(k)
        rec = {"agent": k, "candidate": c, "reason": str(e.get("reason", ""))[:300],
               "approval_id": str(e.get("approval_id", ""))}
        if hid and hid in hyps:
            rec["hypothesis_id"] = hid
            rec["hypothesis_anchored"] = hyps[hid].get("anchors", {}).get(str(k)) == c
        elif hid:   # a stale or unknown id never costs the decision: the move is recorded, the id is not
            rec["hypothesis_rejected"] = f"{hid[:40]} is not an open hypothesis at this replan"
            notes.append(f"agent {k}: {rec['hypothesis_rejected']} (move recorded without it)")
        if e.get("evidence_goal"):
            rec["evidence_goal"] = str(e["evidence_goal"])[:200]
        clean.append(rec)
    if seen != set(agents):
        return {"recorded": False, "detail": f"assign every agent: {sorted(agents)}"}
    (DIR / f"{decision_id}.choice.json").write_text(json.dumps({"assignments": clean}))
    return {"recorded": True, **({"notes": notes} if notes else {})}


if __name__ == "__main__":
    server.run("stdio")
