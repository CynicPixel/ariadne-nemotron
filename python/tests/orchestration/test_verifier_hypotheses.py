"""The slab verifier (verifier.py, D316) on synthetic slabs, its slab guard (D314), and the hypothesis layer
(hypotheses.py, D315/D317) replayed over a recorded run's requests and slabs, with list_hypotheses / assign through the
real MCP server. No network.

  docker run --rm -v "$PWD":/work:ro -w /work/python ariadne-orchestrator:nemotron-1 python -m pytest -q \\
      -p no:cacheprovider tests/orchestration/test_verifier_hypotheses.py

The recorded run (llm_live_run5, 2026-10-23) predates the slab field: its requests get one here, pointing at the
run's own snapshots/slab_<ms>.png of the same replan. Its grid (res 0.05 m, origin (-3.50, 0.05)) is fitted from
the recorded agent_cell / agent_xy_m pairs [ASSUMPTION, checked by test_corpus_geometry]; a live request carries it.
"""
import asyncio
import json
import shutil
from pathlib import Path

import numpy as np
import pytest
from mcp.client.client import Client

from orchestration import verifier as V
from orchestration.hypotheses import HypothesisLayer
from orchestration.orchestrator import mcp_params

REPO = Path(__file__).resolve().parents[3]
CORPUS = REPO / "evaluation" / "runs" / "live-agentic" / "live_loop" / "llm_live_run5_maze_short_90s"
RES, ORIGIN = 0.05, (-3.5, 0.05)
HCFG = {"seed_top_k": 2, "match_radius_m": 1.0, "region_radius_m": 1.5, "rho_responder_m": 0.40,
        "expire_after_replans": 3}


def grid(fill=V.UNKNOWN, n=200):
    return np.full((n, n), fill, np.uint8)


def xy(r, c):   # cell centre, origin (0, 0), res 0.05
    return ((c + 0.5) * RES, (r + 0.5) * RES)


def run_v(slab, centroid=(100, 100), goal=(100, 60)):
    return V.verify(slab, RES, (0.0, 0.0), xy(*centroid), xy(*goal), 1.5, 0.40)


def corridor(width_cells, far_known=True, solid=False):
    """An east-west corridor through the disk (rows 100 +- width/2), its walls observed; beyond the walls unknown, or
    observed occupied (solid: a fully mapped corridor)."""
    s = grid(V.OCCUPIED if solid else V.UNKNOWN)
    h = width_cells // 2
    s[100 - h - 2:100 + h + 3, :] = V.OCCUPIED
    s[100 - h:100 + h + 1, :] = V.FREE
    if not far_known:
        s[:, 101:] = V.UNKNOWN
    return s


# --- verdicts ---

def test_wide_observed_corridor_is_confirmed():
    v = run_v(corridor(26))            # 1.35 m wide: the 0.40 m clearance leaves a ~0.5 m free lane
    assert v["verdict"] == "confirmed", v


def test_narrow_corridor_is_refuted():
    v = run_v(corridor(12, solid=True))   # 0.65 m wide < 2 x 0.40 m: no cell keeps 0.40 m from both walls
    assert v["verdict"] == "refuted" and "0.40 m clearance" in v["reason"], v


def test_narrow_corridor_with_unseen_surroundings_is_not_refuted():
    # the same corridor, but beyond its walls nothing is observed: a passage there is not ruled out
    assert run_v(corridor(12))["verdict"] == "unresolved"


def test_open_way_in_with_unseen_far_side_is_supported():
    v = run_v(corridor(26, far_known=False))
    assert v["verdict"] == "supported", v


def test_wall_across_the_disk_refutes_even_with_unknown_beyond():
    s = grid()
    s[:, :101] = V.FREE
    s[:, 101:104] = V.OCCUPIED          # a wall north-south across the whole disk, the far side unseen
    assert run_v(s)["verdict"] == "refuted"


def test_unobserved_disk_is_unresolved():
    assert run_v(grid())["verdict"] == "unresolved"


def test_off_grid_counts_as_unknown():
    s = grid(V.FREE, n=60)              # the disk around (100, 100) lies outside this 60x60 grid
    assert V.verify(s, RES, (0.0, 0.0), xy(100, 100), xy(100, 60))["verdict"] == "unresolved"


def test_verdict_is_deterministic():
    s = corridor(26, far_known=False)
    assert run_v(s) == run_v(s)


# --- status transitions: only the verifier's verdict moves a status; terminal stays ---

def test_next_status():
    assert V.next_status("unresolved", "supported", 0, 3) == ("supported", "status")
    assert V.next_status("supported", "confirmed", 0, 3) == ("confirmed", "status")
    assert V.next_status("confirmed", "refuted", 0, 3) == ("confirmed", "contradiction")
    assert V.next_status("refuted", "refuted", 0, 3) == ("refuted", None)
    assert V.next_status("supported", "supported", 3, 3) == ("expired", "status")
    assert V.next_status("supported", "refuted", 5, 3) == ("refuted", "status")   # evidence wins over expiry
    assert V.next_status("expired", "confirmed", 9, 3) == ("expired", None)


# --- the slab guard (D314, FM's condition) ---

def _slab_req(ms=20000, slab_ms=20000, t=20.0, slab_t=20.0, nx_ny=(403, 368)):
    return {"decision_id": f"t{ms}ms", "t_s": t,
            "slab": {"path": f"/run/snapshots/slab_{slab_ms}.png", "t_s": slab_t, "res_m": RES, "origin_xy_m": ORIGIN,
                     "rows": 368, "cols": 403, "grid_nx_ny": list(nx_ny)}}


def test_slab_guard_accepts_the_same_replan():
    assert V.check_slab(_slab_req())["path"].endswith("slab_20000.png")
    assert V.check_slab(_slab_req(slab_ms=20001))   # llround vs truncation of the same T


def test_slab_guard_rejects_a_later_slab_or_other_time():
    with pytest.raises(V.SlabMismatch):
        V.check_slab(_slab_req(slab_ms=25000))
    with pytest.raises(V.SlabMismatch):
        V.check_slab(_slab_req(slab_t=25.0))
    with pytest.raises(V.SlabMismatch):
        V.check_slab(_slab_req(nx_ny=(400, 368)))
    with pytest.raises(V.SlabMismatch):
        V.check_slab({"decision_id": "t1ms", "t_s": 0.001})


# --- the recorded run ---

def corpus_requests(tmp: Path, n=None) -> list[dict]:
    files = sorted((CORPUS / "decisions").glob("t*ms.json"), key=lambda p: int(p.stem[1:-2]))
    out = []
    for f in files[:n]:
        r = json.loads(f.read_text())
        ms = int(r["decision_id"][1:-2])
        r["slab"] = {"path": str(CORPUS / "snapshots" / f"slab_{ms}.png"), "t_s": r["t_s"], "res_m": RES,
                     "origin_xy_m": list(ORIGIN), "rows": 368, "cols": 403, "grid_nx_ny": [403, 368]}
        r["approvals_dir"] = str(tmp / "approvals")
        out.append(r)
    return out


def test_corpus_geometry(tmp_path):
    """Recorded agent_cell == floor((xy - origin) / res), and the agent stands on observed free space."""
    for r in corpus_requests(tmp_path):
        slab = V.load_slab(V.check_slab(r))
        for a in r["agents"]:
            (row, col), (x, y) = a["vision"]["agent_cell"], a["vision"]["agent_xy_m"]
            assert (int((y - ORIGIN[1]) // RES), int((x - ORIGIN[0]) // RES)) == (row, col)
            near = slab[row - 2:row + 3, col - 2:col + 3]
            assert (near == V.FREE).any()


def _layer(tmp: Path) -> HypothesisLayer:
    ddir = tmp / "run" / "decisions"
    ddir.mkdir(parents=True)
    return HypothesisLayer(ddir, HCFG)


def test_layer_seeds_reanchors_and_verifies(tmp_path):
    hl = _layer(tmp_path)
    reqs = corpus_requests(tmp_path)
    for r in reqs:
        s = hl.before_turn(r)
        assert s["error"] is None
    ev = [json.loads(x) for x in (tmp_path / "run" / "hypotheses.jsonl").read_text().splitlines()]
    vr = [json.loads(x) for x in (tmp_path / "run" / "verifier.jsonl").read_text().splitlines()]
    kinds = {e["event"] for e in ev}
    assert {"created", "status"} <= kinds
    assert all(e["status_source"] == "verifier" for e in ev if e["event"] in ("status", "contradiction"))
    assert all(e["status_source"] == "none" for e in ev if e["event"] == "created")
    # every status event follows a verifier line for the same hypothesis and decision, on that decision's slab
    seen = {(v["hypothesis_id"], v["decision_id"]) for v in vr}
    assert all((e["hypothesis_id"], e["decision_id"]) in seen for e in ev if e["event"] == "status")
    assert all(Path(v["slab"]["path"]).name == f"slab_{v['decision_id'][1:-2]}.png" for v in vr)
    final = {}
    for e in ev:
        final[e["hypothesis_id"]] = e["status"]
    assert set(final.values()) <= {"unresolved", "supported", "confirmed", "refuted", "expired"}
    assert "refuted" in final.values() and "confirmed" in final.values()   # the recorded run has both outcomes
    view = json.loads((tmp_path / "run" / "decisions" / f"{reqs[-1]['decision_id']}.hypotheses.json").read_text())
    assert view["decision_id"] == reqs[-1]["decision_id"] and len(view["hypotheses"]) <= 12


def test_layer_inputs_are_request_and_slab_only(tmp_path):
    """No ground truth can reach the layer: its only inputs are the request (planner output) and the observed slab.
    A request stripped to the fields the layer documents gives the same events as the full one."""
    keep = {"decision_id", "t_s", "slab", "map_png", "agents"}
    a, b = _layer(tmp_path / "a"), _layer(tmp_path / "b")
    for r in corpus_requests(tmp_path, 6):
        a.before_turn(r)
        b.before_turn({k: v for k, v in r.items() if k in keep})
    strip = lambda p: [{k: v for k, v in json.loads(x).items() if k != "t_utc"} for x in p.read_text().splitlines()]   # noqa: E731
    assert strip(tmp_path / "a/run/hypotheses.jsonl") == strip(tmp_path / "b/run/hypotheses.jsonl")


def test_missing_slab_is_logged_not_fatal(tmp_path):
    hl = _layer(tmp_path)
    r = corpus_requests(tmp_path, 1)[0]
    del r["slab"]
    s = hl.before_turn(r)
    assert "no slab" in s["error"] and s["new"]          # seeded, statuses left unresolved
    assert not (tmp_path / "run" / "verifier.jsonl").exists()


def test_mcp_list_hypotheses_and_assign_fields(tmp_path):
    hl = _layer(tmp_path)
    ddir = tmp_path / "run" / "decisions"
    r = corpus_requests(tmp_path, 1)[0]
    (ddir / f"{r['decision_id']}.json").write_text(json.dumps(r))
    hl.before_turn(r)

    async def go():
        async with Client(mcp_params(ddir, True)) as m:
            lh = json.loads((await m.call_tool("list_hypotheses", {"decision_id": r["decision_id"]})).content[0].text)
            h = next(x for x in lh["hypotheses"] if x["status"] in ("unresolved", "supported"))
            k, c = next(iter(h["anchors"].items()))
            other = 1 - int(k)
            bad = await m.call_tool("assign", {"decision_id": r["decision_id"], "assignments": [
                {"agent": int(k), "candidate": c, "hypothesis_id": "h999"}, {"agent": other, "candidate": 0}]})
            stale = json.loads((ddir / f"{r['decision_id']}.choice.json").read_text())["assignments"]
            ok = await m.call_tool("assign", {"decision_id": r["decision_id"], "assignments": [
                {"agent": int(k), "candidate": c, "hypothesis_id": h["hypothesis_id"],
                 "evidence_goal": "look past the frontier for a 0.8 m gap"}, {"agent": other, "candidate": 0}]})
            return lh, h, bad, ok, stale
    lh, h, bad, ok, stale = asyncio.run(go())
    assert lh["enabled"] and lh["rho_responder_m"] == 0.40
    # a stale/unknown hypothesis id never costs the decision: recorded without the id, with a note
    assert '"recorded":true' in bad.content[0].text.replace(" ", "") and "h999" in bad.content[0].text
    e0 = next(x for x in stale if x["agent"] == int(next(iter(h["anchors"]))))
    assert "hypothesis_id" not in e0 and "h999" in e0["hypothesis_rejected"]
    assert '"recorded":true' in ok.content[0].text.replace(" ", "")
    choice = json.loads((ddir / f"{r['decision_id']}.choice.json").read_text())["assignments"]
    e = next(x for x in choice if x.get("hypothesis_id"))
    assert e["hypothesis_id"] == h["hypothesis_id"] and e["hypothesis_anchored"] is True
    assert e["evidence_goal"].startswith("look past")
    hl.after_turn(r, choice)
    last = json.loads((tmp_path / "run" / "hypotheses.jsonl").read_text().splitlines()[-1])
    assert last["event"] == "targeted" and last["evidence_goal"].startswith("look past")


def test_sample_fixture_matches_the_schema():
    """fixtures/hypotheses_sample.jsonl (for FV's console panel) carries every documented key."""
    keys = {"t_utc", "sim_t_s", "decision_id", "event", "hypothesis_id", "agent_id", "anchor", "anchors", "claim",
            "evidence_goal", "status", "status_source", "verifier", "provenance"}
    fx = Path(__file__).parent / "fixtures"
    seed = [json.loads(x) for x in (fx / "hypotheses_sample.jsonl").read_text().splitlines()]
    vl = [json.loads(x) for x in (fx / "hypotheses_sample_vl.jsonl").read_text().splitlines()]
    for lines in (seed, vl):   # vl_call lines carry hypothesis_id null and their own keys; the panel skips them
        assert lines and all(keys <= set(x) for x in lines if x["hypothesis_id"])
    assert {x["event"] for x in seed} >= {"created", "status", "targeted"}
    assert {x["event"] for x in vl} >= {"created", "status", "vl_call", "vl_annotated", "targeted"}
    assert all(x["hypothesis_id"] is None and x["outcome"] for x in vl if x["event"] == "vl_call")


# --- the VL proposer (F2.4, D319): async one replan behind, validated, logged, seed fallback ---

def _vl_layer(tmp: Path, script):
    from test_nebius_provider import FakeOpenAI
    from orchestration.hypotheses import VLProposer
    from orchestration.providers import load_config, make_provider
    from orchestration.providers.ledger import Ledger
    cfg = load_config(REPO / "configs" / "llm" / "nemotron.yaml")
    fake = FakeOpenAI(script)
    prov = make_provider("nebius", cfg, client=fake, ledger=Ledger(tmp / "spend.jsonl", cfg["prices_usd_per_mtok"]),
                         purpose="hypothesis")
    ddir = tmp / "run" / "decisions"
    ddir.mkdir(parents=True)
    return HypothesisLayer(ddir, {**HCFG, **{k: v for k, v in cfg["hypothesis"].items() if k not in HCFG}},
                           VLProposer(prov, cfg["hypothesis"])), fake


def _proposal(entries):
    from test_nebius_provider import synth
    return synth([("propose_hypotheses", {"hypotheses": entries})], model="nvidia/Nemotron-3.5-Super-VL", pt=3000, ct=120)


def _step(hl, r):
    s = hl.before_turn(r)
    if hl.vl.pending is not None:
        hl.vl.pending.result(timeout=10)   # the test waits; live, the next replan simply finds it done (or not)
    return s


def test_vl_proposals_fold_in_one_replan_later(tmp_path):
    reqs = corpus_requests(tmp_path, 3)
    good = [{"agent": 0, "candidate": 1, "predicts": "dead_end", "rationale": "walls close in on the map", "prior": 0.7},
            {"agent": 1, "candidate": 0, "predicts": "passage", "rationale": "open corridor in the keyframe"},
            {"agent": 1, "candidate": 99, "predicts": "passage", "rationale": "x"},
            {"agent": 0, "candidate": 0, "predicts": "maybe", "rationale": "x"}]
    hl, fake = _vl_layer(tmp_path, [_proposal(good), _proposal([]), _proposal([])])
    s0 = _step(hl, reqs[0])
    assert s0["new"] and all(hl.h[h]["provenance"]["source"] == "seed" for h in s0["new"])   # nothing landed yet
    s1 = _step(hl, reqs[1])
    assert all(hl.h[h]["provenance"]["source"] == "vl" for h in s1["new"])   # VL landed: no seeding this replan
    landed = s1["vl"][0]["created"]
    assert len(landed) == 2
    h = hl.h[landed[0]]
    vl = h["provenance"] if h["provenance"]["source"] == "vl" else h["provenance"]["vl"]   # created or annotated
    assert vl["seen_decision_id"] == reqs[0]["decision_id"] and h["predicts"] in ("passage", "dead_end")
    assert h["orchestrator"]["model"] == "nvidia/Nemotron-3.5-Super-VL" and h["status"] in V.OPEN + V.TERMINAL
    ev = [json.loads(x) for x in (tmp_path / "run" / "hypotheses.jsonl").read_text().splitlines()]
    calls = [e for e in ev if e["event"] == "vl_call"]
    assert calls[0]["valid"] == 2 and len(calls[0]["invalid"]) == 2 and len(calls[0]["created_or_annotated"]) == 2
    assert {i["why"] for i in calls[0]["invalid"]} == {"candidate out of range", "predicts must be passage|dead_end"}
    assert calls[0]["orchestrator"]["tokens_in"] == 3000 and calls[0]["orchestrator"]["latency_ms"] is not None
    s2 = _step(hl, reqs[2])                                   # the second VL answer proposed nothing: seed fills in
    assert any(e["event"] == "vl_call" and e["outcome"] == "zero_valid" for e in
               (json.loads(x) for x in (tmp_path / "run" / "hypotheses.jsonl").read_text().splitlines()))
    assert all(hl.h[h]["provenance"]["source"] == "seed" for h in s2["new"])
    # the VL request: the runtime's own images and the planner's candidates, nothing else
    sent = fake.sent[0]
    assert sent["model"] == "nvidia/Nemotron-3.5-Super-VL" and sent["max_tokens"] <= 2048
    assert sent["extra_body"] == {"reasoning_effort": "none"}
    text = sent["messages"][1]["content"][0]["text"]
    assert "a0:c0" in text and "centroid" in text


def test_vl_failure_is_logged_and_seeded(tmp_path):
    from test_nebius_provider import _conn_error
    reqs = corpus_requests(tmp_path, 2)
    hl, _ = _vl_layer(tmp_path, [_conn_error(), _proposal([])])
    _step(hl, reqs[0])
    s1 = _step(hl, reqs[1])
    assert s1["new"] and all(hl.h[h]["provenance"]["source"] == "seed" for h in s1["new"])
    call = next(json.loads(x) for x in (tmp_path / "run" / "hypotheses.jsonl").read_text().splitlines()
                if json.loads(x)["event"] == "vl_call")
    assert call["outcome"] == "error" and "APIConnectionError" in call["reason"]


def test_vl_hit_rate_counts_terminal_outcomes_only(tmp_path):
    hl = _layer(tmp_path)
    mk = lambda st, pred, src="vl": {"status": st, "predicts": pred, "provenance": {"source": src}}   # noqa: E731
    hl.h = {"a": mk("confirmed", "passage"), "b": mk("refuted", "passage"), "c": mk("refuted", "dead_end"),
            "d": mk("expired", "passage"), "e": mk("supported", "dead_end"), "f": mk("confirmed", None, "seed"),
            "g": mk("refuted", "dead_end", "seed")}   # a seed annotated by the VL counts
    assert hl.hit_rate() == {"n": 4, "hits": 3, "rate": 0.75}
    del hl.h["g"]
    assert hl.hit_rate() == {"n": 3, "hits": 2, "rate": 0.667}


def test_recorded_vl_responses_fold_in(tmp_path):
    """Two REAL Super-VL responses (fixtures/vl_raw, recorded 2026-10-29 ~05:00 IST by a host serve() run of the G1
    config over llm_live_run5 t20000ms..t30000ms, maps rendered from the recorded slabs) through validate + fold-in."""
    raw = [json.loads((Path(__file__).parent / "fixtures" / "vl_raw" / f"seen_{d}.json").read_text())["response"]
           for d in ("t20000ms", "t25000ms")]
    reqs = {r["decision_id"]: r for r in corpus_requests(tmp_path)}
    hl, fake = _vl_layer(tmp_path, raw)
    for d in ("t20000ms", "t25000ms", "t30000ms"):
        _step(hl, reqs[d])
    calls = [json.loads(x) for x in (tmp_path / "run" / "hypotheses.jsonl").read_text().splitlines()
             if json.loads(x)["event"] == "vl_call"]
    assert [c["outcome"] for c in calls] == ["ok", "ok"] and all(c["created_or_annotated"] for c in calls)
    assert all(c["orchestrator"]["model"] == "nvidia/Nemotron-3.5-Super-VL" for c in calls)
    vl = [h for h in hl.h.values() if h.get("predicts")]
    assert vl and all(h["predicts"] in ("passage", "dead_end") for h in vl)


def test_contradictions_logged_once_per_change_and_surfaced(tmp_path):
    hl = _layer(tmp_path)
    req = {"decision_id": "t1000ms", "t_s": 1.0}
    h = {"hypothesis_id": "h001", "agent_id": 0, "anchor": {}, "anchors": {}, "claim": "c", "status": "confirmed",
         "unmatched": 0, "provenance": {"source": "seed"}}
    ref = {"verdict": "refuted", "reason": "observed obstacles", "metrics": {}}
    for t in (1.0, 2.0, 3.0):
        hl._apply({**req, "t_s": t}, h, dict(ref))
    assert h["status"] == "confirmed" and h["contradicted"]["since_t_s"] == 1.0 and h["contradiction_replans"] == 3
    hl._apply({**req, "t_s": 4.0}, h, {"verdict": "confirmed", "reason": "crossing", "metrics": {}})
    assert h["contradicted"] is None
    ev = [json.loads(x)["event"] for x in (tmp_path / "run" / "hypotheses.jsonl").read_text().splitlines()]
    assert ev == ["contradiction", "contradiction_cleared"]


def test_assign_asks_for_approval_before_an_above_threshold_pick(tmp_path):
    hl = _layer(tmp_path)
    ddir = tmp_path / "run" / "decisions"
    r = corpus_requests(tmp_path, 1)[0]
    r["agents"][0]["candidates"][1]["needs_approval"] = True
    r["agents"][0]["candidates"][1]["risk"] = 0.8
    (ddir / f"{r['decision_id']}.json").write_text(json.dumps(r))
    hl.before_turn(r)
    did = r["decision_id"]

    async def go():
        async with Client(mcp_params(ddir, True)) as m:
            asg = [{"agent": 0, "candidate": 1}, {"agent": 1, "candidate": 0}]
            no = await m.call_tool("assign", {"decision_id": did, "assignments": asg})
            (tmp_path / "approvals").mkdir(exist_ok=True)
            (tmp_path / "approvals" / f"{did}-a0-c1.json").write_text(json.dumps({"verdict": "approved"}))
            asg[0]["approval_id"] = f"{did}-a0-c1"
            yes = await m.call_tool("assign", {"decision_id": did, "assignments": asg})
            return no.content[0].text, yes.content[0].text
    no, yes = asyncio.run(go())
    assert '"recorded": false' in no and "request_human_approval(agent=0, candidate=1)" in no
    assert '"recorded": true' in yes


def test_assign_catches_a_hypothesis_index_slip(tmp_path):
    # G2 t95000ms: hypothesis anchored to candidate 0, index 53 sent.
    hl = _layer(tmp_path)
    ddir = tmp_path / "run" / "decisions"
    r = corpus_requests(tmp_path, 1)[0]
    (ddir / f"{r['decision_id']}.json").write_text(json.dumps(r))
    hl.before_turn(r)
    view = json.loads((ddir / f"{r['decision_id']}.hypotheses.json").read_text())
    h = next(x for x in view["hypotheses"] if x["status"] in ("unresolved", "supported") and "0" in x["anchors"])
    good = h["anchors"]["0"]
    wrong = next(i for i in range(len(r["agents"][0]["candidates"])) if i != good)

    async def go():
        async with Client(mcp_params(ddir, True)) as m:
            res = []
            for c in (wrong, good):
                res.append((await m.call_tool("assign", {"decision_id": r["decision_id"], "assignments": [
                    {"agent": 0, "candidate": c, "hypothesis_id": h["hypothesis_id"]},
                    {"agent": 1, "candidate": -1}]})).content[0].text)
            return res
    bad, ok = asyncio.run(go())
    assert '"recorded": false' in bad and f"tested by candidate {good}" in bad
    assert '"recorded": true' in ok
