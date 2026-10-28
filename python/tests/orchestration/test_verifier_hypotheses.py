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
            ok = await m.call_tool("assign", {"decision_id": r["decision_id"], "assignments": [
                {"agent": int(k), "candidate": c, "hypothesis_id": h["hypothesis_id"],
                 "evidence_goal": "look past the frontier for a 0.8 m gap"}, {"agent": other, "candidate": 0}]})
            return lh, h, bad, ok
    lh, h, bad, ok = asyncio.run(go())
    assert lh["enabled"] and lh["rho_responder_m"] == 0.40
    assert '"recorded":false' in bad.content[0].text.replace(" ", "")
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
    lines = [json.loads(x) for x in (Path(__file__).parent / "fixtures" / "hypotheses_sample.jsonl").read_text().splitlines()]
    assert lines and all(keys <= set(x) for x in lines)
    assert {x["event"] for x in lines} >= {"created", "status"}
