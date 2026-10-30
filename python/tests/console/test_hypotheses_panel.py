"""The console's Hypotheses panel backend (fork F3.5) against FP's recorded fixture (design §3.1)."""
import json
import shutil
from pathlib import Path

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from operator_console.hypotheses_panel import router

FIXTURE = Path(__file__).resolve().parents[1] / "orchestration/fixtures/hypotheses_sample.jsonl"
PNG = b"\x89PNG\r\n\x1a\n" + b"\0" * 16


@pytest.fixture
def run(tmp_path, monkeypatch):
    """An outer run dir as live.sh lays it out: hypotheses.jsonl beside decisions/, the runtime's trace one level down."""
    shutil.copy(FIXTURE, tmp_path / "hypotheses.jsonl")
    (tmp_path / "decisions").mkdir()
    (tmp_path / "decisions/t5000ms_map.png").write_bytes(PNG)
    inner = tmp_path / "scene-live-20261029T000000Z"
    inner.mkdir()
    orch = {"provider": "nebius", "model": "nvidia/nemotron-3-super-120b-a12b", "latency_s": 1.24, "input_tokens": 456,
            "output_tokens": 80, "fallback_used": False, "errors": []}
    (inner / "trace.jsonl").write_text(
        json.dumps({"t_s": 5, "choice": {"source": "classical"}}) + "\n"
        + json.dumps({"t_s": 10, "choice": {"source": "orchestrator", "decision_id": "t10000ms", "validation": "ok",
                                           "answer": {"orchestrator": orch}}}) + "\n")
    monkeypatch.delenv("ARIADNE_HYPOTHESES", raising=False)
    monkeypatch.setenv("ARIADNE_LIVE", f"{tmp_path}:/nonexistent/scene")
    app = FastAPI()
    app.include_router(router)
    return tmp_path, TestClient(app)


def last_status(path):
    last = {}
    for line in path.read_text().splitlines():
        r = json.loads(line)
        last[r["hypothesis_id"]] = r["status"]
    return last


def test_newest_record_per_hypothesis_is_shown_as_recorded(run):
    root, c = run
    d = c.get("/api/hypotheses").json()
    assert d["available"] is True
    assert {h["hypothesis_id"]: h["status"] for h in d["hypotheses"]} == last_status(FIXTURE)
    # the fixture ends with every hypothesis settled: h001 and h005 confirmed, h008 refuted
    assert {"confirmed", "refuted"} <= {h["status"] for h in d["hypotheses"]}
    assert sum(d["counts"].values()) == len(d["hypotheses"])
    assert all(ch["status"] for ch in d["changes"])


def test_open_hypotheses_listed_first(run):
    root, c = run
    with open(root / "hypotheses.jsonl", "a") as f:
        f.write(json.dumps({"sim_t_s": 60, "event": "created", "hypothesis_id": "h099", "agent_id": 1,
                            "status": "unresolved", "status_source": "none", "provenance": {"source": "seed"}}) + "\n")
    hs = c.get("/api/hypotheses").json()["hypotheses"]
    assert hs[0]["hypothesis_id"] == "h099" and hs[0]["map_url"] is None


def test_status_history_and_targeting(run):
    _, c = run
    hs = c.get("/api/hypotheses").json()["hypotheses"]
    for h in hs:
        assert h["history"][0]["source"] == "none"                              # created by the layer, not the verifier
        assert all(s["source"] == "verifier" for s in h["history"][1:])         # every later status came from verifier.py
    assert any(h["targeted"] for h in hs)                                       # the fixture's one `targeted` event


def test_decision_block_is_the_newest_orchestrator_answer(run):
    _, c = run
    dec = c.get("/api/hypotheses").json()["decision"]
    assert dec["model"] == "nvidia/nemotron-3-super-120b-a12b" and dec["latency_s"] == 1.24
    assert (dec["provider"], dec["input_tokens"], dec["output_tokens"], dec["decision_id"]) == ("nebius", 456, 80, "t10000ms")


def test_provenance_image_served_from_inside_the_run_only(run):
    root, c = run
    h = next(h for h in c.get("/api/hypotheses").json()["hypotheses"] if h["map_url"])
    assert h["map_url"] == "/api/hypotheses/file?path=decisions/t5000ms_map.png"   # absolute container path re-rooted
    r = c.get(h["map_url"])
    assert r.status_code == 200 and r.content == PNG and r.headers["content-type"] == "image/png"
    (root.parent / "secret.png").write_bytes(PNG)
    for bad in ("../secret.png", "/etc/passwd", "decisions/../../secret.png", "hypotheses.jsonl"):
        assert c.get("/api/hypotheses/file", params={"path": bad}).status_code == 404


def test_tolerant_of_a_half_written_line_and_of_no_file(run, monkeypatch, tmp_path_factory):
    root, c = run
    with open(root / "hypotheses.jsonl", "a") as f:
        f.write('{"t_utc": "2026-10-29T03:1')
    assert c.get("/api/hypotheses").json()["available"] is True
    empty = tmp_path_factory.mktemp("empty")
    monkeypatch.setenv("ARIADNE_LIVE", f"{empty}:/nonexistent/scene")
    d = c.get("/api/hypotheses").json()
    assert d == {"available": False, "hypotheses": [], "counts": {s: 0 for s in d["counts"]}, "changes": [], "decision": None,
                 "vl": None}


def test_recorded_file_mode(run, monkeypatch):
    root, c = run
    monkeypatch.delenv("ARIADNE_LIVE")
    monkeypatch.setenv("ARIADNE_HYPOTHESES", str(root / "hypotheses.jsonl"))
    d = c.get("/api/hypotheses").json()
    assert d["available"] and len(d["hypotheses"]) == len(last_status(FIXTURE)) and d["decision"]["model"]


def test_decision_found_behind_a_long_trace_line(run):
    root, c = run
    with open(next(root.glob("*/trace.jsonl")), "a") as f:   # newer records without an orchestrator answer, > 256 KiB
        f.write(json.dumps({"t_s": 15, "choice": {"source": "classical"}, "pad": "x" * (600 << 10)}) + "\n")
    assert c.get("/api/hypotheses").json()["decision"]["decision_id"] == "t10000ms"


def test_cache_follows_the_file(run):
    root, c = run
    assert c.get("/api/hypotheses").json()["counts"]["expired"] == 0
    with open(root / "hypotheses.jsonl", "a") as f:
        f.write(json.dumps({"sim_t_s": 70, "event": "status", "hypothesis_id": "h001", "agent_id": 0,
                            "status": "expired", "status_source": "verifier", "provenance": {}}) + "\n")
    assert c.get("/api/hypotheses").json()["counts"]["expired"] == 1


G2 = Path(__file__).resolve().parent / "fixtures/g2_b93e"


def test_real_g2_run_layout(tmp_path, monkeypatch):
    """Regression: on G2's real lines the endpoint returned HTTP 500 ('str' object has no attribute 'get'). The second
    agent's choice record carries answer = "see agent 0's record"; at t = 95 s the first agent's is a fallback that still
    carries the failed attempt's orchestrator block. The run-level vl_call events have hypothesis_id null."""
    run = tmp_path / "live-20261028T214744Z-b93e"
    shutil.copytree(G2, run, ignore=shutil.ignore_patterns("README.md"))
    monkeypatch.delenv("ARIADNE_HYPOTHESES", raising=False)
    monkeypatch.setenv("ARIADNE_LIVE", f"{run}:/nonexistent/scene")
    app = FastAPI()
    app.include_router(router)
    r = TestClient(app).get("/api/hypotheses")
    assert r.status_code == 200
    d = r.json()
    events = [json.loads(x) for x in (G2 / "hypotheses.jsonl").read_text().splitlines()]
    want = {}
    for e in events:
        if e.get("hypothesis_id"):
            want[e["hypothesis_id"]] = e["status"]
    assert {h["hypothesis_id"]: h["status"] for h in d["hypotheses"]} == want and None not in want
    dec = d["decision"]
    assert (dec["t_s"], dec["decision_id"], dec["source"], dec["model"]) == (95, "t95000ms", "fallback",
                                                                             "nvidia/Nemotron-3_5-Lightning")
    vl = next(e for e in reversed(events) if e["event"] == "vl_call")
    assert d["vl"]["model"] == vl["orchestrator"]["model"] == "nvidia/Nemotron-3.5-Super-VL"
    assert d["vl"]["latency_s"] == vl["orchestrator"]["latency_ms"] / 1000


@pytest.fixture
def g2(tmp_path, monkeypatch):
    run = tmp_path / "live-20261028T214744Z-b93e"
    shutil.copytree(G2, run, ignore=shutil.ignore_patterns("README.md"))
    monkeypatch.delenv("ARIADNE_HYPOTHESES", raising=False)
    monkeypatch.setenv("ARIADNE_LIVE", f"{run}:/nonexistent/scene")
    app = FastAPI()
    app.include_router(router)
    return run, TestClient(app)


def test_contradiction_surfaced_status_kept(g2):
    """G2: h001 was confirmed, then the map revised and the verifier disagreed on every replan from 40 s (older runs log
    one contradiction per replan). The status stays confirmed; the card shows since when and why."""
    _, c = g2
    h = next(h for h in c.get("/api/hypotheses").json()["hypotheses"] if h["hypothesis_id"] == "h001")
    assert h["status"] == "confirmed"
    assert h["contradicted"] == {"since_t_s": 40, "verdict": "refuted",
                                 "reason": "observed obstacles close every crossing at 0.40 m clearance (unknown treated as free)"}
    assert [s["status"] for s in h["history"]][-1] == "confirmed"      # a contradiction is not a status change


def test_contradiction_cleared(g2):
    run, c = g2
    with open(run / "hypotheses.jsonl", "a") as f:
        f.write(json.dumps({"sim_t_s": 100, "event": "contradiction_cleared", "hypothesis_id": "h001", "agent_id": 0,
                            "status": "confirmed", "status_source": "none", "provenance": {}}) + "\n")
    h = next(h for h in c.get("/api/hypotheses").json()["hypotheses"] if h["hypothesis_id"] == "h001")
    assert h["contradicted"] is None and h["status"] == "confirmed"


def test_panel_as_of_mission_time(g2):
    """?t= (the page's own mission clock): a console replaying a finished run shows the panel as it stood then."""
    _, c = g2
    early = c.get("/api/hypotheses", params={"t": 39}).json()
    h001 = next(h for h in early["hypotheses"] if h["hypothesis_id"] == "h001")
    assert h001["contradicted"] is None and h001["status"] == "confirmed"
    assert all(ch["sim_t_s"] <= 39 for ch in early["changes"])
    assert early["decision"] is None                          # the fixture's trace starts at 90 s
    assert early["vl"]["sim_t_s"] <= 39
    at90 = c.get("/api/hypotheses", params={"t": 90}).json()["decision"]
    assert (at90["decision_id"], at90["source"]) == ("t90000ms", "orchestrator")
    assert c.get("/api/hypotheses", params={"t": 95}).json()["decision"]["source"] == "fallback"
    assert c.get("/api/hypotheses", params={"t": 0}).json()["hypotheses"] == []
