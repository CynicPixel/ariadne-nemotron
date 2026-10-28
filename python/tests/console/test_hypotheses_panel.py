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
    assert d == {"available": False, "hypotheses": [], "counts": {s: 0 for s in d["counts"]}, "changes": [], "decision": None}


def test_recorded_file_mode(run, monkeypatch):
    root, c = run
    monkeypatch.delenv("ARIADNE_LIVE")
    monkeypatch.setenv("ARIADNE_HYPOTHESES", str(root / "hypotheses.jsonl"))
    d = c.get("/api/hypotheses").json()
    assert d["available"] and len(d["hypotheses"]) == len(last_status(FIXTURE)) and d["decision"]["model"]
