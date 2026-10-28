"""The Nebius Token Factory provider (providers/nebius.py, D310-D313) and the spend ledger (D312), offline: the OpenAI
client is replaced by a scripted fake that returns RECORDED Token Factory responses (fixtures/loop_raw, recorded
2026-10-29 by `orchestration.bench --n 1` with ARIADNE_LLM_RECORD; fixtures/probe_raw by record_probe.py) or
synthetic ones in the same vLLM shape, against the real MCP servers over stdio. No network, no spend.

  docker run --rm -v "$PWD":/work:ro -w /work/python ariadne-orchestrator:nemotron-1 python -m pytest -q \\
      -p no:cacheprovider tests/orchestration/test_nebius_provider.py
"""
import asyncio
import json
import struct
import zlib
from pathlib import Path

import httpx2
import openai
import pytest
from mcp.client.client import Client
from openai.types.chat import ChatCompletion

from orchestration.bench import rebase, score
from orchestration.orchestrator import Chooser, decide, mcp_params
from orchestration.providers import BudgetExceeded, load_config, make_provider
from orchestration.providers.ledger import Ledger

REPO = Path(__file__).resolve().parents[3]
FIX = Path(__file__).parent / "fixtures"
CORPUS = REPO / "evaluation" / "runs" / "live-agentic" / "live_loop" / "llm_live_run5_maze_short_90s"
CFG = load_config(REPO / "configs" / "llm" / "nemotron.yaml")
SUPER, LIGHTNING = (s["model"] for s in CFG["decision"]["chain"])
PRICES = CFG["prices_usd_per_mtok"]


def recorded(prefix: str) -> list[dict]:
    return [json.loads(p.read_text())["response"] for p in sorted((FIX / "loop_raw").glob(f"*_{prefix}.json"))]


def synth(tool_calls=(), content=None, finish="tool_calls", pt=1000, ct=50, model=SUPER) -> dict:
    """A chat.completion in the shape Token Factory (vLLM) returned at G0 (g0_calls.json)."""
    return {"id": "chatcmpl-x", "object": "chat.completion", "created": 0, "model": model,
            "usage": {"prompt_tokens": pt, "completion_tokens": ct, "total_tokens": pt + ct,
                      "completion_tokens_details": {"reasoning_tokens": 0}},
            "choices": [{"index": 0, "finish_reason": finish, "message": {
                "role": "assistant", "content": content,
                "tool_calls": [{"id": f"call-{i}", "type": "function",
                                "function": {"name": n, "arguments": json.dumps(a)}} for i, (n, a) in enumerate(tool_calls)]
                or None}}]}


class FakeOpenAI:
    """client.chat.completions.create(**kw) -> the next scripted response (dict) or raises it (exception). Checks the
    OpenAI protocol rule the provider must keep: every tool_call id is answered by a role:tool message."""

    def __init__(self, script):
        self.script, self.sent = list(script), []
        self.chat = self
        self.completions = self

    def create(self, **kw):
        msgs = kw["messages"]
        want = [c["id"] for m in msgs if m["role"] == "assistant" for c in m.get("tool_calls") or []]
        got = [m["tool_call_id"] for m in msgs if m["role"] == "tool"]
        assert want == got, f"unanswered tool calls: {want} vs {got}"
        assert all(set(m) <= {"role", "content", "tool_calls"} for m in msgs if m["role"] == "assistant")
        self.sent.append(json.loads(json.dumps(kw)))
        nxt = self.script.pop(0)
        if isinstance(nxt, Exception):
            raise nxt
        return ChatCompletion.model_validate(nxt)


def _conn_error():
    return openai.APIConnectionError(request=httpx2.Request("POST", "https://api.tokenfactory.nebius.com/v1/chat/completions"))


def _request(tmp: Path) -> dict:
    return rebase(json.loads((CORPUS / "decisions" / "t5000ms.json").read_text()), CORPUS, tmp)


def _run(tmp: Path, script, chain=None, ledger_lines=(), max_calls=10, req=None):
    ddir = tmp / "decisions"
    ddir.mkdir()
    led = Ledger(tmp / "spend.jsonl", PRICES, 40.0, 25.0)
    for line in ledger_lines:
        with open(led.path, "a") as f:
            f.write(json.dumps(line) + "\n")
    fake = FakeOpenAI(script)
    prov = make_provider("nebius", CFG, client=fake, ledger=led)
    chain = chain or CFG["decision"]["chain"]

    async def go():
        async with Client(mcp_params(ddir, False)) as mm, Client(mcp_params(ddir, True)) as tx:
            mcps = {"multimodal": mm, "text": tx}
            cfgs = {("nebius", k): prov.tool_config((await c.list_tools()).tools) for k, c in mcps.items()}
            return await decide(req or _request(tmp), Chooser(ddir, mcps, {"nebius": prov}, chain, cfgs, max_calls))
    return asyncio.run(go()), fake, led


def test_recorded_super_loop_assigns(tmp_path):
    rec = recorded("nemotron-3-super-120b-a12b")
    ans, fake, led = _run(tmp_path, rec)
    o = ans["orchestrator"]
    assert [t["tool"] for t in o["tool_calls"]] == ["list_candidates", "assign"]
    assert ans["assignments"] and {a["agent"] for a in ans["assignments"]} == {0, 1}
    assert (o["provider"], o["model"], o["calls"], o["fallback_used"], o["errors"]) == ("nebius", SUPER, 2, False, [])
    assert o["input_tokens"] == sum(r["usage"]["prompt_tokens"] for r in rec)
    assert o["reasoning_tokens"] == 0
    # the request carried the verified decision-tier flags, and the text-only tool list (no observe_image)
    assert fake.sent[0]["extra_body"] == {"chat_template_kwargs": {"enable_thinking": False}}
    assert "observe_image" not in [t["function"]["name"] for t in fake.sent[0]["tools"]]
    assert fake.sent[0]["tool_choice"] == "auto" and fake.sent[0]["max_tokens"] == 512
    lines = [json.loads(x) for x in led.path.read_text().splitlines()]
    assert [x["model"] for x in lines] == [SUPER, SUPER] and lines[0]["purpose"] == "live:t5000ms"
    assert o["est_usd"] == pytest.approx(sum(x["est_usd"] for x in lines))
    assert score(_request(tmp_path), ans["assignments"])["agents"] == {0: "ok", 1: "ok"}


def test_recorded_lightning_and_nano_loops_assign(tmp_path):
    for i, (name, model) in enumerate((("Nemotron-3_5-Lightning", LIGHTNING),
                                       ("NVIDIA-Nemotron-3-Nano-30B-A3B", "nvidia/NVIDIA-Nemotron-3-Nano-30B-A3B"))):
        spec = {**CFG["decision"]["chain"][1], "model": model}
        rec = recorded(name)
        (tmp_path / str(i)).mkdir()
        ans, _, _ = _run(tmp_path / str(i), rec, chain=[spec])
        assert ans["assignments"] and ans["orchestrator"]["calls"] == len(rec)
        assert ans["orchestrator"]["tool_calls"][-1]["tool"] == "assign"


def test_transport_error_falls_back_to_next_model(tmp_path):
    asg = [{"agent": 0, "candidate": 0, "reason": "a"}, {"agent": 1, "candidate": 0, "reason": "b"}]
    ans, fake, led = _run(tmp_path, [_conn_error(), synth([("assign", {"assignments": asg})], model=LIGHTNING)])
    o = ans["orchestrator"]
    assert [a["candidate"] for a in ans["assignments"]] == [0, 0]
    assert (o["model"], o["fallback_used"], o["calls"]) == (LIGHTNING, True, 1)
    assert "APIConnectionError" in o["errors"][0]
    assert [s["model"] for s in fake.sent] == [SUPER, LIGHTNING]
    assert len(led.path.read_text().splitlines()) == 1     # the failed call cost nothing and is not on the ledger


def test_budget_exceeded_refuses_without_call_or_fallback(tmp_path):
    full = [{"ts_utc": "x", "model": SUPER, "prompt_tokens": 0, "completion_tokens": 0, "est_usd": 40.0}]
    ans, fake, _ = _run(tmp_path, [], ledger_lines=full)
    o = ans["orchestrator"]
    assert ans["assignments"] is None and fake.sent == []
    assert o["errors"] == ["budget_exceeded"] and not o["fallback_used"] and o["calls"] == 0


def test_no_tool_call_and_length_are_not_fallbacks(tmp_path):
    ans, fake, _ = _run(tmp_path, [synth(content="Agent 0 should go north.", finish="stop"),
                                   synth(content=None, finish="length")], max_calls=2)
    o = ans["orchestrator"]
    assert ans["assignments"] is None and not o["fallback_used"] and o["calls"] == 2
    assert o["model_text"] == ["Agent 0 should go north."] and "finish_reason length" in o["errors"][0]
    assert fake.sent[1]["messages"][-1] == {"role": "user", "content": "Call a tool; finish with assign."}
    assert {s["model"] for s in fake.sent} == {SUPER}


def test_bad_arguments_and_think_text_are_handled(tmp_path):
    bad = synth()
    bad["choices"][0]["message"]["tool_calls"] = [{"id": "c0", "type": "function",
                                                   "function": {"name": "list_candidates", "arguments": "{not json"}}]
    bad["choices"][0]["message"]["content"] = "<think>hmm</think>Checking."
    asg = [{"agent": 0, "candidate": 0}, {"agent": 1, "candidate": 0}]
    ans, fake, _ = _run(tmp_path, [bad, synth([("assign", {"assignments": asg})])])
    o = ans["orchestrator"]
    assert ans["assignments"] and o["tool_calls"][0]["status"] == "error"
    assert o["model_text"] == ["Checking."] and any("reasoning text" in e for e in o["errors"])
    assert fake.sent[1]["messages"][3] == {"role": "tool", "tool_call_id": "c0",
                                           "content": "invalid arguments: Expecting property name enclosed in double "
                                                      "quotes: line 1 column 2 (char 1)"}


def _png(path: Path) -> None:
    raw = b"\x00\x80"   # 1x1 8-bit grey: filter byte 0, one pixel
    chunk = lambda t, d: struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d))   # noqa: E731
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 0, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def test_images_follow_tool_messages_for_vl_only(tmp_path):
    req = _request(tmp_path)
    req["map_png"] = str(tmp_path / "map.png")
    _png(tmp_path / "map.png")
    asg = [{"agent": 0, "candidate": 0}, {"agent": 1, "candidate": 0}]
    vl = {"provider": "nebius", "model": "nvidia/Nemotron-3.5-Super-VL", "images": True, "max_tokens": 512}
    ans, fake, _ = _run(tmp_path, [synth([("observe_image", {"what": "map"})]), synth([("assign", {"assignments": asg})])],
                        chain=[vl], req=req)
    assert ans["assignments"]
    msgs = fake.sent[1]["messages"]
    assert msgs[-2]["role"] == "tool" and msgs[-2]["content"] == "(image follows)"
    assert msgs[-1]["role"] == "user" and msgs[-1]["content"][1]["image_url"]["url"].startswith("data:image/png;base64,")
    assert ans["orchestrator"]["tool_calls"][0]["images"][0]["format"] == "png"


# --- the spend ledger (D312, FM requirement) ---

def test_ledger_sums_and_prices(tmp_path):
    led = Ledger(tmp_path / "l.jsonl", PRICES, 40.0, 25.0)
    assert led.total() == 0.0
    led.record(SUPER, 1_000_000, 0)            # 0.30
    led.record(LIGHTNING, 0, 1_000_000)        # 0.24
    assert led.total() == pytest.approx(0.54)
    assert (tmp_path / "l.jsonl").stat().st_mode & 0o777 == 0o600


def test_ledger_unknown_model_priced_at_the_most_expensive(tmp_path):
    led = Ledger(tmp_path / "l.jsonl", PRICES, 40.0, 25.0)
    assert led.price("nvidia/some-new-model") == (1.00, 3.00)    # Ultra's in and out
    assert led.est_usd("nvidia/some-new-model", 1_000_000, 1_000_000) == pytest.approx(4.0)


def test_ledger_refuses_at_cap_and_warns_once(tmp_path, capsys):
    led = Ledger(tmp_path / "l.jsonl", PRICES, 1.0, 0.5)
    led.record("nvidia/Nemotron-3-Ultra-550b-a55b", 600_000, 0)   # 0.60: past warn
    led.check()
    led.check()
    assert capsys.readouterr().out.count("WARNING") == 1
    led.record("nvidia/Nemotron-3-Ultra-550b-a55b", 400_000, 0)   # 1.00: at the cap
    with pytest.raises(BudgetExceeded):
        led.check()


def test_ledger_unreadable_line_stops_spending(tmp_path):
    (tmp_path / "l.jsonl").write_text("garbage\n")
    with pytest.raises(BudgetExceeded):
        Ledger(tmp_path / "l.jsonl", PRICES, 40.0, 25.0).check()


def test_ledger_needs_prices(tmp_path):
    with pytest.raises(ValueError):
        Ledger(tmp_path / "l.jsonl", {}, 40.0, 25.0)


def test_ledger_path_from_env(tmp_path, monkeypatch):
    from orchestration.providers.ledger import ledger_path
    monkeypatch.setenv("NEBIUS_SPEND_LEDGER", str(tmp_path / "x.jsonl"))
    assert ledger_path("~/elsewhere.jsonl") == tmp_path / "x.jsonl"


def test_missing_key_names_the_variable_only(monkeypatch):
    monkeypatch.delenv("NEBIUS_API_KEY", raising=False)
    with pytest.raises(RuntimeError, match="NEBIUS_API_KEY is empty"):
        make_provider("nebius", CFG)


def test_config_has_no_unpriced_chain_model():
    for s in CFG["decision"]["chain"] + [CFG["hypothesis"]]:
        assert s["model"] in PRICES


# --- the bench's replay hygiene and gate scoring (D318) ---

def test_bench_rebase_never_points_at_the_archive(tmp_path):
    r = _request(tmp_path)
    assert r["approvals_dir"] == str(tmp_path / "approvals")
    assert not list((tmp_path).glob("decisions/*.choice.json"))
    for a in r["agents"]:
        for i, c in enumerate(a["candidates"]):
            assert (tmp_path / "approvals" / f"t5000ms-a{a['agent']}-c{i}.json").exists() == bool(c["needs_approval"])


def test_bench_score_mirrors_the_gate():
    def cand(cl, risk):
        return {"cluster": cl, "risk": risk}
    req = {"decision_id": "t1ms", "autonomous_threshold": 0.5,
           "agents": [{"agent": 0, "candidates": [cand(1, 0.1), cand(2, 0.9)]},
                      {"agent": 1, "candidates": [cand(1, 0.1), cand(3, 0.1)]}]}
    assert score(req, None) == {"answered": False, "agents": {}}
    assert score(req, [{"agent": 0, "candidate": 5}, {"agent": 1, "candidate": -1}])["agents"] == {0: "out_of_set", 1: "ok_hold"}
    assert score(req, [{"agent": 0, "candidate": 1, "approval_id": ""}, {"agent": 1, "candidate": 1}])["agents"][0] == \
        "gate_rejected_no_approval"
    assert score(req, [{"agent": 0, "candidate": 1, "approval_id": "t1ms-a0-c1"}, {"agent": 1, "candidate": 1}])["agents"] \
        == {0: "ok", 1: "ok"}
    assert score(req, [{"agent": 0, "candidate": 0}, {"agent": 1, "candidate": 0}])["agents"][1] == "same_cluster"
