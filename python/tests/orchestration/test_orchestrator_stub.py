"""The orchestrator's Bedrock tool loop (providers/bedrock.py, configs/llm/bedrock.yaml; D310) with Bedrock stubbed (botocore Stubber; no network, no spend), against the real
MCP servers over stdio (design agentic.md D156-D160). Runs in ariadne-orchestrator:1:

  docker run --rm -v "$PWD":/work:ro -w /work/python ariadne-orchestrator:nemotron-1 python -m pytest -q -p no:cacheprovider \
      tests/orchestration/test_orchestrator_stub.py

1. Nova inspects, asks for approval of an above-threshold candidate (the console's verdict file is already there),
   assigns: the answer carries the assignment and approval id; the trace has every tool call; the model's wrong
   decision_id is overridden.
2. Nova is throttled: the turn restarts on DeepSeek V3.2, whose tool list has no observe_image (text-only model).
3. The model never assigns: the answer is null and there is NO fallback (a bad answer is not a Bedrock failure;
   the C++ side falls back to the classical choice).
"""
import asyncio
import json
from pathlib import Path

import boto3
from botocore.stub import ANY, Stubber
from mcp.client.client import Client

from orchestration.orchestrator import Chooser, decide, mcp_params
from orchestration.providers import load_config, make_provider

CFG = load_config(Path(__file__).resolve().parents[3] / "configs" / "llm" / "bedrock.yaml")
PRIMARY, FALLBACK = (s["model"] for s in CFG["decision"]["chain"])


async def _chooser(ddir: Path, mm, tx, br, max_calls=10) -> Chooser:
    """What serve() builds, with the Bedrock client injected."""
    prov = make_provider("bedrock", CFG, client=br)
    mcps = {"multimodal": mm, "text": tx}
    cfgs = {("bedrock", k): prov.tool_config((await c.list_tools()).tools) for k, c in mcps.items()}
    return Chooser(ddir, mcps, {"bedrock": prov}, CFG["decision"]["chain"], cfgs, max_calls)


def _request(tmp: Path) -> dict:
    def cand(cluster, risk, needs):
        return {"cluster": cluster, "goal_xy_m": [1.0 + cluster, 2.0], "path_m": 2.5, "cells": 40, "standoff_m": 0.6,
                "risk": risk, "risk_reason": "narrowest gap 0.5 m", "needs_approval": needs}
    est = {"frames": 100, "keyframes": 12, "s_since_keyframe": 0.4}
    return {"decision_id": "t5000ms", "t_s": 5.0, "autonomous_threshold": 0.3, "approvals_dir": str(tmp / "approvals"),
            "map_png": str(tmp / "map.png"), "egress": {"found": False},
            "agents": [{"agent": 0, "vision": {"free_cells": 900}, "estimator": est, "classical_choice": 0,
                        "candidates": [cand(1, 0.1, False), cand(2, 0.6, True)]},
                       {"agent": 1, "vision": {"free_cells": 700}, "estimator": est, "classical_choice": 0,
                        "candidates": [cand(3, 0.2, False)]}]}


def _say(*uses, text=""):
    content = ([{"text": text}] if text else []) + [{"toolUse": {"toolUseId": f"u{i}", "name": n, "input": a}}
                                                    for i, (n, a) in enumerate(uses)]
    return {"output": {"message": {"role": "assistant", "content": content}}, "stopReason": "tool_use" if uses else "end_turn",
            "usage": {"inputTokens": 1000, "outputTokens": 50, "totalTokens": 1050}, "metrics": {"latencyMs": 100}}


def _expect(model):
    return {"modelId": model, "system": ANY, "messages": ANY, "toolConfig": ANY, "inferenceConfig": ANY}


def _run(tmp: Path, script, max_calls=10):
    """Script the Bedrock client, open both MCP servers as serve() does, decide one request."""
    ddir = tmp / "decisions"
    ddir.mkdir()
    br = boto3.client("bedrock-runtime", region_name="eu-north-1", aws_access_key_id="x", aws_secret_access_key="x")

    async def go():
        async with Client(mcp_params(ddir, False)) as mm, Client(mcp_params(ddir, True)) as tx:
            ch = await _chooser(ddir, mm, tx, br, max_calls)
            with Stubber(br) as st:
                script(st)
                ans = await decide(_request(tmp), ch)
                st.assert_no_pending_responses()
            return ans, {k: v for (_, k), v in ch.cfgs.items()}
    return asyncio.run(go())


def test_inspect_approve_assign(tmp_path):
    (tmp_path / "approvals").mkdir()
    (tmp_path / "approvals" / "t5000ms-a0-c1.json").write_text(json.dumps({"verdict": "approved", "operator": "test"}))
    asg = [{"agent": 0, "candidate": 1, "reason": "bigger cluster", "approval_id": "t5000ms-a0-c1"},
           {"agent": 1, "candidate": 0, "reason": "only one"}]

    def script(st):
        st.add_response("converse", _say(("list_candidates", {"decision_id": "bogus"}), text="Looking."), _expect(PRIMARY))
        st.add_response("converse", _say(("request_human_approval", {"agent": 0, "candidate": 1, "reason": "bigger"})),
                        _expect(PRIMARY))
        st.add_response("converse", _say(("assign", {"assignments": asg})), _expect(PRIMARY))
    ans, cfgs = _run(tmp_path, script)
    o = ans["orchestrator"]
    assert ans["decision_id"] == "t5000ms"
    assert [(a["agent"], a["candidate"], a["approval_id"]) for a in ans["assignments"]] == [(0, 1, "t5000ms-a0-c1"), (1, 0, "")]
    assert [t["tool"] for t in o["tool_calls"]] == ["list_candidates", "request_human_approval", "assign"]
    assert all(t["status"] == "success" for t in o["tool_calls"])
    assert '"autonomous_threshold": 0.3' in o["tool_calls"][0]["result"]   # the real request, despite decision_id "bogus"
    assert '"approved"' in o["tool_calls"][1]["result"]
    packet = json.loads((tmp_path / "approvals" / "t5000ms-a0-c1.request.json").read_text())
    assert packet["risk_score"] == 0.6 and packet["orchestrator_reason"] == "bigger"
    assert (o["model"], o["fallback_used"], o["calls"], o["input_tokens"], o["output_tokens"]) == (PRIMARY, False, 3, 3000, 150)
    assert o["model_text"] == ["Looking."]
    names = {k: [t["toolSpec"]["name"] for t in c["tools"]] for k, c in cfgs.items()}
    assert "observe_image" in names["multimodal"] and "observe_image" not in names["text"]


def test_throttled_primary_falls_back_to_text_model(tmp_path):
    asg = [{"agent": 0, "candidate": 0, "reason": "a"}, {"agent": 1, "candidate": 0, "reason": "b"}]

    def script(st):
        st.add_client_error("converse", service_error_code="ThrottlingException", http_status_code=429,
                            expected_params=_expect(PRIMARY))
        st.add_response("converse", _say(("assign", {"assignments": asg})), _expect(FALLBACK))
    ans, _ = _run(tmp_path, script)
    o = ans["orchestrator"]
    assert [a["candidate"] for a in ans["assignments"]] == [0, 0]
    assert (o["model"], o["fallback_used"], o["calls"]) == (FALLBACK, True, 1)
    assert len(o["errors"]) == 1 and "ThrottlingException" in o["errors"][0]


def test_no_assign_is_not_a_fallback(tmp_path):
    def script(st):
        st.add_response("converse", _say(text="I think agent 0 should go north."), _expect(PRIMARY))
        st.add_response("converse", _say(text="Still thinking."), _expect(PRIMARY))
    ans, _ = _run(tmp_path, script, max_calls=2)
    o = ans["orchestrator"]
    assert ans["assignments"] is None
    assert (o["model"], o["fallback_used"], o["calls"], o["errors"]) == (PRIMARY, False, 2, [])


def test_oversized_request_line_is_answered(tmp_path, monkeypatch):
    # Run 4 (2026-10-23): the request crossed asyncio's default 64 KiB line limit, readline raised LimitOverrunError
    # outside the per-line guard, and the runtime lost its socket. The server's handler must take a 300 KiB line.
    import orchestration.orchestrator as orch

    async def fake_decide(req, *a):
        return {"decision_id": req["decision_id"], "assignments": []}
    monkeypatch.setattr(orch, "decide", fake_decide)
    sock = str(tmp_path / "s.sock")

    async def go():
        # no `async with srv`: 3.12's __aexit__ awaits wait_closed(), which waits for every client transport
        srv = await asyncio.start_unix_server(orch.make_handler(None), path=sock,
                                              limit=orch.LINE_LIMIT)
        reader, writer = await asyncio.open_unix_connection(sock, limit=orch.LINE_LIMIT)
        writer.write((json.dumps({"decision_id": "big", "pad": "x" * 300_000}) + "\n").encode())
        await writer.drain()
        ans = json.loads(await reader.readline())
        writer.close()
        srv.close()
        return ans
    ans = asyncio.run(go())
    assert ans["decision_id"] == "big" and ans["assignments"] == []


def test_any_failure_answers_at_once(tmp_path):
    # Expired credentials raise RuntimeError, not a ClientError (live run 3, 2026-10-23): decide must still answer
    # (null assignment, error logged), so the runtime falls back now instead of waiting out its timeout.
    ddir = tmp_path / "decisions"
    ddir.mkdir()

    class Broken:
        def converse(self, **kw):
            raise RuntimeError("Credentials were refreshed, but the refreshed credentials are still expired.")

    async def go():
        async with Client(mcp_params(ddir, False)) as mm, Client(mcp_params(ddir, True)) as tx:
            return await decide(_request(tmp_path), await _chooser(ddir, mm, tx, Broken()))
    ans = asyncio.run(go())
    assert ans["assignments"] is None
    assert "RuntimeError" in ans["orchestrator"]["errors"][0] and not ans["orchestrator"]["fallback_used"]
