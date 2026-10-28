"""The provider seam (D310): one vendor's tool loop over the MCP tools, for one decision request.

A provider owns its message format, so the whole loop lives here, not just the HTTP call: Bedrock Converse and the
OpenAI chat format disagree on where tool results and images go. What every provider shares is the MCP side
(call_tool) and the log record (new_log) that ends up in trace.jsonl as choice.answer.orchestrator.

run_turn returns True once `assign` recorded a choice (<did>.choice.json exists). It RAISES one of
`transport_errors` when the vendor could not be reached (the orchestrator then tries the next model in the chain);
any other outcome (no assign, a truncated answer, a refused budget) is returned or raised as something else and never
triggers a model fallback.
"""
from __future__ import annotations

import base64
import hashlib
from pathlib import Path

NUDGE = "Call a tool; finish with assign."


def new_log() -> dict:
    return {"provider": None, "model": None, "fallback_used": False, "calls": 0, "latency_s": 0.0, "input_tokens": 0,
            "output_tokens": 0, "reasoning_tokens": 0, "est_usd": 0.0, "tool_calls": [], "model_text": [],
            "stop": None, "errors": []}


def first_prompt(req: dict) -> str:
    return (f"decision_id={req['decision_id']}. Mission time {req['t_s']:.1f} s. Agents to assign: "
            f"{[a['agent'] for a in req['agents']]}. Inspect, then assign.")


async def call_tool(mcp, name: str, args: dict, did: str) -> tuple[str, str, list[tuple[str, bytes]], dict]:
    """Run one MCP tool for this decision. -> (status, text, images [(mime, raw)], trace entry). decision_id is always
    this request's, whatever the model passed. Every failure goes back to the model and into the trace."""
    args = dict(args or {})
    args["decision_id"] = did
    entry = {"tool": name, "input": {k: v for k, v in args.items() if k != "decision_id"}}
    text, images = [], []
    try:
        r = await mcp.call_tool(name, args)
        for c in r.content:
            if getattr(c, "type", "") == "image":
                images.append((c.mime_type, base64.b64decode(c.data)))
            else:
                text.append(c.text)
        status = "error" if r.is_error else "success"
    except Exception as e:  # noqa: BLE001
        text, status = [f"{type(e).__name__}: {e}"], "error"
    t = "".join(text)
    entry.update(status=status, result=t[:600],
                 images=[{"sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw), "format": m.split("/")[-1]}
                         for m, raw in images])
    return status, t, images, entry


def chosen(ddir: Path, did: str) -> bool:
    return (ddir / f"{did}.choice.json").exists()


class Provider:
    name = ""
    transport_errors: tuple[type[BaseException], ...] = ()

    def tool_config(self, tools) -> object:
        raise NotImplementedError

    async def run_turn(self, req: dict, ddir: Path, mcp, tools_cfg, spec: dict, system: str, max_calls: int,
                       log: dict) -> bool:
        raise NotImplementedError
