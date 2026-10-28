"""Nebius Token Factory provider (D310, D313): NVIDIA Nemotron over the OpenAI-compatible chat API (vLLM-served).

The tool loop in the OpenAI chat format:
- tools are `{"type": "function", "function": {name, description, parameters}}` built from the MCP tool list;
- the model answers with `message.tool_calls`; each one MUST get a `role: tool` reply with its `tool_call_id`
  before the next request (else HTTP 400);
- a tool message carries text only, so an image a tool returned (observe_image) goes in ONE following `role: user`
  message as `image_url` data URIs, and only when the model is image-capable (spec.images);
- the assistant turn is appended back as {role, content, tool_calls} only: vendor fields (`reasoning`,
  `reasoning_content`) are not resent;
- tool_choice "auto" + the inherited nudge when a turn has no tool call (`required` is not relied on);
- reasoning: the decision tier turns it off per model through `extra_body` (Super 120B: chat_template_kwargs
  enable_thinking false; Super-VL: reasoning_effort none; D301). Reasoning tokens count against max_tokens, so a
  reasoning model with a small cap can stop on `length` with no tool call: logged, and NOT a fallback trigger.

Every call is checked against and written to the spend ledger (ledger.py) first and after. The key comes from the
environment variable named in the config (NEBIUS_API_KEY); it is never logged, and its absence is an error that names
the variable only.
"""
from __future__ import annotations

import asyncio
import base64
import json
import os
import re
import time
from pathlib import Path

from .base import NUDGE, Provider, call_tool, chosen, first_prompt
from .ledger import Ledger, ledger_path

THINK = re.compile(r"<think>.*?(</think>|$)", re.S)


def make_ledger(cfg: dict) -> Ledger:
    b = cfg.get("budget", {})
    return Ledger(ledger_path(b.get("ledger")), cfg.get("prices_usd_per_mtok", {}), b.get("usd_hard", 40.0),
                  b.get("usd_warn", 25.0))


class NebiusProvider(Provider):
    name = "nebius"

    def __init__(self, cfg: dict, client=None, ledger: Ledger | None = None, purpose: str = "live"):
        import openai
        self.transport_errors = (openai.APIConnectionError, openai.APITimeoutError, openai.RateLimitError,
                                 openai.InternalServerError)
        self.timeout_s = float(cfg.get("timeout_s", 45.0))
        if client is None:
            var = cfg.get("api_key_env", "NEBIUS_API_KEY")
            key = os.environ.get(var, "").strip()
            if not key:
                raise RuntimeError(f"nebius: environment variable {var} is empty (pass the key file with --env-file)")
            client = openai.OpenAI(base_url=cfg["base_url"], api_key=key, max_retries=int(cfg.get("max_retries", 1)),
                                   timeout=self.timeout_s)
        self.client = client
        self.ledger = ledger or make_ledger(cfg)
        self.purpose = purpose
        self.record_dir = Path(os.environ["ARIADNE_LLM_RECORD"]) if os.environ.get("ARIADNE_LLM_RECORD") else None

    def tool_config(self, tools) -> list:
        return [{"type": "function", "function": {"name": t.name, "description": t.description or "",
                                                  "parameters": t.input_schema}} for t in tools]

    def chat(self, spec: dict, messages: list, tools: list | None, purpose: str):
        """One paid call: budget check, request, ledger line. -> (response, latency_s, est_usd)."""
        self.ledger.check()
        kw = {"model": spec["model"], "messages": messages, "max_tokens": int(spec.get("max_tokens", 512)),
              "temperature": float(spec.get("temperature", 0.0)), "timeout": self.timeout_s}
        if tools:
            kw["tools"], kw["tool_choice"] = tools, spec.get("tool_choice", "auto")
        if spec.get("extra_body"):
            kw["extra_body"] = spec["extra_body"]
        t0 = time.time()
        resp = self.client.chat.completions.create(**kw)
        dt = time.time() - t0
        u = resp.usage
        usd = self.ledger.record(spec["model"], getattr(u, "prompt_tokens", 0) or 0,
                                 getattr(u, "completion_tokens", 0) or 0, purpose, dt)
        if self.record_dir is not None:   # fixture recording: the response body only (no headers, no key)
            self.record_dir.mkdir(parents=True, exist_ok=True)
            n = len(list(self.record_dir.glob("*.json")))
            (self.record_dir / f"{n:03d}_{spec['model'].split('/')[-1]}.json").write_text(
                json.dumps({"request": {k: v for k, v in kw.items() if k not in ("messages", "timeout")},
                            "n_messages": len(messages), "latency_s": round(dt, 3),
                            "response": resp.model_dump() if hasattr(resp, "model_dump") else resp}, indent=1))
        return resp, dt, usd

    async def run_turn(self, req: dict, ddir: Path, mcp, tools_cfg, spec: dict, system: str, max_calls: int,
                       log: dict) -> bool:
        did, model = req["decision_id"], spec["model"]
        msgs = [{"role": "system", "content": system}, {"role": "user", "content": first_prompt(req)}]
        while log["calls"] < max_calls:
            resp, _, usd = await asyncio.to_thread(self.chat, spec, msgs, tools_cfg, f"{self.purpose}:{did}")
            log["calls"] += 1
            log["provider"], log["model"] = self.name, model
            u = resp.usage
            log["input_tokens"] += getattr(u, "prompt_tokens", 0) or 0
            log["output_tokens"] += getattr(u, "completion_tokens", 0) or 0
            det = getattr(u, "completion_tokens_details", None)
            log["reasoning_tokens"] += (getattr(det, "reasoning_tokens", 0) or 0) if det else 0
            log["est_usd"] = round(log["est_usd"] + usd, 8)
            ch = resp.choices[0]
            msg = ch.message
            log["stop"] = ch.finish_reason
            text = msg.content or ""
            if "<think>" in text:
                log["errors"].append("reasoning text in content (thinking not off)")
                text = THINK.sub("", text)
            if text.strip():
                log["model_text"].append(text.strip()[:400])
            calls = list(msg.tool_calls or [])
            out = {"role": "assistant", "content": text or None}
            if calls:
                out["tool_calls"] = [{"id": c.id, "type": "function",
                                      "function": {"name": c.function.name, "arguments": c.function.arguments or "{}"}}
                                     for c in calls]
            msgs.append(out)
            if not calls:
                if ch.finish_reason == "length":
                    log["errors"].append(f"{model}: finish_reason length with no tool call (max_tokens "
                                         f"{spec.get('max_tokens', 512)})")
                msgs.append({"role": "user", "content": NUDGE})
                continue
            parts = []
            for c in calls:
                try:
                    args = json.loads(c.function.arguments or "{}")
                    if not isinstance(args, dict):
                        raise ValueError("arguments must be a JSON object")
                except ValueError as e:
                    entry = {"tool": c.function.name, "input": c.function.arguments[:300], "status": "error",
                             "result": f"invalid arguments: {e}", "images": []}
                    log["tool_calls"].append(entry)
                    msgs.append({"role": "tool", "tool_call_id": c.id, "content": entry["result"]})
                    continue
                status, ttext, images, entry = await call_tool(mcp, c.function.name, args, did)
                log["tool_calls"].append(entry)
                body = ttext or ("(image follows)" if images else "(empty)")
                msgs.append({"role": "tool", "tool_call_id": c.id,
                             "content": body if status == "success" else f"error: {body}"})
                if spec.get("images"):
                    parts += [{"type": "image_url", "image_url": {
                        "url": f"data:{m};base64,{base64.b64encode(raw).decode()}"}} for m, raw in images]
            if parts:
                msgs.append({"role": "user", "content": [{"type": "text", "text": "Images returned by the tools:"}]
                             + parts})
            if chosen(ddir, did):
                return True
        return chosen(ddir, did)
