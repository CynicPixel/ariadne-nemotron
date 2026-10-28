"""Bedrock Converse provider: the inherited loop (D84, D156-D160), moved out of orchestrator.py unchanged (D310).
Kept as an optional comparator only (CLAUDE.md §1). configs/llm/bedrock.yaml reproduces the old constants:
primary eu.amazon.nova-2-lite-v1:0 (text + image), fallback deepseek.v3.2 (text only), eu-north-1, temperature 0.
Credentials: the boto3 chain (instance role on AWS; live.sh's credential_process profile on the dev host).
"""
from __future__ import annotations

import asyncio
from pathlib import Path

from .base import NUDGE, Provider, call_tool, chosen, first_prompt


class BedrockProvider(Provider):
    name = "bedrock"

    def __init__(self, cfg: dict, client=None):
        from botocore.exceptions import BotoCoreError, ClientError
        self.transport_errors = (ClientError, BotoCoreError)
        if client is None:
            import boto3
            from botocore.config import Config
            client = boto3.client("bedrock-runtime", region_name=cfg.get("region", "eu-north-1"),
                                  config=Config(retries={"max_attempts": 2, "mode": "standard"}, read_timeout=60))
        self.br = client

    def tool_config(self, tools) -> dict:
        return {"tools": [{"toolSpec": {"name": t.name, "description": t.description or "",
                                        "inputSchema": {"json": t.input_schema}}} for t in tools],
                "toolChoice": {"any": {}}}

    async def run_turn(self, req: dict, ddir: Path, mcp, tools_cfg, spec: dict, system: str, max_calls: int,
                       log: dict) -> bool:
        did, model = req["decision_id"], spec["model"]
        msgs = [{"role": "user", "content": [{"text": first_prompt(req)}]}]
        while log["calls"] < max_calls:
            resp = await asyncio.to_thread(
                self.br.converse, modelId=model, system=[{"text": system}], messages=msgs, toolConfig=tools_cfg,
                inferenceConfig={"temperature": spec.get("temperature", 0.0), "maxTokens": spec.get("max_tokens", 600)})
            log["calls"] += 1
            log["provider"], log["model"] = self.name, model
            u = resp.get("usage", {})
            log["input_tokens"] += u.get("inputTokens", 0)
            log["output_tokens"] += u.get("outputTokens", 0)
            log["stop"] = resp.get("stopReason")
            out = resp["output"]["message"]
            msgs.append(out)
            for c in out["content"]:
                if "text" in c and c["text"].strip():
                    log["model_text"].append(c["text"][:400])
            uses = [c["toolUse"] for c in out["content"] if "toolUse" in c]
            if not uses:
                msgs.append({"role": "user", "content": [{"text": NUDGE}]})
                continue
            results = []
            for tu in uses:
                status, text, images, entry = await call_tool(mcp, tu["name"], tu.get("input"), did)
                log["tool_calls"].append(entry)
                content = [{"image": {"format": m.split("/")[-1], "source": {"bytes": raw}}} for m, raw in images]
                if text or not content:
                    content.insert(0, {"text": text})
                results.append({"toolResult": {"toolUseId": tu["toolUseId"], "status": status, "content": content}})
            msgs.append({"role": "user", "content": results})
            if chosen(ddir, did):
                return True
        return chosen(ddir, did)
