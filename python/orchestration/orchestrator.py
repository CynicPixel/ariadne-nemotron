"""The chooser's orchestrator in the CHOOSE step (design agentic.md §0, D156-D160; arch §25; fork D310-D317).

  python -m orchestration.orchestrator --socket RUN/chooser.sock --dir RUN/decisions [--llm-config configs/llm/nemotron.yaml]

ariadne_live (chooser: llm) connects to SOCKET and sends one JSON line per replan: a joint decision request for both
agents. For each, this process writes the request where the MCP server can read it, runs one provider's tool loop
whose tools are the MCP server's (stdio, mcp 2.3.0), and answers with one JSON line:
  {"decision_id", "assignments": [{agent, candidate (-1 = hold), reason, approval_id[, hypothesis_id, evidence_goal]}],
   "orchestrator": {...}}
`assignments` is whatever the `assign` tool recorded, or null. ariadne_live validates every assignment again, per
agent (range, exactly-once, approval for an above-threshold pick) and falls back to the classical choice otherwise.

Models (D311): the decision chain in --llm-config (default configs/llm/nemotron.yaml: Nemotron on Nebius Token Factory;
configs/llm/bedrock.yaml is the inherited Bedrock pair). The next model in the chain runs ONLY when the provider could
not be reached (its transport errors); a bad or empty answer, or a refused budget (D312), never falls back here: the
answer is null and the C++ side keeps the classical choice. A text-only model gets a second MCP server with
--text-only (no observe_image). Images are sent to the model as bytes and logged as path + sha256, never inline.
Before each turn, with hypotheses enabled, the slab verifier and the hypothesis store update (D315-D317).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

from mcp import StdioServerParameters
from mcp.client.client import Client

from .providers import BudgetExceeded, load_config, make_provider
from .providers.base import new_log

SYSTEM = (
    "You direct a team of search agents exploring a dark, partially collapsed building, mapped only from their stereo "
    "cameras. Each turn you assign each agent its next exploration target among candidates a deterministic planner "
    "computed. Use the tools to look at the map (and, if available, its image and the agents' camera keyframes), "
    "the agents' pose certainty, the responder egress route and the candidates; then call assign exactly once. "
    "Prefer targets that reveal more unknown space (larger clusters) per metre driven, do not send two agents to "
    "the same cluster when another is available, keep risk low, and remember the mission: find a route a human "
    "responder can use. A candidate whose risk exceeds the threshold needs request_human_approval first; if the "
    "verdict is not 'approved', pick another candidate or hold. "
    "Operator directives are the operator's orders and come before your own preferences: favour candidates inside an "
    "operator region for that agent (priority_weight > 1, or to_directive_m 0), else the ones nearer it. "
    "Changing an agent's goal costs a turn in place and the progress made so far: keep its current_task (its "
    "`candidate`) unless that task is reached or gone, an operator directive points elsewhere, or another candidate "
    "is clearly better, and say which of these when you switch. A candidate with supervisor_return true is the "
    "energy-reserve return ordered by the planner: take it (it needs no approval). Candidates come best-first by "
    "the planner's own utility (information gain net of costs; `rank` 0 = best): prefer the top-ranked ones, and do "
    "not pick a goal under 1 m away unless nothing else serves the mission. Keep each reason to one sentence.")


def mcp_params(ddir: Path, text_only: bool) -> StdioServerParameters:
    """The MCP server as a stdio subprocess. mcp's stdio client passes only HOME/LOGNAME/PATH/SHELL/TERM/USER to it
    (DEFAULT_INHERITED_ENV_VARS, mcp 2.3.0), so PYTHONPATH is dropped: run it from python/ with this interpreter."""
    return StdioServerParameters(command=sys.executable, cwd=str(Path(__file__).resolve().parents[1]),
                                 args=["-m", "orchestration.mcp_server", str(ddir)] + (["--text-only"] if text_only else []))


class Chooser:
    """What one orchestrator process holds: the decision dir, the MCP clients by kind ("multimodal" | "text"), the
    providers by name, the decision chain, each (provider, kind)'s tool list in that provider's format, and the
    hypothesis layer (None when disabled)."""

    def __init__(self, ddir: Path, mcps: dict, providers: dict, chain: list, cfgs: dict, max_calls: int, hyp=None):
        self.ddir, self.mcps, self.providers, self.chain = ddir, mcps, providers, chain
        self.cfgs, self.max_calls, self.hyp = cfgs, max_calls, hyp

    @staticmethod
    def kind(spec: dict) -> str:
        return "multimodal" if spec.get("images") else "text"


async def decide(req: dict, ch: Chooser) -> dict:
    did = req["decision_id"]
    ddir = ch.ddir
    (ddir / f"{did}.json").write_text(json.dumps(req))
    log = new_log()
    t0 = time.time()
    if ch.hyp is not None:
        try:
            log["hypotheses"] = ch.hyp.before_turn(req)
        except Exception as e:  # noqa: BLE001 - the evidence layer never blocks a decision
            log["errors"].append(f"hypotheses: {type(e).__name__}: {str(e)[:200]}")
    for i, spec in enumerate(ch.chain):
        prov = ch.providers[spec["provider"]]
        kind = ch.kind(spec)
        try:
            await prov.run_turn(req, ddir, ch.mcps[kind], ch.cfgs[(spec["provider"], kind)], spec, SYSTEM,
                                ch.max_calls, log)
            break   # answered (with or without assign): a bad answer is not a transport failure, no fallback
        except BudgetExceeded as e:
            log["errors"].append("budget_exceeded")
            log["budget"] = str(e)[:200]
            break
        except prov.transport_errors as e:
            log["errors"].append(f"{spec['model']}: {type(e).__name__}: {str(e)[:200]}")
            log["fallback_used"] = i + 1 < len(ch.chain)
        except Exception as e:  # noqa: BLE001 - e.g. expired credentials (RuntimeError): answer now, no fallback
            log["errors"].append(f"{spec['model']}: {type(e).__name__}: {str(e)[:200]}")
            break
    log["latency_s"] = round(time.time() - t0, 3)
    choice = ddir / f"{did}.choice.json"
    return {"decision_id": did, "assignments": json.loads(choice.read_text())["assignments"] if choice.exists() else None,
            "orchestrator": log}


LINE_LIMIT = 1 << 24   # bytes per request/answer line; the request grows with the map (16 MiB >> any map seen)


def make_handler(ch: Chooser):
    """One handler per runtime connection. Run 4 (live-20261023T073340Z-e971): the request crossed asyncio's default
    64 KiB line limit at t=55 s; readline raised LimitOverrunError OUTSIDE the per-line guard, the handler died
    unobserved and the runtime segfaulted on the dropped socket. The server now reads with LINE_LIMIT, and a stream
    error is logged, never left to kill the handler silently."""
    lock = asyncio.Lock()

    async def handle(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        pending = set()

        async def one(line: bytes) -> None:
            try:
                ans = await decide(json.loads(line), ch)
            except Exception as e:  # noqa: BLE001 - the runtime must get an answer, not wait out its timeout
                did = json.loads(line).get("decision_id", "?") if line.strip().startswith(b"{") else "?"
                ans = {"decision_id": did, "assignments": None,
                       "orchestrator": {"errors": [f"{type(e).__name__}: {str(e)[:200]}"]}}
            async with lock:
                writer.write((json.dumps(ans) + "\n").encode())
                await writer.drain()

        try:
            while line := await reader.readline():
                t = asyncio.create_task(one(line))
                pending.add(t)
                t.add_done_callback(pending.discard)
        except Exception as e:  # noqa: BLE001
            print(f"orchestrator: request stream error: {type(e).__name__}: {str(e)[:200]}", flush=True)
        await asyncio.gather(*pending)
        writer.close()   # 3.12: Server.wait_closed() waits on every client TRANSPORT; an unclosed writer holds it open

    return handle


async def serve(args) -> None:
    ddir = Path(args.dir)
    ddir.mkdir(parents=True, exist_ok=True)
    cfg = load_config(args.llm_config)
    chain = cfg["decision"]["chain"]
    providers = {n: make_provider(n, cfg) for n in dict.fromkeys(s["provider"] for s in chain)}
    hyp = None
    if cfg.get("hypothesis", {}).get("enabled"):
        from .hypotheses import HypothesisLayer
        hyp = HypothesisLayer(ddir, cfg["hypothesis"])
    async with Client(mcp_params(ddir, False)) as m_mm, Client(mcp_params(ddir, True)) as m_tx:
        mcps = {"multimodal": m_mm, "text": m_tx}
        tools = {k: (await c.list_tools()).tools for k, c in mcps.items()}
        cfgs = {(n, k): p.tool_config(tools[k]) for n, p in providers.items() for k in mcps}
        (ddir / "tools.json").write_text(json.dumps({f"{n}/{k}": v for (n, k), v in cfgs.items()}, indent=1))
        ch = Chooser(ddir, mcps, providers, chain, cfgs, args.max_calls or cfg["decision"].get("max_calls", 10), hyp)
        srv = await asyncio.start_unix_server(make_handler(ch), path=args.socket, limit=LINE_LIMIT)
        print(f"orchestrator: config {cfg['_path']}; chain {[s['provider'] + ':' + s['model'] for s in chain]}; "
              f"hypotheses {'on' if hyp else 'off'}; tools {[t.name for t in tools['multimodal']]}; "
              f"listening on {args.socket}", flush=True)
        async with srv:
            await srv.serve_forever()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--socket", required=True)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--max-calls", type=int, default=0, help="tool-loop turns per decision (0 = the config's)")
    ap.add_argument("--llm-config", default=None, help="configs/llm/*.yaml (default configs/llm/nemotron.yaml)")
    asyncio.run(serve(ap.parse_args()))


if __name__ == "__main__":
    main()
