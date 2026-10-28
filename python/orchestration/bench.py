"""Decision-tier bench (D318): replay recorded chooser requests through each candidate model, score every answer with
the C++ gate's rules, and pick the decision-tier model on validity, latency and cost.

  python -m orchestration.bench --corpus evaluation/runs/live-agentic/live_loop/llm_live_run5_maze_short_90s \\
      [--models nvidia/a,nvidia/b] [--n 10] [--llm-config configs/llm/nemotron.yaml] [--dry-run] [--record DIR]

- Corpus: a run dir with decisions/t*ms.json (the requests ariadne_live sent). Only the requests are copied into a
  scratch dir: the archived <did>.choice.json would make run_turn return at once, and `assign` must never overwrite
  an archived run. Image paths are rebased onto the corpus; approvals_dir onto the scratch dir, pre-filled with an
  `approved` verdict for every needs_approval candidate (the approval stub: request_human_approval answers at once).
- Each model runs alone (a one-entry chain, so no fallback hides a failure), through the orchestrator's decide().
- Scoring per agent, as ariadne_live does: candidate in range; above the threshold only with approval_id = the
  expected id (the stub approves); exactly-once by cluster when another eligible cluster exists.
- Agreement with the recorded (Bedrock-era) choice and with the classical choice is reported as REFERENCE ONLY:
  neither is ground truth for a good decision.
- Cost (D312): an estimate is printed before any call; the run is refused if ledger + estimate >= the hard cap.
  --dry-run prints the estimate and stops. Every call is on the spend ledger.
- Output: evaluation/runs/nemotron/bench_<ts>/ (results.jsonl, summary.json, table.md, manifest.yaml).
"""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import shutil
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml
from mcp.client.client import Client

from .orchestrator import Chooser, decide, mcp_params
from .providers import load_config, make_provider
from .providers.nebius import make_ledger

REPO = Path(__file__).resolve().parents[2]
# Per-request token estimate before any measurement: [ASSUMPTION] a decision of ~4 tool turns re-sending a ~5k-token
# context. Replaced by --est-in/--est-out from measured fixture usage once recorded.
EST_IN, EST_OUT = 24000, 1200


def requests_of(corpus: Path, n: int) -> list[dict]:
    """The first n recorded requests, evenly spread over the run (by decision time)."""
    files = sorted((corpus / "decisions").glob("t*ms.json"), key=lambda p: int(p.stem[1:-2]))
    files = [f for f in files if f.stem.endswith("ms")]
    if n < len(files):
        step = len(files) / n
        files = [files[int(i * step)] for i in range(n)]
    return [json.loads(f.read_text()) for f in files]


def rebase(req: dict, corpus: Path, work: Path) -> dict:
    """Paths in an archived request point at the container's /work/build/...; point them at the corpus copy, and
    approvals at the scratch dir with the stub's verdicts."""
    r = json.loads(json.dumps(req))
    dec = corpus / "decisions"
    r["map_png"] = str(dec / Path(r["map_png"]).name)
    for a in r["agents"]:
        if a.get("keyframe_png"):
            a["keyframe_png"] = str(dec / Path(a["keyframe_png"]).name)
    adir = work / "approvals"
    adir.mkdir(parents=True, exist_ok=True)
    r["approvals_dir"] = str(adir)
    for a in r["agents"]:
        for i, c in enumerate(a["candidates"]):
            if c.get("needs_approval"):
                (adir / f"{r['decision_id']}-a{a['agent']}-c{i}.json").write_text(
                    json.dumps({"verdict": "approved", "operator": "bench-stub"}))
    if isinstance(r.get("slab"), dict) and r["slab"].get("path"):
        r["slab"]["path"] = str(corpus / "snapshots" / Path(r["slab"]["path"]).name)
    return r


def score(req: dict, assignments) -> dict:
    """The C++ gate's per-agent verdicts for one answer (ariadne_live.cpp chooser validation, ~L2510-2575)."""
    if not assignments:
        return {"answered": False, "agents": {}}
    thr = req["autonomous_threshold"]
    out, taken = {}, set()
    by = {e["agent"]: e for e in assignments}
    for a in sorted(req["agents"], key=lambda a: a["agent"]):
        k, cands = a["agent"], a["candidates"]
        e = by.get(k)
        c = e.get("candidate", -2) if e else -2
        if c < -1 or c >= len(cands):
            out[k] = "out_of_set"
            continue
        if c >= 0 and cands[c]["risk"] > thr and not cands[c].get("supervisor_return"):
            if e.get("approval_id") != f"{req['decision_id']}-a{k}-c{c}":
                out[k] = "gate_rejected_no_approval"
                continue
        if c >= 0:
            cl = cands[c]["cluster"]
            other = any(x["cluster"] not in taken and x["cluster"] != cl and x["risk"] <= thr for x in cands)
            if cl in taken and other:
                out[k] = "same_cluster"
                continue
            taken.add(cl)
        out[k] = "ok_hold" if c == -1 else "ok"
    return {"answered": True, "agents": out}


def pct(xs: list[float], q: float) -> float | None:
    if not xs:
        return None
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(q * (len(xs) - 1))))]


async def bench_model(cfg: dict, spec: dict, reqs: list[dict], corpus: Path, work: Path, max_calls: int) -> list[dict]:
    ddir = work / "decisions"
    ddir.mkdir(parents=True, exist_ok=True)
    prov = make_provider(spec["provider"], cfg, purpose="bench")
    kind = Chooser.kind(spec)
    rows = []
    async with Client(mcp_params(ddir, kind == "text")) as m:
        tools = (await m.list_tools()).tools
        ch = Chooser(ddir, {kind: m}, {spec["provider"]: prov}, [spec], {(spec["provider"], kind): prov.tool_config(tools)},
                     max_calls)
        for req in reqs:
            r = rebase(req, corpus, work)
            ans = await decide(r, ch)
            o = ans["orchestrator"]
            rec_choice = corpus / "decisions" / f"{req['decision_id']}.choice.json"
            recorded = json.loads(rec_choice.read_text())["assignments"] if rec_choice.exists() else None
            sc = score(r, ans["assignments"])
            pick = {e["agent"]: e["candidate"] for e in (ans["assignments"] or [])}
            rows.append({
                "model": spec["model"], "decision_id": req["decision_id"], "t_s": req["t_s"], "score": sc,
                "assignments": ans["assignments"], "latency_s": o.get("latency_s"), "calls": o.get("calls"),
                "input_tokens": o.get("input_tokens"), "output_tokens": o.get("output_tokens"),
                "reasoning_tokens": o.get("reasoning_tokens"), "est_usd": o.get("est_usd"), "stop": o.get("stop"),
                "errors": o.get("errors"), "tools": [t["tool"] for t in o.get("tool_calls", [])],
                "ref_same_as_classical": {a["agent"]: pick.get(a["agent"]) == a["classical_choice"] for a in r["agents"]}
                if ans["assignments"] else None,
                "ref_same_as_recorded_bedrock": {e["agent"]: pick.get(e["agent"]) == e["candidate"] for e in recorded}
                if (recorded and ans["assignments"]) else None})
            print(f"bench: {spec['model']} {req['decision_id']} {o.get('latency_s')} s calls {o.get('calls')} "
                  f"tok {o.get('input_tokens')}/{o.get('output_tokens')} -> {sc['agents'] or 'no answer'} "
                  f"{o.get('errors') or ''}", flush=True)
    return rows


def summarize(rows: list[dict]) -> dict:
    out = {}
    for m in dict.fromkeys(r["model"] for r in rows):
        rs = [r for r in rows if r["model"] == m]
        verdicts = [v for r in rs for v in r["score"]["agents"].values()]
        lat = [r["latency_s"] for r in rs if r["latency_s"] is not None]
        out[m] = {
            "requests": len(rs), "answered": sum(r["score"]["answered"] for r in rs),
            "agent_decisions": len(verdicts), "valid": sum(v.startswith("ok") for v in verdicts),
            "holds": verdicts.count("ok_hold"), "out_of_set": verdicts.count("out_of_set"),
            "gate_rejected": verdicts.count("gate_rejected_no_approval"), "same_cluster": verdicts.count("same_cluster"),
            "latency_p50_s": pct(lat, 0.5), "latency_p95_s": pct(lat, 0.95),
            "calls_mean": statistics.mean(r["calls"] or 0 for r in rs),
            "input_tokens_mean": statistics.mean(r["input_tokens"] or 0 for r in rs),
            "output_tokens_mean": statistics.mean(r["output_tokens"] or 0 for r in rs),
            "reasoning_tokens_total": sum(r["reasoning_tokens"] or 0 for r in rs),
            "est_usd_total": round(sum(r["est_usd"] or 0 for r in rs), 6),
            "errors": sum(bool(r["errors"]) for r in rs),
            "ref_agree_classical": sum(v for r in rs if r["ref_same_as_classical"] for v in r["ref_same_as_classical"].values()),
            "ref_agree_recorded_bedrock": sum(v for r in rs if r["ref_same_as_recorded_bedrock"]
                                              for v in r["ref_same_as_recorded_bedrock"].values())}
    return out


def table(summary: dict) -> str:
    cols = ["requests", "answered", "agent_decisions", "valid", "holds", "out_of_set", "gate_rejected", "same_cluster",
            "latency_p50_s", "latency_p95_s", "calls_mean", "input_tokens_mean", "output_tokens_mean",
            "reasoning_tokens_total", "est_usd_total", "errors", "ref_agree_classical", "ref_agree_recorded_bedrock"]
    fmt = lambda v: f"{v:.3g}" if isinstance(v, float) else str(v)   # noqa: E731
    lines = ["| model | " + " | ".join(cols) + " |", "|---|" + "---|" * len(cols)]
    lines += [f"| `{m}` | " + " | ".join(fmt(s[c]) for c in cols) + " |" for m, s in summary.items()]
    return "\n".join(lines) + "\n\n`ref_agree_*` count agent decisions equal to the classical / recorded Bedrock-era choice: " \
                              "reference only, not accuracy.\n"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--corpus", required=True, type=Path)
    ap.add_argument("--llm-config", default=None)
    ap.add_argument("--models", default="", help="comma-separated model IDs from the config (default: the decision chain)")
    ap.add_argument("--n", type=int, default=10, help="requests per model (small by default: D312)")
    ap.add_argument("--max-calls", type=int, default=0)
    ap.add_argument("--est-in", type=int, default=EST_IN, help="estimated prompt tokens per request")
    ap.add_argument("--est-out", type=int, default=EST_OUT, help="estimated completion tokens per request")
    ap.add_argument("--dry-run", action="store_true", help="print the cost estimate and stop")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--allow-ultra", action="store_true", help="FM approved the Ultra model for this run")
    args = ap.parse_args()

    cfg = load_config(args.llm_config)
    chain = cfg["decision"]["chain"]
    specs = {s["model"]: s for s in chain}
    if args.models:
        base = chain[0]
        specs = {m: {**base, **specs.get(m, {}), "model": m} for m in args.models.split(",")}
    if any("Ultra" in m for m in specs) and not args.allow_ultra:
        sys.exit("bench: the Ultra model needs FM's approval (--allow-ultra)")
    reqs = requests_of(args.corpus, args.n)
    ledger = make_ledger(cfg)
    est = sum(len(reqs) * ledger.est_usd(m, args.est_in, args.est_out) for m in specs)
    spent = ledger.total()
    print(f"bench: {len(reqs)} requests x {len(specs)} models {list(specs)}; estimate USD {est:.4f} "
          f"(at {args.est_in}/{args.est_out} tokens per request); ledger USD {spent:.4f}; hard cap "
          f"{ledger.usd_hard:.2f}", flush=True)
    if spent + est >= ledger.usd_hard:
        sys.exit("bench: refused, the estimate would cross the hard cap")
    if args.dry_run:
        return
    ts = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    out = args.out or REPO / "evaluation" / "runs" / "nemotron" / f"bench_{ts}"
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    with tempfile.TemporaryDirectory(prefix="ariadne-bench-") as tmp:
        for i, (m, spec) in enumerate(specs.items()):
            rows += asyncio.run(bench_model(cfg, spec, reqs, args.corpus.resolve(), Path(tmp) / f"m{i}",
                                            args.max_calls or cfg["decision"].get("max_calls", 10)))
    summary = summarize(rows)
    (out / "results.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    (out / "table.md").write_text(table(summary))
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True, text=True).stdout.strip()
    dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "python", "configs"], cwd=REPO,
                                capture_output=True, text=True).stdout.strip())
    cfg_path = Path(cfg["_path"])
    man = {"tool": "orchestration.bench", "ts_utc": ts, "git_commit": commit, "git_dirty_python_configs": dirty,
           "llm_config": str(cfg_path.relative_to(REPO) if cfg_path.is_relative_to(REPO) else cfg_path),
           "llm_config_sha256": hashlib.sha256(cfg_path.read_bytes()).hexdigest(),
           "provider_base_url": cfg.get("nebius", {}).get("base_url"),
           "models": {m: {k: v for k, v in s.items()} for m, s in specs.items()},
           "corpus": str(args.corpus), "corpus_lineage": "requests recorded by ariadne_live (Bedrock-era run); only "
           "the requests are replayed, the recorded answers are reference only",
           "decision_ids": [r["decision_id"] for r in reqs], "approval_stub": "approved for every needs_approval",
           "ledger_usd_before": round(spent, 6), "ledger_usd_after": round(ledger.total(), 6),
           "estimate_usd": round(est, 6), "python": sys.version.split()[0]}
    (out / "manifest.yaml").write_text(yaml.safe_dump(man, sort_keys=False))
    shutil.copy(cfg_path, out / "llm_config.yaml")
    print(table(summary))
    print(f"bench: wrote {out}")


if __name__ == "__main__":
    main()
