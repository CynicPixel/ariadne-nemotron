"""F4 matched-evaluation table: one row per live run, grouped by policy, descriptive only (no significance claims).

  python -m orchestration.f4_table RUN_DIR [RUN_DIR ...] --out DIR   -> DIR/f4_table.{md,json}

Policy per run (from the run's own files): B0 = chooser classical; B2 = chooser llm with no hypotheses.jsonl; B4 = chooser
llm with hypotheses.jsonl. Per run, from trace.jsonl / results.yaml / the orchestrator log:
  - mapped free / occupied cells at the last replan (the operator map), and at t = 60 s
  - egress found at rho_responder: first t_s (or "not found")
  - estimated distance flown per agent (the planner's flown_m, from the estimator)
  - agent decisions by choice source; gate validations other than ok, counted as PROPOSALS REJECTED (the executed action
    is then the classical choice), and, separately (invariant 6), unsafe actions EXECUTED: a `go` whose chosen candidate
    is above the risk threshold with no approval on record (supervisor returns exempt), counted from the trace
  - LLM latency p50/p95, calls, tokens, est. USD (decision tier; VL separately)
  - B4 only: hypotheses, statuses, time to first verified resolution, VL hit rate, complete G2 chains
"""
from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

from .evidence_report import answer_of, jl, locate, report


def pct(xs, q):
    if not xs:
        return None
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(q * (len(xs) - 1))))]


def row(run: Path) -> dict:
    tpath = locate(run, "trace.jsonl")
    trace = [r for r in jl(tpath) if "choice" in r]
    if not trace:
        raise SystemExit(f"{run}: no trace.jsonl with choices")
    chooser = trace[0].get("chooser")
    has_h = locate(run, "hypotheses.jsonl").exists()
    policy = "B0" if chooser == "classical" else ("B4" if has_h else "B2")
    last_t = max(r["t_s"] for r in trace)
    last = [r for r in trace if r["t_s"] == last_t]
    at60 = [r for r in trace if r["t_s"] == 60]
    egress = next((r["t_s"] for r in sorted(trace, key=lambda r: r["t_s"]) if r["vision"].get("egress_found")), None)
    src, val = {}, {}
    for r in trace:
        s = r["choice"].get("source", "?")
        src[s] = src.get(s, 0) + 1
        v = r["choice"].get("validation")
        if v and v != "ok":
            val[v] = val.get(v, 0) + 1
    # executed picks above the risk threshold without an approval on record (supervisor returns are exempt, as in the gate)
    unsafe = 0
    for r in trace:
        c = r.get("chosen", -1)
        if c is None or c < 0 or c >= len(r.get("candidates", [])) or r.get("result") != "go":
            continue
        cand = r["candidates"][c]
        if cand["risk"] > r["autonomous_threshold"] and not cand.get("supervisor_return") and not r["choice"].get("approval"):
            unsafe += 1
    orch = [answer_of(r)["orchestrator"] for r in trace if r["agent"] == min(x["agent"] for x in trace)
            and answer_of(r) and answer_of(r).get("orchestrator")]
    lat = [o["latency_s"] for o in orch if o.get("latency_s") is not None]
    out = {"run": run.name, "policy": policy, "end_s": last_t,
           "free_cells_end": last[0]["vision"].get("free_cells"), "occupied_cells_end": last[0]["vision"].get("occupied_cells"),
           "free_cells_60s": at60[0]["vision"].get("free_cells") if at60 else None,
           "egress_found_t_s": egress, "flown_m_end": (last[0].get("planner") or {}).get("flown_m"),
           "decisions_by_source": src, "proposals_rejected_by_gate": val, "unsafe_actions_executed": unsafe,
           "llm_models": sorted({o.get("model") for o in orch if o.get("model")}),
           "llm_calls": sum(o.get("calls") or 0 for o in orch), "llm_latency_p50_s": pct(lat, 0.5),
           "llm_latency_p95_s": pct(lat, 0.95), "llm_input_tokens": sum(o.get("input_tokens") or 0 for o in orch),
           "llm_output_tokens": sum(o.get("output_tokens") or 0 for o in orch),
           "llm_est_usd": round(sum(o.get("est_usd") or 0 for o in orch), 4)}
    if policy != "B0" and not any(m.startswith("nvidia/") for m in out["llm_models"]):
        out["policy"] = policy + "-NOT-NEMOTRON"   # invariant 4: a pre-fork (Bedrock-era) run is never a Nemotron row
    if policy == "B4":
        e = report(run)
        out.update(hypotheses=e["hypotheses"]["total"], final_status=e["hypotheses"]["final_status"],
                   first_verified_resolution_t_s=e["hypotheses"]["first_verified_resolution_t_s"],
                   vl_calls=e["vl"]["by_outcome"], vl_est_usd=e["vl"]["est_usd"], vl_hit_rate=e["vl"]["hit_rate"],
                   chains_complete=e["chains"]["complete"], chains_targeted=e["chains"]["targeted"],
                   chains_with_refutation=e["chains"]["complete_with_refutation"])
    return out


def markdown(rows: list[dict]) -> str:
    cols = ["policy", "run", "end_s", "free_cells_60s", "free_cells_end", "egress_found_t_s", "flown_m_end",
            "decisions_by_source", "proposals_rejected_by_gate", "unsafe_actions_executed", "llm_latency_p50_s", "llm_latency_p95_s",
            "llm_input_tokens", "llm_est_usd", "first_verified_resolution_t_s", "chains_complete", "vl_hit_rate"]
    def f(v):
        if v is None:
            return "-"
        if isinstance(v, dict) and "hits" in v:   # VL hit rate: all terminal, then the predictive subset
            later = v.get("resolved_after_creation", {})
            return f"{v['hits']}/{v['n']} (later {later.get('hits')}/{later.get('n')})"
        return f"{v:.3g}" if isinstance(v, float) else str(v)
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for r in sorted(rows, key=lambda r: (r["policy"], r["run"])):
        lines.append("| " + " | ".join(f(r.get(c)) for c in cols) + " |")
    agg = ["", "Per policy (median over runs; n shown):", ""]
    for p in sorted({r["policy"] for r in rows}):
        rs = [r for r in rows if r["policy"] == p]
        fe = [r["free_cells_end"] for r in rs if r["free_cells_end"] is not None]
        agg.append(f"- {p}: n = {len(rs)}; free cells at end median {statistics.median(fe) if fe else '-'}; egress found "
                   f"in {sum(r['egress_found_t_s'] is not None for r in rs)}/{len(rs)}; proposals rejected by the gate "
                   f"{sum(sum(r['proposals_rejected_by_gate'].values()) for r in rs)}; unsafe actions executed "
                   f"{sum(r['unsafe_actions_executed'] for r in rs)} (above threshold, no approval on record)")
    return "\n".join(lines + agg) + "\n\nDescriptive only: few runs, one scene; no significance claims.\n"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    rows = [row(r) for r in a.runs]
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "f4_table.json").write_text(json.dumps(rows, indent=1))
    (a.out / "f4_table.md").write_text(markdown(rows))
    print(markdown(rows))


if __name__ == "__main__":
    main()
