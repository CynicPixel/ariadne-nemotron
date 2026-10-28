"""The evidence-loop report for one live run (G2; F4 metrics): joins trace.jsonl, hypotheses.jsonl and verifier.jsonl.

  python -m orchestration.evidence_report RUN_DIR [--out DIR]   -> DIR/evidence_report.{json,md} (default RUN_DIR)

G2 asks for one chain, in order, from the run's own records:
  1. hypothesis      a hypothesis exists (created event), anchored to a planner candidate
  2. decision        the orchestrator's assign names it (hypothesis_id; a `targeted` event) for agent k
  3. gate accept     trace.jsonl for that decision and agent: choice.source "orchestrator", chosen == that candidate
  4. motion          the same trace record: result "go" with a command (the runtime sent the move)
  5. new slab        a verifier.jsonl line for the hypothesis at a LATER decision (a newer slab)
  6. status change   a `status` event for it after the decision (ideally refuted)
  7. different next  agent k's next decision after the status change picks a goal > 0.5 m from the targeted one
Every count is from the logs; nothing is inferred. Metrics (descriptive, per run): decisions by choice source,
hypotheses by provenance and final status, VL calls by outcome, the VL hit rate (terminal outcomes only, with n),
time to the first verified resolution, LLM latency and tokens.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path


def jl(p: Path) -> list[dict]:
    return [json.loads(x) for x in p.read_text().splitlines() if x.strip()] if p.exists() else []


def answer_of(rec: dict) -> dict | None:
    a = (rec.get("choice") or {}).get("answer")
    if isinstance(a, dict):
        return a
    if isinstance(a, str) and a.startswith("{"):
        try:
            return json.loads(a)
        except ValueError:
            return None
    return None


def report(run: Path) -> dict:
    trace = [r for r in jl(run / "trace.jsonl") if "choice" in r]
    hyp = jl(run / "hypotheses.jsonl")
    ver = jl(run / "verifier.jsonl")
    by_dec = {}
    for r in trace:
        by_dec[(r["choice"].get("decision_id"), r["agent"])] = r
    # decisions
    src = {}
    for r in trace:
        s = r["choice"].get("source", "?")
        src[s] = src.get(s, 0) + 1
    orch = [answer_of(r)["orchestrator"] for r in trace if r["agent"] == 0 and answer_of(r)
            and answer_of(r).get("orchestrator")]
    lat = [o["latency_s"] for o in orch if o.get("latency_s") is not None]
    models = sorted({o.get("model") for o in orch if o.get("model")})
    # hypotheses
    final, created_src = {}, {}
    for e in hyp:
        if e.get("hypothesis_id"):
            final[e["hypothesis_id"]] = e
            if e["event"] == "created":
                s = (e.get("provenance") or {}).get("source", "?")
                created_src[s] = created_src.get(s, 0) + 1
    statuses = {}
    for e in final.values():
        statuses[e["status"]] = statuses.get(e["status"], 0) + 1
    vl_calls = [e for e in hyp if e["event"] == "vl_call"]
    vl_out = {}
    for e in vl_calls:
        vl_out[e["outcome"]] = vl_out.get(e["outcome"], 0) + 1
    annotated = {e["hypothesis_id"] for e in hyp if e["event"] == "vl_annotated"}
    predicts = {}
    for e in hyp:
        if e.get("hypothesis_id") and e["event"] in ("created", "vl_annotated"):
            p = (e.get("provenance") or {})
            p = p.get("vl", p) if e["event"] == "vl_annotated" else p
            if p.get("predicts"):
                predicts[e["hypothesis_id"]] = p["predicts"]
    terminal = {h: e["status"] for h, e in final.items() if e["status"] in ("confirmed", "refuted")}
    n = sum(1 for h in predicts if h in terminal)
    hits = sum(1 for h, p in predicts.items() if h in terminal and (p == "passage") == (terminal[h] == "confirmed"))
    first_res = min((e["sim_t_s"] for e in hyp if e["event"] == "status" and e["status"] in ("confirmed", "refuted")),
                    default=None)
    # chains
    chains = []
    for t in (e for e in hyp if e["event"] == "targeted"):
        h, d, k, c = t["hypothesis_id"], t["decision_id"], t["targeted_by_agent"], t["candidate"]
        rec = by_dec.get((d, k))
        ch = {"hypothesis_id": h, "decision_id": d, "t_s": t["sim_t_s"], "agent": k, "candidate": c,
              "evidence_goal": t.get("evidence_goal"), "source": (t.get("provenance") or {}).get("source")}
        ch["gate_accept"] = bool(rec and rec["choice"].get("source") == "orchestrator" and rec.get("chosen") == c)
        ch["motion"] = bool(rec and rec.get("result") == "go" and rec.get("command"))
        later_v = [v for v in ver if v["hypothesis_id"] == h and v["sim_t_s"] > t["sim_t_s"]]
        ch["new_slab"] = later_v[0]["slab"]["path"] if later_v else None
        st = next((e for e in hyp if e["hypothesis_id"] == h and e["event"] == "status" and e["sim_t_s"] > t["sim_t_s"]),
                  None)
        ch["status_change"] = {"t_s": st["sim_t_s"], "status": st["status"], "reason": (st.get("verifier") or {}).get(
            "reason")} if st else None
        ch["different_next"] = None
        if st and rec:
            goal = rec["candidates"][c]["goal_xy_m"] if 0 <= c < len(rec.get("candidates", [])) else None
            nxt = next((r for r in sorted(trace, key=lambda r: r["t_s"]) if r["agent"] == k and r["t_s"] > st["sim_t_s"]
                        and r.get("chosen", -1) >= 0), None)
            if nxt and goal:
                g2 = nxt["candidates"][nxt["chosen"]]["goal_xy_m"]
                ch["different_next"] = {"t_s": nxt["t_s"], "goal_xy_m": g2, "moved_m": round(math.dist(goal, g2), 2),
                                        "source": nxt["choice"].get("source"), "different": math.dist(goal, g2) > 0.5}
        ch["complete"] = bool(ch["gate_accept"] and ch["motion"] and ch["new_slab"] and ch["status_change"]
                              and ch["different_next"] and ch["different_next"]["different"])
        chains.append(ch)
    return {
        "run": str(run), "decisions": {"agent_decisions": len(trace), "by_source": src},
        "llm": {"models": models, "calls": sum(o.get("calls") or 0 for o in orch),
                "latency_p50_s": statistics.median(lat) if lat else None,
                "latency_max_s": max(lat) if lat else None,
                "input_tokens": sum(o.get("input_tokens") or 0 for o in orch),
                "output_tokens": sum(o.get("output_tokens") or 0 for o in orch),
                "est_usd": round(sum(o.get("est_usd") or 0 for o in orch), 5),
                "errors": sorted({e for o in orch for e in (o.get("errors") or [])})[:10]},
        "hypotheses": {"total": len(final), "created_by_source": created_src, "vl_annotated": len(annotated),
                       "final_status": statuses, "first_verified_resolution_t_s": first_res},
        "vl": {"calls": len(vl_calls), "by_outcome": vl_out,
               "est_usd": round(sum((e.get("orchestrator") or {}).get("est_usd") or 0 for e in vl_calls), 5),
               "hit_rate": {"n": n, "hits": hits, "rate": round(hits / n, 3) if n else None,
                            "counts": "confirmed/refuted only; expired/unresolved/supported excluded"}},
        "chains": {"targeted": len(chains), "complete": sum(c["complete"] for c in chains),
                   "complete_with_refutation": sum(c["complete"] and c["status_change"]["status"] == "refuted"
                                                   for c in chains), "items": chains}}


def markdown(r: dict) -> str:
    d, h, v, c, L = r["decisions"], r["hypotheses"], r["vl"], r["chains"], r["llm"]
    out = [f"# Evidence-loop report: `{Path(r['run']).name}`", "",
           f"- Agent decisions: {d['agent_decisions']}, by source {d['by_source']}",
           f"- LLM: {L['models']}; {L['calls']} calls; latency p50 {L['latency_p50_s']} s, max {L['latency_max_s']} s; "
           f"tokens {L['input_tokens']}/{L['output_tokens']}; est. USD {L['est_usd']}",
           f"- Hypotheses: {h['total']} ({h['created_by_source']}; {h['vl_annotated']} VL-annotated); final "
           f"{h['final_status']}; first verified resolution at t = {h['first_verified_resolution_t_s']} s",
           f"- VL calls: {v['calls']} {v['by_outcome']}, est. USD {v['est_usd']}; hit rate {v['hit_rate']['hits']}/"
           f"{v['hit_rate']['n']} ({v['hit_rate']['counts']})",
           f"- G2 chains: {c['targeted']} targeted, {c['complete']} complete, {c['complete_with_refutation']} with a "
           "refutation", "", "| hyp | decision | agent | gate | motion | new slab | status change | next pick | complete |",
           "|---|---|---|---|---|---|---|---|---|"]
    for x in c["items"]:
        sc = x["status_change"]
        dn = x["different_next"]
        out.append(f"| {x['hypothesis_id']} | {x['decision_id']} | {x['agent']} | {x['gate_accept']} | {x['motion']} | "
                   f"{Path(x['new_slab']).name if x['new_slab'] else '-'} | "
                   f"{(str(sc['t_s']) + ' s ' + sc['status']) if sc else '-'} | "
                   f"{(str(dn['t_s']) + ' s, ' + str(dn['moved_m']) + ' m') if dn else '-'} | {x['complete']} |")
    return "\n".join(out) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    r = report(a.run)
    out = a.out or a.run
    out.mkdir(parents=True, exist_ok=True)
    (out / "evidence_report.json").write_text(json.dumps(r, indent=1))
    (out / "evidence_report.md").write_text(markdown(r))
    print(markdown(r))


if __name__ == "__main__":
    main()
