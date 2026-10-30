# F4 matched evaluation: repetition 1 only (n = 1 per policy) — FP

- **Campaign:** run by FM on 2026-10-29 03:41 IST onward at code `65f212e`, local CPU-only Mesa dev box. Scene `maze_short_6`, 90 sim s.
  Live config `live_llm_v1` (gate 0.5, answer window 120 s), planner `mission_v2`. `runs.jsonl` is FM's campaign manifest,
  one line per run.
- **Stopped:** the planned 3 repetitions stopped after repetition 1, when the machine rebooted. No more runs were made. Every number
  below is from **one run per policy**. It is descriptive only: no ranking, no significance claim.

| policy | what differs | run |
|---|---|---|
| B0 | classical chooser (`live_llm_v1_b0.yaml`: `live_llm_v1` with `chooser: classical`), no LLM | `live-20261028T221048Z-d01b` |
| B2 | Nemotron decision tier (`nvidia/Nemotron-3_5-Lightning` via Nebius Token Factory), hypotheses off (`nemotron_b2.yaml`) | `live-20261028T222153Z-5da7` |
| B4 | B2 + the evidence loop: Super-VL hypotheses, slab verifier, `list_hypotheses` (`nemotron.yaml`) | `live-20261028T223308Z-9e2d` |

**Files:**
- `f4_table.{md,json}`: `python -m orchestration.f4_table`.
- `<run>/evidence_report.{md,json}`: `python -m orchestration.evidence_report`.
- Both were generated at `cd341df` from the run dirs on the dev host.

## Results [VERIFIED from each run's own logs; sim time is primary]

| | B0 | B2 | B4 |
|---|---|---|---|
| mapped free cells at 60 s / 90 s | 17,616 / 18,984 | 17,364 / 19,061 | 18,462 / 22,668 |
| distance flown, agent 0 / 1 (m, estimated) | 7.76 / 8.04 | 9.33 / 7.26 | 8.93 / 8.75 |
| responder egress found (0.40 m) | no | no | no |
| agent decisions by source | 36 classical | 36 Nemotron | 34 Nemotron, 2 fallback |
| unsafe proposals rejected by the gate | 0 | 0 | 0 |
| unsafe actions executed | 0 | 0 | 0 |
| operator approval requests | 0 | 0 | 3 (unanswered: no operator) |
| decision-tier LLM, est. USD | — | 0.072 | 0.154 (+ VL 0.072) |

**B4 evidence loop:**
- 25 hypotheses (12 VL, 13 seed; 2 seed hypotheses VL-annotated). Final: 16 confirmed, 3 refuted, 6 supported.
- First verified resolution at t = 10 s.
- 9 targeted decisions, all 9 complete the chain (hypothesis → Nemotron decision → gate accept → motion → newer slab → status
  change → different next pick). None of them ran through a refutation in this run.
- VL calls: 15 ok, 2 zero_valid. VL hit rate 6/12 over terminal outcomes; 3/5 on hypotheses resolved after creation.

## Caveats (read before quoting any number)

**1. n = 1 per policy.** These are single runs on one scene. Free-running timing and model sampling make repeats differ. The
B4 > B2 ≈ B0 ordering of mapped cells at 90 s is **not** evidence that B4 is better.

**2. Unattended approval waits handicap B4.** B4's Nemotron asked the operator to approve 3 above-threshold picks (t80000ms
agent 1 c13, t85000ms agent 0 c37, t90000ms agent 1 c44). F4 has no operator, so each request blocked the MCP tool's full 60 s
window:
- t80000ms and t85000ms took 74.7 s and 73.0 s, then were answered with a different pick (gate `ok`).
- t90000ms crossed the 120 s answer window, so both agents fell back to classical.

At the run's real-time factor (~0.125), each unanswered request is ≈ 7.5 sim s with the agents on their previous command. B4's
coverage figures carry that handicap. B0 and B2 made no requests. The behaviour comes from `e40e36d`, which tells the model to
request approval before an above-threshold pick instead of sending one the gate would refuse.

**3. Wall-clock latencies are not deployment figures.** Every run is on a CPU-only Mesa dev box at ~0.12–0.13 RTF. B2 p50/p95
was 5.6 / 9.8 s; B4 was 8.0 / 73 s, the p95 being the approval waits.

**4. "Confirmed" / "refuted" mean on the map at that time.** They are decided by the deterministic slab verifier at 0.40 m
responder clearance. Later map revisions can contradict a settled status. The trace records it (`contradiction` events,
`contradicted` in the hypothesis view); the status is never silently flipped.

**5. Counting rules.** Unsafe proposals rejected by the C++ gate are counted apart from unsafe actions executed (invariant 6).
"Executed" is counted from the trace: a `go` whose chosen candidate is above the threshold with no approval on record. A missing
answer (the two t90000ms fallbacks) is neither.

**6. No egress route at ρ_responder** was found within 90 s in any run. Coverage and distance carry the comparison, not egress time.
